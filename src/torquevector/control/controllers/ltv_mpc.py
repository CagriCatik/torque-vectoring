"""Linear time-varying MPC for the additional yaw moment with hard input/slew limits and soft state limits (SLSQP)."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from math import atan2
import numpy as np
from scipy.optimize import Bounds
from scipy.optimize import LinearConstraint
from scipy.optimize import minimize
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.prediction.interfaces import PredictionLinearizer
from torquevector.prediction.interfaces import PredictionPreview
from torquevector.prediction.ltv import lqr_terminal
from torquevector.prediction.ltv import prediction_matrices
from torquevector.control.sampling import SampledYawMomentPolicy


@dataclass(frozen=True)
class LTVConfig:
    sample_s: float = 0.04
    horizon_steps: int = 16
    beta_scale_rad: float = 0.06
    yaw_scale_rads: float = 0.07
    moment_scale_nm: float = 1100.0
    moment_limit_nm: float = 900.0
    moment_slew_nm_s: float = 6000.0
    input_weight: float = 0.12
    slew_weight: float = 0.06
    terminal_weight: float = 1.0
    state_soft_weight: float = 50.0
    beta_limit_rad: float = 0.17
    mu_reference: float = 0.9
    yaw_limit_fraction: float = 0.9
    min_speed_ms: float = 5.0
    max_speed_ms: float = 40.0
    max_iterations: int = 75

    def __post_init__(self):
        vals = (
            self.sample_s,
            self.beta_scale_rad,
            self.yaw_scale_rads,
            self.moment_scale_nm,
            self.moment_limit_nm,
            self.moment_slew_nm_s,
            self.input_weight,
            self.slew_weight,
            self.terminal_weight,
            self.state_soft_weight,
            self.beta_limit_rad,
            self.mu_reference,
            self.yaw_limit_fraction,
            self.min_speed_ms,
            self.max_speed_ms,
        )
        if (
            not all(isfinite(float(x)) for x in vals)
            or not 0.02 <= self.sample_s <= 0.2
            or not 2 <= self.horizon_steps <= 70
            or not 0.005 <= self.beta_scale_rad <= 0.5
            or not 0.005 <= self.yaw_scale_rads <= 0.6
            or not 100 <= self.moment_scale_nm <= 10000
            or not 100 <= self.moment_limit_nm <= 3000
            or not 100 <= self.moment_slew_nm_s <= 50000
            or not 0.00001 <= self.input_weight <= 100
            or not 0 <= self.slew_weight <= 100
            or not 0.1 <= self.terminal_weight <= 10
            or not 0 <= self.state_soft_weight <= 10000
            or not 0.02 <= self.beta_limit_rad <= 0.5
            or not 0.1 <= self.mu_reference <= 1.5
            or not 0.2 <= self.yaw_limit_fraction <= 1.5
            or not 3 <= self.min_speed_ms < self.max_speed_ms <= 80
            or not isinstance(self.max_iterations, int)
            or not 10 <= self.max_iterations <= 500
        ):
            raise ValueError('Invalid LTV MPC configuration')


@dataclass(frozen=True)
class LTVResult:
    command_nm: float
    planned_nm: tuple[float, ...]
    predicted_beta_rad: tuple[float, ...]
    predicted_yaw_rads: tuple[float, ...]
    max_state_excess: float
    iterations: int
    objective: float


class LTVMPC:
    def __init__(self, linearizer: PredictionLinearizer, car: VehicleParameters, config: LTVConfig = LTVConfig()):
        self.linearizer = linearizer
        self.car = car
        self.config = config

    def solve(
        self, state: VehicleState, preview: PredictionPreview, previous_nm: float, period_s: float | None = None
    ) -> LTVResult:
        cfg = self.config
        n = cfg.horizon_steps
        preview.validate(n)
        if period_s is not None and abs(period_s - cfg.sample_s) > 1e-9:
            raise ValueError('Control sample and prediction step disagree')
        if not isfinite(previous_nm) or abs(previous_nm) > cfg.moment_limit_nm + 1e-6:
            raise ValueError('Previous command outside actuator bounds')
        if not cfg.min_speed_ms <= state.vx_ms <= cfg.max_speed_ms:
            raise ValueError('State outside valid linear-model speed range')
        speeds = np.clip(np.asarray(preview.speed_ms), cfg.min_speed_ms, cfg.max_speed_ms)
        models = [self.linearizer.discrete(float(v), cfg.sample_s) for v in speeds]
        steering = np.asarray(preview.steering_rad)
        fx, fu, dist = prediction_matrices(models, steering)
        x = np.array([atan2(state.vy_ms, state.vx_ms), state.yaw_rate_rads])
        affine = fx @ x + dist
        target = np.column_stack((preview.desired_beta_rad, preview.desired_yaw_rads)).ravel()
        scales = np.tile([cfg.beta_scale_rad, cfg.yaw_scale_rads], n)
        weight = 1 / scales
        q = np.diag([1 / cfg.beta_scale_rad**2, 1 / cfg.yaw_scale_rads**2])
        r = cfg.input_weight / (cfg.moment_scale_nm / 1000.0) ** 2
        terminal = lqr_terminal(models[-1], q, r) * cfg.terminal_weight
        # For the LQR-equivalence invariant, stages 1..N-1 have Q, stage N has P.
        state_weight = np.tile([1 / cfg.beta_scale_rad**2, 1 / cfg.yaw_scale_rads**2], n)
        state_weight[-2:] = 0
        h_state = fu.T @ (state_weight[:, None] * fu)
        g_state = fu.T @ (state_weight * (affine - target))
        u_terminal = fu[-2:]
        residual_terminal = affine[-2:] - target[-2:]
        h_state += u_terminal.T @ terminal @ u_terminal
        g_state += u_terminal.T @ terminal @ residual_terminal
        diff = np.eye(n)
        diff[1:, :] -= np.eye(n, k=-1)[1:, :]
        old = previous_nm / 1000.0
        d_target = np.r_[old, np.zeros(n - 1)]
        h = h_state + np.eye(n) * r
        g = g_state.copy()
        if cfg.slew_weight:
            rs = cfg.slew_weight / (cfg.moment_scale_nm / 1000.0) ** 2
            h += rs * (diff.T @ diff)
            g -= rs * diff.T @ d_target
        h = (h + h.T) / 2
        max_knm = cfg.moment_limit_nm / 1000.0
        rate_knm = cfg.moment_slew_nm_s * cfg.sample_s / 1000.0
        lower = np.full(n, -rate_knm)
        upper = np.full(n, rate_knm)
        lower[0] += old
        upper[0] += old
        constraints = [LinearConstraint(diff, lower, upper)]
        lim = np.column_stack(
            (np.full(n, cfg.beta_limit_rad), cfg.mu_reference * cfg.yaw_limit_fraction * self.car.gravity / speeds)
        ).ravel()
        # State envelopes are SOFT to avoid infeasible QPs with uncontrollable
        # initial conditions; violations are reported. Actuator limits remain HARD.
        scaled_fu = weight[:, None] * fu
        scaled_affine = weight * affine
        scaled_lim = weight * lim
        soft = cfg.state_soft_weight

        def fg(u):
            val = 0.5 * u @ h @ u + g @ u
            grad = h @ u + g
            if soft:
                z = scaled_affine + scaled_fu @ u
                excess = np.maximum(np.abs(z) - scaled_lim, 0.0)
                val += 0.5 * soft * excess @ excess
                grad += soft * scaled_fu.T @ (np.sign(z) * excess)
            return float(val), grad

        start = np.full(n, old)
        sol = minimize(
            lambda u: fg(u)[0],
            start,
            jac=lambda u: fg(u)[1],
            bounds=Bounds(np.full(n, -max_knm), np.full(n, max_knm)),
            constraints=constraints,
            method='SLSQP',
            options={'ftol': 1e-8, 'maxiter': cfg.max_iterations, 'disp': False},
        )
        u = np.asarray(sol.x, float)
        violation = max(
            float(np.max(np.abs(u) - max_knm)), float(np.max(lower - diff @ u)), float(np.max(diff @ u - upper))
        )
        if not sol.success or not np.all(np.isfinite(u)) or violation > 1e-5:
            raise RuntimeError(f'LTV optimizer failed: {sol.message}; violation={violation:.4g}')
        pred = affine + fu @ u
        excess = np.maximum(np.abs(pred) - lim, 0.0)
        return LTVResult(
            float(u[0] * 1000),
            tuple(map(float, u * 1000)),
            tuple(map(float, pred[::2])),
            tuple(map(float, pred[1::2])),
            float(max(excess)),
            int(sol.nit),
            float(fg(u)[0]),
        )


class SampledLTVPolicy(SampledYawMomentPolicy):
    """LTV compatibility adapter over the neutral sampled policy."""
