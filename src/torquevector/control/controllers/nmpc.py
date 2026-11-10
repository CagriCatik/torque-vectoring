"""Nonlinear MPC (reduced-order single-track model with tanh axle saturation, RK4, CasADi/IPOPT) for the additional yaw moment."""

from __future__ import annotations
from dataclasses import dataclass
from types import SimpleNamespace
from math import isfinite
from math import atan2
import numpy as np
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.prediction.interfaces import PredictionPreview
from torquevector.prediction.nonlinear_bicycle import ReducedOrderTyre
from torquevector.prediction.nonlinear_bicycle import NonlinearBicyclePrediction
from torquevector.prediction.nonlinear_bicycle import tyre_from_nominal
from torquevector.control.sampling import SampledYawMomentPolicy


@dataclass(frozen=True)
class NMPCConfig:
    sample_s: float = 0.04
    horizon_steps: int = 10
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
    max_iterations: int = 35
    tolerance: float = 1e-5

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
            self.tolerance,
        )
        if (
            not all(isfinite(v) for v in vals)
            or not 0.02 <= self.sample_s <= 0.2
            or not isinstance(self.horizon_steps, int)
            or not 2 <= self.horizon_steps <= 40
            or not 0.005 <= self.beta_scale_rad <= 0.5
            or not 0.005 <= self.yaw_scale_rads <= 0.6
            or not 100 <= self.moment_scale_nm <= 10000
            or not 100 <= self.moment_limit_nm <= 3000
            or not 100 <= self.moment_slew_nm_s <= 50000
            or not 0 < self.input_weight <= 100
            or not 0 <= self.slew_weight <= 100
            or not 0.1 <= self.terminal_weight <= 10
            or not 0 <= self.state_soft_weight <= 10000
            or not 0.02 <= self.beta_limit_rad <= 0.5
            or not 0.1 <= self.mu_reference <= 1.5
            or not 0.2 <= self.yaw_limit_fraction <= 1.5
            or not 3 <= self.min_speed_ms < self.max_speed_ms <= 80
            or not isinstance(self.max_iterations, int)
            or not 5 <= self.max_iterations <= 250
            or not 1e-9 <= self.tolerance <= 1e-2
        ):
            raise ValueError('Invalid NMPC configuration')


@dataclass(frozen=True)
class NMPCSolverSettings:
    """CasADi/IPOPT-Optionen; acceptable_tol = max(floor, factor*tolerance)."""

    print_level: int = 0
    print_time: bool = False
    acceptable_tol_floor: float = 1e-4
    acceptable_tol_factor: float = 10.0
    suppress_banner: bool = True
    max_cpu_time_s: float | None = None

    def __post_init__(self):
        if (
            not isinstance(self.print_level, int)
            or isinstance(self.print_level, bool)
            or not 0 <= self.print_level <= 12
            or not isinstance(self.print_time, bool)
            or not isinstance(self.suppress_banner, bool)
            or not (isfinite(self.acceptable_tol_floor) and 0 < self.acceptable_tol_floor <= 1)
            or not (isfinite(self.acceptable_tol_factor) and 1 <= self.acceptable_tol_factor <= 1e6)
            or (self.max_cpu_time_s is not None and not (isfinite(self.max_cpu_time_s) and self.max_cpu_time_s > 0))
        ):
            raise ValueError('Invalid NMPC solver settings')

    def ipopt_options(self, config: NMPCConfig) -> dict:
        options = {
            'ipopt.print_level': self.print_level,
            'print_time': self.print_time,
            'ipopt.max_iter': config.max_iterations,
            'ipopt.tol': config.tolerance,
            'ipopt.acceptable_tol': max(self.acceptable_tol_floor, self.acceptable_tol_factor * config.tolerance),
            'ipopt.sb': 'yes' if self.suppress_banner else 'no',
        }
        if self.max_cpu_time_s is not None:
            options['ipopt.max_cpu_time'] = self.max_cpu_time_s
        return options


@dataclass(frozen=True)
class NMPCResult:
    command_nm: float
    planned_nm: tuple[float, ...]
    predicted_beta_rad: tuple[float, ...]
    predicted_yaw_rads: tuple[float, ...]
    max_state_excess: float
    iterations: int
    objective: float


class NMPC:
    def __init__(
        self,
        car: VehicleParameters,
        tyre: TyreParameters,
        config: NMPCConfig = NMPCConfig(),
        tyre_properties: ReducedOrderTyre | None = None,
        solver_settings: NMPCSolverSettings | None = None,
        friction_preview: bool = False,
    ):
        """``friction_preview``: Reibwertverhältnis je Stufe und Achse als zusätzliche NLP-Parameter
        (Modus map_preview). Die tanh-Achskraft skaliert dann mit r_k (Steifigkeit und Grenze ∝ mu),
        die weiche Gierratengrenze mit min(r_vorne, r_hinten). Ohne Flag unverändertes NLP."""
        try:
            import casadi as ca
        except ImportError as exc:
            raise RuntimeError('NMPC requires casadi: pip install casadi') from exc
        self.car = car
        self.config = config
        self.tyre_properties = tyre_properties or tyre_from_nominal(car, tyre, config.mu_reference)
        self._last_solution: np.ndarray | None = None
        self._ca = ca
        predictor = NonlinearBicyclePrediction(car, self.tyre_properties, ca)
        n = config.horizon_steps
        u = ca.MX.sym('u', n)
        # Parameters: initial beta, yaw, previous moment [kNm],
        # stage speeds, steering, desired beta, desired yaw.
        self.friction_preview = friction_preview
        p = ca.MX.sym('p', 3 + (6 if friction_preview else 4) * n)
        x = p[:2]
        prev = p[2]
        speed = p[3 : 3 + n]
        steer = p[3 + n : 3 + 2 * n]
        betaref = p[3 + 2 * n : 3 + 3 * n]
        yawref = p[3 + 3 * n : 3 + 4 * n]
        cost = 0
        states = []
        rate = []
        tyre0 = self.tyre_properties
        for k in range(n):
            ratio = 1.0
            if friction_preview:
                rf, rr = p[3 + 4 * n + k], p[3 + 5 * n + k]
                scaled = SimpleNamespace(
                    cf_n_rad=tyre0.cf_n_rad * rf,
                    cr_n_rad=tyre0.cr_n_rad * rr,
                    front_limit_n=tyre0.front_limit_n * rf,
                    rear_limit_n=tyre0.rear_limit_n * rr,
                )
                predictor = NonlinearBicyclePrediction(car, scaled, ca)
                ratio = ca.fmin(rf, rr)
            x = predictor.step(x, u[k], speed[k], steer[k], config.sample_s)
            states.append(x)
            e_beta = (x[0] - betaref[k]) / config.beta_scale_rad
            e_yaw = (x[1] - yawref[k]) / config.yaw_scale_rads
            stage_weight = config.terminal_weight if k == n - 1 else 1.0
            cost += stage_weight * (e_beta**2 + e_yaw**2)
            cost += config.input_weight * (1000 * u[k] / config.moment_scale_nm) ** 2
            prior = prev if k == 0 else u[k - 1]
            cost += config.slew_weight * (1000 * (u[k] - prior) / config.moment_scale_nm) ** 2
            rate.append(u[k] - prior)
            yaw_lim = config.mu_reference * ratio * config.yaw_limit_fraction * car.gravity / speed[k]
            beta_excess = ca.fmax(ca.fabs(x[0]) - config.beta_limit_rad, 0) / config.beta_scale_rad
            yaw_excess = ca.fmax(ca.fabs(x[1]) - yaw_lim, 0) / config.yaw_scale_rads
            cost += config.state_soft_weight * (beta_excess**2 + yaw_excess**2)
        x_flat = ca.vertcat(*states)
        nlp = {'x': u, 'f': cost, 'g': ca.vertcat(*rate), 'p': p}
        self.solver_settings = solver_settings or NMPCSolverSettings()
        settings = self.solver_settings.ipopt_options(config)
        self._solver = ca.nlpsol('nmpc_solver', 'ipopt', nlp, settings)
        self._predict = ca.Function('nmpc_predict', [u, p], [x_flat])

    def solve(
        self, state: VehicleState, preview: PredictionPreview, previous_nm: float, period_s: float | None = None
    ) -> NMPCResult:
        cfg = self.config
        n = cfg.horizon_steps
        preview.validate(n)
        if period_s is not None and abs(period_s - cfg.sample_s) > 1e-9:
            raise ValueError('NMPC period and sample period disagree')
        if not cfg.min_speed_ms <= state.vx_ms <= cfg.max_speed_ms:
            raise ValueError('State speed outside NMPC domain')
        if not isfinite(previous_nm) or abs(previous_nm) > cfg.moment_limit_nm + 1e-6:
            raise ValueError('Previous NMPC command outside limits')
        speeds = np.asarray(preview.speed_ms, float)
        if np.any(speeds < cfg.min_speed_ms) or np.any(speeds > cfg.max_speed_ms):
            raise ValueError('Predicted speed outside NMPC domain; no silent clipping')
        beta0 = atan2(state.vy_ms, state.vx_ms)
        pars = np.r_[
            beta0,
            state.yaw_rate_rads,
            previous_nm / 1000,
            speeds,
            preview.steering_rad,
            preview.desired_beta_rad,
            preview.desired_yaw_rads,
        ]
        if self.friction_preview:
            if preview.front_friction_ratio is None:
                raise ValueError('NMPC map_preview benötigt eine Reibwertvorschau im Preview')
            pars = np.r_[pars, preview.front_friction_ratio, preview.rear_friction_ratio]
        if not np.all(np.isfinite(pars)):
            raise ValueError('NMPC received nonfinite parameters')
        max_knm = cfg.moment_limit_nm / 1000
        slew_knm = cfg.moment_slew_nm_s * cfg.sample_s / 1000
        if self._last_solution is None:
            guess = np.full(n, previous_nm / 1000)
        else:
            guess = np.r_[self._last_solution[1:], self._last_solution[-1]]
        # Project the warm-start sequence to guaranteed input feasibility.
        last = previous_nm / 1000
        for i in range(n):
            guess[i] = np.clip(guess[i], max(-max_knm, last - slew_knm), min(max_knm, last + slew_knm))
            last = guess[i]
        sol = self._solver(
            x0=guess,
            p=pars,
            lbx=np.full(n, -max_knm),
            ubx=np.full(n, max_knm),
            lbg=np.full(n, -slew_knm),
            ubg=np.full(n, slew_knm),
        )
        info = self._solver.stats()
        if not info.get('success', False):
            raise RuntimeError(f'NMPC failed closed: {info.get("return_status", "unknown")}')
        candidate = np.asarray(sol['x'], float).reshape(-1)
        differences = np.diff(np.r_[previous_nm / 1000, candidate])
        if (
            not np.all(np.isfinite(candidate))
            or np.max(np.abs(candidate)) > max_knm + 1e-5
            or np.max(np.abs(differences)) > slew_knm + 1e-5
        ):
            raise RuntimeError('NMPC violated hard input limits')
        pred = np.asarray(self._predict(candidate, pars), float).reshape(n, 2)
        if not np.all(np.isfinite(pred)):
            raise RuntimeError('NMPC predicted nonfinite state')
        ratio = np.minimum(preview.front_friction_ratio, preview.rear_friction_ratio) if self.friction_preview else 1.0
        limit_yaw = cfg.mu_reference * ratio * cfg.yaw_limit_fraction * self.car.gravity / speeds
        excess = np.maximum(np.abs(pred[:, 0]) - cfg.beta_limit_rad, 0)
        excess = max(float(np.max(excess)), float(np.max(np.maximum(np.abs(pred[:, 1]) - limit_yaw, 0))))
        self._last_solution = candidate.copy()
        return NMPCResult(
            float(candidate[0] * 1000),
            tuple(map(float, 1000 * candidate)),
            tuple(map(float, pred[:, 0])),
            tuple(map(float, pred[:, 1])),
            excess,
            int(info.get('iter_count', 0)),
            float(sol['f']),
        )


class SampledNMPCPolicy(SampledYawMomentPolicy):
    """Same command and allocator interface as LTV-MPC; new nonlinear solver."""

    pass
