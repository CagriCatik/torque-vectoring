"""Base torque distribution and yaw reference construction for the sampled controllers."""

from __future__ import annotations
from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import smoothstep
from torquevector.domain.controls import YawReference
from torquevector.control.references import steering_reference


def policy_4wd(base, distribution: tuple[float, float, float, float] = (0.2, 0.2, 0.3, 0.3)):
    def fun(t, s, c):
        drive = base(t, s, c)
        total = sum(drive.drive_nm)
        torques = tuple(share * total for share in distribution)
        return TorqueControls(drive.steer_deg, torques, drive.brake_nm, drive.torque_limit_nm)

    return fun


def make_reference(spec, car, mode="path", engage_ramp_s=0.55):
    if mode == "steering":
        return steering_reference(car)
    if mode != "path":
        raise ValueError("Unknown reference mode")
    fallback = steering_reference(car)

    def target(t, s, drive):
        if spec.reference_at(s) is None:
            return fallback(t, s, drive)
        ref, _, _ = spec.reference_at(s)
        # Same gentle 0.55 s target introduction as road-wheel steering ramp.
        engage = smoothstep(t / engage_ramp_s)
        return YawReference(s.vx_ms * ref.curvature_inv_m * engage)

    return target
