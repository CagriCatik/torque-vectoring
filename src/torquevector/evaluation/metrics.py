"""Tracking metrics of a simulated case: yaw-rate and cross-track RMSE, utilisation, speed and torque figures."""

from __future__ import annotations
import numpy as np


def compute_case(spec, sample):
    yaw_err = []
    cross = []
    beta = []
    max_tire = []
    drive = []
    for s in sample:
        state = s.state
        reference = spec.reference_at(state)
        yaw_des = state.vx_ms * reference[0].curvature_inv_m if reference else 0.0
        yaw_err.append(state.yaw_rate_rads - yaw_des)
        if reference:
            cross.append(reference[1])
        beta.append(np.arctan2(state.vy_ms, state.vx_ms))
        max_tire.append(max(w.utilization for w in s.forces.wheels.values()))
        drive.append(s.drive_nm)
    d = np.asarray(drive)
    return {
        'yaw_rmse_rads': float(np.sqrt(np.mean(np.square(yaw_err)))),
        'cross_rmse_m': float(np.sqrt(np.mean(np.square(cross)))) if cross else None,
        'max_abs_beta_deg': float(np.rad2deg(np.max(np.abs(beta)))),
        'max_tyre_utilization': float(max(max_tire)),
        'diff_torque_rms_nm': float(np.sqrt(np.mean((d[:, 3] - d[:, 2]) ** 2))),
        'final_speed_ms': float(sample[-1].state.vx_ms),
    }
