"""Condensed LTV prediction matrices and LQR terminal cost for the LTV-MPC."""

from __future__ import annotations
import numpy as np
from torquevector.prediction.interfaces import DiscreteModel
from scipy.linalg import solve_discrete_are


def prediction_matrices(models: list[DiscreteModel], steering: np.ndarray):
    """Predicted x_(1..N)=Fx x0+Fu u + disturbance, u in kNm."""
    n = len(models)
    if steering.shape != (n,):
        raise ValueError('steering horizon mismatch')
    fx = np.zeros((2 * n, 2))
    fu = np.zeros((2 * n, n))
    dist = np.zeros(2 * n)
    aa = np.eye(2)
    bb = np.zeros((2, n))
    cc = np.zeros(2)
    for k, m in enumerate(models):
        aa = m.a @ aa
        bb = m.a @ bb
        bb[:, k] += m.b
        cc = m.a @ cc + m.steering * steering[k]
        fx[2 * k : 2 * k + 2] = aa
        fu[2 * k : 2 * k + 2] = bb
        dist[2 * k : 2 * k + 2] = cc
    return fx, fu, dist


def lqr_terminal(model: DiscreteModel, q: np.ndarray, r: float):
    return solve_discrete_are(model.a, model.b[:, None], q, np.array([[r]]))
