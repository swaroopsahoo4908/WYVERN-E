#!/usr/bin/env python3
"""WYVERN-E 4.0 ground-test-rig math derivations: reproduces every number in MATH_DERIVATIONS.md.

  1. Load-cell 3x3 cross-axis calibration matrix (both TVC-balance rigs) -- least-squares pinv fit.
     This resolves the full thrust vector (and hence gimbal deflection) directly from the balance.

(The former ToF-ring plane-fit and Kalman-fusion derivations were removed with the ToF sensors.)

Run: python3 derive_math.py   -- prints every intermediate result and writes
phase0_math_constants.json (machine-readable).
"""
import json
import numpy as np

np.set_printoptions(precision=6, suppress=True)
g = 9.80665

# ============================================================================
# 1. LOAD-CELL CALIBRATION MATRIX
# ============================================================================
print("="*70, "\n1. LOAD-CELL CALIBRATION MATRIX\n", "="*70)

S_true = np.array([
    [8500.0, 120.0, -80.0],
    [100.0, 8500.0, 60.0],
    [-50.0, 40.0, 42000.0],
])
c0_true = np.array([524288.0, 524288.0, 524288.0])

def counts_from_force(F, S, c0, noise_sigma=None, rng=None):
    c = c0 + S @ F
    if noise_sigma is not None:
        c = c + rng.normal(0, noise_sigma, size=3)
    return c

rng = np.random.default_rng(7)
noise_sigma = 15.0
F_cal = np.array([
    [0.0, 0.0, 0.0],
    [1.0*g, 0.0, 0.0], [2.0*g, 0.0, 0.0],
    [0.0, 1.0*g, 0.0], [0.0, 2.0*g, 0.0],
    [0.0, 0.0, 2.0*g], [0.0, 0.0, 4.0*g],
    [0.5*g, 0.5*g, 2.0*g], [-0.5*g, 0.7*g, 3.0*g],
])
C_cal = np.array([counts_from_force(F, S_true, c0_true, noise_sigma, rng) for F in F_cal])
A_design = np.column_stack([F_cal, np.ones(len(F_cal))])
A_pinv = np.linalg.pinv(A_design)
params = A_pinv @ C_cal
S_fit = params[:3, :].T
c0_fit = params[3, :]
S_inv = np.linalg.inv(S_fit)

print("S_fit (counts/N):\n", S_fit)
print("cond(S_fit) =", np.linalg.cond(S_fit))
print("eig(S_fit)  =", np.linalg.eigvals(S_fit))
print("max abs err vs S_true:", np.max(np.abs(S_fit - S_true)))

F_test_true = np.array([0.8*g, -0.6*g, 20.0])
c_test = counts_from_force(F_test_true, S_true, c0_true, noise_sigma, rng)
F_test_hat = S_inv @ (c_test - c0_fit)

def thrust_vector(F):
    Fx, Fy, Fz = F
    T = np.sqrt(Fx**2 + Fy**2 + Fz**2)
    theta = np.arctan2(np.sqrt(Fx**2 + Fy**2), Fz)
    phi = np.arctan2(Fy, Fx)
    return T, theta, phi

T_hat, th_hat, ph_hat = thrust_vector(F_test_hat)
print(f"Test point F_hat={F_test_hat}, T={T_hat:.3f}N theta={np.rad2deg(th_hat):.3f}deg phi={np.rad2deg(ph_hat):.3f}deg")

# ============================================================================
# Save machine-readable constants
# ============================================================================
out = {
    "load_cell_calibration": {
        "S_fit_counts_per_N": S_fit.tolist(), "c0_fit_counts": c0_fit.tolist(),
        "S_inv_N_per_count": S_inv.tolist(), "cond_S_fit": float(np.linalg.cond(S_fit)),
    },
}
with open("phase0_math_constants.json", "w") as f:
    json.dump(out, f, indent=2)
print("\nWrote phase0_math_constants.json")
