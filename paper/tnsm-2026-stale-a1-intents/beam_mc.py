#!/usr/bin/env python3
"""Independent Monte Carlo of the beam process of Section III (no ns-3): the best beam walks on a
ring of B = 16 beams with jumps to either neighbor at total rate 1/T_b (wrap-around included,
exact continuous time); an action measured at t_k is applied at t_k + ell and held until the next
one; the pick is right with probability a, otherwise one neighbor. Compares the misalignment with
Proposition 1 (eq. lambda, exact Bessel form) and the break-even gap with Corollary 1.
Writes macros_beam_mc.tex. The grid step is 1 ms, well below the loop times, so that it does not
bias the age difference between hosts.
"""
import os
import numpy as np
from scipy.special import ive
from scipy import integrate
from scipy.optimize import brentq

HERE = os.path.dirname(os.path.abspath(__file__))
B, P = 16, 0.1


def mc_lambda(a, T, ell, horizon=1500.0, dt=0.001, seed=1):
    rng = np.random.default_rng(seed)
    t = np.arange(0, horizon, dt)
    # exact jump process sampled on the grid: number of jumps in each dt is Poisson(dt/T)
    n = rng.poisson(dt / T, len(t))
    steps = np.zeros(len(t), int)
    for i in np.nonzero(n)[0]:
        steps[i] = rng.choice([-1, 1], n[i]).sum()
    best = np.cumsum(steps) % B
    k_meas = np.arange(0, horizon, P)
    meas_idx = np.minimum((k_meas / dt).astype(int), len(t) - 1)
    right = rng.random(len(k_meas)) < a
    side = np.where(rng.random(len(k_meas)) < 0.5, 1, -1)
    pick = np.where(right, best[meas_idx], (best[meas_idx] + side) % B)
    act = k_meas + ell
    idx = np.searchsorted(act, t, side="right") - 1
    applied = np.where(idx >= 0, pick[np.clip(idx, 0, None)], -1)
    return float(np.mean(applied != best))


def an_lambda(a, T, ell):
    phi = lambda A: a * ive(0, A / T) + (1 - a) * ive(1, A / T)
    return 1 - integrate.quad(phi, ell, ell + P)[0] / P


if __name__ == "__main__":
    import json
    _ell = json.load(open(os.path.join(HERE, "beam_model.json")))["ell"]
    # the 550 km, one-gateway loops of the campaign
    a, ell_g, ell_o = 0.9, _ell["ground_s550_gw1"], _ell["onboard-ttl_s550_gw1"]
    rows, cross = [], []
    for T in (0.5, 1, 2, 5):
        for ell in (ell_o, ell_g):
            m = np.mean([mc_lambda(a, T, ell, seed=s) for s in range(4)])
            rows.append((T, ell, m, an_lambda(a, T, ell)))
        # break-even: on-board (a - d, ell_o) equals ground (a, ell_g), Monte Carlo with common seeds
        g = np.mean([mc_lambda(a, T, ell_g, seed=s) for s in range(8)])
        f = lambda d: np.mean([mc_lambda(a - d, T, ell_o, seed=s) for s in range(8)]) - g
        dmc = brentq(f, -0.02, 0.1, xtol=1e-4)
        dex = brentq(lambda d: an_lambda(a - d, T, ell_o) - an_lambda(a, T, ell_g), -0.02, 0.1)
        cross.append((T, 100 * dmc, 100 * dex, 100 * (3 * a - 1) * (ell_g - ell_o) / (2 * T)))
        print(T, rows[-2:], cross[-1], flush=True)
    err = max(abs(r[2] - r[3]) for r in rows) * 100
    cerr = max(abs(c[1] - c[2]) for c in cross)
    with open(os.path.join(HERE, "macros_beam_mc.tex"), "w") as fo:
        fo.write("% written by sim/beam_mc.py; do not edit\n")
        fo.write(f"\\newcommand{{\\BmcLamMaxErr}}{{{err:.2f}}}\n")
        fo.write(f"\\newcommand{{\\BmcCrossMaxErr}}{{{cerr:.2f}}}\n")
        fo.write(f"\\newcommand{{\\BmcN}}{{{len(rows)}}}\n")
    print("max |MC - Prop. 1| =", err, "points; max |MC - Cor. 1 exact| delta* =", cerr)
