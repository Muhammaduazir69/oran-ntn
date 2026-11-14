#!/usr/bin/env python3
"""Exact-model Monte Carlo of Corollary 3 (no ns-3): alternating renewal feeder with exponential
contacts and outages, Poisson intent updates, rates of (H3). Compares the long-run shortfall removed by
the lease tau* against holding with the closed form (value) over T_A1 and the outage mean.
Writes macros_value_mc.tex. Uses the held-out rates of results_macros.tex."""
import os
import re
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
M = {}
for line in open(os.path.join(HERE, "..", "results_macros.tex")):
    g = re.match(r"\\newcommand\{\\(\w+)\}\{(.*)\}\s*$", line)
    if g:
        M[g.group(1)] = g.group(2)
r = {k: float(M[f"EstRate{k.capitalize()}"]) / 100 for k in ["pc", "pv", "ps"]}


def mc(T, theta, nu, eps, n=400000, seed=1):
    rng = np.random.default_rng(seed)
    Th = rng.exponential(theta, n)
    C = rng.exponential(theta * nu / (1 - nu), n)
    tau = -T * np.log(1 - eps)
    # first supersession after the last contact
    U = rng.exponential(T, n)
    over = np.clip(Th - tau, 0, None)
    # time in (tau, Theta) under a superseded snapshot if held, and current otherwise
    sup = np.clip(Th - np.maximum(U, tau), 0, None)
    removed = r["pc"] * (over - sup) + r["pv"] * sup - r["ps"] * over
    return 100 * removed.sum() / (Th.sum() + C.sum())


def closed(T, theta, nu, eps):
    q = (1 - eps) ** (T / theta)
    return 100 * (1 - nu) * q * ((r["pv"] - r["ps"]) - (r["pv"] - r["pc"]) * (1 - eps) / (1 + theta / T))


errs = []
for T in (30, 60, 300, 900, 3600):
    for theta in (5, 45, 200):
        m, c = mc(T, theta, 0.8, 0.2), closed(T, theta, 0.8, 0.2)
        errs.append(abs(m - c))
        print(T, theta, round(m, 3), round(c, 3))
with open(os.path.join(HERE, "macros_value_mc.tex"), "w") as f:
    f.write("% written by sim/lease_value_mc.py; do not edit\n")
    f.write(f"\\newcommand{{\\VmcMaxErr}}{{{max(errs):.3f}}}\n\\newcommand{{\\VmcN}}{{{len(errs)}}}\n")
print("max err", max(errs))
