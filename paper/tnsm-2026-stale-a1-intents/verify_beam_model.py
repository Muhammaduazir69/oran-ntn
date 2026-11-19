#!/usr/bin/env python3
"""Independent check of beam_model.py (Section IV-A): recomputes the misalignment of a few runs
with separate code, the first-order break-even from the loop means, and the printed macros."""
import csv
import json
import os
import re

import numpy as np
from scipy.special import ive

HERE = os.path.dirname(os.path.abspath(__file__))
R = json.load(open(os.path.join(HERE, "beam_model.json")))
M = {}
for line in open(os.path.join(HERE, "macros_beam_model.tex")):
    g = re.match(r"\\newcommand\{\\(\w+)\}\{(.*)\}\s*$", line)
    if g:
        M[g.group(1)] = g.group(2)
fails, n = 0, 0


def check(ok, msg):
    global fails, n
    n += 1
    fails += 0 if ok else 1
    print(("PASS " if ok else "FAIL ") + msg)


def lam(sd, T, a=0.9, tick=0.0):
    rows = list(csv.DictReader(open(os.path.join(sd, "loop_hist.csv"))))
    w = np.array([float(r["count"]) for r in rows if r["cls"] == "nf"])
    b = np.array([float(r["bin_ms"]) for r in rows if r["cls"] == "nf"]) / 1000
    ell = (w * b).sum() / w.sum()
    miss = sorted(float(r["t"]) for r in csv.DictReader(open(os.path.join(sd, "nf_miss.csv"))))
    sw = sorted(float(r["t"]) for r in csv.DictReader(open(os.path.join(sd, "switches.csv"))))
    ph = miss[0] % 0.1
    missed = {int(round((m - ph) / 0.1)) for m in miss}
    good = [ph + 0.1 * k for k in range(int(3600 / 0.1)) if k not in missed]
    tot, cnt, j, s = 0.0, 0, -1, -1
    for t in np.arange(0, 3600, 0.01):
        while j + 1 < len(good) and good[j + 1] + ell <= t:
            j += 1
        while s + 1 < len(sw) and sw[s + 1] <= t:
            s += 1
        if j < 0 or (s >= 0 and good[j] < sw[s]):
            c = 0.0
        else:
            x = (t - good[j] + tick) / T
            c = a * ive(0, x) + (1 - a) * ive(1, x)
        tot += 1 - c
        cnt += 1
    S = next(csv.DictReader(open(os.path.join(sd, "summary.csv"))))
    return 100 * tot / cnt, 100 * float(S[f"beam_mis_T{T:g}_d0"])


for sd in ["runs/main/pal_s550_gw1/seed04", "runs/main/ground_s1200_gw2/seed07"]:
    p, s = lam(os.path.join(HERE, sd), 2)
    check(abs(p - s) < 1.0, f"{sd} T=2: independent prediction {p:.2f} vs simulated {s:.2f}")
for lay in ["s550_gw1", "s550_gw2", "s1200_gw1", "s1200_gw2"]:
    dl = R["ell"][f"ground_{lay}"] - R["ell"][f"onboard-ttl_{lay}"]
    for T in (0.5, 1, 2, 5):
        c = R["cross"][f"{lay}_T{T:g}"]
        check(abs(c["first"] - 100 * (3 * 0.9 - 1) * dl / (2 * T)) < 1e-6, f"{lay} T={T}: first-order delta* formula")
        check(abs(c["exact"] - c["sim"]) <= c["ci"], f"{lay} T={T}: exact {c['exact']:.2f} inside sim {c['sim']:.2f}+-{c['ci']:.2f}")
L = list(R["lambda"].values())
check(str(len(L)) == M["BmLamN"], "macro BmLamN")
check(f"{max(abs(v['pred'] - v['sim']) for v in L):.1f}" == M["BmLamMaxErr"], "macro BmLamMaxErr")
check(f"{1000 * (R['ell']['ground_s550_gw1'] - R['ell']['onboard-ttl_s550_gw1']):.1f}" == M["BmDlFiveGOne"], "macro BmDlFiveGOne")
print(f"\n{n} checks, {fails} failed")
