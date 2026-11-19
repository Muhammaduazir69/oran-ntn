#!/usr/bin/env python3
"""Independent check of the Oct 2026 macros (recent designs, lease model): recomputed from
the raw run outputs without importing analyze.py, compared with results_macros.tex."""
import csv, glob, math, os, re, sys
from math import comb

HERE = os.path.dirname(os.path.abspath(__file__))
M = {k: v for k, v in re.findall(r"\\newcommand\{\\(\w+)\}\{((?:[^{}]|\{[^{}]*\})*)\}", open(os.path.join(HERE, "..", "results_macros.tex")).read())}
num = lambda s: float(s.replace("\\ensuremath{-}", "-"))
out, fails = [], 0


def check(ok, msg):
    global fails
    out.append(("ok   " if ok else "FAIL ") + msg)
    fails += 0 if ok else 1


def runs(group, cfg):
    rr = {}
    for d in sorted(glob.glob(os.path.join(HERE, "runs", group, cfg, "seed*"))):
        p = os.path.join(d, "summary.csv")
        if os.path.exists(p):
            rr[int(os.path.basename(d)[4:])] = d
    return rr


def summ(d):
    return next(csv.DictReader(open(os.path.join(d, "summary.csv"))))


def close(name, val, tol=None):
    s = M.get(name)
    if s is None:
        check(False, f"{name} missing")
        return
    nd = len(s.split(".")[1]) if "." in s else 0
    t = tol if tol is not None else 0.5 * 10 ** -nd + 1e-9
    check(abs(num(s) - val) <= t, f"{name} = {s} vs {val:.4f}")


def sign_p(d):
    d = [x for x in d if abs(x) > 1e-12]
    n = len(d); k = max(sum(x > 0 for x in d), n - sum(x > 0 for x in d))
    return min(1.0, 2 * sum(comb(n, j) for j in range(k, n + 1)) / 2 ** n) if n else 1.0


GROUP = {"onboard": "main", "pal": "main", "pred-ho": "recent", "ground-pre": "recent"}
TAG = {"onboard": "SpaceOran", "pred-ho": "Baranda", "ground-pre": "Kube", "pal": "Pal"}
cuts = []
for shell, gw, L in [(550, 1, "FiveGOne"), (550, 2, "FiveGTwo"), (1200, 1, "TwelveGOne")]:
    R = {s: runs(GROUP[s], f"{s}_s{shell}_gw{gw}") for s in GROUP}
    for s, rr in R.items():
        check(len(rr) == 10, f"{s} {shell}/{gw}: ten seeds")
        v = [float(summ(d)["sla_viol_frac"]) * 100 for d in rr.values()]
        close(f"Rec{TAG[s]}{L}Short", sum(v) / len(v))
        v = [100 - float(summ(d)["nf_hit_ratio"]) * 100 for d in rr.values()]
        close(f"Rec{TAG[s]}{L}NfMiss", sum(v) / len(v))
        v = [float(summ(d)["overprov_mean"]) * 100 for d in rr.values()]
        close(f"Rec{TAG[s]}{L}Over", sum(v) / len(v))
    pal = {k: float(summ(d)["sla_viol_frac"]) * 100 for k, d in R["pal"].items()}
    for s in ["onboard", "pred-ho", "ground-pre"]:
        o = {k: float(summ(d)["sla_viol_frac"]) * 100 for k, d in R[s].items()}
        dd = [pal[k] - o[k] for k in sorted(pal)]
        close(f"RecD{TAG[s]}{L}Short", sum(dd) / len(dd))
        check(str(sum(x < 0 for x in dd)) == M.get(f"RecD{TAG[s]}{L}ShortNpos"), f"RecD{TAG[s]}{L}ShortNpos")
        cut = 100 * (sum(o.values()) - sum(pal.values())) / sum(o.values())
        close(f"RecCut{TAG[s]}{L}Short", cut)
        if s != "ground-pre":
            cuts.append(cut)
        if L == "FiveGOne":
            check(abs(sign_p(dd) - num(M[f"RecD{TAG[s]}{L}ShortP"])) < 1e-3, f"RecD{TAG[s]}{L}ShortP")
close("RecCutMin", min(cuts)); close("RecCutMax", max(cuts))
# pred-ho really carries no lease and ground-pre really provisions and never goes on board
for d in runs("recent", "pred-ho_s550_gw1").values():
    s = summ(d)
    # lease_tau_s logs the RIC's tau* estimate for every strategy; the lease is absent when the
    # safe intent is never used while hand-offs (LEASE grants of authority) still happen
    check(float(s["fallback_frac"]) == 0 and float(s["leases"]) > 0,
          f"pred-ho {os.path.basename(d)}: hand-offs, never on the safe intent")
for d in runs("recent", "ground-pre_s550_gw1").values():
    s = summ(d)
    check(float(s["preprov_frac"]) > 0 and float(s["nf_onboard_frac"]) == 0, f"ground-pre {os.path.basename(d)}: provisioned, never on board")
rep = "\n".join(out) + f"\n\n{len(out)} checks, {fails} failed\n"
open(os.path.join(HERE, "verify_recent_report.txt"), "w").write(rep)
print(rep[-2000:])
sys.exit(1 if fails else 0)
