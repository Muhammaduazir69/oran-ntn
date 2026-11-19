#!/usr/bin/env python3
"""Age-of-action analysis of the NF beam outcome (Section IV-A of the TNSM manuscript).

Lemma 1:      P(correct | age A) = a e^{-x} I0(x) + (1-a) e^{-x} I1(x),  x = A / T_b
Proposition 1: Lambda = time average of [no action applied since the switch] + [1 - P(correct | A(t))]
Corollary 1:  break-even gap delta* = (3a-1) (A_g - A_o) / (2 T_b) + O(T_b^-2)

Three tests of increasing information content. (i) Accounting: the age process of each run is
rebuilt from its own logs (nf_miss.csv: every epoch that was never actuated; loop_hist.csv: the loop
of the applied ones; switches.csv: resets) and compared with the simulator's beam_mis_T*_d0 column.
The simulator implements the beam process of the model, so this checks the age accounting only.
(ii) Out of sample: the age processes of held-out seeds 11-20 predict the mean of seeds 1-10.
(iii) No logs: only the mean loop, the epoch period and the mean blackout per switch.
The beam process is event-driven and integrated exactly, so no grid offset applies.
Writes beam_model.json and macros_beam_model.tex.

    python3 beam_model.py
"""
import csv
import glob
import json
import os
import re

import numpy as np
from scipy import integrate
from scipy.optimize import brentq
from scipy.special import ive

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs", "main")
HELD = os.path.join(HERE, "runs", "heldout")
A_G, P, TICK, TB = 0.9, 0.1, 0.0, (0.5, 1, 2, 5)
LAYOUTS = {"s550_gw1": "FiveGOne", "s550_gw2": "FiveGTwo", "s1200_gw1": "TwelveGOne", "s1200_gw2": "TwelveGTwo"}
STRATS = ["ground", "onboard-ttl", "reactive-ttl", "reactive-ttl-pre", "pal"]
TN = {0.5: "Half", 1: "One", 2: "Two", 5: "Five"}


def correct(A, a, T):
    """Lemma 1 (ive(k, x) = e^{-x} I_k(x))."""
    x = A / T
    return a * ive(0, x) + (1 - a) * ive(1, x)


def loops(sd):
    H = {}
    for r in csv.DictReader(open(os.path.join(sd, "loop_hist.csv"))):
        if r["cls"] == "nf":
            H[float(r["bin_ms"]) / 1000] = H.get(float(r["bin_ms"]) / 1000, 0) + float(r["count"])
    b = np.array(sorted(H))
    w = np.array([H[x] for x in b])
    return b, w / w.sum()


def predict_run(sd, dt=0.0049, horizon=3600.0):
    """Proposition 1 on the age process rebuilt from the run's own logs."""
    S = next(csv.DictReader(open(os.path.join(sd, "summary.csv"))))
    lb, lw = loops(sd)
    ell = float((lb * lw).sum())
    miss = np.array([float(r["t"]) for r in csv.DictReader(open(os.path.join(sd, "nf_miss.csv")))])
    sw = np.array([float(r["t"]) for r in csv.DictReader(open(os.path.join(sd, "switches.csv")))])
    ph = float(miss[0] % P) if len(miss) else 0.0733
    ep = np.arange(ph, horizon, P)
    k = np.round((miss - ph) / P).astype(int)
    keep = np.ones(len(ep), bool)
    keep[k[(k >= 0) & (k < len(ep))]] = False
    applied = ep[keep]
    act = applied + ell
    t = np.arange(0, horizon, dt)
    idx = np.searchsorted(act, t, side="right") - 1
    meas = np.where(idx >= 0, applied[np.clip(idx, 0, None)], -np.inf)
    sidx = np.searchsorted(sw, t, side="right") - 1
    last_sw = np.where(sidx >= 0, sw[np.clip(sidx, 0, None)], -1.0)
    none = (meas < last_sw) | (idx < 0)
    age = np.where(none, 1.0, t - meas + TICK)
    out = {}
    for T in TB:
        c = np.where(none, 0.0, correct(age, A_G, T))
        out[T] = (float(1 - c.mean()), float(S[f"beam_mis_T{T:g}_d0"]))
    return out, ell


def no_log_lambda(sd_dir, T, a=A_G):
    """Equation (13) in exact form from a held-out mean loop and blackout only: age uniform on
    [ell, ell + P), and no action for the mean blackout after each switch."""
    ell, off = [], []
    for sd in sorted(glob.glob(os.path.join(sd_dir, "seed*"))):
        S = next(csv.DictReader(open(os.path.join(sd, "summary.csv"))))
        lb, lw = loops(sd)
        ell.append(float((lb * lw).sum()))
        off.append(float(S["blackout_mean_s"]) * float(S["switches"]) / float(S["served_s"]))
    l = float(np.mean(ell))
    avg = integrate.quad(lambda A: correct(A, a, T), l, l + P)[0] / P
    return 1 - (1 - float(np.mean(off))) * avg


def delta_star(lay, T):
    """Corollary 1, exact form: the gap at which the time-averaged correctness of an on-board
    controller (accuracy a - delta) equals that of the ground (accuracy a), both periodic."""
    def corr(a, pat):
        b, w = [], []
        for sd in glob.glob(os.path.join(RUNS, pat, "seed*")):
            lb, lw = loops(sd)
            b.append(lb)
            w.append(lw)
        lb, lw = np.concatenate(b), np.concatenate(w) / len(b)
        return sum(wi * integrate.quad(lambda A: correct(A + TICK, a, T), li, li + P)[0] / P
                   for li, wi in zip(lb, lw) if wi > 1e-5)
    cg = corr(A_G, f"ground_{lay}")
    return 100 * brentq(lambda d: corr(A_G - d, f"onboard-ttl_{lay}") - cg, -0.05, 0.3)


def main():
    M = {}
    for line in open(os.path.join(HERE, "..", "results_macros.tex")):
        g = re.match(r"\\newcommand\{\\(\w+)\}\{(.*)\}\s*$", line)
        if g:
            M[g.group(1)] = g.group(2)
    res = {"lambda": {}, "cross": {}, "ell": {}}
    for lay in LAYOUTS:
        for st in STRATS:
            pr, sm, ells = {T: [] for T in TB}, {T: [] for T in TB}, []
            for sd in sorted(glob.glob(os.path.join(RUNS, f"{st}_{lay}", "seed*"))):
                o, ell = predict_run(sd)
                ells.append(ell)
                for T in TB:
                    pr[T].append(o[T][0])
                    sm[T].append(o[T][1])
            if not ells:
                continue
            res["ell"][f"{st}_{lay}"] = float(np.mean(ells))
            for T in TB:
                p, s = np.array(pr[T]), np.array(sm[T])
                d = p - s
                h = 2.262 * d.std(ddof=1) / np.sqrt(len(d))
                hs = 2.262 * s.std(ddof=1) / np.sqrt(len(s))
                res["lambda"][f"{st}_{lay}_T{T:g}"] = dict(pred=100 * p.mean(), sim=100 * s.mean(), sim_ci=100 * hs,
                                                         diff=100 * d.mean(), diff_ci=100 * h)
            print(lay, st, "done", flush=True)
        for T in TB:
            ds = delta_star(lay, T)
            sim = float(M[f"BeamCross{LAYOUTS[lay]}T{TN[T]}"])
            ci = float(M[f"BeamCrossSeed{LAYOUTS[lay]}T{TN[T]}Ci"])
            dl = res["ell"][f"ground_{lay}"] - res["ell"][f"onboard-ttl_{lay}"]
            fo = 100 * (3 * A_G - 1) * dl / (2 * T)
            res["cross"][f"{lay}_T{T:g}"] = dict(exact=ds, first=fo, sim=sim, ci=ci)
    # out of sample: held-out seeds predict the scored mean, with and without logs
    oos, nol = [], []
    for lay in ("s550_gw1", "s550_gw2", "s1200_gw1"):
        for st in ("ground", "onboard-ttl", "pal"):
            hd = os.path.join(HELD, f"{st}_{lay}")
            if not os.path.isdir(hd):
                continue
            for T in TB:
                pr = []
                for sd in sorted(glob.glob(os.path.join(hd, "seed*"))):
                    o, _ = predict_run(sd)
                    pr.append(o[T][0])
                key = f"{st}_{lay}_T{T:g}"
                sv = res["lambda"].get(key)
                if sv is None:
                    continue
                e = 100 * float(np.mean(pr)) - sv["sim"]
                oos.append(dict(key=key, err=e, ci=sv["sim_ci"]))
                if st != "ground":
                    nol.append(dict(key=key, err=100 * no_log_lambda(hd, T) - sv["sim"], ci=sv["sim_ci"]))
        print("oos", lay, flush=True)
    res["oos"], res["nolog"] = oos, nol
    json.dump(res, open(os.path.join(HERE, "beam_model.json"), "w"), indent=1)
    L = res["lambda"].values()
    C = res["cross"].values()
    mac = {
        "BmLamN": len(L),
        "BmLamMaxErr": f"{max(abs(v['pred'] - v['sim']) for v in L):.1f}",
        "BmLamMeanErr": f"{np.mean([abs(v['pred'] - v['sim']) for v in L]):.2f}",
        "BmLamInCi": int(sum(abs(v["pred"] - v["sim"]) <= v["sim_ci"] for v in L)),
        "BmCrossN": len(C),
        "BmCrossInCi": int(sum(abs(v["exact"] - v["sim"]) <= v["ci"] for v in C)),
        "BmCrossMaxErr": f"{max(abs(v['exact'] - v['sim']) for v in C):.2f}",
        "BmCrossFirstMaxErr": f"{max(abs(v['first'] - v['sim']) for v in C):.2f}",
        "BmDlFiveGOne": f"{1000 * (res['ell']['ground_s550_gw1'] - res['ell']['onboard-ttl_s550_gw1']):.1f}",
        "BmDlFiveGTwo": f"{1000 * (res['ell']['ground_s550_gw2'] - res['ell']['onboard-ttl_s550_gw2']):.1f}",
        "BmDlTwelveGOne": f"{1000 * (res['ell']['ground_s1200_gw1'] - res['ell']['onboard-ttl_s1200_gw1']):.1f}",
        "BmLoopGndFiveGOne": f"{1000 * res['ell']['ground_s550_gw1']:.1f}",
        "BmLoopOnbFiveGOne": f"{1000 * res['ell']['onboard-ttl_s550_gw1']:.1f}",
        "BmCrossExactFiveGOneTTwo": f"{res['cross']['s550_gw1_T2']['exact']:.2f}",
        "BmCrossFirstFiveGOneTTwo": f"{res['cross']['s550_gw1_T2']['first']:.2f}",
        "BmCrossExactFiveGTwoTHalf": f"{res['cross']['s550_gw2_T0.5']['exact']:.2f}",
        "BmGndPredFiveGOne": f"{res['lambda']['ground_s550_gw1_T2']['pred']:.1f}",
        "BmGndSimFiveGOne": f"{res['lambda']['ground_s550_gw1_T2']['sim']:.1f}",
        "BmOosN": len(oos),
        "BmOosMaxErr": f"{max(abs(v['err']) for v in oos):.2f}" if oos else "n/a",
        "BmOosMeanErr": f"{np.mean([abs(v['err']) for v in oos]):.2f}" if oos else "n/a",
        "BmOosInCi": int(sum(abs(v["err"]) <= v["ci"] for v in oos)),
        "BmNoLogN": len(nol),
        "BmNoLogMaxErr": f"{max(abs(v['err']) for v in nol):.2f}" if nol else "n/a",
        "BmNoLogInCi": int(sum(abs(v["err"]) <= v["ci"] for v in nol)),
    }
    with open(os.path.join(HERE, "macros_beam_model.tex"), "w") as f:
        f.write("% written by sim/beam_model.py; do not edit\n")
        for k in sorted(mac):
            f.write(f"\\newcommand{{\\{k}}}{{{mac[k]}}}\n")
    print(json.dumps(mac, indent=1))


if __name__ == "__main__":
    main()
