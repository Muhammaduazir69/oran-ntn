#!/usr/bin/env python3
"""Round 6 (post hoc): launch-intent attribution at 10N 68E, a stated launch default, the envelope lease under
work conservation. Writes macros_launch.tex and tables/table_launch.tex."""
import csv, glob, math, os
import numpy as np
from scipy import stats
H = os.path.dirname(os.path.abspath(__file__)); PD = os.path.dirname(H); TAB = os.path.join(PD, "tables")
M = {}
def mac(k, v, f="{:.2f}"):
    if isinstance(v, str): M[k] = v; return
    s = f.format(v); M[k + "Abs"] = f.replace("+", "").format(abs(v))
    if s.startswith("-"): s = "\\ensuremath{-}" + s[1:] if any(c in "123456789" for c in s) else s[1:]
    M[k] = s
def rd(p):
    o = {}
    for d in sorted(glob.glob(p + "/seed*")):
        if d.endswith(".tmp"): continue
        r = next(csv.DictReader(open(d + "/dataplane.csv"))); S = next(csv.DictReader(open(d + "/summary.csv")))
        o[int(d.split("seed")[-1])] = dict(crit=100 * float(r["crit_ratio"]), be=100 * float(r["be_share"]), av=float(S["feeder_avail"]), sha=S["binary_sha"])
    return o
def ci(x):
    x = np.asarray(x, float); return float(x.mean()), float(stats.t.ppf(.975, len(x) - 1) * x.std(ddof=1) / math.sqrt(len(x)))
def dif(A, B, k):
    s = sorted(set(A) & set(B)); return np.array([A[i][k] - B[i][k] for i in s])
def sign_p(d):
    d = [x for x in d if abs(x) > 1e-12]; n = len(d); k = max(sum(x > 0 for x in d), sum(x < 0 for x in d))
    return min(1.0, 2 * sum(math.comb(n, j) for j in range(k, n + 1)) / 2 ** n)
def sg(x, nd=2):
    t = f"{x:+.{nd}f}"; return f"{0:.{nd}f}" if float(t) == 0 else t.replace("-", "$-$")
def cell(x, nd=2):
    m, h = ci(x); return f"{sg(m, nd)}$\\pm${h:.{nd}f}"
def rule(x, thr):
    m, h = ci(x); return m <= -thr and m + h < 0 and (x < 0).sum() >= 0.8 * len(x)
K = 8.8
rows = []; shas = set(); FXG = []
for T, tn in [(15, "Fifteen"), (30, "Thirty")]:
    R5 = {a: rd(f"{H}/runs/r5/lo_ta{T}/{a}") for a in ["control", "hold", "env02", "wccontrol", "wcstatic", "wcfixed"]}
    L6 = {a: rd(f"{H}/runs/r6/launch_lo_ta{T}/{a}") for a in ["control", "hold", "env02", "wccontrol", "wcstatic", "wcfixed"]}
    B6 = {a: rd(f"{H}/runs/r6/launch_base_ta{T}/{a}") for a in ["control", "hold"]}
    R5b = {a: rd(f"{H}/runs/r5/base_ta{T}/{a}") for a in ["control", "hold", "wccontrol", "wcfixed", "wcstatic"]}
    WL = {s: rd(f"{H}/runs/r6/wclease_{s}_ta{T}/wcenv02") for s in ("base", "lo")}
    for X in list(L6.values()) + list(B6.values()) + list(WL.values()):
        shas |= {x["sha"] for x in X.values()}
    # attribution from the preregistered arms
    hc = dif(R5["hold"], R5["control"], "crit"); st = dif(R5["wcstatic"], R5["wccontrol"], "crit")
    mac(f"Att{tn}Static", ci(st)[0], "{:+.3f}"); mac(f"Att{tn}StaticCi", ci(st)[1], "{:.3f}")
    mac(f"Att{tn}Share", 100 * ci(st)[0] / ci(hc)[0], "{:.0f}")
    for site, S in (("Base", R5b), ):
        pass
    # launch default (0.8, 0.1)
    c = dif(L6["hold"], L6["control"], "crit"); b = dif(L6["hold"], L6["control"], "be")
    nu = np.mean([x["av"] for x in L6["hold"].values()]); pred = K * (1 - nu) / T
    ec = dif(L6["env02"], L6["hold"], "crit"); eb = dif(L6["env02"], L6["hold"], "be")
    fx = dif(L6["wcfixed"], L6["wccontrol"], "crit"); stl = dif(L6["wcstatic"], L6["wccontrol"], "crit")
    for k, x, nd in [("Crit", c, 3), ("Be", b, 2), ("EnvCrit", ec, 3), ("EnvBe", eb, 2), ("Fixed", fx, 3), ("Static", stl, 3)]:
        mac(f"Ln{tn}{k}", ci(x)[0], "{:+." + str(nd) + "f}"); mac(f"Ln{tn}{k}Ci", ci(x)[1], "{:." + str(nd) + "f}")
    mac(f"Ln{tn}Pred", pred, "{:.3f}"); mac(f"Ln{tn}CritRule", "met" if rule(c, .5) else "not met", "{}"); mac(f"Ln{tn}BeRule", "met" if rule(b, 1.0) else "not met", "{}")
    mac(f"Ln{tn}In", "inside" if abs(-ci(c)[0] - pred) <= ci(c)[1] else "outside", "{}")
    # launch effect itself: hold (launch 0.3/0.2) minus hold (launch 0.8/0.1), same seeds
    le = dif(R5["hold"], L6["hold"], "crit"); lb = dif(R5["hold"], L6["hold"], "be")
    mac(f"LnEff{tn}Crit", ci(le)[0], "{:+.3f}"); mac(f"LnEff{tn}CritCi", ci(le)[1], "{:.3f}"); mac(f"LnEff{tn}Be", ci(lb)[0], "{:+.2f}"); mac(f"LnEff{tn}BeCi", ci(lb)[1], "{:.2f}")
    # base cell with the launch default
    bc = dif(B6["hold"], B6["control"], "crit"); bo = dif(R5b["hold"], R5b["control"], "crit")
    bb = dif(B6["hold"], B6["control"], "be"); mac(f"LnBase{tn}Be", ci(bb)[0], "{:+.2f}"); mac(f"LnBase{tn}BeCi", ci(bb)[1], "{:.2f}")
    mac(f"LnBase{tn}BeRule", "met" if rule(bb, 1.0) else "not met", "{}")
    FXG += [ci(dif(R["wcfixed"], R["wccontrol"], "crit"))[0] for R in (R5b, R5)]
    fs = dif(R5b["wcfixed"], R5b["wcstatic"], "crit"); mac(f"FixMinusStaticBase{tn}", ci(fs)[0], "{:.3f}")
    mac(f"LnBase{tn}Crit", ci(bc)[0], "{:+.3f}"); mac(f"LnBase{tn}CritCi", ci(bc)[1], "{:.3f}")
    # envelope lease under work conservation, and the fixed share against it
    for s_, R in (("Base", R5b), ("Lo", R5)):
        W = WL["base" if s_ == "Base" else "lo"]
        lc = dif(W, R["wccontrol"], "crit"); fl = dif(R["wcfixed"], W, "crit")
        mac(f"Wl{s_}{tn}", ci(lc)[0], "{:+.3f}"); mac(f"Wl{s_}{tn}Ci", ci(lc)[1], "{:.3f}")
        mac(f"Fl{s_}{tn}", ci(fl)[0], "{:+.3f}"); mac(f"Fl{s_}{tn}Ci", ci(fl)[1], "{:.3f}"); mac(f"Fl{s_}{tn}P", sign_p(fl), "{:.2f}")
    for s_, R in (("Base", R5b), ("Lo", R5), ("Ln", L6)):
        fb = dif(R["wcfixed"], R["wccontrol"], "be"); mac(f"FxBe{s_}{tn}", ci(fb)[0], "{:+.2f}"); mac(f"FxBe{s_}{tn}Ci", ci(fb)[1], "{:.2f}")
    fh = dif(R5["wcfixed"], R5["env02"], "crit")
    mac(f"FixVsHardLease{tn}", ci(fh)[0], "{:+.3f}"); mac(f"FixVsHardLease{tn}Ci", ci(fh)[1], "{:.3f}"); mac(f"FixVsHardLease{tn}P", sign_p(fh), "{:.2f}")
    mac(f"FixVsHardLease{tn}Pos", int((fh > 0).sum()), "{:d}")
    rows.append((f"10N 68E, {T} s$^{{\\ddagger}}$", 100 * nu, c, b, pred, ec, eb, stl, fx, rule(c, .5), rule(b, 1.0)))
with open(os.path.join(TAB, "table_launch_rows.tex"), "w") as f:
    f.write("% Generated by the analysis scripts in sim/\n\\midrule\n")
    for lab, av, c, b, pr, ec, eb, st, fx, p1, p2 in rows:
        f.write(f"{lab} & {av:.1f} & 69 & {cell(c,3)}{'$^{\\ast}$' if p1 else ''} & {sg(-pr,3)} & {cell(b)}{'$^{\\ast}$' if p2 else ''} & {cell(ec,3)} & {cell(eb)} & {cell(st,3)} & {cell(fx,3)}\\\\\n")
_ro = [float(r["crit_offered"]) / float(r["cap_bytes"]) for d in glob.glob(f"{H}/runs/r5/*/control/seed*") for r in csv.DictReader(open(d + "/dataplane.csv"))]
mac("CritOfferedShare", 100 * float(np.mean(_ro)), "{:.0f}"); mac("CritOfferedShareMin", 100 * min(_ro), "{:.0f}"); mac("CritOfferedShareMax", 100 * max(_ro), "{:.0f}")
mac("FixGainPctThr", 100 * max(FXG) / 0.5, "{:.0f}")
mac("LaunchHash", "b7601e74c2728542", "{}")
mac("RSixRuns", len(glob.glob(f"{H}/runs/r6/*/*/seed*")), "{:d}")
print("shas", shas)
with open(os.path.join(PD, "macros_launch.tex"), "w") as f:
    f.write("% Generated by the analysis scripts in sim/. Do not edit by hand.\n")
    for k in sorted(M):
        f.write(f"\\newcommand{{\\{k}}}{{{M[k]}}}\n")
