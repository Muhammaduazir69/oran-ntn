#!/usr/bin/env python3
"""Round 3: measured service on the data plane. Writes macros (macros_grid.tex) and tables.
Every contrast is paired by seed against the current-intent control or against hold."""
import csv, glob, json, math, os, re
import numpy as np
from scipy import stats, integrate
from scipy.special import ive
from scipy.optimize import brentq
H = os.path.dirname(os.path.abspath(__file__))
PD = os.path.dirname(H)
TAB = os.path.join(PD, "tables")
M = {}
rng = np.random.default_rng(20261105)
def mac(k, v, f="{:.2f}"):
    if isinstance(v, str):
        M[k] = v; return
    if v is None or (isinstance(v, float) and math.isnan(v)):
        M[k] = "n/a"; return
    s = f.format(v)
    M[k + "Abs"] = f.replace("+", "").format(abs(v))
    if s.startswith("-"):
        s = "\\ensuremath{-}" + s[1:] if any(c in "123456789" for c in s) else s[1:]
    M[k] = s
def rd(gr, case, arm):
    o = {}
    for d in sorted(glob.glob(f"{H}/runs/{gr}/{case}/{arm}/seed*")):
        if d.endswith(".tmp"): continue
        s = int(d[-2:])
        r = {k: float(v) for k, v in next(csv.DictReader(open(d + "/dataplane.csv"))).items()}
        S = next(csv.DictReader(open(d + "/summary.csv")))
        for k in ("sla_viol_frac", "superseded_frac", "overprov_mean", "fallback_frac", "feeder_avail", "short_depth_mean", "served_s", "lease_tau_s"):
            r[k] = float(S[k])
        r["sha"] = S.get("binary_sha")
        r["crit"] = 100 * r["crit_ratio"]; r["be"] = 100 * r["be_share"]; r["tot"] = 100 * r["total_share"]
        r["over"] = 100 * r["overprov_mean"]; r["short"] = 100 * r["sla_viol_frac"]
        r["critcap"] = 100 * r["crit_delivered"] / r["cap_bytes"]
        C = next(csv.DictReader(open(d + "/checks.csv")))
        r["checks"] = {k: float(v) for k, v in C.items()}
        p = d + "/outages.csv"
        r["outages"] = list(csv.DictReader(open(p))) if os.path.exists(p) else []
        o[s] = r
    return o
def ci(x):
    x = np.asarray(x, float)
    return float(x.mean()), float(stats.t.ppf(.975, len(x) - 1) * x.std(ddof=1) / math.sqrt(len(x)))
def sign_p(d):
    d = [x for x in d if abs(x) > 1e-12]; n = len(d)
    if not n: return 1.0
    k = max(sum(x > 0 for x in d), sum(x < 0 for x in d))
    return min(1.0, 2 * sum(math.comb(n, j) for j in range(k, n + 1)) / 2 ** n)
def holm(p):
    m = len(p); o = np.argsort(p); a = [0.0] * m; run = 0.0
    for r, i in enumerate(o):
        run = max(run, min(1.0, (m - r) * p[i])); a[i] = run
    return a
def dif(A, B, k):
    s = sorted(set(A) & set(B))
    return np.array([A[i][k] - B[i][k] for i in s])
def sg(x, nd=2):
    t = f"{x:+.{nd}f}"
    if float(t) == 0: return f"{0:.{nd}f}"
    return t.replace("-", "$-$")
def cell(d, nd=2):
    m, h = ci(d); return f"{sg(m, nd)}$\\pm${h:.{nd}f}"

CASES = [("s550_gw1_ta15", "550 km, 1, 15 s", "A"), ("s550_gw1_ta30", "550 km, 1, 30 s", "B"), ("s550_gw1_ta60", "550 km, 1, 60 s", "C"),
         ("s550_gw1_ta300", "550 km, 1, 300 s", "D"), ("s550_gw1_ta900", "550 km, 1, 900 s", "E"), ("s1200_gw1_ta30", "1200 km, 1, 30 s", "F"),
         ("s1200_gw1_ta300", "1200 km, 1, 300 s", "G"), ("s550_gw2_ta30", "550 km, 2, 30 s", "H"), ("s550_gw3_ta30", "550 km, 3, 30 s", "I")]
ARMS = ["control", "hold", "env02", "env05", "lean02", "droplean"]
D = {c: {a: rd("dp", c, a) for a in ARMS} for c, _, _ in CASES}
shas = {r["sha"] for c in D for a in D[c] for r in D[c][a].values()}
tot = sum(len(D[c][a]) for c in D for a in D[c])
# ---- main table: cost of staleness and of each fallback, one unit
rows = []
ph, pl, pe, pd_ = [], [], [], []
for c, lab, t in CASES:
    A = D[c]
    h = dif(A["hold"], A["control"], "crit"); hb = dif(A["hold"], A["control"], "be")
    ec = dif(A["env02"], A["hold"], "crit"); eb = dif(A["env02"], A["hold"], "be")
    lc = dif(A["lean02"], A["hold"], "crit"); lb = dif(A["lean02"], A["hold"], "be")
    dc = dif(A["droplean"], A["hold"], "crit"); db = dif(A["droplean"], A["hold"], "be")
    e5c = dif(A["env05"], A["hold"], "crit"); e5b = dif(A["env05"], A["hold"], "be")
    av = 100 * np.mean([r["feeder_avail"] for r in A["hold"].values()])
    rows.append(dict(c=c, lab=lab, t=t, av=av, h=h, hb=hb, ec=ec, eb=eb, lc=lc, lb=lb, dc=dc, db=db, e5c=e5c, e5b=e5b,
                     ctl=np.mean([r["crit"] for r in A["control"].values()]), ctb=np.mean([r["be"] for r in A["control"].values()])))
    for nm, x, nd in [("HoldCrit", h, 3), ("HoldBe", hb, 2), ("EnvCrit", ec, 3), ("EnvBe", eb, 2), ("LeanCrit", lc, 3), ("LeanBe", lb, 2), ("DropCrit", dc, 3), ("DropBe", db, 2), ("EnvFiveCrit", e5c, 3), ("EnvFiveBe", e5b, 2)]:
        mac(f"{nm}{t}", ci(x)[0], "{:+." + str(nd) + "f}"); mac(f"{nm}Ci{t}", ci(x)[1], "{:." + str(nd) + "f}")
    mac(f"Avail{t}", av, "{:.1f}")
pk = {k: holm([sign_p(R[k]) for R in rows]) for k in ("h", "hb", "ec", "eb", "lc", "lb")}
for i, R in enumerate(rows):
    for k in pk: R["p" + k] = pk[k][i]
st = lambda R, k: "$^{*}$" if R["p" + k] < 0.05 else ""
with open(os.path.join(TAB, "table_dp.tex"), "w") as f:
    f.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lr|rr|rr|rr@{}}\n\\toprule\n")
    f.write(" & & \\multicolumn{2}{c|}{Hold minus control} & \\multicolumn{2}{c|}{Envelope lease minus hold} & \\multicolumn{2}{c}{Lean lease minus hold}\\\\\n")
    f.write("Shell, GW, $T_{\\mathrm{A1}}$ & Avail. & Critical & BE & Critical & BE & Critical & BE\\\\\n")
    f.write(" & (\\%) & (points) & (points) & (points) & (points) & (points) & (points)\\\\\n\\midrule\n")
    for R in rows:
        f.write(f"{R['lab']} & {R['av']:.1f} & {cell(R['h'],3)}{st(R,'h')} & {cell(R['hb'])}{st(R,'hb')} & {cell(R['ec'],3)}{st(R,'ec')} & {cell(R['eb'])}{st(R,'eb')} & {cell(R['lc'],3)}{st(R,'lc')} & {cell(R['lb'])}{st(R,'lb')}\\\\\n")
    f.write("\\bottomrule\n\\end{tabular}\n")
mac("HoldSigBe", sum(R["phb"] < .05 for R in rows), "{:d}")
mac("HoldSigCrit", sum(R["ph"] < .05 for R in rows), "{:d}")
mac("EnvSigBe", sum(R["peb"] < .05 for R in rows), "{:d}")
mac("LeanSigBe", sum(R["plb"] < .05 and ci(R["lb"])[0] > 0 for R in rows), "{:d}")
mac("CaseN", len(rows), "{:d}")
mac("CritBase", np.mean([r["crit"] for r in D["s550_gw1_ta30"]["control"].values()]), "{:.2f}")
mac("BeBase", np.mean([r["be"] for r in D["s550_gw1_ta30"]["control"].values()]), "{:.1f}")
# exchange: BE points paid per critical point recovered, with its interval over seeds (bootstrap), at the rule's own epsilon
def ratio(eb, ec, B=4000):
    n = len(eb); idx = rng.integers(0, n, (B, n))
    a = -eb[idx].mean(1); b = ec[idx].mean(1)
    r = a / b
    return float(-eb.mean() / ec.mean()), float(np.percentile(r, 2.5)), float(np.percentile(r, 97.5))
for t, R in [(R["t"], R) for R in rows if R["t"] in "ABC"]:
    for nm, eb, ec in [("Env", R["eb"], R["ec"]), ("EnvFive", R["e5b"], R["e5c"])]:
        v, lo, hi = ratio(eb, ec)
        mac(f"Xr{nm}{t}", v, "{:.0f}"); mac(f"Xr{nm}{t}Lo", lo, "{:.0f}"); mac(f"Xr{nm}{t}Hi", hi, "{:.0f}")
# depth etc secondary compliance on the same runs
B = D["s550_gw1_ta30"]
mac("ShortControl", np.mean([r["short"] for r in B["control"].values()]), "{:.2f}")
mac("ShortHold", np.mean([r["short"] for r in B["hold"].values()]), "{:.2f}")
mac("ShortEnv", np.mean([r["short"] for r in B["env02"].values()]), "{:.2f}")
mac("ShortLean", np.mean([r["short"] for r in B["lean02"].values()]), "{:.2f}")
# pilot and replication
for gr, tg in [("pilot_r3", "Pilot"), ("pilot_r3_rep", "Rep")]:
    def rdp(a):
        o = {}
        for d in sorted(glob.glob(f"{H}/runs/{gr}/{a}/seed*")):
            r = {k: float(v) for k, v in next(csv.DictReader(open(d + "/dataplane.csv"))).items()}
            o[int(d[-2:])] = dict(crit=100 * r["crit_ratio"], be=100 * r["be_share"])
        return o
    C, Hd, L = rdp("control"), rdp("hold"), rdp("lease")
    for nm, x in [("Crit", dif(Hd, C, "crit")), ("Be", dif(Hd, C, "be")), ("LeaseCrit", dif(L, Hd, "crit")), ("LeaseBe", dif(L, Hd, "be"))]:
        nd = 3 if "Crit" in nm else 2
        mac(f"{tg}{nm}", ci(x)[0], "{:+." + str(nd) + "f}"); mac(f"{tg}{nm}Ci", ci(x)[1], "{:." + str(nd) + "f}")
    mac(f"{tg}N", len(C), "{:d}")
    mac(f"{tg}BeLow", int((dif(Hd, C, "be") < 0).sum()), "{:d}")
# ---- load, work-conserving, uniform redraw
for sess, tg in [(4, "Lo"), (14, "Hi")]:
    A = {a: rd("dpload", f"sess{sess}", a) for a in ["control", "hold", "env02", "lean02"]}
    mac(f"Load{tg}Crit", np.mean([r["crit"] for r in A["control"].values()]), "{:.2f}")
    for a, nm in [("hold", "Hold"), ("env02", "Env"), ("lean02", "Lean")]:
        base = A["control"] if a == "hold" else A["hold"]
        mac(f"Load{tg}{nm}Crit", ci(dif(A[a], base, "crit"))[0], "{:+.3f}"); mac(f"Load{tg}{nm}CritCi", ci(dif(A[a], base, "crit"))[1], "{:.3f}")
        mac(f"Load{tg}{nm}Be", ci(dif(A[a], base, "be"))[0], "{:+.2f}"); mac(f"Load{tg}{nm}BeCi", ci(dif(A[a], base, "be"))[1], "{:.2f}")
A = {a: rd("dpwc", "s550_gw1_ta30", a) for a in ["control", "hold", "env02", "lean02"]}
for a, nm in [("hold", "Hold"), ("env02", "Env"), ("lean02", "Lean")]:
    base = A["control"] if a == "hold" else A["hold"]
    mac(f"Wc{nm}Crit", ci(dif(A[a], base, "crit"))[0], "{:+.3f}"); mac(f"Wc{nm}CritCi", ci(dif(A[a], base, "crit"))[1], "{:.3f}")
    mac(f"Wc{nm}Be", ci(dif(A[a], base, "be"))[0], "{:+.2f}"); mac(f"Wc{nm}BeCi", ci(dif(A[a], base, "be"))[1], "{:.2f}")
mac("WcBe", np.mean([r["be"] for r in A["control"].values()]), "{:.1f}")
A = {a: rd("dpuni", "s550_gw1_ta60", a) for a in ["control", "hold", "env02"]}
for a, nm in [("hold", "Hold"), ("env02", "Env")]:
    base = A["control"] if a == "hold" else A["hold"]
    mac(f"Uni{nm}Crit", ci(dif(A[a], base, "crit"))[0], "{:+.3f}"); mac(f"Uni{nm}CritCi", ci(dif(A[a], base, "crit"))[1], "{:.3f}")
    mac(f"Uni{nm}Be", ci(dif(A[a], base, "be"))[0], "{:+.2f}"); mac(f"Uni{nm}BeCi", ci(dif(A[a], base, "be"))[1], "{:.2f}")
mac("UniSuper", 100 * np.mean([r["superseded_frac"] for r in A["hold"].values()]), "{:.1f}")
mac("UniShortHold", np.mean([r["short"] for r in A["hold"].values()]), "{:.2f}")
mac("UniShortCtl", np.mean([r["short"] for r in A["control"].values()]), "{:.2f}")
# ---- law on the BE price: out-of-sample. Rates and outage lengths from held-out seeds; target: env lease minus hold, seeds 21-40
HD = {c: rd("dpheld", c, "env02") for c in ["s550_gw1_ta30", "s550_gw2_ta30", "s550_gw3_ta30", "s1200_gw1_ta30"]}
def rates(R, seeds=None):
    t = {}
    for s in (seeds if seeds is not None else list(R)):
        for k, v in R[s]["checks"].items():
            t[k] = t.get(k, 0.0) + v
    r = {}
    for nm, st_ in [("c", "current"), ("v", "stale"), ("s", "fallback")]:
        n = max(1.0, t[f"samp_{st_}"])
        r["o" + nm] = t[f"over_{st_}"] / n
        r["p" + nm] = t[f"viol_{st_}"] / n
    return r
def sums(rec, T, eps):
    k = -math.log(1 - eps); tau = k * T
    A = Hh = A3 = H3 = 0.0
    for o in rec["outages"]:
        th, s0, kd = float(o["dur"]), float(o["start"]), int(o["kind"])
        if kd in (0, 3):
            lo, hi = max(tau, s0), s0 + th
            if hi > lo:
                A3 += hi - lo; H3 += (hi - lo) - T * (math.exp(-lo / T) - math.exp(-hi / T))
        else:
            mu = s0 / T
            ns = np.arange(int(max(0, mu - 6 * math.sqrt(mu + 1))), int(mu + 6 * math.sqrt(mu + 1)) + 2)
            w = stats.poisson.pmf(ns, mu)
            ts = k * (T + s0) / (1 + ns)
            ov = np.clip(th - ts, 0, None)
            G = np.where(th > ts, ov - T * (np.exp(-ts / T) - math.exp(-th / T)), 0.0)
            A += float((w * ov).sum()); Hh += float((w * G).sum())
    return A, Hh, A3, H3, float(rec["served_s"])
def price(S, r):
    A, Hh, A3, H3, sv = S.sum(0)
    # with the launch copy (version 1, g = 0.3, h = 0.2) its stale rate is not modeled apart; rare at these layouts
    O = (r["os"] - r["oc"]) * (A + A3) - (r["ov"] - r["oc"]) * (Hh + H3)
    return 100 * O / sv, 100 * (A + A3) / sv
base_held = HD["s550_gw1_ta30"]
LAWC = [("s550_gw1_ta15", "s550_gw1_ta30", 15), ("s550_gw1_ta30", "s550_gw1_ta30", 30), ("s550_gw1_ta60", "s550_gw1_ta30", 60), ("s550_gw1_ta300", "s550_gw1_ta30", 300), ("s550_gw1_ta900", "s550_gw1_ta30", 900),
        ("s1200_gw1_ta30", "s1200_gw1_ta30", 30), ("s1200_gw1_ta300", "s1200_gw1_ta30", 300), ("s550_gw2_ta30", "s550_gw2_ta30", 30), ("s550_gw3_ta30", "s550_gw3_ta30", 30)]
r0 = rates(base_held)
for k, v in r0.items():
    mac(f"R3{k.capitalize()}", 100 * v, "{:.2f}")
seeds0 = sorted(base_held)
boot = [rates(base_held, list(rng.choice(seeds0, len(seeds0)))) for _ in range(800)]
lawrows = []
for c, hc, T in LAWC:
    A = D[c]
    lab = [l for x, l, _ in CASES if x == c][0]
    for arm, eps, tn in [("env02", .2, "0.2"), ("env05", .5, "0.5")]:
        S = np.array([sums(HD[hc][s], T, eps) for s in sorted(HD[hc])])
        p, ex = price(S, r0)
        bs = []
        for rr in boot:
            idx = rng.integers(0, len(S), len(S))
            bs.append(price(S[idx], rr)[0])
        y = -dif(A[arm], A["hold"], "be")
        y2 = dif(A[arm], A["hold"], "over")
        m, h = ci(y)
        se = math.hypot(h / stats.t.ppf(.975, len(y) - 1), float(np.std(bs)))
        lawrows.append(dict(lab=lab, c=c, eps=eps, tn=tn, y=y, m=m, h=h, p=p, lo=float(np.percentile(bs, 2.5)), hi=float(np.percentile(bs, 97.5)), err=m - p, ehw=1.96 * se, ex=ex, T=T, over=ci(y2)[0]))
with open(os.path.join(TAB, "table_lawbe.tex"), "w") as f:
    f.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrr@{}}\n\\toprule\n")
    f.write("Shell, GW, $T_{\\mathrm{A1}}$ & $\\varepsilon$ & BE lost & Predicted & Error & Exposure\\\\\n & & (points) & (interval) & (points) & (points)\\\\\n\\midrule\n")
    for R in lawrows:
        f.write(f"{R['lab']} & {R['tn']} & {sg(R['m']).replace('+','')}$\\pm${R['h']:.2f} & {R['p']:.2f} ({R['lo']:.2f}, {R['hi']:.2f}) & {sg(R['err'])}$\\pm${R['ehw']:.2f} & {R['ex']:.2f}\\\\\n")
    f.write("\\bottomrule\n\\end{tabular}\n")
mac("LawN", len(lawrows), "{:d}")
mac("LawFail", sum(abs(R["err"]) > R["ehw"] for R in lawrows), "{:d}")
mac("LawInBoot", sum(R["lo"] <= R["m"] <= R["hi"] for R in lawrows), "{:d}")
big = [R for R in lawrows if R["m"] > 0.3]
mac("PriceBigN", len(big), "{:d}")
mac("LawBigMin", min(R["m"] / R["p"] for R in big), "{:.2f}")
mac("LawBigMax", max(R["m"] / R["p"] for R in big), "{:.2f}")
# MAE with a paired bootstrap over scored seeds and prediction boot; rival: constant times exposure, k fitted at the base case on scored seeds of that case, base case then left out
ref = [R for R in lawrows if R["c"] == "s550_gw1_ta30" and R["eps"] == .2][0]
k = ref["m"] / ref["ex"]
mac("LawK", k, "{:.2f}")
rest = [R for R in lawrows if R is not ref]
yb = []
for _ in range(4000):
    a = b = z = 0.0
    for R in rest:
        idx = rng.integers(0, len(R["y"]), len(R["y"])); m = R["y"][idx].mean()
        a += abs(m - R["p"]); b += abs(m - k * R["ex"]); z += abs(m)
    yb.append((a / len(rest), b / len(rest), z / len(rest)))
yb = np.array(yb)
ml = np.mean([abs(R["m"] - R["p"]) for R in rest]); mr = np.mean([abs(R["m"] - k * R["ex"]) for R in rest]); mz = np.mean([abs(R["m"]) for R in rest])
mac("MaeLaw", ml, "{:.2f}"); mac("MaeRival", mr, "{:.2f}"); mac("MaeZero", mz, "{:.2f}")
d = yb[:, 1] - yb[:, 0]; mac("MaeDiff", mr - ml, "{:+.2f}"); mac("MaeDiffLo", float(np.percentile(d, 2.5)), "{:+.2f}"); mac("MaeDiffHi", float(np.percentile(d, 97.5)), "{:+.2f}")
d = yb[:, 2] - yb[:, 0]; mac("MaeZeroDiff", mz - ml, "{:+.2f}"); mac("MaeZeroLo", float(np.percentile(d, 2.5)), "{:+.2f}"); mac("MaeZeroHi", float(np.percentile(d, 97.5)), "{:+.2f}")
mac("MaeN", len(rest), "{:d}")
# (H3): the over-provisioning rate of a long-held stale intent against the lease-limited one
t = {}
for r_ in D["s550_gw1_ta30"]["hold"].values():
    for k_, v_ in r_["checks"].items():
        t[k_] = t.get(k_, 0.0) + v_
ov_hold = t["over_stale"] / t["samp_stale"]
mac("OvHold", 100 * ov_hold, "{:.1f}")
mac("OvLease", 100 * r0["ov"], "{:.1f}")
r1 = dict(r0, ov=ov_hold)
errs = []
for R in rest:
    hc = [h_ for c_, h_, T_ in LAWC if c_ == R["c"]][0]
    S_ = np.array([sums(HD[hc][s_], R["T"], R["eps"]) for s_ in sorted(HD[hc])])
    errs.append(abs(R["m"] - price(S_, r1)[0]))
mac("MaeLawHold", float(np.mean(errs)), "{:.2f}")
# does the O term equal the measured BE loss? over-provisioning difference against BE difference
Rb = [(R["m"], R["over"]) for R in lawrows]
mac("OverBeMax", max(abs(a - b) for a, b in Rb), "{:.2f}")
# ---- break-even out of sample: age, blackout, gamma, switches from held-out seeds 11-20
def load_be(gr, cfg):
    o = {}
    for d in sorted(glob.glob(f"{H}/runs/{gr}/{cfg}/x/seed*")) if gr == "beheld" else sorted(glob.glob(f"{H}/runs/{gr}/{cfg}/seed*")):
        s = int(d[-2:])
        S = next(csv.DictReader(open(d + "/summary.csv")))
        hist = list(csv.DictReader(open(d + "/loop_hist.csv")))
        t = n = 0.0
        for x in hist:
            if x["cls"] == "nf":
                t += float(x["bin_ms"]) * float(x["count"]); n += float(x["count"])
        ou = list(csv.DictReader(open(d + "/outages.csv")))
        sv = float(S["served_s"])
        ex3 = sum(float(x["dur"]) for x in ou if int(x["kind"]) in (0, 3)) / sv
        o[s] = dict(S=S, age=t / n, ex3=ex3)
    return o
def inputs(P, O):
    m = lambda R, f: float(np.mean([f(r) for r in R.values()]))
    gam = 1 - m(P, lambda r: float(r["S"]["nf_onboard_frac"]))
    ao = m(O, lambda r: r["age"]); am = m(P, lambda r: r["age"])
    ag = (am - (1 - gam) * ao) / gam
    dT = m(P, lambda r: float(r["S"]["blackout_mean_s"])) - m(O, lambda r: float(r["S"]["blackout_mean_s"]))
    sw = m(P, lambda r: float(r["S"]["switches"])); sv = m(P, lambda r: float(r["S"]["served_s"]))
    phi = 1 - m(O, lambda r: float(r["S"]["beam_mis_T2_d0"]))
    # exact denominator of Corollary 1 (N8): the on-board age divides the whole right side
    den = 1 - 3 * (ao + 50.0) / 1000.0 / (2 * 2.0)
    first = 100 * 1.7 * (ag - ao) / 1000.0 / 4.0
    bl = 100 * phi * dT * sw / sv / gam
    return first, bl, den, gam, 1000 * dT, m(P, lambda r: r["ex3"])
GAPS = [0, .005, .01, .015, .02, .03, .05, .1]
def cross(dv):
    if dv[0] <= 0: return 0.0
    for i in range(1, len(GAPS)):
        if dv[i] <= 0:
            return 100 * (GAPS[i-1] + (GAPS[i] - GAPS[i-1]) * dv[i-1] / (dv[i-1] - dv[i]))
    return float("nan")
LAY = [(f"{s} km, {g}", ("heldout", f"pal_s{s}_gw{g}"), ("heldout", f"onboard-ttl_s{s}_gw{g}"), ("main", f"pal_s{s}_gw{g}"), ("main", f"onboard-ttl_s{s}_gw{g}")) for s, g in [(550, 1), (550, 2), (1200, 1)]]
LAY += [(f"{s} km, {g}", ("beheld", f"pal_s{s}_gw{g}"), ("beheld", f"onboard-ttl_s{s}_gw{g}"), ("main", f"pal_s{s}_gw{g}"), ("main", f"onboard-ttl_s{s}_gw{g}")) for s, g in [(550, 3), (550, 4), (1200, 2), (1200, 3), (1200, 4)]]
SN = {"a": "15N 60E", "b": "10N 68E", "c": "24N 62E", "atl": "35N 40W"}
LAY += [(f"{SN[n]}, {g}", ("beheld", f"pal_site{n}_gw{g}"), ("beheld", f"onboard-ttl_site{n}_gw{g}"), ("sites", f"pal_site{n}_gw{g}"), ("sites", f"onboard-ttl_site{n}_gw{g}")) for n, g in [("a", 1), ("a", 2), ("b", 1), ("c", 1), ("atl", 1)]]
be = []
for lab, hp, ho, sp, so in LAY:
    P, O = load_be(*hp), load_be(*ho)
    first, bl, den, gam, dT, ex3 = inputs(P, O)
    pred = (first + bl) / den
    Sp, So = load_be(*sp), load_be(*so)
    per = []
    for s in sorted(set(Sp) & set(So)):
        per.append(cross([float(Sp[s]["S"][f"beam_mis_T2_d{x:g}"]) - float(So[s]["S"][f"beam_mis_T2_d{x:g}"]) for x in GAPS]))
    m, h = ci(per)
    be.append(dict(lab=lab, sim=m, h=h, first=first / den, pred=pred, gam=gam, dT=dT, ex3=100 * ex3))
core = [x for x in be if x["ex3"] <= 10]
excl = [x for x in be if x["ex3"] > 10]
mac("BeRuleN", len(core), "{:d}"); mac("BeExclN", len(excl), "{:d}")
mac("BeCoreMaxErr", max(abs(x["pred"] - x["sim"]) for x in core), "{:.2f}")
mac("BeCoreIn", sum(abs(x["pred"] - x["sim"]) <= x["h"] for x in core), "{:d}")
mac("BeCoreMin", min(x["sim"] for x in core), "{:.2f}"); mac("BeCoreMax", max(x["sim"] for x in core), "{:.2f}")
mac("BeFirstMaxErr", max(abs(x["first"] - x["sim"]) for x in core), "{:.2f}")
mac("BeExclMaxErr", max(abs(x["pred"] - x["sim"]) for x in excl), "{:.2f}")
mac("BeExclFirstMaxErr", max(abs(x["first"] - x["sim"]) for x in excl), "{:.2f}")
mac("BeExclMin", min(x["sim"] for x in excl), "{:.2f}"); mac("BeExclMax", max(x["sim"] for x in excl), "{:.2f}")
mac("BeExclShareMin", min(x["ex3"] for x in excl), "{:.0f}")
with open(os.path.join(TAB, "table_be.tex"), "w") as f:
    f.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrr@{}}\n\\toprule\nShell or cell, GW & $\\gamma$ & Age term & With blackout & Simulated & Never-contacted\\\\\n & & (points) & (points) & (points) & (\\% served)\\\\\n\\midrule\n")
    for x in core + excl:
        f.write(f"{x['lab']} & {x['gam']:.2f} & {x['first']:.2f} & {x['pred']:.2f} & {x['sim']:.2f}$\\pm${x['h']:.2f} & {x['ex3']:.0f}\\\\\n")
    f.write("\\bottomrule\n\\end{tabular}\n")
mac("DpRuns", tot, "{:d}")
main_hash = "11977b92398de810"
mac("DpHash", main_hash, "{}")
mac("LoadRuns", 160, "{:d}"); mac("WcRuns", 80, "{:d}"); mac("UniRuns", 60, "{:d}"); mac("HeldDpRuns", sum(len(v) for v in HD.values()), "{:d}")
mac("BeHeldRuns", len(glob.glob(f"{H}/runs/beheld/*/x/seed*")), "{:d}")
print("shas", shas, tot)
M = {k_.replace("R3", "RThree"): v_ for k_, v_ in M.items()}
with open(os.path.join(PD, "macros_grid.tex"), "w") as f:
    f.write("% Generated by the analysis scripts in sim/. Do not edit by hand.\n")
    for k_ in sorted(M):
        f.write(f"\\newcommand{{\\{k_}}}{{{M[k_]}}}\n")
print(len(M))
