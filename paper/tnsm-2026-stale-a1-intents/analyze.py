#!/usr/bin/env python3
"""Analysis for Paper 3 (TNSM). Reads runs/<group>/<config>/seedNN/*.csv and
writes ../results_macros.tex, ../tables/*.tex and ../figures/*.pdf.

Every number quoted in the manuscript is a macro written here. Means are over
the 10 seeds of a configuration; a 95% CI is the Student-t interval of the
seed means. A difference between two strategies is paired by seed (the same
seed gives the same epoch, orbit geometry, rain and ephemeris error for every
strategy), and its CI is the t interval of the per-seed differences.
"""
import csv
import glob
import math
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PAPER = os.path.dirname(HERE)
RUNS = os.environ.get("P3_RUNS", os.path.join(HERE, "runs"))
FIG = os.environ.get("P3_FIG", os.path.join(PAPER, "figures"))
TAB = os.environ.get("P3_TAB", os.path.join(PAPER, "tables"))
os.makedirs(FIG, exist_ok=True)
os.makedirs(TAB, exist_ok=True)

T975 = {2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571, 7: 2.447, 8: 2.365, 9: 2.306, 10: 2.262, 30: 2.045, 40: 2.023}

macros = {}


def mac(name, value, fmt="{:.2f}"):
    if isinstance(value, str):
        macros[name] = value
    elif value is None or (isinstance(value, float) and math.isnan(value)):
        macros[name] = "n/a"
    else:
        s = fmt.format(value)
        if "+" in fmt or s.startswith("-"):
            # unsigned magnitude, for use after a directional verb ("lowers ... by")
            # so that the text never prints a doubled sign (review R3 #7)
            macros[name + "Abs"] = fmt.replace("+", "").format(abs(value))
        if s.startswith("-"):
            # a negative typesets with a math minus; a rounded "-0.0" loses its sign (review m12)
            s = "\\ensuremath{-}" + s[1:] if any(ch in "123456789" for ch in s) else s[1:]
        macros[name] = s


def mac_range(name, values, fmt="{:.0f}", unit_each=""):
    """'a to b' over the formatted values, or a single value when every value
    formats the same (review R3 #7: no degenerate 'x to x' ranges). Signs are
    dropped; callers pass magnitudes or use a sign-free sentence."""
    f = sorted({fmt.format(abs(v)) for v in values}, key=float)
    macros[name] = f[0] + unit_each if len(f) == 1 else f"{f[0]} to {f[-1]}"


def load(group):
    """{config: {seed: {"summary": row, "outages": [...], "switches": [...]}}}"""
    out = defaultdict(dict)
    for d in sorted(glob.glob(os.path.join(RUNS, group, "*", "seed*"))):
        sp = os.path.join(d, "summary.csv")
        if not os.path.exists(sp):
            continue
        cfg = os.path.basename(os.path.dirname(d))
        seed = int(os.path.basename(d)[4:])
        rec = {"summary": next(csv.DictReader(open(sp)))}
        for k in ["outages", "switches", "checks", "nf_miss"]:
            p = os.path.join(d, f"{k}.csv")
            rec[k] = list(csv.DictReader(open(p))) if os.path.exists(p) else []
        lh = os.path.join(d, "loop_hist.csv")
        rec["hist"] = list(csv.DictReader(open(lh))) if os.path.exists(lh) else []
        out[cfg][seed] = rec
    return out


def col(data, cfg, key):
    v = []
    for s in sorted(data[cfg]):
        x = data[cfg][s]["summary"][key]
        try:
            x = float(x)
        except ValueError:
            x = float("nan")
        v.append(x)
    return np.array(v)


def ci(v):
    v = np.asarray([x for x in v if not math.isnan(x)])
    n = len(v)
    if n == 0:
        return float("nan"), float("nan"), 0
    m = float(np.mean(v))
    if n < 2:
        return m, float("nan"), n
    h = T975.get(n, 1.96) * float(np.std(v, ddof=1)) / math.sqrt(n)
    return m, h, n


def paired(data, a, b, key):
    """mean and CI half-width of (a - b), paired by seed."""
    seeds = sorted(set(data[a]) & set(data[b]))
    d = []
    for s in seeds:
        xa = float(data[a][s]["summary"][key])
        xb = float(data[b][s]["summary"][key])
        if not (math.isnan(xa) or math.isnan(xb)):
            d.append(xa - xb)
    return ci(d)


def pooled(data, cfg, kind, key, filt=lambda r: True):
    v = []
    for s in sorted(data[cfg]):
        for r in data[cfg][s][kind]:
            if filt(r):
                try:
                    v.append(float(r[key]))
                except ValueError:
                    pass
    return np.array(v)


REAL = lambda r: r.get("kind") not in ("0", "4")  # D2: no start-up, no censored outages


def real_outages_per_run(data, cfg):
    return np.array([sum(1 for r in data[cfg][s]["outages"] if REAL(r)) for s in sorted(data[cfg])], float)


def recon(data, cfg, key, scale=1.0):
    """Per-seed mean over reconnections (feeder back to the SAME satellite),
    excluding the start-up acquisition at t = 0. Returns per-seed values."""
    out = []
    for s in sorted(data[cfg]):
        v = []
        for r in data[cfg][s]["outages"]:
            if int(r["ended_by_switch"]) == 0 and r.get("kind", "2") in ("1", "2"):
                x = float(r[key])
                if key == "ttc" and x < 0:
                    continue
                v.append(x * scale)
        if v:
            out.append(float(np.mean(v)))
    return np.array(out)


def blackout(data, cfg, stat="mean"):
    """Per-seed blackout statistic in ms over satellite switches, excluding the
    start-up 'switch' at t = 0 (initial acquisition)."""
    out = []
    for s in sorted(data[cfg]):
        v = [float(r["blackout_nf_s"]) * 1000 for r in data[cfg][s]["switches"]
             if float(r["t"]) >= 1.0 and float(r["blackout_nf_s"]) >= 0]
        if v:
            out.append(float(np.mean(v)) if stat == "mean" else float(np.percentile(v, 95)))
    return np.array(out)


def recon_count(data, cfg):
    n = 0
    for s in data[cfg]:
        n += sum(1 for r in data[cfg][s]["outages"]
                 if int(r["ended_by_switch"]) == 0 and r.get("kind", "2") in ("1", "2"))
    return n


def tex_num(x, nd=2):
    return f"{x:.{nd}f}"


def write_macros():
    p = os.environ.get("P3_MACROS", os.path.join(PAPER, "results_macros.tex"))
    with open(p, "w") as f:
        f.write("% Generated by the analysis scripts in sim/. Do not edit by hand.\n")
        for k in sorted(macros):
            f.write(f"\\newcommand{{\\{k}}}{{{macros[k]}}}\n")
    print(f"wrote {len(macros)} macros -> {p}")


# ========== plotting --
def plt_setup():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.style.use("/home/uzair/6g_ntn_ns3/papers/templates/ieee.mplstyle")
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["mathtext.fontset"] = "stix"  # serif math to match the Times text (review R3 #13)
    plt.rcParams["ps.fonttype"] = 42
    return plt


NAVY, BLUE, GOLD, GRAY, STEEL, RED = "#0E3A6B", "#2E78B5", "#C98A0B", "#4A4A4A", "#7FA3CC", "#C23B22"
STRAT_STYLE = {
    "ground": dict(color=GRAY, marker="s", ls=":", label="Ground only"),
    "onboard": dict(color=STEEL, marker="^", ls="-.", label="On-board only"),
    "onboard-ttl": dict(color="#6B8E23", marker="P", ls=(0, (3, 1)), label="O+L"),
    "reactive": dict(color=BLUE, marker="o", ls="--", label="Reactive hybrid"),
    "reactive-ttl": dict(color=NAVY, marker="v", ls="-.", label="R+L"),
    "reactive-ttl-pre": dict(color="#111111", marker="x", ls=(0, (5, 1, 1, 1)), label="R+L+P"),
    "pal": dict(color=GOLD, marker="D", ls="-", label="PAL"),
    "sat-pred": dict(color=RED, marker="*", ls=(0, (1, 1)), label="SSP"),
}
STRATS = ["ground", "onboard", "onboard-ttl", "reactive-ttl", "reactive-ttl-pre", "pal"]          # figures
STRATS_ALL = ["ground", "onboard", "onboard-ttl", "reactive", "reactive-ttl", "reactive-ttl-pre", "pal",
              "sat-pred"]
SNM = {"ground": "Gnd", "onboard": "Onb", "onboard-ttl": "Obl", "reactive": "Rea", "reactive-ttl": "Ttl",
       "reactive-ttl-pre": "Tpr", "pal": "Pal", "sat-pred": "Ssp"}
# NF beam-tracking outcome (review D1, D4): dwell times and on-board accuracy gaps
DWELL = [0.5, 1.0, 2.0, 5.0]
GAPS = [0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.05, 0.1]
T_DEF, G_DEF = 2.0, 0.015  # default dwell; default gap = INT8 loss of ResNet-50 (Jacob et al.)
DWN = {0.5: "Half", 1.0: "One", 2.0: "Two", 5.0: "Five"}


def bcol(T, gp):
    return f"beam_mis_T{T:g}_d{gp:g}"


def sign_p(d):
    """two-sided exact sign test on paired differences (ties dropped)"""
    from math import comb
    d = [x for x in d if abs(x) > 1e-12]
    n = len(d)
    if n == 0:
        return 1.0
    k = sum(1 for x in d if x > 0)
    k = max(k, n - k)
    return min(1.0, 2 * sum(comb(n, j) for j in range(k, n + 1)) / 2 ** n)


def wilcoxon_p(d):
    from scipy.stats import wilcoxon
    d = [x for x in d if abs(x) > 1e-12]
    if len(d) < 2:
        return 1.0
    return float(wilcoxon(d).pvalue)


def paired_vec(data, a, b, key, data_b=None):
    db = data if data_b is None else data_b
    seeds = sorted(set(data[a]) & set(db[b]))
    return np.array([float(data[a][s]["summary"][key]) - float(db[b][s]["summary"][key]) for s in seeds])
WORD = {1: "one", 2: "two", 3: "three", 4: "four"}
ROMAN = {550: "Five", 1200: "Twelve"}


# ========== groups --
def analyze_hb(plt):
    data = load("hb")
    if not data:
        return
    hbs = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0]
    fig, ax = plt.subplots(figsize=(3.45, 2.45))
    gw = 1
    for strat in ["reactive", "pal"]:
        xs, ys, es = [], [], []
        for hb in hbs:
            c = f"{strat}_s550_gw{gw}_hb{hb}"
            if c not in data:
                continue
            m, h, _ = ci((1.0 - col(data, c, "nf_hit_ratio")) * 1e2)
            k, _, _ = ci(col(data, c, "hb_kbps"))
            xs.append(k); ys.append(m); es.append(h)
        st = STRAT_STYLE[strat]
        ax.errorbar(xs, ys, yerr=es, color=st["color"], marker=st["marker"], ls=st["ls"],
                    label=st["label"], capsize=2, lw=1.0)
        for x, y, hb in zip(xs, ys, hbs):
            if strat == "reactive" and hb in (0.05, 2.0):
                ax.annotate(f"$P_{{\\mathrm{{hb}}}}$={int(hb*1000)} ms", (x, y), textcoords="offset points",
                            xytext=(6, -2) if hb == 2.0 else (-30, 8), fontsize=7)
    ax.set_xscale("log")
    ax.set_xlabel("Heartbeat load on the feeder (kb/s)")
    ax.set_ylabel("Near-RT deadline misses (%)")
    ax.legend(loc="upper right", frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_hb_tradeoff.pdf"))
    plt.close(fig)
    # macros at the main-grid heartbeat (0.1 s) and the slowest (2 s), gw 1
    for strat in ["reactive", "pal"]:
        for hb, tag in [(0.1, "Fast"), (2.0, "Slow")]:
            c = f"{strat}_s550_gw1_hb{hb}"
            if c in data:
                m, h, n = ci((1 - col(data, c, "nf_hit_ratio")) * 100)
                mac(f"Hb{strat.capitalize()}{tag}Miss", m, "{:.2f}")
                mac(f"Hb{strat.capitalize()}{tag}MissCi", h, "{:.2f}")
                mac(f"Hb{strat.capitalize()}{tag}Kbps", ci(col(data, c, "hb_kbps"))[0], "{:.1f}")
    HBN = {0.05: "A", 0.1: "B", 0.2: "C", 0.5: "D", 1.0: "E", 2.0: "F"}
    for strat in ["reactive", "pal"]:
        for hb, t in HBN.items():
            c = f"{strat}_s550_gw1_hb{hb}"
            if c in data:
                mac(f"Hb{strat.capitalize()}{t}Miss", ci((1 - col(data, c, "nf_hit_ratio")) * 100)[0], "{:.2f}")
                mac(f"Hb{strat.capitalize()}{t}Kbps", ci(col(data, c, "hb_kbps"))[0], "{:.1f}")
    mac("HbRuns", sum(len(v) for v in data.values()), "{:d}")
    c1, c2 = "pal_s550_gw1_hb2.0", "reactive_s550_gw1_hb2.0"
    if c1 in data and c2 in data:
        m, h, n = paired(data, c2, c1, "nf_hit_ratio")
        mac("HbSlowGainPts", -m * 100, "{:.2f}")
        mac("HbSlowGainPtsCi", h * 100, "{:.2f}")


def rates_from(src, cfg, seeds=None):
    """Per-state rates of the extended cost model, pooled over seeds:
    p_c, p_v, p_s (violation while holding the current, a superseded, the safe
    intent) and o_c, o_v, Delta (over-provisioning in the same states)."""
    tot = defaultdict(float)
    for sd in (sorted(src[cfg]) if seeds is None else seeds):
        for k, v in src[cfg][sd]["checks"][0].items():
            tot[k] += float(v)
    r = {}
    for nm, st_ in [("c", "current"), ("v", "stale"), ("s", "fallback")]:
        n = max(1.0, tot[f"samp_{st_}"])
        r["p" + nm] = tot[f"viol_{st_}"] / n
        r["o" + nm] = tot[f"over_{st_}"] / n
    return r


def eps_ext(r, ratio):
    """extended Lemma 1: eps' = [(p_s-p_c) + w(Delta-o_c)] / [(p_v-p_c) + w(o_v-o_c)], w = w_o/w_v"""
    return ((r["ps"] - r["pc"]) + ratio * (r["os"] - r["oc"])) / ((r["pv"] - r["pc"]) + ratio * (r["ov"] - r["oc"]))


def ratio_for(r, target):
    """weight ratio w_o/w_v at which the extended rule predicts eps' = target"""
    return (target * (r["pv"] - r["pc"]) - (r["ps"] - r["pc"])) / ((r["os"] - r["oc"]) - target * (r["ov"] - r["oc"]))


def analyze_eps(plt, main):
    data = load("eps")
    if not data:
        return
    # eps = 0.2 is the main-grid default: alias those runs
    for strat in ["pal", "reactive-ttl", "reactive-ttl-pre", "onboard-ttl"]:
        for gw in [1, 2]:
            c0 = f"{strat}_s550_gw{gw}"
            if c0 in main:
                data[f"{strat}_s550_gw{gw}_eps0.2"] = main[c0]
    epss = [0.05, 0.2, 0.5, 0.9, 0.99]
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.15))
    for ax, gw in zip(axes, [1, 2]):
        for strat, lab in [("pal", "PAL"),
                           ("reactive-ttl-pre", "R+L+P"),
                           ("reactive-ttl", "R+L"),
                           ("onboard-ttl", "O+L")]:
            xs, ys, exs, eys = [], [], [], []
            grid = sorted(set(epss) | ({0.1, 0.3, 0.4, 0.7} if (strat == "pal" and gw == 1) else set()))
            gl = []
            for eps in grid:
                c = f"{strat}_s550_gw{gw}_eps{eps}"
                if c not in data:
                    continue
                mx, hx, _ = ci(col(data, c, "overprov_mean") * 100)
                my, hy, _ = ci(col(data, c, "sla_viol_frac") * 100)
                xs.append(mx); ys.append(my); exs.append(hx); eys.append(hy); gl.append(eps)
            st = STRAT_STYLE[strat]
            ax.errorbar(xs, ys, xerr=exs, yerr=eys, color=st["color"], marker=st["marker"], ms=3.5,
                        ls=st["ls"], capsize=1.5, lw=0.9, label=lab)
            if strat == "pal":
                for x, y, eps in zip(xs, ys, gl):
                    if eps in (0.05, 0.99):
                        off = (-4, 9) if eps == 0.05 else (7, 9)
                        ax.annotate(rf"$\varepsilon$={eps}", (x, y), textcoords="offset points",
                                    xytext=off, fontsize=7, ha="right" if eps == 0.05 else "left")
        if gw == 1:
            var_ = load("variants")
            xs, ys = [], []
            for eps in [0.05, 0.1, 0.2, 0.3, 0.5, 0.9]:
                c = "pal_s550_gw1_hedge" if eps == 0.2 else f"pal_s550_gw1_hedge_eps{eps}"
                if c in var_:
                    xs.append(ci(col(var_, c, "overprov_mean") * 100)[0])
                    ys.append(ci(col(var_, c, "sla_viol_frac") * 100)[0])
            if xs:
                ax.plot(xs, ys, color="0.55", marker="d", ms=3, ls=":", lw=0.9, label=r"PAL, $g_0$ = 0.6")
        for strat in ["ground", "onboard", "reactive"]:
            c = f"{strat}_s550_gw{gw}"
            if c in main:
                mx, hx, _ = ci(col(main, c, "overprov_mean") * 100)
                my, hy, _ = ci(col(main, c, "sla_viol_frac") * 100)
                st = STRAT_STYLE[strat]
                ax.errorbar([mx], [my], xerr=[hx], yerr=[hy], color=st["color"], marker=st["marker"],
                            ls="none", capsize=2, label=st["label"] + ", no lease" if strat != "ground" else st["label"])
        ax.set_xlabel("Over-provisioning (% of cell)")
        ax.set_ylabel("Shortfall time (%)")
        ax.set_title(f"({'ab'[gw-1]}) {WORD[gw]} gateway{'s' if gw > 1 else ''}", fontsize=8)
        ax.legend(loc="upper right", frameon=False, fontsize=6.0)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_lease_tradeoff.pdf"))
    plt.close(fig)
    mac("EpsRuns", sum(len(v) for k, v in data.items() if not k.endswith("eps0.2")), "{:d}")
    ET = {0.05: "Lo", 0.2: "Mid", 0.5: "Half", 0.9: "Ninety", 0.99: "Hi"}
    for strat in ["pal", "reactive-ttl", "reactive-ttl-pre", "onboard-ttl"]:
        st = SNM[strat]
        for eps, tag in ET.items():
            c = f"{strat}_s550_gw1_eps{eps}"
            if c in data:
                mac(f"Eps{st}{tag}Viol", ci(col(data, c, "sla_viol_frac") * 100)[0], "{:.1f}")
                mac(f"Eps{st}{tag}Over", ci(col(data, c, "overprov_mean") * 100)[0], "{:.2f}")
                mac(f"Eps{tag}Tau", ci(col(data, c, "lease_tau_s"))[0], "{:.1f}")
        # single-factor PAL vs O+L along the sweep
    for eps, tag in ET.items():
        a, o = f"pal_s550_gw1_eps{eps}", f"onboard-ttl_s550_gw1_eps{eps}"
        if a in data and o in data:
            m, h, _ = ci(paired_vec(data, a, o, "sla_viol_frac") * 100)
            mac(f"EpsDViolPalObl{tag}", m, "{:+.2f}")
            mac(f"EpsDViolPalObl{tag}Ci", h, "{:.2f}")

    # == realized-cost check of the (extended) lease rule (review D5-D8).
    # Rates come from HELD-OUT runs (seeds 11-20) of the leased reactive hybrid;
    # the realized cost of the scored PAL runs (seeds 1-10) is
    # J = shortfall time + (w_o/w_v) over-provisioning.
    held = load("heldout")
    var = load("variants")
    a1p = load("a1proc")
    mac("EstHeldOut", "yes" if held else "no", "{}")
    grid = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.9, 0.99]
    rng = np.random.default_rng(20261019)
    lines = []

    def cost_check(tag, src_scored, cfg_fmt, grid_, rates, targets, rates_boot=None, ratio_of=None, rng_=None):
        """returns per target: argmin of the seed-mean cost, bootstrap P(argmin = target),
        P(argmin within one grid step), largest excess at a grid point and its sign test.
        ratio_of(rates) gives the weight ratio w_o/w_v; by default the ratio at which the
        rule predicts the target from these rates."""
        for target, tg in targets:
            rg = rng if rng_ is None else rng_
            rof = ratio_of if ratio_of is not None else (lambda rr, target=target: ratio_for(rr, target))
            ratio = rof(rates)
            J = {}
            for eps in grid_:
                c = cfg_fmt.format(eps=eps)
                if c in src_scored:
                    J[eps] = col(src_scored, c, "sla_viol_frac") + ratio * col(src_scored, c, "overprov_mean")
            if target not in J or len(J) < 3:
                continue
            es = sorted(J)
            best = min(es, key=lambda e: float(np.mean(J[e])))
            n = len(J[target])
            M = np.array([J[e] for e in es])  # grid x seeds
            B = 10000
            hits = 0; near = 0
            ti = es.index(target)
            cnt = defaultdict(int)
            for _ in range(B):
                idx = rg.integers(0, n, n)
                if rates_boot is not None:
                    rb = rates_boot()
                    rat_b = rof(rb)
                    Mb = np.array([col(src_scored, cfg_fmt.format(eps=e), "sla_viol_frac")[idx] +
                                   rat_b * col(src_scored, cfg_fmt.format(eps=e), "overprov_mean")[idx] for e in es])
                    am = int(np.argmin(Mb.mean(axis=1)))
                else:
                    am = int(np.argmin(M[:, idx].mean(axis=1)))
                cnt[es[am]] += 1
                hits += am == ti
                near += abs(am - ti) <= 1
            diffs = []
            for e in es:
                if e == target:
                    continue
                m_, h_, _ = ci(J[e] - J[target])
                diffs.append((e, m_, h_))
            ew, mw, hw = max(diffs, key=lambda d: d[1])
            dv = J[ew] - J[target]
            excess_best = float(np.mean(J[target]) - np.mean(J[best]))
            mac(f"Cost{tag}Target{tg}", target, "{:.2f}")
            mac(f"Cost{tag}Ratio{tg}", ratio, "{:.2f}")
            mac(f"Cost{tag}Argmin{tg}", best, "{:g}")
            mac(f"Cost{tag}Boot{tg}", 100 * hits / B, "{:.0f}")
            mac(f"Cost{tag}BootNear{tg}", 100 * near / B, "{:.0f}")
            mac(f"Cost{tag}ExcessBest{tg}", 100 * excess_best, "{:.3f}")
            mac(f"Cost{tag}ExcessBestRel{tg}", 100 * excess_best / float(np.mean(J[target])), "{:.1f}")
            mac(f"Cost{tag}WorstEps{tg}", ew, "{:g}")
            mac(f"Cost{tag}WorstD{tg}", mw * 100, "{:.2f}")
            mac(f"Cost{tag}WorstDCi{tg}", hw * 100, "{:.2f}")
            mac(f"Cost{tag}WorstSignP{tg}", sign_p(dv), "{:.3f}")
            mac(f"Cost{tag}WorstPos{tg}", int(np.sum(dv > 0)), "{:d}")
            lines.append(f"[{tag}] target {target} ratio {ratio:.3f} argmin {best} boot P(argmin=target) "
                         f"{hits/B:.3f} P(within 1) {near/B:.3f} dist " +
                         ", ".join(f"{e:g}:{cnt[e]/B:.3f}" for e in es) + " | " +
                         "; ".join(f"{e:g}: {m_*100:+.3f}+-{h_*100:.3f}" for e, m_, h_ in diffs))

    def held_rates(cfg, rg=None):
        rg = rng if rg is None else rg
        if cfg not in held:
            return None, None
        r = rates_from(held, cfg)
        seeds = sorted(held[cfg])
        def boot():
            return rates_from(held, cfg, list(rg.choice(seeds, len(seeds))))
        return r, boot

    r0, b0 = held_rates("reactive-ttl_s550_gw1_eps0.2")
    if r0:
        for k, v in r0.items():
            mac(f"EstRate{k.capitalize()}", v * 100, "{:.2f}")
        mac("EstPv", r0["pv"], "{:.2f}")
        mac("EstDelta", r0["os"] * 100, "{:.1f}")
        pvs = []
        for sd in held["reactive-ttl_s550_gw1_eps0.2"]:
            ck = held["reactive-ttl_s550_gw1_eps0.2"][sd]["checks"][0]
            if float(ck["samp_stale"]) > 0:
                pvs.append(float(ck["viol_stale"]) / float(ck["samp_stale"]))
        mac("EstPvMin", min(pvs), "{:.2f}")
        mac("EstPvMax", max(pvs), "{:.2f}")
        # what the original (two-term) lemma predicts at the weights where the
        # extended rule predicts 0.2 and 0.5, and the reverse
        for target, tg in [(0.2, "A"), (0.5, "B")]:
            w_simple = target * r0["pv"] / r0["os"]
            mac(f"ExtEpsAtSimple{tg}", eps_ext(r0, w_simple), "{:.3f}")
            mac(f"ExtTauAtSimple{tg}", -60 * math.log(1 - eps_ext(r0, w_simple)), "{:.1f}")
            mac(f"SimpleTau{tg}", -60 * math.log(1 - target), "{:.1f}")
            mac(f"SimpleRatio{tg}", w_simple, "{:.3f}")
        mac("ExtEpsZeroW", eps_ext(r0, 0.0), "{:+.3f}")
        cost_check("", data, "pal_s550_gw1_eps{eps}", grid, r0, [(0.2, "A"), (0.5, "B")], b0)
        mac("CostGridN", len(grid), "{:d}")
    # hedge fallback: g0 = 0.6, above every version of the alphabet
    re_, be_ = held_rates("reactive-ttl_s550_gw1_hedge_eps0.2")
    if re_ and var:
        mac("EstDeltaHedge", re_["os"] * 100, "{:.1f}")
        mac("EstPsHedge", re_["ps"] * 100, "{:.2f}")
    # non-Poisson intent updates (D8): the rule still assumes Poisson
    for proc, ptag in [("periodic", "Per"), ("bursty", "Bur")]:
        rp, bp = held_rates(f"reactive-ttl_s550_gw1_{proc}_eps0.2")
        if rp and a1p:
            mac(f"EstPv{ptag}", rp["pv"], "{:.2f}")
            cost_check(ptag, a1p, "pal_s550_gw1_" + proc + "_eps{eps}", [0.05, 0.1, 0.2, 0.3, 0.5, 0.9], rp,
                       [(0.2, "A"), (0.5, "B")], bp)
            for eps in [0.2]:
                c = f"pal_s550_gw1_{proc}_eps{eps}"
                if c in a1p:
                    mac(f"Aone{ptag}Viol", ci(col(a1p, c, "sla_viol_frac") * 100)[0], "{:.1f}")
                    mac(f"Aone{ptag}Over", ci(col(a1p, c, "overprov_mean") * 100)[0], "{:.2f}")
    if a1p:
        mac("AoneRuns", sum(len(v) for v in a1p.values()), "{:d}")
    if held:
        mac("HeldRuns", sum(len(v) for v in held.values()), "{:d}")
    with open(os.path.join(TAB, "cost_check.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")


def analyze_main(plt):
    data = load("main")
    if not data:
        return data
    # == scenario description macros
    for shell in [550, 1200]:
        for gw in [1, 2, 3, 4]:
            c = f"pal_s{shell}_gw{gw}"
            if c not in data:
                continue
            tag = f"{ROMAN[shell]}G{WORD[gw].capitalize()}"
            m, h, n = ci(col(data, c, "feeder_avail") * 100)
            mac(f"Avail{tag}", m, "{:.1f}")
            mac(f"Avail{tag}Ci", h, "{:.1f}")
            mac(f"Switches{tag}", ci(col(data, c, "switches"))[0], "{:.1f}")
            mac(f"Outages{tag}", ci(real_outages_per_run(data, c))[0], "{:.1f}")
            # share of outages that end with a satellite switch rather than a
            # reconnection to the same satellite
            ob = pooled(data, c, "outages", "ended_by_switch", REAL)
            mac(f"OutBySwitch{tag}", 100 * float(np.mean(ob)) if len(ob) else float("nan"), "{:.0f}")
            dur = pooled(data, c, "outages", "dur", REAL)
            mac(f"OutDurMed{tag}", float(np.median(dur)) if len(dur) else float("nan"), "{:.0f}")
            pr = pooled(data, c, "outages", "predicted", REAL)
            mac(f"OutPred{tag}", 100 * float(np.mean(pr)) if len(pr) else float("nan"), "{:.0f}")
    # == feeder statistics table
    with open(os.path.join(TAB, "table_feeder.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n")
        t.write("\\begin{tabular}{@{}lrrrrrr@{}}\n\\toprule\n")
        t.write("Shell, GW & Avail. & Outages & Median & Predict. & End by & Switches\\\\\n")
        t.write(" & (\\%) & (/h) & (s) & (\\%) & switch (\\%) & (/h)\\\\\n\\midrule\n")
        for shell in [550, 1200]:
            for gw in [1, 2, 3, 4]:
                c = f"pal_s{shell}_gw{gw}"
                if c not in data:
                    continue
                av, ah, _ = ci(col(data, c, "feeder_avail") * 100)
                ou = ci(real_outages_per_run(data, c))[0]
                dur = pooled(data, c, "outages", "dur", REAL)
                pr = pooled(data, c, "outages", "predicted", REAL)
                ob = pooled(data, c, "outages", "ended_by_switch", REAL)
                sw = ci(col(data, c, "switches"))[0]
                t.write(f"{shell} km, {gw} & {av:.1f}$\\pm${ah:.1f} & {ou:.1f} & {np.median(dur):.1f} & "
                        f"{100*np.mean(pr):.0f} & {100*np.mean(ob):.0f} & {sw:.1f}\\\\\n")
        t.write("\\bottomrule\n\\end{tabular}\n")
    fa = {c: sum(int(float(data[c][sd]["summary"]["false_autonomy"])) for sd in data[c]) for c in data}
    bad = {c: n for c, n in fa.items() if n > 0}
    mac("FalseAutonomyTotal", sum(fa.values()), "{:d}")
    if bad:
        print("WARNING false autonomy:", bad)
    n_runs = sum(len(v) for v in data.values())
    mac("MainRuns", n_runs, "{:d}")
    mac("MainConfigs", len(data), "{:d}")
    mac("SimHours", 1.0, "{:.0f}")
    mx = max(float(data[c][s]["summary"]["max_el_err_deg"]) for c in data for s in data[c])
    mac("MaxElErr", mx, "{:.3f}")
    fails = sum(int(float(data[c][s]["summary"]["kpm_codec_fail"])) for c in data for s in data[c])
    mac("CodecFails", fails, "{:d}")
    kpm = sum(int(float(data[c][s]["summary"]["kpm_delivered"])) for c in data for s in data[c])
    mac("KpmDecoded", f"{kpm:,}".replace(",", "{,}"))
    rtt = sum(int(data[c][s]["checks"][0]["rt_ticks"]) for c in data if c.startswith("pal") for s in data[c] if data[c][s]["checks"])
    rtg = sum(int(data[c][s]["checks"][0]["rt_ground_feasible_ticks"]) for c in data if c.startswith("pal") for s in data[c] if data[c][s]["checks"])
    mac("RtGroundFeasPct", 100.0 * rtg / max(1, rtt), "{:.1f}")
    sats = [float(data[c][s]["summary"]["sats_in_set"]) for c in data for s in data[c] if "_s550_" in c]
    mac("SatsServedFive", float(np.mean(sats)), "{:.0f}")

    # == Fig: RT hit ratio and near-RT misses vs gateways
    fig, axes = plt.subplots(2, 3, figsize=(7.1, 3.15), sharex=True)
    for r, shell in enumerate([550, 1200]):
        for k, (cls, lab) in enumerate([("rt", "RT class, 10 ms"), ("nf", "NF class, 100 ms"),
                                         ("ns", "NS class, 1 s")]):
            ax = axes[r, k]
            for si, strat in enumerate(STRATS + ["sat-pred"]):
                if cls != "rt" and strat in ("onboard", "onboard-ttl"):
                    continue  # zero misses in every run: nothing to draw on a log axis
                xs, ys, es = [], [], []
                for gw in [1, 2, 3, 4]:
                    c = f"{strat}_s{shell}_gw{gw}"
                    if c not in data:
                        continue
                    v = col(data, c, f"{cls}_hit_ratio") * 100
                    if cls != "rt":
                        v = 100 - v
                    m, h, _ = ci(v)
                    # small horizontal offset per strategy so that curves at the
                    # same value (100 % for on-board hosting) stay visible (D26)
                    xs.append(gw + 0.06 * (si - 3.0)); ys.append(m); es.append(h)
                st = STRAT_STYLE[strat]
                ax.errorbar(xs, ys, yerr=es, color=st["color"], marker=st["marker"], ls=st["ls"],
                            label=st["label"], capsize=2, lw=1.0, ms=3.5)
            ax.set_title(f"({'abcdef'[3*r+k]}) {shell} km, {lab}", fontsize=8)
            if cls == "rt":
                ax.set_ylabel("Deadline-hit ratio (%)")
                ax.set_ylim(-5, 105)
            else:
                ax.set_yscale("log")
                from matplotlib.ticker import LogLocator, LogFormatterMathtext
                ax.yaxis.set_major_locator(LogLocator(base=10, numticks=6))
                ax.yaxis.set_major_formatter(LogFormatterMathtext())
                ax.set_ylim(0.05, 40)
                ax.set_ylabel("Deadline misses (%)")
            if r == 1:
                ax.set_xlabel("Gateways")
            ax.set_xticks([1, 2, 3, 4])
    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=4, frameon=False, fontsize=7)
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(os.path.join(FIG, "fig_placement_map.pdf"))
    plt.close(fig)

    # == macros for every strategy x shell x gw on the core metrics
    keys = {
        "rt_hit_ratio": ("RtHit", 100, "{:.1f}"),
        "nf_hit_ratio": ("NfHit", 100, "{:.2f}"),
        "ns_hit_ratio": ("NsHit", 100, "{:.2f}"),
        "nf_p99_ms": ("NfPnn", 1, "{:.1f}"),
        "rt_p99_ms": ("RtPnn", 1, "{:.1f}"),
        "sla_viol_frac": ("Viol", 100, "{:.1f}"),
        "short_depth_mean": ("Depth", 100, "{:.2f}"),
        "short_depth_cond": ("DepthCond", 100, "{:.1f}"),
        "overprov_mean": ("Over", 100, "{:.2f}"),
        "superseded_frac": ("Super", 100, "{:.1f}"),
        "stale_mean_s": ("Stale", 1, "{:.1f}"),
        "blackout_mean_s": ("Black", 1000, "{:.0f}"),
        "blackout_p95_s": ("BlackPnf", 1000, "{:.0f}"),
        "ttc_mean_s": ("Ttc", 1000, "{:.0f}"),
        "conflicts_per_outage": ("Conf", 1, "{:.2f}"),
        "policy_conflicts_per_outage": ("PConf", 1, "{:.2f}"),
        "feeder_kbps": ("Kbps", 1, "{:.1f}"),
        "sig_bytes_per_switch": ("SigB", 1, "{:.0f}"),
        "sig_msgs_per_switch": ("SigM", 1, "{:.1f}"),
        "fallback_frac": ("Fallb", 100, "{:.1f}"),
        "nf_onboard_frac": ("NfOnb", 100, "{:.1f}"),
        "ns_onboard_frac": ("NsOnb", 100, "{:.1f}"),
    }
    SN = SNM
    for strat in STRATS_ALL:
        for shell in [550, 1200]:
            for gw in [1, 2, 3, 4]:
                c = f"{strat}_s{shell}_gw{gw}"
                if c not in data:
                    continue
                tag = f"{SN[strat]}{ROMAN[shell]}G{WORD[gw].capitalize()}"
                for k, (nm, sc, fm) in keys.items():
                    m, h, n = ci(col(data, c, k) * sc)
                    mac(f"{nm}{tag}", m, fm)
                    mac(f"{nm}{tag}Ci", h, fm)
                m, h, n = ci((1 - col(data, c, "nf_hit_ratio")) * 100)
                mac(f"NfMissPct{tag}", m, "{:.2f}")
    # reconnection metrics from outages.csv (start-up acquisition excluded)
    for strat in STRATS_ALL:
        for shell in [550, 1200]:
            for gw in [1, 2, 3, 4]:
                c = f"{strat}_s{shell}_gw{gw}"
                if c not in data:
                    continue
                tag = f"{SN[strat]}{ROMAN[shell]}G{WORD[gw].capitalize()}"
                for key, nm, sc, fm in [("ttc", "Ttc", 1000, "{:.0f}"),
                                        ("direct_conflicts", "Conf", 1, "{:.2f}"),
                                        ("policy_stale", "PConf", 1, "{:.2f}")]:
                    m, h, n = ci(recon(data, c, key, sc))
                    mac(f"{nm}{tag}", m, fm)
                    mac(f"{nm}{tag}Ci", h, fm)
                mac(f"Recon{tag}", recon_count(data, c), "{:d}")
                # pooled event counts over all reconnections (review m6)
                ev = [r for sd in data[c] for r in data[c][sd]["outages"]
                      if int(r["ended_by_switch"]) == 0 and r.get("kind", "2") in ("1", "2")]
                for key, nm in [("direct_conflicts", "Conf"), ("policy_stale", "PConf"), ("fallback", "FbW")]:
                    k_ = int(sum(float(r.get(key, 0) or 0) for r in ev))
                    mac(f"{nm}N{tag}", k_, "{:d}")
                    mac(f"{nm}Pool{tag}", k_ / len(ev) if ev else float("nan"), "{:.2f}")
                mac(f"MergeApplied{tag}", int(sum(float(data[c][sd]["summary"].get("merge_ground_applied", 0) or 0)
                                                  for sd in data[c])), "{:d}")
                for stt, nm in [("mean", "Black"), ("p95", "BlackPnf")]:
                    m, h, n = ci(blackout(data, c, stt))
                    mac(f"{nm}{tag}", m, "{:.0f}")
                    mac(f"{nm}{tag}Ci", h, "{:.0f}")

    # evidence that PAL's procedures actually ran (550 km, one gateway)
    for shell in [550, 1200]:
        c = f"pal_s{shell}_gw1"
        if c in data:
            tg = f"{ROMAN[shell]}GOne"
            mac(f"PalLeases{tg}", ci(col(data, c, "leases"))[0], "{:.1f}")
            mac(f"PalPreprov{tg}", ci(col(data, c, "preprov_frac") * 100)[0], "{:.0f}")
            mac(f"PalAutonomy{tg}", ci(col(data, c, "autonomy_entries"))[0], "{:.1f}")
            mac(f"PalFalseAut{tg}", float(np.sum(col(data, c, "false_autonomy"))), "{:.0f}")
            sp = f"sat-pred_s{shell}_gw1"
            if sp in data:
                mac(f"SspEntries{tg}", ci(col(data, sp, "sat_pred_entries"))[0], "{:.1f}")
                mac(f"SspAutonomy{tg}", ci(col(data, sp, "autonomy_entries"))[0], "{:.1f}")
                mac(f"SspFalseAut{tg}", float(np.sum(col(data, sp, "false_autonomy"))), "{:.0f}")
                mac(f"PalLeaseKbps{tg}", ci(col(data, c, "lease_kbps"))[0], "{:.2f}")
                mac(f"SspLeaseKbps{tg}", ci(col(data, sp, "lease_kbps"))[0], "{:.2f}")
            r = f"reactive_s{shell}_gw1"
            mac(f"ReaAutonomy{tg}", ci(col(data, r, "autonomy_entries"))[0], "{:.1f}")
            mac(f"ReaFalseAut{tg}", float(np.sum(col(data, r, "false_autonomy"))), "{:.0f}")

    # paired PAL - reactive differences (closest baseline)
    for shell in [550, 1200]:
        for gw in [1, 2, 3, 4]:
            a, b = f"pal_s{shell}_gw{gw}", f"reactive_s{shell}_gw{gw}"
            if a not in data or b not in data:
                continue
            tag = f"{ROMAN[shell]}G{WORD[gw].capitalize()}"
            for k, nm, sc, fm in [("sla_viol_frac", "DViol", 100, "{:.1f}"),
                                  ("overprov_mean", "DOver", 100, "{:.2f}"),
                                  ("nf_hit_ratio", "DNf", 100, "{:.2f}"),
                                  ("superseded_frac", "DSuper", 100, "{:.1f}"),
                                  ("feeder_kbps", "DKbps", 1, "{:.1f}")]:
                m, h, n = paired(data, a, b, k)
                mac(f"{nm}{tag}", m * sc, fm)
                mac(f"{nm}{tag}Ci", h * sc, fm)
            bt = f"reactive-ttl_s{shell}_gw{gw}"
            if bt in data:
                for k, nm, sc, fm in [("sla_viol_frac", "DlViol", 100, "{:.1f}"),
                                      ("short_depth_mean", "DlDepth", 100, "{:.2f}"),
                                      ("overprov_mean", "DlOver", 100, "{:.2f}")]:
                    m, h, n = paired(data, bt, b, k)
                    mac(f"{nm}{tag}", m * sc, fm)
                    mac(f"{nm}{tag}Ci", h * sc, fm)
                for k, nm, sc, fm in [("sla_viol_frac", "DtViol", 100, "{:.1f}"),
                                      ("overprov_mean", "DtOver", 100, "{:.2f}"),
                                      ("nf_hit_ratio", "DtNf", 100, "{:.2f}"),
                                      ("superseded_frac", "DtSuper", 100, "{:.1f}")]:
                    m, h, n = paired(data, a, bt, k)
                    mac(f"{nm}{tag}", m * sc, fm)
                    mac(f"{nm}{tag}Ci", h * sc, fm)
            ra = ci(col(data, b, "sla_viol_frac"))[0]
            pa = ci(col(data, a, "sla_viol_frac"))[0]
            mac(f"ViolRel{tag}", 100 * (ra - pa) / ra if ra > 0 else float("nan"), "{:.0f}")

    # == main table: shell x gw in {1,2} x strategy
    rows = []
    for shell, gw in [(550, 1), (550, 2), (1200, 1)]:
        if True:
            # SSP row for 550 km, one gateway (review R3 #11); in the other layouts it
            # equals PAL within the rounding of the table, stated in the caption
            for strat in [x for x in STRATS_ALL if x != "sat-pred" or (shell, gw) == (550, 1)]:
                c = f"{strat}_s{shell}_gw{gw}"
                if c not in data:
                    continue
                def f(k, sc, nd):
                    m, h, _ = ci(col(data, c, k) * sc)
                    return f"{m:.{nd}f}"
                def fpm(k, sc, nd):
                    m, h, _ = ci(col(data, c, k) * sc)
                    return f"{m:.{nd}f}$\\pm${h:.{nd}f}"
                rows.append((shell, gw, strat, [
                    f("rt_hit_ratio", 100, 1), fpm("nf_hit_ratio", 100, 2), f("nf_p99_ms", 1, 1),
                    fpm("sla_viol_frac", 100, 1), f("short_depth_mean", 100, 2), f("overprov_mean", 100, 2),
                    f("superseded_frac", 100, 1), f"{ci(blackout(data, c))[0]:.0f}",
                    f("feeder_kbps", 1, 1), f(bcol(T_DEF, G_DEF), 100, 2),
                    f"{ci(beam_at_measured(data, c))[0]:.2f}"]))
    with open(os.path.join(TAB, "table_main.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n")
        t.write("\\begin{tabular}{@{}llrrrrrrrrrrr@{}}\n\\toprule\n")
        t.write("Shell, GW & Strategy & RT hit & NF hit & NF p99 & Short. & Depth & Over. & Super. & Blk. & Load & Beam & Beam$^{\\ast}$\\\\\n")
        t.write(" & & (\\%) & (\\%) & (ms) & (\\%) & (\\%) & (\\%) & (\\%) & (ms) & (kb/s) & (\\%) & (\\%)\\\\\n\\midrule\n")
        last = None
        for shell, gw, strat, vals in rows:
            key = (shell, gw)
            lead = f"{shell} km, {gw}" if key != last else ""
            if last is not None and key != last:
                t.write("\\addlinespace[2pt]\n")
            name = {"ground": "Ground only", "onboard": "On-board only", "onboard-ttl": "O+L",
                    "reactive": "Reactive hybrid", "reactive-ttl": "R+L", "reactive-ttl-pre": "R+L+P",
                    "pal": "\\textbf{PAL}", "sat-pred": "SSP"}[strat]
            t.write(f"{lead} & {name} & " + " & ".join(vals) + "\\\\\n")
            last = key
        t.write("\\bottomrule\n\\end{tabular}\n")

    # == where violations come from: share state at each 100 ms sample
    rows = []
    for strat in ["onboard", "onboard-ttl", "reactive", "reactive-ttl", "reactive-ttl-pre", "pal"]:
        c = f"{strat}_s550_gw1"
        if c not in data or not data[c][1]["checks"] or "samp_current" not in data[c][1]["checks"][0]:
            continue
        tot = defaultdict(float)
        for sd in data[c]:
            for k, v in data[c][sd]["checks"][0].items():
                tot[k] += float(v)
        allS = sum(tot[f"samp_{x}"] for x in ["default", "fallback", "stale", "current"])
        cells = []
        for x in ["current", "stale", "fallback", "default"]:
            share = 100 * tot[f"samp_{x}"] / allS
            vr = 100 * tot[f"viol_{x}"] / max(1, tot[f"samp_{x}"])
            cells += [f"{share:.1f}", f"{vr:.1f}" if tot[f"samp_{x}"] > 0 else "--"]
            mac(f"State{x.capitalize()}Share{SNM[strat]}", share, "{:.1f}")
            mac(f"State{x.capitalize()}Viol{SNM[strat]}", vr, "{:.1f}")
            if f"short_{x}" in tot:
                # share of the total shortfall depth that accrues in this state (R2-M5)
                dep = sum(tot[f"short_{y}"] for y in ["default", "fallback", "stale", "current"])
                mac(f"State{x.capitalize()}DepthShare{SNM[strat]}", 100 * tot[f"short_{x}"] / dep if dep else 0.0, "{:.0f}")
                mac(f"State{x.capitalize()}DepthRate{SNM[strat]}",
                    100 * tot[f"short_{x}"] / max(1, tot[f"samp_{x}"]), "{:.2f}")
        rows.append(({"onboard": "On-board only", "onboard-ttl": "O+L", "reactive": "Reactive hybrid",
                      "reactive-ttl": "R+L", "reactive-ttl-pre": "R+L+P", "pal": "PAL", "sat-pred": "SSP"}[strat], cells))
    with open(os.path.join(TAB, "table_state.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrrr@{}}\n\\toprule\n")
        t.write(" & \\multicolumn{2}{c}{Current} & \\multicolumn{2}{c}{Superseded} & "
                "\\multicolumn{2}{c}{Safe} & \\multicolumn{2}{c}{Default}\\\\\n")
        t.write("Strategy & time & short. & time & short. & time & short. & time & short.\\\\\n\\midrule\n")
        for name, cells in rows:
            t.write(name + " & " + " & ".join(cells) + "\\\\\n")
        t.write("\\bottomrule\n\\end{tabular}\n")
    # takeovers by a satellite without a feeder (kind 3), pooled
    for strat in STRATS_ALL:
        for shell in [550, 1200]:
            c = f"{strat}_s{shell}_gw1"
            if c in data:
                n3 = sum(1 for sd in data[c] for r in data[c][sd]["outages"] if r.get("kind") == "3")
                mac(f"Dark{SNM[strat]}{ROMAN[shell]}GOne", n3, "{:d}")

    # == NF miss attribution (550 km, one gateway)
    att = miss_attribution(data)
    SN2 = SNM
    for strat, d in att.items():
        for cause in ["transit", "detect", "switch", "onboard"]:
            mac(f"Miss{cause.capitalize()}{SN2[strat]}", d.get(cause, 0.0), "{:.0f}")
        mac(f"MissTotal{SN2[strat]}", sum(d.values()), "{:.0f}")
    hyb = [s_ for s_ in ["reactive-ttl", "reactive-ttl-pre", "pal", "sat-pred"] if s_ in att]
    if hyb:
        mac_range("MissTransitRange", [att[s_].get("transit", 0.0) for s_ in hyb], "{:.0f}")

    cols = ["ground", "onboard", "reactive-ttl", "reactive-ttl-pre", "pal", "sat-pred"]
    with open(os.path.join(TAB, "table_attr.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrr@{}}\n\\toprule\n")
        t.write("Cause & \\shortstack{Ground\\\\only} & \\shortstack{On-board\\\\only} & R+L & R+L+P & PAL & SSP\\\\\n\\midrule\n")
        for cause, lab in [("transit", "Lost in transit"),
                           ("detect", "Down, before failover"),
                           ("switch", "After switch, before setup"),
                           ("onboard", "On board (processing)")]:
            t.write(lab + " & " + " & ".join(f"{att.get(c, {}).get(cause, 0.0):.0f}" for c in cols) + "\\\\\n")
        t.write("\\midrule\nTotal & " + " & ".join(f"{sum(att.get(c, {}).values()):.0f}" for c in cols) +
                "\\\\\n\\bottomrule\n\\end{tabular}\n")

    # == switch blackout: paired CIs and first-epoch actuation
    def first_epoch_share(cfg):
        v = []
        for sd in sorted(data[cfg]):
            r = [x for x in data[cfg][sd]["switches"] if float(x["t"]) >= 1.0]
            ok = sum(1 for x in r if 0 <= float(x["blackout_nf_s"]) <= float(x["first_epoch_s"]) + 0.1 + 1e-9)
            if r:
                v.append(100.0 * ok / len(r))
        return np.array(v)
    for shell in [550, 1200]:
        for gw in [1, 2]:
            tg = f"{ROMAN[shell]}G{WORD[gw].capitalize()}"
            for strat in STRATS_ALL:
                c = f"{strat}_s{shell}_gw{gw}"
                if c in data:
                    m, h, _ = ci(first_epoch_share(c))
                    mac(f"FirstEp{SNM[strat]}{tg}", m, "{:.0f}")
            for a_, b_, nm in [("reactive-ttl", "pal", "DBlkTtlPal"), ("reactive-ttl", "reactive-ttl-pre", "DBlkTtlTpr"),
                               ("reactive-ttl-pre", "pal", "DBlkTprPal")]:
                ca, cb = f"{a_}_s{shell}_gw{gw}", f"{b_}_s{shell}_gw{gw}"
                if ca in data and cb in data:
                    va, vb = blackout(data, ca), blackout(data, cb)
                    m, h, _ = ci(va - vb)
                    mac(f"{nm}{tg}", m, "{:.0f}")
                    mac(f"{nm}{tg}Ci", h, "{:.0f}")
            # one factor at a time (review M1, M2): PAL vs the leased reactive
            # hybrid with pre-provisioning isolates prediction; R+L+P vs R+L
            # isolates pre-provisioning
            for a_, b_, nm in [("reactive-ttl-pre", "pal", "TprPal"), ("reactive-ttl", "reactive-ttl-pre", "TtlTpr"),
                               ("onboard-ttl", "pal", "OblPal"), ("onboard", "onboard-ttl", "OnbObl"),
                               ("reactive-ttl", "pal", "TtlPal"), ("sat-pred", "pal", "SspPal"),
                               ("reactive-ttl-pre", "sat-pred", "TprSsp"), ("reactive", "reactive-ttl", "ReaTtl"),
                               ("onboard-ttl", "sat-pred", "OblSsp"), ("reactive", "pal", "ReaPal")]:
                ca, cb = f"{a_}_s{shell}_gw{gw}", f"{b_}_s{shell}_gw{gw}"
                if ca in data and cb in data:
                    m, h, _ = paired(data, cb, ca, "nf_hit_ratio")
                    mac(f"DNf{nm}{tg}", m * 100, "{:.2f}")
                    mac(f"DNf{nm}{tg}Ci", h * 100, "{:.2f}")
                    dn = paired_vec(data, cb, ca, "nf_hit_ratio")
                    mac(f"DNf{nm}{tg}P", sign_p(dn), "{:.3f}")
                    mac(f"DNf{nm}{tg}W", wilcoxon_p(dn), "{:.3f}")
                    mac(f"DNf{nm}{tg}Npos", int(np.sum(dn > 0)), "{:d}")
                    m, h, _ = paired(data, cb, ca, "sla_viol_frac")
                    mac(f"DViol{nm}{tg}", m * 100, "{:+.2f}")
                    mac(f"DViol{nm}{tg}Ci", h * 100, "{:.2f}")
                    dv = paired_vec(data, cb, ca, "sla_viol_frac")
                    mac(f"DViol{nm}{tg}P", sign_p(dv), "{:.3f}")
                    mac(f"DViol{nm}{tg}W", wilcoxon_p(dv), "{:.3f}")
                    m, h, _ = paired(data, cb, ca, "overprov_mean")
                    mac(f"DOver{nm}{tg}", m * 100, "{:+.2f}")
                    mac(f"DOver{nm}{tg}Ci", h * 100, "{:.2f}")
                    # shortfall depth E[(r - x)^+], points of the cell (R2-M5)
                    dd = paired_vec(data, cb, ca, "short_depth_mean") * 100
                    m, h, _ = ci(dd)
                    mac(f"DDepth{nm}{tg}", m, "{:+.3f}")
                    mac(f"DDepth{nm}{tg}Ci", h, "{:.3f}")
                    mac(f"DDepth{nm}{tg}P", sign_p(dd), "{:.3f}")
                    mac(f"DDepth{nm}{tg}Npos", int(np.sum(dd > 0)), "{:d}")
                    rel = (col(data, cb, "short_depth_mean") - col(data, ca, "short_depth_mean")) / col(data, ca, "short_depth_mean")
                    # reported cut = ratio of the ten-seed means, the definition that
                    # matches the depths printed in Table VII (review R3 #8); the
                    # mean of per-seed ratios is kept as a separate macro
                    rom = float(np.mean(col(data, cb, "short_depth_mean")) / np.mean(col(data, ca, "short_depth_mean")) - 1)
                    mac(f"DDepthRel{nm}{tg}", 100 * rom, "{:+.0f}")
                    mac(f"DepthCut{nm}{tg}", -100 * rom, "{:.0f}")
                    mac(f"DepthCutSeed{nm}{tg}", -100 * ci(rel)[0], "{:.0f}")
                    mac(f"DepthCutSeed{nm}{tg}Ci", 100 * ci(rel)[1], "{:.0f}")
                    db_ = paired_vec(data, cb, ca, bcol(T_DEF, G_DEF)) * 100
                    m, h, _ = ci(db_)
                    mac(f"DBeamX{nm}{tg}", m, "{:+.3f}")
                    mac(f"DBeamX{nm}{tg}Ci", h, "{:.3f}")
                    mac(f"DBeamX{nm}{tg}P", sign_p(db_), "{:.3f}")
                    m, h, _ = paired(data, cb, ca, "feeder_kbps")
                    mac(f"DKbps{nm}{tg}", m, "{:+.1f}")
                    # NF misses per served hour (epochs never actuated or late)
                    hrs = col(data, cb, "served_s") / 3600.0
                    mpb = (col(data, cb, "nf_opp") - col(data, cb, "nf_hit")) / hrs
                    mpa = (col(data, ca, "nf_opp") - col(data, ca, "nf_hit")) / hrs
                    mac(f"DNfEpH{nm}{tg}", ci(mpa - mpb)[0], "{:.0f}")

    # == RT ground feasibility per layout (PAL runs)
    for shell in [550, 1200]:
        for gw in [1, 2, 3, 4]:
            c = f"pal_s{shell}_gw{gw}"
            if c in data:
                tt = sum(int(data[c][sd]["checks"][0]["rt_ticks"]) for sd in data[c])
                tg_ = sum(int(data[c][sd]["checks"][0]["rt_ground_feasible_ticks"]) for sd in data[c])
                mac(f"RtFeas{ROMAN[shell]}G{WORD[gw].capitalize()}", 100.0 * tg_ / max(1, tt), "{:.1f}")

    # == NF loop-time CCDF at 550 km, 2 gateways (pooled histograms)
    fig, ax = plt.subplots(figsize=(3.45, 2.45))
    for strat in STRATS:
        c = f"{strat}_s550_gw2"
        if c not in data:
            continue
        h = defaultdict(int)
        for s in data[c]:
            for r in data[c][s]["hist"]:
                if r["cls"] == "nf":
                    h[float(r["bin_ms"])] += int(r["count"])
        if not h:
            continue
        b = np.array(sorted(h))
        n = np.array([h[x] for x in b], dtype=float)
        ccdf = 1.0 - np.cumsum(n) / n.sum()
        st = STRAT_STYLE[strat]
        ax.step(b + 0.25, np.maximum(ccdf, 1e-6), where="post", color=st["color"], ls=st["ls"],
                label=st["label"], lw=1.1)
    ax.axvline(100, color=RED, ls=":", lw=0.8)
    ax.text(97, 3e-2, "100 ms\ndeadline", color=RED, fontsize=7, va="bottom", ha="right")
    ax.set_yscale("log")
    ax.set_xscale("log")
    ax.set_ylim(1e-5, 1.2)
    ax.set_xlabel("Measure-to-actuation time of applied actions (ms)")
    ax.set_ylabel("CCDF")
    ax.legend(loc="lower left", bbox_to_anchor=(0.155, 0.0), frameon=False, fontsize=6.0, handlelength=1.8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_loop_ccdf.pdf"))
    plt.close(fig)
    return data


def crossover_gap(dvals):
    """dvals[i] = (PAL - O+L) misalignment at GAPS[i]; the gap at which PAL
    starts to beat O+L (linear interpolation), 0 if PAL already wins at no
    gap, nan if it never does on the grid."""
    if dvals[0] <= 0:
        return 0.0
    for i in range(1, len(GAPS)):
        if dvals[i] <= 0:
            g0, g1, d0, d1 = GAPS[i - 1], GAPS[i], dvals[i - 1], dvals[i]
            return g0 + (g1 - g0) * d0 / (d0 - d1)
    return float("nan")


def analyze_beam(plt, data):
    """NF beam-tracking outcome: who wins as a function of the on-board accuracy gap."""
    if not data:
        return
    layouts = [(550, 1), (550, 2), (1200, 1), (1200, 2)]
    for shell, gw in layouts:
        tag = f"{ROMAN[shell]}G{WORD[gw].capitalize()}"
        for strat in STRATS_ALL:
            c = f"{strat}_s{shell}_gw{gw}"
            if c not in data:
                continue
            for T in DWELL:
                for gp, gt in [(0.0, "Zero"), (G_DEF, ""), (0.05, "Five"), (0.1, "Ten")]:
                    m, h, _ = ci(col(data, c, bcol(T, gp)) * 100)
                    mac(f"Beam{gt}{SNM[strat]}{tag}T{DWN[T]}", m, "{:.2f}")
            m, h, _ = ci(col(data, c, bcol(T_DEF, G_DEF)) * 100)
            mac(f"Beam{SNM[strat]}{tag}", m, "{:.2f}")
            mac(f"Beam{SNM[strat]}{tag}Ci", h, "{:.2f}")
            m, h, _ = ci(col(data, c, bcol(T_DEF, 0.0)) * 100)
            mac(f"BeamZero{SNM[strat]}{tag}", m, "{:.2f}")
            od = col(data, c, "nf_onb_decisions") / (col(data, c, "served_s") / 3600.0)
            mac(f"NfOnbDec{SNM[strat]}{tag}", ci(od)[0], "{:,.0f}")
        a, o, tp = f"pal_s{shell}_gw{gw}", f"onboard-ttl_s{shell}_gw{gw}", f"reactive-ttl-pre_s{shell}_gw{gw}"
        if a in data and o in data:
            for gp, gt in [(0.0, "Zero"), (G_DEF, "")]:
                d = paired_vec(data, a, o, bcol(T_DEF, gp)) * 100
                m, h, _ = ci(d)
                mac(f"DBeam{gt}PalObl{tag}", m, "{:+.2f}")
                mac(f"DBeam{gt}PalObl{tag}Ci", h, "{:.2f}")
                mac(f"DBeam{gt}PalObl{tag}P", sign_p(d), "{:.3f}")
                mac(f"DBeam{gt}PalObl{tag}Npos", int(np.sum(d > 0)), "{:d}")
            for T in DWELL:
                per_seed = []
                for sd in sorted(set(data[a]) & set(data[o])):
                    dv = [float(data[a][sd]["summary"][bcol(T, gp)]) - float(data[o][sd]["summary"][bcol(T, gp)])
                          for gp in GAPS]
                    per_seed.append(crossover_gap(dv) * 100)
                mean_d = [float(np.mean(paired_vec(data, a, o, bcol(T, gp)))) for gp in GAPS]
                mac(f"BeamCross{tag}T{DWN[T]}", crossover_gap(mean_d) * 100, "{:.2f}")
                ps = [x for x in per_seed if not math.isnan(x)]
                m, h, n = ci(ps)
                mac(f"BeamCrossSeed{tag}T{DWN[T]}", m, "{:.2f}")
                mac(f"BeamCrossSeed{tag}T{DWN[T]}Ci", h, "{:.2f}")
                mac(f"BeamCrossMax{tag}T{DWN[T]}", max(ps) if ps else float("nan"), "{:.2f}")
                mac(f"BeamCrossMin{tag}T{DWN[T]}", min(ps) if ps else float("nan"), "{:.2f}")
        if a in data and tp in data:
            for gp, gt in [(0.0, "Zero"), (G_DEF, ""), (0.1, "Ten")]:
                d = paired_vec(data, a, tp, bcol(T_DEF, gp)) * 100
                m, h, _ = ci(d)
                mac(f"DBeam{gt}PalTpr{tag}", m, "{:+.3f}")
                mac(f"DBeam{gt}PalTpr{tag}Ci", h, "{:.3f}")
                mac(f"DBeam{gt}PalTpr{tag}P", sign_p(d), "{:.3f}")
        sp = f"sat-pred_s{shell}_gw{gw}"
        if sp in data and o in data:
            for T in DWELL:
                mean_d = [float(np.mean(paired_vec(data, sp, o, bcol(T, gp)))) for gp in GAPS]
                mac(f"BeamCrossSsp{tag}T{DWN[T]}", crossover_gap(mean_d) * 100, "{:.2f}")
        if sp in data and a in data:
            for gp, gt in [(0.0, "Zero"), (G_DEF, ""), (0.1, "Ten")]:
                d = paired_vec(data, a, sp, bcol(T_DEF, gp)) * 100
                m, h, _ = ci(d)
                mac(f"DBeam{gt}PalSsp{tag}", m, "{:+.3f}")
                mac(f"DBeam{gt}PalSsp{tag}Ci", h, "{:.3f}")
                mac(f"DBeam{gt}PalSsp{tag}P", sign_p(d), "{:.3f}")
    analyze_measured_delta(data)
    # first-order decomposition of the break-even (hostile review #3)
    a, o = "pal_s550_gw1", "onboard-ttl_s550_gw1"
    if a in data and o in data:
        mac("PalGroundShareFiveGOne", 1 - ci(col(data, a, "nf_onboard_frac"))[0], "{:.2f}")
        slope = (ci(col(data, o, bcol(T_DEF, 0.1)))[0] - ci(col(data, o, bcol(T_DEF, 0.0)))[0]) / 0.1
        mac("OblSlopeFiveGOne", slope, "{:.2f}")
    # figure: misalignment against the gap (550 km, one gateway) and the crossover
    fig, axes = plt.subplots(1, 2, figsize=(3.5, 1.95), gridspec_kw=dict(width_ratios=[1.25, 1]))
    ax = axes[0]
    SHORT = {"onboard-ttl": "O+L", "reactive-ttl": "R+L", "reactive-ttl-pre": "R+L+P", "pal": "PAL"}
    for strat in ["onboard-ttl", "reactive-ttl", "reactive-ttl-pre", "pal"]:
        c = f"{strat}_s550_gw1"
        if c not in data:
            continue
        ys, es = [], []
        for gp in GAPS:
            m, h, _ = ci(col(data, c, bcol(T_DEF, gp)) * 100)
            ys.append(m); es.append(h)
        st = STRAT_STYLE[strat]
        ax.errorbar([100 * g for g in GAPS], ys, yerr=es, color=st["color"], marker=st["marker"], ms=3,
                    ls=st["ls"], lw=0.9, capsize=1.5, label=SHORT[strat])
    ax.axvline(100 * G_DEF, color=GRAY, lw=0.6, ls=":")
    ax.set_xlabel(r"On-board accuracy gap $\delta$ (points)")
    ax.set_ylabel("Beam misaligned (% of time)")
    ax.set_title(f"(a) 550 km, one gateway, $T_b$ = {T_DEF:g} s", fontsize=7)
    ax.legend(loc="upper left", frameon=False, fontsize=7, handlelength=1.8, labelspacing=0.25)
    ax = axes[1]
    import json
    for gw, mk, cl in [(1, "D", GOLD), (2, "o", NAVY)]:
        a, o = f"pal_s550_gw{gw}", f"onboard-ttl_s550_gw{gw}"
        if a not in data or o not in data:
            continue
        xs, ys = [], []
        for T in DWELL:
            mean_d = [float(np.mean(paired_vec(data, a, o, bcol(T, gp)))) for gp in GAPS]
            xs.append(T); ys.append(100 * crossover_gap(mean_d))
        ax.plot(xs, ys, color=cl, marker=mk, ms=3, lw=0, label=f"{WORD[gw]} gateway{'s' if gw > 1 else ''}, sim.")
        # Corollary 1 (exact form) from beam_model.py, evaluated at the four dwell times
        bj = os.path.join(HERE, "beam_model.json")
        if os.path.exists(bj):
            cr = json.load(open(bj))["cross"]
            ax.plot(DWELL, [cr[f"s550_gw{gw}_T{T:g}"]["exact"] for T in DWELL], color=cl, lw=0.9, ls="-")
    # measured INT8 gap of the beam predictor (beam_quant.py), at its measured dwell
    import json
    mx, my, me = [], [], []
    for narr in [128, 64]:
        pj = os.path.join(HERE, "beam_quant", f"results_N{narr}.json")
        if os.path.exists(pj):
            rj = json.load(open(pj))
            mx.append(rj["dwell_s"]); my.append(rj["gap_int8_x86"][0]); me.append(rj["gap_int8_x86"][1])
    if mx:
        ax.errorbar(mx, my, yerr=me, color=RED, marker="x", ms=4, ls="none", capsize=1.5, lw=0.9,
                    label=r"measured INT8 $\delta$")
    ax.set_xscale("log")
    ax.set_xticks(DWELL)
    ax.set_xticklabels([f"{T:g}" for T in DWELL])
    ax.set_xlabel(r"Beam dwell $T_b$ (s)")
    ax.set_ylabel(r"Break-even $\delta^\ast$ (points)")
    ax.set_title(r"(b) Break-even gap $\delta^\ast$", fontsize=7)
    ax.plot([], [], color=INK2 if "INK2" in globals() else "0.3", lw=0.9, label="analysis (Cor. 1)")
    ax.set_ylim(0, 4.6)
    ax.legend(loc="upper right", frameon=False, fontsize=6.5, handlelength=1.5, labelspacing=0.2)
    fig.tight_layout(pad=0.3, w_pad=0.6)
    fig.savefig(os.path.join(FIG, "fig_beam_gap.pdf"))
    plt.close(fig)


def interp_gap(data, a, b, T, delta):
    """per-seed (a - b) misalignment at an on-board gap delta (fraction), by
    linear interpolation between the two recorded gaps that bracket it."""
    i = max(k for k in range(len(GAPS)) if GAPS[k] <= delta + 1e-12)
    j = min(i + 1, len(GAPS) - 1)
    w = 0.0 if j == i else (delta - GAPS[i]) / (GAPS[j] - GAPS[i])
    di = paired_vec(data, a, b, bcol(T, GAPS[i]))
    dj = paired_vec(data, a, b, bcol(T, GAPS[j]))
    return (1 - w) * di + w * dj


def beam_at_measured(data, c):
    """per-seed misalignment of config c at the measured INT8 gap of the predictor at T_b = 2 s"""
    import json
    r = json.load(open(os.path.join(HERE, "beam_quant", "results_N128.json")))
    delta = max(0.0, r["gap_int8_x86"][0] / 100.0)
    i = max(k for k in range(len(GAPS)) if GAPS[k] <= delta + 1e-12)
    j = min(i + 1, len(GAPS) - 1)
    w = 0.0 if j == i else (delta - GAPS[i]) / (GAPS[j] - GAPS[i])
    return 100 * ((1 - w) * col(data, c, bcol(T_DEF, GAPS[i])) + w * col(data, c, bcol(T_DEF, GAPS[j])))


def analyze_measured_delta(data):
    """Measured on-board accuracy gap of an INT8 beam predictor (beam_quant.py)
    and the PAL - O+L beam outcome at that gap."""
    import json
    for narr, T, nm in [(128, 2.0, "Two"), (64, 5.0, "Five")]:
        p = os.path.join(HERE, "beam_quant", f"results_N{narr}.json")
        if not os.path.exists(p):
            print("missing", p)
            continue
        r = json.load(open(p))
        mac(f"DqSigma{nm}", r["sigma_db"], "{:.1f}")
        mac(f"DqDwell{nm}", r["dwell_s"], "{:.1f}")
        mac(f"DqNtrain{nm}", f"{r['n_train']:,}".replace(",", "{,}"))
        mac(f"DqNtest{nm}", f"{r['n_test']:,}".replace(",", "{,}"))
        mac(f"DqSeeds{nm}", r["seeds"], "{:d}")
        mac(f"DqStay{nm}", 100 * r["stay_acc"], "{:.1f}")
        mac(f"DqAccFp{nm}", 100 * r["fp32"][0], "{:.1f}")
        mac(f"DqAccFp{nm}Ci", 100 * r["fp32"][1], "{:.1f}")
        for k, kn in [("int8_x86", "X"), ("int8_qnnpack", "Q"), ("prune50_int8_x86", "P"), ("int4w", "F")]:
            mac(f"DqAcc{kn}{nm}", 100 * r[k][0], "{:.1f}")
            mac(f"DqGap{kn}{nm}", r["gap_" + k][0], "{:.2f}")
            mac(f"DqGap{kn}{nm}Ci", r["gap_" + k][1], "{:.2f}")
            mac(f"DqGap{kn}{nm}Max", max(x["gap_" + k] for x in r["runs"]), "{:.2f}")
            mac(f"DqGap{kn}{nm}Min", min(x["gap_" + k] for x in r["runs"]), "{:.2f}")
        for gw in [1, 2]:
            tag = f"FiveG{WORD[gw].capitalize()}"
            a, o = f"pal_s550_gw{gw}", f"onboard-ttl_s550_gw{gw}"
            if a not in data or o not in data:
                continue
            for k, kn in [("int8_x86", "X"), ("prune50_int8_x86", "P"), ("int4w", "F")]:
                dlt = max(0.0, r["gap_" + k][0] / 100.0)
                d = interp_gap(data, a, o, T, dlt) * 100
                m, h, _ = ci(d)
                mac(f"DqDBeam{kn}PalObl{tag}T{nm}", m, "{:+.2f}")
                mac(f"DqDBeam{kn}PalObl{tag}T{nm}Ci", h, "{:.2f}")
                mac(f"DqDBeam{kn}PalObl{tag}T{nm}Npos", int(np.sum(d > 0)), "{:d}")
                mac(f"DqDBeam{kn}PalObl{tag}T{nm}P", sign_p(d), "{:.2f}")
            mac(f"DqRatio{tag}T{nm}", r["gap_int8_x86"][0] / max(1e-9, float(macros_num(f"BeamCross{tag}T{nm}"))),
                "{:.1f}")


def macros_num(name):
    v = macros.get(name, "nan").replace("\\ensuremath{-}", "-")
    try:
        return float(v)
    except ValueError:
        return float("nan")


def analyze_variants(main):
    """Soft switch-over, link-layer detection and ARQ (review D11, D12, D27)."""
    data = load("variants")
    if not data:
        return
    mac("VarRuns", sum(len(v) for v in data.values()), "{:d}")
    VN = {"": "Base", "soft": "Soft", "link": "Link", "softlink": "SoftLink", "arq": "Arq"}
    def get(strat, gw, v):
        if v == "":
            return main, f"{strat}_s550_gw{gw}"
        return data, f"{strat}_s550_gw{gw}_{v}"
    rows = []
    rawd = {}
    for v, lab in [("", "Default"), ("soft", "Soft switch-over"),
                   ("link", "Link-layer detect."), ("softlink", "Soft + link-layer"),
                   ("arq", "ARQ, 50 ms")]:
        cells = []
        for gw in [1, 2]:
            dt, ct = get("reactive-ttl-pre", gw, v)
            dp, cp = get("pal", gw, v)
            if ct not in dt or cp not in dp:
                cells += ["--", "--", "--"]
                continue
            mt = ci((1 - col(dt, ct, "nf_hit_ratio")) * 100)[0]
            mp = ci((1 - col(dp, cp, "nf_hit_ratio")) * 100)[0]
            seeds = sorted(set(dt[ct]) & set(dp[cp]))
            d = [100 * (float(dp[cp][sd]["summary"]["nf_hit_ratio"]) - float(dt[ct][sd]["summary"]["nf_hit_ratio"]))
                 for sd in seeds]
            m, h, _ = ci(d)
            tg = f"{VN[v]}G{WORD[gw].capitalize()}"
            mac(f"VarMissTpr{tg}", mt, "{:.2f}")
            mac(f"VarMissPal{tg}", mp, "{:.2f}")
            rawd[(v, gw)] = m
            mac(f"VarDNf{tg}", m, "{:+.2f}")
            mac(f"VarDNf{tg}Ci", h, "{:.2f}")
            mac(f"VarDNf{tg}P", sign_p(d), "{:.3f}")
            mac(f"VarOut{tg}", ci(real_outages_per_run(dp, cp))[0], "{:.1f}")
            mac(f"VarKbpsTpr{tg}", ci(col(dt, ct, "feeder_kbps"))[0], "{:.1f}")
            mac(f"VarKbpsPal{tg}", ci(col(dp, cp, "feeder_kbps"))[0], "{:.1f}")
            db = [100 * (float(dp[cp][sd]["summary"][bcol(T_DEF, G_DEF)]) -
                         float(dt[ct][sd]["summary"][bcol(T_DEF, G_DEF)])) for sd in seeds]
            mb, hb, _ = ci(db)
            mac(f"VarDBeam{tg}", mb, "{:+.2f}")
            mac(f"VarDBeam{tg}Ci", hb, "{:.2f}")
            cells += [f"{mt:.2f}", f"{mp:.2f}", f"{m:+.2f}$\\pm${h:.2f}"]
            # gain kept relative to the default contrast
        rows.append((lab, cells))
    for gw in [1, 2]:
        for v in ["soft", "link", "softlink", "arq"]:
            if ("", gw) in rawd and (v, gw) in rawd and abs(rawd[("", gw)]) > 1e-9:
                mac(f"VarKeep{VN[v]}G{WORD[gw].capitalize()}", 100 * rawd[(v, gw)] / rawd[("", gw)], "{:.0f}")
    for v in ["soft", "link"]:
        for gw in [1, 2]:
            dp_, cp_ = get("pal", gw, v)
            ds_, cs_ = get("sat-pred", gw, v)
            if cp_ in dp_ and cs_ in ds_:
                d = paired_vec(dp_, cp_, cs_, "nf_hit_ratio", ds_) * 100
                m, h, _ = ci(d)
                mac(f"VarDNfPalSsp{VN[v]}G{WORD[gw].capitalize()}", m, "{:+.3f}")
                mac(f"VarDNfPalSsp{VN[v]}G{WORD[gw].capitalize()}Ci", h, "{:.3f}")
                mac(f"VarDNfPalSsp{VN[v]}G{WORD[gw].capitalize()}P", sign_p(d), "{:.3f}")
                mac(f"VarMissSsp{VN[v]}G{WORD[gw].capitalize()}", ci((1 - col(ds_, cs_, "nf_hit_ratio")) * 100)[0], "{:.2f}")
    for v in ["soft", "link"]:
        for strat in ["reactive-ttl", "reactive-ttl-pre", "pal"]:
            for gw in [1, 2]:
                dd, cc = get(strat, gw, v)
                if cc in dd:
                    mac(f"VarMiss{SNM[strat]}{VN[v]}G{WORD[gw].capitalize()}",
                        ci((1 - col(dd, cc, "nf_hit_ratio")) * 100)[0], "{:.2f}")
    for strat in ["reactive-ttl-pre", "pal"]:
        for gw in [1, 2]:
            dd, cc = get(strat, gw, "arq")
            if cc in dd:
                mac(f"VarRetx{SNM[strat]}G{WORD[gw].capitalize()}", ci(col(dd, cc, "arq_retx"))[0], "{:.0f}")
    with open(os.path.join(TAB, "table_variants.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrr@{}}\n\\toprule\n")
        t.write(" & \\multicolumn{3}{c}{One gateway} & \\multicolumn{3}{c}{Two gateways}\\\\\n")
        t.write("\\cmidrule(lr){2-4}\\cmidrule(l){5-7}\n")
        t.write("Variant & R+L+P & PAL & $\\Delta$ & R+L+P & PAL & $\\Delta$\\\\\n\\midrule\n")
        for lab, cells in rows:
            t.write(lab + " & " + " & ".join(cells) + "\\\\\n")
        t.write("\\bottomrule\n\\end{tabular}\n")
    # hedge fallback g0 = 0.6: same lease, capacity above the alphabet
    for strat in ["reactive-ttl", "pal", "onboard-ttl"]:
        c = f"{strat}_s550_gw1_hedge"
        b = f"{strat}_s550_gw1"
        if c in data and b in main:
            tg = SNM[strat]
            mac(f"HedgeViol{tg}", ci(col(data, c, "sla_viol_frac") * 100)[0], "{:.1f}")
            mac(f"HedgeOver{tg}", ci(col(data, c, "overprov_mean") * 100)[0], "{:.2f}")
            d = paired_vec(data, c, b, "overprov_mean", main) * 100
            m, h, _ = ci(d)
            mac(f"HedgeDOver{tg}", m, "{:+.2f}")
            mac(f"HedgeDOver{tg}Ci", h, "{:.2f}")
            mac(f"HedgeDOver{tg}Pos", int(np.sum(d > 0)), "{:d}")
            d = paired_vec(data, c, b, "sla_viol_frac", main) * 100
            m, h, _ = ci(d)
            mac(f"HedgeDViol{tg}", m, "{:+.2f}")
            mac(f"HedgeDViol{tg}Ci", h, "{:.2f}")
            mac(f"HedgeDViol{tg}Neg", int(np.sum(d < 0)), "{:d}")


def miss_attribution(data):
    """Classify never-actuated NF epochs by the state recorded at the epoch.
    Returns {strategy: {cause: misses per hour of service}} at 550 km, 1 gw."""
    out = {}
    for strat in STRATS_ALL:
        c = f"{strat}_s550_gw1"
        if c not in data:
            continue
        cnt = defaultdict(float)
        hours = 0.0
        for s in data[c]:
            hours += float(data[c][s]["summary"]["served_s"]) / 3600.0
            for r in data[c][s]["nf_miss"]:
                lu, ob, e2, sub, aut, ls, asw = (int(r[k]) for k in ["link_up", "onboard",
                    "e2_setup", "subscribed", "autonomous", "leased", "after_switch"])
                if ob:
                    cause = "onboard"
                elif asw and (not e2 or not sub):
                    cause = "switch"
                elif not lu:
                    cause = "detect"
                else:
                    cause = "transit"
                cnt[cause] += 1
        out[strat] = {k: v / hours for k, v in cnt.items()}
    return out


def analyze_ablation(main):
    data = load("ablation")
    if not data:
        return
    rows = []
    for shell in [550]:
        for gw in [1, 2]:
            for strat, name in [("reactive-ttl", "R+L"),
                                ("reactive-ttl-pre", "R+L+P"),
                                ("pal", "PAL"),
                                ("pal-nopre", "PAL without pre-provisioning"),
                                ("pal-gw", "PAL with ground-wins merge")]:
                c = f"{strat}_s{shell}_gw{gw}"
                src = main if strat in ("reactive-ttl", "reactive-ttl-pre", "pal") else data
                if c not in src:
                    continue
                def f(k, sc, nd, src=src, c=c):
                    m, h, _ = ci(col(src, c, k) * sc)
                    return f"{m:.{nd}f}"
                def r_(k, sc, nd, src=src, c=c):
                    m, h, _ = ci(recon(src, c, k, sc))
                    return "--" if math.isnan(m) else f"{m:.{nd}f}"
                rows.append((shell, gw, name, [
                    f"{ci(blackout(src, c))[0]:.0f}", f"{ci(blackout(src, c, 'p95'))[0]:.0f}",
                    r_("ttc", 1000, 0), r_("direct_conflicts", 1, 2),
                    r_("policy_stale", 1, 2), r_("fallback", 1, 2), f("sla_viol_frac", 100, 1),
                    f("sig_bytes_per_switch", 1, 0)]))
    with open(os.path.join(TAB, "table_ablation.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n")
        t.write("\\begin{tabular}{@{}llrrrrrrrr@{}}\n\\toprule\n")
        t.write("Shell, GW & Variant & Blk. & Blk. p95 & TTC & Disag. & Pol. & Safe & Short. & Sig.\\\\\n")
        t.write(" & & (ms) & (ms) & (ms) & /rec. & /rec. & /rec. & (\\%) & (B/switch)\\\\\n\\midrule\n")
        last = None
        for shell, gw, name, vals in rows:
            key = (shell, gw)
            lead = f"{shell} km, {gw}" if key != last else ""
            if last is not None and key != last:
                t.write("\\addlinespace[2pt]\n")
            t.write(f"{lead} & {name} & " + " & ".join(vals) + "\\\\\n")
            last = key
        t.write("\\bottomrule\n\\end{tabular}\n")
    n = sum(len(v) for v in data.values())
    mac("AblRuns", n, "{:d}")
    for strat, tag in [("pal-nopre", "NoPre"), ("pal-gw", "Gw")]:
        for shell in [550, 1200]:
            for gw in [1, 2]:
                c = f"{strat}_s{shell}_gw{gw}"
                if c not in data:
                    continue
                tg = f"{tag}{ROMAN[shell]}G{WORD[gw].capitalize()}"
                mac(f"Black{tg}", ci(blackout(data, c))[0], "{:.0f}")
                mac(f"BlackPnf{tg}", ci(blackout(data, c, "p95"))[0], "{:.0f}")
                mac(f"Ttc{tg}", ci(recon(data, c, "ttc", 1000))[0], "{:.0f}")
                mac(f"Conf{tg}", ci(recon(data, c, "direct_conflicts"))[0], "{:.2f}")


def analyze_sens(main):
    data = load("sens")
    if not data:
        return
    for strat in ["reactive-ttl", "pal", "sat-pred"]:
        for sig in [0.0, 2.0, 5.0]:
            c = f"{strat}_s550_gw1_eph{sig}"
            if c in data:
                tg = f"{SNM[strat]}Eph{ {0: 'Zero', 2: 'Two', 5: 'Five'}[int(sig)] }"
                mac(f"NfMiss{tg}", ci((1 - col(data, c, "nf_hit_ratio")) * 100)[0], "{:.2f}")
                mac(f"Viol{tg}", ci(col(data, c, "sla_viol_frac") * 100)[0], "{:.1f}")
                mac(f"Leases{tg}", ci(col(data, c, "leases" if strat != "sat-pred" else "sat_pred_entries"))[0], "{:.1f}")
    for sig in [0.0, 2.0, 5.0]:
        a2, b2 = f"pal_s550_gw1_eph{sig}", f"sat-pred_s550_gw1_eph{sig}"
        if a2 in data and b2 in data:
            d = paired_vec(data, a2, b2, "nf_hit_ratio") * 100
            m, h, _ = ci(d)
            nm = {0: 'Zero', 2: 'Two', 5: 'Five'}[int(sig)]
            mac(f"DNfPalSspEph{nm}", m, "{:+.3f}")
            mac(f"DNfPalSspEph{nm}Ci", h, "{:.3f}")
    RT = {0.0: "Zero", 0.23: "Itu", 6.0: "Six"}
    for strat in ["reactive-ttl", "reactive-ttl-pre", "pal", "onboard-ttl"]:
        for rain, rt in RT.items():
            c = f"{strat}_s550_gw1_rain{rain}"
            if c in data:
                tg = f"{SNM[strat]}Rain{rt}"
                mac(f"NfMiss{tg}", ci((1 - col(data, c, "nf_hit_ratio")) * 100)[0], "{:.2f}")
                mac(f"Viol{tg}", ci(col(data, c, "sla_viol_frac") * 100)[0], "{:.1f}")
                mac(f"Avail{tg}", ci(col(data, c, "feeder_avail") * 100)[0], "{:.1f}")
                mac(f"Ttc{tg}", ci(recon(data, c, "ttc", 1000))[0], "{:.0f}")
    # single-factor contrasts by outage cause (review D3): PalTpr isolates
    # prediction, TprTtl isolates pre-provisioning, PalObl is PAL against
    # on-board hosting with the same lease
    for rain, tg in RT.items():
        for x_, y_, nm2 in [("pal", "reactive-ttl-pre", "PalTpr"), ("reactive-ttl-pre", "reactive-ttl", "TprTtl"),
                            ("pal", "onboard-ttl", "PalObl"), ("pal", "reactive-ttl", "PalTtl")]:
            a2, b2 = f"{x_}_s550_gw1_rain{rain}", f"{y_}_s550_gw1_rain{rain}"
            if a2 in data and b2 in data:
                for key, nm, sc in [("sla_viol_frac", "Viol", 100), ("nf_hit_ratio", "NfHit", 100),
                                    (bcol(T_DEF, G_DEF), "Beam", 100)]:
                    d = paired_vec(data, a2, b2, key) * sc
                    m, h, n = ci(d)
                    mac(f"Dt{nm}{nm2}Rain{tg}", m, "{:+.2f}")
                    mac(f"Dt{nm}{nm2}Rain{tg}Ci", h, "{:.2f}")
                    mac(f"Dt{nm}{nm2}Rain{tg}P", sign_p(d), "{:.3f}")
    nh = [macros_num(f"DtNfHitPalTprRain{tg}") for tg in RT.values() if f"DtNfHitPalTprRain{tg}" in macros]
    if nh:
        mac_range("DtNfHitPalTprRainRange", nh, "{:.2f}")
        mac("DtNfHitPalTprRainSame", "yes" if len({f"{abs(x):.2f}" for x in nh}) == 1 else "no")
    for strat in ["onboard", "onboard-ttl", "reactive-ttl", "pal"]:
        c = f"{strat}_s550_gw1_proc5"
        if c in data:
            tg = SNM[strat]
            for cls in ["rt", "nf", "ns"]:
                mac(f"ProcFive{cls.capitalize()}Hit{tg}", ci(col(data, c, f"{cls}_hit_ratio") * 100)[0], "{:.2f}")
            mac(f"ProcFiveViol{tg}", ci(col(data, c, "sla_viol_frac") * 100)[0], "{:.1f}")
            mac(f"ProcFiveRtPnn{tg}", ci(col(data, c, "rt_p99_ms"))[0], "{:.1f}")
            mac(f"ProcFiveBeam{tg}", ci(col(data, c, bcol(T_DEF, G_DEF)) * 100)[0], "{:.2f}")
    a, o = "pal_s550_gw1_proc5", "onboard-ttl_s550_gw1_proc5"
    if a in data and o in data:
        for gp, gt in [(0.0, "Zero"), (G_DEF, "")]:
            d = paired_vec(data, a, o, bcol(T_DEF, gp)) * 100
            m, h, _ = ci(d)
            mac(f"ProcFiveDBeam{gt}PalObl", m, "{:+.2f}")
            mac(f"ProcFiveDBeam{gt}PalObl" + "Ci", h, "{:.2f}")
        dv = [float(np.mean(paired_vec(data, a, o, bcol(T_DEF, gp)))) for gp in GAPS]
        mac("ProcFiveBeamCross", 100 * crossover_gap(dv), "{:.2f}")
    n = sum(len(v) for v in data.values())
    mac("SensRuns", n, "{:d}")


def analyze_lease_model(plt, main):
    """Analysis against simulation for the lease (Section V-C model): the state
    times of eq. (cost) evaluated with the outage lengths and rates of the
    HELD-OUT runs (seeds 11-20), against the scored PAL runs of the eps sweep."""
    held = load("heldout")
    data = load("eps")
    hc = "reactive-ttl_s550_gw1_eps0.2"
    if hc not in held or "pal_s550_gw1" not in main:
        return
    data["pal_s550_gw1_eps0.2"] = main["pal_s550_gw1"]
    H = held[hc]
    r = rates_from(held, hc)
    tot = defaultdict(float)
    for sd in H:
        for k, v in H[sd]["checks"][0].items():
            tot[k] += float(v)
    N = sum(tot[f"samp_{x}"] for x in ["current", "stale", "fallback", "default"])
    p_def, o_def = tot["viol_default"] / tot["samp_default"], tot["over_default"] / tot["samp_default"]
    f_def, f_up = tot["samp_default"] / N, tot["samp_up_stale"] / N
    outs = [float(o["dur"]) for sd in H for o in H[sd]["outages"] if float(o["start"]) > 0]
    t_serv = sum(float(H[sd]["summary"]["served_s"]) for sd in H)
    # the RIC's estimate of T_A1 as the held-out runs used it: tau* / -ln(1 - 0.2)
    t_a1 = ci(col(held, hc, "lease_tau_s"))[0] / -math.log(1 - 0.2)

    def pred(tau):
        tc = ts = tf = 0.0
        for th in outs:
            m = min(tau, th)
            a = t_a1 * (1 - math.exp(-m / t_a1))
            tc += a
            ts += m - a
            tf += max(0.0, th - tau)
        fs, ff = ts / t_serv + f_up, tf / t_serv
        fc = 1 - fs - ff - f_def
        return dict(short=100 * (r["pc"] * fc + r["pv"] * fs + r["ps"] * ff + p_def * f_def),
                    over=100 * (r["oc"] * fc + r["ov"] * fs + r["os"] * ff + o_def * f_def),
                    stale=100 * fs, safe=100 * ff)

    grid = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.9, 0.99]
    sim = {}
    for eps in grid:
        c = f"pal_s550_gw1_eps{eps}"
        if c not in data:
            continue
        ck = [data[c][sd]["checks"][0] for sd in sorted(data[c])]
        nn = [sum(float(k[f"samp_{x}"]) for x in ["current", "stale", "fallback", "default"]) for k in ck]
        sim[eps] = dict(short=ci(col(data, c, "sla_viol_frac") * 100), over=ci(col(data, c, "overprov_mean") * 100),
                        stale=ci([100 * float(k["samp_stale"]) / n for k, n in zip(ck, nn)]),
                        safe=ci([100 * float(k["samp_fallback"]) / n for k, n in zip(ck, nn)]),
                        tau=ci(col(data, c, "lease_tau_s"))[0])
    ef = np.linspace(0.02, 0.995, 200)
    P = [pred(-t_a1 * math.log(1 - e)) for e in ef]
    err = {k: [] for k in ["short", "over"]}
    inside = 0
    st_in, st_n, bias_st = 0, 0, []
    for eps, sv in sim.items():
        pv = pred(sv["tau"])
        for k in err:
            err[k].append(abs(pv[k] - sv[k][0]))
            inside += abs(pv[k] - sv[k][0]) <= sv[k][1]
        for k in ["stale", "safe"]:
            st_in += abs(pv[k] - sv[k][0]) <= sv[k][1]
            st_n += 1
        bias_st.append(pv["stale"] - sv["stale"][0])
    mac("LmStateInCi", st_in, "{:d}")
    mac("LmStateN", st_n, "{:d}")
    mac("LmStaleBiasMin", min(bias_st), "{:.1f}")
    mac("LmStaleBiasMax", max(bias_st), "{:.1f}")
    fig, ax = plt.subplots(1, 3, figsize=(7.1, 2.2))
    es = sorted(sim)
    for a_, keys, ttl, yl in [(ax[0], [("stale", "superseded", RED, "o"), ("safe", "safe intent", NAVY, "s")],
                               "(a) time under each intent", "Share of served time (%)"),
                              (ax[1], [("short", None, GOLD, "D")], "(b) intent shortfall", "Shortfall time (%)"),
                              (ax[2], [("over", None, GOLD, "D")], "(c) over-provisioning",
                               "Over-provisioning (% of cell)")]:
        for k, lab, colr, mk in keys:
            a_.plot(ef, [q[k] for q in P], color=colr, lw=1.1, label=f"analysis{', ' + lab if lab else ''}")
            a_.errorbar(es, [sim[e][k][0] for e in es], yerr=[sim[e][k][1] for e in es], fmt=mk, ms=3.2,
                        mfc="white", color=colr, capsize=1.5, lw=0.8,
                        label=f"simulation{', ' + lab if lab else ''}")
        a_.axvline(0.2, color=GRAY, ls=":", lw=0.8)
        a_.set_xlabel(r"Lease threshold $\varepsilon$")
        a_.set_ylabel(yl)
        a_.set_title(ttl, fontsize=8)
        a_.legend(frameon=False, fontsize=6, loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_lease_model.pdf"))
    plt.close(fig)
    mac("LmTaOne", t_a1, "{:.1f}")
    mac("LmOutages", len(outs), "{:d}")
    mac("LmShortErr", max(err["short"]), "{:.2f}")
    mac("LmOverErr", max(err["over"]), "{:.2f}")
    mac("LmInCi", inside, "{:d}")
    mac("LmNPts", 2 * len(sim), "{:d}")
    mac("LmFdef", 100 * f_def, "{:.1f}")
    mac("LmFupStale", 100 * f_up, "{:.1f}")
    p2 = pred(sim[0.2]["tau"]) if 0.2 in sim else None
    if p2:
        mac("LmSafePredMid", p2["safe"], "{:.1f}")
        mac("LmSafeSimMid", sim[0.2]["safe"][0], "{:.1f}")


RECENT = [("onboard", "SpaceOran", "Placement of Space-O-RAN"),
          ("pred-ho", "Baranda", "Migration of Baranda \\emph{et al.}"),
          ("ground-pre", "Kube", "Pre-sync of KubeSpace")]


def analyze_recent(main):
    """PAL against baselines that take their placement from recent designs. The placement of
    Space-O-RAN is the onboard row of the main group; the other two come from the group
    'recent' (pred-ho, ground-pre). Their intent handling is our assumption, the A1 default."""
    rec = load("recent")
    data = dict(main)
    data.update(rec)
    if not rec:
        return
    lay = [(550, 1, "FiveGOne"), (550, 2, "FiveGTwo"), (1200, 1, "TwelveGOne")]
    rows = []
    cutlist = []
    allpos = defaultdict(list)
    for shell, gw, L in lay:
        pal = f"pal_s{shell}_gw{gw}"
        for strat, tag, name in RECENT + [("pal", "Pal", "PAL")]:
            c = f"{strat}_s{shell}_gw{gw}"
            if c not in data or pal not in data:
                continue
            def f(k, sc, nd, pm=False):
                m, h, _ = ci(col(data, c, k) * sc)
                return f"{m:.{nd}f}$\\pm${h:.{nd}f}" if pm else f"{m:.{nd}f}"
            nfmiss = 100 - ci(col(data, c, "nf_hit_ratio") * 100)[0]
            rows.append((shell, gw, name, [f"{nfmiss:.2f}", f("sla_viol_frac", 100, 1, True),
                                           f("short_depth_mean", 100, 2), f("overprov_mean", 100, 2),
                                           f("superseded_frac", 100, 1), f"{ci(blackout(data, c))[0]:.0f}",
                                           f("feeder_kbps", 1, 1), f(bcol(T_DEF, G_DEF), 100, 2)]))
            mac(f"Rec{tag}{L}NfMiss", nfmiss, "{:.2f}")
            mac(f"Rec{tag}{L}Short", ci(col(data, c, "sla_viol_frac") * 100)[0], "{:.1f}")
            mac(f"Rec{tag}{L}Depth", ci(col(data, c, "short_depth_mean") * 100)[0], "{:.2f}")
            mac(f"Rec{tag}{L}Over", ci(col(data, c, "overprov_mean") * 100)[0], "{:.2f}")
            mac(f"Rec{tag}{L}Super", ci(col(data, c, "superseded_frac") * 100)[0], "{:.1f}")
            mac(f"Rec{tag}{L}Blk", ci(blackout(data, c))[0], "{:.0f}")
            mac(f"Rec{tag}{L}Kbps", ci(col(data, c, "feeder_kbps"))[0], "{:.1f}")
            mac(f"Rec{tag}{L}Beam", ci(col(data, c, bcol(T_DEF, G_DEF)) * 100)[0], "{:.2f}")
            if strat == "pal":
                continue
            # PAL minus design, paired by seed: shortfall, depth, NF misses, beam
            for key, nm, sc, fm in [("sla_viol_frac", "Short", 100, "{:.2f}"),
                                    ("short_depth_mean", "Depth", 100, "{:.3f}"),
                                    ("nf_hit_ratio", "NfHit", 100, "{:.2f}"),
                                    (bcol(T_DEF, G_DEF), "Beam", 100, "{:.2f}")]:
                d = paired_vec(data, pal, c, key) * sc
                m, h, _ = ci(d)
                mac(f"RecD{tag}{L}{nm}", m, fm)
                mac(f"RecD{tag}{L}{nm}Ci", h, fm)
                mac(f"RecD{tag}{L}{nm}P", sign_p(d), "{:.3f}")
                better = d > 0 if key == "nf_hit_ratio" else d < 0
                mac(f"RecD{tag}{L}{nm}Npos", int(np.sum(better)), "{:d}")
                allpos[nm].append(int(np.sum(better)))
            # relative shortfall cut against the design
            a, b = ci(col(data, pal, "sla_viol_frac"))[0], ci(col(data, c, "sla_viol_frac"))[0]
            mac(f"RecCut{tag}{L}Short", 100 * (b - a) / b if b > 0 else float("nan"), "{:.0f}")
            if tag != "Kube":
                cutlist.append(100 * (b - a) / b)
    with open(os.path.join(TAB, "table_recent.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n")
        t.write("\\begin{tabular}{@{}llrrrrrrrr@{}}\n\\toprule\n")
        t.write("Shell, GW & Design & NF miss & Short. & Depth & Over. & Super. & Blk. & Load & Beam\\\\\n")
        t.write(" & & (\\%) & (\\%) & (\\%) & (\\%) & (\\%) & (ms) & (kb/s) & (\\%)\\\\\n\\midrule\n")
        last = None
        for shell, gw, name, vals in rows:
            key = (shell, gw)
            lead = f"{shell} km, {gw}" if key != last else ""
            if last is not None and key != last:
                t.write("\\addlinespace[2pt]\n")
            t.write(f"{lead} & {name} & " + " & ".join(vals) + "\\\\\n")
            last = key
        t.write("\\bottomrule\n\\end{tabular}\n")
    mac("RecRuns", sum(len(v) for v in rec.values()), "{:d}")
    if cutlist:
        mac("RecCutMin", min(cutlist), "{:.0f}")
        mac("RecCutMax", max(cutlist), "{:.0f}")


def main():
    plt = plt_setup()
    analyze_hb(plt)
    m = analyze_main(plt)
    analyze_beam(plt, m or {})
    analyze_variants(m or {})
    analyze_eps(plt, m or {})
    analyze_lease_model(plt, m or {})
    analyze_recent(m or {})
    analyze_ablation(m or {})
    analyze_sens(m or {})
    import analyze_nov
    analyze_nov.run(sys.modules[__name__], plt, m or {})
    import analyze_r2
    analyze_r2.run(sys.modules[__name__], plt, m or {})
    write_macros()


if __name__ == "__main__":
    main()
