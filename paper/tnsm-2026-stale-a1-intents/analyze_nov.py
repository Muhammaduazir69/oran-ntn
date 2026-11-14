#!/usr/bin/env python3
"""Analysis added in the November 2026 revision (called from analyze.py).

Writes macros and tables for: the eight-layout table with Holm-adjusted tests, the value-of-lease
prediction (Corollary 3) across layouts and update rates, the T_A1 sweep with no lease, the rule, a
fixed timer and an immediate drop, the 40-seed cost contrast, the cell-site study, the beam stress
cases, the host test and the lead-time sweeps.
"""
import math
import os
from collections import defaultdict

import numpy as np

LAY8 = [(550, 1), (550, 2), (550, 3), (550, 4), (1200, 1), (1200, 2), (1200, 3), (1200, 4)]
WORDS = {1: "One", 2: "Two", 3: "Three", 4: "Four"}
SHELLW = {550: "Five", 1200: "Twelve"}
TA1 = [30, 60, 300, 900, 3600]
TAW = {30: "Thirty", 60: "Sixty", 300: "ThreeH", 900: "NineH", 3600: "Hour"}


def holm(pvals):
    """Holm step-down adjusted p-values, in the input order."""
    m = len(pvals)
    order = np.argsort(pvals)
    adj = [0.0] * m
    run = 0.0
    for r, i in enumerate(order):
        run = max(run, min(1.0, (m - r) * pvals[i]))
        adj[i] = run
    return adj


def sgn(x, nd=2):
    """signed number for a LaTeX table cell, with a math minus and no signed zero"""
    t = f"{x:+.{nd}f}"
    if float(t) == 0:
        return f"{0:.{nd}f}"
    return t.replace("-", "$-$")


def fmt_p(p):
    return "<0.001" if p < 0.001 else f"{p:.3f}"


def value_of_lease(outs, served, T, tau, r):
    """Corollary 3, on an empirical outage sample: shortfall time removed and over-provisioning added
    by a lease tau against holding forever, in points of served time."""
    ds = do = 0.0
    for th in outs:
        if th <= tau:
            continue
        over = th - tau
        G = over - T * (math.exp(-tau / T) - math.exp(-th / T))
        ds += (r["pc"] - r["ps"]) * over + (r["pv"] - r["pc"]) * G
        do += (r["os"] - r["oc"]) * over - (r["ov"] - r["oc"]) * G
    return 100 * ds / served, 100 * do / served


def closed_value(avail, theta, T, eps, r):
    """Corollary 3, exponential outages of mean theta: closed form (points)."""
    q = (1 - eps) ** (T / theta)
    z = (1 - eps) / (1 + theta / T)
    return (100 * (1 - avail) * q * ((r["pv"] - r["ps"]) - (r["pv"] - r["pc"]) * z),
            100 * (1 - avail) * q * ((r["os"] - r["ov"]) + (r["ov"] - r["oc"]) * z))


def outages_of(H, cfg):
    outs = [float(o["dur"]) for sd in H[cfg] for o in H[cfg][sd]["outages"] if float(o["start"]) > 0]
    served = sum(float(H[cfg][sd]["summary"]["served_s"]) for sd in H[cfg])
    return outs, served


def run(A, plt, main):
    mac, ci, col, load, paired_vec, sign_p = A.mac, A.ci, A.col, A.load, A.paired_vec, A.sign_p
    bcol, GAPS, DWELL = A.bcol, A.GAPS, A.DWELL
    TAB, FIG = A.TAB, A.FIG
    held = load("heldout")
    base = "reactive-ttl_s550_gw1_eps0.2"
    if base not in held or not main:
        return
    r0 = A.rates_from(held, base)
    w_a, w_b = A.ratio_for(r0, 0.2), A.ratio_for(r0, 0.5)
    mac("WeightA", w_a, "{:.2f}")
    mac("WeightB", w_b, "{:.2f}")

    # ---- eight layouts: lease on/off, PAL against O+L, PAL against R+L+P, with Holm
    rows, p_l, p_o, p_n, p_over = [], [], [], [], []
    pred_err, gains = [], []
    for shell, gw in LAY8:
        r_, l_, p_, o_, t_ = (f"reactive_s{shell}_gw{gw}", f"reactive-ttl_s{shell}_gw{gw}", f"pal_s{shell}_gw{gw}",
                              f"onboard-ttl_s{shell}_gw{gw}", f"reactive-ttl-pre_s{shell}_gw{gw}")
        if not all(c in main for c in [r_, l_, p_, o_, t_]):
            continue
        hc = base if (shell, gw) == (550, 1) else f"reactive-ttl_s{shell}_gw{gw}_eps0.2"
        d = paired_vec(main, l_, r_, "sla_viol_frac") * 100
        dv = paired_vec(main, l_, r_, "overprov_mean") * 100
        dpo = paired_vec(main, p_, o_, "sla_viol_frac") * 100
        dn = paired_vec(main, p_, t_, "nf_hit_ratio") * 100
        p_l.append(sign_p(d)); p_o.append(sign_p(dpo)); p_n.append(sign_p(dn)); p_over.append(sign_p(dv))
        pred = (float("nan"), float("nan"))
        theta = float("nan")
        if hc in held:
            outs, served = outages_of(held, hc)
            T = ci(col(held, hc, "lease_tau_s"))[0] / -math.log(1 - 0.2)
            pred = value_of_lease(outs, served, T, -T * math.log(1 - 0.2), r0)
            theta = float(np.mean(outs))
        avail = ci(col(main, r_, "feeder_avail") * 100)[0]
        mr, hr, _ = ci(col(main, r_, "sla_viol_frac") * 100)
        ml = ci(col(main, l_, "sla_viol_frac") * 100)[0]
        rows.append(dict(shell=shell, gw=gw, avail=avail, theta=theta, none=mr, lease=ml, d=ci(d), dv=ci(dv),
                         low=int(np.sum(d < 0)), pred=pred, dpo=ci(dpo), dn=ci(dn), cut=100 * (mr - ml) / mr))
        tg = f"{SHELLW[shell]}G{WORDS[gw]}"
        mac(f"LayCut{tg}", 100 * (mr - ml) / mr, "{:.0f}")
        mac(f"LayAvail{tg}", avail, "{:.1f}")
        mac(f"LayD{tg}", -ci(d)[0], "{:.2f}")
        mac(f"LayDCi{tg}", ci(d)[1], "{:.2f}")
        mac(f"LayOver{tg}", ci(dv)[0], "{:.2f}")
        mac(f"LayNone{tg}", mr, "{:.1f}")
        mac(f"LayLease{tg}", ml, "{:.1f}")
        mac(f"LayLow{tg}", int(np.sum(d < 0)), "{:d}")
        mac(f"LayTheta{tg}", theta, "{:.1f}")
        mac(f"LayPred{tg}", pred[0], "{:.2f}")
        if hc in held:
            pred_err.append(abs(pred[0] + ci(d)[0]))
            gains.append((avail, -ci(d)[0], ci(d)[1], pred[0], shell, gw))
    if rows:
        hl, ho, hn = holm(p_l), holm(p_o), holm(p_n)
        for i, R in enumerate(rows):
            R["pl"], R["po"], R["pn"] = hl[i], ho[i], hn[i]
            tg = f"{SHELLW[R['shell']]}G{WORDS[R['gw']]}"
            mac(f"LayHolm{tg}", fmt_p(hl[i]), "{}")
            mac(f"LayPoHolm{tg}", fmt_p(ho[i]), "{}")
            mac(f"LayNfHolm{tg}", fmt_p(hn[i]), "{}")
        sig = [R for R in rows if R["pl"] < 0.05]
        mac("LaySigN", len(sig), "{:d}")
        mac("LayN", len(rows), "{:d}")
        if sig:
            mac("LayCutSigMin", min(R["cut"] for R in sig), "{:.0f}")
            mac("LayCutSigMax", max(R["cut"] for R in sig), "{:.0f}")
            mac("LayAvailSigMax", max(R["avail"] for R in sig), "{:.1f}")
        nsig = [R for R in rows if R["pl"] >= 0.05]
        if nsig:
            mac("LayAvailNsMin", min(R["avail"] for R in nsig), "{:.1f}")
            mac("LayCutNsMin", min(R["cut"] for R in nsig), "{:.0f}")
            mac("LayCutNsMax", max(R["cut"] for R in nsig), "{:.0f}")
        mac("LayPoSigN", sum(R["po"] < 0.05 for R in rows), "{:d}")
        mac("LayNsN", len(nsig), "{:d}")
        mac("LayPoMaxAbs", max(abs(R["dpo"][0]) for R in rows), "{:.2f}")
        sg = [R["dpo"][0] for R in rows if R["po"] < 0.05]
        mac("LayPoMaxSig", max(sg) if sg else float("nan"), "{:.2f}")
        mac("LayNfSigN", sum(R["pn"] < 0.05 for R in rows), "{:d}")
        if pred_err:
            mac("LayPredMaxErr", max(pred_err), "{:.2f}")
            mac("LayPredInCi", sum(abs(R["pred"][0] + R["d"][0]) <= R["d"][1] for R in rows), "{:d}")
        with open(os.path.join(TAB, "table_layouts.tex"), "w") as t:
            t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrrrrr@{}}\n\\toprule\n")
            t.write("Shell, GW & Avail. & $\\bar\\Theta$ & No lease & Lease & Lease effect & Pred. & Holm $p$ & Added over. "
                    "& PAL$-$O+L & Holm $p$\\\\\n")
            t.write(" & (\\%) & (s) & (\\%) & (\\%) & (points) & (points) & & (points) & (points) & \\\\\n\\midrule\n")
            for R in rows:
                t.write(f"{R['shell']} km, {R['gw']} & {R['avail']:.1f} & {R['theta']:.1f} & {R['none']:.2f} & {R['lease']:.2f} & "
                        f"{sgn(R['d'][0])}$\\pm${R['d'][1]:.2f} & {sgn(-R['pred'][0])} & {fmt_p(R['pl'])} & "
                        f"{sgn(R['dv'][0])}$\\pm${R['dv'][1]:.2f} & {sgn(R['dpo'][0])}$\\pm${R['dpo'][1]:.2f} & {fmt_p(R['po'])}\\\\\n")
            t.write("\\bottomrule\n\\end{tabular}\n")
        # all NF gains, adjusted
        wr = [R for R in rows if R["pn"] >= 0.05]
        mac("LayNfFail", ", ".join(f"{R['shell']}~km with {R['gw']}~GW" for R in wr) if wr else "none", "{}")
        wr = [R for R in rows if R["po"] < 0.05]
        mac("LayPoFail", ", ".join(f"{R['shell']}~km with {R['gw']}~GW" for R in wr) if wr else "none", "{}")

    # ---- SSP against PAL in the layouts where Table main omits its row
    mx = 0.0
    for shell, gw in [(550, 2), (1200, 1)]:
        a, b = f"sat-pred_s{shell}_gw{gw}", f"pal_s{shell}_gw{gw}"
        if a in main and b in main:
            for k, sc in [("rt_hit_ratio", 100), ("nf_hit_ratio", 100), ("nf_p99_ms", 1), ("sla_viol_frac", 100),
                          ("short_depth_mean", 100), ("overprov_mean", 100), ("superseded_frac", 100),
                          ("feeder_kbps", 1), (bcol(2.0, 0.015), 100)]:
                mx = max(mx, abs(ci(col(main, a, k) * sc)[0] - ci(col(main, b, k) * sc)[0]))
    mac("SspMaxDiff", mx, "{:.1f}")
    mac("FixedTau", -60 * math.log(0.8), "{:.1f}")
    # all groups: total runs and one binary
    groups = ["main", "heldout", "recent", "ta1", "costseeds", "eps", "sites", "beamstress", "host", "leadsweep",
              "ablation", "a1proc", "hb", "sens", "variants", "step"]
    tot, shas = 0, set()
    for g_ in groups:
        for cfg, by in load(g_).items():
            tot += len(by)
            for sd in by:
                shas.add(by[sd]["summary"].get("binary_sha"))
    mac("TotRuns", f"{tot:,}".replace(",", "{,}"), "{}")
    mac("BinaryCount", len(shas), "{:d}")
    print("binaries:", shas)

    # ---- T_A1 sweep
    ta = load("ta1")
    pts = []
    if ta:
        fig, axes = plt.subplots(1, 3, figsize=(7.1, 2.05))
        STY = {"none": ("No lease", A.BLUE, "o", "--"), "rule": ("Rule", A.GOLD, "D", "-"),
               "fixed": ("Fixed timer", A.NAVY, "v", "-."), "drop": ("Immediate drop", A.GRAY, "s", ":")}
        s, g = 550, 1
        for name, (lab, cl, mk, ls) in STY.items():
            xs, ys, es, ov, oe = [], [], [], [], []
            for T_ in TA1:
                c = f"{name}_s{s}_gw{g}_ta{T_}"
                if c in ta:
                    m, h, _ = ci(col(ta, c, "sla_viol_frac") * 100)
                    o, q, _ = ci(col(ta, c, "overprov_mean") * 100)
                    xs.append(T_); ys.append(m); es.append(h); ov.append(o); oe.append(q)
            axes[0].errorbar(xs, ys, yerr=es, color=cl, marker=mk, ms=3, ls=ls, lw=0.9, capsize=1.5, label=lab)
            axes[1].errorbar(xs, ov, yerr=oe, color=cl, marker=mk, ms=3, ls=ls, lw=0.9, capsize=1.5, label=lab)
        for ax, yl, ttl in [(axes[0], "Shortfall time (%)", "(a) Intent shortfall"),
                            (axes[1], "Over-provisioning (% of cell)", "(b) Over-provisioning")]:
            ax.set_xscale("log")
            ax.set_xticks(TA1)
            ax.set_xticklabels([str(x) for x in TA1])
            ax.minorticks_off()
            ax.set_xlabel(r"Mean update spacing $T_{\mathrm{A1}}$ (s)")
            ax.set_ylabel(yl)
            ax.set_title(ttl, fontsize=8)
        axes[0].legend(frameon=False, fontsize=6.2)
        # shortfall removed by the rule: simulation against Corollary 3, both shells
        hcs = {(550, 1): base, (1200, 1): "reactive-ttl_s1200_gw1_eps0.2"}
        ax = axes[2]
        errs, inside, n = [], 0, 0
        for (s_, g_), cl, mk in [((550, 1), A.GOLD, "D"), ((1200, 1), A.NAVY, "o")]:
            xs, ys, es, pp, cf = [], [], [], [], []
            if hcs[(s_, g_)] not in held:
                continue
            outs, served = outages_of(held, hcs[(s_, g_)])
            th = float(np.mean(outs))
            av = 1 - sum(outs) / served
            for T_ in TA1:
                a, b = f"rule_s{s_}_gw{g_}_ta{T_}", f"none_s{s_}_gw{g_}_ta{T_}"
                if a not in ta or b not in ta:
                    continue
                d = -paired_vec(ta, a, b, "sla_viol_frac") * 100
                m, h, _ = ci(d)
                tau = -T_ * math.log(0.8)
                p = value_of_lease(outs, served, T_, tau, r0)[0]
                xs.append(T_); ys.append(m); es.append(h); pp.append(p)
                errs.append(abs(p - m)); inside += abs(p - m) <= h; n += 1
                pts.append((s_, g_, T_, m, h, p))
                cf.append(closed_value(av, th, T_, 0.2, r0)[0])
            lab = f"{s_} km"
            ax.errorbar(xs, ys, yerr=es, color=cl, marker=mk, ms=3, ls="none", capsize=1.5, lw=0.8, mfc="white",
                        label=f"{lab}, simulation")
            ax.plot(xs, pp, color=cl, lw=0.9, label=f"{lab}, Cor. S2")
            ax.plot(xs, cf, color=cl, lw=0.7, ls=":")
            mac(f"TaCfSixty{A.ROMAN[s_]}", cf[1] if len(cf) > 1 else float("nan"), "{:.2f}")
        ax.set_xscale("log")
        ax.set_xticks(TA1)
        ax.set_xticklabels([str(x) for x in TA1])
        ax.minorticks_off()
        ax.axhline(0, color="0.6", lw=0.5)
        ax.set_xlabel(r"Mean update spacing $T_{\mathrm{A1}}$ (s)")
        ax.set_ylabel("Shortfall removed (points)")
        ax.set_title("(c) Value of the lease", fontsize=8)
        ax.legend(frameon=False, fontsize=6)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, "fig_ta1.pdf"))
        plt.close(fig)
        if errs:
            mac("TaPredMaxErr", max(errs), "{:.2f}")
            mac("TaPredInCi", inside, "{:d}")
            mac("TaPredN", n, "{:d}")
        # per T_A1: the policies at the two weights
        rows = []
        for s_, g_ in [(550, 1), (1200, 1)]:
            for T_ in TA1:
                cs = {k: f"{k}_s{s_}_gw{g_}_ta{T_}" for k in STY}
                if not all(c in ta for c in cs.values()):
                    continue
                tg = f"{SHELLW[s_]}{TAW[T_]}"
                cell = {}
                for k, c in cs.items():
                    sv, ov = ci(col(ta, c, "sla_viol_frac") * 100)[0], ci(col(ta, c, "overprov_mean") * 100)[0]
                    mac(f"Ta{k.capitalize()}Short{tg}", sv, "{:.2f}")
                    mac(f"Ta{k.capitalize()}Over{tg}", ov, "{:.2f}")
                    cell[k] = (sv, ov)
                J = {k: col(ta, c, "sla_viol_frac") * 100 + w_a * col(ta, c, "overprov_mean") * 100 for k, c in cs.items()}
                pr = {}
                for k in ["none", "fixed", "drop"]:
                    d = J[k] - J["rule"]
                    m, h, _ = ci(d)
                    pr[k] = (m, h, sign_p(d))
                    mac(f"TaJ{k.capitalize()}{tg}", m, "{:+.2f}")
                    mac(f"TaJ{k.capitalize()}Ci{tg}", h, "{:.2f}")
                    mac(f"TaJ{k.capitalize()}P{tg}", sign_p(d), "{:.3f}")
                dd = paired_vec(ta, cs["rule"], cs["none"], "sla_viol_frac") * 100
                dv = paired_vec(ta, cs["rule"], cs["none"], "overprov_mean") * 100
                mac(f"TaD{tg}", -ci(dd)[0], "{:.2f}")
                mac(f"TaDCi{tg}", ci(dd)[1], "{:.2f}")
                mac(f"TaDOver{tg}", ci(dv)[0], "{:.2f}")
                mac(f"TaDLow{tg}", int(np.sum(dd < 0)), "{:d}")
                tau_end = ci(col(ta, cs["rule"], "lease_tau_s"))[0]
                mac(f"TaTau{tg}", tau_end, "{:.0f}")
                rows.append((s_, T_, tau_end, cell, pr))
        # Holm over the rule against the fixed timer and no lease, per shell and comparison
        if rows:
            for k in ["none", "fixed", "drop"]:
                adj = holm([R[4][k][2] for R in rows])
                for R, a_ in zip(rows, adj):
                    R[4][k] = R[4][k] + (a_,)
            with open(os.path.join(TAB, "table_ta1.tex"), "w") as t:
                t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrrrrrr@{}}\n\\toprule\n")
                t.write(" & & & \\multicolumn{4}{c}{Shortfall (\\%) / over-provisioning (\\%)} & \\multicolumn{3}{c}{Cost excess over the rule (points)}\\\\\n")
                t.write("\\cmidrule(lr){4-7}\\cmidrule(l){8-10}\n")
                t.write("Shell & $T_{\\mathrm{A1}}$ (s) & $\\tau^\\ast$ (s) & No lease & Rule & Fixed & Drop & No lease & Fixed & Drop\\\\\n\\midrule\n")
                last = None
                for s_, T_, tau_, cell, pr in rows:
                    lead = f"{s_} km" if s_ != last else ""
                    if last is not None and s_ != last:
                        t.write("\\addlinespace[2pt]\n")
                    star = lambda q: "$^{*}$" if q[3] < 0.05 else ""
                    t.write(f"{lead} & {T_} & {tau_:.0f} & " +
                            " & ".join(f"{cell[k][0]:.2f} / {cell[k][1]:.2f}" for k in ["none", "rule", "fixed", "drop"]) + " & " +
                            " & ".join(f"{sgn(pr[k][0])}$\\pm${pr[k][1]:.2f}{star(pr[k])}" for k in ["none", "fixed", "drop"]) + "\\\\\n")
                    last = s_
                t.write("\\bottomrule\n\\end{tabular}\n")
            mac("TaRuleBeatsFixedN", sum(R[4]["fixed"][0] > 0 and R[4]["fixed"][3] < 0.05 for R in rows), "{:d}")
            mac("TaFixedBeatsRuleN", sum(R[4]["fixed"][0] < 0 and R[4]["fixed"][3] < 0.05 for R in rows), "{:d}")
            mac("TaRuleBeatsNoneN", sum(R[4]["none"][0] > 0 and R[4]["none"][3] < 0.05 for R in rows), "{:d}")
            mac("TaNoneBeatsRuleN", sum(R[4]["none"][0] < 0 and R[4]["none"][3] < 0.05 for R in rows), "{:d}")
            mac("TaRows", len(rows), "{:d}")
            ho = [R for R in rows if R[0] == 550 and R[1] == 3600][0]
            mac("TaFixedMinusRuleOverHour", ho[3]["fixed"][1] - ho[3]["rule"][1], "{:.2f}")
            with open(os.path.join(TAB, "ta1_holm.txt"), "w") as f:
                for R in rows:
                    f.write(f"{R[0]} {R[1]} " + " ".join(f"{k}:{R[4][k][0]:+.3f}:{R[4][k][3]:.3f}" for k in ["none", "fixed", "drop"]) + "\n")
        c = "rule_s550_gw1_ta60_prior600"
        if c in ta and "rule_s550_gw1_ta60" in ta:
            mac("TaPriorShort", ci(col(ta, c, "sla_viol_frac") * 100)[0], "{:.2f}")
            mac("TaPriorOver", ci(col(ta, c, "overprov_mean") * 100)[0], "{:.2f}")
            mac("TaPriorTau", ci(col(ta, c, "lease_tau_s"))[0], "{:.1f}")
            m, h, _ = ci(paired_vec(ta, c, "rule_s550_gw1_ta60", "sla_viol_frac") * 100)
            mac("TaPriorD", m, "{:+.2f}")
            mac("TaPriorDCi", h, "{:.2f}")
        mac("TaRuns", sum(len(v) for v in ta.values()), "{:d}")

    # ---- the cost contrast with 40 more seeds
    cs = load("costseeds")
    if cs and "predho" in cs:
        for eps, w, tg in [(0.2, w_a, "A"), (0.5, w_b, "B")]:
            c = f"pal_eps{eps}"
            if c not in cs:
                continue
            Jn = col(cs, "predho", "sla_viol_frac") * 100 + w * col(cs, "predho", "overprov_mean") * 100
            Jp = col(cs, c, "sla_viol_frac") * 100 + w * col(cs, c, "overprov_mean") * 100
            nn = min(len(Jn), len(Jp))
            d = Jn[:nn] - Jp[:nn]
            m, h, n = ci(d)
            mac(f"CsD{tg}", m, "{:+.2f}")
            mac(f"CsDCi{tg}", h, "{:.2f}")
            mac(f"CsPos{tg}", int(np.sum(d > 0)), "{:d}")
            mac(f"CsP{tg}", sign_p(d), "{:.3f}")
            mac(f"CsP{tg}Txt", "<0.001" if sign_p(d) < 0.001 else f"={sign_p(d):.3f}", "{}")
            mac(f"CsWil{tg}", A.wilcoxon_p(d), "{:.3f}")
            mac(f"CsN{tg}", n, "{:d}")
            ds = paired_vec(cs, c, "predho", "sla_viol_frac") * 100
            dv = paired_vec(cs, c, "predho", "overprov_mean") * 100
            mac(f"CsShort{tg}", ci(ds)[0], "{:+.2f}")
            mac(f"CsShortCi{tg}", ci(ds)[1], "{:.2f}")
            mac(f"CsOver{tg}", ci(dv)[0], "{:+.2f}")
            mac(f"CsOverCi{tg}", ci(dv)[1], "{:.2f}")
        mac("CsRuns", sum(len(v) for v in cs.values()), "{:d}")

    # ---- cell sites
    st = load("sites")
    if st:
        NAMES = [("a", "15N 60E"), ("b", "10N 68E"), ("c", "24N 62E"), ("atl", "35N 40W")]
        rows, ps = [], []
        all_av = []
        for nm, lab in NAMES:
            for gw in [1, 2]:
                r_, l_, p_, o_ = [f"{x}_site{nm}_gw{gw}" for x in ["reactive", "reactive-ttl", "pal", "onboard-ttl"]]
                if not all(c in st for c in [r_, l_, p_, o_]):
                    continue
                d = paired_vec(st, l_, r_, "sla_viol_frac") * 100
                dv = paired_vec(st, l_, r_, "overprov_mean") * 100
                av = ci(col(st, r_, "feeder_avail") * 100)[0]
                dseeds = np.array([np.nan])
                mean_d = [float(np.mean(paired_vec(st, p_, o_, bcol(2.0, gp)))) for gp in GAPS]
                cross = A.crossover_gap(mean_d) * 100
                ps.append(sign_p(d))
                rows.append([lab, gw, av, ci(col(st, r_, "sla_viol_frac") * 100)[0], ci(col(st, l_, "sla_viol_frac") * 100)[0], ci(d), ci(dv), cross, ci(col(st, p_, "nf_hit_ratio") * 100)[0]])
                all_av.append(av)
        if rows:
            adj = holm(ps)
            with open(os.path.join(TAB, "table_sites.tex"), "w") as t:
                t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrr@{}}\n\\toprule\n")
                t.write("Cell, GW & Avail. & No lease & Lease & Effect & Holm $p$ & Added over. & $\\delta^\\ast$\\\\\n")
                t.write(" & (\\%) & (\\%) & (\\%) & (points) & & (points) & (points)\\\\\n\\midrule\n")
                for R, a_ in zip(rows, adj):
                    t.write(f"{R[0]}, {R[1]} & {R[2]:.1f} & {R[3]:.2f} & {R[4]:.2f} & {sgn(R[5][0])}$\\pm${R[5][1]:.2f} & {fmt_p(a_)} & {sgn(R[6][0])} & {R[7]:.2f}\\\\\n")
                t.write("\\bottomrule\n\\end{tabular}\n")
            mac("SiteAvailMin", min(all_av), "{:.1f}")
            mac("SiteAvailMax", max(all_av), "{:.1f}")
            mac("SiteSigN", sum(a_ < 0.05 for a_ in adj), "{:d}")
            mac("SiteN", len(rows), "{:d}")
            xs = [R[7] for R in rows if not math.isnan(R[7])]
            mac("SiteCrossMin", min(xs), "{:.2f}")
            mac("SiteCrossMax", max(xs), "{:.2f}")
            mac("SiteRuns", sum(len(v) for v in st.values()), "{:d}")
            lo = min(rows, key=lambda R: R[2])
            mac("SiteLoAvail", lo[2], "{:.1f}")
            mac("SiteLoNone", lo[3], "{:.1f}")
            mac("SiteLoLease", lo[4], "{:.1f}")
            mac("SiteLoOver", lo[6][0], "{:.1f}")

    # ---- beam stress: the break-even under processes Lemma 1 does not assume
    bs = load("beamstress")
    if bs:
        ref = float(A.macros_num("BeamCrossFiveGOneTTwo"))
        for nm, tg in [("erl", "Erl"), ("hyp", "Hyp"), ("drift", "Drift"), ("corr", "Corr")]:
            p_, o_ = f"pal_s550_gw1_{nm}", f"onboard-ttl_s550_gw1_{nm}"
            if p_ not in bs or o_ not in bs:
                continue
            for T, tn in [(2.0, "Two"), (0.5, "Half")]:
                mean_d = [float(np.mean(paired_vec(bs, p_, o_, bcol(T, gp)))) for gp in GAPS]
                mac(f"BsCross{tg}T{tn}", A.crossover_gap(mean_d) * 100, "{:.2f}")
            mac(f"BsMis{tg}", ci(col(bs, p_, bcol(2.0, 0.0)) * 100)[0], "{:.2f}")
        mac("BsRuns", sum(len(v) for v in bs.values()), "{:d}")

    # ---- host test: PAL with NF pinned on board when delta < delta*
    hs = load("host")
    if hs:
        mac("HostRuns", sum(len(v) for v in hs.values()), "{:d}")
        for gw in [1, 2]:
            tg = f"FiveG{WORDS[gw]}"
            c, p_, o_ = f"pal-pin_s550_gw{gw}", f"pal_s550_gw{gw}", f"onboard-ttl_s550_gw{gw}"
            if c not in hs or p_ not in main:
                continue
            mac(f"HostNfHit{tg}", ci(col(hs, c, "nf_hit_ratio") * 100)[0], "{:.2f}")
            mac(f"HostKbps{tg}", ci(col(hs, c, "feeder_kbps"))[0], "{:.1f}")
            mac(f"HostKbpsPal{tg}", ci(col(main, p_, "feeder_kbps"))[0], "{:.1f}")
            mac(f"HostShort{tg}", ci(col(hs, c, "sla_viol_frac") * 100)[0], "{:.2f}")
            d = np.array([float(hs[c][s]["summary"][bcol(2.0, 0.0)]) - float(main[o_][s]["summary"][bcol(2.0, 0.0)]) for s in sorted(hs[c])]) * 100
            mac(f"HostVsObl{tg}", ci(d)[0], "{:+.3f}")
            mac(f"HostVsOblCi{tg}", ci(d)[1], "{:.3f}")
            for dl, tn in [(0.0, "Zero"), (0.015, "Default"), (0.05, "Five")]:
                x = np.array([float(hs[c][s]["summary"][bcol(2.0, dl)]) - float(main[p_][s]["summary"][bcol(2.0, dl)]) for s in sorted(hs[c])]) * 100
                mac(f"HostD{tn}{tg}", ci(x)[0], "{:+.2f}")
                mac(f"HostD{tn}Ci{tg}", ci(x)[1], "{:.2f}")
                mac(f"HostD{tn}Low{tg}", int(np.sum(x < 0)), "{:d}")
                mac(f"HostD{tn}P{tg}", sign_p(x), "{:.3f}")

    # ---- lead-time, provisioning and slack sweeps
    ls = load("leadsweep")
    if ls:
        rows = []
        for lead, c, tag in [(0.5, "pal_s550_gw1_lead0.5", "LeadHalf"), (1.0, "pal_s550_gw1_lead1.0", "LeadOne"),
                             (2.0, None, "LeadTwo"), (5.0, "pal_s550_gw1_lead5.0", "LeadFive"),
                             (10.0, "pal_s550_gw1_lead10.0", "LeadTen")]:
            src = ls if c else main
            c = c or "pal_s550_gw1"
            if c not in src:
                continue
            mac(f"{tag}Nf", ci(col(src, c, "nf_hit_ratio") * 100)[0], "{:.2f}")
            mac(f"{tag}Leases", ci(col(src, c, "leases"))[0], "{:.1f}")
            mac(f"{tag}Short", ci(col(src, c, "sla_viol_frac") * 100)[0], "{:.2f}")
            mac(f"{tag}OnbFrac", ci(col(src, c, "nf_onboard_frac") * 100)[0], "{:.1f}")
            rows.append(lead)
        for pre, tag in [(5.0, "PreFive"), (20.0, "PreTwenty"), (120.0, "PreTwoH")]:
            c = f"pal_s550_gw1_pre{pre}"
            if c in ls:
                mac(f"{tag}Blk", ci(A.blackout(ls, c))[0], "{:.0f}")
                mac(f"{tag}Kbps", ci(col(ls, c, "feeder_kbps"))[0], "{:.1f}")
                mac(f"{tag}Nf", ci(col(ls, c, "nf_hit_ratio") * 100)[0], "{:.2f}")
        for k, tag in [(0.0, "KappaZero"), (0.5, "KappaHalf")]:
            c = f"pal_s1200_gw1_kappa{k}"
            if c in ls:
                mac(f"{tag}Nf", ci(col(ls, c, "nf_hit_ratio") * 100)[0], "{:.2f}")
        mac("LeadRuns", sum(len(v) for v in ls.values()), "{:d}")

    # ---- intents that move one step
    sp = load("step")
    if sp:
        for T_, tn in [(60, "Sixty"), (300, "ThreeH")]:
            a, b = f"rule_s550_gw1_ta{T_}", f"none_s550_gw1_ta{T_}"
            if a in sp and b in sp:
                d = paired_vec(sp, a, b, "sla_viol_frac") * 100
                dv = paired_vec(sp, a, b, "overprov_mean") * 100
                mac(f"StepD{tn}", -ci(d)[0], "{:.2f}")
                mac(f"StepDCi{tn}", ci(d)[1], "{:.2f}")
                mac(f"StepOver{tn}", ci(dv)[0], "{:.2f}")
                mac(f"StepNone{tn}", ci(col(sp, b, "sla_viol_frac") * 100)[0], "{:.2f}")
        mac("StepRuns", sum(len(v) for v in sp.values()), "{:d}")

    # ---- non-Poisson updates: largest excess of the predicted lease over the best grid value
    xs = [A.macros_num(f"Cost{p}ExcessBest{t}") for p in ["Per", "Bur"] for t in ["A", "B"]]
    xs = [x for x in xs if not math.isnan(x)]
    if xs:
        mac("CostNonPoisExcess", max(xs), "{:.3f}")
    p_ = os.path.join(A.HERE, "runtime.txt")
    if os.path.exists(p_):
        mac("RunCpuS", float(open(p_).read().split()[0]), "{:.1f}")

    # ---- sensitivity of the rule to the strained rate p_v (per-seed range of the held-out runs)
    pvs = []
    for sd in held[base]:
        ck = held[base][sd]["checks"][0]
        if float(ck["samp_stale"]) > 0:
            pvs.append(float(ck["viol_stale"]) / float(ck["samp_stale"]))
    if pvs:
        taus = []
        for pv in (min(pvs), max(pvs)):
            rr = dict(r0, pv=pv)
            e = A.eps_ext(rr, w_a)
            taus.append((e, -60 * math.log(1 - min(0.999, max(0.0, e)))))
        mac("PvEpsLo", taus[1][0], "{:.2f}")
        mac("PvEpsHi", taus[0][0], "{:.2f}")
        mac("PvTauLo", taus[1][1], "{:.1f}")
        mac("PvTauHi", taus[0][1], "{:.1f}")

    # ---- the grid artifact is gone: per-seed break-even
    a, o = "pal_s550_gw1", "onboard-ttl_s550_gw1"
    if a in main and o in main:
        per, ph = [], []
        for sd in sorted(set(main[a]) & set(main[o])):
            dv = [float(main[a][sd]["summary"][bcol(2.0, gp)]) - float(main[o][sd]["summary"][bcol(2.0, gp)]) for gp in GAPS]
            per.append(A.crossover_gap(dv) * 100)
            miss = main[a][sd]["nf_miss"]
            if miss:
                ph.append(float(miss[0]["t"]) % 0.1)
        m, h, _ = ci(per)
        mac("SeedCrossMean", m, "{:.2f}")
        mac("SeedCrossCi", h, "{:.2f}")
        mac("SeedCrossMin", min(per), "{:.2f}")
        mac("SeedCrossMax", max(per), "{:.2f}")
        if len(ph) == len(per):
            from scipy.stats import spearmanr
            cc = max(abs(spearmanr((np.array(ph) + s) % 0.01, per).statistic) for s in np.arange(0, 0.01, 0.0005))
            mac("SeedCrossPhaseRho", cc, "{:.2f}")
