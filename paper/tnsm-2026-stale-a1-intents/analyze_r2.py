#!/usr/bin/env python3
"""Second-round analysis (called from analyze.py after analyze_nov). Tests that can fail: prediction error
with intervals on both terms of Corollary 3, explicit null rivals, the domain of the law at the sites, the
net value, the two-branch rule, and the break-even with its blackout term.
"""
import csv
import glob
import math
import os

import numpy as np

W = {1: "One", 2: "Two", 3: "Three", 4: "Four"}
SH = {550: "Five", 1200: "Twelve"}
TN = {30: "Thirty", 60: "Sixty", 300: "ThreeH", 900: "NineH", 3600: "Hour"}
SITE = {"a": "15N 60E", "b": "10N 68E", "c": "24N 62E", "atl": "35N 40W"}
LAUNCH = (0.3, 0.2)  # version 1 held by every satellite from t = 0


def launch_rates():
    """shortfall and over-provisioning of the launch intent against a redrawn current intent, from the
    alphabet and the demand law alone (no simulation)."""
    rng = np.random.default_rng(20261104)
    d = rng.poisson(6, 2000000) * 0.05
    gs, hs = np.array([.2, .3, .4, .5]), np.array([.1, .2, .3])
    gn, hn = gs[rng.integers(0, 4, len(d))], hs[rng.integers(0, 3, len(d))]
    r = np.minimum(.9, np.maximum(gn, d * (1 + hn)))
    x = np.minimum(.9, np.maximum(LAUNCH[0], d * (1 + LAUNCH[1])))
    return float(np.mean(x < r - .01)), float(np.mean(np.clip(x - r, 0, None)))


def seed_sums(rec, T, tau, est=False):
    """per-seed exposure sums. A, H: outages whose snapshot was current at onset; A3, H3: outages on a
    satellite never contacted, whose launch copy has age equal to the onset time. With est=True the lease of
    an outage is the one the RIC's estimator holds at its onset, tau = -ln(0.8) (T + t) / (1 + n), averaged
    over n ~ Poisson(t / T) updates seen; the launch copy keeps the lease set at t = 0."""
    from scipy.stats import poisson
    A = H = A3 = H3 = A0 = H0 = 0.0
    k = -math.log(0.8)
    for o in rec["outages"]:
        th, st_, kind = float(o["dur"]), float(o["start"]), int(o["kind"])
        if th > tau and st_ > 0:
            # the form with every snapshot current at onset, kept to show where it fails
            A0 += th - tau
            H0 += (th - tau) - T * (math.exp(-tau / T) - math.exp(-th / T))
        if kind in (0, 3):
            lo, hi = max(tau, st_), st_ + th
            if hi > lo:
                A3 += hi - lo
                H3 += (hi - lo) - T * (math.exp(-lo / T) - math.exp(-hi / T))
        elif est and tau > 0:
            mu = st_ / T
            ns = np.arange(int(max(0, mu - 6 * math.sqrt(mu + 1))), int(mu + 6 * math.sqrt(mu + 1)) + 2)
            w = poisson.pmf(ns, mu)
            ts = k * (T + st_) / (1 + ns)
            ov = np.clip(th - ts, 0, None)
            G = ov - T * (np.exp(-ts / T) - math.exp(-th / T))
            G = np.where(th > ts, G, 0.0)
            A += float((w * ov).sum())
            H += float((w * G).sum())
        elif th > tau:
            A += th - tau
            H += (th - tau) - T * (math.exp(-tau / T) - math.exp(-th / T))
    return A, H, A3, H3, float(rec["summary"]["served_s"]), A0, H0


def law(sums, r, pl, ol, ext=True):
    A, H, A3, H3, sv, A0, H0 = sums.sum(axis=0)
    if not ext:
        return (100 * ((r["pc"] - r["ps"]) * A0 + (r["pv"] - r["pc"]) * H0) / sv,
                100 * ((r["os"] - r["oc"]) * A0 - (r["ov"] - r["oc"]) * H0) / sv)
    S = (r["pc"] - r["ps"]) * A + (r["pv"] - r["pc"]) * H
    O = (r["os"] - r["oc"]) * A - (r["ov"] - r["oc"]) * H
    if ext:
        S += (r["pc"] - r["ps"]) * A3 + (pl - r["pc"]) * H3
        O += (r["os"] - r["oc"]) * A3 - (ol - r["oc"]) * H3
    return 100 * S / sv, 100 * O / sv


def run(A, plt, main):
    mac, ci, col, load, sign_p = A.mac, A.ci, A.col, A.load, A.sign_p
    from analyze_nov import holm, fmt_p, sgn, closed_value
    TAB, FIG = A.TAB, A.FIG
    held, c2, ta, tw, st, sh, lg, g0 = (load(g) for g in ["heldout", "c2", "ta1", "ta1w", "sites", "siteheld", "long", "g0"])
    base = "reactive-ttl_s550_gw1_eps0.2"
    if base not in held:
        return
    r0 = A.rates_from(held, base)
    w1 = A.ratio_for(r0, 0.2)
    pl, ol = launch_rates()
    mac("LawPl", 100 * pl, "{:.0f}")
    mac("LawOl", 100 * ol, "{:.1f}")
    mac("LaunchG", LAUNCH[0], "{:.1f}")
    mac("LaunchH", LAUNCH[1], "{:.1f}")
    rng = np.random.default_rng(20261105)
    seeds0 = sorted(held[base])
    boot_rates = [A.rates_from(held, base, list(rng.choice(seeds0, len(seeds0)))) for _ in range(800)]

    def full(D, c, n=40):
        return c in D and len(D[c]) >= n

    def cases():
        for shell, gw, T in [(550, 1, 60), (550, 2, 60), (1200, 1, 60), (1200, 2, 60), (550, 3, 60), (1200, 3, 60),
                             (550, 1, 30), (1200, 1, 30), (550, 1, 300), (1200, 1, 300)]:
            a, b = f"rule_s{shell}_gw{gw}_ta{T}", f"none_s{shell}_gw{gw}_ta{T}"
            hc = base if (shell, gw) == (550, 1) else f"reactive-ttl_s{shell}_gw{gw}_eps0.2"
            if hc not in held:
                continue
            if full(c2, a) and full(c2, b):
                d_ = f"drop_s{shell}_gw{gw}_ta{T}"
                yield (f"{shell} km, {gw}, {T} s", f"{SH[shell]}G{W[gw]}T{TN[T]}", c2, a, c2, b, held, hc, T, 40, d_ if full(c2, d_) else None)
            elif T == 60:
                yield (f"{shell} km, {gw}, {T} s", f"{SH[shell]}G{W[gw]}T{TN[T]}", main, f"reactive-ttl_s{shell}_gw{gw}", main, f"reactive_s{shell}_gw{gw}", held, hc, T, 10, None)
            else:
                yield (f"{shell} km, {gw}, {T} s", f"{SH[shell]}G{W[gw]}T{TN[T]}", tw if T == 300 else ta, a, ta, b, held, hc, T, 10, None)
        for shell, T in [(550, 900), (550, 3600), (1200, 900), (1200, 3600)]:
            a, b = f"rule_s{shell}_gw1_ta{T}", f"none_s{shell}_gw1_ta{T}"
            hc = base if shell == 550 else f"reactive-ttl_s{shell}_gw1_eps0.2"
            if a in tw and b in ta and hc in held:
                yield (f"{shell} km, 1, {T} s", f"{SH[shell]}GOneT{TN[T]}", tw, a, ta, b, held, hc, T, 10, None)

    rows = []
    for lab, tg, da, a, db, b, hd, hc, T, n, dropc in cases():
        tau = -T * math.log(0.8)
        sums = np.array([seed_sums(hd[hc][s], T, tau, est=True) for s in sorted(hd[hc])])
        seeds = sorted(set(da[a]) & set(db[b]))
        g = np.array([float(db[b][s]["summary"]["sla_viol_frac"]) - float(da[a][s]["summary"]["sla_viol_frac"]) for s in seeds]) * 100
        o = np.array([float(da[a][s]["summary"]["overprov_mean"]) - float(db[b][s]["summary"]["overprov_mean"]) for s in seeds]) * 100
        dp = np.array([float(db[b][s]["summary"]["short_depth_mean"]) - float(da[a][s]["summary"]["short_depth_mean"]) for s in seeds]) * 100
        ps, po = law(sums, r0, pl, ol)
        bs, bo = [], []
        for rr in boot_rates:
            idx = rng.integers(0, len(sums), len(sums))
            x, y = law(sums[idx], rr, pl, ol)
            bs.append(x); bo.append(y)
        bs, bo = np.array(bs), np.array(bo)
        # combined interval on sim minus prediction
        m_g, h_g, _ = ci(g)
        m_o, h_o, _ = ci(o)
        se_g, se_o = h_g / (2.02 if n >= 40 else 2.262), h_o / (2.02 if n >= 40 else 2.262)
        err_s = (m_g - ps, 1.96 * math.hypot(se_g, float(np.std(bs))))
        err_o = (m_o - po, 1.96 * math.hypot(se_o, float(np.std(bo))))
        th = float(np.mean([float(x["dur"]) for s in hd[hc] for x in hd[hc][s]["outages"]]))
        nu = 1 - sum(float(x["dur"]) for s in hd[hc] for x in hd[hc][s]["outages"]) / sums[:, 4].sum()
        cs = closed_value(nu, th, T, 0.2, r0)[0]
        net = g - w1 * o
        expo = 100 * (sums[:, 0].sum() + sums[:, 2].sum()) / sums[:, 4].sum()
        R = dict(lab=lab, tg=tg, n=n, g=(m_g, h_g), o=(m_o, h_o), ps=ps, po=po, ips=(np.percentile(bs, 2.5), np.percentile(bs, 97.5)),
                 ipo=(np.percentile(bo, 2.5), np.percentile(bo, 97.5)), err_s=err_s, err_o=err_o, net=ci(net), pnet=ps - w1 * po,
                 pnet_p=sign_p(net), closed=cs, depth=ci(dp), expo=expo, T=T, nu=nu, th=th, low=int(np.sum(g > 0)),
                 pg=sign_p(g), wstar=m_g / m_o if m_o > 0 else float("nan"), dropc=dropc, da=da, a=a, b=b, db=db)
        rows.append(R)
        for k, v, f in [("NoneShort", ci(col(db, b, "sla_viol_frac") * 100)[0], "{:.1f}"), ("RuleShort", ci(col(da, a, "sla_viol_frac") * 100)[0], "{:.1f}"),
                        ("Sim", m_g, "{:.2f}"), ("SimCi", h_g, "{:.2f}"), ("Pred", ps, "{:.2f}"), ("Over", m_o, "{:.2f}"),
                        ("OverCi", h_o, "{:.2f}"), ("OverPred", po, "{:.2f}"), ("Net", ci(net)[0], "{:+.2f}"), ("NetCi", ci(net)[1], "{:.2f}"),
                        ("Closed", cs, "{:.2f}"), ("Depth", ci(dp)[0], "{:.2f}"), ("DepthCi", ci(dp)[1], "{:.2f}"), ("Expo", expo, "{:.2f}"),
                        ("Wstar", R["wstar"], "{:.1f}")]:
            mac(f"Law{k}{tg}", v, f)
    if not rows:
        return
    # ---- the test table
    with open(os.path.join(TAB, "table_law.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrrrr@{}}\n\\toprule\n")
        t.write("Shell, GW, $T_{\\mathrm{A1}}$ & Seeds & Removed & Predicted & Error & Closed & Added & Predicted & Error & Net\\\\\n")
        t.write(" & & (points) & (interval) & (points) & form & (points) & (interval) & (points) & (points)\\\\\n\\midrule\n")
        for R in rows:
            t.write(f"{R['lab']} & {R['n']} & {sgn(R['g'][0]).replace('+', '')}$\\pm${R['g'][1]:.2f} & {R['ps']:.2f} ({R['ips'][0]:.2f}, {R['ips'][1]:.2f}) & "
                    f"{sgn(R['err_s'][0])}$\\pm${R['err_s'][1]:.2f} & {R['closed']:.2f} & {sgn(R['o'][0]).replace('+', '')}$\\pm${R['o'][1]:.2f} & "
                    f"{R['po']:.2f} ({R['ipo'][0]:.2f}, {R['ipo'][1]:.2f}) & {sgn(R['err_o'][0])}$\\pm${R['err_o'][1]:.2f} & "
                    f"{sgn(R['net'][0])}$\\pm${R['net'][1]:.2f}\\\\\n")
        t.write("\\bottomrule\n\\end{tabular}\n")
    # tests that can fail: 40-seed cases with a real effect
    R40 = [R for R in rows if R["n"] == 40] or [R for R in rows if R["T"] <= 300]
    mac("LawSeeds", max(R["n"] for R in R40), "{:d}")
    mac("LawCases", len(R40), "{:d}")
    fail_s = [R for R in rows if abs(R["err_s"][0]) > R["err_s"][1]]
    fail_o = [R for R in rows if abs(R["err_o"][0]) > R["err_o"][1]]
    mac("LawFailS", len(fail_s), "{:d}")
    mac("LawFailO", len(fail_o), "{:d}")
    mac("LawAll", len(rows), "{:d}")
    mac("LawFailSList", "; ".join(R["lab"] for R in fail_s) or "none", "{}")
    mac("LawFailOList", "; ".join(R["lab"] for R in fail_o) or "none", "{}")
    mae = lambda f: float(np.mean([abs(R["g"][0] - f(R)) for R in R40]))
    # rivals: zero gain; one constant times the exposure, fitted at 550 km, 1 GW, 60 s; (1 - nu) times a constant
    ref = [R for R in R40 if R["tg"] == "FiveGOneTSixty"]
    if ref:
        k_expo = ref[0]["g"][0] / ref[0]["expo"]
        k_nu = ref[0]["g"][0] / (1 - ref[0]["nu"])
        mac("LawMaeLaw", mae(lambda R: R["ps"]), "{:.2f}")
        mac("LawMaeZero", mae(lambda R: 0.0), "{:.2f}")
        mac("LawMaeExpo", mae(lambda R: k_expo * R["expo"]), "{:.2f}")
        mac("LawMaeNu", mae(lambda R: k_nu * (1 - R["nu"])), "{:.2f}")
        mac("LawKExpo", k_expo, "{:.2f}")
        pass_ = lambda f: sum(abs(R["g"][0] - f(R)) <= R["g"][1] for R in R40)
        mac("LawPassLaw", pass_(lambda R: R["ps"]), "{:d}")
        mac("LawPassZero", pass_(lambda R: 0.0), "{:d}")
        mac("LawPassExpo", pass_(lambda R: k_expo * R["expo"]), "{:d}")
        # range of a constant multiplier that keeps every 40-seed case inside its interval
        ks = np.arange(0.2, 3.0, 0.005)
        ok = [k for k in ks if all(abs(R["g"][0] - k * R["ps"]) <= R["g"][1] for R in R40)]
        mac("LawKLo", min(ok) if ok else float("nan"), "{:.2f}")
        mac("LawKHi", max(ok) if ok else float("nan"), "{:.2f}")
        mac("LawKNone", "yes" if not ok else "no", "{}")
    fast = [R for R in R40 if R["T"] <= 60 and R["g"][0] > 0.5]
    if fast:
        ratios = [R["g"][0] / R["ps"] for R in fast]
        mac("LawFastN", len(fast), "{:d}")
        mac("LawFastMin", min(ratios), "{:.2f}")
        mac("LawFastMax", max(ratios), "{:.2f}")
    near = [R for R in R40 if R["T"] == 300]
    for R in near:
        mac(f"LawNear{R['tg']}", R["g"][0] / R["ps"] if R["ps"] > 0 else float("nan"), "{:.1f}")
    big = [R for R in R40 if R["g"][0] > 3 * 0.1 and R["ps"] > 0.05]
    if big:
        ratios = [R["g"][0] / R["ps"] for R in big]
        mac("LawRatioMin", min(ratios), "{:.2f}")
        mac("LawRatioMax", max(ratios), "{:.2f}")
        mac("LawBigN", len(big), "{:d}")
    # net value, Holm over the 40-seed cases
    adj = holm([R["pnet_p"] for R in R40])
    for R, a_ in zip(R40, adj):
        R["net_holm"] = a_
    pos = [R for R in R40 if R["net_holm"] < 0.05 and R["net"][0] > 0]
    neg = [R for R in R40 if R["net_holm"] < 0.05 and R["net"][0] < 0]
    mac("NetPosN", len(pos), "{:d}")
    mac("NetNegN", len(neg), "{:d}")
    mac("NetPosList", "; ".join(R["lab"] for R in pos) or "none", "{}")
    mac("NetNegList", "; ".join(R["lab"] for R in neg) or "none", "{}")
    lay6 = [R for R in R40 if R["T"] == 60]
    sig = [R for R in R40 if holm([x["pg"] for x in R40])[R40.index(R)] < 0.05 and R["g"][0] > 0]
    mac("GainSigN", len(sig), "{:d}")
    mac("GainSigList", "; ".join(R["lab"] for R in sig) or "none", "{}")
    wst = [R["wstar"] for R in R40 if R["pg"] < 0.05 and R["g"][0] > 0.2]
    if wst:
        mac("WstarMin", min(wst), "{:.1f}")
        mac("WstarMax", max(wst), "{:.1f}")
    mac("NetSigCases", "; ".join(R["lab"] for R in pos) or "none", "{}")
    dr = [R["o"][0] / R["depth"][0] for R in R40 if R["depth"][0] > 0.05 and R["o"][0] > 0.05]
    if dr:
        mac("DepthRatioMin", min(dr), "{:.1f}")
        mac("DepthRatioMax", max(dr), "{:.1f}")

    # ---- rule against drop and no lease on cost, and the two-branch rule from the law
    trows, branch = [], []
    for R in rows:
        da, db, a, b = R["da"], R["db"], R["a"], R["b"]
        cost = lambda D, c, s: float(D[c][s]["summary"]["sla_viol_frac"]) * 100 + w1 * float(D[c][s]["summary"]["overprov_mean"]) * 100
        seeds = sorted(set(da[a]) & set(db[b]))
        Jr = np.array([cost(da, a, s) for s in seeds])
        Jn = np.array([cost(db, b, s) for s in seeds])
        dc = R["dropc"] if R["dropc"] and R["dropc"] in c2 else None
        Dd = c2 if dc else None
        if R["n"] == 10:
            dc = R["b"].replace("none", "drop")
            Dd = ta if dc in ta else None
        if Dd is not None and not set(seeds) <= set(Dd[dc]):
            Dd = None
        Jd = np.array([cost(Dd, dc, s) for s in seeds]) if Dd else None
        # the law's two-branch decision, with tau = 0: drop if net positive, otherwise no lease
        hc = None
        trows.append((R, Jr, Jn, Jd))
    with_drop = [(R, Jr, Jn, Jd) for R, Jr, Jn, Jd in trows if Jd is not None]
    pd_, pn_ = [sign_p(Jd - Jr) for _, Jr, _, Jd in with_drop], [sign_p(Jn - Jr) for _, Jr, Jn, _ in with_drop]
    hd_, hn_ = holm(pd_), holm(pn_)
    for tg_, lst in [("drop", hd_), ("none", hn_)]:
        pass
    with open(os.path.join(TAB, "table_policy.tex"), "w") as t:
        t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrr@{}}\n\\toprule\n")
        t.write(" & & \\multicolumn{3}{c}{Shortfall (\\%) / over-provisioning (\\%)} & \\multicolumn{2}{c}{Cost excess over the rule} & Best\\\\\n")
        t.write("\\cmidrule(lr){3-5}\\cmidrule(lr){6-7}\n")
        t.write("Shell, GW, $T_{\\mathrm{A1}}$ & Seeds & No lease & Rule & Drop & No lease & Drop & mean\\\\\n\\midrule\n")
        cnt = dict(rule=0, none=0, drop=0)
        sig_rule_worse, sig_rule_better = 0, 0
        for i, (R, Jr, Jn, Jd) in enumerate(with_drop):
            da, db, a, b = R["da"], R["db"], R["a"], R["b"]
            dc = R["dropc"] if R["n"] == 40 else b.replace("none", "drop")
            Dd = c2 if R["n"] == 40 else ta
            f = lambda D, c: f"{ci(col(D, c, 'sla_viol_frac') * 100)[0]:.2f} / {ci(col(D, c, 'overprov_mean') * 100)[0]:.2f}"
            mn = ci(Jn - Jr); md = ci(Jd - Jr)
            sn = "$^{*}$" if hn_[i] < 0.05 else ""
            sd = "$^{*}$" if hd_[i] < 0.05 else ""
            best = min([("rule", Jr.mean()), ("none", Jn.mean()), ("drop", Jd.mean())], key=lambda x: x[1])[0]
            cnt[best] += 1
            sig_rule_worse += (hn_[i] < 0.05 and mn[0] < 0) + (hd_[i] < 0.05 and md[0] < 0)
            sig_rule_better += (hn_[i] < 0.05 and mn[0] > 0) + (hd_[i] < 0.05 and md[0] > 0)
            t.write(f"{R['lab']} & {R['n']} & {f(db, b)} & {f(da, a)} & {f(Dd, dc)} & {sgn(mn[0])}$\\pm${mn[1]:.2f}{sn} & {sgn(md[0])}$\\pm${md[1]:.2f}{sd} & "
                    f"{dict(rule='rule', none='no lease', drop='drop')[best]}\\\\\n")
            tg = R["tg"]
            mac(f"PolNone{tg}", mn[0], "{:+.2f}"); mac(f"PolNoneCi{tg}", mn[1], "{:.2f}")
            mac(f"PolDrop{tg}", md[0], "{:+.2f}"); mac(f"PolDropCi{tg}", md[1], "{:.2f}")
            mac(f"PolNoneHolm{tg}", fmt_p(hn_[i]), "{}"); mac(f"PolDropHolm{tg}", fmt_p(hd_[i]), "{}")
        t.write("\\bottomrule\n\\end{tabular}\n")
    fd = [ci(Jd - Jr)[0] for R, Jr, Jn, Jd in with_drop if R["T"] <= 60]
    mac("PolDropFastMin", min(fd), "{:.2f}")
    mac("PolDropFastMax", max(fd), "{:.2f}")
    sl = [ci(Jn - Jr)[0] for R, Jr, Jn, Jd in with_drop if R["T"] >= 900]
    mac("PolNoneSlowMax", max(abs(x) for x in sl), "{:.2f}")
    mac("PolBestRule", cnt["rule"], "{:d}")
    mac("PolBestNone", cnt["none"], "{:d}")
    mac("PolBestDrop", cnt["drop"], "{:d}")
    mac("PolN", len(with_drop), "{:d}")
    mac("PolRuleWorse", sig_rule_worse, "{:d}")
    mac("PolRuleBetter", sig_rule_better, "{:d}")
    # two-branch rule: drop where the law's net at tau = 0 is positive, otherwise no lease; cost against the rule
    excess, labs = [], []
    for R, Jr, Jn, Jd in with_drop:
        hc = base if R["tg"].startswith("FiveGOne") else None
        shell, gw = int(R["lab"].split()[0]), int(R["lab"].split(",")[1])
        hc = base if (shell, gw) == (550, 1) else f"reactive-ttl_s{shell}_gw{gw}_eps0.2"
        s0 = np.array([seed_sums(held[hc][s], R["T"], 0.0) for s in sorted(held[hc])])
        ps0, po0 = law(s0, r0, pl, ol)
        take_drop = ps0 - w1 * po0 > 0
        Jb = Jd if take_drop else Jn
        d = Jb - Jr
        excess.append((ci(d)[0], ci(d)[1], sign_p(d), take_drop, R["lab"]))
    adj = holm([e[2] for e in excess])
    mac("BrBetter", sum(e[0] < 0 and a_ < 0.05 for e, a_ in zip(excess, adj)), "{:d}")
    mac("BrWorse", sum(e[0] > 0 and a_ < 0.05 for e, a_ in zip(excess, adj)), "{:d}")
    mac("BrMeanExcess", float(np.mean([e[0] for e in excess])), "{:+.2f}")
    mac("BrMaxExcess", max(e[0] for e in excess), "{:.2f}")
    mac("BrMinExcess", min(e[0] for e in excess), "{:+.2f}")
    mac("BrN", len(excess), "{:d}")
    mac("BrDropN", sum(e[3] for e in excess), "{:d}")

    # ---- the cut-off of a lease: tau* beyond the long outages
    outs = [float(o["dur"]) for s in held[base] for o in held[base][s]["outages"] if float(o["start"]) > 0 and int(o["kind"]) in (1, 2)]
    th99 = float(np.percentile(outs, 99))
    mac("OutNinetyNine", th99, "{:.0f}")
    mac("TaCut", th99 / -math.log(0.8), "{:.0f}")
    mac("OutMean", float(np.mean(outs)), "{:.0f}")
    mac("OutCv", float(np.std(outs) / np.mean(outs)), "{:.2f}")

    # ---- the sites: both forms, scored against the sites' own runs
    srows, ps_b, ps_e = [], [], []
    for nm, gw in [("a", 1), ("a", 2), ("b", 1), ("c", 1), ("atl", 1)]:
        r_, l_, p_, o_ = [f"{x}_site{nm}_gw{gw}" for x in ["reactive", "reactive-ttl", "pal", "onboard-ttl"]]
        if r_ not in st or l_ not in st:
            continue
        hc = f"reactive-ttl_site{nm}_gw{gw}"
        H = sh[hc] if hc in sh else st[l_]
        T = 60; tau = -T * math.log(0.8)
        sums = np.array([seed_sums(H[s], T, tau) for s in sorted(H)])
        pb = law(sums, r0, pl, ol, ext=False)[0]
        pe, poe = law(sums, r0, pl, ol, ext=True)
        g = (A.paired_vec(st, r_, l_, "sla_viol_frac") * 100)
        o = (A.paired_vec(st, l_, r_, "overprov_mean") * 100)
        dp = (A.paired_vec(st, r_, l_, "short_depth_mean") * 100)
        srows.append(dict(nm=nm, gw=gw, av=ci(col(st, r_, "feeder_avail") * 100)[0], none=ci(col(st, r_, "sla_viol_frac") * 100)[0],
                          lease=ci(col(st, l_, "sla_viol_frac") * 100)[0], g=ci(g), o=ci(o), pb=pb, pe=pe, poe=poe,
                          p=sign_p(g), dp=ci(dp), net=ci(g - w1 * o), g3=100 * sums[:, 2].sum() / sums[:, 4].sum()))
    if srows:
        adj = holm([R["p"] for R in srows])
        for R, a_ in zip(srows, adj):
            R["holm"] = a_
        # break-even, first-order with and without the blackout term
        def beam(c, cO, D):
            seeds = sorted(set(D[c]) & set(D[cO]))
            sl = np.mean([(float(D[c][s]["summary"]["beam_mis_T2_d0.05"]) - float(D[cO][s]["summary"]["beam_mis_T2_d0.05"])
                           - float(D[c][s]["summary"]["beam_mis_T2_d0"]) + float(D[cO][s]["summary"]["beam_mis_T2_d0"])) / .05 for s in seeds])
            return -sl

        def age(D, c):
            out = []
            for s in sorted(D[c]):
                t_ = n_ = 0
                for x in D[c][s]["hist"]:
                    if x["cls"] == "nf":
                        t_ += float(x["bin_ms"]) * float(x["count"]); n_ += float(x["count"])
                out.append(t_ / n_)
            return float(np.mean(out))

        def dstar(D, c, cO):
            gam = 1 - ci(col(D, c, "nf_onboard_frac"))[0]
            lb_o = age(D, cO)
            lb_m = age(D, c)
            f = 1 - gam
            lg_ = (lb_m - f * lb_o) / gam
            fo = 100 * (3 * .9 - 1) * (lg_ - lb_o) / 1000 / (2 * 2.0)
            dT = ci(col(D, c, "blackout_mean_s"))[0] - ci(col(D, cO, "blackout_mean_s"))[0]
            sw_ = ci(col(D, c, "switches"))[0]
            sv = ci(col(D, c, "served_s"))[0]
            phi = 1 - ci(col(D, cO, A.bcol(2.0, 0.0)))[0]
            ext = fo + 100 * phi * dT * sw_ / sv / gam
            seeds = sorted(set(D[c]) & set(D[cO]))
            per = []
            for s in seeds:
                dv = [float(D[c][s]["summary"][A.bcol(2.0, gp)]) - float(D[cO][s]["summary"][A.bcol(2.0, gp)]) for gp in A.GAPS]
                per.append(A.crossover_gap(dv) * 100)
            return dict(fo=fo, ext=ext, sim=ci(per), gam=gam, dT=1000 * dT)

        bl = []
        for lab, D, c, cO in [(f"{s} km, {g}", main, f"pal_s{s}_gw{g}", f"onboard-ttl_s{s}_gw{g}") for s, g in [(550, 1), (550, 2), (550, 3), (550, 4), (1200, 1), (1200, 2), (1200, 3), (1200, 4)]] + \
                [(f"{SITE[R['nm']]}, {R['gw']}", st, f"pal_site{R['nm']}_gw{R['gw']}", f"onboard-ttl_site{R['nm']}_gw{R['gw']}") for R in srows]:
            if c in D and cO in D:
                x = dstar(D, c, cO); x["lab"] = lab; bl.append(x)
        fo_err = [abs(x["fo"] - x["sim"][0]) for x in bl]
        ex_err = [abs(x["ext"] - x["sim"][0]) for x in bl]
        mac("BeN", len(bl), "{:d}")
        mac("BeFoMaxErr", max(fo_err), "{:.2f}")
        mac("BeExtMaxErr", max(ex_err), "{:.2f}")
        mac("BeFoIn", sum(e <= x["sim"][1] for e, x in zip(fo_err, bl)), "{:d}")
        mac("BeExtIn", sum(e <= x["sim"][1] for e, x in zip(ex_err, bl)), "{:d}")
        mac("BeSimMin", min(x["sim"][0] for x in bl), "{:.2f}")
        mac("BeSimMax", max(x["sim"][0] for x in bl), "{:.2f}")
        mac("BeRatioMax", max(x["sim"][0] / x["fo"] for x in bl), "{:.1f}")
        byl = {x["lab"]: x for x in bl}
        with open(os.path.join(TAB, "table_sites.tex"), "w") as t:
            t.write("% Generated by the analysis scripts in sim/\n\\begin{tabular}{@{}lrrrrrrrrr@{}}\n\\toprule\n")
            t.write("Cell, GW & Avail. & Lease effect & Law & Law+ & Holm & Depth & Net & $\\delta^\\ast$ & $\\delta^\\ast$ (+)\\\\\n")
            t.write(" & (\\%) & (points) & (points) & (points) & $p$ & (points) & (points) & sim. & pred.\\\\\n\\midrule\n")
            for R in srows:
                x = byl.get(f"{SITE[R['nm']]}, {R['gw']}")
                t.write(f"{SITE[R['nm']]}, {R['gw']} & {R['av']:.1f} & {R['g'][0]:.2f}$\\pm${R['g'][1]:.2f} & {R['pb']:.2f} & {R['pe']:.2f} & {fmt_p(R['holm'])} & "
                        f"{R['dp'][0]:.2f} & {sgn(R['net'][0])}$\\pm${R['net'][1]:.2f} & {x['sim'][0]:.2f} & {x['ext']:.2f}\\\\\n")
            t.write("\\bottomrule\n\\end{tabular}\n")
        lowav = [R for R in srows if R["av"] < 75]
        if lowav:
            rb = [R["g"][0] / R["pb"] for R in lowav]
            re = [R["g"][0] / R["pe"] for R in lowav]
            mac("SiteBaseRatioMin", min(rb), "{:.1f}")
            mac("SiteBaseRatioMax", max(rb), "{:.1f}")
            mac("SiteExtRatioMin", min(re), "{:.2f}")
            mac("SiteExtRatioMax", max(re), "{:.2f}")
            mac("SiteLaunchShareMin", min(R["g3"] for R in lowav), "{:.0f}")
            mac("SiteLaunchShareMax", max(R["g3"] for R in lowav), "{:.0f}")
            mac("SiteLowN", len(lowav), "{:d}")
            mac("SiteDepthMax", max(R["dp"][0] for R in lowav), "{:.2f}")
        for R in srows:
            tg = f"{R['nm'].capitalize()}{W[R['gw']]}"
            mac(f"Site{tg}Sim", R["g"][0], "{:.1f}"); mac(f"Site{tg}Base", R["pb"], "{:.1f}"); mac(f"Site{tg}Ext", R["pe"], "{:.1f}")
            mac(f"Site{tg}Avail", R["av"], "{:.1f}"); mac(f"Site{tg}None", R["none"], "{:.1f}"); mac(f"Site{tg}Lease", R["lease"], "{:.1f}")
        mac("SiteDistinct", len(srows), "{:d}")
        mac("SiteSigDistinct", sum(R["holm"] < 0.05 for R in srows), "{:d}")
        mac("SiteLoAvailB", min(R["av"] for R in srows), "{:.1f}")
        mac("SiteHeldOut", "yes" if sh else "no", "{}")
        c_ = [R for R in srows if R["nm"] == "c" and R["gw"] == 1]
        if c_:
            mac("SiteCoastD", c_[0]["g"][0], "{:.2f}"); mac("SiteCoastCi", c_[0]["g"][1], "{:.2f}")
    # ---- long horizon, hourly updates
    if lg:
        for k in ["none", "rule", "drop"]:
            c = f"{k}_s550_gw1_ta3600_4h"
            if c in lg:
                mac(f"Lg{k.capitalize()}Short", ci(col(lg, c, "sla_viol_frac") * 100)[0], "{:.2f}")
                mac(f"Lg{k.capitalize()}Over", ci(col(lg, c, "overprov_mean") * 100)[0], "{:.2f}")
                mac(f"Lg{k.capitalize()}Tau", ci(col(lg, c, "lease_tau_s"))[0], "{:.0f}")
        a, b = "rule_s550_gw1_ta3600_4h", "none_s550_gw1_ta3600_4h"
        if a in lg and b in lg:
            J = (A.paired_vec(lg, b, a, "sla_viol_frac") * 100 + w1 * A.paired_vec(lg, b, a, "overprov_mean") * 100)
            mac("LgJ", ci(J)[0], "{:+.2f}"); mac("LgJCi", ci(J)[1], "{:.2f}")
    # ---- fallback below the envelope
    if g0:
        for v, tg in [(0.3, "Three"), (0.4, "Four")]:
            c = f"rule_s550_gw1_g{v}"
            if c in g0 and "rule_s550_gw1_ta60" in ta:
                mac(f"GoShort{tg}", ci(col(g0, c, "sla_viol_frac") * 100)[0], "{:.2f}")
                mac(f"GoOver{tg}", ci(col(g0, c, "overprov_mean") * 100)[0], "{:.2f}")
        for k, cc in [("Rule", "rule_s550_gw1_ta60"), ("None", "none_s550_gw1_ta60")]:
            if cc in ta:
                mac(f"GoRef{k}Short", ci(col(ta, cc, "sla_viol_frac") * 100)[0], "{:.2f}")
                mac(f"GoRef{k}Over", ci(col(ta, cc, "overprov_mean") * 100)[0], "{:.2f}")
    # all groups, one binary, for the text
    groups = ["main", "heldout", "recent", "ta1", "costseeds", "eps", "sites", "beamstress", "host", "leadsweep", "ablation",
              "a1proc", "hb", "sens", "variants", "step", "ta1w", "long", "g0", "c2", "siteheld"]
    tot, shas = 0, set()
    for g_ in groups:
        for cfg, by in load(g_).items():
            tot += len(by)
            shas |= {by[s]["summary"].get("binary_sha") for s in by}
    mac("TotRuns", f"{tot:,}".replace(",", "{,}"), "{}")
    mac("BinaryCount", len(shas), "{:d}")
    mac("NewRuns", sum(len(by) for g_ in ["ta1w", "long", "g0", "c2", "siteheld"] for by in load(g_).values()), "{:d}")
    for g_, tg in [("c2", "CTwoRuns"), ("ta1w", "WarmRuns"), ("long", "LongRuns"), ("g0", "GoRuns"), ("siteheld", "SiteHeldRuns")]:
        mac(tg, sum(len(by) for by in load(g_).values()), "{:d}")
    print("r2 binaries", shas)
