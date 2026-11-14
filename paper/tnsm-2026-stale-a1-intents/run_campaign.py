#!/usr/bin/env python3
"""Campaign driver for Paper 3 (TNSM): oran-ntn-feeder-mgmt.

Every run writes summary.csv (with the binary hash in its last column),
outages.csv, switches.csv, loop_hist.csv, nf_miss.csv and checks.csv into
runs/<group>/<config>/seedNN/. A run goes to seedNN.tmp and is renamed when it
has finished, and a finished run is skipped, so the driver can be restarted.

Usage: run_campaign.py <group> [--jobs N]
groups: see GROUP_ORDER; "all" runs them in that order. heldout uses seeds 11-20, costseeds 21-60.
"""
import argparse
import hashlib
import itertools
import math
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
# One binary set for every run of the article: sim/bin/p3-feeder-mgmt (built by
# ns3_patch/build.sh from a patched copy of the example) with a frozen copy of
# the optimized ns-3 libraries in sim/bin/lib (manifest in sim/bin/lib_manifest.sha256).
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"  # working directory only: the satellite module locates its data from it
BIN = os.path.join(HERE, "bin", "p3-feeder-mgmt")
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(HERE, "bin", "lib"))
BIN_SHA = hashlib.sha256(open(BIN, "rb").read()).hexdigest()[:16]
RUNS = os.path.join(HERE, "runs")
SEEDS = range(1, 11)
SIM_S = 3600

# Heartbeat setting used by `reactive` and `pal` in the main grid; chosen from
# the `hb` group (see analyze.py, tuning section). The default fallback is the
# envelope of the version alphabet (g0 = 0.5); the former g0 = 0.6 is the "hedge"
# variant against versions outside the alphabet.
HB_MAIN = {"hbPeriodS": 0.1, "hbMiss": 3, "safeG": 0.5, "safeH": 0.3}
# Lease policies on one host (R+L), T_A1 sweep. The fixed timer is the rule's value
# at eps = 0.2 and T_A1 = 60 s, tuned there and left alone; 0.001 s drops at once.
FIXED_60 = -60.0 * math.log(1 - 0.2)
TA1 = [30, 60, 300, 900, 3600]
# Cell sites of the geometry study: (name, lat, lon, gwList). Arabian Sea sites share the
# gateway list; atl is a mid-Atlantic cell with Azores and Canary gateways.
SITES = [("a", 15.0, 60.0, ""), ("b", 10.0, 68.0, ""), ("c", 24.0, 62.0, ""),
         ("atl", 35.0, -40.0, "37.74:-25.67,28.10:-15.42,38.72:-9.14,14.69:-17.44")]
GROUP_ORDER = ["heldout", "main", "recent", "ta1", "costseeds", "eps", "sites", "beamstress", "host",
               "leadsweep", "ablation", "a1proc", "hb", "sens", "variants", "step"]
# Round 2 of the revision, same binary: warm-started rule, long horizon, fallback below the envelope,
# and 40 fresh seeds so the law can fail. Run with: run_campaign.py r2
GROUP_R2 = ["ta1w", "long", "g0", "siteheld", "c2"]
PID = os.path.join(HERE, "campaign.pid")
MIN_FREE_GB = 3.0


def jobs_for(group):
    out = []
    if group == "hb":
        for strat, hb in itertools.product(["reactive", "pal"], [0.05, 0.1, 0.2, 0.5, 1.0, 2.0]):
            for gw in [1]:
                cfg = dict(strategy=strat, shell=550, numGw=gw, hbPeriodS=hb, hbMiss=3, safeG=0.5, safeH=0.3)
                out.append((f"{strat}_s550_gw{gw}_hb{hb}", cfg))
    elif group == "eps":
        # eps = 0.2 is the main-grid default and is read from the main group
        for strat, eps in itertools.product(["pal", "reactive-ttl", "reactive-ttl-pre", "onboard-ttl"],
                                            [0.05, 0.5, 0.9, 0.99]):
            for gw in [1, 2]:
                cfg = dict(strategy=strat, shell=550, numGw=gw, leaseEps=eps, **HB_MAIN)
                out.append((f"{strat}_s550_gw{gw}_eps{eps}", cfg))
        # finer grid for the realized-cost check (PAL, one gateway)
        for eps in [0.1, 0.3, 0.4, 0.7]:
            cfg = dict(strategy="pal", shell=550, numGw=1, leaseEps=eps, **HB_MAIN)
            out.append((f"pal_s550_gw1_eps{eps}", cfg))
    elif group == "main":
        for strat, shell, gw in itertools.product(
            ["ground", "onboard", "onboard-ttl", "reactive", "reactive-ttl", "reactive-ttl-pre", "pal",
             "sat-pred"],
            [550, 1200], [1, 2, 3, 4]
        ):
            if strat == "sat-pred" and gw > 2:
                continue
            cfg = dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)
            out.append((f"{strat}_s{shell}_gw{gw}", cfg))
    elif group == "recent":
        # baselines that take their placement from recent designs; the intent handling
        # is our assumption (A1 default). pred-ho: predictive E2 migration after Baranda
        # et al. 2025; ground-pre: ground authority with pre-provisioning after KubeSpace
        # 2026. The placement after Space-O-RAN 2026 is the onboard row of the main group.
        for strat, shell, gw in itertools.product(["pred-ho", "ground-pre"], [550, 1200], [1, 2, 3, 4]):
            cfg = dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)
            out.append((f"{strat}_s{shell}_gw{gw}", cfg))
    elif group == "ablation":
        for strat, shell, gw in itertools.product(["pal-nopre", "pal-gw"], [550, 1200], [1, 2]):
            cfg = dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)
            out.append((f"{strat}_s{shell}_gw{gw}", cfg))
    elif group == "sens":
        # Ephemeris error, rain-fade rate and on-board processing.
        for strat, sig in itertools.product(["reactive-ttl", "pal", "sat-pred"], [0.0, 2.0, 5.0]):
            cfg = dict(strategy=strat, shell=550, numGw=1, ephSigmaS=sig, **HB_MAIN)
            out.append((f"{strat}_s550_gw1_eph{sig}", cfg))
        # 0.23 fades/h = ITU-R P.618-13 with P.837-7 at Karachi, 30 GHz, 40 deg,
        # 10 dB margin (0.26 % of time) at the 40 s mean fade length (D28)
        for strat, rain in itertools.product(["reactive-ttl", "reactive-ttl-pre", "pal", "onboard-ttl"],
                                             [0.0, 0.23, 6.0]):
            cfg = dict(strategy=strat, shell=550, numGw=1, rainPerHour=rain, **HB_MAIN)
            out.append((f"{strat}_s550_gw1_rain{rain}", cfg))
        for strat in ["onboard", "onboard-ttl", "reactive-ttl", "pal"]:
            cfg = dict(strategy=strat, shell=550, numGw=1, procScaleOnboard=5.0, **HB_MAIN)
            out.append((f"{strat}_s550_gw1_proc5", cfg))
    elif group == "variants":
        for gw in [1, 2]:
            for strat in ["reactive-ttl", "reactive-ttl-pre", "pal", "onboard-ttl", "sat-pred"]:
                out.append((f"{strat}_s550_gw{gw}_soft",
                            dict(strategy=strat, shell=550, numGw=gw, softSwitch=1, **HB_MAIN)))
            for strat in ["reactive-ttl", "reactive-ttl-pre", "pal", "sat-pred"]:
                out.append((f"{strat}_s550_gw{gw}_link",
                            dict(strategy=strat, shell=550, numGw=gw, linkDetectS=0.05, **HB_MAIN)))
            for strat in ["reactive-ttl-pre", "pal"]:
                out.append((f"{strat}_s550_gw{gw}_softlink",
                            dict(strategy=strat, shell=550, numGw=gw, softSwitch=1, linkDetectS=0.05, **HB_MAIN)))
                out.append((f"{strat}_s550_gw{gw}_arq",
                            dict(strategy=strat, shell=550, numGw=gw, arqRtoS=0.05, **HB_MAIN)))
        # hedge fallback: g0 = 0.6 holds capacity above every version of the alphabet
        for strat in ["reactive-ttl", "pal", "onboard-ttl"]:
            out.append((f"{strat}_s550_gw1_hedge", dict(strategy=strat, shell=550, numGw=1, **dict(HB_MAIN, safeG=0.6))))
        for eps in [0.05, 0.1, 0.3, 0.5, 0.9]:
            out.append((f"pal_s550_gw1_hedge_eps{eps}",
                        dict(strategy="pal", shell=550, numGw=1, leaseEps=eps, **dict(HB_MAIN, safeG=0.6))))
    elif group == "a1proc":
        # non-Poisson intent updates (D8): the lease rule still assumes Poisson
        for proc, eps in itertools.product(["periodic", "bursty"], [0.05, 0.1, 0.2, 0.3, 0.5, 0.9]):
            out.append((f"pal_s550_gw1_{proc}_eps{eps}",
                        dict(strategy="pal", shell=550, numGw=1, a1Process=proc, leaseEps=eps, **HB_MAIN)))
    elif group == "heldout":
        # rates for the lease cost check, on seeds 11-20, disjoint from the
        # evaluation seeds (one configuration per update process and fallback)
        base = dict(strategy="reactive-ttl", shell=550, numGw=1, leaseEps=0.2, **HB_MAIN)
        out.append(("reactive-ttl_s550_gw1_eps0.2", dict(base)))
        out.append(("reactive-ttl_s550_gw1_hedge_eps0.2", dict(base, safeG=0.6)))
        out.append(("reactive-ttl_s550_gw1_periodic_eps0.2", dict(base, a1Process="periodic")))
        out.append(("reactive-ttl_s550_gw1_bursty_eps0.2", dict(base, a1Process="bursty")))
        # outage laws of every layout, for the out-of-sample value-of-lease prediction
        for shell, gw in itertools.product([550, 1200], [1, 2, 3, 4]):
            if (shell, gw) != (550, 1):
                out.append((f"reactive-ttl_s{shell}_gw{gw}_eps0.2", dict(base, shell=shell, numGw=gw)))
        # age processes for the out-of-sample beam prediction
        for strat, shell, gw in itertools.product(["ground", "onboard-ttl", "pal"], [550, 1200], [1, 2]):
            if (shell, gw) != (1200, 2):
                out.append((f"{strat}_s{shell}_gw{gw}", dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)))
    elif group == "ta1":
        # no lease, the rule, a fixed timer tuned at 60 s, immediate drop; the prior stays 60 s
        for ta, (shell, gw) in itertools.product(TA1, [(550, 1), (1200, 1)]):
            for name, extra in [("none", dict(strategy="reactive")),
                                ("rule", dict(strategy="reactive-ttl")),
                                ("fixed", dict(strategy="reactive-ttl", fixedLeaseS=round(FIXED_60, 3))),
                                ("drop", dict(strategy="reactive-ttl", fixedLeaseS=0.001))]:
                out.append((f"{name}_s{shell}_gw{gw}_ta{ta}", dict(shell=shell, numGw=gw, a1MeanS=ta, **extra, **HB_MAIN)))
        # a wrong prior at the default rate
        out.append(("rule_s550_gw1_ta60_prior600",
                    dict(strategy="reactive-ttl", shell=550, numGw=1, a1MeanS=60, priorUpdateS=600, **HB_MAIN)))
    elif group == "ta1w":
        # the rule with a prior equal to the true spacing, so the lease is right from t = 0
        for ta, shell in itertools.product([300, 900, 3600], [550, 1200]):
            out.append((f"rule_s{shell}_gw1_ta{ta}", dict(strategy="reactive-ttl", shell=shell, numGw=1, a1MeanS=ta,
                                                          priorUpdateS=ta, **HB_MAIN)))
    elif group == "long":
        # hourly updates over four hours instead of one
        for name, extra in [("none", dict(strategy="reactive")),
                            ("rule", dict(strategy="reactive-ttl", priorUpdateS=3600)),
                            ("drop", dict(strategy="reactive-ttl", fixedLeaseS=0.001))]:
            out.append((f"{name}_s550_gw1_ta3600_4h", dict(shell=550, numGw=1, a1MeanS=3600, simSeconds=14400, **extra, **HB_MAIN)))
    elif group == "g0":
        # fallback below the envelope: the price of the lease depends on it
        for g0, shell in itertools.product([0.3, 0.4], [550, 1200]):
            out.append((f"rule_s{shell}_gw1_g{g0}", dict(strategy="reactive-ttl", shell=shell, numGw=1, **dict(HB_MAIN, safeG=g0))))
    elif group == "c2":
        # 40 fresh seeds (21-60): no lease, the rule and an immediate drop on one host
        for shell, gw, ta in [(550, 1, 60), (550, 2, 60), (1200, 1, 60), (1200, 2, 60),
                              (550, 1, 30), (1200, 1, 30), (550, 1, 300), (1200, 1, 300), (550, 3, 60), (1200, 3, 60)]:
            pols = [("none", dict(strategy="reactive")), ("rule", dict(strategy="reactive-ttl", priorUpdateS=ta))]
            if gw < 3:
                pols.append(("drop", dict(strategy="reactive-ttl", fixedLeaseS=0.001)))
            for name, extra in pols:
                out.append((f"{name}_s{shell}_gw{gw}_ta{ta}", dict(shell=shell, numGw=gw, a1MeanS=ta, **extra, **HB_MAIN)))
    elif group == "siteheld":
        # held-out seeds at the sites, so the law is evaluated there without their scored runs
        for nm, gw in [("a", 1), ("a", 2), ("b", 1), ("c", 1), ("atl", 1)]:
            st_ = [x for x in SITES if x[0] == nm][0]
            cfg = dict(strategy="reactive-ttl", shell=550, numGw=gw, cellLat=st_[1], cellLon=st_[2], **HB_MAIN)
            if st_[3]:
                cfg["gwList"] = st_[3]
            out.append((f"reactive-ttl_site{nm}_gw{gw}", cfg))
    elif group == "costseeds":
        # resolve lease against no lease on the realized cost (seeds 21-60)
        out.append(("predho", dict(strategy="pred-ho", shell=550, numGw=1, **HB_MAIN)))
        for eps in [0.2, 0.5]:
            out.append((f"pal_eps{eps}", dict(strategy="pal", shell=550, numGw=1, leaseEps=eps, **HB_MAIN)))
    elif group == "sites":
        for (nm, lat, lon, gl), strat, gw in itertools.product(SITES, ["reactive", "reactive-ttl", "pal", "onboard-ttl"], [1, 2]):
            cfg = dict(strategy=strat, shell=550, numGw=gw, cellLat=lat, cellLon=lon, **HB_MAIN)
            if gl:
                cfg["gwList"] = gl
            out.append((f"{strat}_site{nm}_gw{gw}", cfg))
    elif group == "beamstress":
        # beam processes Lemma 1 does not assume; the break-even is read from each pair
        for nm, extra in [("erl", dict(beamCv2=0.25)), ("hyp", dict(beamCv2=4.0)), ("drift", dict(beamDrift=0.9)),
                          ("corr", dict(errRho=0.5))]:
            for strat in ["pal", "onboard-ttl"]:
                out.append((f"{strat}_s550_gw1_{nm}", dict(strategy=strat, shell=550, numGw=1, **extra, **HB_MAIN)))
    elif group == "host":
        # PAL with the host test: below the break-even the NF class stays on board
        for shell, gw in [(550, 1), (550, 2)]:
            out.append((f"pal-pin_s{shell}_gw{gw}", dict(strategy="pal", shell=shell, numGw=gw, nfPinOnboard=1, **HB_MAIN)))
    elif group == "leadsweep":
        for lead in [0.5, 1.0, 5.0, 10.0]:
            out.append((f"pal_s550_gw1_lead{lead}", dict(strategy="pal", shell=550, numGw=1, leadS=lead, **HB_MAIN)))
        for pre in [5.0, 20.0, 120.0]:
            out.append((f"pal_s550_gw1_pre{pre}", dict(strategy="pal", shell=550, numGw=1, preS=pre, **HB_MAIN)))
        for k in [0.0, 0.5]:
            out.append((f"pal_s1200_gw1_kappa{k}", dict(strategy="pal", shell=1200, numGw=1, slack=k, **HB_MAIN)))
    elif group == "step":
        # intents that move one step in the alphabet instead of a uniform redraw
        for name, st_ in [("none", "reactive"), ("rule", "reactive-ttl")]:
            for ta in [60, 300]:
                out.append((f"{name}_s550_gw1_ta{ta}", dict(strategy=st_, shell=550, numGw=1, a1MeanS=ta, a1Step=1, **HB_MAIN)))
    else:
        sys.exit(f"unknown group {group}")
    return out


def seeds_of(group):
    return {"heldout": range(11, 21), "siteheld": range(11, 21), "costseeds": range(21, 61), "c2": range(21, 61)}.get(group, SEEDS)


def run_one(group, name, cfg, seed):
    d = os.path.join(RUNS, group, name, f"seed{seed:02d}")
    if os.path.exists(os.path.join(d, "summary.csv")):
        return "skip"
    tmp = d + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    if shutil.disk_usage(HERE).free < MIN_FREE_GB * 2**30:
        shutil.rmtree(tmp, ignore_errors=True)
        return "NODISK"
    cfg = dict(cfg)
    sim_s = cfg.pop("simSeconds", SIM_S)
    args = ["nice", "-n", "10", BIN, f"--simSeconds={sim_s}", f"--run={seed}", f"--outputDir={tmp}"]
    args += [f"--{k}={v}" for k, v in cfg.items()]
    r = subprocess.run(args, capture_output=True, text=True, cwd=NS3, env=ENV)
    if r.returncode != 0:
        with open(tmp + ".fail", "w") as f:
            f.write(r.stderr[-2000:])
        shutil.rmtree(tmp, ignore_errors=True)
        return f"FAIL {r.returncode}"
    with open(os.path.join(tmp, "cmd.txt"), "w") as f:
        f.write(" ".join(a.replace(HERE + "/", "").replace(tmp, d.replace(HERE + "/", "")) for a in args[3:]) + "\n")
    # the binary hash goes in each result row
    sp = os.path.join(tmp, "summary.csv")
    head, row = open(sp).read().strip().split("\n")[:2]
    with open(sp, "w") as f:
        f.write(f"{head},binary_sha\n{row},{BIN_SHA}\n")
    if group == "c2":
        # only the summary and the state counts are read for these seeds: keep raw output small
        for x in ["outages.csv", "switches.csv", "nf_miss.csv", "loop_hist.csv", "stdout.txt"]:
            p = os.path.join(tmp, x)
            if os.path.exists(p):
                os.remove(p)
    os.rename(tmp, d)
    return "ok"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("group")
    ap.add_argument("--jobs", type=int, default=3)
    a = ap.parse_args()
    assert a.jobs <= 3
    with open(PID, "w") as f:
        f.write(str(os.getpid()))
    groups = GROUP_ORDER if a.group == "all" else GROUP_R2 if a.group == "r2" else a.group.split(",")
    todo = [(gr, n, c, s) for gr in groups for n, c in jobs_for(gr) for s in seeds_of(gr)]
    print(f"{a.group}: {len(todo)} runs, {a.jobs} parallel, binary {BIN_SHA}", flush=True)
    done = 0
    with ThreadPoolExecutor(a.jobs) as ex:
        for i, res in enumerate(ex.map(lambda t: run_one(*t), todo)):
            done += 1
            if res.startswith("FAIL") or res == "NODISK" or i % 20 == 0:
                print(time.strftime("%H:%M:%S"), i, todo[i][0], todo[i][1], todo[i][3], res, flush=True)
    print("done", flush=True)


if __name__ == "__main__":
    main()
