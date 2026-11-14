#!/usr/bin/env python3
"""Campaign driver for Paper 3 (TNSM): oran-ntn-feeder-mgmt.

Every run writes summary.csv, outages.csv, switches.csv, loop_hist.csv and
checks.csv into runs/<group>/<config>/seedNN/. A run is skipped when its
summary.csv already exists, so the driver can be restarted.

Usage: run_campaign.py <group> [--jobs N]
groups: hb, eps, main, recent, ablation, sens, variants, a1proc, heldout (seeds 11-20)
"""
import argparse
import itertools
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"
# Private snapshot of the binary and the ns-3 libraries, so that rebuilds of
# the shared tree by other work cannot change the libraries under a running
# campaign. p3bin3 = ns-3 4355d1a22 (shortfall depth, sat-pred strategy; the
# main group and all sat-pred runs come from it). Earlier groups: p3bin2
# (514b4c47a, onboard-ttl from 196653bcb); 4355d1a22 reproduces their
# summary.csv columns byte for byte (PROVENANCE.txt).
# p3bin4 = ns-3 5fa3c2f00 (adds the recent-design strategies pred-ho and
# ground-pre; reproduces the stored main and ablation runs column for column,
# PROVENANCE.txt). p3bin3 no longer exists; only the group "recent" was run
# from p3bin4.
SNAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bin")
BIN = os.path.join(SNAP, "ns3.43-oran-ntn-feeder-mgmt-optimized")
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(SNAP, "lib"))
RUNS = os.path.join(HERE, "runs")
SEEDS = range(1, 11)
SIM_S = 3600

# Heartbeat setting used by `reactive` and `pal` in the main grid; chosen from
# the `hb` group (see analyze.py, tuning section).
HB_MAIN = {"hbPeriodS": 0.1, "hbMiss": 3}


def jobs_for(group):
    out = []
    if group == "hb":
        for strat, hb in itertools.product(["reactive", "pal"], [0.05, 0.1, 0.2, 0.5, 1.0, 2.0]):
            for gw in [1]:
                cfg = dict(strategy=strat, shell=550, numGw=gw, hbPeriodS=hb, hbMiss=3)
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
            cfg = dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)
            out.append((f"{strat}_s{shell}_gw{gw}", cfg))
    elif group == "recent":
        # re-implemented recent designs (Oct 2026): Baranda et al. 2025
        # (predictive E2 migration, intent without expiry) and KubeSpace 2026
        # (ground authority, predictive pre-provisioning); Space-O-RAN 2026 and
        # Chiti et al. 2026 are the onboard and reactive rows of the main group
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
        # envelope fallback: the safe intent is the envelope of the version alphabet (D6)
        for strat in ["reactive-ttl", "reactive-ttl-pre", "pal", "onboard-ttl"]:
            out.append((f"{strat}_s550_gw1_env",
                        dict(strategy=strat, shell=550, numGw=1, safeG=0.5, safeH=0.3, **HB_MAIN)))
        # 0.078 = the envelope rule's prediction at the default weights (review R3 #3)
        for eps in [0.05, 0.078, 0.1, 0.3, 0.5, 0.9]:
            out.append((f"pal_s550_gw1_env_eps{eps}",
                        dict(strategy="pal", shell=550, numGw=1, safeG=0.5, safeH=0.3, leaseEps=eps, **HB_MAIN)))
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
        out.append(("reactive-ttl_s550_gw1_env_eps0.2", dict(base, safeG=0.5, safeH=0.3)))
        out.append(("reactive-ttl_s550_gw1_periodic_eps0.2", dict(base, a1Process="periodic")))
        out.append(("reactive-ttl_s550_gw1_bursty_eps0.2", dict(base, a1Process="bursty")))
    else:
        sys.exit(f"unknown group {group}")
    return out


def run_one(group, name, cfg, seed):
    d = os.path.join(RUNS, group, name, f"seed{seed:02d}")
    if os.path.exists(os.path.join(d, "summary.csv")):
        return "skip"
    os.makedirs(d, exist_ok=True)
    args = [BIN, f"--simSeconds={SIM_S}", f"--run={seed}", f"--outputDir={d}"]
    args += [f"--{k}={v}" for k, v in cfg.items()]
    with open(os.path.join(d, "cmd.txt"), "w") as f:
        f.write(" ".join(args) + "\n")
    r = subprocess.run(args, capture_output=True, text=True, cwd=NS3, env=ENV)
    with open(os.path.join(d, "stdout.txt"), "w") as f:
        f.write(r.stdout[-4000:])
        f.write(r.stderr[-4000:])
    return "ok" if r.returncode == 0 else f"FAIL {r.returncode}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("group")
    ap.add_argument("--jobs", type=int, default=2)
    a = ap.parse_args()
    seeds = range(11, 21) if a.group == "heldout" else SEEDS
    todo = [(a.group, n, c, s) for n, c in jobs_for(a.group) for s in seeds]
    print(f"{a.group}: {len(todo)} runs, {a.jobs} parallel", flush=True)
    with ThreadPoolExecutor(a.jobs) as ex:
        for i, res in enumerate(ex.map(lambda t: run_one(*t), todo)):
            if res.startswith("FAIL") or i % 20 == 0:
                print(i, todo[i][1], todo[i][3], res, flush=True)
    print("done", flush=True)


if __name__ == "__main__":
    main()
