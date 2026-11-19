#!/usr/bin/env python3
"""Round 3 campaign on the data plane (plan fixed in Revision_Plan/pilot_r3_preregistration.md). Resumable; pid file;
stops under 2 GB free. nice 19, 4 jobs at most. Writes runs/<group>/<case>/<arm>/seedNN/."""
import hashlib, itertools, os, shutil, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "bin", "p3-dataplane")
SHA = hashlib.sha256(open(BIN, "rb").read()).hexdigest()[:16]
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(HERE, "bin", "lib"))
RUNS = os.path.join(HERE, "runs")
MIN_FREE = 2 * 2**30
HB = dict(hbPeriodS=0.1, hbMiss=3)
ENVH = dict(safeG=0.8, safeH=0.3)
LEAN = dict(safeG=0.2, safeH=0.3)


def arms(T):
    return {"control": dict(strategy="onboard", oracleIntent=1, **ENVH), "hold": dict(strategy="onboard", **ENVH),
            "env02": dict(strategy="onboard-ttl", leaseEps=0.2, **ENVH), "env05": dict(strategy="onboard-ttl", leaseEps=0.5, **ENVH),
            "lean02": dict(strategy="onboard-ttl", leaseEps=0.2, **LEAN), "droplean": dict(strategy="onboard-ttl", fixedLeaseS=0.001, **LEAN)}


def jobs():
    out = []
    S2 = range(21, 41)
    for shell, gw, T in [(550, 1, 15), (550, 1, 30), (550, 1, 60), (550, 1, 300), (550, 1, 900), (1200, 1, 30), (1200, 1, 300), (550, 2, 30), (550, 3, 30)]:
        for a, c in arms(T).items():
            for s in S2:
                out.append(("dp", f"s{shell}_gw{gw}_ta{T}", a, s, dict(shell=shell, numGw=gw, a1MeanS=T, priorUpdateS=T, surge=1, dataPlane=1, **HB, **c)))
    A = arms(30)
    for sess in (4, 14):
        for a in ("control", "hold", "env02", "lean02"):
            for s in S2:
                out.append(("dpload", f"sess{sess}", a, s, dict(shell=550, numGw=1, a1MeanS=30, priorUpdateS=30, surge=1, dataPlane=1, surgeSessions=sess, **HB, **A[a])))
    for a in ("control", "hold", "env02", "lean02"):
        for s in S2:
            out.append(("dpwc", "s550_gw1_ta30", a, s, dict(shell=550, numGw=1, a1MeanS=30, priorUpdateS=30, surge=1, dataPlane=1, workConserving=1, **HB, **A[a])))
    for a, c in [("control", dict(strategy="onboard", oracleIntent=1)), ("hold", dict(strategy="onboard")), ("env02", dict(strategy="onboard-ttl", leaseEps=0.2))]:
        for s in S2:
            out.append(("dpuni", "s550_gw1_ta60", a, s, dict(shell=550, numGw=1, a1MeanS=60, priorUpdateS=60, dataPlane=1, safeG=0.5, safeH=0.3, **HB, **c)))
    # held-out: rates and outage samples for the law, seeds 61 to 100 at the base case, 61 to 80 elsewhere
    for s in range(61, 101):
        out.append(("dpheld", "s550_gw1_ta30", "env02", s, dict(shell=550, numGw=1, a1MeanS=30, priorUpdateS=30, surge=1, dataPlane=1, leaseEps=0.2, strategy="onboard-ttl", **HB, **ENVH)))
    for shell, gw in [(550, 2), (550, 3), (1200, 1)]:
        for s in range(61, 81):
            out.append(("dpheld", f"s{shell}_gw{gw}_ta30", "env02", s, dict(shell=shell, numGw=gw, a1MeanS=30, priorUpdateS=30, surge=1, dataPlane=1, leaseEps=0.2, strategy="onboard-ttl", **HB, **ENVH)))
    # held-out inputs for the break-even (data plane off: reproduces the earlier binary)
    SITES = {"a": (15.0, 60.0, ""), "b": (10.0, 68.0, ""), "c": (24.0, 62.0, ""), "atl": (35.0, -40.0, "37.74:-25.67,28.10:-15.42,38.72:-9.14,14.69:-17.44")}
    for strat in ("pal", "onboard-ttl"):
        for shell, gw in [(550, 3), (550, 4), (1200, 2), (1200, 3), (1200, 4)]:
            for s in range(11, 21):
                out.append(("beheld", f"{strat}_s{shell}_gw{gw}", "x", s, dict(strategy=strat, shell=shell, numGw=gw, safeG=0.5, safeH=0.3, **HB)))
        for nm, gw in [("a", 1), ("a", 2), ("b", 1), ("c", 1), ("atl", 1)]:
            lat, lon, gl = SITES[nm]
            c = dict(strategy=strat, shell=550, numGw=gw, cellLat=lat, cellLon=lon, safeG=0.5, safeH=0.3, **HB)
            if gl:
                c["gwList"] = gl
            for s in range(11, 21):
                out.append(("beheld", f"{strat}_site{nm}_gw{gw}", "x", s, c))
    return out


def one(t):
    gr, case, arm, seed, cfg = t
    d = os.path.join(RUNS, gr, case, arm, f"seed{seed:02d}")
    if os.path.exists(os.path.join(d, "summary.csv")):
        return "skip"
    if shutil.disk_usage(HERE).free < MIN_FREE:
        return "NODISK"
    tmp = d + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    args = ["nice", "-n", "19", BIN, "--simSeconds=3600", f"--run={seed}", f"--outputDir={tmp}"] + [f"--{k}={v}" for k, v in cfg.items()]
    r = subprocess.run(args, capture_output=True, text=True, cwd=NS3, env=ENV)
    if r.returncode:
        open(tmp + ".fail", "w").write(r.stderr[-1500:])
        return "FAIL"
    open(os.path.join(tmp, "cmd.txt"), "w").write(" ".join(args[3:]).replace(tmp, d.replace(HERE + "/", "")) + "\nbinary_sha " + SHA + "\n")
    for x in ("stdout.txt", "nf_miss.csv", "switches.csv"):
        if gr != "beheld" and os.path.exists(os.path.join(tmp, x)):
            os.remove(os.path.join(tmp, x))
    sp = os.path.join(tmp, "summary.csv")
    head, row = open(sp).read().strip().split("\n")[:2]
    open(sp, "w").write(f"{head},binary_sha\n{row},{SHA}\n")
    os.rename(tmp, d)
    return "ok"


if __name__ == "__main__":
    open(os.path.join(HERE, "campaign_dp.pid"), "w").write(str(os.getpid()))
    todo = jobs()
    print(len(todo), SHA, flush=True)
    with ThreadPoolExecutor(4) as ex:
        for i, (t, res) in enumerate(zip(todo, ex.map(one, todo))):
            if res in ("FAIL", "NODISK") or i % 50 == 0:
                print(time.strftime("%H:%M:%S"), i, t[:4], res, flush=True)
    print("done", flush=True)
