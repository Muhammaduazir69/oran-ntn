#!/usr/bin/env python3
"""Round 4 runs (plan in Revision_Plan/pilot_r3_preregistration.md). Resumable, pid file, nice 19, 3 jobs, stops under 2 GB."""
import hashlib, os, shutil, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "bin", "p3-dataplane")
SHA = hashlib.sha256(open(BIN, "rb").read()).hexdigest()[:16]
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(HERE, "bin", "lib"))
RUNS = os.path.join(HERE, "runs", "r4")
MIN_FREE = 2 * 2**30
HB = dict(hbPeriodS=0.1, hbMiss=3)
E_ = dict(safeG=0.8, safeH=0.3)
L_ = dict(safeG=0.2, safeH=0.3)
ROWS9 = [(550, 1, 15), (550, 1, 30), (550, 1, 60), (550, 1, 300), (550, 1, 900), (1200, 1, 30), (1200, 1, 300), (550, 2, 30), (550, 3, 30)]
ROWS5 = [(550, 1, 15), (550, 1, 30), (550, 1, 60), (1200, 1, 30), (550, 2, 30)]
CTL = dict(strategy="onboard", oracleIntent=1, **E_)
HOLD = dict(strategy="onboard", **E_)
ENV02 = dict(strategy="onboard-ttl", leaseEps=0.2, **E_)
STATIC = dict(strategy="onboard", surgeHighG=0.8, surgeLowG=0.8, **E_)
S2 = range(21, 41)
def base(shell, gw, T):
    return dict(shell=shell, numGw=gw, a1MeanS=T, priorUpdateS=T, surge=1, dataPlane=1, **HB)
def jobs():
    o = []
    # M5
    for shell, gw, T in ROWS5:
        rid = f"s{shell}_gw{gw}_ta{T}"
        for a, c in [("control", CTL), ("hold", HOLD), ("env02", ENV02), ("static", STATIC)]:
            if T == 30 and shell == 550 and gw == 1 and a != "static":
                continue
            o += [("wc", rid, a, s, dict(base(shell, gw, T), workConserving=1, **c)) for s in S2]
        o += [("hp", rid, "static", s, dict(base(shell, gw, T), **STATIC)) for s in S2]
    # M1, M2
    for shell, gw, T in ROWS9:
        rid = f"s{shell}_gw{gw}_ta{T}"
        for a, c in [("dropenv", dict(strategy="onboard-ttl", fixedLeaseS=0.35, **E_)), ("droplean", dict(strategy="onboard-ttl", fixedLeaseS=0.35, **L_)), ("lean05", dict(strategy="onboard-ttl", leaseEps=0.5, **L_))]:
            o += [("fact", rid, a, s, dict(base(shell, gw, T), **c)) for s in S2]
    # M6: lead by epoch
    for lead in (0.0, 1.0, 2.0):
        for P in (0.2, 1.0):
            if lead == 2.0 and P == 1.0:
                continue
            for a, c in [("control", CTL), ("hold", HOLD), ("env02", ENV02)]:
                o += [("lead", f"lead{lead}_P{P}", a, s, dict(base(550, 1, 30), surgeLeadS=lead, nsPeriodS=P, **c)) for s in S2]
    # M6: dwell apart from spacing
    for D in (30, 120):
        for T in (30, 120):
            for a, c in [("control", CTL), ("hold", HOLD), ("env02", ENV02)]:
                o += [("dwell", f"D{D}_T{T}", a, s, dict(base(550, 1, T), surgeDwellS=D, **c)) for s in S2]
    # M6: fresh seeds for the out-of-sample test
    for shell, gw, T in [(550, 1, 15), (550, 1, 60), (1200, 1, 60), (550, 2, 60)]:
        for a, c in [("control", CTL), ("hold", HOLD)]:
            o += [("fresh", f"s{shell}_gw{gw}_ta{T}", a, s, dict(base(shell, gw, T), **c)) for s in range(41, 61)]
    # N6
    o += [("n6", "h01", "lean02", s, dict(base(550, 1, 30), strategy="onboard-ttl", leaseEps=0.2, safeG=0.2, safeH=0.1)) for s in S2]
    for a, c in [("control", CTL), ("hold", HOLD), ("env02", ENV02), ("lean02", dict(strategy="onboard-ttl", leaseEps=0.2, **L_))]:
        o += [("n6", "duty25", a, s, dict(base(550, 1, 30), surgeDuty=0.25, **c)) for s in S2]
    # N2
    for a, c in [("control", CTL), ("hold", HOLD), ("env02", ENV02)]:
        o += [("unl", "s550_gw1_ta30", a, s, dict(base(550, 1, 30), surgeUnlinked=1, **c)) for s in S2]
    return o
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
    for x in ("stdout.txt", "nf_miss.csv", "switches.csv", "loop_hist.csv"):
        p = os.path.join(tmp, x)
        if os.path.exists(p):
            os.remove(p)
    sp = os.path.join(tmp, "summary.csv")
    head, row = open(sp).read().strip().split("\n")[:2]
    open(sp, "w").write(f"{head},binary_sha\n{row},{SHA}\n")
    os.rename(tmp, d)
    return "ok"
if __name__ == "__main__":
    open(os.path.join(HERE, "campaign_r4.pid"), "w").write(str(os.getpid()))
    todo = jobs()
    print(len(todo), SHA, flush=True)
    with ThreadPoolExecutor(3) as ex:
        for i, (t, res) in enumerate(zip(todo, ex.map(one, todo))):
            if res in ("FAIL", "NODISK") or i % 50 == 0:
                print(time.strftime("%H:%M:%S"), i, t[:4], res, flush=True)
    print("done", flush=True)
