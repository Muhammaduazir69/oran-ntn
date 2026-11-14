#!/usr/bin/env python3
"""Round 7 post-hoc long runs (Revision_Plan addendum). Resumable, pid file, nice 19, 3 jobs, stops under 2 GB."""
import hashlib, os, shutil, subprocess, time
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__))
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(HERE, "bin", "lib"))
RUNS = os.path.join(HERE, "runs", "r7")
MIN_FREE = 2 * 2**30
BINS = {"new": os.path.join(HERE, "bin", "p3-dataplane"), "r4": os.path.join(HERE, "bin", "p3-dataplane.r4")}
SHA = {k: hashlib.sha256(open(v, "rb").read()).hexdigest()[:16] for k, v in BINS.items()}
assert SHA["r4"] == "528e659bb8bd4b35" and SHA["new"] == "b7601e74c2728542", SHA
HB = dict(hbPeriodS=0.1, hbMiss=3)
E_ = dict(safeG=0.8, safeH=0.3)
ARMS = {"control": dict(strategy="onboard", oracleIntent=1, **E_), "hold": dict(strategy="onboard", **E_),
        "env02": dict(strategy="onboard-ttl", leaseEps=0.2, **E_),
        "wccontrol": dict(strategy="onboard", oracleIntent=1, workConserving=1, **E_),
        "wcstatic": dict(strategy="onboard", surgeHighG=0.8, surgeLowG=0.8, workConserving=1, **E_),
        "wcfixed": dict(strategy="onboard-ttl", fixedLeaseS=0.001, safeG=0.8, safeH=0.1, workConserving=1)}
SITES = {"base": dict(), "lo": dict(cellLat=10.0, cellLon=68.0)}
LAUNCH = dict(initG=0.8, initH=0.1)
def jobs():
    o = []
    for T in (15, 30):
        for lab, L in (("asbuilt", {}), ("launch", LAUNCH)):
            for a in ("control", "hold"):
                o += [("new", f"long_{lab}_lo_ta{T}", a, s, dict(shell=550, numGw=1, a1MeanS=T, priorUpdateS=T, surge=1, dataPlane=1, **HB, **SITES["lo"], **L, **ARMS[a])) for s in range(101, 121)]
    return o
def one(t):
    b, case, arm, seed, cfg = t
    d = os.path.join(RUNS, case, arm, f"seed{seed:03d}")
    if os.path.exists(os.path.join(d, "summary.csv")) and os.path.getsize(os.path.join(d, "dataplane.csv")) > 0:
        return "skip"
    if shutil.disk_usage(HERE).free < MIN_FREE:
        return "NODISK"
    tmp = d + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    args = ["nice", "-n", "19", BINS[b], "--simSeconds=86400", f"--run={seed}", f"--outputDir={tmp}"] + [f"--{k}={v}" for k, v in cfg.items()]
    r = subprocess.run(args, capture_output=True, text=True, cwd=NS3, env=ENV)
    if r.returncode:
        open(tmp + ".fail", "w").write(r.stderr[-1500:])
        return "FAIL"
    open(os.path.join(tmp, "cmd.txt"), "w").write(" ".join(args[3:]).replace(tmp, d.replace(HERE + "/", "")) + "\nbinary_sha " + SHA[b] + "\n")
    for x in ("stdout.txt", "nf_miss.csv", "switches.csv", "loop_hist.csv"):
        p = os.path.join(tmp, x)
        if os.path.exists(p):
            os.remove(p)
    sp = os.path.join(tmp, "summary.csv")
    head, row = open(sp).read().strip().split("\n")[:2]
    open(sp, "w").write(f"{head},binary_sha\n{row},{SHA[b]}\n")
    os.sync()
    os.rename(tmp, d)
    return "ok"
if __name__ == "__main__":
    open(os.path.join(HERE, "campaign_r7.pid"), "w").write(str(os.getpid()))
    todo = jobs()
    print(len(todo), SHA, flush=True)
    with ThreadPoolExecutor(3) as ex:
        for i, (t, res) in enumerate(zip(todo, ex.map(one, todo))):
            if res in ("FAIL", "NODISK") or i % 40 == 0:
                print(time.strftime("%H:%M:%S"), i, t[1:4], res, flush=True)
    print("done", flush=True)
