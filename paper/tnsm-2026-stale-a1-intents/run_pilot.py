#!/usr/bin/env python3
"""Pilot of round 3 (Revision_Plan/pilot_r3_preregistration.md): 30 paired runs, resumable."""
import hashlib, os, shutil, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "bin", "p3-dataplane")
SHA = hashlib.sha256(open(BIN, "rb").read()).hexdigest()[:16]
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"
ENV = dict(os.environ, LD_LIBRARY_PATH=os.path.join(HERE, "bin", "lib"))
RUNS = os.path.join(HERE, "runs", "pilot_r3_rep" if "rep" in sys.argv else "pilot_r3")
BASE = dict(shell=550, numGw=1, hbPeriodS=0.1, hbMiss=3, safeG=0.8, safeH=0.3, a1MeanS=30, priorUpdateS=30, surge=1, dataPlane=1)
ARMS = {"control": dict(strategy="onboard", oracleIntent=1), "hold": dict(strategy="onboard"), "lease": dict(strategy="onboard-ttl")}
MIN_FREE = 2 * 2**30
SEEDS = range(21, 61) if "rep" in sys.argv else range(1, 11)
def one(t):
    arm, seed = t
    d = os.path.join(RUNS, arm, f"seed{seed:02d}")
    if os.path.exists(os.path.join(d, "dataplane.csv")):
        return "skip"
    if shutil.disk_usage(HERE).free < MIN_FREE:
        return "NODISK"
    tmp = d + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    args = ["nice", "-n", "19", BIN, "--simSeconds=3600", f"--run={seed}", f"--outputDir={tmp}"] + [f"--{k}={v}" for k, v in {**BASE, **ARMS[arm]}.items()]
    r = subprocess.run(args, capture_output=True, text=True, cwd=NS3, env=ENV)
    if r.returncode:
        open(tmp + ".fail", "w").write(r.stderr[-1500:])
        return "FAIL"
    open(os.path.join(tmp, "cmd.txt"), "w").write(" ".join(args[3:]).replace(tmp, d) + "\nbinary_sha " + SHA + "\n")
    os.rename(tmp, d)
    return "ok"
if __name__ == "__main__":
    open(os.path.join(HERE, "pilot_r3.pid"), "w").write(str(os.getpid()))
    todo = [(a, s) for s in SEEDS for a in ARMS]
    with ThreadPoolExecutor(4) as ex:
        for t, res in zip(todo, ex.map(one, todo)):
            print(time.strftime("%H:%M:%S"), t, res, flush=True)
    print("done", SHA, flush=True)
