#!/usr/bin/env python3
"""Reads the pilot of round 3 and applies the pre-registered rule. Writes tables/pilot_r3.txt."""
import csv, glob, math, os, sys
import numpy as np
from scipy import stats
H = os.path.dirname(os.path.abspath(__file__))
GROUP = "pilot_r3_rep" if "rep" in sys.argv else "pilot_r3"
def rd(arm):
    out = {}
    for d in sorted(glob.glob(f"{H}/runs/{GROUP}/{arm}/seed*")):
        if d.endswith('.tmp'): continue
        s = int(d[-2:])
        r = {k: float(v) for k, v in next(csv.DictReader(open(d + "/dataplane.csv"))).items()}
        r.update({k: float(v) for k, v in next(csv.DictReader(open(d + "/summary.csv"))).items() if k in ("sla_viol_frac","superseded_frac","overprov_mean","short_depth_mean","fallback_frac","feeder_avail","served_s")})
        out[s] = r
    return out
A = {a: rd(a) for a in ("control", "hold", "lease")}
seeds = sorted(set(A["control"]) & set(A["hold"]) & set(A["lease"]))
def v(a, k, sc=100): return np.array([A[a][s][k] * sc for s in seeds])
def pair(x):
    m = x.mean(); h = stats.t.ppf(.975, len(x)-1) * x.std(ddof=1) / math.sqrt(len(x))
    return m, h, int((x < 0).sum()), int((x > 0).sum())
L = [f"seeds {len(seeds)}"]
for a in A:
    L.append(f"{a}: crit_ratio {v(a,'crit_ratio').mean():.3f} be_share {v(a,'be_share').mean():.3f} total {v(a,'total_share').mean():.3f} p99 {v(a,'crit_p99_ms',1).mean():.0f} short {v(a,'sla_viol_frac').mean():.2f} over {v(a,'overprov_mean').mean():.2f} super {v(a,'superseded_frac').mean():.2f} fb {v(a,'fallback_frac').mean():.2f} avail {v(a,'feeder_avail').mean():.2f}")
for nm, x, y in [("hold-control", "hold", "control"), ("lease-hold", "lease", "hold"), ("lease-control", "lease", "control")]:
    for k in ("crit_ratio", "be_share", "total_share"):
        m, h, n, p = pair(v(x, k) - v(y, k))
        L.append(f"{nm} {k}: {m:+.3f} +- {h:.3f} (neg {n}, pos {p})")
b = [A[a][s]["crit_offered"]-A[a][s]["crit_delivered"]-A[a][s]["crit_dropped"]-A[a][s]["crit_queued"]-A[a][s]["crit_inflight"] for a in A for s in seeds]
L.append(f"balance max {max(abs(x) for x in b)}")
m1,h1,n1,_ = pair(v("hold","crit_ratio")-v("control","crit_ratio"))
m2,h2,n2,_ = pair(v("hold","be_share")-v("control","be_share"))
p1 = m1 <= -0.5 and m1+h1 < 0 and n1 >= 0.8*len(seeds)
p2 = m2 <= -1.0 and m2+h2 < 0 and n2 >= 0.8*len(seeds)
L += [f"P1 met: {p1}", f"P2 met: {p2}", f"rule met: {p1 or p2}"]
print("\n".join(L))
open(os.path.join(H, "..", "tables", GROUP + ".txt"), "w").write("\n".join(L) + "\n")
