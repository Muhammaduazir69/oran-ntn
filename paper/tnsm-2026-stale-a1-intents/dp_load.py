import csv,glob,os,math
import numpy as np
from scipy import stats
H=os.path.dirname(os.path.abspath(__file__))
def rd(gr,case,arm):
    o={}
    for d in sorted(glob.glob(f"{H}/runs/{gr}/{case}/{arm}/seed*")):
        s=int(d[-2:]);r={k:float(v) for k,v in next(csv.DictReader(open(d+'/dataplane.csv'))).items()}
        S=next(csv.DictReader(open(d+'/summary.csv')))
        for k in ('sla_viol_frac','superseded_frac','overprov_mean','fallback_frac','feeder_avail','short_depth_mean'): r[k]=float(S[k])
        r['crit_loss_cap']=(r['crit_offered']-r['crit_delivered'])/r['cap_bytes']
        o[s]=r
    return o
def ci(x):
    x=np.asarray(x);return x.mean(),stats.t.ppf(.975,len(x)-1)*x.std(ddof=1)/math.sqrt(len(x))
