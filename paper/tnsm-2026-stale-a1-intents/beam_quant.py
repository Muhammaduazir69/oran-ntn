#!/usr/bin/env python3
"""Measured on-board accuracy gap delta of a reduced (INT8) beam predictor.

The NF class of the paper is a beam-tracking xApp whose on-board copy runs a
reduced model that loses delta points of decision accuracy (Sec. III-B). This
script measures delta for one concrete predictor instead of assuming it.

Geometry: the Walker-Delta shell of the paper (1584/72/1, 550 km, 53 deg) as
circular Kepler orbits, the cell at 20 N 64 E with a 25 deg serving mask and
the serving rule of the simulator (highest elevation, kept until it drops
below the mask). Each seed draws the Earth rotation angle at the start, as the
simulator's random epoch does. UEs are placed uniformly in a 10 km disc.

Beams: the serving satellite is nadir-pointing with a 64 x 64 half-wavelength
planar array and a DFT codebook in (u, v) = direction cosines along and across
track (earth-moving beams; the best beam of a UE changes every few seconds,
the dwell is reported). The xApp sees the 16 beams of a 4 x 4 block around
the UE's beam of one second earlier, measures their RSRP every 100 ms (the NF
period) with i.i.d. Gaussian error sigma_m dB, and from the last K = 8
measurements predicts which of the 16 beams is best 100 ms later.

Models: an MLP 128-256-128-16 in FP32 (the ground xApp), then the same weights
after post-training static INT8 quantization (per-channel weights, per-tensor
activations, calibrated on training data) executed by PyTorch's quantized
kernels (x86/fbgemm and qnnpack, the ARM mobile backend). Also reported: 50 %
magnitude pruning followed by INT8, and a 4-bit weight variant (simulated).
delta = top-1 accuracy (FP32) - top-1 accuracy (reduced), in points, on test
seeds disjoint from the training and calibration seeds. sigma_m is set so
that the FP32 model is near the paper's assumed ground accuracy a_g = 0.9.

Output: beam_quant/results_N<narr>.json (all numbers the paper uses).
Usage: beam_quant.py [--seeds 5] [--narr 64|128]
"""
import argparse
import json
import os
import time
import warnings

import numpy as np
import torch
import torch.nn as nn

warnings.simplefilter("ignore")
import torch.ao.quantization as tq  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "beam_quant")

RE = 6378137.0
MU = 3.986004418e14
WE = 7.2921150e-5
ALT = 550e3
INC = np.radians(53.0)
T_TOT, P_PL, F_PH = 1584, 72, 1
CELL_LAT, CELL_LON = 20.0, 64.0
SERVE_MASK = 25.0
N_ARR = 64           # array elements per axis -> 64 DFT beams per axis
NF_DT = 0.1          # NF period (s)
K_HIST = 8           # measurement epochs fed to the predictor
LAG_S = 1.0          # the 4 x 4 block is centered on the beam of 1 s earlier
DISC_KM = 10.0
UE_PER_SEG = 12


def geodetic_ecef(lat, lon):
    lat, lon = np.radians(lat), np.radians(lon)
    return RE * np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])


def walker_eci(t, theta0):
    """ECI positions/velocities (T,3) of the shell at times t (n,), and Earth angle."""
    a = RE + ALT
    n = np.sqrt(MU / a**3)
    S = T_TOT // P_PL
    p = np.repeat(np.arange(P_PL), S)
    j = np.tile(np.arange(S), P_PL)
    raan = 2 * np.pi * p / P_PL
    m0 = 2 * np.pi * j / S + 2 * np.pi * F_PH * p / T_TOT
    u = m0[None, :] + n * t[:, None]  # argument of latitude
    cu, su = np.cos(u), np.sin(u)
    cO, sO = np.cos(raan)[None, :], np.sin(raan)[None, :]
    ci, si = np.cos(INC), np.sin(INC)
    x = a * (cO * cu - sO * su * ci)
    y = a * (sO * cu + cO * su * ci)
    z = a * (su * si) * np.ones_like(cO)
    vx = a * n * (-cO * su - sO * cu * ci)
    vy = a * n * (-sO * su + cO * cu * ci)
    vz = a * n * (cu * si) * np.ones_like(cO)
    return np.stack([x, y, z], -1), np.stack([vx, vy, vz], -1)


def eci_to_ecef(v, ang):
    c, s = np.cos(ang), np.sin(ang)
    x = c[..., None] * v[..., 0:1] + s[..., None] * v[..., 1:2]
    y = -s[..., None] * v[..., 0:1] + c[..., None] * v[..., 1:2]
    return np.concatenate([x, y, v[..., 2:3]], -1)


def elevation(sat, gnd):
    d = sat - gnd
    up = gnd / np.linalg.norm(gnd)
    return np.degrees(np.arcsin(np.sum(d * up, -1) / np.linalg.norm(d, axis=-1)))


def serving_schedule(theta0, hours=1.0):
    t = np.arange(0, 3600 * hours + 1, 1.0)
    cell = geodetic_ecef(CELL_LAT, CELL_LON)
    segs, cur, start = [], -1, 0.0
    for k0 in range(0, len(t), 300):
        tt = t[k0:k0 + 300]
        r, _ = walker_eci(tt, theta0)
        r = eci_to_ecef(r, (theta0 + WE * tt)[:, None])
        el = elevation(r, cell[None, None, :])
        for i in range(len(tt)):
            if cur >= 0 and el[i, cur] >= SERVE_MASK:
                continue
            best = int(np.argmax(el[i]))
            if cur >= 0:
                segs.append((cur, start, tt[i]))
            if el[i, best] >= SERVE_MASK:
                cur, start = best, tt[i]
            else:
                cur = -1
    if cur >= 0:
        segs.append((cur, start, t[-1]))
    return [s for s in segs if s[2] - s[1] > 20.0]


def beam_uv(sat_idx, t, theta0, ue_ecef):
    """Direction cosines (u along track, v across track) of UEs (U,3) in the
    nadir-pointing body frame of satellite sat_idx at times t (n,): (n,U,2)."""
    r_all, v_all = walker_eci(t, theta0)
    r, v = r_all[:, sat_idx], v_all[:, sat_idx]
    ang = theta0 + WE * t
    # UE positions in ECI
    c, s = np.cos(ang)[:, None], np.sin(ang)[:, None]
    ue = np.stack([c * ue_ecef[None, :, 0] - s * ue_ecef[None, :, 1],
                   s * ue_ecef[None, :, 0] + c * ue_ecef[None, :, 1],
                   np.broadcast_to(ue_ecef[None, :, 2], c.shape[:1] + ue_ecef.shape[:1])], -1)
    zb = -r / np.linalg.norm(r, axis=-1, keepdims=True)
    xb = v - np.sum(v * zb, -1, keepdims=True) * zb
    xb /= np.linalg.norm(xb, axis=-1, keepdims=True)
    yb = np.cross(zb, xb)
    d = ue - r[:, None, :]
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    return np.stack([np.sum(d * xb[:, None, :], -1), np.sum(d * yb[:, None, :], -1)], -1)


CENT = -1 + (2 * np.arange(N_ARR) + 1) / N_ARR


def af_db(delta):
    x = np.pi * delta / 2
    num = np.sin(N_ARR * x)
    den = N_ARR * np.sin(x)
    g = np.where(np.abs(den) < 1e-12, 1.0, (num / np.where(np.abs(den) < 1e-12, 1, den)) ** 2)
    return 10 * np.log10(np.maximum(g, 1e-4))


def best_idx(w):
    return np.clip(np.round((w + 1) * N_ARR / 2 - 0.5).astype(int), 0, N_ARR - 1)


def make_dataset(seed, sigma_db, hours=1.0):
    rng = np.random.default_rng(seed)
    theta0 = rng.uniform(0, 2 * np.pi)
    cell = geodetic_ecef(CELL_LAT, CELL_LON)
    lat0, lon0 = np.radians(CELL_LAT), np.radians(CELL_LON)
    east = np.array([-np.sin(lon0), np.cos(lon0), 0.0])
    north = np.array([-np.sin(lat0) * np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)])
    X, Y, dwell_changes, dwell_time = [], [], 0, 0.0
    lag = int(round(LAG_S / NF_DT))
    for sat, t0, t1 in serving_schedule(theta0, hours):
        t = np.arange(t0, t1, NF_DT)
        rr = DISC_KM * 1e3 * np.sqrt(rng.uniform(0, 1, UE_PER_SEG))
        ph = rng.uniform(0, 2 * np.pi, UE_PER_SEG)
        ue = cell[None, :] + (rr * np.cos(ph))[:, None] * east + (rr * np.sin(ph))[:, None] * north
        uv = beam_uv(sat, t, theta0, ue)  # (n,U,2)
        bi, bj = best_idx(uv[..., 0]), best_idx(uv[..., 1])
        ch = (np.diff(bi, axis=0) != 0) | (np.diff(bj, axis=0) != 0)
        dwell_changes += int(ch.sum())
        dwell_time += ch.shape[0] * ch.shape[1] * NF_DT
        n = len(t)
        ks = np.arange(max(K_HIST - 1, lag), n - 1)
        for uidx in range(UE_PER_SEG):
            ci = bi[ks - lag, uidx]
            cj = bj[ks - lag, uidx]
            oi = ci - 1 - rng.integers(0, 2, len(ks))
            oj = cj - 1 - rng.integers(0, 2, len(ks))
            oi = np.clip(oi, 0, N_ARR - 4)
            oj = np.clip(oj, 0, N_ARR - 4)
            gi = oi[:, None] + np.arange(4)[None, :]  # (m,4)
            gj = oj[:, None] + np.arange(4)[None, :]
            # history of true gains of the 16 beams at epochs k-K+1..k and target k+1
            hist = []
            for h in range(K_HIST - 1, -2, -1):
                u = uv[ks - h, uidx, 0][:, None]
                v = uv[ks - h, uidx, 1][:, None]
                gu = af_db(u - CENT[gi])  # (m,4)
                gv = af_db(v - CENT[gj])
                hist.append((gu[:, :, None] + gv[:, None, :]).reshape(-1, 16))
            target = hist[-1]
            meas = np.stack(hist[:-1], 1)  # (m,K,16) true dB
            meas = meas + rng.normal(0, sigma_db, meas.shape)
            meas = meas - meas[:, -1:, :].max(-1, keepdims=True)[:, :, :]  # relative to latest max
            X.append(np.clip(meas, -40, 0).reshape(len(ks), -1) / 40.0 + 0.5)
            Y.append(np.argmax(target, -1))
    return (np.concatenate(X).astype(np.float32), np.concatenate(Y).astype(np.int64),
            dwell_time / max(dwell_changes, 1))


class Mlp(nn.Module):
    def __init__(self, h1=256, h2=128):
        super().__init__()
        self.q = tq.QuantStub()
        self.f1 = nn.Linear(16 * K_HIST, h1)
        self.r1 = nn.ReLU()
        self.f2 = nn.Linear(h1, h2)
        self.r2 = nn.ReLU()
        self.f3 = nn.Linear(h2, 16)
        self.dq = tq.DeQuantStub()

    def forward(self, x):
        return self.dq(self.f3(self.r2(self.f2(self.r1(self.f1(self.q(x)))))))


def train(Xtr, Ytr, seed, epochs=12):
    torch.manual_seed(seed)
    m = Mlp()
    opt = torch.optim.Adam(m.parameters(), lr=2e-3)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    X, Y = torch.from_numpy(Xtr), torch.from_numpy(Ytr)
    g = torch.Generator().manual_seed(seed)
    for _ in range(epochs):
        perm = torch.randperm(len(X), generator=g)
        m.train()
        for b in range(0, len(X), 1024):
            idx = perm[b:b + 1024]
            opt.zero_grad()
            loss = nn.functional.cross_entropy(m(X[idx]), Y[idx])
            loss.backward()
            opt.step()
        sched.step()
    return m.eval()


def acc(m, X, Y):
    with torch.no_grad():
        p = torch.cat([m(torch.from_numpy(X[b:b + 8192])).argmax(-1) for b in range(0, len(X), 8192)])
    return (p.numpy() == Y), p.numpy()


def quantize(m, Xcal, engine):
    import copy
    torch.backends.quantized.engine = engine
    q = copy.deepcopy(m).eval()
    q.qconfig = tq.get_default_qconfig(engine)
    tq.prepare(q, inplace=True)
    with torch.no_grad():
        q(torch.from_numpy(Xcal))
    tq.convert(q, inplace=True)
    return q


def prune(m, frac=0.5):
    import copy
    p = copy.deepcopy(m)
    with torch.no_grad():
        for lin in [p.f1, p.f2, p.f3]:
            w = lin.weight
            thr = w.abs().flatten().kthvalue(int(frac * w.numel())).values
            w.mul_((w.abs() > thr).float())
    return p


def int4_weights(m):
    """Weight-only 4-bit, symmetric per output channel (simulated)."""
    import copy
    p = copy.deepcopy(m)
    with torch.no_grad():
        for lin in [p.f1, p.f2, p.f3]:
            w = lin.weight
            s = w.abs().amax(1, keepdim=True) / 7.0
            lin.weight.copy_(torch.clamp(torch.round(w / s), -8, 7) * s)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--sigma", type=float, default=None)
    ap.add_argument("--narr", type=int, default=64, help="array elements per axis (beam width)")
    a = ap.parse_args()
    global N_ARR, CENT
    N_ARR = a.narr
    CENT = -1 + (2 * np.arange(N_ARR) + 1) / N_ARR
    os.makedirs(OUT, exist_ok=True)
    torch.set_num_threads(4)
    t_start = time.time()
    # Calibrate sigma_m on a pilot so that FP32 accuracy is near a_g = 0.9.
    sigmas = [a.sigma] if a.sigma else [1.0, 1.5, 2.0, 2.5, 3.0]
    pilot = {}
    if not a.sigma:
        for sg in sigmas:
            Xa, Ya, _ = make_dataset(1001, sg, 0.5)
            Xb, Yb, _ = make_dataset(1002, sg, 0.5)
            m = train(Xa, Ya, 0, epochs=6)
            pilot[sg] = float(acc(m, Xb, Yb)[0].mean())
            print("pilot", sg, pilot[sg], flush=True)
        sigma = min(sigmas, key=lambda s: abs(pilot[s] - 0.9))
    else:
        sigma = a.sigma
    # Data: train seeds 1-3 (calibration taken from them), test seeds 101-102.
    tr = [make_dataset(s, sigma) for s in range(1, 4)]
    te = [make_dataset(s, sigma) for s in range(101, 103)]
    Xtr = np.concatenate([d[0] for d in tr]); Ytr = np.concatenate([d[1] for d in tr])
    Xte = np.concatenate([d[0] for d in te]); Yte = np.concatenate([d[1] for d in te])
    dwell = float(np.mean([d[2] for d in tr + te]))
    stay = float((np.argmax(Xte.reshape(len(Xte), K_HIST, 16)[:, -1, :], -1) == Yte).mean())
    print(f"sigma={sigma} train={len(Xtr)} test={len(Xte)} dwell={dwell:.2f}s stay={stay:.4f}", flush=True)
    rng = np.random.default_rng(7)
    res = []
    for sd in range(a.seeds):
        m = train(Xtr, Ytr, sd)
        cal = Xtr[rng.choice(len(Xtr), 4096, replace=False)]
        c32, _ = acc(m, Xte, Yte)
        row = {"seed": sd, "fp32": float(c32.mean())}
        for name, mm in [("int8_x86", quantize(m, cal, "x86")),
                         ("int8_qnnpack", quantize(m, cal, "qnnpack")),
                         ("prune50_int8_x86", quantize(prune(m), cal, "x86")),
                         ("int4w", int4_weights(m))]:
            if name.endswith("qnnpack"):
                torch.backends.quantized.engine = "qnnpack"
            elif "x86" in name:
                torch.backends.quantized.engine = "x86"
            cq, _ = acc(mm, Xte, Yte)
            row[name] = float(cq.mean())
            row["gap_" + name] = 100 * (row["fp32"] - row[name])
            # discordant pairs for a paired (McNemar) view
            row["b_" + name] = int((c32 & ~cq).sum())
            row["c_" + name] = int((~c32 & cq).sum())
        print(row, flush=True)
        res.append(row)

    def ci(v):
        v = np.asarray(v, float)
        from scipy import stats
        h = stats.t.ppf(0.975, len(v) - 1) * v.std(ddof=1) / np.sqrt(len(v)) if len(v) > 1 else 0.0
        return float(v.mean()), float(h)

    summ = {"sigma_db": sigma, "pilot": pilot, "n_train": int(len(Xtr)), "n_test": int(len(Xte)),
            "dwell_s": dwell, "stay_acc": stay, "seeds": a.seeds, "runs": res,
            "wall_s": time.time() - t_start, "torch": torch.__version__}
    for k in ["fp32", "int8_x86", "int8_qnnpack", "prune50_int8_x86", "int4w",
              "gap_int8_x86", "gap_int8_qnnpack", "gap_prune50_int8_x86", "gap_int4w"]:
        summ[k] = ci([r[k] for r in res])
    summ["narr"] = N_ARR
    json.dump(summ, open(os.path.join(OUT, f"results_N{N_ARR}.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in summ.items() if k not in ("runs",)}, indent=1))


if __name__ == "__main__":
    main()
