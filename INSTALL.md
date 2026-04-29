# Install & run — oran-ntn

This guide walks you through installing the `oran-ntn` Space O-RAN reference module on top of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit)
or any vanilla ns-3.43 tree with `satellite`, `mmwave`, and (optionally) `ai`.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 (3.13 supported) |
| ns-3 | **3.43** |
| Disk | ~6 GB after build |

---

## 2. Prerequisites

### 2a. ns-3.43 tree

Easiest:

```bash
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
```

### 2b. SNS3 satellite (REQUIRED)

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

### 2c. mmWave (REQUIRED for NR PHY)

If not already in your tree:

```bash
cd contrib/
git clone https://gitlab.com/cttc-lena/nr.git mmwave
cd ..
```

### 2d. ns3-ai fork (OPTIONAL, only for RL-driven xApps)

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/ns3-ai.git ai
cd ..
```

---

## 3. Install the oran-ntn module

```bash
cd contrib/
git clone https://github.com/Muhammaduazir69/oran-ntn.git oran-ntn
cd ..
```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build oran-ntn
```

Verify:

```bash
./ns3 show profile | grep oran-ntn
```

---

## 5. Run examples

### 5a. Full scenario with 5 concurrent xApps (~1 min wall-clock)

```bash
./ns3 run "oran-ntn-full-scenario \
  --simTime=600 \
  --xapps=ho,beamhop,slice,doppler,tnntn \
  --feederDelay=4 \
  --rngRun=1 \
  --outputDir=oran-ntn-output/"
```

CSV outputs land under `oran-ntn-output/`:
`action_log.csv` · `xapp_metrics.csv` · `space_ric_metrics.csv` ·
`conflict_log.csv` · `kpm_dataset.csv`.

### 5b. Stressed-feeder regime (Space RIC takes over)

```bash
./ns3 run "oran-ntn-full-scenario \
  --simTime=600 \
  --xapps=ho,beamhop \
  --feederDelay=250 \
  --feederOutageEvery=30 \
  --feederOutageDuration=2"
```

### 5c. Run the test suite

```bash
./ns3 run "test-runner --suite=oran-ntn --verbose"
```

---

## 6. RL-driven xApp (requires ns3-ai)

```bash
# Train the HO-prediction xApp with DQN
cd contrib/oran-ntn/python/
pip install -r requirements.txt
python3 train_ho_xapp.py --algo=dqn --episodes=200
```

The agent uses the 68-feature observation exposed by the `ntn-cho` module via the ns3-ai shared-memory bridge. Training curves land in `runs/`.

---

## 7. Common issues

**`oran-ntn not found` after configure**
You're missing one of the dependencies. Verify all of `satellite`, `mmwave` are present in `contrib/`.

**Build cache filtering modules**
Run a clean configure:
`./ns3 configure --enable-modules='' --enable-tests --enable-examples`.

**`E2SM-HO-PRED` decoder errors**
Make sure you're on the latest commit — the ASN.1 decoder ships with the module under `model/asn1/`.

---

## 8. Reproduce the paper results

```bash
cd papers/sim_runs/
./run_oran_ntn_scenario.sh
python3 build_figures.py
```

---

## 9. Citing

See [README](README.md#cite-this-work).
