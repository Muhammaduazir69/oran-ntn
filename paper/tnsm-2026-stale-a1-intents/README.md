# Code and data: stale A1 intents on a LEO satellite

Companion material for M. Uzair, "What a Stale A1 Intent Costs on a LEO Satellite: Packet-Level Service, Lease Fallbacks and Control Placement in O-RAN," 2026.

The study measures, on an ns-3 packet data plane with critical and best-effort slices, what it costs when an on-board O-RAN controller keeps enforcing an A1 policy it cannot refresh while the feeder link is down, compares intent-lease fallbacks with re-implemented recent designs, and finds the break-even for ground versus on-board near-RT control.

| Path | Contents |
|---|---|
| `ns3_patch/` | Campaign source of record (feeder-management data-plane example) and build script |
| `run_campaign*.py`, `run_r3.sh` to `run_r8.py`, `run_all.sh` | Campaign drivers, one per campaign stage (main grid, sites, long runs, provisioned case) |
| `runs.tar.xz` | Every run: command line and per-run data-plane summaries (unpack with `tar -xJf runs.tar.xz`) |
| `analyze*.py` | Statistics for each stage (paired by seed, Student-t intervals, sign counts) |
| `beam_*.py`, `beam_quant/`, `lease_value_mc.py`, `check_lease_derivation.py` | Beam predictor accuracy (INT8 and pruned), lease derivation checks, Monte Carlo |
| `verify_*.py`, `PROVENANCE.txt`, `runtime.txt` | Checks of the re-implemented baselines, build hashes and run provenance |

Reproduce: build ns-3.43 with the toolkit's `oran-ntn` module, run `ns3_patch` build, then the campaign drivers; or unpack `runs.tar.xz` and run the `analyze_r*.py` scripts.

Contact: Muhammad Uzair, muhammaduzairr69@gmail.com, ORCID 0009-0002-4104-2680. License: as the repository (GPL-2.0-only for code).
