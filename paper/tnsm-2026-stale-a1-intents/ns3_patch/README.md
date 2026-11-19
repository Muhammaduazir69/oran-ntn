# Paper 3 patch to the example oran-ntn-feeder-mgmt (Nov 2026)

`ns-3-dev` is shared with running campaigns, so this revision did not touch it or its build. All of the
article's ns-3 data come from one binary, `../bin/p3-feeder-mgmt`, built by `build.sh` from the files here,
with a frozen copy of the optimized libraries in `../bin/lib` (hashes: `../bin/lib_manifest.sha256`).

- `orig/`: the example as copied from `contrib/oran-ntn/examples` on 3 Nov 2026 (ns-3 5fa3c2f00).
- `oran-ntn-feeder-mgmt.cc`: patched copy; `oran-ntn-feeder-mgmt_p3.patch` is the diff against `orig/`.
- `apply_edits.py`: how the patched copy is produced from `orig/`.

Only the example changes; no model library symbol is overridden. Changes: event-driven beam process with
exact time integration in place of the 10 ms grid; lease estimator over total exposure; fixed lease timer;
higher lease cap; cell site and gateway list on the command line; NF host pin; beam stress options (dwell law,
direction persistence, correlated decision errors); one-step intents; slack on the command line.

Checked by run: the new options change the outputs (T_A1, fixed timer, pin at nf_onboard_frac = 1, cell and
gateway list), and with the former settings the run reproduces outages.csv and nf_miss.csv of the 29 Oct binary
byte for byte; beam columns differ as intended.

**To merge at the shared build:** apply `oran-ntn-feeder-mgmt_p3.patch` to
`contrib/oran-ntn/examples/oran-ntn-feeder-mgmt.cc` and rebuild. Nothing else depends on it.

## Round 3 (5 Nov 2026): data plane
`apply_edits_dp.py` applies a second set of edits on top of the above and writes `oran-ntn-feeder-mgmt_dp.cc`; `oran-ntn-feeder-mgmt_dp.patch` is the full diff against `orig/`. `build_dp.sh` builds `../bin/p3-dataplane` (sha256 first 16: 11977b92398de810), linked to the frozen libraries. New default-off options: `--dataPlane`, `--surge`, `--oracleIntent`, `--workConserving`, and the surge parameters. Check rows: pilot seed 1 hold reproduces dataplane.csv byte for byte from the pilot binary (`bin/p3-dataplane.pilot`, 544e1f0faa463bd0); with `--dataPlane=0` summary, outages, nf_miss, checks and switches of main/pal_s550_gw1 seed 1 match the earlier binary byte for byte.

**To merge at the shared build:** apply `oran-ntn-feeder-mgmt_dp.patch` to `contrib/oran-ntn/examples/oran-ntn-feeder-mgmt.cc` (it contains the earlier patch too) and rebuild.

## Round 4 (9 Nov 2026)
`apply_edits_dp.py` adds default-off `--surgeLeadS`, `--nsPeriodS`, `--surgeDwellS`, `--surgeDuty`, `--surgeUnlinked`; `build_dp.sh` rebuilds `../bin/p3-dataplane` (sha256 first 16: 528e659bb8bd4b35). Check rows: dp hold seed 21 byte-identical; data plane off reproduces main/pal seed 1. The round 3 build is kept as `../bin/p3-dataplane.r3`. Merge `oran-ntn-feeder-mgmt_dp.patch` at the shared build.

## Round 8 (10 Nov 2026)
`apply_edits_dp.py` adds default-off `--provisionAll` (push each A1 version to every satellite whose feeder is up, retried until acknowledged); `build_r8.sh` builds `../bin/p3-dataplane.r8` (sha256 first 16: 959bfda8edb1f5ab), leaving `../bin/p3-dataplane` (b7601e74c2728542) in place. Check rows with the option off are byte-identical (see `../PROVENANCE.txt`). Merge `oran-ntn-feeder-mgmt_dp.patch` at the shared build.
