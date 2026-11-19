#!/bin/bash
# Round 3 (20 Oct 2026): rerun the main group from ns-3 4355d1a22 (shortfall
# depth for every strategy, sat-pred added), then add the sat-pred runs of
# the sens and variants groups (finished runs are skipped).
cd "$(dirname "$0")"
echo $$ > run_r3.pid
for g in main sens variants; do python3 run_campaign.py $g --jobs ${JOBS:-8} > r3_$g.log 2>&1; done
rm -f run_r3.pid
