#!/bin/bash
# Round 3 acceptance review, issue 3 (20 Oct 2026): envelope-fallback PAL runs
# at eps = 0.078 for the cost check at the default weights; binary p3bin3
# (ns-3 4355d1a22). Finished runs are skipped.
cd "$(dirname "$0")"
echo $$ > run_r3b.pid
python3 run_campaign.py variants --jobs ${JOBS:-6} > r3b_variants.log 2>&1
rm -f run_r3b.pid
