#!/bin/bash
cd "$(dirname "$0")"
echo $$ > run_obl.pid
for g in main eps sens variants; do python3 run_campaign.py $g --jobs ${JOBS:-6} > obl_$g.log 2>&1; done
rm -f run_obl.pid
