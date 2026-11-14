#!/bin/bash
# Waits for run_all.sh to finish, then reruns the groups to fill any run that
# failed or is missing (finished runs are skipped).
cd "$(dirname "$0")"
echo $$ > run_rest.pid
while [ -f run_all.pid ] && kill -0 $(cat run_all.pid) 2>/dev/null; do sleep 10; done
for g in eps hb main ablation sens; do
  python3 run_campaign.py $g --jobs ${JOBS:-2} > ${g}_fill.log 2>&1
done
rm -f run_rest.pid
