#!/bin/bash
# resumes the round 2 runs whenever at least 3 GiB is free; the driver itself stops below that. Gives up after 8 h.
cd "$(dirname "$0")"
end=$(( $(date +%s) + 28800 ))
while [ "$(date +%s)" -lt "$end" ]; do
  n=$(find runs/c2 runs/siteheld -name summary.csv 2>/dev/null | wc -l)
  if [ "$n" -ge 1170 ]; then echo complete; exit 0; fi
  free=$(df --output=avail -B1 . | tail -1)
  if [ "$free" -gt 3100000000 ] && [ "$free" -gt 3221225472 ]; then
    python3 run_campaign.py r2 --jobs 3 >> campaign_r2.log 2>&1
  fi
  sleep 300
done
echo gave_up "$(find runs/c2 runs/siteheld -name summary.csv | wc -l)"
