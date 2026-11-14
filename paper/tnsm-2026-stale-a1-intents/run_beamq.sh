#!/bin/bash
cd "$(dirname "$0")"
echo $$ > run_beamq.pid
mkdir -p beam_quant
for n in 128 64; do python3 beam_quant.py --seeds 5 --narr $n > beam_quant/log_N$n.txt 2>&1; done
rm -f run_beamq.pid
