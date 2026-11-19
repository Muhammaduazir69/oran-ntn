#!/bin/bash
cd "$(dirname "$0")"
echo $$ > run_beamq64.pid
python3 beam_quant.py --seeds 5 --narr 64 --sigma 3.0 > beam_quant/log_N64.txt 2>&1
rm -f run_beamq64.pid
