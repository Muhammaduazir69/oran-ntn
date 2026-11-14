#!/bin/bash
# Standalone build of the data-plane binary against the frozen libraries; touches nothing under ns-3-dev.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
B=/home/uzair/6g_ntn_ns3/ns-3-dev/build
OUT="$HERE/../bin"
python3 "$HERE/apply_edits_dp.py"
(cd "$HERE" && diff -u orig/oran-ntn-feeder-mgmt.cc oran-ntn-feeder-mgmt_dp.cc > oran-ntn-feeder-mgmt_dp.patch || true)
FLAGS="-DEIGEN_MPL2_ONLY -DHAVE_BOOST -DHAVE_BOOST_UNITS -DHAVE_EIGEN3 -DHAVE_GSL -DHAVE_LIBXML2 -DHAVE_PACKET_H -DHAVE_SQLITE3 -DHAVE_STDINT_H -DNS3_BUILD_PROFILE_OPTIMIZED -D__LINUX__ -I$B/include -I/usr/include/eigen3 -I/usr/include/libxml2 -O3 -DNDEBUG -std=c++20 -fPIE -fno-semantic-interposition -march=native -mtune=native"
LIBS=$(ls "$OUT"/lib/*.so)
nice -n 19 g++ $FLAGS -c "$HERE/oran-ntn-feeder-mgmt_dp.cc" -o "$OUT/p3dp8.o"
nice -n 19 g++ -o "$OUT/p3-dataplane.r8" "$OUT/p3dp8.o" $LIBS -Wl,-rpath,"$OUT/lib" -Wl,-rpath-link,"$OUT/lib"
rm -f "$OUT/p3dp8.o"
sha256sum "$OUT/p3-dataplane.r8"
