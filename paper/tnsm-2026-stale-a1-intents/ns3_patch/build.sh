#!/bin/bash
# Standalone build of the paper's one binary; touches nothing under ns-3-dev.
# Flags: cmake-cache/compile_commands.json entry of the example. Links the shared optimized libraries.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
B=/home/uzair/6g_ntn_ns3/ns-3-dev/build
OUT="$HERE/../bin"
mkdir -p "$OUT"
python3 "$HERE/apply_edits.py"
(cd "$HERE" && diff -u orig/oran-ntn-feeder-mgmt.cc oran-ntn-feeder-mgmt.cc > oran-ntn-feeder-mgmt_p3.patch || true)
FLAGS="-DEIGEN_MPL2_ONLY -DHAVE_BOOST -DHAVE_BOOST_UNITS -DHAVE_EIGEN3 -DHAVE_GSL -DHAVE_LIBXML2 -DHAVE_PACKET_H -DHAVE_SQLITE3 -DHAVE_STDINT_H -DNS3_BUILD_PROFILE_OPTIMIZED -D__LINUX__ -I$B/include -I/usr/include/eigen3 -I/usr/include/libxml2 -O3 -DNDEBUG -std=c++20 -fPIE -fno-semantic-interposition -march=native -mtune=native"
LIBS=$(ldd $B/contrib/oran-ntn/examples/ns3.43-oran-ntn-feeder-mgmt-optimized | awk '/libns3.43/{print $3}')
nice -n 10 g++ $FLAGS -c "$HERE/oran-ntn-feeder-mgmt.cc" -o "$OUT/p3.o"
nice -n 10 g++ -o "$OUT/p3-feeder-mgmt" "$OUT/p3.o" $LIBS -Wl,-rpath,$B/lib -Wl,-rpath-link,$B/lib
rm -f "$OUT/p3.o"
sha256sum "$OUT/p3-feeder-mgmt"
