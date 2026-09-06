#!/usr/bin/env bash
# run.sh -- build and run the kernel-log-ring harness on the build host.
#
#   ./Kernel/Source/Debug/serial/Tests/run.sh
#
# Compiles the real Serial.c (no copy, no #ifdef TEST) against the minimal
# stand-ins in stub/, which come first on the include path so they shadow the
# real headers. Same warning flags the kernel builds with.
set -euo pipefail
cd "$(dirname "$0")"

CC="${CC:-cc}"
OUT="${OUT:-/tmp/implusos-serial-log-test}"

$CC -std=c11 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -O1 -g \
    -I stub -I .. \
    serial_log_test.c ../Serial.c -o "$OUT"

"$OUT"
