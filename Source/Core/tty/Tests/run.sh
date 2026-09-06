#!/usr/bin/env bash
# run.sh -- build and run the pty line-discipline harness on the build host.
#
#   ./Kernel/Source/Core/tty/Tests/run.sh
#
# Compiles the real Pty.c (no copy, no #ifdef TEST) against the minimal kernel
# stand-ins in stub/, which come first on the include path so they shadow the
# real Core/... headers. Same warning flags the kernel builds with.
set -euo pipefail
cd "$(dirname "$0")"

CC="${CC:-cc}"
OUT="${OUT:-/tmp/implusos-pty-test}"

$CC -std=c11 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -O1 -g \
    -I stub -I .. \
    pty_test.c ../Pty.c -o "$OUT"

"$OUT"
