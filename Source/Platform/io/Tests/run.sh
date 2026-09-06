#!/usr/bin/env bash
# run.sh -- build and run the block-cache harness on the build host.
#
#   ./Kernel/Source/Platform/io/Tests/run.sh
#
# Compiles the real Block_Cache.c (no copy, no #ifdef TEST) against the
# minimal kernel stand-ins in stub/, which come first on the include path so
# they shadow the real headers. Same warning flags the kernel builds with.
set -euo pipefail
cd "$(dirname "$0")"

CC="${CC:-cc}"
OUT="${OUT:-/tmp/implusos-block-cache-test}"

$CC -std=c11 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -O1 -g \
    -I stub -I .. \
    block_cache_test.c ../Block_Cache.c -o "$OUT"

"$OUT"
