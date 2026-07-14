#!/usr/bin/env bash
# check.sh - single quality gate for WeatherStation V2. Exit 0 = all green.
#
# Usage:
#   ./scripts/check.sh          # native tests + firmware build
#   ./scripts/check.sh --fast   # firmware build only (skip host tests)
#
# Claude Code and CI run this after every change; a red step must be fixed
# before moving on (CLAUDE.md rule #1).
set -euo pipefail
cd "$(dirname "$0")/.."

# pio is not always on PATH (Windows: PlatformIO core lives in ~/.platformio)
if command -v pio >/dev/null 2>&1; then
    PIO=pio
elif [ -x "$HOME/.platformio/penv/Scripts/pio.exe" ]; then
    PIO="$HOME/.platformio/penv/Scripts/pio.exe"
elif [ -x "$HOME/.platformio/penv/bin/pio" ]; then
    PIO="$HOME/.platformio/penv/bin/pio"
else
    echo "ERROR: PlatformIO CLI not found" >&2
    exit 1
fi

FAST=0
[ "${1:-}" = "--fast" ] && FAST=1

if [ "$FAST" -eq 0 ]; then
    echo "=== 1/2 Unit test (native, host) ==="
    "$PIO" test -e native
else
    echo "=== 1/2 Unit test SKIPPED (--fast) ==="
fi

echo "=== 2/2 Build firmware (station) ==="
"$PIO" run -e station

echo ""
echo "check.sh: ALL GREEN"
