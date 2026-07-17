#!/usr/bin/env bash
# gen_docs.sh - generate the Doxygen API documentation into docs/html/.
#
# The firmware version shown in the docs is extracted from include/version.h
# and injected into the Doxyfile via the FW_VERSION environment variable.
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v doxygen >/dev/null 2>&1; then
    echo "ERROR: doxygen not found (https://www.doxygen.nl/download.html)" >&2
    exit 1
fi

FW_VERSION="$(sed -n 's/^#define FW_VERSION "\(.*\)"/\1/p' include/version.h)"
export FW_VERSION

echo "Generating docs for WeatherStation V2 v${FW_VERSION} (doxygen $(doxygen --version))"
doxygen docs/Doxyfile

echo ""
echo "Done: docs/html/index.html"
