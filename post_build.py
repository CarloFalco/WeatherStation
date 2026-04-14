"""
post_build.py  –  WeatherStation post-build script
====================================================
PlatformIO loads this file on every `pio run` via:
    extra_scripts = post:post_build.py

The code at module level runs unconditionally on every build,
regardless of whether any source files changed.
"""

Import("env")   # SCons built-in – must stay at module level

import importlib.util
import os
import re
import shutil
import subprocess
import sys

# ---------------------------------------------------------------------------
# Paths (resolved via SCons variables – always absolute and correct)
# ---------------------------------------------------------------------------
PROJECT_DIR  = env.subst("$PROJECT_DIR")
BUILD_DIR    = env.subst("$BUILD_DIR")
HEADER_FILE  = os.path.join(PROJECT_DIR, "include", "version.h")
#VERSION_TXT  = os.path.join(PROJECT_DIR, "version.txt")
FIRMWARE_SRC = os.path.join(BUILD_DIR, "firmware.bin")
DOCS_SCRIPT  = os.path.join(PROJECT_DIR, "generate_docs.py")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def read_define(filepath, name):
    """Return the string value of a #define from a C header, or None."""
    pattern = re.compile(
        r'^\s*#define\s+' + re.escape(name) + r'\s+"?([^"\n]+)"?'
    )
    try:
        with open(filepath, encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                m = pattern.match(line)
                if m:
                    return m.group(1).strip().strip('"')
    except FileNotFoundError:
        print("[post_build] ERROR: file not found: {}".format(filepath))
    return None


def ensure_reportlab():
    """Import reportlab; auto-install into this interpreter if missing."""
    try:
        import reportlab  # noqa: F401
        return True
    except ImportError:
        pass
    print("[post_build] Installing reportlab into {}".format(sys.executable))
    r = subprocess.run(
        [sys.executable, "-m", "pip", "install", "reportlab"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    if r.returncode == 0:
        print("[post_build] reportlab installed OK")
        return True
    print("[post_build] pip install failed:\n{}".format(
        r.stderr.decode(errors="replace")))
    return False


def generate_pdf():
    """Run generate_docs.py in-process to avoid interpreter mismatch."""
    if not os.path.isfile(DOCS_SCRIPT):
        print("[post_build] generate_docs.py not found – skipping PDF")
        return

    if not ensure_reportlab():
        print("[post_build] reportlab unavailable – skipping PDF")
        return

    if PROJECT_DIR not in sys.path:
        sys.path.insert(0, PROJECT_DIR)

    _orig_argv = sys.argv[:]
    sys.argv = [DOCS_SCRIPT]          # prevent argparse from seeing pio args
    try:
        spec = importlib.util.spec_from_file_location("generate_docs", DOCS_SCRIPT)
        mod  = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.main()
        print("[post_build] PDF generated OK")
    except Exception as exc:
        print("[post_build] ERROR generating PDF: {}".format(exc))
        import traceback
        traceback.print_exc()
    finally:
        sys.argv = _orig_argv

# ---------------------------------------------------------------------------
# Main logic – executed at module level so it runs on EVERY `pio run`
# regardless of whether sources were recompiled.
# ---------------------------------------------------------------------------
print()
print("=" * 62)
print("  WeatherStation post-build")
print("  Python   : {}".format(sys.executable))
print("  Build dir: {}".format(BUILD_DIR))
print("=" * 62)

# 1. Read version
major = read_define(HEADER_FILE, "FW_VERSION_MAJOR")
minor = read_define(HEADER_FILE, "FW_VERSION_MINOR")
patch = read_define(HEADER_FILE, "FW_VERSION_PATCH")

if None in (major, minor, patch):
    print("[post_build] WARNING: could not parse version from version.h")
    _version = "unknown"
else:
    _version = "{}.{}.{}".format(major, minor, patch)

print("[post_build] Firmware version : {}".format(_version))

# 2. Write version.txt
'''
try:
    with open(VERSION_TXT, "w", encoding="utf-8") as _vf:
        _vf.write(_version + "\n")
    print("[post_build] version.txt written  : {}".format(VERSION_TXT))
except OSError as _e:
    print("[post_build] ERROR writing version.txt: {}".format(_e))
'''

# 3. Copy versioned .bin (only if firmware.bin exists)
if not os.path.isfile(FIRMWARE_SRC):
    print("[post_build] firmware.bin not found – skipping copy")
    print("[post_build] (run `pio run` once to build the firmware first)")
else:
    _dest_name = "WeatherStation_v{}.bin".format(_version)
    _dest_path = os.path.join(PROJECT_DIR, _dest_name)
    try:
        shutil.copy2(FIRMWARE_SRC, _dest_path)
        _size_kb = os.path.getsize(_dest_path) / 1024
        print("[post_build] Binary copied        : {}  ({:.1f} KB)".format(
            _dest_name, _size_kb))
    except OSError as _e:
        print("[post_build] ERROR copying .bin: {}".format(_e))

# 4. Generate PDF
generate_pdf()

print("=" * 62)
print()