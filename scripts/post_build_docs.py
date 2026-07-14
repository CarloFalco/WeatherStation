"""Post-build hook: regenerate the Doxygen documentation after a firmware build.

Wired via `extra_scripts = post:scripts/post_build_docs.py` in platformio.ini
(env:station only, so check.sh does not generate the docs twice). Runs only
when the firmware was actually (re)built - an up-to-date build skips post
actions. Missing doxygen is a warning, never a build failure.
"""

import os
import re
import shutil
import subprocess

Import("env")  # noqa: F821 - provided by the PlatformIO SCons runtime


def generate_docs(source, target, env):
    project_dir = env.subst("$PROJECT_DIR")

    doxygen = shutil.which("doxygen")
    if not doxygen:
        print("post_build_docs: doxygen not found, documentation NOT updated")
        return

    # Same version extraction as scripts/gen_docs.sh: the Doxyfile expands
    # PROJECT_NUMBER from the FW_VERSION environment variable.
    version = "unknown"
    version_h = os.path.join(project_dir, "include", "version.h")
    with open(version_h, encoding="utf-8") as fh:
        match = re.search(r'#define FW_VERSION "([^"]+)"', fh.read())
        if match:
            version = match.group(1)

    print(f"post_build_docs: regenerating Doxygen docs (v{version})...")
    result = subprocess.run(
        [doxygen, os.path.join("docs", "Doxyfile")],
        cwd=project_dir,
        env={**os.environ, "FW_VERSION": version},
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        print("post_build_docs: docs/html/index.html updated")
    else:
        # Never break the firmware build over documentation.
        print("post_build_docs: doxygen FAILED (build not affected):")
        print((result.stderr or result.stdout).strip())


env.AddPostAction("buildprog", generate_docs)  # noqa: F821
