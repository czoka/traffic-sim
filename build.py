#!/usr/bin/env python3
"""One-command build for the traffic sim.

    python3 build.py setup     create .venv/ and install SCons into it (once)
    python build.py            desktop extension (debug + release) + C++ tests,
                               plus the web extension when Emscripten is on PATH
    python build.py web        web extension only (and wasm tests under node)
    python build.py export     also export the Web build to build/web
                               (needs GODOT=/path/to/godot and export templates)
    python build.py serve      serve build/web on http://localhost:8060

Options: -j N (parallel jobs), --no-tests.

Python packages live only in the project's .venv/. You don't need to activate it:
build.py switches to .venv's Python by itself.
"""

import argparse
import functools
import http.server
import os
import platform
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
WEB_OUT = os.path.join(ROOT, "build", "web")
VENV = os.path.join(ROOT, ".venv")
VENV_PYTHON = os.path.join(VENV, "Scripts", "python.exe") if os.name == "nt" else os.path.join(VENV, "bin", "python")


def in_project_venv():
    return os.path.realpath(sys.prefix) == os.path.realpath(VENV)


def setup_venv():
    """Creates .venv/ and installs the pinned build tools into it."""
    if not os.path.isfile(VENV_PYTHON):
        run([sys.executable, "-m", "venv", VENV])
    run([VENV_PYTHON, "-m", "pip", "install", "--upgrade", "pip"])
    run([VENV_PYTHON, "-m", "pip", "install", "-r", os.path.join(ROOT, "requirements.txt")])
    print("Build tools installed in .venv/. Now run: python3 build.py")


def use_project_venv():
    """Re-runs this script with .venv's Python so SCons comes from the venv."""
    if in_project_venv():
        return
    if not os.path.isfile(VENV_PYTHON):
        sys.exit("No .venv yet. Run once:  python3 build.py setup")
    os.execv(VENV_PYTHON, [VENV_PYTHON, os.path.abspath(__file__)] + sys.argv[1:])


def run(cmd, **kw):
    print("+ " + " ".join(cmd), flush=True)
    subprocess.check_call(cmd, cwd=ROOT, **kw)


def host_platform():
    s = platform.system()
    if s == "Darwin":
        return "macos"
    if s == "Windows":
        return "windows"
    return "linux"


def ensure_submodule():
    if not os.path.isfile(os.path.join(ROOT, "godot-cpp", "SConstruct")):
        run(["git", "submodule", "update", "--init", "--recursive"])


def scons(args, jobs):
    cmd = [sys.executable, "-m", "SCons"]  # SCons from .venv (see use_project_venv)
    if jobs:
        cmd.append("-j%d" % jobs)
    run(cmd + args)


def build_desktop(jobs, tests):
    plat = host_platform()
    for target in ("template_debug", "template_release"):
        scons(["platform=" + plat, "target=" + target], jobs)
    if tests:
        scons(["platform=" + plat, "tests", "bench"], jobs)


def build_web(jobs, tests):
    if shutil.which("emcc") is None:
        sys.exit("emcc not found: install Emscripten 4.0.11 (emsdk install 4.0.11 && emsdk activate 4.0.11)")
    for target in ("template_debug", "template_release"):
        scons(["platform=web", "threads=no", "target=" + target], jobs)
    if tests and shutil.which("node"):
        scons(["platform=web", "threads=no", "tests"], jobs)


def export_web():
    godot = os.environ.get("GODOT") or shutil.which("godot")
    if not godot:
        sys.exit("Set GODOT=/path/to/Godot (4.7.x) to export.")
    os.makedirs(WEB_OUT, exist_ok=True)
    # Import resources first so the export sees the extension and scripts.
    run([godot, "--headless", "--path", "game", "--import"])
    run([godot, "--headless", "--path", "game", "--export-release", "Web", os.path.join(WEB_OUT, "index.html")])
    print("Web build written to " + WEB_OUT)


def serve(port=8060):
    if not os.path.isfile(os.path.join(WEB_OUT, "index.html")):
        sys.exit("No web build yet. Run: python build.py export")
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=WEB_OUT)
    print("Serving %s on http://localhost:%d (Ctrl+C to stop)" % (WEB_OUT, port))
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="?", default="all", choices=["setup", "all", "desktop", "web", "export", "serve"])
    ap.add_argument("-j", type=int, default=0, help="parallel jobs")
    ap.add_argument("--no-tests", action="store_true")
    a = ap.parse_args()

    if a.command == "setup":
        setup_venv()
        return
    if a.command == "serve":
        serve()
        return
    use_project_venv()
    ensure_submodule()
    tests = not a.no_tests
    if a.command in ("all", "desktop", "export"):
        build_desktop(a.j, tests)
    if a.command == "web" or (a.command in ("all", "export") and shutil.which("emcc")):
        build_web(a.j, tests)
    elif a.command == "all":
        print("Note: emcc not on PATH, skipped the web build.")
    if a.command == "export":
        export_web()


if __name__ == "__main__":
    main()
