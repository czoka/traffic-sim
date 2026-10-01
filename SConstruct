#!/usr/bin/env python
#
# Builds the traffic_sim GDExtension (simulation core + Godot bridge) and the
# native test and benchmark programs.
#
#   scons                               extension for the host, template_debug
#   scons target=template_release       release extension
#   scons platform=web threads=no       web extension (needs Emscripten 4.0.11)
#   scons tests                         build and run the C++ unit tests
#   scons bench                         build the headless benchmark
#   scons platform=web tests            unit tests compiled to wasm, run under node
#
# Most people just run:  python build.py

import os

env = SConscript("godot-cpp/SConstruct", {"api_version": "4.7"})

CORE_SOURCES = [
    "core/src/curve.cpp",
    "core/src/demo_maps.cpp",
    "core/src/document.cpp",
    "core/src/map.cpp",
    "core/src/network.cpp",
    "core/src/ped_network.cpp",
    "core/src/road_geometry.cpp",
    "core/src/road_map.cpp",
    "core/src/road_map_json.cpp",
    "core/src/traffic.cpp",
    "core/src/traffic_peds.cpp",
    "core/src/traffic_run.cpp",
    "core/src/sim.cpp",
    "core/src/validation.cpp",
    "third_party/clipper2/src/clipper.engine.cpp",
    "third_party/clipper2/src/clipper.offset.cpp",
    "third_party/clipper2/src/clipper.rectclip.cpp",
]
CORE_INCLUDES = ["#core/include", "#third_party", "#third_party/clipper2/include"]
# nlohmann::json is used in its no-throw mode (the core builds without exceptions).
CORE_DEFINES = ["JSON_NOEXCEPTION"]


def add_determinism_flags(e, msvc):
    # Identical floating-point results on every platform: no fused multiply-add
    # contraction (clang on Apple Silicon contracts by default) and no fast-math.
    if msvc:
        e.Append(CCFLAGS=["/fp:precise"])
    else:
        e.Append(CCFLAGS=["-ffp-contract=off", "-fno-fast-math"])


# --- GDExtension ------------------------------------------------------------

ext_env = env.Clone()
# Per-target object names, so debug/release/web builds don't overwrite each other.
ext_env["SHOBJSUFFIX"] = env["suffix"] + env["SHOBJSUFFIX"]
ext_env.Append(CPPPATH=CORE_INCLUDES + ["#extension/src"], CPPDEFINES=CORE_DEFINES)
add_determinism_flags(ext_env, env.get("is_msvc", False))

sources = CORE_SOURCES + Glob("extension/src/*.cpp")
lib_name = "libtraffic_sim"

if env["platform"] == "macos":
    base = "{}.{}.{}".format(lib_name, env["platform"], env["target"])
    library = ext_env.SharedLibrary("game/bin/{}.framework/{}".format(base, base), source=sources)
else:
    library = ext_env.SharedLibrary(
        "game/bin/{}{}{}".format(lib_name, env["suffix"], env["SHLIBSUFFIX"]), source=sources
    )

env.NoCache(library)
Default(library)


# --- Native tests and benchmark ----------------------------------------------
# Built with a plain host toolchain (or Emscripten + node for platform=web), not
# with the godot-cpp environment, so they run as ordinary programs.

def make_program_env():
    if env["platform"] == "web":
        t = Environment(ENV=os.environ, tools=["default"], CC="emcc", CXX="em++", LINK="em++", AR="emar",
                        RANLIB="emranlib", PROGSUFFIX=".js", OBJSUFFIX=".o")
        t.Append(CXXFLAGS=["-std=c++17", "-O2", "-fno-exceptions"])
        t.Append(LINKFLAGS=["-sENVIRONMENT=node", "-sALLOW_MEMORY_GROWTH=1", "-sEXIT_RUNTIME=1",
                            "-sSINGLE_FILE=1", "-sSTACK_SIZE=1048576"])
        add_determinism_flags(t, False)
        return t, "node "
    t = Environment(ENV=os.environ)
    msvc = bool(t.get("MSVC_VERSION"))
    if msvc:
        t.Append(CXXFLAGS=["/std:c++17", "/O2", "/utf-8", "/nologo"], CPPDEFINES=["_HAS_EXCEPTIONS=0"])
    else:
        t.Append(CXXFLAGS=["-std=c++17", "-O2", "-fno-exceptions", "-Wall", "-Wextra"])
    add_determinism_flags(t, msvc)
    return t, ""


prog_env, runner = make_program_env()
prog_env.Append(CPPPATH=CORE_INCLUDES, CPPDEFINES=CORE_DEFINES)
build_dir = "build/{}".format("web" if env["platform"] == "web" else "native")
core_objs = [
    prog_env.Object("{}/core/{}".format(build_dir, os.path.splitext(os.path.basename(s))[0]), s)
    for s in CORE_SOURCES
]

test_objs = [
    prog_env.Object("{}/tests/{}".format(build_dir, os.path.splitext(os.path.basename(str(t)))[0]), t)
    for t in sorted(Glob("tests/*.cpp"), key=str)
]
tests_prog = prog_env.Program("{}/tsim_tests".format(build_dir), core_objs + test_objs)
bench_prog = prog_env.Program("{}/tsim_bench".format(build_dir),
                              core_objs + [prog_env.Object("{}/tools/bench_main".format(build_dir),
                                                           "tools/bench_main.cpp")])

run_tests = prog_env.Command("{}/tests.passed".format(build_dir), tests_prog,
                             [runner + "\"${SOURCE.abspath}\"", Touch("$TARGET")])
AlwaysBuild(run_tests)
Alias("tests", run_tests)
Alias("bench", bench_prog)
