# Traffic Sim

A traffic simulation map builder: a C++ simulation core running inside Godot 4, targeting desktop and the browser.

**Status: proof of concept (POC).** The POC answers one question before any real features are built: can a C++ simulation inside Godot's web export handle thousands of cars? It runs IDM cars on a multi-lane ring road, draws them with one MultiMesh, and measures tick time, frame time and determinism.

The design lives in the Game Design Document (claude.ai artifact "Traffic Sim Map Builder — Game Design Document"). Its *Implementation plan* tab has the POC checklist and gate.

## Quick start (macOS)

```sh
git clone --recursive git@github.com:czoka/traffic-sim.git
cd traffic-sim
python3 build.py setup            # once: creates .venv/ and installs SCons there
python3 build.py                  # macOS extension (debug + release) + C++ tests
```

Python packages live only in the project's `.venv/` (pinned in `requirements.txt`); nothing is installed system-wide. You don't need to activate the venv: `build.py` switches to `.venv`'s Python by itself. To run SCons by hand, use `source .venv/bin/activate` first, or call `.venv/bin/scons`.

Then open `game/project.godot` in **Godot 4.7.x** and press Play (F5).

For the web build you also need Emscripten **4.0.11**, the version Godot 4.7's web templates are built with, plus Godot's export templates (Editor → Manage Export Templates):

```sh
brew install emscripten           # or: emsdk install 4.0.11 && emsdk activate 4.0.11
python3 build.py                  # now also builds the web extension
GODOT=/Applications/Godot.app/Contents/MacOS/Godot python3 build.py export
python3 build.py serve            # http://localhost:8060
```

The web build is single-threaded, so it needs no special server headers (no COOP/COEP).

## What's in the POC

| Area | What it does |
| --- | --- |
| Map model | Nodes, straight and arc segments, lanes with stable IDs that are never reused |
| Simulation | Fixed 10 Hz tick, seeded RNG, IDM car-following, synchronous updates, leaders found across lane boundaries |
| Determinism | FNV-1a hash of the whole state; a golden scenario must give the same hash on every platform |
| Bridge | `TrafficSim` class: C++ writes interpolated car transforms and colours into one packed float array per frame; GDScript hands it to a MultiMesh |
| Time slicing | `advance(delta, speed, budget_ms)` runs due ticks until an 8 ms budget is spent, then drops the backlog so frames never stall |
| Editor stub | Pan/zoom camera, throwaway click-to-place straight road, JSON save/load |
| Measurement | In-game stats panel, a benchmark mode, and a headless native benchmark |

### Controls

| Input | Action |
| --- | --- |
| Space / `.` | Pause / step one tick |
| `1`–`5` | Speed 1x, 4x, 16x, 64x, 128x |
| `R` | Road tool: click start, click end; Esc or right-click cancels |
| `F` | Fit the map in view |
| Wheel / pinch | Zoom at the cursor |
| Right- or middle-drag / arrows / trackpad | Pan |
| Ctrl/Cmd+S, Ctrl/Cmd+O | Save / load `user://maps/ring.json` (on the web also downloads the JSON) |

Cars are coloured by speed relative to their desired speed: red is stopped, amber is about half speed, green is free flow.

## Measuring the gate

The POC gate: **2,000 cars at 60 fps in a desktop browser; the same seed gives an identical state hash on web and desktop; a clean checkout builds with one command.**

* **In the app:** click *Benchmark* (or *Determinism* for just the hash check).
* **Desktop, unattended:** `godot --path game -- --bench` runs every phase, writes `user://bench_results.json` and exits non-zero if the gate fails.
* **Browser:** open the exported page with `?bench` in the URL, e.g. `http://localhost:8060/?bench`. It downloads `traffic-sim-bench.json` when done.

The benchmark runs 1,000 / 2,000 / 5,000 cars at 1x, then 2,000 cars at 16x and 128x, and 5,000 cars at 128x. For each phase it reports fps, p95 frame time, sim milliseconds per frame, microseconds per tick and the speed actually achieved.

### Results so far

Measured in the cloud build environment (one core of an Intel Xeon at 2.1 GHz, Linux). **These are not the gate numbers:** those must come from a desktop browser on real hardware.

| | 1,000 cars | 2,000 cars | 5,000 cars |
| --- | --- | --- | --- |
| Tick, native C++ | 10.7 µs | 21.7 µs | 67.8 µs |
| Tick, wasm in Node (V8) | – | ~49–64 µs | – |
| Sim work per frame at 128x, Godot headless | – | 0.20 ms | 0.65 ms |

* At 128x, 2,000 cars need about 13 ticks per 60 Hz frame. Even at the wasm speed that is under 1 ms of the 16.7 ms frame.
* Determinism: the golden scenario (2,000 cars, seed 42, 6,000 ticks = 10 sim minutes) hashes to `d3b987055dad39fb` with GCC, with Clang `-O3 -march=native`, in wasm (Emscripten + V8), and inside Godot 4.7.2. Allowing FMA contraction changes the hash, which is why `-ffp-contract=off` is mandatory.
* Stop-and-go waves: at 1,000 cars traffic flows freely at about 50 km/h. At 2,000 cars waves form within about 4 minutes: 17% of cars are stopped, and the speed spread is 16 km/h.

## Repository layout

```
core/            simulation core, pure C++17, no Godot includes
  include/tsim/  types, map, JSON, simulation, RNG, hash
  src/
extension/src/   GDExtension bridge (TrafficSim class, registration)
game/            Godot 4.7 project (Compatibility renderer)
  scripts/       main scene, HUD, camera, road layer, road tool, benchmark
  maps/ring.json the bundled ring map (loaded at start)
  tests/         engine smoke test (headless)
  bin/           built extension binaries (git-ignored, except macOS Info.plist)
tests/           C++ unit tests (doctest)
tools/           headless benchmark (tsim_bench)
third_party/     doctest 2.4.12, nlohmann/json 3.12.0 (both MIT)
godot-cpp/       submodule, godot-cpp v10 (master @ 507ed9d), api_version 4.7
build.py         one-command build (uses .venv/)
requirements.txt pinned Python build tools (SCons)
SConstruct       extension, tests and benchmark
```

## Build details

`python3 build.py` wraps these SCons commands (run them inside the activated `.venv`):

```sh
scons platform=macos target=template_debug      # or linux / windows
scons platform=macos target=template_release
scons platform=macos tests bench                # build and run C++ tests, build benchmark
scons platform=web threads=no target=template_debug
scons platform=web threads=no tests             # tests compiled to wasm, run under node
```

Other checks:

```sh
build/native/tsim_bench                         # golden scenario: timings + PASS/FAIL
build/native/tsim_bench --cars 5000 --ticks 6000
godot --headless --path game --script res://tests/smoke_test.gd
```

CI (`.github/workflows/ci.yml`) builds Linux, Windows (MSVC) and macOS (Apple Silicon runner) plus the web extension. It runs the unit tests on every platform, including in wasm under Node, so the golden hash is checked across x86-64, ARM64 and wasm on every push. On Linux it also runs the Godot smoke test and a quick benchmark.

## Rules the core follows

* **No Godot in `core/`.** Everything is testable headless.
* **Determinism.** Inside `Simulation::tick()` only `+ - * /` and `sqrt` are used on doubles. There is no `sin`, `cos`, `pow` or `atan2` there, because libm differs between platforms. Builds use `-ffp-contract=off` (MSVC: `/fp:precise`). Iteration order is always by ID or index, and sorting uses a strict total order.
* **Stable IDs.** Nodes, segments, lanes and vehicles get IDs from counters that are saved with the map and never reused.
* **Versioned saves.** Map files carry `"format": "traffic-sim-map"` and `"version"`. Newer versions are rejected with a message, and migrations go in `map_json.cpp`.
* **Coordinates.** Metres, with y pointing down (same as Godot 2D). Lane 0 is the rightmost lane in the direction of travel; a positive arc sweep runs clockwise on screen.

## Known limits (by design, for the POC)

* One-way roads only, and no lane changes (MOBIL is M2). Cars follow the first connector of each lane.
* The road tool places an unconnected two-lane road with no junctions or snapping. It exists to prove the edit → rebuild path; cars never drive on it.
* The web build is single-threaded. If the browser gate is missed, the plan is to decide on a threaded web build or lower-cost agents before M1.
* Browser timers can be coarse (Firefox rounds to 1 ms without cross-origin isolation), so per-tick numbers on the web are averaged over many frames.
