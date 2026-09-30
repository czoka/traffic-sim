# Traffic Sim

A traffic simulation map builder: a C++ simulation core running inside Godot 4, targeting desktop and the browser.

**Status: M1 (road foundations) implemented.** The main scene is a road editor. You can draw straight and curved roads, which join into generated junctions. Each road has a cross-section profile, turn rules per approach and painted no-change lines. Undo/redo is unlimited, and maps save as versioned JSON. Cars don't drive on edited maps yet; that's M2. The POC's ring-road benchmark is still in the project for tracking sim performance and determinism.

The design lives in the Game Design Document (claude.ai artifact "Traffic Sim Map Builder — Game Design Document"). Its *Implementation plan* tab has the checklists and gates.

## Quick start (macOS)

```sh
git clone --recursive git@github.com:czoka/traffic-sim.git
cd traffic-sim
python3 build.py setup            # once: creates .venv/ and installs SCons there
python3 build.py                  # macOS extension (debug + release) + C++ tests
```

Python packages live only in the project's `.venv/` (pinned in `requirements.txt`); nothing is installed system-wide. You don't need to activate the venv: `build.py` switches to `.venv`'s Python by itself. To run SCons by hand, use `source .venv/bin/activate` first, or call `.venv/bin/scons`.

Then open `game/project.godot` in **Godot 4.7.x** and press Play (F5). The editor opens your autosave, or the demo town the first time.

For the web build you also need Emscripten **4.0.11**, the version Godot 4.7's web templates are built with, plus Godot's export templates (Editor → Manage Export Templates):

```sh
brew install emscripten           # or: emsdk install 4.0.11 && emsdk activate 4.0.11
python3 build.py                  # now also builds the web extension
GODOT=/Applications/Godot.app/Contents/MacOS/Godot python3 build.py export
python3 build.py serve            # http://localhost:8060
```

The web build is single-threaded, so it needs no special server headers (no COOP/COEP).

## The road editor (M1)

The layout is a tool palette on the left, an inspector on the right, and a bottom bar with undo/redo, level, snapping, connectors, status and problems.

| Tool | Key | How |
| --- | --- | --- |
| Select / move | `V` | Click to select, Shift-click to add. Drag nodes, whole roads or Bézier handles (straight roads get handles too, so you can bend them). Drop a node on another node to join them. Delete/Backspace removes the selection. |
| Road | `R` | Click the start, click bends, then double-click or press Enter. Clicking an existing node or road ends the road there and makes a junction. Backspace removes the last point; Esc cancels. |
| Curve road | `C` | Click the start, the control point, then the end. Starting at the end of a road keeps the curve tangent to it; hold Alt to bend freely. |
| Lane paint | `L` | Drag along the line between two lanes to paint a no-change zone (solid line); drag over a zone to erase it. Alt paints one side only (Alt+Shift for the other side). Click inside a lane to change its type. |

Snapping order: an existing node, then a point on a road (which splits it into a junction), then 15° angles and the 1 m grid (toggle with `G` / `A`). While drawing, the status bar shows the length and angle of the current leg.

What the editor generates from the map:

* **Cross-sections:** lanes are stacked outward from the median. Every road profile lists its lanes left to right: sidewalk, parking, bike, bus, general and turn lanes, one-way or two-way, with a painted, raised or no median.
* **Junctions:** a junction appears wherever three or more roads meet on the same level. Legs are trimmed back where their kerb lines meet, and the corners are rounded with 8 m curb fillets. Sidewalks wrap around the corners, and stop lines are drawn across the incoming lanes.
* **Connectors and arrows:** connectors come from lane counts and each approach's turn rules (no turn / allowed / turn lane). Lane arrows are painted from the connectors, so the paint always matches what cars may do.
* **Turn pockets:** a turn-lane rule adds a pocket lane for the last N metres with a 15 m taper. Pockets appear only where the turn actually exists.
* **Tapers:** where two roads with different lane counts join end to end, the outer lanes narrow away and get a merge arrow.
* **Lane lines:** they are dashed by default and solid within 30 m of a stop line. Painted zones are solid, or double solid/dashed for one-sided zones.
* **Levels:** roads have a level of −1, 0 or +1, and only roads on the same level connect. The level being edited is drawn normally; other levels are dimmed or see-through.

The **inspector** edits a road's name, speed limit, level, lanes per direction, lane width, median, sidewalks, parking, bike and bus lanes, one-way direction and the turn rules at each junction end. Nine built-in profiles are included, and "Save as…" stores your own in `user://profiles.json`.

The **problems panel** lists errors and warnings. Clicking one jumps to it. Errors are roads that cross on one level without a junction, and lanes with no way out at a junction. Warnings are road ends (cars will need spawn/sink points from M2), turn lanes shorter than 20 m or too long for their road, and roads too short for their junctions.

**Files:** the editor autosaves every 10 s to `user://autosave.json`, which is browser storage on the web. *Export…* and *Open…* use native file dialogs on desktop, and a download or file picker in the browser. *Examples* loads the demo town, the 56-junction test grid, or the POC ring benchmark.

### Other controls

| Input | Action |
| --- | --- |
| Wheel / pinch | Zoom at the cursor |
| Right- or middle-drag / arrows / trackpad | Pan |
| `F` | Fit the map in view |
| Ctrl/Cmd+Z, Ctrl/Cmd+Shift+Z (or Ctrl+Y) | Undo, redo |
| Ctrl/Cmd+S, Ctrl/Cmd+O | Export, open a map file |
| PgUp / PgDn | Edit the level above / below |
| Esc | Cancel the current drawing, then clear the selection |

## Map files

Format `"traffic-sim-map"`, version **2**. The file stores nodes (position, level) and segments: curve (straight, arc or Bézier), level, speed limit, name, profile (lanes with stable IDs), turn rules per end (with pocket lane IDs) and no-change zones. Keys are written in a fixed order and numbers in shortest round-trip form, so save → load → save gives an identical file. Version 1 files (the POC ring) are upgraded on load, and files from a newer version are rejected with a message. Examples are in `game/maps/`.

## M1 gate and measurements

The gate is that **a 50-junction test network draws, edits and saves cleanly; save → load → save gives an identical file; 500 undo steps work.** It is checked by the C++ test `M1 gate` and by `game/tests/editor_smoke_test.gd`:

* **The network:** the test grid (`build_test_grid`, also saved as `game/maps/test_grid_v2.json`) has 52 junctions of three or more legs, plus 4 corners. It includes avenues with bus lanes and turn pockets, one-way pairs, a lane-drop taper and curved Bézier roads, and it reports no errors.
* **Round trip:** save → load → save of the grid is byte-identical.
* **Undo:** 500 random edits (move, split, add, delete, flip, lane counts, turn rules, paint), then 500 undos, restore the exact starting map, and 500 redos restore the exact edited map.

Rebuild cost on the grid (one core of a 2.1 GHz Xeon, debug extension build) is about 7–9 ms of geometry plus about 3 ms to hand the meshes to Godot, per edit or drag step. Validation takes under 0.1 ms.

## POC ring benchmark

*Examples → POC ring benchmark* (or `godot --path game -- --bench`) runs the POC scene: IDM cars on a multi-lane ring road, drawn with one MultiMesh. It still checks the POC gate: **2,000 cars at 60 fps in a desktop browser, and the same seed gives an identical state hash on web and desktop.** In a browser, open the exported page with `?bench`. For each phase it reports fps, p95 frame time, sim milliseconds per frame, microseconds per tick and the speed actually achieved.

| | 1,000 cars | 2,000 cars | 5,000 cars |
| --- | --- | --- | --- |
| Tick, native C++ | 10.7 µs | 21.7 µs | 67.8 µs |
| Tick, wasm in Node (V8) | – | ~49–64 µs | – |
| Sim work per frame at 128x, Godot headless | – | 0.26 ms | 0.85 ms |

These numbers come from the cloud build environment; the gate numbers must come from a desktop browser on real hardware. The golden scenario (2,000 cars, seed 42, 6,000 ticks) hashes to `d3b987055dad39fb` with GCC and Clang on x86-64, GCC on ARM64, in wasm, and inside Godot 4.7.2.

## Repository layout

```
core/                 pure C++17, no Godot includes
  include/tsim/
    road_map.h        editable map: nodes, segments, profiles, turn rules, zones
    document.h        undoable edits (command log) and editing operations
    road_map_json.h   save files, v1 -> v2 migration
    curve.h           straight / arc / Bézier curves, deterministic lengths
    road_geometry.h   cross-sections, junctions, connectors, paint, meshes
    validation.h      problems panel checks
    demo_maps.h       demo town and the M1 gate test grid
    map.h, sim.h      POC runtime lane network and IDM simulation
extension/src/        GDExtension: RoadEditor (editor bridge), TrafficSim (POC)
game/                 Godot 4.7 project (Compatibility renderer)
  editor/             main scene, map view, overlay, tools/, ui/, file I/O
  poc/                ring benchmark scene
  common/             camera controller
  maps/               example maps (v2) and the POC ring (v1)
  tests/              headless smoke tests (editor, POC)
tests/                C++ unit tests (doctest)
tools/                headless sim benchmark (tsim_bench)
third_party/          doctest 2.4.12, nlohmann/json 3.12.0 (MIT), Clipper2 2.0.1 (Boost)
godot-cpp/            submodule, godot-cpp v10 (master @ 507ed9d), api_version 4.7
build.py              one-command build (uses .venv/)
SConstruct            extension, tests and benchmark
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
build/native/tsim_bench                                          # golden scenario: PASS/FAIL
godot --headless --path game --script res://tests/editor_smoke_test.gd
godot --headless --path game --script res://tests/poc_smoke_test.gd
```

CI (`.github/workflows/ci.yml`) builds Linux, Windows (MSVC) and macOS (Apple Silicon runner) plus the web extension. It runs the unit tests on every platform, including in wasm under Node. On Linux it also runs both Godot smoke tests and a quick benchmark.

## Rules the core follows

* **No Godot in `core/`.** Everything is testable headless.
* **The map is the only source of truth.** Lane shapes, junctions, connectors and paint are derived from nodes and segments and never saved. Undo restores nodes and segments, and everything else follows.
* **Determinism.** Inside `Simulation::tick()` only `+ - * /` and `sqrt` are used on doubles. Curve and lane lengths are computed the same way, in closed form or by Gauss–Legendre quadrature, ready for M2's network compile. Builds use `-ffp-contract=off` (MSVC: `/fp:precise`). Iteration order is always by ID, and sorting uses a strict total order.
* **Stable IDs.** Nodes, segments, lanes (including turn pockets) and vehicles get IDs from counters that are saved with the map and never reused, even after undo.
* **Versioned saves.** Every format change bumps the version and adds a migration in `road_map_json.cpp`.
* **Coordinates.** Metres, with y pointing down (same as Godot 2D). Lanes are listed left to right, looking from a segment's from-node to its to-node. A positive arc sweep runs clockwise on screen.

## Known limits

* Cars don't drive on edited maps yet. M2 compiles the editable map into the sim network and adds routing, lane changes and junction control.
* Roads that cross mid-segment don't join automatically; a junction is made by snapping an end onto a road or node. Crossings are flagged in the problems panel.
* The whole map's geometry is rebuilt after each edit (about 10 ms on the 56-junction grid). Incremental rebuilds can come later if bigger maps need them.
* Levels are stored and kept apart, but ramps, bridges and level shadows are M4.
* The web build (including the browser file picker) hasn't been exported and tested yet. The editor has only been run on Linux so far.
