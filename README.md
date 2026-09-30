# Traffic Sim

A traffic simulation map builder: a C++ simulation core running inside Godot 4, targeting desktop and the browser.

**Status: M3 (junctions and transit) implemented.** The main scene is a road editor with a traffic simulation. You draw straight and curved roads, which join into generated junctions; each road has a cross-section profile, turn rules per approach and painted no-change lines. You add spawn points at road ends, pick each junction's control (right-hand priority, priority road, all-way stop, fixed-time signals) or turn it into a roundabout, place bus stops, depots and routes, and press Play: cars, taxis, buses, coaches and bikes route across the network, change lanes, give way, stop at red lights, park and queue. Editing pauses the sim; Play resumes with the changes, recompiling only the junctions that changed. Undo/redo is unlimited, and maps save as versioned JSON. The POC's ring-road benchmark is still in the project for tracking sim performance and determinism.

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
| Spawn point | `N` | Click a road end to add a spawn / sink point (300 cars/h in, cars may leave). Click one to edit it in the inspector; Shift-click removes it. |
| Roundabout | `O` | Click a junction to turn it into a roundabout (lanes follow the widest leg); Shift-click turns it back. Radius, ring lanes, turbo layout and slip lanes are in the inspector. |
| Bus stop | `K` | Click beside a road to add a stop on that side (the direction it serves). `1` kerbside, `2` bus bay, `3` main station. Shift-click removes the nearest stop. |
| Bus depot | `D` | Click a road end to add a depot; Shift-click removes it. |
| Bus route | `U` | Click a depot, then its stops in order; Enter (or the depot again) saves the route. `L` toggles loop / end to end, Backspace drops the last stop, Esc cancels. |

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

The **problems panel** lists errors and warnings. Clicking one jumps to it. Errors are roads that cross on one level without a junction, and lanes with no way out at a junction; errors block Play. Warnings are road ends without a spawn point, spawn points that aren't on a road end, spawn points whose cars can't reach some destinations, turn lanes shorter than 20 m or too long for their road, and roads too short for their junctions.

Selecting a **junction** shows its control: right-hand priority (the default: yield to the right), priority road (tick the legs that form the main road; the two straightest legs are picked by default) or all-way stop. The paint follows: a dashed give-way line for right-hand priority, no line on a main road and shark teeth on its side roads, a wide stop line for an all-way stop. Selecting a **spawn point** shows its rate (cars per hour entering the map), whether cars may leave there, and the share of its trips going to each destination (the origin-destination matrix; uniform by default).

## Junctions and transit (M3)

**Roundabouts.** A roundabout replaces a junction: a ring of 1–3 circulating lanes (radius 12–40 m) with an island, give-way teeth at the entries and one piece of ring per leg. Traffic circulates counter-clockwise and has priority; entries give way. Lane choice is spiral: the right entry lane feeds the outer ring lane and exits are taken from it, cars going further round use the inner lanes (the outer lane's through connectors cost 3 s in routing). A *turbo* roundabout has raised dividers: the lane is chosen at the entry and there are no lane changes inside. A *slip lane* lets a leg's right turns bypass the ring. An entry is only granted while every ring lane keeps a free car-sized slot, so the ring can't lock itself up; a car stuck behind a full piece of the ring leaves at the next exit and re-routes from there.

**Signals.** *Traffic signal* in the junction inspector starts from a default fixed-time plan: opposite legs share a phase (25 s green, left turns permissive), other legs get a phase each (15 s). The phase editor sets each phase's green time and every movement's light (red, green, or green but yield), amber (3 s by default), all-red (2 s), the cycle offset, and which legs have the EU flashing green arrow (right turn on red after a full stop, yielding to everyone). A timeline shows the cycle and where the running sim is in it; signal heads on the map show each approach's light. In the sim, red means no grant; on amber a car goes only if it can no longer stop comfortably (or to clear a waiting permissive left turn); permissive movements yield to protected greens; a grant is taken back when the light changes and the car can still stop.

**Lane classes.** Bike lanes are for bikes only. Cars may enter a bus lane only within 60 m of the stop line to turn off it, and move out of it otherwise (the time cars spend in bus lanes outside that zone is counted as misuse). Taxis use bus lanes freely; buses and coaches prefer them. Parking lanes come in three styles, parallel, 45° and 90°, with bays drawn along the kerb (no parking within 10 m of a junction, marked with a yellow kerb line).

**Vehicles.** A share of car trips (5 %) are taxis and a quarter of car trips park on the way: they pick a free bay on a road their route takes, pull in (parallel 8 s, angled 5 s, perpendicular 7 s, blocking the lane meanwhile), stay 5–30 minutes, wait for a gap and pull out (6–8 s). Bikes enter at spawn points at their own rate, ride on bike lanes (then bus lanes, then the kerb lane) at 16–23 km/h, with their own connectors through junctions. Vehicles are drawn by kind: cars by speed, taxis yellow, buses in their route's colour, coaches purple, bikes teal, parked cars grey. The car inspector shows a bus's route, next stop and stops left, and whether a car will park.

**Transit.** Stops are kerbside (the bus stops in the lane), bus bays (it pulls into a lay-by) or the main station (a lay-by with several bays; one per map, for coaches). A depot sits on a road end and has a capacity; each route lists its stops in order, a headway and loop or end to end. Every headway a bus leaves the depot (while the depot has buses), serves the stops in order (a loop comes back to the first), dwells 20 s at each and returns to the depot. The inspector shows each route's stops, round-trip time at free flow and the fleet it needs (round trip ÷ headway), with the buses out now and the runs finished. Coach lines start at a spawn point: at their frequency a coach enters, dwells at the main station and leaves the map at the line's exit. Routes are drawn on the map along the roads the buses take.

The problems panel also lists stops with no lane for their direction, depots that aren't on a road end, routes that can't reach a stop or get back to the depot, coach lines without an exit, and coach lines on a map without a main station.

## The simulation (M2)

The bar above the bottom bar runs the sim: **Play/Pause** (`Space`), **Step** one sim second (`.`), **Restart** (remove all cars and start again with the seed), **Speed** (16x real time by default, multipliers 0.25–8 for 4x–128x), **Seed**, **Density** (multiplies every spawn rate) and **Max cars** (spawning pauses at that many cars). The status shows the sim clock, cars, trips, mean speed and stopped cars; its tooltip has the tick cost. Cars are coloured by speed, red when stopped to green at their desired speed. Click a car to see its state (driving, queued, yielding, waiting for the junction to clear, exit full, all-way stop…), speed, origin and destination, trip time and the car it waits for; its route is drawn on the map.

How it works:

* **Network compile.** The map and its geometry compile into a lane graph: every travel lane (turn pockets included) and every connector through a node is a sim lane with a polyline. Same-direction neighbours are linked, with the stretches where changing lanes is allowed: not within 30 m of a stop line, not across painted no-change zones (per direction), and into or out of a pocket only along its taper. Each junction gets a conflict matrix: which connector pairs cross, merge or pass closer than 1.6 m, where along each path, and who has priority there. The compiler keeps a signature of each segment's and junction's inputs and compiles only the ones that changed; the result is identical to a full compile.
* **Routing.** A* on the lane graph: connectors and lane changes are edges, costs are observed travel times per lane (free-flow time at first), with small penalties for turns and lane changes and bus lanes counted three times slower. Cars re-route every 5 sim minutes with the latest times, and immediately when a lane change they needed can no longer be made.
* **Driving.** IDM car following, looking ahead along the route through connectors, slowing for tight turns (curvature sets each connector's speed). MOBIL lane changes with European keep-right bias, never across solid lines; mandatory changes toward the lane the route needs start 100 m per lane change ahead, and the car behind in the target lane leaves a gap for a car that is stuck waiting to merge.
* **Junctions.** A car may enter a junction only with a grant. It gets one when no car on a conflicting connector is still in the way (the box is exclusive per conflict point), its exit has room for the whole car (don't block the box), and the rules let it go: it yields to cars with priority (from the right, on the main road, or oncoming when it turns left) unless the gap is at least its critical gap (4–6 s, per driver). At an all-way stop every car stops at the line and cars go in arrival order. A driver who has waited 30 s is let in at the next safe moment, and if everyone waits for someone and the box is empty, the longest waiter goes (deadlock breaker).
* **Demand.** Spawn points release cars as a Poisson process at their rate × density, towards destinations drawn from the origin-destination weights (only reachable ones). Cars queue at the map edge when the entry lane is full, and leave the map at their destination's road end.
* **Editing while paused.** Vehicles, routes and grants are carried over to the recompiled network by stable lane IDs; cars on lanes that no longer exist are removed and every other car re-routes.

**Files:** the editor autosaves every 10 s to `user://autosave.json`, which is browser storage on the web. *Export…* and *Open…* use native file dialogs on desktop, and a download or file picker in the browser. *Examples* loads the demo town, the 56-junction test grid, the small M2 maps, the M3 showcase, or the POC ring benchmark.

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

Format `"traffic-sim-map"`, version **4**. The file stores nodes (position, level, junction control with main-road segments, signal plan, roundabout, depot with its routes, spawn point with rate, sink flag, origin-destination weights, bike rate and coach lines) and segments: curve (straight, arc or Bézier), level, speed limit, name, profile (lanes with stable IDs and parking style), turn rules per end (with pocket lane IDs), no-change zones and bus stops. Keys are written in a fixed order and numbers in shortest round-trip form, so save → load → save gives an identical file. Version 2 and 3 files load unchanged with the defaults for what they lack; version 1 files (the POC ring) are upgraded on load; files from a newer version are rejected with a message. Examples are in `game/maps/` (`*_v4.json` are written by `tsim_bench --write-maps game/maps`).

## M1 gate and measurements

The gate is that **a 50-junction test network draws, edits and saves cleanly; save → load → save gives an identical file; 500 undo steps work.** It is checked by the C++ test `M1 gate` and by `game/tests/editor_smoke_test.gd`:

* **The network:** the test grid (`build_test_grid`, also saved as `game/maps/test_grid_v2.json`) has 52 junctions of three or more legs, plus 4 corners. It includes avenues with bus lanes and turn pockets, one-way pairs, a lane-drop taper and curved Bézier roads, and it reports no errors.
* **Round trip:** save → load → save of the grid is byte-identical.
* **Undo:** 500 random edits (move, split, add, delete, flip, lane counts, turn rules, paint), then 500 undos, restore the exact starting map, and 500 redos restore the exact edited map.

Rebuild cost on the grid (one core of a 2.1 GHz Xeon, debug extension build) is about 7–9 ms of geometry plus about 3 ms to hand the meshes to Godot, per edit or drag step. Validation takes under 0.1 ms.

## M2 gate and measurements

The gate is that **500 cars run 1 sim hour on the test grid with no gridlock and no deadlocked junctions.** It is checked by the C++ test `M2 gate` and by `game/tests/sim_smoke_test.gd`:

* **The network:** the M1 test grid, now with roads out of the grid on every side to 30 spawn points (one-way streets only feed traffic in or out in their own direction). The avenues are priority roads, a few street junctions are all-way stops, the rest keep the right-hand rule. It has 949 sim lanes and 57 controlled junctions.
* **The run:** density ×2 with a cap of 500 cars, so the grid holds 500 cars all hour (a congested but moving regime: 8–15 km/h mean speed and about 5,500 trips per hour).
* **Checks:** no car is taken off the map as stuck; no car stands still for 10 minutes (the longest stop is about 5 minutes, at the busiest entries); no junction goes 90 s without letting a car in while cars wait there whose exit has room (the longest is about 10 s); trips per 5 minutes never fall below half of the first 5 minutes; and at every sample no two cars overlap in a lane and no two cars sit on the conflict point of two crossing connectors. The test uses seed 2026; seeds 1–6, run with `tsim_bench --traffic`, stay within the same stop and junction limits.

Other M2 tests cover the conflict matrix, the right-hand rule, priority roads and gap acceptance, all-way stops, four cars arriving together (the deadlock breaker), don't block the box, MOBIL overtaking and painted lines, mandatory changes into a turn pocket, the lane drop, origin-destination weights and unreachable pairs, editing while paused, incremental versus full compile, and the small test maps (T junction, lane drop, one-way pair, demo town).

Cost on one core of a 2.1 GHz Xeon: about 65 µs per tick with 500 cars on the grid (about 0.8 ms of sim work per frame at 128x), about 90 µs with 800 cars. A full network compile of the grid takes about 6 ms; after a local edit, 1–5 junctions are compiled again in about 2–3 ms.

The golden scenario (the grid, seed 42, density ×2, 500-car cap, 6,000 ticks) hashes to `2b4a55beb24226a0` (M3: parking lanes are now 2 m deep, cars park and some trips are taxis; M2 recorded `a4ef13f8041c564c`). CI checks it with GCC and Clang on x86-64, MSVC on Windows, Clang on Apple Silicon, in wasm, and inside Godot 4.7.2. `tsim_bench` checks it with no arguments; `tsim_bench --traffic [--cars CAP] [--minutes M] [--seed S] [--demand D]` prints a per-minute table.

## M3 gate and measurements

The gate is that **a showcase map using every M3 feature passes validation and runs a full sim day.** It is checked by the C++ tests `showcase map` and `M3 gate` and by `game/tests/sim_smoke_test.gd`:

* **The map** (`build_showcase`, *Examples → Showcase*, `game/maps/showcase_v4.json`): an avenue with bus lanes and parking into a signalized junction (default plan, a left-turn pocket, right on red from the south) and on to a two-lane roundabout with a slip lane; a bike street; roads with parallel, 45° and 90° parking; kerbside stops, two bus bays and a three-bay main station; a depot with a loop route (5 min headway) and an end-to-end route (8 min); a coach line (3 an hour, 5 min at the station); cars in and out at six edges and bikes at two. It has no errors, warnings or network problems.
* **The run:** 24 sim hours with seed 2026. About 41,600 trips (1,600–1,800 an hour), 466 bus runs serving 1,800 stops, 72 coach calls, 5,500 bike trips, 5,700 parkings and 1,100 right turns on red.
* **Checks:** no vehicle taken off the map as stuck, none stopped for 10 minutes (the longest stop is under 4 minutes), no junction stuck by itself for 90 s, trips per hour never below half of the first full hour, no overlaps in any lane, and the bus, coach, bike and parking counts above their floors.

Other M3 tests cover signal timing in ticks, red lights, right on red (full stop first; straight on waits), the default plan, one- and two-lane roundabouts (ring priority, throughput), bike lanes and bike routing, cars keeping out of bus lanes, parking (one car per bay, cars drawn in their bay), a bus following its route stop by stop back to the depot, coaches, taxis and bikes on the showcase, moving stops while buses run, the transit problems, and a golden hash of 15 showcase minutes (`69e85f1801f703bf`, checked on every CI platform).

Cost: the showcase (100–250 vehicles) runs a sim day in about 12 s on one core of a 2.1 GHz Xeon, about 14 µs per tick.

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
    network.h         sim network compile: lanes, connectors, conflict matrices (M2)
    traffic.h         traffic sim: routing, IDM, MOBIL, junction grants, signals,
                      vehicle kinds, parking, buses and coaches, demand (M2, M3)
    traffic_run.h     keeps network and sim in step with the map; M2 golden scenario
    demo_maps.h       demo town, the gate test grid, the M2 test maps, the M3 showcase
    map.h, sim.h      POC runtime lane network and IDM simulation
extension/src/        GDExtension: RoadEditor (editor + sim bridge), TrafficSim (POC)
game/                 Godot 4.7 project (Compatibility renderer)
  editor/             main scene, map view, sim controller, overlay, tools/, ui/, file I/O
  poc/                ring benchmark scene
  common/             camera controller
  maps/               example maps (v4, a few v2) and the POC ring (v1)
  tests/              headless smoke tests (editor, sim, POC)
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
godot --headless --path game --script res://tests/sim_smoke_test.gd
godot --headless --path game --script res://tests/poc_smoke_test.gd
```

CI (`.github/workflows/ci.yml`) builds Linux, Windows (MSVC) and macOS (Apple Silicon runner) plus the web extension. It runs the unit tests (with both golden hashes) on every platform, including in wasm under Node. On Linux it also runs the three Godot smoke tests and a quick benchmark. (Godot 4.7.2 aborts on exit after a headless `--import`; CI tolerates that exit code, and the smoke tests fail if the import really went wrong.)

## Rules the core follows

* **No Godot in `core/`.** Everything is testable headless.
* **The map is the only source of truth.** Lane shapes, junctions, connectors and paint are derived from nodes and segments and never saved. Undo restores nodes and segments, and everything else follows.
* **Determinism.** Inside `Simulation::tick()` and `Traffic::tick()` only `+ - * /` and `sqrt` are used on doubles. Geometry uses trig, so the network compile snaps everything the sim reads to a 1/1024 m grid and derives lengths from the snapped points; the compiled network is then bit-identical on every platform. Builds use `-ffp-contract=off` (MSVC: `/fp:precise`). Iteration order is always by ID, sorting and the A* queue use strict total orders, and random numbers come from the seeded RNG in a fixed order.
* **Stable IDs.** Nodes, segments, lanes (including turn pockets) and vehicles get IDs from counters that are saved with the map and never reused, even after undo.
* **Versioned saves.** Every format change bumps the version and adds a migration in `road_map_json.cpp`.
* **Coordinates.** Metres, with y pointing down (same as Godot 2D). Lanes are listed left to right, looking from a segment's from-node to its to-node. A positive arc sweep runs clockwise on screen.

## Known limits

* Signals are fixed-time only: no actuated or coordinated plans yet, and no pedestrians (so no pedestrian phases).
* Cars may use a bus lane within 60 m of the stop line (the GDD says 30 m; 60 m leaves room to merge in at speed). Bikes ride in the kerb lane where there is no bike lane, and cars can't overtake them in a single-lane road.
* Passengers don't exist yet: buses dwell a fixed 20 s and coaches their line's time; parking trips pick a bay on their way rather than near a destination (both come with M4's people).
* Route lines on the map are the free-flow routes; running buses re-route with traffic.
* A lane change is instant in the sim and drawn as a 3 s sideways slide. There are no U-turns, and cars never turn around at a dead end: routes only lead to spawn points.
* The sim is single-threaded. At 128x a 2,000-car map may be CPU-limited; the status bar then shows the speed actually reached.
* Roads that cross mid-segment don't join automatically; a junction is made by snapping an end onto a road or node. Crossings are flagged in the problems panel.
* The whole map's geometry is rebuilt after each edit (about 10 ms on the 56-junction grid). Incremental rebuilds can come later if bigger maps need them.
* Levels are stored and kept apart, but ramps, bridges and level shadows are M4.
* The web build (including the browser file picker) hasn't been exported and tested yet. The editor has only been run on Linux so far (headless and under Xvfb).
