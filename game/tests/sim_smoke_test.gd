extends SceneTree
## Simulation smoke test (M2, M3, M4): loads the real editor scene and drives the
## sim through the calls the sim bar, the tools, the inspector and car picking
## make.
## Run headless:
##   godot --headless --path game --script res://tests/sim_smoke_test.gd

var _failures := 0


func _initialize() -> void:
	_run.call_deferred()


func _check(ok: bool, what: String) -> void:
	print(("  ok    " if ok else "  FAIL  ") + what)
	if not ok:
		_failures += 1


func _frames(n: int) -> void:
	for i in n:
		await process_frame


func _first_car(road) -> Dictionary:
	for id in range(1, 20000):
		var info: Dictionary = road.sim_car_info(id)
		if not info.is_empty() and info.route.size() >= 2:
			return info
	return {}


func _run() -> void:
	print("Simulation smoke test (Godot %s)" % Engine.get_version_info().string)
	if not ClassDB.class_exists("RoadEditor"):
		print("  FAIL  RoadEditor extension not loaded (build it first)")
		quit(1)
		return
	var ed: MapEditor = load("res://main.tscn").instantiate()
	root.add_child(ed)
	await _frames(3)
	var road = ed.road
	ed.camera.zoom = Vector2.ONE * 2.0

	# Determinism: the M2 golden scenario inside Godot.
	var golden: Dictionary = road.sim_golden_check()
	_check(golden.pass, "M2 golden hash %s (expected %s, %.0f ms)" % [golden.hash, golden.expected, golden.ms])

	# The gate network: cars spawn, drive and arrive.
	ed.load_demo("grid")
	await _frames(2)
	var st: Dictionary = road.get_stats()
	_check(st.errors == 0 and road.get_spawners().size() == 30, "test grid: %d spawn points, %d errors" % [road.get_spawners().size(), st.errors])
	_check(ed.sim.play(), "Play starts the sim")
	await _frames(5)
	_check(ed.sim.playing, "sim is running")
	ed.sim.pause()
	road.sim_step(1800) # 3 sim minutes
	var s1: Dictionary = road.sim_stats()
	_check(s1.vehicles > 100 and s1.arrived > 50, "after 3 min: %d cars, %d trips, %.0f km/h" % [s1.vehicles, s1.arrived, s1.mean_speed_kmh])
	_check(s1.removed_stuck == 0, "no car got stuck")
	var n0: int = road.sim_car_count(0)
	var buf: PackedFloat32Array = road.sim_car_buffer(0, 1.0)
	_check(n0 == s1.vehicles and buf.size() == n0 * 12, "render buffer: 12 floats for each of %d cars" % n0)
	await _frames(2)
	var mm: MultiMesh = ed.sim._layers[0].multimesh
	_check(mm.instance_count == n0, "car MultiMesh has %d instances" % mm.instance_count)

	# Click a car: its route and state.
	var car := _first_car(road)
	_check(not car.is_empty() and car.route.size() >= 2, "car %s has a route of %d points" % [car.get("id", 0), car.get("route", PackedVector2Array()).size()])
	var picked: int = road.sim_pick_car(car.pos - (car.dir as Vector2) * 2.0, 3.0, 0)
	_check(picked != 0, "clicking a car picks it (car %d)" % picked)
	ed.sim.select_car(picked)
	await _frames(2)
	_check(ed.ui.inspector._car_box.visible and ed.ui.inspector._car_info.text.contains("km/h"), "inspector shows the car")
	ed.sim.select_car(0)

	# Same seed, same run.
	ed.sim.seed_value = 7
	ed.sim.reset()
	road.sim_step(600)
	var h1: String = road.sim_state_hash()
	ed.sim.reset()
	road.sim_step(600)
	_check(road.sim_state_hash() == h1, "same seed gives the same state hash (%s)" % h1)

	# Edit while paused, then resume: only the changed part is recompiled.
	var before: int = road.sim_stats().vehicles
	var node_id: int = road.node_ids()[20]
	road.move_node(node_id, road.get_node(node_id).pos + Vector2(3, 2))
	road.sim_advance(0.1, 16.0, 8.0)
	var s2: Dictionary = road.sim_stats()
	_check(s2.recompiled_junctions >= 1 and s2.recompiled_junctions <= 5, "resume recompiled %d junctions in %.1f ms" % [s2.recompiled_junctions, s2.recompile_ms])
	_check(s2.vehicles >= before - 5, "cars carried over the edit (%d -> %d)" % [before, s2.vehicles])

	# Editing while playing pauses the sim.
	ed.sim.play()
	await _frames(2)
	road.move_node(node_id, road.get_node(node_id).pos + Vector2(-3, -2))
	await _frames(2)
	_check(not ed.sim.playing, "an edit pauses the sim")

	# Spawner tool and the node inspector on the T junction.
	ed.load_demo("t_junction")
	await _frames(2)
	var ends: Array = []
	for id in road.node_ids():
		if road.get_node(id).road_end:
			ends.append(id)
	_check(ends.size() == 3 and road.get_spawners().size() == 3, "T junction: 3 road ends with spawn points")
	road.set_spawner(ends[0], {"enabled": false})
	_check(road.get_spawners().size() == 2, "spawn point removed")
	ed.set_tool("spawner")
	var click := InputEventMouseButton.new()
	click.button_index = MOUSE_BUTTON_LEFT
	click.pressed = true
	click.position = Vector2.ZERO
	ed.camera.position = road.get_node(ends[0]).pos
	await _frames(1)
	var tool: SpawnerTool = ed.tools["spawner"]
	var hit: int = tool._road_end_at(road.get_node(ends[0]).pos)
	_check(hit == ends[0], "spawner tool finds the road end")
	road.set_spawner(hit, {"enabled": true, "rate": 250.0, "sink": true})
	_check(road.get_node(hit).spawner.enabled and road.get_spawners().size() == 3, "spawn point added again")
	ed.set_tool("select")

	var centre := 0
	for id in road.node_ids():
		if road.get_node(id).junction:
			centre = id
	ed.select("nodes", centre, false)
	await _frames(1)
	var insp: Inspector = ed.ui.inspector
	_check(insp._control_box.visible, "inspector shows junction control")
	insp._control.select(2)
	insp._set_control()
	_check(road.get_node(centre).control == "all_way_stop", "junction set to all-way stop")
	insp._control.select(1)
	road.set_junction_control(centre, "right_hand", PackedInt64Array())
	insp._fill_node(centre)
	insp._control.select(1)
	insp._set_control()
	var n: Dictionary = road.get_node(centre)
	_check(n.control == "priority_road" and n.priority.size() == 2, "priority road picks the straight pair of legs by default")
	ed.undo()
	_check(road.get_node(centre).control == "right_hand", "undo restores the junction control")

	# Save -> load keeps spawn points and controls (map format v4).
	var text: String = road.save_json()
	var r: Dictionary = road.load_json(text)
	_check(r.ok and road.save_json() == text and text.contains("\"version\": 5"), "v5 save -> load -> save is identical")

	await _m3(ed, road)
	await _m4(ed, road)

	# Errors block Play.
	road.new_map()
	road.add_road([{"pos": Vector2(0, 0)}, {"pos": Vector2(100, 0)}], {"preset": "Street 1+1"}, 0, 50.0)
	road.add_road([{"pos": Vector2(50, -50)}, {"pos": Vector2(50, 50)}], {"preset": "Street 1+1"}, 0, 50.0)
	await _frames(1)
	_check(not ed.sim.play() and not ed.sim.playing, "a map with errors doesn't play")

	print("%d check(s) failed" % _failures)
	quit(_failures)


## M3: the showcase (signals, roundabout, buses, coaches, bikes, parking) and
## the new tools and inspector sections.
func _m3(ed: MapEditor, road) -> void:
	ed.load_demo("showcase")
	await _frames(2)
	var st: Dictionary = road.get_stats()
	_check(st.errors == 0, "showcase loads without errors")
	var stops: Array = road.get_stops()
	var depots: Array = road.get_depots()
	_check(stops.size() == 6 and depots.size() == 1 and depots[0].routes.size() == 2, "showcase: %d stops, %d depot with %d routes" % [stops.size(), depots.size(), depots[0].routes.size() if depots.size() > 0 else 0])
	_check(road.get_bays(0).size() > 20, "parking bays: %d" % road.get_bays(0).size())
	var heads: Array = road.sim_signal_heads(0)
	_check(heads.size() >= 4, "signal heads before playing: %d" % heads.size())
	ed.sim.reset()
	road.sim_step(6000) # 10 sim minutes
	var s: Dictionary = road.sim_stats()
	_check(int(s.buses) >= 1 and int(s.bikes) >= 1 and int(s.parked) >= 1, "after 10 min: %d buses, %d bikes, %d parked, %d taxis" % [s.buses, s.bikes, s.parked, s.taxis])
	_check(int(s.bus_stops_served) >= 1 and int(s.removed_stuck) == 0, "%d stops served, nobody stuck" % s.bus_stops_served)
	var lights := {}
	for h in road.sim_signal_heads(0):
		lights[String(h.light)] = true
	_check(lights.has("red") and (lights.has("green") or lights.has("yield") or lights.has("amber")), "signal heads show %s" % str(lights.keys()))
	var sig_node := 0
	var ring_node := 0
	for id in road.node_ids():
		var n: Dictionary = road.get_node(id)
		if n.control == "signal":
			sig_node = id
		if n.roundabout.enabled:
			ring_node = id
	var ss: Dictionary = road.sim_signal_state(sig_node)
	_check(float(ss.get("cycle", 0.0)) > 20.0 and ss.phases.size() == 2, "signal cycle %.0f s in %d phases" % [ss.get("cycle", 0.0), ss.get("phases", []).size()])
	var rs: Array = road.sim_route_stats()
	_check(rs.size() == 2 and int(rs[0].fleet) >= 1, "route stats: fleet %d, round trip %.0f s" % [rs[0].fleet if rs.size() > 0 else 0, rs[0].round_trip if rs.size() > 0 else 0.0])
	var bus := {}
	for id in range(1, 5000):
		var info: Dictionary = road.sim_car_info(id)
		if not info.is_empty() and info.kind == "bus":
			bus = info
			break
	_check(not bus.is_empty() and String(bus.route_name) != "", "a bus knows its route (%s, next stop %s)" % [bus.get("route_name", ""), bus.get("next_stop", "")])
	var buf: PackedFloat32Array = road.sim_car_buffer(0, 1.0)
	_check(buf.size() == road.sim_car_count(0) * 12, "render buffer covers every vehicle kind")

	# Inspector: the signal plan editor.
	ed.select("nodes", sig_node, false)
	await _frames(1)
	var insp: Inspector = ed.ui.inspector
	_check(insp._signal_box.visible and insp._phases_box.get_child_count() >= 4, "inspector shows the phase editor")
	insp._plan.phases[0]["green"] = 33.0
	insp._send_plan()
	_check(is_equal_approx(float(road.get_node(sig_node).signal.phases[0].green), 33.0), "phase green time edited")
	ed.undo()
	_check(not is_equal_approx(float(road.get_node(sig_node).signal.phases[0].green), 33.0), "undo restores the plan")
	# Roundabout section.
	ed.select("nodes", ring_node, false)
	await _frames(1)
	_check(insp._ring_box.visible and insp._ring_on.button_pressed, "inspector shows the roundabout")
	insp._ring_radius.value = 30.0
	_check(is_equal_approx(float(road.get_node(ring_node).roundabout.radius), 30.0), "roundabout radius edited")
	ed.undo()
	# Depot section.
	ed.select("nodes", int(depots[0].node), false)
	await _frames(1)
	_check(insp._depot_box.visible and insp._routes_box.get_child_count() > 0, "inspector shows the depot and its routes")
	ed.clear_selection()

	# Tools: stop, depot, route and roundabout on the T junction.
	ed.load_demo("t_junction")
	await _frames(2)
	var centre := 0
	var ends: Array = []
	for id in road.node_ids():
		var n: Dictionary = road.get_node(id)
		if n.junction:
			centre = id
		if n.road_end:
			ends.append(id)
	var rt: RoundaboutTool = ed.tools["roundabout"]
	_check(rt._junction_at(road.get_node(centre).pos) == centre, "roundabout tool finds the junction")
	road.set_roundabout(centre, {"enabled": true, "radius": 20.0, "lanes": 1})
	await _frames(1)
	_check(road.get_node(centre).roundabout.enabled and int(road.get_stats().errors) == 0, "junction turned into a roundabout")
	var seg: int = road.get_node(ends[0]).segments[0]
	# Every arm ends at the map edge: buses reach a stop on the outbound side.
	var outbound := "forward" if int(road.get_segment(seg).to) == int(ends[0]) else "backward"
	var inbound := "backward" if outbound == "forward" else "forward"
	var a: int = road.add_stop(seg, 0.5, outbound, "kerbside", "A")
	var b: int = road.add_stop(seg, 0.5, inbound, "bay", "B")
	_check(a != 0 and b != 0 and road.get_segment(seg).stops.size() == 2, "two stops added")
	var dt: DepotTool = ed.tools["depot"]
	_check(dt._end_at(road.get_node(ends[1]).pos) == ends[1], "depot tool finds a road end")
	road.set_depot(ends[1], {"enabled": true, "name": "Test depot", "capacity": 5, "routes": [
		{"id": 0, "name": "T1", "color": 0xd83f3f, "stops": [a], "headway": 120.0, "loop": false}]})
	var dp: Dictionary = road.get_node(ends[1]).depot
	_check(dp.enabled and dp.routes.size() == 1 and int(dp.routes[0].id) != 0, "depot with a route (id %d)" % dp.routes[0].id)
	ed.sim.reset()
	road.sim_step(3000)
	var s2: Dictionary = road.sim_stats()
	_check(int(s2.bus_stops_served) >= 1, "buses serve the new stops (%d served, %d buses, %d unroutable, %d bus runs)" % [s2.bus_stops_served, s2.buses, s2.unroutable, s2.bus_runs])
	var text: String = road.save_json()
	var r: Dictionary = road.load_json(text)
	_check(r.ok and road.save_json() == text, "M3 objects survive save -> load -> save")


## M4: the people town (crossings, bridge, overpass, passengers), the path,
## crossing, fence and bridge tools, the inspector's M4 sections, people
## rendering and picking, and the level filter.
func _m4(ed: MapEditor, road) -> void:
	ed.load_demo("people")
	await _frames(2)
	_check(int(road.get_stats().errors) == 0, "people town loads without errors")
	var crossings: Array = road.get_crossings(0)
	var kinds := {}
	for c in crossings:
		kinds[String(c.kind)] = int(kinds.get(String(c.kind), 0)) + 1
	_check(kinds.get("signal", 0) >= 4 and kinds.get("zebra", 0) >= 4 and kinds.get("uncontrolled", 0) >= 1, "painted crossings: %s" % str(kinds))
	_check(road.get_crossings(1).size() == 0 and road.get_meshes().any(func(m): return int(m.level) == 1), "bridge and overpass on level 1")
	_check(ed.view._shadows.has(1), "level 1 casts a shadow")
	ed.sim.reset()
	road.sim_step(6000) # 10 sim minutes
	var s: Dictionary = road.sim_stats()
	_check(int(s.trips) > 100 and int(s.pedestrians) > 20, "after 10 min: %d people trips, %d on foot, %d riding" % [s.trips, s.pedestrians, s.riding])
	_check(int(s.crossings) > 10 and int(s.cars_yielded) > 0, "%d crossings on foot, cars gave way %d times" % [s.crossings, s.cars_yielded])
	_check(int(s.boarded) > 0, "%d boarded, %d got off" % [s.boarded, s.alighted])
	var n: int = road.sim_ped_count(0)
	var buf: PackedFloat32Array = road.sim_ped_buffer(0, 1.0)
	_check(n > 0 and buf.size() == n * 12, "people render buffer: %d on level 0, %d on level 1" % [n, road.sim_ped_count(1)])
	var lights := {}
	for k in 12:
		for w in road.sim_walk_lights(0):
			lights[String(w.walk)] = true
		road.sim_step(50)
	_check(lights.size() >= 2, "walk lights show %s" % str(lights.keys()))
	var stats: Array = road.sim_stop_stats()
	_check(stats.size() == 4, "stop stats for %d stops" % stats.size())
	_check(road.sim_route_loads().size() == 1, "route loads for the town loop")
	# Pick someone and inspect them.
	var picked := 0
	var at := Vector2.ZERO
	for i in n:
		var p := Vector2(buf[i * 12 + 3], buf[i * 12 + 7])
		picked = road.sim_pick_ped(p, 0.5, 0)
		if picked != 0:
			at = p
			break
	_check(picked != 0, "picked person %d at %s" % [picked, str(at)])
	ed.sim.select_ped(picked)
	await _frames(1)
	var info: Dictionary = ed.sim.ped_info
	_check(not info.is_empty() and ed.ui.inspector._car_box.visible, "person inspector: %s" % String(info.get("state", "")))
	ed.sim.select_ped(0)
	# Level filter.
	ed.set_level_filter(true)
	var hidden := 0
	for key in ed.view._instances:
		hidden += 0 if ed.view._instances[key].visible else 1
	_check(hidden > 0, "level filter hides %d mesh batches" % hidden)
	ed.set_level_filter(false)

	# Tools on a fresh map: a street, a footpath, a crossing, a fence and a bridge.
	road.new_map()
	var street: PackedInt64Array = road.add_road([{"pos": Vector2(-200, 0)}, {"pos": Vector2(200, 0)}], {"preset": "Street 1+1"}, 0, 50.0)
	var path: PackedInt64Array = road.add_road([{"pos": Vector2(-100, 30)}, {"pos": Vector2(-100, 120)}], {"kind": "footpath"}, 0, 5.0)
	_check(street.size() == 1 and path.size() == 1 and road.get_segment(path[0]).kind == "footpath", "path tool template draws a footpath")
	var ct: CrosswalkTool = ed.tools["crosswalk"]
	ed.camera.zoom = Vector2.ONE * 2.0 # tools pick within a few screen pixels
	var t: Dictionary = ct._target(Vector2(0, 1))
	_check(not t.is_empty() and int(t.end) == -1, "crosswalk tool targets mid-block")
	var cid: int = road.add_crossing(t.segment, t.u, "zebra", true, false)
	_check(cid != 0 and road.get_segment(street[0]).crossings.size() == 1, "mid-block zebra with a bike crossing")
	road.set_fence(street[0], 0, 0.6, 0.9, true)
	_check(road.get_segment(street[0]).fences.size() == 1, "fence put up")
	await _frames(1)
	ed.select("segments", street[0], false)
	await _frames(1)
	var insp: Inspector = ed.ui.inspector
	_check(insp._crossings_box.visible and insp._crossings_list.get_child_count() == 1, "inspector lists the crossing")
	var cross_road: PackedInt64Array = road.add_road([{"pos": Vector2(60, -250)}, {"pos": Vector2(60, 250)}], {"preset": "Street 1+1"}, 0, 50.0)
	var plan: Dictionary = road.plan_lift(cross_road[0], Vector2(60, 0), 1)
	_check(plan.ok and plan.points.size() == 4, "bridge preview: ramps from %.0f to %.0f m" % [plan.stations[0] if plan.ok else 0.0, plan.stations[3] if plan.ok else 0.0])
	_check(road.lift(cross_road[0], Vector2(60, 0), 1) == "" and int(road.get_stats().errors) == 0, "bridge built, no errors")
	var ramps := 0
	for id in road.segment_ids():
		var sg: Dictionary = road.get_segment(id)
		if int(sg.rise) != 0:
			ramps += 1
			_check(float(sg.grade) <= float(sg.max_grade) + 1e-6, "ramp grade %.1f %% within %.0f %%" % [float(sg.grade) * 100.0, float(sg.max_grade) * 100.0])
	_check(ramps == 2, "two ramps")
	ed.undo()
	_check(int(road.get_stats().errors) > 0, "undo takes the bridge down")
	ed.redo()
	var text: String = road.save_json()
	var r: Dictionary = road.load_json(text)
	_check(r.ok and road.save_json() == text, "M4 objects survive save -> load -> save")
	ed.clear_selection()
