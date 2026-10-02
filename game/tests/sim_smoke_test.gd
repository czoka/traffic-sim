extends SceneTree
## Simulation smoke test (M2, M3, M4, M5): loads the real editor scene and drives the
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
	# Cars and people must never be culled by stale MultiMesh bounds (zoomed in, they vanished).
	var fixed_bounds := true
	for layers in [ed.sim._layers, ed.sim._ped_layers]:
		for level in layers:
			fixed_bounds = fixed_bounds and (layers[level] as MultiMeshInstance2D).multimesh.custom_aabb.size.x >= 1e5
	_check(fixed_bounds, "car and people layers have fixed bounds, so zooming in never culls them")

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
	_check(r.ok and road.save_json() == text and text.contains("\"version\": 7"), "v7 save -> load -> save is identical")

	await _m3(ed, road)
	await _m4(ed, road)
	await _m5(ed, road)
	await _m6(ed, road)
	await _m7(ed, road)

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
	# One state at a time (#19): a signal junction shows the roundabout checkbox only, not the ring's settings.
	_check(not insp._ring_details.visible and insp._ring_box.visible and not insp._ring_on.button_pressed,
		"signal junction: no roundabout settings beside the signal plan")
	insp._plan.phases[0]["green"] = 33.0
	insp._send_plan()
	_check(is_equal_approx(float(road.get_node(sig_node).signal.phases[0].green), 33.0), "phase green time edited")
	ed.undo()
	_check(not is_equal_approx(float(road.get_node(sig_node).signal.phases[0].green), 33.0), "undo restores the plan")
	# Roundabout section.
	ed.select("nodes", ring_node, false)
	await _frames(1)
	_check(insp._ring_box.visible and insp._ring_on.button_pressed and insp._ring_details.visible and not insp._signal_box.visible,
		"inspector shows the roundabout (its settings, no signal plan)")
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
	if int(s.boarded) == 0:
		road.sim_step(6000) # the first buses can be late in traffic
		s = road.sim_stats()
	_check(int(s.boarded) > 0, "%d boarded, %d got off (by %d min)" % [s.boarded, s.alighted, int(s.sim_time) / 60])
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


## M5: buildings and city life - the new city, the city town with residents,
## the building tool and inspector, resident details, the clock and night.
func _m5(ed: MapEditor, road) -> void:
	ed.new_map()
	await _frames(2)
	var blds: Array = road.get_buildings(0)
	_check(blds.size() == 1 and blds[0].type == "city_offices" and int(road.get_stats().errors) == 0, "a new city has the city offices and no errors")
	_check(road.get_stops().any(func(st): return st.kind == "main_station"), "a new city has the main station")
	ed.load_demo("city_town")
	await _frames(2)
	_check(int(road.get_stats().errors) == 0 and road.get_buildings(0).size() > 15, "city town: %d buildings, no errors" % road.get_buildings(0).size())
	road.sim_set_city({"prefill": 1.0})
	ed.sim.reset()
	road.sim_step(36000 * 11) # 06:00 -> 17:00 (36,000 ticks an hour)
	var c: Dictionary = road.sim_city_stats()
	_check(c.on and int(c.residents) > 50, "%d residents, %d households, %d at work, %d asleep" % [c.residents, c.households, c.working, c.sleeping])
	_check(int(c.shifts) > 0 and int(c.home_meals) > 0, "%d shifts, %d meals at home, %d out" % [c.shifts, c.home_meals, c.meals_out])
	ed.sim._refresh_stats()
	var states: Array = ed.sim.building_states
	var open := 0
	for b in states:
		open += 1 if b.kind != "home" and b.open else 0
	_check(open >= 2, "%d businesses open at %s" % [open, SimController.clock_text(ed.sim.stats.clock_day, ed.sim.stats.clock_minute)])
	# The building inspector.
	var grocery := 0
	for b in road.get_buildings(0):
		if b.type == "grocery":
			grocery = b.id
	ed.select("buildings", grocery, false)
	await _frames(1)
	var insp: Inspector = ed.ui.inspector
	_check(insp._building_box.visible and insp._bld_live.text.contains("Staff in"), "the inspector shows the grocery live")
	_check(road.pick_building(road.get_building(grocery).centre, 0) == grocery, "clicking a lot picks its building")
	# Someone walking: their resident details.
	var who := 0
	var buf: PackedFloat32Array = road.sim_ped_buffer(0, 1.0)
	for i in road.sim_ped_count(0):
		var id: int = road.sim_pick_ped(Vector2(buf[i * 12 + 3], buf[i * 12 + 7]), 0.5, 0)
		if id != 0 and int(road.sim_ped_info(id).get("resident", 0)) != 0:
			who = id
			break
	if who != 0:
		var r: Dictionary = road.sim_resident_info(road.sim_ped_info(who).resident)
		_check(not r.is_empty() and String(r.activity) != "", "a walking resident: %s" % r.activity)
	else:
		_check(true, "nobody walking right now (fine)")
	# The building tool: a snap beside the cross street and a placement.
	ed.camera.zoom = Vector2.ONE * 2.0
	var bt: BuildingTool = ed.tools["building"]
	ed.set_tool("building")
	var snap: Dictionary = road.snap_building("townhouse", Vector2(135, 160), 0)
	_check(snap.ok and not snap.blocked, "the building tool finds a free lot")
	var before: int = road.get_buildings(0).size()
	var id2: int = road.add_building("townhouse", snap.pos, snap.dir, 0)
	_check(id2 != 0 and road.get_buildings(0).size() == before + 1, "a townhouse placed")
	var blocked: Dictionary = road.snap_building("apartment_block", Vector2(165, -60), 0)
	_check(blocked.ok and blocked.blocked, "a lot on another building is blocked")
	ed.undo()
	_check(road.get_buildings(0).size() == before, "undo takes it away")
	# The select tool drags a building to another lot along the street (#15).
	var st: SelectTool = ed.tools["select"]
	ed.set_tool("select")
	var moved_id := 0
	for b in road.get_buildings(0):
		if b.type == "townhouse":
			moved_id = b.id
			break
	var start: Vector2 = road.get_building(moved_id).centre
	var free_spot: Dictionary = road.snap_building("townhouse", Vector2(135, 160), 0, moved_id)
	_check(free_spot.ok and not free_spot.blocked, "a free lot for the townhouse")
	st._start_building_move(moved_id, start)
	st._moved = true
	st._drag_update(Vector2(135, 160) - st._bld_offset)
	var release := InputEventMouseButton.new()
	release.button_index = MOUSE_BUTTON_LEFT
	release.pressed = false
	st.input(release)
	var after: Vector2 = road.get_building(moved_id).centre
	_check(after.distance_to(start) > 20.0 and int(road.get_stats().errors) == 0, "the townhouse moved %.0f m along the street, no errors" % after.distance_to(start))
	_check(road.undo_label() == "Move building", "the move is one undo step (%s)" % road.undo_label())
	ed.undo()
	_check(road.get_building(moved_id).centre.distance_to(start) < 0.01, "undo puts it back")
	# Onto another building: it stays where it last fitted.
	var other := 0
	for b in road.get_buildings(0):
		if b.type == "grocery":
			other = b.id
	st._start_building_move(moved_id, start)
	st._moved = true
	st._drag_update(road.get_building(other).centre)
	st.input(release)
	_check(road.get_building(moved_id).centre.distance_to(start) < 0.01 or int(road.get_stats().errors) == 0, "a building never lands on another one")
	ed.set_tool("select")
	void_ok(bt)
	# Night: the map gets darker.
	road.sim_step(36000 * 5) # to 22:00
	ed.sim._refresh_stats()
	_check(ed.sim.daylight.color.r < 0.8, "night falls (%s)" % SimController.clock_text(ed.sim.stats.clock_day, ed.sim.stats.clock_minute))
	var text: String = road.save_json()
	var r2: Dictionary = road.load_json(text)
	_check(r2.ok and road.save_json() == text, "buildings survive save -> load -> save")
	ed.clear_selection()


## M6: money and ownership - the market city, the economy inspector, the city
## centre tool, the market panel and the finance stats.
func _m6(ed: MapEditor, road) -> void:
	ed.load_demo("city_market")
	await _frames(2)
	_check(int(road.get_stats().errors) == 0 and road.get_city_centre().placed, "city market: %d buildings, centre placed, no errors" % road.get_buildings(0).size())
	ed.sim.reset()
	road.sim_step(36000 * 26) # 06:00 day 1 -> 08:00 day 2
	var c: Dictionary = road.sim_city_stats()
	_check(c.on and int(c.residents) > 500, "%d residents, %d households" % [c.residents, c.households])
	_check(float(c.month_income) > 0.0 and int(c.in_debt) == 0, "city income %.0f, spending %.0f this month, %d in debt" % [c.month_income, c.month_spending, c.in_debt])
	_check(int(c.passes) > 0 and int(c.trips_bus) + int(c.trips_walk) > 0, "%d bus passes, %d walk / %d bus / %d bike trips" % [c.passes, c.trips_walk, c.trips_bus, c.trips_bike])
	_check(int(c.listed) > 0, "%d buildings on the market" % c.listed)
	ed.sim._refresh_stats()
	ed.ui.refresh_sim(ed.sim.stats)
	_check(ed.ui._sim_label.tooltip_text.contains("city income"), "the sim tooltip has the finances")
	# A listed city building in the inspector, and its settings.
	var listed := 0
	var home := 0
	for b in road.get_buildings(0):
		var full: Dictionary = road.get_building(b.id)
		if full.for_sale and listed == 0:
			listed = b.id
		if full.kind == "home" and not full.for_sale and home == 0:
			home = b.id
	ed.select("buildings", home, false)
	await _frames(1)
	var insp: Inspector = ed.ui.inspector
	_check(insp._econ_live.text.contains("Owned by") and insp._econ_rent_row.visible, "the inspector shows the owner: %s" % insp._econ_live.text.get_slice("\n", 0))
	insp._econ_rent.value = 900.0
	_check(is_equal_approx(float(road.get_building(home).rent), 900.0), "rent set from the inspector")
	ed.undo()
	_check(float(road.get_building(home).rent) == 0.0, "undo restores the default rent")
	# The market panel lists the sales; buying an NPC listing if one is up.
	ed.ui._market_panel.visible = true
	ed.ui._refresh_market()
	_check(ed.ui._market.size() == int(c.listed), "the market panel lists %d sale%s" % [ed.ui._market.size(), "" if ed.ui._market.size() == 1 else "s"])
	var npc := 0
	for l in road.sim_market():
		if not l.by_city:
			npc = int(l.id)
	if npc != 0:
		_check(road.sim_buy_building(npc) and String(road.sim_building_info(npc).owner_kind) == "city", "bought an NPC listing")
	else:
		_check(listed != 0, "no NPC listings yet (fine); city building %d is for sale" % listed)
	ed.ui._market_panel.visible = false
	# The city centre tool: click to move it, Shift-click removes it, undo.
	ed.set_tool("centre")
	var press := InputEventMouseButton.new()
	press.button_index = MOUSE_BUTTON_LEFT
	press.pressed = true
	(ed.tools["centre"] as CentreTool).input(press)
	_check(road.get_city_centre().placed, "the centre tool places the centre")
	press.shift_pressed = true
	(ed.tools["centre"] as CentreTool).input(press)
	_check(not road.get_city_centre().placed, "Shift-click removes it")
	ed.undo()
	_check(road.get_city_centre().placed, "undo puts it back")
	ed.set_tool("select")
	var text: String = road.save_json()
	var r: Dictionary = road.load_json(text)
	_check(r.ok and road.save_json() == text, "economy settings survive save -> load -> save")
	ed.clear_selection()


## M7: the tutorial as a new player would follow it (the gate: a working town
## with a shop, homes and a bus route), heatmaps, junction stats, the profile
## library and the new warnings.
func _m7(ed: MapEditor, road) -> void:
	var t0 := Time.get_ticks_msec()
	ed.start_tutorial()
	await _frames(2)
	var guide: Guide = ed.ui.guide
	_check(guide.visible and guide.active and int(road.get_stats().errors) == 0, "the tutorial starts on a map with no errors")
	var loop_bottom := 0
	var high := 0
	var depot_lane := 0
	for id in road.segment_ids():
		var sg: Dictionary = road.get_segment(id)
		if sg.name == "High Street" and absf(road.get_node(sg.from).pos.x + 100.0) < 1.0:
			high = id # between the two ends of Loop Road
		elif sg.name == "Depot Lane":
			depot_lane = id
		elif sg.name == "Loop Road" and absf(road.segment_point(id, 0.5).y - 220.0) < 1.0:
			loop_bottom = id
	_check(high != 0 and loop_bottom != 0 and depot_lane != 0, "High Street, Loop Road and Depot Lane")
	# 1. A street off Loop Road.
	road.add_road([{"pos": Vector2(-100, 120), "segment": _seg_at(road, Vector2(-100, 120))}, {"pos": Vector2(-200, 120)}], {"preset": "Street 1+1"}, 0, 50.0)
	# 2-3. Three homes and a grocery beside Loop Road.
	var placed := 0
	for spot in [Vector2(-85, 60), Vector2(-85, 100), Vector2(-85, 160), Vector2(-60, 205)]:
		var kind := "grocery" if placed == 3 else "townhouse"
		var snap: Dictionary = road.snap_building(kind, spot, 0)
		if snap.ok and not snap.blocked and road.add_building(kind, snap.pos, snap.dir, 0) != 0:
			placed += 1
	_check(placed == 4, "3 homes and a shop placed (%d)" % placed)
	# 4. Two stops: Loop Road's bottom and High Street.
	var a: int = road.add_stop(loop_bottom, 0.5, "forward", "kerbside", "Loop Road")
	var b: int = road.add_stop(high, 0.3, "backward", "kerbside", "High Street")
	# 5-6. The depot at Depot Lane's end and a route through both stops.
	var end_node: int = road.get_segment(depot_lane).to
	road.set_depot(end_node, {"enabled": true, "name": "Depot", "capacity": 4, "routes": [
		{"id": 0, "name": "1", "color": 0x2f7fd8, "stops": [a, b], "headway": 300.0, "loop": true}]})
	await _frames(2)
	guide.refresh()
	for st in ["street", "homes", "shop", "stops", "depot", "route"]:
		_check(guide.done.get(st, false), "tutorial step done: %s" % st)
	var errors := 0
	for pr in road.get_problems():
		if pr.severity == "error":
			errors += 1
			print("    error: ", pr.message)
	_check(errors == 0, "no errors before Play")
	# 7-9. Play: people move in and ride the bus.
	_check(ed.sim.play(), "Play is allowed")
	var riders := false
	for h in 12:
		road.sim_step(36000)
		ed.sim._refresh_stats()
		guide.refresh()
		if guide.done.get("bus", false) and guide.done.get("residents", false):
			break
	var c: Dictionary = road.sim_city_stats()
	print("    tutorial town after %s: %d residents, %d households, %d bus trips, %d stops served, %d shifts" % [SimController.clock_text(ed.sim.stats.clock_day, ed.sim.stats.clock_minute), c.residents, c.households, c.trips_bus, ed.sim.stats.bus_stops_served, c.shifts])
	_check(guide._title.text == "Tutorial: done", "the tutorial is done")
	for st in ["play", "residents", "bus"]:
		_check(guide.done.get(st, false), "tutorial step done: %s" % st)
	ed.sim.pause()
	print("    the tutorial run took %d ms" % (Time.get_ticks_msec() - t0))
	# Heatmaps and the junction inspector.
	ed.ui.set_heatmap("speed")
	await _frames(2)
	var heat: PackedFloat32Array = road.sim_lane_heat("speed")
	var seen := 0
	for v in heat:
		if v >= 0.0:
			seen += 1
	_check(ed.heat.visible and seen > 0 and ed.heat._lines.size() > 0, "speed heatmap: %d lanes with traffic" % seen)
	ed.ui.set_heatmap("flow")
	ed.ui.cycle_heatmap()
	_check(ed.heat.mode == "off" and not ed.heat.visible, "M cycles the heatmaps back to off")
	var junction := int(road.get_segment(loop_bottom).from)
	var js: Dictionary = road.sim_junction_stats(junction)
	_check(js.found and int(js.approaches) > 0, "junction stats: %d approaches, %.0f an hour, %d through" % [js.approaches, js.flow, js.passed])
	# The profile library: import a file of profiles, delete one.
	var before: int = ed.user_presets.size()
	var prof: Dictionary = road.get_segment(high).profile
	var n: int = ed.import_user_presets_text(JSON.stringify({"format": "traffic-sim-profiles", "version": 1, "profiles": [
		{"name": "smoke import", "profile": prof}, {"name": "broken", "profile": {"lanes": []}}]}))
	_check(n == 1 and ed.user_presets.size() == before + 1, "profiles import (the broken one is skipped)")
	ed.delete_user_preset("smoke import")
	_check(ed.user_presets.size() == before, "a saved profile deleted")
	_check(road.presets().size() >= 15, "%d built-in profiles" % road.presets().size())
	# New warnings: a small roundabout with four legs.
	ed.load_demo("grid")
	await _frames(1)
	var centre := 0
	for id in road.node_ids():
		if road.get_node(id).segments.size() == 4:
			centre = id
			break
	road.set_roundabout(centre, {"enabled": true, "radius": 12.0, "lanes": 1})
	var small := false
	for pr in road.get_problems():
		small = small or pr.code == "roundabout_small"
	_check(small, "a 12 m roundabout with four legs gets a warning")
	ed.clear_selection()
	ed.ui.guide.stop()


func _seg_at(road, pos: Vector2) -> int:
	var p: Dictionary = road.pick(pos, 3.0, 0)
	return int(p.id) if p.type == "segment" else 0


func void_ok(_x) -> void:
	pass
