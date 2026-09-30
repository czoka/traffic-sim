extends SceneTree
## Simulation smoke test (M2): loads the real editor scene and drives the sim
## through the calls the sim bar, spawner tool, inspector and car picking make.
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
		if not info.is_empty():
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

	# Save -> load keeps spawn points and controls (map format v3).
	var text: String = road.save_json()
	var r: Dictionary = road.load_json(text)
	_check(r.ok and road.save_json() == text and text.contains("\"version\": 3"), "v3 save -> load -> save is identical")

	# Errors block Play.
	road.new_map()
	road.add_road([{"pos": Vector2(0, 0)}, {"pos": Vector2(100, 0)}], {"preset": "Street 1+1"}, 0, 50.0)
	road.add_road([{"pos": Vector2(50, -50)}, {"pos": Vector2(50, 50)}], {"preset": "Street 1+1"}, 0, 50.0)
	await _frames(1)
	_check(not ed.sim.play() and not ed.sim.playing, "a map with errors doesn't play")

	print("%d check(s) failed" % _failures)
	quit(_failures)
