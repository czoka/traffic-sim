extends SceneTree
## Engine-level smoke test. Run headless from the repo root:
##   godot --headless --path game --script res://tests/smoke_test.gd
## Exits with the number of failed checks.

var _failures := 0


func _initialize() -> void:
	_run.call_deferred()


func _check(ok: bool, what: String) -> void:
	if ok:
		print("  ok    ", what)
	else:
		print("  FAIL  ", what)
		_failures += 1


func _run() -> void:
	print("Traffic sim smoke test (Godot %s)" % Engine.get_version_info().string)
	if not ClassDB.class_exists("TrafficSim"):
		print("  FAIL  TrafficSim extension not loaded (build it first)")
		quit(1)
		return
	var main: Main = load("res://main.tscn").instantiate()
	root.add_child(main)
	await process_frame
	await process_frame
	var sim = main.sim

	var g: Dictionary = sim.run_golden_check()
	_check(g.pass, "golden determinism hash %s (expected %s)" % [g.hash, g.expected])

	var st: Dictionary = sim.get_stats()
	_check(st.lanes == 16, "bundled ring map loaded (16 lanes, got %d)" % st.lanes)
	_check(sim.get_vehicle_count() == main.car_count, "%d cars spawned" % main.car_count)

	var segs: int = st.segments
	main.add_road(Vector2(2000, 0), Vector2(2200, 60))
	_check(sim.get_stats().segments == segs + 1, "road tool adds a segment")
	_check(sim.get_road_lines(8.0).size() > 0, "road drawing data available")

	var before: String = sim.save_json()
	main.save_map()
	_check(main.load_map(), "saved map loads again")
	_check(sim.save_json() == before, "save -> load -> save gives an identical file")
	_check(sim.load_json("{broken") != "", "invalid JSON is rejected with a message")
	_check(sim.save_json() == before, "failed load leaves the map untouched")

	sim.spawn_cars(500, 7)
	sim.step(300)
	var h1: String = sim.get_state_hash()
	sim.spawn_cars(500, 7)
	sim.step(300)
	_check(sim.get_state_hash() == h1, "same seed gives the same hash in the engine")
	sim.spawn_cars(500, 8)
	sim.step(300)
	_check(sim.get_state_hash() != h1, "different seed gives a different hash")

	var buf: PackedFloat32Array = sim.get_render_buffer(1.0)
	_check(buf.size() == sim.get_vehicle_count() * 12, "render buffer is 12 floats per car")
	var x_axis := Vector2(buf[0], buf[4])
	var y_axis := Vector2(buf[1], buf[5])
	_check(is_equal_approx(x_axis.length(), 1.0) and absf(x_axis.dot(y_axis)) < 1e-4,
		"car transforms are orthonormal")

	# Time slicing: a tiny budget cannot keep up with 128x, so the backlog is dropped.
	sim.spawn_cars(5000, 1)
	var ran: int = sim.advance(0.1, 128.0, 0.001)
	var s2: Dictionary = sim.get_stats()
	_check(ran >= 1 and ran < 128 and s2.behind, "time budget limits ticks per frame (%d ran)" % ran)
	ran = sim.advance(0.1, 1.0, 8.0)
	_check(ran == 1 and not sim.get_stats().behind, "1x runs one tick per 0.1 s")

	print("%d check(s) failed" % _failures)
	quit(_failures)
