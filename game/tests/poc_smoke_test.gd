extends SceneTree
## POC ring benchmark smoke test. Run headless from the repo root:
##   godot --headless --path game --script res://tests/poc_smoke_test.gd
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
	print("POC ring smoke test (Godot %s)" % Engine.get_version_info().string)
	if not ClassDB.class_exists("TrafficSim"):
		print("  FAIL  TrafficSim extension not loaded (build it first)")
		quit(1)
		return
	var demo: RingDemo = load("res://poc/ring_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame
	await process_frame
	var sim = demo.sim

	var g: Dictionary = sim.run_golden_check()
	_check(g.pass, "golden determinism hash %s (expected %s)" % [g.hash, g.expected])
	var st: Dictionary = sim.get_stats()
	_check(st.lanes == 16, "ring has 16 lanes (got %d)" % st.lanes)
	_check(sim.get_vehicle_count() == demo.car_count, "%d cars spawned" % demo.car_count)

	sim.spawn_cars(500, 7)
	sim.step(300)
	var h1: String = sim.get_state_hash()
	sim.spawn_cars(500, 7)
	sim.step(300)
	_check(sim.get_state_hash() == h1, "same seed gives the same hash in the engine")

	var buf: PackedFloat32Array = sim.get_render_buffer(1.0)
	_check(buf.size() == sim.get_vehicle_count() * 12, "render buffer is 12 floats per car")
	var x_axis := Vector2(buf[0], buf[4])
	var y_axis := Vector2(buf[1], buf[5])
	_check(is_equal_approx(x_axis.length(), 1.0) and absf(x_axis.dot(y_axis)) < 1e-4,
		"car transforms are orthonormal")

	sim.spawn_cars(5000, 1)
	var ran: int = sim.advance(0.1, 128.0, 0.001)
	_check(ran >= 1 and ran < 128 and sim.get_stats().behind, "time budget limits ticks per frame (%d ran)" % ran)
	ran = sim.advance(0.1, 1.0, 8.0)
	_check(ran == 1 and not sim.get_stats().behind, "1x runs one tick per 0.1 s")

	print("%d check(s) failed" % _failures)
	quit(_failures)
