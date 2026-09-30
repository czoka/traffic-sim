class_name RingDemo
extends Node2D
## POC benchmark scene: a multi-lane ring road with IDM cars, drawn with one
## MultiMesh. Kept to track sim performance and determinism as the project grows.

const RING_RADIUS := 1500.0
const RING_LANES := 4
const LANE_WIDTH := 3.5
const SPEED_LIMIT_KMH := 60.0
const CAR_LENGTH := 4.5
const CAR_WIDTH := 1.8
## Cars are drawn at least this many pixels long, however far you zoom out.
const MIN_CAR_PIXELS := 5.0
const FRAME_WINDOW := 120

## TrafficSim (GDExtension). Untyped so a missing library gives a readable
## error instead of a script parse failure.
var sim = null
var car_count: int = 2000
var seed_value: int = 42
var speed: float = 1.0
var paused: bool = false
## Max milliseconds of simulation work per frame. Extra ticks are dropped.
var budget_ms: float = 8.0

var roads: RingRoadLayer
var cars: MultiMeshInstance2D
var camera: CameraController
var hud: RingHud
var bench: RingBenchmark

var _frame_ms := PackedFloat32Array()
var _frame_idx := 0


func _ready() -> void:
	_frame_ms.resize(FRAME_WINDOW)
	if not ClassDB.class_exists("TrafficSim"):
		_show_fatal("The TrafficSim extension is not loaded.\nBuild it first:  python build.py")
		return
	sim = ClassDB.instantiate("TrafficSim")
	_build_scene()
	new_ring()
	respawn()
	camera.fit(sim.get_map_bounds())
	_apply_view_arg()
	_autorun_benchmark_if_requested()


func _build_scene() -> void:
	roads = RingRoadLayer.new()
	roads.name = "Roads"
	add_child(roads)

	var quad := QuadMesh.new()
	quad.size = Vector2(CAR_LENGTH, CAR_WIDTH)
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_2D
	mm.use_colors = true
	mm.mesh = quad
	cars = MultiMeshInstance2D.new()
	cars.name = "Cars"
	cars.multimesh = mm
	add_child(cars)

	camera = CameraController.new()
	camera.name = "Camera"
	add_child(camera)

	hud = RingHud.new()
	hud.name = "Hud"
	hud.main = self
	add_child(hud)

	bench = RingBenchmark.new()
	bench.name = "Benchmark"
	bench.main = self
	add_child(bench)


func _process(delta: float) -> void:
	_frame_ms[_frame_idx % FRAME_WINDOW] = delta * 1000.0
	_frame_idx += 1
	if sim == null:
		return
	if not paused:
		sim.advance(delta, speed, budget_ms)
	_update_cars()


func _update_cars() -> void:
	var n: int = sim.get_vehicle_count()
	var mm := cars.multimesh
	if mm.instance_count != n:
		mm.instance_count = n
	if n == 0:
		return
	var car_scale := maxf(1.0, MIN_CAR_PIXELS / (CAR_LENGTH * camera.zoom.x))
	mm.buffer = sim.get_render_buffer(car_scale)


# --- Actions (called from the HUD, shortcuts and the benchmark) --------------

func new_ring() -> void:
	sim.new_ring(RING_RADIUS, RING_LANES, LANE_WIDTH, SPEED_LIMIT_KMH)
	roads.rebuild(sim)


func respawn() -> void:
	var placed: int = sim.spawn_cars(car_count, seed_value)
	if placed < car_count:
		notify("Only %d of %d cars fit on the roads." % [placed, car_count])
	_update_cars()


func set_car_count(count: int) -> void:
	car_count = count
	respawn()


func set_speed(multiplier: float) -> void:
	speed = multiplier
	paused = false


func toggle_pause() -> void:
	paused = not paused


func step_once() -> void:
	paused = true
	sim.step(1)


func run_determinism_check() -> Dictionary:
	var r: Dictionary = sim.run_golden_check()
	notify("Determinism %s: %d cars x %d ticks -> %s (expected %s), %.0f ms" % [
		"PASS" if r.pass else "FAIL", r.cars, r.ticks, r.hash, r.expected, r.ms])
	return r


func notify(text: String) -> void:
	print(text)
	if hud:
		hud.show_message(text)


# --- Frame statistics --------------------------------------------------------

func frame_stats() -> Dictionary:
	var count := mini(_frame_idx, FRAME_WINDOW)
	if count == 0:
		return {"avg_ms": 0.0, "p95_ms": 0.0, "max_ms": 0.0, "fps": 0.0}
	var samples := _frame_ms.slice(0, count)
	samples.sort()
	var total := 0.0
	for v in samples:
		total += v
	var avg := total / count
	return {
		"avg_ms": avg,
		"p95_ms": samples[mini(count - 1, int(count * 0.95))],
		"max_ms": samples[count - 1],
		"fps": 1000.0 / avg if avg > 0.0 else 0.0,
	}


# --- Input -------------------------------------------------------------------

func _unhandled_key_input(event: InputEvent) -> void:
	var k := event as InputEventKey
	if k == null or not k.pressed or k.echo or sim == null:
		return
	var handled := true
	match k.keycode:
		KEY_SPACE:
			toggle_pause()
		KEY_PERIOD:
			step_once()
		KEY_1:
			set_speed(1.0)
		KEY_2:
			set_speed(4.0)
		KEY_3:
			set_speed(16.0)
		KEY_4:
			set_speed(64.0)
		KEY_5:
			set_speed(128.0)
		KEY_F:
			camera.fit(sim.get_map_bounds())
		KEY_ESCAPE:
			get_tree().change_scene_to_file("res://main.tscn")
		_:
			handled = false
	if handled:
		get_viewport().set_input_as_handled()
		hud.refresh_controls()


func _autorun_benchmark_if_requested() -> void:
	var args := OS.get_cmdline_user_args()
	var quick := args.has("--bench-quick")
	var run := quick or args.has("--bench")
	if OS.has_feature("web"):
		var query = JavaScriptBridge.eval("window.location.search", true)
		if typeof(query) == TYPE_STRING and (query as String).contains("bench"):
			run = true
	if run:
		bench.start(quick, not OS.has_feature("web"))


## `-- --view=x,y,zoom` starts the camera at a given spot and `-- --speed=16`
## at a given sim speed (handy for screenshots and demos).
func _apply_view_arg() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--speed="):
			set_speed(arg.trim_prefix("--speed=").to_float())
			hud.refresh_controls()
		elif arg.begins_with("--view="):
			var v := arg.trim_prefix("--view=").split_floats(",")
			if v.size() == 3:
				camera.position = Vector2(v[0], v[1])
				camera.zoom = Vector2.ONE * v[2]


func _show_fatal(text: String) -> void:
	push_error(text)
	var layer := CanvasLayer.new()
	var label := Label.new()
	label.text = text
	label.position = Vector2(40, 40)
	label.add_theme_font_size_override("font_size", 22)
	layer.add_child(label)
	add_child(layer)
