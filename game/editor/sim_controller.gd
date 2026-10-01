class_name SimController
extends Node2D
## Simulation in the editor: play / pause / step, speed, seed, demand, car
## rendering (one MultiMesh per level) and the car inspector's selection.
## Editing the map pauses the sim; pressing Play again recompiles only the
## changed junctions and carries on with the same cars.

signal state_changed

## 16x real time is the default; the multipliers give 4x..128x.
const BASE_SPEED := 16.0
const MULTIPLIERS := [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]
const BUDGET_MS := 8.0
const CAR_LENGTH := 4.5
const CAR_WIDTH := 1.8
## Cars are drawn at least this many pixels long, however far you zoom out.
const MIN_CAR_PIXELS := 6.0
const STATS_INTERVAL := 0.25
## People are drawn as dots, at least this many pixels across.
const PED_RADIUS := 0.35
const MIN_PED_PIXELS := 3.5

var editor: MapEditor
var playing := false
var speed_index := 2
var seed_value := 42
var demand := 1.0
var max_cars := 2000
var max_people := 1000
var selected_car := 0
var selected_ped := 0
var stats := {}
var car_info := {}
var ped_info := {}
var stop_stats: Array = [] # [{id, name, pos, waiting, boarded, ...}] while people ride

var _layers := {} # level -> MultiMeshInstance2D
var _ped_layers := {} # level -> MultiMeshInstance2D
var _edit_revision := -1 # map revision when the sim was last running
var _stats_timer := 0.0


func _ready() -> void:
	for level in [-1, 0, 1]:
		var quad := QuadMesh.new()
		quad.size = Vector2(CAR_LENGTH, CAR_WIDTH)
		# The car's front bumper is its position: shift the quad back.
		quad.center_offset = Vector3(-CAR_LENGTH * 0.5, 0, 0)
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_2D
		mm.use_colors = true
		mm.mesh = quad
		var mmi := MultiMeshInstance2D.new()
		mmi.name = "Cars%d" % (level + 1)
		mmi.multimesh = mm
		# Above the level's paint, below the next level's ground.
		mmi.z_index = (level + 2) * 10 + 5
		add_child(mmi)
		_layers[level] = mmi
		var pm := MultiMesh.new()
		pm.transform_format = MultiMesh.TRANSFORM_2D
		pm.use_colors = true
		pm.mesh = _dot_mesh(PED_RADIUS)
		var pmi := MultiMeshInstance2D.new()
		pmi.name = "People%d" % (level + 1)
		pmi.multimesh = pm
		pmi.z_index = (level + 2) * 10 + 6
		add_child(pmi)
		_ped_layers[level] = pmi


static func _dot_mesh(r: float) -> ArrayMesh:
	var verts := PackedVector2Array([Vector2.ZERO])
	var idx := PackedInt32Array()
	for i in 8:
		verts.append(Vector2.from_angle(TAU * i / 8.0) * r)
	for i in 8:
		idx.append_array([0, 1 + i, 1 + (i + 1) % 8])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_INDEX] = idx
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


func reset() -> void:
	editor.road.sim_set_demand(demand)
	editor.road.sim_set_max_vehicles(max_cars)
	editor.road.sim_set_people({"max_pedestrians": max_people})
	editor.road.sim_reset(seed_value)
	selected_car = 0
	car_info = {}
	selected_ped = 0
	ped_info = {}
	_edit_revision = editor.road.revision()
	_refresh_stats()
	_update_cars()
	state_changed.emit()


func speed() -> float:
	return BASE_SPEED * MULTIPLIERS[speed_index]


func speed_label(i: int) -> String:
	return "%s× (%dx)" % [str(MULTIPLIERS[i]), int(BASE_SPEED * MULTIPLIERS[i])]


## Returns false (and says why) when errors block Play.
func play() -> bool:
	var errors := 0
	for p in editor.road.get_problems():
		if p.severity == "error":
			errors += 1
	if errors > 0:
		editor.notify("Fix %d error%s before playing (see Problems)." % [errors, "" if errors == 1 else "s"])
		return false
	if editor.road.get_spawners().is_empty():
		editor.notify("No spawn points yet: press N and click a road end to add one.")
	var resumed: bool = editor.road.revision() != _edit_revision and int(stats.get("tick", 0)) > 0
	playing = true
	editor.road.sim_advance(0.0, 0.0, BUDGET_MS) # compiles now, so the report below is current
	_edit_revision = editor.road.revision()
	if resumed:
		var st: Dictionary = editor.road.sim_stats()
		editor.notify("Resumed: recompiled %d junction%s in %.1f ms." % [st.recompiled_junctions,
			"" if int(st.recompiled_junctions) == 1 else "s", st.recompile_ms])
	state_changed.emit()
	return true


func pause() -> void:
	playing = false
	state_changed.emit()


func toggle() -> void:
	if playing:
		pause()
	else:
		play()


## One sim second (ten ticks).
func step() -> void:
	if playing:
		pause()
	editor.road.sim_step(10)
	_edit_revision = editor.road.revision()
	_refresh_stats()
	_update_cars()


func set_speed_index(i: int) -> void:
	speed_index = clampi(i, 0, MULTIPLIERS.size() - 1)
	state_changed.emit()


func set_demand(d: float) -> void:
	demand = d
	editor.road.sim_set_demand(d)


func set_max_cars(n: int) -> void:
	max_cars = n
	editor.road.sim_set_max_vehicles(n)


func set_max_people(n: int) -> void:
	max_people = n
	editor.road.sim_set_people({"max_pedestrians": n})


func select_car(id: int) -> void:
	selected_car = id
	car_info = editor.road.sim_car_info(id) if id != 0 else {}
	if id != 0:
		selected_ped = 0
		ped_info = {}
	editor.ui.refresh_inspector()
	editor.overlay.queue_redraw()


func select_ped(id: int) -> void:
	selected_ped = id
	ped_info = editor.road.sim_ped_info(id) if id != 0 else {}
	if id != 0:
		selected_car = 0
		car_info = {}
	editor.ui.refresh_inspector()
	editor.overlay.queue_redraw()


func has_cars() -> bool:
	return int(stats.get("vehicles", 0)) > 0


func has_people() -> bool:
	return int(stats.get("pedestrians", 0)) > 0


func _process(delta: float) -> void:
	if editor == null or editor.road == null:
		return
	if playing:
		if editor.road.revision() != _edit_revision:
			# The map was edited while running: editing pauses the sim.
			pause()
			editor.notify("Paused for editing. Press Play to carry on with the changes.")
		else:
			editor.road.sim_advance(delta, speed(), BUDGET_MS)
	_update_cars()
	_stats_timer += delta
	if _stats_timer >= STATS_INTERVAL:
		_stats_timer = 0.0
		_refresh_stats()
	if selected_car != 0:
		car_info = editor.road.sim_car_info(selected_car)
		if car_info.is_empty():
			selected_car = 0
			editor.ui.refresh_inspector()
		editor.overlay.queue_redraw()
	if selected_ped != 0:
		ped_info = editor.road.sim_ped_info(selected_ped)
		if ped_info.is_empty():
			selected_ped = 0
			editor.ui.refresh_inspector()
		editor.overlay.queue_redraw()
	if playing:
		editor.overlay.queue_redraw() # walk lights


func _refresh_stats() -> void:
	stats = editor.road.sim_stats()
	stop_stats = editor.road.sim_stop_stats() if int(stats.get("trips", 0)) > 0 else []
	editor.ui.refresh_sim(stats)
	if selected_car != 0 or selected_ped != 0:
		editor.ui.inspector.refresh_car()
	editor.ui.inspector.refresh_live()


func _update_cars() -> void:
	var zoom: float = editor.camera.zoom.x if editor.camera else 1.0
	var car_scale := maxf(1.0, MIN_CAR_PIXELS / (CAR_LENGTH * zoom))
	for level in _layers:
		var mm: MultiMesh = _layers[level].multimesh
		var n: int = editor.road.sim_car_count(level)
		if mm.instance_count != n:
			mm.instance_count = n
		if n > 0:
			mm.buffer = editor.road.sim_car_buffer(level, car_scale)
	var ped_scale := maxf(1.0, MIN_PED_PIXELS / (PED_RADIUS * 2.0 * zoom))
	for level in _ped_layers:
		var pm: MultiMesh = _ped_layers[level].multimesh
		var n: int = editor.road.sim_ped_count(level)
		if pm.instance_count != n:
			pm.instance_count = n
		if n > 0:
			pm.buffer = editor.road.sim_ped_buffer(level, ped_scale)


## Same look as the map: the level being edited is drawn normally, others dimmed
## (or hidden with the level filter).
func apply_level_style(current_level: int, filter := false) -> void:
	for layers in [_layers, _ped_layers]:
		for level in layers:
			var mmi: MultiMeshInstance2D = layers[level]
			mmi.visible = not filter or level == current_level
			mmi.modulate = MapView.level_tint(level, current_level)
