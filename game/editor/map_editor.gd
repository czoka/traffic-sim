class_name MapEditor
extends Node2D
## Road editor and simulation (main scene). Owns the C++ RoadEditor, the tools,
## selection, snapping, autosave and file import/export. The simulation clock
## and car rendering live in SimController, the UI in EditorUI.

const AUTOSAVE_PATH := "user://autosave.json"
const PRESETS_PATH := "user://profiles.json"
const AUTOSAVE_INTERVAL := 10.0
const PICK_PIXELS := 10.0
const RING_SCENE := "res://poc/ring_demo.tscn"

## RoadEditor (GDExtension). Untyped so a missing library gives a readable error.
var road = null
var level := 0
var snap_grid := true
var snap_angle := true
var show_connectors := false
## Draw only the level being edited (H).
var level_filter := false
var selection := {"nodes": [], "segments": [], "buildings": []}
## What the road and curve tools draw: {preset: name} or {profile: dict}.
var road_template := {"preset": "Street 1+1"}
var road_template_name := "Street 1+1"
var speed_kmh := 50.0
var user_presets: Array = [] # [{name, profile}]

var view: MapView
var sim: SimController
var heat: HeatLayer
var overlay: EditorOverlay
var camera: CameraController
var ui: EditorUI
var file_io: FileIO
var tools := {}
var tool: EditorTool

var _last_revision := -1
var _last_history := Vector2i(-1, -1) # undo and redo counts shown in the UI
var _saved_revision := -1
var _autosave_timer := 0.0
var _template_width := 10.0
var _screenshot_path := ""
var _perf_log := false
var _perf_timer := 0.0
var _perf_frames := 0
var _screenshot_frames := 0


func _ready() -> void:
	if _wants_ring_benchmark():
		get_tree().change_scene_to_file.call_deferred(RING_SCENE)
		return
	if not ClassDB.class_exists("RoadEditor"):
		_show_fatal("The traffic_sim extension is not loaded.\nBuild it first:  python3 build.py")
		return
	road = ClassDB.instantiate("RoadEditor")
	view = MapView.new()
	view.name = "MapView"
	add_child(view)
	heat = HeatLayer.new()
	heat.name = "Heat"
	heat.editor = self
	heat.visible = false
	add_child(heat)
	sim = SimController.new()
	sim.name = "Sim"
	sim.editor = self
	add_child(sim)
	overlay = EditorOverlay.new()
	overlay.name = "Overlay"
	overlay.editor = self
	add_child(overlay)
	camera = CameraController.new()
	camera.name = "Camera"
	add_child(camera)
	file_io = FileIO.new()
	file_io.name = "FileIO"
	add_child(file_io)
	tools = {
		"select": SelectTool.new(self),
		"road": RoadDrawTool.new(self),
		"curve": CurveDrawTool.new(self),
		"lane": LanePaintTool.new(self),
		"spawner": SpawnerTool.new(self),
		"roundabout": RoundaboutTool.new(self),
		"stop": StopTool.new(self),
		"depot": DepotTool.new(self),
		"route": RouteTool.new(self),
		"path": PathDrawTool.new(self),
		"crosswalk": CrosswalkTool.new(self),
		"fence": FenceTool.new(self),
		"bridge": BridgeTool.new(self),
		"building": BuildingTool.new(self),
		"centre": CentreTool.new(self),
	}
	_load_user_presets()
	ui = EditorUI.new()
	ui.name = "UI"
	ui.editor = self
	add_child(ui)
	camera.ui_hit = ui.over_ui

	_load_startup_map()
	set_template({"preset": "Street 1+1"}, "Street 1+1")
	set_tool("select")
	_refresh_map()
	sim.reset()
	fit_view()
	_apply_cmdline()


## Command-line user args; in the browser also the page's query string, so
## index.html?example=showcase&view=-222,-25,20 works like --example=... --view=...
static func user_args() -> PackedStringArray:
	var args := OS.get_cmdline_user_args()
	if OS.has_feature("web"):
		var query = JavaScriptBridge.eval("window.location.search", true)
		if typeof(query) == TYPE_STRING:
			for part in (query as String).trim_prefix("?").split("&", false):
				args.append("--" + part.uri_decode())
	return args


func _wants_ring_benchmark() -> bool:
	if user_args().has("--bench") or user_args().has("--bench-quick"):
		return true
	if OS.has_feature("web"):
		var query = JavaScriptBridge.eval("window.location.search", true)
		return typeof(query) == TYPE_STRING and (query as String).contains("bench")
	return false


func _load_startup_map() -> void:
	var args := user_args()
	if args.has("--grid"):
		road.load_example("grid")
		return
	for arg in args:
		if arg.begins_with("--example="):
			var which := arg.trim_prefix("--example=")
			road.load_example(which)
			sim.city_prefill = 1.0 if which == "city_town" else 0.95 if which in ["city_week", "city_market"] else 0.0
			return
	if args.has("--demo"):
		road.load_demo_town()
		return
	if args.has("--new"):
		road.new_map()
		return
	if FileAccess.file_exists(AUTOSAVE_PATH):
		var r: Dictionary = road.load_json(FileAccess.get_file_as_string(AUTOSAVE_PATH))
		if r.ok:
			return
		push_warning("Autosave could not be loaded: %s" % r.error)
	# First run: a new city, and the welcome (tutorial or explore).
	road.new_city()
	if DisplayServer.get_name() != "headless" and args.is_empty():
		ui.show_welcome.call_deferred()


func _apply_cmdline() -> void:
	for arg in user_args():
		if arg.begins_with("--view="):
			var v := arg.trim_prefix("--view=").split_floats(",")
			if v.size() == 3:
				camera.position = Vector2(v[0], v[1])
				camera.zoom = Vector2.ONE * v[2]
		elif arg.begins_with("--tool="):
			set_tool(arg.trim_prefix("--tool="))
		elif arg.begins_with("--select-segment="):
			select("segments", int(arg.trim_prefix("--select-segment=")), false)
		elif arg.begins_with("--select-node="):
			select("nodes", int(arg.trim_prefix("--select-node=")), false)
		elif arg == "--connectors":
			show_connectors = true
		elif arg == "--play":
			sim.play.call_deferred()
		elif arg.begins_with("--sim-seconds="):
			sim.reset()
			for i in int(float(arg.trim_prefix("--sim-seconds=")) * 10.0 / 50.0):
				road.sim_step(50)
			sim._refresh_stats()
		elif arg.begins_with("--select-ped="):
			sim.select_ped.call_deferred(int(arg.trim_prefix("--select-ped=")))
		elif arg == "--welcome":
			ui.show_welcome.call_deferred()
		elif arg == "--tutorial":
			start_tutorial.call_deferred()
		elif arg.begins_with("--speed="):
			sim.set_speed_index(int(arg.trim_prefix("--speed=")))
		elif arg == "--perf-log":
			_perf_log = true
		elif arg.begins_with("--heatmap="):
			ui.set_heatmap.call_deferred(arg.trim_prefix("--heatmap="))
		elif arg == "--level-filter":
			set_level_filter(true)
		elif arg.begins_with("--level="):
			set_level(int(arg.trim_prefix("--level=")))
		elif arg.begins_with("--select-car="):
			sim.select_car.call_deferred(int(arg.trim_prefix("--select-car=")))
		elif arg.begins_with("--screenshot="):
			_screenshot_path = arg.trim_prefix("--screenshot=")
			_screenshot_frames = 30


func _process(delta: float) -> void:
	if road == null:
		return
	if road.revision() != _last_revision:
		_refresh_map()
	elif Vector2i(road.history_size(), road.redo_size()) != _last_history:
		# A commit changes history without changing the map.
		_last_history = Vector2i(road.history_size(), road.redo_size())
		ui.refresh()
	_autosave_timer += delta
	if _autosave_timer >= AUTOSAVE_INTERVAL:
		_autosave_timer = 0.0
		autosave()
	if _perf_log:
		_perf_timer += delta
		_perf_frames += 1
		if _perf_timer >= 5.0:
			var st: Dictionary = sim.stats
			print("perf: %.0f fps · sim %.0fx of %.0fx%s · %.0f us a tick · %.1f ms of sim a frame · %d vehicles, %d people, %d residents" % [
				_perf_frames / _perf_timer, st.get("effective_speed", 0.0), sim.speed(), " (CPU-limited)" if st.get("behind", false) else "",
				st.get("tick_us", 0.0), st.get("frame_sim_ms", 0.0), st.get("vehicles", 0), int(st.get("pedestrians", 0)) + int(st.get("riding", 0)),
				st.get("residents", 0)])
			_perf_timer = 0.0
			_perf_frames = 0
	if _screenshot_frames > 0:
		_screenshot_frames -= 1
		if _screenshot_frames == 0:
			get_viewport().get_texture().get_image().save_png(_screenshot_path)
			print("Screenshot saved to %s" % _screenshot_path)
			get_tree().quit()


## M7: a PNG of the map as it is on screen, without the editor panels.
func export_screenshot() -> void:
	ui.visible = false
	await RenderingServer.frame_post_draw
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	ui.visible = true
	var stamp := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
	file_io.save_bytes(img.save_png_to_buffer(), "traffic-sim-%s.png" % stamp, "image/png", "*.png ; PNG images")
	notify("Screenshot taken (%d x %d)." % [img.get_width(), img.get_height()])


func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_CLOSE_REQUEST and road != null:
		autosave()


# --- Map refresh ------------------------------------------------------------------

func _refresh_map() -> void:
	_last_revision = road.revision()
	_last_history = Vector2i(road.history_size(), road.redo_size())
	view.rebuild(road.get_meshes(), level, level_filter)
	_prune_selection()
	ui.refresh()
	overlay.queue_redraw()


func _prune_selection() -> void:
	var nodes: Array = []
	for id in selection.nodes:
		if not road.get_node(id).is_empty():
			nodes.append(id)
	var segs: Array = []
	for id in selection.segments:
		if not road.get_segment(id).is_empty():
			segs.append(id)
	var blds: Array = []
	for id in selection.buildings:
		if not road.get_building(id).is_empty():
			blds.append(id)
	selection = {"nodes": nodes, "segments": segs, "buildings": blds}


# --- Tools, selection, level -------------------------------------------------------

func set_tool(tool_name: String) -> void:
	if not tools.has(tool_name):
		return
	if tool:
		tool.deactivate()
	tool = tools[tool_name]
	tool.activate()
	if ui:
		ui.refresh_tools()
		ui.set_status(tool.hint())
	overlay.queue_redraw()


func tool_name() -> String:
	for k in tools:
		if tools[k] == tool:
			return k
	return ""


func select(kind: String, id: int, additive: bool) -> void:
	if sim.selected_car != 0:
		sim.select_car(0)
	if sim.selected_ped != 0:
		sim.select_ped(0)
	if not additive:
		selection = {"nodes": [], "segments": [], "buildings": []}
	var list: Array = selection[kind]
	if additive and list.has(id):
		list.erase(id)
	elif not list.has(id):
		list.append(id)
	ui.refresh_inspector()
	overlay.queue_redraw()


func clear_selection() -> void:
	selection = {"nodes": [], "segments": [], "buildings": []}
	ui.refresh_inspector()
	overlay.queue_redraw()


func has_selection() -> bool:
	return not selection.nodes.is_empty() or not selection.segments.is_empty() or not selection.buildings.is_empty()


func delete_selection() -> void:
	if not has_selection():
		return
	road.begin("Delete")
	for id in selection.buildings:
		road.remove_building(id)
	for id in selection.segments:
		road.delete_segment(id)
	for id in selection.nodes:
		road.delete_node(id)
	road.commit()
	clear_selection()


func set_level(l: int) -> void:
	level = clampi(l, -1, 1)
	_apply_level_style()


func set_level_filter(on: bool) -> void:
	level_filter = on
	_apply_level_style()
	notify("Showing level %d only." % level if on else "Showing every level.")


func _apply_level_style() -> void:
	view.apply_level_style(level, level_filter)
	sim.apply_level_style(level, level_filter)
	heat.refresh()
	ui.refresh()
	overlay.queue_redraw()


func set_template(template: Dictionary, label: String) -> void:
	road_template = template
	road_template_name = label
	var profile: Dictionary
	if template.has("profile"):
		profile = template.profile
	elif template.has("params"):
		profile = road.profile_from_params(template.params)
	else:
		profile = road.profile_from_params(_preset_params(template.preset))
	_template_width = float(profile.get("width", 10.0))


func template_width() -> float:
	return _template_width


func _preset_params(preset_name: String) -> Dictionary:
	for p in road.presets():
		if p.name == preset_name:
			return p.params
	return road.presets()[0].params


# --- Snapping ----------------------------------------------------------------------

func pick_radius() -> float:
	return PICK_PIXELS * EditorOverlay.ui_scale_for(get_viewport_rect().size) / camera.zoom.x


func mouse_world() -> Vector2:
	return get_global_mouse_position()


## Snaps a world point for drawing: existing node > point on a road > angle
## (from `from`) and/or the 1 m grid. Returns {pos, node?, segment?, u?, kind}.
func snap_point(world: Vector2, from = null, exclude_node := 0) -> Dictionary:
	var r := pick_radius()
	var n: int = road.nearest_node(world, r * 1.2, level, exclude_node)
	if n != 0:
		return {"pos": road.get_node(n).pos, "node": n, "kind": "node"}
	var hit: Dictionary = road.pick(world, r * 0.5, level)
	if hit.type == "segment":
		var u: float = road.segment_u_at(hit.id, world)
		return {"pos": road.segment_point(hit.id, u), "segment": hit.id, "u": u, "kind": "segment"}
	var p := world
	var kind := "free"
	if snap_angle and from != null:
		var origin: Vector2 = from
		var d := world - origin
		if d.length() > 0.01:
			var ang := snappedf(d.angle(), deg_to_rad(15.0))
			var length := d.length()
			if snap_grid:
				length = maxf(1.0, roundf(length))
			p = origin + Vector2.from_angle(ang) * length
			kind = "angle"
	elif snap_grid:
		p = world.snapped(Vector2.ONE)
		kind = "grid"
	return {"pos": p, "kind": kind}


static func to_ref(snap: Dictionary) -> Dictionary:
	var ref := {"pos": snap.pos}
	if snap.has("node"):
		ref["node"] = snap.node
	if snap.has("segment"):
		ref["segment"] = snap.segment
	return ref


# --- History -------------------------------------------------------------------------

func undo() -> void:
	if road.undo():
		notify("Undid: %s" % road.redo_label())


func redo() -> void:
	if road.redo():
		notify("Redid: %s" % road.undo_label())


# --- Files ---------------------------------------------------------------------------

func autosave() -> void:
	if road == null or road.revision() == _saved_revision:
		return
	var f := FileAccess.open(AUTOSAVE_PATH, FileAccess.WRITE)
	if f == null:
		return
	f.store_string(road.save_json())
	f.close()
	_saved_revision = road.revision()
	ui.set_autosave_time(Time.get_time_string_from_system().substr(0, 5))


func new_map() -> void:
	road.new_city()
	clear_selection()
	_refresh_map()
	sim.reset()
	fit_view()
	notify("New city: the main station and the city offices. Press H to place homes, shops and offices.")


## M7: the tutorial - a new city and the step-by-step checklist.
func start_tutorial() -> void:
	sim.pause()
	road.load_example("tutorial")
	sim.city_prefill = 0.0
	clear_selection()
	_refresh_map()
	sim.reset()
	fit_view()
	set_tool("select")
	ui.guide.start()
	notify("Tutorial: follow the steps at the top of the screen.")


## "town", "grid", "t_junction", "lane_drop" or "one_way_pair".
func load_demo(which: String) -> void:
	if which == "empty":
		road.new_map()
	else:
		road.load_example(which)
	sim.city_prefill = 1.0 if which == "city_town" else 0.95 if which in ["city_week", "city_market"] else 0.0
	clear_selection()
	_refresh_map()
	sim.pause()
	sim.reset()
	fit_view()


func export_map() -> void:
	file_io.save_text(road.save_json(), "traffic-sim-map.json")


func import_map() -> void:
	file_io.open_text(func(text: String) -> void:
		var r: Dictionary = road.load_json(text)
		if not r.ok:
			notify("Could not import: %s" % r.error)
			return
		clear_selection()
		_refresh_map()
		sim.pause()
		sim.reset()
		fit_view()
		if r.migrated_from > 0:
			notify("Imported a version %d map and upgraded it to version 3." % r.migrated_from)
		else:
			notify("Map imported."))


func open_ring_benchmark() -> void:
	autosave()
	get_tree().change_scene_to_file(RING_SCENE)


func fit_view() -> void:
	var box := Rect2()
	var first := true
	for id in road.node_ids():
		var p: Vector2 = road.get_node(id).pos
		if first:
			box = Rect2(p, Vector2.ZERO)
			first = false
		else:
			box = box.expand(p)
	if first:
		camera.position = Vector2.ZERO
		camera.zoom = Vector2.ONE * 2.0
		return
	camera.fit(box.grow(40.0))


func focus(pos: Vector2) -> void:
	camera.position = pos
	if camera.zoom.x < 1.5:
		camera.zoom = Vector2.ONE * 2.0


# --- User presets ----------------------------------------------------------------------

func save_user_preset(preset_name: String, profile: Dictionary) -> void:
	var clean := profile.duplicate(true)
	for l in clean.lanes:
		l.erase("id")
	for i in user_presets.size():
		if user_presets[i].name == preset_name:
			user_presets.remove_at(i)
			break
	user_presets.append({"name": preset_name, "profile": clean})
	_store_user_presets()
	notify("Saved profile \"%s\"." % preset_name)


func delete_user_preset(preset_name: String) -> void:
	for i in user_presets.size():
		if user_presets[i].name == preset_name:
			user_presets.remove_at(i)
			_store_user_presets()
			notify("Deleted profile \"%s\"." % preset_name)
			return


## The saved profiles as a file, to move them to another browser or computer.
func export_user_presets() -> void:
	file_io.save_text(JSON.stringify({"format": "traffic-sim-profiles", "version": 1, "profiles": user_presets}, "  "),
		"traffic-sim-profiles.json")


## Adds the profiles from a file (same names are replaced). Returns how many.
func import_user_presets_text(text: String) -> int:
	var data = JSON.parse_string(text)
	var list = data.get("profiles", null) if typeof(data) == TYPE_DICTIONARY else data
	if typeof(list) != TYPE_ARRAY:
		notify("That isn't a profiles file.")
		return 0
	var n := 0
	for p in list:
		if typeof(p) != TYPE_DICTIONARY or not p.has("name") or typeof(p.get("profile")) != TYPE_DICTIONARY:
			continue
		if String(road.validate_profile(p.profile)) != "":
			continue
		for i in user_presets.size():
			if user_presets[i].name == p.name:
				user_presets.remove_at(i)
				break
		user_presets.append({"name": String(p.name), "profile": p.profile})
		n += 1
	_store_user_presets()
	notify("Imported %d profile%s." % [n, "" if n == 1 else "s"])
	return n


func import_user_presets() -> void:
	file_io.open_text(func(text: String) -> void: import_user_presets_text(text))


func _store_user_presets() -> void:
	var f := FileAccess.open(PRESETS_PATH, FileAccess.WRITE)
	if f:
		f.store_string(JSON.stringify(user_presets, "  "))
	ui.refresh_presets()
	ui.refresh_inspector()


func _load_user_presets() -> void:
	if not FileAccess.file_exists(PRESETS_PATH):
		return
	var data = JSON.parse_string(FileAccess.get_file_as_string(PRESETS_PATH))
	if typeof(data) == TYPE_ARRAY:
		for p in data:
			if typeof(p) == TYPE_DICTIONARY and p.has("name") and p.has("profile"):
				user_presets.append(p)


# --- Input -------------------------------------------------------------------------------

func _unhandled_input(event: InputEvent) -> void:
	if road == null:
		return
	# The first finger of a pinch also arrives as a mouse drag; it moves the
	# camera, not the tool (#24).
	if event is InputEventMouseMotion and camera.is_pinching():
		return
	if tool and tool.input(event):
		get_viewport().set_input_as_handled()
		overlay.queue_redraw()
		return
	if event is InputEventMouseMotion:
		ui.set_cursor(mouse_world())
		overlay.queue_redraw()
	var k := event as InputEventKey
	if k == null or not k.pressed or k.echo:
		return
	var mod := k.ctrl_pressed or k.meta_pressed
	var handled := true
	match k.keycode:
		KEY_Z when mod and k.shift_pressed:
			redo()
		KEY_Z when mod:
			undo()
		KEY_Y when mod:
			redo()
		KEY_S when mod:
			export_map()
		KEY_O when mod:
			import_map()
		KEY_P when mod and k.shift_pressed:
			export_screenshot()
		KEY_V:
			set_tool("select")
		KEY_R:
			set_tool("road")
		KEY_C:
			set_tool("curve")
		KEY_L:
			set_tool("lane")
		KEY_N:
			set_tool("spawner")
		KEY_O:
			set_tool("roundabout")
		KEY_K:
			set_tool("stop")
		KEY_D:
			set_tool("depot")
		KEY_U:
			set_tool("route")
		KEY_P:
			set_tool("path")
		KEY_W:
			set_tool("crosswalk")
		KEY_E:
			set_tool("fence")
		KEY_B:
			set_tool("bridge")
		KEY_H:
			set_tool("building")
		KEY_T:
			set_tool("centre")
		KEY_J:
			set_level_filter(not level_filter)
		KEY_M:
			ui.cycle_heatmap()
		KEY_SPACE:
			sim.toggle()
		KEY_PERIOD:
			sim.step()
		KEY_F:
			fit_view()
		KEY_G:
			snap_grid = not snap_grid
			ui.refresh()
		KEY_A:
			snap_angle = not snap_angle
			ui.refresh()
		KEY_PAGEUP:
			set_level(level + 1)
		KEY_PAGEDOWN:
			set_level(level - 1)
		KEY_DELETE, KEY_BACKSPACE:
			delete_selection()
		KEY_ESCAPE:
			clear_selection()
			sim.select_car(0)
			sim.select_ped(0)
		_:
			handled = false
	if handled:
		get_viewport().set_input_as_handled()


func notify(text: String) -> void:
	print(text)
	if ui:
		ui.toast(text)


func _show_fatal(text: String) -> void:
	push_error(text)
	var layer := CanvasLayer.new()
	var label := Label.new()
	label.text = text
	label.position = Vector2(40, 40)
	label.add_theme_font_size_override("font_size", 22)
	layer.add_child(label)
	add_child(layer)
