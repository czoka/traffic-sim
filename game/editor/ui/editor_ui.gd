class_name EditorUI
extends CanvasLayer
## Editor chrome: tool palette (left), inspector (right), simulation bar and
## bottom bar (history, level, snapping, status), and the problems panel.

const TOOLS := [
	["select", "Select", "V"],
	["road", "Road", "R"],
	["curve", "Curve road", "C"],
	["lane", "Lane paint", "L"],
	["spawner", "Spawn point", "N"],
	["roundabout", "Roundabout", "O"],
	["stop", "Bus stop", "K"],
	["depot", "Bus depot", "D"],
	["route", "Bus route", "U"],
	["path", "Path", "P"],
	["crosswalk", "Crossing", "W"],
	["fence", "Fence", "E"],
	["bridge", "Bridge / tunnel", "B"],
	["building", "Building", "H"],
]
const PANEL_BG := Color(0.08, 0.09, 0.1, 0.92)

var editor: MapEditor
var inspector: Inspector

var _tool_buttons := {}
var _preset: OptionButton
var _preset_values: Array = [] # template dictionaries matching _preset items
var _speed: SpinBox
var _undo: Button
var _redo: Button
var _levels: Array[Button] = []
var _grid: Button
var _angle: Button
var _connectors: Button
var _level_only: Button
var _status: Label
var _cursor: Label
var _problems_button: Button
var _autosave: Label
var _problems_panel: PanelContainer
var _problems_list: ItemList
var _problems: Array = []
var _toast: Label
var _toast_time := 0.0

# Simulation bar
var _play: Button
var _speed_opt: OptionButton
var _seed: SpinBox
var _demand: HSlider
var _demand_label: Label
var _max_cars: SpinBox
var _sim_label: Label


func _ready() -> void:
	var root := Control.new()
	root.set_anchors_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(root)
	_build_palette(root)
	inspector = Inspector.new()
	inspector.editor = editor
	inspector.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	inspector.position = Vector2(-12, 12)
	root.add_child(inspector)
	_build_bottom_bar(root)
	_build_sim_bar(root)
	_build_problems(root)
	_toast = Label.new()
	_toast.set_anchors_preset(Control.PRESET_CENTER_TOP)
	_toast.position = Vector2(-240, 14)
	_toast.custom_minimum_size = Vector2(480, 0)
	_toast.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_toast.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_toast.add_theme_color_override("font_color", Color(0.85, 0.92, 1.0))
	_toast.add_theme_stylebox_override("normal", _panel_style(Color(0.05, 0.06, 0.08, 0.9)))
	_toast.visible = false
	root.add_child(_toast)
	refresh_presets()


func _process(delta: float) -> void:
	if _toast.visible:
		_toast_time -= delta
		if _toast_time <= 0.0:
			_toast.visible = false
	# Keep the inspector pinned to the right edge as the window resizes.
	inspector.position = Vector2(get_viewport().get_visible_rect().size.x - inspector.size.x - 12, 12)


static func _panel_style(bg: Color) -> StyleBoxFlat:
	var s := StyleBoxFlat.new()
	s.bg_color = bg
	s.set_corner_radius_all(8)
	s.set_content_margin_all(10)
	return s


static func section(text: String) -> Label:
	var l := Label.new()
	l.text = text.to_upper()
	l.add_theme_font_size_override("font_size", 11)
	l.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	return l


func _button(text: String, action: Callable, parent: Control) -> Button:
	var b := Button.new()
	b.text = text
	b.focus_mode = Control.FOCUS_NONE
	b.pressed.connect(action)
	parent.add_child(b)
	return b


# --- Palette -------------------------------------------------------------------------

func _build_palette(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(12, 12)
	panel.add_theme_stylebox_override("panel", _panel_style(PANEL_BG))
	root.add_child(panel)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 6)
	box.custom_minimum_size = Vector2(190, 0)
	panel.add_child(box)
	var title := Label.new()
	title.text = "Traffic Sim"
	title.add_theme_font_size_override("font_size", 17)
	box.add_child(title)

	box.add_child(section("Tools"))
	for t in TOOLS:
		var b := _button("%s   %s" % [t[1], t[2]], editor.set_tool.bind(t[0]), box)
		b.toggle_mode = true
		b.alignment = HORIZONTAL_ALIGNMENT_LEFT
		_tool_buttons[t[0]] = b

	box.add_child(section("New roads"))
	_preset = OptionButton.new()
	_preset.focus_mode = Control.FOCUS_NONE
	_preset.fit_to_longest_item = false
	_preset.item_selected.connect(func(i: int) -> void:
		editor.set_template(_preset_values[i], _preset.get_item_text(i)))
	box.add_child(_preset)
	var speed_row := HBoxContainer.new()
	var sl := Label.new()
	sl.text = "Speed limit"
	speed_row.add_child(sl)
	_speed = SpinBox.new()
	_speed.min_value = 10
	_speed.max_value = 130
	_speed.step = 10
	_speed.suffix = "km/h"
	_speed.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_speed.custom_minimum_size = Vector2(100, 0)
	speed_row.add_child(_speed)
	_speed.value = editor.speed_kmh
	_speed.value_changed.connect(func(v: float) -> void: editor.speed_kmh = v)
	box.add_child(speed_row)

	box.add_child(section("Map"))
	var files := HBoxContainer.new()
	_button("New", _confirm_new, files)
	_button("Open…", editor.import_map, files)
	_button("Export…", editor.export_map, files)
	box.add_child(files)
	var demos := MenuButton.new()
	demos.text = "Examples"
	demos.flat = false
	demos.focus_mode = Control.FOCUS_NONE
	demos.get_popup().add_item("Demo town", 0)
	demos.get_popup().add_item("Test grid (56 junctions, M2 gate)", 1)
	demos.get_popup().add_item("T junction (priority road)", 3)
	demos.get_popup().add_item("Lane drop", 4)
	demos.get_popup().add_item("One-way pair", 5)
	demos.get_popup().add_item("Showcase (M3: signals, roundabout, buses, bikes, parking)", 6)
	demos.get_popup().add_item("People town (M4: crossings, bridge, overpass, passengers)", 7)
	demos.get_popup().add_item("People city (M4 gate: 2,000 vehicles, 1,000 people)", 8)
	demos.get_popup().add_item("City town (M5: homes, shops, offices, residents)", 9)
	demos.get_popup().add_item("City week (M5 gate: 5,000 residents)", 10)
	demos.get_popup().add_item("Empty map", 11)
	demos.get_popup().add_separator()
	demos.get_popup().add_item("POC ring benchmark", 2)
	demos.get_popup().id_pressed.connect(_on_example)
	box.add_child(demos)

	var help := Label.new()
	help.text = "Wheel zoom · right-drag pan · F fit\nCtrl+Z undo · Ctrl+Shift+Z redo\nG grid · A angles · PgUp/PgDn level\nSpace play/pause · . step 1 s"
	help.add_theme_font_size_override("font_size", 11)
	help.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	box.add_child(help)


func _on_example(id: int) -> void:
	match id:
		0:
			editor.load_demo("town")
		1:
			editor.load_demo("grid")
		2:
			editor.open_ring_benchmark()
		3:
			editor.load_demo("t_junction")
		4:
			editor.load_demo("lane_drop")
		5:
			editor.load_demo("one_way_pair")
		6:
			editor.load_demo("showcase")
		7:
			editor.load_demo("people")
		8:
			editor.load_demo("people_city")
		9:
			editor.load_demo("city_town")
		10:
			editor.load_demo("city_week")
		11:
			editor.load_demo("empty")


func _confirm_new() -> void:
	var d := ConfirmationDialog.new()
	d.dialog_text = "Start a new city? It begins with High Street, the main station with a coach line, and the city offices. The current map stays in the autosave until the next save, and undo history is cleared. (Examples → Empty map starts from nothing.)"
	d.confirmed.connect(func() -> void:
		editor.new_map()
		d.queue_free())
	d.canceled.connect(d.queue_free)
	add_child(d)
	d.popup_centered()


func refresh_presets() -> void:
	if _preset == null:
		return
	_preset.clear()
	_preset_values.clear()
	for p in editor.road.presets():
		_preset.add_item(p.name)
		_preset_values.append({"preset": p.name})
	for p in editor.user_presets:
		_preset.add_item("* " + String(p.name))
		_preset_values.append({"profile": p.profile})
	for i in _preset.item_count:
		if _preset.get_item_text(i) == editor.road_template_name:
			_preset.select(i)


func refresh_tools() -> void:
	var current := editor.tool_name()
	for k in _tool_buttons:
		_tool_buttons[k].set_pressed_no_signal(k == current)


# --- Bottom bar ----------------------------------------------------------------------

func _build_bottom_bar(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	panel.offset_top = -44
	panel.add_theme_stylebox_override("panel", _panel_style(PANEL_BG))
	root.add_child(panel)
	var bar := HBoxContainer.new()
	bar.add_theme_constant_override("separation", 8)
	panel.add_child(bar)
	_undo = _button("Undo", editor.undo, bar)
	_redo = _button("Redo", editor.redo, bar)
	bar.add_child(VSeparator.new())
	var ll := Label.new()
	ll.text = "Level"
	bar.add_child(ll)
	for l in [-1, 0, 1]:
		var b := _button(["−1", "0", "+1"][l + 1], editor.set_level.bind(l), bar)
		b.toggle_mode = true
		b.tooltip_text = ["Underpass (level −1)", "Ground (level 0)", "Overpass (level +1)"][l + 1]
		_levels.append(b)
	_level_only = _button("Only this level", func() -> void: editor.set_level_filter(not editor.level_filter), bar)
	_level_only.toggle_mode = true
	_level_only.tooltip_text = "Hide the other levels (J)"
	bar.add_child(VSeparator.new())
	_grid = _button("Grid 1 m", _toggle_grid, bar)
	_grid.toggle_mode = true
	_angle = _button("15°", _toggle_angle, bar)
	_angle.toggle_mode = true
	_connectors = _button("Connectors", _toggle_connectors, bar)
	_connectors.toggle_mode = true
	bar.add_child(VSeparator.new())
	_status = Label.new()
	_status.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status.clip_text = true
	_status.add_theme_color_override("font_color", Color(0.8, 0.84, 0.9))
	bar.add_child(_status)
	_cursor = Label.new()
	_cursor.custom_minimum_size = Vector2(120, 0)
	_cursor.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	bar.add_child(_cursor)
	_problems_button = _button("No problems", _toggle_problems, bar)
	_autosave = Label.new()
	_autosave.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	_autosave.text = "Not saved yet"
	bar.add_child(_autosave)


func _toggle_grid() -> void:
	editor.snap_grid = not editor.snap_grid
	refresh()


func _toggle_angle() -> void:
	editor.snap_angle = not editor.snap_angle
	refresh()


func _toggle_connectors() -> void:
	editor.show_connectors = not editor.show_connectors
	editor.overlay.queue_redraw()
	refresh()


func _toggle_problems() -> void:
	_problems_panel.visible = not _problems_panel.visible


# --- Simulation bar ------------------------------------------------------------------

func _build_sim_bar(root: Control) -> void:
	var sim := editor.sim
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_CENTER_BOTTOM)
	panel.grow_horizontal = Control.GROW_DIRECTION_BOTH
	panel.offset_top = -96
	panel.offset_bottom = -52
	panel.add_theme_stylebox_override("panel", _panel_style(PANEL_BG))
	root.add_child(panel)
	var bar := HBoxContainer.new()
	bar.add_theme_constant_override("separation", 8)
	panel.add_child(bar)
	_play = _button("Play", sim.toggle, bar)
	_play.custom_minimum_size = Vector2(90, 0)
	_play.tooltip_text = "Play / pause (Space). Editing pauses; Play resumes with the changes."
	var step := _button("Step", sim.step, bar)
	step.tooltip_text = "One sim second (.)"
	var restart := _button("Restart", sim.reset, bar)
	restart.tooltip_text = "Remove every car and start again with the seed"
	bar.add_child(VSeparator.new())
	var sl := Label.new()
	sl.text = "Speed"
	bar.add_child(sl)
	_speed_opt = OptionButton.new()
	_speed_opt.focus_mode = Control.FOCUS_NONE
	for i in SimController.MULTIPLIERS.size():
		_speed_opt.add_item(sim.speed_label(i))
	_speed_opt.select(sim.speed_index)
	_speed_opt.item_selected.connect(sim.set_speed_index)
	bar.add_child(_speed_opt)
	var seed_label := Label.new()
	seed_label.text = "Seed"
	bar.add_child(seed_label)
	_seed = SpinBox.new()
	_seed.min_value = 0
	_seed.max_value = 999999
	_seed.value = sim.seed_value
	_seed.tooltip_text = "Same map + same seed = same run. Restart to apply."
	_seed.value_changed.connect(func(v: float) -> void: sim.seed_value = int(v))
	bar.add_child(_seed)
	bar.add_child(VSeparator.new())
	_demand_label = Label.new()
	_demand_label.custom_minimum_size = Vector2(92, 0)
	bar.add_child(_demand_label)
	_demand = HSlider.new()
	_demand.min_value = 0.0
	_demand.max_value = 3.0
	_demand.step = 0.05
	_demand.value = sim.demand
	_demand.custom_minimum_size = Vector2(110, 0)
	_demand.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_demand.focus_mode = Control.FOCUS_NONE
	_demand.tooltip_text = "Multiplies every spawn point's rate"
	_demand.value_changed.connect(func(v: float) -> void:
		sim.set_demand(v)
		_demand_label.text = "Density ×%.2f" % v)
	_demand_label.text = "Density ×%.2f" % sim.demand
	bar.add_child(_demand)
	var cap_label := Label.new()
	cap_label.text = "Max cars"
	bar.add_child(cap_label)
	_max_cars = SpinBox.new()
	_max_cars.min_value = 0
	_max_cars.max_value = 20000
	_max_cars.step = 50
	_max_cars.value = sim.max_cars
	_max_cars.tooltip_text = "Spawning pauses while this many cars are on the map (0 = no limit)"
	_max_cars.value_changed.connect(func(v: float) -> void: sim.set_max_cars(int(v)))
	bar.add_child(_max_cars)
	var people_label := Label.new()
	people_label.text = "Max people"
	bar.add_child(people_label)
	var max_people := SpinBox.new()
	max_people.min_value = 0
	max_people.max_value = 20000
	max_people.step = 100
	max_people.value = sim.max_people
	max_people.tooltip_text = "New trips on foot pause while this many people are on the map (0 = no limit)"
	max_people.value_changed.connect(func(v: float) -> void: sim.set_max_people(int(v)))
	bar.add_child(max_people)
	bar.add_child(VSeparator.new())
	_sim_label = Label.new()
	_sim_label.custom_minimum_size = Vector2(330, 0)
	_sim_label.clip_text = true
	_sim_label.mouse_filter = Control.MOUSE_FILTER_PASS # for the tooltip
	_sim_label.add_theme_color_override("font_color", Color(0.8, 0.84, 0.9))
	bar.add_child(_sim_label)
	sim.state_changed.connect(_refresh_sim_buttons)
	_refresh_sim_buttons()


func _refresh_sim_buttons() -> void:
	if _play == null:
		return
	_play.text = "Pause" if editor.sim.playing else "Play"
	_speed_opt.select(editor.sim.speed_index)


static func clock(seconds: float) -> String:
	var s := int(seconds)
	return "%d:%02d:%02d" % [s / 3600, (s / 60) % 60, s % 60]


func refresh_sim(st: Dictionary) -> void:
	if _sim_label == null or st.is_empty():
		return
	var text := "%s · %d vehicles · %d trips · %.0f km/h · %d stopped" % [
		clock(st.sim_time), st.vehicles, st.arrived, st.mean_speed_kmh, st.stopped]
	if st.get("city_on", false):
		text = "%s · %d residents · %d vehicles · %.0f km/h" % [SimController.clock_text(st.clock_day, st.clock_minute),
			st.residents, st.vehicles, st.mean_speed_kmh]
	var extra: Array = []
	if int(st.buses) + int(st.coaches) > 0:
		extra.append("%d bus%s" % [int(st.buses) + int(st.coaches), "" if int(st.buses) + int(st.coaches) == 1 else "es"])
	if int(st.bikes) > 0:
		extra.append("%d bike%s" % [st.bikes, "" if int(st.bikes) == 1 else "s"])
	if int(st.parked) > 0:
		extra.append("%d parked" % st.parked)
	if not extra.is_empty():
		text += " (" + ", ".join(extra) + ")"
	if int(st.trips) > 0:
		text += " · %d people (%d on buses)" % [int(st.pedestrians) + int(st.riding), st.riding]
	if int(st.waiting_to_enter) > 0:
		text += " · %d waiting to enter" % st.waiting_to_enter
	if editor.sim.playing:
		text += " · %.0fx" % st.effective_speed
		if st.behind:
			text += " (CPU-limited)"
	_sim_label.text = text
	_sim_label.tooltip_text = "Sim %.0f µs per tick, %.1f ms per frame · %d lane changes · %d re-routes · longest stop %.0f s · %d cars taken off (stuck)\n%d cars, %d taxis, %d buses, %d coaches, %d bikes · %d bus runs, %d stops served, %d coach calls · %d parkings (%d found no bay) · %d right turns on red" % [
		st.tick_us, st.frame_sim_ms, st.lane_changes, st.reroutes, st.max_stopped, st.removed_stuck,
		st.cars, st.taxis, st.buses, st.coaches, st.bikes, st.bus_runs, st.bus_stops_served, st.coach_calls,
		st.parkings, st.parking_failed, st.right_on_red]
	var c: Dictionary = editor.sim.city
	if not c.is_empty():
		_sim_label.tooltip_text += "\nCity: %d residents in %d households (%d of %d units vacant), %d visitors · %d asleep, %d at work, %d travelling, %d outside the map\nJobs: %d local, %d outside, %d looking · %d shifts (%d late), %d left to visitors · %d of %d businesses open, %d closed unexpectedly, %d late openings\n%d meals out, %d at home, %d grocery trips · %d immigrants · mean hunger %.0f, energy %.0f, money %.0f" % [
			c.residents, c.households, c.vacant_units, c.units, c.visitors, c.sleeping, c.working, c.travelling, c.outside,
			c.employed, c.employed_outside, c.unemployed, c.shifts, c.late_shifts, c.unfilled_shifts, c.open, c.businesses,
			c.closed_unexpectedly, c.late_openings, c.meals_out, c.home_meals, c.groceries, c.immigrants, c.mean_hunger,
			c.mean_energy, c.mean_money]
	if int(st.trips) > 0:
		_sim_label.tooltip_text += "\n%d people trips: %d walk, %d bus, %d bike, %d car, %d coach · %d arrived\n%d boarded, %d got off, %d left behind, mean wait at stops %.0f s · %d crossings, mean wait at the kerb %.1f s, %d times cars gave way" % [
			st.trips, st.trips_walk, st.trips_bus, st.trips_bike, st.trips_car, st.trips_coach, st.people_arrived,
			st.boarded, st.alighted, st.left_behind, st.mean_wait, st.crossings, st.mean_crossing_wait, st.cars_yielded]


func _build_problems(root: Control) -> void:
	_problems_panel = PanelContainer.new()
	_problems_panel.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	_problems_panel.offset_left = -520
	_problems_panel.offset_top = -344
	_problems_panel.offset_right = -12
	_problems_panel.offset_bottom = -100
	_problems_panel.add_theme_stylebox_override("panel", _panel_style(PANEL_BG))
	_problems_panel.visible = false
	root.add_child(_problems_panel)
	var box := VBoxContainer.new()
	_problems_panel.add_child(box)
	box.add_child(section("Problems"))
	_problems_list = ItemList.new()
	_problems_list.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_problems_list.focus_mode = Control.FOCUS_NONE
	_problems_list.item_selected.connect(_on_problem_selected)
	box.add_child(_problems_list)


func _on_problem_selected(i: int) -> void:
	if i < 0 or i >= _problems.size():
		return
	var p: Dictionary = _problems[i]
	editor.set_level(int(p.level))
	editor.focus(p.pos)
	if not (p.get("buildings", []) as Array).is_empty():
		editor.select("buildings", p.buildings[0], false)
	elif not p.segments.is_empty():
		editor.select("segments", p.segments[0], false)
		for k in range(1, p.segments.size()):
			editor.select("segments", p.segments[k], true)
	elif not p.nodes.is_empty():
		editor.select("nodes", p.nodes[0], false)


# --- Refresh ------------------------------------------------------------------------

func refresh() -> void:
	var road = editor.road
	_undo.disabled = not road.can_undo()
	_redo.disabled = not road.can_redo()
	_undo.tooltip_text = "Undo " + road.undo_label() if road.can_undo() else ""
	_redo.tooltip_text = "Redo " + road.redo_label() if road.can_redo() else ""
	for i in _levels.size():
		_levels[i].set_pressed_no_signal(i - 1 == editor.level)
	_grid.set_pressed_no_signal(editor.snap_grid)
	_angle.set_pressed_no_signal(editor.snap_angle)
	_connectors.set_pressed_no_signal(editor.show_connectors)
	_level_only.set_pressed_no_signal(editor.level_filter)
	_problems = road.get_problems()
	var errors := 0
	_problems_list.clear()
	for p in _problems:
		if p.severity == "error":
			errors += 1
		var icon := "Error: " if p.severity == "error" else "Warning: "
		_problems_list.add_item(icon + String(p.message))
	var warnings := _problems.size() - errors
	if _problems.is_empty():
		_problems_button.text = "No problems"
	else:
		_problems_button.text = "%d error%s, %d warning%s" % [errors, "" if errors == 1 else "s", warnings, "" if warnings == 1 else "s"]
	_problems_button.add_theme_color_override("font_color", Color(0.95, 0.45, 0.4) if errors > 0 else Color(0.9, 0.9, 0.9))
	refresh_tools()
	refresh_inspector()


func refresh_inspector() -> void:
	if inspector:
		inspector.refresh()


func set_status(text: String) -> void:
	if _status:
		_status.text = text


func set_cursor(pos: Vector2) -> void:
	if _cursor:
		_cursor.text = "%.0f, %.0f m" % [pos.x, pos.y]


func set_autosave_time(t: String) -> void:
	_autosave.text = "Autosaved " + t


func toast(text: String) -> void:
	if _toast == null:
		return
	_toast.text = text
	_toast.visible = true
	_toast_time = 3.5
