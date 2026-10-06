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
	["centre", "City centre", "T"],
]
const PANEL_BG := Color(0.08, 0.09, 0.1, 0.92)

var editor: MapEditor
var inspector: Inspector
var guide: Guide

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
var _scale: ScaleBar
var _problems_button: Button
var _autosave: Label
var _problems_panel: PanelContainer
var _problems_list: ItemList
var _problems: Array = []
var _heat_opt: OptionButton
var _heat_legend: Label
var _market_button: Button
var _market_panel: PanelContainer
var _market_list: ItemList
var _market: Array = []
var _market_time := 0.0
var _toast: Label
var _root: Control
var _palette: PanelContainer
var _palette_scroll: ScrollContainer
var _palette_box: VBoxContainer
var _toast_time := 0.0

# Simulation bar
var _play: Button
var _transport: PanelContainer # play / step / restart, icons at the top centre (#16)
const ICON_PLAY := preload("res://editor/icons/play.svg")
const ICON_PAUSE := preload("res://editor/icons/pause.svg")
const ICON_STEP := preload("res://editor/icons/step.svg")
const ICON_RESTART := preload("res://editor/icons/restart.svg")
const ICON_SETTINGS := preload("res://editor/icons/settings.svg")
const ICON_SLOWER := preload("res://editor/icons/slower.svg")
const ICON_FASTER := preload("res://editor/icons/faster.svg")
var _speed_label: Label # current sim speed between the slower / faster buttons (#22)
var _slower: Button
var _faster: Button
var _seed: SpinBox
var _demand: HSlider
var _demand_label: Label
var _max_cars: SpinBox
var _max_people: SpinBox
var _run_dialog: PanelContainer # New run (Restart) / Settings (#26)
var _run_title: Label
var _seed_label: Label
var _run_ok: Button
var _run_cancel: Button
var _run_live := false # true in Settings: changes apply at once
var _settings_btn: Button
var _sim_label: Label


func _ready() -> void:
	var root := Control.new()
	_root = root
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
	_build_market(root)
	guide = Guide.new()
	guide.editor = editor
	guide.position = Vector2(12, 12) # beside the palette (placed in _process)
	root.add_child(guide)
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
	if editor and editor.camera:
		_scale.zoom = editor.camera.zoom.x
	# Keep the inspector pinned to the right edge as the window resizes.
	var vp := get_viewport().get_visible_rect().size
	inspector.fit_height(vp.y - 12.0 - 112.0) # above the sim and bottom bars
	inspector.position = Vector2(vp.x - inspector.size.x - 12, 12)
	# The palette scrolls when the window is too short for it (above the bottom bars).
	if _palette_scroll:
		_palette_scroll.custom_minimum_size.y = minf(_palette_box.size.y, maxf(200.0, vp.y - 140.0))
		guide.position = Vector2(_palette.position.x + _palette.size.x + 12, 12)
	# Play / step / restart at the top centre, clear of the tutorial checklist;
	# the toast goes under them.
	var left := guide.position.x + guide.size.x + 12.0 if guide.visible else 0.0
	if _transport:
		_transport.position = Vector2(maxf(vp.x * 0.5 - _transport.size.x * 0.5, left), 12)
	var below := (_transport.position.y + _transport.size.y + 8.0) if _transport else 14.0
	_toast.position = Vector2(maxf(vp.x * 0.5 - 240.0, left), below)


## True where a panel or bar covers the screen at `pos`: the wheel there
## scrolls the panel and never zooms the map (#12).
func over_ui(pos: Vector2) -> bool:
	if _root == null:
		return false
	for c in _root.get_children():
		var ctl := c as Control
		if ctl and ctl.visible and ctl.mouse_filter != Control.MOUSE_FILTER_IGNORE and ctl.get_global_rect().has_point(pos):
			return true
	return false


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
	_palette = panel
	_palette_scroll = ScrollContainer.new()
	_palette_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	panel.add_child(_palette_scroll)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 6)
	box.custom_minimum_size = Vector2(190, 0)
	_palette_scroll.add_child(box)
	_palette_box = box
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
	demos.get_popup().add_item("City market (M6: money, rent, owners, cars and bikes)", 12)
	demos.get_popup().add_item("Tutorial (a new city and a checklist)", 13)
	demos.get_popup().add_item("Empty map", 11)
	demos.get_popup().add_separator()
	demos.get_popup().add_item("POC ring benchmark", 2)
	demos.get_popup().id_pressed.connect(_on_example)
	box.add_child(demos)
	var learn := HBoxContainer.new()
	var tut := _button("Tutorial", editor.start_tutorial, learn)
	tut.tooltip_text = "A new city and a checklist: build a street, homes, a shop and a bus route, then press Play"
	box.add_child(learn)
	var shot := _button("Screenshot (PNG)", editor.export_screenshot, learn)
	shot.tooltip_text = "Save the map as it is on screen as a PNG, without the panels (Ctrl+Shift+P)"

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
		12:
			editor.load_demo("city_market")
		13:
			editor.start_tutorial()


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
	_scale = ScaleBar.new()
	bar.add_child(_scale)
	_heat_opt = OptionButton.new()
	_heat_opt.focus_mode = Control.FOCUS_NONE
	for l in HeatLayer.MODE_LABELS:
		_heat_opt.add_item(l)
	_heat_opt.tooltip_text = "Colour the roads by what the sim has seen in the last few minutes (M)"
	_heat_opt.item_selected.connect(func(i: int) -> void: set_heatmap(HeatLayer.MODES[i]))
	bar.add_child(_heat_opt)
	_problems_button = _button("No problems", _toggle_problems, bar)
	_market_button = _button("Market", func() -> void:
		_market_panel.visible = not _market_panel.visible
		_problems_panel.visible = false
		_refresh_market(), bar)
	_market_button.tooltip_text = "Buildings for sale while the city sim runs"
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
	_market_panel.visible = false


# --- Simulation bar ------------------------------------------------------------------

func _build_sim_bar(root: Control) -> void:
	var sim := editor.sim
	# Only the run's status stays at the bottom; its settings live in the
	# New run (Restart) and Settings dialogs (#26).
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_CENTER_BOTTOM)
	panel.grow_horizontal = Control.GROW_DIRECTION_BOTH
	panel.offset_top = -96
	panel.offset_bottom = -52
	panel.add_theme_stylebox_override("panel", _panel_style(PANEL_BG))
	root.add_child(panel)
	_build_transport(root)
	_sim_label = Label.new()
	_sim_label.custom_minimum_size = Vector2(460, 0)
	_sim_label.clip_text = true
	_sim_label.mouse_filter = Control.MOUSE_FILTER_PASS # for the tooltip
	_sim_label.add_theme_color_override("font_color", Color(0.8, 0.84, 0.9))
	panel.add_child(_sim_label)
	_build_run_dialog(root)
	_heat_legend = Label.new()
	_heat_legend.set_anchors_preset(Control.PRESET_CENTER_BOTTOM)
	_heat_legend.grow_horizontal = Control.GROW_DIRECTION_BOTH
	_heat_legend.offset_top = -124
	_heat_legend.offset_bottom = -100
	_heat_legend.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_heat_legend.add_theme_font_size_override("font_size", 13)
	_heat_legend.add_theme_color_override("font_color", Color(0.92, 0.94, 0.97))
	_heat_legend.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	_heat_legend.add_theme_constant_override("outline_size", 4)
	_heat_legend.visible = false
	root.add_child(_heat_legend)
	sim.state_changed.connect(_refresh_sim_buttons)
	_refresh_sim_buttons()


## "off", "speed", "wait" or "flow".
func set_heatmap(mode: String) -> void:
	editor.heat.set_mode(mode)
	var i := HeatLayer.MODES.find(editor.heat.mode)
	if _heat_opt and _heat_opt.selected != i:
		_heat_opt.select(i)
	_heat_legend.visible = editor.heat.mode != "off"
	_heat_legend.text = "%s heatmap · %s%s" % [HeatLayer.MODE_LABELS[i], HeatLayer.legend(editor.heat.mode),
		"" if editor.sim.has_cars() or editor.sim.playing else " · press Play to collect data"]


func cycle_heatmap() -> void:
	set_heatmap(HeatLayer.MODES[(HeatLayer.MODES.find(editor.heat.mode) + 1) % HeatLayer.MODES.size()])


## One dialog for the run's settings, shown two ways (#26):
## "new run" (from Restart) has the seed too and applies everything when the
## run restarts; "settings" leaves out the seed and applies each change at once.
func _build_run_dialog(root: Control) -> void:
	var sim := editor.sim
	_run_dialog = PanelContainer.new()
	var style := _panel_style(PANEL_BG)
	style.set_content_margin_all(14)
	_run_dialog.add_theme_stylebox_override("panel", style)
	_run_dialog.visible = false
	root.add_child(_run_dialog)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 8)
	_run_dialog.add_child(box)
	_run_title = Label.new()
	_run_title.add_theme_font_size_override("font_size", 18)
	box.add_child(_run_title)
	var grid := GridContainer.new()
	grid.columns = 2
	grid.add_theme_constant_override("h_separation", 12)
	grid.add_theme_constant_override("v_separation", 6)
	box.add_child(grid)
	_seed_label = Label.new()
	_seed_label.text = "Seed"
	grid.add_child(_seed_label)
	_seed = SpinBox.new()
	_seed.min_value = 0
	_seed.max_value = 999999
	_seed.custom_minimum_size = Vector2(140, 0)
	_seed.tooltip_text = "Same map + same seed = same run"
	grid.add_child(_seed)
	_demand_label = Label.new()
	_demand_label.custom_minimum_size = Vector2(110, 0)
	grid.add_child(_demand_label)
	_demand = HSlider.new()
	_demand.min_value = 0.0
	_demand.max_value = 3.0
	_demand.step = 0.05
	_demand.custom_minimum_size = Vector2(140, 0)
	_demand.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_demand.focus_mode = Control.FOCUS_NONE
	_demand.tooltip_text = "Multiplies every spawn point's rate"
	_demand.value_changed.connect(func(v: float) -> void:
		_demand_label.text = "Density ×%.2f" % v
		if _run_live:
			sim.set_demand(v))
	grid.add_child(_demand)
	var cap_label := Label.new()
	cap_label.text = "Max cars"
	grid.add_child(cap_label)
	_max_cars = SpinBox.new()
	_max_cars.min_value = 0
	_max_cars.max_value = 20000
	_max_cars.step = 50
	_max_cars.tooltip_text = "Spawning pauses while this many cars are on the map (0 = no limit)"
	_max_cars.value_changed.connect(func(v: float) -> void:
		if _run_live:
			sim.set_max_cars(int(v)))
	grid.add_child(_max_cars)
	var people_label := Label.new()
	people_label.text = "Max people"
	grid.add_child(people_label)
	_max_people = SpinBox.new()
	_max_people.min_value = 0
	_max_people.max_value = 20000
	_max_people.step = 100
	_max_people.tooltip_text = "New trips on foot pause while this many people are on the map (0 = no limit)"
	_max_people.value_changed.connect(func(v: float) -> void:
		if _run_live:
			sim.set_max_people(int(v)))
	grid.add_child(_max_people)
	var buttons := HBoxContainer.new()
	buttons.alignment = BoxContainer.ALIGNMENT_END
	buttons.add_theme_constant_override("separation", 8)
	box.add_child(buttons)
	_run_cancel = Button.new()
	_run_cancel.text = "Cancel"
	_run_cancel.focus_mode = Control.FOCUS_NONE
	_run_cancel.pressed.connect(close_run_dialog)
	buttons.add_child(_run_cancel)
	_run_ok = Button.new()
	_run_ok.focus_mode = Control.FOCUS_NONE
	_run_ok.pressed.connect(_confirm_run_dialog)
	buttons.add_child(_run_ok)


## Restart opens the New run dialog: set the seed, density and caps, then restart.
func open_new_run() -> void:
	_open_run_dialog(false)


## The settings button: change density and caps while the run goes on.
func open_settings() -> void:
	_open_run_dialog(true)


func _open_run_dialog(live: bool) -> void:
	var sim := editor.sim
	_run_live = false # loading the current values must not apply anything
	_seed.value = sim.seed_value
	_demand.value = sim.demand
	_demand_label.text = "Density ×%.2f" % sim.demand
	_max_cars.value = sim.max_cars
	_max_people.value = sim.max_people
	_run_live = live
	_run_title.text = "Settings" if live else "New run"
	_seed_label.visible = not live
	_seed.visible = not live
	_run_cancel.visible = not live
	_run_ok.text = "Close" if live else "Restart"
	_run_ok.tooltip_text = "" if live else "Remove every car and person and start again with these settings"
	_run_dialog.visible = true
	_run_dialog.reset_size()
	var vp := get_viewport().get_visible_rect().size
	_run_dialog.position = Vector2(vp.x * 0.5 - _run_dialog.size.x * 0.5, _transport.position.y + _transport.size.y + 8.0)


func close_run_dialog() -> void:
	_run_dialog.visible = false
	_run_live = false


func run_dialog_open() -> bool:
	return _run_dialog != null and _run_dialog.visible


func _confirm_run_dialog() -> void:
	var sim := editor.sim
	if not _run_live:
		sim.seed_value = int(_seed.value)
		sim.set_demand(_demand.value)
		sim.set_max_cars(int(_max_cars.value))
		sim.set_max_people(int(_max_people.value))
		sim.reset()
	close_run_dialog()


## Slower, the current speed, faster, then play / pause, step, restart and
## settings: icon buttons at the top centre; the text shows on hover (#16, #22, #26).
func _build_transport(root: Control) -> void:
	var sim := editor.sim
	_transport = PanelContainer.new()
	var style := _panel_style(PANEL_BG)
	style.set_content_margin_all(6)
	_transport.add_theme_stylebox_override("panel", style)
	root.add_child(_transport)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 6)
	_transport.add_child(row)
	_slower = _icon_button(ICON_SLOWER, "Slower", func() -> void: sim.set_speed_index(sim.speed_index - 1), row)
	_speed_label = Label.new()
	_speed_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_speed_label.custom_minimum_size = Vector2(48, 0)
	_speed_label.mouse_filter = Control.MOUSE_FILTER_PASS
	row.add_child(_speed_label)
	_faster = _icon_button(ICON_FASTER, "Faster", func() -> void: sim.set_speed_index(sim.speed_index + 1), row)
	row.add_child(VSeparator.new())
	_play = _icon_button(ICON_PLAY, "Play (Space)", sim.toggle, row)
	_icon_button(ICON_STEP, "Step: one sim second (.)", sim.step, row)
	_icon_button(ICON_RESTART, "Restart: set up a new run (seed, density, max cars and people)", open_new_run, row)
	_settings_btn = _icon_button(ICON_SETTINGS, "Settings: density, max cars and max people, without restarting", open_settings, row)


func _icon_button(icon: Texture2D, tip: String, action: Callable, parent: Control) -> Button:
	var b := Button.new()
	b.icon = icon
	b.tooltip_text = tip
	b.focus_mode = Control.FOCUS_NONE
	b.custom_minimum_size = Vector2(42, 36)
	b.icon_alignment = HORIZONTAL_ALIGNMENT_CENTER
	b.pressed.connect(action)
	parent.add_child(b)
	return b


func _refresh_sim_buttons() -> void:
	if _play == null:
		return
	_play.icon = ICON_PAUSE if editor.sim.playing else ICON_PLAY
	_play.tooltip_text = ("Pause" if editor.sim.playing else "Play") + " (Space). Editing pauses; Play resumes with the changes."
	var sim := editor.sim
	_speed_label.text = sim.speed_label(sim.speed_index)
	_speed_label.tooltip_text = "Sim speed: %dx real time" % int(SimController.BASE_SPEED * SimController.MULTIPLIERS[sim.speed_index])
	_slower.disabled = sim.speed_index == 0
	_faster.disabled = sim.speed_index == SimController.MULTIPLIERS.size() - 1


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
	var cs: Dictionary = editor.sim.city
	if not cs.is_empty():
		_market_button.text = "Market (%d)" % int(cs.listed)
	_market_time += 1.0
	if _market_time >= 10.0:
		_market_time = 0.0
		_refresh_market()
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
		_sim_label.tooltip_text += "\nMonth %d, day %d · city income %s, spending %s this month (%s / %s in all): rent %s, sales %s, passes and fares %s, buildings sold %s; wages %s, goods %s, buildings bought %s\nOwnership: %d city, %d NPC, %d listed, %d homes owned by residents · %d sold, %d bought · %d evictions, %d households in debt · %d bikes, %d cars, %d bus passes" % [
			int(c.month) + 1, int(c.day_of_month) + 1, Inspector.money(c.month_income), Inspector.money(c.month_spending),
			Inspector.money(c.treasury_income), Inspector.money(c.treasury_spending), Inspector.money(c.income_rent),
			Inspector.money(c.income_sales), Inspector.money(c.income_passes), Inspector.money(c.income_buildings),
			Inspector.money(c.spending_wages), Inspector.money(c.spending_goods), Inspector.money(c.spending_buildings),
			c.city_owned, c.npc_owned, c.listed, c.homes_owned, c.buildings_sold, c.buildings_bought, c.evictions, c.in_debt,
			c.bikes, c.cars, c.passes]
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


func _build_market(root: Control) -> void:
	_market_panel = PanelContainer.new()
	_market_panel.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	_market_panel.offset_left = -520
	_market_panel.offset_top = -344
	_market_panel.offset_right = -12
	_market_panel.offset_bottom = -100
	_market_panel.add_theme_stylebox_override("panel", _panel_style(PANEL_BG))
	_market_panel.visible = false
	root.add_child(_market_panel)
	var box := VBoxContainer.new()
	_market_panel.add_child(box)
	box.add_child(section("Market"))
	_market_list = ItemList.new()
	_market_list.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_market_list.focus_mode = Control.FOCUS_NONE
	_market_list.item_selected.connect(func(i: int) -> void:
		if i >= 0 and i < _market.size():
			var l: Dictionary = _market[i]
			if l.has("pos"):
				editor.focus(l.pos)
			editor.select("buildings", int(l.id), false))
	box.add_child(_market_list)
	var hint := Label.new()
	hint.text = "Click a listing to select it; buy it from the inspector."
	hint.add_theme_font_size_override("font_size", 12)
	hint.add_theme_color_override("font_color", Color(0.7, 0.72, 0.76))
	box.add_child(hint)


func _refresh_market() -> void:
	if _market_panel == null or not _market_panel.visible:
		return
	_market = editor.road.sim_market()
	_market_list.clear()
	if _market.is_empty():
		_market_list.add_item("Nothing for sale (press Play to run the city)")
		_market_list.set_item_disabled(0, true)
		_market = []
		return
	for l in _market:
		var name := String(l.label) + ((" · " + String(l.name)) if String(l.name) != "" else " %d" % l.id)
		_market_list.add_item("%s%s · asking %s (worth %s) · listed %d day%s" % [name, " (yours)" if l.by_city else "",
			Inspector.money(l.asking), Inspector.money(l.value), l.days, "" if int(l.days) == 1 else "s"])


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


## First run (no autosave yet): how to start.
func show_welcome() -> void:
	var d := ConfirmationDialog.new()
	d.title = "Welcome to Traffic Sim"
	d.dialog_text = "Build a town: roads, homes, shops and offices, buses and bike lanes, then press Play and watch people live in it.\n\nThe tutorial starts a new city and shows you each step. You can open it again from the left panel (Tutorial) at any time."
	d.dialog_autowrap = true
	d.min_size = Vector2i(460, 0)
	d.ok_button_text = "Start the tutorial"
	d.cancel_button_text = "Explore on my own"
	d.add_button("Open an example town", true, "example")
	d.confirmed.connect(func() -> void:
		editor.start_tutorial()
		d.queue_free())
	d.canceled.connect(d.queue_free)
	d.custom_action.connect(func(action: StringName) -> void:
		if action == &"example":
			editor.load_demo("city_town")
			editor.notify("City town: press Space to play, then click a home, a shop or a person.")
		d.queue_free())
	add_child(d)
	d.popup_centered()


func toast(text: String) -> void:
	if _toast == null:
		return
	_toast.text = text
	_toast.visible = true
	_toast_time = 3.5
