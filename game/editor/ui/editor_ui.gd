class_name EditorUI
extends CanvasLayer
## Editor chrome: tool palette (left), inspector (right), bottom bar with
## history, level, snapping and status, and the problems panel.

const TOOLS := [
	["select", "Select", "V"],
	["road", "Road", "R"],
	["curve", "Curve road", "C"],
	["lane", "Lane paint", "L"],
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
var _status: Label
var _cursor: Label
var _problems_button: Button
var _autosave: Label
var _problems_panel: PanelContainer
var _problems_list: ItemList
var _problems: Array = []
var _toast: Label
var _toast_time := 0.0


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
	demos.text = "Examples ▾"
	demos.flat = false
	demos.focus_mode = Control.FOCUS_NONE
	demos.get_popup().add_item("Demo town", 0)
	demos.get_popup().add_item("Test grid (56 junctions)", 1)
	demos.get_popup().add_separator()
	demos.get_popup().add_item("POC ring benchmark", 2)
	demos.get_popup().id_pressed.connect(_on_example)
	box.add_child(demos)

	var help := Label.new()
	help.text = "Wheel zoom · right-drag pan · F fit\nCtrl+Z undo · Ctrl+Shift+Z redo\nG grid · A angles · PgUp/PgDn level"
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


func _confirm_new() -> void:
	var d := ConfirmationDialog.new()
	d.dialog_text = "Start an empty map? The current map stays in the autosave until the next save, and undo history is cleared."
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
		_preset.add_item("★ " + String(p.name))
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
	_undo = _button("↶ Undo", editor.undo, bar)
	_redo = _button("↷ Redo", editor.redo, bar)
	bar.add_child(VSeparator.new())
	var ll := Label.new()
	ll.text = "Level"
	bar.add_child(ll)
	for l in [-1, 0, 1]:
		var b := _button(["−1", "0", "+1"][l + 1], editor.set_level.bind(l), bar)
		b.toggle_mode = true
		b.tooltip_text = ["Underpass (level −1)", "Ground (level 0)", "Overpass (level +1)"][l + 1]
		_levels.append(b)
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


func _build_problems(root: Control) -> void:
	_problems_panel = PanelContainer.new()
	_problems_panel.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	_problems_panel.offset_left = -520
	_problems_panel.offset_top = -300
	_problems_panel.offset_right = -12
	_problems_panel.offset_bottom = -52
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
	if not p.segments.is_empty():
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
	_problems = road.get_problems()
	var errors := 0
	_problems_list.clear()
	for p in _problems:
		if p.severity == "error":
			errors += 1
		var icon := "⛔ " if p.severity == "error" else "⚠ "
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
