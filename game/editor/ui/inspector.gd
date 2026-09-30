class_name Inspector
extends PanelContainer
## Right-hand inspector for the selected road, node or car. Widgets are built
## once and filled without firing signals, so editing a field never loses focus.

const RULES := ["disallowed", "allowed", "turn_lane"]
const RULE_LABELS := ["No turn", "Allowed", "Turn lane"]
const MEDIANS := ["none", "painted", "raised"]
const CONTROLS := ["right_hand", "priority_road", "all_way_stop"]
const CONTROL_LABELS := ["Right-hand priority", "Priority road", "All-way stop"]

var editor: MapEditor

var _updating := false
var _seg := 0
var _node := 0
var _params := {}
var _profile := {}

var _title: Label
var _empty: Label
var _segment_box: VBoxContainer
var _node_box: VBoxContainer
var _multi_box: VBoxContainer
var _multi_label: Label

# Segment widgets
var _name: LineEdit
var _speed: SpinBox
var _level: OptionButton
var _preset: OptionButton
var _forward: SpinBox
var _backward: SpinBox
var _lane_width: SpinBox
var _flip: Button
var _median: OptionButton
var _median_width: SpinBox
var _checks := {}
var _lanes_label: Label
var _curve_label: Label
var _end_rows: Array = [] # [{box, title, left, right, length}]

# Node widgets
var _node_info: Label
var _control_box: VBoxContainer
var _control: OptionButton
var _legs_box: VBoxContainer
var _legs_hint: Label
var _spawn_box: VBoxContainer
var _spawn_on: CheckBox
var _spawn_rate: SpinBox
var _spawn_sink: CheckBox
var _od_box: GridContainer
var _od_scroll: ScrollContainer
var _od_hint: Label

# Car widgets
var _car_box: VBoxContainer
var _car_info: Label


func _ready() -> void:
	custom_minimum_size = Vector2(300, 0)
	add_theme_stylebox_override("panel", EditorUI._panel_style(EditorUI.PANEL_BG))
	var outer := VBoxContainer.new()
	outer.add_theme_constant_override("separation", 6)
	add_child(outer)
	_title = Label.new()
	_title.add_theme_font_size_override("font_size", 16)
	outer.add_child(_title)
	_empty = Label.new()
	_empty.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_empty.custom_minimum_size = Vector2(280, 0)
	_empty.add_theme_color_override("font_color", Color(0.7, 0.72, 0.76))
	outer.add_child(_empty)
	_build_segment(outer)
	_build_node(outer)
	_build_car(outer)
	_multi_box = VBoxContainer.new()
	_multi_label = Label.new()
	_multi_box.add_child(_multi_label)
	_delete_button(_multi_box)
	outer.add_child(_multi_box)
	refresh()


# --- building -----------------------------------------------------------------------

func _row(parent: Control, label: String) -> HBoxContainer:
	var r := HBoxContainer.new()
	var l := Label.new()
	l.text = label
	l.custom_minimum_size = Vector2(110, 0)
	r.add_child(l)
	parent.add_child(r)
	return r


func _spin(parent: Control, lo: float, hi: float, step: float, suffix: String, cb: Callable) -> SpinBox:
	var s := SpinBox.new()
	s.min_value = lo
	s.max_value = hi
	s.step = step
	s.suffix = suffix
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	s.value_changed.connect(func(v: float) -> void:
		if not _updating:
			cb.call(v))
	parent.add_child(s)
	return s


func _option(parent: Control, items: Array, cb: Callable) -> OptionButton:
	var o := OptionButton.new()
	o.focus_mode = Control.FOCUS_NONE
	for it in items:
		o.add_item(str(it))
	o.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	o.item_selected.connect(func(i: int) -> void:
		if not _updating:
			cb.call(i))
	parent.add_child(o)
	return o


func _check(parent: Control, text: String, key: String) -> void:
	var c := CheckBox.new()
	c.text = text
	c.focus_mode = Control.FOCUS_NONE
	c.toggled.connect(func(on: bool) -> void:
		if not _updating:
			_set_param(key, on))
	parent.add_child(c)
	_checks[key] = c


func _delete_button(parent: Control) -> void:
	var b := Button.new()
	b.text = "Delete"
	b.focus_mode = Control.FOCUS_NONE
	b.pressed.connect(func() -> void: editor.delete_selection())
	parent.add_child(b)


func _build_segment(outer: VBoxContainer) -> void:
	_segment_box = VBoxContainer.new()
	_segment_box.add_theme_constant_override("separation", 4)
	outer.add_child(_segment_box)
	var box := _segment_box

	var r := _row(box, "Name")
	_name = LineEdit.new()
	_name.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_name.placeholder_text = "unnamed"
	_name.text_submitted.connect(func(t: String) -> void: editor.road.set_segment_name(_seg, t))
	_name.focus_exited.connect(func() -> void:
		if _seg != 0 and _name.text != String(editor.road.get_segment(_seg).get("name", "")):
			editor.road.set_segment_name(_seg, _name.text))
	r.add_child(_name)
	_speed = _spin(_row(box, "Speed limit"), 10, 130, 5, "km/h", func(v: float) -> void: editor.road.set_speed_kmh(_seg, v))
	_level = _option(_row(box, "Level"), ["Underpass (−1)", "Ground (0)", "Overpass (+1)"],
		func(i: int) -> void: editor.road.set_level(_seg, i - 1))
	_curve_label = Label.new()
	_curve_label.add_theme_color_override("font_color", Color(0.7, 0.72, 0.76))
	box.add_child(_curve_label)

	box.add_child(EditorUI.section("Profile"))
	var pr := HBoxContainer.new()
	_preset = OptionButton.new()
	_preset.focus_mode = Control.FOCUS_NONE
	_preset.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_preset.item_selected.connect(_apply_preset)
	pr.add_child(_preset)
	var save := Button.new()
	save.text = "Save as…"
	save.focus_mode = Control.FOCUS_NONE
	save.tooltip_text = "Save this road's cross-section as a profile for new roads"
	save.pressed.connect(_save_preset)
	pr.add_child(save)
	box.add_child(pr)
	_forward = _spin(_row(box, "Lanes →"), 0, 6, 1, "", func(v: float) -> void: _set_param("forward", int(v)))
	_backward = _spin(_row(box, "Lanes ←"), 0, 4, 1, "", func(v: float) -> void: _set_param("backward", int(v)))
	_lane_width = _spin(_row(box, "Lane width"), 2.5, 5.0, 0.05, "m", func(v: float) -> void: _set_param("lane_width", v))
	_lane_width.custom_arrow_step = 0.25
	_flip = Button.new()
	_flip.text = "Flip one-way direction"
	_flip.focus_mode = Control.FOCUS_NONE
	_flip.pressed.connect(func() -> void: editor.road.flip(_seg))
	box.add_child(_flip)
	var mr := _row(box, "Median")
	_median = _option(mr, ["None", "Painted", "Raised"], func(i: int) -> void:
		var p := _params.duplicate()
		p["median"] = MEDIANS[i]
		if i > 0 and float(p.get("median_width", 0.0)) < 0.5:
			p["median_width"] = 1.0 if i == 1 else 2.0
		_apply_params(p))
	_median_width = _spin(mr, 0.5, 10, 0.5, "m", func(v: float) -> void: _set_param("median_width", v))
	var grid := GridContainer.new()
	grid.columns = 2
	_check(grid, "Sidewalk left", "sidewalk_left")
	_check(grid, "Sidewalk right", "sidewalk_right")
	_check(grid, "Parking left", "parking_left")
	_check(grid, "Parking right", "parking_right")
	_check(grid, "Bike lane left", "bike_left")
	_check(grid, "Bike lane right", "bike_right")
	_check(grid, "Bus lane left", "bus_left")
	_check(grid, "Bus lane right", "bus_right")
	box.add_child(grid)
	_lanes_label = Label.new()
	_lanes_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_lanes_label.custom_minimum_size = Vector2(280, 0)
	_lanes_label.add_theme_font_size_override("font_size", 12)
	_lanes_label.add_theme_color_override("font_color", Color(0.75, 0.78, 0.82))
	box.add_child(_lanes_label)

	box.add_child(EditorUI.section("Turn rules at junctions"))
	for e in 2:
		var eb := VBoxContainer.new()
		var t := Label.new()
		t.add_theme_color_override("font_color", Color(0.75, 0.78, 0.82))
		eb.add_child(t)
		var lr := _row(eb, "Left turn")
		var left := _option(lr, RULE_LABELS, func(_i: int) -> void: _set_rules(e))
		var rr := _row(eb, "Right turn")
		var right := _option(rr, RULE_LABELS, func(_i: int) -> void: _set_rules(e))
		var len_row := _row(eb, "Turn lane")
		var length := _spin(len_row, 10, 200, 5, "m", func(_v: float) -> void: _set_rules(e))
		box.add_child(eb)
		_end_rows.append({"box": eb, "title": t, "left": left, "right": right, "length": length, "length_row": len_row})
	_delete_button(box)


func _build_node(outer: VBoxContainer) -> void:
	_node_box = VBoxContainer.new()
	_node_box.add_theme_constant_override("separation", 4)
	outer.add_child(_node_box)
	_node_info = Label.new()
	_node_info.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_node_info.custom_minimum_size = Vector2(280, 0)
	_node_box.add_child(_node_info)

	_control_box = VBoxContainer.new()
	_control_box.add_child(EditorUI.section("Junction control"))
	_control = _option(_control_box, CONTROL_LABELS, func(_i: int) -> void: _set_control())
	_legs_hint = Label.new()
	_legs_hint.text = "Main road (has right of way):"
	_legs_hint.add_theme_color_override("font_color", Color(0.75, 0.78, 0.82))
	_control_box.add_child(_legs_hint)
	_legs_box = VBoxContainer.new()
	_control_box.add_child(_legs_box)
	_node_box.add_child(_control_box)

	_spawn_box = VBoxContainer.new()
	_spawn_box.add_child(EditorUI.section("Spawn point"))
	_spawn_on = CheckBox.new()
	_spawn_on.text = "Traffic enters and leaves the map here"
	_spawn_on.focus_mode = Control.FOCUS_NONE
	_spawn_on.toggled.connect(func(_on: bool) -> void:
		if not _updating:
			_set_spawner())
	_spawn_box.add_child(_spawn_on)
	_spawn_rate = _spin(_row(_spawn_box, "Cars in"), 0, 5000, 50, "/h", func(_v: float) -> void: _set_spawner())
	_spawn_sink = CheckBox.new()
	_spawn_sink.text = "Cars may leave the map here"
	_spawn_sink.focus_mode = Control.FOCUS_NONE
	_spawn_sink.toggled.connect(func(_on: bool) -> void:
		if not _updating:
			_set_spawner())
	_spawn_box.add_child(_spawn_sink)
	_od_hint = Label.new()
	_od_hint.text = "Share of trips to each destination:"
	_od_hint.add_theme_color_override("font_color", Color(0.75, 0.78, 0.82))
	_spawn_box.add_child(_od_hint)
	_od_scroll = ScrollContainer.new()
	_od_scroll.custom_minimum_size = Vector2(280, 0)
	_od_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	_od_box = GridContainer.new()
	_od_box.columns = 2
	_od_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_od_scroll.add_child(_od_box)
	_spawn_box.add_child(_od_scroll)
	_node_box.add_child(_spawn_box)
	_delete_button(_node_box)


func _build_car(outer: VBoxContainer) -> void:
	_car_box = VBoxContainer.new()
	outer.add_child(_car_box)
	_car_info = Label.new()
	_car_info.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_car_info.custom_minimum_size = Vector2(280, 0)
	_car_box.add_child(_car_info)
	var hint := Label.new()
	hint.text = "The route is drawn on the map. Esc deselects."
	hint.add_theme_font_size_override("font_size", 11)
	hint.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	_car_box.add_child(hint)


# --- refresh --------------------------------------------------------------------------

func refresh() -> void:
	if editor == null or editor.road == null or _segment_box == null:
		return
	var sel := editor.selection
	var n_seg: int = sel.segments.size()
	var n_node: int = sel.nodes.size()
	var car: bool = editor.sim != null and editor.sim.selected_car != 0 and n_seg + n_node == 0
	_car_box.visible = car
	_segment_box.visible = n_seg == 1 and n_node == 0
	_node_box.visible = n_node == 1 and n_seg == 0
	_multi_box.visible = n_seg + n_node > 1
	_empty.visible = n_seg + n_node == 0 and not car
	if car:
		refresh_car()
	elif _empty.visible:
		_seg = 0
		_node = 0
		var st: Dictionary = editor.road.get_stats()
		_title.text = "Map"
		_empty.text = "%d roads · %d junctions · %d lanes · %d spawn points\nGeometry %.1f ms · %d vertices\n\nSelect a road or node to edit it. Press R to draw a road, C for a curve, L to paint lanes, N to add spawn points. Press Space to run the traffic and click a car to follow it." % [
			st.segments, st.junctions, st.lanes, editor.road.get_spawners().size(), st.build_ms, st.vertices]
	elif _multi_box.visible:
		_title.text = "Selection"
		_multi_label.text = "%d road%s, %d node%s" % [n_seg, "" if n_seg == 1 else "s", n_node, "" if n_node == 1 else "s"]
	elif _node_box.visible:
		_fill_node(sel.nodes[0])
	else:
		_fill_segment(sel.segments[0])
	reset_size()


func _fill_node(id: int) -> void:
	_node = id
	var n: Dictionary = editor.road.get_node(id)
	if n.is_empty():
		return
	_updating = true
	_title.text = "Node %d" % id
	_node_info.text = "%s · level %+d\nAt %.1f, %.1f m\n%d road%s · %d connector%s" % [
		String(n.kind).capitalize(), n.level, n.pos.x, n.pos.y, n.segments.size(),
		"" if n.segments.size() == 1 else "s", n.connectors, "" if n.connectors == 1 else "s"]
	# Junction control and the main-road legs.
	_control_box.visible = n.junction
	if n.junction:
		_control.select(CONTROLS.find(n.control))
		var priority: bool = n.control == "priority_road"
		_legs_hint.visible = priority
		_legs_box.visible = priority
		for c in _legs_box.get_children():
			c.queue_free()
		if priority:
			for leg in n.legs:
				var cb := CheckBox.new()
				cb.focus_mode = Control.FOCUS_NONE
				var name_text := String(leg.name) if String(leg.name) != "" else "Road %d" % leg.segment
				cb.text = "%s (%s)" % [name_text, _compass(leg.dir)]
				cb.button_pressed = leg.priority
				cb.set_meta("segment", leg.segment)
				cb.toggled.connect(func(_on: bool) -> void:
					if not _updating:
						_set_control())
				_legs_box.add_child(cb)
	# Spawn point.
	var sp: Dictionary = n.spawner
	_spawn_box.visible = n.road_end or sp.enabled
	_spawn_on.button_pressed = sp.enabled
	_spawn_rate.value = sp.rate
	_spawn_rate.editable = sp.enabled
	_spawn_sink.button_pressed = sp.sink
	_spawn_sink.disabled = not sp.enabled
	for c in _od_box.get_children():
		c.queue_free()
	var others: Array = []
	if sp.enabled:
		for o in editor.road.get_spawners():
			if int(o.id) != id and o.sink:
				others.append(o)
	_od_hint.visible = not others.is_empty()
	_od_scroll.visible = not others.is_empty()
	_od_scroll.custom_minimum_size = Vector2(280, minf(others.size() * 34.0, 170.0))
	var weights := {}
	for w in sp.od:
		weights[int(w.to)] = float(w.weight)
	for o in others:
		var l := Label.new()
		l.text = "To node %d" % o.id
		l.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_od_box.add_child(l)
		var w := SpinBox.new()
		w.min_value = 0
		w.max_value = 20
		w.step = 0.5
		w.value = weights.get(int(o.id), 1.0)
		w.set_meta("to", int(o.id))
		w.value_changed.connect(func(_v: float) -> void:
			if not _updating:
				_set_spawner())
		_od_box.add_child(w)
	_updating = false


static func _compass(dir: Vector2) -> String:
	# Screen y points down, so north is -y.
	var names := ["east", "south-east", "south", "south-west", "west", "north-west", "north", "north-east"]
	var i := int(roundf(fposmod(dir.angle(), TAU) / (TAU / 8.0))) % 8
	return names[i]


func _set_control() -> void:
	var prio := PackedInt64Array()
	for cb in _legs_box.get_children():
		if cb is CheckBox and cb.button_pressed and not cb.is_queued_for_deletion():
			prio.append(int(cb.get_meta("segment")))
	var control: String = CONTROLS[_control.selected]
	if control == "priority_road" and prio.is_empty():
		# Default main road: the two legs closest to a straight line.
		var legs: Array = editor.road.get_node(_node).legs
		var best := -2.0
		var pair := []
		for i in legs.size():
			for j in range(i + 1, legs.size()):
				var straightness: float = -(legs[i].dir as Vector2).dot(legs[j].dir)
				if straightness > best:
					best = straightness
					pair = [legs[i].segment, legs[j].segment]
		prio = PackedInt64Array(pair)
	editor.road.set_junction_control(_node, control, prio)


func _set_spawner() -> void:
	var od: Array = []
	for w in _od_box.get_children():
		if w is SpinBox and not w.is_queued_for_deletion():
			od.append({"to": int(w.get_meta("to")), "weight": w.value})
	editor.road.set_spawner(_node, {
		"enabled": _spawn_on.button_pressed,
		"rate": _spawn_rate.value,
		"sink": _spawn_sink.button_pressed,
		"od": od,
	})


func refresh_car() -> void:
	var c: Dictionary = editor.sim.car_info
	if c.is_empty():
		return
	_title.text = "Car %d" % c.id
	var where := "in junction" if c.in_junction else ("on " + (String(c.road) if String(c.road) != "" else "road %d" % c.segment))
	var lines := [
		"%s, %s" % [String(c.state).capitalize(), where],
		"Speed %.0f km/h (wants %.0f) · accel %+.1f m/s²" % [c.speed_kmh, c.desired_kmh, c.accel],
		"From node %d to node %d · %d turn%s left" % [c.origin, c.dest, c.turns_left, "" if int(c.turns_left) == 1 else "s"],
		"Trip %s · %.0f m driven" % [EditorUI.clock(c.trip_time), c.distance],
		"Critical gap %.1f s" % c.critical_gap,
	]
	if float(c.stopped_for) > 0.5:
		lines.append("Stopped for %.0f s" % c.stopped_for)
	if int(c.blocker) != 0:
		lines.append("Waiting for car %d" % c.blocker)
	_car_info.text = "\n".join(lines)


func _fill_segment(id: int) -> void:
	_seg = id
	var s: Dictionary = editor.road.get_segment(id)
	if s.is_empty():
		return
	_updating = true
	_params = s.params
	_profile = s.profile
	_title.text = "Road %d%s" % [id, (" · " + String(s.name)) if String(s.name) != "" else ""]
	if not _name.has_focus():
		_name.text = s.name
	_speed.value = roundf(float(s.speed_kmh))
	_level.select(int(s.level) + 1)
	_curve_label.text = "%s · %.0f m · %s" % [String(s.curve).capitalize(), s.length, "one-way" if s.one_way else "two-way"]
	_refresh_preset_items()
	_forward.value = _params.forward
	_backward.value = _params.backward
	_lane_width.value = _params.lane_width
	_flip.disabled = not s.one_way
	_median.select(MEDIANS.find(_params.median))
	_median_width.value = maxf(0.5, float(_params.median_width))
	_median_width.editable = _params.median != "none"
	for k in _checks:
		_checks[k].set_pressed_no_signal(bool(_params.get(k, false)))
	var parts: Array = []
	for l in s.profile.lanes:
		var arrow := " →" if l.dir == "forward" else (" ←" if l.dir == "backward" else "")
		parts.append("%s%s %s m" % [String(l.type), arrow, str(snappedf(float(l.width), 0.01))])
	_lanes_label.text = "Left to right: " + " | ".join(parts)
	for e in 2:
		var row: Dictionary = _end_rows[e]
		var end: Dictionary = s.ends[e]
		var show: bool = end.junction and int(end.incoming) > 0
		row.box.visible = show
		if show:
			row.title.text = "Arriving at junction %d (%d lane%s)" % [end.node, end.incoming, "" if int(end.incoming) == 1 else "s"]
			row.left.select(RULES.find(end.left))
			row.right.select(RULES.find(end.right))
			row.length.value = end.turn_lane_length
			row.length_row.visible = end.left == "turn_lane" or end.right == "turn_lane"
	_updating = false


func _refresh_preset_items() -> void:
	_preset.clear()
	_preset.add_item("Apply a profile…")
	for p in editor.road.presets():
		_preset.add_item(p.name)
	for p in editor.user_presets:
		_preset.add_item("★ " + String(p.name))
	_preset.select(0)


# --- edits --------------------------------------------------------------------------------

func _set_param(key: String, value) -> void:
	var p := _params.duplicate()
	p[key] = value
	_apply_params(p)


func _apply_params(p: Dictionary) -> void:
	var err: String = editor.road.set_profile_params(_seg, p)
	if err != "":
		editor.notify("Can't change lanes: %s" % err)


func _apply_preset(i: int) -> void:
	if _updating or i == 0:
		return
	var presets: Array = editor.road.presets()
	if i - 1 < presets.size():
		_apply_params(presets[i - 1].params)
	else:
		var up: Dictionary = editor.user_presets[i - 1 - presets.size()]
		var err: String = editor.road.set_profile(_seg, up.profile)
		if err != "":
			editor.notify("Can't apply profile: %s" % err)


func _set_rules(e: int) -> void:
	var row: Dictionary = _end_rows[e]
	editor.road.set_end_rules(_seg, e, {
		"left": RULES[row.left.selected],
		"right": RULES[row.right.selected],
		"turn_lane_length": row.length.value,
	})


func _save_preset() -> void:
	var d := AcceptDialog.new()
	d.title = "Save profile"
	var edit := LineEdit.new()
	edit.placeholder_text = "Profile name"
	edit.custom_minimum_size = Vector2(260, 0)
	d.add_child(edit)
	d.register_text_enter(edit)
	d.confirmed.connect(func() -> void:
		var n := edit.text.strip_edges()
		if n != "":
			editor.save_user_preset(n, _profile)
		d.queue_free())
	d.canceled.connect(d.queue_free)
	add_child(d)
	d.popup_centered()
	edit.grab_focus()
