class_name RingHud
extends CanvasLayer
## Side panel with live measurements and POC controls. Built in code.

const CAR_COUNTS := [1000, 2000, 5000]
const SPEEDS := [1.0, 4.0, 16.0, 64.0, 128.0]
const STATS_INTERVAL := 0.25

var main: RingDemo

var _stats: Label
var _message: Label
var _count_buttons: Array[Button] = []
var _speed_buttons: Array[Button] = []
var _pause_button: Button
var _seed: SpinBox
var _since_update := 0.0


func _ready() -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(12, 12)
	panel.custom_minimum_size = Vector2(340, 0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.08, 0.09, 0.1, 0.86)
	style.set_corner_radius_all(8)
	style.set_content_margin_all(12)
	panel.add_theme_stylebox_override("panel", style)
	add_child(panel)

	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 8)
	panel.add_child(box)

	var title := Label.new()
	title.text = "POC ring benchmark"
	title.add_theme_font_size_override("font_size", 18)
	box.add_child(title)

	_stats = Label.new()
	_stats.add_theme_font_size_override("font_size", 13)
	var mono := SystemFont.new()
	mono.font_names = PackedStringArray(["Menlo", "Consolas", "DejaVu Sans Mono", "Liberation Mono", "monospace"])
	_stats.add_theme_font_override("font", mono)
	box.add_child(_stats)

	box.add_child(_section("Cars"))
	var counts := _row(box)
	for c in CAR_COUNTS:
		var b := _button(counts, "%dk" % (c / 1000), main.set_car_count.bind(c))
		b.toggle_mode = true
		_count_buttons.append(b)

	box.add_child(_section("Speed"))
	var speeds := _row(box)
	_pause_button = _button(speeds, "Pause", main.toggle_pause)
	for s in SPEEDS:
		var b := _button(speeds, "%dx" % s, main.set_speed.bind(s))
		b.toggle_mode = true
		_speed_buttons.append(b)

	var misc := _row(box)
	_button(misc, "Step", main.step_once)
	_button(misc, "Respawn", main.respawn)
	var seed_label := Label.new()
	seed_label.text = "Seed"
	misc.add_child(seed_label)
	_seed = SpinBox.new()
	_seed.min_value = 0
	_seed.max_value = 999999
	_seed.value = main.seed_value
	_seed.value_changed.connect(func(v: float) -> void: main.seed_value = int(v))
	misc.add_child(_seed)

	box.add_child(_section("Scene"))
	var scene_row := _row(box)
	_button(scene_row, "Back to editor", func() -> void: main.get_tree().change_scene_to_file("res://main.tscn"))

	box.add_child(_section("Checks"))
	var checks := _row(box)
	_button(checks, "Determinism", main.run_determinism_check)
	_button(checks, "Benchmark", func() -> void: main.bench.start(false, false))

	_message = Label.new()
	_message.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_message.custom_minimum_size = Vector2(316, 0)
	_message.add_theme_color_override("font_color", Color(0.75, 0.85, 1.0))
	box.add_child(_message)

	var help := Label.new()
	help.text = "Space pause · . step · 1-5 speed · F fit\nwheel zoom · right-drag pan · Esc back to editor"
	help.add_theme_font_size_override("font_size", 11)
	help.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	box.add_child(help)

	refresh_controls()


func _section(text: String) -> Label:
	var l := Label.new()
	l.text = text.to_upper()
	l.add_theme_font_size_override("font_size", 11)
	l.add_theme_color_override("font_color", Color(0.6, 0.62, 0.65))
	return l


func _row(parent: Container) -> HBoxContainer:
	var r := HBoxContainer.new()
	r.add_theme_constant_override("separation", 6)
	parent.add_child(r)
	return r


func _button(parent: Container, text: String, action: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.focus_mode = Control.FOCUS_NONE # keep Space for pause
	b.pressed.connect(func() -> void:
		action.call()
		refresh_controls())
	parent.add_child(b)
	return b


func refresh_controls() -> void:
	for i in _count_buttons.size():
		_count_buttons[i].set_pressed_no_signal(CAR_COUNTS[i] == main.car_count)
	for i in _speed_buttons.size():
		_speed_buttons[i].set_pressed_no_signal(not main.paused and is_equal_approx(SPEEDS[i], main.speed))
	_pause_button.text = "Play" if main.paused else "Pause"


func show_message(text: String) -> void:
	if _message:
		_message.text = text


func _process(delta: float) -> void:
	_since_update += delta
	if _since_update < STATS_INTERVAL or main.sim == null:
		return
	_since_update = 0.0
	var s: Dictionary = main.sim.get_stats()
	var f: Dictionary = main.frame_stats()
	var t: float = s.sim_time
	var clock := "%02d:%02d:%02d" % [int(t / 3600.0), int(fmod(t, 3600.0) / 60.0), int(fmod(t, 60.0))]
	var requested := "paused" if main.paused else "%dx" % main.speed
	_stats.text = "\n".join([
		"fps        %6.1f   frame %5.2f ms" % [f.fps, f.avg_ms],
		"frame p95  %6.2f ms max %5.1f ms" % [f.p95_ms, f.max_ms],
		"sim/frame  %6.2f ms  %5.1f ticks" % [s.avg_frame_sim_ms, s.avg_frame_ticks],
		"tick       %6.1f us  draw %4.0f us" % [s.tick_us, s.render_prep_us],
		"speed      %6.1fx  of %s%s" % [s.effective_speed, requested, "  (budget hit)" if s.behind else ""],
		"cars       %6d   stopped %4.1f%%" % [s.vehicles, 100.0 * s.stopped / maxf(1.0, s.vehicles)],
		"mean       %6.1f km/h  sd %4.1f" % [s.mean_speed_kmh, s.speed_stddev_kmh],
		"sim clock  %s  tick %d" % [clock, s.tick],
		"state      %s" % main.sim.get_state_hash(),
	])
