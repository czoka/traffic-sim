class_name BridgeTool
extends EditorTool
## B: click a road where it crosses another to lift it onto a bridge (level
## +1), with ramps at the grade limit (6 % for roads, 8 % for paths) on both
## sides. T switches to a tunnel (level -1). The preview shows where the
## ramps start and end.

var tunnel := false
var _hover := {}
var _plan := {}


func hint() -> String:
	return "Click a road where it crosses another to build a %s · T: %s" % [
		"tunnel" if tunnel else "bridge", "bridge instead" if tunnel else "tunnel instead"]


func activate() -> void:
	_hover = {}
	_plan = {}


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_update(mouse)
		return false
	if key_pressed(event, KEY_T):
		tunnel = not tunnel
		_update(mouse)
		editor.ui.set_status(hint())
		return true
	if key_pressed(event, KEY_ESCAPE):
		editor.set_tool("select")
		return true
	if is_left_press(event):
		_update(mouse)
		if _hover.is_empty():
			editor.notify("Click the road that should go over (or under) another one.")
			return true
		var err: String = editor.road.lift(_hover.id, mouse, -1 if tunnel else 1)
		if err != "":
			editor.notify("Can't build it here: %s" % err)
		else:
			editor.notify("%s built. Ramps run at most %d %%." % ["Tunnel" if tunnel else "Bridge", int(round(_grade() * 100.0))])
			editor.clear_selection()
		_plan = {}
		return true
	return false


func _grade() -> float:
	if _hover.is_empty():
		return 0.06
	return float(editor.road.get_segment(_hover.id).max_grade)


func _update(mouse: Vector2) -> void:
	_hover = editor.road.pick(mouse, editor.pick_radius(), editor.level)
	if _hover.type != "segment":
		_hover = {}
		_plan = {}
		return
	_plan = editor.road.plan_lift(_hover.id, mouse, -1 if tunnel else 1)
	if _plan.ok:
		var s: PackedFloat32Array = _plan.stations
		editor.ui.set_status("%s: ramps of %.0f m and %.0f m, %.0f m %s · %s" % [
			"Tunnel" if tunnel else "Bridge", s[1] - s[0], s[3] - s[2], s[2] - s[1],
			"underground" if tunnel else "raised", hint()])
	else:
		editor.ui.set_status("Can't build it here: %s" % _plan.error)


func draw(o: EditorOverlay) -> void:
	if _hover.is_empty() or _plan.is_empty():
		return
	if not _plan.ok:
		o.draw_outline(editor.road.segment_outline(_hover.id), EditorOverlay.ERROR, 2.0)
		return
	var s: PackedFloat32Array = _plan.stations
	var length: float = _plan.length
	var col := Color(0.55, 0.75, 1.0) if not tunnel else Color(0.75, 0.6, 0.45)
	var ramp := Color(col.r, col.g, col.b, 0.6)
	var line := func(u0: float, u1: float, c: Color, w: float) -> void:
		var pts := PackedVector2Array()
		var n := maxi(2, int((u1 - u0) * length / 4.0))
		for i in n + 1:
			pts.append(editor.road.segment_point(_hover.id, lerpf(u0, u1, float(i) / n)))
		o.draw_polyline(pts, c, w)
	var width := float(editor.road.get_segment(_hover.id).profile.get("width", 8.0))
	line.call(s[0] / length, s[1] / length, ramp, width)
	line.call(s[1] / length, s[2] / length, col, width)
	line.call(s[2] / length, s[3] / length, ramp, width)
	for p in _plan.points:
		o.draw_circle(p, o.px(5), Color.WHITE)
