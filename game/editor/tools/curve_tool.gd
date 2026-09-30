class_name CurveDrawTool
extends EditorTool
## C: click the start, click the control point, click the end.
## Starting from the end of an existing road keeps the curve tangent to it
## (hold Alt to bend freely).

var _start := {}
var _control = null # Vector2 once placed
var _cursor := {}
var _tangent := Vector2.ZERO # continuation direction, or zero


func hint() -> String:
	return "Click start · click control point · click end · Alt frees the tangent · Esc cancels"


func activate() -> void:
	_reset()


func deactivate() -> void:
	_reset()


func _reset() -> void:
	_start = {}
	_control = null
	_tangent = Vector2.ZERO


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	var alt := Input.is_key_pressed(KEY_ALT)
	if event is InputEventMouseMotion:
		if _start.is_empty():
			_cursor = editor.snap_point(mouse)
		elif _control == null:
			_cursor = {"pos": _constrain(mouse, alt), "kind": "free"}
		else:
			_cursor = editor.snap_point(mouse)
		_status()
		return false
	if is_left_press(event):
		if _start.is_empty():
			_start = editor.snap_point(mouse)
			_tangent = _continuation(_start)
		elif _control == null:
			_control = _constrain(mouse, alt)
		else:
			var end := editor.snap_point(mouse)
			var id: int = editor.road.add_curve(MapEditor.to_ref(_start), _control, MapEditor.to_ref(end),
				editor.road_template, editor.level, editor.speed_kmh)
			if id == 0:
				editor.notify("Curve not added: start and end are too close.")
			_reset()
		return true
	if key_pressed(event, KEY_ESCAPE):
		if _start.is_empty():
			editor.set_tool("select")
		else:
			_reset()
		return true
	return false


## Direction that continues the road ending at the snapped node, if any.
func _continuation(snap: Dictionary) -> Vector2:
	if snap.kind != "node":
		return Vector2.ZERO
	var n: Dictionary = editor.road.get_node(snap.node)
	if n.segments.size() != 1:
		return Vector2.ZERO
	var sid: int = n.segments[0]
	var s: Dictionary = editor.road.get_segment(sid)
	if s.to == snap.node:
		return editor.road.segment_tangent(sid, 1.0)
	return -editor.road.segment_tangent(sid, 0.0)


func _constrain(mouse: Vector2, alt: bool) -> Vector2:
	var start: Vector2 = _start.pos
	if _tangent != Vector2.ZERO and not alt:
		return start + _tangent * maxf(1.0, (mouse - start).dot(_tangent))
	return mouse.snapped(Vector2.ONE) if editor.snap_grid else mouse


func _status() -> void:
	if _start.is_empty():
		editor.ui.set_status(hint())
	elif _control == null:
		editor.ui.set_status("Control point: " + readout(_start.pos, _cursor.pos) +
			(" · keeps the road's direction (Alt to free)" if _tangent != Vector2.ZERO else ""))
	else:
		editor.ui.set_status("End: " + readout(_start.pos, _cursor.pos) + " · snap: " + str(_cursor.kind))


func draw(o: EditorOverlay) -> void:
	if _start.is_empty():
		o.draw_snap(_cursor)
		return
	var a: Vector2 = _start.pos
	o.draw_circle(a, o.px(4), EditorOverlay.SELECT)
	if _cursor.is_empty():
		return
	if _control == null:
		o.draw_line(a, _cursor.pos, EditorOverlay.SELECT, o.px(1.5))
		o.draw_circle(_cursor.pos, o.px(4), EditorOverlay.SELECT, false, o.px(1.5))
		return
	var q: Vector2 = _control
	var b: Vector2 = _cursor.pos
	var pts := PackedVector2Array()
	for i in 33:
		var t := i / 32.0
		pts.append(a * (1 - t) * (1 - t) + q * 2.0 * (1 - t) * t + b * t * t)
	o.draw_polyline(pts, Color(0.4, 0.7, 1.0, 0.35), editor.template_width())
	o.draw_polyline(pts, EditorOverlay.SELECT, o.px(2))
	o.draw_line(a, q, EditorOverlay.HOVER, o.px(1))
	o.draw_line(q, b, EditorOverlay.HOVER, o.px(1))
	o.draw_circle(q, o.px(4), EditorOverlay.SELECT, false, o.px(1.5))
	o.draw_snap(_cursor)
