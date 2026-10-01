class_name RoadDrawTool
extends EditorTool
## R: click the start, click bends, double-click (or Enter) to finish.
## Snaps to nodes, onto existing roads (making a junction), 15° angles and the
## 1 m grid. Backspace removes the last point, Esc cancels.

var _points: Array = [] # snap dictionaries
var _cursor := {}


func hint() -> String:
	return "Click to start · click to bend · double-click or Enter to finish · Backspace undoes a point · Esc cancels"


func activate() -> void:
	_points.clear()
	_cursor = {}


func deactivate() -> void:
	_points.clear()


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_cursor = editor.snap_point(mouse, _last_pos())
		_update_status()
		return false
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		if mb.double_click:
			_finish()
			return true
		var p := editor.snap_point(mouse, _last_pos())
		if _points.is_empty() or (p.pos as Vector2).distance_to(_points[-1].pos) > 0.5:
			_points.append(p)
			# Clicking an existing node or road after the start ends the road there.
			if _points.size() >= 2 and (p.kind == "node" or p.kind == "segment"):
				_finish()
		return true
	if key_pressed(event, KEY_ENTER) or key_pressed(event, KEY_KP_ENTER):
		_finish()
		return true
	if key_pressed(event, KEY_BACKSPACE) and not _points.is_empty():
		_points.pop_back()
		return true
	if key_pressed(event, KEY_ESCAPE):
		if _points.is_empty():
			editor.set_tool("select")
		else:
			_points.clear()
		return true
	return false


func _last_pos():
	return null if _points.is_empty() else _points[-1].pos


func _finish() -> void:
	if _points.size() >= 2:
		var refs: Array = []
		for p in _points:
			refs.append(MapEditor.to_ref(p))
		var ids: PackedInt64Array = editor.road.add_road(refs, _template(), editor.level, _speed_kmh())
		if ids.is_empty():
			editor.notify("%s not added: the points are too close together." % _noun().capitalize())
		else:
			editor.notify("Added %d %s segment%s." % [ids.size(), _noun(), "" if ids.size() == 1 else "s"])
	_points.clear()


## What gets drawn (PathTool overrides these).
func _template() -> Dictionary:
	return editor.road_template


func _speed_kmh() -> float:
	return editor.speed_kmh


func _width() -> float:
	return editor.template_width()


func _noun() -> String:
	return "road"


func _update_status() -> void:
	if _points.is_empty():
		editor.ui.set_status(hint())
		return
	var total := 0.0
	for i in range(1, _points.size()):
		total += (_points[i].pos as Vector2).distance_to(_points[i - 1].pos)
	var leg := readout(_points[-1].pos, _cursor.pos)
	total += (_cursor.pos as Vector2).distance_to(_points[-1].pos)
	editor.ui.set_status("%s · total %.0f m · snap: %s" % [leg, total, _cursor.kind])


func draw(o: EditorOverlay) -> void:
	var pts := PackedVector2Array()
	for p in _points:
		pts.append(p.pos)
	if not _cursor.is_empty() and not _points.is_empty():
		pts.append(_cursor.pos)
	if pts.size() >= 2:
		o.draw_polyline(pts, Color(0.4, 0.7, 1.0, 0.35), _width(), false)
		o.draw_polyline(pts, EditorOverlay.SELECT, o.px(2))
		var a: Vector2 = pts[pts.size() - 2]
		var b: Vector2 = pts[pts.size() - 1]
		var font := ThemeDB.fallback_font
		o.draw_string(font, b + Vector2(o.px(12), -o.px(12)), readout(a, b), HORIZONTAL_ALIGNMENT_LEFT, -1,
			int(o.px(14)), Color.WHITE)
	for p in _points:
		o.draw_circle(p.pos, o.px(4), EditorOverlay.SELECT)
	o.draw_snap(_cursor)
