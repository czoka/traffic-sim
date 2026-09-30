class_name LanePaintTool
extends EditorTool
## L: drag along the line between two lanes to paint a no-change zone (solid
## line); drag over an existing zone to erase it. Alt paints a one-sided zone
## (crossing forbidden from the left lane only; Alt+Shift from the right).
## Click inside a lane to change its type.

const TYPES := ["general", "bus", "bike", "turn", "parking", "sidewalk"]

var _hover := {}
var _painting := false
var _seg := 0
var _edge := -1
var _u0 := 0.0
var _u1 := 0.0
var _erase := false
var _l2r := true
var _r2l := true


func hint() -> String:
	return "Drag along a lane line to paint no-change · drag over a zone to erase · Alt: one side · click a lane to set its type"


func deactivate() -> void:
	_painting = false
	_hover = {}


func _edge_radius() -> float:
	return maxf(0.5, 8.0 / editor.camera.zoom.x)


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		if _painting:
			_u1 = editor.road.segment_u_at(_seg, mouse)
			editor.ui.set_status("%s zone: %.0f%% → %.0f%% of the road" % ["Erasing" if _erase else "Painting", _u0 * 100, _u1 * 100])
			return true
		_hover = editor.road.pick(mouse, editor.pick_radius() * 0.3, editor.level)
		if _hover.type == "node":
			_hover = {}
		return false
	if is_left_press(event):
		_hover = editor.road.pick(mouse, editor.pick_radius() * 0.3, editor.level)
		if _hover.is_empty() or _hover.type != "segment":
			return false
		if _near_edge():
			_seg = _hover.id
			_edge = _hover.edge
			_u0 = _hover.u
			_u1 = _u0
			_erase = _zone_at(_seg, _edge, _u0)
			var mb := event as InputEventMouseButton
			_l2r = not mb.alt_pressed or not mb.shift_pressed
			_r2l = not mb.alt_pressed or mb.shift_pressed
			_painting = true
			return true
		if int(_hover.lane) > 0:
			_lane_menu(_hover.id, _hover.lane)
			return true
		return false
	if is_left_release(event) and _painting:
		_painting = false
		var length: float = editor.road.get_segment(_seg).length * absf(_u1 - _u0)
		if length >= 0.5:
			if _erase:
				editor.road.set_no_change(_seg, _edge, _u0, _u1, false, false)
				editor.notify("Erased %.0f m of no-change line." % length)
			else:
				editor.road.set_no_change(_seg, _edge, _u0, _u1, _l2r, _r2l)
				editor.notify("Painted %.0f m of no-change line%s." % [length, "" if _l2r and _r2l else " (one side)"])
		editor.ui.set_status(hint())
		return true
	if key_pressed(event, KEY_ESCAPE):
		if _painting:
			_painting = false
		else:
			editor.set_tool("select")
		return true
	return false


func _near_edge() -> bool:
	return int(_hover.get("edge", -1)) > 0 and float(_hover.get("edge_distance", 1e9)) <= _edge_radius()


func _zone_at(seg: int, edge: int, u: float) -> bool:
	for z in editor.road.get_segment(seg).no_change:
		if int(z.edge) == edge and u >= float(z.from) and u <= float(z.to):
			return true
	return false


func _lane_menu(seg: int, lane: int) -> void:
	var menu := PopupMenu.new()
	for i in TYPES.size():
		menu.add_item(String(TYPES[i]).capitalize(), i)
	menu.id_pressed.connect(func(i: int) -> void:
		var err: String = editor.road.set_lane_type(seg, lane, TYPES[i])
		if err != "":
			editor.notify("Can't change lane: %s" % err)
		menu.queue_free())
	menu.popup_hide.connect(func() -> void: menu.queue_free.call_deferred())
	editor.ui.add_child(menu)
	menu.popup(Rect2i(Vector2i(editor.get_viewport().get_mouse_position()), Vector2i.ZERO))


func draw(o: EditorOverlay) -> void:
	if _painting:
		var pts: PackedVector2Array = editor.road.edge_polyline(_seg, _edge, _u0, _u1)
		if pts.size() >= 2:
			o.draw_polyline(pts, EditorOverlay.ERROR if _erase else EditorOverlay.SELECT, o.px(5))
		return
	if _hover.is_empty():
		return
	if _near_edge():
		var u: float = _hover.u
		var pts: PackedVector2Array = editor.road.edge_polyline(_hover.id, _hover.edge, u - 0.05, u + 0.05)
		if pts.size() >= 2:
			o.draw_polyline(pts, EditorOverlay.SELECT, o.px(4))
	elif int(_hover.get("lane", 0)) > 0:
		o.draw_outline(editor.road.lane_outline(_hover.id, _hover.lane), EditorOverlay.HOVER, 2.0)
