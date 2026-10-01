class_name FenceTool
extends EditorTool
## E: drag along one side of a road to put up a fence (no informal crossing
## there). Drag over an existing fence, or hold Shift, to take it down.

var _hover := {}
var _painting := false
var _seg := 0
var _side := 1
var _u0 := 0.0
var _u1 := 0.0
var _erase := false


func hint() -> String:
	return "Drag along a road's side to put up a fence · drag over a fence (or Shift-drag) to remove it"


func deactivate() -> void:
	_painting = false
	_hover = {}


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		if _painting:
			_u1 = editor.road.segment_u_at(_seg, mouse)
			var length: float = editor.road.get_segment(_seg).length * absf(_u1 - _u0)
			editor.ui.set_status("%s %.0f m of fence" % ["Removing" if _erase else "Fencing", length])
			return true
		_hover = editor.road.pick(mouse, editor.pick_radius(), editor.level)
		if _hover.type != "segment":
			_hover = {}
		return false
	if is_left_press(event):
		var hit: Dictionary = editor.road.pick(mouse, editor.pick_radius(), editor.level)
		if hit.type != "segment":
			return false
		_seg = hit.id
		_side = 1 if float(hit.offset) >= 0.0 else 0
		_u0 = hit.u
		_u1 = _u0
		_erase = (event as InputEventMouseButton).shift_pressed or _fence_at(_seg, _side, _u0)
		_painting = true
		return true
	if is_left_release(event) and _painting:
		_painting = false
		var lo := minf(_u0, _u1)
		var hi := maxf(_u0, _u1)
		var length: float = editor.road.get_segment(_seg).length * (hi - lo)
		if length >= 0.5:
			editor.road.set_fence(_seg, _side, lo, hi, not _erase)
			editor.notify("%s %.0f m of fence." % ["Removed" if _erase else "Put up", length])
		editor.ui.set_status(hint())
		return true
	if key_pressed(event, KEY_ESCAPE):
		if _painting:
			_painting = false
		else:
			editor.set_tool("select")
		return true
	return false


func _fence_at(seg: int, side: int, u: float) -> bool:
	for f in editor.road.get_segment(seg).fences:
		if int(f.side) == side and u >= float(f.from) and u <= float(f.to):
			return true
	return false


## The road's outer edge on a side (0 left, 1 right).
func _edge(seg: int, side: int) -> int:
	return 0 if side == 0 else (editor.road.get_segment(seg).profile.lanes as Array).size()


func draw(o: EditorOverlay) -> void:
	if _painting:
		var pts: PackedVector2Array = editor.road.edge_polyline(_seg, _edge(_seg, _side), minf(_u0, _u1), maxf(_u0, _u1))
		if pts.size() >= 2:
			o.draw_polyline(pts, EditorOverlay.ERROR if _erase else EditorOverlay.SELECT, o.px(5))
		return
	if _hover.is_empty():
		return
	var side := 1 if float(_hover.offset) >= 0.0 else 0
	var u: float = _hover.u
	var pts: PackedVector2Array = editor.road.edge_polyline(_hover.id, _edge(_hover.id, side), u - 0.05, u + 0.05)
	if pts.size() >= 2:
		o.draw_polyline(pts, EditorOverlay.SELECT, o.px(4))
