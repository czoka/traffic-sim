class_name StopTool
extends EditorTool
## K: click beside a road to add a bus stop on that side (the side of the
## direction it serves). Shift-click removes the nearest stop. Kind (kerbside,
## bus bay, main station), name and bays are set in the inspector.

var kind := "kerbside"
var _hover := {}


func hint() -> String:
	return "Click a road's side to add a %s stop · 1 kerbside, 2 bus bay, 3 main station · Shift-click removes" % kind.replace("_", " ")


func activate() -> void:
	_hover = {}


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_hover = editor.road.pick(mouse, editor.pick_radius() * 2.0, editor.level)
		editor.ui.set_cursor(mouse)
		return false
	for k in [[KEY_1, "kerbside"], [KEY_2, "bay"], [KEY_3, "main_station"]]:
		if key_pressed(event, k[0]):
			kind = k[1]
			editor.ui.set_status(hint())
			return true
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		if mb.shift_pressed:
			var st := nearest_stop(editor, mouse, editor.pick_radius() * 4.0)
			if not st.is_empty():
				editor.road.remove_stop(st.segment, st.id)
				editor.notify("Stop removed.")
			return true
		var hit: Dictionary = editor.road.pick(mouse, editor.pick_radius() * 2.0, editor.level)
		if hit.type != "segment":
			editor.notify("Bus stops go on a road: click the side of the road the buses drive on.")
			return true
		var side := "forward" if float(hit.offset) >= 0.0 else "backward"
		var seg: Dictionary = editor.road.get_segment(hit.id)
		if seg.one_way:
			side = "forward" if int(seg.params.forward) > 0 else "backward"
		var n: int = editor.road.get_stops().size() + 1
		var label := "Main Station" if kind == "main_station" else "Stop %d" % n
		var id: int = editor.road.add_stop(hit.id, float(hit.u), side, kind, label)
		if id == 0:
			editor.notify("Could not add a stop there.")
		else:
			editor.notify("%s added (%s)." % [label, kind.replace("_", " ")])
			editor.select("segments", int(hit.id), false)
		return true
	return false


## The nearest stop to a point: {id, segment, pos, ...} or {}.
static func nearest_stop(e: MapEditor, pos: Vector2, radius: float) -> Dictionary:
	var best := {}
	var best_d := radius
	for st in e.road.get_stops():
		if int(st.level) != e.level:
			continue
		var d := (st.pos as Vector2).distance_to(pos)
		if d < best_d:
			best_d = d
			best = st
	return best


func draw(o: EditorOverlay) -> void:
	if not _hover.is_empty() and _hover.type == "segment":
		var p: Vector2 = editor.road.segment_point(_hover.id, _hover.u)
		o.draw_circle(p, o.px(6), EditorOverlay.STOP, false, o.px(2))
