class_name RouteTool
extends EditorTool
## U: draw a bus route. Click a depot, then its stops in order; Enter (or a
## click on the depot again) saves the route, Backspace drops the last stop,
## Esc cancels. L toggles loop / end to end. Headway and colour are set in
## the depot's inspector.

const COLORS := [0xd83f3f, 0x2f7fd8, 0x2fa84f, 0xe08a1e, 0x8a4fc9, 0x17a6a0]

var _depot := 0
var _stops: Array = [] # stop dictionaries in order
var _loop := true


func hint() -> String:
	if _depot == 0:
		return "Click a depot to start a bus route"
	return "Click stops in order (%d so far) · Enter or click the depot to save · L %s · Backspace undo · Esc cancel" % [
		_stops.size(), "loop" if _loop else "end to end"]


func activate() -> void:
	_depot = 0
	_stops = []


func deactivate() -> void:
	_depot = 0
	_stops = []


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		editor.ui.set_cursor(mouse)
		return false
	if _depot != 0:
		if key_pressed(event, KEY_ENTER) or key_pressed(event, KEY_KP_ENTER):
			_finish()
			return true
		if key_pressed(event, KEY_ESCAPE):
			activate()
			editor.ui.set_status(hint())
			return true
		if key_pressed(event, KEY_BACKSPACE) and not _stops.is_empty():
			_stops.pop_back()
			editor.ui.set_status(hint())
			return true
		if key_pressed(event, KEY_L):
			_loop = not _loop
			editor.ui.set_status(hint())
			return true
	if is_left_press(event):
		var dp := _depot_at(mouse)
		if _depot == 0:
			if dp == 0:
				editor.notify("Start at a depot (add one with D on a road end).")
			else:
				_depot = dp
				_stops = []
				editor.ui.set_status(hint())
			return true
		if dp == _depot:
			_finish()
			return true
		var st := StopTool.nearest_stop(editor, mouse, editor.pick_radius() * 4.0)
		if st.is_empty():
			editor.notify("Click a bus stop (add stops with K).")
		else:
			_stops.append(st)
			editor.ui.set_status(hint())
		return true
	return false


func _finish() -> void:
	if _stops.size() < 1:
		editor.notify("A route needs at least one stop.")
		return
	var node: Dictionary = editor.road.get_node(_depot)
	var dp: Dictionary = node.depot
	var routes: Array = dp.routes
	var ids: Array = []
	for st in _stops:
		ids.append(int(st.id))
	var n := routes.size()
	routes.append({"id": 0, "name": "Route %d" % (n + 1), "color": COLORS[n % COLORS.size()], "stops": ids,
		"headway": 600.0, "loop": _loop})
	dp["routes"] = routes
	editor.road.set_depot(_depot, dp)
	editor.notify("Route %d saved: %d stops, a bus every 10 min." % [n + 1, ids.size()])
	editor.select("nodes", _depot, false)
	activate()
	editor.ui.set_status(hint())


func _depot_at(pos: Vector2) -> int:
	for d in editor.road.get_depots():
		if int(d.level) == editor.level and (d.pos as Vector2).distance_to(pos) < editor.pick_radius() * 3.0:
			return int(d.node)
	return 0


func draw(o: EditorOverlay) -> void:
	if _depot == 0:
		return
	var pts := PackedVector2Array([editor.road.get_node(_depot).pos])
	for st in _stops:
		pts.append(st.pos)
	pts.append(editor.mouse_world())
	var n: int = editor.road.get_node(_depot).depot.routes.size()
	o.draw_polyline(pts, EditorOverlay.rgb(COLORS[n % COLORS.size()]), o.px(3))
	for st in _stops:
		o.draw_circle(st.pos, o.px(6), EditorOverlay.SELECT)
