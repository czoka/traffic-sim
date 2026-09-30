class_name SelectTool
extends EditorTool
## V: click to select, shift-click to add, drag nodes / roads / curve handles.
## Dropping a node on another node joins them. Delete removes the selection.

var _hover := {}
var _drag := "" # "", "move", "handle"
var _press_world := Vector2.ZERO
var _moved := false
var _origins := {} # node id -> start position
var _drag_node := 0 # the node under the cursor when dragging one node
var _merge_target := 0
var _handle_seg := 0
var _handle_index := -1


func hint() -> String:
	return "Click to select (cars too) · Shift-click to add · Drag to move · Drag handles to bend · Delete removes"


func deactivate() -> void:
	if _drag != "":
		editor.road.cancel()
	_drag = ""
	_hover = {}


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		if _drag == "":
			_hover = editor.road.pick(mouse, editor.pick_radius(), editor.level)
			editor.ui.set_cursor(mouse)
			return false
		_moved = true
		_drag_update(mouse)
		return true
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		_press_world = mouse
		_moved = false
		var h := _handle_at(mouse)
		if not h.is_empty():
			_handle_seg = h.segment
			_handle_index = h.index
			editor.road.begin("Bend road")
			_drag = "handle"
			return true
		if editor.sim.has_cars():
			# Cars first: click a car to see its route and state.
			var car: int = editor.road.sim_pick_car(mouse, maxf(1.5, editor.pick_radius() * 0.6), editor.level)
			if car != 0:
				editor.clear_selection()
				editor.sim.select_car(car)
				return true
		var hit: Dictionary = editor.road.pick(mouse, editor.pick_radius(), editor.level)
		if hit.type == "none":
			if not mb.shift_pressed:
				editor.clear_selection()
			return true
		var kind := "nodes" if hit.type == "node" else "segments"
		var already: bool = editor.selection[kind].has(hit.id)
		if mb.shift_pressed or not already:
			editor.select(kind, hit.id, mb.shift_pressed)
		if not editor.selection[kind].has(hit.id):
			return true # shift-click removed it
		_start_move(hit)
		return true
	if is_left_release(event) and _drag != "":
		if _drag == "move" and _merge_target != 0 and _drag_node != 0:
			editor.road.merge_nodes(_drag_node, _merge_target)
			editor.clear_selection()
			editor.select("nodes", _merge_target, false)
		if _moved:
			editor.road.commit()
		else:
			editor.road.cancel()
		_drag = ""
		_merge_target = 0
		return true
	if key_pressed(event, KEY_ESCAPE) and _drag != "":
		editor.road.cancel()
		_drag = ""
		return true
	return false


func _start_move(hit: Dictionary) -> void:
	_origins.clear()
	for id in editor.selection.nodes:
		_origins[id] = editor.road.get_node(id).pos
	for id in editor.selection.segments:
		var s: Dictionary = editor.road.get_segment(id)
		for n in [s.from, s.to]:
			if not _origins.has(n):
				_origins[n] = editor.road.get_node(n).pos
	_drag_node = hit.id if hit.type == "node" and _origins.size() == 1 else 0
	editor.road.begin("Move")
	_drag = "move"


func _drag_update(mouse: Vector2) -> void:
	if _drag == "handle":
		var p := mouse.snapped(Vector2.ONE) if editor.snap_grid else mouse
		editor.road.set_control_point(_handle_seg, _handle_index, p)
		return
	_merge_target = 0
	if _drag_node != 0:
		# A single node follows the cursor and can snap onto another node.
		var snap := editor.snap_point(mouse, null, _drag_node)
		var target: Vector2 = snap.pos
		if snap.kind == "node":
			_merge_target = snap.node
		elif snap.kind == "segment":
			target = mouse.snapped(Vector2.ONE) if editor.snap_grid else mouse
		editor.road.move_node(_drag_node, target)
		editor.ui.set_status("Node at %.0f, %.0f%s" % [target.x, target.y, " · release to join" if _merge_target else ""])
		return
	var delta := mouse - _press_world
	if editor.snap_grid:
		delta = delta.snapped(Vector2.ONE)
	for id in _origins:
		editor.road.move_node(id, _origins[id] + delta)
	editor.ui.set_status("Moved %.0f m" % delta.length())


## Bézier handles of the single selected road (straight roads get virtual ones).
func _handles() -> Array:
	if editor.selection.segments.size() != 1 or not editor.selection.nodes.is_empty():
		return []
	var id: int = editor.selection.segments[0]
	var s: Dictionary = editor.road.get_segment(id)
	if s.is_empty():
		return []
	var a: Vector2 = editor.road.get_node(s.from).pos
	var b: Vector2 = editor.road.get_node(s.to).pos
	var c1: Vector2 = s.c1
	var c2: Vector2 = s.c2
	if s.curve != "bezier":
		c1 = a.lerp(b, 1.0 / 3.0)
		c2 = a.lerp(b, 2.0 / 3.0)
	return [
		{"segment": id, "index": 0, "pos": c1, "anchor": a, "virtual": s.curve != "bezier"},
		{"segment": id, "index": 1, "pos": c2, "anchor": b, "virtual": s.curve != "bezier"},
	]


func _handle_at(mouse: Vector2) -> Dictionary:
	for h in _handles():
		if mouse.distance_to(h.pos) <= editor.pick_radius():
			return h
	return {}


func draw(o: EditorOverlay) -> void:
	if _drag == "" and not _hover.is_empty():
		if _hover.type == "segment" and not editor.selection.segments.has(_hover.id):
			o.draw_outline(editor.road.segment_outline(_hover.id), EditorOverlay.HOVER, 1.5)
		elif _hover.type == "node" and not editor.selection.nodes.has(_hover.id):
			o.draw_circle(editor.road.get_node(_hover.id).pos, o.px(6), EditorOverlay.HOVER)
	if _merge_target != 0:
		o.draw_circle(editor.road.get_node(_merge_target).pos, o.px(10), EditorOverlay.SELECT, false, o.px(2.5))
	for h in _handles():
		o.draw_line(h.anchor, h.pos, EditorOverlay.SELECT, o.px(1.5))
		if h.virtual:
			o.draw_circle(h.pos, o.px(5), EditorOverlay.SELECT, false, o.px(1.5))
		else:
			o.draw_circle(h.pos, o.px(5), EditorOverlay.SELECT)
