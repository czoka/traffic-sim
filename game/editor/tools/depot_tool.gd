class_name DepotTool
extends EditorTool
## D: click a road end to put a bus depot there (buses start and finish their
## runs at it). Shift-click removes it. Capacity and routes are set in the
## inspector; draw routes with the route tool (U).

var _hover := 0


func hint() -> String:
	return "Click a road end to add a bus depot · click one to edit it · Shift-click removes it"


func activate() -> void:
	_hover = 0


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_hover = _end_at(mouse)
		editor.ui.set_cursor(mouse)
		return false
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		var n := _end_at(mouse)
		if n == 0:
			editor.notify("Depots go on road ends: click the loose end of a road.")
			return true
		var node: Dictionary = editor.road.get_node(n)
		var dp: Dictionary = node.depot
		if mb.shift_pressed:
			if dp.enabled:
				dp["enabled"] = false
				editor.road.set_depot(n, dp)
				editor.notify("Depot removed.")
			return true
		if not dp.enabled:
			editor.road.set_depot(n, {"enabled": true, "name": "Depot %d" % (editor.road.get_depots().size() + 1),
				"capacity": 20, "routes": []})
			editor.notify("Depot added. Press U to draw a bus route from it through its stops.")
		editor.select("nodes", n, false)
		return true
	return false


func _end_at(pos: Vector2) -> int:
	var n: int = editor.road.nearest_node(pos, editor.pick_radius() * 2.0, editor.level, 0)
	if n == 0:
		return 0
	var node: Dictionary = editor.road.get_node(n)
	return n if node.road_end or node.depot.enabled else 0


func draw(o: EditorOverlay) -> void:
	if _hover != 0:
		o.draw_circle(editor.road.get_node(_hover).pos, o.px(12), EditorOverlay.SELECT, false, o.px(2))
