class_name SpawnerTool
extends EditorTool
## N: click a road end to add a spawn / sink point (traffic from outside the
## map), or click an existing one to select it. Shift-click removes it. Rates
## and destinations are set in the inspector.

const DEFAULT_RATE := 300.0

var _hover := 0


func hint() -> String:
	return "Click a road end to add a spawn point · click one to edit it · Shift-click removes it"


func activate() -> void:
	_hover = 0


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_hover = _road_end_at(mouse)
		editor.ui.set_cursor(mouse)
		return false
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		var n := _road_end_at(mouse)
		if n == 0:
			editor.notify("Spawn points go on road ends: click the loose end of a road.")
			return true
		var node: Dictionary = editor.road.get_node(n)
		if mb.shift_pressed:
			if node.spawner.enabled:
				editor.road.set_spawner(n, {"enabled": false})
				editor.notify("Spawn point removed.")
			return true
		if not node.spawner.enabled:
			editor.road.set_spawner(n, {"enabled": true, "rate": DEFAULT_RATE, "sink": true})
			editor.notify("Spawn point added: %d cars/h in, and cars may leave here." % int(DEFAULT_RATE))
		editor.select("nodes", n, false)
		return true
	return false


## The nearest road-end node within the pick radius (or a spawn point anywhere).
func _road_end_at(pos: Vector2) -> int:
	var n: int = editor.road.nearest_node(pos, editor.pick_radius() * 2.0, editor.level, 0)
	if n == 0:
		return 0
	var node: Dictionary = editor.road.get_node(n)
	return n if node.road_end or node.spawner.enabled else 0


func draw(o: EditorOverlay) -> void:
	if _hover != 0:
		o.draw_circle(editor.road.get_node(_hover).pos, o.px(11), EditorOverlay.SELECT, false, o.px(2))
