class_name RoundaboutTool
extends EditorTool
## O: click a junction to turn it into a roundabout (or select one that is
## already a roundabout). Shift-click turns it back into a plain junction.
## Radius, lanes, turbo layout and slip lanes are set in the inspector.

const DEFAULT_RADIUS := 20.0

var _hover := 0


func hint() -> String:
	return "Click a junction to make it a roundabout · click one to edit it · Shift-click removes it"


func activate() -> void:
	_hover = 0


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_hover = _junction_at(mouse)
		editor.ui.set_cursor(mouse)
		return false
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		var n := _junction_at(mouse)
		if n == 0:
			editor.notify("Roundabouts replace a junction: click a node where three or more roads meet.")
			return true
		var node: Dictionary = editor.road.get_node(n)
		var r: Dictionary = node.roundabout
		if mb.shift_pressed:
			if r.enabled:
				r["enabled"] = false
				editor.road.set_roundabout(n, r)
				editor.notify("Roundabout removed.")
			return true
		if not r.enabled:
			var lanes := 1
			for s in node.segments:
				var seg: Dictionary = editor.road.get_segment(s)
				lanes = maxi(lanes, maxi(int(seg.params.forward), int(seg.params.backward)))
			editor.road.set_roundabout(n, {"enabled": true, "radius": DEFAULT_RADIUS + 4.0 * (mini(lanes, 3) - 1),
				"lanes": mini(lanes, 3), "turbo": false, "slip": []})
			editor.notify("Roundabout added: traffic on the ring has priority, entries give way.")
		editor.select("nodes", n, false)
		return true
	return false


func _junction_at(pos: Vector2) -> int:
	var n: int = editor.road.nearest_node(pos, editor.pick_radius() * 3.0, editor.level, 0)
	if n == 0:
		return 0
	var node: Dictionary = editor.road.get_node(n)
	return n if node.segments.size() >= 3 or node.roundabout.enabled else 0


func draw(o: EditorOverlay) -> void:
	if _hover != 0:
		var node: Dictionary = editor.road.get_node(_hover)
		var r: float = float(node.roundabout.radius) if node.roundabout.enabled else DEFAULT_RADIUS
		o.draw_circle(node.pos, r, EditorOverlay.SELECT, false, o.px(2))
