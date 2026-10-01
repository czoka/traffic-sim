class_name CentreTool
extends EditorTool
## T: click to put the city centre marker. Home rents and building prices
## scale with distance from it (full price at the centre, half at 3 km).
## Shift-click removes it; the centre of the shops and offices is used then.


func hint() -> String:
	return "Click to place the city centre · Shift-click removes it (prices then use the middle of the shops and offices)"


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		editor.ui.set_cursor(mouse)
		editor.overlay.queue_redraw()
		return false
	if is_left_press(event):
		var mb := event as InputEventMouseButton
		if mb.shift_pressed:
			if editor.road.get_city_centre().placed:
				editor.road.clear_city_centre()
				editor.notify("City centre removed.")
			return true
		editor.road.set_city_centre(mouse)
		editor.notify("City centre placed. Homes and businesses near it cost more.")
		return true
	return false


func draw(o: EditorOverlay) -> void:
	var p := editor.mouse_world()
	o.draw_circle(p, o.px(10), EditorOverlay.SELECT, false, o.px(2))
	o.draw_circle(p, 3000.0, Color(1, 0.85, 0.3, 0.35), false, o.px(1.5))
