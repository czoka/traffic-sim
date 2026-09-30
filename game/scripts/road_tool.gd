class_name RoadTool
extends Node2D
## Throwaway POC tool: click to set a start point, click again to place a
## straight two-lane road. Esc or right-click cancels. The real editor is M1.

var main: Main
var active := false:
	set(value):
		active = value
		_start = null
		queue_redraw()

var _start = null # Vector2 or null
var _mouse := Vector2.ZERO


func _unhandled_input(event: InputEvent) -> void:
	if not active:
		return
	if event is InputEventMouseMotion:
		_mouse = get_global_mouse_position()
		if _start != null:
			queue_redraw()
	elif event is InputEventMouseButton and event.pressed:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT:
			var p := get_global_mouse_position()
			if _start == null:
				_start = p
			else:
				main.add_road(_start, p)
				_start = null
			queue_redraw()
			get_viewport().set_input_as_handled()
		elif mb.button_index == MOUSE_BUTTON_RIGHT and _start != null:
			_start = null
			queue_redraw()
			get_viewport().set_input_as_handled()
	elif event is InputEventKey and event.pressed and (event as InputEventKey).keycode == KEY_ESCAPE:
		if _start != null:
			_start = null
		else:
			active = false
			main.hud.refresh_controls()
		queue_redraw()
		get_viewport().set_input_as_handled()


func _draw() -> void:
	if not active or _start == null:
		return
	var w := 7.0
	draw_line(_start, _mouse, Color(0.4, 0.7, 1.0, 0.5), w)
	draw_circle(_start, w * 0.6, Color(0.4, 0.7, 1.0, 0.9))
