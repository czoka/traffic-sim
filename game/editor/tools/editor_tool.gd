class_name EditorTool
extends RefCounted
## Base class for editor tools. The editor forwards unhandled input and asks
## the active tool to draw its preview on the overlay.

var editor: MapEditor


func _init(e: MapEditor) -> void:
	editor = e


func activate() -> void:
	pass


func deactivate() -> void:
	pass


## Returns true when the event was used.
func input(_event: InputEvent) -> bool:
	return false


func draw(_o: EditorOverlay) -> void:
	pass


func hint() -> String:
	return ""


static func is_left_press(event: InputEvent) -> bool:
	var mb := event as InputEventMouseButton
	return mb != null and mb.button_index == MOUSE_BUTTON_LEFT and mb.pressed


static func is_left_release(event: InputEvent) -> bool:
	var mb := event as InputEventMouseButton
	return mb != null and mb.button_index == MOUSE_BUTTON_LEFT and not mb.pressed


static func key_pressed(event: InputEvent, keycode: Key) -> bool:
	var k := event as InputEventKey
	return k != null and k.pressed and not k.echo and k.keycode == keycode


## "123 m · 45°" for a drawing leg (angle 0 = east, counter-clockwise on screen).
static func readout(a: Vector2, b: Vector2) -> String:
	var d := b - a
	var deg := fposmod(-rad_to_deg(d.angle()), 360.0)
	return "%.1f m · %.0f°" % [d.length(), deg]
