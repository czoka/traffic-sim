class_name CameraController
extends Camera2D
## Pan with middle/right mouse drag, arrow keys or a trackpad; zoom with
## the wheel or pinch, anchored at the cursor. World units are metres.

const MIN_ZOOM := 0.05
const MAX_ZOOM := 40.0
const KEY_PAN_SPEED := 900.0 # screen pixels per second

var _dragging := false


func _ready() -> void:
	make_current()


func fit(rect: Rect2, margin := 1.1) -> void:
	if rect.size == Vector2.ZERO:
		return
	var view := get_viewport_rect().size
	var z := minf(view.x / (rect.size.x * margin), view.y / (rect.size.y * margin))
	zoom = Vector2.ONE * clampf(z, MIN_ZOOM, MAX_ZOOM)
	position = rect.get_center()


func zoom_at(factor: float, screen_point: Vector2) -> void:
	var before := _screen_to_world(screen_point)
	zoom = Vector2.ONE * clampf(zoom.x * factor, MIN_ZOOM, MAX_ZOOM)
	var after := _screen_to_world(screen_point)
	position += before - after


func _screen_to_world(p: Vector2) -> Vector2:
	# Camera is centred (anchor mode DRAG_CENTER) with no rotation.
	return position + (p - get_viewport_rect().size * 0.5) / zoom.x


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		match mb.button_index:
			MOUSE_BUTTON_WHEEL_UP:
				if mb.pressed:
					zoom_at(1.15, mb.position)
			MOUSE_BUTTON_WHEEL_DOWN:
				if mb.pressed:
					zoom_at(1.0 / 1.15, mb.position)
			MOUSE_BUTTON_MIDDLE, MOUSE_BUTTON_RIGHT:
				_dragging = mb.pressed
	elif event is InputEventMouseMotion and _dragging:
		position -= (event as InputEventMouseMotion).relative / zoom.x
	elif event is InputEventPanGesture:
		position += (event as InputEventPanGesture).delta * 12.0 / zoom.x
	elif event is InputEventMagnifyGesture:
		var g := event as InputEventMagnifyGesture
		zoom_at(g.factor, g.position)


func _process(delta: float) -> void:
	var dir := Vector2.ZERO
	if Input.is_key_pressed(KEY_LEFT):
		dir.x -= 1.0
	if Input.is_key_pressed(KEY_RIGHT):
		dir.x += 1.0
	if Input.is_key_pressed(KEY_UP):
		dir.y -= 1.0
	if Input.is_key_pressed(KEY_DOWN):
		dir.y += 1.0
	if dir != Vector2.ZERO:
		position += dir.normalized() * KEY_PAN_SPEED * delta / zoom.x
