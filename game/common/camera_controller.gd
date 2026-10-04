class_name CameraController
extends Camera2D
## Pan with middle/right mouse drag, arrow keys, a trackpad or two fingers;
## zoom with the wheel or pinch (trackpad or touch screen), anchored at the
## cursor or between the fingers. World units are metres.

const MIN_ZOOM := 0.05
const MAX_ZOOM := 40.0
const KEY_PAN_SPEED := 900.0 # screen pixels per second

var _dragging := false
## Touch screens (phones, tablets) send raw touches, not pinch or pan gestures:
## finger index -> screen position for each finger down (#24).
var _touches: Dictionary = {}
## Optional: (screen position) -> true where the UI covers the map (set by the editor).
var ui_hit: Callable


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


## True while the mouse is over a panel or other UI: wheel and gestures there
## belong to it (a list that has scrolled to its end lets the wheel through,
## and the map must not zoom then).
func _over_ui(pos: Vector2) -> bool:
	if ui_hit.is_valid():
		return ui_hit.call(pos)
	return get_viewport().gui_get_hovered_control() != null


## True while two fingers are on the map (pinching or panning).
func is_pinching() -> bool:
	return _touches.size() >= 2


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventScreenTouch or event is InputEventScreenDrag:
		_touch(event)
		return
	if event is InputEventGesture and _over_ui((event as InputEventGesture).position):
		return
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if (mb.button_index == MOUSE_BUTTON_WHEEL_UP or mb.button_index == MOUSE_BUTTON_WHEEL_DOWN) and _over_ui(mb.position):
			return
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


## Two fingers: the gap between them zooms and their midpoint pans, so the
## map stays under the fingers. A finger that lands on a panel is ignored.
func _touch(event: InputEvent) -> void:
	if event is InputEventScreenTouch:
		var t := event as InputEventScreenTouch
		if t.pressed and not _over_ui(t.position):
			_touches[t.index] = t.position
		elif not t.pressed:
			_touches.erase(t.index)
		return
	var d := event as InputEventScreenDrag
	if not _touches.has(d.index):
		return
	if _touches.size() < 2:
		_touches[d.index] = d.position
		return
	var keys := _touches.keys()
	var other: Vector2 = _touches[keys[1] if keys[0] == d.index else keys[0]]
	var old_mid: Vector2 = (_touches[d.index] + other) * 0.5
	var old_gap: float = (_touches[d.index] as Vector2).distance_to(other)
	_touches[d.index] = d.position
	var mid := (d.position + other) * 0.5
	var gap := d.position.distance_to(other)
	position -= (mid - old_mid) / zoom.x
	if old_gap > 1.0 and gap > 1.0:
		zoom_at(gap / old_gap, mid)


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
