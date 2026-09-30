class_name EditorOverlay
extends Node2D
## Everything drawn on top of the roads: selection, hover, Bézier handles,
## connectors, spawn points, the selected car's route, problem markers and the
## active tool's preview.

const SELECT := Color(0.35, 0.72, 1.0, 1.0)
const HOVER := Color(1.0, 1.0, 1.0, 0.55)
const ERROR := Color(0.93, 0.33, 0.29)
const WARNING := Color(0.96, 0.72, 0.2)
const SPAWN := Color(0.4, 0.85, 0.95)
const ROUTE := Color(0.35, 0.9, 1.0, 0.85)
const TURN_COLORS := {
	"straight": Color(1, 1, 1, 0.8),
	"left": Color(0.45, 0.7, 1.0, 0.9),
	"right": Color(1.0, 0.62, 0.3, 0.9),
	"uturn": Color(0.9, 0.4, 0.9, 0.9),
}

var editor: MapEditor
var _cache_rev := -1
var _connectors: Array = []
var _problems: Array = []
var _spawners: Array = []


func _ready() -> void:
	z_index = 4000


## Converts a width in screen pixels to world units at the current zoom.
func px(pixels: float) -> float:
	return pixels / editor.camera.zoom.x


func _draw() -> void:
	if editor == null or editor.road == null:
		return
	var road = editor.road
	if _cache_rev != road.revision():
		_cache_rev = road.revision()
		_problems = road.get_problems()
		_spawners = road.get_spawners()
		_connectors = []
	if editor.show_connectors and _connectors.is_empty():
		_connectors = road.get_connectors()
	if editor.show_connectors:
		for c in _connectors:
			if int(c.level) != editor.level:
				continue
			var pts: PackedVector2Array = c.path
			if pts.size() >= 2:
				draw_polyline(pts, TURN_COLORS.get(c.turn, Color.WHITE), px(1.5))
				_arrow_head(pts[pts.size() - 1], (pts[pts.size() - 1] - pts[pts.size() - 2]).normalized(),
					TURN_COLORS.get(c.turn, Color.WHITE))
	for id in editor.selection.segments:
		_outline(road.segment_outline(id), SELECT, 2.5)
	for id in editor.selection.nodes:
		_outline(road.node_outline(id), SELECT, 2.5)
		var n: Dictionary = road.get_node(id)
		if not n.is_empty():
			draw_circle(n.pos, px(5), SELECT)
	var font := ThemeDB.fallback_font
	for sp in _spawners:
		if int(sp.level) != editor.level:
			continue
		_spawn_marker(sp, font)
	var car: Dictionary = editor.sim.car_info if editor.sim and editor.sim.selected_car != 0 else {}
	if not car.is_empty():
		var route: PackedVector2Array = car.route
		if route.size() >= 2:
			draw_polyline(route, ROUTE, px(3.0))
			draw_circle(route[route.size() - 1], px(6), ROUTE)
		var tail: Vector2 = car.pos - (car.dir as Vector2) * float(car.length)
		draw_circle((car.pos + tail) * 0.5, maxf(px(12), 4.0), SELECT, false, px(2.5))
	for p in _problems:
		if int(p.level) != editor.level:
			continue
		var col := ERROR if p.severity == "error" else WARNING
		draw_circle(p.pos, px(9), col)
		draw_circle(p.pos, px(9), Color.BLACK, false, px(1.5))
		draw_string(font, p.pos + Vector2(-px(2.5), px(5)), "!", HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(15)), Color.BLACK)
	if editor.tool:
		editor.tool.draw(self)


## A ring at the road end with arrows for traffic in and out, and the rate.
func _spawn_marker(sp: Dictionary, font: Font) -> void:
	var p: Vector2 = sp.pos
	var d: Vector2 = sp.dir # into the map
	var col := SPAWN if sp.active else ERROR
	var r := px(10)
	draw_circle(p, r, Color(0.05, 0.08, 0.1, 0.85))
	draw_circle(p, r, col, false, px(2))
	var side := Vector2(-d.y, d.x)
	if float(sp.rate) > 0.0:
		var a := p + side * px(3.5)
		_arrow_head(a + d * px(6), d, col)
		draw_line(a - d * px(5), a + d * px(2), col, px(1.5))
	if sp.sink:
		var b := p - side * px(3.5)
		_arrow_head(b - d * px(6), -d, col)
		draw_line(b + d * px(5), b - d * px(2), col, px(1.5))
	var label := "%d/h" % int(sp.rate) if float(sp.rate) > 0.0 else "out"
	draw_string(font, p + Vector2(px(13), px(5)), label, HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(13)), col)


func _outline(pts: PackedVector2Array, col: Color, width: float) -> void:
	if pts.size() < 2:
		return
	var closed := pts.duplicate()
	closed.append(pts[0])
	draw_polyline(closed, col, px(width))


func draw_outline(pts: PackedVector2Array, col: Color, width: float) -> void:
	_outline(pts, col, width)


func _arrow_head(tip: Vector2, dir: Vector2, col: Color) -> void:
	var s := px(7)
	var side := Vector2(-dir.y, dir.x)
	draw_colored_polygon(PackedVector2Array([tip, tip - dir * s + side * s * 0.5, tip - dir * s - side * s * 0.5]), col)


## Snap indicator used by the drawing tools.
func draw_snap(snap: Dictionary) -> void:
	if snap.is_empty():
		return
	var p: Vector2 = snap.pos
	match snap.kind:
		"node":
			draw_circle(p, px(8), SELECT, false, px(2))
		"segment":
			var s := px(7)
			draw_polyline(PackedVector2Array([p + Vector2(0, -s), p + Vector2(s, 0), p + Vector2(0, s), p + Vector2(-s, 0), p + Vector2(0, -s)]), SELECT, px(2))
		_:
			var s := px(5)
			draw_line(p - Vector2(s, 0), p + Vector2(s, 0), Color.WHITE, px(1.5))
			draw_line(p - Vector2(0, s), p + Vector2(0, s), Color.WHITE, px(1.5))
