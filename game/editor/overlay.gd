class_name EditorOverlay
extends Node2D
## Everything drawn on top of the roads: selection, hover, Bézier handles,
## connectors, spawn points, bus stops, depots and routes, signal heads, the
## selected car's route, problem markers and the active tool's preview.

const SELECT := Color(0.35, 0.72, 1.0, 1.0)
const HOVER := Color(1.0, 1.0, 1.0, 0.55)
const ERROR := Color(0.93, 0.33, 0.29)
const WARNING := Color(0.96, 0.72, 0.2)
const SPAWN := Color(0.4, 0.85, 0.95)
const ROUTE := Color(0.35, 0.9, 1.0, 0.85)
const STOP := Color(0.98, 0.85, 0.2)
const DEPOT := Color(0.95, 0.95, 0.98)
const LIGHTS := {
	"red": Color(0.93, 0.2, 0.18),
	"amber": Color(0.98, 0.7, 0.1),
	"green": Color(0.25, 0.85, 0.35),
	"yield": Color(0.6, 0.95, 0.35),
}
const WALK := {
	"walk": Color(0.95, 0.97, 1.0),
	"flashing": Color(1.0, 0.55, 0.15),
	"dont_walk": Color(0.93, 0.25, 0.2),
}
const PERSON := Color(1.0, 0.85, 0.35)
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
var _stops: Array = []
var _depots: Array = []


## 0xRRGGBB -> Color
static func rgb(c: int) -> Color:
	return Color(((c >> 16) & 0xff) / 255.0, ((c >> 8) & 0xff) / 255.0, (c & 0xff) / 255.0)


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
		_stops = road.get_stops()
		_depots = road.get_depots()
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
	_draw_transit(font)
	for h in road.sim_signal_heads(editor.level):
		var p: Vector2 = h.pos
		var side := Vector2(-(h.dir as Vector2).y, (h.dir as Vector2).x)
		var at := p + side * px(0.0) + (h.dir as Vector2) * px(4)
		draw_circle(at, maxf(px(4.5), 0.9), Color(0.05, 0.05, 0.06, 0.9))
		draw_circle(at, maxf(px(3.2), 0.65), LIGHTS.get(h.light, Color.WHITE))
	_draw_people(font)
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


## Bus stops (a sign beside the road), depots and route lines.
func _draw_transit(font: Font) -> void:
	var playing: bool = editor.sim != null and editor.sim.playing
	for d in _depots:
		if int(d.level) != editor.level:
			continue
		if not playing:
			for r in d.routes:
				var path: PackedVector2Array = r.path
				if path.size() >= 2:
					var c := rgb(int(r.color))
					c.a = 0.7
					draw_polyline(path, c, px(2.5))
		var p: Vector2 = d.pos
		var s := px(9)
		draw_rect(Rect2(p - Vector2(s, s), Vector2(s, s) * 2.0), Color(0.1, 0.12, 0.16, 0.9))
		draw_rect(Rect2(p - Vector2(s, s), Vector2(s, s) * 2.0), DEPOT if d.active else ERROR, false, px(2))
		draw_string(font, p + Vector2(-px(4), px(5)), "D", HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(14)), DEPOT)
		draw_string(font, p + Vector2(px(12), px(5)), String(d.name), HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(12)), DEPOT)
	for st in _stops:
		if int(st.level) != editor.level:
			continue
		var p: Vector2 = st.pos
		var dir: Vector2 = (st.dir as Vector2).normalized()
		var right := Vector2(-dir.y, dir.x)
		var sign_at := p + right * maxf(px(10), 4.5)
		var col := STOP if st.served else ERROR
		var r := px(6) if st.kind != "main_station" else px(9)
		draw_circle(sign_at, r, Color(0.1, 0.1, 0.12, 0.9))
		draw_circle(sign_at, r, col, false, px(2))
		var letter := "H" if st.kind == "main_station" else "B"
		draw_string(font, sign_at + Vector2(-px(3.5), px(4.5)), letter, HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(12)), col)
		if editor.camera.zoom.x > 1.2:
			draw_string(font, sign_at + Vector2(r + px(3), px(4)), String(st.name), HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(11)), col)


## Walk lights at signal crossings, people waiting at stops, the selected person.
func _draw_people(font: Font) -> void:
	var road = editor.road
	var blink := int(Time.get_ticks_msec() / 400) % 2 == 0
	if editor.camera.zoom.x > 0.8:
		for w in road.sim_walk_lights(editor.level):
			var col: Color = WALK.get(w.walk, Color.WHITE)
			if w.walk == "flashing" and not blink:
				col = Color(col.r, col.g, col.b, 0.25)
			var a: Vector2 = w.a
			var b: Vector2 = w.b
			var dir := (b - a).normalized()
			var s := maxf(px(3.5), 0.45)
			for p in [a - dir * s * 1.6, b + dir * s * 1.6]:
				draw_rect(Rect2(p - Vector2(s, s), Vector2(s, s) * 2.0), Color(0.05, 0.05, 0.06, 0.9))
				draw_rect(Rect2(p - Vector2(s, s) * 0.7, Vector2(s, s) * 1.4), col)
			if w.push_button:
				var mid := (a + b) * 0.5
				draw_circle(mid, maxf(px(3.5), 0.5), Color(0.05, 0.05, 0.06, 0.9))
				draw_circle(mid, maxf(px(2.5), 0.35), LIGHTS.get(w.car, Color.WHITE))
	if editor.sim and editor.sim.has_people() and editor.camera.zoom.x > 0.6:
		for st in editor.sim.stop_stats:
			if int(st.waiting) > 0:
				draw_string(font, (st.pos as Vector2) + Vector2(px(14), -px(10)), "%d waiting" % int(st.waiting),
					HORIZONTAL_ALIGNMENT_LEFT, -1, int(px(11)), STOP)
	var ped: Dictionary = editor.sim.ped_info if editor.sim and editor.sim.selected_ped != 0 else {}
	if not ped.is_empty():
		var route: PackedVector2Array = ped.route
		if route.size() >= 2:
			draw_polyline(route, PERSON, px(2.5))
			draw_circle(route[route.size() - 1], px(5), PERSON)
		draw_circle(ped.pos, maxf(px(9), 1.5), PERSON, false, px(2.5))


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
