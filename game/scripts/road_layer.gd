class_name RoadLayer
extends Node2D
## Draws the road network schematically: asphalt, edge lines, lane dividers.
## Geometry comes from the C++ map; this node only redraws when told to.

const ASPHALT := Color(0.18, 0.19, 0.21)
const EDGE := Color(0.92, 0.92, 0.88, 0.9)
const DIVIDER := Color(0.92, 0.92, 0.88, 0.45)
const DASH := 3.0
const GAP := 6.0

var _lines: Array = []


func rebuild(sim) -> void:
	_lines = sim.get_road_lines(8.0)
	queue_redraw()


func _draw() -> void:
	# Asphalt first, then markings on top.
	for line in _lines:
		if line.kind == 0:
			draw_polyline(line.points, ASPHALT, line.width)
	for line in _lines:
		match int(line.kind):
			1:
				draw_polyline(line.points, EDGE, line.width)
			2:
				_draw_dashed(line.points, DIVIDER, line.width)


func _draw_dashed(points: PackedVector2Array, color: Color, width: float) -> void:
	# Dashes continue across polyline vertices so curves look even.
	var on := true
	var left := DASH
	for i in range(points.size() - 1):
		var a := points[i]
		var b := points[i + 1]
		var seg_len := a.distance_to(b)
		var t := 0.0
		while t < seg_len:
			var step := minf(left, seg_len - t)
			if on:
				draw_line(a.lerp(b, t / seg_len), a.lerp(b, (t + step) / seg_len), color, width)
			t += step
			left -= step
			if left <= 0.0:
				on = not on
				left = DASH if on else GAP
