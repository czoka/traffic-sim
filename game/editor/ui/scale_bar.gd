class_name ScaleBar
extends Control
## A map-style scale bar: a bar of a round length (1, 2 or 5 × 10^n metres)
## that is between MIN_PX and MAX_PX long at the current zoom, labelled with
## that length. The exact zoom (screen pixels per metre) is in the tooltip.

const MIN_PX := 50.0
const MAX_PX := 125.0
const COLOR := Color(0.75, 0.78, 0.82)

var zoom := 1.0:
	set(z):
		if z == zoom:
			return
		zoom = z
		tooltip_text = "Zoom %s px/m" % (("%.3f" % z) if z < 1.0 else ("%.2f" % z))
		queue_redraw()


func _init() -> void:
	custom_minimum_size = Vector2(MAX_PX + 70.0, 0)
	mouse_filter = Control.MOUSE_FILTER_PASS
	tooltip_text = "Zoom 1.00 px/m"


## The round length shown at this zoom, in metres.
static func bar_metres(z: float) -> float:
	var m := pow(10.0, floor(log(MIN_PX / z) / log(10.0)))
	for step in [1.0, 2.0, 5.0, 10.0]:
		if m * step * z >= MIN_PX:
			return m * step
	return m * 10.0


static func label(metres: float) -> String:
	if metres >= 1000.0:
		return "%s km" % str(metres / 1000.0)
	if metres >= 1.0:
		return "%d m" % int(round(metres))
	return "%d cm" % int(round(metres * 100.0))


func _draw() -> void:
	var metres := bar_metres(zoom)
	var w := metres * zoom
	var font := get_theme_default_font()
	var fs := get_theme_default_font_size()
	var y := size.y * 0.5 + 4.0
	var x0 := 4.0
	draw_line(Vector2(x0, y), Vector2(x0 + w, y), COLOR, 2.0)
	for x in [x0, x0 + w]:
		draw_line(Vector2(x, y - 6.0), Vector2(x, y + 1.0), COLOR, 2.0)
	draw_line(Vector2(x0 + w * 0.5, y - 3.0), Vector2(x0 + w * 0.5, y), COLOR, 1.0)
	draw_string(font, Vector2(x0 + w + 8.0, y + fs * 0.35), label(metres), HORIZONTAL_ALIGNMENT_LEFT, -1, fs, COLOR)
