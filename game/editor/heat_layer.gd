class_name HeatLayer
extends Node2D
## M7 heatmaps: road lanes on the level being edited, coloured by what the sim
## has seen there over the last few minutes - speed (of the limit), wait time
## (seconds stopped per vehicle) or throughput (vehicles an hour). Lanes with no
## traffic lately are left out. Redrawn once a second; zoom needs no redraw.

const MODES := ["off", "speed", "wait", "flow"]
const MODE_LABELS := ["Heatmap: off", "Speed", "Wait time", "Throughput"]
const WIDTH := 2.4 # m, about a lane
const WAIT_RED := 30.0 # s per vehicle shown fully red
const FLOW_FULL := 1200.0 # vehicles an hour shown at the darkest

var editor: MapEditor
var mode := "off"

var _lines: Array = []
var _lanes := PackedInt32Array()
var _values := PackedFloat32Array()
var _key := ""
var _timer := 0.0
var _zoom := 1.0


func set_mode(m: String) -> void:
	mode = m if m in MODES else "off"
	visible = mode != "off"
	_timer = 1e9
	refresh()


func refresh() -> void:
	if editor == null or mode == "off":
		return
	_values = editor.road.sim_lane_heat(mode)
	var key := "%d:%d:%d" % [editor.road.revision(), editor.level, _values.size()]
	if key != _key:
		_key = key
		var d: Dictionary = editor.road.sim_lane_lines(editor.level)
		_lines = d.lines
		_lanes = d.lanes
	z_index = (editor.level + 2) * 10 + 4 # over the level's paint, under its cars
	queue_redraw()


func _process(delta: float) -> void:
	if mode == "off":
		return
	_timer += delta
	if _timer >= 1.0:
		_timer = 0.0
		refresh()
	elif absf(editor.camera.zoom.x / _zoom - 1.0) > 0.15:
		queue_redraw() # keep lanes at least a few pixels wide


func _draw() -> void:
	if mode == "off":
		return
	_zoom = editor.camera.zoom.x
	var width := maxf(WIDTH, 3.0 / maxf(_zoom, 0.01))
	for k in _lines.size():
		var li: int = _lanes[k]
		if li >= _values.size():
			continue
		var v: float = _values[li]
		if v < 0.0:
			continue
		var pts: PackedVector2Array = _lines[k]
		if pts.size() >= 2:
			draw_polyline(pts, color_for(mode, v), width)


## Speed and wait: red (bad) - amber - green (good). Throughput: pale to deep violet.
static func color_for(m: String, v: float) -> Color:
	match m:
		"speed":
			return _ramp(clampf(v, 0.0, 1.0))
		"wait":
			return _ramp(1.0 - clampf(v / WAIT_RED, 0.0, 1.0))
		"flow":
			var t := clampf(v / FLOW_FULL, 0.0, 1.0)
			return Color(0.80, 0.86, 0.98, 0.75).lerp(Color(0.36, 0.13, 0.62, 0.92), t)
	return Color.WHITE


static func _ramp(t: float) -> Color:
	var red := Color(0.86, 0.22, 0.20, 0.85)
	var amber := Color(0.96, 0.72, 0.18, 0.85)
	var green := Color(0.30, 0.78, 0.42, 0.85)
	return red.lerp(amber, t * 2.0) if t < 0.5 else amber.lerp(green, (t - 0.5) * 2.0)


## Legend text for the sim bar.
static func legend(m: String) -> String:
	match m:
		"speed":
			return "red: stopped · amber: half the limit · green: free flow"
		"wait":
			return "green: no wait · amber: %d s · red: %d s or more stopped per vehicle" % [int(WAIT_RED / 2), int(WAIT_RED)]
		"flow":
			return "pale: few · violet: %d or more vehicles an hour" % int(FLOW_FULL)
	return ""
