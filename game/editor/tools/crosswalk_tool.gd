class_name CrosswalkTool
extends EditorTool
## W: click a road to add a crossing. Near a junction it marks that leg's
## crossing; elsewhere it adds a mid-block crossing. 1 zebra, 2 signal
## (walk phases at a junction, a push button mid-block), 3 uncontrolled;
## B adds a bike crossing beside it; I adds a refuge island (roads with a
## median). Shift-click removes the nearest crossing.

const KINDS := {"zebra": "zebra", "signal": "signal", "uncontrolled": "uncontrolled"}
## A click this close to a junction marks the leg's crossing.
const LEG_DISTANCE := 20.0

var kind := "zebra"
var bike := false
var refuge := false
var _hover := {}


func hint() -> String:
	var extras: Array = []
	if bike:
		extras.append("bike crossing")
	if refuge:
		extras.append("refuge")
	return "Click a road to add a %s crossing%s · 1 zebra, 2 signal, 3 uncontrolled · B bike · I refuge · Shift-click removes" % [
		kind, (" (+ " + ", ".join(extras) + ")") if not extras.is_empty() else ""]


func activate() -> void:
	_hover = {}


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_hover = _target(mouse)
		return false
	for k in [[KEY_1, "zebra"], [KEY_2, "signal"], [KEY_3, "uncontrolled"]]:
		if key_pressed(event, k[0]):
			kind = k[1]
			editor.ui.set_status(hint())
			return true
	if key_pressed(event, KEY_B):
		bike = not bike
		editor.ui.set_status(hint())
		return true
	if key_pressed(event, KEY_I):
		refuge = not refuge
		editor.ui.set_status(hint())
		return true
	if key_pressed(event, KEY_ESCAPE):
		editor.set_tool("select")
		return true
	if not is_left_press(event):
		return false
	var mb := event as InputEventMouseButton
	if mb.shift_pressed:
		_remove_nearest(mouse)
		return true
	var t := _target(mouse)
	if t.is_empty():
		editor.notify("Crossings go on a road: click the road where people should cross.")
		return true
	if t.end >= 0:
		var seg: Dictionary = editor.road.get_segment(t.segment)
		var rules: Dictionary = seg.ends[t.end]
		editor.road.set_end_rules(t.segment, t.end, {"crossing": {"kind": kind, "bike": bike, "refuge": refuge}})
		editor.notify("%s crossing on this junction leg%s." % [kind.capitalize(),
			" (signals get walk phases)" if kind == "signal" and rules.junction else ""])
	else:
		var id: int = editor.road.add_crossing(t.segment, t.u, kind, bike, refuge)
		if id == 0:
			editor.notify("Could not add a crossing there.")
		else:
			editor.notify("%s crossing added%s." % [kind.capitalize(), " (push button)" if kind == "signal" else ""])
	editor.select("segments", int(t.segment), false)
	return true


## {segment, u, end (-1 mid-block), pos} under the mouse, or {}.
func _target(mouse: Vector2) -> Dictionary:
	var hit: Dictionary = editor.road.pick(mouse, editor.pick_radius(), editor.level)
	if hit.type != "segment":
		return {}
	var seg: Dictionary = editor.road.get_segment(hit.id)
	if seg.kind == "footpath":
		return {}
	var u: float = hit.u
	var length: float = seg.length
	var end := -1
	if u * length < LEG_DISTANCE and seg.ends[0].junction:
		end = 0
	elif (1.0 - u) * length < LEG_DISTANCE and seg.ends[1].junction:
		end = 1
	return {"segment": int(hit.id), "u": u, "end": end, "pos": editor.road.segment_point(hit.id, u),
		"dir": editor.road.segment_tangent(hit.id, u), "width": float(seg.profile.get("width", 10.0))}


func _remove_nearest(mouse: Vector2) -> void:
	var best := {}
	var best_d := editor.pick_radius() * 3.0
	for c in editor.road.get_crossings(editor.level):
		var d := (c.mid as Vector2).distance_to(mouse)
		if d < best_d:
			best_d = d
			best = c
	if best.is_empty():
		editor.notify("No crossing here to remove.")
		return
	if int(best.end) >= 0:
		editor.road.set_end_rules(best.segment, best.end, {"crossing": {"kind": "none", "bike": false, "refuge": false}})
	else:
		editor.road.remove_crossing(best.segment, best.id)
	editor.notify("Crossing removed.")


func draw(o: EditorOverlay) -> void:
	for c in editor.road.get_crossings(editor.level):
		o.draw_line(c.a, c.b, Color(1, 1, 1, 0.35), o.px(2))
	if _hover.is_empty():
		return
	var dir: Vector2 = (_hover.dir as Vector2).normalized()
	var side := Vector2(-dir.y, dir.x)
	var half: float = _hover.width * 0.5
	var col := EditorOverlay.SELECT if _hover.end < 0 else EditorOverlay.WARNING
	o.draw_line(_hover.pos - side * half, _hover.pos + side * half, col, o.px(4))
	var font := ThemeDB.fallback_font
	var label := "%s, mid-block" % kind if _hover.end < 0 else "%s, junction leg" % kind
	o.draw_string(font, _hover.pos + side * (half + o.px(8)), label, HORIZONTAL_ALIGNMENT_LEFT, -1, int(o.px(13)), col)
