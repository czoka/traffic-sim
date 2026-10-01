class_name BuildingTool
extends EditorTool
## H: place a building on a lot beside a street. 1 home, 2 shop, 3 office;
## Tab cycles the variants of that kind. The lot snaps to the nearest street
## edge and faces it; red means it would stand on a road or another building.
## Shift-click removes the building under the cursor.

const KINDS := ["home", "shop", "office"]

var kind := "home"
var variant := {} # kind -> type id
var _types: Array = []
var _snap := {}


func activate() -> void:
	_types = editor.road.building_types()
	for k in KINDS:
		if not variant.has(k):
			for t in _types:
				if t.kind == k and not t.city:
					variant[k] = t.id
					break
	_snap = {}


func _type() -> Dictionary:
	for t in _types:
		if t.id == variant.get(kind, ""):
			return t
	return {}


func hint() -> String:
	var t := _type()
	var label: String = t.get("label", kind)
	var what := ""
	if kind == "home":
		what = "%d household%s" % [t.get("households", 1), "" if int(t.get("households", 1)) == 1 else "s"]
	elif kind == "shop":
		what = "%d customers at once" % int(t.get("slots", 0))
	else:
		what = "%d desks" % int(t.get("desks", 0))
	return "Click beside a street to place a %s (%s) · 1 home, 2 shop, 3 office · Tab: next variant · Shift-click removes" % [label, what]


func _cycle() -> void:
	var ids: Array = []
	for t in _types:
		if t.kind == kind and not t.city:
			ids.append(t.id)
	if ids.is_empty():
		return
	var i := ids.find(variant.get(kind, ""))
	variant[kind] = ids[(i + 1) % ids.size()]


func input(event: InputEvent) -> bool:
	var mouse := editor.mouse_world()
	if event is InputEventMouseMotion:
		_snap = editor.road.snap_building(variant.get(kind, ""), mouse, editor.level)
		return false
	for k in [[KEY_1, "home"], [KEY_2, "shop"], [KEY_3, "office"]]:
		if key_pressed(event, k[0]):
			kind = k[1]
			editor.ui.set_status(hint())
			_snap = editor.road.snap_building(variant.get(kind, ""), mouse, editor.level)
			return true
	if key_pressed(event, KEY_TAB):
		_cycle()
		editor.ui.set_status(hint())
		_snap = editor.road.snap_building(variant.get(kind, ""), mouse, editor.level)
		return true
	if key_pressed(event, KEY_ESCAPE):
		editor.set_tool("select")
		return true
	if not is_left_press(event):
		return false
	var mb := event as InputEventMouseButton
	if mb.shift_pressed:
		var id: int = editor.road.pick_building(mouse, editor.level)
		if id != 0:
			editor.road.remove_building(id)
			editor.notify("Building removed.")
		return true
	_snap = editor.road.snap_building(variant.get(kind, ""), mouse, editor.level)
	if not _snap.get("ok", false):
		editor.notify("Buildings go beside a street: click next to one.")
		return true
	if _snap.blocked:
		editor.notify("There's no room there: it would stand on a road or another building.")
		return true
	var new_id: int = editor.road.add_building(variant.get(kind, ""), _snap.pos, _snap.dir, editor.level)
	editor.notify("%s placed." % String(_type().get("label", "Building")))
	editor.select("buildings", new_id, false)
	return true


func draw(o: EditorOverlay) -> void:
	if not _snap.get("ok", false):
		return
	var pts: PackedVector2Array = _snap.corners
	var t := _type()
	var col: Color = EditorOverlay.rgb(int(t.get("color", 0xb0b0b0)))
	col.a = 0.55
	if _snap.blocked:
		col = Color(EditorOverlay.ERROR, 0.55)
	o.draw_colored_polygon(pts, col)
	o.draw_outline(pts, EditorOverlay.ERROR if _snap.blocked else EditorOverlay.SELECT, 2.0)
	var door: Vector2 = _snap.pos
	o.draw_circle(door, o.px(4), Color.WHITE)
