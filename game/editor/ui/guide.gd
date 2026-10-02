class_name Guide
extends PanelContainer
## M7 tutorial: a checklist of steps to a working town - a street, homes, a
## shop, bus stops, a depot and a route, then Play. Each step ticks itself off
## from the map and the sim, and the next one says which keys to press.

signal finished

const STEPS := [
	{"id": "street", "title": "Draw a street",
		"hint": "Press R (Road). Click on Loop Road or High Street to start from it, click about 100 m away, then double-click to finish. Click it afterwards (V) to change its lanes in the inspector. A street that ends nowhere shows a warning in Problems - fine for now."},
	{"id": "homes", "title": "Build 3 homes",
		"hint": "Press H (Building), then 1 for homes. Click beside Loop Road to place one; Tab switches the house type. Red means the lot doesn't fit there."},
	{"id": "shop", "title": "Open a shop",
		"hint": "Still in the building tool, press 2 for shops and click beside a street. A grocery feeds the homes; people also eat out at fast food places."},
	{"id": "stops", "title": "Add 2 bus stops",
		"hint": "Press K (Bus stop) and click the side of a road: one on Loop Road near the homes, and one on High Street between the two ends of Loop Road (so the bus can go round)."},
	{"id": "depot", "title": "Add a bus depot",
		"hint": "Buses start at a depot on a loose road end: press D and click the far end of Depot Lane (the short dead end off Loop Road)."},
	{"id": "route", "title": "Draw a bus route",
		"hint": "Press U (Bus route), click the depot, then each stop in order, and click the depot again (or press Enter) to save. If Problems says the route can't reach a stop, move that stop to the other side of the road or between the ends of Loop Road."},
	{"id": "play", "title": "Press Play",
		"hint": "Press Space or the Play button. Coaches bring newcomers to the main station, and they walk or take your bus to the homes."},
	{"id": "residents", "title": "Wait for people to move in",
		"hint": "Speed the sim up in the bottom bar. Click a home to see who lives there, or a person to follow them."},
	{"id": "bus", "title": "See a bus serve your stops",
		"hint": "A bus leaves the depot every headway and calls at the stops in order. Click a stop to see who waits and boards (people take the bus when it beats walking, so in a small town most walk)."},
]

var editor: MapEditor
var active := false
var done := {}

var _base_segments := 0
var _list: VBoxContainer
var _hint: Label
var _title: Label
var _footer: Label
var _timer := 0.0


func _ready() -> void:
	add_theme_stylebox_override("panel", EditorUI._panel_style(Color(0.10, 0.12, 0.16, 0.95)))
	custom_minimum_size = Vector2(340, 0)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 4)
	add_child(box)
	var head := HBoxContainer.new()
	_title = Label.new()
	_title.text = "Tutorial: build a working town"
	_title.add_theme_font_size_override("font_size", 15)
	_title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(_title)
	var close := Button.new()
	close.text = "×"
	close.flat = true
	close.focus_mode = Control.FOCUS_NONE
	close.tooltip_text = "Close the tutorial (Help → Tutorial opens it again)"
	close.pressed.connect(stop)
	head.add_child(close)
	box.add_child(head)
	_list = VBoxContainer.new()
	box.add_child(_list)
	_hint = Label.new()
	_hint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_hint.custom_minimum_size = Vector2(320, 0)
	_hint.add_theme_color_override("font_color", Color(0.98, 0.86, 0.45))
	box.add_child(_hint)
	_footer = Label.new()
	_footer.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_footer.custom_minimum_size = Vector2(320, 0)
	_footer.add_theme_font_size_override("font_size", 12)
	_footer.add_theme_color_override("font_color", Color(0.95, 0.5, 0.45))
	box.add_child(_footer)
	visible = false


## Starts the checklist on the map as it is now (normally a new city).
func start() -> void:
	active = true
	done = {}
	_base_segments = editor.road.segment_ids().size()
	visible = true
	refresh()


func stop() -> void:
	active = false
	visible = false


func _process(delta: float) -> void:
	if not active:
		return
	_timer += delta
	if _timer >= 0.5:
		_timer = 0.0
		refresh()


## Ticks off what is done; steps stay ticked once done.
func refresh() -> void:
	if not active:
		return
	for s in STEPS:
		if not done.get(s.id, false) and _check(s.id):
			done[s.id] = true
	for c in _list.get_children():
		c.queue_free()
	var current := -1
	for i in STEPS.size():
		var s: Dictionary = STEPS[i]
		var ok: bool = done.get(s.id, false)
		if not ok and current < 0:
			current = i
		var l := Label.new()
		l.text = ("✓  " if ok else ("▶  " if current == i else "○  ")) + String(s.title)
		l.add_theme_color_override("font_color", Color(0.55, 0.85, 0.6) if ok else (Color.WHITE if current == i else Color(0.6, 0.62, 0.66)))
		_list.add_child(l)
	if current < 0:
		_hint.text = "Your town works! Try the heatmaps (M) to see where traffic waits, click a junction for its numbers, and the Market to buy and sell buildings."
		_title.text = "Tutorial: done"
		finished.emit()
	else:
		_hint.text = String(STEPS[current].hint)
	var errors := 0
	for p in editor.road.get_problems():
		if p.severity == "error":
			errors += 1
	_footer.visible = errors > 0
	_footer.text = "%d error%s on the map (see Problems, bottom right): fix them before Play." % [errors, "" if errors == 1 else "s"]


## Whether a step is done now.
func _check(id: String) -> bool:
	var road = editor.road
	match id:
		"street":
			return road.segment_ids().size() > _base_segments
		"homes":
			return _count_kind("home") >= 3
		"shop":
			return _count_kind("shop") >= 1
		"stops":
			var n := 0
			for st in road.get_stops():
				if st.kind != "main_station":
					n += 1
			return n >= 2
		"depot":
			return not road.get_depots().is_empty()
		"route":
			for p in road.get_problems():
				if p.code == "route_broken":
					return false
			for d in road.get_depots():
				for r in d.routes:
					if (r.stops as Array).size() >= 2:
						return true
			return false
		"play":
			return editor.sim.playing or int(editor.sim.stats.get("tick", 0)) > 0 and done.get("route", false)
		"residents":
			return int(editor.sim.city.get("residents", 0)) > 0
		"bus":
			return int(editor.sim.stats.get("bus_stops_served", 0)) > 0
	return false


func _count_kind(kind: String) -> int:
	var n := 0
	for b in editor.road.get_buildings(0):
		if b.kind == kind:
			n += 1
	return n
