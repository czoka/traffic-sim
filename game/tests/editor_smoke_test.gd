extends SceneTree
## Road editor smoke test: loads the real editor scene and drives it through
## the same calls the tools and inspector make. Run headless:
##   godot --headless --path game --script res://tests/editor_smoke_test.gd

var _failures := 0


func _initialize() -> void:
	_run.call_deferred()


func _check(ok: bool, what: String) -> void:
	print(("  ok    " if ok else "  FAIL  ") + what)
	if not ok:
		_failures += 1


func _frames(n: int) -> void:
	for i in n:
		await process_frame


func _run() -> void:
	print("Road editor smoke test (Godot %s)" % Engine.get_version_info().string)
	if not ClassDB.class_exists("RoadEditor"):
		print("  FAIL  RoadEditor extension not loaded (build it first)")
		quit(1)
		return
	var ed: MapEditor = load("res://main.tscn").instantiate()
	root.add_child(ed)
	await _frames(3)
	var road = ed.road
	_check(road != null, "editor scene created a RoadEditor")

	ed.load_demo("town")
	await _frames(2)
	# Headless viewports are tiny, so "fit" zooms far out; use a normal zoom
	# so the 10 px pick radius is 5 m like on a real screen.
	ed.camera.zoom = Vector2.ONE * 2.0
	var st: Dictionary = road.get_stats()
	_check(st.junctions >= 3 and st.errors == 0, "demo town: %d junctions, %d errors, %d warnings" % [st.junctions, st.errors, st.warnings])
	_check(ed.view.get_child_count() >= 3, "map view built %d mesh batches" % ed.view.get_child_count())

	# Road tool path: two clicks snapped to the grid, the second onto an existing road.
	var before: int = road.segment_ids().size()
	var start := ed.snap_point(Vector2(-300.4, -150.2))
	_check(start.kind == "grid" and start.pos == Vector2(-300, -150), "grid snap rounds to 1 m (%s)" % str(start))
	var target: Vector2 = road.get_node(road.node_ids()[0]).pos
	var onto := ed.snap_point(target + Vector2(1, 1))
	_check(onto.kind == "node", "clicking near a node snaps to it")
	var ids: PackedInt64Array = road.add_road([MapEditor.to_ref(start), MapEditor.to_ref(onto)], ed.road_template, 0, 50.0)
	await _frames(1)
	_check(ids.size() == 1 and road.segment_ids().size() == before + 1, "road tool adds a connected road")
	if ids.is_empty():
		print("%d check(s) failed" % (_failures))
		quit(max(1, _failures))
		return
	var origin := Vector2(0, -150) # empty ground in the demo town
	var angled := ed.snap_point(origin + Vector2(10.3, 4.1), origin)
	_check(angled.kind == "angle" and is_equal_approx((angled.pos - origin).angle(), deg_to_rad(15.0)),
		"15° angle snap from the previous point")

	# Select + inspector: change lanes through params, then undo.
	ed.select("segments", ids[0], false)
	await _frames(1)
	_check(ed.ui.inspector._segment_box.visible, "inspector shows the selected road")
	var p: Dictionary = road.get_segment(ids[0]).params
	p["forward"] = 2
	_check(road.set_profile_params(ids[0], p) == "", "set lanes via inspector params")
	_check(int(road.get_segment(ids[0]).params.forward) == 2, "road now has 2 forward lanes")
	road.set_end_rules(ids[0], 1, {"left": "turn_lane", "right": "allowed", "turn_lane_length": 30.0})
	ed.undo()
	ed.undo()
	await _frames(1)
	_check(int(road.get_segment(ids[0]).params.forward) == 1, "undo restores the lane count")

	# Curve tool path with tangent continuation.
	var c: int = road.add_curve({"pos": Vector2(-300, -150)}, Vector2(-350, -250), {"pos": Vector2(-450, -250)}, ed.road_template, 0, 40.0)
	_check(c != 0 and road.get_segment(c).curve == "bezier", "curve tool adds a Bézier road")

	# Lane paint: paint and read back a zone.
	var avenue := 0
	for sid in road.segment_ids():
		if road.get_segment(sid).profile.lanes.size() >= 8:
			avenue = sid
			break
	road.set_no_change(avenue, 3, 0.1, 0.4, true, true)
	_check(road.get_segment(avenue).no_change.size() >= 1, "lane paint stores a no-change zone")
	_check(road.set_lane_type(avenue, road.get_segment(avenue).profile.lanes[3].id, "bike") == "", "lane type change")

	# Levels: moving a road to +1 removes its junctions with ground roads.
	road.set_level(ids[0], 1)
	_check(int(road.get_segment(ids[0]).level) == 1, "road moved to level +1")
	ed.set_level(1)
	ed.set_level(0)

	# Save -> load -> save through the editor's JSON, and autosave.
	var text: String = road.save_json()
	var r: Dictionary = road.load_json(text)
	_check(r.ok and road.save_json() == text, "save -> load -> save is identical")
	ed.autosave()
	_check(FileAccess.file_exists(MapEditor.AUTOSAVE_PATH), "autosave written to user://")
	var bad: Dictionary = road.load_json("{\"format\":\"traffic-sim-map\",\"version\":9}")
	_check(not bad.ok and String(bad.error).contains("newer"), "newer save versions are rejected with a message")

	# Test grid (the M1 gate network) through the editor.
	ed.load_demo("grid")
	await _frames(2)
	st = road.get_stats()
	_check(st.junctions >= 50 and st.errors == 0, "test grid: %d junctions, %d errors, geometry %.1f ms" % [st.junctions, st.errors, st.build_ms])

	# The POC's version 1 ring file is upgraded on import.
	var v1: Dictionary = road.load_json(FileAccess.get_file_as_string("res://maps/poc_ring_v1.json"))
	_check(v1.ok and int(v1.migrated_from) == 1, "v1 map file migrates to version 2")
	# The committed gate network loads and has no errors.
	var grid: Dictionary = road.load_json(FileAccess.get_file_as_string("res://maps/test_grid_v2.json"))
	_check(grid.ok and int(road.get_stats().errors) == 0, "maps/test_grid_v2.json loads without errors")

	# User-saved profiles appear in the road template list.
	var profile: Dictionary = road.get_segment(road.segment_ids()[0]).profile
	var n_before: int = ed.user_presets.size()
	ed.save_user_preset("smoke test profile", profile)
	_check(ed.user_presets.size() == n_before + 1 and not ed.user_presets[-1].profile.lanes[0].has("id"),
		"saving a profile stores it without lane IDs")
	ed.set_template({"profile": ed.user_presets[-1].profile}, "smoke test profile")
	var added: PackedInt64Array = road.add_road([{"pos": Vector2(5000, 5000)}, {"pos": Vector2(5100, 5000)}], ed.road_template, 0, 50.0)
	_check(added.size() == 1, "roads can be drawn with a saved profile")
	ed.user_presets.pop_back()
	DirAccess.remove_absolute(MapEditor.PRESETS_PATH)

	# Every tool activates and deactivates cleanly.
	for t in ["road", "curve", "lane", "select"]:
		ed.set_tool(t)
	_check(ed.tool_name() == "select", "tools switch")

	print("%d check(s) failed" % _failures)
	quit(_failures)
