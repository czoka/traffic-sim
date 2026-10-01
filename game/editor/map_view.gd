class_name MapView
extends Node2D
## Road surfaces and paint, one MeshInstance2D per (level, layer) batch from the
## C++ geometry. Few draw calls whatever the map size. Bridges and other upper
## levels cast a soft shadow on what is below them.

const LAYER_ASPHALT := 1
const LAYER_MARKINGS := 3
## Where the sun puts the shadow of a level, per level of height (m).
const SHADOW_OFFSET := Vector2(2.0, 3.0)
const SHADOW_ALPHA := 0.32

var _instances := {} # "level:layer" -> MeshInstance2D
var _shadows := {} # level -> CanvasGroup with the level's ground and asphalt in black
var _paint_material: ShaderMaterial


func _ready() -> void:
	_paint_material = ShaderMaterial.new()
	_paint_material.shader = load("res://editor/paint_lines.gdshader")


func rebuild(meshes: Array, current_level: int, filter := false) -> void:
	var used := {}
	var used_shadows := {}
	for m in meshes:
		var key := "%d:%d" % [m.level, m.layer]
		used[key] = true
		var mi: MeshInstance2D = _instances.get(key)
		if mi == null:
			mi = MeshInstance2D.new()
			mi.name = "L%d_%d" % [m.level + 1, m.layer]
			add_child(mi)
			_instances[key] = mi
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = m.vertices
		arrays[Mesh.ARRAY_COLOR] = m.colors
		arrays[Mesh.ARRAY_TEX_UV] = m.uvs
		arrays[Mesh.ARRAY_INDEX] = m.indices
		var mesh := ArrayMesh.new()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		mi.mesh = mesh
		mi.z_index = (int(m.level) + 2) * 10 + int(m.layer)
		mi.material = _paint_material if int(m.layer) == LAYER_MARKINGS else null
		mi.set_meta("level", int(m.level))
		if int(m.level) >= 1 and int(m.layer) <= LAYER_ASPHALT:
			var group: CanvasGroup = _shadows.get(int(m.level))
			if group == null:
				group = CanvasGroup.new()
				group.name = "Shadow%d" % int(m.level)
				group.z_index = (int(m.level) + 2) * 10 - 1 # just below the level, over the one under it
				group.position = SHADOW_OFFSET * int(m.level)
				group.self_modulate = Color(1, 1, 1, SHADOW_ALPHA)
				group.set_meta("level", int(m.level))
				add_child(group)
				_shadows[int(m.level)] = group
			used_shadows[int(m.level)] = true
			var sname := "S%d" % int(m.layer)
			var smi: MeshInstance2D = group.get_node_or_null(sname)
			if smi == null:
				smi = MeshInstance2D.new()
				smi.name = sname
				smi.modulate = Color.BLACK
				group.add_child(smi)
			smi.mesh = mesh
	for key in _instances.keys():
		if not used.has(key):
			_instances[key].queue_free()
			_instances.erase(key)
	for lvl in _shadows.keys():
		if not used_shadows.has(lvl):
			_shadows[lvl].queue_free()
			_shadows.erase(lvl)
	apply_level_style(current_level, filter)


## The level being edited is drawn normally; levels above it are see-through
## and levels below it are dimmed. With the filter on, only that level shows.
func apply_level_style(current_level: int, filter := false) -> void:
	for key in _instances:
		var mi: MeshInstance2D = _instances[key]
		var lvl: int = mi.get_meta("level", 0)
		mi.visible = not filter or lvl == current_level
		mi.modulate = level_tint(lvl, current_level)
	for lvl in _shadows:
		# A shadow falls on the level below; it only shows while that level is drawn too.
		_shadows[lvl].visible = not filter and lvl <= current_level + 1
		_shadows[lvl].modulate = Color(1, 1, 1, 1) if lvl > current_level else Color(1, 1, 1, 0.6)


static func level_tint(lvl: int, current_level: int) -> Color:
	if lvl == current_level:
		return Color.WHITE
	if lvl > current_level:
		return Color(1, 1, 1, 0.45)
	return Color(0.55, 0.55, 0.55, 1)
