class_name MapView
extends Node2D
## Road surfaces and paint, one MeshInstance2D per (level, layer) batch from the
## C++ geometry. Few draw calls whatever the map size.

const LAYER_MARKINGS := 3

var _instances := {} # "level:layer" -> MeshInstance2D
var _paint_material: ShaderMaterial


func _ready() -> void:
	_paint_material = ShaderMaterial.new()
	_paint_material.shader = load("res://editor/paint_lines.gdshader")


func rebuild(meshes: Array, current_level: int) -> void:
	var used := {}
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
	for key in _instances.keys():
		if not used.has(key):
			_instances[key].queue_free()
			_instances.erase(key)
	apply_level_style(current_level)


## The level being edited is drawn normally; levels above it are see-through
## and levels below it are dimmed.
func apply_level_style(current_level: int) -> void:
	for key in _instances:
		var mi: MeshInstance2D = _instances[key]
		var lvl: int = mi.get_meta("level", 0)
		if lvl == current_level:
			mi.modulate = Color.WHITE
		elif lvl > current_level:
			mi.modulate = Color(1, 1, 1, 0.45)
		else:
			mi.modulate = Color(0.55, 0.55, 0.55, 1)
