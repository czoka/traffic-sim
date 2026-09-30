// GDExtension bridge for the M1 road editor.
//
// Wraps tsim::Document (editable map + undo) and tsim::RoadGeometry (derived
// shapes). GDScript tools call the edit methods; geometry and problems are
// rebuilt lazily when the map's revision changes.
#pragma once

#include "tsim/document.h"
#include "tsim/road_geometry.h"
#include "tsim/validation.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_int64_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector2.hpp>

namespace godot {

class RoadEditor : public RefCounted {
	GDCLASS(RoadEditor, RefCounted)

public:
	// --- Map and files -------------------------------------------------------
	void new_map();
	void load_demo_town();
	void load_test_grid(int cols, int rows, double spacing);
	String save_json() const;
	// {ok: bool, error: String, migrated_from: int}
	Dictionary load_json(const String &text);
	int64_t revision() const;

	// --- History -------------------------------------------------------------
	void begin(const String &label);
	bool commit();
	void cancel();
	bool undo();
	bool redo();
	bool can_undo() const;
	bool can_redo() const;
	String undo_label() const;
	String redo_label() const;
	int64_t history_size() const;
	int64_t redo_size() const;

	// --- Edits ---------------------------------------------------------------
	// points: Array of {pos: Vector2, node?: int, segment?: int}
	// road: {preset: String} | {params: Dictionary} | {profile: Dictionary}
	PackedInt64Array add_road(const Array &points, const Dictionary &road, int level, double speed_kmh);
	int64_t add_curve(const Dictionary &a, Vector2 control, const Dictionary &b, const Dictionary &road, int level,
			double speed_kmh);
	void move_node(int64_t id, Vector2 pos);
	bool merge_nodes(int64_t from, int64_t into);
	void set_control_point(int64_t seg, int which, Vector2 pos);
	int64_t split_segment(int64_t seg, double u);
	void delete_segment(int64_t id);
	void delete_node(int64_t id);
	String set_profile_params(int64_t seg, const Dictionary &params);
	String set_profile(int64_t seg, const Dictionary &profile);
	String set_lane_type(int64_t seg, int64_t lane, const String &type);
	bool flip(int64_t seg);
	void set_end_rules(int64_t seg, int end, const Dictionary &rules);
	void set_speed_kmh(int64_t seg, double kmh);
	void set_segment_name(int64_t seg, const String &name);
	void set_level(int64_t seg, int level);
	void set_no_change(int64_t seg, int edge, double u0, double u1, bool block_l2r, bool block_r2l);

	// --- Profiles --------------------------------------------------------------
	Array presets() const; // [{name, params}]
	Dictionary params_of_profile(const Dictionary &profile) const;
	Dictionary profile_from_params(const Dictionary &params) const;
	String validate_profile(const Dictionary &profile) const;

	// --- Queries ----------------------------------------------------------------
	Dictionary get_node(int64_t id);
	Dictionary get_segment(int64_t id);
	// {type: "node"|"segment"|"none", id, u, lane, lane_index, offset, edge, edge_distance}
	Dictionary pick(Vector2 pos, double radius, int level);
	int64_t nearest_node(Vector2 pos, double radius, int level, int64_t exclude);
	PackedVector2Array segment_centerline(int64_t seg, double step);
	PackedVector2Array segment_outline(int64_t seg);
	PackedVector2Array node_outline(int64_t node);
	PackedVector2Array lane_outline(int64_t seg, int64_t lane);
	PackedVector2Array edge_polyline(int64_t seg, int edge, double u0, double u1);
	double segment_u_at(int64_t seg, Vector2 pos);
	Vector2 segment_point(int64_t seg, double u);
	Vector2 segment_tangent(int64_t seg, double u);
	PackedInt64Array node_ids() const;
	PackedInt64Array segment_ids() const;

	// --- Geometry output ---------------------------------------------------------
	Array get_meshes(); // [{level, layer, vertices, colors, indices}]
	Array get_connectors(); // [{path, turn, node}]
	Array get_problems(); // [{severity, code, message, pos, level, segments, nodes}]
	Dictionary get_stats();

protected:
	static void _bind_methods();

private:
	void ensure_geometry();
	tsim::Profile road_profile(const Dictionary &road);

	tsim::Document doc_;
	tsim::RoadGeometry geom_;
	std::vector<tsim::Problem> problems_;
	uint64_t built_revision_ = ~0ull;
	double build_ms_ = 0.0;
	double validate_ms_ = 0.0;
};

} // namespace godot
