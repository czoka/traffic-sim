// GDExtension bridge for the M1 road editor.
//
// Wraps tsim::Document (editable map + undo) and tsim::RoadGeometry (derived
// shapes). GDScript tools call the edit methods; geometry and problems are
// rebuilt lazily when the map's revision changes.
#pragma once

#include "tsim/document.h"
#include "tsim/network.h"
#include "tsim/road_geometry.h"
#include "tsim/traffic_run.h"
#include "tsim/validation.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
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
	// "town", "grid", "t_junction", "lane_drop", "one_way_pair". False if unknown.
	bool load_example(const String &name);
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
	// control: "right_hand" | "priority_road" | "all_way_stop"; priority: main-road segment IDs
	void set_junction_control(int64_t node, const String &control, const PackedInt64Array &priority);
	// spawner: {enabled, rate (veh/h), sink, od: [{to, weight}]}
	void set_spawner(int64_t node, const Dictionary &spawner);

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
	Array get_spawners(); // [{id, pos, level, rate, sink, active}]

	// --- Simulation (M2) -----------------------------------------------------------
	// Compiles the network if the map changed (only what changed) and removes
	// every car, then restarts the clock with `seed`.
	void sim_reset(int64_t seed);
	// Adds real_delta * speed of sim time and runs the ticks that are due, at
	// most budget_ms of work (the rest is dropped). Recompiles changed parts of
	// the network first. Returns the number of ticks run.
	int64_t sim_advance(double real_delta, double speed, double budget_ms);
	int64_t sim_step(int64_t ticks);
	void sim_set_demand(double multiplier);
	void sim_set_max_vehicles(int64_t cap);
	// 12 floats per car on `level` (Transform2D + Color), interpolated.
	PackedFloat32Array sim_car_buffer(int level, double car_scale);
	int64_t sim_car_count(int level) const;
	int64_t sim_pick_car(Vector2 pos, double radius, int level);
	Dictionary sim_car_info(int64_t id);
	Dictionary sim_stats();
	String sim_state_hash() const;
	Dictionary sim_golden_check() const;

protected:
	static void _bind_methods();

private:
	void ensure_geometry();
	void ensure_network(); // compiled for the problems panel (no cars)
	void sync_sim();
	tsim::Profile road_profile(const Dictionary &road);

	tsim::Document doc_;
	tsim::RoadGeometry geom_;
	std::vector<tsim::Problem> problems_;
	uint64_t built_revision_ = ~0ull;
	double build_ms_ = 0.0;
	double validate_ms_ = 0.0;

	tsim::NetworkCompiler check_compiler_;
	tsim::Network check_net_;
	uint64_t checked_revision_ = ~0ull;
	std::vector<tsim::NetProblem> net_problems_;

	tsim::TrafficRun sim_;
	double accumulator_ = 0.0; // sim seconds owed
	double tick_us_ema_ = 0.0;
	double frame_sim_ms_ema_ = 0.0;
	bool behind_ = false;
	double window_real_ = 0.0;
	int64_t window_ticks_ = 0;
	double effective_speed_ = 0.0;
	int last_recompiled_junctions_ = -1;
	double last_recompile_ms_ = 0.0;
	PackedFloat32Array car_buffer_;
};

} // namespace godot
