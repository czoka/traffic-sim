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
	// "town", "grid", "t_junction", "lane_drop", "one_way_pair", "showcase", "people", "people_city",
	// "new_city", "city_town", "city_week", "city_market". False if unknown.
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
	// control: "right_hand" | "priority_road" | "all_way_stop" | "signal"; priority: main-road segment IDs
	void set_junction_control(int64_t node, const String &control, const PackedInt64Array &priority);
	// spawner: {enabled, rate (veh/h), sink, od: [{to, weight}], bikes (per h), coaches: [{id, exit, per_hour, dwell}]}
	void set_spawner(int64_t node, const Dictionary &spawner);

	// --- M3: roundabouts, signals, stops, depots --------------------------------
	// {enabled, radius, lanes, turbo, slip: [segment ids]}
	void set_roundabout(int64_t node, const Dictionary &r);
	// {phases: [{green, moves: [{from, to, permissive}]}], amber, all_red, offset, right_on_red: [segment ids]}
	Dictionary default_signal_plan(int64_t node) const;
	void set_signal_plan(int64_t node, const Dictionary &plan);
	// side: "forward" | "backward"; kind: "kerbside" | "bay" | "main_station". Returns the stop id.
	int64_t add_stop(int64_t seg, double u, const String &side, const String &kind, const String &name);
	void set_stop(int64_t seg, const Dictionary &stop); // {id, u, side, kind, name, bays}
	void remove_stop(int64_t seg, int64_t stop);
	// {enabled, name, capacity, routes: [{id, name, color, stops: [ids], headway, loop}]}
	void set_depot(int64_t node, const Dictionary &depot);
	Array get_stops(); // [{id, segment, u, side, kind, name, bays, pos, dir, level}]
	Array get_depots(); // [{node, pos, level, name, capacity, routes: [{id, name, color, headway, loop, stops, path}]}]
	Array get_bays(int level); // [{pos, dir, style}] parking bays, for drawing
	// Signal heads at each signalized approach lane: [{node, pos, dir, light}] with light
	// "red" | "amber" | "green" | "yield"; uses the running sim's clock.
	Array sim_signal_heads(int level);
	// {phase, into, cycle, phases: [{green, amber, all_red}]} for the signal timeline.
	Dictionary sim_signal_state(int64_t node);
	Array sim_route_stats(); // [{id, active, runs, round_trip, fleet}]

	// --- M4: paths, crossings, fences, ramps, bridges ------------------------------
	// Paths: add_road with road = {preset|params|profile, kind: "footpath" | "bike_path" | "shared_path"}.
	// kind: "zebra" | "signal" | "uncontrolled". Returns the crossing id (0 if the road doesn't exist).
	int64_t add_crossing(int64_t seg, double u, const String &kind, bool bike, bool refuge);
	void set_crossing(int64_t seg, const Dictionary &crossing); // {id, u, kind, bike, refuge}
	void remove_crossing(int64_t seg, int64_t id);
	void set_fence(int64_t seg, int side, double u0, double u1, bool on); // side 0 left, 1 right
	void set_ramp(int64_t seg, int rise, bool stairs);
	// What the bridge (delta 1) / tunnel (-1) tool would build: {ok, error, length, stations, points}.
	Dictionary plan_lift(int64_t seg, Vector2 at, int delta);
	String lift(int64_t seg, Vector2 at, int delta); // "" on success
	Array get_crossings(int level); // painted crossings: [{segment, end, id, node, kind, bike, a, b, mid, refuge}]
	// People in the sim: {max_pedestrians, car_owners, bike_owners, bus_capacity}
	void sim_set_people(const Dictionary &config);
	int64_t sim_ped_count(int level) const;
	PackedFloat32Array sim_ped_buffer(int level, double scale); // 12 floats per person (Transform2D + Color)
	int64_t sim_pick_ped(Vector2 pos, double radius, int level);
	Dictionary sim_ped_info(int64_t id);
	Array sim_walk_lights(int level); // [{a, b, walk: "walk"|"flashing"|"dont_walk", push_button, car}]
	Array sim_stop_stats(); // [{id, name, pos, waiting, boarded, alighted, left_behind, mean_wait}]
	Array sim_route_loads(); // [{route, load: PackedFloat32Array per stop}]

	// --- M5: buildings and city life ------------------------------------------------
	void new_city(); // a new map: High Street, the main station with coaches, the city offices
	Array building_types() const; // [{id, label, kind, width, depth, color, households, slots, desks, ...}]
	// Where a lot of `type` would go near a point: {ok, pos, dir, corners, blocked}.
	Dictionary snap_building(const String &type, Vector2 near, int level);
	int64_t add_building(const String &type, Vector2 pos, Vector2 dir, int level);
	void set_building_name(int64_t id, const String &name);
	void remove_building(int64_t id);
	Array get_buildings(int level); // [{id, type, label, kind, name, corners, centre, door}]
	int64_t pick_building(Vector2 pos, int level); // 0: none
	Dictionary get_building(int64_t id);
	Dictionary sim_city_stats();
	Dictionary sim_building_info(int64_t id);
	Array sim_building_states(int level); // [{id, centre, kind, open, in_hours, closed_unexpectedly, inside, ...}]
	Dictionary sim_resident_info(int64_t id);
	void sim_set_city(const Dictionary &config); // {prefill, employment_share}

	// --- M6: the economy ----------------------------------------------------------
	// The player's numbers for a city-owned building: {rent, price_factor, wage,
	// for_sale, asking} (0 = the default). Undoable.
	void set_building_economy(int64_t id, const Dictionary &settings);
	void set_city_centre(Vector2 pos); // undoable
	void clear_city_centre();
	Dictionary get_city_centre(); // {placed, pos} (pos: the marker, or the centroid of shops and offices)
	Array sim_market(); // [{id, name, label, by_city, owner, asking, value, days}]
	bool sim_buy_building(int64_t id); // the player buys an NPC owner's listing

	// --- M7: insight -------------------------------------------------------------
	Dictionary sim_lane_lines(int level); // {lines: [PackedVector2Array], lanes: PackedInt32Array, lane_count}
	// Per sim lane: speed (of the limit, 0-1), wait (s per vehicle) or flow
	// (vehicles an hour); -1 where there was no traffic lately.
	PackedFloat32Array sim_lane_heat(const String &mode);
	Dictionary sim_junction_stats(int64_t node);

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
	static void bind_m4_methods();
	static void bind_m5_methods();
	static void bind_m6_methods();
	static void bind_m7_methods();

private:
	void ensure_geometry();
	void ensure_network(); // compiled for the problems panel (no cars)
	void sync_sim();
	tsim::Profile road_profile(const Dictionary &road);
	PackedVector2Array route_path(const tsim::NetDepot &d, const tsim::BusRoute &r);

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
	PackedFloat32Array ped_buffer_;
};

} // namespace godot
