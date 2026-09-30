// GDExtension bridge: exposes the C++ simulation core to GDScript.
//
// The core (tsim::) knows nothing about Godot. This class owns a map and a
// simulation, runs fixed ticks from real frame time (with a time budget so
// high speeds are sliced across frames), and packs interpolated car transforms
// into a MultiMesh buffer once per frame.
#pragma once

#include "tsim/map.h"
#include "tsim/sim.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/rect2.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <memory>

namespace godot {

class TrafficSim : public RefCounted {
	GDCLASS(TrafficSim, RefCounted)

public:
	// Floats per MultiMesh instance: Transform2D (8) + Color (4).
	static constexpr int kFloatsPerCar = 12;

	TrafficSim();
	~TrafficSim() override;

	// --- Map ---------------------------------------------------------------
	void new_ring(double radius, int lanes, double lane_width, double speed_kmh);
	// Throwaway POC tool: a one-way straight road from a to b. Returns its ID.
	int64_t add_straight_road(Vector2 a, Vector2 b, int lanes, double speed_kmh);
	String save_json() const;
	// Returns an empty string on success, otherwise the error message.
	String load_json(const String &text);
	// Road drawing data: Array of {points: PackedVector2Array, width: float, kind: int}
	// kind 0 = asphalt, 1 = road edge, 2 = lane divider.
	Array get_road_lines(double max_step) const;
	Rect2 get_map_bounds() const;

	// --- Vehicles ----------------------------------------------------------
	int64_t spawn_cars(int64_t count, int64_t seed);
	void clear_cars();
	int64_t get_vehicle_count() const;

	// --- Time --------------------------------------------------------------
	// Adds real_delta * speed of sim time and runs the ticks that are due, but
	// stops after budget_ms of work. Backlog beyond the budget is dropped, so
	// the sim runs slower than requested instead of freezing the frame.
	// Returns the number of ticks run.
	int64_t advance(double real_delta, double speed, double budget_ms);
	int64_t step(int64_t ticks);
	int64_t get_tick() const;
	double get_sim_time() const;
	double get_interpolation_alpha() const;

	// --- Rendering ---------------------------------------------------------
	// Interpolated transforms + speed colours for every car, 12 floats each.
	// car_scale enlarges the car shapes (not their positions) so cars stay
	// visible when zoomed far out.
	PackedFloat32Array get_render_buffer(double car_scale);

	// --- Measurement and determinism ----------------------------------------
	String get_state_hash() const;
	Dictionary get_stats() const;
	// Runs the golden scenario on a private map and compares the hash with the
	// value every platform must produce.
	Dictionary run_golden_check() const;

protected:
	static void _bind_methods();

private:
	void reset_clock();

	tsim::Map map_;
	std::unique_ptr<tsim::Simulation> sim_;
	double accumulator_ = 0.0; // sim seconds owed

	// Measurements
	int64_t last_frame_ticks_ = 0;
	double last_frame_sim_ms_ = 0.0;
	double tick_us_ema_ = 0.0;
	double frame_sim_ms_ema_ = 0.0;
	double frame_ticks_ema_ = 0.0;
	double render_prep_us_ema_ = 0.0;
	bool behind_ = false;
	double window_real_ = 0.0;
	int64_t window_ticks_ = 0;
	double effective_speed_ = 0.0;

	PackedFloat32Array buffer_;
};

} // namespace godot
