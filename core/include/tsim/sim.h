// Fixed-step traffic simulation (10 Hz) with IDM car-following.
//
// The simulation reads the Map but never changes it. After editing the map,
// call on_map_changed() before the next tick.
#pragma once

#include "tsim/map.h"
#include "tsim/rng.h"

#include <cstdint>
#include <vector>

namespace tsim {

struct SimConfig {
	double dt = 0.1; // seconds per tick (10 Hz)
	double accel_noise = 0.3; // m/s^2, uniform +/- per tick; seeds stop-and-go waves
	double lookahead = 300.0; // metres searched ahead for a leader across lanes
	double max_decel = 9.0; // m/s^2, physical braking limit
};

// Intelligent Driver Model parameters, varied per driver at spawn.
struct DriverParams {
	double v0 = 13.9; // desired speed (m/s)
	double T = 1.5; // time headway (s)
	double a = 1.0; // max acceleration (m/s^2)
	double b = 1.5; // comfortable deceleration (m/s^2)
	double s0 = 2.0; // jam distance (m)
	double length = 4.5; // vehicle length (m)
	double two_sqrt_ab = 0.0; // 2*sqrt(a*b), precomputed
};

struct SimStats {
	uint32_t vehicles = 0;
	double mean_speed = 0.0; // m/s
	double speed_stddev = 0.0; // m/s
	uint32_t stopped = 0; // vehicles below 1 m/s
	double mean_desired_speed = 0.0; // m/s
};

class Simulation {
public:
	explicit Simulation(const Map &map, SimConfig config = {});

	// Rebuilds the lane index after the map changed. Vehicles keep their lane
	// IDs; vehicles on lanes that no longer exist are removed.
	void on_map_changed();

	void clear_vehicles();

	// Resets the RNG with `seed`, removes all vehicles and spreads `count`
	// vehicles evenly over every lane that has a successor (so loops such as the
	// ring get cars and dead-end roads do not). Returns how many were placed;
	// fewer than requested when the lanes are too short to hold them.
	uint32_t spawn_even(uint32_t count, uint64_t seed);

	// Advances the world by one fixed step.
	void tick();

	uint64_t tick_count() const { return tick_; }
	double sim_time() const { return static_cast<double>(tick_) * config_.dt; }
	const SimConfig &config() const { return config_; }

	// Hash of the full simulation state (tick, RNG, every vehicle).
	uint64_t state_hash() const;

	size_t vehicle_count() const { return id_.size(); }
	VehicleId vehicle_id(size_t i) const { return id_[i]; }
	LaneId vehicle_lane(size_t i) const { return lanes_[lane_[i]].id; }
	double vehicle_s(size_t i) const { return s_[i]; }
	double vehicle_speed(size_t i) const { return v_[i]; }
	const DriverParams &vehicle_params(size_t i) const { return params_[i]; }

	// Poses at the current tick and at the tick before, for interpolation.
	Pose vehicle_pose(size_t i) const;
	Pose vehicle_prev_pose(size_t i) const;

	SimStats stats() const;

private:
	struct LaneRt {
		LaneId id = kNoId;
		const Lane *lane = nullptr;
		double length = 0.0;
		int32_t next = -1; // dense index of next[0], or -1 for a dead end
		std::vector<uint32_t> cars; // vehicle indices, front (largest s) first
	};

	int32_t dense_index(LaneId id) const;
	void sort_lane(std::vector<uint32_t> &cars) const;
	double idm_accel(size_t i, bool has_leader, double gap, double leader_speed) const;
	void find_leader_ahead(uint32_t lane, double s, bool &has, double &gap, double &speed) const;

	const Map &map_;
	SimConfig config_;
	Rng rng_;
	uint64_t tick_ = 0;
	VehicleId next_vehicle_id_ = 1;

	std::vector<LaneRt> lanes_;

	// Vehicles, struct-of-arrays, in spawn order.
	std::vector<VehicleId> id_;
	std::vector<uint32_t> lane_;
	std::vector<double> s_;
	std::vector<double> v_;
	std::vector<double> acc_;
	std::vector<DriverParams> params_;
	std::vector<uint32_t> prev_lane_;
	std::vector<double> prev_s_;
};

// The reference scenario used by determinism checks on every platform.
struct GoldenScenario {
	static constexpr double kRadius = 1500.0;
	static constexpr int kLanes = 4;
	static constexpr double kLaneWidth = 3.5;
	static constexpr double kSpeedLimit = 60.0 / 3.6; // 60 km/h
	static constexpr uint32_t kCars = 2000;
	static constexpr uint64_t kSeed = 42;
	static constexpr uint64_t kTicks = 6000; // 10 sim minutes, well into the stop-and-go regime
	// Expected state hash after kTicks. Update only on an intentional change to
	// sim behaviour, and note why in the commit message.
	static constexpr const char *kExpectedHash = "d3b987055dad39fb";
};

// Builds the golden ring map.
void build_golden_map(Map &map);
// Spawns and runs a scenario on `map`, returning the final state hash.
uint64_t run_scenario(const Map &map, uint32_t cars, uint64_t seed, uint64_t ticks);

} // namespace tsim
