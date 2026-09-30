// Traffic simulation on a compiled Network (M2).
//
// Cars enter at spawn points, follow an A* route on the lane graph to a sink,
// and drive with the Intelligent Driver Model (car following) and MOBIL (lane
// changes). Junctions hand out grants: a car may enter the box only when no
// car on a conflicting connector is still in the way, the rules (right-hand
// priority, priority road, all-way stop, with per-driver gap acceptance) let
// it go, and its exit has room (don't block the box).
//
// Fixed 10 Hz tick, seeded RNG, and only + - * / and sqrt on doubles in the
// tick, so the same network and seed give the same state hash everywhere.
#pragma once

#include "tsim/network.h"
#include "tsim/rng.h"

#include <cstdint>
#include <string>
#include <vector>

namespace tsim {

struct TrafficConfig {
	double dt = 0.1; // seconds per tick
	double demand = 1.0; // multiplies every spawn rate
	double lookahead = 200.0; // metres searched ahead for a leader or a stop
	double max_decel = 9.0; // m/s^2, physical braking limit
	double accel_noise = 0.1; // m/s^2, uniform +/- per tick
	double reroute_interval = 300.0; // s between route updates per car
	double lane_change_cost = 3.0; // s, routing penalty per lane change
	double bus_lane_factor = 3.0; // cars treat bus lanes as this much slower
	double stuck_timeout = 900.0; // s stopped before a car is taken off the map
	double box_margin = 2.0; // m past a conflict point before it counts as clear
	double impatience = 30.0; // s waiting at a line before a driver takes the next safe chance
	int spawn_queue = 20; // cars waiting to enter at one spawn point
	uint32_t max_vehicles = 0; // spawning pauses while this many cars are on the map (0 = no cap)
};

struct DriverParams2 {
	double speed_factor = 1.0; // desired speed / speed limit
	double T = 1.3; // time headway (s)
	double a = 1.4; // max acceleration (m/s^2)
	double b = 2.0; // comfortable deceleration (m/s^2)
	double s0 = 2.0; // jam distance (m)
	double length = 4.5; // m
	double two_sqrt_ab = 0.0;
	double critical_gap = 5.0; // s, gap accepted at a yield
	double politeness = 0.3; // MOBIL
};

enum class VehicleState : uint8_t {
	Driving = 0,
	Queued = 1, // behind another car
	Approaching = 2, // near a junction without a grant yet
	Yielding = 3, // giving way to a car with priority
	BoxBlocked = 4, // a conflicting car is still in the junction
	ExitBlocked = 5, // don't block the box: no room after the junction
	StopSign = 6, // all-way stop: stopping or waiting for its turn
	InJunction = 7,
	ChangingLane = 8, // needs a lane change and is waiting for a gap
};
const char *vehicle_state_name(VehicleState s);

struct Vehicle {
	VehicleId id = kNoId;
	int32_t lane = -1;
	double s = 0.0; // front bumper along the lane
	double v = 0.0;
	double acc = 0.0;
	int32_t prev_lane = -1;
	double prev_s = 0.0;
	double lat = 0.0; // sideways offset while a lane change is drawn (m, + = right)
	double prev_lat = 0.0;
	DriverParams2 drv;
	NodeId origin = kNoId;
	NodeId dest = kNoId;
	std::vector<int32_t> route; // connectors still to take
	size_t ri = 0; // next connector in route
	int32_t grant = -1; // connector this car may enter
	int32_t held = -1; // connector whose junction box it is still in
	int32_t list_pos = 0; // index in its lane's car list
	uint64_t wait_since = 0; // tick it started waiting at a stop line (0 = not waiting)
	uint64_t stopped_tick = 0; // all-way stop: tick it came to a stop at the line
	uint64_t spawn_tick = 0;
	uint64_t lane_tick = 0; // tick it entered the current road (for travel times)
	uint64_t last_change = 0; // tick of the last lane change
	double stopped_for = 0.0; // s continuously below 0.1 m/s
	double distance = 0.0; // m driven
	int32_t courtesy = -1; // vehicle index this car leaves a gap for
	int32_t merge_lane = -1; // lane it is trying to move into (mandatory)
	double merge_s = 0.0;
	VehicleState state = VehicleState::Driving;
	VehicleId blocker = kNoId; // who it waits for, if known
	bool done = false; // left the map this tick
};

struct TrafficStats {
	uint32_t vehicles = 0;
	uint64_t spawned = 0;
	uint64_t arrived = 0;
	uint64_t removed_stuck = 0;
	uint64_t unroutable = 0; // trips dropped because no route existed
	uint32_t waiting_to_enter = 0; // queued at spawn points
	double mean_speed = 0.0; // m/s
	uint32_t stopped = 0; // below 1 m/s
	double max_stopped = 0.0; // s, longest current stop
	double mean_trip_time = 0.0; // s, of arrived cars
	// s since a junction last let a car in while cars wait there whose exit has
	// room (a junction that is stuck by itself, not by traffic further on)
	double max_junction_wait = 0.0;
	uint64_t lane_changes = 0;
	uint64_t reroutes = 0;
	uint64_t forced_grants = 0; // deadlock breaker uses
};

struct VehicleInfo {
	VehicleId id = kNoId;
	bool found = false;
	double speed = 0.0;
	double desired_speed = 0.0;
	double accel = 0.0;
	VehicleState state = VehicleState::Driving;
	VehicleId blocker = kNoId;
	NodeId origin = kNoId;
	NodeId dest = kNoId;
	double trip_time = 0.0;
	double distance = 0.0;
	double stopped_for = 0.0;
	double critical_gap = 0.0;
	int level = 0;
	LaneKey lane;
	SegmentId segment = kNoId;
	NodeId junction = kNoId; // next junction on the route
	size_t connectors_left = 0;
	std::vector<Vec2> route; // polyline from the car to its destination
};

class Traffic {
public:
	explicit Traffic(TrafficConfig config = {});

	// Switches to a (re)compiled network. Vehicles, routes and grants are
	// carried over by stable lane keys; cars on lanes that no longer exist are
	// removed and cars whose route broke are re-routed.
	void set_network(const Network *net);
	const Network *network() const { return net_; }

	void reset(uint64_t seed); // removes all cars, resets the clock and RNG
	void tick();

	TrafficConfig &config() { return config_; }
	const TrafficConfig &config() const { return config_; }
	uint64_t tick_count() const { return tick_; }
	double sim_time() const { return static_cast<double>(tick_) * config_.dt; }
	uint64_t seed() const { return seed_; }

	const std::vector<Vehicle> &vehicles() const { return veh_; }
	int32_t find_vehicle(VehicleId id) const;
	Pose pose(size_t i, double alpha) const; // interpolated between ticks, lane-change offset included
	int level_of(size_t i) const;
	VehicleInfo info(VehicleId id) const;
	TrafficStats stats() const;
	uint64_t state_hash() const;

	// For tests and tools: add a car directly (returns its id, 0 on failure).
	// Driver parameters default to an average driver.
	VehicleId add_vehicle(int32_t lane, double s, double v, NodeId dest, const DriverParams2 *driver = nullptr);
	// A* on the lane graph from a road lane to a sink node. Fills `route` with
	// the connectors to take and returns false when there is no route.
	bool find_route(int32_t start, NodeId dest, std::vector<int32_t> &route, bool allow_change_first = true) const;

private:
	struct Candidate {
		int32_t veh = -1;
		int32_t conn = -1;
		double dist = 0.0; // to the stop line
	};

	double desired_speed(const Vehicle &v, int32_t lane) const;
	double idm(const Vehicle &v, double v0, bool has, double gap, double lead_v) const;
	// Connector a car takes from road lane `lane` when its next route step is ri.
	int32_t next_connector(const Vehicle &v, int32_t lane, size_t ri) const;
	// Lanes of the same road from which the route's next step can be reached.
	int good_direction(const Vehicle &v, int32_t lane, int &steps) const;
	bool lane_is_good(const Vehicle &v, int32_t lane, size_t ri) const;
	void leader(const Vehicle &v, int32_t lane, double s, size_t ri, bool &has, double &gap, double &lead_v,
			double &v0_cap, VehicleId &who) const;
	double accel_at(size_t i, int32_t lane, double s) const;
	bool box_clear(const NetJunction &j, int32_t conn, const std::vector<int32_t> &granted, VehicleId &blocker) const;
	bool exit_clear(int32_t conn, int32_t veh, const std::vector<int32_t> &granted) const;
	double pos_on(const Vehicle &v, int32_t conn) const;
	double time_to(const Vehicle &v, double dist) const;

	void arbitrate();
	void change_lanes();
	void move();
	void spawn();
	void rebuild_lists();
	void compact();
	bool reroute(Vehicle &v, bool allow_change_first);
	void rebuild_reachability();
	double lane_cost(int32_t lane) const;

	TrafficConfig config_;
	const Network *net_ = nullptr;
	Rng rng_;
	uint64_t seed_ = 1;
	uint64_t tick_ = 0;
	VehicleId next_id_ = 1;
	std::vector<Vehicle> veh_; // ascending id
	std::vector<std::vector<int32_t>> cars_; // per lane, front (largest s) first
	std::vector<double> lane_time_; // observed travel time per road lane (s)
	std::vector<uint32_t> pending_; // per spawner, cars waiting to enter
	std::vector<std::vector<std::pair<size_t, double>>> reach_; // per spawner: (spawner, weight)
	std::vector<uint64_t> junction_last_grant_;
	std::vector<char> junction_waiting_;
	TrafficStats stats_;
	double trip_time_sum_ = 0.0;
};

} // namespace tsim
