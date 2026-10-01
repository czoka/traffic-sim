// Traffic simulation on a compiled Network (M2, M3).
//
// Vehicles enter at spawn points (cars, taxis, bikes), depots (city buses) and
// coach lines, follow A* routes on the lane graph and drive with the
// Intelligent Driver Model (car following) and MOBIL (lane changes). Junctions
// hand out grants: a vehicle may enter the box only when no vehicle on a
// conflicting connector is still in the way, the rules (right-hand priority,
// priority road, all-way stop, signals, roundabout entries, with per-driver
// gap acceptance) let it go, and its exit has room (don't block the box).
//
// M3 adds vehicle classes with their own lanes (bus lanes, bike lanes),
// waypoints (bus stops, the main station, parking bays) and fixed-time
// signals.
//
// Fixed 10 Hz tick, seeded RNG, and only + - * / and sqrt on doubles in the
// tick, so the same network and seed give the same state hash everywhere.
#pragma once

#include "tsim/city_data.h"
#include "tsim/network.h"
#include "tsim/rng.h"

#include <cstdint>
#include <map>
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
	// M3
	double taxi_share = 0.05; // of car trips
	double park_share = 0.25; // of car trips that park on the street on the way (if bays exist)
	double park_min = 300.0, park_max = 1800.0; // s parked
	double bus_dwell = 20.0; // s at each bus stop (fixed until passengers arrive in M4)
	double bus_lane_zone = 60.0; // m before the stop line where cars may enter a bus lane to turn
	// M4: people
	uint32_t max_pedestrians = 0; // trips on foot pause at this many people on the map (0 = no cap)
	double bike_owners = 0.3; // share of people who could ride a bike
	double car_owners = 0.5; // could drive
	double coach_share = 0.05; // trips that leave the map by coach (when coaches run)
	double transfer_penalty = 300.0; // s per bus transfer in mode choice
	double cost_variance = 0.2; // each option's cost is scaled by 1 +/- this, per person
	int bus_capacity = 80, coach_capacity = 50;
	int bus_doors = 2, coach_doors = 1;
	double board_time = 2.0; // s per passenger per door
	double door_time = 3.0; // s to open and close
	// M5: city life
	double city_prefill = 0.0; // share of home units filled with households at reset (tests, the gate)
	double employment_share = 0.8; // residents who look for a job
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
	double max_speed = 50.0; // m/s, the vehicle's own limit (buses, bikes)
	double width = 1.8; // m, for drawing
};

enum class VehicleKind : uint8_t {
	Car = 0,
	Taxi = 1, // a car that may use bus lanes
	Bus = 2, // city bus on a route from a depot
	Coach = 3, // intercity coach to the main station
	Bike = 4,
};
const char *vehicle_kind_name(VehicleKind k);

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
	RedLight = 9,
	AtStop = 10, // bus at a stop, coach at the main station
	Parking = 11, // manoeuvring into or out of a bay
	Parked = 12,
	GivingWay = 13, // people on a crossing ahead (M4)
};
const char *vehicle_state_name(VehicleState s);

// Something a vehicle does on the way: stop, pull into a bay, park.
enum class WaypointAction : uint8_t {
	KerbStop = 0, // stop in the lane
	BayStop = 1, // pull into a lay-by (bus bay, main station)
	Park = 2, // park in a bay for a while
};

struct Waypoint {
	WaypointAction action = WaypointAction::KerbStop;
	int32_t lane = -1;
	double s = 0.0;
	double dwell = 20.0; // s
	int32_t stop = -1; // Network::stops index
	int32_t bay = -1; // Network::bays index
};

struct Vehicle {
	VehicleId id = kNoId;
	std::vector<uint32_t> riders; // pedestrian ids on board (buses, coaches; M4)
	VehicleKind kind = VehicleKind::Car;
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
	std::vector<Waypoint> waypoints;
	size_t wi = 0; // next waypoint
	uint32_t bus_route = 0; // BusRoute id (buses)
	uint32_t coach_line = 0; // CoachLine id (coaches)
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
	// Waypoint actions: 0 none, 1 dwelling in the lane, 2 manoeuvring in,
	// 3 off the lane (in a bay or parked), 4 manoeuvring out.
	uint8_t phase = 0;
	uint64_t phase_start = 0; // tick the current phase began
	uint64_t phase_until = 0; // tick the current phase ends
	Vec2 bay_pos, bay_dir; // where it is drawn while off the lane
	bool off_lane = false;
	bool ped_yield = false; // stopping for people on a crossing (counts yields)
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
	// M3
	uint32_t by_kind[5] = {};
	uint32_t parked = 0;
	uint64_t parkings = 0; // cars that parked
	uint64_t parking_failed = 0; // wanted a bay but none was free
	uint64_t bus_runs = 0; // buses that finished a run back at the depot
	uint64_t bus_stops_served = 0;
	uint64_t coach_calls = 0; // coaches that served the main station
	uint64_t bikes_arrived = 0;
	double bus_lane_misuse = 0.0; // car-seconds in bus lanes outside the turning zone
	uint64_t red_light_waits = 0; // grants refused by a red light
	uint64_t right_on_red = 0; // right turns on red with the flashing arrow
	// M4: people
	uint32_t pedestrians = 0; // on foot or waiting (not riding)
	uint32_t riding = 0;
	uint64_t trips = 0; // people trips started
	uint64_t trips_walk = 0, trips_bus = 0, trips_bike = 0, trips_car = 0, trips_coach = 0;
	uint64_t people_arrived = 0;
	uint64_t boarded = 0, alighted = 0, left_behind = 0;
	double mean_wait = 0.0; // s at stops, of people who boarded
	uint64_t coach_passengers = 0; // arrived by coach
	uint64_t crossings = 0; // crossings started on foot
	double mean_crossing_wait = 0.0; // s at the kerb
	uint64_t cars_yielded = 0; // cars that stopped for someone on a crossing
	// M5
	uint32_t residents = 0;
};

enum class PedState : uint8_t {
	Walking = 0,
	WaitingToCross = 1,
	Crossing = 2,
	WaitingForBus = 3,
	Riding = 4,
};
const char *ped_state_name(PedState s);

struct Pedestrian {
	uint32_t id = 0;
	PedState state = PedState::Walking;
	// On edge `edge`, walking from node `from` towards its other end, d metres in.
	int32_t edge = -1;
	int32_t from = -1;
	double d = 0.0;
	double prev_d = 0.0;
	int32_t prev_edge = -1, prev_from = -1;
	std::vector<int32_t> path; // edges still to walk
	size_t pi = 0;
	double speed = 1.35; // m/s
	// Places (spawn points, then buildings, then the main station; see
	// Traffic::place_count). dest -1: leaves by coach.
	int32_t origin = -1, dest = -1;
	uint32_t resident = 0; // M5: the resident or visitor making this trip (0 none)
	int32_t target = -1; // ped node this leg of the trip ends at
	// Transit: board `route` at stop `board`, ride to `alight`; then the second ride (one transfer).
	uint32_t route = 0, route2 = 0;
	int32_t board = -1, alight = -1, board2 = -1, alight2 = -1; // Network::stops indices
	bool coach = false; // waiting for any coach at the main station
	VehicleId vehicle = kNoId; // riding
	uint64_t wait_since = 0; // tick it started waiting (kerb or stop)
	uint64_t spawn_tick = 0;
	bool done = false;
};

// --- City life (M5) -----------------------------------------------------------

constexpr uint32_t kOutside = 0xFFFFFFFFu; // "outside the map" as a building id

enum class ResidentState : uint8_t {
	Inside = 0, // in a building (cheap: updated once per sim minute)
	Travelling = 1, // a pedestrian (or riding a bus or coach)
	Outside = 2, // away outside the map, back by coach
};
enum class Doing : uint8_t {
	Idle = 0,
	Sleep = 1,
	Offering = 2, // a meal, shopping
	Work = 3,
};
const char *doing_name(Doing d);

// A shift a resident (or visitor) has taken.
struct Booking {
	uint32_t building = 0; // 0 none, kOutside for a job outside the map
	uint64_t start = 0, end = 0; // ticks
	bool weekend = false;
	bool clocked = false; // at work now
	bool break_taken = false;
	bool second_break = false; // very hungry later in the shift
	bool done = false;
};

struct Resident {
	uint32_t id = 0;
	int32_t household = -1; // index in households (-1: a visitor)
	bool visitor = false;
	bool works = true; // looks for a job
	ResidentState state = ResidentState::Inside;
	uint32_t at = 0; // building id while inside
	uint32_t ped = 0; // pedestrian id while travelling
	int32_t going = -1; // place the trip ends at
	uint32_t going_building = 0; // building id the trip ends at (0: none, kOutside: away)
	Doing doing = Doing::Idle;
	int offering = -1; // current or intended offering (CityData index)
	uint64_t until = 0; // tick the current activity ends
	uint64_t next_plan = 0;
	uint64_t updated = 0; // tick the needs were last brought up to date
	double hunger = 80.0, energy = 80.0, money = 0.0;
	uint32_t employer = 0; // building id, kOutside, or 0 none
	int week_minutes = 0; // booked this week
	Booking shift;
	uint32_t then = 0; // trip chaining: building to go on to after this stop
	uint64_t outside_until = 0; // back at the main station from then on
	std::vector<std::pair<uint32_t, uint64_t>> closed; // (building, remembered until tick)
	uint32_t late = 0; // shifts started late
	bool leaving = false; // visitor on the way out
};

struct Household {
	uint32_t id = 0;
	uint32_t home = 0; // building id
	int unit = 0;
	double pantry = 7.0; // portions
	std::vector<uint32_t> members; // resident ids
};

struct ResidentInfo {
	bool found = false;
	uint32_t id = 0;
	bool visitor = false;
	ResidentState state = ResidentState::Inside;
	Doing doing = Doing::Idle;
	std::string activity; // what it is doing or going to
	uint32_t at = 0, home = 0, employer = 0, going = 0;
	double hunger = 0.0, energy = 0.0, money = 0.0, pantry = 0.0;
	int household_size = 0;
	uint64_t shift_start = 0, shift_end = 0; // ticks (0: no shift booked)
	uint32_t shift_building = 0;
	double until = 0.0; // s left in the activity
	uint32_t late = 0;
};

struct BuildingInfo {
	bool found = false;
	uint32_t id = 0;
	int type = -1;
	int households = 0, units = 0, residents = 0, inside = 0;
	int employees = 0, headcount = 0, staff_in = 0, customers = 0, booked_today = 0, unfilled_today = 0;
	bool in_hours = false, open = false, closed_unexpectedly = false;
	int opened_at = -1; // minute of the day it first opened today (-1: not yet)
	int late_minutes_today = 0; // opened this much after its opening time
	int unexpected_minutes_today = 0;
	uint64_t served = 0, turned_away = 0, late_openings = 0;
};

struct CityStats {
	bool on = false;
	int day = 0; // since the start
	int weekday = 0; // 0 Monday
	int minute = 0; // of the day
	uint32_t residents = 0, households = 0, visitors = 0;
	uint32_t inside = 0, travelling = 0, outside = 0, sleeping = 0, working = 0;
	uint32_t employed = 0, employed_outside = 0, unemployed = 0;
	uint32_t homes = 0, units = 0, vacant_units = 0;
	uint32_t businesses = 0, open = 0, closed_unexpectedly = 0;
	uint64_t immigrants = 0, visitor_trips = 0;
	uint64_t meals_out = 0, home_meals = 0, groceries = 0, sleeps = 0;
	uint64_t shifts = 0, late_shifts = 0, late_openings = 0, turned_away = 0, unfilled_shifts = 0;
	double mean_hunger = 0.0, mean_energy = 0.0, min_hunger = 0.0, mean_money = 0.0;
	uint32_t starving = 0; // hunger at 0
};

struct PedInfo {
	uint32_t id = 0;
	bool found = false;
	PedState state = PedState::Walking;
	double speed = 0.0;
	double waited = 0.0;
	double trip_time = 0.0;
	NodeId origin = kNoId, dest = kNoId; // map nodes (dest kNoId: by coach)
	int32_t board = -1, alight = -1; // stops
	uint32_t route = 0;
	VehicleId vehicle = kNoId;
	uint32_t resident = 0; // M5
	std::vector<Vec2> route_line; // where it will walk
};

struct StopStats {
	int32_t stop = -1;
	uint32_t waiting = 0;
	uint64_t boarded = 0, alighted = 0, left_behind = 0;
	double mean_wait = 0.0; // s
};

// Mean load leaving each stop of a route (passengers on board), for the inspector.
struct RouteLoad {
	uint32_t route = 0;
	std::vector<double> load; // per stop in route order
};

struct VehicleInfo {
	VehicleId id = kNoId;
	bool found = false;
	VehicleKind kind = VehicleKind::Car;
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
	uint32_t bus_route = 0;
	uint32_t coach_line = 0;
	int32_t next_stop = -1; // Network::stops index of the next stop
	size_t stops_left = 0;
	bool parks = false; // will park on the way
	std::vector<Vec2> route; // polyline from the car to its destination
};

// Per bus route, for the inspector and tests.
struct RouteStats {
	uint32_t id = 0;
	uint32_t active = 0; // buses out on the route now
	uint64_t runs = 0; // finished runs
	double round_trip = 0.0; // s, estimated at free flow including dwells
	uint32_t fleet = 0; // buses needed: round trip / headway
};

class Traffic {
public:
	explicit Traffic(TrafficConfig config = {});

	// Switches to a (re)compiled network. Vehicles, routes and grants are
	// carried over by stable lane keys; cars on lanes that no longer exist are
	// removed and cars whose route broke are re-routed.
	void set_network(const Network *net);
	const Network *network() const { return net_; }

	void reset(uint64_t seed); // removes all vehicles, resets the clock and RNG
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
	std::vector<RouteStats> route_stats() const;
	uint64_t state_hash() const;
	// Occupied bays (Network::bays indices), for drawing and tests.
	const std::vector<VehicleId> &bay_use() const { return bay_use_; }

	// --- People (M4) ---------------------------------------------------------------
	const std::vector<Pedestrian> &pedestrians() const { return peds_; }
	int32_t find_pedestrian(uint32_t id) const;
	Pose ped_pose(size_t i, double alpha) const; // waiting people stand a little apart
	int ped_level(size_t i) const;
	PedInfo ped_info(uint32_t id) const;
	std::vector<StopStats> stop_stats() const;
	std::vector<RouteLoad> route_loads() const;
	// Light a mid-block push-button signal shows cars (crossing index), for drawing.
	SignalLight crossing_car_light(int32_t crossing) const;
	WalkLight crossing_walk_light(int32_t crossing) const;
	// For tests and tools: a person walking from one ped node to another.
	uint32_t add_pedestrian(int32_t from_node, int32_t to_node, double speed = 1.35);
	// Shortest perceived route on foot (edge list); false when there is none.
	bool find_walk(int32_t from_node, int32_t to_node, std::vector<int32_t> &path, double *cost = nullptr) const;

	// --- City life (M5) -------------------------------------------------------------
	// On when the map has buildings: residents live in homes, work shifts and
	// shop; immigrants and visitors come by coach.
	bool city_on() const { return city_on_; }
	const CityData &city_data() const { return *city_data_; }
	// Another data table (the default is the built-in one). Takes effect at the next reset.
	void set_city_data(const CityData *data) { city_data_ = data ? data : &default_city_data(); }
	int64_t ticks_per_minute() const;
	int64_t clock_minutes() const; // minutes since Monday 00:00 of week 0
	int day() const { return static_cast<int>(clock_minutes() / 1440); }
	int minute_of_day() const { return static_cast<int>(clock_minutes() % 1440); }
	bool is_weekend() const { return day() % 7 >= 5; }
	const std::vector<Resident> &residents() const { return res_; }
	const std::vector<Household> &households() const { return hh_; }
	int32_t find_resident(uint32_t id) const;
	ResidentInfo resident_info(uint32_t id) const;
	BuildingInfo building_info(uint32_t building_id) const;
	CityStats city_stats() const;
	// Places people travel between: spawn points, buildings, the main station.
	size_t place_count() const { return place_entries_.size(); }
	int32_t building_place(uint32_t building_id) const;
	int32_t station_place() const { return station_place_; }
	double travel_estimate(int32_t from_place, int32_t to_place) const; // s (1e9: no way)
	// For tests: a household moves into a home unit now (returns its index, -1 if full).
	int32_t add_household(uint32_t home, int people, bool works = true);

	// For tests and tools: add a vehicle directly (returns its id, 0 on failure).
	// Driver parameters default to the kind's typical driver.
	VehicleId add_vehicle(int32_t lane, double s, double v, NodeId dest, const DriverParams2 *driver = nullptr,
			VehicleKind kind = VehicleKind::Car, const std::vector<Waypoint> &waypoints = {});
	// A* on the lane graph from a road lane to a sink node. Fills `route` with
	// the connectors to take and returns false when there is no route.
	bool find_route(int32_t start, NodeId dest, std::vector<int32_t> &route, bool allow_change_first = true,
			VehicleKind kind = VehicleKind::Car) const;
	// A* from a road lane to a given lane (at or beyond distance s along it).
	bool find_route_to_lane(int32_t start, double start_s, int32_t goal, double goal_s, std::vector<int32_t> &route,
			VehicleKind kind, bool allow_change_first = true) const;
	// Whether a vehicle kind may drive in a lane at all.
	static bool allowed(VehicleKind k, const NetLane &l);
	static DriverParams2 typical_driver(VehicleKind k);

private:
	struct Candidate {
		int32_t veh = -1;
		int32_t conn = -1;
		double dist = 0.0; // to the stop line
	};
	struct Goal {
		NodeId node = kNoId; // any sink lane at this node
		int32_t lane = -1; // or this lane
		bool need_connector = false; // the goal lane must be reached again (it is behind)
		Vec2 pos;
	};

	double desired_speed(const Vehicle &v, int32_t lane) const;
	double idm(const Vehicle &v, double v0, bool has, double gap, double lead_v) const;
	int32_t next_connector(const Vehicle &v, int32_t lane, size_t ri) const;
	int good_direction(const Vehicle &v, int32_t lane, int &steps) const;
	bool lane_is_good(const Vehicle &v, int32_t lane, size_t ri) const;
	bool at_route_end(const Vehicle &v, int32_t lane, size_t ri) const; // drives off the map here
	bool exits_at(const NetLane &l, NodeId node) const; // a lane that leaves the map at a sink or depot
	bool goal_pos(NodeId node, Vec2 &pos) const; // false when vehicles can't end their trip there
	const Waypoint *pending_waypoint(const Vehicle &v) const;
	void leader(const Vehicle &v, int32_t lane, double s, size_t ri, bool &has, double &gap, double &lead_v,
			double &v0_cap, VehicleId &who) const;
	double accel_at(size_t i, int32_t lane, double s) const;
	bool box_clear(const NetJunction &j, int32_t conn, const std::vector<int32_t> &granted, VehicleId &blocker) const;
	bool exit_clear(int32_t conn, int32_t veh, const std::vector<int32_t> &granted) const;
	double pos_on(const Vehicle &v, int32_t conn) const;
	double time_to(const Vehicle &v, double dist) const;
	bool astar(int32_t start, const Goal &goal, VehicleKind kind, bool allow_change_first,
			std::vector<int32_t> &route) const;
	double lane_cost(int32_t lane, VehicleKind kind) const;
	SignalLight light_of(int32_t conn) const;

	void arbitrate();
	void change_lanes();
	void move();
	bool waypoints(); // true when a vehicle had to be taken off the map
	void begin_waypoint(size_t i);
	void complete_waypoint(Vehicle &v);
	void skip_waypoint(Vehicle &v);
	bool retarget(Vehicle &v); // routes to the next waypoint or the destination, dropping what can't be reached
	void spawn();
	void spawn_bike(size_t k);
	bool spawn_bike_to(size_t k, NodeId dest);
	NodeId pick_dest(size_t k);
	DriverParams2 random_driver(VehicleKind k);
	void dispatch();
	void init_transit(bool keep);
	void recount_use();
	void list_insert(size_t i);
	void list_remove(size_t i);
	void rebuild_lists();
	void compact();
	bool reroute(Vehicle &v, bool allow_change_first);
	void rebuild_reachability();
	void estimate_routes();
	bool lane_free_at(int32_t lane, double s, double length) const;
	VehicleId insert(Vehicle v, int32_t lane, double s); // adds a spawned vehicle to the lane lists

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
	// Roundabouts: circulating lanes and how many car-sized slots each lane
	// (0 = outer) may fill, one less than it holds so it can always turn.
	std::vector<std::vector<int32_t>> ring_lanes_; // per junction
	std::vector<std::vector<int32_t>> ring_slots_; // per junction, per circulating lane
	std::vector<int8_t> ring_cycle_; // per lane: which circulating lane it is (-1 = none)
	std::vector<int32_t> ring_load(size_t j) const; // slots taken on each circulating lane, entries included
	std::vector<VehicleId> bay_use_; // per Network::bays: who has it (0 = free)
	std::vector<uint32_t> stop_use_; // per Network::stops: buses in the bays
	struct RouteRun {
		uint32_t id = 0;
		NodeId depot = kNoId;
		size_t depot_idx = 0, route_idx = 0; // into Network::depots and its routes
		uint64_t next_departure = 0;
		uint64_t runs = 0;
		double round_trip = 0.0;
	};
	std::vector<RouteRun> routes_;
	struct CoachRun {
		uint32_t id = 0;
		size_t line = 0; // into Network::coach_lines
		uint64_t next_departure = 0;
	};
	std::vector<CoachRun> coaches_;

	// --- People (M4) ---------------------------------------------------------------
	void peds_prepare(); // crossing occupancy and mid-block signals, before the cars move
	void peds_tick(); // walk, cross, wait; after the cars moved
	void people_demand(); // new trips with mode choice
	bool cross_ok(const Pedestrian &p, int32_t edge, int32_t from) const;
	int crossing_blocks(int32_t crossing, int32_t lane, const Vehicle &v) const; // for cars: 0 free, 1 stop, 2 stop if it comfortably can
	bool lane_clear_for(int32_t lane, double s0, double s1, double ped_time, bool zebra) const;
	double edge_cost(const PedEdge &e) const;
	void ped_costs(); // per spawner: perceived cost to every ped node (Dijkstra)
	void route_times(); // ride times between stops of each route
	void start_walk(Pedestrian &p, int32_t to_node);
	void ped_arrived(size_t i);
	// Alight and board (sets a bus's dwell); `late`: only people who just arrived board. Returns boarders.
	int serve_stop(Vehicle &v, int32_t stop, Waypoint &wp, bool late = false);
	bool serves_people(const Vehicle &v, const Waypoint &wp) const {
		return (people_on_ || city_on_) && wp.stop >= 0 && (v.kind == VehicleKind::Bus || v.kind == VehicleKind::Coach);
	}
	void bus_departs(Vehicle &v, int32_t stop);
	uint64_t boarding_ticks(const Vehicle &v, int people) const;
	struct PedSaved {
		Vec2 at, target;
		int at_level = 0, target_level = 0;
		bool has_target = false;
		uint32_t board = 0, alight = 0, board2 = 0, alight2 = 0; // stop ids (0 none)
		NodeId origin = kNoId, dest = kNoId; // spawn point nodes
		uint32_t dest_building = 0; // or a building id
		bool dest_station = false; // or the main station
	};
	std::vector<PedSaved> peds_save() const; // before the network changes
	void peds_restore(const std::vector<PedSaved> &saved); // after: by position, stop id and spawn point
	void peds_reset_network(); // per-network people state
	// Bus legs between two places (one transfer at most): generalized cost
	// (walking x2, headway, ride, transfer penalty), or 1e300 with no bus.
	struct BusChoice {
		size_t r1 = 0, i1 = 0, j1 = 0; // rides_ index and stop positions
		size_t r2 = 0, i2 = 0, j2 = 0;
		bool transfer = false;
	};
	double best_bus(size_t from_place, size_t to_place, BusChoice &out) const;
	int32_t platform_of(int32_t stop) const;
	void apply_bus_choice(Pedestrian &p, const BusChoice &c) const;

	// City (traffic_city.cpp)
	struct BState { // per Network::buildings
		std::vector<int32_t> units; // household index per unit (-1 vacant)
		std::vector<uint32_t> employees; // resident ids
		int staff_in = 0, customers = 0;
		bool in_hours = false, open = false, unexpected = false;
		int opened_at = -1, late_minutes = 0, unexpected_minutes = 0;
		int booked_today = 0, unfilled_today = 0;
		std::vector<int> slot_booked; // today's plan: staff booked per shift
		uint64_t served = 0, turned_away = 0, late_openings = 0;
	};
	struct VisitorJob {
		uint32_t building = 0;
		uint64_t start = 0, end = 0;
		int count = 0; // still to send
	};
	void city_reset_network(); // places, costs, travel times; carries city state over by building id
	void city_init(); // at reset: the households of city_prefill, today's shifts
	void city_tick(); // after the people moved
	void city_daily();
	void city_minute();
	void city_update(size_t i);
	void city_plan(size_t i);
	void city_arrive(uint32_t resident);
	void city_board_coach(uint32_t resident);
	int city_coach_arrives(int seats); // returns people brought
	void city_finish(size_t i); // the current activity is over
	void city_clock_in(Resident &r, BState &b, uint64_t now);
	void city_clock_out(Resident &r);
	bool city_trip(size_t i, int32_t to_place, uint32_t to_building, bool by_coach);
	bool city_start_offering(size_t i, int offering);
	void city_choose_job(size_t i);
	bool city_book(size_t i); // a shift today at its employer, if one is still open
	int32_t city_add_household(uint32_t home, int people, double work_share);
	void city_leave_building(Resident &r);
	void city_recount();
	BState *bstate(uint32_t building_id);
	const BState *bstate(uint32_t building_id) const;
	uint64_t tick_of(int day, int minute) const; // clock -> tick (0 when before the start)
	int64_t clock_at(uint64_t tick) const; // minutes since Monday 00:00 of week 0
	bool coaches_run() const;
	int32_t new_resident(bool visitor);
	double minutes_between(int32_t from_place, uint32_t to_building) const; // estimate, minutes
	std::vector<Pedestrian> peds_;
	uint32_t next_ped_id_ = 1;
	bool people_on_ = false; // some demand on foot: dwell comes from boarding
	std::vector<std::vector<std::pair<double, int8_t>>> on_crossing_; // per crossing: (t from a, +1/-1 walking direction)
	std::vector<std::vector<uint32_t>> waiting_at_; // per crossing: people waiting at a kerb (push buttons)
	struct MidSignal {
		uint8_t state = 0; // 0 car green, 1 car amber, 2 all red, 3 walk, 4 flashing, 5 clearance
		uint64_t since = 0;
	};
	std::vector<MidSignal> mid_signal_; // per crossing (push-button signals)
	bool crossings_live_ = false; // someone on or at a crossing, or a push-button signal not green
	std::vector<std::vector<double>> ped_cost_; // per ped spawner: cost to every node (s)
	struct RideTimes {
		uint32_t route = 0;
		size_t depot = 0, index = 0;
		std::vector<int32_t> stops; // in order, a loop repeats the first
		std::vector<double> at; // cumulative ride time at each stop (s)
		double headway = 600.0;
	};
	std::vector<RideTimes> rides_;
	std::vector<std::vector<NodeId>> car_trips_; // per vehicle spawner: destinations chosen by people
	struct StopAcc {
		uint64_t boarded = 0, alighted = 0, left_behind = 0;
		double wait_sum = 0.0;
	};
	std::vector<StopAcc> stop_acc_;
	std::map<uint32_t, std::vector<std::pair<double, uint64_t>>> load_acc_; // route -> per stop index (sum, count)
	double crossing_wait_sum_ = 0.0;
	double wait_sum_ = 0.0;
	// --- City life (M5) ---------------------------------------------------------------
	const CityData *city_data_ = &default_city_data();
	bool city_on_ = false;
	std::vector<std::vector<int32_t>> place_entries_; // per place: ped nodes
	int32_t station_place_ = -1;
	std::vector<float> travel_; // place x place, s
	std::vector<BState> bstate_;
	std::vector<Resident> res_; // ascending id
	uint32_t next_res_id_ = 1;
	std::vector<Household> hh_;
	uint32_t next_hh_id_ = 1;
	std::vector<VisitorJob> visitor_jobs_;
	CityStats city_acc_; // running counters
	uint64_t city_last_day_ = ~0ull;
	std::vector<uint32_t> city_arrivals_; // residents whose trip ended this tick
	std::vector<uint32_t> city_boarded_; // residents who left on a coach this tick
	TrafficStats stats_;
	double trip_time_sum_ = 0.0;
};

} // namespace tsim
