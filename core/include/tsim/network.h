// Sim network (M2): the editable map compiled into a lane graph.
//
// Road lanes (one per drawn travel lane, turn pockets included) and connectors
// (paths through nodes) are both "lanes" here: a polyline with a length that
// vehicles drive along. Each junction gets a conflict matrix: which connector
// pairs cross or merge, where along each path, and who has priority.
//
// Determinism: geometry is built with trig, which can differ in the last bit
// between platforms. Everything the simulation reads is snapped to a 1/1024 m
// grid first, and lengths are then summed with sqrt only, so the compiled
// network is bit-identical everywhere.
//
// Incremental: NetworkCompiler keeps the compiled data of every segment and
// node with a signature of its inputs. After an edit only the segments and
// junctions whose inputs changed are compiled again; the result is identical
// to a full compile.
#pragma once

#include "tsim/road_geometry.h"
#include "tsim/road_map.h"

#include <map>
#include <vector>

namespace tsim {

enum class NetLaneKind : uint8_t {
	Road = 0,
	Connector = 1,
};

// Stable identity of a lane across recompiles: a road lane by its LaneId, a
// connector by the pair of road lanes it joins.
struct LaneKey {
	NetLaneKind kind = NetLaneKind::Road;
	uint32_t a = 0; // road: lane id; connector: from lane
	uint32_t b = 0; // connector: to lane
	bool operator<(const LaneKey &o) const {
		return kind != o.kind ? kind < o.kind : a != o.a ? a < o.a : b < o.b;
	}
	bool operator==(const LaneKey &o) const { return kind == o.kind && a == o.a && b == o.b; }
};

// A crossing or merging pair of connectors at one junction.
struct Conflict {
	int32_t other = -1; // dense lane index of the other connector
	double s_self = 0.0; // conflict point along this connector
	double s_other = 0.0; // and along the other one
	int8_t priority = 0; // +1 this connector goes first, -1 the other, 0 first come first served
	bool merge = false; // both end in the same lane
};

struct NetLane {
	NetLaneKind kind = NetLaneKind::Road;
	LaneKey key;
	LaneType type = LaneType::General;
	int level = 0;
	int level_end = 0; // ramps (M4): the level at the lane's end
	double length = 0.0;
	double speed_limit = 13.9; // m/s; connectors: also limited by curvature
	std::vector<Vec2> pts; // travel direction
	std::vector<double> cum; // distance along the lane at each point

	// Road lanes
	SegmentId segment = kNoId;
	LaneDir dir = LaneDir::Forward;
	bool pocket = false;
	int32_t first_sample = 0; // index of pts[0] in the segment's samples, counted in travel order
	int32_t left = -1, right = -1; // same-direction neighbours, in the sense of travel
	// Where a lane change out of this lane is allowed, as [from, to] ranges of s.
	std::vector<std::pair<double, double>> change_left, change_right;
	NodeId start_node = kNoId;
	NodeId end_node = kNoId;
	bool sink = false; // ends at a spawn point that accepts vehicles
	int32_t approach_of = -1; // junction this lane ends at (if it controls entry)
	bool merge_end = false; // lane ends by merging into its neighbour (lane drop)
	bool ring = false; // circulating lane of a roundabout

	// Connectors
	NodeId node = kNoId;
	int32_t junction = -1;
	int32_t from = -1, to = -1; // road lanes
	TurnKind turn = TurnKind::Straight;
	int leg = -1; // index of the incoming leg at the node
	std::vector<Conflict> conflicts;
	double route_penalty = 0.0; // s added by routing
	int32_t movement = -1; // signal movement at a signalized junction

	// Graph: road lane -> its connectors; connector -> its road lane.
	std::vector<int32_t> next;
	std::vector<int32_t> prev;
};

// A fixed-time signal program in ticks (M3).
struct NetSignal {
	bool enabled = false;
	std::vector<int64_t> green; // per phase
	int64_t amber = 30, all_red = 20, offset = 0, cycle = 0;
	// state[phase][movement]: 0 red, 1 green, 2 green but yield (permissive)
	std::vector<std::vector<uint8_t>> state;
	std::vector<uint8_t> right_on_red; // per movement: EU flashing green arrow
	std::vector<std::pair<SegmentId, SegmentId>> movements; // (from leg, to leg)
	std::vector<std::vector<SegmentId>> walk; // per phase: legs whose crosswalk shows walk (M4)
};

enum class WalkLight : uint8_t { DontWalk = 0, Walk = 1, Flashing = 2 };

enum class SignalLight : uint8_t { Red = 0, Green = 1, GreenYield = 2, Amber = 3 };

struct NetJunction {
	NodeId node = kNoId;
	int level = 0;
	Vec2 pos;
	JunctionControl control = JunctionControl::RightHand;
	std::vector<int32_t> connectors;
	std::vector<int32_t> approaches; // road lanes that end here
	bool arbitrated = false; // some connectors conflict: entry needs a grant
	bool joint = false; // two roads joined end to end (continuation or taper)
	bool roundabout = false;
	NetSignal signal;
	// Light for a movement at a tick (signalized junctions only).
	SignalLight light(int32_t movement, int64_t tick) const;
	// Phase index and ticks into it, for display.
	int phase_at(int64_t tick, int64_t *into = nullptr) const;
	// Pedestrian light of the crosswalk across leg `leg` (M4); flashing for the
	// last `clear_ticks` of the walk phase's green.
	WalkLight walk_light(SegmentId leg, int64_t tick, int64_t clear_ticks) const;
};

// --- Pedestrian network (M4) -----------------------------------------------------------

enum class PedEdgeKind : uint8_t {
	Walk = 0, // sidewalk, path, corner
	Crossing = 1, // across a road (see NetCrossing)
	Ramp = 2, // a level change on a ramp
	Stairs = 3,
};

struct PedNode {
	Vec2 pos;
	int level = 0;
};

struct PedEdge {
	int32_t a = -1, b = -1;
	double length = 0.0;
	PedEdgeKind kind = PedEdgeKind::Walk;
	int32_t crossing = -1; // NetCrossing index (Crossing edges)
	int rise = 0; // levels climbed from a to b
};

// Where a car lane passes over a crossing: lane distances [s0, s1] (the car's
// front must not enter it while it is blocked) and the part of the crossing
// line the lane covers, as distances from the crossing's a end.
struct CrossingSpan {
	int32_t lane = -1;
	double s0 = 0.0, s1 = 0.0;
	double t0 = 0.0, t1 = 0.0;
};

struct NetCrossing {
	SegmentId seg = kNoId;
	int end = -1; // 0/1 at a junction leg, -1 mid-block
	uint32_t id = 0;
	NodeId node = kNoId;
	// As it behaves: signal crossings at junctions without signals act as zebras.
	CrossingKind kind = CrossingKind::Zebra;
	bool unmarked = false; // an informal crossing (uncontrolled; no paint, costs more)
	bool push_button = false; // a mid-block signal: cars get red only when someone waits
	int32_t junction = -1; // NetJunction index of a signal crossing at a junction
	int level = 0;
	Vec2 a, b;
	double length = 0.0;
	std::vector<int32_t> edges; // its ped edges, a -> b (two with a refuge)
	std::vector<CrossingSpan> spans;
	int64_t clear_ticks = 80; // flashing time (signals): walking the whole length
};

struct PedStop {
	int32_t stop = -1; // Network::stops index
	int32_t node = -1; // the platform
	Vec2 pos;
	int level = 0;
};

struct PedSpawner {
	NodeId node = kNoId;
	Vec2 pos;
	int level = 0;
	double people = 0.0; // trips per hour starting here
	bool sink = true;
	std::vector<OdWeight> od;
	std::vector<int32_t> entries; // ped nodes where people appear and leave
	bool road = false; // also a vehicle spawn point (cars and bikes can start here)
};

struct PedGraph {
	std::vector<PedNode> nodes;
	std::vector<PedEdge> edges;
	std::vector<std::vector<int32_t>> adj; // node -> edge indices
	std::vector<NetCrossing> crossings;
	std::vector<std::vector<int32_t>> lane_crossings; // per Network lane: crossing indices with a span on it
	std::vector<PedStop> stops;
	std::vector<PedSpawner> spawners;
	void clear();
};

struct NetBay {
	LaneId parking_lane = kNoId;
	int index = 0;
	int32_t lane = -1; // travel lane cars park from
	double s = 0.0; // along that lane
	Vec2 pos;
	Vec2 dir;
	ParkingStyle style = ParkingStyle::Parallel;
};

struct NetStop {
	uint32_t id = 0;
	SegmentId segment = kNoId;
	StopKind kind = StopKind::Kerbside;
	std::string name;
	int32_t lane = -1; // kerb-side lane of the direction it serves
	double s = 0.0;
	int bays = 1;
	Vec2 pos; // where a bus waits
	Vec2 dir;
};

struct NetRoute {
	uint32_t id = 0;
	std::string name;
	uint32_t color = 0;
	std::vector<int32_t> stops; // indices into Network::stops
	double headway = 600.0;
	bool loop = false;
};

struct NetDepot {
	NodeId node = kNoId;
	std::string name;
	Vec2 pos;
	int level = 0;
	int capacity = 20;
	std::vector<int32_t> spawn_lanes, sink_lanes;
	std::vector<NetRoute> routes;
};

struct NetCoachLine {
	uint32_t id = 0;
	NodeId entry = kNoId, exit = kNoId;
	double per_hour = 2.0;
	double dwell = 600.0;
};

struct NetSpawner {
	NodeId node = kNoId;
	Vec2 pos;
	int level = 0;
	Spawner config;
	std::vector<int32_t> spawn_lanes; // road lanes leaving the map edge (cars)
	std::vector<int32_t> bike_lanes; // where bicycles enter
	std::vector<int32_t> sink_lanes; // road lanes arriving at it
};

class Network {
public:
	std::vector<NetLane> lanes;
	std::vector<NetJunction> junctions; // ascending node id
	std::vector<NetSpawner> spawners; // ascending node id, enabled road ends only
	std::vector<NetBay> bays;
	std::vector<NetStop> stops;
	std::vector<NetDepot> depots; // ascending node id
	std::vector<NetCoachLine> coach_lines;
	int32_t main_station = -1; // index into stops
	PedGraph ped; // M4
	int32_t stop_index(uint32_t id) const;
	double max_speed = 13.9; // fastest speed limit, for the routing heuristic

	int32_t find(const LaneKey &k) const;
	int32_t junction_at(NodeId node) const;
	const NetSpawner *spawner_at(NodeId node) const;

	// Position and heading at distance s along a lane (s is clamped).
	Pose pose(int32_t lane, double s) const;
	// Distance along road lane `to` level with distance s along its neighbour
	// `from` (same segment and direction), or -1 when `to` doesn't exist there.
	double map_across(int32_t from, double s, int32_t to) const;
	bool can_change(int32_t lane, bool to_left, double s) const;

	// Hash of everything the simulation reads (used by tests).
	uint64_t hash() const;
	void clear();

	friend class NetworkCompiler;

private:
	std::map<LaneKey, int32_t> index_;
	std::map<NodeId, int32_t> junction_index_;
};

struct CompileStats {
	int segments = 0;
	int junctions = 0;
	int segments_compiled = 0; // compiled this time (the rest came from the cache)
	int junctions_compiled = 0;
	double ms = 0.0;
};

class NetworkCompiler {
public:
	// Compiles `map` (whose geometry is `geom`) into `out`, reusing cached
	// segments and junctions whose inputs did not change.
	void compile(const RoadMap &map, const RoadGeometry &geom, Network &out);
	void clear_cache();
	const CompileStats &stats() const { return stats_; }

	// Tunables.
	double lateral_accel = 2.0; // m/s^2, sets turn speeds on connectors
	double conflict_distance = 1.6; // paths closer than this (m) conflict

private:
	struct SegmentPart {
		uint64_t sig = 0;
		std::vector<NetLane> lanes; // left/right are local indices
	};
	struct NodePart {
		uint64_t sig = 0;
		std::vector<NetLane> ring; // roundabout lanes; left/right are local
		std::vector<NetLane> connectors; // conflicts[].other and leg are local
		bool arbitrated = false;
		bool joint = false;
		bool roundabout = false;
		std::vector<std::pair<SegmentId, SegmentId>> movements; // signalized junctions
	};
	std::map<SegmentId, SegmentPart> segments_;
	std::map<NodeId, NodePart> nodes_;
	CompileStats stats_;
};

// Checks that need the compiled network: spawn points that are not on a road
// end, and origin-destination pairs with no route.
struct NetProblem {
	std::string code;
	std::string message;
	Vec2 pos;
	int level = 0;
	NodeId node = kNoId;
};
std::vector<NetProblem> network_problems(const RoadMap &map, const Network &net);

// Snaps a coordinate to the 1/1024 m grid.
double quantize(double v);

} // namespace tsim
