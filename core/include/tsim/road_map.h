// Editable road map (M1): what the player draws and what save files store.
//
// Nodes and segments are the source of truth. Everything else (lane polygons,
// junction shapes, connectors, markings) is derived by RoadGeometry and can be
// regenerated at any time. Every object has a stable ID that is never reused.
#pragma once

#include "tsim/curve.h"
#include "tsim/types.h"

#include <map>
#include <string>
#include <vector>

namespace tsim {

enum class SegmentKind : uint8_t {
	Road = 0,
	Footpath = 1, // data only until M4
	BikePath = 2, // data only until M4
	SharedPath = 3, // data only until M4
};

enum class LaneType : uint8_t {
	General = 0,
	Bus = 1,
	Bike = 2,
	Turn = 3,
	Parking = 4,
	Sidewalk = 5,
};

// Travel direction relative to the segment's from -> to orientation.
enum class LaneDir : uint8_t {
	Forward = 0,
	Backward = 1,
	None = 2, // sidewalks and parking
};

enum class MedianType : uint8_t {
	None = 0,
	Painted = 1,
	Raised = 2,
};

enum class TurnRule : uint8_t {
	Disallowed = 0,
	Allowed = 1,
	TurnLane = 2, // a dedicated pocket lane for the last N metres
};

// How cars park in a parking lane (M3).
enum class ParkingStyle : uint8_t {
	Parallel = 0, // 2.0 x 6.0 m bays, pull past and reverse in
	Angle45 = 1, // 2.5 x 5.0 m bays at 45 degrees, forward in, reverse out
	Perpendicular = 2, // 2.5 x 5.0 m bays at 90 degrees
};
const char *parking_style_name(ParkingStyle s);
bool parking_style_from_name(const std::string &s, ParkingStyle &out);
double parking_depth(ParkingStyle s); // lane width for the style (m)
double parking_bay_length(ParkingStyle s); // bay length along the kerb (m)

struct LaneSpec {
	LaneId id = kNoId;
	LaneType type = LaneType::General;
	LaneDir dir = LaneDir::Forward;
	double width = 3.25;
	ParkingStyle parking = ParkingStyle::Parallel; // parking lanes only

	bool operator==(const LaneSpec &o) const {
		return id == o.id && type == o.type && dir == o.dir && width == o.width && parking == o.parking;
	}
};

// Cross-section, lanes listed left -> right looking from the from-node to the
// to-node. Valid order: [sidewalk/parking...][backward travel...][forward travel...][parking/sidewalk...].
struct Profile {
	std::vector<LaneSpec> lanes;
	MedianType median = MedianType::None;
	double median_width = 0.0;

	bool operator==(const Profile &o) const {
		return lanes == o.lanes && median == o.median && median_width == o.median_width;
	}
	int count(LaneDir dir) const;
	bool one_way() const { return count(LaneDir::Forward) == 0 || count(LaneDir::Backward) == 0; }
	bool has_travel() const { return count(LaneDir::Forward) + count(LaneDir::Backward) > 0; }
	double total_width() const;
};

bool is_travel(LaneType t);
double default_width(LaneType t);
const char *lane_type_name(LaneType t);
bool lane_type_from_name(const std::string &s, LaneType &out);

// Turn rules for the lanes that arrive at one end of a segment.
struct EndRules {
	TurnRule left = TurnRule::Allowed;
	TurnRule right = TurnRule::Allowed;
	double turn_lane_length = 40.0; // metres, for TurnLane pockets
	// Stable IDs of generated pocket lanes (0 until the rule is TurnLane).
	LaneId left_lane = kNoId;
	LaneId right_lane = kNoId;

	bool operator==(const EndRules &o) const {
		return left == o.left && right == o.right && turn_lane_length == o.turn_lane_length &&
				left_lane == o.left_lane && right_lane == o.right_lane;
	}
};

// A painted no-change zone on the line between lane edge-1 and lane edge
// (edge in 1..lanes-1), from u0 to u1 as fractions of the centreline length.
struct NoChangeZone {
	int edge = 1;
	double u0 = 0.0;
	double u1 = 1.0;
	// Which crossings are forbidden: from the left lane into the right lane,
	// and/or from the right lane into the left lane.
	bool block_left_to_right = true;
	bool block_right_to_left = true;

	bool operator==(const NoChangeZone &o) const {
		return edge == o.edge && u0 == o.u0 && u1 == o.u1 && block_left_to_right == o.block_left_to_right &&
				block_right_to_left == o.block_right_to_left;
	}
};

// How a junction decides who goes first. Roundabouts are a node shape of
// their own (RoadNode::roundabout) and ignore this.
enum class JunctionControl : uint8_t {
	RightHand = 0, // yield to the right (European default for unmarked junctions)
	PriorityRoad = 1, // the legs listed in RoadNode::priority have right of way
	AllWayStop = 2, // everyone stops; first come, first served
	Signal = 3, // fixed-time traffic lights (RoadNode::signal)
};

const char *junction_control_name(JunctionControl c);
bool junction_control_from_name(const std::string &s, JunctionControl &out);

// Share of a spawner's trips that go to one destination (origin-destination
// matrix row). Destinations without an entry get weight 1, so the default is
// uniform.
struct OdWeight {
	NodeId to = kNoId;
	double weight = 1.0;

	bool operator==(const OdWeight &o) const { return to == o.to && weight == o.weight; }
};

// An intercity coach line (M3): coaches enter at the spawn point that holds
// the line, dwell at the map's main station and leave at `exit`.
struct CoachLine {
	uint32_t id = 0;
	NodeId exit = kNoId;
	double per_hour = 2.0;
	double dwell = 600.0; // s at the main station

	bool operator==(const CoachLine &o) const {
		return id == o.id && exit == o.exit && per_hour == o.per_hour && dwell == o.dwell;
	}
};

// Spawn / sink point for traffic from outside the map, on a road end.
struct Spawner {
	bool enabled = false;
	double rate = 300.0; // cars per hour entering here (0 = sink only)
	bool sink = true; // vehicles may leave the map here
	std::vector<OdWeight> od;
	double bikes = 0.0; // bicycles per hour entering here (M3)
	std::vector<CoachLine> coaches; // coach lines entering here (M3)

	bool operator==(const Spawner &o) const {
		return enabled == o.enabled && rate == o.rate && sink == o.sink && od == o.od && bikes == o.bikes &&
				coaches == o.coaches;
	}
	double weight_to(NodeId to) const;
};

// A roundabout in place of a junction (M3). Legs are the roads that end at
// the node. Traffic circulates counter-clockwise (right-hand traffic).
struct Roundabout {
	bool enabled = false;
	double radius = 20.0; // outer edge of the circulating carriageway (m)
	int lanes = 1; // circulating lanes, 1..3
	bool turbo = false; // raised lane dividers: lane chosen at entry, no changes inside
	std::vector<SegmentId> slip; // legs with a right-turn bypass to the next exit

	bool operator==(const Roundabout &o) const {
		return enabled == o.enabled && radius == o.radius && lanes == o.lanes && turbo == o.turbo && slip == o.slip;
	}
};

// Signals (M3): a movement is the traffic from one leg into another.
struct SignalMovement {
	SegmentId from = kNoId;
	SegmentId to = kNoId;
	bool permissive = false; // green, but yields to conflicting green traffic

	bool operator==(const SignalMovement &o) const {
		return from == o.from && to == o.to && permissive == o.permissive;
	}
};

struct SignalPhase {
	double green = 20.0; // s
	std::vector<SignalMovement> moves;

	bool operator==(const SignalPhase &o) const { return green == o.green && moves == o.moves; }
};

// Fixed-time plan: each phase is green, then amber, then all-red.
struct SignalPlan {
	std::vector<SignalPhase> phases;
	double amber = 3.0;
	double all_red = 2.0;
	double offset = 0.0; // s into the cycle at time zero
	// Legs whose right turns have the EU flashing green arrow: right on red
	// after stopping and yielding to everyone.
	std::vector<SegmentId> right_on_red;

	bool operator==(const SignalPlan &o) const {
		return phases == o.phases && amber == o.amber && all_red == o.all_red && offset == o.offset &&
				right_on_red == o.right_on_red;
	}
	double cycle() const;
};

// A city bus route (M3), run from a depot.
struct BusRoute {
	uint32_t id = 0;
	std::string name;
	uint32_t color = 0x2f7fd8; // 0xRRGGBB
	std::vector<uint32_t> stops; // stop ids in order
	double headway = 600.0; // s between departures
	bool loop = false; // loop (back to the first stop) or end to end

	bool operator==(const BusRoute &o) const {
		return id == o.id && name == o.name && color == o.color && stops == o.stops && headway == o.headway &&
				loop == o.loop;
	}
};

// A bus depot on a road end (M3): buses leave and return through it.
struct Depot {
	bool enabled = false;
	int capacity = 20;
	std::string name;
	std::vector<BusRoute> routes;

	bool operator==(const Depot &o) const {
		return enabled == o.enabled && capacity == o.capacity && name == o.name && routes == o.routes;
	}
};

struct RoadNode {
	NodeId id = kNoId;
	Vec2 pos;
	int level = 0;
	JunctionControl control = JunctionControl::RightHand;
	// Segments whose legs form the main road (PriorityRoad only).
	std::vector<SegmentId> priority;
	Spawner spawner;
	Roundabout roundabout;
	SignalPlan signal;
	Depot depot;

	bool operator==(const RoadNode &o) const {
		return id == o.id && pos == o.pos && level == o.level && control == o.control && priority == o.priority &&
				spawner == o.spawner && roundabout == o.roundabout && signal == o.signal && depot == o.depot;
	}
	bool is_priority(SegmentId s) const;
	// Replaces or (with to == kNoId) drops every reference to a segment.
	void rename_segment(SegmentId from, SegmentId to);
};

// Bus stops (M3) sit on a road, on the kerb of one travel direction.
enum class StopKind : uint8_t {
	Kerbside = 0, // the bus stops in its lane
	Bay = 1, // the bus pulls into a lay-by
	MainStation = 2, // the coach terminal, several bays, one per map
};
const char *stop_kind_name(StopKind k);
bool stop_kind_from_name(const std::string &s, StopKind &out);

struct BusStop {
	uint32_t id = 0;
	double u = 0.5; // fraction of the centreline length
	LaneDir side = LaneDir::Forward; // the direction of travel it serves
	StopKind kind = StopKind::Kerbside;
	std::string name;
	int bays = 1; // buses at once (main station: several)

	bool operator==(const BusStop &o) const {
		return id == o.id && u == o.u && side == o.side && kind == o.kind && name == o.name && bays == o.bays;
	}
};

struct RoadSegment {
	SegmentId id = kNoId;
	NodeId from = kNoId;
	NodeId to = kNoId;
	SegmentKind kind = SegmentKind::Road;
	int level = 0;
	// Curve shape. p0/p3 mirror the node positions and are refreshed whenever
	// a node moves; for arcs the centre is recomputed to keep the sweep.
	CurveKind curve = CurveKind::Straight;
	Vec2 c1; // Bezier
	Vec2 c2; // Bezier
	double sweep = 0.0; // Arc (centre derived from end points and sweep)
	Profile profile;
	double speed_limit = 50.0 / 3.6; // m/s
	std::string name;
	EndRules ends[2]; // [0] lanes arriving at the from-node, [1] at the to-node
	std::vector<NoChangeZone> no_change;
	std::vector<BusStop> stops;

	bool operator==(const RoadSegment &o) const;
};

class RoadMap {
public:
	const std::map<NodeId, RoadNode> &nodes() const { return nodes_; }
	const std::map<SegmentId, RoadSegment> &segments() const { return segments_; }
	const RoadNode *node(NodeId id) const;
	const RoadSegment *segment(SegmentId id) const;

	// Segments that start or end at a node, in ascending ID order.
	std::vector<SegmentId> segments_at(NodeId id) const;

	// The curve of a segment with its end points taken from the nodes.
	Curve curve_of(const RoadSegment &s) const;

	uint32_t next_node_id() const { return next_node_id_; }
	uint32_t next_segment_id() const { return next_segment_id_; }
	uint32_t next_lane_id() const { return next_lane_id_; }
	uint32_t next_object_id() const { return next_object_id_; }

	// ID allocation (IDs are never reused, even after undo).
	NodeId alloc_node_id() { return next_node_id_++; }
	SegmentId alloc_segment_id() { return next_segment_id_++; }
	LaneId alloc_lane_id() { return next_lane_id_++; }
	// Stops, bus routes and coach lines.
	uint32_t alloc_object_id() { return next_object_id_++; }
	void set_next_ids(uint32_t n, uint32_t s, uint32_t l, uint32_t o = 1);

	// Raw writes. Use Document for undoable edits.
	void put_node(const RoadNode &n);
	void put_segment(const RoadSegment &s);
	void erase_node(NodeId id);
	void erase_segment(SegmentId id);
	void clear();

	bool operator==(const RoadMap &o) const {
		return nodes_ == o.nodes_ && segments_ == o.segments_;
	}

private:
	void index_add(const RoadSegment &s);
	void index_remove(const RoadSegment &s);

	std::map<NodeId, RoadNode> nodes_;
	std::map<SegmentId, RoadSegment> segments_;
	std::map<NodeId, std::vector<SegmentId>> adjacency_;
	uint32_t next_node_id_ = 1;
	uint32_t next_segment_id_ = 1;
	uint32_t next_lane_id_ = 1;
	uint32_t next_object_id_ = 1;
};

// --- Profiles ---------------------------------------------------------------

// Structured description used by presets and the inspector.
struct ProfileParams {
	int backward = 1; // travel lanes against the from->to direction
	int forward = 1; // travel lanes along from->to
	double lane_width = 3.25;
	MedianType median = MedianType::None;
	double median_width = 0.0;
	bool sidewalk_left = false;
	bool sidewalk_right = false;
	double sidewalk_width = 2.0;
	bool parking_left = false;
	bool parking_right = false;
	bool bike_left = false;
	bool bike_right = false;
	bool bus_left = false; // outermost backward lane is a bus lane
	bool bus_right = false; // outermost forward lane is a bus lane
	ParkingStyle parking_style = ParkingStyle::Parallel;
};

// Builds a profile from params. Lanes are matched to `previous` by role
// (sidewalk, parking, bike, n-th travel lane from the median) so IDs, custom
// types and widths survive edits. New lanes get IDs from `map`.
Profile build_profile(const ProfileParams &p, const Profile *previous, RoadMap &map);
// Best-effort inverse, for showing a profile in the inspector.
ProfileParams params_of(const Profile &p);
// Checks lane order and widths. Returns an empty string when valid.
std::string validate_profile(const Profile &p);

// Role of each lane: (kind, side, index from the median). kind: 0 sidewalk,
// 1 parking, 2 bike, 3 motor; side: 0 left of the median, 1 right. Used to
// match lanes when a profile is rebuilt or where two roads join.
struct LaneRole {
	int kind = 0;
	int side = 0;
	int index = 0;
	bool operator<(const LaneRole &o) const {
		return kind != o.kind ? kind < o.kind : side != o.side ? side < o.side : index < o.index;
	}
	bool operator==(const LaneRole &o) const { return kind == o.kind && side == o.side && index == o.index; }
};
std::vector<LaneRole> profile_roles(const Profile &p);
bool is_directional(LaneType t);

struct ProfilePreset {
	const char *name;
	ProfileParams params;
};
const std::vector<ProfilePreset> &profile_presets();

} // namespace tsim
