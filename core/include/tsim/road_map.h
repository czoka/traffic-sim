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

struct LaneSpec {
	LaneId id = kNoId;
	LaneType type = LaneType::General;
	LaneDir dir = LaneDir::Forward;
	double width = 3.25;

	bool operator==(const LaneSpec &o) const {
		return id == o.id && type == o.type && dir == o.dir && width == o.width;
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

// How a junction decides who goes first (M2). Signals and roundabouts are M3.
enum class JunctionControl : uint8_t {
	RightHand = 0, // yield to the right (European default for unmarked junctions)
	PriorityRoad = 1, // the legs listed in RoadNode::priority have right of way
	AllWayStop = 2, // everyone stops; first come, first served
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

// Spawn / sink point for traffic from outside the map, on a road end.
struct Spawner {
	bool enabled = false;
	double rate = 300.0; // vehicles per hour entering here (0 = sink only)
	bool sink = true; // vehicles may leave the map here
	std::vector<OdWeight> od;

	bool operator==(const Spawner &o) const {
		return enabled == o.enabled && rate == o.rate && sink == o.sink && od == o.od;
	}
	double weight_to(NodeId to) const;
};

struct RoadNode {
	NodeId id = kNoId;
	Vec2 pos;
	int level = 0;
	JunctionControl control = JunctionControl::RightHand;
	// Segments whose legs form the main road (PriorityRoad only).
	std::vector<SegmentId> priority;
	Spawner spawner;

	bool operator==(const RoadNode &o) const {
		return id == o.id && pos == o.pos && level == o.level && control == o.control && priority == o.priority &&
				spawner == o.spawner;
	}
	bool is_priority(SegmentId s) const;
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

	// ID allocation (IDs are never reused, even after undo).
	NodeId alloc_node_id() { return next_node_id_++; }
	SegmentId alloc_segment_id() { return next_segment_id_++; }
	LaneId alloc_lane_id() { return next_lane_id_++; }
	void set_next_ids(uint32_t n, uint32_t s, uint32_t l);

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
