// Derived road geometry: lane shapes, junctions, connectors, markings, meshes.
//
// Built entirely from a RoadMap, so it can be thrown away and rebuilt after any
// edit (including undo). Nothing here is saved.
#pragma once

#include "tsim/road_map.h"

#include <map>
#include <vector>

namespace tsim {

struct Color {
	float r = 1, g = 1, b = 1, a = 1;
};

// Draw order within a level: ground (sidewalks, islands), asphalt, lane tints,
// paint.
enum class Layer : uint8_t {
	Ground = 0,
	Asphalt = 1,
	Tint = 2,
	Markings = 3,
	Count = 4,
};

struct MeshBatch {
	int level = 0;
	Layer layer = Layer::Asphalt;
	std::vector<Vec2> vertices;
	std::vector<Color> colors;
	// For painted lines: the vertex's outward normal times the line's half
	// width (zero for filled shapes). Lets a shader keep thin lines at least a
	// pixel wide when zoomed out.
	std::vector<Vec2> uvs;
	std::vector<int32_t> indices;
};

enum class NodeKind : uint8_t {
	Isolated = 0,
	End = 1, // one road ends here
	Continuation = 2, // two roads join end to end, same lane counts
	Taper = 3, // two roads join end to end and lanes are added or dropped
	Junction = 4, // three or more legs, or a sharp corner
	Roundabout = 5, // RoadNode::roundabout (M3)
};

enum class TurnKind : uint8_t {
	Straight = 0,
	Left = 1,
	Right = 2,
	UTurn = 3,
};

const char *node_kind_name(NodeKind k);
const char *turn_kind_name(TurnKind k);

// A lane as drawn: a profile lane or a generated turn pocket.
struct GeomLane {
	LaneId id = kNoId;
	LaneType type = LaneType::General;
	LaneDir dir = LaneDir::Forward;
	double base_width = 3.25;
	int pocket_end = -1; // 0 or 1 for pockets, -1 for profile lanes
	bool pocket_left = false;
	int profile_index = -1; // index in the segment profile, -1 for pockets
	ParkingStyle parking = ParkingStyle::Parallel;
};

// A parking bay (M3), derived from a parking lane. Stable key: (lane, index).
struct ParkingBay {
	LaneId lane = kNoId; // the parking lane
	int index = 0;
	double s = 0.0; // centreline station of the bay's centre
	Vec2 pos; // bay centre
	Vec2 dir{ 1, 0 }; // heading of a parked car
	ParkingStyle style = ParkingStyle::Parallel;
	bool list_right = true; // on the right of the profile list (next to forward lanes)
};

struct SegmentGeom {
	SegmentId id = kNoId;
	int level = 0;
	Curve curve;
	double length = 0.0;
	double trim[2] = { 0.0, 0.0 }; // cut back at the from / to end
	std::vector<GeomLane> lanes; // left -> right, pockets included
	// Samples over [trim[0], length - trim[1]].
	std::vector<double> s;
	std::vector<Vec2> p; // centreline points
	std::vector<Vec2> n; // right normals
	// Lateral edges, x[k * lanes + i] = {left, right} offset of lane i at sample k.
	std::vector<std::pair<double, double>> x;
	Vec2 bb_min, bb_max; // bounding box of the drawn shape
	std::vector<ParkingBay> bays;

	size_t lane_count() const { return lanes.size(); }
	const std::pair<double, double> &edge(size_t k, size_t lane) const { return x[k * lanes.size() + lane]; }
	Vec2 at(size_t k, double offset) const { return p[k] + n[k] * offset; }
	// Outline of the whole cross-section (for picking and overlap checks).
	std::vector<Vec2> outline() const;
	// Carriageway only (no sidewalks).
	std::vector<Vec2> carriageway() const;
};

struct Leg {
	SegmentId seg = kNoId;
	bool at_start = true; // the segment starts at this node
	Vec2 dir; // unit direction pointing away from the node
	double kerb_left = 0, kerb_right = 0; // carriageway half-widths, leg frame
	double outer_left = 0, outer_right = 0; // incl. sidewalks
	double trim = 0.0;
};

struct Connector {
	SegmentId from_seg = kNoId;
	LaneId from_lane = kNoId;
	SegmentId to_seg = kNoId;
	LaneId to_lane = kNoId;
	TurnKind turn = TurnKind::Straight;
	std::vector<Vec2> path;
	double route_penalty = 0.0; // s added by routing (roundabout lane choice)
};

// One circulating lane of a roundabout between two legs (M3). Its id is
// synthetic (see ring_lane_id) and stable while the leg exists.
struct RingLane {
	LaneId id = kNoId;
	int lane = 0; // 0 = outer
	int piece = 0; // index of the leg it starts after
	std::vector<Vec2> pts; // travel order (counter-clockwise on screen)
	double radius = 0.0;
};

// Synthetic lane ids for roundabout rings: high bit set, so they never clash
// with profile lane ids.
LaneId ring_lane_id(SegmentId leg, bool leg_at_start, int lane);
inline bool is_ring_lane(LaneId id) { return (id & 0x80000000u) != 0; }

struct NodeGeom {
	NodeId id = kNoId;
	int level = 0;
	Vec2 pos;
	NodeKind kind = NodeKind::Isolated;
	std::vector<Leg> legs; // sorted by angle
	std::vector<Vec2> polygon; // junction surface (empty unless a junction)
	std::vector<Connector> connectors;
	// Incoming lanes that have no way out (dead ends).
	std::vector<std::pair<SegmentId, LaneId>> dead_lanes;
	// Roundabouts (M3).
	std::vector<RingLane> ring;
	double ring_radius = 0.0;
	double island_radius = 0.0;
	bool ring_cramped = false; // too many legs for the radius
};

struct LaneHit {
	SegmentId seg = kNoId;
	int lane_index = -1; // in SegmentGeom::lanes
	LaneId lane = kNoId;
	double s = 0.0; // along the centreline
	double u = 0.0; // s / length
	double offset = 0.0; // lateral, positive = right
	int edge = -1; // nearest boundary between lanes (1..lanes-1), -1 if none
	double edge_distance = 1e300;
};

class RoadGeometry {
public:
	void build(const RoadMap &map);

	const std::map<SegmentId, SegmentGeom> &segments() const { return segments_; }
	const std::map<NodeId, NodeGeom> &nodes() const { return nodes_; }
	const SegmentGeom *segment(SegmentId id) const;
	const NodeGeom *node(NodeId id) const;
	const std::vector<MeshBatch> &meshes() const { return meshes_; }

	// Point inside a drawn lane (profile lanes only), optionally on one level.
	bool pick_lane(Vec2 p, int level, LaneHit &out) const;
	// Closest point on a segment's centreline within `radius` of the outline.
	bool pick_segment(Vec2 p, double radius, int level, LaneHit &out) const;

	// Tunables (metres).
	double curb_radius = 8.0;
	double taper_length = 30.0;
	double pocket_taper = 15.0;
	double solid_before_stop = 30.0;

private:
	struct SegCtx;
	void clear();

	std::map<SegmentId, SegmentGeom> segments_;
	std::map<NodeId, NodeGeom> nodes_;
	std::vector<MeshBatch> meshes_;
};

} // namespace tsim
