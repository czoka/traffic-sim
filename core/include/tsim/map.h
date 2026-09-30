// Editable road map: nodes, segments (straight or arc) and lanes.
//
// Every object has a stable ID that is never reused, so save files, undo logs
// and the simulation can refer to things by ID across edits.
#pragma once

#include "tsim/types.h"

#include <string>
#include <vector>

namespace tsim {

enum class SegmentKind : uint8_t {
	Straight = 0,
	Arc = 1,
};

struct Node {
	NodeId id = kNoId;
	Vec2 pos;
};

struct Segment {
	SegmentId id = kNoId;
	NodeId from = kNoId;
	NodeId to = kNoId;
	SegmentKind kind = SegmentKind::Straight;
	// Arc only: centre of the circle and signed sweep in radians. A positive
	// sweep runs clockwise on screen (y-down), negative runs counter-clockwise.
	Vec2 center;
	double sweep = 0.0;
	double lane_width = 3.5;
	double speed_limit = 13.8889; // m/s (50 km/h)
	// All lanes run from -> to in the POC (one-way roads). Index 0 is the
	// rightmost lane in the direction of travel.
	std::vector<LaneId> lanes;
};

struct Lane {
	LaneId id = kNoId;
	SegmentId segment = kNoId;
	uint32_t index = 0;
	// Lateral offset from the segment centreline, positive = right of travel.
	double offset = 0.0;
	// Derived from geometry; recomputed on load. Uses only sqrt and products.
	double length = 0.0;
	// Successor lanes (connectors). Vehicles follow next[0] in the POC.
	std::vector<LaneId> next;
};

class Map {
public:
	NodeId add_node(Vec2 pos);
	// Returns kNoId if the nodes do not exist or the geometry is degenerate.
	SegmentId add_straight(NodeId from, NodeId to, int lane_count, double lane_width, double speed_limit);
	SegmentId add_arc(NodeId from, NodeId to, Vec2 center, double sweep, int lane_count, double lane_width,
			double speed_limit);
	// Adds a connector from one lane to another. Returns false for unknown IDs.
	bool connect(LaneId from, LaneId to);

	const Node *node(NodeId id) const;
	const Segment *segment(SegmentId id) const;
	const Lane *lane(LaneId id) const;

	// Objects in ascending ID order (IDs are allocated increasingly).
	const std::vector<Node> &nodes() const { return nodes_; }
	const std::vector<Segment> &segments() const { return segments_; }
	const std::vector<Lane> &lanes() const { return lanes_; }

	// Rendering helpers (may use trig; never called from the tick).
	Pose lane_pose(const Lane &lane, double s) const;
	void lane_polyline(const Lane &lane, double max_step, std::vector<Vec2> &out) const;
	// Polyline of the segment at a lateral offset (positive = right of travel).
	void offset_polyline(const Segment &seg, double offset, double max_step, std::vector<Vec2> &out) const;

	// Next IDs, saved with the map so IDs are never reused after a reload.
	uint32_t next_node_id() const { return next_node_id_; }
	uint32_t next_segment_id() const { return next_segment_id_; }
	uint32_t next_lane_id() const { return next_lane_id_; }

	void clear();

	// Used by the JSON loader, which restores objects with their saved IDs.
	friend class MapBuilder;

private:
	static double lane_offset(int index, int lane_count, double lane_width);
	bool compute_lane_length(const Segment &seg, Lane &lane) const;
	SegmentId add_segment(Segment seg, int lane_count);

	std::vector<Node> nodes_;
	std::vector<Segment> segments_;
	std::vector<Lane> lanes_;
	uint32_t next_node_id_ = 1;
	uint32_t next_segment_id_ = 1;
	uint32_t next_lane_id_ = 1;
};

// Low-level builder that inserts objects with explicit IDs (used by load).
// Objects must be inserted in ascending ID order.
class MapBuilder {
public:
	explicit MapBuilder(Map &map) : map_(map) {}
	bool add_node(const Node &n, std::string *err);
	bool add_segment(const Segment &s, std::string *err);
	bool add_lane(const Lane &l, std::string *err);
	// Validates references, computes lane geometry and sets the next IDs.
	bool finish(uint32_t next_node, uint32_t next_segment, uint32_t next_lane, std::string *err);

private:
	Map &map_;
};

// Builds a one-way ring road of four quarter arcs (clockwise on screen) around
// the origin. Uses only exact arithmetic so it is identical on every platform.
struct RingInfo {
	std::vector<SegmentId> segments;
};
RingInfo build_ring(Map &map, double radius, int lane_count, double lane_width, double speed_limit);

} // namespace tsim
