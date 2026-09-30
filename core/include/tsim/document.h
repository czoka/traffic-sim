// Undoable editing on top of RoadMap (the command log).
//
// Every edit runs inside a transaction. The first time a transaction touches a
// node or segment, its previous value is saved; commit stores the new values.
// Undo and redo swap those copies back in. Derived data is never stored, so
// restoring nodes and segments restores everything.
#pragma once

#include "tsim/road_map.h"

#include <optional>
#include <set>
#include <string>
#include <vector>

namespace tsim {

// Where a click landed after snapping.
struct PointRef {
	Vec2 pos;
	NodeId node = kNoId; // snapped to an existing node
	SegmentId segment = kNoId; // snapped onto a segment (split there)
};

// A starting signal plan for a junction: opposite legs share a phase (left
// turns permissive), other legs get a phase each.
SignalPlan default_signal_plan(const RoadMap &map, NodeId node);

class Document {
public:
	Document();

	const RoadMap &map() const { return map_; }
	// Replaces the map and clears the history.
	void reset(const RoadMap &map);

	// --- Transactions ------------------------------------------------------
	// Transactions nest; only the outermost commit creates an undo step.
	void begin(const std::string &label);
	// Returns true when an undo step was recorded.
	bool commit();
	// Rolls back everything since the outermost begin().
	void cancel();
	bool in_transaction() const { return depth_ > 0; }

	bool can_undo() const { return !undo_.empty(); }
	bool can_redo() const { return !redo_.empty(); }
	bool undo();
	bool redo();
	std::string undo_label() const { return undo_.empty() ? std::string() : undo_.back().label; }
	std::string redo_label() const { return redo_.empty() ? std::string() : redo_.back().label; }
	size_t undo_count() const { return undo_.size(); }
	size_t redo_count() const { return redo_.size(); }

	// Increments on every change to the map (including undo/redo/cancel).
	uint64_t revision() const { return revision_; }
	// IDs changed since the last call (for incremental geometry rebuilds).
	// Returns true when everything must be rebuilt (e.g. after reset()).
	bool take_dirty(std::set<NodeId> &nodes, std::set<SegmentId> &segments);

	// --- Editing operations (each is its own transaction if none is open) --
	// Resolves a snapped point to a node: an existing node, a split of a
	// segment at the closest point, or a new node.
	NodeId resolve_point(const PointRef &p, int level);
	// Straight road through the points. Returns the new segment IDs.
	std::vector<SegmentId> add_road(const std::vector<PointRef> &points, const Profile &proto, int level,
			double speed_limit);
	// Bezier road from a to b shaped by a quadratic control point.
	SegmentId add_curve(const PointRef &a, Vec2 control, const PointRef &b, const Profile &proto, int level,
			double speed_limit);
	// Arc road (used by generators and v1 imports).
	SegmentId add_arc(NodeId a, NodeId b, double sweep, const Profile &proto, double speed_limit);
	NodeId add_node(Vec2 pos, int level);
	// Splits at u (fraction of the centreline length). Returns the new node.
	NodeId split_segment(SegmentId id, double u);

	void move_node(NodeId id, Vec2 pos);
	// Moves every segment of `from` onto `into` and deletes `from`.
	bool merge_nodes(NodeId from, NodeId into);
	void set_control_point(SegmentId id, int which, Vec2 pos);
	// Converts a straight or arc segment to a Bézier (for dragging handles).
	void make_bezier(SegmentId id);

	void delete_segment(SegmentId id);
	void delete_node(NodeId id);

	// Returns an error message, or an empty string on success.
	std::string set_profile(SegmentId id, const Profile &p);
	std::string set_profile_params(SegmentId id, const ProfileParams &p);
	std::string set_lane_type(SegmentId id, LaneId lane, LaneType type);
	// Flips a one-way road's direction. False for two-way roads.
	bool flip(SegmentId id);
	void set_end_rules(SegmentId id, int end, const EndRules &rules);
	void set_speed_limit(SegmentId id, double mps);
	void set_name(SegmentId id, const std::string &name);
	void set_level(SegmentId id, int level);
	// Paints (or with both flags false, clears) a no-change zone.
	void set_no_change(SegmentId id, int edge, double u0, double u1, bool block_l2r, bool block_r2l);

	// --- Junctions and demand (M2) ------------------------------------------
	// Priority segments not at this node are dropped.
	void set_junction_control(NodeId id, JunctionControl control, const std::vector<SegmentId> &priority);
	// Replaces the node's spawn / sink point (enabled = false removes it).
	// Coach lines without an id get one.
	void set_spawner(NodeId id, const Spawner &spawner);

	// --- M3: roundabouts, signals, transit ------------------------------------
	void set_roundabout(NodeId id, const Roundabout &r);
	// Setting a plan also switches the node to JunctionControl::Signal.
	void set_signal_plan(NodeId id, const SignalPlan &plan);
	// Returns the new stop's id (0 if the road doesn't exist).
	uint32_t add_stop(SegmentId seg, double u, LaneDir side, StopKind kind, const std::string &name);
	void set_stop(SegmentId seg, const BusStop &stop);
	void remove_stop(SegmentId seg, uint32_t stop);
	// Replaces the node's depot (enabled = false removes it). Routes without
	// an id get one.
	void set_depot(NodeId id, const Depot &depot);

	// Fresh lane IDs for a profile template.
	Profile instantiate(const Profile &proto);

private:
	struct NodeChange {
		NodeId id;
		std::optional<RoadNode> before;
		std::optional<RoadNode> after;
	};
	struct SegmentChange {
		SegmentId id;
		std::optional<RoadSegment> before;
		std::optional<RoadSegment> after;
	};
	struct Change {
		std::string label;
		std::vector<NodeChange> nodes;
		std::vector<SegmentChange> segments;
	};

	void touch_node(NodeId id);
	void touch_segment(SegmentId id);
	void put_node(const RoadNode &n);
	void put_segment(const RoadSegment &s);
	void erase_node(NodeId id);
	void erase_segment(SegmentId id);
	void remove_if_isolated(NodeId id);
	void apply(const Change &c, bool forward);

	// RAII helper: opens a transaction for the duration of an operation.
	struct Scope {
		Document &doc;
		Scope(Document &d, const char *label) : doc(d) { doc.begin(label); }
		~Scope() { doc.commit(); }
	};

	RoadMap map_;
	int depth_ = 0;
	Change open_;
	std::set<NodeId> open_nodes_;
	std::set<SegmentId> open_segments_;
	std::vector<Change> undo_;
	std::vector<Change> redo_;
	uint64_t revision_ = 0;
	std::set<NodeId> dirty_nodes_;
	std::set<SegmentId> dirty_segments_;
	bool dirty_all_ = true;
};

} // namespace tsim
