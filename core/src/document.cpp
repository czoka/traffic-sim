#include "tsim/document.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace tsim {

namespace {

EndRules default_end() { return EndRules{}; }

// Maps no-change zones onto a sub-range [a, b] of the segment (as fractions).
std::vector<NoChangeZone> clip_zones(const std::vector<NoChangeZone> &zones, double a, double b) {
	std::vector<NoChangeZone> out;
	const double span = b - a;
	if (span <= 0.0) {
		return out;
	}
	for (const NoChangeZone &z : zones) {
		const double lo = std::max(z.u0, a);
		const double hi = std::min(z.u1, b);
		if (hi - lo > 1e-9) {
			NoChangeZone c = z;
			c.u0 = (lo - a) / span;
			c.u1 = (hi - a) / span;
			out.push_back(c);
		}
	}
	return out;
}

} // namespace

Document::Document() = default;

void Document::reset(const RoadMap &map) {
	map_ = map;
	undo_.clear();
	redo_.clear();
	depth_ = 0;
	open_ = Change{};
	open_nodes_.clear();
	open_segments_.clear();
	++revision_;
	dirty_all_ = true;
	dirty_nodes_.clear();
	dirty_segments_.clear();
}

// --- Transactions ------------------------------------------------------------

void Document::begin(const std::string &label) {
	if (depth_ == 0) {
		open_ = Change{};
		open_.label = label;
		open_nodes_.clear();
		open_segments_.clear();
	}
	++depth_;
}

bool Document::commit() {
	if (depth_ == 0) {
		return false;
	}
	if (--depth_ > 0) {
		return false;
	}
	Change c;
	c.label = open_.label;
	for (NodeChange &n : open_.nodes) {
		const RoadNode *now = map_.node(n.id);
		n.after = now ? std::optional<RoadNode>(*now) : std::nullopt;
		if (!(n.before == n.after)) {
			c.nodes.push_back(n);
		}
	}
	for (SegmentChange &s : open_.segments) {
		const RoadSegment *now = map_.segment(s.id);
		s.after = now ? std::optional<RoadSegment>(*now) : std::nullopt;
		if (!(s.before == s.after)) {
			c.segments.push_back(s);
		}
	}
	open_ = Change{};
	open_nodes_.clear();
	open_segments_.clear();
	if (c.nodes.empty() && c.segments.empty()) {
		return false;
	}
	undo_.push_back(std::move(c));
	redo_.clear();
	return true;
}

void Document::cancel() {
	if (depth_ == 0) {
		return;
	}
	Change c = open_;
	depth_ = 0;
	open_ = Change{};
	open_nodes_.clear();
	open_segments_.clear();
	apply(c, false);
}

void Document::apply(const Change &c, bool forward) {
	for (const SegmentChange &s : c.segments) {
		const std::optional<RoadSegment> &v = forward ? s.after : s.before;
		if (v) {
			map_.put_segment(*v);
		} else {
			map_.erase_segment(s.id);
		}
		dirty_segments_.insert(s.id);
	}
	for (const NodeChange &n : c.nodes) {
		const std::optional<RoadNode> &v = forward ? n.after : n.before;
		if (v) {
			map_.put_node(*v);
		} else {
			map_.erase_node(n.id);
		}
		dirty_nodes_.insert(n.id);
	}
	++revision_;
}

bool Document::undo() {
	if (depth_ > 0 || undo_.empty()) {
		return false;
	}
	Change c = std::move(undo_.back());
	undo_.pop_back();
	apply(c, false);
	redo_.push_back(std::move(c));
	return true;
}

bool Document::redo() {
	if (depth_ > 0 || redo_.empty()) {
		return false;
	}
	Change c = std::move(redo_.back());
	redo_.pop_back();
	apply(c, true);
	undo_.push_back(std::move(c));
	return true;
}

bool Document::take_dirty(std::set<NodeId> &nodes, std::set<SegmentId> &segments) {
	const bool all = dirty_all_;
	nodes.swap(dirty_nodes_);
	segments.swap(dirty_segments_);
	dirty_nodes_.clear();
	dirty_segments_.clear();
	dirty_all_ = false;
	return all;
}

void Document::touch_node(NodeId id) {
	if (depth_ == 0 || !open_nodes_.insert(id).second) {
		return;
	}
	const RoadNode *n = map_.node(id);
	open_.nodes.push_back(NodeChange{ id, n ? std::optional<RoadNode>(*n) : std::nullopt, std::nullopt });
}

void Document::touch_segment(SegmentId id) {
	if (depth_ == 0 || !open_segments_.insert(id).second) {
		return;
	}
	const RoadSegment *s = map_.segment(id);
	open_.segments.push_back(
			SegmentChange{ id, s ? std::optional<RoadSegment>(*s) : std::nullopt, std::nullopt });
}

void Document::put_node(const RoadNode &n) {
	touch_node(n.id);
	map_.put_node(n);
	dirty_nodes_.insert(n.id);
	++revision_;
}

void Document::put_segment(const RoadSegment &s) {
	touch_segment(s.id);
	map_.put_segment(s);
	dirty_segments_.insert(s.id);
	++revision_;
}

void Document::erase_node(NodeId id) {
	touch_node(id);
	map_.erase_node(id);
	dirty_nodes_.insert(id);
	++revision_;
}

void Document::erase_segment(SegmentId id) {
	touch_segment(id);
	map_.erase_segment(id);
	dirty_segments_.insert(id);
	++revision_;
}

void Document::remove_if_isolated(NodeId id) {
	if (map_.node(id) && map_.segments_at(id).empty()) {
		erase_node(id);
	}
}

// --- Editing operations ---------------------------------------------------------

Profile Document::instantiate(const Profile &proto) {
	Profile p = proto;
	for (LaneSpec &l : p.lanes) {
		l.id = map_.alloc_lane_id();
	}
	return p;
}

NodeId Document::add_node(Vec2 pos, int level) {
	Scope scope(*this, "Add node");
	RoadNode n;
	n.id = map_.alloc_node_id();
	n.pos = pos;
	n.level = level;
	put_node(n);
	return n.id;
}

NodeId Document::resolve_point(const PointRef &p, int level) {
	if (p.node != kNoId) {
		const RoadNode *n = map_.node(p.node);
		if (n && n->level == level) {
			return n->id;
		}
	}
	if (p.segment != kNoId) {
		const RoadSegment *s = map_.segment(p.segment);
		if (s && s->level == level) {
			const Curve c = map_.curve_of(*s);
			const double t = closest_t(c, p.pos);
			ArcTable table;
			table.build(c);
			const double len = table.length();
			const double s_along = table.s_at(t);
			// Snap to an end node instead of creating a sliver segment.
			if (s_along < 1.0) {
				return s->from;
			}
			if (len - s_along < 1.0) {
				return s->to;
			}
			return split_segment(p.segment, len > 0.0 ? s_along / len : 0.5);
		}
	}
	return add_node(p.pos, level);
}

std::vector<SegmentId> Document::add_road(const std::vector<PointRef> &points, const Profile &proto, int level,
		double speed_limit) {
	Scope scope(*this, "Add road");
	std::vector<SegmentId> out;
	if (points.size() < 2 || !validate_profile(proto).empty()) {
		return out;
	}
	std::vector<NodeId> nodes;
	for (const PointRef &p : points) {
		PointRef ref = p;
		// A segment snapped earlier in this road may have been split already;
		// re-snap onto whichever piece is closest.
		if (ref.segment != kNoId && !map_.segment(ref.segment)) {
			ref.segment = kNoId;
		}
		if (ref.segment != kNoId) {
			const RoadSegment *s = map_.segment(ref.segment);
			double best = 1e300;
			SegmentId best_id = ref.segment;
			for (NodeId n : { s->from, s->to }) {
				for (SegmentId cand : map_.segments_at(n)) {
					double d = 0.0;
					closest_t(map_.curve_of(*map_.segment(cand)), ref.pos, &d);
					if (d < best) {
						best = d;
						best_id = cand;
					}
				}
			}
			ref.segment = best_id;
		}
		const NodeId n = resolve_point(ref, level);
		if (nodes.empty() || nodes.back() != n) {
			nodes.push_back(n);
		}
	}
	for (size_t i = 0; i + 1 < nodes.size(); ++i) {
		const RoadNode *a = map_.node(nodes[i]);
		const RoadNode *b = map_.node(nodes[i + 1]);
		if (!a || !b || (b->pos - a->pos).length() < 0.5) {
			continue;
		}
		RoadSegment s;
		s.id = map_.alloc_segment_id();
		s.from = nodes[i];
		s.to = nodes[i + 1];
		s.level = level;
		s.curve = CurveKind::Straight;
		s.profile = instantiate(proto);
		s.speed_limit = speed_limit;
		put_segment(s);
		out.push_back(s.id);
	}
	for (NodeId n : nodes) {
		remove_if_isolated(n);
	}
	return out;
}

SegmentId Document::add_curve(const PointRef &a, Vec2 control, const PointRef &b, const Profile &proto, int level,
		double speed_limit) {
	Scope scope(*this, "Add curve");
	if (!validate_profile(proto).empty()) {
		return kNoId;
	}
	const NodeId na = resolve_point(a, level);
	PointRef bref = b;
	if (bref.segment != kNoId && !map_.segment(bref.segment)) {
		bref.segment = kNoId;
	}
	const NodeId nb = resolve_point(bref, level);
	const RoadNode *pa = map_.node(na);
	const RoadNode *pb = map_.node(nb);
	if (na == nb || !pa || !pb || (pb->pos - pa->pos).length() < 0.5) {
		remove_if_isolated(na);
		remove_if_isolated(nb);
		return kNoId;
	}
	RoadSegment s;
	s.id = map_.alloc_segment_id();
	s.from = na;
	s.to = nb;
	s.level = level;
	s.curve = CurveKind::Bezier;
	quadratic_to_cubic(pa->pos, control, pb->pos, s.c1, s.c2);
	s.profile = instantiate(proto);
	s.speed_limit = speed_limit;
	put_segment(s);
	return s.id;
}

SegmentId Document::add_arc(NodeId a, NodeId b, double sweep, const Profile &proto, double speed_limit) {
	Scope scope(*this, "Add arc");
	const RoadNode *pa = map_.node(a);
	const RoadNode *pb = map_.node(b);
	if (!pa || !pb || a == b || sweep == 0.0 || std::fabs(sweep) >= 2.0 * kPi) {
		return kNoId;
	}
	RoadSegment s;
	s.id = map_.alloc_segment_id();
	s.from = a;
	s.to = b;
	s.level = pa->level;
	s.curve = CurveKind::Arc;
	s.sweep = sweep;
	s.profile = instantiate(proto);
	s.speed_limit = speed_limit;
	put_segment(s);
	return s.id;
}

NodeId Document::split_segment(SegmentId id, double u) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return kNoId;
	}
	Scope scope(*this, "Split road");
	const RoadSegment seg = *orig;
	const Curve c = map_.curve_of(seg);
	ArcTable table;
	table.build(c);
	u = std::clamp(u, 0.01, 0.99);
	const double t = table.t_at(u * table.length());
	Curve first, second;
	split_curve(c, t, first, second);

	RoadNode mid;
	mid.id = map_.alloc_node_id();
	mid.pos = first.p3;
	mid.level = seg.level;
	put_node(mid);

	RoadSegment a = seg;
	a.to = mid.id;
	a.c1 = first.c1;
	a.c2 = first.c2;
	a.sweep = first.sweep;
	a.ends[1] = default_end();
	a.no_change = clip_zones(seg.no_change, 0.0, u);

	RoadSegment b = seg;
	b.id = map_.alloc_segment_id();
	b.from = mid.id;
	b.c1 = second.c1;
	b.c2 = second.c2;
	b.sweep = second.sweep;
	b.profile = instantiate(seg.profile);
	b.ends[0] = default_end();
	b.no_change = clip_zones(seg.no_change, u, 1.0);
	// Bus stops go with the piece they are on.
	a.stops.clear();
	b.stops.clear();
	for (const BusStop &st : seg.stops) {
		BusStop c = st;
		if (st.u < u) {
			c.u = st.u / u;
			a.stops.push_back(c);
		} else {
			c.u = (st.u - u) / (1.0 - u);
			b.stops.push_back(c);
		}
	}

	put_segment(a);
	put_segment(b);
	// The far node's leg is now segment b.
	const RoadNode *far = map_.node(seg.to);
	if (far) {
		RoadNode n = *far;
		n.rename_segment(id, b.id);
		if (!(n == *far)) put_node(n);
	}
	return mid.id;
}

void Document::move_node(NodeId id, Vec2 pos) {
	const RoadNode *n = map_.node(id);
	if (!n) {
		return;
	}
	Scope scope(*this, "Move node");
	RoadNode moved = *n;
	const Vec2 delta = pos - moved.pos;
	moved.pos = pos;
	put_node(moved);
	for (SegmentId sid : map_.segments_at(id)) {
		RoadSegment s = *map_.segment(sid);
		if (s.curve == CurveKind::Bezier) {
			if (s.from == id) s.c1 = s.c1 + delta;
			if (s.to == id) s.c2 = s.c2 + delta;
		}
		put_segment(s); // also marks the segment dirty for geometry
	}
}

bool Document::merge_nodes(NodeId from, NodeId into) {
	const RoadNode *a = map_.node(from);
	const RoadNode *b = map_.node(into);
	if (!a || !b || from == into || a->level != b->level) {
		return false;
	}
	Scope scope(*this, "Join roads");
	const Vec2 delta = b->pos - a->pos;
	for (SegmentId sid : map_.segments_at(from)) {
		RoadSegment s = *map_.segment(sid);
		if (s.from == from) {
			s.from = into;
			if (s.curve == CurveKind::Bezier) s.c1 = s.c1 + delta;
		}
		if (s.to == from) {
			s.to = into;
			if (s.curve == CurveKind::Bezier) s.c2 = s.c2 + delta;
		}
		if (s.from == s.to) {
			erase_segment(sid);
		} else {
			put_segment(s);
		}
	}
	erase_node(from);
	remove_if_isolated(into);
	return true;
}

void Document::make_bezier(SegmentId id) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig || orig->curve == CurveKind::Bezier) {
		return;
	}
	Scope scope(*this, "Bend road");
	RoadSegment s = *orig;
	const Curve c = map_.curve_of(s);
	if (s.curve == CurveKind::Straight) {
		s.c1 = c.p0 + (c.p3 - c.p0) * (1.0 / 3.0);
		s.c2 = c.p0 + (c.p3 - c.p0) * (2.0 / 3.0);
	} else {
		// Cubic approximation of the arc: handles along the end tangents.
		const double k = (4.0 / 3.0) * std::tan(c.sweep / 4.0) * (c.p0 - c.center).length();
		s.c1 = c.p0 + c.tangent(0.0) * std::fabs(k);
		s.c2 = c.p3 - c.tangent(1.0) * std::fabs(k);
	}
	s.curve = CurveKind::Bezier;
	put_segment(s);
}

void Document::set_control_point(SegmentId id, int which, Vec2 pos) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return;
	}
	Scope scope(*this, "Shape curve");
	if (orig->curve != CurveKind::Bezier) {
		make_bezier(id);
	}
	RoadSegment s = *map_.segment(id);
	(which == 0 ? s.c1 : s.c2) = pos;
	put_segment(s);
}

void Document::delete_segment(SegmentId id) {
	const RoadSegment *s = map_.segment(id);
	if (!s) {
		return;
	}
	Scope scope(*this, "Delete road");
	const NodeId a = s->from;
	const NodeId b = s->to;
	erase_segment(id);
	for (NodeId end : { a, b }) {
		const RoadNode *n = map_.node(end);
		if (!n) continue;
		RoadNode c = *n;
		c.rename_segment(id, kNoId);
		if (!(c == *n)) put_node(c);
	}
	remove_if_isolated(a);
	remove_if_isolated(b);
}

void Document::delete_node(NodeId id) {
	if (!map_.node(id)) {
		return;
	}
	Scope scope(*this, "Delete node");
	for (SegmentId sid : map_.segments_at(id)) {
		delete_segment(sid);
	}
	if (map_.node(id)) {
		erase_node(id);
	}
}

std::string Document::set_profile(SegmentId id, const Profile &p) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return "no such road";
	}
	const std::string err = validate_profile(p);
	if (!err.empty()) {
		return err;
	}
	Scope scope(*this, "Change profile");
	RoadSegment s = *orig;
	s.profile = p;
	for (LaneSpec &l : s.profile.lanes) {
		if (l.id == kNoId) {
			l.id = map_.alloc_lane_id();
		}
	}
	const int edges = static_cast<int>(s.profile.lanes.size());
	s.no_change.erase(std::remove_if(s.no_change.begin(), s.no_change.end(),
							  [edges](const NoChangeZone &z) { return z.edge < 1 || z.edge >= edges; }),
			s.no_change.end());
	put_segment(s);
	return std::string();
}

std::string Document::set_profile_params(SegmentId id, const ProfileParams &p) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return "no such road";
	}
	Scope scope(*this, "Change lanes");
	const Profile prev = orig->profile;
	return set_profile(id, build_profile(p, &prev, map_));
}

std::string Document::set_lane_type(SegmentId id, LaneId lane, LaneType type) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return "no such road";
	}
	Profile p = orig->profile;
	auto it = std::find_if(p.lanes.begin(), p.lanes.end(), [lane](const LaneSpec &l) { return l.id == lane; });
	if (it == p.lanes.end()) {
		return "no such lane";
	}
	const bool directional = is_travel(type) || type == LaneType::Bike;
	if (!directional) {
		it->dir = LaneDir::None;
	} else if (it->dir == LaneDir::None) {
		// Take the direction of the nearest directional neighbour.
		const size_t i = static_cast<size_t>(it - p.lanes.begin());
		LaneDir d = LaneDir::Forward;
		for (size_t k = 1; k < p.lanes.size(); ++k) {
			if (i >= k && p.lanes[i - k].dir != LaneDir::None) {
				d = p.lanes[i - k].dir;
				break;
			}
			if (i + k < p.lanes.size() && p.lanes[i + k].dir != LaneDir::None) {
				d = p.lanes[i + k].dir;
				break;
			}
		}
		it->dir = d;
	}
	it->type = type;
	it->width = default_width(type);
	Scope scope(*this, "Change lane type");
	return set_profile(id, p);
}

bool Document::flip(SegmentId id) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig || !orig->profile.one_way()) {
		return false;
	}
	Scope scope(*this, "Flip direction");
	RoadSegment s = *orig;
	std::swap(s.from, s.to);
	std::swap(s.c1, s.c2);
	s.sweep = -s.sweep;
	std::reverse(s.profile.lanes.begin(), s.profile.lanes.end());
	std::swap(s.ends[0], s.ends[1]);
	const int n = static_cast<int>(s.profile.lanes.size());
	for (NoChangeZone &z : s.no_change) {
		z.edge = n - z.edge;
		const double u0 = 1.0 - z.u1;
		z.u1 = 1.0 - z.u0;
		z.u0 = u0;
		std::swap(z.block_left_to_right, z.block_right_to_left);
	}
	for (BusStop &st : s.stops) {
		st.u = 1.0 - st.u;
		st.side = st.side == LaneDir::Forward ? LaneDir::Backward : LaneDir::Forward;
	}
	put_segment(s);
	return true;
}

void Document::set_end_rules(SegmentId id, int end, const EndRules &rules) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig || end < 0 || end > 1) {
		return;
	}
	Scope scope(*this, "Change turn rules");
	RoadSegment s = *orig;
	EndRules r = rules;
	r.turn_lane_length = std::clamp(r.turn_lane_length, 10.0, 200.0);
	// Pocket lanes keep their IDs for good once allocated.
	r.left_lane = s.ends[end].left_lane;
	r.right_lane = s.ends[end].right_lane;
	if (r.left == TurnRule::TurnLane && r.left_lane == kNoId) r.left_lane = map_.alloc_lane_id();
	if (r.right == TurnRule::TurnLane && r.right_lane == kNoId) r.right_lane = map_.alloc_lane_id();
	s.ends[end] = r;
	put_segment(s);
}

void Document::set_speed_limit(SegmentId id, double mps) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return;
	}
	Scope scope(*this, "Change speed limit");
	RoadSegment s = *orig;
	s.speed_limit = std::clamp(mps, 1.0, 50.0);
	put_segment(s);
}

void Document::set_name(SegmentId id, const std::string &name) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return;
	}
	Scope scope(*this, "Rename road");
	RoadSegment s = *orig;
	s.name = name;
	put_segment(s);
}

void Document::set_level(SegmentId id, int level) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig || orig->level == level) {
		return;
	}
	Scope scope(*this, "Change level");
	RoadSegment s = *orig;
	s.level = level;
	for (NodeId *end : { &s.from, &s.to }) {
		const RoadNode n = *map_.node(*end);
		if (map_.segments_at(n.id).size() > 1) {
			// Shared with roads on the old level: this road gets its own node.
			RoadNode copy;
			copy.id = map_.alloc_node_id();
			copy.pos = n.pos;
			copy.level = level;
			put_node(copy);
			*end = copy.id;
		} else {
			RoadNode moved = n;
			moved.level = level;
			put_node(moved);
		}
	}
	put_segment(s);
}

void Document::set_no_change(SegmentId id, int edge, double u0, double u1, bool block_l2r, bool block_r2l) {
	const RoadSegment *orig = map_.segment(id);
	if (!orig) {
		return;
	}
	const int lanes = static_cast<int>(orig->profile.lanes.size());
	if (edge < 1 || edge >= lanes) {
		return;
	}
	if (u0 > u1) {
		std::swap(u0, u1);
	}
	u0 = std::clamp(u0, 0.0, 1.0);
	u1 = std::clamp(u1, 0.0, 1.0);
	if (u1 - u0 < 1e-6) {
		return;
	}
	Scope scope(*this, block_l2r || block_r2l ? "Paint no-change zone" : "Clear no-change zone");
	RoadSegment s = *orig;
	std::vector<NoChangeZone> out;
	for (const NoChangeZone &z : s.no_change) {
		if (z.edge != edge || z.u1 <= u0 || z.u0 >= u1) {
			out.push_back(z);
			continue;
		}
		if (z.u0 < u0) {
			NoChangeZone left = z;
			left.u1 = u0;
			out.push_back(left);
		}
		if (z.u1 > u1) {
			NoChangeZone right = z;
			right.u0 = u1;
			out.push_back(right);
		}
	}
	if (block_l2r || block_r2l) {
		out.push_back(NoChangeZone{ edge, u0, u1, block_l2r, block_r2l });
	}
	// Sort and merge touching zones with the same flags.
	std::sort(out.begin(), out.end(), [](const NoChangeZone &a, const NoChangeZone &b) {
		return a.edge != b.edge ? a.edge < b.edge : a.u0 < b.u0;
	});
	std::vector<NoChangeZone> merged;
	for (const NoChangeZone &z : out) {
		if (!merged.empty()) {
			NoChangeZone &m = merged.back();
			if (m.edge == z.edge && m.block_left_to_right == z.block_left_to_right &&
					m.block_right_to_left == z.block_right_to_left && z.u0 <= m.u1 + 1e-9) {
				m.u1 = std::max(m.u1, z.u1);
				continue;
			}
		}
		merged.push_back(z);
	}
	s.no_change = merged;
	put_segment(s);
}

void Document::set_junction_control(NodeId id, JunctionControl control, const std::vector<SegmentId> &priority) {
	const RoadNode *orig = map_.node(id);
	if (!orig) {
		return;
	}
	Scope scope(*this, "Change junction control");
	RoadNode n = *orig;
	n.control = control;
	if (control == JunctionControl::Signal && n.signal.phases.empty()) n.signal = default_signal_plan(map_, id);
	n.priority.clear();
	const std::vector<SegmentId> here = map_.segments_at(id);
	for (SegmentId s : priority) {
		if (std::find(here.begin(), here.end(), s) != here.end() && !n.is_priority(s)) {
			n.priority.push_back(s);
		}
	}
	std::sort(n.priority.begin(), n.priority.end());
	put_node(n);
}

void Document::set_spawner(NodeId id, const Spawner &spawner) {
	const RoadNode *orig = map_.node(id);
	if (!orig) {
		return;
	}
	Scope scope(*this, spawner.enabled ? "Change spawn point" : "Remove spawn point");
	RoadNode n = *orig;
	n.spawner = spawner.enabled ? spawner : Spawner{};
	n.spawner.rate = std::clamp(n.spawner.rate, 0.0, 5000.0);
	n.spawner.bikes = std::clamp(n.spawner.bikes, 0.0, 2000.0);
	for (CoachLine &c : n.spawner.coaches) {
		if (c.id == 0) c.id = map_.alloc_object_id();
		c.per_hour = std::clamp(c.per_hour, 0.0, 30.0);
		c.dwell = std::clamp(c.dwell, 0.0, 7200.0);
	}
	std::vector<OdWeight> od;
	for (const OdWeight &w : n.spawner.od) {
		if (w.to != id && map_.node(w.to) && w.weight >= 0.0 && w.weight != 1.0) od.push_back(w);
	}
	std::sort(od.begin(), od.end(), [](const OdWeight &a, const OdWeight &b) { return a.to < b.to; });
	od.erase(std::unique(od.begin(), od.end(), [](const OdWeight &a, const OdWeight &b) { return a.to == b.to; }),
			od.end());
	n.spawner.od = od;
	put_node(n);
}

// --- M3 -------------------------------------------------------------------------------

namespace {

// Unit direction of a segment's leg, pointing away from `node`.
Vec2 leg_dir(const RoadMap &map, const RoadSegment &s, NodeId node) {
	const Curve c = map.curve_of(s);
	return s.from == node ? c.tangent(0.0) : c.tangent(1.0) * -1.0;
}

} // namespace

SignalPlan default_signal_plan(const RoadMap &map, NodeId node) {
	SignalPlan plan;
	const std::vector<SegmentId> legs = map.segments_at(node);
	std::vector<Vec2> dirs;
	for (SegmentId s : legs) dirs.push_back(leg_dir(map, *map.segment(s), node));
	// Pair legs that face each other.
	std::vector<int> partner(legs.size(), -1);
	for (size_t i = 0; i < legs.size(); ++i) {
		if (partner[i] >= 0) continue;
		double best = -0.85; // within ~30 degrees of straight across
		for (size_t j = i + 1; j < legs.size(); ++j) {
			if (partner[j] >= 0) continue;
			const double d = dirs[i].dot(dirs[j]);
			if (d < best) {
				best = d;
				partner[i] = static_cast<int>(j);
			}
		}
		if (partner[i] >= 0) partner[static_cast<size_t>(partner[i])] = static_cast<int>(i);
	}
	auto turn_is_left = [&](size_t from, size_t to) {
		const Vec2 in = dirs[from] * -1.0;
		return cross(in, dirs[to]) < -0.3; // y-down: negative cross = to the left
	};
	std::vector<bool> done(legs.size(), false);
	for (size_t i = 0; i < legs.size(); ++i) {
		if (done[i]) continue;
		SignalPhase ph;
		std::vector<size_t> group = { i };
		if (partner[i] >= 0) group.push_back(static_cast<size_t>(partner[i]));
		for (size_t g : group) {
			done[g] = true;
			for (size_t t = 0; t < legs.size(); ++t) {
				if (t == g) continue;
				SignalMovement m;
				m.from = legs[g];
				m.to = legs[t];
				m.permissive = group.size() > 1 && turn_is_left(g, t);
				ph.moves.push_back(m);
			}
		}
		ph.green = group.size() > 1 ? 25.0 : 15.0;
		plan.phases.push_back(ph);
	}
	return plan;
}

void Document::set_roundabout(NodeId id, const Roundabout &r) {
	const RoadNode *orig = map_.node(id);
	if (!orig) return;
	Scope scope(*this, r.enabled ? "Change roundabout" : "Remove roundabout");
	RoadNode n = *orig;
	n.roundabout = r.enabled ? r : Roundabout{};
	n.roundabout.lanes = std::clamp(n.roundabout.lanes, 1, 3);
	n.roundabout.radius = std::clamp(n.roundabout.radius, 12.0, 40.0);
	const std::vector<SegmentId> here = map_.segments_at(id);
	std::vector<SegmentId> slip;
	for (SegmentId s : n.roundabout.slip) {
		if (std::find(here.begin(), here.end(), s) != here.end() &&
				std::find(slip.begin(), slip.end(), s) == slip.end()) {
			slip.push_back(s);
		}
	}
	std::sort(slip.begin(), slip.end());
	n.roundabout.slip = slip;
	put_node(n);
}

void Document::set_signal_plan(NodeId id, const SignalPlan &plan) {
	const RoadNode *orig = map_.node(id);
	if (!orig) return;
	Scope scope(*this, "Change signal plan");
	RoadNode n = *orig;
	n.control = JunctionControl::Signal;
	n.signal = plan;
	n.signal.amber = std::clamp(n.signal.amber, 1.0, 6.0);
	n.signal.all_red = std::clamp(n.signal.all_red, 0.0, 6.0);
	for (SignalPhase &p : n.signal.phases) p.green = std::clamp(p.green, 3.0, 180.0);
	put_node(n);
}

uint32_t Document::add_stop(SegmentId seg, double u, LaneDir side, StopKind kind, const std::string &name) {
	const RoadSegment *orig = map_.segment(seg);
	if (!orig) return 0;
	Scope scope(*this, "Add bus stop");
	RoadSegment s = *orig;
	BusStop st;
	st.id = map_.alloc_object_id();
	st.u = std::clamp(u, 0.0, 1.0);
	st.side = side == LaneDir::Backward ? LaneDir::Backward : LaneDir::Forward;
	st.kind = kind;
	st.name = name;
	st.bays = kind == StopKind::MainStation ? 4 : 1;
	s.stops.push_back(st);
	put_segment(s);
	return st.id;
}

void Document::set_stop(SegmentId seg, const BusStop &stop) {
	const RoadSegment *orig = map_.segment(seg);
	if (!orig) return;
	Scope scope(*this, "Change bus stop");
	RoadSegment s = *orig;
	for (BusStop &st : s.stops) {
		if (st.id != stop.id) continue;
		st = stop;
		st.u = std::clamp(st.u, 0.0, 1.0);
		st.bays = std::clamp(st.bays, 1, 12);
	}
	put_segment(s);
}

void Document::remove_stop(SegmentId seg, uint32_t stop) {
	const RoadSegment *orig = map_.segment(seg);
	if (!orig) return;
	Scope scope(*this, "Remove bus stop");
	RoadSegment s = *orig;
	s.stops.erase(std::remove_if(s.stops.begin(), s.stops.end(), [stop](const BusStop &b) { return b.id == stop; }),
			s.stops.end());
	put_segment(s);
}

void Document::set_depot(NodeId id, const Depot &depot) {
	const RoadNode *orig = map_.node(id);
	if (!orig) return;
	Scope scope(*this, depot.enabled ? "Change depot" : "Remove depot");
	RoadNode n = *orig;
	n.depot = depot.enabled ? depot : Depot{};
	n.depot.capacity = std::clamp(n.depot.capacity, 1, 200);
	for (BusRoute &r : n.depot.routes) {
		if (r.id == 0) r.id = map_.alloc_object_id();
		r.headway = std::clamp(r.headway, 60.0, 7200.0);
	}
	put_node(n);
}

} // namespace tsim
