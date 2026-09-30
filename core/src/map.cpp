#include "tsim/map.h"

#include <algorithm>
#include <cmath>

namespace tsim {

namespace {

template <typename T>
const T *find_by_id(const std::vector<T> &items, uint32_t id) {
	auto it = std::lower_bound(items.begin(), items.end(), id, [](const T &item, uint32_t v) { return item.id < v; });
	if (it == items.end() || it->id != id) {
		return nullptr;
	}
	return &*it;
}

template <typename T>
T *find_by_id_mut(std::vector<T> &items, uint32_t id) {
	return const_cast<T *>(find_by_id(static_cast<const std::vector<T> &>(items), id));
}

double sign(double v) { return v < 0.0 ? -1.0 : 1.0; }

} // namespace

double Map::lane_offset(int index, int lane_count, double lane_width) {
	return (static_cast<double>(lane_count - 1) * 0.5 - static_cast<double>(index)) * lane_width;
}

bool Map::compute_lane_length(const Segment &seg, Lane &lane) const {
	const Node *a = node(seg.from);
	const Node *b = node(seg.to);
	if (!a || !b) {
		return false;
	}
	if (seg.kind == SegmentKind::Straight) {
		lane.length = (b->pos - a->pos).length();
	} else {
		const double radius = (a->pos - seg.center).length();
		const double lane_radius = radius - sign(seg.sweep) * lane.offset;
		if (lane_radius <= 0.0) {
			return false;
		}
		lane.length = lane_radius * std::fabs(seg.sweep);
	}
	return lane.length > 0.0;
}

NodeId Map::add_node(Vec2 pos) {
	Node n;
	n.id = next_node_id_++;
	n.pos = pos;
	nodes_.push_back(n);
	return n.id;
}

SegmentId Map::add_segment(Segment seg, int lane_count) {
	if (lane_count < 1 || lane_count > 8 || !(seg.lane_width > 0.0) || !(seg.speed_limit > 0.0)) {
		return kNoId;
	}
	if (!node(seg.from) || !node(seg.to) || seg.from == seg.to) {
		return kNoId;
	}
	// Build lanes first and only commit if every lane has valid geometry.
	std::vector<Lane> new_lanes;
	for (int i = 0; i < lane_count; ++i) {
		Lane l;
		l.index = static_cast<uint32_t>(i);
		l.offset = lane_offset(i, lane_count, seg.lane_width);
		if (!compute_lane_length(seg, l)) {
			return kNoId;
		}
		new_lanes.push_back(l);
	}
	seg.id = next_segment_id_++;
	seg.lanes.clear();
	for (Lane &l : new_lanes) {
		l.id = next_lane_id_++;
		l.segment = seg.id;
		seg.lanes.push_back(l.id);
		lanes_.push_back(l);
	}
	segments_.push_back(seg);
	return seg.id;
}

SegmentId Map::add_straight(NodeId from, NodeId to, int lane_count, double lane_width, double speed_limit) {
	Segment seg;
	seg.from = from;
	seg.to = to;
	seg.kind = SegmentKind::Straight;
	seg.lane_width = lane_width;
	seg.speed_limit = speed_limit;
	return add_segment(seg, lane_count);
}

SegmentId Map::add_arc(NodeId from, NodeId to, Vec2 center, double sweep, int lane_count, double lane_width,
		double speed_limit) {
	const Node *a = node(from);
	const Node *b = node(to);
	if (!a || !b || sweep == 0.0 || std::fabs(sweep) > 2.0 * kPi) {
		return kNoId;
	}
	const double ra = (a->pos - center).length();
	const double rb = (b->pos - center).length();
	if (std::fabs(ra - rb) > 1e-3 * std::max(1.0, ra)) {
		return kNoId;
	}
	Segment seg;
	seg.from = from;
	seg.to = to;
	seg.kind = SegmentKind::Arc;
	seg.center = center;
	seg.sweep = sweep;
	seg.lane_width = lane_width;
	seg.speed_limit = speed_limit;
	return add_segment(seg, lane_count);
}

bool Map::connect(LaneId from, LaneId to) {
	Lane *a = find_by_id_mut(lanes_, from);
	if (!a || !lane(to)) {
		return false;
	}
	if (std::find(a->next.begin(), a->next.end(), to) == a->next.end()) {
		a->next.push_back(to);
	}
	return true;
}

const Node *Map::node(NodeId id) const { return find_by_id(nodes_, id); }
const Segment *Map::segment(SegmentId id) const { return find_by_id(segments_, id); }
const Lane *Map::lane(LaneId id) const { return find_by_id(lanes_, id); }

Pose Map::lane_pose(const Lane &lane, double s) const {
	Pose p;
	const Segment *seg = segment(lane.segment);
	if (!seg) {
		return p;
	}
	const Node *a = node(seg->from);
	const Node *b = node(seg->to);
	if (!a || !b) {
		return p;
	}
	const double t = lane.length > 0.0 ? std::clamp(s / lane.length, 0.0, 1.0) : 0.0;
	if (seg->kind == SegmentKind::Straight) {
		const Vec2 d = (b->pos - a->pos).normalized();
		p.dir = d;
		p.pos = a->pos + d.right() * lane.offset + (b->pos - a->pos) * t;
	} else {
		const Vec2 r0 = a->pos - seg->center;
		const double radius = r0.length() - sign(seg->sweep) * lane.offset;
		const double theta = std::atan2(r0.y, r0.x) + seg->sweep * t;
		const double c = std::cos(theta);
		const double sn = std::sin(theta);
		p.pos = seg->center + Vec2{ c, sn } * radius;
		p.dir = Vec2{ -sn, c } * sign(seg->sweep);
	}
	return p;
}

void Map::offset_polyline(const Segment &seg, double offset, double max_step, std::vector<Vec2> &out) const {
	out.clear();
	const Node *a = node(seg.from);
	const Node *b = node(seg.to);
	if (!a || !b) {
		return;
	}
	if (seg.kind == SegmentKind::Straight) {
		const Vec2 n = (b->pos - a->pos).normalized().right() * offset;
		out.push_back(a->pos + n);
		out.push_back(b->pos + n);
		return;
	}
	const Vec2 r0 = a->pos - seg.center;
	const double radius = r0.length() - sign(seg.sweep) * offset;
	const double theta0 = std::atan2(r0.y, r0.x);
	const double arc_len = std::fabs(seg.sweep) * radius;
	const int steps = std::max(2, static_cast<int>(std::ceil(arc_len / std::max(0.5, max_step))));
	for (int i = 0; i <= steps; ++i) {
		const double th = theta0 + seg.sweep * (static_cast<double>(i) / steps);
		out.push_back(seg.center + Vec2{ std::cos(th), std::sin(th) } * radius);
	}
}

void Map::lane_polyline(const Lane &lane, double max_step, std::vector<Vec2> &out) const {
	const Segment *seg = segment(lane.segment);
	if (!seg) {
		out.clear();
		return;
	}
	offset_polyline(*seg, lane.offset, max_step, out);
}

void Map::clear() {
	nodes_.clear();
	segments_.clear();
	lanes_.clear();
	next_node_id_ = next_segment_id_ = next_lane_id_ = 1;
}

// ---------------------------------------------------------------------------
// MapBuilder

bool MapBuilder::add_node(const Node &n, std::string *err) {
	if (n.id == kNoId || (!map_.nodes_.empty() && map_.nodes_.back().id >= n.id)) {
		if (err) *err = "node ids must be positive and ascending (id " + std::to_string(n.id) + ")";
		return false;
	}
	map_.nodes_.push_back(n);
	return true;
}

bool MapBuilder::add_segment(const Segment &s, std::string *err) {
	if (s.id == kNoId || (!map_.segments_.empty() && map_.segments_.back().id >= s.id)) {
		if (err) *err = "segment ids must be positive and ascending (id " + std::to_string(s.id) + ")";
		return false;
	}
	map_.segments_.push_back(s);
	return true;
}

bool MapBuilder::add_lane(const Lane &l, std::string *err) {
	if (l.id == kNoId || (!map_.lanes_.empty() && map_.lanes_.back().id >= l.id)) {
		if (err) *err = "lane ids must be positive and ascending (id " + std::to_string(l.id) + ")";
		return false;
	}
	map_.lanes_.push_back(l);
	return true;
}

bool MapBuilder::finish(uint32_t next_node, uint32_t next_segment, uint32_t next_lane, std::string *err) {
	auto fail = [err](const std::string &msg) {
		if (err) *err = msg;
		return false;
	};
	for (const Segment &s : map_.segments_) {
		if (!map_.node(s.from) || !map_.node(s.to) || s.from == s.to) {
			return fail("segment " + std::to_string(s.id) + " has invalid nodes");
		}
		if (s.lanes.empty() || s.lanes.size() > 8) {
			return fail("segment " + std::to_string(s.id) + " needs 1-8 lanes");
		}
		if (!(s.lane_width > 0.0) || !(s.speed_limit > 0.0)) {
			return fail("segment " + std::to_string(s.id) + " has invalid lane width or speed limit");
		}
		if (s.kind == SegmentKind::Arc && (s.sweep == 0.0 || std::fabs(s.sweep) > 2.0 * kPi)) {
			return fail("segment " + std::to_string(s.id) + " has an invalid sweep");
		}
		const int count = static_cast<int>(s.lanes.size());
		for (int i = 0; i < count; ++i) {
			Lane *l = find_by_id_mut(map_.lanes_, s.lanes[static_cast<size_t>(i)]);
			if (!l || l->segment != s.id) {
				return fail("segment " + std::to_string(s.id) + " lists a lane that does not belong to it");
			}
			l->index = static_cast<uint32_t>(i);
			l->offset = Map::lane_offset(i, count, s.lane_width);
			if (!map_.compute_lane_length(s, *l)) {
				return fail("lane " + std::to_string(l->id) + " has degenerate geometry");
			}
		}
	}
	for (const Lane &l : map_.lanes_) {
		if (!(l.length > 0.0)) {
			return fail("lane " + std::to_string(l.id) + " is not part of any segment");
		}
		for (LaneId n : l.next) {
			if (!map_.lane(n)) {
				return fail("lane " + std::to_string(l.id) + " connects to missing lane " + std::to_string(n));
			}
		}
	}
	auto max_id = [](const auto &v) { return v.empty() ? 0u : v.back().id; };
	map_.next_node_id_ = std::max(next_node, max_id(map_.nodes_) + 1);
	map_.next_segment_id_ = std::max(next_segment, max_id(map_.segments_) + 1);
	map_.next_lane_id_ = std::max(next_lane, max_id(map_.lanes_) + 1);
	return true;
}

// ---------------------------------------------------------------------------

RingInfo build_ring(Map &map, double radius, int lane_count, double lane_width, double speed_limit) {
	RingInfo info;
	// Four nodes at exact positions so no trig is involved: east, south, west,
	// north (y-down). A +pi/2 sweep runs clockwise on screen.
	const Vec2 pts[4] = { { radius, 0.0 }, { 0.0, radius }, { -radius, 0.0 }, { 0.0, -radius } };
	NodeId nodes[4];
	for (int i = 0; i < 4; ++i) {
		nodes[i] = map.add_node(pts[i]);
	}
	for (int i = 0; i < 4; ++i) {
		const SegmentId s = map.add_arc(nodes[i], nodes[(i + 1) % 4], Vec2{ 0.0, 0.0 }, kHalfPi, lane_count,
				lane_width, speed_limit);
		info.segments.push_back(s);
	}
	for (int i = 0; i < 4; ++i) {
		const Segment *a = map.segment(info.segments[static_cast<size_t>(i)]);
		const Segment *b = map.segment(info.segments[static_cast<size_t>((i + 1) % 4)]);
		if (!a || !b) {
			continue;
		}
		for (size_t k = 0; k < a->lanes.size() && k < b->lanes.size(); ++k) {
			map.connect(a->lanes[k], b->lanes[k]);
		}
	}
	return info;
}

} // namespace tsim
