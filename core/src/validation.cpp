#include "tsim/validation.h"

#include "clipper2/clipper.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace tsim {

namespace {

Clipper2Lib::PathsD to_paths(const std::vector<Vec2> &poly) {
	Clipper2Lib::PathsD p(1);
	for (const Vec2 &v : poly) p[0].push_back(Clipper2Lib::PointD(v.x, v.y));
	return p;
}

Vec2 centroid(const Clipper2Lib::PathsD &paths) {
	double x = 0, y = 0;
	size_t n = 0;
	for (const auto &p : paths) {
		for (const auto &q : p) {
			x += q.x;
			y += q.y;
			++n;
		}
	}
	return n ? Vec2{ x / n, y / n } : Vec2{};
}

std::string fmt_m(double v) {
	char buf[32];
	std::snprintf(buf, sizeof(buf), "%.0f m", v);
	return buf;
}

} // namespace

std::vector<Problem> validate(const RoadMap &map, const RoadGeometry &geom) {
	std::vector<Problem> out;

	// Overlapping roads on the same level that don't share a node.
	std::vector<const SegmentGeom *> segs;
	std::map<SegmentId, Clipper2Lib::PathsD> shapes;
	for (const auto &kv : geom.segments()) {
		segs.push_back(&kv.second);
	}
	for (size_t i = 0; i < segs.size(); ++i) {
		const SegmentGeom &a = *segs[i];
		const RoadSegment *sa = map.segment(a.id);
		if (!sa) continue;
		for (size_t j = i + 1; j < segs.size(); ++j) {
			const SegmentGeom &b = *segs[j];
			if (a.level != b.level) continue;
			if (a.bb_max.x < b.bb_min.x || b.bb_max.x < a.bb_min.x || a.bb_max.y < b.bb_min.y ||
					b.bb_max.y < a.bb_min.y) {
				continue;
			}
			const RoadSegment *sb = map.segment(b.id);
			if (!sb || sa->from == sb->from || sa->from == sb->to || sa->to == sb->from || sa->to == sb->to) continue;
			if (!shapes.count(a.id)) shapes[a.id] = to_paths(a.carriageway());
			if (!shapes.count(b.id)) shapes[b.id] = to_paths(b.carriageway());
			const Clipper2Lib::PathsD hit =
					Clipper2Lib::Intersect(shapes[a.id], shapes[b.id], Clipper2Lib::FillRule::NonZero, 2);
			if (std::fabs(Clipper2Lib::Area(hit)) < 0.5) continue;
			Problem p;
			p.severity = Severity::Error;
			p.code = "overlap";
			p.message = "Roads cross without a junction. Join them at a node, or put one on another level.";
			p.pos = centroid(hit);
			p.level = a.level;
			p.segments = { a.id, b.id };
			out.push_back(p);
		}
	}

	for (const auto &kv : geom.nodes()) {
		const NodeGeom &g = kv.second;
		const RoadNode *rn = map.node(g.id);
		const bool spawn_point = rn && rn->spawner.enabled;
		if (g.kind == NodeKind::End && spawn_point) {
			// Traffic enters and leaves the map here.
		} else if (g.kind == NodeKind::End && !g.dead_lanes.empty()) {
			Problem p;
			p.severity = Severity::Warning;
			p.code = "road_end";
			p.message = "Road ends here. Cars won't use it: add a spawn point (N) or connect it.";
			p.pos = g.pos;
			p.level = g.level;
			p.nodes = { g.id };
			p.segments = { g.legs[0].seg };
			out.push_back(p);
		} else if (!g.dead_lanes.empty()) {
			std::map<SegmentId, int> per_seg;
			for (const auto &d : g.dead_lanes) ++per_seg[d.first];
			for (const auto &ps : per_seg) {
				Problem p;
				p.severity = Severity::Error;
				p.code = "dead_end_lane";
				p.message = std::to_string(ps.second) + (ps.second == 1 ? " lane has" : " lanes have") +
						" no way out at this node. Check turn rules and road directions.";
				p.pos = g.pos;
				p.level = g.level;
				p.nodes = { g.id };
				p.segments = { ps.first };
				out.push_back(p);
			}
		}
		if (g.kind == NodeKind::Junction && g.legs.size() > 6) {
			Problem p;
			p.severity = Severity::Error;
			p.code = "too_many_legs";
			p.message = "Junctions support 3 to 6 legs; this one has " + std::to_string(g.legs.size()) + ".";
			p.pos = g.pos;
			p.level = g.level;
			p.nodes = { g.id };
			out.push_back(p);
		}
	}

	for (const auto &kv : map.segments()) {
		const RoadSegment &s = kv.second;
		const SegmentGeom *sg = geom.segment(s.id);
		if (!sg) continue;
		const double drawn = sg->length - sg->trim[0] - sg->trim[1];
		const Vec2 mid = sg->curve.point(0.5);
		if (drawn < 1.0) {
			Problem p;
			p.severity = Severity::Warning;
			p.code = "short_road";
			p.message = "Road is too short for the junctions at its ends.";
			p.pos = mid;
			p.level = s.level;
			p.segments = { s.id };
			out.push_back(p);
		}
		for (int e = 0; e < 2; ++e) {
			const EndRules &r = s.ends[e];
			if (r.left != TurnRule::TurnLane && r.right != TurnRule::TurnLane) continue;
			const NodeGeom *ng = geom.node(e == 0 ? s.from : s.to);
			if (!ng || ng->kind != NodeKind::Junction) continue;
			const bool too_short = r.turn_lane_length < 20.0;
			const bool no_room = r.turn_lane_length + 15.0 > drawn;
			if (!too_short && !no_room) continue;
			Problem p;
			p.severity = Severity::Warning;
			p.code = "short_turn_lane";
			p.message = too_short ? "Turn lane is shorter than 20 m (" + fmt_m(r.turn_lane_length) + ")."
								  : "Turn lane (" + fmt_m(r.turn_lane_length) + ") doesn't fit on this road (" +
							fmt_m(drawn) + ").";
			p.pos = sg->curve.point(e == 0 ? 0.15 : 0.85);
			p.level = s.level;
			p.segments = { s.id };
			out.push_back(p);
		}
	}

	std::stable_sort(out.begin(), out.end(), [](const Problem &a, const Problem &b) {
		if (a.severity != b.severity) return a.severity > b.severity;
		return a.code < b.code;
	});
	return out;
}

} // namespace tsim
