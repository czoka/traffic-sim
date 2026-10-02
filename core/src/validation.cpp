#include "tsim/validation.h"

#include "tsim/buildings.h"
#include "tsim/city_data.h"

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
			// A ramp meets the lower level along its lower half (M4).
			const RoadSegment *sb0 = map.segment(b.id);
			const bool a_low = sa->is_ramp() && std::min(sa->level, sa->level_end()) == b.level && !(sb0 && sb0->is_ramp());
			const bool b_low = sb0 && sb0->is_ramp() && std::min(sb0->level, sb0->level_end()) == a.level && !sa->is_ramp();
			if (a.level != b.level && !a_low && !b_low) continue;
			if (a.bb_max.x < b.bb_min.x || b.bb_max.x < a.bb_min.x || a.bb_max.y < b.bb_min.y ||
					b.bb_max.y < a.bb_min.y) {
				continue;
			}
			const RoadSegment *sb = map.segment(b.id);
			if (!sb || sa->from == sb->from || sa->from == sb->to || sa->to == sb->from || sa->to == sb->to) continue;
			auto low_half = [](const SegmentGeom &g, const RoadSegment &rs) {
				// The half of a ramp nearest its lower end.
				const double mid = 0.5 * g.length;
				const bool low_at_start = rs.rise > 0;
				return low_at_start ? g.carriageway(-1e300, mid) : g.carriageway(mid, 1e300);
			};
			Clipper2Lib::PathsD pa, pb;
			if (a_low) {
				pa = to_paths(low_half(a, *sa));
			} else {
				if (!shapes.count(a.id)) shapes[a.id] = to_paths(a.carriageway());
				pa = shapes[a.id];
			}
			if (b_low) {
				pb = to_paths(low_half(b, *sb0));
			} else {
				if (!shapes.count(b.id)) shapes[b.id] = to_paths(b.carriageway());
				pb = shapes[b.id];
			}
			if (pa.empty() || pa[0].size() < 3 || pb.empty() || pb[0].size() < 3) continue;
			const Clipper2Lib::PathsD hit = Clipper2Lib::Intersect(pa, pb, Clipper2Lib::FillRule::NonZero, 2);
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
		const bool spawn_point = rn && (rn->spawner.enabled || rn->depot.enabled);
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
		// A roundabout needs room between its entries: about 22 m of ring per leg (M7).
		if (rn && rn->roundabout.enabled && g.legs.size() >= 3) {
			const double per_leg = 22.0;
			const double circumference = 2.0 * 3.14159265358979 * rn->roundabout.radius;
			if (static_cast<double>(g.legs.size()) * per_leg > circumference + 1e-9) {
				Problem p;
				p.severity = Severity::Warning;
				p.code = "roundabout_small";
				p.message = "Roundabout is small for " + std::to_string(g.legs.size()) + " legs: entries are close together. Use a radius of at least " +
						fmt_m(std::ceil(static_cast<double>(g.legs.size()) * per_leg / (2.0 * 3.14159265358979))) + " (it has " +
						fmt_m(rn->roundabout.radius) + ").";
				p.pos = g.pos;
				p.level = g.level;
				p.nodes = { g.id };
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
		// Ramps (M4): grade limit, and no junctions on them.
		if (s.is_ramp()) {
			const double grade = kLevelHeight * std::abs(s.rise) / std::max(1.0, sg->length);
			if (grade > s.max_grade() + 1e-9) {
				Problem p;
				p.severity = Severity::Error;
				p.code = "ramp_steep";
				char buf[200];
				std::snprintf(buf, sizeof(buf), "Ramp is too steep: %.0f%% (at most %.0f%%%s). It needs %.0f m, it has %.0f m.",
						grade * 100.0, s.max_grade() * 100.0, s.kind == SegmentKind::Footpath && !s.stairs ? ", or make it stairs" : "",
						kLevelHeight * std::abs(s.rise) / s.max_grade(), sg->length);
				p.message = buf;
				p.pos = mid;
				p.level = sg->level;
				p.segments = { s.id };
				out.push_back(p);
			}
			for (NodeId nid : { s.from, s.to }) {
				const NodeGeom *ng = geom.node(nid);
				if (!ng || ng->kind != NodeKind::Junction || ng->legs.size() < 3) continue;
				Problem p;
				p.severity = Severity::Error;
				p.code = "ramp_junction";
				p.message = "Ramps can't have junctions: end the ramp first, then join roads on the level above or below.";
				p.pos = ng->pos;
				p.level = ng->level;
				p.segments = { s.id };
				p.nodes = { nid };
				out.push_back(p);
			}
		}
		// Signal crossings at a junction without signals act as zebras (M4).
		for (int e = 0; e < 2; ++e) {
			if (s.ends[e].crossing.kind != CrossingKind::Signal) continue;
			const RoadNode *rn = map.node(e == 0 ? s.from : s.to);
			if (!rn || rn->control == JunctionControl::Signal) continue;
			Problem p;
			p.severity = Severity::Warning;
			p.code = "crossing_no_signal";
			p.message = "Signal crossing at a junction without traffic signals: it works as a zebra crossing.";
			p.pos = rn->pos;
			p.level = rn->level;
			p.segments = { s.id };
			out.push_back(p);
		}
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
		// Parking beside a mid-block crossing (M7): no bays within 10 m of it.
		bool parking = false;
		for (const LaneSpec &l : s.profile.lanes) parking |= l.type == LaneType::Parking;
		if (parking) {
			for (const Crossing &cr : s.crossings) {
				Problem p;
				p.severity = Severity::Warning;
				p.code = "parking_crossing";
				p.message = "Parking next to a crossing: no bays within 10 m of it, so drivers can see people crossing.";
				p.pos = sg->curve.point(cr.u);
				p.level = s.level;
				p.segments = { s.id };
				out.push_back(p);
			}
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

	// Buildings (M5): known type, not on a road, not on each other.
	const CityData &city = default_city_data();
	std::vector<std::pair<const Building *, std::array<Vec2, 4>>> lots;
	for (const auto &kv : map.buildings()) {
		const Building &b = kv.second;
		if (!city.type(b.type)) {
			Problem p;
			p.severity = Severity::Error;
			p.code = "building_type";
			p.message = "Unknown building type \"" + b.type + "\" (not in the city data).";
			p.pos = b.pos;
			p.level = b.level;
			p.buildings = { b.id };
			out.push_back(p);
			continue;
		}
		lots.push_back({ &b, lot_corners(b, city) });
	}
	for (size_t i = 0; i < lots.size(); ++i) {
		const Building &b = *lots[i].first;
		const std::array<Vec2, 4> &lot = lots[i].second;
		const Vec2 centre = (lot[0] + lot[2]) * 0.5;
		for (const auto &kv : geom.segments()) {
			if (kv.second.level != b.level && kv.second.level - kv.second.rise != b.level) continue;
			if (!lot_overlaps_road(lot, kv.second)) continue;
			Problem p;
			p.severity = Severity::Error;
			p.code = "building_on_road";
			p.message = "A building stands on a road. Move it back from the street.";
			p.pos = centre;
			p.level = b.level;
			p.buildings = { b.id };
			p.segments = { kv.first };
			out.push_back(p);
			break;
		}
		for (size_t j = i + 1; j < lots.size(); ++j) {
			if (lots[j].first->level != b.level || !quads_overlap(lot, lots[j].second)) continue;
			Problem p;
			p.severity = Severity::Error;
			p.code = "building_overlap";
			p.message = "Two buildings overlap.";
			p.pos = centre;
			p.level = b.level;
			p.buildings = { b.id, lots[j].first->id };
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
