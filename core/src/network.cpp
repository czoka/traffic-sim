#include "tsim/network.h"

#include "tsim/hash.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>

namespace tsim {

double quantize(double v) { return std::round(v * 1024.0) / 1024.0; }

namespace {

Vec2 qv(Vec2 v) { return Vec2{ quantize(v.x), quantize(v.y) }; }

void set_polyline(NetLane &l, const std::vector<Vec2> &pts) {
	l.pts.clear();
	for (const Vec2 &p : pts) {
		const Vec2 q = qv(p);
		if (!l.pts.empty() && l.pts.back() == q) continue;
		l.pts.push_back(q);
	}
	if (l.pts.size() == 1) l.pts.push_back(l.pts.front());
	l.cum.assign(l.pts.size(), 0.0);
	for (size_t i = 1; i < l.pts.size(); ++i) {
		l.cum[i] = l.cum[i - 1] + (l.pts[i] - l.pts[i - 1]).length();
	}
	l.length = l.pts.empty() ? 0.0 : l.cum.back();
	if (l.length < 0.01 && !l.pts.empty()) {
		// Degenerate joint (two roads meeting exactly): keep it drivable.
		l.cum.back() = 0.01;
		l.length = 0.01;
	}
}

struct Hasher {
	StateHasher h;
	void d(double v) { h.add_double(v); }
	void u(uint64_t v) { h.add_u64(v); }
	void v(Vec2 p) {
		d(p.x);
		d(p.y);
	}
};

// Segment-segment intersection: parameters along both.
bool seg_intersect(Vec2 p, Vec2 p2, Vec2 q, Vec2 q2, double &t, double &u) {
	const Vec2 r = p2 - p;
	const Vec2 s = q2 - q;
	const double den = cross(r, s);
	if (den == 0.0) return false;
	const Vec2 w = q - p;
	t = cross(w, s) / den;
	u = cross(w, r) / den;
	return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
}

// Closest point on segment [a, b] to p: parameter.
double closest_on(Vec2 a, Vec2 b, Vec2 p) {
	const Vec2 d = b - a;
	const double l2 = d.dot(d);
	if (l2 <= 0.0) return 0.0;
	return std::clamp((p - a).dot(d) / l2, 0.0, 1.0);
}

bool is_car_lane(LaneType t) { return t == LaneType::General || t == LaneType::Bus || t == LaneType::Turn; }

// End (0 = from node, 1 = to node) that a lane of segment s travels towards.
int arriving_end(const RoadSegment &s, LaneId lane) {
	for (const LaneSpec &l : s.profile.lanes) {
		if (l.id == lane) return l.dir == LaneDir::Backward ? 0 : 1;
	}
	if (s.ends[0].left_lane == lane || s.ends[0].right_lane == lane) return 0;
	return 1;
}

} // namespace

// --- Network queries -------------------------------------------------------------

int32_t Network::find(const LaneKey &k) const {
	auto it = index_.find(k);
	return it == index_.end() ? -1 : it->second;
}

int32_t Network::junction_at(NodeId node) const {
	auto it = junction_index_.find(node);
	return it == junction_index_.end() ? -1 : it->second;
}

const NetSpawner *Network::spawner_at(NodeId node) const {
	for (const NetSpawner &s : spawners) {
		if (s.node == node) return &s;
	}
	return nullptr;
}

Pose Network::pose(int32_t lane, double s) const {
	const NetLane &l = lanes[static_cast<size_t>(lane)];
	Pose p;
	if (l.pts.empty()) return p;
	if (l.pts.size() == 1) {
		p.pos = l.pts[0];
		return p;
	}
	s = std::clamp(s, 0.0, l.length);
	size_t i = static_cast<size_t>(std::upper_bound(l.cum.begin(), l.cum.end(), s) - l.cum.begin());
	i = std::clamp<size_t>(i, 1, l.pts.size() - 1);
	const double seg = l.cum[i] - l.cum[i - 1];
	const double k = seg > 0.0 ? (s - l.cum[i - 1]) / seg : 0.0;
	p.pos = l.pts[i - 1] + (l.pts[i] - l.pts[i - 1]) * k;
	// Heading: the segment's direction, or the nearest non-degenerate one.
	Vec2 d = l.pts[i] - l.pts[i - 1];
	for (size_t j = i; d.dot(d) == 0.0 && j + 1 < l.pts.size(); ++j) d = l.pts[j + 1] - l.pts[j];
	p.dir = d.normalized();
	return p;
}

double Network::map_across(int32_t from, double s, int32_t to) const {
	const NetLane &a = lanes[static_cast<size_t>(from)];
	const NetLane &b = lanes[static_cast<size_t>(to)];
	if (a.pts.size() < 2 || b.pts.size() < 2) return -1.0;
	s = std::clamp(s, 0.0, a.length);
	size_t i = static_cast<size_t>(std::upper_bound(a.cum.begin(), a.cum.end(), s) - a.cum.begin());
	i = std::clamp<size_t>(i, 1, a.pts.size() - 1) - 1;
	const double seg = a.cum[i + 1] - a.cum[i];
	const double f = seg > 0.0 ? (s - a.cum[i]) / seg : 0.0;
	const int64_t j = static_cast<int64_t>(a.first_sample) + static_cast<int64_t>(i) - b.first_sample;
	if (j < 0 || j + 1 >= static_cast<int64_t>(b.pts.size())) return -1.0;
	const size_t k = static_cast<size_t>(j);
	return b.cum[k] + (b.cum[k + 1] - b.cum[k]) * f;
}

bool Network::can_change(int32_t lane, bool to_left, double s) const {
	const NetLane &l = lanes[static_cast<size_t>(lane)];
	for (const auto &r : to_left ? l.change_left : l.change_right) {
		if (s >= r.first && s <= r.second) return true;
	}
	return false;
}

uint64_t Network::hash() const {
	Hasher h;
	for (const NetLane &l : lanes) {
		h.u(static_cast<uint64_t>(l.kind));
		h.u(l.key.a);
		h.u(l.key.b);
		h.d(l.length);
		h.d(l.speed_limit);
		for (const Vec2 &p : l.pts) h.v(p);
		for (double c : l.cum) h.d(c);
		h.u(static_cast<uint64_t>(static_cast<int64_t>(l.left)));
		h.u(static_cast<uint64_t>(static_cast<int64_t>(l.right)));
		for (const auto &r : l.change_left) {
			h.d(r.first);
			h.d(r.second);
		}
		for (const auto &r : l.change_right) {
			h.d(r.first);
			h.d(r.second);
		}
		for (int32_t n : l.next) h.u(static_cast<uint64_t>(n));
		for (const Conflict &c : l.conflicts) {
			h.u(static_cast<uint64_t>(c.other));
			h.d(c.s_self);
			h.d(c.s_other);
			h.u(static_cast<uint64_t>(c.priority + 1));
			h.u(c.merge);
		}
		h.u(l.sink);
		h.u(l.merge_end);
		h.u(static_cast<uint64_t>(static_cast<int64_t>(l.approach_of)));
	}
	for (const NetJunction &j : junctions) {
		h.u(j.node);
		h.u(static_cast<uint64_t>(j.control));
		h.u(j.arbitrated);
		h.u(j.joint);
	}
	for (const NetSpawner &s : spawners) {
		h.u(s.node);
		for (int32_t l : s.spawn_lanes) h.u(static_cast<uint64_t>(l));
		for (int32_t l : s.sink_lanes) h.u(static_cast<uint64_t>(l));
	}
	return h.h.value();
}

void Network::clear() {
	lanes.clear();
	junctions.clear();
	spawners.clear();
	index_.clear();
	junction_index_.clear();
	max_speed = 13.9;
}

// --- Compiler ------------------------------------------------------------------------

void NetworkCompiler::clear_cache() {
	segments_.clear();
	nodes_.clear();
}

void NetworkCompiler::compile(const RoadMap &map, const RoadGeometry &geom, Network &out) {
	const auto t0 = std::chrono::steady_clock::now();
	stats_ = CompileStats{};
	out.clear();

	auto stop_at = [&](NodeId n) {
		const NodeGeom *g = geom.node(n);
		return g && g->kind == NodeKind::Junction && g->legs.size() >= 3;
	};

	// --- 1. Segments --------------------------------------------------------------
	std::set<SegmentId> live_segments;
	for (const auto &kv : map.segments()) {
		const RoadSegment &seg = kv.second;
		const SegmentGeom *sg = geom.segment(seg.id);
		if (!sg || seg.kind != SegmentKind::Road) continue;
		live_segments.insert(seg.id);
		const bool stop[2] = { stop_at(seg.from), stop_at(seg.to) };

		Hasher h;
		h.u(seg.level);
		h.d(seg.speed_limit);
		h.u(stop[0]);
		h.u(stop[1]);
		h.d(sg->length);
		h.d(sg->trim[0]);
		h.d(sg->trim[1]);
		h.d(geom.pocket_taper);
		h.d(geom.solid_before_stop);
		for (const GeomLane &l : sg->lanes) {
			h.u(l.id);
			h.u(static_cast<uint64_t>(l.type));
			h.u(static_cast<uint64_t>(l.dir));
			h.u(static_cast<uint64_t>(l.pocket_end + 1));
			h.u(static_cast<uint64_t>(l.profile_index + 1));
		}
		for (size_t k = 0; k < sg->s.size(); ++k) {
			h.d(sg->s[k]);
			h.v(sg->p[k]);
			h.v(sg->n[k]);
		}
		for (const auto &e : sg->x) {
			h.d(e.first);
			h.d(e.second);
		}
		for (const NoChangeZone &z : seg.no_change) {
			h.u(static_cast<uint64_t>(z.edge));
			h.d(z.u0);
			h.d(z.u1);
			h.u(z.block_left_to_right);
			h.u(z.block_right_to_left);
		}
		const uint64_t sig = h.h.value();
		SegmentPart &part = segments_[seg.id];
		++stats_.segments;
		if (part.sig == sig && sig != 0) continue;
		++stats_.segments_compiled;
		part.sig = sig;
		part.lanes.clear();

		const size_t ns = sg->s.size();
		const size_t nl = sg->lanes.size();
		std::vector<int32_t> local(nl, -1);
		for (size_t i = 0; i < nl; ++i) {
			const GeomLane &gl = sg->lanes[i];
			if (!is_car_lane(gl.type) || ns < 2) continue;
			const bool fwd = gl.dir == LaneDir::Forward;
			std::vector<Vec2> pts;
			int32_t first = -1;
			for (size_t j = 0; j < ns; ++j) {
				const size_t k = fwd ? j : ns - 1 - j;
				const auto &e = sg->edge(k, i);
				if (gl.pocket_end >= 0 && e.second - e.first <= 0.05) {
					if (first >= 0) break; // pockets are one contiguous run
					continue;
				}
				if (first < 0) first = static_cast<int32_t>(j);
				pts.push_back(sg->at(k, 0.5 * (e.first + e.second)));
			}
			if (pts.size() < 2) continue;
			NetLane l;
			l.kind = NetLaneKind::Road;
			l.key = LaneKey{ NetLaneKind::Road, gl.id, 0 };
			l.type = gl.type;
			l.level = seg.level;
			l.speed_limit = quantize(seg.speed_limit);
			l.segment = seg.id;
			l.dir = gl.dir;
			l.pocket = gl.pocket_end >= 0;
			l.first_sample = first;
			set_polyline(l, pts);
			// Keep samples aligned with first_sample (set_polyline may drop
			// duplicate points only for degenerate lanes).
			if (l.pts.size() != pts.size()) {
				l.pts.clear();
				for (const Vec2 &p : pts) l.pts.push_back(qv(p));
				l.cum.assign(l.pts.size(), 0.0);
				for (size_t k = 1; k < l.pts.size(); ++k) l.cum[k] = l.cum[k - 1] + (l.pts[k] - l.pts[k - 1]).length();
				l.length = std::max(0.01, l.cum.back());
				l.cum.back() = l.length;
			}
			local[i] = static_cast<int32_t>(part.lanes.size());
			part.lanes.push_back(std::move(l));
		}
		// Neighbours in the sense of travel, and where changes are allowed.
		for (size_t i = 0; i < nl; ++i) {
			if (local[i] < 0) continue;
			NetLane &a = part.lanes[static_cast<size_t>(local[i])];
			const bool fwd = a.dir == LaneDir::Forward;
			for (int side = 0; side < 2; ++side) { // 0 = travel-left
				const bool list_left = (side == 0) == fwd;
				if (list_left ? i == 0 : i + 1 >= nl) continue;
				const size_t bi = list_left ? i - 1 : i + 1;
				if (local[bi] < 0 || sg->lanes[bi].dir != a.dir) continue;
				(side == 0 ? a.left : a.right) = local[bi];
				const NetLane &b = part.lanes[static_cast<size_t>(local[bi])];
				const GeomLane &ga = sg->lanes[i];
				const GeomLane &gb = sg->lanes[bi];
				const int end = fwd ? 1 : 0;
				// Pocket starts, as sample indices in travel order.
				const int32_t pocket_start = ga.pocket_end >= 0 ? a.first_sample : gb.pocket_end >= 0 ? b.first_sample : -1;
				std::vector<bool> ok(a.pts.size(), false);
				for (size_t j = 0; j < a.pts.size(); ++j) {
					const int32_t tj = a.first_sample + static_cast<int32_t>(j);
					if (tj < b.first_sample || tj >= b.first_sample + static_cast<int32_t>(b.pts.size())) continue;
					const size_t k = fwd ? static_cast<size_t>(tj) : ns - 1 - static_cast<size_t>(tj);
					const double sc = sg->s[k];
					const double to_stop = end == 1 ? sg->length - sg->trim[1] - sc : sc - sg->trim[0];
					if (stop[end] && to_stop < geom.solid_before_stop) continue;
					if (pocket_start >= 0) {
						const size_t k0 = fwd ? static_cast<size_t>(pocket_start) : ns - 1 - static_cast<size_t>(pocket_start);
						if (std::fabs(sc - sg->s[k0]) > geom.pocket_taper + 5.0) continue;
					}
					bool blocked = false;
					if (ga.profile_index >= 0 && gb.profile_index >= 0 &&
							std::abs(ga.profile_index - gb.profile_index) == 1) {
						const int edge = std::max(ga.profile_index, gb.profile_index);
						const bool l2r = ga.profile_index < gb.profile_index; // list-left to list-right
						const double u = sg->length > 0.0 ? sc / sg->length : 0.0;
						for (const NoChangeZone &z : seg.no_change) {
							if (z.edge != edge || u < z.u0 || u > z.u1) continue;
							if (l2r ? z.block_left_to_right : z.block_right_to_left) blocked = true;
						}
					}
					ok[j] = !blocked;
				}
				auto &ranges = side == 0 ? a.change_left : a.change_right;
				for (size_t j = 0; j + 1 < ok.size(); ++j) {
					if (!ok[j] || !ok[j + 1]) continue;
					if (!ranges.empty() && ranges.back().second == a.cum[j]) {
						ranges.back().second = a.cum[j + 1];
					} else {
						ranges.push_back({ a.cum[j], a.cum[j + 1] });
					}
				}
			}
		}
	}
	for (auto it = segments_.begin(); it != segments_.end();) {
		it = live_segments.count(it->first) ? std::next(it) : segments_.erase(it);
	}

	// --- 2. Nodes: connectors and conflict matrices ------------------------------------
	std::set<NodeId> live_nodes;
	for (const auto &kv : geom.nodes()) {
		const NodeGeom &g = kv.second;
		const RoadNode *rn = map.node(g.id);
		if (!rn || g.connectors.empty()) continue;
		live_nodes.insert(g.id);
		Hasher h;
		h.u(static_cast<uint64_t>(g.kind));
		h.u(static_cast<uint64_t>(g.level));
		h.v(g.pos);
		h.u(static_cast<uint64_t>(rn->control));
		for (SegmentId s : rn->priority) h.u(s);
		h.d(lateral_accel);
		h.d(conflict_distance);
		for (const Leg &leg : g.legs) {
			h.u(leg.seg);
			h.u(leg.at_start);
			h.v(leg.dir);
			const RoadSegment *s = map.segment(leg.seg);
			h.d(s ? s->speed_limit : 0.0);
		}
		for (const Connector &c : g.connectors) {
			h.u(c.from_seg);
			h.u(c.from_lane);
			h.u(c.to_seg);
			h.u(c.to_lane);
			h.u(static_cast<uint64_t>(c.turn));
			for (const Vec2 &p : c.path) h.v(p);
		}
		const uint64_t sig = h.h.value();
		NodePart &part = nodes_[g.id];
		++stats_.junctions;
		if (part.sig == sig && sig != 0) continue;
		++stats_.junctions_compiled;
		part.sig = sig;
		part.connectors.clear();
		part.arbitrated = false;

		std::set<std::pair<LaneId, LaneId>> seen;
		for (const Connector &c : g.connectors) {
			if (!seen.insert({ c.from_lane, c.to_lane }).second) continue;
			const RoadSegment *fs = map.segment(c.from_seg);
			const RoadSegment *ts = map.segment(c.to_seg);
			if (!fs || !ts) continue;
			NetLane l;
			l.kind = NetLaneKind::Connector;
			l.key = LaneKey{ NetLaneKind::Connector, c.from_lane, c.to_lane };
			l.level = g.level;
			l.node = g.id;
			l.turn = c.turn;
			set_polyline(l, c.path);
			double limit = std::min(fs->speed_limit, ts->speed_limit);
			if (l.pts.size() >= 3) {
				const Vec2 da = (l.pts[1] - l.pts[0]).normalized();
				const Vec2 db = (l.pts.back() - l.pts[l.pts.size() - 2]).normalized();
				const double chord = (da - db).length(); // ~ turning angle in radians
				if (chord > 0.05) {
					const double radius = l.length / chord;
					limit = std::min(limit, std::max(3.0, std::sqrt(lateral_accel * radius)));
				}
			}
			l.speed_limit = quantize(limit);
			const int end = arriving_end(*fs, c.from_lane);
			for (size_t k = 0; k < g.legs.size(); ++k) {
				if (g.legs[k].seg == c.from_seg && g.legs[k].at_start == (end == 0)) l.leg = static_cast<int>(k);
			}
			part.connectors.push_back(std::move(l));
		}
		// Merges at continuations and tapers: the first connector into a lane
		// is the through lane; others merge into it and yield.
		const bool joint = g.kind == NodeKind::Continuation || g.kind == NodeKind::Taper;
		part.joint = joint;
		std::map<LaneId, size_t> first_into;
		for (size_t i = 0; i < part.connectors.size(); ++i) {
			first_into.emplace(part.connectors[i].key.b, i);
		}
		const size_t nc = part.connectors.size();
		for (size_t i = 0; i < nc; ++i) {
			for (size_t j = i + 1; j < nc; ++j) {
				NetLane &a = part.connectors[i];
				NetLane &b = part.connectors[j];
				if (a.key.a == b.key.a) continue; // diverge from one lane: followers handle it
				Conflict ca, cb;
				bool hit = false;
				if (a.key.b == b.key.b) {
					hit = true;
					ca.merge = cb.merge = true;
					ca.s_self = a.length;
					ca.s_other = b.length;
				} else {
					// First crossing along a.
					double best = 1e300;
					for (size_t p = 0; p + 1 < a.pts.size(); ++p) {
						for (size_t q = 0; q + 1 < b.pts.size(); ++q) {
							double t = 0.0, u = 0.0;
							if (!seg_intersect(a.pts[p], a.pts[p + 1], b.pts[q], b.pts[q + 1], t, u)) continue;
							const double sa = a.cum[p] + (a.cum[p + 1] - a.cum[p]) * t;
							if (sa < best) {
								best = sa;
								ca.s_self = sa;
								ca.s_other = b.cum[q] + (b.cum[q + 1] - b.cum[q]) * u;
								hit = true;
							}
						}
					}
					if (!hit) {
						// Near miss: paths closer than a car's width.
						double dmin = conflict_distance * conflict_distance;
						for (size_t p = 0; p < a.pts.size(); ++p) {
							for (size_t q = 0; q + 1 < b.pts.size(); ++q) {
								const double u = closest_on(b.pts[q], b.pts[q + 1], a.pts[p]);
								const Vec2 c = b.pts[q] + (b.pts[q + 1] - b.pts[q]) * u;
								const double d2 = (c - a.pts[p]).dot(c - a.pts[p]);
								if (d2 < dmin) {
									dmin = d2;
									ca.s_self = a.cum[p];
									ca.s_other = b.cum[q] + (b.cum[q + 1] - b.cum[q]) * u;
									hit = true;
								}
							}
						}
					}
				}
				if (!hit) continue;
				cb.s_self = ca.s_other;
				cb.s_other = ca.s_self;
				// Priority.
				int8_t prio = 0;
				if (joint) {
					if (ca.merge) {
						const bool a_through = first_into[a.key.b] == i;
						const bool b_through = first_into[b.key.b] == j;
						prio = a_through && !b_through ? 1 : b_through && !a_through ? -1 : 0;
					}
				} else if (rn->control != JunctionControl::AllWayStop && a.leg >= 0 && b.leg >= 0) {
					const Leg &la = g.legs[static_cast<size_t>(a.leg)];
					const Leg &lb = g.legs[static_cast<size_t>(b.leg)];
					const bool major_a = rn->control == JunctionControl::PriorityRoad && rn->is_priority(la.seg);
					const bool major_b = rn->control == JunctionControl::PriorityRoad && rn->is_priority(lb.seg);
					if (major_a != major_b) {
						prio = major_a ? 1 : -1;
					} else if (a.leg != b.leg) {
						const Vec2 in_a = qv(la.dir * -1.0);
						const Vec2 in_b = qv(lb.dir * -1.0);
						const bool b_from_right = qv(lb.dir).dot(in_a.right()) > 0.35;
						const bool a_from_right = qv(la.dir).dot(in_b.right()) > 0.35;
						if (b_from_right && !a_from_right) {
							prio = -1;
						} else if (a_from_right && !b_from_right) {
							prio = 1;
						} else {
							const bool left_a = a.turn == TurnKind::Left || a.turn == TurnKind::UTurn;
							const bool left_b = b.turn == TurnKind::Left || b.turn == TurnKind::UTurn;
							prio = left_a && !left_b ? -1 : left_b && !left_a ? 1 : 0;
						}
					}
				}
				ca.priority = prio;
				cb.priority = static_cast<int8_t>(-prio);
				ca.other = static_cast<int32_t>(j);
				cb.other = static_cast<int32_t>(i);
				a.conflicts.push_back(ca);
				b.conflicts.push_back(cb);
				part.arbitrated = true;
			}
		}
	}
	for (auto it = nodes_.begin(); it != nodes_.end();) {
		it = live_nodes.count(it->first) ? std::next(it) : nodes_.erase(it);
	}

	// --- 3. Assemble dense arrays ------------------------------------------------------
	for (const auto &kv : segments_) {
		const int32_t base = static_cast<int32_t>(out.lanes.size());
		const RoadSegment *seg = map.segment(kv.first);
		for (const NetLane &l : kv.second.lanes) {
			NetLane c = l;
			if (c.left >= 0) c.left += base;
			if (c.right >= 0) c.right += base;
			const bool fwd = c.dir == LaneDir::Forward;
			c.start_node = fwd ? seg->from : seg->to;
			c.end_node = fwd ? seg->to : seg->from;
			out.index_[c.key] = static_cast<int32_t>(out.lanes.size());
			out.max_speed = std::max(out.max_speed, c.speed_limit);
			out.lanes.push_back(std::move(c));
		}
	}
	for (const auto &kv : nodes_) {
		const NodePart &part = kv.second;
		const RoadNode *rn = map.node(kv.first);
		NetJunction j;
		j.node = kv.first;
		j.level = rn->level;
		j.pos = rn->pos;
		j.control = rn->control;
		j.arbitrated = part.arbitrated;
		j.joint = part.joint;
		const int32_t jid = static_cast<int32_t>(out.junctions.size());
		std::vector<int32_t> global(part.connectors.size(), -1);
		for (size_t i = 0; i < part.connectors.size(); ++i) {
			const NetLane &l = part.connectors[i];
			const int32_t from = out.find(LaneKey{ NetLaneKind::Road, l.key.a, 0 });
			const int32_t to = out.find(LaneKey{ NetLaneKind::Road, l.key.b, 0 });
			if (from < 0 || to < 0) continue;
			global[i] = static_cast<int32_t>(out.lanes.size());
			NetLane c = l;
			c.from = from;
			c.to = to;
			c.junction = jid;
			c.type = out.lanes[static_cast<size_t>(to)].type;
			c.next = { to };
			c.prev = { from };
			out.index_[c.key] = global[i];
			out.lanes.push_back(std::move(c));
		}
		for (size_t i = 0; i < part.connectors.size(); ++i) {
			if (global[i] < 0) continue;
			NetLane &c = out.lanes[static_cast<size_t>(global[i])];
			std::vector<Conflict> cs;
			for (Conflict cf : c.conflicts) {
				if (global[static_cast<size_t>(cf.other)] < 0) continue;
				cf.other = global[static_cast<size_t>(cf.other)];
				cs.push_back(cf);
			}
			c.conflicts = std::move(cs);
			j.connectors.push_back(global[i]);
			out.lanes[static_cast<size_t>(c.from)].next.push_back(global[i]);
			out.lanes[static_cast<size_t>(c.to)].prev.push_back(global[i]);
			if (std::find(j.approaches.begin(), j.approaches.end(), c.from) == j.approaches.end()) {
				j.approaches.push_back(c.from);
			}
		}
		std::sort(j.approaches.begin(), j.approaches.end());
		if (j.arbitrated) {
			for (int32_t a : j.approaches) out.lanes[static_cast<size_t>(a)].approach_of = jid;
		}
		out.junction_index_[j.node] = jid;
		out.junctions.push_back(std::move(j));
	}
	// Lanes that only continue by merging (lane drops).
	for (NetLane &l : out.lanes) {
		if (l.kind != NetLaneKind::Road || l.next.empty()) continue;
		bool all = true;
		for (int32_t c : l.next) {
			const NetLane &cn = out.lanes[static_cast<size_t>(c)];
			if (!out.junctions[static_cast<size_t>(cn.junction)].joint) {
				all = false;
				break;
			}
			bool yields = false;
			for (const Conflict &cf : out.lanes[static_cast<size_t>(c)].conflicts) yields |= cf.merge && cf.priority < 0;
			all &= yields;
		}
		l.merge_end = all;
	}
	// Spawn / sink points on road ends.
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		if (!n.spawner.enabled) continue;
		const NodeGeom *g = geom.node(n.id);
		if (!g || g->kind != NodeKind::End) continue;
		NetSpawner sp;
		sp.node = n.id;
		sp.pos = n.pos;
		sp.level = n.level;
		sp.config = n.spawner;
		for (size_t i = 0; i < out.lanes.size(); ++i) {
			NetLane &l = out.lanes[i];
			if (l.kind != NetLaneKind::Road) continue;
			if (l.start_node == n.id && !l.pocket && l.type != LaneType::Bus) {
				sp.spawn_lanes.push_back(static_cast<int32_t>(i));
			}
			if (l.end_node == n.id) {
				sp.sink_lanes.push_back(static_cast<int32_t>(i));
				l.sink = n.spawner.sink;
			}
		}
		out.spawners.push_back(std::move(sp));
	}

	stats_.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// --- Problems -----------------------------------------------------------------------

std::vector<NetProblem> network_problems(const RoadMap &map, const Network &net) {
	std::vector<NetProblem> out;
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		if (n.spawner.enabled && !net.spawner_at(n.id)) {
			NetProblem p;
			p.code = "spawner_not_at_end";
			p.message = "Spawn point is not on a road end, so it is inactive. Move it to the end of a road.";
			p.pos = n.pos;
			p.level = n.level;
			p.node = n.id;
			out.push_back(p);
		}
	}
	// Reachability on the lane graph (connectors and lane changes).
	for (const NetSpawner &src : net.spawners) {
		if (src.config.rate <= 0.0 || src.spawn_lanes.empty()) continue;
		std::vector<char> seen(net.lanes.size(), 0);
		std::vector<int32_t> stack(src.spawn_lanes.begin(), src.spawn_lanes.end());
		for (int32_t l : stack) seen[static_cast<size_t>(l)] = 1;
		while (!stack.empty()) {
			const int32_t l = stack.back();
			stack.pop_back();
			const NetLane &lane = net.lanes[static_cast<size_t>(l)];
			std::vector<int32_t> nb = lane.next;
			if (lane.left >= 0 && !lane.change_left.empty()) nb.push_back(lane.left);
			if (lane.right >= 0 && !lane.change_right.empty()) nb.push_back(lane.right);
			for (int32_t n : nb) {
				if (!seen[static_cast<size_t>(n)]) {
					seen[static_cast<size_t>(n)] = 1;
					stack.push_back(n);
				}
			}
		}
		int unreachable = 0;
		NodeId example = kNoId;
		for (const NetSpawner &dst : net.spawners) {
			if (dst.node == src.node || !dst.config.sink || src.config.weight_to(dst.node) <= 0.0) continue;
			bool ok = false;
			for (int32_t l : dst.sink_lanes) ok |= seen[static_cast<size_t>(l)] != 0;
			if (!ok) {
				++unreachable;
				if (example == kNoId) example = dst.node;
			}
		}
		if (unreachable > 0) {
			NetProblem p;
			p.code = "unreachable";
			p.message = "Cars from this spawn point can't reach " + std::to_string(unreachable) +
					(unreachable == 1 ? " destination" : " destinations") + " (e.g. node " + std::to_string(example) +
					"). Check one-way directions and turn rules.";
			p.pos = src.pos;
			p.level = src.level;
			p.node = src.node;
			out.push_back(p);
		}
	}
	return out;
}

} // namespace tsim
