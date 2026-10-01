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

bool is_sim_lane(LaneType t) { return is_directional(t); } // cars, buses and bikes

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

int32_t Network::stop_index(uint32_t id) const {
	for (size_t i = 0; i < stops.size(); ++i) {
		if (stops[i].id == id) return static_cast<int32_t>(i);
	}
	return -1;
}

int NetJunction::phase_at(int64_t tick, int64_t *into) const {
	const NetSignal &s = signal;
	if (!s.enabled || s.cycle <= 0) return -1;
	int64_t t = (tick + s.offset) % s.cycle;
	for (size_t p = 0; p < s.green.size(); ++p) {
		const int64_t len = s.green[p] + s.amber + s.all_red;
		if (t < len) {
			if (into) *into = t;
			return static_cast<int>(p);
		}
		t -= len;
	}
	return 0;
}

SignalLight NetJunction::light(int32_t movement, int64_t tick) const {
	const NetSignal &s = signal;
	if (movement < 0 || static_cast<size_t>(movement) >= s.movements.size()) return SignalLight::Green;
	int64_t t = 0;
	const int p = phase_at(tick, &t);
	if (p < 0) return SignalLight::Green;
	const size_t ph = static_cast<size_t>(p);
	const uint8_t now = s.state[ph][static_cast<size_t>(movement)];
	if (now == 0) return SignalLight::Red;
	if (t < s.green[ph]) return now == 2 ? SignalLight::GreenYield : SignalLight::Green;
	// Amber and all-red, unless the next phase keeps this movement green.
	const size_t next = (ph + 1) % s.green.size();
	const uint8_t then = s.state[next][static_cast<size_t>(movement)];
	if (then != 0) return then == 2 || now == 2 ? SignalLight::GreenYield : SignalLight::Green;
	return t < s.green[ph] + s.amber ? SignalLight::Amber : SignalLight::Red;
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

int32_t Network::building_index(uint32_t id) const {
	auto it = std::lower_bound(buildings.begin(), buildings.end(), id,
			[](const NetBuilding &b, uint32_t x) { return b.id < x; });
	return it != buildings.end() && it->id == id ? static_cast<int32_t>(it - buildings.begin()) : -1;
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
		h.u(l.ring);
		h.d(l.route_penalty);
		h.u(static_cast<uint64_t>(static_cast<int64_t>(l.movement)));
		h.u(static_cast<uint64_t>(static_cast<int64_t>(l.approach_of)));
	}
	for (const NetJunction &j : junctions) {
		h.u(j.node);
		h.u(static_cast<uint64_t>(j.control));
		h.u(j.arbitrated);
		h.u(j.joint);
		h.u(j.roundabout);
		h.u(static_cast<uint64_t>(j.signal.cycle));
		for (const auto &st : j.signal.state) {
			for (uint8_t v : st) h.u(v);
		}
	}
	for (const NetBay &b : bays) {
		h.u(b.parking_lane);
		h.u(static_cast<uint64_t>(b.lane));
		h.d(b.s);
	}
	for (const NetStop &st : stops) {
		h.u(st.id);
		h.u(static_cast<uint64_t>(st.lane));
		h.d(st.s);
	}
	for (const NetDepot &d : depots) {
		h.u(d.node);
		for (const NetRoute &r : d.routes) {
			h.u(r.id);
			for (int32_t x : r.stops) h.u(static_cast<uint64_t>(x));
		}
	}
	for (const NetSpawner &s : spawners) {
		h.u(s.node);
		for (int32_t l : s.spawn_lanes) h.u(static_cast<uint64_t>(l));
		for (int32_t l : s.sink_lanes) h.u(static_cast<uint64_t>(l));
	}
	// Pedestrian network (M4).
	for (const PedNode &n : ped.nodes) {
		h.v(n.pos);
		h.u(static_cast<uint64_t>(n.level + 8));
	}
	for (const PedEdge &e : ped.edges) {
		h.u(static_cast<uint64_t>(e.a));
		h.u(static_cast<uint64_t>(e.b));
		h.d(e.length);
		h.u(static_cast<uint64_t>(e.kind));
		h.u(static_cast<uint64_t>(e.crossing + 1));
	}
	for (const NetCrossing &c : ped.crossings) {
		h.u(static_cast<uint64_t>(c.kind));
		h.u(c.unmarked);
		h.u(static_cast<uint64_t>(c.junction + 1));
		for (const CrossingSpan &sp : c.spans) {
			h.u(static_cast<uint64_t>(sp.lane));
			h.d(sp.s0);
			h.d(sp.s1);
			h.d(sp.t0);
			h.d(sp.t1);
		}
	}
	for (const PedSpawner &s : ped.spawners) {
		h.u(s.node);
		h.d(s.people);
		for (int32_t e : s.entries) h.u(static_cast<uint64_t>(e));
	}
	for (const PedStop &s : ped.stops) h.u(static_cast<uint64_t>(s.node));
	for (const NetBuilding &b : buildings) {
		h.u(b.id);
		h.u(static_cast<uint64_t>(static_cast<int64_t>(b.type)));
		h.u(static_cast<uint64_t>(static_cast<int64_t>(b.entrance)));
	}
	return h.h.value();
}

void Network::clear() {
	lanes.clear();
	junctions.clear();
	spawners.clear();
	bays.clear();
	stops.clear();
	depots.clear();
	coach_lines.clear();
	main_station = -1;
	ped.clear();
	buildings.clear();
	index_.clear();
	junction_index_.clear();
	max_speed = 13.9;
}

// --- Compiler ------------------------------------------------------------------------

void NetworkCompiler::clear_cache() {
	segments_.clear();
	nodes_.clear();
}

void build_ped_network(const RoadMap &map, const RoadGeometry &geom, Network &out); // ped_network.cpp

void NetworkCompiler::compile(const RoadMap &map, const RoadGeometry &geom, Network &out) {
	const auto t0 = std::chrono::steady_clock::now();
	stats_ = CompileStats{};
	out.clear();

	auto stop_at = [&](NodeId n) {
		const NodeGeom *g = geom.node(n);
		return g && ((g->kind == NodeKind::Junction && g->legs.size() >= 3) || g->kind == NodeKind::Roundabout);
	};

	// --- 1. Segments --------------------------------------------------------------
	std::set<SegmentId> live_segments;
	for (const auto &kv : map.segments()) {
		const RoadSegment &seg = kv.second;
		const SegmentGeom *sg = geom.segment(seg.id);
		if (!sg || seg.kind == SegmentKind::Footpath) continue; // footpaths: pedestrians only (M4)
		live_segments.insert(seg.id);
		const bool stop[2] = { stop_at(seg.from), stop_at(seg.to) };

		Hasher h;
		h.u(seg.level);
		h.u(static_cast<uint64_t>(seg.rise + 8));
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
			if (!is_sim_lane(gl.type) || ns < 2) continue;
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
			l.level = gl.dir == LaneDir::Forward ? seg.level : seg.level_end();
			l.level_end = gl.dir == LaneDir::Forward ? seg.level_end() : seg.level;
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
				// Bikes and motor traffic keep to their own lanes.
				if ((sg->lanes[bi].type == LaneType::Bike) != (sg->lanes[i].type == LaneType::Bike)) continue;
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
			h.d(c.route_penalty);
			for (const Vec2 &p : c.path) h.v(p);
		}
		h.u(rn->roundabout.enabled);
		h.u(rn->roundabout.turbo);
		h.u(static_cast<uint64_t>(rn->roundabout.lanes));
		for (const RingLane &r : g.ring) {
			h.u(r.id);
			for (const Vec2 &p : r.pts) h.v(p);
		}
		// Lane types of the incoming lanes (bike rules).
		for (const Leg &leg : g.legs) {
			const RoadSegment *s = map.segment(leg.seg);
			if (!s) continue;
			for (const LaneSpec &l : s->profile.lanes) h.u(static_cast<uint64_t>(l.type));
		}
		const uint64_t sig = h.h.value();
		NodePart &part = nodes_[g.id];
		++stats_.junctions;
		if (part.sig == sig && sig != 0) continue;
		++stats_.junctions_compiled;
		part.sig = sig;
		part.connectors.clear();
		part.ring.clear();
		part.arbitrated = false;
		part.roundabout = g.kind == NodeKind::Roundabout;

		// Roundabout circulating lanes: ordinary road lanes that belong to no
		// map segment. Their neighbours are the lanes of the same piece.
		std::map<LaneId, int32_t> ring_local;
		for (const RingLane &r : g.ring) {
			NetLane l;
			l.kind = NetLaneKind::Road;
			l.key = LaneKey{ NetLaneKind::Road, r.id, 0 };
			l.type = LaneType::General;
			l.level = l.level_end = g.level;
			l.speed_limit = quantize(std::min(13.9, std::sqrt(2.5 * r.radius)));
			l.segment = 0x80000000u | ((g.id & 0x0FFFFFFFu) << 3) | (static_cast<uint32_t>(r.piece) & 7u);
			l.dir = LaneDir::Forward;
			l.ring = true;
			l.first_sample = 0;
			l.start_node = l.end_node = g.id;
			std::vector<Vec2> pts = r.pts;
			l.pts.clear();
			for (const Vec2 &p : pts) l.pts.push_back(qv(p));
			l.cum.assign(l.pts.size(), 0.0);
			for (size_t k = 1; k < l.pts.size(); ++k) l.cum[k] = l.cum[k - 1] + (l.pts[k] - l.pts[k - 1]).length();
			l.length = std::max(0.01, l.cum.back());
			l.cum.back() = l.length;
			ring_local[r.id] = static_cast<int32_t>(part.ring.size());
			part.ring.push_back(std::move(l));
		}
		const bool turbo = map.node(g.id)->roundabout.turbo && map.node(g.id)->roundabout.lanes >= 2;
		for (const RingLane &r : g.ring) {
			NetLane &l = part.ring[static_cast<size_t>(ring_local[r.id])];
			// Travel-left is the inner lane.
			for (const RingLane &o : g.ring) {
				if (o.piece != r.piece) continue;
				if (o.lane == r.lane + 1) l.left = ring_local[o.id];
				if (o.lane + 1 == r.lane) l.right = ring_local[o.id];
			}
			if (!turbo) {
				if (l.left >= 0) l.change_left.push_back({ 0.0, l.length });
				if (l.right >= 0) l.change_right.push_back({ 0.0, l.length });
			}
		}

		std::set<std::pair<LaneId, LaneId>> seen;
		std::vector<char> from_bike;
		for (const Connector &c : g.connectors) {
			if (!seen.insert({ c.from_lane, c.to_lane }).second) continue;
			const RoadSegment *fs = map.segment(c.from_seg);
			const RoadSegment *ts = map.segment(c.to_seg);
			if ((!fs && !is_ring_lane(c.from_lane)) || (!ts && !is_ring_lane(c.to_lane))) continue;
			NetLane l;
			l.kind = NetLaneKind::Connector;
			l.key = LaneKey{ NetLaneKind::Connector, c.from_lane, c.to_lane };
			l.level = l.level_end = g.level;
			l.node = g.id;
			l.turn = c.turn;
			set_polyline(l, c.path);
			l.route_penalty = c.route_penalty;
			double limit = std::min(fs ? fs->speed_limit : 13.9, ts ? ts->speed_limit : 13.9);
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
			if (fs) {
				const int end = arriving_end(*fs, c.from_lane);
				for (size_t k = 0; k < g.legs.size(); ++k) {
					if (g.legs[k].seg == c.from_seg && g.legs[k].at_start == (end == 0)) l.leg = static_cast<int>(k);
				}
			}
			bool bike = false;
			if (fs) {
				for (const LaneSpec &ls : fs->profile.lanes) bike |= ls.id == c.from_lane && ls.type == LaneType::Bike;
			}
			from_bike.push_back(bike ? 1 : 0);
			part.connectors.push_back(std::move(l));
		}
		// Merges at continuations and tapers: the first connector into a lane
		// is the through lane; others merge into it and yield.
		const bool joint = g.kind == NodeKind::Continuation || g.kind == NodeKind::Taper;
		part.joint = joint;
		// Signal movements: all connectors from one leg into another.
		part.movements.clear();
		if (rn->control == JunctionControl::Signal && !joint && !part.roundabout) {
			for (NetLane &l : part.connectors) {
				SegmentId fs = kNoId, ts = kNoId;
				for (const Connector &c : g.connectors) {
					if (c.from_lane == l.key.a && c.to_lane == l.key.b) {
						fs = c.from_seg;
						ts = c.to_seg;
						break;
					}
				}
				const std::pair<SegmentId, SegmentId> mv{ fs, ts };
				auto it = std::find(part.movements.begin(), part.movements.end(), mv);
				if (it == part.movements.end()) {
					part.movements.push_back(mv);
					it = part.movements.end() - 1;
				}
				l.movement = static_cast<int32_t>(it - part.movements.begin());
			}
		}
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
				const bool bike_a = from_bike[i] != 0, bike_b = from_bike[j] != 0;
				auto car_turn = [](const NetLane &x, bool bike) {
					return !bike && (x.turn == TurnKind::Left || x.turn == TurnKind::Right);
				};
				if (part.roundabout) {
					// Circulating traffic has priority over entries and bypasses.
					const bool ring_a = is_ring_lane(a.key.a), ring_b = is_ring_lane(b.key.a);
					prio = ring_a && !ring_b ? 1 : ring_b && !ring_a ? -1 : 0;
				} else if (!joint && bike_a != bike_b &&
						((bike_a && a.turn == TurnKind::Straight && car_turn(b, bike_b)) ||
								(bike_b && b.turn == TurnKind::Straight && car_turn(a, bike_a)))) {
					// Turning cars yield to bikes going straight.
					prio = bike_a ? 1 : -1;
				} else if (joint) {
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
		const int32_t base = static_cast<int32_t>(out.lanes.size());
		for (const NetLane &l : kv.second.ring) {
			NetLane c = l;
			if (c.left >= 0) c.left += base;
			if (c.right >= 0) c.right += base;
			out.index_[c.key] = static_cast<int32_t>(out.lanes.size());
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
		j.roundabout = part.roundabout;
		if (!part.movements.empty()) {
			// Fixed-time program in ticks (10 Hz).
			NetSignal &sg = j.signal;
			const SignalPlan &plan = rn->signal;
			auto ticks = [](double sec) { return static_cast<int64_t>(std::llround(std::max(0.0, sec) * 10.0)); };
			sg.enabled = !plan.phases.empty();
			sg.movements = part.movements;
			sg.amber = ticks(plan.amber);
			sg.all_red = ticks(plan.all_red);
			sg.right_on_red.assign(part.movements.size(), 0);
			for (size_t m = 0; m < part.movements.size(); ++m) {
				for (SegmentId leg : plan.right_on_red) sg.right_on_red[m] |= part.movements[m].first == leg ? 1 : 0;
			}
			for (const SignalPhase &ph : plan.phases) {
				sg.green.push_back(std::max<int64_t>(10, ticks(ph.green)));
				std::vector<uint8_t> st(part.movements.size(), 0);
				for (const SignalMovement &mv : ph.moves) {
					for (size_t m = 0; m < part.movements.size(); ++m) {
						if (part.movements[m].first == mv.from && part.movements[m].second == mv.to) st[m] = mv.permissive ? 2 : 1;
					}
				}
				sg.state.push_back(std::move(st));
				sg.walk.push_back(ph.walk);
				sg.cycle += sg.green.back() + sg.amber + sg.all_red;
			}
			sg.offset = sg.cycle > 0 ? ((ticks(plan.offset) % sg.cycle) + sg.cycle) % sg.cycle : 0;
			if (sg.enabled) j.arbitrated = true;
		}
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
			if (l.start_node == n.id && !l.pocket && l.type != LaneType::Bus && l.type != LaneType::Bike) {
				sp.spawn_lanes.push_back(static_cast<int32_t>(i));
			}
			if (l.start_node == n.id && l.type == LaneType::Bike) sp.bike_lanes.push_back(static_cast<int32_t>(i));
			if (l.end_node == n.id) {
				sp.sink_lanes.push_back(static_cast<int32_t>(i));
				l.sink = n.spawner.sink;
			}
		}
		if (sp.bike_lanes.empty()) {
			// No bike lane: bikes ride at the kerb of the rightmost lane.
			for (int32_t l : sp.spawn_lanes) {
				if (out.lanes[static_cast<size_t>(l)].right < 0) sp.bike_lanes.push_back(l);
			}
		}
		out.spawners.push_back(std::move(sp));
	}

	// --- 4. Transit and parking (M3) ------------------------------------------------------
	std::map<SegmentId, std::vector<int32_t>> seg_lanes;
	for (size_t i = 0; i < out.lanes.size(); ++i) {
		const NetLane &l = out.lanes[i];
		if (l.kind == NetLaneKind::Road && !l.ring) seg_lanes[l.segment].push_back(static_cast<int32_t>(i));
	}
	// The kerb-side motor lane of one direction of a road.
	auto kerb_lane = [&](SegmentId seg, LaneDir dir) {
		for (int32_t i : seg_lanes[seg]) {
			const NetLane &l = out.lanes[static_cast<size_t>(i)];
			if (l.dir == dir && !l.pocket && l.type != LaneType::Bike && l.right < 0) return i;
		}
		return -1;
	};
	// Distance along a road lane level with a centreline station.
	auto lane_s = [&](int32_t lane, const SegmentGeom &sg, double station) {
		const NetLane &l = out.lanes[static_cast<size_t>(lane)];
		const size_t ns = sg.s.size();
		const bool fwd = l.dir == LaneDir::Forward;
		double best = 0.0, dmin = 1e300;
		for (size_t j = 0; j < l.pts.size(); ++j) {
			const size_t tj = static_cast<size_t>(l.first_sample) + j;
			if (tj >= ns) break;
			const size_t k = fwd ? tj : ns - 1 - tj;
			const double d = std::fabs(sg.s[k] - station);
			if (d < dmin) {
				dmin = d;
				best = l.cum[j];
			}
		}
		return quantize(best);
	};
	for (const auto &kv : map.segments()) {
		const RoadSegment &seg = kv.second;
		const SegmentGeom *sg = geom.segment(seg.id);
		if (!sg) continue;
		for (const ParkingBay &b : sg->bays) {
			const int32_t lane = kerb_lane(seg.id, b.list_right ? LaneDir::Forward : LaneDir::Backward);
			if (lane < 0) continue;
			NetBay nb;
			nb.parking_lane = b.lane;
			nb.index = b.index;
			nb.lane = lane;
			nb.s = lane_s(lane, *sg, b.s);
			nb.pos = qv(b.pos);
			nb.dir = qv(b.dir);
			nb.style = b.style;
			out.bays.push_back(nb);
		}
		for (const BusStop &st : seg.stops) {
			const int32_t lane = kerb_lane(seg.id, st.side);
			if (lane < 0) continue;
			NetStop ns;
			ns.id = st.id;
			ns.segment = seg.id;
			ns.kind = st.kind;
			ns.name = st.name;
			ns.lane = lane;
			const double len = st.kind == StopKind::MainStation ? 15.0 * std::max(1, st.bays) + 5.0 : 18.0;
			const double lo = sg->trim[0] + 0.5 * len + 2.0;
			const double hi = std::max(lo, sg->length - sg->trim[1] - 0.5 * len - 2.0);
			// The bus stops with its front at the far end of the box.
			const double station = std::clamp(st.u * sg->length, lo, hi) + (st.side == LaneDir::Forward ? 0.4 : -0.4) * len;
			ns.s = lane_s(lane, *sg, station);
			ns.bays = st.kind == StopKind::MainStation ? std::max(1, st.bays) : 1;
			const Pose p = out.pose(lane, ns.s);
			const double out_off = st.kind == StopKind::Kerbside ? 0.0 : 3.2;
			ns.pos = qv(p.pos + p.dir.right() * out_off);
			ns.dir = qv(p.dir);
			out.stops.push_back(ns);
			if (st.kind == StopKind::MainStation && out.main_station < 0) {
				out.main_station = static_cast<int32_t>(out.stops.size() - 1);
			}
		}
	}
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		if (n.depot.enabled) {
			const NodeGeom *g = geom.node(n.id);
			if (g && g->kind == NodeKind::End) {
				NetDepot d;
				d.node = n.id;
				d.name = n.depot.name;
				d.pos = n.pos;
				d.level = n.level;
				d.capacity = n.depot.capacity;
				for (size_t i = 0; i < out.lanes.size(); ++i) {
					const NetLane &l = out.lanes[i];
					if (l.kind != NetLaneKind::Road || l.ring || l.type == LaneType::Bike) continue;
					if (l.start_node == n.id && !l.pocket) d.spawn_lanes.push_back(static_cast<int32_t>(i));
					if (l.end_node == n.id) d.sink_lanes.push_back(static_cast<int32_t>(i));
				}
				for (const BusRoute &r : n.depot.routes) {
					NetRoute nr;
					nr.id = r.id;
					nr.name = r.name;
					nr.color = r.color;
					nr.headway = r.headway;
					nr.loop = r.loop;
					for (uint32_t sid : r.stops) {
						const int32_t si = out.stop_index(sid);
						if (si >= 0) nr.stops.push_back(si);
					}
					d.routes.push_back(std::move(nr));
				}
				out.depots.push_back(std::move(d));
			}
		}
		if (n.spawner.enabled && out.spawner_at(n.id)) {
			for (const CoachLine &c : n.spawner.coaches) {
				NetCoachLine cl;
				cl.id = c.id;
				cl.entry = n.id;
				cl.exit = c.exit;
				cl.per_hour = c.per_hour;
				cl.dwell = c.dwell;
				out.coach_lines.push_back(cl);
			}
		}
	}

	// --- 5. Pedestrian network (M4) ---------------------------------------------------------
	build_ped_network(map, geom, out);

	stats_.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// --- Problems -----------------------------------------------------------------------

namespace {

// Lanes reachable from `starts` on the lane graph (connectors and lane changes).
std::vector<char> reachable(const Network &net, const std::vector<int32_t> &starts) {
	std::vector<char> seen(net.lanes.size(), 0);
	std::vector<int32_t> stack;
	for (int32_t l : starts) {
		if (l >= 0 && !seen[static_cast<size_t>(l)]) {
			seen[static_cast<size_t>(l)] = 1;
			stack.push_back(l);
		}
	}
	while (!stack.empty()) {
		const int32_t l = stack.back();
		stack.pop_back();
		const NetLane &lane = net.lanes[static_cast<size_t>(l)];
		std::vector<int32_t> nb = lane.next;
		if (lane.left >= 0 && !lane.change_left.empty()) nb.push_back(lane.left);
		if (lane.right >= 0 && !lane.change_right.empty()) nb.push_back(lane.right);
		for (int32_t n : nb) {
			if (net.lanes[static_cast<size_t>(n)].type == LaneType::Bike) continue; // buses and coaches
			if (!seen[static_cast<size_t>(n)]) {
				seen[static_cast<size_t>(n)] = 1;
				stack.push_back(n);
			}
		}
	}
	return seen;
}

void transit_problems(const RoadMap &map, const Network &net, std::vector<NetProblem> &out) {
	auto add = [&](const char *code, const std::string &msg, Vec2 pos, int level, NodeId node) {
		NetProblem p;
		p.code = code;
		p.message = msg;
		p.pos = pos;
		p.level = level;
		p.node = node;
		out.push_back(p);
	};
	auto stop_pos = [&](const RoadSegment &seg, double u) {
		const RoadNode *a = map.node(seg.from);
		const RoadNode *b = map.node(seg.to);
		return a && b ? a->pos + (b->pos - a->pos) * u : Vec2{};
	};
	int main_stations = 0;
	for (const auto &kv : map.segments()) {
		const RoadSegment &seg = kv.second;
		for (const BusStop &st : seg.stops) {
			if (st.kind == StopKind::MainStation) ++main_stations;
			if (net.stop_index(st.id) < 0) {
				add("stop_no_lane", "Stop \"" + st.name + "\" has no lane for buses going that way. Put it on the other side, or on a road with traffic in that direction.",
						stop_pos(seg, st.u), seg.level, kNoId);
			}
		}
	}
	if (main_stations > 1) add("main_stations", "More than one main station: coaches only use the first.", Vec2{}, 0, kNoId);
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		if (!n.depot.enabled) continue;
		const NetDepot *d = nullptr;
		for (const NetDepot &x : net.depots) {
			if (x.node == n.id) d = &x;
		}
		if (!d) {
			add("depot_not_at_end", "Depot is not on a road end, so no buses leave it. Move it to the end of a road.", n.pos, n.level, n.id);
			continue;
		}
		for (const NetRoute &r : d->routes) {
			// Depot -> each stop in order -> depot, on lanes a bus may use.
			std::vector<int32_t> from = d->spawn_lanes;
			std::vector<int32_t> seq = r.stops;
			if (r.loop && seq.size() > 1) seq.push_back(seq.front());
			std::string broken;
			for (int32_t si : seq) {
				const NetStop &st = net.stops[static_cast<size_t>(si)];
				if (!reachable(net, from)[static_cast<size_t>(st.lane)]) {
					broken = "can't reach stop \"" + st.name + "\"";
					break;
				}
				from = { st.lane };
			}
			if (broken.empty() && !d->sink_lanes.empty()) {
				const std::vector<char> seen = reachable(net, from);
				bool back = false;
				for (int32_t l : d->sink_lanes) back |= seen[static_cast<size_t>(l)] != 0;
				if (!back) broken = "can't get back to the depot after its last stop";
			}
			if (r.stops.empty()) broken = "has no stops";
			if (!broken.empty()) add("route_broken", "Bus route \"" + r.name + "\" " + broken + ".", n.pos, n.level, n.id);
		}
	}
	for (const NetCoachLine &c : net.coach_lines) {
		const NetSpawner *sp = net.spawner_at(c.exit);
		const NetSpawner *in = net.spawner_at(c.entry);
		if (!sp || !sp->config.sink) {
			add("coach_exit", "A coach line from here has no exit: pick a spawn point where vehicles may leave.", in ? in->pos : Vec2{},
					in ? in->level : 0, c.entry);
		}
		if (net.main_station < 0) {
			add("no_main_station", "Coach lines need a main station (a stop of kind main station): coaches will drive straight through.",
					in ? in->pos : Vec2{}, in ? in->level : 0, c.entry);
		}
	}
}

} // namespace

std::vector<NetProblem> network_problems(const RoadMap &map, const Network &net) {
	std::vector<NetProblem> out;
	transit_problems(map, net, out);
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		bool people_only = false; // a spawn point at a footpath end (M4)
		for (const PedSpawner &ps : net.ped.spawners) people_only |= ps.node == n.id && !ps.road;
		if (n.spawner.enabled && !net.spawner_at(n.id) && !people_only) {
			NetProblem p;
			p.code = "spawner_not_at_end";
			p.message = "Spawn point is not on a road end, so it is inactive. Move it to the end of a road.";
			p.pos = n.pos;
			p.level = n.level;
			p.node = n.id;
			out.push_back(p);
		}
	}
	// Buildings (M5): a door on the pedestrian network, and homes people can
	// reach from the main station (immigrants arrive by coach).
	if (!net.buildings.empty()) {
		std::vector<char> from_station(net.ped.nodes.size(), 0);
		int32_t platform = -1;
		for (const PedStop &ps : net.ped.stops) {
			if (ps.stop == net.main_station) platform = ps.node;
		}
		if (platform >= 0) {
			std::vector<int32_t> stack = { platform };
			from_station[static_cast<size_t>(platform)] = 1;
			while (!stack.empty()) {
				const int32_t n = stack.back();
				stack.pop_back();
				for (int32_t ei : net.ped.adj[static_cast<size_t>(n)]) {
					const PedEdge &e = net.ped.edges[static_cast<size_t>(ei)];
					const int32_t m = e.a == n ? e.b : e.a;
					if (!from_station[static_cast<size_t>(m)]) {
						from_station[static_cast<size_t>(m)] = 1;
						stack.push_back(m);
					}
				}
			}
		}
		bool homes = false;
		for (const NetBuilding &b : net.buildings) {
			if (b.type < 0) continue;
			homes |= b.kind == BuildingKind::Home;
			NetProblem p;
			p.pos = b.centre;
			p.level = b.level;
			p.building = b.id;
			if (b.entrance < 0) {
				p.code = "building_no_door";
				p.message = "No sidewalk or path in front of this building, so nobody can get in. Put it beside a street with a sidewalk.";
				out.push_back(p);
			} else if (b.kind == BuildingKind::Home && platform >= 0 && !from_station[static_cast<size_t>(b.entrance)]) {
				p.code = "home_unreachable";
				p.message = "Nobody can walk to this home from the main station, so no one will move in.";
				out.push_back(p);
			}
		}
		if (homes && net.main_station < 0) {
			NetProblem p;
			p.code = "no_main_station_city";
			p.message = "Homes but no main station: immigrants and visitors arrive by coach. Add a main station stop.";
			p.pos = net.buildings.front().centre;
			p.level = net.buildings.front().level;
			out.push_back(p);
		} else if (homes && net.coach_lines.empty()) {
			NetProblem p;
			p.code = "no_coaches_city";
			p.message = "No coach line serves the main station, so no one can move in. Add a coach line at a spawn point.";
			p.pos = net.stops[static_cast<size_t>(net.main_station)].pos;
			p.level = net.buildings.front().level;
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
			bool cars_end_here = dst.sink_lanes.empty(); // not only a bike path
			for (int32_t l : dst.sink_lanes) cars_end_here |= net.lanes[static_cast<size_t>(l)].type != LaneType::Bike;
			if (!cars_end_here) continue;
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
