// Pedestrian network compile (M4): sidewalks, corners, crossings, paths, stop
// platforms and spawn points as a graph people walk on, plus where car lanes
// pass over each crossing.
//
// Positions come from the road geometry (trig) and are snapped to the
// 1/1024 m grid; lengths are then derived from the snapped points with sqrt
// only, like the rest of the network, so the graph is identical everywhere.
#include "tsim/network.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace tsim {

void build_ped_network(const RoadMap &map, const RoadGeometry &geom, Network &out);

namespace {

Vec2 qv(Vec2 v) { return Vec2{ quantize(v.x), quantize(v.y) }; }

// A walkable line along a road side or a path: sample points with their
// centreline station, and the ped nodes placed along it.
struct Chain {
	SegmentId seg = kNoId;
	int side = -1; // 0 profile-left sidewalk, 1 profile-right, -1 a path's centreline
	std::vector<Vec2> pts; // quantized
	std::vector<double> st; // centreline station per point (increasing)
	std::vector<double> cum; // walking distance per point
	double length = 0.0; // centreline length (for the level switch on ramps)
	int level0 = 0, level1 = 0; // at the start / end
	bool stairs = false;
	std::vector<std::pair<double, int32_t>> stations; // (station, ped node), sorted

	double clamp_st(double s) const { return std::clamp(s, st.front(), st.back()); }
	size_t seg_at(double s) const {
		size_t k = static_cast<size_t>(std::upper_bound(st.begin(), st.end(), s) - st.begin());
		k = k == 0 ? 0 : k - 1;
		return std::min(k, pts.size() - 2);
	}
	Vec2 pos_at(double s) const {
		s = clamp_st(s);
		const size_t k = seg_at(s);
		const double span = st[k + 1] - st[k];
		const double f = span > 1e-9 ? (s - st[k]) / span : 0.0;
		return pts[k] + (pts[k + 1] - pts[k]) * f;
	}
	double walk_at(double s) const {
		s = clamp_st(s);
		const size_t k = seg_at(s);
		const double span = st[k + 1] - st[k];
		const double f = span > 1e-9 ? (s - st[k]) / span : 0.0;
		return cum[k] + (cum[k + 1] - cum[k]) * f;
	}
	int level_at(double s) const { return s < 0.5 * length ? level0 : level1; }
	// Closest station to a point (and its distance).
	double project(Vec2 p, double *dist) const {
		double best = 1e300, bs = st.front();
		for (size_t k = 0; k + 1 < pts.size(); ++k) {
			const Vec2 d = pts[k + 1] - pts[k];
			const double l2 = d.dot(d);
			const double t = l2 > 1e-12 ? std::clamp((p - pts[k]).dot(d) / l2, 0.0, 1.0) : 0.0;
			const Vec2 q = pts[k] + d * t;
			const double dd = (q - p).length();
			if (dd < best) {
				best = dd;
				bs = st[k] + (st[k + 1] - st[k]) * t;
			}
		}
		if (dist) *dist = best;
		return bs;
	}
};

struct Builder {
	const RoadMap &map;
	const RoadGeometry &geom;
	Network &net;
	PedGraph &g;
	std::vector<Chain> chains;
	std::map<std::pair<SegmentId, int>, size_t> chain_of; // (segment, side) -> chain
	std::map<NodeId, int32_t> path_node; // footpath joints and ends

	Builder(const RoadMap &m, const RoadGeometry &ge, Network &n) : map(m), geom(ge), net(n), g(n.ped) {}

	int32_t add_node(Vec2 p, int level) {
		g.nodes.push_back(PedNode{ qv(p), level });
		g.adj.emplace_back();
		return static_cast<int32_t>(g.nodes.size() - 1);
	}
	int32_t add_edge(int32_t a, int32_t b, double length, PedEdgeKind kind, int32_t crossing = -1, int rise = 0) {
		if (a < 0 || b < 0 || a == b) return -1;
		PedEdge e;
		e.a = a;
		e.b = b;
		e.length = std::max(0.1, quantize(length));
		e.kind = kind;
		e.crossing = crossing;
		e.rise = rise;
		g.edges.push_back(e);
		const int32_t id = static_cast<int32_t>(g.edges.size() - 1);
		g.adj[static_cast<size_t>(a)].push_back(id);
		g.adj[static_cast<size_t>(b)].push_back(id);
		return id;
	}
	double dist(int32_t a, int32_t b) const {
		return (g.nodes[static_cast<size_t>(a)].pos - g.nodes[static_cast<size_t>(b)].pos).length();
	}
	int32_t walk_link(int32_t a, int32_t b) { return add_edge(a, b, dist(a, b), PedEdgeKind::Walk); }

	// The ped node at a chain station (made if needed).
	int32_t node_at(Chain &c, double s) {
		s = c.clamp_st(s);
		for (const auto &st : c.stations) {
			if (std::fabs(st.first - s) < 0.5) return st.second;
		}
		const int32_t n = add_node(c.pos_at(s), c.level_at(s));
		c.stations.push_back({ s, n });
		std::sort(c.stations.begin(), c.stations.end());
		return n;
	}
	Chain *chain(SegmentId seg, int side) {
		auto it = chain_of.find({ seg, side });
		return it == chain_of.end() ? nullptr : &chains[it->second];
	}

	// Carriageway edges (kerb to kerb) at a centreline station of a road.
	bool kerbs_at(const SegmentGeom &sg, double s, Vec2 &a, Vec2 &b) const {
		if (sg.s.empty()) return false;
		size_t k = static_cast<size_t>(std::lower_bound(sg.s.begin(), sg.s.end(), s) - sg.s.begin());
		k = std::min(k, sg.s.size() - 1);
		double lo = 1e300, hi = -1e300;
		for (size_t i = 0; i < sg.lanes.size(); ++i) {
			if (sg.lanes[i].type == LaneType::Sidewalk) continue;
			lo = std::min(lo, sg.edge(k, i).first);
			hi = std::max(hi, sg.edge(k, i).second);
		}
		if (lo > hi) return false;
		a = qv(sg.at(k, lo));
		b = qv(sg.at(k, hi));
		return true;
	}

	bool fenced(const RoadSegment &seg, double u_lo, double u_hi) const {
		for (const Fence &f : seg.fences) {
			if (f.u1 > u_lo && f.u0 < u_hi) return true;
		}
		return false;
	}

	void build_chains() {
		for (const auto &kv : geom.segments()) {
			const SegmentGeom &sg = kv.second;
			const RoadSegment *seg = map.segment(sg.id);
			if (!seg || sg.s.size() < 2) continue;
			auto make = [&](int side, size_t lane, bool centre) {
				Chain c;
				c.seg = sg.id;
				c.side = side;
				c.length = sg.length;
				c.level0 = seg->level;
				c.level1 = seg->level_end();
				c.stairs = seg->stairs;
				for (size_t k = 0; k < sg.s.size(); ++k) {
					double off = 0.0;
					if (!centre) {
						const auto &e = sg.edge(k, lane);
						off = 0.5 * (e.first + e.second);
					}
					c.pts.push_back(qv(sg.at(k, off)));
					c.st.push_back(sg.s[k]);
				}
				c.cum.assign(c.pts.size(), 0.0);
				for (size_t k = 1; k < c.pts.size(); ++k) c.cum[k] = c.cum[k - 1] + (c.pts[k] - c.pts[k - 1]).length();
				chain_of[{ sg.id, side }] = chains.size();
				chains.push_back(std::move(c));
			};
			if (seg->kind == SegmentKind::Road) {
				const size_t nl = sg.lanes.size();
				if (nl > 0 && sg.lanes.front().type == LaneType::Sidewalk) make(0, 0, false);
				if (nl > 1 && sg.lanes.back().type == LaneType::Sidewalk) make(1, nl - 1, false);
			} else if (is_walkable(seg->kind)) {
				make(-1, 0, true);
			}
		}
		// Chain ends exist as nodes; path ends at map nodes are shared.
		for (Chain &c : chains) {
			const RoadSegment *seg = map.segment(c.seg);
			for (int e = 0; e < 2; ++e) {
				const double s = e == 0 ? c.st.front() : c.st.back();
				if (c.side < 0 && seg->kind == SegmentKind::Footpath) {
					const NodeId nid = e == 0 ? seg->from : seg->to;
					auto it = path_node.find(nid);
					if (it == path_node.end()) {
						const int32_t n = add_node(c.pos_at(s), c.level_at(s));
						path_node[nid] = n;
						c.stations.push_back({ s, n });
					} else {
						c.stations.push_back({ s, it->second });
					}
					std::sort(c.stations.begin(), c.stations.end());
				} else {
					node_at(c, s);
				}
			}
		}
	}

	// The chain on one side of a leg as seen from its node, and its station there.
	struct LegSide {
		Chain *chain = nullptr;
		double station = 0.0;
	};
	LegSide leg_side(const Leg &leg, bool leg_right) {
		LegSide out;
		const RoadSegment *seg = map.segment(leg.seg);
		if (!seg) return out;
		Chain *c = nullptr;
		if (is_walkable(seg->kind)) {
			c = chain(leg.seg, -1);
		} else {
			// Looking away from the node: at the start, leg-right is profile-right.
			const int side = leg.at_start == leg_right ? 1 : 0;
			c = chain(leg.seg, side);
		}
		if (!c) return out;
		out.chain = c;
		out.station = leg.at_start ? c->st.front() : c->st.back();
		return out;
	}

	void corners() {
		for (const auto &kv : geom.nodes()) {
			const NodeGeom &ng = kv.second;
			const bool joins = ng.kind == NodeKind::Junction || ng.kind == NodeKind::Roundabout ||
					ng.kind == NodeKind::Continuation || ng.kind == NodeKind::Taper;
			if (!joins || ng.legs.size() < 2) continue;
			const size_t deg = ng.legs.size();
			auto has_walk = [&](const Leg &l) {
				return leg_side(l, true).chain != nullptr || leg_side(l, false).chain != nullptr;
			};
			for (size_t i = 0; i < deg; ++i) {
				const Leg &a = ng.legs[i];
				if (!has_walk(a)) continue;
				// The next leg anticlockwise that people can walk along (bike-only legs are stepped over).
				size_t j = (i + 1) % deg;
				while (j != i && !has_walk(ng.legs[j])) j = (j + 1) % deg;
				if (j == i) continue;
				const Leg &b = ng.legs[j];
				LegSide ra = leg_side(a, true), lb = leg_side(b, false);
				if (!ra.chain || !lb.chain) continue;
				const int32_t na = node_at(*ra.chain, ra.station);
				const int32_t nb = node_at(*lb.chain, lb.station);
				if (na == nb) continue;
				double len = dist(na, nb) * 1.15; // round the kerb
				if (ng.kind == NodeKind::Roundabout) {
					const double r = ng.ring_radius + 3.0;
					const Vec2 pa = g.nodes[static_cast<size_t>(na)].pos - ng.pos, pb = g.nodes[static_cast<size_t>(nb)].pos - ng.pos;
					double th = std::fabs(std::atan2(cross(pa, pb), pa.dot(pb)));
					len = std::max(len, th * r);
				}
				add_edge(na, nb, len, PedEdgeKind::Walk);
			}
		}
	}

	int32_t add_crossing(NetCrossing c, int32_t ka, int32_t kb, double refuge_t0, double refuge_t1) {
		c.a = qv(c.a);
		c.b = qv(c.b);
		c.length = std::max(1.0, (c.b - c.a).length());
		c.clear_ticks = std::clamp<int64_t>(static_cast<int64_t>(std::ceil(c.length / 1.2 * 10.0)), 40, 250);
		const int32_t ci = static_cast<int32_t>(g.crossings.size());
		g.crossings.push_back(c);
		NetCrossing &cr = g.crossings.back();
		if (refuge_t0 >= 0.0 && refuge_t1 > refuge_t0) {
			const double tm = 0.5 * (refuge_t0 + refuge_t1);
			const Vec2 mid = cr.a + (cr.b - cr.a) * (tm / cr.length);
			const int32_t km = add_node(mid, cr.level);
			cr.edges.push_back(add_edge(ka, km, tm, PedEdgeKind::Crossing, ci));
			cr.edges.push_back(add_edge(km, kb, cr.length - tm, PedEdgeKind::Crossing, ci));
		} else {
			cr.edges.push_back(add_edge(ka, kb, cr.length, PedEdgeKind::Crossing, ci));
		}
		return ci;
	}

	// A crossing between the two sides of a road at a station: kerb nodes at a
	// and b, linked to the sidewalks.
	void cross_road(const SegmentGeom &sg, NetCrossing c, Chain *left, double sl, Chain *right, double sr,
			double rt0 = -1.0, double rt1 = -1.0) {
		if (!left || !right) return;
		const int32_t ka = add_node(c.a, c.level);
		const int32_t kb = add_node(c.b, c.level);
		walk_link(node_at(*left, sl), ka);
		walk_link(kb, node_at(*right, sr));
		(void)sg;
		add_crossing(c, ka, kb, rt0, rt1);
	}

	void leg_crossings() {
		std::map<std::pair<SegmentId, int>, const CrossingGeom *> marked;
		for (const CrossingGeom &cg : geom.crossings()) {
			if (cg.end >= 0) marked[{ cg.seg, cg.end }] = &cg;
		}
		for (const auto &kv : geom.nodes()) {
			const NodeGeom &ng = kv.second;
			const bool junction = (ng.kind == NodeKind::Junction && ng.legs.size() >= 3) || ng.kind == NodeKind::Roundabout;
			if (!junction) continue;
			const RoadNode *rn = map.node(ng.id);
			const int32_t ji = net.junction_at(ng.id);
			for (const Leg &leg : ng.legs) {
				const RoadSegment *seg = map.segment(leg.seg);
				const SegmentGeom *sg = geom.segment(leg.seg);
				if (!seg || !sg || seg->kind != SegmentKind::Road) continue;
				Chain *left = chain(leg.seg, 0), *right = chain(leg.seg, 1);
				if (!left || !right) continue;
				const int e = leg.at_start ? 0 : 1;
				const double s_end = e == 0 ? left->st.front() : left->st.back();
				const double r_end = e == 0 ? right->st.front() : right->st.back();
				NetCrossing c;
				c.seg = leg.seg;
				c.end = e;
				c.node = ng.id;
				c.level = ng.level;
				auto it = marked.find({ leg.seg, e });
				if (it != marked.end()) {
					const CrossingGeom &cg = *it->second;
					c.kind = cg.kind;
					if (c.kind == CrossingKind::Signal && !(rn && rn->control == JunctionControl::Signal && ji >= 0)) {
						c.kind = CrossingKind::Zebra;
					}
					if (c.kind == CrossingKind::Signal) c.junction = ji;
					c.a = cg.a;
					c.b = cg.b;
					cross_road(*sg, c, left, s_end, right, r_end, cg.refuge_t0, cg.refuge_t1);
				} else {
					// Unmarked: people may cross at the stop line, unless a fence is in the way.
					const double u = e == 0 ? 0.0 : 1.0;
					if (fenced(*seg, u - 0.15, u + 0.15)) continue;
					if (!kerbs_at(*sg, e == 0 ? sg->s.front() : sg->s.back(), c.a, c.b)) continue;
					c.kind = CrossingKind::Uncontrolled;
					c.unmarked = true;
					cross_road(*sg, c, left, s_end, right, r_end);
				}
			}
		}
	}

	void mid_block() {
		for (const CrossingGeom &cg : geom.crossings()) {
			if (cg.end >= 0) continue;
			const SegmentGeom *sg = geom.segment(cg.seg);
			Chain *left = chain(cg.seg, 0), *right = chain(cg.seg, 1);
			if (!sg || !left || !right) continue;
			NetCrossing c;
			c.seg = cg.seg;
			c.end = -1;
			c.id = cg.id;
			c.kind = cg.kind;
			c.push_button = cg.kind == CrossingKind::Signal;
			c.level = cg.level;
			c.a = cg.a;
			c.b = cg.b;
			cross_road(*sg, c, left, cg.s, right, cg.s, cg.refuge_t0, cg.refuge_t1);
		}
		// Informal crossings on smaller roads every 60 m, kept away from marked
		// crossings and fences.
		for (const auto &kv : geom.segments()) {
			const SegmentGeom &sg = kv.second;
			const RoadSegment *seg = map.segment(sg.id);
			if (!seg || seg->kind != SegmentKind::Road || seg->is_ramp() || seg->profile.median == MedianType::Raised) continue;
			if (seg->profile.count(LaneDir::Forward) + seg->profile.count(LaneDir::Backward) > 4) continue;
			Chain *left = chain(sg.id, 0), *right = chain(sg.id, 1);
			if (!left || !right || sg.s.size() < 2) continue;
			const double s0 = sg.s.front() + 25.0, s1 = sg.s.back() - 25.0;
			for (double s = s0; s <= s1; s += 60.0) {
				bool near = false;
				for (const CrossingGeom &cg : geom.crossings()) {
					if (cg.seg == sg.id && cg.end < 0 && std::fabs(cg.s - s) < 25.0) near = true;
				}
				const double u = sg.length > 0.0 ? s / sg.length : 0.0;
				const double du = sg.length > 0.0 ? 5.0 / sg.length : 0.0;
				if (near || fenced(*seg, u - du, u + du)) continue;
				NetCrossing c;
				c.seg = sg.id;
				c.end = -1;
				c.kind = CrossingKind::Uncontrolled;
				c.unmarked = true;
				c.level = seg->level;
				if (!kerbs_at(sg, s, c.a, c.b)) continue;
				cross_road(sg, c, left, s, right, s);
			}
		}
	}

	void stops() {
		for (size_t i = 0; i < net.stops.size(); ++i) {
			const NetStop &st = net.stops[i];
			if (st.lane < 0) continue;
			const NetLane &l = net.lanes[static_cast<size_t>(st.lane)];
			Chain *c = chain(st.segment, l.dir == LaneDir::Forward ? 1 : 0);
			if (!c) c = chain(st.segment, l.dir == LaneDir::Forward ? 0 : 1);
			if (!c) continue;
			double d = 0.0;
			const double s = c->project(st.pos, &d);
			PedStop ps;
			ps.stop = static_cast<int32_t>(i);
			ps.node = node_at(*c, s);
			ps.pos = g.nodes[static_cast<size_t>(ps.node)].pos;
			ps.level = g.nodes[static_cast<size_t>(ps.node)].level;
			g.stops.push_back(ps);
		}
	}

	void path_links() {
		for (const auto &kv : path_node) {
			const NodeId nid = kv.first;
			int paths = 0;
			bool spawn = false;
			for (SegmentId sid : map.segments_at(nid)) paths += map.segment(sid)->kind == SegmentKind::Footpath ? 1 : 0;
			const RoadNode *rn = map.node(nid);
			spawn = rn && rn->spawner.enabled;
			if (paths != 1 || spawn) continue;
			const PedNode pn = g.nodes[static_cast<size_t>(kv.second)];
			double best = 10.0;
			Chain *bc = nullptr;
			double bs = 0.0;
			for (Chain &c : chains) {
				const RoadSegment *seg = map.segment(c.seg);
				if (seg->kind == SegmentKind::Footpath) continue;
				if (c.level_at(c.st.front()) != pn.level && c.level_at(c.st.back()) != pn.level) continue;
				double d = 0.0;
				const double s = c.project(pn.pos, &d);
				if (d < best && c.level_at(s) == pn.level) {
					best = d;
					bc = &c;
					bs = s;
				}
			}
			if (bc) walk_link(kv.second, node_at(*bc, bs));
		}
	}

	void spawners() {
		for (const auto &kv : map.nodes()) {
			const RoadNode &n = kv.second;
			if (!n.spawner.enabled) continue;
			PedSpawner ps;
			ps.node = n.id;
			ps.pos = qv(n.pos);
			ps.level = n.level;
			ps.people = n.spawner.people;
			ps.sink = n.spawner.sink;
			ps.od = n.spawner.od;
			ps.road = net.spawner_at(n.id) != nullptr;
			auto pn = path_node.find(n.id);
			if (pn != path_node.end()) ps.entries.push_back(pn->second);
			const NodeGeom *ng = geom.node(n.id);
			if (ng && ng->kind == NodeKind::End && !ng->legs.empty()) {
				for (bool right : { false, true }) {
					LegSide ls = leg_side(ng->legs[0], right);
					if (ls.chain) ps.entries.push_back(node_at(*ls.chain, ls.station));
				}
			}
			std::sort(ps.entries.begin(), ps.entries.end());
			ps.entries.erase(std::unique(ps.entries.begin(), ps.entries.end()), ps.entries.end());
			if (!ps.entries.empty()) g.spawners.push_back(ps);
		}
	}

	void chain_edges() {
		for (Chain &c : chains) {
			for (size_t i = 0; i + 1 < c.stations.size(); ++i) {
				const auto &a = c.stations[i];
				const auto &b = c.stations[i + 1];
				const double len = c.walk_at(b.first) - c.walk_at(a.first);
				const int la = g.nodes[static_cast<size_t>(a.second)].level, lb = g.nodes[static_cast<size_t>(b.second)].level;
				PedEdgeKind kind = PedEdgeKind::Walk;
				if (la != lb) kind = c.stairs ? PedEdgeKind::Stairs : PedEdgeKind::Ramp;
				add_edge(a.second, b.second, len, kind, -1, lb - la);
			}
		}
	}

	// Where car lanes pass over each crossing.
	void spans() {
		g.lane_crossings.assign(net.lanes.size(), {});
		for (size_t ci = 0; ci < g.crossings.size(); ++ci) {
			NetCrossing &c = g.crossings[ci];
			std::vector<int32_t> cand;
			for (size_t li = 0; li < net.lanes.size(); ++li) {
				const NetLane &l = net.lanes[li];
				if (l.kind == NetLaneKind::Road && l.segment == c.seg) cand.push_back(static_cast<int32_t>(li));
			}
			if (c.end >= 0) {
				const int32_t j = net.junction_at(c.node);
				if (j >= 0) {
					for (int32_t cn : net.junctions[static_cast<size_t>(j)].connectors) cand.push_back(cn);
					// Roundabout and other legs' lanes that end or start at this node.
				}
			}
			const Vec2 d = c.b - c.a;
			const double len = c.length;
			const Vec2 u = d * (1.0 / len);
			const Vec2 a = c.a - u * 0.5, bb = c.b + u * 0.5;
			for (int32_t li : cand) {
				const NetLane &l = net.lanes[static_cast<size_t>(li)];
				for (size_t k = 0; k + 1 < l.pts.size(); ++k) {
					const Vec2 p = l.pts[k], r = l.pts[k + 1] - l.pts[k];
					const Vec2 e = bb - a;
					const double den = cross(r, e);
					if (std::fabs(den) < 1e-12) continue;
					const double t = cross(a - p, e) / den;
					const double v = cross(a - p, r) / den;
					if (t < 0.0 || t > 1.0 || v < 0.0 || v > 1.0) continue;
					const double s_hit = l.cum[k] + (l.cum[k + 1] - l.cum[k]) * t;
					const double t_hit = v * (len + 1.0) - 0.5;
					CrossingSpan sp;
					sp.lane = li;
					sp.s0 = quantize(std::max(0.0, s_hit - 1.5 - 0.5));
					sp.s1 = quantize(std::min(l.length, s_hit + 1.5 + 0.5));
					sp.t0 = quantize(t_hit - 1.8);
					sp.t1 = quantize(t_hit + 1.8);
					c.spans.push_back(sp);
					g.lane_crossings[static_cast<size_t>(li)].push_back(static_cast<int32_t>(ci));
					break;
				}
			}
		}
		for (auto &v : g.lane_crossings) {
			std::sort(v.begin(), v.end());
			v.erase(std::unique(v.begin(), v.end()), v.end());
		}
	}
};

} // namespace

void PedGraph::clear() {
	nodes.clear();
	edges.clear();
	adj.clear();
	crossings.clear();
	lane_crossings.clear();
	stops.clear();
	spawners.clear();
}

void build_ped_network(const RoadMap &map, const RoadGeometry &geom, Network &out) {
	out.ped.clear();
	Builder b(map, geom, out);
	b.build_chains();
	b.corners();
	b.leg_crossings();
	b.mid_block();
	b.stops();
	b.path_links();
	b.spawners();
	b.chain_edges();
	b.spans();
}

WalkLight NetJunction::walk_light(SegmentId leg, int64_t tick, int64_t clear_ticks) const {
	const NetSignal &s = signal;
	int64_t into = 0;
	const int p = phase_at(tick, &into);
	if (p < 0 || static_cast<size_t>(p) >= s.walk.size()) return WalkLight::DontWalk;
	const std::vector<SegmentId> &w = s.walk[static_cast<size_t>(p)];
	if (std::find(w.begin(), w.end(), leg) == w.end()) return WalkLight::DontWalk;
	const int64_t green = s.green[static_cast<size_t>(p)];
	if (into >= green) return WalkLight::DontWalk;
	// Flash for long enough to finish crossing, but walk for at least 4 s.
	const int64_t flash_from = std::max<int64_t>(std::min<int64_t>(40, green), green - clear_ticks);
	return into < flash_from ? WalkLight::Walk : WalkLight::Flashing;
}

} // namespace tsim
