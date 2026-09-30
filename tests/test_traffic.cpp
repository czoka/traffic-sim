// M2 tests: network compile, routing, junction control, lane changes, demand,
// editing while paused, determinism and the M2 gate.
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include <doctest/doctest.h>

#include "tsim/demo_maps.h"
#include "tsim/document.h"
#include "tsim/hash.h"
#include "tsim/network.h"
#include "tsim/road_geometry.h"
#include "tsim/road_map_json.h"
#include "tsim/rng.h"
#include "tsim/traffic_run.h"
#include "tsim/validation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

using namespace tsim;

namespace {

struct World {
	Document doc;
	RoadGeometry geom;
	TrafficRun run;

	void sync() {
		geom.build(doc.map());
		run.sync(doc.map(), geom, doc.revision());
	}
	Traffic &t() { return run.traffic(); }
	const Network &net() const { return run.network(); }
	void run_for(double seconds) {
		const uint64_t n = static_cast<uint64_t>(seconds / t().config().dt + 0.5);
		for (uint64_t i = 0; i < n; ++i) t().tick();
	}
};

PointRef free_point(double x, double y) {
	PointRef p;
	p.pos = Vec2{ x, y };
	return p;
}

PointRef node_point(const Document &doc, NodeId n) {
	PointRef p;
	p.node = n;
	p.pos = doc.map().node(n)->pos;
	return p;
}

Profile preset(const char *name) {
	RoadMap scratch;
	return preset_profile(name, scratch);
}

Spawner spawner(double rate, bool sink = true) {
	Spawner s;
	s.enabled = true;
	s.rate = rate;
	s.sink = sink;
	return s;
}

// Four-way junction at the origin with 100 m arms: west (west -> c), east
// (c -> east), north (north -> c), south (c -> south). Spawn / sink points at
// the four ends, with no demand of their own.
struct Cross {
	NodeId c = 0, w = 0, e = 0, n = 0, s = 0;
	SegmentId sw = 0, se = 0, sn = 0, ss = 0;
};

Cross cross_roads(Document &doc, const char *profile = "Street 1+1", JunctionControl ctl = JunctionControl::RightHand) {
	Cross x;
	const Profile p = preset(profile);
	x.sw = doc.add_road({ free_point(-100, 0), free_point(0, 0) }, p, 0, 13.9).front();
	x.c = doc.map().segment(x.sw)->to;
	x.w = doc.map().segment(x.sw)->from;
	x.se = doc.add_road({ node_point(doc, x.c), free_point(100, 0) }, p, 0, 13.9).front();
	x.e = doc.map().segment(x.se)->to;
	x.sn = doc.add_road({ free_point(0, -100), node_point(doc, x.c) }, p, 0, 13.9).front();
	x.n = doc.map().segment(x.sn)->from;
	x.ss = doc.add_road({ node_point(doc, x.c), free_point(0, 100) }, p, 0, 13.9).front();
	x.s = doc.map().segment(x.ss)->to;
	for (NodeId end : { x.w, x.e, x.n, x.s }) doc.set_spawner(end, spawner(0.0));
	doc.set_junction_control(x.c, ctl, {});
	return x;
}

// Car lanes of a segment in one direction, rightmost (in the sense of travel) first.
std::vector<int32_t> lanes_of(const Network &net, SegmentId seg, LaneDir dir) {
	int32_t start = -1;
	for (size_t i = 0; i < net.lanes.size(); ++i) {
		const NetLane &l = net.lanes[i];
		if (l.kind == NetLaneKind::Road && l.segment == seg && l.dir == dir && !l.pocket && l.right < 0) {
			start = static_cast<int32_t>(i);
		}
	}
	std::vector<int32_t> out;
	for (int32_t l = start; l >= 0; l = net.lanes[static_cast<size_t>(l)].left) {
		if (!net.lanes[static_cast<size_t>(l)].pocket) out.push_back(l);
	}
	return out;
}

const Vehicle *car(const Traffic &t, VehicleId id) {
	const int32_t i = t.find_vehicle(id);
	return i < 0 ? nullptr : &t.vehicles()[static_cast<size_t>(i)];
}

bool in_box(const Network &net, const Vehicle &v) {
	return net.lanes[static_cast<size_t>(v.lane)].kind == NetLaneKind::Connector;
}

// Front of a vehicle along a connector, or a large value when it is neither
// on the connector nor on the lane after it.
double pos_on(const Network &net, const Vehicle &v, int32_t conn) {
	const NetLane &c = net.lanes[static_cast<size_t>(conn)];
	if (v.lane == conn) return v.s;
	if (v.lane == c.to) return c.length + v.s;
	return 1e300;
}

// Safety invariants for one tick: no car overlaps the one ahead in its lane,
// and no two cars sit on the crossing point of two conflicting connectors.
struct Invariants {
	int overlaps = 0;
	int box_violations = 0;
	void check(const Network &net, const Traffic &t) {
		std::map<int32_t, std::vector<const Vehicle *>> by_lane;
		for (const Vehicle &v : t.vehicles()) {
			if (!v.off_lane) by_lane[v.lane].push_back(&v); // parked cars are beside the lane
		}
		for (auto &kv : by_lane) {
			auto &vs = kv.second;
			std::sort(vs.begin(), vs.end(), [](const Vehicle *a, const Vehicle *b) { return a->s > b->s; });
			for (size_t k = 1; k < vs.size(); ++k) {
				if (vs[k - 1]->s - vs[k - 1]->drv.length - vs[k]->s < -0.2) ++overlaps;
			}
		}
		std::vector<std::pair<const Vehicle *, int32_t>> boxed;
		for (const Vehicle &v : t.vehicles()) {
			if (in_box(net, v)) boxed.push_back({ &v, v.lane });
			else if (v.held >= 0) boxed.push_back({ &v, v.held });
		}
		for (size_t i = 0; i < boxed.size(); ++i) {
			for (size_t j = i + 1; j < boxed.size(); ++j) {
				const int32_t a = boxed[i].second, b = boxed[j].second;
				for (const Conflict &cf : net.lanes[static_cast<size_t>(a)].conflicts) {
					if (cf.other != b || cf.merge) continue;
					const Vehicle &va = *boxed[i].first;
					const Vehicle &vb = *boxed[j].first;
					const double pa = pos_on(net, va, a), pb = pos_on(net, vb, b);
					const bool on_a = pa >= cf.s_self - 0.5 && pa - va.drv.length <= cf.s_self + 0.5;
					const bool on_b = pb >= cf.s_other - 0.5 && pb - vb.drv.length <= cf.s_other + 0.5;
					if (on_a && on_b) ++box_violations;
				}
			}
		}
	}
};

// Ticks until `pred` holds or the time runs out; returns the sim time taken.
template <typename F>
double run_until(World &w, double max_seconds, F pred) {
	const double start = w.t().sim_time();
	while (w.t().sim_time() - start < max_seconds) {
		if (pred()) return w.t().sim_time() - start;
		w.t().tick();
	}
	return -1.0;
}

} // namespace

// --- Network compile ---------------------------------------------------------------

TEST_CASE("network: four-way junction lanes, connectors and conflict matrix") {
	World w;
	const Cross x = cross_roads(w.doc);
	w.sync();
	const Network &n = w.net();
	REQUIRE(n.junctions.size() == 1);
	const NetJunction &j = n.junctions[0];
	CHECK(j.node == x.c);
	CHECK(j.arbitrated);
	CHECK(j.approaches.size() == 4);
	CHECK(n.spawners.size() == 4);
	int straight = 0;
	for (int32_t c : j.connectors) {
		const NetLane &cn = n.lanes[static_cast<size_t>(c)];
		straight += cn.turn == TurnKind::Straight;
		CHECK(cn.length > 5.0);
		CHECK(cn.speed_limit <= 13.9 + 0.001);
		if (cn.turn == TurnKind::Right) CHECK(cn.speed_limit < 7.0); // tight turns are slow
		// Every conflict is mirrored on the other connector.
		for (const Conflict &cf : cn.conflicts) {
			bool mirrored = false;
			for (const Conflict &back : n.lanes[static_cast<size_t>(cf.other)].conflicts) {
				mirrored |= back.other == c && back.s_self == cf.s_other && back.s_other == cf.s_self &&
						back.priority == -cf.priority;
			}
			CHECK(mirrored);
		}
	}
	CHECK(straight == 4);
	auto conn = [&](SegmentId from, LaneDir fd, SegmentId to) {
		for (int32_t c : j.connectors) {
			const NetLane &cn = n.lanes[static_cast<size_t>(c)];
			const NetLane &a = n.lanes[static_cast<size_t>(cn.from)];
			const NetLane &b = n.lanes[static_cast<size_t>(cn.to)];
			if (a.segment == from && a.dir == fd && b.segment == to) return c;
		}
		return -1;
	};
	auto conflicts = [&](int32_t a, int32_t b) {
		for (const Conflict &cf : n.lanes[static_cast<size_t>(a)].conflicts) {
			if (cf.other == b) return true;
		}
		return false;
	};
	const int32_t we = conn(x.sw, LaneDir::Forward, x.se);
	const int32_t ew = conn(x.se, LaneDir::Backward, x.sw);
	const int32_t ns = conn(x.sn, LaneDir::Forward, x.ss);
	const int32_t sn = conn(x.ss, LaneDir::Backward, x.sn);
	REQUIRE(we >= 0);
	REQUIRE(ew >= 0);
	REQUIRE(ns >= 0);
	REQUIRE(sn >= 0);
	CHECK_FALSE(conflicts(we, ew)); // opposite straights pass each other
	CHECK(conflicts(we, ns));
	// Right-hand rule: a car from the south is on the right of an eastbound car.
	for (const Conflict &cf : n.lanes[static_cast<size_t>(we)].conflicts) {
		if (cf.other == sn) CHECK(cf.priority == -1);
		if (cf.other == ns) CHECK(cf.priority == 1);
	}
	// Everything the sim reads sits on the 1/1024 m grid.
	for (const NetLane &l : n.lanes) {
		for (const Vec2 &p : l.pts) {
			CHECK(p.x * 1024.0 == std::round(p.x * 1024.0));
			CHECK(p.y * 1024.0 == std::round(p.y * 1024.0));
		}
	}
}

TEST_CASE("network: priority road overrides the right-hand rule for side legs") {
	World w;
	build_t_junction(w.doc);
	w.sync();
	const Network &n = w.net();
	int checked = 0;
	for (const NetLane &l : n.lanes) {
		if (l.kind != NetLaneKind::Connector) continue;
		const RoadNode *node = w.doc.map().node(l.node);
		const bool major = node->is_priority(n.lanes[static_cast<size_t>(l.from)].segment);
		for (const Conflict &cf : l.conflicts) {
			const NetLane &o = n.lanes[static_cast<size_t>(cf.other)];
			const bool other_major = node->is_priority(n.lanes[static_cast<size_t>(o.from)].segment);
			if (major != other_major) {
				CHECK(cf.priority == (major ? 1 : -1));
				++checked;
			}
		}
	}
	CHECK(checked > 0);
}

TEST_CASE("network: lane changes are blocked near stop lines and in painted zones") {
	World w;
	const Cross x = cross_roads(w.doc, "Avenue 2+2, median");
	w.sync();
	const std::vector<int32_t> in = lanes_of(w.net(), x.sw, LaneDir::Forward);
	REQUIRE(in.size() == 2);
	const NetLane &right = w.net().lanes[static_cast<size_t>(in[0])];
	CHECK(right.left == in[1]);
	CHECK(w.net().can_change(in[0], true, 10.0));
	CHECK_FALSE(w.net().can_change(in[0], true, right.length - 5.0)); // solid before the stop line
	CHECK_FALSE(w.net().can_change(in[0], false, 10.0)); // nothing to the right
	// Paint the boundary between the two eastbound lanes, blocking one way only.
	const RoadSegment *seg = w.doc.map().segment(x.sw);
	int edge = -1;
	for (size_t i = 1; i < seg->profile.lanes.size(); ++i) {
		if (seg->profile.lanes[i - 1].dir == LaneDir::Forward && seg->profile.lanes[i].dir == LaneDir::Forward &&
				is_travel(seg->profile.lanes[i - 1].type) && is_travel(seg->profile.lanes[i].type)) {
			edge = static_cast<int>(i);
		}
	}
	REQUIRE(edge > 0);
	w.doc.set_no_change(x.sw, edge, 0.0, 1.0, true, false);
	w.sync();
	const std::vector<int32_t> in2 = lanes_of(w.net(), x.sw, LaneDir::Forward);
	// Forward lanes: list-left is travel-left, so left-to-right blocks moving right.
	CHECK_FALSE(w.net().can_change(in2[1], false, 10.0));
	CHECK(w.net().can_change(in2[0], true, 10.0));
}

TEST_CASE("network: incremental recompile matches a full compile and only touches changed junctions") {
	Document doc;
	build_test_grid(doc, 7, 8, 120.0);
	RoadGeometry geom;
	NetworkCompiler incremental;
	Network a, b;
	geom.build(doc.map());
	incremental.compile(doc.map(), geom, a);
	CHECK(incremental.stats().junctions_compiled == incremental.stats().junctions);
	// No change: nothing is compiled again.
	incremental.compile(doc.map(), geom, a);
	CHECK(incremental.stats().junctions_compiled == 0);
	CHECK(incremental.stats().segments_compiled == 0);
	// One local edit recompiles only the junctions around it.
	const NodeId some = 25;
	doc.set_junction_control(some, JunctionControl::AllWayStop, {});
	geom.build(doc.map());
	incremental.compile(doc.map(), geom, a);
	CHECK(incremental.stats().junctions_compiled == 1);
	CHECK(incremental.stats().segments_compiled == 0);
	doc.move_node(some, doc.map().node(some)->pos + Vec2{ 4.0, 3.0 });
	geom.build(doc.map());
	incremental.compile(doc.map(), geom, a);
	CHECK(incremental.stats().junctions_compiled >= 1);
	CHECK(incremental.stats().junctions_compiled <= 5);
	// Random edits: the incremental result always equals a fresh compile.
	Rng rng(7);
	for (int step = 0; step < 25; ++step) {
		const auto &segs = doc.map().segments();
		auto it = segs.begin();
		std::advance(it, static_cast<long>(rng.next_u64() % segs.size()));
		const SegmentId s = it->first;
		switch (rng.next_u64() % 5) {
			case 0: {
				const NodeId nd = doc.map().segment(s)->from;
				doc.move_node(nd, doc.map().node(nd)->pos + Vec2{ rng.range(-8, 8), rng.range(-8, 8) });
				break;
			}
			case 1: {
				ProfileParams p = params_of(doc.map().segment(s)->profile);
				p.forward = 1 + static_cast<int>(rng.next_u64() % 2);
				doc.set_profile_params(s, p);
				break;
			}
			case 2: doc.set_junction_control(doc.map().segment(s)->to, JunctionControl::PriorityRoad, { s }); break;
			case 3: doc.split_segment(s, 0.5); break;
			case 4: {
				EndRules r;
				r.left = TurnRule::TurnLane;
				doc.set_end_rules(s, 1, r);
				break;
			}
		}
		geom.build(doc.map());
		incremental.compile(doc.map(), geom, a);
		NetworkCompiler fresh;
		fresh.compile(doc.map(), geom, b);
		CHECK(a.hash() == b.hash());
	}
}

// --- Routing -------------------------------------------------------------------------

TEST_CASE("routing: A* follows turn rules and one-way streets") {
	World w;
	const Cross x = cross_roads(w.doc);
	w.sync();
	const int32_t from_w = lanes_of(w.net(), x.sw, LaneDir::Forward).front();
	std::vector<int32_t> r;
	REQUIRE(w.t().find_route(from_w, x.e, r));
	REQUIRE(r.size() == 1);
	CHECK(w.net().lanes[static_cast<size_t>(r[0])].turn == TurnKind::Straight);
	REQUIRE(w.t().find_route(from_w, x.n, r));
	CHECK(w.net().lanes[static_cast<size_t>(r[0])].turn == TurnKind::Left);
	CHECK_FALSE(w.t().find_route(from_w, x.w, r)); // no U-turns
	// Ban the left turn: north can't be reached from the west any more.
	EndRules no_left;
	no_left.left = TurnRule::Disallowed;
	w.doc.set_end_rules(x.sw, 1, no_left);
	w.sync();
	const int32_t from_w2 = lanes_of(w.net(), x.sw, LaneDir::Forward).front();
	CHECK_FALSE(w.t().find_route(from_w2, x.n, r));
	CHECK(w.t().find_route(from_w2, x.s, r));

	// One-way pair: westbound traffic comes back east only via a cross street.
	World p;
	build_one_way_pair(p.doc);
	p.sync();
	const NetSpawner *west_in = p.net().spawner_at(1);
	REQUIRE(west_in != nullptr);
	REQUIRE(!west_in->spawn_lanes.empty());
	const NodeId s_w = 5; // end of the westbound street
	REQUIRE(p.t().find_route(west_in->spawn_lanes[0], s_w, r));
	REQUIRE(r.size() == 2);
	CHECK(p.net().lanes[static_cast<size_t>(r[0])].turn == TurnKind::Right); // south on a cross street
	CHECK(p.net().lanes[static_cast<size_t>(r[1])].turn == TurnKind::Right); // then west
	const NetSpawner *east_in = p.net().spawner_at(8);
	REQUIRE(east_in != nullptr);
	CHECK_FALSE(p.t().find_route(east_in->spawn_lanes[0], 8, r)); // not back where it came from
}

// --- Junction control -------------------------------------------------------------------

TEST_CASE("junctions: right-hand priority lets the car from the right go first") {
	for (int flip = 0; flip < 2; ++flip) {
		World w;
		const Cross x = cross_roads(w.doc);
		w.sync();
		w.t().reset(1);
		w.t().config().accel_noise = 0.0;
		const int32_t east = lanes_of(w.net(), x.sw, LaneDir::Forward).front();
		// flip 0: the other car comes from the south (right of an eastbound car);
		// flip 1: from the north (its left).
		const int32_t other = flip == 0 ? lanes_of(w.net(), x.ss, LaneDir::Backward).front()
										: lanes_of(w.net(), x.sn, LaneDir::Forward).front();
		const VehicleId a = w.t().add_vehicle(east, w.net().lanes[static_cast<size_t>(east)].length - 40.0, 8.0, x.e);
		const VehicleId b = w.t().add_vehicle(other, w.net().lanes[static_cast<size_t>(other)].length - 40.0, 8.0,
				flip == 0 ? x.n : x.s);
		REQUIRE(a != kNoId);
		REQUIRE(b != kNoId);
		double enter_a = -1, enter_b = -1;
		Invariants inv;
		for (int i = 0; i < 400 && (enter_a < 0 || enter_b < 0); ++i) {
			w.t().tick();
			inv.check(w.net(), w.t());
			const Vehicle *va = car(w.t(), a), *vb = car(w.t(), b);
			if (enter_a < 0 && va && in_box(w.net(), *va)) enter_a = w.t().sim_time();
			if (enter_b < 0 && vb && in_box(w.net(), *vb)) enter_b = w.t().sim_time();
		}
		REQUIRE(enter_a > 0);
		REQUIRE(enter_b > 0);
		if (flip == 0) CHECK(enter_b < enter_a);
		else CHECK(enter_a < enter_b);
		CHECK(inv.box_violations == 0);
		CHECK(inv.overlaps == 0);
	}
}

TEST_CASE("junctions: priority road and gap acceptance") {
	// Segments: 1 west -> centre (main), 2 centre -> east (main), 3 south -> centre.
	const NodeId west = 1, east = 3;
	World w;
	build_t_junction(w.doc);
	w.sync();
	w.t().reset(1);
	w.t().config().accel_noise = 0.0;
	const int32_t main_in = lanes_of(w.net(), 1, LaneDir::Forward).front();
	const int32_t side_in = lanes_of(w.net(), 3, LaneDir::Forward).front();
	// A side-street car waits at the line to turn left across the eastbound
	// lane; a main-road car is 60 m out at 47 km/h: the side car must wait.
	const VehicleId side = w.t().add_vehicle(side_in, w.net().lanes[static_cast<size_t>(side_in)].length - 2.5, 0.0, west);
	const VehicleId main = w.t().add_vehicle(main_in, w.net().lanes[static_cast<size_t>(main_in)].length - 60.0, 13.0, east);
	REQUIRE(side != kNoId);
	REQUIRE(main != kNoId);
	double side_enter = -1, main_enter = -1, main_min_speed = 1e9;
	for (int i = 0; i < 400 && side_enter < 0; ++i) {
		w.t().tick();
		const Vehicle *m = car(w.t(), main), *sd = car(w.t(), side);
		if (m) main_min_speed = std::min(main_min_speed, m->v);
		if (main_enter < 0 && m && in_box(w.net(), *m)) main_enter = w.t().sim_time();
		if (side_enter < 0 && sd && in_box(w.net(), *sd)) side_enter = w.t().sim_time();
	}
	REQUIRE(side_enter > 0);
	REQUIRE(main_enter > 0);
	CHECK(main_enter < side_enter);
	CHECK(main_min_speed > 9.0); // the main road car barely slowed down
	CHECK(side_enter - main_enter < 15.0); // and the side car went soon after

	// A big gap (main car 250 m away): the side car goes straight away.
	World g;
	build_t_junction(g.doc);
	g.sync();
	g.t().reset(1);
	g.t().config().accel_noise = 0.0;
	const int32_t mi = lanes_of(g.net(), 1, LaneDir::Forward).front();
	const int32_t si = lanes_of(g.net(), 3, LaneDir::Forward).front();
	const VehicleId sd2 = g.t().add_vehicle(si, g.net().lanes[static_cast<size_t>(si)].length - 2.5, 0.0, west);
	g.t().add_vehicle(mi, 5.0, 13.0, east);
	const double waited = run_until(g, 30.0, [&] {
		const Vehicle *v = car(g.t(), sd2);
		return v && in_box(g.net(), *v);
	});
	CHECK(waited >= 0.0);
	CHECK(waited < 4.0);
}

TEST_CASE("junctions: all-way stop makes every car stop, first come first served") {
	World w;
	const Cross x = cross_roads(w.doc, "Street 1+1", JunctionControl::AllWayStop);
	w.sync();
	w.t().reset(3);
	const int32_t east = lanes_of(w.net(), x.sw, LaneDir::Forward).front();
	const VehicleId a = w.t().add_vehicle(east, 20.0, 12.0, x.e);
	REQUIRE(a != kNoId);
	double min_speed_near_line = 1e9;
	bool entered = false;
	for (int i = 0; i < 600 && !entered; ++i) {
		const Vehicle *v = car(w.t(), a);
		if (v->lane == east && w.net().lanes[static_cast<size_t>(east)].length - v->s < 5.0) {
			min_speed_near_line = std::min(min_speed_near_line, v->v);
		}
		w.t().tick();
		entered = in_box(w.net(), *car(w.t(), a));
	}
	CHECK(entered);
	CHECK(min_speed_near_line < 0.25);

	// Two cars stopped at the line: the one that stopped first goes first.
	World f;
	const Cross y = cross_roads(f.doc, "Street 1+1", JunctionControl::AllWayStop);
	f.sync();
	f.t().reset(3);
	f.t().config().accel_noise = 0.0;
	const int32_t we = lanes_of(f.net(), y.sw, LaneDir::Forward).front();
	const int32_t ns = lanes_of(f.net(), y.sn, LaneDir::Forward).front();
	const VehicleId first = f.t().add_vehicle(we, f.net().lanes[static_cast<size_t>(we)].length - 2.6, 0.0, y.e);
	f.run_for(1.0);
	const VehicleId second = f.t().add_vehicle(ns, f.net().lanes[static_cast<size_t>(ns)].length - 2.6, 0.0, y.s);
	double t1 = -1, t2 = -1;
	for (int i = 0; i < 400 && (t1 < 0 || t2 < 0); ++i) {
		f.t().tick();
		const Vehicle *v1 = car(f.t(), first), *v2 = car(f.t(), second);
		if (t1 < 0 && (!v1 || in_box(f.net(), *v1))) t1 = f.t().sim_time();
		if (t2 < 0 && (!v2 || in_box(f.net(), *v2))) t2 = f.t().sim_time();
	}
	CHECK(t1 > 0);
	CHECK(t2 > t1);
}

TEST_CASE("junctions: four cars arriving together at a right-hand junction don't deadlock") {
	World w;
	const Cross x = cross_roads(w.doc);
	w.sync();
	w.t().reset(5);
	const int32_t lanes[4] = { lanes_of(w.net(), x.sw, LaneDir::Forward).front(),
		lanes_of(w.net(), x.se, LaneDir::Backward).front(), lanes_of(w.net(), x.sn, LaneDir::Forward).front(),
		lanes_of(w.net(), x.ss, LaneDir::Backward).front() };
	const NodeId dest[4] = { x.e, x.w, x.s, x.n };
	for (int k = 0; k < 4; ++k) {
		const VehicleId id =
				w.t().add_vehicle(lanes[k], w.net().lanes[static_cast<size_t>(lanes[k])].length - 2.6, 0.0, dest[k]);
		REQUIRE(id != kNoId);
	}
	Invariants inv;
	const double took = run_until(w, 90.0, [&] {
		inv.check(w.net(), w.t());
		return w.t().stats().arrived == 4;
	});
	CHECK(took > 0.0);
	CHECK(w.t().stats().forced_grants >= 1); // everyone yields to the right: the breaker decides
	CHECK(inv.box_violations == 0);
}

TEST_CASE("junctions: don't block the box") {
	World w;
	const Cross x = cross_roads(w.doc);
	w.sync();
	w.t().reset(9);
	w.t().config().accel_noise = 0.0;
	// A queue of stopped cars fills the east arm right up to the junction.
	const int32_t east_out = lanes_of(w.net(), x.se, LaneDir::Forward).front();
	const double len = w.net().lanes[static_cast<size_t>(east_out)].length;
	DriverParams2 parked;
	parked.speed_factor = 0.0;
	for (double s = len - 1.0; s > 5.0; s -= 6.6) w.t().add_vehicle(east_out, s, 0.0, x.e, &parked);
	const int32_t west_in = lanes_of(w.net(), x.sw, LaneDir::Forward).front();
	const VehicleId a = w.t().add_vehicle(west_in, 20.0, 10.0, x.e);
	REQUIRE(a != kNoId);
	w.run_for(40.0);
	const Vehicle *v = car(w.t(), a);
	REQUIRE(v != nullptr);
	CHECK(v->lane == west_in); // waits before the junction instead of entering it
	CHECK(v->state == VehicleState::ExitBlocked);
	// A car crossing north -> south is not held up by it.
	const int32_t north_in = lanes_of(w.net(), x.sn, LaneDir::Forward).front();
	w.t().add_vehicle(north_in, 20.0, 10.0, x.s);
	const uint64_t before = w.t().stats().arrived;
	w.run_for(40.0);
	CHECK(w.t().stats().arrived == before + 1);
}

// --- Lane changes --------------------------------------------------------------------------

namespace {

// A straight one-way road with two lanes, 800 m long.
struct Straight {
	SegmentId seg = 0;
	NodeId a = 0, b = 0;
};

Straight two_lane_road(Document &doc) {
	Straight r;
	r.seg = doc.add_road({ free_point(0, 0), free_point(800, 0) }, preset("One-way 2 lanes"), 0, 13.9).front();
	r.a = doc.map().segment(r.seg)->from;
	r.b = doc.map().segment(r.seg)->to;
	doc.set_spawner(r.a, spawner(0.0, false));
	doc.set_spawner(r.b, spawner(0.0, true));
	return r;
}

} // namespace

TEST_CASE("lane changes: MOBIL overtakes a slow car, painted lines forbid it") {
	for (int painted = 0; painted < 2; ++painted) {
		World w;
		const Straight r = two_lane_road(w.doc);
		if (painted) w.doc.set_no_change(r.seg, 2, 0.0, 1.0, true, true);
		w.sync();
		w.t().reset(2);
		const std::vector<int32_t> l = lanes_of(w.net(), r.seg, LaneDir::Forward);
		REQUIRE(l.size() == 2);
		DriverParams2 slow;
		slow.speed_factor = 0.4;
		const VehicleId s = w.t().add_vehicle(l[0], 120.0, 5.5, r.b, &slow);
		const VehicleId f = w.t().add_vehicle(l[0], 60.0, 13.0, r.b);
		REQUIRE(s != kNoId);
		REQUIRE(f != kNoId);
		Invariants inv;
		for (int i = 0; i < 300; ++i) {
			w.t().tick();
			inv.check(w.net(), w.t());
		}
		const Vehicle *vs = car(w.t(), s), *vf = car(w.t(), f);
		REQUIRE(vs != nullptr);
		if (!painted) {
			CHECK(w.t().stats().lane_changes >= 1);
			CHECK((!vf || vf->distance > vs->distance)); // overtook (or already arrived)
		} else {
			CHECK(w.t().stats().lane_changes == 0);
			REQUIRE(vf != nullptr);
			CHECK(vf->s < vs->s);
		}
		CHECK(inv.overlaps == 0);
	}
}

TEST_CASE("lane changes: early mandatory change into a left-turn pocket") {
	World w;
	const Cross x = cross_roads(w.doc, "Avenue 2+2, median");
	EndRules pocket;
	pocket.left = TurnRule::TurnLane;
	pocket.turn_lane_length = 40.0;
	w.doc.set_end_rules(x.sw, 1, pocket);
	w.sync();
	w.t().reset(4);
	const std::vector<int32_t> in = lanes_of(w.net(), x.sw, LaneDir::Forward);
	REQUIRE(in.size() == 2);
	// Starts in the right lane and wants to turn left (north).
	const VehicleId a = w.t().add_vehicle(in[0], 3.0, 10.0, x.n);
	REQUIRE(a != kNoId);
	bool used_pocket = false;
	for (int i = 0; i < 600 && car(w.t(), a); ++i) {
		const Vehicle *v = car(w.t(), a);
		if (w.net().lanes[static_cast<size_t>(v->lane)].pocket) used_pocket = true;
		w.t().tick();
	}
	CHECK(used_pocket);
	CHECK(w.t().stats().arrived == 1);
}

TEST_CASE("lane drop: two lanes merge into one without stalls") {
	World w;
	build_lane_drop(w.doc);
	w.sync();
	w.t().reset(8);
	Invariants inv;
	for (int i = 0; i < 6000; ++i) {
		w.t().tick();
		if (i % 10 == 0) inv.check(w.net(), w.t());
	}
	const TrafficStats st = w.t().stats();
	CHECK(st.arrived > 80);
	CHECK(st.removed_stuck == 0);
	CHECK(st.max_stopped < 60.0);
	CHECK(st.lane_changes > 10); // cars leave the ending lane early
	CHECK(inv.overlaps == 0);
	CHECK(inv.box_violations == 0);
}

// --- Demand -------------------------------------------------------------------------------

TEST_CASE("demand: spawn rates, origin-destination weights and unreachable pairs") {
	World w;
	build_t_junction(w.doc);
	// West sends nobody east.
	Spawner west = w.doc.map().node(1)->spawner;
	west.od = { OdWeight{ 3, 0.0 } };
	w.doc.set_spawner(1, west);
	w.sync();
	w.t().reset(11);
	int west_to_east = 0, total = 0;
	std::map<VehicleId, bool> seen;
	for (int i = 0; i < 6000; ++i) {
		w.t().tick();
		for (const Vehicle &v : w.t().vehicles()) {
			if (seen.count(v.id)) continue;
			seen[v.id] = true;
			++total;
			if (v.origin == 1 && v.dest == 3) ++west_to_east;
		}
	}
	// 400 + 400 + 300 veh/h for 10 minutes: about 180 trips.
	CHECK(total > 130);
	CHECK(total < 240);
	CHECK(west_to_east == 0);
	CHECK(network_problems(w.doc.map(), w.net()).empty());

	// Make the side street one-way into the junction: nothing can reach its end.
	ProfileParams p = params_of(w.doc.map().segment(3)->profile);
	p.backward = 0;
	w.doc.set_profile_params(3, p);
	// And a spawn point in the middle of the map is inactive.
	w.doc.set_spawner(2, spawner(100.0));
	w.sync();
	bool unreachable = false, not_at_end = false;
	for (const NetProblem &pr : network_problems(w.doc.map(), w.net())) {
		unreachable |= pr.code == "unreachable";
		not_at_end |= pr.code == "spawner_not_at_end";
	}
	CHECK(unreachable);
	CHECK(not_at_end);
}

// --- Editing while paused ----------------------------------------------------------------

TEST_CASE("edit while paused: resume keeps cars and recompiles only what changed") {
	World w;
	build_test_grid(w.doc, 7, 8, 120.0);
	w.sync();
	w.t().config().max_vehicles = 300;
	w.t().config().demand = 2.0;
	w.t().reset(21);
	w.run_for(180.0);
	const size_t before = w.t().vehicles().size();
	REQUIRE(before > 200);
	// Delete a road that has cars on it, and change a junction's control.
	std::map<SegmentId, int> per_seg;
	for (const Vehicle &v : w.t().vehicles()) {
		const NetLane &l = w.net().lanes[static_cast<size_t>(v.lane)];
		if (l.kind == NetLaneKind::Road) ++per_seg[l.segment];
	}
	SegmentId busiest = 0;
	int most = 0;
	for (const auto &kv : per_seg) {
		if (kv.first <= 98 && kv.second > most) {
			most = kv.second;
			busiest = kv.first;
		}
	}
	REQUIRE(busiest != 0);
	const NodeId end = w.doc.map().segment(busiest)->to;
	w.doc.delete_segment(busiest);
	w.doc.set_junction_control(end, JunctionControl::AllWayStop, {});
	w.sync();
	const CompileStats &cs = w.run.compile_stats();
	CHECK(cs.junctions_compiled <= 4);
	CHECK(cs.segments_compiled <= 10);
	const size_t after = w.t().vehicles().size();
	CHECK(after + static_cast<size_t>(most) <= before + 2);
	CHECK(after + static_cast<size_t>(most) + 12 >= before);
	// Nobody is left on the deleted road, and the sim carries on.
	for (const Vehicle &v : w.t().vehicles()) CHECK(w.net().lanes[static_cast<size_t>(v.lane)].segment != busiest);
	const uint64_t arrived = w.t().stats().arrived;
	Invariants inv;
	for (int i = 0; i < 1800; ++i) {
		w.t().tick();
		if (i % 20 == 0) inv.check(w.net(), w.t());
	}
	CHECK(w.t().stats().arrived > arrived + 100);
	CHECK(inv.overlaps == 0);
	CHECK(inv.box_violations == 0);
}

// --- Map model ---------------------------------------------------------------------------

TEST_CASE("map v3: junction control and spawn points save, load, undo and survive splits") {
	Document doc;
	build_t_junction(doc);
	const std::string a = road_map_to_json(doc.map());
	CHECK(a.find("\"version\": 4") != std::string::npos);
	CHECK(a.find("priority_road") != std::string::npos);
	CHECK(a.find("\"spawner\"") != std::string::npos);
	RoadMap loaded;
	std::string err;
	REQUIRE(road_map_from_json(a, loaded, &err));
	CHECK(loaded == doc.map());
	CHECK(road_map_to_json(loaded) == a);

	// Version 2 files load with the default control and no spawn points.
	const std::string v2 = R"({"format":"traffic-sim-map","version":2,"nodes":[{"id":1,"x":0,"y":0,"level":0},
		{"id":2,"x":100,"y":0,"level":0}],"segments":[{"id":1,"from":1,"to":2,"kind":"road","level":0,
		"curve":{"type":"straight"},"speed_limit":13.9,"profile":{"median":"none","lanes":[
		{"id":1,"type":"general","dir":"forward","width":3.25}]}}]})";
	RoadMap old;
	int version = 0;
	REQUIRE(road_map_from_json(v2, old, &err, &version));
	CHECK(version == 2);
	CHECK(old.node(1)->control == JunctionControl::RightHand);
	CHECK_FALSE(old.node(1)->spawner.enabled);
	CHECK_FALSE(road_map_from_json(R"({"format":"traffic-sim-map","version":3,"nodes":[{"id":1,"x":0,"y":0,"level":0,
		"control":{"type":"roundabout"}}],"segments":[]})",
			old, &err));

	// Undo / redo of control and spawner edits.
	const RoadMap before = doc.map();
	doc.set_junction_control(2, JunctionControl::AllWayStop, {});
	doc.set_spawner(1, Spawner{});
	CHECK(doc.map().node(2)->control == JunctionControl::AllWayStop);
	CHECK(doc.map().node(2)->priority.empty());
	CHECK_FALSE(doc.map().node(1)->spawner.enabled);
	CHECK(doc.undo());
	CHECK(doc.undo());
	CHECK(doc.map() == before);
	CHECK(doc.redo());
	CHECK(doc.redo());
	CHECK(doc.map().node(2)->control == JunctionControl::AllWayStop);

	// Splitting a main-road leg keeps the priority on the piece at the junction.
	Document t;
	build_t_junction(t);
	const RoadNode centre = *t.map().node(2);
	REQUIRE(centre.priority.size() == 2);
	const SegmentId east_leg = 2;
	REQUIRE(centre.is_priority(east_leg));
	t.split_segment(east_leg, 0.5); // centre -> east: the far half gets a new id
	t.split_segment(1, 0.5); // west -> centre: the half at the junction gets a new id
	const RoadNode &c2 = *t.map().node(2);
	CHECK(c2.priority.size() == 2);
	CHECK(c2.is_priority(east_leg));
	for (SegmentId s : c2.priority) {
		const RoadSegment *seg = t.map().segment(s);
		REQUIRE(seg != nullptr);
		CHECK((seg->from == 2 || seg->to == 2));
	}
	t.delete_segment(east_leg);
	CHECK_FALSE(t.map().node(2)->is_priority(east_leg));
}

TEST_CASE("test maps: every M2 map validates and runs") {
	struct Case {
		const char *name;
		void (*build)(Document &);
	};
	const Case cases[] = {
		{ "demo town", [](Document &d) { build_demo_town(d); } },
		{ "T junction", [](Document &d) { build_t_junction(d); } },
		{ "lane drop", [](Document &d) { build_lane_drop(d); } },
		{ "one-way pair", [](Document &d) { build_one_way_pair(d); } },
	};
	for (const Case &c : cases) {
		World w;
		c.build(w.doc);
		w.sync();
		int errors = 0;
		for (const Problem &p : validate(w.doc.map(), w.geom)) errors += p.severity == Severity::Error;
		CHECK_MESSAGE(errors == 0, c.name);
		CHECK_MESSAGE(network_problems(w.doc.map(), w.net()).empty(), c.name);
		w.t().reset(1);
		Invariants inv;
		for (int i = 0; i < 6000; ++i) {
			w.t().tick();
			if (i % 10 == 0) inv.check(w.net(), w.t());
		}
		const TrafficStats st = w.t().stats();
		std::printf("%s: %llu trips in 10 min, %u cars now, longest stop %.0f s\n", c.name,
				static_cast<unsigned long long>(st.arrived), st.vehicles, st.max_stopped);
		CHECK_MESSAGE(st.arrived > 20, c.name);
		CHECK_MESSAGE(st.removed_stuck == 0, c.name);
		CHECK_MESSAGE(inv.overlaps == 0, c.name);
		CHECK_MESSAGE(inv.box_violations == 0, c.name);
	}
}

// --- Determinism ------------------------------------------------------------------------

TEST_CASE("determinism: same map and seed give the same hash") {
	const uint64_t a = run_traffic_golden(1500);
	const uint64_t b = run_traffic_golden(1500);
	CHECK(a == b);
	World w;
	build_test_grid(w.doc, 7, 8, 120.0);
	w.sync();
	w.t().config().demand = TrafficGolden::kDemand;
	w.t().config().max_vehicles = TrafficGolden::kMaxVehicles;
	w.t().reset(TrafficGolden::kSeed + 1);
	w.run_for(150.0);
	CHECK(w.t().state_hash() != a);
	// A recompile of an unchanged map keeps every car where it was.
	std::vector<std::pair<VehicleId, double>> pos;
	for (const Vehicle &v : w.t().vehicles()) pos.push_back({ v.id, v.s });
	w.run.invalidate();
	w.sync();
	REQUIRE(w.t().vehicles().size() == pos.size());
	for (size_t i = 0; i < pos.size(); ++i) {
		CHECK(w.t().vehicles()[i].id == pos[i].first);
		CHECK(w.t().vehicles()[i].s == pos[i].second);
	}
}

TEST_CASE("M2 golden scenario hash (cross-platform determinism)") {
	double us = 0.0;
	TrafficStats st;
	const std::string h = hash_to_hex(run_traffic_golden(TrafficGolden::kTicks, &st, &us));
	std::printf("M2 golden: %s (expected %s), %u cars, %llu trips, %.1f us/tick\n", h.c_str(), TrafficGolden::kExpectedHash,
			st.vehicles, static_cast<unsigned long long>(st.arrived), us);
	CHECK(h == TrafficGolden::kExpectedHash);
}

// --- Gate ----------------------------------------------------------------------------------

TEST_CASE("M2 gate: 500 cars run 1 sim hour on the test grid with no gridlock") {
	World w;
	build_test_grid(w.doc, 7, 8, 120.0);
	w.sync();
	Traffic &t = w.t();
	t.config().demand = TrafficGolden::kDemand;
	t.config().max_vehicles = 500;
	t.reset(2026);
	Invariants inv;
	double worst_stop = 0.0, worst_junction = 0.0;
	uint32_t min_cars = 1000000;
	uint64_t last_arrived = 0, min_window = ~0ull;
	std::vector<uint64_t> windows;
	for (int minute = 1; minute <= 60; ++minute) {
		for (int i = 0; i < 600; ++i) {
			t.tick();
			if (i % 50 == 0) {
				const TrafficStats st = t.stats();
				worst_stop = std::max(worst_stop, st.max_stopped);
				worst_junction = std::max(worst_junction, st.max_junction_wait);
			}
			if (i % 100 == 0) inv.check(w.net(), t);
		}
		const TrafficStats st = t.stats();
		if (minute >= 5) min_cars = std::min(min_cars, st.vehicles);
		if (minute % 5 == 0) {
			windows.push_back(st.arrived - last_arrived);
			if (minute > 5) min_window = std::min(min_window, st.arrived - last_arrived);
			last_arrived = st.arrived;
		}
	}
	const TrafficStats st = t.stats();
	std::printf("M2 gate: %llu trips in 1 h, %u cars at the end (min %u after warm-up), mean speed %.1f km/h\n",
			static_cast<unsigned long long>(st.arrived), st.vehicles, min_cars, st.mean_speed * 3.6);
	std::printf("         longest stop %.0f s, longest a junction stayed stuck by itself %.0f s, trips per 5 min:",
			worst_stop, worst_junction);
	for (uint64_t n : windows) std::printf(" %llu", static_cast<unsigned long long>(n));
	std::printf("\n         lane changes %llu, reroutes %llu, deadlock breaks %llu\n",
			static_cast<unsigned long long>(st.lane_changes), static_cast<unsigned long long>(st.reroutes),
			static_cast<unsigned long long>(st.forced_grants));
	CHECK(min_cars >= 480); // the grid really holds 500 cars
	CHECK(st.removed_stuck == 0); // nobody had to be taken off the map
	CHECK(worst_stop < 600.0); // no car stood still for 10 minutes: no gridlock
	CHECK(worst_junction < 90.0); // no junction stays stuck by itself: no deadlock
	CHECK(min_window * 2 > windows.front()); // throughput holds up over the hour
	CHECK(inv.overlaps == 0);
	CHECK(inv.box_violations == 0);
}
