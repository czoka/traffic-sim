// M4 tests: paths, crossings, fences, ramps and the bridge tool, the
// pedestrian network, pedestrians, mode choice, passengers and the M4 gate.
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include <doctest/doctest.h>

#include "tsim/demo_maps.h"
#include "tsim/document.h"
#include "tsim/network.h"
#include "tsim/road_geometry.h"
#include "tsim/road_map_json.h"
#include "tsim/traffic_run.h"
#include "tsim/validation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <queue>
#include <set>
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
	int errors() {
		geom.build(doc.map());
		int n = 0;
		for (const Problem &p : validate(doc.map(), geom)) {
			if (p.severity == Severity::Error) {
				std::printf("  problem: %s\n", p.message.c_str());
				++n;
			}
		}
		return n;
	}
};

PointRef free_point(double x, double y) {
	PointRef p;
	p.pos = Vec2{ x, y };
	return p;
}

Profile preset(const char *name) {
	RoadMap scratch;
	return preset_profile(name, scratch);
}

Spawner spawner(double rate, bool sink = true, double people = 0.0) {
	Spawner s;
	s.enabled = true;
	s.rate = rate;
	s.sink = sink;
	s.people = people;
	return s;
}

bool has_problem(Document &doc, const char *code) {
	RoadGeometry geom;
	geom.build(doc.map());
	for (const Problem &p : validate(doc.map(), geom)) {
		if (p.code == code) return true;
	}
	return false;
}

// A 400 m street from x = -200 to 200 with cars from the west, and a
// mid-block crossing of `kind` in the middle.
struct Street {
	World w;
	SegmentId road = kNoId;
	int32_t crossing = -1; // NetCrossing index
	int32_t a = -1, b = -1; // its two kerb nodes
};

void build_street(Street &st, CrossingKind kind, double rate, bool refuge = false, const char *profile = "Street 1+1") {
	Document &doc = st.w.doc;
	st.road = doc.add_road({ free_point(-200, 0), free_point(200, 0) }, preset(profile), 0, 13.9).front();
	const RoadSegment *seg = doc.map().segment(st.road);
	doc.set_spawner(seg->from, spawner(rate));
	doc.set_spawner(seg->to, spawner(0.0));
	if (kind != CrossingKind::None) doc.add_crossing(st.road, 0.5, kind, false, refuge);
	st.w.sync();
	const PedGraph &g = st.w.net().ped;
	for (size_t c = 0; c < g.crossings.size(); ++c) {
		const NetCrossing &nc = g.crossings[c];
		if (nc.seg != st.road || nc.unmarked) continue;
		st.crossing = static_cast<int32_t>(c);
		st.a = g.edges[static_cast<size_t>(nc.edges.front())].a;
		st.b = g.edges[static_cast<size_t>(nc.edges.back())].b;
	}
}

// Sends one person across; returns the s it waited at the kerb (-1: never crossed).
// `red_runs` counts cars whose front passed the crossing while people had the walk light.
double send_across(Street &st, double max_wait, int *red_runs = nullptr) {
	Traffic &t = st.w.t();
	const uint32_t id = t.add_pedestrian(st.a, st.b);
	REQUIRE(id != 0);
	const NetCrossing &nc = st.w.net().ped.crossings[static_cast<size_t>(st.crossing)];
	double waited = -1.0;
	for (int k = 0; k < static_cast<int>((max_wait + 60.0) * 10.0); ++k) {
		t.tick();
		if (red_runs && (t.crossing_walk_light(st.crossing) != WalkLight::DontWalk)) {
			for (const Vehicle &v : t.vehicles()) {
				for (const CrossingSpan &sp : nc.spans) {
					if (v.lane == sp.lane && v.prev_lane == sp.lane && v.prev_s < sp.s0 && v.s >= sp.s0) ++*red_runs;
				}
			}
		}
		const int32_t i = t.find_pedestrian(id);
		if (i < 0) break;
		const Pedestrian &p = t.pedestrians()[static_cast<size_t>(i)];
		if (p.state == PedState::WaitingToCross) waited = t.ped_info(id).waited;
		if (p.state == PedState::Crossing && waited < 0.0) waited = 0.0;
	}
	return t.find_pedestrian(id) < 0 ? std::max(0.0, waited) : -1.0;
}

// Ped nodes reachable from a node.
std::vector<char> reach(const PedGraph &g, int32_t from) {
	std::vector<char> seen(g.nodes.size(), 0);
	std::vector<int32_t> stack = { from };
	seen[static_cast<size_t>(from)] = 1;
	while (!stack.empty()) {
		const int32_t n = stack.back();
		stack.pop_back();
		for (int32_t e : g.adj[static_cast<size_t>(n)]) {
			const PedEdge &pe = g.edges[static_cast<size_t>(e)];
			const int32_t m = pe.a == n ? pe.b : pe.a;
			if (!seen[static_cast<size_t>(m)]) {
				seen[static_cast<size_t>(m)] = 1;
				stack.push_back(m);
			}
		}
	}
	return seen;
}

} // namespace

TEST_CASE("people town: valid, round-trips, and its pedestrian network joins up") {
	World w;
	build_people_town(w.doc);
	CHECK(w.errors() == 0);
	const std::string json = road_map_to_json(w.doc.map());
	RoadMap back;
	std::string err;
	REQUIRE_MESSAGE(road_map_from_json(json, back, &err), err);
	CHECK(road_map_to_json(back) == json);
	w.sync();
	for (const NetProblem &p : network_problems(w.doc.map(), w.net())) {
		CHECK_MESSAGE(false, "network problem: ", p.message);
	}
	const PedGraph &g = w.net().ped;
	std::map<std::string, int> kinds;
	for (const NetCrossing &c : g.crossings) {
		const std::string k = c.unmarked ? "unmarked" : crossing_kind_name(c.kind);
		++kinds[k];
		CHECK_MESSAGE(!c.spans.empty(), "crossing on segment ", c.seg, " (", k, ") has no car lanes");
	}
	for (const auto &kv : kinds) std::printf("people town crossings: %s %d\n", kv.first.c_str(), kv.second);
	std::printf("people town ped graph: %zu nodes, %zu edges, %zu stops, %zu spawn points\n", g.nodes.size(),
			g.edges.size(), g.stops.size(), g.spawners.size());
	CHECK(kinds["signal"] == 4);
	CHECK(kinds["zebra"] >= 5);
	CHECK(kinds["uncontrolled"] >= 1);
	CHECK(kinds["unmarked"] >= 4);
	CHECK(g.stops.size() == 4);
	REQUIRE(g.spawners.size() >= 7);
	// Every spawn point can reach every other one on foot.
	const std::vector<char> seen = reach(g, g.spawners[0].entries[0]);
	for (const PedSpawner &sp : g.spawners) {
		bool ok = false;
		for (int32_t e : sp.entries) ok |= seen[static_cast<size_t>(e)] != 0;
		CHECK_MESSAGE(ok, "spawn point at node ", sp.node, " is cut off");
	}
	// The bridge: a footpath on level 1 between two ramps.
	int ramps = 0;
	for (const PedEdge &e : g.edges) ramps += e.kind == PedEdgeKind::Ramp ? 1 : 0;
	CHECK(ramps >= 2);
}

TEST_CASE("people town: people walk, cross, ride the bus and arrive") {
	World w;
	build_people_town(w.doc);
	w.sync();
	w.t().reset(7);
	w.run_for(1800.0);
	const TrafficStats st = w.t().stats();
	std::printf("people town 30 min: %llu trips (walk %llu, bus %llu, bike %llu, car %llu, coach %llu), %llu arrived, "
				"%u on foot, %u riding\n",
			(unsigned long long)st.trips, (unsigned long long)st.trips_walk, (unsigned long long)st.trips_bus,
			(unsigned long long)st.trips_bike, (unsigned long long)st.trips_car, (unsigned long long)st.trips_coach,
			(unsigned long long)st.people_arrived, st.pedestrians, st.riding);
	std::printf("  boarded %llu, alighted %llu, left behind %llu, mean wait %.0f s; crossings %llu, mean kerb wait %.1f s, "
				"cars yielded %llu; %u vehicles, %llu unroutable, %llu stuck\n",
			(unsigned long long)st.boarded, (unsigned long long)st.alighted, (unsigned long long)st.left_behind, st.mean_wait,
			(unsigned long long)st.crossings, st.mean_crossing_wait, (unsigned long long)st.cars_yielded, st.vehicles,
			(unsigned long long)st.unroutable, (unsigned long long)st.removed_stuck);
	CHECK(st.trips > 100);
	CHECK(st.trips_walk > 0);
	CHECK(st.trips_bus > 0);
	CHECK(st.people_arrived > (st.trips_walk + st.trips_bus) / 3);
	CHECK(st.boarded > 0);
	CHECK(st.alighted > 0);
	CHECK(st.crossings > 50);
	CHECK(st.cars_yielded > 0);
	CHECK(st.removed_stuck == 0);
	// Nobody stands at a kerb for ages.
	double longest = 0.0;
	for (const Pedestrian &p : w.t().pedestrians()) {
		if (p.state == PedState::WaitingToCross) longest = std::max(longest, w.t().ped_info(p.id).waited);
	}
	CHECK(longest < 120.0);
}

TEST_CASE("zebra: people get priority and cars give way") {
	Street st;
	build_street(st, CrossingKind::Zebra, 900.0);
	REQUIRE(st.crossing >= 0);
	st.w.t().reset(3);
	st.w.run_for(60.0);
	const uint64_t yielded0 = st.w.t().stats().cars_yielded;
	double worst = 0.0;
	for (int k = 0; k < 5; ++k) {
		const double w = send_across(st, 60.0);
		REQUIRE_MESSAGE(w >= 0.0, "person ", k, " never got across the zebra");
		worst = std::max(worst, w);
		st.w.run_for(5.0);
	}
	std::printf("zebra: longest kerb wait %.1f s, %llu cars gave way\n", worst,
			(unsigned long long)(st.w.t().stats().cars_yielded - yielded0));
	CHECK(worst < 10.0);
	CHECK(st.w.t().stats().cars_yielded > yielded0);
	CHECK(st.w.t().stats().removed_stuck == 0);
}

TEST_CASE("uncontrolled crossing: people wait for a gap") {
	Street zebra, gap;
	build_street(zebra, CrossingKind::Zebra, 900.0);
	build_street(gap, CrossingKind::Uncontrolled, 900.0);
	REQUIRE(gap.crossing >= 0);
	double wz = 0.0, wg = 0.0;
	for (Street *s : { &zebra, &gap }) {
		s->w.t().reset(11);
		s->w.run_for(60.0);
		double sum = 0.0;
		for (int k = 0; k < 6; ++k) {
			const double w = send_across(*s, 120.0);
			REQUIRE(w >= 0.0);
			sum += w;
		}
		(s == &zebra ? wz : wg) = sum / 6.0;
	}
	std::printf("mean kerb wait: zebra %.1f s, uncontrolled %.1f s\n", wz, wg);
	CHECK(wg > wz);
	// With no traffic there is nothing to wait for.
	Street empty;
	build_street(empty, CrossingKind::Uncontrolled, 0.0);
	empty.w.t().reset(1);
	CHECK(send_across(empty, 10.0) < 0.5);
}

TEST_CASE("push-button signal: cars get red only when someone waits, and stop for it") {
	Street st;
	build_street(st, CrossingKind::Signal, 1200.0, true, "Avenue 2+2, median");
	REQUIRE(st.crossing >= 0);
	const NetCrossing &nc = st.w.net().ped.crossings[static_cast<size_t>(st.crossing)];
	CHECK(nc.push_button);
	CHECK(nc.edges.size() == 2); // a refuge in the middle
	Traffic &t = st.w.t();
	t.reset(5);
	st.w.run_for(90.0);
	CHECK(t.crossing_car_light(st.crossing) == SignalLight::Green); // nobody pressed
	int red_runs = 0;
	const double w = send_across(st, 60.0, &red_runs);
	REQUIRE(w >= 0.0);
	std::printf("push button: waited %.1f s, %d cars ran the red\n", w, red_runs);
	CHECK(w <= 20.0 + 3.0 + 1.0 + 0.5);
	CHECK(red_runs == 0);
	st.w.run_for(30.0);
	CHECK(t.crossing_car_light(st.crossing) == SignalLight::Green); // back to green
}

TEST_CASE("signal junction: walk lights follow the plan, with a flashing clearance") {
	World w;
	build_people_town(w.doc);
	w.sync();
	const PedGraph &g = w.net().ped;
	int32_t ci = -1;
	for (size_t c = 0; c < g.crossings.size(); ++c) {
		if (g.crossings[c].kind == CrossingKind::Signal && g.crossings[c].junction >= 0) ci = static_cast<int32_t>(c);
	}
	REQUIRE(ci >= 0);
	Traffic &t = w.t();
	t.reset(1);
	std::map<WalkLight, int> seen;
	const NetJunction &j = w.net().junctions[static_cast<size_t>(g.crossings[static_cast<size_t>(ci)].junction)];
	for (int64_t k = 0; k < j.signal.cycle + 10; ++k) {
		++seen[t.crossing_walk_light(ci)];
		t.tick();
	}
	CHECK(seen[WalkLight::Walk] > 40); // at least 4 s
	CHECK(seen[WalkLight::Flashing] > 0);
	CHECK(seen[WalkLight::DontWalk] > 0);
}

TEST_CASE("buses: capacity, people left behind, boarding dwell and route loads") {
	World w;
	build_people_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().bus_capacity = 4;
	t.config().demand = 2.0;
	t.config().car_owners = 0.0;
	t.config().bike_owners = 0.0;
	t.reset(9);
	size_t most = 0;
	for (int k = 0; k < 18000; ++k) {
		t.tick();
		for (const Vehicle &v : t.vehicles()) most = std::max(most, v.riders.size());
	}
	const TrafficStats st = t.stats();
	CHECK(most <= 4);
	CHECK(most > 0);
	CHECK(st.left_behind > 0);
	CHECK(st.trips_car == 0);
	CHECK(st.trips_bike == 0);
	uint64_t boarded = 0;
	for (const StopStats &ss : t.stop_stats()) boarded += ss.boarded;
	CHECK(boarded == st.boarded);
	const std::vector<RouteLoad> loads = t.route_loads();
	REQUIRE(loads.size() == 1);
	double peak = 0.0;
	for (double l : loads[0].load) peak = std::max(peak, l);
	CHECK(peak > 0.0);
	CHECK(peak <= 4.0);
	std::printf("buses with 4 seats: boarded %llu, left behind %llu, peak mean load %.1f\n", (unsigned long long)st.boarded,
			(unsigned long long)st.left_behind, peak);
}

TEST_CASE("mode choice: walk, bus, bike and car all get picked; people without a car or bike don't") {
	World w;
	build_people_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.reset(2);
	w.run_for(1200.0);
	TrafficStats st = t.stats();
	CHECK(st.trips_walk > 0);
	CHECK(st.trips_bus > 0);
	CHECK(st.trips_bike > 0);
	CHECK(st.trips_car > 0);
	CHECK(st.trips == st.trips_walk + st.trips_bus + st.trips_bike + st.trips_car + st.trips_coach);
	t.config().car_owners = 0.0;
	t.config().bike_owners = 0.0;
	t.reset(2);
	w.run_for(1200.0);
	st = t.stats();
	CHECK(st.trips_bike == 0);
	CHECK(st.trips_car == 0);
	CHECK(st.trips_walk > 0);
}

TEST_CASE("ramps: grade limits for roads and paths, stairs for steep footpaths") {
	RoadMap scratch;
	{
		Document doc;
		const SegmentId r = doc.add_road({ free_point(0, 0), free_point(60, 0) }, preset("Street 1+1"), 0, 13.9).front();
		doc.set_ramp(r, 1, false); // 5 m over 60 m: 8.3 %
		CHECK(has_problem(doc, "ramp_steep"));
		Document ok;
		const SegmentId r2 = ok.add_road({ free_point(0, 0), free_point(90, 0) }, preset("Street 1+1"), 0, 13.9).front();
		ok.set_ramp(r2, 1, false); // 5.6 %
		CHECK(!has_problem(ok, "ramp_steep"));
	}
	{
		Document doc;
		const SegmentId f = doc.add_road({ free_point(0, 0), free_point(70, 0) }, path_profile(SegmentKind::Footpath, scratch),
				0, 1.4, SegmentKind::Footpath).front();
		doc.set_ramp(f, 1, false); // 7.1 %: fine on foot
		CHECK(!has_problem(doc, "ramp_steep"));
		const SegmentId s = doc.add_road({ free_point(0, 50), free_point(15, 50) }, path_profile(SegmentKind::Footpath, scratch),
				0, 1.4, SegmentKind::Footpath).front();
		doc.set_ramp(s, 1, false);
		CHECK(has_problem(doc, "ramp_steep"));
		doc.set_ramp(s, 1, true); // stairs
		CHECK(!has_problem(doc, "ramp_steep"));
		World w;
		w.doc.reset(doc.map());
		w.sync();
		int stairs = 0;
		for (const PedEdge &e : w.net().ped.edges) stairs += e.kind == PedEdgeKind::Stairs ? 1 : 0;
		CHECK(stairs == 1);
	}
}

TEST_CASE("bridge tool: lifts a road over another, with ramps within the grade, and cars drive over it") {
	World w;
	Document &doc = w.doc;
	const Profile p = preset("Street 1+1");
	const SegmentId ew = doc.add_road({ free_point(-300, 0), free_point(300, 0) }, p, 0, 13.9).front();
	const SegmentId ns = doc.add_road({ free_point(0, -300), free_point(0, 300) }, p, 0, 13.9).front();
	CHECK(doc.map().segment(ew)->to != doc.map().segment(ns)->from);
	CHECK(w.errors() > 0); // the two roads overlap on one level
	const Document::LiftPlan plan = doc.plan_lift(ns, Vec2{ 0, 0 }, 1);
	REQUIRE_MESSAGE(plan.ok, plan.error);
	CHECK(plan.s[0] < 300.0);
	CHECK(plan.s[3] > 300.0);
	CHECK(doc.lift(ns, Vec2{ 0, 0 }, 1).empty());
	CHECK(w.errors() == 0);
	int ramps = 0, raised = 0;
	for (const auto &kv : doc.map().segments()) {
		const RoadSegment &s = kv.second;
		if (s.is_ramp()) {
			++ramps;
			CHECK(s.max_grade() <= kMaxGradeRoad + 1e-9);
		}
		raised += s.level == 1 && !s.is_ramp() ? 1 : 0;
	}
	CHECK(ramps == 2);
	CHECK(raised == 1);
	CHECK(doc.undo());
	CHECK(w.errors() > 0);
	CHECK(doc.redo());
	// Cars from the south reach the north end over the bridge.
	NodeId south = kNoId, north = kNoId;
	for (const auto &kv : doc.map().nodes()) {
		if (std::abs(kv.second.pos.x) < 1e-6 && kv.second.pos.y > 299.0) north = kv.first;
		if (std::abs(kv.second.pos.x) < 1e-6 && kv.second.pos.y < -299.0) south = kv.first;
	}
	REQUIRE(south != kNoId);
	REQUIRE(north != kNoId);
	doc.set_spawner(south, spawner(600.0));
	doc.set_spawner(north, spawner(0.0));
	w.sync();
	w.t().reset(4);
	bool up = false;
	for (int k = 0; k < 3000; ++k) {
		w.t().tick();
		for (size_t i = 0; i < w.t().vehicles().size(); ++i) up |= w.t().level_of(i) == 1;
	}
	const TrafficStats st = w.t().stats();
	CHECK(st.arrived > 20);
	CHECK(st.removed_stuck == 0);
	CHECK(up);
}

TEST_CASE("fences stop informal crossings; marked ones stay") {
	Street st;
	build_street(st, CrossingKind::Zebra, 0.0);
	auto count = [&](bool unmarked) {
		int n = 0;
		for (const NetCrossing &c : st.w.net().ped.crossings) n += c.seg == st.road && c.unmarked == unmarked ? 1 : 0;
		return n;
	};
	CHECK(count(true) > 0);
	CHECK(count(false) == 1);
	st.w.doc.set_fence(st.road, 0, 0.0, 1.0, true);
	st.w.doc.set_fence(st.road, 1, 0.0, 1.0, true);
	CHECK(st.w.doc.map().segment(st.road)->fences.size() == 2);
	st.w.sync();
	CHECK(count(true) == 0);
	CHECK(count(false) == 1);
	// Erasing the middle splits a fence in two.
	st.w.doc.set_fence(st.road, 0, 0.4, 0.6, false);
	CHECK(st.w.doc.map().segment(st.road)->fences.size() == 3);
}

TEST_CASE("people sim: same seed, same hash; carried over when the map changes") {
	auto run = [](uint64_t seed) {
		World w;
		build_people_town(w.doc);
		w.sync();
		w.t().reset(seed);
		w.run_for(600.0);
		return w.t().state_hash();
	};
	const uint64_t a = run(42);
	CHECK(a == run(42));
	CHECK(run(43) != a);
	std::printf("M4 golden: %016llx (expected 7fbf36952357709f)\n", static_cast<unsigned long long>(a));
	CHECK(a == 0x7fbf36952357709full); // same on every platform, like the M2 and M3 goldens
	World w;
	build_people_town(w.doc);
	w.sync();
	w.t().reset(8);
	w.run_for(600.0);
	const size_t before = w.t().pedestrians().size();
	REQUIRE(before > 20);
	// An unrelated edit far away: a new street spur off the map's east end.
	w.doc.add_road({ free_point(900, 900), free_point(1000, 900) }, preset("Street 1+1"), 0, 13.9);
	w.sync();
	CHECK(w.t().pedestrians().size() >= before * 9 / 10);
	w.run_for(300.0);
	CHECK(w.t().stats().removed_stuck == 0);
}

TEST_CASE("M4 gate: 2,000 vehicles and 1,000 people on the people city") {
	World w;
	build_people_city(w.doc, 12, 12);
	CHECK(w.errors() == 0);
	w.sync();
	Traffic &t = w.t();
	t.config().max_vehicles = 2000;
	t.config().max_pedestrians = 1000;
	t.config().demand = 3.0;
	t.reset(2026);
	// Warm up until both caps are reached (at most 40 sim minutes).
	int warm = 0;
	for (; warm < 24000; ++warm) {
		t.tick();
		const TrafficStats st = t.stats();
		if (st.vehicles >= 1990 && st.pedestrians + st.riding >= 990) break;
	}
	TrafficStats st = t.stats();
	std::printf("M4 gate: %zu lanes, %zu junctions, %zu crossings, %zu ped nodes; full after %.0f s: %u vehicles, %u people\n",
			w.net().lanes.size(), w.net().junctions.size(), w.net().ped.crossings.size(), w.net().ped.nodes.size(),
			warm * 0.1, st.vehicles, st.pedestrians + st.riding);
	CHECK(st.vehicles >= 1990);
	CHECK(st.pedestrians + st.riding >= 990);
	const int ticks = 3000;
	const auto t0 = std::chrono::steady_clock::now();
	uint32_t min_veh = ~0u, min_ped = ~0u;
	for (int k = 0; k < ticks; ++k) {
		t.tick();
		if (k % 100 == 0) {
			const TrafficStats s = t.stats();
			min_veh = std::min(min_veh, s.vehicles);
			min_ped = std::min(min_ped, s.pedestrians + s.riding);
		}
	}
	const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / ticks;
	st = t.stats();
	std::printf("M4 gate: %.0f us/tick over 5 sim minutes (at least %u vehicles, %u people); %llu people arrived, "
				"%llu crossings, mean kerb wait %.1f s, %llu cars gave way, %llu removed as stuck\n",
			us, min_veh, min_ped, (unsigned long long)st.people_arrived, (unsigned long long)st.crossings,
			st.mean_crossing_wait, (unsigned long long)st.cars_yielded, (unsigned long long)st.removed_stuck);
	double longest = 0.0;
	for (const Pedestrian &p : t.pedestrians()) {
		if (p.state != PedState::WaitingToCross) continue;
		const double wt = t.ped_info(p.id).waited;
		longest = std::max(longest, wt);
		if (wt > 120.0) {
			const PedEdge &e = w.net().ped.edges[static_cast<size_t>(p.edge)];
			const NetCrossing &c = w.net().ped.crossings[static_cast<size_t>(e.crossing)];
			std::printf("  waiting %.0f s at crossing %d (%s%s, seg %u end %d, junction %d, %zu spans, light %d)\n", wt,
					e.crossing, c.unmarked ? "unmarked " : "", crossing_kind_name(c.kind), c.seg, c.end, c.junction,
					c.spans.size(), static_cast<int>(t.crossing_walk_light(e.crossing)));
			for (const CrossingSpan &sp : c.spans) {
				const NetLane &l = w.net().lanes[static_cast<size_t>(sp.lane)];
				std::printf("    lane %d kind %d len %.1f s %.1f-%.1f:", sp.lane, static_cast<int>(l.kind), l.length, sp.s0, sp.s1);
				for (const Vehicle &v : t.vehicles()) {
					if (v.lane == sp.lane) std::printf(" [%u s=%.1f v=%.1f %s]", v.id, v.s, v.v, vehicle_state_name(v.state));
				}
				std::printf("\n");
			}
		}
	}
	std::printf("M4 gate: longest kerb wait now %.0f s\n", longest);
	CHECK(min_veh >= 1800);
	CHECK(min_ped >= 900);
	CHECK(st.people_arrived > 100); // the grid is 1.4 km across: most walks take over 10 minutes
	CHECK(longest < 120.0);
	CHECK(st.removed_stuck < 20);
}
