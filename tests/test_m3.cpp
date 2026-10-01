// M3 tests: roundabouts, signals, lane classes, parking, bikes, taxis, transit
// and the M3 gate (the showcase map runs a full sim day).
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
#include <cmath>
#include <cstdio>
#include <map>
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

struct Cross {
	NodeId c = 0, w = 0, e = 0, n = 0, s = 0;
	SegmentId sw = 0, se = 0, sn = 0, ss = 0;
};

// Four-way junction at the origin with 150 m arms and silent spawn points.
Cross cross_roads(Document &doc, const char *profile = "Street 1+1") {
	Cross x;
	const Profile p = preset(profile);
	x.sw = doc.add_road({ free_point(-150, 0), free_point(0, 0) }, p, 0, 13.9).front();
	x.c = doc.map().segment(x.sw)->to;
	x.w = doc.map().segment(x.sw)->from;
	x.se = doc.add_road({ node_point(doc, x.c), free_point(150, 0) }, p, 0, 13.9).front();
	x.e = doc.map().segment(x.se)->to;
	x.sn = doc.add_road({ free_point(0, -150), node_point(doc, x.c) }, p, 0, 13.9).front();
	x.n = doc.map().segment(x.sn)->from;
	x.ss = doc.add_road({ node_point(doc, x.c), free_point(0, 150) }, p, 0, 13.9).front();
	x.s = doc.map().segment(x.ss)->to;
	for (NodeId end : { x.w, x.e, x.n, x.s }) doc.set_spawner(end, spawner(0.0));
	return x;
}

// Kerb-side car lane of a segment in one direction.
int32_t kerb_lane(const Network &net, SegmentId seg, LaneDir dir) {
	for (size_t i = 0; i < net.lanes.size(); ++i) {
		const NetLane &l = net.lanes[i];
		if (l.kind == NetLaneKind::Road && !l.ring && l.segment == seg && l.dir == dir && !l.pocket && l.right < 0 &&
				l.type != LaneType::Bike) {
			return static_cast<int32_t>(i);
		}
	}
	return -1;
}

const Vehicle *find(const Traffic &t, VehicleId id) {
	const int32_t i = t.find_vehicle(id);
	return i < 0 ? nullptr : &t.vehicles()[static_cast<size_t>(i)];
}

// No two vehicles overlap in a lane (parked ones are beside it).
int overlaps(const Traffic &t) {
	std::map<int32_t, std::vector<const Vehicle *>> by_lane;
	for (const Vehicle &v : t.vehicles()) {
		if (!v.off_lane) by_lane[v.lane].push_back(&v);
	}
	int n = 0;
	for (auto &kv : by_lane) {
		auto &vs = kv.second;
		std::sort(vs.begin(), vs.end(), [](const Vehicle *a, const Vehicle *b) { return a->s > b->s; });
		for (size_t k = 1; k < vs.size(); ++k) {
			if (vs[k - 1]->s - vs[k - 1]->drv.length - vs[k]->s < -0.2) ++n;
		}
	}
	return n;
}

} // namespace

TEST_CASE("showcase map: every M3 feature, valid, compiles and round-trips") {
	World w;
	build_showcase(w.doc);
	int errors = 0;
	w.geom.build(w.doc.map());
	for (const Problem &p : validate(w.doc.map(), w.geom)) {
		if (p.severity == Severity::Error) {
			std::printf("showcase problem: %s\n", p.message.c_str());
			++errors;
		}
	}
	CHECK(errors == 0);
	const std::string json = road_map_to_json(w.doc.map());
	RoadMap back;
	std::string err;
	REQUIRE(road_map_from_json(json, back, &err));
	CHECK(road_map_to_json(back) == json);

	w.sync();
	const Network &n = w.net();
	int signals = 0, roundabouts = 0;
	for (const NetJunction &j : n.junctions) {
		signals += j.signal.enabled ? 1 : 0;
		roundabouts += j.roundabout ? 1 : 0;
	}
	CHECK(signals == 1);
	CHECK(roundabouts == 1);
	std::set<int> styles;
	for (const NetBay &b : n.bays) styles.insert(static_cast<int>(b.style));
	CHECK(styles.size() == 3);
	CHECK(n.stops.size() == 6);
	REQUIRE(n.main_station >= 0);
	CHECK(n.stops[static_cast<size_t>(n.main_station)].bays == 3);
	REQUIRE(n.depots.size() == 1);
	CHECK(n.depots[0].routes.size() == 2);
	for (const NetRoute &r : n.depots[0].routes) CHECK(r.stops.size() >= 2);
	CHECK(n.coach_lines.size() == 1);
	bool bike_lane = false, bus_lane = false, ring = false;
	for (const NetLane &l : n.lanes) {
		bike_lane |= l.type == LaneType::Bike;
		bus_lane |= l.type == LaneType::Bus;
		ring |= l.ring;
	}
	CHECK(bike_lane);
	CHECK(bus_lane);
	CHECK(ring);
	for (const NetProblem &p : network_problems(w.doc.map(), n)) std::printf("showcase net problem: %s\n", p.message.c_str());
}

TEST_CASE("signal program: green, amber, all-red in ticks") {
	World w;
	const Cross x = cross_roads(w.doc);
	SignalPlan plan;
	SignalPhase ew, ns;
	ew.green = 20.0;
	ew.moves = { { x.sw, x.se, false }, { x.se, x.sw, false } };
	ns.green = 10.0;
	ns.moves = { { x.sn, x.ss, false }, { x.ss, x.sn, false } };
	plan.phases = { ew, ns };
	plan.amber = 3.0;
	plan.all_red = 2.0;
	w.doc.set_signal_plan(x.c, plan);
	w.sync();
	const NetJunction &j = w.net().junctions[static_cast<size_t>(w.net().junction_at(x.c))];
	REQUIRE(j.signal.enabled);
	CHECK(j.signal.cycle == 400); // (20 + 3 + 2) + (10 + 3 + 2) s at 10 Hz
	int32_t mv = -1;
	for (size_t m = 0; m < j.signal.movements.size(); ++m) {
		if (j.signal.movements[m] == std::make_pair(x.sw, x.se)) mv = static_cast<int32_t>(m);
	}
	REQUIRE(mv >= 0);
	CHECK(j.light(mv, 0) == SignalLight::Green);
	CHECK(j.light(mv, 199) == SignalLight::Green);
	CHECK(j.light(mv, 200) == SignalLight::Amber);
	CHECK(j.light(mv, 229) == SignalLight::Amber);
	CHECK(j.light(mv, 230) == SignalLight::Red);
	CHECK(j.light(mv, 399) == SignalLight::Red);
	CHECK(j.light(mv, 400) == SignalLight::Green);
	CHECK(j.phase_at(260) == 1);
}

TEST_CASE("red light: a car waits at the line and goes on green") {
	World w;
	const Cross x = cross_roads(w.doc);
	SignalPlan plan;
	SignalPhase ns, ew;
	ns.green = 20.0;
	ns.moves = { { x.sn, x.ss, false } };
	ew.green = 20.0;
	ew.moves = { { x.sw, x.se, false } };
	plan.phases = { ns, ew }; // west-east starts red
	w.doc.set_signal_plan(x.c, plan);
	w.sync();
	const int32_t lane = kerb_lane(w.net(), x.sw, LaneDir::Forward);
	REQUIRE(lane >= 0);
	const VehicleId id = w.t().add_vehicle(lane, w.net().lanes[static_cast<size_t>(lane)].length - 60.0, 10.0, x.e);
	REQUIRE(id != 0);
	w.run_for(20.0);
	const Vehicle *v = find(w.t(), id);
	REQUIRE(v);
	CHECK(v->lane == lane); // still before the line
	CHECK(v->v < 0.5);
	CHECK(v->state == VehicleState::RedLight);
	CHECK(w.t().stats().red_light_waits >= 1);
	w.run_for(15.0); // green from 25 s
	v = find(w.t(), id);
	CHECK((!v || v->lane != lane));
}

TEST_CASE("right on red with the flashing arrow: stop, yield, turn") {
	World w;
	const Cross x = cross_roads(w.doc);
	SignalPlan plan;
	SignalPhase ns, ew;
	ns.green = 40.0;
	ns.moves = { { x.sn, x.ss, false }, { x.ss, x.sn, false } };
	ew.green = 20.0;
	ew.moves = { { x.sw, x.se, false } };
	plan.phases = { ns, ew };
	plan.right_on_red = { x.sw };
	w.doc.set_signal_plan(x.c, plan);
	w.sync();
	// Coming from the west (eastbound), right is south in a y-down frame.
	const int32_t lane = kerb_lane(w.net(), x.sw, LaneDir::Forward);
	const VehicleId id = w.t().add_vehicle(lane, w.net().lanes[static_cast<size_t>(lane)].length - 50.0, 10.0, x.s);
	REQUIRE(id != 0);
	bool stopped = false;
	for (int i = 0; i < 300; ++i) {
		w.t().tick();
		const Vehicle *v = find(w.t(), id);
		if (!v) break;
		if (v->lane == lane && v->v < 0.02) stopped = true;
		if (v->lane != lane) break;
	}
	CHECK(stopped); // came to a full stop first
	const Vehicle *v = find(w.t(), id);
	CHECK((!v || v->lane != lane)); // and turned while west-east was still red
	CHECK(w.t().stats().right_on_red == 1);

	// Straight on has no arrow: it waits for green.
	World w2;
	const Cross y = cross_roads(w2.doc);
	w2.doc.set_signal_plan(y.c, [&] {
		SignalPlan p = plan;
		p.phases[0].moves = { { y.sn, y.ss, false }, { y.ss, y.sn, false } };
		p.phases[1].moves = { { y.sw, y.se, false } };
		p.right_on_red = { y.sw };
		return p;
	}());
	w2.sync();
	const int32_t l2 = kerb_lane(w2.net(), y.sw, LaneDir::Forward);
	const VehicleId straight = w2.t().add_vehicle(l2, w2.net().lanes[static_cast<size_t>(l2)].length - 50.0, 10.0, y.e);
	w2.run_for(30.0);
	const Vehicle *s = find(w2.t(), straight);
	REQUIRE(s);
	CHECK(s->lane == l2);
}

TEST_CASE("default signal plan: opposite legs share a phase with permissive lefts") {
	World w;
	const Cross x = cross_roads(w.doc);
	const SignalPlan plan = default_signal_plan(w.doc.map(), x.c);
	REQUIRE(plan.phases.size() == 2);
	for (const SignalPhase &ph : plan.phases) {
		bool any_permissive = false;
		for (const SignalMovement &m : ph.moves) any_permissive |= m.permissive;
		CHECK(any_permissive);
	}
	// Traffic from all four sides gets through.
	w.doc.set_signal_plan(x.c, plan);
	for (NodeId end : { x.w, x.e, x.n, x.s }) w.doc.set_spawner(end, spawner(300.0));
	w.sync();
	w.t().reset(7);
	w.run_for(600.0);
	const TrafficStats st = w.t().stats();
	std::printf("signalized cross: %llu trips in 10 min, longest stop %.0f s\n",
			static_cast<unsigned long long>(st.arrived), st.max_stopped);
	CHECK(st.arrived > 120);
	CHECK(st.removed_stuck == 0);
	CHECK(st.max_stopped < 120.0);
	CHECK(overlaps(w.t()) == 0);
}

TEST_CASE("roundabout: ring traffic has priority and every leg flows") {
	for (int lanes = 1; lanes <= 2; ++lanes) {
		World w;
		const Cross x = cross_roads(w.doc, lanes == 1 ? "Street 1+1" : "Avenue 2+2, median");
		Roundabout r;
		r.enabled = true;
		r.radius = lanes == 1 ? 18.0 : 24.0;
		r.lanes = lanes;
		w.doc.set_roundabout(x.c, r);
		for (NodeId end : { x.w, x.e, x.n, x.s }) w.doc.set_spawner(end, spawner(350.0));
		w.sync();
		const NetJunction &j = w.net().junctions[static_cast<size_t>(w.net().junction_at(x.c))];
		CHECK(j.roundabout);
		CHECK(j.arbitrated);
		// Entries give way to the ring.
		int ring_first = 0, entry_conflicts = 0;
		for (int32_t c : j.connectors) {
			const NetLane &cn = w.net().lanes[static_cast<size_t>(c)];
			if (w.net().lanes[static_cast<size_t>(cn.from)].ring) continue;
			for (const Conflict &cf : cn.conflicts) {
				const NetLane &o = w.net().lanes[static_cast<size_t>(cf.other)];
				if (!w.net().lanes[static_cast<size_t>(o.from)].ring || !cf.merge) continue;
				++entry_conflicts;
				ring_first += cf.priority < 0 ? 1 : 0;
			}
		}
		CHECK(entry_conflicts > 0);
		CHECK(ring_first == entry_conflicts);
		w.t().reset(11);
		w.run_for(900.0);
		const TrafficStats st = w.t().stats();
		std::printf("roundabout (%d lane%s): %llu trips in 15 min, longest stop %.0f s, lane changes %llu\n", lanes,
				lanes == 1 ? "" : "s", static_cast<unsigned long long>(st.arrived), st.max_stopped,
				static_cast<unsigned long long>(st.lane_changes));
		CHECK(st.arrived > 200);
		CHECK(st.removed_stuck == 0);
		CHECK(st.max_stopped < 120.0);
		CHECK(overlaps(w.t()) == 0);
	}
}

TEST_CASE("lane classes: bike lanes for bikes, bus lanes cost cars") {
	World w;
	const Cross x = cross_roads(w.doc, "Street 1+1, bike lanes");
	w.sync();
	const Network &n = w.net();
	int bike_lanes = 0;
	for (const NetLane &l : n.lanes) {
		if (l.type != LaneType::Bike) continue;
		++bike_lanes;
		CHECK_FALSE(Traffic::allowed(VehicleKind::Car, l));
		CHECK_FALSE(Traffic::allowed(VehicleKind::Bus, l));
		CHECK(Traffic::allowed(VehicleKind::Bike, l));
		CHECK(l.left < 0); // never a lane-change neighbour of a car lane
	}
	CHECK(bike_lanes >= 8);
	// A bike from the west bike lane reaches the east edge on bike lanes only.
	int32_t west_bike = -1;
	for (size_t i = 0; i < n.lanes.size(); ++i) {
		const NetLane &l = n.lanes[i];
		if (l.kind == NetLaneKind::Road && l.segment == x.sw && l.dir == LaneDir::Forward && l.type == LaneType::Bike) {
			west_bike = static_cast<int32_t>(i);
		}
	}
	REQUIRE(west_bike >= 0);
	std::vector<int32_t> route;
	REQUIRE(w.t().find_route(west_bike, x.e, route, true, VehicleKind::Bike));
	REQUIRE(route.size() == 1);
	CHECK(n.lanes[static_cast<size_t>(n.lanes[static_cast<size_t>(route[0])].to)].type == LaneType::Bike);
	CHECK_FALSE(w.t().find_route(west_bike, x.e, route, true, VehicleKind::Car));
	const VehicleId b = w.t().add_vehicle(west_bike, 20.0, 5.0, x.e, nullptr, VehicleKind::Bike);
	REQUIRE(b != 0);
	CHECK(find(w.t(), b)->drv.max_speed < 7.0);
	w.run_for(90.0);
	CHECK(w.t().stats().bikes_arrived == 1);

	// Cars on an avenue with bus lanes keep out of them except to turn.
	World a;
	const Cross y = cross_roads(a.doc, "Avenue 2+2, bus lanes, parking");
	TrafficConfig &cfg = a.t().config();
	cfg.park_share = 0.0;
	cfg.taxi_share = 0.0;
	a.doc.set_spawner(y.w, spawner(900.0));
	a.sync();
	a.t().reset(5);
	a.run_for(600.0);
	const TrafficStats st = a.t().stats();
	CHECK(st.arrived > 60);
	CHECK(st.bus_lane_misuse < 0.5 * static_cast<double>(st.arrived)); // only to turn, or just after turning in
}

TEST_CASE("parking: cars pull into free bays, stay, and leave") {
	World w;
	const Profile p = preset("Street 1+1, parking");
	const SegmentId s = w.doc.add_road({ free_point(-300, 0), free_point(300, 0) }, p, 0, 13.9).front();
	w.doc.set_spawner(w.doc.map().segment(s)->from, spawner(600.0));
	w.doc.set_spawner(w.doc.map().segment(s)->to, spawner(600.0));
	w.sync();
	REQUIRE(!w.net().bays.empty());
	Traffic &t = w.t();
	t.config().park_share = 0.5;
	t.config().park_min = 60.0;
	t.config().park_max = 120.0;
	t.reset(3);
	uint32_t most_parked = 0;
	for (int k = 0; k < 60; ++k) {
		w.run_for(10.0);
		const TrafficStats st = t.stats();
		most_parked = std::max(most_parked, st.parked);
		std::set<VehicleId> owners;
		for (VehicleId id : t.bay_use()) {
			if (id == 0) continue;
			CHECK(owners.insert(id).second); // one bay per car
		}
		CHECK(overlaps(t) == 0);
	}
	const TrafficStats st = t.stats();
	std::printf("parking street: %llu parkings, %u parked at most, %llu without a free bay, %llu trips\n",
			static_cast<unsigned long long>(st.parkings), most_parked, static_cast<unsigned long long>(st.parking_failed),
			static_cast<unsigned long long>(st.arrived));
	CHECK(most_parked >= 3);
	CHECK(st.parkings >= 5); // parked and left again
	CHECK(st.removed_stuck == 0);
	// Parked cars are drawn in their bay, beside the lane.
	for (size_t i = 0; i < t.vehicles().size(); ++i) {
		const Vehicle &v = t.vehicles()[i];
		if (!v.off_lane) continue;
		const Pose lane_pose = w.net().pose(v.lane, v.s);
		CHECK((t.pose(i, 0.5).pos - lane_pose.pos).length() > 1.0);
	}
}

TEST_CASE("buses: depot, stops in order, dwell, back to the depot") {
	World w;
	build_showcase(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.reset(21);
	const std::vector<RouteStats> rs0 = t.route_stats();
	REQUIRE(rs0.size() == 2);
	for (const RouteStats &r : rs0) {
		CHECK(r.round_trip > 60.0);
		CHECK(r.fleet >= 1);
	}
	const NetDepot &d = w.net().depots[0];
	CHECK(rs0[0].fleet == static_cast<uint32_t>(std::ceil(rs0[0].round_trip / d.routes[0].headway)));
	// Follow the first bus of route 1 through its stops.
	w.run_for(1.0);
	VehicleId bus = 0;
	for (const Vehicle &v : t.vehicles()) {
		if (v.kind == VehicleKind::Bus && v.bus_route == d.routes[0].id) bus = v.id;
	}
	REQUIRE(bus != 0);
	const size_t n_stops = find(t, bus)->waypoints.size();
	CHECK(n_stops == d.routes[0].stops.size() + 1); // a loop ends where it started
	std::vector<int32_t> served;
	bool dwelled = false;
	for (int i = 0; i < 36000; ++i) {
		t.tick();
		const Vehicle *v = find(t, bus);
		if (!v) break;
		if (v->phase != 0) dwelled = true;
		if (v->phase != 0 && (served.empty() || served.back() != v->waypoints[v->wi].stop)) {
			served.push_back(v->waypoints[v->wi].stop);
		}
	}
	CHECK(dwelled);
	CHECK(find(t, bus) == nullptr); // back in the depot
	std::vector<int32_t> expect = d.routes[0].stops;
	expect.push_back(expect.front());
	CHECK(served == expect);
	const TrafficStats st = t.stats();
	CHECK(st.bus_runs >= 1);
	CHECK(st.bus_stops_served >= expect.size());
	std::printf("bus route 1: round trip %.0f s, fleet %u, %llu runs so far\n", rs0[0].round_trip, rs0[0].fleet,
			static_cast<unsigned long long>(st.bus_runs));
}

TEST_CASE("coaches call at the main station; taxis and bikes join the traffic") {
	World w;
	build_showcase(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.reset(33);
	uint32_t taxis = 0, cars = 0, bikes = 0, at_station = 0;
	for (int k = 0; k < 360; ++k) {
		w.run_for(10.0);
		const TrafficStats st = t.stats();
		taxis = std::max(taxis, st.by_kind[static_cast<size_t>(VehicleKind::Taxi)]);
		cars = std::max(cars, st.by_kind[static_cast<size_t>(VehicleKind::Car)]);
		bikes = std::max(bikes, st.by_kind[static_cast<size_t>(VehicleKind::Bike)]);
		for (const Vehicle &v : t.vehicles()) {
			if (v.kind == VehicleKind::Coach && v.off_lane) ++at_station;
		}
	}
	const TrafficStats st = t.stats();
	std::printf("showcase 1 h: %llu trips, %llu coach calls, %llu bus runs, %llu bikes, %llu parkings, taxis up to %u of %u cars\n",
			static_cast<unsigned long long>(st.arrived), static_cast<unsigned long long>(st.coach_calls),
			static_cast<unsigned long long>(st.bus_runs), static_cast<unsigned long long>(st.bikes_arrived),
			static_cast<unsigned long long>(st.parkings), taxis, cars);
	CHECK(st.coach_calls >= 2);
	CHECK(at_station > 0);
	CHECK(taxis >= 1);
	CHECK(taxis * 5 < cars);
	CHECK(bikes >= 1);
	CHECK(st.bikes_arrived >= 20);
	CHECK(st.bus_runs >= 4);
	CHECK(st.parkings >= 5);
	CHECK(st.removed_stuck == 0);
}

TEST_CASE("editing while buses run: stops move, vehicles carry on") {
	World w;
	build_showcase(w.doc);
	w.sync();
	w.t().reset(8);
	w.run_for(300.0);
	const size_t before = w.t().vehicles().size();
	REQUIRE(before > 10);
	// Move every stop a little and change the roundabout.
	for (const auto &kv : w.doc.map().segments()) {
		for (BusStop st : kv.second.stops) {
			st.u = std::min(0.8, st.u + 0.05);
			w.doc.set_stop(kv.first, st);
		}
	}
	w.sync();
	const size_t after = w.t().vehicles().size();
	CHECK(after + 5 >= before);
	w.run_for(900.0);
	const TrafficStats st = w.t().stats();
	CHECK(st.removed_stuck == 0);
	CHECK(st.bus_stops_served > 0);
	CHECK(overlaps(w.t()) == 0);
}

TEST_CASE("M3 determinism: the showcase replays bit for bit") {
	auto run = [](uint64_t seed) {
		World w;
		build_showcase(w.doc);
		w.sync();
		w.t().reset(seed);
		w.run_for(900.0);
		return w.t().state_hash();
	};
	const uint64_t a = run(42), b = run(42);
	CHECK(a == b);
	CHECK(run(43) != a);
	std::printf("M3 golden: %016llx (expected a240e0db57f4b121)\n", static_cast<unsigned long long>(a));
	CHECK(a == 0xa240e0db57f4b121ull); // same on every platform, like the M2 golden
}

TEST_CASE("M3 gate: the showcase runs a full sim day") {
	World w;
	build_showcase(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.reset(2026);
	double worst_stop = 0.0, worst_junction = 0.0;
	int bad_overlaps = 0;
	std::vector<uint64_t> hourly;
	uint64_t last = 0;
	for (int hour = 0; hour < 24; ++hour) {
		for (int i = 0; i < 36000; ++i) {
			t.tick();
			if (i % 100 == 0) {
				const TrafficStats st = t.stats();
				worst_stop = std::max(worst_stop, st.max_stopped);
				worst_junction = std::max(worst_junction, st.max_junction_wait);
			}
			if (i % 3000 == 0) bad_overlaps += overlaps(t);
		}
		hourly.push_back(t.stats().arrived - last);
		last = t.stats().arrived;
	}
	const TrafficStats st = t.stats();
	std::printf("M3 gate: %llu trips in 24 h, %u vehicles at the end, longest stop %.0f s, longest junction stuck %.0f s\n",
			static_cast<unsigned long long>(st.arrived), st.vehicles, worst_stop, worst_junction);
	std::printf("         bus runs %llu, stops served %llu, coach calls %llu, bikes %llu, parkings %llu (%llu no bay), right on red %llu\n",
			static_cast<unsigned long long>(st.bus_runs), static_cast<unsigned long long>(st.bus_stops_served),
			static_cast<unsigned long long>(st.coach_calls), static_cast<unsigned long long>(st.bikes_arrived),
			static_cast<unsigned long long>(st.parkings), static_cast<unsigned long long>(st.parking_failed),
			static_cast<unsigned long long>(st.right_on_red));
	std::printf("         trips per hour:");
	for (uint64_t h : hourly) std::printf(" %llu", static_cast<unsigned long long>(h));
	std::printf("\n");
	CHECK(st.removed_stuck == 0);
	CHECK(worst_stop < 600.0);
	CHECK(worst_junction < 90.0);
	CHECK(bad_overlaps == 0);
	const uint64_t lo = *std::min_element(hourly.begin() + 1, hourly.end());
	CHECK(lo * 2 > hourly[1]); // throughput holds up all day
	CHECK(st.bus_runs >= 100);
	CHECK(st.coach_calls >= 60);
	CHECK(st.bikes_arrived >= 1000);
	CHECK(st.parkings >= 100);
}

TEST_CASE("transit problems: unserved stops, broken routes, coach lines without a station") {
	World w;
	const Cross x = cross_roads(w.doc);
	// A one-way road out of the east arm: a stop against its direction has no lane.
	const SegmentId one_way = w.doc.add_road({ node_point(w.doc, x.e), free_point(300, 0) }, preset("One-way 2 lanes"), 0, 13.9).front();
	const NodeId far_end = w.doc.map().segment(one_way)->to;
	w.doc.set_spawner(far_end, spawner(0.0));
	const uint32_t bad = w.doc.add_stop(one_way, 0.5, LaneDir::Backward, StopKind::Kerbside, "Wrong side");
	const uint32_t ok = w.doc.add_stop(x.sw, 0.5, LaneDir::Forward, StopKind::Kerbside, "Fine");
	REQUIRE(bad != 0);
	REQUIRE(ok != 0);
	// A depot at the north end whose route needs to go west-bound on the west arm first... from the north it
	// can only reach the west arm heading west (out of the map), so "Fine" (eastbound) is unreachable.
	Depot d;
	d.enabled = true;
	d.name = "North";
	BusRoute r;
	r.name = "Nowhere";
	r.stops = { ok };
	d.routes = { r };
	w.doc.set_spawner(x.n, Spawner{});
	w.doc.set_depot(x.n, d);
	Spawner with_coach = spawner(100.0);
	CoachLine c;
	c.exit = x.s;
	with_coach.coaches = { c };
	w.doc.set_spawner(x.w, with_coach);
	w.sync();
	std::set<std::string> codes;
	for (const NetProblem &p : network_problems(w.doc.map(), w.net())) codes.insert(p.code);
	CHECK(codes.count("stop_no_lane") == 1);
	CHECK(codes.count("route_broken") == 1);
	CHECK(codes.count("no_main_station") == 1);
	CHECK(codes.count("depot_not_at_end") == 0);
	// A depot is a proper road end: no "road ends here" warning.
	w.geom.build(w.doc.map());
	for (const Problem &p : validate(w.doc.map(), w.geom)) CHECK(p.code != "road_end");
}

TEST_CASE("jammed exit: cars wait inside the junction on green and clear it after the light changes") {
	struct Result {
		uint64_t box_waits = 0;
		int caught_at_line = 0; // first at the line, exit full on green, then red before it got in
		int released_on_red = 0; // left the waiting spot after its light turned red
		int past_hold = 0; // a waiting car's front beyond its waiting spot
		int overlaps = 0;
		size_t east = 0, south = 0; // cars through the first junction
	};
	auto run = [](bool wait_in_box, Result &r) {
		World w;
		Document &doc = w.doc;
		const Cross x = cross_roads(doc);
		// A second signal 150 m east with a short east-west green: the eastbound
		// exit of the first junction backs up into it.
		const Profile p = preset("Street 1+1");
		const SegmentId far = doc.add_road({ node_point(doc, x.e), free_point(450, 0) }, p, 0, 13.9).front();
		const NodeId X = doc.map().segment(far)->to;
		const SegmentId n2 = doc.add_road({ free_point(150, -150), node_point(doc, x.e) }, p, 0, 13.9).front();
		const SegmentId s2 = doc.add_road({ node_point(doc, x.e), free_point(150, 150) }, p, 0, 13.9).front();
		doc.set_spawner(x.e, Spawner{});
		doc.set_spawner(doc.map().segment(n2)->from, spawner(0.0));
		doc.set_spawner(doc.map().segment(s2)->to, spawner(0.0));
		doc.set_spawner(X, spawner(0.0));
		Spawner west = spawner(1000.0, false);
		west.od = { { x.s, 0.0 }, { x.n, 0.0 }, { doc.map().segment(n2)->from, 0.0 }, { doc.map().segment(s2)->to, 0.0 } };
		doc.set_spawner(x.w, west);
		Spawner north = spawner(500.0, false);
		north.od = { { X, 0.0 }, { x.w, 0.0 }, { doc.map().segment(n2)->from, 0.0 }, { doc.map().segment(s2)->to, 0.0 } };
		doc.set_spawner(x.n, north);
		auto plan_of = [](SegmentId ew_in, SegmentId ew_out, double ew, SegmentId ns_in, SegmentId ns_out, double ns) {
			SignalPlan plan;
			SignalPhase a, b;
			a.green = ew;
			a.moves = { { ew_in, ew_out, false } };
			b.green = ns;
			b.moves = { { ns_in, ns_out, false } };
			plan.phases = { a, b };
			return plan;
		};
		doc.set_junction_control(x.c, JunctionControl::Signal, {});
		doc.set_signal_plan(x.c, plan_of(x.sw, x.se, 25.0, x.sn, x.ss, 25.0));
		doc.set_junction_control(x.e, JunctionControl::Signal, {});
		doc.set_signal_plan(x.e, plan_of(x.se, far, 6.0, n2, s2, 40.0));
		w.sync();
		Traffic &t = w.t();
		t.config().wait_in_box = wait_in_box;
		t.reset(5);
		const Network &n = w.net();
		const int32_t in_w = kerb_lane(n, x.sw, LaneDir::Forward);
		const int32_t out_e = kerb_lane(n, x.se, LaneDir::Forward);
		const int32_t out_s = kerb_lane(n, x.ss, LaneDir::Forward);
		REQUIRE(in_w >= 0);
		REQUIRE(out_e >= 0);
		REQUIRE(out_s >= 0);
		const NetJunction &junc = n.junctions[static_cast<size_t>(n.junction_at(x.c))];
		auto light = [&](int32_t conn) {
			return junc.light(n.lanes[static_cast<size_t>(conn)].movement, static_cast<int64_t>(t.tick_count()));
		};
		// Where a waiting car stops: short of its first conflict.
		auto hold_of = [&](int32_t conn) {
			const NetLane &c = n.lanes[static_cast<size_t>(conn)];
			double first = c.length;
			for (const Conflict &cf : c.conflicts) {
				if (n.lanes[static_cast<size_t>(cf.other)].from != c.from) first = std::min(first, cf.s_self);
			}
			return first - t.config().box_margin;
		};
		std::set<VehicleId> east, south, exit_full, staged;
		for (int i = 0; i < 12000; ++i) { // 20 minutes
			t.tick();
			std::set<VehicleId> now_staged;
			bool lane_waits_in_box = false; // a car from the jammed approach already waits inside
			for (const Vehicle &v : t.vehicles()) {
				const int32_t c = v.grant >= 0 ? v.grant : v.held;
				if (v.staged && c >= 0 && n.lanes[static_cast<size_t>(c)].from == in_w) lane_waits_in_box = true;
			}
			for (const Vehicle &v : t.vehicles()) {
				if (v.lane == out_e) east.insert(v.id);
				if (v.lane == out_s) south.insert(v.id);
				const int32_t c = v.grant >= 0 ? v.grant : v.held;
				if (v.staged && c >= 0) {
					now_staged.insert(v.id);
					if (v.lane == c && v.s > hold_of(c)) ++r.past_hold;
				} else if (staged.count(v.id) && c >= 0 && light(c) == SignalLight::Red) {
					++r.released_on_red;
				}
				// First at the line of the jammed approach: exit full on green, then caught by the red.
				if (v.lane != in_w || n.lanes[static_cast<size_t>(in_w)].length - v.s > 6.0 || v.grant >= 0) continue;
				if (v.state == VehicleState::ExitBlocked && !lane_waits_in_box) exit_full.insert(v.id);
				if (v.state == VehicleState::RedLight && exit_full.erase(v.id)) ++r.caught_at_line;
			}
			staged.swap(now_staged);
			if (i % 50 == 0) r.overlaps += overlaps(t);
		}
		r.box_waits = t.stats().box_waits;
		r.east = east.size();
		r.south = south.size();
		std::printf("jammed exit, %s: %llu waited in the box (%d went after their red), %d caught at the line by the red, "
					"%zu east and %zu south through the junction, %llu stuck\n",
				wait_in_box ? "waiting in the box" : "waiting at the line", (unsigned long long)r.box_waits,
				r.released_on_red, r.caught_at_line, r.east, r.south, (unsigned long long)t.stats().removed_stuck);
		CHECK(t.stats().removed_stuck == 0);
	};
	Result off, on;
	run(false, off);
	run(true, on);
	CHECK(off.box_waits == 0);
	CHECK(off.caught_at_line > 5); // the problem: green wasted at the line, then a red
	CHECK(on.box_waits > 5);
	CHECK(on.released_on_red > 0); // they clear the junction whatever the light shows
	CHECK(on.caught_at_line < off.caught_at_line / 3);
	CHECK(on.past_hold == 0);
	CHECK(on.overlaps == 0);
	CHECK(on.east + 2 >= off.east);
	CHECK(on.south * 10 >= off.south * 8); // the cross street still flows
}
