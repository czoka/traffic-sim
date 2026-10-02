// M7 tests: insight (lane heat, junction stats), the new warnings, the profile
// library, the tutorial map built the way a new player would, and save files
// from every milestone loading into the current format.
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include <doctest/doctest.h>

#include "tsim/demo_maps.h"
#include "tsim/document.h"
#include "tsim/network.h"
#include "tsim/road_geometry.h"
#include "tsim/road_map_json.h"
#include "tsim/traffic_run.h"
#include "tsim/validation.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
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
	void run_minutes(double minutes) {
		const uint64_t n = static_cast<uint64_t>(minutes * 60.0 / t().config().dt + 0.5);
		for (uint64_t i = 0; i < n; ++i) t().tick();
	}
	std::vector<Problem> problems() {
		geom.build(doc.map());
		return validate(doc.map(), geom);
	}
	int errors() {
		int n = 0;
		for (const Problem &p : problems()) {
			if (p.severity == Severity::Error) {
				std::printf("  problem: %s\n", p.message.c_str());
				++n;
			}
		}
		return n;
	}
	bool has(const char *code) {
		for (const Problem &p : problems()) {
			if (p.code == code) return true;
		}
		return false;
	}
};

std::string data_dir() {
	std::string f = __FILE__;
	const size_t slash = f.find_last_of("/\\");
	return (slash == std::string::npos ? std::string(".") : f.substr(0, slash)) + "/data/migrations/";
}

bool read_file(const std::string &path, std::string &out) {
	std::ifstream in(path, std::ios::binary);
	if (!in) return false;
	std::ostringstream ss;
	ss << in.rdbuf();
	out = ss.str();
	return true;
}

SegmentId segment_named(const RoadMap &m, const char *name, double y = -1.0) {
	for (const auto &kv : m.segments()) {
		if (kv.second.name != name) continue;
		if (y >= 0.0) {
			const Vec2 a = m.node(kv.second.from)->pos, b = m.node(kv.second.to)->pos;
			if (std::fabs(a.y - y) > 1.0 || std::fabs(b.y - y) > 1.0) continue;
		}
		return kv.first;
	}
	return kNoId;
}

} // namespace

TEST_CASE("lane heat: speed, wait and throughput; junction stats; carried over a recompile") {
	World w;
	build_test_grid(w.doc, 7, 8, 120.0);
	w.sync();
	Traffic &t = w.t();
	t.reset(7);
	w.run_minutes(10.0);
	const Network &n = w.net();
	const std::vector<LaneHeat> &heat = t.lane_heat();
	REQUIRE(heat.size() == n.lanes.size());
	int seen = 0, slow = 0;
	double flow = 0.0;
	uint64_t passed = 0;
	for (size_t l = 0; l < heat.size(); ++l) {
		const LaneHeat &h = heat[l];
		if (n.lanes[l].kind != NetLaneKind::Road) {
			CHECK(!h.seen);
			continue;
		}
		if (!h.seen) continue;
		++seen;
		CHECK(h.speed >= 0.0);
		CHECK(h.speed_ratio >= 0.0);
		CHECK(h.speed_ratio <= 1.0);
		CHECK(h.wait >= 0.0);
		slow += h.speed_ratio < 0.5 ? 1 : 0;
		flow += h.flow;
		passed += h.passed;
	}
	std::printf("heat after 10 min: %d lanes with traffic, %d below half the limit, %.0f vehicles an hour leaving lanes, %llu "
				"lane exits\n",
			seen, slow, flow, (unsigned long long)passed);
	CHECK(seen > 100);
	CHECK(slow > 0); // queues at the junctions
	CHECK(passed > 1000);
	// A busy junction: flow in, waits, the worst approach.
	JunctionStats best;
	for (const NetJunction &j : n.junctions) {
		const JunctionStats js = t.junction_stats(j.node);
		REQUIRE(js.found);
		if (js.flow > best.flow) best = js;
	}
	std::printf("busiest junction %u: %.0f an hour, mean wait %.1f s, %u queued, %llu through, worst approach %u (%.1f s)\n",
			best.node, best.flow, best.mean_wait, best.queued, (unsigned long long)best.passed, best.worst_segment,
			best.worst_wait);
	CHECK(best.flow > 200.0);
	CHECK(best.approaches >= 3);
	CHECK(best.passed > 20);
	CHECK(!t.junction_stats(999999).found);
	// Editing one road keeps what every other lane has seen.
	const uint64_t before = heat[static_cast<size_t>(n.junctions[0].approaches[0])].passed;
	const LaneKey key = n.lanes[static_cast<size_t>(n.junctions[0].approaches[0])].key;
	const SegmentId far = w.doc.map().segments().rbegin()->first;
	w.doc.set_speed_limit(far, 40.0 / 3.6);
	w.sync();
	int32_t again = -1;
	for (size_t l = 0; l < w.net().lanes.size(); ++l) {
		if (w.net().lanes[l].key == key) again = static_cast<int32_t>(l);
	}
	REQUIRE(again >= 0);
	CHECK(t.lane_heat()[static_cast<size_t>(again)].passed == before);
	// A reset starts the counts again.
	t.reset(7);
	for (const LaneHeat &h : t.lane_heat()) CHECK(h.passed == 0);
}

TEST_CASE("warnings: a small roundabout for its legs; parking beside a crossing") {
	World w;
	build_test_grid(w.doc, 3, 3, 120.0);
	NodeId four = kNoId;
	for (const auto &kv : w.doc.map().nodes()) {
		int legs = 0;
		for (const auto &s : w.doc.map().segments()) legs += (s.second.from == kv.first) + (s.second.to == kv.first);
		if (legs == 4) four = kv.first;
	}
	REQUIRE(four != kNoId);
	Roundabout r;
	r.enabled = true;
	r.radius = 12.0;
	w.doc.set_roundabout(four, r);
	CHECK(w.has("roundabout_small"));
	r.radius = 20.0;
	w.doc.set_roundabout(four, r);
	CHECK(!w.has("roundabout_small"));
	// Parking on a street with a mid-block zebra.
	World p;
	PointRef a, b;
	a.pos = Vec2{ -150, 0 };
	b.pos = Vec2{ 150, 0 };
	RoadMap scratch;
	const SegmentId s = p.doc.add_road({ a, b }, preset_profile("Street 1+1, parking", scratch), 0, 13.9).front();
	CHECK(!p.has("parking_crossing"));
	p.doc.add_crossing(s, 0.5, CrossingKind::Zebra, false, false);
	CHECK(p.has("parking_crossing"));
}

TEST_CASE("profile library: every built-in profile is valid and builds a road without errors") {
	const std::vector<ProfilePreset> &all = profile_presets();
	CHECK(all.size() >= 15);
	for (const ProfilePreset &pp : all) {
		World w;
		RoadMap scratch;
		const Profile prof = preset_profile(pp.name, scratch);
		CHECK_MESSAGE(validate_profile(prof).empty(), pp.name);
		PointRef a, b;
		a.pos = Vec2{ -100, 0 };
		b.pos = Vec2{ 100, 0 };
		REQUIRE(!w.doc.add_road({ a, b }, prof, 0, 13.9).empty());
		CHECK_MESSAGE(w.errors() == 0, pp.name);
		// Its params give back the same road.
		const ProfileParams back = params_of(prof);
		CHECK(back.forward == pp.params.forward);
		CHECK(back.backward == pp.params.backward);
	}
}

TEST_CASE("M7 gate: the tutorial map, built the way a new player would, makes a working town") {
	World w;
	build_tutorial(w.doc);
	CHECK(w.errors() == 0);
	const RoadMap &m = w.doc.map();
	const SegmentId high = segment_named(m, "High Street", 0.0); // the first piece: west of Loop Road
	SegmentId middle = kNoId; // between the two ends of Loop Road
	for (const auto &kv : m.segments()) {
		if (kv.second.name == "High Street" && std::fabs(m.node(kv.second.from)->pos.x + 100.0) < 1.0) middle = kv.first;
	}
	const SegmentId bottom = segment_named(m, "Loop Road", 220.0);
	const SegmentId lane = segment_named(m, "Depot Lane");
	REQUIRE(high != kNoId);
	REQUIRE(middle != kNoId);
	REQUIRE(bottom != kNoId);
	REQUIRE(lane != kNoId);
	// Homes and a shop along Loop Road.
	w.geom.build(m);
	int homes = 0;
	for (double y : { 60.0, 100.0, 160.0 }) homes += place_building(w.doc, w.geom, "townhouse", Vec2{ -85.0, y }) != 0;
	w.geom.build(m);
	const uint32_t shop = place_building(w.doc, w.geom, "grocery", Vec2{ -60.0, 205.0 });
	CHECK(homes == 3);
	REQUIRE(shop != 0);
	// Two stops, the depot at Depot Lane's end and a loop route.
	const uint32_t a = w.doc.add_stop(bottom, 0.5, LaneDir::Forward, StopKind::Kerbside, "Loop Road");
	const uint32_t b = w.doc.add_stop(middle, 0.3, LaneDir::Backward, StopKind::Kerbside, "High Street");
	Depot d;
	d.enabled = true;
	d.name = "Depot";
	d.capacity = 4;
	BusRoute route;
	route.name = "1";
	route.stops = { a, b };
	route.headway = 300.0;
	route.loop = true;
	d.routes = { route };
	w.doc.set_depot(m.segment(lane)->to, d);
	CHECK(w.errors() == 0);
	w.sync();
	for (const NetProblem &p : network_problems(m, w.net())) {
		std::printf("  network: %s\n", p.message.c_str());
		CHECK(p.code != "route_broken");
	}
	Traffic &t = w.t();
	t.reset(2026);
	w.run_minutes(24 * 60.0);
	const CityStats c = t.city_stats();
	const TrafficStats ts = t.stats();
	const BuildingInfo g = t.building_info(shop);
	std::printf("tutorial town after a day: %u residents in %u households, %llu shifts, grocery served %llu, %llu bus stops "
				"served, %llu bus runs, %llu stuck\n",
			c.residents, c.households, (unsigned long long)c.shifts, (unsigned long long)g.served,
			(unsigned long long)ts.bus_stops_served, (unsigned long long)ts.bus_runs, (unsigned long long)ts.removed_stuck);
	CHECK(c.residents >= 3);
	CHECK(c.households >= 3);
	CHECK(g.served > 0);
	CHECK(ts.bus_stops_served > 10);
	CHECK(ts.removed_stuck == 0);
	CHECK(c.starving == 0);
}

TEST_CASE("migrations: a save file from every milestone loads, upgrades to v7 and runs") {
	struct File {
		const char *name;
		int version;
		bool city;
	};
	const File files[] = {
		{ "poc_ring_v1.json", 1, false },
		{ "demo_town_v2.json", 2, false },
		{ "demo_town_v3.json", 3, false },
		{ "showcase_v4.json", 4, false },
		{ "people_town_v5.json", 5, false },
		{ "city_town_v6.json", 6, true },
		{ "city_market_v7.json", 7, true },
	};
	for (const File &f : files) {
		INFO(f.name);
		std::string text;
		REQUIRE_MESSAGE(read_file(data_dir() + f.name, text), "missing ", data_dir() + f.name);
		World w;
		RoadMap map;
		std::string err;
		int from = 0;
		REQUIRE_MESSAGE(road_map_from_json(text, map, &err, &from), err);
		CHECK(from == f.version);
		// Saved again, it is a v7 file that reads back the same.
		const std::string v7 = road_map_to_json(map);
		CHECK(v7.find("\"version\": 7") != std::string::npos);
		RoadMap back;
		REQUIRE_MESSAGE(road_map_from_json(v7, back, &err), err);
		CHECK(back == map);
		CHECK(road_map_to_json(back) == v7);
		w.doc.reset(map);
		const int errors = w.errors();
		CHECK(errors == 0);
		w.sync();
		Traffic &t = w.t();
		t.config().city_prefill = f.city ? 1.0 : 0.0;
		t.reset(1);
		w.run_minutes(5.0);
		const TrafficStats s = t.stats();
		std::printf("%-22s v%d -> v7: %zu nodes, %zu segments, %zu buildings; 5 min: %llu vehicles spawned, %u people "
					"trips, %u residents, %llu stuck\n",
				f.name, f.version, map.nodes().size(), map.segments().size(), map.buildings().size(),
				(unsigned long long)s.spawned, static_cast<unsigned>(s.trips), t.city_stats().residents,
				(unsigned long long)s.removed_stuck);
		// Spawn points came with v3 (M2): older maps have no traffic of their own.
		if (f.version >= 3) CHECK(s.spawned + s.trips + t.city_stats().residents > 0);
		CHECK(s.removed_stuck == 0);
	}
}
