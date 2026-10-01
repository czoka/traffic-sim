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
