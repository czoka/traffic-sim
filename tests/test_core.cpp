#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include <doctest/doctest.h>

#include "tsim/hash.h"
#include "tsim/map.h"
#include "tsim/rng.h"
#include "tsim/sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace tsim;

TEST_CASE("rng is reproducible and in range") {
	Rng a(123), b(123), c(124);
	for (int i = 0; i < 1000; ++i) {
		const uint64_t x = a.next_u64();
		CHECK(x == b.next_u64());
	}
	CHECK(a.next_u64() != c.next_u64());
	Rng r(7);
	for (int i = 0; i < 10000; ++i) {
		const double u = r.uniform();
		CHECK(u >= 0.0);
		CHECK(u < 1.0);
	}
	// Pinned first value guards against accidental generator changes.
	Rng pin(42);
	CHECK(pin.next_u64() == 0x15780b2e0c2ec716ull);
}

TEST_CASE("ids are stable, ascending and never reused") {
	Map m;
	const NodeId a = m.add_node({ 0, 0 });
	const NodeId b = m.add_node({ 100, 0 });
	CHECK(a == 1);
	CHECK(b == 2);
	const SegmentId s = m.add_straight(a, b, 2, 3.5, 13.9);
	REQUIRE(s == 1);
	const Segment *seg = m.segment(s);
	REQUIRE(seg != nullptr);
	CHECK(seg->lanes.size() == 2);
	CHECK(m.lane(seg->lanes[0])->index == 0);
	CHECK(m.lane(seg->lanes[0])->offset == doctest::Approx(1.75));
	CHECK(m.lane(seg->lanes[1])->offset == doctest::Approx(-1.75));
	CHECK(m.lane(seg->lanes[0])->length == doctest::Approx(100.0));
	// Invalid segments do not consume IDs.
	CHECK(m.add_straight(a, a, 1, 3.5, 13.9) == kNoId);
	CHECK(m.add_straight(a, 99, 1, 3.5, 13.9) == kNoId);
	CHECK(m.next_segment_id() == 2);
}

TEST_CASE("straight lane pose: right-hand offset in a y-down frame") {
	Map m;
	const NodeId a = m.add_node({ 0, 0 });
	const NodeId b = m.add_node({ 100, 0 });
	const SegmentId s = m.add_straight(a, b, 1, 4.0, 10.0);
	// Single lane: centred.
	const Lane *l = m.lane(m.segment(s)->lanes[0]);
	Pose p = m.lane_pose(*l, 25.0);
	CHECK(p.pos.x == doctest::Approx(25.0));
	CHECK(p.pos.y == doctest::Approx(0.0));
	// Two lanes heading east: lane 0 is on the right, i.e. +y (down).
	const NodeId c = m.add_node({ 0, 50 });
	const NodeId d = m.add_node({ 100, 50 });
	const SegmentId s2 = m.add_straight(c, d, 2, 4.0, 10.0);
	const Lane *right = m.lane(m.segment(s2)->lanes[0]);
	CHECK(m.lane_pose(*right, 0.0).pos.y == doctest::Approx(52.0));
}

TEST_CASE("ring geometry") {
	Map m;
	RingInfo ring = build_ring(m, 100.0, 2, 3.5, 13.9);
	REQUIRE(ring.segments.size() == 4);
	CHECK(m.lanes().size() == 8);
	// Clockwise on screen: right-hand lane (index 0) is the inner lane.
	const Segment *s = m.segment(ring.segments[0]);
	const Lane *inner = m.lane(s->lanes[0]);
	const Lane *outer = m.lane(s->lanes[1]);
	CHECK(inner->length == doctest::Approx((100.0 - 1.75) * kHalfPi));
	CHECK(outer->length == doctest::Approx((100.0 + 1.75) * kHalfPi));
	// Every lane loops back to itself after four hops.
	for (const Lane &l : m.lanes()) {
		REQUIRE(l.next.size() == 1);
		LaneId cur = l.id;
		for (int i = 0; i < 4; ++i) cur = m.lane(cur)->next[0];
		CHECK(cur == l.id);
	}
	// Pose at the start of the first arc is on the east side, heading south.
	Pose p = m.lane_pose(*inner, 0.0);
	CHECK(p.pos.x == doctest::Approx(98.25));
	CHECK(p.pos.y == doctest::Approx(0.0));
	CHECK(p.dir.y == doctest::Approx(1.0));
}

TEST_CASE("spawn spreads cars over looping lanes only") {
	Map m;
	build_ring(m, 200.0, 2, 3.5, 13.9);
	const NodeId a = m.add_node({ 500, 0 });
	const NodeId b = m.add_node({ 600, 0 });
	const SegmentId dead_end = m.add_straight(a, b, 1, 3.5, 13.9);
	Simulation sim(m);
	CHECK(sim.spawn_even(100, 1) == 100);
	const LaneId dead_lane = m.segment(dead_end)->lanes[0];
	for (size_t i = 0; i < sim.vehicle_count(); ++i) {
		CHECK(sim.vehicle_lane(i) != dead_lane);
	}
	// Capacity limit: ~2 * 2*pi*200 m of lane at >= 7 m spacing.
	const uint32_t placed = sim.spawn_even(100000, 1);
	CHECK(placed < 400);
	CHECK(placed > 300);
}

TEST_CASE("free-flowing car reaches its desired speed") {
	Map m;
	build_ring(m, 500.0, 1, 3.5, 20.0);
	Simulation sim(m, SimConfig{ 0.1, 0.0, 300.0, 9.0 });
	REQUIRE(sim.spawn_even(1, 3) == 1);
	for (int i = 0; i < 600; ++i) sim.tick();
	CHECK(sim.vehicle_speed(0) == doctest::Approx(sim.vehicle_params(0).v0).epsilon(0.05));
}

TEST_CASE("cars never overlap on a dense ring") {
	Map m;
	build_ring(m, 150.0, 2, 3.5, 16.7);
	Simulation sim(m);
	sim.spawn_even(110, 9); // ~8.5 m per car: near jam density
	for (int t = 0; t < 3000; ++t) {
		sim.tick();
	}
	// Check bumper gaps within each lane by sorting on arc position.
	double min_gap = 1e9;
	for (const Lane &l : m.lanes()) {
		std::vector<std::pair<double, double>> cars; // s, length
		for (size_t i = 0; i < sim.vehicle_count(); ++i) {
			if (sim.vehicle_lane(i) == l.id) cars.push_back({ sim.vehicle_s(i), sim.vehicle_params(i).length });
		}
		std::sort(cars.begin(), cars.end());
		for (size_t k = 1; k < cars.size(); ++k) {
			min_gap = std::min(min_gap, cars[k].first - cars[k].second - cars[k - 1].first);
		}
	}
	CHECK(min_gap > 0.0);
}

TEST_CASE("dead-end lane: cars stop at the end instead of leaving the map") {
	Map m;
	const NodeId a = m.add_node({ 0, 0 });
	const NodeId b = m.add_node({ 100, 0 });
	const NodeId c = m.add_node({ 200, 0 });
	const SegmentId s1 = m.add_straight(a, b, 1, 3.5, 15.0);
	const SegmentId s2 = m.add_straight(b, c, 1, 3.5, 15.0);
	m.connect(m.segment(s1)->lanes[0], m.segment(s2)->lanes[0]);
	Simulation sim(m, SimConfig{ 0.1, 0.0, 300.0, 9.0 });
	REQUIRE(sim.spawn_even(3, 5) == 3); // only s1 has a successor
	for (int t = 0; t < 2000; ++t) sim.tick();
	for (size_t i = 0; i < sim.vehicle_count(); ++i) {
		CHECK(sim.vehicle_lane(i) == m.segment(s2)->lanes[0]);
		CHECK(sim.vehicle_s(i) <= 100.0);
		CHECK(sim.vehicle_speed(i) < 0.5);
	}
}

TEST_CASE("same map and seed give the same hash; different seed differs") {
	Map m;
	build_ring(m, 300.0, 2, 3.5, 16.7);
	const uint64_t h1 = run_scenario(m, 200, 11, 500);
	const uint64_t h2 = run_scenario(m, 200, 11, 500);
	const uint64_t h3 = run_scenario(m, 200, 12, 500);
	CHECK(h1 == h2);
	CHECK(h1 != h3);
}

TEST_CASE("adding a road mid-run keeps vehicles and determinism") {
	auto run = [](bool edit) {
		Map m;
		build_ring(m, 300.0, 2, 3.5, 16.7);
		Simulation sim(m);
		sim.spawn_even(150, 4);
		for (int t = 0; t < 200; ++t) sim.tick();
		if (edit) {
			const NodeId a = m.add_node({ 1000, 0 });
			const NodeId b = m.add_node({ 1100, 0 });
			m.add_straight(a, b, 2, 3.5, 13.9);
			sim.on_map_changed();
		}
		for (int t = 0; t < 200; ++t) sim.tick();
		return sim.state_hash();
	};
	// An unconnected road does not affect traffic on the ring.
	CHECK(run(false) == run(true));
}

TEST_CASE("golden scenario hash (cross-platform determinism)") {
	Map m;
	build_golden_map(m);
	const std::string hash = hash_to_hex(run_scenario(m, GoldenScenario::kCars, GoldenScenario::kSeed,
			GoldenScenario::kTicks));
	std::printf("golden hash: %s (expected %s)\n", hash.c_str(), GoldenScenario::kExpectedHash);
	CHECK(hash == std::string(GoldenScenario::kExpectedHash));
}
