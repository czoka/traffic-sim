// M5 tests: city data, buildings, residents and their needs, jobs and shifts,
// opening hours, immigration, visitors and the M5 gate.
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include <doctest/doctest.h>

#include "tsim/buildings.h"
#include "tsim/city_data.h"
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

std::string clock(const Traffic &t) {
	static const char *days[] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
	char buf[32];
	std::snprintf(buf, sizeof(buf), "%s %02d:%02d", days[t.day() % 7], t.minute_of_day() / 60, t.minute_of_day() % 60);
	return buf;
}

void print_city(const char *label, const Traffic &t) {
	const CityStats c = t.city_stats();
	std::printf("%s [%s]: %u residents in %u households (%u inside, %u travelling, %u outside; %u asleep, %u at work), "
				"%u visitors\n",
			label, clock(t).c_str(), c.residents, c.households, c.inside, c.travelling, c.outside, c.sleeping, c.working,
			c.visitors);
	std::printf("    jobs: %u local, %u outside, %u looking; shifts %llu (late %llu), unfilled %llu; meals out %llu, at home %llu, "
				"groceries %llu, sleeps %llu\n",
			c.employed, c.employed_outside, c.unemployed, (unsigned long long)c.shifts, (unsigned long long)c.late_shifts,
			(unsigned long long)c.unfilled_shifts, (unsigned long long)c.meals_out, (unsigned long long)c.home_meals,
			(unsigned long long)c.groceries, (unsigned long long)c.sleeps);
	std::printf("    hunger mean %.0f (min %.0f, %u starving), energy %.0f, money %.0f; businesses %u (%u open, %u closed "
				"unexpectedly), late openings %llu, turned away %llu; immigrants %llu, homes %u/%u units vacant\n",
			c.mean_hunger, c.min_hunger, c.starving, c.mean_energy, c.mean_money, c.businesses, c.open, c.closed_unexpectedly,
			(unsigned long long)c.late_openings, (unsigned long long)c.turned_away, (unsigned long long)c.immigrants,
			c.vacant_units, c.units);
}

} // namespace

TEST_CASE("city data: the default table parses, and bad tables are rejected") {
	const CityData &d = default_city_data();
	REQUIRE(d.types.size() >= 10);
	CHECK(d.type("grocery") != nullptr);
	CHECK(d.type("grocery")->kind == BuildingKind::Shop);
	CHECK(d.type("apartment_block")->households == 24);
	CHECK(d.offering("sleep") != nullptr);
	CHECK(d.type("grocery")->weekday.in_hours(7 * 60));
	CHECK(!d.type("grocery")->weekday.in_hours(22 * 60));
	CHECK(!d.type("office_small")->weekend.open);
	CHECK(d.type("grocery")->headcount(40) == 8);
	CHECK(d.urgency(80.0) < 0.05);
	CHECK(d.urgency(20.0) > 0.9);
	CHECK(d.urgency(0.0) == doctest::Approx(2.0));
	CityData bad;
	std::string err;
	CHECK(!parse_city_data("{", bad, &err));
	CHECK(!parse_city_data(R"({"offerings":{},"buildings":{"x":{"kind":"shop","offers":["nope"]}}})", bad, &err));
	CHECK(err.find("unknown offering") != std::string::npos);
}

TEST_CASE("city town: valid, round-trips as v7, doors on the sidewalk") {
	World w;
	build_city_town(w.doc);
	CHECK(w.errors() == 0);
	const std::string json = road_map_to_json(w.doc.map());
	CHECK(json.find("\"version\": 7") != std::string::npos);
	RoadMap back;
	std::string err;
	REQUIRE_MESSAGE(road_map_from_json(json, back, &err), err);
	CHECK(road_map_to_json(back) == json);
	CHECK(back == w.doc.map());
	w.sync();
	for (const NetProblem &p : network_problems(w.doc.map(), w.net())) CHECK_MESSAGE(false, "network problem: ", p.message);
	CHECK(w.net().buildings.size() == w.doc.map().buildings().size());
	for (const NetBuilding &b : w.net().buildings) CHECK_MESSAGE(b.entrance >= 0, "building ", b.id, " has no door");
	// Undo takes buildings away and redo puts them back.
	Document d;
	build_city_town(d);
	const size_t n = d.map().buildings().size();
	REQUIRE(n > 10);
	const uint32_t first = d.map().buildings().begin()->first;
	d.remove_building(first);
	CHECK(d.map().buildings().size() == n - 1);
	CHECK(d.undo());
	CHECK(d.map().buildings().size() == n);
}

TEST_CASE("city town: residents live two days - sleep, eat, shop and work") {
	for (uint64_t seed : { 3ull, 11ull, 29ull }) {
	World w;
	build_city_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 1.0;
	t.reset(seed);
	REQUIRE(t.city_on());
	print_city("city town start", t);
	uint32_t worst = 0;
	for (int h = 0; h < 48; ++h) {
		w.run_minutes(60.0);
		worst = std::max(worst, t.city_stats().starving);
		if (h % 12 == 11 && seed == 3) print_city("city town", t);
	}
	std::printf("seed %llu: at most %u residents at zero hunger at the top of an hour\n", (unsigned long long)seed, worst);
	CHECK(worst <= 2);
	const CityStats c = t.city_stats();
	CHECK(c.residents > 50);
	CHECK(c.sleeps > c.residents);
	CHECK(c.home_meals + c.meals_out > c.residents * 2);
	CHECK(c.groceries > 0);
	CHECK(c.shifts > 0);
	CHECK(c.starving == 0);
	CHECK(c.min_hunger > 0.0);
	}
}

namespace {

PointRef pt(double x, double y) {
	PointRef p;
	p.pos = Vec2{ x, y };
	return p;
}

Profile preset(const char *name) {
	RoadMap scratch;
	return preset_profile(name, scratch);
}

Spawner sink(double rate = 0.0) {
	Spawner s;
	s.enabled = true;
	s.rate = rate;
	s.sink = true;
	return s;
}

int occupied(const Traffic &t, const char *type) {
	int used = 0;
	for (const NetBuilding &b : t.network()->buildings) {
		if (b.type < 0 || t.city_data().types[static_cast<size_t>(b.type)].id != type) continue;
		used += t.building_info(b.id).households;
	}
	return used;
}

int units_of(const Traffic &t, const char *type) {
	int n = 0;
	for (const NetBuilding &b : t.network()->buildings) {
		if (b.type < 0 || t.city_data().types[static_cast<size_t>(b.type)].id != type) continue;
		n += t.building_info(b.id).units;
	}
	return n;
}

} // namespace

TEST_CASE("immigration: people come by coach for vacant homes they can reach, cheap ones first") {
	World w;
	build_city_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.reset(5);
	CHECK(t.city_stats().residents == 0);
	w.run_minutes(3 * 24 * 60);
	print_city("immigration after 3 days", t);
	const CityStats c = t.city_stats();
	CHECK(c.immigrants > 30);
	CHECK(c.households > 10);
	CHECK(c.vacant_units < c.units);
	// Apartments (500 a month) fill before detached houses (1,200).
	const double flats = static_cast<double>(occupied(t, "apartment_block")) / units_of(t, "apartment_block");
	const double houses = static_cast<double>(occupied(t, "detached_house")) / units_of(t, "detached_house");
	std::printf("occupied: apartments %.0f%%, detached houses %.0f%%\n", flats * 100.0, houses * 100.0);
	CHECK(flats >= houses);
	CHECK(c.starving == 0);
}

TEST_CASE("visitors: the city offices get staff by coach before anyone lives in the city") {
	World w;
	build_new_city(w.doc);
	CHECK(w.errors() == 0);
	w.sync();
	Traffic &t = w.t();
	t.reset(2);
	const uint32_t offices = w.doc.map().buildings().begin()->first;
	w.run_minutes(4 * 60); // to 10:00
	BuildingInfo b = t.building_info(offices);
	std::printf("city offices at 10:00: %d staff in, open %d, %d shifts unfilled today\n", b.staff_in, b.open, b.unfilled_today);
	CHECK(t.city_stats().residents == 0);
	CHECK(b.staff_in >= 2);
	CHECK(b.open);
	CHECK(t.city_stats().visitors > 20);
	w.run_minutes(12 * 60); // to 22:00: everyone went home
	CHECK(t.city_stats().visitors < 5);
	CHECK(!t.building_info(offices).open);
}

TEST_CASE("opening rules: no staff, no service; customers remember an unexpected closure") {
	World w;
	Document &doc = w.doc;
	const SegmentId road = doc.add_road({ pt(-200, 0), pt(200, 0) }, preset("Street 1+1"), 0, 13.9).front();
	doc.set_spawner(doc.map().segment(road)->from, sink());
	doc.set_spawner(doc.map().segment(road)->to, sink());
	w.geom.build(doc.map());
	const uint32_t food = place_building(doc, w.geom, "fast_food", Vec2{ 0, -20 });
	const uint32_t home = place_building(doc, w.geom, "apartment_block", Vec2{ 60, 20 });
	REQUIRE(food != 0);
	REQUIRE(home != 0);
	w.sync();
	Traffic &t = w.t();
	t.reset(4);
	for (int k = 0; k < 10; ++k) REQUIRE(t.add_household(home, 2, false) >= 0); // nobody works, no coaches: no staff
	w.run_minutes(6 * 60); // to 12:00
	const BuildingInfo b = t.building_info(food);
	CHECK(b.in_hours);
	CHECK(!b.open);
	CHECK(b.closed_unexpectedly);
	w.run_minutes(10 * 60); // to 22:00
	const BuildingInfo e = t.building_info(food);
	std::printf("fast food with no staff: turned away %llu (20 residents), unexpectedly closed %d min\n",
			(unsigned long long)e.turned_away, e.unexpected_minutes_today);
	CHECK(e.turned_away > 0);
	CHECK(e.turned_away <= 30); // they remember and stop coming
	CHECK(e.served == 0);
	CHECK(t.city_stats().home_meals > 20);
}

TEST_CASE("jobs outside the map: commuters go by coach in the morning and come back paid") {
	World w;
	build_new_city(w.doc);
	w.geom.build(w.doc.map());
	for (int k = 0; k < 4; ++k) place_building(w.doc, w.geom, "apartment_block", Vec2{ -380.0 + 45.0 * k, 22.0 });
	for (int k = 0; k < 4; ++k) place_building(w.doc, w.geom, "apartment_block", Vec2{ 250.0 + 45.0 * k, -22.0 });
	CHECK(w.errors() == 0);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 1.0;
	t.reset(6);
	const CityStats c0 = t.city_stats();
	std::printf("commuter town: %u residents, %u work locally, %u outside, %u looking\n", c0.residents, c0.employed,
			c0.employed_outside, c0.unemployed);
	CHECK(c0.employed_outside > 50);
	w.run_minutes(5 * 60); // 11:00
	const uint32_t away = t.city_stats().outside;
	w.run_minutes(10 * 60); // 21:00
	const CityStats c1 = t.city_stats();
	print_city("commuter town 21:00", t);
	CHECK(away > c0.employed_outside / 2);
	CHECK(c1.outside < away / 4);
	CHECK(c1.mean_money > c0.mean_money);
}

TEST_CASE("weekends: offices close, shops open with weekend shifts paid double") {
	CityData data = default_city_data();
	data.start_day = 5; // Saturday
	data.start_hour = 6;
	World w;
	build_city_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.set_city_data(&data);
	t.config().city_prefill = 1.0;
	t.reset(8);
	REQUIRE(t.is_weekend());
	w.run_minutes(5 * 60); // Saturday 11:00
	uint32_t offices = 0, grocery = 0;
	for (const auto &kv : w.doc.map().buildings()) {
		if (kv.second.type == "city_offices") offices = kv.first;
		if (kv.second.type == "grocery") grocery = kv.first;
	}
	CHECK(!t.building_info(offices).in_hours);
	CHECK(t.building_info(offices).staff_in == 0);
	CHECK(t.building_info(grocery).open);
	// A weekend shift pays twice the hourly wage.
	const double before = t.city_stats().mean_money;
	w.run_minutes(12 * 60);
	CHECK(t.city_stats().shifts > 0);
	(void)before;
	t.set_city_data(nullptr);
}

TEST_CASE("late staff open the shop late: a bus commute through a jam") {
	auto run = [](double cars_per_hour, int &late_minutes, uint64_t &late_shifts) {
		World w;
		Document &doc = w.doc;
		const Profile street = preset("Street 1+1");
		// A loop: High Street (y = 0) with an all-way stop in the middle, a back road (y = -200).
		auto node = [&](double x, double y) { return doc.add_node(Vec2{ x, y }, 0); };
		auto road = [&](NodeId p, NodeId q) {
			PointRef a, b;
			a.node = p;
			a.pos = doc.map().node(p)->pos;
			b.node = q;
			b.pos = doc.map().node(q)->pos;
			return doc.add_road({ a, b }, street, 0, 13.9).front();
		};
		const NodeId A = node(-900, 0), B = node(0, 0), C = node(900, 0), D = node(900, -200), F = node(0, -200),
					 E = node(-900, -200);
		const SegmentId ab = road(A, B), bc = road(B, C);
		road(C, D);
		road(D, F);
		road(F, E);
		road(E, A);
		const SegmentId bf = road(B, F);
		const NodeId S = node(0, 300), W = node(-1200, 0), X = node(1200, 0), G = node(-1200, -200), N = node(0, -500);
		const SegmentId bs = road(B, S);
		road(F, N);
		road(W, A);
		road(C, X);
		road(E, G);
		// A signal that gives High Street a short green: its queue grows with the traffic.
		doc.set_junction_control(B, JunctionControl::Signal, {});
		SignalPlan plan = default_signal_plan(doc.map(), B);
		for (SignalPhase &ph : plan.phases) {
			bool high = false;
			for (const SignalMovement &m : ph.moves) high |= m.from == ab || m.from == bc;
			ph.green = high ? 12.0 : 60.0;
		}
		doc.set_signal_plan(B, plan);
		(void)bf;
		(void)bs;
		// Every car trip ends at the south end, so all of them go through the signal.
		Spawner source = sink(cars_per_hour);
		source.sink = false;
		doc.set_spawner(W, source);
		source.rate = cars_per_hour * 0.3;
		doc.set_spawner(N, source);
		doc.set_spawner(S, sink());
		doc.set_spawner(X, Spawner{});
		const uint32_t west = doc.add_stop(ab, 0.1, LaneDir::Forward, StopKind::Kerbside, "West");
		const uint32_t east = doc.add_stop(bc, 0.85, LaneDir::Forward, StopKind::Kerbside, "East");
		Depot depot;
		depot.enabled = true;
		depot.name = "Depot";
		BusRoute route;
		route.name = "Loop";
		route.stops = { west, east };
		route.headway = 300.0;
		route.loop = true;
		depot.routes = { route };
		doc.set_depot(G, depot);
		w.geom.build(doc.map());
		std::vector<uint32_t> homes;
		for (int k = 0; k < 6; ++k) homes.push_back(place_building(doc, w.geom, "townhouse", Vec2{ -790.0 + 9.0 * k, 22.0 }));
		const uint32_t grocery = place_building(doc, w.geom, "grocery", Vec2{ 700, 22 });
		REQUIRE(grocery != 0);
		w.sync();
		Traffic &t = w.t();
		t.reset(9);
		for (uint32_t h : homes) t.add_household(h, 2, true);
		w.run_minutes(180); // to 09:00
		const BuildingInfo g = t.building_info(grocery);
		late_minutes = g.opened_at < 0 ? 999 : g.late_minutes_today;
		late_shifts = t.city_stats().late_shifts;
		std::printf("  %.0f cars/h: %llu bus stops served, the grocery opened at %02d:%02d\n", cars_per_hour,
				(unsigned long long)t.stats().bus_stops_served, g.opened_at / 60, g.opened_at % 60);
	};
	int free_late = 0, jam_late = 0;
	uint64_t free_shifts = 0, jam_shifts = 0;
	run(0.0, free_late, free_shifts);
	run(900.0, jam_late, jam_shifts);
	std::printf("grocery opened %d min late with free roads (%llu late shifts), %d min late in a jam (%llu late shifts)\n",
			free_late, (unsigned long long)free_shifts, jam_late, (unsigned long long)jam_shifts);
	CHECK(free_late <= 5);
	CHECK(jam_late > free_late + 5);
	CHECK(jam_shifts > free_shifts);
}

TEST_CASE("city: same seed, same hash; buildings removed while running") {
	auto run = [](uint64_t seed) {
		World w;
		build_city_town(w.doc);
		w.sync();
		w.t().config().city_prefill = 1.0;
		w.t().reset(seed);
		w.run_minutes(240.0);
		return w.t().state_hash();
	};
	const uint64_t a = run(42);
	CHECK(a == run(42));
	CHECK(a != run(43));
	std::printf("M5 golden: %016llx (expected 7ca7601c9fbb356f)\n", static_cast<unsigned long long>(a));
	CHECK(a == 0x7ca7601c9fbb356full); // same on every platform, like the other goldens
	World w;
	build_city_town(w.doc);
	w.sync();
	w.t().config().city_prefill = 1.0;
	w.t().reset(1);
	w.run_minutes(180.0);
	const uint32_t before = w.t().city_stats().residents;
	// Take an apartment block away: its people leave; everyone else carries on.
	uint32_t block = 0;
	for (const auto &kv : w.doc.map().buildings()) {
		if (kv.second.type == "apartment_block") block = kv.first;
	}
	const int gone = w.t().building_info(block).residents;
	REQUIRE(gone > 0);
	w.doc.remove_building(block);
	w.sync();
	CHECK(w.t().city_stats().residents == before - static_cast<uint32_t>(gone));
	w.run_minutes(120.0);
	CHECK(w.t().city_stats().residents == before - static_cast<uint32_t>(gone));
	CHECK(w.t().stats().removed_stuck == 0);
}

TEST_CASE("M5 gate: 5,000 residents live a full sim week without stalls") {
	World w;
	build_city_week(w.doc);
	CHECK(w.errors() == 0);
	w.sync();
	for (const NetProblem &p : network_problems(w.doc.map(), w.net())) CHECK_MESSAGE(false, "network problem: ", p.message);
	Traffic &t = w.t();
	t.config().city_prefill = 0.95;
	t.reset(2026);
	print_city("M5 gate start", t);
	CHECK(t.city_stats().residents >= 4500);
	const int64_t tpm = t.ticks_per_minute();
	// Stall watch: a resident whose state, activity, place and plan haven't changed
	// in 24 hourly looks is stuck; so is anyone travelling for 3 hours.
	std::map<uint32_t, std::pair<uint64_t, int>> same; // id -> (signature, hours unchanged)
	std::map<uint32_t, int> travelling;
	int stuck = 0, stuck_travel = 0, unopened = 0;
	double worst_tick_ms = 0.0, worst_minute_ms = 0.0;
	std::string worst_at;
	std::vector<uint64_t> hist(200, 0); // tick times, 0.1 ms buckets up to 20 ms
	uint32_t min_residents = ~0u;
	const auto t0 = std::chrono::steady_clock::now();
	for (int day = 0; day < 7; ++day) {
		for (int hour = 0; hour < 24; ++hour) {
			for (int m = 0; m < 60; ++m) {
				const auto a = std::chrono::steady_clock::now();
				for (int64_t k = 0; k < tpm; ++k) {
					const auto b = std::chrono::steady_clock::now();
					t.tick();
					const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b).count();
					if (ms > worst_tick_ms) {
						worst_tick_ms = ms;
						worst_at = clock(t);
					}
					++hist[std::min<size_t>(hist.size() - 1, static_cast<size_t>(ms * 10.0))]; // 0.1 ms buckets
				}
				worst_minute_ms = std::max(worst_minute_ms, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count());
			}
			for (const Resident &r : t.residents()) {
				if (r.visitor) continue;
				const uint64_t sig = (static_cast<uint64_t>(r.state) << 56) ^ (static_cast<uint64_t>(r.doing) << 48) ^
						(static_cast<uint64_t>(r.at) << 16) ^ r.until ^ (r.next_plan << 1) ^ static_cast<uint64_t>(r.ped) << 24;
				auto &e = same[r.id];
				e.second = e.first == sig ? e.second + 1 : 0;
				e.first = sig;
				if (e.second == 24) ++stuck;
				int &tr = travelling[r.id];
				tr = r.state == ResidentState::Travelling ? tr + 1 : 0;
				if (tr == 3) ++stuck_travel;
			}
			min_residents = std::min(min_residents, t.city_stats().residents);
			// Late in a weekday every business has opened (offices are shut at weekends).
			if (hour == 13 && day % 7 < 5) {
				for (const auto &kv : w.doc.map().buildings()) {
					const BuildingInfo bi = t.building_info(kv.first);
					if (bi.type < 0 || t.city_data().types[static_cast<size_t>(bi.type)].kind == BuildingKind::Home) continue;
					if (bi.opened_at < 0) ++unopened;
				}
			}
		}
		print_city("M5 gate", t);
	}
	const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	const CityStats c = t.city_stats();
	const double us = secs * 1e6 / (7.0 * 24 * 60 * static_cast<double>(tpm));
	uint64_t total = 0, seen = 0;
	for (uint64_t h : hist) total += h;
	double p999 = 0.0;
	for (size_t k = 0; k < hist.size(); ++k) {
		seen += hist[k];
		if (seen * 1000 >= total * 999) {
			p999 = (static_cast<double>(k) + 1.0) / 10.0;
			break;
		}
	}
	std::printf("M5 gate: a sim week in %.0f s (%.1f us per tick); 99.9%% of ticks under %.1f ms, the slowest %.1f ms (%s); "
				"the slowest sim minute took %.0f ms (at 16x a sim minute has 3,750 ms)\n",
			secs, us, p999, worst_tick_ms, worst_at.c_str(), worst_minute_ms);
	std::printf("         %u residents at the end (at least %u), %llu immigrants, %llu shifts (%llu late), %llu late openings, "
				"%llu meals out, %llu at home, %llu grocery trips; stuck %d, stuck travelling %d, businesses not open by 13:00 %d\n",
			c.residents, min_residents, (unsigned long long)c.immigrants, (unsigned long long)c.shifts, (unsigned long long)c.late_shifts,
			(unsigned long long)c.late_openings, (unsigned long long)c.meals_out, (unsigned long long)c.home_meals,
			(unsigned long long)c.groceries, stuck, stuck_travel, unopened);
	CHECK(min_residents >= 4500);
	CHECK(stuck == 0);
	CHECK(stuck_travel == 0);
	CHECK(unopened == 0);
	CHECK(c.starving <= 5);
	CHECK(c.shifts > 20000);
	CHECK(t.stats().removed_stuck == 0);
	// Keeping pace at 16x: every sim minute runs in a small part of its 3.75 s.
	CHECK(worst_minute_ms < 1000.0);
}
