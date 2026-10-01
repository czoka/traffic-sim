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

TEST_CASE("city town: valid, round-trips as v6, doors on the sidewalk") {
	World w;
	build_city_town(w.doc);
	CHECK(w.errors() == 0);
	const std::string json = road_map_to_json(w.doc.map());
	CHECK(json.find("\"version\": 6") != std::string::npos);
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
	World w;
	build_city_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 1.0;
	t.reset(3);
	REQUIRE(t.city_on());
	print_city("city town start", t);
	for (int h = 0; h < 48; ++h) {
		w.run_minutes(60.0);
		if (h % 6 == 5) print_city("city town", t);
	}
	const CityStats c = t.city_stats();
	CHECK(c.residents > 50);
	CHECK(c.sleeps > c.residents);
	CHECK(c.home_meals + c.meals_out > c.residents * 2);
	CHECK(c.groceries > 0);
	CHECK(c.shifts > 0);
	CHECK(c.starving == 0);
	CHECK(c.min_hunger > 0.0);
}
