// M6 tests: the economy - money between residents and owners, transport costs,
// the bus pass, bikes and cars, location prices, rent and eviction, NPC owners,
// the market, and the M6 gate (a city runs two sim months without runaway
// prices or mass evictions).
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
	void run_days(int days) { run_minutes(days * 1440.0); }
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
	uint32_t first_of(const char *type) const {
		for (const auto &kv : doc.map().buildings()) {
			if (kv.second.type == type) return kv.first;
		}
		return 0;
	}
};

PointRef pt(double x, double y) {
	PointRef p;
	p.pos = Vec2{ x, y };
	return p;
}

Profile preset(const char *name) {
	RoadMap scratch;
	return preset_profile(name, scratch);
}

Spawner sink() {
	Spawner s;
	s.enabled = true;
	s.sink = true;
	return s;
}

void print_money(const char *label, const Traffic &t) {
	const CityStats c = t.city_stats();
	std::printf("%s [month %d day %d]: %u residents in %u households, mean money %.0f, %u in debt, %llu evicted; city +%.0f "
				"-%.0f (rent %.0f, sales %.0f, passes %.0f, buildings %.0f; wages %.0f, goods %.0f, buildings %.0f)\n",
			label, c.month, c.day_of_month, c.residents, c.households, c.mean_money, c.in_debt,
			(unsigned long long)c.evictions, c.treasury_income, c.treasury_spending, c.income_rent, c.income_sales,
			c.income_passes, c.income_buildings, c.spending_wages, c.spending_goods, c.spending_buildings);
	std::printf("    %u bikes, %u cars, %u passes; trips walk %llu bus %llu bike %llu car %llu coach %llu; owners: %u city, %u "
				"NPC, %u homes by households, %u listed, %llu sold, %llu bought; NPC prices x%.2f-%.2f, wages x%.2f-%.2f, "
				"rents x%.2f-%.2f\n",
			c.bikes, c.cars, c.passes, (unsigned long long)c.trips_walk, (unsigned long long)c.trips_bus,
			(unsigned long long)c.trips_bike, (unsigned long long)c.trips_car, (unsigned long long)c.trips_coach,
			c.city_owned, c.npc_owned, c.homes_owned, c.listed, (unsigned long long)c.buildings_sold,
			(unsigned long long)c.buildings_bought, c.price_factor_min, c.price_factor_max, c.wage_factor_min,
			c.wage_factor_max, c.rent_factor_min, c.rent_factor_max);
}

// A street with sinks at both ends, an apartment block and a grocery.
struct Street {
	uint32_t home = 0, grocery = 0;
};

Street small_street(World &w) {
	Document &doc = w.doc;
	const SegmentId road = doc.add_road({ pt(-200, 0), pt(200, 0) }, preset("Street 1+1"), 0, 13.9).front();
	doc.set_spawner(doc.map().segment(road)->from, sink());
	doc.set_spawner(doc.map().segment(road)->to, sink());
	w.geom.build(doc.map());
	Street s;
	s.grocery = place_building(doc, w.geom, "grocery", Vec2{ 0, -20 });
	s.home = place_building(doc, w.geom, "apartment_block", Vec2{ 60, 20 });
	return s;
}

} // namespace

TEST_CASE("M6 data: the economy table, the bike shop and the car dealership") {
	const CityData &d = default_city_data();
	CHECK(d.month_days == 30);
	CHECK(d.eviction_months == 2);
	CHECK(d.coach_fare > 0.0);
	CHECK(d.car_cost_per_km > 0.0);
	CHECK(d.bus_pass > 0.0);
	CHECK(d.edge_factor == doctest::Approx(0.5));
	CHECK(d.edge_distance == doctest::Approx(3000.0));
	REQUIRE(d.offering("buy_bike") != nullptr);
	REQUIRE(d.offering("buy_car") != nullptr);
	CHECK(d.offering("buy_bike")->unlocks == "bike");
	CHECK(d.offering("buy_car")->unlocks == "car");
	CHECK(d.offering("buy_car")->price > d.offering("buy_bike")->price * 5.0);
	CHECK(d.offering("buy_car")->cost < d.offering("buy_car")->price);
	REQUIRE(d.type("bike_shop") != nullptr);
	REQUIRE(d.type("car_dealership") != nullptr);
	CHECK(d.type("bike_shop")->kind == BuildingKind::Shop);
	CHECK(d.type("office_small")->revenue > d.type("office_small")->wage); // offices earn more than they pay
	CityData bad;
	std::string err;
	REQUIRE(parse_city_data(R"({"economy":{"month_days":0,"eviction_months":0},"offerings":{},"buildings":{}})", bad, &err));
	CHECK(bad.month_days == 1); // at least a day
	CHECK(bad.eviction_months == 1);
	CHECK(!parse_city_data(R"({"offerings":{"x":{"unlocks":"boat"}},"buildings":{}})", bad, &err));
	CHECK(err.find("unlocks") != std::string::npos);
}

TEST_CASE("map v7: economy settings and the city centre round-trip; undo; v6 files load") {
	Document doc;
	build_city_market(doc);
	REQUIRE(doc.map().city_centre().has_value());
	Building b = doc.map().buildings().begin()->second;
	b.rent = 777.0;
	b.price_factor = 1.25;
	b.wage = 21.5;
	b.for_sale = true;
	b.asking = 123456.0;
	doc.set_building(b);
	const std::string json = road_map_to_json(doc.map());
	CHECK(json.find("\"version\": 7") != std::string::npos);
	CHECK(json.find("\"city_centre\"") != std::string::npos);
	RoadMap back;
	std::string err;
	REQUIRE_MESSAGE(road_map_from_json(json, back, &err), err);
	CHECK(back == doc.map());
	CHECK(road_map_to_json(back) == json);
	CHECK(back.building(b.id)->asking == 123456.0);
	// The centre marker is undoable.
	const Vec2 c0 = *doc.map().city_centre();
	doc.set_city_centre(Vec2{ 10.0, 20.0 });
	CHECK(doc.map().city_centre()->x == 10.0);
	doc.set_city_centre(std::nullopt);
	CHECK(!doc.map().city_centre().has_value());
	CHECK(doc.undo());
	CHECK(doc.map().city_centre()->x == 10.0);
	CHECK(doc.undo());
	CHECK(doc.map().city_centre()->x == c0.x);
	CHECK(doc.redo());
	CHECK(doc.map().city_centre()->y == 20.0);
	// Bad numbers are refused.
	std::string bad = json;
	const size_t at = bad.find("\"rent\": 777");
	REQUIRE(at != std::string::npos);
	bad.replace(at, 11, "\"rent\": -5");
	CHECK(!road_map_from_json(bad, back, &err));
	CHECK(err.find("economy") != std::string::npos);
	// A v6 file (M5) loads with the defaults.
	Document old;
	build_city_town(old);
	std::string v6 = road_map_to_json(old.map());
	const size_t v = v6.find("\"version\": 7");
	REQUIRE(v != std::string::npos);
	v6.replace(v, 12, "\"version\": 6");
	REQUIRE_MESSAGE(road_map_from_json(v6, back, &err), err);
	CHECK(!back.city_centre().has_value());
	for (const auto &kv : back.buildings()) {
		CHECK(kv.second.rent == 0.0);
		CHECK(!kv.second.for_sale);
	}
}

TEST_CASE("location: homes cost more near the city centre, half at 3 km") {
	World w;
	build_city_market(w.doc);
	CHECK(w.errors() == 0);
	w.sync();
	Traffic &t = w.t();
	t.reset(1);
	const Vec2 centre = *w.doc.map().city_centre();
	double near_loc = 0.0, far_loc = 2.0;
	for (const NetBuilding &nb : w.net().buildings) {
		const BuildingInfo b = t.building_info(nb.id);
		const double d = (nb.centre - centre).length();
		CHECK(b.location == doctest::Approx(1.0 - 0.5 * std::min(1.0, d / 3000.0)));
		if (nb.kind == BuildingKind::Home) {
			const BuildingType &ty = t.city_data().types[static_cast<size_t>(nb.type)];
			CHECK(b.base_rent == doctest::Approx(ty.rent * b.location));
			CHECK(b.rent == doctest::Approx(b.base_rent)); // the city charges the default
		}
		near_loc = std::max(near_loc, b.location);
		far_loc = std::min(far_loc, b.location);
	}
	std::printf("location factors from %.3f to %.3f\n", far_loc, near_loc);
	CHECK(near_loc > far_loc);
	// Moving the centre 3 km away halves every price.
	w.doc.set_city_centre(Vec2{ centre.x + 10000.0, centre.y });
	w.sync();
	for (const NetBuilding &nb : w.net().buildings) CHECK(t.building_info(nb.id).location == doctest::Approx(0.5));
	// The player's rent overrides the default while the city owns the home.
	const uint32_t flat = w.first_of("apartment_block");
	Building b = *w.doc.map().building(flat);
	b.rent = 640.0;
	w.doc.set_building(b);
	w.sync();
	CHECK(t.building_info(flat).rent == doctest::Approx(640.0));
}

TEST_CASE("money: wages, prices and rent move between residents and owners") {
	World w;
	build_city_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 1.0;
	t.reset(7);
	const CityStats c0 = t.city_stats();
	// Rent day is the first day: everyone paid the city.
	CHECK(c0.income_rent > 0.0);
	double rents = 0.0;
	for (const Household &hh : t.households()) rents += hh.rent;
	CHECK(c0.income_rent == doctest::Approx(rents));
	w.run_days(2);
	print_money("city town after 2 days", t);
	const CityStats c = t.city_stats();
	CHECK(c.income_sales > 0.0); // meals and groceries in city shops
	CHECK(c.spending_wages > 0.0); // the city pays its staff
	CHECK(c.spending_goods > 0.0); // and buys stock
	CHECK(c.wages_paid > 0.0);
	CHECK(c.outside_fares > 0.0); // coach fares leave the map
	CHECK(c.in_debt == 0);
	CHECK(c.evictions == 0);
	// A business shows what it earned and paid this month.
	const uint32_t g = w.first_of("grocery");
	const BuildingInfo b = t.building_info(g);
	CHECK(b.owner == 0);
	CHECK(b.income_month > 0.0);
	CHECK(b.expense_month > 0.0);
	CHECK(b.value > 0.0);
	// The city's accounts add up: what it earned less what it spent.
	CHECK(c.treasury_income == doctest::Approx(c.income_rent + c.income_sales + c.income_passes + c.income_buildings));
	CHECK(c.treasury_spending ==
			doctest::Approx(c.spending_wages + c.spending_goods + c.spending_buildings).epsilon(1e-6));
}

TEST_CASE("bus pass: residents buy a month of buses, visitors a day pass") {
	World w;
	build_city_market(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 0.95;
	t.reset(3);
	w.run_days(1);
	const CityStats c = t.city_stats();
	print_money("city market after a day", t);
	const CityData &cd = t.city_data();
	CHECK(c.passes_sold > 0);
	CHECK(c.trips_bus > 0);
	CHECK(c.income_passes == doctest::Approx(static_cast<double>(c.passes_sold) * cd.bus_pass +
			static_cast<double>(c.day_passes) * cd.bus_pass * cd.day_pass_share));
	int with_pass = 0;
	for (const Resident &r : t.residents()) {
		if (r.visitor) continue;
		if (t.resident_info(r.id).has_pass) ++with_pass;
	}
	CHECK(with_pass == static_cast<int>(c.passes));
}

TEST_CASE("eviction: a household in debt on two rent days in a row leaves") {
	CityData data = default_city_data();
	data.month_days = 1; // rent every day
	data.money_low = 0.0;
	World w;
	const Street s = small_street(w);
	REQUIRE(s.home != 0);
	Building b = *w.doc.map().building(s.home);
	b.rent = 25000.0; // far more than anyone has
	w.doc.set_building(b);
	w.sync();
	Traffic &t = w.t();
	t.set_city_data(&data);
	t.reset(9);
	for (int k = 0; k < 4; ++k) REQUIRE(t.add_household(s.home, 2, false) >= 0);
	w.run_minutes(60.0);
	CHECK(t.city_stats().households == 4);
	w.run_days(1); // first rent day in debt
	CHECK(t.city_stats().in_debt == 4);
	CHECK(t.city_stats().evictions == 0);
	for (const Household &hh : t.households()) {
		if (!hh.members.empty()) CHECK(hh.debt_months == 1);
	}
	w.run_days(1); // second: out
	const CityStats c = t.city_stats();
	print_money("eviction street", t);
	CHECK(c.households_evicted == 4);
	CHECK(c.evictions == 8);
	CHECK(t.building_info(s.home).households == 0);
	w.run_minutes(6 * 60.0);
	for (const Resident &r : t.residents()) CHECK(!r.evicted); // gone by coach
}

TEST_CASE("cars: residents drive from home and park there; outside jobs by car via a map edge") {
	World w;
	build_city_market(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 0.95;
	t.reset(11);
	int given = 0;
	for (const Resident &r : t.residents()) {
		if (!r.visitor && t.give_vehicle(r.id, true)) ++given;
	}
	CHECK(given > 500);
	CHECK(t.city_stats().cars == static_cast<uint32_t>(given));
	int outside_seen = 0;
	for (int h = 0; h < 40; ++h) { // to 22:00 the next day
		w.run_minutes(60.0);
		int n = 0;
		for (const Resident &r : t.residents()) n += r.car_at == kOutside ? 1 : 0;
		outside_seen = std::max(outside_seen, n);
	}
	print_money("everyone has a car, 40 h", t);
	const CityStats c = t.city_stats();
	const TrafficStats ts = t.stats();
	std::printf("car trips %llu, at most %d cars outside the map, fuel outside %.0f, stuck %llu, unroutable %llu\n",
			(unsigned long long)c.trips_car, outside_seen, c.outside_fuel, (unsigned long long)ts.removed_stuck,
			(unsigned long long)ts.unroutable);
	CHECK(c.trips_car > 500);
	CHECK(outside_seen > 20); // commuters drive out by the north road
	CHECK(c.outside_fuel > 0.0);
	CHECK(ts.removed_stuck == 0);
	// Every car is at home or with its owner: parked where they are, being
	// driven, or outside the map with them (someone with a bike as well rides
	// one and leaves the other); in the evening most are at home.
	int home = 0, cars = 0, with_owner = 0;
	for (const Resident &r : t.residents()) {
		if (r.visitor || !r.has_car || r.has_bike) continue;
		++cars;
		const bool at_home = r.household >= 0 && r.car_at == t.households()[static_cast<size_t>(r.household)].home;
		home += at_home ? 1 : 0;
		const bool here = r.state == ResidentState::Inside && r.car_at == r.at;
		const bool out = r.state == ResidentState::Outside && r.car_at == kOutside;
		with_owner += at_home || here || out || r.veh != kNoId ? 1 : 0;
	}
	std::printf("%d of %d cars parked at home at %02d:%02d, %d at home or with their owner\n", home, cars, t.minute_of_day() / 60,
			t.minute_of_day() % 60, with_owner);
	CHECK(home > cars * 8 / 10);
	CHECK(with_owner == cars);
}

TEST_CASE("bikes: a resident with a bike rides it") {
	World w;
	build_city_market(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.config().city_prefill = 0.95;
	t.reset(13);
	int given = 0;
	for (const Resident &r : t.residents()) {
		if (!r.visitor && given < 300 && t.give_vehicle(r.id, false)) ++given;
	}
	CHECK(given == 300);
	w.run_minutes(14 * 60.0);
	const CityStats c = t.city_stats();
	std::printf("300 bikes: %llu bike trips, %llu walk, %llu bus by 20:00\n", (unsigned long long)c.trips_bike,
			(unsigned long long)c.trips_walk, (unsigned long long)c.trips_bus);
	CHECK(c.trips_bike > 100);
	CHECK(c.bikes >= 300);
	CHECK(t.stats().removed_stuck == 0);
}

TEST_CASE("market: a player listing sells with its tenants; NPC owners tune; the player buys back") {
	CityData data = default_city_data();
	data.npc_list_chance = 1.0; // every NPC owner lists its building on the 1st
	data.sale_rate = 0.5; // a bargain sells the next day
	data.month_days = 5;
	World w;
	build_city_town(w.doc);
	const uint32_t flat = w.first_of("apartment_block");
	REQUIRE(flat != 0);
	w.sync();
	Traffic &t = w.t();
	t.set_city_data(&data);
	t.config().city_prefill = 1.0;
	t.reset(21);
	w.run_minutes(60.0);
	const BuildingInfo before = t.building_info(flat);
	REQUIRE(before.owner == 0);
	REQUIRE(before.residents > 0);
	const CityStats c0 = t.city_stats();
	// The player lists it cheap: buyers think it's worth far more.
	Building b = *w.doc.map().building(flat);
	b.for_sale = true;
	b.asking = 1000.0;
	w.doc.set_building(b);
	w.sync();
	bool listed = false;
	for (const Listing &l : t.market()) listed |= l.building == flat && l.by_city && l.asking == 1000.0;
	CHECK(listed);
	CHECK(t.building_value(flat) > 100000.0);
	w.run_days(2);
	BuildingInfo f = t.building_info(flat);
	std::printf("flat: owner %u after 2 days; %d tenants, %d before\n", f.owner, f.residents, before.residents);
	REQUIRE(f.owner != 0);
	CHECK(f.households == before.households); // sold with its tenants
	CHECK(t.city_stats().income_buildings - c0.income_buildings == doctest::Approx(1000.0));
	CHECK(t.city_stats().buildings_sold == c0.buildings_sold + 1);
	for (const Listing &l : t.market()) CHECK(!(l.building == flat && l.by_city)); // the map's tick no longer lists it
	const double rent0 = f.rent;
	data.sale_rate = 0.0; // no more buyers: the NPC listing waits for the player
	// The 1st of the next month: the new owner tunes the rent and lists it.
	w.run_days(4);
	f = t.building_info(flat);
	print_money("market town", t);
	std::printf("flat rent %.0f -> %.0f, listed %d at %.0f (value %.0f)\n", rent0, f.rent, f.listed, f.asking, f.value);
	CHECK(f.rent != doctest::Approx(rent0)); // full: dearer (or vacant: cheaper)
	REQUIRE(f.listed);
	bool on_market = false;
	for (const Listing &l : t.market()) on_market |= l.building == flat && !l.by_city && l.owner == f.owner;
	CHECK(on_market);
	// The player buys it back.
	const double spent = t.city_stats().spending_buildings;
	const double asking = f.asking;
	CHECK(t.buy_building(flat));
	CHECK(!t.buy_building(flat)); // not for sale any more
	f = t.building_info(flat);
	CHECK(f.owner == 0);
	CHECK(f.households == before.households);
	CHECK(t.city_stats().spending_buildings - spent == doctest::Approx(asking));
	CHECK(t.city_stats().buildings_bought == 1);
	// The city's own buildings that it hasn't listed can't be bought.
	CHECK(!t.buy_building(w.first_of("office_small")));
}

TEST_CASE("homes: a household that has saved enough buys the house it lives in") {
	CityData data = default_city_data();
	for (BuildingType &ty : data.types) {
		if (ty.id == "detached_house") ty.price = 1000.0;
	}
	data.value_years = 0.1; // a year of rent: buyers pay little
	data.month_days = 2;
	World w;
	build_city_town(w.doc);
	w.sync();
	Traffic &t = w.t();
	t.set_city_data(&data);
	t.config().city_prefill = 1.0;
	t.reset(5);
	w.run_days(3);
	const CityStats c = t.city_stats();
	print_money("cheap houses", t);
	CHECK(c.homes_bought > 0);
	CHECK(c.homes_owned > 0);
	int owners = 0;
	for (const Household &hh : t.households()) {
		if (!hh.owns) continue;
		++owners;
		const BuildingInfo b = t.building_info(hh.home);
		CHECK(b.owner_household == hh.id);
		CHECK(t.city_data().types[static_cast<size_t>(b.type)].households == 1);
		for (uint32_t m : hh.members) CHECK(t.resident_info(m).owns_home);
	}
	CHECK(owners == static_cast<int>(c.homes_owned));
	CHECK(c.income_buildings > 0.0);
}

TEST_CASE("M6: same seed, same hash") {
	auto run = [](uint64_t seed) {
		World w;
		build_city_market(w.doc);
		w.sync();
		w.t().config().city_prefill = 0.95;
		w.t().reset(seed);
		for (const Resident &r : w.t().residents()) {
			if (!r.visitor && r.id % 4 == 0) w.t().give_vehicle(r.id, r.id % 8 == 0);
		}
		w.run_minutes(240.0);
		return w.t().state_hash();
	};
	const uint64_t a = run(42);
	CHECK(a == run(42));
	CHECK(a != run(43));
	std::printf("M6 golden: %016llx (expected 04d04d8a98ca699d)\n", static_cast<unsigned long long>(a));
	CHECK(a == 0x04d04d8a98ca699dull); // same on every platform, like the other goldens
}

TEST_CASE("M6 gate: a city runs two sim months with no runaway prices and no mass evictions") {
	World w;
	build_city_market(w.doc);
	CHECK(w.errors() == 0);
	w.sync();
	for (const NetProblem &p : network_problems(w.doc.map(), w.net())) CHECK_MESSAGE(false, "network problem: ", p.message);
	Traffic &t = w.t();
	t.config().city_prefill = 0.95;
	t.reset(2026);
	print_money("M6 gate start", t);
	const CityStats c0 = t.city_stats();
	CHECK(c0.residents >= 1000);
	const auto t0 = std::chrono::steady_clock::now();
	uint32_t worst_debt = 0, min_households = c0.households;
	for (int day = 0; day < 60; ++day) {
		w.run_days(1);
		const CityStats c = t.city_stats();
		worst_debt = std::max(worst_debt, c.in_debt);
		min_households = std::min(min_households, c.households);
		if (day % 10 == 9) print_money("M6 gate", t);
	}
	const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	const CityStats c = t.city_stats();
	const TrafficStats ts = t.stats();
	std::printf("M6 gate: 60 days in %.1f s; households %u -> %u (min %u), mean money %.0f -> %.0f, worst %u in debt, "
				"%llu households evicted, %llu buildings sold, %llu bikes and %llu cars bought, %llu homes bought, %llu "
				"stuck\n",
			secs, c0.households, c.households, min_households, c0.mean_money, c.mean_money, worst_debt,
			(unsigned long long)c.households_evicted, (unsigned long long)c.buildings_sold,
			(unsigned long long)c.bikes_bought, (unsigned long long)c.cars_bought, (unsigned long long)c.homes_bought,
			(unsigned long long)ts.removed_stuck);
	const CityData &cd = t.city_data();
	// No runaway prices: NPC numbers stay well inside their limits.
	CHECK(c.price_factor_max < cd.price_max);
	CHECK(c.price_factor_min > cd.price_min);
	CHECK(c.rent_factor_max < 1.5);
	CHECK(c.rent_factor_min > 0.6);
	CHECK(c.wage_factor_max < 1.3);
	// No mass evictions or debt; money doesn't run away either way.
	CHECK(c.households_evicted * 50 < c0.households); // under 2 %
	CHECK(worst_debt * 20 < c0.households); // under 5 %
	CHECK(c.households * 10 > c0.households * 9);
	CHECK(c.mean_money > c0.mean_money * 0.5);
	CHECK(c.mean_money < c0.mean_money * 2.0);
	CHECK(c.starving == 0);
	// The market and the shops are used.
	CHECK(c.buildings_sold > 3);
	CHECK(c.bikes_bought > 0);
	CHECK(c.passes > 0);
	CHECK(ts.removed_stuck == 0);
}
