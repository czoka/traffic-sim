// RoadEditor: M5 editing (buildings) and city life in the sim (residents,
// businesses, the clock).
#include "m3_dicts.h"
#include "road_editor.h"

#include "tsim/buildings.h"
#include "tsim/city_data.h"
#include "tsim/demo_maps.h"

#include <godot_cpp/core/class_db.hpp>

#include <algorithm>
#include <cmath>

namespace godot {

using namespace tsim;
using namespace m3;

namespace {

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }
Vec2 tv(Vector2 v) { return Vec2{ v.x, v.y }; }

PackedVector2Array quad(const std::array<Vec2, 4> &q) {
	PackedVector2Array out;
	for (const Vec2 &p : q) out.push_back(gv(p));
	return out;
}

Array day_dict(const DayPlan &d) {
	Array out;
	if (!d.open) return out;
	for (const ShiftSpec &s : d.shifts) {
		Dictionary sd;
		sd["start"] = s.start;
		sd["end"] = s.end;
		sd["staff"] = s.staff;
		out.push_back(sd);
	}
	return out;
}

const char *state_name(ResidentState s) {
	switch (s) {
		case ResidentState::Inside:
			return "inside";
		case ResidentState::Travelling:
			return "travelling";
		case ResidentState::Outside:
			return "outside";
	}
	return "inside";
}

} // namespace

void RoadEditor::new_city() {
	Document d;
	build_new_city(d);
	doc_.reset(d.map());
}

Array RoadEditor::building_types() const {
	Array out;
	for (const BuildingType &t : default_city_data().types) {
		Dictionary d;
		d["id"] = gstr(t.id);
		d["label"] = gstr(t.label);
		d["kind"] = building_kind_name(t.kind);
		d["width"] = t.width;
		d["depth"] = t.depth;
		d["color"] = static_cast<int64_t>(t.color);
		d["households"] = t.households;
		d["slots"] = t.slots;
		d["desks"] = t.desks;
		d["wage"] = t.wage;
		d["rent"] = t.rent;
		d["parking"] = gstr(t.parking);
		d["city"] = t.city;
		d["headcount"] = t.headcount(default_city_data().weekly_hours);
		d["weekday_open"] = t.weekday.open;
		d["weekday_hours"] = Vector2i(t.weekday.open_at, t.weekday.close_at);
		d["weekend_open"] = t.weekend.open;
		d["weekend_hours"] = Vector2i(t.weekend.open_at, t.weekend.close_at);
		d["weekday_shifts"] = day_dict(t.weekday);
		d["weekend_shifts"] = day_dict(t.weekend);
		d["min_staff"] = t.min_staff;
		Array offers;
		for (const std::string &o : t.offers) {
			const Offering *of = default_city_data().offering(o);
			offers.push_back(of ? gstr(of->label) : gstr(o));
		}
		d["offers"] = offers;
		out.push_back(d);
	}
	return out;
}

Dictionary RoadEditor::snap_building(const String &type, Vector2 near, int level, int64_t ignore) {
	ensure_geometry();
	Dictionary d;
	const LotSnap snap = snap_lot(geom_, tv(near), level);
	d["ok"] = snap.ok;
	if (!snap.ok) return d;
	Building b;
	b.type = str(type);
	b.pos = snap.pos + snap.dir * 0.5;
	b.dir = snap.dir;
	b.level = level;
	const std::array<Vec2, 4> lot = lot_corners(b, default_city_data());
	d["pos"] = gv(b.pos);
	d["dir"] = gv(b.dir);
	d["corners"] = quad(lot);
	bool blocked = false;
	for (const auto &kv : doc_.map().buildings()) {
		if (static_cast<int64_t>(kv.first) == ignore) continue;
		if (kv.second.level == level && quads_overlap(lot, lot_corners(kv.second, default_city_data()))) blocked = true;
	}
	for (const auto &kv : geom_.segments()) {
		if (kv.second.level == level && lot_overlaps_road(lot, kv.second)) blocked = true;
	}
	d["blocked"] = blocked;
	return d;
}

int64_t RoadEditor::add_building(const String &type, Vector2 pos, Vector2 dir, int level) {
	return doc_.add_building(str(type), tv(pos), tv(dir), level);
}

void RoadEditor::move_building(int64_t id, Vector2 pos, Vector2 dir) {
	const Building *b = doc_.map().building(static_cast<uint32_t>(id));
	if (!b) return;
	Building nb = *b;
	nb.pos = tv(pos);
	nb.dir = tv(dir);
	if (nb == *b) return;
	doc_.set_building(nb);
}

void RoadEditor::set_building_name(int64_t id, const String &name) {
	const Building *b = doc_.map().building(static_cast<uint32_t>(id));
	if (!b) return;
	Building nb = *b;
	nb.name = str(name);
	doc_.set_building(nb);
}

void RoadEditor::remove_building(int64_t id) { doc_.remove_building(static_cast<uint32_t>(id)); }

Array RoadEditor::get_buildings(int level) {
	ensure_geometry();
	Array out;
	const CityData &cd = default_city_data();
	for (const auto &kv : doc_.map().buildings()) {
		const Building &b = kv.second;
		if (b.level != level) continue;
		const BuildingType *t = cd.type(b.type);
		const std::array<Vec2, 4> lot = lot_corners(b, cd);
		Dictionary d;
		d["id"] = static_cast<int64_t>(b.id);
		d["type"] = gstr(b.type);
		d["label"] = t ? gstr(t->label) : gstr(b.type);
		d["kind"] = t ? building_kind_name(t->kind) : "home";
		d["name"] = gstr(b.name);
		d["corners"] = quad(lot);
		d["centre"] = gv((lot[0] + lot[2]) * 0.5);
		d["door"] = gv(b.pos);
		out.push_back(d);
	}
	return out;
}

int64_t RoadEditor::pick_building(Vector2 pos, int level) {
	const CityData &cd = default_city_data();
	for (const auto &kv : doc_.map().buildings()) {
		if (kv.second.level != level) continue;
		if (point_in_quad(lot_corners(kv.second, cd), tv(pos))) return static_cast<int64_t>(kv.first);
	}
	return 0;
}

Dictionary RoadEditor::get_building(int64_t id) {
	Dictionary d;
	const Building *b = doc_.map().building(static_cast<uint32_t>(id));
	if (!b) return d;
	const CityData &cd = default_city_data();
	const BuildingType *t = cd.type(b->type);
	d["id"] = id;
	d["type"] = gstr(b->type);
	d["label"] = t ? gstr(t->label) : gstr(b->type);
	d["kind"] = t ? building_kind_name(t->kind) : "unknown";
	d["name"] = gstr(b->name);
	d["level"] = b->level;
	const std::array<Vec2, 4> lot = lot_corners(*b, cd);
	d["corners"] = quad(lot);
	d["centre"] = gv((lot[0] + lot[2]) * 0.5);
	ensure_network();
	const int32_t bi = check_net_.building_index(b->id);
	d["door"] = bi >= 0 && check_net_.buildings[static_cast<size_t>(bi)].entrance >= 0;
	// M6: the player's numbers (0 = the default).
	d["rent"] = b->rent;
	d["price_factor"] = b->price_factor;
	d["wage"] = b->wage;
	d["for_sale"] = b->for_sale;
	d["asking"] = b->asking;
	if (t) {
		d["type_rent"] = t->rent;
		d["type_price"] = t->price;
		d["type_wage"] = t->wage;
		d["households"] = t->households;
	}
	return d;
}

// --- City life in the sim -------------------------------------------------------------------

Dictionary RoadEditor::sim_city_stats() {
	const Traffic &t = sim_.traffic();
	const CityStats c = t.city_stats();
	Dictionary d;
	d["on"] = c.on;
	d["day"] = c.day;
	d["weekday"] = c.weekday;
	d["minute"] = c.minute;
	d["residents"] = static_cast<int64_t>(c.residents);
	d["households"] = static_cast<int64_t>(c.households);
	d["visitors"] = static_cast<int64_t>(c.visitors);
	d["inside"] = static_cast<int64_t>(c.inside);
	d["travelling"] = static_cast<int64_t>(c.travelling);
	d["outside"] = static_cast<int64_t>(c.outside);
	d["sleeping"] = static_cast<int64_t>(c.sleeping);
	d["working"] = static_cast<int64_t>(c.working);
	d["employed"] = static_cast<int64_t>(c.employed);
	d["employed_outside"] = static_cast<int64_t>(c.employed_outside);
	d["unemployed"] = static_cast<int64_t>(c.unemployed);
	d["homes"] = static_cast<int64_t>(c.homes);
	d["units"] = static_cast<int64_t>(c.units);
	d["vacant_units"] = static_cast<int64_t>(c.vacant_units);
	d["businesses"] = static_cast<int64_t>(c.businesses);
	d["open"] = static_cast<int64_t>(c.open);
	d["closed_unexpectedly"] = static_cast<int64_t>(c.closed_unexpectedly);
	d["immigrants"] = static_cast<int64_t>(c.immigrants);
	d["visitor_trips"] = static_cast<int64_t>(c.visitor_trips);
	d["meals_out"] = static_cast<int64_t>(c.meals_out);
	d["home_meals"] = static_cast<int64_t>(c.home_meals);
	d["groceries"] = static_cast<int64_t>(c.groceries);
	d["shifts"] = static_cast<int64_t>(c.shifts);
	d["late_shifts"] = static_cast<int64_t>(c.late_shifts);
	d["late_openings"] = static_cast<int64_t>(c.late_openings);
	d["turned_away"] = static_cast<int64_t>(c.turned_away);
	d["unfilled_shifts"] = static_cast<int64_t>(c.unfilled_shifts);
	d["mean_hunger"] = c.mean_hunger;
	d["mean_energy"] = c.mean_energy;
	d["mean_money"] = c.mean_money;
	d["starving"] = static_cast<int64_t>(c.starving);
	// M6
	d["month"] = c.month;
	d["day_of_month"] = c.day_of_month;
	d["treasury_income"] = c.treasury_income;
	d["treasury_spending"] = c.treasury_spending;
	d["month_income"] = c.month_income;
	d["month_spending"] = c.month_spending;
	d["income_rent"] = c.income_rent;
	d["income_sales"] = c.income_sales;
	d["income_passes"] = c.income_passes;
	d["income_buildings"] = c.income_buildings;
	d["spending_wages"] = c.spending_wages;
	d["spending_goods"] = c.spending_goods;
	d["spending_buildings"] = c.spending_buildings;
	d["evictions"] = static_cast<int64_t>(c.evictions);
	d["households_evicted"] = static_cast<int64_t>(c.households_evicted);
	d["in_debt"] = static_cast<int64_t>(c.in_debt);
	d["bikes"] = static_cast<int64_t>(c.bikes);
	d["cars"] = static_cast<int64_t>(c.cars);
	d["passes"] = static_cast<int64_t>(c.passes);
	d["trips_walk"] = static_cast<int64_t>(c.trips_walk);
	d["trips_bus"] = static_cast<int64_t>(c.trips_bus);
	d["trips_bike"] = static_cast<int64_t>(c.trips_bike);
	d["trips_car"] = static_cast<int64_t>(c.trips_car);
	d["trips_coach"] = static_cast<int64_t>(c.trips_coach);
	d["homes_owned"] = static_cast<int64_t>(c.homes_owned);
	d["npc_owned"] = static_cast<int64_t>(c.npc_owned);
	d["city_owned"] = static_cast<int64_t>(c.city_owned);
	d["listed"] = static_cast<int64_t>(c.listed);
	d["buildings_sold"] = static_cast<int64_t>(c.buildings_sold);
	d["buildings_bought"] = static_cast<int64_t>(c.buildings_bought);
	return d;
}

Dictionary RoadEditor::sim_building_info(int64_t id) {
	Dictionary d;
	const Traffic &t = sim_.traffic();
	if (!t.network()) return d;
	const BuildingInfo b = t.building_info(static_cast<uint32_t>(id));
	if (!b.found) return d;
	d["units"] = b.units;
	d["households"] = b.households;
	d["residents"] = b.residents;
	d["inside"] = b.inside;
	d["employees"] = b.employees;
	d["headcount"] = b.headcount;
	d["staff_in"] = b.staff_in;
	d["customers"] = b.customers;
	d["booked_today"] = b.booked_today;
	d["unfilled_today"] = b.unfilled_today;
	d["in_hours"] = b.in_hours;
	d["open"] = b.open;
	d["closed_unexpectedly"] = b.closed_unexpectedly;
	d["opened_at"] = b.opened_at;
	d["late_minutes_today"] = b.late_minutes_today;
	d["unexpected_minutes_today"] = b.unexpected_minutes_today;
	d["served"] = static_cast<int64_t>(b.served);
	d["turned_away"] = static_cast<int64_t>(b.turned_away);
	d["late_openings"] = static_cast<int64_t>(b.late_openings);
	// M6
	d["owner"] = static_cast<int64_t>(b.owner);
	d["owner_kind"] = b.owner_household != 0 ? "household" : b.owner != 0 ? "npc" : "city";
	d["location"] = b.location;
	d["rent"] = b.rent;
	d["base_rent"] = b.base_rent;
	d["price"] = b.price;
	d["price_factor"] = b.price_factor;
	d["wage"] = b.wage;
	d["value"] = b.value;
	d["listed"] = b.listed;
	d["asking"] = b.asking;
	d["income_month"] = b.income_month;
	d["expense_month"] = b.expense_month;
	d["net_month"] = b.net_month;
	d["sales"] = static_cast<int64_t>(b.sales);
	return d;
}

Array RoadEditor::sim_building_states(int level) {
	Array out;
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	if (!net || !t.city_on()) return out;
	const std::vector<BuildingInfo> infos = t.building_infos();
	for (size_t k = 0; k < net->buildings.size() && k < infos.size(); ++k) {
		const NetBuilding &nb = net->buildings[k];
		if (nb.level != level || nb.type < 0) continue;
		const BuildingInfo &b = infos[k];
		Dictionary d;
		d["id"] = static_cast<int64_t>(nb.id);
		d["centre"] = gv(nb.centre);
		d["kind"] = building_kind_name(nb.kind);
		d["open"] = b.open;
		d["in_hours"] = b.in_hours;
		d["closed_unexpectedly"] = b.closed_unexpectedly;
		d["inside"] = b.inside;
		d["residents"] = b.residents;
		d["units"] = b.units;
		d["households"] = b.households;
		d["customers"] = b.customers;
		d["staff_in"] = b.staff_in;
		d["door"] = nb.entrance >= 0;
		out.push_back(d);
	}
	return out;
}

Dictionary RoadEditor::sim_resident_info(int64_t id) {
	Dictionary d;
	const Traffic &t = sim_.traffic();
	const ResidentInfo r = t.resident_info(static_cast<uint32_t>(id));
	if (!r.found) return d;
	auto name_of = [&](uint32_t b) -> String {
		if (b == 0) return String();
		if (b == kOutside) return "outside the map";
		const Building *bd = doc_.map().building(b);
		if (!bd) return String();
		const BuildingType *ty = default_city_data().type(bd->type);
		const String label = ty ? gstr(ty->label) : gstr(bd->type);
		return bd->name.empty() ? label + String(" ") + String::num_int64(b) : gstr(bd->name);
	};
	d["id"] = id;
	d["visitor"] = r.visitor;
	d["state"] = state_name(r.state);
	d["doing"] = doing_name(r.doing);
	d["activity"] = gstr(r.activity);
	d["at"] = static_cast<int64_t>(r.at);
	d["at_name"] = name_of(r.at);
	d["home"] = static_cast<int64_t>(r.home);
	d["home_name"] = name_of(r.home);
	d["employer"] = static_cast<int64_t>(r.employer == kOutside ? -1 : r.employer);
	d["employer_name"] = name_of(r.employer);
	d["going_name"] = name_of(r.going);
	d["hunger"] = r.hunger;
	d["energy"] = r.energy;
	d["money"] = r.money;
	d["pantry"] = r.pantry;
	d["household_size"] = r.household_size;
	const int64_t tpm = t.ticks_per_minute();
	const int64_t start_min = r.shift_start ? t.clock_at(r.shift_start) % 1440 : -1;
	const int64_t end_min = r.shift_end ? t.clock_at(r.shift_end) % 1440 : -1;
	(void)tpm;
	d["shift_start"] = start_min;
	d["shift_end"] = end_min;
	d["shift_name"] = name_of(r.shift_building);
	d["until"] = r.until;
	d["late"] = static_cast<int64_t>(r.late);
	// M6
	d["has_bike"] = r.has_bike;
	d["has_car"] = r.has_car;
	d["has_pass"] = r.has_pass;
	d["car_at_name"] = name_of(r.car_at);
	d["mode"] = trip_mode_name(r.mode);
	d["owns_home"] = r.owns_home;
	d["household_money"] = r.household_money;
	d["rent"] = r.rent;
	d["debt_months"] = r.debt_months;
	return d;
}

void RoadEditor::sim_set_city(const Dictionary &cfg) {
	TrafficConfig &c = sim_.traffic().config();
	c.city_prefill = std::clamp(static_cast<double>(cfg.get("prefill", c.city_prefill)), 0.0, 1.0);
	c.employment_share = std::clamp(static_cast<double>(cfg.get("employment_share", c.employment_share)), 0.0, 1.0);
}

void RoadEditor::bind_m5_methods() {
	ClassDB::bind_method(D_METHOD("new_city"), &RoadEditor::new_city);
	ClassDB::bind_method(D_METHOD("building_types"), &RoadEditor::building_types);
	ClassDB::bind_method(D_METHOD("snap_building", "type", "near", "level", "ignore"), &RoadEditor::snap_building, DEFVAL(0));
	ClassDB::bind_method(D_METHOD("move_building", "id", "pos", "dir"), &RoadEditor::move_building);
	ClassDB::bind_method(D_METHOD("add_building", "type", "pos", "dir", "level"), &RoadEditor::add_building);
	ClassDB::bind_method(D_METHOD("set_building_name", "id", "name"), &RoadEditor::set_building_name);
	ClassDB::bind_method(D_METHOD("remove_building", "id"), &RoadEditor::remove_building);
	ClassDB::bind_method(D_METHOD("get_buildings", "level"), &RoadEditor::get_buildings);
	ClassDB::bind_method(D_METHOD("pick_building", "pos", "level"), &RoadEditor::pick_building);
	ClassDB::bind_method(D_METHOD("get_building", "id"), &RoadEditor::get_building);
	ClassDB::bind_method(D_METHOD("sim_city_stats"), &RoadEditor::sim_city_stats);
	ClassDB::bind_method(D_METHOD("sim_building_info", "id"), &RoadEditor::sim_building_info);
	ClassDB::bind_method(D_METHOD("sim_building_states", "level"), &RoadEditor::sim_building_states);
	ClassDB::bind_method(D_METHOD("sim_resident_info", "id"), &RoadEditor::sim_resident_info);
	ClassDB::bind_method(D_METHOD("sim_set_city", "config"), &RoadEditor::sim_set_city);
}

} // namespace godot
