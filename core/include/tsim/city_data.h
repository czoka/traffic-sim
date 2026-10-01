// City data tables (M5): needs, offerings and building types, loaded from
// JSON (game/data/city_data.json, compiled in as the default) so new building
// types need no code.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tsim {

enum class BuildingKind : uint8_t {
	Home = 0,
	Shop = 1,
	Office = 2,
};
const char *building_kind_name(BuildingKind k);

// An activity at a building that restores needs for time and money.
struct Offering {
	std::string id, label;
	double hunger = 0.0; // restored (need points, 0..100)
	double energy = 0.0;
	double pantry = 0.0; // portions added to the household's pantry
	double pantry_use = 0.0; // portions taken
	int min_minutes = 15, max_minutes = 15;
	double price = 0.0; // credits
	std::string unlocks; // M6: "bike" or "car" (buying a vehicle)
	double cost = 0.0; // M6: what the shop pays its suppliers per sale (that money leaves the map)
};

struct ShiftSpec {
	int start = 0, end = 0; // minutes of the day
	int staff = 1;
	int minutes() const { return end - start; }
};

struct DayPlan {
	bool open = false;
	int open_at = 0, close_at = 0; // minutes of the day
	std::vector<ShiftSpec> shifts;
	bool in_hours(int minute) const { return open && minute >= open_at && minute < close_at; }
};

struct BuildingType {
	std::string id, label;
	BuildingKind kind = BuildingKind::Home;
	double width = 10.0, depth = 10.0; // m along the street, m back from it
	int households = 0; // homes
	double rent = 0.0, price = 0.0; // homes (used from M6)
	std::vector<std::string> offers; // offering ids (shops)
	int slots = 0; // customers at once (shops)
	int desks = 0; // offices
	double wage = 0.0; // credits per hour
	double revenue = 0.0; // offices: credits earned per staff hour (clients are outside the map)
	double value = 0.0; // what a buyer pays at least (0: from the type's prices)
	DayPlan weekday, weekend;
	int min_staff = 1;
	std::string parking = "none"; // none, street, own, garage
	uint32_t color = 0xb0b0b0;
	bool city = false; // the pre-placed city offices
	bool business() const { return kind != BuildingKind::Home; }
	// Staff a business needs on its roll, from its weekly shift hours.
	int headcount(int weekly_hours) const;
};

struct CityData {
	int start_day = 0; // 0 Monday
	int start_hour = 6;
	double hunger_empty_hours = 6.0, energy_empty_hours = 16.0;
	double asleep_hunger_factor = 0.3;
	double pantry_full = 14.0;
	double calm_above = 60.0, steep_below = 30.0;
	double price_weight = 0.02;
	double travel_weight = 1.0; // per minute
	double closed_penalty = 200.0;
	int closed_memory_days = 3;
	int replan_minutes = 15;
	int leave_margin_minutes = 10;
	int break_minutes = 30;
	double start_money = 10000.0;
	std::vector<double> household_sizes = { 0.35, 0.35, 0.2, 0.1 }; // 1..4 people
	int weekly_hours = 40;
	double weekend_dislike = 30.0;
	double money_low = 3000.0;
	int outside_commute_minutes = 30;
	double outside_wage = 20.0;
	int immigrants_per_coach = 6; // households, at most
	int shoppers_per_coach = 4;
	int visitor_early_minutes = 120;
	// M6: the economy.
	int month_days = 30; // rent is due on day 1 of each month
	double coach_fare = 12.0; // each way
	double car_cost_per_km = 0.25; // fuel and wear
	double outside_drive_km = 15.0; // a trip outside the map by car, beyond the edge
	double bus_pass = 40.0; // a month of city buses
	double day_pass_share = 0.1; // a visitor's day pass, of the monthly price
	double minutes_per_credit = 0.25; // what money is worth in travel time (mode choice)
	double bike_speed = 4.5; // m/s, for trip estimates
	double parking_minutes = 2.0; // getting in and out of the car park
	int eviction_months = 2; // in debt at that many rent days in a row
	double centre_factor = 1.0, edge_factor = 0.5, edge_distance = 3000.0; // location factor of prices
	double value_years = 10.0; // buyers pay this many years of net income
	double sale_rate = 0.1; // chance per day that a fairly priced listing sells
	double npc_list_chance = 0.05; // per month and NPC-owned building
	int listing_days = 60; // an NPC listing is taken down after this
	double price_step = 0.05, wage_step = 0.04, rent_step = 0.03; // NPC owners' monthly moves
	double price_min = 0.6, price_max = 2.0; // of the default prices
	double wage_min = 0.8, wage_max = 1.6;
	double rent_min = 0.5, rent_max = 2.0;
	std::vector<Offering> offerings;
	std::vector<BuildingType> types;

	const BuildingType *type(const std::string &id) const;
	int type_index(const std::string &id) const;
	const Offering *offering(const std::string &id) const;
	int offering_index(const std::string &id) const;
	// Urgency of a need at value v (0 desperate .. 100 satisfied): near zero
	// above calm_above, steep below steep_below.
	double urgency(double v) const;
};

bool parse_city_data(const std::string &json, CityData &out, std::string *error = nullptr);
const CityData &default_city_data();
const std::string &default_city_data_json();

} // namespace tsim
