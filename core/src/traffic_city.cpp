// Traffic (M5): city life.
//
// Residents live in households allocated to home units, take shifts at the
// businesses that employ them (or outside the map, by coach) and look after
// their needs (hunger, energy, the household pantry) with offerings from the
// city data: they score what is reachable now and pick the most relief for the
// least effort, but a booked shift is a fixed appointment. A resident inside a
// building is a cheap state update once per sim minute; only travellers are
// moving agents (pedestrians, bus and coach riders). Businesses open only
// within their hours with enough staff clocked in, so late staff open shops
// late. Immigrants come by coach when there are vacant homes they can reach
// from the main station; visitors come by coach to fill open shifts and to
// shop, and go home the same day.
//
// Like the rest of the tick: only + - * / and sqrt, the seeded RNG, and
// iteration in index order.
#include "tsim/traffic.h"

#include <algorithm>
#include <cmath>

namespace tsim {

const char *doing_name(Doing d) {
	switch (d) {
		case Doing::Idle:
			return "idle";
		case Doing::Sleep:
			return "sleeping";
		case Doing::Offering:
			return "busy";
		case Doing::Work:
			return "working";
	}
	return "idle";
}

namespace {

constexpr double kInf = 1e300;
constexpr float kNoWay = 1e9f;

} // namespace

// --- Clock and lookups -------------------------------------------------------------------

int64_t Traffic::ticks_per_minute() const {
	return std::max<int64_t>(1, static_cast<int64_t>(std::llround(60.0 / config_.dt)));
}

int64_t Traffic::clock_at(uint64_t t) const {
	const CityData &c = *city_data_;
	return static_cast<int64_t>(c.start_day) * 1440 + static_cast<int64_t>(c.start_hour) * 60 +
			static_cast<int64_t>(t) / ticks_per_minute();
}

int64_t Traffic::clock_minutes() const { return clock_at(tick_); }

uint64_t Traffic::tick_of(int d, int minute) const {
	const CityData &c = *city_data_;
	const int64_t m = static_cast<int64_t>(d) * 1440 + minute - (static_cast<int64_t>(c.start_day) * 1440 + c.start_hour * 60);
	return m <= 0 ? 0 : static_cast<uint64_t>(m * ticks_per_minute());
}

int32_t Traffic::find_resident(uint32_t id) const {
	auto it = std::lower_bound(res_.begin(), res_.end(), id, [](const Resident &r, uint32_t x) { return r.id < x; });
	return it != res_.end() && it->id == id ? static_cast<int32_t>(it - res_.begin()) : -1;
}

int32_t Traffic::building_place(uint32_t building_id) const {
	if (!net_ || building_id == 0 || building_id == kOutside) return -1;
	const int32_t bi = net_->building_index(building_id);
	return bi < 0 ? -1 : static_cast<int32_t>(net_->ped.spawners.size()) + bi;
}

Traffic::BState *Traffic::bstate(uint32_t building_id) {
	if (!net_) return nullptr;
	const int32_t bi = net_->building_index(building_id);
	return bi < 0 || static_cast<size_t>(bi) >= bstate_.size() ? nullptr : &bstate_[static_cast<size_t>(bi)];
}

const Traffic::BState *Traffic::bstate(uint32_t building_id) const {
	if (!net_) return nullptr;
	const int32_t bi = net_->building_index(building_id);
	return bi < 0 || static_cast<size_t>(bi) >= bstate_.size() ? nullptr : &bstate_[static_cast<size_t>(bi)];
}

double Traffic::travel_estimate(int32_t from, int32_t to) const {
	const size_t P = place_count();
	if (from < 0 || to < 0 || static_cast<size_t>(from) >= P || static_cast<size_t>(to) >= P || travel_.size() != P * P) {
		return kNoWay;
	}
	return travel_[static_cast<size_t>(from) * P + static_cast<size_t>(to)];
}

bool Traffic::coaches_run() const { return net_ && net_->main_station >= 0 && station_place_ >= 0 && !net_->coach_lines.empty(); }

double Traffic::minutes_between(int32_t from, uint32_t to) const {
	if (to == kOutside) {
		if (!coaches_run()) return kNoWay;
		double wait = 30.0;
		double per_hour = 0.0;
		for (const NetCoachLine &c : net_->coach_lines) per_hour += c.per_hour;
		if (per_hour > 0.0) wait = 30.0 / per_hour;
		const double t = travel_estimate(from, station_place_);
		return t >= kNoWay ? kNoWay : t / 60.0 + wait + city_data_->outside_commute_minutes;
	}
	const double t = travel_estimate(from, building_place(to));
	return t >= kNoWay ? kNoWay : t / 60.0;
}

// --- Network changes and reset -------------------------------------------------------------

void Traffic::city_reset_network() {
	const bool was_on = city_on_;
	city_on_ = net_ && !net_->buildings.empty();
	travel_.clear();
	bstate_.assign(net_ ? net_->buildings.size() : 0, BState{});
	if (!city_on_) {
		if (was_on) {
			for (Pedestrian &p : peds_) {
				if (p.resident != 0) p.done = true;
			}
			res_.clear();
			hh_.clear();
			visitor_jobs_.clear();
		}
		return;
	}
	if (ped_cost_.size() != place_count()) ped_costs();
	if (rides_.empty()) route_times();
	// Expected travel times between places: walking, or one bus ride.
	const size_t P = place_count();
	travel_.assign(P * P, kNoWay);
	for (size_t a = 0; a < P; ++a) {
		const std::vector<double> &ca = ped_cost_[a];
		for (size_t b = 0; b < P; ++b) {
			double best = kInf;
			for (int32_t e : place_entries_[b]) best = std::min(best, ca[static_cast<size_t>(e)]);
			if (a == b) best = 0.0;
			const std::vector<double> &cb = ped_cost_[b];
			for (const RideTimes &rt : rides_) {
				for (size_t i = 0; i + 1 < rt.stops.size(); ++i) {
					const int32_t pi = platform_of(rt.stops[i]);
					if (pi < 0 || ca[static_cast<size_t>(pi)] >= best) continue;
					for (size_t j = i + 1; j < rt.stops.size(); ++j) {
						const int32_t pj = platform_of(rt.stops[j]);
						if (pj < 0) continue;
						const double t = ca[static_cast<size_t>(pi)] + 0.5 * rt.headway + (rt.at[j] - rt.at[i]) + cb[static_cast<size_t>(pj)];
						best = std::min(best, t);
					}
				}
			}
			if (best < 1e8) travel_[a * P + b] = static_cast<float>(best);
		}
	}
	// Households whose home is gone leave the map; residents in buildings that
	// are gone go home; shifts at buildings that are gone are cancelled.
	const CityData &cd = *city_data_;
	auto is_home = [&](uint32_t id) {
		const int32_t bi = net_->building_index(id);
		return bi >= 0 && net_->buildings[static_cast<size_t>(bi)].kind == BuildingKind::Home &&
				net_->buildings[static_cast<size_t>(bi)].type >= 0;
	};
	for (Household &h : hh_) {
		if (h.home == 0) continue;
		const int32_t bi = net_->building_index(h.home);
		const bool ok = is_home(h.home) &&
				h.unit < cd.types[static_cast<size_t>(net_->buildings[static_cast<size_t>(bi)].type)].households;
		if (!ok) {
			h.home = 0;
			for (uint32_t m : h.members) {
				const int32_t ri = find_resident(m);
				if (ri < 0) continue;
				Resident &r = res_[static_cast<size_t>(ri)];
				if (r.ped) {
					const int32_t pi = find_pedestrian(r.ped);
					if (pi >= 0) peds_[static_cast<size_t>(pi)].done = true;
				}
				r.id = 0; // removed below
			}
			h.members.clear();
		}
	}
	for (Resident &r : res_) {
		if (r.id == 0) continue;
		auto gone = [&](uint32_t b) { return b != 0 && b != kOutside && net_->building_index(b) < 0; };
		if (gone(r.employer)) r.employer = 0;
		if (gone(r.shift.building)) r.shift = Booking{};
		if (gone(r.then)) r.then = 0;
		r.closed.erase(std::remove_if(r.closed.begin(), r.closed.end(), [&](const auto &c) { return gone(c.first); }),
				r.closed.end());
		const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
		if (r.state == ResidentState::Inside && gone(r.at)) {
			if (home == 0) {
				r.id = 0;
				continue;
			}
			r.at = home;
			r.doing = Doing::Idle;
			r.offering = -1;
			r.next_plan = tick_;
		}
		if (r.state == ResidentState::Travelling && find_pedestrian(r.ped) < 0) {
			const uint32_t to = gone(r.going_building) || r.going_building == 0 || r.going_building == kOutside ? home : r.going_building;
			if (to == 0) {
				r.id = 0;
				continue;
			}
			r.state = ResidentState::Inside;
			r.at = to;
			r.ped = 0;
			r.doing = Doing::Idle;
			r.offering = -1;
			r.next_plan = tick_;
		} else if (r.state == ResidentState::Travelling) {
			r.going = r.going_building != 0 && r.going_building != kOutside ? building_place(r.going_building) : station_place_;
		}
		if (r.shift.clocked && (r.state != ResidentState::Inside || r.at != r.shift.building)) r.shift.clocked = false;
	}
	res_.erase(std::remove_if(res_.begin(), res_.end(), [](const Resident &r) { return r.id == 0; }), res_.end());
	for (Household &h : hh_) {
		h.members.erase(std::remove_if(h.members.begin(), h.members.end(), [&](uint32_t m) { return find_resident(m) < 0; }),
				h.members.end());
	}
	city_recount();
}

void Traffic::city_recount() {
	const CityData &cd = *city_data_;
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		BState &b = bstate_[bi];
		const NetBuilding &nb = net_->buildings[bi];
		b.units.assign(nb.type >= 0 && nb.kind == BuildingKind::Home ? static_cast<size_t>(cd.types[static_cast<size_t>(nb.type)].households) : 0, -1);
		b.employees.clear();
		b.staff_in = b.customers = 0;
	}
	for (size_t h = 0; h < hh_.size(); ++h) {
		BState *b = hh_[h].home ? bstate(hh_[h].home) : nullptr;
		if (b && hh_[h].unit >= 0 && static_cast<size_t>(hh_[h].unit) < b->units.size()) b->units[static_cast<size_t>(hh_[h].unit)] = static_cast<int32_t>(h);
	}
	for (const Resident &r : res_) {
		if (r.employer != 0 && r.employer != kOutside && !r.visitor) {
			if (BState *b = bstate(r.employer)) b->employees.push_back(r.id);
		}
		if (r.state != ResidentState::Inside) continue;
		BState *b = bstate(r.at);
		if (!b) continue;
		if (r.doing == Doing::Work && r.shift.clocked) ++b->staff_in;
		if (r.doing == Doing::Offering && r.offering >= 0) {
			const int32_t bi = net_->building_index(r.at);
			if (net_->buildings[static_cast<size_t>(bi)].kind == BuildingKind::Shop) ++b->customers;
		}
	}
}

int32_t Traffic::new_resident(bool visitor) {
	Resident r;
	r.id = next_res_id_++;
	r.visitor = visitor;
	r.updated = tick_ + 1;
	r.next_plan = tick_ + 1;
	r.money = visitor ? 0.0 : city_data_->start_money;
	res_.push_back(r);
	return static_cast<int32_t>(res_.size() - 1);
}

int32_t Traffic::add_household(uint32_t home, int people, bool works) {
	BState *b = bstate(home);
	if (!b) return -1;
	int unit = -1;
	for (size_t u = 0; u < b->units.size(); ++u) {
		if (b->units[u] < 0) {
			unit = static_cast<int>(u);
			break;
		}
	}
	if (unit < 0) return -1;
	Household h;
	h.id = next_hh_id_++;
	h.home = home;
	h.unit = unit;
	h.pantry = std::floor(rng_.range(3.0, city_data_->pantry_full));
	const int32_t hi = static_cast<int32_t>(hh_.size());
	b->units[static_cast<size_t>(unit)] = hi;
	hh_.push_back(h);
	for (int k = 0; k < std::max(1, people); ++k) {
		const int32_t ri = new_resident(false);
		Resident &r = res_[static_cast<size_t>(ri)];
		r.household = hi;
		r.works = works;
		r.state = ResidentState::Inside;
		r.at = home;
		r.hunger = rng_.range(55.0, 85.0);
		r.energy = rng_.range(85.0, 100.0);
		r.next_plan = tick_ + 1 + static_cast<uint64_t>(rng_.next_u64() % static_cast<uint64_t>(ticks_per_minute() * 10));
		hh_[static_cast<size_t>(hi)].members.push_back(r.id);
	}
	return hi;
}

void Traffic::city_init() {
	res_.clear();
	hh_.clear();
	visitor_jobs_.clear();
	next_res_id_ = 1;
	next_hh_id_ = 1;
	city_acc_ = CityStats{};
	city_arrivals_.clear();
	city_boarded_.clear();
	city_last_day_ = ~0ull;
	if (!city_on_) return;
	for (BState &b : bstate_) b.served = b.turned_away = b.late_openings = 0;
	city_recount();
	const CityData &cd = *city_data_;
	if (config_.city_prefill > 0.0) {
		for (size_t bi = 0; bi < bstate_.size(); ++bi) {
			const NetBuilding &nb = net_->buildings[bi];
			if (nb.kind != BuildingKind::Home || nb.type < 0 || nb.entrance < 0) continue;
			const size_t units = bstate_[bi].units.size();
			for (size_t u = 0; u < units; ++u) {
				if (!(rng_.uniform() < config_.city_prefill)) continue;
				double x = rng_.uniform();
				int size = 1;
				for (size_t k = 0; k < cd.household_sizes.size(); ++k) {
					if (x < cd.household_sizes[k]) {
						size = static_cast<int>(k) + 1;
						break;
					}
					x -= cd.household_sizes[k];
					size = static_cast<int>(k) + 1;
				}
				const int32_t hi = add_household(nb.id, size, true);
				if (hi < 0) break;
				for (uint32_t m : hh_[static_cast<size_t>(hi)].members) {
					Resident &r = res_[static_cast<size_t>(find_resident(m))];
					r.works = rng_.uniform() < config_.employment_share;
				}
			}
		}
	}
	city_last_day_ = static_cast<uint64_t>(clock_at(tick_ + 1) / 1440);
	city_daily();
}

// --- The tick ------------------------------------------------------------------------------

void Traffic::city_tick() {
	if (!city_on_) return;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	for (size_t k = 0; k < city_boarded_.size(); ++k) city_board_coach(city_boarded_[k]);
	city_boarded_.clear();
	for (size_t k = 0; k < city_arrivals_.size(); ++k) city_arrive(city_arrivals_[k]);
	city_arrivals_.clear();
	if (now % static_cast<uint64_t>(tpm) == 0) {
		const uint64_t today = static_cast<uint64_t>(clock_at(now) / 1440);
		if (today != city_last_day_) {
			city_last_day_ = today;
			city_daily();
		}
		city_minute();
	}
	// A slice of the residents each tick, so each is seen once a sim minute.
	const size_t n = res_.size();
	if (n > 0) {
		const uint64_t k = now % static_cast<uint64_t>(tpm);
		const size_t a = static_cast<size_t>(static_cast<uint64_t>(n) * k / static_cast<uint64_t>(tpm));
		const size_t b = static_cast<size_t>(static_cast<uint64_t>(n) * (k + 1) / static_cast<uint64_t>(tpm));
		for (size_t i = a; i < b && i < res_.size(); ++i) city_update(i);
	}
	// Visitors who left on a coach are gone.
	bool any = false;
	for (const Resident &r : res_) any |= r.visitor && r.leaving && r.state == ResidentState::Outside;
	if (any) {
		res_.erase(std::remove_if(res_.begin(), res_.end(),
						   [](const Resident &r) { return r.visitor && r.leaving && r.state == ResidentState::Outside; }),
				res_.end());
	}
}

void Traffic::city_daily() {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	const int today = static_cast<int>(city_last_day_);
	const bool weekend = today % 7 >= 5;
	if (today % 7 == 0) {
		for (Resident &r : res_) r.week_minutes = 0;
	}
	for (BState &b : bstate_) {
		b.opened_at = -1;
		b.late_minutes = 0;
		b.unexpected_minutes = 0;
		b.booked_today = 0;
		b.unfilled_today = 0;
	}
	for (Resident &r : res_) {
		if (r.shift.building != 0 && (r.shift.done || r.shift.end <= now) && !r.shift.clocked) r.shift = Booking{};
		r.closed.erase(std::remove_if(r.closed.begin(), r.closed.end(), [&](const auto &c) { return c.second <= now; }),
				r.closed.end());
	}
	visitor_jobs_.clear();
	// Job search.
	for (size_t i = 0; i < res_.size(); ++i) {
		const Resident &r = res_[i];
		if (!r.visitor && r.works && r.employer == 0) city_choose_job(i);
	}
	// Today's shifts: employees with the fewest hours this week first;
	// weekend shifts go first to those short on money, last to the rest.
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0 || nb.kind == BuildingKind::Home) continue;
		const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
		const DayPlan &plan = weekend ? t.weekend : t.weekday;
		if (!plan.open) continue;
		BState &b = bstate_[bi];
		for (const ShiftSpec &sl : plan.shifts) {
			const uint64_t start = tick_of(today, sl.start);
			const uint64_t end = tick_of(today, sl.end);
			if (start <= now) continue;
			std::vector<std::pair<double, size_t>> cand;
			for (uint32_t id : b.employees) {
				const int32_t ri = find_resident(id);
				if (ri < 0) continue;
				const Resident &r = res_[static_cast<size_t>(ri)];
				if (r.shift.building != 0) continue;
				if (r.week_minutes + sl.minutes() > cd.weekly_hours * 60 + 120) continue;
				double score = r.week_minutes;
				if (weekend) score += r.money < cd.money_low ? -100000.0 : cd.weekend_dislike * 60.0;
				cand.push_back({ score, static_cast<size_t>(ri) });
			}
			std::sort(cand.begin(), cand.end(), [&](const auto &x, const auto &y) {
				return x.first != y.first ? x.first < y.first : res_[x.second].id < res_[y.second].id;
			});
			int taken = 0;
			for (const auto &c : cand) {
				if (taken >= sl.staff) break;
				Resident &r = res_[c.second];
				r.shift = Booking{};
				r.shift.building = nb.id;
				r.shift.start = start;
				r.shift.end = end;
				r.shift.weekend = weekend;
				r.week_minutes += sl.minutes();
				++taken;
			}
			b.booked_today += taken;
			const int left = sl.staff - taken;
			if (left > 0) {
				b.unfilled_today += left;
				city_acc_.unfilled_shifts += static_cast<uint64_t>(left);
				if (coaches_run()) visitor_jobs_.push_back(VisitorJob{ nb.id, start, end, left });
			}
		}
	}
	// Jobs outside the map: weekdays 08:00-16:00.
	if (!weekend) {
		for (Resident &r : res_) {
			if (r.employer != kOutside || r.shift.building != 0) continue;
			const uint64_t start = tick_of(today, 8 * 60);
			if (start <= now || r.week_minutes + 480 > cd.weekly_hours * 60 + 120) continue;
			r.shift = Booking{};
			r.shift.building = kOutside;
			r.shift.start = start;
			r.shift.end = tick_of(today, 16 * 60);
			r.week_minutes += 480;
		}
	}
	// People asleep get up in time for their shift.
	for (Resident &r : res_) {
		if (r.state != ResidentState::Inside || r.doing != Doing::Sleep || r.shift.building == 0) continue;
		const double t = minutes_between(building_place(r.at), r.shift.building);
		if (t >= kNoWay) continue;
		const double lead = t + cd.leave_margin_minutes + 30.0;
		const int64_t wake = static_cast<int64_t>(r.shift.start) - static_cast<int64_t>(lead * static_cast<double>(tpm));
		if (static_cast<int64_t>(r.until) > wake) r.until = static_cast<uint64_t>(std::max<int64_t>(static_cast<int64_t>(now), wake));
	}
}

void Traffic::city_choose_job(size_t i) {
	const CityData &cd = *city_data_;
	Resident &r = res_[i];
	if (r.household < 0) return;
	const int32_t hp = building_place(hh_[static_cast<size_t>(r.household)].home);
	double best = -kInf;
	uint32_t pick = 0;
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0 || nb.kind == BuildingKind::Home || nb.entrance < 0) continue;
		const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
		if (static_cast<int>(bstate_[bi].employees.size()) >= t.headcount(cd.weekly_hours)) continue;
		const double m = minutes_between(hp, nb.id);
		if (m >= kNoWay) continue;
		// People differ in what they look for: a spread of +/- 60 credits a day.
		const double score = t.wage * 8.0 - 4.0 * m + 60.0 * rng_.symmetric();
		if (score > best) {
			best = score;
			pick = nb.id;
		}
	}
	const double out = minutes_between(hp, kOutside);
	if (out < kNoWay) {
		const double score = cd.outside_wage * 8.0 - 4.0 * out - 10.0 + 60.0 * rng_.symmetric();
		if (score > best) {
			best = score;
			pick = kOutside;
		}
	}
	r.employer = pick;
	if (pick != 0 && pick != kOutside) bstate(pick)->employees.push_back(r.id);
}

void Traffic::city_minute() {
	const CityData &cd = *city_data_;
	const int64_t clock = clock_at(tick_ + 1);
	const int minute = static_cast<int>(clock % 1440);
	const bool weekend = (clock / 1440) % 7 >= 5;
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0 || nb.kind == BuildingKind::Home) continue;
		const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
		const DayPlan &plan = weekend ? t.weekend : t.weekday;
		BState &b = bstate_[bi];
		b.in_hours = plan.in_hours(minute);
		b.open = b.in_hours && b.staff_in >= t.min_staff;
		b.unexpected = b.in_hours && !b.open;
		if (b.open && b.opened_at < 0) {
			b.opened_at = minute;
			b.late_minutes = std::max(0, minute - plan.open_at);
			if (b.late_minutes > 5) {
				++b.late_openings;
				++city_acc_.late_openings;
			}
		}
		if (b.unexpected) ++b.unexpected_minutes;
	}
}

// --- One resident ------------------------------------------------------------------------------

void Traffic::city_update(size_t i) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	Resident &r = res_[i];
	const double dm = static_cast<double>(now - std::min(now, r.updated)) / static_cast<double>(tpm);
	r.updated = now;
	if (!r.visitor) {
		const bool asleep = r.state == ResidentState::Inside && r.doing == Doing::Sleep;
		r.hunger = std::max(0.0, r.hunger - dm * 100.0 / (cd.hunger_empty_hours * 60.0) * (asleep ? cd.asleep_hunger_factor : 1.0));
		if (!asleep) r.energy = std::max(0.0, r.energy - dm * 100.0 / (cd.energy_empty_hours * 60.0));
	}
	switch (r.state) {
		case ResidentState::Travelling:
			if (find_pedestrian(r.ped) < 0) {
				// The trip was lost (an edit took the path away): put them where they were going.
				const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
				const uint32_t to = r.going_building != 0 && r.going_building != kOutside && bstate(r.going_building) ? r.going_building : home;
				if (to == 0) {
					r.state = ResidentState::Outside;
					r.leaving = true;
					r.visitor = true;
					return;
				}
				r.state = ResidentState::Inside;
				r.at = to;
				r.ped = 0;
				r.doing = Doing::Idle;
				r.next_plan = now;
			}
			return;
		case ResidentState::Outside:
			if (!coaches_run() && now >= r.outside_until) {
				// No coaches any more: they find their own way back.
				const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
				if (home == 0) {
					r.leaving = true;
					r.visitor = true;
					return;
				}
				r.state = ResidentState::Inside;
				r.at = home;
				r.doing = Doing::Idle;
				r.next_plan = now;
			}
			return;
		case ResidentState::Inside:
			break;
	}
	// Hungry at work: the break comes early.
	if (r.doing == Doing::Work && !r.visitor && !r.shift.break_taken && r.hunger < 30.0 && r.until < r.shift.end) {
		r.until = now;
	}
	if (r.doing == Doing::Work || r.doing == Doing::Sleep || r.doing == Doing::Offering) {
		if (now >= r.until) city_finish(i);
		return;
	}
	if (now >= r.next_plan) city_plan(i);
}

void Traffic::city_leave_building(Resident &r) {
	if (r.state != ResidentState::Inside) return;
	if (r.doing == Doing::Offering && r.offering >= 0) {
		BState *b = bstate(r.at);
		const int32_t bi = net_->building_index(r.at);
		if (b && bi >= 0 && net_->buildings[static_cast<size_t>(bi)].kind == BuildingKind::Shop && b->customers > 0) --b->customers;
	}
	city_clock_out(r);
	r.doing = Doing::Idle;
}

void Traffic::city_clock_in(Resident &r, BState &b, uint64_t now) {
	const int64_t tpm = ticks_per_minute();
	if (!r.shift.clocked) {
		r.shift.clocked = true;
		++b.staff_in;
		if (now > r.shift.start + static_cast<uint64_t>(tpm) && !r.shift.break_taken) {
			++r.late;
			++city_acc_.late_shifts;
		}
	}
	r.doing = Doing::Work;
	const uint64_t len = r.shift.end - r.shift.start;
	const uint64_t mid = r.shift.start + len / 2;
	const bool breaks = len >= static_cast<uint64_t>(360 * tpm);
	r.until = breaks && !r.shift.break_taken && mid > now ? mid : r.shift.end;
}

void Traffic::city_clock_out(Resident &r) {
	if (!r.shift.clocked) return;
	r.shift.clocked = false;
	if (BState *b = bstate(r.shift.building)) {
		if (b->staff_in > 0) --b->staff_in;
	}
}

bool Traffic::city_trip(size_t i, int32_t to_place, uint32_t to_building, bool by_coach) {
	if (!net_) return false;
	Resident &r = res_[i];
	const int32_t from = r.state == ResidentState::Inside ? building_place(r.at) : station_place_;
	if (from < 0 || static_cast<size_t>(from) >= place_count() || place_entries_[static_cast<size_t>(from)].empty()) return false;
	const PedGraph &g = net_->ped;
	Pedestrian ped;
	ped.speed = rng_.range(1.2, 1.5);
	ped.resident = r.id;
	ped.origin = from;
	ped.spawn_tick = tick_ + 1;
	int32_t target = -1;
	if (by_coach) {
		if (!coaches_run()) return false;
		target = platform_of(net_->main_station);
		ped.coach = true;
		ped.board = net_->main_station;
		ped.dest = -1;
	} else {
		if (to_place < 0 || static_cast<size_t>(to_place) >= place_count()) return false;
		const std::vector<int32_t> &te = place_entries_[static_cast<size_t>(to_place)];
		if (te.empty()) return false;
		ped.dest = to_place;
		const std::vector<double> &cf = ped_cost_[static_cast<size_t>(from)];
		double walk = kInf;
		for (int32_t e : te) {
			if (cf[static_cast<size_t>(e)] < walk) {
				walk = cf[static_cast<size_t>(e)];
				target = e;
			}
		}
		BusChoice bc;
		const double bus = best_bus(static_cast<size_t>(from), static_cast<size_t>(to_place), bc);
		const double vw = 1.0 + config_.cost_variance * rng_.symmetric();
		const double vb = 1.0 + config_.cost_variance * rng_.symmetric();
		if (bus < kInf && (walk >= kInf || bus * vb < 2.0 * walk * vw)) {
			apply_bus_choice(ped, bc);
			target = platform_of(ped.board);
		}
		if (target < 0) return false;
	}
	int32_t entry = place_entries_[static_cast<size_t>(from)][0];
	double near = kInf;
	for (int32_t e : place_entries_[static_cast<size_t>(from)]) {
		const double d = (g.nodes[static_cast<size_t>(e)].pos - g.nodes[static_cast<size_t>(target)].pos).length();
		if (d < near) {
			near = d;
			entry = e;
		}
	}
	std::vector<int32_t> path;
	if (!find_walk(entry, target, path)) return false;
	ped.id = next_ped_id_++;
	ped.from = entry;
	ped.prev_from = entry;
	ped.target = target;
	ped.path = std::move(path);
	city_leave_building(r);
	r.state = ResidentState::Travelling;
	r.ped = ped.id;
	r.going = by_coach ? station_place_ : to_place;
	r.going_building = by_coach ? kOutside : to_building;
	r.doing = Doing::Idle;
	peds_.push_back(std::move(ped));
	return true;
}

bool Traffic::city_start_offering(size_t i, int off) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	Resident &r = res_[i];
	if (off < 0 || static_cast<size_t>(off) >= cd.offerings.size()) return false;
	const Offering &o = cd.offerings[static_cast<size_t>(off)];
	const int32_t bi = net_->building_index(r.at);
	if (bi < 0) return false;
	const NetBuilding &nb = net_->buildings[static_cast<size_t>(bi)];
	BState &b = bstate_[static_cast<size_t>(bi)];
	const bool at_home = r.household >= 0 && hh_[static_cast<size_t>(r.household)].home == r.at;
	const bool sleep = o.energy > 0.0 && o.hunger <= 0.0 && o.pantry <= 0.0;
	if (nb.kind == BuildingKind::Shop && nb.type >= 0) {
		const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
		if (std::find(t.offers.begin(), t.offers.end(), o.id) == t.offers.end()) return false;
		if (!b.open || b.customers >= t.slots || r.money < o.price) {
			++b.turned_away;
			++city_acc_.turned_away;
			// Closed unexpectedly: remembered for days. Full: for an hour.
			const uint64_t memory = b.unexpected ? static_cast<uint64_t>(cd.closed_memory_days * 1440 * tpm)
					: b.open ? static_cast<uint64_t>(60 * tpm) : 0;
			if (memory > 0) {
				r.closed.push_back({ r.at, now + memory });
				if (r.closed.size() > 6) r.closed.erase(r.closed.begin());
			}
			return false;
		}
		++b.customers;
		++b.served;
		r.money -= o.price;
	} else if (at_home) {
		if (o.pantry_use > 0.0) {
			Household &h = hh_[static_cast<size_t>(r.household)];
			if (h.pantry < o.pantry_use) return false;
			h.pantry -= o.pantry_use;
		}
	} else {
		return false;
	}
	const double dur = std::floor(rng_.range(static_cast<double>(o.min_minutes), static_cast<double>(o.max_minutes) + 0.999));
	r.until = now + static_cast<uint64_t>(dur * static_cast<double>(tpm));
	if (sleep && r.shift.building != 0 && !r.shift.done && r.shift.start > now) {
		// Get up in time for the shift.
		const double t = minutes_between(building_place(r.at), r.shift.building);
		if (t < kNoWay) {
			const int64_t wake = static_cast<int64_t>(r.shift.start) -
					static_cast<int64_t>((t + cd.leave_margin_minutes + 30.0) * static_cast<double>(tpm));
			r.until = static_cast<uint64_t>(std::max<int64_t>(static_cast<int64_t>(now + static_cast<uint64_t>(60 * tpm)),
					std::min<int64_t>(static_cast<int64_t>(r.until), wake)));
		}
	}
	// A meal fills you up as you eat it.
	r.hunger = std::min(100.0, r.hunger + o.hunger);
	r.doing = sleep ? Doing::Sleep : Doing::Offering;
	r.offering = off;
	return true;
}

void Traffic::city_finish(size_t i) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	Resident &r = res_[i];
	const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
	if (r.doing == Doing::Work) {
		if (now < r.shift.end && !r.shift.break_taken) {
			// Halfway: a break. Out for a quick meal if hungry and one is near.
			r.shift.break_taken = true;
			if (r.hunger < 60.0 && !r.visitor) {
				const int32_t here = building_place(r.at);
				double best = 0.0;
				uint32_t pick = 0;
				int pick_off = -1;
				for (size_t bi = 0; bi < bstate_.size(); ++bi) {
					const NetBuilding &nb = net_->buildings[bi];
					if (nb.type < 0 || nb.kind != BuildingKind::Shop || !bstate_[bi].open) continue;
					const double m = minutes_between(here, nb.id);
					if (m >= kNoWay) continue;
					for (const std::string &oid : cd.types[static_cast<size_t>(nb.type)].offers) {
						const int oi = cd.offering_index(oid);
						const Offering &o = cd.offerings[static_cast<size_t>(oi)];
						if (o.hunger <= 0.0 || 2.0 * m + o.min_minutes > cd.break_minutes || r.money < o.price) continue;
						const double score = cd.urgency(r.hunger) * o.hunger - m * cd.travel_weight - cd.price_weight * o.price;
						if (score > best) {
							best = score;
							pick = nb.id;
							pick_off = oi;
						}
					}
				}
				if (pick != 0 && pick != r.at) {
					const uint32_t work = r.at;
					r.offering = pick_off;
					r.then = work;
					if (city_trip(i, building_place(pick), pick, false)) return;
					r.offering = -1;
					r.then = 0;
				}
				// No food near: a packed lunch from the household pantry.
				const int meal = cd.offering_index("home_meal");
				if (meal >= 0 && r.household >= 0) {
					Household &h = hh_[static_cast<size_t>(r.household)];
					const Offering &o = cd.offerings[static_cast<size_t>(meal)];
					if (h.pantry >= o.pantry_use) {
						h.pantry -= o.pantry_use;
						r.hunger = std::min(100.0, r.hunger + o.hunger);
						++city_acc_.home_meals;
					}
				}
			}
			if (BState *b = bstate(r.at)) city_clock_in(r, *b, now);
			return;
		}
		// End of the shift: paid by the hour (double at weekends).
		const double hours = static_cast<double>(r.shift.end - r.shift.start) / static_cast<double>(tpm * 60);
		const int32_t bi = net_->building_index(r.shift.building);
		const double wage = bi >= 0 && net_->buildings[static_cast<size_t>(bi)].type >= 0
				? cd.types[static_cast<size_t>(net_->buildings[static_cast<size_t>(bi)].type)].wage
				: cd.outside_wage;
		r.money += wage * hours * (r.shift.weekend ? 2.0 : 1.0);
		city_clock_out(r);
		r.shift.done = true;
		++city_acc_.shifts;
		r.doing = Doing::Idle;
		city_plan(i);
		return;
	}
	if (r.doing == Doing::Sleep || r.doing == Doing::Offering) {
		if (r.offering >= 0) {
			const Offering &o = cd.offerings[static_cast<size_t>(r.offering)];
			if (r.doing == Doing::Sleep) {
				r.energy = 100.0;
				++city_acc_.sleeps;
			} else {
				r.energy = std::min(100.0, r.energy + o.energy);
			}
			if (o.pantry > 0.0 && r.household >= 0) {
				Household &h = hh_[static_cast<size_t>(r.household)];
				h.pantry = std::min(cd.pantry_full * 1.5, h.pantry + o.pantry);
				++city_acc_.groceries;
			}
			if (o.hunger > 0.0) {
				if (r.at == home) ++city_acc_.home_meals;
				else ++city_acc_.meals_out;
			}
		}
		city_leave_building(r);
		r.offering = -1;
	}
	r.doing = Doing::Idle;
	if (r.then != 0 && r.then != r.at) {
		const uint32_t next = r.then;
		r.then = 0;
		if (city_trip(i, building_place(next), next, false)) return;
	}
	r.then = 0;
	city_plan(i);
}

void Traffic::city_plan(size_t i) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	Resident &r = res_[i];
	r.doing = Doing::Idle;
	const int32_t here = building_place(r.at);
	auto idle = [&](double minutes) {
		r.next_plan = now + static_cast<uint64_t>(std::max(1.0, minutes) * static_cast<double>(tpm));
	};
	// Visitors: work the shift or do the shopping they came for, then go home.
	if (r.visitor) {
		if (r.shift.building != 0 && !r.shift.done && now < r.shift.end) {
			if (r.shift.building == r.at) {
				if (BState *b = bstate(r.at)) city_clock_in(r, *b, now);
				return;
			}
			if (city_trip(i, building_place(r.shift.building), r.shift.building, false)) return;
		}
		r.leaving = true;
		if (!city_trip(i, station_place_, kOutside, true)) {
			r.state = ResidentState::Outside; // no way out on foot: gone
		}
		return;
	}
	Household *hh = r.household >= 0 ? &hh_[static_cast<size_t>(r.household)] : nullptr;
	const uint32_t home = hh ? hh->home : 0;
	// 1. A booked shift is a fixed appointment.
	double budget = kInf; // minutes until it must leave for work
	uint32_t work = 0;
	Booking &sh = r.shift;
	if (sh.building != 0 && !sh.done && now < sh.end) {
		if (sh.building == r.at) {
			if (BState *b = bstate(r.at)) {
				city_clock_in(r, *b, now);
				return;
			}
		}
		const double tw = minutes_between(here, sh.building);
		if (tw >= kNoWay) {
			sh.done = true; // can't get there: the shift is lost
		} else {
			const double until_start = (static_cast<double>(sh.start) - static_cast<double>(now)) / static_cast<double>(tpm);
			const double leave_in = until_start - tw - cd.leave_margin_minutes;
			if (leave_in <= 0.0) {
				if (sh.building == kOutside) {
					r.outside_until = sh.end + static_cast<uint64_t>(cd.outside_commute_minutes * tpm);
					if (city_trip(i, station_place_, kOutside, true)) return;
				} else if (city_trip(i, building_place(sh.building), sh.building, false)) {
					return;
				}
				sh.done = true; // no way there right now
			} else {
				budget = leave_in;
				work = sh.building;
			}
		}
	}
	// 2. Score what is reachable now.
	const int minute = minute_of_day();
	const bool night = minute < 6 * 60 || minute >= 22 * 60;
	const bool at_home = home != 0 && r.at == home;
	const double price_k = cd.price_weight * (r.money < cd.money_low ? 3.0 : 1.0);
	double best = 0.0;
	int best_off = -1;
	uint32_t best_at = 0;
	uint32_t best_then = 0;
	bool best_outside = false;
	bool best_home = false;
	auto back_to_work = [&](uint32_t from_building) {
		if (work == 0) return 0.0;
		return minutes_between(building_place(from_building), work);
	};
	auto fits = [&](double travel, double dur, uint32_t at_building) {
		return budget >= kInf || travel + dur + back_to_work(at_building) <= budget;
	};
	const double to_home = at_home ? 0.0 : home ? minutes_between(here, home) : kNoWay;
	// With a shift coming, eat for the hours until the break.
	double hunger = r.hunger;
	if (budget < kInf && budget < 120.0) {
		hunger -= 4.0 * 100.0 / cd.hunger_empty_hours;
	}
	for (size_t oi = 0; oi < cd.offerings.size(); ++oi) {
		const Offering &o = cd.offerings[oi];
		const bool sleep = o.energy > 0.0 && o.hunger <= 0.0 && o.pantry <= 0.0;
		const bool home_only = sleep || o.pantry_use > 0.0;
		if (!home_only || home == 0 || to_home >= kNoWay) continue;
		if (o.pantry_use > 0.0 && (!hh || hh->pantry < o.pantry_use)) continue;
		double value;
		// Sleepy, more so at night; hungry people eat before they go to bed.
		if (sleep) value = 100.0 * cd.urgency(r.energy) - 120.0 + (night ? 100.0 : 0.0) - 50.0 * cd.urgency(r.hunger);
		else value = cd.urgency(hunger) * o.hunger - 4.0;
		const double score = value - to_home * cd.travel_weight;
		if (score > best && fits(to_home, sleep ? 240.0 : o.min_minutes, home)) {
			best = score;
			best_off = static_cast<int>(oi);
			best_at = home;
			best_then = 0;
			best_outside = false;
			best_home = false;
		}
	}
	// Shops, if open on arrival and not remembered as unexpectedly closed.
	const bool weekend = is_weekend();
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0 || nb.kind != BuildingKind::Shop || nb.entrance < 0) continue;
		const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
		const double m = nb.id == r.at ? 0.0 : minutes_between(here, nb.id);
		if (m >= kNoWay) continue;
		const DayPlan &plan = weekend ? t.weekend : t.weekday;
		const int arrive = minute + static_cast<int>(m);
		if (!plan.in_hours(arrive % 1440)) continue;
		double closed_penalty = 0.0;
		for (const auto &c : r.closed) {
			if (c.first == nb.id && c.second > now) closed_penalty = cd.closed_penalty;
		}
		for (const std::string &oid : t.offers) {
			const int oi = cd.offering_index(oid);
			const Offering &o = cd.offerings[static_cast<size_t>(oi)];
			if (r.money < o.price) continue;
			double value = -kInf;
			double travel = m;
			uint32_t then = 0;
			if (o.hunger > 0.0) value = cd.urgency(hunger) * o.hunger - 4.0;
			if (o.pantry > 0.0 && hh) {
				const double need = 100.0 * hh->pantry / std::max(1.0, cd.pantry_full);
				value = std::max(value, cd.urgency(need) * 100.0 - 10.0);
				// Trip chaining: on the way home, only the detour counts.
				if (!at_home && home != 0) {
					const double onward = minutes_between(building_place(nb.id), home);
					if (onward < kNoWay && to_home < kNoWay) {
						travel = std::max(0.0, m + onward - to_home);
						then = home;
					}
				}
			}
			if (value <= -kInf) continue;
			const double score = value - travel * cd.travel_weight - price_k * o.price - closed_penalty;
			if (score > best && fits(m, o.min_minutes, nb.id)) {
				best = score;
				best_off = oi;
				best_at = nb.id;
				best_then = then;
				best_outside = false;
				best_home = false;
			}
		}
	}
	// Shops outside the map (by coach), for food and groceries.
	if (budget >= kInf && coaches_run()) {
		const double out = minutes_between(here, kOutside);
		if (out < kNoWay) {
			for (size_t oi = 0; oi < cd.offerings.size(); ++oi) {
				const Offering &o = cd.offerings[oi];
				if (o.pantry_use > 0.0 || (o.energy > 0.0 && o.hunger <= 0.0 && o.pantry <= 0.0)) continue;
				if (r.money < o.price) continue;
				double value = -kInf;
				if (o.hunger > 0.0) value = cd.urgency(r.hunger) * o.hunger - 10.0;
				if (o.pantry > 0.0 && hh) value = std::max(value, cd.urgency(100.0 * hh->pantry / std::max(1.0, cd.pantry_full)) * 100.0 - 10.0);
				if (value <= -kInf) continue;
				const double score = value - 2.0 * out * cd.travel_weight - price_k * o.price;
				if (score > best) {
					best = score;
					best_off = static_cast<int>(oi);
					best_at = kOutside;
					best_outside = true;
					best_home = false;
				}
			}
		}
	}
	// Going home when there is nothing else to do.
	if (!at_home && home != 0 && to_home < kNoWay && budget >= kInf) {
		const double score = 5.0 + (night ? 20.0 : 0.0);
		if (score > best) {
			best = score;
			best_off = -1;
			best_at = home;
			best_home = true;
			best_outside = false;
		}
	}
	if (best <= 0.0 || (best_off < 0 && !best_home)) {
		idle(std::min<double>(cd.replan_minutes, budget < kInf ? std::max(1.0, budget) : cd.replan_minutes));
		return;
	}
	if (best_outside) {
		const Offering &o = cd.offerings[static_cast<size_t>(best_off)];
		r.offering = best_off;
		r.outside_until = now + static_cast<uint64_t>((2.0 * cd.outside_commute_minutes + o.max_minutes) * static_cast<double>(tpm));
		if (city_trip(i, station_place_, kOutside, true)) return;
		r.offering = -1;
		idle(cd.replan_minutes);
		return;
	}
	if (best_at == r.at) {
		if (best_off >= 0 && city_start_offering(i, best_off)) return;
		idle(cd.replan_minutes);
		return;
	}
	r.offering = best_off;
	r.then = best_then;
	if (city_trip(i, building_place(best_at), best_at, false)) return;
	r.offering = -1;
	r.then = 0;
	idle(cd.replan_minutes);
}

// --- Arrivals and coaches ------------------------------------------------------------------------

void Traffic::city_arrive(uint32_t rid) {
	const int32_t ri = find_resident(rid);
	if (ri < 0) return;
	const size_t i = static_cast<size_t>(ri);
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	Resident &r = res_[i];
	if (r.state != ResidentState::Travelling) return;
	const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
	uint32_t to = r.going_building;
	if (to == 0 || to == kOutside || !bstate(to)) to = home;
	r.ped = 0;
	r.going = -1;
	if (to == 0) {
		r.state = ResidentState::Outside;
		r.leaving = true;
		r.visitor = true;
		return;
	}
	r.state = ResidentState::Inside;
	r.at = to;
	r.doing = Doing::Idle;
	r.updated = std::min(r.updated, now);
	if (r.shift.building == r.at && !r.shift.done && now < r.shift.end) {
		if (now + static_cast<uint64_t>(90 * tpm) >= r.shift.start) {
			city_clock_in(r, *bstate(r.at), now);
			return;
		}
	}
	if (r.offering >= 0) {
		const int off = r.offering;
		r.offering = -1;
		if (city_start_offering(i, off)) return;
		r.then = 0;
	}
	city_plan(i);
}

void Traffic::city_board_coach(uint32_t rid) {
	const int32_t ri = find_resident(rid);
	if (ri < 0) return;
	Resident &r = res_[static_cast<size_t>(ri)];
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	r.state = ResidentState::Outside;
	r.ped = 0;
	r.going = -1;
	if (r.visitor) {
		r.leaving = true;
		++city_acc_.visitor_trips;
		return;
	}
	r.outside_until = std::max(r.outside_until, now + static_cast<uint64_t>(2 * city_data_->outside_commute_minutes * tpm));
}

int Traffic::city_coach_arrives(int seats) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpm = ticks_per_minute();
	int brought = 0;
	// Residents coming back.
	std::vector<size_t> back;
	for (size_t i = 0; i < res_.size(); ++i) {
		const Resident &r = res_[i];
		if (!r.visitor && r.state == ResidentState::Outside && r.outside_until <= now) back.push_back(i);
	}
	std::sort(back.begin(), back.end(), [&](size_t a, size_t b) {
		return res_[a].outside_until != res_[b].outside_until ? res_[a].outside_until < res_[b].outside_until : res_[a].id < res_[b].id;
	});
	for (size_t i : back) {
		if (seats <= 0) break;
		Resident &r = res_[i];
		const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
		// What they went out for is done: a shift (paid), or a meal or shopping.
		if (r.shift.building == kOutside && !r.shift.done && now >= r.shift.end) {
			const double hours = static_cast<double>(r.shift.end - r.shift.start) / static_cast<double>(tpm * 60);
			r.money += cd.outside_wage * hours;
			r.shift.done = true;
			++city_acc_.shifts;
		}
		if (r.offering >= 0) {
			const Offering &o = cd.offerings[static_cast<size_t>(r.offering)];
			r.money -= o.price;
			r.hunger = std::min(100.0, r.hunger + o.hunger);
			if (o.pantry > 0.0 && r.household >= 0) {
				hh_[static_cast<size_t>(r.household)].pantry += o.pantry;
				++city_acc_.groceries;
			}
			if (o.hunger > 0.0) ++city_acc_.meals_out;
			r.offering = -1;
		}
		if (home == 0 || !city_trip(i, building_place(home), home, false)) continue;
		++brought;
		--seats;
	}
	// Visitors for the shifts nobody in the city took.
	for (VisitorJob &vj : visitor_jobs_) {
		if (vj.count <= 0 || now + static_cast<uint64_t>(cd.visitor_early_minutes * tpm) < vj.start ||
				now + static_cast<uint64_t>(60 * tpm) > vj.end) {
			continue;
		}
		while (vj.count > 0 && seats > 0) {
			const int32_t vi = new_resident(true);
			Resident &v = res_[static_cast<size_t>(vi)];
			v.state = ResidentState::Outside;
			v.shift.building = vj.building;
			v.shift.start = vj.start;
			v.shift.end = vj.end;
			v.shift.weekend = is_weekend();
			--vj.count;
			if (!city_trip(static_cast<size_t>(vi), building_place(vj.building), vj.building, false)) {
				res_.pop_back();
				continue;
			}
			++brought;
			--seats;
		}
	}
	// Immigrants, for vacant homes they can reach (cheap ones draw the most).
	if (seats > 0) {
		std::vector<std::pair<uint32_t, double>> vacant; // (home, weight per vacant unit)
		int units = 0;
		for (size_t bi = 0; bi < bstate_.size(); ++bi) {
			const NetBuilding &nb = net_->buildings[bi];
			if (nb.type < 0 || nb.kind != BuildingKind::Home || nb.entrance < 0) continue;
			if (travel_estimate(station_place_, building_place(nb.id)) >= kNoWay) continue;
			int free = 0;
			for (int32_t u : bstate_[bi].units) free += u < 0 ? 1 : 0;
			if (free == 0) continue;
			const double rent = std::max(100.0, cd.types[static_cast<size_t>(nb.type)].rent);
			vacant.push_back({ nb.id, free * (1000.0 / rent) });
			units += free;
		}
		const int households = std::min(cd.immigrants_per_coach, (units + 3) / 4);
		for (int k = 0; k < households && seats > 0 && !vacant.empty(); ++k) {
			double total = 0.0;
			for (const auto &v : vacant) total += v.second;
			double x = rng_.uniform() * total;
			size_t pick = vacant.size() - 1;
			for (size_t q = 0; q < vacant.size(); ++q) {
				if (x < vacant[q].second) {
					pick = q;
					break;
				}
				x -= vacant[q].second;
			}
			double y = rng_.uniform();
			int size = 1;
			for (size_t q = 0; q < cd.household_sizes.size(); ++q) {
				size = static_cast<int>(q) + 1;
				if (y < cd.household_sizes[q]) break;
				y -= cd.household_sizes[q];
			}
			if (size > seats) break;
			const uint32_t home = vacant[pick].first;
			const int32_t hi = add_household(home, size, true);
			if (hi < 0) {
				vacant.erase(vacant.begin() + static_cast<std::ptrdiff_t>(pick));
				continue;
			}
			for (uint32_t m : hh_[static_cast<size_t>(hi)].members) {
				const size_t ri = static_cast<size_t>(find_resident(m));
				res_[ri].works = rng_.uniform() < config_.employment_share;
				res_[ri].state = ResidentState::Outside;
				if (city_trip(ri, building_place(home), home, false)) {
					++brought;
					--seats;
				} else {
					res_[ri].state = ResidentState::Inside;
					res_[ri].at = home;
				}
				++city_acc_.immigrants;
				if (res_[ri].works) city_choose_job(ri);
			}
			// A whole home taken: fewer vacancies there.
			BState *b = bstate(home);
			int free = 0;
			for (int32_t u : b->units) free += u < 0 ? 1 : 0;
			if (free == 0) vacant.erase(vacant.begin() + static_cast<std::ptrdiff_t>(pick));
			else vacant[pick].second -= vacant[pick].second / (free + 1);
		}
	}
	// Shoppers, for shops that are open.
	for (int k = 0; k < cd.shoppers_per_coach && seats > 0; ++k) {
		double total = 0.0;
		for (size_t bi = 0; bi < bstate_.size(); ++bi) {
			const NetBuilding &nb = net_->buildings[bi];
			if (nb.type < 0 || nb.kind != BuildingKind::Shop || !bstate_[bi].open) continue;
			total += cd.types[static_cast<size_t>(nb.type)].slots;
		}
		if (total <= 0.0) break;
		double x = rng_.uniform() * total;
		uint32_t pick = 0;
		int off = -1;
		for (size_t bi = 0; bi < bstate_.size(); ++bi) {
			const NetBuilding &nb = net_->buildings[bi];
			if (nb.type < 0 || nb.kind != BuildingKind::Shop || !bstate_[bi].open) continue;
			const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
			if (x < t.slots || pick == 0) {
				pick = nb.id;
				off = t.offers.empty() ? -1 : cd.offering_index(t.offers[0]);
			}
			if (x < t.slots) break;
			x -= t.slots;
		}
		if (pick == 0 || off < 0) break;
		const int32_t vi = new_resident(true);
		Resident &v = res_[static_cast<size_t>(vi)];
		v.state = ResidentState::Outside;
		v.money = cd.offerings[static_cast<size_t>(off)].price;
		v.offering = off;
		if (!city_trip(static_cast<size_t>(vi), building_place(pick), pick, false)) {
			res_.pop_back();
			continue;
		}
		++brought;
		--seats;
	}
	return brought;
}

// --- Queries -------------------------------------------------------------------------------------

ResidentInfo Traffic::resident_info(uint32_t id) const {
	ResidentInfo out;
	const int32_t ri = find_resident(id);
	if (ri < 0) return out;
	const Resident &r = res_[static_cast<size_t>(ri)];
	const CityData &cd = *city_data_;
	out.found = true;
	out.id = r.id;
	out.visitor = r.visitor;
	out.state = r.state;
	out.doing = r.doing;
	out.at = r.state == ResidentState::Inside ? r.at : 0;
	out.home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
	out.employer = r.employer;
	out.going = r.state == ResidentState::Travelling ? r.going_building : 0;
	out.hunger = r.hunger;
	out.energy = r.energy;
	out.money = r.money;
	out.pantry = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].pantry : 0.0;
	out.household_size = r.household >= 0 ? static_cast<int>(hh_[static_cast<size_t>(r.household)].members.size()) : 0;
	if (r.shift.building != 0 && !r.shift.done) {
		out.shift_start = r.shift.start;
		out.shift_end = r.shift.end;
		out.shift_building = r.shift.building;
	}
	out.late = r.late;
	out.until = r.until > tick_ ? static_cast<double>(r.until - tick_) * config_.dt : 0.0;
	if (r.state == ResidentState::Outside) out.activity = r.visitor ? "gone home" : "outside the map";
	else if (r.doing == Doing::Work) out.activity = "working";
	else if (r.doing == Doing::Sleep) out.activity = "sleeping";
	else if (r.offering >= 0 && static_cast<size_t>(r.offering) < cd.offerings.size()) {
		out.activity = (r.state == ResidentState::Travelling ? "going for: " : "") + cd.offerings[static_cast<size_t>(r.offering)].label;
	} else if (r.state == ResidentState::Travelling) {
		out.activity = r.going_building == kOutside ? "to the main station" : r.shift.building == r.going_building ? "going to work" : "on the way";
	} else {
		out.activity = "idle";
	}
	return out;
}

BuildingInfo Traffic::building_info(uint32_t id) const {
	BuildingInfo out;
	if (!net_) return out;
	const int32_t bi = net_->building_index(id);
	if (bi < 0 || static_cast<size_t>(bi) >= bstate_.size()) return out;
	const NetBuilding &nb = net_->buildings[static_cast<size_t>(bi)];
	const BState &b = bstate_[static_cast<size_t>(bi)];
	const CityData &cd = *city_data_;
	out.found = true;
	out.id = id;
	out.type = nb.type;
	out.units = static_cast<int>(b.units.size());
	for (int32_t u : b.units) {
		if (u < 0) continue;
		++out.households;
		out.residents += static_cast<int>(hh_[static_cast<size_t>(u)].members.size());
	}
	for (const Resident &r : res_) out.inside += r.state == ResidentState::Inside && r.at == id ? 1 : 0;
	out.employees = static_cast<int>(b.employees.size());
	out.headcount = nb.type >= 0 ? cd.types[static_cast<size_t>(nb.type)].headcount(cd.weekly_hours) : 0;
	out.staff_in = b.staff_in;
	out.customers = b.customers;
	out.booked_today = b.booked_today;
	out.unfilled_today = b.unfilled_today;
	out.in_hours = b.in_hours;
	out.open = b.open;
	out.closed_unexpectedly = b.unexpected;
	out.opened_at = b.opened_at;
	out.late_minutes_today = b.late_minutes;
	out.unexpected_minutes_today = b.unexpected_minutes;
	out.served = b.served;
	out.turned_away = b.turned_away;
	out.late_openings = b.late_openings;
	return out;
}

CityStats Traffic::city_stats() const {
	CityStats st = city_acc_;
	st.on = city_on_;
	const int64_t m = clock_minutes();
	st.day = static_cast<int>(m / 1440);
	st.weekday = st.day % 7;
	st.minute = static_cast<int>(m % 1440);
	if (!city_on_) return st;
	double hunger = 0.0, energy = 0.0, money = 0.0;
	st.min_hunger = 100.0;
	for (const Resident &r : res_) {
		if (r.visitor) {
			++st.visitors;
			continue;
		}
		++st.residents;
		hunger += r.hunger;
		energy += r.energy;
		money += r.money;
		st.min_hunger = std::min(st.min_hunger, r.hunger);
		st.starving += r.hunger <= 0.0 ? 1 : 0;
		switch (r.state) {
			case ResidentState::Inside:
				++st.inside;
				st.sleeping += r.doing == Doing::Sleep ? 1 : 0;
				st.working += r.doing == Doing::Work ? 1 : 0;
				break;
			case ResidentState::Travelling:
				++st.travelling;
				break;
			case ResidentState::Outside:
				++st.outside;
				break;
		}
		if (r.employer == kOutside) ++st.employed_outside;
		else if (r.employer != 0) ++st.employed;
		else if (r.works) ++st.unemployed;
	}
	if (st.residents > 0) {
		st.mean_hunger = hunger / st.residents;
		st.mean_energy = energy / st.residents;
		st.mean_money = money / st.residents;
	} else {
		st.min_hunger = 0.0;
	}
	for (const Household &h : hh_) st.households += h.home != 0 ? 1 : 0;
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0) continue;
		if (nb.kind == BuildingKind::Home) {
			++st.homes;
			for (int32_t u : bstate_[bi].units) {
				++st.units;
				st.vacant_units += u < 0 ? 1 : 0;
			}
		} else {
			++st.businesses;
			st.open += bstate_[bi].open ? 1 : 0;
			st.closed_unexpectedly += bstate_[bi].unexpected ? 1 : 0;
		}
	}
	return st;
}

} // namespace tsim
