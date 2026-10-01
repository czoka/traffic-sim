// Traffic (M6): the economy.
//
// Money moves between residents, the owners of buildings and the outside:
// wages come from the employer's owner, prices and rent go to the owner of the
// shop or home, offices earn per staff hour, coach fares and fuel leave the
// map. Every building is owned by the city (the player, whose money is
// unlimited and only counted) or by an NPC owner, who tunes its prices, wages
// and rent each month. Rent is due on the 1st; a household in debt on that
// many rent days in a row is evicted and leaves by coach. Buildings change
// hands on the market: the player lists city buildings in the map, NPC owners
// list theirs, and buyers pay about ten years of net income. Rented homes sell
// with their tenants; a household that saves enough buys the house it lives in.
//
// Like the rest of the tick: only + - * / and sqrt, the seeded RNG, and
// iteration in index order (std::map: ascending id).
#include "tsim/traffic.h"

#include <algorithm>
#include <cmath>

namespace tsim {

const char *trip_mode_name(TripMode m) {
	switch (m) {
		case TripMode::Walk:
			return "walking";
		case TripMode::Bus:
			return "by bus";
		case TripMode::Bike:
			return "by bike";
		case TripMode::Car:
			return "by car";
		case TripMode::Coach:
			return "by coach";
	}
	return "walking";
}

namespace {

double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

} // namespace

// --- Prices and owners -----------------------------------------------------------------------

Traffic::BEcon &Traffic::econ(uint32_t building_id) { return econ_[building_id]; }

void Traffic::econ_reset_network() {
	const CityData &cd = *city_data_;
	// The centre: the marker, or the centroid of the shops and offices.
	Vec2 centre{ 0.0, 0.0 };
	if (net_->has_city_centre) {
		centre = net_->city_centre;
	} else {
		double n = 0.0;
		for (const NetBuilding &b : net_->buildings) {
			if (b.type < 0 || b.kind == BuildingKind::Home) continue;
			centre = centre + b.centre;
			n += 1.0;
		}
		if (n == 0.0) {
			for (const NetBuilding &b : net_->buildings) {
				centre = centre + b.centre;
				n += 1.0;
			}
		}
		if (n > 0.0) centre = centre * (1.0 / n);
	}
	for (size_t bi = 0; bi < net_->buildings.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		BState &b = bstate_[bi];
		const double d = (nb.centre - centre).length();
		b.location = cd.centre_factor - (cd.centre_factor - cd.edge_factor) * clampd(d / cd.edge_distance, 0.0, 1.0);
		if (nb.type >= 0) {
			const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
			b.base_rent = t.rent * b.location;
			b.base_price = t.price * b.location;
		}
		BEcon &e = econ_[nb.id];
		if (!nb.for_sale) e.player_sold = false;
		if (e.owner != 0 && nb.type >= 0) {
			const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
			if (e.wage <= 0.0) e.wage = t.wage;
			if (e.rent <= 0.0) e.rent = b.base_rent;
		}
	}
	// Buildings that are gone take their records with them.
	for (auto it = econ_.begin(); it != econ_.end();) {
		if (net_->building_index(it->first) < 0) it = econ_.erase(it);
		else ++it;
	}
}

void Traffic::econ_init() {
	econ_.clear();
	npc_cash_.clear();
	next_owner_id_ = 1;
	econ_last_day_ = ~0ull;
	city_vehicle_done_.clear();
	if (city_on_) econ_reset_network();
}

double Traffic::rent_of(size_t bi) const {
	const NetBuilding &nb = net_->buildings[bi];
	auto it = econ_.find(nb.id);
	if (it != econ_.end() && it->second.owner != 0) return it->second.rent > 0.0 ? it->second.rent : bstate_[bi].base_rent;
	return nb.rent > 0.0 ? nb.rent : bstate_[bi].base_rent;
}

double Traffic::wage_of(size_t bi) const {
	const NetBuilding &nb = net_->buildings[bi];
	const double base = nb.type >= 0 ? city_data_->types[static_cast<size_t>(nb.type)].wage : 0.0;
	auto it = econ_.find(nb.id);
	if (it != econ_.end() && it->second.owner != 0) return it->second.wage > 0.0 ? it->second.wage : base;
	return nb.wage > 0.0 ? nb.wage : base;
}

double Traffic::price_factor_of(size_t bi) const {
	const NetBuilding &nb = net_->buildings[bi];
	auto it = econ_.find(nb.id);
	if (it != econ_.end() && it->second.owner != 0) return it->second.price_factor;
	return nb.price_factor;
}

double Traffic::offering_price(size_t bi, int offering) const {
	if (offering < 0) return 0.0;
	return city_data_->offerings[static_cast<size_t>(offering)].price * price_factor_of(bi);
}

double Traffic::value_of(size_t bi) const {
	const CityData &cd = *city_data_;
	const NetBuilding &nb = net_->buildings[bi];
	if (nb.type < 0) return 0.0;
	const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
	const BState &b = bstate_[bi];
	double floor;
	double monthly;
	if (t.kind == BuildingKind::Home) {
		floor = b.base_price * t.households;
		monthly = rent_of(bi) * t.households;
	} else {
		floor = (t.value > 0.0 ? t.value : 2000.0 * (t.slots + t.desks)) * b.location;
		auto it = econ_.find(nb.id);
		monthly = it != econ_.end() ? it->second.net : 0.0;
	}
	return std::max(floor, cd.value_years * 12.0 * monthly);
}

double Traffic::building_value(uint32_t building_id) const {
	if (!net_) return 0.0;
	const int32_t bi = net_->building_index(building_id);
	return bi < 0 || static_cast<size_t>(bi) >= bstate_.size() ? 0.0 : value_of(static_cast<size_t>(bi));
}

void Traffic::econ_credit(uint32_t building_id, double amount, int kind) {
	if (amount == 0.0) return;
	BEcon &e = econ_[building_id];
	e.income += amount;
	if (e.owner == 0) {
		city_acc_.treasury_income += amount;
		city_acc_.month_income += amount;
		if (kind == 1) city_acc_.income_rent += amount;
		else city_acc_.income_sales += amount;
	} else {
		npc_cash_[e.owner] += amount;
	}
	if (kind == 1) city_acc_.rent_paid += amount;
	else city_acc_.sales_total += amount;
}

void Traffic::econ_debit(uint32_t building_id, double amount) {
	if (amount == 0.0) return;
	BEcon &e = econ_[building_id];
	e.expense += amount;
	city_acc_.wages_paid += amount;
	if (e.owner == 0) {
		city_acc_.treasury_spending += amount;
		city_acc_.month_spending += amount;
		city_acc_.spending_wages += amount;
	} else {
		npc_cash_[e.owner] -= amount;
	}
}

void Traffic::econ_sell(size_t bi, double price) {
	const NetBuilding &nb = net_->buildings[bi];
	BEcon &e = econ_[nb.id];
	// The seller gets the money.
	if (e.owner == 0) {
		e.player_sold = true;
		city_acc_.treasury_income += price;
		city_acc_.month_income += price;
		city_acc_.income_buildings += price;
	} else {
		npc_cash_[e.owner] += price;
	}
	const uint32_t buyer = next_owner_id_++;
	npc_cash_[buyer] -= price; // buyers bring their own money
	// The new owner starts from the numbers it bought.
	e.price_factor = price_factor_of(bi);
	e.wage = wage_of(bi);
	e.rent = rent_of(bi);
	e.owner = buyer;
	e.listed = false;
	e.asking = 0.0;
	++e.sales;
	++city_acc_.buildings_sold;
}

bool Traffic::buy_building(uint32_t building_id) {
	if (!net_ || !city_on_) return false;
	const int32_t bi = net_->building_index(building_id);
	if (bi < 0) return false;
	auto it = econ_.find(building_id);
	if (it == econ_.end() || it->second.owner == 0 || !it->second.listed) return false;
	BEcon &e = it->second;
	const double price = e.asking > 0.0 ? e.asking : value_of(static_cast<size_t>(bi));
	npc_cash_[e.owner] += price;
	city_acc_.treasury_spending += price;
	city_acc_.month_spending += price;
	city_acc_.spending_buildings += price;
	e.owner = 0;
	e.listed = false;
	e.asking = 0.0;
	++e.sales;
	++city_acc_.buildings_bought;
	return true;
}

std::vector<Listing> Traffic::market() const {
	std::vector<Listing> out;
	if (!net_ || !city_on_) return out;
	const uint64_t now = tick_ + 1;
	const int64_t tpd = ticks_per_minute() * 1440;
	for (size_t bi = 0; bi < net_->buildings.size() && bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0) continue;
		auto it = econ_.find(nb.id);
		const uint32_t owner = it != econ_.end() ? it->second.owner : 0;
		const bool household = it != econ_.end() && it->second.household != 0;
		Listing l;
		l.building = nb.id;
		l.owner = owner;
		l.value = value_of(bi);
		if (owner == 0 && !household && nb.for_sale && (it == econ_.end() || !it->second.player_sold)) {
			l.by_city = true;
			l.asking = nb.asking > 0.0 ? nb.asking : l.value;
		} else if (owner != 0 && it->second.listed) {
			l.asking = it->second.asking;
			l.days = static_cast<int>((now - std::min(now, it->second.listed_at)) / static_cast<uint64_t>(tpd));
		} else {
			continue;
		}
		out.push_back(l);
	}
	return out;
}

bool Traffic::econ_pass(Resident &r) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	if (r.pass_until > now) return true;
	const double day_pass = cd.bus_pass * cd.day_pass_share;
	const double price = r.visitor ? day_pass : cd.bus_pass;
	if (r.money < price) return false;
	r.money -= price;
	city_acc_.treasury_income += price;
	city_acc_.month_income += price;
	city_acc_.income_passes += price;
	const uint64_t days = r.visitor ? 1u : static_cast<uint64_t>(cd.month_days);
	r.pass_until = now + days * 1440ull * static_cast<uint64_t>(ticks_per_minute());
	if (r.visitor) ++city_acc_.day_passes;
	else ++city_acc_.passes_sold;
	return true;
}

// --- The calendar -----------------------------------------------------------------------------

void Traffic::econ_daily(int today) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	const int64_t tpd = ticks_per_minute() * 1440;
	// A household keeps its savings together: what one earns, all can spend.
	for (const Household &hh : hh_) {
		if (hh.members.size() < 2) continue;
		double total = 0.0;
		int n = 0;
		for (uint32_t m : hh.members) {
			const int32_t ri = find_resident(m);
			if (ri < 0) continue;
			total += res_[static_cast<size_t>(ri)].money;
			++n;
		}
		if (n == 0) continue;
		for (uint32_t m : hh.members) {
			const int32_t ri = find_resident(m);
			if (ri >= 0) res_[static_cast<size_t>(ri)].money = total / n;
		}
	}
	// Vacant units, for NPC landlords.
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0 || nb.kind != BuildingKind::Home) continue;
		int vacant = 0;
		for (int32_t u : bstate_[bi].units) vacant += u < 0 ? 1 : 0;
		econ_[nb.id].vacant_days += vacant;
	}
	if (today % cd.month_days == 0) econ_monthly(today / cd.month_days);
	// The market: a listing sells sooner the cheaper it is against what buyers
	// think the building is worth.
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0) continue;
		BEcon &e = econ_[nb.id];
		double asking;
		if (e.owner == 0 && e.household == 0 && nb.for_sale && !e.player_sold) {
			asking = nb.asking > 0.0 ? nb.asking : value_of(bi);
		} else if (e.owner != 0 && e.listed) {
			if (now > e.listed_at + static_cast<uint64_t>(cd.listing_days) * static_cast<uint64_t>(tpd)) {
				e.listed = false; // no buyer: taken off the market
				continue;
			}
			asking = e.asking;
		} else {
			continue;
		}
		const double value = value_of(bi);
		const double appeal = asking > 0.0 ? value / asking : 2.0;
		const double p = cd.sale_rate * clampd((appeal - 0.6) / 0.4, 0.0, 2.0);
		if (rng_.uniform() < p) econ_sell(bi, asking);
	}
}

void Traffic::econ_monthly(int month) {
	const CityData &cd = *city_data_;
	const uint64_t now = tick_ + 1;
	(void)month;
	// 1. NPC owners look at last month and set this month's numbers.
	for (size_t bi = 0; bi < bstate_.size(); ++bi) {
		const NetBuilding &nb = net_->buildings[bi];
		if (nb.type < 0) continue;
		const BuildingType &t = cd.types[static_cast<size_t>(nb.type)];
		BEcon &e = econ_[nb.id];
		e.net = e.income - e.expense;
		if (e.owner != 0) {
			if (t.kind == BuildingKind::Shop) {
				// Full with people turned away, or losing money: dearer. Quiet: cheaper.
				if ((e.served > 0 && e.full * 10 > e.served) || e.net < 0.0) e.price_factor *= 1.0 + cd.price_step;
				else if (e.served < t.slots * 2) e.price_factor *= 1.0 - cd.price_step;
				e.price_factor = clampd(e.price_factor, cd.price_min, cd.price_max);
			}
			if (t.business()) {
				// Shifts nobody took: better pay. Otherwise it drifts back down.
				if (e.unfilled > 0) e.wage *= 1.0 + cd.wage_step;
				else e.wage *= 1.0 - 0.25 * cd.wage_step;
				e.wage = clampd(e.wage, cd.wage_min * t.wage, cd.wage_max * t.wage);
			}
			if (t.kind == BuildingKind::Home && e.household == 0) {
				// Rents out at once: dearer. Vacant a lot: cheaper.
				const double vacancy = e.vacant_days / std::max(1.0, static_cast<double>(t.households * cd.month_days));
				if (vacancy < 0.02) e.rent *= 1.0 + cd.rent_step;
				else if (vacancy > 0.2) e.rent *= 1.0 - cd.rent_step;
				const double base = bstate_[bi].base_rent;
				e.rent = clampd(e.rent, cd.rent_min * base, cd.rent_max * base);
			}
			// Sometimes an owner puts a building on the market.
			if (!e.listed && rng_.uniform() < cd.npc_list_chance) {
				e.listed = true;
				e.listed_at = now;
				e.asking = value_of(bi) * rng_.range(0.9, 1.3);
			}
		}
		e.income = e.expense = 0.0;
		e.served = e.full = e.unfilled = 0;
		e.vacant_days = 0.0;
	}
	city_acc_.month_income = city_acc_.month_spending = 0.0;
	// 2. Rent day: tenants pay their landlord, shared between the members.
	std::vector<size_t> evict;
	for (size_t h = 0; h < hh_.size(); ++h) {
		Household &hh = hh_[h];
		if (hh.home == 0 || hh.members.empty()) continue;
		const int32_t bi = net_->building_index(hh.home);
		if (bi < 0) continue;
		hh.rent = hh.owns ? 0.0 : rent_of(static_cast<size_t>(bi));
		if (!hh.owns) {
			const double share = hh.rent / static_cast<double>(hh.members.size());
			for (uint32_t m : hh.members) {
				const int32_t ri = find_resident(m);
				if (ri >= 0) res_[static_cast<size_t>(ri)].money -= share;
			}
			econ_credit(hh.home, hh.rent, 1);
		}
		double total = 0.0;
		for (uint32_t m : hh.members) {
			const int32_t ri = find_resident(m);
			if (ri >= 0) total += res_[static_cast<size_t>(ri)].money;
		}
		hh.debt_months = total < 0.0 ? hh.debt_months + 1 : 0;
		if (hh.debt_months >= cd.eviction_months) evict.push_back(h);
	}
	for (size_t h : evict) econ_evict(h);
	// 3. Households that have saved enough buy the house they rent.
	for (size_t h = 0; h < hh_.size(); ++h) {
		Household &hh = hh_[h];
		if (hh.home == 0 || hh.owns || hh.members.empty()) continue;
		const int32_t bi = net_->building_index(hh.home);
		if (bi < 0) continue;
		const NetBuilding &nb = net_->buildings[static_cast<size_t>(bi)];
		if (nb.type < 0 || cd.types[static_cast<size_t>(nb.type)].households != 1) continue;
		double total = 0.0;
		for (uint32_t m : hh.members) {
			const int32_t ri = find_resident(m);
			if (ri >= 0) total += res_[static_cast<size_t>(ri)].money;
		}
		const double price = std::max(bstate_[static_cast<size_t>(bi)].base_price, value_of(static_cast<size_t>(bi)));
		if (total < price + cd.money_low * static_cast<double>(hh.members.size())) continue;
		for (uint32_t m : hh.members) {
			const int32_t ri = find_resident(m);
			if (ri < 0) continue;
			Resident &r = res_[static_cast<size_t>(ri)];
			r.money -= price * (r.money / total);
		}
		BEcon &e = econ_[nb.id];
		if (e.owner == 0) {
			city_acc_.treasury_income += price;
			city_acc_.month_income += price;
			city_acc_.income_buildings += price;
		} else {
			npc_cash_[e.owner] += price;
		}
		e.owner = 0;
		e.household = hh.id;
		e.listed = false;
		hh.owns = true;
		++e.sales;
		++city_acc_.homes_bought;
	}
}

void Traffic::econ_evict(size_t h) {
	Household &hh = hh_[h];
	if (BState *b = bstate(hh.home)) {
		if (hh.unit >= 0 && static_cast<size_t>(hh.unit) < b->units.size()) b->units[static_cast<size_t>(hh.unit)] = -1;
	}
	for (uint32_t m : hh.members) {
		const int32_t ri = find_resident(m);
		if (ri < 0) continue;
		Resident &r = res_[static_cast<size_t>(ri)];
		if (r.employer != 0 && r.employer != kOutside) {
			if (BState *b = bstate(r.employer)) {
				b->employees.erase(std::remove(b->employees.begin(), b->employees.end(), r.id), b->employees.end());
			}
		}
		r.employer = 0;
		r.shift = Booking{};
		r.household = -1;
		r.visitor = true; // from now on just someone on the way out
		r.leaving = true;
		r.evicted = true;
		r.then = 0;
		r.offering = -1;
		++city_acc_.evictions;
		if (r.state == ResidentState::Inside) {
			city_leave_building(r);
			r.next_plan = tick_ + 1; // plans its way to the coach
		}
	}
	hh.members.clear();
	hh.home = 0;
	++city_acc_.households_evicted;
}

} // namespace tsim
