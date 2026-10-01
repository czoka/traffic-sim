// Traffic (M6): residents' cars and bikes.
//
// A resident who owns a car or a bike keeps it at a building (home, or where
// they last left it). Trips from there can go by it: the vehicle joins the
// traffic on the kerb lane in front, drives the network like any other, and at
// the destination pulls in and is put away (a short stop in the kerb lane).
// Cars also leave and re-enter the map at the road edges for jobs and shops
// outside. Travel times between buildings by car come from one Dijkstra per
// building on the lane graph (observed lane times), refreshed every day.
//
// Like the rest of the tick: only + - * / and sqrt, the seeded RNG, and
// iteration in index order.
#include "tsim/traffic.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace tsim {

namespace {

constexpr double kInf = 1e300;
constexpr float kNoWay = 1e9f;

} // namespace

void Traffic::drive_costs() {
	const Network &n = *net_;
	const size_t B = n.buildings.size(), L = n.lanes.size();
	drive_t_.assign(B * B, kNoWay);
	drive_d_.assign(B * B, kNoWay);
	edges_.clear();
	edge_in_.clear();
	for (const NetSpawner &sp : n.spawners) {
		if (!sp.config.sink || sp.sink_lanes.empty() || sp.spawn_lanes.empty()) continue;
		edges_.push_back(sp.node);
		edge_in_.push_back(sp.spawn_lanes[0]);
	}
	const size_t E = edges_.size();
	edge_t_.assign(B * E, kNoWay);
	edge_d_.assign(B * E, kNoWay);
	edge_in_t_.assign(E * B, kNoWay);
	if (B == 0 || L == 0) return;
	std::vector<double> g(L), d(L);
	std::vector<char> closed(L);
	using Entry = std::pair<double, int32_t>;
	// Time and distance from the sources to the start of every road lane.
	auto run = [&](const std::vector<std::pair<int32_t, double>> &sources) {
		std::fill(g.begin(), g.end(), kInf);
		std::fill(d.begin(), d.end(), kInf);
		std::fill(closed.begin(), closed.end(), 0);
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
		for (const auto &src : sources) {
			const int32_t l = src.first;
			const NetLane &lane = n.lanes[static_cast<size_t>(l)];
			const double t = -lane_cost(l, VehicleKind::Car) * src.second / std::max(0.01, lane.length);
			if (t < g[static_cast<size_t>(l)]) {
				g[static_cast<size_t>(l)] = t;
				d[static_cast<size_t>(l)] = -src.second;
				open.push({ t, l });
			}
		}
		while (!open.empty()) {
			const Entry e = open.top();
			open.pop();
			const int32_t l = e.second;
			if (closed[static_cast<size_t>(l)]) continue;
			closed[static_cast<size_t>(l)] = 1;
			const NetLane &lane = n.lanes[static_cast<size_t>(l)];
			const double here = g[static_cast<size_t>(l)] + lane_cost(l, VehicleKind::Car);
			const double dist = d[static_cast<size_t>(l)] + lane.length;
			auto relax = [&](int32_t to, double t, double m) {
				if (to < 0 || closed[static_cast<size_t>(to)] || !allowed(VehicleKind::Car, n.lanes[static_cast<size_t>(to)])) return;
				if (t < g[static_cast<size_t>(to)]) {
					g[static_cast<size_t>(to)] = t;
					d[static_cast<size_t>(to)] = m;
					open.push({ t, to });
				}
			};
			for (int32_t c : lane.next) {
				const NetLane &cn = n.lanes[static_cast<size_t>(c)];
				if (cn.kind == NetLaneKind::Connector) {
					relax(cn.to, here + lane_cost(c, VehicleKind::Car), dist + cn.length);
				} else {
					relax(c, here, dist);
				}
			}
			if (lane.left >= 0 && !lane.change_left.empty()) relax(lane.left, g[static_cast<size_t>(l)] + config_.lane_change_cost, d[static_cast<size_t>(l)]);
			if (lane.right >= 0 && !lane.change_right.empty()) relax(lane.right, g[static_cast<size_t>(l)] + config_.lane_change_cost, d[static_cast<size_t>(l)]);
		}
	};
	// To a building: the cheaper of its two kerb lanes, at its door.
	auto to_building = [&](const NetBuilding &b, float &t_out, float &d_out) {
		double bt = kInf, bd = kInf;
		for (int k = 0; k < 2; ++k) {
			const int32_t l = b.car_lane[k];
			if (l < 0 || g[static_cast<size_t>(l)] >= kInf) continue;
			const NetLane &lane = n.lanes[static_cast<size_t>(l)];
			const double frac = b.car_s[k] / std::max(0.01, lane.length);
			const double t = g[static_cast<size_t>(l)] + lane_cost(l, VehicleKind::Car) * frac;
			if (t >= 0.0 && t < bt) {
				bt = t;
				bd = d[static_cast<size_t>(l)] + b.car_s[k];
			}
		}
		if (bt < 1e8) {
			t_out = static_cast<float>(bt);
			d_out = static_cast<float>(bd);
		}
	};
	for (size_t a = 0; a < B; ++a) {
		const NetBuilding &ba = n.buildings[a];
		std::vector<std::pair<int32_t, double>> src;
		for (int k = 0; k < 2; ++k) {
			if (ba.car_lane[k] >= 0) src.push_back({ ba.car_lane[k], ba.car_s[k] });
		}
		if (src.empty()) continue;
		run(src);
		for (size_t b = 0; b < B; ++b) {
			if (a == b) {
				drive_t_[a * B + b] = 0.0f;
				drive_d_[a * B + b] = 0.0f;
				continue;
			}
			to_building(n.buildings[b], drive_t_[a * B + b], drive_d_[a * B + b]);
		}
		for (size_t e = 0; e < E; ++e) {
			const NetSpawner *sp = n.spawner_at(edges_[e]);
			double bt = kInf, bd = kInf;
			for (int32_t l : sp->sink_lanes) {
				if (g[static_cast<size_t>(l)] >= kInf) continue;
				const double t = g[static_cast<size_t>(l)] + lane_cost(l, VehicleKind::Car);
				if (t < bt) {
					bt = t;
					bd = d[static_cast<size_t>(l)] + n.lanes[static_cast<size_t>(l)].length;
				}
			}
			if (bt < 1e8) {
				edge_t_[a * E + e] = static_cast<float>(bt);
				edge_d_[a * E + e] = static_cast<float>(bd);
			}
		}
	}
	for (size_t e = 0; e < E; ++e) {
		run({ { edge_in_[e], 0.0 } });
		float dummy = 0.0f;
		for (size_t b = 0; b < B; ++b) to_building(n.buildings[b], edge_in_t_[e * B + b], dummy);
	}
}

double Traffic::drive_s(uint32_t from_building, uint32_t to_building) const {
	if (!net_) return kNoWay;
	const int32_t a = net_->building_index(from_building), b = net_->building_index(to_building);
	const size_t B = net_->buildings.size();
	if (a < 0 || b < 0 || drive_t_.size() != B * B) return kNoWay;
	return drive_t_[static_cast<size_t>(a) * B + static_cast<size_t>(b)];
}

double Traffic::drive_m(uint32_t from_building, uint32_t to_building) const {
	if (!net_) return kNoWay;
	const int32_t a = net_->building_index(from_building), b = net_->building_index(to_building);
	const size_t B = net_->buildings.size();
	if (a < 0 || b < 0 || drive_d_.size() != B * B) return kNoWay;
	return drive_d_[static_cast<size_t>(a) * B + static_cast<size_t>(b)];
}

// --- Trips -------------------------------------------------------------------------------------

namespace {

// Travel time of a planned route: the lanes it uses, at their observed times.
double route_time(const Network &n, const std::vector<double> &lane_time, const std::vector<int32_t> &route) {
	double t = 0.0;
	for (int32_t c : route) {
		t += lane_time[static_cast<size_t>(c)];
		t += lane_time[static_cast<size_t>(n.lanes[static_cast<size_t>(c)].to)];
	}
	return t;
}

} // namespace

bool Traffic::city_drive(size_t i, uint32_t to_building, bool bike, bool to_edge, bool *busy) {
	const CityData &cd = *city_data_;
	const Network &n = *net_;
	Resident &r = res_[i];
	if (r.state != ResidentState::Inside) return false;
	const int32_t fbi = n.building_index(r.at);
	if (fbi < 0) return false;
	const NetBuilding &fb = n.buildings[static_cast<size_t>(fbi)];
	const VehicleKind kind = bike ? VehicleKind::Bike : VehicleKind::Car;
	const DriverParams2 drv = typical_driver(kind);
	// Where to: the map edge with the shortest drive, or the destination's kerb.
	NodeId edge = kNoId;
	const NetBuilding *tb = nullptr;
	if (to_edge) {
		double best = kInf;
		const size_t E = edges_.size();
		for (size_t e = 0; e < E; ++e) {
			const double t = edge_t_[static_cast<size_t>(fbi) * E + e];
			if (t < best) {
				best = t;
				edge = edges_[e];
			}
		}
		if (edge == kNoId) return false;
	} else {
		const int32_t tbi = n.building_index(to_building);
		if (tbi < 0) return false;
		tb = &n.buildings[static_cast<size_t>(tbi)];
	}
	double best = kInf;
	int32_t s_lane = -1, g_lane = -1;
	double s_s = 0.0, g_s = 0.0;
	std::vector<int32_t> best_route, route;
	for (int a = 0; a < 2; ++a) {
		const int32_t sl = bike ? fb.bike_lane[a] : fb.car_lane[a];
		const double ss = bike ? fb.bike_s[a] : fb.car_s[a];
		if (sl < 0 || !allowed(kind, n.lanes[static_cast<size_t>(sl)])) continue;
		if (!lane_free_at(sl, ss, drv.length + 1.0)) {
			if (busy) *busy = true;
			continue;
		}
		if (to_edge) {
			if (!find_route(sl, edge, route, true, kind)) continue;
			const double t = route_time(n, lane_time_, route);
			if (t < best) {
				best = t;
				s_lane = sl;
				s_s = ss;
				best_route = route;
			}
			continue;
		}
		for (int b = 0; b < 2; ++b) {
			const int32_t gl = bike ? tb->bike_lane[b] : tb->car_lane[b];
			const double gs = bike ? tb->bike_s[b] : tb->car_s[b];
			if (gl < 0 || !find_route_to_lane(sl, ss, gl, gs, route, kind, true)) continue;
			const double t = route_time(n, lane_time_, route);
			if (t < best) {
				best = t;
				s_lane = sl;
				s_s = ss;
				g_lane = gl;
				g_s = gs;
			}
		}
	}
	if (s_lane < 0) return false;
	std::vector<Waypoint> wps;
	if (!to_edge) {
		Waypoint w;
		w.action = WaypointAction::Arrive;
		w.lane = g_lane;
		w.s = g_s;
		w.dwell = bike ? 5.0 : cd.parking_minutes * 15.0; // pulling in; the rest of parking is inside
		wps.push_back(w);
	}
	const VehicleId id = add_vehicle(s_lane, s_s, 0.0, edge, nullptr, kind, wps);
	if (id == kNoId) return false;
	veh_.back().resident = r.id;
	city_leave_building(r);
	r.state = ResidentState::Travelling;
	r.veh = id;
	r.ped = 0;
	r.mode = bike ? TripMode::Bike : TripMode::Car;
	r.going = to_edge ? station_place_ : building_place(to_building);
	r.going_building = to_edge ? kOutside : to_building;
	r.out_edge = edge;
	if (bike) r.bike_at = kOutside;
	else r.car_at = kOutside;
	r.doing = Doing::Idle;
	if (bike) ++city_acc_.trips_bike;
	else ++city_acc_.trips_car;
	return true;
}

bool Traffic::city_drive_in(size_t i) {
	const CityData &cd = *city_data_;
	const Network &n = *net_;
	Resident &r = res_[i];
	const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
	const int32_t hbi = home ? n.building_index(home) : -1;
	if (hbi < 0) return false;
	const NetBuilding &hb = n.buildings[static_cast<size_t>(hbi)];
	// The edge it left by, if it still is one; else any edge.
	size_t e = edges_.size();
	for (size_t k = 0; k < edges_.size(); ++k) {
		if (edges_[k] == r.out_edge) e = k;
	}
	if (e == edges_.size()) e = 0;
	if (e >= edges_.size()) return false;
	const int32_t sl = edge_in_[e];
	const DriverParams2 drv = typical_driver(VehicleKind::Car);
	if (!lane_free_at(sl, drv.length + 1.0, drv.length + 1.0)) return false;
	double best = kInf;
	int32_t g_lane = -1;
	double g_s = 0.0;
	std::vector<int32_t> route;
	for (int b = 0; b < 2; ++b) {
		const int32_t gl = hb.car_lane[b];
		if (gl < 0 || !find_route_to_lane(sl, drv.length + 1.0, gl, hb.car_s[b], route, VehicleKind::Car, true)) continue;
		const double t = route_time(n, lane_time_, route);
		if (t < best) {
			best = t;
			g_lane = gl;
			g_s = hb.car_s[b];
		}
	}
	if (g_lane < 0) return false;
	Waypoint w;
	w.action = WaypointAction::Arrive;
	w.lane = g_lane;
	w.s = g_s;
	w.dwell = cd.parking_minutes * 15.0;
	const VehicleId id = add_vehicle(sl, drv.length + 1.0, 8.0, kNoId, nullptr, VehicleKind::Car, { w });
	if (id == kNoId) return false;
	veh_.back().resident = r.id;
	r.state = ResidentState::Travelling;
	r.veh = id;
	r.mode = TripMode::Car;
	r.going = building_place(home);
	r.going_building = home;
	r.doing = Doing::Idle;
	// The outside driving (both ways).
	const double fuel = 2.0 * cd.outside_drive_km * cd.car_cost_per_km;
	r.money -= fuel;
	city_acc_.outside_fuel += fuel;
	++city_acc_.trips_car;
	return true;
}

void Traffic::city_vehicle_done(const VehDone &d) {
	const CityData &cd = *city_data_;
	const int32_t ri = find_resident(d.resident);
	if (ri < 0) return;
	Resident &r = res_[static_cast<size_t>(ri)];
	if (r.state != ResidentState::Travelling || r.veh == kNoId) return;
	const bool car = r.mode == TripMode::Car;
	r.veh = kNoId;
	if (car) {
		const double fuel = d.metres / 1000.0 * cd.car_cost_per_km;
		r.money -= fuel;
		city_acc_.outside_fuel += fuel;
	}
	if (d.at_edge && r.going_building == kOutside) {
		// Out of the map by car: the job or the shops outside, then back the same way.
		r.state = ResidentState::Outside;
		r.going = -1;
		if (car) r.car_at = kOutside;
		else r.bike_at = kOutside;
		return;
	}
	// Arrived (or, lost on the way, put where it was going).
	uint32_t to = r.going_building;
	const uint32_t home = r.household >= 0 ? hh_[static_cast<size_t>(r.household)].home : 0;
	if (to == 0 || to == kOutside || !bstate(to)) to = home;
	if (to == 0) {
		r.state = ResidentState::Outside;
		r.leaving = true;
		r.visitor = true;
		return;
	}
	r.going_building = to;
	if (car) r.car_at = to;
	else r.bike_at = to;
	city_arrive(r.id);
}

bool Traffic::give_vehicle(uint32_t resident, bool car) {
	const int32_t ri = find_resident(resident);
	if (ri < 0) return false;
	Resident &r = res_[static_cast<size_t>(ri)];
	if (r.state != ResidentState::Inside) return false;
	if (car) {
		r.has_car = true;
		r.car_at = r.at;
	} else {
		r.has_bike = true;
		r.bike_at = r.at;
	}
	return true;
}

} // namespace tsim
