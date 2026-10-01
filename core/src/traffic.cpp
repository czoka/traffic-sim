#include "tsim/traffic.h"

#include "tsim/hash.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <queue>

namespace tsim {

const char *vehicle_kind_name(VehicleKind k) {
	switch (k) {
		case VehicleKind::Car:
			return "car";
		case VehicleKind::Taxi:
			return "taxi";
		case VehicleKind::Bus:
			return "bus";
		case VehicleKind::Coach:
			return "coach";
		case VehicleKind::Bike:
			return "bike";
	}
	return "car";
}

const char *vehicle_state_name(VehicleState s) {
	switch (s) {
		case VehicleState::Driving:
			return "driving";
		case VehicleState::Queued:
			return "queued";
		case VehicleState::Approaching:
			return "approaching junction";
		case VehicleState::Yielding:
			return "yielding";
		case VehicleState::BoxBlocked:
			return "waiting for the junction to clear";
		case VehicleState::ExitBlocked:
			return "exit full (don't block the box)";
		case VehicleState::StopSign:
			return "all-way stop";
		case VehicleState::InJunction:
			return "in junction";
		case VehicleState::ChangingLane:
			return "waiting to change lanes";
		case VehicleState::RedLight:
			return "red light";
		case VehicleState::AtStop:
			return "at a stop";
		case VehicleState::Parking:
			return "parking manoeuvre";
		case VehicleState::Parked:
			return "parked";
		case VehicleState::GivingWay:
			return "giving way to people crossing";
	}
	return "driving";
}

namespace {

constexpr double kInf = 1e300;
constexpr uint64_t kNever = ~0ull;
constexpr VehicleId kPedestrians = 0xFFFFFFFEu; // leader(): people on a crossing

bool is_road(const NetLane &l) { return l.kind == NetLaneKind::Road; }

const NetDepot *depot_at(const Network &n, NodeId node) {
	for (const NetDepot &d : n.depots) {
		if (d.node == node) return &d;
	}
	return nullptr;
}

// Seconds of a parking manoeuvre in and out, by bay style.
double park_in_time(ParkingStyle s) {
	switch (s) {
		case ParkingStyle::Parallel:
			return 8.0;
		case ParkingStyle::Angle45:
			return 5.0;
		case ParkingStyle::Perpendicular:
			return 7.0;
	}
	return 8.0;
}

double park_out_time(ParkingStyle s) { return s == ParkingStyle::Perpendicular ? 8.0 : 6.0; }

constexpr double kBayManoeuvre = 3.0; // s for a bus to pull into or out of a lay-by
constexpr double kStationSlot = 15.0; // m per bay at the main station

} // namespace

Traffic::Traffic(TrafficConfig config) : config_(config) {}

bool Traffic::allowed(VehicleKind k, const NetLane &l) {
	if (l.type == LaneType::Bike) return k == VehicleKind::Bike;
	return true;
}

DriverParams2 Traffic::typical_driver(VehicleKind k) {
	DriverParams2 d;
	switch (k) {
		case VehicleKind::Car:
		case VehicleKind::Taxi:
			break;
		case VehicleKind::Bus:
			d.T = 1.5;
			d.a = 1.0;
			d.b = 1.8;
			d.s0 = 2.5;
			d.length = 12.0;
			d.critical_gap = 6.0;
			d.politeness = 0.5;
			d.max_speed = 14.0;
			d.width = 2.55;
			break;
		case VehicleKind::Coach:
			d.T = 1.5;
			d.a = 0.9;
			d.b = 1.8;
			d.s0 = 2.5;
			d.length = 13.0;
			d.critical_gap = 6.0;
			d.politeness = 0.5;
			d.max_speed = 25.0;
			d.width = 2.55;
			break;
		case VehicleKind::Bike:
			d.T = 1.0;
			d.a = 1.0;
			d.b = 2.0;
			d.s0 = 1.0;
			d.length = 1.8;
			d.critical_gap = 4.0;
			d.politeness = 0.2;
			d.max_speed = 5.5;
			d.width = 0.7;
			break;
	}
	d.two_sqrt_ab = 2.0 * std::sqrt(d.a * d.b);
	return d;
}

DriverParams2 Traffic::random_driver(VehicleKind k) {
	DriverParams2 d = typical_driver(k);
	if (k == VehicleKind::Car || k == VehicleKind::Taxi) {
		d.speed_factor = rng_.range(0.9, 1.1);
		d.T = rng_.range(1.0, 1.6);
		d.a = rng_.range(1.0, 1.8);
		d.b = rng_.range(1.6, 2.4);
		d.s0 = 2.0;
		d.length = rng_.range(4.2, 4.8);
		d.critical_gap = rng_.range(4.0, 6.0);
		d.politeness = rng_.range(0.1, 0.5);
	} else if (k == VehicleKind::Bike) {
		d.max_speed = rng_.range(4.5, 6.5);
		d.a = rng_.range(0.8, 1.2);
		d.critical_gap = rng_.range(3.5, 5.0);
	}
	d.two_sqrt_ab = 2.0 * std::sqrt(d.a * d.b);
	return d;
}

// --- Network binding ----------------------------------------------------------------

void Traffic::set_network(const Network *net) {
	struct SavedWp {
		LaneKey lane;
		bool has_stop = false, has_bay = false;
		uint32_t stop_id = 0;
		std::pair<LaneId, int> bay{ kNoId, 0 };
	};
	struct Saved {
		LaneKey lane, prev;
		bool grant = false, held = false;
		LaneKey grant_key, held_key;
		std::vector<SavedWp> wps; // pending waypoints only
	};
	std::vector<Saved> saved(veh_.size());
	const std::vector<PedSaved> people = peds_save();
	std::map<LaneKey, double> times;
	std::map<NodeId, uint32_t> pending;
	if (net_) {
		for (size_t i = 0; i < veh_.size(); ++i) {
			const Vehicle &v = veh_[i];
			Saved &s = saved[i];
			s.lane = net_->lanes[static_cast<size_t>(v.lane)].key;
			s.prev = net_->lanes[static_cast<size_t>(v.prev_lane)].key;
			if (v.grant >= 0) {
				s.grant = true;
				s.grant_key = net_->lanes[static_cast<size_t>(v.grant)].key;
			}
			if (v.held >= 0) {
				s.held = true;
				s.held_key = net_->lanes[static_cast<size_t>(v.held)].key;
			}
			for (size_t w = v.wi; w < v.waypoints.size(); ++w) {
				const Waypoint &wp = v.waypoints[w];
				SavedWp sw;
				sw.lane = net_->lanes[static_cast<size_t>(wp.lane)].key;
				if (wp.stop >= 0) {
					sw.has_stop = true;
					sw.stop_id = net_->stops[static_cast<size_t>(wp.stop)].id;
				}
				if (wp.action == WaypointAction::Park && wp.bay >= 0) {
					const NetBay &b = net_->bays[static_cast<size_t>(wp.bay)];
					sw.has_bay = true;
					sw.bay = { b.parking_lane, b.index };
				}
				s.wps.push_back(sw);
			}
		}
		for (size_t l = 0; l < lane_time_.size() && l < net_->lanes.size(); ++l) times[net_->lanes[l].key] = lane_time_[l];
		for (size_t k = 0; k < pending_.size() && k < net_->spawners.size(); ++k) {
			pending[net_->spawners[k].node] = pending_[k];
		}
	}
	net_ = net;
	const size_t n = net_ ? net_->lanes.size() : 0;
	cars_.assign(n, {});
	lane_time_.assign(n, 0.0);
	for (size_t l = 0; l < n; ++l) {
		const NetLane &lane = net_->lanes[l];
		const double free = lane.length / std::max(1.0, lane.speed_limit);
		auto it = times.find(lane.key);
		lane_time_[l] = it != times.end() ? std::max(free, it->second) : free;
	}
	pending_.assign(net_ ? net_->spawners.size() : 0, 0);
	for (size_t k = 0; k < pending_.size(); ++k) {
		auto it = pending.find(net_->spawners[k].node);
		if (it != pending.end()) pending_[k] = it->second;
	}
	junction_last_grant_.assign(net_ ? net_->junctions.size() : 0, tick_);
	junction_waiting_.assign(net_ ? net_->junctions.size() : 0, 0);
	bay_use_.assign(net_ ? net_->bays.size() : 0, kNoId);
	stop_use_.assign(net_ ? net_->stops.size() : 0, 0);
	ring_lanes_.assign(net_ ? net_->junctions.size() : 0, {});
	ring_slots_.assign(net_ ? net_->junctions.size() : 0, {});
	ring_cycle_.assign(n, -1);
	for (size_t l = 0; l < n; ++l) {
		const NetLane &lane = net_->lanes[l];
		if (!lane.ring) continue;
		const int32_t j = net_->junction_at(lane.end_node);
		if (j < 0) continue;
		ring_lanes_[static_cast<size_t>(j)].push_back(static_cast<int32_t>(l));
		size_t k = 0;
		for (int32_t r = lane.right; r >= 0 && k < 8; r = net_->lanes[static_cast<size_t>(r)].right) ++k;
		ring_cycle_[l] = static_cast<int8_t>(k);
		std::vector<int32_t> &slots = ring_slots_[static_cast<size_t>(j)];
		if (slots.size() <= k) slots.resize(k + 1, -1);
		// A car needs length + s0 + margin to enter a piece, 6.5 m for each one queued behind.
		slots[k] += 1 + static_cast<int32_t>(std::max(0.0, (lane.length - 8.5) / 6.5));
	}
	rebuild_reachability();
	if (!net_) {
		veh_.clear();
		routes_.clear();
		coaches_.clear();
		peds_.clear();
		peds_reset_network();
		return;
	}
	const Network &nw = *net_;
	std::map<std::pair<LaneId, int>, int32_t> bay_index;
	for (size_t b = 0; b < nw.bays.size(); ++b) bay_index[{ nw.bays[b].parking_lane, nw.bays[b].index }] = static_cast<int32_t>(b);
	// Carry vehicles over by key.
	std::vector<Vehicle> kept;
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle v = veh_[i];
		const Saved &s = saved[i];
		v.lane = nw.find(s.lane);
		if (v.lane < 0) continue;
		const NetLane &lane = nw.lanes[static_cast<size_t>(v.lane)];
		v.s = std::min(v.s, lane.length);
		v.prev_lane = v.lane;
		v.prev_s = v.s;
		v.prev_lat = v.lat;
		v.grant = s.grant ? nw.find(s.grant_key) : -1;
		v.held = s.held ? nw.find(s.held_key) : -1;
		if (v.grant >= 0 && nw.lanes[static_cast<size_t>(v.grant)].from != v.lane) v.grant = -1;
		v.route.clear();
		v.ri = 0;
		v.wait_since = 0;
		v.stopped_tick = 0;
		// Waypoints: stops by id, bays by their parking lane and index, the rest by lane.
		std::vector<Waypoint> wps;
		for (size_t w = 0; w < s.wps.size(); ++w) {
			Waypoint wp = v.waypoints[v.wi + w];
			const SavedWp &sw = s.wps[w];
			const bool current = w == 0 && v.phase != 0;
			if (sw.has_stop) {
				wp.stop = nw.stop_index(sw.stop_id);
				if (wp.stop >= 0) {
					wp.lane = nw.stops[static_cast<size_t>(wp.stop)].lane;
					wp.s = nw.stops[static_cast<size_t>(wp.stop)].s;
				}
			} else if (sw.has_bay) {
				auto it = bay_index.find(sw.bay);
				wp.bay = it != bay_index.end() ? it->second : -1;
				if (wp.bay >= 0) {
					wp.lane = nw.bays[static_cast<size_t>(wp.bay)].lane;
					wp.s = nw.bays[static_cast<size_t>(wp.bay)].s;
				} else {
					wp.lane = -1;
				}
			} else {
				wp.lane = nw.find(sw.lane);
			}
			if (current) {
				// It is at this waypoint now (stopped or in the bay): it leaves from where it is.
				wp.lane = v.lane;
				wp.s = std::min(wp.s, lane.length);
				if (sw.has_stop && wp.stop < 0) wp.stop = -1;
			} else if (wp.lane < 0 || (sw.has_stop && wp.stop < 0)) {
				continue; // gone
			}
			wps.push_back(wp);
		}
		if (v.phase != 0 && (s.wps.empty() || wps.empty() || wps[0].lane != v.lane)) {
			v.phase = 0;
			v.off_lane = false;
		}
		v.waypoints = std::move(wps);
		v.wi = 0;
		kept.push_back(std::move(v));
	}
	veh_ = std::move(kept);
	recount_use();
	init_transit(true);
	rebuild_lists();
	for (Vehicle &v : veh_) {
		if (v.phase != 0) continue; // routes on when it leaves the stop or bay
		if (!retarget(v)) v.route.clear();
	}
	estimate_routes();
	peds_reset_network();
	peds_restore(people);
}

void Traffic::recount_use() {
	std::fill(bay_use_.begin(), bay_use_.end(), kNoId);
	std::fill(stop_use_.begin(), stop_use_.end(), 0u);
	for (const Vehicle &v : veh_) {
		const Waypoint *wp = pending_waypoint(v);
		if (!wp) continue;
		if (wp->action == WaypointAction::Park && wp->bay >= 0 && v.phase != 4) {
			bay_use_[static_cast<size_t>(wp->bay)] = v.id;
		}
		if (wp->action == WaypointAction::BayStop && wp->stop >= 0 && (v.phase == 2 || v.phase == 3)) {
			++stop_use_[static_cast<size_t>(wp->stop)];
		}
	}
}

void Traffic::init_transit(bool keep) {
	std::map<uint32_t, RouteRun> old_routes;
	std::map<uint32_t, CoachRun> old_coaches;
	if (keep) {
		for (const RouteRun &r : routes_) old_routes[r.id] = r;
		for (const CoachRun &c : coaches_) old_coaches[c.id] = c;
	}
	routes_.clear();
	coaches_.clear();
	if (!net_) return;
	const Network &n = *net_;
	for (size_t d = 0; d < n.depots.size(); ++d) {
		for (size_t r = 0; r < n.depots[d].routes.size(); ++r) {
			RouteRun rr;
			rr.id = n.depots[d].routes[r].id;
			rr.depot = n.depots[d].node;
			rr.depot_idx = d;
			rr.route_idx = r;
			rr.next_departure = tick_ + 20ull * routes_.size(); // first buses 2 s apart
			auto it = old_routes.find(rr.id);
			if (it != old_routes.end()) {
				rr.next_departure = it->second.next_departure;
				rr.runs = it->second.runs;
			}
			routes_.push_back(rr);
		}
	}
	for (size_t c = 0; c < n.coach_lines.size(); ++c) {
		CoachRun cr;
		cr.id = n.coach_lines[c].id;
		cr.line = c;
		cr.next_departure = tick_ + 600ull + 300ull * c;
		auto it = old_coaches.find(cr.id);
		if (it != old_coaches.end()) cr.next_departure = it->second.next_departure;
		coaches_.push_back(cr);
	}
}

void Traffic::rebuild_reachability() {
	reach_.clear();
	if (!net_) return;
	const Network &n = *net_;
	for (const NetSpawner &src : n.spawners) {
		std::vector<std::pair<size_t, double>> row;
		if ((src.config.rate > 0.0 && !src.spawn_lanes.empty()) || (src.config.bikes > 0.0 && !src.bike_lanes.empty())) {
			std::vector<char> seen(n.lanes.size(), 0);
			std::vector<int32_t> stack(src.spawn_lanes.begin(), src.spawn_lanes.end());
			stack.insert(stack.end(), src.bike_lanes.begin(), src.bike_lanes.end());
			for (int32_t l : stack) seen[static_cast<size_t>(l)] = 1;
			while (!stack.empty()) {
				const int32_t l = stack.back();
				stack.pop_back();
				const NetLane &lane = n.lanes[static_cast<size_t>(l)];
				auto push = [&](int32_t m) {
					if (m >= 0 && !seen[static_cast<size_t>(m)]) {
						seen[static_cast<size_t>(m)] = 1;
						stack.push_back(m);
					}
				};
				for (int32_t m : lane.next) push(m);
				if (!lane.change_left.empty()) push(lane.left);
				if (!lane.change_right.empty()) push(lane.right);
			}
			for (size_t k = 0; k < n.spawners.size(); ++k) {
				const NetSpawner &dst = n.spawners[k];
				if (dst.node == src.node || !dst.config.sink) continue;
				const double w = src.config.weight_to(dst.node);
				if (w <= 0.0) continue;
				bool ok = false;
				for (int32_t l : dst.sink_lanes) ok |= seen[static_cast<size_t>(l)] != 0;
				if (ok) row.push_back({ k, w });
			}
		}
		reach_.push_back(std::move(row));
	}
}

void Traffic::reset(uint64_t seed) {
	seed_ = seed;
	rng_.reseed(seed);
	tick_ = 0;
	next_id_ = 1;
	veh_.clear();
	for (auto &c : cars_) c.clear();
	std::fill(pending_.begin(), pending_.end(), 0u);
	std::fill(junction_last_grant_.begin(), junction_last_grant_.end(), 0ull);
	std::fill(junction_waiting_.begin(), junction_waiting_.end(), 0);
	std::fill(bay_use_.begin(), bay_use_.end(), kNoId);
	std::fill(stop_use_.begin(), stop_use_.end(), 0u);
	if (net_) {
		for (size_t l = 0; l < lane_time_.size(); ++l) {
			const NetLane &lane = net_->lanes[l];
			lane_time_[l] = lane.length / std::max(1.0, lane.speed_limit);
		}
	}
	init_transit(false);
	estimate_routes();
	stats_ = TrafficStats{};
	trip_time_sum_ = 0.0;
	peds_.clear();
	next_ped_id_ = 1;
	stop_acc_.clear();
	load_acc_.clear();
	crossing_wait_sum_ = 0.0;
	wait_sum_ = 0.0;
	peds_reset_network();
}

// --- Routing ---------------------------------------------------------------------------

bool Traffic::exits_at(const NetLane &l, NodeId node) const {
	if (!is_road(l) || l.end_node != node || l.ring) return false;
	if (l.sink) return true;
	return l.type != LaneType::Bike && depot_at(*net_, node) != nullptr;
}

bool Traffic::goal_pos(NodeId node, Vec2 &pos) const {
	const NetSpawner *sp = net_->spawner_at(node);
	if (sp && sp->config.sink) {
		pos = sp->pos;
		return true;
	}
	if (const NetDepot *d = depot_at(*net_, node)) {
		pos = d->pos;
		return true;
	}
	return false;
}

double Traffic::lane_cost(int32_t l, VehicleKind kind) const {
	const NetLane &lane = net_->lanes[static_cast<size_t>(l)];
	double t = lane_time_[static_cast<size_t>(l)];
	switch (kind) {
		case VehicleKind::Car:
			if (lane.type == LaneType::Bus) t *= config_.bus_lane_factor;
			break;
		case VehicleKind::Taxi:
			break;
		case VehicleKind::Bus:
		case VehicleKind::Coach:
			if (lane.type == LaneType::Bus) t *= 0.7;
			break;
		case VehicleKind::Bike: {
			const double bt = lane.length / 5.0;
			if (bt > t) t = bt;
			if (lane.type == LaneType::Bus) t *= 1.3;
			else if (lane.type != LaneType::Bike) t *= 2.5;
			break;
		}
	}
	return t;
}

bool Traffic::astar(int32_t start, const Goal &goal, VehicleKind kind, bool allow_change_first,
		std::vector<int32_t> &route) const {
	route.clear();
	const Network &n = *net_;
	auto is_goal = [&](int32_t l) {
		if (goal.lane >= 0) return l == goal.lane;
		return exits_at(n.lanes[static_cast<size_t>(l)], goal.node);
	};
	if (!goal.need_connector && is_goal(start)) return true;
	const size_t N = n.lanes.size();
	std::vector<double> g(N, kInf);
	std::vector<int32_t> parent(N, -1), via(N, -1);
	std::vector<char> closed(N, 0);
	using Entry = std::pair<double, int32_t>;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	auto h = [&](int32_t l) { return (n.lanes[static_cast<size_t>(l)].pts.back() - goal.pos).length() / n.max_speed; };
	g[static_cast<size_t>(start)] = lane_cost(start, kind);
	open.push({ g[static_cast<size_t>(start)] + h(start), start });
	double best = kInf;
	int32_t best_from = -1, best_via = -1;
	while (!open.empty()) {
		if (open.top().first >= best) break;
		const int32_t l = open.top().second;
		open.pop();
		if (closed[static_cast<size_t>(l)]) continue;
		closed[static_cast<size_t>(l)] = 1;
		const NetLane &lane = n.lanes[static_cast<size_t>(l)];
		const double gl = g[static_cast<size_t>(l)];
		auto relax = [&](int32_t to, double cost, int32_t through) {
			const double ng = gl + cost;
			if (is_goal(to) && (to != start || through >= 0)) {
				if (ng < best) {
					best = ng;
					best_from = l;
					best_via = through;
				}
				return;
			}
			if (closed[static_cast<size_t>(to)]) return;
			if (ng < g[static_cast<size_t>(to)]) {
				g[static_cast<size_t>(to)] = ng;
				parent[static_cast<size_t>(to)] = l;
				via[static_cast<size_t>(to)] = through;
				open.push({ ng + h(to), to });
			}
		};
		for (int32_t c : lane.next) {
			const NetLane &cn = n.lanes[static_cast<size_t>(c)];
			if (!allowed(kind, cn) || !allowed(kind, n.lanes[static_cast<size_t>(cn.to)])) continue;
			double turn = cn.route_penalty;
			if (cn.turn == TurnKind::Left) turn += 2.0;
			else if (cn.turn == TurnKind::Right) turn += 1.0;
			else if (cn.turn == TurnKind::UTurn) turn += 10.0;
			if (lane.merge_end) turn += 5.0; // prefer leaving a dropped lane early
			double ct = lane_time_[static_cast<size_t>(c)];
			if (kind == VehicleKind::Bike) ct = std::max(ct, cn.length / 5.0);
			relax(cn.to, ct + turn + lane_cost(cn.to, kind), c);
		}
		if (allow_change_first || l != start) {
			for (int side = 0; side < 2; ++side) {
				const int32_t nb = side == 0 ? lane.left : lane.right;
				if (nb < 0 || (side == 0 ? lane.change_left : lane.change_right).empty()) continue;
				if (!allowed(kind, n.lanes[static_cast<size_t>(nb)])) continue;
				const double extra = lane_cost(nb, kind) - lane_cost(l, kind);
				relax(nb, config_.lane_change_cost + (extra > 0.0 ? extra : 0.0), -1);
			}
		}
	}
	if (best_from < 0) return false;
	if (best_via >= 0) route.push_back(best_via);
	for (int32_t l = best_from; l != start; l = parent[static_cast<size_t>(l)]) {
		if (via[static_cast<size_t>(l)] >= 0) route.push_back(via[static_cast<size_t>(l)]);
	}
	std::reverse(route.begin(), route.end());
	return true;
}

bool Traffic::find_route(int32_t start, NodeId dest, std::vector<int32_t> &route, bool allow_change_first,
		VehicleKind kind) const {
	route.clear();
	if (!net_ || start < 0 || static_cast<size_t>(start) >= net_->lanes.size()) return false;
	if (!is_road(net_->lanes[static_cast<size_t>(start)])) return false;
	Goal g;
	g.node = dest;
	if (!goal_pos(dest, g.pos)) return false;
	return astar(start, g, kind, allow_change_first, route);
}

bool Traffic::find_route_to_lane(int32_t start, double start_s, int32_t goal, double goal_s, std::vector<int32_t> &route,
		VehicleKind kind, bool allow_change_first) const {
	route.clear();
	if (!net_ || start < 0 || goal < 0) return false;
	const Network &n = *net_;
	if (static_cast<size_t>(start) >= n.lanes.size() || static_cast<size_t>(goal) >= n.lanes.size()) return false;
	if (!is_road(n.lanes[static_cast<size_t>(start)]) || !is_road(n.lanes[static_cast<size_t>(goal)])) return false;
	Goal g;
	g.lane = goal;
	g.pos = n.lanes[static_cast<size_t>(goal)].pts.front();
	const NetLane &sl = n.lanes[static_cast<size_t>(start)];
	const NetLane &gl = n.lanes[static_cast<size_t>(goal)];
	if (start == goal) {
		g.need_connector = start_s > goal_s - 1.0;
	} else if (sl.segment == gl.segment && sl.dir == gl.dir) {
		double m = n.map_across(goal, goal_s, start);
		if (m < 0.0) m = goal_s;
		if (start_s > m - 1.0) {
			// Already past it on this road: go round.
			g.need_connector = true;
			allow_change_first = false;
		}
	}
	return astar(start, g, kind, allow_change_first, route);
}

bool Traffic::reroute(Vehicle &v, bool allow_change_first) {
	const NetLane &lane = net_->lanes[static_cast<size_t>(v.lane)];
	const bool road = is_road(lane);
	const int32_t start = road ? v.lane : lane.to;
	std::vector<int32_t> r;
	bool ok;
	if (const Waypoint *wp = pending_waypoint(v)) {
		ok = find_route_to_lane(start, road ? v.s : 0.0, wp->lane, wp->s, r, v.kind, allow_change_first || !road);
	} else {
		ok = find_route(start, v.dest, r, allow_change_first || !road, v.kind);
	}
	if (!ok) return false;
	v.route = std::move(r);
	v.ri = 0;
	++stats_.reroutes;
	return true;
}

bool Traffic::retarget(Vehicle &v) {
	for (;;) {
		if (reroute(v, true)) return true;
		if (!pending_waypoint(v)) break;
		skip_waypoint(v);
	}
	// The destination is gone or unreachable: take the first sink that works.
	for (const NetSpawner &sp : net_->spawners) {
		if (!sp.config.sink || sp.node == v.dest) continue;
		const NodeId old = v.dest;
		v.dest = sp.node;
		if (reroute(v, true)) return true;
		v.dest = old;
	}
	return false;
}

const Waypoint *Traffic::pending_waypoint(const Vehicle &v) const {
	return v.wi < v.waypoints.size() ? &v.waypoints[v.wi] : nullptr;
}

int32_t Traffic::next_connector(const Vehicle &v, int32_t lane, size_t ri) const {
	const Network &n = *net_;
	const NetLane &l = n.lanes[static_cast<size_t>(lane)];
	if (v.grant >= 0 && n.lanes[static_cast<size_t>(v.grant)].from == lane) return v.grant;
	if (ri >= v.route.size()) return -1;
	const int32_t want = v.route[ri];
	const NetLane &w = n.lanes[static_cast<size_t>(want)];
	if (w.from == lane) return want;
	// Any connector from this lane into the same road and direction.
	const NetLane &target = n.lanes[static_cast<size_t>(w.to)];
	int32_t best = -1, bus = -1;
	for (int32_t c : l.next) {
		const NetLane &cn = n.lanes[static_cast<size_t>(c)];
		const NetLane &to = n.lanes[static_cast<size_t>(cn.to)];
		if (!allowed(v.kind, cn) || !allowed(v.kind, to)) continue;
		if (to.segment != target.segment || to.dir != target.dir) continue;
		if (cn.to == w.to) return c;
		if (v.kind == VehicleKind::Car && to.type == LaneType::Bus) {
			if (bus < 0) bus = c;
			continue;
		}
		if (best < 0) best = c;
	}
	return best >= 0 ? best : bus;
}

bool Traffic::at_route_end(const Vehicle &v, int32_t lane, size_t ri) const {
	return ri >= v.route.size() && !pending_waypoint(v) && exits_at(net_->lanes[static_cast<size_t>(lane)], v.dest);
}

bool Traffic::lane_is_good(const Vehicle &v, int32_t lane, size_t ri) const {
	if (ri >= v.route.size()) {
		if (const Waypoint *wp = pending_waypoint(v)) return lane == wp->lane;
		return exits_at(net_->lanes[static_cast<size_t>(lane)], v.dest);
	}
	return next_connector(v, lane, ri) >= 0;
}

// Direction of the nearest lane on this road that can take the next route
// step: -1 left, +1 right, 0 when the current lane is fine. `steps` is the
// number of lane changes, or -1 when no lane works.
int Traffic::good_direction(const Vehicle &v, int32_t lane, int &steps) const {
	const Network &n = *net_;
	steps = 0;
	const bool here = lane_is_good(v, lane, v.ri);
	if (here && !n.lanes[static_cast<size_t>(lane)].merge_end) return 0;
	int best_dir = 0, best = -1;
	for (int side = 0; side < 2; ++side) {
		int32_t cur = lane;
		for (int k = 1; k < 12; ++k) {
			cur = side == 0 ? n.lanes[static_cast<size_t>(cur)].left : n.lanes[static_cast<size_t>(cur)].right;
			if (cur < 0) break;
			if (lane_is_good(v, cur, v.ri) && !n.lanes[static_cast<size_t>(cur)].merge_end) {
				if (best < 0 || k < best) {
					best = k;
					best_dir = side == 0 ? -1 : 1;
				}
				break;
			}
		}
	}
	if (best < 0) {
		steps = here ? 0 : -1;
		return 0;
	}
	steps = best;
	return best_dir;
}

void Traffic::estimate_routes() {
	if (!net_) return;
	const Network &n = *net_;
	auto free_time = [&](int32_t l) {
		const NetLane &x = n.lanes[static_cast<size_t>(l)];
		return x.length / std::max(1.0, std::min(x.speed_limit, typical_driver(VehicleKind::Bus).max_speed));
	};
	auto leg_time = [&](int32_t start, const std::vector<int32_t> &route) {
		double t = free_time(start);
		for (int32_t c : route) t += free_time(c) + free_time(n.lanes[static_cast<size_t>(c)].to);
		return t;
	};
	for (RouteRun &rr : routes_) {
		rr.round_trip = 0.0;
		const NetDepot &d = n.depots[rr.depot_idx];
		const NetRoute &r = d.routes[rr.route_idx];
		if (r.stops.empty() || d.spawn_lanes.empty()) continue;
		std::vector<int32_t> seq = r.stops;
		if (r.loop && seq.size() > 1) seq.push_back(seq.front());
		int32_t cur = d.spawn_lanes[0];
		double cur_s = 0.0, total = 0.0;
		bool ok = true;
		std::vector<int32_t> route;
		for (int32_t si : seq) {
			const NetStop &st = n.stops[static_cast<size_t>(si)];
			if (!find_route_to_lane(cur, cur_s, st.lane, st.s, route, VehicleKind::Bus, true)) {
				ok = false;
				break;
			}
			total += leg_time(cur, route) + config_.bus_dwell;
			cur = st.lane;
			cur_s = st.s;
		}
		if (ok && find_route(cur, d.node, route, true, VehicleKind::Bus)) {
			total += leg_time(cur, route);
			rr.round_trip = total;
		}
	}
}

// --- Car following ----------------------------------------------------------------------

double Traffic::desired_speed(const Vehicle &v, int32_t lane) const {
	const NetLane &l = net_->lanes[static_cast<size_t>(lane)];
	const double base = is_road(l) ? l.speed_limit * v.drv.speed_factor : l.speed_limit * std::min(1.0, v.drv.speed_factor);
	return std::min(base, v.drv.max_speed);
}

double Traffic::idm(const Vehicle &v, double v0, bool has, double gap, double lead_v) const {
	const DriverParams2 &p = v.drv;
	const double vv = v.v;
	if (v0 < 0.1) return vv > 0.0 ? -p.b : 0.0; // wants to stand still
	const double r = vv / (v0 > 0.1 ? v0 : 0.1);
	const double r2 = r * r;
	const double free_term = 1.0 - r2 * r2;
	if (!has) return p.a * free_term;
	const double dv = vv - lead_v;
	const double dyn = vv * p.T + vv * dv / p.two_sqrt_ab;
	const double s_star = p.s0 + (dyn > 0.0 ? dyn : 0.0);
	const double g = gap > 0.01 ? gap : 0.01;
	const double q = s_star / g;
	return p.a * (free_term - q * q);
}

void Traffic::leader(const Vehicle &v, int32_t lane, double s, size_t ri, bool &has, double &gap, double &lead_v,
		double &v0_cap, VehicleId &who) const {
	const Network &n = *net_;
	has = false;
	gap = kInf;
	lead_v = 0.0;
	v0_cap = kInf;
	who = kNoId;
	auto take = [&](double g, double lv, VehicleId id) {
		if (g < gap) {
			gap = g;
			lead_v = lv;
			has = true;
			who = id;
		}
	};
	// A waypoint ahead (bus stop, parking bay): stop there as if at a line.
	const Waypoint *wp = pending_waypoint(v);
	auto wp_on = [&](int32_t cur, size_t r) { return wp && r >= v.route.size() && cur == wp->lane; };
	if (wp_on(lane, ri) && wp->s + 0.5 >= s) take(wp->s - s + v.drv.s0, 0.0, kNoId);
	// People on (or about to step onto) a crossing over a lane at `base` metres
	// ahead of the lane start, measured from the car's front.
	const bool peds = crossings_live_ && n.ped.lane_crossings.size() == n.lanes.size();
	double clear_d0 = kInf, clear_d1 = kInf; // the nearest crossing ahead, to keep clear
	auto crossings_on = [&](int32_t ln, double base) {
		for (int32_t ci : n.ped.lane_crossings[static_cast<size_t>(ln)]) {
			for (const CrossingSpan &sp : n.ped.crossings[static_cast<size_t>(ci)].spans) {
				if (sp.lane != ln) continue;
				const double d = base + sp.s0;
				if (d < 0.0) continue; // already over the line
				if (d < clear_d0) {
					clear_d0 = d;
					clear_d1 = base + sp.s1;
				}
				const int b = crossing_blocks(ci, ln, v);
				if (b == 2 && d < v.v * v.v / (2.0 * v.drv.b) + 1.0) continue; // can't stop comfortably: goes on
				if (b != 0) take(d - 0.5 + v.drv.s0, 0.0, kPedestrians);
			}
		}
	};
	if (peds) crossings_on(lane, -s);
	[&]() {
		// Car ahead on the same lane.
		const std::vector<int32_t> &list = cars_[static_cast<size_t>(lane)];
		int32_t ahead = -1;
		if (lane == v.lane && s == v.s && !v.off_lane) {
			if (v.list_pos > 0) ahead = list[static_cast<size_t>(v.list_pos - 1)];
		} else {
			for (int32_t k : list) {
				const Vehicle &o = veh_[static_cast<size_t>(k)];
				if (o.id == v.id) continue;
				if (o.s > s || (o.s == s && o.id < v.id)) ahead = k;
				else break;
			}
		}
		if (ahead >= 0) {
			const Vehicle &o = veh_[static_cast<size_t>(ahead)];
			take(o.s - o.drv.length - s, o.v, o.id);
			return;
		}
		if (has) return;
		double dist = n.lanes[static_cast<size_t>(lane)].length - s;
		int32_t cur = lane;
		size_t r = ri;
		for (int hop = 0; hop < 16 && dist < config_.lookahead; ++hop) {
			const NetLane &l = n.lanes[static_cast<size_t>(cur)];
			int32_t nx;
			if (is_road(l)) {
				if (at_route_end(v, cur, r)) return; // drives off the map
				nx = next_connector(v, cur, r);
				if (nx < 0) {
					take(dist - 0.5 + v.drv.s0, 0.0, kNoId); // end of lane: stop at the line
					return;
				}
				const NetLane &c = n.lanes[static_cast<size_t>(nx)];
				if (c.junction >= 0 && n.junctions[static_cast<size_t>(c.junction)].arbitrated && v.grant != nx) {
					take(dist - 0.5 + v.drv.s0, 0.0, kNoId); // no grant: stop at the line
					return;
				}
				// Cars that just turned off into another connector from this lane.
				for (int32_t sib : l.next) {
					if (sib == nx || cars_[static_cast<size_t>(sib)].empty()) continue;
					const Vehicle &o = veh_[static_cast<size_t>(cars_[static_cast<size_t>(sib)].back())];
					const double rear = o.s - o.drv.length;
					if (rear < 1.0 && o.id != v.id) take(dist + rear, o.v, o.id);
				}
				++r;
			} else {
				nx = l.next.empty() ? -1 : l.next[0];
				if (nx < 0) return;
				if (wp_on(nx, r)) take(dist + wp->s + v.drv.s0, 0.0, kNoId);
			}
			const double lim = desired_speed(v, nx);
			v0_cap = std::min(v0_cap, std::sqrt(lim * lim + 2.0 * v.drv.b * (dist > 0.0 ? dist : 0.0)));
			if (peds) crossings_on(nx, dist);
			const std::vector<int32_t> &nl = cars_[static_cast<size_t>(nx)];
			for (size_t k = nl.size(); k-- > 0;) {
				const Vehicle &o = veh_[static_cast<size_t>(nl[k])];
				if (o.id == v.id) continue;
				take(dist + o.s - o.drv.length, o.v, o.id);
				return;
			}
			if (has) return;
			dist += n.lanes[static_cast<size_t>(nx)].length;
			cur = nx;
		}
	}();
	// Don't stop on a crossing: with a slow car ahead and no room after it, wait before it.
	if (clear_d0 < kInf && has && who != kNoId && who != kPedestrians && lead_v < 3.0 && gap > clear_d0 &&
			gap < clear_d1 + v.drv.length + v.drv.s0 && clear_d0 > v.v * v.v / (2.0 * v.drv.b)) {
		take(clear_d0 - 0.5 + v.drv.s0, 0.0, kPedestrians);
	}
}

double Traffic::accel_at(size_t i, int32_t lane, double s) const {
	const Vehicle &v = veh_[i];
	bool has = false;
	double gap = 0.0, lv = 0.0, cap = 0.0;
	VehicleId who = kNoId;
	leader(v, lane, s, v.ri, has, gap, lv, cap, who);
	return idm(v, std::min(desired_speed(v, lane), cap), has, gap, lv);
}

// --- Junctions -----------------------------------------------------------------------------

double Traffic::pos_on(const Vehicle &v, int32_t conn) const {
	const NetLane &c = net_->lanes[static_cast<size_t>(conn)];
	if (v.lane == conn) return v.s;
	if (v.lane == c.to) return c.length + v.s;
	if (v.lane == c.from) return v.s - net_->lanes[static_cast<size_t>(c.from)].length;
	return kInf;
}

double Traffic::time_to(const Vehicle &v, double dist) const {
	if (dist <= 0.0) return 0.0;
	if (v.v > 2.0) return dist / v.v;
	const double a = v.drv.a;
	return (std::sqrt(v.v * v.v + 2.0 * a * dist) - v.v) / a;
}

bool Traffic::box_clear(const NetJunction &j, int32_t conn, const std::vector<int32_t> &granted,
		VehicleId &blocker) const {
	(void)j;
	const NetLane &x = net_->lanes[static_cast<size_t>(conn)];
	for (const Conflict &cf : x.conflicts) {
		for (int32_t k : granted) {
			const Vehicle &o = veh_[static_cast<size_t>(k)];
			if (o.grant != cf.other && o.held != cf.other) continue;
			if (pos_on(o, cf.other) - o.drv.length < cf.s_other + config_.box_margin) {
				blocker = o.id;
				return false;
			}
		}
	}
	return true;
}

bool Traffic::exit_clear(int32_t conn, int32_t veh, const std::vector<int32_t> &granted) const {
	const Network &n = *net_;
	const int32_t to = n.lanes[static_cast<size_t>(conn)].to;
	const NetLane &t = n.lanes[static_cast<size_t>(to)];
	const std::vector<int32_t> &list = cars_[static_cast<size_t>(to)];
	double free = t.length;
	if (!list.empty()) {
		const Vehicle &rear = veh_[static_cast<size_t>(list.back())];
		free = rear.s - rear.drv.length;
		if (rear.v > 5.0) free += rear.v; // it is moving away quickly
	}
	bool others = false;
	for (int32_t k : granted) {
		if (k == veh) continue;
		const Vehicle &o = veh_[static_cast<size_t>(k)];
		if (o.lane == to) continue;
		for (int32_t c : { o.grant, o.held }) {
			if (c >= 0 && n.lanes[static_cast<size_t>(c)].to == to) {
				free -= o.drv.length + 2.0;
				others = true;
				break;
			}
		}
	}
	const Vehicle &me = veh_[static_cast<size_t>(veh)];
	// Room for the whole car past the box, so it can't stop with its tail inside.
	const double need = me.drv.length + me.drv.s0 + config_.box_margin;
	if (t.length < need) return list.empty() && !others; // very short road: one car at a time
	return free >= need;
}

SignalLight Traffic::light_of(int32_t conn) const {
	const NetLane &c = net_->lanes[static_cast<size_t>(conn)];
	if (c.junction < 0 || c.movement < 0) return SignalLight::Green;
	const NetJunction &j = net_->junctions[static_cast<size_t>(c.junction)];
	if (!j.signal.enabled) return SignalLight::Green;
	return j.light(c.movement, static_cast<int64_t>(tick_));
}

namespace {
int32_t slot_weight(const Vehicle &v) { return 1 + static_cast<int32_t>((v.drv.length + v.drv.s0) / 6.5 - 0.5); }
} // namespace

std::vector<int32_t> Traffic::ring_load(size_t j) const {
	const Network &n = *net_;
	std::vector<int32_t> load(ring_slots_[j].size(), 0);
	for (int32_t l : ring_lanes_[j]) {
		for (int32_t k : cars_[static_cast<size_t>(l)]) load[static_cast<size_t>(ring_cycle_[static_cast<size_t>(l)])] += slot_weight(veh_[static_cast<size_t>(k)]);
	}
	for (int32_t c : n.junctions[j].connectors) {
		const NetLane &cn = n.lanes[static_cast<size_t>(c)];
		const int8_t cyc = ring_cycle_[static_cast<size_t>(cn.to)];
		if (cyc < 0) continue;
		for (int32_t k : cars_[static_cast<size_t>(c)]) load[static_cast<size_t>(cyc)] += slot_weight(veh_[static_cast<size_t>(k)]);
		// Granted entries still before the line.
		if (ring_cycle_[static_cast<size_t>(cn.from)] >= 0) continue;
		for (int32_t k : cars_[static_cast<size_t>(cn.from)]) {
			const Vehicle &o = veh_[static_cast<size_t>(k)];
			if (o.grant == c) load[static_cast<size_t>(cyc)] += slot_weight(o);
		}
	}
	return load;
}

void Traffic::arbitrate() {
	const Network &n = *net_;
	const size_t nj = n.junctions.size();
	std::vector<std::vector<int32_t>> granted(nj);
	for (size_t i = 0; i < veh_.size(); ++i) {
		const Vehicle &v = veh_[i];
		int32_t jg = -1;
		if (v.grant >= 0) {
			jg = n.lanes[static_cast<size_t>(v.grant)].junction;
			granted[static_cast<size_t>(jg)].push_back(static_cast<int32_t>(i));
		}
		if (v.held >= 0) {
			const int32_t jh = n.lanes[static_cast<size_t>(v.held)].junction;
			if (jh != jg) granted[static_cast<size_t>(jh)].push_back(static_cast<int32_t>(i));
		}
	}
	std::vector<Candidate> cands;
	std::vector<SignalLight> lights;
	for (size_t j = 0; j < nj; ++j) {
		const NetJunction &junc = n.junctions[j];
		if (!junc.arbitrated) continue;
		const bool sig = junc.signal.enabled;
		const int phase = sig ? junc.phase_at(static_cast<int64_t>(tick_)) : -1;
		// Right turn on red after stopping (EU flashing green arrow).
		auto ror_ok = [&](int32_t conn) {
			const NetLane &c = n.lanes[static_cast<size_t>(conn)];
			return sig && c.turn == TurnKind::Right && c.movement >= 0 &&
					static_cast<size_t>(c.movement) < junc.signal.right_on_red.size() &&
					junc.signal.right_on_red[static_cast<size_t>(c.movement)] != 0;
		};
		auto permissive = [&](int32_t conn) {
			const int32_t mv = n.lanes[static_cast<size_t>(conn)].movement;
			return phase >= 0 && mv >= 0 && junc.signal.state[static_cast<size_t>(phase)][static_cast<size_t>(mv)] == 2;
		};
		std::vector<int32_t> &box = granted[j];
		for (int32_t k : box) {
			Vehicle &o = veh_[static_cast<size_t>(k)];
			if (o.grant < 0 || n.lanes[static_cast<size_t>(o.grant)].junction != static_cast<int32_t>(j)) continue;
			if (o.lane != n.lanes[static_cast<size_t>(o.grant)].from) continue;
			const double d = n.lanes[static_cast<size_t>(o.lane)].length - o.s;
			if (sig) {
				// The light changed before it reached the line: stop if it still can.
				const SignalLight L = light_of(o.grant);
				const double stop = o.v * o.v / (2.0 * o.drv.b) + 1.0;
				bool revoke = false;
				if (L == SignalLight::Red && !ror_ok(o.grant)) revoke = d > stop || o.v < 0.5;
				else if (L == SignalLight::Amber) revoke = d > stop || (o.v < 0.5 && !permissive(o.grant));
				if (revoke) {
					o.grant = -1;
					o.state = VehicleState::RedLight;
					continue;
				}
			}
			// Give back grants of cars that stopped before the line (their exit filled up).
			if (o.v < 0.3 && d < 8.0 && !exit_clear(o.grant, k, box)) {
				o.grant = -1;
				o.state = VehicleState::ExitBlocked;
			}
		}
		box.erase(std::remove_if(box.begin(), box.end(),
						  [&](int32_t k) {
							  const Vehicle &o = veh_[static_cast<size_t>(k)];
							  const bool g = o.grant >= 0 && n.lanes[static_cast<size_t>(o.grant)].junction == static_cast<int32_t>(j);
							  const bool h = o.held >= 0 && n.lanes[static_cast<size_t>(o.held)].junction == static_cast<int32_t>(j);
							  return !g && !h;
						  }),
				box.end());

		// Candidates: the first car without a grant on each approach lane.
		cands.clear();
		for (int32_t a : junc.approaches) {
			for (int32_t k : cars_[static_cast<size_t>(a)]) {
				Vehicle &o = veh_[static_cast<size_t>(k)];
				if (o.grant >= 0) continue; // granted cars ahead of it
				const double d = n.lanes[static_cast<size_t>(a)].length - o.s;
				if (d <= 150.0 && o.phase == 0) {
					const int32_t c = next_connector(o, a, o.ri);
					if (c >= 0) cands.push_back(Candidate{ k, c, d });
				}
				break;
			}
		}
		bool any_waiting = false;
		bool could_go = false; // someone waits although its exit has room
		for (const Candidate &c : cands) {
			Vehicle &o = veh_[static_cast<size_t>(c.veh)];
			const SignalLight L = sig ? light_of(c.conn) : SignalLight::Green;
			const bool held_by_light = L == SignalLight::Red || L == SignalLight::Amber;
			if (c.dist < 6.0 && o.v < 0.5) {
				if (o.wait_since == 0) {
					o.wait_since = tick_ + 1;
					if (L == SignalLight::Red) ++stats_.red_light_waits;
				}
				if (!held_by_light) {
					any_waiting = true;
					could_go = could_go || exit_clear(c.conn, c.veh, box);
				}
			}
			if (o.stopped_tick == 0 && c.dist < 4.0) {
				// All-way stop: a stop at the line; right on red: a full stop.
				if ((junc.control == JunctionControl::AllWayStop && o.v < 0.2) ||
						(L == SignalLight::Red && ror_ok(c.conn) && o.v < 0.02)) {
					o.stopped_tick = tick_ + 1;
				}
			}
		}
		std::sort(cands.begin(), cands.end(), [&](const Candidate &a, const Candidate &b) {
			const Vehicle &va = veh_[static_cast<size_t>(a.veh)];
			const Vehicle &vb = veh_[static_cast<size_t>(b.veh)];
			const uint64_t wa = va.wait_since ? va.wait_since : kNever;
			const uint64_t wb = vb.wait_since ? vb.wait_since : kNever;
			return wa != wb ? wa < wb : va.id < vb.id;
		});
		lights.assign(cands.size(), SignalLight::Green);
		if (sig) {
			for (size_t q = 0; q < cands.size(); ++q) lights[q] = light_of(cands[q].conn);
		}
		auto conflicts = [&](int32_t a, int32_t b) {
			for (const Conflict &cf : n.lanes[static_cast<size_t>(a)].conflicts) {
				if (cf.other == b) return true;
			}
			return false;
		};
		auto grant = [&](const Candidate &c) {
			Vehicle &o = veh_[static_cast<size_t>(c.veh)];
			o.grant = c.conn;
			o.wait_since = 0;
			o.stopped_tick = 0;
			o.state = VehicleState::Driving;
			o.blocker = kNoId;
			box.push_back(c.veh);
			junction_last_grant_[j] = tick_;
		};
		// Roundabouts: an entry needs a free slot on the ring beyond its own, so
		// the ring can always turn (it can't lock itself up with circulating cars).
		std::vector<int32_t> ring_free;
		if (junc.roundabout) {
			ring_free = ring_load(j);
			for (size_t k = 0; k < ring_free.size(); ++k) ring_free[k] = ring_slots_[j][k] - ring_free[k];
		}
		auto ring_room = [&](int32_t conn, const Vehicle &o) {
			const NetLane &cn = n.lanes[static_cast<size_t>(conn)];
			const int8_t cyc = ring_cycle_[static_cast<size_t>(cn.to)];
			if (!junc.roundabout || cyc < 0 || ring_cycle_[static_cast<size_t>(cn.from)] >= 0) return true;
			return ring_free[static_cast<size_t>(cyc)] >= slot_weight(o);
		};
		auto ring_take = [&](int32_t conn, const Vehicle &o) {
			const NetLane &cn = n.lanes[static_cast<size_t>(conn)];
			const int8_t cyc = ring_cycle_[static_cast<size_t>(cn.to)];
			if (junc.roundabout && cyc >= 0 && ring_cycle_[static_cast<size_t>(cn.from)] < 0) {
				ring_free[static_cast<size_t>(cyc)] -= slot_weight(o);
			}
		};
		bool granted_now = false;
		for (size_t ci = 0; ci < cands.size(); ++ci) {
			const Candidate &c = cands[ci];
			Vehicle &o = veh_[static_cast<size_t>(c.veh)];
			const double reach = o.v * o.v / (2.0 * o.drv.b) + 0.5 * o.v + 6.0;
			if (c.dist > reach) {
				o.state = VehicleState::Approaching;
				continue;
			}
			const SignalLight L = lights[ci];
			bool on_red = false;
			if (L == SignalLight::Red) {
				if (!ror_ok(c.conn) || o.stopped_tick == 0) {
					o.state = VehicleState::RedLight;
					continue;
				}
				on_red = true;
			} else if (L == SignalLight::Amber) {
				// Goes only if it can no longer stop comfortably, or to clear a waiting left turn.
				const bool committed = o.v >= 0.5 && o.v * o.v / (2.0 * o.drv.b) > c.dist - 1.0;
				const bool clearing = o.v < 0.5 && c.dist < 6.0 && permissive(c.conn);
				if (!committed && !clearing) {
					o.state = VehicleState::RedLight;
					continue;
				}
			}
			if (!ring_room(c.conn, o)) {
				o.state = VehicleState::Yielding;
				continue;
			}
			VehicleId blk = kNoId;
			if (!box_clear(junc, c.conn, box, blk)) {
				o.state = VehicleState::BoxBlocked;
				o.blocker = blk;
				continue;
			}
			if (!exit_clear(c.conn, c.veh, box)) {
				o.state = VehicleState::ExitBlocked;
				continue;
			}
			bool go = true;
			if (junc.control == JunctionControl::AllWayStop && !n.lanes[static_cast<size_t>(c.conn)].conflicts.empty()) {
				if (o.stopped_tick == 0) {
					o.state = VehicleState::StopSign;
					continue;
				}
				for (const Candidate &q : cands) {
					if (q.veh == c.veh) continue;
					const Vehicle &p = veh_[static_cast<size_t>(q.veh)];
					if (p.stopped_tick == 0 || p.stopped_tick > o.stopped_tick ||
							(p.stopped_tick == o.stopped_tick && p.id > o.id)) {
						continue;
					}
					if (!conflicts(c.conn, q.conn) || !exit_clear(q.conn, q.veh, box)) continue;
					go = false;
					blk = p.id;
					break;
				}
				if (!go) {
					o.state = VehicleState::StopSign;
					o.blocker = blk;
					continue;
				}
			} else {
				const NetLane &x = n.lanes[static_cast<size_t>(c.conn)];
				const double waited = o.wait_since ? static_cast<double>(tick_ + 1 - o.wait_since) * config_.dt : 0.0;
				const bool impatient = !sig && waited >= config_.impatience;
				for (const Conflict &cf : x.conflicts) {
					for (size_t qi = 0; qi < cands.size(); ++qi) {
						const Candidate &q = cands[qi];
						if (q.conn != cf.other || q.veh == c.veh) continue;
						const Vehicle &p = veh_[static_cast<size_t>(q.veh)];
						int8_t prio = cf.priority;
						if (sig) {
							// Only movements that may go count; protected greens go before permissive ones.
							const SignalLight lq = lights[qi];
							if (lq == SignalLight::Red) continue; // right-on-red cars yield to everyone
							if (lq == SignalLight::Amber && p.v * p.v / (2.0 * p.drv.b) <= q.dist - 1.0) continue;
							if (on_red || lq == SignalLight::Amber) prio = -1;
							else if (L == SignalLight::Green && lq == SignalLight::GreenYield) prio = 1;
							else if (L == SignalLight::GreenYield && lq == SignalLight::Green) prio = -1;
						} else {
							const double p_waited =
									p.wait_since ? static_cast<double>(tick_ + 1 - p.wait_since) * config_.dt : 0.0;
							if (p_waited >= config_.impatience && p_waited > waited) {
								// A driver who has waited long gets let in: no new conflicting
								// grants until it has gone (unless its own exit is full, for a while).
								if (ring_room(q.conn, p) && (p_waited >= 2.0 * config_.impatience || exit_clear(q.conn, q.veh, box))) {
									go = false;
									blk = p.id;
									break;
								}
							}
						}
						if (prio >= 0) continue;
						if (impatient) {
							// Goes unless the car with priority could no longer stop comfortably.
							const double stop = p.v * p.v / (2.0 * p.drv.b) + 5.0;
							if (q.dist > stop || p.v < 0.5) continue;
							go = false;
							blk = p.id;
							break;
						}
						if (p.v < 0.5 && q.dist < 6.0) {
							// Waiting at its line: it keeps its right of way only if it could go.
							VehicleId dummy = kNoId;
							if (!box_clear(junc, q.conn, box, dummy) || !exit_clear(q.conn, q.veh, box)) continue;
						}
						const double tp = time_to(p, q.dist + cf.s_other);
						const double clear = time_to(o, c.dist + cf.s_self + o.drv.length + config_.box_margin);
						const double gap = o.v < 2.0 ? o.drv.critical_gap : 0.6 * o.drv.critical_gap;
						if (tp < std::max(gap, clear + 1.0)) {
							go = false;
							blk = p.id;
							break;
						}
					}
					if (!go) break;
				}
				if (!go) {
					o.state = VehicleState::Yielding;
					o.blocker = blk;
					continue;
				}
			}
			grant(c);
			ring_take(c.conn, o);
			if (on_red) ++stats_.right_on_red;
			granted_now = true;
		}
		// Deadlock breaker: everyone is waiting for someone and the box is empty.
		if (!granted_now && any_waiting && box.empty()) {
			for (size_t ci = 0; ci < cands.size(); ++ci) {
				const Candidate &c = cands[ci];
				const Vehicle &o = veh_[static_cast<size_t>(c.veh)];
				if (lights[ci] != SignalLight::Green && lights[ci] != SignalLight::GreenYield) continue;
				if (o.wait_since == 0 || static_cast<double>(tick_ + 1 - o.wait_since) * config_.dt < 3.0) continue;
				if (junc.control == JunctionControl::AllWayStop && o.stopped_tick == 0) continue;
				if (!exit_clear(c.conn, c.veh, box)) continue;
				if (!ring_room(c.conn, o)) continue;
				grant(c);
				ring_take(c.conn, o);
				++stats_.forced_grants;
				break;
			}
		}
		if (!could_go) junction_last_grant_[j] = tick_;
		junction_waiting_[j] = could_go ? 1 : 0;
	}
}

// --- Lane changes ----------------------------------------------------------------------------

void Traffic::change_lanes() {
	const Network &n = *net_;
	for (Vehicle &v : veh_) v.courtesy = -1;
	struct Request {
		int32_t veh;
		int32_t lane;
		double s;
	};
	std::vector<Request> requests;
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle &v = veh_[i];
		v.merge_lane = -1;
		if (v.off_lane || v.phase != 0) continue;
		const NetLane &l = n.lanes[static_cast<size_t>(v.lane)];
		if (!is_road(l)) continue;
		if (v.grant >= 0) continue; // committed to its junction entry
		if (l.ring && v.v < 0.5 && v.stopped_for > 4.0 && l.length - v.s < 3.0) {
			// Stuck behind a full piece of the ring: leave at this exit if it has room.
			bool left_ring = false;
			for (int32_t c : l.next) {
				const NetLane &cn = n.lanes[static_cast<size_t>(c)];
				if (n.lanes[static_cast<size_t>(cn.to)].ring || !allowed(v.kind, cn)) continue;
				if (v.ri < v.route.size() && v.route[v.ri] == c) break; // already its way out
				const NetLane &out = n.lanes[static_cast<size_t>(cn.to)];
				if (!cars_[static_cast<size_t>(cn.to)].empty()) {
					const Vehicle &rear = veh_[static_cast<size_t>(cars_[static_cast<size_t>(cn.to)].back())];
					if (rear.s - rear.drv.length < v.drv.length + v.drv.s0 + config_.box_margin + 4.0) continue;
				}
				Vehicle probe = v;
				probe.lane = cn.to;
				probe.s = 0.0;
				probe.route.clear();
				probe.ri = 0;
				if (!reroute(probe, true)) continue;
				(void)out;
				v.route.assign(1, c);
				v.route.insert(v.route.end(), probe.route.begin(), probe.route.end());
				v.ri = 0;
				left_ring = true;
				break;
			}
			if (left_ring) continue;
		}
		if (v.last_change != 0 && tick_ + 1 < v.last_change + 30) continue;
		// Where the lane has to be right: its end, or a waypoint on this road.
		double target_s = l.length;
		const Waypoint *wp = v.ri >= v.route.size() ? pending_waypoint(v) : nullptr;
		if (wp) {
			const NetLane &wl = n.lanes[static_cast<size_t>(wp->lane)];
			if (wp->lane == v.lane) {
				if (wp->s - v.s < 150.0) continue; // stay in the lane for the stop
			} else if (wl.segment == l.segment && wl.dir == l.dir) {
				const double m = n.map_across(wp->lane, wp->s, v.lane);
				target_s = m >= 0.0 ? m : wp->s;
			}
		}
		int steps = 0;
		const int want = good_direction(v, v.lane, steps);
		const double remaining = target_s - v.s;
		if (steps < 0) {
			// No lane of this road continues the route: plan again from here.
			if ((tick_ + v.id) % 10 == 0 && !reroute(v, false)) reroute(v, true);
			continue;
		}
		const bool mandatory = want != 0 && remaining < 100.0 * steps + 80.0;
		if (mandatory) {
			// Past the last place where the change is allowed: take another way.
			bool ahead = false;
			for (const auto &r : want < 0 ? l.change_left : l.change_right) ahead |= r.second > v.s + 1.0;
			if (!ahead) {
				if (reroute(v, false)) continue;
			}
		}
		const bool car_in_bus_lane = v.kind == VehicleKind::Car && l.type == LaneType::Bus;
		if (!mandatory && !car_in_bus_lane && (tick_ + v.id) % 5 != 0) continue;

		const bool here_good = lane_is_good(v, v.lane, v.ri);
		int best_side = -1;
		double best_gain = 0.2;
		double best_s = 0.0;
		int32_t toward_lane = -1;
		double toward_s = 0.0;
		for (int side = 0; side < 2; ++side) {
			const bool left = side == 0;
			const int32_t t = left ? l.left : l.right;
			if (t < 0 || !n.can_change(v.lane, left, v.s)) continue;
			const double st = n.map_across(v.lane, v.s, t);
			if (st < 0.0) continue;
			const NetLane &tl = n.lanes[static_cast<size_t>(t)];
			if (!allowed(v.kind, tl)) continue;
			if (tl.ring) {
				// Only into a circulating lane that keeps a free slot.
				const int32_t j = n.junction_at(tl.end_node);
				if (j >= 0) {
					const int8_t cyc = ring_cycle_[static_cast<size_t>(t)];
					const std::vector<int32_t> load = ring_load(static_cast<size_t>(j));
					if (load[static_cast<size_t>(cyc)] + slot_weight(v) > ring_slots_[static_cast<size_t>(j)][static_cast<size_t>(cyc)]) {
						continue;
					}
				}
			}
			const bool toward = want == (left ? -1 : 1);
			if (tl.type == LaneType::Bus && v.kind == VehicleKind::Car) {
				// Cars only enter a bus lane to turn off it soon.
				if (!toward || tl.length - st > config_.bus_lane_zone) continue;
			} else if (tl.type == LaneType::Bus && v.kind == VehicleKind::Bike && !toward) {
				continue;
			}
			if (toward) {
				toward_lane = t;
				toward_s = st;
			}
			if (mandatory && !toward) continue;
			// New leader and follower in the target lane.
			const std::vector<int32_t> &tlist = cars_[static_cast<size_t>(t)];
			int32_t lead = -1, follow = -1;
			for (int32_t k : tlist) {
				const Vehicle &o = veh_[static_cast<size_t>(k)];
				if (o.s > st || (o.s == st && o.id < v.id)) {
					lead = k;
				} else {
					follow = k;
					break;
				}
			}
			double gap_front = kInf;
			if (lead >= 0) {
				const Vehicle &o = veh_[static_cast<size_t>(lead)];
				gap_front = o.s - o.drv.length - st;
			}
			double gap_back = kInf;
			const Vehicle *fv = nullptr;
			if (follow >= 0) {
				fv = &veh_[static_cast<size_t>(follow)];
				gap_back = st - v.drv.length - fv->s;
			} else {
				// A car about to come off a connector into the target lane.
				for (int32_t c : tl.prev) {
					if (cars_[static_cast<size_t>(c)].empty()) continue;
					const Vehicle &o = veh_[static_cast<size_t>(cars_[static_cast<size_t>(c)].front())];
					const double g = st - v.drv.length + (n.lanes[static_cast<size_t>(c)].length - o.s);
					if (g < gap_back) {
						gap_back = g;
						fv = &o;
					}
				}
			}
			if (gap_front < 1.0 || gap_back < 1.0) continue;
			double a_follow_new = 0.0, a_follow_old = 0.0;
			if (fv) {
				a_follow_new = idm(*fv, desired_speed(*fv, t), true, gap_back, v.v);
				a_follow_old = fv->acc;
				const double safe = mandatory ? -4.5 : -3.0;
				if (a_follow_new < safe) continue;
			}
			// It must be able to follow its new leader without braking hard itself.
			const double a_self_new = accel_at(i, t, st);
			if (a_self_new < (mandatory ? -4.5 : -3.0)) continue;
			double gain;
			if (mandatory) {
				gain = 1e6;
			} else {
				gain = a_self_new - v.acc + v.drv.politeness * (a_follow_new - a_follow_old);
				gain += left ? -0.1 : 0.1; // keep right unless overtaking
				if (toward) gain += 0.3;
				if (car_in_bus_lane && tl.type != LaneType::Bus) gain += 1.0; // get out of the bus lane
				if ((v.kind == VehicleKind::Bus || v.kind == VehicleKind::Coach) && tl.type == LaneType::Bus) gain += 0.5;
				if (here_good && !lane_is_good(v, t, v.ri)) {
					if (remaining < 250.0) continue;
					gain -= 0.5;
				}
			}
			if (gain > best_gain) {
				best_gain = gain;
				best_side = side;
				best_s = st;
			}
		}
		if (best_side >= 0) {
			const int32_t t = best_side == 0 ? l.left : l.right;
			const Vec2 from = n.pose(v.lane, v.s).pos;
			const Vec2 to = n.pose(t, best_s).pos;
			const double d = (to - from).length();
			list_remove(i);
			v.lane = t;
			v.s = best_s;
			v.lat = best_side == 0 ? d : -d;
			v.last_change = tick_ + 1;
			list_insert(i);
			++stats_.lane_changes;
		} else if (mandatory && toward_lane >= 0) {
			v.merge_lane = toward_lane;
			v.merge_s = toward_s;
			v.state = VehicleState::ChangingLane;
			requests.push_back({ static_cast<int32_t>(i), toward_lane, toward_s });
		}
		if (best_side < 0 && mandatory && l.ring && remaining < 6.0) {
			// No gap to the exit lane: go round once more.
			reroute(v, false);
			continue;
		}
		if (best_side < 0 && mandatory && remaining < 3.0 && v.v < 0.5 && v.stopped_for > 20.0) {
			// Waited too long for a gap: take any way out of this lane.
			if (pending_waypoint(v)) {
				skip_waypoint(v);
				if (reroute(v, false)) continue;
			}
			if (!reroute(v, false)) {
				for (const NetSpawner &sp : n.spawners) {
					if (!sp.config.sink || sp.node == v.dest) continue;
					const NodeId old = v.dest;
					v.dest = sp.node;
					if (reroute(v, false)) break;
					v.dest = old;
				}
			}
		}
	}
	// Courtesy: the nearest car behind in the target lane leaves a gap.
	for (const Request &r : requests) {
		for (int32_t k : cars_[static_cast<size_t>(r.lane)]) {
			Vehicle &o = veh_[static_cast<size_t>(k)];
			if (o.s >= r.s) continue;
			if (r.s - o.s < 40.0 && o.courtesy < 0) o.courtesy = r.veh;
			break;
		}
	}
}

void Traffic::list_insert(size_t i) {
	const Vehicle &v = veh_[i];
	std::vector<int32_t> &list = cars_[static_cast<size_t>(v.lane)];
	auto pos = std::find_if(list.begin(), list.end(), [&](int32_t k) {
		const Vehicle &o = veh_[static_cast<size_t>(k)];
		return o.s < v.s || (o.s == v.s && o.id > v.id);
	});
	const size_t at = static_cast<size_t>(pos - list.begin());
	list.insert(pos, static_cast<int32_t>(i));
	for (size_t k = at; k < list.size(); ++k) veh_[static_cast<size_t>(list[k])].list_pos = static_cast<int32_t>(k);
}

void Traffic::list_remove(size_t i) {
	std::vector<int32_t> &list = cars_[static_cast<size_t>(veh_[i].lane)];
	auto it = std::find(list.begin(), list.end(), static_cast<int32_t>(i));
	if (it == list.end()) return;
	const size_t at = static_cast<size_t>(it - list.begin());
	list.erase(it);
	for (size_t k = at; k < list.size(); ++k) veh_[static_cast<size_t>(list[k])].list_pos = static_cast<int32_t>(k);
}

// --- Movement --------------------------------------------------------------------------------

void Traffic::move() {
	const Network &n = *net_;
	const double dt = config_.dt;
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle &v = veh_[i];
		if (v.off_lane || v.phase != 0) {
			// Standing at a stop, parked or manoeuvring: not stuck.
			v.v = 0.0;
			v.acc = 0.0;
			v.stopped_for = 0.0;
			continue;
		}
		double a = v.acc + config_.accel_noise * rng_.symmetric();
		if (a < -config_.max_decel) a = -config_.max_decel;
		const double v_old = v.v;
		double vn = v_old + a * dt;
		double ds;
		if (vn < 0.0) {
			ds = a < 0.0 ? -0.5 * v_old * v_old / a : 0.0;
			vn = 0.0;
		} else {
			ds = v_old * dt + 0.5 * a * dt * dt;
		}
		v.v = vn;
		v.distance += ds;
		double s = v.s + ds;
		int32_t lane = v.lane;
		while (s >= n.lanes[static_cast<size_t>(lane)].length) {
			const NetLane &l = n.lanes[static_cast<size_t>(lane)];
			if (is_road(l)) {
				if (at_route_end(v, lane, v.ri)) {
					v.done = true;
					break;
				}
				const int32_t c = next_connector(v, lane, v.ri);
				const bool needs_grant = c >= 0 && n.lanes[static_cast<size_t>(c)].junction >= 0 &&
						n.junctions[static_cast<size_t>(n.lanes[static_cast<size_t>(c)].junction)].arbitrated;
				if (c < 0 || (needs_grant && v.grant != c)) {
					s = l.length;
					v.v = 0.0;
					break;
				}
				const double sample = static_cast<double>(tick_ + 1 - v.lane_tick) * dt;
				double &t = lane_time_[static_cast<size_t>(lane)];
				if (v.kind != VehicleKind::Bike) t = std::max(l.length / std::max(1.0, l.speed_limit), 0.9 * t + 0.1 * sample);
				if (needs_grant) {
					v.held = c;
					v.grant = -1;
				}
				s -= l.length;
				lane = c;
				++v.ri;
			} else {
				s -= l.length;
				lane = l.next[0];
				v.lane_tick = tick_ + 1;
			}
		}
		v.s = s;
		v.lane = lane;
		if (v.lat != 0.0) {
			const double step = 0.12;
			v.lat = v.lat > 0.0 ? std::max(0.0, v.lat - step) : std::min(0.0, v.lat + step);
		}
		if (v.held >= 0 && pos_on(v, v.held) - v.drv.length > n.lanes[static_cast<size_t>(v.held)].length) {
			v.held = -1;
		}
		if (v.kind == VehicleKind::Car && !v.done) {
			const NetLane &cl = n.lanes[static_cast<size_t>(v.lane)];
			if (cl.type == LaneType::Bus && is_road(cl) && cl.length - v.s > config_.bus_lane_zone) {
				stats_.bus_lane_misuse += dt;
			}
		}
		v.stopped_for = v.v < 0.1 ? v.stopped_for + dt : 0.0;
		if (v.stopped_for > config_.stuck_timeout && !v.done) {
			v.done = true;
			++stats_.removed_stuck;
			skip_waypoint(v); // gives back a reserved bay
		} else if (v.done) {
			++stats_.arrived;
			trip_time_sum_ += static_cast<double>(tick_ + 1 - v.spawn_tick) * dt;
			if (v.kind == VehicleKind::Bike) ++stats_.bikes_arrived;
			if (v.kind == VehicleKind::Bus && v.bus_route != 0) {
				++stats_.bus_runs;
				for (RouteRun &rr : routes_) {
					if (rr.id == v.bus_route) ++rr.runs;
				}
			}
		}
	}
}

// --- Waypoints -------------------------------------------------------------------------------

void Traffic::skip_waypoint(Vehicle &v) {
	const Waypoint *wp = pending_waypoint(v);
	if (!wp) return;
	if (wp->action == WaypointAction::Park) {
		if (wp->bay >= 0 && bay_use_[static_cast<size_t>(wp->bay)] == v.id) bay_use_[static_cast<size_t>(wp->bay)] = kNoId;
		if (v.phase == 0) ++stats_.parking_failed;
	}
	if (wp->action == WaypointAction::BayStop && wp->stop >= 0 && (v.phase == 2 || v.phase == 3)) {
		--stop_use_[static_cast<size_t>(wp->stop)];
	}
	++v.wi;
	v.phase = 0;
	v.off_lane = false;
}

void Traffic::begin_waypoint(size_t i) {
	const Network &n = *net_;
	Vehicle &v = veh_[i];
	Waypoint &wp = v.waypoints[v.wi];
	const uint64_t now = tick_ + 1;
	auto ticks = [&](double sec) { return static_cast<uint64_t>(std::llround(std::max(0.1, sec) / config_.dt)); };
	v.phase_start = now;
	v.v = 0.0;
	if (serves_people(v, wp)) serve_stop(v, wp.stop, wp); // dwell from boarding
	switch (wp.action) {
		case WaypointAction::KerbStop:
			v.phase = 1;
			v.phase_until = now + ticks(wp.dwell);
			break;
		case WaypointAction::BayStop: {
			const NetStop *st = wp.stop >= 0 ? &n.stops[static_cast<size_t>(wp.stop)] : nullptr;
			if (!st || stop_use_[static_cast<size_t>(wp.stop)] >= static_cast<uint32_t>(st->bays)) {
				// No lay-by free: serve the stop from the lane.
				v.phase = 1;
				v.phase_until = now + ticks(wp.dwell);
				break;
			}
			// The lowest free slot, counted back from the front of the lay-by.
			std::vector<char> used(static_cast<size_t>(st->bays), 0);
			for (const Vehicle &o : veh_) {
				const Waypoint *ow = pending_waypoint(o);
				if (&o == &v || !ow || ow->action != WaypointAction::BayStop || ow->stop != wp.stop) continue;
				if ((o.phase == 2 || o.phase == 3) && ow->bay >= 0 && ow->bay < st->bays) used[static_cast<size_t>(ow->bay)] = 1;
			}
			int slot = 0;
			while (slot < st->bays - 1 && used[static_cast<size_t>(slot)]) ++slot;
			wp.bay = slot;
			++stop_use_[static_cast<size_t>(wp.stop)];
			v.phase = 2;
			v.phase_until = now + ticks(kBayManoeuvre);
			v.bay_pos = st->pos - st->dir * (kStationSlot * slot);
			v.bay_dir = st->dir;
			break;
		}
		case WaypointAction::Park: {
			if (wp.bay < 0) {
				skip_waypoint(v);
				retarget(v);
				return;
			}
			const NetBay &b = n.bays[static_cast<size_t>(wp.bay)];
			v.phase = 2;
			v.phase_until = now + ticks(park_in_time(b.style));
			v.bay_pos = b.pos + b.dir * (0.5 * v.drv.length);
			v.bay_dir = b.dir;
			break;
		}
	}
	if (v.kind == VehicleKind::Bus && wp.stop >= 0) ++stats_.bus_stops_served;
	if (v.kind == VehicleKind::Coach && wp.stop >= 0 && wp.stop == n.main_station) ++stats_.coach_calls;
}

void Traffic::complete_waypoint(Vehicle &v) {
	const Waypoint &wp = v.waypoints[v.wi];
	if (wp.action == WaypointAction::Park) ++stats_.parkings;
	++v.wi;
	v.phase = 0;
	v.off_lane = false;
	v.route.clear();
	v.ri = 0;
	v.stopped_for = 0.0;
	v.lane_tick = tick_ + 1;
}

bool Traffic::waypoints() {
	const Network &n = *net_;
	const uint64_t now = tick_ + 1;
	bool removed = false;
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle &v = veh_[i];
		const Waypoint *wpp = pending_waypoint(v);
		if (!wpp) continue;
		const Waypoint wp = *wpp;
		switch (v.phase) {
			case 0: {
				if (v.ri < v.route.size() || v.grant >= 0) break;
				const NetLane &l = n.lanes[static_cast<size_t>(v.lane)];
				if (!is_road(l)) break;
				const NetLane &wl = n.lanes[static_cast<size_t>(wp.lane)];
				bool missed = false;
				if (v.lane == wp.lane) {
					if (v.s > wp.s + 1.0) missed = true;
					else if (v.s >= wp.s - v.drv.s0 - 2.0 && v.v < 0.3) begin_waypoint(i);
				} else if (wl.segment == l.segment && wl.dir == l.dir) {
					const double m = n.map_across(wp.lane, wp.s, v.lane);
					missed = v.s > (m >= 0.0 ? m : wp.s) + 1.0;
				}
				if (missed && !reroute(v, false)) {
					skip_waypoint(v);
					if (!retarget(v)) {
						v.done = true;
						++stats_.unroutable;
						removed = true;
					}
				}
				break;
			}
			case 1:
				if (serves_people(v, wp)) {
					// People arriving while it waits get on (and take their time doing so).
					Waypoint &cur = v.waypoints[v.wi];
					const int late = serve_stop(v, wp.stop, cur, true);
					if (late > 0) v.phase_until += boarding_ticks(v, late);
				}
				if (now >= v.phase_until) {
					if (serves_people(v, wp)) bus_departs(v, wp.stop);
					complete_waypoint(v);
					if (!retarget(v)) {
						v.done = true;
						++stats_.unroutable;
						removed = true;
					}
				}
				break;
			case 2:
				if (now >= v.phase_until) {
					// Now out of the lane: in the bay.
					list_remove(i);
					v.off_lane = true;
					v.grant = -1;
					v.held = -1;
					v.lat = 0.0;
					v.phase = 3;
					v.phase_start = now;
					v.phase_until = now + static_cast<uint64_t>(std::llround(std::max(0.1, wp.dwell) / config_.dt));
				}
				break;
			case 3:
				if (serves_people(v, wp) && wp.action == WaypointAction::BayStop) {
					Waypoint &cur = v.waypoints[v.wi];
					const int late = serve_stop(v, wp.stop, cur, true);
					if (late > 0) v.phase_until += boarding_ticks(v, late);
				}
				if (now >= v.phase_until) {
					// Back into the lane when there is a gap.
					const NetLane &wl = n.lanes[static_cast<size_t>(wp.lane)];
					const double s_back = std::clamp(wp.s - v.drv.s0, std::min(v.drv.length, wl.length), wl.length);
					if (!lane_free_at(wp.lane, s_back, v.drv.length)) break;
					double out = kBayManoeuvre;
					if (wp.action == WaypointAction::Park) {
						if (wp.bay >= 0) {
							out = park_out_time(n.bays[static_cast<size_t>(wp.bay)].style);
							if (bay_use_[static_cast<size_t>(wp.bay)] == v.id) bay_use_[static_cast<size_t>(wp.bay)] = kNoId;
						}
					} else if (wp.stop >= 0) {
						--stop_use_[static_cast<size_t>(wp.stop)];
						if (serves_people(v, wp)) bus_departs(v, wp.stop);
					}
					v.off_lane = false;
					v.lane = wp.lane;
					v.s = s_back;
					v.prev_lane = v.lane;
					v.prev_s = v.s;
					v.v = 0.0;
					v.phase = 4;
					v.phase_start = now;
					v.phase_until = now + static_cast<uint64_t>(std::llround(out / config_.dt));
					list_insert(i);
				}
				break;
			case 4:
				if (now >= v.phase_until) {
					complete_waypoint(v);
					if (!retarget(v)) {
						v.done = true;
						++stats_.unroutable;
						removed = true;
					}
				}
				break;
			default:
				break;
		}
	}
	return removed;
}

bool Traffic::lane_free_at(int32_t lane, double s, double length) const {
	const Network &n = *net_;
	for (int32_t k : cars_[static_cast<size_t>(lane)]) {
		const Vehicle &o = veh_[static_cast<size_t>(k)];
		const double rear = o.s - o.drv.length;
		if (rear > s + 1.0) continue; // ahead with room
		if (o.s < s - length) {
			// Behind: it must be able to stop.
			const double gap = (s - length) - o.s;
			if (gap < 3.0 + 2.5 * o.v) return false;
			continue;
		}
		return false; // overlaps
	}
	if (s - length < 15.0) {
		for (int32_t c : n.lanes[static_cast<size_t>(lane)].prev) {
			if (!cars_[static_cast<size_t>(c)].empty()) return false;
		}
	}
	return true;
}

// --- Demand ----------------------------------------------------------------------------------

VehicleId Traffic::insert(Vehicle v, int32_t lane, double s) {
	v.id = next_id_++;
	v.lane = lane;
	v.s = s;
	v.prev_lane = lane;
	v.prev_s = s;
	v.spawn_tick = tick_ + 1;
	v.lane_tick = tick_ + 1;
	veh_.push_back(std::move(v));
	list_insert(veh_.size() - 1);
	++stats_.spawned;
	return veh_.back().id;
}

NodeId Traffic::pick_dest(size_t k) {
	const Network &n = *net_;
	double total = 0.0;
	for (const auto &d : reach_[k]) total += d.second;
	if (total <= 0.0) return kNoId;
	double x = rng_.uniform() * total;
	size_t pick = reach_[k].back().first;
	for (const auto &d : reach_[k]) {
		if (x < d.second) {
			pick = d.first;
			break;
		}
		x -= d.second;
	}
	return n.spawners[pick].node;
}

// Entry speed behind the last car in the lane.
namespace {
double entry_speed(double v0, bool has_rear, double gap, double rear_v, const DriverParams2 &d) {
	if (!has_rear) return v0;
	return std::min(v0, std::max(0.0, std::min(rear_v + 2.0, (gap - d.s0) / d.T)));
}
} // namespace

void Traffic::spawn_bike(size_t k) {
	const NodeId dest = pick_dest(k);
	if (dest == kNoId) return;
	spawn_bike_to(k, dest);
}

bool Traffic::spawn_bike_to(size_t k, NodeId dest) {
	const Network &n = *net_;
	const NetSpawner &sp = n.spawners[k];
	if (sp.bike_lanes.empty()) return false;
	const size_t nl = sp.bike_lanes.size();
	const size_t first = static_cast<size_t>(rng_.next_u64() % nl);
	Vehicle v;
	v.kind = VehicleKind::Bike;
	v.drv = random_driver(VehicleKind::Bike);
	for (size_t m = 0; m < nl; ++m) {
		const int32_t lane = sp.bike_lanes[(first + m) % nl];
		const NetLane &l = n.lanes[static_cast<size_t>(lane)];
		const std::vector<int32_t> &list = cars_[static_cast<size_t>(lane)];
		double free = l.length, rear_v = 0.0;
		if (!list.empty()) {
			const Vehicle &rear = veh_[static_cast<size_t>(list.back())];
			free = rear.s - rear.drv.length;
			rear_v = rear.v;
		}
		if (free < 6.0) continue;
		std::vector<int32_t> route;
		if (!find_route(lane, dest, route, true, VehicleKind::Bike)) continue;
		const double s = std::min(v.drv.length, l.length);
		v.v = entry_speed(desired_speed(v, lane), !list.empty(), free - s, rear_v, v.drv);
		v.origin = sp.node;
		v.dest = dest;
		v.route = std::move(route);
		insert(std::move(v), lane, s);
		return true;
	}
	return false;
}

void Traffic::spawn() {
	const Network &n = *net_;
	for (size_t k = 0; k < n.spawners.size(); ++k) {
		const NetSpawner &sp = n.spawners[k];
		const bool capped = config_.max_vehicles > 0 && veh_.size() >= config_.max_vehicles;
		if (sp.config.bikes > 0.0 && !sp.bike_lanes.empty() && !reach_[k].empty()) {
			const double p = sp.config.bikes * config_.demand * config_.dt / 3600.0;
			if (rng_.uniform() < p && !capped) spawn_bike(k);
		}
		if (sp.config.rate > 0.0 && !sp.spawn_lanes.empty() && !reach_[k].empty()) {
			const double p = sp.config.rate * config_.demand * config_.dt / 3600.0;
			if (rng_.uniform() < p && pending_[k] < static_cast<uint32_t>(config_.spawn_queue)) ++pending_[k];
		}
		// People who chose to drive (M4) go first.
		const bool person = k < car_trips_.size() && !car_trips_[k].empty();
		if (pending_[k] == 0 && !person) continue;
		if (capped) continue;
		const NodeId dest = person ? car_trips_[k].front() : pick_dest(k);
		if (dest == kNoId) {
			pending_[k] = 0;
			continue;
		}
		bool routed = false;
		const size_t nl = sp.spawn_lanes.size();
		const size_t first = static_cast<size_t>(rng_.next_u64() % nl);
		for (size_t m = 0; m < nl; ++m) {
			const int32_t lane = sp.spawn_lanes[(first + m) % nl];
			const NetLane &l = n.lanes[static_cast<size_t>(lane)];
			const std::vector<int32_t> &list = cars_[static_cast<size_t>(lane)];
			double free = l.length;
			double rear_v = 0.0;
			if (!list.empty()) {
				const Vehicle &rear = veh_[static_cast<size_t>(list.back())];
				free = rear.s - rear.drv.length;
				rear_v = rear.v;
			}
			if (free < 12.0) continue;
			std::vector<int32_t> route;
			if (!find_route(lane, dest, route)) continue;
			routed = true;
			Vehicle v;
			v.drv = random_driver(VehicleKind::Car);
			v.kind = config_.taxi_share > 0.0 && rng_.uniform() < config_.taxi_share ? VehicleKind::Taxi : VehicleKind::Car;
			const double s = std::min(v.drv.length, l.length);
			if (v.kind == VehicleKind::Car && !n.bays.empty() && config_.park_share > 0.0 && rng_.uniform() < config_.park_share) {
				// Park on the way: a free bay on a road the route takes anyway.
				std::vector<char> on_route(n.lanes.size(), 0);
				on_route[static_cast<size_t>(lane)] = 1;
				for (int32_t c : route) on_route[static_cast<size_t>(n.lanes[static_cast<size_t>(c)].to)] = 1;
				std::vector<int32_t> free_bays;
				bool any = false;
				for (size_t b = 0; b < n.bays.size(); ++b) {
					const NetBay &bay = n.bays[b];
					if (!on_route[static_cast<size_t>(bay.lane)]) continue;
					if (bay.lane == lane && bay.s < s + 20.0) continue;
					any = true;
					if (bay_use_[b] == kNoId) free_bays.push_back(static_cast<int32_t>(b));
				}
				if (!free_bays.empty()) {
					const int32_t b = free_bays[static_cast<size_t>(rng_.next_u64() % free_bays.size())];
					const NetBay &bay = n.bays[static_cast<size_t>(b)];
					std::vector<int32_t> to_bay;
					if (find_route_to_lane(lane, s, bay.lane, bay.s, to_bay, VehicleKind::Car, true)) {
						Waypoint wp;
						wp.action = WaypointAction::Park;
						wp.lane = bay.lane;
						wp.s = bay.s;
						wp.bay = b;
						wp.dwell = rng_.range(config_.park_min, config_.park_max);
						v.waypoints.push_back(wp);
						route = std::move(to_bay);
					}
				} else if (any) {
					++stats_.parking_failed;
				}
			}
			v.v = entry_speed(desired_speed(v, lane), !list.empty(), free - s, rear_v, v.drv);
			v.origin = sp.node;
			v.dest = dest;
			v.route = std::move(route);
			const VehicleId id = insert(std::move(v), lane, s);
			const Vehicle &nv = veh_.back();
			if (!nv.waypoints.empty()) bay_use_[static_cast<size_t>(nv.waypoints[0].bay)] = id;
			if (person) car_trips_[k].erase(car_trips_[k].begin());
			else --pending_[k];
			break;
		}
		if (person && !routed) {
			// Blocked lanes keep the trip waiting; no route at all drops it.
			bool blocked = false;
			for (int32_t lane : sp.spawn_lanes) {
				const std::vector<int32_t> &list = cars_[static_cast<size_t>(lane)];
				if (!list.empty()) {
					const Vehicle &rear = veh_[static_cast<size_t>(list.back())];
					blocked |= rear.s - rear.drv.length < 12.0;
				}
			}
			if (!blocked) {
				car_trips_[k].erase(car_trips_[k].begin());
				++stats_.unroutable;
			}
		}
	}
}

void Traffic::dispatch() {
	const Network &n = *net_;
	auto ticks = [&](double sec) { return std::max<uint64_t>(10, static_cast<uint64_t>(std::llround(sec / config_.dt))); };
	// Tries to start a vehicle on one of `lanes`; 1 started, 0 blocked, -1 no route.
	auto start = [&](Vehicle v, const std::vector<int32_t> &lanes) {
		bool blocked = false;
		for (int32_t lane : lanes) {
			const NetLane &l = n.lanes[static_cast<size_t>(lane)];
			const double s = std::min(v.drv.length, l.length);
			if (!lane_free_at(lane, s, v.drv.length)) {
				blocked = true;
				continue;
			}
			std::vector<int32_t> route;
			const Waypoint *wp = v.waypoints.empty() ? nullptr : &v.waypoints[0];
			const bool ok = wp ? find_route_to_lane(lane, s, wp->lane, wp->s, route, v.kind, true)
							   : find_route(lane, v.dest, route, true, v.kind);
			if (!ok) continue;
			v.route = std::move(route);
			v.v = 0.0;
			insert(std::move(v), lane, s);
			return 1;
		}
		return blocked ? 0 : -1;
	};
	for (RouteRun &rr : routes_) {
		if (tick_ < rr.next_departure) continue;
		const NetDepot &d = n.depots[rr.depot_idx];
		const NetRoute &r = d.routes[rr.route_idx];
		const uint64_t hw = ticks(r.headway);
		if (r.stops.empty() || d.spawn_lanes.empty()) {
			rr.next_departure += hw;
			continue;
		}
		uint32_t active = 0;
		for (const Vehicle &v : veh_) active += v.kind == VehicleKind::Bus && v.origin == d.node ? 1u : 0u;
		if (active >= static_cast<uint32_t>(d.capacity)) {
			rr.next_departure += hw; // the depot has no bus to send
			continue;
		}
		Vehicle bus;
		bus.kind = VehicleKind::Bus;
		bus.drv = typical_driver(VehicleKind::Bus);
		bus.origin = d.node;
		bus.dest = d.node;
		bus.bus_route = r.id;
		std::vector<int32_t> seq = r.stops;
		if (r.loop && seq.size() > 1) seq.push_back(seq.front());
		for (int32_t si : seq) {
			const NetStop &st = n.stops[static_cast<size_t>(si)];
			Waypoint wp;
			wp.action = st.kind == StopKind::Kerbside ? WaypointAction::KerbStop : WaypointAction::BayStop;
			wp.lane = st.lane;
			wp.s = st.s;
			wp.dwell = config_.bus_dwell;
			wp.stop = si;
			bus.waypoints.push_back(wp);
		}
		const int res = start(std::move(bus), d.spawn_lanes);
		if (res != 0) rr.next_departure += hw; // started, or no route (try again next headway)
		if (res < 0) ++stats_.unroutable;
	}
	for (CoachRun &cr : coaches_) {
		if (tick_ < cr.next_departure) continue;
		const NetCoachLine &cl = n.coach_lines[cr.line];
		const uint64_t interval = ticks(3600.0 / std::max(0.01, cl.per_hour));
		const NetSpawner *sp = n.spawner_at(cl.entry);
		Vec2 dummy;
		if (!sp || sp->spawn_lanes.empty() || !goal_pos(cl.exit, dummy)) {
			cr.next_departure += interval;
			continue;
		}
		Vehicle coach;
		coach.kind = VehicleKind::Coach;
		coach.drv = typical_driver(VehicleKind::Coach);
		coach.origin = cl.entry;
		coach.dest = cl.exit;
		coach.coach_line = cl.id;
		if (n.main_station >= 0) {
			const NetStop &st = n.stops[static_cast<size_t>(n.main_station)];
			Waypoint wp;
			wp.action = WaypointAction::BayStop;
			wp.lane = st.lane;
			wp.s = st.s;
			wp.dwell = cl.dwell;
			wp.stop = n.main_station;
			coach.waypoints.push_back(wp);
		}
		Vehicle direct = coach;
		direct.waypoints.clear();
		int res = start(std::move(coach), sp->spawn_lanes);
		if (res < 0 && !direct.waypoints.empty()) res = start(std::move(direct), sp->spawn_lanes);
		if (res < 0) res = n.main_station >= 0 ? start(std::move(direct), sp->spawn_lanes) : res;
		if (res != 0) cr.next_departure += interval;
		if (res < 0) ++stats_.unroutable;
	}
}

VehicleId Traffic::add_vehicle(int32_t lane, double s, double v, NodeId dest, const DriverParams2 *driver,
		VehicleKind kind, const std::vector<Waypoint> &waypoints) {
	if (!net_ || lane < 0 || static_cast<size_t>(lane) >= net_->lanes.size()) return kNoId;
	Vehicle car;
	car.kind = kind;
	car.drv = driver ? *driver : typical_driver(kind);
	car.drv.two_sqrt_ab = 2.0 * std::sqrt(car.drv.a * car.drv.b);
	car.id = next_id_++;
	car.lane = lane;
	car.s = std::clamp(s, 0.0, net_->lanes[static_cast<size_t>(lane)].length);
	car.v = v;
	car.prev_lane = lane;
	car.prev_s = car.s;
	car.dest = dest;
	car.waypoints = waypoints;
	car.spawn_tick = tick_;
	car.lane_tick = tick_;
	if (!reroute(car, true)) return kNoId;
	--stats_.reroutes;
	for (const Waypoint &wp : car.waypoints) {
		if (wp.action == WaypointAction::Park && wp.bay >= 0) bay_use_[static_cast<size_t>(wp.bay)] = car.id;
	}
	veh_.push_back(std::move(car));
	rebuild_lists();
	++stats_.spawned;
	return veh_.back().id;
}

// --- Tick --------------------------------------------------------------------------------------

void Traffic::rebuild_lists() {
	for (auto &c : cars_) c.clear();
	for (size_t i = 0; i < veh_.size(); ++i) {
		if (!veh_[i].off_lane) cars_[static_cast<size_t>(veh_[i].lane)].push_back(static_cast<int32_t>(i));
	}
	for (auto &c : cars_) {
		if (c.size() < 2) {
			if (!c.empty()) veh_[static_cast<size_t>(c[0])].list_pos = 0;
			continue;
		}
		std::sort(c.begin(), c.end(), [this](int32_t a, int32_t b) {
			const Vehicle &va = veh_[static_cast<size_t>(a)];
			const Vehicle &vb = veh_[static_cast<size_t>(b)];
			return va.s > vb.s || (va.s == vb.s && va.id < vb.id);
		});
		for (size_t k = 0; k < c.size(); ++k) veh_[static_cast<size_t>(c[k])].list_pos = static_cast<int32_t>(k);
	}
}

void Traffic::compact() {
	size_t w = 0;
	for (size_t i = 0; i < veh_.size(); ++i) {
		if (veh_[i].done) {
			// Anyone still on board leaves the map with it.
			for (uint32_t id : veh_[i].riders) {
				const int32_t pi = find_pedestrian(id);
				if (pi >= 0) peds_[static_cast<size_t>(pi)].done = true;
			}
			continue;
		}
		if (w != i) veh_[w] = std::move(veh_[i]);
		++w;
	}
	veh_.resize(w);
}

void Traffic::tick() {
	if (!net_) {
		++tick_;
		return;
	}
	const Network &n = *net_;
	for (Vehicle &v : veh_) {
		v.prev_lane = v.lane;
		v.prev_s = v.s;
		v.prev_lat = v.lat;
		v.blocker = kNoId;
		switch (v.phase) {
			case 0:
				v.state = VehicleState::Driving;
				break;
			case 1:
				v.state = VehicleState::AtStop;
				break;
			case 3: {
				const Waypoint *wp = pending_waypoint(v);
				v.state = wp && wp->action == WaypointAction::Park ? VehicleState::Parked : VehicleState::AtStop;
				break;
			}
			default:
				v.state = VehicleState::Parking;
				break;
		}
	}
	if (people_on_ || !peds_.empty() || crossings_live_) peds_prepare();
	arbitrate();
	change_lanes();
	rebuild_lists();
	// Accelerations from the start-of-tick state.
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle &v = veh_[i];
		if (v.off_lane || v.phase != 0) {
			v.acc = 0.0;
			continue;
		}
		bool has = false;
		double gap = 0.0, lv = 0.0, cap = 0.0;
		VehicleId who = kNoId;
		leader(v, v.lane, v.s, v.ri, has, gap, lv, cap, who);
		if (v.courtesy >= 0) {
			const Vehicle &m = veh_[static_cast<size_t>(v.courtesy)];
			const double g = m.merge_s - m.drv.length - v.s;
			if (m.merge_lane == v.lane && g > 0.5 && g < gap) {
				gap = g;
				lv = m.v;
				has = true;
				who = m.id;
			}
		}
		const double a = idm(v, std::min(desired_speed(v, v.lane), cap), has, gap, lv);
		v.acc = a < -config_.max_decel ? -config_.max_decel : a;
		if (v.state == VehicleState::Driving) {
			if (!is_road(n.lanes[static_cast<size_t>(v.lane)])) {
				v.state = VehicleState::InJunction;
			} else if (has && who == kPedestrians && gap < 15.0 && v.v < 2.0) {
				v.state = VehicleState::GivingWay;
			} else if (has && who != kNoId && gap < 15.0 && v.v < 2.0) {
				v.state = VehicleState::Queued;
				v.blocker = who;
			}
		}
		const bool yields = has && who == kPedestrians && gap < 15.0;
		if (yields && !v.ped_yield) ++stats_.cars_yielded;
		v.ped_yield = yields;
	}
	move();
	// Periodic re-routing with observed travel times, spread over the cars.
	const uint64_t period = static_cast<uint64_t>(config_.reroute_interval / config_.dt);
	if (period > 0) {
		for (Vehicle &v : veh_) {
			if (v.done || v.grant >= 0 || v.phase != 0 || !is_road(n.lanes[static_cast<size_t>(v.lane)])) continue;
			if ((tick_ + static_cast<uint64_t>(v.id) * 7919ull) % period == 0) reroute(v, true);
		}
	}
	compact();
	rebuild_lists();
	if (waypoints()) {
		compact();
		rebuild_lists();
	}
	peds_tick();
	dispatch();
	people_demand();
	spawn(); // new vehicles join at the back of their lane
	++tick_;
}

// --- Queries ---------------------------------------------------------------------------------------

int32_t Traffic::find_vehicle(VehicleId id) const {
	auto it = std::lower_bound(veh_.begin(), veh_.end(), id, [](const Vehicle &v, VehicleId x) { return v.id < x; });
	return it != veh_.end() && it->id == id ? static_cast<int32_t>(it - veh_.begin()) : -1;
}

Pose Traffic::pose(size_t i, double alpha) const {
	const Vehicle &v = veh_[i];
	if (v.off_lane) return Pose{ v.bay_pos, v.bay_dir };
	Pose a = net_->pose(v.prev_lane, v.prev_s);
	Pose b = net_->pose(v.lane, v.s);
	a.pos = a.pos + a.dir.right() * v.prev_lat;
	b.pos = b.pos + b.dir.right() * v.lat;
	Pose p;
	p.pos = a.pos + (b.pos - a.pos) * alpha;
	p.dir = (a.dir + (b.dir - a.dir) * alpha).normalized();
	if ((v.phase == 2 || v.phase == 4) && v.phase_until > v.phase_start && v.bay_dir.length() > 0.5) {
		// Manoeuvring between the lane and the bay.
		double t = (static_cast<double>(tick_) + alpha - static_cast<double>(v.phase_start)) /
				static_cast<double>(v.phase_until - v.phase_start);
		t = std::clamp(t, 0.0, 1.0);
		if (v.phase == 4) t = 1.0 - t;
		p.pos = p.pos + (v.bay_pos - p.pos) * t;
		p.dir = (p.dir + (v.bay_dir - p.dir) * t).normalized();
	}
	return p;
}

int Traffic::level_of(size_t i) const {
	const NetLane &l = net_->lanes[static_cast<size_t>(veh_[i].lane)];
	return veh_[i].s > 0.5 * l.length ? l.level_end : l.level; // ramps change level halfway
}

VehicleInfo Traffic::info(VehicleId id) const {
	VehicleInfo out;
	out.id = id;
	const int32_t i = find_vehicle(id);
	if (i < 0 || !net_) return out;
	const Network &n = *net_;
	const Vehicle &v = veh_[static_cast<size_t>(i)];
	out.found = true;
	out.kind = v.kind;
	out.speed = v.v;
	out.desired_speed = desired_speed(v, v.lane);
	out.accel = v.acc;
	out.state = v.state;
	out.blocker = v.blocker;
	out.origin = v.origin;
	out.dest = v.dest;
	out.trip_time = static_cast<double>(tick_ - std::min(tick_, v.spawn_tick)) * config_.dt;
	out.distance = v.distance;
	out.stopped_for = v.stopped_for;
	out.critical_gap = v.drv.critical_gap;
	out.bus_route = v.bus_route;
	out.coach_line = v.coach_line;
	for (size_t w = v.wi; w < v.waypoints.size(); ++w) {
		const Waypoint &wp = v.waypoints[w];
		if (wp.stop >= 0) {
			if (out.next_stop < 0) out.next_stop = wp.stop;
			++out.stops_left;
		}
		if (wp.action == WaypointAction::Park) out.parks = true;
	}
	const NetLane &l = n.lanes[static_cast<size_t>(v.lane)];
	out.level = l.level;
	out.lane = l.key;
	out.segment = l.segment;
	out.connectors_left = v.route.size() > v.ri ? v.route.size() - v.ri : 0;
	if (v.off_lane) return out;
	// Route polyline: the lanes the car will follow.
	int32_t cur = v.lane;
	double from = v.s;
	size_t r = v.ri;
	for (int hop = 0; hop < 400 && cur >= 0; ++hop) {
		const NetLane &cl = n.lanes[static_cast<size_t>(cur)];
		for (size_t k = 0; k < cl.pts.size(); ++k) {
			if (cl.cum[k] > from) out.route.push_back(cl.pts[k]);
		}
		if (hop == 0) out.route.insert(out.route.begin(), n.pose(cur, from).pos);
		from = 0.0;
		if (is_road(cl)) {
			const int32_t c = next_connector(v, cur, r);
			if (c < 0) break;
			if (out.junction == kNoId) out.junction = n.lanes[static_cast<size_t>(c)].node;
			cur = c;
			++r;
		} else {
			cur = cl.next.empty() ? -1 : cl.next[0];
		}
	}
	return out;
}

TrafficStats Traffic::stats() const {
	TrafficStats st = stats_;
	st.vehicles = static_cast<uint32_t>(veh_.size());
	double sum = 0.0;
	uint32_t moving = 0;
	for (const Vehicle &v : veh_) {
		++st.by_kind[static_cast<size_t>(v.kind)];
		if (v.phase == 3 && v.off_lane) {
			const Waypoint *wp = pending_waypoint(v);
			if (wp && wp->action == WaypointAction::Park) ++st.parked;
		}
		if (v.phase != 0) continue; // at a stop or parked: not traffic
		++moving;
		sum += v.v;
		if (v.v < 1.0) ++st.stopped;
		st.max_stopped = std::max(st.max_stopped, v.stopped_for);
	}
	st.mean_speed = moving ? sum / static_cast<double>(moving) : 0.0;
	st.mean_trip_time = stats_.arrived ? trip_time_sum_ / static_cast<double>(stats_.arrived) : 0.0;
	for (const Pedestrian &p : peds_) {
		if (p.state == PedState::Riding) ++st.riding;
		else ++st.pedestrians;
	}
	st.mean_wait = stats_.boarded ? wait_sum_ / static_cast<double>(stats_.boarded) : 0.0;
	st.mean_crossing_wait = stats_.crossings ? crossing_wait_sum_ / static_cast<double>(stats_.crossings) : 0.0;
	for (uint32_t p : pending_) st.waiting_to_enter += p;
	for (size_t j = 0; j < junction_waiting_.size(); ++j) {
		if (!junction_waiting_[j]) continue;
		st.max_junction_wait = std::max(st.max_junction_wait,
				static_cast<double>(tick_ - std::min(tick_, junction_last_grant_[j])) * config_.dt);
	}
	return st;
}

std::vector<RouteStats> Traffic::route_stats() const {
	std::vector<RouteStats> out;
	if (!net_) return out;
	for (const RouteRun &rr : routes_) {
		RouteStats rs;
		rs.id = rr.id;
		rs.runs = rr.runs;
		rs.round_trip = rr.round_trip;
		const NetRoute &r = net_->depots[rr.depot_idx].routes[rr.route_idx];
		if (r.headway > 0.0 && rr.round_trip > 0.0) rs.fleet = static_cast<uint32_t>(std::ceil(rr.round_trip / r.headway));
		for (const Vehicle &v : veh_) rs.active += v.kind == VehicleKind::Bus && v.bus_route == rr.id ? 1u : 0u;
		out.push_back(rs);
	}
	return out;
}

uint64_t Traffic::state_hash() const {
	StateHasher h;
	h.add_u64(tick_);
	for (int k = 0; k < 4; ++k) h.add_u64(rng_.state()[k]);
	h.add_u64(veh_.size());
	for (const Vehicle &v : veh_) {
		const LaneKey &k = net_->lanes[static_cast<size_t>(v.lane)].key;
		h.add_u32(v.id);
		h.add_u32(static_cast<uint32_t>(v.kind));
		h.add_u32(static_cast<uint32_t>(k.kind));
		h.add_u32(k.a);
		h.add_u32(k.b);
		h.add_double(v.s);
		h.add_double(v.v);
		h.add_u64(v.ri);
		h.add_u64(v.wi);
		h.add_u32(v.phase);
		h.add_u32(v.dest);
		h.add_u64(v.grant >= 0 ? 1 : 0);
	}
	for (uint32_t p : pending_) h.add_u32(p);
	for (VehicleId b : bay_use_) h.add_u32(b);
	if (people_on_ || !peds_.empty()) {
		h.add_u64(peds_.size());
		for (const Pedestrian &p : peds_) {
			h.add_u32(p.id);
			h.add_u32(static_cast<uint32_t>(p.state));
			h.add_u32(static_cast<uint32_t>(p.edge));
			h.add_u32(static_cast<uint32_t>(p.from));
			h.add_double(p.d);
			h.add_u32(p.vehicle);
		}
		for (const MidSignal &m : mid_signal_) h.add_u32(m.state);
	}
	return h.value();
}

} // namespace tsim
