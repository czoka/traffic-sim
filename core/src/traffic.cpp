#include "tsim/traffic.h"

#include "tsim/hash.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <queue>

namespace tsim {

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
	}
	return "driving";
}

namespace {

constexpr double kInf = 1e300;
constexpr uint64_t kNever = ~0ull;

bool is_road(const NetLane &l) { return l.kind == NetLaneKind::Road; }

} // namespace

Traffic::Traffic(TrafficConfig config) : config_(config) {}

// --- Network binding ----------------------------------------------------------------

void Traffic::set_network(const Network *net) {
	struct Saved {
		LaneKey lane, prev;
		std::vector<LaneKey> route;
		bool grant = false, held = false;
		LaneKey grant_key, held_key;
	};
	std::vector<Saved> saved(veh_.size());
	std::map<LaneKey, double> times;
	std::map<NodeId, uint32_t> pending;
	if (net_) {
		for (size_t i = 0; i < veh_.size(); ++i) {
			const Vehicle &v = veh_[i];
			Saved &s = saved[i];
			s.lane = net_->lanes[static_cast<size_t>(v.lane)].key;
			s.prev = net_->lanes[static_cast<size_t>(v.prev_lane)].key;
			for (int32_t c : v.route) s.route.push_back(net_->lanes[static_cast<size_t>(c)].key);
			if (v.grant >= 0) {
				s.grant = true;
				s.grant_key = net_->lanes[static_cast<size_t>(v.grant)].key;
			}
			if (v.held >= 0) {
				s.held = true;
				s.held_key = net_->lanes[static_cast<size_t>(v.held)].key;
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
	rebuild_reachability();
	if (!net_) {
		veh_.clear();
		return;
	}
	// Carry vehicles over by key.
	std::vector<Vehicle> kept;
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle v = veh_[i];
		const Saved &s = saved[i];
		v.lane = net_->find(s.lane);
		if (v.lane < 0) continue;
		const NetLane &lane = net_->lanes[static_cast<size_t>(v.lane)];
		v.s = std::min(v.s, lane.length);
		v.prev_lane = v.lane;
		v.prev_s = v.s;
		v.prev_lat = v.lat;
		v.grant = s.grant ? net_->find(s.grant_key) : -1;
		v.held = s.held ? net_->find(s.held_key) : -1;
		if (v.grant >= 0 && net_->lanes[static_cast<size_t>(v.grant)].from != v.lane) v.grant = -1;
		v.route.clear();
		v.ri = 0;
		v.wait_since = 0;
		v.stopped_tick = 0;
		kept.push_back(std::move(v));
	}
	veh_ = std::move(kept);
	rebuild_lists();
	for (Vehicle &v : veh_) {
		if (!net_->spawner_at(v.dest) || !reroute(v, true)) {
			// The destination is gone or unreachable: pick the first sink that works.
			bool ok = false;
			for (const NetSpawner &sp : net_->spawners) {
				if (!sp.config.sink || sp.node == v.origin) continue;
				v.dest = sp.node;
				if (reroute(v, true)) {
					ok = true;
					break;
				}
			}
			if (!ok) v.route.clear();
		}
	}
}

void Traffic::rebuild_reachability() {
	reach_.clear();
	if (!net_) return;
	const Network &n = *net_;
	for (const NetSpawner &src : n.spawners) {
		std::vector<std::pair<size_t, double>> row;
		if (src.config.rate > 0.0 && !src.spawn_lanes.empty()) {
			std::vector<char> seen(n.lanes.size(), 0);
			std::vector<int32_t> stack(src.spawn_lanes.begin(), src.spawn_lanes.end());
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
	if (net_) {
		for (size_t l = 0; l < lane_time_.size(); ++l) {
			const NetLane &lane = net_->lanes[l];
			lane_time_[l] = lane.length / std::max(1.0, lane.speed_limit);
		}
	}
	stats_ = TrafficStats{};
	trip_time_sum_ = 0.0;
}

// --- Routing ---------------------------------------------------------------------------

double Traffic::lane_cost(int32_t l) const {
	const NetLane &lane = net_->lanes[static_cast<size_t>(l)];
	double t = lane_time_[static_cast<size_t>(l)];
	if (lane.type == LaneType::Bus) t *= config_.bus_lane_factor;
	return t;
}

bool Traffic::find_route(int32_t start, NodeId dest, std::vector<int32_t> &route, bool allow_change_first) const {
	route.clear();
	if (!net_ || start < 0) return false;
	const Network &n = *net_;
	const NetSpawner *sp = n.spawner_at(dest);
	if (!sp || !sp->config.sink || !is_road(n.lanes[static_cast<size_t>(start)])) return false;
	const size_t N = n.lanes.size();
	std::vector<double> g(N, kInf);
	std::vector<int32_t> parent(N, -1), via(N, -1);
	std::vector<char> closed(N, 0);
	using Entry = std::pair<double, int32_t>;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	const Vec2 goal = sp->pos;
	auto h = [&](int32_t l) { return (n.lanes[static_cast<size_t>(l)].pts.back() - goal).length() / n.max_speed; };
	g[static_cast<size_t>(start)] = lane_cost(start);
	open.push({ g[static_cast<size_t>(start)] + h(start), start });
	int32_t found = -1;
	while (!open.empty()) {
		const int32_t l = open.top().second;
		open.pop();
		if (closed[static_cast<size_t>(l)]) continue;
		closed[static_cast<size_t>(l)] = 1;
		const NetLane &lane = n.lanes[static_cast<size_t>(l)];
		if (lane.end_node == dest && lane.sink) {
			found = l;
			break;
		}
		const double gl = g[static_cast<size_t>(l)];
		auto relax = [&](int32_t to, double cost, int32_t through) {
			if (closed[static_cast<size_t>(to)]) return;
			const double ng = gl + cost;
			if (ng < g[static_cast<size_t>(to)]) {
				g[static_cast<size_t>(to)] = ng;
				parent[static_cast<size_t>(to)] = l;
				via[static_cast<size_t>(to)] = through;
				open.push({ ng + h(to), to });
			}
		};
		for (int32_t c : lane.next) {
			const NetLane &cn = n.lanes[static_cast<size_t>(c)];
			double turn = 0.0;
			if (cn.turn == TurnKind::Left) turn = 2.0;
			else if (cn.turn == TurnKind::Right) turn = 1.0;
			else if (cn.turn == TurnKind::UTurn) turn = 10.0;
			if (lane.merge_end) turn += 5.0; // prefer leaving a dropped lane early
			relax(cn.to, lane_time_[static_cast<size_t>(c)] + turn + lane_cost(cn.to), c);
		}
		if (allow_change_first || l != start) {
			for (int side = 0; side < 2; ++side) {
				const int32_t nb = side == 0 ? lane.left : lane.right;
				if (nb < 0 || (side == 0 ? lane.change_left : lane.change_right).empty()) continue;
				const double extra = lane_cost(nb) - lane_cost(l);
				relax(nb, config_.lane_change_cost + (extra > 0.0 ? extra : 0.0), -1);
			}
		}
	}
	if (found < 0) return false;
	for (int32_t l = found; l != start; l = parent[static_cast<size_t>(l)]) {
		if (via[static_cast<size_t>(l)] >= 0) route.push_back(via[static_cast<size_t>(l)]);
	}
	std::reverse(route.begin(), route.end());
	return true;
}

bool Traffic::reroute(Vehicle &v, bool allow_change_first) {
	const NetLane &lane = net_->lanes[static_cast<size_t>(v.lane)];
	const int32_t start = is_road(lane) ? v.lane : lane.to;
	std::vector<int32_t> r;
	if (!find_route(start, v.dest, r, allow_change_first || !is_road(lane))) return false;
	v.route = std::move(r);
	v.ri = 0;
	++stats_.reroutes;
	return true;
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
	int32_t best = -1;
	for (int32_t c : l.next) {
		const NetLane &to = n.lanes[static_cast<size_t>(n.lanes[static_cast<size_t>(c)].to)];
		if (to.segment != target.segment || to.dir != target.dir) continue;
		if (n.lanes[static_cast<size_t>(c)].to == w.to) return c;
		if (best < 0) best = c;
	}
	return best;
}

bool Traffic::lane_is_good(const Vehicle &v, int32_t lane, size_t ri) const {
	const NetLane &l = net_->lanes[static_cast<size_t>(lane)];
	if (ri >= v.route.size()) return l.end_node == v.dest && l.sink;
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

// --- Car following ----------------------------------------------------------------------

double Traffic::desired_speed(const Vehicle &v, int32_t lane) const {
	const NetLane &l = net_->lanes[static_cast<size_t>(lane)];
	return is_road(l) ? l.speed_limit * v.drv.speed_factor : l.speed_limit * std::min(1.0, v.drv.speed_factor);
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
	// Car ahead on the same lane.
	const std::vector<int32_t> &list = cars_[static_cast<size_t>(lane)];
	int32_t ahead = -1;
	if (lane == v.lane && s == v.s) {
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
	double dist = n.lanes[static_cast<size_t>(lane)].length - s;
	int32_t cur = lane;
	size_t r = ri;
	for (int hop = 0; hop < 16 && dist < config_.lookahead; ++hop) {
		const NetLane &l = n.lanes[static_cast<size_t>(cur)];
		int32_t nx;
		if (is_road(l)) {
			if (r >= v.route.size() && l.end_node == v.dest && l.sink) return; // drives off the map
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
		}
		const double lim = desired_speed(v, nx);
		v0_cap = std::min(v0_cap, std::sqrt(lim * lim + 2.0 * v.drv.b * (dist > 0.0 ? dist : 0.0)));
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
	for (size_t j = 0; j < nj; ++j) {
		const NetJunction &junc = n.junctions[j];
		if (!junc.arbitrated) continue;
		std::vector<int32_t> &box = granted[j];
		// Give back grants of cars that stopped before the line (their exit filled up).
		for (int32_t k : box) {
			Vehicle &o = veh_[static_cast<size_t>(k)];
			if (o.grant < 0 || n.lanes[static_cast<size_t>(o.grant)].junction != static_cast<int32_t>(j)) continue;
			if (o.lane != n.lanes[static_cast<size_t>(o.grant)].from) continue;
			const double d = n.lanes[static_cast<size_t>(o.lane)].length - o.s;
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
				if (d <= 150.0) {
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
			if (c.dist < 6.0 && o.v < 0.5) {
				if (o.wait_since == 0) o.wait_since = tick_ + 1;
				any_waiting = true;
				could_go = could_go || exit_clear(c.conn, c.veh, box);
			}
			if (junc.control == JunctionControl::AllWayStop && c.dist < 4.0 && o.v < 0.2 && o.stopped_tick == 0) {
				o.stopped_tick = tick_ + 1;
			}
		}
		std::sort(cands.begin(), cands.end(), [&](const Candidate &a, const Candidate &b) {
			const Vehicle &va = veh_[static_cast<size_t>(a.veh)];
			const Vehicle &vb = veh_[static_cast<size_t>(b.veh)];
			const uint64_t wa = va.wait_since ? va.wait_since : kNever;
			const uint64_t wb = vb.wait_since ? vb.wait_since : kNever;
			return wa != wb ? wa < wb : va.id < vb.id;
		});
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
			o.state = VehicleState::Driving;
			o.blocker = kNoId;
			box.push_back(c.veh);
			junction_last_grant_[j] = tick_;
		};
		bool granted_now = false;
		for (const Candidate &c : cands) {
			Vehicle &o = veh_[static_cast<size_t>(c.veh)];
			const double reach = o.v * o.v / (2.0 * o.drv.b) + 0.5 * o.v + 6.0;
			if (c.dist > reach) {
				o.state = VehicleState::Approaching;
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
				const bool impatient = waited >= config_.impatience;
				for (const Conflict &cf : x.conflicts) {
					for (const Candidate &q : cands) {
						if (q.conn != cf.other || q.veh == c.veh) continue;
						const Vehicle &p = veh_[static_cast<size_t>(q.veh)];
						const double p_waited = p.wait_since ? static_cast<double>(tick_ + 1 - p.wait_since) * config_.dt : 0.0;
						if (p_waited >= config_.impatience && p_waited > waited) {
							// A driver who has waited long gets let in: no new conflicting
							// grants until it has gone (unless its own exit is full, for a while).
							if (p_waited >= 2.0 * config_.impatience || exit_clear(q.conn, q.veh, box)) {
								go = false;
								blk = p.id;
								break;
							}
						}
						if (cf.priority >= 0) continue;
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
			granted_now = true;
		}
		// Deadlock breaker: everyone is waiting for someone and the box is empty.
		if (!granted_now && any_waiting && box.empty()) {
			for (const Candidate &c : cands) {
				const Vehicle &o = veh_[static_cast<size_t>(c.veh)];
				if (o.wait_since == 0 || static_cast<double>(tick_ + 1 - o.wait_since) * config_.dt < 3.0) continue;
				if (junc.control == JunctionControl::AllWayStop && o.stopped_tick == 0) continue;
				if (!exit_clear(c.conn, c.veh, box)) continue;
				grant(c);
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
		const NetLane &l = n.lanes[static_cast<size_t>(v.lane)];
		if (!is_road(l)) continue;
		if (v.grant >= 0) continue; // committed to its junction entry
		if (v.last_change != 0 && tick_ + 1 < v.last_change + 30) continue;
		int steps = 0;
		const int want = good_direction(v, v.lane, steps);
		const double remaining = l.length - v.s;
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
		if (!mandatory && (tick_ + v.id) % 5 != 0) continue;

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
			const bool toward = want == (left ? -1 : 1);
			if (toward) {
				toward_lane = t;
				toward_s = st;
			}
			if (tl.type == LaneType::Bus && !toward) continue;
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
			double follower_v = 0.0;
			const Vehicle *fv = nullptr;
			if (follow >= 0) {
				fv = &veh_[static_cast<size_t>(follow)];
				gap_back = st - v.drv.length - fv->s;
				follower_v = fv->v;
			} else {
				// A car about to come off a connector into the target lane.
				for (int32_t c : tl.prev) {
					if (cars_[static_cast<size_t>(c)].empty()) continue;
					const Vehicle &o = veh_[static_cast<size_t>(cars_[static_cast<size_t>(c)].front())];
					const double g = st - v.drv.length + (n.lanes[static_cast<size_t>(c)].length - o.s);
					if (g < gap_back) {
						gap_back = g;
						follower_v = o.v;
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
			(void)follower_v;
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
			std::vector<int32_t> &old_list = cars_[static_cast<size_t>(v.lane)];
			old_list.erase(std::find(old_list.begin(), old_list.end(), static_cast<int32_t>(i)));
			v.lane = t;
			v.s = best_s;
			v.lat = best_side == 0 ? d : -d;
			v.last_change = tick_ + 1;
			std::vector<int32_t> &new_list = cars_[static_cast<size_t>(t)];
			auto pos = std::find_if(new_list.begin(), new_list.end(), [&](int32_t k) {
				const Vehicle &o = veh_[static_cast<size_t>(k)];
				return o.s < v.s || (o.s == v.s && o.id > v.id);
			});
			new_list.insert(pos, static_cast<int32_t>(i));
			++stats_.lane_changes;
		} else if (mandatory && toward_lane >= 0) {
			v.merge_lane = toward_lane;
			v.merge_s = toward_s;
			v.state = VehicleState::ChangingLane;
			requests.push_back({ static_cast<int32_t>(i), toward_lane, toward_s });
		}
		if (best_side < 0 && mandatory && remaining < 3.0 && v.v < 0.5 && v.stopped_for > 20.0) {
			// Waited too long for a gap: take any way out of this lane.
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

// --- Movement --------------------------------------------------------------------------------

void Traffic::move() {
	const Network &n = *net_;
	const double dt = config_.dt;
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle &v = veh_[i];
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
				if (v.ri >= v.route.size() && l.end_node == v.dest && l.sink) {
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
				t = std::max(l.length / std::max(1.0, l.speed_limit), 0.9 * t + 0.1 * sample);
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
		v.stopped_for = v.v < 0.1 ? v.stopped_for + dt : 0.0;
		if (v.stopped_for > config_.stuck_timeout && !v.done) {
			v.done = true;
			++stats_.removed_stuck;
		} else if (v.done) {
			++stats_.arrived;
			trip_time_sum_ += static_cast<double>(tick_ + 1 - v.spawn_tick) * dt;
		}
	}
}

// --- Demand ----------------------------------------------------------------------------------

void Traffic::spawn() {
	const Network &n = *net_;
	for (size_t k = 0; k < n.spawners.size(); ++k) {
		const NetSpawner &sp = n.spawners[k];
		if (sp.config.rate > 0.0 && !sp.spawn_lanes.empty() && !reach_[k].empty()) {
			const double p = sp.config.rate * config_.demand * config_.dt / 3600.0;
			if (rng_.uniform() < p && pending_[k] < static_cast<uint32_t>(config_.spawn_queue)) ++pending_[k];
		}
		if (pending_[k] == 0) continue;
		if (config_.max_vehicles > 0 && veh_.size() >= config_.max_vehicles) continue;
		// Destination by origin-destination weight.
		double total = 0.0;
		for (const auto &d : reach_[k]) total += d.second;
		if (total <= 0.0) {
			pending_[k] = 0;
			continue;
		}
		double x = rng_.uniform() * total;
		size_t pick = reach_[k].back().first;
		for (const auto &d : reach_[k]) {
			if (x < d.second) {
				pick = d.first;
				break;
			}
			x -= d.second;
		}
		const NodeId dest = n.spawners[pick].node;
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
			Vehicle v;
			v.drv.speed_factor = rng_.range(0.9, 1.1);
			v.drv.T = rng_.range(1.0, 1.6);
			v.drv.a = rng_.range(1.0, 1.8);
			v.drv.b = rng_.range(1.6, 2.4);
			v.drv.s0 = 2.0;
			v.drv.length = rng_.range(4.2, 4.8);
			v.drv.two_sqrt_ab = 2.0 * std::sqrt(v.drv.a * v.drv.b);
			v.drv.critical_gap = rng_.range(4.0, 6.0);
			v.drv.politeness = rng_.range(0.1, 0.5);
			v.id = next_id_++;
			v.lane = lane;
			v.s = std::min(v.drv.length, l.length);
			const double v0 = desired_speed(v, lane);
			double speed = v0;
			if (!list.empty()) {
				const double gap = free - v.s;
				speed = std::min(v0, std::max(0.0, std::min(rear_v + 2.0, (gap - v.drv.s0) / v.drv.T)));
			}
			v.v = speed;
			v.prev_lane = lane;
			v.prev_s = v.s;
			v.origin = sp.node;
			v.dest = dest;
			v.route = std::move(route);
			v.spawn_tick = tick_ + 1;
			v.lane_tick = tick_ + 1;
			v.list_pos = static_cast<int32_t>(list.size());
			veh_.push_back(std::move(v));
			cars_[static_cast<size_t>(lane)].push_back(static_cast<int32_t>(veh_.size() - 1));
			--pending_[k];
			++stats_.spawned;
			break;
		}
	}
}

VehicleId Traffic::add_vehicle(int32_t lane, double s, double v, NodeId dest, const DriverParams2 *driver) {
	if (!net_ || lane < 0 || static_cast<size_t>(lane) >= net_->lanes.size()) return kNoId;
	Vehicle car;
	if (driver) car.drv = *driver;
	car.drv.two_sqrt_ab = 2.0 * std::sqrt(car.drv.a * car.drv.b);
	car.id = next_id_++;
	car.lane = lane;
	car.s = std::clamp(s, 0.0, net_->lanes[static_cast<size_t>(lane)].length);
	car.v = v;
	car.prev_lane = lane;
	car.prev_s = car.s;
	car.dest = dest;
	car.spawn_tick = tick_;
	car.lane_tick = tick_;
	if (!reroute(car, true)) return kNoId;
	--stats_.reroutes;
	veh_.push_back(std::move(car));
	rebuild_lists();
	++stats_.spawned;
	return veh_.back().id;
}

// --- Tick --------------------------------------------------------------------------------------

void Traffic::rebuild_lists() {
	for (auto &c : cars_) c.clear();
	for (size_t i = 0; i < veh_.size(); ++i) cars_[static_cast<size_t>(veh_[i].lane)].push_back(static_cast<int32_t>(i));
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
		if (veh_[i].done) continue;
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
		v.state = VehicleState::Driving;
		v.blocker = kNoId;
	}
	arbitrate();
	change_lanes();
	rebuild_lists();
	// Accelerations from the start-of-tick state.
	for (size_t i = 0; i < veh_.size(); ++i) {
		Vehicle &v = veh_[i];
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
			} else if (has && who != kNoId && gap < 15.0 && v.v < 2.0) {
				v.state = VehicleState::Queued;
				v.blocker = who;
			}
		}
	}
	move();
	// Periodic re-routing with observed travel times, spread over the cars.
	const uint64_t period = static_cast<uint64_t>(config_.reroute_interval / config_.dt);
	if (period > 0) {
		for (Vehicle &v : veh_) {
			if (v.done || v.grant >= 0 || !is_road(n.lanes[static_cast<size_t>(v.lane)])) continue;
			if ((tick_ + static_cast<uint64_t>(v.id) * 7919ull) % period == 0) reroute(v, true);
		}
	}
	compact();
	rebuild_lists();
	spawn(); // new cars join at the back of their lane, so the lists stay sorted
	++tick_;
}

// --- Queries ---------------------------------------------------------------------------------------

int32_t Traffic::find_vehicle(VehicleId id) const {
	auto it = std::lower_bound(veh_.begin(), veh_.end(), id, [](const Vehicle &v, VehicleId x) { return v.id < x; });
	return it != veh_.end() && it->id == id ? static_cast<int32_t>(it - veh_.begin()) : -1;
}

Pose Traffic::pose(size_t i, double alpha) const {
	const Vehicle &v = veh_[i];
	Pose a = net_->pose(v.prev_lane, v.prev_s);
	Pose b = net_->pose(v.lane, v.s);
	a.pos = a.pos + a.dir.right() * v.prev_lat;
	b.pos = b.pos + b.dir.right() * v.lat;
	Pose p;
	p.pos = a.pos + (b.pos - a.pos) * alpha;
	p.dir = (a.dir + (b.dir - a.dir) * alpha).normalized();
	return p;
}

int Traffic::level_of(size_t i) const { return net_->lanes[static_cast<size_t>(veh_[i].lane)].level; }

VehicleInfo Traffic::info(VehicleId id) const {
	VehicleInfo out;
	out.id = id;
	const int32_t i = find_vehicle(id);
	if (i < 0 || !net_) return out;
	const Network &n = *net_;
	const Vehicle &v = veh_[static_cast<size_t>(i)];
	out.found = true;
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
	const NetLane &l = n.lanes[static_cast<size_t>(v.lane)];
	out.level = l.level;
	out.lane = l.key;
	out.segment = l.segment;
	out.connectors_left = v.route.size() > v.ri ? v.route.size() - v.ri : 0;
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
	for (const Vehicle &v : veh_) {
		sum += v.v;
		if (v.v < 1.0) ++st.stopped;
		st.max_stopped = std::max(st.max_stopped, v.stopped_for);
	}
	st.mean_speed = veh_.empty() ? 0.0 : sum / static_cast<double>(veh_.size());
	st.mean_trip_time = stats_.arrived ? trip_time_sum_ / static_cast<double>(stats_.arrived) : 0.0;
	for (uint32_t p : pending_) st.waiting_to_enter += p;
	for (size_t j = 0; j < junction_waiting_.size(); ++j) {
		if (!junction_waiting_[j]) continue;
		st.max_junction_wait = std::max(st.max_junction_wait,
				static_cast<double>(tick_ - std::min(tick_, junction_last_grant_[j])) * config_.dt);
	}
	return st;
}

uint64_t Traffic::state_hash() const {
	StateHasher h;
	h.add_u64(tick_);
	for (int k = 0; k < 4; ++k) h.add_u64(rng_.state()[k]);
	h.add_u64(veh_.size());
	for (const Vehicle &v : veh_) {
		const LaneKey &k = net_->lanes[static_cast<size_t>(v.lane)].key;
		h.add_u32(v.id);
		h.add_u32(static_cast<uint32_t>(k.kind));
		h.add_u32(k.a);
		h.add_u32(k.b);
		h.add_double(v.s);
		h.add_double(v.v);
		h.add_u64(v.ri);
		h.add_u32(v.dest);
		h.add_u64(v.grant >= 0 ? 1 : 0);
	}
	for (uint32_t p : pending_) h.add_u32(p);
	return h.value();
}

} // namespace tsim
