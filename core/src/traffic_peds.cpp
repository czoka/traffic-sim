// Traffic (M4): people on foot and on the bus.
//
// Pedestrians walk the PedGraph along edges, without colliding with each
// other. At a crossing they wait at the kerb until it is safe (zebra: every
// car can still stop; uncontrolled: a gap; signal: walk), and cars give way
// to anyone on a crossing who has not yet passed their lane. People trips
// start at spawn points and pick walking, a bus (one transfer at most), a bike
// or a car on generalized cost; passengers wait at stops, board (capacity,
// 2 s per person per door) and alight.
//
// Like the rest of the tick: only + - * / and sqrt, the seeded RNG, and
// iteration in index order.
#include "tsim/traffic.h"

#include "tsim/hash.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <queue>
#include <tuple>

namespace tsim {

const char *ped_state_name(PedState s) {
	switch (s) {
		case PedState::Walking:
			return "walking";
		case PedState::WaitingToCross:
			return "waiting to cross";
		case PedState::Crossing:
			return "crossing";
		case PedState::WaitingForBus:
			return "waiting for the bus";
		case PedState::Riding:
			return "on the bus";
	}
	return "walking";
}

namespace {

constexpr double kInf = 1e300;
constexpr double kWalkSpeed = 1.35; // m/s, for perceived costs

int32_t other_end(const PedEdge &e, int32_t n) { return e.a == n ? e.b : e.a; }

} // namespace

// --- Costs and routes on foot -----------------------------------------------------------

double Traffic::edge_cost(const PedEdge &e) const {
	double t = e.length / kWalkSpeed;
	switch (e.kind) {
		case PedEdgeKind::Walk:
			break;
		case PedEdgeKind::Ramp:
			t += 30.0 * std::abs(e.rise); // a bridge or subway costs extra effort
			break;
		case PedEdgeKind::Stairs:
			t = e.length / (0.6 * kWalkSpeed) + 20.0 * std::abs(e.rise);
			break;
		case PedEdgeKind::Crossing: {
			const NetCrossing &c = net_->ped.crossings[static_cast<size_t>(e.crossing)];
			if (c.unmarked) t = t * 2.0 + 20.0;
			else if (c.kind == CrossingKind::Signal) t += 20.0; // about half a cycle
			else if (c.kind == CrossingKind::Zebra) t += 3.0;
			else t += 10.0;
			break;
		}
	}
	return t;
}

bool Traffic::find_walk(int32_t from, int32_t to, std::vector<int32_t> &path, double *cost) const {
	path.clear();
	if (!net_) return false;
	const PedGraph &g = net_->ped;
	if (from < 0 || to < 0 || static_cast<size_t>(from) >= g.nodes.size() || static_cast<size_t>(to) >= g.nodes.size()) {
		return false;
	}
	if (from == to) {
		if (cost) *cost = 0.0;
		return true;
	}
	const size_t N = g.nodes.size();
	std::vector<double> best(N, kInf);
	std::vector<int32_t> via(N, -1);
	std::vector<char> closed(N, 0);
	using Entry = std::pair<double, int32_t>;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	const Vec2 goal = g.nodes[static_cast<size_t>(to)].pos;
	auto h = [&](int32_t n) { return (g.nodes[static_cast<size_t>(n)].pos - goal).length() / 1.5; };
	best[static_cast<size_t>(from)] = 0.0;
	open.push({ h(from), from });
	while (!open.empty()) {
		const int32_t n = open.top().second;
		open.pop();
		if (closed[static_cast<size_t>(n)]) continue;
		closed[static_cast<size_t>(n)] = 1;
		if (n == to) break;
		for (int32_t ei : g.adj[static_cast<size_t>(n)]) {
			const PedEdge &e = g.edges[static_cast<size_t>(ei)];
			const int32_t m = other_end(e, n);
			if (closed[static_cast<size_t>(m)]) continue;
			const double c = best[static_cast<size_t>(n)] + edge_cost(e);
			if (c < best[static_cast<size_t>(m)]) {
				best[static_cast<size_t>(m)] = c;
				via[static_cast<size_t>(m)] = ei;
				open.push({ c + h(m), m });
			}
		}
	}
	if (best[static_cast<size_t>(to)] >= kInf) return false;
	for (int32_t n = to; n != from;) {
		const int32_t ei = via[static_cast<size_t>(n)];
		path.push_back(ei);
		n = other_end(g.edges[static_cast<size_t>(ei)], n);
	}
	std::reverse(path.begin(), path.end());
	if (cost) *cost = best[static_cast<size_t>(to)];
	return true;
}

void Traffic::ped_costs() {
	ped_cost_.clear();
	if (!net_) return;
	const PedGraph &g = net_->ped;
	const size_t N = g.nodes.size();
	for (const PedSpawner &sp : g.spawners) {
		std::vector<double> best(N, kInf);
		using Entry = std::pair<double, int32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
		for (int32_t e : sp.entries) {
			best[static_cast<size_t>(e)] = 0.0;
			open.push({ 0.0, e });
		}
		while (!open.empty()) {
			const Entry top = open.top();
			open.pop();
			const int32_t n = top.second;
			if (top.first > best[static_cast<size_t>(n)]) continue;
			for (int32_t ei : g.adj[static_cast<size_t>(n)]) {
				const PedEdge &e = g.edges[static_cast<size_t>(ei)];
				const int32_t m = other_end(e, n);
				const double c = top.first + edge_cost(e);
				if (c < best[static_cast<size_t>(m)]) {
					best[static_cast<size_t>(m)] = c;
					open.push({ c, m });
				}
			}
		}
		ped_cost_.push_back(std::move(best));
	}
}

void Traffic::route_times() {
	rides_.clear();
	if (!net_) return;
	const Network &n = *net_;
	auto free_time = [&](int32_t l) {
		const NetLane &x = n.lanes[static_cast<size_t>(l)];
		return x.length / std::max(1.0, std::min(x.speed_limit, typical_driver(VehicleKind::Bus).max_speed));
	};
	for (size_t di = 0; di < n.depots.size(); ++di) {
		const NetDepot &d = n.depots[di];
		for (size_t ri = 0; ri < d.routes.size(); ++ri) {
			const NetRoute &r = d.routes[ri];
			if (r.stops.empty() || d.spawn_lanes.empty()) continue;
			RideTimes rt;
			rt.route = r.id;
			rt.depot = di;
			rt.index = ri;
			rt.headway = r.headway;
			rt.stops = r.stops;
			if (r.loop && rt.stops.size() > 1) rt.stops.push_back(rt.stops.front());
			int32_t cur = n.stops[static_cast<size_t>(rt.stops[0])].lane;
			double cur_s = n.stops[static_cast<size_t>(rt.stops[0])].s;
			double t = 0.0;
			bool ok = true;
			std::vector<int32_t> route;
			rt.at.push_back(0.0);
			for (size_t k = 1; k < rt.stops.size(); ++k) {
				const NetStop &st = n.stops[static_cast<size_t>(rt.stops[k])];
				if (!find_route_to_lane(cur, cur_s, st.lane, st.s, route, VehicleKind::Bus, true)) {
					ok = false;
					break;
				}
				double leg = free_time(cur);
				for (int32_t c : route) leg += free_time(c) + free_time(n.lanes[static_cast<size_t>(c)].to);
				if (route.empty()) leg = std::max(1.0, (st.s - cur_s) / 10.0);
				t += leg + 15.0; // and some time at the stop
				rt.at.push_back(t);
				cur = st.lane;
				cur_s = st.s;
			}
			if (ok) rides_.push_back(std::move(rt));
		}
	}
}

// --- Crossings ----------------------------------------------------------------------------

SignalLight Traffic::crossing_car_light(int32_t ci) const {
	if (ci < 0 || static_cast<size_t>(ci) >= mid_signal_.size()) return SignalLight::Green;
	switch (mid_signal_[static_cast<size_t>(ci)].state) {
		case 0:
			return SignalLight::Green;
		case 1:
			return SignalLight::Amber;
		default:
			return SignalLight::Red;
	}
}

WalkLight Traffic::crossing_walk_light(int32_t ci) const {
	if (!net_ || ci < 0 || static_cast<size_t>(ci) >= net_->ped.crossings.size()) return WalkLight::DontWalk;
	const NetCrossing &c = net_->ped.crossings[static_cast<size_t>(ci)];
	if (c.kind != CrossingKind::Signal) return WalkLight::Walk;
	if (c.push_button) {
		const uint8_t st = mid_signal_[static_cast<size_t>(ci)].state;
		return st == 3 ? WalkLight::Walk : st == 4 ? WalkLight::Flashing : WalkLight::DontWalk;
	}
	if (c.junction < 0) return WalkLight::DontWalk;
	return net_->junctions[static_cast<size_t>(c.junction)].walk_light(c.seg, static_cast<int64_t>(tick_), c.clear_ticks);
}

// Where a person is along a crossing's line (distance from a) and which way.
namespace {
struct OnCrossing {
	double t = 0.0;
	int8_t dir = 1;
};
} // namespace

void Traffic::peds_prepare() {
	if (!net_) return;
	const PedGraph &g = net_->ped;
	const size_t nc = g.crossings.size();
	if (on_crossing_.size() != nc) on_crossing_.assign(nc, {});
	if (waiting_at_.size() != nc) waiting_at_.assign(nc, {});
	if (mid_signal_.size() != nc) mid_signal_.assign(nc, MidSignal{});
	for (auto &v : on_crossing_) v.clear();
	for (auto &v : waiting_at_) v.clear();
	for (const Pedestrian &p : peds_) {
		if (p.edge < 0) continue;
		const PedEdge &e = g.edges[static_cast<size_t>(p.edge)];
		if (e.kind != PedEdgeKind::Crossing) continue;
		const NetCrossing &c = g.crossings[static_cast<size_t>(e.crossing)];
		// Offset of this edge along the line (two edges with a refuge).
		double off = 0.0;
		for (int32_t ce : c.edges) {
			if (ce == p.edge) break;
			off += g.edges[static_cast<size_t>(ce)].length;
		}
		const bool forward = p.from == e.a;
		const double t = forward ? off + p.d : off + e.length - p.d;
		if (p.state == PedState::Crossing) {
			on_crossing_[static_cast<size_t>(e.crossing)].push_back({ t, static_cast<int8_t>(forward ? 1 : -1) });
		} else if (p.state == PedState::WaitingToCross) {
			waiting_at_[static_cast<size_t>(e.crossing)].push_back(p.id);
			if (c.kind == CrossingKind::Zebra) {
				// About to step onto a zebra: cars that can still stop do.
				on_crossing_[static_cast<size_t>(e.crossing)].push_back({ t, static_cast<int8_t>(forward ? 2 : -2) });
			}
		}
	}
	// Mid-block push-button signals.
	const uint64_t now = tick_;
	auto secs = [&](uint64_t since) { return static_cast<double>(now - since) * config_.dt; };
	for (size_t ci = 0; ci < nc; ++ci) {
		const NetCrossing &c = g.crossings[ci];
		if (!c.push_button) continue;
		MidSignal &m = mid_signal_[ci];
		const double clear = static_cast<double>(c.clear_ticks) * 0.1;
		switch (m.state) {
			case 0: // cars green, at least 20 s
				if (!waiting_at_[ci].empty() && secs(m.since) >= 20.0) {
					m.state = 1;
					m.since = now;
				}
				break;
			case 1: // amber
				if (secs(m.since) >= 3.0) {
					m.state = 2;
					m.since = now;
				}
				break;
			case 2: // all red
				if (secs(m.since) >= 1.0) {
					m.state = 3;
					m.since = now;
				}
				break;
			case 3: // walk
				if (secs(m.since) >= 6.0) {
					m.state = 4;
					m.since = now;
				}
				break;
			case 4: // flashing
				if (secs(m.since) >= clear) {
					m.state = 5;
					m.since = now;
				}
				break;
			default: // clearance
				if (secs(m.since) >= 1.0 && on_crossing_[ci].empty()) {
					m.state = 0;
					m.since = now;
				}
				break;
		}
	}
}

// 0 no, 1 stop, 2 stop if it still can comfortably.
int Traffic::crossing_blocks(int32_t ci, int32_t lane, const Vehicle &v) const {
	(void)v;
	const NetCrossing &c = net_->ped.crossings[static_cast<size_t>(ci)];
	const CrossingSpan *sp = nullptr;
	for (const CrossingSpan &x : c.spans) {
		if (x.lane == lane) sp = &x;
	}
	if (!sp) return 0;
	if (c.push_button && crossing_car_light(ci) != SignalLight::Green) {
		// Amber: stop if it comfortably can; red: stop.
		return crossing_car_light(ci) == SignalLight::Amber ? 2 : 1;
	}
	int out = 0;
	for (const auto &o : on_crossing_[static_cast<size_t>(ci)]) {
		const int8_t dir = o.second;
		// Still ahead of (or on) this lane in its direction of travel.
		if (!(dir > 0 ? o.first < sp->t1 + 0.5 : o.first > sp->t0 - 0.5)) continue;
		if (dir == 2 || dir == -2) {
			out = std::max(out, 2);
		} else {
			return 1;
		}
	}
	return out;
}

// Whether a lane's traffic lets a person step onto [s0, s1] now.
bool Traffic::lane_clear_for(int32_t lane, double s0, double s1, double ped_time, bool zebra) const {
	const Network &n = *net_;
	const NetLane &l = n.lanes[static_cast<size_t>(lane)];
	auto vehicle_ok = [&](const Vehicle &o, double dist_to_s0) {
		if (dist_to_s0 < 0.0) return false; // already over the crossing
		if (dist_to_s0 > 80.0) return true;
		if (zebra) {
			// It can stop comfortably, or has stopped.
			if (o.v < 0.5) return true;
			return dist_to_s0 > o.v * o.v / (2.0 * o.drv.b) + 2.0;
		}
		if (o.v < 0.5) return dist_to_s0 > 2.0;
		return dist_to_s0 / o.v > ped_time + 1.0;
	};
	// Vehicles on the lane: on the span, or the first one before it.
	for (int32_t k : cars_[static_cast<size_t>(lane)]) {
		const Vehicle &o = veh_[static_cast<size_t>(k)];
		const double rear = o.s - o.drv.length;
		if (rear > s1) continue; // past it
		if (o.s >= s0) return false; // on it
		return vehicle_ok(o, s0 - o.s);
	}
	// Coming from the lanes before it (only those heading here).
	for (int32_t pl : l.prev) {
		const NetLane &p = n.lanes[static_cast<size_t>(pl)];
		const std::vector<int32_t> &list = cars_[static_cast<size_t>(pl)];
		if (list.empty()) continue;
		const Vehicle &o = veh_[static_cast<size_t>(list.front())];
		if (p.kind == NetLaneKind::Road && next_connector(o, pl, o.ri) != lane) continue;
		if (!vehicle_ok(o, p.length - o.s + s0)) return false;
	}
	return true;
}

bool Traffic::cross_ok(const Pedestrian &p, int32_t edge, int32_t from) const {
	const PedGraph &g = net_->ped;
	const PedEdge &e = g.edges[static_cast<size_t>(edge)];
	const NetCrossing &c = g.crossings[static_cast<size_t>(e.crossing)];
	if (c.kind == CrossingKind::Signal) {
		return crossing_walk_light(e.crossing) == WalkLight::Walk;
	}
	// The part of the line this edge covers.
	double off = 0.0;
	for (int32_t ce : c.edges) {
		if (ce == edge) break;
		off += g.edges[static_cast<size_t>(ce)].length;
	}
	const double lo = off, hi = off + e.length;
	const bool forward = from == e.a;
	const double waited = p.wait_since ? static_cast<double>(tick_ - p.wait_since) * config_.dt : 0.0;
	const double hurry = waited > 60.0 ? 0.6 : 1.0; // after a long wait, a smaller gap will do
	for (const CrossingSpan &sp : c.spans) {
		if (sp.t1 < lo || sp.t0 > hi) continue;
		// Time to walk past the far side of this lane.
		const double far = forward ? sp.t1 - lo : hi - sp.t0;
		const double t = std::max(0.0, far) / p.speed + 1.5;
		if (!lane_clear_for(sp.lane, sp.s0, sp.s1, t * hurry, c.kind == CrossingKind::Zebra)) return false;
	}
	return true;
}

// --- Trips ------------------------------------------------------------------------------------

void Traffic::start_walk(Pedestrian &p, int32_t to_node) {
	const PedGraph &g = net_->ped;
	int32_t at = p.from;
	if (p.edge >= 0) {
		// Finish the current edge first.
		at = other_end(g.edges[static_cast<size_t>(p.edge)], p.from);
	}
	p.target = to_node;
	std::vector<int32_t> path;
	if (!find_walk(at, to_node, path)) {
		p.path.clear();
		p.pi = 0;
		return;
	}
	p.path = std::move(path);
	p.pi = 0;
}

uint32_t Traffic::add_pedestrian(int32_t from_node, int32_t to_node, double speed) {
	if (!net_) return 0;
	const PedGraph &g = net_->ped;
	if (from_node < 0 || static_cast<size_t>(from_node) >= g.nodes.size()) return 0;
	Pedestrian p;
	p.id = next_ped_id_++;
	p.speed = speed;
	p.from = from_node;
	p.edge = -1;
	p.spawn_tick = tick_;
	std::vector<int32_t> path;
	if (!find_walk(from_node, to_node, path)) return 0;
	p.target = to_node;
	p.path = std::move(path);
	peds_.push_back(std::move(p));
	return peds_.back().id;
}

namespace {

double od_weight(const PedSpawner &o, NodeId to) {
	for (const OdWeight &w : o.od) {
		if (w.to == to) return w.weight;
	}
	return 1.0;
}

} // namespace

void Traffic::people_demand() {
	if (!net_ || !people_on_) return;
	const Network &n = *net_;
	const PedGraph &g = n.ped;
	const size_t ns = g.spawners.size();
	if (ped_cost_.size() != ns) ped_costs();
	auto walk_cost = [&](size_t from_spawner, int32_t node) {
		return ped_cost_[from_spawner][static_cast<size_t>(node)];
	};
	auto to_dest = [&](size_t dest, int32_t node) { return ped_cost_[dest][static_cast<size_t>(node)]; };
	auto platform = [&](int32_t stop) {
		for (const PedStop &ps : g.stops) {
			if (ps.stop == stop) return ps.node;
		}
		return -1;
	};
	const bool coaches_run = !n.coach_lines.empty() && n.main_station >= 0 && platform(n.main_station) >= 0;
	for (size_t k = 0; k < ns; ++k) {
		const PedSpawner &o = g.spawners[k];
		if (o.people <= 0.0) continue;
		const double p = o.people * config_.demand * config_.dt / 3600.0;
		if (!(rng_.uniform() < p)) continue;
		if (config_.max_pedestrians > 0 && peds_.size() >= config_.max_pedestrians) continue;
		Pedestrian ped;
		ped.id = next_ped_id_++;
		ped.speed = rng_.range(1.2, 1.5);
		ped.origin = static_cast<int32_t>(k);
		ped.spawn_tick = tick_ + 1;
		// Leaving by coach?
		if (coaches_run && rng_.uniform() < config_.coach_share) {
			ped.coach = true;
			ped.dest = -1;
			ped.board = n.main_station;
			const int32_t pl = platform(n.main_station);
			int32_t entry = o.entries[0];
			double best = kInf;
			for (int32_t e : o.entries) {
				const double d = (g.nodes[static_cast<size_t>(e)].pos - g.nodes[static_cast<size_t>(pl)].pos).length();
				if (d < best) {
					best = d;
					entry = e;
				}
			}
			ped.from = entry;
			std::vector<int32_t> path;
			if (!find_walk(entry, pl, path)) {
				++stats_.unroutable;
				continue;
			}
			ped.target = pl;
			ped.path = std::move(path);
			++stats_.trips;
			++stats_.trips_coach;
			peds_.push_back(std::move(ped));
			continue;
		}
		// Destination by weight among the other spawn points that take people.
		double total = 0.0;
		for (size_t j = 0; j < ns; ++j) {
			if (j != k && g.spawners[j].sink) total += od_weight(o, g.spawners[j].node);
		}
		if (total <= 0.0) continue;
		double x = rng_.uniform() * total;
		size_t dest = ns;
		for (size_t j = 0; j < ns; ++j) {
			if (j == k || !g.spawners[j].sink) continue;
			const double w = od_weight(o, g.spawners[j].node);
			if (x < w) {
				dest = j;
				break;
			}
			x -= w;
		}
		if (dest >= ns) {
			for (size_t j = ns; j-- > 0;) {
				if (j != k && g.spawners[j].sink) {
					dest = j;
					break;
				}
			}
		}
		const PedSpawner &d = g.spawners[dest];
		// Options on generalized cost: walk x2, wait x2, ride, a penalty per transfer.
		auto vary = [&]() { return 1.0 + config_.cost_variance * rng_.symmetric(); };
		double best = kInf;
		int mode = -1; // 0 walk, 1 bus, 2 bike, 3 car
		double walk = kInf;
		for (int32_t e : d.entries) walk = std::min(walk, walk_cost(k, e));
		if (walk < kInf) {
			best = 2.0 * walk * vary();
			mode = 0;
		}
		// Bus: one route, or two with a transfer at a shared stop.
		struct Ride {
			size_t r;
			size_t i, j;
		};
		Ride r1{ 0, 0, 0 }, r2{ 0, 0, 0 };
		bool transfer = false;
		double bus = kInf;
		for (size_t a = 0; a < rides_.size(); ++a) {
			const RideTimes &ra = rides_[a];
			for (size_t i = 0; i + 1 < ra.stops.size(); ++i) {
				const int32_t pi = platform(ra.stops[i]);
				if (pi < 0) continue;
				const double w0 = walk_cost(k, pi);
				if (w0 >= kInf) continue;
				for (size_t j = i + 1; j < ra.stops.size(); ++j) {
					const int32_t pj = platform(ra.stops[j]);
					if (pj < 0 || ra.stops[j] == ra.stops[i]) continue;
					const double ride = ra.at[j] - ra.at[i];
					const double w1 = to_dest(dest, pj);
					if (w1 < kInf) {
						const double c = 2.0 * (w0 + w1) + ra.headway + ride;
						if (c < bus) {
							bus = c;
							r1 = Ride{ a, i, j };
							transfer = false;
						}
					}
					// Transfer at stop j to another route.
					for (size_t b = 0; b < rides_.size(); ++b) {
						if (b == a) continue;
						const RideTimes &rb = rides_[b];
						for (size_t i2 = 0; i2 + 1 < rb.stops.size(); ++i2) {
							if (rb.stops[i2] != ra.stops[j]) continue;
							for (size_t j2 = i2 + 1; j2 < rb.stops.size(); ++j2) {
								const int32_t pj2 = platform(rb.stops[j2]);
								if (pj2 < 0) continue;
								const double w2 = to_dest(dest, pj2);
								if (w2 >= kInf) continue;
								const double c = 2.0 * (w0 + w2) + ra.headway + rb.headway + ride +
										(rb.at[j2] - rb.at[i2]) + config_.transfer_penalty;
								if (c < bus) {
									bus = c;
									r1 = Ride{ a, i, j };
									r2 = Ride{ b, i2, j2 };
									transfer = true;
								}
							}
						}
					}
				}
			}
		}
		if (bus < kInf) {
			const double c = bus * vary();
			if (c < best) {
				best = c;
				mode = 1;
			}
		}
		const double straight = (d.pos - o.pos).length();
		if (o.road && d.road) {
			const NetSpawner *vs = n.spawner_at(o.node);
			const NetSpawner *vd = n.spawner_at(d.node);
			const bool can = vs && vd && vd->config.sink;
			if (can && !vs->bike_lanes.empty() && rng_.uniform() < config_.bike_owners) {
				const double c = (straight * 1.3 / 5.0 + 60.0) * vary();
				if (c < best) {
					best = c;
					mode = 2;
				}
			}
			if (can && !vs->spawn_lanes.empty() && rng_.uniform() < config_.car_owners) {
				const double c = (straight * 1.4 / 10.0 + 180.0) * vary();
				if (c < best) {
					best = c;
					mode = 3;
				}
			}
		}
		if (mode < 0) {
			++stats_.unroutable;
			continue;
		}
		++stats_.trips;
		if (mode == 2 || mode == 3) {
			// A vehicle trip from the same spawn point.
			size_t vk = 0;
			while (vk < n.spawners.size() && n.spawners[vk].node != o.node) ++vk;
			if (vk >= n.spawners.size()) continue;
			if (mode == 2) {
				++stats_.trips_bike;
				spawn_bike_to(vk, d.node);
			} else {
				++stats_.trips_car;
				if (car_trips_.size() != n.spawners.size()) car_trips_.assign(n.spawners.size(), {});
				if (car_trips_[vk].size() < 50) car_trips_[vk].push_back(d.node);
			}
			continue;
		}
		ped.dest = static_cast<int32_t>(dest);
		int32_t target = -1;
		if (mode == 1) {
			++stats_.trips_bus;
			const RideTimes &ra = rides_[r1.r];
			ped.route = ra.route;
			ped.board = ra.stops[r1.i];
			ped.alight = ra.stops[r1.j];
			if (transfer) {
				const RideTimes &rb = rides_[r2.r];
				ped.route2 = rb.route;
				ped.board2 = rb.stops[r2.i];
				ped.alight2 = rb.stops[r2.j];
			}
			target = platform(ped.board);
		} else {
			++stats_.trips_walk;
			double bd = kInf;
			for (int32_t e : d.entries) {
				const double c = walk_cost(k, e);
				if (c < bd) {
					bd = c;
					target = e;
				}
			}
		}
		// Start at the entry nearest the first target.
		int32_t entry = o.entries[0];
		double near = kInf;
		for (int32_t e : o.entries) {
			const double dd = (g.nodes[static_cast<size_t>(e)].pos - g.nodes[static_cast<size_t>(target)].pos).length();
			if (dd < near) {
				near = dd;
				entry = e;
			}
		}
		ped.from = entry;
		std::vector<int32_t> path;
		if (!find_walk(entry, target, path)) {
			++stats_.unroutable;
			continue;
		}
		ped.target = target;
		ped.path = std::move(path);
		peds_.push_back(std::move(ped));
	}
}

// --- Moving people ------------------------------------------------------------------------------

void Traffic::ped_arrived(size_t i) {
	Pedestrian &p = peds_[i];
	const Network &n = *net_;
	const PedGraph &g = n.ped;
	if (p.board >= 0 && p.state != PedState::Riding && (p.coach || p.route != 0)) {
		// At the stop: wait for the bus.
		p.state = PedState::WaitingForBus;
		p.wait_since = tick_ + 1;
		p.path.clear();
		p.pi = 0;
		return;
	}
	if (p.dest >= 0) {
		const PedSpawner &d = g.spawners[static_cast<size_t>(p.dest)];
		const int32_t at = p.edge >= 0 ? other_end(g.edges[static_cast<size_t>(p.edge)], p.from) : p.from;
		if (std::find(d.entries.begin(), d.entries.end(), at) != d.entries.end()) {
			p.done = true;
			++stats_.people_arrived;
			trip_time_sum_ += 0.0;
			return;
		}
		// Off the planned path (after a ride, say): walk on to the destination.
		int32_t best = d.entries[0];
		double bd = kInf;
		for (int32_t e : d.entries) {
			const double c = (g.nodes[static_cast<size_t>(e)].pos - g.nodes[static_cast<size_t>(at)].pos).length();
			if (c < bd) {
				bd = c;
				best = e;
			}
		}
		p.from = at;
		p.edge = -1;
		p.d = 0.0;
		start_walk(p, best);
		if (p.path.empty()) {
			p.done = true;
			++stats_.unroutable;
		}
		return;
	}
	p.done = true;
}

void Traffic::peds_tick() {
	if (!net_ || peds_.empty()) return;
	const PedGraph &g = net_->ped;
	const double dt = config_.dt;
	for (size_t i = 0; i < peds_.size(); ++i) {
		Pedestrian &p = peds_[i];
		p.prev_edge = p.edge;
		p.prev_from = p.from;
		p.prev_d = p.d;
		if (p.done || p.state == PedState::Riding || p.state == PedState::WaitingForBus) continue;
		if (p.state == PedState::WaitingToCross) {
			if (!cross_ok(p, p.edge, p.from)) continue;
			p.state = PedState::Crossing;
			++stats_.crossings;
			crossing_wait_sum_ += static_cast<double>(tick_ + 1 - p.wait_since) * dt;
			p.wait_since = 0;
		}
		double step = p.speed * dt;
		for (int guard = 0; guard < 8 && step > 0.0; ++guard) {
			if (p.edge < 0) {
				// At a node: take the next edge.
				if (p.pi >= p.path.size()) {
					ped_arrived(i);
					break;
				}
				const int32_t e = p.path[p.pi++];
				p.edge = e;
				p.d = 0.0;
				const PedEdge &pe = g.edges[static_cast<size_t>(e)];
				if (pe.kind == PedEdgeKind::Crossing) {
					p.state = PedState::WaitingToCross;
					p.wait_since = tick_ + 1;
					if (cross_ok(p, p.edge, p.from)) {
						p.state = PedState::Crossing;
						++stats_.crossings;
						p.wait_since = 0;
					} else {
						break;
					}
				} else {
					p.state = PedState::Walking;
				}
			}
			const PedEdge &pe = g.edges[static_cast<size_t>(p.edge)];
			const double f = pe.kind == PedEdgeKind::Stairs ? 0.6 : 1.0;
			const double room = (pe.length - p.d) / f;
			if (step < room) {
				p.d += step * f;
				step = 0.0;
			} else {
				step -= room;
				p.from = other_end(pe, p.from);
				p.edge = -1;
				p.d = 0.0;
				if (p.state == PedState::Crossing) p.state = PedState::Walking;
			}
		}
	}
	// Late arrivals board a bus still at their stop.
	size_t w = 0;
	for (size_t i = 0; i < peds_.size(); ++i) {
		if (peds_[i].done) continue;
		if (w != i) peds_[w] = std::move(peds_[i]);
		++w;
	}
	peds_.resize(w);
}

int Traffic::serve_stop(Vehicle &v, int32_t stop, Waypoint &wp, bool late) {
	const Network &n = *net_;
	const PedGraph &g = n.ped;
	int32_t pl = -1;
	for (const PedStop &ps : g.stops) {
		if (ps.stop == stop) pl = ps.node;
	}
	if (stop_acc_.size() != n.stops.size()) stop_acc_.assign(n.stops.size(), StopAcc{});
	StopAcc &acc = stop_acc_[static_cast<size_t>(stop)];
	const bool coach = v.kind == VehicleKind::Coach;
	const int doors = coach ? config_.coach_doors : config_.bus_doors;
	const int cap = coach ? config_.coach_capacity : config_.bus_capacity;
	int alighted = 0, boarded = 0;
	// Alighting first.
	std::vector<uint32_t> stay;
	if (late) stay = v.riders;
	for (uint32_t pid : late ? std::vector<uint32_t>{} : v.riders) {
		const int32_t pi = find_pedestrian(pid);
		if (pi < 0) continue;
		Pedestrian &p = peds_[static_cast<size_t>(pi)];
		if (p.alight != stop || pl < 0) {
			stay.push_back(pid);
			continue;
		}
		++alighted;
		++acc.alighted;
		++stats_.alighted;
		p.vehicle = kNoId;
		p.state = PedState::Walking;
		p.from = pl;
		p.edge = -1;
		p.d = 0.0;
		p.prev_edge = -1;
		p.prev_from = pl;
		if (p.route2 != 0) {
			// Transfer: wait here (or walk to the next stop) for the second bus.
			p.route = p.route2;
			p.board = p.board2;
			p.alight = p.alight2;
			p.route2 = 0;
			p.board2 = p.alight2 = -1;
			int32_t next = pl;
			for (const PedStop &ps : g.stops) {
				if (ps.stop == p.board) next = ps.node;
			}
			start_walk(p, next);
		} else {
			p.board = -1;
			p.route = 0;
			p.alight = -1;
			p.path.clear();
			p.pi = 0;
			p.target = -1;
		}
	}
	v.riders = std::move(stay);
	// Coaches bring people to the city.
	if (!late && coach && stop == n.main_station && pl >= 0) {
		const int arrivals = static_cast<int>(rng_.range(10.0, 41.0));
		std::vector<size_t> sinks;
		for (size_t j = 0; j < g.spawners.size(); ++j) {
			if (g.spawners[j].sink) sinks.push_back(j);
		}
		for (int a = 0; a < arrivals && !sinks.empty(); ++a) {
			Pedestrian p;
			p.id = next_ped_id_++;
			p.speed = rng_.range(1.2, 1.5);
			p.from = pl;
			p.spawn_tick = tick_ + 1;
			p.dest = static_cast<int32_t>(sinks[static_cast<size_t>(rng_.next_u64() % sinks.size())]);
			const PedSpawner &d = g.spawners[static_cast<size_t>(p.dest)];
			p.target = d.entries[0];
			std::vector<int32_t> path;
			if (!find_walk(pl, p.target, path)) continue;
			p.path = std::move(path);
			peds_.push_back(std::move(p));
			++alighted;
			++stats_.coach_passengers;
		}
	}
	// Boarding: the longest waiting first.
	std::vector<size_t> waiting;
	for (size_t i = 0; i < peds_.size(); ++i) {
		const Pedestrian &p = peds_[i];
		if (p.state != PedState::WaitingForBus || p.board != stop) continue;
		if (coach ? !p.coach : (p.coach || p.route != v.bus_route)) continue;
		waiting.push_back(i);
	}
	std::sort(waiting.begin(), waiting.end(), [&](size_t a, size_t b) {
		return peds_[a].wait_since != peds_[b].wait_since ? peds_[a].wait_since < peds_[b].wait_since
														  : peds_[a].id < peds_[b].id;
	});
	for (size_t i : waiting) {
		if (static_cast<int>(v.riders.size()) >= cap) break;
		Pedestrian &p = peds_[i];
		p.state = PedState::Riding;
		p.vehicle = v.id;
		const double waited = static_cast<double>(tick_ + 1 - p.wait_since) * config_.dt;
		acc.wait_sum += waited;
		wait_sum_ += waited;
		++acc.boarded;
		++stats_.boarded;
		++boarded;
		v.riders.push_back(p.id);
	}
	if (!coach && !late) {
		wp.dwell = config_.door_time + config_.board_time * static_cast<double>(alighted + boarded) / std::max(1, doors);
	}
	return boarded;
}

uint64_t Traffic::boarding_ticks(const Vehicle &v, int people) const {
	const int doors = v.kind == VehicleKind::Coach ? config_.coach_doors : config_.bus_doors;
	const double sec = config_.board_time * static_cast<double>(people) / std::max(1, doors);
	return static_cast<uint64_t>(std::llround(sec / config_.dt));
}

void Traffic::bus_departs(Vehicle &v, int32_t stop) {
	if (stop < 0) return;
	if (stop_acc_.size() != net_->stops.size()) stop_acc_.assign(net_->stops.size(), StopAcc{});
	const bool coach = v.kind == VehicleKind::Coach;
	uint64_t left = 0;
	for (const Pedestrian &p : peds_) {
		if (p.state != PedState::WaitingForBus || p.board != stop) continue;
		if (coach ? p.coach : (!p.coach && p.route == v.bus_route)) ++left;
	}
	stop_acc_[static_cast<size_t>(stop)].left_behind += left;
	stats_.left_behind += left;
	if (!coach && v.bus_route != 0) {
		auto &acc = load_acc_[v.bus_route];
		if (acc.size() <= v.wi) acc.resize(v.wi + 1, { 0.0, 0 });
		acc[v.wi].first += static_cast<double>(v.riders.size());
		++acc[v.wi].second;
	}
	if (coach && stop == net_->main_station) {
		// Riders leave the map with the coach: their trip is done.
		for (uint32_t pid : v.riders) {
			const int32_t pi = find_pedestrian(pid);
			if (pi < 0) continue;
			peds_[static_cast<size_t>(pi)].done = true;
			++stats_.people_arrived;
		}
		v.riders.clear();
	}
}

// --- Network changes -----------------------------------------------------------------------

std::vector<Traffic::PedSaved> Traffic::peds_save() const {
	std::vector<PedSaved> out;
	if (!net_) return out;
	const Network &n = *net_;
	const PedGraph &g = n.ped;
	auto stop_id = [&](int32_t s) { return s >= 0 && static_cast<size_t>(s) < n.stops.size() ? n.stops[static_cast<size_t>(s)].id : 0u; };
	auto spawner_node = [&](int32_t k) { return k >= 0 && static_cast<size_t>(k) < g.spawners.size() ? g.spawners[static_cast<size_t>(k)].node : kNoId; };
	for (const Pedestrian &p : peds_) {
		PedSaved sv;
		int32_t at = p.from;
		if (p.edge >= 0) {
			const PedEdge &e = g.edges[static_cast<size_t>(p.edge)];
			if (p.d > 0.5 * e.length) at = other_end(e, p.from);
		}
		if (at >= 0) {
			sv.at = g.nodes[static_cast<size_t>(at)].pos;
			sv.at_level = g.nodes[static_cast<size_t>(at)].level;
		}
		if (p.target >= 0) {
			sv.has_target = true;
			sv.target = g.nodes[static_cast<size_t>(p.target)].pos;
			sv.target_level = g.nodes[static_cast<size_t>(p.target)].level;
		}
		sv.board = stop_id(p.board);
		sv.alight = stop_id(p.alight);
		sv.board2 = stop_id(p.board2);
		sv.alight2 = stop_id(p.alight2);
		sv.origin = spawner_node(p.origin);
		sv.dest = spawner_node(p.dest);
		out.push_back(sv);
	}
	return out;
}

void Traffic::peds_reset_network() {
	on_crossing_.clear();
	waiting_at_.clear();
	mid_signal_.assign(net_ ? net_->ped.crossings.size() : 0, MidSignal{});
	for (MidSignal &m : mid_signal_) m.since = tick_;
	car_trips_.assign(net_ ? net_->spawners.size() : 0, {});
	people_on_ = false;
	ped_cost_.clear();
	rides_.clear();
	if (!net_) return;
	for (const PedSpawner &sp : net_->ped.spawners) people_on_ |= sp.people > 0.0;
	if (stop_acc_.size() != net_->stops.size()) stop_acc_.assign(net_->stops.size(), StopAcc{});
	if (people_on_) {
		ped_costs();
		route_times();
	}
}

void Traffic::peds_restore(const std::vector<PedSaved> &saved) {
	if (!net_) {
		peds_.clear();
		return;
	}
	const Network &n = *net_;
	const PedGraph &g = n.ped;
	std::map<std::tuple<double, double, int>, int32_t> by_pos;
	for (size_t i = 0; i < g.nodes.size(); ++i) {
		by_pos.emplace(std::make_tuple(g.nodes[i].pos.x, g.nodes[i].pos.y, g.nodes[i].level), static_cast<int32_t>(i));
	}
	auto node_at = [&](Vec2 p, int level) {
		auto it = by_pos.find(std::make_tuple(p.x, p.y, level));
		return it != by_pos.end() ? it->second : -1;
	};
	auto stop_idx = [&](uint32_t id) { return id ? n.stop_index(id) : -1; };
	auto spawner_idx = [&](NodeId node) {
		for (size_t k = 0; k < g.spawners.size(); ++k) {
			if (g.spawners[k].node == node) return static_cast<int32_t>(k);
		}
		return -1;
	};
	auto platform = [&](int32_t stop) {
		for (const PedStop &ps : g.stops) {
			if (ps.stop == stop) return ps.node;
		}
		return -1;
	};
	std::vector<Pedestrian> kept;
	for (size_t i = 0; i < peds_.size() && i < saved.size(); ++i) {
		Pedestrian p = peds_[i];
		const PedSaved &sv = saved[i];
		p.board = stop_idx(sv.board);
		p.alight = stop_idx(sv.alight);
		p.board2 = stop_idx(sv.board2);
		p.alight2 = stop_idx(sv.alight2);
		p.origin = sv.origin != kNoId ? spawner_idx(sv.origin) : -1;
		p.dest = sv.dest != kNoId ? spawner_idx(sv.dest) : -1;
		if (sv.dest != kNoId && p.dest < 0) continue; // its destination is gone
		if ((sv.board && p.board < 0) || (sv.alight && p.alight < 0)) continue;
		if (p.state == PedState::Riding) {
			if (find_vehicle(p.vehicle) < 0) continue;
			p.edge = p.prev_edge = -1;
			p.from = p.prev_from = -1;
			p.target = -1;
			kept.push_back(std::move(p));
			continue;
		}
		const int32_t at = p.state == PedState::WaitingForBus ? platform(p.board) : node_at(sv.at, sv.at_level);
		if (at < 0) continue;
		p.from = p.prev_from = at;
		p.edge = p.prev_edge = -1;
		p.d = p.prev_d = 0.0;
		p.path.clear();
		p.pi = 0;
		if (p.state == PedState::WaitingForBus) {
			p.target = at;
			kept.push_back(std::move(p));
			continue;
		}
		p.state = PedState::Walking;
		p.wait_since = 0;
		int32_t target = sv.has_target ? node_at(sv.target, sv.target_level) : -1;
		if (target < 0 && p.board >= 0) target = platform(p.board);
		if (target < 0 && p.dest >= 0) target = g.spawners[static_cast<size_t>(p.dest)].entries[0];
		if (target < 0) continue;
		start_walk(p, target);
		if (p.path.empty() && at != target) continue;
		kept.push_back(std::move(p));
	}
	peds_ = std::move(kept);
	// Riders whose person is gone get off.
	for (Vehicle &v : veh_) {
		std::vector<uint32_t> stay;
		for (uint32_t id : v.riders) {
			if (find_pedestrian(id) >= 0) stay.push_back(id);
		}
		v.riders = std::move(stay);
	}
}

// --- Queries ---------------------------------------------------------------------------------

int32_t Traffic::find_pedestrian(uint32_t id) const {
	auto it = std::lower_bound(peds_.begin(), peds_.end(), id, [](const Pedestrian &p, uint32_t x) { return p.id < x; });
	return it != peds_.end() && it->id == id ? static_cast<int32_t>(it - peds_.begin()) : -1;
}

namespace {

Vec2 ped_at(const PedGraph &g, int32_t edge, int32_t from, double d) {
	if (edge < 0) return g.nodes[static_cast<size_t>(from)].pos;
	const PedEdge &e = g.edges[static_cast<size_t>(edge)];
	const Vec2 a = g.nodes[static_cast<size_t>(from)].pos;
	const Vec2 b = g.nodes[static_cast<size_t>(other_end(e, from))].pos;
	const double f = e.length > 0.0 ? std::clamp(d / e.length, 0.0, 1.0) : 0.0;
	return a + (b - a) * f;
}

} // namespace

Pose Traffic::ped_pose(size_t i, double alpha) const {
	const Pedestrian &p = peds_[i];
	const PedGraph &g = net_->ped;
	Pose out;
	if (p.state == PedState::Riding) {
		const int32_t vi = find_vehicle(p.vehicle);
		if (vi >= 0) return pose(static_cast<size_t>(vi), alpha);
		return out;
	}
	const Vec2 a = ped_at(g, p.prev_edge, p.prev_from < 0 ? p.from : p.prev_from, p.prev_d);
	const Vec2 b = ped_at(g, p.edge, p.from, p.d);
	out.pos = a + (b - a) * alpha;
	const Vec2 d = b - a;
	out.dir = d.length() > 1e-6 ? d.normalized() : Vec2{ 1, 0 };
	if (p.state == PedState::WaitingForBus || p.state == PedState::WaitingToCross) {
		// Stand a little apart: a small ring by id.
		const uint32_t k = p.id * 2654435761u;
		const double ang = static_cast<double>(k % 360u) * (3.14159265358979 / 180.0);
		const double r = p.state == PedState::WaitingForBus ? 0.6 + static_cast<double>((k >> 9) % 100u) * 0.012 : 0.4;
		out.pos = out.pos + Vec2{ std::cos(ang), std::sin(ang) } * r;
	}
	return out;
}

int Traffic::ped_level(size_t i) const {
	const Pedestrian &p = peds_[i];
	const PedGraph &g = net_->ped;
	if (p.state == PedState::Riding) {
		const int32_t vi = find_vehicle(p.vehicle);
		return vi >= 0 ? level_of(static_cast<size_t>(vi)) : 0;
	}
	if (p.edge < 0) return g.nodes[static_cast<size_t>(p.from)].level;
	const PedEdge &e = g.edges[static_cast<size_t>(p.edge)];
	const int32_t to = other_end(e, p.from);
	return p.d < 0.5 * e.length ? g.nodes[static_cast<size_t>(p.from)].level : g.nodes[static_cast<size_t>(to)].level;
}

PedInfo Traffic::ped_info(uint32_t id) const {
	PedInfo out;
	out.id = id;
	const int32_t i = find_pedestrian(id);
	if (i < 0 || !net_) return out;
	const Pedestrian &p = peds_[static_cast<size_t>(i)];
	const PedGraph &g = net_->ped;
	out.found = true;
	out.state = p.state;
	out.speed = p.speed;
	out.waited = p.wait_since ? static_cast<double>(tick_ - std::min(tick_, p.wait_since)) * config_.dt : 0.0;
	out.trip_time = static_cast<double>(tick_ - std::min(tick_, p.spawn_tick)) * config_.dt;
	out.origin = p.origin >= 0 ? g.spawners[static_cast<size_t>(p.origin)].node : kNoId;
	out.dest = p.dest >= 0 ? g.spawners[static_cast<size_t>(p.dest)].node : kNoId;
	out.board = p.board;
	out.alight = p.alight;
	out.route = p.route;
	out.vehicle = p.vehicle;
	if (p.state != PedState::Riding && p.state != PedState::WaitingForBus) {
		out.route_line.push_back(ped_at(g, p.edge, p.from, p.d));
		int32_t at = p.edge >= 0 ? other_end(g.edges[static_cast<size_t>(p.edge)], p.from) : p.from;
		out.route_line.push_back(g.nodes[static_cast<size_t>(at)].pos);
		for (size_t k = p.pi; k < p.path.size(); ++k) {
			at = other_end(g.edges[static_cast<size_t>(p.path[k])], at);
			out.route_line.push_back(g.nodes[static_cast<size_t>(at)].pos);
		}
	}
	return out;
}

std::vector<StopStats> Traffic::stop_stats() const {
	std::vector<StopStats> out;
	if (!net_) return out;
	for (size_t s = 0; s < net_->stops.size(); ++s) {
		StopStats st;
		st.stop = static_cast<int32_t>(s);
		if (s < stop_acc_.size()) {
			const StopAcc &a = stop_acc_[s];
			st.boarded = a.boarded;
			st.alighted = a.alighted;
			st.left_behind = a.left_behind;
			st.mean_wait = a.boarded ? a.wait_sum / static_cast<double>(a.boarded) : 0.0;
		}
		for (const Pedestrian &p : peds_) {
			if (p.state == PedState::WaitingForBus && p.board == static_cast<int32_t>(s)) ++st.waiting;
		}
		out.push_back(st);
	}
	return out;
}

std::vector<RouteLoad> Traffic::route_loads() const {
	std::vector<RouteLoad> out;
	for (const auto &kv : load_acc_) {
		RouteLoad rl;
		rl.route = kv.first;
		for (const auto &x : kv.second) rl.load.push_back(x.second ? x.first / static_cast<double>(x.second) : 0.0);
		out.push_back(rl);
	}
	return out;
}

} // namespace tsim
