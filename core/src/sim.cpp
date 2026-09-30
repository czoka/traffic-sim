#include "tsim/sim.h"

#include "tsim/hash.h"

#include <algorithm>
#include <cmath>

namespace tsim {

Simulation::Simulation(const Map &map, SimConfig config) : map_(map), config_(config) { on_map_changed(); }

int32_t Simulation::dense_index(LaneId id) const {
	auto it = std::lower_bound(lanes_.begin(), lanes_.end(), id, [](const LaneRt &l, LaneId v) { return l.id < v; });
	if (it == lanes_.end() || it->id != id) {
		return -1;
	}
	return static_cast<int32_t>(it - lanes_.begin());
}

void Simulation::on_map_changed() {
	// Remember lane IDs of vehicles, rebuild the dense index, then remap.
	std::vector<LaneId> cur_ids(id_.size()), prev_ids(id_.size());
	for (size_t i = 0; i < id_.size(); ++i) {
		cur_ids[i] = lanes_[lane_[i]].id;
		prev_ids[i] = lanes_[prev_lane_[i]].id;
	}

	lanes_.clear();
	lanes_.reserve(map_.lanes().size());
	for (const Lane &l : map_.lanes()) { // ascending ID order
		LaneRt rt;
		rt.id = l.id;
		rt.lane = &l;
		rt.length = l.length;
		lanes_.push_back(std::move(rt));
	}
	for (LaneRt &rt : lanes_) {
		rt.next = rt.lane->next.empty() ? -1 : dense_index(rt.lane->next.front());
	}

	size_t w = 0;
	for (size_t i = 0; i < id_.size(); ++i) {
		const int32_t cur = dense_index(cur_ids[i]);
		if (cur < 0) {
			continue; // lane removed: drop vehicle
		}
		const int32_t prev = dense_index(prev_ids[i]);
		id_[w] = id_[i];
		lane_[w] = static_cast<uint32_t>(cur);
		prev_lane_[w] = static_cast<uint32_t>(prev < 0 ? cur : prev);
		s_[w] = std::min(s_[i], lanes_[static_cast<size_t>(cur)].length);
		prev_s_[w] = prev < 0 ? s_[w] : prev_s_[i];
		v_[w] = v_[i];
		acc_[w] = acc_[i];
		params_[w] = params_[i];
		++w;
	}
	id_.resize(w);
	lane_.resize(w);
	prev_lane_.resize(w);
	s_.resize(w);
	prev_s_.resize(w);
	v_.resize(w);
	acc_.resize(w);
	params_.resize(w);

	for (size_t i = 0; i < w; ++i) {
		lanes_[lane_[i]].cars.push_back(static_cast<uint32_t>(i));
	}
	for (LaneRt &rt : lanes_) {
		sort_lane(rt.cars);
	}
}

void Simulation::clear_vehicles() {
	id_.clear();
	lane_.clear();
	s_.clear();
	v_.clear();
	acc_.clear();
	params_.clear();
	prev_lane_.clear();
	prev_s_.clear();
	for (LaneRt &rt : lanes_) {
		rt.cars.clear();
	}
}

uint32_t Simulation::spawn_even(uint32_t count, uint64_t seed) {
	clear_vehicles();
	rng_.reseed(seed);
	tick_ = 0;
	next_vehicle_id_ = 1;

	const double kLength = 4.5;
	const double kJam = 2.0;
	const double kMinSpacing = kLength + kJam + 0.5;

	std::vector<uint32_t> eligible;
	double total = 0.0;
	for (size_t k = 0; k < lanes_.size(); ++k) {
		if (lanes_[k].next >= 0) {
			eligible.push_back(static_cast<uint32_t>(k));
			total += lanes_[k].length;
		}
	}
	if (eligible.empty() || count == 0) {
		return 0;
	}

	// Cumulative allocation: lane k gets floor(count*C_k/total) - floor(count*C_{k-1}/total).
	double cum = 0.0;
	uint64_t placed_before = 0;
	for (uint32_t k : eligible) {
		const LaneRt &rt = lanes_[k];
		cum += rt.length;
		uint64_t upto = static_cast<uint64_t>(std::floor(static_cast<double>(count) * (cum / total)));
		if (k == eligible.back()) {
			upto = count;
		}
		uint64_t n = upto > placed_before ? upto - placed_before : 0;
		placed_before = upto;
		const uint64_t capacity = static_cast<uint64_t>(std::floor(rt.length / kMinSpacing));
		n = std::min(n, capacity);
		if (n == 0) {
			continue;
		}
		const double spacing = rt.length / static_cast<double>(n);
		const Segment *seg = map_.segment(rt.lane->segment);
		const double limit = seg ? seg->speed_limit : 13.8889;
		for (uint64_t j = 0; j < n; ++j) {
			DriverParams p;
			p.v0 = limit * rng_.range(0.9, 1.1);
			p.T = rng_.range(1.2, 1.8);
			p.a = rng_.range(0.5, 0.9); // gentle drivers: waves form at mid density
			p.b = rng_.range(1.5, 2.0);
			p.s0 = kJam;
			p.length = kLength;
			p.two_sqrt_ab = 2.0 * std::sqrt(p.a * p.b);

			const double s = rt.length - spacing * (static_cast<double>(j) + 0.5);
			const double v = std::min(p.v0, std::max(0.0, (spacing - p.length - p.s0) / p.T));
			id_.push_back(next_vehicle_id_++);
			lane_.push_back(k);
			prev_lane_.push_back(k);
			s_.push_back(s);
			prev_s_.push_back(s);
			v_.push_back(v);
			acc_.push_back(0.0);
			params_.push_back(p);
			lanes_[k].cars.push_back(static_cast<uint32_t>(id_.size() - 1));
		}
	}
	for (LaneRt &rt : lanes_) {
		sort_lane(rt.cars);
	}
	return static_cast<uint32_t>(id_.size());
}

void Simulation::sort_lane(std::vector<uint32_t> &cars) const {
	// Insertion sort: lists are nearly sorted from the previous tick. The
	// comparator is a strict total order, so the result is platform-independent.
	auto before = [this](uint32_t a, uint32_t b) { return s_[a] > s_[b] || (s_[a] == s_[b] && id_[a] < id_[b]); };
	for (size_t i = 1; i < cars.size(); ++i) {
		const uint32_t x = cars[i];
		size_t j = i;
		while (j > 0 && before(x, cars[j - 1])) {
			cars[j] = cars[j - 1];
			--j;
		}
		cars[j] = x;
	}
}

void Simulation::find_leader_ahead(uint32_t lane, double s, bool &has, double &gap, double &speed) const {
	has = false;
	double dist = lanes_[lane].length - s; // distance from this vehicle's front to the lane end
	uint32_t cur = lane;
	for (int hops = 0; hops < 64 && dist < config_.lookahead; ++hops) {
		const int32_t nx = lanes_[cur].next;
		if (nx < 0) {
			// Dead end: treat the end of the lane as a stopped obstacle.
			has = true;
			gap = dist;
			speed = 0.0;
			return;
		}
		cur = static_cast<uint32_t>(nx);
		const std::vector<uint32_t> &cars = lanes_[cur].cars;
		if (!cars.empty()) {
			const uint32_t j = cars.back(); // rearmost on that lane
			has = true;
			gap = dist + s_[j] - params_[j].length;
			speed = v_[j];
			return;
		}
		dist += lanes_[cur].length;
	}
}

double Simulation::idm_accel(size_t i, bool has_leader, double gap, double leader_speed) const {
	const DriverParams &p = params_[i];
	const double v = v_[i];
	const double r = v / p.v0;
	const double r2 = r * r;
	const double free_term = 1.0 - r2 * r2; // (v/v0)^4 without pow()
	if (!has_leader) {
		return p.a * free_term;
	}
	const double dv = v - leader_speed;
	const double dyn = v * p.T + v * dv / p.two_sqrt_ab;
	const double s_star = p.s0 + (dyn > 0.0 ? dyn : 0.0);
	const double g = gap > 0.01 ? gap : 0.01;
	const double q = s_star / g;
	return p.a * (free_term - q * q);
}

void Simulation::tick() {
	const size_t n = id_.size();
	std::copy(lane_.begin(), lane_.end(), prev_lane_.begin());
	std::copy(s_.begin(), s_.end(), prev_s_.begin());

	// 1. Accelerations from the start-of-tick state (synchronous update).
	for (size_t L = 0; L < lanes_.size(); ++L) {
		const std::vector<uint32_t> &cars = lanes_[L].cars;
		for (size_t k = 0; k < cars.size(); ++k) {
			const uint32_t i = cars[k];
			bool has = false;
			double gap = 0.0, vl = 0.0;
			if (k > 0) {
				const uint32_t j = cars[k - 1];
				has = true;
				gap = s_[j] - params_[j].length - s_[i];
				vl = v_[j];
			} else {
				find_leader_ahead(static_cast<uint32_t>(L), s_[i], has, gap, vl);
			}
			acc_[i] = idm_accel(i, has, gap, vl);
		}
	}

	// 2. Noise (in vehicle order, so RNG use is deterministic) and integration.
	const double dt = config_.dt;
	for (size_t i = 0; i < n; ++i) {
		double a = acc_[i] + config_.accel_noise * rng_.symmetric();
		if (a < -config_.max_decel) {
			a = -config_.max_decel;
		}
		const double v = v_[i];
		double vn = v + a * dt;
		double ds;
		if (vn < 0.0) {
			// Ballistic stop within the step.
			ds = a < 0.0 ? -0.5 * v * v / a : 0.0;
			vn = 0.0;
		} else {
			ds = v * dt + 0.5 * a * dt * dt;
		}
		v_[i] = vn;
		double s = s_[i] + ds;
		uint32_t lane = lane_[i];
		while (s >= lanes_[lane].length) {
			const int32_t nx = lanes_[lane].next;
			if (nx < 0) {
				s = lanes_[lane].length;
				v_[i] = 0.0;
				break;
			}
			s -= lanes_[lane].length;
			lane = static_cast<uint32_t>(nx);
		}
		s_[i] = s;
		lane_[i] = lane;
	}

	// 3. Move vehicles that changed lane into their new lane's list, then sort.
	std::vector<uint32_t> movers;
	for (size_t L = 0; L < lanes_.size(); ++L) {
		std::vector<uint32_t> &cars = lanes_[L].cars;
		size_t w = 0;
		for (uint32_t i : cars) {
			if (lane_[i] == L) {
				cars[w++] = i;
			} else {
				movers.push_back(i);
			}
		}
		cars.resize(w);
	}
	for (uint32_t i : movers) {
		lanes_[lane_[i]].cars.push_back(i);
	}
	for (LaneRt &rt : lanes_) {
		sort_lane(rt.cars);
	}

	++tick_;
}

uint64_t Simulation::state_hash() const {
	StateHasher h;
	h.add_u64(tick_);
	h.add_u64(static_cast<uint64_t>(id_.size()));
	for (int k = 0; k < 4; ++k) {
		h.add_u64(rng_.state()[k]);
	}
	for (size_t i = 0; i < id_.size(); ++i) {
		h.add_u32(id_[i]);
		h.add_u32(lanes_[lane_[i]].id);
		h.add_double(s_[i]);
		h.add_double(v_[i]);
	}
	return h.value();
}

Pose Simulation::vehicle_pose(size_t i) const { return map_.lane_pose(*lanes_[lane_[i]].lane, s_[i]); }

Pose Simulation::vehicle_prev_pose(size_t i) const {
	return map_.lane_pose(*lanes_[prev_lane_[i]].lane, prev_s_[i]);
}

SimStats Simulation::stats() const {
	SimStats st;
	st.vehicles = static_cast<uint32_t>(id_.size());
	if (id_.empty()) {
		return st;
	}
	double sum = 0.0, sum2 = 0.0, sum_v0 = 0.0;
	for (size_t i = 0; i < id_.size(); ++i) {
		sum += v_[i];
		sum2 += v_[i] * v_[i];
		sum_v0 += params_[i].v0;
		if (v_[i] < 1.0) {
			++st.stopped;
		}
	}
	const double n = static_cast<double>(id_.size());
	st.mean_speed = sum / n;
	st.speed_stddev = std::sqrt(std::max(0.0, sum2 / n - st.mean_speed * st.mean_speed));
	st.mean_desired_speed = sum_v0 / n;
	return st;
}

void build_golden_map(Map &map) {
	map.clear();
	build_ring(map, GoldenScenario::kRadius, GoldenScenario::kLanes, GoldenScenario::kLaneWidth,
			GoldenScenario::kSpeedLimit);
}

uint64_t run_scenario(const Map &map, uint32_t cars, uint64_t seed, uint64_t ticks) {
	Simulation sim(map);
	sim.spawn_even(cars, seed);
	for (uint64_t t = 0; t < ticks; ++t) {
		sim.tick();
	}
	return sim.state_hash();
}

} // namespace tsim
