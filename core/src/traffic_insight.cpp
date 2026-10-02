// Traffic (M7): insight - per-lane heat for the speed, wait and throughput
// heatmaps, and per-junction numbers for the inspector.
//
// Counted in move(): each sim minute a road lane adds up the vehicle-seconds
// on it, their speed, the time they stood (below 1 m/s) and the vehicles that
// left it; heat_fold() turns that into smoothed values. Nothing here feeds
// back into the sim, and none of it is in the state hash.
#include "tsim/traffic.h"

#include <algorithm>
#include <map>

namespace tsim {

void Traffic::heat_fold() {
	if (!net_) return;
	const Network &n = *net_;
	const double minutes = 1.0; // per fold
	const double k = 0.3; // weight of the newest minute
	for (size_t l = 0; l < heat_acc_.size() && l < n.lanes.size(); ++l) {
		if (n.lanes[l].kind != NetLaneKind::Road) continue;
		HeatAcc &a = heat_acc_[l];
		LaneHeat &h = heat_[l];
		const bool busy = a.occ > 0.0 || a.passed > 0;
		const double flow = static_cast<double>(a.passed) * 60.0 / minutes;
		const double wait = a.stopped / std::max(1.0, static_cast<double>(a.passed));
		if (busy) {
			const double speed = a.occ > 0.0 ? a.speed / a.occ : h.speed;
			const double limit = std::max(1.0, n.lanes[l].speed_limit);
			if (a.idle >= 5) { // first traffic in a while: start from this minute
				h.speed = speed;
				h.wait = wait;
				h.flow = flow;
			} else {
				h.speed += k * (speed - h.speed);
				h.wait += k * (wait - h.wait);
				h.flow += k * (flow - h.flow);
			}
			h.speed_ratio = std::min(1.0, h.speed / limit);
			a.idle = 0;
		} else {
			h.flow += k * (0.0 - h.flow);
			h.wait += k * (0.0 - h.wait);
			if (a.idle < 99) ++a.idle;
		}
		h.seen = a.idle < 5;
		h.passed += a.passed;
		h.stopped += a.stopped;
		a.occ = a.speed = a.stopped = 0.0;
		a.passed = 0;
	}
}

JunctionStats Traffic::junction_stats(NodeId node) const {
	JunctionStats out;
	if (!net_) return out;
	const Network &n = *net_;
	const NetJunction *jn = nullptr;
	for (const NetJunction &j : n.junctions) {
		if (j.node == node) jn = &j;
	}
	if (!jn) return out;
	out.found = true;
	out.node = node;
	double wait_flow = 0.0;
	double stopped = 0.0;
	std::map<SegmentId, std::pair<double, double>> by_segment; // wait x flow, flow
	for (int32_t l : jn->approaches) {
		if (l < 0 || static_cast<size_t>(l) >= heat_.size()) continue;
		const LaneHeat &h = heat_[static_cast<size_t>(l)];
		++out.approaches;
		out.flow += h.flow;
		wait_flow += h.wait * h.flow;
		out.passed += h.passed;
		stopped += h.stopped;
		auto &seg = by_segment[n.lanes[static_cast<size_t>(l)].segment];
		seg.first += h.wait * std::max(h.flow, 1.0);
		seg.second += std::max(h.flow, 1.0);
		for (int32_t vi : cars_[static_cast<size_t>(l)]) {
			const Vehicle &v = veh_[static_cast<size_t>(vi)];
			if (v.v < 1.0 && v.phase == 0) {
				++out.queued;
				out.longest_wait = std::max(out.longest_wait, v.stopped_for);
			}
		}
	}
	out.mean_wait = out.flow > 0.0 ? wait_flow / out.flow : 0.0;
	out.total_mean_wait = out.passed ? stopped / static_cast<double>(out.passed) : 0.0;
	for (const auto &kv : by_segment) {
		const double w = kv.second.second > 0.0 ? kv.second.first / kv.second.second : 0.0;
		if (w > out.worst_wait) {
			out.worst_wait = w;
			out.worst_segment = kv.first;
		}
	}
	return out;
}

} // namespace tsim
