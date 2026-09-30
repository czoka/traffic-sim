// RoadEditor: M3 editing (roundabouts, signal plans, bus stops, depots) and
// what the editor draws for them (stops, routes, bays, signal heads).
#include "m3_dicts.h"
#include "road_editor.h"

#include <godot_cpp/core/class_db.hpp>

#include <algorithm>
#include <map>

namespace godot {

using namespace tsim;
using namespace m3;

namespace {

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }

const char *light_name(SignalLight l) {
	switch (l) {
		case SignalLight::Red:
			return "red";
		case SignalLight::Amber:
			return "amber";
		case SignalLight::Green:
			return "green";
		case SignalLight::GreenYield:
			return "yield";
	}
	return "red";
}

} // namespace

void RoadEditor::set_roundabout(int64_t node, const Dictionary &r) {
	doc_.set_roundabout(static_cast<NodeId>(node), roundabout_from(r));
}

Dictionary RoadEditor::default_signal_plan(int64_t node) const {
	return signal_dict(tsim::default_signal_plan(doc_.map(), static_cast<NodeId>(node)));
}

void RoadEditor::set_signal_plan(int64_t node, const Dictionary &plan) {
	doc_.set_signal_plan(static_cast<NodeId>(node), signal_from(plan));
}

int64_t RoadEditor::add_stop(int64_t seg, double u, const String &side, const String &kind, const String &name) {
	StopKind k = StopKind::Kerbside;
	stop_kind_from_name(str(kind), k);
	return doc_.add_stop(static_cast<SegmentId>(seg), u, side == "backward" ? LaneDir::Backward : LaneDir::Forward, k,
			str(name));
}

void RoadEditor::set_stop(int64_t seg, const Dictionary &stop) { doc_.set_stop(static_cast<SegmentId>(seg), stop_from(stop)); }

void RoadEditor::remove_stop(int64_t seg, int64_t stop) {
	doc_.remove_stop(static_cast<SegmentId>(seg), static_cast<uint32_t>(stop));
}

void RoadEditor::set_depot(int64_t node, const Dictionary &depot) {
	doc_.set_depot(static_cast<NodeId>(node), depot_from(depot));
}

Array RoadEditor::get_stops() {
	ensure_network();
	Array out;
	for (const auto &kv : doc_.map().segments()) {
		const RoadSegment &seg = kv.second;
		for (const BusStop &st : seg.stops) {
			Dictionary d = stop_dict(st);
			d["segment"] = static_cast<int64_t>(seg.id);
			d["level"] = seg.level;
			const int32_t i = check_net_.stop_index(st.id);
			if (i >= 0) {
				const NetStop &ns = check_net_.stops[static_cast<size_t>(i)];
				d["pos"] = gv(ns.pos);
				d["dir"] = gv(ns.dir);
				d["bays"] = ns.bays;
				d["served"] = true;
			} else {
				d["pos"] = segment_point(static_cast<int64_t>(seg.id), st.u);
				d["dir"] = segment_tangent(static_cast<int64_t>(seg.id), st.u);
				d["served"] = false; // no kerb lane for that direction
			}
			out.push_back(d);
		}
	}
	return out;
}

// The roads a bus takes on a route: depot -> stops in order -> depot, as the
// router would drive them at free flow.
PackedVector2Array RoadEditor::route_path(const NetDepot &d, const BusRoute &r) {
	const Network &n = check_net_;
	PackedVector2Array out;
	if (d.spawn_lanes.empty()) return out;
	Traffic router;
	router.set_network(&n);
	auto append = [&](int32_t lane, double from, double to) {
		const NetLane &l = n.lanes[static_cast<size_t>(lane)];
		out.push_back(gv(n.pose(lane, from).pos));
		for (size_t k = 0; k < l.pts.size(); ++k) {
			if (l.cum[k] > from && l.cum[k] < to) out.push_back(gv(l.pts[k]));
		}
		out.push_back(gv(n.pose(lane, std::min(to, l.length)).pos));
	};
	auto follow = [&](int32_t start, double s0, const std::vector<int32_t> &route, double end_s) {
		int32_t cur = start;
		double from = s0;
		for (int32_t c : route) {
			append(cur, from, 1e300);
			append(c, 0.0, 1e300);
			cur = n.lanes[static_cast<size_t>(c)].to;
			from = 0.0;
		}
		append(cur, from, end_s);
		return cur;
	};
	int32_t cur = d.spawn_lanes[0];
	double s = 0.0;
	std::vector<int32_t> seq;
	for (uint32_t id : r.stops) {
		const int32_t si = n.stop_index(id);
		if (si >= 0) seq.push_back(si);
	}
	if (r.loop && seq.size() > 1) seq.push_back(seq.front());
	std::vector<int32_t> route;
	for (int32_t si : seq) {
		const NetStop &st = n.stops[static_cast<size_t>(si)];
		if (!router.find_route_to_lane(cur, s, st.lane, st.s, route, VehicleKind::Bus, true)) return out;
		follow(cur, s, route, st.s);
		cur = st.lane;
		s = st.s;
	}
	if (router.find_route(cur, d.node, route, true, VehicleKind::Bus)) follow(cur, s, route, 1e300);
	return out;
}

Array RoadEditor::get_depots() {
	ensure_network();
	Array out;
	for (const auto &kv : doc_.map().nodes()) {
		const RoadNode &n = kv.second;
		if (!n.depot.enabled) continue;
		Dictionary d = depot_dict(n.depot);
		d["node"] = static_cast<int64_t>(n.id);
		d["pos"] = gv(n.pos);
		d["level"] = n.level;
		bool active = false;
		for (const NetDepot &nd : check_net_.depots) active |= nd.node == n.id;
		d["active"] = active; // on a road end
		const NetDepot *nd = nullptr;
		for (const NetDepot &x : check_net_.depots) {
			if (x.node == n.id) nd = &x;
		}
		Array routes = d["routes"];
		for (int64_t r = 0; r < routes.size(); ++r) {
			Dictionary rd = routes[r];
			const BusRoute &br = n.depot.routes[static_cast<size_t>(r)];
			rd["path"] = nd ? route_path(*nd, br) : PackedVector2Array();
			routes[r] = rd;
		}
		d["routes"] = routes;
		out.push_back(d);
	}
	return out;
}

Array RoadEditor::get_bays(int level) {
	ensure_network();
	Array out;
	for (const NetBay &b : check_net_.bays) {
		if (b.lane < 0 || check_net_.lanes[static_cast<size_t>(b.lane)].level != level) continue;
		Dictionary d;
		d["pos"] = gv(b.pos);
		d["dir"] = gv(b.dir);
		d["style"] = parking_style_name(b.style);
		out.push_back(d);
	}
	return out;
}

Array RoadEditor::sim_signal_heads(int level) {
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	if (!net) {
		ensure_network();
		net = &check_net_;
	}
	const int64_t tick = static_cast<int64_t>(t.network() ? t.tick_count() : 0);
	Array out;
	for (const NetJunction &j : net->junctions) {
		if (!j.signal.enabled || j.level != level) continue;
		for (int32_t a : j.approaches) {
			const NetLane &l = net->lanes[static_cast<size_t>(a)];
			// The most permissive light of the lane's movements.
			int best = -1;
			SignalLight shown = SignalLight::Red;
			for (int32_t c : l.next) {
				const NetLane &cn = net->lanes[static_cast<size_t>(c)];
				if (cn.movement < 0) continue;
				const SignalLight s = j.light(cn.movement, tick);
				const int rank = s == SignalLight::Green ? 3 : s == SignalLight::GreenYield ? 2 : s == SignalLight::Amber ? 1 : 0;
				if (rank > best) {
					best = rank;
					shown = s;
				}
			}
			if (best < 0) continue;
			const Pose p = net->pose(a, l.length);
			Dictionary d;
			d["node"] = static_cast<int64_t>(j.node);
			d["pos"] = gv(p.pos);
			d["dir"] = gv(p.dir);
			d["light"] = light_name(shown);
			out.push_back(d);
		}
	}
	return out;
}

Dictionary RoadEditor::sim_signal_state(int64_t node) {
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	if (!net) {
		ensure_network();
		net = &check_net_;
	}
	Dictionary d;
	const int32_t ji = net->junction_at(static_cast<NodeId>(node));
	if (ji < 0) return d;
	const NetJunction &j = net->junctions[static_cast<size_t>(ji)];
	if (!j.signal.enabled) return d;
	int64_t into = 0;
	const int64_t tick = static_cast<int64_t>(t.network() ? t.tick_count() : 0);
	d["phase"] = j.phase_at(tick, &into);
	d["into"] = static_cast<double>(into) / 10.0;
	d["cycle"] = static_cast<double>(j.signal.cycle) / 10.0;
	Array phases;
	for (int64_t g : j.signal.green) {
		Dictionary pd;
		pd["green"] = static_cast<double>(g) / 10.0;
		pd["amber"] = static_cast<double>(j.signal.amber) / 10.0;
		pd["all_red"] = static_cast<double>(j.signal.all_red) / 10.0;
		phases.push_back(pd);
	}
	d["phases"] = phases;
	return d;
}

Array RoadEditor::sim_route_stats() {
	Array out;
	for (const RouteStats &r : sim_.traffic().route_stats()) {
		Dictionary d;
		d["id"] = static_cast<int64_t>(r.id);
		d["active"] = static_cast<int64_t>(r.active);
		d["runs"] = static_cast<int64_t>(r.runs);
		d["round_trip"] = r.round_trip;
		d["fleet"] = static_cast<int64_t>(r.fleet);
		out.push_back(d);
	}
	return out;
}

} // namespace godot
