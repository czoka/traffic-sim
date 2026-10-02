// RoadEditor: M4 editing (paths, crossings, fences, ramps, bridges and tunnels)
// and the people in the sim (pedestrians, walk lights, stop stats).
#include "m3_dicts.h"
#include "road_editor.h"

#include <godot_cpp/core/class_db.hpp>

#include <algorithm>
#include <cmath>

namespace godot {

using namespace tsim;
using namespace m3;

namespace {

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }
Vec2 tv(Vector2 v) { return Vec2{ v.x, v.y }; }

constexpr int kFloatsPerPed = 12;

void rgb(uint32_t c, float out[4]) {
	out[0] = static_cast<float>((c >> 16) & 0xff) / 255.0f;
	out[1] = static_cast<float>((c >> 8) & 0xff) / 255.0f;
	out[2] = static_cast<float>(c & 0xff) / 255.0f;
	out[3] = 1.0f;
}

// Walking light blue, waiting to cross orange, crossing white, waiting at a stop violet.
uint32_t ped_color(PedState s) {
	switch (s) {
		case PedState::Walking:
			return 0x9fd4ff;
		case PedState::WaitingToCross:
			return 0xffa64d;
		case PedState::Crossing:
			return 0xffffff;
		case PedState::WaitingForBus:
			return 0xc58cff;
		case PedState::Riding:
			return 0xc58cff;
	}
	return 0x9fd4ff;
}

const char *walk_name(WalkLight l) {
	switch (l) {
		case WalkLight::Walk:
			return "walk";
		case WalkLight::Flashing:
			return "flashing";
		case WalkLight::DontWalk:
			return "dont_walk";
	}
	return "dont_walk";
}

const char *light_name(SignalLight l) {
	switch (l) {
		case SignalLight::Red:
			return "red";
		case SignalLight::Amber:
			return "amber";
		case SignalLight::Green:
		case SignalLight::GreenYield:
			return "green";
	}
	return "red";
}

} // namespace

// --- Editing ---------------------------------------------------------------------------

int64_t RoadEditor::add_crossing(int64_t seg, double u, const String &kind, bool bike, bool refuge) {
	CrossingKind k = CrossingKind::Zebra;
	crossing_kind_from_name(str(kind), k);
	return doc_.add_crossing(static_cast<SegmentId>(seg), u, k, bike, refuge);
}

void RoadEditor::set_crossing(int64_t seg, const Dictionary &c) {
	const RoadSegment *s = doc_.map().segment(static_cast<SegmentId>(seg));
	if (!s) return;
	const uint32_t id = static_cast<uint32_t>(static_cast<int64_t>(c.get("id", 0)));
	for (const Crossing &x : s->crossings) {
		if (x.id == id) doc_.set_crossing(s->id, crossing_rules_from(c, x));
	}
}

void RoadEditor::remove_crossing(int64_t seg, int64_t id) {
	doc_.remove_crossing(static_cast<SegmentId>(seg), static_cast<uint32_t>(id));
}

void RoadEditor::set_fence(int64_t seg, int side, double u0, double u1, bool on) {
	doc_.set_fence(static_cast<SegmentId>(seg), side, u0, u1, on);
}

void RoadEditor::set_ramp(int64_t seg, int rise, bool stairs) { doc_.set_ramp(static_cast<SegmentId>(seg), rise, stairs); }

Dictionary RoadEditor::plan_lift(int64_t seg, Vector2 at, int delta) {
	ensure_geometry();
	const Document::LiftPlan p = doc_.plan_lift(static_cast<SegmentId>(seg), tv(at), delta);
	Dictionary d;
	d["ok"] = p.ok;
	d["error"] = gstr(p.error);
	d["length"] = p.length;
	PackedFloat32Array s;
	PackedVector2Array pts;
	if (p.ok && p.length > 0.0) {
		for (double x : p.s) {
			s.push_back(static_cast<float>(x));
			pts.push_back(segment_point(seg, x / p.length));
		}
	}
	d["stations"] = s;
	d["points"] = pts;
	return d;
}

String RoadEditor::lift(int64_t seg, Vector2 at, int delta) {
	return gstr(doc_.lift(static_cast<SegmentId>(seg), tv(at), delta));
}

Array RoadEditor::get_crossings(int level) {
	ensure_geometry();
	Array out;
	for (const CrossingGeom &c : geom_.crossings()) {
		if (c.level != level) continue;
		Dictionary d;
		d["segment"] = static_cast<int64_t>(c.seg);
		d["end"] = c.end;
		d["id"] = static_cast<int64_t>(c.id);
		d["node"] = static_cast<int64_t>(c.node);
		d["kind"] = crossing_kind_name(c.kind);
		d["bike"] = c.bike;
		d["a"] = gv(c.a);
		d["b"] = gv(c.b);
		d["mid"] = gv((c.a + c.b) * 0.5);
		d["refuge"] = c.refuge_t1 > c.refuge_t0 && c.refuge_t0 >= 0.0;
		out.push_back(d);
	}
	return out;
}

// --- People in the sim ------------------------------------------------------------------

void RoadEditor::sim_set_people(const Dictionary &cfg) {
	TrafficConfig &c = sim_.traffic().config();
	c.max_pedestrians = static_cast<uint32_t>(std::clamp<int64_t>(cfg.get("max_pedestrians", static_cast<int64_t>(c.max_pedestrians)), 0, 100000));
	c.car_owners = std::clamp(static_cast<double>(cfg.get("car_owners", c.car_owners)), 0.0, 1.0);
	c.bike_owners = std::clamp(static_cast<double>(cfg.get("bike_owners", c.bike_owners)), 0.0, 1.0);
	c.bus_capacity = std::clamp(static_cast<int>(cfg.get("bus_capacity", c.bus_capacity)), 1, 300);
}

int64_t RoadEditor::sim_ped_count(int level) const {
	const Traffic &t = sim_.traffic();
	if (!t.network()) return 0;
	int64_t n = 0;
	for (size_t i = 0; i < t.pedestrians().size(); ++i) {
		const Pedestrian &p = t.pedestrians()[i];
		n += p.state != PedState::Riding && t.ped_level(i) == level;
	}
	return n;
}

PackedFloat32Array RoadEditor::sim_ped_buffer(int level, double scale) {
	const Traffic &t = sim_.traffic();
	const int64_t n = sim_ped_count(level);
	if (ped_buffer_.size() != n * kFloatsPerPed) ped_buffer_.resize(n * kFloatsPerPed);
	if (!t.network() || n == 0) return ped_buffer_;
	const double alpha = std::clamp(accumulator_ / t.config().dt, 0.0, 1.0);
	const double k = std::max(1.0, scale);
	float *w = ped_buffer_.ptrw();
	size_t out = 0;
	for (size_t i = 0; i < t.pedestrians().size(); ++i) {
		const Pedestrian &p = t.pedestrians()[i];
		if (p.state == PedState::Riding || t.ped_level(i) != level) continue;
		const Pose pose = t.ped_pose(i, alpha);
		float *o = w + out * kFloatsPerPed;
		o[0] = static_cast<float>(pose.dir.x * k);
		o[1] = static_cast<float>(-pose.dir.y * k);
		o[2] = 0.0f;
		o[3] = static_cast<float>(pose.pos.x);
		o[4] = static_cast<float>(pose.dir.y * k);
		o[5] = static_cast<float>(pose.dir.x * k);
		o[6] = 0.0f;
		o[7] = static_cast<float>(pose.pos.y);
		rgb(ped_color(p.state), o + 8);
		++out;
	}
	return ped_buffer_;
}

int64_t RoadEditor::sim_pick_ped(Vector2 pos, double radius, int level) {
	const Traffic &t = sim_.traffic();
	if (!t.network()) return 0;
	const double alpha = std::clamp(accumulator_ / t.config().dt, 0.0, 1.0);
	double best = radius;
	int64_t id = 0;
	for (size_t i = 0; i < t.pedestrians().size(); ++i) {
		const Pedestrian &p = t.pedestrians()[i];
		if (p.state == PedState::Riding || t.ped_level(i) != level) continue;
		const double d = (t.ped_pose(i, alpha).pos - tv(pos)).length();
		if (d <= best) {
			best = d;
			id = p.id;
		}
	}
	return id;
}

Dictionary RoadEditor::sim_ped_info(int64_t id) {
	Dictionary d;
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	if (!net) return d;
	const PedInfo info = t.ped_info(static_cast<uint32_t>(id));
	if (!info.found) return d;
	const int32_t i = t.find_pedestrian(static_cast<uint32_t>(id));
	const double alpha = std::clamp(accumulator_ / t.config().dt, 0.0, 1.0);
	d["id"] = id;
	d["pos"] = gv(t.ped_pose(static_cast<size_t>(i), alpha).pos);
	d["level"] = t.ped_level(static_cast<size_t>(i));
	d["state"] = ped_state_name(info.state);
	d["speed_kmh"] = info.speed * 3.6;
	d["waited"] = info.waited;
	d["trip_time"] = info.trip_time;
	d["origin"] = static_cast<int64_t>(info.origin);
	d["dest"] = static_cast<int64_t>(info.dest);
	auto stop_name = [&](int32_t s) {
		return s >= 0 && static_cast<size_t>(s) < net->stops.size() ? gstr(net->stops[static_cast<size_t>(s)].name) : String();
	};
	d["board"] = stop_name(info.board);
	d["alight"] = stop_name(info.alight);
	String route_name;
	for (const NetDepot &dp : net->depots) {
		for (const NetRoute &r : dp.routes) {
			if (r.id == info.route && info.route != 0) route_name = gstr(r.name);
		}
	}
	d["route_name"] = route_name;
	d["vehicle"] = static_cast<int64_t>(info.vehicle);
	d["resident"] = static_cast<int64_t>(info.resident);
	PackedVector2Array line;
	for (const Vec2 &p : info.route_line) line.push_back(gv(p));
	d["route"] = line;
	return d;
}

Array RoadEditor::sim_walk_lights(int level) {
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	if (!net) {
		ensure_network();
		net = &check_net_;
	}
	const bool running = t.network() != nullptr;
	Array out;
	const PedGraph &g = net->ped;
	for (size_t c = 0; c < g.crossings.size(); ++c) {
		const NetCrossing &nc = g.crossings[c];
		if (nc.kind != CrossingKind::Signal || nc.level != level) continue;
		Dictionary d;
		d["a"] = gv(nc.a);
		d["b"] = gv(nc.b);
		d["walk"] = running ? walk_name(t.crossing_walk_light(static_cast<int32_t>(c))) : "dont_walk";
		d["push_button"] = nc.push_button;
		d["car"] = running && nc.push_button ? light_name(t.crossing_car_light(static_cast<int32_t>(c))) : "green";
		out.push_back(d);
	}
	return out;
}

Array RoadEditor::sim_stop_stats() {
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	Array out;
	if (!net) return out;
	for (const StopStats &s : t.stop_stats()) {
		const NetStop &ns = net->stops[static_cast<size_t>(s.stop)];
		Dictionary d;
		d["id"] = static_cast<int64_t>(ns.id);
		d["name"] = gstr(ns.name);
		d["pos"] = gv(ns.pos);
		d["waiting"] = static_cast<int64_t>(s.waiting);
		d["boarded"] = static_cast<int64_t>(s.boarded);
		d["alighted"] = static_cast<int64_t>(s.alighted);
		d["left_behind"] = static_cast<int64_t>(s.left_behind);
		d["mean_wait"] = s.mean_wait;
		d["kind"] = stop_kind_name(ns.kind);
		d["boarded_per_hour"] = s.boarded_per_hour;
		d["left_behind_per_hour"] = s.left_behind_per_hour;
		out.push_back(d);
	}
	return out;
}

Array RoadEditor::sim_route_loads() {
	const Traffic &t = sim_.traffic();
	Array out;
	for (const RouteLoad &r : t.route_loads()) {
		Dictionary d;
		d["route"] = static_cast<int64_t>(r.route);
		PackedFloat32Array load;
		for (double l : r.load) load.push_back(static_cast<float>(l));
		d["load"] = load;
		out.push_back(d);
	}
	return out;
}

void RoadEditor::bind_m4_methods() {
	ClassDB::bind_method(D_METHOD("add_crossing", "segment", "u", "kind", "bike", "refuge"), &RoadEditor::add_crossing);
	ClassDB::bind_method(D_METHOD("set_crossing", "segment", "crossing"), &RoadEditor::set_crossing);
	ClassDB::bind_method(D_METHOD("remove_crossing", "segment", "id"), &RoadEditor::remove_crossing);
	ClassDB::bind_method(D_METHOD("set_fence", "segment", "side", "u0", "u1", "on"), &RoadEditor::set_fence);
	ClassDB::bind_method(D_METHOD("set_ramp", "segment", "rise", "stairs"), &RoadEditor::set_ramp);
	ClassDB::bind_method(D_METHOD("plan_lift", "segment", "at", "delta"), &RoadEditor::plan_lift);
	ClassDB::bind_method(D_METHOD("lift", "segment", "at", "delta"), &RoadEditor::lift);
	ClassDB::bind_method(D_METHOD("get_crossings", "level"), &RoadEditor::get_crossings);
	ClassDB::bind_method(D_METHOD("sim_set_people", "config"), &RoadEditor::sim_set_people);
	ClassDB::bind_method(D_METHOD("sim_ped_count", "level"), &RoadEditor::sim_ped_count);
	ClassDB::bind_method(D_METHOD("sim_ped_buffer", "level", "scale"), &RoadEditor::sim_ped_buffer);
	ClassDB::bind_method(D_METHOD("sim_pick_ped", "pos", "radius", "level"), &RoadEditor::sim_pick_ped);
	ClassDB::bind_method(D_METHOD("sim_ped_info", "id"), &RoadEditor::sim_ped_info);
	ClassDB::bind_method(D_METHOD("sim_walk_lights", "level"), &RoadEditor::sim_walk_lights);
	ClassDB::bind_method(D_METHOD("sim_stop_stats"), &RoadEditor::sim_stop_stats);
	ClassDB::bind_method(D_METHOD("sim_route_loads"), &RoadEditor::sim_route_loads);
}

} // namespace godot
