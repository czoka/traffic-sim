// RoadEditor: the M2 simulation bridge (clock, car rendering, car inspector).
#include "road_editor.h"

#include "tsim/hash.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace godot {

using namespace tsim;

namespace {

using Clock = std::chrono::steady_clock;

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }
String gs(const std::string &s) { return String::utf8(s.c_str(), static_cast<int64_t>(s.size())); }

double elapsed_us(Clock::time_point since) {
	return std::chrono::duration<double, std::micro>(Clock::now() - since).count();
}

double ema(double prev, double sample, double k) { return prev == 0.0 ? sample : prev + (sample - prev) * k; }

// Red (stopped) -> amber -> green (at desired speed).
void speed_color(double ratio, float out[4]) {
	const double t = std::clamp(ratio, 0.0, 1.0);
	const float r0 = 0.86f, g0 = 0.22f, b0 = 0.20f;
	const float r1 = 0.96f, g1 = 0.72f, b1 = 0.18f;
	const float r2 = 0.30f, g2 = 0.78f, b2 = 0.42f;
	if (t < 0.5) {
		const float k = static_cast<float>(t * 2.0);
		out[0] = r0 + (r1 - r0) * k;
		out[1] = g0 + (g1 - g0) * k;
		out[2] = b0 + (b1 - b0) * k;
	} else {
		const float k = static_cast<float>((t - 0.5) * 2.0);
		out[0] = r1 + (r2 - r1) * k;
		out[1] = g1 + (g2 - g1) * k;
		out[2] = b1 + (b2 - b1) * k;
	}
	out[3] = 1.0f;
}

constexpr int kFloatsPerCar = 12;

} // namespace

void RoadEditor::sync_sim() {
	ensure_geometry();
	if (sim_.sync(doc_.map(), geom_, doc_.revision())) {
		last_recompiled_junctions_ = sim_.compile_stats().junctions_compiled;
		last_recompile_ms_ = sim_.compile_stats().ms;
	}
}

void RoadEditor::sim_reset(int64_t seed) {
	sync_sim();
	sim_.traffic().reset(static_cast<uint64_t>(seed));
	accumulator_ = 0.0;
	window_real_ = 0.0;
	window_ticks_ = 0;
	effective_speed_ = 0.0;
	behind_ = false;
}

int64_t RoadEditor::sim_advance(double real_delta, double speed, double budget_ms) {
	sync_sim();
	Traffic &t = sim_.traffic();
	const double dt = t.config().dt;
	// Cap the frame delta so a stalled tab does not queue minutes of sim time.
	const double frame = std::clamp(real_delta, 0.0, 0.25);
	accumulator_ += frame * std::max(0.0, speed);
	const Clock::time_point start = Clock::now();
	int64_t ran = 0;
	while (accumulator_ >= dt) {
		t.tick();
		accumulator_ -= dt;
		++ran;
		if (elapsed_us(start) >= budget_ms * 1000.0) break;
	}
	const double spent = elapsed_us(start);
	behind_ = accumulator_ >= dt;
	if (behind_) accumulator_ = dt * 0.999; // drop the backlog; keep interpolation smooth
	frame_sim_ms_ema_ = ema(frame_sim_ms_ema_, spent / 1000.0, 0.05);
	if (ran > 0) tick_us_ema_ = ema(tick_us_ema_, spent / static_cast<double>(ran), 0.05);
	window_real_ += frame;
	window_ticks_ += ran;
	if (window_real_ >= 1.0) {
		effective_speed_ = static_cast<double>(window_ticks_) * dt / window_real_;
		window_real_ = 0.0;
		window_ticks_ = 0;
	}
	return ran;
}

int64_t RoadEditor::sim_step(int64_t ticks) {
	sync_sim();
	for (int64_t i = 0; i < ticks; ++i) sim_.traffic().tick();
	accumulator_ = 0.0;
	return ticks;
}

void RoadEditor::sim_set_demand(double multiplier) { sim_.traffic().config().demand = std::clamp(multiplier, 0.0, 20.0); }

void RoadEditor::sim_set_max_vehicles(int64_t cap) {
	sim_.traffic().config().max_vehicles = static_cast<uint32_t>(std::clamp<int64_t>(cap, 0, 100000));
}

int64_t RoadEditor::sim_car_count(int level) const {
	const Traffic &t = sim_.traffic();
	if (!t.network()) return 0;
	int64_t n = 0;
	for (size_t i = 0; i < t.vehicles().size(); ++i) n += t.level_of(i) == level;
	return n;
}

PackedFloat32Array RoadEditor::sim_car_buffer(int level, double car_scale) {
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	const int64_t n = sim_car_count(level);
	if (car_buffer_.size() != n * kFloatsPerCar) car_buffer_.resize(n * kFloatsPerCar);
	if (!net || n == 0) return car_buffer_;
	const double alpha = std::clamp(accumulator_ / t.config().dt, 0.0, 1.0);
	const double k = std::max(1.0, car_scale);
	float *w = car_buffer_.ptrw();
	size_t out = 0;
	for (size_t i = 0; i < t.vehicles().size(); ++i) {
		if (t.level_of(i) != level) continue;
		const Vehicle &v = t.vehicles()[i];
		const Pose p = t.pose(i, alpha);
		float *o = w + out * kFloatsPerCar;
		// Transform2D layout: x.x, y.x, pad, origin.x, x.y, y.y, pad, origin.y.
		// The quad is car-sized; its x axis is the heading.
		const double len = v.drv.length / 4.5;
		o[0] = static_cast<float>(p.dir.x * k * len);
		o[1] = static_cast<float>(-p.dir.y * k);
		o[2] = 0.0f;
		o[3] = static_cast<float>(p.pos.x);
		o[4] = static_cast<float>(p.dir.y * k * len);
		o[5] = static_cast<float>(p.dir.x * k);
		o[6] = 0.0f;
		o[7] = static_cast<float>(p.pos.y);
		const NetLane &l = net->lanes[static_cast<size_t>(v.lane)];
		speed_color(v.v / std::max(1.0, l.speed_limit * v.drv.speed_factor), o + 8);
		++out;
	}
	return car_buffer_;
}

int64_t RoadEditor::sim_pick_car(Vector2 pos, double radius, int level) {
	const Traffic &t = sim_.traffic();
	if (!t.network()) return 0;
	const double alpha = std::clamp(accumulator_ / t.config().dt, 0.0, 1.0);
	const Vec2 q{ pos.x, pos.y };
	double best = radius;
	int64_t id = 0;
	for (size_t i = 0; i < t.vehicles().size(); ++i) {
		if (t.level_of(i) != level) continue;
		const Pose p = t.pose(i, alpha);
		// Distance to the car's centre line (front bumper back to its tail).
		const Vec2 tail = p.pos - p.dir * t.vehicles()[i].drv.length;
		const Vec2 mid = (p.pos + tail) * 0.5;
		const double d = (mid - q).length() - 1.0;
		if (d <= best) {
			best = d;
			id = t.vehicles()[i].id;
		}
	}
	return id;
}

Dictionary RoadEditor::sim_car_info(int64_t id) {
	Dictionary d;
	const Traffic &t = sim_.traffic();
	const Network *net = t.network();
	if (!net) return d;
	const VehicleInfo info = t.info(static_cast<VehicleId>(id));
	if (!info.found) return d;
	const int32_t i = t.find_vehicle(static_cast<VehicleId>(id));
	const double alpha = std::clamp(accumulator_ / t.config().dt, 0.0, 1.0);
	const Pose p = t.pose(static_cast<size_t>(i), alpha);
	d["id"] = id;
	d["pos"] = gv(p.pos);
	d["dir"] = gv(p.dir);
	d["length"] = t.vehicles()[static_cast<size_t>(i)].drv.length;
	d["speed_kmh"] = info.speed * 3.6;
	d["desired_kmh"] = info.desired_speed * 3.6;
	d["accel"] = info.accel;
	d["state"] = vehicle_state_name(info.state);
	d["blocker"] = static_cast<int64_t>(info.blocker);
	d["origin"] = static_cast<int64_t>(info.origin);
	d["dest"] = static_cast<int64_t>(info.dest);
	d["trip_time"] = info.trip_time;
	d["distance"] = info.distance;
	d["stopped_for"] = info.stopped_for;
	d["critical_gap"] = info.critical_gap;
	d["level"] = info.level;
	d["segment"] = static_cast<int64_t>(info.segment);
	const RoadSegment *seg = doc_.map().segment(info.segment);
	d["road"] = seg ? gs(seg->name) : String();
	d["in_junction"] = info.lane.kind == NetLaneKind::Connector;
	d["next_junction"] = static_cast<int64_t>(info.junction);
	d["turns_left"] = static_cast<int64_t>(info.connectors_left);
	PackedVector2Array route;
	route.resize(static_cast<int64_t>(info.route.size()));
	Vector2 *w = route.ptrw();
	for (size_t k = 0; k < info.route.size(); ++k) w[k] = gv(info.route[k]);
	d["route"] = route;
	return d;
}

Dictionary RoadEditor::sim_stats() {
	const Traffic &t = sim_.traffic();
	const TrafficStats st = t.stats();
	Dictionary d;
	d["tick"] = static_cast<int64_t>(t.tick_count());
	d["sim_time"] = t.sim_time();
	d["seed"] = static_cast<int64_t>(t.seed());
	d["demand"] = t.config().demand;
	d["vehicles"] = static_cast<int64_t>(st.vehicles);
	d["spawned"] = static_cast<int64_t>(st.spawned);
	d["arrived"] = static_cast<int64_t>(st.arrived);
	d["removed_stuck"] = static_cast<int64_t>(st.removed_stuck);
	d["unroutable"] = static_cast<int64_t>(st.unroutable);
	d["waiting_to_enter"] = static_cast<int64_t>(st.waiting_to_enter);
	d["mean_speed_kmh"] = st.mean_speed * 3.6;
	d["stopped"] = static_cast<int64_t>(st.stopped);
	d["max_stopped"] = st.max_stopped;
	d["mean_trip_time"] = st.mean_trip_time;
	d["max_junction_wait"] = st.max_junction_wait;
	d["lane_changes"] = static_cast<int64_t>(st.lane_changes);
	d["reroutes"] = static_cast<int64_t>(st.reroutes);
	d["forced_grants"] = static_cast<int64_t>(st.forced_grants);
	d["tick_us"] = tick_us_ema_;
	d["frame_sim_ms"] = frame_sim_ms_ema_;
	d["behind"] = behind_;
	d["effective_speed"] = effective_speed_;
	d["recompiled_junctions"] = last_recompiled_junctions_;
	d["recompile_ms"] = last_recompile_ms_;
	d["compiled_revision"] = static_cast<int64_t>(sim_.compiled_revision());
	const Network &n = sim_.network();
	d["lanes"] = static_cast<int64_t>(n.lanes.size());
	d["junctions"] = static_cast<int64_t>(n.junctions.size());
	d["spawners"] = static_cast<int64_t>(n.spawners.size());
	return d;
}

String RoadEditor::sim_state_hash() const { return gs(hash_to_hex(sim_.traffic().state_hash())); }

Dictionary RoadEditor::sim_golden_check() const {
	const Clock::time_point start = Clock::now();
	TrafficStats st;
	const std::string hash = hash_to_hex(run_traffic_golden(TrafficGolden::kTicks, &st));
	Dictionary d;
	d["hash"] = gs(hash);
	d["expected"] = String(TrafficGolden::kExpectedHash);
	d["pass"] = hash == TrafficGolden::kExpectedHash;
	d["cars"] = static_cast<int64_t>(st.vehicles);
	d["ticks"] = static_cast<int64_t>(TrafficGolden::kTicks);
	d["ms"] = elapsed_us(start) / 1000.0;
	return d;
}

} // namespace godot
