#include "traffic_sim.h"

#include "tsim/hash.h"
#include "tsim/map_json.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace godot {

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_us(Clock::time_point since) {
	return std::chrono::duration<double, std::micro>(Clock::now() - since).count();
}

double ema(double prev, double sample, double k) { return prev == 0.0 ? sample : prev + (sample - prev) * k; }

// Red (stopped) -> amber -> green (at desired speed).
void speed_color(double ratio, float out[4]) {
	const double t = std::clamp(ratio, 0.0, 1.0);
	const float r0 = 0.86f, g0 = 0.22f, b0 = 0.20f; // red
	const float r1 = 0.96f, g1 = 0.72f, b1 = 0.18f; // amber
	const float r2 = 0.30f, g2 = 0.78f, b2 = 0.42f; // green
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

PackedVector2Array to_packed(const std::vector<tsim::Vec2> &pts) {
	PackedVector2Array out;
	out.resize(static_cast<int64_t>(pts.size()));
	for (size_t i = 0; i < pts.size(); ++i) {
		out.set(static_cast<int64_t>(i), Vector2(static_cast<real_t>(pts[i].x), static_cast<real_t>(pts[i].y)));
	}
	return out;
}

} // namespace

TrafficSim::TrafficSim() { sim_ = std::make_unique<tsim::Simulation>(map_); }

TrafficSim::~TrafficSim() = default;

void TrafficSim::reset_clock() {
	accumulator_ = 0.0;
	last_frame_ticks_ = 0;
	window_real_ = 0.0;
	window_ticks_ = 0;
	effective_speed_ = 0.0;
	behind_ = false;
}

void TrafficSim::new_ring(double radius, int lanes, double lane_width, double speed_kmh) {
	sim_->clear_vehicles();
	map_.clear();
	tsim::build_ring(map_, radius, lanes, lane_width, speed_kmh / 3.6);
	sim_->on_map_changed();
	reset_clock();
}

int64_t TrafficSim::add_straight_road(Vector2 a, Vector2 b, int lanes, double speed_kmh) {
	const tsim::NodeId na = map_.add_node({ a.x, a.y });
	const tsim::NodeId nb = map_.add_node({ b.x, b.y });
	const tsim::SegmentId s = map_.add_straight(na, nb, lanes, 3.5, speed_kmh / 3.6);
	sim_->on_map_changed();
	return s;
}

String TrafficSim::save_json() const {
	const std::string text = tsim::map_to_json(map_);
	return String::utf8(text.c_str(), static_cast<int64_t>(text.size()));
}

String TrafficSim::load_json(const String &text) {
	const CharString utf8 = text.utf8();
	tsim::Map loaded;
	std::string err;
	if (!tsim::map_from_json(std::string(utf8.get_data(), static_cast<size_t>(utf8.length())), loaded, &err)) {
		return String::utf8(err.c_str());
	}
	sim_->clear_vehicles();
	map_ = std::move(loaded);
	sim_->on_map_changed();
	reset_clock();
	return String();
}

Array TrafficSim::get_road_lines(double max_step) const {
	Array out;
	std::vector<tsim::Vec2> pts;
	for (const tsim::Segment &seg : map_.segments()) {
		const double n = static_cast<double>(seg.lanes.size());
		const double half = n * seg.lane_width * 0.5;

		map_.offset_polyline(seg, 0.0, max_step, pts);
		Dictionary asphalt;
		asphalt["points"] = to_packed(pts);
		asphalt["width"] = n * seg.lane_width;
		asphalt["kind"] = 0;
		out.push_back(asphalt);

		for (int k = 0; k <= static_cast<int>(n); ++k) {
			const double off = half - static_cast<double>(k) * seg.lane_width;
			map_.offset_polyline(seg, off, max_step, pts);
			Dictionary line;
			line["points"] = to_packed(pts);
			line["width"] = 0.15;
			line["kind"] = (k == 0 || k == static_cast<int>(n)) ? 1 : 2;
			out.push_back(line);
		}
	}
	return out;
}

Rect2 TrafficSim::get_map_bounds() const {
	if (map_.nodes().empty()) {
		return Rect2();
	}
	double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
	auto grow = [&](double x, double y) {
		x0 = std::min(x0, x);
		y0 = std::min(y0, y);
		x1 = std::max(x1, x);
		y1 = std::max(y1, y);
	};
	for (const tsim::Node &n : map_.nodes()) {
		grow(n.pos.x, n.pos.y);
	}
	for (const tsim::Segment &s : map_.segments()) {
		if (s.kind == tsim::SegmentKind::Arc) {
			const tsim::Node *a = map_.node(s.from);
			const double r = a ? (a->pos - s.center).length() : 0.0;
			grow(s.center.x - r, s.center.y - r);
			grow(s.center.x + r, s.center.y + r);
		}
	}
	return Rect2(static_cast<real_t>(x0), static_cast<real_t>(y0), static_cast<real_t>(x1 - x0),
			static_cast<real_t>(y1 - y0));
}

int64_t TrafficSim::spawn_cars(int64_t count, int64_t seed) {
	const uint32_t n = static_cast<uint32_t>(std::clamp<int64_t>(count, 0, 1000000));
	const uint32_t placed = sim_->spawn_even(n, static_cast<uint64_t>(seed));
	reset_clock();
	return placed;
}

void TrafficSim::clear_cars() {
	sim_->clear_vehicles();
	reset_clock();
}

int64_t TrafficSim::get_vehicle_count() const { return static_cast<int64_t>(sim_->vehicle_count()); }

int64_t TrafficSim::advance(double real_delta, double speed, double budget_ms) {
	const double dt = sim_->config().dt;
	// Cap the frame delta so a stalled tab does not queue minutes of sim time.
	const double frame = std::clamp(real_delta, 0.0, 0.25);
	accumulator_ += frame * std::max(0.0, speed);

	const Clock::time_point start = Clock::now();
	int64_t ran = 0;
	while (accumulator_ >= dt) {
		sim_->tick();
		accumulator_ -= dt;
		++ran;
		if (elapsed_us(start) >= budget_ms * 1000.0) {
			break;
		}
	}
	const double spent_us = elapsed_us(start);
	behind_ = accumulator_ >= dt;
	if (behind_) {
		accumulator_ = dt * 0.999; // drop the backlog; keep interpolation smooth
	}
	last_frame_ticks_ = ran;
	last_frame_sim_ms_ = spent_us / 1000.0;
	frame_sim_ms_ema_ = ema(frame_sim_ms_ema_, last_frame_sim_ms_, 0.05);
	frame_ticks_ema_ = ema(frame_ticks_ema_, static_cast<double>(ran), 0.05);
	if (ran > 0) {
		tick_us_ema_ = ema(tick_us_ema_, spent_us / static_cast<double>(ran), 0.05);
	}
	window_real_ += frame;
	window_ticks_ += ran;
	if (window_real_ >= 1.0) {
		effective_speed_ = static_cast<double>(window_ticks_) * dt / window_real_;
		window_real_ = 0.0;
		window_ticks_ = 0;
	}
	return ran;
}

int64_t TrafficSim::step(int64_t ticks) {
	for (int64_t i = 0; i < ticks; ++i) {
		sim_->tick();
	}
	accumulator_ = 0.0;
	return ticks;
}

int64_t TrafficSim::get_tick() const { return static_cast<int64_t>(sim_->tick_count()); }

double TrafficSim::get_sim_time() const { return sim_->sim_time(); }

double TrafficSim::get_interpolation_alpha() const {
	return std::clamp(accumulator_ / sim_->config().dt, 0.0, 1.0);
}

PackedFloat32Array TrafficSim::get_render_buffer(double car_scale) {
	const Clock::time_point start = Clock::now();
	const size_t n = sim_->vehicle_count();
	const int64_t size = static_cast<int64_t>(n) * kFloatsPerCar;
	if (buffer_.size() != size) {
		buffer_.resize(size);
	}
	const double alpha = get_interpolation_alpha();
	const double k = std::max(1.0, car_scale);
	float *w = buffer_.ptrw();
	for (size_t i = 0; i < n; ++i) {
		const tsim::Pose p0 = sim_->vehicle_prev_pose(i);
		const tsim::Pose p1 = sim_->vehicle_pose(i);
		const tsim::Vec2 pos = p0.pos + (p1.pos - p0.pos) * alpha;
		const tsim::Vec2 dir = (p0.dir + (p1.dir - p0.dir) * alpha).normalized();
		float *o = w + i * kFloatsPerCar;
		// Transform2D layout: x.x, y.x, pad, origin.x, x.y, y.y, pad, origin.y.
		// x axis = heading, y axis = heading rotated +90 degrees.
		o[0] = static_cast<float>(dir.x * k);
		o[1] = static_cast<float>(-dir.y * k);
		o[2] = 0.0f;
		o[3] = static_cast<float>(pos.x);
		o[4] = static_cast<float>(dir.y * k);
		o[5] = static_cast<float>(dir.x * k);
		o[6] = 0.0f;
		o[7] = static_cast<float>(pos.y);
		speed_color(sim_->vehicle_speed(i) / sim_->vehicle_params(i).v0, o + 8);
	}
	render_prep_us_ema_ = ema(render_prep_us_ema_, elapsed_us(start), 0.05);
	return buffer_;
}

String TrafficSim::get_state_hash() const { return String(tsim::hash_to_hex(sim_->state_hash()).c_str()); }

Dictionary TrafficSim::get_stats() const {
	const tsim::SimStats st = sim_->stats();
	Dictionary d;
	d["tick"] = static_cast<int64_t>(sim_->tick_count());
	d["sim_time"] = sim_->sim_time();
	d["vehicles"] = static_cast<int64_t>(st.vehicles);
	d["mean_speed_kmh"] = st.mean_speed * 3.6;
	d["speed_stddev_kmh"] = st.speed_stddev * 3.6;
	d["stopped"] = static_cast<int64_t>(st.stopped);
	d["last_frame_ticks"] = last_frame_ticks_;
	d["last_frame_sim_ms"] = last_frame_sim_ms_;
	d["avg_frame_sim_ms"] = frame_sim_ms_ema_;
	d["avg_frame_ticks"] = frame_ticks_ema_;
	d["tick_us"] = tick_us_ema_;
	d["render_prep_us"] = render_prep_us_ema_;
	d["behind"] = behind_;
	d["effective_speed"] = effective_speed_;
	d["segments"] = static_cast<int64_t>(map_.segments().size());
	d["lanes"] = static_cast<int64_t>(map_.lanes().size());
	return d;
}

Dictionary TrafficSim::run_golden_check() const {
	const Clock::time_point start = Clock::now();
	tsim::Map map;
	tsim::build_golden_map(map);
	const std::string hash = tsim::hash_to_hex(tsim::run_scenario(map, tsim::GoldenScenario::kCars,
			tsim::GoldenScenario::kSeed, tsim::GoldenScenario::kTicks));
	Dictionary d;
	d["hash"] = String(hash.c_str());
	d["expected"] = String(tsim::GoldenScenario::kExpectedHash);
	d["pass"] = hash == tsim::GoldenScenario::kExpectedHash;
	d["cars"] = static_cast<int64_t>(tsim::GoldenScenario::kCars);
	d["ticks"] = static_cast<int64_t>(tsim::GoldenScenario::kTicks);
	d["ms"] = elapsed_us(start) / 1000.0;
	return d;
}

void TrafficSim::_bind_methods() {
	ClassDB::bind_method(D_METHOD("new_ring", "radius", "lanes", "lane_width", "speed_kmh"), &TrafficSim::new_ring);
	ClassDB::bind_method(D_METHOD("add_straight_road", "a", "b", "lanes", "speed_kmh"), &TrafficSim::add_straight_road);
	ClassDB::bind_method(D_METHOD("save_json"), &TrafficSim::save_json);
	ClassDB::bind_method(D_METHOD("load_json", "text"), &TrafficSim::load_json);
	ClassDB::bind_method(D_METHOD("get_road_lines", "max_step"), &TrafficSim::get_road_lines);
	ClassDB::bind_method(D_METHOD("get_map_bounds"), &TrafficSim::get_map_bounds);
	ClassDB::bind_method(D_METHOD("spawn_cars", "count", "seed"), &TrafficSim::spawn_cars);
	ClassDB::bind_method(D_METHOD("clear_cars"), &TrafficSim::clear_cars);
	ClassDB::bind_method(D_METHOD("get_vehicle_count"), &TrafficSim::get_vehicle_count);
	ClassDB::bind_method(D_METHOD("advance", "real_delta", "speed", "budget_ms"), &TrafficSim::advance);
	ClassDB::bind_method(D_METHOD("step", "ticks"), &TrafficSim::step);
	ClassDB::bind_method(D_METHOD("get_tick"), &TrafficSim::get_tick);
	ClassDB::bind_method(D_METHOD("get_sim_time"), &TrafficSim::get_sim_time);
	ClassDB::bind_method(D_METHOD("get_interpolation_alpha"), &TrafficSim::get_interpolation_alpha);
	ClassDB::bind_method(D_METHOD("get_render_buffer", "car_scale"), &TrafficSim::get_render_buffer);
	ClassDB::bind_method(D_METHOD("get_state_hash"), &TrafficSim::get_state_hash);
	ClassDB::bind_method(D_METHOD("get_stats"), &TrafficSim::get_stats);
	ClassDB::bind_method(D_METHOD("run_golden_check"), &TrafficSim::run_golden_check);
}

} // namespace godot
