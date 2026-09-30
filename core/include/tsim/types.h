// Basic value types shared by the simulation core.
//
// Coordinate system: world units are metres, and the axes match Godot 2D
// (x to the right, y DOWN). "Right of travel" for a direction d is (-d.y, d.x).
//
// Determinism rule: anything that runs inside Simulation::tick() may only use
// + - * / and sqrt on doubles (all correctly rounded by IEEE 754). No sin, cos,
// atan2, pow or exp in the tick, because libm results differ between platforms.
// Build flags must also forbid FMA contraction (-ffp-contract=off).
#pragma once

#include <cmath>
#include <cstdint>

namespace tsim {

using NodeId = uint32_t;
using SegmentId = uint32_t;
using LaneId = uint32_t;
using VehicleId = uint32_t;

// IDs start at 1 and are never reused, so 0 always means "none".
constexpr uint32_t kNoId = 0;

constexpr double kPi = 3.141592653589793;
constexpr double kHalfPi = 1.5707963267948966;

struct Vec2 {
	double x = 0.0;
	double y = 0.0;

	constexpr Vec2() = default;
	constexpr Vec2(double px, double py) : x(px), y(py) {}

	constexpr Vec2 operator+(const Vec2 &o) const { return { x + o.x, y + o.y }; }
	constexpr Vec2 operator-(const Vec2 &o) const { return { x - o.x, y - o.y }; }
	constexpr Vec2 operator*(double k) const { return { x * k, y * k }; }
	constexpr bool operator==(const Vec2 &o) const { return x == o.x && y == o.y; }

	double length() const { return std::sqrt(x * x + y * y); }
	constexpr double dot(const Vec2 &o) const { return x * o.x + y * o.y; }
	// Right-hand normal in a y-down frame.
	constexpr Vec2 right() const { return { -y, x }; }
	Vec2 normalized() const {
		const double l = length();
		return l > 0.0 ? Vec2{ x / l, y / l } : Vec2{ 1.0, 0.0 };
	}
};

// Position and unit heading on a lane. Used for rendering only.
struct Pose {
	Vec2 pos;
	Vec2 dir{ 1.0, 0.0 };
};

} // namespace tsim
