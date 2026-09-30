// Segment centreline curves: straight, circular arc, cubic Bézier.
//
// Lengths and turning angles are computed with + - * / and sqrt only
// (closed form or Gauss-Legendre quadrature) so they are identical on every
// platform. Point sampling for rendering may use trig.
#pragma once

#include "tsim/types.h"

#include <vector>

namespace tsim {

enum class CurveKind : uint8_t {
	Straight = 0,
	Arc = 1,
	Bezier = 2,
};

struct Curve {
	CurveKind kind = CurveKind::Straight;
	Vec2 p0; // start (the segment's from-node position)
	Vec2 p3; // end (the segment's to-node position)
	// Bezier: absolute control points.
	Vec2 c1;
	Vec2 c2;
	// Arc: centre and signed sweep (radians, positive = clockwise on screen).
	Vec2 center;
	double sweep = 0.0;

	Vec2 point(double t) const;
	Vec2 d1(double t) const; // first derivative
	Vec2 d2(double t) const; // second derivative
	Vec2 tangent(double t) const { return d1(t).normalized(); }

	// Total length (deterministic).
	double length() const;
	// Signed total turning angle (deterministic), positive = turning right.
	double turn() const;
	// Length of the curve offset laterally by `offset` (positive = right).
	double offset_length(double offset) const { return length() - offset * turn(); }
};

// Arc-length table so positions can be addressed by distance along the curve.
class ArcTable {
public:
	void build(const Curve &c, int samples = 64);
	double length() const { return s_.empty() ? 0.0 : s_.back(); }
	// Parameter t for a distance s along the curve (clamped).
	double t_at(double s) const;
	// Distance along the curve for parameter t.
	double s_at(double t) const;

private:
	std::vector<double> t_;
	std::vector<double> s_;
};

// Deterministic Gauss-Legendre speed integral of |c'(t)| over [a, b].
double curve_arc_length(const Curve &c, double a, double b);

// Splits a curve at parameter t into two curves (exact for all kinds).
void split_curve(const Curve &c, double t, Curve &first, Curve &second);

// Reverses direction (p0 <-> p3).
Curve reversed(const Curve &c);

// Closest point on the curve to p: returns the parameter t and distance.
double closest_t(const Curve &c, Vec2 p, double *dist = nullptr);

// Arc through p0 and p3 with a given signed sweep: computes the centre.
Vec2 arc_center_for(Vec2 p0, Vec2 p3, double sweep);

// Quadratic control point -> cubic control points.
void quadratic_to_cubic(Vec2 p0, Vec2 q, Vec2 p3, Vec2 &c1, Vec2 &c2);

inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

} // namespace tsim
