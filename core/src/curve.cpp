#include "tsim/curve.h"

#include <algorithm>
#include <cmath>

namespace tsim {

namespace {

// 5-point Gauss-Legendre nodes and weights on [-1, 1].
constexpr double kGLx[5] = { -0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831,
	0.9061798459386640 };
constexpr double kGLw[5] = { 0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665,
	0.2369268850561891 };
constexpr int kSubIntervals = 16;

template <typename F>
double integrate(double a, double b, F f) {
	double total = 0.0;
	const double h = (b - a) / kSubIntervals;
	for (int k = 0; k < kSubIntervals; ++k) {
		const double lo = a + h * k;
		const double mid = lo + 0.5 * h;
		const double half = 0.5 * h;
		double part = 0.0;
		for (int i = 0; i < 5; ++i) {
			part += kGLw[i] * f(mid + half * kGLx[i]);
		}
		total += part * half;
	}
	return total;
}

double arc_radius(const Curve &c) { return (c.p0 - c.center).length(); }

} // namespace

Vec2 Curve::point(double t) const {
	switch (kind) {
		case CurveKind::Straight:
			return p0 + (p3 - p0) * t;
		case CurveKind::Arc: {
			const Vec2 r0 = p0 - center;
			const double r = r0.length();
			const double th = std::atan2(r0.y, r0.x) + sweep * t;
			return center + Vec2{ std::cos(th), std::sin(th) } * r;
		}
		case CurveKind::Bezier: {
			const double u = 1.0 - t;
			return p0 * (u * u * u) + c1 * (3.0 * u * u * t) + c2 * (3.0 * u * t * t) + p3 * (t * t * t);
		}
	}
	return p0;
}

Vec2 Curve::d1(double t) const {
	switch (kind) {
		case CurveKind::Straight:
			return p3 - p0;
		case CurveKind::Arc: {
			const Vec2 r0 = p0 - center;
			const double r = r0.length();
			const double th = std::atan2(r0.y, r0.x) + sweep * t;
			return Vec2{ -std::sin(th), std::cos(th) } * (r * sweep);
		}
		case CurveKind::Bezier: {
			const double u = 1.0 - t;
			return ((c1 - p0) * (u * u) + (c2 - c1) * (2.0 * u * t) + (p3 - c2) * (t * t)) * 3.0;
		}
	}
	return p3 - p0;
}

Vec2 Curve::d2(double t) const {
	switch (kind) {
		case CurveKind::Straight:
			return Vec2{};
		case CurveKind::Arc: {
			const Vec2 r0 = p0 - center;
			const double r = r0.length();
			const double th = std::atan2(r0.y, r0.x) + sweep * t;
			return Vec2{ std::cos(th), std::sin(th) } * (-r * sweep * sweep);
		}
		case CurveKind::Bezier: {
			const double u = 1.0 - t;
			return ((c2 - c1 * 2.0 + p0) * u + (p3 - c2 * 2.0 + c1) * t) * 6.0;
		}
	}
	return Vec2{};
}

double curve_arc_length(const Curve &c, double a, double b) {
	switch (c.kind) {
		case CurveKind::Straight:
			return (c.p3 - c.p0).length() * (b - a);
		case CurveKind::Arc:
			return arc_radius(c) * std::fabs(c.sweep) * (b - a);
		case CurveKind::Bezier:
			return integrate(a, b, [&c](double t) { return c.d1(t).length(); });
	}
	return 0.0;
}

double Curve::length() const { return curve_arc_length(*this, 0.0, 1.0); }

double Curve::turn() const {
	switch (kind) {
		case CurveKind::Straight:
			return 0.0;
		case CurveKind::Arc:
			return sweep;
		case CurveKind::Bezier:
			return integrate(0.0, 1.0, [this](double t) {
				const Vec2 a = d1(t);
				const double sq = a.dot(a);
				return sq > 1e-12 ? cross(a, d2(t)) / sq : 0.0;
			});
	}
	return 0.0;
}

void ArcTable::build(const Curve &c, int samples) {
	const int n = c.kind == CurveKind::Bezier ? std::max(8, samples) : 1;
	t_.assign(static_cast<size_t>(n) + 1, 0.0);
	s_.assign(static_cast<size_t>(n) + 1, 0.0);
	for (int i = 1; i <= n; ++i) {
		const double t0 = static_cast<double>(i - 1) / n;
		const double t1 = static_cast<double>(i) / n;
		t_[static_cast<size_t>(i)] = t1;
		s_[static_cast<size_t>(i)] = s_[static_cast<size_t>(i - 1)] + curve_arc_length(c, t0, t1);
	}
}

double ArcTable::t_at(double s) const {
	if (s_.size() < 2 || s <= 0.0) {
		return 0.0;
	}
	if (s >= s_.back()) {
		return 1.0;
	}
	const auto it = std::upper_bound(s_.begin(), s_.end(), s);
	const size_t i = static_cast<size_t>(it - s_.begin());
	const double seg = s_[i] - s_[i - 1];
	const double k = seg > 0.0 ? (s - s_[i - 1]) / seg : 0.0;
	return t_[i - 1] + (t_[i] - t_[i - 1]) * k;
}

double ArcTable::s_at(double t) const {
	if (s_.size() < 2 || t <= 0.0) {
		return 0.0;
	}
	if (t >= 1.0) {
		return s_.back();
	}
	const auto it = std::upper_bound(t_.begin(), t_.end(), t);
	const size_t i = static_cast<size_t>(it - t_.begin());
	const double k = (t - t_[i - 1]) / (t_[i] - t_[i - 1]);
	return s_[i - 1] + (s_[i] - s_[i - 1]) * k;
}

void split_curve(const Curve &c, double t, Curve &first, Curve &second) {
	first = c;
	second = c;
	const Vec2 mid = c.point(t);
	first.p3 = mid;
	second.p0 = mid;
	switch (c.kind) {
		case CurveKind::Straight:
			break;
		case CurveKind::Arc:
			first.sweep = c.sweep * t;
			second.sweep = c.sweep * (1.0 - t);
			break;
		case CurveKind::Bezier: {
			// De Casteljau.
			const Vec2 a = c.p0 + (c.c1 - c.p0) * t;
			const Vec2 b = c.c1 + (c.c2 - c.c1) * t;
			const Vec2 d = c.c2 + (c.p3 - c.c2) * t;
			const Vec2 ab = a + (b - a) * t;
			const Vec2 bd = b + (d - b) * t;
			first.c1 = a;
			first.c2 = ab;
			second.c1 = bd;
			second.c2 = d;
			break;
		}
	}
}

Curve reversed(const Curve &c) {
	Curve r = c;
	r.p0 = c.p3;
	r.p3 = c.p0;
	r.c1 = c.c2;
	r.c2 = c.c1;
	r.sweep = -c.sweep;
	return r;
}

double closest_t(const Curve &c, Vec2 p, double *dist) {
	double best_t = 0.0;
	if (c.kind == CurveKind::Straight) {
		const Vec2 d = c.p3 - c.p0;
		const double len2 = d.dot(d);
		best_t = len2 > 0.0 ? std::clamp((p - c.p0).dot(d) / len2, 0.0, 1.0) : 0.0;
	} else {
		// Coarse scan, then golden-section refinement around the best sample.
		const int n = 48;
		double best_d = 1e300;
		for (int i = 0; i <= n; ++i) {
			const double t = static_cast<double>(i) / n;
			const Vec2 q = c.point(t) - p;
			const double d = q.dot(q);
			if (d < best_d) {
				best_d = d;
				best_t = t;
			}
		}
		double lo = std::max(0.0, best_t - 1.0 / n);
		double hi = std::min(1.0, best_t + 1.0 / n);
		const double g = 0.6180339887498949;
		for (int it = 0; it < 40; ++it) {
			const double a = hi - (hi - lo) * g;
			const double b = lo + (hi - lo) * g;
			const Vec2 qa = c.point(a) - p;
			const Vec2 qb = c.point(b) - p;
			if (qa.dot(qa) < qb.dot(qb)) {
				hi = b;
			} else {
				lo = a;
			}
		}
		best_t = 0.5 * (lo + hi);
	}
	if (dist) {
		*dist = (c.point(best_t) - p).length();
	}
	return best_t;
}

Vec2 arc_center_for(Vec2 p0, Vec2 p3, double sweep) {
	const Vec2 chord = p3 - p0;
	const double len = chord.length();
	const Vec2 mid = p0 + chord * 0.5;
	if (len <= 0.0 || sweep == 0.0) {
		return mid;
	}
	const Vec2 right = Vec2{ -chord.y, chord.x } * (1.0 / len);
	const double h = (0.5 * len) / std::tan(0.5 * sweep);
	return mid + right * h;
}

void quadratic_to_cubic(Vec2 p0, Vec2 q, Vec2 p3, Vec2 &c1, Vec2 &c2) {
	c1 = p0 + (q - p0) * (2.0 / 3.0);
	c2 = p3 + (q - p3) * (2.0 / 3.0);
}

} // namespace tsim
