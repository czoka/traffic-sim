#include "tsim/buildings.h"

#include <algorithm>
#include <cmath>

namespace tsim {

std::array<Vec2, 4> lot_corners(const Building &b, double width, double depth) {
	const Vec2 d = b.dir;
	// Seen from the street (looking along d), left is d rotated counter-clockwise on screen.
	const Vec2 left{ d.y, -d.x };
	const Vec2 hw = left * (0.5 * width);
	const Vec2 back = d * depth;
	return { b.pos + hw, b.pos - hw, b.pos - hw + back, b.pos + hw + back };
}

std::array<Vec2, 4> lot_corners(const Building &b, const CityData &data) {
	const BuildingType *t = data.type(b.type);
	return lot_corners(b, t ? t->width : 10.0, t ? t->depth : 10.0);
}

LotSnap snap_lot(const RoadGeometry &geom, Vec2 at, int level, double max_distance) {
	LotSnap best;
	double best_d = max_distance;
	for (const auto &kv : geom.segments()) {
		const SegmentGeom &sg = kv.second;
		if (sg.level != level || sg.rise != 0 || sg.p.size() < 2 || sg.lanes.empty()) continue;
		if (at.x < sg.bb_min.x - max_distance || at.x > sg.bb_max.x + max_distance || at.y < sg.bb_min.y - max_distance ||
				at.y > sg.bb_max.y + max_distance) {
			continue;
		}
		for (size_t k = 0; k + 1 < sg.p.size(); ++k) {
			// Closest point on this piece of the centreline.
			const Vec2 a = sg.p[k], b = sg.p[k + 1];
			const Vec2 ab = b - a;
			const double l2 = ab.dot(ab);
			const double t = l2 > 1e-12 ? std::clamp((at - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
			const Vec2 c = a + ab * t;
			const Vec2 n = (sg.n[k] + (sg.n[k + 1] - sg.n[k]) * t).normalized();
			double lo = 1e300, hi = -1e300;
			for (size_t i = 0; i < sg.lanes.size(); ++i) {
				lo = std::min(lo, sg.edge(k, i).first);
				hi = std::max(hi, sg.edge(k, i).second);
			}
			const double off = (at - c).dot(n);
			const bool right = off >= 0.5 * (lo + hi);
			const double edge = right ? hi : lo;
			const Vec2 ep = c + n * edge;
			const double d = (at - ep).length();
			if (d < best_d) {
				best_d = d;
				best.ok = true;
				best.pos = ep;
				best.dir = right ? n : n * -1.0;
				best.segment = sg.id;
				best.distance = d;
			}
		}
	}
	return best;
}

namespace {

void project(const std::array<Vec2, 4> &q, Vec2 axis, double &lo, double &hi) {
	lo = 1e300;
	hi = -1e300;
	for (const Vec2 &p : q) {
		const double v = p.dot(axis);
		lo = std::min(lo, v);
		hi = std::max(hi, v);
	}
}

} // namespace

bool quads_overlap(const std::array<Vec2, 4> &a, const std::array<Vec2, 4> &b, double margin) {
	for (const std::array<Vec2, 4> *q : { &a, &b }) {
		for (size_t i = 0; i < 4; ++i) {
			const Vec2 e = (*q)[(i + 1) % 4] - (*q)[i];
			const double len = e.length();
			if (len < 1e-9) continue;
			const Vec2 axis{ -e.y / len, e.x / len };
			double alo, ahi, blo, bhi;
			project(a, axis, alo, ahi);
			project(b, axis, blo, bhi);
			if (ahi <= blo + margin || bhi <= alo + margin) return false;
		}
	}
	return true;
}

bool point_in_quad(const std::array<Vec2, 4> &q, Vec2 p) {
	int sign = 0;
	for (size_t i = 0; i < 4; ++i) {
		const Vec2 e = q[(i + 1) % 4] - q[i];
		const double c = cross(e, p - q[i]);
		const int s = c > 0.0 ? 1 : c < 0.0 ? -1 : 0;
		if (s == 0) continue;
		if (sign == 0) sign = s;
		else if (s != sign) return false;
	}
	return true;
}

bool lot_overlaps_road(const std::array<Vec2, 4> &lot, const SegmentGeom &sg) {
	if (sg.p.size() < 2 || sg.lanes.empty()) return false;
	double lmin_x = 1e300, lmin_y = 1e300, lmax_x = -1e300, lmax_y = -1e300;
	for (const Vec2 &p : lot) {
		lmin_x = std::min(lmin_x, p.x);
		lmin_y = std::min(lmin_y, p.y);
		lmax_x = std::max(lmax_x, p.x);
		lmax_y = std::max(lmax_y, p.y);
	}
	if (lmax_x < sg.bb_min.x || lmin_x > sg.bb_max.x || lmax_y < sg.bb_min.y || lmin_y > sg.bb_max.y) return false;
	// Each piece of the road as a quad (its full cross-section, shrunk a little
	// so a lot on the back of the sidewalk doesn't count).
	for (size_t k = 0; k + 1 < sg.p.size(); ++k) {
		double lo0 = 1e300, hi0 = -1e300, lo1 = 1e300, hi1 = -1e300;
		for (size_t i = 0; i < sg.lanes.size(); ++i) {
			lo0 = std::min(lo0, sg.edge(k, i).first);
			hi0 = std::max(hi0, sg.edge(k, i).second);
			lo1 = std::min(lo1, sg.edge(k + 1, i).first);
			hi1 = std::max(hi1, sg.edge(k + 1, i).second);
		}
		const std::array<Vec2, 4> piece{ sg.at(k, lo0 + 0.2), sg.at(k, hi0 - 0.2), sg.at(k + 1, hi1 - 0.2),
			sg.at(k + 1, lo1 + 0.2) };
		if (quads_overlap(lot, piece, 0.0)) return true;
	}
	return false;
}

} // namespace tsim
