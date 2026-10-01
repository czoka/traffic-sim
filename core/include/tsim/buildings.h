// Building lots (M5): footprints, snapping a new lot to a street, and the
// overlap checks the problems panel uses.
#pragma once

#include "tsim/city_data.h"
#include "tsim/road_geometry.h"
#include "tsim/road_map.h"

#include <array>

namespace tsim {

// Corners of a lot: front-left, front-right, back-right, back-left (front on
// the street, seen from the street).
std::array<Vec2, 4> lot_corners(const Building &b, double width, double depth);
std::array<Vec2, 4> lot_corners(const Building &b, const CityData &data);

struct LotSnap {
	bool ok = false;
	Vec2 pos; // front centre, at the outer edge of the street (sidewalk included)
	Vec2 dir; // into the lot
	SegmentId segment = kNoId;
	double distance = 0.0; // from the click to the street edge
};
// The street edge nearest `at` on `level` (roads and paths), within
// `max_distance` of it, with the lot on the side of the click.
LotSnap snap_lot(const RoadGeometry &geom, Vec2 at, int level, double max_distance = 60.0);

// Whether two convex quads overlap (separating axis test, with a small margin
// so lots that only touch don't count).
bool quads_overlap(const std::array<Vec2, 4> &a, const std::array<Vec2, 4> &b, double margin = 0.05);
// Whether a lot overlaps a segment's drawn cross-section on its level.
bool lot_overlaps_road(const std::array<Vec2, 4> &lot, const SegmentGeom &sg);
bool point_in_quad(const std::array<Vec2, 4> &q, Vec2 p);

} // namespace tsim
