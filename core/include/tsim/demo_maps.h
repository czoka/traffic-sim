// Generated maps: the M1 gate test network and the editor's starter town.
#pragma once

#include "tsim/document.h"
#include "tsim/road_geometry.h"

namespace tsim {

// Grid of cols x rows junctions `spacing` metres apart. Every third row is an
// avenue (2+2, raised median, turn pockets), columns alternate between two-way
// streets and one-way streets, and one diagonal Bézier road cuts across.
// Roads run out of the grid on every side to spawn / sink points; avenues are
// priority roads and a few street junctions are all-way stops (M2 gate).
// Recorded as one undo step.
void build_test_grid(Document &doc, int cols, int rows, double spacing);

// Small M2 test maps, each with spawn points.
void build_t_junction(Document &doc); // priority road with a side street
void build_lane_drop(Document &doc); // two lanes merge into one
void build_one_way_pair(Document &doc); // parallel one-way streets and cross streets

// A small town that shows every M1 road feature.
void build_demo_town(Document &doc);

// M3 showcase: a signalized junction (with a right-on-red arrow), a two-lane
// roundabout with a slip lane, bus lanes, parallel / 45° / 90° parking, bike
// lanes and bikes, kerbside and bay stops, the main station, a depot with a
// loop and an end-to-end route, and a coach line. Recorded as one undo step.
void build_showcase(Document &doc);

// M4: people and levels. Signal crosswalks with a scramble phase, zebras (one
// mid-block with a refuge), an uncontrolled crossing, fences, a park footpath,
// a bike path into a junction, a pedestrian bridge over the avenue, a car
// overpass, a bus route, and people at every spawn point.
void build_people_town(Document &doc);

// M4 gate: the test grid at a larger size with people at every spawn point,
// signals with walk phases on the avenues and zebras on some streets.
void build_people_city(Document &doc, int cols, int rows);

// M5: what a new map starts with: a street from the west edge to the east
// edge with the main station, a coach line through it, and the city offices.
void build_new_city(Document &doc);
// M7 tutorial: the new city plus Loop Road (south of High Street and back, so
// a bus can go round) and Depot Lane, a dead end for a bus depot.
void build_tutorial(Document &doc);
// M5 test town: the new city plus a cross street with homes (townhouses,
// houses, apartment blocks), a grocery, fast food, a restaurant and an office.
void build_city_town(Document &doc);
// M5 gate: a 6 x 6 block town for about 5,000 residents, with shops, offices,
// a bus loop, the main station and coaches.
void build_city_week(Document &doc);
// M6 gate: a 5 x 5 block town for about 1,500 residents with a bike shop, a
// car dealership, a bus loop and the city centre marker; some buildings are
// on the market from the start.
void build_city_market(Document &doc);
// Places a building of `type` on the street edge nearest `near` (level 0).
// Returns its id, 0 when there is no street within 60 m.
uint32_t place_building(Document &doc, const RoadGeometry &geom, const char *type, Vec2 near);

// Profile for a preset name (see profile_presets()); falls back to the first.
Profile preset_profile(const char *name, RoadMap &map);

} // namespace tsim
