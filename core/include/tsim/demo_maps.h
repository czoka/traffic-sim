// Generated maps: the M1 gate test network and the editor's starter town.
#pragma once

#include "tsim/document.h"

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

// Profile for a preset name (see profile_presets()); falls back to the first.
Profile preset_profile(const char *name, RoadMap &map);

} // namespace tsim
