// Generated maps: the M1 gate test network and the editor's starter town.
#pragma once

#include "tsim/document.h"

namespace tsim {

// Grid of cols x rows junctions `spacing` metres apart. Every third row is an
// avenue (2+2, raised median, turn pockets), columns alternate between two-way
// streets and one-way streets, and one diagonal Bézier road cuts across.
// Recorded as one undo step.
void build_test_grid(Document &doc, int cols, int rows, double spacing);

// A small town that shows every M1 road feature.
void build_demo_town(Document &doc);

// Profile for a preset name (see profile_presets()); falls back to the first.
Profile preset_profile(const char *name, RoadMap &map);

} // namespace tsim
