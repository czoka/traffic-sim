// Save files for the editable road map.
//
// Format "traffic-sim-map". Version history:
//   1  POC: one-way lanes with explicit connectors (migrated on load)
//   2  M1: levels, curves (straight/arc/bezier), road profiles, turn rules,
//      no-change zones
//   3  M2: junction control per node (right-hand, priority road, all-way
//      stop), spawn / sink points with rates and origin-destination weights.
//      Version 2 files load unchanged (the new fields have defaults).
// Numbers are written in shortest round-trip form and keys in a fixed order,
// so save -> load -> save gives an identical file.
#pragma once

#include "tsim/road_map.h"

#include <string>

namespace tsim {

constexpr int kRoadMapVersion = 3;

std::string road_map_to_json(const RoadMap &map);

// On success replaces `out`. `migrated_from` receives the file's version.
bool road_map_from_json(const std::string &text, RoadMap &out, std::string *err, int *migrated_from = nullptr);

// Profiles on their own (for user-saved presets).
std::string profile_to_json(const Profile &p);
bool profile_from_json(const std::string &text, Profile &out, std::string *err);

} // namespace tsim
