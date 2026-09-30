// JSON save/load for maps. Numbers are written in shortest round-trip form,
// so save -> load -> save yields an identical file and identical doubles.
#pragma once

#include "tsim/map.h"

#include <string>

namespace tsim {

constexpr int kMapFormatVersion = 1;

std::string map_to_json(const Map &map, int indent = 1);

// Replaces the contents of `out` on success. On failure `out` is left
// untouched and `err` describes the problem.
bool map_from_json(const std::string &text, Map &out, std::string *err);

} // namespace tsim
