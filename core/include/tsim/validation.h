// Problems panel v1: checks run on the map and its derived geometry.
#pragma once

#include "tsim/road_geometry.h"
#include "tsim/road_map.h"

#include <string>
#include <vector>

namespace tsim {

enum class Severity : uint8_t {
	Warning = 0, // shown, does not block anything
	Error = 1, // will block Play once cars move (M2)
};

struct Problem {
	Severity severity = Severity::Warning;
	std::string code; // stable identifier, e.g. "overlap"
	std::string message;
	Vec2 pos; // where to look
	int level = 0;
	std::vector<SegmentId> segments;
	std::vector<NodeId> nodes;
	std::vector<uint32_t> buildings; // M5
};

std::vector<Problem> validate(const RoadMap &map, const RoadGeometry &geom);

} // namespace tsim
