// RoadEditor: M7, insight - heatmap data and per-junction stats.
#include "m3_dicts.h"
#include "road_editor.h"

#include <godot_cpp/core/class_db.hpp>

namespace godot {

using namespace tsim;
using namespace m3;

namespace {

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }

} // namespace

Dictionary RoadEditor::sim_lane_lines(int level) {
	Dictionary d;
	Array lines;
	PackedInt32Array lanes;
	const Network *net = sim_.traffic().network();
	if (net) {
		for (size_t l = 0; l < net->lanes.size(); ++l) {
			const NetLane &lane = net->lanes[l];
			if (lane.kind != NetLaneKind::Road || (lane.level != level && lane.level_end != level)) continue;
			PackedVector2Array pts;
			for (const Vec2 &p : lane.pts) pts.push_back(gv(p));
			lines.push_back(pts);
			lanes.push_back(static_cast<int32_t>(l));
		}
	}
	d["lines"] = lines;
	d["lanes"] = lanes;
	d["lane_count"] = static_cast<int64_t>(net ? net->lanes.size() : 0);
	return d;
}

PackedFloat32Array RoadEditor::sim_lane_heat(const String &mode) {
	PackedFloat32Array out;
	const Traffic &t = sim_.traffic();
	const std::vector<LaneHeat> &heat = t.lane_heat();
	out.resize(static_cast<int64_t>(heat.size()));
	const int m = mode == "wait" ? 1 : mode == "flow" ? 2 : 0;
	for (size_t l = 0; l < heat.size(); ++l) {
		const LaneHeat &h = heat[l];
		float v = -1.0f; // no traffic lately
		if (h.seen || (m == 2 && h.flow > 0.5)) {
			v = static_cast<float>(m == 0 ? h.speed_ratio : m == 1 ? h.wait : h.flow);
		}
		out.set(static_cast<int64_t>(l), v);
	}
	return out;
}

Dictionary RoadEditor::sim_junction_stats(int64_t node) {
	Dictionary d;
	const JunctionStats s = sim_.traffic().junction_stats(static_cast<NodeId>(node));
	d["found"] = s.found;
	if (!s.found) return d;
	d["approaches"] = s.approaches;
	d["flow"] = s.flow;
	d["mean_wait"] = s.mean_wait;
	d["queued"] = static_cast<int64_t>(s.queued);
	d["longest_wait"] = s.longest_wait;
	d["passed"] = static_cast<int64_t>(s.passed);
	d["total_mean_wait"] = s.total_mean_wait;
	d["worst_segment"] = static_cast<int64_t>(s.worst_segment);
	d["worst_wait"] = s.worst_wait;
	if (s.worst_segment != kNoId) {
		const RoadSegment *seg = doc_.map().segment(s.worst_segment);
		d["worst_name"] = seg ? gstr(seg->name) : String();
	}
	return d;
}

void RoadEditor::bind_m7_methods() {
	ClassDB::bind_method(D_METHOD("sim_lane_lines", "level"), &RoadEditor::sim_lane_lines);
	ClassDB::bind_method(D_METHOD("sim_lane_heat", "mode"), &RoadEditor::sim_lane_heat);
	ClassDB::bind_method(D_METHOD("sim_junction_stats", "node"), &RoadEditor::sim_junction_stats);
}

} // namespace godot
