#include "tsim/map_json.h"

// The core is built without exceptions; nlohmann is used in its no-throw mode
// (parse with allow_exceptions=false, and every access is type-checked first).
#ifndef JSON_NOEXCEPTION
#define JSON_NOEXCEPTION
#endif
#include <nlohmann/json.hpp>

namespace tsim {

using nlohmann::json;

namespace {

nlohmann::ordered_json vec_to_json(const Vec2 &v) { return nlohmann::ordered_json{ { "x", v.x }, { "y", v.y } }; }

bool get_number(const json &obj, const char *key, double &out) {
	auto it = obj.find(key);
	if (it == obj.end() || !it->is_number()) {
		return false;
	}
	out = it->get<double>();
	return std::isfinite(out);
}

bool get_id(const json &obj, const char *key, uint32_t &out) {
	auto it = obj.find(key);
	if (it == obj.end() || !it->is_number_unsigned()) {
		return false;
	}
	const uint64_t v = it->get<uint64_t>();
	if (v > 0xFFFFFFFFull) {
		return false;
	}
	out = static_cast<uint32_t>(v);
	return true;
}

bool get_vec(const json &obj, const char *key, Vec2 &out) {
	auto it = obj.find(key);
	if (it == obj.end() || !it->is_object()) {
		return false;
	}
	return get_number(*it, "x", out.x) && get_number(*it, "y", out.y);
}

bool get_id_list(const json &obj, const char *key, std::vector<uint32_t> &out) {
	out.clear();
	auto it = obj.find(key);
	if (it == obj.end()) {
		return true; // optional, empty
	}
	if (!it->is_array()) {
		return false;
	}
	for (const json &v : *it) {
		if (!v.is_number_unsigned() || v.get<uint64_t>() > 0xFFFFFFFFull) {
			return false;
		}
		out.push_back(static_cast<uint32_t>(v.get<uint64_t>()));
	}
	return true;
}

} // namespace

std::string map_to_json(const Map &map, int indent) {
	// ordered_json keeps keys in insertion order so files read top-down.
	using ojson = nlohmann::ordered_json;
	ojson root;
	root["format"] = "traffic-sim-map";
	root["version"] = kMapFormatVersion;
	root["next_ids"] = ojson{ { "node", map.next_node_id() }, { "segment", map.next_segment_id() },
		{ "lane", map.next_lane_id() } };

	ojson nodes = ojson::array();
	for (const Node &n : map.nodes()) {
		nodes.push_back(ojson{ { "id", n.id }, { "x", n.pos.x }, { "y", n.pos.y } });
	}
	root["nodes"] = std::move(nodes);

	ojson segments = ojson::array();
	for (const Segment &s : map.segments()) {
		ojson js{ { "id", s.id }, { "from", s.from }, { "to", s.to },
			{ "kind", s.kind == SegmentKind::Arc ? "arc" : "straight" }, { "lane_width", s.lane_width },
			{ "speed_limit", s.speed_limit }, { "lanes", s.lanes } };
		if (s.kind == SegmentKind::Arc) {
			js["center"] = vec_to_json(s.center);
			js["sweep"] = s.sweep;
		}
		segments.push_back(std::move(js));
	}
	root["segments"] = std::move(segments);

	ojson lanes = ojson::array();
	for (const Lane &l : map.lanes()) {
		lanes.push_back(ojson{ { "id", l.id }, { "segment", l.segment }, { "next", l.next } });
	}
	root["lanes"] = std::move(lanes);

	return root.dump(indent) + "\n";
}

bool map_from_json(const std::string &text, Map &out, std::string *err) {
	auto fail = [err](const std::string &msg) {
		if (err) *err = msg;
		return false;
	};
	const json root = json::parse(text, nullptr, /*allow_exceptions=*/false);
	if (root.is_discarded() || !root.is_object()) {
		return fail("not valid JSON");
	}
	auto fmt = root.find("format");
	if (fmt == root.end() || !fmt->is_string() || fmt->get<std::string>() != "traffic-sim-map") {
		return fail("not a traffic-sim map file");
	}
	uint32_t version = 0;
	if (!get_id(root, "version", version)) {
		return fail("missing version");
	}
	if (version > static_cast<uint32_t>(kMapFormatVersion)) {
		return fail("map was saved by a newer version (" + std::to_string(version) + ")");
	}
	// Version migrations go here once the format changes (none yet for v1).

	Map map;
	MapBuilder builder(map);
	std::string berr;

	auto nodes = root.find("nodes");
	auto segments = root.find("segments");
	auto lanes = root.find("lanes");
	if (nodes == root.end() || !nodes->is_array() || segments == root.end() || !segments->is_array() ||
			lanes == root.end() || !lanes->is_array()) {
		return fail("nodes, segments and lanes must be arrays");
	}

	for (const json &jn : *nodes) {
		Node n;
		if (!jn.is_object() || !get_id(jn, "id", n.id) || !get_number(jn, "x", n.pos.x) ||
				!get_number(jn, "y", n.pos.y)) {
			return fail("invalid node entry");
		}
		if (!builder.add_node(n, &berr)) {
			return fail(berr);
		}
	}

	for (const json &js : *segments) {
		Segment s;
		if (!js.is_object() || !get_id(js, "id", s.id) || !get_id(js, "from", s.from) || !get_id(js, "to", s.to) ||
				!get_number(js, "lane_width", s.lane_width) || !get_number(js, "speed_limit", s.speed_limit) ||
				!get_id_list(js, "lanes", s.lanes)) {
			return fail("invalid segment entry");
		}
		auto kind = js.find("kind");
		if (kind == js.end() || !kind->is_string()) {
			return fail("segment " + std::to_string(s.id) + " has no kind");
		}
		const std::string k = kind->get<std::string>();
		if (k == "straight") {
			s.kind = SegmentKind::Straight;
		} else if (k == "arc") {
			s.kind = SegmentKind::Arc;
			if (!get_vec(js, "center", s.center) || !get_number(js, "sweep", s.sweep)) {
				return fail("arc segment " + std::to_string(s.id) + " needs center and sweep");
			}
		} else {
			return fail("segment " + std::to_string(s.id) + " has unknown kind '" + k + "'");
		}
		if (!builder.add_segment(s, &berr)) {
			return fail(berr);
		}
	}

	for (const json &jl : *lanes) {
		Lane l;
		if (!jl.is_object() || !get_id(jl, "id", l.id) || !get_id(jl, "segment", l.segment) ||
				!get_id_list(jl, "next", l.next)) {
			return fail("invalid lane entry");
		}
		if (!builder.add_lane(l, &berr)) {
			return fail(berr);
		}
	}

	uint32_t nn = 1, ns = 1, nl = 1;
	auto next_ids = root.find("next_ids");
	if (next_ids != root.end() && next_ids->is_object()) {
		get_id(*next_ids, "node", nn);
		get_id(*next_ids, "segment", ns);
		get_id(*next_ids, "lane", nl);
	}
	if (!builder.finish(nn, ns, nl, &berr)) {
		return fail(berr);
	}
	out = std::move(map);
	return true;
}

} // namespace tsim
