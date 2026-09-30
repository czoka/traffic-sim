#include "tsim/road_map_json.h"

#ifndef JSON_NOEXCEPTION
#define JSON_NOEXCEPTION
#endif
#include <nlohmann/json.hpp>

#include <cmath>
#include <set>

namespace tsim {

using nlohmann::json;
using ojson = nlohmann::ordered_json;

namespace {

// --- enum <-> string ---------------------------------------------------------

const char *dir_name(LaneDir d) {
	switch (d) {
		case LaneDir::Forward:
			return "forward";
		case LaneDir::Backward:
			return "backward";
		case LaneDir::None:
			return "none";
	}
	return "none";
}

bool dir_from(const std::string &s, LaneDir &out) {
	if (s == "forward") out = LaneDir::Forward;
	else if (s == "backward") out = LaneDir::Backward;
	else if (s == "none") out = LaneDir::None;
	else return false;
	return true;
}

const char *median_name(MedianType m) {
	switch (m) {
		case MedianType::None:
			return "none";
		case MedianType::Painted:
			return "painted";
		case MedianType::Raised:
			return "raised";
	}
	return "none";
}

bool median_from(const std::string &s, MedianType &out) {
	if (s == "none") out = MedianType::None;
	else if (s == "painted") out = MedianType::Painted;
	else if (s == "raised") out = MedianType::Raised;
	else return false;
	return true;
}

const char *rule_name(TurnRule r) {
	switch (r) {
		case TurnRule::Disallowed:
			return "disallowed";
		case TurnRule::Allowed:
			return "allowed";
		case TurnRule::TurnLane:
			return "turn_lane";
	}
	return "allowed";
}

bool rule_from(const std::string &s, TurnRule &out) {
	if (s == "disallowed") out = TurnRule::Disallowed;
	else if (s == "allowed") out = TurnRule::Allowed;
	else if (s == "turn_lane") out = TurnRule::TurnLane;
	else return false;
	return true;
}

const char *kind_name(SegmentKind k) {
	switch (k) {
		case SegmentKind::Road:
			return "road";
		case SegmentKind::Footpath:
			return "footpath";
		case SegmentKind::BikePath:
			return "bike_path";
		case SegmentKind::SharedPath:
			return "shared_path";
	}
	return "road";
}

bool kind_from(const std::string &s, SegmentKind &out) {
	if (s == "road") out = SegmentKind::Road;
	else if (s == "footpath") out = SegmentKind::Footpath;
	else if (s == "bike_path") out = SegmentKind::BikePath;
	else if (s == "shared_path") out = SegmentKind::SharedPath;
	else return false;
	return true;
}

// --- checked getters (no exceptions) --------------------------------------------

bool num(const json &o, const char *k, double &out) {
	auto it = o.find(k);
	if (it == o.end() || !it->is_number()) return false;
	out = it->get<double>();
	return std::isfinite(out);
}

bool id(const json &o, const char *k, uint32_t &out) {
	auto it = o.find(k);
	if (it == o.end() || !it->is_number_unsigned() || it->get<uint64_t>() > 0xFFFFFFFFull) return false;
	out = static_cast<uint32_t>(it->get<uint64_t>());
	return true;
}

bool integer(const json &o, const char *k, int &out) {
	auto it = o.find(k);
	if (it == o.end() || !it->is_number_integer()) return false;
	const int64_t v = it->get<int64_t>();
	if (v < -1000 || v > 1000) return false;
	out = static_cast<int>(v);
	return true;
}

bool str(const json &o, const char *k, std::string &out) {
	auto it = o.find(k);
	if (it == o.end() || !it->is_string()) return false;
	out = it->get<std::string>();
	return true;
}

bool vec(const json &o, const char *k, Vec2 &out) {
	auto it = o.find(k);
	return it != o.end() && it->is_object() && num(*it, "x", out.x) && num(*it, "y", out.y);
}

ojson vec_json(Vec2 v) { return ojson{ { "x", v.x }, { "y", v.y } }; }

// --- profile -------------------------------------------------------------------

ojson profile_json(const Profile &p) {
	ojson lanes = ojson::array();
	for (const LaneSpec &l : p.lanes) {
		lanes.push_back(ojson{ { "id", l.id }, { "type", lane_type_name(l.type) }, { "dir", dir_name(l.dir) },
				{ "width", l.width } });
	}
	ojson o;
	o["median"] = median_name(p.median);
	if (p.median != MedianType::None) {
		o["median_width"] = p.median_width;
	}
	o["lanes"] = std::move(lanes);
	return o;
}

bool parse_profile(const json &o, Profile &p, std::string &err) {
	p = Profile{};
	std::string m;
	if (!o.is_object() || !str(o, "median", m) || !median_from(m, p.median)) {
		err = "profile needs a median type";
		return false;
	}
	if (p.median != MedianType::None && !num(o, "median_width", p.median_width)) {
		err = "profile median needs a width";
		return false;
	}
	auto lanes = o.find("lanes");
	if (lanes == o.end() || !lanes->is_array()) {
		err = "profile needs lanes";
		return false;
	}
	for (const json &jl : *lanes) {
		LaneSpec l;
		std::string t, d;
		if (!jl.is_object() || !id(jl, "id", l.id) || !str(jl, "type", t) || !lane_type_from_name(t, l.type) ||
				!str(jl, "dir", d) || !dir_from(d, l.dir) || !num(jl, "width", l.width)) {
			err = "invalid lane in profile";
			return false;
		}
		p.lanes.push_back(l);
	}
	err = validate_profile(p);
	return err.empty();
}

// --- v1 (POC) migration ---------------------------------------------------------

bool migrate_v1(const json &root, RoadMap &map, std::string &err) {
	auto nodes = root.find("nodes");
	auto segments = root.find("segments");
	if (nodes == root.end() || !nodes->is_array() || segments == root.end() || !segments->is_array()) {
		err = "v1 map needs nodes and segments";
		return false;
	}
	for (const json &jn : *nodes) {
		RoadNode n;
		if (!id(jn, "id", n.id) || !num(jn, "x", n.pos.x) || !num(jn, "y", n.pos.y)) {
			err = "invalid v1 node";
			return false;
		}
		map.put_node(n);
	}
	for (const json &js : *segments) {
		RoadSegment s;
		std::string kind;
		double lane_width = 3.5;
		std::vector<uint32_t> lane_ids;
		if (!id(js, "id", s.id) || !id(js, "from", s.from) || !id(js, "to", s.to) || !str(js, "kind", kind) ||
				!num(js, "lane_width", lane_width) || !num(js, "speed_limit", s.speed_limit)) {
			err = "invalid v1 segment";
			return false;
		}
		auto jl = js.find("lanes");
		if (jl == js.end() || !jl->is_array() || jl->empty()) {
			err = "v1 segment without lanes";
			return false;
		}
		for (const json &v : *jl) {
			if (!v.is_number_unsigned()) {
				err = "invalid v1 lane id";
				return false;
			}
			lane_ids.push_back(static_cast<uint32_t>(v.get<uint64_t>()));
		}
		if (kind == "arc") {
			s.curve = CurveKind::Arc;
			if (!num(js, "sweep", s.sweep)) {
				err = "v1 arc without sweep";
				return false;
			}
		} else {
			s.curve = CurveKind::Straight;
		}
		// v1 listed lanes right to left; v2 lists them left to right.
		for (size_t i = lane_ids.size(); i-- > 0;) {
			s.profile.lanes.push_back(LaneSpec{ lane_ids[i], LaneType::General, LaneDir::Forward, lane_width });
		}
		map.put_segment(s);
	}
	auto next = root.find("next_ids");
	if (next != root.end() && next->is_object()) {
		uint32_t n = 1, s = 1, l = 1;
		id(*next, "node", n);
		id(*next, "segment", s);
		id(*next, "lane", l);
		map.set_next_ids(n, s, l);
	}
	return true;
}

// --- v2 ----------------------------------------------------------------------------

bool parse_v2(const json &root, RoadMap &map, std::string &err) {
	auto nodes = root.find("nodes");
	auto segments = root.find("segments");
	if (nodes == root.end() || !nodes->is_array() || segments == root.end() || !segments->is_array()) {
		err = "map needs nodes and segments arrays";
		return false;
	}
	for (const json &jn : *nodes) {
		RoadNode n;
		if (!jn.is_object() || !id(jn, "id", n.id) || n.id == kNoId || !num(jn, "x", n.pos.x) ||
				!num(jn, "y", n.pos.y) || !integer(jn, "level", n.level)) {
			err = "invalid node";
			return false;
		}
		if (map.node(n.id)) {
			err = "duplicate node id " + std::to_string(n.id);
			return false;
		}
		map.put_node(n);
	}
	for (const json &js : *segments) {
		RoadSegment s;
		std::string kind;
		if (!js.is_object() || !id(js, "id", s.id) || s.id == kNoId || !id(js, "from", s.from) ||
				!id(js, "to", s.to) || !str(js, "kind", kind) || !kind_from(kind, s.kind) ||
				!integer(js, "level", s.level) || !num(js, "speed_limit", s.speed_limit)) {
			err = "invalid segment";
			return false;
		}
		const std::string where = "segment " + std::to_string(s.id) + ": ";
		if (map.segment(s.id)) {
			err = "duplicate segment id " + std::to_string(s.id);
			return false;
		}
		const RoadNode *a = map.node(s.from);
		const RoadNode *b = map.node(s.to);
		if (!a || !b || s.from == s.to) {
			err = where + "invalid end nodes";
			return false;
		}
		if (a->level != s.level || b->level != s.level) {
			err = where + "nodes must be on the segment's level";
			return false;
		}
		auto curve = js.find("curve");
		std::string type;
		if (curve == js.end() || !str(*curve, "type", type)) {
			err = where + "missing curve";
			return false;
		}
		if (type == "straight") {
			s.curve = CurveKind::Straight;
		} else if (type == "arc") {
			s.curve = CurveKind::Arc;
			if (!num(*curve, "sweep", s.sweep) || s.sweep == 0.0 || std::fabs(s.sweep) >= 2.0 * kPi) {
				err = where + "invalid arc sweep";
				return false;
			}
		} else if (type == "bezier") {
			s.curve = CurveKind::Bezier;
			if (!vec(*curve, "c1", s.c1) || !vec(*curve, "c2", s.c2)) {
				err = where + "bezier needs c1 and c2";
				return false;
			}
		} else {
			err = where + "unknown curve type '" + type + "'";
			return false;
		}
		auto prof = js.find("profile");
		if (prof == js.end() || !parse_profile(*prof, s.profile, err)) {
			err = where + (err.empty() ? "missing profile" : err);
			return false;
		}
		str(js, "name", s.name);
		auto ends = js.find("ends");
		if (ends != js.end()) {
			if (!ends->is_array() || ends->size() != 2) {
				err = where + "ends must have two entries";
				return false;
			}
			for (size_t e = 0; e < 2; ++e) {
				const json &je = (*ends)[e];
				EndRules &r = s.ends[e];
				std::string l, rr;
				if (!str(je, "left", l) || !rule_from(l, r.left) || !str(je, "right", rr) ||
						!rule_from(rr, r.right) || !num(je, "turn_lane_length", r.turn_lane_length)) {
					err = where + "invalid turn rules";
					return false;
				}
				id(je, "left_lane", r.left_lane);
				id(je, "right_lane", r.right_lane);
			}
		}
		auto zones = js.find("no_change");
		if (zones != js.end()) {
			if (!zones->is_array()) {
				err = where + "no_change must be an array";
				return false;
			}
			for (const json &jz : *zones) {
				NoChangeZone z;
				std::string block;
				if (!integer(jz, "edge", z.edge) || !num(jz, "from", z.u0) || !num(jz, "to", z.u1) ||
						!str(jz, "block", block) || z.edge < 1 ||
						z.edge >= static_cast<int>(s.profile.lanes.size()) || !(z.u0 >= 0.0 && z.u0 < z.u1 && z.u1 <= 1.0)) {
					err = where + "invalid no-change zone";
					return false;
				}
				z.block_left_to_right = block == "both" || block == "left_to_right";
				z.block_right_to_left = block == "both" || block == "right_to_left";
				if (!z.block_left_to_right && !z.block_right_to_left) {
					err = where + "invalid no-change block '" + block + "'";
					return false;
				}
				s.no_change.push_back(z);
			}
		}
		map.put_segment(s);
	}
	// Lane IDs must be unique across the map.
	std::set<LaneId> lanes;
	for (const auto &kv : map.segments()) {
		for (const LaneSpec &l : kv.second.profile.lanes) {
			if (l.id == kNoId || !lanes.insert(l.id).second) {
				err = "duplicate or zero lane id " + std::to_string(l.id);
				return false;
			}
		}
	}
	auto next = root.find("next_ids");
	if (next != root.end() && next->is_object()) {
		uint32_t n = 1, s = 1, l = 1;
		id(*next, "node", n);
		id(*next, "segment", s);
		id(*next, "lane", l);
		map.set_next_ids(n, s, l);
	}
	return true;
}

} // namespace

std::string road_map_to_json(const RoadMap &map) {
	ojson root;
	root["format"] = "traffic-sim-map";
	root["version"] = kRoadMapVersion;
	root["next_ids"] = ojson{ { "node", map.next_node_id() }, { "segment", map.next_segment_id() },
		{ "lane", map.next_lane_id() } };
	ojson nodes = ojson::array();
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		nodes.push_back(ojson{ { "id", n.id }, { "x", n.pos.x }, { "y", n.pos.y }, { "level", n.level } });
	}
	root["nodes"] = std::move(nodes);

	ojson segments = ojson::array();
	for (const auto &kv : map.segments()) {
		const RoadSegment &s = kv.second;
		ojson js;
		js["id"] = s.id;
		js["from"] = s.from;
		js["to"] = s.to;
		js["kind"] = kind_name(s.kind);
		js["level"] = s.level;
		switch (s.curve) {
			case CurveKind::Straight:
				js["curve"] = ojson{ { "type", "straight" } };
				break;
			case CurveKind::Arc:
				js["curve"] = ojson{ { "type", "arc" }, { "sweep", s.sweep } };
				break;
			case CurveKind::Bezier:
				js["curve"] = ojson{ { "type", "bezier" }, { "c1", vec_json(s.c1) }, { "c2", vec_json(s.c2) } };
				break;
		}
		js["speed_limit"] = s.speed_limit;
		if (!s.name.empty()) {
			js["name"] = s.name;
		}
		js["profile"] = profile_json(s.profile);
		if (!(s.ends[0] == EndRules{}) || !(s.ends[1] == EndRules{})) {
			ojson ends = ojson::array();
			for (const EndRules &r : s.ends) {
				ojson je{ { "left", rule_name(r.left) }, { "right", rule_name(r.right) },
					{ "turn_lane_length", r.turn_lane_length } };
				if (r.left_lane != kNoId) je["left_lane"] = r.left_lane;
				if (r.right_lane != kNoId) je["right_lane"] = r.right_lane;
				ends.push_back(std::move(je));
			}
			js["ends"] = std::move(ends);
		}
		if (!s.no_change.empty()) {
			ojson zones = ojson::array();
			for (const NoChangeZone &z : s.no_change) {
				const char *block = z.block_left_to_right && z.block_right_to_left ? "both"
						: z.block_left_to_right										 ? "left_to_right"
																					 : "right_to_left";
				zones.push_back(ojson{ { "edge", z.edge }, { "from", z.u0 }, { "to", z.u1 }, { "block", block } });
			}
			js["no_change"] = std::move(zones);
		}
		segments.push_back(std::move(js));
	}
	root["segments"] = std::move(segments);
	return root.dump(1) + "\n";
}

bool road_map_from_json(const std::string &text, RoadMap &out, std::string *err_out, int *migrated_from) {
	std::string err;
	const json root = json::parse(text, nullptr, false);
	auto fail = [&](const std::string &m) {
		if (err_out) *err_out = m;
		return false;
	};
	if (root.is_discarded() || !root.is_object()) {
		return fail("not valid JSON");
	}
	std::string format;
	if (!str(root, "format", format) || format != "traffic-sim-map") {
		return fail("not a traffic-sim map file");
	}
	uint32_t version = 0;
	if (!id(root, "version", version) || version == 0) {
		return fail("missing version");
	}
	if (version > static_cast<uint32_t>(kRoadMapVersion)) {
		return fail("map was saved by a newer version (" + std::to_string(version) + ")");
	}
	RoadMap map;
	const bool ok = version == 1 ? migrate_v1(root, map, err) : parse_v2(root, map, err);
	if (!ok) {
		return fail(err);
	}
	if (migrated_from) {
		*migrated_from = static_cast<int>(version);
	}
	out = std::move(map);
	return true;
}

std::string profile_to_json(const Profile &p) { return profile_json(p).dump(1) + "\n"; }

bool profile_from_json(const std::string &text, Profile &out, std::string *err_out) {
	const json root = json::parse(text, nullptr, false);
	std::string err;
	Profile p;
	if (root.is_discarded() || !parse_profile(root, p, err)) {
		if (err_out) *err_out = err.empty() ? "not valid JSON" : err;
		return false;
	}
	out = p;
	return true;
}

} // namespace tsim
