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
		ojson jl{ { "id", l.id }, { "type", lane_type_name(l.type) }, { "dir", dir_name(l.dir) }, { "width", l.width } };
		if (l.type == LaneType::Parking && l.parking != ParkingStyle::Parallel) jl["parking"] = parking_style_name(l.parking);
		lanes.push_back(std::move(jl));
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
		std::string ps;
		if (str(jl, "parking", ps) && !parking_style_from_name(ps, l.parking)) {
			err = "invalid parking style '" + ps + "'";
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

// --- v3 node extras: junction control and spawn points ------------------------------

bool parse_node_extras(const json &jn, RoadNode &n, std::string &err) {
	const std::string where = "node " + std::to_string(n.id) + ": ";
	auto ctl = jn.find("control");
	if (ctl != jn.end()) {
		std::string type;
		if (!ctl->is_object() || !str(*ctl, "type", type) || !junction_control_from_name(type, n.control)) {
			err = where + "invalid junction control";
			return false;
		}
		auto pr = ctl->find("priority");
		if (pr != ctl->end()) {
			if (!pr->is_array()) {
				err = where + "priority must be an array of segment ids";
				return false;
			}
			for (const json &v : *pr) {
				if (!v.is_number_unsigned() || v.get<uint64_t>() == 0 || v.get<uint64_t>() > 0xFFFFFFFFull) {
					err = where + "invalid priority segment id";
					return false;
				}
				n.priority.push_back(static_cast<SegmentId>(v.get<uint64_t>()));
			}
		}
	}
	auto sp = jn.find("spawner");
	if (sp != jn.end()) {
		Spawner &s = n.spawner;
		s.enabled = true;
		if (!sp->is_object() || !num(*sp, "rate", s.rate) || s.rate < 0.0) {
			err = where + "spawner needs a rate (vehicles per hour)";
			return false;
		}
		auto sink = sp->find("sink");
		if (sink != sp->end()) {
			if (!sink->is_boolean()) {
				err = where + "spawner sink must be true or false";
				return false;
			}
			s.sink = sink->get<bool>();
		}
		auto od = sp->find("od");
		if (od != sp->end()) {
			if (!od->is_array()) {
				err = where + "spawner od must be an array";
				return false;
			}
			for (const json &jw : *od) {
				OdWeight w;
				if (!jw.is_object() || !id(jw, "to", w.to) || !num(jw, "weight", w.weight) || w.weight < 0.0) {
					err = where + "invalid od weight";
					return false;
				}
				s.od.push_back(w);
			}
		}
		num(*sp, "bikes", s.bikes);
		num(*sp, "people", s.people);
		auto cl = sp->find("coaches");
		if (cl != sp->end()) {
			if (!cl->is_array()) {
				err = where + "coaches must be an array";
				return false;
			}
			for (const json &jc : *cl) {
				CoachLine c;
				if (!jc.is_object() || !id(jc, "id", c.id) || !id(jc, "exit", c.exit) || !num(jc, "per_hour", c.per_hour) ||
						!num(jc, "dwell", c.dwell)) {
					err = where + "invalid coach line";
					return false;
				}
				s.coaches.push_back(c);
			}
		}
	}
	auto rb = jn.find("roundabout");
	if (rb != jn.end()) {
		Roundabout &r = n.roundabout;
		r.enabled = true;
		if (!rb->is_object() || !num(*rb, "radius", r.radius) || !integer(*rb, "lanes", r.lanes) || r.lanes < 1 ||
				r.lanes > 3 || !(r.radius >= 5.0 && r.radius <= 100.0)) {
			err = where + "invalid roundabout";
			return false;
		}
		auto t = rb->find("turbo");
		if (t != rb->end() && t->is_boolean()) r.turbo = t->get<bool>();
		auto sl = rb->find("slip");
		if (sl != rb->end() && sl->is_array()) {
			for (const json &v : *sl) {
				if (v.is_number_unsigned()) r.slip.push_back(static_cast<SegmentId>(v.get<uint64_t>()));
			}
		}
	}
	auto sg = jn.find("signal");
	if (sg != jn.end()) {
		SignalPlan &pl = n.signal;
		if (!sg->is_object() || !num(*sg, "amber", pl.amber) || !num(*sg, "all_red", pl.all_red)) {
			err = where + "invalid signal plan";
			return false;
		}
		num(*sg, "offset", pl.offset);
		auto phases = sg->find("phases");
		if (phases == sg->end() || !phases->is_array()) {
			err = where + "signal plan needs phases";
			return false;
		}
		for (const json &jp : *phases) {
			SignalPhase ph;
			auto moves = jp.find("moves");
			if (!jp.is_object() || !num(jp, "green", ph.green) || moves == jp.end() || !moves->is_array()) {
				err = where + "invalid signal phase";
				return false;
			}
			for (const json &jm : *moves) {
				SignalMovement m;
				if (!jm.is_object() || !id(jm, "from", m.from) || !id(jm, "to", m.to)) {
					err = where + "invalid signal movement";
					return false;
				}
				auto pm = jm.find("permissive");
				if (pm != jm.end() && pm->is_boolean()) m.permissive = pm->get<bool>();
				ph.moves.push_back(m);
			}
			auto wk = jp.find("walk");
			if (wk != jp.end() && wk->is_array()) {
				for (const json &v : *wk) {
					if (v.is_number_unsigned()) ph.walk.push_back(static_cast<SegmentId>(v.get<uint64_t>()));
				}
			}
			pl.phases.push_back(ph);
		}
		auto ror = sg->find("right_on_red");
		if (ror != sg->end() && ror->is_array()) {
			for (const json &v : *ror) {
				if (v.is_number_unsigned()) pl.right_on_red.push_back(static_cast<SegmentId>(v.get<uint64_t>()));
			}
		}
	}
	auto dp = jn.find("depot");
	if (dp != jn.end()) {
		Depot &d = n.depot;
		d.enabled = true;
		if (!dp->is_object() || !integer(*dp, "capacity", d.capacity)) {
			err = where + "invalid depot";
			return false;
		}
		str(*dp, "name", d.name);
		auto routes = dp->find("routes");
		if (routes != dp->end()) {
			if (!routes->is_array()) {
				err = where + "depot routes must be an array";
				return false;
			}
			for (const json &jr : *routes) {
				BusRoute r;
				double color = 0.0;
				auto st = jr.find("stops");
				if (!jr.is_object() || !id(jr, "id", r.id) || !num(jr, "headway", r.headway) || st == jr.end() ||
						!st->is_array()) {
					err = where + "invalid bus route";
					return false;
				}
				str(jr, "name", r.name);
				if (num(jr, "color", color)) r.color = static_cast<uint32_t>(color);
				auto lp = jr.find("loop");
				if (lp != jr.end() && lp->is_boolean()) r.loop = lp->get<bool>();
				for (const json &v : *st) {
					if (v.is_number_unsigned()) r.stops.push_back(static_cast<uint32_t>(v.get<uint64_t>()));
				}
				d.routes.push_back(r);
			}
		}
	}
	return true;
}

bool parse_crossing(const json &jc, Crossing &c, bool mid_block) {
	std::string kind;
	if (!jc.is_object() || !str(jc, "kind", kind) || !crossing_kind_from_name(kind, c.kind)) return false;
	if (mid_block && (!id(jc, "id", c.id) || !num(jc, "u", c.u) || !(c.u >= 0.0 && c.u <= 1.0) ||
							 c.kind == CrossingKind::None)) {
		return false;
	}
	auto b = jc.find("bike");
	if (b != jc.end() && b->is_boolean()) c.bike = b->get<bool>();
	auto r = jc.find("refuge");
	if (r != jc.end() && r->is_boolean()) c.refuge = r->get<bool>();
	return true;
}

ojson crossing_json(const Crossing &c, bool mid_block) {
	ojson o;
	if (mid_block) {
		o["id"] = c.id;
		o["u"] = c.u;
	}
	o["kind"] = crossing_kind_name(c.kind);
	if (c.bike) o["bike"] = true;
	if (c.refuge) o["refuge"] = true;
	return o;
}

ojson node_json(const RoadNode &n) {
	ojson o{ { "id", n.id }, { "x", n.pos.x }, { "y", n.pos.y }, { "level", n.level } };
	if (n.control != JunctionControl::RightHand || !n.priority.empty()) {
		ojson c{ { "type", junction_control_name(n.control) } };
		if (!n.priority.empty()) c["priority"] = n.priority;
		o["control"] = std::move(c);
	}
	if (n.spawner.enabled) {
		ojson s{ { "rate", n.spawner.rate }, { "sink", n.spawner.sink } };
		if (!n.spawner.od.empty()) {
			ojson od = ojson::array();
			for (const OdWeight &w : n.spawner.od) od.push_back(ojson{ { "to", w.to }, { "weight", w.weight } });
			s["od"] = std::move(od);
		}
		if (n.spawner.bikes != 0.0) s["bikes"] = n.spawner.bikes;
		if (n.spawner.people != 0.0) s["people"] = n.spawner.people;
		if (!n.spawner.coaches.empty()) {
			ojson cl = ojson::array();
			for (const CoachLine &c : n.spawner.coaches) {
				cl.push_back(ojson{ { "id", c.id }, { "exit", c.exit }, { "per_hour", c.per_hour }, { "dwell", c.dwell } });
			}
			s["coaches"] = std::move(cl);
		}
		o["spawner"] = std::move(s);
	}
	if (n.roundabout.enabled) {
		ojson r{ { "radius", n.roundabout.radius }, { "lanes", n.roundabout.lanes } };
		if (n.roundabout.turbo) r["turbo"] = true;
		if (!n.roundabout.slip.empty()) r["slip"] = n.roundabout.slip;
		o["roundabout"] = std::move(r);
	}
	if (!n.signal.phases.empty() || !n.signal.right_on_red.empty()) {
		ojson sg{ { "amber", n.signal.amber }, { "all_red", n.signal.all_red } };
		if (n.signal.offset != 0.0) sg["offset"] = n.signal.offset;
		ojson phases = ojson::array();
		for (const SignalPhase &ph : n.signal.phases) {
			ojson moves = ojson::array();
			for (const SignalMovement &m : ph.moves) {
				ojson jm{ { "from", m.from }, { "to", m.to } };
				if (m.permissive) jm["permissive"] = true;
				moves.push_back(std::move(jm));
			}
			ojson jp{ { "green", ph.green }, { "moves", std::move(moves) } };
			if (!ph.walk.empty()) jp["walk"] = ph.walk;
			phases.push_back(std::move(jp));
		}
		sg["phases"] = std::move(phases);
		if (!n.signal.right_on_red.empty()) sg["right_on_red"] = n.signal.right_on_red;
		o["signal"] = std::move(sg);
	}
	if (n.depot.enabled) {
		ojson d{ { "capacity", n.depot.capacity } };
		if (!n.depot.name.empty()) d["name"] = n.depot.name;
		if (!n.depot.routes.empty()) {
			ojson routes = ojson::array();
			for (const BusRoute &r : n.depot.routes) {
				ojson jr{ { "id", r.id } };
				if (!r.name.empty()) jr["name"] = r.name;
				jr["color"] = r.color;
				jr["stops"] = r.stops;
				jr["headway"] = r.headway;
				if (r.loop) jr["loop"] = true;
				routes.push_back(std::move(jr));
			}
			d["routes"] = std::move(routes);
		}
		o["depot"] = std::move(d);
	}
	return o;
}

// --- v2 and v3 ---------------------------------------------------------------------

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
		if (!parse_node_extras(jn, n, err)) {
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
		integer(js, "rise", s.rise);
		if (s.rise < -2 || s.rise > 2) {
			err = where + "invalid rise";
			return false;
		}
		if (a->level != s.level || b->level != s.level + s.rise) {
			err = where + "nodes must be on the segment's level";
			return false;
		}
		auto st_flag = js.find("stairs");
		if (st_flag != js.end() && st_flag->is_boolean()) s.stairs = st_flag->get<bool>();
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
				auto jc = je.find("crossing");
				if (jc != je.end() && !parse_crossing(*jc, r.crossing, false)) {
					err = where + "invalid crossing";
					return false;
				}
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
		auto stops = js.find("stops");
		if (stops != js.end()) {
			if (!stops->is_array()) {
				err = where + "stops must be an array";
				return false;
			}
			for (const json &jst : *stops) {
				BusStop b;
				std::string kind, side;
				if (!jst.is_object() || !id(jst, "id", b.id) || !num(jst, "u", b.u) || !str(jst, "kind", kind) ||
						!stop_kind_from_name(kind, b.kind) || !str(jst, "side", side) || !dir_from(side, b.side) ||
						b.side == LaneDir::None || !(b.u >= 0.0 && b.u <= 1.0)) {
					err = where + "invalid bus stop";
					return false;
				}
				str(jst, "name", b.name);
				integer(jst, "bays", b.bays);
				s.stops.push_back(b);
			}
		}
		auto crossings = js.find("crossings");
		if (crossings != js.end()) {
			if (!crossings->is_array()) {
				err = where + "crossings must be an array";
				return false;
			}
			for (const json &jc : *crossings) {
				Crossing c;
				if (!parse_crossing(jc, c, true)) {
					err = where + "invalid crossing";
					return false;
				}
				s.crossings.push_back(c);
			}
		}
		auto fences = js.find("fences");
		if (fences != js.end()) {
			if (!fences->is_array()) {
				err = where + "fences must be an array";
				return false;
			}
			for (const json &jf : *fences) {
				Fence f;
				if (!jf.is_object() || !integer(jf, "side", f.side) || !num(jf, "from", f.u0) || !num(jf, "to", f.u1) ||
						f.side < 0 || f.side > 1 || !(f.u0 >= 0.0 && f.u0 < f.u1 && f.u1 <= 1.0)) {
					err = where + "invalid fence";
					return false;
				}
				s.fences.push_back(f);
			}
		}
		map.put_segment(s);
	}
	// v6: buildings.
	auto blds = root.find("buildings");
	if (blds != root.end()) {
		if (!blds->is_array()) {
			err = "buildings must be an array";
			return false;
		}
		for (const json &jb : *blds) {
			Building b;
			if (!jb.is_object() || !id(jb, "id", b.id) || b.id == 0 || !str(jb, "type", b.type) || !vec(jb, "pos", b.pos) ||
					!vec(jb, "dir", b.dir)) {
				err = "invalid building";
				return false;
			}
			const double len = b.dir.length();
			if (!(len > 0.5 && len < 1.5) || map.building(b.id)) {
				err = "building " + std::to_string(b.id) + ": invalid direction or duplicate id";
				return false;
			}
			integer(jb, "level", b.level);
			str(jb, "name", b.name);
			// v7: the player's economy settings.
			num(jb, "rent", b.rent);
			num(jb, "price_factor", b.price_factor);
			num(jb, "wage", b.wage);
			if (auto fs = jb.find("for_sale"); fs != jb.end() && fs->is_boolean()) b.for_sale = fs->get<bool>();
			num(jb, "asking", b.asking);
			if (!(b.rent >= 0.0 && b.wage >= 0.0 && b.asking >= 0.0 && b.price_factor > 0.0 && b.price_factor < 100.0)) {
				err = "building " + std::to_string(b.id) + ": invalid economy settings";
				return false;
			}
			map.put_building(b);
		}
	}
	// v7: the city centre marker.
	Vec2 centre;
	if (vec(root, "city_centre", centre)) map.set_city_centre(centre);
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
		uint32_t n = 1, s = 1, l = 1, o = 1;
		id(*next, "node", n);
		id(*next, "segment", s);
		id(*next, "lane", l);
		id(*next, "object", o);
		map.set_next_ids(n, s, l, o);
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
	if (map.next_object_id() > 1) root["next_ids"]["object"] = map.next_object_id();
	ojson nodes = ojson::array();
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		nodes.push_back(node_json(n));
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
		if (s.rise != 0) js["rise"] = s.rise;
		if (s.stairs) js["stairs"] = true;
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
				if (r.crossing.kind != CrossingKind::None) je["crossing"] = crossing_json(r.crossing, false);
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
		if (!s.stops.empty()) {
			ojson stops = ojson::array();
			for (const BusStop &b : s.stops) {
				ojson jst{ { "id", b.id }, { "u", b.u }, { "side", dir_name(b.side) }, { "kind", stop_kind_name(b.kind) } };
				if (!b.name.empty()) jst["name"] = b.name;
				if (b.bays != 1) jst["bays"] = b.bays;
				stops.push_back(std::move(jst));
			}
			js["stops"] = std::move(stops);
		}
		if (!s.crossings.empty()) {
			ojson cs = ojson::array();
			for (const Crossing &c : s.crossings) cs.push_back(crossing_json(c, true));
			js["crossings"] = std::move(cs);
		}
		if (!s.fences.empty()) {
			ojson fs = ojson::array();
			for (const Fence &f : s.fences) fs.push_back(ojson{ { "side", f.side }, { "from", f.u0 }, { "to", f.u1 } });
			js["fences"] = std::move(fs);
		}
		segments.push_back(std::move(js));
	}
	root["segments"] = std::move(segments);
	if (!map.buildings().empty()) {
		ojson bs = ojson::array();
		for (const auto &kv : map.buildings()) {
			const Building &b = kv.second;
			ojson jb;
			jb["id"] = b.id;
			jb["type"] = b.type;
			jb["pos"] = vec_json(b.pos);
			jb["dir"] = vec_json(b.dir);
			if (b.level != 0) jb["level"] = b.level;
			if (!b.name.empty()) jb["name"] = b.name;
			if (b.rent != 0.0) jb["rent"] = b.rent;
			if (b.price_factor != 1.0) jb["price_factor"] = b.price_factor;
			if (b.wage != 0.0) jb["wage"] = b.wage;
			if (b.for_sale) jb["for_sale"] = true;
			if (b.asking != 0.0) jb["asking"] = b.asking;
			bs.push_back(std::move(jb));
		}
		root["buildings"] = std::move(bs);
	}
	if (map.city_centre()) root["city_centre"] = vec_json(*map.city_centre());
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
