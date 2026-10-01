#include "road_editor.h"
#include "m3_dicts.h"

#include "tsim/demo_maps.h"
#include "tsim/road_map_json.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace godot {
using namespace m3;

using namespace tsim;

namespace {

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }
Vec2 tv(Vector2 v) { return Vec2{ v.x, v.y }; }
std::string ss(const String &s) {
	const CharString c = s.utf8();
	return std::string(c.get_data(), static_cast<size_t>(c.length()));
}
String gs(const std::string &s) { return String::utf8(s.c_str(), static_cast<int64_t>(s.size())); }

const char *dir_name(LaneDir d) {
	return d == LaneDir::Forward ? "forward" : d == LaneDir::Backward ? "backward" : "none";
}
LaneDir dir_from(const String &s) {
	if (s == "forward") return LaneDir::Forward;
	if (s == "backward") return LaneDir::Backward;
	return LaneDir::None;
}
const char *median_name(MedianType m) {
	return m == MedianType::Painted ? "painted" : m == MedianType::Raised ? "raised" : "none";
}
MedianType median_from(const String &s) {
	if (s == "painted") return MedianType::Painted;
	if (s == "raised") return MedianType::Raised;
	return MedianType::None;
}
const char *rule_name(TurnRule r) {
	return r == TurnRule::Disallowed ? "disallowed" : r == TurnRule::TurnLane ? "turn_lane" : "allowed";
}
TurnRule rule_from(const String &s) {
	if (s == "disallowed") return TurnRule::Disallowed;
	if (s == "turn_lane") return TurnRule::TurnLane;
	return TurnRule::Allowed;
}

Dictionary params_dict(const ProfileParams &p) {
	Dictionary d;
	d["backward"] = p.backward;
	d["forward"] = p.forward;
	d["lane_width"] = p.lane_width;
	d["median"] = median_name(p.median);
	d["median_width"] = p.median_width;
	d["sidewalk_left"] = p.sidewalk_left;
	d["sidewalk_right"] = p.sidewalk_right;
	d["sidewalk_width"] = p.sidewalk_width;
	d["parking_left"] = p.parking_left;
	d["parking_right"] = p.parking_right;
	d["bike_left"] = p.bike_left;
	d["bike_right"] = p.bike_right;
	d["bus_left"] = p.bus_left;
	d["bus_right"] = p.bus_right;
	d["parking_style"] = parking_style_name(p.parking_style);
	return d;
}

ProfileParams params_from(const Dictionary &d) {
	ProfileParams p;
	p.backward = static_cast<int>(d.get("backward", p.backward));
	p.forward = static_cast<int>(d.get("forward", p.forward));
	p.lane_width = static_cast<double>(d.get("lane_width", p.lane_width));
	p.median = median_from(d.get("median", "none"));
	p.median_width = static_cast<double>(d.get("median_width", p.median_width));
	p.sidewalk_left = d.get("sidewalk_left", p.sidewalk_left);
	p.sidewalk_right = d.get("sidewalk_right", p.sidewalk_right);
	p.sidewalk_width = static_cast<double>(d.get("sidewalk_width", p.sidewalk_width));
	p.parking_left = d.get("parking_left", p.parking_left);
	p.parking_right = d.get("parking_right", p.parking_right);
	p.bike_left = d.get("bike_left", p.bike_left);
	p.bike_right = d.get("bike_right", p.bike_right);
	p.bus_left = d.get("bus_left", p.bus_left);
	p.bus_right = d.get("bus_right", p.bus_right);
	parking_style_from_name(ss(d.get("parking_style", "parallel")), p.parking_style);
	return p;
}

Dictionary profile_dict(const Profile &p) {
	Dictionary d;
	d["median"] = median_name(p.median);
	d["median_width"] = p.median_width;
	Array lanes;
	for (const LaneSpec &l : p.lanes) {
		Dictionary ld;
		ld["id"] = static_cast<int64_t>(l.id);
		ld["type"] = lane_type_name(l.type);
		ld["dir"] = dir_name(l.dir);
		ld["width"] = l.width;
		lanes.push_back(ld);
	}
	d["lanes"] = lanes;
	d["width"] = p.total_width();
	return d;
}

Profile profile_from(const Dictionary &d) {
	Profile p;
	p.median = median_from(d.get("median", "none"));
	p.median_width = static_cast<double>(d.get("median_width", 0.0));
	const Array lanes = d.get("lanes", Array());
	for (int64_t i = 0; i < lanes.size(); ++i) {
		const Dictionary ld = lanes[i];
		LaneSpec l;
		l.id = static_cast<LaneId>(static_cast<int64_t>(ld.get("id", 0)));
		LaneType t = LaneType::General;
		lane_type_from_name(ss(ld.get("type", "general")), t);
		l.type = t;
		l.dir = dir_from(ld.get("dir", "none"));
		l.width = static_cast<double>(ld.get("width", default_width(t)));
		p.lanes.push_back(l);
	}
	return p;
}

PointRef point_from(const Dictionary &d) {
	PointRef p;
	p.pos = tv(d.get("pos", Vector2()));
	p.node = static_cast<NodeId>(static_cast<int64_t>(d.get("node", 0)));
	p.segment = static_cast<SegmentId>(static_cast<int64_t>(d.get("segment", 0)));
	return p;
}

PackedVector2Array packed(const std::vector<Vec2> &pts) {
	PackedVector2Array out;
	out.resize(static_cast<int64_t>(pts.size()));
	Vector2 *w = out.ptrw();
	for (size_t i = 0; i < pts.size(); ++i) w[i] = gv(pts[i]);
	return out;
}

} // namespace

// --- Map and files ------------------------------------------------------------------

void RoadEditor::new_map() { doc_.reset(RoadMap{}); }

void RoadEditor::load_demo_town() {
	Document d;
	build_demo_town(d);
	doc_.reset(d.map());
}

void RoadEditor::load_test_grid(int cols, int rows, double spacing) {
	Document d;
	build_test_grid(d, std::clamp(cols, 2, 40), std::clamp(rows, 2, 40), std::clamp(spacing, 40.0, 1000.0));
	doc_.reset(d.map());
}

bool RoadEditor::load_example(const String &name) {
	Document d;
	if (name == "town") build_demo_town(d);
	else if (name == "grid") build_test_grid(d, 7, 8, 120.0);
	else if (name == "t_junction") build_t_junction(d);
	else if (name == "lane_drop") build_lane_drop(d);
	else if (name == "one_way_pair") build_one_way_pair(d);
	else if (name == "showcase") build_showcase(d);
	else if (name == "people") build_people_town(d);
	else return false;
	doc_.reset(d.map());
	return true;
}

String RoadEditor::save_json() const { return gs(road_map_to_json(doc_.map())); }

Dictionary RoadEditor::load_json(const String &text) {
	Dictionary r;
	RoadMap m;
	std::string err;
	int version = 0;
	const bool ok = road_map_from_json(ss(text), m, &err, &version);
	r["ok"] = ok;
	r["error"] = gs(err);
	r["migrated_from"] = ok && version < kRoadMapVersion ? version : 0;
	if (ok) doc_.reset(m);
	return r;
}

int64_t RoadEditor::revision() const { return static_cast<int64_t>(doc_.revision()); }

// --- History --------------------------------------------------------------------------

void RoadEditor::begin(const String &label) { doc_.begin(ss(label)); }
bool RoadEditor::commit() { return doc_.commit(); }
void RoadEditor::cancel() { doc_.cancel(); }
bool RoadEditor::undo() { return doc_.undo(); }
bool RoadEditor::redo() { return doc_.redo(); }
bool RoadEditor::can_undo() const { return doc_.can_undo(); }
bool RoadEditor::can_redo() const { return doc_.can_redo(); }
String RoadEditor::undo_label() const { return gs(doc_.undo_label()); }
String RoadEditor::redo_label() const { return gs(doc_.redo_label()); }
int64_t RoadEditor::history_size() const { return static_cast<int64_t>(doc_.undo_count()); }
int64_t RoadEditor::redo_size() const { return static_cast<int64_t>(doc_.redo_count()); }

// --- Edits ------------------------------------------------------------------------------

Profile RoadEditor::road_profile(const Dictionary &road) {
	RoadMap scratch;
	if (road.has("profile")) return profile_from(road["profile"]);
	if (road.has("params")) return build_profile(params_from(road["params"]), nullptr, scratch);
	return preset_profile(ss(road.get("preset", "Street 1+1")).c_str(), scratch);
}

PackedInt64Array RoadEditor::add_road(const Array &points, const Dictionary &road, int level, double speed_kmh) {
	std::vector<PointRef> pts;
	for (int64_t i = 0; i < points.size(); ++i) pts.push_back(point_from(points[i]));
	const std::vector<SegmentId> ids = doc_.add_road(pts, road_profile(road), level, speed_kmh / 3.6);
	PackedInt64Array out;
	for (SegmentId id : ids) out.push_back(id);
	return out;
}

int64_t RoadEditor::add_curve(const Dictionary &a, Vector2 control, const Dictionary &b, const Dictionary &road,
		int level, double speed_kmh) {
	return doc_.add_curve(point_from(a), tv(control), point_from(b), road_profile(road), level, speed_kmh / 3.6);
}

void RoadEditor::move_node(int64_t id, Vector2 pos) { doc_.move_node(static_cast<NodeId>(id), tv(pos)); }
bool RoadEditor::merge_nodes(int64_t from, int64_t into) {
	return doc_.merge_nodes(static_cast<NodeId>(from), static_cast<NodeId>(into));
}
void RoadEditor::set_control_point(int64_t seg, int which, Vector2 pos) {
	doc_.set_control_point(static_cast<SegmentId>(seg), which, tv(pos));
}
int64_t RoadEditor::split_segment(int64_t seg, double u) { return doc_.split_segment(static_cast<SegmentId>(seg), u); }
void RoadEditor::delete_segment(int64_t id) { doc_.delete_segment(static_cast<SegmentId>(id)); }
void RoadEditor::delete_node(int64_t id) { doc_.delete_node(static_cast<NodeId>(id)); }
String RoadEditor::set_profile_params(int64_t seg, const Dictionary &params) {
	return gs(doc_.set_profile_params(static_cast<SegmentId>(seg), params_from(params)));
}
String RoadEditor::set_profile(int64_t seg, const Dictionary &profile) {
	return gs(doc_.set_profile(static_cast<SegmentId>(seg), profile_from(profile)));
}
String RoadEditor::set_lane_type(int64_t seg, int64_t lane, const String &type) {
	LaneType t;
	if (!lane_type_from_name(ss(type), t)) return "unknown lane type";
	return gs(doc_.set_lane_type(static_cast<SegmentId>(seg), static_cast<LaneId>(lane), t));
}
bool RoadEditor::flip(int64_t seg) { return doc_.flip(static_cast<SegmentId>(seg)); }
void RoadEditor::set_end_rules(int64_t seg, int end, const Dictionary &rules) {
	EndRules r;
	r.left = rule_from(rules.get("left", "allowed"));
	r.right = rule_from(rules.get("right", "allowed"));
	r.turn_lane_length = static_cast<double>(rules.get("turn_lane_length", 40.0));
	doc_.set_end_rules(static_cast<SegmentId>(seg), end, r);
}
void RoadEditor::set_speed_kmh(int64_t seg, double kmh) { doc_.set_speed_limit(static_cast<SegmentId>(seg), kmh / 3.6); }
void RoadEditor::set_segment_name(int64_t seg, const String &name) {
	doc_.set_name(static_cast<SegmentId>(seg), ss(name));
}
void RoadEditor::set_level(int64_t seg, int level) { doc_.set_level(static_cast<SegmentId>(seg), level); }
void RoadEditor::set_no_change(int64_t seg, int edge, double u0, double u1, bool block_l2r, bool block_r2l) {
	doc_.set_no_change(static_cast<SegmentId>(seg), edge, u0, u1, block_l2r, block_r2l);
}

void RoadEditor::set_junction_control(int64_t node, const String &control, const PackedInt64Array &priority) {
	JunctionControl c = JunctionControl::RightHand;
	junction_control_from_name(ss(control), c);
	std::vector<SegmentId> segs;
	for (int64_t i = 0; i < priority.size(); ++i) segs.push_back(static_cast<SegmentId>(priority[i]));
	doc_.set_junction_control(static_cast<NodeId>(node), c, segs);
}

void RoadEditor::set_spawner(int64_t node, const Dictionary &d) {
	Spawner sp;
	sp.enabled = d.get("enabled", true);
	sp.rate = static_cast<double>(d.get("rate", sp.rate));
	sp.sink = d.get("sink", sp.sink);
	const Array od = d.get("od", Array());
	for (int64_t i = 0; i < od.size(); ++i) {
		const Dictionary w = od[i];
		OdWeight ow;
		ow.to = static_cast<NodeId>(static_cast<int64_t>(w.get("to", 0)));
		ow.weight = static_cast<double>(w.get("weight", 1.0));
		sp.od.push_back(ow);
	}
	sp.bikes = static_cast<double>(d.get("bikes", 0.0));
	const Array coaches = d.get("coaches", Array());
	for (int64_t i = 0; i < coaches.size(); ++i) {
		const Dictionary c = coaches[i];
		CoachLine cl;
		cl.id = static_cast<uint32_t>(static_cast<int64_t>(c.get("id", 0)));
		cl.exit = static_cast<NodeId>(static_cast<int64_t>(c.get("exit", 0)));
		cl.per_hour = static_cast<double>(c.get("per_hour", cl.per_hour));
		cl.dwell = static_cast<double>(c.get("dwell", cl.dwell));
		sp.coaches.push_back(cl);
	}
	doc_.set_spawner(static_cast<NodeId>(node), sp);
}

// --- Profiles --------------------------------------------------------------------------

Array RoadEditor::presets() const {
	Array out;
	for (const ProfilePreset &p : profile_presets()) {
		Dictionary d;
		d["name"] = p.name;
		d["params"] = params_dict(p.params);
		out.push_back(d);
	}
	return out;
}

Dictionary RoadEditor::params_of_profile(const Dictionary &profile) const {
	return params_dict(params_of(profile_from(profile)));
}

Dictionary RoadEditor::profile_from_params(const Dictionary &params) const {
	RoadMap scratch;
	Profile p = build_profile(params_from(params), nullptr, scratch);
	for (LaneSpec &l : p.lanes) l.id = kNoId;
	return profile_dict(p);
}

String RoadEditor::validate_profile(const Dictionary &profile) const {
	return gs(tsim::validate_profile(profile_from(profile)));
}

// --- Queries ------------------------------------------------------------------------------

void RoadEditor::ensure_geometry() {
	if (built_revision_ == doc_.revision()) return;
	using Clock = std::chrono::steady_clock;
	const auto t0 = Clock::now();
	geom_.build(doc_.map());
	const auto t1 = Clock::now();
	problems_ = validate(doc_.map(), geom_);
	const auto t2 = Clock::now();
	build_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
	validate_ms_ = std::chrono::duration<double, std::milli>(t2 - t1).count();
	built_revision_ = doc_.revision();
}

Dictionary RoadEditor::get_node(int64_t id) {
	ensure_geometry();
	Dictionary d;
	const RoadNode *n = doc_.map().node(static_cast<NodeId>(id));
	if (!n) return d;
	d["id"] = id;
	d["pos"] = gv(n->pos);
	d["level"] = n->level;
	const NodeGeom *g = geom_.node(n->id);
	d["kind"] = g ? node_kind_name(g->kind) : "isolated";
	PackedInt64Array segs;
	for (SegmentId s : doc_.map().segments_at(n->id)) segs.push_back(s);
	d["segments"] = segs;
	d["connectors"] = g ? static_cast<int64_t>(g->connectors.size()) : 0;
	d["control"] = junction_control_name(n->control);
	PackedInt64Array prio;
	for (SegmentId p : n->priority) prio.push_back(p);
	d["priority"] = prio;
	// Legs for the junction editor: segment, direction away from the node, name.
	Array legs;
	if (g) {
		for (const Leg &l : g->legs) {
			Dictionary ld;
			ld["segment"] = static_cast<int64_t>(l.seg);
			ld["dir"] = gv(l.dir);
			const RoadSegment *s = doc_.map().segment(l.seg);
			ld["name"] = s ? gs(s->name) : String();
			ld["priority"] = n->is_priority(l.seg);
			legs.push_back(ld);
		}
	}
	d["legs"] = legs;
	d["junction"] = g && g->kind == NodeKind::Junction && g->legs.size() >= 3;
	d["road_end"] = g && g->kind == NodeKind::End;
	Dictionary sp;
	sp["enabled"] = n->spawner.enabled;
	sp["rate"] = n->spawner.rate;
	sp["sink"] = n->spawner.sink;
	Array od;
	for (const OdWeight &w : n->spawner.od) {
		Dictionary wd;
		wd["to"] = static_cast<int64_t>(w.to);
		wd["weight"] = w.weight;
		od.push_back(wd);
	}
	sp["od"] = od;
	sp["bikes"] = n->spawner.bikes;
	Array coaches;
	for (const CoachLine &c : n->spawner.coaches) {
		Dictionary cd;
		cd["id"] = static_cast<int64_t>(c.id);
		cd["exit"] = static_cast<int64_t>(c.exit);
		cd["per_hour"] = c.per_hour;
		cd["dwell"] = c.dwell;
		coaches.push_back(cd);
	}
	sp["coaches"] = coaches;
	d["spawner"] = sp;
	d["roundabout"] = roundabout_dict(n->roundabout);
	d["signal"] = signal_dict(n->signal);
	d["depot"] = depot_dict(n->depot);
	return d;
}

Dictionary RoadEditor::get_segment(int64_t id) {
	ensure_geometry();
	Dictionary d;
	const RoadSegment *s = doc_.map().segment(static_cast<SegmentId>(id));
	if (!s) return d;
	const SegmentGeom *g = geom_.segment(s->id);
	d["id"] = id;
	d["from"] = static_cast<int64_t>(s->from);
	d["to"] = static_cast<int64_t>(s->to);
	d["level"] = s->level;
	d["curve"] = s->curve == CurveKind::Arc ? "arc" : s->curve == CurveKind::Bezier ? "bezier" : "straight";
	d["c1"] = gv(s->c1);
	d["c2"] = gv(s->c2);
	d["length"] = g ? g->length : 0.0;
	d["speed_kmh"] = s->speed_limit * 3.6;
	d["name"] = gs(s->name);
	d["one_way"] = s->profile.one_way();
	d["profile"] = profile_dict(s->profile);
	d["params"] = params_dict(params_of(s->profile));
	Array ends;
	for (int e = 0; e < 2; ++e) {
		const EndRules &r = s->ends[e];
		const NodeId node = e == 0 ? s->from : s->to;
		const NodeGeom *ng = geom_.node(node);
		int incoming = 0;
		for (const LaneSpec &l : s->profile.lanes) {
			if (is_travel(l.type) && l.dir == (e == 1 ? LaneDir::Forward : LaneDir::Backward)) ++incoming;
		}
		Dictionary ed;
		ed["node"] = static_cast<int64_t>(node);
		ed["left"] = rule_name(r.left);
		ed["right"] = rule_name(r.right);
		ed["turn_lane_length"] = r.turn_lane_length;
		ed["incoming"] = incoming;
		ed["junction"] = ng && ng->kind == NodeKind::Junction && ng->legs.size() >= 3;
		ends.push_back(ed);
	}
	d["ends"] = ends;
	Array zones;
	for (const NoChangeZone &z : s->no_change) {
		Dictionary zd;
		zd["edge"] = z.edge;
		zd["from"] = z.u0;
		zd["to"] = z.u1;
		zd["block_l2r"] = z.block_left_to_right;
		zd["block_r2l"] = z.block_right_to_left;
		zones.push_back(zd);
	}
	d["no_change"] = zones;
	Array stops;
	for (const BusStop &st : s->stops) stops.push_back(stop_dict(st));
	d["stops"] = stops;
	return d;
}

Dictionary RoadEditor::pick(Vector2 pos, double radius, int level) {
	ensure_geometry();
	Dictionary d;
	const int64_t n = nearest_node(pos, radius, level, 0);
	if (n != 0) {
		d["type"] = "node";
		d["id"] = n;
		return d;
	}
	LaneHit hit;
	if (geom_.pick_segment(tv(pos), radius, level, hit)) {
		d["type"] = "segment";
		d["id"] = static_cast<int64_t>(hit.seg);
		d["u"] = hit.u;
		d["lane"] = static_cast<int64_t>(hit.lane);
		d["lane_index"] = hit.lane_index;
		d["offset"] = hit.offset;
		d["edge"] = hit.edge;
		d["edge_distance"] = hit.edge_distance;
		return d;
	}
	d["type"] = "none";
	return d;
}

int64_t RoadEditor::nearest_node(Vector2 pos, double radius, int level, int64_t exclude) {
	double best = radius;
	int64_t id = 0;
	for (const auto &kv : doc_.map().nodes()) {
		if (kv.second.level != level || static_cast<int64_t>(kv.first) == exclude) continue;
		const double d = (kv.second.pos - tv(pos)).length();
		if (d <= best) {
			best = d;
			id = kv.first;
		}
	}
	return id;
}

PackedVector2Array RoadEditor::segment_centerline(int64_t seg, double step) {
	const RoadSegment *s = doc_.map().segment(static_cast<SegmentId>(seg));
	if (!s) return PackedVector2Array();
	const Curve c = doc_.map().curve_of(*s);
	ArcTable t;
	t.build(c);
	const int n = s->curve == CurveKind::Straight ? 1 : std::clamp(static_cast<int>(t.length() / std::max(0.5, step)), 2, 400);
	std::vector<Vec2> pts;
	for (int i = 0; i <= n; ++i) pts.push_back(c.point(t.t_at(t.length() * i / n)));
	return packed(pts);
}

PackedVector2Array RoadEditor::segment_outline(int64_t seg) {
	ensure_geometry();
	const SegmentGeom *g = geom_.segment(static_cast<SegmentId>(seg));
	return g ? packed(g->outline()) : PackedVector2Array();
}

PackedVector2Array RoadEditor::node_outline(int64_t node) {
	ensure_geometry();
	const NodeGeom *g = geom_.node(static_cast<NodeId>(node));
	if (!g) return PackedVector2Array();
	if (!g->polygon.empty()) return packed(g->polygon);
	double r = 2.0;
	for (const Leg &l : g->legs) r = std::max({ r, l.kerb_left, l.kerb_right });
	std::vector<Vec2> circle;
	for (int i = 0; i < 24; ++i) {
		const double a = 2.0 * kPi * i / 24.0;
		circle.push_back(g->pos + Vec2{ std::cos(a), std::sin(a) } * r);
	}
	return packed(circle);
}

PackedVector2Array RoadEditor::lane_outline(int64_t seg, int64_t lane) {
	ensure_geometry();
	const SegmentGeom *g = geom_.segment(static_cast<SegmentId>(seg));
	if (!g) return PackedVector2Array();
	for (size_t i = 0; i < g->lanes.size(); ++i) {
		if (g->lanes[i].id != static_cast<LaneId>(lane)) continue;
		std::vector<Vec2> left, right;
		for (size_t k = 0; k < g->s.size(); ++k) {
			left.push_back(g->at(k, g->edge(k, i).first));
			right.push_back(g->at(k, g->edge(k, i).second));
		}
		std::reverse(right.begin(), right.end());
		left.insert(left.end(), right.begin(), right.end());
		return packed(left);
	}
	return PackedVector2Array();
}

PackedVector2Array RoadEditor::edge_polyline(int64_t seg, int edge, double u0, double u1) {
	ensure_geometry();
	const SegmentGeom *g = geom_.segment(static_cast<SegmentId>(seg));
	if (!g) return PackedVector2Array();
	size_t lane = g->lanes.size();
	for (size_t i = 0; i < g->lanes.size(); ++i) {
		if (g->lanes[i].profile_index == edge) lane = i;
	}
	if (lane == g->lanes.size()) return PackedVector2Array();
	if (u0 > u1) std::swap(u0, u1);
	std::vector<Vec2> pts;
	for (size_t k = 0; k < g->s.size(); ++k) {
		const double u = g->length > 0 ? g->s[k] / g->length : 0.0;
		if (u < u0 || u > u1) continue;
		pts.push_back(g->at(k, g->edge(k, lane).first));
	}
	return packed(pts);
}

double RoadEditor::segment_u_at(int64_t seg, Vector2 pos) {
	const RoadSegment *s = doc_.map().segment(static_cast<SegmentId>(seg));
	if (!s) return 0.0;
	const Curve c = doc_.map().curve_of(*s);
	ArcTable t;
	t.build(c);
	const double param = closest_t(c, tv(pos));
	return t.length() > 0 ? t.s_at(param) / t.length() : 0.0;
}

Vector2 RoadEditor::segment_point(int64_t seg, double u) {
	const RoadSegment *s = doc_.map().segment(static_cast<SegmentId>(seg));
	if (!s) return Vector2();
	const Curve c = doc_.map().curve_of(*s);
	ArcTable t;
	t.build(c);
	return gv(c.point(t.t_at(std::clamp(u, 0.0, 1.0) * t.length())));
}

Vector2 RoadEditor::segment_tangent(int64_t seg, double u) {
	const RoadSegment *s = doc_.map().segment(static_cast<SegmentId>(seg));
	if (!s) return Vector2(1, 0);
	const Curve c = doc_.map().curve_of(*s);
	ArcTable t;
	t.build(c);
	return gv(c.tangent(t.t_at(std::clamp(u, 0.0, 1.0) * t.length())));
}

PackedInt64Array RoadEditor::node_ids() const {
	PackedInt64Array out;
	for (const auto &kv : doc_.map().nodes()) out.push_back(kv.first);
	return out;
}

PackedInt64Array RoadEditor::segment_ids() const {
	PackedInt64Array out;
	for (const auto &kv : doc_.map().segments()) out.push_back(kv.first);
	return out;
}

// --- Geometry output ------------------------------------------------------------------------

Array RoadEditor::get_meshes() {
	ensure_geometry();
	Array out;
	for (const MeshBatch &b : geom_.meshes()) {
		PackedVector2Array verts;
		PackedVector2Array uvs;
		PackedColorArray colors;
		PackedInt32Array indices;
		verts.resize(static_cast<int64_t>(b.vertices.size()));
		uvs.resize(static_cast<int64_t>(b.uvs.size()));
		Vector2 *uw = uvs.ptrw();
		for (size_t i = 0; i < b.uvs.size(); ++i) uw[i] = gv(b.uvs[i]);
		colors.resize(static_cast<int64_t>(b.colors.size()));
		indices.resize(static_cast<int64_t>(b.indices.size()));
		Vector2 *vw = verts.ptrw();
		Color *cw = colors.ptrw();
		int32_t *iw = indices.ptrw();
		for (size_t i = 0; i < b.vertices.size(); ++i) vw[i] = gv(b.vertices[i]);
		for (size_t i = 0; i < b.colors.size(); ++i) cw[i] = Color(b.colors[i].r, b.colors[i].g, b.colors[i].b, b.colors[i].a);
		for (size_t i = 0; i < b.indices.size(); ++i) iw[i] = b.indices[i];
		Dictionary d;
		d["level"] = b.level;
		d["layer"] = static_cast<int>(b.layer);
		d["vertices"] = verts;
		d["colors"] = colors;
		d["uvs"] = uvs;
		d["indices"] = indices;
		out.push_back(d);
	}
	return out;
}

Array RoadEditor::get_connectors() {
	ensure_geometry();
	Array out;
	for (const auto &kv : geom_.nodes()) {
		for (const Connector &c : kv.second.connectors) {
			Dictionary d;
			d["path"] = packed(c.path);
			d["turn"] = turn_kind_name(c.turn);
			d["node"] = static_cast<int64_t>(kv.first);
			d["level"] = kv.second.level;
			out.push_back(d);
		}
	}
	return out;
}

void RoadEditor::ensure_network() {
	ensure_geometry();
	if (checked_revision_ == doc_.revision()) return;
	check_compiler_.compile(doc_.map(), geom_, check_net_);
	net_problems_ = network_problems(doc_.map(), check_net_);
	checked_revision_ = doc_.revision();
}

Array RoadEditor::get_spawners() {
	ensure_network();
	Array out;
	for (const auto &kv : doc_.map().nodes()) {
		const RoadNode &n = kv.second;
		if (!n.spawner.enabled) continue;
		Dictionary d;
		d["id"] = static_cast<int64_t>(n.id);
		d["pos"] = gv(n.pos);
		d["level"] = n.level;
		d["rate"] = n.spawner.rate;
		d["sink"] = n.spawner.sink;
		d["active"] = check_net_.spawner_at(n.id) != nullptr;
		// Direction pointing into the map, for drawing.
		const NodeGeom *g = geom_.node(n.id);
		d["dir"] = g && !g->legs.empty() ? gv(g->legs[0].dir) : Vector2(1, 0);
		out.push_back(d);
	}
	return out;
}

Array RoadEditor::get_problems() {
	ensure_network();
	Array out;
	Array net;
	for (const NetProblem &p : net_problems_) {
		Dictionary d;
		d["severity"] = "warning";
		d["code"] = gs(p.code);
		d["message"] = gs(p.message);
		d["pos"] = gv(p.pos);
		d["level"] = p.level;
		d["segments"] = PackedInt64Array();
		PackedInt64Array nodes;
		nodes.push_back(p.node);
		d["nodes"] = nodes;
		net.push_back(d);
	}
	for (const Problem &p : problems_) {
		Dictionary d;
		d["severity"] = p.severity == Severity::Error ? "error" : "warning";
		d["code"] = gs(p.code);
		d["message"] = gs(p.message);
		d["pos"] = gv(p.pos);
		d["level"] = p.level;
		PackedInt64Array segs, nodes;
		for (SegmentId s : p.segments) segs.push_back(s);
		for (NodeId n : p.nodes) nodes.push_back(n);
		d["segments"] = segs;
		d["nodes"] = nodes;
		out.push_back(d);
	}
	// Errors first (problems_ is sorted that way), then network warnings.
	Array sorted;
	for (int64_t i = 0; i < out.size(); ++i) {
		if (String(Dictionary(out[i])["severity"]) == "error") sorted.push_back(out[i]);
	}
	for (int64_t i = 0; i < out.size(); ++i) {
		if (String(Dictionary(out[i])["severity"]) != "error") sorted.push_back(out[i]);
	}
	for (int64_t i = 0; i < net.size(); ++i) sorted.push_back(net[i]);
	return sorted;
}

Dictionary RoadEditor::get_stats() {
	ensure_network();
	int junctions = 0;
	for (const auto &kv : geom_.nodes()) junctions += kv.second.kind == NodeKind::Junction && kv.second.legs.size() >= 3;
	int64_t lanes = 0, verts = 0;
	for (const auto &kv : doc_.map().segments()) lanes += static_cast<int64_t>(kv.second.profile.lanes.size());
	for (const MeshBatch &b : geom_.meshes()) verts += static_cast<int64_t>(b.vertices.size());
	int errors = 0, warnings = 0;
	for (const Problem &p : problems_) (p.severity == Severity::Error ? errors : warnings)++;
	warnings += static_cast<int>(net_problems_.size());
	Dictionary d;
	d["nodes"] = static_cast<int64_t>(doc_.map().nodes().size());
	d["segments"] = static_cast<int64_t>(doc_.map().segments().size());
	d["junctions"] = junctions;
	d["lanes"] = lanes;
	d["vertices"] = verts;
	d["build_ms"] = build_ms_;
	d["validate_ms"] = validate_ms_;
	d["errors"] = errors;
	d["warnings"] = warnings;
	d["revision"] = static_cast<int64_t>(doc_.revision());
	return d;
}

void RoadEditor::_bind_methods() {
	ClassDB::bind_method(D_METHOD("new_map"), &RoadEditor::new_map);
	ClassDB::bind_method(D_METHOD("load_demo_town"), &RoadEditor::load_demo_town);
	ClassDB::bind_method(D_METHOD("load_test_grid", "cols", "rows", "spacing"), &RoadEditor::load_test_grid);
	ClassDB::bind_method(D_METHOD("load_example", "name"), &RoadEditor::load_example);
	ClassDB::bind_method(D_METHOD("save_json"), &RoadEditor::save_json);
	ClassDB::bind_method(D_METHOD("load_json", "text"), &RoadEditor::load_json);
	ClassDB::bind_method(D_METHOD("revision"), &RoadEditor::revision);

	ClassDB::bind_method(D_METHOD("begin", "label"), &RoadEditor::begin);
	ClassDB::bind_method(D_METHOD("commit"), &RoadEditor::commit);
	ClassDB::bind_method(D_METHOD("cancel"), &RoadEditor::cancel);
	ClassDB::bind_method(D_METHOD("undo"), &RoadEditor::undo);
	ClassDB::bind_method(D_METHOD("redo"), &RoadEditor::redo);
	ClassDB::bind_method(D_METHOD("can_undo"), &RoadEditor::can_undo);
	ClassDB::bind_method(D_METHOD("can_redo"), &RoadEditor::can_redo);
	ClassDB::bind_method(D_METHOD("undo_label"), &RoadEditor::undo_label);
	ClassDB::bind_method(D_METHOD("redo_label"), &RoadEditor::redo_label);
	ClassDB::bind_method(D_METHOD("history_size"), &RoadEditor::history_size);
	ClassDB::bind_method(D_METHOD("redo_size"), &RoadEditor::redo_size);

	ClassDB::bind_method(D_METHOD("add_road", "points", "road", "level", "speed_kmh"), &RoadEditor::add_road);
	ClassDB::bind_method(D_METHOD("add_curve", "a", "control", "b", "road", "level", "speed_kmh"), &RoadEditor::add_curve);
	ClassDB::bind_method(D_METHOD("move_node", "id", "pos"), &RoadEditor::move_node);
	ClassDB::bind_method(D_METHOD("merge_nodes", "from", "into"), &RoadEditor::merge_nodes);
	ClassDB::bind_method(D_METHOD("set_control_point", "segment", "which", "pos"), &RoadEditor::set_control_point);
	ClassDB::bind_method(D_METHOD("split_segment", "segment", "u"), &RoadEditor::split_segment);
	ClassDB::bind_method(D_METHOD("delete_segment", "id"), &RoadEditor::delete_segment);
	ClassDB::bind_method(D_METHOD("delete_node", "id"), &RoadEditor::delete_node);
	ClassDB::bind_method(D_METHOD("set_profile_params", "segment", "params"), &RoadEditor::set_profile_params);
	ClassDB::bind_method(D_METHOD("set_profile", "segment", "profile"), &RoadEditor::set_profile);
	ClassDB::bind_method(D_METHOD("set_lane_type", "segment", "lane", "type"), &RoadEditor::set_lane_type);
	ClassDB::bind_method(D_METHOD("flip", "segment"), &RoadEditor::flip);
	ClassDB::bind_method(D_METHOD("set_end_rules", "segment", "end", "rules"), &RoadEditor::set_end_rules);
	ClassDB::bind_method(D_METHOD("set_speed_kmh", "segment", "kmh"), &RoadEditor::set_speed_kmh);
	ClassDB::bind_method(D_METHOD("set_segment_name", "segment", "name"), &RoadEditor::set_segment_name);
	ClassDB::bind_method(D_METHOD("set_level", "segment", "level"), &RoadEditor::set_level);
	ClassDB::bind_method(D_METHOD("set_no_change", "segment", "edge", "u0", "u1", "block_l2r", "block_r2l"),
			&RoadEditor::set_no_change);

	ClassDB::bind_method(D_METHOD("set_junction_control", "node", "control", "priority"), &RoadEditor::set_junction_control);
	ClassDB::bind_method(D_METHOD("set_spawner", "node", "spawner"), &RoadEditor::set_spawner);
	ClassDB::bind_method(D_METHOD("set_roundabout", "node", "roundabout"), &RoadEditor::set_roundabout);
	ClassDB::bind_method(D_METHOD("default_signal_plan", "node"), &RoadEditor::default_signal_plan);
	ClassDB::bind_method(D_METHOD("set_signal_plan", "node", "plan"), &RoadEditor::set_signal_plan);
	ClassDB::bind_method(D_METHOD("add_stop", "segment", "u", "side", "kind", "name"), &RoadEditor::add_stop);
	ClassDB::bind_method(D_METHOD("set_stop", "segment", "stop"), &RoadEditor::set_stop);
	ClassDB::bind_method(D_METHOD("remove_stop", "segment", "stop"), &RoadEditor::remove_stop);
	ClassDB::bind_method(D_METHOD("set_depot", "node", "depot"), &RoadEditor::set_depot);
	ClassDB::bind_method(D_METHOD("get_stops"), &RoadEditor::get_stops);
	ClassDB::bind_method(D_METHOD("get_depots"), &RoadEditor::get_depots);
	ClassDB::bind_method(D_METHOD("get_bays", "level"), &RoadEditor::get_bays);
	ClassDB::bind_method(D_METHOD("sim_signal_heads", "level"), &RoadEditor::sim_signal_heads);
	ClassDB::bind_method(D_METHOD("sim_signal_state", "node"), &RoadEditor::sim_signal_state);
	ClassDB::bind_method(D_METHOD("sim_route_stats"), &RoadEditor::sim_route_stats);

	ClassDB::bind_method(D_METHOD("presets"), &RoadEditor::presets);
	ClassDB::bind_method(D_METHOD("params_of_profile", "profile"), &RoadEditor::params_of_profile);
	ClassDB::bind_method(D_METHOD("profile_from_params", "params"), &RoadEditor::profile_from_params);
	ClassDB::bind_method(D_METHOD("validate_profile", "profile"), &RoadEditor::validate_profile);

	ClassDB::bind_method(D_METHOD("get_node", "id"), &RoadEditor::get_node);
	ClassDB::bind_method(D_METHOD("get_segment", "id"), &RoadEditor::get_segment);
	ClassDB::bind_method(D_METHOD("pick", "pos", "radius", "level"), &RoadEditor::pick);
	ClassDB::bind_method(D_METHOD("nearest_node", "pos", "radius", "level", "exclude"), &RoadEditor::nearest_node);
	ClassDB::bind_method(D_METHOD("segment_centerline", "segment", "step"), &RoadEditor::segment_centerline);
	ClassDB::bind_method(D_METHOD("segment_outline", "segment"), &RoadEditor::segment_outline);
	ClassDB::bind_method(D_METHOD("node_outline", "node"), &RoadEditor::node_outline);
	ClassDB::bind_method(D_METHOD("lane_outline", "segment", "lane"), &RoadEditor::lane_outline);
	ClassDB::bind_method(D_METHOD("edge_polyline", "segment", "edge", "u0", "u1"), &RoadEditor::edge_polyline);
	ClassDB::bind_method(D_METHOD("segment_u_at", "segment", "pos"), &RoadEditor::segment_u_at);
	ClassDB::bind_method(D_METHOD("segment_point", "segment", "u"), &RoadEditor::segment_point);
	ClassDB::bind_method(D_METHOD("segment_tangent", "segment", "u"), &RoadEditor::segment_tangent);
	ClassDB::bind_method(D_METHOD("node_ids"), &RoadEditor::node_ids);
	ClassDB::bind_method(D_METHOD("segment_ids"), &RoadEditor::segment_ids);

	ClassDB::bind_method(D_METHOD("get_meshes"), &RoadEditor::get_meshes);
	ClassDB::bind_method(D_METHOD("get_connectors"), &RoadEditor::get_connectors);
	ClassDB::bind_method(D_METHOD("get_problems"), &RoadEditor::get_problems);
	ClassDB::bind_method(D_METHOD("get_stats"), &RoadEditor::get_stats);
	ClassDB::bind_method(D_METHOD("get_spawners"), &RoadEditor::get_spawners);

	ClassDB::bind_method(D_METHOD("sim_reset", "seed"), &RoadEditor::sim_reset);
	ClassDB::bind_method(D_METHOD("sim_advance", "real_delta", "speed", "budget_ms"), &RoadEditor::sim_advance);
	ClassDB::bind_method(D_METHOD("sim_step", "ticks"), &RoadEditor::sim_step);
	ClassDB::bind_method(D_METHOD("sim_set_demand", "multiplier"), &RoadEditor::sim_set_demand);
	ClassDB::bind_method(D_METHOD("sim_set_max_vehicles", "cap"), &RoadEditor::sim_set_max_vehicles);
	ClassDB::bind_method(D_METHOD("sim_car_buffer", "level", "car_scale"), &RoadEditor::sim_car_buffer);
	ClassDB::bind_method(D_METHOD("sim_car_count", "level"), &RoadEditor::sim_car_count);
	ClassDB::bind_method(D_METHOD("sim_pick_car", "pos", "radius", "level"), &RoadEditor::sim_pick_car);
	ClassDB::bind_method(D_METHOD("sim_car_info", "id"), &RoadEditor::sim_car_info);
	ClassDB::bind_method(D_METHOD("sim_stats"), &RoadEditor::sim_stats);
	ClassDB::bind_method(D_METHOD("sim_state_hash"), &RoadEditor::sim_state_hash);
	ClassDB::bind_method(D_METHOD("sim_golden_check"), &RoadEditor::sim_golden_check);
}

} // namespace godot
