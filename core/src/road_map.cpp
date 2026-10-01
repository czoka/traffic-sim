#include "tsim/road_map.h"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace tsim {

// --- Lane types --------------------------------------------------------------

const char *junction_control_name(JunctionControl c) {
	switch (c) {
		case JunctionControl::RightHand:
			return "right_hand";
		case JunctionControl::PriorityRoad:
			return "priority_road";
		case JunctionControl::AllWayStop:
			return "all_way_stop";
		case JunctionControl::Signal:
			return "signal";
	}
	return "right_hand";
}

bool junction_control_from_name(const std::string &s, JunctionControl &out) {
	if (s == "right_hand") out = JunctionControl::RightHand;
	else if (s == "priority_road") out = JunctionControl::PriorityRoad;
	else if (s == "all_way_stop") out = JunctionControl::AllWayStop;
	else if (s == "signal") out = JunctionControl::Signal;
	else return false;
	return true;
}

const char *segment_kind_name(SegmentKind k) {
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

bool segment_kind_from_name(const std::string &s, SegmentKind &out) {
	if (s == "road") out = SegmentKind::Road;
	else if (s == "footpath") out = SegmentKind::Footpath;
	else if (s == "bike_path") out = SegmentKind::BikePath;
	else if (s == "shared_path") out = SegmentKind::SharedPath;
	else return false;
	return true;
}

const char *crossing_kind_name(CrossingKind k) {
	switch (k) {
		case CrossingKind::None:
			return "none";
		case CrossingKind::Zebra:
			return "zebra";
		case CrossingKind::Signal:
			return "signal";
		case CrossingKind::Uncontrolled:
			return "uncontrolled";
	}
	return "none";
}

bool crossing_kind_from_name(const std::string &s, CrossingKind &out) {
	if (s == "none") out = CrossingKind::None;
	else if (s == "zebra") out = CrossingKind::Zebra;
	else if (s == "signal") out = CrossingKind::Signal;
	else if (s == "uncontrolled") out = CrossingKind::Uncontrolled;
	else return false;
	return true;
}

double RoadSegment::max_grade() const {
	if (kind == SegmentKind::Footpath) return stairs ? kMaxGradeStairs : kMaxGradePath;
	return kMaxGradeRoad;
}

double Spawner::weight_to(NodeId to) const {
	for (const OdWeight &w : od) {
		if (w.to == to) return w.weight;
	}
	return 1.0;
}

bool RoadNode::is_priority(SegmentId s) const {
	return std::find(priority.begin(), priority.end(), s) != priority.end();
}

void RoadNode::rename_segment(SegmentId from, SegmentId to) {
	auto fix = [&](std::vector<SegmentId> &v) {
		for (SegmentId &x : v) {
			if (x == from) x = to;
		}
		v.erase(std::remove(v.begin(), v.end(), kNoId), v.end());
	};
	fix(priority);
	fix(roundabout.slip);
	fix(signal.right_on_red);
	for (SignalPhase &p : signal.phases) {
		for (SignalMovement &m : p.moves) {
			if (m.from == from) m.from = to;
			if (m.to == from) m.to = to;
		}
		p.moves.erase(std::remove_if(p.moves.begin(), p.moves.end(),
							  [](const SignalMovement &m) { return m.from == kNoId || m.to == kNoId; }),
				p.moves.end());
		fix(p.walk);
	}
}

double SignalPlan::cycle() const {
	double c = 0.0;
	for (const SignalPhase &p : phases) c += p.green + amber + all_red;
	return c;
}

const char *parking_style_name(ParkingStyle s) {
	switch (s) {
		case ParkingStyle::Parallel:
			return "parallel";
		case ParkingStyle::Angle45:
			return "angle45";
		case ParkingStyle::Perpendicular:
			return "perpendicular";
	}
	return "parallel";
}

bool parking_style_from_name(const std::string &s, ParkingStyle &out) {
	if (s == "parallel") out = ParkingStyle::Parallel;
	else if (s == "angle45") out = ParkingStyle::Angle45;
	else if (s == "perpendicular") out = ParkingStyle::Perpendicular;
	else return false;
	return true;
}

double parking_depth(ParkingStyle s) {
	switch (s) {
		case ParkingStyle::Parallel:
			return 2.0;
		case ParkingStyle::Angle45:
			return 5.3;
		case ParkingStyle::Perpendicular:
			return 5.0;
	}
	return 2.0;
}

double parking_bay_length(ParkingStyle s) {
	switch (s) {
		case ParkingStyle::Parallel:
			return 6.0;
		case ParkingStyle::Angle45:
			return 3.5; // 2.5 m wide bays at 45 degrees
		case ParkingStyle::Perpendicular:
			return 2.5;
	}
	return 6.0;
}

const char *stop_kind_name(StopKind k) {
	switch (k) {
		case StopKind::Kerbside:
			return "kerbside";
		case StopKind::Bay:
			return "bay";
		case StopKind::MainStation:
			return "main_station";
	}
	return "kerbside";
}

bool stop_kind_from_name(const std::string &s, StopKind &out) {
	if (s == "kerbside") out = StopKind::Kerbside;
	else if (s == "bay") out = StopKind::Bay;
	else if (s == "main_station") out = StopKind::MainStation;
	else return false;
	return true;
}

bool is_travel(LaneType t) { return t == LaneType::General || t == LaneType::Bus || t == LaneType::Turn; }

bool is_directional(LaneType t) { return is_travel(t) || t == LaneType::Bike; }

double default_width(LaneType t) {
	switch (t) {
		case LaneType::General:
			return 3.25;
		case LaneType::Bus:
			return 3.5;
		case LaneType::Bike:
			return 1.5;
		case LaneType::Turn:
			return 3.0;
		case LaneType::Parking:
			return 2.5;
		case LaneType::Sidewalk:
			return 2.0;
	}
	return 3.25;
}

const char *lane_type_name(LaneType t) {
	switch (t) {
		case LaneType::General:
			return "general";
		case LaneType::Bus:
			return "bus";
		case LaneType::Bike:
			return "bike";
		case LaneType::Turn:
			return "turn";
		case LaneType::Parking:
			return "parking";
		case LaneType::Sidewalk:
			return "sidewalk";
	}
	return "general";
}

bool lane_type_from_name(const std::string &s, LaneType &out) {
	static const LaneType all[] = { LaneType::General, LaneType::Bus, LaneType::Bike, LaneType::Turn,
		LaneType::Parking, LaneType::Sidewalk };
	for (LaneType t : all) {
		if (s == lane_type_name(t)) {
			out = t;
			return true;
		}
	}
	return false;
}

int Profile::count(LaneDir dir) const {
	int n = 0;
	for (const LaneSpec &l : lanes) {
		if (is_travel(l.type) && l.dir == dir) {
			++n;
		}
	}
	return n;
}

double Profile::total_width() const {
	double w = median == MedianType::None ? 0.0 : median_width;
	for (const LaneSpec &l : lanes) {
		w += l.width;
	}
	return w;
}

bool RoadSegment::operator==(const RoadSegment &o) const {
	return id == o.id && from == o.from && to == o.to && kind == o.kind && level == o.level && curve == o.curve &&
			c1 == o.c1 && c2 == o.c2 && sweep == o.sweep && profile == o.profile && speed_limit == o.speed_limit &&
			name == o.name && ends[0] == o.ends[0] && ends[1] == o.ends[1] && no_change == o.no_change &&
			stops == o.stops && rise == o.rise && stairs == o.stairs && crossings == o.crossings && fences == o.fences;
}

// --- RoadMap -----------------------------------------------------------------

const RoadNode *RoadMap::node(NodeId id) const {
	auto it = nodes_.find(id);
	return it == nodes_.end() ? nullptr : &it->second;
}

const RoadSegment *RoadMap::segment(SegmentId id) const {
	auto it = segments_.find(id);
	return it == segments_.end() ? nullptr : &it->second;
}

std::vector<SegmentId> RoadMap::segments_at(NodeId id) const {
	auto it = adjacency_.find(id);
	return it == adjacency_.end() ? std::vector<SegmentId>{} : it->second;
}

Curve RoadMap::curve_of(const RoadSegment &s) const {
	Curve c;
	c.kind = s.curve;
	const RoadNode *a = node(s.from);
	const RoadNode *b = node(s.to);
	c.p0 = a ? a->pos : Vec2{};
	c.p3 = b ? b->pos : Vec2{};
	c.c1 = s.c1;
	c.c2 = s.c2;
	c.sweep = s.sweep;
	if (s.curve == CurveKind::Arc) {
		c.center = arc_center_for(c.p0, c.p3, s.sweep);
	}
	return c;
}

const Building *RoadMap::building(uint32_t id) const {
	auto it = buildings_.find(id);
	return it == buildings_.end() ? nullptr : &it->second;
}

void RoadMap::put_building(const Building &b) {
	buildings_[b.id] = b;
	next_object_id_ = std::max(next_object_id_, b.id + 1);
}

void RoadMap::erase_building(uint32_t id) { buildings_.erase(id); }

void RoadMap::set_next_ids(uint32_t n, uint32_t s, uint32_t l, uint32_t o) {
	next_node_id_ = std::max(next_node_id_, n);
	next_segment_id_ = std::max(next_segment_id_, s);
	next_lane_id_ = std::max(next_lane_id_, l);
	next_object_id_ = std::max(next_object_id_, o);
}

void RoadMap::index_add(const RoadSegment &s) {
	for (NodeId n : { s.from, s.to }) {
		std::vector<SegmentId> &v = adjacency_[n];
		auto it = std::lower_bound(v.begin(), v.end(), s.id);
		if (it == v.end() || *it != s.id) {
			v.insert(it, s.id);
		}
	}
}

void RoadMap::index_remove(const RoadSegment &s) {
	for (NodeId n : { s.from, s.to }) {
		auto a = adjacency_.find(n);
		if (a == adjacency_.end()) {
			continue;
		}
		std::vector<SegmentId> &v = a->second;
		v.erase(std::remove(v.begin(), v.end(), s.id), v.end());
		if (v.empty()) {
			adjacency_.erase(a);
		}
	}
}

void RoadMap::put_node(const RoadNode &n) {
	nodes_[n.id] = n;
	next_node_id_ = std::max(next_node_id_, n.id + 1);
	for (const BusRoute &r : n.depot.routes) next_object_id_ = std::max(next_object_id_, r.id + 1);
	for (const CoachLine &c : n.spawner.coaches) next_object_id_ = std::max(next_object_id_, c.id + 1);
}

void RoadMap::put_segment(const RoadSegment &s) {
	auto it = segments_.find(s.id);
	if (it != segments_.end()) {
		index_remove(it->second);
	}
	segments_[s.id] = s;
	index_add(s);
	next_segment_id_ = std::max(next_segment_id_, s.id + 1);
	for (const LaneSpec &l : s.profile.lanes) {
		next_lane_id_ = std::max(next_lane_id_, l.id + 1);
	}
	for (const EndRules &e : s.ends) {
		next_lane_id_ = std::max({ next_lane_id_, e.left_lane + 1, e.right_lane + 1 });
	}
	for (const BusStop &b : s.stops) next_object_id_ = std::max(next_object_id_, b.id + 1);
	for (const Crossing &c : s.crossings) next_object_id_ = std::max(next_object_id_, c.id + 1);
}

void RoadMap::erase_node(NodeId id) { nodes_.erase(id); }

void RoadMap::erase_segment(SegmentId id) {
	auto it = segments_.find(id);
	if (it == segments_.end()) {
		return;
	}
	index_remove(it->second);
	segments_.erase(it);
}

void RoadMap::clear() {
	nodes_.clear();
	segments_.clear();
	buildings_.clear();
	adjacency_.clear();
	next_node_id_ = next_segment_id_ = next_lane_id_ = next_object_id_ = 1;
}

// --- Profiles ------------------------------------------------------------------

namespace {

enum RoleKind { kSidewalk = 0, kParking = 1, kBike = 2, kMotor = 3 };
using Role = std::tuple<int, int, int>; // kind, side (0 = left, 1 = right), index from median

// Index of the first lane right of the median (the reference line of a two-way road).
size_t median_index(const Profile &p) {
	for (size_t i = 0; i < p.lanes.size(); ++i) {
		if (is_directional(p.lanes[i].type) && p.lanes[i].dir == LaneDir::Forward) {
			return i;
		}
	}
	// No forward lanes: the median is right of the last backward lane.
	for (size_t i = p.lanes.size(); i-- > 0;) {
		if (is_directional(p.lanes[i].type)) {
			return i + 1;
		}
	}
	return p.lanes.size() / 2;
}

std::vector<Role> roles_of(const Profile &p) {
	std::vector<Role> roles(p.lanes.size());
	const size_t m = median_index(p);
	int back_idx = 0;
	for (size_t i = m; i-- > 0;) {
		const LaneSpec &l = p.lanes[i];
		if (is_travel(l.type)) {
			roles[i] = Role{ kMotor, 0, back_idx++ };
		}
	}
	int fwd_idx = 0;
	for (size_t i = m; i < p.lanes.size(); ++i) {
		const LaneSpec &l = p.lanes[i];
		if (is_travel(l.type)) {
			roles[i] = Role{ kMotor, 1, fwd_idx++ };
		}
	}
	for (size_t i = 0; i < p.lanes.size(); ++i) {
		const LaneSpec &l = p.lanes[i];
		const int side = i < m ? 0 : 1;
		if (l.type == LaneType::Sidewalk) {
			roles[i] = Role{ kSidewalk, side, 0 };
		} else if (l.type == LaneType::Parking) {
			roles[i] = Role{ kParking, side, 0 };
		} else if (l.type == LaneType::Bike) {
			roles[i] = Role{ kBike, side, 0 };
		}
	}
	return roles;
}

} // namespace

std::vector<LaneRole> profile_roles(const Profile &p) {
	std::vector<LaneRole> out;
	for (const Role &r : roles_of(p)) {
		out.push_back(LaneRole{ std::get<0>(r), std::get<1>(r), std::get<2>(r) });
	}
	return out;
}

Profile path_profile(SegmentKind kind, RoadMap &map) {
	Profile p;
	auto lane = [&](LaneType t, LaneDir d, double w) {
		LaneSpec l;
		l.id = map.alloc_lane_id();
		l.type = t;
		l.dir = d;
		l.width = w;
		p.lanes.push_back(l);
	};
	switch (kind) {
		case SegmentKind::Road:
		case SegmentKind::Footpath:
			lane(LaneType::Sidewalk, LaneDir::None, 3.0);
			break;
		case SegmentKind::BikePath:
			lane(LaneType::Bike, LaneDir::Backward, 1.25);
			lane(LaneType::Bike, LaneDir::Forward, 1.25);
			break;
		case SegmentKind::SharedPath:
			lane(LaneType::Bike, LaneDir::Backward, 2.0);
			lane(LaneType::Bike, LaneDir::Forward, 2.0);
			break;
	}
	return p;
}

Profile build_profile(const ProfileParams &p, const Profile *previous, RoadMap &map) {
	std::map<Role, const LaneSpec *> old;
	if (previous) {
		const std::vector<Role> roles = roles_of(*previous);
		for (size_t i = 0; i < roles.size(); ++i) {
			old.emplace(roles[i], &previous->lanes[i]);
		}
	}
	const int backward = std::clamp(p.backward, 0, 4);
	const int forward = std::clamp(p.forward, 0, 6);
	const LaneDir left_dir = backward > 0 ? LaneDir::Backward : LaneDir::Forward;
	const LaneDir right_dir = forward > 0 ? LaneDir::Forward : LaneDir::Backward;

	Profile out;
	out.median = (backward > 0 && forward > 0) ? p.median : MedianType::None;
	out.median_width = out.median == MedianType::None ? 0.0 : std::max(0.5, p.median_width);

	auto add = [&](Role role, LaneType type, LaneDir dir, double width) {
		LaneSpec l;
		l.type = type;
		l.dir = dir;
		l.width = width;
		if (type == LaneType::Parking) l.parking = p.parking_style;
		auto it = old.find(role);
		if (it != old.end()) {
			l.id = it->second->id;
			// Keep a Turn lane marked by the player; bus is driven by the params.
			if (type == LaneType::General && it->second->type == LaneType::Turn) {
				l.type = LaneType::Turn;
			}
		} else {
			l.id = map.alloc_lane_id();
		}
		out.lanes.push_back(l);
	};

	if (p.sidewalk_left) add(Role{ kSidewalk, 0, 0 }, LaneType::Sidewalk, LaneDir::None, p.sidewalk_width);
	const double park_w = parking_depth(p.parking_style);
	if (p.parking_left) add(Role{ kParking, 0, 0 }, LaneType::Parking, LaneDir::None, park_w);
	if (p.bike_left) add(Role{ kBike, 0, 0 }, LaneType::Bike, left_dir, 1.5);
	for (int k = backward - 1; k >= 0; --k) {
		const bool bus = p.bus_left && k == backward - 1;
		add(Role{ kMotor, 0, k }, bus ? LaneType::Bus : LaneType::General, LaneDir::Backward,
				bus ? std::max(3.5, p.lane_width) : p.lane_width);
	}
	for (int k = 0; k < forward; ++k) {
		const bool bus = p.bus_right && k == forward - 1;
		add(Role{ kMotor, 1, k }, bus ? LaneType::Bus : LaneType::General, LaneDir::Forward,
				bus ? std::max(3.5, p.lane_width) : p.lane_width);
	}
	if (p.bike_right) add(Role{ kBike, 1, 0 }, LaneType::Bike, right_dir, 1.5);
	if (p.parking_right) add(Role{ kParking, 1, 0 }, LaneType::Parking, LaneDir::None, park_w);
	if (p.sidewalk_right) add(Role{ kSidewalk, 1, 0 }, LaneType::Sidewalk, LaneDir::None, p.sidewalk_width);
	return out;
}

ProfileParams params_of(const Profile &p) {
	ProfileParams r;
	r.backward = p.count(LaneDir::Backward);
	r.forward = p.count(LaneDir::Forward);
	r.median = p.median;
	r.median_width = p.median_width;
	const std::vector<Role> roles = roles_of(p);
	bool width_set = false;
	for (size_t i = 0; i < p.lanes.size(); ++i) {
		const LaneSpec &l = p.lanes[i];
		const int kind = std::get<0>(roles[i]);
		const int side = std::get<1>(roles[i]);
		if (l.type == LaneType::Sidewalk) {
			(side == 0 ? r.sidewalk_left : r.sidewalk_right) = true;
			r.sidewalk_width = l.width;
		} else if (l.type == LaneType::Parking) {
			(side == 0 ? r.parking_left : r.parking_right) = true;
			r.parking_style = l.parking;
		} else if (l.type == LaneType::Bike) {
			(side == 0 ? r.bike_left : r.bike_right) = true;
		} else if (kind == kMotor && l.type == LaneType::General && !width_set) {
			r.lane_width = l.width;
			width_set = true;
		}
	}
	// Bus flags: the outermost travel lane of each direction.
	for (size_t i = 0; i < p.lanes.size(); ++i) {
		if (is_travel(p.lanes[i].type) && p.lanes[i].dir == LaneDir::Backward) {
			r.bus_left = p.lanes[i].type == LaneType::Bus;
			break;
		}
	}
	for (size_t i = p.lanes.size(); i-- > 0;) {
		if (is_travel(p.lanes[i].type) && p.lanes[i].dir == LaneDir::Forward) {
			r.bus_right = p.lanes[i].type == LaneType::Bus;
			break;
		}
	}
	return r;
}

std::string validate_profile(const Profile &p) {
	if (p.lanes.empty()) {
		return "a road needs at least one lane";
	}
	if (p.lanes.size() > 20) {
		return "too many lanes (max 20)";
	}
	bool seen_forward = false;
	for (size_t i = 0; i < p.lanes.size(); ++i) {
		const LaneSpec &l = p.lanes[i];
		if (!(l.width >= 0.5 && l.width <= 10.0)) {
			return "lane widths must be between 0.5 and 10 m";
		}
		if (is_directional(l.type)) {
			if (l.dir == LaneDir::None) {
				return std::string(lane_type_name(l.type)) + " lanes need a direction";
			}
			if (l.dir == LaneDir::Forward) {
				seen_forward = true;
			} else if (seen_forward) {
				return "backward lanes must be left of forward lanes";
			}
		} else if (l.dir != LaneDir::None) {
			return std::string(lane_type_name(l.type)) + " lanes have no direction";
		}
		if (l.type == LaneType::Sidewalk && i != 0 && i + 1 != p.lanes.size() &&
				p.lanes[i - 1].type != LaneType::Sidewalk && p.lanes[i + 1].type != LaneType::Sidewalk) {
			return "sidewalks must be on the outside of the road";
		}
	}
	if (p.median != MedianType::None && !(p.median_width >= 0.5 && p.median_width <= 20.0)) {
		return "median width must be between 0.5 and 20 m";
	}
	return std::string();
}

const std::vector<ProfilePreset> &profile_presets() {
	static const std::vector<ProfilePreset> presets = [] {
		std::vector<ProfilePreset> v;
		ProfileParams p;
		p.backward = 1;
		p.forward = 1;
		p.sidewalk_left = p.sidewalk_right = true;
		v.push_back({ "Street 1+1", p });

		ProfileParams q = p;
		q.parking_left = q.parking_right = true;
		v.push_back({ "Street 1+1, parking", q });

		ProfileParams b = p;
		b.bike_left = b.bike_right = true;
		v.push_back({ "Street 1+1, bike lanes", b });

		ProfileParams a;
		a.backward = 2;
		a.forward = 2;
		a.median = MedianType::Raised;
		a.median_width = 2.0;
		a.sidewalk_left = a.sidewalk_right = true;
		a.sidewalk_width = 3.0;
		v.push_back({ "Avenue 2+2, median", a });

		ProfileParams bus = a;
		bus.median = MedianType::Painted;
		bus.median_width = 1.0;
		bus.bus_left = bus.bus_right = true;
		bus.parking_left = bus.parking_right = true;
		v.push_back({ "Avenue 2+2, bus lanes, parking", bus });

		ProfileParams o;
		o.backward = 0;
		o.forward = 2;
		o.sidewalk_left = o.sidewalk_right = true;
		v.push_back({ "One-way 2 lanes", o });

		ProfileParams o3;
		o3.backward = 0;
		o3.forward = 3;
		v.push_back({ "One-way 3 lanes", o3 });

		ProfileParams c;
		c.backward = 1;
		c.forward = 1;
		c.lane_width = 3.5;
		v.push_back({ "Country road 1+1", c });

		ProfileParams h;
		h.backward = 3;
		h.forward = 3;
		h.lane_width = 3.5;
		h.median = MedianType::Raised;
		h.median_width = 3.0;
		v.push_back({ "Highway 3+3", h });
		return v;
	}();
	return presets;
}

} // namespace tsim
