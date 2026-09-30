#include "tsim/demo_maps.h"

#include <cmath>
#include <cstring>

namespace tsim {

Profile preset_profile(const char *name, RoadMap &map) {
	const std::vector<ProfilePreset> &presets = profile_presets();
	for (const ProfilePreset &p : presets) {
		if (std::strcmp(p.name, name) == 0) {
			return build_profile(p.params, nullptr, map);
		}
	}
	return build_profile(presets.front().params, nullptr, map);
}

namespace {

PointRef at(const Document &doc, NodeId n) {
	PointRef r;
	r.node = n;
	r.pos = doc.map().node(n)->pos;
	return r;
}

SegmentId road(Document &doc, NodeId a, NodeId b, const Profile &p, double kmh) {
	const std::vector<SegmentId> s = doc.add_road({ at(doc, a), at(doc, b) }, p, doc.map().node(a)->level, kmh / 3.6);
	return s.empty() ? kNoId : s.front();
}

void pockets(Document &doc, SegmentId s, bool left, bool right, double length) {
	for (int e = 0; e < 2; ++e) {
		EndRules r;
		r.left = left ? TurnRule::TurnLane : TurnRule::Allowed;
		r.right = right ? TurnRule::TurnLane : TurnRule::Allowed;
		r.turn_lane_length = length;
		doc.set_end_rules(s, e, r);
	}
}

} // namespace

void build_test_grid(Document &doc, int cols, int rows, double spacing) {
	RoadMap scratch;
	const Profile street = preset_profile("Street 1+1", scratch);
	const Profile parking = preset_profile("Street 1+1, parking", scratch);
	const Profile avenue = preset_profile("Avenue 2+2, median", scratch);
	const Profile bus = preset_profile("Avenue 2+2, bus lanes, parking", scratch);
	const Profile oneway = preset_profile("One-way 2 lanes", scratch);
	ProfileParams single;
	single.backward = 0;
	single.forward = 1;
	single.sidewalk_left = single.sidewalk_right = true;
	const Profile oneway1 = build_profile(single, nullptr, scratch);

	doc.begin("Generate test grid");
	std::vector<NodeId> ids(static_cast<size_t>(cols * rows));
	auto id = [&](int i, int j) -> NodeId & { return ids[static_cast<size_t>(j * cols + i)]; };
	for (int j = 0; j < rows; ++j) {
		for (int i = 0; i < cols; ++i) {
			id(i, j) = doc.add_node(Vec2{ i * spacing, j * spacing }, 0);
		}
	}
	// Rows: every third row is an avenue with turn pockets.
	for (int j = 0; j < rows; ++j) {
		for (int i = 0; i + 1 < cols; ++i) {
			const bool is_avenue = j % 3 == 1;
			const Profile &p = is_avenue ? (j % 2 == 1 ? bus : avenue) : street;
			const SegmentId s = road(doc, id(i, j), id(i + 1, j), p, is_avenue ? 60.0 : 50.0);
			if (is_avenue) pockets(doc, s, true, false, 30.0);
			if (j == 0) {
				// Gently curved bottom row (Bézier).
				const Vec2 a = doc.map().node(id(i, j))->pos;
				const Vec2 b = doc.map().node(id(i + 1, j))->pos;
				const double bulge = (i % 2 == 0 ? -1.0 : 1.0) * 14.0;
				doc.set_control_point(s, 0, a + (b - a) * (1.0 / 3.0) + Vec2{ 0.0, bulge });
				doc.set_control_point(s, 1, a + (b - a) * (2.0 / 3.0) + Vec2{ 0.0, bulge });
			}
		}
	}
	// Columns: two-way streets and alternating one-way pairs.
	for (int i = 0; i < cols; ++i) {
		for (int j = 0; j + 1 < rows; ++j) {
			if (i % 2 == 1) {
				const bool up = i % 4 == 1;
				const NodeId a = up ? id(i, j + 1) : id(i, j);
				const NodeId b = up ? id(i, j) : id(i, j + 1);
				const SegmentId s = road(doc, a, b, oneway, 50.0);
				if (i == 1 && j == 2) {
					// Lane drop mid-block: the second half narrows to one lane.
					doc.split_segment(s, 0.5);
					const RoadSegment *first = doc.map().segment(s);
					for (SegmentId other : doc.map().segments_at(first->to)) {
						if (other != s) doc.set_profile(other, doc.instantiate(oneway1));
					}
				}
			} else {
				road(doc, id(i, j), id(i, j + 1), parking, 50.0);
			}
		}
	}
	// M2: roads out of the grid with spawn / sink points at their ends. One-way
	// columns only feed traffic in or out, in their own direction.
	const double stub = 80.0;
	auto edge = [&](NodeId inner, Vec2 dir, const Profile &p, int flow, double kmh) {
		// flow: 0 two-way, +1 traffic leaves the grid, -1 traffic enters it.
		const Vec2 at = doc.map().node(inner)->pos + dir * stub;
		const NodeId outer = doc.add_node(at, 0);
		if (flow < 0) {
			road(doc, outer, inner, p, kmh);
		} else {
			road(doc, inner, outer, p, kmh);
		}
		Spawner sp;
		sp.enabled = true;
		sp.rate = flow > 0 ? 0.0 : 300.0;
		sp.sink = flow >= 0;
		doc.set_spawner(outer, sp);
	};
	for (int j = 0; j < rows; ++j) {
		const bool is_avenue = j % 3 == 1;
		const Profile &p = is_avenue ? (j % 2 == 1 ? bus : avenue) : street;
		edge(id(0, j), Vec2{ -1.0, 0.0 }, p, 0, is_avenue ? 60.0 : 50.0);
		edge(id(cols - 1, j), Vec2{ 1.0, 0.0 }, p, 0, is_avenue ? 60.0 : 50.0);
	}
	for (int i = 0; i < cols; ++i) {
		if (i % 2 == 1) {
			const bool up = i % 4 == 1; // traffic runs towards row 0
			edge(id(i, 0), Vec2{ 0.0, -1.0 }, oneway, up ? 1 : -1, 50.0);
			edge(id(i, rows - 1), Vec2{ 0.0, 1.0 }, oneway, up ? -1 : 1, 50.0);
		} else {
			edge(id(i, 0), Vec2{ 0.0, -1.0 }, parking, 0, 50.0);
			edge(id(i, rows - 1), Vec2{ 0.0, 1.0 }, parking, 0, 50.0);
		}
	}
	// Junction control: avenues are priority roads, a few streets get all-way
	// stops, the rest keep the right-hand rule.
	for (int j = 0; j < rows; ++j) {
		for (int i = 0; i < cols; ++i) {
			const NodeId n = id(i, j);
			if (j % 3 == 1) {
				std::vector<SegmentId> main;
				for (SegmentId s : doc.map().segments_at(n)) {
					const RoadSegment *seg = doc.map().segment(s);
					const Vec2 a = doc.map().node(seg->from)->pos;
					const Vec2 b = doc.map().node(seg->to)->pos;
					if (std::fabs(a.y - b.y) < 1.0) main.push_back(s); // east-west legs
				}
				doc.set_junction_control(n, JunctionControl::PriorityRoad, main);
			} else if (i > 0 && i + 1 < cols && j > 0 && j + 1 < rows && (i + j) % 5 == 0) {
				doc.set_junction_control(n, JunctionControl::AllWayStop, {});
			}
		}
	}
	doc.commit();
}

// --- M2 test maps ----------------------------------------------------------------

namespace {

void spawn_at(Document &doc, NodeId n, double rate, bool sink = true) {
	Spawner sp;
	sp.enabled = true;
	sp.rate = rate;
	sp.sink = sink;
	doc.set_spawner(n, sp);
}

} // namespace

void build_t_junction(Document &doc) {
	RoadMap scratch;
	const Profile street = preset_profile("Street 1+1", scratch);
	doc.begin("T junction");
	const NodeId w = doc.add_node(Vec2{ -200, 0 }, 0), c = doc.add_node(Vec2{ 0, 0 }, 0),
				 e = doc.add_node(Vec2{ 200, 0 }, 0), s = doc.add_node(Vec2{ 0, 200 }, 0);
	const SegmentId a = road(doc, w, c, street, 50);
	const SegmentId b = road(doc, c, e, street, 50);
	road(doc, s, c, street, 50);
	doc.set_junction_control(c, JunctionControl::PriorityRoad, { a, b });
	spawn_at(doc, w, 400);
	spawn_at(doc, e, 400);
	spawn_at(doc, s, 300);
	doc.commit();
}

void build_lane_drop(Document &doc) {
	RoadMap scratch;
	ProfileParams two;
	two.backward = 0;
	two.forward = 2;
	two.sidewalk_left = two.sidewalk_right = true;
	ProfileParams one = two;
	one.forward = 1;
	const Profile p2 = build_profile(two, nullptr, scratch);
	const Profile p1 = build_profile(one, nullptr, scratch);
	doc.begin("Lane drop");
	const NodeId a = doc.add_node(Vec2{ -300, 0 }, 0), b = doc.add_node(Vec2{ 0, 0 }, 0),
				 c = doc.add_node(Vec2{ 300, 0 }, 0);
	road(doc, a, b, p2, 50);
	road(doc, b, c, p1, 50);
	spawn_at(doc, a, 900, false);
	spawn_at(doc, c, 0, true);
	doc.commit();
}

void build_one_way_pair(Document &doc) {
	RoadMap scratch;
	const Profile oneway = preset_profile("One-way 2 lanes", scratch);
	const Profile street = preset_profile("Street 1+1", scratch);
	doc.begin("One-way pair");
	// Two parallel one-way streets 100 m apart (east- and westbound), joined
	// by two two-way cross streets that run out to spawn points.
	const NodeId n_w = doc.add_node(Vec2{ -300, 0 }, 0), n1 = doc.add_node(Vec2{ -100, 0 }, 0),
				 n2 = doc.add_node(Vec2{ 100, 0 }, 0), n_e = doc.add_node(Vec2{ 300, 0 }, 0);
	const NodeId s_w = doc.add_node(Vec2{ -300, 100 }, 0), s1 = doc.add_node(Vec2{ -100, 100 }, 0),
				 s2 = doc.add_node(Vec2{ 100, 100 }, 0), s_e = doc.add_node(Vec2{ 300, 100 }, 0);
	road(doc, n_w, n1, oneway, 50);
	road(doc, n1, n2, oneway, 50);
	road(doc, n2, n_e, oneway, 50);
	road(doc, s_e, s2, oneway, 50);
	road(doc, s2, s1, oneway, 50);
	road(doc, s1, s_w, oneway, 50);
	const NodeId t1 = doc.add_node(Vec2{ -100, -150 }, 0), t2 = doc.add_node(Vec2{ 100, -150 }, 0);
	const NodeId b1 = doc.add_node(Vec2{ -100, 250 }, 0), b2 = doc.add_node(Vec2{ 100, 250 }, 0);
	road(doc, t1, n1, street, 50);
	road(doc, n1, s1, street, 50);
	road(doc, s1, b1, street, 50);
	road(doc, t2, n2, street, 50);
	road(doc, n2, s2, street, 50);
	road(doc, s2, b2, street, 50);
	spawn_at(doc, n_w, 500, false);
	spawn_at(doc, n_e, 0, true);
	spawn_at(doc, s_e, 500, false);
	spawn_at(doc, s_w, 0, true);
	for (NodeId n : { t1, t2, b1, b2 }) spawn_at(doc, n, 200);
	doc.commit();
}

void build_demo_town(Document &doc) {
	RoadMap scratch;
	const Profile street = preset_profile("Street 1+1", scratch);
	const Profile bike = preset_profile("Street 1+1, bike lanes", scratch);
	const Profile bus = preset_profile("Avenue 2+2, bus lanes, parking", scratch);
	const Profile avenue = preset_profile("Avenue 2+2, median", scratch);
	const Profile oneway = preset_profile("One-way 2 lanes", scratch);

	doc.begin("Demo town");
	auto node = [&](double x, double y) { return doc.add_node(Vec2{ x, y }, 0); };
	// Main avenue west-east with bus lanes, narrowing to a plain avenue east.
	const NodeId w = node(-360, 0), a1 = node(-120, 0), a2 = node(120, 0), e1 = node(300, 0), e2 = node(480, 0);
	const SegmentId s1 = road(doc, w, a1, bus, 60);
	const SegmentId s2 = road(doc, a1, a2, bus, 60);
	const SegmentId s3 = road(doc, a2, e1, bus, 60);
	road(doc, e1, e2, avenue, 60);
	pockets(doc, s1, true, false, 40);
	pockets(doc, s2, true, false, 40);
	pockets(doc, s3, true, false, 40);
	// North-south street through the west junction.
	const NodeId n1 = node(-120, -260), s_1 = node(-120, 240);
	const SegmentId ns1 = road(doc, n1, a1, street, 50);
	const SegmentId ns2 = road(doc, a1, s_1, street, 50);
	EndRules right_pocket;
	right_pocket.right = TurnRule::TurnLane;
	right_pocket.turn_lane_length = 35;
	doc.set_end_rules(ns1, 1, right_pocket);
	doc.set_end_rules(ns2, 0, right_pocket);
	// One-way southbound through the east junction, on to the map edge.
	const NodeId n2 = node(120, -260), s_2 = node(120, 240), s_3 = node(120, 420);
	road(doc, n2, a2, oneway, 50);
	road(doc, a2, s_2, oneway, 50);
	road(doc, s_2, s_3, oneway, 50);
	// A curved road joining the two northern ends.
	const std::vector<SegmentId> none;
	doc.add_curve(at(doc, n1), Vec2{ 0, -400 }, at(doc, n2), street, 0, 40 / 3.6);
	// Bike street west from the southern end.
	const NodeId bw = node(-380, 240);
	road(doc, s_1, bw, bike, 30);
	// Southern link with a no-change zone on the avenue's inner lanes.
	road(doc, s_1, s_2, street, 50);
	(void)none;
	const RoadSegment *mid = doc.map().segment(s2);
	if (mid) {
		// Solid line between the two eastbound lanes in the middle of the block.
		int edge = -1;
		for (size_t i = 1; i < mid->profile.lanes.size(); ++i) {
			const LaneSpec &l = mid->profile.lanes[i - 1];
			const LaneSpec &r = mid->profile.lanes[i];
			if (l.dir == LaneDir::Forward && r.dir == LaneDir::Forward && is_travel(l.type) && is_travel(r.type)) {
				edge = static_cast<int>(i);
				break;
			}
		}
		if (edge > 0) doc.set_no_change(s2, edge, 0.3, 0.6, true, true);
	}
	// M2: the main avenue has priority at its junctions; traffic enters and
	// leaves at the road ends.
	for (NodeId j : { a1, a2 }) {
		std::vector<SegmentId> main;
		for (SegmentId sid : doc.map().segments_at(j)) {
			for (SegmentId m : { s1, s2, s3 }) {
				if (sid == m) main.push_back(sid);
			}
		}
		doc.set_junction_control(j, JunctionControl::PriorityRoad, main);
	}
	doc.set_junction_control(s_1, JunctionControl::AllWayStop, {});
	Spawner in_out;
	in_out.enabled = true;
	in_out.rate = 400.0;
	doc.set_spawner(w, in_out);
	doc.set_spawner(e2, in_out);
	in_out.rate = 150.0;
	doc.set_spawner(bw, in_out);
	Spawner out_only;
	out_only.enabled = true;
	out_only.rate = 0.0;
	doc.set_spawner(s_3, out_only);
	doc.commit();
}

// --- M3 showcase ----------------------------------------------------------------------

void build_showcase(Document &doc) {
	RoadMap scratch;
	const Profile street = preset_profile("Street 1+1", scratch);
	const Profile bike = preset_profile("Street 1+1, bike lanes", scratch);
	const Profile bus = preset_profile("Avenue 2+2, bus lanes, parking", scratch);
	const Profile avenue = preset_profile("Avenue 2+2, median", scratch);
	auto parking = [&](ParkingStyle style) {
		ProfileParams p;
		p.sidewalk_left = p.sidewalk_right = true;
		p.parking_left = p.parking_right = true;
		p.parking_style = style;
		return build_profile(p, nullptr, scratch);
	};
	const Profile parallel = parking(ParkingStyle::Parallel);
	const Profile angled = parking(ParkingStyle::Angle45);
	const Profile perpendicular = parking(ParkingStyle::Perpendicular);

	doc.begin("Showcase town");
	auto node = [&](double x, double y) { return doc.add_node(Vec2{ x, y }, 0); };
	// Main avenue west-east: bus lanes to a signalized junction (A) and on to a
	// roundabout (B), then a plain avenue to the east edge.
	const NodeId w = node(-560, 0), a = node(-220, 0), b = node(180, 0), e = node(520, 0);
	const SegmentId wa = road(doc, w, a, bus, 50);
	const SegmentId ab = road(doc, a, b, bus, 50);
	road(doc, b, e, avenue, 60);
	pockets(doc, wa, true, false, 40);
	{
		// Westbound left turns into the bike street get a pocket at the signal.
		EndRules r;
		r.left = TurnRule::TurnLane;
		r.turn_lane_length = 45;
		doc.set_end_rules(ab, 0, r);
	}
	// North-south bike street through A.
	const NodeId n1 = node(-220, -320), s1 = node(-220, 300), s3 = node(-220, 480);
	road(doc, n1, a, bike, 40);
	const SegmentId as1 = road(doc, a, s1, bike, 40);
	road(doc, s1, s3, bike, 40);
	// Roundabout legs north and south, with angled and perpendicular parking.
	const NodeId n2 = node(180, -330), s2 = node(180, 300), s4 = node(180, 480);
	const SegmentId bn = road(doc, b, n2, angled, 40);
	const SegmentId bs = road(doc, b, s2, perpendicular, 40);
	road(doc, s2, s4, street, 50);
	// Southern street with parallel parking, and the depot road to the west.
	const SegmentId s12 = road(doc, s1, s2, parallel, 40);
	const NodeId d = node(-520, 300);
	road(doc, d, s1, street, 40);

	// Junction control.
	doc.set_signal_plan(a, default_signal_plan(doc.map(), a));
	{
		SignalPlan plan = doc.map().node(a)->signal;
		plan.right_on_red = { as1 }; // northbound traffic from the south may turn right on red
		doc.set_signal_plan(a, plan);
	}
	Roundabout r;
	r.enabled = true;
	r.radius = 24.0;
	r.lanes = 2;
	r.slip = { bs }; // southern leg: bypass to the east exit
	doc.set_roundabout(b, r);
	doc.set_junction_control(s1, JunctionControl::AllWayStop, {});
	std::vector<SegmentId> main;
	for (SegmentId sid : doc.map().segments_at(s2)) {
		if (sid == bs || sid == s12) continue;
		main.push_back(sid);
	}
	main.push_back(bs);
	doc.set_junction_control(s2, JunctionControl::PriorityRoad, main);
	(void)bn;

	// Stops: the main station on the avenue (eastbound), kerbside stops on the
	// bike street and the roundabout's south leg, a bus bay on the south street.
	const uint32_t station = doc.add_stop(ab, 0.5, LaneDir::Forward, StopKind::MainStation, "Central Station");
	if (const RoadSegment *seg = doc.map().segment(ab)) {
		for (BusStop st : seg->stops) {
			if (st.id != station) continue;
			st.bays = 3;
			doc.set_stop(ab, st);
		}
	}
	const uint32_t south_leg = doc.add_stop(bs, 0.55, LaneDir::Forward, StopKind::Kerbside, "Roundabout South");
	const uint32_t north_leg = doc.add_stop(bs, 0.45, LaneDir::Backward, StopKind::Kerbside, "Roundabout North");
	const uint32_t market_w = doc.add_stop(s12, 0.5, LaneDir::Backward, StopKind::Bay, "Market");
	const uint32_t market_e = doc.add_stop(s12, 0.4, LaneDir::Forward, StopKind::Bay, "Market East");
	const uint32_t school = doc.add_stop(as1, 0.5, LaneDir::Backward, StopKind::Kerbside, "School");

	// Depot with a loop round the block and an end-to-end route.
	Depot depot;
	depot.enabled = true;
	depot.name = "West Depot";
	depot.capacity = 12;
	BusRoute loop;
	loop.name = "1 Circle";
	loop.color = 0xd83f3f;
	loop.stops = { station, south_leg, market_w, school };
	loop.headway = 300.0;
	loop.loop = true;
	BusRoute line;
	line.name = "2 Market";
	line.color = 0x2f7fd8;
	line.stops = { market_e, north_leg };
	line.headway = 480.0;
	depot.routes = { loop, line };
	doc.set_depot(d, depot);

	// Demand: cars at every edge, bikes on the bike street, a coach line west-east.
	Spawner in_out;
	in_out.enabled = true;
	in_out.rate = 350.0;
	Spawner west = in_out;
	CoachLine coach;
	coach.exit = e;
	coach.per_hour = 3.0;
	coach.dwell = 300.0;
	west.coaches = { coach };
	doc.set_spawner(w, west);
	doc.set_spawner(e, in_out);
	in_out.rate = 200.0;
	doc.set_spawner(n2, in_out);
	doc.set_spawner(s4, in_out);
	in_out.bikes = 120.0;
	doc.set_spawner(n1, in_out);
	doc.set_spawner(s3, in_out);
	doc.commit();
}

} // namespace tsim
