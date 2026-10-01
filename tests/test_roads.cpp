// M1 tests: curves, profiles, editing + undo, JSON, geometry, validation, gate.
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include <doctest/doctest.h>

#include "tsim/curve.h"
#include "tsim/demo_maps.h"
#include "tsim/document.h"
#include "tsim/rng.h"
#include "tsim/road_geometry.h"
#include "tsim/road_map_json.h"
#include "tsim/validation.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace tsim;

namespace {

Profile preset(const char *name) {
	RoadMap scratch;
	return preset_profile(name, scratch);
}

PointRef free_point(double x, double y) {
	PointRef p;
	p.pos = Vec2{ x, y };
	return p;
}

PointRef node_point(const Document &doc, NodeId n) {
	PointRef p;
	p.node = n;
	p.pos = doc.map().node(n)->pos;
	return p;
}

int count_errors(const std::vector<Problem> &ps, const char *code = nullptr) {
	int n = 0;
	for (const Problem &p : ps) {
		if (p.severity == Severity::Error && (!code || p.code == code)) ++n;
	}
	return n;
}

// Four-way junction at the origin, arms 100 m long.
Document cross_roads(const char *profile = "Street 1+1") {
	Document doc;
	const Profile p = preset(profile);
	doc.add_road({ free_point(-100, 0), free_point(0, 0) }, p, 0, 13.9);
	const NodeId c = doc.map().segment(1)->to;
	doc.add_road({ node_point(doc, c), free_point(100, 0) }, p, 0, 13.9);
	doc.add_road({ free_point(0, -100), node_point(doc, c) }, p, 0, 13.9);
	doc.add_road({ node_point(doc, c), free_point(0, 100) }, p, 0, 13.9);
	return doc;
}

} // namespace

TEST_CASE("curves: lengths and turning are exact for straight and arc") {
	Curve s;
	s.p0 = { 0, 0 };
	s.p3 = { 30, 40 };
	CHECK(s.length() == doctest::Approx(50.0));
	CHECK(s.turn() == 0.0);

	Curve a;
	a.kind = CurveKind::Arc;
	a.p0 = { 100, 0 };
	a.p3 = { 0, 100 };
	a.sweep = kHalfPi;
	a.center = arc_center_for(a.p0, a.p3, a.sweep);
	CHECK(a.center.x == doctest::Approx(0.0).epsilon(1e-9));
	CHECK(a.center.y == doctest::Approx(0.0).epsilon(1e-9));
	CHECK(a.length() == doctest::Approx(100.0 * kHalfPi));
	// Right side of a clockwise arc is the inside.
	CHECK(a.offset_length(2.0) == doctest::Approx(98.0 * kHalfPi));
}

TEST_CASE("curves: bezier length, turn and split") {
	Curve b;
	b.kind = CurveKind::Bezier;
	b.p0 = { 0, 0 };
	b.c1 = { 30, 0 };
	b.c2 = { 70, 50 };
	b.p3 = { 100, 50 };
	double poly = 0.0;
	Vec2 prev = b.point(0.0);
	for (int i = 1; i <= 20000; ++i) {
		const Vec2 q = b.point(i / 20000.0);
		poly += (q - prev).length();
		prev = q;
	}
	CHECK(b.length() == doctest::Approx(poly).epsilon(1e-6));
	// S-curve: turns right then back left, so the total is ~0.
	CHECK(std::fabs(b.turn()) < 1e-6);

	Curve f, g;
	split_curve(b, 0.3, f, g);
	CHECK((f.p3 - g.p0).length() < 1e-12);
	CHECK(f.length() + g.length() == doctest::Approx(b.length()).epsilon(1e-9));
	CHECK((f.point(1.0) - b.point(0.3)).length() < 1e-9);

	ArcTable t;
	t.build(b);
	CHECK(t.length() == doctest::Approx(b.length()).epsilon(1e-9));
	CHECK(t.s_at(t.t_at(40.0)) == doctest::Approx(40.0).epsilon(1e-9));

	double d = 0.0;
	const double ct = closest_t(b, b.point(0.62) + b.tangent(0.62).right() * 3.0, &d);
	CHECK(ct == doctest::Approx(0.62).epsilon(1e-3));
	CHECK(d == doctest::Approx(3.0).epsilon(1e-3));
}

TEST_CASE("profiles: presets are valid and params round-trip") {
	RoadMap m;
	for (const ProfilePreset &p : profile_presets()) {
		const Profile pr = build_profile(p.params, nullptr, m);
		CHECK_MESSAGE(validate_profile(pr).empty(), p.name);
		const ProfileParams back = params_of(pr);
		CHECK(back.forward == p.params.forward);
		CHECK(back.backward == p.params.backward);
		CHECK(back.sidewalk_left == p.params.sidewalk_left);
		CHECK(back.parking_right == p.params.parking_right);
		CHECK(back.bus_right == p.params.bus_right);
	}
}

TEST_CASE("profiles: rebuilding keeps lane IDs by role") {
	RoadMap m;
	ProfileParams p;
	p.backward = 1;
	p.forward = 1;
	p.sidewalk_left = p.sidewalk_right = true;
	const Profile a = build_profile(p, nullptr, m);
	p.forward = 2; // add a lane on the outside of the forward group
	const Profile b = build_profile(p, &a, m);
	REQUIRE(b.lanes.size() == a.lanes.size() + 1);
	CHECK(b.lanes.front().id == a.lanes.front().id); // left sidewalk
	CHECK(b.lanes.back().id == a.lanes.back().id); // right sidewalk
	CHECK(b.lanes[1].id == a.lanes[1].id); // backward lane
	CHECK(b.lanes[2].id == a.lanes[2].id); // inner forward lane
	CHECK(b.lanes[3].id >= m.next_lane_id() - 1); // new outer forward lane

	Profile bad = a;
	std::swap(bad.lanes[1], bad.lanes[2]);
	CHECK_FALSE(validate_profile(bad).empty());
}

TEST_CASE("editing: roads, snapping, splitting") {
	Document doc;
	const Profile p = preset("Street 1+1");
	const auto s = doc.add_road({ free_point(0, 0), free_point(100, 0), free_point(100, 100) }, p, 0, 13.9);
	CHECK(s.size() == 2);
	CHECK(doc.map().nodes().size() == 3);
	// Start a road on the middle of the first segment: splits it into a T.
	PointRef onto;
	onto.segment = s[0];
	onto.pos = Vec2{ 40, 1 };
	const auto t = doc.add_road({ onto, free_point(40, -80) }, p, 0, 13.9);
	REQUIRE(t.size() == 1);
	CHECK(doc.map().segments().size() == 4);
	const NodeId mid = doc.map().segment(t[0])->from;
	CHECK(doc.map().segments_at(mid).size() == 3);
	CHECK(doc.map().node(mid)->pos.x == doctest::Approx(40.0).epsilon(1e-6));
	// A road on another level does not snap to ground nodes.
	PointRef lvl = node_point(doc, mid);
	const auto u = doc.add_road({ lvl, free_point(40, 80) }, p, 1, 13.9);
	REQUIRE(u.size() == 1);
	CHECK(doc.map().segment(u[0])->from != mid);
	CHECK(doc.map().node(doc.map().segment(u[0])->from)->level == 1);
}

TEST_CASE("editing: undo, redo, cancel restore exact state") {
	Document doc;
	const Profile p = preset("Avenue 2+2, median");
	const RoadMap empty = doc.map();
	doc.add_road({ free_point(0, 0), free_point(200, 0) }, p, 0, 16.7);
	const RoadMap one = doc.map();
	doc.move_node(1, Vec2{ -10, 5 });
	doc.set_speed_limit(1, 20.0);
	CHECK(doc.undo_count() == 3);
	CHECK(doc.undo());
	CHECK(doc.undo());
	CHECK(doc.map() == one);
	CHECK(doc.undo());
	CHECK(doc.map() == empty);
	CHECK_FALSE(doc.undo());
	CHECK(doc.redo());
	CHECK(doc.map() == one);
	// A new edit clears redo.
	doc.set_name(1, "Main");
	CHECK_FALSE(doc.can_redo());
	// Cancel rolls back an open transaction.
	const RoadMap before = doc.map();
	doc.begin("drag");
	doc.move_node(1, Vec2{ 50, 50 });
	doc.move_node(1, Vec2{ 60, 50 });
	doc.cancel();
	CHECK(doc.map() == before);
	// A drag in one transaction is one undo step.
	const size_t n = doc.undo_count();
	doc.begin("drag");
	for (int i = 0; i < 20; ++i) doc.move_node(1, Vec2{ static_cast<double>(i), 0 });
	doc.commit();
	CHECK(doc.undo_count() == n + 1);
	// IDs are never reused after undo.
	const uint32_t next = doc.map().next_segment_id();
	doc.undo();
	doc.add_road({ free_point(0, 50), free_point(100, 50) }, p, 0, 16.7);
	CHECK(doc.map().segments().rbegin()->first >= next);
}

TEST_CASE("editing: move keeps bezier handles, merge joins, delete cleans up") {
	Document doc;
	const Profile p = preset("Street 1+1");
	const SegmentId c = doc.add_curve(free_point(0, 0), Vec2{ 50, -50 }, free_point(100, 0), p, 0, 13.9);
	REQUIRE(c != kNoId);
	const RoadSegment before = *doc.map().segment(c);
	doc.move_node(before.from, Vec2{ -10, 0 });
	CHECK(doc.map().segment(c)->c1.x == doctest::Approx(before.c1.x - 10));
	CHECK(doc.map().segment(c)->c2 == before.c2);

	const auto s = doc.add_road({ free_point(200, 0), free_point(300, 0) }, p, 0, 13.9);
	const NodeId a = doc.map().segment(s[0])->from;
	CHECK(doc.merge_nodes(a, before.to));
	CHECK_FALSE(doc.map().node(a));
	CHECK(doc.map().segments_at(before.to).size() == 2);

	doc.delete_segment(s[0]);
	CHECK(doc.map().nodes().size() == 2);
	doc.delete_node(before.to);
	CHECK(doc.map().segments().empty());
	CHECK(doc.map().nodes().empty());
}

TEST_CASE("editing: flip, level change, lane type, no-change paint") {
	Document doc;
	const Profile ow = preset("One-way 2 lanes");
	const auto s = doc.add_road({ free_point(0, 0), free_point(100, 0), free_point(200, 0) }, ow, 0, 13.9);
	const RoadSegment before = *doc.map().segment(s[0]);
	CHECK(doc.flip(s[0]));
	const RoadSegment &after = *doc.map().segment(s[0]);
	CHECK(after.from == before.to);
	CHECK(after.profile.lanes.front().id == before.profile.lanes.back().id);
	CHECK(doc.flip(s[0]));
	CHECK(*doc.map().segment(s[0]) == before);

	const Profile two = preset("Street 1+1");
	const auto t = doc.add_road({ free_point(0, 50), free_point(100, 50) }, two, 0, 13.9);
	CHECK_FALSE(doc.flip(t[0]));

	// Changing the level of a road whose node is shared gives it its own node.
	const NodeId shared = doc.map().segment(s[0])->to;
	doc.set_level(s[1], 1);
	CHECK(doc.map().segment(s[1])->from != shared);
	CHECK(doc.map().node(doc.map().segment(s[1])->from)->level == 1);
	CHECK(doc.map().segment(s[1])->level == 1);

	// Lane type: sidewalk -> bus lane takes a direction.
	const LaneId sw = doc.map().segment(t[0])->profile.lanes.back().id;
	CHECK(doc.set_lane_type(t[0], sw, LaneType::Bus).empty());
	CHECK(doc.map().segment(t[0])->profile.lanes.back().dir == LaneDir::Forward);

	// Paint, extend (merge) and partly clear a zone.
	doc.set_no_change(s[0], 2, 0.2, 0.5, true, true);
	doc.set_no_change(s[0], 2, 0.4, 0.7, true, true);
	REQUIRE(doc.map().segment(s[0])->no_change.size() == 1);
	CHECK(doc.map().segment(s[0])->no_change[0].u1 == doctest::Approx(0.7));
	doc.set_no_change(s[0], 2, 0.3, 0.4, false, false);
	CHECK(doc.map().segment(s[0])->no_change.size() == 2);
}

TEST_CASE("json: save -> load -> save is identical; errors leave the target alone") {
	Document doc;
	build_demo_town(doc);
	const std::string a = road_map_to_json(doc.map());
	RoadMap loaded;
	std::string err;
	REQUIRE_MESSAGE(road_map_from_json(a, loaded, &err), err);
	CHECK(loaded == doc.map());
	CHECK(road_map_to_json(loaded) == a);

	RoadMap target = loaded;
	CHECK_FALSE(road_map_from_json("{", target, &err));
	CHECK_FALSE(road_map_from_json(R"({"format":"traffic-sim-map","version":6})", target, &err));
	CHECK(err.find("newer") != std::string::npos);
	CHECK_FALSE(road_map_from_json(
			R"({"format":"traffic-sim-map","version":2,"nodes":[{"id":1,"x":0,"y":0,"level":0}],
			"segments":[{"id":1,"from":1,"to":2,"kind":"road","level":0,"curve":{"type":"straight"},
			"speed_limit":10,"profile":{"median":"none","lanes":[]}}]})",
			target, &err));
	CHECK(target == loaded);
}

TEST_CASE("json: v1 (POC) ring migrates to a v2 map") {
	const char *v1 = R"({"format":"traffic-sim-map","version":1,"next_ids":{"node":5,"segment":5,"lane":9},
		"nodes":[{"id":1,"x":100.0,"y":0.0},{"id":2,"x":0.0,"y":100.0},{"id":3,"x":-100.0,"y":0.0},{"id":4,"x":0.0,"y":-100.0}],
		"segments":[
		 {"id":1,"from":1,"to":2,"kind":"arc","lane_width":3.5,"speed_limit":16.6,"lanes":[1,2],"center":{"x":0,"y":0},"sweep":1.5707963267948966},
		 {"id":2,"from":2,"to":3,"kind":"arc","lane_width":3.5,"speed_limit":16.6,"lanes":[3,4],"center":{"x":0,"y":0},"sweep":1.5707963267948966},
		 {"id":3,"from":3,"to":4,"kind":"arc","lane_width":3.5,"speed_limit":16.6,"lanes":[5,6],"center":{"x":0,"y":0},"sweep":1.5707963267948966},
		 {"id":4,"from":4,"to":1,"kind":"arc","lane_width":3.5,"speed_limit":16.6,"lanes":[7,8],"center":{"x":0,"y":0},"sweep":1.5707963267948966}],
		"lanes":[]})";
	RoadMap m;
	std::string err;
	int version = 0;
	REQUIRE_MESSAGE(road_map_from_json(v1, m, &err, &version), err);
	CHECK(version == 1);
	CHECK(m.segments().size() == 4);
	const RoadSegment &s = *m.segment(1);
	CHECK(s.curve == CurveKind::Arc);
	CHECK(s.profile.lanes.size() == 2);
	CHECK(s.profile.lanes[0].id == 2); // v1 listed right-to-left
	CHECK(m.next_lane_id() >= 9);
	RoadGeometry g;
	g.build(m);
	for (const auto &kv : g.nodes()) CHECK(kv.second.kind == NodeKind::Continuation);
	// Re-saving writes the current version.
	CHECK(road_map_to_json(m).find("\"version\": 5") != std::string::npos);
}

TEST_CASE("geometry: four-way junction is trimmed, filled and connected") {
	Document doc = cross_roads();
	RoadGeometry g;
	g.build(doc.map());
	const NodeId c = doc.map().segment(1)->to;
	const NodeGeom &n = *g.node(c);
	CHECK(n.kind == NodeKind::Junction);
	CHECK(n.legs.size() == 4);
	CHECK(n.polygon.size() > 8);
	for (const Leg &l : n.legs) {
		CHECK(l.trim > 5.0);
		CHECK(l.trim < 20.0);
	}
	// 4 approaches x (straight + left + right).
	int straight = 0, left = 0, right = 0;
	for (const Connector &k : n.connectors) {
		straight += k.turn == TurnKind::Straight;
		left += k.turn == TurnKind::Left;
		right += k.turn == TurnKind::Right;
		CHECK(k.path.size() > 2);
	}
	CHECK(straight == 4);
	CHECK(left == 4);
	CHECK(right == 4);
	CHECK(n.dead_lanes.empty());
	CHECK_FALSE(g.meshes().empty());
	for (const MeshBatch &b : g.meshes()) {
		for (const Vec2 &v : b.vertices) CHECK((std::isfinite(v.x) && std::isfinite(v.y)));
		CHECK(b.indices.size() % 3 == 0);
		CHECK(b.uvs.size() == b.vertices.size());
		CHECK(b.colors.size() == b.vertices.size());
	}
}

TEST_CASE("geometry: turn rules change connectors; pockets get their lane IDs") {
	Document doc = cross_roads();
	const NodeId c = doc.map().segment(1)->to;
	EndRules no_left;
	no_left.left = TurnRule::Disallowed;
	doc.set_end_rules(1, 1, no_left); // segment 1 arrives at the centre with its to-end
	RoadGeometry g;
	g.build(doc.map());
	int left_from_1 = 0;
	for (const Connector &k : g.node(c)->connectors) {
		left_from_1 += k.from_seg == 1 && k.turn == TurnKind::Left;
	}
	CHECK(left_from_1 == 0);

	EndRules pocket;
	pocket.left = TurnRule::TurnLane;
	pocket.turn_lane_length = 40;
	doc.set_end_rules(1, 1, pocket);
	const LaneId pocket_id = doc.map().segment(1)->ends[1].left_lane;
	CHECK(pocket_id != kNoId);
	g.build(doc.map());
	const SegmentGeom &sg = *g.segment(1);
	bool found = false;
	for (const GeomLane &l : sg.lanes) {
		if (l.id == pocket_id) {
			found = true;
			CHECK(l.type == LaneType::Turn);
			CHECK(l.pocket_left);
		}
	}
	CHECK(found);
	bool pocket_turns_left = false;
	for (const Connector &k : g.node(c)->connectors) {
		if (k.from_lane == pocket_id) pocket_turns_left = k.turn == TurnKind::Left;
	}
	CHECK(pocket_turns_left);
	// Road widens near the junction and not far from it.
	double w_near = 0, w_far = 0;
	const size_t last = sg.s.size() - 1;
	for (size_t i = 0; i < sg.lanes.size(); ++i) {
		w_near += sg.edge(last, i).second - sg.edge(last, i).first;
		w_far += sg.edge(0, i).second - sg.edge(0, i).first;
	}
	CHECK(w_near == doctest::Approx(w_far + default_width(LaneType::Turn)).epsilon(1e-6));
	// Undo removes the pocket again.
	doc.undo();
	g.build(doc.map());
	CHECK(g.segment(1)->lanes.size() == doc.map().segment(1)->profile.lanes.size());
}

TEST_CASE("geometry: T junction from the stem turns both ways from one lane") {
	Document doc;
	const Profile p = preset("Street 1+1");
	const auto main = doc.add_road({ free_point(-100, 0), free_point(0, 0), free_point(100, 0) }, p, 0, 13.9);
	const NodeId mid = doc.map().segment(main[0])->to;
	const auto stem = doc.add_road({ free_point(0, 100), node_point(doc, mid) }, p, 0, 13.9);
	RoadGeometry g;
	g.build(doc.map());
	int left = 0, right = 0;
	for (const Connector &k : g.node(mid)->connectors) {
		if (k.from_seg != stem[0]) continue;
		left += k.turn == TurnKind::Left;
		right += k.turn == TurnKind::Right;
	}
	CHECK(left == 1);
	CHECK(right == 1);
}

TEST_CASE("geometry: continuation, taper and wrong-way joins") {
	Document doc;
	const Profile two = preset("One-way 2 lanes");
	const auto s = doc.add_road({ free_point(0, 0), free_point(100, 2), free_point(200, 0) }, two, 0, 13.9);
	RoadGeometry g;
	g.build(doc.map());
	const NodeId mid = doc.map().segment(s[0])->to;
	CHECK(g.node(mid)->kind == NodeKind::Continuation);
	CHECK(g.node(mid)->connectors.size() == 2);

	ProfileParams one;
	one.backward = 0;
	one.forward = 1;
	one.sidewalk_left = one.sidewalk_right = true;
	doc.set_profile_params(s[1], one);
	g.build(doc.map());
	CHECK(g.node(mid)->kind == NodeKind::Taper);
	CHECK(g.node(mid)->connectors.size() == 2); // the dropped lane merges
	CHECK(g.node(mid)->dead_lanes.empty());
	// The dropped lane narrows to nothing at the node.
	const SegmentGeom &a = *g.segment(s[0]);
	const size_t last = a.s.size() - 1;
	double outer = a.edge(last, a.lanes.size() - 2).second - a.edge(last, a.lanes.size() - 2).first;
	CHECK(outer < 0.01);

	// Flip the second road: two one-ways meet head on -> dead lanes -> error.
	doc.flip(s[1]);
	g.build(doc.map());
	CHECK_FALSE(g.node(mid)->dead_lanes.empty());
	CHECK(count_errors(validate(doc.map(), g), "dead_end_lane") >= 1);
}

TEST_CASE("validation: crossing without a junction is an error unless on another level") {
	Document doc;
	const Profile p = preset("Street 1+1");
	doc.add_road({ free_point(-100, 0), free_point(100, 0) }, p, 0, 13.9);
	const auto cross = doc.add_road({ free_point(0, -100), free_point(0, 100) }, p, 0, 13.9);
	RoadGeometry g;
	g.build(doc.map());
	std::vector<Problem> ps = validate(doc.map(), g);
	CHECK(count_errors(ps, "overlap") == 1);
	doc.set_level(cross[0], 1);
	g.build(doc.map());
	ps = validate(doc.map(), g);
	CHECK(count_errors(ps, "overlap") == 0);
}

TEST_CASE("M1 gate: 50+ junction network builds, saves identically and survives 500 undo steps") {
	Document doc;
	build_test_grid(doc, 7, 8, 120.0);
	RoadGeometry g;
	const auto t0 = std::chrono::steady_clock::now();
	g.build(doc.map());
	const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
	int junctions = 0, tapers = 0;
	size_t verts = 0;
	for (const auto &kv : g.nodes()) {
		junctions += kv.second.kind == NodeKind::Junction;
		tapers += kv.second.kind == NodeKind::Taper;
	}
	for (const MeshBatch &b : g.meshes()) verts += b.vertices.size();
	std::printf("gate grid: %d junctions, %d taper(s), %zu segments, %zu mesh vertices, geometry %.1f ms\n",
			junctions, tapers, doc.map().segments().size(), verts, ms);
	CHECK(junctions >= 50);
	CHECK(tapers == 1);
	const std::vector<Problem> problems = validate(doc.map(), g);
	for (const Problem &p : problems) {
		if (p.severity == Severity::Error) std::printf("  unexpected: %s (%s)\n", p.message.c_str(), p.code.c_str());
	}
	CHECK(count_errors(problems) == 0);

	// save -> load -> save
	const std::string first = road_map_to_json(doc.map());
	RoadMap loaded;
	std::string err;
	REQUIRE(road_map_from_json(first, loaded, &err));
	CHECK(road_map_to_json(loaded) == first);

	// 500 random edits, then undo them all and redo them all.
	const RoadMap start = doc.map();
	Rng rng(2026);
	int done = 0;
	for (int step = 0; step < 2000 && done < 500; ++step) {
		const auto &segs = doc.map().segments();
		const auto &nodes = doc.map().nodes();
		auto nth_seg = [&](uint64_t k) { auto it = segs.begin(); std::advance(it, static_cast<long>(k % segs.size())); return it->first; };
		auto nth_node = [&](uint64_t k) { auto it = nodes.begin(); std::advance(it, static_cast<long>(k % nodes.size())); return it->first; };
		const size_t before = doc.undo_count();
		switch (rng.next_u64() % 11) {
			case 0: {
				const NodeId n = nth_node(rng.next_u64());
				doc.move_node(n, doc.map().node(n)->pos + Vec2{ rng.range(-20, 20), rng.range(-20, 20) });
				break;
			}
			case 1: doc.set_speed_limit(nth_seg(rng.next_u64()), rng.range(8, 25)); break;
			case 2: {
				const SegmentId s = nth_seg(rng.next_u64());
				ProfileParams p = params_of(doc.map().segment(s)->profile);
				p.forward = 1 + static_cast<int>(rng.next_u64() % 3);
				doc.set_profile_params(s, p);
				break;
			}
			case 3: doc.split_segment(nth_seg(rng.next_u64()), rng.range(0.2, 0.8)); break;
			case 4: {
				const NodeId a = nth_node(rng.next_u64());
				const Vec2 pa = doc.map().node(a)->pos;
				PointRef ra;
				ra.node = a;
				ra.pos = pa;
				doc.add_road({ ra, free_point(pa.x + rng.range(-60, 60), pa.y + rng.range(-60, 60)) },
						preset("Street 1+1"), 0, 13.9);
				break;
			}
			case 5: if (segs.size() > 20) doc.delete_segment(nth_seg(rng.next_u64())); break;
			case 6: doc.flip(nth_seg(rng.next_u64())); break;
			case 7: {
				EndRules r;
				r.left = static_cast<TurnRule>(rng.next_u64() % 3);
				r.right = static_cast<TurnRule>(rng.next_u64() % 3);
				doc.set_end_rules(nth_seg(rng.next_u64()), static_cast<int>(rng.next_u64() % 2), r);
				break;
			}
			case 8: {
				const SegmentId s = nth_seg(rng.next_u64());
				const int lanes = static_cast<int>(doc.map().segment(s)->profile.lanes.size());
				if (lanes > 1) {
					const double a = rng.uniform();
					doc.set_no_change(s, 1 + static_cast<int>(rng.next_u64() % static_cast<uint64_t>(lanes - 1)), a,
							std::min(1.0, a + 0.3), true, rng.uniform() < 0.5);
				}
				break;
			}
			case 9: {
				const NodeId n = nth_node(rng.next_u64());
				doc.set_junction_control(n, static_cast<JunctionControl>(rng.next_u64() % 3), doc.map().segments_at(n));
				break;
			}
			case 10: {
				Spawner sp;
				sp.enabled = rng.uniform() < 0.7;
				sp.rate = rng.range(0, 800);
				sp.sink = rng.uniform() < 0.5;
				doc.set_spawner(nth_node(rng.next_u64()), sp);
				break;
			}
		}
		if (doc.undo_count() > before) ++done;
	}
	CHECK(done == 500);
	g.build(doc.map()); // the edited map still builds
	const RoadMap end = doc.map();
	for (int i = 0; i < done; ++i) CHECK(doc.undo());
	CHECK(doc.map() == start);
	for (int i = 0; i < done; ++i) CHECK(doc.redo());
	CHECK(doc.map() == end);
}

TEST_CASE("demo town: every M1 feature, no errors") {
	Document doc;
	build_demo_town(doc);
	RoadGeometry g;
	g.build(doc.map());
	const std::vector<Problem> ps = validate(doc.map(), g);
	for (const Problem &p : ps) {
		if (p.severity == Severity::Error) std::printf("  demo town: %s\n", p.message.c_str());
	}
	CHECK(count_errors(ps) == 0);
	bool pocket = false, taper = false, bezier = false, zone = false, bus = false, bike = false;
	for (const auto &kv : g.nodes()) taper |= kv.second.kind == NodeKind::Taper;
	for (const auto &kv : g.segments()) {
		for (const GeomLane &l : kv.second.lanes) pocket |= l.pocket_end >= 0;
	}
	for (const auto &kv : doc.map().segments()) {
		bezier |= kv.second.curve == CurveKind::Bezier;
		zone |= !kv.second.no_change.empty();
		for (const LaneSpec &l : kv.second.profile.lanes) {
			bus |= l.type == LaneType::Bus;
			bike |= l.type == LaneType::Bike;
		}
	}
	CHECK(pocket);
	CHECK(taper);
	CHECK(bezier);
	CHECK(zone);
	CHECK(bus);
	CHECK(bike);
}
