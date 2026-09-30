#include "tsim/demo_maps.h"

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
	doc.commit();
}

} // namespace tsim
