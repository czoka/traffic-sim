#include "tsim/road_geometry.h"

#include "clipper2/clipper.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace tsim {

const char *node_kind_name(NodeKind k) {
	switch (k) {
		case NodeKind::Isolated:
			return "isolated";
		case NodeKind::End:
			return "end";
		case NodeKind::Continuation:
			return "continuation";
		case NodeKind::Taper:
			return "taper";
		case NodeKind::Junction:
			return "junction";
	}
	return "isolated";
}

const char *turn_kind_name(TurnKind k) {
	switch (k) {
		case TurnKind::Straight:
			return "straight";
		case TurnKind::Left:
			return "left";
		case TurnKind::Right:
			return "right";
		case TurnKind::UTurn:
			return "uturn";
	}
	return "straight";
}

namespace {

// Schematic palette: flat asphalt, crisp white and yellow paint.
constexpr Color kAsphalt{ 0.227f, 0.235f, 0.251f, 1.0f };
constexpr Color kParking{ 0.286f, 0.294f, 0.314f, 1.0f };
constexpr Color kBus{ 0.478f, 0.235f, 0.227f, 1.0f };
constexpr Color kBike{ 0.235f, 0.408f, 0.290f, 1.0f };
constexpr Color kSidewalk{ 0.620f, 0.639f, 0.604f, 1.0f };
constexpr Color kIsland{ 0.435f, 0.522f, 0.427f, 1.0f };
constexpr Color kWhite{ 0.93f, 0.93f, 0.90f, 1.0f };
constexpr Color kYellow{ 0.91f, 0.77f, 0.28f, 1.0f };

constexpr double kLineWidth = 0.15;
constexpr double kDash = 3.0;
constexpr double kGap = 6.0;
constexpr double kContinuationAngle = 12.0 * kPi / 180.0;

double smooth(double x) {
	x = std::clamp(x, 0.0, 1.0);
	return x * x * (3.0 - 2.0 * x);
}

Vec2 tangent_of(Vec2 n) { return Vec2{ n.y, -n.x }; } // inverse of Vec2::right()

double angle_of(Vec2 d) { return std::atan2(d.y, d.x); }

bool line_intersect(Vec2 p, Vec2 d, Vec2 q, Vec2 e, double &t, double &u) {
	const double den = cross(d, e);
	if (std::fabs(den) < 1e-9) {
		return false;
	}
	const Vec2 w = q - p;
	t = cross(w, e) / den;
	u = cross(w, d) / den;
	return true;
}

Color lane_color(LaneType t) {
	switch (t) {
		case LaneType::Bus:
			return kBus;
		case LaneType::Bike:
			return kBike;
		case LaneType::Parking:
			return kParking;
		case LaneType::Sidewalk:
			return kSidewalk;
		default:
			return kAsphalt;
	}
}

// --- mesh building -------------------------------------------------------------

struct MeshSet {
	std::map<std::pair<int, int>, MeshBatch> batches;

	MeshBatch &get(int level, Layer layer) {
		MeshBatch &b = batches[{ level, static_cast<int>(layer) }];
		b.level = level;
		b.layer = layer;
		return b;
	}
	static void tri(MeshBatch &b, Vec2 a, Vec2 c, Vec2 d, Color col) {
		const int32_t i = static_cast<int32_t>(b.vertices.size());
		b.vertices.push_back(a);
		b.vertices.push_back(c);
		b.vertices.push_back(d);
		b.colors.insert(b.colors.end(), 3, col);
		b.uvs.insert(b.uvs.end(), 3, Vec2{});
		b.indices.insert(b.indices.end(), { i, i + 1, i + 2 });
	}
	static void quad(MeshBatch &b, Vec2 a, Vec2 c, Vec2 d, Vec2 e, Color col) {
		const int32_t i = static_cast<int32_t>(b.vertices.size());
		b.vertices.insert(b.vertices.end(), { a, c, d, e });
		b.colors.insert(b.colors.end(), 4, col);
		b.uvs.insert(b.uvs.end(), 4, Vec2{});
		b.indices.insert(b.indices.end(), { i, i + 1, i + 2, i, i + 2, i + 3 });
	}
	// Strip between two polylines of equal length. `half` (optional) gives
	// each point's normal * half width for line strips.
	static void strip(MeshBatch &b, const std::vector<Vec2> &l, const std::vector<Vec2> &r, Color col,
			const std::vector<Vec2> *half = nullptr) {
		const size_t n = std::min(l.size(), r.size());
		if (n < 2) {
			return;
		}
		const int32_t base = static_cast<int32_t>(b.vertices.size());
		for (size_t k = 0; k < n; ++k) {
			b.vertices.push_back(l[k]);
			b.vertices.push_back(r[k]);
			if (half) {
				b.uvs.push_back((*half)[k] * -1.0);
				b.uvs.push_back((*half)[k]);
			} else {
				b.uvs.push_back(Vec2{});
				b.uvs.push_back(Vec2{});
			}
		}
		b.colors.insert(b.colors.end(), n * 2, col);
		for (size_t k = 0; k + 1 < n; ++k) {
			const int32_t i = base + static_cast<int32_t>(k * 2);
			b.indices.insert(b.indices.end(), { i, i + 1, i + 3, i, i + 3, i + 2 });
		}
	}
	// Thick polyline with explicit per-point normals.
	static void thick(MeshBatch &b, const std::vector<Vec2> &pts, const std::vector<Vec2> &normals, double w,
			Color col) {
		std::vector<Vec2> l, r, half;
		for (size_t k = 0; k < pts.size(); ++k) {
			l.push_back(pts[k] - normals[k] * (0.5 * w));
			r.push_back(pts[k] + normals[k] * (0.5 * w));
			half.push_back(normals[k] * (0.5 * w));
		}
		strip(b, l, r, col, &half);
	}
	// Thick polyline, normals from the path itself.
	static void path(MeshBatch &b, const std::vector<Vec2> &pts, double w, Color col) {
		if (pts.size() < 2) {
			return;
		}
		std::vector<Vec2> normals(pts.size());
		for (size_t k = 0; k < pts.size(); ++k) {
			const Vec2 d = (pts[std::min(k + 1, pts.size() - 1)] - pts[k == 0 ? 0 : k - 1]).normalized();
			normals[k] = d.right();
		}
		thick(b, pts, normals, w, col);
	}
};

// Ear clipping for a simple polygon (any orientation).
void triangulate(const std::vector<Vec2> &poly, MeshBatch &b, Color col) {
	std::vector<Vec2> pts = poly;
	// Drop duplicate consecutive points.
	pts.erase(std::unique(pts.begin(), pts.end(),
					  [](Vec2 a, Vec2 c) { return (a - c).dot(a - c) < 1e-8; }),
			pts.end());
	if (pts.size() >= 2 && (pts.front() - pts.back()).dot(pts.front() - pts.back()) < 1e-8) {
		pts.pop_back();
	}
	if (pts.size() < 3) {
		return;
	}
	double area = 0.0;
	for (size_t i = 0; i < pts.size(); ++i) {
		area += cross(pts[i], pts[(i + 1) % pts.size()]);
	}
	const double sign = area > 0.0 ? 1.0 : -1.0;
	std::vector<size_t> idx(pts.size());
	for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
	size_t guard = 0;
	while (idx.size() > 3 && guard++ < 10000) {
		bool clipped = false;
		for (size_t i = 0; i < idx.size(); ++i) {
			const Vec2 a = pts[idx[(i + idx.size() - 1) % idx.size()]];
			const Vec2 c = pts[idx[i]];
			const Vec2 d = pts[idx[(i + 1) % idx.size()]];
			if (cross(c - a, d - c) * sign <= 1e-12) {
				continue; // reflex or degenerate
			}
			bool inside = false;
			for (size_t k = 0; k < idx.size() && !inside; ++k) {
				const Vec2 q = pts[idx[k]];
				if (q == a || q == c || q == d) continue;
				const double c1 = cross(c - a, q - a) * sign;
				const double c2 = cross(d - c, q - c) * sign;
				const double c3 = cross(a - d, q - d) * sign;
				inside = c1 >= 0 && c2 >= 0 && c3 >= 0;
			}
			if (inside) continue;
			MeshSet::tri(b, a, c, d, col);
			idx.erase(idx.begin() + static_cast<long>(i));
			clipped = true;
			break;
		}
		if (!clipped) {
			// Not simple: fall back to a fan so something is drawn.
			for (size_t i = 1; i + 1 < idx.size(); ++i) {
				MeshSet::tri(b, pts[idx[0]], pts[idx[i]], pts[idx[i + 1]], col);
			}
			return;
		}
	}
	if (idx.size() == 3) {
		MeshSet::tri(b, pts[idx[0]], pts[idx[1]], pts[idx[2]], col);
	}
}

// Cleans a possibly self-intersecting polygon into simple outer rings.
std::vector<std::vector<Vec2>> clean_polygon(const std::vector<Vec2> &poly) {
	Clipper2Lib::PathsD in(1);
	for (const Vec2 &v : poly) {
		in[0].push_back(Clipper2Lib::PointD(v.x, v.y));
	}
	const Clipper2Lib::PathsD out = Clipper2Lib::Union(in, Clipper2Lib::FillRule::NonZero, 3);
	std::vector<std::vector<Vec2>> rings;
	for (const Clipper2Lib::PathD &p : out) {
		if (Clipper2Lib::Area(p) == 0.0) continue;
		std::vector<Vec2> ring;
		for (const Clipper2Lib::PointD &q : p) {
			ring.push_back(Vec2{ q.x, q.y });
		}
		rings.push_back(std::move(ring));
	}
	return rings;
}

std::vector<Vec2> quad_bezier(Vec2 a, Vec2 c, Vec2 b, int n) {
	std::vector<Vec2> out;
	for (int i = 0; i <= n; ++i) {
		const double t = static_cast<double>(i) / n;
		const double u = 1.0 - t;
		out.push_back(a * (u * u) + c * (2.0 * u * t) + b * (t * t));
	}
	return out;
}

// Arrow glyphs in a local frame (x = right of travel, y = along travel).
struct Glyph {
	std::vector<Vec2> shaft;
	Vec2 tip;
	Vec2 dir;
};

Glyph arrow_glyph(TurnKind k) {
	switch (k) {
		case TurnKind::Right:
			return { { { 0.0, -3.0 }, { 0.0, -0.6 }, { 0.8, 0.5 } }, { 1.15, 1.0 }, Vec2{ 0.6, 0.8 }.normalized() };
		case TurnKind::Left:
			return { { { 0.0, -3.0 }, { 0.0, -0.6 }, { -0.8, 0.5 } }, { -1.15, 1.0 }, Vec2{ -0.6, 0.8 }.normalized() };
		default:
			return { { { 0.0, -3.0 }, { 0.0, 1.0 } }, { 0.0, 2.2 }, Vec2{ 0.0, 1.0 } };
	}
}

void emit_glyph(MeshBatch &b, const Glyph &g, Vec2 origin, Vec2 fwd, Color col) {
	const Vec2 right = fwd.right();
	auto w = [&](Vec2 l) { return origin + right * l.x + fwd * l.y; };
	std::vector<Vec2> pts;
	for (const Vec2 &l : g.shaft) pts.push_back(w(l));
	MeshSet::path(b, pts, 0.3, col);
	const Vec2 d = right * g.dir.x + fwd * g.dir.y;
	const Vec2 tip = w(g.tip);
	const Vec2 base = tip - d * 1.3;
	MeshSet::tri(b, tip, base + d.right() * 0.65, base - d.right() * 0.65, col);
}

} // namespace

// ---------------------------------------------------------------------------------

struct RoadGeometry::SegCtx {
	const RoadSegment *seg = nullptr;
	Curve curve;
	ArcTable table;
	double length = 0.0;
	std::vector<GeomLane> lanes;
	bool two_way = false;
	size_t right_start = 0;
	double median = 0.0;
	MedianType median_type = MedianType::None;
	NodeKind end_kind[2] = { NodeKind::End, NodeKind::End };
	bool stop_at[2] = { false, false }; // a real junction (3+ legs) with stop lines
	bool blend[2] = { false, false };
	std::vector<double> target[2];
	double target_median[2] = { 0.0, 0.0 };
	double ramp[2] = { 0.0, 0.0 };
	double trim[2] = { 0.0, 0.0 };
	double pocket_len[2] = { 0.0, 0.0 };
	double pocket_taper = 15.0;

	void recompute_groups() {
		bool fwd = false, back = false;
		for (const GeomLane &l : lanes) {
			if (is_directional(l.type) && l.dir == LaneDir::Forward) fwd = true;
			if (is_directional(l.type) && l.dir == LaneDir::Backward) back = true;
		}
		two_way = fwd && back;
		right_start = lanes.size();
		for (size_t i = 0; i < lanes.size(); ++i) {
			if (is_directional(lanes[i].type) && lanes[i].dir == LaneDir::Forward) {
				right_start = i;
				break;
			}
		}
	}

	void frame(double s, Vec2 &p, Vec2 &n) const {
		const double t = table.t_at(std::clamp(s, 0.0, length));
		p = curve.point(t);
		n = curve.tangent(t).right();
	}

	double width_of(size_t i, double s) const {
		const GeomLane &l = lanes[i];
		if (l.pocket_end >= 0) {
			const int e = l.pocket_end;
			const double d = e == 1 ? length - s : s;
			const double full_end = trim[e] + pocket_len[e];
			if (d >= full_end) return 0.0;
			if (d <= full_end - pocket_taper) return l.base_width;
			return l.base_width * smooth((full_end - d) / pocket_taper);
		}
		double w = l.base_width;
		for (int e = 0; e < 2; ++e) {
			if (!blend[e] || ramp[e] <= 0.0) continue;
			const double d = e == 1 ? length - s : s;
			if (d < ramp[e]) {
				const double tw = target[e][static_cast<size_t>(l.profile_index)];
				w = tw + (w - tw) * smooth(d / ramp[e]);
			}
		}
		return w;
	}

	double median_at(double s) const {
		double m = median;
		for (int e = 0; e < 2; ++e) {
			if (!blend[e] || ramp[e] <= 0.0) continue;
			const double d = e == 1 ? length - s : s;
			if (d < ramp[e]) {
				m = target_median[e] + (m - target_median[e]) * smooth(d / ramp[e]);
			}
		}
		return two_way ? m : 0.0;
	}

	// Lateral {left, right} offsets of every lane at s.
	void cross_section(double s, std::vector<std::pair<double, double>> &out) const {
		const size_t n = lanes.size();
		out.assign(n, { 0.0, 0.0 });
		std::vector<double> w(n);
		for (size_t i = 0; i < n; ++i) w[i] = width_of(i, s);
		if (two_way) {
			const double mw = median_at(s);
			double x = -0.5 * mw;
			for (size_t i = right_start; i-- > 0;) {
				out[i] = { x - w[i], x };
				x -= w[i];
			}
			x = 0.5 * mw;
			for (size_t i = right_start; i < n; ++i) {
				out[i] = { x, x + w[i] };
				x += w[i];
			}
		} else {
			double nominal = 0.0;
			for (size_t i = 0; i < n; ++i) {
				if (lanes[i].pocket_end < 0) nominal += w[i];
			}
			double x = -0.5 * nominal;
			for (size_t i = 0; i < n; ++i) {
				out[i] = { x, x + w[i] };
				x += w[i];
			}
		}
	}

	// Lanes travelling towards end e (incoming) or away from it.
	bool arrives_at(const GeomLane &l, int e) const {
		return is_directional(l.type) && l.dir == (e == 1 ? LaneDir::Forward : LaneDir::Backward);
	}
	bool leaves_from(const GeomLane &l, int e) const {
		return is_directional(l.type) && l.dir == (e == 0 ? LaneDir::Forward : LaneDir::Backward);
	}
};

const SegmentGeom *RoadGeometry::segment(SegmentId id) const {
	auto it = segments_.find(id);
	return it == segments_.end() ? nullptr : &it->second;
}

const NodeGeom *RoadGeometry::node(NodeId id) const {
	auto it = nodes_.find(id);
	return it == nodes_.end() ? nullptr : &it->second;
}

void RoadGeometry::clear() {
	segments_.clear();
	nodes_.clear();
	meshes_.clear();
}

std::vector<Vec2> SegmentGeom::outline() const {
	std::vector<Vec2> left, right;
	const size_t nl = lanes.size();
	if (nl == 0) return {};
	for (size_t k = 0; k < s.size(); ++k) {
		double lo = 1e300, hi = -1e300;
		for (size_t i = 0; i < nl; ++i) {
			lo = std::min(lo, edge(k, i).first);
			hi = std::max(hi, edge(k, i).second);
		}
		left.push_back(at(k, lo));
		right.push_back(at(k, hi));
	}
	std::reverse(right.begin(), right.end());
	left.insert(left.end(), right.begin(), right.end());
	return left;
}

std::vector<Vec2> SegmentGeom::carriageway() const {
	std::vector<Vec2> left, right;
	const size_t nl = lanes.size();
	for (size_t k = 0; k < s.size(); ++k) {
		double lo = 1e300, hi = -1e300;
		for (size_t i = 0; i < nl; ++i) {
			if (lanes[i].type == LaneType::Sidewalk) continue;
			lo = std::min(lo, edge(k, i).first);
			hi = std::max(hi, edge(k, i).second);
		}
		if (lo > hi) return {};
		left.push_back(at(k, lo));
		right.push_back(at(k, hi));
	}
	std::reverse(right.begin(), right.end());
	left.insert(left.end(), right.begin(), right.end());
	return left;
}

void RoadGeometry::build(const RoadMap &map) {
	clear();
	MeshSet mesh;
	std::map<SegmentId, SegCtx> ctx;

	// --- 1. Segment contexts --------------------------------------------------
	for (const auto &kv : map.segments()) {
		const RoadSegment &s = kv.second;
		SegCtx &c = ctx[s.id];
		c.seg = &s;
		c.curve = map.curve_of(s);
		c.table.build(c.curve);
		c.length = c.table.length();
		c.median = s.profile.median == MedianType::None ? 0.0 : s.profile.median_width;
		c.median_type = s.profile.median;
		c.pocket_taper = pocket_taper;
		for (size_t i = 0; i < s.profile.lanes.size(); ++i) {
			const LaneSpec &l = s.profile.lanes[i];
			GeomLane g;
			g.id = l.id;
			g.type = l.type;
			g.dir = l.dir;
			g.base_width = l.width;
			g.profile_index = static_cast<int>(i);
			c.lanes.push_back(g);
		}
		c.recompute_groups();
	}

	// --- 2. Node kinds and legs -------------------------------------------------
	for (const auto &kv : map.nodes()) {
		const RoadNode &n = kv.second;
		NodeGeom &g = nodes_[n.id];
		g.id = n.id;
		g.level = n.level;
		g.pos = n.pos;
		for (SegmentId sid : map.segments_at(n.id)) {
			const SegCtx &c = ctx[sid];
			Leg leg;
			leg.seg = sid;
			leg.at_start = c.seg->from == n.id;
			leg.dir = leg.at_start ? c.curve.tangent(0.0) : c.curve.tangent(1.0) * -1.0;
			g.legs.push_back(leg);
		}
		std::sort(g.legs.begin(), g.legs.end(),
				[](const Leg &a, const Leg &b) { return angle_of(a.dir) < angle_of(b.dir); });
		const size_t deg = g.legs.size();
		if (deg == 0) {
			g.kind = NodeKind::Isolated;
		} else if (deg == 1) {
			g.kind = NodeKind::End;
		} else if (deg == 2) {
			const double dev = kPi - std::acos(std::clamp(g.legs[0].dir.dot(g.legs[1].dir), -1.0, 1.0));
			g.kind = dev <= kContinuationAngle ? NodeKind::Continuation : NodeKind::Junction;
		} else {
			g.kind = NodeKind::Junction;
		}
		for (const Leg &leg : g.legs) {
			SegCtx &c = ctx[leg.seg];
			c.end_kind[leg.at_start ? 0 : 1] = g.kind;
			c.stop_at[leg.at_start ? 0 : 1] = g.kind == NodeKind::Junction && deg >= 3;
		}
	}

	// Turn kind from leg j into leg k (the same rule the connectors use).
	auto turn_between = [](const NodeGeom &g, size_t j, size_t k) {
		const Vec2 in_dir = g.legs[j].dir * -1.0;
		const double ang = std::atan2(cross(in_dir, g.legs[k].dir), in_dir.dot(g.legs[k].dir));
		if (g.legs.size() == 2 || std::fabs(ang) <= 0.61) return TurnKind::Straight;
		if (std::fabs(ang) > 2.8) return TurnKind::UTurn;
		return ang > 0 ? TurnKind::Right : TurnKind::Left;
	};
	// Whether any travel lane leaves the node along this leg.
	auto has_outgoing = [&](const Leg &leg) {
		const LaneDir d = leg.at_start ? LaneDir::Forward : LaneDir::Backward;
		for (const LaneSpec &l : ctx[leg.seg].seg->profile.lanes) {
			if (is_travel(l.type) && l.dir == d) return true;
		}
		return false;
	};

	// --- 3. Turn pockets at junction approaches ----------------------------------
	// Only where the turn exists: a junction of 3+ legs with a road to turn into.
	for (auto &kv : ctx) {
		SegCtx &c = kv.second;
		for (int e = 0; e < 2; ++e) {
			if (c.end_kind[e] != NodeKind::Junction) continue;
			const NodeGeom &ng = nodes_[e == 0 ? c.seg->from : c.seg->to];
			if (ng.legs.size() < 3) continue;
			size_t self = 0;
			while (self < ng.legs.size() &&
					!(ng.legs[self].seg == c.seg->id && ng.legs[self].at_start == (e == 0))) {
				++self;
			}
			bool can_left = false, can_right = false;
			for (size_t k = 0; k < ng.legs.size(); ++k) {
				if (k == self || !has_outgoing(ng.legs[k])) continue;
				const TurnKind tk = turn_between(ng, self, k);
				can_left |= tk == TurnKind::Left;
				can_right |= tk == TurnKind::Right;
			}
			EndRules r = c.seg->ends[e];
			if (!can_left && r.left == TurnRule::TurnLane) r.left = TurnRule::Allowed;
			if (!can_right && r.right == TurnRule::TurnLane) r.right = TurnRule::Allowed;
			const LaneDir approach = e == 1 ? LaneDir::Forward : LaneDir::Backward;
			int first = -1, last = -1;
			for (size_t i = 0; i < c.lanes.size(); ++i) {
				if (is_travel(c.lanes[i].type) && c.lanes[i].dir == approach && c.lanes[i].pocket_end < 0) {
					if (first < 0) first = static_cast<int>(i);
					last = static_cast<int>(i);
				}
			}
			if (first < 0) continue;
			c.pocket_len[e] = r.turn_lane_length;
			auto make = [&](bool left, LaneId id) {
				GeomLane g;
				g.id = id;
				g.type = LaneType::Turn;
				g.dir = approach;
				g.base_width = default_width(LaneType::Turn);
				g.pocket_end = e;
				g.pocket_left = left;
				return g;
			};
			// Forward lanes travel left-to-right-of-list = their left is list-left.
			// Backward lanes are mirrored.
			const bool fwd = approach == LaneDir::Forward;
			if (r.right == TurnRule::TurnLane) {
				const size_t pos = fwd ? static_cast<size_t>(last) + 1 : static_cast<size_t>(first);
				c.lanes.insert(c.lanes.begin() + static_cast<long>(pos), make(false, r.right_lane));
				if (!fwd) ++first, ++last;
			}
			if (r.left == TurnRule::TurnLane) {
				const size_t pos = fwd ? static_cast<size_t>(first) : static_cast<size_t>(last) + 1;
				c.lanes.insert(c.lanes.begin() + static_cast<long>(pos), make(true, r.left_lane));
			}
			c.recompute_groups();
		}
	}

	// --- 4. Continuations and tapers: blend cross-sections -------------------------
	for (auto &kv : nodes_) {
		NodeGeom &g = kv.second;
		if (g.kind != NodeKind::Continuation) continue;
		bool taper = false;
		for (int side = 0; side < 2; ++side) {
			const Leg &la = g.legs[static_cast<size_t>(side)];
			const Leg &lb = g.legs[static_cast<size_t>(1 - side)];
			SegCtx &a = ctx[la.seg];
			const SegCtx &b = ctx[lb.seg];
			const int ea = la.at_start ? 0 : 1;
			const bool same = la.at_start != lb.at_start;
			const std::vector<LaneRole> ra = profile_roles(a.seg->profile);
			std::vector<LaneRole> rb = profile_roles(b.seg->profile);
			if (!same) {
				for (LaneRole &r : rb) r.side = 1 - r.side;
			}
			a.target[ea].assign(a.seg->profile.lanes.size(), 0.0);
			for (size_t i = 0; i < ra.size(); ++i) {
				auto it = std::find(rb.begin(), rb.end(), ra[i]);
				if (it != rb.end()) {
					const double wb = b.seg->profile.lanes[static_cast<size_t>(it - rb.begin())].width;
					a.target[ea][i] = 0.5 * (a.seg->profile.lanes[i].width + wb);
				} else {
					taper = true;
				}
			}
			const double mb = b.two_way ? b.median : 0.0;
			a.target_median[ea] = 0.5 * ((a.two_way ? a.median : 0.0) + mb);
			a.ramp[ea] = std::min(taper_length, 0.4 * a.length);
			a.blend[ea] = true;
		}
		if (taper) g.kind = NodeKind::Taper;
		for (const Leg &leg : g.legs) ctx[leg.seg].end_kind[leg.at_start ? 0 : 1] = g.kind;
	}

	// --- 5. Junction legs and trims -------------------------------------------------
	std::vector<std::pair<double, double>> xs;
	auto leg_widths = [&](Leg &leg) {
		const SegCtx &c = ctx[leg.seg];
		c.cross_section(leg.at_start ? 0.0 : c.length, xs);
		double clo = 1e300, chi = -1e300, olo = 1e300, ohi = -1e300;
		for (size_t i = 0; i < xs.size(); ++i) {
			olo = std::min(olo, xs[i].first);
			ohi = std::max(ohi, xs[i].second);
			if (c.lanes[i].type != LaneType::Sidewalk) {
				clo = std::min(clo, xs[i].first);
				chi = std::max(chi, xs[i].second);
			}
		}
		if (clo > chi) clo = chi = 0.0;
		if (olo > ohi) olo = ohi = 0.0;
		if (leg.at_start) {
			leg.kerb_left = -clo;
			leg.kerb_right = chi;
			leg.outer_left = -olo;
			leg.outer_right = ohi;
		} else {
			leg.kerb_left = chi;
			leg.kerb_right = -clo;
			leg.outer_left = ohi;
			leg.outer_right = -olo;
		}
	};
	for (auto &kv : nodes_) {
		NodeGeom &g = kv.second;
		for (Leg &leg : g.legs) leg_widths(leg);
		if (g.kind != NodeKind::Junction) continue;
		const size_t deg = g.legs.size();
		for (size_t i = 0; i < deg; ++i) {
			Leg &a = g.legs[i];
			Leg &b = g.legs[(i + 1) % deg];
			double theta = angle_of(b.dir) - angle_of(a.dir);
			while (theta <= 0.0) theta += 2.0 * kPi;
			if (theta >= kPi - 0.17) continue; // nearly straight or reflex: no constraint
			const Vec2 pa = g.pos + a.dir.right() * a.kerb_right;
			const Vec2 pb = g.pos - b.dir.right() * b.kerb_left;
			double t = 0.0, u = 0.0;
			if (!line_intersect(pa, a.dir, pb, b.dir, t, u)) continue;
			const double fillet = curb_radius / std::tan(0.5 * theta);
			a.trim = std::max(a.trim, t + fillet);
			b.trim = std::max(b.trim, u + fillet);
		}
		for (Leg &leg : g.legs) {
			const SegCtx &c = ctx[leg.seg];
			leg.trim = std::clamp(leg.trim, 1.0, 0.45 * c.length);
			ctx[leg.seg].trim[leg.at_start ? 0 : 1] = leg.trim;
		}
	}

	// --- 6. Segments: samples, lanes, paint ------------------------------------------
	for (auto &kv : ctx) {
		SegCtx &c = kv.second;
		const RoadSegment &seg = *c.seg;
		SegmentGeom &sg = segments_[seg.id];
		sg.id = seg.id;
		sg.level = seg.level;
		sg.curve = c.curve;
		sg.length = c.length;
		sg.trim[0] = c.trim[0];
		sg.trim[1] = c.trim[1];
		sg.lanes = c.lanes;
		const double s0 = c.trim[0];
		const double s1 = std::max(s0, c.length - c.trim[1]);
		const double span = s1 - s0;
		const int steps = std::clamp(static_cast<int>(std::ceil(span / 2.0)), 1, 1500);
		const size_t nl = c.lanes.size();
		for (int k = 0; k <= steps; ++k) {
			const double s = s0 + span * (static_cast<double>(k) / steps);
			Vec2 p, nrm;
			c.frame(s, p, nrm);
			sg.s.push_back(s);
			sg.p.push_back(p);
			sg.n.push_back(nrm);
			c.cross_section(s, xs);
			sg.x.insert(sg.x.end(), xs.begin(), xs.end());
		}
		sg.bb_min = Vec2{ 1e300, 1e300 };
		sg.bb_max = Vec2{ -1e300, -1e300 };
		for (const Vec2 &q : sg.outline()) {
			sg.bb_min = Vec2{ std::min(sg.bb_min.x, q.x), std::min(sg.bb_min.y, q.y) };
			sg.bb_max = Vec2{ std::max(sg.bb_max.x, q.x), std::max(sg.bb_max.y, q.y) };
		}
		if (span < 0.05 || nl == 0) continue;

		const size_t ns = sg.s.size();
		// Lane surfaces.
		for (size_t i = 0; i < nl; ++i) {
			const GeomLane &l = c.lanes[i];
			const Layer layer = l.type == LaneType::Sidewalk ? Layer::Ground : Layer::Asphalt;
			MeshBatch &b = mesh.get(seg.level, layer);
			std::vector<Vec2> left, right;
			for (size_t k = 0; k < ns; ++k) {
				const auto &e = sg.edge(k, i);
				if (e.second - e.first < 1e-3) {
					// Flush a visible run.
					MeshSet::strip(b, left, right, lane_color(l.type));
					left.clear();
					right.clear();
					continue;
				}
				left.push_back(sg.at(k, e.first));
				right.push_back(sg.at(k, e.second));
			}
			MeshSet::strip(b, left, right, lane_color(l.type));
		}

		// Median.
		if (c.two_way && c.median_type != MedianType::None && c.right_start > 0 && c.right_start < nl) {
			std::vector<Vec2> ml, mr;
			for (size_t k = 0; k < ns; ++k) {
				ml.push_back(sg.at(k, sg.edge(k, c.right_start - 1).second));
				mr.push_back(sg.at(k, sg.edge(k, c.right_start).first));
			}
			if (c.median_type == MedianType::Raised) {
				MeshSet::strip(mesh.get(seg.level, Layer::Ground), ml, mr, kIsland);
				MeshSet::thick(mesh.get(seg.level, Layer::Markings), ml, sg.n, 0.2, kWhite);
				MeshSet::thick(mesh.get(seg.level, Layer::Markings), mr, sg.n, 0.2, kWhite);
			} else {
				MeshSet::strip(mesh.get(seg.level, Layer::Asphalt), ml, mr, kAsphalt);
				MeshSet::thick(mesh.get(seg.level, Layer::Markings), ml, sg.n, kLineWidth, kYellow);
				MeshSet::thick(mesh.get(seg.level, Layer::Markings), mr, sg.n, kLineWidth, kYellow);
				// Hatching every 4 m.
				MeshBatch &mb = mesh.get(seg.level, Layer::Markings);
				for (size_t k = 0; k + 1 < ns; ++k) {
					const double sa = sg.s[k];
					if (std::fmod(sa, 4.0) > 2.0) continue;
					MeshSet::path(mb, { ml[k], mr[k + 1] }, 0.2, kWhite);
				}
			}
		}

		// Lane lines. Boundary b sits between lane b-1 and lane b.
		auto boundary_at = [&](size_t k, size_t b) { return sg.edge(k, b).first; };
		MeshBatch &paint = mesh.get(seg.level, Layer::Markings);
		auto pocket_visible = [&](size_t i, double s) {
			return c.lanes[i].pocket_end < 0 || c.width_of(i, s) > 0.05;
		};
		enum Style { kNone, kDashed, kSolid, kSolidDashed, kDashedSolid };
		for (size_t bi = 1; bi < nl; ++bi) {
			if (c.two_way && bi == c.right_start && c.median_type != MedianType::None) continue;
			const bool center = c.two_way && bi == c.right_start;
			auto style_at = [&](double s) -> std::pair<Style, Color> {
				// A hidden pocket collapses: the boundary on its left takes the
				// style between its neighbours, the one on its right draws nothing.
				if (!pocket_visible(bi - 1, s)) return { kNone, kWhite };
				size_t bj = bi;
				while (bj < nl && !pocket_visible(bj, s)) ++bj;
				if (bj >= nl) return { kNone, kWhite };
				const GeomLane &A = c.lanes[bi - 1];
				const GeomLane &B = c.lanes[bj];
				const bool is_center = center || (c.two_way && bj == c.right_start && bi < c.right_start);
				if (A.type == LaneType::Sidewalk || B.type == LaneType::Sidewalk) return { kNone, kWhite };
				if (is_center) {
					Style st = kDashed;
					for (int e = 0; e < 2; ++e) {
						if (!c.stop_at[e]) continue;
						const double d = e == 1 ? c.length - c.trim[1] - s : s - c.trim[0];
						if (d < solid_before_stop) st = kSolid;
					}
					return { st, kYellow };
				}
				if (A.type == LaneType::Parking || B.type == LaneType::Parking) {
					return { A.type == B.type ? kNone : kSolid, kWhite };
				}
				if (A.pocket_end >= 0 || B.pocket_end >= 0) return { kSolid, kWhite };
				if (A.type == LaneType::Bike || B.type == LaneType::Bike) return { kSolid, kWhite };
				Style st = kDashed;
				const Color col = center ? kYellow : kWhite;
				// Solid near stop lines of the end these lanes drive towards.
				for (int e = 0; e < 2; ++e) {
					if (!c.stop_at[e]) continue;
					const bool towards = center || c.arrives_at(A, e) || c.arrives_at(B, e);
					const double d = e == 1 ? c.length - c.trim[1] - s : s - c.trim[0];
					if (towards && d < solid_before_stop) st = kSolid;
				}
				// Painted zones between two profile lanes.
				if (A.profile_index >= 0 && B.profile_index == A.profile_index + 1) {
					const double u = c.length > 0 ? s / c.length : 0.0;
					for (const NoChangeZone &z : seg.no_change) {
						if (z.edge != B.profile_index || u < z.u0 || u > z.u1) continue;
						if (z.block_left_to_right && z.block_right_to_left) st = kSolid;
						else if (st != kSolid) st = z.block_left_to_right ? kSolidDashed : kDashedSolid;
					}
				}
				return { st, col };
			};
			// Break points where the style may change.
			std::set<double> cuts = { s0, s1 };
			for (int e = 0; e < 2; ++e) {
				const double d = e == 1 ? s1 - solid_before_stop : s0 + solid_before_stop;
				cuts.insert(d);
				const double pe = e == 1 ? c.length - c.trim[1] - c.pocket_len[1] + 0.5 * pocket_taper
										 : c.trim[0] + c.pocket_len[0] - 0.5 * pocket_taper;
				cuts.insert(pe);
			}
			for (const NoChangeZone &z : seg.no_change) {
				cuts.insert(z.u0 * c.length);
				cuts.insert(z.u1 * c.length);
			}
			std::vector<double> cv;
			for (double v : cuts) {
				if (v >= s0 && v <= s1) cv.push_back(v);
			}
			// Point on the boundary at s (linear between samples).
			auto point_at = [&](double s, double extra, Vec2 &out_p, Vec2 &out_n) {
				const double f = span > 0 ? (s - s0) / span * steps : 0.0;
				const size_t k = std::min(static_cast<size_t>(std::max(0.0, std::floor(f))), ns - 2);
				const double a = std::clamp(f - static_cast<double>(k), 0.0, 1.0);
				const double off = boundary_at(k, bi) + (boundary_at(k + 1, bi) - boundary_at(k, bi)) * a + extra;
				const Vec2 p0 = sg.at(k, off);
				const Vec2 p1 = sg.at(k + 1, off);
				out_p = p0 + (p1 - p0) * a;
				out_n = (sg.n[k] + (sg.n[k + 1] - sg.n[k]) * a).normalized();
			};
			auto emit_run = [&](double a, double b, double extra, bool dashed, Color col) {
				if (b - a < 0.05) return;
				auto emit_piece = [&](double pa, double pb) {
					std::vector<Vec2> pts, nrm;
					Vec2 q, qn;
					point_at(pa, extra, q, qn);
					pts.push_back(q);
					nrm.push_back(qn);
					for (size_t k = 0; k < ns; ++k) {
						if (sg.s[k] > pa && sg.s[k] < pb) {
							point_at(sg.s[k], extra, q, qn);
							pts.push_back(q);
							nrm.push_back(qn);
						}
					}
					point_at(pb, extra, q, qn);
					pts.push_back(q);
					nrm.push_back(qn);
					MeshSet::thick(paint, pts, nrm, kLineWidth, col);
				};
				if (!dashed) {
					emit_piece(a, b);
					return;
				}
				const double period = kDash + kGap;
				double start = std::floor(a / period) * period;
				for (double d = start; d < b; d += period) {
					const double lo = std::max(a, d);
					const double hi = std::min(b, d + kDash);
					if (hi - lo > 0.05) emit_piece(lo, hi);
				}
			};
			for (size_t r = 0; r + 1 < cv.size(); ++r) {
				const double a = cv[r], b = cv[r + 1];
				const auto st = style_at(0.5 * (a + b));
				switch (st.first) {
					case kNone:
						break;
					case kDashed:
						emit_run(a, b, 0.0, true, st.second);
						break;
					case kSolid:
						if (center) {
							emit_run(a, b, -0.12, false, st.second);
							emit_run(a, b, 0.12, false, st.second);
						} else {
							emit_run(a, b, 0.0, false, st.second);
						}
						break;
					case kSolidDashed:
						emit_run(a, b, -0.12, false, st.second);
						emit_run(a, b, 0.12, true, st.second);
						break;
					case kDashedSolid:
						emit_run(a, b, -0.12, true, st.second);
						emit_run(a, b, 0.12, false, st.second);
						break;
				}
			}
		}
		// Outer edge lines where a travel lane is outermost.
		for (int side = 0; side < 2; ++side) {
			const size_t i = side == 0 ? 0 : nl - 1;
			if (!is_directional(c.lanes[i].type)) continue;
			std::vector<Vec2> pts;
			for (size_t k = 0; k < ns; ++k) {
				const auto &e = sg.edge(k, i);
				pts.push_back(sg.at(k, side == 0 ? e.first + 0.3 : e.second - 0.3));
			}
			MeshSet::thick(paint, pts, sg.n, kLineWidth, kWhite);
		}
		// Parking bay ticks every 6 m.
		for (size_t i = 0; i < nl; ++i) {
			if (c.lanes[i].type != LaneType::Parking) continue;
			for (size_t k = 0; k < ns; ++k) {
				if (std::fmod(sg.s[k], 6.0) >= 2.0) continue;
				const auto &e = sg.edge(k, i);
				MeshSet::path(paint, { sg.at(k, e.first), sg.at(k, e.second) }, 0.12, kWhite);
			}
		}
	}

	// --- 7. Nodes: junction surfaces, stop lines, connectors, arrows ------------------
	auto leg_point = [&](const Leg &leg, double lateral_leg, double extra_back = 0.0) {
		const SegCtx &c = ctx[leg.seg];
		const double s = leg.at_start ? leg.trim + extra_back : c.length - leg.trim - extra_back;
		Vec2 p, nrm;
		c.frame(s, p, nrm);
		return p + nrm * (leg.at_start ? lateral_leg : -lateral_leg);
	};
	// Lanes of a leg that arrive at / leave from the node, with lateral centres
	// in the leg frame (positive = leg-right), sorted right -> left of travel.
	struct LegLane {
		size_t index;
		LaneId id;
		LaneType type;
		bool pocket;
		bool pocket_left;
		double lateral; // leg frame
		Vec2 point;
		Vec2 dir; // travel direction
	};
	auto leg_lanes = [&](const Leg &leg, bool incoming) {
		const SegCtx &c = ctx[leg.seg];
		const int e = leg.at_start ? 0 : 1;
		const double s = leg.at_start ? leg.trim : c.length - leg.trim;
		c.cross_section(s, xs);
		Vec2 p, nrm;
		c.frame(s, p, nrm);
		const Vec2 tan = tangent_of(nrm);
		std::vector<LegLane> out;
		for (size_t i = 0; i < c.lanes.size(); ++i) {
			const GeomLane &l = c.lanes[i];
			const bool ok = incoming ? c.arrives_at(l, e) : c.leaves_from(l, e);
			if (!ok) continue;
			const double mid = 0.5 * (xs[i].first + xs[i].second);
			LegLane ll;
			ll.index = i;
			ll.id = l.id;
			ll.type = l.type;
			ll.pocket = l.pocket_end >= 0;
			ll.pocket_left = l.pocket_left;
			ll.lateral = leg.at_start ? mid : -mid;
			ll.point = p + nrm * mid;
			ll.dir = (l.dir == LaneDir::Forward) ? tan : tan * -1.0;
			out.push_back(ll);
		}
		// Incoming lanes travel towards the node (against the leg direction), so
		// their right side is the leg's left: sort by leg lateral ascending.
		// Outgoing lanes travel along the leg: their right is leg-right.
		std::sort(out.begin(), out.end(), [incoming](const LegLane &a, const LegLane &b) {
			return incoming ? a.lateral < b.lateral : a.lateral > b.lateral;
		});
		return out;
	};
	auto connector_path = [](Vec2 a, Vec2 da, Vec2 b, Vec2 db) {
		const double k = std::max(1.0, (b - a).length() / 3.0);
		Curve cv;
		cv.kind = CurveKind::Bezier;
		cv.p0 = a;
		cv.c1 = a + da * k;
		cv.c2 = b - db * k;
		cv.p3 = b;
		std::vector<Vec2> pts;
		for (int i = 0; i <= 12; ++i) pts.push_back(cv.point(i / 12.0));
		return pts;
	};

	for (auto &kv : nodes_) {
		NodeGeom &g = kv.second;
		const size_t deg = g.legs.size();
		if (g.kind == NodeKind::End) {
			for (const LegLane &ll : leg_lanes(g.legs[0], true)) {
				if (is_travel(ll.type)) g.dead_lanes.push_back({ g.legs[0].seg, ll.id });
			}
			continue;
		}
		if (g.kind == NodeKind::Continuation || g.kind == NodeKind::Taper) {
			// Joint filler over the small wedge between the two ends.
			const Leg &la = g.legs[0];
			const Leg &lb = g.legs[1];
			const SegCtx &a = ctx[la.seg];
			const SegCtx &b = ctx[lb.seg];
			std::vector<std::pair<double, double>> xa, xb;
			a.cross_section(la.at_start ? 0.0 : a.length, xa);
			b.cross_section(lb.at_start ? 0.0 : b.length, xb);
			Vec2 pa, na, pb, nb;
			a.frame(la.at_start ? 0.0 : a.length, pa, na);
			b.frame(lb.at_start ? 0.0 : b.length, pb, nb);
			const bool same = la.at_start != lb.at_start;
			const std::vector<LaneRole> ra = profile_roles(a.seg->profile);
			std::vector<LaneRole> rb = profile_roles(b.seg->profile);
			if (!same) for (LaneRole &r : rb) r.side = 1 - r.side;
			for (size_t i = 0; i < a.lanes.size(); ++i) {
				if (a.lanes[i].pocket_end >= 0) continue;
				const auto it = std::find(rb.begin(), rb.end(), ra[static_cast<size_t>(a.lanes[i].profile_index)]);
				if (it == rb.end()) continue;
				const int pj = static_cast<int>(it - rb.begin());
				size_t j = 0;
				while (j < b.lanes.size() && b.lanes[j].profile_index != pj) ++j;
				if (j >= b.lanes.size()) continue;
				const Vec2 a0 = pa + na * xa[i].first, a1 = pa + na * xa[i].second;
				Vec2 b0 = pb + nb * xb[j].first, b1 = pb + nb * xb[j].second;
				if (!same) std::swap(b0, b1);
				const Layer layer = a.lanes[i].type == LaneType::Sidewalk ? Layer::Ground : Layer::Asphalt;
				MeshSet::quad(mesh.get(g.level, layer), a0, a1, b1, b0, lane_color(a.lanes[i].type));
			}
			// Connectors through the joint, aligned from the median side.
			for (int side = 0; side < 2; ++side) {
				const Leg &from = g.legs[static_cast<size_t>(side)];
				const Leg &to = g.legs[static_cast<size_t>(1 - side)];
				std::vector<LegLane> in = leg_lanes(from, true);
				std::vector<LegLane> out = leg_lanes(to, false);
				in.erase(std::remove_if(in.begin(), in.end(), [](const LegLane &l) { return !is_travel(l.type); }),
						in.end());
				out.erase(std::remove_if(out.begin(), out.end(), [](const LegLane &l) { return !is_travel(l.type); }),
						out.end());
				// Sorted right->left: reverse to count from the median.
				std::reverse(in.begin(), in.end());
				std::reverse(out.begin(), out.end());
				if (out.empty()) {
					for (const LegLane &l : in) g.dead_lanes.push_back({ from.seg, l.id });
					continue;
				}
				for (size_t i = 0; i < std::max(in.size(), out.size()); ++i) {
					if (in.empty()) break;
					const LegLane &li = in[std::min(i, in.size() - 1)];
					const LegLane &lo = out[std::min(i, out.size() - 1)];
					Connector cn;
					cn.from_seg = from.seg;
					cn.from_lane = li.id;
					cn.to_seg = to.seg;
					cn.to_lane = lo.id;
					cn.turn = TurnKind::Straight;
					cn.path = { li.point, lo.point };
					g.connectors.push_back(cn);
				}
				// Merge arrows in lanes that end at this taper.
				if (in.size() > out.size()) {
					const SegCtx &c = ctx[from.seg];
					for (size_t i = out.size(); i < in.size(); ++i) {
						const double back = c.ramp[from.at_start ? 0 : 1] + 10.0;
						if (back > c.length - 5.0) continue;
						const SegCtx &cc = ctx[from.seg];
						const double s = from.at_start ? back : cc.length - back;
						cc.cross_section(s, xs);
						Vec2 p, nrm;
						cc.frame(s, p, nrm);
						const size_t li = in[i].index;
						const Vec2 center = p + nrm * (0.5 * (xs[li].first + xs[li].second));
						const Vec2 fwd = in[i].dir;
						// Merge towards the median = the travel-left side.
						Glyph gl{ { { 0.0, -3.0 }, { 0.0, -0.6 }, { -0.8, 0.5 } }, { -1.15, 1.0 },
							Vec2{ -0.6, 0.8 }.normalized() };
						emit_glyph(mesh.get(g.level, Layer::Markings), gl, center, fwd, kWhite);
					}
				}
			}
			continue;
		}
		if (g.kind != NodeKind::Junction) continue;

		// Junction surface: corners of each leg joined by curb fillets.
		std::vector<Vec2> poly;
		MeshBatch &ground = mesh.get(g.level, Layer::Ground);
		for (size_t i = 0; i < deg; ++i) {
			const Leg &a = g.legs[i];
			const Leg &b = g.legs[(i + 1) % deg];
			const Vec2 al = leg_point(a, -a.kerb_left);
			const Vec2 ar = leg_point(a, a.kerb_right);
			const Vec2 bl = leg_point(b, -b.kerb_left);
			poly.push_back(al);
			poly.push_back(ar);
			double theta = angle_of(b.dir) - angle_of(a.dir);
			while (theta <= 0.0) theta += 2.0 * kPi;
			const bool curved = std::fabs(std::sin(theta)) > 0.17;
			double t = 0.0, u = 0.0;
			std::vector<Vec2> kerb_curve;
			if (curved && line_intersect(ar, a.dir, bl, b.dir, t, u)) {
				kerb_curve = quad_bezier(ar, ar + a.dir * t, bl, 10);
			} else {
				kerb_curve = { ar, bl };
			}
			for (size_t k = 1; k + 1 < kerb_curve.size(); ++k) poly.push_back(kerb_curve[k]);
			// Sidewalk corner when both sides have sidewalks.
			const bool swa = a.outer_right > a.kerb_right + 0.1;
			const bool swb = b.outer_left > b.kerb_left + 0.1;
			if (swa && swb) {
				const Vec2 aro = leg_point(a, a.outer_right);
				const Vec2 blo = leg_point(b, -b.outer_left);
				std::vector<Vec2> outer;
				double to = 0.0, uo = 0.0;
				if (curved && line_intersect(aro, a.dir, blo, b.dir, to, uo)) {
					outer = quad_bezier(aro, aro + a.dir * to, blo, 10);
				} else {
					outer = quad_bezier(aro, (aro + blo) * 0.5, blo, 10);
				}
				if (kerb_curve.size() == 2) kerb_curve = quad_bezier(ar, (ar + bl) * 0.5, bl, 10);
				MeshSet::strip(ground, kerb_curve, outer, kSidewalk);
			}
		}
		g.polygon = poly;
		for (const std::vector<Vec2> &ring : clean_polygon(poly)) {
			triangulate(ring, mesh.get(g.level, Layer::Asphalt), kAsphalt);
		}

		// Stop lines (not at simple bends).
		MeshBatch &paint = mesh.get(g.level, Layer::Markings);
		for (const Leg &leg : g.legs) {
			if (deg < 3) break;
			const std::vector<LegLane> in = leg_lanes(leg, true);
			if (in.empty()) continue;
			const SegCtx &c = ctx[leg.seg];
			const double s = leg.at_start ? leg.trim + 0.5 : c.length - leg.trim - 0.5;
			c.cross_section(s, xs);
			double lo = 1e300, hi = -1e300;
			for (const LegLane &l : in) {
				lo = std::min(lo, xs[l.index].first);
				hi = std::max(hi, xs[l.index].second);
			}
			Vec2 p, nrm;
			c.frame(s, p, nrm);
			const Vec2 tan = tangent_of(nrm) * 0.2;
			const Vec2 a = p + nrm * lo, b = p + nrm * hi;
			MeshSet::quad(paint, a - tan, b - tan, b + tan, a + tan, kWhite);
		}

		// Default connectors from turn rules.
		for (size_t j = 0; j < deg; ++j) {
			const Leg &leg = g.legs[j];
			std::vector<LegLane> in = leg_lanes(leg, true);
			std::vector<LegLane> general;
			const LegLane *pocket_l = nullptr;
			const LegLane *pocket_r = nullptr;
			for (const LegLane &l : in) {
				if (!is_travel(l.type)) continue;
				if (l.pocket) {
					(l.pocket_left ? pocket_l : pocket_r) = &l;
				} else {
					general.push_back(l);
				}
			}
			if (general.empty() && !pocket_l && !pocket_r) continue;
			const EndRules &rules = ctx[leg.seg].seg->ends[leg.at_start ? 0 : 1];
			const Vec2 in_dir = leg.dir * -1.0;
			struct Target {
				size_t leg;
				double angle;
			};
			std::vector<Target> rights, lefts, straights;
			for (size_t k = 0; k < deg; ++k) {
				if (k == j) continue;
				const double ang = std::atan2(cross(in_dir, g.legs[k].dir), in_dir.dot(g.legs[k].dir));
				switch (turn_between(g, j, k)) {
					case TurnKind::Straight: straights.push_back({ k, ang }); break;
					case TurnKind::Right: rights.push_back({ k, ang }); break;
					case TurnKind::Left: lefts.push_back({ k, ang }); break;
					case TurnKind::UTurn: break; // never by default
				}
			}
			// Keep one straight; the others count as slight turns.
			if (straights.size() > 1) {
				std::sort(straights.begin(), straights.end(),
						[](const Target &a, const Target &b) { return std::fabs(a.angle) < std::fabs(b.angle); });
				for (size_t k = 1; k < straights.size(); ++k) {
					(straights[k].angle > 0 ? rights : lefts).push_back(straights[k]);
				}
				straights.resize(1);
			}
			std::map<LaneId, std::set<TurnKind>> turns;
			auto link = [&](const LegLane &from, size_t to_leg, bool pick_right, size_t offset, TurnKind kind) {
				std::vector<LegLane> out = leg_lanes(g.legs[to_leg], false);
				out.erase(std::remove_if(out.begin(), out.end(), [](const LegLane &l) { return !is_travel(l.type); }),
						out.end());
				if (out.empty()) return;
				const size_t idx = pick_right ? std::min(offset, out.size() - 1)
											  : out.size() - 1 - std::min(offset, out.size() - 1);
				const LegLane &to = out[idx];
				Connector cn;
				cn.from_seg = leg.seg;
				cn.from_lane = from.id;
				cn.to_seg = g.legs[to_leg].seg;
				cn.to_lane = to.id;
				cn.turn = kind;
				cn.path = connector_path(from.point, from.dir, to.point, to.dir);
				g.connectors.push_back(cn);
				turns[from.id].insert(kind);
			};
			const size_t n = general.size();
			if (!straights.empty() && n > 0) {
				for (size_t i = 0; i < n; ++i) link(general[i], straights[0].leg, true, i, TurnKind::Straight);
			}
			if (rules.right != TurnRule::Disallowed) {
				for (const Target &r : rights) {
					if (pocket_r) {
						link(*pocket_r, r.leg, true, 0, TurnKind::Right);
					} else if (n > 0) {
						const size_t share = straights.empty() && !lefts.empty() ? (n + 1) / 2 : 1;
						for (size_t i = 0; i < share; ++i) link(general[i], r.leg, true, i, TurnKind::Right);
					}
				}
			}
			if (rules.left != TurnRule::Disallowed) {
				for (const Target &l : lefts) {
					if (pocket_l) {
						link(*pocket_l, l.leg, false, 0, TurnKind::Left);
					} else if (n > 0) {
						const size_t share = straights.empty() && !rights.empty() ? n / 2 + (n == 1 ? 1 : 0) : 1;
						for (size_t i = 0; i < std::max<size_t>(share, 1); ++i) {
							link(general[n - 1 - i], l.leg, false, i, TurnKind::Left);
						}
					}
				}
			}
			// Lanes the rules left without a way out share the nearest turn.
			for (size_t i = 0; i < n; ++i) {
				if (turns.count(general[i].id)) continue;
				if (rules.left != TurnRule::Disallowed && !lefts.empty()) {
					link(general[i], lefts[0].leg, false, n - 1 - i, TurnKind::Left);
				} else if (rules.right != TurnRule::Disallowed && !rights.empty()) {
					link(general[i], rights[0].leg, true, i, TurnKind::Right);
				} else if (!straights.empty()) {
					link(general[i], straights[0].leg, true, i, TurnKind::Straight);
				}
			}
			// Arrows painted from the connectors, 8 m before the stop line.
			const SegCtx &c = ctx[leg.seg];
			const double back = 8.0;
			const double avail = c.length - c.trim[0] - c.trim[1];
			for (const LegLane &l : in) {
				if (!is_travel(l.type)) continue;
				auto it = turns.find(l.id);
				if (it == turns.end()) {
					g.dead_lanes.push_back({ leg.seg, l.id });
					continue;
				}
				if (avail < 20.0 || deg < 3) continue;
				const double s = leg.at_start ? leg.trim + back : c.length - leg.trim - back;
				c.cross_section(s, xs);
				Vec2 p, nrm;
				c.frame(s, p, nrm);
				const Vec2 center = p + nrm * (0.5 * (xs[l.index].first + xs[l.index].second));
				for (TurnKind tk : it->second) {
					emit_glyph(paint, arrow_glyph(tk), center, l.dir, kWhite);
				}
			}
		}
	}

	// --- 8. Assemble meshes by level, then layer ---------------------------------------
	for (auto &kv : mesh.batches) {
		if (!kv.second.indices.empty()) {
			meshes_.push_back(std::move(kv.second));
		}
	}
	// Copy trims from legs into segment results (junction legs were clamped).
	for (auto &kv : ctx) {
		SegmentGeom &sg = segments_[kv.first];
		sg.trim[0] = kv.second.trim[0];
		sg.trim[1] = kv.second.trim[1];
	}
}

// --- Picking ------------------------------------------------------------------------

bool RoadGeometry::pick_segment(Vec2 p, double radius, int level, LaneHit &out) const {
	double best = 1e300;
	bool found = false;
	for (const auto &kv : segments_) {
		const SegmentGeom &sg = kv.second;
		if (sg.level != level || sg.s.empty()) continue;
		if (p.x < sg.bb_min.x - radius || p.y < sg.bb_min.y - radius || p.x > sg.bb_max.x + radius ||
				p.y > sg.bb_max.y + radius) {
			continue;
		}
		// Nearest sample, refined between neighbours.
		size_t kbest = 0;
		double dbest = 1e300;
		for (size_t k = 0; k < sg.p.size(); ++k) {
			const double d = (sg.p[k] - p).dot(sg.p[k] - p);
			if (d < dbest) {
				dbest = d;
				kbest = k;
			}
		}
		const Vec2 tan = tangent_of(sg.n[kbest]);
		const double along = (p - sg.p[kbest]).dot(tan);
		const double lateral = (p - sg.p[kbest]).dot(sg.n[kbest]);
		double lo = 1e300, hi = -1e300;
		for (size_t i = 0; i < sg.lanes.size(); ++i) {
			lo = std::min(lo, sg.edge(kbest, i).first);
			hi = std::max(hi, sg.edge(kbest, i).second);
		}
		double outside = 0.0;
		if (lateral < lo) outside = lo - lateral;
		if (lateral > hi) outside = lateral - hi;
		const double s = std::clamp(sg.s[kbest] + along, sg.s.front(), sg.s.back());
		const double beyond = std::max(0.0, std::max(sg.s.front() - (sg.s[kbest] + along), (sg.s[kbest] + along) - sg.s.back()));
		const double dist = std::sqrt(outside * outside + beyond * beyond);
		if (dist <= radius && dist < best) {
			best = dist;
			found = true;
			out = LaneHit{};
			out.seg = sg.id;
			out.s = s;
			out.u = sg.length > 0 ? s / sg.length : 0.0;
			out.offset = lateral;
			// Lane and nearest interior boundary at this sample.
			double edist = 1e300;
			for (size_t i = 0; i < sg.lanes.size(); ++i) {
				const auto &e = sg.edge(kbest, i);
				if (lateral >= e.first && lateral <= e.second && e.second - e.first > 0.05) {
					out.lane_index = static_cast<int>(i);
					out.lane = sg.lanes[i].id;
				}
				if (i > 0 && sg.lanes[i].profile_index > 0 && sg.lanes[i - 1].profile_index == sg.lanes[i].profile_index - 1) {
					const double d = std::fabs(lateral - e.first);
					if (d < edist) {
						edist = d;
						out.edge = sg.lanes[i].profile_index;
						out.edge_distance = d;
					}
				}
			}
		}
	}
	return found;
}

bool RoadGeometry::pick_lane(Vec2 p, int level, LaneHit &out) const {
	return pick_segment(p, 0.0, level, out) && out.lane_index >= 0;
}

} // namespace tsim
