// Dictionary <-> tsim conversions for the M3 map objects (roundabouts,
// signal plans, bus stops, depots), shared by the RoadEditor sources.
#pragma once

#include "tsim/road_map.h"

#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_int64_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include <string>
#include <vector>

namespace godot {
namespace m3 {

inline std::string str(const String &s) {
	const CharString c = s.utf8();
	return std::string(c.get_data(), static_cast<size_t>(c.length()));
}
inline String gstr(const std::string &s) { return String::utf8(s.c_str(), static_cast<int64_t>(s.size())); }

inline PackedInt64Array ids(const std::vector<uint32_t> &v) {
	PackedInt64Array out;
	for (uint32_t x : v) out.push_back(static_cast<int64_t>(x));
	return out;
}
inline std::vector<uint32_t> ids_from(const Variant &v) {
	std::vector<uint32_t> out;
	if (v.get_type() == Variant::PACKED_INT64_ARRAY) {
		const PackedInt64Array a = v;
		for (int64_t i = 0; i < a.size(); ++i) out.push_back(static_cast<uint32_t>(a[i]));
	} else if (v.get_type() == Variant::ARRAY) {
		const Array a = v;
		for (int64_t i = 0; i < a.size(); ++i) out.push_back(static_cast<uint32_t>(static_cast<int64_t>(a[i])));
	}
	return out;
}

inline Dictionary roundabout_dict(const tsim::Roundabout &r) {
	Dictionary d;
	d["enabled"] = r.enabled;
	d["radius"] = r.radius;
	d["lanes"] = r.lanes;
	d["turbo"] = r.turbo;
	d["slip"] = ids(r.slip);
	return d;
}
inline tsim::Roundabout roundabout_from(const Dictionary &d) {
	tsim::Roundabout r;
	r.enabled = d.get("enabled", true);
	r.radius = static_cast<double>(d.get("radius", r.radius));
	r.lanes = static_cast<int>(d.get("lanes", r.lanes));
	r.turbo = d.get("turbo", r.turbo);
	r.slip = ids_from(d.get("slip", Array()));
	return r;
}

inline Dictionary signal_dict(const tsim::SignalPlan &p) {
	Dictionary d;
	Array phases;
	for (const tsim::SignalPhase &ph : p.phases) {
		Dictionary pd;
		pd["green"] = ph.green;
		Array moves;
		for (const tsim::SignalMovement &m : ph.moves) {
			Dictionary md;
			md["from"] = static_cast<int64_t>(m.from);
			md["to"] = static_cast<int64_t>(m.to);
			md["permissive"] = m.permissive;
			moves.push_back(md);
		}
		pd["moves"] = moves;
		phases.push_back(pd);
	}
	d["phases"] = phases;
	d["amber"] = p.amber;
	d["all_red"] = p.all_red;
	d["offset"] = p.offset;
	d["right_on_red"] = ids(p.right_on_red);
	d["cycle"] = p.cycle();
	return d;
}
inline tsim::SignalPlan signal_from(const Dictionary &d) {
	tsim::SignalPlan p;
	const Array phases = d.get("phases", Array());
	for (int64_t i = 0; i < phases.size(); ++i) {
		const Dictionary pd = phases[i];
		tsim::SignalPhase ph;
		ph.green = static_cast<double>(pd.get("green", ph.green));
		const Array moves = pd.get("moves", Array());
		for (int64_t k = 0; k < moves.size(); ++k) {
			const Dictionary md = moves[k];
			tsim::SignalMovement m;
			m.from = static_cast<tsim::SegmentId>(static_cast<int64_t>(md.get("from", 0)));
			m.to = static_cast<tsim::SegmentId>(static_cast<int64_t>(md.get("to", 0)));
			m.permissive = md.get("permissive", false);
			ph.moves.push_back(m);
		}
		p.phases.push_back(ph);
	}
	p.amber = static_cast<double>(d.get("amber", p.amber));
	p.all_red = static_cast<double>(d.get("all_red", p.all_red));
	p.offset = static_cast<double>(d.get("offset", p.offset));
	p.right_on_red = ids_from(d.get("right_on_red", Array()));
	return p;
}

inline Dictionary stop_dict(const tsim::BusStop &s) {
	Dictionary d;
	d["id"] = static_cast<int64_t>(s.id);
	d["u"] = s.u;
	d["side"] = s.side == tsim::LaneDir::Backward ? "backward" : "forward";
	d["kind"] = tsim::stop_kind_name(s.kind);
	d["name"] = gstr(s.name);
	d["bays"] = s.bays;
	return d;
}
inline tsim::BusStop stop_from(const Dictionary &d) {
	tsim::BusStop s;
	s.id = static_cast<uint32_t>(static_cast<int64_t>(d.get("id", 0)));
	s.u = static_cast<double>(d.get("u", s.u));
	s.side = String(d.get("side", "forward")) == "backward" ? tsim::LaneDir::Backward : tsim::LaneDir::Forward;
	tsim::stop_kind_from_name(str(d.get("kind", "kerbside")), s.kind);
	s.name = str(d.get("name", ""));
	s.bays = static_cast<int>(d.get("bays", s.bays));
	return s;
}

inline Dictionary depot_dict(const tsim::Depot &dp) {
	Dictionary d;
	d["enabled"] = dp.enabled;
	d["name"] = gstr(dp.name);
	d["capacity"] = dp.capacity;
	Array routes;
	for (const tsim::BusRoute &r : dp.routes) {
		Dictionary rd;
		rd["id"] = static_cast<int64_t>(r.id);
		rd["name"] = gstr(r.name);
		rd["color"] = static_cast<int64_t>(r.color);
		rd["stops"] = ids(r.stops);
		rd["headway"] = r.headway;
		rd["loop"] = r.loop;
		routes.push_back(rd);
	}
	d["routes"] = routes;
	return d;
}
inline tsim::Depot depot_from(const Dictionary &d) {
	tsim::Depot dp;
	dp.enabled = d.get("enabled", true);
	dp.name = str(d.get("name", ""));
	dp.capacity = static_cast<int>(d.get("capacity", dp.capacity));
	const Array routes = d.get("routes", Array());
	for (int64_t i = 0; i < routes.size(); ++i) {
		const Dictionary rd = routes[i];
		tsim::BusRoute r;
		r.id = static_cast<uint32_t>(static_cast<int64_t>(rd.get("id", 0)));
		r.name = str(rd.get("name", ""));
		r.color = static_cast<uint32_t>(static_cast<int64_t>(rd.get("color", static_cast<int64_t>(r.color))));
		r.stops = ids_from(rd.get("stops", Array()));
		r.headway = static_cast<double>(rd.get("headway", r.headway));
		r.loop = rd.get("loop", r.loop);
		dp.routes.push_back(r);
	}
	return dp;
}

} // namespace m3
} // namespace godot
