// RoadEditor: M6, the economy: the player's numbers for city-owned buildings,
// the city centre marker, and the market in the sim.
#include "m3_dicts.h"
#include "road_editor.h"

#include "tsim/city_data.h"

#include <godot_cpp/core/class_db.hpp>

#include <algorithm>
#include <cmath>

namespace godot {

using namespace tsim;
using namespace m3;

namespace {

Vector2 gv(Vec2 v) { return Vector2(static_cast<real_t>(v.x), static_cast<real_t>(v.y)); }

} // namespace

void RoadEditor::set_building_economy(int64_t id, const Dictionary &settings) {
	const Building *cur = doc_.map().building(static_cast<uint32_t>(id));
	if (!cur) return;
	Building b = *cur;
	b.rent = std::max(0.0, static_cast<double>(settings.get("rent", b.rent)));
	b.price_factor = std::clamp(static_cast<double>(settings.get("price_factor", b.price_factor)), 0.1, 10.0);
	b.wage = std::max(0.0, static_cast<double>(settings.get("wage", b.wage)));
	b.for_sale = static_cast<bool>(settings.get("for_sale", b.for_sale));
	b.asking = std::max(0.0, static_cast<double>(settings.get("asking", b.asking)));
	if (b == *cur) return;
	doc_.set_building(b);
}

void RoadEditor::set_city_centre(Vector2 pos) { doc_.set_city_centre(Vec2{ pos.x, pos.y }); }

void RoadEditor::clear_city_centre() {
	if (doc_.map().city_centre()) doc_.set_city_centre(std::nullopt);
}

Dictionary RoadEditor::get_city_centre() {
	Dictionary d;
	const auto &c = doc_.map().city_centre();
	d["placed"] = c.has_value();
	if (c) {
		d["pos"] = gv(*c);
		return d;
	}
	// The default: the centroid of the shops and offices (else of everything).
	ensure_network();
	Vec2 sum{ 0.0, 0.0 }, all{ 0.0, 0.0 };
	double n = 0.0, na = 0.0;
	for (const NetBuilding &b : check_net_.buildings) {
		all = all + b.centre;
		na += 1.0;
		if (b.type < 0 || b.kind == BuildingKind::Home) continue;
		sum = sum + b.centre;
		n += 1.0;
	}
	d["pos"] = gv(n > 0.0 ? sum * (1.0 / n) : na > 0.0 ? all * (1.0 / na) : Vec2{ 0.0, 0.0 });
	return d;
}

Array RoadEditor::sim_market() {
	Array out;
	const Traffic &t = sim_.traffic();
	if (!t.network()) return out;
	const CityData &cd = default_city_data();
	for (const Listing &l : t.market()) {
		Dictionary d;
		d["id"] = static_cast<int64_t>(l.building);
		const Building *b = doc_.map().building(l.building);
		const BuildingType *ty = b ? cd.type(b->type) : nullptr;
		d["label"] = ty ? gstr(ty->label) : String("Building");
		d["name"] = b ? gstr(b->name) : String();
		d["by_city"] = l.by_city;
		d["owner"] = static_cast<int64_t>(l.owner);
		d["asking"] = l.asking;
		d["value"] = l.value;
		d["days"] = l.days;
		if (b) d["pos"] = gv(b->pos);
		out.push_back(d);
	}
	return out;
}

bool RoadEditor::sim_buy_building(int64_t id) { return sim_.traffic().buy_building(static_cast<uint32_t>(id)); }

void RoadEditor::bind_m6_methods() {
	ClassDB::bind_method(D_METHOD("set_building_economy", "id", "settings"), &RoadEditor::set_building_economy);
	ClassDB::bind_method(D_METHOD("set_city_centre", "pos"), &RoadEditor::set_city_centre);
	ClassDB::bind_method(D_METHOD("clear_city_centre"), &RoadEditor::clear_city_centre);
	ClassDB::bind_method(D_METHOD("get_city_centre"), &RoadEditor::get_city_centre);
	ClassDB::bind_method(D_METHOD("sim_market"), &RoadEditor::sim_market);
	ClassDB::bind_method(D_METHOD("sim_buy_building", "id"), &RoadEditor::sim_buy_building);
}

} // namespace godot
