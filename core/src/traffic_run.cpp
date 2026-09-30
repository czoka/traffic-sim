#include "tsim/traffic_run.h"

#include "tsim/demo_maps.h"
#include "tsim/document.h"

#include <chrono>

namespace tsim {

TrafficRun::TrafficRun(TrafficConfig config) : traffic_(config) {}

bool TrafficRun::sync(const RoadMap &map, const RoadGeometry &geom, uint64_t revision) {
	if (revision == revision_) return false;
	const int next = 1 - current_;
	compiler_.compile(map, geom, nets_[next]);
	traffic_.set_network(&nets_[next]);
	current_ = next;
	revision_ = revision;
	return true;
}

void TrafficRun::invalidate() {
	compiler_.clear_cache();
	revision_ = ~0ull;
}

uint64_t run_traffic_golden(uint64_t ticks, TrafficStats *stats, double *us_per_tick) {
	Document doc;
	build_test_grid(doc, 7, 8, 120.0);
	RoadGeometry geom;
	geom.build(doc.map());
	TrafficRun run;
	run.sync(doc.map(), geom, doc.revision());
	Traffic &t = run.traffic();
	t.config().demand = TrafficGolden::kDemand;
	t.config().max_vehicles = TrafficGolden::kMaxVehicles;
	t.reset(TrafficGolden::kSeed);
	const auto t0 = std::chrono::steady_clock::now();
	for (uint64_t i = 0; i < ticks; ++i) t.tick();
	if (us_per_tick) {
		*us_per_tick = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
				static_cast<double>(ticks ? ticks : 1);
	}
	if (stats) *stats = t.stats();
	return t.state_hash();
}

} // namespace tsim
