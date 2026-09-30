// Keeps a compiled Network and a Traffic simulation in step with an editable
// map. Used by the Godot bridge, the tests and the benchmark.
//
// sync() recompiles only when the map's revision changed, and only the parts
// that changed (NetworkCompiler caches the rest). Vehicles carry over to the
// new network, so editing while paused and resuming needs no full rebuild.
#pragma once

#include "tsim/network.h"
#include "tsim/road_geometry.h"
#include "tsim/traffic.h"

namespace tsim {

class TrafficRun {
public:
	explicit TrafficRun(TrafficConfig config = {});

	// Recompiles the network if `revision` differs from the last sync. Returns
	// true when it did.
	bool sync(const RoadMap &map, const RoadGeometry &geom, uint64_t revision);
	// Forgets the cached compile (the next sync compiles everything).
	void invalidate();

	Traffic &traffic() { return traffic_; }
	const Traffic &traffic() const { return traffic_; }
	const Network &network() const { return nets_[current_]; }
	const CompileStats &compile_stats() const { return compiler_.stats(); }
	uint64_t compiled_revision() const { return revision_; }

private:
	NetworkCompiler compiler_;
	Network nets_[2];
	int current_ = 0;
	Traffic traffic_;
	uint64_t revision_ = ~0ull;
};

// The M2 reference run: the test grid, 500 cars at most, demand x2.
struct TrafficGolden {
	static constexpr uint64_t kSeed = 42;
	static constexpr uint64_t kTicks = 6000; // 10 sim minutes
	static constexpr uint32_t kMaxVehicles = 500;
	static constexpr double kDemand = 2.0;
	// Expected state hash after kTicks. Update only on an intentional change to
	// sim behaviour, and note why in the commit message.
	static constexpr const char *kExpectedHash = "2b4a55beb24226a0";
};

// Runs the M2 reference scenario and returns the final state hash.
uint64_t run_traffic_golden(uint64_t ticks = TrafficGolden::kTicks, TrafficStats *stats = nullptr,
		double *us_per_tick = nullptr);

} // namespace tsim
