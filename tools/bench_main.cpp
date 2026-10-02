// Headless benchmark for the simulation core.
//
//   tsim_bench [--cars N] [--ticks T] [--seed S] [--radius R] [--lanes L] [--report-every K]
//   tsim_bench --traffic [--cars CAP] [--minutes M] [--seed S] [--demand D]
//   tsim_bench --write-maps DIR
//   tsim_bench --example NAME [--warmup M] [--minutes M]   (M7: an example map as the editor runs it)
//
// Ring mode prints the mean tick time and periodic traffic stats (to see
// stop-and-go waves form), then the final state hash. --traffic runs the M2
// sim on the test grid. --write-maps saves the example maps. With no arguments
// it runs both golden scenarios (POC ring and M2 grid) and prints PASS/FAIL
// against the expected hashes.
#include "tsim/demo_maps.h"
#include "tsim/document.h"
#include "tsim/hash.h"
#include "tsim/map.h"
#include "tsim/road_map_json.h"
#include "tsim/sim.h"
#include "tsim/traffic_run.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace tsim;

namespace {

bool write_file(const std::string &path, const std::string &text) {
	FILE *f = std::fopen(path.c_str(), "wb");
	if (!f) return false;
	const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
	return std::fclose(f) == 0 && ok;
}

int write_maps(const std::string &dir) {
	struct Entry {
		const char *file;
		void (*build)(Document &);
	};
	const Entry maps[] = {
		{ "demo_town_v7.json", [](Document &d) { build_demo_town(d); } },
		{ "test_grid_v7.json", [](Document &d) { build_test_grid(d, 7, 8, 120.0); } },
		{ "t_junction_v7.json", [](Document &d) { build_t_junction(d); } },
		{ "lane_drop_v7.json", [](Document &d) { build_lane_drop(d); } },
		{ "one_way_pair_v7.json", [](Document &d) { build_one_way_pair(d); } },
		{ "showcase_v7.json", [](Document &d) { build_showcase(d); } },
		{ "people_town_v7.json", [](Document &d) { build_people_town(d); } },
		{ "people_city_v7.json", [](Document &d) { build_people_city(d, 12, 12); } },
		{ "new_city_v7.json", [](Document &d) { build_new_city(d); } },
		{ "city_town_v7.json", [](Document &d) { build_city_town(d); } },
		{ "city_week_v7.json", [](Document &d) { build_city_week(d); } },
		{ "city_market_v7.json", [](Document &d) { build_city_market(d); } },
		{ "tutorial_v7.json", [](Document &d) { build_tutorial(d); } },
	};
	for (const Entry &e : maps) {
		Document doc;
		e.build(doc);
		const std::string path = dir + "/" + e.file;
		if (!write_file(path, road_map_to_json(doc.map()))) {
			std::fprintf(stderr, "could not write %s\n", path.c_str());
			return 1;
		}
		std::printf("wrote %s\n", path.c_str());
	}
	return 0;
}

int run_traffic(uint32_t cap, double minutes, uint64_t seed, double demand) {
	Document doc;
	build_test_grid(doc, 7, 8, 120.0);
	RoadGeometry geom;
	geom.build(doc.map());
	TrafficRun run;
	run.sync(doc.map(), geom, doc.revision());
	Traffic &t = run.traffic();
	t.config().max_vehicles = cap;
	t.config().demand = demand;
	t.reset(seed);
	std::printf("test grid: %zu lanes, %zu junctions, %zu spawn points; cap %u cars, demand x%.2f, seed %llu\n",
			run.network().lanes.size(), run.network().junctions.size(), run.network().spawners.size(), cap, demand,
			static_cast<unsigned long long>(seed));
	std::printf("%6s %6s %8s %8s %8s %9s %9s %8s\n", "min", "cars", "arrived", "kmh", "stopped", "max_stop", "junc_wait",
			"us/tick");
	using clock = std::chrono::steady_clock;
	double total_us = 0.0;
	const uint64_t per_min = static_cast<uint64_t>(60.0 / t.config().dt + 0.5);
	const int mins = static_cast<int>(minutes);
	for (int m = 1; m <= mins; ++m) {
		const auto t0 = clock::now();
		for (uint64_t k = 0; k < per_min; ++k) t.tick();
		const double us = std::chrono::duration<double, std::micro>(clock::now() - t0).count();
		total_us += us;
		const TrafficStats st = t.stats();
		std::printf("%6d %6u %8llu %8.1f %8u %9.0f %9.0f %8.1f\n", m, st.vehicles,
				static_cast<unsigned long long>(st.arrived), st.mean_speed * 3.6, st.stopped, st.max_stopped,
				st.max_junction_wait, us / per_min);
	}
	std::printf("mean tick: %.1f us, state hash %s\n", total_us / (per_min * (mins ? mins : 1)),
			hash_to_hex(t.state_hash()).c_str());
	return 0;
}

// The editor's examples, with the editor's caps (2,000 cars, 1,000 people) and
// city prefill. Prints the mean tick and what 128x (1,280 ticks a second) costs.
int run_example(const std::string &name, double warmup, double minutes) {
	Document doc;
	double prefill = 0.0;
	if (name == "town") build_demo_town(doc);
	else if (name == "grid") build_test_grid(doc, 7, 8, 120.0);
	else if (name == "showcase") build_showcase(doc);
	else if (name == "people") build_people_town(doc);
	else if (name == "people_city") build_people_city(doc, 12, 12);
	else if (name == "city_town") build_city_town(doc), prefill = 1.0;
	else if (name == "city_week") build_city_week(doc), prefill = 0.95;
	else if (name == "city_market") build_city_market(doc), prefill = 0.95;
	else if (name == "tutorial") build_tutorial(doc);
	else {
		std::fprintf(stderr, "unknown example %s\n", name.c_str());
		return 2;
	}
	RoadGeometry geom;
	geom.build(doc.map());
	TrafficRun run;
	run.sync(doc.map(), geom, doc.revision());
	Traffic &t = run.traffic();
	t.config().max_vehicles = 2000;
	t.config().max_pedestrians = 1000;
	t.config().city_prefill = prefill;
	t.reset(42);
	using clock = std::chrono::steady_clock;
	const uint64_t per_min = static_cast<uint64_t>(60.0 / t.config().dt + 0.5);
	for (uint64_t k = 0; k < static_cast<uint64_t>(warmup * per_min); ++k) t.tick();
	const uint64_t n = static_cast<uint64_t>(minutes * per_min);
	double worst_ms = 0.0;
	const auto t0 = clock::now();
	for (uint64_t k = 0; k < n; k += 128) {
		const auto a = clock::now();
		for (uint64_t j = 0; j < 128 && k + j < n; ++j) t.tick();
		worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(clock::now() - a).count());
	}
	const double us = std::chrono::duration<double, std::micro>(clock::now() - t0).count() / static_cast<double>(n);
	const TrafficStats st = t.stats();
	std::printf("%-12s %5u vehicles %5u people %5u residents: %7.1f us/tick, 128x needs %5.0f ms of sim a second "
				"(%s), worst 128 ticks %.0f ms\n",
			name.c_str(), st.vehicles, st.pedestrians + st.riding, t.city_stats().residents, us, us * 1280.0 / 1000.0,
			us * 1280.0 / 1000.0 <= 480.0 ? "fits the 8 ms a frame at 60 fps" : "CPU-limited", worst_ms);
	return 0;
}

} // namespace

int main(int argc, char **argv) {
	if (argc >= 3 && !std::strcmp(argv[1], "--write-maps")) return write_maps(argv[2]);
	if (argc >= 3 && !std::strcmp(argv[1], "--example")) {
		double warmup = 10.0, minutes = 2.0;
		for (int i = 3; i + 1 < argc; i += 2) {
			if (!std::strcmp(argv[i], "--warmup")) warmup = std::strtod(argv[i + 1], nullptr);
			else if (!std::strcmp(argv[i], "--minutes")) minutes = std::strtod(argv[i + 1], nullptr);
		}
		return run_example(argv[2], warmup, minutes);
	}
	if (argc >= 2 && !std::strcmp(argv[1], "--traffic")) {
		uint32_t cap = TrafficGolden::kMaxVehicles;
		double minutes = 60.0, demand = TrafficGolden::kDemand;
		uint64_t seed = TrafficGolden::kSeed;
		for (int i = 2; i + 1 < argc; i += 2) {
			if (!std::strcmp(argv[i], "--cars")) cap = static_cast<uint32_t>(std::strtoul(argv[i + 1], nullptr, 10));
			else if (!std::strcmp(argv[i], "--minutes")) minutes = std::strtod(argv[i + 1], nullptr);
			else if (!std::strcmp(argv[i], "--seed")) seed = std::strtoull(argv[i + 1], nullptr, 10);
			else if (!std::strcmp(argv[i], "--demand")) demand = std::strtod(argv[i + 1], nullptr);
			else {
				std::fprintf(stderr, "unknown option %s\n", argv[i]);
				return 2;
			}
		}
		return run_traffic(cap, minutes, seed, demand);
	}

	uint32_t cars = GoldenScenario::kCars;
	uint64_t ticks = GoldenScenario::kTicks;
	uint64_t seed = GoldenScenario::kSeed;
	double radius = GoldenScenario::kRadius;
	int lanes = GoldenScenario::kLanes;
	uint64_t report_every = 600;
	bool custom = false;

	for (int i = 1; i + 1 < argc; i += 2) {
		const char *k = argv[i];
		const char *v = argv[i + 1];
		custom = true;
		if (!std::strcmp(k, "--cars")) cars = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
		else if (!std::strcmp(k, "--ticks")) ticks = std::strtoull(v, nullptr, 10);
		else if (!std::strcmp(k, "--seed")) seed = std::strtoull(v, nullptr, 10);
		else if (!std::strcmp(k, "--radius")) radius = std::strtod(v, nullptr);
		else if (!std::strcmp(k, "--lanes")) lanes = std::atoi(v);
		else if (!std::strcmp(k, "--report-every")) report_every = std::strtoull(v, nullptr, 10);
		else {
			std::fprintf(stderr, "unknown option %s\n", k);
			return 2;
		}
	}

	Map map;
	build_ring(map, radius, lanes, GoldenScenario::kLaneWidth, GoldenScenario::kSpeedLimit);
	Simulation sim(map);
	const uint32_t placed = sim.spawn_even(cars, seed);
	std::printf("ring r=%.0f m, %d lanes, %u cars (%.1f m per car per lane), seed %llu\n", radius, lanes, placed,
			(2.0 * kPi * radius * lanes) / (placed ? placed : 1), static_cast<unsigned long long>(seed));
	std::printf("%8s %10s %10s %10s %8s\n", "sim_s", "mean_kmh", "sd_kmh", "stopped%", "us/tick");

	using clock = std::chrono::steady_clock;
	double total_us = 0.0;
	double window_us = 0.0;
	uint64_t window_ticks = 0;
	for (uint64_t t = 1; t <= ticks; ++t) {
		const auto t0 = clock::now();
		sim.tick();
		const double us = std::chrono::duration<double, std::micro>(clock::now() - t0).count();
		total_us += us;
		window_us += us;
		++window_ticks;
		if (report_every && (t % report_every == 0 || t == ticks)) {
			const SimStats st = sim.stats();
			std::printf("%8.0f %10.1f %10.1f %10.1f %8.1f\n", sim.sim_time(), st.mean_speed * 3.6, st.speed_stddev * 3.6,
					100.0 * st.stopped / (st.vehicles ? st.vehicles : 1), window_us / window_ticks);
			window_us = 0.0;
			window_ticks = 0;
		}
	}
	const std::string hash = hash_to_hex(sim.state_hash());
	std::printf("mean tick: %.1f us (%.3f us per car)\n", total_us / ticks, total_us / ticks / (placed ? placed : 1));
	std::printf("state hash: %s\n", hash.c_str());
	if (!custom) {
		const bool ok = hash == GoldenScenario::kExpectedHash;
		std::printf("golden check: %s\n", ok ? "PASS" : "FAIL");
		double us = 0.0;
		TrafficStats st;
		const std::string h2 = hash_to_hex(run_traffic_golden(TrafficGolden::kTicks, &st, &us));
		const bool ok2 = h2 == TrafficGolden::kExpectedHash;
		std::printf("M2 grid: %u cars after %llu ticks, %llu trips, %.1f us/tick, state hash %s (expected %s)\n",
				st.vehicles, static_cast<unsigned long long>(TrafficGolden::kTicks),
				static_cast<unsigned long long>(st.arrived), us, h2.c_str(), TrafficGolden::kExpectedHash);
		std::printf("M2 golden check: %s\n", ok2 ? "PASS" : "FAIL");
		return ok && ok2 ? 0 : 1;
	}
	return 0;
}
