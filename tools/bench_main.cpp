// Headless benchmark for the simulation core.
//
//   tsim_bench [--cars N] [--ticks T] [--seed S] [--radius R] [--lanes L] [--report-every K]
//
// Prints the mean tick time and periodic traffic stats (to see stop-and-go
// waves form), then the final state hash. With no arguments it runs the golden
// scenario and prints PASS/FAIL against the expected hash.
#include "tsim/hash.h"
#include "tsim/map.h"
#include "tsim/sim.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace tsim;

int main(int argc, char **argv) {
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
		return ok ? 0 : 1;
	}
	return 0;
}
