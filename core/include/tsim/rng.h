// Seeded, platform-independent random numbers.
//
// std::mt19937 is portable but the std distributions are not, so the core uses
// its own generator (xoshiro256**, seeded with splitmix64) and only derives
// values with integer ops and exact double arithmetic.
#pragma once

#include <cstdint>

namespace tsim {

class Rng {
public:
	explicit Rng(uint64_t seed = 1) { reseed(seed); }

	void reseed(uint64_t seed) {
		uint64_t x = seed;
		for (uint64_t &s : state_) {
			s = splitmix64(x);
		}
	}

	uint64_t next_u64() {
		const uint64_t result = rotl(state_[1] * 5, 7) * 9;
		const uint64_t t = state_[1] << 17;
		state_[2] ^= state_[0];
		state_[3] ^= state_[1];
		state_[1] ^= state_[2];
		state_[0] ^= state_[3];
		state_[2] ^= t;
		state_[3] = rotl(state_[3], 45);
		return result;
	}

	// Uniform in [0, 1), exact: top 53 bits scaled by 2^-53.
	double uniform() { return static_cast<double>(next_u64() >> 11) * 0x1.0p-53; }

	// Uniform in [lo, hi).
	double range(double lo, double hi) { return lo + (hi - lo) * uniform(); }

	// Uniform in [-1, 1).
	double symmetric() { return 2.0 * uniform() - 1.0; }

	const uint64_t *state() const { return state_; }

private:
	static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
	static uint64_t splitmix64(uint64_t &x) {
		uint64_t z = (x += 0x9E3779B97F4A7C15ull);
		z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
		z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
		return z ^ (z >> 31);
	}

	uint64_t state_[4] = {};
};

} // namespace tsim
