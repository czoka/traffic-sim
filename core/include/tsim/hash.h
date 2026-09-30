// FNV-1a 64-bit hash used for determinism checks. Values are fed as explicit
// fixed-width integers so the result does not depend on size_t or endianness.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>

namespace tsim {

class StateHasher {
public:
	void add_u64(uint64_t v) {
		for (int i = 0; i < 8; ++i) {
			h_ ^= (v >> (i * 8)) & 0xFFu;
			h_ *= 0x100000001B3ull;
		}
	}
	void add_u32(uint32_t v) { add_u64(v); }
	void add_double(double d) {
		uint64_t bits;
		static_assert(sizeof(bits) == sizeof(d), "double must be 64-bit");
		std::memcpy(&bits, &d, sizeof(bits));
		add_u64(bits);
	}
	uint64_t value() const { return h_; }

private:
	uint64_t h_ = 0xCBF29CE484222325ull;
};

inline std::string hash_to_hex(uint64_t h) {
	static const char *digits = "0123456789abcdef";
	std::string out(16, '0');
	for (int i = 15; i >= 0; --i) {
		out[static_cast<size_t>(i)] = digits[h & 0xFu];
		h >>= 4;
	}
	return out;
}

} // namespace tsim
