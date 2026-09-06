//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Black-box checksum used to consume every value extracted by a workload driver.
//
// Style Guidance:
//  - Every value read from either reader must flow through 'Checksum::Add' before the parse
//    result is returned. This is what lets 'g_black_hole' (below) keep the whole extraction
//    loop observable to the optimizer, so Release/LTCG cannot fold a workload down to nothing.
//  - The hash itself has no cryptographic purpose; it only needs to be a stable, order-sensitive
//    function of the extracted values so that two backends parsing identical source bytes
//    produce identical checksums, and any divergence in parsed content is reliably detected.
#pragma once
#include "forward.h"

namespace pr::script_bench
{
	// FNV-1a 64-bit accumulator over every extracted value in a workload.
	struct Checksum
	{
		uint64_t m_value = 1469598103934665603ULL; // FNV-1a 64-bit offset basis

		// Fold one byte into the running hash.
		void AddByte(uint8_t b) noexcept
		{
			m_value ^= b;
			m_value *= 1099511628211ULL; // FNV-1a 64-bit prime
		}
		void Add(uint64_t v) noexcept
		{
			for (int i = 0; i != 8; ++i)
			{
				AddByte(uint8_t(v));
				v >>= 8;
			}
		}
		void Add(int64_t v) noexcept
		{
			Add(uint64_t(v));
		}
		void Add(int32_t v) noexcept
		{
			Add(int64_t(v));
		}
		void Add(bool v) noexcept
		{
			Add(uint64_t(v ? 1 : 0));
		}
		void Add(double v) noexcept
		{
			// Hash the exact bit pattern so the two backends only agree when they parsed to the same value.
			uint64_t bits;
			std::memcpy(&bits, &v, sizeof(bits));
			Add(bits);
		}
		void Add(float v) noexcept
		{
			// Widen to double first: this is an exact, lossless conversion for every finite float, so
			// two backends that parsed the same decimal text to equal float values hash identically
			// regardless of which of 'float'/'double' a given extraction call happened to use.
			Add(double(v));
		}
		void Add(std::string_view s) noexcept
		{
			for (unsigned char c : s)
				AddByte(c);

			// Fold the length in too, so e.g. adjacent empty/short strings can't alias longer ones.
			Add(uint64_t(s.size()));
		}
	};

	// Global sink that every measured parse XORs its checksum into. An atomic with relaxed ordering is
	// used purely to give the compiler an observable, non-elidable side effect (this benchmark is
	// single-threaded); this is what stops full optimization/LTCG from proving a workload's result is
	// unused and eliminating the parsing work it exists to measure.
	inline std::atomic<uint64_t> g_black_hole{ 0 };
}
