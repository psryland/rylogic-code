//*****************************************************************************
// Common library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
//http://graphics.stanford.edu/~seander/bithacks.html
#pragma once
#include <type_traits>
#include <concepts>

namespace pr
{
	// Concept for 'BitField'
	template <typename T> concept BitField =
		std::is_integral_v<T> ||
		std::is_enum_v<T> ||
		requires(T t)
		{
			{ t |= t };
			{ t &= t };
		};

	// Unsigned int of the given size
	template <size_t N>
	using uintN_t = 
		std::conditional_t<N == 1, uint8_t,
		std::conditional_t<N == 2, uint16_t,
		std::conditional_t<N == 4, uint32_t,
		uint64_t>>>;

	// Convert to/from uint64 to uint32[2]
	constexpr uint64_t MakeLL(uint32_t hi, uint32_t lo)
	{
		return (static_cast<uint64_t>(hi) << 32) | lo;
	}
	constexpr std::tuple<uint32_t, uint32_t> BreakLL(uint64_t ll)
	{
		return {
			static_cast<uint32_t>((ll >> 32L) & 0xFFFFFFFF),
			static_cast<uint32_t>((ll) & 0xFFFFFFFF),
		};
	}

	// Bit manipulation functions
	constexpr uint32_t Bit32(int n)
	{
		return 1U << n;
	}
	constexpr uint64_t Bit64(int n)
	{
		return 1i64 << n;
	}

	// Sets the masked bits of 'value' to the state 'state'
	// If 'state' is boolean-true, returns 'value | mask'. If false, returns 'value & ~mask'
	// If 'state' is integral, then applies 'mask & state' to 'value'
	template <BitField T, BitField U, typename B> requires (BitField<B> || std::is_same_v<B,bool>)
	[[nodiscard]] constexpr T SetBits(T value, U mask, B state)
	{
		using UT = typename std::conditional_t<std::is_enum_v<T>, std::underlying_type<T>, std::type_identity<T>>::type;
		if constexpr (std::is_same_v<B, bool>)
		{
			return state
				? static_cast<T>(static_cast<UT>(value) | static_cast<UT>(mask))
				: static_cast<T>(static_cast<UT>(value) & ~static_cast<UT>(mask));
		}
		else
		{
			auto result = static_cast<UT>(value);
			result &= ~static_cast<UT>(mask);         // clear masked bits to zero
			result |=  static_cast<UT>(mask & state); // set bits from bit field
			return static_cast<T>(result);
		}
	}

	// Returns true if any bits in 'value & mask != 0'
	template <BitField T, BitField U>
	[[nodiscard]] constexpr bool AnySet(T value, U mask)
	{
		auto value64 = static_cast<uint64_t>(value);
		auto mask64 = static_cast<uint64_t>(mask);
		return (value64 & mask64) != 0;
	}

	// Return true if all bits in 'value & mask == mask'
	template <BitField T, BitField U>
	[[nodiscard]] constexpr bool AllSet(T value, U mask)
	{
		auto value64 = static_cast<uint64_t>(value);
		auto mask64 = static_cast<uint64_t>(mask);
		return (value64 & mask64) == mask64;
	}

	// Reverse the order of bits in 'v'
	constexpr uint8_t ReverseBits8(uint8_t v)
	{
		//if constexpr(sizeof(void*) == 8)
			return static_cast<uint8_t>(((v * 0x80200802ULL) & 0x0884422110ULL) * 0x0101010101ULL >> 32);
		//else
		//	return static_cast<uint8_t>(((v * 0x0802LU & 0x22110LU) | (n * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
	}

	// Reverse the order of bits in 'v'
	constexpr uint32_t ReverseBits32(uint32_t v)
	{
		v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);	// swap odd and even bits
		v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);	// swap consecutive pairs
		v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);	// swap nibbles ...
		v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);	// swap bytes
		v = ( v >> 16             ) | ( v               << 16);	// swap 2-byte long pairs
		return v;
	}

	// Reverse the order of the lower 'n' bits in 'v'. e.g. ReverseBits32(0b00101101, 4) returns 0b00101011
	constexpr uint32_t ReverseBits32(uint32_t v, int n)
	{
		return (v & (0xFFFFFFFFU << n)) | (ReverseBits32(v) >> (32 - n));
	}

	// Reverse the order of bits in 'v'
	constexpr uint64_t ReverseBits64(uint64_t v)
	{
		v = ((v >>  1) & 0x5555555555555555ULL) | ((v & 0x5555555555555555ULL) <<  1); // swap odd and even bits
		v = ((v >>  2) & 0x3333333333333333ULL) | ((v & 0x3333333333333333ULL) <<  2); // swap consecutive pairs
		v = ((v >>  4) & 0x0F0F0F0F0F0F0F0FULL) | ((v & 0x0F0F0F0F0F0F0F0FULL) <<  4); // swap nibbles ...
		v = ((v >>  8) & 0x00FF00FF00FF00FFULL) | ((v & 0x00FF00FF00FF00FFULL) <<  8); // swap bytes
		v = ((v >> 16) & 0x0000FFFF0000FFFFULL) | ((v & 0x0000FFFF0000FFFFULL) << 16); // swap 2-byte pairs
		v = ( v >> 32                         ) | ( v                          << 32); // swap 4-byte pairs
		return v;
	}

	// Reverse the order of the lower 'n' bits in 'v'. e.g. ReverseBits32(0b00101101, 4) returns 0b00101011
	constexpr uint64_t ReverseBits64(uint64_t v, int n)
	{
		return (v & (0xFFFFFFFFFFFFFFFFULL << n)) | (ReverseBits64(v) >> (64 - n));
	}

	// Returns a bit mask containing only the lowest bit of 'n'
	template <std::integral T> constexpr T LowBit(T n)
	{
		return n - ((n - 1) & n);
	}
	template <typename T> requires std::is_enum_v<T> constexpr T LowBit(T n, int = 0)
	{
		auto x = static_cast<std::underlying_type_t<T>>(n);
		return static_cast<T>(LowBit(x));
	}

	// Returns the bit position of the highest bit
	constexpr int HighBitIndex(uint64_t n)
	{
		if (n == 0) return -1;
		auto x = n; int shift = 0, pos = 0;
		shift = ((x & 0xFFFFFFFF00000000ULL) != 0) << 5; x >>= shift; pos |= shift;
		shift = ((x &         0xFFFF0000ULL) != 0) << 4; x >>= shift; pos |= shift;
		shift = ((x &             0xFF00ULL) != 0) << 3; x >>= shift; pos |= shift;
		shift = ((x &               0xF0ULL) != 0) << 2; x >>= shift; pos |= shift;
		shift = ((x &                0xCULL) != 0) << 1; x >>= shift; pos |= shift;
		shift = ((x &                0x2ULL) != 0) << 0; x >>= shift; pos |= shift;
		return pos;
	}

	// Returns the bit position of the lowest bit
	constexpr int LowBitIndex(uint64_t n)
	{
		auto low_bit = LowBit(n);
		return n != 0 ? HighBitIndex(low_bit) : -1;
	}

	// Returns the log2 of 'n' rounded down to the nearest integer
	constexpr int FloorLog2(uint64_t n)
	{
		return HighBitIndex(n);
	}

	// Returns the log2 of 'n' rounded up to the nearest integer
	constexpr int CeilLog2(uint64_t n)
	{
		return HighBitIndex(n) + 1;
	}

	// Returns the number of leading zeros in 'n' (64-bit)
	constexpr int LeadingZeros(uint64_t n)
	{
		if (n == 0) return 64;
		return 63 - FloorLog2(n);
	}

	// Return a bit mask contain only the highest bit of 'n'
	template <std::unsigned_integral T> constexpr T HighBit(T n)
	{
		// Must be a faster way?
		return T(1) << HighBitIndex(n);
	}

	// Returns true if 'n' is a exact power of two
	template <std::integral T> constexpr bool IsPowerOfTwo(T n)
	{
		// Zero is not a power of two because 2^n means "1 doubled n times". There is no number of times you can double 1 to get zero.
		// Incidentally, this is why '2^0 == 1', "1 doubled no times" is still 1.
		return (n & (n - 1)) == 0 && n != 0;
	}

	// Return the highest power of two that is <= 'n'
	template <std::integral T> constexpr T PowerOfTwoLessEqualTo(T n)
	{
		using U = std::make_unsigned_t<T>;

		assert(n > 0);
		return static_cast<T>(HighBit<U>(static_cast<U>(n)));
	}

	// Return the next highest power of two >= 'n'
	template <std::integral T> constexpr T PowerOfTwoGreaterEqualTo(T n)
	{
		using U = std::make_unsigned_t<T>;

		assert(n >= 0 && n <= T(1) << (sizeof(T) * 8 - 1 - std::is_signed_v<T>)); // (1 << 31) is -1
		return T(1) << std::bit_width<U>(static_cast<U>(n + (n == 0) - 1));
	}

	// Returns the number of set bits in 'n'
	template <typename T> constexpr int CountBits(T n)
	{
		int count = 0;
		while (n)
		{
			++count;
			n &= (n - 1);
		}
		return count;
	}
	template <> constexpr int CountBits(unsigned int n)
	{
		// http://infolab.stanford.edu/~manku/bitcount/bitcount.html
		// Constant time bit count works for 32-bit numbers only.
		// Fix last line for 64-bit numbers
		unsigned int tmp = n
			- ((n >> 1) & 033333333333)
			- ((n >> 2) & 011111111111);
		return ((tmp + (tmp >> 3)) & 030707070707) % 63;
	}

	// Interleaves the lower 16 bits of x and y, so the bits of x
	// are in the even positions and bits from y in the odd.
	// Returns the resulting 32-bit Morton Number.
	inline unsigned int InterleaveBits(unsigned int x, unsigned int y)
	{
		const unsigned int B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
		const unsigned int S[] = {1, 2, 4, 8};
		x = (x | (x << S[3])) & B[3];
		x = (x | (x << S[2])) & B[2];
		x = (x | (x << S[1])) & B[1];
		x = (x | (x << S[0])) & B[0];
		y = (y | (y << S[3])) & B[3];
		y = (y | (y << S[2])) & B[2];
		y = (y | (y << S[1])) & B[1];
		y = (y | (y << S[0])) & B[0];
		return x | (y << 1);
	}

	// Pack a value into 'bits' in the range [lo, hi).
	// 'lo' is the LSB, 'hi' is one past the MSB.
	// e.g. PackBits(0b00000001, 0b101, 6, 3) => 0b00101001
	template <std::integral T, std::unsigned_integral U>
	[[nodiscard]] constexpr U PackBits(U bits, T value, int hi, int lo)
	{
		const U mask = (1ULL << (hi - lo)) - 1;
		return static_cast<U>((bits & ~(mask << lo)) | ((value & mask) << lo));
	}

	// Unpack a value from 'bits' in the range [lo, hi).
	// 'lo' is the LSB, 'hi' is one past the MSB.
	// e.g. GrabBits(0b00101001, 6, 3) => 0b101
	template <std::integral T, std::unsigned_integral U>
	[[nodiscard]] constexpr T GrabBits(U bits, int hi, int lo)
	{
		const U mask = (1ULL << (hi - lo)) - 1;
		return static_cast<T>((bits >> lo) & mask);
	}

	// Extract the bit range [hi,lo] (inclusive) from 'value'.
	// 'hi' and 'lo' are zero-based bit indices. The returned result is shifted down by .lo'.
	// e.g. Bits(0b11111111, 6, 3) => 0b01111000 => 0b0001111
	template <typename T> [[deprecated("Use GrabBits")]] constexpr T Bits(unsigned long long value, int hi, int lo)
	{
		unsigned long long mask = (1ULL << (hi - lo + 1)) - 1;
		return static_cast<T>((value >> lo) & mask);
	}

	// Move 'value' to the range [hi,lo] (inclusive) (masking if necessary).
	// 'hi' and 'lo' are zero-based bit indices.
	// e.g. BitStuff(0b11111111, 6, 3) => 0b00001111 => 0b01111000
	template <typename T> [[deprecated("Use PackBits")]] constexpr T BitStuff(unsigned long long value, int hi, int lo)
	{
		unsigned long long mask = (1ULL << (hi - lo + 1)) - 1;
		return static_cast<T>((value & mask) << lo);
	}

	// Convert a string of 1s and 0s into a bitmask
	template <typename T> inline T BitsFromString(std::string_view bits)
	{
		T n = 0;
		for (auto b : bits) n = (n << 1) | T(b == '1');
		return n;
	}

	// Convert an integral type into a string of 0s and 1s
	template <typename T> inline char const* BitsToString(T bits, bool leading_zeros = false)
	{
		thread_local static char str[sizeof(T) * 8 + 1];
		typedef std::make_unsigned<T>::type UT;
		UT mask = UT(1) << (sizeof(T) * 8 - 1);
		
		if (!leading_zeros) // skip leading zeros
			for (; (mask & bits) == 0; mask >>= 1) {}

		char* ptr = str;
		if (mask == 0)
		{
			*ptr++ = '0';
		}
		else
		{
			for (; mask != 0; mask >>= 1)
				*ptr++ = (mask & bits) != 0 ? '1' : '0';
		}
		*ptr = 0;
		return str;
	}

	// Iterator for iterating over a bit mask
	template <typename T> struct bit_citer
	{
		typedef std::forward_iterator_tag iterator_category;
		typedef T value_type;

		value_type m_bits;
		bit_citer(value_type bits) :m_bits(bits) {}

		value_type    operator * () const                   { return LowBit(m_bits); }
		bit_citer<T>& operator ++()                         { m_bits ^= LowBit(m_bits); return *this; }
		bit_citer<T>  operator ++(int)                      { bit_citer<T> iter(m_bits); ++*this; return iter; }
		bool          operator == (bit_citer<T> const& rhs) { return m_bits == rhs.m_bits; }
		bool          operator != (bit_citer<T> const& rhs) { return m_bits != rhs.m_bits; }
	};

	// An iterator over bits in a bit mask
	template <typename T> struct BitEnumerator
	{
		T m_bits;
		BitEnumerator(T bits) :m_bits(bits) {}
		bit_citer<T> begin() const { return bit_citer<T>(m_bits); }
		bit_citer<T> end() const   { return bit_citer<T>(static_cast<T>(0)); }
	};

	// Returns an object for enumerating over the set bits in 'bits'
	template <typename T> BitEnumerator<T> EnumerateBits(T bits)
	{
		return BitEnumerator<T>(bits);
	}

	// Decompose/Compose an IEEE754 double into the sign, mantissa, exponent
	inline std::tuple<int, int32_t, int64_t> Decompose(double x, bool raw = false)
	{
		//    Normal numbers: (-1)^sign x 2^(e - 1023) x 1.fraction
		// Subnormal numbers: (-1)^sign x 2^(1 - 1023) x 0.fraction

		// Translate the double into sign, exponent, and mantissa.
		auto bits = reinterpret_cast<uint64_t const&>(x);
		auto sign = GrabBits<int>(bits, 64, 63) != 0 ? -1 : +1;
		auto exponent = GrabBits<int32_t>(bits, 63, 52);
		auto mantissa = GrabBits<int64_t>(bits, 52, 0);
		if (!raw)
		{
			// Normal numbers: add the '1' to the front of the mantissa.
			if (exponent != 0)
				mantissa |= 1ULL << 52;
			else
				exponent++;

			// Bias the exponent
			exponent -= 1023;
		}
		return std::tie(sign, exponent, mantissa);
	}
	inline double Compose(int sign, int32_t exponent, int64_t mantissa, bool raw = false)
	{
		//    Normal numbers: (-1)^sign x 2^(e - 1023) x 1.fraction
		// Subnormal numbers: (-1)^sign x 2^(1 - 1023) x 0.fraction

		// Translate the sign, exponent, and mantissa into a double.
		if (!raw)
		{
			// Bias the exponent
			exponent += 1023;

			// Normal numbers: remove the '1' from the front of the mantissa.
			if (exponent != 0)
				mantissa &= ~(1ULL << 52);
			else
				exponent--;
		}
		auto bits = 0ULL;
		bits = PackBits(bits, int(sign < 0), 64, 63);
		bits = PackBits(bits, exponent, 63, 52);
		bits = PackBits(bits, mantissa, 52, 0);
		return reinterpret_cast<double const&>(bits);
	}

	// Decompose an IEEE754 float into the sign, mantissa, exponent
	inline std::tuple<int, int32_t, int32_t> Decompose(float x, bool raw = false)
	{
		//    Normal numbers: (-1)^sign x 2^(e - 127) x 1.fraction
		// Subnormal numbers: (-1)^sign x 2^(1 - 127) x 0.fraction

		// Translate the double into sign, exponent, and mantissa.
		auto bits = reinterpret_cast<uint32_t const&>(x);
		auto sign = GrabBits<int>(bits, 32, 31) != 0 ? -1 : +1;
		auto exponent = GrabBits<int32_t>(bits, 31, 23);
		auto mantissa = GrabBits<int32_t>(bits, 23, 0);
		if (!raw)
		{
			// Normal numbers: add the '1' to the front of the mantissa.
			if (exponent != 0)
				mantissa |= 1U << 23;
			else
				exponent++;

			// Bias the exponent
			exponent -= 127;
		}
		return std::tie(sign, exponent, mantissa);
	}
	inline float Compose(int sign, int32_t exponent, int32_t mantissa, bool raw = false)
	{
		//    Normal numbers: (-1)^sign x 2^(e - 127) x 1.fraction
		// Subnormal numbers: (-1)^sign x 2^(1 - 127) x 0.fraction

		// Translate the sign, exponent, and mantissa into a float.
		if (!raw)
		{
			// Bias the exponent
			exponent += 127;

			// Normal numbers: remove the '1' from the front of the mantissa.
			if (exponent != 0)
				mantissa &= ~(1U << 23);
			else
				exponent--;
		}
		auto bits = 0U;
		bits = PackBits(bits, int(sign < 0), 32, 31);
		bits = PackBits(bits, exponent, 31, 23);
		bits = PackBits(bits, mantissa, 23, 0);
		return reinterpret_cast<float const&>(bits);
	}

	// A little bit of evil to preserve the sign in a float at the cost of precision
	template <std::floating_point T> inline T StoreSignInLSB(T x, T sign)
	{
		union Bits { T f; uintN_t<sizeof(T)> u; };

		// Set the LSB of the float's mantissa to 1 for positive or 0 for negative
		Bits bits = { .f = x };
		bits.u = (bits.u & ~1u) | (sign >= 0 ? 1u : 0u);
		return bits.f;
	}
	template <std::floating_point T> inline T ReadSignFromLSB(T x)
	{
		union Bits { T f; uintN_t<sizeof(T)> u; };

		// Set the LSB of the float's mantissa to 1 for positive or 0 for negative
		Bits bits = { .f = x };
		return (bits.u & 1u) != 0 ? +1.0f : -1.0f;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTestClass(BitFunctionTests)
	{
		PRUnitTestMethod(MakeLL)
		{
			auto [hi,lo] = BreakLL(0x0123456789abcdef);
			PR_EXPECT(hi == 0x01234567U);
			PR_EXPECT(lo == 0x89abcdefU);

			auto ll = MakeLL(hi, lo);
			PR_EXPECT(ll == 0x0123456789abcdefULL);
		}
		PRUnitTestMethod(BitsFromString)
		{
			std::string_view mask_str = "1001010011";
			auto mask = BitsFromString<long long>(mask_str);
			PR_EXPECT(BitsToString(mask) == mask_str);

			std::vector<long long> bits;
			for (auto b : pr::EnumerateBits(mask))
				bits.push_back(b);

			PR_EXPECT(bits.size() == 5U);
			PR_EXPECT(bits[0] == 1U << 0);
			PR_EXPECT(bits[1] == 1U << 1);
			PR_EXPECT(bits[2] == 1U << 4);
			PR_EXPECT(bits[3] == 1U << 6);
			PR_EXPECT(bits[4] == 1U << 9);
		}
		PRUnitTestMethod(PackGrabBits)
		{
			uint8_t bits = 0;
			bits = PackBits(bits, 0b101, 6, 3);
			PR_EXPECT(bits == 0b00101000);
			PR_EXPECT(GrabBits<uint8_t>(bits, 6, 3) == 0b101);

			bits = 0b11111100;
			bits = PackBits(bits, 0b101, 6, 3);
			PR_EXPECT(bits == 0b11101100);
			PR_EXPECT(GrabBits<uint8_t>(bits, 6, 3) == 0b101);
		}
		PRUnitTestMethod(FloorLog2)
		{
			auto n0 = 0b1; // 1
			PR_EXPECT(FloorLog2(n0) == 0);
			auto n1 = 0b10; // 2
			PR_EXPECT(FloorLog2(n1) == 1);
			auto n2 = 0b1000000; // 64
			PR_EXPECT(FloorLog2(n2) == 6);
			auto n3 = 0b101010101010101ULL; // 21845
			PR_EXPECT(FloorLog2(n3) == 14);
			auto n4 = 0xFFFFFFFFFFFFFFFFULL; // 18446744073709551615
			PR_EXPECT(FloorLog2(n4) == 63);
		}
		PRUnitTestMethod(HighLowBits)
		{
			std::string_view mask_str = "1001110010";
			auto mask = BitsFromString<uint32_t>(mask_str);
			PR_EXPECT(mask == 626U);
			PR_EXPECT(BitsToString(mask) == mask_str);
			PR_EXPECT(HighBitIndex(mask) == 9);
			PR_EXPECT(LowBitIndex(mask) == 1);
			PR_EXPECT(LowBit(mask) == 2U);
			PR_EXPECT(HighBit(mask) == 0x200U);
		}
		PRUnitTestMethod(BitIndex)
		{
			auto bits = 0b001011000100;
			PR_EXPECT(HighBitIndex(bits) == 9);
			PR_EXPECT(LowBitIndex(bits) == 2);
			PR_EXPECT((1ULL << FloorLog2(bits)) == 0b001000000000ULL);
			PR_EXPECT((1ULL << CeilLog2(bits))  == 0b010000000000ULL);
			PR_EXPECT(LeadingZeros(bits) == 63 - 9);
		
			std::string_view mask_str = "1111010100010";
			auto mask = BitsFromString<uint16_t>(mask_str);
			PR_EXPECT(mask == 7842);
			PR_EXPECT(HighBitIndex(mask) == 12);
			PR_EXPECT(LowBitIndex(mask) == 1);
			PR_EXPECT(LowBit(mask) == 2);
			PR_EXPECT(HighBit(mask) == 0x1000);
		}
		PRUnitTestMethod(PowerOfTwo)
		{
			std::string_view mask_str = "1001001100110010101010010100111010010110010101110110000110100100";
			auto mask = BitsFromString<unsigned long long>(mask_str);
			PR_EXPECT(mask == 0x9332A94E965761A4ULL);
			PR_EXPECT(HighBitIndex(mask) == 63);
			PR_EXPECT(LowBitIndex(mask) == 2);
			PR_EXPECT(LowBit(mask) == 4);
			PR_EXPECT(HighBit(mask) == 0x8000000000000000ULL);

			PR_EXPECT(PowerOfTwoLessEqualTo(1) == 1);
			PR_EXPECT(PowerOfTwoLessEqualTo(256) == 256);
			PR_EXPECT(PowerOfTwoLessEqualTo(255) == 128);

			PR_EXPECT(PowerOfTwoGreaterEqualTo(0) == 1);
			PR_EXPECT(PowerOfTwoGreaterEqualTo(256) == 256);
			PR_EXPECT(PowerOfTwoGreaterEqualTo(0x12345678) == 0x20000000);
			PR_EXPECT(PowerOfTwoGreaterEqualTo(0x7FFFFFFFU) == 0x80000000U);
			PR_EXPECT(PowerOfTwoGreaterEqualTo(0x9876543210UL) == 0x10000000000UL);
			PR_EXPECT(PowerOfTwoGreaterEqualTo(int16_t(0x9a)) == int16_t(0x100));
			PR_EXPECT(PowerOfTwoGreaterEqualTo(uint16_t(0x9a)) == uint16_t(0x100));
		}
		PRUnitTestMethod(Reverseits)
		{
			{
				auto a = uint8_t(0b10110101);
				auto b = uint8_t(0b10101101);
				auto c = ReverseBits8(a);
				PR_EXPECT(b == c);
			}
			{            //01234567890123456789012345678901 32bits
				auto a = 0b01100011110000011111100000001111U;

				auto b = ReverseBits32(a);
				auto B = 0b11110000000111111000001111000110U;
				PR_EXPECT(b == B);

				auto c = ReverseBits32(a, 8); // just the lower 8 bits
				auto C = 0b01100011110000011111100011110000U;
				PR_EXPECT(c == C);
			}
			{            //0123456789_123456789_123456789_123456789_123456789_123456879_123 64bits
				auto a = 0b0110001111000001111110000000111111110000000001111111111000000000ULL;

				auto b = ReverseBits64(a);
				auto B = 0b0000000001111111111000000000111111110000000111111000001111000110ULL;
				PR_EXPECT(b == B);

				auto c = ReverseBits64(a, 12); // just the lower 12 bits
				auto C = 0b0110001111000001111110000000111111110000000001111111000000000111ULL;
				PR_EXPECT(c == C);
			}
		}
		PRUnitTestMethod(IEEEFloats)
		{
			{
				auto d1 = -9.887654321e126;
				auto [sign, exponent, mantissa] = Decompose(d1);
				auto d2 = Compose(sign, exponent, mantissa);
				PR_EXPECT(d1 == d2);
				PR_EXPECT(sign == -1);
				PR_EXPECT(exponent == 421);
				PR_EXPECT(mantissa == 0x001d36ae824ee75f);
			} {
				auto f1 = -9.887654321e25f;
				auto [sign, exponent, mantissa] = Decompose(f1);
				auto f2 = Compose(sign, exponent, mantissa);
				PR_EXPECT(f1 == f2);
				PR_EXPECT(sign == -1);
				PR_EXPECT(exponent == 86);
				PR_EXPECT(mantissa == 0x00a393d8);
			}
		}
	};
}
#endif
