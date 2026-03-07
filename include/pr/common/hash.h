//*************************************
// Hash
//  Copyright (c) Rylogic Ltd 2009
//*************************************
// The main purpose of this header is compile time string hashing.
// For runtime hashing, prefer 'std::hash<T>{}(...)'
// e.g. std::hash<Thing>{}(&thing);
// The compile time hash below uses the same algorithm (FNV-1a hash) and
// magic numbers as std::hash. Note, though, that std::hash uses different
// magic values for 64bit and 32bit builds.
// WARNING: 
//  Careful with std::hash<>, it's not specialised for raw strings.
//  'std::hash<char const*>{}(str)' finds the hash of the pointer, not the string.

#pragma once
#include <cstdint>
#include <iterator>
#include <concepts>
#include <string_view>
#include <type_traits>
#include <cassert>

namespace pr::hash
{
	// 'HashValue' is a signed int so that comparisons
	// with enum values don't generate signed/unsigned warnings
	using HashValue32 = int;
	using HashValue64 = long long;

	// Magic numbers used by the FNV-1a algorithm
	inline static constexpr uint64_t FNV_offset_basis64 = 14695981039346656037ULL;
	inline static constexpr uint64_t FNV_prime64 = 1099511628211ULL;
	inline static constexpr uint32_t FNV_offset_basis32 = 2166136261U;
	inline static constexpr uint32_t FNV_prime32 = 16777619U;

	// Concepts for strings/PODs/chars
	template <typename Ty> concept CharType =
		::std::is_same_v<std::remove_cv_t<Ty>, char> ||
		::std::is_same_v<std::remove_cv_t<Ty>, wchar_t> ||
		::std::is_same_v<std::remove_cv_t<Ty>, char8_t> ||
		::std::is_same_v<std::remove_cv_t<Ty>, char16_t> ||
		::std::is_same_v<std::remove_cv_t<Ty>, char32_t>;
	template <typename Ty> concept PodType =
		::std::is_standard_layout_v<Ty> &&
		::std::is_trivially_copyable_v<Ty> &&
		!::std::is_pointer_v<Ty>;
	template <typename Ty> concept PodType32 =
		sizeof(Ty) <= sizeof(uint32_t) &&
		PodType<Ty>;
	template <typename Ty> concept PodType64 =
		sizeof(Ty) <= sizeof(uint64_t) &&
		PodType<Ty>;

	namespace impl
	{
		// 32 bit multiply without an overflow warning...
		constexpr uint32_t Mul32(uint32_t a, uint32_t b)
		{
			return uint32_t((uint64_t(a) * uint64_t(b)) & ~0U);
		}
		static_assert(Mul32(0x12345678, 0x12345678) == 0x1DF4D840);

		// 64 bit multiply without an overflow warning...
		constexpr uint64_t Lo32(uint64_t x) { return (x) & 0xFFFFFFFFU; }
		constexpr uint64_t Hi32(uint64_t x) { return (x >> 32) & 0xFFFFFFFFU; }
		constexpr uint64_t Mul64(uint64_t a, uint64_t b)
		{
			constexpr uint32_t ffffffff = 0xFFFFFFFFU;
			auto ab = Lo32(a) * Lo32(b);
			auto aB = Lo32(a) * Hi32(b);
			auto Ab = Hi32(a) * Lo32(b);
			auto hi = ((((Hi32(ab) + aB) & ffffffff) + Ab) & ffffffff) << 32;
			auto lo = ab & ffffffff;
			return hi + lo;
		}
		static_assert(Mul64(0x1234567887654321, 0x1234567887654321) == 0x290D0FCAD7A44A41);

		// Dependent value for static asserts
		template <typename T, bool Value> constexpr bool dependent = Value;

		// Convert 'ch' to lower case
		template <CharType Ty> constexpr Ty Lower(Ty ch)
		{
			return ch + 32 * (ch >= 'A' && ch <= 'Z');
		}
	}

	// Compile Time ***************************************************************************

	// Notes:
	//  - *DON'T* try to hash blocks of 4/8 bytes at a time. Whatever the unit size is (u8, u16, etc) you have to hash
	//    each unit individually. This is because the same data should hash to the same value no matter where it is in memory.
	//    If you hash leading bytes, then blocks, then trailing bytes, the memory locality will affect the hash value.
	//  - Compile time hash functions are not focused on being fast, just compile time is enough.
	//  - If comparing runtime string hashes with compile time string hashes, make sure you use the compile time
	//    hash functions for the runtime strings as well.

	// Compile time hash and accumulate 
	constexpr uint32_t Hash32CT(uint32_t ch, uint32_t h)
	{
		return impl::Mul32(h ^ ch, FNV_prime32);
	}
	constexpr uint64_t Hash64CT(uint64_t ch, uint64_t h)
	{
		return impl::Mul64(h ^ ch, FNV_prime64);
	}

	// String range hash
	template <CharType Ty> constexpr uint32_t Hash32CT(Ty const* str, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		for (; str != end; ++str) h = Hash32CT(static_cast<uint32_t>(*str), h);
		return h;
	}
	template <CharType Ty> constexpr uint64_t Hash64CT(Ty const* str, Ty const* end, uint64_t h = FNV_offset_basis64)
	{
		for (; str != end; ++str) h = Hash64CT(static_cast<uint64_t>(*str), h);
		return h;
	}

	// Sentinel string hash
	template <CharType Ty> constexpr uint32_t Hash32CT(Ty const* str, Ty term = Ty(), uint32_t h = FNV_offset_basis32)
	{
		for (; *str != term; ++str) h = Hash32CT(static_cast<uint32_t>(*str), h);
		return h;
	}
	template <CharType Ty> constexpr uint64_t Hash64CT(Ty const* str, Ty term = Ty(), uint64_t h = FNV_offset_basis64)
	{
		for (; *str != term; ++str) h = Hash64CT(static_cast<uint64_t>(*str), h);
		return h;
	}

	// Case insensitive sentinel string hash
	template <CharType Ty> constexpr uint32_t HashI32CT(Ty const* str, Ty term = Ty(), uint32_t h = FNV_offset_basis32)
	{
		for (; *str != term; ++str) h = Hash32CT(static_cast<uint32_t>(impl::Lower(*str)), h);
		return h;
	}
	template <CharType Ty> constexpr uint64_t HashI64CT(Ty const* str, Ty term = Ty(), uint64_t h = FNV_offset_basis64)
	{
		for (; *str != term; ++str) h = Hash64CT(static_cast<uint64_t>(impl::Lower(*str)), h);
		return h;
	}

	// Case insensitive character range hash - Ty must be a character type
	template <CharType Ty> constexpr uint32_t HashI32CT(Ty const* str, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		for (; str != end; ++str) h = Hash32CT(static_cast<uint32_t>(impl::Lower(*str)), h);
		return h;
	}
	template <CharType Ty> constexpr uint64_t HashI64CT(Ty const* str, Ty const* end, uint64_t h = FNV_offset_basis64)
	{
		for (; str != end; ++str) h = Hash64CT(static_cast<uint64_t>(impl::Lower(*str)), h);
		return h;
	}

	// Default hash = 32 bit hash, even on 64bit builds for consistency
	template <CharType Ty> constexpr HashValue32 HashCT(Ty const* str, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		return static_cast<HashValue32>(Hash32CT(str, end, h));
	}
	template <CharType Ty> constexpr HashValue32 HashCT(Ty const* str, Ty term = Ty(), uint32_t h = FNV_offset_basis32)
	{
		return static_cast<HashValue32>(Hash32CT(str, term, h));
	}
	template <CharType Ty> constexpr HashValue32 HashICT(Ty const* str, Ty term = Ty(), uint32_t h = FNV_offset_basis32)
	{
		return static_cast<HashValue32>(HashI32CT(str, term, h));
	}
	template <CharType Ty> constexpr HashValue32 HashICT(Ty const* str, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		return static_cast<HashValue32>(HashI32CT(str, end, h));
	}

	// string to hash value
	constexpr HashValue32 operator ""_hash(char const* str, size_t n)
	{
		return HashCT(str, str + n);
	}
	constexpr HashValue32 operator ""_hashi(char const* str, size_t n)
	{
		return HashICT(str, str + n);
	}
	constexpr HashValue32 operator ""_hash(wchar_t const* str, size_t n)
	{
		return HashCT(str, str + n);
	}
	constexpr HashValue32 operator ""_hashi(wchar_t const* str, size_t n)
	{
		return HashICT(str, str + n);
	}

	//template <unsigned int N> class C; C<HashI("ABC")> c;
	static_assert(HashCT("ABC") == 1552166763U, "Compile time CRC failed");
	static_assert(HashICT("ABC") == 440920331U, "Compile time CRC failed");
	static_assert("hash me!"_hash == HashCT("hash me!"));
	static_assert("HaSh Me ToO!"_hashi == HashICT("hash me too!"));
	static_assert(L"hash me!"_hash == HashCT(L"hash me!"));
	static_assert(L"HaSh Me ToO!"_hashi == HashICT(L"hash me too!"));

	// Run Time *******************************************************************************

	// String range hash
	template <CharType Ty> inline HashValue32 Hash(Ty const* str, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		for (; str != end; ++str) h = Hash32CT(static_cast<uint32_t>(*str), h);
		return h;
	}
	template <CharType Ty> inline HashValue32 HashI(Ty const* str, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		for (; str != end; ++str) h = Hash32CT(static_cast<uint32_t>(impl::Lower(*str)), h);
		return h;
	}

	// Hash a sentinel string
	template <CharType Ty> inline HashValue32 Hash(Ty const* str, Ty term = Ty(), uint32_t h = FNV_offset_basis32)
	{
		for (; *str != term; ++str) h = Hash32CT(static_cast<uint32_t>(*str), h);
		return h;
	}
	template <CharType Ty> inline HashValue32 HashI(Ty const* str, Ty term = Ty(), uint32_t h = FNV_offset_basis32)
	{
		for (; *str != term; ++str) h = Hash32CT(static_cast<uint32_t>(impl::Lower(*str)), h);
		return h;
	}

	// Hash a string_view
	inline HashValue32 Hash(std::string_view str, uint32_t h = FNV_offset_basis32)
	{
		return Hash(str.data(), str.data() + str.size(), h);
	}
	inline HashValue32 Hash(std::wstring_view str, uint32_t h = FNV_offset_basis32)
	{
		return Hash(str.data(), str.data() + str.size(), h);
	}

	// Hash a range of bytes
	namespace impl
	{
		template <typename Ret> requires (std::is_same_v<Ret, HashValue32> || std::is_same_v<Ret, HashValue64>)
		inline Ret HashBytes(void const* ptr, void const* end, Ret h)
		{
			// See comments above. Don't try to hash blocks of 4/8 bytes at
			// a time as this causes memory locality to affect the hash value.
			assert(ptr != nullptr && end != nullptr && ptr <= end);
			using URet = std::make_unsigned_t<Ret>;
			constexpr auto HashCT = [](uint8_t x, URet h)
			{
				if constexpr (std::is_same_v<Ret, HashValue32>)
					return Hash32CT(x, h);
				if constexpr (std::is_same_v<Ret, HashValue64>)
					return Hash64CT(x, h);
			};

			union { void const* vptr; uint8_t const* byte; } p = { ptr }, e = { end };
			for (; p.byte != e.byte; ++p.byte)
				h = HashCT(*p.byte, h);

			return h;
		}
	}
	inline HashValue32 HashBytes32(void const* ptr, void const* end, uint32_t h = FNV_offset_basis32)
	{
		return impl::HashBytes<HashValue32>(ptr, end, h);
	}
	inline HashValue64 HashBytes64(void const* ptr, void const* end, uint64_t h = FNV_offset_basis64)
	{
		return impl::HashBytes<HashValue64>(ptr, end, h);
	}

	// Hash PODs given as arguments. Careful with hidden padding
	template <typename... Args> inline HashValue32 HashArgs32(Args&&... args)
	{
		auto h = FNV_offset_basis32;
		auto hash_one = [&](auto& arg)
		{
			using ArgTy = std::remove_cvref_t<decltype(arg)>;
			using Ty = std::decay_t<ArgTy>;

			// Pod types
			if constexpr (PodType<Ty>)
				h = HashBytes32(&arg, &arg + 1, h);

			// A pointer to a null terminated string
			else if constexpr (std::is_pointer_v<Ty> && CharType<std::remove_pointer_t<Ty>>)
				h = Hash32CT(arg, std::remove_pointer_t<Ty>(), h);

			// An array of pods
			else if constexpr (std::is_bounded_array_v<ArgTy> && PodType<std::remove_pointer_t<Ty>>)
				h = HashBytes32(&arg[0], &arg[0] + _countof(arg), h);

			// Not supported
			else
				static_assert(impl::dependent<Ty, false>, "Unsupported type for HashArgs()");
		};
		(hash_one(args), ...);
		return h;
	}
	template <typename... Args> inline HashValue64 HashArgs64(Args&&... args)
	{
		auto h = FNV_offset_basis64;
		auto hash_one = [&](auto& arg)
		{
			using ArgTy = std::remove_cvref_t<decltype(arg)>;
			using Ty = std::decay_t<ArgTy>;

			// Pod types
			if constexpr (PodType<Ty>)
				h = HashBytes64(&arg, &arg + 1, h);

			// An array of pods
			else if constexpr (std::is_bounded_array_v<ArgTy> && PodType<std::remove_pointer_t<Ty>>)
				h = HashBytes64(&arg[0], &arg[0] + _countof(arg), h);

			// A pointer to a null terminated string
			else if constexpr (std::is_pointer_v<Ty> && CharType<std::remove_pointer_t<Ty>>)
				h = Hash64CT(arg, std::remove_pointer_t<Ty>(), h);

			// Not supported
			else
				static_assert(impl::dependent<Ty, false>, "Unsupported type for Hash()");
		};
		(hash_one(args), ...);
		return h;
	}
	template <typename... Args> inline HashValue32 HashArgs(Args&&... args)
	{
		auto h = HashArgs32(std::forward<Args>(args)...);
		return h;
	}

	// Hash in blocks of 16 bits
	// http://www.azillionmonkeys.com/qed/hash.html, (c) Copyright 2004-2008 by Paul Hsieh
	inline HashValue32 HsiehHash16(void const* data, size_t len, HashValue32 hash = -1)
	{
		if (len == 0 || data == nullptr)
			return hash;

		// Local function for reading 16bit chunks of 'data'
		auto Read16bits = [](uint8_t const* d)
		{
			#if (defined(__GNUC__) && defined(__i386__)) || defined(__WATCOMC__) || defined(_MSC_VER) || defined (__BORLANDC__) || defined (__TURBOC__)
			return *reinterpret_cast<uint16_t const*>(d);
			#else
			return (static_cast<uint32_t>(reinterpret_cast<uint8_t const*>(d)[1]) << 8) +
					static_cast<uint32_t>(reinterpret_cast<uint8_t const*>(d)[0]);
			#endif
		};

		auto ptr = static_cast<uint8_t const*>(data);
		uint32_t tmp;
		uint32_t rem = len & 3;
		len >>= 2;

		// Main loop
		while (len-- != 0)
		{
			hash +=  Read16bits(ptr);
			tmp   = (Read16bits(ptr + 2) << 11) ^ hash;
			hash  = (hash << 16) ^ tmp;
			ptr  += 2 * sizeof(uint16_t);
			hash += hash >> 11;
		}

		// Handle end cases
		switch (rem)
		{
			case 3: hash += Read16bits(ptr);
				hash ^= hash << 16;
				hash ^= ptr[sizeof(uint16_t)] << 18;
				hash += hash >> 11;
				break;
			case 2: hash += Read16bits(ptr);
				hash ^= hash << 11;
				hash += hash >> 17;
				break;
			case 1: hash += *ptr;
				hash ^= hash << 10;
				hash += hash >> 1;
				break;
		}

		// Force "avalanching" of final 127 bits
		hash ^= hash << 3;
		hash += hash >> 5;
		hash ^= hash << 4;
		hash += hash >> 17;
		hash ^= hash << 25;
		hash += hash >> 6;
		return hash;
	}

	// MurmurHash2, by Austin Appleby
	// Note - This code makes a few assumptions about how your machine behaves -
	// 1. We can read a 4-byte value from any address without crashing
	// 2. sizeof(int) == 4
	// And it has a few limitations -
	// 1. It will not work incrementally.
	// 2. It will not produce the same results on little-endian and big-endian machines.
	inline HashValue32 MurmurHash2_32(void const* key, int len, HashValue32 seed = -1)
	{
		// 'm' and 'r' are mixing constants generated offline.
		// They're not really 'magic', they just happen to work well.
		uint32_t const m = 0x5bd1e995;
		int32_t const r = 24;

		// Initialize the hash to a 'random' value
		uint32_t h = seed ^ len;

		// Mix 4 bytes at a time into the hash
		auto data = (uint8_t const*)key;
		while (len >= 4)
		{
			auto k = *reinterpret_cast<uint32_t const*>(data);

			k *= m;
			k ^= k >> r;
			k *= m;

			h *= m;
			h ^= k;

			data += 4;
			len -= 4;
		}

		// Handle the last few bytes of the input array
		switch (len)
		{
		case 3: h ^= data[2] << 16; [[fallthrough]];
		case 2: h ^= data[1] << 8; [[fallthrough]];
		case 1: h ^= data[0]; h *= m; [[fallthrough]];
		default: break;
		}

		// Do a few final mixes of the hash to ensure the last few bytes are well-incorporated.
		h ^= h >> 13;
		h *= m;
		h ^= h >> 15;

		return h;
	}

	// 64-bit hash for 64-bit platforms
	// MurmurHash2, 64-bit versions, by Austin Appleby
	// The same caveats as 32-bit MurmurHash2 apply here - beware of alignment 
	// and endian-ness issues if used across multiple platforms.
	inline HashValue64 MurmurHash2_64(void const* key, int len, HashValue32 seed = -1)
	{
		uint64_t const m = 0xc6a4a7935bd1e995UL;
		int32_t const r = 47;

		uint64_t h = seed ^ (len * m);
		auto data = (uint64_t const*)key;
		auto end = data + (len/8);

		while (data != end)
		{
			auto k = *data++;

			k *= m;
			k ^= k >> r;
			k *= m;

			h ^= k;
			h *= m;
		}

		auto data2 = (uint8_t const*)data;
		switch (len & 7)
		{
		case 7: h ^= uint64_t(data2[6]) << 48; [[fallthrough]];
		case 6: h ^= uint64_t(data2[5]) << 40; [[fallthrough]];
		case 5: h ^= uint64_t(data2[4]) << 32; [[fallthrough]];
		case 4: h ^= uint64_t(data2[3]) << 24; [[fallthrough]];
		case 3: h ^= uint64_t(data2[2]) << 16; [[fallthrough]];
		case 2: h ^= uint64_t(data2[1]) << 8; [[fallthrough]];
		case 1: h ^= uint64_t(data2[0]); h *= m; [[fallthrough]];
		default: break;
		}
 
		h ^= h >> r;
		h *= m;
		h ^= h >> r;

		return h;
	}
}

#if 0
	// Hash a single 'ch'
	template <PodType32 Ty> inline HashValue32 Hash(Ty ch, uint32_t h = FNV_offset_basis32)
	{
		return HashCT32(ch, h);
	}
	template <CharType Ty> inline HashValue32 HashI(Ty ch, uint32_t h = FNV_offset_basis32)
	{
		return HashCT32(impl::Lower(ch), h);
	}
	// Range hash
	template <PodType32 Ty> constexpr uint32_t Hash32CT(Ty const* ptr, Ty const* end, uint32_t h = FNV_offset_basis32)
	{
		// Notes:
		//  - Can't use 'void const*' because 'reinterpret_cast' is not allowed in constexpr functions.

		uintptr_t const unaligned_mask = alignof(uint32_t) - 1;

		// Hash any unaligned bytes
		for (; (reinterpret_cast<uintptr_t>(bptr) & unaligned_mask) != 0; ++bptr)
			Hash32CT(*bptr, h);

		// Hash in units of uint32_t
		for (; bend - bptr > sizeof(uint32_t); bptr += sizeof(uint32_t))
			Hash32CT(*reinterpret_cast<uint32_t const*>(bptr), h);

		// Hash remaining bytes
		for (; bptr != bend; ++bptr)
			Hash32CT(*bptr, h);

		return h;
	}
	template <ByteType Ty> constexpr uint64_t Hash64CT(Ty const* bptr, Ty const* bend, uint64_t h = FNV_offset_basis64)
	{
		// Notes:
		//  - Can't use 'void const*' because 'reinterpret_cast' is not allowed in constexpr functions.
		uintptr_t const unaligned_mask = alignof(uint64_t) - 1;

		// Hash any unaligned bytes
		for (; (reinterpret_cast<uintptr_t>(bptr) & unaligned_mask) != 0; ++bptr)
			Hash64CT(*bptr, h);

		// Hash in units of uint64_t
		for (; bend - bptr > sizeof(uint64_t); bptr += sizeof(uint64_t))
			Hash64CT(*reinterpret_cast<uint64_t const*>(bptr), h);

		// Hash remaining bytes
		for (; bptr != bend; ++bptr)
			Hash64CT(*bptr, h);

		return h;
	}
	template <PodType Ty> constexpr HashValue32 HashCT(Ty ch, uint32_t h = FNV_offset_basis32)
	{
		// Types that can be converted to u32
		if constexpr (std::is_convertible_v<Ty, uint32_t>)
			return Hash32CT(static_cast<uint32_t>(ch), h);

		// Types with size/alignment that is a multiple of 4 bytes
		else if constexpr (std::is_standard_layout_v<Ty> && (sizeof(Ty) & 3) == 0 && alignof(Ty) >= 4)
			return Hash32CT(reinterpret_cast<uint32_t const*>(&ch), reinterpret_cast<uint32_t const*>(&ch) + sizeof(ch)/sizeof(uint32_t), h);
		
		// Types with size/alignment that is a multiple of 2 bytes
		else if constexpr (std::is_standard_layout_v<Ty> && (sizeof(Ty) & 1) == 0 && alignof(Ty) >= 2)
			return Hash32CT(reinterpret_cast<uint16_t const*>(&ch), reinterpret_cast<uint16_t const*>(&ch) + sizeof(ch)/sizeof(uint16_t), h);

		// Types with arbitrary size/alignment
		else if constexpr (std::is_standard_layout_v<Ty>)
			return Hash32CT(reinterpret_cast<uint8_t const*>(&ch), reinterpret_cast<uint8_t const*>(&ch) + sizeof(ch), h);

		// Not supported
		else
			static_assert(impl::dependent<Ty, false>, "Unsupported type for Hash()");
	}
	// Hash a range of elements
	template <typename Iter, typename Ty = std::iterator_traits<Iter>::value_type, typename = impl::enable_if_pod32<Ty>> inline HashValue32 Hash(Iter first, Iter last, uint32_t h = FNV_offset_basis32)
	{
		for (; first != last; ++first) h = HashCT(*first, h);
		return h;
	}
	template <typename Iter, typename Ty = std::iterator_traits<Iter>::value_type, typename = impl::enable_if_char<Ty>> inline HashValue32 HashI(Iter first, Iter last, uint32_t h = FNV_offset_basis32)
	{
		for (; first != last; ++first) h = HashCT(impl::Lower(*first), h);
		return h;
	}
#endif

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(HashTests)
	{
		using namespace pr::hash;

		{// basic hashing
			auto h0 = Hash("");
			PR_EXPECT(h0 == static_cast<HashValue32>(hash::FNV_offset_basis32));
		}
		{// Compile time hash vs. std::hash
			char const data[] = "Paul was here. CrC this, mofo";
			const auto h0  = sizeof(size_t) == 8 ? Hash64CT(data) : Hash32CT(data);
			auto h1 = std::hash<std::string>{}(&data[0]);
			PR_EXPECT(h0 == h1);
		}
		{// Compile time vs. run time
			char const data[] = "Paul was here. CrC this, mofo";
			const auto h0 = HashCT(&data[0]);
			const auto h1 = Hash(&data[0]);
			PR_EXPECT(h0 == h1);

			enum { h2 = HashCT("four") };
			const auto h3 = Hash("four");
			PR_EXPECT(h2 == h3);

			char const* five = "five";
			enum { h4 = HashCT("five") };
			const auto h5 = Hash(five);
			PR_EXPECT(h4 == h5);
		}
		{// Hash Bytes
			uint8_t const alignas(8) data[] = { 0,1,2,3,4,5,6,7, 8,9,0,1,2,3,4,5, 6,7,8,9,0,1,2,3, 4,5,6,7,8,9,0,1,2 };
			auto h0 = HashBytes64(&data[2], &data[4]); // within alignment boundary
			PR_EXPECT(h0 == 0x08395507b4f137f2LL);
			auto h1 = HashBytes64(&data[5], &data[9]); // spanning alignment boundary + < sizeof(u64)
			PR_EXPECT(h1 == 0x614326560c39f905LL);
			auto h2 = HashBytes64(&data[5], &data[18]); // spanning alignment boundary + > sizeof(u64)
			PR_EXPECT(h2 == 0xd2c939dd1c78c0f4LL);
			auto h3 = HashBytes64(&data[8], &data[16]); // on alignment boundaries
			PR_EXPECT(h3 == 0x5b3ae47deb0dc61dLL);
			auto h4 = HashBytes64(&data[15], &data[19]); // locality check
			PR_EXPECT(h4 == h1);
			auto h5 = HashBytes64(&data[15], &data[28]); // locality check
			PR_EXPECT(h5 == h2);
		}
		{// Hash POD
			struct POD { int i; char c[4]; float f; };
			static_assert(std::is_standard_layout_v<POD>);

			auto pod0 = POD{32,{ 'A','B','C','D' },6.28f};
			auto pod1 = POD{31,{ 'D','C','B','A' },3.14f};
			auto pod2 = POD{32,{ 'A','B','C','D' },6.28f};
			const auto h0 = HashArgs(pod0);
			const auto h1 = HashArgs(pod1);
			const auto h2 = HashArgs(pod2);
			PR_EXPECT(h0 != h1);
			PR_EXPECT(h0 == h2);
		}
		{// Case insensitive hash
			enum E { Blah = HashICT("Blah"), };
			const auto h0 = HashI("Blah");
			PR_EXPECT(E(h0) == Blah);
		}
		{ // HsiehHash16
			char const data[] = "Hsieh hash test!";
			auto h0 = HsiehHash16(&data[0], sizeof(data));
			PR_EXPECT(h0 ==int(0xe85f5a90));
		}
		{ // MurmurHash
			char const data[] = "Murmur hash test";
			auto h0 = MurmurHash2_32(&data[0], sizeof(data));
			auto h1 = MurmurHash2_64(&data[0], sizeof(data));

			PR_EXPECT(h0 == 0x6bfb39d7);
			PR_EXPECT(h1 == 0x52ce8bc5882d9212LL);
		}
		{ // Hash arguments
			char const* s = "was";
			struct POD { int i; char c; short s[3]; } pod = {};
			memset(&pod, 0, sizeof(pod)); // Remember padding bytes are not initialised;
			pod.i = 32;
			pod.c = 'X';
			pod.s[0] = 1;
			pod.s[1] = 2;
			pod.s[2] = 3;
			const auto h0 = HashArgs("Paul", s, L"here", 1976, 12.29, 1234U, pod);
			PR_EXPECT(h0 == static_cast<HashValue32>(0xe94b6ef9));
		}
	}
}
#endif