//******************************************
// uint8_t Ptr Cast
//  Copyright (c) March 2008 Paul Ryland
//******************************************
// Use to cast any pointer to a uint8_t pointer
#pragma once
#include <cstdint>
#include <cstddef>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <type_traits>
#include <span>

namespace pr
{
	// Debug helper for displaying types in error messages
	template<typename T> struct show_type;

	namespace impl
	{
		// Test whether a floating-point value truncates to a representable integer target
		// value without producing an out-of-range conversion.
		template <std::integral T, std::floating_point U>
		constexpr bool FloatingToIntegralInRange(U x)
		{
			if (!std::isfinite(x))
				return false;

			// Truncation toward zero means fractions below the lower bound can still land on a
			// representable integer, so the comparison is against the truncation endpoints
			// rather than the original floating value.
			auto const upper = static_cast<U>(std::numeric_limits<T>::max()) + U{1};
			if (x >= upper)
				return false;

			if constexpr (std::is_unsigned_v<T>)
			{
				// Unsigned casts accept the interval just below zero because it truncates to 0.
				return x > U{-1};
			}
			else
			{
				auto const lower = static_cast<U>(std::numeric_limits<T>::lowest());

				// For small integer types the exact lower boundary is representable as
				// `lowest - 1`. For large integer types that subtraction rounds back to `lowest`,
				// so keep the lower comparison inclusive to preserve the valid truncation domain.
				if (lower - U{1} == lower)
					return x >= lower;

				return x > (lower - U{1});
			}
		}
	}

	// Helper for non-const member function overloads. Use 'const_call(member_func());'
	#define const_call(fn) const_cast<std::remove_const_t<decltype(fn)>>(std::as_const(*this).fn)

	// Test alignment of 't'
	template <typename T, int A = alignof(T)> constexpr bool is_aligned(void const* p)
	{
		return (reinterpret_cast<std::uintptr_t>(p) & (A - 1)) == 0;
	}
	template <typename T> constexpr bool is_aligned(T const* t)
	{
		return is_aligned<T, alignof(T)>(t);
	}

	// Casting from any type of pointer to a uint8_t pointer
	// Use:
	//   int* int_ptr = ...
	//   uint8_t* u8_ptr = byte_ptr(int_ptr);
	template <typename T> constexpr std::byte const* byte_ptr(T const* t) { return reinterpret_cast<std::byte const*>(t); }
	template <typename T> constexpr std::byte*       byte_ptr(T*       t) { return reinterpret_cast<std::byte*      >(t); }
	template <typename T> constexpr char const*      char_ptr(T const* t) { return reinterpret_cast<char const*>(t); }
	template <typename T> constexpr char*            char_ptr(T*       t) { return reinterpret_cast<char*      >(t); }
	
	// Handle casting nullptr to bytes/chars
	constexpr std::byte const* byte_ptr(nullptr_t) { return static_cast<std::byte const*>(nullptr); }
	constexpr char const*      char_ptr(nullptr_t) { return static_cast<char const*>(nullptr); }

	// Cast from a void pointer to a pointer of type 'T' (checking alignment)
	template <typename T> constexpr T const* type_ptr(void const* t)
	{
		assert(is_aligned<T>(t) && "Point is not correctly aligned for type");
		return static_cast<T const*>(t);
	}
	template <typename T> constexpr T* type_ptr(void* t)
	{
		assert(((byte_ptr(t) - byte_ptr(nullptr)) % std::alignment_of<T>::value) == 0 && "Point is not correctly aligned for type");
		return static_cast<T*>(t);
	}

	// Static "scalar cast" with runtime overflow checking.
	// 'RuntimeCheck' means an exception is thrown on lost data, otherwise it's just an assert.
	// Use:
	//  int16_t s = -1302;
	//  auto b = s_cast<uint8_t>(s); <- gives an assert because -1302 cannot be stored in a uint8_t
	//  auto b = s_cast<uint8_t,true>(s); <- throws an exception because -1302 cannot be stored in a uint8_t
	template <std::integral T, bool RuntimeCheck = false, std::integral U> constexpr T s_cast(U x)
	{
		if constexpr (RuntimeCheck)
		{
			if (static_cast<U>(static_cast<T>(x)) != x)
				throw std::runtime_error("Cast loses data");
		}
		#ifndef NDEBUG
		{
			if (static_cast<U>(static_cast<T>(x)) != x)
				assert(false && "Cast loses data");
		}
		#endif
		return static_cast<T>(x);
	}
	template <typename T, bool RuntimeCheck = false, typename U> constexpr T s_cast(U x) requires std::is_enum_v<T> && std::is_enum_v<U>
	{
		using ut0 = std::underlying_type_t<T>;
		using ut1 = std::underlying_type_t<U>;
		return static_cast<T>(s_cast<ut0, RuntimeCheck, ut1>(static_cast<ut1>(x)));
	}
	template <std::integral T, bool RuntimeCheck = false, typename U> constexpr T s_cast(U x) requires std::is_enum_v<U>
	{
		using ut = std::underlying_type_t<U>;
		return s_cast<T, RuntimeCheck, ut>(static_cast<ut>(x));
	}
	template <typename T, bool RuntimeCheck = false, std::integral U> constexpr T s_cast(U x) requires std::is_enum_v<T>
	{
		using ut = std::underlying_type_t<T>;
		return static_cast<T>(s_cast<ut, RuntimeCheck, U>(x));
	}
	template <std::floating_point T, bool RuntimeCheck = false, std::floating_point U> constexpr T s_cast(U x)
	{
		if constexpr (RuntimeCheck)
		{
			if (x < std::numeric_limits<T>::lowest() || x > std::numeric_limits<T>::max())
				throw std::runtime_error("Cast loses data");
		}
		#ifndef NDEBUG
		{
			if (x < std::numeric_limits<T>::lowest() || x > std::numeric_limits<T>::max())
				assert(false && "Cast loses data");
		}
		#endif
		return static_cast<T>(x);
	}
	template <std::floating_point T, std::integral U> constexpr T s_cast(U x)
	{
		return static_cast<T>(x);
	}
	template <std::integral T, bool RuntimeCheck = false, std::floating_point U> constexpr T s_cast(U x)
	{
		auto const in_range = impl::FloatingToIntegralInRange<T>(x);

		// Debug builds still assert so invalid calls are noisy during development, but
		// release builds must fail before the narrowing cast so they never enter UB.
		if constexpr (RuntimeCheck)
		{
			if (!in_range)
				throw std::runtime_error("Cast loses data");
		}
		else if (!in_range)
		{
			#ifndef NDEBUG
			assert(false && "Cast loses data");
			#else
			throw std::runtime_error("Cast loses data");
			#endif
		}
		return static_cast<T>(x);
	}

	// Helper for getting the size of a container as an int
	template <typename T> requires (requires (T t) { t.size(); }) inline int isize(T const& cont)
	{
		return s_cast<int>(cont.size());
	}
	template <typename T, int N> inline int isize(T const (&)[N])
	{
		return N;
	}

	// Int sizeof
	template <typename T> inline int isizeof()
	{
		return s_cast<int>(sizeof(T));
	}
	template <typename T> inline int isizeof(T&)
	{
		return s_cast<int>(sizeof(T));
	}

	// Convert {void const* + size_t} to a span of std::byte
	constexpr std::span<std::byte const> byte_span(void const* data, size_t size)
	{
		return std::span<std::byte const>(byte_ptr(data), size);
	}
	constexpr std::span<std::byte> byte_span(void* data, size_t size)
	{
		return std::span<std::byte>(byte_ptr(data), size);
	}

	// Convert a span of 'T' to a span of bytes
	template <typename T> inline std::span<std::byte const> byte_span(std::span<T const> x)
	{
		return std::span<std::byte const>(reinterpret_cast<std::byte const*>(x.data()), x.size_bytes());
	}
	template <typename T> inline std::span<std::byte> byte_span(std::span<T> x)
	{
		return std::span<std::byte>(reinterpret_cast<std::byte const*>(x.data()), x.size_bytes());
	}

	// Convert a span of bytes into a span of 'T'
	template <typename T> inline std::span<T const> type_span(std::span<std::byte const> x)
	{
		assert(x.size_bytes() % sizeof(T) == 0 && "byte span is not a multiple of 'T'");
		assert(uintptr_t(x.data()) % alignof(T) == 0 && "byte span alignment is not valid for 'T'");
		return std::span<T const>(reinterpret_cast<T const*>(x.data()), x.size_bytes() / sizeof(T));
	}
	template <typename T> inline std::span<T> type_span(std::span<std::byte> x)
	{
		assert(x.size_bytes() % sizeof(T) == 0 && "byte span is not a multiple of 'T'");
		assert(uintptr_t(x.data()) % alignof(T) == 0 && "byte span alignment is not valid for 'T'");
		return std::span<T>(reinterpret_cast<T*>(x.data()), x.size_bytes() / sizeof(T));
	}

	// Convert from aligned storage to a type 'T'
	template <typename T> inline T& storage_cast(std::span<std::byte> storage)
	{
		assert(storage.size() >= sizeof(T) && "Storage is too small for type 'T'");
		assert(uintptr_t(storage.data()) % alignof(T) == 0 && "Storage alignment is not valid for type 'T'");
		return *reinterpret_cast<T*>(storage.data());
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(IntegralFromFloatingCastTests)
	{
		// Fractional values inside the truncation domain should survive unchanged apart
		// from the normal toward-zero integer conversion.
		PR_EXPECT((s_cast<int8_t>(127.5) == 127));
		PR_EXPECT((s_cast<int8_t>(-128.5) == -128));
		PR_EXPECT((s_cast<uint8_t>(-0.5) == 0));

		// Values exactly on the range edge and the first representable values beyond it
		// exercise the precision-aware comparisons at both ends of the domain.
		PR_EXPECT((s_cast<int8_t>(std::nextafter(127.0, std::numeric_limits<double>::infinity())) == 127));
		PR_EXPECT((s_cast<int8_t>(std::nextafter(-128.0, -std::numeric_limits<double>::infinity())) == -128));
		PR_EXPECT((s_cast<uint8_t>(std::nextafter(-1.0, std::numeric_limits<double>::infinity())) == 0));
		auto const int64_safe_hi = std::nextafter(static_cast<double>(std::numeric_limits<int64_t>::max()), -std::numeric_limits<double>::infinity());
		PR_EXPECT((s_cast<int64_t>(int64_safe_hi) == static_cast<int64_t>(int64_safe_hi)));
		auto const uint64_safe_hi = std::nextafter(static_cast<double>(std::numeric_limits<uint64_t>::max()), -std::numeric_limits<double>::infinity());
		PR_EXPECT((s_cast<uint64_t>(uint64_safe_hi) == static_cast<uint64_t>(uint64_safe_hi)));

		// The checked path must reject values that stay finite but truncate outside the
		// destination range.
		PR_THROWS((s_cast<int8_t, true>(128.0)), std::runtime_error);
		PR_THROWS((s_cast<int8_t, true>(std::nextafter(128.0, std::numeric_limits<double>::infinity()))), std::runtime_error);
		PR_THROWS((s_cast<int8_t, true>(-129.0)), std::runtime_error);
		PR_THROWS((s_cast<uint8_t, true>(-1.0)), std::runtime_error);
		PR_THROWS((s_cast<uint8_t, true>(std::nextafter(-1.0, -std::numeric_limits<double>::infinity()))), std::runtime_error);
		PR_THROWS((s_cast<int64_t, true>(std::nextafter(static_cast<double>(std::numeric_limits<int64_t>::max()), std::numeric_limits<double>::infinity()))), std::runtime_error);
		PR_THROWS((s_cast<uint64_t, true>(static_cast<double>(std::numeric_limits<uint64_t>::max()))), std::runtime_error);
		PR_THROWS((s_cast<uint64_t, true>(std::nextafter(static_cast<double>(std::numeric_limits<uint64_t>::max()), std::numeric_limits<double>::infinity()))), std::runtime_error);

		// NaN and infinities are rejected explicitly.
		PR_THROWS((s_cast<int, true>(std::numeric_limits<double>::quiet_NaN())), std::runtime_error);
		PR_THROWS((s_cast<int, true>(std::numeric_limits<double>::infinity())), std::runtime_error);
		PR_THROWS((s_cast<int, true>(-std::numeric_limits<double>::infinity())), std::runtime_error);

		#ifdef NDEBUG
		// Release builds must still fail explicitly when the default RuntimeCheck=false
		// path receives a value that would otherwise become UB at the narrowing cast.
		PR_THROWS((s_cast<int8_t>(128.0)), std::runtime_error);
		PR_THROWS((s_cast<uint8_t>(-1.0)), std::runtime_error);
		#endif
	}
}
#endif
