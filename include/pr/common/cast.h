//******************************************
// uint8_t Ptr Cast
//  Copyright (c) March 2008 Paul Ryland
//******************************************
// Use to cast any pointer to a uint8_t pointer
#pragma once
#include <cstddef>
#include <cassert>
#include <stdexcept>
#include <type_traits>
#include <span>

namespace pr
{
	// Debug helper for displaying types in error messages
	template<typename T> struct show_type;

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
		else
		{
			assert("Cast loses data" && static_cast<U>(static_cast<T>(x)) == x);
		}
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
	template <std::floating_point T, std::integral U> constexpr T s_cast(U x)
	{
		return static_cast<T>(x);
	}
	template <std::integral T, std::floating_point U> constexpr T s_cast(U x)
	{
		assert(x == x && "Can't convert NaN to an integral type");
		assert(std::abs(x) != std::numeric_limits<U>::infinity() && "Can't convert '+/-inf' to an integral type");
		return static_cast<T>(x);
	}
	template <std::floating_point T, bool RuntimeCheck = false, std::floating_point U> constexpr T s_cast(U x)
	{
		if constexpr (RuntimeCheck)
		{
			if (x < std::numeric_limits<T>::lowest() || x > std::numeric_limits<T>::max())
				throw std::runtime_error("Cast loses data");
		}
		else
		{
			assert("Cast loses data" && x >= std::numeric_limits<T>::lowest() && x <= std::numeric_limits<T>::max());
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

