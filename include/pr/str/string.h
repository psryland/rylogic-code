//******************************************
// pr::string<>
//  Copyright (c) Rylogic Ltd 2008
//******************************************
#pragma once
#include <string_view> 
#include <format> 
#include "pr/container/vector.h" 
#include "pr/str/string_core.h" 

namespace pr
{
	template <typename Type = char, int LocalCount = 244, bool Fixed = false, typename Allocator = aligned_alloc<Type>>
	struct string : vector<Type, LocalCount, Fixed, alignof(Type), Allocator>
	{
		// Note: 
		//  - no "c_str()". That requires null termination which is not guaranteed by this class. 
		using base = vector<Type, LocalCount, Fixed, alignof(Type), Allocator>;

		// Bring in the constructors 
		using base::base;

		// Construct from std::basic_string_view 
		string(std::basic_string_view<Type> const& str)
		{
			base::append_range(str);
		}

		// Construct from char const* 
		string(Type const* str)
			:string(std::basic_string_view<Type>(str))
		{
		}

		// Assignment from std::basic_string_view 
		string& operator=(std::basic_string_view<Type> const& str)
		{
			base::assign(str.begin(), str.end());
			return *this;
		}

		// Support c_str()
		Type const* c_str() const
		{
			// This is a bit hacky, but it allows us to support c_str() without the overhead of always maintaining a null terminator.
			const_cast<string*>(this)->reserve(base::size() + 1);
			const_cast<string*>(this)->data()[base::size()] = 0;
			return base::data();
		}

		// Assign to the string
		string& assign(std::basic_string_view<Type> const& str)
		{ 
			base::assign(str.begin(), str.end());
			return *this; 
		}

		// Append to the string 
		string& append(std::basic_string_view<Type> const& str)
		{ 
			base::append_range(str);
			return *this; 
		}

		// Insert into the string
		string& insert(size_t pos, std::basic_string_view<Type> const& str)
		{
			base::insert(this->begin() + pos, str.begin(), str.end());
			return *this;
		}
 
		// Allow implicit cast to std::basic_string_view 
		operator std::basic_string_view<Type>() const noexcept 
		{ 
			return std::basic_string_view<Type>(this->data(), this->size()); 
		} 
 
		// Boolean comparison operators 
		friend bool operator==(string const& lhs, string const& rhs) noexcept 
		{ 
			return std::basic_string_view<Type>(lhs) == std::basic_string_view<Type>(rhs); 
		} 
		friend bool operator!=(string const& lhs, string const& rhs) noexcept 
		{ 
			return std::basic_string_view<Type>(lhs) != std::basic_string_view<Type>(rhs); 
		} 
		friend bool operator==(string const& lhs, std::basic_string_view<Type> const& rhs) noexcept 
		{ 
			return std::basic_string_view<Type>(lhs) == rhs; 
		} 
		friend bool operator!=(string const& lhs, std::basic_string_view<Type> const& rhs) noexcept 
		{ 
			return std::basic_string_view<Type>(lhs) != rhs; 
		} 
		friend bool operator==(std::basic_string_view<Type> const& lhs, string const& rhs) noexcept 
		{ 
			return lhs == std::basic_string_view<Type>(rhs); 
		} 
		friend bool operator!=(std::basic_string_view<Type> const& lhs, string const& rhs) noexcept 
		{ 
			return lhs != std::basic_string_view<Type>(rhs); 
		} 
		friend bool operator==(string const& lhs, Type const* rhs) noexcept 
		{ 
			return std::basic_string_view<Type>(lhs) == rhs; 
		} 
		friend bool operator!=(string const& lhs, Type const* rhs) noexcept 
		{ 
			return std::basic_string_view<Type>(lhs) != rhs; 
		} 
		friend bool operator==(Type const* lhs, string const& rhs) noexcept 
		{ 
			return lhs == std::basic_string_view<Type>(rhs); 
		} 
		friend bool operator!=(Type const* lhs, string const& rhs) noexcept 
		{ 
			return lhs != std::basic_string_view<Type>(rhs); 
		} 
	};

	// 'string_traits' specialisation
	template <typename T, int L, bool F, typename A>
	struct string_traits<string<T, L, F, A>> :char_traits<T> 
	{ 
		using value_type = T; 
		using string_type = string<T, L, F, A>; 
		static bool const null_terminated = false; 
		static bool const dynamic_size = true; 
 
		static value_type const* ptr(string_type const& str) { return str.data(); } 
		static value_type* ptr(string_type& str) { return str.data(); } 
		static size_t size(string_type const& str) { return str.size(); } 
		static bool empty(string_type const& str) { return str.empty(); } 
		static void resize(string_type& str, size_t n) { str.resize(n); } 
	};
	template <typename T, int L, bool F, typename A>
	struct string_traits<string<T, L, F, A> const> :char_traits<T> 
	{ 
		using value_type = T const; 
		using string_type = string<T, L, F, A> const; 
		static bool const null_terminated = false; 
		static bool const dynamic_size = true; 
 
		static value_type* ptr(string_type& str) { return str.data(); } 
		static size_t size(string_type const& str) { return str.size(); } 
		static bool empty(string_type const& str) { return str.empty(); } 
		static void resize(string_type& str, size_t n) { static_assert(dependent_false<T>, "Immutable string cannot be resized"); } 
	};
	static_assert(std::is_same_v<string_traits<string<char>>::value_type, char>);
	static_assert(std::is_same_v<string_traits<string<wchar_t>>::value_type, wchar_t>);

	// 'is_string' trait
	template <typename T, int L, bool F, typename A>
	struct is_string<string<T, L, F, A>> :is_char<T> {};
	static_assert(is_string_v<string<char>>);
}
namespace std
{
	// std::hash specialisation for pr::string
	template <typename T, int L, bool F, typename A>
	struct hash<pr::string<T, L, F, A>>
	{
		size_t operator()(pr::string<T, L, F, A> const& key) const noexcept
		{
			return hash<std::basic_string_view<T>>{}(std::basic_string_view<T>(key.data(), key.size()));
		}
	};

	// std::format support
	template <typename T, int L, bool F, typename A>
	struct formatter<pr::string<T,L,F,A>> : formatter<std::basic_string_view<T>>
	{
		auto format(pr::string<T,L,F,A> const& v, auto& ctx) const
		{
			if constexpr (std::is_same_v<T, char>)
				return formatter<std::string_view>::format(std::string_view(v.data(), v.size()), ctx);
			else
				return formatter<std::basic_string_view<T>>::format(std::basic_string_view<T>(v.data(), v.size()), ctx);
		}
	};
}

#if PR_UNITTESTS
#include <string>
#include "pr/common/unittests.h"
namespace pr::str::tests
{
	PRUnitTestClass(PrStringTests)
	{
		PRUnitTestMethod(ConstructorsAndAssignment)
		{
			// TODO: Test the basic pr::string functionality
		}
	};
}
#endif
