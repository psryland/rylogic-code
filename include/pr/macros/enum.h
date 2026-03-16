//******************************************************************************
// Reflected enum
//  Copyright (c) Rylogic Ltd 2013
//******************************************************************************
// Use:
///*
//  enum class TestEnum1
//  {
//     #define PR_ENUM(x)\
//     x(A, "a", = 0)/* comment */\
//     x(B, "b", = 1)/* comment */\
//     x(C, "c", = 2)/* comment */
//     PR_ENUM_MEMBERS3(PR_ENUM)
//  };
//  PR_ENUM_REFLECTION3(TestEnum1, PR_ENUM);
//  #undef PR_ENUM
//*/
#pragma once
#include <exception>
#include <string>
#include <string_view>
#include <type_traits>
#include <optional>
#include <iostream>
#include <sstream>

// Traits
namespace pr
{
	// Enum metadata type access - generic case
	inline void pr_reflected_enum_metadata(...) {}

	// True if 'T' is a reflected enum
	template <typename T>
	constexpr bool is_reflected_enum_v = !std::is_same_v<void, std::decay_t<decltype(pr_reflected_enum_metadata(std::declval<T>()))>>;

	// Reflected Enum concept
	template <typename T>
	concept ReflectedEnum = std::is_enum_v<T> && is_reflected_enum_v<T>;

	// using Enum<EMyEnum> = EMyEnum_;
	template <ReflectedEnum T>
	using Enum = std::decay_t<decltype(pr_reflected_enum_metadata(std::declval<T>()))>;
}

// Enum generator macros
#pragma region Reflected Enum Generator

#pragma region Enum Field Expansions
#define PR_ENUM_NULL(x)
#define PR_ENUM_EXPAND(x) x

#define PR_ENUM_DEFINE1(id)           id,
#define PR_ENUM_DEFINE2(id, val)      id val,
#define PR_ENUM_DEFINE3(id, str, val) id val,

#define PR_ENUM_COUNT1(id)            + 1
#define PR_ENUM_COUNT2(id, val)       + 1
#define PR_ENUM_COUNT3(id, str, val)  + 1

#define PR_ENUM_TOSTRING1(id)           case E::id: return #id;
#define PR_ENUM_TOSTRING2(id, val)      case E::id: return #id;
#define PR_ENUM_TOSTRING3(id, str, val) case E::id: return str;
#define PR_ENUM_TOWSTRING1(id)           case E::id: return L""#id;
#define PR_ENUM_TOWSTRING2(id, val)      case E::id: return L""#id;
#define PR_ENUM_TOWSTRING3(id, str, val) case E::id: return L##str;

#define PR_ENUM_STRCMP1(id)            if (name == #id)                                    { e = E::id; return true; }
#define PR_ENUM_STRCMP2(id, val)       if (name == #id)                                    { e = E::id; return true; }
#define PR_ENUM_STRCMP3(id, str, val)  if (name == #id || name == str)                     { e = E::id; return true; }
#define PR_ENUM_STRCMPI1(id)           if (ieql<char>(name, #id))                          { e = E::id; return true; }
#define PR_ENUM_STRCMPI2(id, val)      if (ieql<char>(name, #id))                          { e = E::id; return true; }
#define PR_ENUM_STRCMPI3(id, str, val) if (ieql<char>(name, #id) || ieql<char>(name, str)) { e = E::id; return true; }

#define PR_ENUM_WSTRCMP1(id)            if (name == L""#id)                                             { e = E::id; return true; }
#define PR_ENUM_WSTRCMP2(id, val)       if (name == L""#id)                                             { e = E::id; return true; }
#define PR_ENUM_WSTRCMP3(id, str, val)  if (name == L""#id || name == L##str)                           { e = E::id; return true; }
#define PR_ENUM_WSTRCMPI1(id)           if (ieql<wchar_t>(name, L""#id))                                { e = E::id; return true; }
#define PR_ENUM_WSTRCMPI2(id, val)      if (ieql<wchar_t>(name, L""#id))                                { e = E::id; return true; }
#define PR_ENUM_WSTRCMPI3(id, str, val) if (ieql<wchar_t>(name, L""#id) || ieql<wchar_t>(name, L##str)) { e = E::id; return true; }

#define PR_ENUM_TOTRUE1(id)           case E::id: return true;
#define PR_ENUM_TOTRUE2(id, val)      case E::id: return true;
#define PR_ENUM_TOTRUE3(id, str, val) case E::id: return true;

#define PR_ENUM_FIELDS1(id)           E::id,
#define PR_ENUM_FIELDS2(id, val)      E::id,
#define PR_ENUM_FIELDS3(id, str, val) E::id,
#pragma endregion

// Expand the enum fields within an 'enum class EEnum { <here> }'
#define PR_ENUM_MEMBERS_IMPL(field_names1, field_names2, field_names3)\
	field_names1(PR_ENUM_DEFINE1)\
	field_names2(PR_ENUM_DEFINE2)\
	field_names3(PR_ENUM_DEFINE3)

// Generate an enum reflection class
#define PR_ENUM_REFLECTION_IMPL(enum_name, enum_vals1, enum_vals2, enum_vals3)\
	struct enum_name##_\
	{\
		using reflected_enum_t = enum_name;\
		using underlying_type_t = std::underlying_type_t<enum_name>;\
		\
		/* The name of the enum as a literal string */\
		template <typename Char> static constexpr Char const* Name()\
		{\
			if constexpr (std::is_same_v<Char, char   >) return #enum_name;\
			if constexpr (std::is_same_v<Char, wchar_t>) return L""#enum_name;\
		}\
		static constexpr char const*    NameA() { return Name<char>(); }\
		static constexpr wchar_t const* NameW() { return Name<wchar_t>(); }\
		\
		/* The number of values in the enum */\
		static constexpr int NumberOf = 0\
			enum_vals1(PR_ENUM_COUNT1)\
			enum_vals2(PR_ENUM_COUNT2)\
			enum_vals3(PR_ENUM_COUNT3);\
		\
		/* Return an enum member as a string */\
		template <typename Char> static constexpr Char const* ToString(enum_name e)\
		{\
			using E = enum_name;\
			if constexpr (std::is_same_v<Char, char>)\
			{\
				switch (e) {\
				enum_vals1(PR_ENUM_TOSTRING1)\
				enum_vals2(PR_ENUM_TOSTRING2)\
				enum_vals3(PR_ENUM_TOSTRING3)\
				default: return "";\
				}\
			}\
			if constexpr (std::is_same_v<Char,wchar_t>)\
			{\
				switch (e) {\
				enum_vals1(PR_ENUM_TOWSTRING1)\
				enum_vals2(PR_ENUM_TOWSTRING2)\
				enum_vals3(PR_ENUM_TOWSTRING3)\
				default: return L"";\
				}\
			}\
		}\
		static constexpr char const*    ToStringA(enum_name e) { return ToString<char>(e); }\
		static constexpr wchar_t const* ToStringW(enum_name e) { return ToString<wchar_t>(e); }\
		static constexpr char const*    ToStringA(underlying_type_t e) { return ToString<char>(static_cast<enum_name>(e)); }\
		static constexpr wchar_t const* ToStringW(underlying_type_t e) { return ToString<wchar_t>(static_cast<enum_name>(e)); }\
		\
		/* Try to convert a string name into it's enum value (inverse of ToString)*/ \
		template <typename Char> static bool TryParse(enum_name& e, std::basic_string_view<Char> name, bool match_case = true)\
		{\
			using E = enum_name;\
			if (name.empty()) return false;\
			if constexpr (std::is_same_v<Char,char>)\
			{\
				if (match_case)\
				{\
					enum_vals1(PR_ENUM_STRCMP1)\
					enum_vals2(PR_ENUM_STRCMP2)\
					enum_vals3(PR_ENUM_STRCMP3)\
				}\
				else\
				{\
					enum_vals1(PR_ENUM_STRCMPI1)\
					enum_vals2(PR_ENUM_STRCMPI2)\
					enum_vals3(PR_ENUM_STRCMPI3)\
				}\
				return false;\
			}\
			if constexpr (std::is_same_v<Char,wchar_t>)\
			{\
				if (match_case)\
				{\
					enum_vals1(PR_ENUM_WSTRCMP1)\
					enum_vals2(PR_ENUM_WSTRCMP2)\
					enum_vals3(PR_ENUM_WSTRCMP3)\
				}\
				else\
				{\
					enum_vals1(PR_ENUM_WSTRCMPI1)\
					enum_vals2(PR_ENUM_WSTRCMPI2)\
					enum_vals3(PR_ENUM_WSTRCMPI3)\
				}\
				return false;\
			}\
		}\
		static std::optional<enum_name> TryParse(std::string_view name, bool match_case = true)\
		{\
			enum_name e;\
			return TryParse<char>(e, name, match_case) ? std::make_optional(e) : std::nullopt;\
		}\
		static std::optional<enum_name> TryParse(std::wstring_view name, bool match_case = true)\
		{\
			enum_name e;\
			return TryParse<wchar_t>(e, name, match_case) ? std::make_optional(e) : std::nullopt;\
		}\
		\
		/* Convert a string into it's enum value*/ \
		template <typename Char> static enum_name Parse(std::basic_string_view<Char> name, bool match_case = true)\
		{\
			enum_name r;\
			if (TryParse(r, name, match_case)) return r;\
			throw std::runtime_error("Parse failed, no matching value in enum "#enum_name);\
		}\
		static enum_name Parse(std::string_view name, bool match_case = true)\
		{\
			return Parse<char>(name, match_case);\
		}\
		static enum_name Parse(std::wstring_view name, bool match_case = true)\
		{\
			return Parse<wchar_t>(name, match_case);\
		}\
		\
		/* Returns true if 'val' is convertible to one of the values in this enum */\
		static constexpr bool IsValue(underlying_type_t val)\
		{\
			using E = enum_name;\
			switch (static_cast<E>(val)) {\
			enum_vals1(PR_ENUM_TOTRUE1)\
			enum_vals2(PR_ENUM_TOTRUE2)\
			enum_vals3(PR_ENUM_TOTRUE3)\
			default: return false;\
			}\
		}\
		\
		/* Convert an integral type to an enum value, throws if 'val' is not a valid value */\
		template <typename T> static constexpr enum_name From(T val)\
		{\
			auto ut_value = static_cast<underlying_type_t>(val);\
			if (!IsValue(ut_value)) throw std::runtime_error("value is not a valid member of enum "#enum_name);\
			return static_cast<enum_name>(val);\
		}\
		\
		/* Return an enum member by index. (const& so that address-of can be used) */\
		static enum_name const& Member(int index)\
		{\
			using E = enum_name;\
			static enum_name const map[] =\
			{\
				enum_vals1(PR_ENUM_FIELDS1)\
				enum_vals2(PR_ENUM_FIELDS2)\
				enum_vals3(PR_ENUM_FIELDS3)\
			};\
			if (index < 0 || index >= NumberOf) throw std::runtime_error("index out of range for enum "#enum_name);\
			return map[index];\
		}\
		\
		/* Return the name of an enum member by index */ \
		template <typename Char> static constexpr Char const* MemberName(int index)\
		{\
			return ToString<Char>(Member(index));\
		}\
		static constexpr char const*    MemberNameA(int index) { return MemberName<char   >(index); }\
		static constexpr wchar_t const* MemberNameW(int index) { return MemberName<wchar_t>(index); }\
		\
		/* Returns an iterator range for iterating over each element in the enum*/\
		static std::initializer_list<enum_name> Members()\
		{\
			auto b = &Member(0);\
			return std::initializer_list<enum_name>(b, b + NumberOf);\
		}\
		\
	private:\
		\
		/* Case-insensitive compare */\
		template <typename Char> static bool ieql(std::basic_string_view<Char> lhs, std::basic_string_view<Char> rhs)\
		{\
			if constexpr (std::is_same_v<Char, char>)\
				return lhs.size() == rhs.size() && _strnicmp(lhs.data(), rhs.data(), lhs.size()) == 0;\
			else if constexpr (std::is_same_v<Char, wchar_t>)\
				return lhs.size() == rhs.size() && _wcsnicmp(lhs.data(), rhs.data(), lhs.size()) == 0;\
			else\
				static_assert(!sizeof(Char*), "Case insensitive compare not defined for this string type");\
		}\
	};\
	\
	/* Stream the enum value to/from basic streams */ \
	template <typename Char> inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& stream, enum_name e)\
	{\
		return stream << enum_name##_::ToString<Char>(e);\
	}\
	template <typename Char> inline std::basic_istream<Char>& operator >> (std::basic_istream<Char>& stream, enum_name& e)\
	{\
		std::string s; stream >> s;\
		e = enum_name##_::Parse(s.c_str());\
		return stream;\
	}\
	inline constexpr char const* ToString(enum_name e) { return enum_name##_::ToString<char>(e); }\
	\
	/* Forward declare a function that returns the meta data type for 'enum_name'*/\
	enum_name##_& pr_reflected_enum_metadata(enum_name);\
	/* end of enum macro */

// Macro to expand the members
#define PR_ENUM_MEMBERS1(x) PR_ENUM_MEMBERS_IMPL(x, PR_ENUM_NULL, PR_ENUM_NULL)
#define PR_ENUM_MEMBERS2(x) PR_ENUM_MEMBERS_IMPL(PR_ENUM_NULL, x, PR_ENUM_NULL)
#define PR_ENUM_MEMBERS3(x) PR_ENUM_MEMBERS_IMPL(PR_ENUM_NULL, PR_ENUM_NULL, x)

// Macro to generate the reflection class
#define PR_ENUM_REFLECTION1(enum_name, x) PR_ENUM_REFLECTION_IMPL(enum_name, x, PR_ENUM_NULL, PR_ENUM_NULL)
#define PR_ENUM_REFLECTION2(enum_name, x) PR_ENUM_REFLECTION_IMPL(enum_name, PR_ENUM_NULL, x, PR_ENUM_NULL)
#define PR_ENUM_REFLECTION3(enum_name, x) PR_ENUM_REFLECTION_IMPL(enum_name, PR_ENUM_NULL, PR_ENUM_NULL, x)

#pragma endregion

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/flags_enum.h"
namespace pr::common
{
	namespace unittests::enum2
	{
		// Normal enum
		enum class TestEnum0
		{
			A,
			B,
			C,
		};

		enum class TestEnum1
		{
			#define PR_ENUM(x)\
			x(A)/* */\
			x(B)/* this is 'B' */\
			x(C)
			PR_ENUM_MEMBERS1(PR_ENUM)
		};
		PR_ENUM_REFLECTION1(TestEnum1, PR_ENUM);
		#undef PR_ENUM

		enum class TestEnum2
		{
			#define PR_ENUM(x)\
			x(A, = 42)\
			x(B, = 43) /* this is 'B' */ \
			x(C, = 44)
			PR_ENUM_MEMBERS2(PR_ENUM)
		};
		PR_ENUM_REFLECTION2(TestEnum2, PR_ENUM);
		#undef PR_ENUM

		enum class TestEnum3
		{
			#define PR_ENUM(x)\
			x(A, "a", = 0x0A)\
			x(B, "b", = 0x0B)\
			x(C, "c", = 0x0C)
			PR_ENUM_MEMBERS3(PR_ENUM)
		};
		PR_ENUM_REFLECTION3(TestEnum3, PR_ENUM);
		#undef PR_ENUM

		enum class TestEnum4
		{
			#define PR_ENUM(x)\
			x(A, = 1 << 0)\
			x(B, = 1 << 1)\
			x(C, = 1 << 2)\
			x(_flags_enum, = 0)
			PR_ENUM_MEMBERS2(PR_ENUM)
		};
		PR_ENUM_REFLECTION2(TestEnum4, PR_ENUM);
		#undef PR_ENUM

		enum class TestEnum5
		{
			#define PR_ENUM(x)\
			x(A, "a", = 1 << 0)\
			x(B, "b", = 1 << 1)\
			x(C, "c", = 1 << 2 | B)\
			x(_flags_enum,"", = 0)
			PR_ENUM_MEMBERS3(PR_ENUM)
		};
		PR_ENUM_REFLECTION3(TestEnum5, PR_ENUM);
		#undef PR_ENUM
	}
	PRUnitTest(Enum2Tests)
	{
		using namespace pr::unittests;
		using namespace unittests::enum2;
		using namespace std::literals;

		static_assert(!is_reflected_enum_v<TestEnum0>);
		static_assert(is_reflected_enum_v<TestEnum1>);
		static_assert(is_reflected_enum_v<TestEnum2>);
		static_assert(is_reflected_enum_v<TestEnum3>);
		static_assert(is_reflected_enum_v<TestEnum4>);
		static_assert(is_reflected_enum_v<TestEnum5>);

		PR_EXPECT(UTEqual(Enum<TestEnum1>::NameA(), "TestEnum1"));
		PR_EXPECT(UTEqual(Enum<TestEnum2>::NameA(), "TestEnum2"));
		PR_EXPECT(UTEqual(Enum<TestEnum3>::NameA(), "TestEnum3"));
		PR_EXPECT(UTEqual(Enum<TestEnum4>::NameA(), "TestEnum4"));
		PR_EXPECT(UTEqual(Enum<TestEnum5>::NameA(), "TestEnum5"));

		PR_EXPECT(Enum<TestEnum1>::NumberOf == 3);
		PR_EXPECT(Enum<TestEnum2>::NumberOf == 3);
		PR_EXPECT(Enum<TestEnum3>::NumberOf == 3);
		PR_EXPECT(Enum<TestEnum4>::NumberOf == 4);
		PR_EXPECT(Enum<TestEnum5>::NumberOf == 4);

		PR_EXPECT(UTEqual(Enum<TestEnum1>::ToStringA(TestEnum1::A), "A"));
		PR_EXPECT(UTEqual(Enum<TestEnum2>::ToStringA(TestEnum2::A), "A"));
		PR_EXPECT(UTEqual(Enum<TestEnum3>::ToStringA(TestEnum3::A), "a"));
		PR_EXPECT(UTEqual(Enum<TestEnum4>::ToStringA(TestEnum4::A), "A"));
		PR_EXPECT(UTEqual(Enum<TestEnum5>::ToStringA(TestEnum5::A), "a"));

		PR_EXPECT(Enum<TestEnum1>::Parse("A") == TestEnum1::A);
		PR_EXPECT(Enum<TestEnum2>::Parse("A") == TestEnum2::A);
		PR_EXPECT(Enum<TestEnum3>::Parse("a") == TestEnum3::A);
		PR_EXPECT(Enum<TestEnum4>::Parse("A") == TestEnum4::A);
		PR_EXPECT(Enum<TestEnum5>::Parse("a") == TestEnum5::A);

		PR_EXPECT(Enum<TestEnum1>::Parse("A"s) == TestEnum1::A);
		PR_EXPECT(Enum<TestEnum2>::Parse("A"s) == TestEnum2::A);
		PR_EXPECT(Enum<TestEnum3>::Parse("a"s) == TestEnum3::A);
		PR_EXPECT(Enum<TestEnum4>::Parse("A"s) == TestEnum4::A);
		PR_EXPECT(Enum<TestEnum5>::Parse("a"s) == TestEnum5::A);

		PR_EXPECT(Enum<TestEnum1>::Parse("A"sv) == TestEnum1::A);
		PR_EXPECT(Enum<TestEnum2>::Parse("A"sv) == TestEnum2::A);
		PR_EXPECT(Enum<TestEnum3>::Parse("a"sv) == TestEnum3::A);
		PR_EXPECT(Enum<TestEnum4>::Parse("A"sv) == TestEnum4::A);
		PR_EXPECT(Enum<TestEnum5>::Parse("a"sv) == TestEnum5::A);

		// Initialisation
		TestEnum1 a1 = TestEnum1::A;
		TestEnum2 a2 = TestEnum2::A;
		TestEnum3 a3 = TestEnum3::A;
		TestEnum4 a4 = TestEnum4::A;
		TestEnum5 a5 = TestEnum5::A;

		// Assignment
		TestEnum1 b1; b1 = TestEnum1::B;
		TestEnum2 b2; b2 = TestEnum2::B;
		TestEnum3 b3; b3 = TestEnum3::B;
		TestEnum4 b4; b4 = TestEnum4::B;
		TestEnum5 b5; b5 = TestEnum5::B;
		TestEnum4 b6; b6 = TestEnum4::B; b6 |= TestEnum4::C;
		TestEnum5 b7; b7 = TestEnum5::B; b7 |= TestEnum5::C;

		{
			std::stringstream s; s << a1 << a2 << a3 << a4 << a5;
			PR_EXPECT(s.str() == "AAaAa"); // Stream as a name
		}
		{
			std::stringstream s; s << TestEnum1::A << TestEnum2::A << TestEnum3::A << TestEnum4::A << TestEnum5::A;
			PR_EXPECT(s.str() == "AAaAa"); // Stream as a name
		}
		{
			std::stringstream s; s << TestEnum1::A;
			TestEnum1 out; s >> out;
			PR_EXPECT(out == TestEnum1::A);
		}
		{
			volatile int i = 4; // invalid conversion, 4 is not an enum value
			PR_THROWS(Enum<TestEnum3>::From(i), std::exception);
		}
		char const* names[] = {"A","B","C"};
		TestEnum1 values[] = {TestEnum1::A, TestEnum1::B, TestEnum1::C};
		for (int i = 0; i != Enum<TestEnum1>::NumberOf; ++i)
		{
			PR_EXPECT(Enum<TestEnum1>::MemberNameA(i) == names[i]); // Access names by index
			PR_EXPECT(Enum<TestEnum1>::Member(i) == values[i]);     // Access members by index
		}

		// Enumerate members
		{
			int idx = 0;
			for (auto e : Enum<TestEnum1>::Members())
			{
				PR_EXPECT(e == Enum<TestEnum1>::Member(idx));
				PR_EXPECT(Enum<TestEnum1>::ToStringA(e) == Enum<TestEnum1>::MemberNameA(idx));
				++idx;
			}
		}
	}
}
#endif