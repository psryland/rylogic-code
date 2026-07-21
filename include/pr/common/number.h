//*************************************************************
// Number
//  Copyright (c) Rylogic Ltd 2008
//*************************************************************
// A value type that is either a double, 'long long', or 'unsigned long long'
#pragma once
#include <charconv>
#include <cassert>
#include <exception>
#include <limits>
#include <algorithm>
#include <string_view>
#include <type_traits>

namespace pr
{
	struct Number
	{
		union {
		double             m_db;
		long long          m_ll;
		unsigned long long m_ul;
		};
		
		enum class EType { Unknown, FP, Int, UInt } m_type;

		Number()
			:m_db()
			,m_type(EType::Unknown)
		{}
		Number(double d)
			:m_db(d)
			,m_type(EType::FP)
		{}
		Number(long long l)
			:m_ll(l)
			,m_type(EType::Int)
		{}
		Number(unsigned long long u)
			:m_ul(u)
			,m_type(EType::UInt)
		{}
		explicit Number(std::string_view expr, int radix = 0)
		{
			auto [num, consumed] = From(expr, radix);
			*this = num;
		}
		explicit Number(std::wstring_view expr, int radix = 0)
		{
			auto [num, consumed] = From(expr, radix);
			*this = num;
		}

		Number& operator = (float              v) { m_db = v; m_type = EType::FP;   return *this; }
		Number& operator = (double             v) { m_db = v; m_type = EType::FP;   return *this; }
		Number& operator = (unsigned long long v) { m_ul = v; m_type = EType::UInt; return *this; }
		Number& operator = (unsigned long      v) { m_ul = v; m_type = EType::UInt; return *this; }
		Number& operator = (long long          v) { m_ll = v; m_type = EType::Int;  return *this; }
		Number& operator = (long               v) { m_ll = v; m_type = EType::Int;  return *this; }
		Number& operator = (bool               v) { m_ll = v; m_type = EType::Int;  return *this; }

		// Access the value as a double
		double db() const
		{
			assert(m_type != EType::Unknown);
			return
				m_type == EType::FP ? static_cast<double>(m_db) :
				m_type == EType::Int ? static_cast<double>(m_ll) :
				m_type == EType::UInt ? static_cast<double>(m_ul) :
				0.0;
		}

		// Access the value as a signed integral type
		long long ll() const
		{
			assert(m_type != EType::Unknown);
			return
				m_type == EType::FP ? static_cast<long long>(m_db) :
				m_type == EType::Int ? static_cast<long long>(m_ll) :
				m_type == EType::UInt ? static_cast<long long>(m_ul) :
				0LL;
		}

		// Access the value as an unsigned integral type
		unsigned long long ul() const
		{
			assert(m_type != EType::Unknown);
			return
				m_type == EType::FP ? static_cast<unsigned long long>(m_db) :
				m_type == EType::Int ? static_cast<unsigned long long>(m_ll) :
				m_type == EType::UInt ? static_cast<unsigned long long>(m_ul) :
				0ULL;
		}

		// Read a value (greedily) from 'expr'. Returns the number and the number of characters consumed.
		template <typename Char>
		static std::tuple<Number, size_t> From(std::basic_string_view<Char> expr, int radix = 0)
		{
			Number num;

			// Narrow to char for std::from_chars (number literals are always ASCII)
			std::string_view sv;
			char narrow_buf[128];
			{
				if constexpr (std::is_same_v<Char, char>)
				{
					sv = { expr.data(), expr.size() };
				}
				else
				{
					auto n = (std::min)(expr.size(), sizeof(narrow_buf));
					for (size_t i = 0; i != n; ++i)
						narrow_buf[i] = static_cast<char>(expr[i]);

					sv = { narrow_buf, n };
				}
			}

			auto const beg = sv.data();
			auto const end = beg + sv.size();
			if (beg == end)
				return { num, 0 };

			// Detect sign. from_chars handles '-' for floats but not '+' for either type.
			bool is_neg = false;
			auto after_sign = beg;
			if (*after_sign == '+')
			{
				++after_sign;
			}
			else if (*after_sign == '-')
			{
				is_neg = true;
				++after_sign;
			}

			// Detect 0x/0b prefix (after sign)
			auto after_prefix = after_sign;
			bool has_hex_prefix = (after_prefix + 1 < end && after_prefix[0] == '0' && (after_prefix[1] == 'x' || after_prefix[1] == 'X'));
			bool has_bin_prefix = (after_prefix + 1 < end && after_prefix[0] == '0' && (after_prefix[1] == 'b' || after_prefix[1] == 'B'));
			if (has_hex_prefix || has_bin_prefix)
				after_prefix += 2;

			// Try parse as floating point. from_chars handles '-' natively but not '+', so skip '+' only
			double db_val = 0;
			auto fp_end = beg;

			if (has_hex_prefix)
			{
				// Hex float: from_chars(hex) doesn't expect 0x prefix
				auto r = std::from_chars(after_prefix, end, db_val, std::chars_format::hex);
				if (r.ec == std::errc{})
				{
					if (is_neg) db_val = -db_val;
					fp_end = r.ptr;
				}
			}
			else
			{
				auto fp_start = (*beg == '+') ? after_sign : beg;
				auto r = std::from_chars(fp_start, end, db_val, std::chars_format::general);
				if (r.ec == std::errc{})
					fp_end = r.ptr;
			}

			// Try parse as integer
			auto int_radix = radix;
			auto digit_start = after_sign;

			if (int_radix == 0)
			{
				if (has_hex_prefix)      { int_radix = 16; digit_start = after_prefix; }
				else if (has_bin_prefix) { int_radix = 2;  digit_start = after_prefix; }
				else if (after_sign != end && *after_sign == '0') int_radix = 8;
				else                     int_radix = 10;
			}
			else
			{
				// With explicit radix, skip matching prefix if present
				if (int_radix == 16 && has_hex_prefix) digit_start = after_prefix;
				else if (int_radix == 2 && has_bin_prefix) digit_start = after_prefix;
			}

			// Parse digits as unsigned, then apply sign manually.
			// from_chars can't handle sign + prefix being non-contiguous (e.g. "-0xFF" → "-" then "FF").
			unsigned long long raw = 0;
			auto int_result = std::from_chars(digit_start, end, raw, int_radix);
			auto int_end = (int_result.ec == std::errc{}) ? int_result.ptr : beg;

			// If no characters contributed to either parse, not a number
			if (fp_end <= beg && int_end <= beg)
				return { num, 0 };

			// Determine the type based on which parse consumed more characters
			if (fp_end > int_end)
			{
				num.m_db = db_val;
				num.m_type = EType::FP;
			}
			else if (is_neg)
			{
				num.m_ll = -static_cast<long long>(raw);
				num.m_type = EType::Int;
			}
			else if (raw <= static_cast<unsigned long long>((std::numeric_limits<long long>::max)()))
			{
				num.m_ll = static_cast<long long>(raw);
				num.m_type = EType::Int;
			}
			else
			{
				num.m_ul = raw;
				num.m_type = EType::UInt;
			}
			return { num, static_cast<size_t>(std::max(fp_end, int_end) - beg) };
		}

		// Comparison operators. Compare as floating point if either is floating point
		friend bool operator == (Number const& lhs, Number const& rhs)
		{
			return (lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? lhs.db() == rhs.db() : lhs.ll() == rhs.ll();
		}
		friend bool operator != (Number const& lhs, Number const& rhs)
		{
			return !(lhs == rhs);
		}
		friend bool operator < (Number const& lhs, Number const& rhs)
		{
			return (lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? lhs.db() <  rhs.db() : lhs.ll() <  rhs.ll();
		}
		friend bool operator > (Number const& lhs, Number const& rhs)
		{
			return (lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? lhs.db() >  rhs.db() : lhs.ll() >  rhs.ll();
		}
		friend bool operator <= (Number const& lhs, Number const& rhs)
		{
			return (lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? lhs.db() <= rhs.db() : lhs.ll() <= rhs.ll();
		}
		friend bool operator >= (Number const& lhs, Number const& rhs)
		{
			return (lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? lhs.db() >= rhs.db() : lhs.ll() >= rhs.ll();
		}

		// Binary operators
		friend Number operator + (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() + rhs.db()) :
				(lhs.m_type == EType::UInt || rhs.m_type == EType::UInt) ? Number(lhs.ul() + rhs.ul()) :
				Number(lhs.ll() + rhs.ll());
		}
		friend Number operator - (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() - rhs.db()) :
				(lhs.m_type == EType::UInt || rhs.m_type == EType::UInt) ? Number(lhs.ul() - rhs.ul()) :
				Number(lhs.ll() - rhs.ll());
		}
		friend Number operator * (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() * rhs.db()) :
				(lhs.m_type == EType::UInt || rhs.m_type == EType::UInt) ? Number(lhs.ul() * rhs.ul()) :
				Number(lhs.ll() * rhs.ll());
		}
		friend Number operator / (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() / rhs.db()) :
				(lhs.m_type == EType::UInt || rhs.m_type == EType::UInt) ? Number(lhs.ul() / rhs.ul()) :
				Number(lhs.ll() / rhs.ll());
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/math/math.h"
namespace pr::common
{
	PRUnitTest(NumberTests)
	{
		// Keep values above LLONG_MAX in the unsigned domain so arithmetic preserves the intended bit pattern.
		auto const big = Number{ 1ULL << 63 };
		PR_EXPECT(big.m_type == Number::EType::UInt);
		PR_EXPECT(big.ul() == (1ULL << 63));

		auto n0 = Number{1.3};
		PR_EXPECT(n0.db() == 1.3);
		PR_EXPECT(n0.ll() == 1LL);
		PR_EXPECT(n0.ul() == 1ULL);

		auto n1 = Number{1ULL};
		PR_EXPECT(n1.db() == 1.0);
		PR_EXPECT(n1.ll() == 1LL);
		PR_EXPECT(n1.ul() == 1ULL);

		auto n2 = Number("+0.1");
		PR_EXPECT(n2.db() == 0.1);
		PR_EXPECT(n2.ll() == 0);
		PR_EXPECT(n2.ul() == 0);

		auto n3 = Number("1ULL");
		PR_EXPECT(n3.db() == 1.0);
		PR_EXPECT(n3.ll() == 1LL);
		PR_EXPECT(n3.ul() == 1ULL);

		auto n4 = Number("-1.234e-13f");
		PR_EXPECT(FEql(n4.db(), -1.234e-13));
		PR_EXPECT(n4.ll() == 0);
		PR_EXPECT(n4.ul() == 0);

		auto n5 = Number("0xDeaDBeeF");
		PR_EXPECT(FEql(n5.db(), static_cast<double>(0xDeaDBeeF)));
		PR_EXPECT(n5.ll() == 0xDeaDBeeFLL);
		PR_EXPECT(n5.ul() == 0xDeaDBeeFULL);

		auto n6 = Number("10110101", 2);
		PR_EXPECT(FEql(n6.db(), static_cast<double>(0b10110101)));
		PR_EXPECT(n6.ll() == 0b10110101LL);
		PR_EXPECT(n6.ul() == 0b10110101ULL);

		// Exercise each integer operator with a UInt above LLONG_MAX so the unsigned branch is the one that matters.
		auto add = big + Number{ 1ULL };
		PR_EXPECT(add.m_type == Number::EType::UInt);
		PR_EXPECT(add.ul() == ((1ULL << 63) + 1ULL));

		auto sub = big - Number{ 1ULL };
		PR_EXPECT(sub.m_type == Number::EType::UInt);
		PR_EXPECT(sub.ul() == ((1ULL << 63) - 1ULL));

		auto mul = big * Number{ 2ULL };
		PR_EXPECT(mul.m_type == Number::EType::UInt);
		PR_EXPECT(mul.ul() == 0ULL);

		auto div = big / Number{ 2ULL };
		PR_EXPECT(div.m_type == Number::EType::UInt);
		PR_EXPECT(div.ul() == (1ULL << 62));
	}
}
#endif
