//*************************************************************
// Number
//  Copyright (c) Rylogic Ltd 2008
//*************************************************************
// A value type that is either a double, 'long long', or 'unsigned long long'
#pragma once
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cassert>
#include <exception>

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
		explicit Number(char const* expr, char const** end = nullptr, int radix = 0)
		{
			*this = From(expr, end, radix);
		}
		explicit Number(wchar_t const* expr, wchar_t const** end = nullptr, int radix = 0)
		{
			*this = From(expr, end, radix);
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

		#pragma region traits
		template <typename Char> struct traits_base;
		template <> struct traits_base<char>
		{
			static double             strtod(char const* str, char** end)                   { return ::strtod(str, end); }
			static long               strtol(char const* str, char** end, int radix)        { return ::strtol(str, end, radix); }
			static unsigned long      strtoul(char const* str, char** end, int radix)       { return ::strtoul(str, end, radix); }
			static long long          strtoi64(char const* str, char** end, int radix)      { return ::_strtoi64(str, end, radix); }
			static unsigned long long strtoui64(char const* str, char** end, int radix)     { return ::_strtoui64(str, end, radix); }
			static int                strnicmp(char const* lhs, char const* rhs, int count) { return ::_strnicmp(lhs, rhs, count); }
			static char const*        str(char const* str, wchar_t const*)                  { return str; }
		};
		template <> struct traits_base<wchar_t>
		{
			static double             strtod(wchar_t const* str, wchar_t** end)                   { return ::wcstod(str, end); }
			static long               strtol(wchar_t const* str, wchar_t** end, int radix)        { return ::wcstol(str, end, radix); }
			static long long          strtoi64(wchar_t const* str, wchar_t** end, int radix)      { return ::_wcstoi64(str, end, radix); }
			static unsigned long      strtoul(wchar_t const* str, wchar_t** end, int radix)       { return ::wcstoul(str, end, radix); }
			static unsigned long long strtoui64(wchar_t const* str, wchar_t** end, int radix)     { return ::_wcstoui64(str, end, radix); }
			static int                strnicmp(wchar_t const* lhs, wchar_t const* rhs, int count) { return ::_wcsnicmp(lhs, rhs, count); }
			static wchar_t const*     str(char const*, wchar_t const* str)                        { return str; }
		};
		#pragma endregion

		// Read a value (greedily) from 'expr'
		template <typename Char> static Number From(Char const* expr, Char const** end = nullptr, int radix = 0)
		{
			using traits = traits_base<Char>;

			// Read as a floating point number
			Char* endf; errno = 0;
			auto db = traits::strtod(expr, &endf);

			// Read as a integer
			Char* endi; errno = 0;
			auto ll = traits::strtoi64(expr, &endi, radix);

			// Read as an unsigned integer
			Char* endu; errno = 0;
			auto ul = traits::strtoui64(expr, &endu, radix);

			Number num;

			// If no characters contributed to the number, assume not a number
			if (endf == expr && endi == expr && endu == expr)
			{
				if (end) *end = const_cast<Char*>(expr);
				return num;
			}

			// If more chars are read as a float, then assume a float
			if (endf > endi && endf > endu)
			{
				num.m_db = db;
				num.m_type = EType::FP;

				if (end)
				{
					// Skip suffix characters
					if (*endf == 'f' || *endf == 'F') ++endf;
					*end = endf;
				}
			}
			// If more characters are read as an unsigned int, assume unsigned int
			else if (endu > endi)
			{
				num.m_ul = ul;
				num.m_type = EType::UInt;
				if (end)
				{
					// Skip suffix characters
					if (*endu == 'u' || *endu == 'U') ++endu;
					if (*endu == 'l' || *endu == 'L') ++endu;
					if (*endu == 'l' || *endu == 'L') ++endu;
					*end = endu;
				}
			}
			// Otherwise assume signed integer
			else
			{
				num.m_ll = ll;
				num.m_type = EType::Int;
				if (end)
				{
					// Skip suffix characters
					if (*endi == 'u' || *endi == 'U') ++endi;
					if (*endi == 'l' || *endi == 'L') ++endi;
					if (*endi == 'l' || *endi == 'L') ++endi;
					*end = endi;
				}
			}
			return num;
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
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.ul() + rhs.ul()) :
				Number(lhs.ll() + rhs.ll());
		}
		friend Number operator - (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() - rhs.db()) :
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.ul() - rhs.ul()) :
				Number(lhs.ll() - rhs.ll());
		}
		friend Number operator * (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() * rhs.db()) :
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.ul() * rhs.ul()) :
				Number(lhs.ll() * rhs.ll());
		}
		friend Number operator / (Number const& lhs, Number const& rhs)
		{
			return
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.db() / rhs.db()) :
				(lhs.m_type == EType::FP || rhs.m_type == EType::FP) ? Number(lhs.ul() / rhs.ul()) :
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

		auto n6 = Number("10110101", nullptr, 2);
		PR_EXPECT(FEql(n6.db(), static_cast<double>(0b10110101)));
		PR_EXPECT(n6.ll() == 0b10110101LL);
		PR_EXPECT(n6.ul() == 0b10110101ULL);
	}
}
#endif
