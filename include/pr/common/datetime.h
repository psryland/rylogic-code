//***********************************************
// DateTime
//  Copyright (c) Rylogic Ltd 2013
//***********************************************
#pragma once

#include <chrono>
#include <concepts>
#include "pr/common/to.h"
#include "pr/common/fmt.h"
#include "pr/str/string.h"

namespace pr
{
	// Duration to formatted string.
	// Supported format specifiers:
	//   %Y - years,        %y - years
	//   %D - days,         %d - days % 365
	//   %H - hours,        %h - hours % 24
	//   %M - minutes       %m - minutes % 60
	//   %S - seconds       %s - seconds % 60
	//   %F - milliseconds  %f - milliseconds % 1000
	//   %U - microseconds  %u - microseconds % 1000
	//   %N - nanoseconds   %n - nanoseconds % 1000
	// Use repeated format specifiers to indicate minimum characters
	// e.g. %sss for 23seconds = 023
	namespace convert
	{
		template <StringType Str>
		struct DateTimeToString
		{
			using Char = typename Str::value_type;

			template <typename TRep, typename TPeriod>
			static Str Func(std::chrono::duration<TRep,TPeriod> duration, Char const* fmt = nullptr)
			{
				fmt = fmt ? fmt : PR_STRLITERAL(Char, "%s");
				return FmtF(fmt, [=](Char const*& code)
				{
					using namespace std::chrono;
					auto f = PR_STRLITERAL(Char, "%0*d");
					int dp = 1;
					for (auto start = code; *(code + 1) == *start; ++code) { ++dp; }
					switch (*code)
					{
					case 'Y': return pr::Fmt(f, dp, duration_cast<years        >(duration).count()       );
					case 'y': return pr::Fmt(f, dp, duration_cast<years        >(duration).count()       );
					case 'D': return pr::Fmt(f, dp, duration_cast<days         >(duration).count()       );
					case 'd': return pr::Fmt(f, dp, duration_cast<days         >(duration).count() % 365 );
					case 'H': return pr::Fmt(f, dp, duration_cast<hours        >(duration).count()       );
					case 'h': return pr::Fmt(f, dp, duration_cast<hours        >(duration).count() % 24  );
					case 'M': return pr::Fmt(f, dp, duration_cast<minutes      >(duration).count()       );
					case 'm': return pr::Fmt(f, dp, duration_cast<minutes      >(duration).count() % 60  );
					case 'S': return pr::Fmt(f, dp, duration_cast<seconds      >(duration).count()       );
					case 's': return pr::Fmt(f, dp, duration_cast<seconds      >(duration).count() % 60  );
					case 'F': return pr::Fmt(f, dp, duration_cast<milliseconds >(duration).count()       );
					case 'f': return pr::Fmt(f, dp, duration_cast<milliseconds >(duration).count() % 1000);
					case 'U': return pr::Fmt(f, dp, duration_cast<microseconds >(duration).count()       );
					case 'u': return pr::Fmt(f, dp, duration_cast<microseconds >(duration).count() % 1000);
					case 'N': return pr::Fmt(f, dp, duration_cast<nanoseconds  >(duration).count()       );
					case 'n': return pr::Fmt(f, dp, duration_cast<nanoseconds  >(duration).count() % 1000);
					default: throw std::runtime_error("unknown string format code");
					}
				});
			}
		};
	}
	template <typename TRep, typename TPeriod, typename Char>
	struct Convert<std::basic_string<Char>, std::chrono::duration<TRep,TPeriod>> :convert::DateTimeToString<std::basic_string<Char>>
	{};
	template <typename TRep, typename TPeriod, typename Char, int L, bool F>
	struct Convert<pr::string<Char,L,F>, std::chrono::duration<TRep,TPeriod>> :convert::DateTimeToString<pr::string<Char,L,F>>
	{};

	namespace datetime
	{
		// Convert a duration into a count down
		// XXX days XX hours XX mins XX secs
		enum class EMaxUnit { Years, Days, Hours, Minutes, Seconds };
		inline std::string ToCountdownString(std::chrono::seconds period, EMaxUnit max_unit)
		{
			// Ignores leap years
			static double const seconds_p_min = 60.0;
			static double const seconds_p_hour = 60.0 * seconds_p_min;
			static double const seconds_p_day = 24.0 * seconds_p_hour;
			static double const seconds_p_year = 365.0 * seconds_p_day;
			auto seconds = static_cast<double>(period.count());

			std::string s;
			switch (max_unit)
			{
				case EMaxUnit::Years:
				{
					long years = long(seconds / seconds_p_year);
					s.append(pr::FmtS("%dyrs ", years));
					seconds -= years * seconds_p_year;
					[[fallthrough]];
				}
				case EMaxUnit::Days:
				{
					long days = long(seconds / seconds_p_day);
					s.append(pr::FmtS("%ddays ", days));
					seconds -= days * seconds_p_day;
					[[fallthrough]];
				}
				case EMaxUnit::Hours:
				{
					long hours = long(seconds / seconds_p_hour);
					s.append(pr::FmtS("%dhrs ", hours));
					seconds -= hours * seconds_p_hour;
					[[fallthrough]];
				}
				case EMaxUnit::Minutes:
				{
					long mins = long(seconds / seconds_p_min);
					s.append(pr::FmtS("%dmins ", mins));
					seconds -= mins * seconds_p_min;
					[[fallthrough]];
				}
				case EMaxUnit::Seconds:
				{
					s.append(pr::FmtS("%2.3fsecs ", seconds));
					[[fallthrough]];
				}
				default:
				{
					break;
				}
			}
			s.resize(s.size() - 1);
			return s;
		}
	}
}


#if PR_UNITTESTS
#include <iomanip>
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(DateTimeTests, Quick)
	{
		using namespace std::chrono;

		{
			auto t = seconds(1234);
			auto s = pr::To<std::string>(t, "%S seconds");
			PR_EXPECT(s == "1234 seconds");
		}
		{
			auto t = hours(1) + minutes(23) + seconds(45) + milliseconds(67);
			auto s = pr::To<std::string>(t, "%hh:%mm:%ss.%fff");
			PR_EXPECT(s == "01:23:45.067");
		}
	}
}
#endif
