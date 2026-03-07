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
	PRUnitTest(DateTimeTests)
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


/*
OLD STUFF, DON'T USE

#include <cassert>
#include <ctime>
#include <chrono>
#include <exception>
#include "pr/macros/constexpr.h"
#include "pr/macros/noexcept.h"
#include "pr/common/to.h"
#include "pr/common/fmt.h"
#include "pr/math/math.h"
#include "pr/str/string.h"

namespace pr
{
	// Julian year, these are not in the standard because of ambiguity over calender type
	typedef std::chrono::duration<long long, std::ratio<86400>>    days;
	typedef std::chrono::duration<long long, std::ratio<31557600>> years;

	namespace datetime
	{
		// These functions are from the 'chrono-Compatible Low-Level Date Algorithms'
		// by http://howardhinnant.github.io/date_algorithms.html
		// Notes:
		//  These algorithms internally assume that March 1 is the first day of the year.
		//  This is convenient because it puts the leap day, Feb. 29 as the last day of
		//  the year, or actually the preceding year.

		// Returns number of days since civil 1970-01-01.
		// Negative values indicate days prior to 1970-01-01.
		//  y-m-d represents a date in the civil (Gregorian) calendar
		//  m is in [1, 12]
		//  d is in [1, last_day_of_month(y, m)]
		//  y is "approximately" in     [numeric_limits<Int>::min()/366, numeric_limits<Int>::max()/366]
		//  Exact range of validity is: [civil_from_days(numeric_limits<Int>::min()), civil_from_days(numeric_limits<Int>::max()-719468)]
		template <class Int> Int days_from_civil(Int y, int m, int d) noexcept
		{
			static_assert(std::numeric_limits<int>::digits >= 18, "This algorithm has not been ported to a 16 bit unsigned integer");
			static_assert(std::numeric_limits<Int>::digits >= 20, "This algorithm has not been ported to a 16 bit signed integer");
			y -= m <= 2;
			const Int era = (y >= 0 ? y : y-399) / 400;
			const int yoe = static_cast<int>(y - era * 400);      // [0, 399]
			const int doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1;  // [0, 365]
			const int doe = yoe * 365 + yoe/4 - yoe/100 + doy;         // [0, 146096]
			return era * 146097 + static_cast<Int>(doe) - 719468;
		}

		// Returns year/month/day triple in civil calendar
		//  z is number of days since 1970-01-01 and is in the range: [numeric_limits<Int>::min(), numeric_limits<Int>::max()-719468].
		template <class Int> std::tuple<Int, int, int> civil_from_days(Int z) noexcept
		{
			static_assert(std::numeric_limits<int>::digits >= 18, "This algorithm has not been ported to a 16 bit unsigned integer");
			static_assert(std::numeric_limits<Int>::digits >= 20, "This algorithm has not been ported to a 16 bit signed integer");
			z += 719468;
			const Int era = (z >= 0 ? z : z - 146096) / 146097;
			const int doe = static_cast<int>(z - era * 146097);          // [0, 146096]
			const int yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;  // [0, 399]
			const Int y = static_cast<Int>(yoe) + era * 400;
			const int doy = doe - (365*yoe + yoe/4 - yoe/100);                // [0, 365]
			const int mp = (5*doy + 2)/153;                                   // [0, 11]
			const int d = doy - (153*mp+2)/5 + 1;                             // [1, 31]
			const int m = mp + (mp < 10 ? 3 : -9);                            // [1, 12]
			return std::tuple<Int, int, int>(y + (m <= 2), m, d);
		}

		// Returns: true if y is a leap year in the civil calendar, else false
		template <class Int> constexpr inline bool is_leap(Int y) noexcept
		{
			return  y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
		}

		// Preconditions: m is in [1, 12]
		// Returns: The number of days in the month m of common year
		// The result is always in the range [28, 31].
		inline int last_day_of_month_common_year(int m) noexcept
		{
			static int const a[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
			return a[m-1];
		}

		// Preconditions: m is in [1, 12]
		// Returns: The number of days in the month m of leap year
		// The result is always in the range [29, 31].
		inline int last_day_of_month_leap_year(int m) noexcept
		{
			static int const a[]{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
			return a[m-1];
		}

		// Preconditions: m is in [1, 12]
		// Returns: The number of days in the month m of year y
		// The result is always in the range [28, 31].
		template <class Int> constexpr inline int last_day_of_month(Int y, int m) noexcept
		{
			return m != 2 || !is_leap(y) ? last_day_of_month_common_year(m) : 29u;
		}

		// Returns day of week in civil calendar [0, 6] -> [Sun, Sat]
		// z is number of days since 1970-01-01 and is in the range: [numeric_limits<Int>::min(), numeric_limits<Int>::max()-4].
		template <class Int> constexpr inline int weekday_from_days(Int z) noexcept
		{
			return static_cast<int>(z >= -4 ? (z+4) % 7 : (z+5) % 7 + 6);
		}

		// Returns: The number of days from the weekday y to the weekday x.
		// Preconditions: x <= 6 && y <= 6
		// The result is always in the range [0, 6].
		constexpr inline int weekday_difference(int x, int y) noexcept
		{
			return (x - y) <= 6 ? (x - y) : (x - y) + 7;
		}

		// Returns: The weekday following wd
		// Preconditions: wd <= 6
		// The result is always in the range [0, 6].
		constexpr inline int next_weekday(int wd) noexcept
		{
			return wd < 6 ? wd+1 : 0;
		}

		// Returns: The weekday prior to wd
		// Preconditions: wd <= 6
		// The result is always in the range [0, 6].
		constexpr inline int prev_weekday(int wd) noexcept
		{
			return wd > 0 ? wd-1 : 6;
		}

	}

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
		template <typename Str, typename = std::enable_if_t<is_string_v<Str>>>
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

	// Helper wrapper around 'tm'
	struct DateTimeStruct :tm
	{
		DateTimeStruct() :tm() {}

		// yr   = year (e.g. 1976, 2014, etc)
		// mon  = month [1,12] -> [Jan,Dec]
		// mday = month_day [1,31]
		// hr   = hour [0,23]
		// min  = minute [0,59]
		// sec  = second [0,60]
		// dls  = daylight savings time in effect
		DateTimeStruct(int yr, int mon, int mday, int hr, int min, int sec, int dls = -1) :tm()
		{
			tm_year = yr - 1900; // years since 1900                 
			tm_mon  = mon - 1;   // months since January - [0,11]    
			tm_mday = mday;      // day of the month - [1,31]        
			tm_hour = hr;        // hours since midnight - [0,23]    
			tm_min  = min;       // minutes after the hour - [0,59]  
			tm_sec  = sec;       // seconds after the minute - [0,59]
			tm_isdst = dls;      // daylight savings time flag       

			// time_t mktime (struct tm * timeptr);
			//  Convert tm structure to time_t
			//  Returns the value of type time_t that represents the local time described
			//  by the tm structure pointed by timeptr (which may be modified).
			//  This function performs the reverse translation that localtime() does.
			//  The values of the members tm_wday and tm_yday of timeptr are ignored, and
			//  the values of the other members are interpreted even if out of their valid
			//  ranges (see struct tm). For example, tm_mday may contain values above 31,
			//  which are interpreted accordingly as the days that follow the last day of the selected month.
			//  A call to this function automatically adjusts the values of the members of
			//  timeptr if they are off-range or -in the case of tm_wday and tm_yday- if they
			// have values that do not match the date described by the other members.
			auto ticks = _mkgmtime(this);
			if (ticks == -1)
				throw std::runtime_error("calender time cannot be represented");
		}

		enum EDaylightSaving { Unknown = -1, NotInEffect = 0, InEffect = 1 };
		EDaylightSaving daylight_savings() const { return EDaylightSaving((tm_isdst > 0) - (tm_isdst < 0)); }

		// Get/Set the second [0,59]
		int second() const { return tm_sec; }
		void second(int s) { tm_sec = s; }

		// Get/Set the minute [0,59]
		int minute() const { return tm_min; }
		void minute(int m) { tm_min = m; }

		// Get/Set the hour [0,23]
		int hour() const { return tm_hour; }
		void hour(int h) { tm_hour = h; }

		// Get/Set the day of the month [1,31]
		int month_day() const { return tm_mday; }
		void month_day(int mday) { tm_mday = mday; }

		// Get/Set the month [1,12]
		int month() const { return tm_mon + 1; }
		void month(int m) { tm_mon = m - 1; }

		// Get/Set the year [1900,3000]
		int year() const { return tm_year + 1900; }
		void year(int y) { tm_year = y - 1900; }

		// Get/Set the weekday [0,6]
		enum EWeekday { Sunday = 0, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday };
		EWeekday week_day() const { return EWeekday(tm_wday); }
		void week_day(EWeekday wday) { tm_wday = wday; }

		// Get/Set the year day[1,365]
		int year_day() const { return tm_yday + 1; }
		void year_day(int yday) { tm_yday = yday - 1; }

		// Return the DateTimeStruct tm's are relative to
		static DateTimeStruct Epoch()
		{
			return DateTimeStruct(1970,1,1,0,0,0,0);
		}

		// Return the time as a string
		std::string ToString() const
		{
			char buf[256];
			if (asctime_s(buf, this) == 0) throw std::runtime_error("asctime_s failed");
			std::string s = buf;
			while (s.rbegin() != s.rend() && *s.rbegin() == '\n') s.resize(s.size() - 1);
			return str::TrimChars(s, "\n", false, true);
		}

		// Return a pretty string representation of the time
		// The formatting codes are:
		// %a - Abbreviated weekday name
		// %A - Full weekday name
		// %b - Abbreviated month name
		// %B - Full month name
		// %c - Date and time representation appropriate for locale
		// %d - Day of month as decimal number (01 - 31)
		// %H - Hour in 24-hour format (00 - 23)
		// %I - Hour in 12-hour format (01 - 12)
		// %j - Day of year as decimal number (001 - 366)
		// %m - Month as decimal number (01 - 12)
		// %M - Minute as decimal number (00 - 59)
		// %p - Current locale's A.M./P.M. indicator for 12-hour clock
		// %S - Second as decimal number (00 - 59)
		// %U - Week of year as decimal number, with Sunday as first day of week (00 - 53)
		// %w - Weekday as decimal number (0 - 6; Sunday is 0)
		// %W - Week of year as decimal number, with Monday as first day of week (00 - 53)
		// %x - Date representation for current locale
		// %X - Time representation for current locale
		// %y - Year without century, as decimal number (00 - 99)
		// %Y - Year with century, as decimal number
		// %z, %Z - Either the time-zone name or time zone abbreviation, depending on registry settings; no characters if time zone is unknown
		// %% - Percent sign
		// As in the printf function, the # flag may prefix any formatting code. In that case,
		// the meaning of the format code is changed as follows.
		// Format code | Meaning
		// %#a, %#A, %#b, %#B, %#p, %#X, %#z, %#Z, %#% | # flag is ignored.
		// %#c | Long date and time representation, appropriate for current locale. For example: "Tuesday, March 14, 1995, 12:41:29".
		// %#x | Long date representation, appropriate to current locale. For example: "Tuesday, March 14, 1995".
		// %#d, %#H, %#I, %#j, %#m, %#M, %#S, %#U, %#w, %#W, %#y, %#Y | Remove leading zeros (if any).
		std::string ToString(char const* fmt) const
		{
			char buf[256];
			if (strftime(buf, sizeof(buf), fmt, this) == 0) throw std::runtime_error("strftime failed");
			return buf;
		}
	};

	// Represents a time_point (modelled off the .NET DateTimeOffset class)
	struct DateTime
	{
		using clock_t = std::chrono::system_clock;                   // The system clock
		using ticks_t = std::chrono::nanoseconds;                    // In units of nanoseconds
		using date_point_t = std::chrono::time_point<clock_t, days>; // In units of days

		date_point_t m_date; // Days relative to 1970-1-1 00:00:00
		ticks_t m_time;      // UTC time (in ns), relative to 'm_date'
		ticks_t m_offset;    // Offset (in ns) from utc to local time

		DateTime()
			:m_date()
			,m_time()
			,m_offset()
		{}
		DateTime(date_point_t date, ticks_t time, ticks_t offset = ticks_t::zero())
			:m_date(date)
			,m_time(time)
			,m_offset(offset)
		{}

		// yr   = year (e.g. 1976, 2014, etc)
		// mon  = month [1,12] -> [Jan,Dec]
		// mday = month_day [1,31]
		// hr   = hour [0,23]
		// min  = minute [0,59]
		// sec  = second [0,60]
		// ofs  = offset from utc
		DateTime(int yr, int mon, int mday, int hr, int min, int sec, std::chrono::hours utc_ofs = std::chrono::hours::zero())
			:m_date(days(pr::datetime::days_from_civil(yr, mon, mday)))
			,m_time(std::chrono::hours(hr) + std::chrono::minutes(min) + std::chrono::seconds(sec))
			,m_offset(utc_ofs)
		{
			assert(mday >= 1 && mday <= (int)pr::datetime::last_day_of_month(yr, mon) && "month day is invalid");
		}

		// Construct from C's time_t
		DateTime(time_t t, std::chrono::hours utc_ofs = std::chrono::hours::zero())
		{
			from_time_t(t, utc_ofs);
		}

		// Local time value
		ticks_t local_time() const { return m_time + m_offset; }

		// Convert this time to/from a C time_t (throws if out of range)
		std::time_t to_time_t() const
		{
			using namespace std::chrono;
			return clock_t::to_time_t(time_point_cast<seconds>(m_date + m_time));
		}
		void from_time_t(time_t t, std::chrono::hours utc_ofs = std::chrono::hours::zero())
		{
			using namespace std::chrono;

			// time_t is not necessarily since 1970, need to use difftime() to get the time difference
			// Likewise, the epoch for clock_t is unknown
			auto unix_epoch      = DateTimeStruct::Epoch();
			auto t0              = _mkgmtime(&unix_epoch);                              // Get a time_t representing 1/1/1970
			auto secs_since_1970 = seconds::rep(std::difftime(t, t0));                  // Find the number of seconds since 1970
			auto timepoint_at_t  = clock_t::from_time_t(t0) + seconds(secs_since_1970); // The clock_t::time_point at 't' = the clock_t value at 1/1/1970 and add on the number of seconds

			m_date = time_point_cast<days>(timepoint_at_t);
			m_time = timepoint_at_t - m_date;
			m_offset = utc_ofs;
		}

		// Pretty print the datetime
		std::string ToString() const
		{
			return "ToDo";
			//::ctime(&dt.ticks)
		}

		// Return the DateTime that DateTime's are relative to
		static DateTime Epoch()
		{
			return DateTime(1970,1,1,0,0,0);
		}

		// The current system time in UTC
		static DateTime NowUTC()
		{
			return DateTime(time(nullptr));
		}

		// The current system time in local time
		static DateTime Now()
		{
			using namespace std::chrono;

			auto time = clock_t::now();
			auto date = time_point_cast<days>(time);
			auto tz_offset = current_zone()->get_info(time).offset;
			return DateTime(date, time_point_cast<ticks_t>(time), duration_cast<ticks_t>(tz_offset));
			//tm tmp;
			//if (gmtime_s(&tmp, &now) != 0) throw std::runtime_error("failed to convert 'now' to UTC");
			//auto utc_ofs = seconds(seconds::rep(std::difftime(now, mktime(&tmp))));
			//return DateTime(now, duration_cast<hours>(utc_ofs));
		}

		// Implicate conversion to time point
		operator std::chrono::time_point<clock_t, ticks_t>()
		{
			return m_date + m_time;
		}

		// Time points are equivalent if they represent the same UTC time
		// The utc offset describes geographical location, not time.
		friend bool operator == (DateTime const& lhs, DateTime const& rhs) { return lhs.m_date == rhs.m_date && lhs.m_time == rhs.m_time; }
		friend bool operator != (DateTime const& lhs, DateTime const& rhs) { return !(lhs == rhs); }
		friend bool operator <  (DateTime const& lhs, DateTime const& rhs) { return lhs.m_date != rhs.m_date ? lhs.m_date < rhs.m_date : lhs.m_time < rhs.m_time; }
		friend bool operator >  (DateTime const& lhs, DateTime const& rhs) { return rhs < lhs; }
		friend bool operator <= (DateTime const& lhs, DateTime const& rhs) { return !(lhs > rhs); }
		friend bool operator >= (DateTime const& lhs, DateTime const& rhs) { return !(lhs < rhs); }
	};

	// Represents a difference of DateTimes
	struct TimeSpan
	{
		using clock_t = DateTime::clock_t; // The system clock
		using ticks_t = DateTime::ticks_t; // In units of nanoseconds
		using date_duration_t = DateTime::date_point_t::duration; // In units of days

		date_duration_t m_ddate;   // delta date
		ticks_t m_dtime;   // delta time
		// Note, delta timezone represents a geographical location difference, not a time difference

		TimeSpan()
			:m_ddate()
			,m_dtime()
		{}
		TimeSpan(date_duration_t ddate, ticks_t dtime)
			:m_ddate(ddate)
			,m_dtime(dtime)
		{}
		
		// Construct from a std::chrono::duration
		template <typename Rep,typename Period> TimeSpan(std::chrono::duration<Rep,Period> duration)
			:m_ddate(std::chrono::duration_cast<date_duration_t>(duration))
			,m_dtime(std::chrono::duration_cast<ticks_t>(duration - m_ddate))
		{}

		// Converts the timespan into a std::chrono::duration
		template <typename Duration> Duration To() const
		{
			return std::chrono::duration_cast<Duration>(m_ddate + m_dtime);
		}
	};

	inline TimeSpan operator - (TimeSpan const& rhs)
	{
		return TimeSpan(-rhs.m_ddate, -rhs.m_dtime);
	}
	inline DateTime operator + (DateTime const& lhs, TimeSpan const& rhs)
	{
		return DateTime(lhs.m_date + rhs.m_ddate, lhs.m_time + rhs.m_dtime, lhs.m_offset);
	}
	inline TimeSpan operator - (DateTime const& lhs, DateTime const& rhs)
	{
		return TimeSpan(lhs.m_date - rhs.m_date, lhs.m_time - rhs.m_time);
	}
	inline TimeSpan operator + (TimeSpan const& lhs, TimeSpan const& rhs)
	{
		return TimeSpan(lhs.m_ddate + rhs.m_ddate, lhs.m_dtime + rhs.m_dtime);
	}
	inline TimeSpan operator - (TimeSpan const& lhs, TimeSpan const& rhs)
	{
		return lhs + (-rhs);
	}
}

#if PR_UNITTESTS
#include <iomanip>
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(DateTimeTests)
	{
		using namespace std::chrono;
		using namespace pr::datetime;

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

		{// Testing DateTime
			auto tz_offset = std::chrono::current_zone()->get_info(system_clock::now()).offset;
			auto tz_hours = duration_cast<hours>(tz_offset).count();

			auto dt1 = DateTime::NowUTC();
			auto dt2 = DateTime::Now();
			auto ofs1 = dt2 - dt1;
			auto ofs1_seconds = ofs1.To<seconds>().count();
			auto ofs_hours = duration_cast<hours>(dt2.m_offset - dt1.m_offset).count();
			PR_EXPECT(ofs1_seconds == 0); // Both times should represent the same moment in time
			PR_EXPECT(ofs_hours == tz_hours);

			auto dt3 = DateTime(1976,12,29,3,45,0,hours(12));
			auto dt4 = DateTime(1977,12,8,10,15,0,hours(12));
			auto ofs2 = dt4 - dt3;
			PR_EXPECT(ofs2.To<seconds>().count() == 29745000);

			auto ts1 = TimeSpan(days(1) + seconds(5000));
			PR_EXPECT(ts1.To<minutes>().count() == 1523);
		}

		{ // unit test of chrono-Compatible Low-Level Date Algorithms
			PR_EXPECT(days_from_civil(1970, 1, 1) == 0);                    // 1970-01-01 is day 0
			PR_EXPECT(civil_from_days(0) == std::make_tuple(1970, 1, 1));   // 1970-01-01 is day 0
			PR_EXPECT(weekday_from_days(days_from_civil(1970, 1, 1)) == 4); // 1970-01-01 is a Thursday

			auto ystart = -10;// for speed instead of -1000000;
			auto prev_z = days_from_civil(ystart, 1, 1) - 1;
			PR_EXPECT(prev_z < 0);
				
			auto prev_wd = weekday_from_days(prev_z);
			PR_EXPECT(0 <= prev_wd && prev_wd <= 6);

			for (auto y = ystart; y <= -ystart; ++y)
			{
				for (auto m = 1; m <= 12; ++m)
				{
					auto e = last_day_of_month(y, m);
					for (auto d = 1; d <= e; ++d)
					{
						int z = days_from_civil(y, m, d);
						PR_EXPECT(prev_z < z);
						PR_EXPECT(z == prev_z+1);

						int yp; int mp, dp;
						std::tie(yp, mp, dp) = civil_from_days(z);
						PR_EXPECT(y == yp);
						PR_EXPECT(m == mp);
						PR_EXPECT(d == dp);

						auto wd = weekday_from_days(z);
						PR_EXPECT(0 <= wd && wd <= 6);
						PR_EXPECT(wd == next_weekday(prev_wd));
						PR_EXPECT(prev_wd == prev_weekday(wd));
						prev_z = z;
						prev_wd = wd;
					}
				}
			}
			auto count_days = days_from_civil(1000000, 12, 31) - days_from_civil(-1000000, 1, 1);
			PR_EXPECT(count_days == 730485365);
		}
		{// Example of using the datetime functions to avoid C time interfaces
			long long year; unsigned month; unsigned day;
			hours h; minutes m; seconds s; microseconds us;
			std::stringstream ss; std::string str;
			auto utc_offset = hours(+12);  // my current UTC offset

			// Get duration in local units
			auto now = system_clock::now().time_since_epoch() + utc_offset;

			// Get duration in days
			auto today = duration_cast<days>(now);

			// Convert days into year/month/day
			std::tie(year, month, day) = civil_from_days(today.count());

			// Subtract off days, leaving now containing time since local midnight
			now -= today; h  = duration_cast<hours>(now);
			now -= h;     m  = duration_cast<minutes>(now);
			now -= m;     s  = duration_cast<seconds>(now);
			now -= s;     us = duration_cast<microseconds>(now);

			ss = std::stringstream{};
			ss << "Today is "
					<< year << '-' << std::setw(2) << month << '-' << std::setw(2) << day << " at "
					<< std::setw(2) << h.count() << ':'
					<< std::setw(2) << m.count() << ':'
					<< std::setw(2) << s.count() << '.'
					<< std::setw(6) << us.count();
			str = ss.str();

			// Can also go the other way: Specify a date in terms of a year/month/day triple and then convert that into a system_clock::time_point:
			// Build a time point in local days::hours::minutes and then convert to UTC
			auto birthdate = system_clock::time_point(days(days_from_civil(1976, 12, 29)) + hours(03) + minutes(45) - utc_offset);
			ss = std::stringstream{};
			ss << "Paul is " << duration_cast<seconds>(system_clock::now() - birthdate).count() << " seconds old\n";
			str = ss.str();

			// current utc date time
			now = system_clock::now().time_since_epoch();
			today = duration_cast<days>(now);

			std::tie(year, month, day) = civil_from_days(today.count());

			now -= today; h = duration_cast<hours>(now);
			now -= h;     m = duration_cast<minutes>(now);
			now -= m;     s = duration_cast<seconds>(now);

			ss = std::stringstream{};
			ss << "Today is " << year << '-' << std::setw(2) << (unsigned)month << '-' << std::setw(2) << (unsigned)day << " at "
				<< std::setw(2) << h.count() << ':'
				<< std::setw(2) << m.count() << ':'
				<< std::setw(2) << s.count() << " UTC\n";
			str = ss.str();
		}
	}
}
#endif
*/