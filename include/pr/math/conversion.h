//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include <format>
#include "pr/common/to.h"
#include "pr/common/cast.h"
#include "pr/str/string_core.h"
#include "pr/math/math.h"

namespace pr
{
	// Vec2
	template <StringTypeDynamic Str, math::ScalarType S>
	struct Convert<Str, math::Vec2<S>>
	{
		static Str Func(math::Vec2<S> x)
		{
			using Char = typename string_traits<Str>::value_type;
			if constexpr (std::is_same_v<Char, char>)
				return std::format("{} {}", x.x, x.y);
			else if constexpr (std::is_same_v<Char, wchar_t>)
				return std::format(L"{} {}", x.x, x.y);
		}
	};
	template <math::ScalarType S, typename TFrom>
	struct Convert<math::Vec2<S>, TFrom>
	{
		// Vec2 to Vec2
		template <math::ScalarType SS> static math::Vec2<S> Func(math::Vec2<SS> v) noexcept
		{
			return math::Vec2<S>(s_cast<S>(v.x), s_cast<S>(v.y));
		}

		// String to Vec2<floating_point>
		template <StringType Str>
		static math::Vec2<S> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) requires std::floating_point<S>
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, &e);
			auto y = To<S>(e, &e);
			if (end) *end = e;
			return math::Vec2<S>(x, y);
		}

		// String to Vec2<integral>
		template <StringType Str>
		static math::Vec2<S> Func(Str const& s, int radix = 10, typename string_traits<Str>::value_type const** end = nullptr) requires std::integral<S>
		{
			using Char = typename string_traits<Str>::value_type;
			
			Char const* e = {};
			auto x = To<S>(s, radix, &e);
			auto y = To<S>(e, radix, &e);
			if (end) *end = e;
			return math::Vec2<S>(x, y);
		}

		// Rect to Vec2
		static math::Vec2<S> Func(FRect x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.SizeX()), s_cast<S>(x.SizeY()));
		}
		static math::Vec2<S> Func(IRect x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.SizeX()), s_cast<S>(x.SizeY()));
		}

		// Win32 primitive types to Vec2
		#ifdef _WINDEF_
		static math::Vec2<S> Func(POINT x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.x), s_cast<S>(x.y));
		}
		static math::Vec2<S> Func(SIZE x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.cx), s_cast<S>(x.cy));
		}
		static math::Vec2<S> Func(RECT x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.right - x.left), s_cast<S>(x.bottom - x.top));
		}
		#endif

		// GDI+ types to Vec2
		#ifdef _GDIPLUS_H
		static math::Vec2<S> Func(Gdiplus::Rect x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.Width), s_cast<S>(x.Height));
		}
		static math::Vec2<S> Func(Gdiplus::RectF x) noexcept
		{
			return math::Vec2<S>(s_cast<S>(x.Width), s_cast<S>(x.Height));
		}
		#endif
	};

	// Vec3
	template <StringTypeDynamic Str, math::ScalarType S>
	struct Convert<Str, math::Vec3<S>>
	{
		static Str Func(math::Vec3<S> x)
		{
			using Char = typename string_traits<Str>::value_type;
			if constexpr (std::is_same_v<Char, char>)
				return std::format("{} {} {}", x.x, x.y, x.z);
			else if constexpr (std::is_same_v<Char, wchar_t>)
				return std::format(L"{} {} {}", x.x, x.y, x.z);
		}
	};
	template <math::ScalarType S, typename TFrom>
	struct Convert<math::Vec3<S>, TFrom>
	{
		// Vec3 to Vec3
		template <math::ScalarType SS> static math::Vec3<S> Func(math::Vec3<SS> v) noexcept
		{
			return math::Vec3<S>(s_cast<S>(v.x), s_cast<S>(v.y), s_cast<S>(v.z));
		}

		// String to Vec3
		template <StringType Str>
		static math::Vec3<S> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, &e);
			auto y = To<S>(e, &e);
			auto z = To<S>(e, &e);
			if (end) *end = e;
			return math::Vec3<S>(x, y, z);
		}
		template <typename Str>
		static math::Vec3<S> Func(Str const& s, int radix = 10, typename string_traits<Str>::value_type const** end = nullptr) requires std::integral<S>
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, radix, &e);
			auto y = To<S>(e, radix, &e);
			auto z = To<S>(e, radix, &e);
			if (end) *end = e;
			return math::Vec3<S>(x, y, z);
		}
	};

	// Vec4
	template <StringTypeDynamic Str, math::ScalarType S>
	struct Convert<Str, math::Vec4<S>>
	{
		static Str Func(math::Vec4<S> x)
		{
			using Char = typename string_traits<Str>::value_type;
			if constexpr (std::is_same_v<Char, char>)
				return std::format("{} {} {} {}", x.x, x.y, x.z, x.w);
			else if constexpr (std::is_same_v<Char, wchar_t>)
				return std::format(L"{} {} {} {}", x.x, x.y, x.z, x.w);
		}
	};
	template <math::ScalarType S, typename TFrom> requires (!std::is_same_v<TFrom, math::Vec4<S>>)
	struct Convert<math::Vec4<S>, TFrom>
	{
		// Vec4 to Vec4
		template <math::ScalarType SS> static math::Vec4<S> Func(math::Vec4<SS> v) noexcept
		{
			return math::Vec4<S>(s_cast<S>(v.x), s_cast<S>(v.y), s_cast<S>(v.z), s_cast<S>(v.w));
		}

		// String to Vec4
		template <StringType Str>
		static math::Vec4<S> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, &e);
			auto y = To<S>(e, &e);
			auto z = To<S>(e, &e);
			auto w = To<S>(e, &e);
			if (end) *end = e;
			return math::Vec4<S>(x, y, z, w);
		}
		template <StringType Str>
		static math::Vec4<S> Func(Str const& s, S w, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, &e);
			auto y = To<S>(e, &e);
			auto z = To<S>(e, &e);
			if (end) *end = e;
			return math::Vec4<S>(x, y, z, w);
		}
		template <StringType Str>
		static math::Vec4<S> Func(Str const& s, int radix = 10, typename string_traits<Str>::value_type const** end = nullptr) requires std::integral<S>
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, radix, &e);
			auto y = To<S>(e, radix, &e);
			auto z = To<S>(e, radix, &e);
			auto w = To<S>(e, radix, &e);
			if (end) *end = e;
			return math::Vec4<S>(x, y, z, w);
		}
		template <StringType Str>
		static math::Vec4<S> Func(Str const& s, S w, int radix = 10, typename string_traits<Str>::value_type const** end = nullptr) requires std::integral<S>
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<S>(s, radix, &e);
			auto y = To<S>(e, radix, &e);
			auto z = To<S>(e, radix, &e);
			if (end) *end = e;
			return math::Vec4<S>(x, y, z, w);
		}
	};

	// Vec8
	template <StringTypeDynamic Str, math::ScalarType S, typename T>
	struct Convert<Str, math::Vec8<S, T>>
	{
		static Str Func(math::Vec8<S, T> x)
		{
			using Char = typename string_traits<Str>::value_type;
			if constexpr (std::is_same_v<Char, char>)
				return std::format("{} {} {} {}  {} {} {} {}", x.ang.x, x.ang.y, x.ang.z, x.ang.w, x.lin.x, x.lin.y, x.lin.z, x.lin.w);
			else if constexpr (std::is_same_v<Char, wchar_t>)
				return std::format(L"{} {} {} {}  {} {} {} {}", x.ang.x, x.ang.y, x.ang.z, x.ang.w, x.lin.x, x.lin.y, x.lin.z, x.lin.w);
		}
	};
	template <math::ScalarType S, typename T, typename TFrom>
	struct Convert<math::Vec8<S,T>, TFrom>
	{
		// Vec8 to Vec8
		template <math::ScalarType SS, typename TT> static math::Vec8<S, T> Func(math::Vec8<SS, TT> v) noexcept
		{
			return math::Vec8<S, T>(To<math::Vec4<S>>(v.ang), To<math::Vec4<S>>(v.lin));
		}

		// String to Vec8
		template <StringType Str>
		static math::Vec8<S, T> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) requires std::floating_point<S>
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto ang_x = To<S>(s, &e);
			auto ang_y = To<S>(e, &e);
			auto ang_z = To<S>(e, &e);
			auto ang_w = To<S>(e, &e);
			auto lin_x = To<S>(e, &e);
			auto lin_y = To<S>(e, &e);
			auto lin_z = To<S>(e, &e);
			auto lin_w = To<S>(e, &e);
			if (end) *end = e;
			return math::Vec8<S, T>(
				math::Vec4<S>(ang_x, ang_y, ang_z, ang_w),
				math::Vec4<S>(lin_x, lin_y, lin_z, lin_w));
		}
		template <StringType Str>
		static math::Vec8<S, T> Func(Str const& s, int radix = 10, typename string_traits<Str>::value_type const** end = nullptr) requires std::integral<S>
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto ang_x = To<S>(s, radix, &e);
			auto ang_y = To<S>(e, radix, &e);
			auto ang_z = To<S>(e, radix, &e);
			auto ang_w = To<S>(e, radix, &e);
			auto lin_x = To<S>(e, radix, &e);
			auto lin_y = To<S>(e, radix, &e);
			auto lin_z = To<S>(e, radix, &e);
			auto lin_w = To<S>(e, radix, &e);
			if (end) *end = e;
			return math::Vec8<S, T>(
				math::Vec4<S>(ang_x, ang_y, ang_z, ang_w),
				math::Vec4<S>(lin_x, lin_y, lin_z, lin_w));
		}
	};

	// Mat2x2
	template <StringTypeDynamic Str, math::ScalarType S>
	struct Convert<Str, math::Mat2x2<S>>
	{
		static Str Func(math::Mat2x2<S> const& x)
		{
			using Char = typename string_traits<Str>::value_type;
			return
				To<Str>(x.x) + Char(' ') +
				To<Str>(x.y);
		}
	};
	template <math::ScalarType S, typename TFrom>
	struct Convert<math::Mat2x2<S>, TFrom>
	{
		// String to Mat2x2
		template <StringType Str>
		static math::Mat2x2<S> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<math::Vec2<S>>(s, &e);
			auto y = To<math::Vec2<S>>(e, &e);
			if (end) *end = e;
			return math::Mat2x2<S>(x, y);
		}
	};

	// Mat3x4
	template <StringTypeDynamic Str, math::ScalarType S>
	struct Convert<Str, math::Mat3x4<S>>
	{
		static Str Func(math::Mat3x4<S> const& x)
		{
			using Char = typename string_traits<Str>::value_type;
			return
				To<Str>(x.x) + Char(' ') +
				To<Str>(x.y) + Char(' ') +
				To<Str>(x.z);
		}
	};
	template <math::ScalarType S, typename TFrom>
	struct Convert<math::Mat3x4<S>, TFrom>
	{
		// String to Mat3x4
		template <StringType Str>
		static math::Mat3x4<S> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<Vec3<S, void>>(s, &e);
			auto y = To<Vec3<S, void>>(e, &e);
			auto z = To<Vec3<S, void>>(e, &e);
			if (end) *end = e;
			return math::Mat3x4<S>(x, y, z);
		}
	};

	// Mat4x4
	template <StringTypeDynamic Str, math::ScalarType S>
	struct Convert<Str, math::Mat4x4<S>>
	{
		static Str Func(math::Mat4x4<S> const& x)
		{
			using Char = typename string_traits<Str>::value_type;
			return
				To<Str>(x.x) + Char(' ') +
				To<Str>(x.y) + Char(' ') +
				To<Str>(x.z) + Char(' ') +
				To<Str>(x.w);
		}
	};
	template <math::ScalarType S, typename TFrom>
	struct Convert<math::Mat4x4<S>, TFrom>
	{
		// String to Mat4x4
		template <StringType Str>
		static math::Mat4x4<S> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<math::Vec4<S>>(s, &e);
			auto y = To<math::Vec4<S>>(e, &e);
			auto z = To<math::Vec4<S>>(e, &e);
			auto w = To<math::Vec4<S>>(e, &e);
			if (end) *end = e;
			return math::Mat4x4<S>(x, y, z, w);
		}
	};

	// Mat6x8
	template <StringTypeDynamic Str, math::ScalarType S, typename A, typename B>
	struct Convert<Str, math::Mat6x8<S, A, B>>
	{
		static Str Func(math::Mat6x8<S, A, B> const& x)
		{
			using Char = typename string_traits<Str>::value_type;
			return
				To<Str>(x[0]) + Char(' ') +
				To<Str>(x[1]) + Char(' ') +
				To<Str>(x[2]) + Char(' ') +
				To<Str>(x[3]) + Char(' ') +
				To<Str>(x[4]) + Char(' ') +
				To<Str>(x[5]);
		}
	};
	template <math::ScalarType S, typename A, typename B, typename TFrom>
	struct Convert<math::Mat6x8<S, A, B>, TFrom>
	{
		// Mat6x8 to Mat6x8
		template <math::ScalarType SS, typename AA, typename BB> static math::Mat6x8<S, A, B> Func(math::Mat6x8<SS, AA, BB> const& v) noexcept
		{
			return math::Mat6x8<S, A, B>(
				To<math::Mat3x4<S>>(v.m00),
				To<math::Mat3x4<S>>(v.m01),
				To<math::Mat3x4<S>>(v.m10),
				To<math::Mat3x4<S>>(v.m11));
		}

		// String to Mat6x8
		template <StringType Str>
		static math::Mat6x8<S, A, B> Func(Str const& s, typename string_traits<Str>::value_type const** end = nullptr) noexcept
		{
			using Char = typename string_traits<Str>::value_type;

			Char const* e = {};
			auto x = To<math::Vec8<S>>(s, &e);
			auto y = To<math::Vec8<S>>(e, &e);
			auto z = To<math::Vec8<S>>(e, &e);
			auto u = To<math::Vec8<S>>(e, &e);
			auto v = To<math::Vec8<S>>(e, &e);
			auto w = To<math::Vec8<S>>(e, &e);
			if (end) *end = e;
			return math::Mat6x8<S, A, B>(x, y, z, u, v, w);
		}
	};

	// Rect
	template <math::ScalarType S, typename TFrom>
	struct Convert<math::Rectangle<S>, TFrom>
	{
		template <math::ScalarType V>
		static math::Rectangle<S> Func(math::Vec2<S> x) noexcept
		{
			return math::Rectangle<S>{
				math::Vec2<S>(0, 0),
				To<math::Vec2<S>>(x)
			};
		}
		template <math::ScalarType V>
		static math::Rectangle<S> Func(math::Rectangle<V> x) noexcept
		{
			return math::Rectangle<S>{
				math::Vec2<S>(s_cast<S>(x.m_min.x), s_cast<S>(x.m_min.y)),
				math::Vec2<S>(s_cast<S>(x.m_max.x), s_cast<S>(x.m_max.y))
			};
		}

		#ifdef _WINDEF_
		static Rectangle<S> Func(RECT x) noexcept
		{
			return math::Rectangle<S>{s_cast<S>(x.left), s_cast<S>(x.top), s_cast<S>(x.right), s_cast<S>(x.bottom)};
		}
		static Rectangle<S> Func(SIZE x) noexcept
		{
			return math::Rectangle<S>(S(0), S(0), s_cast<S>(x.cx), s_cast<S>(x.cy));
		}
		#endif
	};

	// Convert an integer to a string of 0s and 1s
	template <StringTypeDynamic Str, std::integral Int>
	inline Str ToBinaryStr(Int n)
	{
		int const bits = sizeof(Int) * 8;
		Str str = {};
		string_traits<Str>::resize(str, bits);
		for (int i = 0, j = bits; i != bits;)
			str[i++] = (n & Bit64(--j)) ? '1' : '0';

		return str;
	}
}
namespace pr::math
{
	// Write a vector to a stream (in pr::math for ADL with /permissive-)
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Vec2<S> vec)
	{
		return out << vec.x << " " << vec.y;
	}
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Vec3<S> vec)
	{
		return out << vec.x << " " << vec.y << " " << vec.z;
	}
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Vec4<S> vec)
	{
		return out << vec.x << " " << vec.y << " " << vec.z << " " << vec.w;
	}
	template <typename Char, ScalarType S, typename T>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Vec8<S,T> vec)
	{
		return out << vec.ang << " " << vec.lin;
	}
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Quat<S> quat)
	{
		return out << quat.xyzw;
	}
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Mat2x2<S> const& mat)
	{
		return out << mat.x << " " << mat.y;
	}
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Mat3x4<S> const& mat)
	{
		auto m = mat;
		auto sep = " ";
		return out << m.x << sep << m.y << sep << m.z;
	}
	template <typename Char, ScalarType S>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Mat4x4<S> const& mat)
	{
		auto m = mat;
		auto sep = " ";
		return out << m.x << sep << m.y << sep << m.z << sep << m.w;
	}
	template <typename Char, ScalarType S, typename A, typename B>
	inline std::basic_ostream<Char>& operator << (std::basic_ostream<Char>& out, Mat6x8<S,A,B> const& mat)
	{
		auto m = mat;
		auto sep = " ";
		return out << mat[0] << sep << mat[1] << sep << mat[2] << sep << mat[3] << sep << mat[4] << sep << mat[5];
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(ToStringTests)
	{
		PRUnitTestMethod(Vec2)
		{
			using namespace pr::math;

			PR_EXPECT(To<std::string>(Vec2<float>(2.125f, -4.75f)) == "2.125 -4.75");
			PR_EXPECT(To<std::string>(Vec2<double>(2.125, -4.75))  == "2.125 -4.75");
			PR_EXPECT(To<std::string>(Vec2<int>(12, -34))          == "12 -34");
			PR_EXPECT(To<std::string>(Vec2<long long>(12, -34))    == "12 -34");

			PR_EXPECT((To<Vec2<double>>("1.2 3.4")) == (Vec2<double>(1.2, 3.4)));
			PR_EXPECT((To<Vec2<int>>("1 -2")      ) == (Vec2<int>(1, -2)));
			PR_EXPECT((To<Vec2<int>>("AA 55", 16) ) == (Vec2<int>(170, 85)));
		}
		PRUnitTestMethod(Vec3)
		{
			using namespace pr::math;

			PR_EXPECT(To<std::string>(Vec3<float>(2.125f, 4.75f, -1.375f)) == "2.125 4.75 -1.375");
			PR_EXPECT(To<std::string>(Vec3<double>(2.125, 4.75, -1.375))   == "2.125 4.75 -1.375");
			PR_EXPECT(To<std::string>(Vec3<int>(12, -34, 56))              == "12 -34 56");
			PR_EXPECT(To<std::string>(Vec3<long long>(12, -34, 56))        == "12 -34 56");
		}
		PRUnitTestMethod(Vec4)
		{
			PR_EXPECT(To<std::string>(Vec4<float>(2.125f, 4.75f, -1.375f, -0.825f)) == "2.125 4.75 -1.375 -0.825");
			PR_EXPECT(To<std::string>(Vec4<double>(2.125, 4.75, -1.375, -0.825))    == "2.125 4.75 -1.375 -0.825");
			PR_EXPECT(To<std::string>(Vec4<int>(12, -34, 56, -78))                  == "12 -34 56 -78");
			PR_EXPECT(To<std::string>(Vec4<long long>(12, -34, 56, -78))            == "12 -34 56 -78");
		}
		PRUnitTestMethod(Vec8)
		{
			PR_EXPECT(To<std::string>(Vec8<float, void>(-1.125f, 2.25f, -3.375f, 4.4f, -5.5f, 6.675f)) == "-1.125 2.25 -3.375 0  4.4 -5.5 6.675 0");
			PR_EXPECT(To<std::string>(Vec8<double, void>(-1.125, 2.25, -3.375, 4.4, -5.5, 6.675))      == "-1.125 2.25 -3.375 0  4.4 -5.5 6.675 0");
			PR_EXPECT(To<std::string>(Vec8<int, void>(12, -34, 56, -78, 90, -11))                      == "12 -34 56 0  -78 90 -11 0");
			PR_EXPECT(To<std::string>(Vec8<long long, void>(12, -34, 56, -78, 90, -11))                == "12 -34 56 0  -78 90 -11 0");

			PR_EXPECT((To<Vec8<double, void>>("-1.125 2.25 -3.375 0  4.4 -5.5 6.675 0")) == (Vec8<double, void>(-1.125, 2.25, -3.375, 0.0, 4.4, -5.5, 6.675, 0.0)));
			PR_EXPECT((To<Vec8<int, void>>("12 -34 56 0  -78 90 -11 0"))                 == (Vec8<int, void>(12, -34, 56, 0, -78, 90, -11, 0)));
			PR_EXPECT((To<Vec8<int, void>>("10 11 12 13  14 15 16 17", 16))              == (Vec8<int, void>(16, 17, 18, 19, 20, 21, 22, 23)));
		}
		PRUnitTestMethod(Mat2x2)
		{
			PR_EXPECT(To<std::string>(Mat2x2<float>(
				2.125f, -4.75f,
				-1.375f, -0.825f)) ==
				"2.125 -4.75 "
				"-1.375 -0.825");
			PR_EXPECT(To<std::string>(Mat2x2<double>(
				2.125, -4.75,
				-1.375, -0.825)) ==
				"2.125 -4.75 "
				"-1.375 -0.825");
			PR_EXPECT(To<std::string>(Mat2x2<int>(
				12, -34,
				56, -78)) ==
				"12 -34 "
				"56 -78");
			PR_EXPECT(To<std::string>(Mat2x2<long long>(
				12, -34,
				56, -78)) ==
				"12 -34 "
				"56 -78");
		}
		PRUnitTestMethod(Mat3x4)
		{
			PR_EXPECT(To<std::string>(Mat3x4<float>(
				Vec4<float>(1.2f, 2.4f, -4.8f, -8.16f),
				Vec4<float>(2.1f, 4.2f, -8.4f, -1.68f),
				Vec4<float>(1.1f, 2.2f, -3.3f, -4.44f))) ==
				"1.2 2.4 -4.8 -8.16 "
				"2.1 4.2 -8.4 -1.68 "
				"1.1 2.2 -3.3 -4.44");
			PR_EXPECT(To<std::string>(Mat3x4<double>(
				Vec4<double>(1.2, 2.4, -4.8, -8.16),
				Vec4<double>(2.1, 4.2, -8.4, -1.68),
				Vec4<double>(1.1, 2.2, -3.3, -4.44))) ==
				"1.2 2.4 -4.8 -8.16 "
				"2.1 4.2 -8.4 -1.68 "
				"1.1 2.2 -3.3 -4.44");
			PR_EXPECT(To<std::string>(Mat3x4<int>(
				Vec4<int>(1, 2, -3, -4),
				Vec4<int>(5, 6, -7, -8),
				Vec4<int>(9, 0, -1, -2))) ==
				"1 2 -3 -4 "
				"5 6 -7 -8 "
				"9 0 -1 -2");
			PR_EXPECT(To<std::string>(Mat3x4<long long>(
				Vec4<long long>(1, 2, -3, -4),
				Vec4<long long>(5, 6, -7, -8),
				Vec4<long long>(9, 0, -1, -2))) ==
				"1 2 -3 -4 "
				"5 6 -7 -8 "
				"9 0 -1 -2");
		}
		PRUnitTestMethod(Mat4x4)
		{
			PR_EXPECT(To<std::string>(Mat4x4<float>(
				Vec4<float>(1.2f, 2.4f, -4.8f, -8.16f),
				Vec4<float>(2.1f, 4.2f, -8.4f, -1.68f),
				Vec4<float>(1.1f, 2.2f, -3.3f, -4.44f),
				Vec4<float>(0.2f, 0.5f, -0.1f, -0.12f))) ==
				"1.2 2.4 -4.8 -8.16 "
				"2.1 4.2 -8.4 -1.68 "
				"1.1 2.2 -3.3 -4.44 "
				"0.2 0.5 -0.1 -0.12");
			PR_EXPECT(To<std::string>(Mat4x4<double>(
				Vec4<double>(1.2, 2.4, -4.8, -8.16),
				Vec4<double>(2.1, 4.2, -8.4, -1.68),
				Vec4<double>(1.1, 2.2, -3.3, -4.44),
				Vec4<double>(0.2, 0.5, -0.1, -0.12))) ==
				"1.2 2.4 -4.8 -8.16 "
				"2.1 4.2 -8.4 -1.68 "
				"1.1 2.2 -3.3 -4.44 "
				"0.2 0.5 -0.1 -0.12");
			PR_EXPECT(To<std::string>(Mat4x4<int>(
				Vec4<int>(1, 2, -3, -4),
				Vec4<int>(5, 6, -7, -8),
				Vec4<int>(9, 0, -1, -2),
				Vec4<int>(3, 4, -5, -6))) ==
				"1 2 -3 -4 "
				"5 6 -7 -8 "
				"9 0 -1 -2 "
				"3 4 -5 -6");
			PR_EXPECT(To<std::string>(Mat4x4<long long>(
				Vec4<long long>(1, 2, -3, -4),
				Vec4<long long>(5, 6, -7, -8),
				Vec4<long long>(9, 0, -1, -2),
				Vec4<long long>(3, 4, -5, -6))) ==
				"1 2 -3 -4 "
				"5 6 -7 -8 "
				"9 0 -1 -2 "
				"3 4 -5 -6");
		}
		PRUnitTestMethod(Mat6x8)
		{
			PR_EXPECT(To<std::string>(Mat6x8<float, void, void>(
				Vec8<float, void>(1.1f, 2.2f, -3.25f, 4.5f, 5.25f, -6.125f),
				Vec8<float, void>(1.1f, 2.2f, -3.25f, 4.5f, 5.25f, -6.125f),
				Vec8<float, void>(1.1f, 2.2f, -3.25f, 4.5f, 5.25f, -6.125f),
				Vec8<float, void>(1.1f, 2.2f, -3.25f, 4.5f, 5.25f, -6.125f),
				Vec8<float, void>(1.1f, 2.2f, -3.25f, 4.5f, 5.25f, -6.125f),
				Vec8<float, void>(1.1f, 2.2f, -3.25f, 4.5f, 5.25f, -6.125f))) ==
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0");
			PR_EXPECT(To<std::string>(Mat6x8<double, void, void>(
				Vec8<double, void>(1.1, 2.2, -3.25, 4.5, 5.25, -6.125),
				Vec8<double, void>(1.1, 2.2, -3.25, 4.5, 5.25, -6.125),
				Vec8<double, void>(1.1, 2.2, -3.25, 4.5, 5.25, -6.125),
				Vec8<double, void>(1.1, 2.2, -3.25, 4.5, 5.25, -6.125),
				Vec8<double, void>(1.1, 2.2, -3.25, 4.5, 5.25, -6.125),
				Vec8<double, void>(1.1, 2.2, -3.25, 4.5, 5.25, -6.125))) ==
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0 "
				"1.1 2.2 -3.25 0  4.5 5.25 -6.125 0");
			PR_EXPECT(To<std::string>(Mat6x8<int, void, void>(
				Vec8<int, void>(1, -2, 3, -4, 5, -6),
				Vec8<int, void>(1, -2, 3, -4, 5, -6),
				Vec8<int, void>(1, -2, 3, -4, 5, -6),
				Vec8<int, void>(1, -2, 3, -4, 5, -6),
				Vec8<int, void>(1, -2, 3, -4, 5, -6),
				Vec8<int, void>(1, -2, 3, -4, 5, -6))) ==
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0");
			PR_EXPECT(To<std::string>(Mat6x8<long long, void, void>(
				Vec8<long long, void>(1, -2, 3, -4, 5, -6),
				Vec8<long long, void>(1, -2, 3, -4, 5, -6),
				Vec8<long long, void>(1, -2, 3, -4, 5, -6),
				Vec8<long long, void>(1, -2, 3, -4, 5, -6),
				Vec8<long long, void>(1, -2, 3, -4, 5, -6),
				Vec8<long long, void>(1, -2, 3, -4, 5, -6))) ==
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0 "
				"1 -2 3 0  -4 5 -6 0");
		}
		PRUnitTestMethod(General)
		{
			PR_EXPECT(To<std::string>(v4(1, 2, 3, 4)) == "1 2 3 4");

			PR_EXPECT(To<v2>("2 3") == v2(2, 3));
			PR_EXPECT(To<v4>("1 2 3 4") == v4(1, 2, 3, 4));

			PR_EXPECT(ToBinaryStr<std::string>(uint8_t(0b11001010)) == "11001010");
		}
	};
}
#endif
