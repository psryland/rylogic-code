//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Hacks to allow hlsl code to build under C++
#pragma once
#include <concepts>
#include <type_traits>
#include "pr/maths/maths.h"
#include "pr/view3d-12/resource/image.h"
#include "pr/view3d-12/lighting/light.h"

namespace pr::hlsl
{
	#define uniform
	#define row_major
	#define line
	#define inout

	#pragma warning(disable: 4201) // nameless struct/union
	template <typename T, int N> struct Vec {};
	template <typename T> struct Vec<T, 1>
	{
		union {
			struct { T arr[1]; };
			struct { T x; };
		};
		Vec(T x) : x(x) {}
		operator T() const { return x; }
		operator Vec<float, 1>() const requires std::is_same_v<T, int>
		{
			return Vec<float, 1>(static_cast<float>(x));
		}
		template <typename U> explicit operator Vec<U, 1>() const
		{
			return Vec<U, 1>(static_cast<U>(x));
		}
		T operator[](int i) const
		{
			assert(i >= 0 && i < 1);
			return arr[i];
		}
		T& operator[](int i)
		{
			assert(i >= 0 && i < 1);
			return arr[i];
		}
	};
	template <typename T> struct Vec<T, 2>
	{
		union {
			struct { T arr[2]; };
			struct { Vec<T, 1> x, y; };
		};
		Vec(T x) : Vec(x, x) {}
		Vec(T x, T y) : x(x), y(y) {}
		operator Vec<float, 2>() const requires std::is_same_v<T, int>
		{
			return Vec<float, 2>(static_cast<float>(x), static_cast<float>(y));
		}
		template <typename U> explicit operator Vec<U, 2>() const
		{
			return Vec<U, 2>(static_cast<U>(x), static_cast<U>(y));
		}
		T operator[](int i) const
		{
			assert(i >= 0 && i < 2);
			return arr[i];
		}
		T& operator[](int i)
		{
			assert(i >= 0 && i < 2);
			return arr[i];
		}
	};
	template <typename T> struct Vec<T, 3>
	{
		union {
			struct { T arr[3]; };
			struct { Vec<T, 1> x, y, z; };
			struct { Vec<T, 2> xy; };
		};
		Vec(T x) : Vec(x, x, x) {}
		Vec(T x, T y, T z) : x(x), y(y), z(z) {}
		operator Vec<float, 3>() const requires std::is_same_v<T, int>
		{
			return Vec<float, 3>(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
		}
		template <typename U> explicit operator Vec<U, 3>() const
		{
			return Vec<U, 3>(static_cast<U>(x), static_cast<U>(y), static_cast<U>(z));
		}
		T operator[](int i) const
		{
			assert(i >= 0 && i < 3);
			return arr[i];
		}
		T& operator[](int i)
		{
			assert(i >= 0 && i < 3);
			return arr[i];
		}
	};
	template <typename T> struct Vec<T, 4>
	{
		union {
			struct { T arr[4]; };
			struct { Vec<T, 1> x, y, z, w; };
			struct { Vec<T, 2> xy, zw; };
			struct { Vec<T, 3> xyz; };
		};
		Vec(T x) : Vec(x, x, x, x) {}
		Vec(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
		Vec(Vec<T,2> xy, T z, T w) : xy(xy), z(z), w(w) {}
		Vec(Vec<T,2> xy, Vec<T,2> zw) : xy(xy), zw(zw) {}
		Vec(Vec<T,3> xyz, T w) : xyz(xyz), w(w) {}
		operator Vec<float, 4>() const requires std::is_same_v<T, int>
		{
			return Vec<float, 4>(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w));
		}
		template <typename U> explicit operator Vec<U, 4>() const
		{
			return Vec<U, 4>(static_cast<U>(x), static_cast<U>(y), static_cast<U>(z), static_cast<U>(w));
		}
		T operator[](int i) const
		{
			assert(i >= 0 && i < 4);
			return arr[i];
		}
		T& operator[](int i)
		{
			assert(i >= 0 && i < 4);
			return arr[i];
		}
	};
	#pragma warning(default: 4201)

	template <int N> using Bool = Vec<bool, N>;
	template <int N> using Int = Vec<int, N>;
	template <int N> using Float = Vec<float, N>;

	using uint     = unsigned int;
	using bool1    = Bool<1>;
	using bool2    = Bool<2>;
	using bool3    = Bool<3>;
	using bool4    = Bool<4>;
	using int1     = Int<1>;
	using int2     = Int<2>;
	using int3     = Int<3>;
	using int4     = Int<4>;
	using float1   = Float<1>;
	using float2   = Float<2>;
	using float3   = Float<3>;
	using float4   = Float<4>;
	using float4x4 = Vec<float4, 4>;

	// Operators
	template <typename T, int N> Bool<N> operator == (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Bool<N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] == rhs[i];
		return result;
	}
	template <typename T, int N> Bool<N> operator != (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		return !(lhs == rhs);
	}
	template <typename T, int N> Bool<N> operator < (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Bool<N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] < rhs[i];
		return result;
	}
	template <typename T, int N> Bool<N> operator > (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Bool<N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] > rhs[i];
		return result;
	}
	template <typename T, int N> Bool<N> operator <= (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		return !(lhs > rhs);
	}
	template <typename T, int N> Bool<N> operator >= (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		return !(lhs < rhs);
	}
	template <typename T, int N> Vec<T, N> operator + (Vec<T, N> const& lhs)
	{
		return lhs;
	}
	template <typename T, int N> Vec<T, N> operator - (Vec<T, N> const& lhs)
	{
		Vec<T, N> result;
		for (int i = 0; i != N; ++i) result[i] = -lhs[i];
		return result;
	}
	template <typename T, int N> Vec<T, N> operator + (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Vec<T, N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] + rhs[i];
		return result;
	}
	template <typename T, int N> Vec<T, N> operator - (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Vec<T, N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] - rhs[i];
		return result;
	}
	template <typename T, int N> Vec<T, N> operator * (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Vec<T, N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] * rhs[i];
		return result;
	}
	template <typename T, int N> Vec<T, N> operator / (Vec<T, N> const& lhs, Vec<T, N> const& rhs)
	{
		Vec<T, N> result;
		for (int i = 0; i != N; ++i) result[i] = lhs[i] / rhs[i];
		return result;
	}
	template <typename T, int N> Vec<T, N> operator * (Vec<T, N> const& lhs, T rhs)
	{
		return lhs * Vec<T, N>(rhs);
	}
	template <typename T, int N> Vec<T, N> operator * (T lhs, Vec<T, N> const& rhs)
	{
		return Vec<T, N>(lhs) * rhs;
	}
	template <typename T, int N> Vec<T, N> operator / (Vec<T, N> const& lhs, T rhs)
	{
		return lhs / Vec<T, N>(rhs);
	}
	template <typename T, int N> Vec<T, N> operator / (T lhs, Vec<T, N> const& rhs)
	{
		return Vec<T, N>(lhs) / rhs;
	}

	template <typename T, int N> Float<N> operator + (Vec<T, N> const& lhs, float rhs)
	{
		return static_cast<Vec<float, N>>(lhs) + Float<N>(rhs);
	}
	template <typename T, int N> Float<N> operator + (float lhs, Vec<T, N> const& rhs)
	{
		return Float<N>(lhs) + static_cast<Vec<float, N>>(rhs);
	}
	template <typename T, int N> Float<N> operator - (Vec<T, N> const& lhs, float rhs)
	{
		return static_cast<Vec<float, N>>(lhs) - Float<N>(rhs);
	}
	template <typename T, int N> Float<N> operator - (float lhs, Vec<T, N> const& rhs)
	{
		return Float<N>(lhs) - static_cast<Vec<float, N>>(rhs);
	}
	template <typename T, int N> Float<N> operator * (Vec<T, N> const& lhs, float rhs)
	{
		return static_cast<Vec<float, N>>(lhs) * Float<N>(rhs);
	}
	template <typename T, int N> Float<N> operator * (float lhs, Vec<T, N> const& rhs)
	{
		return Float<N>(lhs) * static_cast<Vec<float, N>>(rhs);
	}
	template <typename T, int N> Float<N> operator / (Vec<T, N> const& lhs, float rhs)
	{
		return static_cast<Vec<float, N>>(lhs) / Float<N>(rhs);
	}
	template <typename T, int N> Float<N> operator / (float lhs, Vec<T, N> const& rhs)
	{
		return Float<N>(lhs) / static_cast<Vec<float, N>>(rhs);
	}
	template <typename T, typename U, int N> Float<N> operator * (Vec<T, N> const& lhs, Vec<U,N> const& rhs) requires std::is_same_v<T, float> || std::is_same_v<U, float>
	{
		return static_cast<Vec<float, N>>(lhs) * static_cast<Vec<float, N>>(rhs);
	}

	template <int N> Bool<N> operator < (Float<N> const& lhs, float rhs)
	{
		return lhs < Float<N>(rhs);
	}
	template <int N> Bool<N> operator > (Float<N> const& lhs, float rhs)
	{
		return lhs > Float<N>(rhs);
	}
	template <int N> Bool<N> operator <= (Float<N> const& lhs, float rhs)
	{
		return !(lhs > Float<N>(rhs));
	}
	template <int N> Bool<N> operator >= (Float<N> const& lhs, float rhs)
	{
		return !(lhs < Float<N>(rhs));
	}

	// Shader intrinsic functions
	bool clip(float x)
	{
		return x < 0.0f;
	}
	template <int N> Int<N> step(Float<N> const& lo, Float<N> const& hi)
	{
		Int<N> result;
		for (int i = 0; i != N; ++i) result[i] = hi[i] >= lo[i] ? 1 : 0;
		return result;
	}
	template <int N> Int<N> sign(Float<N> v)
	{
		Int<N> result;
		for (int i = 0; i != N; ++i) result[i] = v[i] < 0.0f ? -1 : v.acc[i] > 0.0f ? +1 : 0;
		return result;
	}
	template <int N> Float<N> smoothstep(Float<N> const& lo, Float<N> const& hi, Float<N> const& v)
	{
		if (lo == hi)
			return lo;
		
		Float<N> result;
		for (int i = 0; i != N; ++i)
		{
			auto t = (v[i] - lo[i]) / (hi[i] - lo[i]);
			result[i] = t <= 0 ? lo[i] : t >= 1 ? hi[i] : t * t * (3 - 2 * t);
		}
		return result;
	}
	template <int N> Float<N> saturate(Float<N> const& v)
	{
		Float<N> result;
		for (int i = 0; i != N; ++i) result[i] = v[i] <= 0 ? 0 : v[i] >= 1 ? 1 : v[i];
		return result;
	}
	template <int N> Float<N> min(Float<N> const& a, Float<N> const& b)
	{
		Float<N> result;
		for (int i = 0; i != N; ++i) result[i] = std::min(a[i], b[i]);
		return result;
	}
	template <int N> Float<N> max(Float<N> const& a, Float<N> const& b)
	{
		Float<N> result;
		for (int i = 0; i != N; ++i) result[i] = std::max(a[i], b[i]);
		return result;
	}
	template <int N> float dot(Float<N> const& a, Float<N> const& b)
	{
		float result = 0;
		for (int i = 0; i != N; ++i) result += a[i] * b[i];
		return result;
	}
	template <int N> float length_sq(Float<N> const& v)
	{
		return dot(v, v);
	}
	template <int N> float length(Float<N> const& v)
	{
		return sqrt(length_sq(v));
	}
	template <int N> Float<N> normalize(Float<N> const& v)
	{
		return v / length(v);
	}
	template <int N> Float<N> lerp(Float<N> const& a, Float<N> const& b, float t)
	{
		return (1-t)*a + t*b;
	}
	//float4 mul(float4 const& v, float4x4 const& m)
	//{
	//	m4x4(m.x, m.y, m.z, m.w);
	//	return m * static_cast<v4>(v);
	//}

	struct SamplerState
	{
	};
	template <typename Format> struct Texture2D
	{
		rdr12::Image m_img;
		virtual Format Sample(SamplerState const&, float2 const& uv)
		{
			int u = int(uv.x * m_img.m_dim.x);
			int v = int(uv.y * m_img.m_dim.y);
			return ReadPixel(u, v);
		}
		virtual Format ReadPixel(int u, int v)
		{
			auto* px = static_cast<Format const*>(m_img.m_pixels);
			if (px == nullptr) return Format();
			return px[v * m_img.m_pitch.x + u];
		}
	};
	template <typename T> struct TriangleStream
	{
		virtual void Append(T const&) {}
		virtual void RestartStrip() {}
	};

	struct SLight
	{
		int4   m_info;         // x = light type (0:ambient, 1:directional, 2:point, 3:spot), yzw = unused
		float4 m_ws_direction; // The direction of the global light source
		float4 m_ws_position;  // The position of the global light source
		float4 m_ambient;      // The colour of the ambient light
		float4 m_colour;       // The colour of the directional light
		float4 m_specular;     // The colour of the specular light. alpha channel is specular power
		float4 m_range;        // x = inner angle, y = outer angle, z = range, w = falloff

		SLight(rdr12::Light const& light)
			:m_info((int)light.m_type,0,0,0)
			,m_ws_direction(To<float4>(light.m_direction))
			,m_ws_position(To<float4>(light.m_position))
			,m_ambient(To<float4>(To<Colour>(light.m_ambient).rgba))
			,m_colour(To<float4>(To<Colour>(light.m_diffuse).rgba))
			,m_specular(To<float4>(v4(To<Colour>(light.m_specular).rgb, light.m_specular_power)))
			,m_range(light.m_inner_angle, light.m_outer_angle, light.m_range, light.m_falloff)
		{}
	};

	#undef uniform
	#undef row_major
	#undef line
	//#undef inout
}
namespace pr
{
	template <> struct Convert<v4, hlsl::float4>
	{
		static v4 Func(hlsl::float4 const& v)
		{
			return v4(v.x, v.y, v.z, v.w);
		}
	};
	template <> struct Convert<hlsl::float4, v4>
	{
		static hlsl::float4 Func(v4 v)
		{
			return hlsl::float4(v.x, v.y, v.z, v.w);
		}
	};
}