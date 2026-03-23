//*********************************************
// Shader Interop
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Hacks to allow hlsl code to build under C++
// Usage:
/*
	// test.cpp
	using namespace pr::hlsl;
	#include "src/example.hlsl"
	void EntryPoint()
	{
		// Do buffer setup here
		// Change the entry point name if needed
		GpuEmulator emu(CSFaceNormal, NumThreads);
		emu.Dispatch({ 1, 1, 1 });
	}
*/
#pragma once
#include <concepts>
#include <type_traits>
#include <functional>
#include <execution>
#include <vector>
#include <ranges>
#include <cstdint>
#include <cmath>

#include "pr/math/math.h"

namespace pr::hlsl
{
	using uint     = uint32_t;
	using bool1    = bool;
	using bool2    = math::Vec2<bool>;
	using bool3    = math::Vec3<bool>;
	using bool4    = math::Vec4<bool>;
	using int1     = int;
	using int2     = math::Vec2<int>;
	using int3     = math::Vec3<int>;
	using int4     = math::Vec4<int>;
	using uint1    = uint;
	using uint2    = math::Vec2<uint>;
	using uint3    = math::Vec3<uint>;
	using uint4    = math::Vec4<uint>;
	using float1   = float;
	using float2   = math::Vec2<float>;
	using float3   = math::Vec3<float>;
	using float4   = math::Vec4<float>;
	using float4x4 = math::Mat4x4<float>;
	using voidp    = void const*;

	// Shader structure types
	#pragma region Shader structures
	template <typename T>
	struct StructuredBuffer
	{
		std::vector<T> m_data;
		T operator[] (int index) const
		{
			return m_data[index];
		}
		virtual T Read(int index)
		{
			return m_data[index];
		}
	};

	template <typename T>
	struct RWStructuredBuffer
	{
		std::vector<T> m_data;
		T& operator[] (int index)
		{
			return m_data[index];
		}
	};

	struct SamplerState
	{
	};

	template <typename Format>
	struct Texture2D
	{
		//rdr12::Image m_img;
		std::vector<Format> m_img;
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

	template <typename T>
	struct TriangleStream
	{
		virtual void Append(T const&) {}
		virtual void RestartStrip() {}
	};
	#pragma endregion

	// Shader intrinsic functions
	#pragma region Intrinsics
	// --- clip ---
	constexpr bool clip(float x)
	{
		return x < 0.0f;
	}

	// --- abs ---
	inline float abs(float x)
	{
		return math::Abs(x);
	}
	inline int abs(int x)
	{
		return math::Abs(x);
	}
	inline float2 abs(float2 v)
	{
		return math::Abs(v);
	}
	inline float3 abs(float3 v)
	{
		return math::Abs(v);
	}
	inline float4 abs(float4 v)
	{
		return math::Abs(v);
	}
	inline int2 abs(int2 v)
	{
		return math::Abs(v);
	}
	inline int3 abs(int3 v)
	{
		return math::Abs(v);
	}
	inline int4 abs(int4 v)
	{
		return math::Abs(v);
	}

	// --- sign ---
	// Returns -1, 0, or +1 as a float (matching HLSL usage in multiplication contexts)
	inline float sign(float x)
	{
		return x > 0.0f ? 1.0f : x < 0.0f ? -1.0f : 0.0f;
	}
	inline int sign(int x)
	{
		return x > 0 ? 1 : x < 0 ? -1 : 0;
	}
	inline float2 sign(float2 v)
	{
		return float2(sign(v.x), sign(v.y));
	}
	inline float3 sign(float3 v)
	{
		return float3(sign(v.x), sign(v.y), sign(v.z));
	}
	inline float4 sign(float4 v)
	{
		return float4(sign(v.x), sign(v.y), sign(v.z), sign(v.w));
	}
	inline int2 sign(int2 v)
	{
		return int2(sign(v.x), sign(v.y));
	}
	inline int3 sign(int3 v)
	{
		return int3(sign(v.x), sign(v.y), sign(v.z));
	}
	inline int4 sign(int4 v)
	{
		return int4(sign(v.x), sign(v.y), sign(v.z), sign(v.w));
	}

	// --- floor ---
	inline float floor(float x)
	{
		return std::floor(x);
	}
	inline float2 floor(float2 v)
	{
		return float2(std::floor(v.x), std::floor(v.y));
	}
	inline float3 floor(float3 v)
	{
		return float3(std::floor(v.x), std::floor(v.y), std::floor(v.z));
	}
	inline float4 floor(float4 v)
	{
		return float4(std::floor(v.x), std::floor(v.y), std::floor(v.z), std::floor(v.w));
	}

	// --- ceil ---
	inline float ceil(float x)
	{
		return std::ceil(x);
	}
	inline float2 ceil(float2 v)
	{
		return float2(std::ceil(v.x), std::ceil(v.y));
	}
	inline float3 ceil(float3 v)
	{
		return float3(std::ceil(v.x), std::ceil(v.y), std::ceil(v.z));
	}
	inline float4 ceil(float4 v)
	{
		return float4(std::ceil(v.x), std::ceil(v.y), std::ceil(v.z), std::ceil(v.w));
	}

	// --- round ---
	inline float round(float x)
	{
		return std::round(x);
	}
	inline float2 round(float2 v)
	{
		return float2(std::round(v.x), std::round(v.y));
	}
	inline float3 round(float3 v)
	{
		return float3(std::round(v.x), std::round(v.y), std::round(v.z));
	}
	inline float4 round(float4 v)
	{
		return float4(std::round(v.x), std::round(v.y), std::round(v.z), std::round(v.w));
	}

	// --- frac ---
	inline float frac(float x)
	{
		return x - std::floor(x);
	}
	inline float2 frac(float2 v)
	{
		return float2(frac(v.x), frac(v.y));
	}
	inline float3 frac(float3 v)
	{
		return float3(frac(v.x), frac(v.y), frac(v.z));
	}
	inline float4 frac(float4 v)
	{
		return float4(frac(v.x), frac(v.y), frac(v.z), frac(v.w));
	}

	// --- fmod ---
	inline float fmod(float x, float y)
	{
		return std::fmod(x, y);
	}
	inline float2 fmod(float2 x, float2 y)
	{
		return float2(std::fmod(x.x, y.x), std::fmod(x.y, y.y));
	}
	inline float3 fmod(float3 x, float3 y)
	{
		return float3(std::fmod(x.x, y.x), std::fmod(x.y, y.y), std::fmod(x.z, y.z));
	}
	inline float4 fmod(float4 x, float4 y)
	{
		return float4(std::fmod(x.x, y.x), std::fmod(x.y, y.y), std::fmod(x.z, y.z), std::fmod(x.w, y.w));
	}

	// --- sqrt ---
	inline float sqrt(float x)
	{
		return std::sqrt(x);
	}
	inline float2 sqrt(float2 v)
	{
		return float2(std::sqrt(v.x), std::sqrt(v.y));
	}
	inline float3 sqrt(float3 v)
	{
		return float3(std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z));
	}
	inline float4 sqrt(float4 v)
	{
		return float4(std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z), std::sqrt(v.w));
	}

	// --- rsqrt ---
	inline float rsqrt(float x)
	{
		return 1.0f / std::sqrt(x);
	}
	inline float2 rsqrt(float2 v)
	{
		return float2(rsqrt(v.x), rsqrt(v.y));
	}
	inline float3 rsqrt(float3 v)
	{
		return float3(rsqrt(v.x), rsqrt(v.y), rsqrt(v.z));
	}
	inline float4 rsqrt(float4 v)
	{
		return float4(rsqrt(v.x), rsqrt(v.y), rsqrt(v.z), rsqrt(v.w));
	}

	// --- pow ---
	inline float pow(float x, float y)
	{
		return std::pow(x, y);
	}
	inline float2 pow(float2 x, float2 y)
	{
		return float2(std::pow(x.x, y.x), std::pow(x.y, y.y));
	}
	inline float3 pow(float3 x, float3 y)
	{
		return float3(std::pow(x.x, y.x), std::pow(x.y, y.y), std::pow(x.z, y.z));
	}
	inline float4 pow(float4 x, float4 y)
	{
		return float4(std::pow(x.x, y.x), std::pow(x.y, y.y), std::pow(x.z, y.z), std::pow(x.w, y.w));
	}

	// --- exp ---
	inline float exp(float x)
	{
		return std::exp(x);
	}
	inline float2 exp(float2 v)
	{
		return float2(std::exp(v.x), std::exp(v.y));
	}
	inline float3 exp(float3 v)
	{
		return float3(std::exp(v.x), std::exp(v.y), std::exp(v.z));
	}
	inline float4 exp(float4 v)
	{
		return float4(std::exp(v.x), std::exp(v.y), std::exp(v.z), std::exp(v.w));
	}

	// --- exp2 ---
	inline float exp2(float x)
	{
		return std::exp2(x);
	}
	inline float2 exp2(float2 v)
	{
		return float2(std::exp2(v.x), std::exp2(v.y));
	}
	inline float3 exp2(float3 v)
	{
		return float3(std::exp2(v.x), std::exp2(v.y), std::exp2(v.z));
	}
	inline float4 exp2(float4 v)
	{
		return float4(std::exp2(v.x), std::exp2(v.y), std::exp2(v.z), std::exp2(v.w));
	}

	// --- log ---
	inline float log(float x)
	{
		return std::log(x);
	}
	inline float2 log(float2 v)
	{
		return float2(std::log(v.x), std::log(v.y));
	}
	inline float3 log(float3 v)
	{
		return float3(std::log(v.x), std::log(v.y), std::log(v.z));
	}
	inline float4 log(float4 v)
	{
		return float4(std::log(v.x), std::log(v.y), std::log(v.z), std::log(v.w));
	}

	// --- log2 ---
	inline float log2(float x)
	{
		return std::log2(x);
	}
	inline float2 log2(float2 v)
	{
		return float2(std::log2(v.x), std::log2(v.y));
	}
	inline float3 log2(float3 v)
	{
		return float3(std::log2(v.x), std::log2(v.y), std::log2(v.z));
	}
	inline float4 log2(float4 v)
	{
		return float4(std::log2(v.x), std::log2(v.y), std::log2(v.z), std::log2(v.w));
	}

	// --- log10 ---
	inline float log10(float x)
	{
		return std::log10(x);
	}
	inline float2 log10(float2 v)
	{
		return float2(std::log10(v.x), std::log10(v.y));
	}
	inline float3 log10(float3 v)
	{
		return float3(std::log10(v.x), std::log10(v.y), std::log10(v.z));
	}
	inline float4 log10(float4 v)
	{
		return float4(std::log10(v.x), std::log10(v.y), std::log10(v.z), std::log10(v.w));
	}

	// --- sin ---
	inline float sin(float x)
	{
		return std::sin(x);
	}
	inline float2 sin(float2 v)
	{
		return float2(std::sin(v.x), std::sin(v.y));
	}
	inline float3 sin(float3 v)
	{
		return float3(std::sin(v.x), std::sin(v.y), std::sin(v.z));
	}
	inline float4 sin(float4 v)
	{
		return float4(std::sin(v.x), std::sin(v.y), std::sin(v.z), std::sin(v.w));
	}

	// --- cos ---
	inline float cos(float x)
	{
		return std::cos(x);
	}
	inline float2 cos(float2 v)
	{
		return float2(std::cos(v.x), std::cos(v.y));
	}
	inline float3 cos(float3 v)
	{
		return float3(std::cos(v.x), std::cos(v.y), std::cos(v.z));
	}
	inline float4 cos(float4 v)
	{
		return float4(std::cos(v.x), std::cos(v.y), std::cos(v.z), std::cos(v.w));
	}

	// --- sincos ---
	inline void sincos(float x, float& s, float& c)
	{
		s = std::sin(x);
		c = std::cos(x);
	}

	// --- tan ---
	inline float tan(float x)
	{
		return std::tan(x);
	}
	inline float2 tan(float2 v)
	{
		return float2(std::tan(v.x), std::tan(v.y));
	}
	inline float3 tan(float3 v)
	{
		return float3(std::tan(v.x), std::tan(v.y), std::tan(v.z));
	}
	inline float4 tan(float4 v)
	{
		return float4(std::tan(v.x), std::tan(v.y), std::tan(v.z), std::tan(v.w));
	}

	// --- asin ---
	inline float asin(float x)
	{
		return std::asin(x);
	}
	inline float2 asin(float2 v)
	{
		return float2(std::asin(v.x), std::asin(v.y));
	}
	inline float3 asin(float3 v)
	{
		return float3(std::asin(v.x), std::asin(v.y), std::asin(v.z));
	}
	inline float4 asin(float4 v)
	{
		return float4(std::asin(v.x), std::asin(v.y), std::asin(v.z), std::asin(v.w));
	}

	// --- acos ---
	inline float acos(float x)
	{
		return std::acos(x);
	}
	inline float2 acos(float2 v)
	{
		return float2(std::acos(v.x), std::acos(v.y));
	}
	inline float3 acos(float3 v)
	{
		return float3(std::acos(v.x), std::acos(v.y), std::acos(v.z));
	}
	inline float4 acos(float4 v)
	{
		return float4(std::acos(v.x), std::acos(v.y), std::acos(v.z), std::acos(v.w));
	}

	// --- atan ---
	inline float atan(float x)
	{
		return std::atan(x);
	}
	inline float2 atan(float2 v)
	{
		return float2(std::atan(v.x), std::atan(v.y));
	}
	inline float3 atan(float3 v)
	{
		return float3(std::atan(v.x), std::atan(v.y), std::atan(v.z));
	}
	inline float4 atan(float4 v)
	{
		return float4(std::atan(v.x), std::atan(v.y), std::atan(v.z), std::atan(v.w));
	}

	// --- atan2 ---
	inline float atan2(float y, float x)
	{
		return std::atan2(y, x);
	}
	inline float2 atan2(float2 y, float2 x)
	{
		return float2(std::atan2(y.x, x.x), std::atan2(y.y, x.y));
	}
	inline float3 atan2(float3 y, float3 x)
	{
		return float3(std::atan2(y.x, x.x), std::atan2(y.y, x.y), std::atan2(y.z, x.z));
	}
	inline float4 atan2(float4 y, float4 x)
	{
		return float4(std::atan2(y.x, x.x), std::atan2(y.y, x.y), std::atan2(y.z, x.z), std::atan2(y.w, x.w));
	}

	// --- min ---
	inline float min(float a, float b)
	{
		return math::Min(a, b);
	}
	inline int min(int a, int b)
	{
		return math::Min(a, b);
	}
	inline uint min(uint a, uint b)
	{
		return math::Min(a, b);
	}
	inline float2 min(float2 a, float2 b)
	{
		return math::Min(a, b);
	}
	inline float3 min(float3 a, float3 b)
	{
		return math::Min(a, b);
	}
	inline float4 min(float4 a, float4 b)
	{
		return math::Min(a, b);
	}
	inline int2 min(int2 a, int2 b)
	{
		return math::Min(a, b);
	}
	inline int3 min(int3 a, int3 b)
	{
		return math::Min(a, b);
	}
	inline int4 min(int4 a, int4 b)
	{
		return math::Min(a, b);
	}
	inline uint2 min(uint2 a, uint2 b)
	{
		return math::Min(a, b);
	}
	inline uint3 min(uint3 a, uint3 b)
	{
		return math::Min(a, b);
	}
	inline uint4 min(uint4 a, uint4 b)
	{
		return math::Min(a, b);
	}

	// --- max ---
	inline float max(float a, float b)
	{
		return math::Max(a, b);
	}
	inline int max(int a, int b)
	{
		return math::Max(a, b);
	}
	inline uint max(uint a, uint b)
	{
		return math::Max(a, b);
	}
	inline float2 max(float2 a, float2 b)
	{
		return math::Max(a, b);
	}
	inline float3 max(float3 a, float3 b)
	{
		return math::Max(a, b);
	}
	inline float4 max(float4 a, float4 b)
	{
		return math::Max(a, b);
	}
	inline int2 max(int2 a, int2 b)
	{
		return math::Max(a, b);
	}
	inline int3 max(int3 a, int3 b)
	{
		return math::Max(a, b);
	}
	inline int4 max(int4 a, int4 b)
	{
		return math::Max(a, b);
	}
	inline uint2 max(uint2 a, uint2 b)
	{
		return math::Max(a, b);
	}
	inline uint3 max(uint3 a, uint3 b)
	{
		return math::Max(a, b);
	}
	inline uint4 max(uint4 a, uint4 b)
	{
		return math::Max(a, b);
	}

	// --- clamp ---
	inline float clamp(float x, float lo, float hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline int clamp(int x, int lo, int hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline uint clamp(uint x, uint lo, uint hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline float2 clamp(float2 x, float2 lo, float2 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline float3 clamp(float3 x, float3 lo, float3 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline float4 clamp(float4 x, float4 lo, float4 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline int2 clamp(int2 x, int2 lo, int2 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline int3 clamp(int3 x, int3 lo, int3 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline int4 clamp(int4 x, int4 lo, int4 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline uint2 clamp(uint2 x, uint2 lo, uint2 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline uint3 clamp(uint3 x, uint3 lo, uint3 hi)
	{
		return math::Clamp(x, lo, hi);
	}
	inline uint4 clamp(uint4 x, uint4 lo, uint4 hi)
	{
		return math::Clamp(x, lo, hi);
	}

	// --- saturate ---
	inline float saturate(float x)
	{
		return math::Clamp(x, 0.0f, 1.0f);
	}
	inline float2 saturate(float2 v)
	{
		return math::Clamp(v, float2(0.0f), float2(1.0f));
	}
	inline float3 saturate(float3 v)
	{
		return math::Clamp(v, float3(0.0f), float3(1.0f));
	}
	inline float4 saturate(float4 v)
	{
		return math::Clamp(v, float4(0.0f), float4(1.0f));
	}

	// --- lerp ---
	inline float lerp(float a, float b, float t)
	{
		return math::Lerp(a, b, t);
	}
	inline float2 lerp(float2 a, float2 b, float t)
	{
		return math::Lerp(a, b, t);
	}
	inline float3 lerp(float3 a, float3 b, float t)
	{
		return math::Lerp(a, b, t);
	}
	inline float4 lerp(float4 a, float4 b, float t)
	{
		return math::Lerp(a, b, t);
	}

	// --- step ---
	// Returns 0 if x < edge, 1 otherwise
	inline float step(float edge, float x)
	{
		return x >= edge ? 1.0f : 0.0f;
	}
	inline float2 step(float2 edge, float2 x)
	{
		return float2(step(edge.x, x.x), step(edge.y, x.y));
	}
	inline float3 step(float3 edge, float3 x)
	{
		return float3(step(edge.x, x.x), step(edge.y, x.y), step(edge.z, x.z));
	}
	inline float4 step(float4 edge, float4 x)
	{
		return float4(step(edge.x, x.x), step(edge.y, x.y), step(edge.z, x.z), step(edge.w, x.w));
	}

	// --- smoothstep ---
	inline float smoothstep(float lo, float hi, float x)
	{
		if (lo == hi)
			return lo;
		auto t = saturate((x - lo) / (hi - lo));
		return t * t * (3.0f - 2.0f * t);
	}
	inline float2 smoothstep(float2 lo, float2 hi, float2 x)
	{
		return float2(smoothstep(lo.x, hi.x, x.x), smoothstep(lo.y, hi.y, x.y));
	}
	inline float3 smoothstep(float3 lo, float3 hi, float3 x)
	{
		return float3(smoothstep(lo.x, hi.x, x.x), smoothstep(lo.y, hi.y, x.y), smoothstep(lo.z, hi.z, x.z));
	}
	inline float4 smoothstep(float4 lo, float4 hi, float4 x)
	{
		return float4(smoothstep(lo.x, hi.x, x.x), smoothstep(lo.y, hi.y, x.y), smoothstep(lo.z, hi.z, x.z), smoothstep(lo.w, hi.w, x.w));
	}

	// --- mad ---
	// Multiply-add: a * b + c
	inline float mad(float a, float b, float c)
	{
		return a * b + c;
	}
	inline float2 mad(float2 a, float2 b, float2 c)
	{
		return a * b + c;
	}
	inline float3 mad(float3 a, float3 b, float3 c)
	{
		return a * b + c;
	}
	inline float4 mad(float4 a, float4 b, float4 c)
	{
		return a * b + c;
	}
	inline int mad(int a, int b, int c)
	{
		return a * b + c;
	}
	inline int2 mad(int2 a, int2 b, int2 c)
	{
		return a * b + c;
	}
	inline int3 mad(int3 a, int3 b, int3 c)
	{
		return a * b + c;
	}
	inline int4 mad(int4 a, int4 b, int4 c)
	{
		return a * b + c;
	}
	inline uint mad(uint a, uint b, uint c)
	{
		return a * b + c;
	}
	inline uint2 mad(uint2 a, uint2 b, uint2 c)
	{
		return a * b + c;
	}
	inline uint3 mad(uint3 a, uint3 b, uint3 c)
	{
		return a * b + c;
	}
	inline uint4 mad(uint4 a, uint4 b, uint4 c)
	{
		return a * b + c;
	}

	// --- rcp ---
	inline float rcp(float x)
	{
		return 1.0f / x;
	}
	inline float2 rcp(float2 v)
	{
		return float2(1.0f) / v;
	}
	inline float3 rcp(float3 v)
	{
		return float3(1.0f) / v;
	}
	inline float4 rcp(float4 v)
	{
		return float4(1.0f) / v;
	}

	// --- isnan / isinf / isfinite ---
	inline bool isnan(float x)
	{
		return std::isnan(x);
	}
	inline bool isinf(float x)
	{
		return std::isinf(x);
	}
	inline bool isfinite(float x)
	{
		return std::isfinite(x);
	}

	// --- dot ---
	inline float dot(float2 a, float2 b)
	{
		return math::Dot(a, b);
	}
	inline float dot(float3 a, float3 b)
	{
		return math::Dot(a, b);
	}
	inline float dot(float4 a, float4 b)
	{
		return math::Dot(a, b);
	}

	// --- length_sq ---
	inline float length_sq(float2 v)
	{
		return math::LengthSq(v);
	}
	inline float length_sq(float3 v)
	{
		return math::LengthSq(v);
	}
	inline float length_sq(float4 v)
	{
		return math::LengthSq(v);
	}

	// --- length ---
	inline float length(float2 v)
	{
		return math::Length(v);
	}
	inline float length(float3 v)
	{
		return math::Length(v);
	}
	inline float length(float4 v)
	{
		return math::Length(v);
	}

	// --- distance ---
	inline float distance(float2 a, float2 b)
	{
		return math::Length(a - b);
	}
	inline float distance(float3 a, float3 b)
	{
		return math::Length(a - b);
	}
	inline float distance(float4 a, float4 b)
	{
		return math::Length(a - b);
	}

	// --- normalize ---
	inline float2 normalize(float2 v)
	{
		return math::Normalise(v);
	}
	inline float3 normalize(float3 v)
	{
		return math::Normalise(v);
	}
	inline float4 normalize(float4 v)
	{
		return math::Normalise(v);
	}

	// --- cross ---
	inline float3 cross(float3 a, float3 b)
	{
		return math::Cross(a, b);
	}
	inline float4 cross(float4 a, float4 b)
	{
		return math::Cross(a, b);
	}

	// --- reflect ---
	// Reflects the incident vector v off a surface with normal n
	inline float2 reflect(float2 v, float2 n)
	{
		return v - n * (2.0f * dot(v, n));
	}
	inline float3 reflect(float3 v, float3 n)
	{
		return v - n * (2.0f * dot(v, n));
	}
	inline float4 reflect(float4 v, float4 n)
	{
		return v - n * (2.0f * dot(v, n));
	}

	// --- refract ---
	// Refracts the incident vector v through a surface with normal n and index of refraction eta
	inline float2 refract(float2 v, float2 n, float eta)
	{
		auto d = dot(v, n);
		auto k = 1.0f - eta * eta * (1.0f - d * d);
		return k < 0.0f ? float2(0.0f) : v * eta - n * (eta * d + sqrt(k));
	}
	inline float3 refract(float3 v, float3 n, float eta)
	{
		auto d = dot(v, n);
		auto k = 1.0f - eta * eta * (1.0f - d * d);
		return k < 0.0f ? float3(0.0f) : v * eta - n * (eta * d + sqrt(k));
	}
	inline float4 refract(float4 v, float4 n, float eta)
	{
		auto d = dot(v, n);
		auto k = 1.0f - eta * eta * (1.0f - d * d);
		return k < 0.0f ? float4(0.0f) : v * eta - n * (eta * d + sqrt(k));
	}

	// --- any ---
	// Returns true if any component is non-zero
	inline bool any(bool2 v)
	{
		return v.x || v.y;
	}
	inline bool any(bool3 v)
	{
		return v.x || v.y || v.z;
	}
	inline bool any(bool4 v)
	{
		return v.x || v.y || v.z || v.w;
	}
	inline bool any(float2 v)
	{
		return v.x != 0.0f || v.y != 0.0f;
	}
	inline bool any(float3 v)
	{
		return v.x != 0.0f || v.y != 0.0f || v.z != 0.0f;
	}
	inline bool any(float4 v)
	{
		return v.x != 0.0f || v.y != 0.0f || v.z != 0.0f || v.w != 0.0f;
	}
	inline bool any(int2 v)
	{
		return v.x != 0 || v.y != 0;
	}
	inline bool any(int3 v)
	{
		return v.x != 0 || v.y != 0 || v.z != 0;
	}
	inline bool any(int4 v)
	{
		return v.x != 0 || v.y != 0 || v.z != 0 || v.w != 0;
	}
	inline bool any(uint2 v)
	{
		return v.x != 0 || v.y != 0;
	}
	inline bool any(uint3 v)
	{
		return v.x != 0 || v.y != 0 || v.z != 0;
	}
	inline bool any(uint4 v)
	{
		return v.x != 0 || v.y != 0 || v.z != 0 || v.w != 0;
	}

	// --- all ---
	// Returns true if all components are non-zero
	inline bool all(bool2 v)
	{
		return v.x && v.y;
	}
	inline bool all(bool3 v)
	{
		return v.x && v.y && v.z;
	}
	inline bool all(bool4 v)
	{
		return v.x && v.y && v.z && v.w;
	}
	inline bool all(float2 v)
	{
		return v.x != 0.0f && v.y != 0.0f;
	}
	inline bool all(float3 v)
	{
		return v.x != 0.0f && v.y != 0.0f && v.z != 0.0f;
	}
	inline bool all(float4 v)
	{
		return v.x != 0.0f && v.y != 0.0f && v.z != 0.0f && v.w != 0.0f;
	}
	inline bool all(int2 v)
	{
		return v.x != 0 && v.y != 0;
	}
	inline bool all(int3 v)
	{
		return v.x != 0 && v.y != 0 && v.z != 0;
	}
	inline bool all(int4 v)
	{
		return v.x != 0 && v.y != 0 && v.z != 0 && v.w != 0;
	}
	inline bool all(uint2 v)
	{
		return v.x != 0 && v.y != 0;
	}
	inline bool all(uint3 v)
	{
		return v.x != 0 && v.y != 0 && v.z != 0;
	}
	inline bool all(uint4 v)
	{
		return v.x != 0 && v.y != 0 && v.z != 0 && v.w != 0;
	}

	// --- mul ---
	// HLSL mul(vector, matrix) = row-vector × matrix
	inline float4 mul(float4 v, float4x4 const& m)
	{
		return float4(
			dot(v, float4(m.x.x, m.y.x, m.z.x, m.w.x)),
			dot(v, float4(m.x.y, m.y.y, m.z.y, m.w.y)),
			dot(v, float4(m.x.z, m.y.z, m.z.z, m.w.z)),
			dot(v, float4(m.x.w, m.y.w, m.z.w, m.w.w)));
	}

	// HLSL mul(matrix, vector) = matrix × column-vector
	inline float4 mul(float4x4 const& m, float4 v)
	{
		return float4(
			dot(float4(m.x.x, m.x.y, m.x.z, m.x.w), v),
			dot(float4(m.y.x, m.y.y, m.y.z, m.y.w), v),
			dot(float4(m.z.x, m.z.y, m.z.z, m.z.w), v),
			dot(float4(m.w.x, m.w.y, m.w.z, m.w.w), v));
	}

	// HLSL mul(matrix, matrix)
	inline float4x4 mul(float4x4 const& a, float4x4 const& b)
	{
		return a * b;
	}

	// --- transpose ---
	inline float4x4 transpose(float4x4 const& m)
	{
		return math::Transpose(m);
	}

	// --- determinant ---
	inline float determinant(float4x4 const& m)
	{
		return math::Determinant(m);
	}
	#pragma endregion

	// GPU Kernel signature: void(DTID, GID, GTID, GIDX)
	using Kernel = std::function<void(int3, int3, int3, int)>;

	// Execution emulator for running HLSL compute shaders.
	// Supports kernel functions with any subset of the standard parameters:
	//   void fn(int3 dtid)
	//   void fn(int3 dtid, int3 gid)
	//   void fn(int3 dtid, int3 gid, int3 gtid)
	//   void fn(int3 dtid, int3 gid, int3 gtid, int gidx)
	struct GpuEmulator
	{
		Kernel m_kernel;
		int3 m_num_threads;

		explicit GpuEmulator(Kernel kernel, int3 num_threads)
			: m_kernel(std::move(kernel))
			, m_num_threads(num_threads)
		{
		}

		// Overload: void(int3 dtid)
		explicit GpuEmulator(std::function<void(int3)> kernel, int3 num_threads)
			: m_kernel([k = std::move(kernel)](int3 dtid, int3, int3, int) { k(dtid); })
			, m_num_threads(num_threads)
		{
		}

		// Overload: void(int3 dtid, int3 gid)
		explicit GpuEmulator(std::function<void(int3, int3)> kernel, int3 num_threads)
			: m_kernel([k = std::move(kernel)](int3 dtid, int3 gid, int3, int) { k(dtid, gid); })
			, m_num_threads(num_threads)
		{
		}

		// Overload: void(int3 dtid, int3 gid, int3 gtid)
		explicit GpuEmulator(std::function<void(int3, int3, int3)> kernel, int3 num_threads)
			: m_kernel([k = std::move(kernel)](int3 dtid, int3 gid, int3 gtid, int) { k(dtid, gid, gtid); })
			, m_num_threads(num_threads)
		{
		}

		// Emulate dispatching thread groups. The total number of threads = group_dispatch * m_num_threads.
		void Dispatch(int3 group_dispatch)
		{
			for (int z = 0; z != group_dispatch.z; ++z)
			{
				for (int y = 0; y != group_dispatch.y; ++y)
				{
					for (int x = 0; x != group_dispatch.x; ++x)
					{
						int3 group = { x, y, z };
						RunThreadGroup(group);
					}
				}
			}
		}

		// Emulate running a single thread group.
		void RunThreadGroup(int3 group)
		{
			int threads_per_group = m_num_threads.x * m_num_threads.y * m_num_threads.z;

			auto threads = std::ranges::views::iota(0, threads_per_group);
			std::for_each(std::execution::par, threads.begin(), threads.end(), [gid = group, this](int i)
			{
				// Thread id within the group
				int3 gtid = {
					(i / (1                                )) % m_num_threads.x,
					(i / (m_num_threads.x                  )) % m_num_threads.y,
					(i / (m_num_threads.x * m_num_threads.y)) % m_num_threads.z,
				};

				// Dispatch thread ID = thread id within the entire dispatch
				int3 dtid = {
					gid.x * m_num_threads.x + gtid.x,
					gid.y * m_num_threads.y + gtid.y,
					gid.z * m_num_threads.z + gtid.z,
				};

				// Global thread index
				int gi =
					gtid.z * m_num_threads.y * m_num_threads.x +
					gtid.y * m_num_threads.x +
					gtid.x;

				m_kernel(dtid, gid, gtid, gi);
			});
		}
	};
}
