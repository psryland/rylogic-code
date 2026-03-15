//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_CORE_HLSLI
#define PR_HLSL_CORE_HLSLI

static const float tau = 6.28318530717958647693f;
static const float root2_by_2 = 0.70710678118f;
static const float float_max = 3.402823466e+38f;
static const uint FNV_offset_basis32 = 2166136261U;
static const uint FNV_prime32 = 16777619U;

// A transform represented as quaternion rotation + translation.
struct Transform
{
	float4 translation; // {x, y, z, 1}
	float4 rotation;    // {x, y, z, w} (quaternion)
	float4 scale;       // {x, y, z, 1}
};

// Component sign (never zero) functions
inline float sign_nz(float x)
{
	return 2.0f * (x >= 0.0f) - 1.0f;
}
inline float2 sign_nz(float2 x)
{
	return 2.0f * (x >= 0.0f) - 1.0f;
}
inline float3 sign_nz(float3 x)
{
	return 2.0f * (x >= 0.0f) - 1.0f;
}
inline float4 sign_nz(float4 x)
{
	return 2.0f * (x >= 0.0f) - 1.0f;
}

// Component square functions
inline uint sqr(uint v)
{
	return v * v;
}
inline float1 sqr(float1 v)
{
	return v * v;
}
inline float2 sqr(float2 v)
{
	return v * v;
}
inline float3 sqr(float3 v)
{
	return v * v;
}
inline float4 sqr(float4 v)
{
	return v * v;
}

// Component cube functions
inline float1 cube(float1 v)
{
	return v * v * v;
}
inline float2 cube(float2 v)
{
	return v * v * v;
}
inline float3 cube(float3 v)
{
	return v * v * v;
}
inline float4 cube(float4 v)
{
	return v * v * v;
}

// Component signed square function
inline float1 signed_sqr(float1 v)
{
	return sign(v) * sqr(v);
}
inline float2 signed_sqr(float2 v)
{
	return sign(v) * sqr(v);
}
inline float3 signed_sqr(float3 v)
{
	return sign(v) * sqr(v);
}
inline float4 signed_sqr(float4 v)
{
	return sign(v) * sqr(v);
}

// Swap two values
inline void swap(inout float a, inout float b)
{
	float t = a;
	a = b;
	b = t;
}
inline void swap(inout float2 a, inout float2 b)
{
	float2 t = a;
	a = b;
	b = t;
}
inline void swap(inout float3 a, inout float3 b)
{
	float3 t = a;
	a = b;
	b = t;
}
inline void swap(inout float4 a, inout float4 b)
{
	float4 t = a;
	a = b;
	b = t;
}

// Length squared functions
inline float1 length_sq(float1 a)
{
	return dot(a, a);
}
inline float length_sq(float2 a)
{
	return dot(a, a);
}
inline float length_sq(float3 a)
{
	return dot(a, a);
}
inline float length_sq(float4 a)
{
	return dot(a, a);
}

// Sum the components of a float4
inline float sum(float1 v)
{
	return dot(v, float1(1));
}
inline float sum(float2 v)
{
	return dot(v, float2(1,1));
}
inline float sum(float3 v)
{
	return dot(v, float3(1,1,1));
}
inline float sum(float4 v)
{
	return dot(v, float4(1,1,1,1));
}

// Intrinsic for condition ? true_case : false_case
inline int select(bool condition, int true_case, int false_case)
{
	int mask = -(int)condition;
	return (mask & true_case) | (~mask & false_case);
}
inline float1 select(bool condition, float1 true_case, float1 false_case)
{
	return lerp(false_case, true_case, float(condition));
}
inline float2 select(bool condition, float2 true_case, float2 false_case)
{
	return lerp(false_case, true_case, float(condition));
}
inline float3 select(bool condition, float3 true_case, float3 false_case)
{
	return lerp(false_case, true_case, float(condition));
}
inline float4 select(bool condition, float4 true_case, float4 false_case)
{
	return lerp(false_case, true_case, float(condition));
}

// Return the index of the component with the minimum value
inline int min_component_index(float2 v)
{
	// step(b,a) == (a >= b) ? 1 : 0 == (b < a) ? 1 : 0
	return (int)step(v.y, v.x); // y < x ? 1 : 0
}
inline int min_component_index(float3 v)
{
	// step(b,a) == (a >= b) ? 1 : 0 == (b < a) ? 1 : 0
	int xy = (int)step(v.y, v.x); // y < x ? 1 : 0
	return select(v[xy] < v.z, xy, 2);
}
inline int min_component_index(float4 v)
{
	// step(b,a) == (a >= b) ? 1 : 0 == (b < a) ? 1 : 0
	int xy = (int)step(v.y, v.x) + 0; // y < x ? 1 : 0
	int zw = (int)step(v.w, v.z) + 2; // w < z ? 3 : 2
	return select(v[xy] < v[zw], xy, zw);
}
inline int max_component_index(float2 v)
{
	// step(b,a) == (a >= b) ? 1 : 0 == (b < a) ? 1 : 0
	return (int)step(v.x, v.y); // x < y ? 1 : 0
}
inline int max_component_index(float3 v)
{
	// step(b,a) == (a >= b) ? 1 : 0 == (b < a) ? 1 : 0
	int xy = (int)step(v.x, v.y) + 0; // x < y ? 1 : 0
	return select(v[xy] > v.z, xy, 2);
}
inline int max_component_index(float4 v)
{
	// step(b,a) == (a >= b) ? 1 : 0 == (b < a) ? 1 : 0
	int xy = (int)step(v.x, v.y) + 0; // x < y ? 1 : 0
	int zw = (int)step(v.z, v.w) + 2; // z < w ? 3 : 2
	return select(v[xy] > v[zw], xy, zw);
}

// Bit flag test
bool HasFlag(int mask, int flag)
{
	return (mask & flag) != 0;
}
bool HasFlag(uint mask, uint flag)
{
	return (mask & flag) != 0;
}

// Return the parametric position of 'x' on the range [mn, mx]
float Frac(float mn, float x, float mx)
{
	return (x - mn) / (mx - mn);
}
float4 Frac(float4 mn, float4 x, float4 mx)
{
	return (x - mn) / (mx - mn);
}

// Integer square root
int64_t ISqrt(int64_t x)
{
	// Compile time version of the square root.
	//  - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
	//  - This method always converges or oscillates about the answer with a difference of 1.
	//  - returns 0 for x < 0
	if (x < 0)
		return 0;
	
	int64_t curr = x, prev = 0, pprev = 0;
	for (; curr != prev && curr != pprev; )
	{
		pprev = prev;
		prev = curr;
		curr = (curr + x / curr) >> 1;
	}
	return abs(x - curr * curr) < abs(x - prev * prev) ? curr : prev;
}

// Accumulative hash function
inline uint Hash(int value, uint hash = FNV_offset_basis32)
{
	return hash = (value + hash) * FNV_prime32;
}

// A random number on the interval (-1, +1)
inline float RandomN(float2 seed)
{
	// float2(e^pi = 'Gelfond's constant), 2^sqrt(2) = 'Gelfond Schneider's constant)
	const float2 K1 = float2(23.14069263277926, 2.665144142690225);
	return 2.0f * frac(cos(dot(seed, K1)) * 12345.6789) - 1.0f;
}

// A random normalised direction vector on the interval (-1, +1)
inline float4 Random2N(float2 seed)
{
	float t = RandomN(seed) * tau * 0.5f;
	return float4(cos(t), sin(t), 0, 0);
	//float x = RandomN(seed);
	//float y = RandomN(x);
	//return normalize(float2(x, y));
}
inline float4 Random3N(float2 seed)
{
	float t = RandomN(seed) * tau * 0.5f;
	float z = RandomN(t);
	float r = sqrt(1 - sqr(z));
	return float4(r * cos(t), r * sin(t), z, 0);
	//float x = RandomN(seed);
	//float y = RandomN(x);
	//float z = RandomN(y);
	//return normalize(float4(x, y, z, 0));
}
inline float4 Random4(float2 seed)
{
	float x = RandomN(seed);
	float y = RandomN(x);
	float z = RandomN(y);
	float w = RandomN(z);
	return normalize(float4(x, y, z, w));
}

// Optimised conditional test. If all active threads in a wave have 'condition' equal
// the wave will take only take one of the conditional branches
inline bool WaveActiveOrThisThreadTrue(bool condition)
{
	if (WaveActiveAllEqual(condition))
		return condition;

	return condition;
}

#endif
