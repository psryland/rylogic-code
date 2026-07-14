//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_INTEROP_HLSLI
#define PR_HLSL_INTEROP_HLSLI

// Shared types for HLSL shaders and C++ code.
// This file is included from both HLSL and C++ source.
// Any C++ code that includes hlsl files will need `using namespace pr::hlsl;`

// Notes:
// - use float4x4 not matrix... they're not the same (don't know why tho)
// - HLSL float4x4 is column major (by default) but pr::m4x4 is row major.
//   Remember to transpose matrices or use 'row_major' before any float4x4's.
// - Use ConstantBuffer<T> rather than 'cbuffer' for better C++ interop.

#ifdef __cplusplus
#include "pr/hlsl/interop.h"

	// Note:
	//   This error: "error X3000: syntax error: unexpected token 'enum'"
	//   means you have an hlsl file somewhere that hasn't been set to 'Custom Build Tool'.
	//   It will be using the HLSL Compiler build type, which doesn't know about the 'SHADER_BUILD' define

	#define numthreads(kernel, x, y, z) static int3 constexpr kernel ## _NumThreads = {x,y,z};
	#define resource(name, reg_number) name 
	#define semantic(semantic_name)
	#define arrayout_(ty, name, size) ty (&name)[size]
	#define inout_(ty) ty&
	#define out_(ty) ty&
	#define in_(ty) ty const&
	#define row_major

	#define DTID(name) name
	#define GID(name) name
	#define GTID(name) name
	#define GIDX(name) name

	// Function-definition decorators that work in both C++ and HLSL contexts.
	//   odr           - allow a function definition to live in a header. In C++ it expands to
	//                   'template <typename = void>' so each call instantiates a single weak-linkage
	//                   instance (no ODR violation across translation units). In HLSL it is empty,
	//                   leaving the function as an ordinary definition that DXC can inline freely.
	//   hlsl_noinline - request that DXC keep the function out-of-line (one shared body, called
	//                   indirectly). No effect in C++ (where ordinary inlining decisions apply).
	//                   Use sparingly on large functions that have multiple call sites in the same
	//                   compute shader, where inlining produces excessive bytecode and slow compiles.
	#define odr template <typename = void>
	#define hlsl_noinline

#else

	#define numthreads(kernel, x, y, z) [numthreads(x, y, z)]
	#define resource(name, reg_number) name : register(reg_number)
	#define semantic(semantic_name) :semantic_name
	#define arrayout_(ty, name, size) out ty name[size]
	#define inout_(ty) inout ty
	#define out_(ty) out ty
	#define in_(ty) in ty
	#define voidp uint2

	#define DTID(name) name : SV_DispatchThreadID
	#define GID(name) name : SV_GroupID
	#define GTID(name) name : SV_GroupThreadID
	#define GIDX(name) name : SV_GroupIndex

	// See the C++ branch above for documentation. In HLSL, 'odr' is empty (DXC inlines by default
	// for compute targets), and 'hlsl_noinline' becomes the [noinline] attribute, which forces DXC
	// to emit one shared DXIL function body instead of duplicating it at every call site.
	#define odr
	#define hlsl_noinline [noinline]

	#define assert(x)

	// Return the number of elements in a StructuredBuffer or RWStructuredBuffer.
	template <typename T> inline int ssize(in StructuredBuffer<T> buf)
	{
		uint count, stride;
		buf.GetDimensions(count, stride);
		return (int)count;
	}
	template <typename T> inline int ssize(in RWStructuredBuffer<T> buf)
	{
		uint count, stride;
		buf.GetDimensions(count, stride);
		return (int)count;
	}

#endif

#endif
