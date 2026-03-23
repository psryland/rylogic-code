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

#ifdef __cplusplus
#include "pr/hlsl/interop.h"
namespace pr::hlsl
{
	// Note:
	//   This error: "error X3000: syntax error: unexpected token 'enum'"
	//   means you have an hlsl file somewhere that hasn't been set to 'Custom Build Tool'.
	//   It will be using the HLSL Compiler build type, which doesn't know about the 'SHADER_BUILD' define

	#define cbuffer struct
	#define numthreads(x, y, z) static int3 constexpr NumThreads = {x,y,z};
	#define reg(reg_number, space) ShaderReg<decltype(reg_number), reg_number, space>
	#define resource(name, reg_number) name 
	#define semantic(semantic_name)
	#define row_major
	#define uniform
	#define row_major
	#define line
	#define inout

	#define DTID(name) name
	#define GID(name) name
	#define GTID(name) name
	#define GIDX(name) name
}

// Make HLSL types visible at the including scope so that .hlsli
// files included from C++ can use float4, int4, float4x4, etc.
using namespace pr::hlsl;

#else

	#define numthreads(x, y, z) [numthreads(x, y, z)]
	#define reg(reg_number, space) : register(reg_number)
	#define resource(name, reg_number) name : register(reg_number)
	#define semantic(semantic_name) :semantic_name
	#define voidp uint2

	#define DTID(name) name : SV_DispatchThreadID
	#define GID(name) name : SV_GroupID
	#define GTID(name) name : SV_GroupThreadID
	#define GIDX(name) name : SV_GroupIndex

#endif

#endif
