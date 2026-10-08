//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
#ifndef PR_VIEW3D_SHADER_TYPES_HLSLI
#define PR_VIEW3D_SHADER_TYPES_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/view3d-12/shaders/vertex.hlsli"

static const float TINY = 0.0001f;
static const int MaxProjectedTextures = 1;
static const int MaxSamplers = 1;

// Model flags
static const int ModelFlags_HasNormals          = (1 << 0);
static const int ModelFlags_IsSkinned           = (1 << 1);
static const int ModelFlags_TwoSided            = (1 << 2);
static const int TextureFlags_HasDiffuse        = (1 << 0);
static const int TextureFlags_IsReflective      = (1 << 1);
static const int TextureFlags_ProjectFromEnvMap = (1 << 2);
static const int AlphaFlags_HasAlpha            = (1 << 0);

// Texture interpretation flags for physically based materials.
static const int PbrTextureFlag_BaseColourSrgb    = (1 << 0);
static const int PbrTextureFlag_EmissiveSrgb      = (1 << 1);
static const int PbrTextureFlag_HasBaseColourMap  = (1 << 2);
static const int PbrTextureFlag_HasMetallicMap    = (1 << 3);
static const int PbrTextureFlag_HasRoughnessMap   = (1 << 4);
static const int PbrTextureFlag_HasEmissiveMap    = (1 << 5);
static const int PbrTextureFlag_HasNormalMap      = (1 << 6);
static const int PbrTextureFlag_NormalMapModel    = (1 << 7); // The normal map stores model-space normals rather than tangent-space perturbations.

// Row major matrix for use in structured buffers
struct Mat4x4
{
	row_major float4x4 m;
};

// Camera
struct Camera
{
	row_major float4x4 c2w; // camera to world
	row_major float4x4 c2s; // camera to screen
	row_major float4x4 w2c; // world to camera
	row_major float4x4 w2s; // world to screen
};

// EnvMap
struct EnvMap
{
	row_major float4x4 w2env; // world to environment map to transform
	float4 blend;             // x = weight of the current environment map over the previous one, in [0,1]
	float4 centre;            // xyz = world-space capture centre of the current map, w = scale of the distances in the map's distance cube (0 = no distances)
	float4 centre_prev;       // xyz = world-space capture centre of the previous map, w = unused
	float4 bounds_min;        // xyz = world-space lower corner of the parallax bounds, w = 1 if reflections correct parallax within the bounds, 0 otherwise
	float4 bounds_max;        // xyz = world-space upper corner of the parallax bounds
};

// Projected textures
struct ProjTexture
{
	int4 info; // x = count of projected textures
	row_major float4x4 w2t[MaxProjectedTextures]; // World to texture space projection transform
};

// Skinned Meshes
struct Skinfluence
{
	uint4 bones;   // 8 16-bit bone indices
	uint4 weights; // 8 16-bit bone weights
};

// Texture coordinate transforms
struct TexXForm
{
	float4 m_x; // First output row of the texture-coordinate transform
	float4 m_y; // Second output row of the texture-coordinate transform
};

// Models
inline bool HasNormals (int4 flags) { return AnySet(flags.x, ModelFlags_HasNormals); }
inline bool IsSkinned  (int4 flags) { return AnySet(flags.x, ModelFlags_IsSkinned); }
inline bool TwoSided   (int4 flags) { return AnySet(flags.x, ModelFlags_TwoSided); }
inline bool HasTex0    (int4 flags) { return AnySet(flags.y, TextureFlags_HasDiffuse); }
inline bool HasEnvMap  (int4 flags) { return AnySet(flags.y, TextureFlags_IsReflective); }
inline bool EnvMapProj (int4 flags) { return AnySet(flags.y, TextureFlags_ProjectFromEnvMap); }
inline bool HasAlpha   (int4 flags) { return AnySet(flags.z, AlphaFlags_HasAlpha); }

// Stock vertex shaders consume the canonical View3D buffered vertex.
typedef View3DVertex VSIn;

// Pixel shader input format
struct PSIn
{
	float4 ss_vert semantic(SV_POSITION);
	float4 ws_vert semantic(POSITION1);
	float4 ws_norm semantic(NORMAL0);
	float4 diff    semantic(COLOR0);
	float2 tex0    semantic(TEXCOORD0);
	float2 idx0    semantic(INDICES0);
};

// Pixel shader input for material variants that interpolate optional texture-coordinate lanes.
struct PSInTexN
{
	float4 ss_vert semantic(SV_POSITION);
	float4 ws_vert semantic(POSITION1);
	float4 ws_norm semantic(NORMAL0);
	float4 diff    semantic(COLOR0);
	float2 tex0    semantic(TEXCOORD0);
	float2 tex1    semantic(TEXCOORD1);
	float2 tex2    semantic(TEXCOORD2);
	float2 tex3    semantic(TEXCOORD3);
	float2 tex4    semantic(TEXCOORD4);
	float2 idx0    semantic(INDICES0);
};

// Compute shader input
struct CSIn
{
	// Example:
	//  [numthreads(10,8,3)] = Number of threads in one thread group.
	//  Dispatch(5,3,2) = Run (5*3*2=30) thread groups.
	//  The threads in each group execute in parallel.

	// 'group_id' is the 3d address in units of groups.
	// 'group_id' is the highest level partitioning, with each value representing a block of threads.
	// e.g. group_id = (0,0,0) = first block of (10*8*3) threads, (1,0,0) is the next block of (10*8*3) threads
	//      Values in the range [0,0,0] -> [5,3,2]
	uint3 group_id semantic(SV_GroupID);

	// 'thread_id' is the global address of the thread, equal to 'group_id'*[numthreads] + 'group_thread_id'.
	//  e.g. Values in the range [0,0,0] -> [5,3,2]*[10,8,3]
	uint3 thread_id semantic(SV_DispatchThreadID);

	// 'group_thread_id' is the address of a thread within a group.
	// e.g. group_thread_id = (0,0,0) = first thread in the current block
	//      Values in the range [0,0,0] -> [10,8,3]
	uint3 group_thread_id semantic(SV_GroupThreadID);

	// 'group_idx' is the 3d address within a group, converted to a 1d index: Z*width*height + Y*width + X
	// e.g. group_idx = group_thread_id.z*numthreads.x*numthreads.y + group_thread_id.y*numthreads.x + group_thread_id.x
	//      Values in the range [0] -> [3*10*8 + 8*10 + 10]
	uint group_idx semantic(SV_GroupIndex);
};

#ifdef SHADER_BUILD
#endif

#endif
