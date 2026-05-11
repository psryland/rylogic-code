//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/shaders/hlsl/ray_tracing/ray_tracing_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/phong_lighting.hlsli"

#ifdef PR_RDR_LSHADER_ray_trace

ConstantBuffer<CBufFrame> g_frame : register(b0);
RaytracingAccelerationStructure g_scene : register(t0);
Texture2D<float4> g_input : register(t1);
Texture2DMS<float> g_depth : register(t2);
Texture2DMS<float4> g_reflection_attrs : register(t3);
Texture2D<uint4> g_alpha_rt_attrs : register(t4);
StructuredBuffer<RayTracingMaterial> g_materials : register(t5);
StructuredBuffer<RayTracingVertex> g_vertices : register(t6);
Buffer<uint> g_indices16 : register(t7);
Buffer<uint> g_indices32 : register(t8);
StructuredBuffer<RayTracingGeometry> g_geometry : register(t9);
RWTexture2D<float4> g_output : register(u0);

static const uint RayPayloadMode_Diagnostic = 0;
static const uint RayPayloadMode_Primary = 1;
static const uint RayPayloadMode_Shadow = 2;
static const uint RayPayloadMode_Reflection = 3;
static const uint RayPayloadMode_Caustic = 4;

static const float GlassIOR = 1.5f;
static const float DefaultGlassTransmission = 0.85f;

struct RayPayload
{
	float4 colour;
	float hit_t;
	uint mode;
	uint hit;
	uint occluded;
};
struct RasterDepth
{
	float depth;
	uint sample;
};
struct GlassLayer
{
	float3 normal;
	float transmission;
};

// Return a stable pseudo-random colour for diagnostic object/primitive ids.
float3 DiagnosticColour(uint seed)
{
	seed ^= seed >> 16;
	seed *= 0x7feb352du;
	seed ^= seed >> 15;
	seed *= 0x846ca68bu;
	seed ^= seed >> 16;

	float3 colour = float3(
		((seed >> 0) & 0xFFu) / 255.0f,
		((seed >> 8) & 0xFFu) / 255.0f,
		((seed >> 16) & 0xFFu) / 255.0f);
	return lerp(colour, float3(1.0f, 1.0f, 1.0f), 0.20f);
}

// Convert the dispatch pixel to a world-space camera ray using the same camera convention as View3D's raster path.
RayDesc MakeCameraRay(uint2 pixel, uint2 dim)
{
	float2 nss = float2(
		(2.0f * (pixel.x + 0.5f) / dim.x) - 1.0f,
		1.0f - (2.0f * (pixel.y + 0.5f) / dim.y));

	float half_height = g_frame.camera.z * tan(0.5f * g_frame.camera.y);
	float half_width = g_frame.camera.x * half_height;

	RayDesc ray = (RayDesc)0;
	ray.TMin = max(g_frame.clip.x, 0.001f);
	ray.TMax = g_frame.clip.y;

	if (g_frame.camera.w != 0.0f)
	{
		ray.Origin = g_frame.cam.c2w[3].xyz + nss.x * half_width * g_frame.cam.c2w[0].xyz + nss.y * half_height * g_frame.cam.c2w[1].xyz;
		ray.Direction = -g_frame.cam.c2w[2].xyz;
	}
	else
	{
		float3 cs_point = float3(nss.x * half_width, nss.y * half_height, -g_frame.camera.z);
		ray.Origin = g_frame.cam.c2w[3].xyz;
		ray.Direction = normalize(cs_point.x * g_frame.cam.c2w[0].xyz + cs_point.y * g_frame.cam.c2w[1].xyz + cs_point.z * g_frame.cam.c2w[2].xyz);
	}

	return ray;
}

// Return the closest raster depth sample for a pixel, plus the sample index that produced it.
RasterDepth LoadRasterDepth(uint2 pixel)
{
	uint width, height, sample_count;
	g_depth.GetDimensions(width, height, sample_count);

	RasterDepth raster = (RasterDepth)0;
	raster.depth = 1.0f;
	raster.sample = 0;
	for (uint sample = 0; sample != sample_count; ++sample)
	{
		float depth = g_depth.Load(pixel, sample);
		if (depth < raster.depth)
		{
			raster.depth = depth;
			raster.sample = sample;
		}
	}

	return raster;
}

// Return the closest raster depth value for a pixel.
float LoadDepth(uint2 pixel)
{
	return LoadRasterDepth(pixel).depth;
}

// Reconstruct the visible raster position in world space.
float3 WorldPosition(uint2 pixel, uint2 dim, float depth)
{
	float2 nss = float2(
		(2.0f * (pixel.x + 0.5f) / dim.x) - 1.0f,
		1.0f - (2.0f * (pixel.y + 0.5f) / dim.y));

	float4 ws_pos = mul(float4(nss, depth, 1.0f), g_frame.s2w);
	return ws_pos.xyz / ws_pos.w;
}

// Reconstruct an approximate receiver normal from neighbouring depth samples.
float3 ReceiverNormal(uint2 pixel, uint2 dim, float depth, RayDesc camera_ray)
{
	float3 pos = WorldPosition(pixel, dim, depth);
	uint2 pixel_x = pixel;
	uint2 pixel_y = pixel;
	float x_sign = 1.0f;
	float y_sign = 1.0f;

	if (pixel.x + 1u < dim.x)
		pixel_x.x = pixel.x + 1u;
	else if (pixel.x > 0u)
	{
		pixel_x.x = pixel.x - 1u;
		x_sign = -1.0f;
	}

	if (pixel.y + 1u < dim.y)
		pixel_y.y = pixel.y + 1u;
	else if (pixel.y > 0u)
	{
		pixel_y.y = pixel.y - 1u;
		y_sign = -1.0f;
	}

	if (all(pixel_x == pixel) || all(pixel_y == pixel))
		return normalize(-camera_ray.Direction);

	float depth_x = LoadDepth(pixel_x);
	float depth_y = LoadDepth(pixel_y);
	if (depth_x >= 0.999999f)
		depth_x = depth;
	if (depth_y >= 0.999999f)
		depth_y = depth;

	float3 dx = (WorldPosition(pixel_x, dim, depth_x) - pos) * x_sign;
	float3 dy = (WorldPosition(pixel_y, dim, depth_y) - pos) * y_sign;
	float3 normal = cross(dx, dy);
	float len_sq = dot(normal, normal);
	if (len_sq < 1e-10f)
		return normalize(-camera_ray.Direction);

	normal *= rsqrt(len_sq);
	if (dot(normal, -camera_ray.Direction) < 0.0f)
		normal = -normal;

	return normal;
}

// Return a simple sky fallback for reflection rays that miss the TLAS.
float3 ReflectionMissColour(float3 ws_direction)
{
	float t = saturate(0.5f + 0.5f * ws_direction.y);
	return lerp(float3(0.04f, 0.045f, 0.055f), float3(0.28f, 0.34f, 0.45f), t);
}

// Return the material table index for the current closest hit.
uint HitMaterialIndex()
{
	return InstanceID() + GeometryIndex();
}

// Return the diffuse material colour for the hit instance geometry.
float4 HitMaterial()
{
	uint material_count = uint(g_frame.options.y);
	uint material_index = HitMaterialIndex();
	if (material_index < material_count)
		return g_materials[material_index].diffuse;

	uint instance_index = InstanceIndex();
	if (instance_index < material_count)
		return g_materials[instance_index].diffuse;

	return float4(0.75f, 0.75f, 0.75f, 1.0f);
}

// Return true if the current hit has enough sidecar metadata for triangle reconstruction.
bool HasHitGeometry(uint geometry_index)
{
	uint geometry_count = uint(g_frame.options.z);
	return geometry_index < geometry_count && (g_geometry[geometry_index].flags.x & RayTracingGeometryFlag_HasGeometry) != 0;
}

// Return one triangle index from the packed RT index buffers.
uint HitTriangleIndex(RayTracingGeometry geometry, uint corner)
{
	uint index = geometry.ranges.z + PrimitiveIndex() * 3u + corner;
	if ((geometry.flags.x & RayTracingGeometryFlag_Index16) != 0)
		return g_indices16[index];

	return g_indices32[index];
}

// Shade the closest-hit surface by interpolating packed geometry attributes.
float4 ShadeRayHit(in BuiltInTriangleIntersectionAttributes attrib)
{
	uint geometry_index = HitMaterialIndex();
	float4 material = HitMaterial();
	if (!HasHitGeometry(geometry_index))
		return material;

	RayTracingGeometry geometry = g_geometry[geometry_index];
	uint first_index = PrimitiveIndex() * 3u;
	if (first_index + 2u >= geometry.ranges.w)
		return material;

	uint i0 = HitTriangleIndex(geometry, 0) - geometry.ranges.y;
	uint i1 = HitTriangleIndex(geometry, 1) - geometry.ranges.y;
	uint i2 = HitTriangleIndex(geometry, 2) - geometry.ranges.y;
	if (i0 >= geometry.flags.y || i1 >= geometry.flags.y || i2 >= geometry.flags.y)
		return material;

	float3 bary = float3(1.0f - attrib.barycentrics.x - attrib.barycentrics.y, attrib.barycentrics.x, attrib.barycentrics.y);

	RayTracingVertex v0 = g_vertices[geometry.ranges.x + i0];
	RayTracingVertex v1 = g_vertices[geometry.ranges.x + i1];
	RayTracingVertex v2 = g_vertices[geometry.ranges.x + i2];
	float4 colour = (bary.x * v0.colour + bary.y * v1.colour + bary.z * v2.colour) * material;
	if ((geometry.flags.x & RayTracingGeometryFlag_HasNormals) == 0)
		return colour;

	float4 normal = bary.x * v0.normal + bary.y * v1.normal + bary.z * v2.normal;
	normal.w = 0.0f;
	float normal_len_sq = dot(normal.xyz, normal.xyz);
	if (normal_len_sq < 1e-10f)
		return colour;

	float4 ws_normal = mul(normal * rsqrt(normal_len_sq), geometry.normal_to_world);
	ws_normal.w = 0.0f;
	float ws_normal_len_sq = dot(ws_normal.xyz, ws_normal.xyz);
	if (ws_normal_len_sq < 1e-10f)
		return colour;

	ws_normal *= rsqrt(ws_normal_len_sq);
	if (dot(ws_normal.xyz, -WorldRayDirection()) < 0.0f)
		ws_normal = -ws_normal;

	float4 ws_pos = float4(WorldRayOrigin() + RayTCurrent() * WorldRayDirection(), 1.0f);
	return Illuminate(g_frame.global_light, ws_pos, ws_normal, g_frame.cam.c2w[3], 1.0f, colour);
}

// Return a stable pseudo-random value for a projected caustic cell.
float2 Hash22(float2 value)
{
	float2 h = float2(
		dot(value, float2(127.1f, 311.7f)),
		dot(value, float2(269.5f, 183.3f)));
	return frac(sin(h) * 43758.5453f);
}

// Return a jittered cellular filament mask for a caustic pattern layer.
float CausticCells(float2 p)
{
	float2 cell = floor(p);
	float2 cell_uv = frac(p);
	float nearest = 16.0f;
	float second_nearest = 16.0f;

	// Measure the two closest jittered feature points. Brightness comes from the boundary between nearby cells, which avoids the old axis-product grid.
	[unroll]
	for (int y = -1; y != 2; ++y)
	{
		[unroll]
		for (int x = -1; x != 2; ++x)
		{
			float2 neighbour = float2(x, y);
			float2 jitter = Hash22(cell + neighbour) - 0.5f;
			float2 feature = neighbour + 0.5f + 0.85f * jitter;
			float dist_sq = dot(feature - cell_uv, feature - cell_uv);

			if (dist_sq < nearest)
			{
				second_nearest = nearest;
				nearest = dist_sq;
			}
			else if (dist_sq < second_nearest)
			{
				second_nearest = dist_sq;
			}
		}
	}

	float edge = sqrt(second_nearest) - sqrt(nearest);
	return 1.0f - smoothstep(0.025f, 0.18f, edge);
}

// Return a visible caustic proof pattern in the receiver plane projected along the light direction.
float CausticPattern(float3 ws_pos, float3 surface_to_light)
{
	float3 up = abs(surface_to_light.z) < 0.9f ? float3(0.0f, 0.0f, 1.0f) : float3(0.0f, 1.0f, 0.0f);
	float3 axis_x = normalize(cross(up, surface_to_light));
	float3 axis_y = cross(surface_to_light, axis_x);
	float2 p = float2(dot(ws_pos, axis_x), dot(ws_pos, axis_y)) * max(g_frame.caustic.z, 0.001f);

	float2 p0 = float2(
		dot(p, float2(0.8660254f, -0.5f)),
		dot(p, float2(0.5f, 0.8660254f)));
	float2 p1 = float2(
		dot(p, float2(0.4226183f, -0.9063078f)),
		dot(p, float2(0.9063078f, 0.4226183f))) * 1.73f + 19.37f;

	float cells = CausticCells(p0);
	float fine_cells = CausticCells(p1);
	float shimmer = 0.5f + 0.5f * sin(dot(p, float2(6.7f, -4.9f)) + 1.7f * fine_cells);
	return saturate(0.65f * cells + 0.25f * fine_cells + 0.10f * shimmer);
}

// Unpack an RGBA8 value produced by the alpha RT sidecar.
float4 UnpackRtRGBA8(uint colour)
{
	uint4 c = uint4(colour, colour >> 8, colour >> 16, colour >> 24) & 0xFFu;
	return float4(c) / 255.0f;
}

// Fold one packed alpha sidecar layer into the strongest glass layer candidate for this pixel.
void ConsiderGlassLayer(inout GlassLayer best, uint packed_attrs, float3 incident)
{
	float4 attrs = UnpackRtRGBA8(packed_attrs);
	if (attrs.a <= best.transmission)
		return;

	float3 normal = 2.0f * attrs.rgb - 1.0f;
	float len_sq = dot(normal, normal);
	if (len_sq < 1e-6f)
		return;

	normal *= rsqrt(len_sq);
	if (dot(normal, incident) > 0.0f)
		normal = -normal;

	best.normal = normal;
	best.transmission = attrs.a;
}

// Return the strongest camera-visible transparent layer for this pixel, treating it as default glass.
GlassLayer StrongestGlassLayer(uint2 pixel, float3 fallback_normal, float3 incident)
{
	uint4 attrs = g_alpha_rt_attrs.Load(int3(pixel, 0));
	GlassLayer glass;
	glass.normal = fallback_normal;
	glass.transmission = 0.0f;

	ConsiderGlassLayer(glass, attrs.x, incident);
	ConsiderGlassLayer(glass, attrs.y, incident);
	ConsiderGlassLayer(glass, attrs.z, incident);
	ConsiderGlassLayer(glass, attrs.w, incident);
	return glass;
}

// Estimate a receiver-to-light ray through a temporary glass material model.
float3 GlassRayDirection(float3 incident, GlassLayer glass)
{
	if (glass.transmission <= 0.0f)
		return incident;

	float3 refracted = refract(incident, glass.normal, 1.0f / GlassIOR);
	float refracted_len_sq = dot(refracted, refracted);
	if (refracted_len_sq < 1e-6f)
		return incident;

	refracted *= rsqrt(refracted_len_sq);
	return normalize(lerp(incident, refracted, glass.transmission));
}

// Approximate how much of the caustic-producing light survives a temporary glass material.
float GlassTransmission(float3 incident, GlassLayer glass, bool hit_glass)
{
	if (!hit_glass)
		return 0.0f;

	float transmission = glass.transmission > 0.0f ? glass.transmission : DefaultGlassTransmission;
	float facing = glass.transmission > 0.0f ? abs(dot(incident, glass.normal)) : 1.0f;
	float fresnel_reflectance = 0.04f + 0.96f * pow(1.0f - saturate(facing), 5.0f);
	return transmission * (1.0f - fresnel_reflectance);
}

[shader("raygeneration")]
void RayGen()
{
	uint2 pixel = DispatchRaysIndex().xy;
	uint2 dim = DispatchRaysDimensions().xy;

	RayPayload payload;
	payload.colour = float4(0.02f, 0.03f, 0.05f, 1.0f);
	payload.hit_t = 0.0f;
	payload.mode = RayPayloadMode_Diagnostic;
	payload.hit = 0;
	payload.occluded = 0;

	if (g_frame.options.x == RayTracingMode_Diagnostic)
	{
		RayDesc ray = MakeCameraRay(pixel, dim);
		TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE, RayTracingInstanceMask_All, 0, 1, 0, ray, payload);
		g_output[pixel] = payload.colour;
		return;
	}

	float4 colour = g_input.Load(int3(pixel, 0));
	if (g_frame.options.x == RayTracingMode_Reflections || g_frame.options.x == RayTracingMode_ReflectionsAndCaustics)
	{
		RasterDepth raster = LoadRasterDepth(pixel);
		if (raster.depth < 0.999999f)
		{
			float4 reflection_attrs = g_reflection_attrs.Load(pixel, raster.sample);
			float reflectivity = saturate(reflection_attrs.a * g_frame.reflection.x);
			if (reflectivity > 0.0f)
			{
				RayDesc camera_ray = MakeCameraRay(pixel, dim);
				float3 hit_pos = WorldPosition(pixel, dim, raster.depth);
				float3 normal = normalize(2.0f * reflection_attrs.xyz - 1.0f);
				if (dot(normal, -camera_ray.Direction) < 0.0f)
					normal = -normal;

				float3 reflection_dir = normalize(reflect(camera_ray.Direction, normal));
				float reflection_bias = max(g_frame.reflection.y, length(hit_pos - camera_ray.Origin) * 0.0001f);

				RayPayload reflection_payload;
				reflection_payload.colour = 0.0f;
				reflection_payload.hit_t = 0.0f;
				reflection_payload.mode = RayPayloadMode_Reflection;
				reflection_payload.hit = 0;
				reflection_payload.occluded = 0;

				RayDesc reflection_ray = (RayDesc)0;
				reflection_ray.Origin = hit_pos + normal * reflection_bias;
				reflection_ray.Direction = reflection_dir;
				reflection_ray.TMin = 0.0f;
				reflection_ray.TMax = g_frame.clip.y;

				TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE, RayTracingInstanceMask_All, 0, 1, 0, reflection_ray, reflection_payload);

				float3 reflection_colour = reflection_payload.hit != 0
					? reflection_payload.colour.rgb
					: ReflectionMissColour(reflection_dir);
				colour.rgb = lerp(colour.rgb, reflection_colour, reflectivity);
			}
		}

		if (g_frame.options.x == RayTracingMode_Reflections)
		{
			g_output[pixel] = colour;
			return;
		}
	}

	if (g_frame.options.x == RayTracingMode_Caustics || g_frame.options.x == RayTracingMode_ReflectionsAndCaustics)
	{
		RasterDepth raster = LoadRasterDepth(pixel);
		if (DirectionalLight(g_frame.global_light) && raster.depth < 0.999999f)
		{
			RayDesc camera_ray = MakeCameraRay(pixel, dim);
			float3 hit_pos = WorldPosition(pixel, dim, raster.depth);
			float3 normal = ReceiverNormal(pixel, dim, raster.depth, camera_ray);
			float3 light_to_surface = normalize(g_frame.global_light.ws_direction.xyz);
			float3 surface_to_light = -light_to_surface;
			float receiver_facing = saturate(dot(normal, surface_to_light));

			if (receiver_facing > 0.0f)
			{
				GlassLayer glass = StrongestGlassLayer(pixel, normal, surface_to_light);
				float3 caustic_dir = GlassRayDirection(surface_to_light, glass);

				RayPayload caustic_payload;
				caustic_payload.colour = 0.0f;
				caustic_payload.hit_t = 0.0f;
				caustic_payload.mode = RayPayloadMode_Caustic;
				caustic_payload.hit = 0;
				caustic_payload.occluded = 1;

				float caustic_bias = max(g_frame.caustic.y, length(hit_pos - camera_ray.Origin) * 0.0001f);
				RayDesc caustic_ray = (RayDesc)0;
				caustic_ray.Origin = hit_pos + surface_to_light * caustic_bias;
				caustic_ray.Direction = caustic_dir;
				caustic_ray.TMin = 0.0f;
				caustic_ray.TMax = g_frame.clip.y;

				TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER, RayTracingInstanceMask_Caustic, 0, 1, 0, caustic_ray, caustic_payload);
				if (caustic_payload.occluded == 0 && glass.transmission > 0.0f)
				{
					caustic_payload.occluded = 1;
					caustic_ray.Direction = surface_to_light;
					TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER, RayTracingInstanceMask_Caustic, 0, 1, 0, caustic_ray, caustic_payload);
				}
				if (caustic_payload.occluded != 0)
				{
					float3 pattern_pos = hit_pos + caustic_dir * max(g_frame.caustic.w, 0.0f);
					float pattern = CausticPattern(pattern_pos, caustic_dir);
					float glass_transmission = GlassTransmission(surface_to_light, glass, true);
					float focus = 1.0f + saturate(length(caustic_dir - surface_to_light) * max(g_frame.caustic.w, 0.0f));
					float strength = g_frame.caustic.x *
						receiver_facing *
						lerp(0.35f, 1.0f, pattern) *
						glass_transmission *
						focus;
					colour.rgb += g_frame.global_light.colour.rgb * strength;
				}
			}
		}

		g_output[pixel] = colour;
		return;
	}

	if (!DirectionalLight(g_frame.global_light))
	{
		g_output[pixel] = colour;
		return;
	}

	float depth = LoadDepth(pixel);
	if (depth < 0.999999f)
	{
		float3 light_to_surface = normalize(g_frame.global_light.ws_direction.xyz);
		float3 surface_to_light = -light_to_surface;
		float shadow_bias = max(g_frame.shadow.y, length(WorldPosition(pixel, dim, depth) - g_frame.cam.c2w[3].xyz) * 0.0001f);
		float3 hit_pos = WorldPosition(pixel, dim, depth);

		RayPayload shadow_payload;
		shadow_payload.colour = 0.0f;
		shadow_payload.hit_t = 0.0f;
		shadow_payload.mode = RayPayloadMode_Shadow;
		shadow_payload.hit = 0;
		shadow_payload.occluded = 1;

		RayDesc shadow_ray = (RayDesc)0;
		shadow_ray.Origin = hit_pos + surface_to_light * shadow_bias;
		shadow_ray.Direction = surface_to_light;
		shadow_ray.TMin = 0.0f;
		shadow_ray.TMax = g_frame.clip.y;

		TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER, RayTracingInstanceMask_All, 0, 1, 0, shadow_ray, shadow_payload);
		if (shadow_payload.occluded != 0)
			colour.rgb *= saturate(1.0f - g_frame.shadow.x);
	}

	g_output[pixel] = colour;
}

[shader("miss")]
void Miss(inout RayPayload payload)
{
	if (payload.mode == RayPayloadMode_Shadow || payload.mode == RayPayloadMode_Caustic)
	{
		payload.occluded = 0;
		return;
	}
	if (payload.mode == RayPayloadMode_Reflection)
	{
		payload.hit = 0;
		return;
	}

	payload.hit = 0;
	payload.colour = float4(0.02f, 0.03f, 0.05f, 1.0f);
}

[shader("closesthit")]
void ClosestHit(inout RayPayload payload, in BuiltInTriangleIntersectionAttributes attrib)
{
	if (payload.mode == RayPayloadMode_Reflection)
	{
		payload.hit = 1;
		payload.hit_t = RayTCurrent();
		payload.colour = ShadeRayHit(attrib);
		return;
	}
	if (payload.mode != RayPayloadMode_Diagnostic)
	{
		payload.hit = 1;
		payload.hit_t = RayTCurrent();
		return;
	}

	uint seed = InstanceIndex() ^ (PrimitiveIndex() * 747796405u);
	float edge = min(attrib.barycentrics.x, min(attrib.barycentrics.y, 1.0f - attrib.barycentrics.x - attrib.barycentrics.y));
	float wire = smoothstep(0.0f, 0.02f, edge);
	payload.colour = float4(lerp(float3(0.02f, 0.02f, 0.02f), DiagnosticColour(seed), wire), 1.0f);
}

#endif

#if defined(PR_RDR_VSHADER_ray_trace_present) || defined(PR_RDR_PSHADER_ray_trace_present)

struct RTPresentIn
{
	float4 ss_vert :SV_Position;
	float2 tex0 :TEXCOORD0;
};

#ifdef PR_RDR_VSHADER_ray_trace_present
RTPresentIn main(uint vid :SV_VertexID)
{
	RTPresentIn Out = (RTPresentIn)0;
	Out.tex0 = float2((vid << 1) & 2, vid & 2);
	Out.ss_vert = float4(Out.tex0 * float2(2, -2) + float2(-1, 1), 0, 1);
	return Out;
}
#endif

#ifdef PR_RDR_PSHADER_ray_trace_present
Texture2D<float4> g_rt_output : register(t0);

float4 main(RTPresentIn In) :SV_Target
{
	return g_rt_output.Load(int3(In.ss_vert.xy, 0));
}
#endif

#endif
