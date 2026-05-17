//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/shaders/hlsl/ray_tracing/ray_tracing_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/phong_lighting.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/kbuffer.hlsli"

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
RWTexture2D<uint4> g_alpha_colour : register(u1);
RWTexture2D<uint4> g_alpha_depth : register(u2);

static const uint RayPayloadMode_Diagnostic = 0;
static const uint RayPayloadMode_Primary = 1;
static const uint RayPayloadMode_Shadow = 2;
static const uint RayPayloadMode_Reflection = 3;
static const uint RayPayloadMode_Caustic = 4;

static const uint RayPayloadFlag_HasSurfaceNormal = 0x01;
static const uint RayPayloadFlag_ReflectionValid = 0x02;

static const uint MaxReflectionTransparentHits = 4;
static const float ReflectionOpaqueAlpha = 0.995f;
static const float DefaultGlassIOR = 1.5f;
static const float DefaultGlassTransmission = 0.85f;

// This shader is a hybrid post-raster pass. The raster pipeline remains authoritative for camera-visible colour and depth; DXR adds optional overlays:
// diagnostics trace the TLAS directly, reflections start from raster or alpha-layer hits, caustics pull transmissive information from the light path,
// and shadow mode casts a visibility ray from the raster receiver.
struct RayPayload
{
	float3 colour;
	uint mode;
	float3 normal;
	uint flags;
	float hit_t;
	float reflectivity;
	float opacity;
	uint hit;
	uint occluded;
};
struct RasterDepth
{
	float depth;
	uint sample;
};

// Return true if 'mode' enables raster-seeded reflection tracing.
bool RayTracingModeIncludesReflections(uint mode)
{
	return mode == RayTracingMode_Reflections || mode == RayTracingMode_ReflectionsAndCaustics;
}

// Return true if 'mode' enables receiver-pull caustic tracing.
bool RayTracingModeIncludesCaustics(uint mode)
{
	return mode == RayTracingMode_Caustics || mode == RayTracingMode_ReflectionsAndCaustics;
}

// Create a fully-initialised payload for a ray tracing query.
RayPayload InitRayPayload(uint mode, float3 colour)
{
	RayPayload payload = (RayPayload)0;
	payload.colour = colour;
	payload.mode = mode;
	return payload;
}

// Create a visibility-style payload where the miss shader clears occlusion.
RayPayload InitOcclusionPayload(uint mode)
{
	RayPayload payload = InitRayPayload(mode, 0.0f);
	payload.occluded = 1;
	return payload;
}

// Return true if 'mode' treats a miss as "not occluded" rather than as a colour sample.
bool PayloadModeIsVisibilityQuery(uint mode)
{
	return mode == RayPayloadMode_Shadow || mode == RayPayloadMode_Caustic;
}

// Return true if 'payload' includes a usable world-space surface normal.
bool PayloadHasSurfaceNormal(RayPayload payload)
{
	return (payload.flags & RayPayloadFlag_HasSurfaceNormal) != 0;
}

// Return true if a reflection payload is allowed to seed another bounce.
bool PayloadCanSpawnReflection(RayPayload payload)
{
	return (payload.flags & RayPayloadFlag_ReflectionValid) != 0;
}

// Return true if the reflection path should reserve reflected energy and continue to the next bounce.
bool ReflectionCanContinue(
	uint bounce,
	uint max_bounces,
	float throughput,
	float transparent_visibility,
	float hit_reflectivity,
	float min_throughput,
	RayPayload payload)
{
	return bounce + 1u < max_bounces &&
		throughput * transparent_visibility * hit_reflectivity > min_throughput &&
		PayloadCanSpawnReflection(payload);
}

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

	// Orthographic rays start on the camera plane and share the camera forward direction.
	if (g_frame.camera.w != 0.0f)
	{
		ray.Origin = g_frame.cam.c2w[3].xyz + nss.x * half_width * g_frame.cam.c2w[0].xyz + nss.y * half_height * g_frame.cam.c2w[1].xyz;
		ray.Direction = -g_frame.cam.c2w[2].xyz;
	}
	else
	{
		// Perspective rays all start at the camera origin and pass through the corresponding point on the focus plane.
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

	// Choose one horizontal and one vertical neighbour. At image borders, sample inward and flip the derivative direction to keep the winding consistent.
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

	// Missing neighbour depth means the neighbour is background. Reuse the centre depth so silhouettes don't generate enormous derivatives.
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

// Return the small world-space offset used to restart rays without self-intersecting the current surface.
float RayBias(float base_bias, float distance)
{
	return max(base_bias, distance * 0.0001f);
}

// Return 'normal' flipped toward the incoming ray direction.
float3 FaceNormalToRay(float3 normal, float3 ray_direction)
{
	if (dot(normal, -ray_direction) < 0.0f)
		normal = -normal;

	return normal;
}

// Return a world-space ray with a zero near plane, used for secondary RT probes launched from known surface points.
RayDesc MakeSecondaryRay(float3 origin, float3 direction, float t_max)
{
	RayDesc ray = (RayDesc)0;
	ray.Origin = origin;
	ray.Direction = direction;
	ray.TMin = 0.0f;
	ray.TMax = t_max;
	return ray;
}

// Return a reflection ray seeded from a raster or alpha-layer surface.
RayDesc MakeSurfaceReflectionRay(RayDesc camera_ray, float3 hit_pos, float3 normal, float t_max)
{
	normal = FaceNormalToRay(normal, camera_ray.Direction);
	float reflection_bias = RayBias(g_frame.reflection.y, length(hit_pos - camera_ray.Origin));
	float3 reflection_dir = normalize(reflect(camera_ray.Direction, normal));
	return MakeSecondaryRay(hit_pos + normal * reflection_bias, reflection_dir, t_max);
}

// Advance a ray through a transparent closest hit so the next query can see layers behind it.
void AdvanceRayThroughTransparentHit(inout RayDesc ray, RayPayload payload)
{
	float3 hit_pos = ray.Origin + ray.Direction * payload.hit_t;
	float through_bias = RayBias(g_frame.reflection.y * 0.1f, payload.hit_t);
	ray.Origin = hit_pos + ray.Direction * through_bias;
	ray.TMin = 0.0f;
}

// Return a ray reflected from a closest-hit payload.
RayDesc ReflectionRayFromHit(RayDesc incident_ray, RayPayload payload)
{
	float3 normal = FaceNormalToRay(normalize(payload.normal), incident_ray.Direction);
	float3 hit_pos = incident_ray.Origin + incident_ray.Direction * payload.hit_t;
	float reflection_bias = RayBias(g_frame.reflection.y, payload.hit_t);
	float3 reflection_dir = normalize(reflect(incident_ray.Direction, normal));
	return MakeSecondaryRay(hit_pos + normal * reflection_bias, reflection_dir, incident_ray.TMax);
}

// Trace a bounded reflection path without spawning reflected branches from transparent hits.
float3 TraceReflectionPathThroughAlpha(RayDesc reflection_ray, uint max_bounces)
{
	float min_throughput = max(g_frame.reflection.w, 0.0f);
	float3 reflection_colour = 0.0f;
	float throughput = 1.0f;

	// This specialised traversal is called from the main reflection path for a transparent hit's reflected layer. It deliberately does not launch further
	// reflected branches from transparent hits, which keeps the call graph non-recursive while still allowing opaque bounce candidates.
	[loop]
	for (uint bounce = 0; bounce != max_bounces; ++bounce)
	{
		RayDesc layer_ray = reflection_ray;
		RayPayload reflection_payload = InitRayPayload(RayPayloadMode_Reflection, 0.0f);
		float3 transparent_colour = 0.0f;
		float transparent_visibility = 1.0f;

		[loop]
		for (uint transparent_hit = 0; transparent_hit != MaxReflectionTransparentHits; ++transparent_hit)
		{
			reflection_payload = InitRayPayload(RayPayloadMode_Reflection, 0.0f);
			TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE, RayTracingInstanceMask_All, 0, 1, 0, layer_ray, reflection_payload);

			if (reflection_payload.hit == 0)
			{
				reflection_colour += throughput * (transparent_colour + transparent_visibility * ReflectionMissColour(layer_ray.Direction));
				return reflection_colour;
			}

			float hit_opacity = saturate(reflection_payload.opacity);
			if (hit_opacity >= ReflectionOpaqueAlpha)
				break;

			transparent_colour += transparent_visibility * hit_opacity * reflection_payload.colour;
			transparent_visibility *= 1.0f - hit_opacity;
			if (throughput * transparent_visibility <= min_throughput)
			{
				reflection_colour += throughput * transparent_colour;
				return reflection_colour;
			}
			if (transparent_hit + 1u == MaxReflectionTransparentHits)
			{
				reflection_colour += throughput * (transparent_colour + transparent_visibility * ReflectionMissColour(layer_ray.Direction));
				return reflection_colour;
			}

			AdvanceRayThroughTransparentHit(layer_ray, reflection_payload);
		}

		// If the terminal surface can bounce, keep only the non-reflected part locally and carry the reflected energy into the next iteration.
		float hit_reflectivity = saturate(reflection_payload.reflectivity);
		bool can_continue = ReflectionCanContinue(bounce, max_bounces, throughput, transparent_visibility, hit_reflectivity, min_throughput, reflection_payload);

		reflection_colour += throughput * (transparent_colour + transparent_visibility * (can_continue ? 1.0f - hit_reflectivity : 1.0f) * reflection_payload.colour);
		if (!can_continue)
			break;

		throughput *= transparent_visibility * hit_reflectivity;
		if (throughput <= min_throughput)
			break;

		reflection_ray = ReflectionRayFromHit(layer_ray, reflection_payload);
	}

	return reflection_colour;
}

// Trace a bounded iterative reflection path and return the reflected colour seen along 'reflection_ray'.
float3 TraceReflectionPath(RayDesc reflection_ray)
{
	uint max_bounces = max(1u, uint(g_frame.reflection.z + 0.5f));
	float min_throughput = max(g_frame.reflection.w, 0.0f);
	float3 reflection_colour = 0.0f;
	float throughput = 1.0f;

	// Iterate in raygen rather than using recursive DXR so the bounce count and payload size remain predictable.
	[loop]
	for (uint bounce = 0; bounce != max_bounces; ++bounce)
	{
		RayDesc layer_ray = reflection_ray;
		RayPayload reflection_payload = InitRayPayload(RayPayloadMode_Reflection, 0.0f);
		float3 transparent_colour = 0.0f;
		float transparent_visibility = 1.0f;

		// Composite a small number of transparent intersections along the same ray before treating the next surface as the bounce candidate.
		[loop]
		for (uint transparent_hit = 0; transparent_hit != MaxReflectionTransparentHits; ++transparent_hit)
		{
			reflection_payload = InitRayPayload(RayPayloadMode_Reflection, 0.0f);
			TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE, RayTracingInstanceMask_All, 0, 1, 0, layer_ray, reflection_payload);

			if (reflection_payload.hit == 0)
			{
				// A miss terminates the path with a cheap sky colour so mirror-only scenes still have something to reflect.
				reflection_colour += throughput * (transparent_colour + transparent_visibility * ReflectionMissColour(layer_ray.Direction));
				return reflection_colour;
			}

			float hit_opacity = saturate(reflection_payload.opacity);
			if (hit_opacity >= ReflectionOpaqueAlpha)
				break;

			// Transparent layers are visible through their opacity, but their own reflectivity can still contribute a bounded one-off reflected sample.
			float hit_reflectivity = saturate(reflection_payload.reflectivity);
			float3 layer_colour = reflection_payload.colour;
			if (ReflectionCanContinue(bounce, max_bounces, throughput, transparent_visibility, hit_reflectivity, min_throughput, reflection_payload))
			{
				RayDesc surface_reflection_ray = ReflectionRayFromHit(layer_ray, reflection_payload);
				uint remaining_bounces = max_bounces - bounce - 1u;
				layer_colour = lerp(layer_colour, TraceReflectionPathThroughAlpha(surface_reflection_ray, remaining_bounces), hit_reflectivity);
			}

			transparent_colour += transparent_visibility * hit_opacity * layer_colour;
			transparent_visibility *= 1.0f - hit_opacity;
			if (throughput * transparent_visibility <= min_throughput)
			{
				reflection_colour += throughput * transparent_colour;
				return reflection_colour;
			}
			if (transparent_hit + 1u == MaxReflectionTransparentHits)
			{
				reflection_colour += throughput * (transparent_colour + transparent_visibility * ReflectionMissColour(layer_ray.Direction));
				return reflection_colour;
			}

			AdvanceRayThroughTransparentHit(layer_ray, reflection_payload);
		}

		float hit_reflectivity = saturate(reflection_payload.reflectivity);

		// Only reserve energy for the next bounce if it will actually be traced; otherwise fold all remaining energy into the local hit colour.
		bool can_continue = ReflectionCanContinue(bounce, max_bounces, throughput, transparent_visibility, hit_reflectivity, min_throughput, reflection_payload);

		reflection_colour += throughput * (transparent_colour + transparent_visibility * (can_continue ? 1.0f - hit_reflectivity : 1.0f) * reflection_payload.colour);
		if (!can_continue)
			break;

		throughput *= transparent_visibility * hit_reflectivity;
		if (throughput <= min_throughput)
			break;

		reflection_ray = ReflectionRayFromHit(layer_ray, reflection_payload);
	}

	return reflection_colour;
}

// Convert a packed K-buffer depth slot into the linear camera-view Z used when alpha layers were collected.
float AlphaLayerViewZ(uint packed_depth)
{
	uint depth = DepthOf(packed_depth) & ~KBufferTieMask;
	float t = float(depth) / float(KBufferDepthFar);
	return lerp(g_frame.clip.x, g_frame.clip.y, t);
}

// Reconstruct the world position for an alpha K-buffer layer using the packed layer depth and the dispatch pixel.
float3 AlphaLayerWorldPosition(uint2 pixel, uint2 dim, RayDesc camera_ray, uint packed_depth)
{
	float view_z = AlphaLayerViewZ(packed_depth);
	float3 camera_forward = -g_frame.cam.c2w[2].xyz;
	float ray_forward = max(dot(camera_ray.Direction, camera_forward), 1e-6f);
	return camera_ray.Origin + camera_ray.Direction * (view_z / ray_forward);
}

// Fold RT reflection colour into one sorted alpha K-buffer layer while preserving the layer alpha used by the final resolve.
void ApplyAlphaReflectionLayer(inout uint packed_colour, uint packed_depth, uint packed_attrs, uint2 pixel, uint2 dim, RayDesc camera_ray)
{
	if (DepthOf(packed_depth) == KBufferDepthFar)
		return;

	float4 layer_colour = UnpackRGBA8(packed_colour);
	float4 layer_attrs = UnpackRGBA8(packed_attrs);
	float reflectivity = saturate(layer_attrs.a * g_frame.reflection.x);
	if (reflectivity <= 0.0f || layer_colour.a <= 0.0f)
		return;

	float3 normal = 2.0f * layer_attrs.rgb - 1.0f;
	float normal_len_sq = dot(normal, normal);
	if (normal_len_sq < 1e-6f)
		return;

	normal *= rsqrt(normal_len_sq);
	float3 hit_pos = AlphaLayerWorldPosition(pixel, dim, camera_ray, packed_depth);
	RayDesc reflection_ray = MakeSurfaceReflectionRay(camera_ray, hit_pos, normal, g_frame.clip.y);
	float3 reflection_colour = TraceReflectionPath(reflection_ray);
	layer_colour.rgb = lerp(layer_colour.rgb, reflection_colour, reflectivity);
	packed_colour = PackRGBA8(layer_colour);
}

// Update all retained alpha K-buffer layers for this pixel with first-pass baked RT reflections.
void ApplyAlphaReflections(uint2 pixel, uint2 dim)
{
	if ((g_frame.options.w & RayTracingOption_AlphaReflections) == 0)
		return;

	RayDesc camera_ray = MakeCameraRay(pixel, dim);
	uint4 alpha_colour = g_alpha_colour[pixel];
	uint4 alpha_depth = g_alpha_depth[pixel];
	uint4 alpha_rt_attrs = g_alpha_rt_attrs.Load(int3(pixel, 0));

	ApplyAlphaReflectionLayer(alpha_colour.x, alpha_depth.x, alpha_rt_attrs.x, pixel, dim, camera_ray);
	ApplyAlphaReflectionLayer(alpha_colour.y, alpha_depth.y, alpha_rt_attrs.y, pixel, dim, camera_ray);
	ApplyAlphaReflectionLayer(alpha_colour.z, alpha_depth.z, alpha_rt_attrs.z, pixel, dim, camera_ray);
	ApplyAlphaReflectionLayer(alpha_colour.w, alpha_depth.w, alpha_rt_attrs.w, pixel, dim, camera_ray);

	g_alpha_colour[pixel] = alpha_colour;
}

// Return the material table index for the current closest hit.
uint HitMaterialIndex()
{
	return InstanceID() + GeometryIndex();
}

// Return a conservative fallback material for invalid hit-material indices.
RayTracingMaterial DefaultRayTracingMaterial()
{
	RayTracingMaterial material = (RayTracingMaterial)0;
	material.diffuse = float4(0.75f, 0.75f, 0.75f, 1.0f);
	material.optics = float4(0.0f, DefaultGlassTransmission, DefaultGlassIOR, max(g_frame.caustic.w, 0.0f));
	return material;
}

// Return the RT material payload for the hit instance geometry.
RayTracingMaterial HitMaterial()
{
	uint material_count = uint(g_frame.options.y);
	uint material_index = HitMaterialIndex();
	if (material_index < material_count)
		return g_materials[material_index];

	return DefaultRayTracingMaterial();
}

// Return true if 'material' should participate in RT caustic transmission.
bool MaterialIsTransmissive(RayTracingMaterial material)
{
	return (material.flags.x & RayTracingMaterialFlag_Transmissive) != 0;
}

// Return the clamped material reflectivity used for reflection bounces.
float MaterialReflectivity(RayTracingMaterial material)
{
	return saturate(material.optics.x);
}

// Return the clamped material transmission used for caustic energy.
float MaterialTransmission(RayTracingMaterial material)
{
	return saturate(material.optics.y);
}

// Return the opacity used when reflection rays composite through transparent RT hits.
float MaterialOpacity(RayTracingMaterial material, float shaded_alpha)
{
	float alpha_opacity = saturate(shaded_alpha);
	if (alpha_opacity < ReflectionOpaqueAlpha)
		return alpha_opacity;
	if (MaterialIsTransmissive(material))
		return saturate(1.0f - MaterialTransmission(material));

	return 1.0f;
}

// Return a safe material index of refraction.
float MaterialIOR(RayTracingMaterial material)
{
	return max(material.optics.z, 1.0f);
}

// Return a safe material thickness approximation.
float MaterialThickness(RayTracingMaterial material)
{
	return max(material.optics.w, 0.0f);
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

// Shade a reflected hit with the cheap first-pass lighting model used by RT reflections.
float4 ShadeDirectionalHit(float4 ws_normal, float4 colour)
{
	float intensity = LightDirectional(g_frame.global_light.ws_direction, ws_normal, colour.a);
	float3 lit = g_frame.global_light.ambient.rgb + intensity * g_frame.global_light.colour.rgb;
	float3 diffuse = 2.0f * (lit - 0.5f) * colour.rgb;
	return saturate(float4(diffuse, 0.0f) + colour);
}

// Shade the closest-hit surface by interpolating packed geometry attributes.
float4 ShadeRayHit(in BuiltInTriangleIntersectionAttributes attrib, RayTracingMaterial material, out float3 ws_normal_out, out float reflectivity)
{
	ws_normal_out = 0.0f;

	uint geometry_index = HitMaterialIndex();
	reflectivity = MaterialReflectivity(material);
	if (!HasHitGeometry(geometry_index))
		return material.diffuse;

	// Geometry sidecar data is optional because fallback BLAS geometry can still be ray-hit even if it cannot be shaded from original vertices.
	RayTracingGeometry geometry = g_geometry[geometry_index];
	uint first_index = PrimitiveIndex() * 3u;
	if (first_index + 2u >= geometry.ranges.w)
		return material.diffuse;

	uint i0 = HitTriangleIndex(geometry, 0) - geometry.ranges.y;
	uint i1 = HitTriangleIndex(geometry, 1) - geometry.ranges.y;
	uint i2 = HitTriangleIndex(geometry, 2) - geometry.ranges.y;
	if (i0 >= geometry.flags.y || i1 >= geometry.flags.y || i2 >= geometry.flags.y)
		return material.diffuse;

	float3 bary = float3(1.0f - attrib.barycentrics.x - attrib.barycentrics.y, attrib.barycentrics.x, attrib.barycentrics.y);

	RayTracingVertex v0 = g_vertices[geometry.ranges.x + i0];
	RayTracingVertex v1 = g_vertices[geometry.ranges.x + i1];
	RayTracingVertex v2 = g_vertices[geometry.ranges.x + i2];
	float4 colour = (bary.x * v0.colour + bary.y * v1.colour + bary.z * v2.colour) * material.diffuse;

	// Without packed normals the hit can still contribute diffuse colour, but it cannot spawn another reflection bounce safely.
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

	// Degenerate normals fall back to a terminal hit colour rather than risking NaNs in later reflection rays.
	if (ws_normal_len_sq < 1e-10f)
		return colour;

	ws_normal *= rsqrt(ws_normal_len_sq);
	if (dot(ws_normal.xyz, -WorldRayDirection()) < 0.0f)
		ws_normal = -ws_normal;

	ws_normal_out = ws_normal.xyz;

	if (DirectionalLight(g_frame.global_light))
		return ShadeDirectionalHit(ws_normal, colour);

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
	// Build a stable 2D basis in the receiver plane so the procedural caustic pattern follows the light direction rather than screen space.
	float3 up = abs(surface_to_light.z) < 0.9f ? float3(0.0f, 0.0f, 1.0f) : float3(0.0f, 1.0f, 0.0f);
	float3 axis_x = normalize(cross(up, surface_to_light));
	float3 axis_y = cross(surface_to_light, axis_x);
	float2 p = float2(dot(ws_pos, axis_x), dot(ws_pos, axis_y)) * max(g_frame.caustic.z, 0.001f);

	// Combine two rotated cell layers to avoid an obvious grid pattern while keeping the shader cheap enough for the current prototype.
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

// Return a refracted direction for the receiver-pull caustic path, or the incident direction if the approximation cannot refract safely.
float3 CausticRefractedDirection(float3 incident, float3 material_normal, float eta)
{
	float normal_len_sq = dot(material_normal, material_normal);
	if (normal_len_sq < 1e-6f)
		return incident;

	material_normal *= rsqrt(normal_len_sq);
	if (dot(material_normal, incident) > 0.0f)
		material_normal = -material_normal;

	float3 refracted = refract(incident, material_normal, eta);
	float refracted_len_sq = dot(refracted, refracted);
	if (refracted_len_sq < 1e-6f)
		return incident;

	return refracted * rsqrt(refracted_len_sq);
}

// Approximate how much light survives the transmissive material hit found by the caustic ray.
float CausticTransmission(float3 receiver_to_light, float3 material_normal, float material_transmission, float material_ior)
{
	float normal_len_sq = dot(material_normal, material_normal);
	float facing = 1.0f;
	if (normal_len_sq >= 1e-6f)
	{
		material_normal *= rsqrt(normal_len_sq);
		facing = abs(dot(receiver_to_light, material_normal));
	}

	float f0 = pow((material_ior - 1.0f) / max(material_ior + 1.0f, 1.0f), 2.0f);
	float fresnel_reflectance = f0 + (1.0f - f0) * pow(1.0f - saturate(facing), 5.0f);
	return saturate(material_transmission) * (1.0f - fresnel_reflectance);
}

// Dispatch one screen-space RT query per output pixel and composite the selected RT feature over the raster colour.
[shader("raygeneration")]
void RayGen()
{
	uint2 pixel = DispatchRaysIndex().xy;
	uint2 dim = DispatchRaysDimensions().xy;
	uint mode = g_frame.options.x;

	RayPayload payload = InitRayPayload(RayPayloadMode_Diagnostic, float3(0.02f, 0.03f, 0.05f));

	// Diagnostic mode ignores the raster image and traces a camera ray directly into the TLAS to visualise ray tracing coverage.
	if (mode == RayTracingMode_Diagnostic)
	{
		RayDesc ray = MakeCameraRay(pixel, dim);
		TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE, RayTracingInstanceMask_All, 0, 1, 0, ray, payload);
		g_output[pixel] = float4(payload.colour, 1.0f);
		return;
	}

	float4 colour = g_input.Load(int3(pixel, 0));

	// Reflection mode starts from the resolved raster colour and only traces pixels whose raster material exported reflection attributes.
	if (RayTracingModeIncludesReflections(mode))
	{
		RasterDepth raster = LoadRasterDepth(pixel);
		if (raster.depth < 0.999999f)
		{
			float4 reflection_attrs = g_reflection_attrs.Load(pixel, raster.sample);
			float reflectivity = saturate(reflection_attrs.a * g_frame.reflection.x);
			if (reflectivity > 0.0f)
			{
				// The first reflection ray is seeded from raster data so opaque raster shading remains the primary camera-visible path.
				RayDesc camera_ray = MakeCameraRay(pixel, dim);
				float3 hit_pos = WorldPosition(pixel, dim, raster.depth);
				float3 normal = normalize(2.0f * reflection_attrs.xyz - 1.0f);
				RayDesc reflection_ray = MakeSurfaceReflectionRay(camera_ray, hit_pos, normal, g_frame.clip.y);
				colour.rgb = lerp(colour.rgb, TraceReflectionPath(reflection_ray), reflectivity);
			}
		}

		ApplyAlphaReflections(pixel, dim);

		// Reflection-only dispatches are complete here. Combined mode deliberately falls through so caustics add to the reflected raster result.
		if (mode == RayTracingMode_Reflections)
		{
			g_output[pixel] = colour;
			return;
		}
	}

	// Caustics mode adds light to the raster colour. It uses raster depth as the receiver and RT only to find transmissive blockers along the light path.
	if (RayTracingModeIncludesCaustics(mode))
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
				// Receiver-pull caustics trace from the opaque receiver toward the light. Any transmissive RT hit on that segment supplies the material
				// properties used to estimate how light bends and focuses before reaching this receiver.
				RayPayload caustic_payload = InitOcclusionPayload(RayPayloadMode_Caustic);
				float caustic_bias = RayBias(g_frame.caustic.y, length(hit_pos - camera_ray.Origin));
				RayDesc caustic_ray = MakeSecondaryRay(hit_pos + surface_to_light * caustic_bias, surface_to_light, g_frame.clip.y);

				TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, RayTracingInstanceMask_Caustic, 0, 1, 0, caustic_ray, caustic_payload);
				if (caustic_payload.occluded != 0)
				{
					// Treat the hit material as a thin refractive layer and modulate a projected proof pattern by Fresnel/transmission terms.
					float material_transmission = saturate(caustic_payload.colour.x);
					float material_ior = max(caustic_payload.colour.y, 1.0f);
					float material_thickness = max(caustic_payload.colour.z, 0.0f);
					float3 material_normal = PayloadHasSurfaceNormal(caustic_payload) ? caustic_payload.normal : normal;
					float material_distance = max(caustic_payload.hit_t, 0.0f);
					float3 material_pos = caustic_ray.Origin + caustic_ray.Direction * material_distance;
					float3 entry_dir = CausticRefractedDirection(surface_to_light, material_normal, 1.0f / material_ior);
					float entry_bend = saturate(length(entry_dir - surface_to_light));
					float3 caustic_dir = entry_dir;
					float3 pattern_pos = material_pos + caustic_dir * material_thickness;
					float optical_thickness = material_thickness;
					float path_alignment = 0.65f + 0.35f * pow(saturate(dot(caustic_dir, surface_to_light)), 2.0f);

					// A second caustic probe gives closed transparent shapes, such as spheres, a cheap entry/exit refraction approximation instead of a flat
					// one-surface projection.
					if (entry_bend > 1e-4f)
					{
						RayPayload exit_payload = InitOcclusionPayload(RayPayloadMode_Caustic);
						RayDesc exit_ray = MakeSecondaryRay(material_pos + entry_dir * caustic_bias, entry_dir, g_frame.clip.y);

						TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, RayTracingInstanceMask_Caustic, 0, 1, 0, exit_ray, exit_payload);
						if (exit_payload.occluded != 0)
						{
							float3 exit_normal = PayloadHasSurfaceNormal(exit_payload)
								? exit_payload.normal
								: -material_normal;
							float exit_distance = max(exit_payload.hit_t, 0.0f);
							float3 exit_pos = exit_ray.Origin + exit_ray.Direction * exit_distance;
							caustic_dir = CausticRefractedDirection(entry_dir, exit_normal, material_ior);
							optical_thickness = max(material_thickness, exit_distance);
							pattern_pos = exit_pos + caustic_dir * material_thickness;
							path_alignment = 0.25f + 0.75f * pow(saturate(dot(caustic_dir, surface_to_light)), 4.0f);

							// View3D alpha is a through-object visibility hint rather than a physical per-surface absorption coefficient. Do not square it for
							// the paired entry/exit hit, otherwise normal semi-transparent objects contribute almost no caustic energy.
							material_transmission = max(material_transmission, saturate(exit_payload.colour.x));
						}
					}

					float pattern = CausticPattern(pattern_pos, caustic_dir);
					float caustic_transmission = CausticTransmission(surface_to_light, material_normal, material_transmission, material_ior);
					float focus = 1.0f + 4.0f * saturate(entry_bend * optical_thickness * 3.0f);
					float strength = g_frame.caustic.x *
						receiver_facing *
						lerp(0.35f, 1.0f, pattern) *
						caustic_transmission *
						focus *
						path_alignment;
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

	// Hard-shadow mode is the fallback RT path. It traces a visibility ray from the raster receiver toward the directional light.
	float depth = LoadDepth(pixel);
	if (depth < 0.999999f)
	{
		float3 light_to_surface = normalize(g_frame.global_light.ws_direction.xyz);
		float3 surface_to_light = -light_to_surface;
		float3 hit_pos = WorldPosition(pixel, dim, depth);
		float shadow_bias = RayBias(g_frame.shadow.y, length(hit_pos - g_frame.cam.c2w[3].xyz));

		RayPayload shadow_payload = InitOcclusionPayload(RayPayloadMode_Shadow);
		RayDesc shadow_ray = MakeSecondaryRay(hit_pos + surface_to_light * shadow_bias, surface_to_light, g_frame.clip.y);

		TraceRay(
			g_scene,
			RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
			RayTracingInstanceMask_All,
			0, 1, 0,
			shadow_ray,
			shadow_payload);
		if (shadow_payload.occluded != 0)
			colour.rgb *= saturate(1.0f - g_frame.shadow.x);
	}

	g_output[pixel] = colour;
}

// Handle ray misses for each payload mode.
[shader("miss")]
void Miss(inout RayPayload payload)
{
	// Visibility-style rays treat a miss as unoccluded; colour-style rays either use their caller's miss colour or the diagnostic fallback.
	if (PayloadModeIsVisibilityQuery(payload.mode))
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
	payload.colour = float3(0.02f, 0.03f, 0.05f);
}

// Handle closest hits for reflection shading, caustic material queries, visibility queries, and diagnostics.
[shader("closesthit")]
void ClosestHit(inout RayPayload payload, in BuiltInTriangleIntersectionAttributes attrib)
{
	if (payload.mode == RayPayloadMode_Reflection)
	{
		// Reflection rays need enough payload to either shade a terminal hit or seed the next bounce. Opacity is carried separately from colour so the
		// raygen traversal can composite transparent hits before deciding whether an opaque bounce candidate remains.
		RayTracingMaterial material = HitMaterial();
		float3 ws_normal;
		float reflectivity;
		float4 colour = ShadeRayHit(attrib, material, ws_normal, reflectivity);

		payload.hit = 1;
		payload.hit_t = RayTCurrent();
		payload.colour = colour.rgb;
		payload.normal = ws_normal;
		payload.reflectivity = reflectivity;
		payload.opacity = MaterialOpacity(material, colour.a);
		payload.flags = dot(ws_normal, ws_normal) > 0.0f
			? RayPayloadFlag_HasSurfaceNormal | (reflectivity > 0.0f ? RayPayloadFlag_ReflectionValid : 0)
			: 0;
		return;
	}
	if (payload.mode == RayPayloadMode_Caustic)
	{
		// Caustic rays only care about transmissive materials. Opaque hits clear the occlusion flag so raygen knows not to add caustic energy at this
		// receiver, while transmissive hits encode transmission/IOR/thickness into the colour payload.
		RayTracingMaterial material = HitMaterial();
		if (!MaterialIsTransmissive(material))
		{
			payload.hit = 0;
			payload.occluded = 0;
			return;
		}

		float3 ws_normal;
		float reflectivity;
		ShadeRayHit(attrib, material, ws_normal, reflectivity);

		payload.hit = 1;
		payload.hit_t = RayTCurrent();
		payload.occluded = 1;
		payload.normal = ws_normal;
		payload.flags = dot(ws_normal, ws_normal) > 0.0f
			? RayPayloadFlag_HasSurfaceNormal
			: 0;
		payload.colour = float3(MaterialTransmission(material), MaterialIOR(material), MaterialThickness(material));
		return;
	}
	if (payload.mode != RayPayloadMode_Diagnostic)
	{
		// Shadow rays skip closest-hit today, but keep non-diagnostic hits conservative if another visibility mode reaches this shader.
		payload.hit = 1;
		return;
	}

	// Diagnostic mode shades each hit with a stable object/primitive colour and a small wire overlay to show triangle coverage.
	uint seed = InstanceIndex() ^ (PrimitiveIndex() * 747796405u);
	float edge = min(attrib.barycentrics.x, min(attrib.barycentrics.y, 1.0f - attrib.barycentrics.x - attrib.barycentrics.y));
	float wire = smoothstep(0.0f, 0.02f, edge);
	payload.colour = lerp(float3(0.02f, 0.02f, 0.02f), DiagnosticColour(seed), wire);
}

