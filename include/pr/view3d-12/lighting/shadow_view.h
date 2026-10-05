//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/lighting/light.h"

namespace pr::rdr12
{
	// The maximum number of shadow views in a frame. Must match 'MaxShadowViews' in lighting_cbuf.hlsli.
	inline static constexpr int MaxShadowViews = 32;

	// The number of shadow views that can be rendered in one draw call. Limited by the number of viewports a pipeline can use.
	inline static constexpr int ShadowViewBatchSize = 16;

	// The maximum number of cascades for a directional light. Must match 'MaxShadowCascades' in lighting_cbuf.hlsli.
	inline static constexpr int MaxShadowCascades = 4;

	// Scene-wide shadow rendering settings
	struct ShadowSettings
	{
		int   m_atlas_size;             // Width and height of the square shadow atlas (in pixels)
		int   m_directional_resolution; // Requested size of each directional light cascade view (in pixels)
		int   m_spot_resolution;        // Largest size of a spot light shadow view (in pixels). Smaller sizes are used for lights that cover less of the screen
		int   m_point_resolution;       // Largest size of each of the six point light shadow views (in pixels). Smaller sizes are used for lights that cover less of the screen
		int   m_max_shadow_lights;      // The maximum number of lights that cast shadows. Zero disables shadows
		int   m_cascade_count;          // The number of cascades for directional lights, in [1, MaxShadowCascades]
		float m_shadow_distance;        // Distance from the camera beyond which directional lights cast no shadows. Shadows fade out over the last 10% of this distance. Zero means fit to the shadow casters
		float m_cascade_split_blend;    // Cascade split distribution in [0,1]. 0 = even spacing, 1 = logarithmic spacing (more detail near the camera)
		int   m_filter_size;            // Width of the shadow edge filter (in shadow texels). Either 5 or 7
		int   m_depth_bias;             // Constant depth bias applied when rendering shadow depth (in units of the smallest depth step)
		float m_slope_bias;             // Depth bias scaled by the depth slope of each triangle
		float m_normal_bias;            // Distance to move receiver positions along their normal before the shadow test (in shadow texels)
		bool  m_cache_views;            // Re-render shadow views only when their content changes. False re-renders every view every frame

		ShadowSettings()
			:m_atlas_size(4096)
			,m_directional_resolution(1024)
			,m_spot_resolution(1024)
			,m_point_resolution(1024)
			,m_max_shadow_lights(4)
			,m_cascade_count(3)
			,m_shadow_distance(0.0f)
			,m_cascade_split_blend(0.9f)
			,m_filter_size(5)
			,m_depth_bias(0)
			,m_slope_bias(2.0f)
			,m_normal_bias(1.0f)
			,m_cache_views(true)
		{}

		friend bool operator == (ShadowSettings const&, ShadowSettings const&) = default;
	};

	// The camera properties that shadow view selection depends on
	struct ShadowCamera
	{
		m4x4  m_c2w;             // Camera to world transform. The camera looks down its -Z axis
		float m_near;            // Distance to the near clip plane
		float m_far;             // Distance to the far clip plane
		v2    m_view_size;       // Perspective: width and height of the view at unit distance. Orthographic: width and height of the view
		float m_viewport_height; // Height of the rendered image (in pixels). Zero means unknown, which gives views their largest size
		bool  m_orthographic;    // True for an orthographic projection

		// Width and height of the view at 'dist' from the camera
		v2 ViewSizeAt(float dist) const
		{
			return m_orthographic ? m_view_size : m_view_size * dist;
		}
	};

	// One depth render of the scene from a shadow-casting light, stored in a region of the shadow atlas
	struct ShadowView
	{
		m4x4  m_w2s;         // World space to clip space transform (D3D clip space, depth in [0,1])
		IRect m_atlas_rect;  // The region of the atlas that holds this view (in pixels)
		float m_normal_bias; // Receiver normal offset. World units for orthographic views, world units per unit distance from the light for perspective views
		v2    m_fade_depth;  // Distances from the camera (start, end) over which the shadow fades to fully lit. An end of zero means no fade
		int   m_light_index; // Index of the light (in the resolved light list) that this view belongs to
		int   m_face;        // Cube face index for point lights (+X,-X,+Y,-Y,+Z,-Z), cascade index for directional lights, otherwise 0
		bool  m_clamp_depth; // True if casters in front of the near plane are rendered at depth 0 instead of being clipped (orthographic views)
	};

	// The shadow views for a frame
	struct ShadowViewSet
	{
		pr::vector<ShadowView, MaxShadowViews, true> m_views; // All shadow views. Views of the same light are contiguous
		pr::vector<iv2, 8> m_light_views;                     // Per resolved light: (first view index, view count). (-1, 0) for lights without shadows

		// True if there are no shadow views
		bool empty() const
		{
			return m_views.empty();
		}

		// Reset to no views for 'light_count' lights
		void reset(int light_count);
	};

	// Choose the shadow views for 'lights' and pack them into the atlas. 'caster_bounds' is the world space bounds of all shadow casters.
	// Lights are considered in order. At most 'settings.m_max_shadow_lights' lights and 'MaxShadowViews' views are used.
	// Directional lights get cascades that cover the part of the camera view containing casters. Each cascade is snapped to whole shadow
	// texels so shadows do not shimmer when the camera moves. Point and spot light views are sized by how much of the screen the light covers.
	void BuildShadowViews(std::span<Light const> lights, BBox const& caster_bounds, ShadowCamera const& camera, ShadowSettings const& settings, ShadowViewSet& out);

	// Return the distances from the camera to the cascade boundaries for 'count' cascades over [zn, zf]. 'out' receives 'count + 1' values.
	// 'blend' mixes even spacing (0) with logarithmic spacing (1).
	void ShadowCascadeSplits(float zn, float zf, int count, float blend, std::span<float> out);

	// Assign square atlas regions for views with the requested sizes. Sizes are rounded down to powers of two and clamped to the atlas size.
	// When the views do not fit, the largest views are halved until they do. The result is deterministic for the same inputs.
	void PackShadowAtlas(std::span<int const> requested_sizes, int atlas_size, std::span<IRect> out);

	// Return the point light cube face index (+X,-X,+Y,-Y,+Z,-Z) that contains the direction 'light_to_point'
	int ShadowCubeFace(v4 light_to_point);

	// The world space clip volume of a shadow view, used to find the boxes that the view may see
	struct ShadowViewVolume
	{
		v4  m_planes[6];    // World space planes facing into the volume. The near plane is last so it can be excluded
		int m_plane_count;  // The number of planes in use. 5 when the near plane is excluded

		// Build the volume from the clip space defined by 'w2s'.
		// 'near_plane' is false for views that clamp depth, where boxes in front of the near plane still cast shadows.
		explicit ShadowViewVolume(m4x4 const& w2s, bool near_plane = true);

		// True if the world space box 'bbox' may be visible in the volume
		bool Sees(BBox const& bbox) const;

		// True if the world space box with corners 'lower' and 'upper' may be visible in the volume
		bool Sees(v4 lower, v4 upper) const;
	};

	// Tracks the content of each shadow atlas region so that unchanged views are not rendered again
	struct ShadowViewCache
	{
		// A region of the atlas and a hash of the content rendered into it
		struct Entry
		{
			IRect    m_rect;
			uint64_t m_hash;
		};
		pr::vector<Entry, MaxShadowViews, true> m_entries; // The atlas content at the end of the last rendered frame

		// Return a bit mask of the views in 'rects'/'hashes' whose atlas region does not already hold the same content, then record the new content.
		// Every region is dirty when 'enabled' is false.
		uint32_t Update(std::span<IRect const> rects, std::span<uint64_t const> hashes, bool enabled);

		// Forget all atlas content, so every view is rendered next frame
		void Invalidate();
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::tests
{
	// A 90 degree perspective camera at 'pos' looking toward the origin, rendering an image 1000 pixels high
	inline ShadowCamera TestCamera(v4 pos, float near_)
	{
		return ShadowCamera{
			.m_c2w = m4x4::LookAt(pos, v4::Origin(), Perpendicular(pos - v4::Origin(), v4::YAxis())),
			.m_near = near_,
			.m_far = 10000.0f,
			.m_view_size = v2(2, 2),
			.m_viewport_height = 1000.0f,
			.m_orthographic = false,
		};
	}

	PRUnitTest(ShadowViewTests, Quick)
	{
		// Packed views keep their power-of-two sizes, stay in the atlas, and do not overlap.
		{
			int sizes[] = { 1024, 2048, 1024, 1000, 512 };
			IRect rects[_countof(sizes)];
			PackShadowAtlas(sizes, 4096, rects);

			int expected[] = { 1024, 2048, 1024, 512, 512 };
			for (int i = 0; i != _countof(sizes); ++i)
			{
				PR_EXPECT(rects[i].SizeX() == expected[i]);
				PR_EXPECT(rects[i].SizeY() == expected[i]);
				PR_EXPECT(rects[i].m_min.x >= 0 && rects[i].m_max.x <= 4096);
				PR_EXPECT(rects[i].m_min.y >= 0 && rects[i].m_max.y <= 4096);
				for (int j = 0; j != i; ++j)
				{
					auto overlap =
						rects[i].m_min.x < rects[j].m_max.x && rects[j].m_min.x < rects[i].m_max.x &&
						rects[i].m_min.y < rects[j].m_max.y && rects[j].m_min.y < rects[i].m_max.y;
					PR_EXPECT(!overlap);
				}
			}
		}

		// Views that do not fit are halved, largest first.
		{
			int sizes[] = { 4096, 4096, 1024 };
			IRect rects[_countof(sizes)];
			PackShadowAtlas(sizes, 4096, rects);
			PR_EXPECT(rects[0].SizeX() == 2048);
			PR_EXPECT(rects[1].SizeX() == 2048);
			PR_EXPECT(rects[2].SizeX() == 1024);
		}

		// Cube face selection uses the dominant axis.
		{
			PR_EXPECT(ShadowCubeFace(v4(+2, 1, 1, 0)) == 0);
			PR_EXPECT(ShadowCubeFace(v4(-2, 1, 1, 0)) == 1);
			PR_EXPECT(ShadowCubeFace(v4(1, +2, 1, 0)) == 2);
			PR_EXPECT(ShadowCubeFace(v4(1, -2, 1, 0)) == 3);
			PR_EXPECT(ShadowCubeFace(v4(1, 1, +2, 0)) == 4);
			PR_EXPECT(ShadowCubeFace(v4(1, 1, -2, 0)) == 5);
		}

		// Frustum culling rejects boxes outside the view volume.
		{
			auto l2w = m4x4::LookAt(v4(0, 0, 10, 1), v4::Origin(), v4::YAxis());
			auto w2s = m4x4::ProjectionPerspective(1.0f, 1.0f, 1.0f, 20.0f, true) * InvertOrthonormal(l2w);
			PR_EXPECT(ShadowViewVolume(w2s).Sees(BBox(v4::Origin(), v4(1, 1, 1, 0))));
			PR_EXPECT(!ShadowViewVolume(w2s).Sees(BBox(v4(0, 0, 20, 1), v4(1, 1, 1, 0))));
			PR_EXPECT(!ShadowViewVolume(w2s).Sees(BBox(v4(50, 0, 0, 1), v4(1, 1, 1, 0))));
			PR_EXPECT(!ShadowViewVolume(w2s).Sees(BBox(v4(0, 0, -30, 1), v4(1, 1, 1, 0))));
		}

		// View selection: point lights use six views, non-casters get none, and the light limit applies.
		{
			Light dir_light;
			dir_light.m_cast_shadow = 1.0f;
			Light point_light;
			point_light.m_type = ELight::Point;
			point_light.m_position = v4(0, 5, 0, 1);
			point_light.m_cast_shadow = 0.5f;
			Light no_shadow;
			Light spot_light;
			spot_light.m_type = ELight::Spot;
			spot_light.m_position = v4(0, 5, 0, 1);
			spot_light.m_direction = v4(0, -1, 0, 0);
			spot_light.m_cast_shadow = 1.0f;

			Light lights[] = { dir_light, point_light, no_shadow, spot_light };
			auto bounds = BBox(v4::Origin(), v4(2, 2, 2, 0));

			// A camera that sees the whole caster sphere, so the directional light needs only one view
			auto camera = TestCamera(v4(0, 0, 20, 1), 1.0f);

			ShadowSettings settings;
			ShadowViewSet set;
			BuildShadowViews(lights, bounds, camera, settings, set);
			PR_EXPECT(set.m_views.size() == 8);
			PR_EXPECT(set.m_light_views.size() == 4);
			PR_EXPECT(All(set.m_light_views[0] == iv2(0, 1)));
			PR_EXPECT(All(set.m_light_views[1] == iv2(1, 6)));
			PR_EXPECT(All(set.m_light_views[2] == iv2(-1, 0)));
			PR_EXPECT(All(set.m_light_views[3] == iv2(7, 1)));
			PR_EXPECT(set.m_views[0].m_clamp_depth);
			PR_EXPECT(!set.m_views[1].m_clamp_depth);
			for (int i = 0; i != 6; ++i)
				PR_EXPECT(set.m_views[1 + i].m_face == i);

			// The caster bounds centre is in front of each view that should see it
			PR_EXPECT(ShadowViewVolume(set.m_views[0].m_w2s, false).Sees(bounds));
			PR_EXPECT(ShadowViewVolume(set.m_views[7].m_w2s).Sees(bounds));
			PR_EXPECT(ShadowViewVolume(set.m_views[1 + ShadowCubeFace(v4(0, -1, 0, 0))].m_w2s).Sees(BBox(v4::Origin(), v4(0.1f, 0.1f, 0.1f, 0))));

			settings.m_max_shadow_lights = 1;
			BuildShadowViews(lights, bounds, camera, settings, set);
			PR_EXPECT(set.m_views.size() == 1);
			PR_EXPECT(All(set.m_light_views[1] == iv2(-1, 0)));

			settings.m_max_shadow_lights = 0;
			BuildShadowViews(lights, bounds, camera, settings, set);
			PR_EXPECT(set.empty());
		}

		// A point light that is out of range of the casters has no views.
		{
			Light point_light;
			point_light.m_type = ELight::Point;
			point_light.m_position = v4(0, 50, 0, 1);
			point_light.m_range = 10.0f;
			point_light.m_cast_shadow = 1.0f;

			ShadowViewSet set;
			BuildShadowViews({ &point_light, 1 }, BBox(v4::Origin(), v4(1, 1, 1, 0)), TestCamera(v4(0, 0, 20, 1), 1.0f), ShadowSettings{}, set);
			PR_EXPECT(set.empty());
			PR_EXPECT(All(set.m_light_views[0] == iv2(-1, 0)));
		}

		// Cascade splits cover the range in increasing order, and blend between even and logarithmic spacing
		{
			float splits[4];
			ShadowCascadeSplits(1.0f, 1000.0f, 3, 0.0f, splits);
			PR_EXPECT(FEql(splits[0], 1.0f) && FEql(splits[1], 334.0f) && FEql(splits[2], 667.0f) && FEql(splits[3], 1000.0f));
			ShadowCascadeSplits(1.0f, 1000.0f, 3, 1.0f, splits);
			PR_EXPECT(FEql(splits[1], 10.0f) && FEql(splits[2], 100.0f) && FEql(splits[3], 1000.0f));
			ShadowCascadeSplits(1.0f, 1000.0f, 3, 0.5f, splits);
			PR_EXPECT(FEql(splits[1], 172.0f) && FEql(splits[2], 383.5f));
		}

		// Directional lights get cascades when the casters extend beyond what the first cascade covers.
		// Cascades are nested in size, and the camera near the origin is inside the first cascade.
		{
			Light sun;
			sun.m_direction = Normalise(v4(1, -2, -1, 0));
			sun.m_cast_shadow = 1.0f;
			auto bounds = BBox(v4::Origin(), v4(500, 500, 5, 0));
			auto camera = TestCamera(v4(0, -10, 5, 1), 0.1f);

			ShadowSettings settings;
			ShadowViewSet set;
			BuildShadowViews({ &sun, 1 }, bounds, camera, settings, set);
			PR_EXPECT(All(set.m_light_views[0] == iv2(0, 3)));
			for (int i = 0; i != 3; ++i)
			{
				PR_EXPECT(set.m_views[i].m_face == i);
				PR_EXPECT(set.m_views[i].m_clamp_depth);
			}

			// Cascade world sizes increase. The x scale of an orthographic w2s is 2 / width.
			PR_EXPECT(set.m_views[0].m_w2s.x.x * set.m_views[0].m_w2s.x.x + set.m_views[0].m_w2s.y.x * set.m_views[0].m_w2s.y.x + set.m_views[0].m_w2s.z.x * set.m_views[0].m_w2s.z.x >
			          set.m_views[1].m_w2s.x.x * set.m_views[1].m_w2s.x.x + set.m_views[1].m_w2s.y.x * set.m_views[1].m_w2s.y.x + set.m_views[1].m_w2s.z.x * set.m_views[1].m_w2s.z.x);

			// A point just in front of the camera projects inside the first cascade
			auto p = set.m_views[0].m_w2s * (camera.m_c2w.pos - 1.0f * camera.m_c2w.z);
			PR_EXPECT(Abs(p.x) < 1 && Abs(p.y) < 1 && p.z > 0 && p.z < 1);

			// A single cascade setting gives one view
			settings.m_cascade_count = 1;
			BuildShadowViews({ &sun, 1 }, bounds, camera, settings, set);
			PR_EXPECT(All(set.m_light_views[0] == iv2(0, 1)));

			// A shadow distance shorter than the camera's distance to all casters gives no views
			settings.m_cascade_count = 3;
			settings.m_shadow_distance = 1.0f;
			BuildShadowViews({ &sun, 1 }, BBox(v4(0, 100, 0, 1), v4(1, 1, 1, 0)), camera, settings, set);
			PR_EXPECT(set.empty());
		}

		// Cascade projections are stable under sub-texel camera moves. Snapping keeps the shadow texel grid fixed in world space,
		// so a world point maps to the same fractional texel position after the camera moves by less than a texel.
		{
			Light sun;
			sun.m_direction = Normalise(v4(1, -2, -1, 0));
			sun.m_cast_shadow = 1.0f;
			auto bounds = BBox(v4::Origin(), v4(500, 500, 5, 0));
			ShadowSettings settings;

			// Texel position of a world point in view 'v'
			auto texel_of = [](ShadowView const& v, v4 ws)
			{
				auto ss = v.m_w2s * ws;
				return v2((0.5f + 0.5f * ss.x) * v.m_atlas_rect.SizeX(), (0.5f - 0.5f * ss.y) * v.m_atlas_rect.SizeY());
			};

			ShadowViewSet set0, set1;
			auto cam0 = TestCamera(v4(0, -10, 5, 1), 0.1f);
			auto cam1 = cam0;
			cam1.m_c2w.pos += v4(0.0013f, 0.0007f, 0, 0);
			BuildShadowViews({ &sun, 1 }, bounds, cam0, settings, set0);
			BuildShadowViews({ &sun, 1 }, bounds, cam1, settings, set1);
			PR_EXPECT(set0.m_views.size() == set1.m_views.size());
			for (int i = 0; i != isize(set0.m_views); ++i)
			{
				// Texel positions differ by whole texels only
				auto ws = v4(0.37f, 0.21f, 0.5f, 1);
				auto d = texel_of(set1.m_views[i], ws) - texel_of(set0.m_views[i], ws);
				PR_EXPECT(FEqlAbsolute(d.x, std::round(d.x), 0.01f));
				PR_EXPECT(FEqlAbsolute(d.y, std::round(d.y), 0.01f));
			}
		}

		// Point and spot view sizes shrink with the light's screen coverage, in powers of two up to the setting
		{
			Light spot;
			spot.m_type = ELight::Spot;
			spot.m_direction = v4(0, 0, -1, 0);
			spot.m_range = 5.0f;
			spot.m_cast_shadow = 1.0f;
			auto bounds = BBox(v4::Origin(), v4(1, 1, 1, 0));
			ShadowSettings settings;
			ShadowViewSet set;

			// Near the camera, the light covers the screen and gets the largest size
			spot.m_position = v4(0, 0, 3, 1);
			BuildShadowViews({ &spot, 1 }, bounds, TestCamera(v4(0, 0, 5, 1), 0.1f), settings, set);
			PR_EXPECT(set.m_views[0].m_atlas_rect.SizeX() == 1024);

			// Far from the camera, the light covers a few pixels and gets a small power-of-two size
			BuildShadowViews({ &spot, 1 }, bounds, TestCamera(v4(0, 0, 2000, 1), 0.1f), settings, set);
			auto size = set.m_views[0].m_atlas_rect.SizeX();
			PR_EXPECT(size < 1024 && std::has_single_bit(static_cast<uint32_t>(size)));
		}

		// The view cache reports regions as dirty only when their content changes
		{
			ShadowViewCache cache;
			IRect rects[] = { IRect(0, 0, 512, 512), IRect(512, 0, 1024, 512) };
			uint64_t hashes[] = { 1, 2 };
			PR_EXPECT(cache.Update(rects, hashes, true) == 0b11);
			PR_EXPECT(cache.Update(rects, hashes, true) == 0b00);

			// Changed content in one region
			hashes[1] = 3;
			PR_EXPECT(cache.Update(rects, hashes, true) == 0b10);

			// Views that swap regions are both dirty
			IRect swapped[] = { rects[1], rects[0] };
			PR_EXPECT(cache.Update(swapped, hashes, true) == 0b11);

			// Disabled caching and invalidation make every region dirty
			PR_EXPECT(cache.Update(swapped, hashes, false) == 0b11);
			cache.Invalidate();
			PR_EXPECT(cache.Update(swapped, hashes, true) == 0b11);
			PR_EXPECT(cache.Update(swapped, hashes, true) == 0b00);
		}
	}
}
#endif
