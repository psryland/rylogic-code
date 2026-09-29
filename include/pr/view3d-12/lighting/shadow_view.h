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

	// Scene-wide shadow rendering settings
	struct ShadowSettings
	{
		int   m_atlas_size;             // Width and height of the square shadow atlas (in pixels)
		int   m_directional_resolution; // Requested size of a directional light shadow view (in pixels)
		int   m_spot_resolution;        // Requested size of a spot light shadow view (in pixels)
		int   m_point_resolution;       // Requested size of each of the six point light shadow views (in pixels)
		int   m_max_shadow_lights;      // The maximum number of lights that cast shadows. Zero disables shadows
		int   m_depth_bias;             // Constant depth bias applied when rendering shadow depth (in units of the smallest depth step)
		float m_slope_bias;             // Depth bias scaled by the depth slope of each triangle
		float m_normal_bias;            // Distance to move receiver positions along their normal before the shadow test (in shadow texels)

		ShadowSettings()
			:m_atlas_size(4096)
			,m_directional_resolution(2048)
			,m_spot_resolution(1024)
			,m_point_resolution(1024)
			,m_max_shadow_lights(4)
			,m_depth_bias(100)
			,m_slope_bias(2.0f)
			,m_normal_bias(1.0f)
		{}

		friend bool operator == (ShadowSettings const&, ShadowSettings const&) = default;
	};

	// One depth render of the scene from a shadow-casting light, stored in a region of the shadow atlas
	struct ShadowView
	{
		m4x4  m_w2s;         // World space to clip space transform (D3D clip space, depth in [0,1])
		IRect m_atlas_rect;  // The region of the atlas that holds this view (in pixels)
		float m_normal_bias; // Receiver normal offset. World units for orthographic views, world units per unit distance from the light for perspective views
		int   m_light_index; // Index of the light (in the resolved light list) that this view belongs to
		int   m_face;        // Cube face index for point lights (+X,-X,+Y,-Y,+Z,-Z), otherwise 0
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
	void BuildShadowViews(std::span<Light const> lights, BBox const& caster_bounds, ShadowSettings const& settings, ShadowViewSet& out);

	// Assign square atlas regions for views with the requested sizes. Sizes are rounded down to powers of two and clamped to the atlas size.
	// When the views do not fit, the largest views are halved until they do. The result is deterministic for the same inputs.
	void PackShadowAtlas(std::span<int const> requested_sizes, int atlas_size, std::span<IRect> out);

	// Return the point light cube face index (+X,-X,+Y,-Y,+Z,-Z) that contains the direction 'light_to_point'
	int ShadowCubeFace(v4 light_to_point);

	// True if the world space box 'bbox' may be visible in the clip space defined by 'w2s'
	bool ShadowViewSees(m4x4 const& w2s, BBox const& bbox);
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::tests
{
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
			PR_EXPECT(ShadowViewSees(w2s, BBox(v4::Origin(), v4(1, 1, 1, 0))));
			PR_EXPECT(!ShadowViewSees(w2s, BBox(v4(0, 0, 20, 1), v4(1, 1, 1, 0))));
			PR_EXPECT(!ShadowViewSees(w2s, BBox(v4(50, 0, 0, 1), v4(1, 1, 1, 0))));
			PR_EXPECT(!ShadowViewSees(w2s, BBox(v4(0, 0, -30, 1), v4(1, 1, 1, 0))));
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

			ShadowSettings settings;
			ShadowViewSet set;
			BuildShadowViews(lights, bounds, settings, set);
			PR_EXPECT(set.m_views.size() == 8);
			PR_EXPECT(set.m_light_views.size() == 4);
			PR_EXPECT(All(set.m_light_views[0] == iv2(0, 1)));
			PR_EXPECT(All(set.m_light_views[1] == iv2(1, 6)));
			PR_EXPECT(All(set.m_light_views[2] == iv2(-1, 0)));
			PR_EXPECT(All(set.m_light_views[3] == iv2(7, 1)));
			for (int i = 0; i != 6; ++i)
				PR_EXPECT(set.m_views[1 + i].m_face == i);

			// The caster bounds centre is in front of each view that should see it
			PR_EXPECT(ShadowViewSees(set.m_views[0].m_w2s, bounds));
			PR_EXPECT(ShadowViewSees(set.m_views[7].m_w2s, bounds));
			PR_EXPECT(ShadowViewSees(set.m_views[1 + ShadowCubeFace(v4(0, -1, 0, 0))].m_w2s, BBox(v4::Origin(), v4(0.1f, 0.1f, 0.1f, 0))));

			settings.m_max_shadow_lights = 1;
			BuildShadowViews(lights, bounds, settings, set);
			PR_EXPECT(set.m_views.size() == 1);
			PR_EXPECT(All(set.m_light_views[1] == iv2(-1, 0)));

			settings.m_max_shadow_lights = 0;
			BuildShadowViews(lights, bounds, settings, set);
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
			BuildShadowViews({ &point_light, 1 }, BBox(v4::Origin(), v4(1, 1, 1, 0)), ShadowSettings{}, set);
			PR_EXPECT(set.empty());
			PR_EXPECT(All(set.m_light_views[0] == iv2(-1, 0)));
		}
	}
}
#endif
