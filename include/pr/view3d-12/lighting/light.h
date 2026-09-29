//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// The maximum number of lights that shade a frame. Must match 'MaxLights' in lighting_cbuf.hlsli.
	inline static constexpr int MaxLights = 64;

	struct alignas(16) Light
	{
		v4       m_position;       // Position, only valid for point,spot lights
		v4       m_direction;      // Direction, only valid for directional,spot lights
		ELight   m_type;           // One of directional, point, spot
		Colour32 m_diffuse;        // Main light colour
		Colour32 m_specular;       // Specular light colour
		float    m_specular_power; // Specular power (controls specular spot size)
		float    m_intensity;      // Light intensity scale
		float    m_range;          // Light range. Point and spot lights do not illuminate surfaces beyond this distance
		float    m_falloff;        // Intensity falloff per unit distance
		float    m_inner_angle;    // Spot light inner angle 100% light (in radians)
		float    m_outer_angle;    // Spot light outer angle 0% light (in radians)
		float    m_cast_shadow;    // Shadow strength in [0,1]. Values > 0 mean the light casts shadows
		bool     m_cam_relative;   // True if the light should move with the camera
		bool     m_on;             // True if this light is on

		Light();
		bool IsValid() const;

		// True if this light casts shadows
		bool CastsShadow() const
		{
			return m_cast_shadow > 0;
		}

		// Return a copy of this light in world space. Camera-relative lights are transformed by 'c2w'.
		Light InWorldSpace(m4x4 const& c2w) const;

		// Returns a light to world transform appropriate for this light type and facing 'centre'
		m4x4 LightToWorld(v4 centre, float centre_dist, m4x4 const& c2w = m4x4::Identity()) const;

		// Returns a projection transform appropriate for this light type
		m4x4 Projection(float zn, float zf, float w, float h, float focus_dist) const;
		m4x4 ProjectionFOV(float zn, float zf, float aspect, float fovY, float focus_dist) const;

		// Get/Set light settings - throws Exception<HRESULT> if the settings are invalid
		std::string Settings() const;
		void Settings(std::string_view settings);

		// Operators
		friend bool operator == (Light const& lhs, Light const& rhs)
		{
			// Compare members rather than bytes because the struct contains padding
			return
				!Any(lhs.m_position != rhs.m_position) &&
				!Any(lhs.m_direction != rhs.m_direction) &&
				lhs.m_type == rhs.m_type &&
				lhs.m_diffuse == rhs.m_diffuse &&
				lhs.m_specular == rhs.m_specular &&
				lhs.m_specular_power == rhs.m_specular_power &&
				lhs.m_intensity == rhs.m_intensity &&
				lhs.m_range == rhs.m_range &&
				lhs.m_falloff == rhs.m_falloff &&
				lhs.m_inner_angle == rhs.m_inner_angle &&
				lhs.m_outer_angle == rhs.m_outer_angle &&
				lhs.m_cast_shadow == rhs.m_cast_shadow &&
				lhs.m_cam_relative == rhs.m_cam_relative &&
				lhs.m_on == rhs.m_on;
		}
		friend bool operator != (Light const& lhs, Light const& rhs)
		{
			return !(lhs == rhs);
		}
	};

	// A list of lights. Most scenes have only a few lights, so a small local buffer avoids allocations.
	using LightList = pr::vector<Light, 8>;

	// Build the world space lights that shade a frame, in a deterministic order: 'scene_lights' first, then 'frame_lights'.
	// Camera-relative lights are transformed by 'c2w'. Lights that are off are skipped. At most 'MaxLights' lights are written to 'out'.
	// Returns the number of lights that were on but skipped because the limit was reached.
	int ResolveLights(std::span<Light const> scene_lights, std::span<Light const> frame_lights, m4x4 const& c2w, LightList& out);
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::tests
{
	PRUnitTest(LightResolutionTests, Quick)
	{
		// Camera-relative lights move with the camera; world lights do not.
		{
			auto c2w = m4x4::Transform(v4::YAxis(), constants<float>::tau_by_4, v4(1, 2, 3, 1));

			Light cam_light;
			cam_light.m_type = ELight::Spot;
			cam_light.m_position = v4(0, 0, 1, 1);
			cam_light.m_direction = v4(0, 0, -1, 0);
			cam_light.m_cam_relative = true;

			Light world_light;
			world_light.m_type = ELight::Point;
			world_light.m_position = v4(5, 6, 7, 1);

			LightList out;
			auto dropped = ResolveLights({ &cam_light, 1 }, { &world_light, 1 }, c2w, out);
			PR_EXPECT(dropped == 0);
			PR_EXPECT(out.size() == 2);
			PR_EXPECT(FEql(out[0].m_position, c2w * cam_light.m_position));
			PR_EXPECT(FEql(out[0].m_direction, c2w * cam_light.m_direction));
			PR_EXPECT(out[0].m_cam_relative == false);
			PR_EXPECT(FEql(out[1].m_position, world_light.m_position));
		}

		// Lights that are off are dropped without counting against the limit.
		{
			Light on_light;
			Light off_light;
			off_light.m_on = false;
			off_light.m_diffuse = Colour32Red;

			Light lights[] = { off_light, on_light, off_light };
			LightList out;
			auto dropped = ResolveLights(lights, {}, m4x4::Identity(), out);
			PR_EXPECT(dropped == 0);
			PR_EXPECT(out.size() == 1);
			PR_EXPECT(out[0].m_diffuse == on_light.m_diffuse);
		}

		// The limit keeps the first 'MaxLights' lights, scene lights before frame lights.
		{
			std::vector<Light> scene_lights(MaxLights - 2);
			std::vector<Light> frame_lights(5);
			for (int i = 0; i != isize(frame_lights); ++i)
				frame_lights[i].m_intensity = float(i);

			LightList out;
			auto dropped = ResolveLights(scene_lights, frame_lights, m4x4::Identity(), out);
			PR_EXPECT(dropped == 3);
			PR_EXPECT(out.size() == MaxLights);
			PR_EXPECT(out[MaxLights - 2].m_intensity == 0.0f);
			PR_EXPECT(out[MaxLights - 1].m_intensity == 1.0f);
		}
	}
}
#endif
