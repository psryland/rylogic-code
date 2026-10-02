//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Atmospheric sky shared by native scenes and View3D objects.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/instance/instance.h"

namespace pr::rdr12
{
	struct ProceduralSkyShader;

	// Per-frame atmosphere and cloud parameters. Directions and positions are in the sky frame (Z up), and distances are in world units (assumed metres).
	struct ProceduralSkySettings
	{
		// Direction toward the sun. Must be finite and nonzero; it is normalised by the sky.
		v4 m_sun_direction = v4(0.5f, 0.3f, 0.8f, 0);

		// Linear sun colour and intensity (0=night, 1=noon). Both must be finite and nonnegative.
		v4 m_sun_colour = v4(1.0f, 0.95f, 0.85f, 1);
		float m_sun_intensity = 1.0f;

		// Cloud cover in [0,1]: 0 is clear, 0.5 is scattered white cloud, and 1 is dark storm overcast.
		// Used everywhere when no weather map is bound, otherwise used outside the weather map's area.
		float m_cloud_cover = 0.0f;

		// Speed (>= 0) of the lowest cloud layer in world units per second. Higher layers move proportionally slower.
		float m_wind_speed = 0.0f;

		// Azimuth of cloud travel in radians, measured from +X toward +Y.
		float m_wind_direction = 0.0f;

		// Absolute time in seconds. Clouds move by the wind over the change in time since the previous update, so wind changes never make the clouds jump.
		double m_time = 0.0;
	};

	// Owns a Z-up atmospheric sky with an optional retained cubemap. Create, update and release on the renderer owner thread.
	struct ProceduralSky
	{
		// A translation-independent background instance, sorted after opaque geometry.
		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4       , m_i2w  , EInstComp::I2WTransform)\
			x(ModelPtr   , m_model, EInstComp::ModelPtr)\
			x(SKOverride , m_sko  , EInstComp::SortkeyOverride)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		Instance m_inst;
		ProceduralSkyShader* m_shader;
		std::optional<double> m_last_time;
		v2 m_cloud_offset[3];
		float m_cloud_evolve[3];

		// Create the model and shader once; no image files or runtime shader compiler are required.
		explicit ProceduralSky(Renderer& rdr);

		// Set the atmosphere and cloud parameters. Invalid settings throw and leave the previous sky unchanged.
		void Update(ProceduralSkySettings const& settings);

		// Use 'weather' as the cloud cover field, or null for uniform cover. The map is retained, and later uploads to it are used automatically.
		void Weather(WeatherMapPtr weather);

		// Blend an optional retained cubemap into the atmosphere (0=cubemap, 1=atmosphere).
		// The orthonormal transforms map scene directions into each background's world frame; the cubemap's own orientation is also respected.
		// With no cubemap only weight 1 is valid. Updates must not overlap rendering.
		void Blend(TextureCubePtr background, float weight, m4x4 const& world_to_sky, m4x4 const& world_to_background);

		// Add the background, independent of camera translation and clip distances.
		void AddToScene(Scene& scene);
	};
}
