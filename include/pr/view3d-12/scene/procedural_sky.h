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

		// Azimuth of cloud travel in radians, measured from +X toward +Y. Changes never make the clouds jump: they always move with the current wind,
		// and their streaks turn to follow a new direction by cross-fading to clouds aligned with it (see ProceduralSky::WindFrameTurn).
		float m_wind_direction = 0.0f;

		// Absolute time in seconds. Clouds move by the wind over the change in time since the previous update, so wind changes never make the clouds jump.
		double m_time = 0.0;

		// Bit mask of cloud layers to hide: bit i hides layer i (0 = low, 1 = mid, 2 = cirrus). Zero shows all layers.
		uint32_t m_hidden_cloud_layers = 0;

		// Lightning flashes inside the cloud: xy = position in the sky frame, z = radius (> 0), w = brightness (>= 0, 1 is a strong flash, 0 is no flash).
		// The caller owns when flashes happen and how they flicker; the sky shows each flash's current brightness.
		static constexpr int LightningMax = 4;
		v4 m_lightning[LightningMax] = {};

		// Strength of the low and mid clouds' shadows on the scene, in [0,1]. 0 casts no cloud shadows.
		// Cloud shadows dim the scene's directional light that shines from the sky's sun.
		float m_cloud_shadow_strength = 1.0f;

		// Brightness of the moon, finite and nonnegative. 0 hides the moon, 1 is a full moon on a clear night. The moon is drawn additively,
		// so it is naturally faint against a daytime sky. The moon also adds a dim moonlit glow to the night sky and hides the faintest stars.
		float m_moon_intensity = 0.0f;

		// Direction toward the moon. Must be finite and nonzero when the moon is shown; it is normalised by the sky.
		v4 m_moon_direction = v4(-0.5f, -0.3f, 0.5f, 0);

		// Direction from the moon toward the light that lights it (the sun). It sets the moon's phase: equal to m_moon_direction (the sun is
		// behind the moon) is a new moon, and equal to -m_moon_direction (the sun is behind the viewer) is a full moon. It is independent of
		// m_sun_direction, so callers may run the moon's cycle separately from the day. Must be finite and nonzero when the moon is shown.
		v4 m_moon_light_direction = v4(0.5f, 0.3f, -0.5f, 0);

		// Rotation from the sky frame into the star frame, where Z is the north celestial pole and X points toward right ascension 0h.
		// Stars, the Milky Way and the Magellanic Clouds are fixed in the star frame, so the caller turns this to set the observer's latitude
		// and to wheel the stars through the night. Must be a rotation (orthonormal, no translation).
		m4x4 m_sky_to_stars = m4x4::Identity();
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

		// The clouds are drawn in two wind frames, each aligned with a wind direction (see CloudConstants::frame_axes). Frame 1 follows the wind.
		// When the wind turns more than 'WindFrameTurn' (radians) away from frame 1, frame 1 becomes frame 0, a new frame 1 is aligned with
		// the wind, and the clouds cross-fade to it over 'WindFrameFade' seconds of sky time.
		static constexpr double WindFrameTurn = 10.0 * constants<double>::tau / 360.0;
		static constexpr double WindFrameFade = 20.0;
		struct WindFrame
		{
			// The frame's x axis azimuth (radians from +X toward +Y), and each cloud layer's offset in noise tiles of the frame.
			double m_angle;
			v2 m_offset[3];
		};

		Instance m_inst;
		ProceduralSkyShader* m_shader;
		std::optional<double> m_last_time;
		WindFrame m_frames[2];
		double m_frame_blend;
		float m_cloud_evolve[3];

		// The total distance the low cloud has moved with the wind, in world units in the sky frame. Views use it to follow the cloud between frames.
		double m_low_cloud_drift[2];

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

		// Add the background, independent of camera translation and clip distances. The scene also uses this sky for cloud shadows.
		void AddToScene(Scene& scene);

		// Get the cloud field for cloud shadows on the scene, and the sky's texture table (cube, weather map, cloud noise) that the cloud field reads.
		// Returns false, with 'clouds' still filled, when the sky casts no cloud shadows.
		bool CloudShadows(shaders::CloudConstants& clouds, Descriptor (&textures)[3]) const;
	};
}
