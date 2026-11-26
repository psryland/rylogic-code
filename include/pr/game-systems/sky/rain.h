//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Falling rain drawn as motion-blurred streaks around the camera.
// The rain is a field of drops that fills all of space, so it looks the same at any camera speed: the camera moves through the drops and is never
// ahead of them. The field is made of a few nested layers of a repeating tile of drops. The smallest tile has the most drops per volume and is drawn
// near the camera; larger tiles are drawn further away, where drops are too small to see one by one, so they need fewer drops per volume.
// Every drop is a streak along its motion relative to the camera over a short exposure, so the streaks lean into the camera's motion.
#pragma once
#include "pr/game-systems/forward.h"

namespace pr::game_systems
{
	// The state of the rain. The frame is any right-handed frame; the fall velocity decides which way is down.
	struct RainSettings
	{
		// How hard it rains, in [0,1]. 0 is no rain, small values are a light sparse shower, and 1 is a heavy downpour.
		// More drops are shown as the intensity rises, and the drops get a little larger.
		float m_intensity = 0.0f;

		// The velocity of the air, in world units per second. The drops drift with it.
		v4 m_wind = v4::Zero();

		// The velocity of a falling drop in still air, in world units per second. Drops fall at between 80% and 120% of it. Must be nonzero.
		v4 m_fall_velocity = v4(0, 0, -7, 0);

		// The velocity of the camera, in world units per second. Streaks show the drops' motion relative to the camera.
		v4 m_camera_velocity = v4::Zero();

		// The colour of a lit drop (linear rgb, nonnegative). The caller lights the rain, e.g. with the sky's ambient light.
		v4 m_colour = v4(0.6f, 0.65f, 0.7f, 1.0f);

		// A world-space plane (xyz = normal, w = offset) that the rain does not fall through. Drops at points p where dot(xyz, p) + w < 0 are not drawn,
		// so a plane on a water surface keeps the rain out of the water. A zero plane hides no drops. Must be finite.
		v4 m_clip_plane = v4::Zero();

		// The most a single streak covers what is behind it, in [0,1].
		float m_opacity = 0.25f;

		// The exposure time of the streaks, in seconds (> 0). Longer times give longer streaks.
		float m_streak_time = 0.02f;

		// The width of a drop at full intensity, in world units (> 0).
		float m_drop_width = 0.0015f;

		// The caller's clock, in seconds. Drops move by the change in time since the previous update, so a slower clock gives slow-motion rain.
		// Any finite time is valid. A step back in time moves the drops back.
		double m_time = 0.0;
	};

	struct RainShader;

	// Rain around the camera. Create, update and release on the renderer owner thread. The model is drawn without a transform:
	// it places its drops around the camera of whichever view draws it.
	struct Rain
	{
		// The number of nested tile layers, and the number of speed classes that drops fall at.
		static constexpr int LayerCount = 4;
		static constexpr int SpeedClasses = 4;

		// The drops in each layer's tile.
		static constexpr int DropsPerLayer = 32768;

		// The side length of the smallest tile, in world units, and the ratio between the side lengths of successive layers.
		// Each layer's side length divides the largest one, so the whole field repeats with the largest tile.
		static constexpr double TileSize = 8.0;
		static constexpr int TileRatio = 3;

		// The fall speed of each speed class, relative to RainSettings::m_fall_velocity.
		static constexpr float SpeedScale[SpeedClasses] = { 0.8f, 0.93f, 1.07f, 1.2f };

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

		// The rain's shader, which also holds the drops' drift. The model's material owns it, so it lives as long as the model.
		RainShader* m_shader;

		// The time of the last update, or none before the first update.
		std::optional<double> m_last_time;

		// Create the rain's model. It shows no rain until updated with an intensity above zero.
		explicit Rain(Renderer& rdr);

		// Change the rain. Throws std::invalid_argument for invalid settings, leaving the rain unchanged.
		void Update(RainSettings const& settings);

		// The side length of 'layer's tile, in world units.
		static double LayerSize(int layer);
	};
}
