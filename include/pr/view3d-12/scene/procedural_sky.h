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

		// Create the model and shader once; no image files or runtime shader compiler are required.
		explicit ProceduralSky(Renderer& rdr);

		// Set finite sun parameters; direction points toward the sun and is normalized here. Colour/intensity must be nonnegative.
		void Update(v4 sun_direction, v4 sun_colour, float sun_intensity);

		// Blend an optional retained cubemap into the atmosphere (0=cubemap, 1=atmosphere).
		// The orthonormal transforms map scene directions into each background's world frame; the cubemap's own orientation is also respected.
		// With no cubemap only weight 1 is valid. Updates must not overlap rendering.
		void Blend(TextureCubePtr background, float weight, m4x4 const& world_to_sky, m4x4 const& world_to_background);

		// Add the background, independent of camera translation and clip distances.
		void AddToScene(Scene& scene);
	};
}
