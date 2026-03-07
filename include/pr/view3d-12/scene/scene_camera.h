//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	struct SceneCamera :Camera
	{
		// Notes:
		//  - The properties of a camera that looks into the scene.

		float m_shadow_zfar;            // The far plane for shadows
		float m_shadow_max_caster_dist; // The maximum distance for objects that cast shadows (used to normalise depth values in the smap)

		SceneCamera();
		SceneCamera(Camera const& cam);
		SceneCamera(m4x4 const& c2w, float fovY = pr::constants<double>::tau_by_8, float aspect = 1.0f, float focus_dist = 1.0f, bool orthographic = false, float near_ = 0.01f, float far_ = 100.0f);

		// Return the scene views for the left and right eye in stereoscopic view
		void Stereo(float separation, SceneCamera (&eye)[Enum<EEye>::NumberOf]) const;
	};
}
