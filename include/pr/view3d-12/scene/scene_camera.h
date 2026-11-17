//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Camera transforms shared by draws in one render pass. Recreate these values when the camera changes, including between stereo eyes.
	struct CameraTransforms
	{
		m4x4 const m_w2c;
		m4x4 const m_w2s;

		// Capture the view and its default projection for the current pass.
		explicit CameraTransforms(Camera const& camera);
	};

	struct SceneCamera :Camera
	{
		// Notes:
		//  - The properties of a camera that looks into the scene.

		float m_shadow_zfar;            // The far plane for shadows
		float m_shadow_max_caster_dist; // The maximum distance for objects that cast shadows (used to normalise depth values in the smap)

		SceneCamera();
		SceneCamera(Camera const& cam);
		SceneCamera(m4x4 const& c2w, float fovY = pr::constants<double>::tau_by_8, float aspect = 1.0f, float focus_dist = 1.0f, bool orthographic = false, float near_ = 0.01f, float far_ = 100.0f);

		// Return the camera for one eye of a parallel-axis stereo pair derived from this (logical) camera.
		// 'separation' is the distance between the eyes and 'convergence' is the distance to the zero-disparity plane, both in scene units.
		// The eyes are offset along the camera's x axis and keep its orientation. Each eye's projection is shifted so that points on the
		// convergence plane project to the same screen position in both eyes. 'eye_aspect' is the width/height ratio of the eye's image.
		// Orthographic cameras have no depth disparity, so both eyes are the logical camera with 'eye_aspect'.
		SceneCamera StereoEye(EEye eye, float separation, float convergence, float eye_aspect) const;
	};
}
