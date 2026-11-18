//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/scene/scene_camera.h"

namespace pr::rdr12
{
	CameraTransforms::CameraTransforms(Camera const& camera)
		:m_w2c(camera.WorldToCamera())
		,m_w2s(camera.CameraToScreen() * m_w2c)
	{}

	// Construct scene views
	SceneCamera::SceneCamera()
		:Camera()
		,m_shadow_zfar(10.0f)
		,m_shadow_max_caster_dist(20.0f)
	{
		PR_ASSERT(PR_DBG_RDR, meta::is_aligned<SceneCamera>(this), "My alignment is broke");
	}
	SceneCamera::SceneCamera(pr::Camera const& cam)
		:Camera(cam)
		,m_shadow_zfar(s_cast<float>(3.0 * cam.FocusDist()))
		,m_shadow_max_caster_dist(s_cast<float>(4.0 * cam.FocusDist()))
	{}
	SceneCamera::SceneCamera(m4x4 const& c2w, float fovY, float aspect, float focus_dist, bool orthographic, float near_, float far_)
		:pr::Camera(c2w, fovY, aspect, focus_dist, orthographic, near_, far_)
		,m_shadow_zfar(3.0f * focus_dist)
		,m_shadow_max_caster_dist(4.0f * focus_dist)
	{}

	// Return the camera for one eye of a parallel-axis stereo pair derived from this (logical) camera.
	SceneCamera SceneCamera::StereoEye(EEye eye, float separation, float convergence, float eye_aspect) const
	{
		// Both eyes keep the logical field of view and focus distance, but use the eye image's aspect ratio
		auto cam = *this;
		cam.Aspect(eye_aspect);
		if (Orthographic())
			return cam;

		// The eye sits half the separation to the side of the logical camera, with the same orientation (no toe-in)
		auto sign = 0.0f;
		switch (eye)
		{
			case EEye::Left: { sign = -1.0f; break; }
			case EEye::Right: { sign = +1.0f; break; }
			default: throw std::runtime_error("Unknown eye");
		}
		auto c2w = CameraToWorld();
		c2w.pos += (sign * 0.5f * separation) * c2w.x;
		cam.CameraToWorld(c2w, false);

		// A point on the logical view axis at the convergence distance is offset by -sign*s/2 in the eye's camera space,
		// which projects to -sign*(s/2)/(C*tanX) in normalised screen space. Shift the projection by the opposite amount so it lands at 0.
		PR_ASSERT(PR_DBG_RDR, convergence > 0, "Stereo convergence distance must be positive");
		auto tan_half_fovx = std::tan(0.5 * cam.FovY()) * eye_aspect;
		auto shift = s_cast<float>(sign * 0.5 * separation / (convergence * tan_half_fovx));
		cam.LensShift(v2(shift, 0.0f));
		return cam;
	}
}