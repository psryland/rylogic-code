//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Deterministic world-anchored root projection (implementation-plan.md sections 2.2/4.4, M8).
// Every function here is a pure function of its arguments: no device, no View3D call, no global
// state, so the projection a world root is laid out, hit-tested, described and drawn with is one
// value computed once per Update and shared by all four consumers.
//
// Conventions match View3D's camera (pr::math::ProjectionPerspective/ProjectionOrthographic with
// righthanded == true): the camera looks along CameraState::forward, camera space is right-handed
// with -z along the look direction, and normalised device depth runs 0 at the near plane to 1 at
// the far plane.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// An orthonormal camera basis derived from a CameraState. 'valid' is 0 when the camera is
	// absent or degenerate (zero-length or parallel forward/up), which culls every world root
	// deterministically instead of producing NaN geometry.
	struct CameraFrame
	{
		Vec3 position;
		Vec3 right;     // Camera-space +x in world space.
		Vec3 up;        // Camera-space +y in world space.
		Vec3 forward;   // Unit look direction; camera-space -z in world space.
		std::int32_t valid;
	};

	// Where and how one root is placed in viewport-relative screen DIP space for this update.
	// Screen roots always produce scale 1, depth 0 and visible 1, so the M0-M5 behaviour is
	// bit-for-bit unchanged; world roots carry the projected rect plus the depth and fade
	// parameters the renderer needs to select and drive its host stage.
	struct RootPlacement
	{
		ERootPolicy policy;
		std::int32_t visible;           // 0 => culled this update; the subtree emits no draw items and is semantically Offscreen.
		Rect rect;                      // Screen-DIP rect of the root's own box.
		float scale;                    // Screen DIPs per local DIP applied to the whole subtree.
		float view_depth;               // Distance from the camera along 'forward' to the anchor, world units; 0 for screen roots.
		float clip_depth;               // Normalised [0, 1] device depth recorded for depth-tested/occlusion-faded roots; 0 for screen roots.
		float occlusion_min_opacity;    // Copied from WorldRootParams; only meaningful under ERootPolicy::OcclusionFaded.
		float occlusion_fade_depth;
		float occlusion_depth_bias;
	};

	// Build the orthonormal camera basis for 'camera'; the result's 'valid' is 0 if no usable
	// basis exists.
	CameraFrame MakeCameraFrame(CameraState const& camera);

	// Throw EngineException if 'camera' claims to be valid but carries a value that would make
	// projection non-deterministic or non-finite. A camera with valid == 0 is always accepted.
	void ValidateCameraState(CameraState const& camera);

	// The placement of a screen-space root occupying 'rect'.
	RootPlacement ScreenRootPlacement(Rect const& rect);

	// Project a world-anchored root's local DIP box into viewport-relative screen DIP space.
	// 'box_w_dip'/'box_h_dip' are the root's authored (unscaled) layout size. The returned
	// placement is culled when there is no usable camera, when the anchor is behind the camera or
	// nearer than the near plane, when it is beyond the far plane, or when the projected rect
	// lies entirely outside the viewport.
	RootPlacement ProjectWorldRoot(ERootPolicy policy, WorldRootParams const& world, float box_w_dip, float box_h_dip, ViewportState const& viewport);

	// Screen DIPs spanned by one world unit measured on the viewport's vertical axis, at
	// 'view_depth' world units in front of 'camera'. Pure helper shared by the sizing rule and
	// its tests.
	float DipsPerWorldUnit(CameraState const& camera, ViewportState const& viewport, float view_depth);

	// Normalised [0, 1] device depth for a point 'view_depth' world units in front of 'camera',
	// using the same right-handed convention as View3D's projection matrices. Values outside the
	// near/far range are clamped so a recorded depth is always inside the depth buffer's domain.
	float NormalisedDeviceDepth(CameraState const& camera, float view_depth);
}
