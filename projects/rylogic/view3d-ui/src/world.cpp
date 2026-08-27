//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "world.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Minimal Vec3 arithmetic. Deliberately local rather than pulled from pr::maths: this
		// module is part of the dependency-minimal wire/layout core and must stay usable from the
		// ABI-facing headers without dragging in the maths library.
		Vec3 Sub(Vec3 const& a, Vec3 const& b)
		{
			return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z };
		}
		Vec3 Neg(Vec3 const& a)
		{
			return Vec3{ -a.x, -a.y, -a.z };
		}
		float Dot(Vec3 const& a, Vec3 const& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
		Vec3 Cross(Vec3 const& a, Vec3 const& b)
		{
			return Vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
		}
		bool IsFinite(Vec3 const& a)
		{
			return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
		}

		// Normalise 'a' in place, returning false (leaving 'a' untouched) when its length is not
		// usable. The threshold is an absolute length rather than a relative epsilon so the test
		// is itself deterministic for any input magnitude.
		bool TryNormalise(Vec3& a)
		{
			auto len_sq = Dot(a, a);
			if (!std::isfinite(len_sq) || len_sq < 1e-24f)
				return false;

			auto len = std::sqrt(len_sq);
			a = Vec3{ a.x / len, a.y / len, a.z / len };
			return true;
		}

		// Offset of a root's near edge from its anchor point, for a box of 'size' DIPs.
		float AnchorOffset(EAnchorPoint anchor, float size)
		{
			switch (anchor)
			{
				case EAnchorPoint::Min: return 0.0f;
				case EAnchorPoint::Centre: return -size * 0.5f;
				case EAnchorPoint::Max: return -size;
				case EAnchorPoint::Count:
				default: throw EngineException(EStatus::UnknownType, std::format("AnchorOffset: unknown EAnchorPoint {}", static_cast<int>(anchor)));
			}
		}

		// Viewport aspect ratio (width / height). Zero-height viewports are reported as zero so
		// callers cull rather than divide by zero.
		float ViewportAspect(ViewportState const& viewport)
		{
			if (!(viewport.viewport_width_px > 0.0f) || !(viewport.viewport_height_px > 0.0f))
				return 0.0f;

			return viewport.viewport_width_px / viewport.viewport_height_px;
		}

		// A placement carrying no geometry, used for every culled root so downstream consumers
		// never see partially-populated values.
		RootPlacement CulledPlacement(ERootPolicy policy, WorldRootParams const& world)
		{
			return RootPlacement{
				.policy = policy,
				.visible = 0,
				.rect = Rect{ 0.0f, 0.0f, 0.0f, 0.0f },
				.scale = 1.0f,
				.view_depth = 0.0f,
				.clip_depth = 0.0f,
				.occlusion_min_opacity = world.occlusion_min_opacity,
				.occlusion_fade_depth = world.occlusion_fade_depth,
				.occlusion_depth_bias = world.occlusion_depth_bias,
			};
		}
	}

	CameraFrame MakeCameraFrame(CameraState const& camera)
	{
		auto frame = CameraFrame{ .position = camera.position, .right = {}, .up = {}, .forward = {}, .valid = 0 };
		if (camera.valid == 0)
			return frame;

		if (!IsFinite(camera.position) || !IsFinite(camera.forward) || !IsFinite(camera.up))
			return frame;

		auto forward = camera.forward;
		if (!TryNormalise(forward))
			return frame;

		// View3D's camera is right-handed and looks along -z in camera space, so the camera's own
		// z axis points backwards along the look direction (see pr::math::LookAt).
		auto cam_z = Neg(forward);
		auto right = Cross(camera.up, cam_z);
		if (!TryNormalise(right))
			return frame; // 'up' is parallel to the look direction; no usable basis exists

		frame.right = right;
		frame.up = Cross(cam_z, right);
		frame.forward = forward;
		frame.valid = 1;
		return frame;
	}

	void ValidateCameraState(CameraState const& camera)
	{
		if (camera.valid == 0)
			return;

		if (static_cast<std::uint32_t>(camera.projection) >= static_cast<std::uint32_t>(EProjection::Count))
			throw EngineException(EStatus::UnknownType, std::format("camera: unknown EProjection {}", static_cast<int>(camera.projection)));
		if (!IsFinite(camera.position) || !IsFinite(camera.forward) || !IsFinite(camera.up))
			throw EngineException(EStatus::InvalidArgument, "camera: position/forward/up must all be finite");
		if (!std::isfinite(camera.near_plane) || camera.near_plane <= 0.0f)
			throw EngineException(EStatus::InvalidArgument, std::format("camera: near plane {} must be finite and greater than zero", camera.near_plane));
		if (!std::isfinite(camera.far_plane) || camera.far_plane <= camera.near_plane)
			throw EngineException(EStatus::InvalidArgument, std::format("camera: far plane {} must be finite and greater than the near plane {}", camera.far_plane, camera.near_plane));

		// Only the field applicable to the declared projection is required to be in range; the
		// other is ignored exactly like every other "applies to a subset" wire field.
		switch (camera.projection)
		{
			case EProjection::Perspective:
			{
				if (!std::isfinite(camera.fov_y_rad) || camera.fov_y_rad <= 0.0f || camera.fov_y_rad >= 3.14159265f)
					throw EngineException(EStatus::InvalidArgument, std::format("camera: vertical field of view {} must be finite and within (0, pi)", camera.fov_y_rad));

				break;
			}
			case EProjection::Orthographic:
			{
				if (!std::isfinite(camera.ortho_height) || camera.ortho_height <= 0.0f)
					throw EngineException(EStatus::InvalidArgument, std::format("camera: orthographic height {} must be finite and greater than zero", camera.ortho_height));

				break;
			}
			case EProjection::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("camera: unknown EProjection {}", static_cast<int>(camera.projection)));
			}
		}
	}

	RootPlacement ScreenRootPlacement(Rect const& rect)
	{
		return RootPlacement{
			.policy = ERootPolicy::Screen,
			.visible = 1,
			.rect = rect,
			.scale = 1.0f,
			.view_depth = 0.0f,
			.clip_depth = 0.0f,
			.occlusion_min_opacity = 1.0f,
			.occlusion_fade_depth = 1.0f,
			.occlusion_depth_bias = 0.0f,
		};
	}

	float DipsPerWorldUnit(CameraState const& camera, ViewportState const& viewport, float view_depth)
	{
		if (!(viewport.dpi > 0.0f) || !(viewport.viewport_height_px > 0.0f))
			return 0.0f;

		// Pixels spanned by one world unit measured vertically at the centre of the viewport,
		// then converted to DIPs so the result composes with the DIP-based layout box.
		auto px_per_world = 0.0f;
		switch (camera.projection)
		{
			case EProjection::Perspective:
			{
				auto tan_half = std::tan(camera.fov_y_rad * 0.5f);
				if (!(tan_half > 0.0f) || !(view_depth > 0.0f))
					return 0.0f;

				px_per_world = viewport.viewport_height_px * 0.5f / (view_depth * tan_half);
				break;
			}
			case EProjection::Orthographic:
			{
				if (!(camera.ortho_height > 0.0f))
					return 0.0f;

				px_per_world = viewport.viewport_height_px / camera.ortho_height;
				break;
			}
			case EProjection::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("DipsPerWorldUnit: unknown EProjection {}", static_cast<int>(camera.projection)));
			}
		}
		return px_per_world * 96.0f / viewport.dpi;
	}

	float NormalisedDeviceDepth(CameraState const& camera, float view_depth)
	{
		auto zn = camera.near_plane;
		auto zf = camera.far_plane;
		if (!(zf > zn))
			return 0.0f;

		auto z = std::clamp(view_depth, zn, zf);
		auto depth = 0.0f;
		switch (camera.projection)
		{
			case EProjection::Perspective:
			{
				depth = zf * (z - zn) / (z * (zf - zn));
				break;
			}
			case EProjection::Orthographic:
			{
				depth = (z - zn) / (zf - zn);
				break;
			}
			case EProjection::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("NormalisedDeviceDepth: unknown EProjection {}", static_cast<int>(camera.projection)));
			}
		}
		return std::clamp(depth, 0.0f, 1.0f);
	}

	RootPlacement ProjectWorldRoot(ERootPolicy policy, WorldRootParams const& world, float box_w_dip, float box_h_dip, ViewportState const& viewport)
	{
		auto const& camera = viewport.camera;

		// No camera, no usable basis, or a degenerate viewport: cull rather than guess.
		auto frame = MakeCameraFrame(camera);
		auto aspect = ViewportAspect(viewport);
		if (frame.valid == 0 || !(aspect > 0.0f) || !(viewport.dpi > 0.0f))
			return CulledPlacement(policy, world);

		// Depth along the look direction decides visibility before any division happens, so a
		// behind-camera or out-of-frustum anchor can never produce a NaN or a mirrored rect.
		auto delta = Sub(world.anchor, frame.position);
		auto view_depth = Dot(delta, frame.forward);
		if (!std::isfinite(view_depth) || view_depth < camera.near_plane || view_depth > camera.far_plane)
			return CulledPlacement(policy, world);

		// Camera-space lateral offsets projected to normalised device coordinates.
		auto cam_x = Dot(delta, frame.right);
		auto cam_y = Dot(delta, frame.up);
		auto ndc_x = 0.0f;
		auto ndc_y = 0.0f;
		switch (camera.projection)
		{
			case EProjection::Perspective:
			{
				auto tan_half = std::tan(camera.fov_y_rad * 0.5f);
				if (!(tan_half > 0.0f))
					return CulledPlacement(policy, world);

				ndc_x = cam_x / (view_depth * tan_half * aspect);
				ndc_y = cam_y / (view_depth * tan_half);
				break;
			}
			case EProjection::Orthographic:
			{
				auto half_h = camera.ortho_height * 0.5f;
				if (!(half_h > 0.0f))
					return CulledPlacement(policy, world);

				ndc_x = cam_x / (half_h * aspect);
				ndc_y = cam_y / half_h;
				break;
			}
			case EProjection::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("ProjectWorldRoot: unknown EProjection {}", static_cast<int>(camera.projection)));
			}
		}
		if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y))
			return CulledPlacement(policy, world);

		// Normalised device coordinates to viewport pixels (y runs down the screen), then to the
		// viewport-relative DIP space every other stage of the pipeline works in. The viewport's
		// own origin within the render target is deliberately not added: it belongs to the target
		// space, and the renderer's RSSetViewports call is the single place it is applied.
		auto dip_per_px = 96.0f / viewport.dpi;
		auto anchor_x_dip = (ndc_x * 0.5f + 0.5f) * viewport.viewport_width_px * dip_per_px;
		auto anchor_y_dip = (0.5f - ndc_y * 0.5f) * viewport.viewport_height_px * dip_per_px;

		// Apparent size: constant-DIP roots keep their authored extent, world-unit roots convert
		// their extent through the camera so they shrink with distance exactly like geometry.
		auto scale = 1.0f;
		switch (world.sizing)
		{
			case EWorldSizing::ConstantDip:
			{
				scale = 1.0f;
				break;
			}
			case EWorldSizing::WorldUnits:
			{
				scale = world.world_units_per_dip * DipsPerWorldUnit(camera, viewport, view_depth);
				break;
			}
			case EWorldSizing::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("ProjectWorldRoot: unknown EWorldSizing {}", static_cast<int>(world.sizing)));
			}
		}
		if (!std::isfinite(scale) || !(scale > 0.0f))
			return CulledPlacement(policy, world);

		auto w = box_w_dip * scale;
		auto h = box_h_dip * scale;
		auto rect = Rect{
			anchor_x_dip + AnchorOffset(world.anchor_h, w),
			anchor_y_dip + AnchorOffset(world.anchor_v, h),
			w,
			h,
		};

		// A root entirely outside the viewport is culled so it contributes no draw items and is
		// reported Offscreen; partial overlap is left to the host scissor as for screen roots.
		// The viewport is the origin-anchored box a screen root of default size would occupy.
		auto view_right = viewport.viewport_width_px * dip_per_px;
		auto view_bottom = viewport.viewport_height_px * dip_per_px;
		if (rect.x >= view_right || rect.x + rect.w <= 0.0f || rect.y >= view_bottom || rect.y + rect.h <= 0.0f)
			return CulledPlacement(policy, world);

		return RootPlacement{
			.policy = policy,
			.visible = 1,
			.rect = rect,
			.scale = scale,
			.view_depth = view_depth,
			.clip_depth = NormalisedDeviceDepth(camera, view_depth - world.depth_offset),
			.occlusion_min_opacity = world.occlusion_min_opacity,
			.occlusion_fade_depth = world.occlusion_fade_depth,
			.occlusion_depth_bias = world.occlusion_depth_bias,
		};
	}
}
