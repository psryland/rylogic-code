//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/shadow_caster.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "view3d-12/src/render/render_smap.h"

namespace pr::rdr12
{
	// Shadow caster constructor
	ShadowCaster::ShadowCaster(Texture2DPtr smap, Light const& light, int size)
		: m_params()
		, m_light(&light)
		, m_smap(smap)
		, m_size(size)
	{}

	// Update the projection parameters for the given scene
	void ShadowCaster::UpdateParams(Scene const& scene, BBox ws_bounds)
	{
		auto const& c2w = scene.m_cam.CameraToWorld();
		auto l2w = m_light->LightToWorld(ws_bounds.Centre(), 0.5f * ws_bounds.Diametre(), c2w);
		m_params.m_l2w = l2w;

		constexpr int UseLiSPSM = 0;
		if constexpr (UseLiSPSM)
		{
			// Create a light space perspective (LSP) transform that skews the scene to give detail
			// near the camera. The projection must be perpendicular to the light direction and the
			// camera must be in front of the LSP near plane.

			// Camera near/far plane distances, and LiSPSM "warp" factor lsp_zn
			auto cam_zn = scene.m_cam.Near(false);
			auto cam_zf = scene.m_cam.Far(false);
			auto lsp_zn = cam_zn + sqrt(cam_zn * cam_zf);

			// Grow the bounds to include the camera near plane
			auto ws_bounds_cam = ws_bounds;
			Grow(ws_bounds_cam, c2w.pos - s_cast<float>(cam_zn) * c2w.z);

			// Create a LSP to world transform. This is the space that the skew projection is in.
			// The projection clip planes are parallel to the light direction. The origin of LSP
			// space is 'lsp_zn * lsp_z' from the nearest point of 'ws_bounds_cam'
			auto lsp_z = Perpendicular(l2w.z, -c2w.z);
			auto lsp_rot = m3x4::Rotation(-v4::ZAxis(), lsp_z);
			auto lsp_dz = Dot(Abs(lsp_z), ws_bounds_cam.Radius());
			auto lsp_origin = ws_bounds_cam.Centre() - s_cast<float>(lsp_zn + lsp_dz) * lsp_z;
			auto lsp2w = m4x4{lsp_rot, lsp_origin};
			auto w2lsp = InvertAffine(lsp2w);

			// Get the scene+camera bounds in LSP space.
			auto lsp_bounds_cam = w2lsp * ws_bounds_cam;

			// Create a frustum (i.e. projection) that maps the lsp scene bounds to the unit cube.
			// The amount of perspective warp is controlled by offsetting both zn and zf by a fixed amount.
			// When the light is at 90 deg to the camera, the optimal zn value is: cam_zn + sqrt(cam_zn * cam_zf)
			auto sz = Max(lsp_bounds_cam.SizeX(), lsp_bounds_cam.SizeY());
			auto lsp = m4x4::ProjectionPerspective(sz, sz, s_cast<float>(lsp_zn), s_cast<float>(lsp_zn + 2.0f*lsp_dz), true);

			// Rotate, scale, and shift the unit cube so that it is viewed from the light.
			auto ls_bounds = MulNonAffine(lsp, lsp_bounds_cam);
			{
				#if PR_DBG_SMAP
				ldr::Builder B;
				auto& g = B.Group("ls");
				for (auto& p : Corners(lsp_bounds_cam))
				{
					auto v = lsp * p;
					//v = lsp * v;
					v /= v.w;
					g.Sphere("p", 0xFFFFFFFF).r(0.05f).pos(v);
				}
				B.Write("P:\\dump\\smap_lispsm.ldr");
				#endif
			}


			// Because the light direction is perpendicular to the lsp clip planes, the transform
			// from lsp space to light space is just a 90deg rotation, scale, and offset of the unit cube.
			static float Z = 1.0f;
			static AxisId From = AxisId::PosY;
			static AxisId To = AxisId::PosZ;
			static float SX = 1.0f;
			static float SY = 1.0f;
			static float SZ = 1.0f;
			auto lsp2ls = m4x4::Transform(From, To, v4{0, 0, Z, 1}) * m4x4::Scale(SX, SY, SZ, v4Origin);

			// World to perspective skewed light space to light space
			auto w2ls = lsp2ls * lsp * w2lsp;
			m_params.m_w2ls = w2ls;

			// Create a projection that maps the lsp unit cube onto the shadow map
			static float W = 1.0f;
			static float H = 1.0f;
			auto ls2s = m4x4::ProjectionOrthographic(W, H, -5.0f, 5.0f, true);
			m_params.m_ls2s = ls2s;

			#if PR_DBG_SMAP
			{
				ldr::Builder b;
				b.Box("scene_bounds", 0xFF0000FF).bbox(ws_bounds_cam).wireframe();
				b.Frustum("camera_view", 0xFF00FFFF).nf(scene.m_cam.Near(false), scene.m_cam.FocusDist() * 2).fov(scene.m_cam.FovY(), scene.m_cam.Aspect()).o2w(c2w).wireframe().axis(AxisId::NegZ);
				b.Frustum("lsp", 0x4000FF00).proj(lsp).o2w(InvertAffine(w2lsp));
				auto& blight = b.Add<LdrLight>("light", 0xFFFFFF00).light(*m_light).scale(scene.m_cam.FocusDist() * 0.05).o2w(l2w);
		//		blight.Box("light_bounds", 0xFFFFFF00).bbox(m_params.m_bounds).wireframe();
		//		blight.Frustum("light_proj", 0xFFFF00FF).proj(ls2s).wireframe().o2w(l2w);
				b.Write("P:\\dump\\smap_view.ldr");
			}
			#endif
			
			//{
			//	ldr::Builder B;
			//	auto& g = B.Group("ls");
			//	for (auto& p : Corners(ws_bounds_cam))
			//	{
			//		auto v = w2lsp * p;
			//		v = lsp * v;
			//		v /= v.w;
			//		g.Sphere("p", 0xFFFFFFFF).r(0.05f).pos(v);
			//	}
			//	B.Write("P:\\dump\\smap_lispsm.ldr");
			//}

		}
		else
		{
			// World to light space
			auto w2ls = InvertAffine(l2w);
			m_params.m_w2ls = w2ls;

			// Get the scene bounds in light space
			// Inflate the bounds slightly so that the edge of the smap is avoided
			auto ls_bounds = w2ls * ws_bounds;
			ls_bounds.m_radius = Max(ls_bounds.m_radius * 1.01f, v4::TinyF().w0());
			m_params.m_bounds = ls_bounds;

			// Create a projection that encloses the scene bounds. This is basically "c2s"
			auto zn = Abs(ls_bounds.Centre().z + ls_bounds.Radius().z);
			auto zf = Abs(ls_bounds.Centre().z - ls_bounds.Radius().z);
			if (zf - zn < maths::tinyf) zf = zn + 1.0f;
			auto ls2s = m_light->Projection(zn, zf, ls_bounds.SizeX(), ls_bounds.SizeY(), Length(ls_bounds.Centre() - l2w.pos));
			m_params.m_ls2s = ls2s;

			// Output the camera, light position, scene bounds, and smap projection.
			#if PR_DBG_SMAP
			{
				ldr::Builder b;
				b.Box("scene_bounds", 0xFF0000FF).bbox(ws_bounds).wireframe();
				b.Frustum("camera_view", 0xFF00FFFF).nf(scene.m_cam.Near(false), scene.m_cam.FocusDist() * 2).fov(scene.m_cam.FovY(), scene.m_cam.Aspect()).o2w(c2w).wireframe().axis(AxisId::NegZ);
				auto& blight = b.Add<LdrLight>("light", 0xFFFFFF00).light(*m_light).scale(scene.m_cam.FocusDist() * 0.05f).o2w(l2w);
				blight.Box("light_bounds", 0xFFFFFF00).bbox(m_params.m_bounds).wireframe();
				blight.Frustum("light_proj", 0xFFFF00FF).proj(ls2s).wireframe();
				b.Write("P:\\dump\\smap_view.ldr");
			}
			#endif
		}
	}
}