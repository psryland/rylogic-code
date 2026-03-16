//***************************************************************************************************
// View 3D Panel (Static Library Version)
//  Copyright (c) Rylogic Ltd 2025
//***************************************************************************************************
// A GUI panel that wraps the View3D-12 static library directly (Renderer, Window, Scene),
// rather than going through the V3dWindow / Context / DLL C API layer. This gives consuming
// projects full C++ access to the renderer, scene graph, camera, and LDraw objects — which
// is necessary when the project also needs view3d-12 internals (e.g. GPU compute shaders
// in the physics engine).
//
// Design follows the same pattern as pr::app::Main:
//   Renderer → Window → Scene → Camera (reference to Scene.m_cam)
// Objects are added to the scene each frame via LdrObject::AddToScene().
//
// Usage:
//   - Link view3d-12-static.lib instead of view3d-12.imp
//   - No delay-loaded view3d-12.dll needed
//   - Create LdrObjects via ldraw::Parse(m_rdr, script)
//   - Add objects to the scene in your render loop
#pragma once
#include "pr/gui/wingui.h"
#include "pr/gui/misc.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/camera/camera.h"

namespace pr::gui
{
	struct View3DPanelStatic : Panel
	{
		struct Params : Panel::Params<Params>
		{
			using this_type = typename Params::this_type;

			Colour m_bkgd_colour;
			int m_multisamp;

			Params()
				: m_bkgd_colour(ColourBlack)
				, m_multisamp(4)
			{
				name("view3d")
					.margin(0)
					.selectable();
			}
			this_type& bkgd_colour(Colour c)
			{
				m_bkgd_colour = c;
				return me();
			}
			this_type& multisamp(int samples)
			{
				m_multisamp = samples;
				return me();
			}
		};

		// The renderer owns the D3D12 device, command queues, and resource management
		rdr12::Renderer m_rdr;

		// The window owns the swap chain and back buffers for an HWND
		rdr12::Window m_wnd;

		// The scene owns the camera, drawlists, and render steps.
		// Objects are added via AddToScene() each frame, then Scene.Render() draws them.
		rdr12::Scene m_scene;

		// Convenience reference to the scene camera
		Camera& m_cam;

		// True while a mouse button is held down for navigation
		bool m_nav_active;

		View3DPanelStatic()
			: View3DPanelStatic(Params())
		{}
		explicit View3DPanelStatic(Params const& p)
			: Panel(p)
			, m_rdr(rdr12::RdrSettings(GetModuleHandle(nullptr)))
			, m_wnd(m_rdr, rdr12::WndSettings(CreateHandle(), true, m_rdr.Settings()))
			, m_scene(m_wnd)
			, m_cam(m_scene.m_cam)
			, m_nav_active(false)
		{
			// Set background colour
			m_wnd.BkgdColour(p.m_bkgd_colour);

			// Default camera: perspective, Z-up, looking from the front
			m_cam.FovY(constants<float>::tau_by_8);
			m_cam.Align(v4{0, 0, 1, 0});
			m_cam.Orthographic(false);
			m_cam.LookAt(
				v4(0, -5, 2, 1),
				v4::Origin(),
				v4{0, 0, 1, 0});
		}

		// The parameters used to create this control
		Params const& cp() const
		{
			return m_cp->as<Params>();
		}
		Params& cp()
		{
			return m_cp->as<Params>();
		}

		// Access the D3D12 device (e.g. for GPU physics integration)
		ID3D12Device4* D3DDevice()
		{
			return m_rdr.D3DDevice();
		}

		// Create an LdrObject from an LDraw script string.
		// Returns a ref-counted pointer; the caller owns the reference.
		rdr12::ldraw::LdrObjectPtr CreateObject(std::string_view ldr_script)
		{
			auto result = rdr12::ldraw::Parse(m_rdr, ldr_script);
			if (result.m_objects.empty())
				return {};

			return result.m_objects.front();
		}

		// Render the scene. Call this after populating the drawlists.
		// Follows the same ClearDrawlists → NewFrame → Render → Present pattern as app::Main.
		void DoRender(rdr12::EGpuFlush flush = rdr12::EGpuFlush::Block)
		{
			m_scene.ClearDrawlists();

			// Allow the consuming code to add objects to the scene via OnAddToScene event
			OnAddToScene(*this, m_scene);

			auto& frame = m_wnd.NewFrame();
			m_scene.Render(frame);
			m_wnd.Present(frame, flush);
		}

		// Block until the GPU has finished processing all commands.
		void WaitForGpu()
		{
			m_wnd.WaitForGpu();
			m_scene.ClearDrawlists();
		}

		// Event raised during DoRender after ClearDrawlists, before Scene.Render.
		// Subscribers should call obj->AddToScene(scene) for each object to draw.
		EventHandler<View3DPanelStatic&, rdr12::Scene&> OnAddToScene;

		// Mouse navigation using normalised screen coordinates, same as app::Main
		void OnMouseButton(MouseEventArgs& args) override
		{
			Panel::OnMouseButton(args);
			if (args.m_handled) return;

			m_nav_active = args.m_down;
			auto pt = NormalisePoint(m_hwnd, args.m_point);
			auto op = camera::MouseBtnToNavOp(int(args.m_down ? args.m_button : gui::EMouseKey::None));
			m_cam.MouseControl(pt, op, true);
			Invalidate();
		}
		void OnMouseMove(MouseEventArgs& args) override
		{
			Panel::OnMouseMove(args);
			if (args.m_handled || !m_nav_active) return;

			auto pt = NormalisePoint(m_hwnd, args.m_point);
			auto op = camera::MouseBtnToNavOp(int(args.m_button));
			m_cam.MouseControl(pt, op, false);
			Invalidate();
		}
		void OnMouseWheel(MouseWheelArgs& args) override
		{
			Panel::OnMouseWheel(args);
			if (args.m_handled) return;

			auto pt = NormalisePoint(m_hwnd, args.m_point);
			m_cam.MouseControlZ(pt, args.m_delta, true);
			Invalidate();
		}

		// Render on paint
		void OnPaint(PaintEventArgs& args) override
		{
			DoRender();
			args.m_handled = true;
		}

		// Resize the back buffer when the window changes size
		void OnWindowPosChange(WindowPosEventArgs const& args) override
		{
			Control::OnWindowPosChange(args);
			if (!args.m_before && args.IsResize() && !args.Iconic())
			{
				auto size = iv2{ args.m_wp->cx, args.m_wp->cy };
				m_wnd.BackBufferSize(size, false);
				m_scene.m_viewport.Set(size);
				m_cam.Aspect(1.0 * size.x / size.y);
			}
		}
	};
}
