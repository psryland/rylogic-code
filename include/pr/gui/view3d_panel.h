//***************************************************************************************************
// View 3D
//  Copyright (c) Rylogic Ltd 2009
//***************************************************************************************************
#pragma once

#include "pr/common/to.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"

namespace pr::gui
{
	struct View3DPanel :Panel
	{
		struct Params :Panel::Params<Params>
		{
			using this_type = typename Params::this_type;

			view3d::WindowOptions m_win_opts;
			bool m_show_focus_point; // True if the focus cross hair should be rendered

			Params()
				: m_win_opts()
				,m_show_focus_point(false)
			{
				m_win_opts.multisamp(4);
				name("view3d")
					.margin(0)
					.selectable();
			}
			view3d::WindowOptions const& wnd_opts() const
			{
				return m_win_opts;
			}
			this_type& error_cb(view3d::ReportErrorCB error_cb)
			{
				m_win_opts.error_cb(error_cb);
				return me();
			}
			this_type& error_cb(view3d::ReportErrorCB::FuncCB cb, void* ctx)
			{
				m_win_opts.error_cb({ ctx,  cb });
				return me();
			}
			this_type& gdi_compat(bool on = true)
			{
				m_win_opts.gdi_compat(on);
				return me();
			}
			this_type& multisamp(int samples)
			{
				m_win_opts.multisamp(samples);
				return me();
			}

			this_type& show_focus_point(bool on = true)
			{
				m_show_focus_point = on;
				return me();
			}
		
			static void __stdcall DefaultErrorHandler(void*, char const* msg, char const* filepath, int line, int64_t)
			{
				throw std::runtime_error(std::format("{}({}): {}", filepath, line, msg));
			}
		};

		view3d::DllHandle m_ctx;
		view3d::Window m_win;

		View3DPanel()
			:View3DPanel(Params())
		{}
		explicit View3DPanel(Params const& p)
			:Panel(p)
			,m_ctx(View3D_Initialise(p.m_win_opts.m_error_cb))
			,m_win(View3D_WindowCreate(CreateHandle(), p.wnd_opts()))
		{
			//View3D_CreateDemoScene(m_win);
			View3D_StockObjectVisibleSet(m_win, view3d::EStockObject::FocusPoint, cp().m_show_focus_point);
		}
		~View3DPanel()
		{
			//View3D_DeleteDemoScene();
			if (m_win) View3D_WindowDestroy(m_win);
			if (m_ctx) View3D_Shutdown(m_ctx);
		}

		// The parameters used to create this control (but updated to the current state)
		Params const& cp() const
		{
			return m_cp->as<Params>();
		}
		Params& cp()
		{
			return m_cp->as<Params>();
		}

		// Key shortcuts
		void OnKey(KeyEventArgs& args) override
		{
			Panel::OnKey(args);
			if (args.m_handled) return;

			auto mouse_position = To<view3d::Vec2>(args.point());
			if (View3D_TranslateKey(m_win, args.m_vk_key, mouse_position))
				args.m_handled = true;
		}

		// Mouse navigation
		void OnMouseButton(MouseEventArgs& args) override
		{
			auto op = View3D_MouseBtnToNavOp(int(args.m_button));
			if (View3D_MouseNavigate(m_win, To<view3d::Vec2>(args.m_point), args.m_down ? op : view3d::ENavOp::None, TRUE))
			{
				Invalidate();
				UpdateWindow(m_hwnd);
			}
		}
		void OnMouseMove(MouseEventArgs& args) override
		{
			auto op = View3D_MouseBtnToNavOp(int(args.m_button));
			if (View3D_MouseNavigate(m_win, To<view3d::Vec2>(args.m_point), op, FALSE))
			{
				Invalidate();
				UpdateWindow(m_hwnd);
			}
		}
		void OnMouseWheel(MouseWheelArgs& args) override
		{
			if (View3D_MouseNavigateZ(m_win, To<view3d::Vec2>(args.m_point), args.m_delta, TRUE))
			{
				Invalidate();
				UpdateWindow(m_hwnd);
			}
		}

		// Render the panel
		void OnPaint(PaintEventArgs& args) override
		{
			if (m_win)
			{
				View3D_WindowRender(m_win);
				args.m_handled = true;
				return;
			}
			Panel::OnPaint(args);
		}

		// Handle window size changing starting or stopping
		void OnWindowPosChange(WindowPosEventArgs const& args) override
		{
			Control::OnWindowPosChange(args);
			if (!args.m_before && args.IsResize() && !args.Iconic())
				View3D_WindowBackBufferSizeSet(m_win, { args.m_wp->cx, args.m_wp->cy }, FALSE);
		}
	};
}
namespace pr
{
	template <> struct Convert<view3d::Vec2, gui::Point>
	{
		static view3d::Vec2 Func(gui::Point const& v)
		{
			return view3d::Vec2{float(v.x), float(v.y)};
		}
	};
	template <> struct Convert<gui::Point, view3d::Vec2>
	{
		static gui::Point Func(view3d::Vec2 const& v)
		{
			return gui::Point(long(v.x), long(v.y));
		}
	};
}
