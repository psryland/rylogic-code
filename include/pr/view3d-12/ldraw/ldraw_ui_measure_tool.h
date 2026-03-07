//***************************************************************************************************
// Ldr Measure
//  Copyright (c) Rylogic Ltd 2010
//***************************************************************************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"
#include "pr/gui/wingui.h"

namespace pr::rdr12::ldraw
{
	// A UI for measuring distances within a 3D environment
	struct alignas(16) MeasureUI :gui::Form
	{
		// Callback function for reading a world space point
		using ReadPointCB = v4(__stdcall*)(void* ctx);

	private:

		enum { ID_BTN_SET0 = 100, ID_BTN_SET1, ID_TB_VALUES };

		// Members
		Guid         m_context_id;     // A graphics context Id
		ReadPointCB  m_read_point_cb;  // The callback for reading a world space point
		void* m_read_point_ctx; // Context for the callback function
		Renderer& m_rdr;            // Reference to the renderer
		LdrObjectPtr m_gfx;            // Graphics created by this tool
		gui::Button  m_btn_set0;       // Set the start point for measuring
		gui::Button  m_btn_set1;       // Set the end point for measuring
		gui::TextBox m_tb_values;      // The measured values
		v4           m_point0;         // The start of the measurement
		v4           m_point1;         // The end of the measurement

	public:

		MeasureUI(HWND parent, ReadPointCB read_point_cb, void* ctx, Renderer& rdr)
			:Form(Params<>()
				.parent(parent).name("ldr-measure-ui").title(L"Measure Distances")
				.wh(300, 150).style_ex('+', WS_EX_TOOLWINDOW)
				.hide_on_close(true).pin_window(true)
				.wndclass(RegisterWndClass<MeasureUI>()))
			, m_context_id(GenerateGUID())
			, m_read_point_cb(read_point_cb)
			, m_read_point_ctx(ctx)
			, m_rdr(rdr)
			, m_gfx()
			, m_btn_set0(gui::Button::Params<>().parent(this_).name("btn-set0").id(ID_BTN_SET0).xy(0, 0).anchor(EAnchor::TopLeft).text(L"Set Point 0"))
			, m_btn_set1(gui::Button::Params<>().parent(this_).name("btn-set1").id(ID_BTN_SET1).xy(Left | RightOf | ID_BTN_SET0, 0).anchor(EAnchor::TopLeft).text(L"Set Point 1"))
			, m_tb_values(gui::TextBox::Params<>().parent(this_).name("tb-values").id(ID_TB_VALUES).wh(Fill, Fill).xy(0, Top | BottomOf | ID_BTN_SET0).anchor(EAnchor::All).multiline(true))
			, m_point0(v4::Origin())
			, m_point1(v4::Origin())
		{
			CreateHandle();
			m_btn_set0.Click += std::bind(&MeasureUI::HandleSetPoint, this, _1, _2);
			m_btn_set1.Click += std::bind(&MeasureUI::HandleSetPoint, this, _1, _2);
			UpdateMeasurementInfo();
		}

		// Set the callback function used to read points in the 3d environment
		void SetReadPoint(ReadPointCB cb, void* ctx)
		{
			m_read_point_cb = cb;
			m_read_point_ctx = ctx;
		}

		// Graphics associated with the this measure tool
		LdrObjectPtr Gfx() const
		{
			return m_gfx;
		}

		// The context id for graphics objects belonging to this measurement UI
		Guid GfxContextId() const
		{
			return m_context_id;
		}

		// Handle a 'Set Point' button being clicked
		void HandleSetPoint(gui::Button& btn, gui::EmptyArgs const&)
		{
			auto dummy = v4{};
			auto& point =
				&btn == &m_btn_set0 ? m_point0 :
				&btn == &m_btn_set1 ? m_point1 :
				dummy;

			// Read the 3D point from the scene
			point = m_read_point_cb(m_read_point_ctx);

			// Update the measurement data
			UpdateMeasurementInfo();
		}

		// Update the text in the measurement details text box
		void UpdateMeasurementInfo()
		{
			using namespace math;

			// Remove any existing graphics
			m_gfx = nullptr;

			// Create graphics for the two measurement points
			if (m_point0 != m_point1)
			{
				auto p0 = v4(m_point1.x, m_point0.y, m_point0.z, 1.0f);
				auto p1 = v4(m_point1.x, m_point1.y, m_point0.z, 1.0f);

				Builder ldr;
				auto& group = ldr.Group("Measurement");
				group.Line("dist", 0xFFFFFFFF).line(m_point0, m_point1);
				group.Line("distX", 0xFFFF0000).line(m_point0, p0);
				group.Line("distY", 0xFF00FF00).line(p0, p1);
				group.Line("distZ", 0xFF0000FF).line(p1, m_point1);
				auto data = ldr.ToBinary();

				mem_istream<char> src{ data.data(), data.size() };
				BinaryReader reader(src, {});
				auto out = Parse(m_rdr, reader, GfxContextId());
				if (!out.m_objects.empty())
					m_gfx = out.m_objects.back();
			}

			auto dx = m_point1.x - m_point0.x;
			auto dy = m_point1.y - m_point0.y;
			auto dz = m_point1.z - m_point0.z;
			auto len = Len(dx, dy, dz);
			auto dxy = Len(dx, dy);
			auto dyz = Len(dy, dz);
			auto dzx = Len(dz, dx);
			auto angx = dyz > constants<float>::tiny && fabs(dy) > constants<float>::tiny ? RadiansToDegrees(Angle(dyz, fabs(dy), fabs(dz))) : 0.0f;
			auto angy = dzx > constants<float>::tiny && fabs(dx) > constants<float>::tiny ? RadiansToDegrees(Angle(dzx, fabs(dx), fabs(dz))) : 0.0f;
			auto angz = dxy > constants<float>::tiny && fabs(dx) > constants<float>::tiny ? RadiansToDegrees(Angle(dxy, fabs(dx), fabs(dy))) : 0.0f;

			// Update the text description
			m_tb_values.Text(FmtS(
				L"     sep: %f %f %f  (%f) \r\n"
				L"xy,yz,zx: %f %f %f \r\n"
				L" ang (\uC2B0): %f %f %f \r\n"
				, dx, dy, dz, len
				, dxy, dyz, dzx
				, angx, angy, angz));

			// Notify the measurement data changed
			MeasurementChanged(*this, gui::EmptyArgs());
		}

		// Raised when the measurement data changes
		// MeasurementChanged += [&](MeasureUI&,EmptyArgs const&){}
		gui::EventHandler<MeasureUI&, gui::EmptyArgs const&> MeasurementChanged;
	};
}

// Add a manifest dependency on common controls version 6
#if defined _M_IX86
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
