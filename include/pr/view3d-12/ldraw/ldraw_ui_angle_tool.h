//***************************************************************************************************
// Ldr Measure
//  Copyright (c) Rylogic Ltd 2010
//***************************************************************************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/common/ldraw.h"
#include "pr/gui/wingui.h"

namespace pr::rdr12::ldraw
{
	// A UI for measuring angles within a 3D environment
	struct alignas(16) AngleUI :gui::Form
	{
		// Callback function for reading a world space point
		using ReadPointCB = v4(__stdcall*)(void* ctx);

	private:

		enum { ID_BTN_ORIG = 100, ID_BTN_SET0, ID_BTN_SET1, ID_TB_VALUES };

		// Members
		Guid m_context_id;           // A graphics context Id
		ReadPointCB m_read_point_cb; // The callback for reading a world space point
		void* m_read_point_ctx;      // Context for the callback function
		Renderer& m_rdr;             // Reference to the renderer
		LdrObjectPtr m_gfx;          // Graphics created by this tool
		gui::Button m_btn_orig;      // Set the origin for angle measurement
		gui::Button m_btn_set0;      // Set the point 0 for angle measurement
		gui::Button m_btn_set1;      // Set the point 1 for angle measurement
		gui::TextBox m_tb_values;    // The measured values
		v4 m_origin;                 // The angle apex
		v4 m_point0;                 // The point0
		v4 m_point1;                 // The end of the measurement

	public:

		AngleUI(HWND parent, ReadPointCB read_point_cb, void* ctx, Renderer& rdr)
			:Form(Params<>()
				.parent(parent).name("ldr-angle-ui").title(L"Measure Angles")
				.wh(220, 186).style_ex('+', WS_EX_TOOLWINDOW)
				.hide_on_close(true).pin_window(true)
				.wndclass(RegisterWndClass<AngleUI>()))
			, m_context_id(GenerateGUID())
			, m_read_point_cb(read_point_cb)
			, m_read_point_ctx(ctx)
			, m_rdr(rdr)
			, m_gfx()
			, m_btn_orig(gui::Button::Params<>().parent(this_).name("btn-orig").id(ID_BTN_ORIG).wh(50, 20).xy(0, 0).anchor(EAnchor::TopLeft).text(L"Origin"))
			, m_btn_set0(gui::Button::Params<>().parent(this_).name("btn-set0").id(ID_BTN_SET0).wh(50, 20).xy(Left | RightOf | ID_BTN_ORIG, 0).anchor(EAnchor::TopLeft).text(L"Point 0"))
			, m_btn_set1(gui::Button::Params<>().parent(this_).name("btn-set1").id(ID_BTN_SET1).wh(50, 20).xy(Left | RightOf | ID_BTN_SET0, 0).anchor(EAnchor::TopLeft).text(L"Point 1"))
			, m_tb_values(gui::TextBox::Params<>().parent(this_).name("tb-values").id(ID_TB_VALUES).wh(Fill, Fill).xy(0, Top | BottomOf | ID_BTN_ORIG).anchor(EAnchor::All).multiline(true))
			, m_origin(v4::Origin())
			, m_point0(v4::Origin())
			, m_point1(v4::Origin())
		{
			CreateHandle();
			m_btn_orig.Click += std::bind(&AngleUI::HandleSetPoint, this, _1, _2);
			m_btn_set0.Click += std::bind(&AngleUI::HandleSetPoint, this, _1, _2);
			m_btn_set1.Click += std::bind(&AngleUI::HandleSetPoint, this, _1, _2);
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
				&btn == &m_btn_orig ? m_origin :
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

			// Create graphics
			if (m_origin != m_point0 || m_origin != m_point1)
			{
				pr::ldraw::Builder ldr;
				auto& group = ldr.Group("Angle");
				group.Line("edge0", 0xFFFFFFFF).line(m_origin, m_point0);
				group.Line("edge1", 0xFFFFFF00).line(m_origin, m_point1);
				group.Line("edge2", 0xFF00FF00).line(m_point0, m_point1);
				auto data = ldr.ToBinary();

				mem_istream<char> src{ reinterpret_cast<char const*>(data.data()), data.size() };
				BinaryReader reader(src, {});
				auto out = Parse(m_rdr, reader, GfxContextId());
				if (!out.m_objects.empty())
					m_gfx = out.m_objects.back();
			}

			auto  e0 = m_point0 - m_origin;
			auto  e1 = m_point1 - m_origin;
			auto  e2 = m_point1 - m_point0;
			float edge0 = Length(e0);
			float edge1 = Length(e1);
			float edge2 = Length(e2);
			float ang = (edge0 < constants<float>::tiny || edge1 < constants<float>::tiny) ? 0.0f : RadiansToDegrees(Acos(Clamp(Dot3(e0, e1) / (edge0 * edge1), -1.0f, 1.0f)));

			// Update the text description
			m_tb_values.Text(FmtS(
				L"edge0: %f\r\n"
				L"edge1: %f\r\n"
				L"edge2: %f\r\n"
				L"angle: %f\uC2B0\r\n"
				, edge0, edge1, edge2
				, ang
			));

			// Notify the measurement data changed
			MeasurementChanged(*this, gui::EmptyArgs());
		}

		// Raised when the measurement data changes
		// MeasurementChanged += [&](AngleUI&,EmptyArgs const&){}
		gui::EventHandler<AngleUI&, gui::EmptyArgs const&> MeasurementChanged;
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
