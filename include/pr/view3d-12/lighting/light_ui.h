//************************************
// Lighting Dialog
//  Copyright (c) Rylogic Ltd 2009
//************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/lighting/light.h"
#include "pr/gui/wingui.h"

namespace pr::rdr12
{
	class LightingUI :public gui::Form
	{
		enum
		{
			ID_RADIO_AMBIENT = 100, ID_RADIO_DIRECTIONAL, ID_RADIO_POINT, ID_RADIO_SPOT,
			ID_EDIT_POSITION, ID_EDIT_DIRECTION, ID_CHECK_CAMERA_RELATIVE,
			ID_EDIT_RANGE, ID_EDIT_FALLOFF, ID_EDIT_SHADOW_RANGE,
			ID_EDIT_AMBIENT, ID_EDIT_DIFFUSE, ID_EDIT_SPECULAR, ID_EDIT_SPECULAR_POWER, ID_EDIT_INTENSITY,
			ID_EDIT_INNER_ANGLE, ID_EDIT_OUTER_ANGLE,
		};

		gui::Panel       m_panel_btns;
		gui::Button      m_btn_preview;
		gui::Button      m_btn_cancel;
		gui::Button      m_btn_ok;

		gui::GroupBox    m_grp_light_type;
		gui::Button      m_rdo_ambient;
		gui::Button      m_rdo_directional;
		gui::Button      m_rdo_point;
		gui::Button      m_rdo_spot;

		gui::TextBox     m_tb_position;
		gui::TextBox     m_tb_direction;
		gui::Button      m_chk_cam_rel;
		gui::TextBox     m_tb_range;
		gui::TextBox     m_tb_falloff;
		gui::TextBox     m_tb_shadow_range;
		gui::TextBox     m_tb_ambient;
		gui::TextBox     m_tb_diffuse;
		gui::TextBox     m_tb_specular;
		gui::TextBox     m_tb_spec_power;
		gui::TextBox     m_tb_intensity;
		gui::TextBox     m_tb_spot_inner;
		gui::TextBox     m_tb_spot_outer;

		gui::Label       m_lbl_position;
		gui::Label       m_lbl_direction;
		gui::Label       m_lbl_range;
		gui::Label       m_lbl_falloff;
		gui::Label       m_lbl_shadow_range;
		gui::Label       m_lbl_ambient;
		gui::Label       m_lbl_diffuse;
		gui::Label       m_lbl_specular;
		gui::Label       m_lbl_spec_power;
		gui::Label       m_lbl_intensity;
		gui::Label       m_lbl_inner;
		gui::Label       m_lbl_outer;

		gui::ToolTip     m_tt;

	public:

		// The light we're displaying properties for
		Light m_light;

		// Show the lighting UI with preview callback: Preview(Light const& light, bool cam_rel);
		LightingUI(HWND parent, Light const& light)
			:Form(Params<>().dlg()
				.parent(parent)
				.name("rdr-lighting-ui")
				.title(L"Lighting Options")
				.wh(300,440)
				.resizeable(false)
				.style_ex('+',WS_EX_TOOLWINDOW)
				.start_pos(EStartPosition::CentreParent)
				.wndclass(RegisterWndClass<LightingUI>()))

			,m_panel_btns        (gui::Panel ::Params<>().parent(this_).wh(Fill, gui::Button::DefH*3/2).dock(EDock::Bottom))
			,m_btn_preview       (gui::Button::Params<>().parent(&m_panel_btns).text(L"Preview").id(IDRETRY ).dock(EDock::Left))
			,m_btn_cancel        (gui::Button::Params<>().parent(&m_panel_btns).text(L"Cancel" ).id(IDCANCEL).dock(EDock::Right))
			,m_btn_ok            (gui::Button::Params<>().parent(&m_panel_btns).text(L"OK"     ).id(IDOK    ).dock(EDock::Right))

			,m_grp_light_type    (gui::GroupBox::Params<>().parent(this_)          .text(L"Light Type" ).wh(84, 128).xy(3, 3))
			,m_rdo_ambient       (gui::Button::Params<>().parent(&m_grp_light_type).text(L"Ambient"    ).xy(0, 12                               ).radio().id(ID_RADIO_AMBIENT    ).margin(3, 0, 0, 0))
			,m_rdo_directional   (gui::Button::Params<>().parent(&m_grp_light_type).text(L"Directional").xy(0, Top|BottomOf|ID_RADIO_AMBIENT    ).radio().id(ID_RADIO_DIRECTIONAL).margin(3, 0, 0, 0))
			,m_rdo_point         (gui::Button::Params<>().parent(&m_grp_light_type).text(L"Point"      ).xy(0, Top|BottomOf|ID_RADIO_DIRECTIONAL).radio().id(ID_RADIO_POINT      ).margin(3, 0, 0, 0))
			,m_rdo_spot          (gui::Button::Params<>().parent(&m_grp_light_type).text(L"Spot"       ).xy(0, Top|BottomOf|ID_RADIO_POINT      ).radio().id(ID_RADIO_SPOT       ).margin(3, 0, 0, 0))

			,m_tb_position       (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_POSITION        ).w(119 ).xy(-1, 0                                    ).anchor(EAnchor::TopRight))
			,m_tb_direction      (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_DIRECTION       ).w(119 ).xy(-1, Top|BottomOf|ID_EDIT_POSITION        ).anchor(EAnchor::TopRight))
			,m_chk_cam_rel       (gui::Button ::Params<>().parent(this_).id(ID_CHECK_CAMERA_RELATIVE).w(120 ).xy(-1, Top|BottomOf|ID_EDIT_DIRECTION       ).anchor(EAnchor::TopRight).text(L"Camera Relative:").chk_box().style('+',BS_LEFTTEXT))
			,m_tb_range          (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_RANGE           ).w(75  ).xy(-1, Top|BottomOf|ID_CHECK_CAMERA_RELATIVE).anchor(EAnchor::TopRight))
			,m_tb_falloff        (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_FALLOFF         ).w(75  ).xy(-1, Top|BottomOf|ID_EDIT_RANGE           ).anchor(EAnchor::TopRight))
			,m_tb_shadow_range   (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_SHADOW_RANGE    ).w(75  ).xy(-1, Top|BottomOf|ID_EDIT_FALLOFF         ).anchor(EAnchor::TopRight))
			,m_tb_ambient        (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_AMBIENT         ).w(119 ).xy(-1, Top|BottomOf|ID_EDIT_SHADOW_RANGE    ).anchor(EAnchor::TopRight))
			,m_tb_diffuse        (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_DIFFUSE         ).w(119 ).xy(-1, Top|BottomOf|ID_EDIT_AMBIENT         ).anchor(EAnchor::TopRight))
			,m_tb_specular       (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_SPECULAR        ).w(119 ).xy(-1, Top|BottomOf|ID_EDIT_DIFFUSE         ).anchor(EAnchor::TopRight))
			,m_tb_spec_power     (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_SPECULAR_POWER  ).w(75  ).xy(-1, Top|BottomOf|ID_EDIT_SPECULAR        ).anchor(EAnchor::TopRight))
			,m_tb_intensity      (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_INTENSITY       ).w(75  ).xy(-1, Top|BottomOf|ID_EDIT_SPECULAR_POWER  ).anchor(EAnchor::TopRight))
			,m_tb_spot_inner     (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_INNER_ANGLE     ).w(39  ).xy(-1, Top|BottomOf|ID_EDIT_INTENSITY       ).anchor(EAnchor::TopRight))
			,m_tb_spot_outer     (gui::TextBox::Params<>().parent(this_).id(ID_EDIT_OUTER_ANGLE     ).w(39  ).xy(-1, Top|BottomOf|ID_EDIT_INNER_ANGLE     ).anchor(EAnchor::TopRight))

			,m_lbl_position      (gui::Label::Params<>().parent(this_).text(L"Position:"          ).xy(Right|LeftOf|ID_EDIT_POSITION      , Centre|CentreOf|ID_EDIT_POSITION      ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_direction     (gui::Label::Params<>().parent(this_).text(L"Direction:"         ).xy(Right|LeftOf|ID_EDIT_DIRECTION     , Centre|CentreOf|ID_EDIT_DIRECTION     ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_range         (gui::Label::Params<>().parent(this_).text(L"Range:"             ).xy(Right|LeftOf|ID_EDIT_RANGE         , Centre|CentreOf|ID_EDIT_RANGE         ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_falloff       (gui::Label::Params<>().parent(this_).text(L"Falloff:"           ).xy(Right|LeftOf|ID_EDIT_FALLOFF       , Centre|CentreOf|ID_EDIT_FALLOFF       ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_shadow_range  (gui::Label::Params<>().parent(this_).text(L"Shadow Range:"      ).xy(Right|LeftOf|ID_EDIT_SHADOW_RANGE  , Centre|CentreOf|ID_EDIT_SHADOW_RANGE  ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_ambient       (gui::Label::Params<>().parent(this_).text(L"Ambient (RRGGBB):"  ).xy(Right|LeftOf|ID_EDIT_AMBIENT       , Centre|CentreOf|ID_EDIT_AMBIENT       ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_diffuse       (gui::Label::Params<>().parent(this_).text(L"Diffuse (RRGGBB):"  ).xy(Right|LeftOf|ID_EDIT_DIFFUSE       , Centre|CentreOf|ID_EDIT_DIFFUSE       ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_specular      (gui::Label::Params<>().parent(this_).text(L"Specular (RRGGBB):" ).xy(Right|LeftOf|ID_EDIT_SPECULAR      , Centre|CentreOf|ID_EDIT_SPECULAR      ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_spec_power    (gui::Label::Params<>().parent(this_).text(L"Specular Power:"    ).xy(Right|LeftOf|ID_EDIT_SPECULAR_POWER, Centre|CentreOf|ID_EDIT_SPECULAR_POWER).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_intensity     (gui::Label::Params<>().parent(this_).text(L"Intensity:"         ).xy(Right|LeftOf|ID_EDIT_INTENSITY     , Centre|CentreOf|ID_EDIT_INTENSITY     ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_inner         (gui::Label::Params<>().parent(this_).text(L"Spot Angles: Inner:").xy(Right|LeftOf|ID_EDIT_INNER_ANGLE   , Centre|CentreOf|ID_EDIT_INNER_ANGLE   ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))
			,m_lbl_outer         (gui::Label::Params<>().parent(this_).text(L"Outer:"             ).xy(Right|LeftOf|ID_EDIT_OUTER_ANGLE   , Centre|CentreOf|ID_EDIT_OUTER_ANGLE   ).style('+', SS_LEFT).style_ex('+',WS_EX_RIGHT).anchor(EAnchor::TopRight))

			,m_tt                (gui::ToolTip::Params<>().parent(this_))

			,m_light(light)
			,Commit()
			,Preview()
		{
			CreateHandle();

			m_rdo_ambient.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				m_light.m_type = ELight::Ambient;
				UpdateUI();
			};
			m_rdo_directional.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				m_light.m_type = ELight::Directional;
				UpdateUI();
			};
			m_rdo_point.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				m_light.m_type = ELight::Point;
				UpdateUI();
			};
			m_rdo_spot.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				m_light.m_type = ELight::Spot;
				UpdateUI();
			};

			m_btn_preview.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				ReadValues();
				Preview(*this, m_light);
			};
			m_btn_cancel.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				Close(EDialogResult::Cancel);
			};
			m_btn_ok.Click += [&](gui::Button&, gui::EmptyArgs const&)
			{
				Close(EDialogResult::Ok);
			};

			//m_tt.Create(m_hWnd, 0, 0, TTS_NOPREFIX);//|TTS_BALLOON
			//
			//CToolInfo ti(TTF_IDISHWND | TTF_SUBCLASS, m_hWnd, 0, 0, MAKEINTRESOURCE(IDD));
			//m_tt.AddTool(&ti);

			//m_tt.SetDelayTime(TTDT_INITIAL, 1000);
			//m_tt.SetDelayTime(TTDT_AUTOPOP, 10000);
			//m_tt.SetDelayTime(TTDT_RESHOW, 1000);
			//
			//m_tt.SetMaxTipWidth(300);
			//m_tt.SetTipBkColor(0xe0e0e0);
			//m_tt.SetTipTextColor(0x000000);
			//
			//m_tt.AddTool(m_edit_position    , "The position of the light in world space or camera space if 'Camera Relative' is checked");
			//m_tt.AddTool(m_edit_direction   , "The light direction in world space or camera space if 'Camera Relative' is checked");
			//m_tt.AddTool(m_check_cam_rel    , "Check to have the light move with the camera");
			//m_tt.AddTool(m_edit_range       , "The maximum range of the light.");
			//m_tt.AddTool(m_edit_falloff     , "Controls the light attenuation with distance. 0 means no attenuation");
			//m_tt.AddTool(m_edit_shadow_range, "");
			//m_tt.AddTool(m_edit_ambient     , "The ambient light colour");
			//m_tt.AddTool(m_edit_diffuse     , "The colour of the light emitted from this light");
			//m_tt.AddTool(m_edit_specular    , "The colour of specular reflected light");
			//m_tt.AddTool(m_edit_spec_power  , "Controls the scattering of the specular reflection");
			//m_tt.AddTool(m_edit_spot_inner  , "The solid angle (deg) of maximum intensity for a spot light");
			//m_tt.AddTool(m_edit_spot_outer  , "The solid angle (deg) of zero intensity for a spot light");
			//m_tt.Activate(TRUE);

			PopulateControls();
			UpdateUI();
		}

		// Handler for when the user commits changes
		EventHandler<LightingUI&, Light const&> Commit;

		// Handler for when a preview is needed
		EventHandler<LightingUI&, Light const&> Preview;

		// Update the values in the controls
		void PopulateControls()
		{
			m_chk_cam_rel      .Checked(m_light.m_cam_relative);
			m_tb_position      .Text(FmtS(L"%3.3f %3.3f %3.3f" ,m_light.m_position.x ,m_light.m_position.y ,m_light.m_position.z));
			m_tb_direction     .Text(FmtS(L"%3.3f %3.3f %3.3f" ,m_light.m_direction.x ,m_light.m_direction.y ,m_light.m_direction.z));
			m_tb_range         .Text(FmtS(L"%3.3f" ,m_light.m_range));
			m_tb_falloff       .Text(FmtS(L"%3.3f" ,m_light.m_falloff));
			m_tb_shadow_range  .Text(FmtS(L"%3.3f" ,m_light.m_cast_shadow));
			m_tb_ambient       .Text(FmtS(L"%6.6X" ,0xFFFFFF & m_light.m_ambient.argb ));
			m_tb_diffuse       .Text(FmtS(L"%6.6X" ,0xFFFFFF & m_light.m_diffuse.argb ));
			m_tb_specular      .Text(FmtS(L"%6.6X" ,0xFFFFFF & m_light.m_specular.argb));
			m_tb_spec_power    .Text(FmtS(L"%d" ,(int)(0.5f + m_light.m_specular_power)));
			m_tb_intensity     .Text(FmtS(L"%3.3f" ,m_light.m_intensity));
			m_tb_spot_inner    .Text(FmtS(L"%d" ,(int)(0.5f + RadiansToDegrees(m_light.m_inner_angle))));
			m_tb_spot_outer    .Text(FmtS(L"%d" ,(int)(0.5f + RadiansToDegrees(m_light.m_outer_angle))));

			Invalidate();
		}

		// Read an validate values from the controls
		void ReadValues()
		{
			// Light type
			if (m_rdo_ambient    .Checked()) m_light.m_type = ELight::Ambient;
			if (m_rdo_directional.Checked()) m_light.m_type = ELight::Directional;
			if (m_rdo_point      .Checked()) m_light.m_type = ELight::Point;
			if (m_rdo_spot       .Checked()) m_light.m_type = ELight::Spot;

			// Transform
			m_light.m_position       = To<v4>(m_tb_position.Text(), 1.0f);
			m_light.m_direction      = Normalise(To<v4>(m_tb_direction.Text(), 0.0f));
			m_light.m_cam_relative   = m_chk_cam_rel.Checked();
			m_light.m_range          = To<float>(m_tb_range.Text());
			m_light.m_falloff        = To<float>(m_tb_falloff.Text());
			m_light.m_cast_shadow    = To<float>(m_tb_shadow_range.Text());
			m_light.m_ambient        = To<Colour32>(m_tb_ambient.Text()).a0();
			m_light.m_diffuse        = To<Colour32>(m_tb_diffuse.Text()).a1();
			m_light.m_specular       = To<Colour32>(m_tb_specular.Text()).a0();
			m_light.m_specular_power = To<float>(m_tb_spec_power.Text());
			m_light.m_intensity      = std::max(0.0f, To<float>(m_tb_intensity.Text()));
			m_light.m_inner_angle    = DegreesToRadians(To<float>(m_tb_spot_inner.Text()));
			m_light.m_outer_angle    = DegreesToRadians(To<float>(m_tb_spot_outer.Text()));
		}

		// Enable/Disable controls
		void UpdateUI()
		{
			m_rdo_ambient     .Checked(m_light.m_type == ELight::Ambient);
			m_rdo_directional .Checked(m_light.m_type == ELight::Directional);
			m_rdo_point       .Checked(m_light.m_type == ELight::Point);
			m_rdo_spot        .Checked(m_light.m_type == ELight::Spot);

			m_tb_position    .Enabled(m_light.m_type == ELight::Point       || m_light.m_type == ELight::Spot);
			m_tb_direction   .Enabled(m_light.m_type == ELight::Directional || m_light.m_type == ELight::Spot);
			m_chk_cam_rel    .Enabled(m_light.m_type != ELight::Ambient);
			m_tb_range       .Enabled(m_light.m_type != ELight::Ambient);
			m_tb_falloff     .Enabled(m_light.m_type != ELight::Ambient);
			m_tb_shadow_range.Enabled(m_light.m_type != ELight::Ambient);
			m_tb_ambient     .Enabled(true);
			m_tb_diffuse     .Enabled(m_light.m_type != ELight::Ambient);
			m_tb_specular    .Enabled(m_light.m_type != ELight::Ambient);
			m_tb_spec_power  .Enabled(m_light.m_type != ELight::Ambient);
			m_tb_intensity   .Enabled(true);
			m_tb_spot_inner  .Enabled(m_light.m_type == ELight::Spot);
			m_tb_spot_outer  .Enabled(m_light.m_type == ELight::Spot);

			Invalidate();
		}
	};
}
