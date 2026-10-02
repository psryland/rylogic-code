//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of screen effects: far clip fading, the underwater post effect, and output dithering.
#include <format>
#include <stdexcept>
#include <string>
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/utility/conversion.h"
#include "demo.h"
#include "controls_panel.h"

using namespace pr;

namespace view3d_test
{
	namespace
	{
		// Add a square field of boxes, 'count' boxes on a side spaced 'spacing' apart, centred on the origin
		void AddBoxField(demo::SceneObjects& scene, int count, float spacing)
		{
			// One object containing all of the boxes keeps the object count low. Each item is 'width height depth x y z'.
			auto script = std::string("*BoxList field FF80A0C0 { *Data { ");
			for (int j = 0; j != count; ++j)
			{
				for (int i = 0; i != count; ++i)
				{
					// Vary the height so that distant rows remain distinguishable
					auto x = (i - (count - 1) * 0.5f) * spacing;
					auto y = (j - (count - 1) * 0.5f) * spacing;
					auto h = 0.8f + 0.6f * ((i * 7 + j * 3) % 4);
					script += std::format("0.8 0.8 {} {} {} {} ", h, x, y, h * 0.5f);
				}
			}
			script += "} }";
			scene.Add(script.c_str());
		}

		// Objects fade out as they approach the far clip plane, instead of being cut off abruptly
		struct FarClipFadeDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::FarClipFadeProps m_props;
			view3d::Object m_sky;
			float m_far;

			explicit FarClipFadeDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_props{ .m_enabled = TRUE, .m_start_fraction = 0.7f, .m_end_fraction = 0.99f }
				, m_sky()
				, m_far(60.0f)
			{
				// A large field of boxes extends beyond the far plane
				AddBoxField(m_scene, 25, 5.0f);
				m_scene.Add("*Plane ground FF406040 { *Data {200 200} *AxisId {+3} }");
				View3D_CameraPositionSet(ctx.m_window, { 0, -60, 6, 1 }, { 0, 0, 0, 1 }, { 0, 0, 1, 0 });
				Apply();

				// Fade controls
				auto& ui = ctx.m_controls;
				ui.AddCheckBox(L"Fade enabled", true, [this](bool on)
				{
					m_props.m_enabled = on ? TRUE : FALSE;
					Apply();
				});
				ui.AddSlider(L"Start fraction", 0, 1, m_props.m_start_fraction, [this](float v)
				{
					m_props.m_start_fraction = v;
					Apply();
				});
				ui.AddSlider(L"End fraction", 0, 1, m_props.m_end_fraction, [this](float v)
				{
					m_props.m_end_fraction = v;
					Apply();
				});
				ui.AddSlider(L"Far plane", 10, 200, m_far, [this](float v)
				{
					m_far = v;
					Apply();
				}, 190);
				ui.AddCheckBox(L"Procedural sky", false, [this](bool on)
				{
					SetSky(on);
				});
				ui.AddLabel(L"Invalid settings (start > end) are rejected and shown in the log.");
			}
			~FarClipFadeDemo()
			{
				// The sky is not part of the scene objects because it can be toggled
				SetSky(false);
			}

			// Apply the fade properties and the far plane
			void Apply()
			{
				// The DLL rejects invalid fractions and leaves the previous settings in place
				View3D_FarClipFadePropertiesSet(m_ctx.m_window, m_props);
				auto clip = View3D_CameraClipPlanesGet(m_ctx.m_window, view3d::EClipPlanes::Both);
				View3D_CameraClipPlanesSet(m_ctx.m_window, clip.x, m_far, view3d::EClipPlanes::Both);
			}

			// Show or hide a procedural sky behind the fading objects
			void SetSky(bool on)
			{
				// Fading objects should blend into the sky as well as into the background colour
				if (on && m_sky == nullptr)
				{
					m_sky = View3D_ObjectCreateProceduralSky("sky", { 0.5f, 0.3f, 0.8f, 0 }, { 1, 0.95f, 0.85f, 1 }, 1, nullptr);
					View3D_WindowAddObject(m_ctx.m_window, m_sky);
				}
				else if (!on && m_sky != nullptr)
				{
					View3D_ObjectDelete(m_sky);
					m_sky = nullptr;
				}
			}
		};

		// The underwater post effect, with an adjustable water surface
		struct UnderwaterDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::UnderwaterProps m_props;
			view3d::Object m_water;
			float m_surface_height;

			explicit UnderwaterDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_props(View3D_PostEffectUnderwaterGet(ctx.m_window))
				, m_water()
				, m_surface_height(2.0f)
			{
				// A sea floor with objects at different depths, and a translucent water surface for reference
				AddBoxField(m_scene, 9, 3.0f);
				m_scene.Add("*Plane floor FFC0B080 { *Data {40 40} *AxisId {+3} }");
				m_scene.Add("*Sphere buoy FFFF4040 { *Data {0.5} *o2w {*pos {0 0 2}} }");
				m_water = m_scene.Add("*Plane water 4040A0FF { *Data {40 40} *AxisId {+3} }");
				View3D_CameraPositionSet(ctx.m_window, { 0, -12, 1, 1 }, { 0, 0, 0.5f, 1 }, { 0, 0, 1, 0 });
				m_props.m_enabled = TRUE;
				Apply();

				// Effect controls
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Move the camera above and below the surface to see the waterline.");
				ui.AddCheckBox(L"Underwater enabled", true, [this](bool on)
				{
					m_props.m_enabled = on ? TRUE : FALSE;
					Apply();
				});
				ui.AddSlider(L"Visibility", 1, 100, m_props.m_visibility, [this](float v)
				{
					m_props.m_visibility = v;
					Apply();
				}, 99);
				ui.AddSlider(L"Distortion amplitude", 0, 0.02f, m_props.m_distortion_amplitude, [this](float v)
				{
					m_props.m_distortion_amplitude = v;
					Apply();
				});
				ui.AddSlider(L"Distortion frequency", 0, 30, m_props.m_distortion_frequency, [this](float v)
				{
					m_props.m_distortion_frequency = v;
					Apply();
				});
				ui.AddSlider(L"Distortion speed", 0, 2, m_props.m_distortion_speed, [this](float v)
				{
					m_props.m_distortion_speed = v;
					Apply();
				});
				ui.AddSlider(L"Surface height", -2, 10, m_surface_height, [this](float v)
				{
					m_surface_height = v;
					Apply();
				}, 120);
				ui.AddSlider(L"Fade depth", 0, 5, m_props.m_fade_depth, [this](float v)
				{
					m_props.m_fade_depth = v;
					Apply();
				});
			}

			// Apply the effect settings and move the water surface to match
			void Apply()
			{
				// The surface plane is 'n.p + w = 0' with its normal pointing out of the water
				m_props.m_surface = view3d::Vec4{ 0, 0, 1, -m_surface_height };
				View3D_PostEffectUnderwaterSet(m_ctx.m_window, m_props);
				View3D_ObjectO2WSet(m_water, To<view3d::Mat4x4>(m4x4::Translation(0, 0, m_surface_height)), nullptr);
			}
		};

		// Output dithering, shown on smooth gradients where 8-bit banding is visible
		struct DitherDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;

			explicit DitherDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
			{
				// Dim, softly lit surfaces produce slow gradients that band without dithering
				m_scene.Add("*Sphere ball FF404040 { *Data {3} }");
				m_scene.Add("*Plane ground FF303030 { *Data {40 40} *AxisId {+3} *o2w {*pos {0 0 -3}} }");
				View3D_AmbientSet(ctx.m_window, 0xFF000000);
				demo::EditLight(ctx.m_window, 0, [](view3d::Light& light)
				{
					light.m_type = view3d::ELight::Spot;
					light.m_position = view3d::Vec4{ 0, -2, 8, 1 };
					light.m_direction = view3d::Vec4{ 0, 0.24f, -0.97f, 0 };
					light.m_inner_angle = 0.1f;
					light.m_outer_angle = 1.2f;
					light.m_range = 50.0f;
					light.m_falloff = 0.0f;
					light.m_diffuse = 0xFF606060;
					light.m_specular = 0xFF000000;
				});
				View3D_CameraPositionSet(ctx.m_window, { 0, -12, 4, 1 }, { 0, 0, -1, 1 }, { 0, 0, 1, 0 });
				View3D_DitherAmountSet(ctx.m_window, 1.0f);

				// Dither controls
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Compare the banding in the dark gradients with dithering on and off.");
				ui.AddSlider(L"Dither amount", 0, 2, 1.0f, [this](float v)
				{
					View3D_DitherAmountSet(m_ctx.m_window, v);
				});
			}
		};
	}

	// Demo factories
	std::unique_ptr<IDemo> CreateFarClipFadeDemo(DemoContext const& ctx)
	{
		return std::make_unique<FarClipFadeDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateUnderwaterDemo(DemoContext const& ctx)
	{
		return std::make_unique<UnderwaterDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateDitherDemo(DemoContext const& ctx)
	{
		return std::make_unique<DitherDemo>(ctx);
	}
}
