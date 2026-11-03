//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of screen effects: the underwater post effect, ambient occlusion, and output dithering.
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

		// Screen-space ambient occlusion, on geometry with creases, contacts, and a glass pane that must not be darkened
		struct AmbientOcclusionDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::AmbientOcclusionProps m_props;

			explicit AmbientOcclusionDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_props(View3D_AmbientOcclusionGet(ctx.m_window))
			{
				// Boxes on a floor and against a wall make creases; spheres resting on the floor make contact shadows
				AddBoxField(m_scene, 5, 2.5f);
				m_scene.Add("*Plane floor FFD0D0D0 { *Data {30 30} *AxisId {+3} }");
				m_scene.Add("*Box wall FFD0D0D0 { *Data {30 0.5 6} *o2w {*pos {0 8 3}} }");
				m_scene.Add("*Sphere ball0 FFE0E0E0 { *Data {1.0} *o2w {*pos {-3 -6 1.0}} }");
				m_scene.Add("*Sphere ball1 FFE0E0E0 { *Data {0.5} *o2w {*pos {-1.2 -6 0.5}} }");
				m_scene.Add("*Box step0 FFE0E0E0 { *Data {3 3 0.5} *o2w {*pos {4 -6 0.25}} }");
				m_scene.Add("*Box step1 FFE0E0E0 { *Data {2 2 0.5} *o2w {*pos {4 -6 0.75}} }");
				m_scene.Add("*Box step2 FFE0E0E0 { *Data {1 1 0.5} *o2w {*pos {4 -6 1.25}} }");
				m_scene.Add("*Box glass 6080C0FF { *Data {6 0.1 3} *o2w {*pos {0 -3.5 1.5}} }");
				View3D_CameraPositionSet(ctx.m_window, { 2, -16, 7, 1 }, { 0, 0, 0.5f, 1 }, { 0, 0, 1, 0 });
				m_props.m_enabled = TRUE;
				Apply();

				// Effect controls
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Creases and contacts darken. The glass pane must not be darkened.");
				ui.AddCheckBox(L"Ambient occlusion enabled", true, [this](bool on)
				{
					m_props.m_enabled = on ? TRUE : FALSE;
					Apply();
				});
				ui.AddCheckBox(L"Show occlusion only", false, [this](bool on)
				{
					m_props.m_debug_view = on ? TRUE : FALSE;
					Apply();
				});
				ui.AddCheckBox(L"Full resolution", false, [this](bool on)
				{
					m_props.m_resolution = on ? view3d::EAOResolution::Full : view3d::EAOResolution::Half;
					Apply();
				});
				ui.AddSlider(L"Quality (low, medium, high)", 0, 2, static_cast<float>(m_props.m_quality), [this](float v)
				{
					m_props.m_quality = static_cast<view3d::EAOQuality>(static_cast<int>(v + 0.5f));
					Apply();
				}, 2);
				ui.AddSlider(L"Radius", 0.05f, 3.0f, m_props.m_radius, [this](float v)
				{
					m_props.m_radius = v;
					Apply();
				});
				ui.AddSlider(L"Intensity", 0, 4, m_props.m_intensity, [this](float v)
				{
					m_props.m_intensity = v;
					Apply();
				});
				ui.AddSlider(L"Falloff", 0.05f, 1.0f, m_props.m_falloff, [this](float v)
				{
					m_props.m_falloff = v;
					Apply();
				});
				ui.AddSlider(L"Max screen radius", 0.01f, 0.3f, m_props.m_max_screen_radius, [this](float v)
				{
					m_props.m_max_screen_radius = v;
					Apply();
				});
			}

			// Apply the effect settings
			void Apply()
			{
				View3D_AmbientOcclusionSet(m_ctx.m_window, m_props);
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
	std::unique_ptr<IDemo> CreateUnderwaterDemo(DemoContext const& ctx)
	{
		return std::make_unique<UnderwaterDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateAmbientOcclusionDemo(DemoContext const& ctx)
	{
		return std::make_unique<AmbientOcclusionDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateDitherDemo(DemoContext const& ctx)
	{
		return std::make_unique<DitherDemo>(ctx);
	}
}
