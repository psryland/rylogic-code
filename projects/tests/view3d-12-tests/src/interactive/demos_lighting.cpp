//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of lighting features: shadow mapping, skyboxes, and environment map reflections.
#include <cmath>
#include <filesystem>
#include <format>
#include <stdexcept>
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "demo.h"
#include "controls_panel.h"

using namespace pr;

namespace view3d_test
{
	namespace
	{
		// The world space direction toward a sun at 'elevation' above the horizon and 'azimuth' around +Z (in degrees)
		view3d::Vec4 SunDirection(float elevation, float azimuth)
		{
			// Z is up, and an azimuth of zero points along +X
			auto el = elevation * constants<float>::tau / 360.0f;
			auto az = azimuth * constants<float>::tau / 360.0f;
			return view3d::Vec4{ std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el), 0 };
		}

		// A scene of shadow casters on a ground plane, lit by one shadow casting light
		struct ShadowsDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::ShadowSettings m_settings;

			explicit ShadowsDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_settings(View3D_ShadowSettingsGet(ctx.m_window))
			{
				// Casters of different shapes and heights, so shadows overlap and fall across each other
				m_scene.Add("*Plane ground FFC0C0C0 { *Data {20 20} *AxisId {+3} }");
				m_scene.Add("*Box box FF4080FF { *Data {1 1 2} *o2w {*pos {0 0 1}} }");
				m_scene.Add("*Sphere ball FFFF8040 { *Data {0.7} *o2w {*pos {2 1 1.5}} }");
				m_scene.Add("*Cylinder post FF40C040 { *Data {3 0.2} *AxisId {+3} *o2w {*pos {-2 1.5 1.5}} }");
				m_scene.Add("*Box slab FFE0E040 { *Data {3 0.2 0.1} *o2w {*pos {-1 -2 2.5}} }");
				View3D_AmbientSet(ctx.m_window, 0xFF202020);
				SetDirectional();

				// Light type
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Light type:");
				ui.AddButton(L"Directional", [this]
				{
					SetDirectional();
				});
				ui.AddButton(L"Point", [this]
				{
					// A point light above the scene, inside the ring of casters
					demo::EditLight(m_ctx.m_window, 0, [](view3d::Light& light)
					{
						light.m_type = view3d::ELight::Point;
						light.m_position = view3d::Vec4{ 0.5f, -0.5f, 4, 1 };
						light.m_range = 50.0f;
						light.m_falloff = 0.0f;
						light.m_cast_shadow = 1.0f;
					});
				});
				ui.AddButton(L"Spot", [this]
				{
					// A spot light aimed at the origin from one side
					demo::EditLight(m_ctx.m_window, 0, [](view3d::Light& light)
					{
						light.m_type = view3d::ELight::Spot;
						light.m_position = view3d::Vec4{ 4, -4, 5, 1 };
						light.m_direction = view3d::Vec4{ -0.53f, 0.53f, -0.66f, 0 };
						light.m_inner_angle = 0.4f;
						light.m_outer_angle = 0.7f;
						light.m_range = 50.0f;
						light.m_falloff = 0.0f;
						light.m_cast_shadow = 1.0f;
					});
				});

				// Shadow settings
				ui.AddCheckBox(L"Shadows enabled", m_settings.m_max_shadow_lights != 0, [this](bool on)
				{
					m_settings.m_max_shadow_lights = on ? 4 : 0;
					Apply();
				});
				ui.AddSlider(L"Cascades", 1, 4, static_cast<float>(m_settings.m_cascade_count), [this](float v)
				{
					m_settings.m_cascade_count = static_cast<int>(std::lround(v));
					Apply();
				}, 3);
				ui.AddSlider(L"Shadow distance (0 = fit)", 0, 50, m_settings.m_shadow_distance, [this](float v)
				{
					m_settings.m_shadow_distance = v;
					Apply();
				}, 50);
				ui.AddSlider(L"Cascade split blend", 0, 1, m_settings.m_cascade_split_blend, [this](float v)
				{
					m_settings.m_cascade_split_blend = v;
					Apply();
				});
				ui.AddCheckBox(L"Wide filter (7 texels)", m_settings.m_filter_size == 7, [this](bool on)
				{
					m_settings.m_filter_size = on ? 7 : 5;
					Apply();
				});
				ui.AddSlider(L"Depth bias", 0, 1000, static_cast<float>(m_settings.m_depth_bias), [this](float v)
				{
					m_settings.m_depth_bias = static_cast<int>(std::lround(v));
					Apply();
				});
				ui.AddSlider(L"Slope bias", 0, 5, m_settings.m_slope_bias, [this](float v)
				{
					m_settings.m_slope_bias = v;
					Apply();
				});
				ui.AddSlider(L"Normal bias", 0, 5, m_settings.m_normal_bias, [this](float v)
				{
					m_settings.m_normal_bias = v;
					Apply();
				});
			}

			// Make light 0 a low, shadow casting directional light
			void SetDirectional()
			{
				// A low sun gives long shadows that cross several casters
				demo::EditLight(m_ctx.m_window, 0, [](view3d::Light& light)
				{
					light.m_type = view3d::ELight::Directional;
					light.m_direction = view3d::Vec4{ -0.6f, 0.4f, -0.69f, 0 };
					light.m_cast_shadow = 1.0f;
				});
			}

			// Apply the edited shadow settings
			void Apply()
			{
				// The host restores the original settings when the demo closes
				View3D_ShadowSettingsSet(m_ctx.m_window, m_settings);
			}
		};

		// The kind of sky shown by the skybox demo
		enum class ESky
		{
			None,
			CubeMap,
			Procedural,
		};

		// Cube map and procedural skyboxes, with environment map reflections on a row of spheres
		struct SkyboxDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::Object m_sky;
			view3d::CubeMapPtr m_env_map;
			std::filesystem::path m_cube_map_faces;
			ESky m_sky_kind;
			float m_sun_elevation;
			float m_sun_azimuth;
			bool m_reflections;

			explicit SkyboxDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_sky()
				, m_env_map()
				, m_cube_map_faces(ctx.m_assets / "textures/cubemaps/hanger/hanger-??.jpg")
				, m_sky_kind(ESky::None)
				, m_sun_elevation(30.0f)
				, m_sun_azimuth(45.0f)
				, m_reflections(true)
			{
				// The cube map comes from the rylogic assets checkout named in the config file
				auto first_face = ctx.m_assets / "textures/cubemaps/hanger/hanger-px.jpg";
				if (!std::filesystem::exists(first_face))
					throw std::runtime_error(std::format("Cube map not found: {}. Check 'RylogicAssets' in view3d-12-tests.config.json", first_face.string()));

				// The DX cube face layout is left-handed and Y-up. Mapping cube Y to world Z and cube Z to world Y is a mirror that both
				// stands the faces up in this Z-up scene and corrects their handedness. The sky and the reflections share this cube map.
				auto options = view3d::CubeMapOptions{
					.m_cube2w = view3d::Mat4x4{ { 1, 0, 0, 0 }, { 0, 0, 1, 0 }, { 0, 1, 0, 0 }, { 0, 0, 0, 1 } },
					.m_dbg_name = "hanger",
				};
				m_env_map.reset(View3D_CubeMapCreateFromUri(m_cube_map_faces.string().c_str(), options));

				// Spheres of increasing reflectivity show the environment map
				for (int i = 0; i != 5; ++i)
				{
					auto script = std::format("*Sphere ball{} FF808080 {{ *Data {{0.6}} *o2w {{*pos {{{} 0 0.6}}}} }}", i, (i - 2) * 1.5f);
					auto ball = m_scene.Add(script.c_str());
					View3D_ObjectReflectivitySet(ball, 0.25f * i, "");
				}
				m_scene.Add("*Plane ground FF606060 { *Data {12 6} *AxisId {+3} }");
				SetSky(ESky::CubeMap);
				SetReflections(m_reflections);

				// Sky selection
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Sky:");
				ui.AddButton(L"Hanger cube map", [this]
				{
					SetSky(ESky::CubeMap);
				});
				ui.AddButton(L"Procedural sky", [this]
				{
					SetSky(ESky::Procedural);
				});
				ui.AddButton(L"None", [this]
				{
					SetSky(ESky::None);
				});

				// The sun direction drives the procedural sky and the main light
				ui.AddSlider(L"Sun elevation (deg)", -10, 90, m_sun_elevation, [this](float v)
				{
					m_sun_elevation = v;
					UpdateSun();
				});
				ui.AddSlider(L"Sun azimuth (deg)", 0, 360, m_sun_azimuth, [this](float v)
				{
					m_sun_azimuth = v;
					UpdateSun();
				}, 72);
				ui.AddCheckBox(L"Environment map reflections", m_reflections, [this](bool on)
				{
					SetReflections(on);
				});
			}
			~SkyboxDemo()
			{
				// The host clears the window's environment map before destroying the demo, so the cube map is no longer in use
				RemoveSky();
			}

			// Replace the sky object
			void SetSky(ESky kind)
			{
				// Each sky is a separate object, so the old one is removed first
				RemoveSky();
				m_sky_kind = kind;
				switch (kind)
				{
					case ESky::None:
					{
						break;
					}
					case ESky::CubeMap:
					{
						m_sky = View3D_ObjectCreateSkybox("sky", m_env_map.get(), nullptr);
						break;
					}
					case ESky::Procedural:
					{
						m_sky = View3D_ObjectCreateProceduralSky("sky", SunDirection(m_sun_elevation, m_sun_azimuth), view3d::Vec4{ 1, 0.95f, 0.85f, 1 }, 1.0f, nullptr);
						break;
					}
					default:
					{
						throw std::runtime_error("Unknown sky kind");
					}
				}
				if (m_sky != nullptr)
					View3D_WindowAddObject(m_ctx.m_window, m_sky);

				UpdateSun();
			}

			// Remove and delete the current sky object
			void RemoveSky()
			{
				// Not every sky kind has an object
				if (m_sky == nullptr)
					return;

				View3D_ObjectDelete(m_sky);
				m_sky = nullptr;
			}

			// Point the main light away from the sun, and move the procedural sky's sun
			void UpdateSun()
			{
				// The light travels from the sun toward the scene
				auto sun = SunDirection(m_sun_elevation, m_sun_azimuth);
				demo::EditLight(m_ctx.m_window, 0, [&](view3d::Light& light)
				{
					light.m_type = view3d::ELight::Directional;
					light.m_direction = view3d::Vec4{ -sun.x, -sun.y, -sun.z, 0 };
				});
				if (m_sky_kind == ESky::Procedural && !View3D_ObjectUpdateProceduralSky(m_sky, sun, view3d::Vec4{ 1, 0.95f, 0.85f, 1 }, 1.0f))
					throw std::runtime_error("Procedural sky update failed");
			}

			// Use the hanger cube map for reflections, or turn reflections off
			void SetReflections(bool on)
			{
				// The environment map is independent of the visible sky
				m_reflections = on;
				View3D_WindowEnvMapSet(m_ctx.m_window, on ? m_env_map.get() : nullptr);
			}
		};
	}

	// Demo factories
	std::unique_ptr<IDemo> CreateShadowsDemo(DemoContext const& ctx)
	{
		return std::make_unique<ShadowsDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateSkyboxDemo(DemoContext const& ctx)
	{
		return std::make_unique<SkyboxDemo>(ctx);
	}
}
