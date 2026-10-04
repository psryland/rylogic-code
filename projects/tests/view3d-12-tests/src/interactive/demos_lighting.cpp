//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of lighting features: shadow mapping, skyboxes, and environment map reflections.
#include <algorithm>
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

		// Interpolate each channel of two ARGB colours. 't' is in [0,1].
		view3d::Colour LerpColour(view3d::Colour a, view3d::Colour b, float t)
		{
			// Blend each 8-bit channel independently, rounding to the nearest value
			auto result = view3d::Colour{};
			for (int shift = 0; shift != 32; shift += 8)
			{
				auto ca = static_cast<float>((a >> shift) & 0xFF);
				auto cb = static_cast<float>((b >> shift) & 0xFF);
				result |= static_cast<view3d::Colour>(ca + (cb - ca) * t + 0.5f) << shift;
			}
			return result;
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

		// The weather map spans this distance (in metres) either side of the origin. It reaches far enough that a storm front first appears on the horizon.
		constexpr float WeatherHalfSize = 60000.0f;

		// The storm band's width, edge softness, and the time (in seconds) it takes to cross the weather map. The crossing is time-lapsed so it can be watched.
		constexpr float StormWidth = 40000.0f;
		constexpr float StormEdge = 15000.0f;
		constexpr float StormCrossingTime = 90.0f;

		// The minimum time (in seconds) between reflection captures while the sky animates. Reflections change slowly, so a few updates per second are enough.
		constexpr double RecapturePeriod = 0.25;

		// Cube map and procedural skyboxes, with environment map reflections on a row of spheres
		struct SkyboxDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::Object m_sky;
			view3d::CubeMapPtr m_env_map;
			view3d::CubeMapPtr m_sky_env_map;
			view3d::WeatherMapPtr m_weather;
			std::vector<view3d::Object> m_balls;
			std::filesystem::path m_cube_map_faces;
			ESky m_sky_kind;
			view3d::Colour m_base_ambient;
			float m_sun_elevation;
			float m_sun_azimuth;
			float m_cloud_cover;
			float m_wind_speed;
			float m_wind_direction;
			float m_storm_progress;
			double m_time;
			double m_since_capture;
			bool m_storm_front;
			bool m_reflections;
			bool m_recapture;

			explicit SkyboxDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_sky()
				, m_env_map()
				, m_sky_env_map()
				, m_weather()
				, m_balls()
				, m_cube_map_faces(ctx.m_assets / "textures/cubemaps/hanger/hanger-??.jpg")
				, m_sky_kind(ESky::None)
				, m_base_ambient(View3D_AmbientGet(ctx.m_window))
				, m_sun_elevation(30.0f)
				, m_sun_azimuth(45.0f)
				, m_cloud_cover(0.4f)
				, m_wind_speed(20.0f)
				, m_wind_direction(45.0f)
				, m_storm_progress(0.0f)
				, m_time(0.0)
				, m_since_capture(0.0)
				, m_storm_front(false)
				, m_reflections(true)
				, m_recapture(false)
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
				m_sky_env_map.reset(View3D_CubeMapCreate(256));

				// The weather map varies the cloud cover across the sky. Its contents are rebuilt each frame while the storm front moves.
				m_weather.reset(View3D_WeatherMapCreate(256, 256, view3d::Vec2{ -WeatherHalfSize, -WeatherHalfSize }, view3d::Vec2{ +WeatherHalfSize, +WeatherHalfSize }));
				if (m_weather == nullptr)
					throw std::runtime_error("Weather map creation failed");

				RebuildWeather();

				// Spheres of increasing reflectivity show the environment map
				for (int i = 0; i != 5; ++i)
				{
					auto script = std::format("*Sphere ball{} FF808080 {{ *Data {{0.6}} *o2w {{*pos {{{} 0 0.6}}}} }}", i, (i - 2) * 1.5f);
					auto ball = m_scene.Add(script.c_str());
					View3D_ObjectReflectivitySet(ball, 0.25f * i, "");
					m_balls.push_back(ball);
				}
				m_scene.Add("*Plane ground FF606060 { *Data {12 6} *AxisId {+3} }");
				SetSky(ESky::CubeMap);

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

				// Clouds: cover ranges from clear, through white cumulus, to dark storm overcast
				ui.AddLabel(L"Clouds:");
				ui.AddSlider(L"Cover", 0, 1, m_cloud_cover, [this](float v)
				{
					m_cloud_cover = v;
					RebuildWeather();
					UpdateSun();
				});
				ui.AddSlider(L"Wind speed (m/s)", 0, 200, m_wind_speed, [this](float v)
				{
					m_wind_speed = v;
					UpdateSun();
				});
				ui.AddSlider(L"Wind direction (deg)", 0, 360, m_wind_direction, [this](float v)
				{
					m_wind_direction = v;
					RebuildWeather();
					UpdateSun();
				}, 72);
				ui.AddCheckBox(L"Storm front", m_storm_front, [this](bool on)
				{
					m_storm_front = on;
					m_storm_progress = 0.0f;
					RebuildWeather();
					UpdateSun();
				});
			}
			~SkyboxDemo()
			{
				// The host clears the window's environment map before destroying the demo, so the cube map is no longer in use
				RemoveSky();
			}

			// Animate the clouds, then recapture the procedural sky's reflections at most once per frame
			void Step(double dt) override
			{
				// A recapture requested by a UI change since the last frame is shown immediately, while animation only refreshes it periodically.
				// Capturing six cube faces of the sky every frame would cost far more than drawing the sky itself.
				auto capture_now = m_recapture;
				m_since_capture += dt;

				// The sky integrates the wind over the change in time, and the storm band advances across the weather map, wrapping around
				m_time += dt;
				if (m_storm_front)
				{
					m_storm_progress = std::fmod(m_storm_progress + static_cast<float>(dt) / StormCrossingTime, 1.0f);
					RebuildWeather();
				}
				if (m_sky_kind == ESky::Procedural && (m_wind_speed != 0 || m_storm_front))
					UpdateSun();

				// Spheres are hidden during the capture so they do not appear in their own reflections
				// Animation requests a recapture every frame, so a request skipped by the period is dropped rather than carried to the next frame
				auto due = m_recapture && (capture_now || m_since_capture >= RecapturePeriod);
				m_recapture = false;
				if (!due)
					return;

				m_since_capture = 0.0;
				for (auto ball : m_balls)
					View3D_ObjectVisibilitySet(ball, FALSE, "");

				View3D_WindowEnvMapCapture(m_ctx.m_window, m_sky_env_map.get(), view3d::Vec4{ 0, 0, 0.6f, 1 });
				for (auto ball : m_balls)
					View3D_ObjectVisibilitySet(ball, TRUE, "");
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
						m_sky = View3D_ObjectCreateProceduralSky("sky", SkySettings(), nullptr);
						if (m_sky != nullptr && !View3D_ObjectProceduralSkyWeatherSet(m_sky, m_weather.get()))
							throw std::runtime_error("Procedural sky weather failed");

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

			// The unit vector of cloud travel
			view3d::Vec2 WindDirection() const
			{
				// Measured from +X toward +Y, matching the sky's wind direction
				auto az = m_wind_direction * constants<float>::tau / 360.0f;
				return view3d::Vec2{ std::cos(az), std::sin(az) };
			}

			// The procedural sky's settings for the current sun, clouds and time
			view3d::ProceduralSkySettings SkySettings() const
			{
				// The sun colour is fixed; the sky adds sunset colours itself
				return view3d::ProceduralSkySettings{
					.m_sun_direction = SunDirection(m_sun_elevation, m_sun_azimuth),
					.m_sun_colour = view3d::Vec4{ 1, 0.95f, 0.85f, 1 },
					.m_sun_intensity = 1.0f,
					.m_cloud_cover = m_cloud_cover,
					.m_wind_speed = m_wind_speed,
					.m_wind_direction = m_wind_direction * constants<float>::tau / 360.0f,
					.m_time = m_time,
				};
			}

			// Fill the weather map with the default cover plus some variation, and add the storm band when enabled
			void RebuildWeather()
			{
				// Noise breaks up the uniform cover so the sky has clearer and cloudier regions. Its amplitude is limited by the cover so a clear sky stays clear.
				View3D_WeatherMapFill(m_weather.get(), m_cloud_cover);
				View3D_WeatherMapAddNoise(m_weather.get(), 15000.0f, std::min(0.15f, m_cloud_cover), 1234);

				// The storm band travels downwind from beyond one edge of the map to beyond the other. The leading front darkens everything
				// upwind of it, and the trailing front restores the default cover upwind of the band.
				if (m_storm_front)
				{
					auto dir = WindDirection();
					auto travel = 2.0f * WeatherHalfSize + StormWidth + 2.0f * StormEdge;
					auto lead = -WeatherHalfSize - StormEdge + m_storm_progress * travel;
					auto trail = lead - StormWidth;
					View3D_WeatherMapAddFront(m_weather.get(), view3d::Vec2{ dir.x * lead, dir.y * lead }, dir, StormEdge, 0.95f);
					View3D_WeatherMapAddFront(m_weather.get(), view3d::Vec2{ dir.x * trail, dir.y * trail }, dir, StormEdge, m_cloud_cover);
				}
				View3D_WeatherMapUpload(m_weather.get());
			}

			// Point the main light away from the sun, dim it under cloud, and update the procedural sky
			void UpdateSun()
			{
				// Only the procedural sky has clouds, so other skies keep the full light
				auto cover = 0.0f;
				if (m_sky_kind == ESky::Procedural)
				{
					auto c2w = View3D_CameraToWorldGet(m_ctx.m_window);
					cover = View3D_WeatherMapCoverAt(m_weather.get(), view3d::Vec2{ c2w.w.x, c2w.w.y }, m_cloud_cover);
				}

				// Thick cloud overhead blocks the direct light, leaving a flat grey ambient
				auto overcast = std::clamp((cover - 0.4f) / 0.6f, 0.0f, 1.0f);
				auto ambient = LerpColour(m_base_ambient, 0xFF5A5E64U, overcast);
				View3D_AmbientSet(m_ctx.m_window, ambient);

				// The light travels from the sun toward the scene
				auto sun = SunDirection(m_sun_elevation, m_sun_azimuth);
				demo::EditLight(m_ctx.m_window, 0, [&](view3d::Light& light)
				{
					light.m_type = view3d::ELight::Directional;
					light.m_direction = view3d::Vec4{ -sun.x, -sun.y, -sun.z, 0 };
					light.m_intensity = 1.0f - 0.85f * overcast;
				});
				if (m_sky_kind == ESky::Procedural && !View3D_ObjectUpdateProceduralSky(m_sky, SkySettings()))
					throw std::runtime_error("Procedural sky update failed");

				// The procedural sky's reflections depend on the sun, so they are recaptured whenever it moves
				SetReflections(m_reflections);
			}

			// Reflect the visible sky, or turn reflections off
			void SetReflections(bool on)
			{
				// The procedural sky is drawn by a shader and has no cube map, so one is captured from the scene in 'Step'.
				// The cube map sky and no sky both reflect the hanger.
				m_reflections = on;
				view3d::CubeMap env_map = nullptr;
				if (on && m_sky_kind == ESky::Procedural)
				{
					m_recapture = true;
					env_map = m_sky_env_map.get();
				}
				else if (on)
				{
					env_map = m_env_map.get();
				}
				View3D_WindowEnvMapSet(m_ctx.m_window, env_map);
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
