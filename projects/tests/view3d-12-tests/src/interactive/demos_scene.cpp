//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of general scene features: spatial audio, the view3d UI, and hit testing.
#include <charconv>
#include <cstring>
#include <format>
#include <stdexcept>
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/utility/conversion.h"
#include "pr/audio/audio-dll.h"
#include "demo.h"
#include "controls_panel.h"
#include "view3d_ui_demo.h"

using namespace pr;
using namespace pr::gui;

namespace view3d_test
{
	namespace
	{
		// Reject a failed audio ABI operation at its call site
		void CheckAudio(audio::EStatus status, char const* operation)
		{
			// Every audio call in these demos is required to succeed
			if (status != audio::EStatus::Success)
				throw std::runtime_error(std::format("{} failed with audio status {}", operation, static_cast<int>(status)));
		}

		// Generate a one second, loopable, mono PCM16 tone as a WAV file image, so the demo needs no audio asset
		std::vector<std::byte> MakeToneWave(float frequency_hz)
		{
			// Write the RIFF/WAVE header for 48kHz mono 16-bit PCM
			constexpr auto sample_rate = std::uint32_t{48000};
			constexpr auto sample_count = sample_rate;
			auto data_size = sample_count * sizeof(std::int16_t);
			auto bytes = std::vector<std::byte>(44 + data_size);
			auto write = [&](std::size_t offset, auto value)
			{
				std::memcpy(bytes.data() + offset, &value, sizeof(value));
			};

			write(0, std::uint32_t{0x46464952});
			write(4, std::uint32_t{36 + static_cast<std::uint32_t>(data_size)});
			write(8, std::uint32_t{0x45564157});
			write(12, std::uint32_t{0x20746D66});
			write(16, std::uint32_t{16});
			write(20, std::uint16_t{1});
			write(22, std::uint16_t{1});
			write(24, sample_rate);
			write(28, sample_rate * sizeof(std::int16_t));
			write(32, std::uint16_t{sizeof(std::int16_t)});
			write(34, std::uint16_t{16});
			write(36, std::uint32_t{0x61746164});
			write(40, data_size);

			// A whole number of cycles per second makes the clip loop without a click
			auto samples = reinterpret_cast<std::int16_t*>(bytes.data() + 44);
			for (auto i = std::uint32_t{}; i != sample_count; ++i)
			{
				auto phase = constants<float>::tau * frequency_hz * i / sample_rate;
				samples[i] = static_cast<std::int16_t>(std::sin(phase) * 5000.0f);
			}
			return bytes;
		}

		// How the audio box demo advances time
		enum class EStepMode
		{
			Run,
			Single,
		};

		// A rotating box that emits a spatialised tone. The listener follows the camera.
		struct AudioBoxDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::Object m_box;
			audio::EngineHandle m_engine;
			audio::ClipHandle m_clip;
			audio::VoiceHandle m_voice;
			audio::Vector3 m_previous_listener_position;
			bool m_listener_initialized;
			bool m_occluded;
			EStepMode m_step_mode;
			int m_pending_steps;
			float m_spin_rate;
			double m_time;
			pr::gui::Label* m_time_label;

			explicit AudioBoxDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_box()
				, m_engine()
				, m_clip()
				, m_voice()
				, m_previous_listener_position()
				, m_listener_initialized(false)
				, m_occluded(false)
				, m_step_mode(EStepMode::Run)
				, m_pending_steps(0)
				, m_spin_rate(0.8f)
				, m_time(0.0)
				, m_time_label()
			{
				// The scene is a box that rotates about the origin, above a reference grid
				m_box = m_scene.Add("*Box nice_box FF00FF00 { *Data {1.23 1.23 1.23} }");
				m_scene.Add("*Grid ground FF606060 { *Data {10 10 20 20} *AxisId {+3} *o2w {*pos {0 0 -0.615}} }");

				// Create one looping spatial voice attached to the box
				CheckAudio(Audio_EngineCreate(ctx.m_audio, nullptr, &m_engine), "Audio_EngineCreate");
				auto tone = MakeToneWave(220.0f);
				CheckAudio(Audio_ClipCreateWave(m_engine, tone.data(), tone.size(), &m_clip), "Audio_ClipCreateWave");
				auto voice_desc = audio::VoiceDesc{
					.header = {sizeof(audio::VoiceDesc), audio::AUDIO_STRUCT_VERSION},
					.clip = m_clip,
					.bus = audio::EBus::Effects,
					.spatial = true,
					.loop_count = audio::AUDIO_INFINITE_LOOP,
					.priority = 100,
					.volume = 0.3f,
					.pitch = 1.0f,
				};
				CheckAudio(Audio_VoiceCreate(m_engine, &voice_desc, &m_voice), "Audio_VoiceCreate");
				CheckAudio(Audio_VoicePlay(m_engine, m_voice), "Audio_VoicePlay");

				// Controls mirror the key bindings
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Keys: O = occlusion, R = run, E = restart in single step, T/Space = step");
				ui.AddCheckBox(L"Occluded", m_occluded, [this](bool on)
				{
					m_occluded = on;
				});
				ui.AddSlider(L"Spin rate (rad/s)", 0.0f, 4.0f, m_spin_rate, [this](float v)
				{
					m_spin_rate = v;
				});
				ui.AddButton(L"Run", [this]
				{
					m_step_mode = EStepMode::Run;
				});
				ui.AddButton(L"Single step", [this]
				{
					m_step_mode = EStepMode::Single;
					++m_pending_steps;
				});
				m_time_label = ui.AddLabel(L"Time: 0.000");
			}
			~AudioBoxDemo()
			{
				// Release audio children before their owning engine
				if (m_voice != 0)
					Audio_VoiceDestroy(m_engine, m_voice);
				if (m_clip != 0)
					Audio_ClipDestroy(m_engine, m_clip);
				if (m_engine != 0)
					Audio_EngineDestroy(m_engine);
			}

			// Spin the box and update the listener and emitter
			void Step(double dt) override
			{
				// Advance the demo clock according to the step mode
				auto previous_time = m_time;
				switch (m_step_mode)
				{
					case EStepMode::Run:
					{
						m_time += dt;
						break;
					}
					case EStepMode::Single:
					{
						if (m_pending_steps > 0)
						{
							m_time += dt;
							--m_pending_steps;
						}
						break;
					}
					default:
					{
						throw std::runtime_error("Unknown step mode");
					}
				}

				// Drive the listener from the same camera pose used for the frame
				auto c2w = View3D_CameraToWorldGet(m_ctx.m_window);
				auto listener_position = audio::Vector3{c2w.w.x, c2w.w.y, c2w.w.z};
				auto listener_velocity = m_listener_initialized && dt > 0.0 && dt < 0.25
					? audio::Vector3{
						static_cast<float>((listener_position.x - m_previous_listener_position.x) / dt),
						static_cast<float>((listener_position.y - m_previous_listener_position.y) / dt),
						static_cast<float>((listener_position.z - m_previous_listener_position.z) / dt)}
					: audio::Vector3{};
				auto listener = audio::ListenerState{
					.header = {sizeof(audio::ListenerState), audio::AUDIO_STRUCT_VERSION},
					.position = listener_position,
					.forward = {-c2w.z.x, -c2w.z.y, -c2w.z.z},
					.up = {c2w.y.x, c2w.y.y, c2w.y.z},
					.velocity = listener_velocity,
				};
				CheckAudio(Audio_ListenerSet(m_engine, &listener), "Audio_ListenerSet");
				m_previous_listener_position = listener_position;
				m_listener_initialized = true;

				// Spin the box about world Z and derive the sound pose from the same transform
				auto angle = static_cast<float>(m_time) * m_spin_rate;
				auto box_o2w = m4x4::Transform(v4::ZAxis(), angle, v4::Origin());
				View3D_ObjectO2WSet(m_box, To<view3d::Mat4x4>(box_o2w), nullptr);

				auto emitter_position = box_o2w * v4{1.23f * 0.5f, 0, 0, 1};
				auto emitter_forward = box_o2w * v4::XAxis();
				auto angular_speed = dt > 0.0 ? static_cast<float>((m_time - previous_time) / dt) * m_spin_rate : 0.0f;
				auto emitter_velocity = v4{-angular_speed * emitter_position.y, angular_speed * emitter_position.x, 0, 0};
				auto emitter = audio::EmitterState{
					.header = {sizeof(audio::EmitterState), audio::AUDIO_STRUCT_VERSION},
					.position = {emitter_position.x, emitter_position.y, emitter_position.z},
					.forward = {emitter_forward.x, emitter_forward.y, emitter_forward.z},
					.up = {0, 0, 1},
					.velocity = {emitter_velocity.x, emitter_velocity.y, emitter_velocity.z},
					.min_distance = 0.5f,
					.max_distance = 30.0f,
					.cone_inner_angle = constants<float>::tau / 8.0f,
					.cone_outer_angle = constants<float>::tau / 3.0f,
					.cone_outer_gain = 0.1f,
					.doppler_scale = 1.0f,
					.obstruction = 0.0f,
					.occlusion = m_occluded ? 0.8f : 0.0f,
					.reverb_send = 0.25f,
				};
				CheckAudio(Audio_VoiceEmitterSet(m_engine, m_voice, &emitter), "Audio_VoiceEmitterSet");
				CheckAudio(Audio_EngineUpdate(m_engine), "Audio_EngineUpdate");

				// Show the demo clock, so single stepping is visible
				if (m_time != previous_time)
					m_time_label->Text(std::format(L"Time: {:.3f}", m_time).c_str());
			}

			// Step mode and occlusion keys
			void OnKey(KeyEventArgs& args) override
			{
				// Claim both the key down and key up events, but act on key up only
				switch (args.m_vk_key)
				{
					case 'E':
					case 'R':
					case 'T':
					case 'O':
					case VK_SPACE:
					{
						args.m_handled = true;
						break;
					}
					default:
					{
						return;
					}
				}
				if (args.m_down)
					return;

				// Apply the key
				switch (args.m_vk_key)
				{
					case 'E':
					{
						m_step_mode = EStepMode::Single;
						m_time = 0.0;
						break;
					}
					case 'R':
					{
						m_step_mode = EStepMode::Run;
						break;
					}
					case 'T':
					case VK_SPACE:
					{
						m_step_mode = EStepMode::Single;
						++m_pending_steps;
						break;
					}
					case 'O':
					{
						m_occluded = !m_occluded;
						break;
					}
				}
			}
		};

		// The view3d UI controls, with a slider that changes the size of a box
		struct UiGalleryDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::Object m_box;
			std::optional<View3dUiDemo> m_ui;

			explicit UiGalleryDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_box()
				, m_ui()
			{
				// The box gives the UI slider something visible to change
				m_box = m_scene.Add("*Box nice_box FF00FF00 { *Data {1.23 1.23 1.23} }");

				// Lay out the UI against the view's real size, DPI, and camera
				m_ui.emplace(ctx.m_view3d, ctx.m_window, [this](float value) { UpdateBoxDimensions(value); });
				m_ui->Update(ctx.m_view_hwnd, 0.0);
			}
			~UiGalleryDemo()
			{
				// Release the UI before the scene objects it may refer to
				m_ui.reset();
			}

			// Animate the UI
			void Step(double dt) override
			{
				// The UI clock advances at real elapsed time
				m_ui->Update(m_ctx.m_view_hwnd, dt);
			}

			// The UI receives untouched Win32 messages before they become camera input
			bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				// Messages that the UI consumes do not reach the camera
				return m_ui->ProcessWindowMessage(hwnd, message, wparam, lparam, result);
			}

			// Replace only the box model so its object identity and scene membership remain stable
			void UpdateBoxDimensions(float value)
			{
				// Format the size with enough precision to round trip
				char buffer[64] = {};
				auto [end, error] = std::to_chars(std::begin(buffer), std::end(buffer), value, std::chars_format::general, 9);
				if (error != std::errc{})
					throw std::runtime_error("Failed to format box dimensions");

				auto value_text = std::string(buffer, end);
				auto value_wide = std::wstring(value_text.begin(), value_text.end());
				auto script = L"*Box nice_box FF00FF00 { *Data {" + value_wide + L" " + value_wide + L" " + value_wide + L"} }";
				auto object_count = View3D_WindowObjectCount(m_ctx.m_window);
				View3D_ObjectUpdate(m_box, script.c_str(), view3d::EUpdateObject::Model);

				// The update must preserve the existing object's scene membership and apply the new size
				if (View3D_WindowObjectCount(m_ctx.m_window) != object_count)
					throw std::runtime_error("Updating box dimensions changed its scene membership");

				auto bounds = View3D_ObjectBBoxMS(m_box, view3d::EBBoxFlags::None);
				auto expected_radius = value * 0.5f;
				if (std::abs(bounds.radius.x - expected_radius) > 1e-5f || std::abs(bounds.radius.y - expected_radius) > 1e-5f || std::abs(bounds.radius.z - expected_radius) > 1e-5f)
					throw std::runtime_error("Updated box dimensions do not match the accepted value");
			}
		};

		// Ray casts from the mouse position into the scene
		struct HitTestDemo :IDemo
		{
			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			view3d::Object m_marker;
			view3d::ESnapMode m_snap_mode;
			pr::gui::Label* m_result;

			explicit HitTestDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_marker()
				, m_snap_mode(view3d::ESnapMode::Faces)
				, m_result()
			{
				// A few shapes to hit, and a coordinate frame that marks the hit point
				m_scene.Add("*Box box FF00FF00 { *Data {1.2 1.2 1.2} }");
				m_scene.Add("*Sphere ball FFFF8000 { *Data {0.7} *o2w {*pos {2 0 0}} }");
				m_scene.Add("*Cylinder can FF0080FF { *Data {1.5 0.5} *AxisId {+3} *o2w {*pos {0 2 0}} }");
				m_scene.Add("*Plane ground FF808080 { *Data {8 8} *AxisId {+3} *o2w {*pos {0 0 -0.6}} }");
				m_marker = m_scene.Add("*CoordFrame marker { *Scale {0.5} }");
				View3D_ObjectFlagsSet(m_marker, view3d::ELdrFlags::HitTestExclude, TRUE, nullptr);

				// Controls for the snap behaviour
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Shift + Left Click to cast a ray from the mouse position.");
				ui.AddButton(L"Snap: Faces", [this]
				{
					m_snap_mode = view3d::ESnapMode::Faces;
				});
				ui.AddButton(L"Snap: Faces, Edges, Verts", [this]
				{
					m_snap_mode = view3d::ESnapMode::All;
				});
				m_result = ui.AddLabel(L"No hit\n\n");
			}

			// Shift + Left Click casts a ray
			void OnMouseButton(MouseEventArgs& args) override
			{
				// Claim both the press and the release, so the camera does not start navigating
				if (!AllSet(args.m_key_state, EMouseKey::Shift) || !AllSet(args.m_button, EMouseKey::Left))
					return;

				args.m_handled = true;
				if (args.m_down)
					HitTest(args.point_px());
			}

			// Cast a ray through 'screen_px' and move the marker to the nearest hit
			void HitTest(Point screen_px)
			{
				// Convert the screen point to a world space ray
				auto screen = view3d::Vec2{ static_cast<float>(screen_px.x), static_cast<float>(screen_px.y) };
				view3d::Vec4 ws_pos, ws_dir;
				View3D_SSPointToWSRay(m_ctx.m_window, screen, ws_pos, ws_dir);

				view3d::HitTestRay ray = { ws_pos, ws_dir, m_snap_mode, 0.05f };
				view3d::HitTestResult hit = {};
				View3D_WindowHitTestByCtx(m_ctx.m_window, &ray, &hit, 1, {});

				// Show the result
				if (!hit.IsHit())
				{
					m_result->Text(L"No hit");
					return;
				}
				auto o2w = m4x4::Translation(To<v4>(hit.m_ws_intercept));
				View3D_ObjectO2WSet(m_marker, To<view3d::Mat4x4>(o2w), nullptr);
				m_result->Text(std::format(L"Hit: {:.3f} {:.3f} {:.3f}\nNormal: {:.3f} {:.3f} {:.3f}\nSnap type: {}",
					hit.m_ws_intercept.x, hit.m_ws_intercept.y, hit.m_ws_intercept.z,
					hit.m_ws_normal.x, hit.m_ws_normal.y, hit.m_ws_normal.z,
					static_cast<int>(hit.m_snap_type)).c_str());
			}
		};
	}

	// Demo factories
	std::unique_ptr<IDemo> CreateAudioBoxDemo(DemoContext const& ctx)
	{
		return std::make_unique<AudioBoxDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateUiGalleryDemo(DemoContext const& ctx)
	{
		return std::make_unique<UiGalleryDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateHitTestDemo(DemoContext const& ctx)
	{
		return std::make_unique<HitTestDemo>(ctx);
	}
}
