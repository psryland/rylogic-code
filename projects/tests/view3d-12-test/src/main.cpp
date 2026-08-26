#include <stdexcept>
#include <charconv>
#include <optional>
#include <windows.h>
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/common/ldraw.h"
#include "pr/win32/windows_com.h"
#include "pr/win32/win32.h"

#include "pr/view3d-12/view3d.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/utility/conversion.h"
#include "pr/view3d-ui/view3d-ui.h"
#include "pr/audio/audio-dll.h"

using namespace pr;
using namespace pr::gui;
using namespace pr::rdr12;

std::filesystem::path const RylogicRoot = "E:\\Rylogic\\Code";
std::filesystem::path const RylogicAssets = "E:\\Rylogic\\Assets";

enum class EStepMode
{
	Single,
	Run,
};

namespace
{
	using namespace pr::view3d::ui;
	using UIColour = pr::view3d::ui::Colour;

	inline constexpr ControlId UI_Root = 1;
	inline constexpr ControlId UI_Panel = 2;
	inline constexpr ControlId UI_Label = 3;
	inline constexpr ControlId UI_Dimension = 4;
	inline constexpr ControlId UI_Update = 5;
	inline constexpr ResourceId UI_Font = 100;
	inline constexpr StyleId UI_RootStyle = 200;
	inline constexpr StyleId UI_PanelStyle = 201;
	inline constexpr StyleId UI_TextStyle = 202;
	inline constexpr StyleId UI_TextBoxStyle = 203;
	inline constexpr StyleId UI_ButtonStyle = 204;
	inline constexpr TemplateId UI_TextBoxTemplate = 300;
	inline constexpr TemplateId UI_ButtonTemplate = 301;

	// Populate a current-version structure header for one public View3DUI record.
	template <typename T>
	StructHeader UIHeader()
	{
		return StructHeader{sizeof(T), VIEW3D_UI_STRUCT_VERSION};
	}

	// Build one fixed-layout control description before its strings are packed into a transaction.
	ControlDesc UIControl(ControlId id, ControlId parent_id, EControlType type, ELayoutMode layout_mode, LayoutParams layout, StyleId style_id, TemplateId template_id = 0)
	{
		auto desc = ControlDesc{};
		desc.header = UIHeader<ControlDesc>();
		desc.id = id;
		desc.parent_id = parent_id;
		desc.type = type;
		desc.root_policy = ERootPolicy::Screen;
		desc.layout_mode = layout_mode;
		desc.template_id = template_id;
		desc.style_id = style_id;
		desc.enabled = 1;
		desc.visible = 1;
		desc.focusable = type == EControlType::TextBox || type == EControlType::Button;
		desc.validation_state = EValidationState::NotApplicable;
		desc.layout = layout;
		desc.max_text_length = 32;
		desc.font_resource_id = type == EControlType::Root || type == EControlType::Panel ? 0 : UI_Font;
		return desc;
	}

	// Create a layout value with explicit size and alignment while leaving optional spacing at zero.
	LayoutParams UILayout(float width, float height, EHAlign h_align = EHAlign::Left, EVAlign v_align = EVAlign::Top)
	{
		auto layout = LayoutParams{};
		layout.width = width;
		layout.height = height;
		layout.h_align = h_align;
		layout.v_align = v_align;
		return layout;
	}

	// Create one style with an identical base visual in every state channel.
	StyleDesc UIStyle(StyleId id, UIColour fill, UIColour border, float border_thickness, float corner_radius, float opacity = 1.0f)
	{
		auto style = StyleDesc{};
		style.header = UIHeader<StyleDesc>();
		style.id = id;
		for (auto i = std::size_t{}; i != static_cast<std::size_t>(EStateChannel::Count); ++i)
		{
			style.visuals[i] = StyleVisual{fill, border, border_thickness, corner_radius, opacity};
			style.transitions[i] = TransitionDesc{100.0f, EEasing::EaseInOut};
		}
		return style;
	}

	// Add one application-owned font resource to a public transaction builder.
	void AddUIFont(TransactionBuilder& builder, ResourceId id, std::string_view family, float size, UIColour colour)
	{
		auto resource = ResourceDesc{};
		resource.header = UIHeader<ResourceDesc>();
		resource.id = id;
		resource.kind = EResourceKind::Font;
		resource.colour = colour;
		resource.font_size = size;
		builder.AddNamedResource(resource, family);
	}

	// Parsed numeric state owned by the demonstration application.
	struct DimensionValue
	{
		EValidationState m_validation;
		float m_value;
	};

	// Parse the deliberately invariant positive-decimal grammar used by the demonstration.
	DimensionValue ParseDimension(std::string_view text)
	{
		if (text.empty() || text == "." || text == "+" || text == "-")
			return {EValidationState::Pending, 0.0f};

		auto index = std::size_t{};
		auto negative = false;
		if (text[index] == '+' || text[index] == '-')
		{
			negative = text[index] == '-';
			++index;
		}

		auto value = 0.0;
		auto digit_count = 0;
		while (index != text.size() && text[index] >= '0' && text[index] <= '9')
		{
			value = value * 10.0 + (text[index] - '0');
			++digit_count;
			++index;
		}

		if (index != text.size() && text[index] == '.')
		{
			++index;
			if (index == text.size())
				return {EValidationState::Pending, 0.0f};

			auto scale = 0.1;
			while (index != text.size() && text[index] >= '0' && text[index] <= '9')
			{
				value += (text[index] - '0') * scale;
				scale *= 0.1;
				++digit_count;
				++index;
			}
		}

		if (index != text.size() || digit_count == 0)
			return {EValidationState::Invalid, 0.0f};

		value = negative ? -value : value;
		return std::isfinite(value) && value > 0.0 && value <= std::numeric_limits<float>::max()
			? DimensionValue{EValidationState::Valid, static_cast<float>(value)}
			: DimensionValue{EValidationState::Invalid, 0.0f};
	}

	// Create a UI context while balancing the temporary View3D device lease on every path.
	view3d::ui::UiContext CreateUIContext(view3d::ui::Runtime const& runtime, view3d::DllHandle view3d_context, view3d::Window window)
	{
		auto device = static_cast<IUnknown*>(View3D_DeviceLeaseAcquire(view3d_context));
		if (device == nullptr)
			throw std::runtime_error("Failed to acquire the View3D device for View3DUI");

		try
		{
			auto ui = view3d::ui::UiContext(runtime, device, window);
			device->Release();
			return ui;
		}
		catch (...)
		{
			device->Release();
			throw;
		}
	}
}

// Application window
struct Main :Form
{
	enum { IDR_MAINFRAME = 100 };
	enum { ID_FILE, ID_FILE_EXIT };
	enum { IDC_PROGRESS = 100, IDC_NM_PROGRESS, IDC_MODELESS, IDC_CONTEXTMENU, IDC_POSTEST, IDC_ABOUT, IDC_MSGBOX, IDC_SCINT, IDC_TAB, IDC_TAB1, IDC_TAB2, IDC_SPLITL, IDC_SPLITR };

	bool m_ui_ready;
	view3d::DllHandle m_view3d;
	view3d::Window m_win3d;
	std::optional<view3d::ui::Runtime> m_ui_runtime;
	std::optional<view3d::ui::UiContext> m_ui;
	view3d::CubeMap m_envmap;
	view3d::Object m_obj0;
	view3d::Object m_obj1;
	audio::DllHandle m_audio;
	audio::EngineHandle m_audio_engine;
	audio::ClipHandle m_audio_clip;
	audio::VoiceHandle m_box_voice;
	audio::Vector3 m_previous_listener_position;
	bool m_listener_initialized;
	bool m_audio_occluded;
	GUID m_file_ctx;
	EStepMode m_step_mode;
	int m_pending_steps;
	double m_time = 0.0;
	double m_ui_time_ms = 0.0;
	std::uint64_t m_ui_revision = 0;
	std::string m_dimension_text = "1.23";
	DimensionValue m_dimension = ParseDimension(m_dimension_text);
	float m_box_dimension = m_dimension.m_value;
	std::vector<view3d::ui::Event> m_ui_events;
	std::vector<std::byte> m_ui_event_payload;
	
	// Error handler
	static void __stdcall ReportError(void*, char const* msg, char const* filepath, int line, int64_t)
	{
		std::cout << filepath << "(" << line << "): " << msg << std::endl;
		throw std::runtime_error(std::string(msg));
	}

	// Surface synchronous audio failures through the same visible test-host path.
	static void __stdcall ReportAudioError(void*, char const* msg, char const* filepath, int line)
	{
		std::cout << filepath << "(" << line << "): " << msg << std::endl;
	}

	// Reject a failed audio ABI operation at its call site.
	static void CheckAudio(audio::EStatus status, char const* operation)
	{
		if (status != audio::EStatus::Success)
			throw std::runtime_error(std::format("{} failed with audio status {}", operation, static_cast<int>(status)));
	}

	// Generate a small loopable mono PCM16 tone without adding a demo asset dependency.
	static std::vector<std::byte> MakeToneWave(float frequency_hz)
	{
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

		auto samples = reinterpret_cast<std::int16_t*>(bytes.data() + 44);
		for (auto i = std::uint32_t{}; i != sample_count; ++i)
		{
			auto phase = constants<float>::tau * frequency_hz * i / sample_rate;
			samples[i] = static_cast<std::int16_t>(std::sin(phase) * 5000.0f);
		}
		return bytes;
	}
	static view3d::WindowOptions WndOptions(Main& main)
	{
		return view3d::WindowOptions()
			.error_cb({ &main, ReportError })
			.back_colour(0xFF908080)
			.alt_enter()
			.multisamp(8)
			.name("TestWnd")
			//.xr_support()
			;
	}

	Main(HINSTANCE)
		: Form(Params<>()
			.name("main")
			.title(L"View3d 12 Test")
			.xy(1400,100).wh(1024, 768, true)
			.main_wnd(true)
			.dbl_buffer(true)
			.wndclass(RegisterWndClass<Main>()))
		, m_ui_ready(false)
		, m_view3d(View3D_Initialise({ this, ReportError }))
		, m_win3d(View3D_WindowCreate(CreateHandle(), WndOptions(*this)))
		, m_ui_runtime(std::in_place)
		, m_ui(CreateUIContext(*m_ui_runtime, m_view3d, m_win3d))
		, m_envmap(View3D_CubeMapCreateFromUri((RylogicAssets / "textures/cubemaps/hanger/hanger-??.jpg").string().c_str(), {}))
		, m_obj0()
		, m_obj1()
		, m_audio(Audio_Initialise({this, ReportAudioError}))
		, m_audio_engine()
		, m_audio_clip()
		, m_box_voice()
		, m_previous_listener_position()
		, m_listener_initialized(false)
		, m_audio_occluded(false)
		, m_file_ctx()
		, m_step_mode(EStepMode::Run)
		, m_pending_steps()
	{
		m_ui_ready = true;

		if (m_audio == nullptr)
			throw std::runtime_error("Audio initialization failed");

		// Create one looping spatial voice owned by the rotating box demonstration.
		CheckAudio(Audio_EngineCreate(m_audio, nullptr, &m_audio_engine), "Audio_EngineCreate");
		auto tone = MakeToneWave(220.0f);
		CheckAudio(Audio_ClipCreateWave(m_audio_engine, tone.data(), tone.size(), &m_audio_clip), "Audio_ClipCreateWave");
		auto voice_desc = audio::VoiceDesc{
			.header = {sizeof(audio::VoiceDesc), audio::AUDIO_STRUCT_VERSION},
			.clip = m_audio_clip,
			.bus = audio::EBus::Effects,
			.spatial = true,
			.loop_count = audio::AUDIO_INFINITE_LOOP,
			.priority = 100,
			.volume = 0.3f,
			.pitch = 1.0f,
		};
		CheckAudio(Audio_VoiceCreate(m_audio_engine, &voice_desc, &m_box_voice), "Audio_VoiceCreate");
		CheckAudio(Audio_VoicePlay(m_audio_engine, m_box_voice), "Audio_VoicePlay");

		// Set up the scene
		View3D_CameraPositionSet(m_win3d, {5, -5, 4, 1}, {0, 0, 0, 1}, {0, 0, 1, 0});
	
		// Cast shadows
		auto light = View3D_LightPropertiesGet(m_win3d);
		light.m_type = view3d::ELight::Directional;
		light.m_direction = To<view3d::Vec4>(v4::Normal(-1, -1, -1, 0));
		light.m_cast_shadow = 0.0f;// 10.0f;
		light.m_cam_relative = false;
		View3D_LightPropertiesSet(m_win3d, light);

		// Create 'm_obj0', 'm_obj1'
		{
			std::default_random_engine rng;
			std::uniform_real_distribution dist(-10.0f, 10.0f);

			m_obj0 = View3D_ObjectCreateLdrA(
				//"*Triangle nice_tri FF00FF00 { *Data { -1 -1 0  +1 -1 0  0 +1 0} }"
				"*Box nice_box FF00FF00 { *Data {1.23 1.23 1.23} }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\Pendulum\\Pendulum.fbx\" } }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\AnimCharacter\\AnimatedCharacter.fbx\" } }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\Pendulum\\Pendulum.fbx\" } *Animation{*Style{Repeat}} }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\AnimCharacter\\AnimatedCharacter.fbx\" } *Animation{*Style{PingPong}} }"
				, false, nullptr, nullptr);

			m_obj1 = View3D_ObjectCreateLdrA(
				//"*Sphere sever FF0080FF { *Data {0.4} }"
				//"*Box Origin FF00FF00 { *Data {1 1 1} }"
				"*CoordFrame origin { *Scale {1} }"
				, false, nullptr, nullptr);

			//auto builder = ldraw::Builder();
			//auto& pts = builder.Point("pts", 0xFF00FF00).size({ 40, 40 }).style(ldraw::EPointStyle::Star);
			//for (int i = 0; i != 100; ++i)
			//	pts.pt(v3::Random(rng, v3::Zero(), 0.5f).w1());
			//m_obj0 = View3D_ObjectCreateLdrA(builder.ToString(true).c_str(), false, nullptr, nullptr);

			//auto builder = ldr::Builder();
			//auto& points = builder.Point("points", 0xFF00FF00);
			//points.size(10.0f);
			//for (int i = 0; i != 10000; ++i) points.pt({ dist(rng), dist(rng), 0, 1 });
			//auto& spline = builder.Spline("spline");
			//spline.spline(v4{ 0, 0, 0, 1 }, v4{ -1, 1, 0, 1 }, v4{ -1, 2, 0, 1 }, v4{ 0, 1.5f, 0, 1 }, 0xFF00FF00);
			//spline.spline(v4{ 0, 0, 0, 1 }, v4{ +1, 1, 0, 1 }, v4{ +1, 2, 0, 1 }, v4{ 0, 1.5f, 0, 1 }, 0xFFFF0000);
			//spline.width(4);
			//spline.pos(v4{ 0, 10, 0, 1 });

			// Load script
			//m_file_ctx = View3D_LoadScriptFromFile("E:/Dump/Splines.Scene.bdr", nullptr, nullptr, {});

			View3D_ObjectFlagsSet(m_obj1, view3d::ELdrFlags::HitTestExclude, true, nullptr);
		}

		// Add objects to the scene
		{
			View3D_WindowAddObject(m_win3d, m_obj0);
			View3D_WindowAddObject(m_win3d, m_obj1);
			//View3D_WindowAddObjectsById(m_win3d, { &m_file_ctx, [](void* ctx, GUID const& id) { return *type_ptr<GUID>(ctx) == id; } });
			//View3D_DemoSceneCreateText(m_win3d);
			//View3D_DemoSceneCreateBinary(m_win3d);
		}

		// EnvMap
		//View3D_WindowEnvMapSet(m_win3d, m_envmap);
		View3D_WindowEnumObjects(m_win3d, { nullptr, [](void*, view3d::Object obj)
			{
				View3D_ObjectReflectivitySet(obj, 0.2f, "");
				return true;
			}});

		// Streaming
		View3D_StreamingEnable(true, 1976);

		// Create the retained View3DUI demonstration tree and its application-authored look.
		ApplyInitialUI();
		UpdateUI();
	}
	~Main()
	{
		// Detach and release View3DUI before destroying its View3D host window and device.
		m_ui_ready = false;
		m_ui.reset();
		m_ui_runtime.reset();

		// Release audio children before their owning engine and DLL context.
		if (m_box_voice != 0)
			Audio_VoiceDestroy(m_audio_engine, m_box_voice);
		if (m_audio_clip != 0)
			Audio_ClipDestroy(m_audio_engine, m_audio_clip);
		if (m_audio_engine != 0)
			Audio_EngineDestroy(m_audio_engine);
		if (m_audio != nullptr)
			Audio_Shutdown(m_audio);

		View3D_CubeMapRelease(m_envmap);
		View3D_WindowDestroy(m_win3d);
		View3D_ObjectDelete(m_obj0);
		View3D_ObjectDelete(m_obj1);
		View3D_Shutdown(m_view3d);
	}
	void Step(double dt)
	{
		m_ui_time_ms += dt * 1000.0;

		static double time_scale = 1.0;
		dt *= time_scale;
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

		auto c2w = View3D_CameraToWorldGet(m_win3d);

		// Drive the rendered listener from the same camera pose used for the frame.
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
		CheckAudio(Audio_ListenerSet(m_audio_engine, &listener), "Audio_ListenerSet");
		m_previous_listener_position = listener_position;
		m_listener_initialized = true;

		// Spin the box about world Z at the origin and derive the sound pose from the same transform.
		constexpr auto spin_rate = 0.8f;
		auto angle = static_cast<float>(m_time) * spin_rate;
		auto box_o2w = m4x4::Transform(v4::ZAxis(), angle, v4::Origin());
		View3D_ObjectO2WSet(m_obj0, To<view3d::Mat4x4>(box_o2w), nullptr);

		auto emitter_position = box_o2w * v4{m_box_dimension * 0.5f, 0, 0, 1};
		auto emitter_forward = box_o2w * v4::XAxis();
		auto angular_speed = dt > 0.0 ? static_cast<float>((m_time - previous_time) / dt) * spin_rate : 0.0f;
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
			.occlusion = m_audio_occluded ? 0.8f : 0.0f,
			.reverb_send = 0.25f,
		};
		CheckAudio(Audio_VoiceEmitterSet(m_audio_engine, m_box_voice, &emitter), "Audio_VoiceEmitterSet");
		CheckAudio(Audio_EngineUpdate(m_audio_engine), "Audio_EngineUpdate");

		SetWindowTextA(*this, pr::FmtS("View3d 12 Test - Cam: %3.3f %3.3f %3.3f  Dir: %3.3f %3.3f %3.3f", c2w.w.x, c2w.w.y, c2w.w.z, -c2w.z.x, -c2w.z.y, -c2w.z.z));
		DrainUIEvents();
		UpdateUI();
		View3D_WindowRender(m_win3d);
	}
	bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
	{
		// UI receives untouched Win32 messages before Form translates them into camera input.
		if (m_ui_ready && m_ui)
		{
			auto invalidate = std::int32_t{};
			auto ui_result = LRESULT{};
			if (m_ui->ProcessWindowMessage(hwnd, message, wparam, lparam, ui_result, invalidate) != 0)
			{
				result = ui_result;
				return true;
			}
		}

		return Form::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
	}
	void OnWindowPosChange(WindowPosEventArgs const& args) override
	{
		Form::OnWindowPosChange(args);
		if (!args.m_before && args.IsResize() && !IsIconic(*this))
		{
			auto rect = ClientRect(false);
			auto w = rect.width();
			auto h = rect.height();
			View3D_WindowBackBufferSizeSet(m_win3d, { w, h }, false);
			View3D_WindowViewportSet(m_win3d, view3d::Viewport{
				.m_x = 0,
				.m_y = 0,
				.m_width = 1.f * w,
				.m_height = 1.f * h,
				.m_min_depth = 0,
				.m_max_depth = 1,
				.m_screen_w = w,
				.m_screen_h = h,
				});
		}
	}
	void OnMouseButton(MouseEventArgs& args) override
	{
		Form::OnMouseButton(args);
		if (AllSet(args.m_key_state, EMouseKey::Shift) && AllSet(args.m_button, EMouseKey::Left))
		{
			HitTest(args.point_px());
			args.m_handled = true;
		}
		if (!args.m_handled)
		{
			view3d::Vec2 pt = {s_cast<float>(args.m_point.x), s_cast<float>(args.m_point.y)};
			auto nav_op =
				AllSet(args.m_button, EMouseKey::Left) ? view3d::ENavOp::Rotate :
				AllSet(args.m_button, EMouseKey::Right) ? view3d::ENavOp::Translate :
				view3d::ENavOp::None;

			View3D_MouseNavigate(m_win3d, pt, nav_op, TRUE);
		}
	}
	void OnMouseMove(MouseEventArgs& args) override
	{
		Form::OnMouseMove(args);
		if (!args.m_handled)
		{
			view3d::Vec2 pt = {s_cast<float>(args.m_point.x), s_cast<float>(args.m_point.y)};
			auto nav_op =
				AllSet(args.m_button, EMouseKey::Left) ? view3d::ENavOp::Rotate :
				AllSet(args.m_button, EMouseKey::Right) ? view3d::ENavOp::Translate :
				view3d::ENavOp::None;

			View3D_MouseNavigate(m_win3d, pt, nav_op, FALSE);
		}
	}
	void OnMouseWheel(MouseWheelArgs& args) override
	{
		Form::OnMouseWheel(args);
		if (!args.m_handled)
		{
			view3d::Vec2 pt = {s_cast<float>(args.m_point.x), s_cast<float>(args.m_point.y)};
			View3D_MouseNavigateZ(m_win3d, pt, args.m_delta, TRUE);
		}
	}
	void OnKey(KeyEventArgs& args) override
	{
		Form::OnKey(args);
		if (args.m_down)
			return;

		switch (args.m_vk_key)
		{
			case VK_F7:
			{
				View3D_ReloadScriptSources();
				args.m_handled = true;
				break;
			}
			case 'E':
			{
				m_step_mode = EStepMode::Single;
				m_time = 0.0;
				break;
			}
			case 'R':
			{
				m_step_mode = EStepMode::Run;
				args.m_handled = true;
				break;
			}
			case 'T':
			{
				m_step_mode = EStepMode::Single;
				++m_pending_steps;
				args.m_handled = true;
				break;
			}
			case 'O':
			{
				m_audio_occluded = !m_audio_occluded;
				args.m_handled = true;
				break;
			}
			case VK_SPACE:
			{
				if (m_step_mode == EStepMode::Single)
					++m_pending_steps;
				break;
			}
		}
	}
	void HitTest(gui::Point screen_px)
	{
		auto screen = view3d::Vec2{ static_cast<float>(screen_px.x), static_cast<float>(screen_px.y) };
		auto c2w = View3D_CameraToWorldGet(m_win3d);
		view3d::Vec4 ws_pos, ws_dir; View3D_SSPointToWSRay(m_win3d, screen, ws_pos, ws_dir);
		view3d::HitTestRay rays[1] = {
			{ws_pos, ws_dir, view3d::ESnapMode::Faces, 0.001f},
		//	{c2w.w, {-c2w.z.x, -c2w.z.y, -c2w.z.z, 0}},
		};
		view3d::HitTestResult results[2] = {};
		View3D_WindowHitTestByCtx(m_win3d, &rays[0], &results[0], _countof(rays), {});

		for (auto const& hit : results)
		{
			if (!hit.IsHit()) continue;
			auto o2w = m4x4::Translation(To<v4>(hit.m_ws_intercept));
			View3D_ObjectO2WSet(m_obj1, To<view3d::Mat4x4>(o2w), nullptr);
		}
	}

	// Apply the complete initial UI tree, resources, templates, and styles in one revision.
	void ApplyInitialUI()
	{
		using namespace view3d::ui;

		TransactionBuilder builder;
		AddUIFont(builder, UI_Font, "Segoe UI", 18.0f, UIColour{1, 1, 1, 1});

		auto root_style = UIStyle(UI_RootStyle, UIColour{0, 0, 0, 0}, UIColour{0, 0, 0, 0}, 0, 0, 0);
		auto panel_style = UIStyle(UI_PanelStyle, UIColour{0.16f, 0.13f, 0.13f, 0.82f}, UIColour{0.3f, 0.3f, 0.3f, 0.8f}, 1.0f, 0.0f);
		auto text_style = UIStyle(UI_TextStyle, UIColour{0, 0, 0, 0}, UIColour{1, 1, 1, 1}, 0, 0);
		auto textbox_style = UIStyle(UI_TextBoxStyle, UIColour{0.12f, 0.10f, 0.10f, 1}, UIColour{0.5f, 0.5f, 0.5f, 1}, 1.0f, 3.0f);
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].border_colour = UIColour{0.7f, 0.7f, 0.7f, 1};
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_colour = UIColour{0.15f, 0.5f, 1.0f, 1};
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_thickness = 2.0f;
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Invalid)].border_colour = UIColour{1.0f, 0.2f, 0.2f, 1};
		textbox_style.visuals[static_cast<std::size_t>(EStateChannel::Invalid)].border_thickness = 2.0f;
		auto button_style = UIStyle(UI_ButtonStyle, UIColour{0.46f, 0.46f, 0.46f, 1}, UIColour{0.55f, 0.55f, 0.55f, 1}, 1.0f, 10.0f);
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Hover)].fill = UIColour{0.58f, 0.58f, 0.58f, 1};
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Pressed)].fill = UIColour{0.32f, 0.32f, 0.32f, 1};
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_colour = UIColour{0.15f, 0.5f, 1.0f, 1};
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].border_thickness = 2.0f;
		button_style.visuals[static_cast<std::size_t>(EStateChannel::Disabled)].opacity = 0.45f;
		builder.AddStyle(root_style);
		builder.AddStyle(panel_style);
		builder.AddStyle(text_style);
		builder.AddStyle(textbox_style);
		builder.AddStyle(button_style);

		auto textbox_template = TemplateDesc{};
		textbox_template.header = UIHeader<TemplateDesc>();
		textbox_template.id = UI_TextBoxTemplate;
		textbox_template.applies_to = EControlType::TextBox;
		builder.AddTemplatePart(textbox_template, "PART_Text", EVisualPrimitive::TextPresenter);
		builder.AddTemplatePart(textbox_template, "PART_Selection", EVisualPrimitive::SolidBox);
		builder.AddTemplatePart(textbox_template, "PART_Caret", EVisualPrimitive::SolidBox);
		builder.AddTemplatePart(textbox_template, "PART_ValidationOutline", EVisualPrimitive::Border);
		builder.AddTemplatePart(textbox_template, "PART_FocusOutline", EVisualPrimitive::Border);
		builder.AddTemplate(textbox_template);

		auto button_template = TemplateDesc{};
		button_template.header = UIHeader<TemplateDesc>();
		button_template.id = UI_ButtonTemplate;
		button_template.applies_to = EControlType::Button;
		builder.AddTemplatePart(button_template, "PART_ContentPresenter", EVisualPrimitive::ContentPresenter);
		builder.AddTemplatePart(button_template, "PART_FocusOutline", EVisualPrimitive::Border);
		builder.AddTemplate(button_template);

		auto root_layout = UILayout(0, 0, EHAlign::Stretch, EVAlign::Stretch);
		auto panel_layout = UILayout(320, 180, EHAlign::Right, EVAlign::Top);
		panel_layout.margin_top = 28;
		panel_layout.margin_right = 28;
		panel_layout.padding_left = 20;
		panel_layout.padding_top = 20;
		panel_layout.padding_right = 20;
		panel_layout.padding_bottom = 20;
		panel_layout.stack_spacing = 10;
		auto label_layout = UILayout(280, 26, EHAlign::Stretch);
		auto textbox_layout = UILayout(280, 36, EHAlign::Stretch);
		auto button_layout = UILayout(280, 48, EHAlign::Stretch);

		builder.Upsert(UIControl(UI_Root, 0, EControlType::Root, ELayoutMode::Overlay, root_layout, UI_RootStyle), {}, "Dimension editor");
		builder.Upsert(UIControl(UI_Panel, UI_Root, EControlType::Panel, ELayoutMode::StackVertical, panel_layout, UI_PanelStyle), {}, "Box dimensions panel");
		builder.Upsert(UIControl(UI_Label, UI_Panel, EControlType::Text, ELayoutMode::Overlay, label_layout, UI_TextStyle), "Enter the box dimensions:", "Box dimension label");
		auto textbox = UIControl(UI_Dimension, UI_Panel, EControlType::TextBox, ELayoutMode::Overlay, textbox_layout, UI_TextBoxStyle, UI_TextBoxTemplate);
		textbox.validation_state = m_dimension.m_validation;
		builder.Upsert(textbox, m_dimension_text, "Box dimensions", "A positive decimal value applied uniformly to the box");
		auto button = UIControl(UI_Update, UI_Panel, EControlType::Button, ELayoutMode::Overlay, button_layout, UI_ButtonStyle, UI_ButtonTemplate);
		button.enabled = m_dimension.m_validation == EValidationState::Valid;
		builder.Upsert(button, "Update", "Update box dimensions");
		builder.Reorder(UI_Root, {UI_Panel});
		builder.Reorder(UI_Panel, {UI_Label, UI_Dimension, UI_Update});

		m_ui->TransactionApply(builder.Build(m_ui_revision, m_ui_revision + 1));
		++m_ui_revision;
	}

	// Reconcile application-owned text, validation, and command enablement in one revision.
	void ApplyUIState()
	{
		using namespace view3d::ui;

		TransactionBuilder builder;
		auto textbox = UIControl(UI_Dimension, UI_Panel, EControlType::TextBox, ELayoutMode::Overlay, UILayout(280, 36, EHAlign::Stretch), UI_TextBoxStyle, UI_TextBoxTemplate);
		textbox.validation_state = m_dimension.m_validation;
		builder.Upsert(textbox, m_dimension_text, "Box dimensions", "A positive decimal value applied uniformly to the box");
		auto button = UIControl(UI_Update, UI_Panel, EControlType::Button, ELayoutMode::Overlay, UILayout(280, 48, EHAlign::Stretch), UI_ButtonStyle, UI_ButtonTemplate);
		button.enabled = m_dimension.m_validation == EValidationState::Valid;
		builder.Upsert(button, "Update", "Update box dimensions");

		m_ui->TransactionApply(builder.Build(m_ui_revision, m_ui_revision + 1));
		++m_ui_revision;
	}

	// Drain typed UI events and dispatch centrally by stable control id.
	void DrainUIEvents()
	{
		if (!m_ui)
			return;

		auto pending = m_ui->EventsPendingSizes();
		if (pending.m_count == 0)
			return;

		m_ui_events.resize(pending.m_count);
		m_ui_event_payload.resize(pending.m_payload_bytes);
		m_ui->EventsCopy(m_ui_events, m_ui_event_payload);

		for (auto const& event : m_ui_events)
		{
			switch (event.kind)
			{
				case view3d::ui::EEventKind::TextChangeProposed:
				{
					if (event.control_id != UI_Dimension)
						break;

					auto const* text = reinterpret_cast<char const*>(m_ui_event_payload.data() + event.payload_offset);
					m_dimension_text.assign(text, event.payload_length);
					m_dimension = ParseDimension(m_dimension_text);
					ApplyUIState();
					break;
				}
				case view3d::ui::EEventKind::CommandInvoked:
				{
					if (event.control_id == UI_Update && m_dimension.m_validation == view3d::ui::EValidationState::Valid)
						UpdateBoxDimensions(m_dimension.m_value);
					break;
				}
				case view3d::ui::EEventKind::FocusChanged:
				case view3d::ui::EEventKind::PointerCaptureChanged:
				case view3d::ui::EEventKind::QueueOverflow:
				case view3d::ui::EEventKind::Diagnostic:
				{
					break;
				}
				case view3d::ui::EEventKind::Count:
				default:
				{
					throw std::runtime_error("Unknown View3DUI event kind");
				}
			}
		}
	}

	// Recompute deterministic layout, semantics, transitions, and draw packets at explicit UI time.
	void UpdateUI()
	{
		if (!m_ui)
			return;

		auto client = ClientRect(false);
		auto target = View3D_WindowBackBufferSizeGet(m_win3d);
		auto viewport = View3D_WindowViewportGet(m_win3d);
		auto state = view3d::ui::ViewportState{
			UIHeader<view3d::ui::ViewportState>(),
			s_cast<std::uint32_t>(std::max(0, client.width())),
			s_cast<std::uint32_t>(std::max(0, client.height())),
			s_cast<std::uint32_t>(std::max(0L, target.cx)),
			s_cast<std::uint32_t>(std::max(0L, target.cy)),
			viewport.m_x,
			viewport.m_y,
			viewport.m_width,
			viewport.m_height,
			static_cast<float>(GetDpiForWindow(*this)),
			m_ui_time_ms,
		};
		m_ui->Update(state);
	}

	// Replace only the existing box model so its object identity and per-frame transform remain stable.
	void UpdateBoxDimensions(float value)
	{
		char buffer[64] = {};
		auto [end, error] = std::to_chars(std::begin(buffer), std::end(buffer), value, std::chars_format::general, 9);
		if (error != std::errc{})
			throw std::runtime_error("Failed to format box dimensions");

		auto value_text = std::string(buffer, end);
		auto value_wide = std::wstring(value_text.begin(), value_text.end());
		auto script = L"*Box nice_box FF00FF00 { *Data {" + value_wide + L" " + value_wide + L" " + value_wide + L"} }";
		auto object_count = View3D_WindowObjectCount(m_win3d);
		View3D_ObjectUpdate(m_obj0, script.c_str(), view3d::EUpdateObject::Model);

		// The update must preserve the existing object's scene membership.
		if (View3D_WindowObjectCount(m_win3d) != object_count)
			throw std::runtime_error("Updating box dimensions changed its scene membership");

		auto bounds = View3D_ObjectBBoxMS(m_obj0, view3d::EBBoxFlags::None);
		auto expected_radius = value * 0.5f;
		if (std::abs(bounds.radius.x - expected_radius) > 1e-5f || std::abs(bounds.radius.y - expected_radius) > 1e-5f || std::abs(bounds.radius.z - expected_radius) > 1e-5f)
			throw std::runtime_error("Updated box dimensions do not match the accepted value");

		m_box_dimension = value;
	}
};

// Entry point
int __stdcall WinMain(HINSTANCE hinstance, HINSTANCE, LPTSTR, int)
{
	try
	{
		pr::InitCom com;
		pr::win32::LoadDll<struct Audio>("audio.dll");
		pr::win32::LoadDll<struct View3d>("view3d-12.dll");
		(void)view3d::ui::Dll::Get();

		// Register the owning message loop so window destruction can drain continuously posted renderer messages and observe WM_QUIT.
		WinGuiMsgLoop loop;
		Main main(hinstance);
		main.cp().msg_loop(&loop);
		main.Show();

		loop.AddMessageFilter(main);
		loop.AddLoop(100.0, true, [&main](auto dt) { main.Step(dt); });
		return loop.Run();
	}
	catch (std::exception const& ex)
	{
		OutputDebugStringA("Died: ");
		OutputDebugStringA(ex.what());
		OutputDebugStringA("\n");
		return -1;
	}
}
