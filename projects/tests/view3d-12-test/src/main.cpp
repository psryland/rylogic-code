#include <stdexcept>
#include <windows.h>
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/win32/windows_com.h"
#include "pr/win32/win32.h"

#include "pr/view3d-12/view3d.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"
#include "pr/view3d-12/utility/conversion.h"

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

// Application window
struct Main :Form
{
	enum { IDR_MAINFRAME = 100 };
	enum { ID_FILE, ID_FILE_EXIT };
	enum { IDC_PROGRESS = 100, IDC_NM_PROGRESS, IDC_MODELESS, IDC_CONTEXTMENU, IDC_POSTEST, IDC_ABOUT, IDC_MSGBOX, IDC_SCINT, IDC_TAB, IDC_TAB1, IDC_TAB2, IDC_SPLITL, IDC_SPLITR };

	view3d::DllHandle m_view3d;
	view3d::Window m_win3d;
	view3d::CubeMap m_envmap;
	view3d::Object m_obj0;
	view3d::Object m_obj1;
	GUID m_file_ctx;
	EStepMode m_step_mode;
	int m_pending_steps;
	double m_time = 0.0;
	
	// Error handler
	static void __stdcall ReportError(void*, char const* msg, char const* filepath, int line, int64_t)
	{
		std::cout << filepath << "(" << line << "): " << msg << std::endl;
		throw std::runtime_error(std::string(msg));
	}
	static view3d::WindowOptions WndOptions(Main& main)
	{
		return view3d::WindowOptions()
			.error_cb({ &main, ReportError })
			.back_colour(0xFF908080)
			.alt_enter()
			.multisamp(8)
			.name("TestWnd")
			.xr_support();
	}

	Main(HINSTANCE)
		: Form(Params<>()
			.name("main")
			.title(L"View3d 12 Test")
			.xy(1400,100).wh(1024, 768, true)
			.main_wnd(true)
			.dbl_buffer(true)
			.wndclass(RegisterWndClass<Main>()))
		, m_view3d(View3D_Initialise({ this, ReportError }))
		, m_win3d(View3D_WindowCreate(CreateHandle(), WndOptions(*this)))
		, m_envmap(View3D_CubeMapCreateFromUri((RylogicAssets / "textures/cubemaps/hanger/hanger-??.jpg").string().c_str(), {}))
		, m_obj0()
		, m_obj1()
		, m_file_ctx()
		, m_step_mode(EStepMode::Single)
		, m_pending_steps()
	{
		// Set up the scene
		//View3D_CameraPositionSet(m_win3d, {0, +35, +40, 1}, {0, 0, 0, 1}, {0, 1, 0, 0});
		View3D_CameraPositionSet(m_win3d, {2, 0, 0, 1}, {0, 0, 1, 1}, {0, 0, 1, 0});
		//View3D_CameraPositionSet(m_win3d, {0, 0, 10, 1}, {0, 0, 0, 1}, {0, 1, 0, 0});
	
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
				//"*Box nice_box FF00FF00 { *Data {1 2 3} }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\Pendulum\\Pendulum.fbx\" } }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\AnimCharacter\\AnimatedCharacter.fbx\" } }"
				//"*Model { *Filepath { \"E:\\Dump\\Hyperpose\\fbx\\hyperpose_sample.fbx\" } }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\Pendulum\\Pendulum.fbx\" } *Animation{*Style{Repeat}} }"
				//"*Model { *Filepath { \"E:\\Rylogic\\Code\\art\\models\\AnimCharacter\\AnimatedCharacter.fbx\" } *Animation{*Style{PingPong}} }"
				//"*Model { *Filepath { \"E:\\Dump\\Hyperpose\\fbx\\hyperpose_sample.fbx\" } *Animation{*Style{PingPong}} }"
				//"*Model { *Filepath { \"E:\\Dump\\Hyperpose\\fbx\\hyperpose_sample2.fbx\" } *Animation{*Style{PingPong}} }"
				"*Model { *Filepath { \"E:\\Dump\\Hyperpose\\fbx\\Extra_Wall_Flip.fbx\" } *LoadAtFrame {20} }"//*Animation{*Style{PingPong}} }"
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
	}
	~Main()
	{
		View3D_CubeMapRelease(m_envmap);
		View3D_WindowDestroy(m_win3d);
		View3D_ObjectDelete(m_obj0);
		View3D_ObjectDelete(m_obj1);
		View3D_Shutdown(m_view3d);
	}
	void Step(double dt)
	{
		static double time_scale = 1.0;
		dt *= time_scale;

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

		//auto i2w = m4x4::Transform(time*0.5f, time*0.3f, time*0.1f, v4::Origin());
		//View3D_ObjectO2WSet(m_obj0, To<view3d::Mat4x4>(i2w), nullptr);
		//m_inst0.m_i2w = m4x4::Transform(time*0.5f, time*0.3f, time*0.1f, v4::Origin());
		//m_inst1.m_i2w = m4x4::Transform(time*0.5f, time*0.3f, time*0.1f, v4::Origin());

		// Animation
		static bool animate = true;
		if (animate)
			View3D_ObjectAnimTimeSet(m_obj0, static_cast<float>(m_time), "");
		else
			View3D_ObjectAnimTimeSet(m_obj0, 0, "");


		auto c2w = View3D_CameraToWorldGet(m_win3d);
		SetWindowTextA(*this, pr::FmtS("View3d 12 Test - Cam: %3.3f %3.3f %3.3f  Dir: %3.3f %3.3f %3.3f", c2w.w.x, c2w.w.y, c2w.w.z, -c2w.z.x, -c2w.z.y, -c2w.z.z));
		View3D_WindowRender(m_win3d);
	}
	void OnWindowPosChange(WindowPosEventArgs const& args) override
	{
		Form::OnWindowPosChange(args);
		if (!args.m_before && args.IsResize() && !IsIconic(*this))
		{
			auto rect = ClientRect(false);
			auto dpi = GetDpiForWindow(*this);
			auto w = s_cast<int>(rect.width() * dpi / 96.0);
			auto h = s_cast<int>(rect.height() * dpi / 96.0);
			View3D_WindowBackBufferSizeSet(m_win3d, { w, h }, false);
			View3D_WindowViewportSet(m_win3d, view3d::Viewport{
				.m_x = 0,
				.m_y = 0,
				.m_width = 1.f * w,
				.m_height = 1.f * h,
				.m_min_depth = 0,
				.m_max_depth = 1,
				.m_screen_w = args.m_wp->cx,
				.m_screen_h = args.m_wp->cy,
				});
			//m_wnd.BackBufferSize(sz, false);
			//m_scn.m_viewport.Set(sz);
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
};

// Entry point
int __stdcall WinMain(HINSTANCE hinstance, HINSTANCE, LPTSTR, int)
{
	try
	{
		pr::InitCom com;
		pr::win32::LoadDll<struct View3d>("view3d-12.dll");

		Main main(hinstance);
		main.Show();

		WinGuiMsgLoop loop;
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
