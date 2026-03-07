// Fluid
#include "src/forward.h"
#include "src/fluid_visualisation.h"
#include "src/idemo_scene.h"
#include "src/probe.h"
#include "src/demo/tube2d.h"
#include "src/demo/scene2d.h"
#include "src/demo/scene3d.h"

using namespace pr;
using namespace pr::gui;
using namespace pr::fluid;

// Application window
struct Main :Form, IProbeActions
{
	// Sim constants
	inline static constexpr int MaxParticleCount = 10;
	inline static constexpr float SimTimeStepSec = 0.01f;
	inline static constexpr float ParticleRadius = 0.05f;
	inline static constexpr int GridCellCount = 65521;//1021;//65521;//1048573;//16777213;
	inline static constexpr iv2 WinSize = { 2048, 1600 };

	enum { IDR_MAINFRAME = 100 };
	enum { ID_FILE, ID_FILE_EXIT };
	enum { IDC_PROGRESS = 100, IDC_NM_PROGRESS, IDC_MODELESS, IDC_CONTEXTMENU, IDC_POSTEST, IDC_ABOUT, IDC_MSGBOX, IDC_SCINT, IDC_TAB, IDC_TAB1, IDC_TAB2, IDC_SPLITL, IDC_SPLITR };

	enum class ERunMode
	{
		Paused,
		SingleStep,
		FreeRun,
	};

	using rtc_time_t = std::chrono::high_resolution_clock::time_point;
	using ema_t = math::ExpMovingAvr<double>;
	using demo_scenes_t = std::vector<std::shared_ptr<IDemoScene>>;
	using colours_t = FluidSimulation::ColourData;

	struct FPS
	{
		ema_t m_fps = {20};
		rtc_time_t m_time0 = {};
		int m_frame0 = {};

		void reset()
		{
			*this = FPS{};
		}
		double get() const
		{
			return m_fps.Mean();
		}
		void frame(int frame)
		{
			using namespace std::chrono;

			auto time = rtc_time_t::clock::now();
			if (time - m_time0 > milliseconds(200))
			{
				auto duration = 1e-9 * duration_cast<nanoseconds>(time - m_time0).count();
				auto fps = (frame - m_frame0) / duration;
				m_fps.Add(fps);
				m_time0 = time;
				m_frame0 = frame;
			}
		}
	};

	rdr12::Renderer m_rdr;
	rdr12::Window m_wnd;
	rdr12::Scene m_scn;
	GpuJob m_job;

	Probe m_probe;
	demo_scenes_t m_demo;
	WinGuiMsgLoop m_loop;
	FluidSimulation m_fluid_sim;
	FluidVisualisation m_fluid_vis;
	particles_t m_cpu_particles;
	colours_t m_colour_data;

	std::string m_title;
	ERunMode m_run_mode;
	int m_scene_index;
	bool m_frame_lock;
	int m_last_frame;
	float m_time;
	FPS m_fps;

	Main(HINSTANCE hinst)
		: Form(Params<>()
			.name("main")
			.title(L"Fluid")
			.xy(1200, 100)
			.wh(WinSize.x, WinSize.y, true)
			.main_wnd()
			.dbl_buffer()
			.wndclass(RegisterWndClass<Main>()))
		, m_rdr(rdr12::RdrSettings(hinst).DebugLayer())
		, m_wnd(m_rdr, rdr12::WndSettings(CreateHandle(), true, m_rdr.Settings()).BackgroundColour(0xFFA0A080))
		, m_scn(m_wnd)
		, m_job(m_rdr.D3DDevice(), "Fluid", 0xFFA83250, 5)
		, m_probe(m_rdr, ParticleRadius, this)
		, m_demo(CreateDemo())
		, m_loop()
		, m_fluid_sim(m_rdr)
		, m_fluid_vis(m_rdr, m_scn)
		, m_cpu_particles()
		, m_colour_data()
		, m_title()
		, m_fps()
		, m_run_mode(ERunMode::Paused)
		, m_scene_index(-1)
		, m_frame_lock(rdr12::pix::IsAttachedForGpuCapture())
		, m_last_frame(-1)
		, m_time()
	{
		Tweakables::filepath = "E:/Rylogic/Code/projects/apps/fluid/tweakables.ini";
		ApplyTweakables();

		// Load the next demo scene
		NextScene();

		m_loop.AddMessageFilter(*this);
		m_loop.AddLoop(1.0 / SimTimeStepSec, false, [this](double dt)
		{
			auto elapsed_s = static_cast<float>(dt);

			//hack
			//elapsed_s *= 0.1f;

			switch (m_run_mode)
			{
				case ERunMode::Paused:
				{
					break;
				}
				case ERunMode::SingleStep:
				{
					m_time += elapsed_s;
					StepSim(elapsed_s);
					m_run_mode = ERunMode::Paused;
					break;
				}
				case ERunMode::FreeRun:
				{
					m_time += elapsed_s;
					StepSim(elapsed_s);
					m_fps.frame(m_fluid_sim.m_frame);
					break;
				}
			}
		});
		m_loop.AddLoop(20.0, false, [this](auto) { RenderLoop(); });
		m_loop.AddLoop(10.0, false, [this](auto) { ApplyTweakables(); });
	}

	// Run the application
	int Run()
	{
		return m_loop.Run();
	}

	// Reset the sim
	void Reset()
	{
		// Preserve the camera
		auto cam = m_scn.m_cam;

		m_probe.Reset();
		m_fps.reset();
		m_cpu_particles.resize(0);
		m_fluid_sim.m_frame = 0;
		m_last_frame = -1;
		m_time = 0;

		// Reset the sim
		m_scene_index = -1;
		NextScene();

		m_scn.m_cam = cam;
	}

	// Advance the Simulation
	void StepSim(float elapsed_s)
	{
		bool read_back = true;

		// Colour the particles
		m_fluid_sim.UpdateColours(m_job, m_colour_data);

		// Apply the probe
		m_probe.Step(m_job, elapsed_s);

		// Step the simulation
		m_fluid_sim.Step(m_job, elapsed_s, read_back);

		// Debugging
		m_fluid_sim.Debugging(m_job, elapsed_s, m_probe.Data());

		// Run the jobs
		m_job.Run();

		// Update the particle count
		m_fluid_sim.Config.NumParticles = m_fluid_sim.Output.ParticleCount();

		// Update the sys-memory copy of the particle buffer
		if (read_back)
		{
			m_cpu_particles.resize(m_fluid_sim.Config.NumParticles);
			m_fluid_sim.Output.ReadParticles(0, isize(m_cpu_particles), [&](Particle const* particles, Dynamics const* dynamics)
			{
				for (int i = 0; i != isize(m_cpu_particles); ++i)
				{
					m_cpu_particles[i].pos = particles[i].pos;
					m_cpu_particles[i].vel = dynamics[i].vel;
					m_cpu_particles[i].acc = dynamics[i].accel.w0();
					m_cpu_particles[i].surface = dynamics[i].surface;
				}
			});
		}
	}

	// Render the simulation
	void RenderLoop()
	{
		// Update the window title
		UpdateWindowTitle();

		// Use this only render per main loop step
		if (m_frame_lock && m_last_frame == m_fluid_sim.m_frame)
			return;

		EScene scene = EScene::None;

		Tweakable<bool, "ShowParticles"> ShowParticles = true;
		if (ShowParticles)
		{
			Tweakable<float, "DropletSize"> DropletSize = 0.4f;
			m_fluid_vis.m_gs_points->m_size = v2(DropletSize);
			scene |= EScene::Particles;
		}

		// Show vectors for each particle
		Tweakable<int, "VectorFieldMode"> VectorFieldMode = 0;
		if (VectorFieldMode != 0)
		{
			Tweakable<float, "VectorFieldScale"> VectorFieldScale = 0.01f;
			m_fluid_vis.UpdateVectorField(m_cpu_particles, ParticleRadius, VectorFieldScale, VectorFieldMode);
			scene |= EScene::VectorField;
		}

		// Show the map
		Tweakable<int, "MapType"> MapType = 0;
		if (MapType != 0)
		{
			auto map_size = m_fluid_vis.m_tex_map->m_dim.xy;
			FluidSimulation::MapData map_data = {
				.MapToWorld = m4x4::Scale(2.0f / map_size.x, 2.0f / map_size.y, 1.0f, v4(-1, -1, 0, 1)),
				.TexDim = map_size,
				.Type = MapType,
			};
			m_fluid_sim.GenerateMap(m_job, m_fluid_vis.m_tex_map, map_data, m_colour_data);
			scene |= EScene::Map;
		}

		// Show particle within the probe radius
		//{
		//	m_fluid_sim.UpdateColours(m_job, m_colour_data);
		//	m_fluid_sim.Debugging(m_job, m_probe.Data());
		//	m_job.Run();
		//}


		// Wait for the compute job to finish
		m_job.m_gsync.Wait();

		// Render the particles
		m_scn.ClearDrawlists();
		m_probe.AddToScene(m_scn);
		m_fluid_vis.AddToScene(m_scn, scene, m_fluid_sim.Config.NumParticles);

		// Render the frame
		auto& frame = m_wnd.NewFrame();
		m_scn.Render(frame);
		m_wnd.Present(frame, rdr12::EGpuFlush::Block);

		m_last_frame = m_fluid_sim.m_frame;
	}

	// Tweak settings
	void ApplyTweakables()
	{
		Tweakable<v4, "Gravity"> Gravity = v4(0, -10, 0, 0);
		Tweakable<float, "ForceScale"> ForceScale = 10.0f;
		Tweakable<float, "ForceRange"> ForceRange = 1.0f;
		Tweakable<float, "ForceBalance"> ForceBalance = 0.8f;
		Tweakable<float, "ForceDip"> ForceDip = 0.05f;
		Tweakable<float, "Viscosity"> Viscosity = 10.0f;
		Tweakable<float, "ThermalDiffusion"> ThermalDiffusion = 0.01f;
		m_fluid_sim.Config.Dyn.Gravity = Gravity;
		m_fluid_sim.Config.Dyn.ForceScale = ForceScale;
		m_fluid_sim.Config.Dyn.ForceRange = ForceRange;
		m_fluid_sim.Config.Dyn.ForceBalance = ForceBalance;
		m_fluid_sim.Config.Dyn.ForceDip = ForceDip;
		m_fluid_sim.Config.Dyn.Viscosity = Viscosity;
		m_fluid_sim.Config.Dyn.ThermalDiffusion = ThermalDiffusion;

		Tweakable<v2, "Restitution"> Restitution = v2{ 1.0f, 1.0f };
		m_fluid_sim.m_collision.Config.Restitution = Restitution;

		Tweakable<int, "ColourScheme"> ColourScheme = 0;
		Tweakable<v2, "ColourRange"> ColourRange = v2{ 0.0f, 1.0f };
		m_colour_data.Range = ColourRange;
		m_colour_data.Scheme = ColourScheme;
	}

	// Update the window title
	void UpdateWindowTitle()
	{
		m_title = "Fluid";

		if (m_frame_lock)
			m_title.append(std::format("[FL={}]", m_last_frame));

		auto pos = m_probe.m_position;
		m_title.append(std::format(" - FPS: {:.3f}", m_fps.get()));
		m_title.append(std::format(" - Pos: {:.3f} {:.3f} {:.3f}", pos.x, pos.y, pos.z));
		m_title.append(std::format(" - Probe Radius: {:.3f}", m_probe.m_radius));

		if (m_probe.m_mode != EProbeMode::None)
		{
			auto count = 0;
			auto nearest = 0;
			
			auto rad_sq = Sqr(m_probe.m_radius);
			auto nearest_dist_sq = std::numeric_limits<float>::max();
			for (auto& particle : m_cpu_particles)
			{
				auto dist_sq = LengthSq(particle.pos - pos);
				if (dist_sq > rad_sq)
					continue;

				++count;
				if (dist_sq < nearest_dist_sq)
				{
					nearest = s_cast<int>(&particle - m_cpu_particles.data());
					nearest_dist_sq = dist_sq;
				}
			}
			m_title.append(std::format(" - Nearest: {}", nearest));
			m_title.append(std::format(" - Count: {}", count));
		}
		else
		{
			auto c2w = m_scn.m_cam.CameraToWorld();
			m_title.append(std::format(" - Time: {:.3f}s", m_time));
			m_title.append(std::format(" - Frame: {}", m_fluid_sim.m_frame));
			m_title.append(std::format(" - PCount: {}", m_fluid_sim.Config.NumParticles));
			m_title.append(std::format(" - Cam: {:.3f} {:.3f} {:.3f}  Dir: {:.3f} {:.3f} {:.3f}", c2w.w.x, c2w.w.y, c2w.w.z, -c2w.z.x, -c2w.z.y, -c2w.z.z));
			m_title.append(std::format(" - NRG: {}", m_fluid_sim.Output.DebugResults().p0_energy));
		}

		SetWindowTextA(*this, m_title.c_str());
	}

	// Probe actions
	v4 PositionProbe(gui::Point ss_pt) const override
	{
		if (m_scene_index == -1)
			return v4::Origin();

		auto& scene = *m_demo[m_scene_index].get();
		return scene.PositionProbe(ss_pt, m_scn);
	}
	void PushPull(GpuJob& job, FluidSimulation::ProbeData const& data) override
	{
		m_fluid_sim.ApplyProbeForces(job, data);
	}
	void SourceSink(GpuJob& job, int count) override
	{
		// Add up to 'count' particles from within the probe volume
		if (count > 0)
		{
			int start = m_fluid_sim.Config.NumParticles;
			std::vector<Particle> particles(s_cast<size_t>(std::min(count, MaxParticleCount - start)));
			std::default_random_engine rng(s_cast<uint32_t>(m_time * 1000.0f));
			for (auto& particle : particles)
			{
				auto centre = m_probe.m_position.xyz;
				auto rad = v3(m_probe.m_radius);
				auto pos = Random<v3>(rng, centre - rad, centre + rad).w1();
				if (m_fluid_sim.m_collision.Config.SpatialDimensions != 3) pos.z = 0;
				particle = Particle{
					.pos = pos,
					.col = v4::One(),
				};
			}

			// Add the new particles to the particle buffer
			m_fluid_sim.WriteParticles(job, start, particles, {});
			m_fluid_sim.Config.NumParticles += isize(particles);
		}

		// Remove up to 'count' particles from within the probe volume
		else
		{
			/* TODO
			int removed = 0;
			auto rad_sq = Sqr(m_probe.m_radius);
			for (auto const& particle : m_cpu_particles)
			{
				if (LengthSq(particle.pos - m_probe.m_position) > rad_sq) continue;

				removed++;
			}

			// Update the whole particle buffer because we could've removed from anywhere
			m_fluid_sim.WriteParticles(job, 0, m_cpu_particles);
			m_fluid_sim.Config.NumParticles = isize(m_cpu_particles);
			*/
		}
	}

	// Windows events
	void OnWindowPosChange(WindowPosEventArgs const& args) override
	{
		Form::OnWindowPosChange(args);
		if (!args.m_before && args.IsResize() && !IsIconic(*this))
		{
			auto rect = ClientRect(false);
			auto dpi = GetDpiForWindow(*this);
			auto w = s_cast<int>(rect.width() * dpi / 96.0);
			auto h = s_cast<int>(rect.height() * dpi / 96.0);
			m_wnd.BackBufferSize({ w, h }, false);
			m_scn.m_viewport.Set({ w, h });
			m_scn.m_cam.Aspect(1.0*w/h);
		}
	}
	void OnMouseButton(MouseEventArgs& args) override
	{
		Form::OnMouseButton(args);
		m_fluid_vis.OnMouseButton(args);
		m_probe.OnMouseButton(args);
		if (args.m_handled)
			return;
		
		auto nss_point = m_scn.m_viewport.SSPointToNSSPoint(To<v2>(args.m_point));
		auto nav_op =
			pr::AllSet(args.m_button, EMouseKey::Left) ? Camera::ENavOp::Rotate :
			pr::AllSet(args.m_button, EMouseKey::Right) ? Camera::ENavOp::Translate :
			Camera::ENavOp::None;

		m_scn.m_cam.MouseControl(nss_point, nav_op, true);
	}
	void OnMouseMove(MouseEventArgs& args) override
	{
		Form::OnMouseMove(args);
		m_fluid_vis.OnMouseMove(args);
		m_probe.OnMouseMove(args);
		if (args.m_handled)
			return;

		auto nss_point = m_scn.m_viewport.SSPointToNSSPoint(To<v2>(args.m_point));
		auto nav_op =
			pr::AllSet(args.m_button, EMouseKey::Left) ? Camera::ENavOp::Rotate :
			pr::AllSet(args.m_button, EMouseKey::Right) ? Camera::ENavOp::Translate :
			Camera::ENavOp::None;

		m_scn.m_cam.MouseControl(nss_point, nav_op, false);
	}
	void OnMouseWheel(MouseWheelArgs& args) override
	{
		Form::OnMouseWheel(args);
		m_fluid_vis.OnMouseWheel(args);
		m_probe.OnMouseWheel(args);
		if (args.m_handled)
			return;

		auto nss_point = m_scn.m_viewport.SSPointToNSSPoint(To<v2>(args.m_point));
		m_scn.m_cam.MouseControlZ(nss_point, args.m_delta, true);
	}
	void OnKey(KeyEventArgs& args) override
	{
		Form::OnKey(args);
		m_fluid_vis.OnKey(args);
		m_probe.OnKey(args);
		if (args.m_handled)
			return;

		if (args.m_down) return;
		switch (args.m_vk_key)
		{
			case VK_ESCAPE:
			{
				Close();
				break;
			}
			case 'F':
			{
				m_frame_lock = !m_frame_lock;
				break;
			}
			case 'R':
			{
				Reset();
				break;
			}
			case VK_F5:
			{
				m_run_mode = m_run_mode != ERunMode::FreeRun ? ERunMode::FreeRun : ERunMode::Paused;
				break;
			}
			case VK_F6:
			{
				m_run_mode = ERunMode::SingleStep;
				break;
			}
			case VK_SPACE:
			{
				m_run_mode = KeyDown(VK_CONTROL) ? ERunMode::FreeRun : ERunMode::SingleStep;
				break;
			}
		}
	}

	// Load the next demo scene
	void NextScene()
	{
		// Advance to the next scene
		if (m_scene_index + 1 == isize(m_demo)) return;
		++m_scene_index;

		// Get the next scene
		auto& scene = *m_demo[m_scene_index].get();

		// Remove models from the draw lists
		m_scn.ClearDrawlists();

		// Setup the simulation (override defaults)
		FluidSimulation::Setup fs_setup = {
			.ParticleCapacity = MaxParticleCount,
			.Config = {
				.Particles = {
					.Radius = ParticleRadius,
				},
				.NumParticles = isize(scene.Particles()),
			},
			.ParticleInitData = scene.Particles(),
			.DynamicsInitData = scene.Dynamics(),
		};
		ParticleCollision::Setup pc_setup = {
			.PrimitiveCapacity = isize(scene.Collision()),
			.Config = {
				.NumPrimitives = isize(scene.Collision()),
				.SpatialDimensions = scene.SpatialDimensions(),
				.Culling = scene.Culling(),
			},
			.CollisionInitData = scene.Collision(),
		};
		SpatialPartition::Setup sp_setup = {
			.Capacity = MaxParticleCount,
			.Config {
				.CellCount = GridCellCount,
				.GridScale = 1.0f / ParticleRadius,
			},
		};

		// Reset the sim for the current scene
		m_fluid_sim.Init(m_job, fs_setup, pc_setup, sp_setup);

		// Reset the visualisation for the current scene
		m_fluid_vis.Init(MaxParticleCount, scene.LdrScene(), m_fluid_sim.m_r_particles);

		// Set the initial camera position
		scene.Camera(m_scn.m_cam);
		m_scn.m_cam.Aspect(m_scn.m_viewport.Aspect());
	}

	// Create the scenes of the demo
	static demo_scenes_t CreateDemo()
	{
		auto scenes = demo_scenes_t();
		scenes.emplace_back(new Tube2d(MaxParticleCount, ParticleRadius));
		//scenes.emplace_back(new Scene2d(MaxParticleCount));
		//scenes.emplace_back(new Scene3d(MaxParticleCount));
		return scenes;
	}

	// Error handler
	static void __stdcall ReportError(void*, char const* msg, char const* filepath, int line, int64_t)
	{
		std::cout << filepath << "(" << line << "): " << msg << std::endl;
	}
};

// Entry point
int __stdcall wWinMain(HINSTANCE hinstance, HINSTANCE, LPWSTR, int)
{
	try
	{
		InitCom com;

		Main main(hinstance);
		main.Show();
		return main.Run();
	}
	catch (std::exception const& ex)
	{
		OutputDebugStringA("Died: ");
		OutputDebugStringA(ex.what());
		OutputDebugStringA("\n");
		return -1;
	}
}
