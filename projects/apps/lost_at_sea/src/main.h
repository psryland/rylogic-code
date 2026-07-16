//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#pragma once
#include "src/forward.h"
#include "src/settings.h"
#include "src/core/frame_tasks.h"
#include "src/core/state_snapshot.h"
#include "src/core/sim_state.h"
#include "src/core/physics/physics_system.h"
#include "src/core/cameras/icamera.h"
#include "src/core/input/input_handler.h"
#include "src/world/water/water_system.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/distant_ocean.h"
#include "src/world/terrain/height_field.h"
#include "src/world/terrain/terrain.h"
#include "src/world/sky/procedural_sky.h"
#include "src/world/sky/day_night_cycle.h"
#include "src/world/ship/ship.h"
#include "src/diag/diag_ui.h"

namespace las
{
	// Main application logic
	struct Main :pr::app::Main<Main, MainUI, Settings>
	{
		using base = pr::app::Main<Main, MainUI, Settings>;
		using ImGuiUI = pr::rdr12::imgui::ImGuiUI;
		using CameraPtr = std::shared_ptr<camera::ICamera>;
		static char const* AppName() { return "LostAtSea"; }

		InputHandler m_input;
		CameraPtr m_camera;
		water::System m_water;
		PhysicsSystem m_physics;
		ProceduralSky m_sky;
		DayNightCycle m_day_cycle;
		Ocean m_ocean;
		DistantOcean m_distant_ocean;
		Terrain m_terrain;
		HeightField m_height_field; // CPU-side height queries for future physics
		Ship m_ship;

		// Simulation state snapshot: Step writes, Render reads
		StateSnapshot<SimState> m_sim_state;

		double m_sim_time;
		int64_t m_render_frame;
		int m_camera_mode;  // 0 = Ship, 1 = Free
		std::mutex m_water_generator_position_mutex;
		v2 m_water_generator_position;

		// Task graphs for parallel execution
		pr::task_graph::Graph<StepTaskId> m_step_graph;
		pr::task_graph::Graph<RenderTaskId> m_render_graph;

		// ImGui overlay
		ImGuiUI m_imgui;

		// Diagnostic UI (toggled with F3)
		DiagUI m_diag;

		Main(MainUI& ui);
		~Main();

		// Cycle to the next camera mode
		void CycleCamera();

		void SimStep(double elapsed_seconds);
		void DoRender(bool force = false);
		void RenderUI(Frame& frame);
		void UpdateScene(Scene& scene, UpdateSceneArgs const& args);
	};

	// Main app window
	struct MainUI :pr::app::MainUI<MainUI, Main, pr::gui::WinGuiMsgLoop>
	{
		using base = pr::app::MainUI<MainUI, Main, pr::gui::WinGuiMsgLoop>;
		static wchar_t const* AppTitle() { return L"Lost at Sea"; }

		MainUI(wchar_t const* lpstrCmdLine, int nCmdShow);

		// Override WndProc to forward messages to imgui
		bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override;

		// Override default mouse behaviour
		void OnMouseButton(gui::MouseEventArgs& args) override
		{
			m_main->m_input.OnMouseButton(args);
		}
		void OnMouseClick(gui::MouseEventArgs& args) override
		{
			m_main->m_input.OnMouseClick(args);
		}
		void OnMouseMove(gui::MouseEventArgs& args) override
		{
			m_main->m_input.OnMouseMove(args);
		}
		void OnMouseWheel(gui::MouseWheelArgs& args) override
		{
			m_main->m_input.OnMouseWheel(args);
		}

		// Override default keyboard behaviour
		void OnKey(KeyEventArgs& args) override
		{
			m_main->m_input.OnKey(args);
		}
	};
}

namespace pr::app
{
	std::unique_ptr<IAppMainUI> CreateUI(wchar_t const* lpstrCmdLine, int nCmdShow);
}