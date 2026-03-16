#pragma once
#include "src/forward.h"
#include "src/scene/scene.h"
#include "src/ui/recent_files.h"
#include "src/ui/media_panel.h"
#include "src/ui/details_panel.h"

namespace physics_sandbox
{
	// Main window for the physics sandbox application.
	// Assembles the 3D viewport, media controls, details panel, and menu bar
	// into a resizable layout, and wires up the simulation loop.
	struct SandboxUI : Form
	{
		// UI components, ordered by docking precedence (bottom-up):
		// StatusBar docks to bottom, MediaPanel above it, DetailsPanel to right,
		// View3DPanelStatic fills the remaining space.
		StatusBar m_status;
		MediaPanel m_media;
		DetailsPanel m_details;
		View3DPanelStatic m_view3d;

		// Physics simulation
		Scene m_scene;
		int m_steps_remaining; // 0 = paused, -1 = running, N = step N times
		bool m_pause_on_collision;

		// The scenario to load on reset.
		EScenario m_scenario;

		// Path of the last loaded scene file, so Reset reloads the same scene
		std::filesystem::path m_scene_filepath;

		// Most-recently-used scene files list, persisted to %APPDATA%
		RecentFiles m_recent;

		// Cached status text to avoid flickering from constant WM_SETTEXT
		std::wstring m_last_status;

		// FPS measurement
		int m_frame_count;
		double m_fps_elapsed;
		double m_fps;

		// Rate-limiting accumulators for expensive operations.
		// These prevent costly Win32 API calls from running at the full render rate.
		double m_title_elapsed;   // Title bar update interval (every 0.25s)
		double m_details_elapsed; // Details panel update interval (every 0.2s)
		double m_status_elapsed;  // Status panel update interval (every 0.2s)

		// Set when WM_CLOSE is received to prevent step/render after destruction begins
		bool m_closing;

		SandboxUI();
		~SandboxUI();

		// Process window messages for this form.
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override;

		// Reset the scene and sync graphics
		void ResetScene();

		// Show the Open File dialog and load a JSON scene file
		void OpenSceneFile();

		// Rebuild the Recent Files submenu from the current m_recent list
		void RebuildRecentFilesMenu();

		// Load a scene from a JSON file path (by value to avoid dangling references)
		void LoadSceneFile(std::filesystem::path filepath);

		// Advance the simulation by one timestep
		void Step(double elapsed_seconds);

		// Render a frame: sync graphics, rebuild overlays, update details panel
		void Render(double elapsed_seconds);

		// Compute a bounding box that encompasses all bodies in the scene
		BBox ComputeSceneBBox() const;
	};
}
