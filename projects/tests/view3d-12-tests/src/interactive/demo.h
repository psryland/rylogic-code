//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Interactive demos. Each demo shows one view3d feature that needs visual inspection.
#pragma once
#include <filesystem>
#include <memory>
#include <span>
#include <string_view>
#include <vector>
#include <windows.h>
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/audio/audio-dll.h"

namespace view3d_test
{
	struct ControlsPanel;

	// Resources that the interactive host shares with the active demo.
	// The host owns all of them and keeps them alive for longer than any demo.
	struct DemoContext
	{
		pr::view3d::DllHandle m_view3d;  // The view3d DLL context
		pr::view3d::Window m_window;     // The window that the demo renders into
		HWND m_view_hwnd;                // The native window hosting 'm_window'
		std::filesystem::path m_assets;  // Root of a rylogic-assets checkout
		pr::audio::DllHandle m_audio;    // The audio DLL context
		ControlsPanel& m_controls;       // Panel for the demo's controls. The host clears it before destroying the demo.
	};

	// An interactive demo of one view3d feature.
	// The constructor adds the demo's objects and controls, and the destructor releases what the demo created. The host removes all
	// objects from the window and restores the shared scene settings (lights, camera, effects) between demos, so demos may change them freely.
	struct IDemo
	{
		virtual ~IDemo() = default;

		// Advance the demo by 'dt' seconds of real time. Called once per frame before rendering.
		virtual void Step(double dt)
		{
			// Static demos have nothing to animate
			(void)dt;
		}

		// Handle a key event in the 3D view. Set 'args.m_handled' on both the key down and key up events of keys the demo uses,
		// so that the default view3d key bindings do not also act on them.
		virtual void OnKey(pr::gui::KeyEventArgs& args)
		{
			// Demos without key bindings leave the default bindings active
			(void)args;
		}

		// Handle a mouse button event in the 3D view. Set 'args.m_handled' to prevent the default camera navigation.
		virtual void OnMouseButton(pr::gui::MouseEventArgs& args)
		{
			// Demos without mouse bindings leave the default navigation active
			(void)args;
		}

		// Handle a raw window message sent to the 3D view, before any other handling. Return true if the message was consumed.
		virtual bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result)
		{
			// Demos without raw input handling leave every message to the host
			(void)hwnd, (void)message, (void)wparam, (void)lparam, (void)result;
			return false;
		}
	};

	// Create a demo instance
	using DemoFactory = std::unique_ptr<IDemo>(*)(DemoContext const& ctx);

	// A catalogue entry describing one demo
	struct DemoInfo
	{
		wchar_t const* m_category;      // The 'Demos' menu sub menu that contains this demo
		std::string_view m_name;        // Unique name used on the command line and in the settings file
		wchar_t const* m_display_name;  // Menu label
		wchar_t const* m_description;   // What the demo shows and how to use it
		DemoFactory m_create;           // Create the demo
	};

	// All demos, in menu order. Demos in the same category are adjacent.
	std::span<DemoInfo const> DemoCatalogue();

	// Find a demo by name. Case and the difference between '-' and '_' are ignored. Returns null if there is no matching demo.
	DemoInfo const* FindDemo(std::string_view name);

	// Shared helpers for demos
	namespace demo
	{
		// Owns view3d objects and adds them to a window. Deletes the objects on destruction.
		struct SceneObjects
		{
			pr::view3d::Window m_window;
			std::vector<pr::view3d::Object> m_objects;

			explicit SceneObjects(pr::view3d::Window window);
			~SceneObjects();
			SceneObjects(SceneObjects const&) = delete;
			SceneObjects& operator=(SceneObjects const&) = delete;

			// Create an object from LDraw script and add it to the window. Throws if the script does not create an object.
			pr::view3d::Object Add(char const* ldr_script);

			// Take ownership of an existing object and add it to the window
			pr::view3d::Object Add(pr::view3d::Object object);
		};

		// The light at 'index', modified by 'edit', then applied to 'window'
		template <typename Edit>
		void EditLight(pr::view3d::Window window, int index, Edit edit)
		{
			// Read-modify-write so unmodified light properties keep their current values
			auto light = View3D_LightGet(window, index);
			edit(light);
			View3D_LightSet(window, index, light);
		}
	}
}
