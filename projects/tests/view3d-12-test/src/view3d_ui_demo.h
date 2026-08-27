//*********************************************
// View3d 12 Test
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Retained-mode View3DUI gallery demonstration, extracted from 'main.cpp' so the application's
// window/scene/audio glue is not entangled with UI construction, editing, and event dispatch.
#pragma once
#include <functional>
#include <memory>
#include <windows.h>
#include "pr/view3d-12/view3d-dll.h"

namespace view3d_test
{
	// Owns the complete View3DUI gallery demonstration: the runtime/context lifetime, the retained
	// control/style/template tree, the dimension text editor's parse/validation state, gallery
	// action status, per-frame event drain/dispatch, and the viewport/camera snapshot fed to
	// View3DUI's 'Update'. The only state this class does not own is the application's rendered
	// box model, which is reported back to the caller through the constructor's callback.
	class View3dUiDemo
	{
		struct Impl;
		std::unique_ptr<Impl> m_impl;

	public:

		// Invoked once per accepted 'Update' command in the dimension editor, with the newly validated positive dimension.
		using UpdateBoxDimensionsCB = std::function<void(float)>;

		// Create the View3DUI runtime/context for 'window' and populate the initial gallery tree.
		View3dUiDemo(pr::view3d::DllHandle view3d, pr::view3d::Window window, UpdateBoxDimensionsCB update_box_dimensions);
		~View3dUiDemo();

		View3dUiDemo(View3dUiDemo&&) = delete;
		View3dUiDemo(View3dUiDemo const&) = delete;
		View3dUiDemo& operator=(View3dUiDemo&&) = delete;
		View3dUiDemo& operator=(View3dUiDemo const&) = delete;

		// Forward one raw Win32 message to View3DUI before the host window's own input handling.
		// Returns true if View3DUI consumed the message, in which case 'result' holds its LRESULT.
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result);

		// Advance the UI clock by 'elapsed_seconds', drain and dispatch queued gallery events, then
		// refresh View3DUI's retained layout/semantics/draw state from 'hwnd's current client size,
		// DPI, and the View3D window's camera.
		void Update(HWND hwnd, double elapsed_seconds);
	};
}
