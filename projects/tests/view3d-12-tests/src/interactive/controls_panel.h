//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// A docked panel that holds the controls for the active demo.
#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <windows.h>
#include "pr/gui/wingui.h"

namespace view3d_test
{
	// A vertical stack of labels, check boxes, sliders, and buttons.
	// Layout is in pixels. Child controls use the system DPI as their design DPI so that wingui does not rescale them.
	// Layout is done in pixels. Child controls are created with the system DPI as their design DPI so that wingui does not rescale them.
	// Controls are added top to bottom, and 'Clear' removes all of them. The callbacks usually capture the active demo,
	// so the owner must call 'Clear' before destroying that demo.
	struct ControlsPanel :pr::gui::Panel
	{
		// Construction parameters. The panel docks to the right of its parent by default.
		struct Params :pr::gui::Panel::Params<Params>
		{
			Params();
		};

		explicit ControlsPanel(Params const& p);
		~ControlsPanel();

		// Add a word-wrapped label. The returned label stays valid until 'Clear' is called.
		pr::gui::Label* AddLabel(std::wstring const& text);

		// Add a section heading
		void AddHeading(std::wstring const& text);

		// Add a check box. 'on_changed' is called when the user changes the state, but not for the initial state.
		void AddCheckBox(std::wstring const& text, bool checked, std::function<void(bool)> on_changed);

		// Add a button
		void AddButton(std::wstring const& text, std::function<void()> on_click);

		// Add a horizontal slider for values in [min, max], with 'steps' discrete positions. The label shows the current value.
		// 'on_changed' is called when the user moves the slider, but not for the initial value.
		void AddSlider(std::wstring const& text, float min, float max, float value, std::function<void(float)> on_changed, int steps = 100);

		// Remove all controls
		void Clear();

	protected:

		// Handle slider notifications, which the trackbars send to their parent
		LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override;

	private:

		// A native trackbar and the label that shows its value
		struct Slider
		{
			HWND m_hwnd;
			pr::gui::Label* m_label;
			std::wstring m_text;
			float m_min;
			float m_max;
			int m_steps;
			std::function<void(float)> m_on_changed;
		};

		std::vector<std::unique_ptr<pr::gui::Control>> m_ctrls;
		std::vector<Slider> m_sliders;
		int m_cursor_y;

		// The area available for controls, in pixels
		pr::gui::Rect ContentArea() const;

		// Convert 'design' from 96-DPI design units to pixels at the panel's DPI
		int Px(int design) const;

		// The value of 'slider' at its current position
		static float SliderValue(Slider const& slider);

		// Update the text of the label of 'slider'
		static void UpdateSliderLabel(Slider const& slider);
	};
}
