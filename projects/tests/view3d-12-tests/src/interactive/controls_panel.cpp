//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include <algorithm>
#include <cmath>
#include <format>
#include "controls_panel.h"
#include <commctrl.h>

using namespace pr::gui;

namespace view3d_test
{
	// Layout metrics in 96-DPI design units. 'Px' converts them to pixels.
	static constexpr int Spacing = 4;
	static constexpr int ButtonHeight = 24;
	static constexpr int CheckBoxHeight = 20;
	static constexpr int SliderHeight = 26;
	static constexpr int HeadingGap = 10;

	ControlsPanel::Params::Params()
	{
		// Dock to the right with a fixed width, filling the height of the parent.
		// Clip children so that painting the panel background does not erase the native trackbars.
		this->name("controls").dock(EDock::Right).wh(260, Fill).padding(6).style('+', WS_CLIPCHILDREN);
	}

	ControlsPanel::ControlsPanel(Params const& p)
		: Panel(p)
		, m_ctrls()
		, m_sliders()
		, m_cursor_y(0)
	{
		// The trackbar window class is a common control and must be registered before use
		static bool s_registered = []
		{
			INITCOMMONCONTROLSEX icc = { sizeof(INITCOMMONCONTROLSEX), ICC_BAR_CLASSES };
			return ::InitCommonControlsEx(&icc) != FALSE;
		}();
		(void)s_registered;
	}

	ControlsPanel::~ControlsPanel()
	{
		// The trackbars are raw windows, not wingui controls, so they are destroyed explicitly
		Clear();
	}

	// Add a word-wrapped label
	Label* ControlsPanel::AddLabel(std::wstring const& text)
	{
		// Size the label to fit the wrapped text at the full content width
		CreateHandle();
		auto area = ContentArea();
		auto size = MeasureString(text, area.width());
		auto label = std::make_unique<Label>(Label::Params<>().parent(this).dpi(DpiScale::DPI()).text(text.c_str()).xy(0, m_cursor_y).wh(area.width(), size.cy).margin(0));
		label->CreateHandle();
		m_cursor_y += size.cy + Px(Spacing);

		auto ptr = label.get();
		m_ctrls.push_back(std::move(label));
		return ptr;
	}

	// Add a section heading
	void ControlsPanel::AddHeading(std::wstring const& text)
	{
		// A gap above the heading separates it from the previous section
		if (m_cursor_y != 0)
			m_cursor_y += Px(HeadingGap);

		AddLabel(text);
	}

	// Add a check box
	void ControlsPanel::AddCheckBox(std::wstring const& text, bool checked, std::function<void(bool)> on_changed)
	{
		// Create the check box below the previous control
		CreateHandle();
		auto area = ContentArea();
		auto chk = std::make_unique<Button>(Button::Params<>().parent(this).dpi(DpiScale::DPI()).text(text.c_str()).chk_box().style('-', BS_CENTER).style('+', BS_LEFT).xy(0, m_cursor_y).wh(area.width(), Px(CheckBoxHeight)).margin(0));
		chk->CreateHandle();
		m_cursor_y += Px(CheckBoxHeight) + Px(Spacing);

		// Set the initial state before subscribing, because the setter raises 'CheckedChanged'
		chk->Checked(checked);
		chk->CheckedChanged += [cb = std::move(on_changed)](Button& btn, EmptyArgs const&)
		{
			// Report the new state
			cb(btn.Checked());
		};
		m_ctrls.push_back(std::move(chk));
	}

	// Add a button
	void ControlsPanel::AddButton(std::wstring const& text, std::function<void()> on_click)
	{
		// Create the button below the previous control
		CreateHandle();
		auto area = ContentArea();
		auto btn = std::make_unique<Button>(Button::Params<>().parent(this).dpi(DpiScale::DPI()).text(text.c_str()).xy(0, m_cursor_y).wh(area.width(), Px(ButtonHeight)).margin(0));
		btn->CreateHandle();
		m_cursor_y += Px(ButtonHeight) + Px(Spacing);

		btn->Click += [cb = std::move(on_click)](Button&, EmptyArgs const&)
		{
			// Forward the click
			cb();
		};
		m_ctrls.push_back(std::move(btn));
	}

	// Add a horizontal slider
	void ControlsPanel::AddSlider(std::wstring const& text, float min, float max, float value, std::function<void(float)> on_changed, int steps)
	{
		// The label above the trackbar shows the name and current value
		auto label = AddLabel(text);
		m_cursor_y -= Px(Spacing);

		// Create the native trackbar. Its range is [0, steps], mapped linearly onto [min, max].
		auto area = ContentArea();
		auto hwnd = ::CreateWindowExW(0, TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_TABSTOP | TBS_HORZ | TBS_NOTICKS, area.left, area.top + m_cursor_y, area.width(), Px(SliderHeight), m_hwnd, nullptr, ::GetModuleHandleW(nullptr), nullptr);
		Check(hwnd != nullptr, "Failed to create a trackbar");
		m_cursor_y += Px(SliderHeight) + Px(Spacing);

		// Position the trackbar at the initial value
		auto pos = max > min ? static_cast<int>(std::lround((std::clamp(value, min, max) - min) / (max - min) * steps)) : 0;
		::SendMessageW(hwnd, TBM_SETRANGE, TRUE, MAKELPARAM(0, steps));
		::SendMessageW(hwnd, TBM_SETPOS, TRUE, pos);

		auto& slider = m_sliders.emplace_back(Slider{ hwnd, label, text, min, max, steps, std::move(on_changed) });
		UpdateSliderLabel(slider);
	}

	// Remove all controls
	void ControlsPanel::Clear()
	{
		// Destroy the native trackbars, then the wingui controls
		for (auto& slider : m_sliders)
			::DestroyWindow(slider.m_hwnd);

		m_sliders.clear();
		m_ctrls.clear();
		m_cursor_y = 0;
		if (::IsWindow(m_hwnd))
			::InvalidateRect(m_hwnd, nullptr, TRUE);
	}

	// Handle slider notifications
	LRESULT ControlsPanel::WndProc(UINT message, WPARAM wparam, LPARAM lparam)
	{
		// Trackbars report movement to their parent with WM_HSCROLL, with the trackbar's handle in 'lparam'
		if (message == WM_HSCROLL && lparam != 0)
		{
			auto hwnd = reinterpret_cast<HWND>(lparam);
			auto it = std::find_if(m_sliders.begin(), m_sliders.end(), [=](Slider const& s) { return s.m_hwnd == hwnd; });
			if (it != m_sliders.end())
			{
				// Copy the callback, because it may clear this panel and invalidate 'it'
				UpdateSliderLabel(*it);
				auto cb = it->m_on_changed;
				auto value = SliderValue(*it);
				if (cb)
					cb(value);

				return 0;
			}
		}
		return Panel::WndProc(message, wparam, lparam);
	}

	// The area available for controls, in pixels
	Rect ControlsPanel::ContentArea() const
	{
		// wingui offsets the positions of its child controls by the padding, but native child windows such as the trackbars must be offset explicitly
		return ClientRect(true);
	}

	// Convert 'design' from 96-DPI design units to pixels at the panel's DPI
	int ControlsPanel::Px(int design) const
	{
		// The panel's metrics track the DPI of the monitor that the window is on
		return Metrics().X(design);
	}

	// The value of 'slider' at its current position
	float ControlsPanel::SliderValue(Slider const& slider)
	{
		// Map the trackbar position linearly onto [min, max]
		auto pos = static_cast<int>(::SendMessageW(slider.m_hwnd, TBM_GETPOS, 0, 0));
		return slider.m_min + (slider.m_max - slider.m_min) * pos / slider.m_steps;
	}

	// Update the text of the label of 'slider'
	void ControlsPanel::UpdateSliderLabel(Slider const& slider)
	{
		// Show three significant figures, which is enough to compare settings visually
		auto text = std::format(L"{}: {:.3g}", slider.m_text, SliderValue(slider));
		slider.m_label->Text(text.c_str());
	}
}
