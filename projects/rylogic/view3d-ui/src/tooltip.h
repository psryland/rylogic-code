//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Hover tooltips. A tooltip describes one target: either a control with tooltip text (the hovered interactive control or its
// nearest ancestor that has tooltip text), or an application target set with TooltipSet against a root. The tooltip appears
// after the pointer rests on its target for TooltipDelayMs, stays where it appeared until the target changes, and is dismissed
// by a press, wheel, key or focus loss until the target changes. Timing uses host time from Update(), so it is deterministic.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "pr/view3d-ui/draw_packet.h"
#include "tree.h"
#include "input.h"

namespace pr::view3d::ui
{
	class TextShaper;

	// How long the pointer must rest on a target before its tooltip appears, in host milliseconds.
	constexpr double TooltipDelayMs = 500.0;

	// How far the pointer may drift, in DIPs, while still counting as resting.
	constexpr float TooltipRestToleranceDip = 3.0f;

	// Space between the tooltip text and the tooltip edge, in DIPs, inside any style border.
	constexpr float TooltipPadX = 6.0f;
	constexpr float TooltipPadY = 3.0f;

	// The thing a tooltip describes. 'control_id' is non-zero for a control tooltip; otherwise 'app_key' names an application
	// target. 'root_id' is the root whose tooltip style and font are used. A value-initialised target means "no tooltip".
	struct TooltipTarget
	{
		ControlId control_id;
		ControlId root_id;
		std::uint64_t app_key;

		friend bool operator==(TooltipTarget const&, TooltipTarget const&) = default;
	};

	// The tooltip to draw this frame: its text and its box in viewport DIPs. Only valid while 'visible' is true.
	struct TooltipView
	{
		bool visible;
		ControlId root_id;
		std::string text;
		Rect bounds;
	};

	// Tooltip state that persists across input records and Update() calls.
	class TooltipState
	{
	public:
		// Application tooltip set with TooltipSet. 'm_app_key == 0' means none.
		ControlId m_app_root_id = 0;
		std::uint64_t m_app_key = 0;
		std::string m_app_text;

		// Latest pointer position in viewport DIPs, and where the pointer most recently came to rest.
		Vec2 m_pointer = {};
		Vec2 m_rest_point = {};
		bool m_pointer_known = false;
		bool m_rest_restart = false;

		// The current target and its timing. 'm_dismissed' suppresses the target's tooltip until the target changes. 'm_anchor' is the
		// rest point captured when the tooltip appeared, so a shown tooltip stays put while the pointer moves over the same target.
		TooltipTarget m_target = {};
		double m_rest_start_ms = 0.0;
		Vec2 m_anchor = {};
		bool m_shown = false;
		bool m_dismissed = false;

		// What the most recent Update() decided to draw.
		TooltipView m_view = {};

		// Set or clear (key == 0) the application tooltip. The caller has validated the root and the text.
		void SetApp(ControlId root_id, std::uint64_t key, std::string_view text);

		// Update pointer tracking and dismissal from one input record, after the input state machine has processed it.
		void ObserveInput(NormalizedInput const& input);

		// Clear the application tooltip when its root is no longer in the accepted tree.
		void Prune(TreeModel const& tree);

		// Choose the target for this frame, advance the rest timer to 'time_ms', and measure the tooltip box when it is shown.
		// 'shaper' may be null, in which case the text size is estimated from the font size.
		void Update(TreeModel const& tree, InputState const& input, ViewportState const& viewport, TextShaper* shaper, double time_ms);
	};
}
