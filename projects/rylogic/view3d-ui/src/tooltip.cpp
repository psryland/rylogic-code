//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "tooltip.h"
#include "control_geometry.h"
#include "text_layout.h"
#include "text_shaper.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Offset of the tooltip's top-left corner from the pointer, in DIPs, so the box sits below and to the right of the
		// arrow cursor instead of under it.
		constexpr float TooltipOffsetX = 12.0f;
		constexpr float TooltipOffsetY = 20.0f;

		// The root that owns 'id'.
		ControlId RootOf(TreeModel const& tree, ControlId id)
		{
			// Climb parents until reaching the root, whose parent id is 0.
			for (;;)
			{
				auto const& node = tree.m_controls.at(id);
				if (node.desc.parent_id == 0)
					return id;

				id = node.desc.parent_id;
			}
		}

		// The target under the pointer: the hovered control or its nearest ancestor with tooltip text, otherwise the application
		// target when the pointer is not over any interactive control.
		TooltipTarget ResolveTarget(TreeModel const& tree, InputState const& input, TooltipState const& state)
		{
			// A control under the pointer hides the application tooltip, even when the control has no tooltip of its own.
			if (input.m_hover_id != 0)
			{
				// Search the hovered control and its ancestors for tooltip text.
				for (auto id = input.m_hover_id; id != 0;)
				{
					auto const& node = tree.m_controls.at(id);
					if (!node.tooltip.empty())
						return TooltipTarget{ .control_id = id, .root_id = RootOf(tree, id), .app_key = 0 };

					id = node.desc.parent_id;
				}
				return TooltipTarget{};
			}

			// No control is hovered, so the application target applies.
			if (state.m_app_key != 0)
				return TooltipTarget{ .control_id = 0, .root_id = state.m_app_root_id, .app_key = state.m_app_key };

			return TooltipTarget{};
		}

		// Estimated text size for a context without a text shaper: half an em per byte on the longest line, and 1.25 em per line.
		Vec2 EstimateTextSize(std::string_view text, float font_size)
		{
			// Count lines and the longest line, in bytes, splitting on explicit line breaks.
			auto lines = 1;
			auto longest = std::size_t{};
			auto line_start = std::size_t{};
			for (auto i = std::size_t{}; i <= text.size(); ++i)
			{
				// Close the current line at a break or at the end of the text.
				if (i != text.size() && text[i] != '\n')
					continue;

				longest = std::max(longest, i - line_start);
				line_start = i + 1;
				if (i != text.size())
					++lines;
			}
			return Vec2{ static_cast<float>(longest) * font_size * 0.5f, static_cast<float>(lines) * font_size * 1.25f };
		}
	}

	void TooltipState::SetApp(ControlId root_id, std::uint64_t key, std::string_view text)
	{
		// A zero key clears the application tooltip; otherwise replace it. Keeping the key lets the text change without
		// restarting the rest timer.
		m_app_root_id = key != 0 ? root_id : 0;
		m_app_key = key;
		m_app_text = key != 0 ? std::string(text) : std::string{};
	}

	void TooltipState::ObserveInput(NormalizedInput const& input)
	{
		// Pointer movement restarts the rest timer until the tooltip shows. A shown tooltip stays where it appeared.
		switch (input.kind)
		{
			case EInputKind::PointerMove:
			{
				// Small drifts still count as resting, so a slightly unsteady hand does not keep restarting the delay.
				m_pointer = Vec2{ input.pointer_x, input.pointer_y };
				auto const dx = m_pointer.x - m_rest_point.x;
				auto const dy = m_pointer.y - m_rest_point.y;
				if (!m_pointer_known || dx * dx + dy * dy > TooltipRestToleranceDip * TooltipRestToleranceDip)
				{
					m_rest_point = m_pointer;
					m_rest_restart = true;
				}
				m_pointer_known = true;
				return;
			}
			case EInputKind::PointerButtonDown:
			case EInputKind::PointerWheel:
			case EInputKind::KeyDown:
			case EInputKind::FocusLost:
			{
				// Deliberate interaction hides the tooltip until the pointer moves to another target.
				m_dismissed = true;
				m_shown = false;
				return;
			}
			default:
			{
				return;
			}
		}
	}

	void TooltipState::Prune(TreeModel const& tree)
	{
		// The application tooltip cannot outlive the root that styles it.
		if (m_app_key != 0 && !tree.m_controls.contains(m_app_root_id))
			SetApp(0, 0, {});
	}

	void TooltipState::Update(TreeModel const& tree, InputState const& input, ViewportState const& viewport, TextShaper* shaper, double time_ms)
	{
		// Nothing is drawn unless this update decides the tooltip is shown.
		m_view = TooltipView{};

		// A new target starts a fresh rest period at the pointer's rest point and forgets any dismissal.
		auto const target = ResolveTarget(tree, input, *this);
		if (target != m_target)
		{
			m_target = target;
			m_rest_start_ms = time_ms;
			m_shown = false;
			m_dismissed = false;
			m_rest_restart = false;
		}
		else if (m_rest_restart)
		{
			// Movement on the same target restarts the delay only while the tooltip is not yet shown.
			if (!m_shown)
				m_rest_start_ms = time_ms;

			m_rest_restart = false;
		}

		// No tooltip shows without a target and a known pointer, after dismissal, or while the pointer is busy with a press,
		// a drag or an open ComboBox popup.
		auto const busy = input.m_pressed_id != 0 || input.m_captured_id != 0 || input.m_owned_buttons != 0 || input.m_open_combo_id != 0;
		if (target == TooltipTarget{} || !m_pointer_known || m_dismissed || busy)
		{
			m_shown = false;
			return;
		}
		if (!m_shown && time_ms - m_rest_start_ms < TooltipDelayMs)
			return;

		// The tooltip is shown and stays anchored at the point where the pointer came to rest.
		if (!m_shown)
			m_anchor = m_rest_point;

		m_shown = true;

		// Resolve the text and the root's tooltip font. An empty application text shows nothing.
		auto const& root = tree.m_controls.at(target.root_id).desc;
		auto text = target.control_id != 0 ? tree.m_controls.at(target.control_id).tooltip : m_app_text;
		if (text.empty())
			return;

		auto const font = ResolveControlFont(tree, root.tooltip_font_resource_id);
		auto const& visual = StyleOf(tree, root.tooltip_style_id).desc.visuals[static_cast<std::size_t>(EStateChannel::Normal)];

		// Measure the text. A missing font is reported by the renderer for the whole frame and its text is not drawn, so the
		// tooltip is skipped rather than shown as an empty box.
		auto text_size = Vec2{};
		if (shaper != nullptr)
		{
			try
			{
				text_size = Vec2{ shaper->LayoutWidth(font.family, font.size, text), shaper->LayoutHeight(font.family, font.size, text) };
			}
			catch (EngineException const& ex)
			{
				if (ex.Status() != EStatus::MissingAsset)
					throw;

				return;
			}
		}
		else
		{
			text_size = EstimateTextSize(text, font.size);
		}

		// The box holds the text plus padding and the style's border on every side.
		auto const border = std::max(0.0f, visual.border_thickness);
		auto const w = text_size.x + 2.0f * (TooltipPadX + border);
		auto const h = text_size.y + 2.0f * (TooltipPadY + border);

		// Place the box below and to the right of the anchor. When it would leave the viewport it flips to the other side
		// of the pointer, then it is clamped so as much of it as possible stays visible.
		auto const viewport_w = viewport.viewport_width_px * 96.0f / viewport.dpi;
		auto const viewport_h = viewport.viewport_height_px * 96.0f / viewport.dpi;
		auto x = m_anchor.x + TooltipOffsetX;
		auto y = m_anchor.y + TooltipOffsetY;
		if (x + w > viewport_w)
			x = m_anchor.x - w;
		if (y + h > viewport_h)
			y = m_anchor.y - h;

		x = std::clamp(x, 0.0f, std::max(0.0f, viewport_w - w));
		y = std::clamp(y, 0.0f, std::max(0.0f, viewport_h - h));

		m_view = TooltipView{
			.visible = true,
			.root_id = target.root_id,
			.text = std::move(text),
			.bounds = Rect{ x, y, w, h },
		};
	}
}
