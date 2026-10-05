//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "draw_packet_builder.h"
#include "text_layout.h"

namespace pr::view3d::ui
{
	namespace
	{
		StyleRecord const& StyleFor(TreeModel const& tree, StyleId style_id);

		EVisualPrimitive PrimitiveFor(EControlType type, StyleVisual const& visual)
		{
			switch (type)
			{
				case EControlType::Text: return EVisualPrimitive::TextPresenter;
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::TextBox:
				case EControlType::Button:
				case EControlType::ProgressBar:
				case EControlType::Slider:
				case EControlType::ComboBox:
				{
					return visual.corner_radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
				}

				case EControlType::Count:
				default:
				{
					throw EngineException(EStatus::InvalidArgument, "unknown control type");
				}
			}
		}

		// Paint the accepted slider position as a filled track plus a compact thumb. Both use the
		// existing quad primitives and style colours, so custom templates remain lookless.
		void AppendSlider(ControlNode const& node, Rect bounds, StyleVisual const& visual, float scale, DrawPacket& out)
		{
			// Derive indicator and thumb geometry only from the accepted descriptor and current DPI scale.
			auto const range = node.desc.maximum - node.desc.minimum;
			auto const fraction = (node.desc.value - node.desc.minimum) / range;
			auto const inset = std::clamp(visual.border_thickness * scale, 0.0f, std::min(bounds.w, bounds.h) * 0.5f);
			auto const width = std::max(0.0f, bounds.w - 2.0f * inset);
			auto const height = std::max(0.0f, bounds.h - 2.0f * inset);
			if (width <= 0.0f || height <= 0.0f)
				return;

			// Foreground fills the accepted portion of the track.
			if (fraction > 0.0f)
			{
				auto indicator = DrawItem{};
				indicator.control_id = node.desc.id;
				indicator.bounds = Rect{ bounds.x + inset, bounds.y + inset, width * fraction, height };
				indicator.fill = visual.foreground;
				indicator.opacity = visual.opacity;
				indicator.primitive = EVisualPrimitive::SolidBox;
				out.items.push_back(std::move(indicator));
			}

			// The thumb remains visible at both endpoints and scales with the root's apparent DIP scale.
			auto const thumb_width = std::min(width, std::max(4.0f * scale, height * 0.5f));
			auto thumb = DrawItem{};
			thumb.control_id = node.desc.id;
			thumb.bounds = Rect{ bounds.x + inset + (width - thumb_width) * fraction, bounds.y + inset, thumb_width, height };
			thumb.fill = visual.foreground;
			thumb.opacity = visual.opacity;
			thumb.corner_radius = std::min(thumb_width, height) * 0.5f;
			thumb.primitive = EVisualPrimitive::RoundedBox;
			out.items.push_back(std::move(thumb));
		}


		// Paint the closed ComboBox face using the accepted selected item and a fixed drop-down glyph.
		void AppendComboBoxFace(TreeModel const& tree, ControlNode const& node, Rect bounds, float scale, DrawPacket& out)
		{
			// The closed box shows only the caller-authoritative selected item plus a glyph indicator.
			auto const& style_record = StyleFor(tree, node.desc.style_id);
			auto const normal = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Normal)];
			auto const font = ResolveControlFont(tree, node.desc.font_resource_id);
			auto const placement = TextPlacementFor(EControlType::TextBox);
			auto const selected_text = node.desc.selected_index >= 0 && static_cast<std::uint32_t>(node.desc.selected_index) < node.combo_items.size() ? node.combo_items[static_cast<std::size_t>(node.desc.selected_index)] : std::string{};
			if (!selected_text.empty())
			{
				DrawItem text_item{};
				text_item.control_id = node.desc.id;
				text_item.primitive = EVisualPrimitive::TextPresenter;
				text_item.bounds = Rect{ bounds.x, bounds.y, std::max(0.0f, bounds.w - 24.0f * scale), bounds.h };
				text_item.fill = font.colour;
				text_item.opacity = normal.opacity;
				text_item.text = selected_text;
				text_item.font_family = font.family;
				text_item.font_size = font.size * scale;
				text_item.text_align = placement.align;
				text_item.text_inset_dip = placement.inset_dip * scale;
				out.items.push_back(std::move(text_item));
			}

			DrawItem glyph{};
			glyph.control_id = node.desc.id;
			glyph.primitive = EVisualPrimitive::TextPresenter;
			glyph.bounds = Rect{ bounds.x + std::max(0.0f, bounds.w - 24.0f * scale), bounds.y, 24.0f * scale, bounds.h };
			glyph.fill = font.colour;
			glyph.opacity = normal.opacity;
			glyph.text = "\xE2\x96\xBE";
			glyph.font_family = font.family;
			glyph.font_size = font.size * scale;
			glyph.text_align = ETextAlign::Center;
			glyph.text_inset_dip = 0.0f;
			out.items.push_back(std::move(glyph));
		}

		// Paint the transient ComboBox popup as a topmost draw group.
		void AppendComboBoxPopup(TreeModel const& tree, ControlNode const& node, std::unordered_map<ControlId, Rect> const& layout, InputState const& input_state, ViewportState const& viewport, float scale, DrawPacket& out)
		{
			// A closed or empty popup contributes no transient visuals.
			if (input_state.m_open_combo_id != node.desc.id)
				return;

			auto const popup = ComboBoxPopupRect(tree, layout, viewport, node.desc.id);
			if (popup.w <= 0.0f || popup.h <= 0.0f)
				return;

			auto const& style_record = StyleFor(tree, node.desc.style_id);
			auto const normal = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Normal)];
			auto const hover = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Hover)];
			auto const selected = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Selected)];
			auto const font = ResolveControlFont(tree, node.desc.font_resource_id);
			auto const placement = TextPlacementFor(EControlType::TextBox);

			DrawItem background{};
			background.control_id = node.desc.id;
			background.primitive = normal.corner_radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
			background.bounds = popup;
			background.fill = normal.fill;
			background.border_colour = normal.border_colour;
			background.border_thickness = normal.border_thickness * scale;
			background.corner_radius = normal.corner_radius * scale;
			background.opacity = normal.opacity;
			out.items.push_back(std::move(background));

			auto const row_h = popup.h / static_cast<float>(std::max<std::uint32_t>(1U, std::min(node.desc.combo_item_count, node.desc.max_visible_items)));
			auto const visible_count = std::min(node.desc.combo_item_count - input_state.m_combo_scroll_offset, node.desc.max_visible_items);
			for (auto row = std::uint32_t{}; row != visible_count; ++row)
			{
				auto const item_index = input_state.m_combo_scroll_offset + row;
				auto const row_bounds = Rect{ popup.x, popup.y + row_h * static_cast<float>(row), popup.w, row_h };
				auto const& row_visual = static_cast<std::int32_t>(item_index) == input_state.m_combo_highlight_index ? hover : static_cast<std::int32_t>(item_index) == node.desc.selected_index ? selected : normal;
				DrawItem row_box{};
				row_box.control_id = node.desc.id;
				row_box.primitive = EVisualPrimitive::SolidBox;
				row_box.bounds = row_bounds;
				row_box.fill = row_visual.fill;
				row_box.opacity = row_visual.opacity;
				out.items.push_back(std::move(row_box));

				DrawItem row_text{};
				row_text.control_id = node.desc.id;
				row_text.primitive = EVisualPrimitive::TextPresenter;
				row_text.bounds = row_bounds;
				row_text.fill = font.colour;
				row_text.opacity = row_visual.opacity;
				row_text.text = item_index < node.combo_items.size() ? node.combo_items[item_index] : std::string{};
				row_text.font_family = font.family;
				row_text.font_size = font.size * scale;
				row_text.text_align = placement.align;
				row_text.text_inset_dip = placement.inset_dip * scale;
				out.items.push_back(std::move(row_text));
			}
		}

		StyleRecord const& StyleFor(TreeModel const& tree, StyleId style_id)
		{
			if (style_id == 0)
				return TreeModel::DefaultStyle();

			auto it = tree.m_styles.find(style_id);
			return it != tree.m_styles.end() ? it->second : TreeModel::DefaultStyle();
		}

		// Paint completion or activity within the track's border, without changing accepted state.
		void AppendProgress(ControlNode const& node, Rect bounds, StyleVisual const& visual, double time_ms, float scale, DrawPacket& out)
		{
			// A quarter-width indicator travels smoothly back and forth once every 2400 ms.
			// Host time, rather than update count or transactions, drives the animation.
			auto const inset = std::clamp(visual.border_thickness * scale, 0.0f, std::min(bounds.w, bounds.h) * 0.5f);
			auto const width = bounds.w - 2.0f * inset;
			auto const height = bounds.h - 2.0f * inset;
			auto const fraction = node.desc.is_indeterminate != 0 ? 0.25f : node.desc.value;
			auto const offset = node.desc.is_indeterminate != 0 ? static_cast<float>(0.5 - 0.5 * std::cos(std::fmod(time_ms, 2400.0) * (6.283185307179586 / 2400.0))) * (1.0f - fraction) : 0.0f;
			if (width <= 0.0f || height <= 0.0f || fraction == 0.0f)
				return;

			// Reuse the existing quad renderer; each indicator is entirely inside its track.
			auto item = DrawItem{};
			item.control_id = node.desc.id;
			item.bounds = Rect{ bounds.x + inset + width * offset, bounds.y + inset, width * fraction, height };
			item.fill = visual.foreground;
			item.opacity = visual.opacity;
			item.corner_radius = std::clamp(visual.corner_radius * scale - inset, 0.0f, std::min(item.bounds.w, item.bounds.h) * 0.5f);
			item.primitive = item.corner_radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
			out.items.push_back(std::move(item));
		}

		void Walk(TreeModel const& tree, ControlId id, std::unordered_map<ControlId, Rect> const& layout, ViewportState const& viewport, StyleResolver& styles, InputState const& input_state, double time_ms, float scale, DrawPacket& out)
		{
			auto const& node = tree.m_controls.at(id);
			if (!IsVisible(node.desc.visibility))
			{
				// An invisible control hides its whole subtree from the draw packet, matching
				// hit-test/tab-order; record it so a later Update() where it becomes visible again
				// fires exactly one Visibility transition (style.h::StyleResolver::MarkInvisible).
				styles.MarkInvisible(id);
				return;
			}

			auto const& style_record = StyleFor(tree, node.desc.style_id);
			auto visual = styles.Resolve(node, style_record, input_state.m_hover_id, input_state.m_pressed_id, input_state.m_focus_id, time_ms);
			auto bounds = layout.at(id);
			auto primitive = PrimitiveFor(node.desc.type, visual);

			// Root/Panel/TextBox/Button/ProgressBar paint a box first; a bare Text control has no box of its
			// own (it is itself the TextPresenter item emitted below).
			if (primitive != EVisualPrimitive::TextPresenter)
			{
				DrawItem box{};
				box.control_id = node.desc.id;
				box.primitive = primitive;
				box.bounds = bounds;
				box.fill = visual.fill;
				box.border_colour = visual.border_colour;

				// Layout rects are already in screen space, but style-derived lengths are authored
				// in the root's local DIP space, so they take the root's apparent scale here.
				box.border_thickness = visual.border_thickness * scale;
				box.corner_radius = visual.corner_radius * scale;
				box.opacity = visual.opacity;
				out.items.push_back(std::move(box));
			}

			// Progress is a read-only visual; labels remain separate retained Text controls.
			switch (node.desc.type)
			{
				case EControlType::ProgressBar: { AppendProgress(node, bounds, visual, time_ms, scale, out); break; }
				case EControlType::Slider: { AppendSlider(node, bounds, visual, scale, out); break; }
				case EControlType::ComboBox: { AppendComboBoxFace(tree, node, bounds, scale, out); break; }
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::Text:
				case EControlType::TextBox:
				case EControlType::Button:
				{
					break;
				}
				default: { throw EngineException(EStatus::UnknownType, "unknown control type"); }
			}

			// A label/text item follows for the three control types that can display text. A
			// focused TextBox's live pending edit (if any) always takes priority over its accepted
			// text, so the draw packet paints exactly what the user is currently typing rather than
			// a stale accepted value, with any in-progress IME composition spliced in.
			if (node.desc.type == EControlType::Text || node.desc.type == EControlType::Button || node.desc.type == EControlType::TextBox)
			{
				auto text = node.text;
				auto ranges = TextEditRanges{ .caret = 0, .selection_start = 0, .selection_end = 0, .composition_start = 0, .composition_length = 0 };
				auto has_edit_state = false;
				if (node.desc.type == EControlType::TextBox)
				{
					auto edit_it = input_state.m_text_edits.find(id);
					if (edit_it != input_state.m_text_edits.end() && edit_it->second.initialized != 0)
					{
						text = DisplayTextOf(node.desc, edit_it->second);
						ranges = DisplayRangesOf(node.desc, edit_it->second);
						has_edit_state = true;
					}
				}

				// A focused TextBox always emits its item even when empty, because the caret still
				// has to be drawn; every other case with nothing to show is skipped entirely.
				auto const focused = input_state.m_focus_id == id;
				if (!text.empty() || (focused && has_edit_state))
				{
					auto font = ResolveControlFont(tree, node.desc.font_resource_id);
					auto placement = TextPlacementFor(node.desc.type);

					DrawItem text_item{};
					text_item.control_id = node.desc.id;
					text_item.primitive = EVisualPrimitive::TextPresenter;
					text_item.bounds = bounds;
					text_item.fill = font.colour;
					text_item.border_colour = visual.border_colour;
					text_item.border_thickness = 0.0f;
					text_item.corner_radius = 0.0f;
					text_item.opacity = visual.opacity;
					text_item.text = std::move(text);
					text_item.font_family = std::move(font.family);
					text_item.font_size = font.size * scale;
					text_item.text_align = placement.align;
					text_item.text_inset_dip = placement.inset_dip * scale;

					// Selection/composition/caret decorations are only meaningful for the control
					// the user is actually editing, so an unfocused TextBox reports none of them.
					if (focused && has_edit_state)
					{
						text_item.selection_start = ranges.selection_start;
						text_item.selection_end = ranges.selection_end;
						text_item.composition_start = ranges.composition_start;
						text_item.composition_length = ranges.composition_length;
						text_item.caret_offset = ranges.caret;
						text_item.caret_visible = 1;
					}
					out.items.push_back(std::move(text_item));
				}
			}

			for (auto child_id : node.children)
				Walk(tree, child_id, layout, viewport, styles, input_state, time_ms, scale, out);
		}
	}

	DrawPacket BuildDrawPacket(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, std::unordered_map<ControlId, RootPlacement> const& placements, StyleResolver& styles, InputState const& input_state, std::uint64_t accepted_revision, std::uint64_t visual_sequence, double time_ms, float viewport_dpi, ViewportState const& viewport)
	{
		DrawPacket out;
		out.accepted_revision = accepted_revision;
		out.visual_sequence = visual_sequence;
		out.viewport_dpi = viewport_dpi;
		out.items.reserve(tree.m_controls.size());
		out.groups.reserve(tree.m_roots.size());

		for (auto root_id : tree.m_roots)
		{
			auto placement_it = placements.find(root_id);
			if (placement_it == placements.end())
				throw EngineException(EStatus::InternalError, std::format("BuildDrawPacket: no placement was computed for root {}", root_id));

			// A culled root contributes nothing to draw; it is still reported semantically, so
			// dropping it here is purely a visual decision and never changes accepted state.
			auto const& placement = placement_it->second;
			if (placement.visible == 0)
				continue;

			auto first_item = static_cast<std::uint32_t>(out.items.size());
			Walk(tree, root_id, layout, viewport, styles, input_state, time_ms, placement.scale, out);

			// An entirely-invisible subtree emits no items; skipping the empty group keeps the
			// renderer's per-pass work proportional to what is actually drawn.
			auto item_count = static_cast<std::uint32_t>(out.items.size()) - first_item;
			if (item_count == 0)
				continue;

			out.groups.push_back(DrawGroup{
				.root_id = root_id,
				.policy = placement.policy,
				.first_item = first_item,
				.item_count = item_count,
				.clip_depth = placement.clip_depth,
				.view_depth = placement.view_depth,
				.occlusion_min_opacity = placement.occlusion_min_opacity,
				.occlusion_fade_depth = placement.occlusion_fade_depth,
				.occlusion_depth_bias = placement.occlusion_depth_bias,
			});
		}

		if (input_state.m_open_combo_id != 0)
		{
			// Emit the open popup as the final draw group so it renders above every normal root.
			auto combo_it = tree.m_controls.find(input_state.m_open_combo_id);
			if (combo_it != tree.m_controls.end() && combo_it->second.desc.type == EControlType::ComboBox)
			{
				auto root_id = input_state.m_open_combo_id;
				for (;;)
				{
					auto const& walk = tree.m_controls.at(root_id);
					if (walk.desc.parent_id == 0)
						break;

					root_id = walk.desc.parent_id;
				}

				auto placement_it = placements.find(root_id);
				if (placement_it != placements.end() && placement_it->second.visible != 0)
				{
					auto first_item = static_cast<std::uint32_t>(out.items.size());
					auto const& placement = placement_it->second;
					AppendComboBoxPopup(tree, combo_it->second, layout, input_state, viewport, placement.scale, out);
					auto item_count = static_cast<std::uint32_t>(out.items.size()) - first_item;
					if (item_count != 0)
					{
						out.groups.push_back(DrawGroup{
							.root_id = root_id,
							.policy = placement.policy,
							.first_item = first_item,
							.item_count = item_count,
							.clip_depth = placement.clip_depth,
							.view_depth = placement.view_depth,
							.occlusion_min_opacity = placement.occlusion_min_opacity,
							.occlusion_fade_depth = placement.occlusion_fade_depth,
							.occlusion_depth_bias = placement.occlusion_depth_bias,
						});
					}
				}
			}
		}

		return out;
	}
}
