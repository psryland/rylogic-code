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
		EVisualPrimitive PrimitiveFor(EControlType type, StyleVisual const& visual)
		{
			switch (type)
			{
				case EControlType::Text: return EVisualPrimitive::TextPresenter;
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::TextBox:
				case EControlType::Button:
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

		StyleRecord const& StyleFor(TreeModel const& tree, StyleId style_id)
		{
			if (style_id == 0)
				return TreeModel::DefaultStyle();

			auto it = tree.m_styles.find(style_id);
			return it != tree.m_styles.end() ? it->second : TreeModel::DefaultStyle();
		}

		void Walk(TreeModel const& tree, ControlId id, std::unordered_map<ControlId, Rect> const& layout, StyleResolver& styles, InputState const& input_state, double time_ms, float scale, DrawPacket& out)
		{
			auto const& node = tree.m_controls.at(id);
			if (node.desc.visible == 0)
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

			// Root/Panel/TextBox/Button paint a box first; a bare Text control has no box of its
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
						text = DisplayTextOf(edit_it->second);
						ranges = DisplayRangesOf(edit_it->second);
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
				Walk(tree, child_id, layout, styles, input_state, time_ms, scale, out);
		}
	}

	DrawPacket BuildDrawPacket(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, std::unordered_map<ControlId, RootPlacement> const& placements, StyleResolver& styles, InputState const& input_state, std::uint64_t accepted_revision, std::uint64_t visual_sequence, double time_ms, float viewport_dpi)
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
			Walk(tree, root_id, layout, styles, input_state, time_ms, placement.scale, out);

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

		return out;
	}
}
