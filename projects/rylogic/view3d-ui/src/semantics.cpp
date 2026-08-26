//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "semantics.h"
#include "text_unicode.h"

namespace pr::view3d::ui
{
	namespace
	{
		bool RectsIntersect(Rect const& a, Rect const& b)
		{
			return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
		}

		// The current text 'value' for one control: TextBox reports the exact string a renderer
		// would draw - its live pending edit with any in-progress IME composition spliced in - so
		// an assistive client observes the same text the user sees, falling back to the accepted
		// node text when no edit buffer exists; Text reports its accepted content; other closed
		// control types have no value text.
		std::string ValueTextFor(ControlNode const& node, InputState const& input)
		{
			switch (node.desc.type)
			{
				case EControlType::TextBox:
				{
					auto it = input.m_text_edits.find(node.desc.id);
					return it != input.m_text_edits.end() && it->second.initialized != 0 ? DisplayTextOf(it->second) : node.text;
				}
				case EControlType::Text:
				{
					return node.text;
				}
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::Button:
				case EControlType::Count:
				default:
				{
					return {};
				}
			}
		}

		// Fills in the caret/selection/composition fields of 'semantic' for an editable control.
		// Offsets are relative to the node's own value text, not the shared blob, so a consumer
		// can index the value text directly without first subtracting 'value_offset'.
		void ApplyTextRanges(SemanticNode& semantic, ControlNode const& node, InputState const& input)
		{
			if (node.desc.type != EControlType::TextBox)
				return;

			auto it = input.m_text_edits.find(node.desc.id);
			if (it == input.m_text_edits.end() || it->second.initialized == 0)
				return;

			// The ranges are computed against the same display string ValueTextFor returned. An IME
			// may report a caret or clause boundary inside a cluster it is still assembling, so
			// every offset is snapped to a grapheme boundary of that display string here: the
			// semantic contract promises boundaries a UI Automation text provider can rely on.
			auto const& edit = it->second;
			auto const display = DisplayTextOf(edit);
			auto const ranges = DisplayRangesOf(edit);
			auto const snap = [&display](std::uint32_t offset)
			{
				return ClampToGraphemeBoundary(display, std::min<std::uint32_t>(offset, static_cast<std::uint32_t>(display.size())));
			};

			auto flags = static_cast<std::uint32_t>(ESemanticTextFlag::HasCaret);
			semantic.caret = snap(ranges.caret);

			auto const selection_start = snap(std::min(ranges.selection_start, ranges.selection_end));
			auto const selection_end = snap(std::max(ranges.selection_start, ranges.selection_end));
			if (selection_start != selection_end)
			{
				semantic.selection_start = selection_start;
				semantic.selection_end = selection_end;
				flags |= static_cast<std::uint32_t>(ESemanticTextFlag::HasSelection);
			}
			if (edit.composition.active != 0)
			{
				// Snapping the two ends independently can only shrink the span, never invert it,
				// but the length is derived from the clamped ends so it can never run past them.
				auto const composition_start = snap(ranges.composition_start);
				auto const composition_end = std::max(composition_start, snap(ranges.composition_start + ranges.composition_length));
				semantic.composition_start = composition_start;
				semantic.composition_length = composition_end - composition_start;
				flags |= static_cast<std::uint32_t>(ESemanticTextFlag::Composing);
			}

			semantic.text_flags = flags;
		}

		std::uint32_t SupportedActionsFor(ControlNode const& node)
		{
			auto actions = static_cast<std::uint32_t>(ESemanticAction::None);
			if (node.desc.enabled != 0)
			{
				switch (node.desc.type)
				{
					// An editable control also advertises SetSelection, which is what lets a text
					// pattern client move the caret or select a range without synthesising input.
					case EControlType::TextBox:
					{
						actions |= static_cast<std::uint32_t>(ESemanticAction::SetValue);
						actions |= static_cast<std::uint32_t>(ESemanticAction::SetSelection);
						break;
					}
					case EControlType::Button: actions |= static_cast<std::uint32_t>(ESemanticAction::Invoke); break;
					case EControlType::Root:
					case EControlType::Panel:
					case EControlType::Text:
					case EControlType::Count:
					default: break;
				}
				if (node.desc.focusable != 0)
					actions |= static_cast<std::uint32_t>(ESemanticAction::Focus);
			}
			return actions;
		}

		std::uint32_t StateFlagsFor(ControlNode const& node, InputState const& input, Rect const& bounds, Rect const& root_bounds)
		{
			auto flags = static_cast<std::uint32_t>(ESemanticState::None);
			if (node.desc.enabled != 0)
				flags |= static_cast<std::uint32_t>(ESemanticState::Enabled);
			if (node.desc.visible != 0)
				flags |= static_cast<std::uint32_t>(ESemanticState::Visible);
			if (node.desc.id == input.m_focus_id)
				flags |= static_cast<std::uint32_t>(ESemanticState::Focused);
			if (node.desc.focusable != 0)
				flags |= static_cast<std::uint32_t>(ESemanticState::Focusable);
			if (node.desc.selected != 0)
				flags |= static_cast<std::uint32_t>(ESemanticState::Selected);
			if (node.desc.validation_state == EValidationState::Invalid)
				flags |= static_cast<std::uint32_t>(ESemanticState::Invalid);
			if (!RectsIntersect(bounds, root_bounds))
				flags |= static_cast<std::uint32_t>(ESemanticState::Offscreen);
			return flags;
		}

		// Append 'text' to the shared blob and return its (offset, length) as a pair.
		std::pair<std::uint32_t, std::uint32_t> AppendBlob(std::string& blob, std::string_view text)
		{
			auto offset = static_cast<std::uint32_t>(blob.size());
			blob.append(text);
			return { offset, static_cast<std::uint32_t>(text.size()) };
		}

		void Walk(TreeModel const& tree, ControlId id, Rect const& root_bounds, std::unordered_map<ControlId, Rect> const& layout, InputState const& input_state, std::uint64_t accepted_revision, std::uint64_t& sequence, SemanticSnapshot& out)
		{
			auto const& node = tree.m_controls.at(id);
			auto const& bounds = layout.at(id);

			SemanticNode semantic{};
			semantic.header.size = sizeof(SemanticNode);
			semantic.header.version = VIEW3D_UI_STRUCT_VERSION;
			semantic.id = node.desc.id;
			semantic.parent_id = node.desc.parent_id;
			semantic.role = node.desc.type;
			std::tie(semantic.name_offset, semantic.name_length) = AppendBlob(out.m_text_blob, node.name);
			std::tie(semantic.desc_offset, semantic.desc_length) = AppendBlob(out.m_text_blob, node.description);
			auto const value_text = ValueTextFor(node, input_state);
			std::tie(semantic.value_offset, semantic.value_length) = AppendBlob(out.m_text_blob, value_text);
			semantic.value_grapheme_count = static_cast<std::uint32_t>(GraphemeCount(value_text));
			ApplyTextRanges(semantic, node, input_state);
			semantic.state_flags = StateFlagsFor(node, input_state, bounds, root_bounds);
			semantic.supported_actions = SupportedActionsFor(node);
			semantic.bounds = bounds;
			semantic.accepted_revision = accepted_revision;
			semantic.semantic_sequence = sequence++;
			out.m_nodes.push_back(semantic);

			for (auto child_id : node.children)
				Walk(tree, child_id, root_bounds, layout, input_state, accepted_revision, sequence, out);
		}
	}

	SemanticSnapshot BuildSemanticSnapshot(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, std::unordered_map<ControlId, RootPlacement> const& placements, InputState const& input, std::uint64_t accepted_revision)
	{
		SemanticSnapshot out;
		out.m_nodes.reserve(tree.m_controls.size());

		std::uint64_t sequence = 1;
		for (auto root_id : tree.m_roots)
		{
			// A culled world root has no on-screen extent at all, so an empty rect (which nothing
			// can intersect) marks its whole subtree Offscreen. A visible root uses its own rect,
			// which is what every screen root has always used.
			auto placement = placements.find(root_id);
			auto culled = placement != placements.end() && placement->second.visible == 0;
			auto root_bounds = culled ? Rect{ 0.0f, 0.0f, 0.0f, 0.0f } : layout.at(root_id);
			Walk(tree, root_id, root_bounds, layout, input, accepted_revision, sequence, out);
		}

		return out;
	}
}
