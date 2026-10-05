//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "input.h"
#include "text_layout.h"
#include "text_shaper.h"
#include "text_unicode.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	namespace
	{
		bool RectContains(Rect const& r, Vec2 pt)
		{
			return pt.x >= r.x && pt.x < r.x + r.w && pt.y >= r.y && pt.y < r.y + r.h;
		}

		// Whether a control type can itself be returned as a hit-test result. Root/Panel/Text are
		// pure layout/decoration and must be hit-test transparent: an autosized Root commonly covers
		// the whole viewport, and a Panel/Text commonly covers area with no interactive purpose, so
		// treating their own bounds as a hit would swallow every pointer event over the viewport and
		// starve the host application's own scene/camera input of clicks that land outside any real
		// control (section 7.3). Button, TextBox, and Slider are the closed interactive controls.
		bool IsHitTestable(EControlType type)
		{
			switch (type)
			{
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::Text:
				case EControlType::ProgressBar:
				{
					return false;
				}
				case EControlType::ComboBox:
				case EControlType::TextBox:
				case EControlType::Button:
				case EControlType::Slider:
				{
					return true;
				}
				case EControlType::Count:
				default:
				{
					throw EngineException(EStatus::InvalidArgument, "unknown control type");
				}
			}
		}

		// Depth-first hit test that stops descending into an invisible control's subtree and
		// prefers the deepest/last-drawn (topmost) match: later siblings and children are tested
		// before their earlier siblings/ancestor, matching typical top-to-bottom paint order. A
		// control whose own type is not hit-testable (see IsHitTestable) is transparent to the test
		// even when its bounds contain the point: the search still descends into its children,
		// but the container itself is never returned as the hit, so clicking empty layout/decoration
		// area falls through to a miss (id 0) rather than being absorbed by an enclosing container.
		ControlId HitTestRecurse(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, ControlId id, Vec2 pt)
		{
			auto const& node = tree.m_controls.at(id);
			if (!IsVisible(node.desc.visibility))
				return 0;

			for (auto it = node.children.rbegin(); it != node.children.rend(); ++it)
			{
				auto hit = HitTestRecurse(tree, layout, *it, pt);
				if (hit != 0)
					return hit;
			}

			auto rect_it = layout.find(id);
			return rect_it != layout.end() && RectContains(rect_it->second, pt) && node.desc.enabled != 0 && IsHitTestable(node.desc.type) ? id : 0;
		}

		void ComputeTabOrderRecurse(TreeModel const& tree, ControlId id, std::vector<ControlId>& order)
		{
			auto const& node = tree.m_controls.at(id);
			if (!IsVisible(node.desc.visibility))
				return; // an invisible control and its whole subtree are excluded from Tab order

			if (node.desc.focusable != 0 && node.desc.enabled != 0)
				order.push_back(id);

			for (auto child_id : node.children)
				ComputeTabOrderRecurse(tree, child_id, order);
		}

		// Nearest focusable ancestor-or-self of 'id', or 0 if none of the chain is focusable. Used
		// so clicking a non-focusable decoration inside a focusable container still moves focus.
		ControlId NearestFocusable(TreeModel const& tree, ControlId id)
		{
			for (auto walk_id = id; walk_id != 0;)
			{
				auto const& node = tree.m_controls.at(walk_id);
				if (node.desc.focusable != 0 && node.desc.enabled != 0)
					return walk_id;

				walk_id = node.desc.parent_id;
			}
			return 0;
		}

		// Nearest ancestor-or-self of 'id' whose control type is exactly 'type', or 0 if none of
		// the chain matches. Used so a pointer press/release on a non-focusable visual/content
		// descendant (e.g. a Button's label Text control, or a TextBox's caret/placeholder part)
		// still activates the owning Button/TextBox rather than being silently ignored, matching
		// how a real UI toolkit treats a control's rendered content as part of the control itself.
		ControlId NearestOfType(TreeModel const& tree, ControlId id, EControlType type)
		{
			for (auto walk_id = id; walk_id != 0;)
			{
				auto const& node = tree.m_controls.at(walk_id);
				if (node.desc.type == type)
					return walk_id;

				walk_id = node.desc.parent_id;
			}
			return 0;
		}

		void PushOrThrow(EventQueue& events, ControlId control_id, EEventKind kind, std::uint64_t accepted_revision, std::uint32_t edit_generation, std::string payload)
		{
			if (!events.Push(control_id, kind, accepted_revision, edit_generation, std::move(payload)))
				throw EngineException(EStatus::QueueOverflow, std::format("event queue overflow while enqueueing event kind {} for control {}", static_cast<int>(kind), control_id));
		}

		// Snap a finite candidate to the descriptor's step lattice anchored at minimum, while
		// preserving exact endpoints so both bounds remain reachable when the range is not an
		// integral number of steps.
		double SnapSliderValue(ControlDesc const& desc, double candidate)
		{
			// Preserve exact endpoints before rounding interior values to the step lattice.
			if (candidate <= desc.minimum)
				return desc.minimum;
			if (candidate >= desc.maximum)
				return desc.maximum;

			auto const step_count = std::round((candidate - desc.minimum) / desc.step);
			return std::clamp(static_cast<double>(desc.minimum) + step_count * desc.step, static_cast<double>(desc.minimum), static_cast<double>(desc.maximum));
		}

		// Emit a typed numeric proposal without changing the accepted descriptor value.
		void ProposeSliderValue(EventQueue& events, ControlNode const& node, double candidate, std::uint64_t accepted_revision)
		{
			// Carry the proposal as a typed number while leaving the retained descriptor untouched.
			auto const value = SnapSliderValue(node.desc, candidate);
			if (!events.Push(node.desc.id, EEventKind::ValueChangeProposed, accepted_revision, 0, {}, 1, value))
				throw EngineException(EStatus::QueueOverflow, std::format("event queue overflow while enqueueing slider proposal for control {}", node.desc.id));
		}

		// Convert a pointer position into the slider's inclusive scalar range.
		void ProposeSliderFromPointer(std::unordered_map<ControlId, Rect> const& layout, EventQueue& events, ControlNode const& node, Vec2 point, std::uint64_t accepted_revision)
		{
			// Map the pointer across the whole control width and let the shared proposal path snap it.
			auto const bounds = layout.at(node.desc.id);
			auto const fraction = bounds.w > 0.0f ? std::clamp((point.x - bounds.x) / bounds.w, 0.0f, 1.0f) : 0.0f;
			ProposeSliderValue(events, node, node.desc.minimum + fraction * (node.desc.maximum - node.desc.minimum), accepted_revision);
		}

		TextEditState& GetOrInitTextEdit(InputState& state, ControlNode const& node)
		{
			auto& edit = state.m_text_edits[node.desc.id];
			if (edit.initialized == 0)
			{
				edit.pending_text = node.text;
				edit.last_accepted_text = node.text;
				edit.caret = static_cast<std::uint32_t>(edit.pending_text.size());
				edit.selection_start = edit.caret;
				edit.edit_generation = 0;
				edit.initialized = 1;
			}
			return edit;
		}

		// Seed the live edit state of a newly focused TextBox so its caret exists from the moment
		// focus arrives rather than only after the first keystroke. Every consumer of the caret -
		// the draw packet, the semantic snapshot and the ABI's caret rectangle - reads this state,
		// so without it a focused but untouched field would report and draw no caret at all.
		void SeedFocusedTextEdit(TreeModel const& tree, InputState& state, ControlId id)
		{
			auto it = tree.m_controls.find(id);
			if (it == tree.m_controls.end() || it->second.desc.type != EControlType::TextBox)
				return;

			GetOrInitTextEdit(state, it->second);
		}

		std::wstring Utf8ToWide(std::string_view s)
		{
			std::wstring w;
			if (!Utf8ToUtf16(s, w))
				throw EngineException(EStatus::InvalidArgument, "text is not valid UTF-8");

			return w;
		}

		// Best-effort OS clipboard access for Ctrl+C/X/V (section 7.5). Clipboard unavailability
		// (e.g. another process holding it open) is transient host state, not a UI validation
		// failure, so these helpers degrade to a no-op rather than throwing.
		void ClipboardSetText(std::string_view utf8)
		{
			if (!OpenClipboard(nullptr))
				return;

			auto wide = Utf8ToWide(utf8);
			EmptyClipboard();
			auto bytes = (wide.size() + 1) * sizeof(wchar_t);
			auto mem = GlobalAlloc(GMEM_MOVEABLE, bytes);
			if (mem != nullptr)
			{
				auto* dst = static_cast<wchar_t*>(GlobalLock(mem));
				std::memcpy(dst, wide.c_str(), bytes);
				GlobalUnlock(mem);
				SetClipboardData(CF_UNICODETEXT, mem);
			}
			CloseClipboard();
		}

		// Reads CF_UNICODETEXT as UTF-8. Text the OS reports that is not well-formed UTF-16 is
		// discarded rather than converted with substitutions, so a corrupt clipboard can never
		// introduce replacement characters into an edit buffer.
		std::string ClipboardGetText()
		{
			if (!IsClipboardFormatAvailable(CF_UNICODETEXT) || !OpenClipboard(nullptr))
				return {};

			std::string result;
			auto mem = GetClipboardData(CF_UNICODETEXT);
			if (mem != nullptr)
			{
				auto* src = static_cast<wchar_t const*>(GlobalLock(mem));
				if (src != nullptr)
				{
					if (!Utf16ToUtf8(std::wstring_view(src), result))
						result.clear();

					GlobalUnlock(mem);
				}
			}
			CloseClipboard();
			return result;
		}

		// Truncates 'insert' to the longest prefix ending on one of its own grapheme boundaries for
		// which 'prefix' + prefix-of-insert + 'suffix' still holds at most 'desc.max_text_length'
		// clusters. Counting the whole concatenation rather than the insertion alone is what makes
		// a cluster that merges across the prefix or suffix boundary - a combining mark typed after
		// an existing letter, say - count once instead of twice. The count is monotonic in the cut
		// index, so the longest admissible cut is found by binary search over the boundary list.
		std::string_view TruncateInsertion(ControlDesc const& desc, std::string_view prefix, std::string_view suffix, std::string_view insert)
		{
			if (desc.max_text_length == 0)
				return insert;

			auto measure = [&](std::string_view candidate)
			{
				auto combined = std::string(prefix);
				combined.append(candidate);
				combined.append(suffix);
				return GraphemeCount(combined);
			};

			if (measure(insert) <= desc.max_text_length)
				return insert;

			// lo is always admissible (the empty insertion cannot make an over-long field worse)
			// and hi is always inadmissible, so the loop converges on the boundary between them.
			auto const boundaries = GraphemeBoundaries(insert);
			auto lo = std::size_t(0);
			auto hi = boundaries.size() - 1;
			while (hi - lo > 1)
			{
				auto const mid = lo + (hi - lo) / 2;
				if (measure(insert.substr(0, boundaries[mid])) <= desc.max_text_length)
					lo = mid;
				else
					hi = mid;
			}

			return insert.substr(0, boundaries[lo]);
		}

		// Replace the current selection (if any) with 'insert', clamped to the control's
		// max_text_length in grapheme clusters, and return whether the pending text actually
		// changed. The caret and selection collapse to the end of what was inserted.
		bool ReplaceSelection(ControlDesc const& desc, TextEditState& edit, std::string_view insert)
		{
			auto lo = std::min(edit.caret, edit.selection_start);
			auto hi = std::max(edit.caret, edit.selection_start);
			auto clamped = TruncateInsertion(desc, std::string_view(edit.pending_text).substr(0, lo), std::string_view(edit.pending_text).substr(hi), insert);
			if (lo == hi && clamped.empty())
				return false;

			edit.pending_text.replace(lo, hi - lo, clamped.data(), clamped.size());
			edit.caret = static_cast<std::uint32_t>(lo + clamped.size());
			edit.selection_start = edit.caret;
			return true;
		}

		// Emit the TextChangeProposed event for a locally-edited proposal. Every proposal in flight
		// gets its own nonzero generation, so a reconciling application can tell distinct proposals
		// apart even if two happen to produce identical text; the queue's coalescing keeps only the
		// latest one queued.
		void ProposeTextChange(EventQueue& events, ControlId control_id, TextEditState& edit, std::uint64_t accepted_revision)
		{
			++edit.edit_generation;
			PushOrThrow(events, control_id, EEventKind::TextChangeProposed, accepted_revision, edit.edit_generation, edit.pending_text);
		}

		// The focused control, when it is an enabled TextBox ready to be edited, or null. A record
		// that needs an edit target and finds none is left unconsumed rather than failing, which is
		// what lets the host application still see keystrokes the UI has no use for.
		ControlNode const* EditableFocusTarget(TreeModel const& tree, InputState const& state)
		{
			if (state.m_focus_id == 0)
				return nullptr;

			auto it = tree.m_controls.find(state.m_focus_id);
			if (it == tree.m_controls.end())
				return nullptr;

			auto const& node = it->second;
			return node.desc.type == EControlType::TextBox && node.desc.enabled != 0 ? &node : nullptr;
		}

		// Validates a borrowed text payload for a record kind that requires one, rejecting invalid
		// UTF-8 and offsets that are out of range or inside a code point. Composition offsets must
		// be usable as caret/selection positions directly, so an offset mid-sequence is a malformed
		// payload rather than something to silently round.
		void ValidateTextPayload(InputTextRecord const* payload, bool check_offsets)
		{
			if (payload == nullptr)
				throw EngineException(EStatus::InvalidArgument, "input record kind requires a text payload");

			if (!Utf8Validate(payload->text))
				throw EngineException(EStatus::InvalidArgument, "input text payload is not valid UTF-8");

			if (!check_offsets)
				return;

			auto const size = static_cast<std::uint32_t>(payload->text.size());
			auto on_boundary = [&](std::uint32_t offset)
			{
				return offset <= size && (offset == size || (static_cast<unsigned char>(payload->text[offset]) & 0xC0u) != 0x80u);
			};
			if (!on_boundary(payload->caret) || !on_boundary(payload->selection_start) || !on_boundary(payload->selection_end))
				throw EngineException(EStatus::InvalidArgument, "input text payload offsets are out of range or inside a code point");

			if (payload->selection_start > payload->selection_end)
				throw EngineException(EStatus::InvalidArgument, "input text payload selection is inverted");
		}

		// The one control with an active composition, verified to still be the focused editable
		// target. A composition record that arrives after focus moved, or after the composing
		// control left the tree, is stale and is rejected rather than applied to whatever is
		// focused now.
		TextEditState& ActiveComposition(TreeModel const& tree, InputState& state)
		{
			if (state.m_composing_id == 0)
				throw EngineException(EStatus::InvalidArgument, "composition record received while no composition is active");

			if (state.m_composing_id != state.m_focus_id || EditableFocusTarget(tree, state) == nullptr)
				throw EngineException(EStatus::InvalidArgument, "composition record is stale: the composing control is no longer the focused editable control");

			auto it = state.m_text_edits.find(state.m_composing_id);
			if (it == state.m_text_edits.end() || it->second.composition.active == 0)
				throw EngineException(EStatus::InternalError, "composing control has no active composition state");

			return it->second;
		}

		// Places the caret from a pointer position using the same shaped layout the renderer draws
		// from. Without a metrics source the caret deterministically goes to the end of the text,
		// which is the documented degraded behaviour rather than an arbitrary guess. When 'extend'
		// is set the existing selection anchor is kept so the caret sweeps a selection out of it,
		// which is what Shift+click and drag selection both need.
		void PlaceCaretFromPointer(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, TextHitContext const& hit_context, ControlNode const& node, TextEditState& edit, Vec2 pt, bool extend)
		{
			auto const raw_text = edit.pending_text;
			auto const text = node.desc.masked != 0 ? MaskedTextOf(raw_text) : raw_text;
			auto layout_it = layout.find(node.desc.id);
			if (hit_context.shaper == nullptr || layout_it == layout.end() || text.empty())
			{
				edit.caret = static_cast<std::uint32_t>(raw_text.size());
				if (!extend)
					edit.selection_start = edit.caret;

				return;
			}

			// The run origin must match the renderer's: a TextBox is left-aligned and inset from
			// its own left edge, with both the inset and the font size taking the root's scale.
			auto const scale = ControlScale(tree, hit_context.placements, node.desc.id);
			auto const font = ResolveControlFont(tree, node.desc.font_resource_id);
			auto const placement = TextPlacementFor(node.desc.type);
			auto const origin_x = layout_it->second.x + placement.inset_dip * scale;
			auto const layout_height = hit_context.shaper->LayoutHeight(font.family, font.size * scale, text);
			auto const origin_y = TextOriginYDip(layout_it->second.y, layout_it->second.h, layout_height);

			// OffsetFromPoint already reports a grapheme boundary chosen by proximity, so it is
			// used verbatim; rounding it down again here would discard the trailing-hit result.
			auto const display_offset = hit_context.shaper->OffsetFromPoint(font.family, font.size * scale, text, pt.x - origin_x, pt.y - origin_y);
			auto const cluster_index = GraphemeCount(std::string_view(text).substr(0, display_offset));
			edit.caret = GraphemeOffsetAt(raw_text, cluster_index);
			if (!extend)
				edit.selection_start = edit.caret;
		}
	}

	std::string RawDisplayTextOf(TextEditState const& edit)
	{
		if (edit.composition.active == 0)
			return edit.pending_text;

		auto const at = std::min<std::uint32_t>(edit.composition.insert_at, static_cast<std::uint32_t>(edit.pending_text.size()));
		auto display = edit.pending_text.substr(0, at);
		display += edit.composition.text;
		display += edit.pending_text.substr(at);
		return display;
	}

	std::string MaskedTextOf(std::string_view text)
	{
		auto display = std::string{};
		auto const count = GraphemeCount(text);
		display.reserve(count * 3);
		for (auto i = std::uint32_t{}; i != count; ++i)
			display.append("\xE2\x80\xA2");

		return display;
	}

	std::uint32_t DisplayOffsetOf(ControlDesc const& desc, std::string_view raw_text, std::uint32_t raw_offset)
	{
		if (desc.masked == 0)
			return raw_offset;

		auto const clamped = ClampToGraphemeBoundary(raw_text, std::min<std::uint32_t>(raw_offset, static_cast<std::uint32_t>(raw_text.size())));
		auto const cluster_index = GraphemeCount(raw_text.substr(0, clamped));
		return cluster_index * 3U;
	}

	std::string DisplayTextOf(ControlDesc const& desc, TextEditState const& edit)
	{
		auto const raw = RawDisplayTextOf(edit);
		return desc.masked != 0 ? MaskedTextOf(raw) : raw;
	}

	TextEditRanges DisplayRangesOf(ControlDesc const& desc, TextEditState const& edit)
	{
		if (edit.composition.active == 0)
		{
			auto const raw = std::string_view(edit.pending_text);
			return TextEditRanges{
				.caret = DisplayOffsetOf(desc, raw, edit.caret),
				.selection_start = DisplayOffsetOf(desc, raw, std::min(edit.caret, edit.selection_start)),
				.selection_end = DisplayOffsetOf(desc, raw, std::max(edit.caret, edit.selection_start)),
				.composition_start = 0,
				.composition_length = 0,
			};
		}

		// While composing, the caret follows the IME's cursor inside the composition and the
		// selection is the IME's target clause, both reported against the spliced display string.
		auto const at = std::min<std::uint32_t>(edit.composition.insert_at, static_cast<std::uint32_t>(edit.pending_text.size()));
		auto const raw = RawDisplayTextOf(edit);
		auto const raw_composition_start = at;
		auto const raw_composition_end = at + static_cast<std::uint32_t>(edit.composition.text.size());
		return TextEditRanges{
			.caret = DisplayOffsetOf(desc, raw, at + edit.composition.caret),
			.selection_start = DisplayOffsetOf(desc, raw, at + edit.composition.sel_start),
			.selection_end = DisplayOffsetOf(desc, raw, at + edit.composition.sel_end),
			.composition_start = DisplayOffsetOf(desc, raw, raw_composition_start),
			.composition_length = DisplayOffsetOf(desc, raw, raw_composition_end) - DisplayOffsetOf(desc, raw, raw_composition_start),
		};
	}

	// The height of one drop-down row follows the accepted control's font size with padding.
	float ComboBoxRowHeight(TreeModel const& tree, ControlNode const& node)
	{
		// Match the closed box's resolved font closely enough for deterministic input and drawing.
		auto const font = ResolveControlFont(tree, node.desc.font_resource_id);
		return std::max(1.0f, font.size + 8.0f);
	}

	// Clamp the highlighted item into the visible scroll window.
	void KeepComboHighlightVisible(ControlNode const& node, InputState& state)
	{
		// A closed or empty ComboBox has no list window to maintain.
		if (state.m_open_combo_id != node.desc.id || node.desc.combo_item_count == 0 || state.m_combo_highlight_index < 0)
			return;

		auto const max_rows = std::max<std::uint32_t>(1U, std::min(node.desc.max_visible_items, node.desc.combo_item_count));
		auto const highlight = static_cast<std::uint32_t>(state.m_combo_highlight_index);
		if (highlight < state.m_combo_scroll_offset)
			state.m_combo_scroll_offset = highlight;
		else if (highlight >= state.m_combo_scroll_offset + max_rows)
			state.m_combo_scroll_offset = highlight + 1U - max_rows;
	}

	// Open the transient drop-down for 'node' and initialise highlight/scroll from the accepted selection.
	void OpenComboBox(ControlNode const& node, InputState& state)
	{
		// The accepted descriptor remains the only durable selection authority.
		state.m_open_combo_id = node.desc.id;
		state.m_combo_highlight_index = node.desc.selected_index >= 0 ? node.desc.selected_index : (node.desc.combo_item_count != 0 ? 0 : -1);
		state.m_combo_scroll_offset = 0;
		KeepComboHighlightVisible(node, state);
	}

	// Close any transient ComboBox popup without proposing a selection.
	void CloseComboBox(InputState& state)
	{
		// Closing forgets only transient list navigation state.
		state.m_open_combo_id = 0;
		state.m_combo_highlight_index = -1;
		state.m_combo_scroll_offset = 0;
	}

	// Emit the owning-app selection proposal for a ComboBox item index.
	void ProposeComboBoxSelection(EventQueue& events, ControlNode const& node, std::int32_t index, std::uint64_t accepted_revision)
	{
		// Keep descriptor selection immutable; this is the same proposal contract as Slider.
		if (index < 0 || static_cast<std::uint32_t>(index) >= node.desc.combo_item_count)
			return;
		if (!events.Push(node.desc.id, EEventKind::ValueChangeProposed, accepted_revision, 0, {}, 1, static_cast<double>(index)))
			throw EngineException(EStatus::QueueOverflow, std::format("event queue overflow while enqueueing ComboBox proposal for control {}", node.desc.id));
	}

	// Move an open ComboBox highlight by a signed row delta.
	void MoveComboHighlight(ControlNode const& node, InputState& state, std::int32_t delta)
	{
		// Clamp to the application-authored item domain.
		if (node.desc.combo_item_count == 0)
			return;

		auto const last = static_cast<std::int32_t>(node.desc.combo_item_count) - 1;
		auto current = state.m_combo_highlight_index >= 0 ? state.m_combo_highlight_index : 0;
		state.m_combo_highlight_index = std::clamp(current + delta, 0, last);
		KeepComboHighlightVisible(node, state);
	}


	void InputState::Prune(std::unordered_set<ControlId> const& live_ids)
	{
		if (!live_ids.contains(m_hover_id))
			m_hover_id = 0;
		if (!live_ids.contains(m_pressed_id))
			m_pressed_id = 0;
		if (!live_ids.contains(m_captured_id))
			m_captured_id = 0;
		if (!live_ids.contains(m_focus_id))
			m_focus_id = 0;

		// A composition whose control has gone is abandoned outright: its saved pending edit is
		// discarded along with the rest of the control's edit state just below.
		if (!live_ids.contains(m_composing_id))
			m_composing_id = 0;

		for (auto it = m_text_edits.begin(); it != m_text_edits.end();)
		{
			if (live_ids.contains(it->first))
				++it;
			else
				it = m_text_edits.erase(it);
		}
	}

	std::vector<ControlId> ComputeTabOrder(TreeModel const& tree)
	{
		std::vector<ControlId> order;
		for (auto root_id : tree.m_roots)
			ComputeTabOrderRecurse(tree, root_id, order);

		return order;
	}

	Rect ComboBoxPopupRect(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, ViewportState const& viewport, ControlId combo_id)
	{
		// Place the popup in viewport DIP space, flipping above when below has too little room.
		auto node_it = tree.m_controls.find(combo_id);
		auto layout_it = layout.find(combo_id);
		if (node_it == tree.m_controls.end() || layout_it == layout.end() || node_it->second.desc.type != EControlType::ComboBox)
			return Rect{};

		auto const& node = node_it->second;
		auto const control = layout_it->second;
		auto const row_count = std::min(node.desc.combo_item_count, node.desc.max_visible_items);
		auto const row_h = ComboBoxRowHeight(tree, node);
		auto const popup_h = row_h * static_cast<float>(row_count);
		auto const viewport_w = viewport.viewport_width_px * 96.0f / viewport.dpi;
		auto const viewport_h = viewport.viewport_height_px * 96.0f / viewport.dpi;
		auto x = std::clamp(control.x, 0.0f, std::max(0.0f, viewport_w - control.w));
		auto y = control.y + control.h;
		if (y + popup_h > viewport_h && control.y >= popup_h)
			y = control.y - popup_h;
		y = std::clamp(y, 0.0f, std::max(0.0f, viewport_h - popup_h));
		return Rect{ x, y, std::min(control.w, viewport_w), popup_h };
	}

	HitTestResult HitTestDetailed(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, InputState const& state, ViewportState const& viewport, Vec2 pt)
	{
		// The open popup is a topmost transient layer above every root.
		if (state.m_open_combo_id != 0)
		{
			auto node_it = tree.m_controls.find(state.m_open_combo_id);
			if (node_it != tree.m_controls.end() && node_it->second.desc.type == EControlType::ComboBox && tree.IsVisible(state.m_open_combo_id) && node_it->second.desc.enabled != 0)
			{
				auto popup = ComboBoxPopupRect(tree, layout, viewport, state.m_open_combo_id);
				if (RectContains(popup, pt))
				{
					auto const row_h = ComboBoxRowHeight(tree, node_it->second);
					auto const row = row_h > 0.0f ? static_cast<std::uint32_t>((pt.y - popup.y) / row_h) : 0U;
					auto const index = state.m_combo_scroll_offset + row;
					return HitTestResult{ state.m_open_combo_id, index < node_it->second.desc.combo_item_count ? static_cast<std::int32_t>(index) : -1, 1 };
				}
			}
		}

		// Fall back to ordinary control hit testing.
		auto id = HitTest(tree, layout, pt);
		return HitTestResult{ id, -1, 0 };
	}


	ControlId HitTest(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, Vec2 pt)
	{
		for (auto it = tree.m_roots.rbegin(); it != tree.m_roots.rend(); ++it)
		{
			auto hit = HitTestRecurse(tree, layout, *it, pt);
			if (hit != 0)
				return hit;
		}
		return 0;
	}

	void CancelActiveComposition(InputState& state)
	{
		if (state.m_composing_id == 0)
			return;

		auto it = state.m_text_edits.find(state.m_composing_id);
		if (it != state.m_text_edits.end() && it->second.composition.active != 0)
		{
			auto& edit = it->second;
			edit.pending_text = edit.composition.saved_pending_text;
			edit.caret = edit.composition.saved_caret;
			edit.selection_start = edit.composition.saved_selection_start;
			edit.edit_generation = edit.composition.saved_edit_generation;
			edit.composition = CompositionState{};
		}
		state.m_composing_id = 0;
	}

	bool HasEditableFocus(TreeModel const& tree, InputState const& state)
	{
		return EditableFocusTarget(tree, state) != nullptr;
	}

	bool InputKindCarriesText(EInputKind kind)
	{
		switch (kind)
		{
			case EInputKind::TextInput:
			case EInputKind::CompositionUpdate:
			case EInputKind::CompositionCommit:
			{
				return true;
			}
			case EInputKind::PointerMove:
			case EInputKind::PointerButtonDown:
			case EInputKind::PointerButtonUp:
			case EInputKind::PointerWheel:
			case EInputKind::KeyDown:
			case EInputKind::KeyUp:
			case EInputKind::Char:
			case EInputKind::FocusLost:
			case EInputKind::FocusGained:
			case EInputKind::CompositionStart:
			case EInputKind::CompositionCancel:
			{
				return false;
			}
			case EInputKind::Count:
			default:
			{
				throw EngineException(EStatus::InvalidArgument, "NormalizedInput: unknown input kind");
			}
		}
	}

	InputResult ProcessNormalizedInput(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, ViewportState const& viewport, NormalizedInput const& input, InputTextRecord const* text_payload, TextHitContext const& hit_context, InputState& state, EventQueue& events, std::uint64_t accepted_revision)
	{
		switch (input.kind)
		{
			case EInputKind::PointerMove:
			{
				auto hit_info = HitTestDetailed(tree, layout, state, viewport, Vec2{ input.pointer_x, input.pointer_y });
				auto hit = hit_info.control_id;
				auto changed = hit != state.m_hover_id;
				state.m_hover_id = hit;
				if (hit_info.in_combo_popup != 0 && hit_info.combo_item_index >= 0)
				{
					state.m_combo_highlight_index = hit_info.combo_item_index;
					changed = true;
				}

				// A captured TextBox is mid drag-selection, so the pointer sweeps the caret away
				// from the anchor the press established. Capture is what makes this keep working
				// once the pointer leaves the control's bounds.
				if (state.m_captured_id != 0)
				{
					auto it = tree.m_controls.find(state.m_captured_id);
					if (it != tree.m_controls.end() && it->second.desc.type == EControlType::TextBox && it->second.desc.enabled != 0)
					{
						auto& edit = GetOrInitTextEdit(state, it->second);
						auto const caret_before = edit.caret;
						PlaceCaretFromPointer(tree, layout, hit_context, it->second, edit, Vec2{ input.pointer_x, input.pointer_y }, true);
						changed = changed || edit.caret != caret_before;
					}
					else if (it != tree.m_controls.end() && it->second.desc.type == EControlType::Slider && it->second.desc.enabled != 0)
					{
						ProposeSliderFromPointer(layout, events, it->second, Vec2{ input.pointer_x, input.pointer_y }, accepted_revision);
						changed = true;
					}
				}

				return InputResult{ hit != 0 || state.m_captured_id != 0, changed };
			}
			case EInputKind::PointerButtonDown:
			{
				// A pointer press is a deliberate move away from whatever the IME was composing, so
				// the composition is cancelled and the pending edit restored exactly before the
				// press is interpreted. This keeps a click from silently committing half a word.
				auto const was_composing = state.m_composing_id != 0;
				CancelActiveComposition(state);

				auto hit_info = HitTestDetailed(tree, layout, state, viewport, Vec2{ input.pointer_x, input.pointer_y });
				auto hit = hit_info.control_id;
				if (hit_info.in_combo_popup != 0)
				{
					if (hit_info.combo_item_index >= 0)
					{
						auto const& combo = tree.m_controls.at(hit_info.control_id);
						state.m_combo_highlight_index = hit_info.combo_item_index;
						ProposeComboBoxSelection(events, combo, hit_info.combo_item_index, accepted_revision);
						CloseComboBox(state);
					}
					return InputResult{ true, true };
				}
				if (state.m_open_combo_id != 0 && hit != state.m_open_combo_id)
				{
					CloseComboBox(state);
					return InputResult{ true, true };
				}
				if (hit == 0)
				{
					// Outside click closes an open ComboBox first and consumes that light-dismiss click.
					if (state.m_open_combo_id != 0)
					{
						CloseComboBox(state);
						return InputResult{ true, true };
					}

					// Outside click: clear focus (if any) but do not consume the input, so the
					// application's own scene/game input still observes it (section 7.3). Capture
					// whether focus actually existed before clearing it, since 'invalidate' must
					// report that a redraw is needed precisely when the focus visual disappeared.
					auto had_focus = state.m_focus_id != 0;
					if (had_focus)
					{
						PushOrThrow(events, 0, EEventKind::FocusChanged, accepted_revision, 0, {});
						state.m_focus_id = 0;
					}
					return InputResult{ false, had_focus || was_composing };
				}
				if (input.button != EPointerButton::Left)
					return InputResult{ true, was_composing };

				if (auto focus_target = NearestFocusable(tree, hit); focus_target != 0 && focus_target != state.m_focus_id)
				{
					PushOrThrow(events, focus_target, EEventKind::FocusChanged, accepted_revision, 0, {});
					state.m_focus_id = focus_target;
					SeedFocusedTextEdit(tree, state, focus_target);
				}

				// A press on 'hit' itself or any of its content descendants activates the nearest
				// enclosing Button/TextBox/Slider ancestor (see NearestOfType).
				if (auto button_target = NearestOfType(tree, hit, EControlType::Button); button_target != 0 && tree.m_controls.at(button_target).desc.enabled != 0)
				{
					if (state.m_captured_id != button_target)
						PushOrThrow(events, button_target, EEventKind::PointerCaptureChanged, accepted_revision, 0, {});

					state.m_pressed_id = button_target;
					state.m_captured_id = button_target;
				}
				else if (auto textbox_target = NearestOfType(tree, hit, EControlType::TextBox); textbox_target != 0 && tree.m_controls.at(textbox_target).desc.enabled != 0)
				{
					auto const& node = tree.m_controls.at(textbox_target);
					auto& edit = GetOrInitTextEdit(state, node);

					// Shift+click extends the existing selection from its anchor; a plain press
					// collapses it and becomes the anchor for the drag that may follow.
					auto const extend = (input.modifiers & static_cast<std::uint32_t>(EInputModifier::Shift)) != 0;
					PlaceCaretFromPointer(tree, layout, hit_context, node, edit, Vec2{ input.pointer_x, input.pointer_y }, extend);

					// The TextBox takes capture so a drag that leaves its bounds keeps selecting.
					if (state.m_captured_id != textbox_target)
						PushOrThrow(events, textbox_target, EEventKind::PointerCaptureChanged, accepted_revision, 0, {});

					state.m_pressed_id = textbox_target;
					state.m_captured_id = textbox_target;
				}
				else if (auto combo_target = NearestOfType(tree, hit, EControlType::ComboBox); combo_target != 0 && tree.m_controls.at(combo_target).desc.enabled != 0)
				{
					auto const& node = tree.m_controls.at(combo_target);
					if (state.m_open_combo_id == combo_target)
						CloseComboBox(state);
					else
						OpenComboBox(node, state);
				}
				else if (auto slider_target = NearestOfType(tree, hit, EControlType::Slider); slider_target != 0 && tree.m_controls.at(slider_target).desc.enabled != 0)
				{
					auto const& node = tree.m_controls.at(slider_target);
					ProposeSliderFromPointer(layout, events, node, Vec2{ input.pointer_x, input.pointer_y }, accepted_revision);
					if (state.m_captured_id != slider_target)
						PushOrThrow(events, slider_target, EEventKind::PointerCaptureChanged, accepted_revision, 0, {});

					state.m_pressed_id = slider_target;
					state.m_captured_id = slider_target;
				}
				return InputResult{ true, true };
			}
			case EInputKind::PointerButtonUp:
			{
				if (input.button != EPointerButton::Left || state.m_pressed_id == 0)
					return InputResult{ state.m_captured_id != 0, false };

				auto hit_info = HitTestDetailed(tree, layout, state, viewport, Vec2{ input.pointer_x, input.pointer_y });
				auto hit = hit_info.control_id;
				if (hit_info.in_combo_popup != 0 && hit_info.combo_item_index >= 0)
				{
					auto const& combo = tree.m_controls.at(hit_info.control_id);
					ProposeComboBoxSelection(events, combo, hit_info.combo_item_index, accepted_revision);
					CloseComboBox(state);
					return InputResult{ true, true };
				}
				auto pressed_id = state.m_pressed_id;
				if (NearestOfType(tree, hit, EControlType::Button) == pressed_id)
					PushOrThrow(events, pressed_id, EEventKind::CommandInvoked, accepted_revision, 0, {});

				if (state.m_captured_id != 0)
					PushOrThrow(events, 0, EEventKind::PointerCaptureChanged, accepted_revision, 0, {});

				state.m_pressed_id = 0;
				state.m_captured_id = 0;
				return InputResult{ true, true };
			}
			case EInputKind::PointerWheel:
			{
				// ELayoutMode::Scroll's offset is an application-authored descriptor field
				// (LayoutParams::scroll_offset_x/y, set via transaction), not a runtime input
				// target the state machine owns the way it owns hover/focus/pressed; wheel input
				// therefore has nothing to directly act on here and is left unconsumed.
				if (state.m_open_combo_id != 0)
				{
					auto node_it = tree.m_controls.find(state.m_open_combo_id);
					if (node_it != tree.m_controls.end() && node_it->second.desc.type == EControlType::ComboBox)
					{
						auto const& node = node_it->second;
						auto const rows = static_cast<std::int32_t>(std::max<std::uint32_t>(1U, node.desc.max_visible_items));
						MoveComboHighlight(node, state, input.wheel_delta < 0.0f ? 1 : -1);
						(void)rows;
						return InputResult{ true, true };
					}
				}
				return InputResult{ false, false };
			}
			case EInputKind::KeyDown:
			{
				if (input.vk == VK_TAB)
				{
					// Moving focus away abandons any composition, exactly as a pointer press does.
					CancelActiveComposition(state);
					CloseComboBox(state);

					auto order = ComputeTabOrder(tree);
					if (order.empty())
						return InputResult{ false, false };

					auto direction = (input.modifiers & static_cast<std::uint32_t>(EInputModifier::Shift)) != 0 ? -1 : 1;
					auto current = std::find(order.begin(), order.end(), state.m_focus_id);
					std::size_t next_index;
					if (current == order.end())
					{
						next_index = direction > 0 ? 0 : order.size() - 1;
					}
					else
					{
						auto index = static_cast<std::ptrdiff_t>(current - order.begin());
						auto count = static_cast<std::ptrdiff_t>(order.size());
						next_index = static_cast<std::size_t>(((index + direction) % count + count) % count);
					}

					auto next_focus = order[next_index];
					PushOrThrow(events, next_focus, EEventKind::FocusChanged, accepted_revision, 0, {});
					state.m_focus_id = next_focus;
					SeedFocusedTextEdit(tree, state, next_focus);
					return InputResult{ true, true };
				}

				if (state.m_focus_id == 0)
				{
					if (input.vk == VK_ESCAPE && state.m_open_combo_id != 0)
					{
						CloseComboBox(state);
						return InputResult{ true, true };
					}
					return InputResult{ false, false };
				}

				auto const& node = tree.m_controls.at(state.m_focus_id);
				if (node.desc.type == EControlType::ComboBox && node.desc.enabled != 0)
				{
					// Closed keys either open the list or propose adjacent accepted indices directly.
					auto const alt_held = (input.modifiers & static_cast<std::uint32_t>(EInputModifier::Alt)) != 0;
					if (state.m_open_combo_id != node.desc.id)
					{
						switch (input.vk)
						{
							case VK_DOWN:
							{
								if (alt_held)
									OpenComboBox(node, state);
								else
									ProposeComboBoxSelection(events, node, std::min<std::int32_t>(static_cast<std::int32_t>(node.desc.combo_item_count) - 1, node.desc.selected_index + 1), accepted_revision);
								return InputResult{ true, true };
							}
							case VK_UP:
							{
								ProposeComboBoxSelection(events, node, std::max(0, node.desc.selected_index <= 0 ? 0 : node.desc.selected_index - 1), accepted_revision);
								return InputResult{ true, true };
							}
							case VK_HOME: { ProposeComboBoxSelection(events, node, 0, accepted_revision); return InputResult{ true, true }; }
							case VK_END: { ProposeComboBoxSelection(events, node, static_cast<std::int32_t>(node.desc.combo_item_count) - 1, accepted_revision); return InputResult{ true, true }; }
							case VK_F4:
							case VK_SPACE:
							case VK_RETURN: { OpenComboBox(node, state); return InputResult{ true, true }; }
							default: { break; }
						}
					}
					else
					{
						auto const page = static_cast<std::int32_t>(std::max<std::uint32_t>(1U, node.desc.max_visible_items));
						switch (input.vk)
						{
							case VK_ESCAPE: { CloseComboBox(state); return InputResult{ true, true }; }
							case VK_RETURN: { ProposeComboBoxSelection(events, node, state.m_combo_highlight_index, accepted_revision); CloseComboBox(state); return InputResult{ true, true }; }
							case VK_UP: { MoveComboHighlight(node, state, -1); return InputResult{ true, true }; }
							case VK_DOWN: { MoveComboHighlight(node, state, 1); return InputResult{ true, true }; }
							case VK_HOME: { state.m_combo_highlight_index = 0; KeepComboHighlightVisible(node, state); return InputResult{ true, true }; }
							case VK_END: { state.m_combo_highlight_index = static_cast<std::int32_t>(node.desc.combo_item_count) - 1; KeepComboHighlightVisible(node, state); return InputResult{ true, true }; }
							case VK_PRIOR: { MoveComboHighlight(node, state, -page); return InputResult{ true, true }; }
							case VK_NEXT: { MoveComboHighlight(node, state, page); return InputResult{ true, true }; }
							case VK_F4: { CloseComboBox(state); return InputResult{ true, true }; }
							default: { break; }
						}
					}
				}

				if (input.vk == VK_RETURN && node.desc.type == EControlType::Button && node.desc.enabled != 0)
				{
					PushOrThrow(events, node.desc.id, EEventKind::CommandInvoked, accepted_revision, 0, {});
					return InputResult{ true, true };
				}
				if (input.vk == VK_SPACE && node.desc.type == EControlType::Button && node.desc.enabled != 0)
				{
					state.m_pressed_id = node.desc.id;
					return InputResult{ true, true };
				}
				if (node.desc.type == EControlType::Slider && node.desc.enabled != 0)
				{
					switch (input.vk)
					{
						case VK_LEFT:
						case VK_DOWN:
						{
							ProposeSliderValue(events, node, static_cast<double>(node.desc.value) - node.desc.step, accepted_revision);
							return InputResult{ true, true };
						}
						case VK_RIGHT:
						case VK_UP:
						{
							ProposeSliderValue(events, node, static_cast<double>(node.desc.value) + node.desc.step, accepted_revision);
							return InputResult{ true, true };
						}
						case VK_HOME:
						{
							ProposeSliderValue(events, node, node.desc.minimum, accepted_revision);
							return InputResult{ true, true };
						}
						case VK_END:
						{
							ProposeSliderValue(events, node, node.desc.maximum, accepted_revision);
							return InputResult{ true, true };
						}
						default:
						{
							return InputResult{ false, false };
						}
					}
				}
				if (node.desc.type != EControlType::TextBox || node.desc.enabled == 0)
					return InputResult{ false, false };

				auto& edit = GetOrInitTextEdit(state, node);
				auto const masked = node.desc.masked != 0;

				// While an IME owns the keyboard, editing keys belong to the IME, not to this edit
				// buffer; the IME reports their effect as composition updates instead.
				if (edit.composition.active != 0)
					return InputResult{ false, false };

				auto shift_held = (input.modifiers & static_cast<std::uint32_t>(EInputModifier::Shift)) != 0;
				auto ctrl_held = (input.modifiers & static_cast<std::uint32_t>(EInputModifier::Ctrl)) != 0;
				auto changed = false;

				// Caret and selection moves redraw the control without changing its text, so the
				// pre-edit positions are captured here and compared after the switch rather than
				// making every navigation case remember to raise its own invalidate flag.
				auto const caret_before = edit.caret;
				auto const selection_before = edit.selection_start;
				switch (input.vk)
				{
					case VK_BACK:
					{
						if (edit.caret != edit.selection_start)
						{
							changed = ReplaceSelection(node.desc, edit, {});
						}
						else if (ctrl_held && masked && !edit.pending_text.empty())
						{
							// A masked value behaves like one password word, so Ctrl+Backspace
							// clears the whole value rather than revealing structure through stops.
							edit.pending_text.clear();
							edit.caret = 0;
							edit.selection_start = 0;
							changed = true;
						}
						else if (edit.caret > 0)
						{
							// Backspace removes one whole grapheme cluster, so a flag, a skin-toned
							// emoji or an accented letter disappears in one keystroke rather than
							// decomposing into its parts.
							auto from = PrevGraphemeBoundary(edit.pending_text, edit.caret);
							edit.pending_text.erase(from, edit.caret - from);
							edit.caret = from;
							edit.selection_start = from;
							changed = true;
						}
						break;
					}
					case VK_DELETE:
					{
						if (edit.caret != edit.selection_start)
						{
							changed = ReplaceSelection(node.desc, edit, {});
						}
						else if (ctrl_held && masked && !edit.pending_text.empty())
						{
							// A masked value behaves like one password word, so Ctrl+Delete clears
							// the whole value rather than revealing structure through stops.
							edit.pending_text.clear();
							edit.caret = 0;
							edit.selection_start = 0;
							changed = true;
						}
						else if (edit.caret < edit.pending_text.size())
						{
							auto to = NextGraphemeBoundary(edit.pending_text, edit.caret);
							edit.pending_text.erase(edit.caret, to - edit.caret);
							changed = true;
						}
						break;
					}
					case VK_LEFT:
					{
						edit.caret = ctrl_held && masked ? 0 : ctrl_held ? PrevWordBoundary(edit.pending_text, edit.caret) : PrevGraphemeBoundary(edit.pending_text, edit.caret);
						if (!shift_held)
							edit.selection_start = edit.caret;

						break;
					}
					case VK_RIGHT:
					{
						edit.caret = ctrl_held && masked ? static_cast<std::uint32_t>(edit.pending_text.size()) : ctrl_held ? NextWordBoundary(edit.pending_text, edit.caret) : NextGraphemeBoundary(edit.pending_text, edit.caret);
						if (!shift_held)
							edit.selection_start = edit.caret;

						break;
					}
					case VK_HOME:
					{
						edit.caret = 0;
						if (!shift_held)
							edit.selection_start = edit.caret;

						break;
					}
					case VK_END:
					{
						edit.caret = static_cast<std::uint32_t>(edit.pending_text.size());
						if (!shift_held)
							edit.selection_start = edit.caret;

						break;
					}
					case 'A':
					{
						if (!ctrl_held)
							return InputResult{ false, false };

						edit.selection_start = 0;
						edit.caret = static_cast<std::uint32_t>(edit.pending_text.size());
						break;
					}
					case 'C':
					case 'X':
					{
						if (!ctrl_held)
							return InputResult{ false, false };

						if (masked)
							break;

						auto lo = std::min(edit.caret, edit.selection_start);
						auto hi = std::max(edit.caret, edit.selection_start);
						if (hi > lo)
							ClipboardSetText(std::string_view(edit.pending_text).substr(lo, hi - lo));

						if (input.vk == 'X' && hi > lo)
							changed = ReplaceSelection(node.desc, edit, {});

						break;
					}
					case 'V':
					{
						if (!ctrl_held)
							return InputResult{ false, false };

						changed = ReplaceSelection(node.desc, edit, ClipboardGetText());
						break;
					}
					default:
					{
						return InputResult{ false, false };
					}
				}

				if (changed)
					ProposeTextChange(events, node.desc.id, edit, accepted_revision);

				auto const moved = edit.caret != caret_before || edit.selection_start != selection_before;
				return InputResult{ true, changed || moved };
			}
			case EInputKind::KeyUp:
			{
				if (input.vk == VK_SPACE && state.m_pressed_id != 0)
				{
					auto const& node = tree.m_controls.at(state.m_pressed_id);
					if (node.desc.type == EControlType::Button)
					{
						PushOrThrow(events, node.desc.id, EEventKind::CommandInvoked, accepted_revision, 0, {});
						state.m_pressed_id = 0;
						return InputResult{ true, true };
					}
				}
				return InputResult{ false, false };
			}
			case EInputKind::Char:
			{
				// C0 controls, DEL and the C1 block are keyboard side effects (Ctrl+letter, Escape,
				// Backspace) rather than text, so they never reach the edit buffer; the editing keys
				// they correspond to arrive separately as KeyDown records.
				auto const is_control = input.char_code < 0x20u || input.char_code == 0x7Fu || (input.char_code >= 0x80u && input.char_code <= 0x9Fu);
				auto const* node = EditableFocusTarget(tree, state);
				if (node == nullptr || is_control)
					return InputResult{ false, false };

				auto& edit = GetOrInitTextEdit(state, *node);
				if (edit.composition.active != 0)
					return InputResult{ false, false }; // characters during composition arrive as composition records

				std::string insert;
				Utf8Append(insert, static_cast<char32_t>(input.char_code));
				auto changed = ReplaceSelection(node->desc, edit, insert);
				if (changed)
					ProposeTextChange(events, node->desc.id, edit, accepted_revision);

				return InputResult{ true, changed };
			}
			case EInputKind::TextInput:
			{
				// Committed text with no composition involved: a pasted or injected string, or the
				// character a dead-key sequence finally produced.
				ValidateTextPayload(text_payload, false);

				auto const* node = EditableFocusTarget(tree, state);
				if (node == nullptr)
					return InputResult{ false, false };

				auto& edit = GetOrInitTextEdit(state, *node);
				if (edit.composition.active != 0)
					throw EngineException(EStatus::InvalidArgument, "TextInput received while a composition is active; commit or cancel it first");

				auto changed = ReplaceSelection(node->desc, edit, text_payload->text);
				if (changed)
					ProposeTextChange(events, node->desc.id, edit, accepted_revision);

				return InputResult{ true, changed };
			}
			case EInputKind::CompositionStart:
			{
				auto const* node = EditableFocusTarget(tree, state);
				if (node == nullptr)
					return InputResult{ false, false }; // nothing to compose into; the host keeps the input

				if (state.m_composing_id != 0)
					throw EngineException(EStatus::InvalidArgument, "CompositionStart received while a composition is already active");

				auto& edit = GetOrInitTextEdit(state, *node);

				// The pending edit is saved verbatim so a later cancellation restores it exactly,
				// including its edit generation - a cancelled composition must leave no trace.
				edit.composition = CompositionState{
					.insert_at = std::min(edit.caret, edit.selection_start),
					.saved_pending_text = edit.pending_text,
					.saved_caret = edit.caret,
					.saved_selection_start = edit.selection_start,
					.saved_edit_generation = edit.edit_generation,
					.active = 1,
				};

				// A composition replaces the selection, but only visually: the characters are
				// removed from the pending text without proposing anything, because nothing is
				// committed until the IME produces a result string.
				auto const lo = std::min(edit.caret, edit.selection_start);
				auto const hi = std::max(edit.caret, edit.selection_start);
				if (hi > lo)
				{
					edit.pending_text.erase(lo, hi - lo);
					edit.caret = lo;
					edit.selection_start = lo;
				}

				state.m_composing_id = node->desc.id;
				return InputResult{ true, true };
			}
			case EInputKind::CompositionUpdate:
			{
				ValidateTextPayload(text_payload, true);
				auto& edit = ActiveComposition(tree, state);
				auto const& node = tree.m_controls.at(state.m_composing_id);

				// The candidate string is clamped here, while it is still visible, so the user sees
				// exactly the text that a commit will keep. Clamping only at commit time would let
				// the field display more than it can hold and then silently truncate it.
				auto const at = std::min<std::uint32_t>(edit.composition.insert_at, static_cast<std::uint32_t>(edit.pending_text.size()));
				auto const clamped = TruncateInsertion(node.desc, std::string_view(edit.pending_text).substr(0, at), std::string_view(edit.pending_text).substr(at), text_payload->text);

				// Offsets the IME reported against the full candidate must be pulled back onto the
				// clamped string, and onto its cluster boundaries, before they become caret and
				// selection positions.
				auto const limit = static_cast<std::uint32_t>(clamped.size());
				edit.composition.text = std::string(clamped);
				edit.composition.caret = ClampToGraphemeBoundary(edit.composition.text, std::min(text_payload->caret, limit));
				edit.composition.sel_start = ClampToGraphemeBoundary(edit.composition.text, std::min(text_payload->selection_start, limit));
				edit.composition.sel_end = ClampToGraphemeBoundary(edit.composition.text, std::min(text_payload->selection_end, limit));
				edit.composition.sel_end = std::max(edit.composition.sel_start, edit.composition.sel_end);
				return InputResult{ true, true };
			}
			case EInputKind::CompositionCommit:
			{
				ValidateTextPayload(text_payload, false);
				auto& edit = ActiveComposition(tree, state);
				auto const& node = tree.m_controls.at(state.m_composing_id);

				// Only now does the composed text become part of the durable proposal. Insertion
				// happens at the recorded point rather than the live caret so an IME that moved
				// the caret around during composition still commits where composition began.
				edit.caret = std::min<std::uint32_t>(edit.composition.insert_at, static_cast<std::uint32_t>(edit.pending_text.size()));
				edit.selection_start = edit.caret;
				ReplaceSelection(node.desc, edit, text_payload->text);

				auto const changed = edit.pending_text != edit.composition.saved_pending_text;
				edit.composition = CompositionState{};
				state.m_composing_id = 0;

				if (changed)
					ProposeTextChange(events, node.desc.id, edit, accepted_revision);

				return InputResult{ true, true };
			}
			case EInputKind::CompositionCancel:
			{
				// Validate the composition is live before restoring, so a spurious cancel is
				// reported rather than quietly resetting an edit that was never composing.
				ActiveComposition(tree, state);
				CancelActiveComposition(state);
				return InputResult{ true, true };
			}
			case EInputKind::FocusLost:
			{
				// Losing focus mid-composition abandons it: the IME's window is gone, so the only
				// deterministic outcome is the pending edit exactly as it was before composing.
				auto const was_composing = state.m_composing_id != 0;
				CancelActiveComposition(state);
				CloseComboBox(state);

				if (state.m_focus_id == 0)
					return InputResult{ true, was_composing };

				PushOrThrow(events, 0, EEventKind::FocusChanged, accepted_revision, 0, {});
				state.m_focus_id = 0;
				state.m_pressed_id = 0;
				state.m_captured_id = 0;
				return InputResult{ true, true };
			}
			case EInputKind::FocusGained:
			{
				// Focus restoration on host-window activation is not implemented in this
				// milestone; the application may re-focus a control explicitly if desired.
				return InputResult{ false, false };
			}
			case EInputKind::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("ProcessNormalizedInput: unknown EInputKind {}", static_cast<int>(input.kind)));
			}
		}
	}

	std::int32_t ReconcileInputAfterTransaction(TreeModel const& new_tree, std::vector<ControlId> const& old_tab_order, InputState& state, EventQueue& events, std::uint64_t accepted_revision)
	{
		// A hidden or collapsed subtree must not keep hover, pressed, or captured interaction.
		auto changed = false;
		auto capture_changed = false;
		if (state.m_hover_id != 0 && !new_tree.IsVisible(state.m_hover_id))
		{
			state.m_hover_id = 0;
			changed = true;
		}
		if (state.m_pressed_id != 0 && !new_tree.IsVisible(state.m_pressed_id))
		{
			state.m_pressed_id = 0;
			changed = true;
		}
		if (state.m_captured_id != 0 && !new_tree.IsVisible(state.m_captured_id))
		{
			state.m_captured_id = 0;
			changed = true;
			capture_changed = true;
		}

		// Recover only a previously focused target; an unfocused tree must not invent initial focus.
		auto focus_changed = false;
		if (state.m_focus_id != 0)
		{
			auto new_tab_order = ComputeTabOrder(new_tree);
			if (std::find(new_tab_order.begin(), new_tab_order.end(), state.m_focus_id) == new_tab_order.end())
			{
				// The outgoing control cannot keep a live composition after becoming unavailable.
				CancelActiveComposition(state);
				ControlId next_focus = 0;
				if (!new_tab_order.empty())
				{
					auto old_it = std::find(old_tab_order.begin(), old_tab_order.end(), state.m_focus_id);
					auto old_index = old_it != old_tab_order.end() ? static_cast<std::size_t>(old_it - old_tab_order.begin()) : 0u;
					auto new_index = std::min(old_index, new_tab_order.size() - 1);
					next_focus = new_tab_order[new_index];
				}
				state.m_focus_id = next_focus;
				SeedFocusedTextEdit(new_tree, state, next_focus);
				focus_changed = true;
			}
		}

		// Commit safe interaction state before notification: a full queue cannot resurrect a hidden target.
		if (capture_changed)
			PushOrThrow(events, 0, EEventKind::PointerCaptureChanged, accepted_revision, 0, {});

		if (focus_changed)
			PushOrThrow(events, state.m_focus_id, EEventKind::FocusChanged, accepted_revision, 0, {});

		if (state.m_open_combo_id != 0)
		{
			auto open_it = new_tree.m_controls.find(state.m_open_combo_id);
			if (open_it == new_tree.m_controls.end() || open_it->second.desc.type != EControlType::ComboBox || open_it->second.desc.enabled == 0 || !new_tree.IsVisible(state.m_open_combo_id))
			{
				CloseComboBox(state);
				changed = true;
			}
		}

		return changed || focus_changed ? 1 : 0;
	}

	void ReconcileTextEditsAfterTransaction(TreeModel const& new_tree, InputState& state)
	{
		for (auto& [id, edit] : state.m_text_edits)
		{
			// An uninitialized entry has no outstanding local state to reconcile, and an entry
			// for a control that no longer exists (or is no longer a TextBox) is left for Prune to
			// discard - reconciling it against a nonexistent/foreign descriptor would be meaningless.
			if (edit.initialized == 0)
				continue;

			auto node_it = new_tree.m_controls.find(id);
			if (node_it == new_tree.m_controls.end() || node_it->second.desc.type != EControlType::TextBox)
				continue;

			auto const& descriptor_text = node_it->second.text;
			if (descriptor_text == edit.pending_text)
			{
				// The application committed exactly what was proposed (or nothing was proposed):
				// the proposal, if any, is now acknowledged and no longer outstanding.
				edit.last_accepted_text = descriptor_text;
				edit.edit_generation = 0;
			}
			else if (descriptor_text != edit.last_accepted_text)
			{
				// The application changed the text to something other than what was proposed -
				// managed normalization/rejection - so the local edit is discarded in favour of the
				// descriptor's text, with the caret/selection deterministically collapsed to its end.
				edit.pending_text = descriptor_text;
				edit.caret = static_cast<std::uint32_t>(edit.pending_text.size());
				edit.selection_start = edit.caret;
				edit.last_accepted_text = descriptor_text;
				edit.edit_generation = 0;
			}
			// else: the descriptor is unchanged from the last accepted text but still differs from
			// the pending text, so this transaction was unrelated to this control - the outstanding
			// local proposal is preserved untouched.
		}
	}

	InputResult ApplySemanticAction(TreeModel const& tree, SemanticActionRequest const& request, InputState& state, EventQueue& events, std::uint64_t accepted_revision)
	{
		// A stale id is the normal consequence of a client acting on an element the application has
		// since removed, so it is reported as a rejected request rather than treated as corruption.
		auto const node_it = tree.m_controls.find(request.control_id);
		if (node_it == tree.m_controls.end())
			throw EngineException(EStatus::InvalidArgument, std::format("semantic action names control {} which is not in the accepted tree", request.control_id));

		auto const& node = node_it->second;
		if (!tree.IsVisible(request.control_id))
			throw EngineException(EStatus::UnsupportedFeature, "semantic action target is hidden or collapsed");

		switch (request.kind)
		{
			case ESemanticActionKind::Focus:
			{
				if (node.desc.enabled == 0 || node.desc.focusable == 0)
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} is not a focusable target", request.control_id));

				if (state.m_focus_id == request.control_id)
					return InputResult{ true, false };

				// Moving focus abandons any composition, exactly as Tab and a pointer press do.
				CancelActiveComposition(state);
				PushOrThrow(events, request.control_id, EEventKind::FocusChanged, accepted_revision, 0, {});
				state.m_focus_id = request.control_id;
				SeedFocusedTextEdit(tree, state, request.control_id);
				return InputResult{ true, true };
			}
			case ESemanticActionKind::Invoke:
			{
				if (node.desc.type != EControlType::Button || node.desc.enabled == 0)
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} does not support Invoke", request.control_id));

				// Identical to the pointer-release and Enter activations: one CommandInvoked event
				// carrying no payload and no edit generation.
				PushOrThrow(events, node.desc.id, EEventKind::CommandInvoked, accepted_revision, 0, {});
				return InputResult{ true, true };
			}
			case ESemanticActionKind::SetValue:
			{
				if (node.desc.enabled == 0 || (node.desc.type != EControlType::TextBox && node.desc.type != EControlType::Slider))
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} does not support SetValue", request.control_id));

				if (node.desc.type == EControlType::Slider)
				{
					if (!std::isfinite(request.numeric_value) || request.numeric_value < node.desc.minimum || request.numeric_value > node.desc.maximum)
						throw EngineException(EStatus::InvalidArgument, std::format("slider semantic value {} must be finite and within [{}, {}]", request.numeric_value, node.desc.minimum, node.desc.maximum));

					ProposeSliderValue(events, node, request.numeric_value, accepted_revision);
					return InputResult{ true, true };
				}

				if (!Utf8Validate(request.text))
					throw EngineException(EStatus::InvalidArgument, "semantic SetValue text is not valid UTF-8");

				// A programmatic value replacement supersedes whatever the IME was converting, so
				// the composition is discarded before the buffer changes underneath it.
				if (state.m_composing_id == request.control_id)
					CancelActiveComposition(state);

				auto& edit = GetOrInitTextEdit(state, node);
				auto const before = edit.pending_text;

				// Selecting the whole buffer and reusing the keyboard path's replacement helper is
				// what gives max_text_length truncation and grapheme safety for free, and leaves
				// last_accepted_text - the application-authoritative text - untouched.
				edit.selection_start = 0;
				edit.caret = static_cast<std::uint32_t>(edit.pending_text.size());
				ReplaceSelection(node.desc, edit, request.text);
				if (edit.pending_text == before)
					return InputResult{ true, false };

				ProposeTextChange(events, node.desc.id, edit, accepted_revision);
				return InputResult{ true, true };
			}
			case ESemanticActionKind::ExpandCollapse:
			{
				if (node.desc.type != EControlType::ComboBox || node.desc.enabled == 0)
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} does not support ExpandCollapse", request.control_id));

				if (state.m_open_combo_id == request.control_id)
					CloseComboBox(state);
				else
					OpenComboBox(node, state);

				return InputResult{ true, true };
			}
			case ESemanticActionKind::SetSelection:
			{
				if (node.desc.type != EControlType::TextBox || node.desc.enabled == 0)
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} does not support SetSelection", request.control_id));

				auto& edit = GetOrInitTextEdit(state, node);

				// While composing, the reported offsets address the spliced display string rather
				// than the pending buffer, so honouring them would move the caret into text the
				// application does not own yet.
				if (edit.composition.active != 0)
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} cannot change its selection while an IME composition is active", request.control_id));

				// Snap both edges to grapheme-cluster boundaries so a selection edge can never land
				// inside a surrogate pair, a combining sequence or an emoji ZWJ sequence.
				auto const text = std::string_view(edit.pending_text);
				auto const size = static_cast<std::uint32_t>(text.size());
				auto const lo = ClampToGraphemeBoundary(text, std::min(std::min(request.selection_start, request.selection_end), size));
				auto const hi = ClampToGraphemeBoundary(text, std::min(std::max(request.selection_start, request.selection_end), size));
				auto const changed = edit.selection_start != lo || edit.caret != hi;
				edit.selection_start = lo;
				edit.caret = hi;
				return InputResult{ true, changed };
			}
			default:
			{
				throw EngineException(EStatus::InvalidArgument, std::format("unknown semantic action kind {}", static_cast<int>(request.kind)));
			}
		}
	}
}
