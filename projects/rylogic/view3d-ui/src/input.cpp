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
		// control (section 7.3). Only Button/TextBox are closed interactive controls in this milestone.
		bool IsHitTestable(EControlType type)
		{
			switch (type)
			{
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::Text:
				{
					return false;
				}
				case EControlType::TextBox:
				case EControlType::Button:
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
			if (node.desc.visible == 0)
				return 0;

			for (auto it = node.children.rbegin(); it != node.children.rend(); ++it)
			{
				auto hit = HitTestRecurse(tree, layout, *it, pt);
				if (hit != 0)
					return hit;
			}

			auto rect_it = layout.find(id);
			return rect_it != layout.end() && RectContains(rect_it->second, pt) && IsHitTestable(node.desc.type) ? id : 0;
		}

		void ComputeTabOrderRecurse(TreeModel const& tree, ControlId id, std::vector<ControlId>& order)
		{
			auto const& node = tree.m_controls.at(id);
			if (node.desc.visible == 0)
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
			auto const text = edit.pending_text;
			auto layout_it = layout.find(node.desc.id);
			if (hit_context.shaper == nullptr || layout_it == layout.end() || text.empty())
			{
				edit.caret = static_cast<std::uint32_t>(text.size());
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

			// OffsetFromPoint already reports a grapheme boundary chosen by proximity, so it is
			// used verbatim; rounding it down again here would discard the trailing-hit result.
			edit.caret = hit_context.shaper->OffsetFromPoint(font.family, font.size * scale, text, pt.x - origin_x, 0.0f);
			if (!extend)
				edit.selection_start = edit.caret;
		}
	}

	std::string DisplayTextOf(TextEditState const& edit)
	{
		if (edit.composition.active == 0)
			return edit.pending_text;

		auto const at = std::min<std::uint32_t>(edit.composition.insert_at, static_cast<std::uint32_t>(edit.pending_text.size()));
		auto display = edit.pending_text.substr(0, at);
		display += edit.composition.text;
		display += edit.pending_text.substr(at);
		return display;
	}

	TextEditRanges DisplayRangesOf(TextEditState const& edit)
	{
		if (edit.composition.active == 0)
		{
			return TextEditRanges{
				.caret = edit.caret,
				.selection_start = std::min(edit.caret, edit.selection_start),
				.selection_end = std::max(edit.caret, edit.selection_start),
				.composition_start = 0,
				.composition_length = 0,
			};
		}

		// While composing, the caret follows the IME's cursor inside the composition and the
		// selection is the IME's target clause, both reported against the spliced display string.
		auto const at = std::min<std::uint32_t>(edit.composition.insert_at, static_cast<std::uint32_t>(edit.pending_text.size()));
		return TextEditRanges{
			.caret = at + edit.composition.caret,
			.selection_start = at + edit.composition.sel_start,
			.selection_end = at + edit.composition.sel_end,
			.composition_start = at,
			.composition_length = static_cast<std::uint32_t>(edit.composition.text.size()),
		};
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

	InputResult ProcessNormalizedInput(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, NormalizedInput const& input, InputTextRecord const* text_payload, TextHitContext const& hit_context, InputState& state, EventQueue& events, std::uint64_t accepted_revision)
	{
		switch (input.kind)
		{
			case EInputKind::PointerMove:
			{
				auto hit = HitTest(tree, layout, Vec2{ input.pointer_x, input.pointer_y });
				auto changed = hit != state.m_hover_id;
				state.m_hover_id = hit;

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

				auto hit = HitTest(tree, layout, Vec2{ input.pointer_x, input.pointer_y });
				if (hit == 0)
				{
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
				// enclosing Button/TextBox ancestor (see NearestOfType); mutually exclusive because
				// a Button template never nests a TextBox part or vice versa in this milestone.
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
				return InputResult{ true, true };
			}
			case EInputKind::PointerButtonUp:
			{
				if (input.button != EPointerButton::Left || state.m_pressed_id == 0)
					return InputResult{ state.m_captured_id != 0, false };

				auto hit = HitTest(tree, layout, Vec2{ input.pointer_x, input.pointer_y });
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
				return InputResult{ false, false };
			}
			case EInputKind::KeyDown:
			{
				if (input.vk == VK_TAB)
				{
					// Moving focus away abandons any composition, exactly as a pointer press does.
					CancelActiveComposition(state);

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
					return InputResult{ false, false };

				auto const& node = tree.m_controls.at(state.m_focus_id);
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
				if (node.desc.type != EControlType::TextBox || node.desc.enabled == 0)
					return InputResult{ false, false };

				auto& edit = GetOrInitTextEdit(state, node);

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
						edit.caret = ctrl_held ? PrevWordBoundary(edit.pending_text, edit.caret) : PrevGraphemeBoundary(edit.pending_text, edit.caret);
						if (!shift_held)
							edit.selection_start = edit.caret;

						break;
					}
					case VK_RIGHT:
					{
						edit.caret = ctrl_held ? NextWordBoundary(edit.pending_text, edit.caret) : NextGraphemeBoundary(edit.pending_text, edit.caret);
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

	std::int32_t ReconcileFocusAfterTransaction(TreeModel const& new_tree, std::vector<ControlId> const& old_tab_order, InputState& state, EventQueue& events, std::uint64_t accepted_revision)
	{
		// Nothing was focused before this transaction, so there is nothing to preserve: this
		// function only recovers a previously-valid focus target that the transaction has just
		// invalidated (section 7.5), it must never invent an initial focus of its own.
		if (state.m_focus_id == 0)
			return 0;

		auto new_tab_order = ComputeTabOrder(new_tree);
		if (std::find(new_tab_order.begin(), new_tab_order.end(), state.m_focus_id) != new_tab_order.end())
			return 0; // still a valid focus target; nothing to reconcile

		// Focus is about to move, so any composition owned by the outgoing control is abandoned
		// before its edit state is reconciled below.
		CancelActiveComposition(state);

		ControlId next_focus = 0;
		if (!new_tab_order.empty())
		{
			auto old_it = std::find(old_tab_order.begin(), old_tab_order.end(), state.m_focus_id);
			auto old_index = old_it != old_tab_order.end() ? static_cast<std::size_t>(old_it - old_tab_order.begin()) : 0u;
			auto new_index = std::min(old_index, new_tab_order.size() - 1);
			next_focus = new_tab_order[new_index];
		}

		if (next_focus == state.m_focus_id)
			return 0;

		PushOrThrow(events, next_focus, EEventKind::FocusChanged, accepted_revision, 0, {});
		state.m_focus_id = next_focus;
		SeedFocusedTextEdit(new_tree, state, next_focus);
		return 1;
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
		switch (request.kind)
		{
			case ESemanticActionKind::Focus:
			{
				if (node.desc.enabled == 0 || node.desc.visible == 0 || node.desc.focusable == 0)
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
				if (node.desc.type != EControlType::TextBox || node.desc.enabled == 0)
					throw EngineException(EStatus::UnsupportedFeature, std::format("control {} does not support SetValue", request.control_id));

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
