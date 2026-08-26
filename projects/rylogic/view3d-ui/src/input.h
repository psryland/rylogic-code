//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Normalized input state machine (implementation-plan.md section 7): pointer hit-testing,
// capture, focus, Tab/Shift+Tab focus traversal, Button mouse/Enter/Space activation, and TextBox
// editing including IME composition and clipboard. Raw Win32 messages and deterministic test
// injection both funnel through ProcessNormalizedInput so their behaviour is identical.
//
// All editing here moves in Unicode grapheme clusters (see text_unicode.h), never in UTF-8 bytes
// or bare scalar values, so a caret or selection edge can never land inside a surrogate pair, a
// combining sequence, an emoji ZWJ sequence, a regional-indicator pair or a CRLF break.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "tree.h"
#include "events.h"
#include "world.h"

namespace pr::view3d::ui
{
	class TextShaper;

	// The transient state of one in-progress IME composition. Everything here is owned by this
	// module and is never part of the application's durable text: 'text' is spliced into the
	// display string at 'insert_at' for rendering and semantics but is deliberately kept out of
	// TextEditState::pending_text, which is what guarantees no TextChangeProposed event can escape
	// before the IME produces a result string. The saved_* fields capture the pending edit exactly
	// as it stood when composition began, so a cancellation restores it byte for byte.
	struct CompositionState
	{
		std::string text;                 // UTF-8 composition string as last reported by the IME
		std::uint32_t caret = 0;          // byte offset of the IME's cursor within 'text'
		std::uint32_t sel_start = 0;      // byte offsets of the IME's converted/target clause within 'text'
		std::uint32_t sel_end = 0;
		std::uint32_t insert_at = 0;      // byte offset in pending_text where 'text' is displayed
		std::string saved_pending_text;
		std::uint32_t saved_caret = 0;
		std::uint32_t saved_selection_start = 0;
		std::uint32_t saved_edit_generation = 0;
		std::int32_t active = 0;
	};

	// Live editing state for one focused-or-previously-focused TextBox. 'pending_text' is the
	// locally-edited buffer shown to the user before the application commits it back via a
	// transaction; 'caret'/'selection_start' are UTF-8 byte offsets into 'pending_text' and are
	// always on grapheme-cluster boundaries. 'last_accepted_text' is the control's descriptor text
	// as observed at the most recent seed/reconciliation point, used by
	// ReconcileTextEditsAfterTransaction to distinguish the application echoing back an outstanding
	// proposal from the application changing the text unilaterally (managed normalization/
	// rejection). 'edit_generation' is zero while 'pending_text' has no outstanding local edit
	// relative to the last accepted transaction, and is a monotonically increasing nonzero value
	// for each locally-edited proposal in flight.
	struct TextEditState
	{
		std::string pending_text;
		std::string last_accepted_text;
		std::uint32_t caret = 0;
		std::uint32_t selection_start = 0;
		std::uint32_t edit_generation = 0;
		std::int32_t initialized = 0; // 0 until pending_text has been seeded from the accepted tree
		CompositionState composition;
	};

	// The text a control currently displays: its pending edit with any active composition spliced
	// in. Rendering, semantics and hit testing all measure this exact string, which is what stops
	// the caret, the selection highlight and the reported text ranges from disagreeing.
	std::string DisplayTextOf(TextEditState const& edit);

	// Caret/selection/composition offsets expressed as UTF-8 byte offsets into DisplayTextOf(),
	// so a consumer never has to know whether a composition is in progress to interpret them.
	// 'composition_length' is 0 when no composition is active.
	struct TextEditRanges
	{
		std::uint32_t caret;
		std::uint32_t selection_start;
		std::uint32_t selection_end;
		std::uint32_t composition_start;
		std::uint32_t composition_length;
	};
	TextEditRanges DisplayRangesOf(TextEditState const& edit);

	// All input focus/capture/editing state that persists across ProcessNormalizedInput calls.
	// Bounded by the live control count via Prune(), never grows without bound.
	class InputState
	{
	public:
		ControlId m_hover_id = 0;
		ControlId m_pressed_id = 0;
		ControlId m_captured_id = 0;
		ControlId m_focus_id = 0;
		ControlId m_composing_id = 0; // the one control with an active composition, or 0
		std::unordered_map<ControlId, TextEditState> m_text_edits;

		// Discard hover/pressed/capture/focus targets and text-edit state for controls no longer
		// present in the accepted tree.
		void Prune(std::unordered_set<ControlId> const& live_ids);
	};

	// Deterministic pre-order Tab-traversal candidates: focusable, enabled, visible controls.
	std::vector<ControlId> ComputeTabOrder(TreeModel const& tree);

	// The topmost visible, interactive (Button/TextBox) control whose bounds contain 'pt', or 0 if
	// none (an "outside" hit). Root/Panel/Text are hit-test transparent: the search still descends
	// through them to find an interactive descendant, but their own bounds are never returned as a
	// hit, so a click over layout/decoration area with no interactive descendant is a miss.
	ControlId HitTest(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, Vec2 pt);

	// Borrowed variable-length text accompanying one normalized input record. The caller owns the
	// storage and it is never retained past the ProcessNormalizedInput call. 'caret'/'selection_*'
	// are UTF-8 byte offsets within 'text' and are only meaningful for composition records.
	struct InputTextRecord
	{
		std::string_view text;
		std::uint32_t caret;
		std::uint32_t selection_start;
		std::uint32_t selection_end;
	};

	// Everything beyond the tree and layout that is needed to place a caret from a pointer
	// position. Both members may be null, in which case pointer caret placement degrades
	// deterministically to the end of the text rather than failing the input.
	struct TextHitContext
	{
		TextShaper* shaper = nullptr;
		std::unordered_map<ControlId, RootPlacement> const* placements = nullptr;
	};

	// Whether 'kind' must be accompanied by an InputTextRecord. Pure, so the ABI layer and the
	// state machine agree on which records carry text without duplicating the list.
	bool InputKindCarriesText(EInputKind kind);

	// Result of processing one normalized input record.
	struct InputResult
	{
		bool consumed;
		bool invalidate;
	};

	// Apply one normalized input record to the shared state machine, hit-testing/focus-traversing/
	// editing against 'tree'+'layout' (the most recently computed layout), enqueueing any resulting
	// typed events into 'events'. 'text_payload' carries the borrowed text for the TextInput and
	// Composition* record kinds and must be null for every other kind. Returns whether the input
	// was consumed by the UI and whether the caller should treat the view as needing to be
	// redrawn/re-queried. Throws EngineException for a malformed record (invalid UTF-8, offsets
	// that are not on code-point boundaries, a composition record that does not follow the
	// start/update/commit-or-cancel ordering, or a composition record naming a control that is no
	// longer the composing one).
	InputResult ProcessNormalizedInput(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, NormalizedInput const& input, InputTextRecord const* text_payload, TextHitContext const& hit_context, InputState& state, EventQueue& events, std::uint64_t accepted_revision);

	// Reconcile keyboard focus after a transaction changes the accepted tree (section 7.5): if the
	// currently focused control is no longer a valid focus target in 'new_tree' (removed, hidden,
	// disabled, or made unfocusable), focus moves to the next eligible control at or after its
	// former position in 'old_tab_order' (clamped into range as the tree may have shrunk), or is
	// cleared entirely if no eligible control remains. 'old_tab_order' must be ComputeTabOrder(tree)
	// captured against the tree as it stood immediately before the transaction was applied. Returns
	// whether focus was altered; emits FocusChanged when it was.
	std::int32_t ReconcileFocusAfterTransaction(TreeModel const& new_tree, std::vector<ControlId> const& old_tab_order, InputState& state, EventQueue& events, std::uint64_t accepted_revision);

	// Reconcile every tracked TextEditState against 'new_tree' after a transaction is accepted:
	// - descriptor text == pending text: the application echoed back the outstanding proposal (or
	//   there was none), so it is acknowledged - 'last_accepted_text' advances and the proposal is
	//   no longer outstanding (edit_generation resets to zero).
	// - descriptor text != last_accepted_text (and != pending text): the application changed the
	//   text unilaterally (managed normalization/rejection of whatever was proposed), so the local
	//   edit is replaced deterministically with the descriptor text and the caret/selection collapse
	//   to its end; the discarded local proposal is no longer outstanding.
	// - otherwise (descriptor text == last_accepted_text but != pending text): the application made
	//   an unrelated change and the outstanding local proposal is preserved untouched.
	// Entries for controls no longer present in 'new_tree', or no longer of type TextBox, are left
	// for InputState::Prune to discard and are not reconciled here.
	void ReconcileTextEditsAfterTransaction(TreeModel const& new_tree, InputState& state);

	// Cancel any active composition, restoring the pending edit exactly as it stood when the
	// composition began. Used when focus is lost or the composing control leaves the tree, so a
	// composition can never outlive the control or the focus it belongs to.
	void CancelActiveComposition(InputState& state);

	// True when the focused control is an enabled TextBox, and therefore a valid target for text
	// input, composition or caret geometry. The raw-message path uses this to decide whether IME
	// messages should become input records at all, so an IME activated over a non-editable UI is
	// left to the host's window procedure rather than being swallowed.
	bool HasEditableFocus(TreeModel const& tree, InputState const& state);

	// Closed vocabulary of the actions an accessibility client can request against a semantic
	// node. These mirror ESemanticAction, which is what a semantic snapshot advertises per node,
	// and they are deliberately the only way anything other than normalized input can drive the
	// state machine (section 5.5/7).
	enum class ESemanticActionKind
	{
		Focus,
		Invoke,
		SetValue,
		SetSelection,
	};

	// One semantic action request. 'text' is the replacement UTF-8 value for SetValue and is
	// ignored otherwise. 'selection_start'/'selection_end' are UTF-8 byte offsets into the target
	// control's current pending text for SetSelection and are ignored otherwise; they need not be
	// ordered or grapheme-aligned because SetSelection snaps them.
	struct SemanticActionRequest
	{
		ESemanticActionKind kind;
		ControlId control_id;
		std::string text;
		std::uint32_t selection_start;
		std::uint32_t selection_end;
	};

	// Apply one semantic action to the same state machine, focus rules and event queue normalized
	// input uses, so an accessibility client and a keystroke are indistinguishable downstream. No
	// application or managed code is called. Throws EngineException(InvalidArgument) for an
	// unknown control or malformed text, and EngineException(UnsupportedFeature) when the target
	// does not currently advertise the requested action (disabled, wrong control type, not
	// focusable, or composing). Returns whether the action was consumed and whether the view needs
	// redrawing.
	InputResult ApplySemanticAction(TreeModel const& tree, SemanticActionRequest const& request, InputState& state, EventQueue& events, std::uint64_t accepted_revision);
}
