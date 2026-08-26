//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Raw Win32 message -> NormalizedInput translation (implementation-plan.md section 7.1/7.4). This
// is the only place in the module that reads Win32 window-message wire formats; everything
// downstream (ProcessNormalizedInput) is identical for the production message path and
// deterministic test injection.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// Persistent state the raw-message translator needs across calls, distinct from the control
	// state machine's own InputState (section 7.1: "WM_CHAR/UTF-16 surrogate handling").
	struct Win32InputTranslatorState
	{
		std::uint16_t pending_high_surrogate = 0; // high half of a WM_CHAR surrogate pair awaiting its low half
		std::int32_t pending_dead_key = 0;        // a WM_DEADCHAR was consumed; the next WM_CHAR carries the composed character
		std::int32_t composition_active = 0;      // the engine has a live composition; the caller resyncs this from InputState before every message
		std::int32_t ime_composition_open = 0;    // the OS IME is between WM_IME_STARTCOMPOSITION and WM_IME_ENDCOMPOSITION
	};

	// One normalized record produced by translating a message, with the borrowed text it needs.
	// 'text' is owned by the TranslatedMessage that carries it, so a caller can hand a
	// string_view over it straight to ProcessNormalizedInput. 'caret'/'selection_*' are UTF-8 byte
	// offsets within 'text' and are only meaningful for composition-update records.
	struct TranslatedInput
	{
		NormalizedInput input;
		std::string text;
		std::uint32_t caret;
		std::uint32_t selection_start;
		std::uint32_t selection_end;
	};

	// Result of translating one raw Win32 window message. A single message can imply more than one
	// normalized record - notably WM_IME_COMPOSITION, which may carry a result string (commit) and
	// a new composition string in the same message - so records are returned as an ordered list
	// that the caller applies in sequence.
	struct TranslatedMessage
	{
		std::int32_t recognised;        // 0/1: whether this module participates in the message at all
		std::int32_t handled;           // 0/1: whether the caller must stop the default window procedure seeing it
		LRESULT result;                 // the value to return from the window procedure when 'handled'
		std::int32_t place_ime_windows; // 0/1: the caller should reposition the IME composition/candidate windows
		std::vector<TranslatedInput> inputs;
	};

	// Translate one raw HWND message into the normalized input records consumed by
	// ProcessNormalizedInput, converting physical client pixel coordinates into DIPs via
	// 'viewport' (section 7.4: client px -> render-target px by size ratio, then -> DIPs by
	// 96/dpi). 'editable_focus' tells the translator whether the engine currently has an enabled
	// TextBox focused: IME messages produce records, and are marked handled, only when it does, so
	// an IME activated over a non-editable UI is left to the default window procedure instead of
	// being swallowed as successful input. Messages this module does not participate in (WM_PAINT,
	// etc.) yield recognised == 0 and the caller must let the rest of the window procedure handle
	// them unmodified. Aside from reading the IME's composition strings for the message being
	// translated, this function has no Win32 side effects of its own (no SetCapture/clipboard/
	// IME-window calls), so deterministic test injection can exercise the identical control-state
	// machine without a real window (section 7.2).
	TranslatedMessage TranslateWindowMessage(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, ViewportState const& viewport, double time_ms, bool editable_focus, Win32InputTranslatorState& translator_state);

	// Ask the OS IME to abandon its current composition. Called when engine-side interaction - a
	// click elsewhere, Tab, or focus loss - has already cancelled the composition this module was
	// showing, so the IME does not keep converting into a composition nothing is displaying.
	void Win32CancelImeComposition(HWND hwnd);

	// Apply the Win32 SetCapture/ReleaseCapture side effect implied by a pointer-capture-id
	// transition. The caller invokes this itself after ProcessNormalizedInput reports the new
	// InputState::m_captured_id, so real OS capture is never touched by translation/injection
	// alone (section 7.2's "injection does not call Win32 capture ... services").
	void Win32ApplyCaptureTransition(HWND hwnd, ControlId previous_captured_id, ControlId current_captured_id);

	// Move the IME's composition and candidate windows to the caret, so conversion candidates
	// appear against the text being typed rather than at the window origin. 'caret_dip' is the
	// caret rectangle in DIPs within the render target, which is converted back to client pixels
	// here. Like Win32ApplyCaptureTransition this is a deliberate Win32 side effect kept out of
	// the pure translator, so injection never touches a real IME.
	void Win32PlaceImeWindows(HWND hwnd, ViewportState const& viewport, Rect const& caret_dip);
}
