//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M9 tests for the raw Win32 message translator (src/win32_input.h). TranslateWindowMessage is a
// pure function apart from reading the IME's composition strings, and ImmGetContext(nullptr)
// returns NULL, so the UTF-16, surrogate, dead-key and IME-lifecycle paths are all exercised here
// against a null HWND with no window, no message pump and no installed IME.
#include "pr/common/unittests.h"
#include "test_support.h"
#include "win32_input.h"
#include <string>

namespace pr::view3d::ui::tests
{
	namespace
	{
		// A 300x200 DIP viewport at 96 dpi, so client pixels and DIPs coincide and the coordinate
		// conversion cannot mask a translation error.
		ViewportState TranslatorViewport()
		{
			return Viewport(300, 200);
		}

		// Translate one message against a fresh translator state.
		TranslatedMessage TranslateOnce(UINT msg, WPARAM wparam, LPARAM lparam)
		{
			auto state = Win32InputTranslatorState{};
			return TranslateWindowMessage(nullptr, msg, wparam, lparam, TranslatorViewport(), 0.0, true, state);
		}
	}

	PRUnitTest(WmCharTranslatesABasicMultilingualPlaneCharacterToOneCharRecord, Quick)
	{
		// 'handled' is left to the state machine: with no focused text box the host must still let
		// its own window procedure see the character, so only the IME and WM_UNICHAR paths, whose
		// default handling would corrupt the input stream, force it here.
		auto const translated = TranslateOnce(WM_CHAR, 0x0041, 0);
		PR_EXPECT(translated.recognised != 0);
		PR_EXPECT(translated.handled == 0);
		PR_EXPECT(translated.inputs.size() == 1);
		PR_EXPECT(translated.inputs[0].input.kind == EInputKind::Char);
		PR_EXPECT(translated.inputs[0].input.char_code == 0x0041);
	}

	PRUnitTest(WmCharSurrogatePairsAreJoinedIntoOneAstralTextRecord, Quick)
	{
		// Windows delivers U+1F600 as two WM_CHARs. The high half alone must produce no record
		// (emitting it would corrupt the buffer with half a character), and the low half completes
		// the pair as a single Char record carrying the whole UTF-32 scalar.
		auto state = Win32InputTranslatorState{};
		auto const high = TranslateWindowMessage(nullptr, WM_CHAR, 0xD83D, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(high.recognised != 0);
		PR_EXPECT(high.handled != 0);
		PR_EXPECT(high.inputs.empty());
		PR_EXPECT(state.pending_high_surrogate == 0xD83D);

		auto const low = TranslateWindowMessage(nullptr, WM_CHAR, 0xDE00, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(low.inputs.size() == 1);
		PR_EXPECT(low.inputs[0].input.kind == EInputKind::Char);
		PR_EXPECT(low.inputs[0].input.char_code == 0x1F600);
		PR_EXPECT(state.pending_high_surrogate == 0);
	}

	PRUnitTest(AnUnpairedHighSurrogateIsDiscardedRatherThanEmittedAsText, Quick)
	{
		// A high surrogate followed by an ordinary character must not smuggle the orphan into the
		// buffer; the orphan is dropped and the ordinary character still arrives.
		auto state = Win32InputTranslatorState{};
		TranslateWindowMessage(nullptr, WM_CHAR, 0xD83D, 0, TranslatorViewport(), 0.0, true, state);
		auto const plain = TranslateWindowMessage(nullptr, WM_CHAR, 0x0042, 0, TranslatorViewport(), 0.0, true, state);

		PR_EXPECT(state.pending_high_surrogate == 0);
		PR_EXPECT(plain.inputs.size() == 1);
		PR_EXPECT(plain.inputs[0].input.kind == EInputKind::Char);
		PR_EXPECT(plain.inputs[0].input.char_code == 0x0042);

		// A low surrogate with no high half waiting is equally unencodable and is dropped rather
		// than inserted as a lone U+DCxx, which would make the buffer invalid UTF-8.
		auto const orphan_low = TranslateWindowMessage(nullptr, WM_CHAR, 0xDE00, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(orphan_low.recognised != 0);
		PR_EXPECT(orphan_low.inputs.empty());
	}

	PRUnitTest(WmUniCharAnswersTheNoCharProbeAndDeliversAstralScalarsDirectly, Quick)
	{
		// The UNICODE_NOCHAR probe must be answered TRUE, or the sender concludes the window is
		// not WM_UNICHAR-aware and falls back to WM_CHAR.
		auto const probe = TranslateOnce(WM_UNICHAR, UNICODE_NOCHAR, 0);
		PR_EXPECT(probe.recognised != 0);
		PR_EXPECT(probe.handled != 0);
		PR_EXPECT(probe.result == TRUE);
		PR_EXPECT(probe.inputs.empty());

		// WM_UNICHAR carries a whole UTF-32 scalar, so no surrogate pairing is involved. It is
		// always marked handled because DefWindowProc would otherwise re-post it as WM_CHAR and
		// the character would be inserted twice.
		auto const astral = TranslateOnce(WM_UNICHAR, 0x1F600, 0);
		PR_EXPECT(astral.handled != 0);
		PR_EXPECT(astral.inputs.size() == 1);
		PR_EXPECT(astral.inputs[0].input.kind == EInputKind::Char);
		PR_EXPECT(astral.inputs[0].input.char_code == 0x1F600);

		// A BMP scalar takes the same path; only its magnitude differs.
		auto const bmp = TranslateOnce(WM_UNICHAR, 0x00E9, 0);
		PR_EXPECT(bmp.inputs.size() == 1);
		PR_EXPECT(bmp.inputs[0].input.kind == EInputKind::Char);
		PR_EXPECT(bmp.inputs[0].input.char_code == 0x00E9);
	}

	PRUnitTest(WmUniCharRejectsSurrogateAndOutOfRangeScalarValues, Quick)
	{
		// A lone surrogate or an out-of-range scalar cannot be encoded, so nothing is emitted
		// rather than a replacement character being invented.
		for (auto wparam : { WPARAM(0xD83D), WPARAM(0xDE00), WPARAM(0x110000) })
		{
			auto const translated = TranslateOnce(WM_UNICHAR, wparam, 0);
			PR_EXPECT(translated.recognised != 0);
			PR_EXPECT(translated.inputs.empty());
		}
	}

	PRUnitTest(DeadKeyMessagesAreConsumedAndProduceNoTextOfTheirOwn, Quick)
	{
		// WM_DEADCHAR is the accent key itself; Windows delivers the composed character as a
		// later WM_CHAR. Emitting anything here would double-insert the accent.
		auto state = Win32InputTranslatorState{};
		auto const dead = TranslateWindowMessage(nullptr, WM_DEADCHAR, 0x00B4, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(dead.recognised != 0);
		PR_EXPECT(dead.handled != 0);
		PR_EXPECT(dead.inputs.empty());
		PR_EXPECT(state.pending_dead_key != 0);

		// The composed character then arrives as an ordinary WM_CHAR and clears the pending flag.
		auto const composed = TranslateWindowMessage(nullptr, WM_CHAR, 0x00E9, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(composed.inputs.size() == 1);
		PR_EXPECT(composed.inputs[0].input.char_code == 0x00E9);
		PR_EXPECT(state.pending_dead_key == 0);

		// WM_SYSDEADCHAR behaves identically.
		auto sys_state = Win32InputTranslatorState{};
		auto const sys_dead = TranslateWindowMessage(nullptr, WM_SYSDEADCHAR, 0x00B4, 0, TranslatorViewport(), 0.0, true, sys_state);
		PR_EXPECT(sys_dead.recognised != 0);
		PR_EXPECT(sys_dead.inputs.empty());
	}

	PRUnitTest(ImeStartCompositionOpensACompositionAndAsksForWindowPlacement, Quick)
	{
		auto state = Win32InputTranslatorState{};
		auto const started = TranslateWindowMessage(nullptr, WM_IME_STARTCOMPOSITION, 0, 0, TranslatorViewport(), 0.0, true, state);

		PR_EXPECT(started.recognised != 0);
		PR_EXPECT(started.handled != 0);
		PR_EXPECT(state.composition_active != 0);
		PR_EXPECT(started.place_ime_windows != 0);
		PR_EXPECT(started.inputs.size() == 1);
		PR_EXPECT(started.inputs[0].input.kind == EInputKind::CompositionStart);
		PR_EXPECT(started.inputs[0].text.empty());
	}

	PRUnitTest(ImeEndCompositionCancelsAnyStillLiveCompositionExactlyOnce, Quick)
	{
		// A composition that ends without a result string must be cancelled, not committed, or a
		// half-typed draft would become durable text.
		auto state = Win32InputTranslatorState{};
		TranslateWindowMessage(nullptr, WM_IME_STARTCOMPOSITION, 0, 0, TranslatorViewport(), 0.0, true, state);

		auto const ended = TranslateWindowMessage(nullptr, WM_IME_ENDCOMPOSITION, 0, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(ended.recognised != 0);
		PR_EXPECT(state.composition_active == 0);
		PR_EXPECT(ended.inputs.size() == 1);
		PR_EXPECT(ended.inputs[0].input.kind == EInputKind::CompositionCancel);

		// A second end with nothing live emits nothing, so a duplicate message cannot cancel a
		// composition that a later start has since opened.
		auto const again = TranslateWindowMessage(nullptr, WM_IME_ENDCOMPOSITION, 0, 0, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(again.recognised != 0);
		PR_EXPECT(again.inputs.empty());
	}

	PRUnitTest(ImeCompositionWithoutAnImeContextYieldsNoRecords, Quick)
	{
		// With a null HWND there is no IMM context to read, so the translator must report the
		// message as recognised/handled but produce no records rather than fabricating text.
		auto state = Win32InputTranslatorState{};
		TranslateWindowMessage(nullptr, WM_IME_STARTCOMPOSITION, 0, 0, TranslatorViewport(), 0.0, true, state);

		auto const composing = TranslateWindowMessage(nullptr, WM_IME_COMPOSITION, 0, GCS_COMPSTR, TranslatorViewport(), 0.0, true, state);
		PR_EXPECT(composing.recognised != 0);
		PR_EXPECT(composing.handled != 0);
		PR_EXPECT(composing.inputs.empty());
		PR_EXPECT(state.composition_active != 0);
	}

	PRUnitTest(UnrelatedMessagesAreNotRecognisedAndAreLeftToTheWindowProcedure, Quick)
	{
		for (auto msg : { UINT(WM_PAINT), UINT(WM_TIMER), UINT(WM_ERASEBKGND) })
		{
			auto const translated = TranslateOnce(msg, 0, 0);
			PR_EXPECT(translated.recognised == 0);
			PR_EXPECT(translated.handled == 0);
			PR_EXPECT(translated.inputs.empty());
		}
	}

	PRUnitTest(PointerMessagesStillTranslateClientPixelsIntoViewportDips, Quick)
	{
		// The pointer path is pre-M9 behaviour, retained here because M9 rewrote the translator's
		// return shape and a regression would silently break every pointer test's premise.
		auto const moved = TranslateOnce(WM_MOUSEMOVE, 0, MAKELPARAM(25, 40));
		PR_EXPECT(moved.recognised != 0);
		PR_EXPECT(moved.inputs.size() == 1);
		PR_EXPECT(moved.inputs[0].input.kind == EInputKind::PointerMove);
		PR_EXPECT(moved.inputs[0].input.pointer_x == 25.0f);
		PR_EXPECT(moved.inputs[0].input.pointer_y == 40.0f);

		auto const pressed = TranslateOnce(WM_LBUTTONDOWN, MK_LBUTTON, MAKELPARAM(10, 12));
		PR_EXPECT(pressed.inputs.size() == 1);
		PR_EXPECT(pressed.inputs[0].input.kind == EInputKind::PointerButtonDown);
		PR_EXPECT(pressed.inputs[0].input.button == EPointerButton::Left);
	}

	PRUnitTest(PlacingImeWindowsWithoutAWindowIsAHarmlessNoOp, Quick)
	{
		// The side-effecting half of the IME path must tolerate the absence of a window, so a
		// headless test run never faults inside IMM.
		Win32PlaceImeWindows(nullptr, TranslatorViewport(), Rect{ .x = 10.0f, .y = 40.0f, .w = 1.0f, .h = 16.0f });
	}
}
