//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M9 corrective-pass tests. These cover the seams a review found unproven: the end-to-end
// UiEngine::ProcessWindowMessage path where the raw Win32 translator and the control state machine
// must agree about whether a composition is live, RTL glyph placement against DirectWrite's own
// geometry, caret-only keyboard actions that must invalidate without proposing a text change,
// grapheme-safe pointer selection, composition length limits, semantic offset normalisation, caret
// geometry for an empty field and the bounded font caches. Nothing here needs a window, an
// installed IME or a real GPU device.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "test_support.h"
#include "events.h"
#include "input.h"
#include "text_shaper.h"
#include "text_unicode.h"
#include "tree.h"
#include "win32_input.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace pr::view3d::ui::tests
{
	namespace
	{
		auto const kShift = static_cast<std::uint32_t>(EInputModifier::Shift);
		auto const kCtrl = static_cast<std::uint32_t>(EInputModifier::Ctrl);
		auto const kFamily = std::string_view("Segoe UI");
		auto const kSize = 14.0f;

		// A 300x200 root (id 1) with a focusable TextBox (id 2) at [10,40)-[210,60) and a Button
		// (id 3) at [10,80)-[110,100), so a test can move focus onto and away from an editable
		// target. The TextBox is seeded with 'initial_text'; 'max_text_length' of 0 is unbounded.
		void Scene(UiEngine& engine, std::string_view initial_text, std::uint32_t max_text_length = 0)
		{
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

			auto text_box = MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(200.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 40.0f;
			text_box.max_text_length = max_text_length;
			std::tie(text_box.text_offset, text_box.text_length) = b.AddText(initial_text);
			b.Upsert(text_box);

			auto button = MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			button.layout.margin_left = 10.0f;
			button.layout.margin_top = 80.0f;
			b.Upsert(button);

			engine.TransactionApply(b.Build(0, 1));
			engine.Update(Viewport(300, 200));
		}

		// The semantic node reported for 'id', together with the snapshot's text blob.
		struct SemanticView
		{
			SemanticNode node;
			std::string blob;

			std::string Value() const
			{
				return blob.substr(node.value_offset, node.value_length);
			}
		};
		SemanticView SemanticOf(UiEngine& engine, ControlId id)
		{
			engine.Update(Viewport(300, 200));
			auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
			auto blob = std::vector<char>(engine.SemanticTextBytesPending());
			engine.SemanticsCopy(nodes, blob);

			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			if (it == nodes.end())
				throw std::runtime_error("test helper: semantic node not found for the requested control id");

			return SemanticView{ .node = *it, .blob = std::string(blob.data(), blob.size()) };
		}

		// Drains the queue and reports whether it contained a TextChangeProposed for 'id'.
		bool AnyProposal(UiEngine& engine, ControlId id)
		{
			auto events = std::vector<Event>(engine.EventCount());
			auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
			engine.EventsCopy(events, payload);

			return std::any_of(events.begin(), events.end(), [id](Event const& e) { return e.kind == EEventKind::TextChangeProposed && e.control_id == id; });
		}

		// Focus the TextBox with Tab, then drop the events that produced.
		void FocusTextBox(UiEngine& engine)
		{
			engine.InputInject(KeyDownInput(VK_TAB));
			AnyProposal(engine, 2);
		}

		// What the engine did with one window message.
		struct MessageOutcome
		{
			std::int32_t used;
			LRESULT result;
			std::int32_t invalidate;
		};
		MessageOutcome Send(UiEngine& engine, UINT msg, WPARAM wparam = 0, LPARAM lparam = 0)
		{
			LRESULT result = 0;
			std::int32_t invalidate = 0;
			auto const used = engine.ProcessWindowMessage(nullptr, msg, wparam, lparam, result, invalidate);
			return MessageOutcome{ .used = used, .result = result, .invalidate = invalidate };
		}

		// A minimal hand-built tree with one focused enabled TextBox, driven through the free
		// ProcessNormalizedInput so a test can see the InputResult (consumed/invalidate) that the
		// engine facade folds away into a single redraw request.
		struct DirectInput
		{
			TreeModel tree;
			std::unordered_map<ControlId, Rect> layout;
			InputState state;
			EventQueue events;

			explicit DirectInput(std::string_view text)
				: tree()
				, layout()
				, state()
				, events(64)
			{
				auto const desc = MakeControl(2, 0, EControlType::TextBox, ELayoutMode::Overlay, Lp(200.0f, 20.0f));
				tree.m_revision = 1;
				tree.m_controls[2] = ControlNode{ .desc = desc, .text = std::string(text), .name = {}, .description = {}, .children = {} };
				layout[2] = Rect{ .x = 10.0f, .y = 40.0f, .w = 200.0f, .h = 20.0f };
				state.m_focus_id = 2;
			}

			InputResult Send(NormalizedInput const& input)
			{
				auto const hit = TextHitContext{ .shaper = nullptr, .placements = nullptr };
				return ProcessNormalizedInput(tree, layout, input, nullptr, hit, state, events, tree.m_revision);
			}

			// True when a TextChangeProposed is queued; drains the queue either way.
			bool AnyProposal()
			{
				auto drained = std::vector<Event>(events.Count());
				auto payload = std::vector<std::byte>(events.PayloadBytesPending());
				events.Copy(drained, payload);

				return std::any_of(drained.begin(), drained.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed; });
			}
		};
	}

	PRUnitTest(ImeMessagesWithNoEditableFocusAreNeitherRejectedNorSwallowed, Quick)
	{
		// B1/N12: an IME activated while a Button (or nothing) has focus used to be reported as
		// handled UI input, and its follow-up messages then reached a state machine with no
		// composition to update, surfacing as an ABI InvalidArgument. Neither may happen: with no
		// editable target the messages must be left to the host's window procedure.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "abc");

		for (auto msg : { UINT(WM_IME_STARTCOMPOSITION), UINT(WM_IME_COMPOSITION), UINT(WM_IME_ENDCOMPOSITION) })
		{
			auto const outcome = Send(engine, msg, 0, GCS_COMPSTR);
			PR_EXPECT(outcome.used == 0);
			PR_EXPECT(outcome.invalidate == 0);
		}

		// Focusing the Button rather than the TextBox is the same situation.
		engine.InputInject(KeyDownInput(VK_TAB));
		engine.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(Send(engine, WM_IME_STARTCOMPOSITION).used == 0);
		PR_EXPECT(Send(engine, WM_IME_COMPOSITION, 0, GCS_RESULTSTR).used == 0);
	}

	PRUnitTest(ClickingAwayDuringACompositionCancelsItWithoutAnAbiFailure, Quick)
	{
		// B1: engine-side interaction can end a composition the OS IME still believes is running.
		// The follow-up IME message must resynchronise instead of being rejected as out of order.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "abc");
		FocusTextBox(engine);

		auto const composition = std::string("ka");
		engine.InputInject(TextInputRecord(EInputKind::CompositionStart));
		engine.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(composition, 2, 0, 2));
		PR_EXPECT(SemanticOf(engine, 2).Value() == "abcka");

		// Pressing the pointer on the Button moves focus off the TextBox, which cancels.
		engine.InputInject(PointerDownInput(40.0f, 90.0f));
		PR_EXPECT(SemanticOf(engine, 2).Value() == "abc");

		// A stale WM_IME_COMPOSITION now arrives. It must not throw, and with no editable focus it
		// must not be claimed as consumed UI input either.
		PR_EXPECT(Send(engine, WM_IME_COMPOSITION, 0, GCS_COMPSTR).used == 0);
	}

	PRUnitTest(TabbingAwayDuringACompositionLeavesTheEngineAndTranslatorInStep, Quick)
	{
		// B1: Tab cancels the composition inside the state machine. Whatever IME message arrives
		// next must find the translator's mirror of that state already corrected.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "xy");
		FocusTextBox(engine);

		auto const composition = std::string("zz");
		engine.InputInject(TextInputRecord(EInputKind::CompositionStart));
		engine.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(composition, 2, 0, 2));
		engine.InputInject(KeyDownInput(VK_TAB));

		// The pending text is exactly what it was before the composition began.
		PR_EXPECT(SemanticOf(engine, 2).Value() == "xy");

		// Messages that follow are handled without throwing, whichever order they arrive in.
		PR_EXPECT(Send(engine, WM_IME_COMPOSITION, 0, GCS_RESULTSTR).used == 0);
		PR_EXPECT(Send(engine, WM_IME_ENDCOMPOSITION).used == 0);
	}

	PRUnitTest(FocusLossDuringACompositionRestoresThePendingEditExactly, Quick)
	{
		// B1: WM_KILLFOCUS both cancels the composition and clears the translator's surrogate,
		// dead-key and composition state, so a window that regains focus starts clean.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "seed");
		FocusTextBox(engine);

		auto const composition = std::string("ni");
		engine.InputInject(TextInputRecord(EInputKind::CompositionStart));
		engine.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(composition, 2, 0, 2));
		PR_EXPECT(SemanticOf(engine, 2).Value() == "seedni");

		Send(engine, WM_KILLFOCUS);
		PR_EXPECT(SemanticOf(engine, 2).Value() == "seed");

		// No committed change was ever proposed, since the composition produced no result string.
		PR_EXPECT(!AnyProposal(engine, 2));

		// A stale composition message after focus loss is inert rather than fatal.
		PR_EXPECT(Send(engine, WM_IME_COMPOSITION, 0, GCS_COMPSTR).used == 0);
	}

	PRUnitTest(ImeCharAndSetContextAreRecognisedWithoutDuplicatingText, Quick)
	{
		// N3: WM_IME_CHAR repeats characters the result string already delivered, so it must be
		// consumed silently; WM_IME_SETCONTEXT must be forwarded with the composition-window bit
		// cleared because this module draws the composition itself.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "");
		FocusTextBox(engine);

		auto state = Win32InputTranslatorState{};
		auto const ime_char = TranslateWindowMessage(nullptr, WM_IME_CHAR, 0x4E2D, 0, Viewport(300, 200), 0.0, true, state);
		PR_EXPECT(ime_char.recognised != 0);
		PR_EXPECT(ime_char.handled != 0);
		PR_EXPECT(ime_char.inputs.empty());

		auto const set_context = TranslateWindowMessage(nullptr, WM_IME_SETCONTEXT, TRUE, ISC_SHOWUIALL, Viewport(300, 200), 0.0, true, state);
		PR_EXPECT(set_context.recognised != 0);
		PR_EXPECT(set_context.handled != 0);
		PR_EXPECT(set_context.inputs.empty());

		// End to end, the message is used and nothing was inserted into the field.
		PR_EXPECT(Send(engine, WM_IME_CHAR, 0x4E2D, 0).used != 0);
		PR_EXPECT(SemanticOf(engine, 2).Value().empty());
	}

	PRUnitTest(TheUniCharNoCharProbeStillAnswersTrueAfterTheFocusGating, Quick)
	{
		// N12 gated the IME messages on editable focus; WM_UNICHAR must keep its own semantics, or
		// senders conclude the window is not UTF-32 aware and fall back to WM_CHAR.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "");

		auto const probe = Send(engine, WM_UNICHAR, UNICODE_NOCHAR, 0);
		PR_EXPECT(probe.used != 0);
		PR_EXPECT(probe.result == TRUE);

		// The probe is answered whether or not anything editable has focus.
		FocusTextBox(engine);
		PR_EXPECT(Send(engine, WM_UNICHAR, UNICODE_NOCHAR, 0).result == TRUE);

		// A real astral scalar still arrives as one grapheme cluster.
		Send(engine, WM_UNICHAR, 0x1F600, 0);
		PR_EXPECT(SemanticOf(engine, 2).Value() == "\U0001F600");
	}

	PRUnitTest(ControlCharactersFromTheKeyboardNeverEnterTheEditBuffer, Quick)
	{
		// N4: WM_CHAR delivers Escape, Backspace and Ctrl+letter as C0 controls, and some layouts
		// can produce DEL or a C1 code. None of them are text.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "");
		FocusTextBox(engine);

		for (auto code : { WPARAM(0x1B), WPARAM(0x08), WPARAM(0x01), WPARAM(0x7F), WPARAM(0x85), WPARAM(0x9F) })
			Send(engine, WM_CHAR, code, 0);

		PR_EXPECT(SemanticOf(engine, 2).Value().empty());
		PR_EXPECT(!AnyProposal(engine, 2));

		// Printable characters are unaffected, including the NBSP an AltGr layout can produce.
		Send(engine, WM_CHAR, WPARAM(0x0041), 0);
		Send(engine, WM_CHAR, WPARAM(0x00A0), 0);
		PR_EXPECT(SemanticOf(engine, 2).Value() == "A\u00A0");
	}

	PRUnitTest(CaretOnlyKeystrokesInvalidateWithoutProposingATextChange, Quick)
	{
		// B5: moving or selecting must repaint, because the caret and selection are drawn, but must
		// never look like an edit to the application.
		auto direct = DirectInput("hello brave world");

		struct Probe
		{
			std::int32_t key;
			std::uint32_t modifiers;
		};
		Probe const probes[] = {
			{ VK_HOME, 0 },
			{ VK_END, 0 },
			{ VK_HOME, 0 },
			{ VK_RIGHT, 0 },
			{ VK_RIGHT, kShift },
			{ VK_LEFT, kShift },
			{ VK_RIGHT, kCtrl },
			{ VK_RIGHT, kCtrl | kShift },
			{ VK_LEFT, kCtrl },
			{ VK_LEFT, kCtrl | kShift },
			{ VK_END, kShift },
			{ VK_HOME, kShift },
			{ 'A', kCtrl },
		};
		for (auto const& probe : probes)
		{
			auto const result = direct.Send(KeyDownInput(probe.key, probe.modifiers));
			PR_EXPECT(result.consumed);
			PR_EXPECT(result.invalidate);
			PR_EXPECT(!direct.AnyProposal());
		}

		// Copy neither moves the caret nor changes the text, so it is consumed without proposing
		// anything; it is listed separately because it has no reason to request a redraw.
		auto const copied = direct.Send(KeyDownInput('C', kCtrl));
		PR_EXPECT(copied.consumed);
		PR_EXPECT(!direct.AnyProposal());

		// The pending text is untouched throughout, which is what makes the absence of a proposal
		// correct rather than merely missing.
		PR_EXPECT(direct.state.m_text_edits[2].pending_text == "hello brave world");

		// An actual edit both invalidates and proposes, so the assertions above distinguish the two
		// cases rather than passing vacuously.
		auto const edited = direct.Send(KeyDownInput(VK_BACK));
		PR_EXPECT(edited.consumed);
		PR_EXPECT(edited.invalidate);
		PR_EXPECT(direct.AnyProposal());
	}

	PRUnitTest(CaretMovementThroughTheWindowMessagePathAlsoRequestsARedraw, Quick)
	{
		// B5, end to end: the same requirement seen through ProcessWindowMessage, which is the path
		// a real host uses and the only one that reports 'invalidate' to it.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "hello");
		FocusTextBox(engine);
		AnyProposal(engine, 2);

		Send(engine, WM_KEYDOWN, VK_END, 0);
		auto const moved = Send(engine, WM_KEYDOWN, VK_LEFT, 0);
		PR_EXPECT(moved.used != 0);
		PR_EXPECT(moved.invalidate != 0);
		PR_EXPECT(!AnyProposal(engine, 2));
		PR_EXPECT(SemanticOf(engine, 2).Value() == "hello");
	}

	PRUnitTest(RightToLeftGlyphOriginsAgreeWithDirectWritesOwnGeometry, Quick)
	{
		// B4: an odd bidi level means the run's baseline origin is its right edge and glyphs walk
		// leftwards from it. Ignoring that mirrored every RTL run, so shaped glyph positions
		// disagreed with the caret and selection rectangles taken from the same layout.
		auto shaper = TextShaper{};
		auto const hebrew = std::string("\u05E9\u05DC\u05D5\u05DD");

		auto glyphs = std::vector<ShapedGlyph>{};
		auto const width = shaper.Shape(kFamily, kSize, 1.0f, hebrew, glyphs);
		PR_EXPECT(!glyphs.empty());
		PR_EXPECT(width > 0.0f);

		// Every glyph in a pure RTL run reports an odd level and sits inside the measured run.
		for (auto const& glyph : glyphs)
		{
			PR_EXPECT((glyph.bidi_level % 2) == 1);
			PR_EXPECT(glyph.origin_x >= -1.0f);
			PR_EXPECT(glyph.origin_x <= width + 1.0f);
		}

		// The first logical glyph must be the rightmost, which is what walking the run backwards
		// produces and what the mirrored version got wrong.
		auto const rightmost = std::max_element(glyphs.begin(), glyphs.end(), [](ShapedGlyph const& a, ShapedGlyph const& b) { return a.origin_x < b.origin_x; });
		PR_EXPECT(glyphs.front().origin_x >= rightmost->origin_x - 0.5f);

		// The caret at the logical start comes from DirectWrite's hit-test API rather than from the
		// glyph walk, so the two agreeing is the real check that the walk is right.
		auto const caret_start = shaper.CaretAt(kFamily, kSize, hebrew, 0);
		PR_EXPECT(caret_start.is_rtl != 0);
		PR_EXPECT(std::abs(caret_start.x - width) < 2.0f);
	}

	PRUnitTest(MixedDirectionTextKeepsEachRunOnItsOwnSideOfTheLayout, Quick)
	{
		// B4: with Latin and Hebrew in one paragraph the runs carry different bidi levels, and the
		// glyphs shaped for the RTL word must fall inside the rectangle DirectWrite reports for
		// that same byte range.
		auto shaper = TextShaper{};
		auto const mixed = std::string("abc \u05E9\u05DC\u05D5\u05DD def");

		auto glyphs = std::vector<ShapedGlyph>{};
		shaper.Shape(kFamily, kSize, 1.0f, mixed, glyphs);
		PR_EXPECT(!glyphs.empty());

		// Both directions are present, which is what makes this a bidi case at all.
		PR_EXPECT(std::any_of(glyphs.begin(), glyphs.end(), [](ShapedGlyph const& g) { return (g.bidi_level % 2) == 1; }));
		PR_EXPECT(std::any_of(glyphs.begin(), glyphs.end(), [](ShapedGlyph const& g) { return (g.bidi_level % 2) == 0; }));

		// The Hebrew word occupies bytes [4, 12).
		auto const rects = shaper.RangeRects(kFamily, kSize, mixed, 4, 12, 32);
		PR_EXPECT(!rects.empty());

		auto left = rects.front().x;
		auto right = rects.front().x + rects.front().w;
		for (auto const& r : rects)
		{
			left = std::min(left, r.x);
			right = std::max(right, r.x + r.w);
		}
		PR_EXPECT(right > left);

		for (auto const& glyph : glyphs)
		{
			if ((glyph.bidi_level % 2) == 0)
				continue;

			PR_EXPECT(glyph.origin_x >= left - 2.0f);
			PR_EXPECT(glyph.origin_x <= right + 2.0f);
		}

		// Arabic joins as well as reorders, so it exercises a different shaping path to Hebrew.
		auto arabic_glyphs = std::vector<ShapedGlyph>{};
		auto const arabic = std::string("x \u0627\u0644\u0639\u0631\u0628\u064A\u0629 y");
		shaper.Shape(kFamily, kSize, 1.0f, arabic, arabic_glyphs);
		PR_EXPECT(std::any_of(arabic_glyphs.begin(), arabic_glyphs.end(), [](ShapedGlyph const& g) { return (g.bidi_level % 2) == 1; }));
	}

	PRUnitTest(PointerHitTestingSnapsToTheNearestClusterEdgeNotTheOneBefore, Quick)
	{
		// N6: DirectWrite reports whether a point fell in the trailing half of a cluster. Rounding
		// down regardless made it impossible to click after an emoji or an accented letter.
		auto shaper = TextShaper{};
		auto const text = std::string("a\U0001F600b");

		auto const before = shaper.CaretAt(kFamily, kSize, text, 1);
		auto const after = shaper.CaretAt(kFamily, kSize, text, 5);
		PR_EXPECT(after.x > before.x);

		// Just inside the leading edge of the emoji resolves to its start.
		PR_EXPECT(shaper.OffsetFromPoint(kFamily, kSize, text, before.x + 0.5f, 0.0f) == 1);

		// Just inside its trailing edge resolves to its end, which is the case that was wrong.
		PR_EXPECT(shaper.OffsetFromPoint(kFamily, kSize, text, after.x - 0.5f, 0.0f) == 5);

		// Every reported offset is a cluster boundary, whatever point is probed.
		for (auto x = -5.0f; x < after.x + 20.0f; x += 0.7f)
			PR_EXPECT(IsGraphemeBoundary(text, shaper.OffsetFromPoint(kFamily, kSize, text, x, 0.0f)));
	}

	PRUnitTest(DraggingAndShiftClickingSelectWholeGraphemeClusters, Quick)
	{
		// N7: press-move-release must select, and Shift+click must extend, without either ever
		// landing inside a cluster.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "hello world");
		FocusTextBox(engine);

		// Press near the start of the text, drag to the far right, release.
		engine.InputInject(PointerDownInput(12.0f, 50.0f));
		engine.InputInject(PointerMoveInput(205.0f, 50.0f));
		engine.InputInject(PointerUpInput(205.0f, 50.0f));

		auto const dragged = SemanticOf(engine, 2);
		PR_EXPECT((dragged.node.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::HasSelection)) != 0);
		PR_EXPECT(dragged.node.selection_end > dragged.node.selection_start);
		PR_EXPECT(IsGraphemeBoundary(dragged.Value(), dragged.node.selection_start));
		PR_EXPECT(IsGraphemeBoundary(dragged.Value(), dragged.node.selection_end));

		// Typing replaces the dragged selection wholesale.
		engine.InputInject(CharInputRecord('Z'));
		PR_EXPECT(SemanticOf(engine, 2).Value().size() < std::string("hello world").size());

		// Shift+click extends from the caret rather than starting a new selection.
		auto engine2 = UiEngine(MakeConfig());
		Scene(engine2, "hello world");
		FocusTextBox(engine2);

		engine2.InputInject(PointerDownInput(12.0f, 50.0f));
		engine2.InputInject(PointerUpInput(12.0f, 50.0f));
		engine2.InputInject(PointerDownInput(100.0f, 50.0f, EPointerButton::Left, kShift));

		auto const extended = SemanticOf(engine2, 2);
		PR_EXPECT((extended.node.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::HasSelection)) != 0);
		PR_EXPECT(IsGraphemeBoundary(extended.Value(), extended.node.selection_start));
		PR_EXPECT(IsGraphemeBoundary(extended.Value(), extended.node.selection_end));
	}

	PRUnitTest(CompositionUpdatesRespectMaxLengthSoTheCommitMatchesWhatWasShown, Quick)
	{
		// N8/N9: the limit is counted in grapheme clusters over the whole resulting string, so a
		// composition that would overflow is truncated where it is displayed rather than silently
		// losing characters at commit time.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "abc", 5);
		FocusTextBox(engine);

		auto const candidate = std::string("xyzw");
		engine.InputInject(KeyDownInput(VK_END));
		engine.InputInject(TextInputRecord(EInputKind::CompositionStart));
		engine.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(candidate, 4, 0, 4));

		// Only two clusters fit inside the limit of five.
		auto const shown = SemanticOf(engine, 2);
		PR_EXPECT(shown.Value() == "abcxy");
		PR_EXPECT(GraphemeCount(shown.Value()) == 5);

		// Committing the same string produces exactly the text that was on screen.
		engine.InputInjectText(TextInputRecord(EInputKind::CompositionCommit), TextPayload(candidate, 0, 0, 0));
		PR_EXPECT(SemanticOf(engine, 2).Value() == "abcxy");

		// A composition whose clusters are multi-byte is counted in clusters, not bytes.
		auto engine2 = UiEngine(MakeConfig());
		Scene(engine2, "", 2);
		FocusTextBox(engine2);

		auto const emoji = std::string("\U0001F600\U0001F601\U0001F602");
		engine2.InputInject(TextInputRecord(EInputKind::CompositionStart));
		engine2.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(emoji, 0, 0, 0));
		PR_EXPECT(GraphemeCount(SemanticOf(engine2, 2).Value()) == 2);
	}

	PRUnitTest(SemanticTextOffsetsAlwaysLandOnGraphemeBoundaries, Quick)
	{
		// N10: caret, selection and composition offsets are published for a future UI Automation
		// text provider, so they must be usable as text positions without further validation.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "a\U0001F468\u200D\U0001F469\u200D\U0001F467b");
		FocusTextBox(engine);

		engine.InputInject(KeyDownInput(VK_END));
		engine.InputInject(KeyDownInput('A', kCtrl));

		auto const selected = SemanticOf(engine, 2);
		PR_EXPECT(IsGraphemeBoundary(selected.Value(), selected.node.caret));
		PR_EXPECT(IsGraphemeBoundary(selected.Value(), selected.node.selection_start));
		PR_EXPECT(IsGraphemeBoundary(selected.Value(), selected.node.selection_end));
		PR_EXPECT(selected.node.value_grapheme_count == GraphemeCount(selected.Value()));

		// A composition publishes its own range, which must satisfy the same contract against the
		// full display string rather than against the pending text alone.
		auto const candidate = std::string("\U0001F600z");
		engine.InputInject(TextInputRecord(EInputKind::CompositionStart));
		engine.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(candidate, 4, 4, 4));

		auto const composing = SemanticOf(engine, 2);
		PR_EXPECT((composing.node.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::Composing)) != 0);
		PR_EXPECT(IsGraphemeBoundary(composing.Value(), composing.node.composition_start));
		PR_EXPECT(IsGraphemeBoundary(composing.Value(), composing.node.composition_start + composing.node.composition_length));
		PR_EXPECT(IsGraphemeBoundary(composing.Value(), composing.node.caret));
		PR_EXPECT(composing.node.composition_start + composing.node.composition_length <= composing.node.value_length);
	}

	PRUnitTest(CaretGeometryUsesTheRenderersOwnVerticalCentring, Quick)
	{
		// N11: the IME's candidate window is placed at this rectangle, so it must be the rectangle
		// the caret is actually drawn in. Both now come from TextOriginYDip.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "hello");
		FocusTextBox(engine);

		Rect caret{};
		std::int32_t valid = 0;
		engine.CaretGeometry(2, caret, valid);
		PR_EXPECT(valid != 0);
		PR_EXPECT(caret.h > 0.0f);

		// The TextBox occupies y in [40, 60), so a correctly centred caret is centred on the box.
		auto const box_centre = 50.0f;
		PR_EXPECT(std::abs((caret.y + caret.h * 0.5f) - box_centre) < 1.5f);

		// B3: an empty focused TextBox still has a caret to place, or the IME would have nowhere to
		// put its candidate list and nothing would be drawn.
		auto engine2 = UiEngine(MakeConfig());
		Scene(engine2, "");
		FocusTextBox(engine2);

		Rect empty_caret{};
		std::int32_t empty_valid = 0;
		engine2.CaretGeometry(2, empty_caret, empty_valid);
		PR_EXPECT(empty_valid != 0);
		PR_EXPECT(empty_caret.h > 0.0f);
		PR_EXPECT(std::abs((empty_caret.y + empty_caret.h * 0.5f) - box_centre) < 1.5f);

		// Both carets share the same vertical placement, since the formula depends on the box and
		// the font metrics rather than on the text.
		PR_EXPECT(std::abs(caret.y - empty_caret.y) < 0.01f);
		PR_EXPECT(std::abs(caret.h - empty_caret.h) < 0.01f);
	}

	PRUnitTest(TextOriginCentringIsTheSameFormulaEverywhere, Quick)
	{
		// N11: the caret geometry and the renderer both centre on the font's ascent plus descent
		// rather than on the control box alone, so the shared helper is pinned here.
		PR_EXPECT(TextOriginYDip(40.0f, 20.0f, 14.0f, 4.0f) == 40.0f + (20.0f - 18.0f) * 0.5f);
		PR_EXPECT(TextOriginYDip(0.0f, 10.0f, 16.0f, 4.0f) == (10.0f - 20.0f) * 0.5f);
		PR_EXPECT(TextOriginYDip(5.0f, 0.0f, 0.0f, 0.0f) == 5.0f);
	}

	PRUnitTest(AnEmptyFocusedTextBoxStillEmitsACaretInTheDrawPacket, Quick)
	{
		// B3: the draw packet's text item is what the renderer consumes. An empty focused TextBox
		// must still carry a visible caret, since that is the only thing telling the user where
		// their typing will go.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "");
		FocusTextBox(engine);
		engine.Update(Viewport(300, 200));

		auto const& packet = engine.DrawPackets();
		auto it = std::find_if(packet.items.begin(), packet.items.end(), [](DrawItem const& i) { return i.control_id == 2 && i.primitive == EVisualPrimitive::TextPresenter; });
		PR_EXPECT(it != packet.items.end());
		PR_EXPECT(it->text.empty());
		PR_EXPECT(it->caret_visible != 0);
	}

	PRUnitTest(TheFontAndRunFaceCachesStayBoundedUnderRepeatedDistinctRequests, Quick)
	{
		// N13: a long-running UI must not accumulate one cached DirectWrite object per font it has
		// ever measured. The caches evict least-recently-used entries instead.
		auto shaper = TextShaper{};
		auto const families = std::vector<std::string_view>{
			"Segoe UI", "Arial", "Tahoma", "Verdana", "Consolas", "Courier New", "Georgia",
			"Times New Roman", "Trebuchet MS", "Calibri", "Cambria", "Candara", "Corbel",
			"Franklin Gothic Medium", "Impact", "Lucida Console", "Palatino Linotype",
			"Segoe UI Semibold", "Segoe UI Light", "Malgun Gothic", "Microsoft YaHei",
			"MS Gothic", "Sylfaen", "Ebrima",
		};

		auto measured = 0;
		for (auto family : families)
		{
			// Not every machine has every family; only the ones that resolve exercise the cache.
			try
			{
				auto ascent = 0.0f;
				auto descent = 0.0f;
				shaper.Metrics(family, kSize, ascent, descent);
				++measured;
			}
			catch (EngineException const& ex)
			{
				PR_EXPECT(ex.Status() == EStatus::MissingAsset);
			}
			PR_EXPECT(shaper.CachedFontCount() <= TextShaper::MaxCachedFonts);
		}
		PR_EXPECT(measured > 0);

		// Shaping populates the run-face cache, which is bounded the same way.
		for (auto family : families)
		{
			try
			{
				auto glyphs = std::vector<ShapedGlyph>{};
				shaper.Shape(family, kSize, 1.0f, "sample text", glyphs);
			}
			catch (EngineException const& ex)
			{
				PR_EXPECT(ex.Status() == EStatus::MissingAsset);
			}
			PR_EXPECT(shaper.CachedRunFaceCount() <= TextShaper::MaxCachedRunFaces);
		}

		// Re-requesting a family that is already cached must not grow the cache.
		auto ascent = 0.0f;
		auto descent = 0.0f;
		shaper.Metrics(kFamily, kSize, ascent, descent);
		auto const settled = shaper.CachedFontCount();
		shaper.Metrics(kFamily, kSize, ascent, descent);
		PR_EXPECT(shaper.CachedFontCount() == settled);
	}

	PRUnitTest(RepeatedIdenticalInputProducesIdenticalSnapshotsAndGeometry, Quick)
	{
		// The corrective changes touched caching, hit-testing and semantics, all of which must stay
		// deterministic for identical input, or layout snapshots stop being comparable.
		struct Observation
		{
			std::string value;
			std::uint32_t selection_start;
			std::uint32_t selection_end;
			Rect caret;
		};
		auto Run = []()
		{
			auto engine = UiEngine(MakeConfig());
			Scene(engine, "deterministic \u05E9\u05DC\u05D5\u05DD text");
			FocusTextBox(engine);

			engine.InputInject(KeyDownInput(VK_END));
			engine.InputInject(KeyDownInput(VK_LEFT, kShift | kCtrl));

			auto const view = SemanticOf(engine, 2);

			Rect caret{};
			std::int32_t valid = 0;
			engine.CaretGeometry(2, caret, valid);
			PR_EXPECT(valid != 0);

			return Observation{ .value = view.Value(), .selection_start = view.node.selection_start, .selection_end = view.node.selection_end, .caret = caret };
		};

		auto const first = Run();
		auto const second = Run();

		PR_EXPECT(first.value == second.value);
		PR_EXPECT(first.selection_start == second.selection_start);
		PR_EXPECT(first.selection_end == second.selection_end);
		PR_EXPECT(first.caret.x == second.caret.x);
		PR_EXPECT(first.caret.y == second.caret.y);
		PR_EXPECT(first.caret.h == second.caret.h);
	}
}
