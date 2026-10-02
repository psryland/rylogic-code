//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M2 tests (implementation-plan.md sections 7.2-7.5): raw Win32 message processing translated
// into the same normalized input state machine as deterministic injection; pointer hit-
// testing/capture/focus; Tab/Shift+Tab; Button mouse/Enter/Space; TextBox editing and clipboard
// basics; outside click clears focus and remains unconsumed; UI keys only consumed while
// applicable.
#include "pr/common/unittests.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Root (id 1, fixed 300x200) containing one Button (id 2, rect [10,60)x[10,30)) and one
		// TextBox (id 3, rect [10,110)x[40,60)), both focusable and enabled.
		void BuildRootWithButtonAndTextBox(UiContext& ctx)
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
			auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
			button.layout.margin_left = 10.0f;
			button.layout.margin_top = 10.0f;
			b0.Upsert(button);
			auto text_box = MakeControl(3, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 40.0f;
			b0.Upsert(text_box);
			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(300, 200));
		}

		// Drain and return every currently queued event (test-only convenience; production hosts
		// would size their buffers from EventsPendingSizes as the other M1 tests already cover).
		std::vector<Event> DrainEvents(UiContext& ctx)
		{
			auto sizes = ctx.EventsPendingSizes();
			auto events = std::vector<Event>(sizes.m_count);
			auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
			ctx.EventsCopy(events, payload);
			return events;
		}

		bool HasEventFor(std::vector<Event> const& events, EEventKind kind, ControlId control_id)
		{
			return std::any_of(events.begin(), events.end(), [&](Event const& e) { return e.kind == kind && e.control_id == control_id; });
		}

		// The 'Focused' semantic state flag for one control, after a fresh Update() snapshot. The
		// viewport defaults to the 300x200 size used by BuildRootWithButtonAndTextBox; callers with
		// a differently-sized root (e.g. an autosized Root under a larger viewport) must pass their
		// own viewport_w/viewport_h so this helper does not silently reset the layout to the wrong size.
		bool IsFocused(UiContext& ctx, ControlId id, int viewport_w = 300, int viewport_h = 200)
		{
			ctx.Update(Viewport(viewport_w, viewport_h));
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			return it != nodes.end() && (it->state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) != 0;
		}
	}

	PRUnitTest(InputPointerClickOnButtonSetsFocusAndInvokesCommand, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		ctx.InputInject(PointerMoveInput(20.0f, 15.0f));
		ctx.InputInject(PointerDownInput(20.0f, 15.0f));
		ctx.InputInject(PointerUpInput(20.0f, 15.0f));

		PR_EXPECT(IsFocused(ctx, 2));

		auto events = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(events, EEventKind::FocusChanged, 2));
		PR_EXPECT(HasEventFor(events, EEventKind::CommandInvoked, 2));
	}

	PRUnitTest(InputPointerUpOutsideButtonAfterDownDoesNotInvokeCommand, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		// Press on the button, drag off it, release over empty space: real OS mouse capture would
		// still deliver the Up to this context, but a "click" (CommandInvoked) only fires when the
		// release point is still over the pressed control (section 7.3).
		ctx.InputInject(PointerMoveInput(20.0f, 15.0f));
		ctx.InputInject(PointerDownInput(20.0f, 15.0f));
		ctx.InputInject(PointerMoveInput(250.0f, 150.0f));
		ctx.InputInject(PointerUpInput(250.0f, 150.0f));

		auto events = DrainEvents(ctx);
		PR_EXPECT(!HasEventFor(events, EEventKind::CommandInvoked, 2));
	}

	PRUnitTest(InputTabAdvancesFocusForwardWithWraparound, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		// Tab order is pre-order over focusable+enabled controls: button (2) then text box (3).
		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 2));

		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 3));

		// A third Tab wraps back around to the first focusable control.
		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 2));
	}

	PRUnitTest(InputShiftTabMovesFocusBackwardWithWraparound, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		ctx.InputInject(KeyDownInput(VK_TAB)); // focus -> button (2)
		PR_EXPECT(IsFocused(ctx, 2));

		// Shift+Tab from the first control must wrap backward to the last one.
		ctx.InputInject(KeyDownInput(VK_TAB, static_cast<std::uint32_t>(EInputModifier::Shift)));
		PR_EXPECT(IsFocused(ctx, 3));
	}

	PRUnitTest(InputEnterInvokesFocusedButtonImmediately, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		ctx.InputInject(KeyDownInput(VK_TAB)); // focus -> button (2)
		ctx.InputInject(KeyDownInput(VK_RETURN));

		auto events = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(events, EEventKind::CommandInvoked, 2));
	}

	PRUnitTest(InputSpaceInvokesFocusedButtonOnlyAfterKeyUp, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		ctx.InputInject(KeyDownInput(VK_TAB)); // focus -> button (2)
		ctx.InputInject(KeyDownInput(VK_SPACE));

		// Pressing Space arms the button (mirrors physical press-and-hold) but must not invoke it
		// until the key is released (section 7.3's Button state machine).
		auto after_down = DrainEvents(ctx);
		PR_EXPECT(!HasEventFor(after_down, EEventKind::CommandInvoked, 2));

		ctx.InputInject(KeyUpInput(VK_SPACE));
		auto after_up = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(after_up, EEventKind::CommandInvoked, 2));
	}

	PRUnitTest(InputTextBoxTypingBackspaceDeleteAndCaretMovementEditPendingText, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		// Focus the text box, then type "abc" (caret ends at 3, after 'c').
		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 3));
		ctx.InputInject(CharInputRecord('a'));
		ctx.InputInject(CharInputRecord('b'));
		ctx.InputInject(CharInputRecord('c'));

		// Home (caret->0, no text change), Right (caret->1, no text change), Delete (removes 'b'
		// at index 1, leaving "ac"), End (caret->2, no text change), Backspace (removes 'c' at
		// index 1, leaving "a").
		ctx.InputInject(KeyDownInput(VK_HOME));
		ctx.InputInject(KeyDownInput(VK_RIGHT));
		ctx.InputInject(KeyDownInput(VK_DELETE));
		ctx.InputInject(KeyDownInput(VK_END));
		ctx.InputInject(KeyDownInput(VK_BACK));

		// All five edits target the same control, so TextChangeProposed coalesces down to one
		// queued entry carrying only the final "a" (section 5.4's per-control latest-value
		// coalescing); the pure caret-movement keys (Home/Right/End) must not have queued
		// anything at all.
		auto sizes = ctx.EventsPendingSizes();
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);

		auto text_changes = std::count_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed; });
		PR_EXPECT(text_changes == 1);

		auto it = std::find_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed; });
		PR_EXPECT(it != events.end());
		auto text = std::string(reinterpret_cast<char const*>(payload.data()) + it->payload_offset, it->payload_length);
		PR_EXPECT(text == "a");
	}

	PRUnitTest(InputTextBoxCtrlAThenCtrlCThenCtrlVRoundTripsSelectionThroughClipboard, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 3));
		ctx.InputInject(CharInputRecord('h'));
		ctx.InputInject(CharInputRecord('i'));

		// Select all ("hi"), copy it, move caret to the end (deselecting), then paste: the pasted
		// copy is inserted at the (now collapsed) caret, doubling the text to "hihi".
		auto ctrl = static_cast<std::uint32_t>(EInputModifier::Ctrl);
		ctx.InputInject(KeyDownInput('A', ctrl));
		ctx.InputInject(KeyDownInput('C', ctrl));
		ctx.InputInject(KeyDownInput(VK_END));
		ctx.InputInject(KeyDownInput('V', ctrl));

		auto sizes = ctx.EventsPendingSizes();
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);

		auto it = std::find_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed; });
		PR_EXPECT(it != events.end());
		auto text = std::string(reinterpret_cast<char const*>(payload.data()) + it->payload_offset, it->payload_length);

		// Clipboard access is best-effort (section 7.5): a host session without a reachable OS
		// clipboard (e.g. a locked-down service session) leaves Ctrl+C/Ctrl+V as no-ops rather
		// than throwing, so only the interactive-desktop case can observe the doubled "hihi";
		// otherwise the unpasted "hi" is the correct degrade-to-no-op outcome.
		auto clipboard_available = OpenClipboard(nullptr) != 0;
		if (clipboard_available)
			CloseClipboard();
		PR_EXPECT(text == (clipboard_available ? "hihi" : "hi"));
	}

	PRUnitTest(InputOutsideClickClearsFocusAndRemainsUnconsumed, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		auto hwnd = GetConsoleWindow();

		// True "outside click" means a point with no interactive (Button/TextBox) control beneath
		// it: Root/Panel/Text are hit-test transparent (see input.cpp's IsHitTestable), so this is
		// unaffected by whether the point falls within the root's own bounds. With nothing focused
		// yet, an outside click must report both unconsumed and no redraw needed (nothing changed).
		{
			LRESULT result = 0;
			auto invalidate = std::int32_t{};
			auto consumed = ctx.ProcessWindowMessage(hwnd, WM_LBUTTONDOWN, 0, MAKELPARAM(350, 250), result, invalidate);
			PR_EXPECT(consumed == 0);
			PR_EXPECT(invalidate == 0);
		}

		// Focus the button, then click outside: this time focus is actually cleared, so
		// 'invalidate' must be true even though the click itself is still unconsumed (section
		// 7.3; this is the behaviour fixed in input.cpp this session).
		ctx.InputInject(PointerMoveInput(20.0f, 15.0f));
		ctx.InputInject(PointerDownInput(20.0f, 15.0f));
		ctx.InputInject(PointerUpInput(20.0f, 15.0f));
		PR_EXPECT(IsFocused(ctx, 2));

		{
			LRESULT result = 0;
			auto invalidate = std::int32_t{};
			auto consumed = ctx.ProcessWindowMessage(hwnd, WM_LBUTTONDOWN, 0, MAKELPARAM(350, 250), result, invalidate);
			PR_EXPECT(consumed == 0);
			PR_EXPECT(invalidate != 0);
		}
		PR_EXPECT(!IsFocused(ctx, 2));
	}

	PRUnitTest(InputProcessWindowMessageIsEquivalentToDeterministicInjectionForButtonClick, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		auto hwnd = GetConsoleWindow();
		LRESULT result = 0;
		auto invalidate = std::int32_t{};

		// A raw WM_LBUTTONDOWN/WM_LBUTTONUP pair at the button's centre must drive the identical
		// FocusChanged + CommandInvoked outcome as the deterministic InputInject path, since both
		// funnel through the same normalized input state machine (section 7.1).
		auto consumed_down = ctx.ProcessWindowMessage(hwnd, WM_LBUTTONDOWN, 0, MAKELPARAM(20, 15), result, invalidate);
		auto consumed_up = ctx.ProcessWindowMessage(hwnd, WM_LBUTTONUP, 0, MAKELPARAM(20, 15), result, invalidate);
		PR_EXPECT(consumed_down != 0);
		PR_EXPECT(consumed_up != 0);

		auto events = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(events, EEventKind::FocusChanged, 2));
		PR_EXPECT(HasEventFor(events, EEventKind::CommandInvoked, 2));
	}

	PRUnitTest(InputUnrecognisedWindowMessageIsIgnoredAndLeftUnconsumed, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonAndTextBox(ctx);

		auto hwnd = GetConsoleWindow();
		LRESULT result = 0;
		auto invalidate = std::int32_t{};

		// WM_PAINT (or any message outside the recognised UI input vocabulary) must be reported
		// as unconsumed with no state change, so the host's own window procedure handles it
		// unmodified (section 7.1).
		auto consumed = ctx.ProcessWindowMessage(hwnd, WM_PAINT, 0, 0, result, invalidate);
		PR_EXPECT(consumed == 0);
		PR_EXPECT(invalidate == 0);
	}

	PRUnitTest(InputHitTestIsTransparentThroughRootAndPanelSoOutsideThePanelClearsFocusUnconsumed, Quick)
	{
		// A common host arrangement: an autosized Root (width/height 0 => sized to the full
		// viewport, see layout.cpp's ComputeLayout) hosting one small Panel (id 2, rect
		// [20,40)x[20,60)) that in turn hosts one Button (id 3, rect [30,80)x[30,50)). Root and
		// Panel are pure layout/decoration and must be hit-test transparent (section 7.2): a click
		// anywhere in the 400x300 viewport that is not over the Button must be a miss, even though
		// the autosized Root's own bounds cover the entire point, or every click outside the small
		// panel would be wrongly absorbed by the root and never reach the host's own scene/camera
		// input (section 7.3).
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
			auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(60.0f, 40.0f));
			panel.layout.margin_left = 20.0f;
			panel.layout.margin_top = 20.0f;
			b0.Upsert(panel);
			auto button = MakeControl(3, 2, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
			button.layout.margin_left = 10.0f;
			button.layout.margin_top = 10.0f;
			b0.Upsert(button);
			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(400, 300));
		}

		// Focus the button first, so the outside click below has a focus to actually clear.
		ctx.InputInject(PointerMoveInput(50.0f, 40.0f));
		ctx.InputInject(PointerDownInput(50.0f, 40.0f));
		ctx.InputInject(PointerUpInput(50.0f, 40.0f));
		PR_EXPECT(IsFocused(ctx, 3, 400, 300));
		DrainEvents(ctx); // discard the FocusChanged from focusing the button; only the outside click matters below

		// (399, 299) lies at the extreme corner of the 400x300 autosized root, and outside the
		// panel/button entirely: this must clear focus yet remain unconsumed, so the host's own
		// camera/scene input still observes the click (section 7.3). Routed through
		// ProcessWindowMessage (as a real host would) rather than InputInject, since only the
		// former reports 'consumed'.
		auto hwnd = GetConsoleWindow();
		LRESULT result = 0;
		auto invalidate = std::int32_t{};
		auto consumed = ctx.ProcessWindowMessage(hwnd, WM_LBUTTONDOWN, 0, MAKELPARAM(399, 299), result, invalidate);
		PR_EXPECT(consumed == 0);
		PR_EXPECT(invalidate != 0);
		PR_EXPECT(!IsFocused(ctx, 3, 400, 300));

		auto events = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(events, EEventKind::FocusChanged, 0));
	}
}
