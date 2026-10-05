//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M1 tests (implementation-plan.md section 5.4): bounded event queue coalescing, CommandInvoked
// never silently dropped, explicit overflow fault/reconciliation, typed ID/kind/revision/sequence
// event records.
#include "pr/common/unittests.h"
#include "events.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// A Root containing one focusable, enabled Button, ready for click-driven CommandInvoked
		// sequences. Returns the button's control id (2) and its centre point in DIPs.
		void BuildRootWithButton(UiContext& ctx, ControlId button_id, float x, float y, float w, float h)
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
			auto button = MakeControl(button_id, 1, EControlType::Button, ELayoutMode::Overlay, Lp(w, h));
			button.layout.margin_left = x;
			button.layout.margin_top = y;
			b0.Upsert(button);
			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(400, 300));
		}

		// A full mouse click (down+up) at (x,y), yielding one CommandInvoked when it lands on an
		// enabled Button.
		void ClickAt(UiContext& ctx, float x, float y)
		{
			ctx.InputInject(PointerMoveInput(x, y));
			ctx.InputInject(PointerDownInput(x, y));
			ctx.InputInject(PointerUpInput(x, y));
		}
	}

	PRUnitTest(EventQueueCoalescesRepeatedFocusChanged, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackHorizontal, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f)));
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(400, 300));

		// Tab repeatedly toggles focus between the two buttons; every FocusChanged coalesces down
		// to the single latest value because only the most recent focus target matters to a host
		// that has not yet drained the queue (section 5.4's latest-value coalescing).
		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(KeyDownInput(VK_TAB));

		PR_EXPECT(ctx.EventCount() == 1);

		auto events = std::array<Event, 4>{};
		auto payload = std::array<std::byte, 256>{};
		ctx.EventsCopy(events, payload);
		PR_EXPECT(events[0].kind == EEventKind::FocusChanged);

		// Draining clears the queue.
		PR_EXPECT(ctx.EventCount() == 0);
	}

	PRUnitTest(EventQueueNeverCoalescesCommandInvoked, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButton(ctx, 2, 10.0f, 10.0f, 50.0f, 20.0f);

		// Draining focus-change noise from the first click before counting invocations.
		ClickAt(ctx, 20.0f, 15.0f);
		ClickAt(ctx, 20.0f, 15.0f);
		ClickAt(ctx, 20.0f, 15.0f);

		auto sizes = ctx.EventsPendingSizes();
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);

		// Every one of the three clicks must have produced its own CommandInvoked; unlike
		// FocusChanged, these must never be collapsed into fewer entries (section 5.4).
		auto command_count = std::count_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::CommandInvoked; });
		PR_EXPECT(command_count == 3);
	}

	PRUnitTest(EventQueueCoalescesTextChangeProposedPerControl, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f)));
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(400, 300));

		// Focus the text box, then type three characters; each keystroke proposes a new full
		// pending_text value, but only the latest matters until the host drains the queue.
		ctx.InputInject(PointerMoveInput(10.0f, 10.0f));
		ctx.InputInject(PointerDownInput(10.0f, 10.0f));
		ctx.InputInject(PointerUpInput(10.0f, 10.0f));
		ctx.InputInject(CharInputRecord('a'));
		ctx.InputInject(CharInputRecord('b'));
		ctx.InputInject(CharInputRecord('c'));

		auto sizes = ctx.EventsPendingSizes();
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);

		auto text_changes = std::count_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed; });
		PR_EXPECT(text_changes == 1);

		auto it = std::find_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed; });
		PR_EXPECT(it != events.end());
		auto text = std::string(reinterpret_cast<char const*>(payload.data()) + it->payload_offset, it->payload_length);
		PR_EXPECT(text == "abc");
	}

	PRUnitTest(EventQueueMarksOverflowWhenEvictingADifferentCoalescingKey, Quick)
	{
		// Two content slots plus the reserved overflow marker. Text proposals for different
		// controls are independent keys, so making room for control 2 by evicting control 1 must
		// be observable even though the newer proposal itself can still be accepted.
		auto queue = EventQueue{3};
		PR_EXPECT(queue.Push(1, EEventKind::TextChangeProposed, 7, 1, "first"));
		PR_EXPECT(queue.Push(9, EEventKind::CommandInvoked, 7, 0, {}));
		PR_EXPECT(queue.Push(2, EEventKind::TextChangeProposed, 7, 1, "second"));
		PR_EXPECT(queue.OverflowActive());
		PR_EXPECT(queue.Count() == 3);

		auto events = std::array<Event, 3>{};
		auto payload = std::array<std::byte, 32>{};
		queue.Copy(events, payload);

		PR_EXPECT(std::any_of(events.begin(), events.end(), [](Event const& event) { return event.kind == EEventKind::QueueOverflow; }));
		PR_EXPECT(std::none_of(events.begin(), events.end(), [](Event const& event) { return event.kind == EEventKind::TextChangeProposed && event.control_id == 1; }));

		auto replacement = std::find_if(events.begin(), events.end(), [](Event const& event) { return event.kind == EEventKind::TextChangeProposed && event.control_id == 2; });
		PR_EXPECT(replacement != events.end());
		auto text = std::string(reinterpret_cast<char const*>(payload.data()) + replacement->payload_offset, replacement->payload_length);
		PR_EXPECT(text == "second");
	}

	PRUnitTest(EventQueueOverflowRaisesFaultAndDoesNotDropCommandInvoked, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};

		// content_capacity = max_queued_events - 1 = 2. Focus the button once (one coalescible
		// FocusChanged), then invoke it via Enter three times: Enter alone pushes only a
		// CommandInvoked (no capture-change noise), so this deterministically exercises "two
		// commands fit, the coalescible FocusChanged is evicted to make room, and the third
		// command finds no coalescible victim left and must raise QueueOverflow" (section 5.4).
		auto config = MakeConfig(4096, 64, 64, 4096, 1u << 20, 256, 256, 256, /*max_queued_events*/ 3);
		auto ctx = UiContext(runtime, &config, &device);
		BuildRootWithButton(ctx, 2, 10.0f, 10.0f, 50.0f, 20.0f);

		ctx.InputInject(KeyDownInput(VK_TAB));    // FocusChanged                                -> [FocusChanged]
		ctx.InputInject(KeyDownInput(VK_RETURN)); // CommandInvoked                               -> [FocusChanged, CommandInvoked]
		ctx.InputInject(KeyDownInput(VK_RETURN)); // CommandInvoked evicts FocusChanged to fit     -> [CommandInvoked, CommandInvoked]

		auto overflow_seen = false;
		try
		{
			// No coalescible entry remains to evict, so this third command must overflow.
			ctx.InputInject(KeyDownInput(VK_RETURN));
		}
		catch (Exception const& ex)
		{
			overflow_seen = true;
			PR_EXPECT(ex.Status() == EStatus::QueueOverflow);
		}
		PR_EXPECT(overflow_seen);

		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.event_overflow_count >= 1);

		// The queue afterward must report the overflow marker itself as one pending event.
		auto sizes = ctx.EventsPendingSizes();
		PR_EXPECT(sizes.m_count >= 1);
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);
		auto has_overflow_marker = std::any_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::QueueOverflow; });
		PR_EXPECT(has_overflow_marker);
	}

	PRUnitTest(EventRecordsCarryStableIdKindRevisionSequence, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButton(ctx, 2, 10.0f, 10.0f, 50.0f, 20.0f);

		ClickAt(ctx, 20.0f, 15.0f);

		auto sizes = ctx.EventsPendingSizes();
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);

		auto it = std::find_if(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::CommandInvoked; });
		PR_EXPECT(it != events.end());
		PR_EXPECT(it->control_id == 2);
		PR_EXPECT(it->accepted_revision == 1);
		// Sequence numbers are monotonically increasing per context; a freshly created context's
		// first event must have a well-defined, non-zero sequence.
		PR_EXPECT(it->sequence != 0);
	}
}
