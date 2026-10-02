//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M3 black-box test (implementation-plan.md section 7.1/7.3): a pointer press/release landing on
// a Button's non-focusable label/content descendant (or a TextBox's non-focusable content
// descendant) must still activate/target the owning Button/TextBox itself, not be silently
// ignored just because the innermost hit-tested control isn't itself focusable. Exercised entirely
// through the public UiContext facade, exactly like every M0-M2 black-box test.
#include "pr/common/unittests.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Root(1, 300x200) containing:
		//  - Button(2) at [10,10)-[90,40) with a non-focusable Text label child(3) at
		//    [10,10)-[50,24), fully inside the button.
		//  - TextBox(4) at [10,60)-[110,80) with a non-focusable Panel content child(5) at
		//    [12,62)-[32,72), fully inside the text box.
		// Neither child(3) nor child(5) is itself focusable (MakeControl only marks Button/TextBox
		// focusable), so a click landing on one of them can only be observed to "do something" if
		// the owning ancestor is the one actually activated/targeted.
		void BuildRootWithButtonLabelAndTextBoxContent(UiContext& ctx)
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

			auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80.0f, 30.0f));
			button.layout.margin_left = 10.0f;
			button.layout.margin_top = 10.0f;
			b0.Upsert(button);

			auto label = MakeControl(3, 2, EControlType::Text, ELayoutMode::Overlay, Lp(40.0f, 14.0f));
			std::tie(label.text_offset, label.text_length) = b0.AddText("OK");
			b0.Upsert(label);

			auto text_box = MakeControl(4, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 60.0f;
			b0.Upsert(text_box);

			auto content = MakeControl(5, 4, EControlType::Panel, ELayoutMode::Overlay, Lp(20.0f, 10.0f));
			content.layout.margin_left = 2.0f;
			content.layout.margin_top = 2.0f;
			b0.Upsert(content);

			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(300, 200));
		}

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

		bool IsFocused(UiContext& ctx, ControlId id)
		{
			ctx.Update(Viewport(300, 200));
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			return it != nodes.end() && (it->state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) != 0;
		}
	}

	PRUnitTest(PointerPressOnButtonLabelActivatesTheOwningButtonNotTheLabel, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonLabelAndTextBoxContent(ctx);

		// (25,15) lies within both the label(3)'s rect [10,10)-[50,24) and the button(2)'s rect
		// [10,10)-[90,40); the label is hit-tested first as the innermost/topmost control there.
		ctx.InputInject(PointerMoveInput(25.0f, 15.0f));
		ctx.InputInject(PointerDownInput(25.0f, 15.0f));
		ctx.InputInject(PointerUpInput(25.0f, 15.0f));

		auto events = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(events, EEventKind::CommandInvoked, 2));  // the Button, not...
		PR_EXPECT(!HasEventFor(events, EEventKind::CommandInvoked, 3)); // ...its label
		PR_EXPECT(IsFocused(ctx, 2));
	}

	PRUnitTest(PointerReleaseMustStayOverTheSameOwningButtonThroughDescendantsToInvoke, Quick)
	{
		// Press on the label, then release outside the button entirely: NearestOfType must see the
		// release did not land back on the same owning Button and must not invoke the command,
		// exactly matching the existing "release must stay over the pressed control" contract, now
		// verified through a content descendant on both ends of the gesture.
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonLabelAndTextBoxContent(ctx);

		ctx.InputInject(PointerMoveInput(25.0f, 15.0f));
		ctx.InputInject(PointerDownInput(25.0f, 15.0f));
		ctx.InputInject(PointerMoveInput(250.0f, 150.0f));
		ctx.InputInject(PointerUpInput(250.0f, 150.0f));

		auto events = DrainEvents(ctx);
		PR_EXPECT(!HasEventFor(events, EEventKind::CommandInvoked, 2));
	}

	PRUnitTest(PointerPressOnTextBoxContentFocusesTheOwningTextBoxNotTheContent, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithButtonLabelAndTextBoxContent(ctx);

		// (20,65) lies within both the content panel(5)'s rect [12,62)-[32,72) and the text box(4)'s
		// rect [10,60)-[110,80); the content panel is the innermost control hit-tested there.
		ctx.InputInject(PointerMoveInput(20.0f, 65.0f));
		ctx.InputInject(PointerDownInput(20.0f, 65.0f));
		ctx.InputInject(PointerUpInput(20.0f, 65.0f));

		PR_EXPECT(IsFocused(ctx, 4));  // the TextBox itself gained focus, not...
		PR_EXPECT(!IsFocused(ctx, 5)); // ...its non-focusable content (which cannot be focused at all)

		// Typing after this click must edit the TextBox's own text, proving the caret was actually
		// placed in TextBox(4) and not silently dropped for lacking a focusable hit target.
		ctx.InputInject(CharInputRecord('Z'));
		auto events = DrainEvents(ctx);
		PR_EXPECT(HasEventFor(events, EEventKind::TextChangeProposed, 4));
	}
}
