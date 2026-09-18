//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Focused Slider coverage for descriptor validation, typed proposal events, pointer capture,
// keyboard interaction, authoritative reconciliation, disabled behaviour, visuals, and semantics.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "input.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Build one root with a horizontal slider at [10, 110] DIPs.
		ControlDesc BuildSlider(UiContext& ctx, float value = 2.0f)
		{
			// Keep all interaction tests on the same non-normalized range and step lattice.
			auto transaction = TxnBuilder{};
			transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 80.0f)));
			auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			slider.layout.margin_left = 10.0f;
			slider.layout.margin_top = 10.0f;
			slider.minimum = -2.0f;
			slider.maximum = 6.0f;
			slider.step = 0.5f;
			slider.value = value;
			transaction.Upsert(slider);
			ctx.TransactionApply(transaction.Build(0, 1));
			ctx.Update(Viewport(200, 80));
			return slider;
		}

		// Drain every event with its fixed ABI record intact.
		std::vector<Event> DrainSliderEvents(UiContext& ctx)
		{
			// Size both event buffers exactly as the public drain contract requires.
			auto const sizes = ctx.EventsPendingSizes();
			auto events = std::vector<Event>(sizes.m_count);
			auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
			ctx.EventsCopy(events, payload);
			return events;
		}

		// Return the latest typed slider proposal in one drained event batch.
		Event const& SliderProposal(std::vector<Event> const& events)
		{
			// Locate the coalesced proposal while retaining other ordering/capture events for assertions.
			auto const found = std::find_if(events.begin(), events.end(), [](Event const& event)
			{
				// Match only the slider's typed value event.
				return event.kind == EEventKind::ValueChangeProposed;
			});
			if (found == events.end())
				throw std::runtime_error("slider proposal event was not found");

			return *found;
		}
	}

	// Every scalar contract violation is rejected without replacing the accepted tree.
	PRUnitTest(SliderRejectsInvalidDescriptorScalars, Quick)
	{
		// Establish one valid accepted slider as the atomic-rejection baseline.
		auto engine = UiEngine(MakeConfig());
		auto transaction = TxnBuilder{};
		transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 80.0f)));
		auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		slider.value = 0.5f;
		transaction.Upsert(slider);
		engine.TransactionApply(transaction.Build(0, 1));

		auto reject = [&](ControlDesc invalid)
		{
			// Every rejected replacement must leave revision 1 authoritative.
			auto change = TxnBuilder{};
			change.Upsert(invalid);
			PR_THROWS(engine.TransactionApply(change.Build(1, 2)), EngineException);
			PR_EXPECT(engine.DiagnosticsGet().accepted_revision == 1);
		};
		for (auto value : { std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -0.1f, 1.1f })
		{
			auto invalid = slider;
			invalid.value = value;
			reject(invalid);
		}
		for (auto step : { 0.0f, -0.1f, 1.1f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() })
		{
			auto invalid = slider;
			invalid.step = step;
			reject(invalid);
		}
		for (auto bounds : { std::pair{1.0f, 1.0f}, std::pair{2.0f, 1.0f}, std::pair{std::numeric_limits<float>::quiet_NaN(), 1.0f}, std::pair{0.0f, std::numeric_limits<float>::infinity()} })
		{
			auto invalid = slider;
			invalid.minimum = bounds.first;
			invalid.maximum = bounds.second;
			reject(invalid);
		}
	}

	// Click and captured drag propose snapped typed values while leaving the accepted value unchanged.
	PRUnitTest(SliderPointerCaptureProposesTypedValueWithoutMutation, Quick)
	{
		// Drag beyond the right edge while captured to propose the exact maximum.
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		auto slider = BuildSlider(ctx);

		ctx.InputInject(PointerDownInput(35.0f, 15.0f));
		ctx.InputInject(PointerMoveInput(150.0f, 15.0f));
		ctx.InputInject(PointerUpInput(150.0f, 15.0f));
		auto const events = DrainSliderEvents(ctx);
		auto const& proposal = SliderProposal(events);
		PR_EXPECT(proposal.control_id == 2);
		PR_EXPECT(proposal.has_numeric_value != 0);
		PR_EXPECT(proposal.numeric_value == 6.0);
		PR_EXPECT(proposal.payload_length == 0);
		PR_EXPECT(proposal.accepted_revision == 1);
		PR_EXPECT(std::any_of(events.begin(), events.end(), [](Event const& event) { return event.kind == EEventKind::PointerCaptureChanged && event.control_id == 0; }));

		// The draw packet and semantics still expose the descriptor's accepted value.
		ctx.Update(Viewport(200, 80));
		auto const sizes = ctx.SemanticsPendingSizes();
		auto nodes = std::vector<SemanticNode>(sizes.m_count);
		auto text = std::vector<char>(sizes.m_payload_bytes);
		ctx.SemanticsCopy(nodes, text);
		auto const found = std::find_if(nodes.begin(), nodes.end(), [](SemanticNode const& node) { return node.id == 2; });
		PR_EXPECT(found != nodes.end() && found->progress_value == slider.value);
	}

	// Arrow/Home/End keys use Step and bounds, and reconciliation changes the next proposal base.
	PRUnitTest(SliderKeyboardUsesAcceptedValueAndReconciles, Quick)
	{
		// Focus the slider through normal traversal before issuing range keys.
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		auto slider = BuildSlider(ctx);
		ctx.InputInject(KeyDownInput(VK_TAB));
		DrainSliderEvents(ctx);

		ctx.InputInject(KeyDownInput(VK_RIGHT));
		auto proposal = SliderProposal(DrainSliderEvents(ctx));
		PR_EXPECT(proposal.numeric_value == 2.5);

		// Accept the proposal, then the next arrow starts from that new authoritative descriptor.
		slider.value = static_cast<float>(proposal.numeric_value);
		auto change = TxnBuilder{};
		change.Upsert(slider);
		ctx.TransactionApply(change.Build(1, 2));
		ctx.InputInject(KeyDownInput(VK_LEFT));
		PR_EXPECT(SliderProposal(DrainSliderEvents(ctx)).numeric_value == 2.0);
		ctx.InputInject(KeyDownInput(VK_HOME));
		PR_EXPECT(SliderProposal(DrainSliderEvents(ctx)).numeric_value == -2.0);
		ctx.InputInject(KeyDownInput(VK_END));
		PR_EXPECT(SliderProposal(DrainSliderEvents(ctx)).numeric_value == 6.0);
	}

	// Slider semantics expose range role/value/action metadata, with no SetValue action when disabled.
	PRUnitTest(SliderSemanticsAndDisabledStateFollowAcceptedDescriptor, Quick)
	{
		// Capture the enabled range metadata before applying a disabled replacement.
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		auto slider = BuildSlider(ctx);
		auto read = [&]()
		{
			// Refresh and return the slider's current semantic record by value.
			ctx.Update(Viewport(200, 80));
			auto const sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text);
			return *std::find_if(nodes.begin(), nodes.end(), [](SemanticNode const& node) { return node.id == 2; });
		};
		auto semantic = read();
		PR_EXPECT(semantic.role == EControlType::Slider);
		PR_EXPECT(semantic.progress_value == 2.0f);
		PR_EXPECT(semantic.range_minimum == -2.0f && semantic.range_maximum == 6.0f && semantic.range_step == 0.5f);
		PR_EXPECT((semantic.supported_actions & static_cast<std::uint32_t>(ESemanticAction::SetValue)) != 0);

		slider.enabled = 0;
		auto change = TxnBuilder{};
		change.Upsert(slider);
		ctx.TransactionApply(change.Build(1, 2));
		semantic = read();
		PR_EXPECT((semantic.state_flags & static_cast<std::uint32_t>(ESemanticState::Enabled)) == 0);
		PR_EXPECT((semantic.supported_actions & static_cast<std::uint32_t>(ESemanticAction::SetValue)) == 0);
		ctx.InputInject(PointerDownInput(60.0f, 15.0f));
		PR_EXPECT(ctx.EventCount() == 0);
	}

	// Semantic SetValue follows the same typed proposal and authoritative reconciliation contract.
	PRUnitTest(SliderSemanticSetValueProposesWithoutMutation, Quick)
	{
		// Apply the accessibility action through the engine path used by UI Automation.
		auto engine = UiEngine(MakeConfig());
		auto transaction = TxnBuilder{};
		transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 80.0f)));
		auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		slider.minimum = -2.0f;
		slider.maximum = 6.0f;
		slider.step = 0.5f;
		slider.value = 2.0f;
		transaction.Upsert(slider);
		engine.TransactionApply(transaction.Build(0, 1));
		engine.Update(Viewport(200, 80));
		engine.ApplySemanticAction(SemanticActionRequest{
			.kind = ESemanticActionKind::SetValue,
			.control_id = 2,
			.numeric_value = 5.2,
		});

		// The proposed value is snapped, typed, and sequenced against the accepted revision.
		auto events = std::vector<Event>(engine.EventCount());
		auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
		engine.EventsCopy(events, payload);
		auto const& proposal = SliderProposal(events);
		PR_EXPECT(proposal.control_id == 2);
		PR_EXPECT(proposal.has_numeric_value != 0);
		PR_EXPECT(proposal.numeric_value == 5.0);
		PR_EXPECT(proposal.accepted_revision == 1);

		// Accessibility continues to publish the accepted descriptor until the caller reconciles.
		engine.Update(Viewport(200, 80));
		auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
		auto text = std::vector<char>(engine.SemanticTextBytesPending());
		engine.SemanticsCopy(nodes, text);
		auto const found = std::find_if(nodes.begin(), nodes.end(), [](SemanticNode const& node) { return node.id == 2; });
		PR_EXPECT(found != nodes.end() && found->progress_value == 2.0f);
	}

	// Lookless rendering uses track fill plus foreground indicator/thumb and DPI-scaled geometry.
	PRUnitTest(SliderDrawPacketUsesExistingStylePrimitives, Quick)
	{
		// Resolve one half-range slider through the ordinary style and draw-packet pipeline.
		auto engine = UiEngine(MakeConfig());
		auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		slider.value = 0.5f;
		slider.style_id = 10;
		auto style = MakeStyle(10);
		for (auto& visual : style.visuals)
		{
			visual.fill = Colour{ 0.1f, 0.1f, 0.1f, 1.0f };
			visual.foreground = Colour{ 0.0f, 1.0f, 0.0f, 1.0f };
		}
		auto transaction = TxnBuilder{};
		transaction.AddStyle(style);
		transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 80.0f)));
		transaction.Upsert(slider);
		engine.TransactionApply(transaction.Build(0, 1));
		engine.Update(Viewport(200, 80, 192.0f));
		auto const& items = engine.DrawPackets().items;
		PR_EXPECT(items.size() == 4);
		PR_EXPECT(items[2].primitive == EVisualPrimitive::SolidBox && items[2].fill.g == 1.0f);
		PR_EXPECT(items[3].primitive == EVisualPrimitive::RoundedBox && items[3].fill.g == 1.0f);
	}

	// A hovered Slider selects the standard hand cursor without installing a custom cursor resource.
	PRUnitTest(SliderHoverSelectsInteractivePointerCursor, Quick)
	{
		// Seed hover through deterministic normalized input before asking Win32 for the cursor.
		auto engine = UiEngine(MakeConfig());
		auto transaction = TxnBuilder{};
		transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 80.0f)));
		transaction.Upsert(MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(100.0f, 20.0f)));
		engine.TransactionApply(transaction.Build(0, 1));
		engine.Update(Viewport(200, 80));
		engine.InputInject(PointerMoveInput(50.0f, 10.0f));

		auto result = LRESULT{};
		auto invalidate = std::int32_t{};
		auto const handled = engine.ProcessWindowMessage(nullptr, WM_SETCURSOR, 0, MAKELPARAM(HTCLIENT, WM_MOUSEMOVE), result, invalidate);
		PR_EXPECT(handled != 0 && result == TRUE);
		PR_EXPECT(GetCursor() == LoadCursor(nullptr, IDC_HAND));
	}
}
