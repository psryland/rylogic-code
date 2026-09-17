//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M3 white-box tests (implementation-plan.md sections 8/9.3) for UiEngine::DrawPackets(): the
// immutable, renderer-neutral draw packet consumed natively by the future View3D host bridge.
// DrawPacket/DrawItem carry no public ABI surface (draw_packet.h is a plain internal header, not
// part of view3d-ui.h), so these tests construct UiEngine directly rather than going through the
// UiContext facade every other test file in this project uses; view3d-ui-tests.vcxproj links
// view3d-ui.vcxproj's static library directly to make UiEngine reachable here.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	// Completion controls use existing quads, with no accepted-state mutation during animation.
	PRUnitTest(ProgressBarDrawsNormalizedCompletionAndHostTimedActivity, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300, 100));
		auto progress = MakeControl(2, 1, EControlType::ProgressBar, ELayoutMode::Overlay, Lp(100, 20));
		auto style = MakeStyle(10);
		for (auto& visual : style.visuals)
		{
			visual.border_thickness = 2;
			visual.corner_radius = 6;
			visual.foreground = Colour{ 0, 1, 0, 1 };
		}
		progress.style_id = 10;
		auto txn = TxnBuilder{};
		txn.AddStyle(style);
		txn.Upsert(root);
		txn.Upsert(progress);
		engine.TransactionApply(txn.Build(0, 1));
		engine.Update(Viewport(300, 100));
		PR_EXPECT(engine.DrawPackets().items.size() == 2); // Root and empty track.

		// Endpoints and fractional completion fill only the track's interior.
		auto revision = std::uint64_t{ 1 };
		for (auto value : { 0.25f, 1.0f })
		{
			progress.value = value;
			auto change = TxnBuilder{};
			change.Upsert(progress);
			engine.TransactionApply(change.Build(revision, revision + 1));
			++revision;
			engine.Update(Viewport(300, 100));
			auto const& item = engine.DrawPackets().items.back();
			PR_EXPECT(engine.DrawPackets().items.size() == 3);
			PR_EXPECT(item.bounds.x == 2 && item.bounds.y == 2);
			PR_EXPECT(item.bounds.w == 96 * value && item.bounds.h == 16);
			PR_EXPECT(item.fill.g == 1 && item.fill.r == 0);
			PR_EXPECT(item.corner_radius == 4 && item.border_thickness == 0);
		}

		// The same descriptor moves without transactions and loops continuously in host time.
		progress.is_indeterminate = 1;
		auto change = TxnBuilder{};
		change.Upsert(progress);
		engine.TransactionApply(change.Build(revision, revision + 1));
		++revision;
		for (auto time_ms : { 0.0, 600.0, 1200.0, 1800.0, 2400.0 })
		{
			engine.Update(Viewport(300, 100, 96, time_ms));
			auto const& item = engine.DrawPackets().items.back();
			auto const expected_x = time_ms == 1200 ? 74.0f : time_ms == 600 || time_ms == 1800 ? 38.0f : 2.0f;
			PR_EXPECT(std::abs(item.bounds.x - expected_x) < 0.001f);
			PR_EXPECT(item.bounds.w == 24 && item.bounds.x + item.bounds.w <= 98);
			PR_EXPECT(engine.DrawPackets().accepted_revision == revision);
			PR_EXPECT(engine.EventCount() == 0);
		}

		// Hidden and collapsed indicators do not keep emitting activity geometry.
		for (auto visibility : { EVisibility::Hidden, EVisibility::Collapsed })
		{
			progress.visibility = visibility;
			auto hidden = TxnBuilder{};
			hidden.Upsert(progress);
			engine.TransactionApply(hidden.Build(revision, revision + 1));
			++revision;
			engine.Update(Viewport(300, 100, 192, 600));
			PR_EXPECT(engine.DrawPackets().items.size() == 1);
		}
	}

	// Invalid percentages and timestamps fail explicitly without replacing accepted progress.
	PRUnitTest(ProgressBarRejectsInvalidValuesAndTime, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto txn = TxnBuilder{};
		txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300, 100)));
		auto progress = MakeControl(2, 1, EControlType::ProgressBar, ELayoutMode::Overlay, Lp(100, 20));
		progress.value = 0.5f;
		txn.Upsert(progress);
		engine.TransactionApply(txn.Build(0, 1));
		engine.Update(Viewport(300, 100));
		for (auto value : { -0.01f, 1.01f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN() })
		{
			progress.value = value;
			progress.is_indeterminate = 1;
			auto rejected = TxnBuilder{};
			rejected.Upsert(progress);
			PR_THROWS(engine.TransactionApply(rejected.Build(1, 2)), EngineException);
			PR_EXPECT(engine.DrawPackets().accepted_revision == 1);
		}
		PR_THROWS(engine.Update(Viewport(300, 100, 96, std::numeric_limits<double>::quiet_NaN())), EngineException);
		PR_EXPECT(engine.DrawPackets().accepted_revision == 1);
	}

	// Progress uses normal template validation, world scaling, and state interpolation.
	PRUnitTest(ProgressBarTemplatesScalingAndForegroundTransitions, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto world = WorldParams(Vec3{ 0, 0, 10 });
		world.sizing = EWorldSizing::WorldUnits;
		world.world_units_per_dip = 0.02f;
		auto progress = MakeControl(2, 1, EControlType::ProgressBar, ELayoutMode::Overlay, Lp(100, 20));
		progress.value = 0.5f;
		progress.style_id = 10;
		progress.template_id = 20;
		auto style = MakeStyle(10);
		for (auto& visual : style.visuals)
		{
			visual.border_thickness = 2;
			visual.foreground = Colour{ 0, 1, 0, 1 };
		}
		auto const disabled = static_cast<std::size_t>(EStateChannel::Disabled);
		style.visuals[disabled].foreground = Colour{ 1, 0, 0, 1 };
		style.transitions[disabled].duration_ms = 100;
		auto txn = TxnBuilder{};
		auto parts = MakeTemplate(20, EControlType::ProgressBar);
		txn.AddTemplatePart(parts, "PART_Track", EVisualPrimitive::SolidBox);
		auto incomplete = TxnBuilder{};
		auto missing_indicator = MakeTemplate(20, EControlType::ProgressBar);
		incomplete.AddTemplatePart(missing_indicator, "PART_Track", EVisualPrimitive::SolidBox);
		incomplete.AddTemplate(missing_indicator);
		try
		{
			engine.TransactionApply(incomplete.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (EngineException const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidTree);
		}
		txn.AddTemplatePart(parts, "PART_Indicator", EVisualPrimitive::SolidBox);
		txn.AddTemplate(parts);
		txn.AddStyle(style);
		txn.Upsert(MakeWorldRoot(1, ERootPolicy::Overlay, 200, 100, world));
		txn.Upsert(progress);
		engine.TransactionApply(txn.Build(0, 1));
		auto viewport = Viewport(800, 600, 96, 0, OrthographicCamera(6));
		engine.Update(viewport);
		auto const first = engine.DrawPackets().items.back();
		PR_EXPECT(std::abs(first.bounds.w - 96) < 0.01f);
		PR_EXPECT(std::abs(first.bounds.h - 32) < 0.01f);

		// State changes interpolate foreground with the same transition timing as the track.
		progress.enabled = 0;
		auto disable = TxnBuilder{};
		disable.Upsert(progress);
		engine.TransactionApply(disable.Build(1, 2));
		engine.Update(viewport);
		viewport.time_ms = 50;
		engine.Update(viewport);
		auto const halfway = engine.DrawPackets().items.back();
		PR_EXPECT(halfway.fill.r == 0.5f && halfway.fill.g == 0.5f);
		viewport.time_ms = 100;
		engine.Update(viewport);
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 1);

		// Zero-size tracks remain valid controls but emit no indicator.
		progress.layout.width = 0;
		auto empty = TxnBuilder{};
		empty.Upsert(progress);
		engine.TransactionApply(empty.Build(2, 3));
		engine.Update(viewport);
		PR_EXPECT(engine.DrawPackets().items.size() == 2);
	}

	PRUnitTest(DrawPacketEmitsAStablePreOrderBoxThenTextItemSequence, Quick)
	{
		// Root(1) -> [Panel(2) -> Text(3, "Hi"), Button(4, "OK")]: a bare Text control emits only
		// its own text item (no box), while Root/Panel/Button each emit a box first; children are
		// visited in the order they were inserted (draw_packet_builder.cpp's Walk()).
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

		auto text_ctrl = MakeControl(3, 2, EControlType::Text, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(text_ctrl.text_offset, text_ctrl.text_length) = b0.AddText("Hi");
		b0.Upsert(text_ctrl);

		auto button_ctrl = MakeControl(4, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(button_ctrl.text_offset, button_ctrl.text_length) = b0.AddText("OK");
		b0.Upsert(button_ctrl);

		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		auto const& packet = engine.DrawPackets();
		PR_EXPECT(packet.items.size() == 5);
		PR_EXPECT(packet.items[0].control_id == 1); // Root box
		PR_EXPECT(packet.items[0].primitive != EVisualPrimitive::TextPresenter);
		PR_EXPECT(packet.items[1].control_id == 2); // Panel box
		PR_EXPECT(packet.items[1].primitive != EVisualPrimitive::TextPresenter);
		PR_EXPECT(packet.items[2].control_id == 3); // Text has no box, only its text item
		PR_EXPECT(packet.items[2].primitive == EVisualPrimitive::TextPresenter);
		PR_EXPECT(packet.items[2].text == "Hi");
		PR_EXPECT(packet.items[3].control_id == 4); // Button box
		PR_EXPECT(packet.items[3].primitive != EVisualPrimitive::TextPresenter);
		PR_EXPECT(packet.items[4].control_id == 4); // Button label text, after its own box
		PR_EXPECT(packet.items[4].primitive == EVisualPrimitive::TextPresenter);
		PR_EXPECT(packet.items[4].text == "OK");
	}

	PRUnitTest(DrawPacketPaintsAFocusedTextBoxLivePendingTextNotItsCommittedText, Quick)
	{
		// A focused TextBox that has been typed into must draw exactly what the user currently
		// sees mid-edit, not the last accepted value, matching draw_packet_builder.cpp's documented
		// "live pending text always takes priority" contract.
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto text_box = MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		std::tie(text_box.text_offset, text_box.text_length) = b0.AddText("Hi");
		b0.Upsert(text_box);
		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		auto const TextItemOf = [&](ControlId id) -> DrawItem const&
		{
			auto const& packet = engine.DrawPackets();
			for (auto const& item : packet.items)
			{
				if (item.control_id == id && item.primitive == EVisualPrimitive::TextPresenter)
					return item;
			}
			throw std::runtime_error("test helper: no text item found for the requested control id");
		};

		// Before any focus/edit, the draw packet must still show the committed text unchanged.
		PR_EXPECT(TextItemOf(2).text == "Hi");

		// TextBox(2) is the only focusable control, so one Tab focuses it directly; typing 'X'
		// appends to the end of the lazily-initialized pending text (input.cpp's
		// GetOrInitTextEdit seeds pending_text from the accepted text on first use).
		engine.InputInject(KeyDownInput(VK_TAB));
		engine.InputInject(CharInputRecord('X'));
		engine.Update(Viewport(300, 200));

		PR_EXPECT(TextItemOf(2).text == "HiX");
	}

	PRUnitTest(DrawPacketResolvesTextFillFromTheReferencedFontResourceNotTheBoxBorderColour, Quick)
	{
		// Regression guard for the M3 review bug where a text item's fill colour was wired to
		// visual.border_colour (the resolved box border), so a focused/invalid TextBox's border-
		// colour-driven state channel recoloured its text too. A distinct style border colour and
		// a distinct Font resource colour must never be confused with each other.
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		auto style = MakeStyle(10);
		auto const border_channel = static_cast<std::size_t>(EStateChannel::Normal);
		style.visuals[border_channel].border_colour = Colour{ 1.0f, 0.0f, 0.0f, 1.0f }; // red border
		b0.AddStyle(style);
		b0.AddResource(MakeResource(20, EResourceKind::Font, Colour{ 0.0f, 1.0f, 0.0f, 1.0f })); // green text

		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto text_ctrl = MakeControl(2, 1, EControlType::Text, ELayoutMode::Overlay, Lp(50.0f, 20.0f), 20);
		text_ctrl.style_id = 10;
		std::tie(text_ctrl.text_offset, text_ctrl.text_length) = b0.AddText("Hi");
		b0.Upsert(text_ctrl);

		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		auto const& packet = engine.DrawPackets();
		auto const& text_item = packet.items.back();
		PR_EXPECT(text_item.primitive == EVisualPrimitive::TextPresenter);
		PR_EXPECT(text_item.fill.r == 0.0f && text_item.fill.g == 1.0f && text_item.fill.b == 0.0f && text_item.fill.a == 1.0f);
		PR_EXPECT(text_item.border_colour.r == 1.0f); // the resolved style border colour is unaffected
	}

	PRUnitTest(DrawPacketFallsBackToOpaqueBlackTextFillWhenNoUsableFontColourIsSpecified, Quick)
	{
		// Both "no font_resource_id at all" and "a Font resource whose colour was left zero-
		// initialized" (the pre-existing "'colour' is unused for Font" contract) must resolve to
		// a visible fallback rather than an invisible (alpha-zero) or otherwise arbitrary colour.
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(20, EResourceKind::Font, Colour{ 0.3f, 0.4f, 0.5f, 0.0f })); // alpha 0 => "unset"

		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto no_font_ctrl = MakeControl(2, 1, EControlType::Text, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(no_font_ctrl.text_offset, no_font_ctrl.text_length) = b0.AddText("A");
		b0.Upsert(no_font_ctrl);
		auto zero_colour_ctrl = MakeControl(3, 1, EControlType::Text, ELayoutMode::Overlay, Lp(50.0f, 20.0f), 20);
		std::tie(zero_colour_ctrl.text_offset, zero_colour_ctrl.text_length) = b0.AddText("B");
		b0.Upsert(zero_colour_ctrl);

		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		auto const& packet = engine.DrawPackets();
		auto const TextItemOf = [&](ControlId id) -> DrawItem const&
		{
			for (auto const& item : packet.items)
			{
				if (item.control_id == id && item.primitive == EVisualPrimitive::TextPresenter)
					return item;
			}
			throw std::runtime_error("test helper: no text item found for the requested control id");
		};

		auto const expect_black = [](Colour const& c)
		{
			PR_EXPECT(c.r == 0.0f && c.g == 0.0f && c.b == 0.0f && c.a == 1.0f);
		};
		expect_black(TextItemOf(2).fill);
		expect_black(TextItemOf(3).fill);
	}

	PRUnitTest(DrawPacketAssignsTextPlacementByControlTypeWithTextBoxInsetAndButtonCentered, Quick)
	{
		// Static Text stays flush-left with no inset (it has no box of its own); a TextBox insets
		// its content from the left edge like a conventional text field; a Button centers its
		// label - centering itself is computed by the renderer from the shaped run's advance, but
		// the intent (ETextAlign::Center, ignoring inset) is decided here and must survive intact.
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

		auto text_ctrl = MakeControl(2, 1, EControlType::Text, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(text_ctrl.text_offset, text_ctrl.text_length) = b0.AddText("A");
		b0.Upsert(text_ctrl);

		auto button_ctrl = MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(button_ctrl.text_offset, button_ctrl.text_length) = b0.AddText("OK");
		b0.Upsert(button_ctrl);

		auto text_box_ctrl = MakeControl(4, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		std::tie(text_box_ctrl.text_offset, text_box_ctrl.text_length) = b0.AddText("Hi");
		b0.Upsert(text_box_ctrl);

		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		auto const& packet = engine.DrawPackets();
		auto const TextItemOf = [&](ControlId id) -> DrawItem const&
		{
			for (auto const& item : packet.items)
			{
				if (item.control_id == id && item.primitive == EVisualPrimitive::TextPresenter)
					return item;
			}
			throw std::runtime_error("test helper: no text item found for the requested control id");
		};

		PR_EXPECT(TextItemOf(2).text_align == ETextAlign::Left);
		PR_EXPECT(TextItemOf(2).text_inset_dip == 0.0f);
		PR_EXPECT(TextItemOf(3).text_align == ETextAlign::Center);
		PR_EXPECT(TextItemOf(3).text_inset_dip == 0.0f);
		PR_EXPECT(TextItemOf(4).text_align == ETextAlign::Left);
		PR_EXPECT(TextItemOf(4).text_inset_dip == 8.0f);
	}
}
