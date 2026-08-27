//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M5 tests (implementation-plan.md's native-hardening milestone): Scroll/Canvas layout, the
// Selected/Visibility/ValueChanged style channels, resource/style/template removal semantics,
// bounds/overflow/lifecycle stress, and determinism across repeated Update() calls.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// A Root containing one focusable, enabled Button, ready for click/command sequences.
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

		// Locate one control's computed bounds/state within a drained semantic snapshot.
		SemanticNode const& NodeOf(std::vector<SemanticNode> const& nodes, ControlId id)
		{
			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			if (it == nodes.end())
				throw std::runtime_error("test helper: semantic node not found for the requested control id");

			return *it;
		}

		std::vector<SemanticNode> SnapshotSemantics(UiContext& ctx)
		{
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			return nodes;
		}
	}

	//------------------------------------------------------------------------------------------
	// Scroll/Canvas layout (M5 item 1)
	//------------------------------------------------------------------------------------------

	PRUnitTest(LayoutCanvasPlacesChildAtAbsoluteOffsetIgnoringAlignmentAndMargins, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Canvas, Lp(300.0f, 200.0f)));
		auto child = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f, EHAlign::Right, EVAlign::Bottom));
		child.layout.margin_left = 100.0f; // must be ignored: Canvas position is already explicit
		child.layout.canvas_x = 40.0f;
		child.layout.canvas_y = 15.0f;
		b0.Upsert(child);
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		auto nodes1 = SnapshotSemantics(ctx);
		auto const& node = NodeOf(nodes1, 2);
		PR_EXPECT(node.bounds.x == 40.0f);
		PR_EXPECT(node.bounds.y == 15.0f);
		PR_EXPECT(node.bounds.w == 50.0f);
		PR_EXPECT(node.bounds.h == 20.0f);
	}

	PRUnitTest(LayoutCanvasAcceptsNegativePositionsPartiallyOffTheParentEdge, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Canvas, Lp(300.0f, 200.0f)));
		auto child = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		child.layout.canvas_x = -10.0f;
		child.layout.canvas_y = -5.0f;
		b0.Upsert(child);
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		auto nodes2 = SnapshotSemantics(ctx);
		auto const& node = NodeOf(nodes2, 2);
		PR_EXPECT(node.bounds.x == -10.0f);
		PR_EXPECT(node.bounds.y == -5.0f);
	}

	PRUnitTest(LayoutScrollShiftsChildPlacementByNegativeScrollOffset, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::Scroll, Lp(300.0f, 200.0f));
		root.layout.scroll_offset_x = 10.0f;
		root.layout.scroll_offset_y = 5.0f;
		b0.Upsert(root);
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f, EHAlign::Left, EVAlign::Top)));
		b0.Upsert(MakeControl(3, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f, EHAlign::Right, EVAlign::Bottom)));
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		auto nodes = SnapshotSemantics(ctx);

		// Top/Left: content is shifted by (-scroll_offset_x, -scroll_offset_y) before Overlay's
		// usual placement rule runs, so a Left/Top child's near edge lands at -scroll_offset.
		auto const& top_left = NodeOf(nodes, 2);
		PR_EXPECT(top_left.bounds.x == -10.0f);
		PR_EXPECT(top_left.bounds.y == -5.0f);

		// Bottom/Right: (avail - size) is computed against the shifted content rect too, so the
		// far edge moves by the same amount as the near edge did above.
		auto const& bottom_right = NodeOf(nodes, 3);
		PR_EXPECT(bottom_right.bounds.x == 300.0f - 50.0f - 10.0f);
		PR_EXPECT(bottom_right.bounds.y == 200.0f - 20.0f - 5.0f);
	}

	PRUnitTest(TransactionApplyRejectsNonFiniteCanvasPosition, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Canvas, Lp(300.0f, 200.0f)));
		auto child = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		child.layout.canvas_x = std::numeric_limits<float>::quiet_NaN();
		b0.Upsert(child);
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidArgument);
		}
	}

	PRUnitTest(TransactionApplyRejectsNegativeScrollOffset, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::Scroll, Lp(300.0f, 200.0f));
		root.layout.scroll_offset_x = -1.0f;
		b0.Upsert(root);
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidArgument);
		}
	}

	//------------------------------------------------------------------------------------------
	// Selected/Visibility/ValueChanged style channels (M5 item 2)
	//------------------------------------------------------------------------------------------

	PRUnitTest(StyleSelectedChannelAppliesWhenSetAndOverriddenByFocusedAndPressed, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		auto style = MakeStyle(10);
		style.visuals[static_cast<std::size_t>(EStateChannel::Normal)].fill = Colour{ 0.1f, 0.1f, 0.1f, 1.0f };
		style.visuals[static_cast<std::size_t>(EStateChannel::Selected)].fill = Colour{ 0.2f, 0.2f, 0.2f, 1.0f };
		style.visuals[static_cast<std::size_t>(EStateChannel::Focused)].fill = Colour{ 0.3f, 0.3f, 0.3f, 1.0f };
		b0.AddStyle(style);
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto btn = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		btn.style_id = 10;
		btn.selected = 1;
		b0.Upsert(btn);
		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		// Not hovered/pressed/focused: Selected wins over Normal. The Root paints its own
		// (unstyled) box first, so the Button box is the last item, not the first.
		auto const& box1 = engine.DrawPackets().items.back();
		PR_EXPECT(box1.fill.r == 0.2f);

		// Give the same control focus (Tab lands on the only focusable control); Focused must
		// still take priority over the durable Selected flag (section 6.4's priority order).
		engine.InputInject(KeyDownInput(VK_TAB));
		engine.Update(Viewport(300, 200));
		auto const& box2 = engine.DrawPackets().items.back();
		PR_EXPECT(box2.fill.r == 0.3f);
	}

	PRUnitTest(StyleVisibilityChannelAppliesOnlyOnTheUpdateThatMakesAnAlreadySeenControlVisibleAgain, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		auto style = MakeStyle(10);
		style.visuals[static_cast<std::size_t>(EStateChannel::Normal)].fill = Colour{ 0.1f, 0.1f, 0.1f, 1.0f };
		style.visuals[static_cast<std::size_t>(EStateChannel::Visibility)].fill = Colour{ 0.9f, 0.9f, 0.9f, 1.0f };
		b0.AddStyle(style);
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		panel.style_id = 10;
		b0.Upsert(panel);
		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));

		// A brand-new already-visible control must never spuriously transition on its own first
		// Update() (StyleRuntimeState::was_visible defaults to 1 for exactly this reason).
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 0.1f);

		// Hide it (Walk() skips its whole subtree and marks it not-visible via MarkInvisible), then
		// show it again: the very next Update() that observes it visible must resolve Visibility.
		auto b1 = TxnBuilder{};
		auto hidden = panel;
		hidden.visible = 0;
		b1.Upsert(hidden);
		engine.TransactionApply(b1.Build(1, 2));
		engine.Update(Viewport(300, 200)); // control emits no box while hidden

		auto b2 = TxnBuilder{};
		auto shown = panel;
		shown.visible = 1;
		b2.Upsert(shown);
		engine.TransactionApply(b2.Build(2, 3));
		engine.Update(Viewport(300, 200));
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 0.9f);

		// A subsequent Update() with nothing changed must fall back to Normal again (Visibility
		// only fires for the one Update() call that observed the off-to-on transition).
		engine.Update(Viewport(300, 200));
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 0.1f);
	}

	PRUnitTest(StyleValueChangedChannelAppliesOnlyOnTheUpdateThatObservesANewValueSequence, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		auto style = MakeStyle(10);
		style.visuals[static_cast<std::size_t>(EStateChannel::Normal)].fill = Colour{ 0.1f, 0.1f, 0.1f, 1.0f };
		style.visuals[static_cast<std::size_t>(EStateChannel::ValueChanged)].fill = Colour{ 0.7f, 0.7f, 0.7f, 1.0f };
		b0.AddStyle(style);
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		panel.style_id = 10;
		panel.value_sequence = 1;
		b0.Upsert(panel);
		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200));
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 0.1f); // first observation: no "changed" edge yet

		auto b1 = TxnBuilder{};
		auto bumped = panel;
		bumped.value_sequence = 2;
		b1.Upsert(bumped);
		engine.TransactionApply(b1.Build(1, 2));
		engine.Update(Viewport(300, 200));
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 0.7f);

		engine.Update(Viewport(300, 200)); // nothing changed since: reverts to Normal
		PR_EXPECT(engine.DrawPackets().items.back().fill.r == 0.1f);
	}

	PRUnitTest(SemanticSnapshotReportsSelectedStateFlagFromControlDescSelected, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto btn = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		btn.selected = 1;
		b0.Upsert(btn);
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		auto nodes3 = SnapshotSemantics(ctx);
		auto const& node = NodeOf(nodes3, 2);
		PR_EXPECT((node.state_flags & static_cast<std::uint32_t>(ESemanticState::Selected)) != 0);
	}

	//------------------------------------------------------------------------------------------
	// Resource/style/template removal semantics (M5 item 3)
	//------------------------------------------------------------------------------------------

	PRUnitTest(TransactionApplyRemovesAnUnreferencedResourceStyleAndTemplate, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(20, EResourceKind::Colour, Colour{ 1, 1, 1, 1 }));
		b0.AddStyle(MakeStyle(10));
		auto panel_tmpl = MakeTemplate(30, EControlType::Panel);
		b0.AddTemplatePart(panel_tmpl, "PART_ContentPresenter", EVisualPrimitive::ContentPresenter, true);
		b0.AddTemplate(panel_tmpl);
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		auto b1 = TxnBuilder{};
		b1.RemoveResource(20);
		b1.RemoveStyle(10);
		b1.RemoveTemplate(30);
		ctx.TransactionApply(b1.Build(1, 2)); // must not throw: none of the three ids are referenced

		// The freed ids must be immediately reusable (new records, not stale ones).
		auto b2 = TxnBuilder{};
		b2.AddResource(MakeResource(20, EResourceKind::Font, Colour{ 0, 0, 0, 1 }));
		ctx.TransactionApply(b2.Build(2, 3));
	}

	PRUnitTest(TransactionApplyRejectsRemovalOfAnUnknownResourceStyleOrTemplateId, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto try_remove = [&](std::function<void(TxnBuilder&)> const& setup)
		{
			auto b0 = TxnBuilder{};
			setup(b0);
			try
			{
				ctx.TransactionApply(b0.Build(0, 1));
				PR_EXPECT(false);
			}
			catch (Exception const& ex)
			{
				PR_EXPECT(ex.Status() == EStatus::UnknownResource);
			}
		};
		try_remove([](TxnBuilder& b) { b.RemoveResource(999); });
		try_remove([](TxnBuilder& b) { b.RemoveStyle(999); });
		try_remove([](TxnBuilder& b) { b.RemoveTemplate(999); });
	}

	PRUnitTest(TransactionApplyRejectsRemovalOfAResourceStyleOrTemplateStillReferencedByALiveControl, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(20, EResourceKind::Font, Colour{ 0, 0, 0, 1 }));
		b0.AddStyle(MakeStyle(10));
		auto tmpl = MakeTemplate(30, EControlType::Button);
		b0.AddTemplatePart(tmpl, "PART_ContentPresenter", EVisualPrimitive::SolidBox, false);
		b0.AddTemplatePart(tmpl, "PART_FocusOutline", EVisualPrimitive::Border, false);
		b0.AddTemplate(tmpl);
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		auto btn = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f), 20);
		btn.style_id = 10;
		btn.template_id = 30;
		b0.Upsert(btn);
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		auto try_remove_rejected = [&](std::function<void(TxnBuilder&)> const& setup, std::uint64_t base_rev)
		{
			auto b = TxnBuilder{};
			setup(b);
			try
			{
				ctx.TransactionApply(b.Build(base_rev, base_rev + 1));
				PR_EXPECT(false);
			}
			catch (Exception const& ex)
			{
				PR_EXPECT(ex.Status() == EStatus::ResourceInUse);
			}
		};
		try_remove_rejected([](TxnBuilder& b) { b.RemoveResource(20); }, 1);
		try_remove_rejected([](TxnBuilder& b) { b.RemoveStyle(10); }, 1);
		try_remove_rejected([](TxnBuilder& b) { b.RemoveTemplate(30); }, 1);

		// Atomicity: every rejected removal above must have left the prior snapshot untouched, so
		// the button (still referencing all three ids) must still be present and unaffected.
		auto nodes = SnapshotSemantics(ctx);
		PR_EXPECT(std::any_of(nodes.begin(), nodes.end(), [](SemanticNode const& n) { return n.id == 2; }));
	}

	PRUnitTest(TransactionApplyRemovalSucceedsOnceTheReferencingControlIsRemovedInTheSameTransaction, Quick)
	{
		// A single transaction may both stop referencing an id (by removing the control) and
		// remove that id, because the removal loop runs after every Upsert/Remove operation has
		// already applied (tree.cpp's ordering contract).
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(20, EResourceKind::Font, Colour{ 1, 1, 1, 1 }));
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f), 20);
		b0.Upsert(panel);
		ctx.TransactionApply(b0.Build(0, 1));

		auto b1 = TxnBuilder{};
		b1.Remove(2);
		b1.RemoveResource(20);
		ctx.TransactionApply(b1.Build(1, 2)); // must not throw
	}

	//------------------------------------------------------------------------------------------
	// Bounds/limit stress (M5 item 4)
	//------------------------------------------------------------------------------------------

	PRUnitTest(TransactionApplyEnforcesMaxTreeDepthAsInvalidTreeNotResourceLimit, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto config = MakeConfig(4096, 64, /*max_tree_depth*/ 4);
		auto ctx = UiContext(runtime, &config, &device);

		// A chain of five controls (Root + 4 nested Panels) exceeds max_tree_depth == 4.
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		for (auto id = ControlId{2}; id != 7; ++id)
			b0.Upsert(MakeControl(id, id - 1, EControlType::Panel, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidTree);
		}
	}

	PRUnitTest(TransactionApplyEnforcesMaxBlobBytesResourceLimit, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto config = MakeConfig(4096, 64, 64, 4096, /*max_blob_bytes*/ 4);
		auto ctx = UiContext(runtime, &config, &device);

		auto b0 = TxnBuilder{};
		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f));
		std::tie(root.text_offset, root.text_length) = b0.AddText("too long for 4 bytes");
		b0.Upsert(root);
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::ResourceLimit);
		}
	}

	PRUnitTest(TransactionApplyEnforcesMaxResourcesStylesAndTemplatesResourceLimit, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto config = MakeConfig(4096, 64, 64, 4096, 1u << 20, 256, 256, /*max_resources*/ 1);
		auto ctx = UiContext(runtime, &config, &device);

		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(20, EResourceKind::Colour, Colour{ 1, 1, 1, 1 }));
		b0.AddResource(MakeResource(21, EResourceKind::Colour, Colour{ 1, 1, 1, 1 }));
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::ResourceLimit);
		}
	}

	PRUnitTest(EngineUpdateEnforcesMaxSemanticRecordsResourceLimitWithoutCorruptingThePublishedSnapshot, Quick)
	{
		// White-box (UiEngine directly): max_semantic_records is only checked inside Update(), and
		// M5's atomicity fix requires that a rejected Update() never overwrite the previously
		// published semantics/draw-packet with the over-limit values it computed internally.
		auto config = MakeConfig(4096, 64, 64, 4096, 1u << 20, 256, 256, 256, 1024, /*max_semantic_records*/ 2);
		auto engine = UiEngine(config);
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f)));
		engine.TransactionApply(b0.Build(0, 1));
		engine.Update(Viewport(300, 200)); // exactly at the limit: 2 semantic records, must succeed
		PR_EXPECT(engine.SemanticCount() == 2);

		auto b1 = TxnBuilder{};
		b1.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		b1.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f)));
		b1.Upsert(MakeControl(3, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f)));
		engine.TransactionApply(b1.Build(1, 2)); // the tree itself accepts 3 controls fine
		try
		{
			engine.Update(Viewport(300, 200)); // ...but 3 semantic records exceeds the limit of 2
			PR_EXPECT(false);
		}
		catch (EngineException const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::ResourceLimit);
		}

		// The previously published (2-record) snapshot must still be exactly what it was.
		PR_EXPECT(engine.SemanticCount() == 2);
	}

	//------------------------------------------------------------------------------------------
	// Queue overflow recovery, repeated lifecycle, resize/DPI churn, font diagnostics (M5 item 4)
	//------------------------------------------------------------------------------------------

	PRUnitTest(EventQueueAcceptsNewEventsNormallyAfterAPriorOverflowFault, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto config = MakeConfig(4096, 64, 64, 4096, 1u << 20, 256, 256, 256, /*max_queued_events*/ 3);
		auto ctx = UiContext(runtime, &config, &device);
		BuildRootWithButton(ctx, 2, 10.0f, 10.0f, 50.0f, 20.0f);

		ctx.InputInject(KeyDownInput(VK_TAB));    // FocusChanged
		ctx.InputInject(KeyDownInput(VK_RETURN)); // CommandInvoked
		ctx.InputInject(KeyDownInput(VK_RETURN)); // CommandInvoked, evicts FocusChanged
		try
		{
			ctx.InputInject(KeyDownInput(VK_RETURN)); // overflow: no coalescible victim left
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::QueueOverflow);
		}

		// Draining the queue (including its overflow marker) must restore normal operation: a
		// fresh command afterward must queue and drain exactly like any other CommandInvoked.
		auto sizes = ctx.EventsPendingSizes();
		auto events = std::vector<Event>(sizes.m_count);
		auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
		ctx.EventsCopy(events, payload);
		PR_EXPECT(ctx.EventCount() == 0);

		ctx.InputInject(KeyDownInput(VK_RETURN)); // CommandInvoked, queue is empty again
		PR_EXPECT(ctx.EventCount() == 1);
	}

	PRUnitTest(RepeatedContextCreateUpdateDestroyCyclesLeaveNoDeviceRefcountOrStateLeak, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto refcount_before = device.RefCount();

		for (auto i = 0; i != 25; ++i)
		{
			auto ctx = UiContext(runtime, &device);
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(200 + i, 150 + i));
			// ctx destructs here, calling View3DUI_ContextDestroy.
		}

		PR_EXPECT(device.RefCount() == refcount_before);
	}

	PRUnitTest(RepeatedResizeAndDpiChurnProducesDeterministicAutoSizedRootBoundsEachTime, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f))); // 0x0 => auto-sizes to the viewport
		ctx.TransactionApply(b0.Build(0, 1));

		// Repeated resize/DPI changes must each independently recompute the root's DIP-space
		// bounds from client_width_px/height_px and dpi; there is no residual/blended state.
		ctx.Update(Viewport(800, 600, 96.0f));
		PR_EXPECT(NodeOf(SnapshotSemantics(ctx), 1).bounds.w == 800.0f);
		PR_EXPECT(NodeOf(SnapshotSemantics(ctx), 1).bounds.h == 600.0f);

		ctx.Update(Viewport(1920, 1080, 192.0f)); // double DPI halves the DIP-space size
		PR_EXPECT(NodeOf(SnapshotSemantics(ctx), 1).bounds.w == 960.0f);
		PR_EXPECT(NodeOf(SnapshotSemantics(ctx), 1).bounds.h == 540.0f);

		ctx.Update(Viewport(800, 600, 96.0f)); // back to the first size: must reproduce exactly
		PR_EXPECT(NodeOf(SnapshotSemantics(ctx), 1).bounds.w == 800.0f);
		PR_EXPECT(NodeOf(SnapshotSemantics(ctx), 1).bounds.h == 600.0f);
	}

	PRUnitTest(TransactionApplyRejectsAnUnknownFontResourceIdAndSurfacesItInDiagnostics, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto desc = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f), /*font_resource_id*/ 999);
		b0.Upsert(desc);
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::UnknownResource);
		}

		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.last_failure_status == EStatus::UnknownResource);
		PR_EXPECT(diagnostics.rejected_revision_attempts >= 1);
		PR_EXPECT(diagnostics.accepted_revision == 0); // the rejected attempt never advanced the accepted tree
	}

	PRUnitTest(TransactionApplyRejectsAResourceIdThatDoesNotReferenceAFontResource, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(20, EResourceKind::Colour, Colour{ 1, 1, 1, 1 })); // not a Font resource
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f), 20));
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::UnknownResource);
		}
	}

	//------------------------------------------------------------------------------------------
	// Determinism across repeated Update() calls with unchanged state (M5 item 4)
	//------------------------------------------------------------------------------------------

	PRUnitTest(RepeatedUpdateCallsWithUnchangedStateProduceIdenticalLayoutSemanticsAndDrawOrder, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::StackVertical, Lp(300.0f, 200.0f)));
		auto text_ctrl = MakeControl(3, 2, EControlType::Text, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(text_ctrl.text_offset, text_ctrl.text_length) = b0.AddText("Hi");
		b0.Upsert(text_ctrl);
		auto button_ctrl = MakeControl(4, 2, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		std::tie(button_ctrl.text_offset, button_ctrl.text_length) = b0.AddText("OK");
		b0.Upsert(button_ctrl);
		engine.TransactionApply(b0.Build(0, 1));

		engine.Update(Viewport(300, 200, 96.0f, 0.0));
		auto const first_items = engine.DrawPackets().items;
		auto const first_semantic_count = engine.SemanticCount();

		for (auto i = 0; i != 5; ++i)
		{
			engine.Update(Viewport(300, 200, 96.0f, static_cast<double>(i))); // host time advances; nothing else changes
			auto const& items = engine.DrawPackets().items;
			PR_EXPECT(items.size() == first_items.size());
			for (auto k = std::size_t{}; k != items.size(); ++k)
			{
				PR_EXPECT(items[k].control_id == first_items[k].control_id);
				PR_EXPECT(items[k].primitive == first_items[k].primitive);
				PR_EXPECT(items[k].bounds.x == first_items[k].bounds.x);
				PR_EXPECT(items[k].bounds.y == first_items[k].bounds.y);
				PR_EXPECT(items[k].text == first_items[k].text);
			}
			PR_EXPECT(engine.SemanticCount() == first_semantic_count);
		}
	}
}
