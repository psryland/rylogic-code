//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M2 tests (implementation-plan.md section 6): deterministic retained layout for Overlay and
// horizontal/vertical Stack in DIPs, root auto-sizing to the viewport, and Reorder actually
// changing the resulting stacking order (not just being accepted).
#include "pr/common/unittests.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Locate one control's computed bounds within a drained semantic snapshot. Throws (a
		// plain std::runtime_error, not a View3DUI Exception) if 'id' is absent, which is always
		// a test-authoring bug rather than a case a test should tolerate.
		Rect const& BoundsOf(std::vector<SemanticNode> const& nodes, ControlId id)
		{
			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			if (it == nodes.end())
				throw std::runtime_error("test helper: semantic node not found for the requested control id");

			return it->bounds;
		}

		bool RectEqual(Rect const& a, Rect const& b)
		{
			return a.x == b.x && a.y == b.y && a.w == b.w && a.h == b.h;
		}

		// Applies 'txn' and calls Update(viewport), returning every control's computed bounds via
		// a fresh semantic snapshot (the only ABI-visible window onto retained layout output).
		std::vector<SemanticNode> ApplyAndSnapshotBounds(UiContext& ctx, Transaction const& txn, ViewportState const& viewport)
		{
			ctx.TransactionApply(txn);
			ctx.Update(viewport);

			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			return nodes;
		}
	}

	PRUnitTest(LayoutOverlayAlignsChildToBottomRightWithinContentRect, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f, EHAlign::Right, EVAlign::Bottom)));

		// Root fills its own fixed 200x100 rect with no padding, so the content rect for
		// Overlay placement is identical to the root rect; Bottom/Right alignment then pins the
		// child's near edge at (avail - size) on each axis (section 6.1).
		auto nodes = ApplyAndSnapshotBounds(ctx, b0.Build(0, 1), Viewport(800, 600));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{ 0.0f, 0.0f, 200.0f, 100.0f }));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{ 150.0f, 80.0f, 50.0f, 20.0f }));
	}

	PRUnitTest(LayoutOverlayCentersChildWithinContentRect, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(40.0f, 20.0f, EHAlign::Center, EVAlign::Center)));

		auto nodes = ApplyAndSnapshotBounds(ctx, b0.Build(0, 1), Viewport(800, 600));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{ 80.0f, 40.0f, 40.0f, 20.0f }));
	}

	PRUnitTest(LayoutOverlayStretchFillsAvailableSpaceMinusMargins, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)));
		auto child = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(1.0f, 1.0f, EHAlign::Stretch, EVAlign::Stretch));
		child.layout.margin_left = 10.0f;
		child.layout.margin_top = 10.0f;
		child.layout.margin_right = 10.0f;
		child.layout.margin_bottom = 10.0f;
		b0.Upsert(child);

		// Stretch ignores the child's own explicit width/height entirely and fills the content
		// rect shrunk by its own margins on every side (section 6.1).
		auto nodes = ApplyAndSnapshotBounds(ctx, b0.Build(0, 1), Viewport(800, 600));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{ 10.0f, 10.0f, 180.0f, 80.0f }));
	}

	PRUnitTest(LayoutStackHorizontalPositionsChildrenSequentiallyWithSpacing, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackHorizontal, Lp(300.0f, 50.0f, EHAlign::Left, EVAlign::Top, /*stack_spacing*/ 5.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(40.0f, 20.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(30.0f, 15.0f)));

		// Main-axis (x) positions accumulate each child's own width plus the container's
		// stack_spacing; cross-axis (y) uses each child's own vertical alignment within the full
		// content height (section 6.2).
		auto nodes = ApplyAndSnapshotBounds(ctx, b0.Build(0, 1), Viewport(800, 600));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{ 0.0f, 0.0f, 40.0f, 20.0f }));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 3), Rect{ 45.0f, 0.0f, 30.0f, 15.0f }));
	}

	PRUnitTest(LayoutStackVerticalPositionsChildrenSequentiallyWithSpacing, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackVertical, Lp(100.0f, 300.0f, EHAlign::Left, EVAlign::Top, /*stack_spacing*/ 8.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 25.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(60.0f, 35.0f)));

		auto nodes = ApplyAndSnapshotBounds(ctx, b0.Build(0, 1), Viewport(800, 600));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{ 0.0f, 0.0f, 50.0f, 25.0f }));
		PR_EXPECT(RectEqual(BoundsOf(nodes, 3), Rect{ 0.0f, 33.0f, 60.0f, 35.0f }));
	}

	PRUnitTest(LayoutRootAutoSizesToViewportDipsWhenWidthAndHeightAreZero, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		// At 96 DPI, 1 physical pixel == 1 DIP, so the root exactly matches the client area.
		{
			ctx.Update(Viewport(800, 600, 96.0f));
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{ 0.0f, 0.0f, 800.0f, 600.0f }));
		}

		// At 192 DPI (200% scaling), the same 800x600 physical client area is only 400x300 DIPs
		// (section 7.4's px-to-DIP conversion).
		{
			ctx.Update(Viewport(800, 600, 192.0f));
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{ 0.0f, 0.0f, 400.0f, 300.0f }));
		}
	}

	PRUnitTest(LayoutReorderActuallyChangesResultingStackOrder, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackHorizontal, Lp(300.0f, 50.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(40.0f, 20.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(30.0f, 20.0f)));
		b0.Upsert(MakeControl(4, 1, EControlType::Button, ELayoutMode::Overlay, Lp(20.0f, 20.0f)));
		auto nodes_before = ApplyAndSnapshotBounds(ctx, b0.Build(0, 1), Viewport(800, 600));
		PR_EXPECT(BoundsOf(nodes_before, 2).x == 0.0f);
		PR_EXPECT(BoundsOf(nodes_before, 3).x == 40.0f);
		PR_EXPECT(BoundsOf(nodes_before, 4).x == 70.0f);

		auto b1 = TxnBuilder{};
		b1.Reorder(1, { 4, 3, 2 });
		auto nodes_after = ApplyAndSnapshotBounds(ctx, b1.Build(1, 2), Viewport(800, 600));

		// The same three controls, re-stacked in the new order: control 4 (width 20) now leads,
		// so it occupies [0,20), then control 3 (width 30) occupies [20,50), then control 2
		// (width 40) occupies [50,90) -- proving Reorder changed actual layout output, not merely
		// the accepted child-order metadata.
		PR_EXPECT(BoundsOf(nodes_after, 4).x == 0.0f);
		PR_EXPECT(BoundsOf(nodes_after, 3).x == 20.0f);
		PR_EXPECT(BoundsOf(nodes_after, 2).x == 50.0f);
	}
}
