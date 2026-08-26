//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M1 tests (implementation-plan.md section 5.3/9.2): atomic revision transactions, staging
// validation for every EOperationKind, bounded capacities, invalid-transaction rollback, closed
// Root/Panel/Text/TextBox/Button control types, and resources/styles/templates descriptor and
// blob-range validation.
#include "pr/common/unittests.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// A single Root control with default layout, the minimum valid first transaction.
		Transaction BuildRootOnly(TxnBuilder& b, ControlId root_id)
		{
			b.Upsert(MakeControl(root_id, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
			return b.Build(0, 1);
		}
	}

	PRUnitTest(TransactionApplyAcceptsRootThenChild, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		ctx.TransactionApply(BuildRootOnly(b0, 1));

		auto b1 = TxnBuilder{};
		b1.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(100.0f, 100.0f)));
		ctx.TransactionApply(b1.Build(1, 2));

		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.accepted_revision == 2);
		PR_EXPECT(diagnostics.control_count == 2);
		PR_EXPECT(diagnostics.rejected_revision_attempts == 0);
	}

	PRUnitTest(TransactionApplyRejectsStaleBaseRevision, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		ctx.TransactionApply(BuildRootOnly(b0, 1));

		// base_revision 0 is stale now that revision 1 has been accepted.
		auto b1 = TxnBuilder{};
		b1.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		try
		{
			ctx.TransactionApply(b1.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::StaleRevision);
		}

		// Rejection must preserve the last accepted snapshot exactly.
		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.accepted_revision == 1);
		PR_EXPECT(diagnostics.control_count == 1);
		PR_EXPECT(diagnostics.rejected_revision_attempts == 1);
	}

	PRUnitTest(TransactionApplyRejectsNonSequentialRevision, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));

		// revision must be exactly base_revision + 1; 2 is not a valid successor to base 0.
		try
		{
			ctx.TransactionApply(b0.Build(0, 2));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidArgument);
		}
	}

	PRUnitTest(TransactionApplyRollbackLeavesTreeUntouched, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		ctx.TransactionApply(BuildRootOnly(b0, 1));

		// A control referencing a parent that does not exist must be rejected wholesale, even
		// though the same transaction also creates other, otherwise-valid controls.
		auto b1 = TxnBuilder{};
		b1.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b1.Upsert(MakeControl(3, 999, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		try
		{
			ctx.TransactionApply(b1.Build(1, 2));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidTree);
		}

		// Neither control 2 nor control 3 must exist: the whole transaction rolled back.
		auto diagnostics = ctx.DiagnosticsGet();
		PR_EXPECT(diagnostics.accepted_revision == 1);
		PR_EXPECT(diagnostics.control_count == 1);
		PR_EXPECT(diagnostics.rejected_revision_attempts == 1);

		// A further transaction must still use base_revision 1 (the rejected attempt did not
		// advance the accepted revision).
		auto b2 = TxnBuilder{};
		b2.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		ctx.TransactionApply(b2.Build(1, 2));
		PR_EXPECT(ctx.DiagnosticsGet().control_count == 2);
	}

	PRUnitTest(TransactionApplyRejectsControlIdZero, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(0, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		PR_THROWS(ctx.TransactionApply(b0.Build(0, 1)), Exception);
	}

	PRUnitTest(TransactionApplyRejectsUnknownControlType, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto desc = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f));
		desc.type = static_cast<EControlType>(9999);
		b0.Upsert(desc);
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::UnknownType);
		}
	}

	PRUnitTest(TransactionApplyRejectsRootWithNonZeroParent, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 7, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
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

	PRUnitTest(TransactionApplyRejectsNonRootWithZeroParent, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
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

	PRUnitTest(TransactionApplyRejectsUnknownLayoutMode, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, static_cast<ELayoutMode>(99), Lp(0.0f, 0.0f)));
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::UnknownType);
		}
	}

	PRUnitTest(TransactionApplyRejectsNonFiniteLayoutDimension, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto desc = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f));
		desc.layout.width = -1.0f;
		b0.Upsert(desc);
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

	PRUnitTest(TransactionApplyRejectsUnknownTemplateOrStyleReference, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto desc = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f));
		desc.template_id = 42;
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
	}

	PRUnitTest(TransactionApplyRejectsReparentingViaUpsert, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		// Re-Upsert of control 2 changing its parent_id from 1 to 3 must be rejected outright.
		auto b1 = TxnBuilder{};
		b1.Upsert(MakeControl(2, 3, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		try
		{
			ctx.TransactionApply(b1.Build(1, 2));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidTree);
		}

		// The correct way to re-parent: Remove then re-Upsert under the new parent.
		auto b2 = TxnBuilder{};
		b2.Remove(2);
		b2.Upsert(MakeControl(2, 3, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		ctx.TransactionApply(b2.Build(1, 2));
		PR_EXPECT(ctx.DiagnosticsGet().accepted_revision == 2);
	}

	PRUnitTest(TransactionApplyRemoveDeletesEntireSubtree, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(3, 2, EControlType::Panel, ELayoutMode::Overlay, Lp(5.0f, 5.0f)));
		ctx.TransactionApply(b0.Build(0, 1));
		PR_EXPECT(ctx.DiagnosticsGet().control_count == 3);

		auto b1 = TxnBuilder{};
		b1.Remove(2);
		ctx.TransactionApply(b1.Build(1, 2));

		// Removing control 2 must also remove its child, control 3.
		PR_EXPECT(ctx.DiagnosticsGet().control_count == 1);
	}

	PRUnitTest(TransactionApplyRemoveRejectsMissingTarget, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		ctx.TransactionApply(BuildRootOnly(b0, 1));

		auto b1 = TxnBuilder{};
		b1.Remove(999);
		try
		{
			ctx.TransactionApply(b1.Build(1, 2));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidTree);
		}
	}

	PRUnitTest(TransactionApplyReplaceSubtreeThenUpsertRecreates, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(3, 2, EControlType::Panel, ELayoutMode::Overlay, Lp(5.0f, 5.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		// ReplaceSubtree deletes control 2 (and its child 3), then the following Upsert recreates
		// control 2 fresh (without control 3), all within one atomic transaction.
		auto b1 = TxnBuilder{};
		b1.ReplaceSubtree(2);
		b1.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(20.0f, 20.0f)));
		ctx.TransactionApply(b1.Build(1, 2));

		PR_EXPECT(ctx.DiagnosticsGet().control_count == 2);
	}

	PRUnitTest(TransactionApplyReorderPermutesChildren, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackHorizontal, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(4, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		// Reverse the children of the root from {2,3,4} to {4,3,2}; verified indirectly via
		// SemanticsCopy's pre-order sequence in tests_m2_layout.cpp. Here we only need the
		// transaction itself to be accepted (a non-permutation would be rejected).
		auto b1 = TxnBuilder{};
		b1.Reorder(1, { 4, 3, 2 });
		ctx.TransactionApply(b1.Build(1, 2));
		PR_EXPECT(ctx.DiagnosticsGet().accepted_revision == 2);
	}

	PRUnitTest(TransactionApplyReorderRejectsNonPermutation, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackHorizontal, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		// {2,2} is not a permutation of the current children {2,3}.
		auto b1 = TxnBuilder{};
		b1.Reorder(1, { 2, 2 });
		try
		{
			ctx.TransactionApply(b1.Build(1, 2));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidTree);
		}
	}

	PRUnitTest(TransactionApplyEnforcesMaxControlsResourceLimit, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto config = MakeConfig(/*max_controls*/ 2);
		auto ctx = UiContext(runtime, &config, &device);

		// Root + 2 panels = 3 controls, exceeding a max_controls of 2 configured at context
		// creation (section 9.2: every bounded resource is sized from Config, never unbounded).
		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::ResourceLimit);
		}

		// The rejected transaction must leave the tree empty (nothing was ever accepted).
		PR_EXPECT(ctx.DiagnosticsGet().control_count == 0);
	}

	PRUnitTest(TransactionApplyEnforcesMaxOperationsPerTransaction, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto config = MakeConfig(/*max_controls*/ 4096, /*max_roots*/ 64, /*max_tree_depth*/ 64, /*max_operations_per_transaction*/ 1);
		auto ctx = UiContext(runtime, &config, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(10.0f, 10.0f)));
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

	PRUnitTest(TransactionApplyAcceptsResourceStyleAndTemplate, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.AddResource(MakeResource(10, EResourceKind::Colour, Colour{ 1, 0, 0, 1 }));
		b0.AddStyle(MakeStyle(20));

		auto tmpl = MakeTemplate(30, EControlType::Button);
		b0.AddTemplatePart(tmpl, "PART_ContentPresenter", EVisualPrimitive::ContentPresenter, true);
		b0.AddTemplatePart(tmpl, "PART_FocusOutline", EVisualPrimitive::Border, true);
		b0.AddTemplate(tmpl);

		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f));
		b0.Upsert(root);
		auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80.0f, 24.0f));
		button.style_id = 20;
		button.template_id = 30;
		b0.Upsert(button);

		ctx.TransactionApply(b0.Build(0, 1));
		PR_EXPECT(ctx.DiagnosticsGet().control_count == 2);
	}

	PRUnitTest(TransactionApplyRejectsTemplateMissingRequiredPart, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		// Button requires PART_ContentPresenter and PART_FocusOutline; this template supplies
		// only the former, so it must be rejected before any control referencing it is applied.
		auto b0 = TxnBuilder{};
		auto tmpl = MakeTemplate(30, EControlType::Button);
		b0.AddTemplatePart(tmpl, "PART_ContentPresenter", EVisualPrimitive::ContentPresenter, true);
		b0.AddTemplate(tmpl);

		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f));
		b0.Upsert(root);
		auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80.0f, 24.0f));
		button.template_id = 30;
		b0.Upsert(button);

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

	PRUnitTest(TransactionApplyDefaultTemplatesSatisfyRequiredPartsForButtonAndTextBox, Quick)
	{
		// template_id == 0 selects the built-in default template, which must already satisfy its
		// own control type's required parts without the application authoring anything.
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		b0.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80.0f, 24.0f)));
		b0.Upsert(MakeControl(3, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(120.0f, 24.0f)));
		ctx.TransactionApply(b0.Build(0, 1));

		PR_EXPECT(ctx.DiagnosticsGet().control_count == 3);
	}

	PRUnitTest(TransactionApplyRejectsResourceBlobRangeOutOfBounds, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto resource = MakeResource(10, EResourceKind::Colour, Colour{ 1, 1, 1, 1 });
		resource.name_offset = 1000; // no such range exists in the (empty) blob
		resource.name_length = 4;
		b0.AddResource(resource);
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));

		try
		{
			ctx.TransactionApply(b0.Build(0, 1));
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::InvalidStruct);
		}
	}

	PRUnitTest(TransactionApplyRejectsMalformedStructHeader, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		auto txn = BuildRootOnly(b0, 1);
		auto bad_header = txn.header;
		bad_header.version = VIEW3D_UI_STRUCT_VERSION + 1;
		txn.header = bad_header;
		try
		{
			ctx.TransactionApply(txn);
			PR_EXPECT(false);
		}
		catch (Exception const& ex)
		{
			PR_EXPECT(ex.Status() == EStatus::SchemaMismatch);
		}
	}
}
