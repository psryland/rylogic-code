//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M2 tests (implementation-plan.md section 5.5): deterministic semantic snapshot with
// role/name/description/value/state/actions/focus/bounds, pre-order sequence numbering shared
// across roots, repeat-call byte-identity, offscreen detection, and TextBox live-pending-text
// value reporting.
#include "pr/common/unittests.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Locate one control's semantic record; throws (a plain std::runtime_error, not a
		// View3DUI Exception) if 'id' is absent, which is always a test-authoring bug.
		SemanticNode const& NodeOf(std::vector<SemanticNode> const& nodes, ControlId id)
		{
			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			if (it == nodes.end())
				throw std::runtime_error("test helper: semantic node not found for the requested control id");

			return *it;
		}

		// The text a semantic record's (offset,length) span denotes, materialised out of the
		// drained text blob for direct string comparison.
		std::string TextOf(std::vector<char> const& blob, std::uint32_t offset, std::uint32_t length)
		{
			return std::string(blob.data() + offset, length);
		}

		// One Root (id 1, 300x200) containing a named/described Button (id 2), a named TextBox
		// with accepted text "Alice" (id 3), and a Panel (id 4) pushed far outside the root's own
		// bounds via a large positive left margin (well past the 300-DIP-wide root) so it is
		// guaranteed to be flagged Offscreen without relying on a disallowed negative margin.
		void BuildTree(UiContext& ctx)
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

			auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
			button.layout.margin_left = 10.0f;
			button.layout.margin_top = 10.0f;
			std::tie(button.name_offset, button.name_length) = b0.AddText("OK button");
			std::tie(button.desc_offset, button.desc_length) = b0.AddText("Confirms the pending action");
			b0.Upsert(button);

			auto text_box = MakeControl(3, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 40.0f;
			std::tie(text_box.name_offset, text_box.name_length) = b0.AddText("Name field");
			std::tie(text_box.text_offset, text_box.text_length) = b0.AddText("Alice");
			b0.Upsert(text_box);

			auto offscreen_panel = MakeControl(4, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
			offscreen_panel.layout.margin_left = 1000.0f;
			std::tie(offscreen_panel.name_offset, offscreen_panel.name_length) = b0.AddText("Off-screen panel");
			b0.Upsert(offscreen_panel);

			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(300, 200));
		}

		// Drains the current semantic snapshot as (nodes, text_blob), sized exactly from
		// SemanticsPendingSizes as every real host must.
		std::pair<std::vector<SemanticNode>, std::vector<char>> DrainSemantics(UiContext& ctx)
		{
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			return { std::move(nodes), std::move(text_blob) };
		}
	}

	PRUnitTest(SemanticsReportsRoleNameDescriptionValueBoundsAndRevision, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildTree(ctx);

		auto [nodes, blob] = DrainSemantics(ctx);

		auto const& button = NodeOf(nodes, 2);
		PR_EXPECT(button.parent_id == 1);
		PR_EXPECT(button.role == EControlType::Button);
		PR_EXPECT(TextOf(blob, button.name_offset, button.name_length) == "OK button");
		PR_EXPECT(TextOf(blob, button.desc_offset, button.desc_length) == "Confirms the pending action");
		PR_EXPECT(button.value_length == 0); // Button reports no value text (only TextBox/Text do)
		PR_EXPECT(button.bounds.x == 10.0f && button.bounds.y == 10.0f && button.bounds.w == 50.0f && button.bounds.h == 20.0f);
		PR_EXPECT(button.accepted_revision == 1);
		PR_EXPECT((button.state_flags & static_cast<std::uint32_t>(ESemanticState::Enabled)) != 0);
		PR_EXPECT((button.state_flags & static_cast<std::uint32_t>(ESemanticState::Visible)) != 0);
		PR_EXPECT((button.state_flags & static_cast<std::uint32_t>(ESemanticState::Focusable)) != 0);
		PR_EXPECT((button.supported_actions & static_cast<std::uint32_t>(ESemanticAction::Invoke)) != 0);
		PR_EXPECT((button.supported_actions & static_cast<std::uint32_t>(ESemanticAction::Focus)) != 0);

		auto const& text_box = NodeOf(nodes, 3);
		PR_EXPECT(text_box.role == EControlType::TextBox);
		PR_EXPECT(TextOf(blob, text_box.value_offset, text_box.value_length) == "Alice");
		PR_EXPECT((text_box.supported_actions & static_cast<std::uint32_t>(ESemanticAction::SetValue)) != 0);
	}

	PRUnitTest(SemanticsSequenceIsPreOrderAndSharedAcrossRoots, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildTree(ctx);

		auto [nodes, blob] = DrainSemantics(ctx);
		(void)blob;

		// Pre-order over a single root: Root(1) first, then its three children in upsert/child
		// order (Button=2, TextBox=3, Panel=4); the sequence counter starts at 1 and is strictly
		// increasing with no gaps (section 5.5).
		auto const& root = NodeOf(nodes, 1);
		auto const& button = NodeOf(nodes, 2);
		auto const& text_box = NodeOf(nodes, 3);
		auto const& panel = NodeOf(nodes, 4);
		PR_EXPECT(root.semantic_sequence == 1);
		PR_EXPECT(button.semantic_sequence == 2);
		PR_EXPECT(text_box.semantic_sequence == 3);
		PR_EXPECT(panel.semantic_sequence == 4);
	}

	PRUnitTest(SemanticsRepeatedCopyWithoutMutationIsByteIdentical, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildTree(ctx);

		auto [nodes_a, blob_a] = DrainSemantics(ctx);
		auto [nodes_b, blob_b] = DrainSemantics(ctx);

		// Update() caches one snapshot; repeated SemanticsPendingSizes/SemanticsCopy calls without
		// an intervening Update() must read that same cached snapshot byte-for-byte (section 5.5).
		PR_EXPECT(nodes_a.size() == nodes_b.size());
		PR_EXPECT(blob_a.size() == blob_b.size());
		PR_EXPECT(nodes_a.size() != 0 && std::memcmp(nodes_a.data(), nodes_b.data(), nodes_a.size() * sizeof(SemanticNode)) == 0);
		PR_EXPECT(blob_a.size() != 0 && std::memcmp(blob_a.data(), blob_b.data(), blob_a.size()) == 0);
	}

	PRUnitTest(SemanticsFlagsOffscreenControlOutsideItsOwnRootBounds, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildTree(ctx);

		auto [nodes, blob] = DrainSemantics(ctx);
		(void)blob;

		auto const& button = NodeOf(nodes, 2);
		auto const& offscreen_panel = NodeOf(nodes, 4);
		PR_EXPECT((button.state_flags & static_cast<std::uint32_t>(ESemanticState::Offscreen)) == 0);
		PR_EXPECT((offscreen_panel.state_flags & static_cast<std::uint32_t>(ESemanticState::Offscreen)) != 0);
	}

	PRUnitTest(SemanticsFocusedFlagTracksCurrentFocusTarget, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildTree(ctx);

		{
			auto [nodes, blob] = DrainSemantics(ctx);
			(void)blob;
			PR_EXPECT((NodeOf(nodes, 2).state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) == 0);
		}

		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.Update(Viewport(300, 200));

		{
			auto [nodes, blob] = DrainSemantics(ctx);
			(void)blob;
			PR_EXPECT((NodeOf(nodes, 2).state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) != 0);
			PR_EXPECT((NodeOf(nodes, 3).state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) == 0);
		}
	}

	PRUnitTest(SemanticsTextBoxValueReflectsLivePendingEditNotAcceptedText, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildTree(ctx);

		// Before any edit, the TextBox reports its accepted text ("Alice").
		{
			auto [nodes, blob] = DrainSemantics(ctx);
			PR_EXPECT(TextOf(blob, NodeOf(nodes, 3).value_offset, NodeOf(nodes, 3).value_length) == "Alice");
		}

		// Focus the text box (Tab, Tab) and type one more character without committing; the
		// engine lazily seeds the pending edit buffer from the accepted text on first keystroke,
		// so the caret starts at the end and the appended char yields "AliceX" (section 7.4). The
		// underlying accepted ControlNode text is untouched until a host applies a follow-up
		// Upsert, so this is the only observable difference between "live" and "accepted" value.
		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(CharInputRecord('X'));
		ctx.Update(Viewport(300, 200));

		{
			auto [nodes, blob] = DrainSemantics(ctx);
			auto const& text_box = NodeOf(nodes, 3);
			PR_EXPECT((text_box.state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) != 0);
			PR_EXPECT(TextOf(blob, text_box.value_offset, text_box.value_length) == "AliceX");
		}
	}

	PRUnitTest(SemanticsDisabledControlOmitsEnabledFlagAndActions, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b0 = TxnBuilder{};
		b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(50.0f, 20.0f));
		button.enabled = 0;
		b0.Upsert(button);
		ctx.TransactionApply(b0.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		auto [nodes, blob] = DrainSemantics(ctx);
		(void)blob;

		// A disabled control still reports its structural facts (role/bounds/focusable) but no
		// Enabled state flag and no actions at all, even ones (like Focus) it would otherwise
		// support purely from being focusable (section 5.5).
		auto const& button_node = NodeOf(nodes, 2);
		PR_EXPECT((button_node.state_flags & static_cast<std::uint32_t>(ESemanticState::Enabled)) == 0);
		PR_EXPECT((button_node.state_flags & static_cast<std::uint32_t>(ESemanticState::Focusable)) != 0);
		PR_EXPECT(button_node.supported_actions == static_cast<std::uint32_t>(ESemanticAction::None));
	}
}
