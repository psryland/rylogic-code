//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M3 black-box tests for TextEditState reconciliation after a transaction is accepted
// (implementation-plan.md section 7.4): a focused TextBox's locally-edited "pending text" is
// reconciled against the just-committed descriptor text on every TransactionApply, using
// last_accepted_text to distinguish three cases -
//  - the descriptor now matches the pending text exactly: the outstanding proposal is
//    acknowledged (case covered by TextEditProposalIsAcknowledgedByAnExactEcho).
//  - the descriptor differs from both the pending text and the previously tracked
//    last_accepted_text: the application changed the text unilaterally (managed
//    normalization/rejection), so the pending text/caret are replaced deterministically (case
//    covered by TextEditProposalIsNormalizedWhenATransactionCommitsUnexpectedText).
//  - the descriptor still equals last_accepted_text (this transaction did not touch this
//    control's text at all) while the pending text still differs: the outstanding proposal is
//    preserved untouched (case covered by
//    TextEditProposalSurvivesAnUnrelatedTransactionThatLeavesItsTextUnchanged, which also proves
//    that acknowledgement correctly advances last_accepted_text rather than leaving it stale).
// Exercised entirely through the public UiContext facade, exactly like every M0-M2 black-box test.
#include "pr/common/unittests.h"
#include "test_support.h"
#include <optional>

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Root (id 1, fixed 300x200) containing one TextBox (id 2, rect [10,40)x[110,60)),
		// focusable/enabled, seeded with 'initial_text' as its accepted text at revision 1.
		void BuildRootWithTextBox(UiContext& ctx, std::string_view initial_text)
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

			auto text_box = MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 40.0f;
			std::tie(text_box.text_offset, text_box.text_length) = b0.AddText(initial_text);
			b0.Upsert(text_box);

			ctx.TransactionApply(b0.Build(0, 1));
			ctx.Update(Viewport(300, 200));
		}

		// Re-Upsert TextBox(2) with 'new_text' as the sole change (create-or-update semantics
		// mean every other field must be resupplied identically), advancing the accepted revision
		// from 'base_revision' to 'base_revision + 1'.
		void CommitTextBoxText(UiContext& ctx, std::uint64_t base_revision, std::string_view new_text)
		{
			auto b = TxnBuilder{};
			auto text_box = MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 40.0f;
			std::tie(text_box.text_offset, text_box.text_length) = b.AddText(new_text);
			b.Upsert(text_box);

			ctx.TransactionApply(b.Build(base_revision, base_revision + 1));
			ctx.Update(Viewport(300, 200));
		}

		std::vector<Event> DrainEvents(UiContext& ctx, std::vector<std::byte>& payload_out)
		{
			auto sizes = ctx.EventsPendingSizes();
			auto events = std::vector<Event>(sizes.m_count);
			payload_out.resize(sizes.m_payload_bytes);
			ctx.EventsCopy(events, payload_out);
			return events;
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

		// The semantic 'value' text currently reported for 'id' (the TextBox's live pending text
		// per section 5.5), refreshed from a fresh Update() snapshot.
		std::string SemanticValueOf(UiContext& ctx, ControlId id)
		{
			ctx.Update(Viewport(300, 200));
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);
			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			if (it == nodes.end())
				throw std::runtime_error("test helper: semantic node not found for the requested control id");

			return std::string(text_blob.data() + it->value_offset, it->value_length);
		}

		// The most recently queued TextChangeProposed payload/edit_generation for 'id', or
		// std::nullopt if none is currently queued for it.
		struct TextProposal
		{
			std::string text;
			std::uint32_t edit_generation;
		};
		std::optional<TextProposal> LatestTextProposal(UiContext& ctx, ControlId id)
		{
			auto payload = std::vector<std::byte>{};
			auto events = DrainEvents(ctx, payload);
			auto it = std::find_if(events.begin(), events.end(), [id](Event const& e) { return e.kind == EEventKind::TextChangeProposed && e.control_id == id; });
			if (it == events.end())
				return std::nullopt;

			return TextProposal{ std::string(reinterpret_cast<char const*>(payload.data()) + it->payload_offset, it->payload_length), it->edit_generation };
		}
	}

	PRUnitTest(TextEditProposalIsAcknowledgedByAnExactEcho, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithTextBox(ctx, "seed");

		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 2));
		ctx.InputInject(CharInputRecord('a'));
		ctx.InputInject(CharInputRecord('b'));
		ctx.InputInject(CharInputRecord('c'));

		auto proposal = LatestTextProposal(ctx, 2);
		PR_EXPECT(proposal.has_value());
		PR_EXPECT(proposal->text == "seedabc");
		PR_EXPECT(proposal->edit_generation != 0); // an outstanding local proposal always carries a nonzero generation

		// The application commits exactly the proposed text: the proposal is acknowledged, so the
		// pending text is left exactly as it was (there is nothing to reconcile away).
		CommitTextBoxText(ctx, 1, "seedabc");
		PR_EXPECT(SemanticValueOf(ctx, 2) == "seedabc");
		PR_EXPECT(IsFocused(ctx, 2)); // acknowledging a proposal must not disturb focus

		// Editing must continue to work seamlessly after acknowledgement: the caret is still at the
		// end of "seedabc", so typing 'd' appends rather than replacing/corrupting anything, and the
		// new proposal still carries a nonzero generation.
		ctx.InputInject(CharInputRecord('d'));
		auto next_proposal = LatestTextProposal(ctx, 2);
		PR_EXPECT(next_proposal.has_value());
		PR_EXPECT(next_proposal->text == "seedabcd");
		PR_EXPECT(next_proposal->edit_generation != 0);
	}

	PRUnitTest(TextEditProposalIsNormalizedWhenATransactionCommitsUnexpectedText, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithTextBox(ctx, "seed");

		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 2));
		ctx.InputInject(CharInputRecord('a'));
		ctx.InputInject(CharInputRecord('b'));
		ctx.InputInject(CharInputRecord('c'));
		PR_EXPECT(LatestTextProposal(ctx, 2).has_value()); // drain and discard: only the reconciliation outcome matters below

		// The application commits text that is neither the outstanding proposal ("seedabc") nor
		// the last accepted text ("seed"): this is managed normalization/rejection, so the pending
		// text/caret are replaced deterministically with the descriptor's own text.
		CommitTextBoxText(ctx, 1, "ZZZ");
		PR_EXPECT(SemanticValueOf(ctx, 2) == "ZZZ");

		// The caret must have been placed deterministically at the end of the replacement text:
		// typing 'Q' appends to "ZZZ" rather than inserting/overwriting at some other position.
		ctx.InputInject(CharInputRecord('Q'));
		auto proposal = LatestTextProposal(ctx, 2);
		PR_EXPECT(proposal.has_value());
		PR_EXPECT(proposal->text == "ZZZQ");
		PR_EXPECT(proposal->edit_generation != 0);
	}

	PRUnitTest(TextEditProposalSurvivesAnUnrelatedTransactionThatLeavesItsTextUnchanged, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		BuildRootWithTextBox(ctx, "seed");

		ctx.InputInject(KeyDownInput(VK_TAB));
		PR_EXPECT(IsFocused(ctx, 2));
		ctx.InputInject(CharInputRecord('a'));
		ctx.InputInject(CharInputRecord('b'));
		ctx.InputInject(CharInputRecord('c'));
		PR_EXPECT(LatestTextProposal(ctx, 2).has_value());

		// Acknowledge "seedabc" first, so last_accepted_text advances away from the original
		// seed - this is what makes the later assertion a genuine test of last_accepted_text being
		// tracked correctly, rather than accidentally passing because it was never updated.
		CommitTextBoxText(ctx, 1, "seedabc");
		PR_EXPECT(SemanticValueOf(ctx, 2) == "seedabc");

		// A further local edit is now outstanding and has not been committed by anything yet.
		ctx.InputInject(CharInputRecord('X'));
		PR_EXPECT(SemanticValueOf(ctx, 2) == "seedabcX");

		// An unrelated transaction commits, but this control's own text is unchanged from
		// last_accepted_text ("seedabc"): the outstanding "seedabcX" proposal must be preserved
		// untouched rather than being clobbered back to "seedabc". If last_accepted_text had not
		// been advanced by the acknowledgement above, this commit would look like a normalizing
		// change away from the stale "seed" baseline and incorrectly overwrite the pending text.
		CommitTextBoxText(ctx, 2, "seedabc");
		PR_EXPECT(SemanticValueOf(ctx, 2) == "seedabcX");

		// Editing must still work normally on top of the preserved proposal.
		ctx.InputInject(CharInputRecord('Y'));
		auto proposal = LatestTextProposal(ctx, 2);
		PR_EXPECT(proposal.has_value());
		PR_EXPECT(proposal->text == "seedabcXY");
		PR_EXPECT(proposal->edit_generation != 0);
	}
}
