//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M9 black-box tests for grapheme-cluster editing, selection, max-length enforcement and the IME
// composition lifecycle, driven entirely through the public UiContext facade (deterministic
// NormalizedInput plus InputTextPayload). No live IME, no window and no real device are required.
#include "pr/common/unittests.h"
#include "test_support.h"
#include <algorithm>
#include <optional>
#include <string>

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Root (id 1, 300x200) containing one focusable TextBox (id 2) at [10,40)x[210,60) seeded
		// with 'initial_text'. 'max_text_length' of 0 means unbounded.
		void M9Scene(UiContext& ctx, std::string_view initial_text, std::uint32_t max_text_length = 0)
		{
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));

			auto text_box = MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(200.0f, 20.0f));
			text_box.layout.margin_left = 10.0f;
			text_box.layout.margin_top = 40.0f;
			text_box.max_text_length = max_text_length;
			std::tie(text_box.text_offset, text_box.text_length) = b.AddText(initial_text);
			b.Upsert(text_box);

			ctx.TransactionApply(b.Build(0, 1));
			ctx.Update(Viewport(300, 200));
			ctx.InputInject(KeyDownInput(VK_TAB));
		}

		// The semantic node reported for 'id' in a freshly refreshed snapshot, together with the
		// text blob it indexes into.
		struct SemanticView
		{
			SemanticNode node;
			std::string blob;

			std::string Value() const
			{
				return blob.substr(node.value_offset, node.value_length);
			}
		};
		SemanticView SemanticOf(UiContext& ctx, ControlId id)
		{
			ctx.Update(Viewport(300, 200));
			auto sizes = ctx.SemanticsPendingSizes();
			auto nodes = std::vector<SemanticNode>(sizes.m_count);
			auto text_blob = std::vector<char>(sizes.m_payload_bytes);
			ctx.SemanticsCopy(nodes, text_blob);

			auto it = std::find_if(nodes.begin(), nodes.end(), [id](SemanticNode const& n) { return n.id == id; });
			if (it == nodes.end())
				throw std::runtime_error("test helper: semantic node not found for the requested control id");

			return SemanticView{ .node = *it, .blob = std::string(text_blob.data(), text_blob.size()) };
		}

		// The live pending/display text of 'id' as the semantic snapshot reports it.
		std::string ValueOf(UiContext& ctx, ControlId id)
		{
			return SemanticOf(ctx, id).Value();
		}

		// Drains the queue and returns the text of the last TextChangeProposed for 'id', if any.
		std::optional<std::string> LatestProposal(UiContext& ctx, ControlId id)
		{
			auto sizes = ctx.EventsPendingSizes();
			auto events = std::vector<Event>(sizes.m_count);
			auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
			ctx.EventsCopy(events, payload);

			auto it = std::find_if(events.rbegin(), events.rend(), [id](Event const& e) { return e.kind == EEventKind::TextChangeProposed && e.control_id == id; });
			if (it == events.rend())
				return std::nullopt;

			return std::string(reinterpret_cast<char const*>(payload.data()) + it->payload_offset, it->payload_length);
		}

		// True if any TextChangeProposed for 'id' is currently queued; drains the queue either way.
		bool AnyProposal(UiContext& ctx, ControlId id)
		{
			return LatestProposal(ctx, id).has_value();
		}

		// Types 'text' one Char record per code point. Only used for BMP scalars, which is what
		// WM_CHAR-style Char records carry.
		void TypeAscii(UiContext& ctx, std::string_view text)
		{
			for (auto ch : text)
				ctx.InputInject(CharInputRecord(static_cast<std::uint32_t>(static_cast<unsigned char>(ch))));
		}

		auto const kShift = static_cast<std::uint32_t>(EInputModifier::Shift);
		auto const kCtrl = static_cast<std::uint32_t>(EInputModifier::Ctrl);
	}

	PRUnitTest(BackspaceRemovesAWholeGraphemeClusterNotOneByteOrScalar, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		// Base + combining acute, an astral emoji, a ZWJ family, a flag pair and a CRLF: one
		// Backspace must remove exactly one of these each time.
		M9Scene(ctx, "x\u0301\U0001F600\U0001F468\u200D\U0001F469\U0001F1F3\U0001F1FFa\r\n");
		ctx.InputInject(KeyDownInput(VK_END));

		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "x\u0301\U0001F600\U0001F468\u200D\U0001F469\U0001F1F3\U0001F1FFa");

		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "x\u0301\U0001F600\U0001F468\u200D\U0001F469\U0001F1F3\U0001F1FF");

		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "x\u0301\U0001F600\U0001F468\u200D\U0001F469");

		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "x\u0301\U0001F600");

		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "x\u0301");

		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "");

		// Backspace at the start of an empty buffer is a no-op, not an underflow.
		ctx.InputInject(KeyDownInput(VK_BACK));
		PR_EXPECT(ValueOf(ctx, 2) == "");
	}

	PRUnitTest(DeleteRemovesTheFollowingGraphemeClusterWhole, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "\U0001F1F3\U0001F1FF\U0001F600z");
		ctx.InputInject(KeyDownInput(VK_HOME));

		ctx.InputInject(KeyDownInput(VK_DELETE));
		PR_EXPECT(ValueOf(ctx, 2) == "\U0001F600z");

		ctx.InputInject(KeyDownInput(VK_DELETE));
		PR_EXPECT(ValueOf(ctx, 2) == "z");

		ctx.InputInject(KeyDownInput(VK_DELETE));
		PR_EXPECT(ValueOf(ctx, 2) == "");

		// Delete at the end is a no-op.
		ctx.InputInject(KeyDownInput(VK_DELETE));
		PR_EXPECT(ValueOf(ctx, 2) == "");
	}

	PRUnitTest(ArrowMovementStepsOverWholeClustersInBothDirections, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "a\U0001F600b");
		ctx.InputInject(KeyDownInput(VK_HOME));

		// One Right lands past the emoji; typing there proves the caret is on a boundary.
		ctx.InputInject(KeyDownInput(VK_RIGHT));
		ctx.InputInject(KeyDownInput(VK_RIGHT));
		TypeAscii(ctx, "|");
		PR_EXPECT(ValueOf(ctx, 2) == "a\U0001F600|b");

		// Two Lefts step back over the '|' and the whole emoji.
		ctx.InputInject(KeyDownInput(VK_LEFT));
		ctx.InputInject(KeyDownInput(VK_LEFT));
		TypeAscii(ctx, "#");
		PR_EXPECT(ValueOf(ctx, 2) == "a#\U0001F600|b");
	}

	PRUnitTest(ControlArrowMovesByWholeWords, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "alpha beta gamma");
		ctx.InputInject(KeyDownInput(VK_HOME));

		ctx.InputInject(KeyDownInput(VK_RIGHT, kCtrl));
		TypeAscii(ctx, "-");
		PR_EXPECT(ValueOf(ctx, 2) == "alpha -beta gamma");

		ctx.InputInject(KeyDownInput(VK_END));
		ctx.InputInject(KeyDownInput(VK_LEFT, kCtrl));
		TypeAscii(ctx, "+");
		PR_EXPECT(ValueOf(ctx, 2) == "alpha -beta +gamma");
	}

	PRUnitTest(HomeAndEndMoveTheCaretToTheTextExtremes, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "mid");

		ctx.InputInject(KeyDownInput(VK_HOME));
		TypeAscii(ctx, "<");
		ctx.InputInject(KeyDownInput(VK_END));
		TypeAscii(ctx, ">");
		PR_EXPECT(ValueOf(ctx, 2) == "<mid>");
	}

	PRUnitTest(ShiftArrowExtendsASelectionThatTypingThenReplaces, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "abcdef");
		ctx.InputInject(KeyDownInput(VK_HOME));

		// Select "abc" then replace it in one keystroke.
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));

		auto const selected = SemanticOf(ctx, 2).node;
		PR_EXPECT((selected.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::HasSelection)) != 0);
		PR_EXPECT(selected.selection_start == 0);
		PR_EXPECT(selected.selection_end == 3);

		TypeAscii(ctx, "Z");
		PR_EXPECT(ValueOf(ctx, 2) == "Zdef");
	}

	PRUnitTest(ShiftControlArrowExtendsASelectionByWholeWords, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "one two three");
		ctx.InputInject(KeyDownInput(VK_HOME));

		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift | kCtrl));
		PR_EXPECT(SemanticOf(ctx, 2).node.selection_end == 4);

		ctx.InputInject(KeyDownInput(VK_DELETE));
		PR_EXPECT(ValueOf(ctx, 2) == "two three");
	}

	PRUnitTest(SelectAllCutAndPasteRoundTripThroughTheClipboard, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "clip\U0001F600");

		// Ctrl+A then Ctrl+X empties the control and puts the text on the clipboard.
		ctx.InputInject(KeyDownInput('A', kCtrl));
		PR_EXPECT(SemanticOf(ctx, 2).node.selection_end != 0);
		ctx.InputInject(KeyDownInput('X', kCtrl));
		PR_EXPECT(ValueOf(ctx, 2) == "");

		// Ctrl+V restores it byte for byte, so the astral character survived the round trip.
		ctx.InputInject(KeyDownInput('V', kCtrl));
		PR_EXPECT(ValueOf(ctx, 2) == "clip\U0001F600");

		// Ctrl+C leaves the text alone and Ctrl+V then doubles it.
		ctx.InputInject(KeyDownInput('A', kCtrl));
		ctx.InputInject(KeyDownInput('C', kCtrl));
		PR_EXPECT(ValueOf(ctx, 2) == "clip\U0001F600");
		ctx.InputInject(KeyDownInput(VK_END));
		ctx.InputInject(KeyDownInput('V', kCtrl));
		PR_EXPECT(ValueOf(ctx, 2) == "clip\U0001F600clip\U0001F600");
	}

	PRUnitTest(MaxTextLengthIsCountedInGraphemeClustersNotBytes, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		// Three clusters allowed. Two flag pairs would be 16 bytes but only 2 clusters, so a
		// byte-based limit would wrongly reject them.
		M9Scene(ctx, "", 3);
		auto flags = std::string("\U0001F1F3\U0001F1FF\U0001F1E6\U0001F1FA");
		ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), TextPayload(flags));
		PR_EXPECT(ValueOf(ctx, 2) == flags);

		// The third cluster fits; the fourth is truncated away rather than failing the call.
		TypeAscii(ctx, "ab");
		PR_EXPECT(ValueOf(ctx, 2) == flags + "a");
	}

	PRUnitTest(TextInputInsertsAtTheCaretAndReplacesTheSelection, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "abcd");
		ctx.InputInject(KeyDownInput(VK_HOME));
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));

		auto const replacement = std::string("\u4e2d\u6587");
		ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), TextPayload(replacement));
		PR_EXPECT(ValueOf(ctx, 2) == replacement + "cd");
		PR_EXPECT(LatestProposal(ctx, 2) == std::optional<std::string>(replacement + "cd"));
	}

	PRUnitTest(TextInjectionRejectsMalformedUtf8AndOversizeLengthsExplicitly, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "keep");

		// A truncated multi-byte sequence must fail loudly, never be stored or substituted.
		auto const bad = std::string("ok\xE4\xB8");
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), TextPayload(bad)), Exception);
		PR_EXPECT(ValueOf(ctx, 2) == "keep");

		// A null pointer with a nonzero length is a caller bug, not an empty insertion.
		auto payload = TextPayload("");
		payload.text_utf8 = nullptr;
		payload.text_length = 4;
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), payload), Exception);
		PR_EXPECT(ValueOf(ctx, 2) == "keep");

		// A length far beyond any bounded edit buffer is rejected up front.
		auto const source = std::string("abc");
		auto oversize = TextPayload(source);
		oversize.text_length = 0xFFFFFFFFU;
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), oversize), Exception);
		PR_EXPECT(ValueOf(ctx, 2) == "keep");
	}

	PRUnitTest(CompositionRendersLiveButProposesNoCommittedTextUntilItCommits, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "");
		PR_EXPECT(!AnyProposal(ctx, 2)); // drain the focus-time queue

		// CompositionStart carries no text of its own; it only marks the insertion point.
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		PR_EXPECT(ValueOf(ctx, 2) == "");
		PR_EXPECT(!AnyProposal(ctx, 2));

		auto const update = std::string("\u306B\u307B");
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(update, static_cast<std::uint32_t>(update.size())));
		PR_EXPECT(ValueOf(ctx, 2) == update);
		PR_EXPECT(!AnyProposal(ctx, 2));

		// The snapshot marks the composing span so a renderer/AT can present it distinctly.
		auto const composing = SemanticOf(ctx, 2).node;
		PR_EXPECT((composing.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::Composing)) != 0);
		PR_EXPECT(composing.composition_start == 0);
		PR_EXPECT(composing.composition_length == update.size());

		// Only the commit produces a durable proposal, and it carries the result string.
		auto const result = std::string("\u65E5\u672C");
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionCommit), TextPayload(result));
		PR_EXPECT(ValueOf(ctx, 2) == result);
		PR_EXPECT(LatestProposal(ctx, 2) == std::optional<std::string>(result));

		auto const settled = SemanticOf(ctx, 2).node;
		PR_EXPECT((settled.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::Composing)) == 0);
		PR_EXPECT(settled.composition_length == 0);
	}

	PRUnitTest(CompositionCancellationRestoresThePriorPendingEditExactly, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "base");
		ctx.InputInject(KeyDownInput(VK_HOME));
		ctx.InputInject(KeyDownInput(VK_RIGHT));
		TypeAscii(ctx, "-");
		PR_EXPECT(ValueOf(ctx, 2) == "b-ase");
		PR_EXPECT(AnyProposal(ctx, 2));

		auto const draft = std::string("\u304B\u306A");
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(draft));
		PR_EXPECT(ValueOf(ctx, 2) == "b-" + draft + "ase");

		// Cancel restores the exact pre-composition pending text and emits no proposal.
		ctx.InputInject(TextInputRecord(EInputKind::CompositionCancel));
		PR_EXPECT(ValueOf(ctx, 2) == "b-ase");
		PR_EXPECT(!AnyProposal(ctx, 2));

		// The caret is restored too, so the next keystroke lands where it did before.
		TypeAscii(ctx, "+");
		PR_EXPECT(ValueOf(ctx, 2) == "b-+ase");
	}

	PRUnitTest(CompositionReplacesTheSelectionWithoutProposingUntilCommit, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "abcd");
		ctx.InputInject(KeyDownInput('A', kCtrl));
		PR_EXPECT(!AnyProposal(ctx, 2));

		// Starting a composition consumes the selection visually but proposes nothing.
		auto const draft = std::string("\u3042");
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload(draft));
		PR_EXPECT(ValueOf(ctx, 2) == draft);
		PR_EXPECT(!AnyProposal(ctx, 2));

		// Cancelling after the selection was consumed still restores the original text exactly.
		ctx.InputInject(TextInputRecord(EInputKind::CompositionCancel));
		PR_EXPECT(ValueOf(ctx, 2) == "abcd");
		PR_EXPECT(!AnyProposal(ctx, 2));
	}

	PRUnitTest(MalformedCompositionOrderingIsRejectedRatherThanSilentlyAccepted, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "seed");

		// Update and commit without a live composition have no start to attach to.
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("x")), Exception);
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionCommit), TextPayload("x")), Exception);
		PR_THROWS(ctx.InputInject(TextInputRecord(EInputKind::CompositionCancel)), Exception);
		PR_EXPECT(ValueOf(ctx, 2) == "seed");

		// A second start while one is already live is a protocol violation.
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		PR_THROWS(ctx.InputInject(TextInputRecord(EInputKind::CompositionStart)), Exception);

		// Plain TextInput while composing would silently interleave two authorities.
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), TextPayload("b")), Exception);

		// A text-carrying kind injected without its payload, and a payload-free kind injected with
		// one, are both caller mistakes rather than empty edits.
		PR_THROWS(ctx.InputInject(TextInputRecord(EInputKind::CompositionUpdate)), Exception);
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionStart), TextPayload("b")), Exception);

		// The composition is untouched by every rejection above and still cancels cleanly.
		ctx.InputInject(TextInputRecord(EInputKind::CompositionCancel));
		PR_EXPECT(ValueOf(ctx, 2) == "seed");
	}

	PRUnitTest(CompositionOffsetsBeyondTheCompositionTextAreRejected, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "");
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("ab"));

		// A caret or selection outside the composition string cannot be rendered coherently.
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("ab", 9)), Exception);
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("ab", 1, 0, 9)), Exception);
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("ab", 1, 2, 1)), Exception);

		// The still-live composition is unchanged and cancels cleanly.
		PR_EXPECT(ValueOf(ctx, 2) == "ab");
		ctx.InputInject(TextInputRecord(EInputKind::CompositionCancel));
		PR_EXPECT(ValueOf(ctx, 2) == "");
	}

	PRUnitTest(LosingFocusDuringACompositionCancelsItAndKeepsCommittedTextIntact, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "keep");
		PR_EXPECT(!AnyProposal(ctx, 2));

		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("\u3042"));
		PR_EXPECT(ValueOf(ctx, 2) == "keep\u3042");

		// Focus loss abandons the transient composition; the durable text is untouched and no
		// half-composed proposal escapes.
		ctx.InputInject(TextInputRecord(EInputKind::FocusLost));
		PR_EXPECT(ValueOf(ctx, 2) == "keep");
		PR_EXPECT(!AnyProposal(ctx, 2));

		// The composition is genuinely gone, so a stray update is now a protocol error.
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("x")), Exception);
	}

	PRUnitTest(CompositionOnAnUnfocusedOrNonEditableControlIsRejected, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80.0f, 20.0f));
		b.Upsert(button);
		ctx.TransactionApply(b.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		// Nothing is focused, and later a focused Button is still not an editable target. Text and
		// composition records are left unconsumed rather than throwing, exactly as an ordinary
		// Char record is, so the host can still route the input elsewhere.
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), TextPayload("a"));

		ctx.InputInject(KeyDownInput(VK_TAB));
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), TextPayload("a"));

		// None of that invented an edit buffer on a control that cannot have one.
		PR_EXPECT(SemanticOf(ctx, 2).node.value_length == 0);
		PR_EXPECT((SemanticOf(ctx, 2).node.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::Composing)) == 0);

		// Nothing started, so an update still has no composition to attach to.
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("a")), Exception);
	}

	PRUnitTest(CommittingAnEmptyResultAfterACompositionProposesNothingNew, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "text");
		PR_EXPECT(!AnyProposal(ctx, 2));

		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("\u3042"));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionCommit), TextPayload(""));

		// Committing nothing leaves the durable text unchanged, so no proposal is emitted.
		PR_EXPECT(ValueOf(ctx, 2) == "text");
		PR_EXPECT(!AnyProposal(ctx, 2));
	}

	PRUnitTest(SemanticSnapshotsCarryCaretSelectionAndGraphemeCountsForATextProvider, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "a\U0001F600b");
		ctx.InputInject(KeyDownInput(VK_HOME));

		auto at_start = SemanticOf(ctx, 2).node;
		PR_EXPECT((at_start.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::HasCaret)) != 0);
		PR_EXPECT(at_start.caret == 0);
		PR_EXPECT(at_start.value_grapheme_count == 3);
		PR_EXPECT((at_start.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::HasSelection)) == 0);
		PR_EXPECT(at_start.selection_start == at_start.selection_end);

		// The caret offset is a byte offset that always lands on a cluster boundary.
		ctx.InputInject(KeyDownInput(VK_RIGHT));
		ctx.InputInject(KeyDownInput(VK_RIGHT));
		auto past_emoji = SemanticOf(ctx, 2).node;
		PR_EXPECT(past_emoji.caret == 5);

		// SetSelection is advertised so a later UIA Text provider has a supported action to map.
		PR_EXPECT((past_emoji.supported_actions & static_cast<std::uint32_t>(ESemanticAction::SetSelection)) != 0);
	}

	PRUnitTest(RepeatedSnapshotsOfAnUnchangedEditStateAreByteIdentical, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "deterministic");
		ctx.InputInject(KeyDownInput(VK_HOME));
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));
		ctx.InputInject(KeyDownInput(VK_RIGHT, kShift));
		ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
		ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("\u3042", 3));

		auto const first = SemanticOf(ctx, 2);
		auto const second = SemanticOf(ctx, 2);
		PR_EXPECT(std::memcmp(&first.node, &second.node, sizeof(SemanticNode)) == 0);
		PR_EXPECT(first.blob == second.blob);
	}

	PRUnitTest(CaretGeometryIsReportedOnlyForTheFocusedEditableControl, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);
		M9Scene(ctx, "caret");

		// The focused TextBox reports a caret rect inside its own bounds, which is what an IMM
		// candidate window is positioned against.
		auto valid = std::int32_t{};
		auto caret = ctx.CaretGeometry(2, valid);
		PR_EXPECT(valid != 0);
		PR_EXPECT(caret.w > 0.0f && caret.h > 0.0f);
		PR_EXPECT(caret.x >= 10.0f && caret.x <= 210.0f);
		PR_EXPECT(caret.y >= 40.0f && caret.y + caret.h <= 60.0f + 1.0f);

		// The caret advances as text is added, so the candidate window follows the caret.
		auto const at_home_x = caret.x;
		ctx.InputInject(KeyDownInput(VK_END));
		caret = ctx.CaretGeometry(2, valid);
		PR_EXPECT(valid != 0);
		PR_EXPECT(caret.x >= at_home_x);

		// A non-editable or unknown control reports "no caret" rather than failing.
		caret = ctx.CaretGeometry(1, valid);
		PR_EXPECT(valid == 0);
	}

	PRUnitTest(TheSameEditSequenceProducesTheSameEventsEveryRun, Quick)
	{
		// Determinism across whole context lifetimes, not just repeated reads of one context.
		auto Run = [](std::vector<std::string>& out)
		{
			auto runtime = Runtime{};
			auto device = FakeDevice{};
			auto ctx = UiContext(runtime, &device);
			M9Scene(ctx, "abc");
			ctx.InputInject(KeyDownInput(VK_END));
			TypeAscii(ctx, "de");
			ctx.InputInject(KeyDownInput(VK_BACK));
			ctx.InputInject(TextInputRecord(EInputKind::CompositionStart));
			ctx.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("\u3042"));
			ctx.InputInjectText(TextInputRecord(EInputKind::CompositionCommit), TextPayload("\u65E5"));

			auto sizes = ctx.EventsPendingSizes();
			auto events = std::vector<Event>(sizes.m_count);
			auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
			ctx.EventsCopy(events, payload);
			for (auto const& e : events)
			{
				if (e.kind != EEventKind::TextChangeProposed)
					continue;

				out.push_back(std::string(reinterpret_cast<char const*>(payload.data()) + e.payload_offset, e.payload_length));
			}
		};

		auto first = std::vector<std::string>{};
		auto second = std::vector<std::string>{};
		Run(first);
		Run(second);
		PR_EXPECT(!first.empty());
		PR_EXPECT(first == second);
		PR_EXPECT(first.back() == "abcd\u65E5");
	}
}
