//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M9 tests for the public wire ABI the milestone added: the InputTextPayload record, the
// SemanticNode text-range block, the closed EInputKind extension, and the two new exports. M9 is
// an additive but layout-affecting change, so the version was bumped to 0x00030000 / struct
// version 3 and these tests pin both the new layout and the fact that every pre-M9 field kept its
// meaning.
#include "pr/common/unittests.h"
#include "test_support.h"
#include <type_traits>

namespace pr::view3d::ui::tests
{
	PRUnitTest(TheM9AbiBumpIsDeclaredConsistentlyAcrossHeaderAndDll, Quick)
	{
		// A managed or native caller compiled against the M8 headers must be rejected, not
		// silently handed the M9 layout, so the two version constants move together.
		PR_EXPECT(VIEW3D_UI_STRUCT_VERSION == 3U);
		PR_EXPECT(VIEW3D_UI_API_VERSION == 0x00030000U);
		PR_EXPECT(ApiVersion() == VIEW3D_UI_API_VERSION);

		// The header's compiled-in version and the DLL's reported version are the same value, so a
		// stale DLL beside a fresh header is caught at load rather than at first misuse.
		PR_EXPECT(Dll::Get().ApiVersion() == VIEW3D_UI_API_VERSION);
	}

	PRUnitTest(InputTextPayloadIsAFixedLayoutBorrowedRecord, Quick)
	{
		// The payload crosses the C ABI, so it must be standard layout, trivially copyable and
		// describable by StructSize like every other wire struct.
		static_assert(std::is_standard_layout_v<InputTextPayload>, "InputTextPayload crosses the C ABI");
		static_assert(std::is_trivially_copyable_v<InputTextPayload>, "InputTextPayload must be memcpy-able across the ABI");
		static_assert(offsetof(InputTextPayload, header) == 0, "Every wire struct starts with its StructHeader");

		// The text is borrowed, never owned: a pointer plus an explicit length, with no capacity
		// or ownership field, is what makes the caller's buffer safe to reuse after the call.
		static_assert(std::is_same_v<decltype(InputTextPayload::text_utf8), char const*>, "Payload text is a borrowed UTF-8 pointer");
		static_assert(std::is_same_v<decltype(InputTextPayload::text_length), std::uint32_t>, "Payload length is an explicit byte count");

		PR_EXPECT(StructSize(EStructId::InputTextPayload) == sizeof(InputTextPayload));

		// Pin the exact offsets: this record is written by managed and native callers alike, so a
		// silent repack would corrupt every injected string rather than fail to compile.
		static_assert(offsetof(InputTextPayload, text_utf8) == 8, "InputTextPayload layout changed");
		static_assert(offsetof(InputTextPayload, text_length) == 16, "InputTextPayload layout changed");
		static_assert(offsetof(InputTextPayload, caret) == 20, "InputTextPayload layout changed");
		static_assert(offsetof(InputTextPayload, selection_start) == 24, "InputTextPayload layout changed");
		static_assert(offsetof(InputTextPayload, selection_end) == 28, "InputTextPayload layout changed");
		static_assert(sizeof(InputTextPayload) == 32, "InputTextPayload layout changed");
	}

	PRUnitTest(TheInputKindEnumStaysClosedAndKeepsEveryPreM9Value, Quick)
	{
		// EInputKind is a closed enum crossing the ABI; the M9 kinds are appended so a pre-M9
		// caller's records still mean exactly what they meant before.
		PR_EXPECT(static_cast<int>(EInputKind::PointerMove) == 0);
		PR_EXPECT(static_cast<int>(EInputKind::PointerButtonDown) == 1);
		PR_EXPECT(static_cast<int>(EInputKind::PointerButtonUp) == 2);
		PR_EXPECT(static_cast<int>(EInputKind::PointerWheel) == 3);
		PR_EXPECT(static_cast<int>(EInputKind::KeyDown) == 4);
		PR_EXPECT(static_cast<int>(EInputKind::KeyUp) == 5);
		PR_EXPECT(static_cast<int>(EInputKind::Char) == 6);
		PR_EXPECT(static_cast<int>(EInputKind::FocusGained) == 7);
		PR_EXPECT(static_cast<int>(EInputKind::FocusLost) == 8);

		// The five M9 additions.
		PR_EXPECT(static_cast<int>(EInputKind::TextInput) == 9);
		PR_EXPECT(static_cast<int>(EInputKind::CompositionStart) == 10);
		PR_EXPECT(static_cast<int>(EInputKind::CompositionUpdate) == 11);
		PR_EXPECT(static_cast<int>(EInputKind::CompositionCommit) == 12);
		PR_EXPECT(static_cast<int>(EInputKind::CompositionCancel) == 13);
		PR_EXPECT(static_cast<int>(EInputKind::Count) == 14);
	}

	PRUnitTest(SemanticNodeCarriesTheM9TextRangeBlockAfterEveryPreM9Member, Quick)
	{
		static_assert(std::is_standard_layout_v<SemanticNode>, "SemanticNode crosses the C ABI");

		// The text block is appended, so a caller that only reads the pre-M9 prefix is unaffected
		// and a zero-initialised node still means "no caret, no selection, not composing".
		static_assert(offsetof(SemanticNode, caret) > offsetof(SemanticNode, value_length), "The M9 text block must follow every pre-M9 SemanticNode member");
		static_assert(offsetof(SemanticNode, text_flags) > offsetof(SemanticNode, caret), "text_flags belongs to the M9 text block");
		static_assert(offsetof(SemanticNode, value_grapheme_count) > offsetof(SemanticNode, caret), "value_grapheme_count belongs to the M9 text block");

		PR_EXPECT(StructSize(EStructId::SemanticNode) == sizeof(SemanticNode));
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticTextFlag::None) == 0U);
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticTextFlag::HasCaret) == 1U);
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticTextFlag::HasSelection) == 2U);
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticTextFlag::Composing) == 4U);

		// SetSelection is a new bit in the existing action mask, not a renumbering of the old ones.
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticAction::Invoke) == 1U);
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticAction::SetValue) == 2U);
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticAction::Focus) == 4U);
		PR_EXPECT(static_cast<std::uint32_t>(ESemanticAction::SetSelection) == 8U);
	}

	PRUnitTest(TheM9ExportsAreResolvableFromTheBuiltDll, Quick)
	{
		// Dll::Get() resolves every entry of VIEW3D_UI_API_TABLE at construction and throws if one
		// is missing, so reaching here at all proves both new exports exist. Assert the specific
		// pointers anyway so a future table edit that drops them fails here rather than at first
		// call by a managed caller.
		auto const& dll = Dll::Get();
		PR_EXPECT(dll.InputInjectText != nullptr);
		PR_EXPECT(dll.CaretGeometry != nullptr);

		// They are also present under their exported C names, which is what the managed P/Invoke
		// layer binds to.
		auto module = ::GetModuleHandleW(L"view3d-ui.dll");
		PR_EXPECT(module != nullptr);
		if (module != nullptr)
		{
			PR_EXPECT(::GetProcAddress(module, "View3DUI_InputInjectText") != nullptr);
			PR_EXPECT(::GetProcAddress(module, "View3DUI_CaretGeometry") != nullptr);
		}
	}

	PRUnitTest(TheM9ExportsValidateTheirArgumentsBeforeTouchingAnyState, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		auto text_box = MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Overlay, Lp(100.0f, 20.0f));
		std::tie(text_box.text_offset, text_box.text_length) = b.AddText("abi");
		b.Upsert(text_box);
		ctx.TransactionApply(b.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		// A stale struct version must be rejected on both the record and the payload, or a caller
		// built against the M8 layout would be reinterpreted as M9.
		auto stale_input = TextInputRecord(EInputKind::TextInput);
		stale_input.header.version = VIEW3D_UI_STRUCT_VERSION + 1;
		PR_THROWS(ctx.InputInjectText(stale_input, TextPayload("x")), Exception);

		auto stale_payload = TextPayload("x");
		stale_payload.header.version = VIEW3D_UI_STRUCT_VERSION + 1;
		PR_THROWS(ctx.InputInjectText(TextInputRecord(EInputKind::TextInput), stale_payload), Exception);

		// A kind that carries no text is not a valid InputInjectText record; that path is
		// InputInject's.
		PR_THROWS(ctx.InputInjectText(KeyDownInput(VK_TAB), TextPayload("x")), Exception);

		// Nothing above changed any state: the control still reports its committed text.
		auto sizes = ctx.SemanticsPendingSizes();
		auto nodes = std::vector<SemanticNode>(sizes.m_count);
		auto blob = std::vector<char>(sizes.m_payload_bytes);
		ctx.SemanticsCopy(nodes, blob);
		auto it = std::find_if(nodes.begin(), nodes.end(), [](SemanticNode const& n) { return n.id == 2; });
		PR_EXPECT(it != nodes.end());
		PR_EXPECT(std::string(blob.data() + it->value_offset, it->value_length) == "abi");
	}

	PRUnitTest(CaretGeometryRejectsNullOutputsAndUnknownControlsDistinctly, Quick)
	{
		auto runtime = Runtime{};
		auto device = FakeDevice{};
		auto ctx = UiContext(runtime, &device);

		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(300.0f, 200.0f)));
		ctx.TransactionApply(b.Build(0, 1));
		ctx.Update(Viewport(300, 200));

		// A null output is a caller bug and fails; an unknown control is an ordinary "no caret
		// here" answer and succeeds with valid == 0. Conflating the two would hide real bugs.
		auto caret = Rect{};
		auto valid = std::int32_t{};
		PR_EXPECT(Dll::Get().CaretGeometry(ctx.Handle(), 1, nullptr, &valid) != EStatus::Success);
		PR_EXPECT(Dll::Get().CaretGeometry(ctx.Handle(), 1, &caret, nullptr) != EStatus::Success);

		valid = 1;
		PR_EXPECT(Dll::Get().CaretGeometry(ctx.Handle(), 9999, &caret, &valid) == EStatus::Success);
		PR_EXPECT(valid == 0);
	}
}
