//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Focused ComboBox coverage for ABI item encoding, validation, transient popup input, proposals,
// popup drawing, and semantics.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "input.h"
#include "semantics.h"
#include "uia_bridge.h"
#include "uia_provider.h"
#include "uia_snapshot.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Append items to the transaction and attach the resulting span to 'combo'.
		void SetItems(TxnBuilder& transaction, ControlDesc& combo, std::initializer_list<std::string_view> items)
		{
			// The item array is transaction-level and the descriptor stores only a span into it.
			auto const [offset, count] = transaction.AddComboBoxItems(items);
			combo.combo_item_offset = offset;
			combo.combo_item_count = count;
		}

		// Build one root with a ComboBox at [10, 10, 130, 30].
		ControlDesc BuildCombo(UiEngine& engine, std::int32_t selected_index = 1)
		{
			// Keep the item list longer than the visible row count so scrolling/highlight tests have range.
			auto transaction = TxnBuilder{};
			transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(220.0f, 160.0f)));
			auto combo = MakeControl(2, 1, EControlType::ComboBox, ELayoutMode::Overlay, Lp(120.0f, 24.0f));
			combo.layout.margin_left = 10.0f;
			combo.layout.margin_top = 10.0f;
			combo.selected_index = selected_index;
			combo.max_visible_items = 3;
			SetItems(transaction, combo, { "Alpha", "Beta", "Gamma", "Delta", "Epsilon" });
			transaction.Upsert(combo);
			engine.TransactionApply(transaction.Build(0, 1));
			engine.Update(Viewport(220, 160));
			return combo;
		}

		// Drain every event with its fixed ABI record intact.
		std::vector<Event> DrainEvents(UiEngine& engine)
		{
			// Size both buffers from the engine's pending counts.
			auto events = std::vector<Event>(engine.EventCount());
			auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
			engine.EventsCopy(events, payload);
			return events;
		}

		// Return the coalesced ComboBox selection proposal.
		Event const& ComboProposal(std::vector<Event> const& events)
		{
			// Match the shared ValueChangeProposed event kind used by Slider.
			auto found = std::find_if(events.begin(), events.end(), [](Event const& event)
			{
				return event.kind == EEventKind::ValueChangeProposed;
			});
			if (found == events.end())
				throw std::runtime_error("ComboBox proposal event was not found");

			return *found;
		}

		// True when the event queue contains a ComboBox selection proposal.
		bool HasComboProposal(UiEngine& engine)
		{
			// Focus changes can share the queue with the interaction under test, so filter by kind.
			auto const events = DrainEvents(engine);
			return std::any_of(events.begin(), events.end(), [](Event const& event)
			{
				return event.kind == EEventKind::ValueChangeProposed;
			});
		}
	}

	// Invalid item spans, selected indices and visible-row counts reject the transaction atomically.
	PRUnitTest(ComboBoxRejectsInvalidDescriptorAndItemRanges, Quick)
	{
		// Establish a valid baseline before trying rejected updates.
		auto engine = UiEngine(MakeConfig());
		auto combo = BuildCombo(engine);
		auto reject = [&](ControlDesc invalid, std::uint64_t revision)
		{
			// A rejected descriptor must leave the previous accepted revision in place.
			auto change = TxnBuilder{};
			change.Upsert(invalid);
			PR_THROWS(engine.TransactionApply(change.Build(1, revision)), EngineException);
			PR_EXPECT(engine.DiagnosticsGet().accepted_revision == 1);
		};

		auto invalid = combo;
		invalid.max_visible_items = 0;
		reject(invalid, 2);

		invalid = combo;
		invalid.selected_index = 99;
		reject(invalid, 2);

		invalid = combo;
		invalid.combo_item_offset = 100;
		reject(invalid, 2);

		auto bad_text = std::string{ static_cast<char>(0xFF) };
		auto invalid_utf8 = combo;
		auto change = TxnBuilder{};
		SetItems(change, invalid_utf8, { bad_text });
		change.Upsert(invalid_utf8);
		PR_THROWS(engine.TransactionApply(change.Build(1, 2)), EngineException);
		PR_EXPECT(engine.DiagnosticsGet().accepted_revision == 1);
	}

	// Closed keyboard navigation proposes accepted indices without mutating the descriptor.
	PRUnitTest(ComboBoxClosedKeysProposeSelection, Quick)
	{
		// Focus through Tab so the control receives keyboard input.
		auto engine = UiEngine(MakeConfig());
		BuildCombo(engine, 1);
		engine.InputInject(KeyDownInput(VK_TAB));
		DrainEvents(engine);

		engine.InputInject(KeyDownInput(VK_DOWN));
		auto proposal = ComboProposal(DrainEvents(engine));
		PR_EXPECT(proposal.control_id == 2);
		PR_EXPECT(proposal.has_numeric_value != 0);
		PR_EXPECT(proposal.numeric_value == 2.0);

		engine.InputInject(KeyDownInput(VK_HOME));
		PR_EXPECT(ComboProposal(DrainEvents(engine)).numeric_value == 0.0);
		engine.InputInject(KeyDownInput(VK_END));
		PR_EXPECT(ComboProposal(DrainEvents(engine)).numeric_value == 4.0);
	}

	// Open popup keyboard navigation moves a transient highlight and Enter proposes it.
	PRUnitTest(ComboBoxOpenKeysHighlightScrollAndCommit, Quick)
	{
		// PageDown from item 1 moves the highlight while keeping the accepted value unchanged until Enter.
		auto engine = UiEngine(MakeConfig());
		BuildCombo(engine, 1);
		engine.InputInject(KeyDownInput(VK_TAB));
		DrainEvents(engine);
		engine.InputInject(KeyDownInput(VK_SPACE));
		engine.InputInject(KeyDownInput(VK_NEXT));
		engine.Update(Viewport(220, 160));
		PR_EXPECT(engine.DrawPackets().items.size() > 8);

		engine.InputInject(KeyDownInput(VK_RETURN));
		auto proposal = ComboProposal(DrainEvents(engine));
		PR_EXPECT(proposal.numeric_value == 4.0);
	}

	// Every closed-state opening key opens the popup before Enter commits the transient highlight.
	PRUnitTest(ComboBoxOpeningKeysAllOpenPopup, Quick)
	{
		// Exercise the Windows ComboBox opening vocabulary one key at a time.
		auto const alt = static_cast<std::uint32_t>(EInputModifier::Alt);
		for (auto const& open_key : { KeyDownInput(VK_SPACE), KeyDownInput(VK_RETURN), KeyDownInput(VK_F4), KeyDownInput(VK_DOWN, alt) })
		{
			auto engine = UiEngine(MakeConfig());
			BuildCombo(engine, 1);
			engine.InputInject(KeyDownInput(VK_TAB));
			DrainEvents(engine);
			engine.InputInject(open_key);
			engine.InputInject(KeyDownInput(VK_DOWN));
			engine.InputInject(KeyDownInput(VK_RETURN));
			auto proposal = ComboProposal(DrainEvents(engine));
			PR_EXPECT(proposal.numeric_value == 2.0);
		}
	}

	// Escape and light dismiss close without producing a selection proposal.
	PRUnitTest(ComboBoxEscapeAndLightDismissCloseWithoutProposal, Quick)
	{
		// Escape closes the popup, and an outside click is consumed for light dismiss.
		auto engine = UiEngine(MakeConfig());
		BuildCombo(engine, 1);
		engine.InputInject(KeyDownInput(VK_TAB));
		DrainEvents(engine);
		engine.InputInject(KeyDownInput(VK_SPACE));
		engine.InputInject(KeyDownInput(VK_ESCAPE));
		PR_EXPECT(engine.EventCount() == 0);

		engine.InputInject(KeyDownInput(VK_SPACE));
		engine.InputInject(KeyDownInput(VK_F4));
		PR_EXPECT(!HasComboProposal(engine));

		engine.InputInject(PointerDownInput(20.0f, 20.0f));
		engine.InputInject(PointerDownInput(20.0f, 20.0f));
		PR_EXPECT(!HasComboProposal(engine));

		engine.InputInject(KeyDownInput(VK_SPACE));
		engine.InputInject(PointerDownInput(200.0f, 150.0f));
		PR_EXPECT(engine.EventCount() == 0);
	}


	// Focus loss and accepted-tree invalidation close the popup without proposing a selection.
	PRUnitTest(ComboBoxFocusLossAndTreeChangeCloseWithoutProposal, Quick)
	{
		// Losing HWND focus closes transient UI state before later Enter keys can commit the old highlight.
		auto engine = UiEngine(MakeConfig());
		auto combo = BuildCombo(engine, 1);
		engine.InputInject(KeyDownInput(VK_TAB));
		DrainEvents(engine);
		engine.InputInject(KeyDownInput(VK_SPACE));
		engine.InputInject(TextInputRecord(EInputKind::FocusLost));
		engine.InputInject(KeyDownInput(VK_RETURN));
		PR_EXPECT(!HasComboProposal(engine));

		// Disabling the accepted control closes the popup during transaction reconciliation.
		engine.InputInject(TextInputRecord(EInputKind::FocusGained));
		engine.InputInject(KeyDownInput(VK_TAB));
		engine.InputInject(KeyDownInput(VK_SPACE));
		combo.enabled = 0;
		auto change = TxnBuilder{};
		SetItems(change, combo, { "Alpha", "Beta", "Gamma", "Delta", "Epsilon" });
		change.Upsert(combo);
		engine.TransactionApply(change.Build(1, 2));
		engine.InputInject(KeyDownInput(VK_RETURN));
		PR_EXPECT(!HasComboProposal(engine));
	}

	// Popup mouse clicks have priority above controls beneath the popup and propose the clicked item.
	PRUnitTest(ComboBoxPopupHitTestHasPriorityAndMouseCommitProposes, Quick)
	{
		// Add an overlapping button beneath the popup; the popup item wins the hit test.
		auto engine = UiEngine(MakeConfig());
		auto transaction = TxnBuilder{};
		transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(220.0f, 160.0f)));
		auto button = MakeControl(3, 1, EControlType::Button, ELayoutMode::Overlay, Lp(120.0f, 80.0f));
		button.layout.margin_left = 10.0f;
		button.layout.margin_top = 34.0f;
		transaction.Upsert(button, "Under", "Under");
		auto combo = MakeControl(2, 1, EControlType::ComboBox, ELayoutMode::Overlay, Lp(120.0f, 24.0f));
		combo.layout.margin_left = 10.0f;
		combo.layout.margin_top = 10.0f;
		combo.max_visible_items = 3;
		combo.selected_index = 0;
		SetItems(transaction, combo, { "Alpha", "Beta", "Gamma", "Delta" });
		transaction.Upsert(combo);
		engine.TransactionApply(transaction.Build(0, 1));
		engine.Update(Viewport(220, 160));
		engine.InputInject(PointerDownInput(20.0f, 20.0f));
		engine.InputInject(PointerDownInput(20.0f, 58.0f));
		auto proposal = ComboProposal(DrainEvents(engine));
		PR_EXPECT(proposal.control_id == 2);
		PR_EXPECT(proposal.numeric_value == 1.0);
	}

	// A press that commits a popup item owns the pointer until release, even though the popup has
	// closed, so a drag or release over the scene behind it is not reported as unconsumed scene input.
	PRUnitTest(ComboBoxPopupCommitPressOwnsPointerUntilRelease, Quick)
	{
		// Open the popup with a full click, then commit an item with a press.
		auto engine = UiEngine(MakeConfig());
		BuildCombo(engine);
		LRESULT result = 0;
		std::int32_t invalidate = 0;
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONDOWN, MK_LBUTTON, MAKELPARAM(20, 20), result, invalidate) != 0);
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONUP, 0, MAKELPARAM(20, 20), result, invalidate) != 0);
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONDOWN, MK_LBUTTON, MAKELPARAM(20, 58), result, invalidate) != 0);
		PR_EXPECT(ComboProposal(DrainEvents(engine)).control_id == 2);

		// The held drag and release now lie outside every control but still belong to the UI.
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_MOUSEMOVE, MK_LBUTTON, MAKELPARAM(200, 150), result, invalidate) != 0);
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONUP, 0, MAKELPARAM(200, 150), result, invalidate) != 0);

		// After the release, ordinary scene input outside the UI is unconsumed again.
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_MOUSEMOVE, 0, MAKELPARAM(201, 150), result, invalidate) == 0);
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONDOWN, MK_LBUTTON, MAKELPARAM(200, 150), result, invalidate) == 0);
		PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONUP, 0, MAKELPARAM(200, 150), result, invalidate) == 0);
	}

	// Popup placement flips above when there is no room below and clamps inside the viewport.
	PRUnitTest(ComboBoxPopupPlacementFlipsAndClamps, Quick)
	{
		// A low control must place its popup above rather than extending past the viewport bottom.
		auto engine = UiEngine(MakeConfig());
		auto transaction = TxnBuilder{};
		transaction.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(160.0f, 90.0f)));
		auto combo = MakeControl(2, 1, EControlType::ComboBox, ELayoutMode::Overlay, Lp(140.0f, 24.0f));
		combo.layout.margin_left = 30.0f;
		combo.layout.margin_top = 62.0f;
		combo.max_visible_items = 2;
		combo.selected_index = 0;
		SetItems(transaction, combo, { "Alpha", "Beta", "Gamma" });
		transaction.Upsert(combo);
		engine.TransactionApply(transaction.Build(0, 1));
		engine.Update(Viewport(160, 90));
		engine.InputInject(PointerDownInput(40.0f, 70.0f));
		engine.Update(Viewport(160, 90));
		auto const& items = engine.DrawPackets().items;
		auto found_above = std::any_of(items.begin(), items.end(), [](DrawItem const& item)
		{
			return item.control_id == 2 && item.bounds.y < 62.0f && item.bounds.h > 20.0f;
		});
		PR_EXPECT(found_above);
	}

	// World-anchored roots support ComboBox popup input in projected viewport DIP space.
	PRUnitTest(ComboBoxWorksInWorldAnchoredRoots, Quick)
	{
		// Project a world root in front of the camera and select through its transient popup.
		auto engine = UiEngine(MakeConfig());
		auto transaction = TxnBuilder{};
		transaction.Upsert(MakeWorldRoot(1, ERootPolicy::Overlay, 120.0f, 24.0f, WorldParams()));
		auto combo = MakeControl(2, 1, EControlType::ComboBox, ELayoutMode::Overlay, Lp(120.0f, 24.0f));
		combo.max_visible_items = 3;
		combo.selected_index = 0;
		SetItems(transaction, combo, { "Alpha", "Beta", "Gamma" });
		transaction.Upsert(combo);
		engine.TransactionApply(transaction.Build(0, 1));
		engine.Update(Viewport(220, 160, 96.0f, 0.0, PerspectiveCamera()));
		engine.InputInject(PointerDownInput(110.0f, 80.0f));
		engine.InputInject(PointerDownInput(110.0f, 120.0f));
		auto proposal = ComboProposal(DrainEvents(engine));
		PR_EXPECT(proposal.control_id == 2);
		PR_EXPECT(proposal.numeric_value == 1.0);
	}

	// Semantics expose ComboBox role, selected value and expand/collapse action.
	PRUnitTest(ComboBoxSemanticsExposeRoleValueAndExpandAction, Quick)
	{
		// Read the semantic value from the packed text blob.
		auto engine = UiEngine(MakeConfig());
		BuildCombo(engine, 1);
		engine.Update(Viewport(220, 160));
		auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
		auto text = std::vector<char>(engine.SemanticTextBytesPending());
		engine.SemanticsCopy(nodes, text);
		auto const node = *std::find_if(nodes.begin(), nodes.end(), [](SemanticNode const& n) { return n.id == 2; });
		auto value = std::string(text.data() + node.value_offset, node.value_length);
		PR_EXPECT(node.role == EControlType::ComboBox);
		PR_EXPECT(value == "Beta");
		PR_EXPECT((node.supported_actions & static_cast<std::uint32_t>(ESemanticAction::ExpandCollapse)) != 0);
	}

	// UI Automation exposes read-only ValuePattern and ExpandCollapsePattern for ComboBox.
	PRUnitTest(ComboBoxUiaValueAndExpandCollapsePatterns, Quick)
	{
		// Publish the semantic snapshot through the UIA projection and exercise the provider directly.
		auto engine = UiEngine(MakeConfig());
		BuildCombo(engine, 1);
		auto const viewport = Viewport(220, 160);
		engine.Update(viewport);
		auto semantic_nodes = std::vector<SemanticNode>(engine.SemanticCount());
		auto semantic_text = std::vector<char>(engine.SemanticTextBytesPending());
		engine.SemanticsCopy(semantic_nodes, semantic_text);
		auto semantics = SemanticSnapshot{};
		semantics.m_nodes = std::move(semantic_nodes);
		semantics.m_text_blob.assign(semantic_text.begin(), semantic_text.end());
		auto snapshot = BuildUiaSnapshot(semantics, viewport, 1, 1);
		auto shared = std::make_shared<UiaSharedState>();
		shared->Bind(GetDesktopWindow());
		shared->Publish(snapshot);
		auto* provider = CreateUiaElementProvider(shared, 2);
		PR_EXPECT(provider != nullptr);

		auto property = VARIANT{};
		PR_EXPECT(provider->GetPropertyValue(UIA_ControlTypePropertyId, &property) == S_OK);
		PR_EXPECT(property.vt == VT_I4 && property.lVal == UIA_ComboBoxControlTypeId);
		VariantClear(&property);
		PR_EXPECT(provider->GetPropertyValue(UIA_ValueValuePropertyId, &property) == S_OK);
		PR_EXPECT(property.vt == VT_BSTR && std::wstring(property.bstrVal) == L"Beta");
		VariantClear(&property);
		PR_EXPECT(provider->GetPropertyValue(UIA_ValueIsReadOnlyPropertyId, &property) == S_OK);
		PR_EXPECT(property.vt == VT_BOOL && property.boolVal != VARIANT_FALSE);
		VariantClear(&property);

		auto* value_unknown = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(provider->GetPatternProvider(UIA_ValuePatternId, &value_unknown) == S_OK && value_unknown != nullptr);
		auto* value = static_cast<IValueProvider*>(nullptr);
		PR_EXPECT(value_unknown->QueryInterface(IID_PPV_ARGS(&value)) == S_OK);
		value_unknown->Release();
		auto readonly = BOOL{};
		PR_EXPECT(value->get_IsReadOnly(&readonly) == S_OK && readonly != FALSE);
		PR_EXPECT(value->SetValue(L"Gamma") == UIA_E_INVALIDOPERATION);
		value->Release();

		auto* expand_unknown = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(provider->GetPatternProvider(UIA_ExpandCollapsePatternId, &expand_unknown) == S_OK && expand_unknown != nullptr);
		auto* expand = static_cast<IExpandCollapseProvider*>(nullptr);
		PR_EXPECT(expand_unknown->QueryInterface(IID_PPV_ARGS(&expand)) == S_OK);
		expand_unknown->Release();
		auto state = ExpandCollapseState{};
		PR_EXPECT(expand->get_ExpandCollapseState(&state) == S_OK && state == ExpandCollapseState_Collapsed);
		expand->Release();
		provider->Release();
	}

}
