//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Hover tooltips: the rest delay, anchoring, dismissal, control-versus-application priority, ancestor inheritance, viewport
// placement, root style and font validation, and the final screen draw group that carries the tooltip.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "test_support.h"
#include <algorithm>

namespace pr::view3d::ui::tests
{
	namespace
	{
		constexpr ControlId kRoot = 1;
		constexpr ControlId kPanel = 2;
		constexpr ControlId kButton = 3;
		constexpr StyleId kTipStyle = 20;

		// A full-viewport root holding a panel with one button at its top-left corner. The tooltips are optional so each test
		// chooses where the text lives.
		void Scene(UiEngine& engine, std::string_view button_tip, std::string_view panel_tip = {}, std::uint64_t revision = 1)
		{
			// Give the root a bordered tooltip style so the box size includes the border.
			auto style = MakeStyle(kTipStyle);
			for (auto& visual : style.visuals)
				visual.border_thickness = 1.0f;

			auto root = MakeControl(kRoot, 0, EControlType::Root, ELayoutMode::Overlay, Lp(800, 600));
			root.tooltip_style_id = kTipStyle;
			auto panel = MakeControl(kPanel, kRoot, EControlType::Panel, ELayoutMode::Overlay, Lp(200, 100));
			auto button = MakeControl(kButton, kPanel, EControlType::Button, ELayoutMode::Overlay, Lp(100, 40));

			auto b = TxnBuilder{};
			b.AddStyle(style);
			b.Upsert(root);
			b.Upsert(panel, {}, {}, {}, panel_tip);
			b.Upsert(button, "OK", {}, {}, button_tip);
			engine.TransactionApply(b.Build(revision - 1, revision));
		}

		// The tooltip's draw group, or null when no tooltip is drawn. The tooltip is the only group beyond the single root.
		DrawGroup const* TooltipGroup(DrawPacket const& packet)
		{
			// The root always draws one group, so a second group can only be the tooltip.
			return packet.groups.size() == 2 ? &packet.groups.back() : nullptr;
		}

		// The text drawn by the tooltip group, or empty when no tooltip is drawn.
		std::string TooltipText(DrawPacket const& packet)
		{
			// The box comes first, then the text item.
			auto const* group = TooltipGroup(packet);
			return group != nullptr ? packet.items[group->first_item + 1].text : std::string{};
		}

		// The tooltip box bounds, or an empty rectangle when no tooltip is drawn.
		Rect TooltipBounds(DrawPacket const& packet)
		{
			// The box item spans the whole tooltip.
			auto const* group = TooltipGroup(packet);
			return group != nullptr ? packet.items[group->first_item].bounds : Rect{};
		}

		// Move the pointer and run one update, so the hovered target is resolved and its rest delay starts at 'time_ms'.
		void HoverAt(UiEngine& engine, float x, float y, double time_ms)
		{
			// Hover resolution happens on input, but the tooltip target only changes during Update.
			engine.InputInject(PointerMoveInput(x, y));
			engine.Update(Viewport(800, 600, 96.0f, time_ms));
		}

		// Expect 'action' to throw an EngineException with 'status'.
		template <typename Action>
		void ExpectStatus(EStatus status, Action action)
		{
			// Report a missing throw as a failure.
			try
			{
				action();
				PR_EXPECT(false);
			}
			catch (EngineException const& ex)
			{
				PR_EXPECT(ex.Status() == status);
			}
		}
	}

	PRUnitTest(TooltipAppearsOnlyAfterThePointerRestsForTheDelay, Quick)
	{
		// Hover the button and advance time to just before and just after the delay.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "Confirm");
		engine.Update(Viewport(800, 600, 96.0f, 0.0));
		engine.InputInject(PointerMoveInput(20.0f, 15.0f));

		engine.Update(Viewport(800, 600, 96.0f, 100.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);
		engine.Update(Viewport(800, 600, 96.0f, 599.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);
		engine.Update(Viewport(800, 600, 96.0f, 650.0));

		// The tooltip is drawn last, in screen space, with the button's text.
		auto const& packet = engine.DrawPackets();
		PR_EXPECT(TooltipGroup(packet) != nullptr);
		PR_EXPECT(packet.groups.back().policy == ERootPolicy::Screen);
		PR_EXPECT(packet.groups.back().root_id == kRoot);
		PR_EXPECT(packet.groups.back().first_item + packet.groups.back().item_count == packet.items.size());
		PR_EXPECT(TooltipText(packet) == "Confirm");
	}

	PRUnitTest(MovementRestartsTheDelayButAShownTooltipStaysAnchored, Quick)
	{
		// Movement beyond the rest tolerance before the tooltip shows restarts the delay.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "Confirm");
		engine.Update(Viewport(800, 600, 96.0f, 0.0));
		engine.InputInject(PointerMoveInput(20.0f, 15.0f));
		engine.Update(Viewport(800, 600, 96.0f, 100.0));
		engine.InputInject(PointerMoveInput(40.0f, 15.0f));
		engine.Update(Viewport(800, 600, 96.0f, 400.0));
		engine.Update(Viewport(800, 600, 96.0f, 650.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);
		engine.Update(Viewport(800, 600, 96.0f, 950.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) != nullptr);
		auto const shown = TooltipBounds(engine.DrawPackets());

		// Once shown, moving within the same target keeps the tooltip where it appeared.
		engine.InputInject(PointerMoveInput(80.0f, 30.0f));
		engine.Update(Viewport(800, 600, 96.0f, 1000.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) != nullptr);
		auto const moved = TooltipBounds(engine.DrawPackets());
		PR_EXPECT(moved.x == shown.x && moved.y == shown.y);

		// The box sits below and to the right of the rest point, sized for the text, padding and border.
		PR_EXPECT(shown.x == 40.0f + 12.0f);
		PR_EXPECT(shown.y == 15.0f + 20.0f);
		PR_EXPECT(shown.w > 2.0f * (6.0f + 1.0f));
		PR_EXPECT(shown.h > 2.0f * (3.0f + 1.0f));
	}

	PRUnitTest(PressingDismissesTheTooltipUntilTheTargetChanges, Quick)
	{
		// Show the tooltip, then press the button.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, "Confirm");
		engine.Update(Viewport(800, 600, 96.0f, 0.0));
		HoverAt(engine, 20.0f, 15.0f, 10.0);
		engine.Update(Viewport(800, 600, 96.0f, 600.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) != nullptr);
		engine.InputInject(PointerDownInput(20.0f, 15.0f));
		engine.InputInject(PointerUpInput(20.0f, 15.0f));
		engine.Update(Viewport(800, 600, 96.0f, 700.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);

		// Resting on the same target does not bring it back.
		engine.Update(Viewport(800, 600, 96.0f, 2000.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);

		// Leaving and returning is a new target, so the tooltip shows again after the delay.
		engine.InputInject(PointerMoveInput(400.0f, 300.0f));
		engine.Update(Viewport(800, 600, 96.0f, 2100.0));
		engine.InputInject(PointerMoveInput(20.0f, 15.0f));
		engine.Update(Viewport(800, 600, 96.0f, 2200.0));
		engine.Update(Viewport(800, 600, 96.0f, 2750.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) != nullptr);
	}

	PRUnitTest(ControlsInheritAncestorTooltipsAndHideTheApplicationTooltip, Quick)
	{
		// The button has no text of its own, so the panel's tooltip applies.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, {}, "Panel help");
		engine.TooltipSet(kRoot, 7, "Oak tree");
		engine.Update(Viewport(800, 600, 96.0f, 0.0));
		HoverAt(engine, 20.0f, 15.0f, 10.0);
		engine.Update(Viewport(800, 600, 96.0f, 600.0));
		PR_EXPECT(TooltipText(engine.DrawPackets()) == "Panel help");

		// Away from every control, the application tooltip applies after its own delay.
		engine.InputInject(PointerMoveInput(400.0f, 300.0f));
		engine.Update(Viewport(800, 600, 96.0f, 700.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);
		engine.Update(Viewport(800, 600, 96.0f, 1250.0));
		PR_EXPECT(TooltipText(engine.DrawPackets()) == "Oak tree");

		// The same key updates the text in place without restarting the delay.
		engine.TooltipSet(kRoot, 7, "Oak tree\nHealth 41/64");
		engine.Update(Viewport(800, 600, 96.0f, 1300.0));
		PR_EXPECT(TooltipText(engine.DrawPackets()) == "Oak tree\nHealth 41/64");

		// A new key is a new target and restarts the delay. Key 0 clears it.
		engine.TooltipSet(kRoot, 8, "Granite");
		engine.Update(Viewport(800, 600, 96.0f, 1350.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);
		engine.Update(Viewport(800, 600, 96.0f, 1900.0));
		PR_EXPECT(TooltipText(engine.DrawPackets()) == "Granite");
		engine.TooltipSet(0, 0, {});
		engine.Update(Viewport(800, 600, 96.0f, 2000.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);

		// Over a control without any tooltip in its ancestry, the application tooltip is hidden.
		auto engine2 = UiEngine(MakeConfig());
		Scene(engine2, {});
		engine2.TooltipSet(kRoot, 7, "Oak tree");
		engine2.Update(Viewport(800, 600, 96.0f, 0.0));
		HoverAt(engine2, 20.0f, 15.0f, 10.0);
		engine2.Update(Viewport(800, 600, 96.0f, 600.0));
		PR_EXPECT(TooltipGroup(engine2.DrawPackets()) == nullptr);
	}

	PRUnitTest(TooltipsFlipAndClampToStayInsideTheViewport, Quick)
	{
		// Rest near the bottom-right corner, where the default placement would overflow both edges.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, {});
		engine.TooltipSet(kRoot, 1, "A tooltip near the corner");
		engine.Update(Viewport(800, 600, 96.0f, 0.0));
		HoverAt(engine, 790.0f, 590.0f, 10.0);
		engine.Update(Viewport(800, 600, 96.0f, 600.0));

		// The box flips above and to the left of the pointer, staying fully inside the viewport.
		auto const bounds = TooltipBounds(engine.DrawPackets());
		PR_EXPECT(bounds.w > 0.0f);
		PR_EXPECT(bounds.x + bounds.w == 790.0f);
		PR_EXPECT(bounds.y + bounds.h == 590.0f);
		PR_EXPECT(bounds.x >= 0.0f && bounds.y >= 0.0f);
	}

	PRUnitTest(TooltipSetValidatesTheRootAndClearsWhenTheRootIsRemoved, Quick)
	{
		// Only an accepted Root can style an application tooltip, and the text must be valid UTF-8.
		auto engine = UiEngine(MakeConfig());
		Scene(engine, {});
		ExpectStatus(EStatus::InvalidArgument, [&] { engine.TooltipSet(99, 1, "x"); });
		ExpectStatus(EStatus::InvalidArgument, [&] { engine.TooltipSet(kButton, 1, "x"); });
		ExpectStatus(EStatus::InvalidArgument, [&] { engine.TooltipSet(kRoot, 1, "\xFF"); });

		// Clearing needs no root.
		engine.TooltipSet(99, 0, {});

		// Removing the root drops its application tooltip, so a recreated root starts without one.
		engine.TooltipSet(kRoot, 1, "Oak tree");
		auto remove = TxnBuilder{};
		remove.Remove(kRoot);
		engine.TransactionApply(remove.Build(1, 2));
		Scene(engine, {}, {}, 3);
		engine.Update(Viewport(800, 600, 96.0f, 0.0));
		HoverAt(engine, 400.0f, 300.0f, 10.0);
		engine.Update(Viewport(800, 600, 96.0f, 600.0));
		PR_EXPECT(TooltipGroup(engine.DrawPackets()) == nullptr);
	}

	PRUnitTest(RootTooltipStylesAndFontsAreValidatedAndHeldInUse, Quick)
	{
		// Unknown tooltip styles and fonts, and non-font resources, are rejected.
		auto engine = UiEngine(MakeConfig());
		auto root = MakeControl(kRoot, 0, EControlType::Root, ELayoutMode::Overlay, Lp(800, 600));
		root.tooltip_style_id = 77;
		auto bad_style = TxnBuilder{};
		bad_style.Upsert(root);
		ExpectStatus(EStatus::UnknownResource, [&] { engine.TransactionApply(bad_style.Build(0, 1)); });

		root.tooltip_style_id = 0;
		root.tooltip_font_resource_id = 50;
		auto bad_font = TxnBuilder{};
		bad_font.Upsert(root);
		ExpectStatus(EStatus::UnknownResource, [&] { engine.TransactionApply(bad_font.Build(0, 1)); });

		auto not_font = TxnBuilder{};
		not_font.AddResource(MakeResource(50, EResourceKind::Colour, Colour{ 1, 1, 1, 1 }));
		not_font.Upsert(root);
		ExpectStatus(EStatus::UnknownResource, [&] { engine.TransactionApply(not_font.Build(0, 1)); });

		// Invalid UTF-8 tooltip text is rejected.
		auto bad_text = TxnBuilder{};
		root.tooltip_font_resource_id = 0;
		bad_text.Upsert(root, {}, {}, {}, "\xC3");
		ExpectStatus(EStatus::InvalidArgument, [&] { engine.TransactionApply(bad_text.Build(0, 1)); });

		// A referenced tooltip style or font cannot be removed while the root uses it.
		Scene(engine, {});
		auto font = TxnBuilder{};
		font.AddResource(MakeResource(60, EResourceKind::Font, Colour{ 1, 1, 1, 1 }, 12.0f));
		root.tooltip_style_id = kTipStyle;
		root.tooltip_font_resource_id = 60;
		font.Upsert(root);
		engine.TransactionApply(font.Build(1, 2));

		auto remove_style = TxnBuilder{};
		remove_style.RemoveStyle(kTipStyle);
		ExpectStatus(EStatus::ResourceInUse, [&] { engine.TransactionApply(remove_style.Build(2, 3)); });
		auto remove_font = TxnBuilder{};
		remove_font.RemoveResource(60);
		ExpectStatus(EStatus::ResourceInUse, [&] { engine.TransactionApply(remove_font.Build(2, 3)); });
	}
}
