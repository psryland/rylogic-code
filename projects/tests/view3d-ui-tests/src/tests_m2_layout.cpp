//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M2 tests (implementation-plan.md section 6): deterministic retained layout for Overlay and
// horizontal/vertical Stack in DIPs, root auto-sizing to the viewport, and Reorder actually
// changing the resulting stacking order (not just being accepted).
#include "pr/common/unittests.h"
#include "test_support.h"
#include "pr/view3d-ui/engine.h"
#include "renderer.h"
#include "text_shaper.h"
#include "uia_snapshot.h"
#include "input.h"
#include "events.h"

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

		// Compare authored rectangles exactly where the layout arithmetic is integral.
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

		// Read the engine's current semantic rectangles without involving a renderer or window.
		std::vector<SemanticNode> SnapshotBounds(UiEngine& engine)
		{
			auto nodes = std::vector<SemanticNode>(engine.SemanticCount());
			auto text = std::vector<char>(engine.SemanticTextBytesPending());
			engine.SemanticsCopy(nodes, text);
			return nodes;
		}

		// Allow only rounding noise when checking content containment in viewport DIPs.
		bool Contains(Rect const& outer, Rect const& inner)
		{
			auto const tolerance = 0.001f;
			return inner.x >= outer.x - tolerance && inner.y >= outer.y - tolerance &&
				inner.x + inner.w <= outer.x + outer.w + tolerance && inner.y + inner.h <= outer.y + outer.h + tolerance;
		}
	}

	// Anchored root children and their actual text geometry must track resize/DPI without drift.
	PRUnitTest(LayoutNestedContentTracksViewportAndDpiWithoutDrift, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto shaper = TextShaper{};
		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(0.0f, 0.0f)));
		auto font = MakeResource(1, EResourceKind::Font, Colour{0, 0, 0, 1}, 16.0f);
		std::tie(font.name_offset, font.name_length) = b.AddText("Segoe UI");
		b.AddResource(font);

		// A padded six-line panel at the near edges reproduces multiline content, not just its box.
		auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::StackVertical, Lp(340.0f, 156.0f));
		panel.layout.margin_left = panel.layout.margin_top = 12.0f;
		panel.layout.padding_left = 12.0f;
		panel.layout.padding_top = 8.0f;
		b.Upsert(panel);
		auto text = MakeControl(3, 2, EControlType::Text, ELayoutMode::Overlay, Lp(316.0f, 140.0f), 1);
		std::tie(text.text_offset, text.text_length) = b.AddText("First line\nSecond line\nThird line\nFourth line\nFifth line\nLast line");
		b.Upsert(text);

		// A right/centre stack exercises nested padding, margins, labels, and centred button text.
		auto menu = MakeControl(4, 1, EControlType::Panel, ELayoutMode::StackVertical, Lp(300.0f, 440.0f, EHAlign::Right, EVAlign::Center, 8.0f));
		menu.layout.margin_right = 24.0f;
		menu.layout.padding_left = menu.layout.padding_right = 16.0f;
		menu.layout.padding_top = menu.layout.padding_bottom = 16.0f;
		b.Upsert(menu);
		auto title = MakeControl(5, 4, EControlType::Text, ELayoutMode::Overlay, Lp(268.0f, 60.0f), 1);
		std::tie(title.text_offset, title.text_length) = b.AddText("Choose\nan action");
		b.Upsert(title);
		auto buttons = MakeControl(6, 4, EControlType::Panel, ELayoutMode::StackHorizontal, Lp(268.0f, 56.0f, EHAlign::Left, EVAlign::Top, 8.0f));
		buttons.layout.padding_top = buttons.layout.padding_bottom = 4.0f;
		b.Upsert(buttons);
		for (auto id : { 7u, 8u })
		{
			auto button = MakeControl(id, 6, EControlType::Button, ELayoutMode::Overlay, Lp(120.0f, 40.0f, EHAlign::Left, EVAlign::Center), 1);
			button.layout.margin_left = 4.0f;
			std::tie(button.text_offset, button.text_length) = b.AddText("Action");
			b.Upsert(button);
		}
		auto bottom = MakeControl(9, 1, EControlType::Button, ELayoutMode::Overlay, Lp(100.0f, 40.0f, EHAlign::Center, EVAlign::Bottom), 1);
		bottom.layout.margin_bottom = 20.0f;
		std::tie(bottom.text_offset, bottom.text_length) = b.AddText("Continue");
		b.Upsert(bottom);
		engine.TransactionApply(b.Build(0, 1));

		// Change viewport, target/client ratios, and DPI independently, returning to the same DIP size.
		auto original = std::vector<SemanticNode>{};
		for (auto dpi : { 96.0f, 120.0f, 144.0f, 192.0f, 96.0f })
		{
			for (auto size : { Vec2{960.0f, 720.0f}, Vec2{1200.0f, 900.0f}, Vec2{800.0f, 600.0f}, Vec2{960.0f, 720.0f} })
			{
				auto viewport = Viewport(2048, 1600, dpi);
				viewport.client_width_px = 1280;
				viewport.client_height_px = 1000;
				viewport.viewport_x_px = 32.0f;
				viewport.viewport_y_px = 24.0f;
				viewport.viewport_width_px = size.x * dpi / 96.0f;
				viewport.viewport_height_px = size.y * dpi / 96.0f;
				viewport.target_width_px = static_cast<std::uint32_t>(viewport.viewport_width_px + 64.0f);
				viewport.target_height_px = static_cast<std::uint32_t>(viewport.viewport_height_px + 48.0f);
				engine.Update(viewport);
				auto const nodes = SnapshotBounds(engine);
				PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{0, 0, size.x, size.y}));
				PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{12, 12, 340, 156}));
				PR_EXPECT(RectEqual(BoundsOf(nodes, 3), Rect{24, 20, 316, 140}));
				PR_EXPECT(RectEqual(BoundsOf(nodes, 4), Rect{size.x - 324, (size.y - 440) * 0.5f, 300, 440}));
				PR_EXPECT(RectEqual(BoundsOf(nodes, 9), Rect{(size.x - 100) * 0.5f, size.y - 60, 100, 40}));
				for (auto const& node : nodes)
				{
					if (node.parent_id != 0)
						PR_EXPECT(Contains(BoundsOf(nodes, node.parent_id), node.bounds));
				}

				// Every drawn text line must fit its semantic box and its parent; both use the renderer's origins.
				for (auto const& item : engine.DrawPackets().items)
				{
					PR_EXPECT(RectEqual(item.bounds, BoundsOf(nodes, item.control_id)));
					switch (item.primitive)
					{
						case EVisualPrimitive::SolidBox:
						case EVisualPrimitive::RoundedBox: { continue; }
						case EVisualPrimitive::TextPresenter: { break; }
						default: { throw std::runtime_error("unexpected fixture draw primitive"); }
					}
					auto glyphs = std::vector<ShapedGlyph>{};
					auto const width = shaper.Shape(item.font_family, item.font_size, dpi / 96.0f, item.text, glyphs);
					auto const height = shaper.LayoutHeight(item.font_family, item.font_size, item.text);
					auto const x = Renderer::TextRunStartXDip(item, width);
					auto const y = TextOriginYDip(item.bounds.y, item.bounds.h, height);
					PR_EXPECT(Contains(item.bounds, Rect{x, y, width, height}));
					for (auto const& line : shaper.RangeRects(item.font_family, item.font_size, item.text, 0, static_cast<std::uint32_t>(item.text.size()), 32))
						PR_EXPECT(Contains(item.bounds, Rect{x + line.x, y + line.y, line.w, line.h}));
				}

				// A client-pixel click at a drawn button's centre must activate that same semantic control.
				auto const pixels = UiaClientPixelRect(viewport, BoundsOf(nodes, 7));
				auto const point = MAKELPARAM(static_cast<int>(pixels.x + pixels.w * 0.5f), static_cast<int>(pixels.y + pixels.h * 0.5f));
				LRESULT result = 0;
				std::int32_t invalidate = 0;
				PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONDOWN, MK_LBUTTON, point, result, invalidate) != 0);
				PR_EXPECT(engine.ProcessWindowMessage(nullptr, WM_LBUTTONUP, 0, point, result, invalidate) != 0);
				auto events = std::vector<Event>(engine.EventCount());
				auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
				engine.EventsCopy(events, payload);
				PR_EXPECT(std::any_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::CommandInvoked && e.control_id == 7; }));

				// Repeated return visits compare every child rectangle, not only the anchored panel.
				if (size.x == 960.0f)
				{
					if (original.empty())
						original = nodes;

					for (auto const& node : nodes)
						PR_EXPECT(RectEqual(node.bounds, BoundsOf(original, node.id)));
				}
			}
		}
	}

	// Every layout mode removes collapsed geometry, and stacks skip every collapsed sibling slot.
	PRUnitTest(LayoutCollapsedSubtreesRemoveExtentAndRestoreWithoutDrift, Quick)
	{
		for (auto mode : { ELayoutMode::Overlay, ELayoutMode::StackHorizontal, ELayoutMode::StackVertical, ELayoutMode::Canvas, ELayoutMode::Scroll })
		{
			for (auto mask = 0u; mask != 8u; ++mask)
			{
				auto engine = UiEngine(MakeConfig());
				auto b = TxnBuilder{};
				auto root = MakeControl(1, 0, EControlType::Root, mode, Lp(300, 200, EHAlign::Left, EVAlign::Top, 7));
				root.layout.padding_left = root.layout.padding_top = 5;
				b.Upsert(root);
				auto children = std::vector<ControlDesc>{};
				for (auto id = 2u; id != 5u; ++id)
				{
					auto child = MakeControl(id, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(40, 30));
					child.visibility = id == 3u ? EVisibility::Hidden : EVisibility::Visible;
					child.layout.margin_left = child.layout.margin_right = child.layout.margin_top = child.layout.margin_bottom = 2;
					child.layout.padding_left = child.layout.padding_top = 3;
					child.layout.canvas_x = static_cast<float>(id * 50);
					children.push_back(child);
					b.Upsert(child);
					b.Upsert(MakeControl(id + 10, id, EControlType::Button, ELayoutMode::Overlay, Lp(10, 10)));
				}
				engine.TransactionApply(b.Build(0, 1));
				engine.Update(Viewport(600, 400));
				auto const original = SnapshotBounds(engine);

				// Mix retained Hidden and Visible children with every collapse mask, including their margins and spacing.
				auto collapse = TxnBuilder{};
				for (auto i = 0u; i != children.size(); ++i)
				{
					auto child = children[i];
					if ((mask & (1u << i)) != 0)
						child.visibility = EVisibility::Collapsed;

					collapse.Upsert(child);
				}
				engine.TransactionApply(collapse.Build(1, 2));
				for (auto dpi : { 96.0f, 120.0f, 144.0f, 192.0f })
				{
					engine.Update(Viewport(800, 600, dpi));
					auto const nodes = SnapshotBounds(engine);
					auto cursor_x = 5.0f;
					auto cursor_y = 5.0f;
					for (auto i = 0u; i != children.size(); ++i)
					{
						auto const id = children[i].id;
						if ((mask & (1u << i)) != 0)
						{
							PR_EXPECT(RectEqual(BoundsOf(nodes, id), Rect{}));
							PR_EXPECT(RectEqual(BoundsOf(nodes, id + 10), Rect{}));
							continue;
						}

						// Only retained children consume a main-axis size and the gap before the next retained child.
						switch (mode)
						{
							case ELayoutMode::StackHorizontal:
							{
								PR_EXPECT(RectEqual(BoundsOf(nodes, id), Rect{cursor_x + 2, 7, 40, 30}));
								cursor_x += 44 + 7;
								break;
							}
							case ELayoutMode::StackVertical:
							{
								PR_EXPECT(RectEqual(BoundsOf(nodes, id), Rect{7, cursor_y + 2, 40, 30}));
								cursor_y += 34 + 7;
								break;
							}
							case ELayoutMode::Overlay:
							case ELayoutMode::Canvas:
							case ELayoutMode::Scroll:
							{
								PR_EXPECT(RectEqual(BoundsOf(nodes, id), BoundsOf(original, id)));
								break;
							}
							default: { throw std::runtime_error("unexpected test mode"); }
						}
					}
				}

				// Restoration always starts from authored descriptors rather than the zeroed collapsed rectangles.
				auto restore = TxnBuilder{};
				for (auto const& child : children)
					restore.Upsert(child);

				engine.TransactionApply(restore.Build(2, 3));
				engine.Update(Viewport(600, 400));
				for (auto const& node : SnapshotBounds(engine))
				{
					PR_EXPECT(RectEqual(node.bounds, BoundsOf(original, node.id)));
					auto const before = std::find_if(original.begin(), original.end(), [&](SemanticNode const& n) { return n.id == node.id; });
					PR_EXPECT(node.state_flags == before->state_flags);
					PR_EXPECT(node.supported_actions == before->supported_actions);
				}
			}
		}
	}

	// Hidden and collapsed roots suppress their descendants while keeping their distinct layout contracts.
	PRUnitTest(LayoutRootVisibilityControlsTheEntireSubtree, Quick)
	{
		auto engine = UiEngine(MakeConfig());
		auto root = MakeControl(1, 0, EControlType::Root, ELayoutMode::StackVertical, Lp(0, 0));
		auto revision = std::uint64_t{0};
		for (auto visibility : { EVisibility::Visible, EVisibility::Hidden, EVisibility::Collapsed, EVisibility::Visible })
		{
			auto b = TxnBuilder{};
			root.visibility = visibility;
			b.Upsert(root);
			b.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(40, 30)));
			engine.TransactionApply(b.Build(revision, revision + 1));
			++revision;
			engine.Update(Viewport(800, 600, 192));
			auto const nodes = SnapshotBounds(engine);
			switch (visibility)
			{
				case EVisibility::Visible:
				{
					PR_EXPECT(!engine.DrawPackets().items.empty());
					PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{0, 0, 400, 300}));
					break;
				}
				case EVisibility::Hidden:
				{
					PR_EXPECT(engine.DrawPackets().items.empty());
					PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{0, 0, 400, 300}));
					PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{0, 0, 40, 30}));
					break;
				}
				case EVisibility::Collapsed:
				{
					PR_EXPECT(engine.DrawPackets().items.empty());
					PR_EXPECT(RectEqual(BoundsOf(nodes, 1), Rect{}));
					PR_EXPECT(RectEqual(BoundsOf(nodes, 2), Rect{}));
					break;
				}
				default: { throw std::runtime_error("unexpected visibility"); }
			}
		}
	}

	// Unavailable ancestors cancel child interaction and are reflected in every descendant's semantics.
	PRUnitTest(VisibilityTransitionsReconcileFocusCaptureAndSemantics, Quick)
	{
		for (auto visibility : { EVisibility::Hidden, EVisibility::Collapsed })
		{
			auto engine = UiEngine(MakeConfig());
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackVertical, Lp(300, 200)));
			auto parent = MakeControl(2, 1, EControlType::Panel, ELayoutMode::StackVertical, Lp(200, 100));
			b.Upsert(parent);
			auto box = MakeControl(3, 2, EControlType::TextBox, ELayoutMode::Overlay, Lp(100, 30));
			std::tie(box.text_offset, box.text_length) = b.AddText("edit");
			b.Upsert(box);
			b.Upsert(MakeControl(4, 2, EControlType::Button, ELayoutMode::Overlay, Lp(100, 30)));
			b.Upsert(MakeControl(5, 1, EControlType::Button, ELayoutMode::Overlay, Lp(100, 30)));
			engine.TransactionApply(b.Build(0, 1));
			engine.Update(Viewport(300, 200));
			engine.InputInject(PointerDownInput(10, 10));
			engine.InputInject(TextInputRecord(EInputKind::CompositionStart));
			engine.InputInjectText(TextInputRecord(EInputKind::CompositionUpdate), TextPayload("candidate", 9, 9, 9));
			auto old_events = std::vector<Event>(engine.EventCount());
			auto old_payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
			engine.EventsCopy(old_events, old_payload);

			// Hiding/collapsing an ancestor must release capture and move focus to the remaining eligible sibling.
			auto hide = TxnBuilder{};
			parent.visibility = visibility;
			hide.Upsert(parent);
			engine.TransactionApply(hide.Build(1, 2));
			engine.Update(Viewport(300, 200));
			auto const nodes = SnapshotBounds(engine);
			for (auto const& node : nodes)
			{
				if (node.id >= 2 && node.id <= 4)
				{
					PR_EXPECT((node.state_flags & static_cast<std::uint32_t>(ESemanticState::Visible)) == 0);
					PR_EXPECT((node.state_flags & static_cast<std::uint32_t>(ESemanticState::Offscreen)) != 0);
					PR_EXPECT(node.supported_actions == 0);
					PR_EXPECT((node.text_flags & static_cast<std::uint32_t>(ESemanticTextFlag::Composing)) == 0);
				}
				if (node.id == 5)
					PR_EXPECT((node.state_flags & static_cast<std::uint32_t>(ESemanticState::Focused)) != 0);
			}
			auto events = std::vector<Event>(engine.EventCount());
			auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
			engine.EventsCopy(events, payload);
			PR_EXPECT(std::any_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::PointerCaptureChanged && e.control_id == 0; }));
			PR_EXPECT(std::any_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::FocusChanged && e.control_id == 5; }));
			PR_THROWS(engine.ApplySemanticAction(SemanticActionRequest{.kind = ESemanticActionKind::Invoke, .control_id = 4}), EngineException);
			PR_THROWS(engine.ApplySemanticAction(SemanticActionRequest{.kind = ESemanticActionKind::Focus, .control_id = 3}), EngineException);
			Rect caret{};
			std::int32_t valid = 1;
			engine.CaretGeometry(3, caret, valid);
			PR_EXPECT(valid == 0);

			// Showing the subtree cannot restore a stale pressed/captured state.
			auto show = TxnBuilder{};
			parent.visibility = EVisibility::Visible;
			show.Upsert(parent);
			engine.TransactionApply(show.Build(2, 3));
			engine.Update(Viewport(300, 200));
			engine.InputInject(PointerUpInput(10, 10));
			events.resize(engine.EventCount());
			payload.resize(engine.EventPayloadBytesPending());
			engine.EventsCopy(events, payload);
			PR_EXPECT(std::none_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::CommandInvoked; }));
		}
	}

	// Queue pressure cannot preserve interaction with a subtree that has become unavailable.
	PRUnitTest(VisibilityReconciliationRemainsSafeWhenNotificationsOverflow, Quick)
	{
		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(100, 100)));
		b.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(40, 30)));
		auto tree = TreeModel{}.Apply(b.Build(0, 1), MakeConfig());
		auto const old_tab_order = ComputeTabOrder(tree);
		tree.m_controls.at(1).desc.visibility = EVisibility::Collapsed;
		auto state = InputState{};
		state.m_hover_id = state.m_pressed_id = state.m_captured_id = state.m_focus_id = 2;
		auto events = EventQueue{2};
		PR_EXPECT(events.Push(2, EEventKind::CommandInvoked, 1, 0, {}));
		PR_THROWS(ReconcileInputAfterTransaction(tree, old_tab_order, state, events, 2), EngineException);
		PR_EXPECT(state.m_hover_id == 0);
		PR_EXPECT(state.m_pressed_id == 0);
		PR_EXPECT(state.m_captured_id == 0);
		PR_EXPECT(state.m_focus_id == 0);

		// Capture is also released when there was no focus to reconcile, without inventing focus.
		state.m_captured_id = 2;
		auto available_events = EventQueue{4};
		PR_EXPECT(ReconcileInputAfterTransaction(tree, old_tab_order, state, available_events, 2) != 0);
		PR_EXPECT(state.m_captured_id == 0);
		PR_EXPECT(state.m_focus_id == 0);
	}

	// Visibility hides a subtree without changing its retained stack allocation.
	PRUnitTest(LayoutHiddenChildrenKeepTheirStackSpaceAcrossVisibilityChanges, Quick)
	{
		for (auto mode : { ELayoutMode::StackVertical, ELayoutMode::StackHorizontal })
		{
			auto engine = UiEngine(MakeConfig());
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(1, 0, EControlType::Root, mode, Lp(200.0f, 200.0f, EHAlign::Left, EVAlign::Top, 8.0f)));
			auto hidden = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(40.0f, 40.0f));
			hidden.visibility = EVisibility::Hidden;
			b.Upsert(hidden);
			b.Upsert(MakeControl(3, 2, EControlType::Button, ELayoutMode::Overlay, Lp(20.0f, 20.0f)));
			b.Upsert(MakeControl(4, 1, EControlType::Button, ELayoutMode::Overlay, Lp(40.0f, 40.0f)));
			engine.TransactionApply(b.Build(0, 1));
			engine.Update(Viewport(200, 200));
			auto const before = SnapshotBounds(engine);
			switch (mode)
			{
				case ELayoutMode::StackVertical: { PR_EXPECT(BoundsOf(before, 4).y == 48.0f); break; }
				case ELayoutMode::StackHorizontal: { PR_EXPECT(BoundsOf(before, 4).x == 48.0f); break; }
				default: { throw std::runtime_error("unexpected test layout mode"); }
			}
			PR_EXPECT(std::none_of(engine.DrawPackets().items.begin(), engine.DrawPackets().items.end(), [](DrawItem const& item) { return item.control_id == 2 || item.control_id == 3; }));
			engine.InputInject(PointerDownInput(10.0f, 10.0f));
			engine.InputInject(PointerUpInput(10.0f, 10.0f));
			auto events = std::vector<Event>(engine.EventCount());
			auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
			engine.EventsCopy(events, payload);
			PR_EXPECT(std::none_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::CommandInvoked; }));

			// Restoring visibility must restore drawing without moving this or later siblings.
			auto show = TxnBuilder{};
			hidden.visibility = EVisibility::Visible;
			show.Upsert(hidden);
			engine.TransactionApply(show.Build(1, 2));
			engine.Update(Viewport(200, 200));
			auto const after = SnapshotBounds(engine);
			for (auto const& node : after)
				PR_EXPECT(RectEqual(node.bounds, BoundsOf(before, node.id)));

			// A restored descendant must reappear in the draw packet.
			PR_EXPECT(std::any_of(engine.DrawPackets().items.begin(), engine.DrawPackets().items.end(), [](DrawItem const& item) { return item.control_id == 3; }));
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
