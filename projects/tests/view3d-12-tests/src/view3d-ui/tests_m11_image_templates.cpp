//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Image resources and brushes, role-based template parts, generated ticks, and vertical value controls.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "input.h"
#include "test_support.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// A fake View3D texture handle. The engine only carries it into draw items, so it never needs to resolve.
		constexpr std::uint64_t FakeTexture = 0x1234;

		// An Image resource over a 'width' x 'height' fake texture.
		ResourceDesc MakeImage(ResourceId id, std::uint32_t width, std::uint32_t height)
		{
			// Image resources need a texture handle and a non-zero size.
			auto desc = MakeResource(id, EResourceKind::Image, Colour{ 1, 1, 1, 1 });
			desc.texture = FakeTexture;
			desc.image_width = width;
			desc.image_height = height;
			return desc;
		}

		// Apply 'txn' and return the status it was rejected with, or Success when it was accepted.
		EStatus ApplyStatus(UiEngine& engine, Transaction const& txn)
		{
			// Rejections report their status through the thrown exception.
			try
			{
				engine.TransactionApply(txn);
				return EStatus::Success;
			}
			catch (EngineException const& ex)
			{
				return ex.Status();
			}
		}

		// The items of the draw packet that belong to 'id' and use 'primitive'.
		std::vector<DrawItem> ItemsOf(UiEngine const& engine, ControlId id, EVisualPrimitive primitive)
		{
			// Keep the packet order so callers can check draw order.
			auto result = std::vector<DrawItem>{};
			for (auto const& item : engine.DrawPackets().items)
			{
				if (item.control_id == id && item.primitive == primitive)
					result.push_back(item);
			}
			return result;
		}

		// True when 'a' and 'b' are within a small tolerance.
		bool Near(float a, float b)
		{
			// Geometry is computed in float, so exact equality is too strict.
			return std::abs(a - b) < 0.001f;
		}

		// True when every field of 'a' is near the matching field of 'b'.
		bool Near(Rect a, Rect b)
		{
			// Compare position and size separately.
			return Near(a.x, b.x) && Near(a.y, b.y) && Near(a.w, b.w) && Near(a.h, b.h);
		}

		// A 200 x 200 screen root with one Panel at the origin using 'style_id'.
		void BuildPanel(TxnBuilder& txn, StyleId style_id, float width, float height)
		{
			// The panel sits at the root's origin, so its bounds equal its size.
			txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200, 200)));
			auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(width, height));
			panel.style_id = style_id;
			txn.Upsert(panel);
		}

		// Drain every pending event from 'ctx'.
		std::vector<Event> DrainEvents(UiContext& ctx)
		{
			// Size both buffers exactly as the drain contract requires.
			auto const sizes = ctx.EventsPendingSizes();
			auto events = std::vector<Event>(sizes.m_count);
			auto payload = std::vector<std::byte>(sizes.m_payload_bytes);
			ctx.EventsCopy(events, payload);
			return events;
		}

		// The value of the last slider proposal in 'events'.
		double LastProposal(std::vector<Event> const& events)
		{
			// Search from the end so the latest proposal wins.
			for (auto it = events.rbegin(); it != events.rend(); ++it)
			{
				if (it->kind == EEventKind::ValueChangeProposed)
					return it->numeric_value;
			}
			throw std::runtime_error("slider proposal event was not found");
		}
	}

	// Image resources need a texture and a size, and a style image brush must name an Image resource.
	PRUnitTest(ImageResourcesAndBrushesAreValidated, Quick)
	{
		// Each case uses a fresh engine so the rejections are independent.
		auto reject = [](auto const& build, EStatus expected)
		{
			// Build the transaction in a fresh engine and check its rejection status.
			auto engine = UiEngine(MakeConfig());
			auto txn = TxnBuilder{};
			build(txn);
			PR_EXPECT(ApplyStatus(engine, txn.Build(0, 1)) == expected);
		};

		// An Image resource without a texture or with no size is an invalid argument.
		reject([](TxnBuilder& txn)
		{
			// No texture handle.
			auto image = MakeImage(5, 16, 16);
			image.texture = 0;
			txn.AddResource(image);
		}, EStatus::InvalidArgument);
		reject([](TxnBuilder& txn)
		{
			// No width.
			txn.AddResource(MakeImage(5, 0, 16));
		}, EStatus::InvalidArgument);
		reject([](TxnBuilder& txn)
		{
			// No height.
			txn.AddResource(MakeImage(5, 16, 0));
		}, EStatus::InvalidArgument);

		// A brush naming an unknown resource, or a resource that is not an image, is an unknown resource.
		reject([](TxnBuilder& txn)
		{
			// Resource 5 does not exist.
			auto style = MakeStyle(10);
			style.visuals[0].image.image_id = 5;
			txn.AddStyle(style);
		}, EStatus::UnknownResource);
		reject([](TxnBuilder& txn)
		{
			// Resource 5 is a colour.
			txn.AddResource(MakeResource(5, EResourceKind::Colour, Colour{ 1, 0, 0, 1 }));
			auto style = MakeStyle(10);
			style.visuals[0].image.image_id = 5;
			txn.AddStyle(style);
		}, EStatus::UnknownResource);

		// A valid image and brush are accepted.
		reject([](TxnBuilder& txn)
		{
			// Image 5 is valid, and style 10 uses it.
			txn.AddResource(MakeImage(5, 16, 16));
			auto style = MakeStyle(10);
			style.visuals[0].image.image_id = 5;
			txn.AddStyle(style);
		}, EStatus::Success);
	}

	// Images, label fonts and part styles cannot be removed while a style or template still uses them.
	PRUnitTest(ImageFontAndPartStyleRemovalIsRejectedWhileReferenced, Quick)
	{
		// Style 10 uses image 5; template 20 uses part style 11 and label font 6.
		auto engine = UiEngine(MakeConfig());
		auto txn = TxnBuilder{};
		txn.AddResource(MakeImage(5, 16, 16));
		txn.AddResource(MakeResource(6, EResourceKind::Font, Colour{ 0, 0, 0, 1 }, 10.0f));
		auto image_style = MakeStyle(10);
		image_style.visuals[0].image.image_id = 5;
		txn.AddStyle(image_style);
		txn.AddStyle(MakeStyle(11));
		auto tmpl = MakeTemplate(20, EControlType::Slider);
		txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Track, .style_id = 11 });
		txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Ticks, .ticks = TickParams{ .interval = 0.5f, .major_length = 4, .thickness = 1, .label_font_id = 6 } });
		txn.AddTemplate(tmpl);
		PR_EXPECT(ApplyStatus(engine, txn.Build(0, 1)) == EStatus::Success);

		// Each removal is rejected and leaves revision 1 authoritative.
		auto remove = [&](auto const& build)
		{
			// Build one removal against revision 1.
			auto change = TxnBuilder{};
			build(change);
			PR_EXPECT(ApplyStatus(engine, change.Build(1, 2)) == EStatus::ResourceInUse);
			PR_EXPECT(engine.DiagnosticsGet().accepted_revision == 1);
		};
		remove([](TxnBuilder& b) { b.RemoveResource(5); });
		remove([](TxnBuilder& b) { b.RemoveResource(6); });
		remove([](TxnBuilder& b) { b.RemoveStyle(11); });
	}

	// A sliced image brush draws nine cells over the fill, and a brush without slices draws one.
	PRUnitTest(ImageBrushNineSliceEmitsCellsWithMatchingUVs, Quick)
	{
		// A 30 x 30 image with 10 pixel slices drawn over a 100 x 60 panel.
		auto engine = UiEngine(MakeConfig());
		auto txn = TxnBuilder{};
		txn.AddResource(MakeImage(5, 30, 30));
		auto style = MakeStyle(10);
		for (auto& visual : style.visuals)
			visual.image = ImageBrush{ .image_id = 5, .slice_left = 10, .slice_top = 10, .slice_right = 10, .slice_bottom = 10, .tint = Colour{ 1, 0.5f, 0.25f, 1 } };

		txn.AddStyle(style);
		BuildPanel(txn, 10, 100, 60);
		engine.TransactionApply(txn.Build(0, 1));
		engine.Update(Viewport(200, 200));

		// Corners keep their size, edges stretch along one axis, and the centre stretches both ways.
		auto const third = 1.0f / 3.0f;
		auto cells = ItemsOf(engine, 2, EVisualPrimitive::Image);
		PR_EXPECT(cells.size() == 9);
		PR_EXPECT(Near(cells[0].bounds, Rect{ 0, 0, 10, 10 }));
		PR_EXPECT(Near(cells[0].uv, Rect{ 0, 0, third, third }));
		PR_EXPECT(Near(cells[4].bounds, Rect{ 10, 10, 80, 40 }));
		PR_EXPECT(Near(cells[4].uv, Rect{ third, third, third, third }));
		PR_EXPECT(Near(cells[8].bounds, Rect{ 90, 50, 10, 10 }));
		PR_EXPECT(Near(cells[8].uv, Rect{ 2 * third, 2 * third, third, third }));
		for (auto const& cell : cells)
		{
			// Every cell carries the texture, its size and the tint.
			PR_EXPECT(cell.image_texture == FakeTexture);
			PR_EXPECT(cell.image_width == 30 && cell.image_height == 30);
			PR_EXPECT(cell.fill.r == 1 && cell.fill.g == 0.5f && cell.fill.b == 0.25f);
		}

		// The image is drawn after the panel's own box.
		auto const& items = engine.DrawPackets().items;
		auto const box = std::find_if(items.begin(), items.end(), [](DrawItem const& item) { return item.control_id == 2; });
		PR_EXPECT(box != items.end() && box->primitive != EVisualPrimitive::Image);

		// Without slices the whole source maps onto the whole box.
		for (auto& visual : style.visuals)
			visual.image = ImageBrush{ .image_id = 5, .tint = Colour{ 1, 1, 1, 1 } };

		auto change = TxnBuilder{};
		change.AddStyle(style);
		engine.TransactionApply(change.Build(1, 2));
		engine.Update(Viewport(200, 200));
		cells = ItemsOf(engine, 2, EVisualPrimitive::Image);
		PR_EXPECT(cells.size() == 1);
		PR_EXPECT(Near(cells[0].bounds, Rect{ 0, 0, 100, 60 }));
		PR_EXPECT(Near(cells[0].uv, Rect{ 0, 0, 1, 1 }));
	}

	// Style transitions interpolate the image tint.
	PRUnitTest(ImageBrushTintFollowsStyleTransitions, Quick)
	{
		// A white tint when enabled and a black tint when disabled, over 100 ms.
		auto engine = UiEngine(MakeConfig());
		auto txn = TxnBuilder{};
		txn.AddResource(MakeImage(5, 8, 8));
		auto style = MakeStyle(10);
		for (auto& visual : style.visuals)
			visual.image = ImageBrush{ .image_id = 5, .tint = Colour{ 1, 1, 1, 1 } };

		auto const disabled = static_cast<std::size_t>(EStateChannel::Disabled);
		style.visuals[disabled].image.tint = Colour{ 0, 0, 0, 1 };
		style.transitions[disabled].duration_ms = 100;
		txn.AddStyle(style);
		BuildPanel(txn, 10, 50, 50);
		engine.TransactionApply(txn.Build(0, 1));
		auto viewport = Viewport(200, 200);
		engine.Update(viewport);
		PR_EXPECT(ItemsOf(engine, 2, EVisualPrimitive::Image).at(0).fill.r == 1);

		// Disabling the panel moves the tint halfway at half the duration.
		auto panel = MakeControl(2, 1, EControlType::Panel, ELayoutMode::Overlay, Lp(50, 50));
		panel.style_id = 10;
		panel.enabled = 0;
		auto disable = TxnBuilder{};
		disable.Upsert(panel);
		engine.TransactionApply(disable.Build(1, 2));
		engine.Update(viewport);
		viewport.time_ms = 50;
		engine.Update(viewport);
		PR_EXPECT(Near(ItemsOf(engine, 2, EVisualPrimitive::Image).at(0).fill.r, 0.5f));
		viewport.time_ms = 100;
		engine.Update(viewport);
		PR_EXPECT(ItemsOf(engine, 2, EVisualPrimitive::Image).at(0).fill.r == 0);
	}

	// An unknown orientation is rejected.
	PRUnitTest(ControlOrientationIsValidated, Quick)
	{
		// Orientation Count is not a real orientation.
		auto engine = UiEngine(MakeConfig());
		auto txn = TxnBuilder{};
		txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200, 200)));
		auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(20, 100));
		slider.orientation = EOrientation::Count;
		txn.Upsert(slider);
		PR_EXPECT(ApplyStatus(engine, txn.Build(0, 1)) == EStatus::UnknownType);
	}

	// Vertical sliders put the minimum at the bottom, for both built-in and template thumbs.
	PRUnitTest(VerticalSliderThumbRisesWithValue, Quick)
	{
		// Draw the slider at 'value' and return its thumb, which is always drawn last.
		auto thumb_at = [](float value, TemplateId template_id)
		{
			// The slider is 20 x 100 at the root's origin.
			auto engine = UiEngine(MakeConfig());
			auto txn = TxnBuilder{};
			auto tmpl = MakeTemplate(20, EControlType::Slider);
			txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Track });
			txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Thumb, .width = 20, .height = 10 });
			txn.AddTemplate(tmpl);
			txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200, 200)));
			auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(20, 100));
			slider.orientation = EOrientation::Vertical;
			slider.template_id = template_id;
			slider.value = value;
			txn.Upsert(slider);
			engine.TransactionApply(txn.Build(0, 1));
			engine.Update(Viewport(200, 200));
			return engine.DrawPackets().items.back().bounds;
		};

		// The built-in thumb moves up as the value grows and stays within the control.
		auto const low = thumb_at(0.0f, 0);
		auto const high = thumb_at(1.0f, 0);
		PR_EXPECT(low.y > high.y);
		PR_EXPECT(high.y >= 0.0f && low.y + low.h <= 100.0f);
		PR_EXPECT(low.w > low.h);

		// The template thumb travels the track inset by half its height: value 0.25 sits at 5 + 90 * 0.75.
		auto const quarter = thumb_at(0.25f, 20);
		PR_EXPECT(Near(quarter, Rect{ 0, 67.5f, 20, 10 }));
		PR_EXPECT(Near(thumb_at(0.0f, 20), Rect{ 0, 90, 20, 10 }));
		PR_EXPECT(Near(thumb_at(1.0f, 20), Rect{ 0, 0, 20, 10 }));
	}

	// Pointer input on a vertical slider maps the bottom to the minimum and the top to the maximum.
	PRUnitTest(VerticalSliderPointerMapsUpwardValues, Quick)
	{
		// Click at 'y' on a fresh vertical slider spanning y = [10, 110] and return the proposal.
		auto click_at = [](float y)
		{
			// The slider has a 0..10 range in whole steps and starts in the middle.
			auto runtime = Runtime{};
			auto device = FakeDevice{};
			auto ctx = UiContext(runtime, &device);
			auto txn = TxnBuilder{};
			txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200, 200)));
			auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, Lp(20, 100));
			slider.layout.margin_left = 10.0f;
			slider.layout.margin_top = 10.0f;
			slider.orientation = EOrientation::Vertical;
			slider.minimum = 0.0f;
			slider.maximum = 10.0f;
			slider.step = 1.0f;
			slider.value = 5.0f;
			txn.Upsert(slider);
			ctx.TransactionApply(txn.Build(0, 1));
			ctx.Update(Viewport(200, 200));
			ctx.InputInject(PointerDownInput(20.0f, y));
			ctx.InputInject(PointerUpInput(20.0f, y));
			return LastProposal(DrainEvents(ctx));
		};

		// Points beyond either end of the travel clamp to that end; the middle maps to the middle.
		PR_EXPECT(click_at(109.0f) == 0.0);
		PR_EXPECT(click_at(11.0f) == 10.0);
		PR_EXPECT(click_at(80.0f) < 5.0);
		PR_EXPECT(click_at(40.0f) > 5.0);
	}

	// A Ticks part draws a box per interval, longer major ticks, and labels at major ticks.
	PRUnitTest(TicksPartDrawsMajorMinorTicksAndLabels, Quick)
	{
		// Build a 100 x 20 slider (or a 20 x 100 vertical one) with a track and one Ticks part, and no thumb, so the
		// values travel the whole control.
		auto build = [](UiEngine& engine, EOrientation orientation, EAnchorPoint align)
		{
			// Ticks every 0.25, every second one major, with one-decimal labels in font 6.
			auto txn = TxnBuilder{};
			txn.AddResource(MakeResource(6, EResourceKind::Font, Colour{ 0, 0, 0, 1 }, 10.0f));
			auto tmpl = MakeTemplate(20, EControlType::Slider);
			txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Track });
			txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Ticks, .ticks = TickParams{ .interval = 0.25f, .major_every = 2, .major_length = 8, .minor_length = 4, .thickness = 1, .align = align, .label_font_id = 6, .label_decimals = 1, .label_gap = 2 } });
			txn.AddTemplate(tmpl);
			txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200, 200)));
			auto const vertical = orientation == EOrientation::Vertical;
			auto slider = MakeControl(2, 1, EControlType::Slider, ELayoutMode::Overlay, vertical ? Lp(20, 100) : Lp(100, 20));
			slider.orientation = orientation;
			slider.template_id = 20;
			txn.Upsert(slider);
			engine.TransactionApply(txn.Build(0, 1));
			engine.Update(Viewport(200, 200));
		};

		// Horizontal ticks start at the band's top, at x = 0, 25, 50, 75 and 100.
		auto engine = UiEngine(MakeConfig());
		build(engine, EOrientation::Horizontal, EAnchorPoint::Min);
		auto ticks = std::vector<DrawItem>{};
		for (auto const& item : engine.DrawPackets().items)
		{
			// Tick boxes are the control's only 1 DIP wide boxes.
			if (item.control_id == 2 && item.primitive != EVisualPrimitive::TextPresenter && Near(item.bounds.w, 1.0f))
				ticks.push_back(item);
		}
		PR_EXPECT(ticks.size() == 5);
		for (auto i = 0; i != static_cast<int>(ticks.size()); ++i)
		{
			// Even ticks are major and 8 long; odd ticks are minor and 4 long.
			PR_EXPECT(Near(ticks[i].bounds, Rect{ 25.0f * i - 0.5f, 0, 1, i % 2 == 0 ? 8.0f : 4.0f }));
		}
		auto labels = ItemsOf(engine, 2, EVisualPrimitive::TextPresenter);
		PR_EXPECT(labels.size() == 3);
		PR_EXPECT(labels[0].text == "0.0" && labels[1].text == "0.5" && labels[2].text == "1.0");
		PR_EXPECT(labels[1].text_align == ETextAlign::Center);
		PR_EXPECT(Near(labels[1].bounds.y, 8 + 2));

		// Vertical ticks aligned to the band's right edge put right-aligned labels to their left, and the minimum at the bottom.
		auto vertical = UiEngine(MakeConfig());
		build(vertical, EOrientation::Vertical, EAnchorPoint::Max);
		labels = ItemsOf(vertical, 2, EVisualPrimitive::TextPresenter);
		PR_EXPECT(labels.size() == 3);
		PR_EXPECT(labels[0].text == "0.0" && labels[2].text == "1.0");
		PR_EXPECT(labels[0].bounds.y > labels[2].bounds.y);
		PR_EXPECT(labels[0].text_align == ETextAlign::Right);
		PR_EXPECT(Near(labels[0].bounds.x + labels[0].bounds.w, 20 - 8 - 2));
	}

	// A Text part with its own style draws a box under the text, inset by its margins.
	PRUnitTest(TextPartWithStyleDrawsItsOwnBox, Quick)
	{
		// The button's Background uses the control style; its Text part uses style 11 inset by 2.
		auto engine = UiEngine(MakeConfig());
		auto txn = TxnBuilder{};
		auto well = MakeStyle(11);
		for (auto& visual : well.visuals)
			visual.fill = Colour{ 0, 0, 1, 1 };

		txn.AddStyle(well);
		auto tmpl = MakeTemplate(20, EControlType::Button);
		txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Background });
		txn.AddTemplatePart(tmpl, TemplatePart{ .role = ETemplatePart::Text, .style_id = 11, .margin_left = 2, .margin_top = 2, .margin_right = 2, .margin_bottom = 2 });
		txn.AddTemplate(tmpl);
		txn.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200, 200)));
		auto button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Overlay, Lp(80, 30));
		button.template_id = 20;
		txn.Upsert(button, "OK", "OK");
		engine.TransactionApply(txn.Build(0, 1));
		engine.Update(Viewport(200, 200));

		// The background box comes first, then the text well, then the text inside the well.
		auto const& items = engine.DrawPackets().items;
		auto own = std::vector<DrawItem>{};
		std::copy_if(items.begin(), items.end(), std::back_inserter(own), [](DrawItem const& item) { return item.control_id == 2; });
		PR_EXPECT(own.size() == 3);
		PR_EXPECT(Near(own[0].bounds, Rect{ 0, 0, 80, 30 }));
		PR_EXPECT(Near(own[1].bounds, Rect{ 2, 2, 76, 26 }));
		PR_EXPECT(own[1].fill.b == 1 && own[1].fill.r == 0);
		PR_EXPECT(own[2].primitive == EVisualPrimitive::TextPresenter);
		PR_EXPECT(Near(own[2].bounds, Rect{ 2, 2, 76, 26 }));
	}
}
