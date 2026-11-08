#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// A final-overlay marker whose colour and rectangle are supplied through the provider context.
	struct OverlayMarker
	{
		float m_colour[4];
		D3D12_RECT m_rect;
	};

	// Paint the marker described by 'context' so tests can attach several distinguishable providers.
	ui::EHostStatus __stdcall MarkerOverlay(void* context, ui::Pass const* pass)
	{
		switch (pass->m_pass)
		{
			case ui::EPass::FinalOverlay:
			{
				auto const& marker = *static_cast<OverlayMarker const*>(context);
				pass->m_command_list->ClearRenderTargetView(pass->m_rtv, marker.m_colour, 1, &marker.m_rect);
				return ui::EHostStatus::Success;
			}
			case ui::EPass::Prepare:
			case ui::EPass::DepthTested:
			case ui::EPass::OcclusionFaded:
			case ui::EPass::Overlay:
			{
				return ui::EHostStatus::Success;
			}
			default: { throw std::runtime_error("Unknown UI pass"); }
		}
	}

	// Check that several UI providers record in attach order, and that attach and detach identify providers by context.
	void UIProviderOrderTests()
	{
		// Two overlapping markers: red at [4,20) and blue at [12,28), against the black background.
		auto fixture = Fixture(1);
		auto module = GetModuleHandleW(L"view3d-12.dll");
		auto attach = reinterpret_cast<ui::AttachFn>(GetProcAddress(module, ui::AttachExport));
		auto detach = reinterpret_cast<ui::DetachFn>(GetProcAddress(module, ui::DetachExport));
		Require(attach != nullptr && detach != nullptr, "UI bridge missing");
		auto red = OverlayMarker{{1, 0, 0, 1}, {4, 4, 20, 20}};
		auto blue = OverlayMarker{{0, 0, 1, 1}, {12, 12, 28, 28}};
		auto red_provider = ui::Provider{{sizeof(ui::Provider), ui::HostStructVersion}, &red, MarkerOverlay, nullptr};
		auto blue_provider = ui::Provider{{sizeof(ui::Provider), ui::HostStructVersion}, &blue, MarkerOverlay, nullptr};

		// The later provider draws on top where the markers overlap.
		Require(attach(fixture.m_window, &red_provider) == ui::EHostStatus::Success, "First provider attach failed");
		Require(attach(fixture.m_window, &blue_provider) == ui::EHostStatus::Success, "Second provider attach failed");
		auto both = fixture.Image();
		Expect(both, 1,0,0, 6,6);
		Expect(both, 0,0,1, 16,16);
		Expect(both, 0,0,1, 24,24);
		Expect(both, 0,0,0);

		// A context can only be attached once, and detaching an unknown context reports that it is not attached.
		Require(attach(fixture.m_window, &red_provider) == ui::EHostStatus::AlreadyAttached, "Duplicate provider was accepted");
		auto unknown = int{};
		Require(detach(fixture.m_window, &unknown) == ui::EHostStatus::NotAttached, "Unknown provider detach was accepted");

		// Detaching the first provider leaves the second provider drawing.
		Require(detach(fixture.m_window, &red) == ui::EHostStatus::Success, "First provider detach failed");
		auto second_only = fixture.Image();
		Expect(second_only, 0,0,0, 6,6);
		Expect(second_only, 0,0,1, 16,16);

		// Reattaching the first provider moves it to the top of the order.
		Require(attach(fixture.m_window, &red_provider) == ui::EHostStatus::Success, "Provider reattach failed");
		Expect(fixture.Image(), 1,0,0, 16,16);
		Require(detach(fixture.m_window, &red) == ui::EHostStatus::Success, "First provider detach failed");
		Require(detach(fixture.m_window, &blue) == ui::EHostStatus::Success, "Second provider detach failed");
		Expect(fixture.Image(), 0,0,0, 16,16);

		// Any GPU validation error fails the fixture.
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS UI provider order\n";
	}

	// Check that an imgui.dll context attached to a window draws its completed frames into the final overlay, in display colours.
	void ImGuiProviderTests()
	{
		// The context takes the render target format from the window, so it cannot start a frame before the window first renders.
		auto fixture = Fixture(1);
		auto errors = std::vector<std::string>{};
		auto report = [](void* context, char const* message, size_t length)
		{
			// Collect messages so the test can fail with the first one.
			static_cast<std::vector<std::string>*>(context)->emplace_back(message, length);
		};
		auto ui = pr::rdr12::imgui::ImGuiUI({fixture.m_device.Get(), nullptr, fixture.m_hwnd, DXGI_FORMAT_UNKNOWN, 0, 1.0f}, {&errors, report});
		Require(static_cast<bool>(ui), "ImGui context creation failed");
		Require(ui.AttachToWindow(fixture.m_window), "ImGui attach failed");
		Require(!ui.NewFrame(), "ImGui started a frame before the window rendered");
		Expect(fixture.Image(), 0,0,0, 40,40);

		// Check raw bytes because ImGui colours are display colours that must be written without sRGB encoding.
		auto expect_bytes = [](std::vector<unsigned char> const& image, int value, int x, int y)
		{
			// Allow for rounding differences between GPUs.
			auto pixel = image.data() + (y * ImageSize + x) * 4;
			for (auto channel = 0; channel != 3; ++channel)
			{
				if (std::abs(pixel[channel] - value) > 2)
					throw std::runtime_error("ImGui pixel mismatch channel " + std::to_string(channel) + ": actual " + std::to_string(pixel[channel]) + ", expected " + std::to_string(value));
			}
		};

		// An opaque, undecorated window fills [8,72) with the default dark style's window background, (0.06, 0.06, 0.06).
		// A new window may be hidden on its first frame, so the second frame is checked.
		auto image = std::vector<unsigned char>{};
		for (auto frame = 0; frame != 2; ++frame)
		{
			Require(ui.NewFrame(), "ImGui did not start a frame after the window rendered");
			ui.SetNextWindowPos(8, 8);
			ui.SetNextWindowSize(64, 64);
			ui.SetNextWindowBgAlpha(1.0f);
			ui.BeginWindow("ImGuiProviderTest", nullptr, 1 | 2 | 4 | 8);
			ui.EndWindow();
			ui.EndFrame();
			image = fixture.Image();
		}
		expect_bytes(image, 15, 40, 40);
		Expect(image, 0,0,0, 100,100);

		// Detaching stops drawing.
		ui.DetachFromWindow();
		Expect(fixture.Image(), 0,0,0, 40,40);

		// Any error reported by the DLL or GPU validation fails the fixture.
		Require(errors.empty(), errors.empty() ? "" : errors.front().c_str());
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS ImGui provider\n";
	}

	// Check that View3DUI colours reach the window's sRGB target unchanged, rather than being sRGB-encoded a second time.
	void View3DUIColourSpaceTests()
	{
		// The window's final overlay target is sRGB, so the UI shader must output linear colour for the hardware to encode.
		auto fixture = Fixture(1);
		auto runtime = ui::Runtime();
		auto device = static_cast<IUnknown*>(View3D_DeviceLeaseAcquire(fixture.m_context));
		Require(device != nullptr, "View3D device lease failed");
		auto context = ui::UiContext(runtime, device, fixture.m_window);
		device->Release();

		// A transparent screen root holding one opaque mid-grey panel at [16,80) with no border or rounding.
		auto header = [](uint32_t size)
		{
			// Every descriptor carries its size and the header's struct version.
			return ui::StructHeader{size, ui::VIEW3D_UI_STRUCT_VERSION};
		};
		auto control = [&](ui::ControlId id, ui::ControlId parent_id, ui::EControlType type, ui::LayoutParams layout, ui::StyleId style_id)
		{
			// Fixed-layout, always visible control.
			auto desc = ui::ControlDesc{};
			desc.header = header(sizeof(ui::ControlDesc));
			desc.id = id;
			desc.parent_id = parent_id;
			desc.type = type;
			desc.root_policy = ui::ERootPolicy::Screen;
			desc.layout_mode = ui::ELayoutMode::Overlay;
			desc.style_id = style_id;
			desc.enabled = 1;
			desc.visibility = ui::EVisibility::Visible;
			desc.validation_state = ui::EValidationState::NotApplicable;
			desc.layout = layout;
			return desc;
		};
		auto style = [&](ui::StyleId id, ui::Colour fill)
		{
			// The same flat fill in every state channel.
			auto desc = ui::StyleDesc{};
			desc.header = header(sizeof(ui::StyleDesc));
			desc.id = id;
			for (auto& visual : desc.visuals)
				visual = ui::StyleVisual{ .fill = fill, .opacity = 1.0f };

			return desc;
		};

		auto root_layout = ui::LayoutParams{};
		root_layout.h_align = ui::EHAlign::Stretch;
		root_layout.v_align = ui::EVAlign::Stretch;
		auto panel_layout = ui::LayoutParams{};
		panel_layout.width = 64;
		panel_layout.height = 64;
		panel_layout.margin_left = 16;
		panel_layout.margin_top = 16;
		auto builder = ui::TransactionBuilder{};
		builder.AddStyle(style(1, {0.5f, 0.5f, 0.5f, 1.0f}));
		builder.AddStyle(style(2, {0.0f, 0.0f, 0.0f, 0.0f}));
		builder.Upsert(control(1, 0, ui::EControlType::Root, root_layout, 2));
		builder.Upsert(control(2, 1, ui::EControlType::Panel, panel_layout, 1));
		context.TransactionApply(builder.Build(0, 1));
		context.Update(ui::ViewportState{
			.header = header(sizeof(ui::ViewportState)),
			.client_width_px = ImageSize,
			.client_height_px = ImageSize,
			.target_width_px = ImageSize,
			.target_height_px = ImageSize,
			.viewport_width_px = ImageSize,
			.viewport_height_px = ImageSize,
			.dpi = 96.0f,
		});

		// A fill of 0.5 must produce the byte 128 (not 188, its sRGB encoding), and the background stays black.
		auto image = fixture.Image();
		auto pixel = image.data() + (48 * ImageSize + 48) * 4;
		for (auto channel = 0; channel != 3; ++channel)
		{
			if (std::abs(pixel[channel] - 128) > 2)
				throw std::runtime_error("View3DUI pixel mismatch channel " + std::to_string(channel) + ": actual " + std::to_string(pixel[channel]) + ", expected 128");
		}
		Expect(image, 0,0,0, 100,100);

		// Any GPU validation error fails the fixture.
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS View3DUI colour space\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_View3DUIColourSpace, Extended | Serial)
	{
		render_tests::View3DUIColourSpaceTests();
	}
	PRUnitTest(View3d12_UIProviderOrder, Extended | Serial)
	{
		render_tests::UIProviderOrderTests();
	}
	PRUnitTest(View3d12_ImGuiProvider, Extended | Serial)
	{
		render_tests::ImGuiProviderTests();
	}
}
