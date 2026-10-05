//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M3 white-box tests for the DLL-internal D3D12 renderer (renderer.h/.cpp): construction/no-throw
// guarantees, reserved-pass and FinalOverlay-before-Prepare bounds that need no real GPU at all,
// and (only where a real ID3D12Device can actually be created in this environment) a full
// Prepare+FinalOverlay round trip plus a deterministic glyph-cache resource-limit surfacing. The
// device-backed tests gracefully no-op (rather than fail) when no D3D12 device is available, since
// CI/headless environments are not guaranteed to expose one.
#include "pr/common/unittests.h"
#include "test_support.h"
#include "renderer.h"
#include <dxgi1_4.h>
#include <optional>
#pragma comment(lib, "dxgi.lib")

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Attempts a hardware D3D12 device first, falling back to the WARP software adapter; returns
		// a null ComPtr (never throws) if neither is available, so device-backed tests can skip
		// themselves cleanly rather than failing in an environment without a usable adapter.
		Microsoft::WRL::ComPtr<ID3D12Device> TryCreateDevice()
		{
			Microsoft::WRL::ComPtr<ID3D12Device> device;
			if (SUCCEEDED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device))))
				return device;

			Microsoft::WRL::ComPtr<IDXGIFactory4> factory;
			if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory))))
				return nullptr;

			Microsoft::WRL::ComPtr<IDXGIAdapter> warp_adapter;
			if (FAILED(factory->EnumWarpAdapter(IID_PPV_ARGS(&warp_adapter))))
				return nullptr;

			if (SUCCEEDED(D3D12CreateDevice(warp_adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device))))
				return device;

			return nullptr;
		}

		// A minimal but complete device-backed rig: one open (never closed/executed) direct command
		// list and one committed render-target resource with an RTV, just enough for Renderer::Record
		// to record real Prepare/FinalOverlay work without this test ever submitting or presenting.
		struct DeviceFixture
		{
			Microsoft::WRL::ComPtr<ID3D12Device> device;
			Microsoft::WRL::ComPtr<ID3D12CommandAllocator> allocator;
			Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> command_list;
			Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtv_heap;
			Microsoft::WRL::ComPtr<ID3D12Resource> render_target;
			D3D12_CPU_DESCRIPTOR_HANDLE rtv{};
		};

		std::optional<DeviceFixture> TryBuildFixture()
		{
			auto device = TryCreateDevice();
			if (device == nullptr)
				return std::nullopt;

			auto fx = DeviceFixture{};
			fx.device = device;

			if (FAILED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&fx.allocator))))
				return std::nullopt;
			if (FAILED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, fx.allocator.Get(), nullptr, IID_PPV_ARGS(&fx.command_list))))
				return std::nullopt;

			auto heap_desc = D3D12_DESCRIPTOR_HEAP_DESC{};
			heap_desc.NumDescriptors = 1;
			heap_desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
			if (FAILED(device->CreateDescriptorHeap(&heap_desc, IID_PPV_ARGS(&fx.rtv_heap))))
				return std::nullopt;

			auto heap_props = D3D12_HEAP_PROPERTIES{};
			heap_props.Type = D3D12_HEAP_TYPE_DEFAULT;

			auto tex_desc = D3D12_RESOURCE_DESC{};
			tex_desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
			tex_desc.Width = 256;
			tex_desc.Height = 128;
			tex_desc.DepthOrArraySize = 1;
			tex_desc.MipLevels = 1;
			tex_desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			tex_desc.SampleDesc.Count = 1;
			tex_desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

			auto clear = D3D12_CLEAR_VALUE{};
			clear.Format = DXGI_FORMAT_R8G8B8A8_UNORM;

			if (FAILED(device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE, &tex_desc, D3D12_RESOURCE_STATE_RENDER_TARGET, &clear, IID_PPV_ARGS(&fx.render_target))))
				return std::nullopt;

			fx.rtv = fx.rtv_heap->GetCPUDescriptorHandleForHeapStart();
			device->CreateRenderTargetView(fx.render_target.Get(), nullptr, fx.rtv);
			return fx;
		}

		Pass MakePreparePass(DeviceFixture const& fx)
		{
			auto pass = Pass{};
			pass.m_header = { sizeof(Pass), HostStructVersion };
			pass.m_pass = EPass::Prepare;
			pass.m_command_list = fx.command_list.Get();
			pass.m_colour_format = DXGI_FORMAT_R8G8B8A8_UNORM;
			pass.m_width = 256;
			pass.m_height = 128;
			pass.m_sample_count = 1;
			pass.m_sample_quality = 0;
			pass.m_dpi_x = 96.0f;
			pass.m_dpi_y = 96.0f;
			return pass;
		}

		// Root(1, 200x100) with a single Text(2, "Hi") child, produced through a real UiEngine so the
		// DrawPacket exercises the same TextPresenter path a live context would.
		DrawPacket BuildTextDrawPacket(UiEngine& engine)
		{
			auto b0 = TxnBuilder{};
			b0.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::Overlay, Lp(200.0f, 100.0f)));
			auto text = MakeControl(2, 1, EControlType::Text, ELayoutMode::Overlay, Lp(150.0f, 30.0f));
			std::tie(text.text_offset, text.text_length) = b0.AddText("Hi");
			b0.Upsert(text);
			engine.TransactionApply(b0.Build(0, 1));
			engine.Update(Viewport(200, 100));
			return engine.DrawPackets();
		}
	}

	PRUnitTest(RendererConstructionNeverTouchesOrQueriesTheDevice, Quick)
	{
		static_assert(noexcept(Renderer(std::declval<IUnknown*>(), std::declval<Config const&>())), "Renderer's constructor must be noexcept to participate in ContextCreate's all-or-nothing rollback");

		auto device = FakeDevice{};
		auto renderer = Renderer(&device, MakeConfig());
		(void)renderer;

		// FakeDevice::QueryInterface always fails and its AddRef/Release are instrumented; an
		// unchanged refcount of 1 proves the constructor stored the raw pointer only.
		PR_EXPECT(device.RefCount() == 1);
	}

	PRUnitTest(RecordDuringPrepareSurfacesInternalErrorForADeviceThatIsNotReallyID3D12Device, Quick)
	{
		auto device = FakeDevice{};
		auto renderer = Renderer(&device, MakeConfig());

		auto pass = Pass{};
		pass.m_header = { sizeof(Pass), HostStructVersion };
		pass.m_pass = EPass::Prepare;
		pass.m_colour_format = DXGI_FORMAT_R8G8B8A8_UNORM;

		auto packet = DrawPacket{};
		PR_EXPECT(renderer.Record(pass, packet) == EStatus::InternalError);
		PR_EXPECT(device.RefCount() == 1); // QueryInterface failed before any AddRef could occur
	}

	PRUnitTest(RecordAcceptsEveryWorldPassAsANoOpBeforeAnyPrepare, Quick)
	{
		// Each world stage must be a trivial success (not a crash, and not the InvalidArgument the
		// pre-M8 renderer returned) when Prepare never ran this session, since a context that failed
		// to attach to a window could still be handed these passes by a misbehaving host.
		auto device = FakeDevice{};
		auto renderer = Renderer(&device, MakeConfig());
		auto packet = DrawPacket{};

		for (auto world : { EPass::DepthTested, EPass::OcclusionFaded, EPass::Overlay })
		{
			auto pass = Pass{};
			pass.m_header = { sizeof(Pass), HostStructVersion };
			pass.m_pass = world;
			// m_command_list/m_rtv/m_dsv are deliberately left null/zeroed: Draw() must never be reached.
			PR_EXPECT(renderer.Record(pass, packet) == EStatus::Success);
		}
		PR_EXPECT(device.RefCount() == 1);
	}

	PRUnitTest(RecordDuringFinalOverlayBeforeAnyPrepareIsADocumentedNoOp, Quick)
	{
		// FinalOverlay must be a trivial success (not a crash) if Prepare never ran this session,
		// since a context that failed to attach to a window would otherwise never call Prepare at
		// all yet could still, in principle, be handed a FinalOverlay pass by a misbehaving host.
		auto device = FakeDevice{};
		auto renderer = Renderer(&device, MakeConfig());

		auto pass = Pass{};
		pass.m_header = { sizeof(Pass), HostStructVersion };
		pass.m_pass = EPass::FinalOverlay;
		// m_command_list/m_rtv are deliberately left null/zeroed: Draw() must never be reached.

		auto packet = DrawPacket{};
		PR_EXPECT(renderer.Record(pass, packet) == EStatus::Success);
	}

	PRUnitTest(RendererCompletesAPrepareThenFinalOverlayRoundTripAgainstARealDevice, Quick)
	{
		auto fixture = TryBuildFixture();
		if (!fixture.has_value())
			return; // no D3D12 adapter (hardware or WARP) available in this environment; nothing to verify

		auto engine = UiEngine(MakeConfig());
		auto packet = BuildTextDrawPacket(engine);
		auto renderer = Renderer(fixture->device.Get(), MakeConfig());

		auto prepare_pass = MakePreparePass(*fixture);
		PR_EXPECT(renderer.Record(prepare_pass, packet) == EStatus::Success);

		auto final_pass = prepare_pass;
		final_pass.m_pass = EPass::FinalOverlay;
		final_pass.m_colour_target = fixture->render_target.Get();
		final_pass.m_rtv = fixture->rtv;
		final_pass.m_viewport = D3D12_VIEWPORT{ 0.0f, 0.0f, 256.0f, 128.0f, 0.0f, 1.0f };
		final_pass.m_scissor = D3D12_RECT{ 0, 0, 256, 128 };
		PR_EXPECT(renderer.Record(final_pass, packet) == EStatus::Success);
	}

	PRUnitTest(RendererSurfacesResourceLimitWhenTheGlyphCacheHasNoPageBudget, Quick)
	{
		auto fixture = TryBuildFixture();
		if (!fixture.has_value())
			return; // see above: gracefully skip without a real adapter

		auto engine = UiEngine(MakeConfig());
		auto packet = BuildTextDrawPacket(engine);

		// A zero-page glyph cache guarantees the very first not-yet-resident glyph this session
		// (every glyph in "Hi") cannot be placed, deterministically forcing ResourceLimit without
		// depending on any pixel-packing arithmetic.
		auto renderer_config = MakeConfig();
		renderer_config.max_glyph_cache_pages = 0;
		auto renderer = Renderer(fixture->device.Get(), renderer_config);

		auto prepare_pass = MakePreparePass(*fixture);
		PR_EXPECT(renderer.Record(prepare_pass, packet) == EStatus::ResourceLimit);
	}

	PRUnitTest(TextRunStartXDipAppliesLeftInsetIndependentlyOfRunWidthAndCentersIgnoringInset, Quick)
	{
		// Pure/static math, no device or live TextShaper needed: exercises exactly the formulas
		// draw_packet_builder.cpp's TextPlacementFor and DrawTextItem's caller depend on.
		auto item = DrawItem{};
		item.bounds = Rect{ 10.0f, 0.0f, 100.0f, 20.0f };

		item.text_align = ETextAlign::Left;
		item.text_inset_dip = 8.0f;
		PR_EXPECT(Renderer::TextRunStartXDip(item, 40.0f) == 18.0f); // bounds.x + inset; the run's own width is irrelevant when left-aligned
		PR_EXPECT(Renderer::TextRunStartXDip(item, 90.0f) == 18.0f); // unchanged even for a run wider than the box

		item.text_align = ETextAlign::Center;
		PR_EXPECT(Renderer::TextRunStartXDip(item, 40.0f) == 40.0f); // bounds.x + (bounds.w - advance) * 0.5; inset is ignored
	}

	PRUnitTest(TextRunStartXDipThrowsForAnUnknownTextAlignValue, Quick)
	{
		auto item = DrawItem{};
		item.text_align = static_cast<ETextAlign>(99);
		PR_THROWS(Renderer::TextRunStartXDip(item, 0.0f), EngineException);
	}
}
