#include "forward.h"
#include "render_test_support.h"

// Fixture bytecode calls the production atlas sampler with synthetic depth maps.
namespace render_tests::compiled
{
	#include "shadow_transition.h"
}

namespace render_tests
{
	// Sample deliberately disagreeing depth maps to measure cascade-border continuity independently of scene geometry and shadow bias.
	void ShadowTransitionTests()
	{
		// A fine black view and a coarser white view make any hard selection boundary a full visibility jump.
		Fixture fixture(1);
		auto* device = fixture.m_device.Get();
		constexpr UINT width = 512, height = 256, sample_count = 257, case_count = 27, edge_count = 8;
		constexpr UINT result_count = sample_count * case_count * edge_count;
		auto buffer = [device](UINT64 bytes, D3D12_HEAP_TYPE type, D3D12_RESOURCE_STATES state, D3D12_RESOURCE_FLAGS flags = D3D12_RESOURCE_FLAG_NONE)
		{
			// Each buffer owns its storage through completion of the bounded command list.
			auto heap = D3D12_HEAP_PROPERTIES{.Type = type};
			auto desc = D3D12_RESOURCE_DESC{.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER, .Width = bytes, .Height = 1, .DepthOrArraySize = 1, .MipLevels = 1, .SampleDesc = {1, 0}, .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR, .Flags = flags};
			ComPtr<ID3D12Resource> resource;
			Check(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, state, nullptr, IID_PPV_ARGS(&resource)));
			return resource;
		};
		auto upload = [&](void const* data, size_t bytes)
		{
			// Upload heaps keep immutable fixture inputs available without a second copy command.
			auto resource = buffer(bytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
			void* mapped = nullptr;
			Check(resource->Map(0, nullptr, &mapped));
			std::memcpy(mapped, data, bytes);
			resource->Unmap(0, nullptr);
			return resource;
		};

		// Duplicate only the public shader ABI here; the production ShadowVisibility implementation runs unchanged on the GPU.
		struct View
		{
			pr::m4x4 m_w2s;
			pr::v4 m_rect;
			pr::v4 m_bias;
		};
		static_assert(sizeof(View) == 96);
		auto views = std::array<View, case_count * 3>{};
		for (UINT c = 0; c != case_count; ++c)
		{
			// The second view has half the resolution and twice the world coverage; a third view exercises overlapping blend bands.
			auto filter = c < 9 ? 5.0f : 7.0f;
			auto fine = View{pr::m4x4::Identity(), {0.5f, 1, 0, 0}, {0, filter, 0, 0}};
			auto coarse = View{pr::m4x4{{0.5f, 0, 0, 0}, {0, 0.5f, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}, {0.25f, 0.5f, 0.5f, 0}, {0, filter, 0, 0}};
			views[3 * c] = fine;
			views[3 * c + 1] = coarse;
			views[3 * c + 2] = coarse;
			views[3 * c + 1].m_w2s.pos.x = c % 9 == 2 ? 100.0f : 0.0f;
			views[3 * c].m_bias.z = c % 9 == 3 ? 10.0f : 0.0f;
			views[3 * c].m_bias.w = c % 9 == 3 ? 20.0f : 0.0f;
			if (c % 9 == 4)
			{
				// The medium view overlaps the fine blend band but is black, so the final broad white view must also contribute.
				views[3 * c + 1].m_w2s.x.x = 0.95f;
				views[3 * c + 1].m_w2s.y.y = 0.95f;
				views[3 * c + 1].m_rect = pr::v4{0.25f, 0.5f, 0.75f, 0};
			}
			if (c % 9 == 8)
			{
				// A non-nested coarse view enters inside the fine blend band; its appearance must not create another seam.
				views[3 * c + 1].m_w2s.pos.x = -1.42f;
				views[3 * c + 1].m_bias.y = 1;
			}
		}
		auto view_buffer = upload(views.data(), sizeof(views));
		auto pixels = std::vector<float>(width * height, 0.0f);
		for (UINT y = 0; y != 128; ++y)
		{
			// White occupies only the coarse view; adjacent atlas regions stay black to expose cross-region filtering.
			for (UINT x = 256; x != 384; ++x)
				pixels[y * width + x] = 1;
		}
		auto texture_upload = upload(pixels.data(), pixels.size() * sizeof(float));
		auto heap = D3D12_HEAP_PROPERTIES{.Type = D3D12_HEAP_TYPE_DEFAULT};
		auto texture_desc = D3D12_RESOURCE_DESC{.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D, .Width = width, .Height = height, .DepthOrArraySize = 1, .MipLevels = 1, .Format = DXGI_FORMAT_R32_FLOAT, .SampleDesc = {1, 0}};
		ComPtr<ID3D12Resource> atlas;
		Check(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &texture_desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&atlas)));
		auto output = buffer(result_count * sizeof(float), D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
		auto readback = buffer(result_count * sizeof(float), D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_STATE_COPY_DEST);

		// Bind the atlas as a texture table and the structured inputs/output as root descriptors, matching their HLSL resource types.
		auto range = D3D12_DESCRIPTOR_RANGE{D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0, 0};
		auto parameters = std::array<D3D12_ROOT_PARAMETER, 3>{};
		parameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
		parameters[0].DescriptorTable = {1, &range};
		parameters[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
		parameters[1].Descriptor = {1, 0};
		parameters[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
		parameters[2].Descriptor = {0, 0};
		auto sampler = D3D12_STATIC_SAMPLER_DESC{.Filter = D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT, .AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP, .AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP, .AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP, .ComparisonFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL, .MaxLOD = D3D12_FLOAT32_MAX};
		auto signature_desc = D3D12_ROOT_SIGNATURE_DESC{static_cast<UINT>(parameters.size()), parameters.data(), 1, &sampler};
		ComPtr<ID3DBlob> signature_bytes, errors;
		Check(D3D12SerializeRootSignature(&signature_desc, D3D_ROOT_SIGNATURE_VERSION_1, &signature_bytes, &errors));
		ComPtr<ID3D12RootSignature> signature;
		Check(device->CreateRootSignature(0, signature_bytes->GetBufferPointer(), signature_bytes->GetBufferSize(), IID_PPV_ARGS(&signature)));
		auto pipeline_desc = D3D12_COMPUTE_PIPELINE_STATE_DESC{.pRootSignature = signature.Get(), .CS = {compiled::shadow_transition, sizeof(compiled::shadow_transition)}};
		ComPtr<ID3D12PipelineState> pipeline;
		Check(device->CreateComputePipelineState(&pipeline_desc, IID_PPV_ARGS(&pipeline)));
		auto descriptor_desc = D3D12_DESCRIPTOR_HEAP_DESC{D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, 1, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE};
		ComPtr<ID3D12DescriptorHeap> descriptors;
		Check(device->CreateDescriptorHeap(&descriptor_desc, IID_PPV_ARGS(&descriptors)));
		auto srv = D3D12_SHADER_RESOURCE_VIEW_DESC{.Format = DXGI_FORMAT_R32_FLOAT, .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D, .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING};
		srv.Texture2D.MipLevels = 1;
		device->CreateShaderResourceView(atlas.Get(), &srv, descriptors->GetCPUDescriptorHandleForHeapStart());

		// Execute and read back a bounded dispatch on a fixture-owned queue without touching a running application's device.
		auto queue_desc = D3D12_COMMAND_QUEUE_DESC{};
		ComPtr<ID3D12CommandQueue> queue;
		ComPtr<ID3D12CommandAllocator> allocator;
		ComPtr<ID3D12GraphicsCommandList> list;
		Check(device->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&queue)));
		Check(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
		Check(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), pipeline.Get(), IID_PPV_ARGS(&list)));
		auto source = D3D12_TEXTURE_COPY_LOCATION{.pResource = texture_upload.Get(), .Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT};
		source.PlacedFootprint.Footprint = {DXGI_FORMAT_R32_FLOAT, width, height, 1, width * sizeof(float)};
		auto destination = D3D12_TEXTURE_COPY_LOCATION{.pResource = atlas.Get(), .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX};
		list->CopyTextureRegion(&destination, 0, 0, 0, &source, nullptr);
		auto barrier = D3D12_RESOURCE_BARRIER{.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION};
		barrier.Transition = {atlas.Get(), D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE};
		list->ResourceBarrier(1, &barrier);
		ID3D12DescriptorHeap* heaps[] = {descriptors.Get()};
		list->SetDescriptorHeaps(1, heaps);
		list->SetComputeRootSignature(signature.Get());
		list->SetComputeRootDescriptorTable(0, descriptors->GetGPUDescriptorHandleForHeapStart());
		list->SetComputeRootShaderResourceView(1, view_buffer->GetGPUVirtualAddress());
		list->SetComputeRootUnorderedAccessView(2, output->GetGPUVirtualAddress());
		list->Dispatch((sample_count + 63) / 64, case_count, edge_count);
		barrier.Transition = {output.Get(), D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE};
		list->ResourceBarrier(1, &barrier);
		list->CopyBufferRegion(readback.Get(), 0, output.Get(), 0, result_count * sizeof(float));
		Check(list->Close());
		ID3D12CommandList* lists[] = {list.Get()};
		queue->ExecuteCommandLists(1, lists);
		ComPtr<ID3D12Fence> fence;
		Check(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
		Check(queue->Signal(fence.Get(), 1));
		auto event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
		Require(event != nullptr, "Shadow transition fence event creation failed");
		auto completion = fence->SetEventOnCompletion(1, event);
		auto wait = SUCCEEDED(completion) ? WaitForSingleObject(event, 10000) : WAIT_FAILED;
		CloseHandle(event);
		Check(completion);
		Require(wait == WAIT_OBJECT_0, "Shadow transition readback timed out");

		// Assert the full transition profile, not merely a blended pixel, and retain constant single-view/fallback behavior.
		void* mapped = nullptr;
		Check(readback->Map(0, nullptr, &mapped));
		auto results = std::vector<float>(result_count);
		std::memcpy(results.data(), mapped, result_count * sizeof(float));
		readback->Unmap(0, nullptr);
		for (UINT c = 0; c != case_count; ++c)
		{
			// Check 5/7-texel filters and the single-sample shader limit on all four sides and corners.
			auto mode = c % 9;
			for (UINT edge = 0; edge != edge_count; ++edge)
			{
				// Smooth cases must contain a broad monotonic ramp, with no hard jump at the exact filter-safe border.
				auto const* row = results.data() + (c * edge_count + edge) * sample_count;
				if (mode == 0 || mode == 3 || mode == 4 || mode == 8)
				{
					// Distance fading affects the final visibility once, after the cascade contributions are combined.
					auto floor = mode == 3 ? 0.5f : 0.0f;
					Require(std::abs(row[0] - floor) < 1e-4f && std::abs(row[256] - 1) < 1e-4f, "Cascade transition endpoints changed");
					auto blended = 0;
					for (UINT i = 1; i != sample_count; ++i)
					{
						// A hard selection gives a unit jump and no intermediate values, even if its location moves.
						Require(std::isfinite(row[i]) && row[i] >= row[i - 1] - 1e-4f && row[i] - row[i - 1] < (mode == 8 ? 0.08f : 0.04f), "Cascade border is discontinuous");
						blended += row[i] > floor + 0.05f && row[i] < 0.95f;
					}
					Require(blended > 25, "Cascade transition is too narrow to hide the map border");
				}
				else
				{
					// An absent coarse map keeps the fine shadow, while receivers outside every map remain lit.
					for (UINT i = 0; i != sample_count; ++i)
					{
						// The fully outside tail of a single/fallback view is lit; covered samples must not fade to white.
						auto x = 0.6f + 0.45f * i / 256.0f;
						auto expected = mode == 7 || x > 1 ? 1.0f : 0.0f;
						Require(std::abs(row[i] - expected) < 1e-4f, "Single-view or missing-coverage shadow behavior changed");
					}
				}
			}
		}
		fixture.CheckDebugLayer();
		std::cout << "PASS 55,512 GPU shadow samples: borders, corners, unequal resolutions, overlapping bands, non-nested coverage, distance fade, and point/spot isolation\n";
	}

	// Average linear red over an image rectangle
	float RegionRed(std::vector<unsigned char> const& image, int x0, int x1, int y0, int y1)
	{
		// Red is enough because the test lights and surfaces are grey
		auto sum = 0.0f;
		for (auto y = y0; y != y1; ++y)
		{
			for (auto x = x0; x != x1; ++x)
				sum += Linear(image[4 * (y * ImageSize + x)]);
		}
		return sum / ((x1 - x0) * (y1 - y0));
	}

	// Verify shadow atlas rendering for point and spot lights, batching across viewport arrays, and the shadow settings API
	void ShadowTests(int samples)
	{
		// A receiver behind a narrow caster, lit from the side so the shadow falls on the visible part of the receiver.
		// The caster covers world x in [-10,10] at depth 40. A light at (30,0,-20) projects it onto the receiver at depth 60 over x in [-50,-10].
		Fixture fixture(samples);
		auto receiver = fixture.Quad(60, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto caster = fixture.Quad(40, 0xFFFFFFFF, 10, nullptr, 0, 0xFFFFFFFF, false, true);
		(void)receiver;

		// Image regions (pixels) inside the expected shadow and on the lit receiver outside it
		auto shadow_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 15, 35, 40, 88); };
		auto lit_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 90, 110, 40, 88); };

		// Replace the default directional light with a single point light that casts shadows
		auto main_light = View3D_LightGet(fixture.m_window, 0);
		main_light.m_on = FALSE;
		View3D_LightSet(fixture.m_window, 0, main_light);
		View3D_AmbientSet(fixture.m_window, 0xFF000000);
		auto light = api::Light{};
		light.m_type = api::ELight::Point;
		light.m_position = api::Vec4{30, 0, -20, 1};
		light.m_direction = api::Vec4{0, 0, -1, 0};
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_specular_power = 1;
		light.m_intensity = 1;
		light.m_range = 500;
		light.m_falloff = 0;
		light.m_inner_angle = 0.5f;
		light.m_outer_angle = 1.2f;
		light.m_cast_shadow = 1.0f;
		light.m_cam_relative = FALSE;
		light.m_on = TRUE;
		auto point_index = View3D_LightAdd(fixture.m_window, light);
		fixture.CheckErrors();

		// Settings round trip with the documented defaults
		auto settings = View3D_ShadowSettingsGet(fixture.m_window);
		Require(settings.m_atlas_size == 4096 && settings.m_max_shadow_lights == 4, "Unexpected default shadow settings");

		// Compare the image with and without the caster in the shadow pass
		auto shadow_difference = [&](char const* message)
		{
			// The caster must darken its shadow region without changing the lit region
			auto with_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
			auto without_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
			fixture.CheckErrors();
			Require(shadow_region(with_shadow) < 0.5f * shadow_region(without_shadow), message);
			Require(std::abs(lit_region(with_shadow) - lit_region(without_shadow)) < 0.02f, "Shadow darkened a region the caster cannot shade");
			return ImageDifference(with_shadow, without_shadow);
		};

		// Point lights render six cube face views
		shadow_difference("Point light cube shadow missing");

		// Spot lights render one perspective view
		light.m_type = api::ELight::Spot;
		light.m_direction = api::Vec4{-0.6f, 0, -0.8f, 0};
		View3D_LightSet(fixture.m_window, point_index, light);
		shadow_difference("Spot light shadow missing");

		// Four point shadow lights need 24 views, more than one viewport batch. The first light's shadow must survive batching.
		// The extra lights are far above the scene so they light the receiver evenly without shading the tested regions.
		light.m_type = api::ELight::Point;
		View3D_LightSet(fixture.m_window, point_index, light);
		for (int i = 0; i != 3; ++i)
		{
			auto extra = light;
			extra.m_position = api::Vec4{-40.0f + 40.0f * i, 60, 200, 1};
			extra.m_intensity = 0.1f;
			View3D_LightAdd(fixture.m_window, extra);
		}
		fixture.CheckErrors();
		shadow_difference("Point light shadow missing when shadow views span more than one batch");

		// Zero shadow lights disables shadows, making the caster flag irrelevant
		settings.m_max_shadow_lights = 0;
		View3D_ShadowSettingsSet(fixture.m_window, settings);
		{
			auto disabled = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
			auto excluded = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
			fixture.CheckErrors();
			Require(ImageDifference(disabled, excluded) == 0, "Shadows rendered with zero shadow lights");
		}

		// A smaller atlas recreates the atlas and still renders shadows
		settings.m_max_shadow_lights = 4;
		settings.m_atlas_size = 1024;
		View3D_ShadowSettingsSet(fixture.m_window, settings);
		Require(View3D_ShadowSettingsGet(fixture.m_window).m_atlas_size == 1024, "Shadow settings did not round trip");
		shadow_difference("Shadow missing after atlas resize");

		// Invalid settings are reported and leave the current settings unchanged
		auto invalid = settings;
		invalid.m_atlas_size = 1000;
		View3D_ShadowSettingsSet(fixture.m_window, invalid);
		Require(!fixture.m_errors.empty(), "Non power-of-two atlas size was accepted");
		fixture.m_errors.clear();
		Require(View3D_ShadowSettingsGet(fixture.m_window).m_atlas_size == 1024, "Rejected shadow settings changed the scene");
		std::cout << "PASS point/spot shadow views, viewport batching, shadow light limit, atlas resize, and settings validation: MSAA " << samples << '\n';
	}

	// Verify directional light cascades, the shadow edge filter sizes, and shadow view caching
	void DirectionalShadowTests(int samples)
	{
		// A receiver behind a narrow caster, lit by a directional light travelling along (-1.5, 0, -1).
		// The caster covers world x in [-10,10] at depth 40, so its shadow on the receiver at depth 60 covers x in [-40,-20].
		Fixture fixture(samples);
		auto receiver = fixture.Quad(60, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto caster = fixture.Quad(40, 0xFFFFFFFF, 10, nullptr, 0, 0xFFFFFFFF, false, true);
		(void)receiver;

		// Image regions (pixels) inside the expected shadow and on the lit receiver outside it
		auto shadow_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 15, 35, 40, 88); };
		auto lit_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 90, 110, 40, 88); };

		// Make the main light a world-space directional light that casts shadows
		View3D_AmbientSet(fixture.m_window, 0xFF000000);
		auto light = View3D_LightGet(fixture.m_window, 0);
		light.m_type = api::ELight::Directional;
		light.m_direction = api::Vec4{-0.8320503f, 0, -0.5547002f, 0};
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 1;
		light.m_cast_shadow = 1.0f;
		light.m_cam_relative = FALSE;
		light.m_on = TRUE;
		View3D_LightSet(fixture.m_window, 0, light);
		fixture.CheckErrors();

		// The documented defaults
		auto settings = View3D_ShadowSettingsGet(fixture.m_window);
		Require(settings.m_cascade_count == 3 && settings.m_directional_resolution == 1024 && settings.m_filter_size == 5 && settings.m_shadow_distance == 0 && settings.m_cache_views != FALSE, "Unexpected default cascade settings");

		// Compare the image with and without the caster in the shadow pass
		auto shadow_difference = [&](char const* message)
		{
			// The caster must darken its shadow region without changing the lit region
			auto with_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
			auto without_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
			fixture.CheckErrors();
			Require(shadow_region(with_shadow) < 0.5f * shadow_region(without_shadow), message);
			Require(std::abs(lit_region(with_shadow) - lit_region(without_shadow)) < 0.02f, "Shadow darkened a region the caster cannot shade");
		};

		// Every cascade count and both filter sizes render the shadow
		for (int cascades = 1; cascades <= 4; ++cascades)
		{
			for (int filter : { 5, 7 })
			{
				settings.m_cascade_count = cascades;
				settings.m_filter_size = filter;
				View3D_ShadowSettingsSet(fixture.m_window, settings);
				fixture.CheckErrors();
				shadow_difference("Directional cascade shadow missing");
			}
		}

		// Cached views must match views rendered every frame, both for a static scene and after a caster moves
		auto render_pair = [&](char const* message)
		{
			// Render twice with caching so the second image uses cached views, then once without caching
			settings.m_cache_views = TRUE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			fixture.Image();
			auto cached = fixture.Image();
			settings.m_cache_views = FALSE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			auto uncached = fixture.Image();
			settings.m_cache_views = TRUE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			fixture.CheckErrors();
			Require(ImageDifference(cached, uncached) == 0, message);
			return cached;
		};
		settings.m_cascade_count = 3;
		settings.m_filter_size = 5;
		auto before = render_pair("Cached shadow views differ from rendered views in a static scene");
		api::Mat4x4 moved{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {8,0,0,1}};
		View3D_ObjectO2WSet(caster, moved, nullptr);
		auto first_after_move = fixture.Image();
		auto after = render_pair("Cached shadow views were not updated after a caster moved");
		Require(ImageDifference(before, after) > 50, "Moving the caster did not move its shadow");
		Require(ImageDifference(first_after_move, after) == 0, "The first frame after a caster moved used stale cached views");

		// Invalid cascade and filter settings are rejected
		auto invalid = settings;
		invalid.m_cascade_count = 5;
		View3D_ShadowSettingsSet(fixture.m_window, invalid);
		Require(!fixture.m_errors.empty(), "Cascade count above the maximum was accepted");
		fixture.m_errors.clear();
		invalid = settings;
		invalid.m_filter_size = 6;
		View3D_ShadowSettingsSet(fixture.m_window, invalid);
		Require(!fixture.m_errors.empty(), "Unsupported filter size was accepted");
		fixture.m_errors.clear();
		std::cout << "PASS directional cascades, filter sizes, shadow view caching, and cascade settings validation: MSAA " << samples << '\n';
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_ShadowTransitions, Extended | Serial)
	{
		// Run only the synthetic sampler regression so scene-based shadow cases remain independently selectable.
		render_tests::ShadowTransitionTests();
	}

	PRUnitTest(View3d12_Shadows, Extended | Serial)
	{
		render_tests::ShadowTests(1);
		render_tests::ShadowTests(4);
	}
	PRUnitTest(View3d12_DirectionalShadows, Extended | Serial)
	{
		render_tests::DirectionalShadowTests(1);
		render_tests::DirectionalShadowTests(4);
	}
}
