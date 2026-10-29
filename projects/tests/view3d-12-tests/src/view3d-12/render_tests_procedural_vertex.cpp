#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Create one copied procedural vertex shader through the public descriptor.
	api::Shader ProceduralVertexShader(api::ERenderStep rdr_step, unsigned char const* bytecode, size_t bytecode_size, ProceduralVertexConstants const& constants)
	{
		// Supply only the bounded stage, bytecode, constants, and diagnostic name owned by this ABI.
		auto options = api::ShaderOptions{
			.m_struct_size = sizeof(api::ShaderOptions),
			.m_version = api::ShaderOptions::CurrentVersion,
			.m_stage = api::EShaderStage::Vertex,
			.m_bytecode = bytecode,
			.m_bytecode_size = bytecode_size,
			.m_dbg_name = "ProceduralVertexAbiTest",
			.m_procedural = {
				.m_rdr_step = rdr_step,
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			},
		};
		return View3D_ShaderCreate(options);
	}

	// Read DLL-owned physical vertices using SDK commands; this is internal structural evidence, not a public export.
	std::vector<pr::rdr12::Vert> PhysicalVertices(Fixture& fixture, api::Object object)
	{
		// Finish renderer work before inspecting its canonical storage on a separate queue.
		using namespace pr;
		View3D_WindowGSyncWait(fixture.m_window);
		auto& model = *object->m_model.get();
		auto size = static_cast<UINT64>(model.m_vcount * sizeof(rdr12::Vert));
		D3D12_HEAP_PROPERTIES heap{};
		heap.Type = D3D12_HEAP_TYPE_READBACK;
		auto desc = model.m_vb->GetDesc();
		desc.Width = size;
		desc.Flags = D3D12_RESOURCE_FLAG_NONE;
		ComPtr<ID3D12Resource> readback;
		Check(fixture.m_device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&readback)));
		D3D12_COMMAND_QUEUE_DESC queue_desc{};
		ComPtr<ID3D12CommandQueue> queue;
		ComPtr<ID3D12CommandAllocator> allocator;
		ComPtr<ID3D12GraphicsCommandList> list;
		Check(fixture.m_device->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&queue)));
		Check(fixture.m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
		Check(fixture.m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(&list)));
		auto state = compute::DefaultResState(model.m_vb.get());
		D3D12_RESOURCE_BARRIER barrier{};
		barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		barrier.Transition = {model.m_vb.get(), D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, state, D3D12_RESOURCE_STATE_COPY_SOURCE};
		list->ResourceBarrier(1, &barrier);
		list->CopyBufferRegion(readback.Get(), 0, model.m_vb.get(), 0, size);
		std::swap(barrier.Transition.StateBefore, barrier.Transition.StateAfter);
		list->ResourceBarrier(1, &barrier);
		Check(list->Close());
		ID3D12CommandList* lists[] = {list.Get()};
		queue->ExecuteCommandLists(1, lists);
		ComPtr<ID3D12Fence> fence;
		Check(fixture.m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
		Check(queue->Signal(fence.Get(), 1));
		auto event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
		Require(event != nullptr, "Vertex readback event creation failed");
		auto completion = fence->SetEventOnCompletion(1, event);
		auto wait = SUCCEEDED(completion) ? WaitForSingleObject(event, 10000) : WAIT_FAILED;
		CloseHandle(event);
		Check(completion);
		Require(wait == WAIT_OBJECT_0, "Vertex readback timed out");

		// Copy only declared vertices, excluding arbitrary padding from the resource allocation.
		void* mapped = nullptr;
		D3D12_RANGE range{0, static_cast<SIZE_T>(size)};
		Check(readback->Map(0, &range, &mapped));
		auto vertices = std::vector<rdr12::Vert>(static_cast<size_t>(model.m_vcount));
		std::memcpy(vertices.data(), mapped, static_cast<size_t>(size));
		D3D12_RANGE written{0, 0};
		readback->Unmap(0, &written);
		return vertices;
	}

	// Measure an interior patch in linear RGB, away from coverage and silhouette differences.
	pr::v4 LightingPatch(std::vector<unsigned char> const& image)
	{
		// The focused triangle covers this patch under every tested placement.
		auto sum = pr::v4::Zero();
		for (auto y = 60; y != 68; ++y)
		{
			for (auto x = 60; x != 68; ++x)
			{
				auto offset = 4 * (y * ImageSize + x);
				sum += pr::v4(Linear(image[offset]), Linear(image[offset + 1]), Linear(image[offset + 2]), 0);
			}
		}
		return sum / 64.0f;
	}

	// Prove generated capability flags reach stock lighting without reading physical placeholder attributes.
	void ProceduralVertexLightingTests()
	{
		// Public creation/render/readback uses the DLL; descriptor/SetFlags/readback inspections below are explicitly internal.
		using namespace pr;
		namespace rdr = pr::rdr12;
		static_assert(sizeof(api::Vertex) == 48);
		static_assert(sizeof(rdr::Vert) == 64);
		auto fixture = Fixture(1);
		Require(fixture.m_info != nullptr, "Procedural lighting requires D3D12 debug validation");
		std::cout << "Procedural lighting: public creation and internal storage/flag checks\n";
		auto const lit_mask = api::EGeom::Vert | api::EGeom::Colr | api::EGeom::Norm;
		auto const unlit_mask = api::EGeom::Vert | api::EGeom::Colr;
		auto const normal = Normalise(v4(1, 0, 1, 0));
		auto constants = ProceduralVertexConstants{
			.m_positions = {{-40, -40, 0, 1}, {40, -40, 0, 1}, {0, 40, 0, 1}},
			.m_colour = {0.6f, 0.4f, 0.2f, 1},
			.m_normal = {normal.x, normal.y, normal.z, 0},
		};
		auto shader = ProceduralVertexShader(api::ERenderStep::ForwardRender, compiled::procedural_vertex_forward, sizeof(compiled::procedural_vertex_forward), constants);
		Require(shader != nullptr, "Lighting shader creation failed");
		fixture.m_shaders.push_back(shader);
		auto nan = std::numeric_limits<float>::quiet_NaN();
		auto placeholder = api::Vertex{{nan, nan, nan, nan}, {nan, nan, nan, nan}, {nan, nan}, 0x00010203, 0};
		uint32_t indices[] = {70000, 70001, 70002, 70000, 70001, 70002};
		auto options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::ProceduralVertexId,
			.m_vcount_logical = 70003,
			.m_bbox = {{0, 0, 0, 1}, {40, 40, 0, 0}},
		};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = lit_mask;
		nugget.m_v1 = 1;
		nugget.m_i1 = 3;
		nugget.m_tint = 0xFFFFFFFF;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_shaders[0] = {shader, api::ERenderStep::ForwardRender, 0};
		auto create = [&](api::EGeom mask)
		{
			// Each object owns its capabilities independently of the shared shader's emitted data.
			auto desc = nugget;
			desc.m_geom = mask;
			auto object = View3D_ObjectCreateU32("GeneratedLighting", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &desc, options, GUID{});
			Require(object != nullptr, "Generated capability mask rejected");
			fixture.m_objects.push_back(object);
			return object;
		};
		auto lit = create(lit_mask);
		auto unlit = create(unlit_mask);
		auto position_only = create(api::EGeom::Vert);
		auto textured = create(lit_mask | api::EGeom::Tex0);

		// Invalid generated declarations fail at creation, before publishing any model.
		for (auto mask : {api::EGeom::Unknown, api::EGeom::Norm, static_cast<api::EGeom>(1 | (1 << 8))})
		{
			auto desc = nugget;
			desc.m_geom = mask;
			fixture.m_errors.clear();
			auto rejected = View3D_ObjectCreateU32("InvalidGeneratedMask", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &desc, options, GUID{});
			Require(rejected == nullptr, "Invalid generated mask was accepted");
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("supported generated attributes") != std::string::npos, "Invalid generated mask diagnostic missing");
		}
		fixture.m_errors.clear();

		// The physical-copy union must not spread one nugget's Norm bit into another nugget.
		api::Nugget mixed_nuggets[] = {nugget, nugget};
		mixed_nuggets[1].m_geom = unlit_mask;
		mixed_nuggets[1].m_i0 = 3;
		mixed_nuggets[1].m_i1 = 6;
		auto mixed = View3D_ObjectCreateU32("MixedGeneratedCapabilities", 0xFFFFFFFF, 1, 6, 2, &placeholder, indices, mixed_nuggets, options, GUID{});
		Require(mixed != nullptr, "Mixed generated nuggets rejected");
		fixture.m_objects.push_back(mixed);
		auto inspect_flags = [&](api::Object object, rdr::Nugget const& nug, bool has_normals, bool has_texture)
		{
			// This helper runs executable-linked SetFlags on DLL-owned descriptors; rendered tests remain independent public proof.
			auto cb = rdr::shaders::fwd::ElementConstants{};
			rdr::SetFlags(cb, object->m_base, nug.mat(), nug, false);
			Require(((cb.flags.x & rdr::shaders::ModelFlags_HasNormals) != 0) == has_normals, "Per-nugget normal flag mismatch");
			Require(((cb.flags.y & rdr::shaders::TextureFlags_HasDiffuse) != 0) == has_texture, "Generated Tex0 contract mismatch");
		};
		Require(mixed->m_model->m_nuggets->m_next != nullptr, "Mixed nugget chain incomplete");
		for (auto* nug = mixed->m_model->m_nuggets.get(); nug != nullptr; nug = nug->m_next.get())
		{
			// Identify the caller's range independently of the model's chain order.
			auto has_normals = nug->m_irange.begin() == 0;
			Require(nug->m_geom == static_cast<rdr::EGeom>(has_normals ? lit_mask : unlit_mask), "Generated capability mask changed");
			inspect_flags(mixed, *nug, has_normals, false);
		}
		inspect_flags(textured, *textured->m_model->m_nuggets.get(), true, true);

		// Nugget indices follow the order the caller supplied the nuggets, so index 1 is the second range.
		Require(mixed->m_model->m_nuggets->m_irange.begin() == 0, "Nugget chain does not keep creation order");
		View3D_ObjectNuggetFlagsSet(mixed, api::ENuggetFlag::ShadowCastExclude, TRUE, nullptr, 1);
		Require(AllSet(mixed->m_model->m_nuggets->m_next->m_nflags, rdr::ENuggetFlag::ShadowCastExclude) && !AllSet(mixed->m_model->m_nuggets->m_nflags, rdr::ENuggetFlag::ShadowCastExclude), "Nugget index does not follow creation order");
		View3D_ObjectNuggetFlagsSet(mixed, api::ENuggetFlag::ShadowCastExclude, FALSE, nullptr, 1);
		inspect_flags(position_only, *position_only->m_model->m_nuggets.get(), false, false);
		for (auto object : {lit, unlit, position_only, textured, mixed})
		{
			// Generated capabilities neither widen the canonical allocation nor enable input-assembler fetches.
			auto& model = *object->m_model.get();
			Require(model.m_vcount == 1 && model.m_vb_view.StrideInBytes == 64 && model.m_vb_view.SizeInBytes == 64, "Procedural storage is not one canonical Vert");
			for (auto* nug = model.m_nuggets.get(); nug != nullptr; nug = nug->m_next.get())
			{
				auto layout = nug->m_pso.Find<rdr::EPipeState::InputLayout>();
				Require(layout != nullptr && layout->NumElements == 0 && nug->m_vrange == Range(0, 1), "Procedural IA/range contract changed");
			}
			auto vertices = PhysicalVertices(fixture, object);
			auto const& vertex = vertices[0];
			Require(All(vertex.m_vert == v4::Origin()) && vertex.m_diff == ColourWhite && All(vertex.m_norm == v4::Zero()) && All(vertex.m_tex0 == v2::Zero()), "Placeholder fields leaked into canonical storage");
		}
		std::cout << "PASS internal generated masks, mixed nuggets, Tex0, canonical 64-byte storage and empty IA; public Vertex=48 bytes\n";

		// Fixture-only controlled lighting leaves product lights, materials and shadows untouched.
		auto light = SceneLightingGet(fixture.m_window);
		light.m_type = api::ELight::Directional;
		light.m_ambient = 0xFF000000;
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 2;
		light.m_cast_shadow = 0;
		light.m_cam_relative = FALSE;
		light.m_on = TRUE;
		auto placement = m4x4::Identity();
		placement.pos.z = -20;
		auto render = [&](api::Object object, m4x4 const& transform, v4 toward_light)
		{
			// Use only public scene/placement/light/render operations for the framebuffer result.
			fixture.Clear();
			View3D_ObjectO2WSet(object, To<api::Mat4x4>(transform), nullptr);
			View3D_WindowAddObject(fixture.m_window, object);
			light.m_direction = {-toward_light.x, -toward_light.y, -toward_light.z, 0};
			SceneLightingSet(fixture.m_window, light);
			return LightingPatch(fixture.Image());
		};
		auto luminance = [](v4 colour)
		{
			// Linear luminance separates actual illumination from colour-channel noise.
			return Dot3(colour, v4(0.2126f, 0.7152f, 0.0722f, 0));
		};
		auto promote = [&](api::Object object)
		{
			// Neutral procedural PBR preserves the generated albedo and geometric normal.
			auto surface = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Soil);
			surface.m_colour0 = surface.m_colour1 = surface.m_colour2 = surface.m_colour3 = 0xFFFFFFFF;
			surface.m_normal_strength = 0;
			surface.m_roughness_min = surface.m_roughness_max = 1;
			View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
			fixture.CheckErrors();
		};
		for (auto pbr : {false, true})
		{
			// Both stock Forward and stock PBR must obey opt-in normals, not merely emit a visible triangle.
			std::cout << "Procedural lighting: public " << (pbr ? "PBR" : "Forward") << " opposed-light comparison\n";
			if (pbr)
			{
				promote(lit);
				promote(unlit);
			}
			auto front = render(lit, placement, normal);
			auto back = render(lit, placement, -normal);
			auto no_norm_front = render(unlit, placement, normal);
			auto no_norm_back = render(unlit, placement, -normal);
			Require(luminance(front) - luminance(back) >= 0.10f && luminance(back) <= 0.02f, "Generated normals did not produce directional stock lighting");
			Require(Length(no_norm_front - no_norm_back) <= 0.01f && luminance(no_norm_front) > 0.1f, "No-normal procedural surface was forced into lighting");
			std::cout << "PASS public " << (pbr ? "PBR" : "Forward") << " opposed lights: front=" << luminance(front) << " back=" << luminance(back) << " noNormDelta=" << Length(no_norm_front - no_norm_back) << '\n';
		}
		View3D_ObjectNuggetProceduralSurfaceClear(lit, nullptr, 0);
		Require(luminance(render(lit, placement, normal)) - luminance(render(lit, placement, -normal)) >= 0.10f, "Clearing procedural component lost generated lighting");

		// Independent transformed tangents expose naive direction transforms under nonuniform scale.
		std::cout << "Procedural lighting: public nonuniform placement and buffered reference comparison\n";
		auto rotation = m4x4(v4(0, 1, 0, 0), v4(-1, 0, 0, 0), v4(0, 0, 1, 0), v4::Origin());
		auto scale = m4x4::Identity();
		scale.x.x = 4;
		scale.z.z = 0.25f;
		auto transformed = rotation * scale;
		transformed.pos.z = -20;
		auto expected_normal = Normalise(Cross(transformed * v4(0, 1, 0, 0), transformed * v4(-1, 0, 1, 0)));
		auto wrong_normal = Normalise(transformed * normal);
		auto toward_light = Normalise(rotation * v4(-1, 0, 1, 0));
		Require(Dot3(expected_normal, toward_light) > 0.5f && Dot3(wrong_normal, toward_light) < 0, "Transformed-normal negative control lacks separation");
		auto buffered = [&](v4 reference_normal, bool u32, api::EGeom mask)
		{
			// Bake world-space positions and an independently constructed normal; the reference uses identity placement.
			api::Vertex vertices[3]{};
			for (auto i = 0; i != 3; ++i)
			{
				vertices[i].pos = To<api::Vec4>(transformed * To<v4>(constants.m_positions[i]));
				vertices[i].norm = To<api::Vec4>(reference_normal);
				vertices[i].col = 0xFFCCAA7C;
				vertices[i].tex = {0.25f * i, 0.75f};
			}
			auto desc = nugget;
			desc.m_geom = mask;
			desc.m_v1 = 3;
			desc.m_shaders[0] = {};
			uint16_t i16[] = {0, 1, 2};
			uint32_t i32[] = {0, 1, 2};
			auto buffer_options = options;
			buffer_options.m_vertex_source = api::EVertexSource::Buffer;
			auto object = u32
				? View3D_ObjectCreateU32("BufferedLightingU32", 0xFFFFFFFF, 3, 3, 1, vertices, i32, &desc, buffer_options, GUID{})
				: View3D_ObjectCreate("BufferedLightingU16", 0xFFFFFFFF, 3, 3, 1, vertices, i16, &desc, GUID{});
			Require(object != nullptr, "Buffered reference creation failed");
			fixture.m_objects.push_back(object);
			auto storage = PhysicalVertices(fixture, object);
			Require(storage.size() == 3 && object->m_model->m_vb_view.StrideInBytes == 64, "Buffered vertex layout changed");
			Require(object->m_model->m_nuggets->m_pso.Find<rdr::EPipeState::InputLayout>() == nullptr, "Buffered IA layout was replaced");
			for (auto i = 0; i != 3; ++i)
			{
				Require(All(storage[i].m_vert == To<v4>(vertices[i].pos)) && storage[i].m_diff == Colour(vertices[i].col), "Buffered position/colour copy changed");
				auto copied_normal = AllSet(mask, api::EGeom::Norm) ? reference_normal : v4::Zero();
				auto copied_uv = AllSet(mask, api::EGeom::Tex0) ? v2(vertices[i].tex.x, vertices[i].tex.y) : v2::Zero();
				Require(All(storage[i].m_norm == copied_normal) && All(storage[i].m_tex0 == copied_uv), "Buffered normal/UV copy did not follow its mask");
			}
			promote(object);
			return object;
		};

		// Match packed buffered colour exactly, avoiding colour quantization as a false normal-transform failure.
		auto reference_colour = Colour(0xFFCCAA7C).rgba;
		constants.m_colour = To<api::Vec4>(reference_colour);
		auto matched_shader = ProceduralVertexShader(api::ERenderStep::ForwardRender, compiled::procedural_vertex_forward, sizeof(compiled::procedural_vertex_forward), constants);
		Require(matched_shader != nullptr, "Transformed-normal shader creation failed");
		fixture.m_shaders.push_back(matched_shader);
		nugget.m_shaders[0].m_shader = matched_shader;
		auto transformed_object = create(lit_mask);
		promote(transformed_object);
		auto generated = render(transformed_object, transformed, toward_light);
		for (auto u32 : {false, true})
		{
			auto reference = buffered(expected_normal, u32, lit_mask | api::EGeom::Tex0);
			auto actual = render(reference, m4x4::Identity(), toward_light);
			Require(std::abs(actual.x - generated.x) <= 0.025f && std::abs(actual.y - generated.y) <= 0.025f && std::abs(actual.z - generated.z) <= 0.025f, "Generated transformed normal differs from independent buffered reference");
			auto no_norm = buffered(expected_normal, u32, unlit_mask);
			auto no_norm_front = render(no_norm, m4x4::Identity(), toward_light);
			auto no_norm_back = render(no_norm, m4x4::Identity(), -toward_light);
			Require(Length(no_norm_front - no_norm_back) <= 0.01f, "Buffered no-normal behavior changed");
			std::cout << "PASS public transformed normal versus buffered " << (u32 ? "U32" : "U16") << " reference; RGBdelta=" << Length(actual - generated) << '\n';
		}
		auto wrong = buffered(wrong_normal, true, lit_mask);
		auto wrong_colour = render(wrong, m4x4::Identity(), toward_light);
		Require(luminance(generated) - luminance(wrong_colour) >= 0.10f, "Wrong transformed normal was not rejected by framebuffer comparison");
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS public generated lighting and transformed normal; wrong-normal luminance delta=" << luminance(generated) - luminance(wrong_colour) << "; no DXR/shadow cases executed\n";
	}

	// Verify pooled generator scratch storage can alternate index widths without preserving stale geometry.
	void ModelGeneratorCacheResetTests()
	{
		// Populate a U32 logical-ID buffer with values that cannot be represented by the following U16 allocation.
		auto buffers = pr::rdr12::ModelGenerator::Buffers<pr::rdr12::Vert>{};
		buffers.Reset(1, 3, 0, sizeof(uint32_t));
		auto u32_indices = buffers.m_icont.begin<int>();
		*u32_indices++ = 70000;
		*u32_indices++ = 70001;
		*u32_indices++ = 70002;
		Require(buffers.m_icont.stride() == sizeof(uint32_t) && buffers.m_icont.size() == 3, "Initial U32 generator cache allocation is invalid");
		Require(buffers.m_icont[0] == 70000 && buffers.m_icont[1] == 70001 && buffers.m_icont[2] == 70002, "Initial U32 generator cache contents are invalid");

		// Reset to a different U16 count and prove the next generator owns all newly written contents.
		buffers.Reset(2, 6, 0, sizeof(uint16_t));
		int const u16_expected[] = {3, 1, 4, 1, 5, 0};
		auto u16_indices = buffers.m_icont.begin<int>();
		for (auto value : u16_expected)
			*u16_indices++ = value;

		Require(buffers.m_icont.stride() == sizeof(uint16_t) && buffers.m_icont.size() == std::size(u16_expected), "U16 generator cache allocation is invalid");
		for (auto i = size_t{}; i != std::size(u16_expected); ++i)
			Require(buffers.m_icont[i] == static_cast<uint64_t>(u16_expected[i]), "U16 generator cache retained stale U32 contents");

		// Reset back to U32 with another count and prove later widening also starts with caller-owned data.
		buffers.Reset(3, 4, 0, sizeof(uint32_t));
		int const u32_expected[] = {80003, 70002, 90004, 42};
		auto widened_indices = buffers.m_icont.begin<int>();
		for (auto value : u32_expected)
			*widened_indices++ = value;

		Require(buffers.m_icont.stride() == sizeof(uint32_t) && buffers.m_icont.size() == std::size(u32_expected), "Second U32 generator cache allocation is invalid");
		for (auto i = size_t{}; i != std::size(u32_expected); ++i)
			Require(buffers.m_icont[i] == static_cast<uint64_t>(u32_expected[i]), "Second U32 generator cache contents are invalid");
	}

	// Prove the optional caller buffer is validated, copied once, and readable by the vertex stage after the caller's storage is discarded.
	void ProceduralVertexBufferTests()
	{
		// Build a forward procedural triangle whose colour lives only in the last float4 of a caller buffer.
		Fixture fixture(1);
		std::cout << "Procedural vertex buffer: validation and copied raw SRV\n";
		auto constants = ProceduralVertexConstants{
			.m_positions = {
				api::Vec4{-25, -25, -10, 1},
				api::Vec4{+25, -25, -10, 1},
				api::Vec4{0, +25, -10, 1},
			},
			.m_colour = api::Vec4{0, 1, 0, 1},
		};
		auto buffer = std::vector<api::Vec4>(1024, api::Vec4{0, 1, 0, 1});
		buffer.back() = api::Vec4{1, 0, 1, 1};
		auto valid = api::ShaderOptions{
			.m_struct_size = sizeof(api::ShaderOptions),
			.m_version = api::ShaderOptions::CurrentVersion,
			.m_stage = api::EShaderStage::Vertex,
			.m_bytecode = compiled::procedural_vertex_buffer,
			.m_bytecode_size = sizeof(compiled::procedural_vertex_buffer),
			.m_dbg_name = "ProceduralVertexBuffer",
			.m_procedural = {
				.m_rdr_step = api::ERenderStep::ForwardRender,
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
				.m_buffer = buffer.data(),
				.m_buffer_size = buffer.size() * sizeof(api::Vec4),
			},
		};

		// Inconsistent or unaligned buffer descriptors fail without publishing a shader.
		auto expect_error = [&fixture](api::ShaderOptions options)
		{
			// Require a null handle and the buffer-specific diagnostic.
			fixture.m_errors.clear();
			Require(View3D_ShaderCreate(options) == nullptr, "Malformed procedural buffer descriptor was accepted");
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("buffer size") != std::string::npos, "Malformed procedural buffer reported the wrong error");
		};
		auto bad = valid;
		bad.m_procedural.m_buffer_size = 0;
		expect_error(bad);
		bad = valid;
		bad.m_procedural.m_buffer_size -= 2;
		expect_error(bad);
		bad = valid;
		bad.m_procedural.m_buffer = nullptr;
		expect_error(bad);
		bad = valid;
		bad.m_procedural.m_buffer_size = api::ProceduralBinding::MaxBufferSize + 4;
		expect_error(bad);
		fixture.m_errors.clear();

		// Publish the object, then destroy the caller's copies before the first frame reads them.
		auto shader = View3D_ShaderCreate(valid);
		Require(shader != nullptr, "Procedural buffer shader creation failed");
		fixture.m_shaders.push_back(shader);
		std::fill(buffer.begin(), buffer.end(), api::Vec4{});
		auto placeholder = api::Vertex{};
		uint32_t indices[] = {70000, 70001, 70002};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = api::EGeom::Vert;
		nugget.m_v0 = 0;
		nugget.m_v1 = 1;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_tint = 0xFFFFFFFF;
		nugget.m_shaders[0] = api::Nugget::Shader{shader, api::ERenderStep::ForwardRender, 0};
		auto options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::ProceduralVertexId,
			.m_vcount_logical = 70003,
			.m_bbox = api::BBox{
				.centre = api::Vec4{0, 0, -10, 1},
				.radius = api::Vec4{25, 25, 0, 0},
			},
		};
		auto object = View3D_ObjectCreateU32("ProceduralVertexBuffer", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &nugget, options, GUID{});
		Require(object != nullptr, "Procedural buffer object creation failed");
		fixture.m_objects.push_back(object);
		View3D_WindowAddObject(fixture.m_window, object);
		fixture.CheckErrors();

		// The rendered colour can only come from the renderer-owned copy of the buffer's final element.
		Expect(fixture.Image(), 1, 0, 1);
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS procedural vertex buffer\n";
	}

	// Prove a buffer bound to procedural slot 1 is readable at its register, can be queried back, and that invalid slots are rejected.
	void ProceduralBufferSlotTests()
	{
		// Build a forward procedural triangle with no slot 0 buffer, whose colour lives only in a shared buffer bound at slot 1.
		Fixture fixture(1);
		std::cout << "Procedural buffer slots: slot 1 binding and query\n";
		auto constants = ProceduralVertexConstants{
			.m_positions = {
				api::Vec4{-25, -25, -10, 1},
				api::Vec4{+25, -25, -10, 1},
				api::Vec4{0, +25, -10, 1},
			},
			.m_colour = api::Vec4{0, 1, 0, 1},
		};
		auto shader = View3D_ShaderCreate(api::ShaderOptions{
			.m_struct_size = sizeof(api::ShaderOptions),
			.m_version = api::ShaderOptions::CurrentVersion,
			.m_stage = api::EShaderStage::Vertex,
			.m_bytecode = compiled::procedural_vertex_buffer1,
			.m_bytecode_size = sizeof(compiled::procedural_vertex_buffer1),
			.m_dbg_name = "ProceduralBufferSlot1",
			.m_procedural = {
				.m_rdr_step = api::ERenderStep::ForwardRender,
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			},
		});
		Require(shader != nullptr, "Procedural slot shader creation failed");
		fixture.m_shaders.push_back(shader);
		auto placeholder = api::Vertex{};
		uint32_t indices[] = {70000, 70001, 70002};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = api::EGeom::Vert;
		nugget.m_v0 = 0;
		nugget.m_v1 = 1;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_tint = 0xFFFFFFFF;
		nugget.m_shaders[0] = api::Nugget::Shader{shader, api::ERenderStep::ForwardRender, 0};
		auto options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::ProceduralVertexId,
			.m_vcount_logical = 70003,
			.m_bbox = api::BBox{
				.centre = api::Vec4{0, 0, -10, 1},
				.radius = api::Vec4{25, 25, 0, 0},
			},
		};
		auto object = View3D_ObjectCreateU32("ProceduralBufferSlot1", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &nugget, options, GUID{});
		Require(object != nullptr, "Procedural slot object creation failed");
		fixture.m_objects.push_back(object);

		// Bind a renderer buffer at slot 1, then drop the caller's reference so only the shader keeps it alive.
		auto colour = api::Vec4{1, 0, 1, 1};
		auto buffer = View3D_ProceduralBufferCreate(sizeof(colour), &colour, sizeof(colour), FALSE, "ProceduralBufferSlot1");
		Require(buffer != nullptr, "Procedural slot buffer creation failed");
		View3D_ObjectProceduralBufferSet(object, 1, buffer);
		View3D_ObjectProceduralBufferSet(object, 1, buffer);
		static_cast<IUnknown*>(buffer)->Release();
		Require(View3D_ObjectProceduralBufferGet(object, 1) == buffer, "Procedural slot 1 query returned the wrong buffer");
		Require(View3D_ObjectProceduralBufferGet(object, 0) == nullptr, "Procedural slot 0 query returned a buffer that was never bound");
		fixture.CheckErrors();

		// Slots outside the fixed range are reported and change nothing.
		View3D_ObjectProceduralBufferSet(object, api::ProceduralBinding::BufferSlotCount, nullptr);
		Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("slot") != std::string::npos, "Invalid procedural buffer slot was not reported");
		fixture.m_errors.clear();
		Require(View3D_ObjectProceduralBufferGet(object, -1) == nullptr, "Invalid procedural buffer slot query returned a buffer");
		Require(!fixture.m_errors.empty(), "Invalid procedural buffer slot query was not reported");
		fixture.m_errors.clear();

		// The rendered colour can only come from the slot 1 buffer.
		View3D_WindowAddObject(fixture.m_window, object);
		Expect(fixture.Image(), 1, 0, 1);
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS procedural buffer slots\n";
	}

	// Prove a GPU-generated object receives its optional caller input at t0, and that malformed input descriptors are rejected.
	void GeneratedVertexInputTests()
	{
		// The generator copies positions from the input buffer, so a rendered triangle proves the input reached the dispatch.
		Fixture fixture(1);
		std::cout << "Generated vertex input: validation and t0 binding\n";
		struct GeneratedConstants
		{
			api::Vec4 m_colour;
			uint32_t m_count[4];
		};
		auto constants = GeneratedConstants{
			.m_colour = api::Vec4{0, 1, 0, 1},
			.m_count = {3, 0, 0, 0},
		};
		auto positions = std::vector<api::Vec4>{
			api::Vec4{-25, -25, -10, 1},
			api::Vec4{+25, -25, -10, 1},
			api::Vec4{0, +25, -10, 1},
		};
		uint32_t indices[] = {0, 1, 2};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = api::EGeom::Vert | api::EGeom::Colr;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_tint = 0xFFFFFFFF;
		auto options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::GpuGeneratedBuffer,
			.m_bbox = api::BBox{
				.centre = api::Vec4{0, 0, -10, 1},
				.radius = api::Vec4{25, 25, 0, 0},
			},
			.m_compute_bytecode = compiled::generated_vertex_input,
			.m_compute_bytecode_size = sizeof(compiled::generated_vertex_input),
			.m_constants = &constants,
			.m_constants_size = sizeof(constants),
			.m_thread_group_size_x = 64,
			.m_input = positions.data(),
			.m_input_size = positions.size() * sizeof(api::Vec4),
		};

		// Inconsistent or unaligned input descriptors, and input on non-generated sources, fail without publishing an object.
		auto expect_error = [&fixture, &indices, &nugget](api::ObjectCreateOptions const& object_options, api::Vertex const* verts)
		{
			// Require a null object and the input-specific diagnostic.
			fixture.m_errors.clear();
			auto rejected = View3D_ObjectCreateU32("RejectedGeneratedInput", 0xFFFFFFFF, 3, 3, 1, verts, indices, &nugget, object_options, GUID{});
			Require(rejected == nullptr, "Malformed generator input descriptor was accepted");
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("input") != std::string::npos, "Malformed generator input reported the wrong error");
		};
		auto bad = options;
		bad.m_input_size = 0;
		expect_error(bad, nullptr);
		bad = options;
		bad.m_input = nullptr;
		expect_error(bad, nullptr);
		bad = options;
		bad.m_input_size -= 2;
		expect_error(bad, nullptr);
		api::Vertex buffered[3] = {};
		bad = options;
		bad.m_vertex_source = api::EVertexSource::Buffer;
		expect_error(bad, buffered);
		fixture.m_errors.clear();

		// Publish the object, then clear the caller's input to show the dispatch already consumed it.
		auto object = View3D_ObjectCreateU32("GeneratedVertexInput", 0xFFFFFFFF, 3, 3, 1, nullptr, indices, &nugget, options, GUID{});
		Require(object != nullptr, "Generated input object creation failed");
		fixture.m_objects.push_back(object);
		std::fill(positions.begin(), positions.end(), api::Vec4{});
		View3D_WindowAddObject(fixture.m_window, object);
		fixture.CheckErrors();

		// The triangle covers the image centre only if the generator read the input positions.
		Expect(fixture.Image(), 0, 1, 0);
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS generated vertex input\n";
	}

	// Exercise the public U32 logical-ID domain, copied shader inputs, raster output, and stock RayCast geometry stage.
	void ProceduralVertexAbiTests()
	{
		// Prove alternating index widths discard pooled scratch contents before any renderer resources are created.
		ModelGeneratorCacheResetTests();

		// Create three immutable stage recipes from caller-owned buffers that are discarded immediately after publication.
		Fixture fixture(1);
		auto constants = ProceduralVertexConstants{
			.m_positions = {
				api::Vec4{-25, -25, -10, 1},
				api::Vec4{+25, -25, -10, 1},
				api::Vec4{0, +25, -10, 1},
			},
			.m_colour = api::Vec4{0, 1, 0, 1},
		};
		auto forward_bytecode = std::vector<unsigned char>(std::begin(compiled::procedural_vertex_forward), std::end(compiled::procedural_vertex_forward));
		auto raycast_bytecode = std::vector<unsigned char>(std::begin(compiled::procedural_vertex_raycast), std::end(compiled::procedural_vertex_raycast));
		auto shadow_bytecode = std::vector<unsigned char>(std::begin(compiled::procedural_vertex_shadow), std::end(compiled::procedural_vertex_shadow));

		// Malformed descriptors fail through the public error callback without publishing partial shader state.
		auto expect_shader_error = [&fixture](api::ShaderOptions options, std::string_view expected)
		{
			// Require both a null handle and a diagnostic owned by the rejected descriptor.
			fixture.m_errors.clear();
			Require(View3D_ShaderCreate(options) == nullptr, "Malformed procedural shader descriptor was accepted");
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find(expected) != std::string::npos, "Malformed shader descriptor reported the wrong error");
		};
		auto valid_shader_options = api::ShaderOptions{
			.m_struct_size = sizeof(api::ShaderOptions),
			.m_version = api::ShaderOptions::CurrentVersion,
			.m_stage = api::EShaderStage::Vertex,
			.m_bytecode = forward_bytecode.data(),
			.m_bytecode_size = forward_bytecode.size(),
			.m_dbg_name = "ProceduralVertexAbiValidation",
			.m_procedural = {
				.m_rdr_step = api::ERenderStep::ForwardRender,
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			},
		};
		auto bad_shader_options = valid_shader_options;
		bad_shader_options.m_struct_size -= 1;
		expect_shader_error(bad_shader_options, "structure size");
		bad_shader_options = valid_shader_options;
		bad_shader_options.m_version = 1;
		expect_shader_error(bad_shader_options, "version");
		bad_shader_options = valid_shader_options;
		bad_shader_options.m_procedural.m_constants_size -= 16;
		expect_shader_error(bad_shader_options, "1024");
		bad_shader_options = valid_shader_options;
		bad_shader_options.m_bytecode_size = 3;
		expect_shader_error(bad_shader_options, "bytecode");

		// Unsupported stages must not require bytecode or a procedural binding that they do not implement.
		for (auto stage : {api::EShaderStage::Geometry, api::EShaderStage::Hull, api::EShaderStage::Domain, api::EShaderStage::Compute})
		{
			// Allocate the complete current descriptor; null buffers are deliberately irrelevant for this stage.
			auto unsupported = api::ShaderOptions{
				.m_struct_size = sizeof(api::ShaderOptions),
				.m_version = api::ShaderOptions::CurrentVersion,
				.m_stage = stage,
			};
			expect_shader_error(unsupported, "stage is not supported");
		}
		bad_shader_options = {};
		bad_shader_options.m_struct_size = sizeof(api::ShaderOptions);
		bad_shader_options.m_version = api::ShaderOptions::CurrentVersion;
		bad_shader_options.m_stage = static_cast<api::EShaderStage>(-1);
		expect_shader_error(bad_shader_options, "Invalid shader stage");
		fixture.m_errors.clear();

		// Valid descriptors retain private copies and bind only to their declared render steps.
		auto forward = ProceduralVertexShader(api::ERenderStep::ForwardRender, forward_bytecode.data(), forward_bytecode.size(), constants);
		auto raycast = ProceduralVertexShader(api::ERenderStep::RayCast, raycast_bytecode.data(), raycast_bytecode.size(), constants);
		auto shadow = ProceduralVertexShader(api::ERenderStep::ShadowMap, shadow_bytecode.data(), shadow_bytecode.size(), constants);
		Require(forward != nullptr && raycast != nullptr && shadow != nullptr, "Procedural vertex shader creation failed");
		fixture.CheckErrors();

		// Publish sparse IDs above 65K while retaining one physical placeholder vertex and explicit generated bounds.
		auto placeholder = api::Vertex{};
		uint32_t indices[] = {70000, 70001, 70002};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = api::EGeom::Vert;
		nugget.m_v0 = 0;
		nugget.m_v1 = 1;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_tint = 0xFFFFFFFF;
		nugget.m_shaders[0] = api::Nugget::Shader{forward, api::ERenderStep::ForwardRender, 0};
		nugget.m_shaders[1] = api::Nugget::Shader{raycast, api::ERenderStep::RayCast, 0};
		nugget.m_shaders[2] = api::Nugget::Shader{shadow, api::ERenderStep::ShadowMap, 0};
		auto options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::ProceduralVertexId,
			.m_vcount_logical = 70003,
			.m_bbox = api::BBox{
				.centre = api::Vec4{0, 0, -10, 1},
				.radius = api::Vec4{25, 25, 0, 0},
			},
		};

		// Malformed geometry domains, physical ranges, bounds, and stage bindings fail before model publication.
		auto expect_object_error = [&fixture, &placeholder, &indices, &nugget](api::ObjectCreateOptions const& object_options, api::Nugget const& object_nugget, std::string_view expected)
		{
			// Require both a null object and the diagnostic for the violated creation contract.
			fixture.m_errors.clear();
			auto rejected = View3D_ObjectCreateU32("RejectedProceduralVertexAbi", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &object_nugget, object_options, GUID{});
			Require(rejected == nullptr, "Malformed procedural object descriptor was accepted");
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find(expected) != std::string::npos, "Malformed object descriptor reported the wrong error");
		};
		auto bad_options = options;
		bad_options.m_struct_size -= 1;
		expect_object_error(bad_options, nugget, "structure size");
		bad_options = options;
		bad_options.m_vcount_logical = 70002;
		expect_object_error(bad_options, nugget, "vertex-ID domain");
		bad_options = options;
		bad_options.m_bbox.radius.x = -1;
		expect_object_error(bad_options, nugget, "bounding box");
		auto bad_nugget = nugget;
		bad_nugget.m_v0 = 0;
		bad_nugget.m_v1 = 2;
		expect_object_error(options, bad_nugget, "physical vertex buffer");
		bad_nugget = nugget;
		bad_nugget.m_geom = api::EGeom::Norm;
		expect_object_error(options, bad_nugget, "supported generated attributes");
		bad_nugget.m_geom = static_cast<api::EGeom>(static_cast<int>(api::EGeom::Vert) | (1 << 8));
		expect_object_error(options, bad_nugget, "supported generated attributes");
		bad_nugget = nugget;
		bad_nugget.m_shaders[0].m_rdr_step = api::ERenderStep::RayCast;
		expect_object_error(options, bad_nugget, "render-step contract");
		bad_options = options;
		bad_options.m_vertex_source = api::EVertexSource::Buffer;
		expect_object_error(bad_options, nugget, "vertex-ID domain");

		// Procedural geometry must use its U32 logical-ID buffer rather than falling through to a non-indexed placeholder draw.
		fixture.m_errors.clear();
		auto non_indexed = View3D_ObjectCreateU32("RejectedNonIndexedProceduralVertexAbi", 0xFFFFFFFF, 1, 0, 1, &placeholder, nullptr, &nugget, options, GUID{});
		Require(non_indexed == nullptr, "Non-indexed procedural object descriptor was accepted");
		Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("indexed geometry") != std::string::npos, "Non-indexed procedural object reported the wrong error");
		fixture.m_errors.clear();

		// Publish the valid procedural object after all rejected inputs leave the context unchanged.
		auto object = View3D_ObjectCreateU32("ProceduralVertexAbi", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &nugget, options, GUID{});
		Require(object != nullptr, "Procedural U32 object creation failed");
		fixture.m_objects.push_back(object);
		View3D_WindowAddObject(fixture.m_window, object);
		fixture.CheckErrors();

		// Destroying a model before its retained caller shader handle must release only the model-owned reference.
		auto retained_shader = ProceduralVertexShader(api::ERenderStep::ForwardRender, forward_bytecode.data(), forward_bytecode.size(), constants);
		Require(retained_shader != nullptr, "Retained shader creation failed");
		auto retained_nugget = nugget;
		retained_nugget.m_shaders[0] = api::Nugget::Shader{retained_shader, api::ERenderStep::ForwardRender, 0};
		retained_nugget.m_shaders[1] = {};
		auto retained_object = View3D_ObjectCreateU32("ProceduralReleaseOrder", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &retained_nugget, options, GUID{});
		Require(retained_object != nullptr, "Release-order object creation failed");
		View3D_ObjectDelete(retained_object);
		View3D_ShaderRelease(retained_shader);
		fixture.CheckErrors();

		// Release every caller handle and mutate the source buffers before the renderer first consumes the recipes.
		View3D_ShaderRelease(forward);
		View3D_ShaderRelease(raycast);
		View3D_ShaderRelease(shadow);
		std::fill(forward_bytecode.begin(), forward_bytecode.end(), static_cast<unsigned char>(0));
		std::fill(raycast_bytecode.begin(), raycast_bytecode.end(), static_cast<unsigned char>(0));
		std::fill(shadow_bytecode.begin(), shadow_bytecode.end(), static_cast<unsigned char>(0));
		constants = {};

		// Forward must render the generated triangle rather than the physical placeholder vertex.
		auto image = fixture.Image();
		Expect(image, 0, 1, 0);

		// Public procedural-surface promotion must retain all three vertex overlays while selecting the stock PBR pixel shader.
		auto surface = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Soil);
		surface.m_colour0 = surface.m_colour1 = surface.m_colour2 = surface.m_colour3 = 0xFF00FF00;
		surface.m_normal_strength = 0;
		View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
		fixture.CheckErrors();
		auto round_trip = api::ProceduralSurface{};
		auto has_surface = View3D_ObjectNuggetProceduralSurfaceGet(object, round_trip, nullptr, 0);
		fixture.CheckErrors();
		Require(has_surface, "Procedural U32 object did not promote to the stock PBR material");
		auto ambient_light = SceneLightingGet(fixture.m_window);
		ambient_light.m_ambient = 0xFFFFFFFF;
		ambient_light.m_diffuse = 0xFF000000;
		ambient_light.m_specular = 0xFF000000;
		ambient_light.m_intensity = 1;
		ambient_light.m_on = TRUE;
		SceneLightingSet(fixture.m_window, ambient_light);
		Expect(fixture.Image(), 0, 1, 0);

		// Rejected procedural parameters must leave the existing enabled component unchanged.
		auto invalid_surface = surface;
		invalid_surface.m_feature_scale = 0;
		View3D_ObjectNuggetProceduralSurfaceSet(object, invalid_surface, nullptr, 0);
		Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("feature scale") != std::string::npos, "Invalid procedural component did not report its contract error");
		fixture.m_errors.clear();
		has_surface = View3D_ObjectNuggetProceduralSurfaceGet(object, round_trip, nullptr, 0);
		fixture.CheckErrors();
		Require(has_surface && round_trip.m_feature_scale == surface.m_feature_scale, "Invalid procedural component replaced the active state");

		// Clearing procedural evaluation leaves the ordinary PBR colour and vertex overlay intact, and can be repeated.
		View3D_ObjectNuggetProceduralSurfaceClear(object, nullptr, 0);
		fixture.CheckErrors();
		View3D_ObjectNuggetProceduralSurfaceClear(object, nullptr, 0);
		fixture.CheckErrors();
		has_surface = View3D_ObjectNuggetProceduralSurfaceGet(object, round_trip, nullptr, 0);
		fixture.CheckErrors();
		Require(!has_surface, "Cleared procedural component remained visible");
		Expect(fixture.Image(), 0, 1, 0);
		View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
		fixture.CheckErrors();
		has_surface = View3D_ObjectNuggetProceduralSurfaceGet(object, round_trip, nullptr, 0);
		fixture.CheckErrors();
		Require(has_surface, "Cleared procedural component could not be re-enabled");
		Expect(fixture.Image(), 0, 1, 0);

		// RayCast must use the same generated world-space triangle while retaining the stock topology geometry shader.
		auto ray = api::HitTestRay{};
		ray.m_ws_origin = api::Vec4{0, 0, 0, 1};
		ray.m_ws_direction = api::Vec4{0, 0, -1, 0};
		auto hit = api::HitTestResult{};
		View3D_WindowHitTestObjects(fixture.m_window, &ray, &hit, 1, &object, 1);
		fixture.CheckErrors();
		Require(hit.m_obj == object && std::abs(hit.m_distance - 10.0f) < 0.01f, "Procedural Forward and RayCast geometry disagree");

		// ShadowMap must use the supplied procedural VS while retaining stock depth and material handling.
		auto receiver = fixture.Quad(20, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto light = SceneLightingGet(fixture.m_window);
		light.m_type = api::ELight::Directional;
		light.m_direction = api::Vec4{0.70710678f, 0, -0.70710678f, 0};
		light.m_ambient = 0xFF202020;
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 1;
		light.m_cast_shadow = 1.0f;
		light.m_on = TRUE;
		SceneLightingSet(fixture.m_window, light);
		auto with_shadow = fixture.Image();
		View3D_ObjectFlagsSet(object, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
		auto without_shadow = fixture.Image();
		Require(ImageDifference(with_shadow, without_shadow) > 50, "Procedural ShadowMap VS did not cast a raster shadow");
		View3D_ObjectFlagsSet(object, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
		View3D_WindowRemoveObject(fixture.m_window, receiver);
		fixture.CheckErrors();

		// DXR must reject placeholder geometry explicitly rather than tracing it or silently omitting it.
		if (View3D_WindowRayTracingInfoGet(fixture.m_window).m_available)
		{
			// Removing the receiver clears all scene drawlists; rebuild the remaining procedural instance before testing resident-source rejection.
			fixture.Image();
			auto& scene = fixture.m_window->m_scene;
			Require(std::find(scene.m_instances.begin(), scene.m_instances.end(), &object->m_base) != scene.m_instances.end(), "Resident-source fixture has not admitted its procedural instance");
			Require(object->m_model != nullptr && object->m_model->m_vertex_source == pr::rdr12::EVertexSource::ProceduralVertexId, "Resident-source fixture does not contain procedural geometry");

			// Direct pipeline replacement must reject before deleting any of the existing scene's render steps.
			auto previous_steps = std::vector<pr::rdr12::RenderStep const*>{};
			for (auto const& step : scene.m_render_steps)
				previous_steps.push_back(step.get());

			auto requested_steps = std::array{pr::rdr12::ERenderStep::RenderForward, pr::rdr12::ERenderStep::RayTracing};
			auto rejected_steps = false;
			try
			{
				// Exercise the native state-change boundary not exposed by the DLL's per-window toggle.
				scene.SetRenderSteps(requested_steps);
			}
			catch (std::runtime_error const& error)
			{
				// Only the documented source incompatibility is an expected rejection.
				Require(std::string_view(error.what()).find("procedural vertex-ID") != std::string_view::npos, "Render-step replacement reported the wrong error");
				rejected_steps = true;
			}
			Require(rejected_steps, "Render-step replacement accepted procedural DXR geometry");
			Require(scene.m_render_steps.size() == previous_steps.size(), "Rejected render-step replacement changed the pipeline size");
			for (auto i = size_t{}; i != previous_steps.size(); ++i)
				Require(scene.m_render_steps[i].get() == previous_steps[i], "Rejected render-step replacement destroyed an existing pass");

			// Enabling DXR with resident procedural geometry must fail without replacing the raster pipeline.
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("procedural vertex-ID") != std::string::npos, "DXR enable did not reject resident procedural geometry");
			fixture.m_errors.clear();
			Require(!View3D_WindowRayTracingEnabledGet(fixture.m_window), "Failed DXR enable changed the active pipeline");
			SceneLightingSet(fixture.m_window, ambient_light);
			Expect(fixture.Image(), 0, 1, 0);

			// Rebuild the empty scene before enabling DXR, then add the model to exercise late admission on the same window.
			View3D_WindowRemoveObject(fixture.m_window, object);
			fixture.Image();
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			fixture.CheckErrors();
			Require(View3D_WindowRayTracingEnabledGet(fixture.m_window), "DXR did not enable on an empty scene");
			View3D_WindowAddObject(fixture.m_window, object);
			View3D_WindowRender(fixture.m_window);
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("procedural vertex-ID") != std::string::npos, "DXR did not report the procedural geometry boundary");
			Require(std::find(scene.m_instances.begin(), scene.m_instances.end(), &object->m_base) == scene.m_instances.end(), "Rejected procedural instance was published to the scene");
			fixture.m_errors.clear();
			View3D_WindowRayTracingEnabledSet(fixture.m_window, FALSE);
			Expect(fixture.Image(), 0, 1, 0);

			// The same model may remain raster-visible in one window while a nested instance is rejected by another scene's DXR admission.
			fixture.CheckDebugLayer();
			{
				// Keep secondary-window resources alive until its own recovery render has completed.
				Fixture shared_fixture(1);
				auto group = View3D_ObjectCreateLdrA("*Group NestedProcedural {*Group Child {}}", FALSE, nullptr, nullptr);
				Require(group != nullptr, "Nested procedural fixture group creation failed");
				shared_fixture.m_objects.push_back(group);
				auto child = View3D_ObjectGetChildByIndex(group, 0);
				Require(child != nullptr, "Nested procedural fixture child is missing");
				Require(child->m_model == nullptr, "Nested child must not own a model before sharing");

				// Only add a reference here; model creation and final release remain inside the DLL.
				child->m_model = object->m_model;
				SceneLightingSet(shared_fixture.m_window, ambient_light);
				View3D_WindowRayTracingEnabledSet(shared_fixture.m_window, TRUE);
				shared_fixture.CheckErrors();
				View3D_WindowAddObject(shared_fixture.m_window, group);
				View3D_WindowRender(shared_fixture.m_window);
				Require(!shared_fixture.m_errors.empty() && shared_fixture.m_errors.back().find("procedural vertex-ID") != std::string::npos, "Nested shared procedural model bypassed DXR admission");
				shared_fixture.m_errors.clear();
				View3D_WindowRayTracingEnabledSet(shared_fixture.m_window, FALSE);
				Expect(shared_fixture.Image(), 0, 1, 0);
				Expect(fixture.Image(), 0, 1, 0);
				shared_fixture.CheckDebugLayer();
			}
		}

		// The shared U32 path must render on the same window after the expected DXR rejection, proving frame recording was never opened.
		fixture.Clear();
		api::Vertex buffered_verts[] = {
			{api::Vec4{-25, -25, -10, 1}, api::Vec4{0, 0, 1, 0}, {}, 0xFF00FF00, 0},
			{api::Vec4{+25, -25, -10, 1}, api::Vec4{0, 0, 1, 0}, {}, 0xFF00FF00, 0},
			{api::Vec4{0, +25, -10, 1}, api::Vec4{0, 0, 1, 0}, {}, 0xFF00FF00, 0},
		};
		uint32_t buffered_indices[] = {0, 1, 2};
		auto buffered_nugget = api::Nugget{};
		buffered_nugget.m_topo = api::ETopo::TriList;
		buffered_nugget.m_geom = static_cast<api::EGeom>(static_cast<int>(api::EGeom::Vert) | static_cast<int>(api::EGeom::Colr));
		buffered_nugget.m_cull_mode = api::ECullMode::None;
		buffered_nugget.m_tint = 0xFFFFFFFF;
		auto buffered_options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::Buffer,
		};
		auto buffered_object = View3D_ObjectCreateU32("BufferedU32", 0xFFFFFFFF, 3, 3, 1, buffered_verts, buffered_indices, &buffered_nugget, buffered_options, GUID{});
		Require(buffered_object != nullptr, "Ordinary U32 buffered object creation failed");
		fixture.m_objects.push_back(buffered_object);
		View3D_WindowAddObject(fixture.m_window, buffered_object);
		Expect(fixture.Image(), 0, 1, 0);
		fixture.CheckDebugLayer();
		std::cout << "PASS procedural descriptor rejection, U32 logical IDs, copied recipes, release ordering, Forward, RayCast, ShadowMap, DXR boundary, and buffered U32\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_ProceduralVertexAbi, Extended | Serial)
	{
		render_tests::ProceduralVertexAbiTests();
		render_tests::ProceduralVertexBufferTests();
		render_tests::ProceduralBufferSlotTests();
	}
	PRUnitTest(View3d12_GeneratedVertexInput, Extended | Serial)
	{
		render_tests::GeneratedVertexInputTests();
	}
	PRUnitTest(View3d12_ProceduralVertexLighting, Extended | Serial)
	{
		render_tests::ProceduralVertexLightingTests();
	}
}
