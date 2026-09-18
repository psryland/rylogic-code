#include "forward.h"

namespace fade_tests
{
	namespace api = pr::view3d;
	namespace ui = pr::view3d::ui;
	using Microsoft::WRL::ComPtr;
	constexpr int ImageSize = 128;

	// Fail the fixture without relying on debug-only assertions.
	void Require(bool condition, char const* message)
	{
		if (!condition)
			throw std::runtime_error(message);
	}

	// Read the packed blend value through the current single-value public API.
	api::Colour View3D_ObjectColourBlendColourGet(api::Object object, char const* name)
	{
		return ::View3D_ObjectColourBlendGet(object, name);
	}

	// Decode the packed alpha byte as the public linear blend weight.
	float View3D_ObjectColourBlendAmountGet(api::Object object, char const* name)
	{
		return ((::View3D_ObjectColourBlendGet(object, name) >> 24) & 0xFFU) / 255.0f;
	}

	// Reject a failed D3D operation at its ownership boundary.
	void Check(HRESULT result)
	{
		if (FAILED(result))
			throw std::runtime_error("D3D operation failed: " + std::to_string(result));
	}

	// Collect DLL failures without throwing through native callback frames.
	void __stdcall ReportError(void* context, char const* message, char const*, int, int64_t)
	{
		static_cast<std::vector<std::string>*>(context)->emplace_back(message);
	}

	// Paint a fixed marker through the same final-overlay host hook used by retained screen UI.
	ui::EHostStatus __stdcall Overlay(void*, ui::Pass const* pass)
	{
		switch (pass->m_pass)
		{
			case ui::EPass::FinalOverlay:
			{
				float colour[] = {0, 1, 0, 1};
				D3D12_RECT rect{4, 4, 20, 20};
				pass->m_command_list->ClearRenderTargetView(pass->m_rtv, colour, 1, &rect);
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

	// Convert framebuffer bytes back to the linear colour space used by alpha blending.
	float Linear(unsigned char value)
	{
		auto channel = value / 255.0f;
		return channel <= 0.04045f ? channel / 12.92f : std::pow((channel + 0.055f) / 1.055f, 2.4f);
	}

	// Own a bounded invisible window, its renderer context, and all fixture geometry.
	struct Fixture
	{
		std::vector<std::string> m_errors;
		HWND m_hwnd = nullptr;
		api::DllHandle m_context = nullptr;
		api::Window m_window = nullptr;
		std::vector<api::Object> m_objects;
		std::vector<api::Shader> m_shaders;
		ComPtr<ID3D12Device> m_device;
		ComPtr<ID3D12InfoQueue> m_info;

		// Set up a deterministic orthographic camera and the existing alpha/MSAA configuration.
		explicit Fixture(int samples, api::Colour background = 0xFF000000)
		{
			m_hwnd = CreateWindowExW(0, L"STATIC", L"Far clip fade test", WS_POPUP, 0, 0, ImageSize, ImageSize, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
			Require(m_hwnd != nullptr, "CreateWindowEx failed");
			m_context = View3D_Initialise({&m_errors, ReportError});
			Require(m_context != nullptr, "View3D_Initialise failed");
			m_window = View3D_WindowCreate(m_hwnd, api::WindowOptions().error_cb({&m_errors, ReportError}).back_colour(background).multisamp(samples).name("FarClipFadeTests"));
			Require(m_window != nullptr, "View3D_WindowCreate failed");
			m_device.Attach(static_cast<ID3D12Device*>(View3D_DeviceLeaseAcquire(m_context)));
			m_device.As(&m_info);
			std::cout << "D3D12 debug validation " << (m_info ? "active" : "unavailable") << std::endl;
			if (m_info)
				m_info->ClearStoredMessages();

			// Camera depth is absolute, not a percentage of focus distance.
			api::Mat4x4 identity{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
			View3D_CameraToWorldSet(m_window, identity);
			View3D_CameraOrthographicSet(m_window, TRUE);
			View3D_CameraViewRectAtDistanceSet(m_window, api::Vec2{100,100}, 1.0f);
			View3D_CameraClipPlanesSet(m_window, 1, 100, api::EClipPlanes::Both);
			View3D_WindowViewportSet(m_window, api::Viewport{0,0,ImageSize,ImageSize,0,1,ImageSize,ImageSize});
			CheckErrors();
		}

		// Release fixture-owned resources only, without touching other processes or applications.
		~Fixture()
		{
			if (m_window)
			{
				View3D_WindowGSyncWait(m_window);
				View3D_WindowDestroy(m_window);
			}
			for (auto object : m_objects)
				View3D_ObjectDelete(object);

			// Shader wrappers outlive every object that references their fixture-owned bytecode.
			for (auto shader : m_shaders)
				View3D_ShaderRelease(shader);

			m_info.Reset();
			m_device.Reset();
			if (m_context)
				View3D_Shutdown(m_context);

			if (m_hwnd)
				DestroyWindow(m_hwnd);
		}

		// Surface unexpected DLL errors immediately rather than accepting a stale frame.
		void CheckErrors()
		{
			if (m_errors.empty())
				return;

			throw std::runtime_error(m_errors.front());
		}

		// Add an unlit two-sided quad at a chosen forward depth.
		api::Object Quad(float depth, unsigned colour, float half_width = 45, api::Shader shader = nullptr, float right_depth = 0, unsigned vertex_colour = 0xFFFFFFFF, bool textured = false, bool normals = false, api::EFillMode fill_mode = api::EFillMode::Default)
		{
			// Distinct vertex and material colours let surface tests distinguish an override from another multiplicative tint.
			auto normal = normals ? api::Vec4{0, 0, 1, 0} : api::Vec4{};
			api::Vertex verts[] =
			{
				{{-half_width,-45,-depth,1}, normal, {}, vertex_colour, 0},
				{{+half_width,-45,-(right_depth != 0 ? right_depth : depth),1}, normal, {}, vertex_colour, 0},
				{{+half_width,+45,-(right_depth != 0 ? right_depth : depth),1}, normal, {}, vertex_colour, 0},
				{{-half_width,+45,-depth,1}, normal, {}, vertex_colour, 0},
			};
			UINT16 indices[] = {0,1,2,0,2,3};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::TriList;
			nugget.m_geom = api::EGeom::Vert;
			if (vertex_colour != 0xFFFFFFFF)
				nugget.m_geom = static_cast<api::EGeom>(static_cast<int>(nugget.m_geom) | static_cast<int>(api::EGeom::Colr));

			if (textured)
				nugget.m_geom = static_cast<api::EGeom>(static_cast<int>(nugget.m_geom) | static_cast<int>(api::EGeom::Tex0));
			if (normals)
				nugget.m_geom = static_cast<api::EGeom>(static_cast<int>(nugget.m_geom) | static_cast<int>(api::EGeom::Norm));

			nugget.m_cull_mode = api::ECullMode::None;
			nugget.m_fill_mode = fill_mode;
			nugget.m_tint = colour;
			if (shader != nullptr)
				nugget.m_shaders[0] = api::Nugget::Shader{shader, api::ERenderStep::ForwardRender, 0};

			auto object = View3D_ObjectCreate("FadeQuad", 0xFFFFFFFF, 4, 6, 1, verts, indices, &nugget, GUID{});
			Require(object != nullptr, "Quad creation failed");
			m_objects.push_back(object);
			View3D_WindowAddObject(m_window, object);
			CheckErrors();
			return object;
		}

		// Add one non-triangle primitive to prove SolidWire preserves ordinary line rendering.
		api::Object Line(float depth, unsigned colour)
		{
			// Cross the framebuffer centre so one stable pixel witnesses the line.
			api::Vertex verts[] =
			{
				{{-45,0,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{+45,0,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
			};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::LineList;
			nugget.m_geom = api::EGeom::Vert;
			nugget.m_tint = colour;
			auto object = View3D_ObjectCreate("SolidWireLine", 0xFFFFFFFF, 2, 0, 1, verts, nullptr, &nugget, GUID{});
			Require(object != nullptr, "Line creation failed");
			m_objects.push_back(object);
			View3D_WindowAddObject(m_window, object);
			CheckErrors();
			return object;
		}

		// Add one unindexed triangle to cover the direct vertex-draw form of SolidWire.
		api::Object Triangle(float depth, unsigned colour)
		{
			// Cover the framebuffer centre with one broad triangle and no index buffer.
			api::Vertex verts[] =
			{
				{{-45,-45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{+45,-45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{0,+45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
			};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::TriList;
			nugget.m_geom = api::EGeom::Vert;
			nugget.m_cull_mode = api::ECullMode::None;
			nugget.m_tint = colour;
			auto object = View3D_ObjectCreate("SolidWireTriangle", 0xFFFFFFFF, 3, 0, 1, verts, nullptr, &nugget, GUID{});
			Require(object != nullptr, "Triangle creation failed");
			m_objects.push_back(object);
			View3D_WindowAddObject(m_window, object);
			CheckErrors();
			return object;
		}

		// Add one unindexed quad whose shared diagonal gives the wire overlay a stable centre-pixel witness.
		api::Object UnindexedQuad(float depth, unsigned colour)
		{
			// Duplicate the shared vertices so the draw uses the direct non-indexed path.
			api::Vertex verts[] =
			{
				{{-45,-45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{+45,-45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{+45,+45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{-45,-45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{+45,+45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{-45,+45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
			};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::TriList;
			nugget.m_geom = api::EGeom::Vert;
			nugget.m_cull_mode = api::ECullMode::None;
			nugget.m_tint = colour;
			auto object = View3D_ObjectCreate("SolidWireUnindexedQuad", 0xFFFFFFFF, _countof(verts), 0, 1, verts, nullptr, &nugget, GUID{});
			Require(object != nullptr, "Unindexed quad creation failed");
			m_objects.push_back(object);
			View3D_WindowAddObject(m_window, object);
			CheckErrors();
			return object;
		}

		// Add one independently owned rectangular mesh for cross-object coordinate continuity tests.
		api::Object QuadRect(float x0, float x1, float depth)
		{
			// Supply normals but no UV or tangent data so the procedural material owns all surface channels.
			auto normal = api::Vec4{0, 0, 1, 0};
			api::Vertex verts[] =
			{
				{{x0,-45,-depth,1}, normal, {}, 0xFFFFFFFF, 0},
				{{x1,-45,-depth,1}, normal, {}, 0xFFFFFFFF, 0},
				{{x1,+45,-depth,1}, normal, {}, 0xFFFFFFFF, 0},
				{{x0,+45,-depth,1}, normal, {}, 0xFFFFFFFF, 0},
			};
			UINT16 indices[] = {0,1,2,0,2,3};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::TriList;
			nugget.m_geom = static_cast<api::EGeom>(static_cast<int>(api::EGeom::Vert) | static_cast<int>(api::EGeom::Norm));
			nugget.m_cull_mode = api::ECullMode::None;
			nugget.m_tint = 0xFFFFFFFF;
			auto object = View3D_ObjectCreate("ProceduralQuad", 0xFFFFFFFF, 4, 6, 1, verts, indices, &nugget, GUID{});
			Require(object != nullptr, "Procedural rectangle creation failed");
			m_objects.push_back(object);
			View3D_WindowAddObject(m_window, object);
			CheckErrors();
			return object;
		}

		// Replace the active scene without changing ownership of any created object.
		void Clear()
		{
			View3D_WindowRemoveAllObjects(m_window);
		}

		// Apply the same opt-in API used by managed consumers.
		void Fade(bool enabled)
		{
			Require(View3D_FarClipFadePropertiesSet(m_window, api::FarClipFadeProps{enabled, 0.9f, 0.99f}) != FALSE, "Fade settings rejected");
			CheckErrors();
		}

		// Attach a fixed retained-overlay host marker for pixel-invariance checks.
		void AttachOverlay()
		{
			auto attach = reinterpret_cast<ui::AttachFn>(GetProcAddress(GetModuleHandleW(L"view3d-12.dll"), ui::AttachExport));
			Require(attach != nullptr, "UI bridge missing");
			ui::Provider provider{{sizeof(ui::Provider), ui::HostStructVersion}, this, Overlay, nullptr};
			Require(attach(m_window, &provider) == ui::EHostStatus::Success, "UI bridge attach failed");
		}

		// Reuse an owned overlay wrapper while supplying independent fixture vertex or pixel bytecode.
		api::Shader CustomShader(bool vertex_only)
		{
			auto shader = View3D_ShaderCreateStock(api::EStockShader::PointSpritesGS, "");
			Require(shader != nullptr, "Shader wrapper creation failed");
			m_shaders.push_back(shader);
			shader->m_code = {};
			if (vertex_only)
				shader->m_code.VS = pr::compute::ByteCode{compiled::fade_vertex};
			else
				shader->m_code.PS = pr::compute::ByteCode{compiled::unsupported_pixel};

			return shader;
		}

		// Supply a different root signature to prove that incompatible resource bindings fail explicitly.
		api::Shader UnsupportedRootSignature()
		{
			auto shader = CustomShader(true);
			D3D12_ROOT_SIGNATURE_DESC desc{};
			ComPtr<ID3DBlob> blob;
			ComPtr<ID3DBlob> errors;
			Check(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &errors));
			Check(m_device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(shader->m_signature.address_of())));
			return shader;
		}

		// Read the completed final image on a separate queue after synchronizing the renderer.
		std::vector<unsigned char> Image()
		{
			View3D_WindowRender(m_window);
			View3D_WindowGSyncWait(m_window);
			CheckErrors();
			auto frame = View3D_WindowFrameOutputGet(m_window);
			Require(frame.m_render_target != nullptr, "No final frame");
			auto desc = frame.m_render_target->GetDesc();
			auto bgra = false;
			switch (desc.Format)
			{
				case DXGI_FORMAT_B8G8R8A8_UNORM:
				case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB: { bgra = true; break; }
				case DXGI_FORMAT_R8G8B8A8_UNORM:
				case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: { break; }
				default: { throw std::runtime_error("Unexpected frame format " + std::to_string(desc.Format)); }
			}

			// Copy into one readback allocation with the device's required row pitch.
			D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
			UINT64 size{};
			m_device->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &size);
			D3D12_HEAP_PROPERTIES heap{};
			heap.Type = D3D12_HEAP_TYPE_READBACK;
			D3D12_RESOURCE_DESC buffer{};
			buffer.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
			buffer.Width = size;
			buffer.Height = buffer.DepthOrArraySize = buffer.MipLevels = 1;
			buffer.SampleDesc.Count = 1;
			buffer.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
			ComPtr<ID3D12Resource> readback;
			Check(m_device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &buffer, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&readback)));
			ComPtr<ID3D12CommandQueue> queue;
			D3D12_COMMAND_QUEUE_DESC queue_desc{};
			Check(m_device->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&queue)));
			ComPtr<ID3D12CommandAllocator> allocator;
			Check(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
			ComPtr<ID3D12GraphicsCommandList> list;
			Check(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(&list)));
			D3D12_RESOURCE_BARRIER barrier{};
			barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
			barrier.Transition = {frame.m_render_target, D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_SOURCE};
			list->ResourceBarrier(1, &barrier);
			D3D12_TEXTURE_COPY_LOCATION source{};
			source.pResource = frame.m_render_target;
			source.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
			D3D12_TEXTURE_COPY_LOCATION destination{};
			destination.pResource = readback.Get();
			destination.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
			destination.PlacedFootprint = footprint;
			list->CopyTextureRegion(&destination, 0, 0, 0, &source, nullptr);
			std::swap(barrier.Transition.StateBefore, barrier.Transition.StateAfter);
			list->ResourceBarrier(1, &barrier);
			Check(list->Close());
			ID3D12CommandList* lists[] = {list.Get()};
			queue->ExecuteCommandLists(1, lists);
			ComPtr<ID3D12Fence> fence;
			Check(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
			Check(queue->Signal(fence.Get(), 1));
			auto event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
			Check(fence->SetEventOnCompletion(1, event));
			auto wait = WaitForSingleObject(event, 10000);
			CloseHandle(event);
			Require(wait == WAIT_OBJECT_0, "Readback timed out");

			// Strip GPU row padding so image comparisons describe pixels only.
			unsigned char* mapped{};
			D3D12_RANGE range{0, static_cast<SIZE_T>(size)};
			Check(readback->Map(0, &range, reinterpret_cast<void**>(&mapped)));
			std::vector<unsigned char> pixels(ImageSize * ImageSize * 4);
			for (auto y = 0; y != ImageSize; ++y)
				std::memcpy(pixels.data() + y * ImageSize * 4, mapped + footprint.Offset + y * footprint.Footprint.RowPitch, ImageSize * 4);

			readback->Unmap(0, nullptr);
			if (bgra)
				for (auto i = size_t{}; i != pixels.size(); i += 4)
					std::swap(pixels[i], pixels[i+2]);

			return pixels;
		}

		// Read one completed reflection-sidecar pixel after the renderer has finished using the resource.
		std::array<float, 4> ReflectionPixel(int x, int y)
		{
			// Borrow the internal diagnostic resource without changing its renderer-owned lifetime.
			pr::rdr12::RenderRayTracing* ray_tracing = nullptr;
			for (auto const& step : m_window->m_scene.m_render_steps)
			{
				if (step->m_step_id == pr::rdr12::ERenderStep::RayTracing)
					ray_tracing = static_cast<pr::rdr12::RenderRayTracing*>(step.get());
			}
			Require(ray_tracing != nullptr, "Ray tracing render step is unavailable");
			auto* attributes = const_cast<ID3D12Resource*>(ray_tracing->ReflectionAttributesForDiagnostics());
			Require(attributes != nullptr, "Reflection attributes were not prepared");
			auto desc = attributes->GetDesc();
			Require(desc.Format == pr::rdr12::RayTracingReflectionAttributeFormat, "Unexpected reflection attribute format");

			// Resolve MSAA before copying the selected half-float texel to a CPU-visible buffer.
			ComPtr<ID3D12Resource> resolved;
			auto copy_desc = desc;
			copy_desc.SampleDesc = {1, 0};
			copy_desc.Flags = D3D12_RESOURCE_FLAG_NONE;
			if (desc.SampleDesc.Count != 1)
			{
				D3D12_HEAP_PROPERTIES default_heap{};
				default_heap.Type = D3D12_HEAP_TYPE_DEFAULT;
				Check(m_device->CreateCommittedResource(&default_heap, D3D12_HEAP_FLAG_NONE, &copy_desc, D3D12_RESOURCE_STATE_RESOLVE_DEST, nullptr, IID_PPV_ARGS(&resolved)));
			}

			D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
			UINT64 size{};
			m_device->GetCopyableFootprints(&copy_desc, 0, 1, 0, &footprint, nullptr, nullptr, &size);
			D3D12_HEAP_PROPERTIES readback_heap{};
			readback_heap.Type = D3D12_HEAP_TYPE_READBACK;
			D3D12_RESOURCE_DESC buffer{};
			buffer.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
			buffer.Width = size;
			buffer.Height = buffer.DepthOrArraySize = buffer.MipLevels = 1;
			buffer.SampleDesc.Count = 1;
			buffer.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
			ComPtr<ID3D12Resource> readback;
			Check(m_device->CreateCommittedResource(&readback_heap, D3D12_HEAP_FLAG_NONE, &buffer, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&readback)));

			// Use a private queue only after the public GPU wait has completed, then restore the renderer's tracked resource state.
			ComPtr<ID3D12CommandQueue> queue;
			D3D12_COMMAND_QUEUE_DESC queue_desc{};
			Check(m_device->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&queue)));
			ComPtr<ID3D12CommandAllocator> allocator;
			Check(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
			ComPtr<ID3D12GraphicsCommandList> list;
			Check(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(&list)));
			auto source = attributes;
			D3D12_RESOURCE_BARRIER barriers[2]{};
			barriers[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
			barriers[0].Transition = {
				attributes,
				D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
				D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
				desc.SampleDesc.Count == 1 ? D3D12_RESOURCE_STATE_COPY_SOURCE : D3D12_RESOURCE_STATE_RESOLVE_SOURCE,
			};
			list->ResourceBarrier(1, barriers);
			if (resolved)
			{
				list->ResolveSubresource(resolved.Get(), 0, attributes, 0, desc.Format);
				barriers[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
				barriers[1].Transition = {resolved.Get(), D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_COPY_SOURCE};
				list->ResourceBarrier(1, &barriers[1]);
				source = resolved.Get();
			}
			D3D12_TEXTURE_COPY_LOCATION source_location{};
			source_location.pResource = source;
			source_location.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
			D3D12_TEXTURE_COPY_LOCATION destination{};
			destination.pResource = readback.Get();
			destination.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
			destination.PlacedFootprint = footprint;
			list->CopyTextureRegion(&destination, 0, 0, 0, &source_location, nullptr);
			std::swap(barriers[0].Transition.StateBefore, barriers[0].Transition.StateAfter);
			list->ResourceBarrier(1, barriers);
			Check(list->Close());
			ID3D12CommandList* lists[] = {list.Get()};
			queue->ExecuteCommandLists(1, lists);
			ComPtr<ID3D12Fence> fence;
			Check(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
			Check(queue->Signal(fence.Get(), 1));
			auto event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
			Check(fence->SetEventOnCompletion(1, event));
			auto wait = WaitForSingleObject(event, 10000);
			CloseHandle(event);
			Require(wait == WAIT_OBJECT_0, "Reflection readback timed out");

			// Decode the four IEEE-754 binary16 channels used by the reflection sidecar.
			auto half_to_float = [](uint16_t value)
			{
				auto sign = (value & 0x8000U) != 0 ? -1.0f : 1.0f;
				auto exponent = (value >> 10) & 0x1FU;
				auto mantissa = value & 0x03FFU;
				if (exponent == 0)
					return sign * std::ldexp(static_cast<float>(mantissa), -24);

				if (exponent == 0x1FU)
					return mantissa == 0 ? sign * std::numeric_limits<float>::infinity() : std::numeric_limits<float>::quiet_NaN();

				return sign * std::ldexp(1.0f + static_cast<float>(mantissa) / 1024.0f, static_cast<int>(exponent) - 15);
			};
			unsigned char* mapped{};
			D3D12_RANGE range{0, static_cast<SIZE_T>(size)};
			Check(readback->Map(0, &range, reinterpret_cast<void**>(&mapped)));
			auto* pixel = reinterpret_cast<uint16_t const*>(mapped + footprint.Offset + y * footprint.Footprint.RowPitch + x * 8);
			auto result = std::array<float, 4>{half_to_float(pixel[0]), half_to_float(pixel[1]), half_to_float(pixel[2]), half_to_float(pixel[3])};
			readback->Unmap(0, nullptr);
			return result;
		}

		// Reject GPU debug-layer errors after all bounded render cases have completed.
		void CheckDebugLayer()
		{
			if (!m_info)
				return;

			for (UINT64 i = 0; i != m_info->GetNumStoredMessages(); ++i)
			{
				SIZE_T size{};
				m_info->GetMessage(i, nullptr, &size);
				std::vector<unsigned char> storage(size);
				auto* message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
				Check(m_info->GetMessage(i, message, &size));
				if (message->Severity == D3D12_MESSAGE_SEVERITY_ERROR || message->Severity == D3D12_MESSAGE_SEVERITY_CORRUPTION)
					throw std::runtime_error(message->pDescription);
			}
		}
	};

	// Count differing RGB bytes so GPU channel tests do not depend on one hand-picked lattice value.
	size_t ImageDifference(std::vector<unsigned char> const& lhs, std::vector<unsigned char> const& rhs)
	{
		// Compare only visible colour channels while requiring identical framebuffer layouts.
		Require(lhs.size() == rhs.size(), "Image sizes differ");
		auto difference = size_t{};
		for (auto i = size_t{}; i != lhs.size(); i += 4)
		{
			if (lhs[i + 0] != rhs[i + 0] || lhs[i + 1] != rhs[i + 1] || lhs[i + 2] != rhs[i + 2])
				++difference;
		}
		return difference;
	}

	// Count changed pixels whose replacement is the fixed black diagnostic edge colour.
	size_t BlackOverlayDifference(std::vector<unsigned char> const& solid, std::vector<unsigned char> const& solid_wire)
	{
		// Require matching framebuffer layouts before comparing visible RGB values.
		Require(solid.size() == solid_wire.size(), "Image sizes differ");
		auto difference = size_t{};
		for (auto i = size_t{}; i != solid.size(); i += 4)
		{
			auto changed = solid[i + 0] != solid_wire[i + 0] || solid[i + 1] != solid_wire[i + 1] || solid[i + 2] != solid_wire[i + 2];
			auto black = solid_wire[i + 0] <= 2 && solid_wire[i + 1] <= 2 && solid_wire[i + 2] <= 2;
			if (changed && black)
				++difference;
		}
		return difference;
	}

	// Prove the complete scene-wide SolidWire contract through real forward rendering and framebuffer readback.
	void SolidWireTests()
	{
		// Exercise both ordinary and multisampled render targets because the mode is a per-frame diagnostic.
		for (auto samples : {1, 4})
		{
			Fixture fixture(samples);
			fixture.AttachOverlay();

			// Opaque indexed triangles gain black edges while material interiors and final retained UI remain unchanged.
			auto object = fixture.Quad(10, 0xFF4080FF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
			auto surface = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Grass);
			View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto solid = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::SolidWire);
			auto solid_wire = fixture.Image();
			Require(BlackOverlayDifference(solid, solid_wire) > 100, "SolidWire did not add fixed black triangle edges");
			auto interior = (64 * ImageSize + 32) * 4;
			Require(
				solid[interior + 0] == solid_wire[interior + 0] &&
				solid[interior + 1] == solid_wire[interior + 1] &&
				solid[interior + 2] == solid_wire[interior + 2],
				"SolidWire changed the procedural solid interior");
			for (auto y = 4; y != 20; ++y)
			{
				for (auto x = 4; x != 20; ++x)
				{
					auto pixel = (y * ImageSize + x) * 4;
					Require(
						solid[pixel + 0] == solid_wire[pixel + 0] &&
						solid[pixel + 1] == solid_wire[pixel + 1] &&
						solid[pixel + 2] == solid_wire[pixel + 2],
						"SolidWire changed final retained UI output");
				}
			}

			// A nearer explicitly solid surface occludes the diagnostic edges behind it without hidden-line leakage.
			fixture.Clear();
			fixture.Quad(10, 0xFF20E040, 20, nullptr, 0, 0xFFFFFFFF, false, false, api::EFillMode::Solid);
			auto far_quad = fixture.Quad(12, 0xFFFF8000);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto occluded_solid = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::SolidWire);
			auto occluded_wire = fixture.Image();
			auto centre = (64 * ImageSize + 64) * 4;
			Require(
				occluded_solid[centre + 0] == occluded_wire[centre + 0] &&
				occluded_solid[centre + 1] == occluded_wire[centre + 1] &&
				occluded_solid[centre + 2] == occluded_wire[centre + 2],
				"SolidWire leaked hidden edges through nearer geometry");
			Require(BlackOverlayDifference(occluded_solid, occluded_wire) > 20, "Visible background edges were lost with an occluder");

			// Disable the farther object's depth test to prove the centre witness detects a hidden-edge rejection failure.
			View3D_ObjectFlagsSet(far_quad, api::ELdrFlags::NoZTest, TRUE, nullptr);
			auto depth_disabled = fixture.Image();
			Require(
				depth_disabled[centre + 0] != occluded_wire[centre + 0] ||
				depth_disabled[centre + 1] != occluded_wire[centre + 1] ||
				depth_disabled[centre + 2] != occluded_wire[centre + 2],
				"Occlusion witness did not detect disabled depth rejection");
			View3D_ObjectFlagsSet(far_quad, api::ELdrFlags::NoZTest, FALSE, nullptr);

			// Sky and alpha groups retain one ordinary material pass instead of receiving duplicate diagnostic shading.
			fixture.Clear();
			auto sky = fixture.Quad(50, 0xFF305080);
			View3D_ObjectSortGroupSet(sky, api::ESortGroup::Skybox, nullptr);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto sky_solid = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::SolidWire);
			Require(sky_solid == fixture.Image(), "SolidWire changed sky rendering");
			fixture.Clear();
			fixture.Quad(12, 0xFF0000FF, 45, nullptr, 0, 0xFFFFFFFF, false, false, api::EFillMode::Solid);
			fixture.Quad(10, 0x80FF4000);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto alpha_solid = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::SolidWire);
			Require(alpha_solid == fixture.Image(), "SolidWire shaded or collected alpha geometry twice");

			// Non-triangle primitives are unchanged, while the pre-existing pure Wireframe mode remains edge-only and material-coloured.
			fixture.Clear();
			fixture.Line(10, 0xFFFFFFFF);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto line_solid = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::SolidWire);
			Require(line_solid == fixture.Image(), "SolidWire changed a non-triangle primitive");
			fixture.Clear();
			fixture.Triangle(10, 0xFF4080FF);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto unindexed_solid = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::SolidWire);
			Require(BlackOverlayDifference(unindexed_solid, fixture.Image()) > 20, "SolidWire omitted an unindexed triangle");

			// Faded opaque geometry must replace its collected layer colour at edges without collecting its opacity twice.
			for (auto indexed : {true, false})
			{
				// The quad diagonal crosses the centre pixel in both indexed and direct-draw forms.
				Fixture fade_fixture(samples);
				fade_fixture.Fade(true);
				View3D_WindowFillModeSet(fade_fixture.m_window, api::EFillMode::Solid);
				if (indexed)
					fade_fixture.Quad(94.5f, 0xFFFF0000);
				else
					fade_fixture.UnindexedQuad(94.5f, 0xFFFF0000);

				auto faded_solid = fade_fixture.Image();
				View3D_WindowFillModeSet(fade_fixture.m_window, api::EFillMode::SolidWire);
				auto faded_wire = fade_fixture.Image();
				auto edge = (64 * ImageSize + 64) * 4;
				auto fade_interior = (32 * ImageSize + 64) * 4;
				Require(Linear(faded_wire[edge + 0]) < 0.1f, indexed ? "Indexed faded edge retained material colour" : "Unindexed faded edge retained material colour");
				Require(std::abs(int(faded_wire[fade_interior + 0]) - int(faded_solid[fade_interior + 0])) <= 1, indexed ? "Indexed faded interior changed" : "Unindexed faded interior changed");
			}

			// A white clear colour exposes whether the black edge preserved the original 0.5 fade opacity.
			Fixture alpha_fixture(samples, 0xFFFFFFFF);
			alpha_fixture.Fade(true);
			View3D_WindowFillModeSet(alpha_fixture.m_window, api::EFillMode::SolidWire);
			alpha_fixture.Quad(94.5f, 0xFFFF0000);
			auto faded_alpha = alpha_fixture.Image();
			auto alpha_edge = (64 * ImageSize + 64) * 4;
			for (auto channel = 0; channel != 3; ++channel)
				Require(std::abs(Linear(faded_alpha[alpha_edge + channel]) - 0.5f) < 0.08f, "Faded wire edge changed source opacity");

			// The edge redraw must leave the reflection MRT normal and final reflected hit unchanged.
			Fixture reflection_fixture(samples);
			auto rt_info = View3D_WindowRayTracingInfoGet(reflection_fixture.m_window);
			Require(rt_info.m_available, "SolidWire reflection MRT test requires DXR");
			auto mirror = View3D_ObjectCreateLdrA("*Plane Mirror FF000000 {*Data{90 90} *Reflectivity{1} *o2w{*pos{0 0 -50}}}", FALSE, nullptr, nullptr);
			Require(mirror != nullptr, "SolidWire mirror creation failed");
			reflection_fixture.m_objects.push_back(mirror);
			View3D_WindowAddObject(reflection_fixture.m_window, mirror);
			reflection_fixture.Quad(-10, 0xFF00FF00);
			View3D_RayTracingPropertiesSet(reflection_fixture.m_window, {api::ERayTracingFeature::Reflections, 1});
			View3D_WindowRayTracingEnabledSet(reflection_fixture.m_window, TRUE);
			View3D_WindowFillModeSet(reflection_fixture.m_window, api::EFillMode::Solid);
			reflection_fixture.Image();
			auto reflected_solid = reflection_fixture.Image();
			auto solid_attrs = reflection_fixture.ReflectionPixel(64, 64);
			View3D_WindowFillModeSet(reflection_fixture.m_window, api::EFillMode::SolidWire);
			reflection_fixture.Image();
			auto reflected_wire = reflection_fixture.Image();
			auto wire_attrs = reflection_fixture.ReflectionPixel(64, 64);
			Require(std::abs(solid_attrs[0] - 0.5f) < 0.01f && std::abs(solid_attrs[1] - 0.5f) < 0.01f && solid_attrs[2] > 0.99f && solid_attrs[3] > 0.99f, "Reflective reference sidecar has the wrong normal");
			for (auto channel = 0; channel != 4; ++channel)
				Require(std::abs(wire_attrs[channel] - solid_attrs[channel]) < 0.01f, "SolidWire corrupted the reflection sidecar");
			auto reflected_pixel = (64 * ImageSize + 64) * 4;
			Require(Linear(reflected_solid[reflected_pixel + 1]) > 0.5f, "Reflective reference did not hit the known green target");
			Require(std::abs(Linear(reflected_wire[reflected_pixel + 1]) - Linear(reflected_solid[reflected_pixel + 1])) < 0.05f, "SolidWire changed the known-normal reflected hit");
			View3D_WindowRayTracingEnabledSet(reflection_fixture.m_window, FALSE);

			fixture.Clear();
			auto empty = fixture.Image();
			fixture.Quad(10, 0xFF4080FF);
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Solid);
			auto filled = fixture.Image();
			View3D_WindowFillModeSet(fixture.m_window, api::EFillMode::Wireframe);
			auto wire = fixture.Image();
			Require(ImageDifference(filled, wire) > 1000, "Pure Wireframe no longer removed triangle interiors");
			Require(ImageDifference(empty, wire) > 100, "Pure Wireframe lost its material-coloured triangle edges");
			fixture.CheckDebugLayer();
		}
		std::cout << "PASS solid-wire opaque edges, occlusion, sky, alpha, nontriangles, UI, and pure wireframe\n";
	}

	// Exercise the public procedural surface API through real forward rendering and framebuffer readback.
	void ProceduralSurfaceTests()
	{
		// Use one bounded fixture and replace its scene between independent material contracts.
		Fixture fixture(1);
		auto object = fixture.Quad(10, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto presets = std::array{
			api::EProceduralSurfacePreset::Soil,
			api::EProceduralSurfacePreset::Grass,
			api::EProceduralSurfacePreset::Sand,
			api::EProceduralSurfacePreset::Rock,
			api::EProceduralSurfacePreset::Snow,
		};
		auto images = std::vector<std::vector<unsigned char>>{};
		for (auto preset : presets)
		{
			auto surface = View3D_ProceduralSurfacePreset(preset);
			surface.m_feature_scale = 7.0f;
			surface.m_seed = 0x12345678u;
			View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
			fixture.CheckErrors();
			images.push_back(fixture.Image());
		}
		for (auto i = size_t{1}; i != images.size(); ++i)
			Require(ImageDifference(images[0], images[i]) > 1000, "Procedural presets did not produce distinct rendered surfaces");

		// Repeated frames and camera translation along the view axis must not move a world-coordinate field.
		auto default_light = View3D_LightPropertiesGet(fixture.m_window);
		auto ambient_light = default_light;
		ambient_light.m_ambient = 0xFFFFFFFF;
		ambient_light.m_diffuse = 0xFF000000;
		ambient_light.m_specular = 0xFF000000;
		ambient_light.m_on = TRUE;
		ambient_light.m_intensity = 1.0f;
		View3D_LightPropertiesSet(fixture.m_window, ambient_light);
		auto soil = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Soil);
		soil.m_feature_scale = 5.0f;
		soil.m_seed = 42;
		View3D_ObjectNuggetProceduralSurfaceSet(object, soil, nullptr, 0);
		auto deterministic = fixture.Image();
		Require(deterministic == fixture.Image(), "Procedural output changed between identical GPU frames");
		api::Mat4x4 camera{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,2,1}};
		View3D_CameraToWorldSet(fixture.m_window, camera);
		Require(deterministic == fixture.Image(), "Camera movement changed stable world-coordinate procedural output");
		camera.w.z = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);

		// Seed and palette changes must affect the GPU-evaluated albedo rather than a generated texture resource.
		auto changed_seed = soil;
		changed_seed.m_seed = 43;
		View3D_ObjectNuggetProceduralSurfaceSet(object, changed_seed, nullptr, 0);
		Require(ImageDifference(deterministic, fixture.Image()) > 1000, "Procedural seed did not affect rendered output");
		auto red = soil;
		red.m_colour0 = red.m_colour1 = red.m_colour2 = red.m_colour3 = 0xFFFF2020;
		View3D_ObjectNuggetProceduralSurfaceSet(object, red, nullptr, 0);
		auto red_image = fixture.Image();
		auto green = red;
		green.m_colour0 = green.m_colour1 = green.m_colour2 = green.m_colour3 = 0xFF20FF20;
		View3D_ObjectNuggetProceduralSurfaceSet(object, green, nullptr, 0);
		Require(ImageDifference(red_image, fixture.Image()) > 1000, "Procedural albedo palette did not affect rendered output");

		// Normal strength and roughness ranges independently affect lit PBR output without UVs or tangent streams.
		View3D_LightPropertiesSet(fixture.m_window, default_light);
		auto channels = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Rock);
		channels.m_feature_scale = 8.0f;
		channels.m_colour0 = channels.m_colour1 = channels.m_colour2 = channels.m_colour3 = 0xFF808080;
		channels.m_normal_strength = 0.0f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		auto flat_normal = fixture.Image();
		channels.m_normal_strength = 1.2f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		Require(ImageDifference(flat_normal, fixture.Image()) > 100, "Procedural normal channel did not affect rendered output");
		channels.m_normal_strength = 0.0f;
		channels.m_roughness_min = channels.m_roughness_max = 0.04f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		auto smooth = fixture.Image();
		channels.m_roughness_min = channels.m_roughness_max = 1.0f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		Require(ImageDifference(smooth, fixture.Image()) > 50, "Procedural roughness channel did not affect rendered output");

		// Object-space coordinates move with the object rather than being anchored to the world field.
		View3D_LightPropertiesSet(fixture.m_window, ambient_light);
		auto object_space = soil;
		object_space.m_coordinate_space = api::EProceduralCoordinateSpace::Object;
		View3D_ObjectNuggetProceduralSurfaceSet(object, object_space, nullptr, 0);
		auto object_reference = fixture.Image();
		api::Mat4x4 translated{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {1000,0,0,1}};
		View3D_ObjectO2WSet(object, translated, nullptr);
		camera.w.x = 1000;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		auto moved_object = fixture.Image();
		auto object_difference = size_t{};
		for (auto y = 24; y != 104; ++y)
		{
			for (auto x = 24; x != 104; ++x)
			{
				auto pixel = (y * ImageSize + x) * 4;
				if (object_reference[pixel + 0] != moved_object[pixel + 0] || object_reference[pixel + 1] != moved_object[pixel + 1] || object_reference[pixel + 2] != moved_object[pixel + 2])
					++object_difference;
			}
		}
		Require(object_difference < 100, "Object-coordinate procedural field did not move with the object");
		translated.w.x = 0;
		View3D_ObjectO2WSet(object, translated, nullptr);
		camera.w.x = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		View3D_LightPropertiesSet(fixture.m_window, default_light);

		// A large translated object can retain local detail by selecting a matching caller-owned coordinate origin.
		auto large = soil;
		View3D_ObjectNuggetProceduralSurfaceSet(object, large, nullptr, 0);
		fixture.Image();
		large.m_coordinate_origin = {1000000, 0, 0, 1};
		View3D_ObjectNuggetProceduralSurfaceSet(object, large, nullptr, 0);
		translated.w.x = 1000000;
		View3D_ObjectO2WSet(object, translated, nullptr);
		camera.w.x = 1000000;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		auto large_image = fixture.Image();
		Require(large_image == fixture.Image(), "Large-coordinate procedural output was not deterministic");
		auto varied_pixels = size_t{};
		auto reference_pixel = (64 * ImageSize + 64) * 4;
		for (auto y = 24; y != 104; ++y)
		{
			for (auto x = 24; x != 104; ++x)
			{
				auto pixel = (y * ImageSize + x) * 4;
				if (large_image[reference_pixel + 0] != large_image[pixel + 0] || large_image[reference_pixel + 1] != large_image[pixel + 1] || large_image[reference_pixel + 2] != large_image[pixel + 2])
					++varied_pixels;
			}
		}
		Require(varied_pixels > 1000, "Large-coordinate procedural output lost local detail");

		// A broad field must not repeat like a wrapped 2D texture over separated image regions.
		auto non_repeating = large_image;
		auto patch_equal = true;
		for (auto y = 40; y != 88 && patch_equal; ++y)
		{
			for (auto x = 20; x != 44; ++x)
			{
				auto lhs = (y * ImageSize + x) * 4;
				auto rhs = (y * ImageSize + x + 64) * 4;
				if (non_repeating[lhs + 0] != non_repeating[rhs + 0] || non_repeating[lhs + 1] != non_repeating[rhs + 1] || non_repeating[lhs + 2] != non_repeating[rhs + 2])
				{
					patch_equal = false;
					break;
				}
			}
		}
		Require(!patch_equal, "Procedural field repeated across separated framebuffer regions");

		// Adjacent independently owned meshes must match the same world-coordinate field on both sides of their shared boundary.
		camera.w.x = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		View3D_LightPropertiesSet(fixture.m_window, ambient_light);
		fixture.Clear();
		auto full = fixture.QuadRect(-45, 45, 10);
		View3D_ObjectNuggetProceduralSurfaceSet(full, soil, nullptr, 0);
		auto continuous_reference = fixture.Image();
		fixture.Clear();
		auto left = fixture.QuadRect(-45, 0, 10);
		auto right = fixture.QuadRect(0, 45, 10);
		View3D_ObjectNuggetProceduralSurfaceSet(left, soil, nullptr, 0);
		View3D_ObjectNuggetProceduralSurfaceSet(right, soil, nullptr, 0);
		auto split = fixture.Image();
		auto continuity_difference = size_t{};
		for (auto y = 8; y != ImageSize - 8; ++y)
		{
			for (auto x = 8; x != ImageSize - 8; ++x)
			{
				if (x >= 62 && x <= 65)
					continue;

				auto pixel = (y * ImageSize + x) * 4;
				if (continuous_reference[pixel + 0] != split[pixel + 0] || continuous_reference[pixel + 1] != split[pixel + 1] || continuous_reference[pixel + 2] != split[pixel + 2])
					++continuity_difference;
			}
		}
		Require(continuity_difference == 0, "World-coordinate procedural field changed across a mesh boundary");
		View3D_LightPropertiesSet(fixture.m_window, default_light);

		// Promotion preserves an ordinary material's alpha path so procedural colour still composites as authored.
		fixture.Clear();
		View3D_LightPropertiesSet(fixture.m_window, ambient_light);
		fixture.Quad(12, 0xFF0000FF);
		auto transparent = fixture.Quad(10, 0x80FFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		red.m_normal_strength = 0.0f;
		View3D_ObjectNuggetProceduralSurfaceSet(transparent, red, nullptr, 0);
		auto alpha_image = fixture.Image();
		auto alpha_pixel = (64 * ImageSize + 64) * 4;
		Require(alpha_image[alpha_pixel + 0] > 40 && alpha_image[alpha_pixel + 2] > 40, "Procedural assignment discarded ordinary alpha blending");

		// Custom shader overlays are rejected because stock PBR promotion cannot preserve caller-supplied shader stages.
		fixture.Clear();
		auto custom = fixture.Quad(10, 0xFFFFFFFF, 45, fixture.CustomShader(true), 0, 0xFFFFFFFF, false, true);
		View3D_ObjectNuggetProceduralSurfaceSet(custom, soil, nullptr, 0);
		Require(!fixture.m_errors.empty(), "Procedural assignment silently discarded a custom shader overlay");
		Require(fixture.m_errors.back().find("custom shader overlays") != std::string::npos, "Custom shader rejection reported the wrong diagnostic");
		fixture.m_errors.clear();
		api::ProceduralSurface round_trip{};
		Require(!View3D_ObjectNuggetProceduralSurfaceGet(custom, round_trip, nullptr, 0), "Rejected custom shader assignment changed the material");

		// Public round-trip and clear operations preserve ordinary material ownership.
		fixture.Clear();
		View3D_LightPropertiesSet(fixture.m_window, ambient_light);
		auto round_trip_object = fixture.QuadRect(-45, 0, 10);
		View3D_ObjectNuggetProceduralSurfaceSet(round_trip_object, soil, nullptr, 0);
		Require(View3D_ObjectNuggetProceduralSurfaceGet(round_trip_object, round_trip, nullptr, 0), "Procedural surface getter did not find assigned state");
		Require(round_trip.m_seed == soil.m_seed && round_trip.m_feature_scale == soil.m_feature_scale, "Procedural surface getter changed caller parameters");

		// Unsupported DXR secondary-hit shading must report a material diagnostic instead of silently using flat PBR values.
		if (View3D_WindowRayTracingInfoGet(fixture.m_window).m_available)
		{
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			View3D_WindowRender(fixture.m_window);
			Require(!fixture.m_errors.empty(), "DXR silently accepted a procedural secondary-hit material");
			Require(fixture.m_errors.back().find("procedural surface") != std::string::npos, "DXR reported the wrong procedural material diagnostic");
			fixture.m_errors.clear();
			View3D_WindowRayTracingEnabledSet(fixture.m_window, FALSE);
		}
		View3D_ObjectNuggetProceduralSurfaceClear(round_trip_object, nullptr, 0);
		Require(!View3D_ObjectNuggetProceduralSurfaceGet(round_trip_object, round_trip, nullptr, 0), "Procedural surface clear retained assigned state");
		fixture.CheckDebugLayer();
		std::cout << "PASS procedural presets, determinism, channels, large coordinates, and non-repetition\n";
	}

	// Assert compositing against linear source-over expectations, allowing RGBA8 quantization.
	void Expect(std::vector<unsigned char> const& image, float red, float green, float blue, int x = 64, int y = 64)
	{
		auto pixel = image.data() + (y * ImageSize + x) * 4;
		float expected[] = {red, green, blue};
		for (auto channel = 0; channel != 3; ++channel)
		{
			auto actual = Linear(pixel[channel]);
			if (std::abs(actual - expected[channel]) > 0.025f)
				throw std::runtime_error("Pixel mismatch channel " + std::to_string(channel) + ": actual " + std::to_string(actual) + ", expected " + std::to_string(expected[channel]));
		}
	}

	// Verify normal directions independently of their arbitrary common matrix scale.
	void NormalTransformTests()
	{
		using namespace pr;
		auto const normal = Normalise(v4(1, 2, 3, 0));
		auto scale = m4x4::Identity();
		scale.x.x = 2;
		scale.y.y = 3;
		scale.z.z = 0.25f;
		auto shear = m4x4::Identity();
		shear.y.x = 0.7f;
		shear.z.y = -0.3f;
		auto reflection = m4x4::Identity();
		reflection.x.x = -1;
		auto rotation = m4x4(v4(0, 1, 0, 0), v4(-1, 0, 0, 0), v4(0, 0, 1, 0), v4(2, 3, 4, 1));
		for (auto const& placement : {m4x4::Identity(), scale, shear, reflection, rotation, rotation * scale * shear * reflection})
		{
			auto expected = Normalise(Transpose(Invert(placement)).rot * normal.xyz);
			auto actual = Normalise((rdr12::NormalTransform(placement) * normal).xyz);
			Require(Length(actual - expected) < 0.00001f, "Normal transform differs from inverse transpose");
		}

		// The singular endpoint retains the remaining plane, while a line or point has no area normal.
		auto flat = m4x4::Identity();
		flat.z.z = 0;
		auto flat_normal = Normalise(rdr12::NormalTransform(rotation * flat) * normal);
		Require(Length(flat_normal - rotation.z) < 0.00001f, "Flattening lost the surviving plane normal");
		flat.y.y = 0;
		Require(Length(rdr12::NormalTransform(flat) * normal) == 0, "A line must not invent an area normal");
		Require(Length(rdr12::NormalTransform(m4x4::Zero()) * normal) == 0, "A point must not invent an area normal");

		// Extremely large and small uniform scales must not overflow their intermediate cross products.
		for (auto magnitude : {1.0e-30f, 1.0e30f})
		{
			auto placement = m4x4::Identity();
			placement.x.x = placement.y.y = placement.z.z = magnitude;
			Require(Length(Normalise(rdr12::NormalTransform(placement) * normal) - normal) < 0.00001f, "Normal transform lost extreme scale");
		}
	}

	// Validate defaults, invalid ranges, and absolute far-depth resolution without a GPU.
	void NumericTests()
	{
		auto props = pr::rdr12::FarClipFadeProps{};
		Require(!props.m_enabled, "Default must be disabled");
		auto range = props.DepthRange(100);
		Require(range.x == 90 && range.y == 99, "Depth range incorrect");
		float invalid[][2] = {{-1,0.99f}, {0.9f,0.9f}, {0.9f,1}, {0.9f,std::numeric_limits<float>::infinity()}, {std::numeric_limits<float>::quiet_NaN(),0.99f}};
		for (auto const& pair : invalid)
		{
			props.m_start_fraction = pair[0];
			props.m_end_fraction = pair[1];
			auto rejected = false;
			try { props.Validate(); }
			catch (std::invalid_argument const&) { rejected = true; }
			Require(rejected, "Invalid range accepted");
		}

		// Camera-dependent validation also rejects intervals that cannot end before hardware clipping.
		props = {};
		float invalid_depths[] = {0, -1, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::denorm_min()};
		for (auto depth : invalid_depths)
		{
			auto rejected = false;
			try { props.DepthRange(depth); }
			catch (std::invalid_argument const&) { rejected = true; }
			Require(rejected, "Invalid camera depth accepted");
		}
	}

	// Exercise the real opaque/alpha renderer at 1x and MSAA without showing or controlling a user window.
	void RenderTests(int samples)
	{
		Fixture fixture(samples);
		auto defaults = View3D_FarClipFadePropertiesGet(fixture.m_window);
		Require(!defaults.m_enabled && defaults.m_start_fraction == 0.9f && defaults.m_end_fraction == 0.99f, "DLL defaults differ");
		Require(!View3D_FarClipFadePropertiesSet(fixture.m_window, api::FarClipFadeProps{TRUE,0.9f,1}), "DLL accepted invalid range");
		Require(!fixture.m_errors.empty(), "DLL did not report invalid range");
		fixture.m_errors.clear();
		Require(!View3D_FarClipFadePropertiesGet(fixture.m_window).m_enabled, "Failed setter mutated settings");

		// Disabled settings preserve full-image output; re-enabling cannot leave stale alpha or depth.
		std::cout << "Defaults and toggle, MSAA " << samples << std::endl;
		fixture.Quad(94.5f, 0xFFFF0000);
		auto disabled = fixture.Image();
		Expect(disabled, 1,0,0);
		fixture.Fade(true);
		Expect(fixture.Image(), 0.5f,0,0);
		fixture.Fade(false);
		Require(disabled == fixture.Image(), "Disable did not restore original pixels");

		// Planar off-axis pixels must use forward depth rather than radial distance.
		std::cout << "Forward depth and ramp" << std::endl;
		fixture.Fade(true);
		Expect(fixture.Image(), 0.5f,0,0, 105,64);
		float depths[] = {89,90,92.25f,94.5f,96.75f,99,99.5f};
		float opacity[] = {1,1,0.84375f,0.5f,0.15625f,0,0};
		for (auto i = 0; i != 7; ++i)
		{
			fixture.Clear();
			fixture.Quad(depths[i], 0xFFFF0000);
			Expect(fixture.Image(), opacity[i],0,0);
		}

		// A single primitive crossing the interval must partition per fragment, not per object.
		fixture.Clear();
		fixture.Quad(89, 0xFFFF0000, 45, nullptr, 100);
		auto crossing = fixture.Image();
		Expect(crossing, 1,0,0, 8,64);
		Expect(crossing, 0.49f,0,0);
		Expect(crossing, 0,0,0, 118,64);

		// Camera motion and clipping updates remain the only authority for the world-depth interval.
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000);
		View3D_CameraClipPlanesSet(fixture.m_window, 1, 105, api::EClipPlanes::Both);
		Expect(fixture.Image(), 1,0,0);
		View3D_CameraClipPlanesSet(fixture.m_window, 1, 100, api::EClipPlanes::Both);
		api::Mat4x4 camera{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,-4.5f,1}};
		View3D_CameraToWorldSet(fixture.m_window, camera);
		Expect(fixture.Image(), 1,0,0);
		camera.w.z = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		Expect(fixture.Image(), 0.5f,0,0);

		// Originally transparent material opacity is multiplied, not replaced by fade opacity.
		std::cout << "Material opacity and overlap" << std::endl;
		fixture.Clear();
		fixture.Quad(94.5f, 0x80FF0000);
		Expect(fixture.Image(), 0.25f,0,0);

		// Perspective off-axis coverage uses the same forward-depth interval.
		std::cout << "Perspective" << std::endl;
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000);
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{1.5707963268f,1.5707963268f});
		Expect(fixture.Image(), 0.5f,0,0, 90,64);
		View3D_CameraOrthographicSet(fixture.m_window, TRUE);
		View3D_CameraViewRectAtDistanceSet(fixture.m_window, api::Vec2{100,100}, 1.0f);

		// A deformed custom vertex shader retains stock lighting and fades at its emitted world depth.
		std::cout << "Custom vertex shader" << std::endl;
		fixture.Clear();
		fixture.Quad(90, 0xFFFF0000, 45, fixture.CustomShader(true));
		Expect(fixture.Image(), 0.5f,0,0);

		// An emissive PBR material reaches the same fade through its stock PBR output variant.
		std::cout << "PBR" << std::endl;
		fixture.Clear();
		auto pbr = View3D_ObjectCreateLdrA("*Plane PbrFade { *Data {90 90} *Material { *BaseColour {FF000000} *Emissive {FFFF0000} } *o2w {*pos {0 0 -94.5}} }", FALSE, nullptr, nullptr);
		Require(pbr != nullptr, "PBR fixture creation failed");
		fixture.m_objects.push_back(pbr);
		View3D_WindowAddObject(fixture.m_window, pbr);
		Expect(fixture.Image(), 0.5f,0,0);

		// Edge pixels retain the existing single-sample alpha policy even when opaque geometry uses MSAA.
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000, 31.125f);
		auto faded_edge = fixture.Image();
		fixture.Clear();
		fixture.Quad(89, 0x80FF0000, 31.125f);
		Require(faded_edge == fixture.Image(), "Fade introduced different coverage from existing alpha");
		std::cout << "Edge coverage matches existing alpha, MSAA " << samples << std::endl;

		// Source-over ordering is the same whether fading opaque geometry is in front of or behind transparency.
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000);
		fixture.Quad(96.75f, 0x800000FF);
		Expect(fixture.Image(), 0.5f,0,0.0392f);
		fixture.Clear();
		auto front = fixture.Quad(92.25f, 0x800000FF);
		auto back = fixture.Quad(94.5f, 0xFFFF0000);
		auto ordered = fixture.Image();
		Expect(ordered, 0.2882f,0,0.4235f);
		fixture.Clear();
		View3D_WindowAddObject(fixture.m_window, back);
		View3D_WindowAddObject(fixture.m_window, front);
		Require(ordered == fixture.Image(), "Submission order changed sorted alpha output");

		// Near opaque geometry must still hide every farther alpha layer.
		fixture.Quad(80, 0xFF00FF00);
		Expect(fixture.Image(), 0,1,0);

		// Background and retained final overlays stay outside the world fade.
		std::cout << "Background and UI" << std::endl;
		fixture.Clear();
		auto procedural_sky = View3D_ObjectCreateProceduralSky("FadeProceduralSky", {0.5f,0.3f,0.8f,0}, {1,0.95f,0.85f,1}, 1, nullptr);
		Require(procedural_sky != nullptr, "Procedural sky creation failed");
		fixture.m_objects.push_back(procedural_sky);
		Require(View3D_ObjectSortGroupGet(procedural_sky, nullptr) == api::ESortGroup::Skybox, "Procedural sky lost its background classification");
		View3D_ObjectFlagsSet(procedural_sky, api::ELdrFlags::Hidden, TRUE, nullptr);
		View3D_ObjectFlagsSet(procedural_sky, api::ELdrFlags::Hidden, FALSE, nullptr);
		View3D_ObjectFlagsSet(procedural_sky, api::ELdrFlags::NoZWrite, FALSE, nullptr);
		Require(View3D_ObjectSortGroupGet(procedural_sky, nullptr) == api::ESortGroup::Skybox, "Unrelated or unchanged flags erased the sky sort group");
		View3D_WindowAddObject(fixture.m_window, procedural_sky);
		auto faded_sky = fixture.Image();
		fixture.Fade(false);
		Require(faded_sky == fixture.Image(), "Procedural sky changed with fade enabled");
		fixture.Clear();
		fixture.Fade(true);

		// A plain skybox-group object follows the same exclusion as the procedural sky.
		auto sky = fixture.Quad(99.5f, 0xFF0000FF);
		View3D_ObjectSortGroupSet(sky, api::ESortGroup::Skybox, nullptr);
		Expect(fixture.Image(), 0,0,1);
		fixture.Clear();
		auto post_alpha = fixture.Quad(94.5f, 0xFF0000FF);
		View3D_ObjectSortGroupSet(post_alpha, api::ESortGroup::PostAlpha, nullptr);
		View3D_ObjectFlagsSet(post_alpha, api::ELdrFlags::HitTestExclude, TRUE, nullptr);
		Require(View3D_ObjectSortGroupGet(post_alpha, nullptr) == api::ESortGroup::PostAlpha, "Unrelated flags erased the overlay sort group");
		Expect(fixture.Image(), 0,0,1);
		fixture.Clear();

		// Invisible world geometry remains available to geometric picking.
		auto invisible = fixture.Quad(99.5f, 0xFFFF0000);
		Expect(fixture.Image(), 0,0,0);
		api::HitTestRay ray{};
		ray.m_ws_origin = {0,0,0,1};
		ray.m_ws_direction = {0,0,-1,0};
		api::HitTestResult hit{};
		View3D_WindowHitTestObjects(fixture.m_window, &ray, &hit, 1, &invisible, 1);
		fixture.CheckErrors();
		Require(hit.m_obj == invisible && std::abs(hit.m_distance - 99.5f) < 0.01f, "Fade changed geometric picking");

		// Final retained UI coverage is byte-identical with the world fade enabled or disabled.
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000);
		fixture.AttachOverlay();
		auto faded_ui = fixture.Image();
		Expect(faded_ui, 0,1,0, 10,10);
		fixture.Fade(false);
		auto solid_ui = fixture.Image();
		for (auto y = 4; y != 20; ++y)
			for (auto x = 4; x != 20; ++x)
				for (auto c = 0; c != 4; ++c)
					Require(faded_ui[(y*ImageSize+x)*4+c] == solid_ui[(y*ImageSize+x)*4+c], "Retained overlay changed");

		fixture.CheckDebugLayer();

		// Unsupported custom pixel output is an explicit error, not silently unfaded world geometry.
		std::cout << "Unsupported pixel rejection" << std::endl;
		fixture.Fade(true);
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000, 45, fixture.CustomShader(false));
		View3D_WindowRender(fixture.m_window);
		Require(!fixture.m_errors.empty(), "Unsupported pixel shader was silently accepted");
		Require(fixture.m_errors.back().find("stock forward") != std::string::npos, "Wrong custom shader rejection");
		fixture.m_errors.clear();

		// A custom vertex stage is supported only when it retains the stock resource-binding contract.
		fixture.Clear();
		fixture.Quad(94.5f, 0xFFFF0000, 45, fixture.UnsupportedRootSignature());
		View3D_WindowRender(fixture.m_window);
		Require(!fixture.m_errors.empty(), "Unsupported root signature was silently accepted");
		Require(fixture.m_errors.back().find("forward root signature") != std::string::npos, "Wrong root signature rejection");
		fixture.m_errors.clear();
		std::cout << "PASS fade ramp/crossing, camera, material alpha, custom VS, PBR, overlap, coverage, sky/PostAlpha/UI, picking, custom PS/root rejection: MSAA " << samples << '\n';
	}

	// Validate the RGB override API and GPU output without running the unrelated fade cases.
	void ColourBlendTests(int samples)
	{
		// The shared shader expression preserves alpha and the disabled value exactly, including HDR RGB.
		using colour_blend_tests::SurfaceColourBlend;
		auto const surface = pr::v4(1.5f, 0.25f, 0.75f, 0.37f);
		for (auto amount : {0.0f, 0.5f, 1.0f})
		{
			// Weights affect only RGB, not the original floating-point opacity.
			auto result = SurfaceColourBlend(surface, pr::v4(0.25f, 0.5f, 1.0f, amount));
			Require(result.w == surface.w, "Surface blend changed alpha");
			Require(result.x == surface.x + (0.25f - surface.x) * amount, "Surface blend weight differs");
		}

		// Use non-white vertices and material colour to detect accidental CPU tint/vertex multiplication.
		Fixture fixture(samples);
		auto quad = fixture.Quad(50, 0xFF808080, 45, nullptr, 0, 0xFF808080);
		auto original = fixture.Image();
		Require(View3D_ObjectColourBlendAmountGet(quad, nullptr) == 0, "New object blend is not disabled");
		auto const original_tint = View3D_ObjectColourGet(quad, FALSE, nullptr);
		auto const original_material = View3D_ObjectNuggetTintGet(quad, nullptr, 0);
		auto const original_flags = View3D_ObjectNuggetFlagsGet(quad, nullptr, 0);
		auto const original_sort = View3D_ObjectSortGroupGet(quad, nullptr);
		auto source = Linear(original[(64 * ImageSize + 64) * 4]);
		for (auto alpha : {0U, 128U, 255U, 0U})
		{
			// Packed alpha controls only RGB interpolation and never makes this opaque surface transparent.
			auto amount = alpha / 255.0f;
			auto blend = (alpha << 24) | 0x004080C0U;
			View3D_ObjectColourBlendSet(quad, blend, nullptr);
			auto image = fixture.Image();
			Expect(image, source + (Linear(64) - source) * amount, source + (Linear(128) - source) * amount, source + (Linear(192) - source) * amount);
			Require(View3D_ObjectColourBlendColourGet(quad, nullptr) == blend, "Packed target and weight did not round-trip");
			Require(View3D_ObjectColourBlendAmountGet(quad, nullptr) == amount, "Weight did not round-trip");
			Require(View3D_ObjectColourGet(quad, FALSE, nullptr) == original_tint, "Override mutated tint");
			Require(View3D_ObjectNuggetTintGet(quad, nullptr, 0) == original_material, "Override mutated material");
			Require(View3D_ObjectNuggetFlagsGet(quad, nullptr, 0) == original_flags, "Override mutated nugget flags");
			Require(View3D_ObjectSortGroupGet(quad, nullptr) == original_sort, "Override changed sort group");
			if (amount == 0)
				Require(image == original, "Zero override did not preserve exact pixels");
		}

		// Every alpha byte is valid and decodes linearly, with no sRGB conversion or separate weight storage.
		for (auto alpha = 0U; alpha != 256U; ++alpha)
		{
			// Exhaust the packed weight domain without requiring 256 GPU readbacks.
			auto blend = (alpha << 24) | 0x004080C0U;
			View3D_ObjectColourBlendSet(quad, blend, nullptr);
			Require(View3D_ObjectColourBlendColourGet(quad, nullptr) == blend, "Packed blend changed bytes");
			Require(View3D_ObjectColourBlendAmountGet(quad, nullptr) == alpha / 255.0f, "Packed weight did not decode as UNORM8");
		}

		// Texture transfer happens before the blend: encoded sRGB 0x80 is about 0.216 linear, not 0.502.
		fixture.Clear();
		auto textured = fixture.Quad(50, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, true);
		auto texture_options = api::TextureOptions{};
		texture_options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
		texture_options.m_usage = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
		texture_options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
		texture_options.m_clear_value.Format = texture_options.m_format;
		texture_options.m_mips = 1;
		texture_options.m_multisamp = {1, 0};
		texture_options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
		unsigned texture_pixel = 0xFF808080;
		auto texture = View3D_TextureCreate(1, 1, &texture_pixel, sizeof(texture_pixel), texture_options);
		Require(texture != nullptr, "Texture creation failed");
		View3D_ObjectSetTexture(textured, texture, nullptr);
		View3D_TextureRelease(texture);
		for (auto alpha : {0U, 128U, 255U})
		{
			// A white override replaces even a dark texel, rather than multiplying it again.
			auto amount = alpha / 255.0f;
			View3D_ObjectColourBlendSet(textured, (alpha << 24) | 0x00FFFFFFU, nullptr);
			auto expected = Linear(128) + (1 - Linear(128)) * amount;
			Expect(fixture.Image(), expected, expected, expected);
		}

		// Lit simple and PBR surfaces must match an equivalent authored base colour at full override.
		auto light = View3D_LightPropertiesGet(fixture.m_window);
		light.m_ambient = 0xFF808080;
		light.m_diffuse = light.m_specular = 0xFF000000;
		light.m_on = TRUE;
		light.m_intensity = 1;
		View3D_LightPropertiesSet(fixture.m_window, light);
		for (auto pbr : {false, true})
		{
			// Keep geometry and lighting identical while varying only the material's linear RGB.
			auto plane = [&](char const* colour)
			{
				// The normal-bearing plane exercises lighting, unlike the unlit vertex-colour fixture above.
				auto script = pbr
					? std::string("*Plane Lit { *Data {90 90} *Material {*BaseColour{") + colour + "} *Metallic{0} *Roughness{1}} *o2w{*pos{0 0 -50}}}"
					: std::string("*Plane Lit ") + colour + " { *Data {90 90} *o2w{*pos{0 0 -50}}}";
				auto object = View3D_ObjectCreateLdrA(script.c_str(), FALSE, nullptr, nullptr);
				Require(object != nullptr, "Lit plane creation failed");
				fixture.m_objects.push_back(object);
				View3D_WindowAddObject(fixture.m_window, object);
				return object;
			};
			fixture.Clear();
			plane("FF004000");
			auto expected = fixture.Image();
			Require(expected[(64 * ImageSize + 64) * 4 + 1] > 0, "Lit reference did not render");
			fixture.Clear();
			auto overridden = plane("FF800000");
			View3D_ObjectColourBlendSet(overridden, 0xFF004000, nullptr);
			Require(expected == fixture.Image(), "Override was not applied before simple/PBR lighting");
		}

		// Existing alpha and K-buffer ordering must survive RGB replacement and reversed submission order.
		fixture.Clear();
		auto back = fixture.Quad(60, 0xFF0000FF);
		auto front = fixture.Quad(40, 0x80FF0000);
		auto const midpoint = 128.0f / 255;
		View3D_ObjectColourBlendSet(front, 0x8000FF00, nullptr);
		Expect(fixture.Image(), midpoint * (1 - midpoint), midpoint * midpoint, 1 - midpoint);
		View3D_ObjectColourBlendSet(front, 0xFF00FF00, nullptr);
		auto ordered = fixture.Image();
		Expect(ordered, 0, 128.0f / 255, 127.0f / 255);
		fixture.Clear();
		View3D_WindowAddObject(fixture.m_window, front);
		View3D_WindowAddObject(fixture.m_window, back);
		Require(ordered == fixture.Image(), "RGB override changed sorted alpha output");

		// Put the coloured target behind the camera so only a real reflected hit can show its RGB.
		if (View3D_WindowRayTracingInfoGet(fixture.m_window).m_available)
		{
			// The same target remains in the TLAS while its override changes, exercising reusable material-buffer updates.
			fixture.Clear();
			auto mirror = View3D_ObjectCreateLdrA("*Plane Mirror FF000000 {*Data{90 90} *Reflectivity{1} *o2w{*pos{0 0 -50}}}", FALSE, nullptr, nullptr);
			Require(mirror != nullptr, "Mirror creation failed");
			fixture.m_objects.push_back(mirror);
			View3D_WindowAddObject(fixture.m_window, mirror);
			auto reflected = fixture.Quad(-10, 0xFF000000);
			View3D_RayTracingPropertiesSet(fixture.m_window, {api::ERayTracingFeature::Reflections, 1});
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			Require(View3D_WindowRayTracingEnabledGet(fixture.m_window), "Ray tracing did not enable");
			for (auto alpha : {0U, 128U, 255U, 0U})
			{
				// The hidden target is unlit, so reflected RGB directly reveals the blend weight and transfer function.
				auto amount = alpha / 255.0f;
				View3D_ObjectColourBlendSet(reflected, (alpha << 24) | 0x00008000U, nullptr);
				Expect(fixture.Image(), 0, Linear(128) * amount, 0);
			}
			View3D_WindowRayTracingEnabledSet(fixture.m_window, FALSE);
		}
		else
		{
			// Hardware availability is reported rather than mistaking a raster-only pass for reflection coverage.
			std::cout << "SKIP colour blend reflected-hit test: DXR unavailable\n";
		}

		// Name selection and cloning operate on instance state, including model-less hierarchy roots.
		auto root = View3D_ObjectCreateLdrA("*Group Root {*Sphere Child {1} *Sphere Other {1}}", FALSE, nullptr, nullptr);
		Require(root != nullptr, "Hierarchy creation failed");
		fixture.m_objects.push_back(root);
		View3D_ObjectColourBlendSet(root, 0x80123456, nullptr);
		Require(View3D_ObjectColourBlendAmountGet(root, "Child") == 0, "Self selection affected a child");
		View3D_ObjectColourBlendSet(root, 0xFFABCDEF, "Child");
		Require(View3D_ObjectColourBlendAmountGet(root, nullptr) == midpoint, "Named selection affected root");
		Require(View3D_ObjectColourBlendAmountGet(root, "Other") == 0, "Named selection affected sibling");
		View3D_ObjectColourBlendSet(root, 0x80373737, "");
		auto clone = View3D_ObjectCreateInstance(root);
		Require(clone != nullptr, "Clone creation failed");
		fixture.m_objects.push_back(clone);
		Require(View3D_ObjectColourBlendAmountGet(clone, "Child") == midpoint, "Clone lost child override");
		View3D_ObjectColourBlendSet(clone, 0xFFFFFFFF, "");
		Require(View3D_ObjectColourBlendColourGet(root, "Child") == 0x80373737, "Clone mutated source override");
		View3D_ObjectResetColour(root, "");
		Require(View3D_ObjectColourBlendAmountGet(root, "Child") == midpoint, "Tint reset changed independent override");
		View3D_ObjectUpdate(root, L"*Group Root {}", api::EUpdateObject::Colour);
		Require(View3D_ObjectColourBlendAmountGet(root, nullptr) == 0, "Colour replacement did not reset override");
		fixture.CheckErrors();

		// Missing selections return the same disabled packed value as a new object.
		Require(View3D_ObjectColourBlendColourGet(root, "Missing") == 0, "Missing selection returned an active packed override");
		Require(View3D_ObjectColourBlendAmountGet(root, "Missing") == 0, "Missing selection returned an active weight");
		fixture.CheckDebugLayer();
		std::cout << "PASS packed colour blend identity/UNORM8 weights/linear RGB/opacity/sorting/scope/clone/reset: MSAA " << samples << '\n';
	}

	// Replace a custom-pixel scene with stock world geometry and a real sky without an intervening render.
	void SceneHandoffTests(int samples)
	{
		Fixture fixture(samples);
		for (auto cycle = 0; cycle != 2; ++cycle)
		{
			// Existing unsupported world pixels still block opt-in until their scene membership is removed.
			fixture.Fade(false);
			fixture.Clear();
			auto custom = fixture.Quad(89, 0xFFFF0000, 45, fixture.CustomShader(false));
			fixture.Image();
			Require(!View3D_FarClipFadePropertiesSet(fixture.m_window, api::FarClipFadeProps{TRUE,0.9f,0.99f}), "Existing custom pixel shader was accepted");
			Require(!fixture.m_errors.empty(), "Unsupported retained draw did not report its rejection");
			fixture.m_errors.clear();
			View3D_WindowRemoveObject(fixture.m_window, custom);
			fixture.Fade(true);

			// Sky creation and repeated toggling must preserve its exclusion with populated draw lists.
			auto sky = View3D_ObjectCreateProceduralSky("HandoffSky", {0.5f,0.3f,0.8f,0}, {1,0.95f,0.85f,1}, 1, nullptr);
			Require(sky != nullptr, "Handoff sky creation failed");
			fixture.m_objects.push_back(sky);
			View3D_WindowAddObject(fixture.m_window, sky);
			auto sky_image = fixture.Image();
			fixture.Fade(false);
			Require(sky_image == fixture.Image(), "Scene handoff changed sky pixels");
			fixture.Fade(true);

			// Both fading opaque and ordinary alpha layers reveal the excluded sky through the same resolve.
			fixture.Quad(94.5f, 0xFFFF0000);
			fixture.Quad(96.75f, 0x800000FF);
			auto pixel = sky_image.data() + (64 * ImageSize + 64) * 4;
			Expect(fixture.Image(), 0.5f + 0.4608f * Linear(pixel[0]), 0.4608f * Linear(pixel[1]), 0.0392f + 0.4608f * Linear(pixel[2]));
			fixture.CheckDebugLayer();
		}

		// Automatic depth ordering changes only on depth-policy transitions, with no-test overlays taking precedence.
		fixture.Fade(false);
		fixture.Clear();
		auto quad = fixture.Quad(89, 0xFFFFFFFF);
		auto default_group = View3D_ObjectSortGroupGet(quad, nullptr);
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZTest, TRUE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == api::ESortGroup::PostAlpha, "NoZTest must render after alpha");
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZWrite, TRUE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == api::ESortGroup::PostAlpha, "NoZWrite overrode NoZTest overlay ordering");
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZTest, FALSE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == api::ESortGroup::PreOpaques, "NoZWrite must render before opaques");
		View3D_ObjectFlagsSet(quad, api::ELdrFlags::NoZWrite, FALSE, nullptr);
		Require(View3D_ObjectSortGroupGet(quad, nullptr) == default_group, "Disabling depth overrides did not restore default ordering");
		fixture.CheckErrors();
		std::cout << "PASS repeated custom-scene/world handoff, real procedural sky overlap, and flag sort-group ownership: MSAA " << samples << '\n';
	}
}

// Run only the bounded far-clip fixture and return a failing process status for any mismatch.
int main(int argc, char const* const* argv)
{
	// Select the focused fixture before running any unrelated numeric or GPU cases.
	try
	{
		// The RGB blend selector deliberately excludes the pre-existing fade tests.
		if (argc == 2 && std::string_view(argv[1]) == "--colour-blend")
		{
			// Validate both ordinary and multisampled rendering with the same surface expectations.
			fade_tests::ColourBlendTests(1);
			fade_tests::ColourBlendTests(4);
			return 0;
		}
		if (argc == 2 && std::string_view(argv[1]) == "--procedural-surface")
		{
			// Run only the focused procedural GPU/readback evidence.
			fade_tests::ProceduralSurfaceTests();
			return 0;
		}
		if (argc == 2 && std::string_view(argv[1]) == "--solid-wire")
		{
			// Run only the focused SolidWire GPU/readback evidence.
			fade_tests::SolidWireTests();
			return 0;
		}
		fade_tests::Require(argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--numeric-only"), "Expected no arguments or --numeric-only");
		fade_tests::NumericTests();
		fade_tests::NormalTransformTests();
		if (argc == 2)
			return 0;

		fade_tests::RenderTests(1);
		fade_tests::RenderTests(4);
		fade_tests::SceneHandoffTests(1);
		fade_tests::SceneHandoffTests(4);
		return 0;
	}
	catch (std::exception const& error)
	{
		std::cerr << error.what() << '\n';
		return 1;
	}
}
