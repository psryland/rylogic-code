#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	void Require(bool condition, char const* message)
	{
		if (!condition)
			throw std::runtime_error(message);
	}

	void Check(HRESULT result)
	{
		if (FAILED(result))
			throw std::runtime_error("D3D operation failed: " + std::to_string(result));
	}

	SceneLighting SceneLightingGet(api::Window window)
	{
		// Light 0 is the window's main light.
		return SceneLighting{ View3D_LightGet(window, 0), View3D_AmbientGet(window) };
	}

	void SceneLightingSet(api::Window window, SceneLighting const& lighting)
	{
		// Apply the light and ambient as one logical lighting state.
		View3D_LightSet(window, 0, lighting);
		View3D_AmbientSet(window, lighting.m_ambient);
	}

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

	float Linear(unsigned char value)
	{
		auto channel = value / 255.0f;
		return channel <= 0.04045f ? channel / 12.92f : std::pow((channel + 0.055f) / 1.055f, 2.4f);
	}

	Fixture::Fixture(int samples)
	{
		m_hwnd = CreateWindowExW(0, L"STATIC", L"View3d render test", WS_POPUP, 0, 0, ImageSize, ImageSize, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
		Require(m_hwnd != nullptr, "CreateWindowEx failed");
		m_context = View3D_Initialise({&m_errors, ReportError});
		Require(m_context != nullptr, "View3D_Initialise failed");
		m_window = View3D_WindowCreate(m_hwnd, api::WindowOptions().error_cb({&m_errors, ReportError}).back_colour(0xFF000000).multisamp(samples).name("RenderTests"));
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

	Fixture::~Fixture()
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

	void Fixture::CheckErrors()
	{
		if (m_errors.empty())
			return;

		throw std::runtime_error(m_errors.front());
	}

	api::Object Fixture::Quad(float depth, unsigned colour, float half_width, api::Shader shader, float right_depth, unsigned vertex_colour, bool textured, bool normals)
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
		nugget.m_tint = colour;
		if (shader != nullptr)
			nugget.m_shaders[0] = api::Nugget::Shader{shader, api::ERenderStep::ForwardRender, 0};

		auto object = View3D_ObjectCreate("TestQuad", 0xFFFFFFFF, 4, 6, 1, verts, indices, &nugget, GUID{});
		Require(object != nullptr, "Quad creation failed");
		m_objects.push_back(object);
		View3D_WindowAddObject(m_window, object);
		CheckErrors();
		return object;
	}

	api::Object Fixture::QuadRect(float x0, float x1, float depth)
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

	void Fixture::Clear()
	{
		View3D_WindowRemoveAllObjects(m_window);
	}

	void Fixture::AttachOverlay()
	{
		auto attach = reinterpret_cast<ui::AttachFn>(GetProcAddress(GetModuleHandleW(L"view3d-12.dll"), ui::AttachExport));
		Require(attach != nullptr, "UI bridge missing");
		ui::Provider provider{{sizeof(ui::Provider), ui::HostStructVersion}, this, Overlay, nullptr};
		Require(attach(m_window, &provider) == ui::EHostStatus::Success, "UI bridge attach failed");
	}

	api::Shader Fixture::CustomShader(bool vertex_only)
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

	std::vector<unsigned char> Fixture::Image()
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

	void Fixture::CheckDebugLayer()
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

	void Expect(std::vector<unsigned char> const& image, float red, float green, float blue, int x, int y)
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
}
