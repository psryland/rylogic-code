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
		explicit Fixture(int samples)
		{
			m_hwnd = CreateWindowExW(0, L"STATIC", L"Far clip fade test", WS_POPUP, 0, 0, ImageSize, ImageSize, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
			Require(m_hwnd != nullptr, "CreateWindowEx failed");
			m_context = View3D_Initialise({&m_errors, ReportError});
			Require(m_context != nullptr, "View3D_Initialise failed");
			m_window = View3D_WindowCreate(m_hwnd, api::WindowOptions().error_cb({&m_errors, ReportError}).back_colour(0xFF000000).multisamp(samples).name("FarClipFadeTests"));
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
		api::Object Quad(float depth, unsigned colour, float half_width = 45, api::Shader shader = nullptr, float right_depth = 0)
		{
			api::Vertex verts[] =
			{
				{{-half_width,-45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
				{{+half_width,-45,-(right_depth != 0 ? right_depth : depth),1}, {}, {}, 0xFFFFFFFF, 0},
				{{+half_width,+45,-(right_depth != 0 ? right_depth : depth),1}, {}, {}, 0xFFFFFFFF, 0},
				{{-half_width,+45,-depth,1}, {}, {}, 0xFFFFFFFF, 0},
			};
			UINT16 indices[] = {0,1,2,0,2,3};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::TriList;
			nugget.m_geom = api::EGeom::Vert;
			nugget.m_cull_mode = api::ECullMode::None;
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
	try
	{
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
