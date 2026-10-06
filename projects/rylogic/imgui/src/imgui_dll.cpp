//********************************
// ImGui DLL for view3d-12
//  Copyright (c) Rylogic Ltd 2025
//********************************
// Implements the C-style API for the imgui DLL.
// All imgui types are contained within this DLL.
// This file does NOT include the client header (pr/view3d-12/imgui/imgui.h)
// to avoid type name collisions between imgui's ImGuiContext and ours.

// imgui implementation (unity build)
#include "imgui.cpp"
#include "imgui_draw.cpp"
#include "imgui_tables.cpp"
#include "imgui_widgets.cpp"
#include "imgui_demo.cpp"

// imgui platform/renderer backends
#include "imgui_impl_win32.cpp"
#include "imgui_impl_dx12.cpp"

// Standard library
#include <d3d12.h>
#include <dxgi1_4.h>
#include <mutex>
#include <vector>
#include <memory>
#include <string>
#include <string_view>
#include <stdexcept>
#include <format>

// View3D host bridge, for drawing into a View3D window's final overlay pass
#include "pr/view3d-12/view3d-ui-bridge.h"

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")

// Forward declaration from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Mirror the types from the client header so the ABI matches.
// These must be layout-compatible with the types in pr/view3d-12/imgui/imgui.h.
namespace dll
{
	namespace bridge = pr::view3d::ui;

	// Version of this DLL's exported functions. Must match the client header and the managed wrapper.
	constexpr std::uint32_t ApiVersion = 0x00020000U;

	struct InitArgs
	{
		ID3D12Device* m_device;
		ID3D12CommandQueue* m_cmd_queue;
		HWND m_hwnd;
		DXGI_FORMAT m_rtv_format;
		int m_num_frames_in_flight;
		float m_font_scale;
	};

	struct ErrorHandler
	{
		using FuncCB = void(*)(void*, char const* msg, size_t len);

		void* m_ctx;
		FuncCB m_cb;

		void operator()(std::string_view message) const
		{
			if (m_cb) m_cb(m_ctx, message.data(), message.size());
			else throw std::runtime_error(std::string(message));
		}
	};

	// Return the UNORM view format for an sRGB colour format, or 'format' itself when it is not sRGB.
	inline DXGI_FORMAT LinearFormat(DXGI_FORMAT format)
	{
		// ImGui colours are display values, so drawing through an sRGB view would brighten them.
		switch (format)
		{
			case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: return DXGI_FORMAT_R8G8B8A8_UNORM;
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB: return DXGI_FORMAT_B8G8R8A8_UNORM;
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB: return DXGI_FORMAT_B8G8R8X8_UNORM;
			default: return format;
		}
	}

	// The internal context holding all imgui state. Opaque to the client.
	struct Context
	{
		ImGuiContext* m_imgui_ctx;
		ID3D12Device* m_device;
		ID3D12CommandQueue* m_cmd_queue;    // Queue the renderer backend uses for texture uploads
		ID3D12DescriptorHeap* m_srv_heap;   // Shader-visible heap for the font texture SRV
		ID3D12DescriptorHeap* m_rtv_heap;   // One CPU-only RTV slot for drawing into a View3D window's target
		ErrorHandler m_error_cb;
		int m_num_frames_in_flight;

		// The render target format of the renderer backend. 'DXGI_FORMAT_UNKNOWN' until the backend is initialised.
		DXGI_FORMAT m_rtv_format;

		// True between EndFrame and the next NewFrame, while ImGui's draw data describes a complete frame.
		bool m_draw_data_ready;

		// Storage for 'io.IniFilename', which ImGui reads by pointer.
		std::string m_ini_filename;

		// The View3D window this context draws into, and the bridge function that detaches from it.
		void* m_window;
		bridge::DetachFn m_bridge_detach;

		// Display size override: when set, mouse coordinates from WndProc are
		// rescaled from client-rect space to this target space, and io.DisplaySize
		// is overridden in NewFrame. Fixes DPI mismatch between GetClientRect
		// (physical pixels) and the actual render target dimensions.
		float m_target_display_w;
		float m_target_display_h;

		// SRV descriptor callbacks for ImGui_ImplDX12_InitInfo
		static void SrvDescriptorAlloc(ImGui_ImplDX12_InitInfo* info, D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu)
		{
			*out_cpu = info->SrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart();
			*out_gpu = info->SrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart();
		}
		static void SrvDescriptorFree(ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_GPU_DESCRIPTOR_HANDLE)
		{
			// Nothing to do — we own the entire heap and release it in Cleanup
		}

		// Create the ImGui context. A 'DXGI_FORMAT_UNKNOWN' render target format defers renderer setup until AttachToWindow supplies the format.
		Context(InitArgs const& args, ErrorHandler error_cb)
			: m_imgui_ctx(nullptr)
			, m_device(args.m_device)
			, m_cmd_queue(nullptr)
			, m_srv_heap(nullptr)
			, m_rtv_heap(nullptr)
			, m_error_cb(error_cb)
			, m_num_frames_in_flight(args.m_num_frames_in_flight > 0 ? args.m_num_frames_in_flight : 3)
			, m_rtv_format(DXGI_FORMAT_UNKNOWN)
			, m_draw_data_ready(false)
			, m_ini_filename()
			, m_window(nullptr)
			, m_bridge_detach(nullptr)
			, m_target_display_w(0)
			, m_target_display_h(0)
		{
			// Release partial state on failure, because the destructor does not run for a constructor that throws.
			try
			{
				if (m_device == nullptr)
					throw std::runtime_error("ImGui requires a D3D12 device");

				m_device->AddRef();

				// Create a descriptor heap for imgui's font texture SRV
				D3D12_DESCRIPTOR_HEAP_DESC desc = {};
				desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
				desc.NumDescriptors = 1;
				desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
				auto hr = m_device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_srv_heap));
				if (FAILED(hr))
					throw std::runtime_error("Failed to create imgui descriptor heap");

				// Create the RTV slot used when drawing into a host-owned render target.
				D3D12_DESCRIPTOR_HEAP_DESC rtv_desc = {};
				rtv_desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
				rtv_desc.NumDescriptors = 1;
				rtv_desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
				hr = m_device->CreateDescriptorHeap(&rtv_desc, IID_PPV_ARGS(&m_rtv_heap));
				if (FAILED(hr))
					throw std::runtime_error("Failed to create imgui RTV heap");

				// Use the caller's queue for texture uploads, or a private direct queue when the caller has none.
				if (args.m_cmd_queue != nullptr)
				{
					m_cmd_queue = args.m_cmd_queue;
					m_cmd_queue->AddRef();
				}
				else
				{
					D3D12_COMMAND_QUEUE_DESC queue_desc = {};
					queue_desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
					hr = m_device->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&m_cmd_queue));
					if (FAILED(hr))
						throw std::runtime_error("Failed to create imgui command queue");
				}

				// Create imgui context
				m_imgui_ctx = ImGui::CreateContext();
				ImGui::SetCurrentContext(m_imgui_ctx);

				// Disable layout persistence until the caller chooses a file, because ImGui's default path is relative to the working directory.
				auto& io = ImGui::GetIO();
				io.IniFilename = nullptr;
				io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

				// Scale text and spacing together so the layout keeps its proportions.
				auto scale = args.m_font_scale > 0.0f ? args.m_font_scale : 1.0f;
				auto& style = ImGui::GetStyle();
				style.FontScaleMain = scale;
				style.ScaleAllSizes(scale);

				// Initialise platform backend
				ImGui_ImplWin32_Init(args.m_hwnd);

				// Initialise the renderer backend now if the target format is known
				if (args.m_rtv_format != DXGI_FORMAT_UNKNOWN)
					InitRenderer(args.m_rtv_format);
			}
			catch (...)
			{
				Cleanup();
				throw;
			}
		}
		~Context()
		{
			Cleanup();
		}

		// Initialise the DX12 renderer backend for render targets of 'rtv_format'.
		void InitRenderer(DXGI_FORMAT rtv_format)
		{
			// The pipeline state is built for one target format, so the backend is initialised once.
			ImGui_ImplDX12_InitInfo init_info = {};
			init_info.Device = m_device;
			init_info.CommandQueue = m_cmd_queue;
			init_info.NumFramesInFlight = m_num_frames_in_flight;
			init_info.RTVFormat = rtv_format;
			init_info.DSVFormat = DXGI_FORMAT_UNKNOWN;
			init_info.SrvDescriptorHeap = m_srv_heap;
			init_info.SrvDescriptorAllocFn = SrvDescriptorAlloc;
			init_info.SrvDescriptorFreeFn = SrvDescriptorFree;
			if (!ImGui_ImplDX12_Init(&init_info))
				throw std::runtime_error("Failed to initialise the imgui DX12 renderer");

			m_rtv_format = rtv_format;
		}

		// Release everything this context owns. Safe to call on a partially constructed context.
		void Cleanup()
		{
			// Stop the host calling back into a context that is being destroyed.
			if (m_window != nullptr && m_bridge_detach != nullptr)
				m_bridge_detach(m_window, this);

			m_window = nullptr;
			m_bridge_detach = nullptr;

			// Shut down the backends before the context. Destroying the context also saves the layout file.
			if (m_imgui_ctx)
			{
				ImGui::SetCurrentContext(m_imgui_ctx);
				if (m_rtv_format != DXGI_FORMAT_UNKNOWN)
					ImGui_ImplDX12_Shutdown();
				if (ImGui::GetIO().BackendPlatformUserData != nullptr)
					ImGui_ImplWin32_Shutdown();

				ImGui::DestroyContext(m_imgui_ctx);
				m_imgui_ctx = nullptr;
				m_rtv_format = DXGI_FORMAT_UNKNOWN;
			}

			// Release D3D objects last, after the backend has released its own references.
			auto release = [](auto*& p)
			{
				// Release and clear one COM pointer
				if (p == nullptr)
					return;

				p->Release();
				p = nullptr;
			};
			release(m_rtv_heap);
			release(m_srv_heap);
			release(m_cmd_queue);
			release(m_device);
		}
	};

	// Run 'fn' with 'ctx' as the current ImGui context, reporting exceptions through the context's error handler.
	template <typename Fn>
	auto Call(Context& ctx, Fn&& fn) -> decltype(fn())
	{
		using Result = decltype(fn());
		try
		{
			ImGui::SetCurrentContext(ctx.m_imgui_ctx);
			return fn();
		}
		catch (std::exception const& ex)
		{
			ctx.m_error_cb(ex.what());
			return Result();
		}
	}

	// Bridge callback that draws the last completed ImGui frame into the host's final overlay.
	bridge::EHostStatus __stdcall RecordThunk(void* context, bridge::Pass const* pass) noexcept
	{
		auto& ctx = *static_cast<Context*>(context);
		try
		{
			// Screen-space windows draw only in the final overlay, above all other host output.
			if (pass == nullptr)
				return bridge::EHostStatus::InvalidArgument;

			switch (pass->m_pass)
			{
				case bridge::EPass::FinalOverlay:
				{
					break;
				}
				case bridge::EPass::Prepare:
				case bridge::EPass::DepthTested:
				case bridge::EPass::OcclusionFaded:
				case bridge::EPass::Overlay:
				{
					return bridge::EHostStatus::Success;
				}
				default:
				{
					return bridge::EHostStatus::InvalidArgument;
				}
			}

			// The first pass supplies the target format the renderer pipeline needs. Drawing starts with the next frame.
			ImGui::SetCurrentContext(ctx.m_imgui_ctx);
			auto rtv_format = LinearFormat(pass->m_colour_format);
			if (ctx.m_rtv_format == DXGI_FORMAT_UNKNOWN)
			{
				ctx.InitRenderer(rtv_format);
				return bridge::EHostStatus::Success;
			}
			if (ctx.m_rtv_format != rtv_format)
				throw std::runtime_error("The View3D window's render target format changed after ImGui was initialised");

			// Draw only a complete frame. A frame that has started but not ended has no valid draw data.
			auto draw_data = ImGui::GetDrawData();
			if (!ctx.m_draw_data_ready || draw_data == nullptr || !draw_data->Valid)
				return bridge::EHostStatus::Success;

			// Draw through a private view of the host target. RTV descriptors are read when they are bound, so one slot can be reused every frame.
			auto rtv = ctx.m_rtv_heap->GetCPUDescriptorHandleForHeapStart();
			auto rtv_desc = D3D12_RENDER_TARGET_VIEW_DESC{ .Format = rtv_format, .ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D };
			ctx.m_device->CreateRenderTargetView(const_cast<ID3D12Resource*>(pass->m_colour_target), &rtv_desc, rtv);

			// Bind the target and ImGui's descriptor heap, then record the draw lists. The backend sets its own viewport and scissors.
			auto cmd_list = pass->m_command_list;
			cmd_list->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
			ID3D12DescriptorHeap* heaps[] = { ctx.m_srv_heap };
			cmd_list->SetDescriptorHeaps(1, heaps);
			ImGui_ImplDX12_RenderDrawData(draw_data, cmd_list);
			return bridge::EHostStatus::Success;
		}
		catch (std::exception const& ex)
		{
			// No exception may cross into the host, including one thrown by an error handler without a callback.
			try { ctx.m_error_cb(ex.what()); } catch (...) {}
			return bridge::EHostStatus::ProviderFailed;
		}
	}

	// Bridge callback for a host that destroys its window while this context is still attached.
	void __stdcall DetachedThunk(void* context) noexcept
	{
		// The host has already removed the provider, so the context must not detach again.
		auto& ctx = *static_cast<Context*>(context);
		ctx.m_window = nullptr;
		ctx.m_bridge_detach = nullptr;
	}
}

// DLL global state
static std::mutex g_mutex;
static std::vector<std::unique_ptr<dll::Context>> g_contexts;
static HINSTANCE g_instance;

extern "C"
{
	BOOL APIENTRY DllMain(HINSTANCE hInstance, DWORD ul_reason_for_call, LPVOID)
	{
		(void)ul_reason_for_call;
		g_instance = hInstance;
		return TRUE;
	}

	using namespace dll;

	// Return the version of this DLL's exported functions
	__declspec(dllexport) std::uint32_t __stdcall ImGui_ApiVersion()
	{
		return dll::ApiVersion;
	}

	// Create a dll context
	__declspec(dllexport) Context* __stdcall ImGui_Initialise(InitArgs const& args, ErrorHandler error_cb)
	{
		try
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			g_contexts.push_back(std::make_unique<Context>(args, error_cb));
			return g_contexts.back().get();
		}
		catch (std::exception const& ex)
		{
			error_cb(ex.what());
			return nullptr;
		}
	}

	// Release a dll context
	__declspec(dllexport) void __stdcall ImGui_Shutdown(Context* ctx)
	{
		try
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			auto it = std::remove_if(g_contexts.begin(), g_contexts.end(), [ctx](auto const& p) { return p.get() == ctx; });
			g_contexts.erase(it, g_contexts.end());
		}
		catch (std::exception const& ex)
		{
			ctx->m_error_cb(ex.what());
		}
	}

	// Draw this context's frames into the final overlay of a View3D window. 'window' is a View3D window handle from view3d-12.dll.
	__declspec(dllexport) bool __stdcall ImGui_AttachToWindow(Context& ctx, void* window)
	{
		return Call(ctx, [&]
		{
			// Resolve the bridge from the already-loaded View3D module, checking it uses the same ABI as this DLL.
			if (window == nullptr)
				throw std::runtime_error("AttachToWindow requires a View3D window");
			if (ctx.m_window != nullptr)
				throw std::runtime_error("This ImGui context is already attached to a View3D window");

			auto module = ::GetModuleHandleW(L"view3d-12.dll");
			if (module == nullptr)
				throw std::runtime_error("view3d-12.dll is not loaded");

			auto api_version = reinterpret_cast<bridge::ApiVersionFn>(::GetProcAddress(module, bridge::ApiVersionExport));
			auto struct_size = reinterpret_cast<bridge::StructSizeFn>(::GetProcAddress(module, bridge::StructSizeExport));
			auto attach = reinterpret_cast<bridge::AttachFn>(::GetProcAddress(module, bridge::AttachExport));
			auto detach = reinterpret_cast<bridge::DetachFn>(::GetProcAddress(module, bridge::DetachExport));
			if (api_version == nullptr || struct_size == nullptr || attach == nullptr || detach == nullptr)
				throw std::runtime_error("view3d-12.dll does not export the UI host bridge");
			if (api_version() != bridge::HostApiVersion)
				throw std::runtime_error(std::format("view3d-12.dll UI host bridge version {:08X} does not match the expected version {:08X}", api_version(), bridge::HostApiVersion));

			auto provider_size = std::uint32_t{};
			auto pass_size = std::uint32_t{};
			if (struct_size(bridge::EHostStructId::Provider, &provider_size) != bridge::EHostStatus::Success || provider_size != sizeof(bridge::Provider) ||
				struct_size(bridge::EHostStructId::Pass, &pass_size) != bridge::EHostStatus::Success || pass_size != sizeof(bridge::Pass))
				throw std::runtime_error("view3d-12.dll UI host bridge structures do not match this DLL");

			// Attach as a provider identified by this context.
			auto provider = bridge::Provider{
				.m_header = {sizeof(bridge::Provider), bridge::HostStructVersion},
				.m_context = &ctx,
				.m_record = RecordThunk,
				.m_detached = DetachedThunk,
			};
			auto status = attach(window, &provider);
			if (status != bridge::EHostStatus::Success)
				throw std::runtime_error(std::format("Attaching ImGui to the View3D window failed with status {}", static_cast<int>(status)));

			ctx.m_window = window;
			ctx.m_bridge_detach = detach;
			return true;
		});
	}

	// Stop drawing into the attached View3D window. Does nothing if the context is not attached.
	__declspec(dllexport) void __stdcall ImGui_DetachFromWindow(Context& ctx)
	{
		Call(ctx, [&]
		{
			// Clear the attachment even if the host rejects the detach, so it is never repeated.
			if (ctx.m_window == nullptr)
				return;

			auto window = ctx.m_window;
			auto detach = ctx.m_bridge_detach;
			ctx.m_window = nullptr;
			ctx.m_bridge_detach = nullptr;
			auto status = detach(window, &ctx);
			if (status != bridge::EHostStatus::Success)
				throw std::runtime_error(std::format("Detaching ImGui from the View3D window failed with status {}", static_cast<int>(status)));
		});
	}

	// Set the file that stores window positions, sizes, and collapsed state. Null or empty disables persistence. Call before the first NewFrame.
	__declspec(dllexport) void __stdcall ImGui_SetIniFilename(Context& ctx, char const* path)
	{
		Call(ctx, [&]
		{
			// ImGui keeps the pointer, so the context owns the string.
			ctx.m_ini_filename = path != nullptr ? path : "";
			ImGui::GetIO().IniFilename = ctx.m_ini_filename.empty() ? nullptr : ctx.m_ini_filename.c_str();
		});
	}

	// Start a new imgui frame. Returns false, and starts no frame, while the renderer is waiting for its first View3D pass.
	__declspec(dllexport) bool __stdcall ImGui_NewFrame(Context& ctx)
	{
		return Call(ctx, [&]
		{
			// A deferred renderer has no pipeline until the attached window has rendered once.
			if (ctx.m_rtv_format == DXGI_FORMAT_UNKNOWN)
				return false;

			ctx.m_draw_data_ready = false;
			ImGui_ImplDX12_NewFrame();
			ImGui_ImplWin32_NewFrame();

			// Override display size if a target has been set. This must happen
			// after Win32 backend (which sets DisplaySize from GetClientRect)
			// but before ImGui::NewFrame (which processes input events).
			if (ctx.m_target_display_w > 0 && ctx.m_target_display_h > 0)
			{
				auto& io = ImGui::GetIO();
				io.DisplaySize = ImVec2(ctx.m_target_display_w, ctx.m_target_display_h);
			}

			ImGui::NewFrame();
			return true;
		});
	}

	// Finish the current frame so the attached View3D window draws it in its next render.
	__declspec(dllexport) void __stdcall ImGui_EndFrame(Context& ctx)
	{
		Call(ctx, [&]
		{
			// ImGui::Render produces the draw data that the bridge callback records.
			ImGui::Render();
			ctx.m_draw_data_ready = true;
		});
	}

	// Finish the current frame and record it into 'cmd_list'. For callers that do not attach to a View3D window.
	__declspec(dllexport) void __stdcall ImGui_Render(Context& ctx, ID3D12GraphicsCommandList* cmd_list)
	{
		Call(ctx, [&]
		{
			// The caller has already bound its render target.
			ImGui::Render();
			ctx.m_draw_data_ready = true;

			// Set the descriptor heap for imgui's font texture
			ID3D12DescriptorHeap* heaps[] = { ctx.m_srv_heap };
			cmd_list->SetDescriptorHeaps(1, heaps);

			ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), cmd_list);
		});
	}

	// Forward a window message to imgui. Returns true if imgui handled it.
	__declspec(dllexport) bool __stdcall ImGui_WndProc(Context& ctx, HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
	{
		return Call(ctx, [&]
		{
			// Rescale mouse coordinates from client-rect space to target display space
			if (ctx.m_target_display_w > 0 && ctx.m_target_display_h > 0)
			{
				if (msg == WM_MOUSEMOVE || msg == WM_LBUTTONDOWN || msg == WM_LBUTTONUP || msg == WM_LBUTTONDBLCLK ||
					msg == WM_RBUTTONDOWN || msg == WM_RBUTTONUP || msg == WM_RBUTTONDBLCLK ||
					msg == WM_MBUTTONDOWN || msg == WM_MBUTTONUP || msg == WM_MBUTTONDBLCLK ||
					msg == WM_XBUTTONDOWN || msg == WM_XBUTTONUP || msg == WM_XBUTTONDBLCLK)
				{
					RECT rect;
					if (::GetClientRect(hwnd, &rect) && rect.right > 0 && rect.bottom > 0)
					{
						float sx = ctx.m_target_display_w / (float)(rect.right - rect.left);
						float sy = ctx.m_target_display_h / (float)(rect.bottom - rect.top);
						int x = (int)(GET_X_LPARAM(lparam) * sx);
						int y = (int)(GET_Y_LPARAM(lparam) * sy);
						lparam = MAKELPARAM(x, y);
					}
				}
			}

			auto result = ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam);
			return result != 0;
		});
	}

	// True when imgui uses the mouse in the current frame, so the application should ignore mouse input.
	__declspec(dllexport) bool __stdcall ImGui_WantCaptureMouse(Context& ctx)
	{
		return Call(ctx, [&] { return ImGui::GetIO().WantCaptureMouse; });
	}

	// True when imgui uses the keyboard in the current frame, so the application should ignore keyboard input.
	__declspec(dllexport) bool __stdcall ImGui_WantCaptureKeyboard(Context& ctx)
	{
		return Call(ctx, [&] { return ImGui::GetIO().WantCaptureKeyboard; });
	}

	// Set the target display size. Mouse coordinates in WndProc will be rescaled
	// from client-rect space to this target, and io.DisplaySize will be overridden
	// in NewFrame. Pass (0, 0) to disable the override and use GetClientRect directly.
	__declspec(dllexport) void __stdcall ImGui_SetDisplaySize(Context& ctx, float w, float h)
	{
		ctx.m_target_display_w = w;
		ctx.m_target_display_h = h;
	}

	// Windows

	__declspec(dllexport) bool __stdcall ImGui_BeginWindow(Context& ctx, char const* name, bool* p_open, int flags)
	{
		return Call(ctx, [&] { return ImGui::Begin(name, p_open, static_cast<ImGuiWindowFlags>(flags)); });
	}
	__declspec(dllexport) void __stdcall ImGui_EndWindow(Context& ctx)
	{
		Call(ctx, [&] { ImGui::End(); });
	}
	__declspec(dllexport) void __stdcall ImGui_SetNextWindowPos(Context& ctx, float x, float y, int cond)
	{
		Call(ctx, [&] { ImGui::SetNextWindowPos(ImVec2(x, y), static_cast<ImGuiCond>(cond)); });
	}
	__declspec(dllexport) void __stdcall ImGui_SetNextWindowSize(Context& ctx, float w, float h, int cond)
	{
		Call(ctx, [&] { ImGui::SetNextWindowSize(ImVec2(w, h), static_cast<ImGuiCond>(cond)); });
	}
	__declspec(dllexport) void __stdcall ImGui_SetNextWindowBgAlpha(Context& ctx, float alpha)
	{
		Call(ctx, [&] { ImGui::SetNextWindowBgAlpha(alpha); });
	}

	// Menus

	__declspec(dllexport) bool __stdcall ImGui_BeginMainMenuBar(Context& ctx)
	{
		return Call(ctx, [&] { return ImGui::BeginMainMenuBar(); });
	}
	__declspec(dllexport) void __stdcall ImGui_EndMainMenuBar(Context& ctx)
	{
		Call(ctx, [&] { ImGui::EndMainMenuBar(); });
	}
	__declspec(dllexport) bool __stdcall ImGui_BeginMenu(Context& ctx, char const* label, bool enabled)
	{
		return Call(ctx, [&] { return ImGui::BeginMenu(label, enabled); });
	}
	__declspec(dllexport) void __stdcall ImGui_EndMenu(Context& ctx)
	{
		Call(ctx, [&] { ImGui::EndMenu(); });
	}
	__declspec(dllexport) bool __stdcall ImGui_MenuItem(Context& ctx, char const* label, char const* shortcut, bool* selected, bool enabled)
	{
		return Call(ctx, [&] { return ImGui::MenuItem(label, shortcut, selected, enabled); });
	}

	// Layout

	__declspec(dllexport) void __stdcall ImGui_SameLine(Context& ctx, float offset_from_start_x, float spacing)
	{
		Call(ctx, [&] { ImGui::SameLine(offset_from_start_x, spacing); });
	}
	__declspec(dllexport) void __stdcall ImGui_Separator(Context& ctx)
	{
		Call(ctx, [&] { ImGui::Separator(); });
	}
	__declspec(dllexport) void __stdcall ImGui_SeparatorText(Context& ctx, char const* label)
	{
		Call(ctx, [&] { ImGui::SeparatorText(label); });
	}
	__declspec(dllexport) void __stdcall ImGui_Spacing(Context& ctx)
	{
		Call(ctx, [&] { ImGui::Spacing(); });
	}
	__declspec(dllexport) void __stdcall ImGui_SetNextItemWidth(Context& ctx, float width)
	{
		Call(ctx, [&] { ImGui::SetNextItemWidth(width); });
	}
	__declspec(dllexport) void __stdcall ImGui_PushID(Context& ctx, char const* id)
	{
		Call(ctx, [&] { ImGui::PushID(id); });
	}
	__declspec(dllexport) void __stdcall ImGui_PopID(Context& ctx)
	{
		Call(ctx, [&] { ImGui::PopID(); });
	}
	__declspec(dllexport) void __stdcall ImGui_BeginDisabled(Context& ctx, bool disabled)
	{
		Call(ctx, [&] { ImGui::BeginDisabled(disabled); });
	}
	__declspec(dllexport) void __stdcall ImGui_EndDisabled(Context& ctx)
	{
		Call(ctx, [&] { ImGui::EndDisabled(); });
	}
	__declspec(dllexport) bool __stdcall ImGui_CollapsingHeader(Context& ctx, char const* label, int flags)
	{
		return Call(ctx, [&] { return ImGui::CollapsingHeader(label, static_cast<ImGuiTreeNodeFlags>(flags)); });
	}

	// Text

	__declspec(dllexport) void __stdcall ImGui_Text(Context& ctx, char const* text)
	{
		Call(ctx, [&] { ImGui::TextUnformatted(text); });
	}
	__declspec(dllexport) void __stdcall ImGui_TextDisabled(Context& ctx, char const* text)
	{
		Call(ctx, [&] { ImGui::TextDisabled("%s", text); });
	}
	__declspec(dllexport) void __stdcall ImGui_TextWrapped(Context& ctx, char const* text)
	{
		Call(ctx, [&] { ImGui::TextWrapped("%s", text); });
	}
	__declspec(dllexport) void __stdcall ImGui_TextColored(Context& ctx, float r, float g, float b, float a, char const* text)
	{
		Call(ctx, [&] { ImGui::TextColored(ImVec4(r, g, b, a), "%s", text); });
	}
	__declspec(dllexport) void __stdcall ImGui_SetItemTooltip(Context& ctx, char const* text)
	{
		Call(ctx, [&] { ImGui::SetItemTooltip("%s", text); });
	}

	// Widgets

	__declspec(dllexport) bool __stdcall ImGui_Button(Context& ctx, char const* label)
	{
		return Call(ctx, [&] { return ImGui::Button(label); });
	}
	__declspec(dllexport) bool __stdcall ImGui_Checkbox(Context& ctx, char const* label, bool* v)
	{
		return Call(ctx, [&] { return ImGui::Checkbox(label, v); });
	}
	__declspec(dllexport) bool __stdcall ImGui_SliderFloat(Context& ctx, char const* label, float* v, float v_min, float v_max, char const* format, int flags)
	{
		return Call(ctx, [&] { return ImGui::SliderFloat(label, v, v_min, v_max, format, static_cast<ImGuiSliderFlags>(flags)); });
	}
	__declspec(dllexport) bool __stdcall ImGui_SliderInt(Context& ctx, char const* label, int* v, int v_min, int v_max, char const* format, int flags)
	{
		return Call(ctx, [&] { return ImGui::SliderInt(label, v, v_min, v_max, format, static_cast<ImGuiSliderFlags>(flags)); });
	}
	__declspec(dllexport) bool __stdcall ImGui_DragFloat(Context& ctx, char const* label, float* v, float speed, float v_min, float v_max, char const* format, int flags)
	{
		return Call(ctx, [&] { return ImGui::DragFloat(label, v, speed, v_min, v_max, format, static_cast<ImGuiSliderFlags>(flags)); });
	}
	__declspec(dllexport) bool __stdcall ImGui_InputFloat(Context& ctx, char const* label, float* v, float step, float step_fast, char const* format, int flags)
	{
		return Call(ctx, [&] { return ImGui::InputFloat(label, v, step, step_fast, format, static_cast<ImGuiInputTextFlags>(flags)); });
	}
	__declspec(dllexport) bool __stdcall ImGui_InputText(Context& ctx, char const* label, char* buf, size_t buf_size, int flags)
	{
		return Call(ctx, [&] { return ImGui::InputText(label, buf, buf_size, static_cast<ImGuiInputTextFlags>(flags)); });
	}
	__declspec(dllexport) bool __stdcall ImGui_Combo(Context& ctx, char const* label, int* current_item, char const* items_separated_by_zeros, int popup_max_height_in_items)
	{
		return Call(ctx, [&] { return ImGui::Combo(label, current_item, items_separated_by_zeros, popup_max_height_in_items); });
	}
	__declspec(dllexport) void __stdcall ImGui_PlotLines(Context& ctx, char const* label, float const* values, int values_count, int values_offset, char const* overlay_text, float scale_min, float scale_max, float graph_w, float graph_h)
	{
		Call(ctx, [&] { ImGui::PlotLines(label, values, values_count, values_offset, overlay_text, scale_min, scale_max, ImVec2(graph_w, graph_h)); });
	}
}
