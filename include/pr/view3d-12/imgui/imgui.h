//********************************
// ImGui integration for view3d-12
//  Copyright (c) Rylogic Ltd 2025
//********************************
// Notes:
//  - This provides a DLL-based integration of Dear ImGui with the view3d-12 renderer.
//  - All imgui types are hidden within the DLL. The client sees only a C-style API.
//  - To avoid making this a build dependency, this header dynamically loads 'imgui.dll' as needed.
//  - The DLL manages its own imgui context, descriptor heap, and pipeline state objects.
//  - A context can either record into a caller-provided command list (Render), or attach to a View3D window (AttachToWindow)
//    and draw each completed frame (EndFrame) into that window's final overlay pass.
#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <stdexcept>
#include <cassert>
#include <cfloat>
#include "pr/win32/win32.h"

// Forward declarations for D3D12 types (no d3d12.h dependency here)
struct ID3D12Device;
struct ID3D12CommandQueue;
struct ID3D12GraphicsCommandList;

namespace pr::rdr12::imgui
{
	// Version of the DLL's exported functions that this header expects
	constexpr std::uint32_t ApiVersion = 0x00020000U;

	// Opaque DLL context handle. Defined within the DLL.
	struct Context;

	// Initialisation parameters passed to the DLL
	struct InitArgs
	{
		ID3D12Device* m_device;          // D3D12 device
		ID3D12CommandQueue* m_cmd_queue; // Direct queue used for texture uploads. Null creates a private queue.
		HWND m_hwnd;                     // Window handle for input
		DXGI_FORMAT m_rtv_format;        // Render target format. 'DXGI_FORMAT_UNKNOWN' takes the format from the attached View3D window.
		int m_num_frames_in_flight;      // Number of buffered frames. Zero or less uses 3.
		float m_font_scale;              // Global scale for text and spacing. Zero or less uses 1.
	};

	// Error handling callback
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

	// Dynamically loaded ImGui DLL
	class ImGuiDll
	{
		friend struct ImGuiUI;
		HMODULE m_module;

		#define PR_IMGUI_API(x)\
		x(, ApiVersion         , std::uint32_t (__stdcall*)())\
		x(, Initialise         , Context* (__stdcall*)(InitArgs const& args, ErrorHandler error_cb))\
		x(, Shutdown           , void (__stdcall*)(Context* ctx))\
		x(, AttachToWindow     , bool (__stdcall*)(Context& ctx, void* window))\
		x(, DetachFromWindow   , void (__stdcall*)(Context& ctx))\
		x(, SetIniFilename     , void (__stdcall*)(Context& ctx, char const* path))\
		x(, NewFrame           , bool (__stdcall*)(Context& ctx))\
		x(, EndFrame           , void (__stdcall*)(Context& ctx))\
		x(, Render             , void (__stdcall*)(Context& ctx, ID3D12GraphicsCommandList* cmd_list))\
		x(, WndProc            , bool (__stdcall*)(Context& ctx, HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam))\
		x(, WantCaptureMouse   , bool (__stdcall*)(Context& ctx))\
		x(, WantCaptureKeyboard, bool (__stdcall*)(Context& ctx))\
		x(, SetDisplaySize     , void (__stdcall*)(Context& ctx, float w, float h))\
		x(, BeginWindow        , bool (__stdcall*)(Context& ctx, char const* name, bool* p_open, int flags))\
		x(, EndWindow          , void (__stdcall*)(Context& ctx))\
		x(, SetNextWindowPos   , void (__stdcall*)(Context& ctx, float x, float y, int cond))\
		x(, SetNextWindowSize  , void (__stdcall*)(Context& ctx, float w, float h, int cond))\
		x(, SetNextWindowBgAlpha, void (__stdcall*)(Context& ctx, float alpha))\
		x(, BeginMainMenuBar   , bool (__stdcall*)(Context& ctx))\
		x(, EndMainMenuBar     , void (__stdcall*)(Context& ctx))\
		x(, BeginMenu          , bool (__stdcall*)(Context& ctx, char const* label, bool enabled))\
		x(, EndMenu            , void (__stdcall*)(Context& ctx))\
		x(, MenuItem           , bool (__stdcall*)(Context& ctx, char const* label, char const* shortcut, bool* selected, bool enabled))\
		x(, SameLine           , void (__stdcall*)(Context& ctx, float offset_from_start_x, float spacing))\
		x(, Separator          , void (__stdcall*)(Context& ctx))\
		x(, SeparatorText      , void (__stdcall*)(Context& ctx, char const* label))\
		x(, Spacing            , void (__stdcall*)(Context& ctx))\
		x(, SetNextItemWidth   , void (__stdcall*)(Context& ctx, float width))\
		x(, PushID             , void (__stdcall*)(Context& ctx, char const* id))\
		x(, PopID              , void (__stdcall*)(Context& ctx))\
		x(, BeginDisabled      , void (__stdcall*)(Context& ctx, bool disabled))\
		x(, EndDisabled        , void (__stdcall*)(Context& ctx))\
		x(, CollapsingHeader   , bool (__stdcall*)(Context& ctx, char const* label, int flags))\
		x(, Text               , void (__stdcall*)(Context& ctx, char const* text))\
		x(, TextDisabled       , void (__stdcall*)(Context& ctx, char const* text))\
		x(, TextWrapped        , void (__stdcall*)(Context& ctx, char const* text))\
		x(, TextColored        , void (__stdcall*)(Context& ctx, float r, float g, float b, float a, char const* text))\
		x(, SetItemTooltip     , void (__stdcall*)(Context& ctx, char const* text))\
		x(, Button             , bool (__stdcall*)(Context& ctx, char const* label))\
		x(, Checkbox           , bool (__stdcall*)(Context& ctx, char const* label, bool* v))\
		x(, SliderFloat        , bool (__stdcall*)(Context& ctx, char const* label, float* v, float v_min, float v_max, char const* format, int flags))\
		x(, SliderInt          , bool (__stdcall*)(Context& ctx, char const* label, int* v, int v_min, int v_max, char const* format, int flags))\
		x(, DragFloat          , bool (__stdcall*)(Context& ctx, char const* label, float* v, float speed, float v_min, float v_max, char const* format, int flags))\
		x(, InputFloat         , bool (__stdcall*)(Context& ctx, char const* label, float* v, float step, float step_fast, char const* format, int flags))\
		x(, InputText          , bool (__stdcall*)(Context& ctx, char const* label, char* buf, size_t buf_size, int flags))\
		x(, Combo              , bool (__stdcall*)(Context& ctx, char const* label, int* current_item, char const* items_separated_by_zeros, int popup_max_height_in_items))\
		x(, PlotLines          , void (__stdcall*)(Context& ctx, char const* label, float const* values, int values_count, int values_offset, char const* overlay_text, float scale_min, float scale_max, float graph_w, float graph_h))
		#define PR_IMGUI_FUNCTION_MEMBERS(prefix, name, function_type) using prefix##name##Fn = function_type; prefix##name##Fn prefix##name = {};
		PR_IMGUI_API(PR_IMGUI_FUNCTION_MEMBERS)
		#undef PR_IMGUI_FUNCTION_MEMBERS

		ImGuiDll()
			: m_module(win32::LoadDll<struct ImGuiDllTag>("imgui.dll"))
		{
			// Resolve every export, then reject a DLL built for a different version of this header
			#pragma warning(push)
			#pragma warning(disable: 4191)
			#define PR_IMGUI_GET_PROC_ADDRESS(prefix, name, function_type) prefix##name = reinterpret_cast<prefix##name##Fn>(GetProcAddress(m_module, "ImGui_" #prefix #name));
			PR_IMGUI_API(PR_IMGUI_GET_PROC_ADDRESS)
			#undef PR_IMGUI_GET_PROC_ADDRESS
			#pragma warning(pop)
			if (ApiVersion == nullptr || ApiVersion() != imgui::ApiVersion)
				throw std::runtime_error("imgui.dll does not match the version of pr/view3d-12/imgui/imgui.h");
		}

		static ImGuiDll& get() { static ImGuiDll s_this; return s_this; }
	};

	// RAII wrapper for the imgui DLL context. This is the client-side API.
	struct ImGuiUI
	{
		Context* m_ctx;

		ImGuiUI()
			: m_ctx()
		{
		}
		ImGuiUI(InitArgs const& args, ErrorHandler error_cb = {})
			: m_ctx(ImGuiDll::get().Initialise(args, error_cb))
		{
		}
		ImGuiUI(ImGuiUI&& rhs) noexcept
			: m_ctx()
		{
			std::swap(m_ctx, rhs.m_ctx);
		}
		ImGuiUI(ImGuiUI const&) = delete;
		ImGuiUI& operator=(ImGuiUI&& rhs) noexcept
		{
			if (this != &rhs) std::swap(m_ctx, rhs.m_ctx);
			return *this;
		}
		ImGuiUI& operator=(ImGuiUI const&) = delete;
		~ImGuiUI()
		{
			if (m_ctx)
				ImGuiDll::get().Shutdown(m_ctx);
		}

		// True if the context is valid and ready for use
		explicit operator bool() const
		{
			return m_ctx != nullptr;
		}

		// Draw completed frames into the final overlay of a View3D window. The GPU must be idle for the window before Shutdown.
		bool AttachToWindow(void* window)
		{
			assert(m_ctx != nullptr && "ImGuiUI not initialised");
			return ImGuiDll::get().AttachToWindow(*m_ctx, window);
		}
		void DetachFromWindow()
		{
			assert(m_ctx != nullptr && "ImGuiUI not initialised");
			ImGuiDll::get().DetachFromWindow(*m_ctx);
		}

		// Set the layout persistence file. Null or empty disables persistence. Call before the first NewFrame.
		void SetIniFilename(char const* path)
		{
			assert(m_ctx != nullptr && "ImGuiUI not initialised");
			ImGuiDll::get().SetIniFilename(*m_ctx, path);
		}

		// Start a new imgui frame. Returns false, and starts no frame, while an attached renderer waits for its first View3D pass.
		bool NewFrame()
		{
			assert(m_ctx != nullptr && "ImGuiUI not initialised");
			return ImGuiDll::get().NewFrame(*m_ctx);
		}

		// Finish the current frame for drawing by the attached View3D window
		void EndFrame()
		{
			assert(m_ctx != nullptr && "ImGuiUI not initialised");
			ImGuiDll::get().EndFrame(*m_ctx);
		}

		// Finish the current frame and record it into the command list
		void Render(ID3D12GraphicsCommandList* cmd_list)
		{
			assert(m_ctx != nullptr && "ImGuiUI not initialised");
			ImGuiDll::get().Render(*m_ctx, cmd_list);
		}

		// Forward a Win32 message to imgui. Returns true if imgui consumed the message.
		bool WndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
		{
			if (!m_ctx) return false;
			return ImGuiDll::get().WndProc(*m_ctx, hwnd, msg, wparam, lparam);
		}

		// True when imgui uses the mouse or keyboard, so the application should ignore that input
		bool WantCaptureMouse() const
		{
			return m_ctx != nullptr && ImGuiDll::get().WantCaptureMouse(*m_ctx);
		}
		bool WantCaptureKeyboard() const
		{
			return m_ctx != nullptr && ImGuiDll::get().WantCaptureKeyboard(*m_ctx);
		}

		// Override the display size to match the actual render target dimensions.
		// Fixes DPI mismatches between GetClientRect and the swap chain. Takes effect in the next NewFrame.
		void SetDisplaySize(float w, float h)
		{
			ImGuiDll::get().SetDisplaySize(*m_ctx, w, h);
		}

		// Windows
		bool BeginWindow(char const* name, bool* p_open = nullptr, int flags = 0)
		{
			return ImGuiDll::get().BeginWindow(*m_ctx, name, p_open, flags);
		}
		void EndWindow()
		{
			ImGuiDll::get().EndWindow(*m_ctx);
		}
		void SetNextWindowPos(float x, float y, int cond = 0)
		{
			ImGuiDll::get().SetNextWindowPos(*m_ctx, x, y, cond);
		}
		void SetNextWindowSize(float w, float h, int cond = 0)
		{
			ImGuiDll::get().SetNextWindowSize(*m_ctx, w, h, cond);
		}
		void SetNextWindowBgAlpha(float alpha)
		{
			ImGuiDll::get().SetNextWindowBgAlpha(*m_ctx, alpha);
		}

		// Menus
		bool BeginMainMenuBar()
		{
			return ImGuiDll::get().BeginMainMenuBar(*m_ctx);
		}
		void EndMainMenuBar()
		{
			ImGuiDll::get().EndMainMenuBar(*m_ctx);
		}
		bool BeginMenu(char const* label, bool enabled = true)
		{
			return ImGuiDll::get().BeginMenu(*m_ctx, label, enabled);
		}
		void EndMenu()
		{
			ImGuiDll::get().EndMenu(*m_ctx);
		}
		bool MenuItem(char const* label, char const* shortcut = nullptr, bool* selected = nullptr, bool enabled = true)
		{
			return ImGuiDll::get().MenuItem(*m_ctx, label, shortcut, selected, enabled);
		}

		// Layout
		void SameLine(float offset_from_start_x = 0.0f, float spacing = -1.0f)
		{
			ImGuiDll::get().SameLine(*m_ctx, offset_from_start_x, spacing);
		}
		void Separator()
		{
			ImGuiDll::get().Separator(*m_ctx);
		}
		void SeparatorText(char const* label)
		{
			ImGuiDll::get().SeparatorText(*m_ctx, label);
		}
		void Spacing()
		{
			ImGuiDll::get().Spacing(*m_ctx);
		}
		void SetNextItemWidth(float width)
		{
			ImGuiDll::get().SetNextItemWidth(*m_ctx, width);
		}
		void PushID(char const* id)
		{
			ImGuiDll::get().PushID(*m_ctx, id);
		}
		void PopID()
		{
			ImGuiDll::get().PopID(*m_ctx);
		}
		void BeginDisabled(bool disabled = true)
		{
			ImGuiDll::get().BeginDisabled(*m_ctx, disabled);
		}
		void EndDisabled()
		{
			ImGuiDll::get().EndDisabled(*m_ctx);
		}
		bool CollapsingHeader(char const* label, int flags = 0)
		{
			return ImGuiDll::get().CollapsingHeader(*m_ctx, label, flags);
		}

		// Text
		void Text(char const* text)
		{
			ImGuiDll::get().Text(*m_ctx, text);
		}
		void TextDisabled(char const* text)
		{
			ImGuiDll::get().TextDisabled(*m_ctx, text);
		}
		void TextWrapped(char const* text)
		{
			ImGuiDll::get().TextWrapped(*m_ctx, text);
		}
		void TextColored(float r, float g, float b, float a, char const* text)
		{
			ImGuiDll::get().TextColored(*m_ctx, r, g, b, a, text);
		}
		void SetItemTooltip(char const* text)
		{
			ImGuiDll::get().SetItemTooltip(*m_ctx, text);
		}

		// Widgets
		bool Button(char const* label)
		{
			return ImGuiDll::get().Button(*m_ctx, label);
		}
		bool Checkbox(char const* label, bool* v)
		{
			return ImGuiDll::get().Checkbox(*m_ctx, label, v);
		}
		bool SliderFloat(char const* label, float* v, float v_min, float v_max, char const* format = "%.3f", int flags = 0)
		{
			return ImGuiDll::get().SliderFloat(*m_ctx, label, v, v_min, v_max, format, flags);
		}
		bool SliderInt(char const* label, int* v, int v_min, int v_max, char const* format = "%d", int flags = 0)
		{
			return ImGuiDll::get().SliderInt(*m_ctx, label, v, v_min, v_max, format, flags);
		}
		bool DragFloat(char const* label, float* v, float speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, char const* format = "%.3f", int flags = 0)
		{
			return ImGuiDll::get().DragFloat(*m_ctx, label, v, speed, v_min, v_max, format, flags);
		}
		bool InputFloat(char const* label, float* v, float step = 0.0f, float step_fast = 0.0f, char const* format = "%.3f", int flags = 0)
		{
			return ImGuiDll::get().InputFloat(*m_ctx, label, v, step, step_fast, format, flags);
		}
		bool InputText(char const* label, char* buf, size_t buf_size, int flags = 0)
		{
			return ImGuiDll::get().InputText(*m_ctx, label, buf, buf_size, flags);
		}
		bool Combo(char const* label, int* current_item, char const* items_separated_by_zeros, int popup_max_height_in_items = -1)
		{
			return ImGuiDll::get().Combo(*m_ctx, label, current_item, items_separated_by_zeros, popup_max_height_in_items);
		}
		void PlotLines(char const* label, float const* values, int values_count, int values_offset = 0, char const* overlay_text = nullptr, float scale_min = FLT_MAX, float scale_max = FLT_MAX, float graph_w = 0, float graph_h = 0)
		{
			ImGuiDll::get().PlotLines(*m_ctx, label, values, values_count, values_offset, overlay_text, scale_min, scale_max, graph_w, graph_h);
		}
	};
}
