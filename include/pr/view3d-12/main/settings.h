//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/config.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	/// <summary>Options</summary>
	enum class ERdrOptions
	{
		None           = 0,
		DeviceDebug    = 1 << 0,
		DeviceGPUDebug = 1 << 1 | DeviceDebug,
		BreakOnErrors  = 1 << 2 | DeviceDebug,
		BGRASupport    = 1 << 3,
		D2D1_DebugInfo = 1 << 4,

		_flags_enum = 0,
	};

	/// <summary>Settings for constructing the renderer</summary>
	struct RdrSettings
	{
		struct BuildOptions
		{
			StdBuildOptions m_std;
			int RunTimeShaders;
			BuildOptions()
				:m_std()
				,RunTimeShaders(PR_RDR_RUNTIME_SHADERS)
			{}
		};

		HINSTANCE             m_instance;      // Executable instance 
		BuildOptions          m_build_options; // The state of #defines. Used to check for incompatibilities
		D3D_FEATURE_LEVEL     m_feature_level; // Required feature level.
		ERdrOptions           m_options;       // Extra features to optionally support
		SystemConfig::Adapter m_adapter;       // The adapter to use.

		// Keep this inline so that m_build_options can be verified.
		explicit RdrSettings(HINSTANCE inst)
			:m_instance(inst)
			,m_build_options()
			,m_feature_level(D3D_FEATURE_LEVEL_11_0)
			,m_options(ERdrOptions::None)
			,m_adapter()
		{}

		// Enable the debug layer
		RdrSettings& DebugLayer(bool enable = true, bool gpu_debug = false, bool break_on_errors = true)
		{
			if (m_adapter.ptr != nullptr) Check(false, "DebugLayer must be enabled before setting the adapter (technically before creating the DXGI factory)");
			m_options = SetBits(m_options, ERdrOptions::DeviceDebug, enable);
			m_options = SetBits(m_options, ERdrOptions::DeviceGPUDebug, enable && gpu_debug);
			m_options = SetBits(m_options, ERdrOptions::BreakOnErrors, enable && break_on_errors);
			return *this;
		}

		// Select the default adaptor (Call after setting the debug layer)
		RdrSettings& DefaultAdapter()
		{
			SystemConfig cfg(AllSet(m_options, ERdrOptions::DeviceDebug));
			if (!cfg.adapters.empty())
				m_adapter = cfg.adapters[0];

			return *this;
		}
	};

	/// <summary>Settings for a window</summary>
	struct WndSettings
	{
		// Credit: https://www.rastertek.com/dx12tut03.html
		// Before we can initialize the swap chain we have to get the refresh rate from the video card/monitor.
		// Each computer may be slightly different so we will need to query for that information. We query for
		// the numerator and denominator values and then pass them to DirectX during the setup and it will calculate
		// the proper refresh rate. If we don't do this and just set the refresh rate to a default value which may
		// not exist on all computers then DirectX will respond by performing a buffer copy instead of a buffer flip
		// which will degrade performance and give us annoying errors in the debug output.

		HWND                  m_hwnd;             // The Win32 Windows handle. (Can be null for off-screen only rendering)
		RdrSettings const*    m_rdr_settings;     // The settings for the owning renderer instance
		SystemConfig::Output  m_output;           // The monitor to use, nullptr means use the default. See 'SystemConfig'
		BOOL                  m_windowed;         // Windowed mode or full screen
		DisplayMode           m_mode;             // Display mode to use (note: must be valid for the adapter, use FindClosestMatchingMode if needed)
		DXGI_SWAP_EFFECT      m_swap_effect;      // How to swap the back buffer to the front buffer
		DXGI_SWAP_CHAIN_FLAG  m_swap_chain_flags; // Options to allow GDI and DX together (see DXGI_SWAP_CHAIN_FLAG)
		DXGI_FORMAT           m_depth_format;     // Depth buffer format
		MultiSamp             m_multisamp;        // Number of samples per pixel (AA/Multi-sampling)
		Colour                m_bkgd_colour;      // The clear value colour for the window
		DXGI_USAGE            m_usage;            // Usage flags for the swap chain buffer
		DXGI_SCALING          m_scaling;          // 
		DXGI_ALPHA_MODE       m_alpha_mode;       //
		UINT                  m_buffer_count;     // Number of buffers in the chain, 1 = front only, 2 = front and back, 3 = triple buffering, etc
		UINT                  m_vsync;            // Present SyncInterval value
		bool                  m_xr_support;       // Is this window used for XR rendering
		bool                  m_use_w_buffer;     // Use W-Buffer depth rather than Z-Buffer
		bool                  m_allow_alt_enter;  // Allow switching to full screen with alt-enter
		string32              m_name;             // A debugging name for the window

		// Notes:
		// - VSync has different meaning for the swap effect modes.
		//   BitBlt modes: 0 = present immediately, 1,2,3,.. present after the nth vertical blank (has the effect of locking the frame rate to a fixed multiple of the VSync rate)
		//   Flip modes (Sequential): 0 = drop this frame if there is a new frame waiting, n > 0 = same as bitblt case

		WndSettings(HWND hwnd, bool windowed, RdrSettings const& rdr_settings)
			:m_hwnd(hwnd)
			,m_rdr_settings(&rdr_settings)
			,m_output()
			,m_windowed(windowed)
			,m_mode(iv2::Zero())
			,m_swap_effect(DXGI_SWAP_EFFECT_FLIP_DISCARD)
			,m_swap_chain_flags(DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH)
			,m_depth_format(DXGI_FORMAT_D32_FLOAT)
			,m_multisamp()
			,m_bkgd_colour(ColourBlack)
			,m_usage(DXGI_USAGE_RENDER_TARGET_OUTPUT|DXGI_USAGE_SHADER_INPUT)
			,m_scaling(DXGI_SCALING_STRETCH)
			,m_alpha_mode(DXGI_ALPHA_MODE_UNSPECIFIED)
			,m_buffer_count(2)
			,m_vsync(1)
			,m_xr_support(false)
			,m_use_w_buffer(true)
			,m_allow_alt_enter(false)
			,m_name()
		{
			DefaultOutput();
			DefaultMode();

			// Default to the window client area
			// Null hwnd is allowed when off-screen only rendering
			if (m_output.ptr != nullptr && hwnd != nullptr)
				Size(hwnd);
		}
		WndSettings& DefaultOutput()
		{
			if (!m_rdr_settings->m_adapter.outputs.empty())
				m_output = m_rdr_settings->m_adapter.outputs[0];
			
			return *this;
		}
		WndSettings& DefaultMode()
		{
			return m_windowed ? Size(1024, 768) : Mode(m_output.FindBestFullScreenMode());
		}
		WndSettings& Mode(DisplayMode const& mode)
		{
			m_mode = mode;
			return *this;
		}
		WndSettings& AllowAltEnter(bool allow)
		{
			m_allow_alt_enter = allow;
			return *this;
		}
		WndSettings& XrSupport(bool enable)
		{
			m_xr_support = enable;
			return *this;
		}
		WndSettings& Size(iv2 const& area)
		{
			Check(m_output.ptr != nullptr, "Set the output before setting the display mode");
			auto mode = m_output.FindClosestMatchingMode(DisplayMode(area));
			if (m_windowed) { mode.Width = area.x; mode.Height = area.y; }
			return Mode(mode);
		}
		WndSettings& Size(int w, int h)
		{
			return Size(iv2(w,h));
		}
		WndSettings& Size(HWND hwnd)
		{
			RECT rect;

			#if _WIN32_WINNT >= _WIN32_WINNT_WIN10
			auto dpi = GetDpiForWindow(hwnd);
			#else
			auto dpi = 96;
			#endif

			Check(::GetClientRect(hwnd, &rect), "GetClientRect failed.");
			return Size(iv2(
				(rect.right - rect.left) * dpi / 96,
				(rect.bottom - rect.top) * dpi / 96));
		}
		WndSettings& MutliSampling(int count)
		{
			m_multisamp = MultiSamp(count);
			return *this;
		}
		WndSettings& BackgroundColour(Colour colour)
		{
			m_bkgd_colour = colour;
			return *this;
		}
		WndSettings& BackgroundColour(Colour32 colour)
		{
			return BackgroundColour(Colour(colour));
		}
		WndSettings& UseWBuffer(bool use = true)
		{
			m_use_w_buffer = use;
			return *this;
		}
		WndSettings& GdiCompatible()
		{
			// Must use B8G8R8A8_UNORM for GDI compatibility
			m_mode.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
			
			// Make the swap chain GDI compatible
			m_swap_chain_flags |= DXGI_SWAP_CHAIN_FLAG_GDI_COMPATIBLE;
			
			// Also, multi-sampling isn't supported
			m_multisamp = MultiSamp();
			return *this;
		}
		WndSettings& Name(std::string_view name)
		{
			m_name = name;
			return *this;
		}
	};
}

