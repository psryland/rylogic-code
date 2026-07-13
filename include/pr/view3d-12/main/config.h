//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Create an instance of this object to enumerate the adapters and their outputs on the current system.
	// Note: modes are not enumerated because they depend on DXGI_FORMAT. Users should create a SystemConfig,
	// then call 'GetDisplayModes' for the format needed.
	struct SystemConfig
	{
		using DisplayMode = ::pr::compute::DisplayMode;

		// An output of a graphics adapter (i.e. a monitor)
		struct Output
		{
			D3DPtr<IDXGIOutput> ptr;
			DXGI_OUTPUT_DESC desc;
				
			Output()
				:ptr()
				,desc()
			{}
			Output(D3DPtr<IDXGIOutput>& output)
				:ptr(std::move(output))
				,desc()
			{
				Check(ptr->GetDesc(&desc));
			}

			// Return the number of modes for a given surface format
			UINT ModeCount(DXGI_FORMAT format) const
			{
				UINT mode_count = 0;
				Check(ptr->GetDisplayModeList(format, 0, &mode_count, nullptr));
				return mode_count;
			}

			// Populate a list of display modes for the given format
			pr::vector<DisplayMode, 8> DisplayModes(DXGI_FORMAT format) const
			{
				auto mode_count = ModeCount(format);
				
				pr::vector<DisplayMode, 8> modes(mode_count);
				if (!modes.empty())
					Check(ptr->GetDisplayModeList(format, 0, &mode_count, modes.data()));

				return modes;
			}

			// Return the best match for the given mode
			DisplayMode FindClosestMatchingMode(DisplayMode const& ideal) const
			{
				DisplayMode closest;
				Check(ptr->FindClosestMatchingMode(&ideal, &closest, nullptr));
				return closest;
			}

			// Return a full screen mode that is at least 60Hz
			DisplayMode FindBestFullScreenMode() const
			{
				auto monitor_info = MONITORINFOEXW{ {.cbSize = sizeof(MONITORINFOEXW)} };
				Check(GetMonitorInfoW(desc.Monitor, &monitor_info));
  
				auto dev_mode = DEVMODEW{
					.dmSize = sizeof(DEVMODEW),
					.dmDriverExtra = 0,
				};
				Check(EnumDisplaySettingsW(monitor_info.szDevice, ENUM_CURRENT_SETTINGS, &dev_mode));

				auto mode = DisplayMode(dev_mode.dmPelsWidth, dev_mode.dmPelsHeight, DXGI_FORMAT_R8G8B8A8_UNORM);
				if (dev_mode.dmDisplayFrequency == 1 || dev_mode.dmDisplayFrequency == 0) mode.default_refresh_rate();
				else mode.refresh_rate(dev_mode.dmDisplayFrequency, 1);
				return FindClosestMatchingMode(mode);
			}
		};
		
		// A graphics adapter on the system
		struct Adapter
		{
			D3DPtr<IDXGIAdapter1> ptr;
			std::vector<Output> outputs;
			DXGI_ADAPTER_DESC1 desc;

			// Constructs a representation of a graphics adapter including its supported modes
			Adapter()
				: ptr()
				, outputs()
				, desc()
			{}
			Adapter(D3DPtr<IDXGIAdapter1>& adapter)
				: ptr(std::move(adapter))
			{
				// Read the description
				Check(ptr->GetDesc1(&desc));

				// Enumerate the outputs
				D3DPtr<IDXGIOutput> output;
				for (UINT i = 0; ptr->EnumOutputs(i, output.address_of()) != DXGI_ERROR_NOT_FOUND; ++i)
					outputs.emplace_back(output);
			}

			// True if this adapter can create a D3D12 device at 'feature_level' and supports at least 'shader_model'.
			// Note: the basic display adapter used in some VMs can create a device but does not support the shader
			// models (SM6+) that the stock shaders are compiled to, so a shader-model check is needed as well.
			bool Supports(D3D_FEATURE_LEVEL feature_level, D3D_SHADER_MODEL shader_model)
			{
				if (ptr == nullptr)
					return false;

				// The adapter must be able to create a D3D12 device at the required feature level
				D3DPtr<ID3D12Device> device;
				if (D3D12CreateDevice(ptr.get(), feature_level, __uuidof(ID3D12Device), (void**)device.address_of()) != S_OK)
					return false;

				// ...and support at least the required shader model. Query the highest supported model
				// (descending until the runtime accepts the query, as newer SDK enum values may be unknown to the driver).
				constexpr D3D_SHADER_MODEL versions[] =
				{
					D3D_HIGHEST_SHADER_MODEL,
					D3D_SHADER_MODEL_6_7, D3D_SHADER_MODEL_6_6, D3D_SHADER_MODEL_6_5, D3D_SHADER_MODEL_6_4,
					D3D_SHADER_MODEL_6_3, D3D_SHADER_MODEL_6_2, D3D_SHADER_MODEL_6_1, D3D_SHADER_MODEL_6_0,
					D3D_SHADER_MODEL_5_1,
				};
				D3D12_FEATURE_DATA_SHADER_MODEL sm = {};
				for (auto v : versions)
				{
					sm.HighestShaderModel = v;
					auto hr = device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &sm, sizeof(sm));
					if (hr == E_INVALIDARG) continue;
					Check(hr);
					break;
				}
				return sm.HighestShaderModel >= shader_model;
			}
		};

		// The DXGI factory used to enumerate adapters (kept so the WARP adapter can be created on demand)
		D3DPtr<IDXGIFactory4> factory;

		// Adapters on the system
		std::vector<Adapter> adapters;

		explicit SystemConfig(bool with_debug_layer)
			:factory()
			,adapters()
		{
			// Create a DXGIFactory
			pr::Check(CreateDXGIFactory2(with_debug_layer ? DXGI_CREATE_FACTORY_DEBUG : 0, __uuidof(IDXGIFactory4), (void**)factory.address_of()));

			// Enumerate each adapter on the system (this includes the software  warp adapter)
			D3DPtr<IDXGIAdapter1> adapter;
			for (UINT i = 0; factory->EnumAdapters1(i, (IDXGIAdapter1**)adapter.address_of()) != DXGI_ERROR_NOT_FOUND; ++i)
				adapters.emplace_back(adapter);
		}

		// Return the WARP (software) adapter, or an empty adapter if it is unavailable.
		// WARP is a software rasterizer that supports modern feature levels and shader models,
		// useful as a fallback on machines without a hardware DX12 adapter (e.g. inside a VM).
		Adapter WarpAdapter() const
		{
			D3DPtr<IDXGIAdapter1> warp;
			if (factory == nullptr || factory->EnumWarpAdapter(__uuidof(IDXGIAdapter1), (void**)warp.address_of()) != S_OK)
				return Adapter();
			return Adapter(warp);
		}
	};
}

