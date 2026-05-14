//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	// Create an instance of this object to enumerate the adapters and their outputs on the current system.
	// Note: modes are not enumerated because they depend on DXGI_FORMAT. Users should create a SystemConfig,
	// then call 'GetDisplayModes' for the format needed.
	struct SystemConfig
	{
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
		};

		// Adapters on the system
		std::vector<Adapter> adapters;

		explicit SystemConfig(bool with_debug_layer)
			:adapters()
		{
			// Create a DXGIFactory
			D3DPtr<IDXGIFactory4> factory;
			pr::Check(CreateDXGIFactory2(with_debug_layer ? DXGI_CREATE_FACTORY_DEBUG : 0, __uuidof(IDXGIFactory4), (void**)factory.address_of()));

			// Enumerate each adapter on the system (this includes the software  warp adapter)
			D3DPtr<IDXGIAdapter1> adapter;
			for (UINT i = 0; factory->EnumAdapters1(i, (IDXGIAdapter1**)adapter.address_of()) != DXGI_ERROR_NOT_FOUND; ++i)
				adapters.emplace_back(adapter);
		}
	};
}

