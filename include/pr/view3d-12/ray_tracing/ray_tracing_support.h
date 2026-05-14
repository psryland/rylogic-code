//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/main/settings.h"
#include "pr/view3d-12/utility/features.h"

namespace pr::rdr12
{
	// Captures whether ray tracing was requested and whether the current device can support it.
	struct RayTracingSupport
	{
		bool m_requested;
		D3D12_RAYTRACING_TIER m_tier;

		// Create a disabled ray tracing capability state.
		RayTracingSupport();

		// Create ray tracing capability state from renderer options and device feature support.
		RayTracingSupport(ERdrOptions options, FeatureSupport const& features);

		// Return true if the renderer was created with ray tracing support requested.
		bool Requested() const;

		// Return true if the current D3D12 device reports a supported ray tracing tier.
		bool HardwareSupported() const;

		// Return true if ray tracing was requested and the current device supports it.
		bool Available() const;

		// Return the device ray tracing tier as a short diagnostic string.
		char const* TierName() const;
	};
}
