//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_support.h"

namespace pr::rdr12
{
	RayTracingSupport::RayTracingSupport()
		:m_requested(false)
		,m_tier(D3D12_RAYTRACING_TIER_NOT_SUPPORTED)
	{}

	RayTracingSupport::RayTracingSupport(ERdrOptions options, FeatureSupport const& features)
		:m_requested(AllSet(options, ERdrOptions::RayTracingSupport))
		,m_tier(features.Options5.RaytracingTier)
	{}

	bool RayTracingSupport::Requested() const
	{
		return m_requested;
	}

	bool RayTracingSupport::HardwareSupported() const
	{
		return m_tier != D3D12_RAYTRACING_TIER_NOT_SUPPORTED;
	}

	bool RayTracingSupport::Available() const
	{
		return Requested() && HardwareSupported();
	}

	char const* RayTracingSupport::TierName() const
	{
		switch (m_tier)
		{
			case D3D12_RAYTRACING_TIER_NOT_SUPPORTED:
			{
				return "not supported";
			}
			case D3D12_RAYTRACING_TIER_1_0:
			{
				return "tier 1.0";
			}
			case D3D12_RAYTRACING_TIER_1_1:
			{
				return "tier 1.1";
			}
			default:
			{
				return "unknown";
			}
		}
	}
}
