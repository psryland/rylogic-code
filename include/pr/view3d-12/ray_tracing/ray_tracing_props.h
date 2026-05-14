//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Ray tracing render settings for a scene.
	struct RayTracingProps
	{
		static constexpr int MinReflectionBounces = 1;
		static constexpr int MaxReflectionBounces = 4;

		ERayTracingFeature m_features;
		int m_max_reflection_bounces;

		// Create default ray tracing render settings.
		RayTracingProps()
			: m_features(ERayTracingFeature::All)
			, m_max_reflection_bounces(MinReflectionBounces)
		{
		}

		// Clamp tunable properties to their supported runtime ranges.
		void Clamp()
		{
			m_max_reflection_bounces = std::clamp(m_max_reflection_bounces, MinReflectionBounces, MaxReflectionBounces);
		}

		// True if 'lhs' and 'rhs' have identical values.
		friend bool operator == (RayTracingProps const& lhs, RayTracingProps const& rhs)
		{
			return
				lhs.m_features == rhs.m_features &&
				lhs.m_max_reflection_bounces == rhs.m_max_reflection_bounces;
		}
	};
}
