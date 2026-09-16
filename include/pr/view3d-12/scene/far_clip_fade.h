//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/sortkey.h"

namespace pr::rdr12
{
	// Sort groups are an extensible numeric range; background and post-alpha overlays are not world geometry.
	inline bool FarClipFadeApplies(ESortGroup group)
	{
		switch (group)
		{
			case ESortGroup::Skybox: { return false; }
			default: { return group < ESortGroup::PostAlpha; }
		}
	}

	// Opt-in forward-rendered world opacity, expressed as fractions of camera-forward far depth.
	struct FarClipFadeProps
	{
		bool m_enabled = false;
		float m_start_fraction = 0.9f;
		float m_end_fraction = 0.99f;

		// Reject invalid ranges without changing the current scene settings.
		void Validate() const
		{
			if (!std::isfinite(m_start_fraction) || !std::isfinite(m_end_fraction) ||
				m_start_fraction < 0.0f || m_start_fraction >= m_end_fraction || m_end_fraction >= 1.0f)
				throw std::invalid_argument("Far clip fade requires finite 0 <= start_fraction < end_fraction < 1");
		}

		// Resolve the fade interval before the hardware far plane, rejecting unrepresentable intervals.
		v2 DepthRange(float far_depth) const
		{
			Validate();
			auto range = v2{m_start_fraction * far_depth, m_end_fraction * far_depth};
			if (!std::isfinite(far_depth) || far_depth <= 0.0f || range.x >= range.y || range.y >= far_depth)
				throw std::invalid_argument("Far clip fade requires a finite positive far depth and a representable interval before it");

			return range;
		}

		// Compare all settings, including the range retained while disabled.
		friend bool operator == (FarClipFadeProps const&, FarClipFadeProps const&) = default;
	};
}
