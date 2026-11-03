//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once

#include "pr/physics/forward.h"
#include "pr/physics/shape/inertia.h"
#include "pr/collision/collision.h"

namespace pr::physics
{
	// Extrapolate the position based on the given velocity and acceleration (about the CoM, all in the same space).
	// The accuracy of this extrapolation decreases with larger angular velocity or greater 'dt'.
	inline m4x4 ExtrapolateO2W(m4x4 const& o2w, v4 com_os, v8motion const& velocity, v8motion const& acceleration, float dt)
	{
		// S = So + Vt + 0.5At²
		auto dx = dt * velocity + (0.5f * dt * dt) * acceleration;

		// CoM-based position update: translate CoM, derive model origin from new rotation
		auto com_ws = o2w.pos + o2w.rot * com_os;
		auto new_rot = m3x3::Rotation(dx.ang.xyz) * o2w.rot;
		auto new_com = com_ws + dx.lin;
		auto new_pos = new_com - new_rot * com_os;

		return m4x4{new_rot, new_pos};
	}
}