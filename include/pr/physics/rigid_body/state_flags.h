//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

namespace pr::physics
{
	enum class ERigidBodyStateFlags :int
	{
		None = 0,
		Static = 1 << 0,
		Sleeping = 1 << 1,
		_flags_enum = 0,
	};
}
