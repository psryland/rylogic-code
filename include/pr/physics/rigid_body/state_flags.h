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
		Sleeping = 1 << 0,

		_flags_enum = 0,
	};
}
