//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/physics-dll.h"

namespace pr::physics
{
	// An expected ABI failure. The ABI entry points convert it into its status code, so exceptions never cross the C boundary.
	struct ApiException : std::runtime_error
	{
		EStatus m_status;

		ApiException(EStatus status, std::string message)
			: std::runtime_error(std::move(message))
			, m_status(status)
		{}
	};

	// Copying cannot throw, so code that reports an exception while another is active can copy one without replacing the active exception.
	// The copy keeps the status; the C++ library may duplicate the message, and with MSVC a copy made when memory is exhausted has no message text.
	static_assert(std::is_nothrow_copy_constructible_v<ApiException> && std::is_nothrow_move_constructible_v<ApiException>);
}
