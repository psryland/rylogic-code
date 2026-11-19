//*********************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/physics-dll.h"

namespace pr::physics
{
	// An expected ABI failure converted to its status code at the C boundary.
	struct ApiException :std::runtime_error
	{
		EStatus m_status;

		// Associate the status with the diagnostic returned by this call.
		ApiException(EStatus status, std::string message)
			: std::runtime_error(std::move(message))
			, m_status(status)
		{}
	};

	// A copy made while handling another exception must not replace it. MSVC may lose message text under allocation failure.
	static_assert(std::is_nothrow_copy_constructible_v<ApiException> && std::is_nothrow_move_constructible_v<ApiException>);
}
