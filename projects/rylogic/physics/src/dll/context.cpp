//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "physics/src/dll/context.h"
#include "physics/src/dll/interop.h"

namespace pr::physics
{
	Context::Context(ReportErrorCB error_cb)
		: m_inits()
		, m_mutex()
		, m_error_cb(error_cb)
		, m_interop(new InteropState())
	{}

	Context::~Context()
	{}
}
