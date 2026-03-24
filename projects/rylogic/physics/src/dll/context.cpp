//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "physics/src/dll/context.h"

namespace pr::physics
{
	Context::Context(ReportErrorCB error_cb)
		: m_inits()
		, m_mutex()
		, m_error_cb(error_cb)
	{}

	Context::~Context()
	{}
}
