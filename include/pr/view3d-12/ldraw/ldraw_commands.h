//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#pragma once
#include "pr/view3d-12/ldraw/ldraw.h"

namespace pr::rdr12::ldraw
{
	// LDraw commands - These must be POD types
	struct alignas(16) Command_Invalid
	{
		ECommandId m_id;
		uint8_t pad[12];
	};
	struct alignas(16) Command_Clear
	{
		ECommandId m_id;
		uint8_t pad[12];
	};
	struct alignas(16) Command_ObjectToWorld
	{
		ECommandId m_id;
		char m_obj_addr[60];
		m4x4 m_o2w;
	};
	struct alignas(16) Command_ObjectColour
	{
		ECommandId m_id;
		char m_obj_addr[60];
		Colour32 m_col;
		uint8_t pad[12];
	};
	struct alignas(16) Command_Render
	{
		ECommandId m_id;
		uint8_t pad[12];
	};
}
