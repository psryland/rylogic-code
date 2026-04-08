//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#pragma once
#include "pr/view3d-12/ldraw/ldraw.h"

namespace pr::rdr12::ldraw
{
	struct CommandHeader
	{
		// The type of command this is.
		alignas(16) ECommandId m_id;

		// When to execute this command in relation to the objects in the source output.
		// i.e. if 'm_exe_index' == 0, execute this command before any objects are processed.
		// If 'm_exe_index' == 1, execute this command after the first object is added, etc.
		int m_exe_index;

		int pad0;
		int pad1;
	};

	// LDraw commands - These must be POD types
	struct Command_Invalid
	{
		CommandHeader m_hdr;
	};
	struct Command_Clear
	{
		CommandHeader m_hdr;
	};
	struct Command_ObjectToWorld
	{
		CommandHeader m_hdr;
		char m_obj_addr[64];
		m4x4 m_o2w;
	};
	struct Command_ObjectColour
	{
		CommandHeader m_hdr;
		char m_obj_addr[64];
		Colour32 m_col;
		int pad0;
		int pad1;
		int pad2;
	};
	struct Command_Render
	{
		CommandHeader m_hdr;
	};
}
