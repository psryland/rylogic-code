//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Dynamically load the WinPixEventRuntime.dll and call its functions
// This is so that if PIX is enabled, but the WinPixEventRuntime.dll is missing, the application will still run

#include "pr/compute/forward.h"
#include "pr/view3d-12/utility/pix.h"

#if PR_PIX_ENABLED
#pragma message (PR_LINK "warning : ************************************************* PIX Enabled")

using namespace pr::rdr12::pix;

namespace pr::rdr12::pix
{
	// Return the PIX dll module handle if it can be loaded
	inline HMODULE Dll()
	{
		static HMODULE s_module = ::LoadLibraryA("WinPixEventRuntime");
		return s_module;
	}

	// Return the PIX dll module handle if it can be loaded
	HMODULE LoadDll()
	{
		auto handle = Dll();
		if (PIXLoadLatestWinPixGpuCapturerLibrary() == 0) PR_INFO(PR_PIX_ENABLED, "WinPixGpuCapturerLibrary not found");
		if (PIXLoadLatestWinPixTimingCapturerLibrary() == 0) PR_INFO(PR_PIX_ENABLED, "WinPixTimingCapturerLibrary not found");
		return handle;
	}
}

extern "C" PIXEventsThreadInfo* WINAPI PIXGetThreadInfo() noexcept
{
	using PIXGetThreadInfoFn = PIXEventsThreadInfo * (WINAPI *)() noexcept;
	static PIXGetThreadInfoFn s_func = nullptr;
	static PIXEventsThreadInfo s_info = {};

	if (!s_func)
	{
		if (Dll())
			s_func = reinterpret_cast<PIXGetThreadInfoFn>(::GetProcAddress(Dll(), "PIXGetThreadInfo"));
		if (!s_func)
			return &s_info;
	}
	return s_func();
}

extern "C" UINT64 WINAPI PIXEventsReplaceBlock(PIXEventsThreadInfo* threadInfo, bool getEarliestTime) noexcept
{
	using PIXEventsReplaceBlockFn = UINT64(WINAPI *)(PIXEventsThreadInfo*, bool) noexcept;
	static PIXEventsReplaceBlockFn s_func = nullptr;

	if (!s_func)
	{
		if (Dll())
			s_func = reinterpret_cast<PIXEventsReplaceBlockFn>(::GetProcAddress(Dll(), "PIXEventsReplaceBlock"));
		if (!s_func)
			return 0;
	}
	return s_func(threadInfo, getEarliestTime);
}

#endif
