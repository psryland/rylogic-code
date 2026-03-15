//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// The physics DLL is a thin wrapper around the static library,
// providing a C-linkage API for use from other languages or as
// a dynamically loaded library.
#include "pr/physics/physics-dll.h"
#include "physics/src/dll/context.h"

using namespace pr::physics;

// DLL entry point
HINSTANCE g_hInstance;
BOOL APIENTRY DllMain(HMODULE hInstance, DWORD reason, LPVOID)
{
	switch (reason)
	{
		case DLL_PROCESS_ATTACH: g_hInstance = hInstance; break;
		case DLL_PROCESS_DETACH: g_hInstance = nullptr; break;
		case DLL_THREAD_ATTACH:  break;
		case DLL_THREAD_DETACH:  break;
	}
	return TRUE;
}

// Global DLL context
static Context* g_ctx = nullptr;
static Context& Dll()
{
	if (g_ctx) return *g_ctx;
	throw std::runtime_error("Physics not initialised");
}

// Helper macros for exception trapping in API functions
#define DllLockGuard LockGuard lock(Dll().m_mutex)

// Initialise calls are reference counted and must be matched with Shutdown calls.
PHYSICS_API DllHandle __stdcall Physics_Initialise(ReportErrorCB global_error_cb)
{
	try
	{
		// Create the dll context on the first call
		if (g_ctx == nullptr)
			g_ctx = new Context(global_error_cb);

		// Generate a unique handle per Initialise call, used to match up with Shutdown calls
		static DllHandle handles = nullptr;
		Dll().m_inits.insert(++handles);
		return handles;
	}
	catch (std::exception const& e)
	{
		global_error_cb(std::format("Failed to initialise Physics.\nReason: {}\n", e.what()).c_str(), "", 0, 0);
		return nullptr;
	}
	catch (...)
	{
		global_error_cb("Failed to initialise Physics.\nReason: An unknown exception occurred\n", "", 0, 0);
		return nullptr;
	}
}
PHYSICS_API void __stdcall Physics_Shutdown(DllHandle context)
{
	if (!g_ctx) return;

	Dll().m_inits.erase(context);
	if (!Dll().m_inits.empty())
		return;

	delete g_ctx;
	g_ctx = nullptr;
}
