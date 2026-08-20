//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// The physics DLL is a thin wrapper around the static library,
// providing a C-linkage API for use from other languages or as
// a dynamically loaded library.
#include "pr/physics/physics-dll.h"
#include "physics/src/dll/context.h"
#include "physics/src/dll/interop.h"

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

// Context lookup and replacement are serialized independently of per-context API state.
static std::mutex g_context_mutex;
static std::shared_ptr<Context> g_ctx;
std::shared_ptr<Context> pr::physics::PinDll()
{
	auto lock = std::lock_guard{g_context_mutex};
	if (!g_ctx)
		throw std::runtime_error("Physics not initialised");

	return g_ctx;
}
std::shared_ptr<Context> pr::physics::TryPinDll()
{
	auto lock = std::lock_guard{g_context_mutex};
	return g_ctx;
}

// Initialise calls are reference counted and must be matched with Shutdown calls.
PHYSICS_API DllHandle __stdcall Physics_Initialise(ReportErrorCB global_error_cb)
{
	try
	{
		// Serialize first-context construction and registration so concurrent initialisers share one context.
		auto context_lock = std::lock_guard{g_context_mutex};
		if (!g_ctx)
			g_ctx = std::make_shared<Context>(global_error_cb);

		// Generate a unique non-null token without pointer arithmetic on a fabricated pointer.
		static std::uintptr_t next_handle = 0;
		auto handle = reinterpret_cast<DllHandle>(++next_handle);
		auto state_lock = LockGuard{g_ctx->m_mutex};
		g_ctx->m_inits.insert(handle);
		return handle;
	}
	catch (std::exception const& e)
	{
		if (global_error_cb)
			global_error_cb(std::format("Failed to initialise Physics.\nReason: {}\n", e.what()).c_str(), "", 0, 0);
		return nullptr;
	}
	catch (...)
	{
		if (global_error_cb)
			global_error_cb("Failed to initialise Physics.\nReason: An unknown exception occurred\n", "", 0, 0);
		return nullptr;
	}
}
PHYSICS_API void __stdcall Physics_Shutdown(DllHandle context)
{
	auto live_engines = false;
	auto invalid_handle = false;
	auto ctx = std::shared_ptr<Context>{};
	{
		// Keep the context pinned while validating the token and deciding whether final teardown is safe.
		auto context_lock = std::lock_guard{g_context_mutex};
		ctx = g_ctx;
		if (!ctx)
			return;

		auto state_lock = LockGuard{ctx->m_mutex};
		if (!ctx->m_inits.contains(context))
		{
			invalid_handle = true;
		}
		else if (ctx->m_inits.size() == 1 && ctx->m_interop->HasLiveEngines())
		{
			live_engines = true;
		}
		else
		{
			ctx->m_inits.erase(context);
			if (ctx->m_inits.empty())
				g_ctx.reset();
		}
	}

	// Report contract violations after releasing lifecycle locks so callbacks may safely re-enter the API.
	if (invalid_handle && ctx->m_error_cb)
		ctx->m_error_cb("Physics_Shutdown received an invalid context handle", "", 0, 0);
	if (live_engines && ctx->m_error_cb)
		ctx->m_error_cb("Physics_Shutdown cannot release the final context while engines are still alive", "", 0, 0);
}
