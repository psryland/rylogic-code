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
		case DLL_PROCESS_ATTACH: g_hInstance = hInstance; pr::ApplyAssertReportingFromEnvironment(); break;
		case DLL_PROCESS_DETACH: g_hInstance = nullptr; break;
		case DLL_THREAD_ATTACH:  break;
		case DLL_THREAD_DETACH:  break;
	}
	return TRUE;
}

// Serializes context replacement. This anchor has process lifetime: DLL finalization must not destroy retained GPU work under the loader lock.
// Normal successful shutdown releases the context and all its resources; only the empty registry itself remains until process exit.
struct ContextRegistry
{
	std::mutex m_mutex;
	std::shared_ptr<Context> m_context;
};

// Return the process-lifetime anchor without introducing a second cleanup queue or a background worker.
static ContextRegistry& Registry()
{
	// Deliberately omit static destruction: unfinished ownership is released only by explicit successful cleanup or by the OS at process exit.
	static auto* registry = new ContextRegistry;
	return *registry;
}

std::shared_ptr<Context> pr::physics::PinDll()
{
	auto& registry = Registry();
	auto lock = std::lock_guard{registry.m_mutex};
	if (!registry.m_context)
		throw std::runtime_error("Physics not initialised");

	return registry.m_context;
}
std::shared_ptr<Context> pr::physics::TryPinDll()
{
	auto& registry = Registry();
	auto lock = std::lock_guard{registry.m_mutex};
	return registry.m_context;
}

void pr::physics::ReleaseUnusedContext()
{
	// Follow the same registry-before-context lock order as initialization and shutdown.
	auto& registry = Registry();
	auto lock = std::lock_guard{registry.m_mutex};
	auto ctx = registry.m_context;
	if (!ctx)
		return;

	auto state_lock = LockGuard{ctx->m_mutex};
	if (ctx->m_inits.empty() && !ctx->m_interop->HasLiveEngines())
		registry.m_context.reset();
}

// Report initialization failure without allocating formatted text or allowing a callback exception across the ABI.
static void ReportInitialisationFailure(ReportErrorCB callback, char const* message) noexcept
{
	// Preserve the original diagnostic even when a caller-supplied callback fails.
	SetLastError(message);
	try
	{
		// The callback is synchronous and is not retained by the native context.
		if (callback)
			callback(message, "", 0, 0);
	}
	catch (...)
	{
		// A failed callback cannot replace the original initialization error.
		OutputDebugStringA("Physics initialization error callback threw an exception.\n");
		fputs("Physics initialization error callback threw an exception.\n", stderr);
	}
}

// Initialise calls are reference counted and must be matched with Shutdown calls.
PHYSICS_API DllHandle __stdcall Physics_Initialise(ReportErrorCB global_error_cb)
{
	// A successful initialization must not leave an earlier thread-local error visible.
	SetLastError("");
	try
	{
		// Serialize first-context construction and registration so concurrent initialisers share one context.
		auto& registry = Registry();
		auto context_lock = std::lock_guard{registry.m_mutex};
		if (!registry.m_context)
			registry.m_context = std::make_shared<Context>();

		// Generate a unique non-null token without pointer arithmetic on a fabricated pointer.
		static std::uintptr_t next_handle = 0;
		auto handle = reinterpret_cast<DllHandle>(++next_handle);
		auto state_lock = LockGuard{registry.m_context->m_mutex};
		registry.m_context->m_inits.insert(handle);
		return handle;
	}
	catch (std::exception const& e)
	{
		// Allocation failures must still produce a nonthrowing initialization result.
		ReportInitialisationFailure(global_error_cb, e.what());
		return nullptr;
	}
	catch (...)
	{
		// Report unclassified failures through the same bounded diagnostic path.
		ReportInitialisationFailure(global_error_cb, "Failed to initialise Physics: unknown exception");
		return nullptr;
	}
}
PHYSICS_API EStatus __stdcall Physics_Shutdown(DllHandle context)
{
	SetLastError("");
	try
	{
		// A refused shutdown leaves the token valid. No stored managed callback survives its caller's lifetime.
		auto& registry = Registry();
		auto context_lock = std::lock_guard{registry.m_mutex};
		auto ctx = registry.m_context;
		if (!ctx)
		{
			// A missing context cannot consume the supplied token.
			SetLastError("Physics context is not initialized");
			return EStatus::InvalidHandle;
		}

		auto state_lock = LockGuard{ctx->m_mutex};
		if (!ctx->m_inits.contains(context))
		{
			// Preserve other callers' initialization tokens.
			SetLastError("Physics context token is invalid");
			return EStatus::InvalidHandle;
		}

		if (ctx->m_inits.size() == 1 && ctx->m_interop->HasLiveEngines())
		{
			// Keep the final token available for explicit abandoned-work collection.
			SetLastError("Physics context still owns engines; close them or collect abandoned cleanup before shutdown");
			return EStatus::StepPending;
		}

		ctx->m_inits.erase(context);
		if (ctx->m_inits.empty())
			registry.m_context.reset();

		return EStatus::Success;
	}
	catch (std::exception const& ex)
	{
		// Even an allocation failure can be reported through the ABI's emergency diagnostic.
		SetLastError(ex.what());
		return EStatus::InternalError;
	}
	catch (...)
	{
		// The status is explicit even if diagnostics cannot be allocated.
		SetLastError("Unknown physics context shutdown failure");
		return EStatus::InternalError;
	}
}

PHYSICS_API void __stdcall Physics_ContextAbandon(DllHandle context)
{
	try
	{
		// Finalization relinquishes only the caller token, not ownership of outstanding GPU resources.
		auto ctx = TryPinDll();
		if (!ctx)
			return;

		{
			// A process context with retained engines remains reachable by a subsequently initialized host.
			auto lock = LockGuard{ctx->m_mutex};
			ctx->m_inits.erase(context);
		}
		ReleaseUnusedContext();
	}
	catch (...)
	{
		// Never hide a finalizer failure or route it through a possibly finalized managed callback.
		OutputDebugStringA("Physics context finalization failed; native ownership is retained.\n");
		fputs("Physics context finalization failed; native ownership is retained.\n", stderr);
	}
}
