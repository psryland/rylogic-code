//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Thin C-linkage wrapper around the native UiEngine (implementation-plan.md section 5/8). Mirrors
// the process-init-token/generation-aware-handle pattern used by pr::audio's own DLL layer.
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/view3d-ui-dll.h"
#include "pr/view3d-ui/engine.h"
#include "context.h"
#include "host_bridge.h"
#include "renderer.h"
#include "../src/uia_bridge.h"

using namespace pr::view3d::ui;

namespace
{
	thread_local std::string g_last_error;
	std::mutex g_context_mutex;
	std::shared_ptr<Context> g_ctx;
	std::atomic<std::uint32_t> g_next_context_generation{1};

	// Return the context slot index encoded in a public handle. Delegates to the shared,
	// non-throwing decoder in context.h (also used by host_bridge.cpp) so the two files can never
	// disagree on how a handle is unpacked.
	std::size_t ContextIndex(ContextHandle handle)
	{
		return ContextHandleIndex(handle);
	}

	// Advance a context generation without producing zero.
	void AdvanceGeneration(std::uint32_t& generation)
	{
		++generation;
		if (generation == 0)
			generation = 1;
	}

	// Allocate a process-wide generation so handles can never alias across recreated contexts.
	std::uint32_t NextContextGeneration()
	{
		for (;;)
		{
			auto generation = g_next_context_generation.fetch_add(1, std::memory_order_relaxed);
			if (generation != 0)
				return generation;
		}
	}

	// Store and report one synchronous ABI diagnostic.
	EStatus Fail(std::shared_ptr<Context> const& context, EStatus status, char const* message, char const* file, int line)
	{
		g_last_error = message;
		if (context && context->m_error_cb)
		{
			// A foreign callback must not be able to unwind through the stable C ABI.
			try
			{
				context->m_error_cb(message, file, line);
			}
			catch (...)
			{}
		}
		return status;
	}

	// Translate native failures into stable status values without crossing the ABI. Shared by
	// every export, including the schema-diagnosis exports below that run without a pinned
	// Context (their 'context' argument is then null, so Fail simply skips the error callback).
	template <typename Func>
	EStatus RunAndTranslate(std::shared_ptr<Context> const& context, Func&& func, char const* file, int line)
	{
		try
		{
			func();
			return EStatus::Success;
		}
		catch (EngineException const& ex)
		{
			return Fail(context, ex.Status(), ex.what(), file, line);
		}
		catch (std::length_error const& ex)
		{
			return Fail(context, EStatus::BufferTooSmall, ex.what(), file, line);
		}
		catch (std::invalid_argument const& ex)
		{
			return Fail(context, EStatus::InvalidArgument, ex.what(), file, line);
		}
		catch (std::exception const& ex)
		{
			return Fail(context, EStatus::InternalError, ex.what(), file, line);
		}
		catch (...)
		{
			return Fail(context, EStatus::InternalError, "Unknown native View3DUI failure", file, line);
		}
	}

	// Pin the process context (throwing if not initialised) and run 'func' under its lock.
	template <typename Func>
	EStatus ApiCall(Func&& func, char const* file, int line)
	{
		auto context = TryPinDll();
		return RunAndTranslate(context, [&]
			{
				if (!context)
					throw std::invalid_argument("View3DUI runtime handle is invalid or the module is not initialised");

				auto lock = LockGuard{context->m_mutex};
				func(*context);
			}, file, line);
	}

	// Validate a required versioned structure. A truncated/undersized buffer is a structurally
	// malformed argument (InvalidStruct), whereas a header version the running module doesn't
	// recognise is a schema-level incompatibility (SchemaMismatch) distinct from an ABI mismatch
	// (which is reported separately via View3DUI_ApiVersion) - callers can react differently to
	// "recompile/relink against a matching version" versus "the payload itself is corrupt".
	template <typename T>
	void Validate(T const* value, char const* name)
	{
		if (value == nullptr)
			throw std::invalid_argument(std::string(name) + " is null");
		if (value->header.size < sizeof(T))
			throw EngineException(EStatus::InvalidStruct, std::string(name) + " has an incompatible size");
		if (value->header.version != VIEW3D_UI_STRUCT_VERSION)
			throw EngineException(EStatus::SchemaMismatch, std::string(name) + " has an incompatible struct version");
	}

	// Resolve a context and enforce its owner-thread contract. Applied uniformly to every
	// context-scoped export, including read-only queries (implementation-plan.md section 8.2:
	// "every public operation checks the owner OS thread ... including queries").
	UiEngine& ContextFromHandle(Context& context, ContextHandle handle)
	{
		auto index = ContextIndex(handle);
		if (index >= context.m_contexts.size())
			throw std::invalid_argument("View3DUI context handle is invalid");

		auto& slot = context.m_contexts[index];
		if (slot.m_generation != ContextHandleGeneration(handle) || !slot.m_engine)
			throw EngineException(EStatus::StaleHandle, "View3DUI context handle is stale");
		if (slot.m_owner_thread != std::this_thread::get_id())
			throw EngineException(EStatus::WrongThread, "View3DUI operation called from a non-owner OS thread");

		return *slot.m_engine;
	}
}

namespace pr::view3d::ui
{
	Context::Context(ReportErrorCB error_cb)
		: m_inits()
		, m_mutex()
		, m_error_cb(error_cb)
		, m_contexts()
	{}

	std::shared_ptr<Context> PinDll()
	{
		auto lock = std::lock_guard{g_context_mutex};
		if (!g_ctx)
			throw std::invalid_argument("View3DUI runtime handle is invalid or the module is not initialised");

		return g_ctx;
	}

	std::shared_ptr<Context> TryPinDll()
	{
		auto lock = std::lock_guard{g_context_mutex};
		return g_ctx;
	}
}

// --- ABI/schema diagnosis -------------------------------------------------------------------
// These three exports are pure compile-time/thread-local diagnostics with no dependency on
// View3DUI_Initialise having been called, so a caller can fail fast on an ABI/schema mismatch
// before committing to any runtime state (implementation-plan.md section 5.1/5.2).

VIEW3D_UI_API std::uint32_t __stdcall View3DUI_ApiVersion()
{
	return VIEW3D_UI_API_VERSION;
}

VIEW3D_UI_API EStatus __stdcall View3DUI_StructSize(EStructId struct_id, std::uint32_t* size)
{
	return RunAndTranslate(nullptr, [&]
		{
			if (size == nullptr)
				throw std::invalid_argument("View3DUI_StructSize output is null");

			// Enumerates the complete EStructId set (section 5.2), including the M3-reserved
			// HostBridgeVersion/HostPassContext placeholders, so callers can size every public
			// structure this ABI version will ever describe.
			switch (struct_id)
			{
				case EStructId::Config: { *size = sizeof(Config); break; }
				case EStructId::Transaction: { *size = sizeof(Transaction); break; }
				case EStructId::Operation: { *size = sizeof(Operation); break; }
				case EStructId::Control: { *size = sizeof(ControlDesc); break; }
				case EStructId::Resource: { *size = sizeof(ResourceDesc); break; }
				case EStructId::Style: { *size = sizeof(StyleDesc); break; }
				case EStructId::Template: { *size = sizeof(TemplateDesc); break; }
				case EStructId::NormalizedInput: { *size = sizeof(NormalizedInput); break; }
				case EStructId::InputTextPayload: { *size = sizeof(InputTextPayload); break; }
				case EStructId::ViewportState: { *size = sizeof(ViewportState); break; }
				case EStructId::Event: { *size = sizeof(Event); break; }
				case EStructId::SemanticNode: { *size = sizeof(SemanticNode); break; }
				case EStructId::Diagnostics: { *size = sizeof(Diagnostics); break; }
				case EStructId::HostBridgeVersion: { *size = sizeof(HostBridgeVersion); break; }
				case EStructId::HostPassContext: { *size = sizeof(HostPassContext); break; }
				default: { throw std::invalid_argument("Unknown View3DUI structure identifier"); }
			}
		}, __FILE__, __LINE__);
}

// Copy the calling thread's last synchronous View3DUI diagnostic.
VIEW3D_UI_API EStatus __stdcall View3DUI_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required)
{
	if (required == nullptr)
		return EStatus::InvalidArgument;

	*required = static_cast<std::uint32_t>(g_last_error.size() + 1);
	if (buffer == nullptr || capacity < *required)
		return EStatus::BufferTooSmall;

	std::memcpy(buffer, g_last_error.c_str(), *required);
	return EStatus::Success;
}

// --- Process-level module lifetime ----------------------------------------------------------

// Register one process initialization token, constructing the shared process context on demand.
VIEW3D_UI_API RuntimeHandle __stdcall View3DUI_Initialise(ReportErrorCB global_error_cb)
{
	try
	{
		auto context_lock = std::lock_guard{g_context_mutex};
		if (!g_ctx)
			g_ctx = std::make_shared<Context>(global_error_cb);

		static std::uintptr_t next_handle = 0;
		auto handle = reinterpret_cast<RuntimeHandle>(++next_handle);
		auto state_lock = LockGuard{g_ctx->m_mutex};
		g_ctx->m_inits.insert(handle);
		return handle;
	}
	catch (std::exception const& ex)
	{
		g_last_error = ex.what();
		if (global_error_cb)
			global_error_cb(ex.what(), __FILE__, __LINE__);
		return nullptr;
	}
}

// Release one initialization token. Returns ResourceInUse (without releasing the token) if this
// is the final registered token and one or more contexts are still alive (section 8.1: "runtime
// shutdown with live contexts returns ResourceInUse").
VIEW3D_UI_API EStatus __stdcall View3DUI_Shutdown(RuntimeHandle runtime)
{
	auto context = std::shared_ptr<Context>{};
	{
		auto context_lock = std::lock_guard{g_context_mutex};
		context = g_ctx;
		if (!context)
			return EStatus::Success; // nothing registered; shutdown of an already-torn-down runtime is a no-op

		auto state_lock = LockGuard{context->m_mutex};
		if (!context->m_inits.contains(runtime))
			return Fail(context, EStatus::InvalidHandle, "View3DUI_Shutdown received an invalid runtime handle", __FILE__, __LINE__);

		auto live_context = std::any_of(context->m_contexts.begin(), context->m_contexts.end(), [](ContextSlot const& slot)
			{
				return slot.m_engine != nullptr;
			});
		if (context->m_inits.size() == 1 && live_context)
			return Fail(context, EStatus::ResourceInUse, "View3DUI_Shutdown cannot release the final runtime handle while contexts are alive", __FILE__, __LINE__);

		context->m_inits.erase(runtime);
		if (context->m_inits.empty())
			g_ctx.reset();
	}
	return EStatus::Success;
}

// Release a forgotten runtime token and any final process context from any thread.
VIEW3D_UI_API void __stdcall View3DUI_ContextAbandon(RuntimeHandle runtime)
{
	try
	{
		auto abandoned = std::shared_ptr<Context>{};
		{
			auto context_lock = std::lock_guard{g_context_mutex};
			auto context = g_ctx;
			if (!context)
				return;

			auto state_lock = LockGuard{context->m_mutex};
			if (!context->m_inits.contains(runtime))
				return;

			context->m_inits.erase(runtime);
			if (context->m_inits.empty())
				abandoned = std::move(g_ctx);
		}

		// Destruction can release engine-owned resources, so keep it outside the process/state locks.
		abandoned.reset();
	}
	catch (...)
	{}
}

// --- Per-window owner-thread UI context -----------------------------------------------------

// Create one owner-thread UI context, AddRef'ing the borrowed external ID3D12Device for the
// context's lifetime (section 8.3) and, if 'view3d_window' is non-null, constructing the M3
// screen-space renderer and attaching its Prepare/FinalOverlay provider to that window via the
// private view3d-12 UI host bridge (host_bridge.h/renderer.h). Every fallible step is built into
// a purely local variable first; nothing is written into 'context' until every one of them has
// already succeeded, so a failure at any point - including bridge attach, which must run last
// because it is the only step below that can still fail once the device reference has been
// taken - can never leave an attached provider or a leaked device reference behind.
VIEW3D_UI_API EStatus __stdcall View3DUI_ContextCreate(RuntimeHandle runtime, Config const* config, void* external_d3d12_device, void* view3d_window, ContextHandle* context_out)
{
	return ApiCall([&](Context& context)
		{
			if (!context.m_inits.contains(runtime))
				throw std::invalid_argument("View3DUI runtime handle is invalid");
			if (context_out == nullptr)
				throw std::invalid_argument("View3DUI context output is null");
			if (external_d3d12_device == nullptr)
				throw std::invalid_argument("View3DUI external_d3d12_device is null");
			if (config != nullptr)
				Validate(config, "Config");

			// Locate a free slot (or the just-past-end index that will require growing the
			// vector) and its would-be generation/handle up front, without mutating 'context' yet;
			// the handle must exist before Attach (host_bridge.cpp) below because it is the opaque
			// provider-context token the bridge round-trips back to this module's render thunks.
			auto index = context.m_contexts.size();
			for (auto i = std::size_t{}; i != context.m_contexts.size(); ++i)
			{
				if (!context.m_contexts[i].m_engine)
				{
					index = i;
					break;
				}
			}

			auto generation = NextContextGeneration();
			auto handle = MakeContextHandle(index, generation);
			auto effective_config = config != nullptr ? *config : DefaultConfig();

			// ID3D12Device's vtable begins with IUnknown's three methods with no multiple-
			// inheritance offset adjustment, so this reinterpret_cast is a correct, dependency-
			// minimal way to AddRef/Release the caller's device without including d3d12.h here.
			auto device = reinterpret_cast<IUnknown*>(external_d3d12_device);

			// Build the engine and (for a windowed context) the renderer as purely local objects
			// first: UiEngine's constructor can throw on a malformed Config, and while Renderer's
			// own constructor is provably non-throwing (renderer.h), std::make_unique for it can
			// still fail with bad_alloc. Neither failure here has touched 'context' or the device
			// refcount yet, so both simply propagate with nothing to unwind.
			auto instance = std::make_unique<UiEngine>(effective_config);
			auto renderer = view3d_window != nullptr
				? std::make_unique<Renderer>(device, effective_config)
				: std::unique_ptr<Renderer>{};

			// Reserve room for the new slot before taking the one remaining reversible action
			// (the device AddRef below), so nothing past this point can fail merely because the
			// vector needed to reallocate.
			if (index == context.m_contexts.size())
				context.m_contexts.reserve(index + 1);

			device->AddRef();
			try
			{
				// Attach to the host window last: it is the only step in this function that can
				// still fail once the device reference has been taken, so any exception here is
				// caught just long enough to release that reference before rethrowing, leaving no
				// COM leak and (since attach itself failed) no provider left attached either.
				if (view3d_window != nullptr)
					Attach(handle, view3d_window);
			}
			catch (...)
			{
				device->Release();
				throw;
			}

			// Every fallible step has now succeeded; commit as one non-throwing group (the
			// reserve() above guarantees this push_back cannot reallocate).
			if (index == context.m_contexts.size())
				context.m_contexts.push_back({});

			auto& slot = context.m_contexts[index];
			slot.m_generation = generation;
			slot.m_owner_thread = std::this_thread::get_id();
			slot.m_engine = std::move(instance);
			slot.m_device = device;
			slot.m_bridge_window = view3d_window;
			slot.m_renderer = std::move(renderer);
			*context_out = handle;
		}, __FILE__, __LINE__);
}

// Destroy one owner-thread UI context: detach any host-bridge provider first (so no further
// Record call can be dispatched against it), then destroy the renderer, then release the
// borrowed device reference - the ordering context.h's ContextSlot comment requires.
VIEW3D_UI_API EStatus __stdcall View3DUI_ContextDestroy(ContextHandle context_handle)
{
	return ApiCall([&](Context& context)
		{
			ContextFromHandle(context, context_handle); // validates the handle + owner thread before mutating
			auto& slot = context.m_contexts[ContextIndex(context_handle)];

			// Make the accessibility bridge unavailable first, so a UI Automation client already
			// inside a provider call starts failing with UIA_E_ELEMENTNOTAVAILABLE before the
			// window and engine it would otherwise reach are torn down.
			slot.m_engine->Uia().Shutdown();
			if (slot.m_bridge_window != nullptr)
			{
				Detach(context_handle, slot.m_bridge_window);
				slot.m_bridge_window = nullptr;
			}
			slot.m_engine.reset();
			slot.m_renderer.reset();
			if (slot.m_device != nullptr)
			{
				slot.m_device->Release();
				slot.m_device = nullptr;
			}
			AdvanceGeneration(slot.m_generation);
		}, __FILE__, __LINE__);
}

// Release a forgotten context from any thread without allowing finalizer failures to escape.
VIEW3D_UI_API void __stdcall View3DUI_UiContextAbandon(ContextHandle context_handle)
{
	try
	{
		auto context = TryPinDll();
		if (!context)
			return;

		auto lock = LockGuard{context->m_mutex};
		auto index = ContextIndex(context_handle);
		if (index >= context->m_contexts.size())
			return;

		auto& slot = context->m_contexts[index];
		if (slot.m_generation != ContextHandleGeneration(context_handle) || !slot.m_engine)
			return;

		// Abandonment can run on any thread, so the bridge is marked unavailable here for the same
		// reason as in ContextDestroy: outstanding provider references must stop resolving before
		// the engine behind them disappears.
		slot.m_engine->Uia().Shutdown();

		if (slot.m_bridge_window != nullptr)
		{
			Detach(context_handle, slot.m_bridge_window);
			slot.m_bridge_window = nullptr;
		}
		slot.m_engine.reset();
		slot.m_renderer.reset();
		if (slot.m_device != nullptr)
		{
			slot.m_device->Release();
			slot.m_device = nullptr;
		}
		AdvanceGeneration(slot.m_generation);
	}
	catch (...)
	{}
}

#define VIEW3D_UI_CONTEXT_CALL(name, parameters, expression) \
	VIEW3D_UI_API EStatus __stdcall name parameters \
	{ \
		return ApiCall([&](Context& context) { expression; }, __FILE__, __LINE__); \
	}

// --- Retained model --------------------------------------------------------------------------
VIEW3D_UI_CONTEXT_CALL(View3DUI_TransactionApply, (ContextHandle context_handle, Transaction const* transaction), Validate(transaction, "Transaction"); ContextFromHandle(context, context_handle).TransactionApply(*transaction))

// --- Input -------------------------------------------------------------------------------------
VIEW3D_UI_CONTEXT_CALL(View3DUI_InputInject, (ContextHandle context_handle, NormalizedInput const* input), Validate(input, "NormalizedInput"); ContextFromHandle(context, context_handle).InputInject(*input))
VIEW3D_UI_CONTEXT_CALL(View3DUI_InputInjectText, (ContextHandle context_handle, NormalizedInput const* input, InputTextPayload const* payload), Validate(input, "NormalizedInput"); Validate(payload, "InputTextPayload"); ContextFromHandle(context, context_handle).InputInjectText(*input, *payload))
VIEW3D_UI_CONTEXT_CALL(View3DUI_CaretGeometry, (ContextHandle context_handle, ControlId control_id, Rect* caret_dip, std::int32_t* valid), if (caret_dip == nullptr || valid == nullptr) throw std::invalid_argument("View3DUI_CaretGeometry output is null"); ContextFromHandle(context, context_handle).CaretGeometry(control_id, *caret_dip, *valid))

// --- Update and render-neutral snapshot -----------------------------------------------------------
VIEW3D_UI_CONTEXT_CALL(View3DUI_Update, (ContextHandle context_handle, ViewportState const* viewport), Validate(viewport, "ViewportState"); ContextFromHandle(context, context_handle).Update(*viewport))

// --- Events ------------------------------------------------------------------------------------
VIEW3D_UI_CONTEXT_CALL(View3DUI_EventCount, (ContextHandle context_handle, std::uint32_t* count), if (count == nullptr) throw std::invalid_argument("View3DUI event count output is null"); *count = ContextFromHandle(context, context_handle).EventCount())

// --- Diagnostics ---------------------------------------------------------------------------------
VIEW3D_UI_CONTEXT_CALL(View3DUI_DiagnosticsGet, (ContextHandle context_handle, Diagnostics* diagnostics), if (diagnostics == nullptr) throw std::invalid_argument("View3DUI diagnostics output is null"); *diagnostics = ContextFromHandle(context, context_handle).DiagnosticsGet())

#undef VIEW3D_UI_CONTEXT_CALL

// Translate one raw Win32 message and report the resulting consumed/invalidate flags.
VIEW3D_UI_API EStatus __stdcall View3DUI_ProcessWindowMessage(ContextHandle context_handle, HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, std::int32_t* consumed, LRESULT* result, std::int32_t* invalidate)
{
	return ApiCall([&](Context& context)
		{
			if (consumed == nullptr || result == nullptr || invalidate == nullptr)
				throw std::invalid_argument("View3DUI_ProcessWindowMessage output is null");

			auto& engine = ContextFromHandle(context, context_handle);
			*consumed = engine.ProcessWindowMessage(hwnd, msg, wparam, lparam, *result, *invalidate);
		}, __FILE__, __LINE__);
}

// Copy and drain buffered owner-thread events plus their packed UTF-8 payload bytes.
VIEW3D_UI_API EStatus __stdcall View3DUI_EventsCopy(ContextHandle context_handle, Event* events, std::uint32_t capacity, std::uint32_t* required, std::byte* payload_blob, std::uint32_t payload_capacity, std::uint32_t* payload_required)
{
	return ApiCall([&](Context& context)
		{
			if (required == nullptr || payload_required == nullptr)
				throw std::invalid_argument("View3DUI event count output is null");

			auto& engine = ContextFromHandle(context, context_handle);
			*required = engine.EventCount();
			*payload_required = engine.EventPayloadBytesPending();
			if (capacity < *required || (events == nullptr && *required != 0))
				throw std::length_error("View3DUI event buffer is too small");
			if (payload_capacity < *payload_required || (payload_blob == nullptr && *payload_required != 0))
				throw std::length_error("View3DUI event payload buffer is too small");

			engine.EventsCopy(std::span{events, events != nullptr ? capacity : 0U}, std::span{payload_blob, payload_blob != nullptr ? payload_capacity : 0U});
		}, __FILE__, __LINE__);
}

// Copy the current semantic snapshot plus its packed UTF-8 text bytes (not drained; persistent
// until the next View3DUI_Update call).
VIEW3D_UI_API EStatus __stdcall View3DUI_SemanticsCopy(ContextHandle context_handle, SemanticNode* nodes, std::uint32_t capacity, std::uint32_t* required, char* text_blob, std::uint32_t text_capacity, std::uint32_t* text_required)
{
	return ApiCall([&](Context& context)
		{
			if (required == nullptr || text_required == nullptr)
				throw std::invalid_argument("View3DUI semantic count output is null");

			auto& engine = ContextFromHandle(context, context_handle);
			*required = engine.SemanticCount();
			*text_required = engine.SemanticTextBytesPending();
			if (capacity < *required || (nodes == nullptr && *required != 0))
				throw std::length_error("View3DUI semantic node buffer is too small");
			if (text_capacity < *text_required || (text_blob == nullptr && *text_required != 0))
				throw std::length_error("View3DUI semantic text buffer is too small");

			engine.SemanticsCopy(std::span{nodes, nodes != nullptr ? capacity : 0U}, std::span{text_blob, text_blob != nullptr ? text_capacity : 0U});
		}, __FILE__, __LINE__);
}
