//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// See host_bridge.h. Resolves and calls the four view3d-12 UI host bridge exports dynamically,
// via GetProcAddress against the already-loaded view3d-12.dll module; this module never links an
// import library for view3d-12 and never calls LoadLibrary for it (an unattached, headless
// View3DUI context must not force-load a renderer module it does not otherwise need).
#include "pr/view3d-12/view3d-ui-bridge.h"
#include "pr/view3d-ui/engine.h"
#include "context.h"
#include "host_bridge.h"
#include "renderer.h"

namespace pr::view3d::ui
{
	namespace
	{
		// The four resolved bridge exports, cached for the process lifetime once resolution
		// succeeds; view3d-12.dll, once loaded, is never unloaded before this module is.
		struct BridgeExports
		{
			ApiVersionFn m_api_version;
			StructSizeFn m_struct_size;
			AttachFn m_attach;
			DetachFn m_detach;
		};

		std::mutex g_resolve_mutex;
		BridgeExports g_exports = {};
		bool g_resolved = false;

		// Resolve and validate the bridge export table on first use, throwing a diagnosable
		// EStatus if view3d-12.dll is not loaded, does not export the bridge, or exposes an
		// incompatible ABI/schema version. Cached thereafter so every subsequent Attach/Detach call
		// only pays for the mutex lock, not repeated GetProcAddress lookups.
		BridgeExports const& ResolveExports()
		{
			auto lock = std::lock_guard{g_resolve_mutex};
			if (g_resolved)
				return g_exports;

			auto module = ::GetModuleHandleW(L"view3d-12.dll");
			if (module == nullptr)
				throw EngineException(EStatus::UnsupportedFeature, "view3d-12.dll is not loaded in this process");

			auto api_version = reinterpret_cast<ApiVersionFn>(::GetProcAddress(module, ApiVersionExport));
			auto struct_size = reinterpret_cast<StructSizeFn>(::GetProcAddress(module, StructSizeExport));
			auto attach = reinterpret_cast<AttachFn>(::GetProcAddress(module, AttachExport));
			auto detach = reinterpret_cast<DetachFn>(::GetProcAddress(module, DetachExport));
			if (api_version == nullptr || struct_size == nullptr || attach == nullptr || detach == nullptr)
				throw EngineException(EStatus::UnsupportedFeature, "view3d-12.dll does not export the UI host bridge");

			if (api_version() != HostApiVersion)
				throw EngineException(EStatus::AbiMismatch, "view3d-12.dll UI host bridge API version does not match");

			// Struct-size discovery (mirroring this module's own View3DUI_StructSize contract)
			// catches a stale/mismatched bridge header before any Pass/Provider pointer is ever
			// dereferenced across the boundary.
			auto provider_size = std::uint32_t{};
			if (struct_size(EHostStructId::Provider, &provider_size) != EHostStatus::Success || provider_size != sizeof(Provider))
				throw EngineException(EStatus::InvalidStruct, "view3d-12.dll UI host bridge Provider struct size does not match");

			auto pass_size = std::uint32_t{};
			if (struct_size(EHostStructId::Pass, &pass_size) != EHostStatus::Success || pass_size != sizeof(Pass))
				throw EngineException(EStatus::InvalidStruct, "view3d-12.dll UI host bridge Pass struct size does not match");

			g_exports = { api_version, struct_size, attach, detach };
			g_resolved = true;
			return g_exports;
		}

		// Translate a bridge EHostStatus into this module's own EStatus for exception reporting.
		EStatus TranslateStatus(EHostStatus status)
		{
			switch (status)
			{
				case EHostStatus::Success: { return EStatus::Success; }
				case EHostStatus::InvalidArgument: { return EStatus::InvalidArgument; }
				case EHostStatus::InvalidStruct: { return EStatus::InvalidStruct; }
				case EHostStatus::AlreadyAttached: { return EStatus::ResourceInUse; }
				case EHostStatus::NotAttached: { return EStatus::InvalidHandle; }
				case EHostStatus::WrongThread: { return EStatus::WrongThread; }
				case EHostStatus::ProviderFailed: { return EStatus::InternalError; }
				default: throw std::invalid_argument("Unknown view3d-12 UI host bridge status");
			}
		}

		// Erase a context handle to the opaque provider-context token the bridge round-trips back
		// to RecordThunk/DetachedThunk unmodified. ContextHandle is a 64 bit value and this module
		// is x64-only, so this round-trips exactly through a void*.
		void* ContextHandleToToken(ContextHandle context_handle)
		{
			return reinterpret_cast<void*>(static_cast<std::uintptr_t>(context_handle));
		}

		// Render callback invoked by view3d-12 once per attached pass, per frame, on the host
		// window's owner/render thread - the same thread that attached this provider at context
		// creation - so decoding the context handle back into its slot needs no additional cross-
		// thread synchronisation beyond the process mutex every other ABI entry point already
		// takes. It is a foreign no-throw ABI boundary: no C++ exception may unwind into view3d-12,
		// so every branch returns an EHostStatus instead of throwing, including for the
		// reserved, not-yet-driven pass values and for a handle that no longer resolves to a live
		// context (e.g. a race with the owner thread tearing the context down).
		EHostStatus __stdcall RecordThunk(void* context, Pass const* pass) noexcept
		{
			try
			{
				if (pass == nullptr)
					return EHostStatus::InvalidArgument;

				auto process = TryPinDll();
				if (!process)
					return EHostStatus::NotAttached;

				auto lock = LockGuard{process->m_mutex};
				auto handle = static_cast<ContextHandle>(reinterpret_cast<std::uintptr_t>(context));
				auto index = ContextHandleIndex(handle);
				if (index >= process->m_contexts.size())
					return EHostStatus::NotAttached;

				auto& slot = process->m_contexts[index];
				if (slot.m_generation != ContextHandleGeneration(handle) || !slot.m_renderer)
					return EHostStatus::NotAttached;

				// Record against the renderer's own EStatus, surfacing a fault (currently only
				// DeviceLost) through the same Diagnostics::last_failure_status field used for
				// rejected transactions/input, then translate to the bridge's status vocabulary
				// purely for the host's own diagnostic logging - this call's return value never
				// stops or rolls back the host's frame either way.
				auto const status = slot.m_renderer->Record(*pass, slot.m_engine->DrawPackets());
				if (status != EStatus::Success)
					slot.m_engine->NotifyRendererStatus(status);

				switch (status)
				{
					case EStatus::Success: { return EHostStatus::Success; }
					case EStatus::ResourceLimit: { return EHostStatus::Success; } // degraded but not a provider failure
					case EStatus::MissingAsset: { return EHostStatus::Success; } // an unresolvable font skips its text, the rest of the frame still draws
					case EStatus::DeviceLost: { return EHostStatus::ProviderFailed; }
					case EStatus::InvalidArgument: { return EHostStatus::InvalidArgument; }
					case EStatus::InternalError: { return EHostStatus::ProviderFailed; }
					default: return EHostStatus::ProviderFailed;
				}
			}
			catch (...)
			{
				return EHostStatus::ProviderFailed;
			}
		}

		// Invoked if the host force-detaches this provider (e.g. window destruction) without
		// View3DUI_ContextDestroy having called Detach first. Clears the owning slot's
		// 'm_bridge_window' so a later ContextDestroy/UiContextAbandon does not attempt a
		// redundant, now-invalid Detach call against a window the host has already torn down.
		void __stdcall DetachedThunk(void* context) noexcept
		{
			try
			{
				auto process = TryPinDll();
				if (!process)
					return;

				auto lock = LockGuard{process->m_mutex};
				auto handle = static_cast<ContextHandle>(reinterpret_cast<std::uintptr_t>(context));
				auto index = ContextHandleIndex(handle);
				if (index >= process->m_contexts.size())
					return;

				auto& slot = process->m_contexts[index];
				if (slot.m_generation == ContextHandleGeneration(handle))
					slot.m_bridge_window = nullptr;
			}
			catch (...)
			{}
		}
	}

	void Attach(ContextHandle context_handle, void* window)
	{
		auto const& exports = ResolveExports();

		// The provider's context token is the context handle itself, erased to void*; no extra
		// allocation is needed because RecordThunk/DetachedThunk only ever decode it back into a
		// slot index/generation pair, never dereference it as a pointer.
		auto provider = Provider
		{
			.m_header = {sizeof(Provider), HostStructVersion},
			.m_context = ContextHandleToToken(context_handle),
			.m_record = &RecordThunk,
			.m_detached = &DetachedThunk,
		};

		auto status = exports.m_attach(window, &provider);
		if (status != EHostStatus::Success)
			throw EngineException(TranslateStatus(status), "view3d-12.dll UI host bridge attach failed");
	}

	void Detach(ContextHandle context_handle, void* window) noexcept
	{
		try
		{
			auto const& exports = ResolveExports();
			exports.m_detach(window, ContextHandleToToken(context_handle));
		}
		catch (...)
		{}
	}
}
