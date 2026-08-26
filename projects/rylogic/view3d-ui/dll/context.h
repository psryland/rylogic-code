//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Process-level ABI state: registered runtime-init tokens plus the generation-aware table of
// owner-thread UI contexts exposed through the C ABI (implementation-plan.md section 8.1/8.2).
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/view3d-ui-dll.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	using LockGuard = std::lock_guard<std::recursive_mutex>;

	// Forward declaration only: the concrete D3D12/DirectWrite renderer type is defined in
	// renderer.h (DLL-only, private to this module) so this header never needs <d3d12.h>, matching
	// the same dependency-minimal reasoning already applied to 'm_device' below.
	class Renderer;

	// One generation-aware UI context slot exposed through the C ABI. 'm_device' is the borrowed,
	// AddRef'd external ID3D12Device (as IUnknown, so this header never needs d3d12.h); it is
	// released, after 'm_renderer' has been destroyed, exactly once on context destroy/abandon
	// (section 8.3). 'm_bridge_window' is the borrowed pr::rdr12::V3dWindow* (erased to void* for
	// the same dependency-minimal reason) this context attached a host-bridge provider to, or null
	// if no window was supplied at creation; see host_bridge.h. 'm_renderer' is the M3 screen-space
	// retained renderer driven by host_bridge.cpp's RecordThunk while a window is attached, or null
	// for a headless context (one never created with a window, e.g. every ABI/logic unit test).
	struct ContextSlot
	{
		std::uint32_t m_generation = 1;
		std::thread::id m_owner_thread;
		std::unique_ptr<UiEngine> m_engine;
		IUnknown* m_device = nullptr;
		void* m_bridge_window = nullptr;
		std::unique_ptr<Renderer> m_renderer;
	};

	// Own process-wide ABI state while initialization handles remain registered.
	struct Context
	{
		using InitSet = std::unordered_set<RuntimeHandle>;

		InitSet m_inits;
		std::recursive_mutex m_mutex;
		ReportErrorCB m_error_cb;
		std::vector<ContextSlot> m_contexts;

		explicit Context(ReportErrorCB error_cb);
		Context(Context&&) = delete;
		Context(Context const&) = delete;
		Context& operator=(Context&) = delete;
		Context& operator=(Context const&) = delete;
	};

	// Pin the process context for the duration of one ABI operation, throwing if not initialised.
	std::shared_ptr<Context> PinDll();

	// Pin the process context if initialization has completed, otherwise return null.
	std::shared_ptr<Context> TryPinDll();

	// Encode a generation and slot index into a non-zero context handle. Shared by view3d-ui.cpp
	// (which owns generation allocation/advancement) and host_bridge.cpp (which only needs to
	// decode a handle it was already given back into a slot index, never to mint one).
	inline ContextHandle MakeContextHandle(std::size_t index, std::uint32_t generation)
	{
		return (std::uint64_t{generation} << 32) | (std::uint64_t{index} + 1);
	}

	// Return the context slot index encoded in a public handle, or throw if the handle is null.
	inline std::size_t ContextHandleIndex(ContextHandle handle)
	{
		auto low = static_cast<std::uint32_t>(handle);
		if (low == 0)
			throw std::invalid_argument("View3DUI context handle is null");

		return static_cast<std::size_t>(low - 1);
	}

	// Return the generation encoded in a public handle.
	inline std::uint32_t ContextHandleGeneration(ContextHandle handle)
	{
		return static_cast<std::uint32_t>(handle >> 32);
	}
}
