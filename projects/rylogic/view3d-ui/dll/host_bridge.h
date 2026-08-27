//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Optional integration with the private view3d-12 UI host bridge (pr/view3d-12/view3d-ui-bridge.h),
// resolved dynamically from an already-loaded view3d-12.dll module. This file and renderer.h/.cpp
// are the only files in this module permitted to include that private header (and therefore
// <d3d12.h>); every other view3d-ui header/source stays dependency-minimal per implementation-
// plan.md section 2.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "context.h"

namespace pr::view3d::ui
{
	// Attach a Prepare/FinalOverlay provider for 'context_handle' to the host window 'window' (a
	// borrowed pr::rdr12::V3dWindow*, erased to void*), via view3d-12's private UI host bridge. Both
	// passes are recorded by this module's own D3D12 renderer (renderer.h/.cpp): Prepare uploads
	// resources into the render target's command list and FinalOverlay draws the current snapshot's
	// packets; view3d-12 itself remains renderer-neutral. View3D currently invokes the provider on
	// the host window's owner/render thread, so every context/render operation reached through this
	// bridge is owner-thread-affine. Throws pr::view3d::ui::EngineException with:
	//  - EStatus::UnsupportedFeature if view3d-12.dll is not loaded in this process, or does not
	//    export the bridge (headless callers that never pass a window never hit this path);
	//  - EStatus::AbiMismatch if the bridge's own HostApiVersion does not match;
	//  - EStatus::InvalidStruct if the bridge's Provider/Pass struct sizes do not match this
	//    module's compiled-against layout;
	//  - any other status the bridge's View3D_UIHostAttach itself returns (e.g. AlreadyAttached,
	//    WrongThread), translated to the nearest pr::view3d::ui::EStatus.
	void Attach(ContextHandle context_handle, void* window);

	// Detach the provider previously attached to 'window' for 'context_handle'. Never throws; a
	// failure is swallowed because this is only ever called while tearing down a context that is
	// already being destroyed or abandoned; the bridge's own DetachedThunk handles host-initiated
	// detachment (e.g. the host window is destroyed first).
	void Detach(ContextHandle context_handle, void* window) noexcept;
}
