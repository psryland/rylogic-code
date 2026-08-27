//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// COM providers that expose the published semantic snapshot to Windows UI Automation
// (implementation-plan.md M10).
//
// Only the two factory functions are public: everything a client can reach is reached through COM
// interfaces, so the provider classes themselves stay in uia_provider.cpp. Both factories are
// callable from any thread and never touch UiEngine; see uia_bridge.h for the threading contract.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "uia_bridge.h"

namespace pr::view3d::ui
{
	// Create the HWND fragment root for 'shared'. The caller owns one reference. Returns null only
	// on allocation failure.
	IRawElementProviderSimple* CreateUiaRootProvider(std::shared_ptr<UiaSharedState> shared);

	// Create the provider for the semantic node 'id'. The node does not have to be present in the
	// current snapshot: an element whose node has gone away answers UIA_E_ELEMENTNOTAVAILABLE, which
	// is exactly what a client holding a stale reference must be told. The caller owns one
	// reference. Returns null only on allocation failure.
	IRawElementProviderSimple* CreateUiaElementProvider(std::shared_ptr<UiaSharedState> shared, ControlId id);
}
