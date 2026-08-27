//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Deterministic retained layout: Overlay, horizontal/vertical Stack, Scroll, and Canvas, entirely
// in DIPs, driven only by each control's explicit LayoutParams (implementation-plan.md section
// 6.2). No content measurement is performed; Root is the sole auto-sized exception (see
// ControlDesc/LayoutParams).
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "tree.h"
#include "world.h"

namespace pr::view3d::ui
{
	// Compute the per-root placement (screen rect, world projection, depth and fade parameters)
	// of every root in 'tree' for the current 'viewport'. Computed once per Update and shared by
	// layout, semantics and draw-packet construction so all three agree exactly.
	std::unordered_map<ControlId, RootPlacement> ComputeRootPlacements(TreeModel const& tree, ViewportState const& viewport);

	// Compute the absolute (DIP, viewport-relative) bounds of every control in 'tree' from the
	// root placements computed for the current viewport. Deterministic: identical inputs always
	// produce identical bounds.
	std::unordered_map<ControlId, Rect> ComputeLayout(TreeModel const& tree, std::unordered_map<ControlId, RootPlacement> const& placements);
}
