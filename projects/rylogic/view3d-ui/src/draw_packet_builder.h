//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Builds the internal, renderer-neutral DrawPacket from the accepted tree, most recently computed
// layout, and resolved per-control style visuals (implementation-plan.md section 4.3/9.3). Not
// part of the public ABI; consumed natively by this module's own dll/host_bridge.cpp and
// dll/renderer.cpp during the FinalOverlay pass.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "pr/view3d-ui/draw_packet.h"
#include "tree.h"
#include "style.h"
#include "input.h"
#include "world.h"

namespace pr::view3d::ui
{
	// Build one immutable draw packet in deterministic pre-order (invisible controls and their
	// whole subtree are excluded, matching hit-testing/tab-order). One DrawGroup is emitted per
	// root that draws anything, in tree root order, carrying the root's host-stage policy and
	// depth/fade parameters; culled world roots contribute nothing. 'styles' is mutated (its
	// transition runtime state advances to 'time_ms' for every visited control).
	DrawPacket BuildDrawPacket(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, std::unordered_map<ControlId, RootPlacement> const& placements, StyleResolver& styles, InputState const& input_state, std::uint64_t accepted_revision, std::uint64_t visual_sequence, double time_ms, float viewport_dpi);
}
