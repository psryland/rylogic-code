//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Deterministic semantic snapshot builder (implementation-plan.md section 5.5): the single
// source of role/name/description/value/state/actions/bounds data for native tests and the later
// UI Automation provider.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "tree.h"
#include "input.h"
#include "world.h"

namespace pr::view3d::ui
{
	// One built semantic snapshot: a deterministic pre-order SemanticNode array plus the packed
	// UTF-8 text blob its name/desc/value offsets index into.
	class SemanticSnapshot
	{
	public:
		std::vector<SemanticNode> m_nodes;
		std::string m_text_blob;
	};

	// Build a fresh semantic snapshot from the accepted tree, most recently computed layout, root
	// placements, and current focus, stamping every node with 'accepted_revision' and a per-node
	// sequence assigned in the same pre-order used for tab order/hit testing/draw order, so
	// repeated calls against unchanged input are byte-for-byte identical (section 9.3). Controls
	// under a culled world root are reported Offscreen.
	SemanticSnapshot BuildSemanticSnapshot(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, std::unordered_map<ControlId, RootPlacement> const& placements, InputState const& input, std::uint64_t accepted_revision);
}
