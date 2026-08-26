//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Bounded, host-time-driven resolution of a control's active style state channel and its
// transition-interpolated visual (implementation-plan.md section 6.4). One style applies as a
// whole to a control (not per template part), which keeps StyleDesc fixed-layout.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "tree.h"

namespace pr::view3d::ui
{
	// Per-control transition tracking that persists across Update() calls so a channel change can
	// be interpolated over its declared duration rather than snapping instantly. 'was_visible' and
	// 'last_value_sequence' additionally let Resolve detect a Visibility/ValueChanged transition,
	// which (unlike every other channel) depends on this control's own history rather than being
	// derivable from its descriptor alone.
	struct StyleRuntimeState
	{
		EStateChannel active_channel = EStateChannel::Normal;
		EStateChannel previous_channel = EStateChannel::Normal;
		double transition_start_ms = 0.0;
		std::int32_t seen_before = 0;
		std::int32_t was_visible = 1; // defaults visible so a brand-new already-visible control never spuriously transitions
		std::uint32_t last_value_sequence = 0;
	};

	// Resolves the single active EStateChannel for a control from its own enablement/validation/
	// selection plus the input state machine's current hover/pressed/focus targets. Priority
	// (highest first): Disabled, Pressed, Invalid, Focused, Hover, Selected, Normal - an enabled
	// control being actively pressed shows its pressed visual even while also invalid or focused,
	// but a disabled control always shows Disabled regardless of any other condition. Never returns
	// Visibility or ValueChanged: those depend on per-control history and are folded in by
	// StyleResolver::Resolve, which alone has access to that history.
	EStateChannel ResolveActiveChannel(ControlNode const& node, ControlId hover_id, ControlId pressed_id, ControlId focus_id);

	// Bounded (one entry per live control id) resolver of the current blended StyleVisual for every
	// control, tracking per-control transition state across Update() calls.
	class StyleResolver
	{
		std::unordered_map<ControlId, StyleRuntimeState> m_runtime;

	public:
		// Resolve 'node's combined active channel (base interaction/validation channel, folding in
		// a one-Update()-call Visibility/ValueChanged transition when applicable - see the
		// EStateChannel doc comment in types.h for the exact combined priority) at 'time_ms' and
		// return the current blended visual sampled from 'style'.
		StyleVisual Resolve(ControlNode const& node, StyleRecord const& style, ControlId hover_id, ControlId pressed_id, ControlId focus_id, double time_ms);

		// Record that 'id' is currently invisible (ControlDesc::visible == 0), so the next Resolve
		// call for 'id' after it becomes visible again fires exactly one Visibility transition.
		// Must only be called for a control's own visible flag, never for a hidden ancestor's
		// descendants (which Resolve never observes anyway, since an invisible subtree is skipped).
		void MarkInvisible(ControlId id);

		// Discard transition state for controls no longer present in the accepted tree, bounding
		// this resolver's memory to the current control count rather than growing unboundedly.
		void Prune(std::unordered_set<ControlId> const& live_ids);
	};
}
