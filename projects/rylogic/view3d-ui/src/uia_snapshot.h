//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Immutable published projection of one Update()'s semantic snapshot, plus the change diff that
// selects which UI Automation events to raise (implementation-plan.md sections 5.5/7.4, M10).
//
// UI Automation calls arrive on arbitrary COM/RPC worker threads, so nothing here may reach into
// UiEngine. The owner thread builds one of these values after every Update and publishes it; every
// provider property, navigation and text-range read then works purely against the published value,
// which is never mutated after construction. This is a projection of the semantic snapshot into
// the shapes UI Automation needs (UTF-16 strings, resolved parent/child indices, screen pixels) -
// deliberately not a second element model: no node exists here that the semantic snapshot did not
// already publish, and no ordering differs from the semantic pre-order.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "semantics.h"

namespace pr::view3d::ui
{
	// Sentinel used by UiaNode::parent_index for a node whose parent is the HWND fragment root
	// rather than another semantic node.
	inline constexpr std::size_t UiaNoIndex = static_cast<std::size_t>(-1);

	// One published semantic element. Strings are pre-converted to UTF-16 because every UI
	// Automation property is a BSTR and a worker thread must not have to re-decode the packed
	// UTF-8 blob (and must not be able to fail while doing so). 'value_utf8' is retained alongside
	// the UTF-16 value because the text-range operations move in UTF-8 byte offsets, which is the
	// unit the semantic snapshot and the edit model both address text in.
	struct UiaNode
	{
		ControlId id;
		ControlId parent_id;
		EControlType role;
		std::wstring name;
		std::wstring description;
		std::wstring value;
		std::string value_utf8;
		Rect bounds_dip;
		std::uint32_t state_flags;         // Bitmask of ESemanticState.
		std::uint32_t supported_actions;   // Bitmask of ESemanticAction.
		std::uint32_t text_flags;          // Bitmask of ESemanticTextFlag.
		std::uint32_t value_grapheme_count;
		std::uint32_t caret;               // UTF-8 byte offsets into 'value_utf8'.
		std::uint32_t selection_start;
		std::uint32_t selection_end;
		std::uint32_t composition_start;
		std::uint32_t composition_length;
		std::uint64_t semantic_sequence;
		std::size_t parent_index;          // UiaNoIndex when the node is a semantic root.
		std::size_t sibling_position;      // Index of this node within its parent's child list.
		std::vector<std::size_t> children; // Deterministic semantic order, same as the pre-order walk.

		// True when 'flag' is present in this node's state bitmask.
		bool HasState(ESemanticState flag) const;

		// True when 'action' is present in this node's supported-action bitmask.
		bool HasAction(ESemanticAction action) const;

		// True when 'flag' is present in this node's text bitmask.
		bool HasTextFlag(ESemanticTextFlag flag) const;
	};

	// One published snapshot. Every member is written once during construction and read-only
	// afterwards, which is what makes a shared_ptr<UiaSnapshot const> safe to hand to a worker
	// thread without further locking.
	class UiaSnapshot
	{
	public:
		std::vector<UiaNode> m_nodes;
		std::vector<std::size_t> m_roots;                       // Semantic roots, in pre-order.
		std::unordered_map<ControlId, std::size_t> m_index;
		ViewportState m_viewport{};
		std::uint64_t m_revision = 0;
		std::uint64_t m_publish_sequence = 0;
		ControlId m_focus_id = 0;                               // The node carrying ESemanticState::Focused, or 0.

		// The published node with 'id', or null when no such node is in this snapshot. A null
		// result is how a provider detects that the element it represents has gone away.
		UiaNode const* Find(ControlId id) const;
	};

	// Convert one viewport-relative DIP rectangle into client physical pixels, inverting exactly
	// the client-pixels-to-DIP mapping the input translator applies to pointer positions
	// (section 7.4). Pure, so the DPI/viewport-offset arithmetic is directly testable without a
	// window.
	Rect UiaClientPixelRect(ViewportState const& viewport, Rect const& dip);

	// Build the published projection of 'semantics' as observed under 'viewport'. 'revision' and
	// 'publish_sequence' stamp the result so a provider can tell two publications apart.
	std::shared_ptr<UiaSnapshot const> BuildUiaSnapshot(SemanticSnapshot const& semantics, ViewportState const& viewport, std::uint64_t revision, std::uint64_t publish_sequence);

	// What changed between two published snapshots, expressed as the minimum set of UI Automation
	// notifications a client needs. Per-node lists are capped (see UiaMaxChangedNodes): when the
	// cap is exceeded 'truncated' is set and the caller raises one structure-changed notification
	// instead of thousands of property notifications, which is what keeps event work bounded.
	struct UiaSnapshotDiff
	{
		std::int32_t structure_changed;
		std::int32_t focus_changed;
		std::int32_t truncated;
		std::int32_t viewport_changed;
		ControlId focused_id;
		std::vector<ControlId> name_changed;
		std::vector<ControlId> value_changed;
		std::vector<ControlId> state_changed;
		std::vector<ControlId> bounds_changed;
	};

	// Upper bound on how many per-node property notifications one publication may produce.
	inline constexpr std::size_t UiaMaxChangedNodes = 64;

	// Compare 'current' against 'previous' (which may be null for the first publication) and
	// report what a client must be told about. Membership, parent or sibling-order differences all
	// count as a structure change; property lists only ever contain ids present in both snapshots.
	// A change to the viewport mapping alone moves every element on screen without altering its
	// layout rectangle, so it is reported separately rather than as thousands of bounds changes.
	UiaSnapshotDiff DiffUiaSnapshots(UiaSnapshot const* previous, UiaSnapshot const& current);
}
