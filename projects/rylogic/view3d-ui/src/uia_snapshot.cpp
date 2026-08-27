//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Published UI Automation snapshot construction and change diffing; see uia_snapshot.h for the
// threading contract these values exist to satisfy.
#include "uia_snapshot.h"
#include "text_unicode.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Decode one packed text-blob slice into UTF-16. The blob was produced by this module from
		// text it already validated, so a decode failure means the blob is inconsistent with its
		// offsets; degrading to an empty string keeps snapshot publication total (it runs inside
		// Update and must not be able to reject a frame) while still being observably wrong rather
		// than silently plausible.
		std::wstring WideSlice(std::string const& blob, std::uint32_t offset, std::uint32_t length)
		{
			if (offset > blob.size() || length > blob.size() - offset)
				return {};

			auto wide = std::wstring{};
			if (!Utf8ToUtf16(std::string_view(blob).substr(offset, length), wide))
				return {};

			return wide;
		}

		// The raw UTF-8 slice behind a text-blob offset/length pair, empty when the pair is out of
		// range for the same reason as WideSlice.
		std::string Utf8Slice(std::string const& blob, std::uint32_t offset, std::uint32_t length)
		{
			if (offset > blob.size() || length > blob.size() - offset)
				return {};

			return blob.substr(offset, length);
		}

		// True when two rectangles differ by more than half a DIP on any edge. Layout is computed
		// in floating point from the same inputs each frame, so an exact comparison would be
		// stable, but a tolerance keeps a future sub-pixel animation from flooding clients with
		// bounds notifications no user could perceive.
		bool BoundsDiffer(Rect const& lhs, Rect const& rhs)
		{
			auto const differs = [](float a, float b) { return std::abs(a - b) > 0.5f; };
			return differs(lhs.x, rhs.x) || differs(lhs.y, rhs.y) || differs(lhs.w, rhs.w) || differs(lhs.h, rhs.h);
		}
	}

	bool UiaNode::HasState(ESemanticState flag) const
	{
		return (state_flags & static_cast<std::uint32_t>(flag)) != 0;
	}

	bool UiaNode::HasAction(ESemanticAction action) const
	{
		return (supported_actions & static_cast<std::uint32_t>(action)) != 0;
	}

	bool UiaNode::HasTextFlag(ESemanticTextFlag flag) const
	{
		return (text_flags & static_cast<std::uint32_t>(flag)) != 0;
	}

	UiaNode const* UiaSnapshot::Find(ControlId id) const
	{
		auto const it = m_index.find(id);
		return it != m_index.end() ? &m_nodes[it->second] : nullptr;
	}

	Rect UiaClientPixelRect(ViewportState const& viewport, Rect const& dip)
	{
		// DIPs -> render-target pixels -> client pixels, the exact inverse of the pointer mapping
		// in win32_input.cpp. Both ratios degrade to 1 when the host reports a zero-sized target,
		// which keeps the conversion total rather than producing infinities.
		auto const dpi_scale = viewport.dpi != 0.0f ? viewport.dpi / 96.0f : 1.0f;
		auto const ratio_x = viewport.target_width_px != 0 ? static_cast<float>(viewport.client_width_px) / static_cast<float>(viewport.target_width_px) : 1.0f;
		auto const ratio_y = viewport.target_height_px != 0 ? static_cast<float>(viewport.client_height_px) / static_cast<float>(viewport.target_height_px) : 1.0f;

		auto const left = (dip.x * dpi_scale + viewport.viewport_x_px) * ratio_x;
		auto const top = (dip.y * dpi_scale + viewport.viewport_y_px) * ratio_y;
		auto const right = ((dip.x + dip.w) * dpi_scale + viewport.viewport_x_px) * ratio_x;
		auto const bottom = ((dip.y + dip.h) * dpi_scale + viewport.viewport_y_px) * ratio_y;
		return Rect{ left, top, right - left, bottom - top };
	}

	std::shared_ptr<UiaSnapshot const> BuildUiaSnapshot(SemanticSnapshot const& semantics, ViewportState const& viewport, std::uint64_t revision, std::uint64_t publish_sequence)
	{
		auto snapshot = std::make_shared<UiaSnapshot>();
		snapshot->m_viewport = viewport;
		snapshot->m_revision = revision;
		snapshot->m_publish_sequence = publish_sequence;
		snapshot->m_nodes.reserve(semantics.m_nodes.size());
		snapshot->m_index.reserve(semantics.m_nodes.size());

		// Project each semantic record in the order it was published; the semantic walk is already
		// pre-order, so index order is navigation order and no sorting is needed anywhere below.
		for (auto const& record : semantics.m_nodes)
		{
			auto node = UiaNode{
				.id = record.id,
				.parent_id = record.parent_id,
				.role = record.role,
				.name = WideSlice(semantics.m_text_blob, record.name_offset, record.name_length),
				.description = WideSlice(semantics.m_text_blob, record.desc_offset, record.desc_length),
				.value = WideSlice(semantics.m_text_blob, record.value_offset, record.value_length),
				.value_utf8 = Utf8Slice(semantics.m_text_blob, record.value_offset, record.value_length),
				.bounds_dip = record.bounds,
				.state_flags = record.state_flags,
				.supported_actions = record.supported_actions,
				.text_flags = record.text_flags,
				.value_grapheme_count = record.value_grapheme_count,
				.caret = record.caret,
				.selection_start = record.selection_start,
				.selection_end = record.selection_end,
				.composition_start = record.composition_start,
				.composition_length = record.composition_length,
				.semantic_sequence = record.semantic_sequence,
				.parent_index = UiaNoIndex,
				.sibling_position = 0,
				.children = {},
			};
			if (node.HasState(ESemanticState::Focused))
				snapshot->m_focus_id = node.id;

			snapshot->m_index.emplace(node.id, snapshot->m_nodes.size());
			snapshot->m_nodes.push_back(std::move(node));
		}

		// Resolve parent/child links now so no navigation call ever has to search. A parent id that
		// is absent from the snapshot means the record's parent was not itself published, which the
		// semantic walk never produces; treating it as a root keeps the tree reachable regardless.
		for (auto i = std::size_t{}; i != snapshot->m_nodes.size(); ++i)
		{
			auto& node = snapshot->m_nodes[i];
			auto const parent_it = node.parent_id != 0 ? snapshot->m_index.find(node.parent_id) : snapshot->m_index.end();
			if (parent_it == snapshot->m_index.end())
			{
				node.sibling_position = snapshot->m_roots.size();
				snapshot->m_roots.push_back(i);
				continue;
			}

			auto& parent = snapshot->m_nodes[parent_it->second];
			node.parent_index = parent_it->second;
			node.sibling_position = parent.children.size();
			parent.children.push_back(i);
		}

		return snapshot;
	}

	UiaSnapshotDiff DiffUiaSnapshots(UiaSnapshot const* previous, UiaSnapshot const& current)
	{
		auto diff = UiaSnapshotDiff{
			.structure_changed = 0,
			.focus_changed = 0,
			.truncated = 0,
			.viewport_changed = 0,
			.focused_id = current.m_focus_id,
			.name_changed = {},
			.value_changed = {},
			.state_changed = {},
			.bounds_changed = {},
		};

		// The first publication is entirely new to any client, so it is reported as a structure
		// change with no per-node property notifications.
		if (previous == nullptr)
		{
			diff.structure_changed = 1;
			diff.focus_changed = current.m_focus_id != 0 ? 1 : 0;
			return diff;
		}

		// Focus leaving the fragment altogether is as much a change as focus arriving: a client that
		// is only told about arrivals keeps reporting a control that no longer has focus.
		diff.focus_changed = previous->m_focus_id != current.m_focus_id ? 1 : 0;
		if (previous->m_nodes.size() != current.m_nodes.size() || previous->m_roots.size() != current.m_roots.size())
			diff.structure_changed = 1;

		// A viewport, DPI or client-mapping change relocates or resizes every element on screen
		// while leaving its layout rectangle untouched, so detect it from the mapping itself.
		auto const& before_vp = previous->m_viewport;
		auto const& after_vp = current.m_viewport;
		diff.viewport_changed =
			before_vp.dpi != after_vp.dpi ||
			before_vp.target_width_px != after_vp.target_width_px ||
			before_vp.target_height_px != after_vp.target_height_px ||
			before_vp.client_width_px != after_vp.client_width_px ||
			before_vp.client_height_px != after_vp.client_height_px ||
			before_vp.viewport_x_px != after_vp.viewport_x_px ||
			before_vp.viewport_y_px != after_vp.viewport_y_px ||
			before_vp.viewport_width_px != after_vp.viewport_width_px ||
			before_vp.viewport_height_px != after_vp.viewport_height_px ? 1 : 0;

		// Walk the current nodes once, accumulating property differences and detecting any change
		// to membership, parentage or sibling order as a structure change.
		auto changed_count = std::size_t{};
		for (auto const& node : current.m_nodes)
		{
			auto const* before = previous->Find(node.id);
			if (before == nullptr)
			{
				diff.structure_changed = 1;
				continue;
			}
			if (before->parent_id != node.parent_id || before->sibling_position != node.sibling_position || before->role != node.role)
			{
				diff.structure_changed = 1;
				continue;
			}

			auto const name_changed = before->name != node.name || before->description != node.description;
			auto const value_changed = before->value != node.value;
			auto const state_changed = before->state_flags != node.state_flags || before->supported_actions != node.supported_actions;
			auto const bounds_changed = BoundsDiffer(before->bounds_dip, node.bounds_dip);
			if (!name_changed && !value_changed && !state_changed && !bounds_changed)
				continue;

			// Past the cap, stop recording individual notifications and let the caller fall back to
			// a single structure change; this bounds both the vectors and the raised event count.
			if (changed_count == UiaMaxChangedNodes)
			{
				diff.truncated = 1;
				diff.structure_changed = 1;
				break;
			}

			++changed_count;
			if (name_changed)
				diff.name_changed.push_back(node.id);
			if (value_changed)
				diff.value_changed.push_back(node.id);
			if (state_changed)
				diff.state_changed.push_back(node.id);
			if (bounds_changed)
				diff.bounds_changed.push_back(node.id);
		}

		// A removal leaves no current node to observe, so it is only visible as a count difference
		// or as an id present before and absent now.
		if (diff.structure_changed == 0)
		{
			for (auto const& node : previous->m_nodes)
			{
				if (current.Find(node.id) != nullptr)
					continue;

				diff.structure_changed = 1;
				break;
			}
		}

		// Property notifications are redundant once the whole subtree is being re-read.
		if (diff.truncated != 0)
		{
			diff.name_changed.clear();
			diff.value_changed.clear();
			diff.state_changed.clear();
			diff.bounds_changed.clear();
		}

		return diff;
	}
}
