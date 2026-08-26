//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "layout.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Offset of a child's near edge within 'avail' space, for a child of 'size' using 'align'.
		float AlignOffset(std::int32_t align_value, std::int32_t stretch_value, float avail, float size)
		{
			if (align_value == stretch_value)
				return 0.0f; // Stretch: the child already fills 'avail', so it starts at the origin.

			// Left/Top == 0, Center == 1, Right/Bottom == 2 for both EHAlign and EVAlign.
			switch (align_value)
			{
				case 0: return 0.0f;
				case 1: return (avail - size) * 0.5f;
				case 2: return avail - size;
				default: throw EngineException(EStatus::UnknownType, std::format("AlignOffset: unrecognised alignment value {}", align_value));
			}
		}

		// Resolve one child's cross-axis (or single-axis, for Overlay) extent and offset.
		float ResolveExtent(bool stretch, float explicit_size, float avail)
		{
			return stretch ? avail : std::min(explicit_size, avail);
		}

		void LayoutChildren(TreeModel const& tree, ControlNode const& node, Rect const& self_rect, std::unordered_map<ControlId, Rect>& rects);

		// Position one child within its parent's Overlay content rect using its own align/margins.
		Rect PlaceOverlay(Rect const& content, LayoutParams const& cl)
		{
			auto avail_w = std::max(0.0f, content.w - cl.margin_left - cl.margin_right);
			auto avail_h = std::max(0.0f, content.h - cl.margin_top - cl.margin_bottom);
			auto w = ResolveExtent(cl.h_align == EHAlign::Stretch, cl.width, avail_w);
			auto h = ResolveExtent(cl.v_align == EVAlign::Stretch, cl.height, avail_h);
			auto ox = AlignOffset(static_cast<std::int32_t>(cl.h_align), static_cast<std::int32_t>(EHAlign::Stretch), avail_w, w);
			auto oy = AlignOffset(static_cast<std::int32_t>(cl.v_align), static_cast<std::int32_t>(EVAlign::Stretch), avail_h, h);
			return Rect{ content.x + cl.margin_left + ox, content.y + cl.margin_top + oy, w, h };
		}

		// Lay out every child of 'node' according to its own layout_mode, within its content rect
		// (self_rect shrunk by node's own padding), then recurse into each child.
		void LayoutChildren(TreeModel const& tree, ControlNode const& node, Rect const& self_rect, std::unordered_map<ControlId, Rect>& rects)
		{
			auto const& lp = node.desc.layout;
			Rect content{
				self_rect.x + lp.padding_left,
				self_rect.y + lp.padding_top,
				std::max(0.0f, self_rect.w - lp.padding_left - lp.padding_right),
				std::max(0.0f, self_rect.h - lp.padding_top - lp.padding_bottom),
			};

			switch (node.desc.layout_mode)
			{
				case ELayoutMode::Overlay:
				{
					for (auto child_id : node.children)
					{
						auto const& child = tree.m_controls.at(child_id);
						auto rect = PlaceOverlay(content, child.desc.layout);
						rects[child_id] = rect;
						LayoutChildren(tree, child, rect, rects);
					}
					break;
				}
				case ELayoutMode::StackHorizontal:
				{
					// Children flow left-to-right; cross-axis (vertical) alignment/stretch applies
					// within the container's full content height, main-axis size is each child's
					// own explicit width plus its own margins and the container's stack_spacing.
					auto cursor = content.x;
					for (auto child_id : node.children)
					{
						auto const& child = tree.m_controls.at(child_id);
						auto const& cl = child.desc.layout;
						auto avail_h = std::max(0.0f, content.h - cl.margin_top - cl.margin_bottom);
						auto h = ResolveExtent(cl.v_align == EVAlign::Stretch, cl.height, avail_h);
						auto oy = AlignOffset(static_cast<std::int32_t>(cl.v_align), static_cast<std::int32_t>(EVAlign::Stretch), avail_h, h);
						auto x = cursor + cl.margin_left;
						Rect rect{ x, content.y + cl.margin_top + oy, cl.width, h };
						rects[child_id] = rect;
						LayoutChildren(tree, child, rect, rects);
						cursor = x + cl.width + cl.margin_right + lp.stack_spacing;
					}
					break;
				}
				case ELayoutMode::StackVertical:
				{
					// Children flow top-to-bottom; cross-axis (horizontal) alignment/stretch
					// applies within the container's full content width.
					auto cursor = content.y;
					for (auto child_id : node.children)
					{
						auto const& child = tree.m_controls.at(child_id);
						auto const& cl = child.desc.layout;
						auto avail_w = std::max(0.0f, content.w - cl.margin_left - cl.margin_right);
						auto w = ResolveExtent(cl.h_align == EHAlign::Stretch, cl.width, avail_w);
						auto ox = AlignOffset(static_cast<std::int32_t>(cl.h_align), static_cast<std::int32_t>(EHAlign::Stretch), avail_w, w);
						auto y = cursor + cl.margin_top;
						Rect rect{ content.x + cl.margin_left + ox, y, w, cl.height };
						rects[child_id] = rect;
						LayoutChildren(tree, child, rect, rects);
						cursor = y + cl.height + cl.margin_bottom + lp.stack_spacing;
					}
					break;
				}
				case ELayoutMode::Scroll:
				{
					// Same per-child placement rule as Overlay, but against a content rect shifted
					// by the container's own scroll offset; clipping the shifted-out portion to the
					// visible rect is a template Clip-primitive/renderer concern, not layout's.
					Rect scrolled_content{ content.x - lp.scroll_offset_x, content.y - lp.scroll_offset_y, content.w, content.h };
					for (auto child_id : node.children)
					{
						auto const& child = tree.m_controls.at(child_id);
						auto rect = PlaceOverlay(scrolled_content, child.desc.layout);
						rects[child_id] = rect;
						LayoutChildren(tree, child, rect, rects);
					}
					break;
				}
				case ELayoutMode::Canvas:
				{
					// Each child is placed at its own explicit canvas_x/y offset from the parent's
					// content origin; alignment/margins are ignored because the position is already
					// explicit, unlike every other layout mode which derives position from them.
					for (auto child_id : node.children)
					{
						auto const& child = tree.m_controls.at(child_id);
						auto const& cl = child.desc.layout;
						Rect rect{ content.x + cl.canvas_x, content.y + cl.canvas_y, cl.width, cl.height };
						rects[child_id] = rect;
						LayoutChildren(tree, child, rect, rects);
					}
					break;
				}
				case ELayoutMode::Count:
				default:
				{
					// Rejected during TreeModel::Apply validation; reaching here would indicate an
					// accepted tree bypassed validation, which is an internal invariant violation.
					throw EngineException(EStatus::InternalError, std::format("LayoutChildren: control {} has unsupported layout_mode {}", node.desc.id, static_cast<int>(node.desc.layout_mode)));
				}
			}
		}

		// Map every rect in the subtree rooted at 'id' from the root's local DIP space onto the
		// screen, scaling about the local origin and then translating to 'origin_x'/'origin_y'.
		// Applied only to world roots; screen roots are laid out directly in screen space.
		void TransformSubtree(TreeModel const& tree, ControlId id, float origin_x, float origin_y, float scale, std::unordered_map<ControlId, Rect>& rects)
		{
			if (auto it = rects.find(id); it != rects.end())
			{
				auto& rect = it->second;
				rect = Rect{ origin_x + rect.x * scale, origin_y + rect.y * scale, rect.w * scale, rect.h * scale };
			}
			if (auto node = tree.m_controls.find(id); node != tree.m_controls.end())
			{
				for (auto child_id : node->second.children)
					TransformSubtree(tree, child_id, origin_x, origin_y, scale, rects);
			}
		}
	}

	std::unordered_map<ControlId, RootPlacement> ComputeRootPlacements(TreeModel const& tree, ViewportState const& viewport)
	{
		std::unordered_map<ControlId, RootPlacement> placements;
		placements.reserve(tree.m_roots.size());

		// The whole pipeline works in viewport-relative DIPs: the origin is the top-left of the
		// host's viewport within the render target, and one unit is 1/96in at the reported DPI.
		// This is the same space win32_input.cpp's ClientPixelsToDip maps pointer positions into,
		// so layout, hit-testing, semantics and drawing all share one coordinate system.
		auto viewport_dip_w = viewport.viewport_width_px * 96.0f / viewport.dpi;
		auto viewport_dip_h = viewport.viewport_height_px * 96.0f / viewport.dpi;

		for (auto root_id : tree.m_roots)
		{
			auto const& root = tree.m_controls.at(root_id);
			auto policy = root.desc.root_policy;
			auto width = root.desc.layout.width;
			auto height = root.desc.layout.height;
			if (!IsWorldPolicy(policy))
			{
				// A screen root with a zero authored size fills the host viewport; its rect is
				// the viewport-relative origin, exactly as before world roots existed.
				if (width == 0.0f && height == 0.0f)
				{
					width = viewport_dip_w;
					height = viewport_dip_h;
				}
				placements[root_id] = ScreenRootPlacement(Rect{ 0.0f, 0.0f, width, height });
				continue;
			}

			placements[root_id] = ProjectWorldRoot(policy, root.desc.world, width, height, viewport);
		}
		return placements;
	}

	std::unordered_map<ControlId, Rect> ComputeLayout(TreeModel const& tree, std::unordered_map<ControlId, RootPlacement> const& placements)
	{
		std::unordered_map<ControlId, Rect> rects;
		rects.reserve(tree.m_controls.size());

		for (auto root_id : tree.m_roots)
		{
			auto const& root = tree.m_controls.at(root_id);
			auto placement_it = placements.find(root_id);
			if (placement_it == placements.end())
				throw EngineException(EStatus::InternalError, std::format("ComputeLayout: no placement was computed for root {}", root_id));

			auto const& placement = placement_it->second;

			// Lay the subtree out in the root's own unscaled DIP space so every layout rule stays
			// resolution- and camera-independent, then map the whole subtree onto the placement.
			// For a screen root the mapping is the identity, so its rects are unchanged.
			auto local_w = placement.scale > 0.0f ? placement.rect.w / placement.scale : 0.0f;
			auto local_h = placement.scale > 0.0f ? placement.rect.h / placement.scale : 0.0f;
			Rect local_root{ 0.0f, 0.0f, local_w, local_h };
			rects[root_id] = local_root;
			LayoutChildren(tree, root, local_root, rects);

			// Screen roots are already in their final space; only world roots need the transform,
			// and applying it here keeps hit-testing, semantics and drawing on one rect set.
			if (!IsWorldPolicy(root.desc.root_policy))
				continue;

			TransformSubtree(tree, root_id, placement.rect.x, placement.rect.y, placement.scale, rects);
		}
		return rects;
	}
}
