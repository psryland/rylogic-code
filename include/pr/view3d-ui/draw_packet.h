//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Internal, renderer-neutral, immutable draw-packet data produced by UiEngine::Update(). This is
// not part of the public C ABI; it is the internal hook the View3D-12 host uses (in a later
// milestone) to record retained visual draw calls. It is a plain C++ header because the parent
// renderer links against this module natively rather than through the dependency-minimal ABI.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// Internal (non-ABI) horizontal placement rule for a TextPresenter DrawItem, resolved once by
	// draw_packet_builder.cpp per control type and applied by the renderer against the actual
	// shaped run width it alone knows (TextShaper::Shape's returned total advance): Left starts
	// the run at bounds.x + text_inset_dip (a bare Text label uses inset 0; a TextBox insets its
	// content from the left edge like a conventional text field); Center places the run in the
	// middle of bounds.w, ignoring text_inset_dip, used for a Button's label. This is purely a
	// renderer hint - it never reaches the public C ABI - so it uses ordinary C++ enum semantics
	// rather than the ABI's fixed-width EFoo convention.
	enum class ETextAlign
	{
		Left = 0,
		Center = 1,
		Count = 2,
	};

	// One resolved visual primitive instance ready for a renderer to draw, in DIP space. Ordering
	// within DrawPacket::items is a stable pre-order tree/part traversal so submission order is
	// deterministic given identical accepted state (section 9.3). 'text'/'font_family'/'font_size'/
	// 'text_align'/'text_inset_dip' and the text-edit decoration fields below are only populated
	// when primitive == TextPresenter; every other field applies uniformly.
	struct DrawItem
	{
		ControlId control_id;
		EVisualPrimitive primitive;
		Rect bounds;
		Colour fill;
		Colour border_colour;
		float border_thickness;
		float corner_radius;
		float opacity;
		std::string text;        // UTF-8; the accepted text, or a focused TextBox's live pending/composing text
		std::string font_family; // resolved from ControlDesc::font_resource_id; empty selects the built-in fallback
		float font_size;         // DIPs; resolved from the referenced Font resource, or a fixed default
		ETextAlign text_align;   // renderer hint only, see ETextAlign; not part of the public C ABI
		float text_inset_dip;    // DIPs; only applied when text_align == Left (see ETextAlign)

		// Text-edit decorations, as UTF-8 byte offsets into 'text'. They are populated only for the
		// focused editable control, so an unfocused field costs no extra draws. The renderer
		// resolves all three from the same shaped layout as the glyphs, which is what keeps a
		// highlight, an underline and a caret from ever disagreeing with the text they decorate.
		std::uint32_t selection_start;    // == selection_end when nothing is selected
		std::uint32_t selection_end;
		std::uint32_t composition_start;  // start of an active IME composition within 'text'
		std::uint32_t composition_length; // 0 when no composition is active
		std::uint32_t caret_offset;
		std::int32_t caret_visible;
	};

	// A contiguous run of DrawPacket::items belonging to one root, carrying the host-stage policy
	// and per-root depth/fade parameters the renderer needs. Grouping is what lets the renderer
	// select the items for one host pass without inspecting individual controls: each pass draws
	// only the groups whose policy names it, in group order.
	struct DrawGroup
	{
		ControlId root_id;
		ERootPolicy policy;
		std::uint32_t first_item;        // Index of the group's first item in DrawPacket::items.
		std::uint32_t item_count;
		float clip_depth;                // Normalised [0, 1] device depth recorded for depth-tested/occlusion-faded roots.
		float view_depth;                // Distance from the camera to the root's anchor along the look direction, world units.
		float occlusion_min_opacity;     // Opacity floor reached when fully occluded; only used by ERootPolicy::OcclusionFaded.
		float occlusion_fade_depth;      // World units of occluding depth over which opacity falls to the floor.
		float occlusion_depth_bias;      // World units subtracted from the sampled scene depth before comparison.
	};

	// One immutable render snapshot for a context, replaced wholesale by each Update() call. The
	// renderer never records into a partially-updated packet: it either has the previous complete
	// snapshot or the next complete one.
	struct DrawPacket
	{
		std::uint64_t accepted_revision;
		std::uint64_t visual_sequence;
		float viewport_dpi;
		std::vector<DrawItem> items;
		std::vector<DrawGroup> groups; // One per visible root, in tree root order; groups partition 'items'.
	};
}
