//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Shared resolution of a control's font and its text's horizontal placement. Caret placement
// (input.cpp), semantic text ranges (semantics.cpp) and the draw packet (draw_packet_builder.cpp)
// must all describe the same run of shaped text, so the rules that turn a control descriptor into
// a family/size/colour and a run origin live here rather than being restated by each of them.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "pr/view3d-ui/draw_packet.h"
#include "tree.h"
#include "world.h"

namespace pr::view3d::ui
{
	// The only family this module substitutes when a control names no font at all. A control that
	// *does* name a font resource keeps that resource's family verbatim, so an unknown family
	// surfaces as EStatus::MissingAsset from the text shaper instead of being quietly replaced.
	constexpr char const* DefaultFontFamily = "Segoe UI";
	constexpr float DefaultFontSize = 14.0f;
	// Opaque black, used when a Font resource specifies no text colour. A resource authored (or
	// left zero-initialized) before text colour existed would otherwise be fully transparent, which
	// would make existing content invisible rather than merely unstyled.
	constexpr Colour DefaultFontColour{ 0.0f, 0.0f, 0.0f, 1.0f };

	// How far a TextBox insets its content from its own left edge, in local DIPs, like a
	// conventional text field.
	constexpr float TextBoxHorizontalInsetDip = 8.0f;

	// A Font resource's family, DIP size and text colour with all "unspecified" fallbacks applied.
	// 'family' is empty only when the control named no font resource, which is the one case the
	// text shaper is permitted to answer with its built-in fallback face.
	struct ResolvedFont
	{
		std::string family;
		float size;
		Colour colour;
	};

	// Resolves a control's declared font_resource_id to family/size/colour. font_resource_id is
	// already validated as a Font resource at transaction-apply time, so only the "unset" cases
	// remain: id 0, an unregistered id, a non-positive size, or a fully transparent colour.
	ResolvedFont ResolveControlFont(TreeModel const& tree, ResourceId font_resource_id);

	// Horizontal placement of a text-bearing control's run within its own bounds.
	struct TextPlacement
	{
		ETextAlign align;
		float inset_dip;
	};

	// The placement rule for a text-bearing control type. Throws EngineException(InvalidArgument)
	// for a type that displays no text of its own.
	TextPlacement TextPlacementFor(EControlType type);

	// The apparent screen-DIPs-per-local-DIP scale of the root that owns 'id', or 1.0 when no
	// placement has been computed for it (which is the correct identity for a screen-space root).
	float ControlScale(TreeModel const& tree, std::unordered_map<ControlId, RootPlacement> const* placements, ControlId id);
}
