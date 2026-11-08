//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Shared geometry of a control's drawing template parts (implementation-plan.md section 6.3).
// The draw packet builder draws these rectangles and input maps pointer positions through them,
// so both must use exactly the same rules. All rectangles are in the same screen space as layout
// rects, and every DIP length taken from a template part is multiplied by the root's DIP scale.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "tree.h"

namespace pr::view3d::ui
{
	// The accepted style with id 'style_id', or the built-in style for 0 or an unknown id.
	StyleRecord const& StyleOf(TreeModel const& tree, StyleId style_id);

	// The accepted template a control draws through, or null when it uses built-in drawing.
	TemplateDesc const* TemplateOf(TreeModel const& tree, ControlNode const& node);

	// The first part with 'role' in 'tmpl', or null when 'tmpl' is null or has no such part.
	TemplatePart const* FindPart(TemplateDesc const* tmpl, ETemplatePart role);

	// 'bounds' inset by the part's margins. The result never has a negative size.
	Rect PartRect(Rect bounds, TemplatePart const& part, float scale);

	// Layout of a value control (Slider or ProgressBar) along its value axis.
	struct ValueGeometry
	{
		// The axis values are shown along. Vertical values increase upward.
		EOrientation orientation;

		// The rectangle the value is shown in.
		Rect track;

		// The rectangle the value position moves over: the track inset along the axis by half the thumb length,
		// so a thumb at either end of the range stays inside the track.
		Rect travel;

		// The thumb width and height. Zero when the control draws no thumb.
		Vec2 thumb_size;
	};

	// The value geometry of a Slider or ProgressBar laid out in 'bounds'. 'border' is the screen-space inset that built-in
	// drawing (no template) keeps inside the control's border; templates ignore it and use their Track and Thumb parts.
	ValueGeometry ValueGeometryOf(TreeModel const& tree, ControlNode const& node, Rect bounds, float border, float scale);

	// The part of the range the control's accepted value represents, in [0, 1].
	float ValueFraction(ControlDesc const& desc);

	// The screen coordinate on the value axis (x when horizontal, y when vertical) that shows 'fraction' of the range.
	float AxisPosition(ValueGeometry const& geom, float fraction);

	// The part of the range, in [0, 1], whose axis position is nearest to 'point'.
	float FractionAtPoint(ValueGeometry const& geom, Vec2 point);

	// The thumb rectangle for 'fraction' of the range: centred on its axis position and on the track's cross-axis centre.
	Rect ThumbRect(ValueGeometry const& geom, float fraction);

	// The part of 'rect' between the two value-axis coordinates 'a' and 'b' (in either order), clamped to 'rect'.
	Rect CutAlongAxis(Rect rect, EOrientation orientation, float a, float b);

	// The rectangle a control lays out its text in: its Text part rectangle, or 'bounds' when it has none.
	Rect TextRectOf(TreeModel const& tree, ControlNode const& node, Rect bounds, float scale);
}
