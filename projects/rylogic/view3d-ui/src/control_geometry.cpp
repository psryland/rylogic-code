//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "control_geometry.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Shrink 'rect' along the value axis by 'inset' at each end, collapsing to its centre when it is too short.
		Rect InsetAlongAxis(Rect rect, EOrientation orientation, float inset)
		{
			// Keep the centre fixed so a collapsed rectangle still marks the middle of the original.
			switch (orientation)
			{
				case EOrientation::Horizontal:
				{
					auto const w = std::max(0.0f, rect.w - 2.0f * inset);
					return Rect{ rect.x + (rect.w - w) * 0.5f, rect.y, w, rect.h };
				}
				case EOrientation::Vertical:
				{
					auto const h = std::max(0.0f, rect.h - 2.0f * inset);
					return Rect{ rect.x, rect.y + (rect.h - h) * 0.5f, rect.w, h };
				}
				default:
				{
					throw EngineException(EStatus::UnknownType, "unknown orientation");
				}
			}
		}
	}

	StyleRecord const& StyleOf(TreeModel const& tree, StyleId style_id)
	{
		// Unset or unknown ids fall back to the built-in style.
		if (style_id == 0)
			return TreeModel::DefaultStyle();

		auto it = tree.m_styles.find(style_id);
		return it != tree.m_styles.end() ? it->second : TreeModel::DefaultStyle();
	}

	TemplateDesc const* TemplateOf(TreeModel const& tree, ControlNode const& node)
	{
		// Accepted trees only reference existing templates, but an unset id means built-in drawing.
		if (node.desc.template_id == 0)
			return nullptr;

		auto it = tree.m_templates.find(node.desc.template_id);
		return it != tree.m_templates.end() ? &it->second.desc : nullptr;
	}

	TemplatePart const* FindPart(TemplateDesc const* tmpl, ETemplatePart role)
	{
		// Search the declared parts in order so the first matching role wins.
		if (tmpl == nullptr)
			return nullptr;

		for (auto i = std::uint32_t{}; i != tmpl->part_count; ++i)
		{
			if (tmpl->parts[i].role == role)
				return &tmpl->parts[i];
		}
		return nullptr;
	}

	Rect PartRect(Rect bounds, TemplatePart const& part, float scale)
	{
		// Margins are DIP lengths, so they take the root's scale like other style lengths.
		auto const l = part.margin_left * scale;
		auto const t = part.margin_top * scale;
		auto const r = part.margin_right * scale;
		auto const b = part.margin_bottom * scale;
		return Rect{ bounds.x + l, bounds.y + t, std::max(0.0f, bounds.w - l - r), std::max(0.0f, bounds.h - t - b) };
	}

	ValueGeometry ValueGeometryOf(TreeModel const& tree, ControlNode const& node, Rect bounds, float border, float scale)
	{
		// Templates place the track and thumb through their parts.
		auto const orientation = node.desc.orientation;
		if (auto const* tmpl = TemplateOf(tree, node); tmpl != nullptr)
		{
			// No Track part means the track fills the control. No Thumb part means no thumb, so values use the whole track.
			auto const* track_part = FindPart(tmpl, ETemplatePart::Track);
			auto const* thumb_part = FindPart(tmpl, ETemplatePart::Thumb);
			auto const track = track_part != nullptr ? PartRect(bounds, *track_part, scale) : bounds;
			auto const thumb_size = thumb_part != nullptr ? Vec2{ thumb_part->width * scale, thumb_part->height * scale } : Vec2{ 0.0f, 0.0f };
			auto const thumb_length = orientation == EOrientation::Vertical ? thumb_size.y : thumb_size.x;
			return ValueGeometry{
				.orientation = orientation,
				.track = track,
				.travel = InsetAlongAxis(track, orientation, thumb_length * 0.5f),
				.thumb_size = thumb_size,
			};
		}

		// Built-in drawing keeps the value inside the control's border.
		auto const inset = std::clamp(border, 0.0f, std::min(bounds.w, bounds.h) * 0.5f);
		auto const track = Rect{ bounds.x + inset, bounds.y + inset, std::max(0.0f, bounds.w - 2.0f * inset), std::max(0.0f, bounds.h - 2.0f * inset) };
		switch (node.desc.type)
		{
			case EControlType::Slider:
			{
				// The built-in thumb spans the track's cross axis and is half as long as that, but at least 4 DIP.
				auto thumb_size = Vec2{ 0.0f, 0.0f };
				switch (orientation)
				{
					case EOrientation::Horizontal:
					{
						thumb_size = Vec2{ std::min(track.w, std::max(4.0f * scale, track.h * 0.5f)), track.h };
						break;
					}
					case EOrientation::Vertical:
					{
						thumb_size = Vec2{ track.w, std::min(track.h, std::max(4.0f * scale, track.w * 0.5f)) };
						break;
					}
					default:
					{
						throw EngineException(EStatus::UnknownType, "unknown orientation");
					}
				}
				auto const thumb_length = orientation == EOrientation::Vertical ? thumb_size.y : thumb_size.x;
				return ValueGeometry{
					.orientation = orientation,
					.track = track,
					.travel = InsetAlongAxis(track, orientation, thumb_length * 0.5f),
					.thumb_size = thumb_size,
				};
			}
			case EControlType::ProgressBar:
			{
				// A progress bar has no thumb, so its value uses the whole track.
				return ValueGeometry{
					.orientation = orientation,
					.track = track,
					.travel = track,
					.thumb_size = Vec2{ 0.0f, 0.0f },
				};
			}
			default:
			{
				throw EngineException(EStatus::InvalidArgument, "value geometry needs a Slider or ProgressBar");
			}
		}
	}

	float ValueFraction(ControlDesc const& desc)
	{
		// Sliders have an authored range; a progress value is already a fraction.
		switch (desc.type)
		{
			case EControlType::Slider:
			{
				return std::clamp((desc.value - desc.minimum) / (desc.maximum - desc.minimum), 0.0f, 1.0f);
			}
			case EControlType::ProgressBar:
			{
				return std::clamp(desc.value, 0.0f, 1.0f);
			}
			default:
			{
				throw EngineException(EStatus::InvalidArgument, "value fraction needs a Slider or ProgressBar");
			}
		}
	}

	float AxisPosition(ValueGeometry const& geom, float fraction)
	{
		// Horizontal values grow to the right, vertical values grow upward.
		switch (geom.orientation)
		{
			case EOrientation::Horizontal:
			{
				return geom.travel.x + geom.travel.w * fraction;
			}
			case EOrientation::Vertical:
			{
				return geom.travel.y + geom.travel.h * (1.0f - fraction);
			}
			default:
			{
				throw EngineException(EStatus::UnknownType, "unknown orientation");
			}
		}
	}

	float FractionAtPoint(ValueGeometry const& geom, Vec2 point)
	{
		// Invert AxisPosition, clamping points beyond either end of the travel to that end.
		switch (geom.orientation)
		{
			case EOrientation::Horizontal:
			{
				if (geom.travel.w <= 0.0f)
					return point.x < geom.travel.x ? 0.0f : 1.0f;

				return std::clamp((point.x - geom.travel.x) / geom.travel.w, 0.0f, 1.0f);
			}
			case EOrientation::Vertical:
			{
				if (geom.travel.h <= 0.0f)
					return point.y > geom.travel.y ? 0.0f : 1.0f;

				return std::clamp(1.0f - (point.y - geom.travel.y) / geom.travel.h, 0.0f, 1.0f);
			}
			default:
			{
				throw EngineException(EStatus::UnknownType, "unknown orientation");
			}
		}
	}

	Rect ThumbRect(ValueGeometry const& geom, float fraction)
	{
		// Centre the thumb on the value position along the axis and on the track across it.
		auto const position = AxisPosition(geom, fraction);
		switch (geom.orientation)
		{
			case EOrientation::Horizontal:
			{
				auto const cy = geom.track.y + geom.track.h * 0.5f;
				return Rect{ position - geom.thumb_size.x * 0.5f, cy - geom.thumb_size.y * 0.5f, geom.thumb_size.x, geom.thumb_size.y };
			}
			case EOrientation::Vertical:
			{
				auto const cx = geom.track.x + geom.track.w * 0.5f;
				return Rect{ cx - geom.thumb_size.x * 0.5f, position - geom.thumb_size.y * 0.5f, geom.thumb_size.x, geom.thumb_size.y };
			}
			default:
			{
				throw EngineException(EStatus::UnknownType, "unknown orientation");
			}
		}
	}

	Rect CutAlongAxis(Rect rect, EOrientation orientation, float a, float b)
	{
		// Order and clamp the two coordinates to the rectangle's extent on the axis.
		switch (orientation)
		{
			case EOrientation::Horizontal:
			{
				auto const lo = std::clamp(std::min(a, b), rect.x, rect.x + rect.w);
				auto const hi = std::clamp(std::max(a, b), rect.x, rect.x + rect.w);
				return Rect{ lo, rect.y, hi - lo, rect.h };
			}
			case EOrientation::Vertical:
			{
				auto const lo = std::clamp(std::min(a, b), rect.y, rect.y + rect.h);
				auto const hi = std::clamp(std::max(a, b), rect.y, rect.y + rect.h);
				return Rect{ rect.x, lo, rect.w, hi - lo };
			}
			default:
			{
				throw EngineException(EStatus::UnknownType, "unknown orientation");
			}
		}
	}

	Rect TextRectOf(TreeModel const& tree, ControlNode const& node, Rect bounds, float scale)
	{
		// Only a template's Text part moves the text away from the control bounds.
		auto const* text_part = FindPart(TemplateOf(tree, node), ETemplatePart::Text);
		return text_part != nullptr ? PartRect(bounds, *text_part, scale) : bounds;
	}
}
