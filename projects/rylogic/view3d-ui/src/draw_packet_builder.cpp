//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "draw_packet_builder.h"
#include "control_geometry.h"
#include "text_layout.h"

namespace pr::view3d::ui
{
	namespace
	{
		// The box primitive that draws 'visual': rounded when it has a corner radius.
		EVisualPrimitive BoxPrimitive(StyleVisual const& visual)
		{
			return visual.corner_radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
		}

		EVisualPrimitive PrimitiveFor(EControlType type, StyleVisual const& visual)
		{
			switch (type)
			{
				case EControlType::Text: return EVisualPrimitive::TextPresenter;
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::TextBox:
				case EControlType::Button:
				case EControlType::ProgressBar:
				case EControlType::Slider:
				case EControlType::ComboBox:
				{
					return BoxPrimitive(visual);
				}

				case EControlType::Count:
				default:
				{
					throw EngineException(EStatus::InvalidArgument, "unknown control type");
				}
			}
		}

		// Paint the image brush of 'visual' over 'rect'. A brush with slice insets is split into a 3 x 3 grid of
		// Image items: corners keep their scaled size, edges stretch along one axis, and the centre stretches both ways.
		void AppendImage(TreeModel const& tree, ControlId id, Rect rect, StyleVisual const& visual, float scale, DrawPacket& out)
		{
			// Styles without an image, or with an empty destination, draw nothing.
			auto const& brush = visual.image;
			if (brush.image_id == 0 || rect.w <= 0.0f || rect.h <= 0.0f)
				return;

			auto const resource_it = tree.m_resources.find(brush.image_id);
			if (resource_it == tree.m_resources.end())
				return;

			// A source with no area selects the whole image.
			auto const& image = resource_it->second.desc;
			auto const image_w = static_cast<float>(image.image_width);
			auto const image_h = static_cast<float>(image.image_height);
			auto const source = brush.source.w > 0.0f && brush.source.h > 0.0f ? brush.source : Rect{ 0.0f, 0.0f, image_w, image_h };

			// Source slices that overlap are shrunk in proportion so they meet instead of crossing.
			auto src_l = brush.slice_left;
			auto src_r = brush.slice_right;
			auto src_t = brush.slice_top;
			auto src_b = brush.slice_bottom;
			if (src_l + src_r > source.w)
			{
				auto const f = source.w / (src_l + src_r);
				src_l *= f;
				src_r *= f;
			}
			if (src_t + src_b > source.h)
			{
				auto const f = source.h / (src_t + src_b);
				src_t *= f;
				src_b *= f;
			}

			// Destination borders are the source slices in DIP, scaled by the root, then shrunk in proportion when the
			// destination is too small to hold both borders.
			auto const k = (brush.slice_scale == 0.0f ? 1.0f : brush.slice_scale) * scale;
			auto dst_l = src_l * k;
			auto dst_r = src_r * k;
			auto dst_t = src_t * k;
			auto dst_b = src_b * k;
			if (dst_l + dst_r > rect.w)
			{
				auto const f = rect.w / (dst_l + dst_r);
				dst_l *= f;
				dst_r *= f;
			}
			if (dst_t + dst_b > rect.h)
			{
				auto const f = rect.h / (dst_t + dst_b);
				dst_t *= f;
				dst_b *= f;
			}

			// Grid lines for the destination and source. Without slices only the outer lines matter and one cell is emitted.
			auto const sliced = src_l > 0.0f || src_r > 0.0f || src_t > 0.0f || src_b > 0.0f;
			float const dx[4] = { rect.x, rect.x + dst_l, rect.x + rect.w - dst_r, rect.x + rect.w };
			float const dy[4] = { rect.y, rect.y + dst_t, rect.y + rect.h - dst_b, rect.y + rect.h };
			float const sx[4] = { source.x, source.x + src_l, source.x + source.w - src_r, source.x + source.w };
			float const sy[4] = { source.y, source.y + src_t, source.y + source.h - src_b, source.y + source.h };
			auto const cells = sliced ? 3 : 1;
			for (auto row = 0; row != cells; ++row)
			{
				// Each cell maps its source rectangle, normalised by the image size, onto its destination rectangle.
				for (auto col = 0; col != cells; ++col)
				{
					// A single cell spans the outer grid lines; sliced cells span adjacent lines.
					auto const c0 = sliced ? col : 0;
					auto const c1 = sliced ? col + 1 : 3;
					auto const r0 = sliced ? row : 0;
					auto const r1 = sliced ? row + 1 : 3;
					auto const bounds = Rect{ dx[c0], dy[r0], dx[c1] - dx[c0], dy[r1] - dy[r0] };
					if (bounds.w <= 0.0f || bounds.h <= 0.0f)
						continue;

					auto item = DrawItem{};
					item.control_id = id;
					item.primitive = EVisualPrimitive::Image;
					item.bounds = bounds;
					item.fill = brush.tint;
					item.opacity = visual.opacity;
					item.image_texture = image.texture;
					item.image_width = image.image_width;
					item.image_height = image.image_height;
					item.uv = Rect{ sx[c0] / image_w, sy[r0] / image_h, (sx[c1] - sx[c0]) / image_w, (sy[r1] - sy[r0]) / image_h };
					out.items.push_back(std::move(item));
				}
			}
		}

		// Paint a styled box over 'rect': fill and border, then the style's image brush.
		void AppendBox(TreeModel const& tree, ControlId id, Rect rect, StyleVisual const& visual, float scale, DrawPacket& out)
		{
			// Layout rects are already in screen space, but style-derived lengths are authored
			// in the root's local DIP space, so they take the root's apparent scale here.
			auto box = DrawItem{};
			box.control_id = id;
			box.primitive = BoxPrimitive(visual);
			box.bounds = rect;
			box.fill = visual.fill;
			box.border_colour = visual.border_colour;
			box.border_thickness = visual.border_thickness * scale;
			box.corner_radius = visual.corner_radius * scale;
			box.opacity = visual.opacity;
			out.items.push_back(std::move(box));

			// The image is drawn over the fill and border.
			AppendImage(tree, id, rect, visual, scale, out);
		}

		// Paint a template part box, skipping empty rectangles.
		void AppendPartBox(TreeModel const& tree, ControlId id, Rect rect, StyleVisual const& visual, float scale, DrawPacket& out)
		{
			// Parts can collapse to nothing, for example an indicator at the minimum value.
			if (rect.w <= 0.0f || rect.h <= 0.0f)
				return;

			AppendBox(tree, id, rect, visual, scale, out);
		}

		// The visual a template part draws with. A part without its own style uses the control's resolved (possibly
		// transitioning) visual. A part style uses its visual for the control's active state channel, without a transition.
		StyleVisual PartVisual(TreeModel const& tree, ControlId id, TemplatePart const& part, StyleVisual const& control_visual, StyleResolver const& styles)
		{
			// Part styles follow the control's state but keep no transition state of their own.
			if (part.style_id == 0)
				return control_visual;

			return StyleOf(tree, part.style_id).desc.visuals[static_cast<std::size_t>(styles.ActiveChannel(id))];
		}

		// The axis coordinate of the minimum-value end of 'rect': its left edge, or its bottom edge when vertical.
		float MinimumEdge(Rect rect, EOrientation orientation)
		{
			// Vertical values increase upward, so the minimum is at the bottom.
			switch (orientation)
			{
				case EOrientation::Horizontal: { return rect.x; }
				case EOrientation::Vertical: { return rect.y + rect.h; }
				default: { throw EngineException(EStatus::UnknownType, "unknown orientation"); }
			}
		}

		// Paint the built-in slider: a filled indicator plus a compact thumb, using the existing quad primitives
		// and style colours.
		void AppendBuiltinSlider(ControlNode const& node, ValueGeometry const& geom, StyleVisual const& visual, float scale, DrawPacket& out)
		{
			// Derive indicator and thumb geometry only from the accepted descriptor and current DPI scale.
			auto const fraction = ValueFraction(node.desc);
			if (geom.track.w <= 0.0f || geom.track.h <= 0.0f)
				return;

			// Foreground fills the accepted portion of the track, ending at the thumb centre so no track shows between
			// the fill and the thumb's rounded corners. The minimum end follows the track's inner corner radius, and a
			// square box over the value end removes the rounded corners there, so the fill meets the thumb cleanly.
			if (fraction > 0.0f)
			{
				auto const fill = CutAlongAxis(geom.track, geom.orientation, MinimumEdge(geom.track, geom.orientation), AxisPosition(geom, fraction));
				auto const border = visual.border_thickness * scale;
				auto const radius = std::clamp(visual.corner_radius * scale - border, 0.0f, std::min(fill.w, fill.h) * 0.5f);

				auto indicator = DrawItem{};
				indicator.control_id = node.desc.id;
				indicator.bounds = fill;
				indicator.fill = visual.foreground;
				indicator.opacity = visual.opacity;
				indicator.corner_radius = radius;
				indicator.primitive = radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
				out.items.push_back(indicator);

				if (radius > 0.0f)
				{
					// Square off the value end so only the minimum end is rounded.
					switch (geom.orientation)
					{
						case EOrientation::Horizontal:
						{
							indicator.bounds = Rect{ fill.x + radius, fill.y, fill.w - radius, fill.h };
							break;
						}
						case EOrientation::Vertical:
						{
							indicator.bounds = Rect{ fill.x, fill.y, fill.w, fill.h - radius };
							break;
						}
						default:
						{
							throw EngineException(EStatus::UnknownType, "unknown orientation");
						}
					}
					indicator.corner_radius = 0.0f;
					indicator.primitive = EVisualPrimitive::SolidBox;
					out.items.push_back(std::move(indicator));
				}
			}

			// The thumb is drawn last, in its own colour, so it sits on top of the indicator.
			auto thumb = DrawItem{};
			thumb.control_id = node.desc.id;
			thumb.bounds = ThumbRect(geom, fraction);
			thumb.fill = visual.thumb;
			thumb.opacity = visual.opacity;
			thumb.corner_radius = std::min(thumb.bounds.w, thumb.bounds.h) * 0.5f;
			thumb.primitive = EVisualPrimitive::RoundedBox;
			out.items.push_back(std::move(thumb));
		}

		// The part of the range a progress indicator covers: [start, start + length]. An indeterminate indicator is a quarter of
		// the range whose centre travels smoothly from the minimum to the maximum and back once every 2400 ms, so near either end
		// part of it lies outside [0, 1] and is clipped by the track. Host time, rather than update count or transactions, drives
		// the animation.
		std::pair<float, float> ProgressSpan(ControlDesc const& desc, double time_ms)
		{
			// Determinate progress always starts at the minimum.
			if (desc.is_indeterminate == 0)
				return { 0.0f, ValueFraction(desc) };

			auto const length = 0.25f;
			auto const centre = static_cast<float>(0.5 - 0.5 * std::cos(std::fmod(time_ms, 2400.0) * (6.283185307179586 / 2400.0)));
			return { centre - 0.5f * length, length };
		}

		// The part of 'rect' between axis coordinates 'a' and 'b'. Unlike CutAlongAxis, the span may extend beyond 'rect'.
		Rect SpanAlongAxis(Rect rect, EOrientation orientation, float a, float b)
		{
			// Only the axis extent changes; the cross-axis extent is that of 'rect'.
			auto const lo = std::min(a, b);
			auto const hi = std::max(a, b);
			switch (orientation)
			{
				case EOrientation::Horizontal: { return Rect{ lo, rect.y, hi - lo, rect.h }; }
				case EOrientation::Vertical: { return Rect{ rect.x, lo, rect.w, hi - lo }; }
				default: { throw EngineException(EStatus::UnknownType, "unknown orientation"); }
			}
		}

		// Crop the box and image items from index 'first' onward to 'clip', removing items left with no area. Image items keep
		// their texture mapping, so the visible part of the image is unchanged. Box items are simply resized, so a border or
		// rounded corner appears at a cut edge.
		void ClipItems(DrawPacket& out, std::size_t first, Rect clip)
		{
			// Each item is intersected with 'clip' independently.
			auto keep = first;
			for (auto i = first; i != out.items.size(); ++i)
			{
				// Items wholly outside the clip are dropped.
				auto item = std::move(out.items[i]);
				auto const& b = item.bounds;
				auto const x0 = std::max(b.x, clip.x);
				auto const y0 = std::max(b.y, clip.y);
				auto const x1 = std::min(b.x + b.w, clip.x + clip.w);
				auto const y1 = std::min(b.y + b.h, clip.y + clip.h);
				if (x1 <= x0 || y1 <= y0)
					continue;

				// The image's source rectangle shrinks in proportion to its destination rectangle.
				switch (item.primitive)
				{
					case EVisualPrimitive::Image:
					{
						auto const su = item.uv.w / b.w;
						auto const sv = item.uv.h / b.h;
						item.uv = Rect{ item.uv.x + (x0 - b.x) * su, item.uv.y + (y0 - b.y) * sv, (x1 - x0) * su, (y1 - y0) * sv };
						break;
					}
					case EVisualPrimitive::SolidBox:
					case EVisualPrimitive::RoundedBox:
					{
						item.corner_radius = std::min(item.corner_radius, std::min(x1 - x0, y1 - y0) * 0.5f);
						break;
					}
					default:
					{
						throw EngineException(EStatus::InvalidArgument, "only box and image items can be clipped");
					}
				}
				item.bounds = Rect{ x0, y0, x1 - x0, y1 - y0 };
				out.items[keep++] = std::move(item);
			}
			out.items.resize(keep);
		}

		// Paint the built-in progress indicator within the track's border, without changing accepted state.
		void AppendBuiltinProgress(ControlNode const& node, ValueGeometry const& geom, StyleVisual const& visual, double time_ms, float scale, DrawPacket& out)
		{
			// Reuse the existing quad renderer. CutAlongAxis keeps each indicator inside its track, which crops an indeterminate span at either end.
			auto const [start, length] = ProgressSpan(node.desc, time_ms);
			if (geom.track.w <= 0.0f || geom.track.h <= 0.0f || length == 0.0f)
				return;

			auto const border = std::clamp(visual.border_thickness * scale, 0.0f, std::min(geom.track.w, geom.track.h) * 0.5f);
			auto item = DrawItem{};
			item.control_id = node.desc.id;
			item.bounds = CutAlongAxis(geom.track, geom.orientation, AxisPosition(geom, start), AxisPosition(geom, start + length));
			item.fill = visual.foreground;
			item.opacity = visual.opacity;
			item.corner_radius = std::clamp(visual.corner_radius * scale - border, 0.0f, std::min(item.bounds.w, item.bounds.h) * 0.5f);
			item.primitive = item.corner_radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
			out.items.push_back(std::move(item));
		}

		// Paint one Ticks part: a box per tick in the part's band, at the same axis positions the thumb takes for those
		// values, plus a value label beyond each major tick when the part names a label font.
		void AppendTicks(TreeModel const& tree, ControlNode const& node, TemplatePart const& part, ValueGeometry const& geom, Rect bounds, StyleVisual const& visual, float scale, DrawPacket& out)
		{
			// Ticks step through the control's value range. A slider's step is the default interval; a progress bar has a [0, 1] range.
			auto const& ticks = part.ticks;
			auto const is_slider = node.desc.type == EControlType::Slider;
			auto const minimum = is_slider ? static_cast<double>(node.desc.minimum) : 0.0;
			auto const maximum = is_slider ? static_cast<double>(node.desc.maximum) : 1.0;
			auto const interval = static_cast<double>(ticks.interval > 0.0f ? ticks.interval : node.desc.step);
			if (interval <= 0.0 || maximum <= minimum)
				return;

			// A small tolerance keeps a tick at the maximum when the range is a whole number of intervals.
			auto const band = PartRect(bounds, part, scale);
			auto const tick_count = static_cast<std::uint64_t>(std::floor((maximum - minimum) / interval + 1e-4)) + 1;
			auto const thickness = ticks.thickness * scale;
			auto const gap = ticks.label_gap * scale;
			auto const has_labels = ticks.label_font_id != 0;
			auto const font = has_labels ? ResolveControlFont(tree, ticks.label_font_id) : ResolvedFont{};
			auto const label_h = font.size * scale * 1.5f;
			auto const label_w = font.size * scale * 8.0f;
			for (auto i = std::uint64_t{}; i != tick_count; ++i)
			{
				// Place the tick on the value axis, then size it across the axis from the band edge chosen by 'align'.
				auto const value = minimum + static_cast<double>(i) * interval;
				auto const fraction = static_cast<float>((value - minimum) / (maximum - minimum));
				auto const position = AxisPosition(geom, std::clamp(fraction, 0.0f, 1.0f));
				auto const major = ticks.major_every == 0 || i % ticks.major_every == 0;
				auto const length = (major ? ticks.major_length : ticks.minor_length) * scale;
				auto const cross_lo = geom.orientation == EOrientation::Horizontal ? band.y : band.x;
				auto const cross_hi = geom.orientation == EOrientation::Horizontal ? band.y + band.h : band.x + band.w;
				auto tick_start = 0.0f;
				switch (ticks.align)
				{
					case EAnchorPoint::Min: { tick_start = cross_lo; break; }
					case EAnchorPoint::Centre: { tick_start = (cross_lo + cross_hi - length) * 0.5f; break; }
					case EAnchorPoint::Max: { tick_start = cross_hi - length; break; }
					default: { throw EngineException(EStatus::UnknownType, "unknown tick alignment"); }
				}
				auto const tick_rect = geom.orientation == EOrientation::Horizontal
					? Rect{ position - thickness * 0.5f, tick_start, thickness, length }
					: Rect{ tick_start, position - thickness * 0.5f, length, thickness };
				AppendPartBox(tree, node.desc.id, tick_rect, visual, scale, out);
				if (!major || !has_labels)
					continue;

				// Labels sit beyond the tick's far end: below or right of ticks that start at the Min or Centre of the band,
				// above or left of ticks that start at its Max edge. Labels left of a tick end at the tick, so they align right.
				auto label = DrawItem{};
				label.control_id = node.desc.id;
				label.primitive = EVisualPrimitive::TextPresenter;
				label.fill = font.colour;
				label.opacity = visual.opacity;
				label.text = std::format("{:.{}f}", value, ticks.label_decimals);
				label.font_family = font.family;
				label.font_size = font.size * scale;
				label.text_inset_dip = 0.0f;
				auto const before = ticks.align == EAnchorPoint::Max;
				switch (geom.orientation)
				{
					case EOrientation::Horizontal:
					{
						auto const y = before ? tick_start - gap - label_h : tick_start + length + gap;
						label.bounds = Rect{ position - label_w * 0.5f, y, label_w, label_h };
						label.text_align = ETextAlign::Center;
						break;
					}
					case EOrientation::Vertical:
					{
						auto const x = before ? tick_start - gap - label_w : tick_start + length + gap;
						label.bounds = Rect{ x, position - label_h * 0.5f, label_w, label_h };
						label.text_align = before ? ETextAlign::Right : ETextAlign::Left;
						break;
					}
					default:
					{
						throw EngineException(EStatus::UnknownType, "unknown orientation");
					}
				}
				out.items.push_back(std::move(label));
			}
		}

		// Paint the value parts of a Slider or ProgressBar: built-in visuals without a template, otherwise the template's
		// Track, Indicator, Ticks and Thumb parts in that order.
		void AppendValueParts(TreeModel const& tree, ControlNode const& node, Rect bounds, StyleVisual const& visual, StyleResolver const& styles, double time_ms, float scale, DrawPacket& out)
		{
			// Built-in drawing keeps the value inside the control's border.
			auto const* tmpl = TemplateOf(tree, node);
			auto const geom = ValueGeometryOf(tree, node, bounds, visual.border_thickness * scale, scale);
			if (tmpl == nullptr)
			{
				// Each value control type has its own built-in look.
				switch (node.desc.type)
				{
					case EControlType::Slider: { AppendBuiltinSlider(node, geom, visual, scale, out); return; }
					case EControlType::ProgressBar: { AppendBuiltinProgress(node, geom, visual, time_ms, scale, out); return; }
					default: { throw EngineException(EStatus::InvalidArgument, "value parts need a Slider or ProgressBar"); }
				}
			}

			// The track is drawn under everything else.
			auto const id = node.desc.id;
			if (auto const* track = FindPart(tmpl, ETemplatePart::Track); track != nullptr)
				AppendPartBox(tree, id, geom.track, PartVisual(tree, id, *track, visual, styles), scale, out);

			// The indicator covers its inset track from the minimum end to the value position (or the moving indeterminate span).
			if (auto const* indicator = FindPart(tmpl, ETemplatePart::Indicator); indicator != nullptr)
			{
				auto const rect = PartRect(geom.track, *indicator, scale);
				auto from = MinimumEdge(rect, geom.orientation);
				auto to = from;
				switch (node.desc.type)
				{
					case EControlType::Slider:
					{
						to = AxisPosition(geom, ValueFraction(node.desc));
						break;
					}
					case EControlType::ProgressBar:
					{
						// Progress spans the indicator's own rect, so an indicator with margins still reaches both of its ends.
						auto const [start, length] = ProgressSpan(node.desc, time_ms);
						auto const span_geom = ValueGeometry{ .orientation = geom.orientation, .track = rect, .travel = rect, .thumb_size = Vec2{ 0.0f, 0.0f } };
						if (node.desc.is_indeterminate == 0)
						{
							to = AxisPosition(span_geom, start + length);
							break;
						}

						// An indeterminate indicator is drawn at full size and then cropped to its rect, so an image brush is
						// clipped at the ends rather than squashed. 'to' stays at 'from', so the shared draw below adds nothing.
						auto const first = out.items.size();
						auto const span = SpanAlongAxis(rect, geom.orientation, AxisPosition(span_geom, start), AxisPosition(span_geom, start + length));
						AppendPartBox(tree, id, span, PartVisual(tree, id, *indicator, visual, styles), scale, out);
						ClipItems(out, first, rect);
						break;
					}
					default:
					{
						throw EngineException(EStatus::InvalidArgument, "value parts need a Slider or ProgressBar");
					}
				}
				AppendPartBox(tree, id, CutAlongAxis(rect, geom.orientation, from, to), PartVisual(tree, id, *indicator, visual, styles), scale, out);
			}

			// Every Ticks part draws its own band, so ticks can appear on both sides of the track.
			for (auto i = std::uint32_t{}; i != tmpl->part_count; ++i)
			{
				auto const& part = tmpl->parts[i];
				if (part.role == ETemplatePart::Ticks)
					AppendTicks(tree, node, part, geom, bounds, PartVisual(tree, id, part, visual, styles), scale, out);
			}

			// The thumb sits on top of the track, indicator and ticks.
			if (auto const* thumb = FindPart(tmpl, ETemplatePart::Thumb); thumb != nullptr)
				AppendPartBox(tree, id, ThumbRect(geom, ValueFraction(node.desc)), PartVisual(tree, id, *thumb, visual, styles), scale, out);
		}

		// Paint the closed ComboBox face using the accepted selected item and a fixed drop-down glyph.
		void AppendComboBoxFace(TreeModel const& tree, ControlNode const& node, Rect bounds, float scale, DrawPacket& out)
		{
			// The closed box shows only the caller-authoritative selected item plus a glyph indicator.
			auto const& style_record = StyleOf(tree, node.desc.style_id);
			auto const normal = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Normal)];
			auto const font = ResolveControlFont(tree, node.desc.font_resource_id);
			auto const placement = TextPlacementFor(EControlType::TextBox);
			auto const selected_text = node.desc.selected_index >= 0 && static_cast<std::uint32_t>(node.desc.selected_index) < node.combo_items.size() ? node.combo_items[static_cast<std::size_t>(node.desc.selected_index)] : std::string{};
			if (!selected_text.empty())
			{
				DrawItem text_item{};
				text_item.control_id = node.desc.id;
				text_item.primitive = EVisualPrimitive::TextPresenter;
				text_item.bounds = Rect{ bounds.x, bounds.y, std::max(0.0f, bounds.w - 24.0f * scale), bounds.h };
				text_item.fill = font.colour;
				text_item.opacity = normal.opacity;
				text_item.text = selected_text;
				text_item.font_family = font.family;
				text_item.font_size = font.size * scale;
				text_item.text_align = placement.align;
				text_item.text_inset_dip = placement.inset_dip * scale;
				out.items.push_back(std::move(text_item));
			}

			DrawItem glyph{};
			glyph.control_id = node.desc.id;
			glyph.primitive = EVisualPrimitive::TextPresenter;
			glyph.bounds = Rect{ bounds.x + std::max(0.0f, bounds.w - 24.0f * scale), bounds.y, 24.0f * scale, bounds.h };
			glyph.fill = font.colour;
			glyph.opacity = normal.opacity;
			glyph.text = "\xE2\x96\xBE";
			glyph.font_family = font.family;
			glyph.font_size = font.size * scale;
			glyph.text_align = ETextAlign::Center;
			glyph.text_inset_dip = 0.0f;
			out.items.push_back(std::move(glyph));
		}

		// Paint the transient ComboBox popup as a topmost draw group.
		void AppendComboBoxPopup(TreeModel const& tree, ControlNode const& node, std::unordered_map<ControlId, Rect> const& layout, InputState const& input_state, ViewportState const& viewport, float scale, DrawPacket& out)
		{
			// A closed or empty popup contributes no transient visuals.
			if (input_state.m_open_combo_id != node.desc.id)
				return;

			auto const popup = ComboBoxPopupRect(tree, layout, viewport, node.desc.id);
			if (popup.w <= 0.0f || popup.h <= 0.0f)
				return;

			auto const& style_record = StyleOf(tree, node.desc.style_id);
			auto const normal = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Normal)];
			auto const hover = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Hover)];
			auto const selected = style_record.desc.visuals[static_cast<std::size_t>(EStateChannel::Selected)];
			auto const font = ResolveControlFont(tree, node.desc.font_resource_id);
			auto const placement = TextPlacementFor(EControlType::TextBox);

			DrawItem background{};
			background.control_id = node.desc.id;
			background.primitive = normal.corner_radius > 0.0f ? EVisualPrimitive::RoundedBox : EVisualPrimitive::SolidBox;
			background.bounds = popup;
			background.fill = normal.fill;
			background.border_colour = normal.border_colour;
			background.border_thickness = normal.border_thickness * scale;
			background.corner_radius = normal.corner_radius * scale;
			background.opacity = normal.opacity;
			out.items.push_back(std::move(background));

			auto const row_h = popup.h / static_cast<float>(std::max<std::uint32_t>(1U, std::min(node.desc.combo_item_count, node.desc.max_visible_items)));
			auto const visible_count = std::min(node.desc.combo_item_count - input_state.m_combo_scroll_offset, node.desc.max_visible_items);
			for (auto row = std::uint32_t{}; row != visible_count; ++row)
			{
				auto const item_index = input_state.m_combo_scroll_offset + row;
				auto const row_bounds = Rect{ popup.x, popup.y + row_h * static_cast<float>(row), popup.w, row_h };
				auto const& row_visual = static_cast<std::int32_t>(item_index) == input_state.m_combo_highlight_index ? hover : static_cast<std::int32_t>(item_index) == node.desc.selected_index ? selected : normal;
				DrawItem row_box{};
				row_box.control_id = node.desc.id;
				row_box.primitive = EVisualPrimitive::SolidBox;
				row_box.bounds = row_bounds;
				row_box.fill = row_visual.fill;
				row_box.opacity = row_visual.opacity;
				out.items.push_back(std::move(row_box));

				DrawItem row_text{};
				row_text.control_id = node.desc.id;
				row_text.primitive = EVisualPrimitive::TextPresenter;
				row_text.bounds = row_bounds;
				row_text.fill = font.colour;
				row_text.opacity = row_visual.opacity;
				row_text.text = item_index < node.combo_items.size() ? node.combo_items[item_index] : std::string{};
				row_text.font_family = font.family;
				row_text.font_size = font.size * scale;
				row_text.text_align = placement.align;
				row_text.text_inset_dip = placement.inset_dip * scale;
				out.items.push_back(std::move(row_text));
			}
		}

		void Walk(TreeModel const& tree, ControlId id, std::unordered_map<ControlId, Rect> const& layout, ViewportState const& viewport, StyleResolver& styles, InputState const& input_state, double time_ms, float scale, DrawPacket& out)
		{
			auto const& node = tree.m_controls.at(id);
			if (!IsVisible(node.desc.visibility))
			{
				// An invisible control hides its whole subtree from the draw packet, matching
				// hit-test/tab-order; record it so a later Update() where it becomes visible again
				// fires exactly one Visibility transition (style.h::StyleResolver::MarkInvisible).
				styles.MarkInvisible(id);
				return;
			}

			auto const& style_record = StyleOf(tree, node.desc.style_id);
			auto visual = styles.Resolve(node, style_record, input_state.m_hover_id, input_state.m_pressed_id, input_state.m_focus_id, time_ms);
			auto bounds = layout.at(id);
			auto primitive = PrimitiveFor(node.desc.type, visual);
			auto const* tmpl = TemplateOf(tree, node);

			// A template's Background part replaces the control's own box. Otherwise Root/Panel/TextBox/Button/ProgressBar/
			// Slider/ComboBox paint their style box first; a bare Text control has no box of its own (it is itself the
			// TextPresenter item emitted below).
			if (auto const* background = FindPart(tmpl, ETemplatePart::Background); background != nullptr)
				AppendPartBox(tree, id, PartRect(bounds, *background, scale), PartVisual(tree, id, *background, visual, styles), scale, out);
			else if (primitive != EVisualPrimitive::TextPresenter)
				AppendBox(tree, id, bounds, visual, scale, out);

			// A Text part with its own style draws a box behind the text, such as the inner well of a text box. Without its own
			// style the part only places the text, because drawing the control's style again would repeat the control's box.
			if (auto const* text_part = FindPart(tmpl, ETemplatePart::Text); text_part != nullptr && text_part->style_id != 0)
				AppendPartBox(tree, id, PartRect(bounds, *text_part, scale), PartVisual(tree, id, *text_part, visual, styles), scale, out);

			// Value controls draw their track, indicator, ticks and thumb; labels remain separate retained Text controls.
			switch (node.desc.type)
			{
				case EControlType::ProgressBar:
				case EControlType::Slider:
				{
					AppendValueParts(tree, node, bounds, visual, styles, time_ms, scale, out);
					break;
				}
				case EControlType::ComboBox:
				{
					AppendComboBoxFace(tree, node, TextRectOf(tree, node, bounds, scale), scale, out);
					break;
				}
				case EControlType::Root:
				case EControlType::Panel:
				case EControlType::Text:
				case EControlType::TextBox:
				case EControlType::Button:
				{
					break;
				}
				default: { throw EngineException(EStatus::UnknownType, "unknown control type"); }
			}

			// A label/text item follows for the three control types that can display text. A
			// focused TextBox's live pending edit (if any) always takes priority over its accepted
			// text, so the draw packet paints exactly what the user is currently typing rather than
			// a stale accepted value, with any in-progress IME composition spliced in.
			if (node.desc.type == EControlType::Text || node.desc.type == EControlType::Button || node.desc.type == EControlType::TextBox)
			{
				auto text = node.text;
				auto ranges = TextEditRanges{ .caret = 0, .selection_start = 0, .selection_end = 0, .composition_start = 0, .composition_length = 0 };
				auto has_edit_state = false;
				auto scroll_dip = 0.0f;
				if (node.desc.type == EControlType::TextBox)
				{
					auto edit_it = input_state.m_text_edits.find(id);
					if (edit_it != input_state.m_text_edits.end() && edit_it->second.initialized != 0)
					{
						text = DisplayTextOf(node.desc, edit_it->second);
						ranges = DisplayRangesOf(node.desc, edit_it->second);
						scroll_dip = edit_it->second.scroll_dip;
						has_edit_state = true;
					}
				}

				// A focused TextBox always emits its item even when empty, because the caret still
				// has to be drawn; every other case with nothing to show is skipped entirely.
				auto const focused = input_state.m_focus_id == id;
				if (!text.empty() || (focused && has_edit_state))
				{
					auto font = ResolveControlFont(tree, node.desc.font_resource_id);
					auto placement = TextPlacementFor(node.desc.type);
					auto const text_part = FindPart(tmpl, ETemplatePart::Text);
					auto const text_bounds = TextRectOf(tree, node, bounds, scale);

					DrawItem text_item{};
					text_item.control_id = node.desc.id;
					text_item.primitive = EVisualPrimitive::TextPresenter;
					text_item.bounds = text_bounds;
					text_item.fill = font.colour;
					text_item.border_colour = visual.border_colour;
					text_item.border_thickness = 0.0f;
					text_item.corner_radius = 0.0f;
					text_item.opacity = visual.opacity;
					text_item.text = std::move(text);
					text_item.font_family = std::move(font.family);
					text_item.font_size = font.size * scale;
					text_item.text_align = placement.align;
					text_item.text_inset_dip = placement.inset_dip * scale;
					text_item.text_scroll_dip = scroll_dip;

					// A TextBox keeps its text inside its border (or inside its Text part), so long text is cut at the
					// inner edge rather than drawn over its neighbours.
					if (node.desc.type == EControlType::TextBox)
					{
						auto const border = text_part != nullptr ? 0.0f : std::clamp(visual.border_thickness * scale, 0.0f, std::min(text_bounds.w, text_bounds.h) * 0.5f);
						text_item.text_clip = Rect{ text_bounds.x + border, text_bounds.y + border, text_bounds.w - 2.0f * border, text_bounds.h - 2.0f * border };
						text_item.text_clipped = 1;
					}

					// Selection/composition/caret decorations are only meaningful for the control
					// the user is actually editing, so an unfocused TextBox reports none of them.
					if (focused && has_edit_state)
					{
						text_item.selection_start = ranges.selection_start;
						text_item.selection_end = ranges.selection_end;
						text_item.composition_start = ranges.composition_start;
						text_item.composition_length = ranges.composition_length;
						text_item.caret_offset = ranges.caret;
						text_item.caret_visible = 1;
					}
					out.items.push_back(std::move(text_item));
				}
			}

			for (auto child_id : node.children)
				Walk(tree, child_id, layout, viewport, styles, input_state, time_ms, scale, out);
		}
	}

	DrawPacket BuildDrawPacket(TreeModel const& tree, std::unordered_map<ControlId, Rect> const& layout, std::unordered_map<ControlId, RootPlacement> const& placements, StyleResolver& styles, InputState const& input_state, std::uint64_t accepted_revision, std::uint64_t visual_sequence, double time_ms, float viewport_dpi, ViewportState const& viewport)
	{
		DrawPacket out;
		out.accepted_revision = accepted_revision;
		out.visual_sequence = visual_sequence;
		out.viewport_dpi = viewport_dpi;
		out.items.reserve(tree.m_controls.size());
		out.groups.reserve(tree.m_roots.size());

		for (auto root_id : tree.m_roots)
		{
			auto placement_it = placements.find(root_id);
			if (placement_it == placements.end())
				throw EngineException(EStatus::InternalError, std::format("BuildDrawPacket: no placement was computed for root {}", root_id));

			// A culled root contributes nothing to draw; it is still reported semantically, so
			// dropping it here is purely a visual decision and never changes accepted state.
			auto const& placement = placement_it->second;
			if (placement.visible == 0)
				continue;

			auto first_item = static_cast<std::uint32_t>(out.items.size());
			Walk(tree, root_id, layout, viewport, styles, input_state, time_ms, placement.scale, out);

			// An entirely-invisible subtree emits no items; skipping the empty group keeps the
			// renderer's per-pass work proportional to what is actually drawn.
			auto item_count = static_cast<std::uint32_t>(out.items.size()) - first_item;
			if (item_count == 0)
				continue;

			out.groups.push_back(DrawGroup{
				.root_id = root_id,
				.policy = placement.policy,
				.first_item = first_item,
				.item_count = item_count,
				.clip_depth = placement.clip_depth,
				.view_depth = placement.view_depth,
				.occlusion_min_opacity = placement.occlusion_min_opacity,
				.occlusion_fade_depth = placement.occlusion_fade_depth,
				.occlusion_depth_bias = placement.occlusion_depth_bias,
			});
		}

		if (input_state.m_open_combo_id != 0)
		{
			// Emit the open popup as the final draw group so it renders above every normal root.
			auto combo_it = tree.m_controls.find(input_state.m_open_combo_id);
			if (combo_it != tree.m_controls.end() && combo_it->second.desc.type == EControlType::ComboBox)
			{
				auto root_id = input_state.m_open_combo_id;
				for (;;)
				{
					auto const& walk = tree.m_controls.at(root_id);
					if (walk.desc.parent_id == 0)
						break;

					root_id = walk.desc.parent_id;
				}

				auto placement_it = placements.find(root_id);
				if (placement_it != placements.end() && placement_it->second.visible != 0)
				{
					auto first_item = static_cast<std::uint32_t>(out.items.size());
					auto const& placement = placement_it->second;
					AppendComboBoxPopup(tree, combo_it->second, layout, input_state, viewport, placement.scale, out);
					auto item_count = static_cast<std::uint32_t>(out.items.size()) - first_item;
					if (item_count != 0)
					{
						out.groups.push_back(DrawGroup{
							.root_id = root_id,
							.policy = placement.policy,
							.first_item = first_item,
							.item_count = item_count,
							.clip_depth = placement.clip_depth,
							.view_depth = placement.view_depth,
							.occlusion_min_opacity = placement.occlusion_min_opacity,
							.occlusion_fade_depth = placement.occlusion_fade_depth,
							.occlusion_depth_bias = placement.occlusion_depth_bias,
						});
					}
				}
			}
		}

		return out;
	}
}
