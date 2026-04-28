//*****************************************
// SVG to LDraw
//	Copyright (c) Rylogic Ltd 2026
//*****************************************
#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include "pr/common/bit_fields.h"
#include "pr/common/cast.h"
#include "pr/common/ldraw.h"
#include "pr/common/min_max_fix.h"
#include "pr/geometry/triangle.h"
#include "pr/math/math.h"
#include "pr/storage/xml.h"

namespace pr::ldraw::svg
{
	struct Options
	{
		// SVG lengths are treated as model units. Non-pixel absolute units are converted using the SVG/CSS 96dpi convention.
		double m_dpi = 96.0;
		double m_curve_tolerance = 0.25;
		double m_arc_max_angle = 10.0;
		double m_depth_step = 1.0e-2;
		int m_circle_facets = 64;
	};

	Builder Read(std::istream& src, Options const& opts = {});
	Builder Read(char const* svg, Options const& opts = {});
	Builder Read(std::string_view svg, Options const& opts = {});
	Builder Read(std::filesystem::path const& filepath, Options const& opts = {});

	namespace impl
	{
		struct Translator
		{	
			using Vec2 = math::Vec2<double>;
			using Vec3 = math::Vec3<double>;
			using Mat2D = math::Mat3x3<double>;

			struct Paint
			{
				bool m_none = false;
				uint32_t m_colour = 0xFF000000;
			};
			struct Style
			{
				Paint m_fill = { false, 0xFF000000 };
				Paint m_stroke = { true, 0xFF000000 };
				double m_opacity = 1.0;
				double m_fill_opacity = 1.0;
				double m_stroke_opacity = 1.0;
				double m_stroke_width = 1.0;
				std::optional<Vec2> m_stroke_dash;
				double m_font_size = 16.0;
				std::string m_font_family;
				bool m_display = true;
				bool m_visible = true;
			};
			struct StyleEdit
			{
				std::optional<Paint> m_fill;
				std::optional<Paint> m_stroke;
				std::optional<double> m_opacity;
				std::optional<double> m_fill_opacity;
				std::optional<double> m_stroke_opacity;
				std::optional<double> m_stroke_width;
				std::optional<std::optional<Vec2>> m_stroke_dash;
				std::optional<double> m_font_size;
				std::optional<std::string> m_font_family;
				std::optional<bool> m_display;
				std::optional<bool> m_visible;
			};
			struct SubPath
			{
				std::vector<Vec2> m_points;
				bool m_closed = false;
			};
			struct State
			{
				Mat2D m_transform = Mat2D::Identity();
				Style m_style;
			};

			Options const& m_opts;
			int m_next_id = 1;
			int m_next_depth = 0;

			Translator(Options const& opts)
				:m_opts(opts)
			{}

			// Create LDraw from SVG
			Builder Build(xml::Node const& root)
			{
				if (!EqualsI(pr::Narrow(root.m_tag), "svg"))
					throw std::runtime_error("Expected an SVG root element");
	
				auto builder = Builder{};
				auto state = State{};
				state.m_transform = RootTransform(root);
				state.m_style = ApplyStyle(root, {});
	
				auto& group = builder.Group(Name(root, "svg", m_next_id));
				ConvertChildren(root, group, state);
				return builder;
			}

			Mat2D TransformAffine(double a, double b, double c, double d, double e, double f)
			{
				return Mat2D(
					Vec3{ a, b, 0.0 },
					Vec3{ c, d, 0.0 },
					Vec3{ e, f, 1.0 });
			}
			Mat2D TransformIdentity()
			{
				return Mat2D::Identity();
			}
			Mat2D TransformTranslation(double x, double y)
			{
				return TransformAffine(1.0, 0.0, 0.0, 1.0, x, y);
			}
			Mat2D TransformScale(double x, double y)
			{
				return TransformAffine(x, 0.0, 0.0, y, 0.0, 0.0);
			}
			Mat2D TransformRotation(double angle)
			{
				auto const s = std::sin(angle);
				auto const c = std::cos(angle);
				return TransformAffine(c, s, -s, c, 0.0, 0.0);
			}
			Vec2 TransformPoint(Mat2D const& m, Vec2 const& pt)
			{
				auto const p = m * Vec3{ pt.x, pt.y, 1.0 };
				return Vec2{ p.x, p.y };
			}
	
			seri::Mat4 ToMat4(Mat2D const& m)
			{
				return
				{
					{ float(m.x.x), float(m.x.y), 0.0f, 0.0f },
					{ float(m.y.x), float(m.y.y), 0.0f, 0.0f },
					{ 0.0f, 0.0f, 1.0f, 0.0f },
					{ float(m.z.x), float(m.z.y), 0.0f, 1.0f },
				};
			}
	
			double DistanceToLine(Vec2 const& pt, Vec2 const& a, Vec2 const& b)
			{
				auto const ab = b - a;
				auto const len = math::Length(ab);
				if (len == 0.0)
					return math::Length(pt - a);
	
				return std::abs(math::Cross(pt - a, ab)) / len;
			}
	
			std::string_view Trim(std::string_view str)
			{
				auto first = size_t{};
				auto last = str.size();
				for (; first != last && std::isspace(static_cast<unsigned char>(str[first])); ++first) {}
				for (; last != first && std::isspace(static_cast<unsigned char>(str[last - 1])); --last) {}
				return str.substr(first, last - first);
			}
			std::string Lower(std::string_view str)
			{
				std::string out(str);
				std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
				return out;
			}
			bool EqualsI(std::string_view lhs, std::string_view rhs)
			{
				return Lower(lhs) == Lower(rhs);
			}
	
			std::optional<std::string> Attr(xml::Node const& node, std::string_view name)
			{
				for (auto const& attr : node.m_attr)
				{
					if (EqualsI(pr::Narrow(attr.m_localname), name))
						return pr::Narrow(attr.m_value);
				}
				return {};
			}
	
			bool HasAttr(xml::Node const& node, std::string_view name)
			{
				return Attr(node, name).has_value();
			}
	
			double ParseDouble(std::string_view value)
			{
				auto str = std::string(Trim(value));
				char* end = nullptr;
				auto const result = std::strtod(str.c_str(), &end);
				if (end == str.c_str())
					throw std::runtime_error("Expected number in SVG value '" + str + "'");
	
				return result;
			}
	
			double ParseLength(std::string_view value, double percent_scale = 1.0, double default_value = 0.0)
			{
				auto str = std::string(Trim(value));
				if (str.empty())
					return default_value;
	
				char* end = nullptr;
				auto const number = std::strtod(str.c_str(), &end);
				if (end == str.c_str())
					return default_value;
	
				auto unit = Lower(Trim(std::string_view(end, str.c_str() + str.size() - end)));
				if (unit.empty() || unit == "px")
					return number;
				if (unit == "%")
					return number * percent_scale / 100.0;
				if (unit == "in")
					return number * m_opts.m_dpi;
				if (unit == "cm")
					return number * m_opts.m_dpi / 2.54;
				if (unit == "mm")
					return number * m_opts.m_dpi / 25.4;
				if (unit == "pt")
					return number * m_opts.m_dpi / 72.0;
				if (unit == "pc")
					return number * m_opts.m_dpi / 6.0;
	
				return number;
			}
	
			void SkipWsComma(std::string const& str, size_t& pos)
			{
				for (; pos != str.size(); ++pos)
				{
					auto const ch = str[pos];
					if (!std::isspace(static_cast<unsigned char>(ch)) && ch != ',')
						break;
				}
			}
	
			bool IsNumberStart(char ch)
			{
				return ch == '+' || ch == '-' || ch == '.' || std::isdigit(static_cast<unsigned char>(ch));
			}
	
			bool HasNumber(std::string const& str, size_t pos)
			{
				SkipWsComma(str, pos);
				return pos != str.size() && IsNumberStart(str[pos]);
			}
	
			double ReadNumber(std::string const& str, size_t& pos)
			{
				SkipWsComma(str, pos);
				if (pos == str.size())
					throw std::runtime_error("Unexpected end of SVG number list");
	
				char* end = nullptr;
				auto const result = std::strtod(str.c_str() + pos, &end);
				if (end == str.c_str() + pos)
					throw std::runtime_error("Expected number in SVG value '" + str + "'");
	
				pos = static_cast<size_t>(end - str.c_str());
				return result;
			}
	
			std::vector<double> ParseNumberList(std::string_view value)
			{
				auto str = std::string(value);
				std::vector<double> out;
				for (auto pos = size_t{}; HasNumber(str, pos); )
					out.push_back(ReadNumber(str, pos));
	
				return out;
			}
	
			std::vector<Vec2> ParsePoints(std::string_view value)
			{
				auto numbers = ParseNumberList(value);
				if (numbers.size() % 2 != 0)
					throw std::runtime_error("SVG point list contains an odd number of values");
	
				std::vector<Vec2> points;
				points.reserve(numbers.size() / 2);
				for (auto i = size_t{}; i != numbers.size(); i += 2)
					points.push_back({ numbers[i + 0], numbers[i + 1] });
	
				return points;
			}
	
			Mat2D ParseTransform(std::string_view value)
			{
				auto str = std::string(value);
				auto out = TransformIdentity();
				for (auto pos = size_t{}; pos != str.size(); )
				{
					for (; pos != str.size() && std::isspace(static_cast<unsigned char>(str[pos])); ++pos) {}
					if (pos == str.size())
						break;
	
					auto name_beg = pos;
					for (; pos != str.size() && (std::isalpha(static_cast<unsigned char>(str[pos])) || str[pos] == '-'); ++pos) {}
					auto name = Lower(std::string_view(str).substr(name_beg, pos - name_beg));
	
					for (; pos != str.size() && std::isspace(static_cast<unsigned char>(str[pos])); ++pos) {}
					if (pos == str.size() || str[pos] != '(')
						throw std::runtime_error("Expected '(' in SVG transform");
	
					auto depth = 1;
					auto args_beg = ++pos;
					for (; pos != str.size() && depth != 0; ++pos)
					{
						switch (str[pos])
						{
							case '(':
							{
								++depth;
								break;
							}
							case ')':
							{
								--depth;
								break;
							}
							default:
							{
								break;
							}
						}
					}
					if (depth != 0)
						throw std::runtime_error("Unclosed SVG transform");
	
					auto args = ParseNumberList(std::string_view(str).substr(args_beg, pos - args_beg - 1));
					auto transform = TransformIdentity();
					if (name == "matrix")
					{
						if (args.size() != 6)
							throw std::runtime_error("SVG matrix transform expects six values");
						transform = TransformAffine(args[0], args[1], args[2], args[3], args[4], args[5]);
					}
					else if (name == "translate")
					{
						if (args.empty() || args.size() > 2)
							throw std::runtime_error("SVG translate transform expects one or two values");
						transform = TransformTranslation(args[0], args.size() == 2 ? args[1] : 0.0);
					}
					else if (name == "scale")
					{
						if (args.empty() || args.size() > 2)
							throw std::runtime_error("SVG scale transform expects one or two values");
						transform = TransformScale(args[0], args.size() == 2 ? args[1] : args[0]);
					}
					else if (name == "rotate")
					{
						if (args.size() != 1 && args.size() != 3)
							throw std::runtime_error("SVG rotate transform expects one or three values");
						transform = TransformRotation(args[0] * constants<double>::tau_by_360);
						if (args.size() == 3)
							transform = TransformTranslation(args[1], args[2]) * transform * TransformTranslation(-args[1], -args[2]);
					}
					else if (name == "skewx")
					{
						if (args.size() != 1)
							throw std::runtime_error("SVG skewX transform expects one value");
						transform = TransformAffine(1.0, 0.0, std::tan(args[0] * constants<double>::tau_by_360), 1.0, 0.0, 0.0);
					}
					else if (name == "skewy")
					{
						if (args.size() != 1)
							throw std::runtime_error("SVG skewY transform expects one value");
						transform = TransformAffine(1.0, std::tan(args[0] * constants<double>::tau_by_360), 0.0, 1.0, 0.0, 0.0);
					}
	
					out = out * transform;
				}
	
				return out;
			}
	
			uint8_t Byte(double value)
			{
				return static_cast<uint8_t>(std::clamp(std::lround(value), 0L, 255L));
			}
	
			uint32_t Colour(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 0xFF)
			{
				return (uint32_t(a) << 24) | (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
			}
	
			std::optional<uint8_t> HexDigit(char ch)
			{
				if (ch >= '0' && ch <= '9') return static_cast<uint8_t>(ch - '0');
				if (ch >= 'a' && ch <= 'f') return static_cast<uint8_t>(10 + ch - 'a');
				if (ch >= 'A' && ch <= 'F') return static_cast<uint8_t>(10 + ch - 'A');
				return {};
			}
	
			uint8_t HexByte(char hi, char lo)
			{
				auto const h = HexDigit(hi);
				auto const l = HexDigit(lo);
				if (!h || !l)
					throw std::runtime_error("Invalid SVG colour");
	
				return static_cast<uint8_t>((*h << 4) | *l);
			}
	
			std::optional<uint32_t> ParseColour(std::string_view value)
			{
				auto str = Trim(value);
				auto lower = Lower(str);
				if (lower.empty() || lower == "none")
					return {};
				if (lower == "transparent")
					return Colour(0, 0, 0, 0);
	
				if (lower.starts_with('#'))
				{
					auto hex = std::string_view(lower).substr(1);
					if (hex.size() == 3 || hex.size() == 4)
					{
						auto const r = HexDigit(hex[0]);
						auto const g = HexDigit(hex[1]);
						auto const b = HexDigit(hex[2]);
						auto const a = hex.size() == 4 ? HexDigit(hex[3]) : std::optional<uint8_t>{ uint8_t{ 0xF } };
						if (!r || !g || !b || !a)
							throw std::runtime_error("Invalid SVG colour");
	
						return Colour(uint8_t((*r << 4) | *r), uint8_t((*g << 4) | *g), uint8_t((*b << 4) | *b), uint8_t((*a << 4) | *a));
					}
					if (hex.size() == 6 || hex.size() == 8)
					{
						auto const r = HexByte(hex[0], hex[1]);
						auto const g = HexByte(hex[2], hex[3]);
						auto const b = HexByte(hex[4], hex[5]);
						auto const a = hex.size() == 8 ? HexByte(hex[6], hex[7]) : uint8_t{ 0xFF };
						return Colour(r, g, b, a);
					}
					throw std::runtime_error("Invalid SVG colour");
				}
	
				if (lower.starts_with("rgb(") || lower.starts_with("rgba("))
				{
					auto args_beg = lower.find('(');
					auto args_end = lower.rfind(')');
					if (args_beg == std::string::npos || args_end == std::string::npos || args_end <= args_beg)
						throw std::runtime_error("Invalid SVG rgb colour");
	
					auto args = std::string_view(lower).substr(args_beg + 1, args_end - args_beg - 1);
					auto parts = std::vector<std::string>{};
					for (auto part_beg = size_t{}; part_beg != args.size(); )
					{
						auto part_end = args.find_first_of(", ", part_beg);
						if (part_end == std::string_view::npos)
							part_end = args.size();
	
						auto part = Trim(args.substr(part_beg, part_end - part_beg));
						if (!part.empty() && part != "/")
							parts.emplace_back(part);
	
						part_beg = part_end;
						for (; part_beg != args.size() && (args[part_beg] == ',' || std::isspace(static_cast<unsigned char>(args[part_beg])) || args[part_beg] == '/'); ++part_beg) {}
					}
					if (parts.size() < 3 || parts.size() > 4)
						throw std::runtime_error("Invalid SVG rgb colour");
	
					auto component = [this](std::string const& part)
					{
						if (part.ends_with('%'))
							return Byte(ParseDouble(std::string_view(part).substr(0, part.size() - 1)) * 255.0 / 100.0);
						return Byte(ParseDouble(part));
					};
					auto alpha = [this](std::string const& part)
					{
						if (part.ends_with('%'))
							return Byte(ParseDouble(std::string_view(part).substr(0, part.size() - 1)) * 255.0 / 100.0);
						return Byte(ParseDouble(part) * 255.0);
					};
	
					return Colour(component(parts[0]), component(parts[1]), component(parts[2]), parts.size() == 4 ? alpha(parts[3]) : uint8_t{ 0xFF });
				}
	
				static constexpr std::array<std::pair<std::string_view, uint32_t>, 17> named_colours =
				{{
					{ "black",   0xFF000000 },
					{ "silver",  0xFFC0C0C0 },
					{ "gray",    0xFF808080 },
					{ "grey",    0xFF808080 },
					{ "white",   0xFFFFFFFF },
					{ "maroon",  0xFF800000 },
					{ "red",     0xFFFF0000 },
					{ "purple",  0xFF800080 },
					{ "fuchsia", 0xFFFF00FF },
					{ "magenta", 0xFFFF00FF },
					{ "green",   0xFF008000 },
					{ "lime",    0xFF00FF00 },
					{ "olive",   0xFF808000 },
					{ "yellow",  0xFFFFFF00 },
					{ "navy",    0xFF000080 },
					{ "blue",    0xFF0000FF },
					{ "orange",  0xFFFFA500 },
				}};
	
				auto const iter = std::find_if(named_colours.begin(), named_colours.end(), [&](auto const& pair) { return pair.first == lower; });
				if (iter != named_colours.end())
					return iter->second;
	
				return {};
			}
	
			std::optional<Paint> ParsePaint(std::string_view value)
			{
				auto str = Trim(value);
				if (EqualsI(str, "none"))
					return Paint{ true, 0xFF000000 };
				if (EqualsI(str, "currentcolor"))
					return {};
	
				auto colour = ParseColour(str);
				if (!colour)
					return {};
	
				return Paint{ false, *colour };
			}
	
			double ParseOpacity(std::string_view value)
			{
				return std::clamp(ParseDouble(value), 0.0, 1.0);
			}
	
			uint32_t ApplyOpacity(uint32_t colour, double opacity)
			{
				auto const a = double((colour >> 24) & 0xFF);
				return (uint32_t(Byte(a * std::clamp(opacity, 0.0, 1.0))) << 24) | (colour & 0x00FFFFFF);
			}
	
			bool Draws(Paint const& paint, double opacity)
			{
				return !paint.m_none && ((ApplyOpacity(paint.m_colour, opacity) >> 24) & 0xFF) != 0;
			}
	
			std::optional<Vec2> ParseDashArray(std::string_view value)
			{
				auto str = Trim(value);
				if (str.empty() || EqualsI(str, "none"))
					return {};
	
				auto values = ParseNumberList(str);
				if (values.empty())
					return {};
	
				auto const dash = values[0];
				auto const gap = values.size() != 1 ? values[1] : values[0];
				if (dash <= 0.0 && gap <= 0.0)
					return {};
	
				return Vec2{ dash, gap };
			}
	
			std::string FirstFontFamily(std::string_view family)
			{
				auto str = Trim(family);
				auto comma = str.find(',');
				if (comma != std::string_view::npos)
					str = Trim(str.substr(0, comma));
	
				if (str.size() >= 2 && ((str.front() == '"' && str.back() == '"') || (str.front() == '\'' && str.back() == '\'')))
					str = str.substr(1, str.size() - 2);
	
				return std::string(str);
			}

			void ApplyDeclaration(StyleEdit& edit, std::string_view name, std::string_view value)
			{
				auto prop = Lower(Trim(name));
				auto val = Trim(value);
				if (prop == "fill")
				{
					if (auto paint = ParsePaint(val); paint)
						edit.m_fill = *paint;
				}
				else if (prop == "stroke")
				{
					if (auto paint = ParsePaint(val); paint)
						edit.m_stroke = *paint;
				}
				else if (prop == "opacity")
				{
					edit.m_opacity = ParseOpacity(val);
				}
				else if (prop == "fill-opacity")
				{
					edit.m_fill_opacity = ParseOpacity(val);
				}
				else if (prop == "stroke-opacity")
				{
					edit.m_stroke_opacity = ParseOpacity(val);
				}
				else if (prop == "stroke-width")
				{
					edit.m_stroke_width = ParseLength(val);
				}
				else if (prop == "stroke-dasharray")
				{
					edit.m_stroke_dash = ParseDashArray(val);
				}
				else if (prop == "font-size")
				{
					edit.m_font_size = ParseLength(val);
				}
				else if (prop == "font-family")
				{
					edit.m_font_family = FirstFontFamily(val);
				}
				else if (prop == "display")
				{
					edit.m_display = !EqualsI(val, "none");
				}
				else if (prop == "visibility")
				{
					edit.m_visible = !EqualsI(val, "hidden") && !EqualsI(val, "collapse");
				}
			}
	
			void ParseInlineStyle(StyleEdit& edit, std::string_view style)
			{
				for (auto decl_beg = size_t{}; decl_beg != style.size(); )
				{
					auto decl_end = style.find(';', decl_beg);
					if (decl_end == std::string_view::npos)
						decl_end = style.size();
	
					auto decl = style.substr(decl_beg, decl_end - decl_beg);
					auto colon = decl.find(':');
					if (colon != std::string_view::npos)
						ApplyDeclaration(edit, decl.substr(0, colon), decl.substr(colon + 1));
	
					decl_beg = decl_end == style.size() ? decl_end : decl_end + 1;
				}
			}
	
			Style ApplyStyle(xml::Node const& node, Style const& parent)
			{
				auto presentation = StyleEdit{};
				static constexpr std::array<std::string_view, 12> presentation_attrs =
				{
					"fill", "stroke", "opacity", "fill-opacity", "stroke-opacity", "stroke-width", "stroke-dasharray", "font-size", "font-family", "display", "visibility", "color",
				};
				for (auto name : presentation_attrs)
				{
					if (auto value = Attr(node, name); value)
						ApplyDeclaration(presentation, name, *value);
				}
	
				auto inline_style = presentation;
				if (auto style = Attr(node, "style"); style)
					ParseInlineStyle(inline_style, *style);
	
				auto out = parent;
				if (inline_style.m_fill)
					out.m_fill = *inline_style.m_fill;
				if (inline_style.m_stroke)
					out.m_stroke = *inline_style.m_stroke;
				if (inline_style.m_opacity)
					out.m_opacity = parent.m_opacity * *inline_style.m_opacity;
				if (inline_style.m_fill_opacity)
					out.m_fill_opacity = *inline_style.m_fill_opacity;
				if (inline_style.m_stroke_opacity)
					out.m_stroke_opacity = *inline_style.m_stroke_opacity;
				if (inline_style.m_stroke_width)
					out.m_stroke_width = *inline_style.m_stroke_width;
				if (inline_style.m_stroke_dash)
					out.m_stroke_dash = *inline_style.m_stroke_dash;
				if (inline_style.m_font_size)
					out.m_font_size = *inline_style.m_font_size;
				if (inline_style.m_font_family)
					out.m_font_family = *inline_style.m_font_family;
				if (inline_style.m_display)
					out.m_display = *inline_style.m_display;
				if (inline_style.m_visible)
					out.m_visible = *inline_style.m_visible;
	
				return out;
			}
	
			double SignedArea(std::vector<Vec2> const& polygon)
			{
				auto area = 0.0;
				for (auto i = size_t{}, j = polygon.size() - 1; i != polygon.size(); j = i++)
					area += polygon[j].x * polygon[i].y - polygon[i].x * polygon[j].y;
	
				return area * 0.5;
			}
	
			bool SamePoint(Vec2 const& lhs, Vec2 const& rhs)
			{
				auto constexpr tol = 1e-9;
				return std::abs(lhs.x - rhs.x) <= tol && std::abs(lhs.y - rhs.y) <= tol;
			}
	
			void RemoveDuplicateClose(std::vector<Vec2>& polygon)
			{
				if (polygon.size() >= 2 && SamePoint(polygon.front(), polygon.back()))
					polygon.pop_back();
			}
	
			void FlattenQuadratic(std::vector<Vec2>& out, Vec2 const& p0, Vec2 const& p1, Vec2 const& p2, double tolerance, int depth = 0)
			{
				if (depth > 16 || DistanceToLine(p1, p0, p2) <= tolerance)
				{
					out.push_back(p2);
					return;
				}
	
				auto const p01 = (p0 + p1) * 0.5;
				auto const p12 = (p1 + p2) * 0.5;
				auto const p012 = (p01 + p12) * 0.5;
				FlattenQuadratic(out, p0, p01, p012, tolerance, depth + 1);
				FlattenQuadratic(out, p012, p12, p2, tolerance, depth + 1);
			}
	
			void FlattenCubic(std::vector<Vec2>& out, Vec2 const& p0, Vec2 const& p1, Vec2 const& p2, Vec2 const& p3, double tolerance, int depth = 0)
			{
				if (depth > 16 || std::max(DistanceToLine(p1, p0, p3), DistanceToLine(p2, p0, p3)) <= tolerance)
				{
					out.push_back(p3);
					return;
				}
	
				auto const p01 = (p0 + p1) * 0.5;
				auto const p12 = (p1 + p2) * 0.5;
				auto const p23 = (p2 + p3) * 0.5;
				auto const p012 = (p01 + p12) * 0.5;
				auto const p123 = (p12 + p23) * 0.5;
				auto const p0123 = (p012 + p123) * 0.5;
				FlattenCubic(out, p0, p01, p012, p0123, tolerance, depth + 1);
				FlattenCubic(out, p0123, p123, p23, p3, tolerance, depth + 1);
			}
	
			double AngleBetween(Vec2 const& lhs, Vec2 const& rhs)
			{
				auto const len = math::Length(lhs) * math::Length(rhs);
				if (len == 0.0)
					return 0.0;
	
				auto const dot = std::clamp(math::Dot(lhs, rhs) / len, -1.0, 1.0);
				auto const cross = math::Cross(lhs, rhs);
				return std::copysign(std::acos(dot), cross);
			}
	
			void FlattenArc(std::vector<Vec2>& out, Vec2 const& p0, double rx, double ry, double x_axis_rotation, bool large_arc, bool sweep, Vec2 const& p1)
			{
				if (rx == 0.0 || ry == 0.0 || SamePoint(p0, p1))
				{
					out.push_back(p1);
					return;
				}
	
				rx = std::abs(rx);
				ry = std::abs(ry);
				auto const phi = x_axis_rotation * constants<double>::tau_by_360;
				auto const cos_phi = std::cos(phi);
				auto const sin_phi = std::sin(phi);
				auto const dx = (p0.x - p1.x) * 0.5;
				auto const dy = (p0.y - p1.y) * 0.5;
				auto const x1p = cos_phi * dx + sin_phi * dy;
				auto const y1p = -sin_phi * dx + cos_phi * dy;
	
				auto const lambda = x1p * x1p / (rx * rx) + y1p * y1p / (ry * ry);
				if (lambda > 1.0)
				{
					auto const scale = std::sqrt(lambda);
					rx *= scale;
					ry *= scale;
				}
	
				auto const rx2 = rx * rx;
				auto const ry2 = ry * ry;
				auto const x1p2 = x1p * x1p;
				auto const y1p2 = y1p * y1p;
				auto const denom = rx2 * y1p2 + ry2 * x1p2;
				auto const numer = std::max(0.0, rx2 * ry2 - rx2 * y1p2 - ry2 * x1p2);
				auto const coef = (large_arc == sweep ? -1.0 : 1.0) * std::sqrt(denom != 0.0 ? numer / denom : 0.0);
				auto const cxp = coef * rx * y1p / ry;
				auto const cyp = coef * -ry * x1p / rx;
				auto const cx = cos_phi * cxp - sin_phi * cyp + (p0.x + p1.x) * 0.5;
				auto const cy = sin_phi * cxp + cos_phi * cyp + (p0.y + p1.y) * 0.5;
				auto const theta1 = AngleBetween({ 1.0, 0.0 }, { (x1p - cxp) / rx, (y1p - cyp) / ry });
				auto delta = AngleBetween({ (x1p - cxp) / rx, (y1p - cyp) / ry }, { (-x1p - cxp) / rx, (-y1p - cyp) / ry });
				if (!sweep && delta > 0.0)
					delta -= constants<double>::tau;
				if (sweep && delta < 0.0)
					delta += constants<double>::tau;
	
				auto const max_angle = std::max(m_opts.m_arc_max_angle * constants<double>::tau_by_360, constants<double>::tau_by_360);
				auto const segments = std::max(1, static_cast<int>(std::ceil(std::abs(delta) / max_angle)));
				for (auto i = 1; i != segments + 1; ++i)
				{
					auto const theta = theta1 + delta * double(i) / double(segments);
					auto const x = cos_phi * rx * std::cos(theta) - sin_phi * ry * std::sin(theta) + cx;
					auto const y = sin_phi * rx * std::cos(theta) + cos_phi * ry * std::sin(theta) + cy;
					out.push_back({ x, y });
				}
			}
	
			bool IsPathCommand(char ch)
			{
				switch (ch)
				{
					case 'M': case 'm':
					case 'L': case 'l':
					case 'H': case 'h':
					case 'V': case 'v':
					case 'C': case 'c':
					case 'S': case 's':
					case 'Q': case 'q':
					case 'T': case 't':
					case 'A': case 'a':
					case 'Z': case 'z':
					{
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
	
			std::vector<SubPath> ParsePath(std::string_view value)
			{
				auto str = std::string(value);
				auto out = std::vector<SubPath>{};
				auto current = Vec2{};
				auto start = Vec2{};
				auto last_cubic_control = Vec2{};
				auto last_quadratic_control = Vec2{};
				auto have_cubic_control = false;
				auto have_quadratic_control = false;
				auto command = char{};
	
				auto add_point = [&](Vec2 pt)
				{
					if (out.empty())
						out.push_back({});
					if (out.back().m_points.empty() || !SamePoint(out.back().m_points.back(), pt))
						out.back().m_points.push_back(pt);
					current = pt;
				};
				auto relative = [&](Vec2 pt, bool rel)
				{
					return rel ? current + pt : pt;
				};
	
				for (auto pos = size_t{}; ; )
				{
					SkipWsComma(str, pos);
					if (pos == str.size())
						break;
	
					if (IsPathCommand(str[pos]))
						command = str[pos++];
					if (command == 0)
						throw std::runtime_error("SVG path data does not start with a command");
	
					auto const rel = std::islower(static_cast<unsigned char>(command)) != 0;
					auto const cmd = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));
					switch (cmd)
					{
						case 'M':
						{
							auto first = true;
							while (HasNumber(str, pos))
							{
								auto const pt = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								if (first)
								{
									out.push_back({});
									start = pt;
									add_point(pt);
									first = false;
								}
								else
								{
									add_point(pt);
								}
							}
							command = rel ? 'l' : 'L';
							have_cubic_control = have_quadratic_control = false;
							break;
						}
						case 'L':
						{
							while (HasNumber(str, pos))
								add_point(relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel));
							have_cubic_control = have_quadratic_control = false;
							break;
						}
						case 'H':
						{
							while (HasNumber(str, pos))
							{
								auto const x = ReadNumber(str, pos);
								add_point({ rel ? current.x + x : x, current.y });
							}
							have_cubic_control = have_quadratic_control = false;
							break;
						}
						case 'V':
						{
							while (HasNumber(str, pos))
							{
								auto const y = ReadNumber(str, pos);
								add_point({ current.x, rel ? current.y + y : y });
							}
							have_cubic_control = have_quadratic_control = false;
							break;
						}
						case 'C':
						{
							while (HasNumber(str, pos))
							{
								auto const p1 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								auto const p2 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								auto const p3 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								FlattenCubic(out.back().m_points, current, p1, p2, p3, m_opts.m_curve_tolerance);
								current = p3;
								last_cubic_control = p2;
								have_cubic_control = true;
								have_quadratic_control = false;
							}
							break;
						}
						case 'S':
						{
							while (HasNumber(str, pos))
							{
								auto const p1 = have_cubic_control ? current * 2.0 - last_cubic_control : current;
								auto const p2 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								auto const p3 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								FlattenCubic(out.back().m_points, current, p1, p2, p3, m_opts.m_curve_tolerance);
								current = p3;
								last_cubic_control = p2;
								have_cubic_control = true;
								have_quadratic_control = false;
							}
							break;
						}
						case 'Q':
						{
							while (HasNumber(str, pos))
							{
								auto const p1 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								auto const p2 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								FlattenQuadratic(out.back().m_points, current, p1, p2, m_opts.m_curve_tolerance);
								current = p2;
								last_quadratic_control = p1;
								have_quadratic_control = true;
								have_cubic_control = false;
							}
							break;
						}
						case 'T':
						{
							while (HasNumber(str, pos))
							{
								auto const p1 = have_quadratic_control ? current * 2.0 - last_quadratic_control : current;
								auto const p2 = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								FlattenQuadratic(out.back().m_points, current, p1, p2, m_opts.m_curve_tolerance);
								current = p2;
								last_quadratic_control = p1;
								have_quadratic_control = true;
								have_cubic_control = false;
							}
							break;
						}
						case 'A':
						{
							while (HasNumber(str, pos))
							{
								auto const rx = ReadNumber(str, pos);
								auto const ry = ReadNumber(str, pos);
								auto const x_axis_rotation = ReadNumber(str, pos);
								auto const large_arc = ReadNumber(str, pos) != 0.0;
								auto const sweep = ReadNumber(str, pos) != 0.0;
								auto const pt = relative({ ReadNumber(str, pos), ReadNumber(str, pos) }, rel);
								FlattenArc(out.back().m_points, current, rx, ry, x_axis_rotation, large_arc, sweep, pt);
								current = pt;
								have_cubic_control = have_quadratic_control = false;
							}
							break;
						}
						case 'Z':
						{
							add_point(start);
							out.back().m_closed = true;
							current = start;
							have_cubic_control = have_quadratic_control = false;
							break;
						}
						default:
						{
							throw std::runtime_error("Unsupported SVG path command");
						}
					}
				}
	
				return out;
			}
	
			std::vector<Vec2> TransformPoints(std::vector<Vec2> points, Mat2D const& transform)
			{
				for (auto& point : points)
					point = TransformPoint(transform, point);
	
				return points;
			}
	
			std::vector<Vec2> RectPoints(double x, double y, double w, double h)
			{
				return
				{
					{ x, y },
					{ x + w, y },
					{ x + w, y + h },
					{ x, y + h },
				};
			}
	
			std::vector<Vec2> RoundedRectPoints(double x, double y, double w, double h, double rx, double ry)
			{
				rx = std::min(std::abs(rx), w * 0.5);
				ry = std::min(std::abs(ry), h * 0.5);
				if (rx == 0.0 || ry == 0.0)
					return RectPoints(x, y, w, h);
	
				auto const corner_count = std::max(3, m_opts.m_circle_facets / 4);
				std::vector<Vec2> points;
				points.reserve(size_t(corner_count * 4));
				auto add_corner = [&](double cx, double cy, double a0, double a1)
				{
					for (auto i = 0; i != corner_count + 1; ++i)
					{
						if (!points.empty() && i == 0)
							continue;
	
						auto const t = double(i) / double(corner_count);
						auto const angle = a0 + (a1 - a0) * t;
						points.push_back({ cx + rx * std::cos(angle), cy + ry * std::sin(angle) });
					}
				};
	
				add_corner(x + w - rx, y + ry, -constants<double>::tau_by_4, 0.0);
				add_corner(x + w - rx, y + h - ry, 0.0, constants<double>::tau_by_4);
				add_corner(x + rx, y + h - ry, constants<double>::tau_by_4, constants<double>::tau_by_2);
				add_corner(x + rx, y + ry, constants<double>::tau_by_2, 3 * constants<double>::tau_by_4);
				return points;
			}
	
			std::vector<Vec2> EllipsePoints(double cx, double cy, double rx, double ry)
			{
				auto const count = std::max(8, m_opts.m_circle_facets);
				std::vector<Vec2> points;
				points.reserve(size_t(count));
				for (auto i = 0; i != count; ++i)
				{
					auto const angle = constants<double>::tau * double(i) / double(count);
					points.push_back({ cx + rx * std::cos(angle), cy + ry * std::sin(angle) });
				}
				return points;
			}
	
			float NextDepth()
			{
				return float(m_opts.m_depth_step * m_next_depth++);
			}
	
			bool AddStroke(LdrBase& parent, std::string const& name, Style const& style, std::vector<Vec2> const& points, bool closed)
			{
				if (!style.m_visible || points.size() < 2 || style.m_stroke_width <= 0.0 || !Draws(style.m_stroke, style.m_opacity * style.m_stroke_opacity))
					return false;
	
				auto colour = ApplyOpacity(style.m_stroke.m_colour, style.m_opacity * style.m_stroke_opacity);
				auto& line = parent.Line(name + "_stroke", colour).style(std::string_view("LineStrip"));
				if (style.m_stroke_width > 0.0)
					line.width(float(style.m_stroke_width));
				if (style.m_stroke_dash)
					line.dashed({ float(style.m_stroke_dash->x), float(style.m_stroke_dash->y) });
	
				auto const z = NextDepth();
				line.strip({ float(points[0].x), float(points[0].y), z });
				for (auto i = size_t{ 1 }; i != points.size(); ++i)
					line.line_to({ float(points[i].x), float(points[i].y), z });
				if (closed && !SamePoint(points.front(), points.back()))
					line.line_to({ float(points.front().x), float(points.front().y), z });
	
				return true;
			}
	
			bool AddFill(LdrBase& parent, std::string const& name, Style const& style, std::vector<Vec2> polygon)
			{
				if (!style.m_visible || polygon.size() < 3 || !Draws(style.m_fill, style.m_opacity * style.m_fill_opacity))
					return false;
	
				RemoveDuplicateClose(polygon);
				if (polygon.size() < 3)
					return false;
	
				if (SignedArea(polygon) < 0.0)
					std::reverse(polygon.begin(), polygon.end());
	
				std::vector<v2> verts;
				verts.reserve(polygon.size());
				for (auto const& point : polygon)
					verts.push_back(v2{ float(point.x), float(point.y) });
	
				auto colour = ApplyOpacity(style.m_fill.m_colour, style.m_opacity * style.m_fill_opacity);
				auto& mesh = parent.Mesh(name + "_fill", colour);
				mesh.generate_normals();
				auto const z = NextDepth();
				for (auto const& point : polygon)
					mesh.vert(float(point.x), float(point.y), z);
	
				auto triangulator = geometry::SeidelTriangulation{};
				triangulator.Triangulate(std::span<v2 const>(verts.data(), verts.size()), [&](int i0, int i1, int i2)
				{
					mesh.face(i0, i1, i2);
				});
	
				return true;
			}
	
			std::string TextContent(xml::Node const& node)
			{
				auto out = pr::Narrow(node.m_value);
				for (auto const& child : node.m_child)
					out += TextContent(child);
	
				return out;
			}
	
			std::string Name(xml::Node const& node, std::string_view fallback, int& counter)
			{
				if (auto id = Attr(node, "id"); id && !Trim(*id).empty())
					return std::string(Trim(*id));
	
				std::ostringstream out;
				out << fallback << "_" << counter++;
				return out.str();
			}
	
			void AddShape(LdrBase& parent, std::string const& name, Style const& style, Mat2D const& transform, std::vector<Vec2> points, bool closed, bool fill)
			{
				points = TransformPoints(std::move(points), transform);
				if (closed && fill)
					AddFill(parent, name, style, points);
				AddStroke(parent, name, style, points, closed);
			}
	
			Mat2D NodeTransform(xml::Node const& node, State const& state)
			{
				auto transform = state.m_transform;
				if (auto attr = Attr(node, "transform"); attr)
					transform = transform * ParseTransform(*attr);
	
				return transform;
			}
	
			void ConvertText(xml::Node const& node, LdrBase& parent, std::string const& name, State const& state)
			{
				auto text = TextContent(node);
				if (Trim(text).empty() || !state.m_style.m_visible || !Draws(state.m_style.m_fill, state.m_style.m_opacity * state.m_style.m_fill_opacity))
					return;
	
				auto const x = ParseLength(Attr(node, "x").value_or("0"));
				auto const y = ParseLength(Attr(node, "y").value_or("0"));
				auto const colour = ApplyOpacity(state.m_style.m_fill.m_colour, state.m_style.m_opacity * state.m_style.m_fill_opacity);
				auto& text_obj = parent.Text(name, colour).text(text).font([&](auto& f)
				{
					if (!state.m_style.m_font_family.empty()) f.name(state.m_style.m_font_family);
					f.size(float(state.m_style.m_font_size)).colour(colour);
				});
	
				auto o2w = ToMat4(state.m_transform * TransformTranslation(x, y));
				o2w.w.z = NextDepth();
				text_obj.o2w(o2w);
			}
	
			void ConvertNode(xml::Node const& node, LdrBase& parent, State const& parent_state)
			{
				auto tag = Lower(pr::Narrow(node.m_tag));
				if (tag.empty())
					return;
	
				auto state = parent_state;
				state.m_style = ApplyStyle(node, parent_state.m_style);
				if (!state.m_style.m_display)
					return;
	
				state.m_transform = NodeTransform(node, state);
				auto name = Name(node, tag, m_next_id);
	
				if (tag == "svg" || tag == "g" || tag == "defs" || tag == "symbol")
				{
					auto& group = parent.Group(name);
					ConvertChildren(node, group, state);
				}
				else if (tag == "rect")
				{
					auto const x = ParseLength(Attr(node, "x").value_or("0"));
					auto const y = ParseLength(Attr(node, "y").value_or("0"));
					auto const w = ParseLength(Attr(node, "width").value_or("0"));
					auto const h = ParseLength(Attr(node, "height").value_or("0"));
					if (w > 0.0 && h > 0.0)
					{
						auto const rx = ParseLength(Attr(node, "rx").value_or("0"));
						auto const ry = ParseLength(Attr(node, "ry").value_or(rx != 0.0 ? std::to_string(rx) : "0"));
						AddShape(parent, name, state.m_style, state.m_transform, RoundedRectPoints(x, y, w, h, rx, ry), true, true);
					}
				}
				else if (tag == "circle")
				{
					auto const cx = ParseLength(Attr(node, "cx").value_or("0"));
					auto const cy = ParseLength(Attr(node, "cy").value_or("0"));
					auto const r = ParseLength(Attr(node, "r").value_or("0"));
					if (r > 0.0)
						AddShape(parent, name, state.m_style, state.m_transform, EllipsePoints(cx, cy, r, r), true, true);
				}
				else if (tag == "ellipse")
				{
					auto const cx = ParseLength(Attr(node, "cx").value_or("0"));
					auto const cy = ParseLength(Attr(node, "cy").value_or("0"));
					auto const rx = ParseLength(Attr(node, "rx").value_or("0"));
					auto const ry = ParseLength(Attr(node, "ry").value_or("0"));
					if (rx > 0.0 && ry > 0.0)
						AddShape(parent, name, state.m_style, state.m_transform, EllipsePoints(cx, cy, rx, ry), true, true);
				}
				else if (tag == "line")
				{
					auto const x1 = ParseLength(Attr(node, "x1").value_or("0"));
					auto const y1 = ParseLength(Attr(node, "y1").value_or("0"));
					auto const x2 = ParseLength(Attr(node, "x2").value_or("0"));
					auto const y2 = ParseLength(Attr(node, "y2").value_or("0"));
					AddShape(parent, name, state.m_style, state.m_transform, { { x1, y1 }, { x2, y2 } }, false, false);
				}
				else if (tag == "polyline")
				{
					if (auto points = Attr(node, "points"); points)
						AddShape(parent, name, state.m_style, state.m_transform, ParsePoints(*points), false, false);
				}
				else if (tag == "polygon")
				{
					if (auto points = Attr(node, "points"); points)
						AddShape(parent, name, state.m_style, state.m_transform, ParsePoints(*points), true, true);
				}
				else if (tag == "path")
				{
					if (auto data = Attr(node, "d"); data)
					{
						auto subpaths = ParsePath(*data);
						for (auto i = size_t{}; i != subpaths.size(); ++i)
						{
							auto subname = subpaths.size() == 1 ? name : name + "_" + std::to_string(i);
							AddShape(parent, subname, state.m_style, state.m_transform, subpaths[i].m_points, subpaths[i].m_closed, subpaths[i].m_closed);
						}
					}
				}
				else if (tag == "text")
				{
					ConvertText(node, parent, name, state);
				}
			}
			void ConvertChildren(xml::Node const& node, LdrBase& parent, State const& state)
			{
				for (auto const& child : node.m_child)
					ConvertNode(child, parent, state);
			}
			Mat2D RootTransform(xml::Node const& root)
			{
				auto min_x = 0.0;
				auto min_y = 0.0;
				auto width = 0.0;
				auto height = 0.0;
	
				if (auto viewbox = Attr(root, "viewBox"); viewbox)
				{
					auto values = ParseNumberList(*viewbox);
					if (values.size() != 4)
						throw std::runtime_error("SVG viewBox expects four values");
	
					min_x = values[0];
					min_y = values[1];
					width = values[2];
					height = values[3];
				}
				else
				{
					width = ParseLength(Attr(root, "width").value_or("0"));
					height = ParseLength(Attr(root, "height").value_or("0"));
				}
	
				if (height > 0.0)
					return TransformAffine(1.0, 0.0, 0.0, -1.0, -min_x, min_y + height);
	
				return TransformAffine(1.0, 0.0, 0.0, -1.0, -min_x, min_y);
			}
		};
	}

	// Read SVG from various sources and convert to LDraw Builder
	inline Builder Read(std::istream& src, Options const& opts)
	{
		auto data = std::string(std::istreambuf_iterator<char>(src), std::istreambuf_iterator<char>());
		return Read(std::string_view(data), opts);
	}
	inline Builder Read(char const* svg, Options const& opts)
	{
		return Read(std::string_view(svg), opts);
	}
	inline Builder Read(std::string_view svg, Options const& opts)
	{
		auto root = xml::Load(svg.data(), svg.size());
		return impl::Translator{ opts }.Build(root);
	}
	inline Builder Read(std::filesystem::path const& filepath, Options const& opts)
	{
		auto file = std::ifstream(filepath, std::ios::binary);
		if (!file)
			throw std::runtime_error("Failed to open SVG file '" + filepath.string() + "'");

		return Read(file, opts);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::ldraw::svg
{
	PRUnitTest(SvgToLDrawBasicShapes)
	{
		auto const svg =
			"<svg viewBox='0 0 10 10'>"
			"  <g id='layer1' fill='#ff0000' stroke='blue'>"
			"    <rect id='box' x='1' y='2' width='3' height='4'/>"
			"  </g>"
			"</svg>";

		auto builder = Read(svg);
		auto ldr = builder.ToString();
		PR_EXPECT(ldr.find("*Group layer1") != std::string::npos);
		PR_EXPECT(ldr.find("*Mesh box_fill ffff0000") != std::string::npos);
		PR_EXPECT(ldr.find("*Line box_stroke ff0000ff") != std::string::npos);
		PR_EXPECT(ldr.find("*GenerateNormals {0}") != std::string::npos);
		PR_EXPECT(ldr.find("*Mesh box_fill") < ldr.find("*Line box_stroke"));
		PR_EXPECT(ldr.find("0.01", ldr.find("*Line box_stroke")) != std::string::npos);
	}

	PRUnitTest(SvgToLDrawText)
	{
		auto const svg =
			"<svg viewBox='0 0 100 20'>"
			"  <text id='label' x='10' y='12' fill='white' font-family='Arial' font-size='14'>Hello</text>"
			"</svg>";

		auto builder = Read(svg);
		auto ldr = builder.ToString();
		PR_EXPECT(ldr.find("*Text label ffffffff") != std::string::npos);
		PR_EXPECT(ldr.find("*Data {\"Hello\"}") != std::string::npos);
		PR_EXPECT(ldr.find("*Font {") != std::string::npos);
		PR_EXPECT(ldr.find("*Name {\"Arial\"}") != std::string::npos);
		PR_EXPECT(ldr.find("*Size {14}") != std::string::npos);
	}
}
#endif
