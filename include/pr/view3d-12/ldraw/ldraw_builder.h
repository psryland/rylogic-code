//************************************
// LDraw Builder
//  Copyright (c) Rylogic Ltd 2006
//************************************
// Notes:
//  - You can write ldraw data to a stream using:
//      stream << builder.ToText() << std::flush; or
//      stream << builder.ToBinary() << std::flush;
//  - Use a socket_stream for streaming to LDraw:
//      #include "pr/network/winsock.h"
//      #include "pr/network/socket_stream.h"
//      pr::network::Winsock winsock;
//      pr::network::socket_stream ldr("localhost", 1976);
//      ldr << builder.ToText(false) << std::flush;
#pragma once
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"
#include "pr/view3d-12/ldraw/ldraw_writer_text.h"
#include "pr/view3d-12/ldraw/ldraw_writer_binary.h"

namespace pr::rdr12::ldraw
{
	enum class ESaveFlags : uint32_t
	{
		None = 0,
		Binary = 1 << 0,
		Pretty = 1 << 1,
		Append = 1 << 2,
		NoThrowOnFailure = 1 << 8,
		_flags_enum = 0,
	};

	// Write the contents of 'ldr' to a file
	inline void Write(std::string_view ldr, std::filesystem::path const& filepath, bool append = false)
	{
		if (ldr.empty()) return;
		std::filesystem::create_directories(filepath.parent_path());
		std::ofstream file(filepath, append ? std::ios::app : std::ios::out);
		file << ldr;
		file.close();
	}
	inline void Write(std::wstring_view ldr, std::filesystem::path const& filepath, bool append = false)
	{
		if (ldr.empty()) return;
		std::filesystem::create_directories(filepath.parent_path());
		std::wofstream file(filepath, append ? std::ios::app : std::ios::out);
		file << ldr;
		file.close();
	}
	inline void Write(std::span<std::byte const> ldr, std::filesystem::path const& filepath, bool append = false)
	{
		if (ldr.empty()) return;
		std::filesystem::create_directories(filepath.parent_path());
		std::ofstream file(filepath, std::ios::binary | (append ? std::ios::app : std::ios::out));
		file.write(reinterpret_cast<char const*>(ldr.data()), static_cast<std::streamsize>(ldr.size()));
		file.close();
	}

	// Pretty format Ldraw script
	template <typename TStr> requires(requires(TStr t)
	{
		t.reserve(0);
		t.push_back('c');
		{ t.append(0, 'c') } -> std::convertible_to<TStr&>;
	})
	TStr FormatScript(TStr const& str)
	{
		TStr out = {};
		out.reserve(str.size());

		int indent = 0;
		for (auto c : str)
		{
			if (c == '{')
			{
				++indent;
				out.push_back(c);
				out.append(1,'\n').append(indent, '\t');
			}
			else if (c == '}')
			{
				--indent;
				out.append(1,'\n').append(indent, '\t');
				out.push_back(c);
			}
			else
			{
				if (!out.empty() && out.back() == '}')
					out.append(1,'\n').append(indent, '\t');

				out.push_back(c);
			}
		}
		return out;
	}

	// Ldr object fluent helper
	namespace fluent
	{
		template <typename T> concept WriterType = requires { true; }; // todo
		static_assert(WriterType<BinaryWriter>);
		static_assert(WriterType<TextWriter>);

		struct LdrPoint;
		struct LdrLine;
		struct LdrCoordFrame;
		struct LdrTriangle;
		struct LdrPlane;
		struct LdrCircle;
		struct LdrSphere;
		struct LdrBox;
		struct LdrCylinder;
		struct LdrCone;
		struct LdrFrustum;
		struct LdrModel;
		struct LdrInstance;
		struct LdrGroup;
		struct LdrCommands;
		struct LdrBinaryStream;
		struct LdrTextStream;

		struct LdrBuilder
		{
			using ObjPtr = std::unique_ptr<LdrBuilder>;
			using ObjCont = std::vector<ObjPtr>;

			ObjCont m_objects;

			LdrBuilder() = default;
			LdrBuilder(LdrBuilder&&) = default;
			LdrBuilder(LdrBuilder const&) = delete;
			LdrBuilder& operator=(LdrBuilder&&) = default;
			LdrBuilder& operator=(LdrBuilder const&) = delete;
			virtual ~LdrBuilder() = default;

			// Reset the builder
			LdrBuilder& Clear(int count = -1)
			{
				if (count >= 0 && count < ssize(m_objects))
					m_objects.resize(m_objects.size() - count);
				else
					m_objects.clear();

				return *this;
			}

			// Object types
			LdrPoint& Point(Name name = {}, Colour colour = Colour());
			LdrLine& Line(Name name = {}, Colour colour = Colour());
			LdrCoordFrame& CoordFrame(Name name = {}, Colour colour = Colour());
			LdrTriangle& Triangle(Name name = {}, Colour colour = Colour());
			LdrPlane& Plane(Name name = {}, Colour colour = Colour());
			LdrCircle& Circle(Name name = {}, Colour colour = Colour());
			LdrSphere& Sphere(Name name = {}, Colour colour = Colour());
			LdrBox& Box(Name name = {}, Colour colour = Colour());
			LdrCylinder& Cylinder(Name name = {}, Colour colour = Colour());
			LdrCone& Cone(Name name = {}, Colour colour = Colour());
			LdrFrustum& Frustum(Name name = {}, Colour colour = Colour());
			LdrModel& Model(Name name = {}, Colour colour = Colour());
			LdrInstance& Instance(Name name = {}, Colour colour = Colour());
			LdrGroup& Group(Name name = {}, Colour colour = Colour());
			LdrCommands& Command(Name name = {}, Colour colour = Colour());

			// Switch data stream modes
			LdrBuilder& BinaryStream();
			LdrBuilder& TextStream();

			// Extension objects. Use: `builder._<LdrCustom>("name", 0xFFFFFFFF)`
			template <typename LdrCustom> requires std::is_base_of_v<LdrBuilder, LdrCustom>
			LdrCustom& _(std::string_view name = "", Colour colour = Colour())
			{
				auto ptr = new LdrCustom;
				m_objects.emplace_back(ptr);
				return (*ptr).name(name).colour(colour);
			}

			// Wrap all objects into a group
			LdrBuilder& WrapAsGroup(Name name = {}, Colour colour = Colour());

			// Serialise the ldr script to a string
			textbuf ToText(bool pretty) const
			{
				textbuf out;
				Write(out);
				if (pretty) out = FormatScript(out);
				return out;
			}

			// Serialise the ldr script to binary
			bytebuf ToBinary() const
			{
				bytebuf out;
				Write(out);
				return out;
			}

			// Serialise to 'out'
			virtual void Write(textbuf& out) const
			{
				for (auto& obj : m_objects)
					obj->Write(out);
			}
			virtual void Write(bytebuf& out) const
			{
				for (auto& obj : m_objects)
					obj->Write(out);
			}

			// Write the script to a file
			LdrBuilder& Save(std::filesystem::path const& filepath, ESaveFlags flags = ESaveFlags::None)
			{
				try
				{
					std::default_random_engine rng;
					std::uniform_int_distribution<uint64_t> dist(0);
					auto tmp_path = filepath.parent_path() / std::format("{}.tmp", dist(rng));

					auto binary = AllSet(flags, ESaveFlags::Binary);
					auto append = AllSet(flags, ESaveFlags::Append);
					auto pretty = AllSet(flags, ESaveFlags::Pretty);

					if (!std::filesystem::exists(tmp_path.parent_path()))
						std::filesystem::create_directories(tmp_path.parent_path());

					if (binary)
					{
						bytebuf out;
						Write(out);
						ldraw::Write(out, tmp_path, append);
					}
					else
					{
						textbuf out;
						Write(out);
						if (pretty) out = FormatScript(out);
						ldraw::Write(out, tmp_path, append);
					}

					auto outpath = filepath;
					if (outpath.has_extension() == false)
						outpath.replace_extension(binary ? ".bdr" : ".ldr");

					// Replace/rename
					std::filesystem::rename(tmp_path, outpath);
					// 'std::filesystem::rename' might not replace existing files on windows...
					//MoveFileExW(fpath.c_str(), outpath.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
				}
				catch (std::exception const& ex)
				{
					if (!AllSet(flags, ESaveFlags::NoThrowOnFailure)) throw;
					OutputDebugStringA(std::format("LDraw save failed: {}", ex.what()).c_str());
				}

				return *this;
			}
		};

		template <typename Derived>
		struct LdrBase :LdrBuilder
		{
			LdrBase()
				: m_name()
				, m_colour()
				, m_o2w()
				, m_wire()
				, m_axis_id()
				, m_solid()
			{}
			Derived* me()
			{
				return reinterpret_cast<Derived*>(this);
			}

			// Object name
			Derived& name(Name n)
			{
				m_name = n;
				return *me();
			}
			Name m_name;

			// Object colour
			Derived& colour(Colour c)
			{
				m_colour = c;
				m_colour.m_kw = EKeyword::Colour;
				return *me();
			}
			Colour m_colour;

			// Object colour mask
			Derived& group_colour(Colour c)
			{
				m_group_colour = c;
				m_group_colour.m_kw = EKeyword::GroupColour;
				return *me();
			}
			Colour m_group_colour;

			// Object to world transform
			Derived& o2w(m4x4 const& o2w)
			{
				m_o2w.m_mat = o2w * m_o2w.m_mat;
				return *me();
			}
			Derived& o2w(m3x4 const& rot, v4 pos)
			{
				m_o2w.m_mat = m4x4{ rot, pos } * m_o2w.m_mat;
				return *me();
			}
			Derived& ori(v4 dir, pr::AxisId axis = pr::AxisId::PosZ)
			{
				return ori(m3x4::Rotation(axis.vec(), dir));
			}
			Derived& ori(m3x4 const& rot)
			{
				return o2w(rot, v4::Origin());
			}
			Derived& ori(quat const& q)
			{
				return o2w(m4x4::Transform(q, v4::Origin()));
			}
			Derived& pos(float x, float y, float z)
			{
				return o2w(m4x4::Translation(x, y, z));
			}
			Derived& pos(v4 pos)
			{
				return o2w(m4x4::Translation(pos));
			}
			Derived& pos(v3 pos)
			{
				return this->pos(pos.w1());
			}
			Derived& scale(float s)
			{
				return scale(s, s, s);
			}
			Derived& scale(float sx, float sy, float sz)
			{
				return ori(m3x4::Scale(sx, sy, sz));
			}
			Derived& scale(v4 scale)
			{
				return ori(m3x4::Scale(scale.x, scale.y, scale.z));
			}
			Derived& euler(float pitch_deg, float yaw_deg, float roll_deg)
			{
				return ori(m3x4::Rotation(
					DegreesToRadians(pitch_deg),
					DegreesToRadians(yaw_deg),
					DegreesToRadians(roll_deg)));
			}
			O2W m_o2w;

			// Hidden
			Derived& hide(bool hidden = true)
			{
				m_hide.m_hide = hidden;
				return *me();
			}
			Hidden m_hide;

			// Wire frame
			Derived& wireframe(bool w = true)
			{
				m_wire.m_wire = w;
				return *me();
			}
			Wireframe m_wire;

			// Axis id
			Derived& axis(pr::AxisId axis_id)
			{
				m_axis_id = axis_id;
				return *me();
			}
			AxisId m_axis_id;

			// Solid
			Derived& solid(bool s = true)
			{
				m_solid.m_solid = s;
				return *me();
			}
			Solid m_solid;

			// Copy all modifiers from another object
			template <typename D> Derived& modifiers(LdrBase<D> const& rhs)
			{
				m_name = rhs.m_name;
				m_colour = rhs.m_colour;
				m_o2w = rhs.m_o2w;
				m_wire = rhs.m_wire;
				m_axis_id = rhs.m_axis_id;
				return *me();
			}

			// Write nested objects to 'out'
			virtual void Write(textbuf& out) const override
			{
				auto const& derived = *reinterpret_cast<Derived const*>(this);
				derived.Derived::WriteTo<TextWriter>(out);
			}
			virtual void Write(bytebuf& out) const override 
			{
				auto const& derived = *reinterpret_cast<Derived const*>(this);
				derived.Derived::WriteTo<BinaryWriter>(out);
			}

			// Write ldraw script to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Append(out, m_axis_id, m_wire, m_solid, m_group_colour, m_o2w);
				LdrBuilder::Write(out);
			}
		};

		// Modifiers
		struct LdrTexture
		{
			std::filesystem::path m_filepath = {};
			EAddrMode m_addr[2] = {EAddrMode::Wrap, EAddrMode::Wrap};
			EFilter m_filter = EFilter::Linear;
			Alpha m_has_alpha = {};
			O2W m_t2s = {};

			// Texture filepath
			LdrTexture& path(std::filesystem::path filepath)
			{
				m_filepath = filepath;
				return *this;
			}

			// Addressing mode
			LdrTexture& addr(EAddrMode addrU, EAddrMode addrV)
			{
				m_addr[0] = addrU;
				m_addr[1] = addrV;
				return *this;
			}

			// Filtering mode
			LdrTexture& filter(EFilter filter)
			{
				m_filter = filter;
				return *this;
			}

			// Texture to surface transform
			LdrTexture& t2s(O2W t2s)
			{
				m_t2s = t2s;
				return *this;
			}

			// Has alpha flag
			LdrTexture& alpha(Alpha has_alpha)
			{
				m_has_alpha = has_alpha;
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				if (m_filepath.empty()) return;
				Writer::Write(out, EKeyword::Texture, [&]
				{
					Writer::Write(out, EKeyword::FilePath, "\"", m_filepath.string(), "\"");
					Writer::Write(out, EKeyword::Addr, m_addr[0], m_addr[1]);
					Writer::Write(out, EKeyword::Filter, m_filter);
					Writer::Append(out, m_has_alpha);
					Writer::Append(out, m_t2s);
				});
			}
		};
		struct LdrAnimation
		{
			std::optional<pr::Range<int>> m_frame_range = {};
			bool m_no_translation = false;
			bool m_no_rotation = false;

			// Limit frame range
			LdrAnimation& frames(int beg, int end)
			{
				m_frame_range = pr::Range<int>(beg, end);
				return *this;
			}
			LdrAnimation& frame(int frame)
			{
				return frames(frame, frame + 1);
			}

			// Anim flags
			LdrAnimation& no_translation(bool on = true)
			{
				m_no_translation = on;
				return *this;
			}
			LdrAnimation& no_rotation(bool on = true)
			{
				m_no_rotation = on;
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Animation, [&]
				{
					if (m_frame_range)
					{
						auto& range = *m_frame_range;
						if (range.size() == 1)
							Writer::Write(out, EKeyword::Frame, range.begin());
						else
							Writer::Write(out, EKeyword::FrameRange, range.begin(), range.end());
					}
					if (m_no_translation)
					{
						Writer::Write(out, EKeyword::NoRootTranslation);
					}
					if (m_no_rotation)
					{
						Writer::Write(out, EKeyword::NoRootRotation);
					}
				});
			}
		};

		// Object types
		struct LdrPoint :LdrBase<LdrPoint>
		{
			struct Point { v4 pt; Colour32 col; };
			std::vector<Point> m_points;
			Size2 m_size;
			Depth m_depth;
			PointStyle m_style;
			PerItemColour m_per_item_colour;
			LdrTexture m_tex;

			LdrPoint()
				: m_points()
				, m_size()
				, m_style()
				, m_per_item_colour()
			{}

			// Points
			LdrPoint& pt(v4 point, std::optional<Colour32> colour = {})
			{
				m_points.push_back({ point, colour ? *colour : Colour32White });
				if (colour) m_per_item_colour = true;
				return *this;
			}
			LdrPoint& pt(v3 point, std::optional<Colour32> colour = {})
			{
				return pt(point.w1(), colour);
			}

			// Point size (in pixels if depth == false, in world space if depth == true)
			LdrPoint& size(float s)
			{
				m_size = v2(s);
				return *this;
			}
			LdrPoint& size(v2 s)
			{
				m_size = s;
				return *this;
			}

			// Points have depth
			LdrPoint& depth(bool d = true)
			{
				m_depth = d;
				return *this;
			}

			// Point style
			LdrPoint& style(EPointStyle s)
			{
				m_style = s;
				return *this;
			}

			// Texture for point sprites
			LdrTexture& texture()
			{
				return m_tex;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Point, m_name, m_colour, [&]
				{
					Writer::Append(out, m_style, m_size, m_depth, m_per_item_colour);
					Writer::Write(out, EKeyword::Data, [&]
					{
						for (auto& point : m_points)
						{
							Writer::Append(out, point.pt.xyz);
							if (m_per_item_colour.m_per_item_colour)
								Writer::Append(out, point.col);
						}
					});
					m_tex.WriteTo<Writer>(out);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrLine :LdrBase<LdrLine>
		{
			struct Ln { v4 a, b; Colour32 col; };
			struct Pt { v4 a; Colour32 col; };
			struct Block
			{
				vector<Ln> m_lines;
				vector<Pt> m_strip;
				LineStyle m_style;
				Smooth m_smooth;
				Width m_width;
				Dashed m_dashed;
				ArrowHeads m_arrow;
				DataPoints m_data_points;
				PerItemColour m_per_item_colour;
				explicit operator bool() const { return !m_lines.empty() || !m_strip.empty(); }
			};

			std::vector<Block> m_blocks;
			Block m_current;

			LdrLine()
				: m_blocks()
				, m_current()
			{}

			LdrLine& style(ELineStyle sty)
			{
				m_current.m_style = sty;
				return *this;
			}
			LdrLine& per_item_colour(bool on = true)
			{
				m_current.m_per_item_colour = on;
				return *this;
			}
			LdrLine& smooth(bool smooth = true)
			{
				m_current.m_smooth = smooth;
				return *this;
			}
			LdrLine& width(Width w)
			{
				m_current.m_width = w;
				return *this;
			}
			LdrLine& dashed(Dashed dash)
			{
				m_current.m_dashed = dash;
				return *this;
			}
			LdrLine& arrow(EArrowType style = EArrowType::Fwd, float size = 10.0f)
			{
				m_current.m_arrow = ArrowHeads(style, size);
				return *this;
			}

			LdrLine& line(v4 a, v4 b, std::optional<Colour32> colour = {})
			{
				// Don't overwrite style here, it could be direction or segments
				m_current.m_lines.push_back({ a, b, colour ? *colour : Colour32White });
				if (colour) m_current.m_per_item_colour = true;
				m_current.m_strip.clear();
				return *this;
			}
			LdrLine& line(v3 a, v3 b, std::optional<Colour32> colour = {})
			{
				return line(a.w1(), b.w1(), colour);
			}
			LdrLine& lines(std::span<v4 const> verts, std::span<int const> indices)
			{
				assert((isize(indices) % 2) == 0);
				for (int const* i = indices.data(), *iend = i + indices.size(); i < iend; i += 2)
					line(verts[*(i+0)], verts[*(i+1)]);

				return *this;
			}

			LdrLine& strip(v4 start, std::optional<Colour32> colour = {})
			{
				m_current.m_strip.push_back({ start, colour ? *colour : Colour32White });
				if (colour) m_current.m_per_item_colour = true;
				m_current.m_lines.clear();
				return *this;
			}
			LdrLine& strip(v3 start, std::optional<Colour32> colour = {})
			{
				return strip(start.w1(), colour);
			}
			LdrLine& line_to(v4 pt, std::optional<Colour32> colour = {})
			{
				if (m_current.m_strip.empty()) strip(pt, colour);
				return strip(pt, colour);
			}
			LdrLine& line_to(v3 pt, std::optional<Colour32> colour = {})
			{
				return strip(pt.w1(), colour);
			}

			LdrLine& new_block()
			{
				m_blocks.push_back(std::move(m_current));
				m_current.m_lines.clear();
				m_current.m_strip.clear();
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Line, m_name, m_colour, [&]
				{
					auto WriteBlock = [](TOut& out, Block const& block)
					{
						Writer::Append(out, block.m_style, block.m_smooth, block.m_width, block.m_dashed, block.m_arrow, block.m_data_points, block.m_per_item_colour);
						Writer::Write(out, EKeyword::Data, [&]
						{
							for (auto& ln : block.m_lines)
							{
								Writer::Append(out, ln.a.xyz, ln.b.xyz);
								if (block.m_per_item_colour)
									Writer::Append(out, ln.col);
							}
							for (auto& pt : block.m_strip)
							{
								Writer::Append(out, pt.a.xyz);
								if (block.m_per_item_colour)
									Writer::Append(out, pt.col);
							}
						});
					};

					for (auto& block : m_blocks)
						WriteBlock(out, block);
					if (m_current)
						WriteBlock(out, m_current);

					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrCoordFrame :LdrBase<LdrCoordFrame>
		{
			Scale m_scale = {};
			LeftHanded m_lh = {};

			// Scale size
			LdrCoordFrame& scale(float s)
			{
				m_scale = s;
				return *this;
			}

			// Left handed axis
			LdrCoordFrame& left_handed(bool lh = true)
			{
				m_lh = lh;
				return *this;
			}

			// Write to 'outp
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::CoordFrame, m_name, m_colour, [&]
				{
					Writer::Append(out, m_scale, m_lh);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrTriangle :LdrBase<LdrTriangle>
		{
			struct Tri { v4 a, b, c; Colour32 col; };
			std::vector<Tri> m_tris;
			PerItemColour m_per_item_colour;

			LdrTriangle()
				: m_tris()
				, m_per_item_colour()
			{}

			LdrTriangle& tri(v4 a, v4 b, v4 c, Colour32 colour)
			{
				tri(a, b, c);
				m_tris.back().col = colour;
				m_per_item_colour = true;
				return *this;
			}
			LdrTriangle& tri(v4 a, v4 b, v4 c)
			{
				m_tris.push_back({ a, b, c });
				return *this;
			}
			LdrTriangle& tris(v4 const* verts, std::span<int const> faces)
			{
				assert((isize(faces) % 3) == 0);
				for (int const* i = faces.data(), *iend = i + faces.size(); i < iend; i += 3)
					tri(verts[*(i+0)], verts[*(i+1)], verts[*(i+2)]);

				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Triangle, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, [&]
					{
						for (auto& tri : m_tris)
						{
							Writer::Append(out, tri.a.xyz, tri.b.xyz, tri.c.xyz);
							if (m_per_item_colour.m_per_item_colour)
								Writer::Append(out, tri.col);
						}
					});
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrPlane :LdrBase<LdrPlane>
		{
			v2 m_wh;
			LdrTexture m_tex;

			LdrPlane()
				: m_wh(1,1)
				, m_tex()
			{}

			LdrPlane& plane(v4 p)
			{
				pos((p.xyz * -p.w).w1());
				ori(Normalise(p.xyz.w0()), pr::AxisId::PosZ);
				return *this;
			}
			LdrPlane& wh(float width, float height)
			{
				m_wh = { width, height };
				return *this;
			}
			LdrPlane& wh(v2 wh)
			{
				m_wh = wh;
				return *this;
			}

			LdrTexture& texture()
			{
				return m_tex;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Plane, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_wh);
					m_tex.WriteTo<Writer>(out);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrCircle :LdrBase<LdrCircle>
		{
			float m_radius;

			LdrCircle()
				:m_radius(1.0f)
			{}

			LdrCircle& radius(float r)
			{
				m_radius = r;
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Circle, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_radius);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrSphere :LdrBase<LdrSphere>
		{
			v4 m_radius;

			LdrSphere()
				:m_radius()
			{}

			// Radius
			LdrSphere& radius(float r)
			{
				return radius({ r, r, r, 0 });
			}
			LdrSphere& radius(v4 r)
			{
				m_radius = r;
				return *this;
			}

			// Create from bounding sphere
			LdrSphere& bsphere(BSphere bsphere)
			{
				if (bsphere == BSphere::Reset()) return *this;
				return radius(bsphere.Radius()).pos(bsphere.Centre());
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Sphere, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_radius.xyz);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrBox :LdrBase<LdrBox>
		{
			v4 m_dim;

			LdrBox()
				:m_dim()
			{}

			// Box dimensions
			LdrBox& radii(float radii)
			{
				return dim(radii * 2);
			}
			LdrBox& radii(v4 radii)
			{
				return dim(radii * 2);
			}
			LdrBox& dim(float dim)
			{
				m_dim = v4{dim, dim, dim, 0};
				return *this;
			}
			LdrBox& dim(v4 dim)
			{
				m_dim = v4(dim.x, dim.y, dim.z, 0);
				return *this;
			}
			LdrBox& dim(float sx, float sy, float sz)
			{
				m_dim = v4(sx, sy, sz, 0);
				return *this;
			}

			// Create from bounding box
			LdrBox& bbox(BBox bbox)
			{
				if (bbox == BBox::Reset()) return *this;
				return dim(2 * bbox.Radius()).pos(bbox.Centre());
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Box, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_dim.xyz);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrCylinder :LdrBase<LdrCylinder>
		{
			v2 m_radius; // x = base, y = tip
			Scale2 m_scale;
			float m_height;

			LdrCylinder()
				: m_radius(0.5f)
				, m_scale()
				, m_height(1)
			{}

			// Height/Radius
			LdrCylinder& cylinder(float height, float radius)
			{
				return cylinder(height, radius, radius);
			}
			LdrCylinder& cylinder(float height, float radius_base, float radius_tip)
			{
				m_height = height;
				m_radius = v2{ radius_base, radius_tip };
				return *this;
			}

			// Scale
			LdrCylinder& scale(Scale2 scale)
			{
				m_scale = scale;
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Cylinder, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_height, m_radius.x, m_radius.y);
					Writer::Append(out, m_scale);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrCone :LdrBase<LdrCone>
		{
			v2 m_distance; // x = tip-to-top face, y = tip-to-base
			Scale2 m_scale;
			float m_angle;

			LdrCone()
				: m_distance(0, 1)
				, m_scale()
				, m_angle(45.0f)
			{}

			// Height/Radius
			LdrCone& angle(float solid_angle_deg)
			{
				m_angle = solid_angle_deg;
				return *this;
			}
			LdrCone& height(float height)
			{
				m_distance = v2{ m_distance.x, m_distance.x + height };
				return *this;
			}
			LdrCone& dist(float dist0, float dist1)
			{
				m_distance = v2{ dist0, dist1 };
				return *this;
			}

			// Scale
			LdrCone& scale(Scale2 scale)
			{
				m_scale = scale;
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Cone, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_angle, m_distance.x, m_distance.y);
					Writer::Append(out, m_scale);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrFrustum :LdrBase<LdrFrustum>
		{
			v2 m_wh;
			v2 m_nf;
			float m_fovY;
			float m_aspect;
			bool m_ortho;

			LdrFrustum()
				: m_wh()
				, m_nf()
				, m_fovY()
				, m_aspect()
				, m_ortho()
			{}

			// Orthographic
			LdrFrustum& ortho(bool ortho = true)
			{
				m_ortho = ortho;
				return *this;
			}

			// Near/Far
			LdrFrustum& nf(float n, float f)
			{
				m_nf = v2{ n, f };
				return *this;
			}
			LdrFrustum& nf(v2 nf_)
			{
				return nf(nf_.x, nf_.y);
			}

			// Frustum dimensions
			LdrFrustum& wh(float w, float h)
			{
				m_wh = v2{ w, h };
				m_fovY = 0;
				m_aspect = 0;
				return *this;
			}
			LdrFrustum& wh(v2 sz)
			{
				return wh(sz.x, sz.y);
			}

			// Frustum angles
			LdrFrustum& fov(float fovY, float aspect)
			{
				m_ortho = false;
				m_wh = v2::Zero();
				m_fovY = fovY;
				m_aspect = aspect;
				return *this;
			}

			// From maths frustum
			LdrFrustum& frustum(pr::Frustum const& f)
			{
				return nf(0, f.zfar()).fov(f.fovY(), f.aspect());
			}

			// From projection matrix
			LdrFrustum& proj(m4x4 const& c2s)
			{
				if (c2s.w.w == 1) // If orthographic
				{
					auto rh = -Sign(c2s.z.z);
					auto zn = Div(c2s.w.z, c2s.z.z, 0.0f);
					auto zf = Div(zn * (c2s.w.z - rh), c2s.w.z, 1.0f);
					auto w = 2.0f / c2s.x.x;
					auto h = 2.0f / c2s.y.y;
					return ortho(true).nf(zn, zf).wh(w,h);
				}
				else // Otherwise perspective
				{
					auto rh = -Sign(c2s.z.w);
					auto zn = rh * c2s.w.z / c2s.z.z;
					auto zf = Div(zn * c2s.z.z, (rh + c2s.z.z), zn * 1000.0f);
					auto w = 2.0f * zn / c2s.x.x;
					auto h = 2.0f * zn / c2s.y.y;
					return ortho(false).nf(zn, zf).wh(w, h);
				}
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				if (m_ortho)
				{
					Writer::Write(out, EKeyword::Box, m_name, m_colour, [&]
					{
						Writer::Write(out, EKeyword::Data, m_wh.x, m_wh.y, m_nf.y - m_nf.x);
						Writer::Append(out, O2W{ v4{ 0, 0, -0.5f * s_cast<float>(m_nf.x + m_nf.y), 1 } });
						LdrBase::WriteTo<Writer>(out);
					});
				}
				else if (m_wh != v2::Zero())
				{
					Writer::Write(out, EKeyword::FrustumWH, m_name, m_colour, [&]
					{
						Writer::Write(out, EKeyword::Data, m_wh.x, m_wh.y, m_nf.x, m_nf.y);
						LdrBase::WriteTo<Writer>(out);
					});
				}
				else
				{
					Writer::Write(out, EKeyword::FrustumFA, m_name, m_colour, [&]
					{
						Writer::Write(out, EKeyword::Data, RadiansToDegrees(m_fovY), m_aspect, m_nf.x, m_nf.y);
						LdrBase::WriteTo<Writer>(out);
					});
				}
			}
		};
		struct LdrModel :LdrBase<LdrModel>
		{
			std::filesystem::path m_filepath;
			std::optional<LdrAnimation> m_anim;
			bool m_no_materials;

			LdrModel()
				: m_filepath()
				, m_anim()
				, m_no_materials()
			{}

			// Model filepath
			LdrModel& filepath(std::filesystem::path filepath)
			{
				m_filepath = filepath;
				return *this;
			}

			// Add animation to the model
			LdrAnimation& anim()
			{
				if (!m_anim) m_anim = LdrAnimation{};
				return *m_anim;
			}

			LdrModel& no_materials(bool on = true)
			{
				m_no_materials = on;
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Model, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::FilePath, std::format("\"{}\"", m_filepath.string()));
					if (m_anim) m_anim->WriteTo<Writer>(out);
					if (m_no_materials) Writer::Write(out, EKeyword::NoMaterials);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrInstance :LdrBase<LdrInstance>
		{
			std::string m_address = {};
			std::optional<LdrAnimation> m_anim = {};

			/// <summary>Set the object address that this is an instance of</summary>
			LdrInstance& inst(std::string_view address)
			{
				m_address = address;
				return *this;
			}

			/// <summary>Add animation to the instance</summary>
			LdrAnimation& anim()
			{
				if (!m_anim) m_anim = LdrAnimation{};
				return *m_anim;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Instance, m_name, m_colour, [&]
				{
					Writer::Write(out, EKeyword::Data, m_address);
					if (m_anim) m_anim->WriteTo<Writer>(out);
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrGroup :LdrBase<LdrGroup>
		{
			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Group, m_name, m_colour, [&]
				{
					LdrBase::WriteTo<Writer>(out);
				});
			}
		};
		struct LdrCommands :LdrBase<LdrCommands>
		{
			using nstr_t = StringWithLength;
			using param_t = union param_t
			{
				// Don't need type descrimination, the command implies the parameter types
				m4x4 mat4;
				v4 vec4;
				v2 vec2;
				nstr_t nstr;
				float f;
				int i;
				bool b;
			};
			using params_t = pr::vector<param_t, 4>;
			using cmd_t = struct Cmd
			{
				ECommandId m_id;
				params_t m_params;
			};

			pr::vector<cmd_t> m_cmds;

			// Add objects created by this script to scene 'scene_id'
			LdrCommands& add_to_scene(int scene_id)
			{
				m_cmds.push_back({ ECommandId::AddToScene, {{.i = scene_id}} });
				return *this;
			}

			// Apply a transform to an object with the given name
			LdrCommands& object_transform(std::string_view object_name, m4x4 const& o2w)
			{
				m_cmds.push_back({ ECommandId::ObjectToWorld, {{.nstr = object_name}, {.mat4 = o2w}} });
				return *this;
			}

			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::Commands, [&]
				{
					for (auto& cmd : m_cmds)
					{
						Writer::Write(out, EKeyword::Data, [&]
						{
							Writer::Append(out, (int)cmd.m_id);
							switch (cmd.m_id)
							{
								case ECommandId::AddToScene:
								{
									Writer::Append(out, cmd.m_params[0].i);
									break;
								}
								case ECommandId::ObjectToWorld:
								{
									Writer::Append(out, cmd.m_params[0].nstr);
									Writer::Append(out, cmd.m_params[1].mat4);
									break;
								}
								default:
								{
									throw std::runtime_error("Unknown command id");
								}
							}
						});
					}
				});
			}
		};
		struct LdrBinaryStream :LdrBase<LdrBinaryStream>
		{
			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::BinaryStream);
			}
		};
		struct LdrTextStream :LdrBase<LdrTextStream>
		{
			// Write to 'out'
			template <WriterType Writer, typename TOut>
			void WriteTo(TOut& out) const
			{
				Writer::Write(out, EKeyword::TextStream);
			}
		};

		#pragma region LdrBuilder::Implementation
		inline LdrPoint& LdrBuilder::Point(Name name, Colour colour)
		{
			auto ptr = new LdrPoint;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrLine& LdrBuilder::Line(Name name, Colour colour)
		{
			auto ptr = new LdrLine;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrCoordFrame& LdrBuilder::CoordFrame(Name name, Colour colour)
		{
			auto ptr = new LdrCoordFrame;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrTriangle& LdrBuilder::Triangle(Name name, Colour colour)
		{
			auto ptr = new LdrTriangle;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrPlane& LdrBuilder::Plane(Name name, Colour colour)
		{
			auto ptr = new LdrPlane;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrCircle& LdrBuilder::Circle(Name name, Colour colour)
		{
			auto ptr = new LdrCircle;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrSphere& LdrBuilder::Sphere(Name name, Colour colour)
		{
			auto ptr = new LdrSphere;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrBox& LdrBuilder::Box(Name name, Colour colour)
		{
			auto ptr = new LdrBox;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrCylinder& LdrBuilder::Cylinder(Name name, Colour colour)
		{
			auto ptr = new LdrCylinder;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrCone& LdrBuilder::Cone(Name name, Colour colour)
		{
			auto ptr = new LdrCone;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrFrustum& LdrBuilder::Frustum(Name name, Colour colour)
		{
			auto ptr = new LdrFrustum;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrModel& LdrBuilder::Model(Name name, Colour colour)
		{
			auto ptr = new LdrModel;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrInstance& LdrBuilder::Instance(Name name, Colour colour)
		{
			auto ptr = new LdrInstance;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrGroup& LdrBuilder::Group(Name name, Colour colour)
		{
			auto ptr = new LdrGroup;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrCommands& LdrBuilder::Command(Name name, Colour colour)
		{
			auto ptr = new LdrCommands;
			m_objects.emplace_back(ptr);
			return (*ptr).name(name).colour(colour);
		}
		inline LdrBuilder& LdrBuilder::WrapAsGroup(Name name, Colour colour)
		{
			auto ptr = new LdrGroup;
			swap(m_objects, ptr->m_objects);
			m_objects.emplace_back(ptr);
			(*ptr).name(name).colour(colour);
			return *this;
		}
		inline LdrBuilder& LdrBuilder::BinaryStream()
		{
			auto ptr = new LdrBinaryStream;
			m_objects.emplace_back(ptr);
			return *this;
		}
		inline LdrBuilder& LdrBuilder::TextStream()
		{
			auto ptr = new LdrTextStream;
			m_objects.emplace_back(ptr);
			return *this;
		}
		#pragma endregion
	}

	// Fluent Ldraw script builder
	using Builder = fluent::LdrBuilder;
	using Group = fluent::LdrGroup;
}

#if PR_UNITTESTS&&0
namespace pr::rdr12::ldraw
{
	PRUnitTest(LdrHelperTextTests)
	{
		{
			Builder L;
			L.Box("b", 0xFF00FF00).dim(1).o2w(m4x4::Identity());
			auto str = L.ToText(false);
			PR_EXPECT(str::Equal(str, "*Box b FF00FF00 {*Data {1 1 1}}"));
		}
		{
			Builder L;
			L.Triangle().name("tri").colour(0xFFFF0000).tri(v4{ 0,0,0,1 }, v4{ 1, 0, 0, 1 }, v4{ 0, 1, 0, 1 });
			auto str = L.ToText(false);
			PR_EXPECT(str::Equal(str, "*Triangle tri FFFF0000 {*Data {0 0 0 1 0 0 0 1 0}}"));
		}
		{
			Builder L;
			L.LineD().name("lined").colour(0xFF00FF00).line(v4{ 0,0,0,1 }, v4{ 1, 0, 0, 1 }).line(v4{ 0, 0, 0, 1 }, v4{ 0, 0, 1, 1 });
			auto str = L.ToText(true);
			PR_EXPECT(str::Equal(str, "*LineD lined FF00FF00 {\n\t*Data {\n\t\t0 0 0 1 0 0 0 0 0 0 0 1\n\t}\n}"));
		}
		{
			Builder L;
			L.Box("box", 0xFF00FF00).dim(1, 2, 3);
			
			std::stringstream ss;
			ss << L.ToText(false) << std::flush;
			PR_EXPECT(str::Equal(ss.str(), "*Box box FF00FF00 {*Data {1 2 3}}"));
		}
	}
	PRUnitTest(LdrHelperBinaryTests)
	{
		Builder builder;
		auto& group = builder.Group("TestGroup", 0xFF112233);
		auto& points = group.Point("TestPoints", 0xFF00FF00).size(5.0f);
		for (int i = 0; i != 10; ++i)
		{
			auto f = i * 1.f;
			points.pt(v4{ f, 0, 0.0f, 1.0f });
			points.pt(v4{ f, f, 0.0f, 1.0f });
			points.pt(v4{ 0, f, 0.0f, 1.0f });
		}

		auto ldr = builder.ToBinary();
	}
}
#endif
