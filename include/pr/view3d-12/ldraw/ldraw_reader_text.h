//********************************
// Ldraw Script Text Serialiser
//  Copyright (c) Rylogic Ltd 2025
//********************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	struct TextReader : IReader
	{
		alignas(8) std::byte m_src[224];
		alignas(8) std::byte m_pp[896];
		mutable Location m_location;
		string32 m_keyword;
		wstring32 m_delim;
		int m_section_level;
		int m_nest_level;

		TextReader(std::istream& stream, std::filesystem::path src_filepath, EEncoding enc = EEncoding::utf8, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(std::wistream& stream, std::filesystem::path src_filepath, EEncoding enc = EEncoding::utf16_le, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(TextReader&&) = delete;
		TextReader(TextReader const&) = delete;
		TextReader& operator=(TextReader&&) = delete;
		TextReader& operator=(TextReader const&) = delete;
		~TextReader();

		// Return the current location in the source
		virtual Location const& Loc() const override;

		// Move into a nested section
		virtual void PushSection() override;

		// Leave the current nested section
		virtual void PopSection() override;

		// True when the current position has reached the end of the current section
		virtual bool IsSectionEnd() override;

		// True when the source is exhausted
		virtual bool IsSourceEnd() override;

		// Get the next keyword within the current section.
		// Returns false if at the end of the section
		virtual bool NextKeywordImpl(int& kw) override;

		// Read an identifier from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
		virtual string32 IdentifierImpl(bool incl_dot) override;

		// Read a utf8 string from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
		virtual string32 StringImpl(char escape_char) override;

		// Read an integral value from the current section
		virtual int64_t IntImpl(int byte_count, int radix) override;

		// Read a floating point value from the current section
		virtual double RealImpl(int byte_count) override;

		// Read an enum value from the current section
		virtual int64_t EnumImpl(int byte_count, ParseEnumIdentCB parse) override;

		// Read a boolean value from the current section
		virtual bool BoolImpl() override;
	};
}

#if PR_UNITTESTS
#include <sstream>
#include "pr/common/unittests.h"
#include "pr/common/ldraw.h"
namespace pr::rdr12::ldraw::tests
{
	PRUnitTestClass(LDrawTextSerialiserTests)
	{
		using Builder = pr::ldraw::Builder;

		void Dump(std::string const& data)
		{
			(void)data;
			#if PR_UNITTESTS_VISUALISE
			{
				std::ofstream ofile(temp_dir() / "ldraw_test.ldr");
				ofile.write(data.data(), data.size());
			}
			#endif
		}
		PRUnitTestMethod(TestPoint)
		{
			Builder builder;
			builder.Point("TestPoints", 0xFF00FF00).pt(v3(1, 1, 1)).pt(v3(2, 2, 2)).pt(v3(3, 3, 3));
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Point);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "TestPoints");

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00FF00);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 1, 1, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(2, 2, 2, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(3, 3, 3, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestLine)
		{
			Builder builder;
			builder.Line("TestLines", 0xFF0000FF).style(ELineStyle::LineSegments).line(v3(-1, -1, 0), v3(1, 1, 0)).line(v3(-1, 1, 0), v3(1, -1, 0));
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Line);
			{
				auto line = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "TestLines");

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF0000FF);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Style);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Vector3f().w1() == v4(-1, -1, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(+1, +1, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(-1, +1, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(+1, -1, 0, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestSphere)
		{
			Builder builder;
			builder.Sphere("TestSphere", 0xFFFF0000).radius(1.0f);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Sphere);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "TestSphere");

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF0000);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 1.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestBox)
		{
			Builder builder;
			builder.Box("B", 0xFFFF0000).box(1, 2, 3);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Box);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "B");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF0000);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 1.0f);
				PR_EXPECT(reader.Real<float>() == 2.0f);
				PR_EXPECT(reader.Real<float>() == 3.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestTriangle)
		{
			Builder builder;
			builder.Triangle("T", 0xFF00FF00).tri({0,0,0}, {1,0,0}, {0,1,0});
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Triangle);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "T");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00FF00);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 1, 0, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestQuad)
		{
			Builder builder;
			builder.Quad("Q", 0xFF0000FF).quad({0,0,0}, {1,0,0}, {1,1,0}, {0,1,0});
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Quad);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Q");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF0000FF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 1, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 1, 0, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestPlane)
		{
			Builder builder;
			builder.Plane("P", 0xFFAAAA00).wh(10, 10);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Plane);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "P");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFAAAA00);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 10.0f);
				PR_EXPECT(reader.Real<float>() == 10.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestCircle)
		{
			Builder builder;
			builder.Circle("C", 0xFF00AAFF).radius(2.0f);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Circle);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "C");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00AAFF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 2.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestRect)
		{
			Builder builder;
			builder.Rect("R", 0xFFFF00FF).wh(3, 4);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Rect);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "R");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF00FF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 3.0f);
				PR_EXPECT(reader.Real<float>() == 4.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestGroup)
		{
			Builder builder;
			builder.Group("G", 0xFF808080).Box("inner", 0xFFFF0000).box(1);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Group);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "G");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF808080);

				// Child Box
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Box);
				{
					auto inner = reader.SectionScope();
					PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
					PR_EXPECT(reader.Identifier<string32>() == "inner");
					PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
					PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF0000);
					PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
					PR_EXPECT(reader.Real<float>() == 1.0f);
					PR_EXPECT(reader.Real<float>() == 1.0f);
					PR_EXPECT(reader.Real<float>() == 1.0f);
				}
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestLineBox)
		{
			Builder builder;
			builder.LineBox("LB", 0xFF00FF00).dim(2, 3, 4);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::LineBox);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "LB");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00FF00);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 2.0f);
				PR_EXPECT(reader.Real<float>() == 3.0f);
				PR_EXPECT(reader.Real<float>() == 4.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestGrid)
		{
			Builder builder;
			builder.Grid("Gr", 0xFFAAAAAA).wh(5, 5);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Grid);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Gr");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFAAAAAA);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 5.0f);
				PR_EXPECT(reader.Real<float>() == 5.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestCoordFrame)
		{
			Builder builder;
			builder.CoordFrame("CF");
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::CoordFrame);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "CF");
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestRibbon)
		{
			Builder builder;
			builder.Ribbon("Rb", 0xFFFF8800).pt({0,0,0}).pt({1,1,0}).pt({2,0,0});
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Ribbon);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Rb");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF8800);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 1, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(2, 0, 0, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestPie)
		{
			Builder builder;
			builder.Pie("Pi", 0xFF00FF88).angles(0, 90).radii(0.5f, 1.0f);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Pie);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Pi");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00FF88);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 0.0f);
				PR_EXPECT(reader.Real<float>() == 90.0f);
				PR_EXPECT(reader.Real<float>() == 0.5f);
				PR_EXPECT(reader.Real<float>() == 1.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestPolygon)
		{
			Builder builder;
			builder.Polygon("Pg", 0xFFFFFF00).pt({0,0}).pt({1,0}).pt({0.5f,1});
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Polygon);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Pg");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFFFF00);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Vector2f() == v2(0, 0));
				PR_EXPECT(reader.Vector2f() == v2(1, 0));
				PR_EXPECT(reader.Vector2f() == v2(0.5f, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestCylinder)
		{
			Builder builder;
			builder.Cylinder("Cy", 0xFF00FFFF).hr(2, 0.5f);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Cylinder);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Cy");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00FFFF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 2.0f);
				PR_EXPECT(reader.Real<float>() == 0.5f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestCone)
		{
			Builder builder;
			builder.Cone("Co", 0xFFFF00FF).angle(30).height(2);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Cone);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Co");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF00FF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 30.0f);
				PR_EXPECT(reader.Real<float>() == 0.0f);
				PR_EXPECT(reader.Real<float>() == 2.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestMesh)
		{
			Builder builder;
			builder.Mesh("M", 0xFFFF0000).vert({0,0,0}).vert({1,0,0}).vert({0,1,0}).face(0,1,2);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Mesh);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "M");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFF0000);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Verts);
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 1, 0, 1));

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Faces);
				PR_EXPECT(reader.Int<int>() == 0);
				PR_EXPECT(reader.Int<int>() == 1);
				PR_EXPECT(reader.Int<int>() == 2);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestConvexHull)
		{
			Builder builder;
			builder.ConvexHull("CH", 0xFF00FF00).vert(0,0,0).vert(1,0,0).vert(0,1,0).vert(0,0,1);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::ConvexHull);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "CH");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF00FF00);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Verts);
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(1, 0, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 1, 0, 1));
				PR_EXPECT(reader.Vector3f().w1() == v4(0, 0, 1, 1));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestFrustum)
		{
			Builder builder;
			builder.Frustum("Fr", 0xFF0000FF).wh(2, 1, 0.1f, 10.0f);
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::FrustumWH);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Fr");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF0000FF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 2.0f);
				PR_EXPECT(reader.Real<float>() == 1.0f);
				PR_EXPECT(reader.Real<float>() == 0.1f);
				PR_EXPECT(reader.Real<float>() == 10.0f);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestText)
		{
			Builder builder;
			builder.Text("Txt", 0xFFFFFFFF).text("Hello");
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Text);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Txt");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFFFFFF);
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.String<string32>() == "Hello");
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestLightSource)
		{
			Builder builder;
			builder.LightSource("L").style("Point");
			auto const txt = builder.ToString();
			Dump(txt);

			std::istringstream src(txt);
			TextReader reader(src, {});

			EKeyword kw;
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::LightSource);
			{
				auto scope = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "L");
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Style);
				PR_EXPECT(reader.Identifier<string32>() == "Point");
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
	};
}
#endif
