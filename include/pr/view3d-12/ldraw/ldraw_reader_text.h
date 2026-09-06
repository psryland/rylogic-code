//********************************
// Ldraw Script Text Reader
//  Copyright (c) Rylogic Ltd 2026
//********************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	// Convert caller-owned LDraw source bytes to the UTF-8 contract required by Reader.
	std::string ToUtf8Source(std::string_view source, EEncoding encoding, std::filesystem::path const& filepath = {});

	// Adapts the UTF-8-native script Reader to the streamed LDraw parser interface.
	struct TextReader : IReader
	{
		struct Impl;
		std::unique_ptr<Impl> m_impl;

		TextReader(std::string_view utf8_source, std::filesystem::path src_filepath = {}, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(std::string_view utf8_source, Location const& source_location, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(std::istream& utf8_stream, std::filesystem::path src_filepath = {}, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(TextReader&&) = delete;
		TextReader(TextReader const&) = delete;
		TextReader& operator=(TextReader&&) = delete;
		TextReader& operator=(TextReader const&) = delete;
		~TextReader();

		// Return the current location in the source.
		Location const& Loc() const override;

		// Move into a nested section.
		void PushSection() override;

		// Leave the current nested section.
		void PopSection() override;

		// True when the current position has reached the end of the current section.
		bool IsSectionEnd() override;

		// True when the source is exhausted.
		bool IsSourceEnd() override;

		// Get the next keyword within the current section.
		bool NextKeywordImpl(int& kw) override;

		// Read an identifier from the current section.
		string32 IdentifierImpl(bool incl_dot) override;

		// Read a UTF-8 string from the current section.
		string32 StringImpl(char escape_char) override;

		// Read an integral value from the current section.
		int64_t IntImpl(int byte_count, int radix) override;

		// Read a dense sequence of integral values without repeated interface dispatch.
		void IntsImpl(int byte_count, std::span<int64_t> values, int radix) override;

		// Read a floating point value from the current section.
		double RealImpl(int byte_count) override;

		// Read a dense sequence of floating-point values without repeated interface dispatch.
		void RealsImpl(int byte_count, std::span<double> values) override;

		// Read an enum value from the current section.
		int64_t EnumImpl(int byte_count, ParseEnumIdentCB parse) override;

		// Read a boolean value from the current section.
		bool BoolImpl() override;

		// Return the most recently read keyword for diagnostics.
		string32 LastKeywordString() const override;
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
		inline static constexpr bool CreateVisuals = false;
		using Builder = pr::ldraw::Builder;

		void Dump(std::string const& data)
		{
			(void)data;
			#if PR_UNITTESTS_VISUALISE
			if constexpr (CreateVisuals)
			{
				std::ofstream ofile(temp_dir() / "ldraw_test.ldr");
				ofile.write(data.data(), data.size());
			}
			#endif
		}
		PRUnitTestMethod(TestPoint, Quick)
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
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 1, 1, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(2, 2, 2, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(3, 3, 3, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestLine, Quick)
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
				PR_EXPECT(All(reader.Vector3f().w1() == v4(-1, -1, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(+1, +1, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(-1, +1, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(+1, -1, 0, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestSphere, Quick)
		{
			Builder builder;
			builder.Sphere("TestSphere", 0xFFFF0000).sphere(1.0f);
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
		PRUnitTestMethod(TestBox, Quick)
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
		PRUnitTestMethod(TestTriangle, Quick)
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
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 1, 0, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestQuad, Quick)
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
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 1, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 1, 0, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestPlane, Quick)
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
		PRUnitTestMethod(TestCircle, Quick)
		{
			Builder builder;
			builder.Circle("C", 0xFF00AAFF).circle(2.0f);
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
		PRUnitTestMethod(TestRect, Quick)
		{
			Builder builder;
			builder.Rect("R", 0xFFFF00FF).rect(3, 4);
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
		PRUnitTestMethod(TestGroup, Quick)
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
		PRUnitTestMethod(TestLineBox, Quick)
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
		PRUnitTestMethod(TestGrid, Quick)
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
		PRUnitTestMethod(TestCoordFrame, Quick)
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
		PRUnitTestMethod(TestRibbon, Quick)
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
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 1, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(2, 0, 0, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestPie, Quick)
		{
			Builder builder;
			builder.Pie("Pi", 0xFF00FF88).wedge(0, 90, 0.5f, 1.0f);
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
		PRUnitTestMethod(TestPolygon, Quick)
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
				PR_EXPECT(All(reader.Vector2f() == v2(0, 0)));
				PR_EXPECT(All(reader.Vector2f() == v2(1, 0)));
				PR_EXPECT(All(reader.Vector2f() == v2(0.5f, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestCylinder, Quick)
		{
			Builder builder;
			builder.Cylinder("Cy", 0xFF00FFFF).cylinder(2, 0.5f);
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
		PRUnitTestMethod(TestCone, Quick)
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
		PRUnitTestMethod(TestMesh, Quick)
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
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 1, 0, 1)));

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Faces);
				PR_EXPECT(reader.Int<int>() == 0);
				PR_EXPECT(reader.Int<int>() == 1);
				PR_EXPECT(reader.Int<int>() == 2);
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestConvexHull, Quick)
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

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(1, 0, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 1, 0, 1)));
				PR_EXPECT(All(reader.Vector3f().w1() == v4(0, 0, 1, 1)));
			}
			PR_EXPECT(!reader.NextKeyword(kw));
		}
		PRUnitTestMethod(TestFrustum, Quick)
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
		PRUnitTestMethod(TestText, Quick)
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
		PRUnitTestMethod(TestLightSource, Quick)
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

#if PR_UNITTESTS
#include <sstream>
#include "pr/common/unittests.h"
namespace pr::rdr12::ldraw::tests
{
	PRUnitTestClass(LDrawTextReaderTests)
	{
		PRUnitTestMethod(CompactObjectHeader, Quick)
		{
			auto source = std::string("*Point Point_A FF123456 { *Data {1 2 3} } *Box FFABCDEF Box_B { *Data {4 5 6} }");
			TextReader reader(source);

			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Point);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Point_A");
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF123456);
				PR_EXPECT(reader.NextKeyword(keyword));
				PR_EXPECT(keyword == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f() == v3(1, 2, 3)));
			}
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Box);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFABCDEF);
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Box_B");
				PR_EXPECT(reader.NextKeyword(keyword));
				PR_EXPECT(keyword == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f() == v3(4, 5, 6)));
			}
			PR_EXPECT(!reader.NextKeyword(keyword));
		}
		PRUnitTestMethod(IncludeBridge, Quick)
		{
			PathResolver resolver;
			resolver.AddString("child", "*Point Child FFFFFFFF { *Data {7 8 9} }");
			auto source = std::string("#include \"child\"\n");
			TextReader reader(source, "root.ldr", nullptr, nullptr, resolver);

			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Point);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Child");
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFFFFFF);
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f() == v3(7, 8, 9)));
			}
		}
		PRUnitTestMethod(SourceEncodingConversion, Quick)
		{
			auto utf16_le = std::string("\xFF\xFE\x2A\x00\x54\x00\x65\x00\x78\x00\x74\x00\x20\x00\x7B\x00\x22\x00\x3D\xD8\x00\xDE\x22\x00\x7D\x00", 26);
			auto converted = ToUtf8Source(utf16_le, EEncoding::auto_detect, "unicode.ldr");
			PR_EXPECT(converted == "*Text {\"\xF0\x9F\x98\x80\"}");

			auto extended = std::string("\xE9", 1);
			PR_EXPECT(ToUtf8Source(extended, EEncoding::ascii_extended) == "\xC3\xA9");
		}
		PRUnitTestMethod(AdjacentStrings, Quick)
		{
			auto source = std::string("*Text {\"foo\" \"bar\"} *Text {\"left\", \"right\"}");
			TextReader reader(source);

			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Text);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.StringImpl(0) == "foobar");
			}
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Text);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.StringImpl(0) == "left");
				PR_EXPECT(reader.StringImpl(0) == "right");
			}
		}
		PRUnitTestMethod(IncludeBomLocation, Quick)
		{
			// Compare equivalent includes so the only location difference is the stripped physical BOM.
			auto include_offset = [](std::string child)
			{
				PathResolver resolver;
				resolver.AddString("child", std::move(child));
				TextReader reader("#include \"child\"\n", "root.ldr", nullptr, nullptr, resolver);
				EKeyword keyword;
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Point);
				return reader.Loc().m_offset;
			};

			auto plain_offset = include_offset("*Point {}");
			auto bom_offset = include_offset("\xEF\xBB\xBF*Point {}");
			PR_EXPECT(bom_offset == plain_offset + 3);
		}
		PRUnitTestMethod(StreamSourceLocation, Quick)
		{
			auto filepath = std::filesystem::path("snapshot.ldr");
			auto stream = filesys::FileSnapshotStream(filepath, "xx*Point {}", 2);
			TextReader reader(stream);

			auto const& loc = reader.Loc();
			PR_EXPECT(loc.m_filepath == filepath);
			PR_EXPECT(loc.m_filesize == 11);
			PR_EXPECT(loc.m_offset == 2);

			auto source = std::string("*Point {}");
			auto source_location = Location{ .m_filesize = 40, .m_offset = 12, .m_column = 3, .m_line = 2 };
			TextReader memory_reader(source, source_location);
			auto const& memory_loc = memory_reader.Loc();
			PR_EXPECT(memory_loc.m_filepath.empty());
			PR_EXPECT(memory_loc.m_filesize == 40);
			PR_EXPECT(memory_loc.m_offset == 12);
			PR_EXPECT(memory_loc.m_column == 3);
			PR_EXPECT(memory_loc.m_line == 2);
		}
		PRUnitTestMethod(SourceEndLocation, Quick)
		{
			auto source = std::string("*Point {}");
			auto filepath = std::filesystem::path("finished.ldr");
			TextReader reader(source, filepath);
			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword));
			{
				auto section = reader.SectionScope();
			}
			PR_EXPECT(reader.IsSourceEnd());

			auto const& loc = reader.Loc();
			PR_EXPECT(loc.m_filepath == filepath);
			PR_EXPECT(loc.m_filesize == static_cast<int64_t>(source.size()));
			PR_EXPECT(loc.m_offset == static_cast<int64_t>(source.size()));
		}
	};
}
#endif
