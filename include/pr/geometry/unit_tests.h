//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/geometry/point.h"
#include "pr/geometry/distance.h"
#include "pr/geometry/closest_point.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"
namespace pr::geometry
{
	PRUnitTestClass(PointTests)
	{
		PRUnitTestMethod(PointWithinConvexPolygon)
		{
			v4 poly[] =
			{
				v4(-2.0f, -1.0f, 0.0f, 1.0f),
				v4(+2.5f, -1.5f, 0.0f, 1.0f),
				v4(+2.0f, +0.5f, 0.0f, 1.0f),
				v4(-0.5f, +2.0f, 0.0f, 1.0f),
			};
			PR_EXPECT(PointWithinConvexPolygon(v4::Origin(), poly, _countof(poly)));
			PR_EXPECT(!PointWithinConvexPolygon(poly[0], poly, _countof(poly)));
			PR_EXPECT(!PointWithinConvexPolygon(v4(-1.0f, +2.0f, 0.0f, 1.0f), poly, _countof(poly)));
			PR_EXPECT(PointWithinConvexPolygon(v4(+1.0f, -0.5f, 0.0f, 1.0f), poly, _countof(poly)));
		}
	};
	PRUnitTestClass(DistanceTests)
	{
		PRUnitTestMethod(DistanceSq_PointToLine)
		{
			auto s = pr::v4(1.0f, 1.0f, 0.0f, 1.0f);
			auto e = pr::v4(3.0f, 2.0f, 0.0f, 1.0f);
			auto a = pr::v4(2.0f, 1.0f, 0.0f, 1.0f);
			PR_EXPECT(FEql(distance::PointToLineSq(s, s, e), 0.0f));
			PR_EXPECT(FEql(distance::PointToLineSq(e, s, e), 0.0f));
			PR_EXPECT(FEql(distance::PointToLineSq((s + e) * 0.5f, s, e), 0.0f));
			PR_EXPECT(FEql(distance::PointToLineSq(a, s, e), Sqr(sin(atan(0.5f)))));
		}
	};
	PRUnitTestClass(ClosestPointTests)
	{
		PRUnitTestMethod(PointToPlane)
		{
		}
		PRUnitTestMethod(LineToBBox)
		{
			std::default_random_engine rng;
			for (int i = 0; i != 100; ++i)
			{
				auto bbox = BBox{ Random<v4>(rng, v4::Origin(), 3.0f).w1(), Random<v4>(rng, v4{0.f}, v4{3.f}).w0() };
				auto s = Random<v4>(rng, v4::Origin(), 10.0f).w1();
				auto e = Random<v4>(rng, v4::Origin(), 10.0f).w1();

				v4 pt0, pt1;
				auto sep = closest_point::LineToBBox(s, e, bbox, pt0, pt1);
				//auto dist = -sep.Depth();
				auto axis = sep.SeparatingAxis();

				rdr12::ldraw::Builder builder;
				builder.Box("bbox", 0x8000FF00).bbox(bbox);
				builder.Line("line", 0xFFFF0000).line(s, e);
				builder.Box("cp1", 0xFF0000FF).dim(0.01f).pos(pt0);
				builder.Box("cp2", 0xFF0000FF).dim(0.01f).pos(pt1);
				builder.Line("axis", 0xFF0000FF).line(pt0, pt1);
				//builder.Write(L"\\dump\\test.ldr");
			}
		}
		PRUnitTestMethod(RayVsTriangle)
		{
			using namespace pr::rdr12::ldraw;

			std::default_random_engine rng;
			for (int i = 0; i != 100; ++i)
			{
				auto s = Random<v4>(rng, v4::Origin(), 10.0f).w1();
				auto d = Random<v4>(rng, v4::Origin(), 10.0f).w1() - s;

				auto a = Random<v4>(rng, v4::Origin(), 10.0f).w1();
				auto b = Random<v4>(rng, v4::Origin(), 10.0f).w1();
				auto c = Random<v4>(rng, v4::Origin(), 10.0f).w1();

				auto para = closest_point::RayToTriangle(s, d, a, b, c);
				auto pt0 = s + para.w * d;
				auto pt1 = BaryPoint(a, b, c, para.xyz);

				Builder builder;
				builder.Line("ray", 0xFFFF0000).style(ELineStyle::Direction).line(s, 5 * d);
				builder.Triangle("tri", 0xFF0000FF).tri(a, b, c);
				builder.Point("cp0", 0xFFFFFF00).size(20).pt(pt0);
				builder.Point("cp1", 0xFF00FFFF).size(20).pt(pt1);
				//builder.Save(temp_dir() / "geometry.ldr", ESaveFlags::Pretty);
			}
		}
	};
	PRUnitTestClass(Intersect2DTests)
	{
		PRUnitTestMethod(RayVsRay)
		{
			v2 pt;
			PR_EXPECT(intersect::RayVsRay(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 0, 0.5f }, v2{ 2, 1.5f }, pt));
			PR_EXPECT(FEql(pt, v2(1, 1)));

			// Parallel
			PR_EXPECT(!intersect::RayVsRay(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 1, 0 }, v2{ 0, 1 }, pt));

			// Colinear
			PR_EXPECT(!intersect::RayVsRay(v2{ 0, 2 }, v2{ 1, 1 }, v2{ 2, 0 }, v2{ 1, 1 }, pt));
		}
		PRUnitTestMethod(LineVsLine)
		{
			float ta, tb;

			PR_EXPECT(intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 0, 0.5f }, v2{ 2, 1.5f }, ta, tb));
			PR_EXPECT(FEql(ta, 0.5f));
			PR_EXPECT(FEql(tb, 0.5f));

			// Non-parallel but not crossing
			PR_EXPECT(!intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 0, 0.5f }, v2{ 0.9f, 0.95f }, ta, tb));

			// Non-parallel but not crossing, other side
			PR_EXPECT(!intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 1.1f, 1.01f }, v2{ 2, 1.5f }, ta, tb));

			// Parallel
			PR_EXPECT(!intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 1, 0 }, v2{ 0, 1 }, ta, tb));

			// Colinear - meeting at point
			PR_EXPECT(intersect::LineVsLine(v2{ 0, 2 }, v2{ 1, 1 }, v2{ 2, 0 }, v2{ 1, 1 }, ta, tb));
			PR_EXPECT(FEql(ta, 1.0f));
			PR_EXPECT(FEql(tb, 1.0f));

			// Colinear - overlapping
			PR_EXPECT(intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 1, 1 }, v2{ 2, 0 }, ta, tb));
			PR_EXPECT(FEql(ta, 0.5f));
			PR_EXPECT(FEql(tb, 1.0f));

			// Colinear - overlapping b within a
			PR_EXPECT(intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ 0.5f, 1.5f }, v2{ 1.5f, 0.5f }, ta, tb));
			PR_EXPECT(FEql(ta, 0.25f));
			PR_EXPECT(FEql(tb, 1.0f));

			// Colinear - overlapping a within b
			PR_EXPECT(intersect::LineVsLine(v2{ 0, 2 }, v2{ 2, 0 }, v2{ -0.5f, 2.5f }, v2{ 2.5f,-0.5f }, ta, tb));
			PR_EXPECT(FEql(ta, 0.0f));
			PR_EXPECT(FEql(tb, 5.f / 6.f));
		}
		PRUnitTestMethod(LineVsBBox)
		{
		}
	};
	PRUnitTestClass(Intersect3DTests)
	{
		PRUnitTestMethod(LineToBBox)
		{
			float tmin = 0.0f, tmax = 1.0f;
			auto s = pr::v4(+1.0f, +0.2f, +0.5f, 1.0f);
			auto e = pr::v4(-1.0f, -0.2f, -0.4f, 1.0f);
			auto d = e - s;
			auto bbox = BBox(v4::Origin(), v4(0.25f, 0.15f, 0.2f, 0.0f));

			auto r = intersect::RayVsBBox(s, d, bbox, tmin, tmax);
			PR_EXPECT(r);
			PR_EXPECT(FEqlRelative(s + tmin * d, pr::v4(+0.25f, +0.05f, +0.163f, 1.0f), 0.001f));
			PR_EXPECT(FEqlRelative(s + tmax * d, pr::v4(-0.25f, -0.05f, -0.063f, 1.0f), 0.001f));

			s = pr::v4(+1.0f, +0.2f, -0.22f, 1.0f);
			r = intersect::RayVsBBox(s, d, bbox, tmin, tmax);
			PR_EXPECT(!r);
		}
		PRUnitTestMethod(RayVsSphere)
		{
			float tmin = 0.0f, tmax = 1.0f;
			auto s = pr::v4(+1.0f, +0.2f, +0.5f, 1.0f);
			auto e = pr::v4(-1.0f, -0.2f, -0.4f, 1.0f);
			auto d = e - s;
			auto rad = 0.3f;

			auto r = intersect::RayVsSphere(s, d, rad, tmin, tmax);
			PR_EXPECT(r);
			PR_EXPECT(FEqlRelative(s + tmin * d, pr::v4(+0.247f, +0.049f, +0.161f, 1.0f), 0.001f));
			PR_EXPECT(FEqlRelative(s + tmax * d, pr::v4(-0.284f, -0.057f, -0.078f, 1.0f), 0.001f));

			s = pr::v4(+1.0f, +0.2f, -0.22f, 1.0f);
			r = intersect::RayVsSphere(s, d, rad, tmin, tmax);
			PR_EXPECT(!r);
		}
		PRUnitTestMethod(BBoxVsPlane)
		{
			auto p = Plane(v4(0.1f, 0.4f, -0.3f, 1), v4::Normal(0.3f, -0.4f, 0.5f, 0));
			auto b = BBox(v4(0.0f, 0.2f, 0.0f, 1.0f), v4(0.25f, 0.15f, 0.2f, 0));
			auto r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(r);

			b.m_centre = v4(0.0f, 0.1f, 0.0f, 1.0f);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(!r);

			b.m_centre = v4(0.0f, 0.4f, -0.7f, 1.0f);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(r);

			b.m_centre = v4(0.0f, 0.4f, -0.72f, 1.0f);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(!r);

			// Degenerate cases
			p = Plane(v4::Origin(), v4::XAxis());
			b.m_centre = v4(-0.250001f, 0, 0, 1);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(!r);

			b.m_centre = v4(-0.2499f, 0, 0, 1);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(r);

			b.m_centre = v4(+0.2499f, 0, 0, 1);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(r);

			b.m_centre = v4(+0.250001f, 0, 0, 1);
			r = intersect::BBoxVsPlane(b, p);
			PR_EXPECT(!r);
		}
	};
}
#endif