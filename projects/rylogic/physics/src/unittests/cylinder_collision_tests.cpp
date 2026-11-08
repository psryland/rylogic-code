//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Narrowphase tests for every cylinder shape pair.
// Each scenario is run through the CPU dispatcher (collision::Collide) and the GPU dispatcher compiled as C++ (physics::CollideShapes),
// and both results are checked against analytic expectations. The measured depth and axis errors are logged because they show
// which pairs are accurate enough with the generic GJK path.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/unittest_helpers.h"
#include "pr/physics/physics.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "src/compute/physics_types.h"
#include "src/compute/collision.hlsli"
#include "src/compute/gjk.hlsli"

namespace pr::physics::tests
{
	PRUnitTestClass(CylinderCollisionTests)
	{
		// The cylinder under test. It is upright (axis = Z) in shape space.
		static constexpr float CylLength = 1.0f;
		static constexpr float CylRadius = 0.5f;

		// Penetration used by the resting scenarios
		static constexpr float RestDepth = 0.02f;

		// Expected outcome of one scenario. 'min_points' and 'max_points' bound the manifold point count.
		struct Expect
		{
			v4 axis;
			float depth;
			int min_points;
			int max_points;
		};

		// A partner shape with its world transform and the height of its top surface.
		struct Partner
		{
			std::string_view name;
			collision::Shape const* shape;
			m4x4 o2w;
			float top;
			bool flat; // True if the partner's top is a flat face
		};

		// Results gathered for the accuracy report
		struct Errors
		{
			float max_depth_error = 0;
			float max_axis_error = 0;
		};

		// Collide 'a' and 'b' on the CPU and with the C++ build of the GPU code. Check both against 'expected', or for no contact when null.
		void Check(std::string_view label, collision::Shape const& a, m4x4 const& a2w, collision::Shape const& b, m4x4 const& b2w, Expect const* expected, Errors& errors, float depth_tol = 2e-3f, float axis_tol = 2e-3f)
		{
			// Pack the shapes for the GPU code
			hlsl::StructuredBuffer<float4> verts;
			hlsl::StructuredBuffer<GpuPolytopeFace> faces;
			hlsl::StructuredBuffer<GpuPolytopeEdge> edges;
			auto sa = PackShape(a, verts, &faces, &edges);
			auto sb = PackShape(b, verts, &faces, &edges);

			// Run both dispatchers
			collision::Contact cpu;
			collision::Collide(a, a2w, b, b2w, cpu);
			auto gpu_contact = GpuContact{};
			physics::CollideShapes(sa, a2w, sb, b2w, verts, faces, edges, gpu_contact);
			auto gpu = To<collision::Contact>(gpu_contact);

			// Check each result against the analytic expectation
			for (auto [which, c] : { std::pair{"CPU", &cpu}, std::pair{"GPU", &gpu} })
			{
				// Report the measured error so the accuracy of each pair can be compared
				if (expected != nullptr && c->contact())
				{
					auto depth_error = Abs(c->m_depth - expected->depth);
					auto axis_error = Length(c->m_axis - expected->axis);
					errors.max_depth_error = std::max(errors.max_depth_error, depth_error);
					errors.max_axis_error = std::max(errors.max_axis_error, axis_error);
					log_stream() << std::format("{} {}: depth={:.6f} (err {:.2e}) axis=({:.4f},{:.4f},{:.4f}) (err {:.2e}) points={}\n",
						label, which, c->m_depth, depth_error, c->m_axis.x, c->m_axis.y, c->m_axis.z, axis_error, c->Count());
				}
				else
				{
					log_stream() << std::format("{} {}: contact={}\n", label, which, c->contact());
				}

				// Separated cases must not report contact
				PR_EXPECT(c->contact() == (expected != nullptr));
				if (expected == nullptr || !c->contact())
					continue;

				// The axis and depth must match the analytic contact
				PR_EXPECT(Abs(c->m_depth - expected->depth) <= depth_tol);
				PR_EXPECT(Length(c->m_axis - expected->axis) <= axis_tol);
				PR_EXPECT(c->Count() >= expected->min_points && c->Count() <= expected->max_points);

				// Manifold points lie on the contact plane, half way between the two surfaces
				auto plane_d = Dot(c->Point() - v4::Origin(), c->m_axis);
				for (auto const& pt : c->Points())
					PR_EXPECT(Abs(Dot(pt - v4::Origin(), c->m_axis) - plane_d) <= 2 * depth_tol);
			}
		}

		// Run the standard scenarios for one partner: cap rest, side rest, rim rest, separated, deep, and the reversed pair order.
		void RunPartner(Partner const& p)
		{
			// The cylinder is shape B in the canonical ordering here, so the expected axis points up from the partner (+Z).
			auto cyl = collision::ShapeCylinder(CylLength, CylRadius);
			auto hlen = CylLength * 0.5f;
			auto errors = Errors{};
			auto up = v4::ZAxis();
			auto const& sp = *p.shape;

			// Cap resting on the partner. Flat partners touch the whole cap; curved or edge partners touch at a point or a line.
			{
				auto c2w = m4x4::Translation(0, 0, p.top + hlen - RestDepth);
				auto exp = Expect{ up, RestDepth, p.flat ? 3 : 1, 4 };
				Check(std::format("{} cap", p.name), sp, p.o2w, cyl, c2w, &exp, errors);
			}

			// Side resting on the partner, with the cylinder axis along Y. Flat partners touch along a line.
			{
				auto c2w = m4x4::Transform(m3x3::RotationDeg(90, 0, 0), v4(0, 0, p.top + CylRadius - RestDepth, 1));
				auto exp = Expect{ up, RestDepth, p.flat ? 2 : 1, 2 };
				Check(std::format("{} side", p.name), sp, p.o2w, cyl, c2w, &exp, errors);
			}

			// Rim resting on a flat partner after tilting the cylinder by 30 degrees about X
			if (p.flat)
			{
				auto tilt = 30.0f * constants<float>::tau_by_360;
				auto ext = hlen * Cos(tilt) + CylRadius * Sin(tilt);
				auto c2w = m4x4::Transform(m3x3::RotationDeg(30, 0, 0), v4(0, 0, p.top + ext - RestDepth, 1));
				auto exp = Expect{ up, RestDepth, 1, 1 };
				Check(std::format("{} rim", p.name), sp, p.o2w, cyl, c2w, &exp, errors);
			}

			// Separated by a small gap
			{
				auto c2w = m4x4::Translation(0, 0, p.top + hlen + 0.01f);
				Check(std::format("{} separated", p.name), sp, p.o2w, cyl, c2w, nullptr, errors);
			}

			// Deep penetration of a flat partner. The upward axis is still the minimum separation direction.
			if (p.flat)
			{
				auto deep = 0.3f;
				auto c2w = m4x4::Translation(0, 0, p.top + hlen - deep);
				auto exp = Expect{ up, deep, 1, 4 };
				Check(std::format("{} deep", p.name), sp, p.o2w, cyl, c2w, &exp, errors, 5e-3f, 5e-3f);
			}

			// Reversing the pair order must reverse the axis and keep the depth
			{
				auto c2w = m4x4::Translation(0, 0, p.top + hlen - RestDepth);
				auto exp = Expect{ -up, RestDepth, p.flat ? 3 : 1, 4 };
				Check(std::format("{} flipped", p.name), cyl, c2w, sp, p.o2w, &exp, errors);
			}

			log_stream() << std::format("{}: max depth error={:.2e}, max axis error={:.2e}\n\n", p.name, errors.max_depth_error, errors.max_axis_error);
		}

		PRUnitTestMethod(CylinderVsSphere, Quick)
		{
			// A sphere touches the cap and side at a single point
			auto sph = collision::ShapeSphere(0.5f);
			RunPartner({ "sphere", sph, m4x4::Identity(), 0.5f, false });
		}
		PRUnitTestMethod(CylinderVsBox, Quick)
		{
			// The box top is a large flat face at z = 0.5
			auto box = collision::ShapeBox(v4{ 4, 4, 1, 0 });
			RunPartner({ "box", box, m4x4::Identity(), 0.5f, true });
		}
		PRUnitTestMethod(CylinderVsLine, Quick)
		{
			// A capsule lying along X under the cylinder. Its top is a ridge, so a cap touches it along a line.
			auto line = collision::ShapeLine(2.0f, 0.1f);
			auto l2w = m4x4::Transform(m3x3::RotationDeg(0, 90, 0), v4::Origin());
			RunPartner({ "line", line, l2w, 0.1f, false });

			// The flat cap rests on the capsule ridge along a line
			auto cyl = collision::ShapeCylinder(CylLength, CylRadius);
			auto errors = Errors{};
			auto exp = Expect{ v4::ZAxis(), RestDepth, 2, 2 };
			Check("line cap ridge", line, l2w, cyl, m4x4::Translation(0, 0, 0.1f + CylLength * 0.5f - RestDepth), &exp, errors);
		}
		PRUnitTestMethod(CylinderVsTriangle, Quick)
		{
			// A large triangle in the XY plane, facing +Z
			auto tri = collision::ShapeTriangle(v4(-4, -4, 0, 1), v4(4, -4, 0, 1), v4(0, 4, 0, 1));
			RunPartner({ "triangle", tri, m4x4::Identity(), 0.0f, true });
		}
		PRUnitTestMethod(CylinderVsPolytope, Quick)
		{
			// A slab polytope with its top face at z = 0.5
			v4 const pts[] = {
				v4(-2, -2, -0.5f, 1), v4(+2, -2, -0.5f, 1), v4(-2, +2, -0.5f, 1), v4(+2, +2, -0.5f, 1),
				v4(-2, -2, +0.5f, 1), v4(+2, -2, +0.5f, 1), v4(-2, +2, +0.5f, 1), v4(+2, +2, +0.5f, 1),
			};
			auto poly = collision::BuildPolytopeFromPoints(pts);
			RunPartner({ "polytope", poly.as<collision::ShapePolytope>(), m4x4::Identity(), 0.5f, true });
		}
		PRUnitTestMethod(CylinderVsCylinder, Quick)
		{
			// A wide upright cylinder whose top cap is flat at z = 0.5
			auto base = collision::ShapeCylinder(1.0f, 2.0f);
			RunPartner({ "cylinder", base, m4x4::Identity(), 0.5f, true });

			// Two cylinders lying side by side with parallel axes touch along a line. Both surfaces are curved across the contact,
			// so the generic GPU EPA (24 vertices) stops at an approximate face. Its axis error is about 2e-2 here.
			auto cyl = collision::ShapeCylinder(CylLength, CylRadius);
			auto errors = Errors{};
			auto lie = m3x3::RotationDeg(90, 0, 0);
			auto exp = Expect{ v4::XAxis(), RestDepth, 2, 2 };
			Check("cylinder side-by-side", cyl, m4x4::Transform(lie, v4::Origin()), cyl, m4x4::Transform(lie, v4(2 * CylRadius - RestDepth, 0, 0, 1)), &exp, errors, 2e-3f, 5e-2f);

			// Crossed cylinders lying on their sides touch at a single point. See above for the tolerance.
			auto cross = m3x3::RotationDeg(0, 90, 0);
			auto exp_cross = Expect{ v4::ZAxis(), RestDepth, 1, 1 };
			Check("cylinder crossed", cyl, m4x4::Transform(lie, v4::Origin()), cyl, m4x4::Transform(cross, v4(0, 0, 2 * CylRadius - RestDepth, 1)), &exp_cross, errors, 2e-3f, 5e-2f);
			log_stream() << std::format("cylinder pairs: max depth error={:.2e}, max axis error={:.2e}\n", errors.max_depth_error, errors.max_axis_error);
		}
		PRUnitTestMethod(RollingOnLargeBox, Quick)
		{
			// A cylinder lying on a ground-sized box must give the same line contact at every roll angle.
			// The box is much larger than the cylinder, which stresses the precision of the generic GJK path.
			auto cyl = collision::ShapeCylinder(1.2f, 0.3f);
			auto ground = collision::ShapeBox(v4{ 200, 200, 10, 0 });
			auto g2w = m4x4::Translation(-30, 40, -5);
			auto errors = Errors{};
			auto exp = Expect{ -v4::ZAxis(), 0.006f, 2, 2 };
			for (int i = 0; i != 360; ++i)
			{
				auto roll = m3x3::RotationDeg(float(i), 0, 0) * m3x3::RotationDeg(0, 90, 0);
				auto c2w = m4x4::Transform(roll, v4(-8.0f, -24.0f + i * 0.0667f, 0.3f - 0.006f, 1));
				Check(std::format("roll {}", i), cyl, c2w, ground, g2w, &exp, errors);
			}
			log_stream() << std::format("rolling: max depth error={:.2e}, max axis error={:.2e}\n", errors.max_depth_error, errors.max_axis_error);

			// Random lying poses on a ground-sized box, through both the CPU and the GPU collision code.
			// The Minkowski coordinates are tens of metres, so this checks that EPA converges at the float precision of those coordinates.
			{
				auto g = collision::ShapeBox(v4{ 107, 107, 10, 0 });
				hlsl::StructuredBuffer<float4> verts;
				hlsl::StructuredBuffer<GpuPolytopeFace> faces;
				hlsl::StructuredBuffer<GpuPolytopeEdge> edges;
				auto sa = PackShape(cyl, verts, &faces, &edges);
				auto sb = PackShape(g, verts, &faces, &edges);
				auto rng = std::default_random_engine(1);
				auto roll = std::uniform_real_distribution<float>(0, 360);
				auto tilt = std::uniform_real_distribution<float>(-0.1f, 0.1f);
				auto pos = std::uniform_real_distribution<float>(-30, 30);
				auto dep = std::uniform_real_distribution<float>(0.002f, 0.008f);
				auto max_depth_error = 0.0f;
				auto min_axis_dot = 1.0f;
				auto max_point_dist = 0.0f;
				for (int i = 0; i != 20000; ++i)
				{
					// Pose the cylinder in world space. The deepest point is on the lower rim of the lower cap.
					auto rot = m3x3::RotationDeg(roll(rng), 0, 0) * m3x3::RotationDeg(0, 90 + tilt(rng), 0);
					auto d = dep(rng);
					auto o2w = m4x4::Transform(rot, v4(pos(rng), pos(rng), 0.3f - d, 1));
					auto az = Abs(o2w.z.z);
					auto expect_depth = d - 0.3f + 0.6f * az + 0.3f * Sqrt(1 - az * az);

					// Collide in the cylinder's frame, as the GPU detector does
					auto b2a = InvertOrthonormal(o2w) * m4x4::Translation(0, 0, -5);
					auto down = InvertOrthonormal(o2w) * -v4::ZAxis();
					auto gc = GpuContact{};
					physics::CollideShapes(sa, m4x4::Identity(), sb, b2a, verts, faces, edges, gc);
					auto cc = collision::Contact{};
					collision::Collide(cyl, m4x4::Identity(), g, b2a, cc);
					for (auto const& c : { To<collision::Contact>(gc), cc })
					{
						PR_EXPECT(c.contact());
						max_depth_error = std::max(max_depth_error, Abs(c.m_depth - expect_depth));
						min_axis_dot = std::min(min_axis_dot, Dot(c.m_axis, down));
						for (auto const& p : c.Points())
							max_point_dist = std::max(max_point_dist, Length(p.w0()));
					}
				}
				log_stream() << std::format("random poses: max depth error={:.2e}, min axis dot={:.7f}, max point distance={:.4f}\n", max_depth_error, min_axis_dot, max_point_dist);
				PR_EXPECT(max_depth_error < 2e-4f);
				PR_EXPECT(min_axis_dot > 0.9999f);
				PR_EXPECT(max_point_dist < 0.68f);
			}
		}
	};

	// Shape-level queries for the cylinder: bounds, support, closest point, mass, ray cast, and surface sampling.
	PRUnitTestClass(CylinderShapeTests)
	{
		// Build a closed prism with 'sides' sides that approximates a cylinder of the given size.
		static byte_data<16> Prism(float length, float radius, int sides)
		{
			// Place the rim points of both caps around the Z axis
			auto pts = std::vector<v4>{};
			for (int i = 0; i != sides; ++i)
			{
				auto ang = constants<float>::tau * i / sides;
				pts.push_back(v4(radius * std::cos(ang), radius * std::sin(ang), -0.5f * length, 1));
				pts.push_back(v4(radius * std::cos(ang), radius * std::sin(ang), +0.5f * length, 1));
			}
			return collision::BuildPolytopeFromPoints(pts);
		}

		PRUnitTestMethod(BoundsSupportAndClosestPoint, Quick)
		{
			auto cyl = collision::ShapeCylinder(2.0f, 0.5f);

			// The bounding box is tight on the radius and half-length
			PR_EXPECT(FEql(cyl.m_base.m_bbox.m_radius, v4(0.5f, 0.5f, 1.0f, 0)));

			// Support points lie on the rim of the cap facing the direction
			int id;
			PR_EXPECT(FEql(collision::SupportVertex(cyl, v4(1, 0, 1, 0), 0, id), v4(0.5f, 0, 1, 1)));
			PR_EXPECT(FEql(collision::SupportVertex(cyl, v4(0, -1, -1, 0), 0, id), v4(0, -0.5f, -1, 1)));
			PR_EXPECT(FEql(collision::SupportVertex(cyl, v4(0, 0, 1, 0), 0, id), v4(0, 0, 1, 1)));

			// Closest points clamp to the side, the cap, and the rim. Inside points are their own closest point.
			auto dist = 0.0f;
			auto closest = v4{};
			collision::ClosestPoint(cyl, v4(2, 0, 0, 1), dist, closest);
			PR_EXPECT(FEql(dist, 1.5f) && FEql(closest, v4(0.5f, 0, 0, 1)));
			collision::ClosestPoint(cyl, v4(0, 0, 3, 1), dist, closest);
			PR_EXPECT(FEql(dist, 2.0f) && FEql(closest, v4(0, 0, 1, 1)));
			collision::ClosestPoint(cyl, v4(0, 1.5f, 2, 1), dist, closest);
			PR_EXPECT(FEql(dist, std::sqrt(2.0f)) && FEql(closest, v4(0, 0.5f, 1, 1)));
			collision::ClosestPoint(cyl, v4(0.1f, 0.1f, 0.2f, 1), dist, closest);
			PR_EXPECT(dist == 0 && FEql(closest, v4(0.1f, 0.1f, 0.2f, 1)));
		}
		PRUnitTestMethod(MassProperties, Quick)
		{
			// A fine prism converges to the cylinder, so it is an independent check of the closed-form mass and inertia
			auto cyl = collision::ShapeCylinder(2.0f, 0.5f);
			auto prism_buf = Prism(2.0f, 0.5f, 120);
			auto const& prism = prism_buf.as<collision::ShapePolytope>();
			for (auto dist : { EMassDistribution::Solid, EMassDistribution::Shell })
			{
				auto mp_cyl = CalcMassProperties(cyl, 3.0f, dist);
				auto mp_poly = CalcMassProperties(prism, 3.0f, dist);
				log_stream() << std::format("dist={} mass cyl={} prism={}, Ixx cyl={} prism={}, Izz cyl={} prism={}\n",
					static_cast<int>(dist), mp_cyl.m_mass, mp_poly.m_mass,
					mp_cyl.m_os_unit_inertia.x.x, mp_poly.m_os_unit_inertia.x.x, mp_cyl.m_os_unit_inertia.z.z, mp_poly.m_os_unit_inertia.z.z);
				PR_EXPECT(FEqlRelative(mp_cyl.m_mass, mp_poly.m_mass, 2e-3f));
				PR_EXPECT(FEqlRelative(mp_cyl.m_os_unit_inertia, mp_poly.m_os_unit_inertia, 2e-3f));
				PR_EXPECT(FEqlAbsolute(mp_cyl.m_centre_of_mass, v4::Zero(), 1e-6f));
			}

			// Closed form of the solid cylinder
			auto pi = constants<float>::tau_by_2;
			auto solid = CalcMassProperties(cyl, 3.0f, EMassDistribution::Solid);
			PR_EXPECT(FEqlRelative(solid.m_mass, 3.0f * pi * 0.25f * 2.0f, 1e-5f));
			PR_EXPECT(FEqlRelative(solid.m_os_unit_inertia.x.x, (3 * 0.25f + 4.0f) / 12.0f, 1e-5f));
			PR_EXPECT(FEqlRelative(solid.m_os_unit_inertia.z.z, 0.5f * 0.25f, 1e-5f));
		}
		PRUnitTestMethod(RayCast, Quick)
		{
			auto cyl = collision::ShapeCylinder(1.0f, 0.5f);

			// Down the axis through both caps
			auto r = collision::RayCast(collision::Ray(v4(0, 0, 2, 1), v4(0, 0, -4, 0)), cyl);
			PR_EXPECT(r.m_shape != nullptr && FEql(r.m_t0, 0.375f) && FEql(r.m_t1, 0.625f) && FEql(r.m_normal, v4(0, 0, 1, 0)));

			// Through the side
			r = collision::RayCast(collision::Ray(v4(2, 0, 0, 1), v4(-4, 0, 0, 0)), cyl);
			PR_EXPECT(r.m_shape != nullptr && FEql(r.m_t0, 0.375f) && FEql(r.m_t1, 0.625f) && FEql(r.m_normal, v4(1, 0, 0, 0)));

			// Just above the cap misses, and a thick ray reaches it
			r = collision::RayCast(collision::Ray(v4(2, 0, 0.6f, 1), v4(-4, 0, 0, 0)), cyl);
			PR_EXPECT(r.m_shape == nullptr);
			r = collision::RayCast(collision::Ray(v4(2, 0, 0.6f, 1), v4(-4, 0, 0, 0), 0.2f), cyl);
			PR_EXPECT(r.m_shape != nullptr);

			// Passing beside the curved side misses
			r = collision::RayCast(collision::Ray(v4(2, 0.6f, 0, 1), v4(-4, 0, 0, 0)), cyl);
			PR_EXPECT(r.m_shape == nullptr);
		}
		PRUnitTestMethod(SurfacePlan, Quick)
		{
			// The surface samples cover the two caps and the side exactly, with outward normals on each patch
			auto cyl = collision::ShapeCylinder(2.0f, 0.5f);
			auto plan = surface::BuildPlan(cyl.m_base, 0.1f);
			auto area = 0.0;
			auto normal_sum = v4::Zero();
			for (uint32_t i = 0; i != plan.m_count; ++i)
			{
				// Each sample lies on the surface and its normal points away from the axis or out of the cap
				auto s = surface::EmitSurfaceSample(plan, i);
				auto on_cap = FEqlAbsolute(Abs(s.m_pos_local.z), 1.0f, 1e-5f) && Len(s.m_pos_local.x, s.m_pos_local.y) <= 0.5f + 1e-5f;
				auto on_side = FEqlAbsolute(Len(s.m_pos_local.x, s.m_pos_local.y), 0.5f, 1e-5f) && Abs(s.m_pos_local.z) <= 1.0f + 1e-5f;
				PR_EXPECT(on_cap || on_side);
				PR_EXPECT(Dot(s.m_normal_local, s.m_pos_local.w0()) > 0);
				area += s.m_darea;
				normal_sum += s.m_normal_local * s.m_darea;
			}
			auto pi = constants<double>::tau_by_2;
			auto expected = 2 * pi * 0.25 + 2 * pi * 0.5 * 2.0;
			log_stream() << std::format("surface samples={} area={} expected={}\n", plan.m_count, area, expected);
			PR_EXPECT(std::abs(area - expected) < 1e-4 * expected);
			PR_EXPECT(Length(normal_sum) < 1e-3f);
		}
	};
}
#endif
