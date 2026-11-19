//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"
#include "src/compute/constraint_coordinates.hlsli"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return a minimally valid D6 descriptor between two endpoints.
		ConstraintDesc MakeD6(BodyRef body_a, BodyRef body_b)
		{
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = body_a;
			desc.m_frame_b.m_body = body_b;
			return desc;
		}

		// Return a hard-locked scalar axis.
		ConstraintAxisDesc LockedAxis()
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			return axis;
		}

		// Require two floating-point values to agree within an absolute tolerance.
		void ExpectNear(float actual, float expected, float tolerance = 1.0e-5f)
		{
			PR_EXPECT(std::abs(actual - expected) <= tolerance);
		}

		// Require a spatial force vector to agree with its expected angular and linear components.
		void ExpectSpatial(v8force actual, v4 expected_angular, v4 expected_linear)
		{
			PR_EXPECT(FEqlAbsolute(actual.ang, expected_angular, 1.0e-5f));
			PR_EXPECT(FEqlAbsolute(actual.lin, expected_linear, 1.0e-5f));
		}

		// Build a two-link floating tree with a controllable root orientation and one translated child attachment.
		std::pair<Articulation, LinkHandle> MakeConstraintTree(float root_x, float root_angle = 0.0f)
		{
			auto const link = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.25f, 1.0f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link, m4x4::Transform(v4::ZAxis(), root_angle, v4{root_x, 0.0f, 0.0f, 1}));
			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(1.0f, 0.0f, 0.0f)), link);
			return {builder.Build(), child};
		}
	}

	// Broken descriptors stay inactive until explicitly repaired, even when their endpoints are no longer submitted.
	PRUnitTestClass(ConstraintCompilerBrokenStateTests)
	{
		// A broken latch must suppress rows before endpoint resolution without changing the descriptor's enabled flag.
		PRUnitTestMethod(BrokenRowsOmitEndpointsUntilRepair, Quick)
		{
			// Start with one genuine active row so an unconditional compiler early-out cannot pass.
			auto shape = collision::ShapeSphere{0.25f};
			auto body_a = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto body_b = RigidBody{&shape, m4x4::Translation(2, 0, 0), Inertia::Sphere(shape.m_radius, 1.0f)};
			body_a.VelocityWS(v8motion{v4::Zero(), v4{5, 0, 0, 0}});
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.D6().m_linear[0] = LockedAxis();
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array{&body_a, &body_b};
			auto const present = BodyRemap(body_ptrs);
			auto const omitted = BodyRemap(std::span<RigidBody* const>{});
			auto const active = CompileConstraints(constraints, present);
			PR_EXPECT(active.m_blocks.size() == 1);
			PR_EXPECT(active.m_rows.size() == 1);

			// Only the engine owns the break latch; produce one overload through that owner without exposing a test-only setter.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Step(1.0f / 60.0f, body_ptrs, constraints);
			PR_EXPECT(constraints.IsBroken(handle));

			// Latching overload leaves the authored enabled flag intact; redundant enable calls do not repair it.
			PR_EXPECT(constraints.Get(handle).m_enabled);
			constraints.SetEnabled(handle, true);
			PR_EXPECT(constraints.IsBroken(handle));
			auto const broken_present = CompileConstraints(constraints, present);
			PR_EXPECT(broken_present.m_blocks.empty());
			PR_EXPECT(broken_present.m_rows.empty());
			auto const broken_omitted = CompileConstraints(constraints, omitted);
			PR_EXPECT(broken_omitted.m_blocks.empty());
			PR_EXPECT(broken_omitted.m_rows.empty());

			// Explicit repair restores both the row and its requirement that live endpoints be submitted.
			constraints.Repair(handle);
			auto const repaired = CompileConstraints(constraints, present);
			PR_EXPECT(!constraints.IsBroken(handle));
			PR_EXPECT(repaired.m_blocks.size() == 1);
			PR_EXPECT(repaired.m_rows.size() == 1);
			PR_EXPECT(repaired.m_blocks[0].m_source == handle);
			PR_THROWS(CompileConstraints(constraints, omitted), std::exception);
		}
	};

	// Nonzero rotation-vector coordinates require their true velocity derivative, not only the frame axes.
	PRUnitTestClass(AngularCoordinateJacobianTests)
	{
		// Differentiate every endpoint rotation independently at rotated nonzero joint frames.
		PRUnitTestMethod(RotationVectorRowsMatchFiniteDifferences, Quick)
		{
			// Observe the compiler's actual coordinate while perturbing physical world-space angular velocity.
			auto a = RigidBody{};
			auto b = RigidBody{};
			a.SetMassProperties(Inertia{1.0f, 1.0f});
			b.SetMassProperties(Inertia{1.0f, 1.0f});
			auto const pose_a = m4x4{m3x3::Rotation(v3{0.31f, -0.42f, 0.2f}), v4::Origin()};
			a.O2W(pose_a);
			auto desc = MakeD6(BodyRef::Rigid(a), BodyRef::Rigid(b));
			desc.m_frame_a.m_constraint_to_body = m4x4::Transform(v4::XAxis(), 0.23f, v4::Origin());
			desc.m_frame_b.m_constraint_to_body = m4x4::Transform(v4::ZAxis(), -0.18f, v4::Origin());
			for (auto& axis : desc.D6().m_angular)
				axis = LockedAxis();

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto bodies = std::array{&a, &b};
			auto remap = BodyRemap{bodies};
			auto max_error = 0.0f;
			for (auto const angle : {0.0f, 0.05f, 0.1f, 1.2f, 3.0f})
			{
				// Cover the near-zero series, its join, and a large principal angle without crossing the half-turn discontinuity.
				auto const pose_b = pose_a * desc.m_frame_a.m_constraint_to_body *
					m4x4::Transform(Normalise(v4{1, 2, -1, 0}), angle, v4::Origin()) * InvertOrthonormal(desc.m_frame_b.m_constraint_to_body);
				b.O2W(pose_b);
				auto const rows = CompileConstraints(constraints, remap).m_rows;
				for (int endpoint = 0; endpoint != 2; ++endpoint)
				{
					// Move each physical endpoint independently so a sign error cannot cancel between the two Jacobians.
					for (int component = 0; component != 3; ++component)
					{
						// A two-milliradian centered perturbation keeps FP32 subtraction noise below the derivative tolerance at large angles.
						auto const initial = endpoint == 0 ? pose_a : pose_b;
						auto rotation = v3{};
						rotation[component] = 2.0e-3f;
						bodies[endpoint]->O2W(m4x4{m3x3::Rotation(rotation), v4::Origin()} * initial);
						auto const positive = CompileConstraints(constraints, remap).m_rows;
						bodies[endpoint]->O2W(m4x4{m3x3::Rotation(-rotation), v4::Origin()} * initial);
						auto const negative = CompileConstraints(constraints, remap).m_rows;
						bodies[endpoint]->O2W(initial);
						for (int axis = 0; axis != 3; ++axis)
						{
							// Compare the true coordinate change with its dual row coefficient, including both endpoint signs.
							auto const derivative = (positive[axis].m_position - negative[axis].m_position) / 4.0e-3f;
							auto const jacobian = endpoint == 0 ? rows[axis].m_jacobian_a : rows[axis].m_jacobian_b;
							max_error = std::max(max_error, std::abs(jacobian.ang[component] - derivative));
							if (std::abs(jacobian.ang[component] - derivative) > 2.5e-4f)
								printf("angular derivative mismatch: angle=%.9g endpoint=%d axis=%d component=%d row=%.9g finite-difference=%.9g\n", angle, endpoint, axis, component, jacobian.ang[component], derivative);

							ExpectNear(jacobian.ang[component], derivative, 2.5e-4f);
						}
					}
				}
			}
			printf("angular coordinate: maximum centered-difference error=%.9g\n", max_error);
		}
	};

	// Frozen swing/twist coordinates and their physical angular derivatives, independent of constraint-row preparation.
	PRUnitTestClass(SphericalCoordinateTests)
	{
		// Return the ordered swing-X, swing-Z, twist-Y values for numerical differentiation.
		static v3 Values(SphericalCoordinates const& coordinates)
		{
			// Keep the two swing components adjacent rather than assuming XYZ log-coordinate order.
			return v3{coordinates.m_swing.x, coordinates.m_swing.y, coordinates.m_twist};
		}

		// Independent double-precision arc sampling and local distance minimization; it does not use the production stationary equation.
		static double BoundaryDistanceReference(v2 point, v2 x_range, v2 z_range)
		{
			// Include all four arcs, since an interior point's closest boundary need not be in its own quadrant.
			auto best = std::numeric_limits<double>::infinity();
			for (int quadrant = 0; quadrant != 4; ++quadrant)
			{
				// Degenerate arcs still parameterize the complete zero-width side segment.
				auto const radius_x = static_cast<double>((quadrant & 1) == 0 ? x_range.y : x_range.x);
				auto const radius_z = static_cast<double>((quadrant & 2) == 0 ? z_range.y : z_range.x);
				auto squared_distance = [&](double angle)
				{
					// Compare actual curve points, independently of any normal or derivative equation.
					auto const x = radius_x * std::cos(angle) - static_cast<double>(point.x);
					auto const z = radius_z * std::sin(angle) - static_cast<double>(point.y);
					return x * x + z * z;
				};
				auto const step = 1.57079632679489661923 / 512.0;
				auto best_index = 0;
				for (int index = 0; index != 513; ++index)
				{
					// The fixtures use ordinary radii; a dense scan brackets the best local arc minimum before refinement.
					if (squared_distance(index * step) < squared_distance(best_index * step))
						best_index = index;
				}
				auto lower = std::max(0, best_index - 1) * step;
				auto upper = std::min(512, best_index + 1) * step;
				for (int iteration = 0; iteration != 64; ++iteration)
				{
					// Distance-only ternary refinement avoids reproducing the production bracket derivation.
					auto const left = (2.0 * lower + upper) / 3.0;
					auto const right = (lower + 2.0 * upper) / 3.0;
					if (squared_distance(left) < squared_distance(right))
						upper = right;
					else
						lower = left;
				}
				best = std::min({best, squared_distance(0.0), squared_distance(1.57079632679489661923), squared_distance(0.5 * (lower + upper))});
			}
			return std::sqrt(best);
		}

		// The law uses quadrant ellipses and actual Euclidean boundary distance, including asymmetric and collapsed sides.
		PRUnitTestMethod(QuadrantEllipseDistance, Quick)
		{
			// Deliberately include points inside the independent X/Z box but outside its ellipse.
			auto const cases = std::array{
				std::tuple{v2{0.8f, 0.8f}, v2{-1, 1}, v2{-1, 1}, false},
				std::tuple{v2{1.5f, 0.8f}, v2{-2, 2}, v2{-0.5f, 0.5f}, false},
				std::tuple{v2{0.1f, 0.1f}, v2{-0.2f, 2}, v2{-0.4f, 1}, true},
				std::tuple{v2{-0.8f, 0.7f}, v2{-0.6f, 1.4f}, v2{-1.2f, 0.8f}, false},
				std::tuple{v2{0, 0}, v2{-0.3f, 1.5f}, v2{-0.6f, 1}, true},
				std::tuple{v2{0, 0.2f}, v2{-0.3f, 1.5f}, v2{-0.6f, 1}, true},
				std::tuple{v2{0.2f, -0.1f}, v2{0, 1}, v2{0, 0.8f}, false},
				std::tuple{v2{-0.2f, -0.1f}, v2{0, 1}, v2{0, 0.8f}, false},
				std::tuple{v2{0, 0.3f}, v2{0, 0}, v2{-0.8f, 0.8f}, true},
				std::tuple{v2{0.2f, 0.3f}, v2{0, 0}, v2{-0.8f, 0.8f}, false},
				std::tuple{v2{0.2f, 0.3f}, v2{0, 0}, v2{0, 0}, false},
				std::tuple{v2{1, 0}, v2{-1, 1}, v2{-0.5f, 0.5f}, true},
			};
			auto max_error = 0.0;
			for (auto const& [point, x_range, z_range, inside] : cases)
			{
				// Compare against an independently minimized boundary rather than radial rescaling of the input.
				auto const measured = MeasureSwingBoundary(point, x_range, z_range);
				auto const expected = (inside ? -1.0 : 1.0) * BoundaryDistanceReference(point, x_range, z_range);
				max_error = std::max(max_error, std::abs(measured.m_distance - expected));
				if (std::abs(measured.m_distance - expected) >= 2.0e-6)
					printf("ellipse distance mismatch: query=(%.9g,%.9g) X=(%.9g,%.9g) Z=(%.9g,%.9g), actual=%.12g expected=%.12g nearest=(%.9g,%.9g)\n", point.x, point.y, x_range.x, x_range.y, z_range.x, z_range.y, measured.m_distance, expected, measured.m_point.x, measured.m_point.y);

				PR_EXPECT(IsFinite(measured.m_point) && IsFinite(measured.m_outward) && std::isfinite(measured.m_distance));
				PR_EXPECT(std::abs(measured.m_distance - expected) < 2.0e-6);
				ExpectNear(Length(measured.m_outward), 1.0f, 2.0e-6f);
			}
			PR_EXPECT(MeasureSwingBoundary(v2{0.1f, 0.1f}, v2{-0.2f, 2}, v2{-0.4f, 1}).m_point.x < 0.0f);
			printf("swing quadrant ellipse: maximum signed-distance difference=%.12g\n", max_error);
		}

		// Constructed swing/twist factors and exact half-turns establish the diagnostic convention.
		PRUnitTestMethod(CoordinatesAndQuaternionSign, Quick)
		{
			// Recover known factors, not values from another invocation of the decomposition helper.
			for (auto const swing : {v3::Zero(), v3{0.8f, 0, -0.5f}, v3{-1.7f, 0, 0.4f}})
			{
				// Both twist signs and quaternion signs represent the same prescribed factorization.
				for (auto const twist : {-1.4f, 0.0f, 1.9f})
				{
					// Quaternion exponentials take half of the rotation vector.
					auto const q = ExpMap<quat>(0.5f * swing) * ExpMap<quat>(v3{0, 0.5f * twist, 0});
					for (auto const sign : {-1.0f, 1.0f})
					{
						// The native helper must recover the factors even when w, y, or a swing component is negative.
						auto const measured = MeasureSphericalCoordinates(sign * q.xyzw);
						PR_EXPECT(measured.m_regular);
						PR_EXPECT(FEqlAbsolute(Values(measured), v3{swing.x, swing.z, twist}, 2.0e-6f));
					}
				}
			}

			// The frozen ordering makes a pure half-turn twist positive and a no-twist half-turn swing choose positive X or Z.
			auto const pi = math::constants<float>::tau_by_2;
			for (auto const sign : {-1.0f, 1.0f})
			{
				// The diagnostic at the swing antipode is defined, but its split derivative is explicitly unavailable.
				auto const twist = MeasureSphericalCoordinates(float4{0, sign, 0, 0});
				auto const swing_x = MeasureSphericalCoordinates(float4{sign, 0, 0, 0});
				auto const swing_z = MeasureSphericalCoordinates(float4{0, 0, sign, 0});
				PR_EXPECT(twist.m_regular && !swing_x.m_regular && !swing_z.m_regular);
				PR_EXPECT(FEqlAbsolute(Values(twist), v3{0, 0, pi}, 1.0e-6f));
				PR_EXPECT(FEqlAbsolute(Values(swing_x), v3{pi, 0, 0}, 1.0e-6f));
				PR_EXPECT(FEqlAbsolute(Values(swing_z), v3{0, pi, 0}, 1.0e-6f));
			}
		}

		// Near-boundary Jacobians follow the selected feature, not a tiny difference between rounded points.
		PRUnitTestMethod(SmoothBoundaryNormalsAndCornerCones, Quick)
		{
			// A unit circle provides a closed-form normal oracle independent of projection and its reported distance.
			auto const base = v2{std::bit_cast<float>(0x3f19999bU), std::bit_cast<float>(0x3f4ccccdU)};
			auto max_error = 0.0f;
			auto check_normal = [&](v2 query, v2 x_range, v2 z_range, v2 expected)
			{
				// Require the direction, not merely a finite vector of unit length.
				auto const measured = MeasureSwingBoundary(query, x_range, z_range);
				auto const error = Length(measured.m_outward - expected);
				max_error = std::max(max_error, error);
				if (error >= 2.0e-5f)
					printf("boundary normal mismatch: query=(%.12g,%.12g), nearest=(%.12g,%.12g), distance=%.12g normal=(%.12g,%.12g), expected=(%.12g,%.12g), error=%.12g\n", query.x, query.y, measured.m_point.x, measured.m_point.y, measured.m_distance, measured.m_outward.x, measured.m_outward.y, expected.x, expected.y, error);

				PR_EXPECT(error < 2.0e-5f);
			};
			for (auto const displacement : {0.0f, -8.0e-7f, 8.0e-7f})
			{
				// Both signs of distance must retain the same outward circle normal.
				auto const query = (1.0f + displacement) * base;
				auto const length = std::hypot(static_cast<double>(query.x), static_cast<double>(query.y));
				auto const expected = v2{static_cast<float>(query.x / length), static_cast<float>(query.y / length)};
				check_normal(query, v2{-1, 1}, v2{-1, 1}, expected);
			}

			// The same rule must work on an ellipse; this normal is derived directly from its known parametric boundary point.
			auto const boundary = v2{static_cast<float>(1.5 * std::cos(0.7)), static_cast<float>(0.4 * std::sin(0.7))};
			auto const gradient_x = std::cos(0.7) / 1.5;
			auto const gradient_z = std::sin(0.7) / 0.4;
			auto const norm = std::hypot(gradient_x, gradient_z);
			auto const normal = v2{static_cast<float>(gradient_x / norm), static_cast<float>(gradient_z / norm)};
			for (auto const displacement : {-1.0e-6f, 0.0f, 1.0e-6f})
			{
				// Floating-point input rounding shifts the true foot slightly, far below the direction tolerance.
				check_normal(boundary + displacement * normal, v2{-1.5f, 1.5f}, v2{-0.4f, 0.4f}, normal);
			}

			// Corners admit a cone of normals, and a collapsed line admits either transverse direction.
			auto const origin_query = v2{-1.0e-7f, -2.0e-7f};
			check_normal(origin_query, v2{0, 1}, v2{0, 1}, Normalise(origin_query));
			auto const corner_query = v2{1.0f + 2.0e-7f, -4.0e-7f};
			check_normal(corner_query, v2{0, 1}, v2{0, 1}, Normalise(corner_query - v2{1, 0}));
			check_normal(v2{-4.0e-7f, 0.3f}, v2{0, 0}, v2{-1, 1}, v2{-1, 0});
			auto const line_end_query = v2{-4.0e-7f, 1.0f + 8.0e-7f};
			check_normal(line_end_query, v2{0, 0}, v2{-1, 1}, Normalise(line_end_query - v2{0, 1}));
			printf("swing boundary normals: maximum direction error=%.12g across 10 smooth/corner/line cases\n", max_error);
		}

		// Finite differences move the bodies, not the coordinate rows, through independently rotated local joint frames.
		PRUnitTestMethod(MovingFramesDerivativesAndPhysicalDamping, Quick)
		{
			// Nonidentity attachments expose world/parent/child frame confusion and endpoint-sign mistakes.
			auto const pose_a = m4x4{m3x3::Rotation(v3{0.31f, -0.42f, 0.2f}), v4::Origin()};
			auto const local_a = m4x4::Transform(v4::XAxis(), 0.23f, v4::Origin());
			auto const local_b = m4x4::Transform(v4::ZAxis(), -0.18f, v4::Origin());
			auto const frame_a = pose_a * local_a;
			auto max_error = 0.0f;
			for (auto const rotation : {v3::Zero(), v3{1.0e-4f, -2.0e-4f, 3.0e-4f}, v3{0, 1.2f, 0}, v3{0.8f, -0.5f, 0.4f}, v3{-1.2f, 1.0f, 0.6f}})
			{
				// Each configuration stays away from the twist cut and swing antipode during the centered differences.
				auto const relative = m4x4{m3x3::Rotation(rotation), v4::Origin()};
				auto const pose_b = frame_a * relative * InvertOrthonormal(local_b);
				auto const q = quat{relative.rot};
				auto const measured = MeasureSphericalCoordinates(q.xyzw);
				auto const gradients = std::array{measured.m_swing_x_gradient, measured.m_swing_z_gradient, measured.m_twist_gradient};
				for (auto const step : {4.0e-3f, 2.0e-3f, 1.0e-3f})
				{
					// Repeating at three steps distinguishes a derivative discrepancy from FP32 differencing noise.
					for (int endpoint = 0; endpoint != 2; ++endpoint)
					{
						// Perturb each physical endpoint independently along each world-space angular axis.
						for (int component = 0; component != 3; ++component)
						{
							// Reconstruct the joint quaternion from the perturbed body poses.
							auto delta = v3::Zero();
							delta[component] = step;
							auto values = std::array<v3, 2>{};
							for (int side = 0; side != 2; ++side)
							{
								// Left multiplication is a world-space body rotation, unlike the child-frame configuration drift.
								auto const move = m4x4{m3x3::Rotation((side == 0 ? 1.0f : -1.0f) * delta), v4::Origin()};
								auto const moved_a = (endpoint == 0 ? move * pose_a : pose_a) * local_a;
								auto const moved_b = (endpoint == 1 ? move * pose_b : pose_b) * local_b;
								auto const moved_q = quat{Transpose(moved_a.rot) * moved_b.rot};
								values[side] = Values(MeasureSphericalCoordinates(moved_q.xyzw));
							}
							auto const derivative = (values[0] - values[1]) / (2.0f * step);
							for (int coordinate = 0; coordinate != 3; ++coordinate)
							{
								// Coordinate impulses are dual to physical angular velocity in the parent frame.
								auto const expected = (endpoint == 0 ? -1.0f : 1.0f) * (frame_a.rot * gradients[coordinate])[component];
								max_error = std::max(max_error, std::abs(expected - derivative[coordinate]));
								ExpectNear(derivative[coordinate], expected, 4.0e-4f);
							}
						}
					}
				}
			}

			// An independent ordinary-angle oracle makes coordinate-gradient damping visibly different from physical isotropic torque.
			auto const q = ExpMap<quat>(v3{0.4f, -0.25f, 0.2f});
			auto const measured = MeasureSphericalCoordinates(q.xyzw);
			auto const omega = v3{0.3f, 0.7f, -0.2f};
			auto const coordinate_torque = -6.0f * (
				measured.m_swing_x_gradient * Dot(measured.m_swing_x_gradient, omega) +
				measured.m_swing_z_gradient * Dot(measured.m_swing_z_gradient, omega) +
				measured.m_twist_gradient * Dot(measured.m_twist_gradient, omega));
			PR_EXPECT(FEqlAbsolute(coordinate_torque, v3{-1.9185739902f, -7.8353865432f, 1.3320984408f}, 4.0e-6f));
			PR_EXPECT(Length(coordinate_torque + 6.0f * omega) > 3.0f);
			printf("spherical coordinates: maximum moving-frame difference error=%.9g; coordinate damping=(%.9g,%.9g,%.9g), physical=(-1.8,-4.2,1.2)\n", max_error, coordinate_torque.x, coordinate_torque.y, coordinate_torque.z);
		}
	};

	PRUnitTestClass(ConstraintCompilerAlgebraRegressionTests)
	{
		// A rotating frame-A axis contributes to the derivative of its projected anchor separation.
		PRUnitTestMethod(LinearJacobianMatchesFiniteDifference, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			body_a.SetMassProperties(Inertia{1.0f, 1.0f});
			body_b.SetMassProperties(Inertia{1.0f, 1.0f});
			body_b.O2W(m4x4::Translation(2.0f, 0.0f, 0.0f));
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.D6().m_linear[1] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto remap = BodyRemap(body_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			// Rotate A alone: C_y = -2 sin(theta), so its derivative at zero must be -2.
			auto const epsilon = 1.0e-3f;
			body_a.O2W(m4x4::Transform(v4::ZAxis(), epsilon, v4::Origin()));
			auto const positive = CompileConstraints(constraints, remap).m_rows[0].m_position;
			body_a.O2W(m4x4::Transform(v4::ZAxis(), -epsilon, v4::Origin()));
			auto const negative = CompileConstraints(constraints, remap).m_rows[0].m_position;
			body_a.O2W(m4x4::Identity());
			auto const derivative = (positive - negative) / (2.0f * epsilon);
			auto const& row = compiled.m_rows[0];
			ExpectNear(derivative, -2.0f, 2.0e-6f);
			ExpectNear(Dot(row.m_jacobian_a, v8motion{v4::ZAxis(), v4::Zero()}), derivative, 2.0e-6f);

			// Common rigid rotation changes neither coordinate, even though B moves at 2Y.
			auto const common_velocity =
				Dot(row.m_jacobian_a, v8motion{v4::ZAxis(), v4::Zero()}) +
				Dot(row.m_jacobian_b, v8motion{v4::ZAxis(), 2.0f * v4::YAxis()});
			ExpectNear(common_velocity, 0.0f, 2.0e-6f);
		}

		// The moving-axis derivative must be rotated into the articulation link's velocity coordinates.
		PRUnitTestMethod(ArticulationAxisDerivativeUsesLinkFrame, Quick)
		{
			auto const root_pose = m4x4::Transform(v4::XAxis(), 0.6f, v4::Origin());
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia{1.0f, 1.0f}}, root_pose);
			auto articulation = builder.Build();
			auto body = RigidBody{};
			body.SetMassProperties(Inertia{1.0f, 1.0f});
			body.O2W(root_pose * m4x4::Translation(2.0f, 0.0f, 0.0f));
			auto desc = MakeD6(BodyRef::Link(articulation, root), BodyRef::Rigid(body));
			desc.D6().m_linear[1] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto remap = BodyRemap(body_ptrs, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			// Local A rotation contributes -2Z, while B's matching world translation contributes +2.
			auto const& row = compiled.m_rows[0];
			ExpectSpatial(row.m_jacobian_a, -2.0f * v4::ZAxis(), -v4::YAxis());
			auto const common_velocity =
				Dot(row.m_jacobian_a, v8motion{v4::ZAxis(), v4::Zero()}) +
				Dot(row.m_jacobian_b, v8motion{root_pose.rot * v4::ZAxis(), root_pose.rot * (2.0f * v4::YAxis())});
			ExpectNear(common_velocity, 0.0f, 2.0e-6f);
		}
	};

	PRUnitTestClass(ConstraintFoundationTests)
	{
		// Preserve body identity through value relocation and reject simultaneous duplicate identities.
		PRUnitTestMethod(BodyIdentitySurvivesRelocation, Quick)
		{
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			auto const first_id = bodies.front().Id();
			for (int index = 0; index != 64; ++index)
				bodies.emplace_back();

			PR_EXPECT(static_cast<bool>(first_id));
			PR_EXPECT(bodies.front().Id() == first_id);
			for (int lhs = 0; lhs != isize(bodies); ++lhs)
				for (int rhs = lhs + 1; rhs != isize(bodies); ++rhs)
					PR_EXPECT(bodies[lhs].Id() != bodies[rhs].Id());

			auto duplicate = bodies.front();
			auto duplicate_ptrs = std::array<RigidBody*, 2>{&bodies.front(), &duplicate};
			PR_THROWS((void)BodyRemap(duplicate_ptrs), std::exception);
		}

		// Reuse descriptor slots with a new generation and reject every operation through the stale handle.
		PRUnitTestMethod(ConstraintHandlesAreGenerational, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.D6().m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};

			auto const original = constraints.Add(desc);
			PR_EXPECT(constraints.Contains(original));
			PR_EXPECT(constraints.Count() == 1);
			PR_EXPECT(constraints.CapacitySlots() == 1);

			constraints.Remove(original);
			PR_EXPECT(!constraints.Contains(original));
			PR_EXPECT(constraints.Count() == 0);
			PR_THROWS(constraints.Get(original), std::exception);
			PR_THROWS(constraints.Update(original, desc), std::exception);
			PR_THROWS(constraints.SetEnabled(original, false), std::exception);
			PR_THROWS(constraints.Repair(original), std::exception);
			PR_THROWS(constraints.IsBroken(original), std::exception);
			PR_THROWS(constraints.Remove(original), std::exception);

			auto const replacement = constraints.Add(desc);
			PR_EXPECT(replacement.m_index == original.m_index);
			PR_EXPECT(replacement.m_generation != original.m_generation);
			PR_EXPECT(constraints.Contains(replacement));
			PR_EXPECT(constraints.Count() == 1);
		}

		// Merge adjacent dirty slots while retaining separated update ranges and removed tombstones.
		PRUnitTestMethod(DirtyRangesTrackDescriptorUploads, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.D6().m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			auto handles = std::array{
				constraints.Add(desc),
				constraints.Add(desc),
				constraints.Add(desc),
			};

			auto ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 1);
			PR_EXPECT(ranges[0].m_begin == 0 && ranges[0].m_end == 3);

			constraints.ClearDirty();
			desc.D6().m_linear[0].m_target_position = 0.25f;
			constraints.Update(handles[0], desc);
			constraints.Update(handles[2], desc);
			ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 2);
			PR_EXPECT(ranges[0].m_begin == 0 && ranges[0].m_end == 1);
			PR_EXPECT(ranges[1].m_begin == 2 && ranges[1].m_end == 3);

			constraints.Update(handles[1], desc);
			ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 1);
			PR_EXPECT(ranges[0].m_begin == 0 && ranges[0].m_end == 3);

			constraints.ClearDirty();
			constraints.Remove(handles[1]);
			ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 1);
			PR_EXPECT(ranges[0].m_begin == 1 && ranges[0].m_end == 2);
		}

		// Emit active D6 rows in canonical axis order with correct anchor Jacobians and current position errors.
		PRUnitTestMethod(D6CompilerEmitsDeterministicRows, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto const com_a = v4{0.0f, +0.25f, 0.0f, 0.0f};
			auto const com_b = v4{0.0f, -0.50f, 0.0f, 0.0f};
			body_a.SetMassProperties(Inertia::Sphere(1.0f, 1.0f, com_a), com_a);
			body_b.SetMassProperties(Inertia::Sphere(1.0f, 1.0f, com_b), com_b);
			body_a.O2W(m4x4::Identity());
			body_b.O2W(m4x4::Translation(2.0f, 0.0f, 0.0f));

			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.m_frame_a.m_constraint_to_body = m4x4::Translation(0.0f, 1.0f, 0.0f);
			desc.m_frame_b.m_constraint_to_body = m4x4::Transform(v4::XAxis(), 0.25f, v4::Origin());
			desc.D6().m_linear[0] = LockedAxis();
			desc.D6().m_linear[2].m_mode = EConstraintAxisMode::Limited;
			desc.D6().m_linear[2].m_limits = Range<float>{-0.5f, +0.5f};
			desc.D6().m_angular[0].m_mode = EConstraintAxisMode::Driven;
			desc.D6().m_angular[0].m_target_velocity = 1.5f;
			desc.D6().m_angular[0].m_max_force = 12.0f;
			desc.D6().m_angular[1] = LockedAxis();

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks.size() == 1);
			PR_EXPECT(compiled.m_rows.size() == 4);
			auto const& block = compiled.m_blocks[0];
			PR_EXPECT(block.m_source == handle);
			PR_EXPECT(block.m_endpoint_a.m_rigid_index == 0);
			PR_EXPECT(block.m_endpoint_b.m_rigid_index == 1);
			PR_EXPECT(block.m_row_begin == 0);
			PR_EXPECT(block.m_row_count == 4);

			auto const& linear_x = compiled.m_rows[0];
			PR_EXPECT(linear_x.m_kind == EConstraintRowKind::Linear);
			PR_EXPECT(linear_x.m_slot == 0);
			PR_EXPECT(linear_x.m_mode == EConstraintAxisMode::Locked);
			ExpectNear(linear_x.m_position, 2.0f);
			ExpectSpatial(linear_x.m_jacobian_a, v4{0, 0, -0.25f, 0}, v4{-1, 0, 0, 0});
			ExpectSpatial(linear_x.m_jacobian_b, v4{0, 0, -0.50f, 0}, v4{+1, 0, 0, 0});

			auto const& linear_z = compiled.m_rows[1];
			PR_EXPECT(linear_z.m_kind == EConstraintRowKind::Linear);
			PR_EXPECT(linear_z.m_slot == 2);
			PR_EXPECT(linear_z.m_mode == EConstraintAxisMode::Limited);
			ExpectNear(linear_z.m_position, 0.0f);
			ExpectSpatial(linear_z.m_jacobian_a, v4{+0.25f, +2.0f, 0, 0}, v4{0, 0, -1, 0});
			ExpectSpatial(linear_z.m_jacobian_b, v4{+0.50f, 0, 0, 0}, v4{0, 0, +1, 0});

			auto const& angular_x = compiled.m_rows[2];
			PR_EXPECT(angular_x.m_kind == EConstraintRowKind::Angular);
			PR_EXPECT(angular_x.m_slot == 3);
			PR_EXPECT(angular_x.m_mode == EConstraintAxisMode::Driven);
			ExpectNear(angular_x.m_position, 0.25f);
			ExpectNear(angular_x.m_target_velocity, 1.5f);
			ExpectNear(angular_x.m_max_force, 12.0f);
			ExpectSpatial(angular_x.m_jacobian_a, v4{-1, 0, 0, 0}, v4::Zero());
			ExpectSpatial(angular_x.m_jacobian_b, v4{+1, 0, 0, 0}, v4::Zero());

			auto const& angular_y = compiled.m_rows[3];
			PR_EXPECT(angular_y.m_kind == EConstraintRowKind::Angular);
			PR_EXPECT(angular_y.m_slot == 4);
			PR_EXPECT(angular_y.m_mode == EConstraintAxisMode::Locked);
			ExpectNear(angular_y.m_position, 0.0f);
		}

		// Resolve stable endpoints to each frame's current packed indices after reorder and insertion.
		PRUnitTestMethod(EndpointRemapSurvivesBodyOrdering, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto inserted = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.D6().m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);

			auto original_order = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto reordered = std::array<RigidBody*, 3>{&inserted, &body_b, &body_a};
			auto missing_endpoint = std::array<RigidBody*, 2>{&inserted, &body_a};
			auto const original = CompileConstraints(constraints, BodyRemap(original_order));
			auto const remapped = CompileConstraints(constraints, BodyRemap(reordered));

			PR_EXPECT(original.m_blocks[0].m_endpoint_a.m_rigid_index == 0);
			PR_EXPECT(original.m_blocks[0].m_endpoint_b.m_rigid_index == 1);
			PR_EXPECT(remapped.m_blocks[0].m_endpoint_a.m_rigid_index == 2);
			PR_EXPECT(remapped.m_blocks[0].m_endpoint_b.m_rigid_index == 1);
			PR_THROWS(CompileConstraints(constraints, BodyRemap(missing_endpoint)), std::exception);
		}

		// Compile named joint descriptors to the expected D6 row counts without introducing separate solver primitives.
		PRUnitTestMethod(NamedDescriptorsCompileToD6, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto const frame_a = BodyFrame{.m_body = BodyRef::Rigid(body_a)};
			auto const frame_b = BodyFrame{.m_body = BodyRef::Rigid(body_b)};
			auto constraints = ConstraintSet{};
			auto const ball = constraints.Add(BallSocketConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto const hinge = constraints.Add(HingeConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto const slider = constraints.Add(SliderConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto const weld = constraints.Add(WeldConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};

			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks.size() == 4);
			PR_EXPECT(compiled.m_rows.size() == 19);
			PR_EXPECT(compiled.m_blocks[0].m_source == ball && compiled.m_blocks[0].m_row_count == 3);
			PR_EXPECT(compiled.m_blocks[1].m_source == hinge && compiled.m_blocks[1].m_row_count == 5);
			PR_EXPECT(compiled.m_blocks[2].m_source == slider && compiled.m_blocks[2].m_row_count == 5);
			PR_EXPECT(compiled.m_blocks[3].m_source == weld && compiled.m_blocks[3].m_row_count == 6);
		}

		// Omit enabled all-free descriptors so unused persistent declarations create no active solver work.
		PRUnitTestMethod(AllFreeD6ProducesNoActiveBlock, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto constraints = ConstraintSet{};
			constraints.Add(MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b)));
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};

			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks.empty());
			PR_EXPECT(compiled.m_rows.empty());
		}

		// Keep world Jacobians zero while compiling the dynamic endpoint normally.
		PRUnitTestMethod(WorldEndpointHasNoJacobian, Quick)
		{
			auto body = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			desc.D6().m_linear[0] = LockedAxis();
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(1.0f, 0.0f, 0.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};

			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks[0].m_endpoint_a.m_rigid_index == 0);
			PR_EXPECT(compiled.m_blocks[0].m_endpoint_b.IsWorld());
			ExpectSpatial(compiled.m_rows[0].m_jacobian_a, v4::Zero(), v4{-1, 0, 0, 0});
			ExpectSpatial(compiled.m_rows[0].m_jacobian_b, v4::Zero(), v4::Zero());
			ExpectNear(compiled.m_rows[0].m_position, 1.0f);
		}

		// Resolve stable articulation/link identities to their hidden-proxy slots and compile rows in link coordinates.
		PRUnitTestMethod(ArticulationLinkEndpointsCompileInLinkFrame, Quick)
		{
			auto [articulation, child] = MakeConstraintTree(10.0f, constants<float>::tau_by_4);
			auto desc = MakeD6(BodyRef::Link(articulation, child), BodyRef::World());
			desc.m_frame_a.m_constraint_to_body = m4x4::Translation(0.0f, 1.0f, 0.0f);
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(9.0f, 1.0f, 0.0f);
			desc.D6().m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto remap = BodyRemap({}, articulation_ptrs);

			auto const endpoint = remap.ResolveEndpoint(desc.m_frame_a.m_body);
			PR_EXPECT(endpoint.IsLink());
			PR_EXPECT(endpoint.m_articulation_index == 0);
			PR_EXPECT(endpoint.m_link_index == 1);
			PR_EXPECT(endpoint.m_packed_body_index == 1);
			PR_EXPECT(endpoint.m_link == child);

			auto const compiled = CompileConstraints(constraints, remap);
			PR_EXPECT(compiled.m_blocks.size() == 1);
			PR_EXPECT(compiled.m_blocks[0].m_endpoint_a.IsLink());
			PR_EXPECT(compiled.m_blocks[0].m_endpoint_b.IsWorld());
			PR_EXPECT(compiled.m_rows.size() == 1);
			ExpectNear(compiled.m_rows[0].m_position, 0.0f);
			ExpectSpatial(compiled.m_rows[0].m_jacobian_a, v4{0, 0, +1, 0}, v4{-1, 0, 0, 0});
			ExpectSpatial(compiled.m_rows[0].m_jacobian_b, v4::Zero(), v4::Zero());
		}

		// Reject missing, duplicate, stale, and cross-articulation link identities before compiling solver rows.
		PRUnitTestMethod(ArticulationLinkEndpointValidation, Quick)
		{
			auto [articulation_a, child_a] = MakeConstraintTree(0.0f);
			auto [articulation_b, child_b] = MakeConstraintTree(2.0f);
			PR_THROWS(BodyRef::Link(articulation_a, child_b), std::exception);

			auto const endpoint = BodyRef::Link(articulation_a, child_a);
			auto articulation_a_ptrs = std::array<Articulation*, 1>{&articulation_a};
			auto duplicate_ptrs = std::array<Articulation*, 2>{&articulation_a, &articulation_a};
			auto missing_remap = BodyRemap({}, std::span<Articulation* const>{});
			PR_THROWS(missing_remap.ResolveEndpoint(endpoint), std::exception);
			PR_THROWS((void)BodyRemap({}, duplicate_ptrs), std::exception);

			auto remap = BodyRemap({}, articulation_a_ptrs);
			auto stale = endpoint;
			++stale.m_link.m_generation;
			PR_THROWS(remap.ResolveEndpoint(stale), std::exception);
		}

		// Reject invalid endpoint topology, frames, axis modes, and position limits before persistent storage changes.
		PRUnitTestMethod(InvalidDescriptorsAreRejected, Quick)
		{
			auto body = RigidBody{};
			auto constraints = ConstraintSet{};
			auto world_to_world = MakeD6(BodyRef::World(), BodyRef::World());
			auto self = MakeD6(BodyRef::Rigid(body), BodyRef::Rigid(body));
			auto invalid_frame = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_frame.m_frame_a.m_constraint_to_body.x.x = 2.0f;
			auto invalid_limit = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_limit.D6().m_linear[0].m_mode = EConstraintAxisMode::Limited;
			invalid_limit.D6().m_linear[0].m_limits = Range<float>{+1.0f, -1.0f};
			auto invalid_mode = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_mode.D6().m_linear[0].m_mode = static_cast<EConstraintAxisMode>(100);

			PR_THROWS(constraints.Add(world_to_world), std::exception);
			PR_THROWS(constraints.Add(self), std::exception);
			PR_THROWS(constraints.Add(invalid_frame), std::exception);
			PR_THROWS(constraints.Add(invalid_limit), std::exception);
			PR_THROWS(constraints.Add(invalid_mode), std::exception);
			PR_EXPECT(constraints.Count() == 0);
		}
	};
}
#endif
