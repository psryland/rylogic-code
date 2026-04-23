//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"

namespace pr::physics::tests
{
	void ForceLink_Impulse() {}

	PRUnitTestClass(ImpulseTests)
	{
		// Head-on collision through CoM, unequal masses.
		// ObjA (10kg, stationary) vs ObjB (5kg, moving at -1 m/s along X).
		// Perfectly elastic, frictionless.
		PRUnitTestMethod(HeadOnThroughCoM)
		{
			ShapeBox box(v4{constants<float>::inv_root2, constants<float>::inv_root2, constants<float>::inv_root2, 0},
				m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin()));
			RigidBody objA(&box, m4x4::Translation(-0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			RigidBody objB(&box, m4x4::Translation(+0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 5.0f));

			Material bouncy = {
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 1.0f,
				.m_elasticity_tang = 0.0f,
			};

			objA.VelocityWS(v4{0, 0, 0, 0}, v4{+0, 0, 0, 0});
			objB.VelocityWS(v4{0, 0, 0, 0}, v4{-1, 0, 0, 0});

			RbContact c(objA, objB);
			c.m_axis = v4{1,0,0,0};
			c.m_point = v4{0.5f, 0, 0, 0};
			c.m_mat = bouncy;
			c.Update(0);

			auto impulse_pair = RestitutionImpulse(c);
			objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
			auto velA = objA.VelocityWS();
			auto velB = objB.VelocityWS();

			// 1D elastic: vA' = 2*5*(-1)/(10+5) = -2/3, vB' = ((5-10)*(-1))/(10+5) = 1/3
			PR_EXPECT(FEql(velA, v8motion{0,0,0, -2.0f/3.0f,0,0}));
			PR_EXPECT(FEql(velB, v8motion{0,0,0, +1.0f/3.0f,0,0}));
		}

		// Off-normal collision with sticky (friction + tangential elasticity) material.
		// Contact normal is at 45° between X and Y.
		PRUnitTestMethod(OffNormalSticky)
		{
			ShapeBox box(v4{constants<float>::inv_root2, constants<float>::inv_root2, constants<float>::inv_root2, 0},
				m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin()));
			RigidBody objA(&box, m4x4::Translation(-0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			RigidBody objB(&box, m4x4::Translation(+0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 5.0f));

			Material sticky = {
				.m_friction_static = 1.0f,
				.m_elasticity_norm = 1.0f,
				.m_elasticity_tang = 1.0f,
			};

			objA.VelocityWS(v4{0, 0, 0, 0}, v4{+0, 0, 0, 0});
			objB.VelocityWS(v4{0, 0, 0, 0}, v4{-1,-1, 0, 0});

			RbContact c(objA, objB);
			c.m_axis = Normalise(v4{1,1,0,0});
			c.m_point = v4{0.5f, 0, 0, 0};
			c.m_mat = sticky;
			c.Update(0);

			auto impulse_pair = RestitutionImpulse(c);
			objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
			auto velA = objA.VelocityWS();
			auto velB = objB.VelocityWS();

			// Verify momentum conservation (total momentum before = after)
			// Before: pA = 0, pB = 5*(-1,-1,0) = (-5,-5,0). Total = (-5,-5,0)
			// After:  pA = 10*velA.lin, pB = 5*velB.lin
			auto total_lin_before = v4{-5, -5, 0, 0};
			auto total_lin_after = 10.0f * velA.lin + 5.0f * velB.lin;
			PR_EXPECT(FEqlRelative(total_lin_after, total_lin_before, 0.001f));
		}

		// Off-normal frictionless collision between equal masses.
		// Only the normal component should change; tangential velocity is preserved.
		PRUnitTestMethod(OffNormalFrictionless)
		{
			ShapeBox box(v4{constants<float>::inv_root2, constants<float>::inv_root2, constants<float>::inv_root2, 0},
				m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin()));
			RigidBody objA(&box, m4x4::Translation(-0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			RigidBody objB(&box, m4x4::Translation(+0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));

			Material bouncy = {
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 1.0f,
				.m_elasticity_tang = 0.0f,
			};

			objA.VelocityWS(v4{0, 0, 0, 0}, v4{+0, 0, 0, 0});
			objB.VelocityWS(v4{0, 0, 0, 0}, v4{-1,-1, 0, 0});

			auto axis = Normalise(v4{Cos(constants<float>::tau/16), Sin(constants<float>::tau/16),0,0});
			RbContact c(objA, objB);
			c.m_axis = axis;
			c.m_point = v4{0.5f, 0, 0, 0};
			c.m_mat = bouncy;
			c.Update(0);

			auto impulse_pair = RestitutionImpulse(c);
			objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);

			// The contact-point relative velocity should have its normal component reversed
			// (elastic), and since this is frictionless the tangential component is unchanged.
			auto dvel = c.m_b2a * objB.VelocityOS() - objA.VelocityOS();
			auto vout = dvel.LinAt(c.m_point);
			auto vn_out = Dot(vout, c.m_axis);

			// For elastic collision, the normal component of relative velocity at the
			// contact point should reverse sign.
			auto vin = v4{-1,-1,0,0};
			auto vn_in = Dot(vin, c.m_axis);
			PR_EXPECT(FEqlRelative(vn_out, -vn_in, 0.05f));
		}

		// Glancing collision: rotating objects with nearly-zero normal approach.
		// With frictionless material and tiny normal component, the impulse should be near zero.
		PRUnitTestMethod(GlancingFrictionless)
		{
			ShapeBox box(v4{constants<float>::inv_root2, constants<float>::inv_root2, constants<float>::inv_root2, 0},
				m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin()));
			RigidBody objA(&box, m4x4::Translation(-0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			RigidBody objB(&box, m4x4::Translation(+0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));

			Material bouncy = {
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 1.0f,
				.m_elasticity_tang = 0.0f,
			};

			// Spin both objects so the contact point moves but with near-zero normal approach
			objA.VelocityWS(v4{0, 0, -1, 0}, v4{});
			objB.VelocityWS(v4{0, 0, +1, 0}, v4{-tiny<float>, 0, 0, 0});

			RbContact c(objA, objB);
			c.m_axis = v4{1,0,0,0};
			c.m_point = v4{0.5f, 0, 0, 0};
			c.m_mat = bouncy;
			c.Update(0);

			auto impulse_pair = RestitutionImpulse(c);
			PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objA, v8force{}, 0.001f));
			PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objB, v8force{}, 0.001f));
		}

		// Spinning objects with sticky material — contact point has opposing velocity.
		// With tangential elasticity, this should produce an angular impulse.
		PRUnitTestMethod(SpinningSticky)
		{
			ShapeBox box(v4{constants<float>::inv_root2, constants<float>::inv_root2, constants<float>::inv_root2, 0},
				m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin()));
			RigidBody objA(&box, m4x4::Translation(-0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			RigidBody objB(&box, m4x4::Translation(+0.5f, 0, 1), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));

			Material sticky = {
				.m_friction_static = 1.0f,
				.m_elasticity_norm = 1.0f,
				.m_elasticity_tang = 1.0f,
			};

			// Spin the same way so the contact point has opposing velocity
			objA.VelocityWS(v4{0, 0, +1, 0}, v4{});
			objB.VelocityWS(v4{0, 0, +1, 0}, v4{-tiny<float>, 0, 0, 0});

			RbContact c(objA, objB);
			c.m_axis = v4{1,0,0,0};
			c.m_point = v4{0.5f, 0, 0, 0};
			c.m_mat = sticky;
			c.Update(0);

			// With sticky material and opposing tangential velocity, the impulse should be non-zero
			auto impulse_pair = RestitutionImpulse(c);
			PR_EXPECT(!FEqlRelative(impulse_pair.m_os_impulse_objA, v8force{}, 0.001f));
			PR_EXPECT(!FEqlRelative(impulse_pair.m_os_impulse_objB, v8force{}, 0.001f));

			// Momentum conservation: impulse on A should equal -impulse on B
			auto total_impulse = impulse_pair.m_os_impulse_objA + impulse_pair.m_os_impulse_objB;
			PR_EXPECT(FEqlRelative(total_impulse.lin, v4{}, 0.001f));
		}
	};
}
#endif
