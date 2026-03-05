//************************************
// Physics-2 Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics-2/integrator/impulse.h"

namespace pr::physics
{
	//HACK todo fix this!
	PRUnitTestClass(ImpulseTests)
	{
		PRUnitTestMethod(ImpulseTests)
		{
			#if 0
			ShapeSphere sph(0.5f);
			ShapeBox    box(v4{maths::inv_root2f, maths::inv_root2f, maths::inv_root2f, 0}, m4x4::Transform(0, 0, maths::tau_by_8f, v4::Origin()));
			RigidBody objA(&box, m4x4::Transform(0, 0, 0, v4{-0.5f, 0, 1, 1}), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			RigidBody objB(&box, m4x4::Transform(0, 0, 0, v4{+0.5f, 0, 1, 1}), Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));

			Contact c(objA, objB);
			ImpulsePair impulse_pair;
			{
				// Normal collision through the CoM for both objects, unequal masses.
				objA.Mass(10);
				objB.Mass(5);
				objA.VelocityWS(v4{0, 0, 0, 0}, v4{+0, 0, 0, 0});
				objB.VelocityWS(v4{0, 0, 0, 0}, v4{-1, 0, 0, 0});
				c.m_axis = v4{1,0,0,0};
				c.m_point = v4{0.5f, 0, 0, 0};
				c.m_mat.m_friction_static =  0.0f; // frictionless
				c.m_mat.m_elasticity_norm = +1.0f; // elastic
				c.m_mat.m_elasticity_tors = +1.0f; // elastic
				c.update(0);
				Dump(c);

				impulse_pair = RestitutionImpulse(c);
				objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
				objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
				auto velA = objA.VelocityWS();
				auto velB = objB.VelocityWS();
				PR_EXPECT(FEql(velA, v8motion{0,0,0, -2.0f/3.0f,0,0}));
				PR_EXPECT(FEql(velB, v8motion{0,0,0, +1.0f/3.0f,0,0}));
			}
			{
				// Normal collision, not through the CoM for both objects, unequal masses.
				objA.Mass(10);
				objB.Mass(5);
				objA.VelocityWS(v4{0, 0, 0, 0}, v4{+0, 0, 0, 0});
				objB.VelocityWS(v4{0, 0, 0, 0}, v4{-1,-1, 0, 0});
				c.m_axis = Normalise(v4{1,1,0,0});
				c.m_point = v4{0.5f, 0, 0, 0};
				c.m_mat.m_friction_static = 1.0f; // sticky
				c.m_mat.m_elasticity_norm = 1.0f; // elastic
				c.m_mat.m_elasticity_tors = 1.0f; // elastic
				c.update(0);
				Dump(c);

				// Tangential component is frictionless so velocity should be reflected
				impulse_pair = RestitutionImpulse(c);
				objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
				objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
				auto velA = objA.VelocityWS();
				auto velB = objB.VelocityWS();
				PR_EXPECT(FEql(velA, v8motion{0,0,-1.14286f, -2.0f/3.0f,-0.285714f, 0}));
				PR_EXPECT(FEql(velB, v8motion{0,0,-1.14286f, +1.0f/3.0f,-0.428572f, 0}));
			}
			{
				// Off-Normal collision, not through the CoM for both objects, equal masses.
				objA.Mass(10);
				objB.Mass(10);
				objA.VelocityWS(v4{0, 0, 0, 0}, v4{+0, 0, 0, 0});
				objB.VelocityWS(v4{0, 0, 0, 0}, v4{-1,-1, 0, 0});
				c.m_axis = Normalise(v4{Cos(constants<float>::tau/16), Sin(constants<float>::tau/16),0,0});
				c.m_point = v4{0.5f, 0, 0, 0};
				c.m_mat.m_friction_static = 0.0f; // frictionless
				c.m_mat.m_elasticity_norm = 1.0f; // elastic
				c.m_mat.m_elasticity_tors = -1.0f; // anti-elastic
				c.update(0);
				Dump(c);

				// Tangential component is frictionless so velocity should be reflected
				impulse_pair = RestitutionImpulse(c);
				objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
				objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
				auto velA = objA.VelocityWS();
				auto velB = objB.VelocityWS();
				//PR_EXPECT(FEql(velA, v8motion{0,0,-0.228571f, -0.733333f,-0.228571f, 0}));
				//PR_EXPECT(FEql(velB, v8motion{0,0,-2.28571f, +0.466666f,-0.542857f, 0}));

				auto dvel = c.m_b2a * objB.VelocityOS() - objA.VelocityOS();
				auto vout = dvel.LinAt(c.m_point);
				auto expected_vout = v4{-1,-1,0,0} - 2 * Dot(v4{-1,-1,0,0}, c.m_axis) * c.m_axis;
				PR_EXPECT(FEql(vout, expected_vout));
			}
			{
				// Tangential impulse. Glancing collision from two rotating, but not translating, objects.
				objA.Mass(10);
				objB.Mass(10);
				objA.VelocityWS(v4{0, 0, -1, 0}, v4{});
				objB.VelocityWS(v4{0, 0, +1, 0}, v4{-tiny<float>, 0, 0, 0});
				c.m_axis = v4{1,0,0,0};
				c.m_point = v4{0.5f, 0, 0, 0};
				c.m_mat.m_friction_static = 1.0f; // sticky
				c.m_mat.m_elasticity_norm = 1.0f; // elastic
				c.m_mat.m_elasticity_tors = 1.0f; // elastic
				c.update(0);
				Dump(c);

				// Collision should produce a near-zero impulse because the collision is
				// friction less, there is a tiny normal component, and the relative
				// velocity at the contact point is nearly zero.
				impulse_pair = RestitutionImpulse(c);
				PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objA, v8force{}, 0.001f));
				PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objB, v8force{}, 0.001f));
			}
			{
				// Change the angular velocity so that the collision point now does have opposing velocity.
				objA.Mass(10);
				objB.Mass(10);
				objA.VelocityWS(v4{0, 0, +1, 0}, v4{});
				objB.VelocityWS(v4{0, 0, +1, 0}, v4{-tiny<float>, 0, 0, 0});
				c.m_axis = v4{1,0,0,0};
				c.m_point = v4{0.5f, 0, 0, 0};
				c.m_mat.m_friction_static = 1.0f; // sticky
				c.m_mat.m_elasticity_norm = 1.0f; // elastic
				c.m_mat.m_elasticity_tors = 1.0f; // elastic
				c.update(0);
				Dump(c);

				// Impulses are still near zero because the normal component of the impulse is zero
				impulse_pair = RestitutionImpulse(c);
				PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objA, v8force{0,0,-2, 0,-4,0}, 0.001f));
				PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objB, v8force{0,0,-2, 0,+4,0}, 0.001f));
			}
			//{
			//	// Same setup, but with elastic tangential should still be zero because
			//	// the relative velocity is nearly zero at the contact point
			//	objA.Mass(10);
			//	objB.Mass(10);
			//	objA.VelocityWS(v4{0, 0, -1, 0}, v4{});
			//	objB.VelocityWS(v4{0, 0, +1, 0}, v4{-tiny<float>, 0, 0, 0});
			//	c.m_axis = v4{1,0,0,0};
			//	c.m_point = v4{0.5f, 0, 0, 0};
			//	c.m_mat.m_friction_static = 1.0f;
			//	c.m_mat.m_elasticity_norm = 1.0f; // elastic
			//	c.m_mat.m_elasticity_tors = -1.0f; // elastic
			//	c.update();
			//	Dump(c);

			//	impulse_pair = RestitutionImpulse(c);
			//	PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objA, v8force{}, 0.001f));
			//	PR_EXPECT(FEqlRelative(impulse_pair.m_os_impulse_objB, v8force{}, 0.001f));
			//}
			//{
			//	// Glancing collision from two rotating, but not translating, objects.
			//	// Collision should reverse the rotations because the collision is tangentially elastic.
			//	RigidBody objA(&sph, m4x4::Transform(0, 0,                0, v4{-0.5f, 0, 1, 1}), Inertia::Sphere(0.5f, 10.0f));
			//	RigidBody objB(&box, m4x4::Transform(0, 0, maths::tau_by_8f, v4{+0.5f, 0, 1, 1}), Inertia::Sphere(0.5f, 10.0f));//Inertia::Box(v4{0.5f,0.5f,0.5f,0}, 10.0f));
			//
			//	// Spin the same way so that the contact point has opposing velocity.
			//	objA.VelocityWS(v4{0, 0, +1, 0}, v4{});
			//	objB.VelocityWS(v4{0, 0, +1, 0}, v4{});
			//
			//	Contact c(objA, objB);
			//	c.m_axis = v4{1,0,0,0};
			//	c.m_point = v4{0.5f, 0, 0, 0};
			//	c.m_velocity = c.m_b2a * objB.VelocityOS() - objA.VelocityOS();
			//	c.m_mat.m_elasticity_norm = 1.0f; // elastic
			//	c.m_mat.m_elasticity_tang = -1.0f; // elastic
			//
			//	Dump(c);
			//
			//	auto impulse_pair = RestitutionImpulse(c);
			//	objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			//	objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
			//
			//	auto velA = objA.VelocityWS();
			//	auto velB = objB.VelocityWS();
			//	PR_EXPECT(FEql(velA, v8motion{0, 0, -0.428571f,  0, -0.285714f, +0}));
			//	PR_EXPECT(FEql(velB, v8motion{0, 0, -0.428571f,  0, +0.285714f, +0}));
			//}
			//{// ObjA and ObjB rotating in opposite directions, should stop after collision impulses
			//	ShapeSphere sph(0.5f);
			//	RigidBody objA(&sph, m4x4::Transform(0, 0, maths::tau_by_8f, v4{-0.5f, 0, 1, 1}), Inertia::Sphere(0.5f, 10.0f));
			//	RigidBody objB(&sph, m4x4::Transform(0, 0, maths::tau_by_8f, v4{+0.5f, 0, 1, 1}), Inertia::Sphere(0.5f, 10.0f));
			//
			//	objA.VelocityWS(v4{0, 0, -1, 0}, v4{});
			//	objB.VelocityWS(v4{0, 0, +1, 0}, v4{});
			//
			//	Contact c(objA, objB);
			//	c.m_axis = v4{1,0,0,0};
			//	c.m_point = v4{0.5f, 0, 0, 0};
			//	c.m_velocity = c.m_b2a * objB.VelocityOS() - objA.VelocityOS();
			//	c.m_mat.m_elasticity_norm = 1.0f;
			//	c.m_mat.m_elasticity_tang = 0.0f;
			//
			//	auto impulse_pair = RestitutionImpulse(c);
			//	objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			//	objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
			//
			//	// 
			//	PR_EXPECT(FEql(objA.O2W().pos, v4{-0.5f, 0, 1, 1}));
			//	PR_EXPECT(FEql(objB.O2W().pos, v4{+0.5f, 0, 1, 1}));
			//	PR_EXPECT(FEql(objA.VelocityWS(), v8motion{}));
			//	PR_EXPECT(FEql(objB.VelocityWS(), v8motion{}));
			//}
			#endif
		}
	};
}
#endif
