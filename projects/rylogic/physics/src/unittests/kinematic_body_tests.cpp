//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/interop/resolve_runner.h"
#include "src/unittests/shared_gpu.h"
#include "src/compute/resolve_gpu.h"

namespace pr::physics::tests
{
	// A kinematic box moving along +x touching a resting dynamic box on its +x face. Body 0 is kinematic, body 1 is dynamic.
	struct KinematicPushScene
	{
		collision::ShapeBox m_box;
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuResolveContact> m_contacts;
		std::vector<GpuMaterial> m_materials;
		EngineConfig m_config;
		v8motion m_twist;

		KinematicPushScene()
			: m_box(v4{1, 1, 1, 0})
			, m_bodies()
			, m_contacts()
			, m_materials()
			, m_config()
			, m_twist(0, 0, 0, 2, 0, 0)
		{
			// Remove position correction and bounce so the only response is the push from the kinematic body
			m_config.velocity_baumgarte = 0.0f;
			m_config.deep_penetration_baumgarte_min = 0.0f;
			m_config.deep_penetration_baumgarte_max = 0.0f;

			// The pusher keeps its own mass, but the solver must treat it as immovable
			auto pusher = RigidBody{&m_box, m4x4::Identity(), Inertia::Box(m_box.m_radius, 1.0f)};
			pusher.Motion(ERigidBodyMotion::Kinematic, m_twist);
			auto box = RigidBody{&m_box, m4x4::Translation(2, 0, 0), Inertia::Box(m_box.m_radius, 1.0f)};
			m_bodies.push_back(PackDynamics(pusher, 0));
			m_bodies.push_back(PackDynamics(box, 1));

			// The contact is on the shared face, in the pusher's space, with the normal pointing from the pusher to the box
			m_contacts.push_back(GpuResolveContact{
				.axis = v4::XAxis(),
				.contact_point = v4{1, 0, 0, 1},
				.manifold = {v4{1, 0, 0, 1}},
				.b2a = m4x4::Translation(2, 0, 0),
				.body_idx_a = 0,
				.body_idx_b = 1,
				.mat_id_a = 0,
				.mat_id_b = 0,
				.depth = 0.001f,
				.feature = 1,
			});
			m_materials.push_back(GpuMaterial{.friction_static = 0.0f, .elasticity_norm = 0.0f});
		}

		// Check that the box was pushed up to the pusher's speed and that the pusher's twist did not change.
		void CheckPushed(std::span<GpuRigidBody const> bodies) const
		{
			// The kinematic twist is stored in the momentum slots and must be unchanged
			PR_EXPECT(FEql(bodies[0].momentum_ang, m_twist.ang));
			PR_EXPECT(FEql(bodies[0].momentum_lin, m_twist.lin));

			// A perfectly inelastic push leaves the box moving with the pusher. The energy guard must not reduce this push.
			auto const box_velocity = bodies[1].os_com_and_invmass.w * bodies[1].momentum_lin;
			PR_EXPECT(FEqlAbsolute(box_velocity, m_twist.lin, 1.0e-3f));
			PR_EXPECT(FEqlAbsolute(bodies[1].momentum_ang, v4::Zero(), 1.0e-4f));
		}
	};

	PRUnitTestClass(KinematicBodyTests)
	{
		// A non-kinematic body is static exactly when its mass is infinite.
		PRUnitTestMethod(MotionFollowsMass, Quick)
		{
			// Finite mass makes a dynamic body; infinite mass makes a static body
			auto dynamic = RigidBody{nullptr, m4x4::Identity(), Inertia::Sphere(1, 5.0f)};
			auto fixed = RigidBody{nullptr, m4x4::Identity(), Inertia::Infinite()};
			PR_EXPECT(dynamic.Motion() == ERigidBodyMotion::Dynamic);
			PR_EXPECT(fixed.Motion() == ERigidBodyMotion::Static);

			// Switching to Static discards the dynamic mass
			dynamic.Motion(ERigidBodyMotion::Static, v8motion{});
			PR_EXPECT(dynamic.Motion() == ERigidBodyMotion::Static);
			PR_EXPECT(dynamic.InvMass() == 0.0f);
			PR_EXPECT(dynamic.DynamicInertiaInvOS().InvMass() == 0.0f);

			// Restoring finite mass properties makes the body dynamic again
			dynamic.SetMassProperties(Inertia::Sphere(1, 5.0f));
			PR_EXPECT(dynamic.Motion() == ERigidBodyMotion::Dynamic);
		}

		// A kinematic body has infinite solver mass, keeps its own mass properties, and reports its twist as its velocity.
		PRUnitTestMethod(KinematicKeepsMassAndTwist, Quick)
		{
			// Make a dynamic body with some momentum and force, then switch it to kinematic
			auto rb = RigidBody{nullptr, m4x4::Identity(), Inertia::Box(v4{0.5f, 1.0f, 1.5f, 0}, 3.0f)};
			auto const dynamic_inertia = rb.DynamicInertiaInvOS();
			rb.MomentumWS(v8force{0, 0, 1, 1, 2, 3});
			rb.ApplyForceWS(v8force{0, 1, 0, 4, 5, 6});
			rb.GravityWS(v4{0, 0, -9.8f, 0});

			auto const twist = v8motion{0, 0, 0.5f, 1, 0, 0};
			rb.Motion(ERigidBodyMotion::Kinematic, twist);

			// The solver sees an immovable body, but the body's own mass is kept
			PR_EXPECT(rb.Motion() == ERigidBodyMotion::Kinematic);
			PR_EXPECT(rb.Kinematic());
			PR_EXPECT(!AllSet(rb.StateFlags(), ERigidBodyStateFlags::Static));
			PR_EXPECT(rb.InvMass() == 0.0f);
			PR_EXPECT(rb.DynamicInertiaInvOS() == dynamic_inertia);

			// Earlier momentum and forces are gone, the twist is the velocity, and gravity does not act
			PR_EXPECT(FEql(rb.MomentumWS(), v8force{}));
			PR_EXPECT(FEql(rb.ForceWS(), v8force{}));
			PR_EXPECT(FEql(rb.VelocityWS(), twist));
			PR_EXPECT(FEql(rb.KinematicTwistWS(), twist));
			PR_EXPECT(FEql(rb.GravityForceWS(), v8force{}));

			// Setting the velocity of a kinematic body sets its twist
			auto const twist2 = v8motion{0, 0, 0, 0, 2, 0};
			rb.VelocityWS(twist2);
			PR_EXPECT(FEql(rb.KinematicTwistWS(), twist2));

			// New mass properties are stored for later and the body stays kinematic
			rb.SetMassProperties(Inertia::Sphere(1, 2.0f));
			PR_EXPECT(rb.Kinematic());
			PR_EXPECT(rb.InvMass() == 0.0f);
			PR_EXPECT(FEql(rb.DynamicInertiaInvOS().InvMass(), 0.5f));
		}

		// Dynamic -> Kinematic -> Dynamic restores the mass exactly and uses only the caller's release velocity.
		PRUnitTestMethod(ReleaseRestoresMassAndVelocity, Quick)
		{
			// A dynamic body with an offset centre of mass and some momentum
			auto rb = RigidBody{};
			rb.SetMassProperties(Inertia::Box(v4{0.5f, 1.0f, 1.5f, 0}, 3.0f), v4{0.1f, -0.2f, 0.3f, 0});
			auto const dynamic_inertia = rb.DynamicInertiaInvOS();
			auto const com = rb.CentreOfMassOS();
			rb.MomentumWS(v8force{1, 0, 0, 0, 5, 0});

			// Hold it kinematically, then release it with a chosen velocity
			rb.Motion(ERigidBodyMotion::Kinematic, v8motion{0, 0, 1, 0, 0, 0});
			auto const release = v8motion{0, 0.25f, 0, 2, 0, -1};
			rb.Motion(ERigidBodyMotion::Dynamic, release);

			// The mass is restored exactly and the velocity is the release velocity
			PR_EXPECT(rb.Motion() == ERigidBodyMotion::Dynamic);
			PR_EXPECT(rb.InertiaInvOS() == dynamic_inertia);
			PR_EXPECT(All(rb.CentreOfMassOS() == com));
			PR_EXPECT(FEql(rb.VelocityWS(), release));
			PR_EXPECT(FEql(rb.MomentumWS(), rb.InertiaWS() * release));
		}

		// The contact solver reads a kinematic body's twist as its velocity, pushes a dynamic body with it, and never changes the twist.
		PRUnitTestMethod(KinematicPushesDynamicCpu, Quick)
		{
			// Run the shader code compiled as C++
			auto scene = KinematicPushScene{};
			auto bodies = scene.m_bodies;
			auto contacts = scene.m_contacts;
			auto runner = ResolveInteropRunner{scene.m_config};
			runner.Run(ResolveRunnerBuffers{1.0f / 60.0f, bodies, contacts, scene.m_materials});
			scene.CheckPushed(bodies);
		}

		// The GPU contact solver gives the same kinematic push as the shader code compiled as C++.
		PRUnitTestMethod(KinematicPushesDynamicGpu, Extended)
		{
			// Resolve the same scene on the GPU
			auto scene = KinematicPushScene{};
			auto bodies = scene.m_bodies;
			auto& gpu = SharedTestGpu();
			auto resolver = GpuResolver{gpu, scene.m_config, &SharedTestShaderCache()};
			resolver.Resolve(gpu.m_job, 1.0f / 60.0f, scene.m_contacts, bodies, scene.m_materials);
			scene.CheckPushed(bodies);
		}
	};
}
#endif
