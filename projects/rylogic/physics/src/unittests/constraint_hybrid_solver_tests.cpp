//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"
#include "src/constraint/constraint_solver.h"

namespace pr::physics::tests
{
	namespace
	{
		// One articulation and the stable handle of its constrained moving link.
		struct PrismaticTree
		{
			Articulation m_articulation;
			LinkHandle m_link;
		};

		// Return simple finite link mass properties for well-conditioned solver fixtures.
		ArticulationLinkDesc HybridLink(float mass = 1.0f)
		{
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.5f, mass),
			};
		}

		// Build one fixed-root prismatic link with deterministic initial position and velocity.
		PrismaticTree MakePrismaticTree(float position, float velocity, float root_x = 0.0f)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(HybridLink(), m4x4::Translation(root_x, 0.0f, 0.0f));
			auto joint = ArticulationJointDesc::Prismatic(v4::XAxis());
			joint.m_initial_position[0] = position;
			joint.m_initial_velocity[0] = velocity;
			auto const link = builder.AddLink(root, joint, HybridLink());
			return PrismaticTree{
				.m_articulation = builder.Build(),
				.m_link = link,
			};
		}

		// Build one ordinary dynamic sphere with deterministic world-space state.
		RigidBody MakeHybridBody(float velocity_x = 0.0f)
		{
			auto body = RigidBody{};
			body.SetMassProperties(Inertia::Sphere(0.5f, 1.0f));
			body.VelocityWS(v8motion{v4{}, v4{velocity_x, 0.0f, 0.0f, 0}});
			return body;
		}

		// Return one hard linear-X constraint between arbitrary supported endpoints.
		D6ConstraintDesc LinearXConstraint(BodyRef body_a, BodyRef body_b)
		{
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = body_a;
			desc.m_frame_b.m_body = body_b;
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			return desc;
		}

		// Return velocity-only hybrid settings with deterministic fixed work and no retained warm start.
		CpuConstraintSolverConfig HybridVelocityConfig(int iterations = 24)
		{
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = iterations;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			return config;
		}

		// Return an endpoint's current twist in the coordinate frame used by its compiled Jacobian.
		v8motion EndpointVelocity(CompiledConstraintEndpoint const& endpoint, BodyRemap const& remap)
		{
			switch (endpoint.m_type)
			{
				case EConstraintBodyType::World:
				{
					return {};
				}
				case EConstraintBodyType::Rigid:
				{
					return remap.Body(endpoint.m_rigid_index).VelocityWS();
				}
				case EConstraintBodyType::ArticulationLink:
				{
					return remap.ArticulationBody(endpoint.m_articulation_index).LinkVelocity(endpoint.m_link);
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint endpoint type");
				}
			}
		}

		// Return the maximum absolute constrained row velocity in one compiled set.
		float MaximumRowVelocity(CompiledConstraintSet const& constraints, BodyRemap const& remap)
		{
			auto residual = 0.0f;
			for (auto const& block : constraints.m_blocks)
			{
				for (uint32_t local_index = 0; local_index != block.m_row_count; ++local_index)
				{
					auto const& row = constraints.m_rows[block.m_row_begin + local_index];
					auto const velocity =
						Dot(row.m_jacobian_a, EndpointVelocity(block.m_endpoint_a, remap)) +
						Dot(row.m_jacobian_b, EndpointVelocity(block.m_endpoint_b, remap));
					residual = Max(residual, std::abs(velocity));
				}
			}
			return residual;
		}

		// Return all physical velocity state needed to prove deterministic hybrid replay.
		std::array<float, 3> RunDeterministicHybridSolve()
		{
			auto tree = MakePrismaticTree(0.0f, -1.5f);
			auto body_a = MakeHybridBody(+2.0f);
			auto body_b = MakeHybridBody(-0.5f);
			auto constraints = ConstraintSet{};
			constraints.Add(LinearXConstraint(BodyRef::World(), BodyRef::Rigid(body_a)));
			constraints.Add(LinearXConstraint(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b)));
			constraints.Add(LinearXConstraint(BodyRef::Rigid(body_b), BodyRef::Link(tree.m_articulation, tree.m_link)));
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto remap = BodyRemap(body_ptrs, articulation_ptrs);

			auto solver = CpuConstraintSolver{};
			solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, HybridVelocityConfig(32));
			return {
				body_a.VelocityWS().lin.x,
				body_b.VelocityWS().lin.x,
				tree.m_articulation.JointVelocity(tree.m_link)[0],
			};
		}
	}

	PRUnitTestClass(ConstraintHybridSolverTests)
	{
		// Drive one articulation link against world with the exact one-endpoint self-link preconditioner.
		PRUnitTestMethod(LinkToWorldConvergesInOneExactSweep, Quick)
		{
			auto tree = MakePrismaticTree(0.0f, 3.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(LinearXConstraint(BodyRef::World(), BodyRef::Link(tree.m_articulation, tree.m_link)));
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto remap = BodyRemap(std::span<RigidBody* const>{}, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);
			auto config = HybridVelocityConfig(1);
			config.m_coupled_relaxation = 1.0f;

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 1.0f / 60.0f, config);

			PR_EXPECT(MaximumRowVelocity(compiled, remap) < 2.0e-5f);
			PR_EXPECT(std::abs(tree.m_articulation.JointVelocity(tree.m_link)[0]) < 2.0e-5f);
			PR_EXPECT(metrics.m_coupled_velocity_rows == 1);
			PR_EXPECT(metrics.m_coupled_sweeps == 1);
			PR_EXPECT(metrics.m_rejected_coupled_sweeps == 0);
		}

		// Couple an ordinary rigid body to one reduced-coordinate link and converge their relative velocity.
		PRUnitTestMethod(RigidToLinkUsesCompleteTreeResponse, Quick)
		{
			auto tree = MakePrismaticTree(0.0f, -2.0f);
			auto body = MakeHybridBody(+4.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(LinearXConstraint(BodyRef::Link(tree.m_articulation, tree.m_link), BodyRef::Rigid(body)));
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto remap = BodyRemap(body_ptrs, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 1.0f / 60.0f, HybridVelocityConfig());

			PR_EXPECT(MaximumRowVelocity(compiled, remap) < 2.0e-4f);
			PR_EXPECT(metrics.m_projected_velocity_residual < 2.0e-4f);
			PR_EXPECT(metrics.m_rejected_coupled_sweeps == 0);
		}

		// Close one loop between two links of the same tree while retaining the exact batched cross-link ABA response.
		PRUnitTestMethod(SameTreeLoopConverges, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(HybridLink());
			auto joint_a = ArticulationJointDesc::Revolute(v4::ZAxis());
			joint_a.m_initial_velocity[0] = +1.0f;
			auto const link_a = builder.AddLink(root, joint_a, HybridLink());
			auto joint_b = ArticulationJointDesc::Revolute(v4::ZAxis());
			joint_b.m_initial_velocity[0] = -2.0f;
			auto const link_b = builder.AddLink(link_a, joint_b, HybridLink());
			auto articulation = builder.Build();

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(articulation, link_a);
			desc.m_frame_b.m_body = BodyRef::Link(articulation, link_b);
			desc.m_angular[2].m_mode = EConstraintAxisMode::Locked;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto remap = BodyRemap(std::span<RigidBody* const>{}, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 1.0f / 120.0f, HybridVelocityConfig(64));

			PR_EXPECT(MaximumRowVelocity(compiled, remap) < 5.0e-4f);
			PR_EXPECT(metrics.m_projected_velocity_residual < 5.0e-4f);
			PR_EXPECT(metrics.m_rejected_coupled_sweeps == 0);
		}

		// Join two independent articulations and require both complete-tree responses to participate in one block update.
		PRUnitTestMethod(TwoArticulationsConvergeTogether, Quick)
		{
			auto tree_a = MakePrismaticTree(0.0f, +3.0f);
			auto tree_b = MakePrismaticTree(0.0f, -1.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(LinearXConstraint(
				BodyRef::Link(tree_a.m_articulation, tree_a.m_link),
				BodyRef::Link(tree_b.m_articulation, tree_b.m_link)));
			auto articulation_ptrs = std::array<Articulation*, 2>{&tree_a.m_articulation, &tree_b.m_articulation};
			auto remap = BodyRemap(std::span<RigidBody* const>{}, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			auto solver = CpuConstraintSolver{};
			solver.Solve(compiled, remap, 1.0f / 60.0f, HybridVelocityConfig(24));

			PR_EXPECT(MaximumRowVelocity(compiled, remap) < 2.0e-4f);
			PR_EXPECT(FEqlAbsolute(
				tree_a.m_articulation.JointVelocity(tree_a.m_link)[0],
				tree_b.m_articulation.JointVelocity(tree_b.m_link)[0],
				2.0e-4f));
		}

		// Alternate rigid PGS and coupled ABA sweeps through one mixed island until every block is satisfied.
		PRUnitTestMethod(MixedRigidArticulationIslandConverges, Quick)
		{
			auto tree = MakePrismaticTree(0.0f, -1.5f);
			auto body_a = MakeHybridBody(+2.0f);
			auto body_b = MakeHybridBody(-0.5f);
			auto constraints = ConstraintSet{};
			constraints.Add(LinearXConstraint(BodyRef::World(), BodyRef::Rigid(body_a)));
			constraints.Add(LinearXConstraint(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b)));
			constraints.Add(LinearXConstraint(BodyRef::Rigid(body_b), BodyRef::Link(tree.m_articulation, tree.m_link)));
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto remap = BodyRemap(body_ptrs, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 1.0f / 60.0f, HybridVelocityConfig(48));

			PR_EXPECT(MaximumRowVelocity(compiled, remap) < 5.0e-4f);
			PR_EXPECT(metrics.m_projected_velocity_residual < 5.0e-4f);
			PR_EXPECT(metrics.m_coupled_sweeps == 48);
		}

		// Produce bit-identical rigid and generalized velocities for repeated hybrid solves with the same insertion order.
		PRUnitTestMethod(RepeatedHybridSolveIsDeterministic, Quick)
		{
			auto const first = RunDeterministicHybridSolve();
			auto const second = RunDeterministicHybridSolve();
			PR_EXPECT(first == second);
		}

		// Reduce articulation coordinate drift through detached pseudo motion without changing physical generalized velocity.
		PRUnitTestMethod(GeneralizedPositionCorrectionPreservesVelocity, Quick)
		{
			auto tree = MakePrismaticTree(1.0f, 0.75f);
			auto constraints = ConstraintSet{};
			constraints.Add(LinearXConstraint(BodyRef::World(), BodyRef::Link(tree.m_articulation, tree.m_link)));
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto remap = BodyRemap(std::span<RigidBody* const>{}, articulation_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);
			auto const velocity_before = tree.m_articulation.JointVelocity(tree.m_link)[0];

			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 0;
			config.m_position_iterations = 1;
			config.m_position_beta = 0.2f;
			config.m_max_position_speed = 10.0f;
			config.m_warm_start_factor = 0.0f;
			config.m_coupled_relaxation = 1.0f;
			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 0.1f, config);

			PR_EXPECT(FEqlAbsolute(tree.m_articulation.JointPosition(tree.m_link)[0], 0.8f, 2.0e-5f));
			PR_EXPECT(tree.m_articulation.JointVelocity(tree.m_link)[0] == velocity_before);
			PR_EXPECT(metrics.m_initial_position_error == 1.0f);
		}
	};
}
#endif
