//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/articulation_force_aba_gpu.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"
#include "src/compute/coupled_constraint_position_gpu.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/compute/interop/articulation_mobility_runner.h"
#include "src/compute/interop/coupled_constraint_prepare_runner.h"
#include "src/compute/interop/coupled_constraint_position_runner.h"
#include "src/compute/interop/coupled_constraint_velocity_runner.h"
#include "src/constraint/constraint_solver.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// One mixed rigid/articulation island with nontrivial state and three active D6 rows.
		struct CoupledVelocityFixture
		{
			Articulation m_articulation;
			LinkHandle m_child;
			RigidBody m_body;
			ConstraintSet m_constraints;
			ConstraintHandle m_constraint;

			// Build a floating two-link tree before the owning fixture because Articulation has no empty state.
			static std::pair<Articulation, LinkHandle> BuildArticulation()
			{
				auto root_desc = ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.33f, 0.24f, 0.18f, 0}, 2.4f, v4{0.02f, -0.03f, 0.01f, 0}),
				};
				auto child_desc = ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.16f, 0.29f, 0.21f, 0}, 1.3f, v4{-0.01f, 0.02f, 0.03f, 0}),
				};
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(
					root_desc,
					m4x4::Transform(Normalise(v4{1, 2, -1, 0}), 0.27f, v4{0.6f, -0.3f, 0.8f, 1}),
					v8motion{v4{0.11f, -0.07f, 0.16f, 0}, v4{-0.19f, 0.13f, 0.08f, 0}});
				auto joint = ArticulationJointDesc::Revolute(
					v4::ZAxis(),
					m4x4::Transform(v4::YAxis(), 0.21f, v4{0.28f, -0.09f, 0.17f, 1}),
					m4x4::Transform(v4::XAxis(), -0.13f, v4{-0.14f, 0.12f, 0.04f, 1}));
				joint.m_initial_position[0] = 0.23f;
				joint.m_initial_velocity[0] = -0.17f;
				auto const child = builder.AddLink(root, joint, child_desc);
				return {builder.Build(), child};
			}

			// Return the mixed D6 descriptor so multi-island tests can rebuild equivalent independent constraints.
			ConstraintDesc Description()
			{
				auto desc = ConstraintDesc{};
				desc.m_frame_a = BodyFrame{
					.m_body = BodyRef::Link(m_articulation, m_child),
					.m_constraint_to_body = m4x4::Transform(v4::XAxis(), 0.16f, v4{0.11f, -0.07f, 0.14f, 1}),
				};
				desc.m_frame_b = BodyFrame{
					.m_body = BodyRef::Rigid(m_body),
					.m_constraint_to_body = m4x4::Transform(v4::ZAxis(), -0.12f, v4{-0.08f, 0.10f, -0.11f, 1}),
				};
				desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
				desc.D6().m_linear[0].m_max_force = 160.0f;
				desc.D6().m_linear[1].m_mode = EConstraintAxisMode::Limited;
				desc.D6().m_linear[1].m_limits = Range<float>{-0.04f, +0.04f};
				desc.D6().m_linear[1].m_max_force = 180.0f;
				desc.D6().m_angular[2].m_mode = EConstraintAxisMode::Driven;
				desc.D6().m_angular[2].m_target_velocity = 0.29f;
				desc.D6().m_angular[2].m_max_force = 130.0f;
				return desc;
			}

			// Build a deterministic mixed island suitable for CPU/GPU one-sweep parity.
			CoupledVelocityFixture()
				: CoupledVelocityFixture(BuildArticulation())
			{
			}

		private:

			// Finish construction while retaining the generated child handle.
			explicit CoupledVelocityFixture(std::pair<Articulation, LinkHandle> tree)
				: m_articulation(std::move(tree.first))
				, m_child(tree.second)
				, m_body()
				, m_constraints()
				, m_constraint()
			{
				m_body.SetMassProperties(Inertia::Box(v4{0.22f, 0.27f, 0.31f, 0}, 1.7f, v4{0.01f, -0.02f, 0.03f, 0}));
				m_body.O2W(m4x4::Transform(Normalise(v4{-1, 1, 2, 0}), -0.17f, v4{1.0f, 0.25f, -0.2f, 1}));
				m_body.VelocityWS(v8motion{v4{-0.08f, 0.12f, 0.05f, 0}, v4{0.21f, -0.14f, 0.09f, 0}});
				m_constraint = m_constraints.Add(Description());
			}
		};

		// One fixed-root articulation coupled to a dynamic rigid body through a hard positional row.
		struct FixedCoupledPositionFixture
		{
			Articulation m_articulation;
			LinkHandle m_child;
			RigidBody m_body;
			ConstraintSet m_constraints;
			ConstraintHandle m_constraint;

			// Build the fixed two-link tree before the owning fixture because Articulation has no empty state.
			static std::pair<Articulation, LinkHandle> BuildArticulation()
			{
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFixedRoot(
					ArticulationLinkDesc{.m_inertia = Inertia::Box(v4{0.3f, 0.2f, 0.25f, 0}, 2.0f)},
					m4x4::Transform(v4::YAxis(), 0.2f, v4{-0.4f, 0.1f, 0.2f, 1}));
				auto joint = ArticulationJointDesc::Revolute(
					v4::ZAxis(),
					m4x4::Transform(v4::XAxis(), 0.1f, v4{0.3f, 0.0f, 0.0f, 1}),
					m4x4::Transform(v4::YAxis(), -0.1f, v4{-0.2f, 0.0f, 0.0f, 1}));
				joint.m_initial_position[0] = 0.35f;
				joint.m_initial_velocity[0] = -0.12f;
				auto const child = builder.AddLink(
					root,
					joint,
					ArticulationLinkDesc{.m_inertia = Inertia::Box(v4{0.15f, 0.22f, 0.18f, 0}, 1.1f)});
				return {builder.Build(), child};
			}

			// Build deterministic fixed-root position-correction inputs with measurable linear drift.
			FixedCoupledPositionFixture()
				: FixedCoupledPositionFixture(BuildArticulation())
			{
			}

		private:

			// Finish construction while retaining the generated child handle.
			explicit FixedCoupledPositionFixture(std::pair<Articulation, LinkHandle> tree)
				: m_articulation(std::move(tree.first))
				, m_child(tree.second)
				, m_body()
				, m_constraints()
				, m_constraint()
			{
				m_body.SetMassProperties(Inertia::Box(v4{0.2f, 0.24f, 0.28f, 0}, 1.5f));
				m_body.O2W(m4x4::Transform(v4::XAxis(), -0.15f, v4{0.9f, 0.35f, -0.1f, 1}));
				m_body.VelocityWS(v8motion{v4{0.02f, -0.03f, 0.04f, 0}, v4{-0.05f, 0.07f, 0.01f, 0}});

				auto desc = ConstraintDesc{};
				desc.m_frame_a = BodyFrame{
					.m_body = BodyRef::Link(m_articulation, m_child),
					.m_constraint_to_body = m4x4::Translation(v4{0.1f, -0.05f, 0.08f, 1}),
				};
				desc.m_frame_b = BodyFrame{
					.m_body = BodyRef::Rigid(m_body),
					.m_constraint_to_body = m4x4::Translation(v4{-0.06f, 0.04f, -0.03f, 1}),
				};
				desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
				desc.D6().m_linear[0].m_max_force = 120.0f;
				desc.D6().m_angular[2].m_mode = EConstraintAxisMode::Locked;
				desc.D6().m_angular[2].m_max_force = 120.0f;
				m_constraint = m_constraints.Add(desc);
			}
		};

		// Prepared immutable inputs for one coupled velocity replay.
		struct CoupledVelocityInputs
		{
			GpuConstraintUpload m_constraint_upload;
			GpuArticulationUpload m_articulation_upload;
			std::vector<GpuRigidBody> m_bodies;
			std::vector<GpuConstraintBlock> m_blocks;
			std::vector<GpuConstraintRow> m_rows;
			std::vector<GpuCoupledConstraintPreconditioner> m_preconditioners;
			std::vector<GpuConstraintBlock> m_position_blocks;
			std::vector<GpuCoupledConstraintPreconditioner> m_position_preconditioners;
		};

		// Compile arbitrary independent fixtures through the same final-configuration replay kernels as production.
		CoupledVelocityInputs MakeCoupledVelocityInputs(ConstraintSet const& constraints, std::span<RigidBody* const> bodies, std::span<Articulation* const> articulations)
		{
			auto remap = BodyRemap(bodies, articulations);
			auto articulation_upload = PackGpuArticulations(articulations);
			auto constraint_upload = PackGpuConstraints(constraints, remap);
			auto mobility = ArticulationMobilityInteropRunner{};
			mobility.Run(articulation_upload, constraint_upload.m_coupled_articulation_indices);
			auto link_to_world = std::vector<GpuConstraintFrame>{};
			link_to_world.reserve(articulation_upload.m_links.size());
			for (auto const* articulation : articulations)
				for (int link_idx = 0; link_idx != articulation->LinkCount(); ++link_idx)
					link_to_world.push_back(PackGpuTransform(articulation->LinkToWorld(articulation->LinkAt(link_idx))));

			auto packed_bodies = std::vector<GpuRigidBody>{};
			packed_bodies.reserve(bodies.size());
			for (int body_idx = 0; body_idx != isize(bodies); ++body_idx)
				packed_bodies.push_back(PackDynamics(*bodies[body_idx], body_idx));
			auto prepare = CoupledConstraintPrepareInteropRunner{};
			prepare.Run(
				1.0f / 60.0f,
				1.0e-6f,
				constraint_upload,
				packed_bodies,
				link_to_world,
				mobility.Mobilities(),
				mobility.Scratch());
			auto velocity_blocks = std::vector<GpuConstraintBlock>(prepare.Blocks().begin(), prepare.Blocks().end());
			auto velocity_rows = std::vector<GpuConstraintRow>(prepare.Rows().begin(), prepare.Rows().end());
			auto velocity_preconditioners = std::vector<GpuCoupledConstraintPreconditioner>(prepare.Preconditioners().begin(), prepare.Preconditioners().end());
			prepare.PreparePositionPreconditioners();
			return CoupledVelocityInputs{
				.m_constraint_upload = std::move(constraint_upload),
				.m_articulation_upload = std::move(articulation_upload),
				.m_bodies = std::move(packed_bodies),
				.m_blocks = std::move(velocity_blocks),
				.m_rows = std::move(velocity_rows),
				.m_preconditioners = std::move(velocity_preconditioners),
				.m_position_blocks = std::vector<GpuConstraintBlock>(prepare.Blocks().begin(), prepare.Blocks().end()),
				.m_position_preconditioners = std::vector<GpuCoupledConstraintPreconditioner>(prepare.Preconditioners().begin(), prepare.Preconditioners().end()),
			};
		}

		// Compile one canonical mixed fixture through the generalized input builder.
		CoupledVelocityInputs MakeCoupledVelocityInputs(CoupledVelocityFixture& fixture)
		{
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			return MakeCoupledVelocityInputs(fixture.m_constraints, bodies, articulations);
		}

		// Return a one-row unbounded constraint whose exact scalar inverse has a known monotonicity threshold.
		ConstraintDesc ScalarCoupledVelocityDescription(CoupledVelocityFixture& fixture)
		{
			auto desc = fixture.Description();
			desc.D6().m_linear[0].m_max_force = std::numeric_limits<float>::max();
			desc.D6().m_linear[1] = {};
			desc.D6().m_angular[2] = {};
			return desc;
		}

		// Require one scalar to agree under a relative tolerance with a stable absolute floor.
		void ExpectCoupledVelocityNear(float actual, float expected, float tolerance)
		{
			auto const scale = std::max({1.0f, std::abs(actual), std::abs(expected)});
			PR_EXPECT(std::abs(actual - expected) <= tolerance * scale);
		}

		// Pack current articulation link frames in the canonical order consumed by coupled preparation.
		std::vector<GpuConstraintFrame> CoupledLinkFrames(std::span<Articulation* const> articulations)
		{
			auto frames = std::vector<GpuConstraintFrame>{};
			for (auto const* articulation : articulations)
				for (int link_idx = 0; link_idx != articulation->LinkCount(); ++link_idx)
					frames.push_back(PackGpuTransform(articulation->LinkToWorld(articulation->LinkAt(link_idx))));
			return frames;
		}

		// Pack the canonical mixed fixture's articulation link frames.
		std::vector<GpuConstraintFrame> CoupledVelocityLinkFrames(CoupledVelocityFixture const& fixture)
		{
			auto frames = std::vector<GpuConstraintFrame>{};
			frames.reserve(fixture.m_articulation.LinkCount());
			for (int link_idx = 0; link_idx != fixture.m_articulation.LinkCount(); ++link_idx)
				frames.push_back(PackGpuTransform(fixture.m_articulation.LinkToWorld(fixture.m_articulation.LinkAt(link_idx))));
			return frames;
		}

		// Reuse one D3D12 device and command job across coupled velocity hardware tests.
		Gpu& CoupledVelocityTestGpu()
		{
			return SharedTestGpu();
		}

		// Convert stored components without passing through binary32 inertia or velocity reconstruction.
		math::Vec4<double> WorkDouble(float4 value)
		{
			return {double(value.x), double(value.y), double(value.z), 0.0};
		}

		// Evaluate a symmetric stored unit-inertia quadratic in double precision.
		double WorkQuadratic(float4 diagonal, float4 products, math::Vec4<double> value)
		{
			return diagonal.x * value.x * value.x + diagonal.y * value.y * value.y + diagonal.z * value.z * value.z +
				2.0 * (products.x * value.x * value.y + products.y * value.x * value.z + products.z * value.y * value.z);
		}

		// Apply the inverse stored quaternion without normalizing or reconstructing another rounded metric.
		math::Vec4<double> WorkRotateInverse(float4 rotation, math::Vec4<double> value)
		{
			auto const axis = -WorkDouble(rotation);
			auto const twice_cross = 2.0 * Cross(axis, value);
			return value + double(rotation.w) * twice_cross + Cross(axis, twice_cross);
		}

		// Measure the same frozen physical metric from primary momenta/speeds and the actual device-produced transport/screw inputs.
		double PrimaryWorkEnergy(GpuArticulationUpload const& upload, GpuCoupledConstraintVelocityResult const& metric, std::span<GpuRigidBody const> bodies, std::span<float const> speeds)
		{
			auto energy = 0.0;
			for (auto const& body : bodies)
			{
				if (AllSet(body.state_flags, ERigidBodyStateFlags_Kinematic) || body.os_com_and_invmass.w == 0.0f)
					continue;

				auto const angular = WorkDouble(body.momentum_ang);
				auto const local = math::Vec4<double>{Dot3(WorkDouble(body.o2w.x), angular), Dot3(WorkDouble(body.o2w.y), angular), Dot3(WorkDouble(body.o2w.z), angular), 0};
				auto const translation = WorkDouble(body.momentum_lin);
				energy += 0.5 * body.os_com_and_invmass.w * (Dot3(translation, translation) + WorkQuadratic(body.inertia_inv_diagonal, body.inertia_inv_products, local));
			}

			// Reconstruct each parent once, using the GPU's stored frame and screw rather than an independently rounded CPU factorization.
			auto motions = std::vector<math::Vec8<double, Motion>>(upload.m_links.size());
			for (auto const& articulation : upload.m_articulations)
			for (int local_link = 0; local_link != articulation.link_count; ++local_link)
			{
				auto const index = articulation.link_offset + local_link;
				auto const& link = upload.m_links[index];
				auto& motion = motions[index];
				if (local_link == 0)
				{
					switch (articulation.root_type)
					{
						case GpuArticulationRootType_Fixed: { break; }
						case GpuArticulationRootType_Floating:
						{
							for (int component = 0; component != 3; ++component)
							{
								motion.ang[component] = speeds[articulation.velocity_offset + component];
								motion.lin[component] = speeds[articulation.velocity_offset + component + 3];
							}
							break;
						}
						default: { throw std::runtime_error("Unexpected primary-work root type"); }
					}
				}
				else
				{
					auto const& parent = motions[link.parent_link_index];
					auto const& frame = metric.m_articulation_scratch[index].child_to_parent;
					auto const inverse_position = WorkRotateInverse(frame.rotation, -WorkDouble(frame.position));
					motion.ang = WorkRotateInverse(frame.rotation, parent.ang);
					motion.lin = WorkRotateInverse(frame.rotation, parent.lin) + Cross(inverse_position, motion.ang);
					for (int axis = 0; axis != link.dof_count; ++axis)
					{
						auto const& screw = metric.m_articulation_dof_scratch[link.dof_offset + axis].motion_subspace;
						motion.ang += WorkDouble(screw.ang) * double(speeds[link.velocity_offset + axis]);
						motion.lin += WorkDouble(screw.lin) * double(speeds[link.velocity_offset + axis]);
					}
				}
				auto const com_velocity = motion.lin + Cross(motion.ang, WorkDouble(link.inertia_com_and_mass));
				energy += 0.5 * link.inertia_com_and_mass.w * (WorkQuadratic(link.inertia_diagonal, link.inertia_products, motion.ang) + Dot3(com_velocity, com_velocity));
			}
			return energy;
		}
	}

	// Exercise the actual complete-work owner rather than a copied acceptance predicate.
	PRUnitTestClass(CoupledCompleteWorkTests)
	{
		// A newly evaluable finite zero-work candidate must not inherit the previous unscaled allowance-overflow rejection.
		PRUnitTestMethod(ExtremeFiniteWorkCommitsThroughOwner, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto descriptors = ConstraintSet{};
			descriptors.Add(ScalarCoupledVelocityDescription(fixture));
			descriptors.Add(ScalarCoupledVelocityDescription(fixture));
			auto bodies = std::array{&fixture.m_body};
			auto forest = std::array{&fixture.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(descriptors, bodies, forest);
			std::fill(inputs.m_articulation_upload.m_velocities.begin(), inputs.m_articulation_upload.m_velocities.end(), 0.0f);
			inputs.m_bodies[0].momentum_ang = float4(0, 0, 0, 0);
			inputs.m_bodies[0].momentum_lin = float4(std::ldexp(1.0f, 120), 0, 0, 0);
			inputs.m_bodies[0].os_com_and_invmass.w = std::ldexp(1.0f, -100);
			for (int slot = 0; slot != 2; ++slot)
			{
				auto& row = inputs.m_rows[GpuConstraintRowsPerBlock * slot];
				row = {};
				row.jacobian_b_lin.x = std::ldexp(1.0f, 100);
				row.bounds = float4(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0, 0);
				inputs.m_preconditioners[slot].packed[0].x = std::ldexp(1.0f, -100);
			}

			// The controlled prepared rows map P to -P at alpha=1, while squared impulses remain finite.
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1.0f;
			config.constraint_coupled_backtrack_limit = 1;
			config.constraint_warm_start_factor = 0.0f;
			auto& gpu = CoupledVelocityTestGpu();
			auto constraints = GpuConstraintSolver{gpu, config};
			auto force_aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{gpu, force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto job = GpuJob{gpu.m_gpu, "ExtremeFiniteWork", 0xFF00AAFF, 1};
			auto const result = solver.Solve(job, FreshConstraintTestEpoch(1.0f / 60.0f, 0.0f), inputs.m_constraint_upload,
				inputs.m_articulation_upload, inputs.m_bodies, CoupledVelocityLinkFrames(fixture), inputs.m_preconditioners, inputs.m_rows);
			printf("finite-allowance-owner: status=%u attempts=%u failure=%u result=%u accepted=%u before=%.17g after=%.17g merit=%.17g\n",
				result.m_island_states[0].status, result.m_island_states[0].iteration_count, result.m_island_states[0].failure_flags,
				result.m_results[0], result.m_acceptance[0].valid, double(inputs.m_bodies[0].momentum_lin.x), double(result.m_bodies[0].momentum_lin.x),
				double(result.m_island_states[0].merit_change));
			PR_EXPECT(result.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(result.m_island_states[0].iteration_count == 1u);
			PR_EXPECT(result.m_island_states[0].failure_flags == GpuCoupledConstraintFailure_None);
			PR_EXPECT(result.m_results[0] == GpuConstraintStage_Accepted && result.m_acceptance[0].valid != 0u);
			PR_EXPECT(result.m_bodies[0].momentum_lin.x == -inputs.m_bodies[0].momentum_lin.x);
			PR_EXPECT(result.m_articulation_velocities == inputs.m_articulation_upload.m_velocities);
		}

		// Two actual visits in one epoch must descend from the admitted incumbent, not merely remain below owner-free energy.
		PRUnitTestMethod(SameEpochIncumbentRequiresDescent, Extended)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.2f, 1.0f)}, m4x4::Identity(), v8motion{v4::Zero(), v4::XAxis()});
			auto articulation = builder.Build();
			auto descriptors = ConstraintSet{};
			for (int index = 0; index != 3; ++index)
			{
				auto descriptor = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Link(articulation, root)}};
				descriptor.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
				descriptors.Add(descriptor);
			}
			auto forest = std::array{&articulation};
			auto inputs = MakeCoupledVelocityInputs(descriptors, {}, forest);
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 0.9f;
			config.constraint_coupled_backtrack_limit = 4;
			config.constraint_warm_start_factor = 1.0f;
			auto& gpu = CoupledVelocityTestGpu();
			auto constraints = GpuConstraintSolver{gpu, config};
			auto force_aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{gpu, force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto job = GpuJob{gpu.m_gpu, "SameEpochDescent", 0xFF00AAFF, 1};
			auto const epoch = FreshConstraintTestEpoch(1.0f / 60.0f, 1.0f);
			auto const first = solver.Solve(job, epoch, inputs.m_constraint_upload, inputs.m_articulation_upload, inputs.m_bodies, CoupledLinkFrames(forest));

			// Feed back the actual published primary state; the solver retains its actual owned response and accepted ledger.
			inputs.m_articulation_upload.m_velocities = first.m_articulation_velocities;
			auto const second = solver.Solve(job, epoch, inputs.m_constraint_upload, inputs.m_articulation_upload, first.m_bodies, CoupledLinkFrames(forest));
			auto const v1 = double(first.m_articulation_velocities[3]);
			auto const v2 = double(second.m_articulation_velocities[3]);
			printf("same-epoch-descent: first=%.17g second=%.17g first_attempts=%u second_attempts=%u first_lambda=%.17g second_lambda=%.17g first_energy=%.17g second_energy=%.17g\n",
				v1, v2, first.m_island_states[0].iteration_count, second.m_island_states[0].iteration_count,
				double(first.m_accepted_rows[0].impulses[0]), double(second.m_accepted_rows[0].impulses[0]), 0.5 * v1 * v1, 0.5 * v2 * v2);
			PR_EXPECT(first.m_results[0] == GpuConstraintStage_Accepted && second.m_results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(first.m_island_states[0].iteration_count == 2u && second.m_island_states[0].iteration_count == 2u);
			PR_EXPECT(first.m_acceptance[0].valid != 0u && second.m_acceptance[0].valid != 0u);
			PR_EXPECT(std::abs(v1 + 0.35) < 1.0e-6 && std::abs(v2 - 0.1225) < 1.0e-6);
			PR_EXPECT(v2 * v2 < v1 * v1);
		}

		// Transforms, COM offsets, fixed roots and moving prescribed endpoints use the actual primary-state physical metric.
		PRUnitTestMethod(PrimaryMetricAndPrescribedEndpoints, Extended)
		{
			auto run = [&](auto& fixture, int case_index)
			{
				auto bodies = std::array{&fixture.m_body};
				auto forest = std::array{&fixture.m_articulation};
				auto inputs = MakeCoupledVelocityInputs(fixture.m_constraints, bodies, forest);
				if (case_index == 3)
				{
					inputs.m_bodies[0].momentum_ang.x = std::ldexp(1.0f, 100);
					inputs.m_bodies[0].inertia_inv_diagonal.x = std::ldexp(1.0f, 100);
				}
				auto config = EngineConfig{};
				config.constraint_coupled_relaxation = 0.9f;
				config.constraint_warm_start_factor = 0.0f;
				auto& gpu = CoupledVelocityTestGpu();
				auto constraints = GpuConstraintSolver{gpu, config};
				auto force_aba = GpuArticulationForceAba{gpu};
				auto mobility = GpuArticulationMobility{force_aba};
				auto impulse_aba = GpuArticulationImpulseAba{gpu, force_aba, mobility};
				auto prepare = GpuCoupledConstraintPrepare{constraints, config};
				auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
				auto job = GpuJob{gpu.m_gpu, "PrimaryWorkMetric", 0xFF00AAFF, 1};
				auto const result = solver.Solve(job, FreshConstraintTestEpoch(1.0f / 60.0f, 0.0f), inputs.m_constraint_upload,
					inputs.m_articulation_upload, inputs.m_bodies, CoupledLinkFrames(forest));
				auto const before = PrimaryWorkEnergy(inputs.m_articulation_upload, result, inputs.m_bodies, inputs.m_articulation_upload.m_velocities);
				auto const after = PrimaryWorkEnergy(inputs.m_articulation_upload, result, result.m_bodies, result.m_articulation_velocities);
				auto row_work = 0.0;
				auto source_magnitude = 0.0;
				for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
				{
					auto const lambda = double(result.m_accepted_rows[0].impulses[axis]);
					auto const& row = result.m_rows[axis];
					auto prescribed = 0.0;
					if (case_index == 2)
						prescribed = Dot3(WorkDouble(row.jacobian_b_ang), WorkDouble(inputs.m_bodies[0].momentum_ang)) +
							Dot3(WorkDouble(row.jacobian_b_lin), WorkDouble(inputs.m_bodies[0].momentum_lin));

					auto const source = double(row.solve.y) - row.solve.z - prescribed;
					auto const work = -source * lambda + 0.5 * row.solve.w * lambda * lambda;
					row_work += work;
					source_magnitude += std::abs(work);
				}
				auto const work = after - before + row_work;
				auto const scale = before + source_magnitude;
				printf("primary-work: case=%d before=%.17g after=%.17g row=%.17g work=%.17g reported=%.17g status=%u attempts=%u\n",
					case_index, before, after, row_work, work, double(result.m_island_states[0].merit_change),
					result.m_island_states[0].status, result.m_island_states[0].iteration_count);
				PR_EXPECT(result.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
				PR_EXPECT(std::isfinite(work) && work <= 2.0e-5 * scale);
				PR_EXPECT(std::abs(work - result.m_island_states[0].merit_change) <= 2.0e-5 * scale);
			};
			auto floating = CoupledVelocityFixture{};
			run(floating, 0);
			auto fixed = FixedCoupledPositionFixture{};
			run(fixed, 1);
			auto prescribed = CoupledVelocityFixture{};
			prescribed.m_body.Motion(ERigidBodyMotion::Kinematic, v8motion{v4{0.21f, -0.13f, 0.17f, 0}, v4{0.41f, 0.23f, -0.09f, 0}});
			run(prescribed, 2);
			auto stationary = CoupledVelocityFixture{};
			stationary.m_body.Motion(ERigidBodyMotion::Static, v8motion{});
			run(stationary, 3);
		}

		// Repeated child transports in one owner must commit the expected chain response and account for prescribed work.
		PRUnitTestMethod(TenLinkChainPrimaryWork, Extended)
		{
			// Keep each topology independently selectable, including when the other topology fails.
			CheckTenLinkPrimaryWork(false);
		}

		// Sibling transports in one owner must commit the expected star response and account for prescribed work.
		PRUnitTestMethod(TenLinkStarPrimaryWork, Extended)
		{
			// Exercise the same physical and work checks with a different parent graph.
			CheckTenLinkPrimaryWork(true);
		}

		// Compare a ten-link actual-owner sweep with CPU replay and the independent double-precision physical metric.
		void CheckTenLinkPrimaryWork(bool star)
		{
			// Each local child frame cyclically permutes axes; nonzero offsets couple angular and linear transport.
			auto const* label = star ? "star" : "chain";
			auto const axes = std::array{v4::XAxis(), v4::YAxis(), v4::ZAxis()};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				ArticulationLinkDesc{.m_inertia = Inertia::Box(v4{0.33f, 0.24f, 0.18f, 0}, 2.4f, v4{0.02f, -0.03f, 0.01f, 0})},
				m4x4::Transform(Normalise(v4{1, 2, -1, 0}), 0.27f, v4{0.6f, -0.3f, 0.8f, 1}),
				v8motion{v4{0.31f, -0.27f, 0.16f, 0}, v4{-0.39f, 0.23f, 0.18f, 0}});
			auto tip = root;
			for (int index = 1; index != 10; ++index)
			{
				// A 120-degree rotation about (1,1,1) gives nonidentity local rotations at every depth or sibling.
				auto joint = ArticulationJointDesc::Revolute(axes[index % 3],
					m4x4::Transform(Normalise(v4{1, 1, 1, 0}), 2.09439510239319549f, v4{0.28f, -0.09f, 0.17f, 1}),
					m4x4::Translation(v4{-0.14f, 0.12f, 0.04f, 1}));
				joint.m_initial_velocity[0] = 0.04f + 0.015f * index;
				tip = builder.AddLink(star ? root : tip, joint, ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.16f, 0.29f, 0.21f, 0}, 1.0f + 0.1f * index, v4{-0.01f, 0.02f, 0.03f, 0})});
			}
			auto articulation = builder.Build();

			// A moving prescribed endpoint and a driven row require external work in addition to kinetic-energy change.
			auto body = RigidBody{};
			body.SetMassProperties(Inertia::Box(v4{0.22f, 0.27f, 0.31f, 0}, 1.7f));
			body.O2W(m4x4::Transform(v4::YAxis(), -0.17f, v4{1.0f, 0.25f, -0.2f, 1}));
			body.Motion(ERigidBodyMotion::Kinematic, v8motion{v4{0.21f, -0.13f, 0.17f, 0}, v4{0.41f, 0.23f, -0.09f, 0}});
			auto descriptor = ConstraintDesc{
				.m_frame_a = {.m_body = BodyRef::Link(articulation, tip), .m_constraint_to_body = m4x4::Translation(v4{0.11f, -0.07f, 0.14f, 1})},
				.m_frame_b = {.m_body = BodyRef::Rigid(body), .m_constraint_to_body = m4x4::Translation(v4{-0.08f, 0.10f, -0.11f, 1})}};
			descriptor.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			descriptor.D6().m_linear[0].m_max_force = 160.0f;
			descriptor.D6().m_linear[1].m_mode = EConstraintAxisMode::Limited;
			descriptor.D6().m_linear[1].m_limits = Range<float>{-0.04f, 0.04f};
			descriptor.D6().m_linear[1].m_max_force = 180.0f;
			descriptor.D6().m_angular[2].m_mode = EConstraintAxisMode::Driven;
			descriptor.D6().m_angular[2].m_target_velocity = 0.29f;
			descriptor.D6().m_angular[2].m_max_force = 130.0f;
			auto descriptors = ConstraintSet{};
			descriptors.Add(descriptor);
			auto bodies = std::array{&body};
			auto forest = std::array{&articulation};
			auto inputs = MakeCoupledVelocityInputs(descriptors, bodies, forest);
			auto const frames = CoupledLinkFrames(forest);
			auto const config = EngineConfig{};

			// Replay the numerical response on the CPU, then use the actual production preparation, admission and publication PSOs.
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(config.constraint_coupled_relaxation, inputs.m_constraint_upload, inputs.m_articulation_upload, inputs.m_blocks, inputs.m_rows,
				inputs.m_preconditioners, inputs.m_bodies, {.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities}, config.constraint_coupled_backtrack_limit);
			auto& gpu = CoupledVelocityTestGpu();
			auto constraints = GpuConstraintSolver{gpu, config};
			auto force_aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{gpu, force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto job = GpuJob{gpu.m_gpu, "TenLinkPrimaryWork", 0xFF00AAFF, 1};
			auto const result = solver.Solve(job, FreshConstraintTestEpoch(1.0f / 60.0f, config.constraint_warm_start_factor),
				inputs.m_constraint_upload, inputs.m_articulation_upload, inputs.m_bodies, frames);

			// Retain deterministic inputs and complete observed primary states before any assertion can terminate this case.
			auto dump = [&](char const* name, auto const& values)
			{
				// These test-owned files preserve exact binary32 data for a failed production-owner comparison.
				auto output = std::ofstream{};
				output.exceptions(std::ios::failbit | std::ios::badbit);
				output.open(temp_dir() / std::format("{}-{}.bin", label, name), std::ios::binary);
				output.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
			};
			dump("input-articulations", inputs.m_articulation_upload.m_articulations);
			dump("input-links", inputs.m_articulation_upload.m_links);
			dump("input-speeds", inputs.m_articulation_upload.m_velocities);
			dump("input-bodies", inputs.m_bodies);
			dump("input-frames", frames);
			dump("input-rows", inputs.m_rows);
			dump("input-preconditioners", inputs.m_preconditioners);
			dump("actual-speeds", result.m_articulation_velocities);
			dump("actual-bodies", result.m_bodies);
			dump("actual-scratch", result.m_articulation_scratch);
			dump("actual-screws", result.m_articulation_dof_scratch);
			dump("actual-rows", result.m_rows);
			dump("actual-accepted-rows", result.m_accepted_rows);
			dump("actual-islands", result.m_island_states);
			dump("actual-acceptance", result.m_acceptance);
			dump("actual-results", result.m_results);
			dump("expected-speeds", replay.ArticulationVelocities());
			dump("expected-rows", replay.Rows());
			dump("expected-bodies", replay.Bodies());
			dump("expected-scratch", replay.ArticulationScratch());
			PR_EXPECT(inputs.m_articulation_upload.m_links.size() == 10 && result.m_articulation_scratch.size() == 10);
			PR_EXPECT(result.m_island_states.size() == 1 && replay.IslandStates().size() == 1);
			PR_EXPECT(result.m_articulation_velocities.size() == replay.ArticulationVelocities().size());

			// Include both prescribed endpoint work and row source/regularization work in the independent ledger.
			auto const before = PrimaryWorkEnergy(inputs.m_articulation_upload, result, inputs.m_bodies, inputs.m_articulation_upload.m_velocities);
			auto const after = PrimaryWorkEnergy(inputs.m_articulation_upload, result, result.m_bodies, result.m_articulation_velocities);
			auto const expected_after = PrimaryWorkEnergy(inputs.m_articulation_upload, result, replay.Bodies(), replay.ArticulationVelocities());
			auto row_work = 0.0;
			auto expected_row_work = 0.0;
			auto source_magnitude = 0.0;
			for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
			{
				// Kinematic momentum fields store prescribed angular and linear velocity, not dynamic kinetic energy.
				auto const& row = result.m_rows[axis];
				auto const prescribed = Dot3(WorkDouble(row.jacobian_b_ang), WorkDouble(inputs.m_bodies[0].momentum_ang)) +
					Dot3(WorkDouble(row.jacobian_b_lin), WorkDouble(inputs.m_bodies[0].momentum_lin));
				auto const source = double(row.solve.y) - row.solve.z - prescribed;
				auto const lambda = double(result.m_accepted_rows[0].impulses[axis]);
				auto const expected_lambda = double(replay.Rows()[axis].bounds.z);
				auto const work = -source * lambda + 0.5 * row.solve.w * lambda * lambda;
				row_work += work;
				expected_row_work += -source * expected_lambda + 0.5 * row.solve.w * expected_lambda * expected_lambda;
				source_magnitude += std::abs(work);
			}
			auto const work = after - before + row_work;
			auto const expected_work = expected_after - before + expected_row_work;
			auto const scale = before + source_magnitude;
			printf("ten-link-owner: topology=%s before=%.17g after=%.17g expected_after=%.17g row=%.17g expected_row=%.17g work=%.17g expected_work=%.17g reported=%.17g status=%u cpu_status=%u attempts=%u failure=%u\n",
				label, before, after, expected_after, row_work, expected_row_work, work, expected_work, double(result.m_island_states[0].merit_change),
				result.m_island_states[0].status, replay.IslandStates()[0].status, result.m_island_states[0].iteration_count, result.m_island_states[0].failure_flags);

			// A rejection, unchanged state, or acceptance stamp alone cannot satisfy the physical-response checks.
			auto max_change = 0.0f;
			auto expected_change = 0.0f;
			for (size_t index = 0; index != result.m_articulation_velocities.size(); ++index)
			{
				// Compare every generalized coordinate rather than only the constrained tip or root.
				max_change = std::max(max_change, std::abs(result.m_articulation_velocities[index] - inputs.m_articulation_upload.m_velocities[index]));
				expected_change = std::max(expected_change, std::abs(replay.ArticulationVelocities()[index] - inputs.m_articulation_upload.m_velocities[index]));
				ExpectCoupledVelocityNear(result.m_articulation_velocities[index], replay.ArticulationVelocities()[index], 2.0e-3f);
			}
			PR_EXPECT(max_change > 1.0e-3f && expected_change > 1.0e-3f);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(result.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(result.m_results[0] == GpuConstraintStage_Accepted && result.m_acceptance[0].valid != 0u);
			PR_EXPECT(result.m_island_states[0].failure_flags == GpuCoupledConstraintFailure_None);
			PR_EXPECT(std::isfinite(work) && work < 0.0 && expected_work < 0.0);
			PR_EXPECT(std::abs(work - result.m_island_states[0].merit_change) <= 2.0e-5 * scale);
			PR_EXPECT(std::abs(work - expected_work) <= 2.0e-3 * scale);
			PR_EXPECT(std::memcmp(&result.m_bodies[0].momentum_ang, &inputs.m_bodies[0].momentum_ang, sizeof(result.m_bodies[0].momentum_ang)) == 0);
			PR_EXPECT(std::memcmp(&result.m_bodies[0].momentum_lin, &inputs.m_bodies[0].momentum_lin, sizeof(result.m_bodies[0].momentum_lin)) == 0);
			for (size_t index = 0; index != result.m_articulation_scratch.size(); ++index)
			{
				// Published link motion must agree with the expected generalized response throughout the tree.
				PR_EXPECT(FEqlAbsolute(result.m_articulation_scratch[index].link_velocity.ang, replay.ArticulationScratch()[index].link_velocity.ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(result.m_articulation_scratch[index].link_velocity.lin, replay.ArticulationScratch()[index].link_velocity.lin, 2.0e-3f));
			}
		}
	};

	PRUnitTestClass(CoupledConstraintVelocityGpuTests)
	{

		// Match one CPU hybrid pseudo sweep while preserving all physical momentum and articulation velocity state.
		PRUnitTestMethod(PositionReplayMatchesCpuHybridAndPreservesPhysicalState, Quick)
		{
			constexpr auto timestep = 1.0f / 60.0f;
			auto replay_fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(replay_fixture);
			auto const rigid_before = inputs.m_bodies;

			auto baseline_aba = ArticulationImpulseAbaInteropRunner{};
			auto zero_impulses = std::vector<GpuArticulationSpatialVector>(inputs.m_articulation_upload.m_links.size());
			baseline_aba.Run(inputs.m_articulation_upload, inputs.m_constraint_upload.m_coupled_articulation_indices, zero_impulses);
			auto const velocities_before = std::vector<float>(baseline_aba.Velocities().begin(), baseline_aba.Velocities().end());
			auto const accelerations_before = std::vector<float>(baseline_aba.Accelerations().begin(), baseline_aba.Accelerations().end());
			auto const scratch_before = std::vector<GpuArticulationAbaScratch>(baseline_aba.Scratch().begin(), baseline_aba.Scratch().end());

			auto replay = CoupledConstraintPositionInteropRunner{};
			replay.Run(
				timestep,
				0.9f,
				0.2f,
				2.0f,
				1,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_position_blocks,
				inputs.m_rows,
				inputs.m_position_preconditioners,
				inputs.m_bodies,
				4);
			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(replay.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);

			auto cpu_fixture = CoupledVelocityFixture{};
			auto cpu_bodies = std::array<RigidBody*, 1>{&cpu_fixture.m_body};
			auto cpu_articulations = std::array<Articulation*, 1>{&cpu_fixture.m_articulation};
			auto cpu_remap = BodyRemap(cpu_bodies, cpu_articulations);
			auto cpu_solver = CpuConstraintSolver{};
			auto config = CpuConstraintSolverConfig{
				.m_velocity_iterations = 0,
				.m_position_iterations = 1,
				.m_coupled_relaxation = 0.9f,
				.m_position_relaxation = 1.0f,
				.m_position_beta = 0.2f,
				.m_max_position_speed = 2.0f,
				.m_warm_start_factor = 0.0f,
				.m_coupled_backtrack_limit = 4,
			};
			auto const metrics = cpu_solver.Solve(
				CompileConstraints(cpu_fixture.m_constraints, cpu_remap),
				cpu_remap,
				timestep,
				config);
			PR_EXPECT(metrics.m_rejected_coupled_sweeps == 0);

			auto const expected_articulations = PackGpuArticulations(cpu_articulations);
			PR_EXPECT(FEqlAbsolute(replay.Bodies()[0].o2w, cpu_fixture.m_body.O2W(), 3.0e-3f));
			PR_EXPECT(FEqlAbsolute(
				UnpackGpuTransform(replay.Articulations()[0].root_to_world),
				UnpackGpuTransform(expected_articulations.m_articulations[0].root_to_world),
				3.0e-3f));
			PR_EXPECT(replay.Positions().size() == expected_articulations.m_positions.size());
			for (int position_idx = 0; position_idx != isize(replay.Positions()); ++position_idx)
				ExpectCoupledVelocityNear(replay.Positions()[position_idx], expected_articulations.m_positions[position_idx], 3.0e-3f);

			// Position correction changes only coordinates; physical state remains bitwise untouched.
			PR_EXPECT(std::memcmp(&replay.Bodies()[0].momentum_ang, &rigid_before[0].momentum_ang, sizeof(float4)) == 0);
			PR_EXPECT(std::memcmp(&replay.Bodies()[0].momentum_lin, &rigid_before[0].momentum_lin, sizeof(float4)) == 0);
			PR_EXPECT(replay.PhysicalVelocities().size_bytes() == std::span<float const>{velocities_before}.size_bytes());
			PR_EXPECT(std::memcmp(replay.PhysicalVelocities().data(), velocities_before.data(), replay.PhysicalVelocities().size_bytes()) == 0);
			PR_EXPECT(replay.PhysicalAccelerations().size_bytes() == std::span<float const>{accelerations_before}.size_bytes());
			PR_EXPECT(std::memcmp(replay.PhysicalAccelerations().data(), accelerations_before.data(), replay.PhysicalAccelerations().size_bytes()) == 0);
			PR_EXPECT(replay.PhysicalScratch().size_bytes() == std::span<GpuArticulationAbaScratch const>{scratch_before}.size_bytes());
			PR_EXPECT(std::memcmp(replay.PhysicalScratch().data(), scratch_before.data(), replay.PhysicalScratch().size_bytes()) == 0);
		}

		// Reject a non-finite exact-self candidate without changing any rigid or articulation coordinate.
		PRUnitTestMethod(PositionReplayRejectsTransactionally, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			inputs.m_position_preconditioners[0].packed[0].x = std::numeric_limits<float>::quiet_NaN();
			auto const bodies_before = inputs.m_bodies;
			auto const articulations_before = inputs.m_articulation_upload.m_articulations;
			auto const positions_before = inputs.m_articulation_upload.m_positions;

			auto replay = CoupledConstraintPositionInteropRunner{};
			replay.Run(
				1.0f / 60.0f,
				0.9f,
				0.2f,
				2.0f,
				1,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_position_blocks,
				inputs.m_rows,
				inputs.m_position_preconditioners,
				inputs.m_bodies,
				4);

			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(replay.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(replay.Bodies().data(), bodies_before.data(), replay.Bodies().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(replay.Articulations().data(), articulations_before.data(), replay.Articulations().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(replay.Positions().data(), positions_before.data(), replay.Positions().size_bytes()) == 0);
		}



		// Treat accumulated impulses as already applied during later sweeps unless the caller explicitly starts a new substep.
		PRUnitTestMethod(ReplayDoesNotReapplyAccumulatedImpulses, Quick)
		{
			auto first_fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(first_fixture);
			auto first = CoupledConstraintVelocityInteropRunner{};
			first.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities});

			auto second_upload = inputs.m_articulation_upload;
			second_upload.m_velocities.assign(first.ArticulationVelocities().begin(), first.ArticulationVelocities().end());
			auto second = CoupledConstraintVelocityInteropRunner{};
			second.Run(
				0.9f,
				inputs.m_constraint_upload,
				second_upload,
				inputs.m_blocks,
				first.Rows(),
				inputs.m_preconditioners,
				first.Bodies(),
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities});

			auto cpu_fixture = CoupledVelocityFixture{};
			auto cpu_bodies = std::array<RigidBody*, 1>{&cpu_fixture.m_body};
			auto cpu_articulations = std::array<Articulation*, 1>{&cpu_fixture.m_articulation};
			auto cpu_remap = BodyRemap(cpu_bodies, cpu_articulations);
			auto cpu_solver = CpuConstraintSolver{};
			auto config = CpuConstraintSolverConfig{
				.m_velocity_iterations = 2,
				.m_position_iterations = 0,
				.m_coupled_relaxation = 0.9f,
				.m_warm_start_factor = 0.0f,
				.m_coupled_backtrack_limit = 4,
			};
			cpu_solver.Solve(CompileConstraints(cpu_fixture.m_constraints, cpu_remap), cpu_remap, 1.0f / 60.0f, config);

			auto const expected_momentum = cpu_fixture.m_body.MomentumWS();
			PR_EXPECT(FEqlAbsolute(second.Bodies()[0].momentum_ang, expected_momentum.ang, 3.0e-3f));
			PR_EXPECT(FEqlAbsolute(second.Bodies()[0].momentum_lin, expected_momentum.lin, 3.0e-3f));
			auto const cpu_upload = PackGpuArticulations(cpu_articulations);
			for (size_t velocity_idx = 0; velocity_idx != second.ArticulationVelocities().size(); ++velocity_idx)
				ExpectCoupledVelocityNear(second.ArticulationVelocities()[velocity_idx], cpu_upload.m_velocities[velocity_idx], 3.0e-3f);
		}

		// Reject a non-finite block candidate without changing any rigid, generalized, cached-link, or warm-start state.
		PRUnitTestMethod(ReplayRejectsCandidateTransactionally, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			inputs.m_preconditioners[slot_idx].packed[0].x = std::numeric_limits<float>::quiet_NaN();
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities});

			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(replay.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(replay.Bodies().data(), inputs.m_bodies.data(), replay.Bodies().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(replay.ArticulationVelocities().data(), inputs.m_articulation_upload.m_velocities.data(), replay.ArticulationVelocities().size_bytes()) == 0);
			PR_EXPECT(replay.ArticulationScratch()[0].solve_valid != 0);
			PR_EXPECT(replay.Rows()[slot_idx * GpuConstraintRowsPerBlock].bounds.z == 0.0f);
		}


		// Reject a finite candidate whose prospective rigid momentum would overflow before any island state commits.
		PRUnitTestMethod(ReplayRejectsRigidMomentumOverflow, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			auto& block = inputs.m_blocks[slot_idx];
			block.velocity_mask = 1u;
			block.flags |= ConstraintBlockFlags_Active | ConstraintBlockFlags_CoupledPreconditionerValid;
			block.body_idx_a = -1;
			block.body_idx_b = 0;
			auto& row = inputs.m_rows[slot_idx * GpuConstraintRowsPerBlock];
			row.jacobian_a_ang = {};
			row.jacobian_a_lin = {};
			row.jacobian_b_ang = {};
			row.jacobian_b_lin = float4{1.0f, 0.0f, 0.0f, 0.0f};
			row.solve = float4{0.0f, std::numeric_limits<float>::max(), 0.0f, 0.0f};
			row.bounds = float4{-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0f, 0.0f};
			inputs.m_preconditioners[slot_idx] = {};
			inputs.m_preconditioners[slot_idx].packed[0].x = 1.0f;
			inputs.m_bodies[0].momentum_lin.x = 3.0e38f;
			auto const body_before = inputs.m_bodies[0];
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities});

			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(replay.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(replay.Bodies().data(), &body_before, sizeof(body_before)) == 0);
			PR_EXPECT(replay.Rows()[slot_idx * GpuConstraintRowsPerBlock].bounds.z == 0.0f);
		}

		// Match detached generalized replay on real D3D12 while preserving physical state and optional allocation behavior.
		PRUnitTestMethod(PositionHardwareMatchesReplayAndOptionalCost, Extended)
		{
			constexpr auto timestep = 1.0f / 60.0f;
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto replay = CoupledConstraintPositionInteropRunner{};
			replay.Run(
				timestep,
				0.9f,
				0.2f,
				2.0f,
				2,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_position_blocks,
				inputs.m_rows,
				inputs.m_position_preconditioners,
				inputs.m_bodies,
				4);

			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 0.9f;
			config.constraint_position_relaxation = 1.0f;
			config.constraint_position_beta = 0.2f;
			config.constraint_max_position_speed = 2.0f;
			config.push_out_iterations = 2;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto solver = GpuCoupledConstraintPosition{velocity, config};
			auto const hardware = solver.Solve(
				CoupledVelocityTestGpu().m_job,
				timestep,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture));

			PR_EXPECT(hardware.m_bodies.size() == replay.Bodies().size());
			PR_EXPECT(hardware.m_articulations.size() == replay.Articulations().size());
			PR_EXPECT(hardware.m_positions.size() == replay.Positions().size());
			PR_EXPECT(FEqlAbsolute(hardware.m_bodies[0].o2w, replay.Bodies()[0].o2w, 2.0e-3f));
			PR_EXPECT(std::memcmp(&hardware.m_bodies[0].momentum_ang, &inputs.m_bodies[0].momentum_ang, sizeof(float4)) == 0);
			PR_EXPECT(std::memcmp(&hardware.m_bodies[0].momentum_lin, &inputs.m_bodies[0].momentum_lin, sizeof(float4)) == 0);
			PR_EXPECT(FEqlAbsolute(
				UnpackGpuTransform(hardware.m_articulations[0].root_to_world),
				UnpackGpuTransform(replay.Articulations()[0].root_to_world),
				2.0e-3f));
			for (int position_idx = 0; position_idx != isize(hardware.m_positions); ++position_idx)
				ExpectCoupledVelocityNear(hardware.m_positions[position_idx], replay.Positions()[position_idx], 2.0e-3f);
			for (int velocity_idx = 0; velocity_idx != isize(hardware.m_articulation_velocities); ++velocity_idx)
				ExpectCoupledVelocityNear(hardware.m_articulation_velocities[velocity_idx], replay.PhysicalVelocities()[velocity_idx], 2.0e-3f);
			for (int acceleration_idx = 0; acceleration_idx != isize(hardware.m_articulation_accelerations); ++acceleration_idx)
				ExpectCoupledVelocityNear(hardware.m_articulation_accelerations[acceleration_idx], replay.PhysicalAccelerations()[acceleration_idx], 2.0e-3f);
			for (int link_idx = 0; link_idx != isize(hardware.m_articulation_scratch); ++link_idx)
			{
				PR_EXPECT(FEqlAbsolute(hardware.m_articulation_scratch[link_idx].link_velocity.ang, replay.PhysicalScratch()[link_idx].link_velocity.ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(hardware.m_articulation_scratch[link_idx].link_velocity.lin, replay.PhysicalScratch()[link_idx].link_velocity.lin, 2.0e-3f));
			}
			PR_EXPECT(hardware.m_island_states[0].status == replay.IslandStates()[0].status);
			PR_EXPECT(hardware.m_island_states[0].failure_flags == replay.IslandStates()[0].failure_flags);
			PR_EXPECT(FEqlAbsolute(hardware.m_island_states[0].relaxation, replay.IslandStates()[0].relaxation, 1.0e-6f));
			ExpectCoupledVelocityNear(hardware.m_island_states[0].merit_change, replay.IslandStates()[0].merit_change, 2.0e-3f);
			PR_EXPECT(solver.Stats().m_dispatch_count == 2 + config.push_out_iterations * (5 * (config.constraint_coupled_backtrack_limit + 1) + 5));
			PR_EXPECT(solver.Stats().m_logical_bytes != 0);

			auto empty = GpuConstraintUpload{};
			constraints.Upload(CoupledVelocityTestGpu().m_job, empty);
			prepare.Upload(CoupledVelocityTestGpu().m_job, empty);
			velocity.Upload(CoupledVelocityTestGpu().m_job, empty);
			PR_EXPECT(!solver.Prepare(CoupledVelocityTestGpu().m_job, timestep, 0, nullptr, nullptr));
			PR_EXPECT(solver.Stats().m_allocated_feature_bytes == 0);
		}

		// Reject a non-finite D3D12 position candidate without changing either coordinate stream.
		PRUnitTestMethod(PositionHardwareRejectsTransactionally, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			inputs.m_position_preconditioners[0].packed[0].x = std::numeric_limits<float>::quiet_NaN();

			auto config = EngineConfig{};
			config.push_out_iterations = 1;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto solver = GpuCoupledConstraintPosition{velocity, config};
			auto const hardware = solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_position_preconditioners,
				inputs.m_rows);

			PR_EXPECT(hardware.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(hardware.m_island_states[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(hardware.m_bodies.data(), inputs.m_bodies.data(), hardware.m_bodies.size() * sizeof(GpuRigidBody)) == 0);
			PR_EXPECT(std::memcmp(hardware.m_articulations.data(), inputs.m_articulation_upload.m_articulations.data(), hardware.m_articulations.size() * sizeof(GpuArticulation)) == 0);
			PR_EXPECT(std::memcmp(hardware.m_positions.data(), inputs.m_articulation_upload.m_positions.data(), hardware.m_positions.size() * sizeof(float)) == 0);
		}

		// Preserve fixed-root coordinates while matching child-joint pseudo correction between replay and real D3D12.
		PRUnitTestMethod(PositionHardwareMatchesFixedRootReplay, Extended)
		{
			auto fixture = FixedCoupledPositionFixture{};
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(fixture.m_constraints, bodies, articulations);
			auto const link_frames = CoupledLinkFrames(articulations);
			auto replay = CoupledConstraintPositionInteropRunner{};
			replay.Run(
				1.0f / 60.0f,
				0.9f,
				0.2f,
				2.0f,
				2,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_position_blocks,
				inputs.m_rows,
				inputs.m_position_preconditioners,
				inputs.m_bodies,
				4);

			auto config = EngineConfig{};
			config.push_out_iterations = 2;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto solver = GpuCoupledConstraintPosition{velocity, config};
			auto const hardware = solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				link_frames);

			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(hardware.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(std::memcmp(&replay.Articulations()[0].root_to_world, &inputs.m_articulation_upload.m_articulations[0].root_to_world, sizeof(GpuConstraintFrame)) == 0);
			PR_EXPECT(std::memcmp(&hardware.m_articulations[0].root_to_world, &inputs.m_articulation_upload.m_articulations[0].root_to_world, sizeof(GpuConstraintFrame)) == 0);
			PR_EXPECT(!FEqlAbsolute(replay.Positions()[0], inputs.m_articulation_upload.m_positions[0], 1.0e-6f));
			ExpectCoupledVelocityNear(hardware.m_positions[0], replay.Positions()[0], 2.0e-3f);
			PR_EXPECT(FEqlAbsolute(hardware.m_bodies[0].o2w, replay.Bodies()[0].o2w, 2.0e-3f));
			PR_EXPECT(std::memcmp(hardware.m_articulation_velocities.data(), inputs.m_articulation_upload.m_velocities.data(), hardware.m_articulation_velocities.size() * sizeof(float)) == 0);
		}




		// Reject a hardware NaN correction before HLSL clamp can select a finite bound and commit an arbitrary impulse.
		PRUnitTestMethod(HardwareRejectsNonFiniteCorrection, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			inputs.m_preconditioners[slot_idx].packed[0].x = std::numeric_limits<float>::quiet_NaN();

			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 0.9f;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto job = GpuJob{CoupledVelocityTestGpu().m_gpu, "CompleteWorkRejection", 0xFF00AAFF, 1};
			auto const hardware = solver.Solve(
				job,
				physics::tests::FreshConstraintTestEpoch(1.0f / 60.0f, config.constraint_warm_start_factor),
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_preconditioners);

			PR_EXPECT(hardware.m_island_states.size() == 1);
			PR_EXPECT(hardware.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(hardware.m_island_states[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(hardware.m_bodies.data(), inputs.m_bodies.data(), hardware.m_bodies.size() * sizeof(hardware.m_bodies[0])) == 0);
			PR_EXPECT(hardware.m_rows[slot_idx * GpuConstraintRowsPerBlock].bounds.z == 0.0f);
		}
	};
	// Separate private numerical acceptance from published owner records and count only actual attempts.
	PRUnitTestClass(EngineMigrationCoupledDiagnosticTests)
	{
		// Match one production CPU hybrid sweep across rigid momentum, generalized velocity, and cached link velocity.
		PRUnitTestMethod(ReplayMatchesCpuHybridSweep, Quick)
		{
			auto replay_fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(replay_fixture);
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities},
				0);

			auto cpu_fixture = CoupledVelocityFixture{};
			auto cpu_bodies = std::array<RigidBody*, 1>{&cpu_fixture.m_body};
			auto cpu_articulations = std::array<Articulation*, 1>{&cpu_fixture.m_articulation};
			auto cpu_remap = BodyRemap(cpu_bodies, cpu_articulations);
			auto cpu_solver = CpuConstraintSolver{};
			auto config = CpuConstraintSolverConfig{
				.m_velocity_iterations = 1,
				.m_position_iterations = 0,
				.m_coupled_relaxation = 0.9f,
				.m_warm_start_factor = 0.0f,
				.m_coupled_backtrack_limit = 0,
			};
			auto const metrics = cpu_solver.Solve(
				CompileConstraints(cpu_fixture.m_constraints, cpu_remap),
				cpu_remap,
				1.0f / 60.0f,
				config);
			PR_EXPECT(metrics.m_rejected_coupled_sweeps == 0);
			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(replay.IslandStates()[0].iteration_count == 1u);
			PR_EXPECT(replay.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);

			auto const expected_momentum = cpu_fixture.m_body.MomentumWS();
			PR_EXPECT(FEqlAbsolute(replay.Bodies()[0].momentum_ang, expected_momentum.ang, 2.0e-3f));
			PR_EXPECT(FEqlAbsolute(replay.Bodies()[0].momentum_lin, expected_momentum.lin, 2.0e-3f));
			auto const cpu_upload = PackGpuArticulations(cpu_articulations);
			PR_EXPECT(replay.ArticulationVelocities().size() == cpu_upload.m_velocities.size());
			for (int velocity_idx = 0; velocity_idx != isize(cpu_upload.m_velocities); ++velocity_idx)
				ExpectCoupledVelocityNear(replay.ArticulationVelocities()[velocity_idx], cpu_upload.m_velocities[velocity_idx], 2.0e-3f);
			for (int link_idx = 0; link_idx != cpu_fixture.m_articulation.LinkCount(); ++link_idx)
			{
				auto const expected = cpu_fixture.m_articulation.LinkVelocity(cpu_fixture.m_articulation.LinkAt(link_idx));
				auto const actual = replay.ArticulationScratch()[link_idx].link_velocity;
				PR_EXPECT(FEqlAbsolute(actual.ang, expected.ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual.lin, expected.lin, 2.0e-3f));
			}

			auto const row_offset = inputs.m_constraint_upload.m_coupled_island_blocks[0] * GpuConstraintRowsPerBlock;
			PR_EXPECT(std::abs(replay.Rows()[row_offset + 0].bounds.z) > 1.0e-5f);
		}

		// Halve an over-large finite step until merit decreases, and reject the same step when its retry budget is exhausted.
		PRUnitTestMethod(ReplayBacktracksAndRejectsBoundedly, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto constraints = ConstraintSet{};
			auto const constraint = constraints.Add(ScalarCoupledVelocityDescription(fixture));
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints, bodies, articulations);
			auto const slot_idx = constraint.m_index;

			// Four times the exact scalar inverse overshoots at 0.9 relaxation but becomes monotone after one halving.
			inputs.m_preconditioners[slot_idx].packed[0].x *= 4.0f;
			auto accepted = CoupledConstraintVelocityInteropRunner{};
			accepted.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities},
				1);

			PR_EXPECT(accepted.IslandStates().size() == 1);
			PR_EXPECT(accepted.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(accepted.IslandStates()[0].iteration_count == 2u);
			PR_EXPECT(accepted.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);
			PR_EXPECT(FEqlAbsolute(accepted.IslandStates()[0].relaxation, 0.45f, 1.0e-6f));
			PR_EXPECT(accepted.IslandStates()[0].merit_change < 0.0f);
			PR_EXPECT(std::abs(accepted.Rows()[slot_idx * GpuConstraintRowsPerBlock].bounds.z) > 1.0e-5f);

			// With no retry available, the identical positive-merit candidate must leave every authoritative stream unchanged.
			auto rejected = CoupledConstraintVelocityInteropRunner{};
			rejected.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities},
				0);

			PR_EXPECT(rejected.IslandStates().size() == 1);
			PR_EXPECT(rejected.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(rejected.IslandStates()[0].iteration_count == 1u);
			PR_EXPECT(AllSet(rejected.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_Merit));
			PR_EXPECT(FEqlAbsolute(rejected.IslandStates()[0].relaxation, 0.9f, 1.0e-6f));
			PR_EXPECT(rejected.IslandStates()[0].merit_change > 0.0f);
			PR_EXPECT(std::memcmp(rejected.Bodies().data(), inputs.m_bodies.data(), rejected.Bodies().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(rejected.ArticulationVelocities().data(), inputs.m_articulation_upload.m_velocities.data(), rejected.ArticulationVelocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(rejected.Rows().data(), inputs.m_rows.data(), rejected.Rows().size_bytes()) == 0);
		}

		// Let independent islands accept at different attempts without reevaluating or losing the first island's detached response.
		PRUnitTestMethod(ReplayAcceptsIndependentIslandsAtDifferentAttempts, Quick)
		{
			auto first = CoupledVelocityFixture{};
			auto second = CoupledVelocityFixture{};
			auto constraints = ConstraintSet{};
			auto const first_constraint = constraints.Add(ScalarCoupledVelocityDescription(first));
			auto const second_constraint = constraints.Add(ScalarCoupledVelocityDescription(second));
			auto bodies = std::array<RigidBody*, 2>{&first.m_body, &second.m_body};
			auto articulations = std::array<Articulation*, 2>{&first.m_articulation, &second.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints, bodies, articulations);
			inputs.m_preconditioners[second_constraint.m_index].packed[0].x *= 4.0f;

			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities},
				1);

			auto const first_island = inputs.m_constraint_upload.m_coupled_block_topology[first_constraint.m_index].island_idx;
			auto const second_island = inputs.m_constraint_upload.m_coupled_block_topology[second_constraint.m_index].island_idx;
			PR_EXPECT(replay.IslandStates().size() == 2);
			PR_EXPECT(replay.IslandStates()[first_island].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(replay.IslandStates()[second_island].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(replay.IslandStates()[first_island].iteration_count == 1u);
			PR_EXPECT(replay.IslandStates()[second_island].iteration_count == 2u);
			PR_EXPECT(FEqlAbsolute(replay.IslandStates()[first_island].relaxation, 0.9f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(replay.IslandStates()[second_island].relaxation, 0.45f, 1.0e-6f));
			PR_EXPECT(replay.IslandStates()[first_island].merit_change <= 0.0f);
			PR_EXPECT(replay.IslandStates()[second_island].merit_change < 0.0f);
		}

		// Treat a dormant runtime limit block as a successful no-op rather than rejecting unrelated rows in its island.
		PRUnitTestMethod(ReplayAcceptsDormantLimitBlock, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			inputs.m_blocks[slot_idx].velocity_mask = 0;
			inputs.m_blocks[slot_idx].flags = SetFlag(inputs.m_blocks[slot_idx].flags, ConstraintBlockFlags_Active, false);
			inputs.m_blocks[slot_idx].flags = SetFlag(inputs.m_blocks[slot_idx].flags, ConstraintBlockFlags_CoupledPreconditionerValid, false);
			inputs.m_bodies[0].state_flags |= ERigidBodyStateFlags_Sleeping;
			inputs.m_bodies[0].sleep.timer_s = 2.0f;
			inputs.m_bodies[0].sleep.island_id = 7;
			auto const sleeping_body = inputs.m_bodies[0];
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities});

			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(replay.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);
			PR_EXPECT(std::memcmp(replay.Bodies().data(), &sleeping_body, sizeof(sleeping_body)) == 0);
			PR_EXPECT(std::memcmp(replay.ArticulationVelocities().data(), inputs.m_articulation_upload.m_velocities.data(), replay.ArticulationVelocities().size_bytes()) == 0);
		}

		// Commit a valid island while independently rejecting a non-finite island on replay and real D3D12.
		PRUnitTestMethod(PositionHardwareRejectsOnlyInvalidIsland, Extended)
		{
			auto first = CoupledVelocityFixture{};
			auto second = CoupledVelocityFixture{};
			auto constraints_set = ConstraintSet{};
			auto const first_constraint = constraints_set.Add(first.Description());
			auto const second_constraint = constraints_set.Add(second.Description());
			auto bodies = std::array<RigidBody*, 2>{&first.m_body, &second.m_body};
			auto articulations = std::array<Articulation*, 2>{&first.m_articulation, &second.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints_set, bodies, articulations);
			inputs.m_position_preconditioners[second_constraint.m_index].packed[0].x = std::numeric_limits<float>::quiet_NaN();
			auto const link_frames = CoupledLinkFrames(articulations);

			auto replay = CoupledConstraintPositionInteropRunner{};
			replay.Run(
				1.0f / 60.0f,
				0.9f,
				0.2f,
				2.0f,
				1,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_position_blocks,
				inputs.m_rows,
				inputs.m_position_preconditioners,
				inputs.m_bodies,
				4);
			auto const first_island = inputs.m_constraint_upload.m_coupled_block_topology[first_constraint.m_index].island_idx;
			auto const second_island = inputs.m_constraint_upload.m_coupled_block_topology[second_constraint.m_index].island_idx;
			PR_EXPECT(replay.IslandStates()[first_island].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(replay.IslandStates()[second_island].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(replay.IslandStates()[first_island].iteration_count == 1u);
			PR_EXPECT(replay.IslandStates()[second_island].iteration_count == 1u);
			PR_EXPECT(std::memcmp(&replay.Bodies()[1].o2w, &inputs.m_bodies[1].o2w, sizeof(float4x4)) == 0);
			PR_EXPECT(std::memcmp(&replay.Articulations()[1].root_to_world, &inputs.m_articulation_upload.m_articulations[1].root_to_world, sizeof(GpuConstraintFrame)) == 0);
			PR_EXPECT(std::memcmp(&replay.Bodies()[0].o2w, &inputs.m_bodies[0].o2w, sizeof(float4x4)) != 0);

			auto config = EngineConfig{};
			config.push_out_iterations = 1;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto solver = GpuCoupledConstraintPosition{velocity, config};
			auto const hardware = solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				link_frames,
				inputs.m_position_preconditioners,
				inputs.m_rows);

			PR_EXPECT(hardware.m_island_states[first_island].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(hardware.m_island_states[second_island].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(hardware.m_island_states[first_island].iteration_count == 1u);
			PR_EXPECT(hardware.m_island_states[second_island].iteration_count == 1u);
			PR_EXPECT(FEqlAbsolute(hardware.m_bodies[0].o2w, replay.Bodies()[0].o2w, 2.0e-3f));
			PR_EXPECT(std::memcmp(&hardware.m_bodies[1].o2w, &inputs.m_bodies[1].o2w, sizeof(float4x4)) == 0);
			PR_EXPECT(FEqlAbsolute(
				UnpackGpuTransform(hardware.m_articulations[0].root_to_world),
				UnpackGpuTransform(replay.Articulations()[0].root_to_world),
				2.0e-3f));
			PR_EXPECT(std::memcmp(&hardware.m_articulations[1].root_to_world, &inputs.m_articulation_upload.m_articulations[1].root_to_world, sizeof(GpuConstraintFrame)) == 0);
		}

		// Match the deterministic replay on real D3D12 in one submission and release all velocity-lane storage when idle.
		PRUnitTestMethod(HardwareMatchesReplayAndOptionalCost, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities});

			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 0.9f;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto job = GpuJob{CoupledVelocityTestGpu().m_gpu, "CompleteWorkParity", 0xFF00AAFF, 1};
			auto const hardware = solver.Solve(
				job,
				physics::tests::FreshConstraintTestEpoch(1.0f / 60.0f, config.constraint_warm_start_factor),
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture));

			PR_EXPECT(hardware.m_bodies.size() == replay.Bodies().size());
			PR_EXPECT(hardware.m_rows.size() == replay.Rows().size());
			PR_EXPECT(hardware.m_articulation_velocities.size() == replay.ArticulationVelocities().size());
			PR_EXPECT(hardware.m_articulation_accelerations.size() == replay.ArticulationAccelerations().size());
			PR_EXPECT(hardware.m_articulation_scratch.size() == replay.ArticulationScratch().size());
			PR_EXPECT(hardware.m_island_states.size() == replay.IslandStates().size());
			for (size_t body_idx = 0; body_idx != hardware.m_bodies.size(); ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(hardware.m_bodies[body_idx].momentum_ang, replay.Bodies()[body_idx].momentum_ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(hardware.m_bodies[body_idx].momentum_lin, replay.Bodies()[body_idx].momentum_lin, 2.0e-3f));
			}
			for (size_t velocity_idx = 0; velocity_idx != hardware.m_articulation_velocities.size(); ++velocity_idx)
				ExpectCoupledVelocityNear(hardware.m_articulation_velocities[velocity_idx], replay.ArticulationVelocities()[velocity_idx], 2.0e-3f);
			for (size_t acceleration_idx = 0; acceleration_idx != hardware.m_articulation_accelerations.size(); ++acceleration_idx)
				ExpectCoupledVelocityNear(hardware.m_articulation_accelerations[acceleration_idx], replay.ArticulationAccelerations()[acceleration_idx], 2.0e-3f);
			for (size_t link_idx = 0; link_idx != hardware.m_articulation_scratch.size(); ++link_idx)
			{
				PR_EXPECT(FEqlAbsolute(hardware.m_articulation_scratch[link_idx].link_velocity.ang, replay.ArticulationScratch()[link_idx].link_velocity.ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(hardware.m_articulation_scratch[link_idx].link_velocity.lin, replay.ArticulationScratch()[link_idx].link_velocity.lin, 2.0e-3f));
			}
			for (size_t row_idx = 0; row_idx != hardware.m_rows.size(); ++row_idx)
				PR_EXPECT(FEqlAbsolute(hardware.m_accepted_rows[row_idx / GpuConstraintRowsPerBlock].impulses[row_idx % GpuConstraintRowsPerBlock], replay.Rows()[row_idx].bounds.z, 2.0e-3f));

			// Numerical replay accepts a private result; only the hardware owner publishes its cache and commit stamp.
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Accepted);
			PR_EXPECT(hardware.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(hardware.m_results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(hardware.m_acceptance[0].valid != 0u);
			PR_EXPECT(hardware.m_island_states[0].iteration_count == replay.IslandStates()[0].iteration_count);
			PR_EXPECT(hardware.m_island_states[0].failure_flags == replay.IslandStates()[0].failure_flags);
			PR_EXPECT(FEqlAbsolute(hardware.m_island_states[0].relaxation, replay.IslandStates()[0].relaxation, 1.0e-6f));
			ExpectCoupledVelocityNear(hardware.m_island_states[0].merit_change, replay.IslandStates()[0].merit_change, 2.0e-3f);
			// Count the authoritative stage/refresh/work loop; ABA and preparation retain separate counters.
			PR_EXPECT(solver.Stats().m_dispatch_count == 15 * (config.constraint_coupled_backtrack_limit + 1) + 29);
			PR_EXPECT(solver.Stats().m_logical_bytes != 0);

			auto empty = GpuConstraintUpload{};
			PR_EXPECT(!solver.Upload(CoupledVelocityTestGpu().m_job, empty));
			PR_EXPECT(solver.Stats().m_allocated_feature_bytes == 0);
		}

		// Match successful merit backtracking and bounded transactional exhaustion on real D3D12.
		PRUnitTestMethod(HardwareBacktracksAndRejectsBoundedly, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto constraints = ConstraintSet{};
			auto const constraint = constraints.Add(ScalarCoupledVelocityDescription(fixture));
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints, bodies, articulations);
			auto const slot_idx = constraint.m_index;
			inputs.m_preconditioners[slot_idx].packed[0].x *= 4.0f;

			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				{.m_bodies = inputs.m_bodies, .m_velocities = inputs.m_articulation_upload.m_velocities},
				1);

			// The one-retry hardware lane must commit the same half-relaxation candidate and exact merit as deterministic replay.
			auto accepted_config = EngineConfig{};
			accepted_config.constraint_coupled_relaxation = 0.9f;
			accepted_config.constraint_coupled_backtrack_limit = 1;
			auto accepted_constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), accepted_config};
			auto accepted_force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto accepted_mobility = GpuArticulationMobility{accepted_force_aba};
			auto accepted_impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), accepted_force_aba, accepted_mobility};
			auto accepted_prepare = GpuCoupledConstraintPrepare{accepted_constraints, accepted_config};
			auto accepted_solver = GpuCoupledConstraintVelocity{accepted_prepare, accepted_impulse_aba, accepted_config};
			auto accepted_job = GpuJob{CoupledVelocityTestGpu().m_gpu, "CompleteWorkBacktrack", 0xFF00AAFF, 1};
			auto const accepted = accepted_solver.Solve(
				accepted_job,
				physics::tests::FreshConstraintTestEpoch(1.0f / 60.0f, accepted_config.constraint_warm_start_factor),
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_preconditioners);

			PR_EXPECT(accepted.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(accepted.m_island_states[0].iteration_count == 2u);
			PR_EXPECT(accepted.m_results[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(accepted.m_acceptance[0].valid != 0u);
			PR_EXPECT(FEqlAbsolute(accepted.m_island_states[0].relaxation, 0.45f, 1.0e-6f));
			ExpectCoupledVelocityNear(accepted.m_island_states[0].merit_change, replay.IslandStates()[0].merit_change, 2.0e-3f);
			PR_EXPECT(FEqlAbsolute(accepted.m_bodies[0].momentum_ang, replay.Bodies()[0].momentum_ang, 2.0e-3f));
			PR_EXPECT(FEqlAbsolute(accepted.m_bodies[0].momentum_lin, replay.Bodies()[0].momentum_lin, 2.0e-3f));
			PR_EXPECT(accepted_solver.Stats().m_dispatch_count == 59);

			// A zero-retry hardware lane must report merit exhaustion and preserve all authoritative state.
			auto rejected_config = EngineConfig{};
			rejected_config.constraint_coupled_relaxation = 0.9f;
			rejected_config.constraint_coupled_backtrack_limit = 0;
			auto rejected_constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), rejected_config};
			auto rejected_force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto rejected_mobility = GpuArticulationMobility{rejected_force_aba};
			auto rejected_impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), rejected_force_aba, rejected_mobility};
			auto rejected_prepare = GpuCoupledConstraintPrepare{rejected_constraints, rejected_config};
			auto rejected_solver = GpuCoupledConstraintVelocity{rejected_prepare, rejected_impulse_aba, rejected_config};
			auto rejected_job = GpuJob{CoupledVelocityTestGpu().m_gpu, "CompleteWorkExhaustion", 0xFF00AAFF, 1};
			auto const rejected = rejected_solver.Solve(
				rejected_job,
				physics::tests::FreshConstraintTestEpoch(1.0f / 60.0f, rejected_config.constraint_warm_start_factor),
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_preconditioners);

			PR_EXPECT(rejected.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(rejected.m_island_states[0].iteration_count == 1u);
			PR_EXPECT(rejected.m_results[0] == GpuConstraintStage_SolveRejected);
			PR_EXPECT(rejected.m_acceptance[0].valid == 0u);
			PR_EXPECT(AllSet(rejected.m_island_states[0].failure_flags, GpuCoupledConstraintFailure_Merit));
			PR_EXPECT(rejected.m_island_states[0].merit_change > 0.0f);
			PR_EXPECT(std::memcmp(rejected.m_bodies.data(), inputs.m_bodies.data(), rejected.m_bodies.size() * sizeof(rejected.m_bodies[0])) == 0);
			PR_EXPECT(std::memcmp(rejected.m_articulation_velocities.data(), inputs.m_articulation_upload.m_velocities.data(), rejected.m_articulation_velocities.size() * sizeof(rejected.m_articulation_velocities[0])) == 0);
			// Prepared coefficients are private trial scratch; only the authoritative accepted ledger must survive rejection unchanged.
			for (size_t row_idx = 0; row_idx != rejected.m_rows.size(); ++row_idx)
				PR_EXPECT(rejected.m_accepted_rows[row_idx / GpuConstraintRowsPerBlock].impulses[row_idx % GpuConstraintRowsPerBlock] == inputs.m_rows[row_idx].bounds.z);
			PR_EXPECT(rejected_solver.Stats().m_dispatch_count == 44);
		}
	};

}
#endif
