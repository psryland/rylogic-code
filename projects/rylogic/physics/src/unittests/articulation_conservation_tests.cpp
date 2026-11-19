//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return signed work from a constant scalar generalized torque and unwrapped joint coordinates.
		double ConstantScalarJointWork(float torque, float position, float initial_position)
		{
			// Promote before subtracting binary32 coordinates so their travel retains binary64 precision.
			return static_cast<double>(torque) * (static_cast<double>(position) - initial_position);
		}

		// Authored physical data shared by the integrated chain and branching controls.
		struct EngineConservationFixture
		{
			std::array<ArticulationLinkDesc, 10> m_links;
			std::array<ArticulationJointDesc, 10> m_joints;
			std::array<int, 10> m_parents;
		};

		// Binary32 primary state only: root matrix (column-major), root angular/linear velocity,
		// nine four-slot joint configurations, then nine three-slot joint velocities. Unused slots are zero.
		using EngineConservationState = std::array<float, 85>;
		using ConservationVector = Vec4<double>;
		using ConservationFrame = Mat4x4<double>;

		// World-origin physical invariants, evaluated without production spatial-inertia or energy helpers.
		struct EngineConservationInvariants
		{
			double m_energy = 0;
			ConservationVector m_linear = {};
			ConservationVector m_angular = {};
			std::array<double, 5> m_quaternion_squared_norm_error = {};
			std::array<double, 10> m_rotation_orthogonality_error = {};
		};

		// Build one floating ten-link tree with five spherical and four revolute joints and no external work.
		std::pair<Articulation, EngineConservationFixture> BuildEngineConservationTree(bool branching)
		{
			// The same masses, frames and initial speeds apply to both parent graphs.
			auto fixture = EngineConservationFixture{};
			auto builder = ArticulationBuilder{};
			auto handles = std::array<LinkHandle, 10>{};
			auto const axes = std::array{v4::XAxis(), v4::YAxis(), v4::ZAxis()};
			for (int index = 0; index != 10; ++index)
			{
				// Strictly positive asymmetric central tensors include products of inertia and nonzero COM offsets.
				fixture.m_links[index].m_inertia = Inertia(
					v4{0.12f + 0.01f * index, 0.18f + 0.014f * index, 0.23f + 0.016f * index, 0},
					v4{0.012f, -0.008f, 0.009f, 0}, index == 0 ? 3.1f : 1.0f + 0.1f * index,
					v4{-0.04f + 0.004f * index, 0.07f - 0.003f * index, 0.03f + 0.002f * index, 0});
				fixture.m_parents[index] = index == 0 ? -1 : branching ? (index - 1) / 2 : index - 1;
				if (index == 0)
				{
					// Translational motion keeps this conservative control away from automatic sleep without changing its settings.
					handles[index] = builder.AddFloatingRoot(fixture.m_links[index],
						m4x4::Transform(v4::YAxis(), 0.31f, v4{1.7f, -0.8f, 2.1f, 1}),
						v8motion{v4{0.71f, -0.46f, 0.58f, 0}, v4{1.2f, -0.7f, 0.9f, 0}});
					continue;
				}

				// Rotations and offsets on both sides of every joint exercise moving frames and orbital momentum.
				auto const parent_frame = m4x4::Transform(Normalise(v4{1, 1, 1, 0}), 2.09439510239319549f, v4{0.28f, -0.09f, 0.17f, 1});
				auto const child_frame = m4x4::Transform(Normalise(v4{-1, 2, 1, 0}), -0.19f - 0.01f * index, v4{-0.14f, 0.12f, 0.04f, 1});
				auto& joint = fixture.m_joints[index];
				if (index % 2 != 0)
				{
					// Spherical speeds are expressed in the moving child-joint frame.
					joint = ArticulationJointDesc::Spherical(parent_frame, child_frame);
					auto q = ExpMap<quat>(v3{0.13f + 0.01f * index, -0.21f, 0.17f});
					joint.m_initial_position = {q.x, q.y, q.z, q.w};
					joint.m_initial_velocity = {0.31f + 0.02f * index, -0.22f, 0.19f};
				}
				else
				{
					// Alternating scalar spins avoid a common-axis rigid rotation.
					joint = ArticulationJointDesc::Revolute(axes[index % 3], parent_frame, child_frame);
					joint.m_initial_position[0] = -0.36f + 0.04f * index;
					joint.m_initial_velocity[0] = (index % 4 == 0 ? -1.0f : 1.0f) * (0.4f + 0.03f * index);
				}
				handles[index] = builder.AddLink(handles[fixture.m_parents[index]], joint, fixture.m_links[index]);
			}
			return {builder.Build(), fixture};
		}

		// Copy only published primary getters, none of which refresh derived link kinematics.
		EngineConservationState ReadEngineConservationState(Articulation const& articulation)
		{
			// Fixed zero-filled slots make every published state independently decodable without object padding.
			auto state = EngineConservationState{};
			static_assert(sizeof(m4x4) == 16 * sizeof(float));
			memcpy(state.data(), &articulation.RootToWorld(), sizeof(m4x4));
			auto const root_velocity = articulation.RootVelocity();
			for (int axis = 0; axis != 3; ++axis)
			{
				// Root spatial components are expressed at the root origin in root-link axes.
				state[16 + axis] = root_velocity.ang[axis];
				state[19 + axis] = root_velocity.lin[axis];
			}
			for (int index = 1; index != 10; ++index)
			{
				// Configuration and velocity dimensions differ for spherical joints.
				auto const handle = articulation.LinkAt(index);
				auto const position = articulation.JointPosition(handle);
				auto const velocity = articulation.JointVelocity(handle);
				std::copy(position.begin(), position.end(), state.begin() + 22 + 4 * (index - 1));
				std::copy(velocity.begin(), velocity.end(), state.begin() + 58 + 3 * (index - 1));
			}
			return state;
		}

		// Evaluate the raw recorded quaternion polynomial in double precision without repairing its representation.
		ConservationFrame ConservationRotation(ConservationVector q)
		{
			// Length errors remain observable in the resulting rotation rather than being normalized away.
			auto const [x, y, z, w] = std::array{q.x, q.y, q.z, q.w};
			return ConservationFrame{
				{1 - 2 * (y*y + z*z), 2 * (x*y + z*w), 2 * (x*z - y*w), 0},
				{2 * (x*y - z*w), 1 - 2 * (x*x + z*z), 2 * (y*z + x*w), 0},
				{2 * (x*z + y*w), 2 * (y*z - x*w), 1 - 2 * (x*x + y*y), 0},
				{0, 0, 0, 1}};
		}

		// Reconstruct world kinematics from primary coordinates and authored frames, then sum COM energy and world-origin momenta.
		EngineConservationInvariants EngineConservationMetric(EngineConservationFixture const& fixture, EngineConservationState const& state)
		{
			// All calculations below are binary64; no production kinematics, cache, inertia multiplication or work report is queried.
			auto frames = std::array<ConservationFrame, 10>{};
			auto angular = std::array<ConservationVector, 10>{};
			auto linear = std::array<ConservationVector, 10>{};
			auto result = EngineConservationInvariants{};
			for (int column = 0; column != 4; ++column)
			{
				// The published root matrix is retained exactly as binary32 before promotion.
				for (int row = 0; row != 4; ++row)
					frames[0][column][row] = state[4 * column + row];
			}
			angular[0] = frames[0] * ConservationVector{state[16], state[17], state[18], 0};
			linear[0] = frames[0] * ConservationVector{state[19], state[20], state[21], 0};
			for (int index = 0; index != 10; ++index)
			{
				// A parent's pivot velocity is shared by its child; the child's own angular velocity locates its origin from that pivot.
				if (index != 0)
				{
					// Joint coordinates determine orientation, while relative angular velocity uses the child-joint frame.
					auto const& joint = fixture.m_joints[index];
					auto const* q = state.data() + 22 + 4 * (index - 1);
					auto const* speed = state.data() + 58 + 3 * (index - 1);
					auto relative = ConservationVector{};
					auto rotation = ConservationFrame{};
					switch (joint.m_type)
					{
						case EArticulationJointType::Spherical:
						{
							// Retain the signed squared-norm error of each recorded quaternion without imposing a new tolerance.
							auto const orientation = ConservationVector{q[0], q[1], q[2], q[3]};
							result.m_quaternion_squared_norm_error[(index - 1) / 2] = orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z + orientation.w * orientation.w - 1;
							rotation = ConservationRotation(orientation);
							relative = {speed[0], speed[1], speed[2], 0};
							break;
						}
						case EArticulationJointType::Axes:
						{
							// This fixture owns exactly one revolute axis in every scalar joint.
							assert(joint.m_dof_count == 1 && joint.m_axes[0].m_type == EArticulationAxisType::Revolute);
							auto const axis = static_cast<ConservationVector>(joint.m_axes[0].m_axis);
							auto orientation = std::sin(0.5 * q[0]) * axis;
							orientation.w = std::cos(0.5 * q[0]);
							rotation = ConservationRotation(orientation);
							relative = static_cast<double>(speed[0]) * axis;
							break;
						}
						default: { throw std::runtime_error("Unexpected conservative fixture joint"); }
					}
					auto const parent = fixture.m_parents[index];
					auto const parent_frame = static_cast<ConservationFrame>(joint.m_joint_to_parent);
					auto const child_frame = static_cast<ConservationFrame>(joint.m_joint_to_child);
					auto const pivot_frame = frames[parent] * parent_frame;
					auto const pivot = pivot_frame.pos;

					// Authored rotations have binary32 roundoff. Use the rigid-frame transpose convention on their exact promoted
					// coefficients, without demanding binary64 orthonormality or projecting either frame back onto a rotation.
					auto const child_rotation = pivot_frame.rot * rotation.rot * Transpose(child_frame.rot);
					frames[index] = ConservationFrame{child_rotation, pivot - child_rotation * child_frame.pos.w0()};
					angular[index] = angular[parent] + frames[index] * child_frame * relative;
					linear[index] = linear[parent] + Cross(angular[parent], pivot - frames[parent].pos) + Cross(angular[index], frames[index].pos - pivot);
				}

				// Report max |R^T R - I| for each reconstructed world rotation, including the raw published root matrix.
				for (int column = 0; column != 3; ++column)
				{
					// Diagonal and off-diagonal defects are diagnostics only, separate from the physical energy screen.
					for (int row = 0; row != 3; ++row)
						result.m_rotation_orthogonality_error[index] = std::max(result.m_rotation_orthogonality_error[index], std::abs(Dot3(frames[index][column], frames[index][row]) - (column == row ? 1.0 : 0.0)));
				}

				// Central rotational energy and COM translation avoid mixing inertia about the link origin with COM velocity.
				auto const& inertia = fixture.m_links[index].m_inertia;
				auto const mass = static_cast<double>(inertia.m_com_and_mass.w);
				auto const centre_offset = frames[index] * static_cast<ConservationVector>(inertia.m_com_and_mass.w0());
				auto const centre = frames[index].pos + centre_offset;
				auto const centre_velocity = linear[index] + Cross(angular[index], centre_offset);
				auto const omega = ConservationVector{Dot3(frames[index].x, angular[index]), Dot3(frames[index].y, angular[index]), Dot3(frames[index].z, angular[index]), 0};
				auto const d = static_cast<ConservationVector>(inertia.m_diagonal);
				auto const p = static_cast<ConservationVector>(inertia.m_products);
				auto const spin = mass * ConservationVector{d.x * omega.x + p.x * omega.y + p.y * omega.z, p.x * omega.x + d.y * omega.y + p.z * omega.z, p.y * omega.x + p.z * omega.y + d.z * omega.z, 0};
				auto const momentum = mass * centre_velocity;
				result.m_energy += 0.5 * (Dot3(omega, spin) + mass * Dot3(centre_velocity, centre_velocity));
				result.m_linear += momentum;
				result.m_angular += frames[index] * spin + Cross(centre.w0(), momentum);
			}
			return result;
		}

		// Return a finite asymmetric link inertia with an offset centre of mass.
		ArticulationLinkDesc ConservationLink(v4 half_extents, float mass, v4 centre_of_mass)
		{
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(half_extents, mass, centre_of_mass),
			};
		}

		// Return a revolute joint with explicit parent and child attachment frames.
		ArticulationJointDesc RevoluteJoint(v4 axis, m4x4 const& joint_to_parent, m4x4 const& joint_to_child, float position, float velocity)
		{
			auto joint = ArticulationJointDesc::Revolute(axis, joint_to_parent, joint_to_child);
			joint.m_initial_position[0] = position;
			joint.m_initial_velocity[0] = velocity;
			return joint;
		}

		// Build a free asymmetric three-link tree with internal motion and no external force.
		Articulation BuildConservationArticulation()
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				ConservationLink(v4{0.45f, 0.63f, 0.82f, 0}, 3.1f, v4{0.08f, -0.05f, 0.04f, 0}),
				m4x4::Transform(v4::YAxis(), 0.31f, v4{1.7f, -0.8f, 2.1f, 1}),
				v8motion{v4{0.71f, -0.46f, 0.58f, 0}, v4{-0.23f, 0.37f, 0.19f, 0}});
			auto const child = builder.AddLink(
				root,
				RevoluteJoint(
					Normalise(v4{1, 2, -1, 0}),
					m4x4::Translation(+0.62f, -0.11f, +0.17f),
					m4x4::Translation(-0.31f, +0.09f, -0.14f),
					0.43f,
					1.17f),
				ConservationLink(v4{0.27f, 0.39f, 0.51f, 0}, 1.4f, v4{-0.04f, 0.07f, 0.03f, 0}));
			builder.AddLink(
				child,
				RevoluteJoint(
					Normalise(v4{-1, 1, 2, 0}),
					m4x4::Translation(-0.23f, +0.41f, +0.12f),
					m4x4::Translation(+0.18f, -0.26f, +0.08f),
					-0.36f,
					-0.83f),
				ConservationLink(v4{0.19f, 0.28f, 0.34f, 0}, 0.8f, v4{0.03f, 0.02f, -0.05f, 0}));
			return builder.Build();
		}

		// Long-run invariant errors and least-squares relative energy trend.
		struct ConservationMetrics
		{
			float m_max_linear_momentum_error = 0.0f;
			float m_max_angular_momentum_error = 0.0f;
			float m_max_energy_error = 0.0f;
			double m_energy_slope = 0.0;
		};

		// Measure invariant envelopes and secular energy trend over a force-free trajectory.
		ConservationMetrics MeasureConservation(float timestep, float duration)
		{
			auto articulation = BuildConservationArticulation();
			auto const momentum_start = articulation.MomentumWS();
			auto const energy_start = articulation.KineticEnergy();
			auto const linear_scale = std::max(Length(momentum_start.lin), 1.0f);
			auto const angular_scale = std::max(Length(momentum_start.ang), 1.0f);
			auto const energy_scale = std::max(Abs(energy_start), 1.0f);
			auto metrics = ConservationMetrics{};
			auto sample_count = 0.0;
			auto sum_time = 0.0;
			auto sum_error = 0.0;
			auto sum_time_sq = 0.0;
			auto sum_time_error = 0.0;

			auto const step_count = static_cast<int>(std::round(duration / timestep));
			for (int step = 0; step != step_count; ++step)
			{
				articulation.Integrate(timestep);

				// Track all invariants every step so oscillatory errors cannot hide between sparse samples.
				auto const time = static_cast<double>(step + 1) * timestep;
				auto const momentum = articulation.MomentumWS();
				auto const relative_energy_error = static_cast<double>(articulation.KineticEnergy() - energy_start) / energy_scale;
				metrics.m_max_linear_momentum_error = std::max(metrics.m_max_linear_momentum_error, Length(momentum.lin - momentum_start.lin) / linear_scale);
				metrics.m_max_angular_momentum_error = std::max(metrics.m_max_angular_momentum_error, Length(momentum.ang - momentum_start.ang) / angular_scale);
				metrics.m_max_energy_error = std::max(metrics.m_max_energy_error, static_cast<float>(std::abs(relative_energy_error)));
				sample_count += 1.0;
				sum_time += time;
				sum_error += relative_energy_error;
				sum_time_sq += time * time;
				sum_time_error += time * relative_energy_error;
			}

			// A positive least-squares slope reveals secular gain even when the bounded error oscillates around zero.
			auto const denominator = sample_count * sum_time_sq - sum_time * sum_time;
			metrics.m_energy_slope = denominator != 0.0
				? (sample_count * sum_time_error - sum_time * sum_error) / denominator
				: 0.0;
			return metrics;
		}

		// Advance one fresh articulation to a common endpoint for timestep-convergence comparisons.
		Articulation Simulate(float timestep, float duration)
		{
			auto articulation = BuildConservationArticulation();
			auto const step_count = static_cast<int>(std::round(duration / timestep));
			for (int step = 0; step != step_count; ++step)
				articulation.Integrate(timestep);

			return articulation;
		}

		// Return a scale-independent maximum state difference between equal-topology articulations.
		float StateDifference(Articulation const& lhs, Articulation const& rhs)
		{
			auto difference = 0.0f;
			auto const lhs_root = lhs.RootToWorld();
			auto const rhs_root = rhs.RootToWorld();
			for (int column = 0; column != 4; ++column)
				difference = std::max(difference, Length(lhs_root[column] - rhs_root[column]));
			difference = std::max(difference, Length(lhs.RootVelocity().ang - rhs.RootVelocity().ang));
			difference = std::max(difference, Length(lhs.RootVelocity().lin - rhs.RootVelocity().lin));

			// Stable topological order makes every reduced state range directly comparable.
			for (int link_index = 1; link_index != lhs.LinkCount(); ++link_index)
			{
				auto const lhs_link = lhs.LinkAt(link_index);
				auto const rhs_link = rhs.LinkAt(link_index);
				for (int axis_index = 0; axis_index != lhs.JointDofCount(lhs_link); ++axis_index)
				{
					difference = std::max(difference, Abs(lhs.JointPosition(lhs_link)[axis_index] - rhs.JointPosition(rhs_link)[axis_index]));
					difference = std::max(difference, Abs(lhs.JointVelocity(lhs_link)[axis_index] - rhs.JointVelocity(rhs_link)[axis_index]));
				}
			}
			return difference;
		}
	}

	PRUnitTestClass(ArticulationConservationTests)
	{
		// Check the same signed, unwrapped scalar work expression used by the trajectory ledger.
		PRUnitTestMethod(ScalarJointWorkAnalytic, Quick)
		{
			// Eight radians exceeds one turn; reverse travel must remove rather than add actuator work.
			PR_EXPECT(ConstantScalarJointWork(0.25f, 9.0f, 1.0f) == 2.0);
			PR_EXPECT(ConstantScalarJointWork(0.25f, 1.0f, 9.0f) == -2.0);

			// Torque sign and travel sign both matter; zero torque or zero travel contributes no work.
			PR_EXPECT(ConstantScalarJointWork(-0.5f, 1.0f, -3.0f) == -2.0);
			PR_EXPECT(ConstantScalarJointWork(-0.5f, -3.0f, 1.0f) == 2.0);
			PR_EXPECT(ConstantScalarJointWork(0.0f, 9.0f, -9.0f) == 0.0);
			PR_EXPECT(ConstantScalarJointWork(0.25f, 3.0f, 3.0f) == 0.0);

			// Binary32 subtraction would lose the half-ULP term at one; the expected result is an exact binary fraction.
			PR_EXPECT(ConstantScalarJointWork(0.25f, 1.0f, -0x1p-24f) == 0.25 + 0x1p-26);
		}

		// Screen integrated mixed-joint chain energy over ten seconds of ordinary Engine steps.
		PRUnitTestMethod(EngineTenLinkChainConservation, Extended)
		{
			// Keep each topology selectable independently so a failed control need not be followed by another run.
			CheckEngineTenLinkEnergyBalance(false, 3, 0.0f);
		}

		// Screen the same physical inputs with branches rather than a serial parent graph.
		PRUnitTestMethod(EngineTenLinkBranchConservation, Extended)
		{
			// Share the fixture, publication sampling and physical oracle with the chain case.
			CheckEngineTenLinkEnergyBalance(true, 3, 0.0f);
		}

		// Characterize internal timestep refinement without changing the fixture or publication schedule.
		PRUnitTestMethod(EngineTenLinkChainSixSubstepConservation, Extended)
		{
			// Six internal substeps retain the same 600 Engine visits and 601 published-state observations.
			CheckEngineTenLinkEnergyBalance(false, 6, 0.0f);
		}

		// Account for signed work from constant hinge torques during ten seconds of full-Engine mixed-joint motion.
		PRUnitTestMethod(EngineTenLinkChainDrivenWork, Extended)
		{
			// The exact binary32 torque is applied to all four hinges; the physical fixture and initial motion remain unchanged.
			CheckEngineTenLinkEnergyBalance(false, 3, 0.25f);
		}

		// Retain 601 published states and screen energy minus constant hinge work, with zero torque recovering the free controls.
		void CheckEngineTenLinkEnergyBalance(bool branching, int substep_count, float hinge_torque)
		{
			// Default Engine settings and the original physical fixture apply to both free and driven inputs.
			auto const* label = branching ? "engine-branch" : "engine-chain";
			auto [articulation, fixture] = BuildEngineConservationTree(branching);
			auto forest = std::array{&articulation};
			auto const dt = 1.0f / 60.0f;
			auto open = [&](char const* suffix, bool binary = false)
			{
				// Stream exceptions expose evidence-write failures rather than silently losing a failing state.
				auto stream = std::ofstream{};
				stream.exceptions(std::ios::badbit | std::ios::failbit);
				stream.open(temp_dir() / std::format("{}-{}", label, suffix), binary ? std::ios::binary : std::ios::out);
				stream.precision(17);
				return stream;
			};
			auto raw = open("primary.bin", true);
			auto metrics = open("metrics.csv");
			auto inputs = open("fixture.csv");
			auto config = open("config.txt");
			auto forces = open("forces.bin", true);
			auto work_records = open("work.csv");
			config << "EngineConfig{} defaults; ResetEngineForNextTest; no custom force subscribers, shapes, constraints, water, gravity or damping\n"
				<< "steps=600\nsubsteps=" << substep_count << "\nelapsed_seconds=" << dt << "\nsubstep_seconds=" << dt / substep_count << "\n"
				<< "primary_record=85 little-endian binary32 values: root matrix16(column-major),root velocity6(angular then linear,root frame),joint positions9x4,joint speeds9x3\n"
				<< "energy_scale=initial positive physical kinetic energy; positive deviation screen=0.01; no momentum tolerance\n"
				<< "oracle=raw quaternion polynomial; exact promoted authored frame coefficients with rigid transpose; no normalization/projection\n"
				<< "quaternion_norm_error=q dot q-1 for spherical links1,3,5,7,9; rotation_orthogonality_error=max_abs(R^T R-I) for world rotations0..9; diagnostic only, no thresholds\n"
				<< "hinge_torque=" << hinge_torque << "\nhinge_links=2,4,6,8; JointForce replaces each scalar torque before every Engine::Step; root/spherical forces remain zero\n"
				<< "force_record=25 little-endian binary32 values per submitted frame: root angular3/linear3, then joint forces in link order (spherical3,revolute1)\n"
				<< "work=sum(tau_j*(q_j-q_j_initial)); q_j are unwrapped scalar hinge coordinates in radians, not quaternion displacement\n"
				<< "balance=K-K_initial-work; relative balance=balance/K_initial; no work-dependent scale or projection work\n"
				<< "window mean/min/max=K-work; signed work and energy balance are retained for every published state\n";
			config.flush();
			inputs << "link,parent,kind,dof,ixx,iyy,izz,ixy,ixz,iyz,cx,cy,cz,mass";
			for (auto const* name : {"parent", "child"})
			{
				// Frame coefficients are column-major and copied directly from the authored descriptors.
				for (int index = 0; index != 16; ++index)
					inputs << ',' << name << index;
			}
			inputs << ",axis_x,axis_y,axis_z\n";
			for (int index = 0; index != 10; ++index)
			{
				// Initial generalized coordinates and root motion are in raw state zero, not inferred from the dynamics output.
				auto const& joint = fixture.m_joints[index];
				auto const& inertia = fixture.m_links[index].m_inertia;
				inputs << index << ',' << fixture.m_parents[index] << ',' << static_cast<int>(joint.m_type) << ',' << joint.m_dof_count;
				for (auto vector : {inertia.m_diagonal, inertia.m_products, inertia.m_com_and_mass})
				{
					// Tensor coefficients and COM offsets each occupy three columns; mass follows separately.
					for (int axis = 0; axis != 3; ++axis)
						inputs << ',' << vector[axis];
				}
				inputs << ',' << inertia.m_com_and_mass.w;
				for (auto const& frame : {joint.m_joint_to_parent, joint.m_joint_to_child})
				{
					// Write every coefficient, including homogeneous entries, for independent reconstruction.
					for (int column = 0; column != 4; ++column)
						for (int row = 0; row != 4; ++row)
							inputs << ',' << frame[column][row];
				}
				inputs << ',' << joint.m_axes[0].m_axis.x << ',' << joint.m_axes[0].m_axis.y << ',' << joint.m_axes[0].m_axis.z << '\n';
			}
			inputs.flush();

			// Persist the initial primary input exactly once before the oracle or GPU setup can fail.
			auto state = ReadEngineConservationState(articulation);
			auto const initial_state = state;
			raw.write(reinterpret_cast<char const*>(state.data()), sizeof(state));
			raw.flush();

			// Initial energy is a nonzero physical scale supplied by this moving finite-mass fixture, not an arbitrary numeric floor.
			auto const initial = EngineConservationMetric(fixture, state);
			PR_EXPECT(std::isfinite(initial.m_energy) && initial.m_energy > 0);
			if (!std::isfinite(initial.m_energy) || initial.m_energy <= 0)
				return;

			// GPU resources are only acquired after the CPU observer has evaluated its retained initial input.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto const scale = initial.m_energy;
			auto max_gain = 0.0, max_loss = 0.0, max_linear = 0.0, max_angular = 0.0;
			auto max_quaternion_norm_error = 0.0, max_rotation_orthogonality_error = 0.0;
			auto total_work = 0.0, max_abs_work = 0.0, balance = 0.0;
			auto sum_t = 0.0, sum_e = 0.0, sum_tt = 0.0, sum_te = 0.0;
			auto window_sum = 0.0, window_min = scale, window_max = scale;
			auto all_finite = true, all_awake = true, substeps_match = true;
			auto final = initial;
			metrics << "step,time,energy,px,py,pz,lx,ly,lz,substeps,sleeping";
			for (int index = 0; index != 5; ++index)
				metrics << ",q" << 2 * index + 1 << "_squared_norm_error";

			for (int index = 0; index != 10; ++index)
				metrics << ",r" << index << "_orthogonality_error";

			metrics << '\n';
			work_records << "step,time,q2,q4,q6,q8,work2,work4,work6,work8,total_work,energy_delta,balance,balance_relative,work_to_screen\n";
			for (int step = 0; step != 601; ++step)
			{
				// Inspect only the public completion boundary, with no cache-warming link or aggregate physics queries.
				if (step != 0)
				{
					// Public generalized forces are frame inputs retained across internal substeps; zero-torque controls keep their original calls.
					if (hinge_torque != 0.0f)
					{
						// A scalar revolute torque does work against its own unwrapped coordinate, including negative travel.
						for (int link = 2; link != 10; link += 2)
							articulation.JointForce(articulation.LinkAt(link), std::array{hinge_torque});
					}

					// Retain all actual public generalized forces before submission, including the unforced root and spherical components.
					auto applied = std::array<float, 25>{};
					auto const root_force = articulation.RootForce();
					for (int axis = 0; axis != 3; ++axis)
					{
						// Root wrench components use the same angular/linear ordering as the primary root velocity.
						applied[axis] = root_force.ang[axis];
						applied[3 + axis] = root_force.lin[axis];
					}
					auto cursor = applied.begin() + 6;
					for (int link = 1; link != 10; ++link)
					{
						// Read the setter's stored force values without refreshing kinematic caches.
						auto const force = articulation.JointForce(articulation.LinkAt(link));
						cursor = std::copy(force.begin(), force.end(), cursor);
					}
					forces.write(reinterpret_cast<char const*>(applied.data()), sizeof(applied));
					forces.flush();

					// The publication schedule and ordinary internal subdivision are unchanged.
					engine.Step(Engine::StepInput{.m_articulations = forest, .m_elapsed_seconds = dt, .m_substep_count = substep_count, .m_time_s = (step - 1) * static_cast<double>(dt)});
					state = ReadEngineConservationState(articulation);
					substeps_match &= engine.LastStepProfile().m_substep_count == substep_count;
					raw.write(reinterpret_cast<char const*>(state.data()), sizeof(state));
					raw.flush();
					final = EngineConservationMetric(fixture, state);
				}
				auto const time = step * static_cast<double>(dt);

				// Constant generalized torque has an exact endpoint line integral even while passive spherical joints move.
				auto hinge_work = std::array<double, 4>{};
				total_work = 0;
				work_records << step << ',' << time;
				for (int hinge = 0; hinge != 4; ++hinge)
				{
					// Promote before subtraction and keep signed scalar travel without wrapping it onto an orientation interval.
					auto const position_slot = 22 + 4 * (2 + 2 * hinge - 1);
					hinge_work[hinge] = ConstantScalarJointWork(hinge_torque, state[position_slot], initial_state[position_slot]);
					total_work += hinge_work[hinge];
					work_records << ',' << state[position_slot];
				}
				for (auto value : hinge_work)
					work_records << ',' << value;

				// Applied work never enlarges the initial-energy denominator or hides a positive unexplained balance.
				balance = final.m_energy - initial.m_energy - total_work;
				auto const error = balance / scale;
				work_records << ',' << total_work << ',' << final.m_energy - initial.m_energy << ',' << balance << ',' << error << ',' << total_work / (0.01 * scale) << '\n';
				work_records.flush();
				max_abs_work = std::max(max_abs_work, std::abs(total_work));
				all_finite &= std::isfinite(final.m_energy) && std::isfinite(balance) && std::isfinite(total_work) && IsFinite(final.m_linear) && IsFinite(final.m_angular) && std::ranges::all_of(state, [](float value)
				{
					// The metric alone must not hide a nonfinite unused or homogeneous primary component.
					return std::isfinite(value);
				});
				all_awake &= !articulation.Sleeping();
				metrics << step << ',' << time << ',' << final.m_energy << ',' << final.m_linear.x << ',' << final.m_linear.y << ',' << final.m_linear.z
					<< ',' << final.m_angular.x << ',' << final.m_angular.y << ',' << final.m_angular.z << ',' << (step != 0 ? engine.LastStepProfile().m_substep_count : 0) << ',' << articulation.Sleeping();
				for (auto const error_value : final.m_quaternion_squared_norm_error)
				{
					// Preserve signed per-joint defects and report the largest absolute error over the trajectory.
					metrics << ',' << error_value;
					max_quaternion_norm_error = std::max(max_quaternion_norm_error, std::abs(error_value));
				}
				for (auto const error_value : final.m_rotation_orthogonality_error)
				{
					// World-rotation defects include the accumulated authored-frame and primary-state roundoff.
					metrics << ',' << error_value;
					max_rotation_orthogonality_error = std::max(max_rotation_orthogonality_error, error_value);
				}
				metrics << '\n';
				metrics.flush();
				max_gain = std::max(max_gain, error);
				max_loss = std::max(max_loss, -error);
				max_linear = std::max(max_linear, Length(final.m_linear - initial.m_linear));
				max_angular = std::max(max_angular, Length(final.m_angular - initial.m_angular));
				sum_t += time; sum_e += error; sum_tt += time * time; sum_te += time * error;
				if (step != 0)
				{
					// Removing signed input work leaves the same raw-energy windows for the zero-torque controls.
					auto const corrected_energy = final.m_energy - total_work;
					if (step % 60 == 1)
					{
						// Each window starts from its own first published value, rather than including the initial scale as an extremum.
						window_sum = 0; window_min = corrected_energy; window_max = corrected_energy;
					}
					window_sum += corrected_energy;
					window_min = std::min(window_min, corrected_energy);
					window_max = std::max(window_max, corrected_energy);
					if (step % 60 == 0)
						printf("%s window=%d mean=%.17g min=%.17g max=%.17g\n", label, step / 60, window_sum / 60, window_min, window_max);
				}

				// Preserve the first failing published state and its work ledger before stopping without another Engine visit.
				if (!all_finite || !all_awake || !substeps_match || !(error <= 0.01))
				{
					// These are the existing finite/awake/substep and positive-energy obligations, not additional physical tolerances.
					printf("%s first_failure_step=%d energy=%.17g work=%.17g balance=%.17g relative=%.17g\n", label, step, final.m_energy, total_work, balance, error);
					PR_EXPECT(all_finite);
					PR_EXPECT(all_awake && substeps_match);
					PR_EXPECT(error <= 0.01);
					ResetEngineForNextTest(engine);
					return;
				}
			}

			// These are diagnostic envelopes and a finite-window least-squares slope, not an exact-conservation or asymptotic theorem.
			auto const slope = (601 * sum_te - sum_t * sum_e) / (601 * sum_tt - sum_t * sum_t);
			printf("%s initial_energy=%.17g max_positive_relative=%.17g max_loss_relative=%.17g endpoint_relative=%.17g relative_slope_per_second=%.17g max_linear_error=%.17g max_angular_error=%.17g\n",
				label, scale, max_gain, max_loss, balance / scale, slope, max_linear, max_angular);
			printf("%s endpoint_energy=%.17g endpoint_work=%.17g endpoint_balance=%.17g max_abs_work=%.17g max_abs_work_to_screen=%.17g\n", label, final.m_energy, total_work, balance, max_abs_work, max_abs_work / (0.01 * scale));
			printf("%s max_abs_quaternion_squared_norm_error=%.17g max_rotation_orthogonality_error=%.17g\n", label, max_quaternion_norm_error, max_rotation_orthogonality_error);
			PR_EXPECT(all_finite);
			PR_EXPECT(all_awake && substeps_match);
			PR_EXPECT(state != initial_state);
			PR_EXPECT(max_gain <= 0.01);
			ResetEngineForNextTest(engine);
		}

		// Integrate applied force once, retain finite state, and clear the per-step force accumulators.
		PRUnitTestMethod(IntegrationConsumesForces, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot({});
			auto const child = builder.AddLink(
				root,
				RevoluteJoint(v4::ZAxis(), m4x4::Identity(), m4x4::Identity(), 0.2f, -0.3f),
				ConservationLink(v4{0.3f, 0.4f, 0.5f, 0}, 1.2f, v4{}));
			auto articulation = builder.Build();
			articulation.JointForce(child, std::array{0.7f});
			auto const position_start = articulation.JointPosition(child)[0];
			auto const velocity_start = articulation.JointVelocity(child)[0];

			articulation.Integrate(1.0f / 240.0f);

			PR_EXPECT(articulation.JointPosition(child)[0] != position_start);
			PR_EXPECT(articulation.JointVelocity(child)[0] != velocity_start);
			PR_EXPECT(articulation.JointForce(child)[0] == 0.0f);
			PR_EXPECT(IsFinite(articulation.KineticEnergy()));
		}

		// Reject invalid timesteps without changing accepted articulation state.
		PRUnitTestMethod(InvalidTimestepPreservesState, Quick)
		{
			auto articulation = BuildConservationArticulation();
			auto const transform_start = articulation.RootToWorld();
			auto const velocity_start = articulation.RootVelocity();
			PR_THROWS(articulation.Integrate(-0.01f), std::exception);
			PR_THROWS(articulation.Integrate(std::numeric_limits<float>::quiet_NaN()), std::exception);
			PR_EXPECT(FEql(articulation.RootToWorld(), transform_start));
			PR_EXPECT(FEql(articulation.RootVelocity(), velocity_start));
		}

		// Restore accepted generalized, pose, and acceleration outputs after a nonlinear midpoint solve fails.
		PRUnitTestMethod(MidpointFailureRollsBackState, Quick)
		{
			auto articulation = BuildConservationArticulation();
			articulation.ForwardDynamics();
			auto const accepted = BuildConservationArticulation();
			auto const root_acceleration = articulation.RootAcceleration();
			auto link_acceleration = std::vector<v8motion>(articulation.LinkCount());
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				link_acceleration[link_index] = articulation.LinkAcceleration(articulation.LinkAt(link_index));

			PR_THROWS(articulation.Integrate(100.0f), std::exception);

			PR_EXPECT(StateDifference(articulation, accepted) == 0.0f);
			PR_EXPECT(FEql(articulation.RootAcceleration(), root_acceleration));
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				PR_EXPECT(FEql(articulation.LinkAcceleration(articulation.LinkAt(link_index)), link_acceleration[link_index]));

			// A failed large step must leave the articulation usable for a smaller retry.
			articulation.Integrate(1.0f / 1000.0f);
			PR_EXPECT(IsFinite(articulation.KineticEnergy()));
		}

		// Preserve force-free world momentum and bounded energy without a positive secular trend.
		PRUnitTestMethod(FreeFloatingConservation, Extended)
		{
			auto const metrics = MeasureConservation(1.0f / 1000.0f, 5.0f);
			PR_EXPECT(metrics.m_max_linear_momentum_error < 1.0e-4f);
			PR_EXPECT(metrics.m_max_angular_momentum_error < 1.0e-4f);
			PR_EXPECT(metrics.m_max_energy_error < 2.0e-5f);
			PR_EXPECT(metrics.m_energy_slope < 1.0e-5);
		}

		// Require trajectory error to decrease when the implicit-midpoint timestep is halved.
		PRUnitTestMethod(TimestepConvergence, Extended)
		{
			auto const coarse = Simulate(1.0f / 30.0f, 1.0f);
			auto const medium = Simulate(1.0f / 60.0f, 1.0f);
			auto const fine = Simulate(1.0f / 120.0f, 1.0f);
			auto const coarse_error = StateDifference(coarse, fine);
			auto const medium_error = StateDifference(medium, fine);
			PR_EXPECT(medium_error < 0.25f * coarse_error);
		}
	};
}
#endif
