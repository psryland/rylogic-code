//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"
#include "src/constraint/constraint_gpu.h"
#include "src/constraint/constraint_solver.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/interop/constraint_runner.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// Construct a finite body with controllable transform and off-origin centre of mass.
		RigidBody MakeConstraintGpuBody(float mass = 1.0f, m4x4 const& o2w = m4x4::Identity(), v4 com_os = v4::Zero())
		{
			auto body = RigidBody{};
			body.SetMassProperties(Inertia::Sphere(1.0f, mass, com_os), com_os);
			body.O2W(o2w);
			return body;
		}

		// Return a hard locked axis with a finite force bound.
		ConstraintAxisDesc MakeConstraintGpuLockedAxis(float max_force = 1000.0f)
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			axis.m_max_force = max_force;
			return axis;
		}

		// Compare one runtime Jacobian with the corresponding CPU compiler row.
		void ExpectConstraintGpuJacobian(GpuConstraintRow const& gpu, CompiledConstraintRow const& cpu, float tolerance = 2.0e-5f)
		{
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_a_ang, cpu.m_jacobian_a.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_a_lin, cpu.m_jacobian_a.lin, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_b_ang, cpu.m_jacobian_b.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_b_lin, cpu.m_jacobian_b.lin, tolerance));
		}

		// Return true when all transform and momentum components of a packed body are finite.
		bool ConstraintGpuBodyFinite(GpuRigidBody const& body)
		{
			for (int row = 0; row != 4; ++row)
				for (int column = 0; column != 4; ++column)
					if (!std::isfinite(body.o2w[row][column]))
						return false;
			return
				IsFinite(body.momentum_ang) &&
				IsFinite(body.momentum_lin);
		}

		// Reuse one D3D12 device and command job across hardware constraint tests.
		Gpu& ConstraintTestGpu()
		{
			return SharedTestGpu();
		}

		// Return a short velocity-only solve configuration for hub topology tests.
		CpuConstraintSolverConfig MakeConstraintGpuHubConfig()
		{
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 2;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			return config;
		}

		// Solve one frame of a hub where every leaf body is locked to body zero, so each constraint needs a distinct colour.
		void RunConstraintGpuHub(ConstraintInteropRunner& runner, int leaf_count, std::vector<GpuRigidBody>& bodies)
		{
			// Bodies are slightly separated so every locked axis has a finite, well-defined frame.
			bodies.clear();
			for (int body_idx = 0; body_idx != leaf_count + 1; ++body_idx)
			{
				auto const rigid_body = MakeConstraintGpuBody(1.0f, m4x4::Translation(0.01f * body_idx, 0, 0));
				bodies.push_back(PackDynamics(rigid_body, body_idx));
			}

			// Each slot couples the shared hub to its own leaf.
			auto endpoints = std::vector<GpuConstraintEndpoint>(leaf_count);
			auto descriptors = std::vector<GpuD6ConstraintDesc>(leaf_count);
			for (int slot_idx = 0; slot_idx != leaf_count; ++slot_idx)
			{
				endpoints[slot_idx].body_idx_a = 0;
				endpoints[slot_idx].body_idx_b = slot_idx + 1;
				endpoints[slot_idx].flags = GpuConstraintEndpointFlags_Enabled;
				descriptors[slot_idx].frame_a.rotation = float4{0, 0, 0, 1};
				descriptors[slot_idx].frame_b.rotation = float4{0, 0, 0, 1};
				descriptors[slot_idx].axes[0].mode = GpuConstraintAxisMode_Locked;
				descriptors[slot_idx].axes[0].max_force = 1000.0f;
			}
			runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, endpoints, descriptors});
		}

		// Build a minimal floating tree for testing the unsupported coupled-solver boundary.
		std::pair<Articulation, LinkHandle> MakeConstraintGpuArticulation()
		{
			auto const link = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.25f, 1.0f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link);
			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis()), link);
			return {builder.Build(), child};
		}
	}

	// Compare actual descriptor packing with an explicit scalar oracle independent of the public struct layout.
	PRUnitTestClass(ConstraintAxisPackingTests)
	{
		// Map frames and every canonical axis field independently, without copying public object representations.
		static GpuD6ConstraintDesc ExplicitDescriptor(D6ConstraintDesc const& desc)
		{
			// Frame values must remain intact beside the contiguous axis payload.
			auto expected = GpuD6ConstraintDesc{
				.frame_a = {.rotation = desc.m_frame_a.m_rotation.xyzw, .position = desc.m_frame_a.m_position},
				.frame_b = {.rotation = desc.m_frame_b.m_rotation.xyzw, .position = desc.m_frame_b.m_position},
			};
			for (auto i = 0; i != 6; ++i)
			{
				// Explicit enum mapping does not assume the public enum has the GPU's numeric representation.
				auto const& source = i < 3 ? desc.m_linear[i] : desc.m_angular[i - 3];
				auto& target = expected.axes[i];
				switch (source.m_mode)
				{
					case EConstraintAxisMode::Free:
					{
						// Select the GPU free-axis value independently of the public enum.
						target.mode = GpuConstraintAxisMode_Free;
						break;
					}
					case EConstraintAxisMode::Locked:
					{
						// Select the GPU locked-axis value independently of the public enum.
						target.mode = GpuConstraintAxisMode_Locked;
						break;
					}
					case EConstraintAxisMode::Limited:
					{
						// Select the GPU limited-axis value independently of the public enum.
						target.mode = GpuConstraintAxisMode_Limited;
						break;
					}
					case EConstraintAxisMode::Driven:
					{
						// Select the GPU driven-axis value independently of the public enum.
						target.mode = GpuConstraintAxisMode_Driven;
						break;
					}
					default:
					{
						// Reject test inputs outside the public axis domain.
						throw std::invalid_argument("Unknown axis mode in explicit packing oracle");
					}
				}
				target.lower_limit = source.m_limits.m_beg;
				target.upper_limit = source.m_limits.m_end;
				target.target_position = source.m_target_position;
				target.target_velocity = source.m_target_velocity;
				target.stiffness = source.m_stiffness;
				target.damping = source.m_damping;
				target.max_force = source.m_max_force;
			}
			return expected;
		}

		// Every axis position must preserve every mode and distinct finite fields in active and disabled descriptors.
		PRUnitTestMethod(CanonicalFiniteDescriptorBytes, Quick)
		{
			// Vary frames and every scalar so an offset, stride, or linear/angular ordering error changes the packed bytes.
			auto body = MakeConstraintGpuBody();
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto constraints = ConstraintSet{};
			auto handles = std::array<ConstraintHandle, 4>{};
			auto const modes = std::array{EConstraintAxisMode::Free, EConstraintAxisMode::Locked, EConstraintAxisMode::Limited, EConstraintAxisMode::Driven};
			for (auto variant = 0; variant != 4; ++variant)
			{
				// Rotate the mode sequence so each of the six canonical positions sees all four public modes.
				auto const variant_f = static_cast<float>(variant);
				auto desc = D6ConstraintDesc{
					.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body), m4x4::TransformDeg(10.0f + 5.0f * variant_f, 20, -30, v4{1.25f, -2.5f, 3.75f + variant, 1})),
					.m_frame_b = BodyFrame::FromMatrix(BodyRef::World(), m4x4::TransformDeg(-15, 25.0f + 10.0f * variant_f, 40, v4{-4.5f, 5.25f + variant, -6.75f, 1})),
					.m_break_force = 100.0f + variant,
					.m_break_torque = 200.0f + variant,
					.m_collide_connected = true,
					.m_enabled = variant % 2 == 0,
				};
				for (auto i = 0; i != 6; ++i)
				{
					// Distinct values also expose accidental swaps between adjacent float fields.
					auto& axis = i < 3 ? desc.m_linear[i] : desc.m_angular[i - 3];
					auto const value = 10.0f * (variant + 1) + i;
					axis = ConstraintAxisDesc{
						.m_mode = modes[(variant + i) % 4],
						.m_limits = {-value - 0.5f, value + 0.75f},
						.m_target_position = 0.125f * value,
						.m_target_velocity = -0.375f * value,
						.m_stiffness = 0.25f * value,
						.m_damping = 0.5f * value,
						.m_max_force = 1000.0f + value,
					};
				}
				handles[variant] = constraints.Add(desc);
			}

			// Observe the actual owning packer, including both frames and all 192 axis bytes in each descriptor.
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			PR_EXPECT(upload.m_descriptors.size() == handles.size());
			for (size_t i = 0; i != handles.size(); ++i)
			{
				// The scalar reference reads the stored descriptor after normal caller validation.
				auto const& desc = constraints.Get(handles[i]);
				auto const expected = ExplicitDescriptor(desc);
				PR_EXPECT(std::memcmp(&upload.m_descriptors.at(i), &expected, sizeof(expected)) == 0);
				PR_EXPECT(AllSet(upload.m_endpoints.at(i).flags, GpuConstraintEndpointFlags_Enabled) == desc.m_enabled);
				PR_EXPECT(upload.m_endpoints.at(i).break_force == desc.m_break_force && upload.m_endpoints.at(i).break_torque == desc.m_break_torque);
			}
			log_stream() << std::format("finite_descriptors=4 canonical_axes=24 cpu_axis_bytes={} gpu_axis_bytes={} range_bytes={} three_axis_bytes={} descriptor_bytes={}\n", sizeof(ConstraintAxisDesc), sizeof(GpuConstraintAxisDesc), sizeof(Range<float>), sizeof(std::array<ConstraintAxisDesc, 3>), sizeof(GpuD6ConstraintDesc));
		}

		// Preserve signed zero, finite extremes, and valid unbounded limits/force bounds without scalar arithmetic.
		PRUnitTestMethod(SignedZeroAndInfiniteBoundBytes, Quick)
		{
			// Ordered one-sided and unbounded intervals are valid public inputs, including for Limited axes.
			auto body = MakeConstraintGpuBody();
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const infinity = std::numeric_limits<float>::infinity();
			auto const negative_zero = std::bit_cast<float>(uint32_t{0x80000000});
			auto desc = D6ConstraintDesc{
				.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body), m4x4::TransformDeg(35, -20, 15, v4{-1.25f, 2.5f, -3.75f, 1})),
				.m_frame_b = BodyFrame::FromMatrix(BodyRef::World(), m4x4::TransformDeg(-25, 30, -10, v4{4.5f, -5.25f, 6.75f, 1})),
			};
			for (auto i = 0; i != 6; ++i)
			{
				// Alternating zero signs distinguish the linear and angular blocks even when their numeric values compare equal.
				auto& axis = i < 3 ? desc.m_linear[i] : desc.m_angular[i - 3];
				axis = ConstraintAxisDesc{
					.m_mode = EConstraintAxisMode::Limited,
					.m_limits = {i % 3 == 0 ? -infinity : negative_zero, i % 3 == 2 ? 0.0f : infinity},
					.m_target_position = i % 2 == 0 ? 0.0f : negative_zero,
					.m_target_velocity = i % 2 == 0 ? negative_zero : 0.0f,
					.m_stiffness = negative_zero,
					.m_damping = 0.0f,
					.m_max_force = i % 2 == 0 ? infinity : negative_zero,
				};
			}
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto expected = ExplicitDescriptor(constraints.Get(handle));
			PR_EXPECT(std::memcmp(&upload.m_descriptors.at(0), &expected, sizeof(expected)) == 0);
			PR_EXPECT(std::bit_cast<uint32_t>(upload.m_descriptors[0].axes[0].target_velocity) == 0x80000000U);
			PR_EXPECT(std::bit_cast<uint32_t>(upload.m_descriptors[0].axes[3].target_velocity) == 0U);
			PR_EXPECT(std::bit_cast<uint32_t>(upload.m_descriptors[0].axes[0].lower_limit) == 0xff800000U);
			PR_EXPECT(std::bit_cast<uint32_t>(upload.m_descriptors[0].axes[0].max_force) == 0x7f800000U);

			// A later validated update exercises the same packed slot with the largest and smallest finite float values.
			for (auto i = 0; i != 6; ++i)
			{
				// Packing has no narrower numeric domain than the public descriptor's validation.
				auto& axis = i < 3 ? desc.m_linear[i] : desc.m_angular[i - 3];
				axis.m_limits = {-std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
				axis.m_target_position = i % 2 == 0 ? std::numeric_limits<float>::max() : std::numeric_limits<float>::min();
				axis.m_target_velocity = i % 2 == 0 ? -std::numeric_limits<float>::max() : std::numeric_limits<float>::denorm_min();
				axis.m_stiffness = std::numeric_limits<float>::min();
				axis.m_damping = std::numeric_limits<float>::max();
				axis.m_max_force = std::numeric_limits<float>::max();
			}
			constraints.Update(handle, desc);
			upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			expected = ExplicitDescriptor(constraints.Get(handle));
			PR_EXPECT(std::memcmp(&upload.m_descriptors.at(0), &expected, sizeof(expected)) == 0);
			log_stream() << "signed_zero_bits_preserved=1 infinite_limits_and_forces_preserved=1 finite_extremes_match_scalar_oracle=1\n";
		}
	};

	// Compare narrow target replacement with full descriptor updates, including native persistent-state behavior.
	PRUnitTestClass(ConstraintTargetVelocityTests, Serial)
	{
		// Apply the same six values through the existing full-descriptor boundary.
		static void FullUpdate(ConstraintSet& constraints, ConstraintHandle handle, std::array<float, 6> const& values)
		{
			// Retain the original descriptor-copy operation as the independent mutation reference.
			auto desc = constraints.Get(handle);
			for (auto axis = 0; axis != 3; ++axis)
			{
				// Replace the canonical linear and angular slots without altering their other parameters.
				desc.m_linear[axis].m_target_velocity = values[axis];
				desc.m_angular[axis].m_target_velocity = values[3 + axis];
			}
			constraints.Update(handle, desc);
		}

		// Compare the packed rigid-slot contract; source pointers intentionally belong to different sets.
		static void ExpectPacked(GpuConstraintUpload const& actual, GpuConstraintUpload const& expected)
		{
			// Packed structures have an explicit GPU layout, unlike potentially padded CPU descriptors.
			PR_EXPECT(actual.m_descriptors.size() == expected.m_descriptors.size());
			PR_EXPECT(actual.m_endpoints.size() == expected.m_endpoints.size());
			for (size_t i = 0; i != std::min(actual.m_descriptors.size(), expected.m_descriptors.size()); ++i)
			{
				// Compare all descriptor fields and endpoint flags, not merely the six new targets.
				PR_EXPECT(std::memcmp(&actual.m_descriptors[i], &expected.m_descriptors[i], sizeof(GpuD6ConstraintDesc)) == 0);
				PR_EXPECT(std::memcmp(&actual.m_endpoints[i], &expected.m_endpoints[i], sizeof(GpuConstraintEndpoint)) == 0);
			}
			PR_EXPECT(actual.m_endpoint_identities == expected.m_endpoint_identities);
			PR_EXPECT(actual.m_topology_revision == expected.m_topology_revision);
			PR_EXPECT(actual.m_parameter_revision == expected.m_parameter_revision);
			PR_EXPECT(actual.m_rigid_active_count == expected.m_rigid_active_count);
			PR_EXPECT(actual.m_coupled_active_count == expected.m_coupled_active_count);
			PR_EXPECT(actual.m_breakable_count == expected.m_breakable_count);
			PR_EXPECT(actual.m_collision_exclusions.m_count == expected.m_collision_exclusions.m_count);
		}

		// Preserve the full packed descriptor and Update's revision/dirty semantics for changed and identical inputs.
		PRUnitTestMethod(PackedDescriptorAndRevisionEquivalence, Quick)
		{
			// Nondefault frames and axis parameters expose accidental replacement of fields outside the six targets.
			auto body = MakeConstraintGpuBody();
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const remap = BodyRemap(body_ptrs);
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body), m4x4::TransformDeg(10, 20, 30, v4{1, 2, 3, 1}));
			desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::World(), m4x4::TransformDeg(-20, 15, 5, v4{-2, 1, 4, 1}));
			desc.m_break_force = 20.75f;
			desc.m_break_torque = 21.5f;
			desc.m_collide_connected = true;
			auto const modes = std::array{EConstraintAxisMode::Free, EConstraintAxisMode::Locked, EConstraintAxisMode::Limited, EConstraintAxisMode::Driven, EConstraintAxisMode::Free, EConstraintAxisMode::Driven};
			for (auto i = 0; i != 6; ++i)
			{
				// Cover every axis mode while retaining distinct limits, drive terms, and force bounds.
				auto& axis = i < 3 ? desc.m_linear[i] : desc.m_angular[i - 3];
				axis.m_mode = modes[i];
				axis.m_limits = {-0.75f, 1.25f};
				axis.m_target_position = 0.1f * i;
				axis.m_target_velocity = -0.25f * i;
				axis.m_stiffness = 2.0f + i;
				axis.m_damping = 3.0f + i;
				axis.m_max_force = 10.0f + i;
			}
			auto narrow = ConstraintSet{};
			narrow.Add(desc);
			auto const handle = narrow.Add(desc);
			auto full = narrow;
			auto const values = std::array{
				std::array<float, 6>{1, 2, 3, 4, 5, 6},
				std::array<float, 6>{-0.0f, std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), 7.25f, -8.5f, 9.75f},
				std::array<float, 6>{-0.0f, std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), 7.25f, -8.5f, 9.75f},
			};

			// Inactive descriptors and identical values use the same parameter-update contract as ordinary active changes.
			for (auto enabled : {true, false})
			{
				// Match activation state before each sequence; the second slot alone is changed.
				narrow.SetEnabled(handle, enabled);
				full.SetEnabled(handle, enabled);
				for (auto const& targets : values)
				{
					// The repeated final input must still dirty its slot and advance exactly one parameter revision.
					narrow.ClearDirty();
					full.ClearDirty();
					auto const topology = narrow.TopologyRevision();
					auto const parameters = narrow.ParameterRevision();
					narrow.SetTargetVelocities(handle, targets);
					FullUpdate(full, handle, targets);
					ExpectPacked(PackGpuConstraints(narrow, remap), PackGpuConstraints(full, remap));
					PR_EXPECT(narrow.TopologyRevision() == topology);
					PR_EXPECT(narrow.ParameterRevision() == parameters + 1);
					PR_EXPECT(narrow.Count() == 2 && narrow.CapacitySlots() == 2 && narrow.Contains(handle) && !narrow.IsBroken(handle));
					auto const ranges = narrow.DirtyRanges();
					PR_EXPECT(ranges.size() == 1);
					if (ranges.size() == 1)
					{
						// No unchanged neighboring descriptor is dirtied.
						PR_EXPECT(ranges[0].m_begin == handle.m_index && ranges[0].m_end == handle.m_index + 1);
					}
				}
			}
		}

		// Reject every nonfinite input position and invalid handle without partially updating values or bookkeeping.
		PRUnitTestMethod(InvalidRequestsAreAtomic, Quick)
		{
			// Reuse a removed slot so a stale handle is distinct from an out-of-range or default handle.
			auto body = MakeConstraintGpuBody();
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const remap = BodyRemap(body_ptrs);
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			auto constraints = ConstraintSet{};
			auto const stale = constraints.Add(desc);
			constraints.Remove(stale);
			auto const handle = constraints.Add(desc);
			constraints.ClearDirty();
			auto const original = PackGpuConstraints(constraints, remap);
			auto const invalid_handles = std::array{ConstraintHandle{}, stale, ConstraintHandle{.m_index = 1000, .m_generation = handle.m_generation}};
			for (auto invalid_handle : invalid_handles)
			{
				// Rejected identities cannot modify the replacement occupying the same slot.
				auto const values = std::array<float, 6>{101, 102, 103, 104, 105, 106};
				PR_THROWS(constraints.SetTargetVelocities(invalid_handle, values), std::invalid_argument);
				ExpectPacked(PackGpuConstraints(constraints, remap), original);
				PR_EXPECT(constraints.DirtyRanges().empty());
			}

			// Earlier valid positions differ from stored values, so a late invalid value detects partial mutation.
			for (auto invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
			{
				// Exercise all six canonical positions for each nonfinite category.
				for (auto i = 0; i != 6; ++i)
				{
					// Neither descriptor bytes nor either revision may change on rejection.
					auto values = std::array<float, 6>{101, 102, 103, 104, 105, 106};
					values[i] = invalid;
					PR_THROWS(constraints.SetTargetVelocities(handle, values), std::invalid_argument);
					ExpectPacked(PackGpuConstraints(constraints, remap), original);
					PR_EXPECT(constraints.DirtyRanges().empty());
				}
			}
		}

		// Preserve native warm retention/reset decisions and the CPU broken latch without a new solver inspection seam.
		PRUnitTestMethod(WarmStartAndBrokenStateEquivalence, Extended)
		{
			// Seed identical nonzero native impulses in two independently owned persistent solvers.
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const remap = BodyRemap(body_ptrs);
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto narrow = ConstraintSet{};
			auto const handle = narrow.Add(desc);
			auto full = narrow;
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.constraint_warm_start_factor = 0.85f;
			auto narrow_solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto full_solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto actual = std::array{PackDynamics(body, 0)};
			auto expected = actual;
			auto actual_rows = std::array<GpuConstraintRow, GpuConstraintRowsPerBlock>{};
			auto expected_rows = actual_rows;
			narrow_solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(narrow, remap), actual, {}, actual_rows);
			full_solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(full, remap), expected, {}, expected_rows);
			PR_EXPECT(std::abs(actual_rows[0].bounds.z) > 1.0e-4f);

			// With no solve iterations, the second frame exposes retained warm impulses and the third exposes their reset.
			config.solver_iterations = 0;
			body.VelocityWS(v8motion{});
			for (auto changed : {false, true})
			{
				// Equal targets retain warm state; changed targets must take the same downstream reset path as full Update.
				auto values = std::array<float, 6>{};
				values[0] = changed ? 2.0f : 0.0f;
				narrow.SetTargetVelocities(handle, values);
				FullUpdate(full, handle, values);
				actual[0] = expected[0] = PackDynamics(body, 0);
				narrow_solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(narrow, remap), actual, {}, actual_rows);
				full_solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(full, remap), expected, {}, expected_rows);
				PR_EXPECT(std::memcmp(actual.data(), expected.data(), sizeof(actual)) == 0);
				PR_EXPECT(std::memcmp(actual_rows.data(), expected_rows.data(), sizeof(actual_rows)) == 0);
				PR_EXPECT(changed ? std::abs(actual[0].momentum_lin.x) < 1.0e-6f : std::abs(actual[0].momentum_lin.x) > 1.0e-4f);
				log_stream() << std::format("target_changed={} native_momentum_x={} full_update_momentum_x={}\n", changed, actual[0].momentum_lin.x, expected[0].momentum_lin.x);
			}

			// Obtain a real broken latch from Engine's completed overload readback, not a manufactured private slot state.
			auto shape = collision::ShapeSphere{0.25f};
			auto break_body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			break_body.VelocityWS(v8motion{v4::Zero(), v4{5, 0, 0, 0}});
			desc.m_frame_a.m_body = BodyRef::Rigid(break_body);
			desc.m_break_force = 10.0f;
			auto broken = ConstraintSet{};
			auto const broken_handle = broken.Add(desc);
			auto break_bodies = std::array<RigidBody*, 1>{&break_body};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Step(Engine::StepInput{.m_bodies = break_bodies, .m_constraints = &broken, .m_elapsed_seconds = 1.0f / 30.0f, .m_substep_count = 4});
			PR_EXPECT(broken.IsBroken(broken_handle));
			auto broken_reference = broken;
			auto const values = std::array<float, 6>{1, 2, 3, 4, 5, 6};
			broken.SetTargetVelocities(broken_handle, values);
			FullUpdate(broken_reference, broken_handle, values);
			auto const packed = PackGpuConstraints(broken, BodyRemap(break_bodies));
			ExpectPacked(packed, PackGpuConstraints(broken_reference, BodyRemap(break_bodies)));
			PR_EXPECT(broken.IsBroken(broken_handle) && broken_reference.IsBroken(broken_handle));
			PR_EXPECT(broken.Get(broken_handle).m_enabled && packed.m_rigid_active_count == 0);
			PR_EXPECT(!AllSet(packed.m_endpoints[0].flags, GpuConstraintEndpointFlags_Enabled));
			log_stream() << "broken_after_target_update=1 packed_active=0\n";
			ResetEngineForNextTest(engine);
		}
	};

	// Empty uploads must not transfer retained runtime state between independently owned constraint sets.
	PRUnitTestClass(ConstraintUploadProvenanceTests, Serial)
	{
		// A replacement source must not inherit a broken block merely because it was submitted empty first.
		PRUnitTestMethod(EmptySourceTransitionClearsBrokenSlots, Quick)
		{
			// Break a finite-force world anchor while retaining its GPU block in the solver.
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{5, 0, 0, 0}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			desc.m_break_force = 10.0f;
			auto source_a = ConstraintSet{};
			auto source_b = ConstraintSet{};
			source_a.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto bodies = std::array{PackDynamics(body, 0)};
			auto blocks = std::array<GpuConstraintBlock, 1>{};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(source_a, BodyRemap(body_ptrs)), bodies, blocks);
			PR_EXPECT(AllSet(blocks[0].flags, ConstraintBlockFlags_Broken));

			// The empty submission owns no blocks. Its subsequent identical slot is a new source, not a repair of source A.
			PR_EXPECT(!solver.Upload(ConstraintTestGpu().m_job, PackGpuConstraints(source_b, BodyRemap(body_ptrs))));
			source_b.Add(desc);
			body.VelocityWS(v8motion{v4::Zero(), v4{0.01f, 0, 0, 0}});
			bodies[0] = PackDynamics(body, 0);
			auto const replacement = PackGpuConstraints(source_b, BodyRemap(body_ptrs));
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, replacement, bodies, blocks);

			// A cold solver independently proves that this small load is supported rather than broken.
			auto cold_solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto cold_bodies = std::array{PackDynamics(body, 0)};
			auto cold_blocks = std::array<GpuConstraintBlock, 1>{};
			cold_solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, replacement, cold_bodies, cold_blocks);
			log_stream() << std::format("empty_source_transition flags={} cold_flags={} momentum_x={} cold_momentum_x={}\n", blocks[0].flags, cold_blocks[0].flags, bodies[0].momentum_lin.x, cold_bodies[0].momentum_lin.x);
			PR_EXPECT(!AllSet(cold_blocks[0].flags, ConstraintBlockFlags_Broken));
			PR_EXPECT(Abs(cold_bodies[0].momentum_lin.x) < 1.0e-6f);
			PR_EXPECT(!AllSet(blocks[0].flags, ConstraintBlockFlags_Broken));
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin, cold_bodies[0].momentum_lin, 1.0e-6f));
		}
	};

	// Observe actual descriptor-copy envelopes and runtime-reset ranges independently of shader compilation.
	PRUnitTestClass(ConstraintUploadBookkeepingTests, Serial)
	{
		// Completed GPU contents immediately after Upload, before any solver dispatch can hide a missing write.
		struct Snapshot
		{
			std::vector<GpuConstraintEndpoint> m_endpoints;
			std::vector<GpuD6ConstraintDesc> m_descriptors;
			std::vector<GpuConstraintBlock> m_blocks;
			std::vector<GpuConstraintRow> m_rows;
			std::vector<GpuConstraintBreakState> m_breaks;
		};

		RigidBody m_body = MakeConstraintGpuBody();
		RigidBody m_other_body = MakeConstraintGpuBody();
		ConstraintSet m_source_a;
		ConstraintSet m_source_b;
		EngineConfig m_config;
		GpuConstraintSolver m_solver{ConstraintTestGpu(), m_config};
		GpuConstraintUpload m_upload;
		std::vector<GpuConstraintBlock> m_seed_blocks;
		std::vector<GpuConstraintRow> m_seed_rows;

		// Build seven stable synthetic slots with valid endpoint identities and finite break thresholds.
		TestClass_ConstraintUploadBookkeepingTests()
		{
			// Packed streams are the tested boundary; their source pointers identify independent owners.
			m_upload.m_source = &m_source_a;
			for (int index = 0; index != 7; ++index)
			{
				// Distinct descriptor bytes and generations expose misplaced copies and latch initialization.
				m_upload.m_endpoints.push_back(GpuConstraintEndpoint{
					.body_idx_a = 0,
					.body_idx_b = -1,
					.flags = GpuConstraintEndpointFlags_Enabled,
					.generation = static_cast<uint32_t>(index + 1),
					.break_force = 100.0f,
					.break_torque = 100.0f,
				});
				m_upload.m_endpoint_identities.push_back({BodyRef::Rigid(m_body), BodyRef::World()});
				auto& descriptor = m_upload.m_descriptors.emplace_back();
				descriptor.frame_a.rotation = float4{0, 0, 0, 1};
				descriptor.frame_b.rotation = float4{0, 0, 0, 1};
				descriptor.frame_b.position.x = static_cast<float>(index);
				descriptor.axes[0].mode = GpuConstraintAxisMode_Locked;
				descriptor.axes[0].max_force = 1000.0f;
			}
			Capture();
		}

		// Queue a direct buffer write; no shader reads these diagnostic sentinels.
		template <typename T> static void WriteBuffer(ID3D12Resource* resource, std::vector<T> const& values)
		{
			// The production job supplies the same upload allocator and transition tracker as Upload.
			auto& job = ConstraintTestGpu().m_job;
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<T>(isize(values));
			memcpy(allocation.ptr<T>(), values.data(), values.size() * sizeof(T));
			job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
		}

		// Queue completed-buffer inspection in the same native transaction as the upload.
		template <typename T> static ReadbackAlloc ReadBuffer(ID3D12Resource* resource, int count)
		{
			// Read directly rather than running Prepare, which would recompile and overwrite rows.
			auto& job = ConstraintTestGpu().m_job;
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			auto allocation = job.m_readback.Alloc<T>(count);
			job.m_cmd_list.CopyBufferRegion(allocation, resource, 0);
			return allocation;
		}

		// Execute Upload and return its actual completed writes, including frame-local break latches.
		Snapshot Capture()
		{
			// All synthetic descriptors have finite thresholds; disabled slots are excluded from active counts.
			auto const slot_count = isize(m_upload.m_endpoints);
			m_upload.m_rigid_active_count = std::ranges::count_if(m_upload.m_endpoints, [](auto const& endpoint)
			{
				// Only enabled slots contribute independent rigid work.
				return AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled);
			});
			m_upload.m_breakable_count = m_upload.m_rigid_active_count;
			auto& job = ConstraintTestGpu().m_job;
			PR_EXPECT(m_solver.Upload(job, m_upload));
			auto endpoints = ReadBuffer<GpuConstraintEndpoint>(m_solver.m_r_endpoints.get(), slot_count);
			auto descriptors = ReadBuffer<GpuD6ConstraintDesc>(m_solver.m_r_descriptors.get(), slot_count);
			auto blocks = ReadBuffer<GpuConstraintBlock>(m_solver.m_r_blocks.get(), slot_count);
			auto rows = ReadBuffer<GpuConstraintRow>(m_solver.m_r_rows.get(), GpuConstraintRowsPerBlock * slot_count);
			auto breaks = ReadBuffer<GpuConstraintBreakState>(m_solver.BreakOutput().m_states, slot_count);
			job.Run();
			return Snapshot{
				.m_endpoints = {endpoints.ptr<GpuConstraintEndpoint>(), endpoints.ptr<GpuConstraintEndpoint>() + slot_count},
				.m_descriptors = {descriptors.ptr<GpuD6ConstraintDesc>(), descriptors.ptr<GpuD6ConstraintDesc>() + slot_count},
				.m_blocks = {blocks.ptr<GpuConstraintBlock>(), blocks.ptr<GpuConstraintBlock>() + slot_count},
				.m_rows = {rows.ptr<GpuConstraintRow>(), rows.ptr<GpuConstraintRow>() + GpuConstraintRowsPerBlock * slot_count},
				.m_breaks = {breaks.ptr<GpuConstraintBreakState>(), breaks.ptr<GpuConstraintBreakState>() + slot_count},
			};
		}

		// Seed retained GPU bytes without changing host shadows, making every omitted or oversized write observable.
		std::vector<GpuD6ConstraintDesc> SeedGpu()
		{
			// Descriptor sentinels distinguish a real copy of clean interior bytes from merely matching host-side values.
			auto descriptors = m_upload.m_descriptors;
			for (int index = 0; index != isize(descriptors); ++index)
			{
				// Different finite values make copy bounds visible without relying on uninitialized memory.
				descriptors[index].frame_b.position.x = -1000.0f - index;
			}
			WriteBuffer(m_solver.m_r_descriptors.get(), descriptors);

			// Runtime sentinels prove that gaps between reset runs survive even when descriptor copies span them.
			m_seed_blocks.assign(descriptors.size(), GpuConstraintBlock{.body_idx_a = 0, .body_idx_b = -1, .colour = MaxColours, .flags = ConstraintBlockFlags_Active});
			m_seed_rows.assign(descriptors.size() * GpuConstraintRowsPerBlock, GpuConstraintRow{});
			for (int index = 0; index != isize(m_seed_rows); ++index)
			{
				// Each row has a distinct retained impulse so partial or shifted clears cannot pass.
				m_seed_rows[index].bounds.z = 10.0f + index;
			}
			WriteBuffer(m_solver.m_r_blocks.get(), m_seed_blocks);
			WriteBuffer(m_solver.m_r_rows.get(), m_seed_rows);
			return descriptors;
		}

		// Compare exact uploaded bytes, warm-start flags, runtime clears, and break generations.
		void ExpectWrites(Snapshot const& actual, std::vector<GpuD6ConstraintDesc> const& descriptors, std::initializer_list<int> warm_resets, std::initializer_list<int> runtime_resets)
		{
			// Whole-descriptor comparisons also cover padding and untouched fields within the transfer envelope.
			PR_EXPECT(memcmp(actual.m_descriptors.data(), descriptors.data(), descriptors.size() * sizeof(GpuD6ConstraintDesc)) == 0);
			for (int index = 0; index != isize(actual.m_endpoints); ++index)
			{
				// Copying a clean descriptor must not imply semantic invalidation or a runtime clear.
				auto const warm_reset = std::ranges::find(warm_resets, index) != warm_resets.end();
				auto const runtime_reset = std::ranges::find(runtime_resets, index) != runtime_resets.end();
				auto endpoint = m_upload.m_endpoints[index];
				endpoint.flags = SetFlag(endpoint.flags, GpuConstraintEndpointFlags_ResetWarmStart, warm_reset);
				auto const block = runtime_reset ? GpuConstraintBlock{.body_idx_a = -1, .body_idx_b = -1, .colour = MaxColours} : m_seed_blocks[index];
				auto const break_state = GpuConstraintBreakState{.generation = endpoint.generation, .substep_index = -1};
				PR_EXPECT(memcmp(&actual.m_endpoints[index], &endpoint, sizeof(endpoint)) == 0);
				PR_EXPECT(memcmp(&actual.m_blocks[index], &block, sizeof(block)) == 0);
				PR_EXPECT(memcmp(&actual.m_breaks[index], &break_state, sizeof(break_state)) == 0);
				for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
				{
					// Every row in a reset slot clears, while all rows in a retained slot remain byte-identical.
					auto const row_index = index * GpuConstraintRowsPerBlock + axis;
					auto const row = runtime_reset ? GpuConstraintRow{} : m_seed_rows[row_index];
					PR_EXPECT(memcmp(&actual.m_rows[row_index], &row, sizeof(row)) == 0);
				}
			}
		}

		// Clean, sparse, boundary, and shrink uploads preserve descriptor envelopes independently of semantic reset flags.
		PRUnitTestMethod(DescriptorEnvelopeAndExactWarmFlags, Quick)
		{
			// A clean upload must not touch persistent descriptors or runtime.
			auto expected = SeedGpu();
			ExpectWrites(Capture(), expected, {}, {});

			// Two isolated changes copy the clean interior, but neither outside edge.
			expected = SeedGpu();
			m_upload.m_descriptors[2].axes[0].target_velocity += 1.0f;
			m_upload.m_descriptors[4].axes[0].target_velocity += 1.0f;
			std::copy(m_upload.m_descriptors.begin() + 2, m_upload.m_descriptors.begin() + 5, expected.begin() + 2);
			ExpectWrites(Capture(), expected, {2, 4}, {});

			// Independently exercise both end slots and then an envelope covering every slot.
			for (auto index : {0, 6})
			{
				// A single dirty edge must not copy any neighboring sentinel.
				expected = SeedGpu();
				m_upload.m_descriptors[index].axes[0].target_velocity += 1.0f;
				expected[index] = m_upload.m_descriptors[index];
				ExpectWrites(Capture(), expected, {index}, {});
			}
			expected = SeedGpu();
			m_upload.m_descriptors[0].axes[0].target_velocity += 1.0f;
			m_upload.m_descriptors[6].axes[0].target_velocity += 1.0f;
			ExpectWrites(Capture(), m_upload.m_descriptors, {0, 6}, {});

			// A smaller shadow shape seeds the full descriptor copy without resetting equal prefix slots.
			auto const capacity = m_solver.Stats().m_slot_capacity;
			for (auto count : {6, 5})
			{
				// Include a changed interior slot first, then an entirely equal prefix; neither may shrink away the full-copy seed.
				m_upload.m_endpoints.resize(count);
				m_upload.m_endpoint_identities.resize(count);
				m_upload.m_descriptors.resize(count);
				SeedGpu();
				if (count == 6)
					m_upload.m_descriptors[1].axes[0].target_velocity += 1.0f;

				ExpectWrites(Capture(), m_upload.m_descriptors, count == 6 ? std::initializer_list<int>{1} : std::initializer_list<int>{}, {});
				PR_EXPECT(m_solver.Stats().m_slot_capacity == capacity);
			}
			log_stream() << "descriptor copies: clean; [2,5); [0,1); [6,7); [0,7); shrink [0,6) with only slot1 reset; shrink [0,5) with no resets\n";
		}

		// Disjoint activation and acknowledged repair clear only their contiguous runs; endpoint changes invalidate exact warm starts.
		PRUnitTestMethod(SeparateResetRunsAndEndpointSemantics, Quick)
		{
			// Retain gaps between three reset runs, including runs touching both ends of the stream.
			for (auto index : {0, 1, 3, 6})
			{
				// Disable these slots in the previous endpoint shadow.
				m_upload.m_endpoints[index].flags = 0;
			}
			Capture();
			auto expected = SeedGpu();
			for (auto index : {0, 1, 3, 6})
			{
				// Re-activation must clear stale row state and break flags only in these slots.
				m_upload.m_endpoints[index].flags = GpuConstraintEndpointFlags_Enabled;
			}
			ExpectWrites(Capture(), expected, {0, 1, 3, 6}, {0, 1, 3, 6});

			// A body-index remap alone is not a new identity; generation, identity, and threshold changes are semantic.
			expected = SeedGpu();
			for (auto& endpoint : m_upload.m_endpoints)
			{
				// The same body now occupies packed index one.
				endpoint.body_idx_a = 1;
			}
			m_upload.m_endpoints[0].flags |= GpuConstraintEndpointFlags_ResetWarmStart;
			++m_upload.m_endpoints[1].generation;
			m_upload.m_endpoint_identities[2].m_body_a = BodyRef::Rigid(m_other_body);
			m_upload.m_endpoints[2].body_idx_a = 0;
			m_upload.m_endpoints[4].break_force += 1.0f;
			ExpectWrites(Capture(), expected, {0, 1, 2, 4}, {});
			m_upload.m_endpoints[0].flags &= ~GpuConstraintEndpointFlags_ResetWarmStart;

			// An acknowledged break followed by explicit repair must clear the old broken block.
			auto breaks = std::vector<GpuConstraintBreakState>(7);
			breaks[4] = GpuConstraintBreakState{.flags = GpuConstraintBreakFlags_Broken, .generation = m_upload.m_endpoints[4].generation};
			m_solver.AcknowledgeBreaks(breaks);
			expected = SeedGpu();
			ExpectWrites(Capture(), expected, {4}, {4});
			log_stream() << "runtime resets: [0,2), [3,4), [6,7); endpoint semantic flags only at 0/1/2/4; repaired slot [4,5)\n";
		}

		// Source replacement resets runtime without unnecessary descriptor copies; capacity replacement initializes all streams.
		PRUnitTestMethod(SourceAndCapacityReplacement, Quick)
		{
			// Identical descriptors under a new source reuse GPU bytes but never inherit runtime.
			auto expected = SeedGpu();
			m_upload.m_source = &m_source_b;
			ExpectWrites(Capture(), expected, {0, 1, 2, 3, 4, 5, 6}, {0, 1, 2, 3, 4, 5, 6});

			// Growing past retained capacity replaces resources and therefore uploads and resets every slot.
			m_upload.m_endpoints.push_back(m_upload.m_endpoints.back());
			m_upload.m_endpoint_identities.push_back(m_upload.m_endpoint_identities.back());
			m_upload.m_descriptors.push_back(m_upload.m_descriptors.back());
			++m_upload.m_endpoints.back().generation;
			m_upload.m_descriptors.back().frame_b.position.x = 7.0f;
			ExpectWrites(Capture(), m_upload.m_descriptors, {0, 1, 2, 3, 4, 5, 6, 7}, {0, 1, 2, 3, 4, 5, 6, 7});
			PR_EXPECT(m_solver.Stats().m_slot_capacity >= 8);

			// Explicit invalidation remains pending across an empty upload from the same source.
			expected = SeedGpu();
			m_solver.InvalidateRuntimeState();
			PR_EXPECT(!m_solver.Upload(ConstraintTestGpu().m_job, GpuConstraintUpload{.m_source = &m_source_b}));
			ExpectWrites(Capture(), expected, {0, 1, 2, 3, 4, 5, 6, 7}, {0, 1, 2, 3, 4, 5, 6, 7});
			log_stream() << "replacement: source resets all without descriptor copy; capacity growth copies/resets all; invalidation survives empty source\n";
		}
	};

	// Retained CPU graph work must follow exact endpoint keys without changing runtime reset or GPU colouring behavior.
	PRUnitTestClass(ConstraintColourReuseTests, Serial)
	{
		ConstraintSet m_source_a;
		ConstraintSet m_source_b;
		EngineConfig m_config;
		GpuConstraintSolver m_solver{ConstraintTestGpu(), m_config};
		GpuConstraintSolver m_reference{ConstraintTestGpu(), m_config};
		GpuConstraintUpload m_upload;

		// Build a branching packed input that includes a world endpoint.
		TestClass_ConstraintColourReuseTests()
		{
			// Only row preparation and colouring are needed for the native recolouring case.
			m_config.solver_iterations = 0;
			m_config.push_out_iterations = 0;
			m_config.constraint_warm_start_factor = 0.0f;
			m_upload.m_source = &m_source_a;
			for (auto [a, b] : std::array<std::pair<int, int>, 6>{{{0, 1}, {1, 2}, {2, 3}, {0, 4}, {4, 5}, {5, -1}}})
			{
				// Raw packed streams isolate the existing Upload boundary from front-end packing.
				m_upload.m_endpoints.push_back({.body_idx_a = a, .body_idx_b = b, .flags = GpuConstraintEndpointFlags_Enabled, .generation = 1, .break_force = std::numeric_limits<float>::infinity(), .break_torque = std::numeric_limits<float>::infinity()});
				m_upload.m_endpoint_identities.emplace_back();
				auto& desc = m_upload.m_descriptors.emplace_back();
				desc.frame_a.rotation = desc.frame_b.rotation = float4{0, 0, 0, 1};
				desc.axes[0].mode = GpuConstraintAxisMode_Locked;
				desc.axes[0].max_force = 1000.0f;
			}
		}

		// Compare retained CPU work and actual native transfers with a newly built schedule and bound.
		void Check(char const* name, bool rebuild, std::initializer_list<int> warm_resets = {}, std::initializer_list<int> runtime_resets = {})
		{
			// Build scratch is not consumed by rendering or solving; sentinels expose an otherwise invisible redundant CPU rebuild.
			m_solver.m_body_degrees.push_back(-101);
			m_solver.m_colour_schedule.m_unsorted.push_back({.slot_idx = -202});
			auto const retained_slots = isize(m_solver.m_endpoint_shadow);
			if (retained_slots != 0)
			{
				// Only task padding changes, so even an accidental later shader read cannot use an invalid slot or predecessor.
				auto tasks = m_solver.m_colour_schedule.m_tasks;
				for (auto& task : tasks)
				{
					// A repeated transfer would overwrite this distinct GPU-only marker.
					task.pad = 73;
				}
				if (!tasks.empty())
					TestClass_ConstraintUploadBookkeepingTests::WriteBuffer(m_solver.m_r_colour_schedule.get(), tasks);

				// Broken-block and impulse sentinels make any hidden warm/runtime reset observable.
				TestClass_ConstraintUploadBookkeepingTests::WriteBuffer(m_solver.m_r_blocks.get(), std::vector<GpuConstraintBlock>(retained_slots, GpuConstraintBlock{.flags = ConstraintBlockFlags_Broken}));
				auto rows = std::vector<GpuConstraintRow>(GpuConstraintRowsPerBlock * retained_slots);
				for (auto& row : rows)
				{
					// Every retained canonical row carries the same easily checked impulse.
					row.bounds.z = 17.0f;
				}
				TestClass_ConstraintUploadBookkeepingTests::WriteBuffer(m_solver.m_r_rows.get(), rows);
			}

			// The reference always rebuilds from the current input and never uses the candidate's endpoint shadow.
			auto reference = ConstraintColourSchedule{};
			reference.Build(m_upload.m_endpoints);
			auto const bound = m_reference.ColourBound(m_upload.m_endpoints);
			m_upload.m_rigid_active_count = std::ranges::count_if(m_upload.m_endpoints, [](auto const& endpoint)
			{
				// Coupled rows remain owned by their separate solver.
				return AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) && !AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled);
			});
			m_upload.m_breakable_count = std::ranges::count_if(m_upload.m_endpoints, [](auto const& endpoint)
			{
				// Keep the packed count consistent when a test changes a finite threshold.
				return AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) && (std::isfinite(endpoint.break_force) || std::isfinite(endpoint.break_torque));
			});
			auto& job = ConstraintTestGpu().m_job;
			PR_EXPECT(m_solver.Upload(job, m_upload) == (m_upload.m_rigid_active_count != 0));
			auto const count = isize(m_upload.m_endpoints);
			auto const task_count = isize(reference.m_tasks);
			auto task_read = ReadbackAlloc{};
			if (task_count != 0)
				task_read = TestClass_ConstraintUploadBookkeepingTests::ReadBuffer<GpuConstraintColourTask>(m_solver.m_r_colour_schedule.get(), task_count);

			// Complete the real D3D transfers before reading tasks, endpoint flags, or retained runtime.
			auto endpoints = TestClass_ConstraintUploadBookkeepingTests::ReadBuffer<GpuConstraintEndpoint>(m_solver.m_r_endpoints.get(), count);
			auto blocks = TestClass_ConstraintUploadBookkeepingTests::ReadBuffer<GpuConstraintBlock>(m_solver.m_r_blocks.get(), count);
			auto rows = TestClass_ConstraintUploadBookkeepingTests::ReadBuffer<GpuConstraintRow>(m_solver.m_r_rows.get(), GpuConstraintRowsPerBlock * count);
			job.Run();
			PR_EXPECT((std::ranges::find(m_solver.m_body_degrees, -101) == m_solver.m_body_degrees.end()) == rebuild);
			PR_EXPECT((std::ranges::find_if(m_solver.m_colour_schedule.m_unsorted, [](auto const& task)
			{
				// Real schedule tasks always refer to nonnegative stable slots.
				return task.slot_idx == -202;
			}) == m_solver.m_colour_schedule.m_unsorted.end()) == rebuild);
			PR_EXPECT(m_solver.m_colour_count == bound);
			PR_EXPECT(m_solver.m_colour_schedule.m_level_offsets == reference.m_level_offsets);
			PR_EXPECT(m_solver.m_colour_schedule.m_tasks.size() == reference.m_tasks.size());
			for (int index = 0; index != task_count; ++index)
			{
				// Changed graphs must reach native storage; unchanged graphs must leave the GPU-only padding marker intact.
				auto expected = reference.m_tasks[index];
				PR_EXPECT(memcmp(&m_solver.m_colour_schedule.m_tasks[index], &expected, sizeof(expected)) == 0);
				expected.pad = rebuild ? 0 : 73;
				PR_EXPECT(memcmp(&task_read.ptr<GpuConstraintColourTask>()[index], &expected, sizeof(expected)) == 0);
			}
			for (int index = 0; index != count; ++index)
			{
				// The graph decision must not add warm resets or clear previously broken blocks.
				auto const warm = std::ranges::find(warm_resets, index) != warm_resets.end();
				auto const reset = std::ranges::find(runtime_resets, index) != runtime_resets.end();
				PR_EXPECT(AllSet(endpoints.ptr<GpuConstraintEndpoint>()[index].flags, GpuConstraintEndpointFlags_ResetWarmStart) == warm);
				PR_EXPECT(blocks.ptr<GpuConstraintBlock>()[index].flags == (reset ? 0u : ConstraintBlockFlags_Broken));
				for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
				{
					// All rows of a cleared slot reset, and all other rows retain their impulse.
					PR_EXPECT(rows.ptr<GpuConstraintRow>()[index * GpuConstraintRowsPerBlock + axis].bounds.z == (reset ? 0.0f : 17.0f));
				}
			}
			log_stream() << name << " rebuilt=" << rebuild << " slots=" << count << " tasks=" << task_count << " levels=" << reference.LevelCount() << " bound=" << bound << " gpu_tasks_and_runtime_match=1\n";
		}

		// Non-key parameters and repeated endpoints must not rebuild or retransmit the graph.
		PRUnitTestMethod(UnchangedAndParameterOnly, Quick)
		{
			// Both CPU scratch and GPU-only markers must survive each non-key mutation.
			Check("initial", true, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5});
			Check("unchanged", false);
			m_upload.m_descriptors[1].axes[0].target_velocity = 0.25f;
			Check("target", false, {1});
			m_upload.m_endpoints[2].break_force = 100.0f;
			Check("threshold", false, {2});
			++m_upload.m_endpoints[3].generation;
			Check("generation", false, {3});
			m_upload.m_endpoints[4].flags |= GpuConstraintEndpointFlags_CollideConnected;
			Check("collision-flag", false, {4});
			m_upload.m_endpoints[5].flags |= GpuConstraintEndpointFlags_ResetWarmStart;
			Check("input-warm-flag", false, {5});
			m_upload.m_endpoints[5].flags &= ~GpuConstraintEndpointFlags_ResetWarmStart;
			m_solver.Deactivate();
			Check("deactivated-resume", false);
		}

		// Every graph-key dimension and storage replacement must refresh the existing native schedule.
		PRUnitTestMethod(GraphKeysAndStorageLifecycle, Quick)
		{
			// Packed reordering changes indices without changing semantic endpoint identities or warm starts.
			Check("initial", true, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5});
			for (auto& endpoint : m_upload.m_endpoints)
			{
				// Apply the same body permutation to both ends of every slot.
				for (auto* index : {&endpoint.body_idx_a, &endpoint.body_idx_b})
				{
					// A consistent permutation leaves physical endpoint identity untouched.
					if (*index == 0) *index = 5;
					else if (*index == 5) *index = 0;
				}
			}
			Check("packed-reorder", true);
			std::swap(m_upload.m_endpoints[0], m_upload.m_endpoints[3]);
			++m_upload.m_endpoints[0].generation;
			++m_upload.m_endpoints[3].generation;
			Check("slot-reorder", true, {0, 3});
			m_upload.m_endpoints[2].flags &= ~GpuConstraintEndpointFlags_Enabled;
			Check("disabled", true, {2});
			m_upload.m_endpoints[2].body_idx_a = 5;
			Check("disabled-index-a-bound", true);
			m_upload.m_endpoints[2].body_idx_b = 1;
			Check("disabled-index-b-bound", true);
			m_upload.m_endpoints[1].flags |= GpuConstraintEndpointFlags_Coupled;
			Check("coupled", true, {1});
			m_upload.m_endpoints[1].flags &= ~GpuConstraintEndpointFlags_Coupled;
			Check("uncoupled", true, {1});

			// A built empty graph has offsets {0}; it is distinct from an invalidated schedule.
			for (auto& endpoint : m_upload.m_endpoints)
			{
				// Keep slot shape and indices while removing all scheduled tasks.
				endpoint.flags &= ~GpuConstraintEndpointFlags_Enabled;
			}
			Check("built-empty", true, {0, 1, 3, 4, 5});
			Check("built-empty-reuse", false);
			for (auto& endpoint : m_upload.m_endpoints)
			{
				// Restore active tasks without treating the empty schedule as unbuilt.
				endpoint.flags |= GpuConstraintEndpointFlags_Enabled;
			}
			Check("reactivated", true, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5});
			auto full = m_upload;
			m_upload.m_endpoints.resize(5);
			m_upload.m_endpoint_identities.resize(5);
			m_upload.m_descriptors.resize(5);
			Check("shrink", true);
			m_upload = full;
			Check("grow-within-capacity", true, {5}, {5});
			m_upload.m_endpoints.push_back(m_upload.m_endpoints.back());
			m_upload.m_endpoint_identities.emplace_back();
			m_upload.m_descriptors.push_back(m_upload.m_descriptors.back());
			Check("capacity-replacement", true, {0, 1, 2, 3, 4, 5, 6}, {0, 1, 2, 3, 4, 5, 6});
			PR_EXPECT(!m_solver.Upload(ConstraintTestGpu().m_job, GpuConstraintUpload{.m_source = &m_source_b}));
			m_upload.m_source = &m_source_b;
			Check("source-through-empty", true, {0, 1, 2, 3, 4, 5, 6}, {0, 1, 2, 3, 4, 5, 6});
			PR_EXPECT(!m_solver.Upload(ConstraintTestGpu().m_job, GpuConstraintUpload{.m_source = &m_source_a}));
			Check("retained-owner-through-empty", false);
			m_solver.InvalidateRuntimeState();
			PR_EXPECT(!m_solver.Upload(ConstraintTestGpu().m_job, GpuConstraintUpload{.m_source = &m_source_b}));
			Check("invalidated-through-empty", true, {0, 1, 2, 3, 4, 5, 6}, {0, 1, 2, 3, 4, 5, 6});
		}

		// Acknowledgement changes the shadow before Upload can compare it with a disabled or repaired stream.
		PRUnitTestMethod(AcknowledgedBreakAndRepair, Quick)
		{
			// Disabled upload deliberately matches the already-mutated shadow, exposing a stale retained graph.
			Check("initial", true, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5});
			auto breaks = std::vector<GpuConstraintBreakState>(6);
			breaks[1] = {.flags = GpuConstraintBreakFlags_Broken, .generation = 1};
			m_solver.AcknowledgeBreaks(breaks);
			m_upload.m_endpoints[1].flags &= ~GpuConstraintEndpointFlags_Enabled;
			Check("acknowledged-disabled", true);
			m_solver.AcknowledgeBreaks(breaks);
			Check("repeated-disabled-ack", false);
			m_upload.m_endpoints[1].flags |= GpuConstraintEndpointFlags_Enabled;
			Check("repair", true, {1}, {1});

			// An empty stream cannot erase graph invalidation while the last nonempty shadow remains retained.
			m_solver.AcknowledgeBreaks(breaks);
			PR_EXPECT(!m_solver.Upload(ConstraintTestGpu().m_job, GpuConstraintUpload{.m_source = &m_source_b}));
			m_upload.m_endpoints[1].flags &= ~GpuConstraintEndpointFlags_Enabled;
			Check("acknowledged-empty-disabled", true);
			m_upload.m_endpoints[1].flags |= GpuConstraintEndpointFlags_Enabled;
			Check("repair-after-empty", true, {1}, {1});
			m_solver.AcknowledgeBreaks(breaks);
			Check("immediate-repair", true, {1}, {1});
		}

		// Identical CPU graph inputs must still run GPU colouring when current body dynamics change.
		PRUnitTestMethod(NativeColouringStillRuns, Quick)
		{
			// Static participation changes native colours, but neither the schedule nor conservative bound depends on mass.
			Check("initial", true, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5});
			auto previous = std::vector<uint32_t>{};
			for (auto static_middle : {false, true})
			{
				// The reference invalidates every time; the candidate must keep its CPU graph and still match its new native colours.
				auto bodies = std::vector<GpuRigidBody>{};
				for (int index = 0; index != 6; ++index)
				{
					// Fresh identical body state isolates colour generation from earlier solving.
					bodies.push_back(PackDynamics(MakeConstraintGpuBody(), index));
				}
				if (static_middle)
					bodies[1].os_com_and_invmass.w = 0.0f;

				// A scratch marker detects CPU rebuilding independently of the completed block colours.
				m_solver.m_body_degrees.push_back(-101);
				auto expected_bodies = bodies;
				auto blocks = std::vector<GpuConstraintBlock>(6);
				auto expected_blocks = blocks;
				m_reference.InvalidateRuntimeState();
				m_solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, m_upload, bodies, blocks);
				m_reference.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, m_upload, expected_bodies, expected_blocks);
				PR_EXPECT(std::ranges::find(m_solver.m_body_degrees, -101) != m_solver.m_body_degrees.end());
				PR_EXPECT(m_solver.Stats().m_dispatch_count > 0);
				auto colours = std::vector<uint32_t>{};
				for (int index = 0; index != 6; ++index)
				{
					// Match the freshly rebuilt solver's actual shader output at every stable slot.
					PR_EXPECT(blocks[index].colour == expected_blocks[index].colour);
					colours.push_back(blocks[index].colour);
				}
				if (static_middle)
					PR_EXPECT(colours != previous);

				// Keep both native vectors in the evidence instead of only a boolean success.
				log_stream() << "static_middle=" << static_middle << " native_colours=";
				for (auto colour : colours)
				{
					// Slot order is preserved in the retained output.
					log_stream() << colour << ',';
				}
				log_stream() << '\n';
				previous = colours;
			}
		}
	};

	// The recording bound must cover native serial-order colouring, including its overflow fallback.
	PRUnitTestClass(ConstraintColourBoundTests, Serial)
	{
		// Compare native colours and indirect arguments with an independent serial visit of the completed blocks.
		std::pair<int, int> Check(char const* name, std::vector<std::pair<int, int>> const& edges, int static_body = -1, std::span<int const> disabled = {}, std::span<int const> coupled = {})
		{
			// Exercise the per-colour recorder without changing the engine or benchmark configuration.
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_warm_start_factor = 0.0f;
			config.fused_colour_sweep_limit = 0;
			auto& gpu = ConstraintTestGpu();
			auto solver = GpuConstraintSolver{gpu, config};
			auto upload = GpuConstraintUpload{};
			auto body_count = 0;
			for (auto [a, b] : edges)
			{
				// Raw packed endpoints isolate colouring from the front-end constraint builder.
				body_count = std::max(body_count, std::max(a, b) + 1);
				upload.m_endpoints.push_back({.body_idx_a = a, .body_idx_b = b, .flags = GpuConstraintEndpointFlags_Enabled});
				upload.m_endpoint_identities.emplace_back();
				auto& descriptor = upload.m_descriptors.emplace_back();
				descriptor.frame_a.rotation = descriptor.frame_b.rotation = float4{0, 0, 0, 1};
				descriptor.axes[0].mode = GpuConstraintAxisMode_Locked;
				descriptor.axes[0].max_force = 1000.0f;
			}
			for (auto slot : disabled)
			{
				// Disabled slots still participate conservatively in the host bound.
				upload.m_endpoints[slot].flags = GpuConstraintEndpointFlags_None;
			}
			for (auto slot : coupled)
			{
				// Coupled slots belong to a different solver and cannot consume rigid colours.
				upload.m_endpoints[slot].flags |= GpuConstraintEndpointFlags_Coupled;
			}
			upload.m_rigid_active_count = std::ranges::count_if(upload.m_endpoints, [](auto const& endpoint)
			{
				// Match the packed stream's active independent-rigid count.
				return AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) && !AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled);
			});
			auto bodies = std::vector<GpuRigidBody>{};
			for (int body = 0; body != body_count; ++body)
			{
				// Static participation differs from the conservative host assumption.
				bodies.push_back(PackDynamics(MakeConstraintGpuBody(), body));
				if (body == static_body)
					bodies.back().os_com_and_invmass.w = 0.0f;
			}

			// Read actual shader output, not the C++ shader emulation.
			auto blocks = std::vector<GpuConstraintBlock>(edges.size());
			solver.Solve(gpu.m_job, 1.0f / 60.0f, upload, bodies, blocks);
			auto const bound = solver.m_colour_count;
			auto masks = std::vector<uint32_t>(body_count, 0u);
			auto expected = std::vector<uint32_t>(edges.size(), MaxColours);
			auto overflow = false;
			auto used_count = 0;
			auto dynamic = [&](int body)
			{
				// World and static endpoints do not restrict the first free colour.
				return body >= 0 && body != static_body;
			};
			for (int slot = 0; slot != isize(edges); ++slot)
			{
				// Only participating rows can occupy a colour at an earlier slot.
				auto const& block = blocks[slot];
				if (!AllSet(block.flags, ConstraintBlockFlags_Active) || block.velocity_mask == 0u || AllSet(upload.m_endpoints[slot].flags, GpuConstraintEndpointFlags_Coupled))
					continue;

				auto const [a, b] = edges[slot];
				auto const used = (dynamic(a) ? masks[a] : 0u) | (dynamic(b) ? masks[b] : 0u);
				if (~used == 0u)
				{
					// The native finish pass clears all assigned colours before its serial fallback.
					overflow = true;
					std::ranges::fill(expected, MaxColours);
					break;
				}
				auto const colour = static_cast<uint32_t>(std::countr_zero(~used));
				expected[slot] = colour;
				used_count = std::max(used_count, int(colour) + 1);
				if (dynamic(a))
					masks[a] |= 1u << colour;

				if (dynamic(b))
					masks[b] |= 1u << colour;
			}

			// The bound and fallback must cover every occupied indirect entry; omitted tail entries must be empty.
			auto state_read = TestClass_ConstraintUploadBookkeepingTests::ReadBuffer<GpuConstraintSolverState>(solver.FrameState(), 1);
			auto args_read = TestClass_ConstraintUploadBookkeepingTests::ReadBuffer<DispatchArguments>(solver.m_r_colour_dispatch.get(), MaxColours);
			gpu.m_job.Run();
			auto const& state = *state_read.ptr<GpuConstraintSolverState>();
			auto const* args = args_read.ptr<DispatchArguments>();
			PR_EXPECT(state.failure_slot_plus_one == 0u);
			PR_EXPECT((state.colour_overflow != 0u) == overflow);
			PR_EXPECT(bound >= 1 && bound <= MaxColours);
			PR_EXPECT(overflow ? bound == MaxColours : used_count <= bound);
			for (int slot = 0; slot != isize(edges); ++slot)
			{
				// Native level scheduling must reproduce the independent serial choice at every slot.
				PR_EXPECT(blocks[slot].colour == expected[slot]);
			}
			for (int colour = 0; colour != MaxColours; ++colour)
			{
				// Overflow must retain colour zero, and no skipped tail colour may contain work.
				if (overflow)
					PR_EXPECT(args[colour].ThreadGroupCountX == (colour == 0 ? 1u : 0u));
				else if (colour >= bound)
					PR_EXPECT(args[colour].ThreadGroupCountX == 0u);
			}
			log_stream() << name << " slots=" << edges.size() << " bound=" << bound << " serial_count=" << used_count
				<< " overflow=" << overflow << " recorded_dispatches=" << solver.Stats().m_dispatch_count << '\n';
			return {bound, used_count};
		}

		// Deep and branching graphs must remain covered when stable slot order changes.
		PRUnitTestMethod(DeepBranchingAndReordered, Quick)
		{
			// Long chains exercise repeated low-degree slots beyond the fused sweep's normal size.
			auto edges = std::vector<std::pair<int, int>>{};
			for (int slot = 0; slot != 300; ++slot)
			{
				// Each new edge shares only its preceding chain endpoint.
				edges.emplace_back(slot, slot + 1);
			}
			auto const chain = Check("deep", edges);
			std::ranges::reverse(edges);
			Check("deep-reversed", edges);

			// A binary tree and a fixed reordered visit exercise branching and future incident edges.
			edges.clear();
			for (int body = 1; body != 64; ++body)
			{
				// Parent indices define a tree without imposing its eventual colouring order.
				edges.emplace_back((body - 1) / 2, body);
			}
			Check("branching", edges);
			auto rng = std::default_random_engine{1234u};
			std::shuffle(edges.begin(), edges.end(), rng);
			Check("branching-reordered", edges);
			PR_EXPECT(chain.first == chain.second);
		}

		// Parallel, reversed, world, static, disabled and coupled endpoints retain their established participation.
		PRUnitTestMethod(EndpointParticipation, Quick)
		{
			// Repeated endpoint pairs share both masks without requiring a pair cache.
			Check("parallel-reversed", {{0, 1}, {1, 0}, {0, 1}, {1, 0}, {0, 1}, {1, 0}});
			Check("world-static-disabled-coupled", {{0, -1}, {0, 1}, {0, 2}, {-1, 0}, {2, 0}, {1, 2}, {2, 1}, {2, 0}, {0, -1}, {0, 2}}, 1, std::array{1, 4, 9}, std::array{2, 7});

			// An empty or entirely coupled stream still permits the colour-zero fallback.
			auto config = EngineConfig{};
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			PR_EXPECT(solver.ColourBound({}) == 1);
			auto coupled = std::array{GpuConstraintEndpoint{.body_idx_a = 0, .body_idx_b = 1, .flags = GpuConstraintEndpointFlags_Coupled}};
			PR_EXPECT(solver.ColourBound(coupled) == 1);
		}

		// Exhaustion must keep the existing coherent serial pass rather than dropping the overflowing slot.
		PRUnitTestMethod(OverflowKeepsColourZero, Quick)
		{
			// A shared dynamic hub requires more colours than the fixed native mask can represent.
			auto edges = std::vector<std::pair<int, int>>{};
			for (int slot = 0; slot != MaxColours + 3; ++slot)
			{
				// Distinct leaves prevent unrelated endpoint conflicts from explaining the overflow.
				edges.emplace_back(0, slot + 1);
			}
			Check("overflow-hub", edges);
			for (auto& edge : edges)
			{
				// A world endpoint cannot remove the shared dynamic body's colour exhaustion.
				edge.second = -1;
			}
			Check("overflow-world", edges);
		}
	};

	PRUnitTestClass(ConstraintGpuAlgebraRegressionTests)
	{
		// Compilation reports the highest failing slot even when colouring later selects the serial fallback.
		PRUnitTestMethod(FrameLatchPreservesCompileFailureThroughColouring, Quick)
		{
			auto body = MakeConstraintGpuBody();
			auto constraints = ConstraintSet{};
			auto const slot_count = MaxColours + 4;
			for (int index = 0; index != slot_count; ++index)
			{
				auto desc = D6ConstraintDesc{};
				desc.m_frame_b.m_body = BodyRef::Rigid(body);
				desc.m_linear[0] = MakeConstraintGpuLockedAxis();
				desc.m_linear[0].m_stiffness = index == 0 || index == slot_count / 2 || index == slot_count - 1 ? 1.0e-20f : 0.0f;
				constraints.Add(desc);
			}
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto const buffers = ConstraintRunnerBuffers{1.0e-20f, bodies, upload.m_endpoints, upload.m_descriptors};
			auto runner = ConstraintInteropRunner{};
			runner.Load(buffers);
			runner.CompileConstraints();
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == static_cast<uint32_t>(slot_count));

			// More surviving constraints than colours share one body; changing the fallback word must preserve the failure word.
			runner.AssignColours();
			PR_EXPECT(runner.ColourOverflow());
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == static_cast<uint32_t>(slot_count));
			runner.PrepareVelocitySolve();
			runner.SolveVelocity();
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == static_cast<uint32_t>(slot_count));
			PR_THROWS(runner.Store(buffers), std::exception);

			// Only loading the next frame resets both words.
			runner.Load(buffers);
			PR_EXPECT(runner.FrameState().colour_overflow == 0u);
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == 0u);
		}

		// The level-parallel colouring reproduces the serial greedy colouring exactly, including the overflow fallback.
		PRUnitTestMethod(ParallelColouringMatchesSerialGreedy, Quick)
		{
			// Visit the compiled blocks in slot order, as the serial colouring does, and record colours, body masks, and the indirect arguments.
			struct SerialResult
			{
				std::vector<uint32_t> m_colours;
				std::vector<uint32_t> m_body_masks;
				std::vector<DispatchArguments> m_args;
				bool m_overflow;
			};
			auto serial_colouring = [](std::span<GpuConstraintBlock const> blocks, std::span<GpuConstraintEndpoint const> endpoints, std::span<GpuRigidBody const> bodies)
			{
				// Track per-body masks and the union of used colours.
				auto result = SerialResult{
					.m_colours = std::vector<uint32_t>(blocks.size(), MaxColours),
					.m_body_masks = std::vector<uint32_t>(bodies.size(), 0u),
					.m_args = std::vector<DispatchArguments>(MaxColours),
					.m_overflow = false,
				};
				auto dynamic = [&](int body_idx)
				{
					return body_idx >= 0 && body_idx < isize(bodies) && bodies[body_idx].os_com_and_invmass.w > 0.0f && !AllSet(bodies[body_idx].state_flags, ERigidBodyStateFlags_Static);
				};
				auto used_colours = 0u;
				for (int slot_idx = 0; slot_idx != isize(blocks); ++slot_idx)
				{
					// Coupled, inactive, and velocity-free slots are not coloured.
					auto const& block = blocks[slot_idx];
					if (AllSet(endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled) || !AllSet(block.flags, ConstraintBlockFlags_Active) || block.velocity_mask == 0u)
						continue;

					auto used = 0u;
					if (dynamic(block.body_idx_a))
						used |= result.m_body_masks[block.body_idx_a];
					if (dynamic(block.body_idx_b))
						used |= result.m_body_masks[block.body_idx_b];
					if (~used == 0u)
					{
						result.m_overflow = true;
						break;
					}

					auto const colour = static_cast<uint32_t>(std::countr_zero(~used));
					result.m_colours[slot_idx] = colour;
					used_colours |= 1u << colour;
					if (dynamic(block.body_idx_a))
						result.m_body_masks[block.body_idx_a] |= 1u << colour;
					if (dynamic(block.body_idx_b))
						result.m_body_masks[block.body_idx_b] |= 1u << colour;
				}

				// Overflow selects one serial pass in colour zero; otherwise each used colour covers every slot.
				if (result.m_overflow)
					std::ranges::fill(result.m_colours, MaxColours);

				auto const groups = static_cast<uint32_t>((blocks.size() + ConstraintThreadCount - 1) / ConstraintThreadCount);
				for (int colour = 0; colour != MaxColours; ++colour)
				{
					auto& args = result.m_args[colour];
					args.ThreadGroupCountX = result.m_overflow ? (colour == 0 ? 1u : 0u) : ((used_colours & (1u << colour)) != 0u ? groups : 0u);
					args.ThreadGroupCountY = 1;
					args.ThreadGroupCountZ = 1;
				}
				return result;
			};

			// Random topologies mix dynamic, static, and world endpoints with disabled and coupled slots. Every fourth case adds a hub that
			// needs more colours than exist, so the overflow path is covered at different slot positions.
			auto rng = std::default_random_engine{1234u};
			auto pick = [&](int count)
			{
				return std::uniform_int_distribution<int>(0, count - 1)(rng);
			};
			for (int test_case = 0; test_case != 24; ++test_case)
			{
				// Bodies are slightly separated so every locked axis has a finite frame. Some bodies are static.
				auto const body_count = 4 + pick(60);
				auto bodies = std::vector<GpuRigidBody>{};
				for (int body_idx = 0; body_idx != body_count; ++body_idx)
				{
					auto const rigid_body = MakeConstraintGpuBody(1.0f, m4x4::Translation(0.01f * body_idx, 0, 0));
					bodies.push_back(PackDynamics(rigid_body, body_idx));
					if (pick(8) == 0)
						bodies.back().os_com_and_invmass.w = 0.0f;
				}

				// Random slots, with an optional hub run of more than 'MaxColours' slots on body zero.
				auto const hub = test_case % 4 == 3;
				auto const slot_count = 1 + pick(200) + (hub ? MaxColours + 8 : 0);
				auto const hub_begin = pick(slot_count - (hub ? MaxColours + 8 : 0));
				auto endpoints = std::vector<GpuConstraintEndpoint>(slot_count);
				auto descriptors = std::vector<GpuD6ConstraintDesc>(slot_count);
				for (int slot_idx = 0; slot_idx != slot_count; ++slot_idx)
				{
					auto& endpoint = endpoints[slot_idx];
					auto const in_hub = hub && slot_idx >= hub_begin && slot_idx < hub_begin + MaxColours + 8;
					endpoint.body_idx_a = in_hub ? 0 : pick(body_count + 1) - 1;
					endpoint.body_idx_b = in_hub ? 1 + (slot_idx - hub_begin) % (body_count - 1) : pick(body_count + 1) - 1;
					endpoint.flags = pick(10) == 0 ? GpuConstraintEndpointFlags_None : GpuConstraintEndpointFlags_Enabled;
					if (!in_hub && pick(12) == 0)
						endpoint.flags |= GpuConstraintEndpointFlags_Coupled;

					auto& descriptor = descriptors[slot_idx];
					descriptor.frame_a.rotation = float4{0, 0, 0, 1};
					descriptor.frame_b.rotation = float4{0, 0, 0, 1};
					descriptor.axes[0].mode = GpuConstraintAxisMode_Locked;
					descriptor.axes[0].max_force = 1000.0f;
				}
				if (hub)
					bodies[0].os_com_and_invmass.w = 1.0f;

				// Compile, then compare the parallel colouring with the serial reference over the same compiled blocks.
				auto runner = ConstraintInteropRunner{MakeConstraintGpuHubConfig()};
				runner.Load(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, endpoints, descriptors});
				runner.CompileConstraints();
				auto const compiled = std::vector<GpuConstraintBlock>(runner.Blocks().begin(), runner.Blocks().end());
				auto const expected = serial_colouring(compiled, endpoints, bodies);
				runner.AssignColours();

				PR_EXPECT(runner.ColourOverflow() == expected.m_overflow);
				if (hub)
					PR_EXPECT(expected.m_overflow);

				for (int slot_idx = 0; slot_idx != slot_count; ++slot_idx)
					PR_EXPECT(runner.Colours()[slot_idx] == expected.m_colours[slot_idx]);

				for (int body_idx = 0; body_idx != body_count; ++body_idx)
					PR_EXPECT(runner.Bodies()[body_idx].colour_used == expected.m_body_masks[body_idx]);

				for (int colour = 0; colour != MaxColours; ++colour)
				{
					PR_EXPECT(runner.ColourDispatch()[colour].ThreadGroupCountX == expected.m_args[colour].ThreadGroupCountX);
					PR_EXPECT(runner.ColourDispatch()[colour].ThreadGroupCountY == 1u);
					PR_EXPECT(runner.ColourDispatch()[colour].ThreadGroupCountZ == 1u);
				}
			}
		}

		// A later successful row compilation cannot erase an earlier physical-solve failure from the same loaded frame.
		PRUnitTestMethod(FrameLatchSurvivesSuccessfulRecompile, Quick)
		{
			auto body = MakeConstraintGpuBody();
			auto desc = D6ConstraintDesc{};
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			for (int index = 0; index != 3; ++index)
				constraints.Add(desc);

			// Nonfinite physical input reaches the solve failure path without invalidating the finite descriptor geometry.
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			bodies[0].momentum_lin.x = std::numeric_limits<float>::quiet_NaN();
			auto const buffers = ConstraintRunnerBuffers{0.1f, bodies, upload.m_endpoints, upload.m_descriptors};
			auto runner = ConstraintInteropRunner{};
			runner.Load(buffers);
			runner.CompileConstraints();
			runner.AssignColours();
			runner.PrepareVelocitySolve();
			runner.SolveVelocity();
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == 3u);

			// Fresh row flags can be valid while the frame remains failed; publication must consult the independent latch.
			runner.CompileConstraints();
			runner.AssignColours();
			for (auto const& block : runner.Blocks())
				PR_EXPECT(!AllSet(block.flags, ConstraintBlockFlags_NumericalFailure));

			PR_EXPECT(runner.FrameState().failure_slot_plus_one == 3u);
			PR_THROWS(runner.Store(buffers), std::exception);

			// Corrected physical input succeeds after a new frame load resets the latch.
			bodies[0] = PackDynamics(body, 0);
			runner.Run(buffers);
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == 0u);
		}

		// Unrepresentable soft arithmetic is observable on both public solve paths and never becomes a successful hard lock.
		PRUnitTestMethod(UnrepresentableSoftRowsReportFailure, Quick)
		{
			auto body = MakeConstraintGpuBody(1.0f, m4x4::Translation(1.0f, 0.0f, 0.0f));
			body.VelocityWS(v8motion{v4::Zero(), v4::XAxis()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			desc.m_linear[0].m_stiffness = 1.0e-20f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			auto solver = CpuConstraintSolver{};

			// The positive spring denominator underflows in float, so replay must retain the shader failure bit and reject output publication.
			PR_THROWS(runner.Run(ConstraintRunnerBuffers{1.0e-20f, bodies, upload.m_endpoints, upload.m_descriptors}), std::exception);
			PR_EXPECT(AllSet(runner.Blocks()[0].flags, ConstraintBlockFlags_NumericalFailure));
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == 1u);
			PR_EXPECT(bodies[0].momentum_lin.x == 1.0f);
			PR_THROWS(solver.Solve(CompileConstraints(constraints, remap), remap, 1.0e-20f, config), std::exception);
			PR_EXPECT(body.VelocityWS().lin.x == 1.0f);

			// A fresh solve with representable coefficients clears the diagnostic and recovers without rebuilding the runner.
			runner.Run(ConstraintRunnerBuffers{0.1f, bodies, upload.m_endpoints, upload.m_descriptors});
			PR_EXPECT(!AllSet(runner.Blocks()[0].flags, ConstraintBlockFlags_NumericalFailure));
			PR_EXPECT(runner.FrameState().failure_slot_plus_one == 0u);
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin.x, 1.0f, 1.0e-6f));
		}

		// A large spring gamma must not hide a valid heavy-body hard row in the same dense block.
		PRUnitTestMethod(MixedMassAndComplianceScalesPreserveHardRow, Quick)
		{
			auto body = MakeConstraintGpuBody(1.0e6f);
			body.VelocityWS(v8motion{v4::Zero(), v4::XAxis()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(std::numeric_limits<float>::infinity());
			desc.m_linear[1] = MakeConstraintGpuLockedAxis(std::numeric_limits<float>::infinity());
			desc.m_linear[1].m_stiffness = 0.01f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors});
			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, config);

			// The diagonal response spans eleven orders of magnitude but is full rank, so no regularization is justified.
			PR_EXPECT(Abs(body.VelocityWS().lin.x) < 2.0e-6f);
			PR_EXPECT(Abs(bodies[0].momentum_lin.x * bodies[0].os_com_and_invmass.w) < 2.0e-6f);
			PR_EXPECT(metrics.m_regularized_blocks == 0);
			PR_EXPECT(metrics.m_singular_blocks == 0);
		}

		// Shader replay must solve the response-metric bounded block rather than converge to a clamped dense fixed point.
		PRUnitTestMethod(BoundedBlockSatisfiesAnalyticalKkt, Quick)
		{
			for (auto const iterations : {1, 2, 12})
			for (auto const initial_speed : {0.0f, -1.0f})
			{
				auto body = MakeConstraintGpuBody();
				body.SetMassProperties(Inertia{1.0f, 1.0f});
				body.VelocityWS(v8motion{v4::Zero(), v4{initial_speed, initial_speed, 0, 0}});
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::World(), m4x4::Translation(1.0f, -1.0f, 0.0f));
				desc.m_frame_b = BodyFrame{BodyRef::Rigid(body), desc.m_frame_a.m_rotation, desc.m_frame_a.m_position};
				desc.m_linear[0] = MakeConstraintGpuLockedAxis(std::numeric_limits<float>::infinity());
				desc.m_linear[1].m_mode = EConstraintAxisMode::Driven;
				desc.m_linear[1].m_target_velocity = initial_speed == 0.0f ? 3.0f : 0.0f;
				desc.m_linear[1].m_max_force = 1.0f;
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 1>{&body};
				auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = iterations;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;
				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{0.1f, bodies, upload.m_endpoints, upload.m_descriptors});

				// With K = [[2,1],[1,2]], the saturated Y impulse fixes lambda_x = (-v_x - 0.1) / 2.
				auto const expected_x = (-initial_speed - 0.1f) / 2.0f;
				auto const point_velocity = bodies[0].momentum_lin + Cross(bodies[0].momentum_ang, v4{1, -1, 0, 0});
				PR_EXPECT(FEqlAbsolute(runner.Rows()[0].bounds.z, expected_x, 2.0e-6f));
				PR_EXPECT(FEqlAbsolute(runner.Rows()[1].bounds.z, 0.1f, 2.0e-6f));
				PR_EXPECT(Abs(point_velocity.x) < 2.0e-6f);
				PR_EXPECT(FEqlAbsolute(point_velocity.y, initial_speed + expected_x + 0.2f, 2.0e-6f));
			}
		}

		// A common slider rotation is an exact admissible velocity, not a source of dissipative joint impulses.
		PRUnitTestMethod(CommonSliderRotationPreservesAngularMomentum, Quick)
		{
			auto body_a = MakeConstraintGpuBody();
			auto body_b = MakeConstraintGpuBody(1.0f, m4x4::Translation(2.0f, 0.0f, 0.0f));
			body_a.SetMassProperties(Inertia{1.0f, 1.0f});
			body_b.SetMassProperties(Inertia{1.0f, 1.0f});
			body_a.VelocityWS(v8motion{v4::ZAxis(), v4::Zero()});
			body_b.VelocityWS(v8motion{v4::ZAxis(), 2.0f * v4::YAxis()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
			desc.m_linear[1] = MakeConstraintGpuLockedAxis();
			desc.m_linear[2] = MakeConstraintGpuLockedAxis();
			for (auto& axis : desc.m_angular)
				axis = MakeConstraintGpuLockedAxis();

			// Run the production shader through its deterministic C++ interop surface without requiring a GPU device.
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{0.1f, bodies, upload.m_endpoints, upload.m_descriptors});

			// The derivative of C_y under A-only rotation is -2; spin plus orbital momentum must remain 6.
			auto const angular_momentum = bodies[0].momentum_ang.z + bodies[1].momentum_ang.z + 2.0f * bodies[1].momentum_lin.y;
			PR_EXPECT(FEqlAbsolute(runner.Rows()[1].jacobian_a_ang.z, -2.0f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(angular_momentum, 6.0f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_ang, v4::ZAxis(), 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[1].momentum_ang, v4::ZAxis(), 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin, v4::Zero(), 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[1].momentum_lin, 2.0f * v4::YAxis(), 2.0e-5f));
		}

		// CPU and shader paths must both preserve the analytically specified weak implicit spring.
		PRUnitTestMethod(WeakSpringMatchesImplicitVelocityAndCpu, Quick)
		{
			auto body = MakeConstraintGpuBody(1.0f, m4x4::Translation(1.0f, 0.0f, 0.0f));
			body.VelocityWS(v8motion{v4::Zero(), v4::XAxis()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(std::numeric_limits<float>::infinity());
			desc.m_linear[0].m_stiffness = 0.01f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto const timestep = 1.0f / 60.0f;
			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{timestep, bodies, upload.m_endpoints, upload.m_descriptors});
			auto solver = CpuConstraintSolver{};
			solver.Solve(CompileConstraints(constraints, remap), remap, timestep, config);

			// The independent backward-Euler value prevents two matching implementations from agreeing on a hard lock.
			auto const expected = (1.0f - timestep * 0.01f) / (1.0f + timestep * timestep * 0.01f);
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin.x, expected, 3.0e-7f));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, expected, 3.0e-7f));
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin.x, body.VelocityWS().lin.x, 3.0e-7f));
		}

		// Scalar mass scaling must not cause the CPU path to discard a row that shader arithmetic can resolve.
		PRUnitTestMethod(HeavyBodyWorldLockMatchesAnalyticalVelocityAndCpu, Quick)
		{
			auto body = MakeConstraintGpuBody(1.0e6f);
			body.VelocityWS(v8motion{v4::Zero(), v4::XAxis()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(std::numeric_limits<float>::infinity());
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{0.1f, bodies, upload.m_endpoints, upload.m_descriptors});
			auto solver = CpuConstraintSolver{};
			solver.Solve(CompileConstraints(constraints, remap), remap, 0.1f, config);

			// An unlimited hard world lock has exactly zero final X velocity regardless of finite body mass.
			auto const gpu_velocity = bodies[0].momentum_lin.x * bodies[0].os_com_and_invmass.w;
			PR_EXPECT(Abs(gpu_velocity) < 2.0e-6f);
			PR_EXPECT(Abs(body.VelocityWS().lin.x) < 2.0e-6f);
			PR_EXPECT(FEqlAbsolute(gpu_velocity, body.VelocityWS().lin.x, 2.0e-6f));
		}

		// Split-position compilation must retain satisfied hard rows that oppose another row's correction.
		PRUnitTestMethod(SatisfiedHardRowsParticipateInPositionSolve, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.SetMassProperties(Inertia{1.0f, 1.0f});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::World(), m4x4::Translation(1.0f, -1.0f, 0.0f));
			desc.m_frame_b = BodyFrame{BodyRef::Rigid(body), desc.m_frame_a.m_rotation, desc.m_frame_a.m_position};
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			desc.m_linear[1] = MakeConstraintGpuLockedAxis();
			desc.m_linear[1].m_target_position = 0.01f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 0;
			config.m_position_iterations = 1;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{0.1f, bodies, upload.m_endpoints, upload.m_descriptors});

			// The second row requests 0.002 Y displacement, while the first row enforces zero linearized X displacement.
			auto const anchor = bodies[0].o2w * v4{1, -1, 0, 1};
			PR_EXPECT(runner.Blocks()[0].position_mask == 3u);
			PR_EXPECT(FEqlAbsolute(anchor.x, 1.0f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(anchor.y, -0.998f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin, v4::Zero(), 1.0e-6f));
		}
	};

	PRUnitTestClass(ConstraintGpuSolverTests)
	{
		// Match CPU world-frame compilation for rotated frames, offset CoMs, a world endpoint, and mixed canonical axis modes.
		PRUnitTestMethod(CompileMatchesCpuCanonicalRows, Quick)
		{
			auto body = MakeConstraintGpuBody(
				2.0f,
				m4x4::Transform(v4::YAxis(), 0.35f, v4{1.0f, -2.0f, 0.5f, 1}),
				v4{0.2f, -0.1f, 0.3f, 0});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body), m4x4::Transform(v4::ZAxis(), -0.2f, v4{0.4f, 0.1f, -0.3f, 1}));
			desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::World(), m4x4::Transform(v4::XAxis(), 0.15f, v4{1.3f, -1.7f, 0.8f, 1}));
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			desc.m_linear[1].m_mode = EConstraintAxisMode::Limited;
			desc.m_linear[1].m_limits = Range<float>{-0.05f, +0.05f};
			desc.m_linear[1].m_stiffness = 12.0f;
			desc.m_linear[1].m_damping = 3.0f;
			desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
			desc.m_angular[2].m_target_position = 0.1f;
			desc.m_angular[2].m_target_velocity = -0.4f;
			desc.m_angular[2].m_max_force = 50.0f;

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const cpu = CompileConstraints(constraints, remap);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto runner = ConstraintInteropRunner{};
			auto buffers = ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = gpu_bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			};
			runner.Load(buffers);
			runner.CompileConstraints();

			PR_EXPECT(cpu.m_blocks.size() == 1);
			PR_EXPECT(cpu.m_rows.size() == 3);
			PR_EXPECT(runner.Blocks()[0].velocity_mask == ((1u << 0) | (1u << 1) | (1u << 5)));
			PR_EXPECT((runner.Blocks()[0].position_mask & (1u << 5)) == 0u);
			ExpectConstraintGpuJacobian(runner.Rows()[0], cpu.m_rows[0]);
			ExpectConstraintGpuJacobian(runner.Rows()[1], cpu.m_rows[1]);
			ExpectConstraintGpuJacobian(runner.Rows()[5], cpu.m_rows[2]);
			PR_EXPECT(FEqlAbsolute(runner.Rows()[0].solve.x, cpu.m_rows[0].m_position - cpu.m_rows[0].m_target_position, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(runner.Rows()[5].solve.y, cpu.m_rows[2].m_target_velocity, 1.0e-6f));
			PR_EXPECT(runner.Rows()[1].solve.w > 0.0f);
		}

		// Match the CPU block PGS momentum result for a coupled off-centre three-axis lock.
		PRUnitTestMethod(VelocitySolveMatchesCpuReference, Quick)
		{
			auto body_a = MakeConstraintGpuBody(2.0f, m4x4::Identity(), v4{0.2f, -0.1f, 0.05f, 0});
			auto body_b = MakeConstraintGpuBody(3.0f, m4x4::Translation(0.2f, -0.1f, 0.3f), v4{-0.1f, 0.15f, -0.05f, 0});
			body_a.VelocityWS(v8motion{v4{0.3f, -0.2f, 0.1f, 0}, v4{1.0f, -0.5f, 0.7f, 0}});
			body_b.VelocityWS(v8motion{v4{-0.1f, 0.4f, -0.2f, 0}, v4{-0.6f, 0.8f, -0.3f, 0}});

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body_a), m4x4::Translation(0.4f, 0.2f, -0.1f));
			desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::Rigid(body_b), m4x4::Translation(0.2f, 0.3f, -0.2f));
			for (int axis = 0; axis != 3; ++axis)
				desc.m_linear[axis] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 4;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;

			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = gpu_bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});
			auto cpu_solver = CpuConstraintSolver{};
			cpu_solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, config);

			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_ang, body_a.MomentumWS().ang, 5.0e-4f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_lin, body_a.MomentumWS().lin, 5.0e-4f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_ang, body_b.MomentumWS().ang, 5.0e-4f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_lin, body_b.MomentumWS().lin, 5.0e-4f));
		}

		// Match the CPU reference across deterministic six-axis blocks with varied frames, CoMs, transforms, and momenta.
		PRUnitTestMethod(RandomizedD6BlocksMatchCpuReference, Quick)
		{
			for (int trial = 0; trial != 8; ++trial)
			{
				auto const seed = static_cast<float>(trial + 1);
				auto body_a = MakeConstraintGpuBody(
					1.0f + 0.2f * seed,
					m4x4::Transform(Normalise(v4{0.3f, 0.5f, 0.7f, 0}), 0.07f * seed, v4{0.1f * seed, -0.03f * seed, 0.02f * seed, 1}),
					v4{0.02f * seed, -0.01f * seed, 0.015f * seed, 0});
				auto body_b = MakeConstraintGpuBody(
					1.5f + 0.1f * seed,
					m4x4::Transform(Normalise(v4{0.6f, -0.2f, 0.4f, 0}), -0.05f * seed, v4{-0.04f * seed, 0.06f * seed, -0.01f * seed, 1}),
					v4{-0.01f * seed, 0.025f * seed, -0.02f * seed, 0});
				body_a.VelocityWS(v8motion{v4{0.13f * seed, -0.07f * seed, 0.04f * seed, 0}, v4{0.2f * seed, -0.11f * seed, 0.09f * seed, 0}});
				body_b.VelocityWS(v8motion{v4{-0.08f * seed, 0.05f * seed, -0.03f * seed, 0}, v4{-0.15f * seed, 0.12f * seed, -0.06f * seed, 0}});

				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body_a), m4x4::Transform(v4::ZAxis(), 0.03f * seed, v4{0.2f, -0.1f, 0.15f, 1}));
				desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::Rigid(body_b), m4x4::Transform(v4::YAxis(), -0.02f * seed, v4{-0.1f, 0.25f, -0.05f, 1}));
				for (int axis = 0; axis != 3; ++axis)
				{
					desc.m_linear[axis] = MakeConstraintGpuLockedAxis(5000.0f);
					desc.m_angular[axis] = MakeConstraintGpuLockedAxis(5000.0f);
				}
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto remap = BodyRemap(body_ptrs);
				auto const upload = PackGpuConstraints(constraints, remap);
				auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 6;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;

				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 120.0f, gpu_bodies, upload.m_endpoints, upload.m_descriptors});
				auto cpu_solver = CpuConstraintSolver{};
				cpu_solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 120.0f, config);

				PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_ang, body_a.MomentumWS().ang, 1.5e-3f));
				PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_lin, body_a.MomentumWS().lin, 1.5e-3f));
				PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_ang, body_b.MomentumWS().ang, 1.5e-3f));
				PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_lin, body_b.MomentumWS().lin, 1.5e-3f));
			}
		}

		// Resolve a scalar constraint across the required mass-ratio range without losing passivity, momentum, or finite state.
		PRUnitTestMethod(ExtremeMassRatiosRemainFiniteAndPassive, Quick)
		{
			for (auto const heavy_mass : {1.0f, 1.0e2f, 1.0e4f, 1.0e6f})
			{
				auto body_a = MakeConstraintGpuBody(1.0f);
				auto body_b = MakeConstraintGpuBody(heavy_mass);
				body_a.VelocityWS(v8motion{v4::Zero(), v4::XAxis()});
				body_b.VelocityWS(v8motion{});

				// A single bilateral scalar row has a closed-form shared velocity that remains well-conditioned at 10^6:1.
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
				desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
				desc.m_linear[0] = MakeConstraintGpuLockedAxis();
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 1;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;

				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors});

				// The projected update must converge to the analytic inelastic velocity without creating kinetic energy.
				auto const velocity_a = bodies[0].momentum_lin.x * bodies[0].os_com_and_invmass.w;
				auto const velocity_b = bodies[1].momentum_lin.x * bodies[1].os_com_and_invmass.w;
				auto const expected_velocity = 1.0f / (1.0f + heavy_mass);
				auto const momentum_after = bodies[0].momentum_lin.x + bodies[1].momentum_lin.x;
				auto const energy_after = 0.5f * (
					Sqr(bodies[0].momentum_lin.x) * bodies[0].os_com_and_invmass.w +
					Sqr(bodies[1].momentum_lin.x) * bodies[1].os_com_and_invmass.w);
				PR_EXPECT(ConstraintGpuBodyFinite(bodies[0]));
				PR_EXPECT(ConstraintGpuBodyFinite(bodies[1]));
				PR_EXPECT(Abs(velocity_a - expected_velocity) < 2.0e-6f);
				PR_EXPECT(Abs(velocity_b - expected_velocity) < 2.0e-6f);
				PR_EXPECT(Abs(velocity_a - velocity_b) < 2.0e-6f);
				PR_EXPECT(Abs(momentum_after - 1.0f) < 2.0e-5f);
				PR_EXPECT(energy_after <= 0.50001f);
			}
		}

		// Preserve constraint results under arbitrary common world translations and rotations.
		PRUnitTestMethod(WorldTransformMetamorphicEquivalence, Quick)
		{
			auto const base_o2w_a = m4x4::Transform(v4::YAxis(), +0.23f, v4{-0.4f, +0.2f, -0.1f, 1});
			auto const base_o2w_b = m4x4::Transform(v4::XAxis(), -0.31f, v4{+0.5f, -0.3f, +0.2f, 1});
			auto const base_velocity_a = v8motion{v4{+0.3f, -0.2f, +0.1f, 0}, v4{+1.1f, -0.4f, +0.7f, 0}};
			auto const base_velocity_b = v8motion{v4{-0.1f, +0.4f, -0.3f, 0}, v4{-0.6f, +0.8f, -0.2f, 0}};

			// Solve the same endpoint-local problem after applying one common rigid world transform.
			auto run = [&](m4x4 const& transformed_from_base)
			{
				auto body_a = MakeConstraintGpuBody(2.0f, transformed_from_base * base_o2w_a, v4{+0.2f, -0.1f, +0.05f, 0});
				auto body_b = MakeConstraintGpuBody(3.0f, transformed_from_base * base_o2w_b, v4{-0.1f, +0.15f, -0.05f, 0});
				body_a.VelocityWS(v8motion{transformed_from_base.rot * base_velocity_a.ang, transformed_from_base.rot * base_velocity_a.lin});
				body_b.VelocityWS(v8motion{transformed_from_base.rot * base_velocity_b.ang, transformed_from_base.rot * base_velocity_b.lin});

				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body_a), m4x4::Transform(v4::ZAxis(), +0.17f, v4{+0.3f, -0.2f, +0.1f, 1}));
				desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::Rigid(body_b), m4x4::Transform(v4::YAxis(), -0.11f, v4{-0.2f, +0.25f, -0.15f, 1}));
				for (int axis = 0; axis != 3; ++axis)
				{
					desc.m_linear[axis] = MakeConstraintGpuLockedAxis(5000.0f);
					desc.m_angular[axis] = MakeConstraintGpuLockedAxis(5000.0f);
				}
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 6;
				config.m_position_iterations = 4;
				config.m_warm_start_factor = 0.0f;

				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 120.0f, bodies, upload.m_endpoints, upload.m_descriptors});
				return bodies;
			};

			auto const baseline = run(m4x4::Identity());
			auto const transforms = std::array{
				m4x4::Translation(+1000.0f, -2000.0f, +3000.0f),
				m4x4::Transform(Normalise(v4{0.3f, -0.5f, 0.7f, 0}), 0.63f, v4{+7.0f, -11.0f, +13.0f, 1}),
			};

			// Momentum is translation-invariant about each centre of mass and rotates covariantly with the complete scene.
			for (auto const& transformed_from_base : transforms)
			{
				auto const transformed = run(transformed_from_base);
				for (int body_idx = 0; body_idx != isize(transformed); ++body_idx)
				{
					auto const expected_o2w = transformed_from_base * baseline[body_idx].o2w;
					auto const expected_momentum_ang = transformed_from_base.rot * baseline[body_idx].momentum_ang;
					auto const expected_momentum_lin = transformed_from_base.rot * baseline[body_idx].momentum_lin;

					// At 3000 world units one float step is 2^-12, larger than 2e-4; permit one adjacent value only for translation components.
					for (int column = 0; column != 4; ++column)
					for (int row = 0; row != 4; ++row)
					{
						auto const actual = transformed[body_idx].o2w[column][row];
						auto const expected = expected_o2w[column][row];
						auto const adjacent_translation =
							column == 3 && row < 3 &&
							std::isfinite(actual) &&
							std::isfinite(expected) &&
							actual == std::nextafter(expected, actual);
						PR_EXPECT(FEqlAbsolute(actual, expected, 2.0e-4f) || adjacent_translation);
					}

					// Momentum retains its original covariance tolerance independently of world-position quantization.
					PR_EXPECT(FEqlAbsolute(transformed[body_idx].momentum_ang, expected_momentum_ang, 2.0e-3f));
					PR_EXPECT(FEqlAbsolute(transformed[body_idx].momentum_lin, expected_momentum_lin, 2.0e-3f));
				}
			}
		}

		// Match split position correction for a hard world lock while preserving physical momentum and an off-origin CoM.
		PRUnitTestMethod(PositionSolveMatchesCpuReference, Quick)
		{
			auto body = MakeConstraintGpuBody(
				2.0f,
				m4x4::Transform(v4::YAxis(), 0.12f, v4{0.3f, -0.2f, 0.1f, 1}),
				v4{0.2f, -0.1f, 0.15f, 0});
			body.MomentumWS(v8force{v4{0.2f, -0.1f, 0.3f, 0}, v4{0.4f, 0.5f, -0.2f, 0}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body), m4x4::Translation(0.25f, -0.15f, 0.1f));
			desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::World(), m4x4::Translation(0.7f, -0.05f, 0.2f));
			for (int axis = 0; axis != 3; ++axis)
				desc.m_linear[axis] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto const momentum_before = body.MomentumWS();
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 0;
			config.m_position_iterations = 4;
			config.m_warm_start_factor = 0.0f;

			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, gpu_bodies, upload.m_endpoints, upload.m_descriptors});
			auto cpu_solver = CpuConstraintSolver{};
			cpu_solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, config);

			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].o2w, body.O2W(), 2.0e-3f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_ang, momentum_before.ang, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_lin, momentum_before.lin, 1.0e-6f));
		}

		// Disabled and enabled all-free stable slots produce no block work or body mutation.
		PRUnitTestMethod(DisabledAndFreeSlotsAreNoOp, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4{0.1f, 0.2f, 0.3f, 0}, v4{1, 2, 3, 0}});
			auto before = PackDynamics(body, 0);
			auto bodies = std::vector<GpuRigidBody>{before};
			auto endpoints = std::vector<GpuConstraintEndpoint>(2);
			auto descriptors = std::vector<GpuD6ConstraintDesc>(2);
			endpoints[0].body_idx_a = 0;
			endpoints[0].body_idx_b = -1;
			endpoints[0].flags = GpuConstraintEndpointFlags_Enabled;

			auto runner = ConstraintInteropRunner{};
			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_endpoints = endpoints,
				.m_descriptors = descriptors,
			});

			PR_EXPECT(runner.Blocks()[0].velocity_mask == 0u);
			PR_EXPECT(runner.Blocks()[1].velocity_mask == 0u);
			PR_EXPECT(FEql(bodies[0].momentum_ang, before.momentum_ang));
			PR_EXPECT(FEql(bodies[0].momentum_lin, before.momentum_lin));
			PR_EXPECT(FEql(bodies[0].o2w, before.o2w));
		}

		// ResetWarmStart invalidates retained impulses even when the row's active limit state is unchanged.
		PRUnitTestMethod(ResetWarmStartClearsRetainedRows, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{2, 0, 0, 0}});
			auto constraints = ConstraintSet{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			auto runner = ConstraintInteropRunner{config};
			auto buffers = ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors};
			runner.Run(buffers);
			PR_EXPECT(std::abs(runner.Rows()[0].bounds.z) > 1.0e-5f);

			auto reset_endpoints = upload.m_endpoints;
			reset_endpoints[0].flags |= GpuConstraintEndpointFlags_ResetWarmStart;
			buffers.m_endpoints = reset_endpoints;
			runner.Load(buffers);
			runner.CompileConstraints();
			PR_EXPECT(runner.Rows()[0].bounds.z == 0.0f);
		}

		// Leave coupled runtime storage exclusively owned by its compiler even when a descriptor change requests warm-start reset.
		PRUnitTestMethod(RigidCompilerNeverMutatesCoupledSlots, Quick)
		{
			auto body = MakeConstraintGpuBody();
			auto constraints = ConstraintSet{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto runner = ConstraintInteropRunner{};
			auto buffers = ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors};
			runner.Load(buffers);
			runner.CompileConstraints();
			auto const block_before = runner.Blocks()[0];
			auto rows_before = std::array<GpuConstraintRow, GpuConstraintRowsPerBlock>{};
			std::copy_n(runner.Rows().begin(), rows_before.size(), rows_before.begin());

			auto coupled_endpoints = upload.m_endpoints;
			coupled_endpoints[0].flags |= GpuConstraintEndpointFlags_Coupled | GpuConstraintEndpointFlags_ResetWarmStart;
			buffers.m_endpoints = coupled_endpoints;
			runner.Load(buffers);
			runner.CompileConstraints();

			PR_EXPECT(std::memcmp(&runner.Blocks()[0], &block_before, sizeof(block_before)) == 0);
			PR_EXPECT(std::memcmp(runner.Rows().data(), rows_before.data(), sizeof(rows_before)) == 0);
		}

		// Convert force caps to timestep-scaled impulse bounds and reject invalid timesteps before mutation.
		PRUnitTestMethod(TimestepAndForceBoundsRemainFinite, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{100, 0, 0, 0}});
			auto constraints = ConstraintSet{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(2.0f);
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 2;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			auto buffers = ConstraintRunnerBuffers{0.25f, bodies, upload.m_endpoints, upload.m_descriptors};
			runner.Run(buffers);

			PR_EXPECT(FEqlAbsolute(runner.Rows()[0].bounds.x, -0.5f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(runner.Rows()[0].bounds.y, +0.5f, 1.0e-6f));
			PR_EXPECT(std::abs(runner.Rows()[0].bounds.z) <= 0.5f);
			buffers.m_dt = 0.0f;
			PR_THROWS(runner.Run(buffers), std::invalid_argument);
		}

		// Convert canonical linear impulses to force, latch the first overload, and disable later work in the same submitted frame.
		PRUnitTestMethod(BreakThresholdLatchesCanonicalLoad, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};

			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});

			auto const& state = runner.BreakStates()[0];
			PR_EXPECT(upload.m_breakable_count == 1);
			PR_EXPECT(AllSet(state.flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(state.generation == upload.m_endpoints[0].generation);
			PR_EXPECT(state.substep_index == 0);
			PR_EXPECT(state.peak_force > desc.m_break_force);
			PR_EXPECT(state.peak_torque == 0.0f);
			PR_EXPECT(AllSet(runner.Blocks()[0].flags, ConstraintBlockFlags_Broken));
			PR_EXPECT(runner.Blocks()[0].velocity_mask == 0u);
		}

		// Distinguish torque from force and report the canonical angular load that crossed the configured threshold.
		PRUnitTestMethod(BreakTorqueUsesAngularImpulse, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4{5.0f, 0.0f, 0.0f, 0.0f}, v4::Zero()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_angular[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_torque = 10.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};

			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});

			auto const& state = runner.BreakStates()[0];
			PR_EXPECT(AllSet(state.flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(state.peak_force == 0.0f);
			PR_EXPECT(state.peak_torque > desc.m_break_torque);
		}

		// Respect per-axis force clamps before overload testing, making an aggregate threshold above the clamp unreachable for one active axis.
		PRUnitTestMethod(BreakForceObservesAxisForceLimit, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{100.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(2.0f);
			desc.m_break_force = 2.1f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};

			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 0.25f,
				.m_bodies = bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});

			auto const& state = runner.BreakStates()[0];
			PR_EXPECT(!AllSet(state.flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(Abs(state.peak_force - desc.m_linear[0].m_max_force) < 1.0e-5f);
		}

		// Reordering independent stable slots does not change their disjoint body results.
		PRUnitTestMethod(IndependentInsertionPermutationsAgree, Quick)
		{
			auto initial_bodies = std::vector<GpuRigidBody>(4);
			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				initial_bodies[body_idx].o2w = m4x4::Identity();
				initial_bodies[body_idx].inertia_inv_diagonal = v4{1, 1, 1, 0};
				initial_bodies[body_idx].os_com_and_invmass = v4{0, 0, 0, 1};
				initial_bodies[body_idx].momentum_lin = v4{static_cast<float>(body_idx + 1), 0, 0, 0};
				initial_bodies[body_idx].sleep.island_id = -1;
			}
			auto endpoint_a = GpuConstraintEndpoint{.body_idx_a = 0, .body_idx_b = 1, .flags = GpuConstraintEndpointFlags_Enabled};
			auto endpoint_b = GpuConstraintEndpoint{.body_idx_a = 2, .body_idx_b = 3, .flags = GpuConstraintEndpointFlags_Enabled};
			auto descriptor = GpuD6ConstraintDesc{};
			descriptor.frame_a.rotation = float4{0, 0, 0, 1};
			descriptor.frame_b.rotation = float4{0, 0, 0, 1};
			descriptor.axes[0].mode = GpuConstraintAxisMode_Locked;
			descriptor.axes[0].max_force = 1000.0f;
			auto descriptors = std::vector<GpuD6ConstraintDesc>{descriptor, descriptor};
			auto endpoints_ab = std::vector<GpuConstraintEndpoint>{endpoint_a, endpoint_b};
			auto endpoints_ba = std::vector<GpuConstraintEndpoint>{endpoint_b, endpoint_a};
			auto bodies_ab = initial_bodies;
			auto bodies_ba = initial_bodies;
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 2;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner_ab = ConstraintInteropRunner{config};
			auto runner_ba = ConstraintInteropRunner{config};
			runner_ab.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies_ab, endpoints_ab, descriptors});
			runner_ba.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies_ba, endpoints_ba, descriptors});

			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(bodies_ab[body_idx].momentum_ang, bodies_ba[body_idx].momentum_ang, 1.0e-6f));
				PR_EXPECT(FEqlAbsolute(bodies_ab[body_idx].momentum_lin, bodies_ba[body_idx].momentum_lin, 1.0e-6f));
			}
		}

		// A degree-33 dynamic hub exhausts the 32-colour mask and remains finite under coherent serial fallback.
		PRUnitTestMethod(HighDegreeHubUsesSerialFallback, Quick)
		{
			auto bodies = std::vector<GpuRigidBody>{};
			auto runner = ConstraintInteropRunner{MakeConstraintGpuHubConfig()};
			RunConstraintGpuHub(runner, MaxColours + 1, bodies);

			PR_EXPECT(runner.ColourOverflow());
			for (auto const& body : bodies)
				PR_EXPECT(ConstraintGpuBodyFinite(body));

			// The serial fallback runs on one thread in colour zero, so every other colour dispatches no groups.
			for (int colour = 0; colour != MaxColours; ++colour)
			{
				auto const& args = runner.ColourDispatch()[colour];
				PR_EXPECT(args.ThreadGroupCountX == (colour == 0 ? 1u : 0u));
				PR_EXPECT(args.ThreadGroupCountY == 1u && args.ThreadGroupCountZ == 1u);
			}
		}

		// Occupied colours dispatch the full stable-slot grid and empty colours dispatch nothing.
		PRUnitTestMethod(ColourDispatchMatchesOccupiedColours, Quick)
		{
			auto bodies = std::vector<GpuRigidBody>{};
			auto runner = ConstraintInteropRunner{MakeConstraintGpuHubConfig()};
			RunConstraintGpuHub(runner, 5, bodies);
			PR_EXPECT(!runner.ColourOverflow());

			// Derive occupancy independently from the colour assignments, rather than trusting the shader that wrote the arguments.
			auto const groups = static_cast<uint32_t>((5 + ConstraintThreadCount - 1) / ConstraintThreadCount);
			for (int colour = 0; colour != MaxColours; ++colour)
			{
				auto const occupied = std::ranges::find(runner.Colours(), static_cast<uint32_t>(colour)) != runner.Colours().end();
				auto const& args = runner.ColourDispatch()[colour];
				PR_EXPECT(args.ThreadGroupCountX == (occupied ? groups : 0u));
				PR_EXPECT(args.ThreadGroupCountY == 1u && args.ThreadGroupCountZ == 1u);
			}
		}

		// Preserve quiet-time accumulation for awake loaded bodies while still invalidating persisted sleeping state after an impulse.
		PRUnitTestMethod(ConstraintImpulsesRespectSleepState, Quick)
		{
			auto run = [](bool sleeping)
			{
				auto rigid_body = MakeConstraintGpuBody();
				rigid_body.VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(rigid_body, 0)};
				bodies[0].sleep.timer_s = 0.4f;
				bodies[0].sleep.island_id = -1;
				bodies[0].sleep.generation = 7;
				if (sleeping)
					bodies[0].state_flags |= ERigidBodyStateFlags_Sleeping;

				auto endpoints = std::vector<GpuConstraintEndpoint>(1);
				endpoints[0].body_idx_a = 0;
				endpoints[0].body_idx_b = -1;
				endpoints[0].flags = GpuConstraintEndpointFlags_Enabled;
				auto descriptors = std::vector<GpuD6ConstraintDesc>(1);
				descriptors[0].frame_a.rotation = float4{0, 0, 0, 1};
				descriptors[0].frame_b.rotation = float4{0, 0, 0, 1};
				descriptors[0].axes[0].mode = GpuConstraintAxisMode_Locked;
				descriptors[0].axes[0].max_force = 1000.0f;
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 1;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;
				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, endpoints, descriptors});
				return bodies[0];
			};

			auto const awake = run(false);
			PR_EXPECT(awake.sleep.generation == 7);
			PR_EXPECT(awake.sleep.timer_s == 0.4f);

			auto const sleeping = run(true);
			PR_EXPECT(sleeping.sleep.generation == 8);
			PR_EXPECT(!AllSet(sleeping.state_flags, ERigidBodyStateFlags_Sleeping));
			PR_EXPECT(sleeping.sleep.timer_s == 0.0f);
		}

		// Match the HLSL-as-C++ reference through the production D3D12 compiler, pseudo-position, and velocity passes.
		PRUnitTestMethod(HardwarePipelineMatchesInterop, Extended)
		{
			auto body_a = MakeConstraintGpuBody(
				2.0f,
				m4x4::Transform(v4::YAxis(), 0.17f, v4{0.35f, -0.15f, 0.2f, 1}),
				v4{0.2f, -0.1f, 0.15f, 0});
			auto body_b = MakeConstraintGpuBody(
				3.0f,
				m4x4::Transform(v4::XAxis(), -0.11f, v4{-0.1f, 0.25f, -0.05f, 1}),
				v4{-0.1f, 0.05f, -0.08f, 0});
			body_a.VelocityWS(v8motion{v4{0.2f, -0.3f, 0.1f, 0}, v4{0.8f, -0.4f, 0.5f, 0}});
			body_b.VelocityWS(v8motion{v4{-0.1f, 0.15f, -0.2f, 0}, v4{-0.3f, 0.6f, -0.2f, 0}});

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body_a), m4x4::Translation(0.3f, -0.2f, 0.1f));
			desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::Rigid(body_b), m4x4::Translation(-0.15f, 0.25f, -0.05f));
			for (int axis = 0; axis != 3; ++axis)
			{
				desc.m_linear[axis] = MakeConstraintGpuLockedAxis(5000.0f);
				desc.m_angular[axis] = MakeConstraintGpuLockedAxis(5000.0f);
			}

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto expected = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
			auto actual = expected;
			auto config = EngineConfig{};
			config.solver_iterations = 2;
			config.push_out_iterations = 2;
			config.constraint_relaxation = 0.73f;
			config.constraint_position_relaxation = 0.61f;
			config.constraint_position_beta = 0.17f;
			config.constraint_max_position_speed = 1.3f;
			config.constraint_regularization = 3.0e-6f;
			config.constraint_warm_start_factor = 0.42f;

			auto reference = ConstraintInteropRunner{config};
			reference.Run(ConstraintRunnerBuffers{1.0f / 120.0f, expected, upload.m_endpoints, upload.m_descriptors});
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto const empty_stats = solver.Stats();
			PR_EXPECT(solver.FrameState() == nullptr);
			PR_EXPECT(empty_stats.m_slot_capacity == 0);
			PR_EXPECT(empty_stats.m_body_capacity == 0);
			PR_EXPECT(empty_stats.m_break_capacity == 0);
			PR_EXPECT(empty_stats.m_dispatch_count == 0);
			PR_EXPECT(empty_stats.m_logical_bytes == 0);
			PR_EXPECT(empty_stats.m_allocated_feature_bytes == 0);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 120.0f, upload, actual);
			PR_EXPECT(solver.FrameState() != nullptr);
			auto const first_stats = solver.Stats();
			PR_EXPECT(first_stats.m_slot_count == 1);
			PR_EXPECT(first_stats.m_active_count == 1);
			PR_EXPECT(first_stats.m_breakable_count == 0);
			PR_EXPECT(first_stats.m_slot_capacity >= 1);
			PR_EXPECT(first_stats.m_body_capacity >= 2);
			PR_EXPECT(first_stats.m_break_capacity == 0);
			PR_EXPECT(first_stats.m_dispatch_count > 0);
			PR_EXPECT(first_stats.m_logical_bytes > 0);
			PR_EXPECT(first_stats.m_allocated_feature_bytes >= first_stats.m_logical_bytes);

			// Report exact logical and retained-capacity costs from the published per-slot and per-body resource formula.
			auto const slot_bytes =
				sizeof(GpuConstraintEndpoint) +
				sizeof(GpuD6ConstraintDesc) +
				sizeof(GpuConstraintBlock) +
				GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow) +
				sizeof(GpuConstraintSolveCache) +
				sizeof(GpuConstraintColourTask) +
				2 * sizeof(uint32_t);
			auto const expected_logical_bytes =
				static_cast<size_t>(first_stats.m_slot_count) * slot_bytes +
				sizeof(GpuConstraintSolverState) +
				actual.size() * sizeof(GpuConstraintPseudoVelocity);
			auto const expected_allocated_bytes =
				static_cast<size_t>(first_stats.m_slot_capacity) * slot_bytes +
				sizeof(GpuConstraintSolverState) +
				static_cast<size_t>(first_stats.m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(first_stats.m_break_capacity) * sizeof(GpuConstraintBreakState);
			PR_EXPECT(first_stats.m_logical_bytes == expected_logical_bytes);
			PR_EXPECT(first_stats.m_allocated_feature_bytes == expected_allocated_bytes);

			for (int body_idx = 0; body_idx != isize(actual); ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(actual[body_idx].o2w, expected[body_idx].o2w, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_ang, expected[body_idx].momentum_ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_lin, expected[body_idx].momentum_lin, 2.0e-3f));
			}

			// A second frame makes the configured warm-start factor observable in both persistent runtimes.
			for (int body_idx = 0; body_idx != isize(actual); ++body_idx)
			{
				expected[body_idx].momentum_ang = float4{};
				expected[body_idx].momentum_lin = float4{};
				actual[body_idx].momentum_ang = float4{};
				actual[body_idx].momentum_lin = float4{};
			}
			reference.Run(ConstraintRunnerBuffers{1.0f / 120.0f, expected, upload.m_endpoints, upload.m_descriptors});
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 120.0f, upload, actual);
			for (int body_idx = 0; body_idx != isize(actual); ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(actual[body_idx].o2w, expected[body_idx].o2w, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_ang, expected[body_idx].momentum_ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_lin, expected[body_idx].momentum_lin, 2.0e-3f));
			}
		}

		// The single-group fused colour sweep must give the same bodies as one dispatch per colour.
		PRUnitTestMethod(HardwareFusedColourSweepMatchesPerColour, Extended)
		{
			// A chain needs two colours and a star hub needs one colour per spoke, and the slot count exceeds one group's threads.
			constexpr auto ChainSlots = 150;
			constexpr auto StarSpokes = 6;
			constexpr auto BodyCount = ChainSlots + 1 + 1 + StarSpokes;
			auto upload = GpuConstraintUpload{};
			auto add_slot = [&](int body_a, int body_b)
			{
				// Lock every axis so each slot exchanges impulses between its two dynamic bodies.
				auto& endpoint = upload.m_endpoints.emplace_back();
				endpoint.body_idx_a = body_a;
				endpoint.body_idx_b = body_b;
				endpoint.flags = GpuConstraintEndpointFlags_Enabled;
				endpoint.generation = 1;
				upload.m_endpoint_identities.emplace_back();
				auto& descriptor = upload.m_descriptors.emplace_back();
				descriptor.frame_a.rotation = float4{0, 0, 0, 1};
				descriptor.frame_b.rotation = float4{0, 0, 0, 1};
				for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
				{
					descriptor.axes[axis_idx].mode = GpuConstraintAxisMode_Locked;
					descriptor.axes[axis_idx].max_force = 1000.0f;
				}
			};
			for (int slot_idx = 0; slot_idx != ChainSlots; ++slot_idx)
				add_slot(slot_idx, slot_idx + 1);
			for (int spoke = 0; spoke != StarSpokes; ++spoke)
				add_slot(ChainSlots + 1, ChainSlots + 2 + spoke);

			upload.m_rigid_active_count = isize(upload.m_endpoints);

			// Give every body a distinct velocity so each colour has real work.
			auto initial = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != BodyCount; ++body_idx)
			{
				auto rigid_body = MakeConstraintGpuBody(1.0f + 0.01f * body_idx);
				auto const phase = 0.37f * body_idx;
				rigid_body.VelocityWS(v8motion{v4{std::sin(phase), std::cos(phase), 0.5f * std::sin(2 * phase), 0}, v4{0.3f * std::cos(phase), 0.2f, -0.4f * std::sin(phase), 0}});
				initial.push_back(PackDynamics(rigid_body, body_idx));
			}

			// Solve two frames per form so retained warm starts are compared as well.
			auto run = [&](int fused_limit, int& dispatch_count)
			{
				auto config = EngineConfig{};
				config.solver_iterations = 3;
				config.push_out_iterations = 2;
				config.constraint_warm_start_factor = 0.8f;
				config.fused_colour_sweep_limit = fused_limit;
				auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
				auto bodies = initial;
				solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies);
				solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies);
				dispatch_count = solver.Stats().m_dispatch_count;
				return bodies;
			};
			auto per_colour_dispatches = 0;
			auto fused_dispatches = 0;
			auto const per_colour = run(0, per_colour_dispatches);
			auto const fused = run(256, fused_dispatches);
			PR_EXPECT(fused_dispatches < per_colour_dispatches);
			for (int body_idx = 0; body_idx != BodyCount; ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(fused[body_idx].o2w, per_colour[body_idx].o2w, 1.0e-5f));
				PR_EXPECT(FEqlAbsolute(fused[body_idx].momentum_lin, per_colour[body_idx].momentum_lin, 1.0e-5f));
				PR_EXPECT(FEqlAbsolute(fused[body_idx].momentum_ang, per_colour[body_idx].momentum_ang, 1.0e-5f));
			}
		}

		// Slots that each anchor one body to the world need only one colour, so the per-colour form records one pass per sweep, like the fused form.
		PRUnitTestMethod(HardwareWorldAnchoredSlotsRecordOneColour, Extended)
		{
			// More slots than one group's threads, so the per-colour form is not limited by the slot count
			constexpr auto BodyCount = 150;
			auto upload = GpuConstraintUpload{};
			for (int body_idx = 0; body_idx != BodyCount; ++body_idx)
			{
				// Lock every axis of the body to the world
				auto& endpoint = upload.m_endpoints.emplace_back();
				endpoint.body_idx_a = body_idx;
				endpoint.body_idx_b = -1;
				endpoint.flags = GpuConstraintEndpointFlags_Enabled;
				endpoint.generation = 1;
				upload.m_endpoint_identities.emplace_back();
				auto& descriptor = upload.m_descriptors.emplace_back();
				descriptor.frame_a.rotation = float4{0, 0, 0, 1};
				descriptor.frame_b.rotation = float4{0, 0, 0, 1};
				for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
				{
					descriptor.axes[axis_idx].mode = GpuConstraintAxisMode_Locked;
					descriptor.axes[axis_idx].max_force = 1000.0f;
				}
			}
			upload.m_rigid_active_count = isize(upload.m_endpoints);

			// Give every body a distinct velocity so the locks have work to do
			auto initial = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != BodyCount; ++body_idx)
			{
				auto rigid_body = MakeConstraintGpuBody(1.0f + 0.01f * body_idx);
				auto const phase = 0.37f * body_idx;
				rigid_body.VelocityWS(v8motion{v4{std::sin(phase), std::cos(phase), 0, 0}, v4{0.3f * std::cos(phase), 0.2f, -0.4f * std::sin(phase), 0}});
				initial.push_back(PackDynamics(rigid_body, body_idx));
			}

			// Solve one frame in each form and compare both the work recorded and the results
			auto run = [&](int fused_limit, int& dispatch_count)
			{
				auto config = EngineConfig{};
				config.solver_iterations = 3;
				config.push_out_iterations = 2;
				config.constraint_warm_start_factor = 0.8f;
				config.fused_colour_sweep_limit = fused_limit;
				auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
				auto bodies = initial;
				solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies);
				dispatch_count = solver.Stats().m_dispatch_count;
				return bodies;
			};
			auto per_colour_dispatches = 0;
			auto fused_dispatches = 0;
			auto const per_colour = run(0, per_colour_dispatches);
			auto const fused = run(256, fused_dispatches);
			PR_EXPECT(per_colour_dispatches == fused_dispatches);
			for (int body_idx = 0; body_idx != BodyCount; ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(fused[body_idx].o2w, per_colour[body_idx].o2w, 1.0e-5f));
				PR_EXPECT(FEqlAbsolute(fused[body_idx].momentum_lin, per_colour[body_idx].momentum_lin, 1.0e-5f));
				PR_EXPECT(FEqlAbsolute(fused[body_idx].momentum_ang, per_colour[body_idx].momentum_ang, 1.0e-5f));
			}
		}

		// Read a production compute-shader overload latch back through the solver's single diagnostic transaction.
		PRUnitTestMethod(HardwareReportsBreakState, Extended)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto break_states = std::vector<GpuConstraintBreakState>(upload.m_endpoints.size());
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_warm_start_factor = 0.0f;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};

			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies, {}, {}, break_states);

			PR_EXPECT(AllSet(break_states[0].flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(break_states[0].generation == upload.m_endpoints[0].generation);
			PR_EXPECT(break_states[0].substep_index == 0);
			PR_EXPECT(break_states[0].peak_force > desc.m_break_force);

			// Breakable slots add one stable-slot latch to both logical and retained resource accounting.
			auto const stats = solver.Stats();
			auto const slot_bytes =
				sizeof(GpuConstraintEndpoint) +
				sizeof(GpuD6ConstraintDesc) +
				sizeof(GpuConstraintBlock) +
				GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow) +
				sizeof(GpuConstraintSolveCache) +
				sizeof(GpuConstraintColourTask) +
				2 * sizeof(uint32_t);
			auto const expected_logical_bytes =
				slot_bytes +
				sizeof(GpuConstraintSolverState) +
				sizeof(GpuConstraintPseudoVelocity) +
				sizeof(GpuConstraintBreakState);
			auto const expected_allocated_bytes =
				static_cast<size_t>(stats.m_slot_capacity) * slot_bytes +
				sizeof(GpuConstraintSolverState) +
				static_cast<size_t>(stats.m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(stats.m_break_capacity) * sizeof(GpuConstraintBreakState);
			PR_EXPECT(stats.m_breakable_count == 1);
			PR_EXPECT(stats.m_logical_bytes == expected_logical_bytes);
			PR_EXPECT(stats.m_allocated_feature_bytes == expected_allocated_bytes);
		}

		// Preserve coherent stable-slot execution when a real GPU hub requires one hundred edge colours.
		PRUnitTestMethod(HardwareDegreeHundredUsesSerialFallback, Extended)
		{
			constexpr int LeafCount = 100;
			auto rigid_bodies = std::vector<RigidBody>{};
			rigid_bodies.reserve(LeafCount + 1);
			for (int body_idx = 0; body_idx != LeafCount + 1; ++body_idx)
				rigid_bodies.push_back(MakeConstraintGpuBody(1.0f, m4x4::Translation(0.01f * body_idx, 0, 0)));
			rigid_bodies[0].VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});

			auto body_ptrs = std::vector<RigidBody*>{};
			body_ptrs.reserve(rigid_bodies.size());
			for (auto& body : rigid_bodies)
				body_ptrs.push_back(&body);

			auto constraints = ConstraintSet{};
			for (int leaf_idx = 0; leaf_idx != LeafCount; ++leaf_idx)
			{
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = BodyRef::Rigid(rigid_bodies[0]);
				desc.m_frame_b.m_body = BodyRef::Rigid(rigid_bodies[leaf_idx + 1]);
				desc.m_linear[0] = MakeConstraintGpuLockedAxis();
				constraints.Add(desc);
			}

			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{};
			bodies.reserve(rigid_bodies.size());
			for (int body_idx = 0; body_idx != isize(rigid_bodies); ++body_idx)
				bodies.push_back(PackDynamics(rigid_bodies[body_idx], body_idx));
			auto blocks = std::vector<GpuConstraintBlock>(LeafCount);
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.constraint_warm_start_factor = 0.0f;

			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies, blocks);

			for (auto const& block : blocks)
				PR_EXPECT(block.colour == MaxColours);
			for (auto const& body : bodies)
				PR_EXPECT(ConstraintGpuBodyFinite(body));
		}

		// Retain ordinary warm starts while rejecting identical stable slots owned by a different constraint set.
		PRUnitTestMethod(HardwareWarmStartTracksConstraintSource, Extended)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints_a = ConstraintSet{};
			constraints_a.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.constraint_warm_start_factor = 0.85f;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto rows = std::vector<GpuConstraintRow>(GpuConstraintRowsPerBlock);

			auto first = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(constraints_a, BodyRemap(body_ptrs)), first, {}, rows);
			PR_EXPECT(std::abs(rows[0].bounds.z) > 1.0e-4f);

			// With iterations disabled, any second-frame momentum is solely the retained warm start.
			config.solver_iterations = 0;
			body.VelocityWS(v8motion{});
			auto retained = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(constraints_a, BodyRemap(body_ptrs)), retained);
			PR_EXPECT(std::abs(retained[0].momentum_lin.x) > 1.0e-4f);

			auto constraints_b = ConstraintSet{};
			constraints_b.Add(desc);
			auto replaced = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(constraints_b, BodyRemap(body_ptrs)), replaced);
			PR_EXPECT(std::abs(replaced[0].momentum_lin.x) < 1.0e-6f);
		}

		// An empty constraint overlay must leave ordinary rigid-body output and the established frame transaction bit-identical.
		PRUnitTestMethod(EmptyConstraintSetPreservesOrdinaryStep, Extended)
		{
			auto shape = collision::ShapeBox{v4{0.4f, 0.6f, 0.8f, 0}};
			auto initial_o2w = m4x4::Transform(Normalise(v4{0.2f, 0.7f, -0.3f, 0}), 0.37f, v4{+1.0f, -2.0f, +3.0f, 1});
			auto overlay_body = RigidBody{&shape, initial_o2w, Inertia::Box(shape.m_radius, 2.5f)};
			auto baseline_body = RigidBody{&shape, initial_o2w, Inertia::Box(shape.m_radius, 2.5f)};
			auto const initial_velocity = v8motion{v4{+1.2f, -0.8f, +0.4f, 0}, v4{+0.3f, -0.2f, +0.5f, 0}};
			overlay_body.VelocityWS(initial_velocity);
			baseline_body.VelocityWS(initial_velocity);
			auto overlay_bodies = std::array<RigidBody*, 1>{&overlay_body};
			auto baseline_bodies = std::array<RigidBody*, 1>{&baseline_body};
			auto constraints = ConstraintSet{};
			auto engine = Engine{
				EngineConfig{},
				&SharedTestShaderCache(),
				SharedTestGpu().m_gpu.device(),
				SharedTestGpu().m_gpu.queue(),
			};

			// Warm the ordinary one-body path without touching optional lanes so first-frame setup cannot masquerade as overlay cost.
			auto warmup_body = RigidBody{&shape, initial_o2w, Inertia::Box(shape.m_radius, 2.5f)};
			warmup_body.VelocityWS(initial_velocity);
			auto warmup_bodies = std::array<RigidBody*, 1>{&warmup_body};
			engine.Step(Engine::StepInput{
				.m_bodies = warmup_bodies,
				.m_elapsed_seconds = 1.0f / 120.0f,
			});

			// Submit the optional empty overlay before any feature allocation can weaken the zero-cost assertion.
			engine.Step(Engine::StepInput{
				.m_bodies = overlay_bodies,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 120.0f,
			});
			auto const overlay_stats = engine.LastFeatureStats();
			auto const overlay_profile = engine.LastStepProfile();

			// Repeat the same ordinary path without a constraint pointer and compare the complete published dynamic state.
			engine.Step(Engine::StepInput{
				.m_bodies = baseline_bodies,
				.m_elapsed_seconds = 1.0f / 120.0f,
			});
			auto const baseline_profile = engine.LastStepProfile();
			auto const overlay_o2w = overlay_body.O2W();
			auto const baseline_o2w = baseline_body.O2W();
			auto const overlay_momentum = overlay_body.MomentumWS();
			auto const baseline_momentum = baseline_body.MomentumWS();
			PR_EXPECT(std::memcmp(&overlay_o2w, &baseline_o2w, sizeof(m4x4)) == 0);
			PR_EXPECT(std::memcmp(&overlay_momentum, &baseline_momentum, sizeof(v8force)) == 0);
			PR_EXPECT(overlay_body.Sleeping() == baseline_body.Sleeping());

			// Optional lanes retain no feature storage or work, and the empty overlay adds no submission, wait, or readback.
			PR_EXPECT(overlay_stats.m_constraints.m_declared_count == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_active_count == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_resources.m_dispatch_count == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_resources.m_logical_bytes == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(overlay_stats.m_articulations.m_resources.m_dispatch_count == 0);
			PR_EXPECT(overlay_stats.m_articulations.m_resources.m_logical_bytes == 0);
			PR_EXPECT(overlay_stats.m_articulations.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(overlay_stats.m_coupled.m_resources.m_dispatch_count == 0);
			PR_EXPECT(overlay_stats.m_coupled.m_resources.m_logical_bytes == 0);
			PR_EXPECT(overlay_stats.m_coupled.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(overlay_profile.m_submission_count == baseline_profile.m_submission_count);
			PR_EXPECT(overlay_profile.m_wait_count == baseline_profile.m_wait_count);
			PR_EXPECT(overlay_profile.m_readback_copy_count == baseline_profile.m_readback_copy_count);
		}

		// Exercise the public constrained Engine step and prove a hard world lock corrects drift without physical momentum injection.
		PRUnitTestMethod(EngineStepAppliesWorldLock, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Translation(0.5f, 0, 0), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto const momentum_before = body.MomentumWS();
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);

			engine.Step(1.0f / 60.0f, bodies, constraints);

			PR_EXPECT(body.O2W().pos.x < 0.5f);
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().ang, momentum_before.ang, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().lin, momentum_before.lin, 1.0e-6f));
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(stats.m_constraints.m_declared_count == 1);
			PR_EXPECT(stats.m_constraints.m_active_count == 1);
			PR_EXPECT(stats.m_constraints.m_breakable_count == 0);
			PR_EXPECT(stats.m_constraints.m_slot_capacity >= 1);
			PR_EXPECT(stats.m_constraints.m_body_capacity >= 1);
			PR_EXPECT(stats.m_constraints.m_resources.m_dispatch_count > 0);
			PR_EXPECT(stats.m_constraints.m_resources.m_logical_bytes > 0);
			PR_EXPECT(stats.m_constraints.m_resources.m_allocated_bytes >= stats.m_constraints.m_resources.m_logical_bytes);
			PR_EXPECT(stats.m_articulations.m_articulation_count == 0);
			PR_EXPECT(stats.m_articulations.m_resources.m_dispatch_count == 0);
			PR_EXPECT(stats.m_coupled.m_constraint_count == 0);
			PR_EXPECT(stats.m_coupled.m_resources.m_dispatch_count == 0);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 1);
			PR_EXPECT(stats.m_frame_output.m_readback_bytes > 0);
			PR_EXPECT(stats.m_failure.Succeeded());

			// Removing the live descriptor leaves one reusable stable slot, but declared diagnostics report only caller-visible constraints.
			constraints.Remove(handle);
			engine.Step(1.0f / 60.0f, bodies, constraints);
			auto const& removed_stats = engine.LastFeatureStats();
			PR_EXPECT(constraints.CapacitySlots() == 1);
			PR_EXPECT(removed_stats.m_constraints.m_declared_count == 0);
			PR_EXPECT(removed_stats.m_constraints.m_active_count == 0);
		}

		// Empty simulation input still validates live constraint endpoints while allowing disabled descriptors to remain declared.
		PRUnitTestMethod(EmptyFrameValidatesLiveConstraints, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);

			PR_THROWS(engine.Step(Engine::StepInput{
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			}), std::invalid_argument);

			constraints.SetEnabled(handle, false);
			engine.Step(Engine::StepInput{
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(stats.m_constraints.m_declared_count == 1);
			PR_EXPECT(stats.m_constraints.m_active_count == 0);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 0);
		}

		// Detect an early-substep overload, publish one edge event through the sole frame readback, and require explicit repair.
		PRUnitTestMethod(EngineBreaksAndRepairsConstraint, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto events = std::vector<ConstraintBreakEvent>{};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.ConstraintsBroken += [&](auto&, auto broken)
			{
				events.insert(events.end(), broken.begin(), broken.end());
			};

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 4,
			});

			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 1);
			PR_EXPECT(events[0].m_constraint == handle);
			PR_EXPECT(events[0].m_substep_index == 0);
			PR_EXPECT(events[0].m_force > desc.m_break_force);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);

			// Routine descriptor updates preserve the broken state and cannot emit duplicate events.
			auto updated = constraints.Get(handle);
			updated.m_linear[0].m_target_position = 0.1f;
			constraints.Update(handle, updated);
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			engine.Step(1.0f / 60.0f, body_ptrs, constraints);
			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 1);

			// Explicit repair reactivates the same stable handle and permits a later overload to generate a new edge event.
			constraints.Repair(handle);
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			engine.Step(1.0f / 60.0f, body_ptrs, constraints);
			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 2);
			PR_EXPECT(events[1].m_constraint == handle);

			// Repeated enable calls do not repair; a real disable-to-enable transition intentionally does.
			constraints.SetEnabled(handle, true);
			PR_EXPECT(constraints.IsBroken(handle));
			constraints.SetEnabled(handle, false);
			PR_EXPECT(constraints.IsBroken(handle));
			constraints.SetEnabled(handle, true);
			PR_EXPECT(!constraints.IsBroken(handle));
		}

		// Run articulation-only coupled work through shared stable-slot storage and the Engine's one-submission substep schedule.
		PRUnitTestMethod(CoupledConstraintsUseSharedGpuStorage, Quick)
		{
			auto [articulation, child] = MakeConstraintGpuArticulation();
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(articulation, child);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto const upload = PackGpuConstraints(constraints, BodyRemap({}, articulation_ptrs));

			auto solver = GpuConstraintSolver{ConstraintTestGpu(), EngineConfig{}};
			PR_EXPECT(!solver.Upload(ConstraintTestGpu().m_job, upload));
			ConstraintTestGpu().m_job.Run();

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			articulation.Sleep();
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 4,
			});

			auto const& profile = engine.LastStepProfile();
			PR_EXPECT(!articulation.Sleeping());
			PR_EXPECT(profile.m_submission_count == 1);
			PR_EXPECT(profile.m_wait_count == 1);
			PR_EXPECT(profile.m_readback_copy_count == 1);
			PR_EXPECT(std::ranges::all_of(articulation.JointPosition(child), [](float value) { return std::isfinite(value); }));
			PR_EXPECT(std::ranges::all_of(articulation.JointVelocity(child), [](float value) { return std::isfinite(value); }));
			PR_EXPECT(IsFinite(articulation.LinkToWorld(child).pos));
		}

		// Suppress connected-body collisions through the filtered broadphase without activating solver rows.
		PRUnitTestMethod(ConnectedBodiesCanSuppressCollision, Quick)
		{
			auto run = [](bool collide_connected)
			{
				auto shape = collision::ShapeSphere{0.5f};
				auto body_a = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
				auto body_b = RigidBody{&shape, m4x4::Translation(0.75f, 0, 0), Inertia::Sphere(shape.m_radius, 1.0f)};
				auto bodies = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
				desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
				desc.m_collide_connected = collide_connected;
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto collision_count = 0;
				auto& engine = SharedEngine();
				ResetEngineForNextTest(engine);
				engine.Collisions += [&](auto&, auto contacts)
				{
					collision_count += isize(contacts);
				};

				engine.Step(1.0f / 60.0f, bodies, constraints);
				return collision_count;
			};

			PR_EXPECT(run(false) == 0);
			PR_EXPECT(run(true) != 0);
		}
	};

	// Wall-clock scaling measurements need the GPU to themselves, so this class never runs at the same time as other test classes.
	PRUnitTestClass(ConstraintGpuSolverScalingTests, Serial)
	{
		// Measure end-to-end GPU solver scaling through at least one hundred thousand active scalar rows.
		PRUnitTestMethod(HardwareHundredThousandRowScaling, Extended)
		{
			constexpr auto SmallSlotCount = 8334;
			constexpr auto LargeSlotCount = 16667;
			constexpr auto SampleCount = 5;
			auto make_upload = [](int slot_count)
			{
				auto upload = GpuConstraintUpload{};
				upload.m_endpoints.resize(slot_count);
				upload.m_endpoint_identities.resize(slot_count);
				upload.m_descriptors.resize(slot_count);
				upload.m_rigid_active_count = slot_count;
				for (int slot_idx = 0; slot_idx != slot_count; ++slot_idx)
				{
					auto& endpoint = upload.m_endpoints[slot_idx];
					endpoint.body_idx_a = slot_idx;
					endpoint.body_idx_b = -1;
					endpoint.flags = GpuConstraintEndpointFlags_Enabled;
					endpoint.generation = 1;

					auto& descriptor = upload.m_descriptors[slot_idx];
					descriptor.frame_a.rotation = float4{0, 0, 0, 1};
					descriptor.frame_b.rotation = float4{0, 0, 0, 1};
					for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
					{
						descriptor.axes[axis_idx].mode = GpuConstraintAxisMode_Locked;
						descriptor.axes[axis_idx].max_force = 1000.0f;
					}
				}
				return upload;
			};
			auto make_bodies = [](int body_count)
			{
				auto rigid_body = MakeConstraintGpuBody();
				rigid_body.VelocityWS(v8motion{v4{0.1f, -0.2f, 0.3f, 0}, v4{0.4f, -0.5f, 0.6f, 0}});
				auto packed = PackDynamics(rigid_body, 0);
				return std::vector<GpuRigidBody>(body_count, packed);
			};
			auto const small_upload = make_upload(SmallSlotCount);
			auto const large_upload = make_upload(LargeSlotCount);
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_warm_start_factor = 0.0f;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};

			// Allocate the maximum resources before timing so the comparison measures steady-state work rather than one-time growth.
			auto warmup_bodies = make_bodies(LargeSlotCount);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, large_upload, warmup_bodies);
			auto const warmup_stats = solver.Stats();

			// Load the smaller working set against retained maximum capacity so logical and allocated accounting must diverge correctly.
			auto small_probe_bodies = make_bodies(SmallSlotCount);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, small_upload, small_probe_bodies);
			auto const small_stats = solver.Stats();
			auto const slot_bytes =
				sizeof(GpuConstraintEndpoint) +
				sizeof(GpuD6ConstraintDesc) +
				sizeof(GpuConstraintBlock) +
				GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow) +
				sizeof(GpuConstraintSolveCache) +
				sizeof(GpuConstraintColourTask) +
				2 * sizeof(uint32_t);
			auto const expected_small_logical_bytes =
				static_cast<size_t>(SmallSlotCount) * slot_bytes +
				sizeof(GpuConstraintSolverState) +
				static_cast<size_t>(SmallSlotCount) * sizeof(GpuConstraintPseudoVelocity);
			PR_EXPECT(small_stats.m_logical_bytes == expected_small_logical_bytes);
			PR_EXPECT(small_stats.m_allocated_feature_bytes == warmup_stats.m_allocated_feature_bytes);
			PR_EXPECT(small_stats.m_logical_bytes < small_stats.m_allocated_feature_bytes);

			// Restore the larger workload immediately before timing so both measured sizes start from an active, saturated GPU.
			warmup_bodies = make_bodies(LargeSlotCount);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, large_upload, warmup_bodies);
			auto measure = [&](GpuConstraintUpload const& upload)
			{
				auto bodies = make_bodies(isize(upload.m_endpoints));
				auto const begin = std::chrono::steady_clock::now();
				solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies);
				auto const elapsed = std::chrono::steady_clock::now() - begin;
				for (auto const& body : bodies)
					PR_EXPECT(ConstraintGpuBodyFinite(body));
				return std::chrono::duration<double, std::milli>(elapsed).count();
			};

			// Interleaved medians reject transient GPU clock and queue noise without concealing persistent superlinear work.
			auto small_samples = std::array<double, SampleCount>{};
			auto large_samples = std::array<double, SampleCount>{};
			for (int sample = 0; sample != SampleCount; ++sample)
			{
				small_samples[sample] = measure(small_upload);
				large_samples[sample] = measure(large_upload);
			}
			std::ranges::sort(small_samples);
			std::ranges::sort(large_samples);
			auto const small_ms = small_samples[SampleCount / 2];
			auto const large_ms = large_samples[SampleCount / 2];
			auto const small_rows = SmallSlotCount * GpuConstraintRowsPerBlock;
			auto const large_rows = LargeSlotCount * GpuConstraintRowsPerBlock;
			log_stream() << std::format(
				"  [benchmark] {} constraint rows: {:.3f} ms; {} rows: {:.3f} ms ({:.2f}x time for {:.2f}x rows)\n",
				small_rows,
				small_ms,
				large_rows,
				large_ms,
				large_ms / small_ms,
				static_cast<double>(large_rows) / small_rows);
			PR_EXPECT(large_rows >= 100000);
			PR_EXPECT(large_ms / small_ms < 2.2);

			// Repeated smaller submissions must reuse the maximum allocation, whose accounting follows the capacity formula exactly.
			auto const final_stats = solver.Stats();
			auto const expected_allocated_bytes =
				static_cast<size_t>(final_stats.m_slot_capacity) * slot_bytes +
				sizeof(GpuConstraintSolverState) +
				static_cast<size_t>(final_stats.m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(final_stats.m_break_capacity) * sizeof(GpuConstraintBreakState);
			PR_EXPECT(final_stats.m_slot_capacity == warmup_stats.m_slot_capacity);
			PR_EXPECT(final_stats.m_body_capacity == warmup_stats.m_body_capacity);
			PR_EXPECT(final_stats.m_allocated_feature_bytes == warmup_stats.m_allocated_feature_bytes);
			PR_EXPECT(final_stats.m_allocated_feature_bytes == expected_allocated_bytes);
		}
	};
}
#endif
