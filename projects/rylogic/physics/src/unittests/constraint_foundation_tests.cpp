//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return a minimally valid D6 descriptor between two endpoints.
		D6ConstraintDesc MakeD6(BodyRef body_a, BodyRef body_b)
		{
			auto desc = D6ConstraintDesc{};
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
			desc.m_linear[1] = LockedAxis();
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
			desc.m_linear[1] = LockedAxis();
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

	// Frame-local identity lookup must preserve borrowed ownership, input order, and packed endpoint semantics.
	PRUnitTestClass(ConstraintBodyRemapTests)
	{
		// Exercise full-width and invalid identities through the existing protected rigid-body identity seam.
		struct IdentifiedBody : RigidBody
		{
			// Construct a fixture body with the specified stable identity.
			explicit IdentifiedBody(uint64_t id)
			{
				// No global identity generator or production API is changed by the fixture.
				m_id = BodyId{id};
			}
		};

		// Reordered domains and a new rigid prefix must preserve physical identity and descriptor bytes.
		PRUnitTestMethod(PreservesDomainsOrderAndPackedEndpoints, Quick)
		{
			// Different tree sizes make forest offsets observable; overlapping numeric identities must remain separate domains.
			auto [tree_a, child_a] = MakeConstraintTree(3.0f);
			auto builder = ArticulationBuilder{};
			auto const root_b = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.25f, 1.0f)});
			auto const middle_b = builder.AddLink(root_b, ArticulationJointDesc::Fixed(), ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.25f, 1.0f)});
			auto const last_b = builder.AddLink(middle_b, ArticulationJointDesc::Fixed(), ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.25f, 1.0f)});
			auto tree_b = builder.Build();
			auto body_a = IdentifiedBody{tree_a.Id().m_value};
			auto body_b = IdentifiedBody{0x100000015ULL};
			auto body_c = IdentifiedBody{0x200000015ULL};
			auto prefix = IdentifiedBody{0x8000000000000015ULL};
			auto const refs = std::array{
				std::pair{BodyRef::Rigid(body_a), BodyRef::Rigid(body_b)},
				std::pair{BodyRef::Link(tree_a, child_a), BodyRef::Rigid(body_c)},
				std::pair{BodyRef::Link(tree_b, last_b), BodyRef::World()},
				std::pair{BodyRef::Link(tree_a, tree_a.Root()), BodyRef::Link(tree_b, middle_b)},
			};
			auto constraints = ConstraintSet{};
			auto handles = std::vector<ConstraintHandle>{};
			for (auto const& [a, b] : refs)
			{
				// Each stored slot has an active row and stable, independently retained endpoint identities.
				auto desc = MakeD6(a, b);
				desc.m_linear[0] = LockedAxis();
				handles.push_back(constraints.Add(desc));
			}
			auto const first_desc = constraints.Get(handles[0]);
			constraints.Remove(handles[0]);
			handles[0] = constraints.Add(first_desc);

			// These expected packed indices are explicit input-layout oracles, independent of either lookup implementation.
			auto const body_orders = std::array{
				std::vector<RigidBody*>{&body_a, &body_b, &body_c},
				std::vector<RigidBody*>{&body_c, &body_a, &body_b},
				std::vector<RigidBody*>{&prefix, &body_b, &body_c, &body_a},
			};
			auto const tree_orders = std::array{
				std::array<Articulation*, 2>{&tree_a, &tree_b},
				std::array<Articulation*, 2>{&tree_b, &tree_a},
				std::array<Articulation*, 2>{&tree_a, &tree_b},
			};
			auto const expected_indices = std::array{
				std::array<std::pair<int, int>, 4>{{{0, 1}, {4, 2}, {7, -1}, {3, 6}}},
				std::array<std::pair<int, int>, 4>{{{1, 2}, {7, 0}, {5, -1}, {6, 4}}},
				std::array<std::pair<int, int>, 4>{{{3, 1}, {5, 2}, {8, -1}, {4, 7}}},
			};
			auto original_descriptors = std::vector<GpuD6ConstraintDesc>{};
			for (int scenario = 0; scenario != 3; ++scenario)
			{
				// Every frame constructs and destroys its own remap; no iterator or prior layout is retained.
				auto const& bodies = body_orders[scenario];
				auto const& trees = tree_orders[scenario];
				auto remap = BodyRemap(bodies, trees);
				PR_EXPECT(remap.BodyCount() == isize(bodies));
				PR_EXPECT(remap.ArticulationCount() == isize(trees));
				for (int index = 0; index != isize(bodies); ++index)
				{
					// Indexed access and stable-ID resolution must agree with the original pointer stream.
					PR_EXPECT(&remap.Body(index) == bodies[index]);
					PR_EXPECT(&remap.MutableBody(index) == bodies[index]);
					PR_EXPECT(remap.Resolve(BodyRef::Rigid(*bodies[index])) == index);
				}
				auto forest_offset = 0;
				for (int index = 0; index != isize(trees); ++index)
				{
					// Forest offsets exclude the rigid prefix, while packed proxy indices include it.
					PR_EXPECT(&remap.ArticulationBody(index) == trees[index]);
					PR_EXPECT(&remap.MutableArticulation(index) == trees[index]);
					PR_EXPECT(remap.ArticulationLinkOffset(index) == forest_offset);
					for (int link_index = 0; link_index != trees[index]->LinkCount(); ++link_index)
					{
						// Resolve every valid handle, not only the links referenced by user constraints.
						auto const link = trees[index]->LinkAt(link_index);
						auto const endpoint = remap.ResolveEndpoint(BodyRef::Link(*trees[index], link));
						PR_EXPECT(endpoint.IsLink() && endpoint.m_articulation_index == index);
						PR_EXPECT(endpoint.m_link_index == link_index && endpoint.m_link == link);
						PR_EXPECT(endpoint.m_packed_body_index == isize(bodies) + forest_offset + link_index);
					}
					forest_offset += trees[index]->LinkCount();
				}

				// Actual CPU packing must emit the expected endpoint bytes, flags, generations, and unchanged descriptors.
				auto const packed = PackGpuConstraints(constraints, remap);
				PR_EXPECT(packed.m_endpoints.size() == refs.size());
				PR_EXPECT(packed.m_coupled_articulation_indices == (std::vector<int>{0, 1}));
				if (scenario == 0)
					original_descriptors = packed.m_descriptors;

				for (int slot = 0; slot != isize(refs); ++slot)
				{
					// All expected fields come from fixture inputs, not the remap or another call to the packer.
					auto const& desc = constraints.Get(handles[slot]);
					auto const expected = GpuConstraintEndpoint{
						.body_idx_a = expected_indices[scenario][slot].first,
						.body_idx_b = expected_indices[scenario][slot].second,
						.flags = GpuConstraintEndpointFlags_Enabled | (slot != 0 ? GpuConstraintEndpointFlags_Coupled : 0u),
						.generation = handles[slot].m_generation,
						.break_force = desc.m_break_force,
						.break_torque = desc.m_break_torque,
					};
					PR_EXPECT(memcmp(&packed.m_endpoints[slot], &expected, sizeof(expected)) == 0);
					PR_EXPECT(packed.m_endpoint_identities[slot].m_body_a == refs[slot].first);
					PR_EXPECT(packed.m_endpoint_identities[slot].m_body_b == refs[slot].second);
					PR_EXPECT(memcmp(&packed.m_descriptors[slot], &original_descriptors[slot], sizeof(GpuD6ConstraintDesc)) == 0);

					// Both trees participate, so compact mobility link indices equal forest link indices; only the second tree has a nonzero root offset.
					auto const tree_a_index = scenario == 1 ? 1 : 0;
					auto const tree_b_index = 1 - tree_a_index;
					auto const owner_a = slot == 0 ? -1 : slot == 2 ? tree_b_index : tree_a_index;
					auto const owner_b = slot == 3 ? tree_b_index : -1;
					auto const expected_coupled = GpuCoupledConstraintEndpoint{
						.articulation_idx_a = owner_a,
						.link_idx_a = owner_a < 0 ? -1 : expected.body_idx_a - isize(bodies),
						.mobility_idx_a = owner_a < 0 ? -1 : expected.body_idx_a - isize(bodies),
						.root_link_idx_a = owner_a < 0 ? -1 : owner_a == 0 ? 0 : trees[0]->LinkCount(),
						.articulation_idx_b = owner_b,
						.link_idx_b = owner_b < 0 ? -1 : expected.body_idx_b - isize(bodies),
						.mobility_idx_b = owner_b < 0 ? -1 : expected.body_idx_b - isize(bodies),
						.root_link_idx_b = owner_b < 0 ? -1 : owner_b == 0 ? 0 : trees[0]->LinkCount(),
					};
					PR_EXPECT(memcmp(&packed.m_coupled_endpoints[slot], &expected_coupled, sizeof(expected_coupled)) == 0);
				}
				log_stream() << "scenario=" << scenario << " rigid_prefix=" << bodies.size() << " links=" << forest_offset << " packed_endpoint_oracle=1\n";
			}
			PR_EXPECT(body_a.Id().m_value == tree_a.Id().m_value);
			PR_EXPECT(tree_a.LinkCount() == 2 && tree_b.LinkCount() == 3);
		}

		// Sparse full-width keys with colliding hash slots must still resolve by identity and insertion order.
		PRUnitTestMethod(SparseCollidingIdentities, Quick)
		{
			// Sharing eight low hash bits forces collisions in the existing reserved twelve-entry table without private container access.
			auto bodies = std::vector<IdentifiedBody>{};
			bodies.reserve(12);
			auto const first_id = uint64_t{0x1000000000000000ULL};
			for (auto id = first_id; id != first_id + 65536 && bodies.size() != 12; ++id)
			{
				// Search a bounded high-identity corpus rather than making fixture setup depend on a particular hash's long-period behaviour.
				if ((std::hash<uint64_t>{}(id) & 255U) == 0)
					bodies.emplace_back(id);
			}
			if (bodies.size() != 12)
				throw std::runtime_error("The fixture corpus did not produce twelve colliding identity hashes");

			// Retain addresses only after the object vector has reached its final size.
			auto pointers = std::vector<RigidBody*>{};
			for (auto& body : bodies)
			{
				// Pointer storage is complete before remapping; object relocation cannot invalidate borrowed addresses.
				pointers.push_back(&body);
			}
			for (int order = 0; order != 2; ++order)
			{
				// Both forward and reversed collision chains must preserve the caller's current insertion order.
				auto remap = BodyRemap(pointers);
				for (int index = 0; index != isize(pointers); ++index)
				{
					// A linear pointer-stream oracle is independent of sorted or hashed lookup.
					PR_EXPECT(remap.Resolve(BodyRef::Rigid(*pointers[index])) == index);
					PR_EXPECT(&remap.Body(index) == pointers[index]);
				}
				auto absent = BodyRef::Rigid(*pointers.front());
				absent.m_body_id.m_value ^= uint64_t{1} << 63;
				PR_THROWS(remap.Resolve(absent), std::invalid_argument);
				std::reverse(pointers.begin(), pointers.end());
			}
			auto duplicate = bodies[4];
			pointers.push_back(&duplicate);
			PR_THROWS((void)BodyRemap(pointers), std::invalid_argument);
			log_stream() << "sparse64=12 shared_low_hash_bits=8 forward_reverse_and_missing=1 duplicate_copy_rejected=1\n";
		}

		// Empty, malformed, missing, stale, and duplicate inputs keep their existing rejection boundaries.
		PRUnitTestMethod(RejectsInvalidInputsAndEndpoints, Quick)
		{
			// Empty domains admit only the identity-free world endpoint and no indexed body access.
			auto empty = BodyRemap({});
			PR_EXPECT(empty.BodyCount() == 0 && empty.ArticulationCount() == 0);
			PR_EXPECT(empty.Resolve(BodyRef::World()) == -1);
			PR_THROWS(empty.Body(0), std::out_of_range);
			PR_THROWS(empty.MutableBody(-1), std::out_of_range);
			PR_THROWS(empty.ArticulationBody(0), std::out_of_range);
			PR_THROWS(empty.MutableArticulation(-1), std::out_of_range);
			PR_THROWS(empty.ArticulationLinkOffset(0), std::out_of_range);
			auto invalid = IdentifiedBody{0};
			auto valid = IdentifiedBody{0x100000007ULL};
			PR_THROWS((void)BodyRemap(std::array<RigidBody*, 1>{nullptr}), std::invalid_argument);
			PR_THROWS((void)BodyRemap(std::array<RigidBody*, 1>{&invalid}), std::invalid_argument);
			PR_THROWS((void)BodyRemap(std::array<RigidBody*, 2>{&valid, &valid}), std::invalid_argument);
			PR_THROWS((void)BodyRemap({}, std::array<Articulation*, 1>{nullptr}), std::invalid_argument);
			PR_THROWS(empty.Resolve(BodyRef::Rigid(valid)), std::invalid_argument);

			// Construction failure cannot change caller-owned objects; the same valid inputs remain usable.
			auto [tree_a, child_a] = MakeConstraintTree(0.0f);
			auto [tree_b, child_b] = MakeConstraintTree(2.0f);
			PR_THROWS((void)BodyRemap({}, std::array<Articulation*, 2>{&tree_a, &tree_a}), std::invalid_argument);
			auto remap = BodyRemap(std::array<RigidBody*, 1>{&valid}, std::array<Articulation*, 1>{&tree_a});
			PR_EXPECT(remap.Resolve(BodyRef::Rigid(valid)) == 0);
			PR_THROWS(remap.Body(-1), std::out_of_range);
			PR_THROWS(remap.MutableBody(1), std::out_of_range);
			PR_THROWS(remap.ArticulationBody(-1), std::out_of_range);
			PR_THROWS(remap.MutableArticulation(1), std::out_of_range);
			PR_THROWS(remap.ArticulationLinkOffset(-1), std::out_of_range);
			PR_THROWS(remap.ArticulationLinkOffset(1), std::out_of_range);
			PR_THROWS(remap.Resolve(BodyRef::Link(tree_b, child_b)), std::invalid_argument);
			auto endpoint = BodyRef::Link(tree_a, child_a);
			++endpoint.m_link.m_generation;
			PR_THROWS(remap.Resolve(endpoint), std::out_of_range);
			endpoint = BodyRef::Link(tree_a, child_a);
			endpoint.m_link.m_index = static_cast<uint32_t>(tree_a.LinkCount());
			PR_THROWS(remap.Resolve(endpoint), std::out_of_range);
			endpoint.m_link = child_b;
			PR_THROWS(remap.Resolve(endpoint), std::out_of_range);
			endpoint = BodyRef::Link(tree_a, child_a);
			endpoint.m_articulation_id.m_value |= uint64_t{1} << 48;
			PR_THROWS(remap.Resolve(endpoint), std::invalid_argument);
			endpoint = BodyRef::Rigid(valid);
			endpoint.m_body_id = {};
			PR_THROWS(remap.Resolve(endpoint), std::invalid_argument);

			// World and unknown-kind validation stays separate from either identity lookup domain.
			endpoint = BodyRef::World();
			endpoint.m_body_id = valid.Id();
			PR_THROWS(remap.Resolve(endpoint), std::invalid_argument);
			endpoint = BodyRef::World();
			endpoint.m_articulation_id = tree_a.Id();
			PR_THROWS(remap.Resolve(endpoint), std::invalid_argument);
			endpoint = BodyRef::World();
			endpoint.m_link = child_a;
			PR_THROWS(remap.Resolve(endpoint), std::invalid_argument);
			endpoint = BodyRef::World();
			endpoint.m_type = static_cast<EConstraintBodyType>(100);
			PR_THROWS(remap.Resolve(endpoint), std::invalid_argument);
			auto moved = std::move(tree_b);
			PR_THROWS((void)BodyRemap({}, std::array<Articulation*, 1>{&tree_b}), std::logic_error);
			PR_EXPECT(moved.LinkCount() == 2);
			log_stream() << "empty_null_invalid_duplicate_missing_world_stale_cross_tree_and_moved_from_rejected=1\n";
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
			desc.m_linear[0] = LockedAxis();
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
			desc.m_linear[0] = LockedAxis();
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
			desc.m_linear[0].m_target_position = 0.25f;
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
			desc.m_frame_a = BodyFrame::FromMatrix(desc.m_frame_a.m_body, m4x4::Translation(0.0f, 1.0f, 0.0f));
			desc.m_frame_b = BodyFrame::FromMatrix(desc.m_frame_b.m_body, m4x4::Transform(v4::XAxis(), 0.25f, v4::Origin()));
			desc.m_linear[0] = LockedAxis();
			desc.m_linear[2].m_mode = EConstraintAxisMode::Limited;
			desc.m_linear[2].m_limits = Range<float>{-0.5f, +0.5f};
			desc.m_angular[0].m_mode = EConstraintAxisMode::Driven;
			desc.m_angular[0].m_target_velocity = 1.5f;
			desc.m_angular[0].m_max_force = 12.0f;
			desc.m_angular[1] = LockedAxis();

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
			PR_EXPECT(linear_x.m_axis == 0);
			PR_EXPECT(linear_x.m_mode == EConstraintAxisMode::Locked);
			ExpectNear(linear_x.m_position, 2.0f);
			ExpectSpatial(linear_x.m_jacobian_a, v4{0, 0, -0.25f, 0}, v4{-1, 0, 0, 0});
			ExpectSpatial(linear_x.m_jacobian_b, v4{0, 0, -0.50f, 0}, v4{+1, 0, 0, 0});

			auto const& linear_z = compiled.m_rows[1];
			PR_EXPECT(linear_z.m_kind == EConstraintRowKind::Linear);
			PR_EXPECT(linear_z.m_axis == 2);
			PR_EXPECT(linear_z.m_mode == EConstraintAxisMode::Limited);
			ExpectNear(linear_z.m_position, 0.0f);
			ExpectSpatial(linear_z.m_jacobian_a, v4{+0.25f, +2.0f, 0, 0}, v4{0, 0, -1, 0});
			ExpectSpatial(linear_z.m_jacobian_b, v4{+0.50f, 0, 0, 0}, v4{0, 0, +1, 0});

			auto const& angular_x = compiled.m_rows[2];
			PR_EXPECT(angular_x.m_kind == EConstraintRowKind::Angular);
			PR_EXPECT(angular_x.m_axis == 0);
			PR_EXPECT(angular_x.m_mode == EConstraintAxisMode::Driven);
			ExpectNear(angular_x.m_position, 0.25f);
			ExpectNear(angular_x.m_target_velocity, 1.5f);
			ExpectNear(angular_x.m_max_force, 12.0f);
			ExpectSpatial(angular_x.m_jacobian_a, v4{-1, 0, 0, 0}, v4::Zero());
			ExpectSpatial(angular_x.m_jacobian_b, v4{+1, 0, 0, 0}, v4::Zero());

			auto const& angular_y = compiled.m_rows[3];
			PR_EXPECT(angular_y.m_kind == EConstraintRowKind::Angular);
			PR_EXPECT(angular_y.m_axis == 1);
			PR_EXPECT(angular_y.m_mode == EConstraintAxisMode::Locked);
			ExpectNear(angular_y.m_position, 0.0f);
		}

		// Angular rows predict the exact change of their rotation-vector coordinates, even far from the identity rotation.
		PRUnitTestMethod(AngularRowsMeasureExactRateForLargeRotations, Quick)
		{
			// A large compound relative rotation makes frame A's axes a poor approximation of the rotation-vector rates
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			body_a.SetMassProperties(Inertia{1.0f, 1.0f});
			body_b.SetMassProperties(Inertia{1.0f, 1.0f});
			body_a.O2W(m4x4::Transform(Normalise(v4{0.3f, -1.0f, 0.5f, 0.0f}), 0.7f, v4::Origin()));
			auto const b_rotation = m4x4::Transform(Normalise(v4{-0.5f, 1.0f, 0.7f, 0.0f}), 1.9f, v4::Origin()) * body_a.O2W();
			body_b.O2W(b_rotation);

			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			for (auto& axis : desc.m_angular)
			{
				axis.m_mode = EConstraintAxisMode::Limited;
				axis.m_limits = Range<float>{-0.1f, +0.1f};
			}
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));
			PR_EXPECT(compiled.m_rows.size() == 3);

			// Each row's B Jacobian must predict the measured coordinate change when body B turns slightly about each world axis
			auto const h = 1e-3f;
			for (auto direction : {v4::XAxis(), v4::YAxis(), v4::ZAxis()})
			{
				body_b.O2W(m4x4::Transform(direction, h, v4::Origin()) * b_rotation);
				auto const turned = CompileConstraints(constraints, BodyRemap(body_ptrs));
				for (size_t row = 0; row != compiled.m_rows.size(); ++row)
				{
					auto const measured = (turned.m_rows[row].m_position - compiled.m_rows[row].m_position) / h;
					auto const predicted = Dot3(compiled.m_rows[row].m_jacobian_b.ang, direction);
					PR_EXPECT(std::abs(measured - predicted) < 5e-3f);
				}
			}
			body_b.O2W(b_rotation);
		}

		// Resolve stable endpoints to each frame's current packed indices after reorder and insertion.
		PRUnitTestMethod(EndpointRemapSurvivesBodyOrdering, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto inserted = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.m_linear[0] = LockedAxis();
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
			desc.m_linear[0] = LockedAxis();
			desc.m_frame_b = BodyFrame::FromMatrix(desc.m_frame_b.m_body, m4x4::Translation(1.0f, 0.0f, 0.0f));
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
			desc.m_frame_a = BodyFrame::FromMatrix(desc.m_frame_a.m_body, m4x4::Translation(0.0f, 1.0f, 0.0f));
			desc.m_frame_b = BodyFrame::FromMatrix(desc.m_frame_b.m_body, m4x4::Translation(9.0f, 1.0f, 0.0f));
			desc.m_linear[0] = LockedAxis();
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
			invalid_frame.m_frame_a.m_rotation.w = 2.0f;
			auto invalid_limit = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_limit.m_linear[0].m_mode = EConstraintAxisMode::Limited;
			invalid_limit.m_linear[0].m_limits = Range<float>{+1.0f, -1.0f};
			auto invalid_mode = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_mode.m_linear[0].m_mode = static_cast<EConstraintAxisMode>(100);

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
