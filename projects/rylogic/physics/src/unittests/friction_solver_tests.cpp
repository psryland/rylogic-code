//*********************************************
// Physics Engine
// Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/interop/resolve_runner.h"
#include "src/compute/contact_coulomb.hlsli"
#include "src/compute/contact_coulomb_linearisation.hlsli"
#include "src/compute/coupled_contact_gpu.h"
#include "src/compute/frame_output_gpu.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	// A unit Cartesian child has three independent translations and no rotational response.
	static ArticulationJointDesc CartesianContactJoint()
	{
		// Share the authored joint across the baseline, propagation, sliding, and singular owner cases.
		auto joint = ArticulationJointDesc::Fixed();
		joint.m_dof_count = 3;
		joint.m_axes = {
			ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::XAxis()},
			ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::YAxis()},
			ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::ZAxis()},
		};
		return joint;
	}

	// Borrowed completed GPU state, valid only during the authored-contact observer callback.
	struct ContactOwnerState
	{
		std::span<GpuArticulation const> m_articulations;
		std::span<float const> m_positions;
		std::span<float const> m_velocities;
		std::span<float const> m_seeded_velocities;
		std::span<GpuCoupledContactKrylov const> m_updates;
		std::span<GpuRigidBody const> m_bodies;
		std::span<GpuResolveContact const> m_contacts;
		GpuCollisionCounters const& m_counters;
		GpuCoupledContactState const& m_state;
	};

	// Execute the complete caller budget and observe one zero-based physical update after its submission completes.
	template <typename Observe> static void RunAuthoredContactOwner(GpuArticulationUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuResolveContact const> manifold, GpuMaterial const& material, EngineConfig const& config, float dt, int observed_iteration, Observe&& observe)
	{
		// Owners outlive the submission and its observer; failures retire only an unsubmitted command prefix.
		assert(observed_iteration >= 0 && observed_iteration < config.solver_iterations);
		auto& gpu = SharedTestGpu();
		auto integrator = GpuIntegrator{gpu, config};
		auto aba = GpuArticulationForceAba{gpu};
		auto midpoint = GpuArticulationMidpoint{aba};
		auto proxies = GpuArticulationLinkProxies{aba, config};
		auto solver = GpuCoupledContactSolver{gpu, aba, proxies, config};
		auto counters = D3DPtr<ID3D12Resource>{};
		auto contacts = D3DPtr<ID3D12Resource>{};
		auto materials = D3DPtr<ID3D12Resource>{};
		auto recording = true;
		try
		{
			// Keep the same preparation, warm admission and caller-owned physical sweep budget.
			PR_EXPECT(aba.Upload(gpu.m_job, upload));
			integrator.Upload(gpu.m_job, bodies);
			proxies.Upload(gpu.m_job);
			proxies.Refresh(gpu.m_job, integrator, 0);
			PR_EXPECT(solver.Upload(gpu.m_job, upload));
			auto upload_buffer = [&](D3DPtr<ID3D12Resource>& resource, auto const& values, char const* name)
			{
				// Publish resource ownership before recording a transfer that may throw.
				using T = typename std::remove_cvref_t<decltype(values)>::value_type;
				resource = gpu.CreateResource(compute::ResDesc::Buf<T>(isize(values), {}).usage(compute::EUsage::UnorderedAccess), gpu.m_job, name);
				auto allocation = gpu.m_job.m_upload.Alloc<T>(isize(values));
				std::ranges::copy(values, allocation.template ptr<T>());
				gpu.m_job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
			};
			upload_buffer(counters, std::array{GpuCollisionCounters{.contact_count = isize(manifold)}}, "Test:CouplingCounters");
			upload_buffer(contacts, manifold, "Test:CouplingContacts");
			upload_buffer(materials, std::array{material}, "Test:CouplingMaterials");
			solver.PrepareVelocity(gpu.m_job, dt, isize(bodies), isize(bodies) - isize(upload.m_links), isize(manifold), counters, contacts, integrator.Bodies(), materials, 0);
			solver.ApplyWarmStart(gpu.m_job);

			// Ordinary rigid momenta and articulation root/q/qdot are primary; proxy momentum is not an articulation observer.
			auto output = midpoint.Output();
			auto roots = gpu.m_job.m_readback.Alloc<GpuArticulation>(isize(upload.m_articulations));
			auto positions = gpu.m_job.m_readback.Alloc<float>(isize(upload.m_positions));
			auto velocities = gpu.m_job.m_readback.Alloc<float>(isize(upload.m_velocities));
			auto seeded_velocities = gpu.m_job.m_readback.Alloc<float>(isize(upload.m_velocities));
			auto updates = std::vector<ReadbackAlloc>{};
			auto body_output = gpu.m_job.m_readback.Alloc<GpuRigidBody>(isize(bodies));
			auto contact_output = gpu.m_job.m_readback.Alloc<GpuResolveContact>(isize(manifold));
			auto counter_output = gpu.m_job.m_readback.Alloc<GpuCollisionCounters>(1);
			auto state = gpu.m_job.m_readback.Alloc<GpuCoupledContactState>(1);
			auto readback = [&](auto& allocation, ID3D12Resource* resource)
			{
				// A floating root with no joints has no position stream or corresponding resource to transition.
				if (allocation.m_size == 0)
					return;

				// Restore tracked ownership so observing an early update does not change subsequent solver work.
				auto const source_state = gpu.m_job.m_cmd_list.ResState(resource).Mip0State();
				gpu.m_job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(allocation, resource, 0);
				gpu.m_job.m_barriers.Transition(resource, source_state).Commit();
			};
			readback(seeded_velocities, output.m_velocities);
			for (int iteration = 0; iteration != config.solver_iterations; ++iteration)
			{
				// Record only the selected boundary, without reducing the executed sweep budget.
				solver.SolveVelocityIteration(gpu.m_job);
				updates.push_back(gpu.m_job.m_readback.Alloc<GpuCoupledContactKrylov>(isize(manifold)));
				readback(updates.back(), solver.CandidateWork());
				if (iteration != observed_iteration)
					continue;

				readback(roots, output.m_articulations);
				readback(positions, output.m_positions);
				readback(velocities, output.m_velocities);
				readback(body_output, integrator.Bodies().get());
				readback(contact_output, contacts.get());
				readback(counter_output, counters.get());
				readback(state, solver.FrameState());
			}
			recording = false;
			gpu.m_job.Run();
			auto retained_work = std::vector<GpuCoupledContactKrylov>{};
			for (auto const& allocation : updates)
			{
				// Flatten iteration-major diagnostics without changing the owner execution or its primary state.
				auto records = std::span{allocation.ptr<GpuCoupledContactKrylov>(), manifold.size()};
				retained_work.insert(retained_work.end(), records.begin(), records.end());
			}
			observe(ContactOwnerState{
				.m_articulations = {roots.ptr<GpuArticulation>(), upload.m_articulations.size()},
				.m_positions = {positions.ptr<float>(), upload.m_positions.size()},
				.m_velocities = {velocities.ptr<float>(), upload.m_velocities.size()},
				.m_seeded_velocities = {seeded_velocities.ptr<float>(), upload.m_velocities.size()},
				.m_updates = retained_work,
				.m_bodies = {body_output.ptr<GpuRigidBody>(), bodies.size()},
				.m_contacts = {contact_output.ptr<GpuResolveContact>(), manifold.size()},
				.m_counters = *counter_output.ptr<GpuCollisionCounters>(),
				.m_state = *state.ptr<GpuCoupledContactState>(),
			});
		}
		catch (...)
		{
			// Preserve the original failure rather than leaving recorded references to destroyed test resources.
			auto error = std::current_exception();
			if (recording)
				gpu.m_job.RetireRecordedWork();

			std::rethrow_exception(error);
		}
	}

	// Execute a frozen authored same-tree contact case and check actual primary-derived final velocities.
	static void CheckCartesianContactOwner(std::span<v4 const> speeds, std::span<GpuResolveContact const> manifold, bool supported_sliding, std::filesystem::path const& directory, std::ostream& log)
	{
		// Each child is independent in the mechanical tree; authored contacts create the coupling being tested.
		std::filesystem::create_directories(directory);
		auto builder = ArticulationBuilder{};
		auto const desc = ArticulationLinkDesc{.m_inertia = Inertia::Sphere(1, 1), .m_collide_parent = true};
		auto root = builder.AddFixedRoot(desc);
		auto handles = std::vector{root};
		for (size_t child = 0; child != speeds.size(); ++child)
			handles.push_back(builder.AddLink(root, CartesianContactJoint(), desc));

		auto tree = builder.Build();
		for (size_t child = 0; child != speeds.size(); ++child)
			tree.JointVelocity(handles[child + 1], std::array{speeds[child].x, speeds[child].y, speeds[child].z});

		auto forest = std::array{&tree};
		auto upload = PackGpuArticulations(forest);
		auto bodies = PackGpuArticulationProxies(upload, forest, std::vector<int>(handles.size(), -1), 0);
		auto config = EngineConfig{};
		config.solver_iterations = 10;
		auto const dt = (1.0f / 60) / 3;
		auto const material = GpuMaterial{.friction_static = 0.3f};
		auto const friction = material.friction_static / (1.000001f - material.friction_static);
		auto dump = [&](char const* name, auto const& values)
		{
			// Flush complete typed prefixes before any final-law assertion can terminate this method.
			auto stream = std::ofstream{};
			stream.exceptions(std::ios::failbit | std::ios::badbit);
			stream.open(directory / std::format("{}.bin", name), std::ios::binary);
			stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
			log << std::format("raw name={} count={} stride={}\n", name, values.size(), sizeof(values[0]));
		};
		dump("input-articulations", upload.m_articulations);
		dump("input-links", upload.m_links);
		dump("input-dofs", upload.m_dofs);
		dump("input-positions", upload.m_positions);
		dump("input-velocities", upload.m_velocities);
		dump("input-bodies", bodies);
		dump("input-contacts", manifold);
		dump("input-materials", std::array{material});
		log << std::format("sweeps={} relaxation={} tolerance={} dt={} friction={} participant_degree_from_prefix={} degree_buffer_readback=false\n",
			config.solver_iterations, config.constraint_coupled_relaxation, config.contact_velocity_tolerance, dt, friction, 2 * manifold.size());

		// Reuse the same owner submission for Cartesian and general-topology fixtures.
		RunAuthoredContactOwner(upload, bodies, manifold, material, config, dt, config.solver_iterations - 1, [&](ContactOwnerState const& state)
		{
			// Retain complete final prefixes before a law assertion can stop this method.
			dump("final-articulations", state.m_articulations);
			dump("final-positions", state.m_positions);
			dump("final-velocities", state.m_velocities);
			dump("final-contacts", state.m_contacts);
			dump("final-counters", std::span{&state.m_counters, 1});
			dump("final-state", std::span{&state.m_state, 1});

			// Reconstruct link kinematics from primary root/q/qdot using the existing CPU articulation owner.
			tree.RootToWorld(UnpackGpuTransform(state.m_articulations[0].root_to_world));
			for (size_t child = 1; child != handles.size(); ++child)
			{
				auto const& link = upload.m_links[child];
				tree.JointPosition(handles[child], state.m_positions.subspan(link.position_offset, link.dof_count));
				tree.JointVelocity(handles[child], state.m_velocities.subspan(link.velocity_offset, link.dof_count));
			}
			tree.UpdateKinematics();
			auto valid = state.m_state.valid != 0 && state.m_state.failure_flags == 0 && state.m_state.frame_failure_flags == 0;
			valid &= state.m_counters.contact_count == isize(manifold);
			valid &= std::ranges::equal(state.m_positions, upload.m_positions);
			valid &= All(state.m_articulations[0].root_to_world.rotation == upload.m_articulations[0].root_to_world.rotation);
			valid &= All(state.m_articulations[0].root_to_world.position == upload.m_articulations[0].root_to_world.position);
			for (size_t row = 0; row != manifold.size(); ++row)
			{
				// The fixture has zero levers and identity frames; exact self response is I for root/child and 2I for child/child.
				auto const& contact = state.m_contacts[row];
				auto a = contact.body_idx_a;
				auto b = contact.body_idx_b;
				auto const relative = tree.LinkVelocity(handles[b]).lin - tree.LinkVelocity(handles[a]).lin;
				auto const initial = (b == 0 ? v4::Zero() : speeds[b - 1]) - (a == 0 ? v4::Zero() : speeds[a - 1]);
				auto const tangent = Cross(contact.axis, v4::YAxis());
				auto const impulse = contact.warmstart_impulse;
				auto const lambda = float3{Dot3(impulse, contact.axis), impulse.y, Dot3(impulse, tangent)};
				auto const residual = float3{Dot3(relative, contact.axis), relative.y, Dot3(relative, tangent)};
				auto const weight = a == 0 || b == 0 ? 1.0f : 2.0f;
				auto const error = CoulombLawError(lambda, residual, friction, weight, weight);
				auto const slip = std::hypot(residual.y, residual.z);
				valid &= std::isfinite(error) && error <= config.contact_velocity_tolerance;
				if (supported_sliding)
					valid &= lambda.x > 0 && slip > config.contact_velocity_tolerance;

				log << std::format("row={} start_normal={} start_slip={} lambda=({},{},{}) residual=({},{},{}) normal_error={} full_error={} slip={}\n",
					row, Dot3(initial, contact.axis), std::hypot(initial.y, initial.z), lambda.x, lambda.y, lambda.z, residual.x, residual.y, residual.z,
					std::abs(std::min(weight * lambda.x, residual.x)), error, slip);
			}
			log.flush();
			PR_EXPECT(valid);
		});
	}

	// Actual-owner contact laws with angular response, mixed participants, and disconnected components.
	PRUnitTestClass(ArticulatedGeneralTopologyGpuTests)
	{
		// Check either an interior sticking witness or a supported-sliding witness on the same authored geometry.
		static void CheckGeneralTopologyContactOwner(bool supported_sliding, std::filesystem::path const& directory, std::ostream& log)
		{
			// Each moving tree has a floating root and one offset revolute child with asymmetric mass properties.
			auto build_tree = [](m4x4 const& pose)
			{
				// Non-aligned joint frames and a nonzero coordinate expose angular/linear frame mistakes.
				auto builder = ArticulationBuilder{};
				auto root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Box(v4{0.6f, 0.9f, 0.7f, 0}, 1.2f)}, pose, {});
				auto joint = ArticulationJointDesc::Revolute(Normalise(v4{1, 2, -1, 0}), m4x4::Transform(v4::XAxis(), 0.23f, v4{0.5f, -0.2f, 0.1f, 1}), m4x4::Transform(v4::YAxis(), -0.31f, v4{-0.2f, 0.1f, -0.1f, 1}));
				joint.m_initial_position[0] = 0.27f;
				builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = Inertia::Box(v4{0.45f, 0.65f, 0.5f, 0}, 0.8f, v4{0.02f, -0.03f, 0.01f, 0})});
				return builder.Build();
			};
			auto trees = std::array{
				build_tree(m4x4::Transform(Normalise(v4{1, -2, 1, 0}), 0.42f, v4{-0.4f, 0.3f, 0.6f, 1})),
				build_tree(m4x4::Transform(Normalise(v4{-1, 1, 2, 0}), -0.53f, v4{1.1f, 0.1f, -0.2f, 1})),
				build_tree(m4x4::Transform(v4::YAxis(), -0.6f, v4{8, 0.3f, 0.2f, 1})),
			};
			auto forest = std::array{&trees[0], &trees[1], &trees[2]};
			auto shape = collision::ShapeBox{v4{0.8f, 0.6f, 0.7f, 0}};
			auto ordinary = std::array{
				RigidBody{&shape, m4x4::Transform(v4::ZAxis(), 0.45f, v4{2.1f, -0.3f, 0.2f, 1}), Inertia::Box(v4{0.8f, 0.6f, 0.7f, 0}, 1.4f)},
				RigidBody{&shape, m4x4::Translation(8, -1, -0.7f), Inertia::Infinite()},
			};
			auto upload = PackGpuArticulations(forest);
			auto pack_bodies = [&]
			{
				// Ordinary bodies precede proxies; links carry the authoritative packed body mapping.
				auto result = std::vector{PackDynamics(ordinary[0], -1), PackDynamics(ordinary[1], -1)};
				auto proxies = PackGpuArticulationProxies(upload, forest, std::array{-1, -1, -1, -1, -1, -1}, isize(ordinary));
				result.insert(result.end(), proxies.begin(), proxies.end());
				return result;
			};
			auto bodies = pack_bodies();
			auto mapping = std::vector<iv2>(bodies.size(), iv2{-1, -1});
			for (int tree_index = 0; tree_index != isize(trees); ++tree_index)
			{
				// Record local link identity explicitly rather than interpreting global offsets as tree-local indices.
				auto const& header = upload.m_articulations[tree_index];
				for (int link = 0; link != header.link_count; ++link)
					mapping[upload.m_links[header.link_offset + link].proxy_body_index] = iv2{tree_index, link};
			}

			// Every row touches a proxy, as required by this owner; interleave the disconnected tree/static row.
			auto const endpoints = std::array{iv2{3, 5}, iv2{7, 1}, iv2{5, 0}};
			auto const points = std::array{v4{0.6f, 0.35f, -0.4f, 1}, v4{8.2f, -0.1f, 0.4f, 1}, v4{1.8f, -0.4f, 0.7f, 1}};
			auto const normals = std::array{Normalise(v4{1, 2, -1, 0}), Normalise(v4{1, -1, 2, 0}), Normalise(v4{-2, 1, 3, 0})};
			auto tangents = std::array<v4, 3>{};
			auto witness = std::array<v4, 3>{};
			auto manifold = std::array<GpuResolveContact, 3>{};
			auto config = EngineConfig{};
			config.solver_iterations = 10;
			auto const dt = (1.0f / 60) / 3;
			auto const material = GpuMaterial{.friction_static = 0.3f};
			auto const friction = material.friction_static / (1.000001f - material.friction_static);
			for (int row = 0; row != 3; ++row)
			{
				// Both witnesses retain positive support; sliding opposes the prescribed final tangent at the disc boundary.
				tangents[row] = Normalise(Cross(normals[row], v4::YAxis()));
				auto const support = std::array{0.4f, 0.3f, 0.6f}[row];
				auto const tangent_impulse = supported_sliding ? -friction * support : std::array{0.04f, 0.025f, -0.05f}[row];
				witness[row] = support * normals[row] + tangent_impulse * tangents[row];
				auto const a = endpoints[row].x;
				auto const b = endpoints[row].y;
				auto const w2a = InvertOrthonormal(bodies[a].o2w);
				manifold[row] = GpuResolveContact{.axis = w2a.rot * normals[row], .contact_point = w2a * points[row], .b2a = w2a * bodies[b].o2w, .body_idx_a = a, .body_idx_b = b};
			}

			// Pure final world translations give relative slips t0, t1, t2 without angular or joint motion.
			if (supported_sliding)
			{
				// Root velocity is link-local even though the desired translations are authored in world coordinates.
				auto const translations = std::array{v4::Zero(), tangents[0], -tangents[1]};
				for (int tree = 0; tree != isize(trees); ++tree)
					trees[tree].RootVelocity(v8motion{v4::Zero(), Transpose(trees[tree].RootToWorld().rot) * translations[tree]});

				ordinary[0].VelocityWS(v8motion{v4::Zero(), tangents[0] + tangents[2]});
			}

			// Subtract the known feasible impulse response from the chosen final primary state.
			auto requests = std::array<std::vector<ArticulationImpulse>, 3>{};
			auto rigid_requests = std::array<v8force, 2>{};
			for (int row = 0; row != 3; ++row)
			{
				// A receives minus the accepted impulse, B plus; reverse those signs for the free state.
				for (int side = 0; side != 2; ++side)
				{
					auto const body = endpoints[row][side];
					auto const impulse = (side == 0 ? 1.0f : -1.0f) * witness[row];
					auto const owner = mapping[body];
					if (owner.x >= 0)
					{
						// Articulation wrenches are link-local and measured about the link origin.
						auto& tree = trees[owner.x];
						auto const link = tree.LinkAt(owner.y);
						auto const w2l = InvertOrthonormal(tree.LinkToWorld(link));
						auto const local_impulse = w2l.rot * impulse;
						requests[owner.x].push_back(ArticulationImpulse{.m_link = link, .m_impulse = v8force{Cross((w2l * points[row]).w0(), local_impulse), local_impulse}});
					}
					else if (body != 1)
					{
						// Ordinary rigid momenta use a world-oriented wrench about the physical COM.
						auto& rigid = ordinary[body];
						auto const wrench = v8force{Cross(points[row] - rigid.CentreOfMassPositionWS(), impulse), impulse};
						rigid_requests[body] += wrench;
						rigid.MomentumWS(rigid.MomentumWS() + wrench);
					}
				}
			}
			for (int tree = 0; tree != isize(trees); ++tree)
				trees[tree].ApplyImpulses(requests[tree]);

			// Retain represented inputs after CPU response construction, with no integration or contact generation.
			upload = PackGpuArticulations(forest);
			bodies = pack_bodies();
			std::filesystem::create_directories(directory);
			auto dump = [&](char const* name, auto const& values)
			{
				// Exact primary inputs and accepted outputs let a separate observer reconstruct all point responses.
				auto stream = std::ofstream{};
				stream.exceptions(std::ios::failbit | std::ios::badbit);
				stream.open(directory / std::format("{}.bin", name), std::ios::binary);
				stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
				log << std::format("raw name={} count={} stride={}\n", name, values.size(), sizeof(values[0]));
			};
			dump("input-articulations", upload.m_articulations);
			dump("input-links", upload.m_links);
			dump("input-dofs", upload.m_dofs);
			dump("input-positions", upload.m_positions);
			dump("input-velocities", upload.m_velocities);
			dump("input-bodies", bodies);
			dump("input-contacts", manifold);
			dump("input-materials", std::array{material});
			dump("body-tree-link-map", mapping);
			dump("world-points", points);
			dump("world-normals", normals);
			dump("feasible-world-impulses", witness);
			log << std::format("sweeps={} relaxation={} tolerance={} friction={} ordinary_primary_prefix=2 components_from_authored_topology=2 supported_sliding={}\n", config.solver_iterations, config.constraint_coupled_relaxation, config.contact_velocity_tolerance, friction, supported_sliding);

			// Reconstruct point velocity from CPU-owned primary state, never from GPU proxy momentum or cached GPU twists.
			auto point_velocity = [&](int body, v4 point)
			{
				// Link velocity is local at the link origin; rigid velocity is world-oriented at the COM.
				auto const owner = mapping[body];
				if (owner.x >= 0)
				{
					// Apply the point lever before rotating the resulting linear velocity to world.
					auto& tree = trees[owner.x];
					auto const link = tree.LinkAt(owner.y);
					auto const pose = tree.LinkToWorld(link);
					auto const velocity = tree.LinkVelocity(link);
					return pose.rot * (velocity.lin + Cross(velocity.ang, (InvertOrthonormal(pose) * point).w0()));
				}
				auto const velocity = ordinary[body].VelocityWS();
				return velocity.lin + Cross(velocity.ang, point - ordinary[body].CentreOfMassPositionWS());
			};
			auto restore_primary = [&](std::span<GpuArticulation const> roots, std::span<float const> positions, std::span<float const> velocities, std::span<GpuRigidBody const> rigid_bodies)
			{
				// All fixture roots float; their first six speeds precede the one-DOF revolute child.
				for (int index = 0; index != isize(trees); ++index)
				{
					// Configuration comes from the authoritative root/q buffers, not proxy transforms.
					auto& tree = trees[index];
					auto const& root = roots[index];
					auto const qdot = velocities.subspan(root.velocity_offset, 6);
					tree.RootToWorld(UnpackGpuTransform(root.root_to_world));
					tree.RootVelocity(v8motion{v4{qdot[0], qdot[1], qdot[2], 0}, v4{qdot[3], qdot[4], qdot[5], 0}});
					auto const& child = upload.m_links[root.link_offset + 1];
					tree.JointPosition(tree.LinkAt(1), positions.subspan(child.position_offset, child.dof_count));
					tree.JointVelocity(tree.LinkAt(1), velocities.subspan(child.velocity_offset, child.dof_count));
					tree.UpdateKinematics();
				}
				for (int body = 0; body != isize(ordinary); ++body)
					UnpackDynamics(rigid_bodies[body], ordinary[body]);
			};
			auto starting = std::array<v4, 3>{};
			for (int row = 0; row != 3; ++row)
				starting[row] = point_velocity(endpoints[row].y, points[row]) - point_velocity(endpoints[row].x, points[row]);

			// Verify the feasible witness on represented inputs, then restore the exact initial primary state for the GPU owner.
			for (int tree = 0; tree != isize(trees); ++tree)
			{
				// Reversing the construction impulses cancels their complete-tree response.
				for (auto& request : requests[tree])
					request.m_impulse = -request.m_impulse;

				trees[tree].ApplyImpulses(requests[tree]);
			}
			for (int body = 0; body != isize(ordinary); ++body)
				ordinary[body].MomentumWS(ordinary[body].MomentumWS() - rigid_requests[body]);

			auto witness_velocity = std::array<v4, 3>{};
			auto witness_valid = true;
			for (int row = 0; row != 3; ++row)
			{
				// Positive unit scales test the known witness law; actual-result acceptance below uses mechanical response scales.
				auto const& contact = manifold[row];
				auto const& pose_a = bodies[contact.body_idx_a].o2w;
				auto const point = pose_a * contact.contact_point;
				auto const normal = contact.axis;
				auto const seed = Abs(normal.x) < Abs(normal.y) ? v4::XAxis() : v4::YAxis();
				auto const tangent_0 = Normalise(Cross(normal, seed));
				auto const tangent_1 = Cross(normal, tangent_0);
				witness_velocity[row] = point_velocity(endpoints[row].y, point) - point_velocity(endpoints[row].x, point);
				auto const impulse = Transpose(pose_a.rot) * witness[row];
				auto const relative = Transpose(pose_a.rot) * witness_velocity[row];
				auto const lambda = float3{Dot3(impulse, normal), Dot3(impulse, tangent_0), Dot3(impulse, tangent_1)};
				auto const velocity = float3{Dot3(relative, normal), Dot3(relative, tangent_0), Dot3(relative, tangent_1)};
				auto const error = CoulombLawError(lambda, velocity, friction, 1, 1);
				witness_valid &= std::isfinite(error) && error <= config.contact_velocity_tolerance;
				if (supported_sliding)
					witness_valid &= lambda.x > 0 && std::hypot(velocity.y, velocity.z) > config.contact_velocity_tolerance;

				log << std::format("witness row={} normal={} slip={} witness_error={}\n", row, velocity.x, std::hypot(velocity.y, velocity.z), error);
			}
			dump("starting-world-relative-velocities", starting);
			dump("feasible-world-relative-velocities", witness_velocity);
			dump("feasible-primary-velocities", PackGpuArticulations(forest).m_velocities);
			dump("feasible-ordinary-bodies", std::array{PackDynamics(ordinary[0], -1), PackDynamics(ordinary[1], -1)});
			restore_primary(upload.m_articulations, upload.m_positions, upload.m_velocities, bodies);
			log.flush();
			PR_EXPECT(witness_valid);

			// Obtain each exact endpoint self response without changing primary velocity; no same-tree endpoint approximation is needed here.
			auto endpoint_response = [&](int body, v4 point, v4 impulse)
			{
				// The ordinary branch uses the same COM convention as its primary momentum representation.
				auto const owner = mapping[body];
				if (owner.x < 0)
				{
					// Static inverse inertia gives zero response without an artificial mobility floor.
					auto const lever = point - ordinary[body].CentreOfMassPositionWS();
					auto const velocity = ordinary[body].InertiaInvWS() * v8force{Cross(lever, impulse), impulse};
					return velocity.lin + Cross(velocity.ang, lever);
				}
				auto& tree = trees[owner.x];
				auto const link = tree.LinkAt(owner.y);
				auto const pose = tree.LinkToWorld(link);
				auto const w2l = InvertOrthonormal(pose);
				auto const local_point = (w2l * point).w0();
				auto const local_impulse = w2l.rot * impulse;
				auto const request = ArticulationImpulse{.m_link = link, .m_impulse = v8force{Cross(local_point, local_impulse), local_impulse}};
				auto delta = std::vector<float>(tree.DofCount());
				auto link_delta = std::vector<v8motion>(tree.LinkCount());
				detail::ComputeArticulationImpulseResponse(tree, std::span{&request, 1}, delta, link_delta);
				return pose.rot * (link_delta[owner.y].lin + Cross(link_delta[owner.y].ang, local_point));
			};
			auto weights = std::array<v4, 3>{};

			// Use the same ten-sweep production transaction as the Cartesian cases, including both disconnected components in one dispatch.
			RunAuthoredContactOwner(upload, bodies, manifold, material, config, dt, config.solver_iterations - 1, [&](ContactOwnerState const& state)
			{
				// Save all primary prefixes before evaluating the law; only the first two body records are ordinary rigid primary.
				dump("final-articulations", state.m_articulations);
				dump("final-positions", state.m_positions);
				dump("final-velocities", state.m_velocities);
				dump("final-bodies", state.m_bodies);
				dump("final-contacts", state.m_contacts);
				dump("final-counters", std::span{&state.m_counters, 1});
				dump("final-state", std::span{&state.m_state, 1});
				restore_primary(state.m_articulations, state.m_positions, state.m_velocities, state.m_bodies);
				auto valid = state.m_state.valid != 0 && state.m_state.failure_flags == 0 && state.m_state.frame_failure_flags == 0;
				valid &= state.m_counters.contact_count == isize(manifold) && std::ranges::equal(state.m_positions, upload.m_positions);
				for (int tree = 0; tree != isize(trees); ++tree)
				{
					// Physical contact response must not advance a floating configuration.
					valid &= All(state.m_articulations[tree].root_to_world.rotation == upload.m_articulations[tree].root_to_world.rotation);
					valid &= All(state.m_articulations[tree].root_to_world.position == upload.m_articulations[tree].root_to_world.position);
				}
				auto final_relative = std::array<v4, 3>{};
				auto represented_points = std::array<v4, 3>{};
				auto represented_normals = std::array<v4, 3>{};
				for (int row = 0; row != 3; ++row)
				{
					// Use the represented point and A-frame basis consumed by the GPU, not an earlier authored world-space round trip.
					auto const& contact = state.m_contacts[row];
					auto const& pose_a = state.m_bodies[contact.body_idx_a].o2w;
					auto const point = pose_a * contact.contact_point;
					auto const normal = contact.axis;
					auto const seed = Abs(normal.x) < Abs(normal.y) ? v4::XAxis() : v4::YAxis();
					auto const tangent_0 = Normalise(Cross(normal, seed));
					auto const tangent_1 = Cross(normal, tangent_0);
					auto basis = std::array{pose_a.rot * normal, pose_a.rot * tangent_0, pose_a.rot * tangent_1};
					for (int axis = 0; axis != 3; ++axis)
						weights[row][axis] = Dot3(basis[axis], endpoint_response(contact.body_idx_a, point, basis[axis]) + endpoint_response(contact.body_idx_b, point, basis[axis]));

					// Only primary-derived velocities enter the law; the stored proxy pose identifies the frozen contact coordinates.
					represented_points[row] = point;
					represented_normals[row] = basis[0];
					final_relative[row] = point_velocity(contact.body_idx_b, point) - point_velocity(contact.body_idx_a, point);
					auto const relative = Transpose(pose_a.rot) * final_relative[row];
					auto const impulse = contact.warmstart_impulse;
					auto const lambda = float3{Dot3(impulse, normal), Dot3(impulse, tangent_0), Dot3(impulse, tangent_1)};
					auto const residual = float3{Dot3(relative, normal), Dot3(relative, tangent_0), Dot3(relative, tangent_1)};
					auto const error = CoulombLawError(lambda, residual, friction, weights[row].x, 0.5f * (weights[row].y + weights[row].z));
					valid &= std::isfinite(error) && error <= config.contact_velocity_tolerance;
					if (supported_sliding)
						valid &= lambda.x > 0 && std::hypot(residual.y, residual.z) > config.contact_velocity_tolerance;

					log << std::format("row={} bodies={}/{} start_normal_at_authored_point={} lambda=({},{},{}) residual=({},{},{}) normal_error={} full_error={}\n",
						row, contact.body_idx_a, contact.body_idx_b, Dot3(starting[row], normals[row]), lambda.x, lambda.y, lambda.z, residual.x, residual.y, residual.z,
						std::abs(std::min(weights[row].x * lambda.x, residual.x)), error);
				}
				dump("self-response-basis-diagonal", weights);
				dump("represented-world-points", represented_points);
				dump("represented-world-normals", represented_normals);
				dump("final-world-relative-velocities", final_relative);
				log.flush();
				PR_EXPECT(valid);
			});
		}

		// Preserve the existing feasible sticking input while sharing its setup with the sliding variant.
		PRUnitTestMethod(GeneralTopologyAndPointFrames, Extended)
		{
			// The ordinary interior-disc case remains a separate observation.
			CheckGeneralTopologyContactOwner(false, temp_dir() / "GeneralTopologyAndPointFrames", log_stream());
		}

		// Exercise the nonsymmetric sliding Jacobian with angular response and mixed connected participants.
		PRUnitTestMethod(GeneralTopologySupportedSliding, Extended)
		{
			// Positive support and actual nonzero slip must satisfy the ordinary law at the unchanged budget.
			CheckGeneralTopologyContactOwner(true, temp_dir() / "GeneralTopologySupportedSliding", log_stream());
		}
	};

	// Source-constant frozen ten-link geometry and signed-wrench witnesses; no capture files are required at runtime.
	struct FrozenContactInput
	{
		struct Joint
		{
			int m_parent;
			float m_mass;
			v4 m_inertia;
			v4 m_parent_point;
			v4 m_child_point;
			v4 m_position;
			bool m_spherical;
		};
		std::array<Joint, 9> m_joints;
		std::array<float, 25> m_speeds;
		GpuConstraintFrame m_root;
		std::vector<GpuResolveContact> m_contacts;
		std::vector<v4> m_witness;
		std::vector<float> m_seeded_speeds;
		std::vector<float> m_witness_speeds;
	};

#include "src/unittests/frozen_contact_cases.h"

	// Execute a standalone frozen problem; optional supplied primary references check fixture construction before final-law acceptance.
	static void CheckFrozenContactOwner(FrozenContactInput const& input, bool first_update, std::filesystem::path const& directory, std::ostream& log)
	{
		// Keep the physical descriptors and caller budget common to the weak-support and supplied original-workload cases.
		auto const& joints = input.m_joints;
		auto const& speeds = input.m_speeds;
		auto builder = ArticulationBuilder{};
		auto const root_pose = UnpackGpuTransform(input.m_root);
		auto const root_inertia = Inertia{v4{0.0340333357f, 0.0408000015f, 0.0148333339f, 0}, v4{}, 30};
		auto handles = std::vector{builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = root_inertia}, root_pose, {})};
		auto cursor = 6;
		for (auto const& joint_input : joints)
		{
			// Spherical velocity slots are child-joint angular coordinates; hinges use their one X-axis slot.
			auto const rotation = joint_input.m_spherical ? v4{0.707106769f, 0, 0, 0.707106769f} : v4{0, 0, 0, 1};
			auto const parent = UnpackGpuTransform(GpuConstraintFrame{rotation, joint_input.m_parent_point});
			auto const child = UnpackGpuTransform(GpuConstraintFrame{rotation, joint_input.m_child_point});
			auto joint = joint_input.m_spherical ? ArticulationJointDesc::Spherical(parent, child) : ArticulationJointDesc::Revolute(v4::XAxis(), parent, child);
			std::copy_n(&joint_input.m_position.x, joint.PositionCount(), joint.m_initial_position.begin());
			std::copy_n(speeds.begin() + cursor, joint.m_dof_count, joint.m_initial_velocity.begin());
			cursor += joint.m_dof_count;
			handles.push_back(builder.AddLink(handles[joint_input.m_parent], joint, ArticulationLinkDesc{.m_inertia = Inertia{joint_input.m_inertia, v4{}, joint_input.m_mass}, .m_collide_parent = true}));
		}
		auto tree = builder.Build();
		tree.RootVelocity(v8motion{v4{speeds[0], speeds[1], speeds[2], 0}, v4{speeds[3], speeds[4], speeds[5], 0}});
		auto forest = std::array{&tree};
		auto upload = PackGpuArticulations(forest);
		auto bodies = PackGpuArticulationProxies(upload, forest, std::vector<int>(handles.size(), -1), 0);
		auto manifold = input.m_contacts;
		auto config = EngineConfig{};
		config.solver_iterations = 10;
		auto const dt = (1.0f / 60) / 3;
		auto const material = GpuMaterial{.friction_static = 0.6f};
		auto const friction = material.friction_static / (1.000001f - material.friction_static);
		auto dump = [&](char const* name, auto const& values)
		{
			// Retain the represented standalone inputs and actual primary output before any assertion can stop the test.
			auto stream = std::ofstream{};
			stream.exceptions(std::ios::failbit | std::ios::badbit);
			stream.open(directory / std::format("{}.bin", name), std::ios::binary);
			stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
			log << std::format("raw name={} count={} stride={}\n", name, values.size(), sizeof(values[0]));
		};
		std::filesystem::create_directories(directory);
		dump("input-articulations", upload.m_articulations);
		dump("input-links", upload.m_links);
		dump("input-dofs", upload.m_dofs);
		dump("input-positions", upload.m_positions);
		dump("input-velocities", upload.m_velocities);
		dump("input-bodies", bodies);
		dump("input-contacts", manifold);
		dump("input-materials", std::array{material});
		log << std::format("sweeps={} observed_update={} relaxation={} tolerance={} dt={} friction={}\n",
			config.solver_iterations, first_update ? 1 : config.solver_iterations, config.constraint_coupled_relaxation, config.contact_velocity_tolerance, dt, friction);

		// Use endpoint self sums only for residual units; signed whole-tree impulses below include the shared-tree cross response.
		auto request = [&](int body, v4 point, v4 impulse)
		{
			// The point wrench is expressed in the receiving link's frame, not the generating A frame.
			auto const w2l = InvertOrthonormal(tree.LinkToWorld(handles[body]));
			auto const lever = (w2l * point).w0();
			auto const force = w2l.rot * impulse;
			return ArticulationImpulse{.m_link = handles[body], .m_impulse = v8force{Cross(lever, force), force}};
		};
		auto endpoint_response = [&](int body, v4 point, v4 impulse)
		{
			// Derive the self response from current primary configuration at the supplied represented world point.
			auto const response_input = request(body, point, impulse);
			auto delta = std::vector<float>(tree.DofCount());
			auto twists = std::vector<v8motion>(tree.LinkCount());
			detail::ComputeArticulationImpulseResponse(tree, std::span{&response_input, 1}, delta, twists);
			auto const& pose = tree.LinkToWorld(handles[body]);
			auto const lever = (InvertOrthonormal(pose) * point).w0();
			return pose.rot * (twists[body].lin + Cross(twists[body].ang, lever));
		};
		auto seed_requests = std::vector<ArticulationImpulse>{};
		for (int row = 0; row != isize(manifold); ++row)
		{
			// Keep the represented contact point and frozen generating frame for both endpoints.
			auto const& contact = manifold[row];
			auto const& pose_a = bodies[contact.body_idx_a].o2w;
			auto const point = pose_a * contact.contact_point;
			auto const impulse = pose_a.rot * contact.warmstart_impulse;
			seed_requests.push_back(request(contact.body_idx_a, point, -impulse));
			seed_requests.push_back(request(contact.body_idx_b, point, +impulse));
		}

		// Observe the seeded primary state independently; the uploaded free state remains unchanged and is warmed once by the GPU.
		tree.ApplyImpulses(seed_requests);
		auto point_velocity = [&](int body, v4 point)
		{
			// Reconstruct from primary tree state at the frozen world point.
			auto const& pose = tree.LinkToWorld(handles[body]);
			auto const twist = tree.LinkVelocity(handles[body]);
			auto const lever = (InvertOrthonormal(pose) * point).w0();
			return pose.rot * (twist.lin + Cross(twist.ang, lever));
		};
		auto measure = [&](std::span<GpuResolveContact const> contacts, std::span<GpuRigidBody const> frames, bool require_law, char const* name)
		{
			// Evaluate the canonical signed residual and independently report ordinary normal/friction errors.
			auto squared = 0.0f;
			auto valid = true;
			auto relative_velocities = std::vector<v4>(manifold.size());
			auto response = std::vector<float3x3>(manifold.size());
			for (int row = 0; row != isize(manifold); ++row)
			{
				// Contact coordinates belong to the generating A frame, not the final link-frame orientation.
				auto const& contact = contacts[row];
				auto const& pose_a = frames[contact.body_idx_a].o2w;
				auto const point = pose_a * contact.contact_point;
				auto const relative = Transpose(pose_a.rot) * (point_velocity(contact.body_idx_b, point) - point_velocity(contact.body_idx_a, point));
				auto const normal = contact.axis;
				auto const tangent_0 = Normalise(Cross(normal, Abs(normal.x) < Abs(normal.y) ? v4::XAxis() : v4::YAxis()));
				auto const tangent_1 = Cross(normal, tangent_0);

				// Remeasure units at this stage's generating point and basis rather than reuse the input proxy's rounded geometry.
				for (int axis = 0; axis != 3; ++axis)
				{
					// Endpoint self sums scale the law; shared-tree cross response still comes from the actual primary velocity.
					auto const direction = v4{pose_a.rot[axis], 0};
					auto const sum = endpoint_response(contact.body_idx_a, point, direction) + endpoint_response(contact.body_idx_b, point, direction);
					auto const column = Transpose(pose_a.rot) * sum;
					for (int component = 0; component != 3; ++component)
						response[row][component][axis] = column[component];
				}
				auto law = CoulombLinearisation{};
				valid &= LineariseCoulombLaw(response[row], relative.xyz, contact.warmstart_impulse.xyz, normal.xyz, friction, law);
				squared += dot(law.residual, law.residual);
				auto const lambda = float3{Dot3(contact.warmstart_impulse, normal), Dot3(contact.warmstart_impulse, tangent_0), Dot3(contact.warmstart_impulse, tangent_1)};
				auto const velocity = float3{Dot3(relative, normal), Dot3(relative, tangent_0), Dot3(relative, tangent_1)};
				auto const error = CoulombLawError(lambda, velocity, friction, law.normal_weight, law.tangent_weight);
				valid &= std::isfinite(error) && (!require_law || error <= config.contact_velocity_tolerance);
				relative_velocities[row] = relative;
				log << std::format("row={} normal_error={} full_error={} signed_squared={} support={} slip={}\n",
					row, std::abs(std::min(law.normal_weight * lambda.x, velocity.x)), error, dot(law.residual, law.residual), lambda.x, std::hypot(velocity.y, velocity.z));
			}
			dump(name, relative_velocities);
			auto const weights_name = std::format("{}-self-response", name);
			dump(weights_name.c_str(), response);
			log.flush();
			PR_EXPECT(valid && std::isfinite(squared));
			return squared;
		};
		// Reference comparisons check represented primary construction; physical law still uses the unchanged .01 velocity tolerance.
		auto check_primary = [&](std::span<float const> actual, std::span<float const> expected, char const* name)
		{
			dump(name, actual);
			if (expected.empty())
				return;

			auto valid = actual.size() == expected.size();
			auto maximum_error = 0.0f;
			auto primary_scale = 1.0f;
			for (auto value : speeds)
				primary_scale = std::max(primary_scale, std::abs(value));

			for (auto value : expected)
				primary_scale = std::max(primary_scale, std::abs(value));

			// Compare coupled primary vectors normwise: individual outputs can cancel large free/impulse-response terms.
			for (size_t i = 0; i != expected.size() && i != actual.size(); ++i)
			{
				auto const error = std::abs(actual[i] - expected[i]);
				maximum_error = std::max(maximum_error, error);
				valid &= std::isfinite(error) && error <= 128 * std::numeric_limits<float>::epsilon() * primary_scale;
			}
			log << std::format("{} maximum_primary_difference={} reference_scale={} relative_difference={}\n", name, maximum_error, primary_scale, maximum_error / primary_scale);
			log.flush();
			PR_EXPECT(valid);
		};
		check_primary(PackGpuArticulations(forest).m_velocities, input.m_seeded_speeds, "seeded-primary-velocities");
		auto const initial_squared = measure(manifold, bodies, false, "seeded-relative-velocities");

		// Construct the witness directly from the same free primary state, avoiding a seed/application/subtraction round trip.
		tree.RootVelocity(v8motion{v4{speeds[0], speeds[1], speeds[2], 0}, v4{speeds[3], speeds[4], speeds[5], 0}});
		for (int link_index = 1; link_index != tree.LinkCount(); ++link_index)
		{
			auto const& link = upload.m_links[link_index];
			tree.JointVelocity(handles[link_index], std::span{speeds}.subspan(link.velocity_offset, link.dof_count));
		}

		// A feasible impulse witness acts through signed point wrenches, never through an assigned expected velocity.
		auto const& witness = input.m_witness;
		auto witness_contacts = manifold;
		auto witness_requests = std::vector<ArticulationImpulse>{};
		for (int row = 0; row != isize(manifold); ++row)
		{
			// Apply the positive witness once while leaving the GPU's free upload untouched.
			auto const& contact = manifold[row];
			auto const& pose_a = bodies[contact.body_idx_a].o2w;
			auto const point = pose_a * contact.contact_point;
			auto const delta = pose_a.rot * witness[row];
			witness_requests.push_back(request(contact.body_idx_a, point, -delta));
			witness_requests.push_back(request(contact.body_idx_b, point, +delta));
			witness_contacts[row].warmstart_impulse = witness[row];
		}
		tree.ApplyImpulses(witness_requests);
		dump("feasible-contacts", witness_contacts);
		check_primary(PackGpuArticulations(forest).m_velocities, input.m_witness_speeds, "feasible-primary-velocities");
		measure(witness_contacts, bodies, true, "feasible-relative-velocities");

		// Every invocation executes all ten sweeps; the selected readback distinguishes first-step descent from final convergence.
		RunAuthoredContactOwner(upload, bodies, manifold, material, config, dt, first_update ? 0 : config.solver_iterations - 1, [&](ContactOwnerState const& state)
		{
			// Preserve the actual accepted primary state and impulses before reconstructing point velocities.
			dump("all-candidate-work", state.m_updates);
			dump("final-articulations", state.m_articulations);
			dump("final-positions", state.m_positions);
			dump("final-velocities", state.m_velocities);
			dump("final-bodies", state.m_bodies);
			dump("final-contacts", state.m_contacts);
			dump("final-counters", std::span{&state.m_counters, 1});
			dump("final-state", std::span{&state.m_state, 1});
			check_primary(state.m_seeded_velocities, input.m_seeded_speeds, "gpu-seeded-primary-velocities");
			auto valid = state.m_state.valid != 0 && state.m_state.failure_flags == 0 && state.m_state.frame_failure_flags == 0;
			valid &= state.m_counters.contact_count == isize(manifold) && std::ranges::equal(state.m_positions, upload.m_positions);
			valid &= All(state.m_articulations[0].root_to_world.rotation == upload.m_articulations[0].root_to_world.rotation);
			valid &= All(state.m_articulations[0].root_to_world.position == upload.m_articulations[0].root_to_world.position);
			tree.RootToWorld(UnpackGpuTransform(state.m_articulations[0].root_to_world));
			auto const qdot = state.m_velocities;
			tree.RootVelocity(v8motion{v4{qdot[0], qdot[1], qdot[2], 0}, v4{qdot[3], qdot[4], qdot[5], 0}});
			for (int link_index = 1; link_index != tree.LinkCount(); ++link_index)
			{
				// Quaternion position width is distinct from spherical angular velocity width.
				auto const& link = upload.m_links[link_index];
				auto const position_count = joints[link_index - 1].m_spherical ? 4 : 1;
				tree.JointPosition(handles[link_index], state.m_positions.subspan(link.position_offset, position_count));
				tree.JointVelocity(handles[link_index], qdot.subspan(link.velocity_offset, link.dof_count));
			}
			tree.UpdateKinematics();
			auto const final_squared = measure(state.m_contacts, state.m_bodies, !first_update, "final-relative-velocities");
			log << std::format("initial_squared={} observed_squared={}\n", initial_squared, final_squared);
			log.flush();
			PR_EXPECT(valid);
			if (first_update)
				PR_EXPECT(final_squared < initial_squared);
		});
	}

	// A weakly supported floating tree whose Newton direction crosses friction regimes.
	PRUnitTestClass(ArticulatedContactSafeguardGpuTests)
	{
		// The accepted first update must reduce the nonlinear law, not merely its frozen linear equation.
		PRUnitTestMethod(RegimeChangingFirstUpdate, Extended)
		{
			// Observe the first update without changing the configured ten-sweep execution.
			CheckFrozenContactOwner(WeakSupportInput(), true, temp_dir() / "RegimeChangingFirstUpdate", log_stream());
		}

		// The safeguarded owner must meet the existing final physical law on the portable weak-support case.
		PRUnitTestMethod(RegimeChangingFinalLaw, Extended)
		{
			// No projection or joint correction follows this primary-state measurement.
			CheckFrozenContactOwner(WeakSupportInput(), false, temp_dir() / "RegimeChangingFinalLaw", log_stream());
		}
	};

	// Supplied frozen problems have no interleaved integrator or joint correction and retain the original ten-update budget.
	PRUnitTestClass(ArticulatedContactArcGpuTests)
	{
		// Weak productive progress with a poorly resolved finite CGLS step.
		PRUnitTestMethod(FrozenTree68, Extended)
		{
			CheckFrozenContactOwner(tests::FrozenTree68(), false, temp_dir() / "FrozenTree68", log_stream());
		}

		// A small frozen-linear residual does not make the full projected endpoint useful.
		PRUnitTestMethod(FrozenTree91, Extended)
		{
			CheckFrozenContactOwner(tests::FrozenTree91(), false, temp_dir() / "FrozenTree91", log_stream());
		}

		// A full small gradient segment does not establish final ordinary-law convergence.
		PRUnitTestMethod(FrozenTree77, Extended)
		{
			CheckFrozenContactOwner(tests::FrozenTree77(), false, temp_dir() / "FrozenTree77", log_stream());
		}
	};

	// Invoke the production arc selector through the existing runtime-shader test seam, without a production input override.
	PRUnitTestClass(ContactProjectedArcTests)
	{
		// Cover an exhausted Newton ray, shortened original-gradient arc, represented no-change, and invalid arithmetic.
		PRUnitTestMethod(FallbackAndRepresentedNoChange, Quick)
		{
			// This private entry point calls the same selector as the production kernel, with ordinary rigid mechanics for its W products.
			auto const source = std::string{R"(
#include "physics/src/compute/coupled_contact_newton.hlsl"
numthreads(Probe, 1, 1, 1)
void Probe(uint3 id : SV_DispatchThreadID)
{
	GpuResolveContact contact = g_coupled_contact_contacts[0];
	GpuCoupledContactBlock block = g_coupled_contact_blocks[0];
	float3 velocity = CoupledContactRelativeVelocity(contact, contact.contact_point.xyz) - block.target_normal_speed * contact.axis.xyz;
	CoulombLinearisation law;
	bool valid = LineariseCoulombLaw(float3x3(block.response_0.xyz, block.response_1.xyz, block.response_2.xyz), velocity, contact.warmstart_impulse.xyz, contact.axis.xyz, block.friction, law);
	GpuCoupledContactKrylov work = g_contact_krylov[0];
	for (int axis = 0; axis != 3; ++axis)
	{
		work.velocity_derivative[axis] = float4(law.velocity_derivative[axis], 0);
		work.impulse_derivative[axis] = float4(law.impulse_derivative[axis], 0);
	}
	work.residual = float4(-law.residual, 0);
	g_contact_krylov[0] = work;
	float squared = dot(law.residual, law.residual);
	if (valid)
	{
		if (g_coupled_contact.phase == 1)
		{
			bool accepted;
			uint products = 0;
			valid = ContactSearchArc(0, 1, squared, 0.0001f, false, accepted, products);
			g_contact_krylov[0].inverse_weights.zw = float2(products, 0);
		}
		else
		{
			valid = ContactSelectCandidate(0, 1, squared, 0.0001f);
		}
	}
	if (!valid)
		CoupledContactFail(CoupledContactFailureNonFinite);
}
)"};
			auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path().parent_path();
			auto resolver = compute::shader_cache::FileSourceResolver(std::vector<std::filesystem::path>{root / "include", root / "projects" / "rylogic"});
			auto bytecode = compute::ShaderCompiler{}.Source(source).Includes(resolver).EntryPoint(L"Probe").ShaderModel(L"cs_6_0").Optimise(false).DebugInfo().Arg(L"-Gfp").Arg(L"-Gis").Arg(L"-Ges").Arg(L"-WX").Arg(L"-Zpc").Compile();
			auto dump = [&](std::string const& name, auto const& values)
			{
				// Keep the runtime source/image and every represented input/output before assertions.
				auto stream = std::ofstream{};
				stream.exceptions(std::ios::failbit | std::ios::badbit);
				stream.open(temp_dir() / name, std::ios::binary);
				stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
			};
			dump("arc-probe.hlsl", source);
			dump("arc-probe.cso", bytecode);
			auto& gpu = SharedTestGpu();
			auto signature = compute::RootSig(compute::ERootSigFlags::ComputeOnly)
				.U32<GpuCoupledContactParams>(ECBufReg::b0)
				.SRV(ESRVReg::t0).SRV(ESRVReg::t1).SRV(ESRVReg::t2).SRV(ESRVReg::t3).SRV(ESRVReg::t4)
				.SRV(ESRVReg::t5).SRV(ESRVReg::t6).SRV(ESRVReg::t7).SRV(ESRVReg::t8)
				.UAV(EUAVReg::u0).UAV(EUAVReg::u1).UAV(EUAVReg::u2).UAV(EUAVReg::u3).UAV(EUAVReg::u4)
				.UAV(EUAVReg::u5).UAV(EUAVReg::u6).UAV(EUAVReg::u7).UAV(EUAVReg::u8).UAV(EUAVReg::u9)
				.UAV(EUAVReg::u10).UAV(EUAVReg::u11).UAV(EUAVReg::u12)
				.Create(gpu, "ContactArc:Sig");
			auto pso = compute::ComputePSO(signature.get(), bytecode).Create(gpu, "ContactArc:PSO");
			auto const axis = v4{0.707106769f, 0.707106769f, 0, 0};
			auto const lever = v4{-0.80995798f, 0.58648791f, 0, 0};
			auto const inertia = Inertia{v4{0.0868090629f, 0.0868090629f, 0.0027695729f, 0}, v4{}, 125.195346f};

			// Its physical inertia has positive principal values and satisfies their triangle inequalities.
			// At this lever the point response is [[1,1.37,0],[1.37,1.9,0],[0,0,.1]] up to represented rounding.
			auto shape = collision::ShapeBox{v4{1, 1, 1, 0}};
			for (int example = 0; example != 5; ++example)
			{
				// Modes exist only in this test entry point: production has one selector and no override/backend.
				auto fixed = RigidBody{&shape, m4x4::Identity(), Inertia::Infinite()};
				auto moving = RigidBody{&shape, m4x4::Identity(), inertia};
				moving.VelocityWS(v8motion{v4{}, example == 0 ? v4{-1, 1, 0, 0} : v4{}});
				auto bodies = std::array{PackDynamics(fixed, -1), PackDynamics(moving, -1)};
				auto contact = GpuResolveContact{.axis = v4::XAxis(), .contact_point = lever.w1(), .body_idx_a = 0, .body_idx_b = 1};
				auto block = GpuCoupledContactBlock{.response_0 = {1, 1.37f, 0, 0}, .response_1 = {1.37f, 1.9f, 0, 0}, .response_2 = {0, 0, 0.1f, 0}, .friction = 0.9f, .participant_a = -1, .participant_b = 1};
				auto work = GpuCoupledContactKrylov{.inverse_weights = {1, 1, 0, 0}, .step = {-4.29184549f, 3.86266094f, 0, 0}, .response = {7, 8, 9, 0}};
				auto params = GpuCoupledContactParams{.max_contacts = 1, .body_count = 2, .rigid_body_count = 2, .target_count = 2, .velocity_tolerance = 0.01f, .phase = example == 1 ? 1 : 0, .relaxation = 0.9f};
				if (example == 1 || example == 3)
				{
					// Raw no-change and already-converged state must preserve the non-idempotent frictionless base exactly.
					contact.axis = contact.warmstart_impulse = axis;
					block.friction = 0;
					work.step = v4{1e-30f, 1e-30f, 0, 0};
				}
				if (example == 2)
				{
					// Coincident equal-and-opposite wrenches have W=0, while the unresolved target keeps F nonzero.
					contact.body_idx_a = contact.body_idx_b = 1;
					contact.warmstart_impulse = v4::XAxis();
					block.participant_a = block.participant_b = 1;
					block.target_normal_speed = 1;
					block.friction = 0;
					work.step = v4::XAxis();
				}
				if (example == 4)
				{
					// Invalid arithmetic must fail the transaction rather than masquerade as an exhausted finite direction.
					block.target_normal_speed = 1;
					work.step.x = std::numeric_limits<float>::infinity();
				}

				// Bind the actual owner's register layout; unused articulation buffers are separate inert sentinels.
				auto job = GpuJob{gpu.m_gpu, "ContactArc", 0xFF00AAFF, 1};
				auto srvs = std::array<D3DPtr<ID3D12Resource>, 9>{};
				auto uavs = std::array<D3DPtr<ID3D12Resource>, 13>{};
				auto upload = [&](auto const& values, bool writable)
				{
					using T = typename std::remove_cvref_t<decltype(values)>::value_type;
					auto resource = gpu.CreateResource(compute::ResDesc::Buf<T>(isize(values), {}).usage(writable ? compute::EUsage::UnorderedAccess : compute::EUsage::Default), job, "ContactArc:Buffer");
					auto allocation = job.m_upload.Alloc<T>(isize(values));
					std::ranges::copy(values, allocation.template ptr<T>());
					job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
					job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
					return resource;
				};
				srvs[0] = upload(std::array{GpuCollisionCounters{.contact_count = 1}}, false);
				srvs[6] = upload(std::array{block}, false);
				srvs[7] = upload(std::array{contact}, false);
				srvs[8] = upload(bodies, false);
				uavs[0] = upload(std::array{GpuCoupledContactScratch{.candidate_impulse = {0, 0, 0, 1}}}, true);
				uavs[2] = upload(std::array<GpuArticulationSpatialVector, 2>{}, true);
				uavs[4] = upload(std::array{GpuCoupledContactState{.valid = 1}}, true);
				uavs[5] = upload(std::array{iv2{0, 0}, iv2{1, 0}}, true);
				uavs[6] = upload(std::array{iv2{1, 0}}, true);
				uavs[7] = upload(std::array{work}, true);
				for (auto& resource : srvs)
				{
					// Never replace an upload target whose recorded transfer still borrows it.
					if (resource == nullptr)
						resource = upload(std::array<uint32_t, 64>{}, false);
				}
				for (auto& resource : uavs)
				{
					if (resource == nullptr)
						resource = upload(std::array<uint32_t, 64>{}, true);
				}
				auto prefix = std::format("case-{}-", example);
				dump(prefix + "bodies.bin", bodies);
				dump(prefix + "contact.bin", std::array{contact});
				dump(prefix + "block.bin", std::array{block});
				dump(prefix + "work-input.bin", std::array{work});
				dump(prefix + "params.bin", std::array{params});
				for (auto const& resource : srvs)
					job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

				for (auto const& resource : uavs)
					job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

				job.m_barriers.Commit();
				job.m_cmd_list.SetPipelineState(pso.get());
				job.m_cmd_list.SetComputeRootSignature(signature.get());
				job.m_cmd_list.AddComputeRoot32BitConstants(params);
				for (auto const& resource : srvs)
					job.m_cmd_list.AddComputeRootShaderResourceView(resource);

				for (auto const& resource : uavs)
					job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);

				job.m_cmd_list.Dispatch(1, 1, 1);
				auto candidate_output = job.m_readback.Alloc<GpuCoupledContactScratch>(1);
				auto work_output = job.m_readback.Alloc<GpuCoupledContactKrylov>(1);
				auto state_output = job.m_readback.Alloc<GpuCoupledContactState>(1);
				auto read = [&](auto& allocation, int slot)
				{
					job.m_barriers.Transition(uavs[slot].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
					job.m_cmd_list.CopyBufferRegion(allocation, uavs[slot].get(), 0);
				};
				read(candidate_output, 0);
				read(work_output, 7);
				read(state_output, 4);
				job.Run();
				auto const candidate = candidate_output.ptr<GpuCoupledContactScratch>()->candidate_impulse.w0();
				auto const& actual = *work_output.ptr<GpuCoupledContactKrylov>();
				auto const& state = *state_output.ptr<GpuCoupledContactState>();
				dump(prefix + "candidate.bin", std::span{candidate_output.ptr<GpuCoupledContactScratch>(), 1});
				dump(prefix + "work-output.bin", std::span{&actual, 1});
				dump(prefix + "state.bin", std::span{&state, 1});
				log_stream() << std::format("case={} candidate=({},{},{}) W_products={} gradient_products={} valid={} failures={}\n", example, candidate.x, candidate.y, candidate.z, actual.inverse_weights.z, actual.inverse_weights.w, state.valid, state.failure_flags);
				log_stream().flush();
				if (example == 0)
				{
					// The negative-gradient arc accepts its quarter step, not the failed full or half step.
					auto const expected = v4{0.277582873f, 0.249824586f, 0, 0};
					auto const delta = candidate - contact.warmstart_impulse;
					auto const twist = moving.InertiaInvWS() * v8force{Cross(lever, delta), delta};
					auto const response = twist.lin + Cross(twist.ang, lever);
					PR_EXPECT(state.valid != 0 && state.failure_flags == 0);
					PR_EXPECT(FEqlAbsolute(candidate, expected, 2e-6f) && actual.inverse_weights.z == 3 && actual.inverse_weights.w == 1);
					PR_EXPECT(All(actual.operand == delta) && FEqlAbsolute(actual.response, response, 2e-6f));
				}
				else if (example == 4)
				{
					PR_EXPECT(state.valid == 0 && state.failure_flags != 0);
				}
				else
				{
					// Exhaustion clears both retained fields; convergence avoids projecting or perturbing the base.
					PR_EXPECT(state.valid != 0 && state.failure_flags == 0);
					PR_EXPECT(All(candidate == contact.warmstart_impulse) && All(actual.operand == v4{}) && All(actual.response == v4{}));
					PR_EXPECT(example == 2 ? actual.inverse_weights.z > 0 && actual.inverse_weights.w == 1 : actual.inverse_weights.z == 0 && actual.inverse_weights.w == 0);
				}
			}
		}
	};

	// The metric projection constructs a search point; it must not substitute a different contact law.
	PRUnitTestClass(ContactMetricProjectionTests)
	{
		// Compare with the independent double-precision weighted boundary minimizer, not merely cone membership.
		PRUnitTestMethod(NearestScaledCone, Quick)
		{
			// Exercise shader-like flush-to-zero arithmetic without changing other tests' floating-point environment.
			struct FlushSubnormals
			{
				unsigned int m_saved = _mm_getcsr();
				FlushSubnormals()
				{
					_mm_setcsr(m_saved | (1u << 15) | (1u << 6));
				}
				~FlushSubnormals()
				{
					_mm_setcsr(m_saved);
				}
			} flush_subnormals;

			// Scales are the normal/tangent entries of S, not the physical cone slope.
			struct Example
			{
				float3 m_value;
				float m_friction;
				float2 m_scales;
			};
			auto const examples = std::array{
				Example{float3(-1, 1.5f, 0), 1, float2(1, 0.5f)},
				Example{float3(-1, -1.5f, 0), 1, float2(1, 0.5f)},
				Example{float3(1, -2, 3), 1, float2(3, 0.25f)},
				Example{float3(2, 1, 0), 0, float2(1, 0.5f)},
				Example{float3(-2, 1, 0), 0, float2(1, 0.5f)},
				Example{float3(0, 0, 0), 1, float2(1, 0.5f)},
				Example{float3(-2, 1, 0), 1, float2(1, 1)},
				Example{float3(2, 0.5f, 0.5f), 1, float2(1e-20f, 1e-20f)},
				Example{float3(-1e-30f, 1.5e-30f, 0), 1, float2(1e20f, 5e19f)},
				Example{float3(-1e30f, 1.5e30f, 0), 1, float2(1e-20f, 5e-21f)},
				Example{float3(-1, 2, 0), 1e10f, float2(1e20f, 1e-20f)},
				Example{float3(-1e30f, 2e-20f, 0), 1e-20f, float2(1e10f, 1e-10f)},
				Example{float3(-1e30f, 2e20f, 0), 1e-10f, float2(1e5f, 1e-5f)},
				Example{float3(-float_max, float_max, float_max), 1, float2(1, 1)},
				Example{float3(1e38f, 1, 1), 1.2e-38f, float2(1, 1)},
				Example{float3(-1, 1, 1), 1e38f, float2(1, 1)},
				Example{float3(-1, 1e30f, 1e-20f), 1, float2(1, 1)},
			};
			auto cpu_results = std::vector<float3>{};
			auto gpu_inputs = std::vector<v4>{};
			for (auto const& example : examples)
			{
				// Double intermediates can represent every product of these float inputs without the shader's range problem.
				auto const n = static_cast<double>(example.m_value.x);
				auto const y = static_cast<double>(example.m_value.y);
				auto const z = static_cast<double>(example.m_value.z);
				auto const r = std::hypot(y, z);
				auto const mu = static_cast<double>(example.m_friction);
				auto const sn = static_cast<double>(example.m_scales.x);
				auto const st = static_cast<double>(example.m_scales.y);
				auto expected = std::array{n, y, z};
				if (!(n >= 0 && r <= mu * n))
				{
					// The cone boundary minimizes squared S^-1 distance; negative boundary support belongs to the polar cone.
					auto const support = std::max(0.0, (st*st*n + mu*sn*sn*r) / (st*st + mu*mu*sn*sn));
					expected = {support, r != 0 ? mu*support*y/r : 0, r != 0 ? mu*support*z/r : 0};
				}
				auto actual = float3{};
				PR_EXPECT(ProjectCoulombMetricImpulse(example.m_value, float3(1, 0, 0), example.m_friction, example.m_scales, actual));
				auto valid = true;
				for (int axis = 0; axis != 3; ++axis)
					valid &= std::abs(static_cast<double>(actual[axis]) - expected[axis]) <= 8 * std::numeric_limits<float>::epsilon() * std::abs(expected[axis]);

				log_stream() << std::format("input=({},{},{}) mu={} scales=({},{}) actual=({},{},{}) expected=({},{},{})\n",
					example.m_value.x, example.m_value.y, example.m_value.z, example.m_friction, example.m_scales.x, example.m_scales.y,
					actual.x, actual.y, actual.z, expected[0], expected[1], expected[2]);
				PR_EXPECT(valid);
				cpu_results.push_back(actual);
				gpu_inputs.push_back(v4{example.m_value.x, example.m_value.y, example.m_value.z, example.m_friction});
				gpu_inputs.push_back(v4{example.m_scales.x, example.m_scales.y, 0, 0});
			}

			// Run the same finite-range inputs through the actual unoptimised shader helper, separately from the contact owner.
			_mm_setcsr(flush_subnormals.m_saved);
			auto const source = std::string{R"(
#include "physics/src/compute/contact_coulomb_linearisation.hlsli"
StructuredBuffer<float4> inputs : register(t0);
RWStructuredBuffer<float4> outputs : register(u0);
numthreads(Probe, 1, 1, 1)
void Probe(uint3 id : SV_DispatchThreadID)
{
	float4 value = inputs[2*id.x];
	float2 scales = inputs[2*id.x+1].xy;
	float3 result;
	bool valid = ProjectCoulombMetricImpulse(value.xyz, float3(1,0,0), value.w, scales, result);
	outputs[id.x] = float4(result, valid ? 1 : 0);
}
)"};
			auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path().parent_path();
			auto resolver = compute::shader_cache::FileSourceResolver(std::vector<std::filesystem::path>{root / "include", root / "projects" / "rylogic"});
			auto bytecode = compute::ShaderCompiler{}.Source(source).Includes(resolver).EntryPoint(L"Probe").ShaderModel(L"cs_6_0").Optimise(false).DebugInfo().Arg(L"-Gfp").Arg(L"-Gis").Arg(L"-Ges").Arg(L"-WX").Arg(L"-Zpc").Compile();
			auto dump = [&](char const* name, auto const& values)
			{
				// Bind the independent helper dispatch to its exact bytecode and represented inputs.
				auto stream = std::ofstream{};
				stream.exceptions(std::ios::failbit | std::ios::badbit);
				stream.open(temp_dir() / name, std::ios::binary);
				stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
			};
			dump("metric-probe.cso", bytecode);
			dump("metric-probe.hlsl", source);
			dump("metric-probe-inputs.bin", gpu_inputs);
			dump("metric-probe-expected.bin", cpu_results);
			log_stream().flush();
			auto& gpu = SharedTestGpu();
			auto signature = compute::RootSig(compute::ERootSigFlags::ComputeOnly).SRV(ESRVReg::t0).UAV(EUAVReg::u0).Create(gpu, "MetricProjection:Sig");
			auto pso = compute::ComputePSO(signature.get(), bytecode).Create(gpu, "MetricProjection:PSO");
			auto input = D3DPtr<ID3D12Resource>{};
			auto output = D3DPtr<ID3D12Resource>{};
			auto job = GpuJob{gpu.m_gpu, "MetricProjection", 0xFF00AAFF, 1};
			input = gpu.CreateResource(compute::ResDesc::Buf<v4>(isize(gpu_inputs), {}), job, "MetricProjection:Input");
			output = gpu.CreateResource(compute::ResDesc::Buf<v4>(isize(examples), {}).usage(compute::EUsage::UnorderedAccess), job, "MetricProjection:Output");
			auto upload = job.m_upload.Alloc<v4>(isize(gpu_inputs));
			std::ranges::copy(gpu_inputs, upload.ptr<v4>());
			job.m_barriers.Transition(input.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyBufferRegion(input.get(), 0, upload);
			job.m_barriers.Transition(input.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			job.m_cmd_list.SetPipelineState(pso.get());
			job.m_cmd_list.SetComputeRootSignature(signature.get());
			job.m_cmd_list.AddComputeRootShaderResourceView(input);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(output);
			job.m_cmd_list.Dispatch(isize(examples), 1, 1);
			auto readback = job.m_readback.Alloc<v4>(isize(examples));
			job.m_barriers.Transition(output.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			job.m_cmd_list.CopyBufferRegion(readback, output.get(), 0);
			job.Run();
			auto actual = std::span{readback.ptr<v4>(), examples.size()};
			dump("metric-probe-actual.bin", actual);
			for (size_t row = 0; row != examples.size(); ++row)
			{
				// The CPU results above already satisfy an independent double-precision nearest-point oracle.
				auto valid = actual[row].w == 1;
				for (int axis = 0; axis != 3; ++axis)
					valid &= std::abs(static_cast<double>(actual[row][axis]) - cpu_results[row][axis]) <= 8 * std::numeric_limits<float>::epsilon() * std::abs(static_cast<double>(cpu_results[row][axis]));

				log_stream() << std::format("gpu row={} result=({},{},{},{})\n", row, actual[row].x, actual[row].y, actual[row].z, actual[row].w);
				PR_EXPECT(valid);
			}
		}
	};

	// A physical single-point case whose projected Newton endpoint is not a descent direction.
	PRUnitTestClass(ArticulatedContactDirectionGpuTests)
	{
		// Feasible ordinary sliding must not stall solely because a Newton endpoint projects to zero support.
		PRUnitTestMethod(SupportedPointNonDescent, Extended)
		{
			// This positive-mass central inertia satisfies the principal-inertia triangle inequalities.
			auto const inertia = Inertia{v4{0.208866704f, 0.209415407f, 0.131782583f, 0}, v4{-0.00803462765f, 0.0266452302f, 0.0257509273f, 0}, 4.9447999f};
			auto const lever = v4{-0.580662272f, -0.711502561f, -0.395721406f, 0};
			auto const free_velocity = v4{-2.47400636f, 3.29091459f, -0.240131693f, 0};
			auto const seed = v4{2.66737764f, 0.694440925f, 1.81622017f, 0};
			auto const material = GpuMaterial{.friction_static = 0.6f};
			auto const friction = material.friction_static / (1.000001f - material.friction_static);
			auto const witness = v4{1, -friction, 0, 0};
			auto builder = ArticulationBuilder{};
			auto const handle = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertia}, m4x4::Identity(), v8motion{v4{}, free_velocity});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto upload = PackGpuArticulations(forest);
			auto shape = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto fixed = RigidBody{&shape, m4x4::Identity(), Inertia::Infinite()};
			auto bodies = std::vector{PackDynamics(fixed, -1)};
			auto proxies = PackGpuArticulationProxies(upload, forest, std::array{-1}, 1);
			bodies.insert(bodies.end(), proxies.begin(), proxies.end());
			auto manifold = std::array{GpuResolveContact{
				.axis = v4::XAxis(), .contact_point = lever.w1(), .b2a = m4x4::Identity(),
				.body_idx_a = 0, .body_idx_b = 1, .warmstart_impulse = seed,
			}};
			auto config = EngineConfig{};
			config.solver_iterations = 10;
			auto const dt = (1.0f / 60) / 3;
			auto& log = log_stream();
			auto const directory = temp_dir() / "SupportedPointNonDescent";
			std::filesystem::create_directories(directory);
			auto dump = [&](char const* name, auto const& values)
			{
				// Keep every represented input and actual physical output before any final-law assertion.
				auto stream = std::ofstream{};
				stream.exceptions(std::ios::failbit | std::ios::badbit);
				stream.open(directory / std::format("{}.bin", name), std::ios::binary);
				stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
				log << std::format("raw name={} count={} stride={}\n", name, values.size(), sizeof(values[0]));
			};
			dump("input-articulations", upload.m_articulations);
			dump("input-links", upload.m_links);
			dump("input-dofs", upload.m_dofs);
			dump("input-positions", upload.m_positions);
			dump("input-velocities", upload.m_velocities);
			dump("input-bodies", bodies);
			dump("input-contacts", manifold);
			dump("input-materials", std::array{material});
			log << std::format("sweeps={} relaxation={} tolerance={} dt={} friction={}\n",
				config.solver_iterations, config.constraint_coupled_relaxation, config.contact_velocity_tolerance, dt, friction);

			// Measure the represented point law and its units from the floating root's actual primary state.
			auto measure = [&](v4 impulse, v4 point, m4x4 const& pose_a, char const* name)
			{
				// The static endpoint contributes no velocity or response.
				auto const& pose = tree.RootToWorld();
				auto const w2l = InvertOrthonormal(pose);
				auto const r = (w2l * point).w0();
				auto const twist = tree.RootVelocity();
				auto const relative = Transpose(pose_a.rot) * (pose.rot * (twist.lin + Cross(twist.ang, r)));
				auto response = float3x3{};
				for (int axis = 0; axis != 3; ++axis)
				{
					// Rotate each generating-basis point wrench into the physical central-inertia frame.
					auto const force = w2l.rot * v4{pose_a.rot[axis], 0};
					auto const delta = Invert(inertia) * v8force{Cross(r, force), force};
					auto const column = Transpose(pose_a.rot) * (pose.rot * (delta.lin + Cross(delta.ang, r)));
					for (int component = 0; component != 3; ++component)
						response[component][axis] = column[component];
				}
				auto law = CoulombLinearisation{};
				PR_EXPECT(LineariseCoulombLaw(response, relative.xyz, impulse.xyz, float3{1, 0, 0}, friction, law));
				auto const error = CoulombLawError(impulse.xyz, relative.xyz, friction, law.normal_weight, law.tangent_weight);
				dump(name, std::array{relative});
				auto const response_name = std::format("{}-response", name);
				dump(response_name.c_str(), std::array{response});
				log << std::format("stage={} squared={} normal_error={} full_error={} support={} slip={}\n",
					name, dot(law.residual, law.residual), std::abs(std::min(law.normal_weight * impulse.x, relative.x)), error, impulse.x, std::hypot(relative.y, relative.z));
				log.flush();
				return error;
			};

			// Apply the positive feasible witness as a point wrench, without assigning its expected resulting velocity.
			auto const witness_request = ArticulationImpulse{.m_link = handle, .m_impulse = v8force{Cross(lever, witness), witness}};
			tree.ApplyImpulses(std::span{&witness_request, 1});
			dump("feasible-primary-velocities", PackGpuArticulations(forest).m_velocities);
			dump("feasible-impulse", std::array{witness});
			PR_EXPECT(measure(witness, lever.w1(), m4x4::Identity(), "feasible-relative-velocity") <= config.contact_velocity_tolerance);

			// Reset only the CPU observer and apply the seed once; the GPU still receives the untouched free upload.
			tree.RootVelocity(v8motion{v4{}, free_velocity});
			auto const seed_request = ArticulationImpulse{.m_link = handle, .m_impulse = v8force{Cross(lever, seed), seed}};
			tree.ApplyImpulses(std::span{&seed_request, 1});
			dump("seeded-primary-velocities", PackGpuArticulations(forest).m_velocities);
			measure(seed, lever.w1(), m4x4::Identity(), "seeded-relative-velocity");

			// Exercise the actual owner for its unchanged ten-sweep budget, not an offline Newton equation.
			RunAuthoredContactOwner(upload, bodies, manifold, material, config, dt, config.solver_iterations - 1, [&](ContactOwnerState const& state)
			{
				// Primary root and six root speeds define the final represented-point observer.
				dump("final-articulations", state.m_articulations);
				dump("final-positions", state.m_positions);
				dump("final-velocities", state.m_velocities);
				dump("final-bodies", state.m_bodies);
				dump("final-contacts", state.m_contacts);
				dump("final-counters", std::span{&state.m_counters, 1});
				dump("final-state", std::span{&state.m_state, 1});
				PR_EXPECT(state.m_state.valid != 0 && state.m_state.failure_flags == 0 && state.m_state.frame_failure_flags == 0 && state.m_counters.contact_count == 1);
				tree.RootToWorld(UnpackGpuTransform(state.m_articulations[0].root_to_world));
				auto const qdot = state.m_velocities;
				tree.RootVelocity(v8motion{v4{qdot[0], qdot[1], qdot[2], 0}, v4{qdot[3], qdot[4], qdot[5], 0}});
				auto const& contact = state.m_contacts[0];
				auto const& pose_a = state.m_bodies[contact.body_idx_a].o2w;
				auto const error = measure(contact.warmstart_impulse, pose_a * contact.contact_point, pose_a, "final-relative-velocity");
				PR_EXPECT(error <= config.contact_velocity_tolerance);
			});
		}
	};

	// Coupled propagation, non-associated sliding, and singular response guards for the production contact owner.
	PRUnitTestClass(ArticulatedContactCouplingGpuTests)
	{
		// A nine-row tridiagonal normal response must propagate support at the unchanged ten-sweep budget.
		PRUnitTestMethod(TenLinkCartesianContactChain, Extended)
		{
			// Unit impulse on every row gives zero final velocity: targets are jointly feasible.
			auto speeds = std::array<v4, 9>{};
			speeds.back() = -v4::XAxis();
			auto contacts = std::array<GpuResolveContact, 9>{};
			for (int row = 0; row != 9; ++row)
				contacts[row] = GpuResolveContact{.axis = v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = row, .body_idx_b = row + 1};

			CheckCartesianContactOwner(speeds, contacts, false, temp_dir() / "TenLinkCartesianContactChain", log_stream());
		}

		// Positive support with saturated friction must not acquire the cone-energy model's normal separation.
		PRUnitTestMethod(SupportedSlidingOrdinaryLaw, Extended)
		{
			// Represented free inputs admit impulses (1,0,-mu), (-2,0,-2mu) and final speeds (0,0,1), (0,0,2).
			auto const ratio = 0.3f;
			auto const friction = ratio / (1.000001f - ratio);
			auto speeds = std::array{v4{-3, 0, 1 - friction, 0}, v4{2, 0, 2 + 2 * friction, 0}};
			auto contacts = std::array{
				GpuResolveContact{.axis = v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 0, .body_idx_b = 1},
				GpuResolveContact{.axis = -v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 1, .body_idx_b = 2},
			};
			CheckCartesianContactOwner(speeds, contacts, true, temp_dir() / "SupportedSlidingOrdinaryLaw", log_stream());
		}

		// Duplicate rows have a singular contact Jacobian but a feasible combined support impulse.
		PRUnitTestMethod(DuplicateContactNullSpace, Extended)
		{
			// Only the sum of these two impulses is determined; do not assert an artificial unique impulse split.
			auto speeds = std::array{-v4::XAxis()};
			auto contact = GpuResolveContact{.axis = v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 0, .body_idx_b = 1};
			CheckCartesianContactOwner(speeds, std::array{contact, contact}, false, temp_dir() / "DuplicateContactNullSpace", log_stream());
		}
	};

	// Exercise the exact feasibility helper shared by cache admission and Newton candidate publication.
	PRUnitTestClass(CoupledContactProjectionTests)
	{
		// Finite dyadic impulses must not skip projection at small scale or lose friction through norm overflow.
		PRUnitTestMethod(ScaleSafeFeasibility, Quick)
		{
			// Each represented scale has the same ordinary-law problem after its inverse mobility rescales the response.
			for (auto scale : std::array{std::ldexp(1.0f, -42), 1.0f, std::ldexp(1.0f, 80)})
			{
				// Preserve support while projecting the tangent onto the radius .5*scale disc.
				auto const input = scale * float3{1, 2, 0};
				auto const projected = ProjectCoulombImpulse(input, float3{1, 0, 0}, 0.5f);
				PR_EXPECT(projected.x == scale && projected.y == 0.5f * scale && projected.z == 0);
				auto const velocity = float3{-1, -2, 0} + projected / scale;
				auto const error = CoulombLawError(projected, velocity, 0.5f, 1 / scale, 1 / scale);
				PR_EXPECT(error == 0);

				// Already feasible impulses are returned intact; zero friction and negative support have zero tangent impulse.
				auto const feasible = scale * float3{1, 0.25f, 0};
				auto const retained = ProjectCoulombImpulse(feasible, float3{1, 0, 0}, 0.5f);
				auto const frictionless = ProjectCoulombImpulse(input, float3{1, 0, 0}, 0);
				auto const released = ProjectCoulombImpulse(scale * float3{-1, 2, 0}, float3{1, 0, 0}, 0.5f);
				PR_EXPECT(retained.x == feasible.x && retained.y == feasible.y && retained.z == feasible.z);
				PR_EXPECT(frictionless.x == scale && frictionless.y == 0 && frictionless.z == 0);
				PR_EXPECT(released.x == 0 && released.y == 0 && released.z == 0);
				log_stream() << std::format("scale={} projected_over_scale=({},{},{}) full_law_error={}\n", scale, projected.x / scale, projected.y / scale, projected.z / scale, error);
			}
		}
	};

	// Check the nonsymmetric ordinary-law derivative independently of the GPU Krylov implementation.
	PRUnitTestClass(CoupledCoulombJacobianTests)
	{
		// Finite dyadic scales must preserve an independently scaled normal component.
		PRUnitTestMethod(SeparatedMobilityScaling, Quick)
		{
			// The subtract-scales expression rounds 1-2^26 to -2^26 and incorrectly erases this normal direction.
			auto const axis = float3{1, 0, 0};
			auto const tangent_mobility = 1.0f / 67108864.0f;
			auto const normal = ScaleCoulombCoordinates(axis, axis, 1, 1 / tangent_mobility);
			auto const mixed = ScaleCoulombCoordinates(float3{1, 0, 1}, axis, 1, 1 / tangent_mobility);
			PR_EXPECT(normal.x == 1 && normal.y == 0 && normal.z == 0);
			PR_EXPECT(mixed.x == 1 && mixed.y == 0 && mixed.z == 67108864.0f);
		}

		// Smooth sliding derivatives must include support-radius variation and the correct transpose ordering.
		PRUnitTestMethod(OrdinaryLawDerivativeAndTranspose, Quick)
		{
			// Use the authored two-row mechanical W, but nonzero smooth residuals and directions rather than a solved state.
			auto const identity = float3x3{float3{1,0,0}, float3{0,1,0}, float3{0,0,1}};
			auto const axes = std::array{float3{1,0,0}, float3{-1,0,0}};
			auto const impulses = std::array{float3{1,0.2f,-0.35f}, float3{-2,-0.1f,-0.7f}};
			auto const velocities = std::array{float3{-0.2f,0.3f,1}, float3{0.3f,-0.4f,1.2f}};
			auto const direction = std::array{float3{0.3f,-0.5f,0.7f}, float3{-0.2f,0.9f,-0.4f}};
			auto const dual = std::array{float3{0.7f,0.1f,-0.6f}, float3{0.4f,-0.8f,0.5f}};
			auto const friction = 0.3f / (1.000001f - 0.3f);
			auto laws = std::array<CoulombLinearisation, 2>{};
			for (int row = 0; row != 2; ++row)
			{
				PR_EXPECT(LineariseCoulombLaw(static_cast<float>(row + 1) * identity, velocities[row], impulses[row], axes[row], friction, laws[row]));
			}

			// Compare the signed derivative with a central difference, and the transpose with the defining dual-product identity.
			auto forward_dot = 0.0f;
			auto transpose_dot = 0.0f;
			auto const h = 1.0f / 1024;
			for (int row = 0; row != 2; ++row)
			{
				auto const weight = static_cast<float>(row + 1);
				auto const response = weight * direction[row] - direction[1 - row];
				auto const jv = pr::hlsl::mul(laws[row].velocity_derivative, response) + pr::hlsl::mul(laws[row].impulse_derivative, direction[row]);
				auto const jtu = weight * pr::hlsl::mul(pr::hlsl::transpose(laws[row].velocity_derivative), dual[row])
					- pr::hlsl::mul(pr::hlsl::transpose(laws[1 - row].velocity_derivative), dual[1 - row])
					+ pr::hlsl::mul(pr::hlsl::transpose(laws[row].impulse_derivative), dual[row]);
				auto plus = CoulombLinearisation{};
				auto minus = CoulombLinearisation{};
				PR_EXPECT(LineariseCoulombLaw(weight * identity, velocities[row] + h * response, impulses[row] + h * direction[row], axes[row], friction, plus));
				PR_EXPECT(LineariseCoulombLaw(weight * identity, velocities[row] - h * response, impulses[row] - h * direction[row], axes[row], friction, minus));
				auto const error = Length((plus.residual - minus.residual) / (2 * h) - jv);
				PR_EXPECT(error < 16 * std::numeric_limits<float>::epsilon() / h);
				forward_dot += Dot(dual[row], jv);
				transpose_dot += Dot(jtu, direction[row]);
				auto const lambda = float3{Dot(axes[row], impulses[row]), impulses[row].y, impulses[row].z};
				auto const velocity = float3{Dot(axes[row], velocities[row]), velocities[row].y, velocities[row].z};
				auto const signed_error = std::max(Abs(Dot(axes[row], laws[row].residual)), Length(laws[row].residual - Dot(axes[row], laws[row].residual) * axes[row]));
				PR_EXPECT(Abs(signed_error - CoulombLawError(lambda, velocity, friction, weight, weight)) < 16 * std::numeric_limits<float>::epsilon());
				log_stream() << std::format("row={} derivative_error={} full_law_error={}\n", row, error, signed_error);
			}
			log_stream() << std::format("dual_forward={} dual_transpose={}\n", forward_dot, transpose_dot);
			PR_EXPECT(Abs(forward_dot - transpose_dot) < 32 * std::numeric_limits<float>::epsilon());
		}
	};

	// Analytical laws for the shared native/shader articulated contact solve, independent of the outer iteration budget.
	PRUnitTestClass(ArticulatedCoulombTests)
	{
		// Off-diagonal normal/tangent response must not enter the four-sweep zero-impulse cycle.
		PRUnitTestMethod(CoupledSlidingCycle, Quick)
		{
			auto const response = float3x3{float3{1, -1.5f, 0}, float3{-1.5f, 3.25f, 0}, float3{0, 0, 1}};
			auto const free_velocity = float3{-1, 6, 0};
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, free_velocity, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(Length(impulse - float3{8.0f / 17, -6.0f / 17, 0}) < 1.0e-6f);
			auto const velocity = free_velocity + pr::hlsl::mul(response, impulse);
			PR_EXPECT(Abs(velocity.x) < 1.0e-6f);
			PR_EXPECT(velocity.y > 4);
		}

		// A small response scale still has to recover the independently known impulse, not merely a loose residual.
		PRUnitTestMethod(ScaledSlidingImpulse, Quick)
		{
			auto const response = float3x3{float3{0.01f, -0.005f, 0.0075f}, float3{-0.005f, 0.01f, 0}, float3{0.0075f, 0, 0.01f}};
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, float3{-0.875f, 1.001f, -0.375f}, float3{1, 0, 0}, 1.5f, 1.0e-6f, impulse));
			PR_EXPECT(Abs(impulse.x - 50) < 1.0e-2f);
			PR_EXPECT(Abs(impulse.y + 75) < 1.0e-2f);
			PR_EXPECT(Abs(impulse.z) < 1.0e-2f);
		}

		// Normal anisotropy must not erase a finite friction residual through cancellation of nearly equal impulses.
		PRUnitTestMethod(NormalScaleCannotHideSlip, Quick)
		{
			auto const response = float3x3{float3{1.0e8f, 0, 0}, float3{0, 1, 0}, float3{0, 0, 1}};
			auto const b = float3{-1.0e8f, -2, 0};
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, b, float3{1, 0, 0}, 1, 1.0e-5f, impulse));
			PR_EXPECT(Abs(impulse.x - 1) < 1.0e-5f);
			PR_EXPECT(Abs(impulse.y - 1) < 1.0e-5f);
			log_stream() << std::format("impulse=({},{},{}) tangent_velocity={}\n", impulse.x, impulse.y, impulse.z, -2 + impulse.y);
		}

		// Finite rounding error still fails a caller's tighter normal-law tolerance.
		PRUnitTestMethod(FrictionlessChecksReconstructedVelocity, Quick)
		{
			auto const response = float3x3{float3{5.267640590667725f, 0, 0}, float3{0, 1, 0}, float3{0, 0, 1}};
			auto const b = float3{-826.021728515625f, 0, 0};
			auto impulse = float3{};
			PR_EXPECT(!SolveCoulombContact(response, b, float3{1, 0, 0}, 0, 1.0e-5f, impulse));
			PR_EXPECT(Abs((b + pr::hlsl::mul(response, impulse)).x) > 1.0e-5f);
			PR_EXPECT(SolveCoulombContact(response, b, float3{1, 0, 0}, 0, 1.0e-3f, impulse));
		}

		// A locked tangent direction still consumes friction allowance when it has a nonzero prescribed slip.
		PRUnitTestMethod(SingularTangentRetainsFrictionDisc, Quick)
		{
			auto const response = float3x3{float3{1, 0, 0}, float3{0, 1, 0}, float3{0, 0, 0}};
			auto const free_velocity = float3{-1, 0, 2};
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, free_velocity, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(Length(impulse - float3{1, 0, -0.75f}) < 1.0e-6f);
			PR_EXPECT(SolveCoulombContact(response, float3{-1, 0.25f, 0}, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(Length(impulse - float3{1, -0.25f, 0}) < 1.0e-6f);
		}

		// An oblique null direction and zero tangent mobility must preserve both the sticking and sliding laws.
		PRUnitTestMethod(ObliqueAndZeroTangentMobility, Quick)
		{
			auto const response = float3x3{float3{1, 0, 0}, float3{0, 0.5f, 0.5f}, float3{0, 0.5f, 0.5f}};
			auto const direction = float3{0, -0.6f, 0.8f};
			auto const expected = float3{1, -0.45f, 0.6f};
			auto const free_velocity = -2.0f * direction - pr::hlsl::mul(response, expected);
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, free_velocity, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(Length(impulse - expected) < 2.0e-6f);
			auto const immobile_tangent = float3x3{float3{1, 0, 0}, float3{0, 0, 0}, float3{0, 0, 0}};
			PR_EXPECT(SolveCoulombContact(immobile_tangent, float3{-1, 3, 4}, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(Length(impulse - float3{1, -0.45f, -0.6f}) < 2.0e-6f);
		}

		// A sticking null-space impulse can supply the required normal support even when the minimum-norm solution is outside the disc.
		PRUnitTestMethod(SingularNormalCouplingFindsStickingSupport, Quick)
		{
			auto const response = float3x3{float3{1, 0, 2}, float3{0, 1, 0}, float3{2, 0, 4}};
			auto const b = float3{-1, 2, -2};
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, b, float3{1, 0, 0}, 0.75f, 2.0e-6f, impulse));
			PR_EXPECT(Length(b + pr::hlsl::mul(response, impulse)) < 2.0e-6f);
			PR_EXPECT(impulse.x > 0);
			PR_EXPECT(std::hypot(impulse.y, impulse.z) <= 0.75f * impulse.x + 2.0e-6f);
			PR_EXPECT(impulse.z < -0.9f);
		}

		// Known sliding solutions cover anisotropic responses and rotations of the normal, independently checking the final law.
		PRUnitTestMethod(RotatedPhysicalResponseLaws, Quick)
		{
			auto rng = std::default_random_engine{481};
			auto range = std::uniform_real_distribution<float>{-1, 1};
			for (int trial = 0; trial != 256; ++trial)
			{
				auto const r = float3{range(rng), range(rng), range(rng)};
				auto const axis = Normalise(float3{range(rng), range(rng), range(rng)});
				auto const tangent = Normalise(Cross(axis, float3{range(rng), range(rng), range(rng)}));
				auto const mu = 0.1f + 1.4f * Abs(range(rng));
				auto const expected = (1.0f + Abs(range(rng))) * (axis + mu * tangent);
				auto const response = float3x3{
					float3{0.25f + r.y * r.y + r.z * r.z, -r.x * r.y, -r.x * r.z},
					float3{-r.x * r.y, 0.25f + r.x * r.x + r.z * r.z, -r.y * r.z},
					float3{-r.x * r.z, -r.y * r.z, 0.25f + r.x * r.x + r.y * r.y}};
				auto const b = -(0.1f + Abs(range(rng))) * tangent - pr::hlsl::mul(response, expected);
				auto impulse = float3{};
				PR_EXPECT(SolveCoulombContact(response, b, axis, mu, 2.0e-5f, impulse));
				auto const velocity = b + pr::hlsl::mul(response, impulse);
				auto const normal = Dot(axis, impulse);
				auto const normal_velocity = Dot(axis, velocity);
				auto const tangent_impulse = impulse - normal * axis;
				auto const slip = velocity - normal_velocity * axis;
				PR_EXPECT(normal >= -1.0e-5f && normal_velocity >= -2.1e-5f);
				PR_EXPECT(Abs(std::min(normal, normal_velocity)) < 2.1e-5f);
				PR_EXPECT(Length(tangent_impulse) <= mu * normal + 2.1e-5f);
				if (Length(slip) > 1.0e-4f && normal > 1.0e-4f)
					PR_EXPECT(Length(tangent_impulse + mu * normal * Normalise(slip)) < 5.0e-4f);
			}
		}

		// Warm state changes the residual, not the physical solution; a changed separating input withdraws all old support.
		PRUnitTestMethod(WarmEquivalenceAndSupportRelease, Quick)
		{
			auto const response = float3x3{float3{1, -1.5f, 0}, float3{-1.5f, 3.25f, 0}, float3{0, 0, 1}};
			auto const b = float3{-1, 6, 0};
			auto const warm = float3{2, 1, 0.25f};
			auto cold_result = float3{};
			auto warm_result = float3{};
			auto residual = b + pr::hlsl::mul(response, warm);
			PR_EXPECT(SolveCoulombContact(response, b, float3{1, 0, 0}, 0.75f, 1.0e-6f, cold_result));
			PR_EXPECT(SolveCoulombContact(response, residual - pr::hlsl::mul(response, warm), float3{1, 0, 0}, 0.75f, 1.0e-6f, warm_result));
			PR_EXPECT(Length(warm_result - cold_result) < 1.0e-6f);
			residual = float3{1, 6, 0} + pr::hlsl::mul(response, warm);
			PR_EXPECT(SolveCoulombContact(response, residual - pr::hlsl::mul(response, warm), float3{1, 0, 0}, 0.75f, 1.0e-6f, warm_result));
			PR_EXPECT(Length(warm_result) == 0);
			PR_EXPECT(Length(warm + (warm_result - warm)) == 0);
		}

		// Removing the entire old impulse must restore a passive free state, including separating and non-finite inputs.
		PRUnitTestMethod(FreeStateAndNonFiniteInput, Quick)
		{
			auto const response = float3x3{float3{1, -1.5f, 0}, float3{-1.5f, 3.25f, 0}, float3{0, 0, 1}};
			auto const warm = float3{1, -0.5f, 0};
			auto const residual = pr::hlsl::mul(response, warm);
			auto impulse = float3{};
			PR_EXPECT(SolveCoulombContact(response, residual - pr::hlsl::mul(response, warm), float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(Length(impulse) == 0);
			PR_EXPECT(!SolveCoulombContact(response, float3{1, std::numeric_limits<float>::infinity(), 0}, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(!SolveCoulombContact(response, float3{1, 0, std::numeric_limits<float>::quiet_NaN()}, float3{1, 0, 0}, 0.75f, 1.0e-6f, impulse));
			PR_EXPECT(!std::isfinite(CoulombLawError(float3{1.0e38f, 0, 0}, float3{0, 0, 0}, 0.75f, 10, 1)));
		}
	};

	// A failed local contact solve must reject the public step before publishing detached body or tree state.
	PRUnitTestClass(ArticulatedContactFailureTests)
	{
		// An intentionally unattainable caller tolerance exercises the existing public Engine boundary without a test-only injection hook.
		PRUnitTestMethod(LocalFailureRejectsEngineOutput, Quick)
		{
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = EngineConfig{};
			config.sleeping_enabled = false;
			config.selective_refresh_passes = 0;
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.constraint_coupled_relaxation = 1.0f;
			config.contact_velocity_tolerance = std::numeric_limits<float>::min();
			config.warm_start_scale = 0;
			engine.Config(config);
			engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0});
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = RigidBody{&sphere, m4x4::Translation(0.99f, 0, 0), Inertia::Infinite()};
			auto builder = ArticulationBuilder{};
			builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.5f, 1.0f / 5.267640590667725f), .m_shape = collision::shape_cast(&sphere)},
				m4x4::Identity(), v8motion{v4::Zero(), v4{826.021728515625f, 6.123f, 2.718f, 0}});
			auto tree = builder.Build();
			auto bodies = std::array{&ground};
			auto forest = std::array{&tree};
			auto const original_pose = tree.RootToWorld();
			auto const original_velocity = tree.RootVelocity();
			auto callback_count = 0;
			engine.Collisions += [&](Engine&, std::span<RbContact const>)
			{
				++callback_count;
			};
			auto const input = Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_elapsed_seconds = 1.0e-6f};
			PR_THROWS(engine.Step(input), std::runtime_error);
			PR_EXPECT(engine.LastFeatureStats().m_failure.m_reason == EStepFailure::ContactNumerics);
			PR_EXPECT(All(tree.RootToWorld() == original_pose));
			PR_EXPECT(tree.RootVelocity() == original_velocity);
			PR_EXPECT(callback_count == 0);

			// The same finite physical input is admissible at the caller's ordinary contact target on the next frame.
			config.contact_velocity_tolerance = 0.01f;
			engine.Config(config);
			engine.Step(input);
			PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
			PR_EXPECT(tree.RootVelocity().lin.x < original_velocity.lin.x);
		}
	};

	// Exercise the production GPU contact preparation, endpoint reduction, and two separate impulse-ABA roots.
	PRUnitTestClass(ArticulatedCoulombGpuTests)
	{
		// Physical mass-eight bodies at opposite lever arms realize the exact well-conditioned cycling response.
		PRUnitTestMethod(TwoRootCycleWarmAndRelease, Quick)
		{
			auto& gpu = SharedTestGpu();
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1.0f;
			config.contact_velocity_tolerance = 1.0e-5f;
			auto integrator = GpuIntegrator{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto proxies = GpuArticulationLinkProxies{aba, config};
			auto solver = GpuCoupledContactSolver{gpu, aba, proxies, config};
			auto const inertia = Inertia{v4{5.0f / 3, 5.0f / 3, 1.0f / 3, 0}, v4::Zero(), 8.0f};
			auto const expected = v4{8.0f / 17, -6.0f / 17, 0, 0};
			auto const warm_values = std::array{v4::Zero(), v4{1, -0.5f, 0, 0}, v4{1, -0.5f, 0, 0}, v4::Zero(), v4::Zero()};
			for (int run = 0; run != isize(warm_values); ++run)
			{
				// Run two changes the free normal velocity while retaining a nonzero previous impulse.
				auto const b = v4{run == 2 ? 1.0f : -1.0f, 6, 0, 0};
				auto builder_a = ArticulationBuilder{};
				auto builder_b = ArticulationBuilder{};
				builder_a.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertia}, m4x4::Translation(-2, -1, 0), v8motion{v4::Zero(), -0.5f * b});
				builder_b.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertia}, m4x4::Translation(2, 1, 0), v8motion{v4::Zero(), +0.5f * b});
				auto tree_a = builder_a.Build();
				auto tree_b = builder_b.Build();
				auto forest = std::array{&tree_a, &tree_b};
				auto upload = PackGpuArticulations(forest);
				auto bodies = PackGpuArticulationProxies(upload, forest, std::array{-1, -1}, 0);
				PR_EXPECT(aba.Upload(gpu.m_job, upload));
				integrator.Upload(gpu.m_job, bodies);
				proxies.Upload(gpu.m_job);
				proxies.Refresh(gpu.m_job, integrator, 0);
				PR_EXPECT(solver.Upload(gpu.m_job, upload));

				// Feed a fixed contact manifold directly; collision generation and time integration are not part of this oracle.
				auto upload_buffer = [&]<typename T>(T const& value, char const* name, int count = 1)
				{
					auto buffer = gpu.CreateResource(compute::ResDesc::Buf<T>(count, {}).usage(compute::EUsage::UnorderedAccess), gpu.m_job, name);
					auto source = gpu.m_job.m_upload.Alloc<T>(count);
					std::fill_n(source.template ptr<T>(), count, value);
					gpu.m_job.m_barriers.Transition(buffer.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
					gpu.m_job.m_cmd_list.CopyBufferRegion(buffer.get(), 0, source);
					return buffer;
				};
				auto contact = GpuResolveContact{
					.axis = v4::XAxis(),
					.contact_point = v4{2, 1, 0, 1},
					.b2a = m4x4::Translation(4, 2, 0),
					.body_idx_a = 0, .body_idx_b = 1,
					.warmstart_impulse = warm_values[run],
				};
				auto counters = upload_buffer(GpuCollisionCounters{.contact_count = 1}, "Test:CoulombCounters");
				auto const capacity = run == 1 ? 65 : 1;
				auto contacts = upload_buffer(contact, "Test:CoulombContact", capacity);
				auto materials = upload_buffer(GpuMaterial{.friction_static = 0.75f * 1.000001f / 1.75f}, "Test:CoulombMaterial");
				auto invalid_contacts = D3DPtr<ID3D12Resource>{};
				if (run == 3)
				{
					// A later successful preparation must not erase a failed earlier substep from the public frame result.
					auto invalid = contact;
					invalid.contact_point.x = std::numeric_limits<float>::quiet_NaN();
					invalid_contacts = upload_buffer(invalid, "Test:CoulombInvalidContact");
					solver.PrepareVelocity(gpu.m_job, 1.0f / 180, 2, 0, 1, counters, invalid_contacts, integrator.Bodies(), materials, 0);
					solver.SolveVelocityIteration(gpu.m_job);
				}
				solver.PrepareVelocity(gpu.m_job, 1.0f / 180, 2, 0, capacity, counters, contacts, integrator.Bodies(), materials, 0);
				solver.ApplyWarmStart(gpu.m_job);
				solver.SolveVelocityIteration(gpu.m_job);
				proxies.Refresh(gpu.m_job, integrator, 0);

				// Check final applied root momenta and the committed total impulse, not the internal multiplier equation.
				auto contact_result = gpu.m_job.m_readback.Alloc<GpuResolveContact>(1);
				auto body_result = gpu.m_job.m_readback.Alloc<GpuRigidBody>(2);
				auto state_result = gpu.m_job.m_readback.Alloc<GpuCoupledContactState>(1);
				gpu.m_job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(integrator.Bodies().get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(solver.FrameState(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(contact_result, contacts.get(), 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(body_result, integrator.Bodies().get(), 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(state_result, solver.FrameState(), 0);
				auto output = GpuFrameOutput{gpu};
				output.BeginFrame(gpu.m_job, 0, 0, 1);
				auto output_result = output.GatherAndReadback(gpu.m_job, 0, nullptr, GpuArticulationMidpointOutput{}, GpuConstraintBreakOutput{}, GpuConstraintRejectionOutput{}, 0, nullptr, solver.FrameState());
				gpu.m_job.Run();
				auto body_a = RigidBody{};
				auto body_b = RigidBody{};
				body_a.SetMassProperties(inertia);
				body_b.SetMassProperties(inertia);
				UnpackDynamics(body_result.ptr<GpuRigidBody>()[0], body_a);
				UnpackDynamics(body_result.ptr<GpuRigidBody>()[1], body_b);
				auto const velocity = body_b.VelocityWS().LinAt(v4{-2, -1, 0, 0}) - body_a.VelocityWS().LinAt(v4{2, 1, 0, 0});
				auto const impulse = contact_result.ptr<GpuResolveContact>()->warmstart_impulse;
				auto const target = run == 2 ? v4::Zero() : expected;
				PR_EXPECT(state_result.ptr<GpuCoupledContactState>()->frame_failure_flags == (run == 3 ? 2u : 0u));
				PR_EXPECT(GpuFrameOutput::Header(output_result).contact_failure_flags == (run == 3 ? 2u : 0u));
				PR_EXPECT(Length(impulse.xyz - target.xyz) < 2.0e-5f);
				PR_EXPECT(Abs(velocity.x - (run == 2 ? 1.0f : 0.0f)) < 2.0e-5f);
				PR_EXPECT(velocity.y > 4.0f);
				PR_EXPECT(body_a.KineticEnergy() + body_b.KineticEnergy() <= tree_a.KineticEnergy() + tree_b.KineticEnergy() + 1.0e-5f);
				log_stream() << std::format("run={} impulse=({},{},{}) velocity=({},{},{}) failure={}\n", run, impulse.x, impulse.y, impulse.z, velocity.x, velocity.y, velocity.z, state_result.ptr<GpuCoupledContactState>()->frame_failure_flags);
			}
		}
	};

	// Require the actual coupled result to satisfy the contact law at the ordinary workload budget.
	PRUnitTestClass(ArticulatedFinalLawGpuTests)
	{
		// Interacting same-tree contacts must satisfy complementarity and friction after all accepted responses.
		PRUnitTestMethod(SameTreeFinalLawAtOrdinaryBudget, Extended)
		{
			// Two independent Cartesian slides on one fixed root give exact self responses I and 2I, with normal cross response +1.
			auto joint = CartesianContactJoint();
			auto const link_desc = ArticulationLinkDesc{.m_inertia = Inertia::Sphere(1, 1), .m_collide_parent = true};
			auto config = EngineConfig{};
			config.solver_iterations = 10;
			auto const dt = (1.0f / 60) / 3;
			auto const material = GpuMaterial{.friction_static = 0.3f};
			auto const friction = material.friction_static / (1.000001f - material.friction_static);
			auto errors = std::array<std::array<float, 2>, 2>{};
			auto states_valid = true;
			auto lawful_separation = true;

			// Impulses (0,0,0) and (-1,0,-.25) give both unit-mass children velocity (.5,0,.75), satisfying both contact laws.
			PR_EXPECT(CoulombLawError(float3{}, float3{0.5f, 0, 0.75f}, friction, 1, 1) <= config.contact_velocity_tolerance);
			PR_EXPECT(CoulombLawError(float3{1, 0, 0.25f}, float3{}, friction, 2, 2) <= config.contact_velocity_tolerance);

			// Preserve both declared inputs before the final law assertion can stop the method; the second has lawful separating slip.
			for (int run = 0; run != 2; ++run)
			{
				// These are frozen authored point contacts, not a collision-generation or integration fixture.
				auto builder = ArticulationBuilder{};
				auto root = builder.AddFixedRoot(link_desc);
				auto first = builder.AddLink(root, joint, link_desc);
				auto second = builder.AddLink(root, joint, link_desc);
				auto tree = builder.Build();
				auto handles = std::array{root, first, second};
				tree.JointVelocity(first, std::array{run == 0 ? -0.5f : 1.0f, 0.0f, 0.5f});
				tree.JointVelocity(second, std::array{run == 0 ? 1.5f : -1.0f, 0.0f, 1.0f});
				auto forest = std::array{&tree};
				auto upload = PackGpuArticulations(forest);
				auto bodies = PackGpuArticulationProxies(upload, forest, std::array{-1, -1, -1}, 0);
				auto manifold = std::array{
					GpuResolveContact{.axis = v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 0, .body_idx_b = 1},
					GpuResolveContact{.axis = -v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 1, .body_idx_b = 2},
				};
				auto dump = [&](char const* name, auto const& values)
				{
					// Raw typed prefixes and their element sizes let a reviewer reconstruct the result without this test's observer.
					auto stream = std::ofstream{};
					stream.exceptions(std::ios::failbit | std::ios::badbit);
					stream.open(temp_dir() / std::format("{}-{}.bin", run, name), std::ios::binary);
					stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(values[0])));
					log_stream() << std::format("raw run={} name={} count={} stride={}\n", run, name, values.size(), sizeof(values[0]));
				};
				dump("input-articulations", upload.m_articulations);
				dump("input-links", upload.m_links);
				dump("input-dofs", upload.m_dofs);
				dump("input-positions", upload.m_positions);
				dump("input-velocities", upload.m_velocities);
				dump("input-bodies", bodies);
				dump("input-contacts", manifold);
				dump("input-materials", std::array{material});
				auto const starting_velocities = std::array{tree.LinkVelocity(first).lin, tree.LinkVelocity(second).lin - tree.LinkVelocity(first).lin};
				dump("starting-relative-velocities", starting_velocities);
				if (run == 0)
				{
					// This explicit admissible witness rules out incompatible targets without requiring this particular solution from the solver.
					dump("feasible-velocities", std::array{0.5f, 0.0f, 0.75f, 0.5f, 0.0f, 0.75f});
					dump("feasible-impulses", std::array{v4::Zero(), v4{-1, 0, -0.25f, 0}});
				}

				// Owners outlive any unsubmitted command prefix, including failure while recording the final readbacks.
				auto& gpu = SharedTestGpu();
				auto integrator = GpuIntegrator{gpu, config};
				auto aba = GpuArticulationForceAba{gpu};
				auto midpoint = GpuArticulationMidpoint{aba};
				auto proxies = GpuArticulationLinkProxies{aba, config};
				auto solver = GpuCoupledContactSolver{gpu, aba, proxies, config};
				auto counters = D3DPtr<ID3D12Resource>{};
				auto contacts = D3DPtr<ID3D12Resource>{};
				auto materials = D3DPtr<ID3D12Resource>{};
				auto recording = true;
				try
				{
					// Use the existing contact owner, with unchanged relaxation and the explicitly requested ten physical sweeps.
					PR_EXPECT(aba.Upload(gpu.m_job, upload));
					integrator.Upload(gpu.m_job, bodies);
					proxies.Upload(gpu.m_job);
					proxies.Refresh(gpu.m_job, integrator, 0);
					PR_EXPECT(solver.Upload(gpu.m_job, upload));
					auto upload_buffer = [&]<typename T, size_t N>(D3DPtr<ID3D12Resource>& resource, std::array<T, N> const& values, char const* name)
					{
						// Publish ownership before recording the transfer, as in the neighbouring actual-owner fixtures.
						resource = gpu.CreateResource(compute::ResDesc::Buf<T>(static_cast<int>(N), {}).usage(compute::EUsage::UnorderedAccess), gpu.m_job, name);
						auto allocation = gpu.m_job.m_upload.Alloc<T>(static_cast<int>(N));
						std::ranges::copy(values, allocation.template ptr<T>());
						gpu.m_job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
						gpu.m_job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
					};
					upload_buffer(counters, std::array{GpuCollisionCounters{.contact_count = 2}}, "Test:FinalLawCounters");
					upload_buffer(contacts, manifold, "Test:FinalLawContacts");
					upload_buffer(materials, std::array{material}, "Test:FinalLawMaterials");
					solver.PrepareVelocity(gpu.m_job, dt, 3, 0, 2, counters, contacts, integrator.Bodies(), materials, 0);
					solver.ApplyWarmStart(gpu.m_job);
					for (int iteration = 0; iteration != config.solver_iterations; ++iteration)
						solver.SolveVelocityIteration(gpu.m_job);

					// Copy authoritative configuration and speeds, not refreshed proxy velocity or an inner predicted local solution.
					auto output = midpoint.Output();
					auto final_roots = gpu.m_job.m_readback.Alloc<GpuArticulation>(1);
					auto final_positions = gpu.m_job.m_readback.Alloc<float>(isize(upload.m_positions));
					auto final_velocities = gpu.m_job.m_readback.Alloc<float>(isize(upload.m_velocities));
					auto final_contacts = gpu.m_job.m_readback.Alloc<GpuResolveContact>(2);
					auto final_counters = gpu.m_job.m_readback.Alloc<GpuCollisionCounters>(1);
					auto final_state = gpu.m_job.m_readback.Alloc<GpuCoupledContactState>(1);
					gpu.m_job.m_barriers.Transition(output.m_articulations, D3D12_RESOURCE_STATE_COPY_SOURCE);
					gpu.m_job.m_barriers.Transition(output.m_positions, D3D12_RESOURCE_STATE_COPY_SOURCE);
					gpu.m_job.m_barriers.Transition(output.m_velocities, D3D12_RESOURCE_STATE_COPY_SOURCE);
					gpu.m_job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
					gpu.m_job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
					gpu.m_job.m_barriers.Transition(solver.FrameState(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
					gpu.m_job.m_cmd_list.CopyBufferRegion(final_roots, output.m_articulations, 0);
					gpu.m_job.m_cmd_list.CopyBufferRegion(final_positions, output.m_positions, 0);
					gpu.m_job.m_cmd_list.CopyBufferRegion(final_velocities, output.m_velocities, 0);
					gpu.m_job.m_cmd_list.CopyBufferRegion(final_contacts, contacts.get(), 0);
					gpu.m_job.m_cmd_list.CopyBufferRegion(final_counters, counters.get(), 0);
					gpu.m_job.m_cmd_list.CopyBufferRegion(final_state, solver.FrameState(), 0);
					recording = false;
					gpu.m_job.Run();
					dump("final-articulations", std::span{final_roots.ptr<GpuArticulation>(), 1});
					dump("final-positions", std::span{final_positions.ptr<float>(), upload.m_positions.size()});
					dump("final-velocities", std::span{final_velocities.ptr<float>(), upload.m_velocities.size()});
					dump("final-contacts", std::span{final_contacts.ptr<GpuResolveContact>(), 2});
					dump("final-counters", std::span{final_counters.ptr<GpuCollisionCounters>(), 1});
					dump("final-state", std::span{final_state.ptr<GpuCoupledContactState>(), 1});

					// Reconstruct CPU kinematics from the actual final primary values using the existing articulation owner.
					auto const& final_frame = final_roots.ptr<GpuArticulation>()->root_to_world;
					states_valid &= All(final_frame.rotation == upload.m_articulations[0].root_to_world.rotation) && All(final_frame.position == upload.m_articulations[0].root_to_world.position);
					states_valid &= std::ranges::equal(std::span{final_positions.ptr<float>(), upload.m_positions.size()}, upload.m_positions);
					tree.RootToWorld(UnpackGpuTransform(final_roots.ptr<GpuArticulation>()->root_to_world));
					for (int link = 1; link != 3; ++link)
					{
						// Both authored joints have three scalar coordinates and three reduced speeds.
						auto const& packed = upload.m_links[link];
						tree.JointPosition(handles[link], std::span{final_positions.ptr<float>() + packed.position_offset, static_cast<size_t>(packed.dof_count)});
						tree.JointVelocity(handles[link], std::span{final_velocities.ptr<float>() + packed.velocity_offset, static_cast<size_t>(packed.dof_count)});
					}
					tree.UpdateKinematics();
					auto const& state = *final_state.ptr<GpuCoupledContactState>();
					states_valid &= state.valid != 0 && state.failure_flags == 0 && state.frame_failure_flags == 0 && final_counters.ptr<GpuCollisionCounters>()->contact_count == 2;
					log_stream() << std::format("config run={} sweeps={} relaxation={} tolerance={} dt={} friction={} flags={}/{}/{}\n",
						run, config.solver_iterations, config.constraint_coupled_relaxation, config.contact_velocity_tolerance, dt, friction, state.valid, state.failure_flags, state.frame_failure_flags);

					// Derive the actual endpoint-incidence degree from the returned contact prefix and packing map, not a private degree-buffer readback.
					auto degree = 0;
					for (auto const& contact : std::span{final_contacts.ptr<GpuResolveContact>(), 2})
					{
						// Every endpoint, including the anchored root link, belongs to this single participating tree.
						if (contact.body_idx_a < 0 || contact.body_idx_a >= 3 || contact.body_idx_b < 0 || contact.body_idx_b >= 3)
							throw std::runtime_error("Final-law fixture returned an invalid contact endpoint");

						degree += (upload.m_links[contact.body_idx_a].articulation_index == 0 ? 1 : 0) + (upload.m_links[contact.body_idx_b].articulation_index == 0 ? 1 : 0);
					}
					log_stream() << std::format("participant_degree_from_prefix={} degree_buffer_readback=false\n", degree);

					// Evaluate the full law in an orthonormal normal/Y/cross(normal,Y) basis using accepted total impulses.
					for (int row = 0; row != 2; ++row)
					{
						// The contact point and impulse are in generating body-A space; kinematics and point velocity are reconstructed in world space.
						auto const& contact = final_contacts.ptr<GpuResolveContact>()[row];
						auto const a = handles[contact.body_idx_a];
						auto const b = handles[contact.body_idx_b];
						auto const& a2w = tree.LinkToWorld(a);
						auto const& b2w = tree.LinkToWorld(b);
						auto const point_ws = a2w * contact.contact_point;
						auto const point_a = contact.contact_point - v4::Origin();
						auto const point_b = InvertOrthonormal(b2w) * point_ws - v4::Origin();
						auto const relative_ws = b2w.rot * tree.LinkVelocity(b).LinAt(point_b) - a2w.rot * tree.LinkVelocity(a).LinAt(point_a);
						auto const velocity = Transpose(a2w.rot) * relative_ws;
						auto const tangent = Cross(contact.axis, v4::YAxis());
						auto const impulse = contact.warmstart_impulse;
						auto const lambda = float3{Dot3(impulse, contact.axis), impulse.y, Dot3(impulse, tangent)};
						auto const residual = float3{Dot3(velocity, contact.axis), velocity.y, Dot3(velocity, tangent)};
						auto const response = static_cast<float>(row + 1);
						errors[run][row] = CoulombLawError(lambda, residual, friction, response, response);
						auto const normal_error = std::abs(std::min(response * lambda.x, residual.x));
						auto const slip = std::hypot(residual.y, residual.z);
						auto const separating_without_support = residual.x > 0 && Length(impulse) == 0 && slip > 0;
						if (run == 1)
							lawful_separation &= separating_without_support;

						log_stream() << std::format("start run={} row={} residual=({},{},{}) target=0\n",
							run, row, Dot3(starting_velocities[row], contact.axis), starting_velocities[row].y, Dot3(starting_velocities[row], tangent));
						log_stream() << std::format("law run={} row={} lambda=({},{},{}) residual=({},{},{}) normal_error={} full_error={} slip={} separating_support={} zero_support_separating_slip={} closing={}\n",
							run, row, lambda.x, lambda.y, lambda.z, residual.x, residual.y, residual.z, normal_error, errors[run][row], slip,
							residual.x > 0 && lambda.x > 0, separating_without_support, residual.x < 0);
					}
					log_stream().flush();
				}
				catch (...)
				{
					// Discard only an unsubmitted prefix; preserve the original failure and any already written raw result.
					auto error = std::current_exception();
					if (recording)
						gpu.m_job.RetireRecordedWork();

					std::rethrow_exception(error);
				}
			}

			// A separating sliding contact with zero support is lawful; positive support during separation is not automatically lawful.
			PR_EXPECT(states_valid);
			PR_EXPECT(lawful_separation);
			PR_EXPECT(std::isfinite(errors[1][0]) && errors[1][0] <= config.contact_velocity_tolerance);
			PR_EXPECT(std::isfinite(errors[1][1]) && errors[1][1] <= config.contact_velocity_tolerance);
			for (auto const error : errors[0])
				PR_EXPECT(std::isfinite(error) && error <= config.contact_velocity_tolerance);
		}
	};

	// Check position complementarity through real contact reduction and independent floating-root responses.
	PRUnitTestClass(ArticulatedPositionContactGpuTests)
	{
		// A later shared-body correction must release support that the first simultaneous sweep overestimated.
		PRUnitTestMethod(SharedBodySupportRelease, Extended)
		{
			// Two unit-mass roots and a fixed endpoint realize D=[[1,.5],[.5,2]] with contact directions having dot product -.5.
			auto inertia = Inertia::Sphere(1.0f, 1.0f);
			auto ground_builder = ArticulationBuilder{};
			auto a_builder = ArticulationBuilder{};
			auto b_builder = ArticulationBuilder{};
			auto initial_speeds = std::array{
				v8motion{v4{0.2f, -0.3f, 0.4f, 0}, v4{0.4f, -0.2f, 0.1f, 0}},
				v8motion{v4{-0.5f, 0.1f, 0.3f, 0}, v4{-0.3f, 0.6f, -0.4f, 0}},
			};
			ground_builder.AddFixedRoot(ArticulationLinkDesc{.m_inertia = inertia});
			a_builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertia}, m4x4::Identity(), initial_speeds[0]);
			b_builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertia}, m4x4::Identity(), initial_speeds[1]);
			auto ground = ground_builder.Build();
			auto tree_a = a_builder.Build();
			auto tree_b = b_builder.Build();
			auto forest = std::array{&ground, &tree_a, &tree_b};
			auto upload = PackGpuArticulations(forest);
			auto bodies = PackGpuArticulationProxies(upload, forest, std::array{-1, -1, -1}, 0);
			auto direction = v4{-0.5f, std::sqrt(0.75f), 0, 0};
			auto dt = 0.1f;
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1.0f;
			config.position_baumgarte = 0.2f;
			auto manifold = std::array{
				GpuResolveContact{.axis = v4::XAxis(), .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 0, .body_idx_b = 1, .depth = config.position_slop + 0.5f * dt / config.position_baumgarte},
				GpuResolveContact{.axis = direction, .contact_point = v4::Origin(), .b2a = m4x4::Identity(), .body_idx_a = 1, .body_idx_b = 2, .depth = config.position_slop + 2.0f * dt / config.position_baumgarte},
			};

			// Keep all referenced owners alive if recording is interrupted before submission.
			auto& gpu = SharedTestGpu();
			auto integrator = GpuIntegrator{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto proxies = GpuArticulationLinkProxies{aba, config};
			auto solver = GpuCoupledContactSolver{gpu, aba, proxies, config};
			auto counters = D3DPtr<ID3D12Resource>{};
			auto contacts = D3DPtr<ID3D12Resource>{};
			auto materials = D3DPtr<ID3D12Resource>{};
			auto recording = true;
			try
			{
				// This is a frozen two-contact position problem, not collision detection or a scene workload.
				PR_EXPECT(aba.Upload(gpu.m_job, upload));
				integrator.Upload(gpu.m_job, bodies);
				proxies.Upload(gpu.m_job);
				proxies.Refresh(gpu.m_job, integrator, 0);
				PR_EXPECT(solver.Upload(gpu.m_job, upload));
				auto upload_buffer = [&]<typename T, size_t N>(D3DPtr<ID3D12Resource>& resource, std::array<T, N> const& values, char const* name)
				{
					// Publish ownership before recording a transfer that references this resource.
					resource = gpu.CreateResource(compute::ResDesc::Buf<T>(static_cast<int>(N), {}).usage(compute::EUsage::UnorderedAccess), gpu.m_job, name);
					auto allocation = gpu.m_job.m_upload.Alloc<T>(static_cast<int>(N));
					std::ranges::copy(values, allocation.template ptr<T>());
					gpu.m_job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
					gpu.m_job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
				};
				upload_buffer(counters, std::array{GpuCollisionCounters{.contact_count = 2}}, "Test:PositionReleaseCounters");
				upload_buffer(contacts, manifold, "Test:PositionReleaseContacts");
				upload_buffer(materials, std::array{GpuMaterial{}}, "Test:PositionReleaseMaterials");
				solver.PrepareVelocity(gpu.m_job, dt, 3, 0, 2, counters, contacts, integrator.Bodies(), materials, 0.0f);
				PR_EXPECT(solver.PreparePosition(gpu.m_job, 32));
				auto pseudo = solver.PseudoState();
				auto count = isize(upload.m_velocities);
				auto first = gpu.m_job.m_readback.Alloc<float>(count);
				for (int iteration = 0; iteration != 32; ++iteration)
				{
					// The first accelerated simultaneous sweep has lambda=(.5,1); later damped sweeps must be able to reduce it.
					solver.SolvePositionIteration(gpu.m_job, iteration);
					if (iteration == 0)
					{
						// Preserve the actual first applied pseudo response, rather than reconstructing a hypothetical iteration.
						gpu.m_job.m_barriers.Transition(pseudo.m_generalized_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
						gpu.m_job.m_cmd_list.CopyBufferRegion(first, pseudo.m_generalized_velocities.get(), 0);
					}
				}

				// Retain the solved pseudo state before applying it through the production configuration and proxy routes.
				auto final = gpu.m_job.m_readback.Alloc<float>(count);
				gpu.m_job.m_barriers.Transition(pseudo.m_generalized_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(final, pseudo.m_generalized_velocities.get(), 0);
				solver.ApplyPosition(gpu.m_job);
				proxies.Refresh(gpu.m_job, integrator, 0);

				// Read authoritative root poses, refreshed proxies, unchanged physical speed, and the sticky failure state in one submission.
				auto physical = gpu.m_job.m_readback.Alloc<float>(count);
				auto failure = gpu.m_job.m_readback.Alloc<GpuCoupledContactState>(1);
				auto roots = gpu.m_job.m_readback.Alloc<GpuArticulation>(3);
				auto body_poses = gpu.m_job.m_readback.Alloc<GpuRigidBody>(3);
				auto output = midpoint.Output();
				gpu.m_job.m_barriers.Transition(output.m_articulations, D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(output.m_velocities, D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(integrator.Bodies().get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(solver.FrameState(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(roots, output.m_articulations, 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(body_poses, integrator.Bodies().get(), 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(physical, output.m_velocities, 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(failure, solver.FrameState(), 0);
				recording = false;
				gpu.m_job.Run();

				// Independent unit-mass momentum balance recovers both total normal impulses from the two root velocities.
				auto read_linear = [&](float const* values, int tree)
				{
					// Floating-root generalized speeds are angular XYZ followed by linear XYZ in the identity root frame.
					auto offset = upload.m_articulations[tree].velocity_offset + 3;
					return v4{values[offset], values[offset + 1], values[offset + 2], 0};
				};
				auto first_a = read_linear(first.ptr<float>(), 1);
				auto first_b = read_linear(first.ptr<float>(), 2);
				auto final_a = read_linear(final.ptr<float>(), 1);
				auto final_b = read_linear(final.ptr<float>(), 2);
				auto first_impulse = v2{first_a.x + first_b.x, Dot3(first_b, direction)};
				auto final_impulse = v2{final_a.x + final_b.x, Dot3(final_b, direction)};
				auto residual = v2{final_a.x - 0.5f, Dot3(final_b - final_a, direction) - 2.0f};
				auto law_error = std::max(std::abs(std::min(final_impulse.x, residual.x)), std::abs(std::min(2.0f * final_impulse.y, residual.y)));
				log_stream() << std::format("position release: first=({},{}) final=({},{}) residual=({},{}) law_error={}\n",
					first_impulse.x, first_impulse.y, final_impulse.x, final_impulse.y, residual.x, residual.y, law_error);
				PR_EXPECT(Length(first_impulse - v2{0.5f, 1}) < 1.0e-5f);
				PR_EXPECT(Length(final_impulse - v2{0, 1}) < 2.0e-5f);
				PR_EXPECT(law_error < 2.0e-5f);
				PR_EXPECT(std::ranges::equal(std::span{physical.ptr<float>(), upload.m_velocities.size()}, upload.m_velocities));
				PR_EXPECT(failure.ptr<GpuCoupledContactState>()->frame_failure_flags == 0u);

				// Zero contact levers produce pure translation: the configuration owner applies each accepted root pseudo speed once.
				auto expected_speeds = std::array{v4::Zero(), final_a, final_b};
				auto pose_error = 0.0f;
				for (int tree = 0; tree != 3; ++tree)
				{
					// The fixed root remains fixed, and each proxy must reflect its owner's new pose without an independent correction.
					auto const& actual = roots.ptr<GpuArticulation>()[tree].root_to_world;
					auto const& proxy = body_poses.ptr<GpuRigidBody>()[tree];
					auto expected = v4::Origin() + dt * expected_speeds[tree];
					pose_error = std::max(pose_error, Length(actual.position - expected));
					PR_EXPECT(FEqlAbsolute(actual.position, expected, 1.0e-6f));
					PR_EXPECT(FEqlAbsolute(actual.rotation, quat::Identity().xyzw, 1.0e-6f));
					PR_EXPECT(FEqlAbsolute(proxy.o2w.pos, actual.position, 1.0e-6f));
					PR_EXPECT(FEqlAbsolute(proxy.o2w.rot, m3x3::Identity(), 1.0e-6f));
					if (tree != 0)
					{
						// Nonzero angular and linear physical inputs make accidental clearing observable in the refreshed proxy too.
						PR_EXPECT(FEqlAbsolute(proxy.momentum_ang, inertia.Ic3x3() * initial_speeds[tree - 1].ang, 1.0e-6f));
						PR_EXPECT(FEqlAbsolute(proxy.momentum_lin, initial_speeds[tree - 1].lin, 1.0e-6f));
					}
				}
				auto position_a = roots.ptr<GpuArticulation>()[1].root_to_world.position;
				auto position_b = roots.ptr<GpuArticulation>()[2].root_to_world.position;
				auto separation = v2{position_a.x, Dot3(position_b - position_a, direction)};
				PR_EXPECT(Length(separation - dt * v2{0.5f, 2.0f}) < 2.0e-6f);
				log_stream() << std::format("contact pose application: error={} separation=({},{}) all 12 nonzero physical speed components unchanged; proxy angular/linear momenta preserved\n", pose_error, separation.x, separation.y);
			}
			catch (...)
			{
				// Retire only an unsubmitted prefix; a submitted job retains its own completion/failure ownership.
				auto error = std::current_exception();
				if (recording)
					gpu.m_job.RetireRecordedWork();

				std::rethrow_exception(error);
			}
		}
	};

	// Exercise physical friction budgets independently of penetration bias and fresh normal closing.
	PRUnitTestClass(FrictionSolverTests)
	{
		// A warm-started support impulse must resist slip even when the normal constraint is already separating.
		PRUnitTestMethod(WarmNormalSupportCancelsSlip, Quick)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.velocity_baumgarte = 0;
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
			auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.5f), Inertia::Sphere(0.5f, 1)};
			auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body.VelocityWS(v4{0, 0.1f, 0, 0}, v4{-0.01f, 0, 0.001f, 0});
			auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
			auto contacts = std::array{GpuResolveContact{
				.axis = -v4::ZAxis(),
				.contact_point = v4{0, 0, -0.5f, 1},
				.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
				.body_idx_a = 0, .body_idx_b = 1,
				.warmstart_impulse = v4{0, 0, -0.16f, 0},
			}};
			auto materials = std::array{GpuMaterial{.friction_static = 0.3f}};
			auto const energy_before = body.KineticEnergy();
			auto runner = ResolveInteropRunner{config};
			runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
			UnpackDynamics(bodies[0], body);

			// Neither fresh closing nor bias is available: only the already-applied normal support can fund this tangent impulse.
			// The solver withdraws the part of that support that leaves the bodies separating, so the normal speed settles at zero.
			auto const velocity = body.VelocityWS();
			PR_EXPECT(Abs(velocity.LinAt(v4{0, 0, -0.5f, 0}).x) < 1.0e-5f);
			PR_EXPECT(FEqlAbsolute(velocity.lin.z, 0.0f, 1.0e-6f));
			PR_EXPECT(body.KineticEnergy() <= energy_before + 1.0e-7f);
		}

		// Extra sweeps must not multiply the Coulomb allowance of a saturated sliding contact.
		PRUnitTestMethod(SlidingImpulseBudgetDoesNotGrowWithIterations, Quick)
		{
			auto momentum_after = std::array<float, 2>{};
			for (int run = 0; run != 2; ++run)
			{
				auto config = EngineConfig{};
				config.push_out_iterations = 0;
				config.velocity_baumgarte = 0;
				config.solver_iterations = run == 0 ? 1 : 12;
				auto sphere = collision::ShapeSphere{0.5f};
				auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.5f), Inertia::Sphere(0.5f, 1)};
				auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4::Zero(), v4{10, 0, 0, 0});
				auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
				auto contacts = std::array{GpuResolveContact{
					.axis = -v4::ZAxis(),
					.contact_point = v4{0, 0, -0.5f, 1},
					.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
					.body_idx_a = 0, .body_idx_b = 1,
					.warmstart_impulse = v4{0, 0, -0.1f, 0},
				}};
				auto materials = std::array{GpuMaterial{.friction_static = 0.3f}};
				auto runner = ResolveInteropRunner{config};
				runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
				momentum_after[run] = bodies[0].momentum_lin.x;

				// The accumulated delta reaches, but cannot exceed, the one physical support budget.
				auto const limit = 0.1f * 0.3f / (1.000001f - 0.3f);
				PR_EXPECT(FEqlAbsolute(10 - momentum_after[run], limit, 2.0e-6f));
				PR_EXPECT(Length(contacts[0].friction_impulses[0]) <= limit + 1.0e-6f);
			}
			PR_EXPECT(FEqlAbsolute(momentum_after[0], momentum_after[1], 1.0e-6f));
		}

		// A broad face resists torsion using a shared physical budget, but penetration bias alone cannot create that budget.
		PRUnitTestMethod(ManifoldFrictionUsesPhysicalSupportNotBias, Quick)
		{
			for (auto const supported : {false, true})
			{
				auto config = EngineConfig{};
				config.push_out_iterations = 0;
				auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
				auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&box, m4x4::Translation(0, 0, 0.5f), Inertia::Box(box.m_radius, 1)};
				auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4::ZAxis(), v4::Zero());
				auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
				auto contacts = std::array{GpuResolveContact{
					.axis = -v4::ZAxis(),
					.contact_point = v4{0, 0, -0.5f, 1},
					.manifold = {v4{-0.5f, -0.5f, -0.5f, 1}, v4{+0.5f, -0.5f, -0.5f, 1}, v4{+0.5f, +0.5f, -0.5f, 1}, v4{-0.5f, +0.5f, -0.5f, 1}},
					.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
					.body_idx_a = 0, .body_idx_b = 1,
					.depth = supported ? 0.0f : 0.1f,
					.feature = 4,
					.warmstart_impulse = supported ? v4{0, 0, -0.16f, 0} : v4::Zero(),
				}};
				auto materials = std::array{GpuMaterial{.friction_static = 0.3f}};
				auto runner = ResolveInteropRunner{config};
				runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
				UnpackDynamics(bodies[0], body);

				// All four point accumulators share the contact's normal allowance rather than each receiving a full friction cone.
				auto total_tangent = 0.0f;
				for (auto const& impulse : contacts[0].friction_impulses)
					total_tangent += Length(impulse);

				if (supported)
				{
					PR_EXPECT(body.VelocityWS().ang.z < 0.99f);
					PR_EXPECT(total_tangent <= 0.16f * 0.3f / (1.000001f - 0.3f) + 1.0e-5f);
				}
				else
				{
					PR_EXPECT(FEqlAbsolute(body.VelocityWS().ang.z, 1.0f, 1.0e-6f));
					PR_EXPECT(total_tangent == 0);
					PR_EXPECT(body.VelocityWS().lin.z > 0);
				}
			}
		}

		// Resolving an impact must retain the transform that generated the stored contact geometry.
		PRUnitTestMethod(CollisionTimeRewindPreservesGeneratingTransform, Quick)
		{
			// Shallow overlap and a closing velocity force a nonzero collision-time rewind without position correction or friction.
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.velocity_baumgarte = 0;
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
			auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.495f), Inertia::Sphere(0.5f, 1)};
			auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body.VelocityWS(v4::Zero(), -v4::ZAxis());
			auto const generating_b2a = InvertOrthonormal(body.O2W()) * fixed.O2W();
			auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
			auto contacts = std::array{GpuResolveContact{
				.axis = -v4::ZAxis(),
				.contact_point = v4{0, 0, -0.4975f, 1},
				.b2a = generating_b2a,
				.body_idx_a = 0, .body_idx_b = 1,
				.depth = 0.005f,
			}};
			auto materials = std::array{GpuMaterial{}};
			auto runner = ResolveInteropRunner{config};
			runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});

			// Require a real rewind and impulse writeback, then check that all four generating transform columns survive unchanged.
			PR_EXPECT(contacts[0].collision_time < -1.0e-6f);
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin.z, 0.0f, 1.0e-5f));
			for (int column = 0; column != 4; ++column)
				PR_EXPECT(FEqlAbsolute(contacts[0].b2a[column], generating_b2a[column], 1.0e-6f));
		}

		// Mixed rigid/articulation scheduling must spend only the accepted part of an ordinary contact's preloaded normal impulse.
		PRUnitTestMethod(MixedPreloadedWarmStartUsesAcceptedSupport, Extended)
		{
			// Seed the same 0.1 N s cache entry, then distinguish rejected, partially accepted, and fully accepted normal support.
			auto const normal_velocities = std::array{1.0f, -0.02f, -0.1f};
			auto const accepted_normal_impulses = std::array{0.0f, 0.04f, 0.1f};
			for (int run = 0; run != isize(normal_velocities); ++run)
			{
				auto& engine = SharedEngine();
				ResetEngineForNextTest(engine);
				engine.Terrain(std::nullopt);
				engine.CylindricalBoundary(std::nullopt);
				auto config = EngineConfig{};
				config.sleeping_enabled = false;
				config.selective_refresh_passes = 0;
				config.push_out_iterations = 0;
				config.solver_iterations = 1;
				config.velocity_baumgarte = 0;
				config.warm_start_scale = 1;
				engine.Config(config);
				engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0});
				auto sphere = collision::ShapeSphere{0.5f};
				auto ground_shape = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.45f), Inertia::Sphere(0.5f, 1)};
				auto ground = RigidBody{&ground_shape, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4::Zero(), v4{0, 0, -0.1f, 0});

				// A shaped floating root touching the same immovable floor activates the coupled lane and its all-contact preload dispatch.
				auto builder = ArticulationBuilder{};
				builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.5f, 1), .m_shape = collision::shape_cast(&sphere)},
					m4x4::Translation(-3, 0, 0.45f), v8motion{v4::Zero(), v4{0, 0, -0.1f, 0}});
				auto articulation = builder.Build();
				auto bodies = std::array<RigidBody*, 2>{&body, &ground};
				auto articulations = std::array<Articulation*, 1>{&articulation};
				auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = articulations, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 1};

				// Retain the actual ordinary contact so a cache miss caused by changed local geometry cannot masquerade as a rejected preload.
				auto rigid_contact = std::optional<RbContact>{};
				engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
				{
					for (auto const& contact : contacts)
					{
						if ((contact.m_objA == &body && contact.m_objB == &ground) || (contact.m_objA == &ground && contact.m_objB == &body))
							rigid_contact = contact;
					}
				};
				engine.Step(input);
				PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 2);
				PR_EXPECT(engine.LastFeatureStats().m_articulations.m_articulation_count == 1);
				PR_EXPECT(engine.LastFeatureStats().m_coupled.m_resources.m_dispatch_count > 0);
				PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, 0.0f, 1.0e-5f));
				auto const seed_contact = rigid_contact.value();
				auto const contact_o2w = body.O2W();

				// Offset only the fixture's initial pose to cancel integration drift; WarmStartKey hashes the quantised local contact point.
				auto const normal_velocity = normal_velocities[run];
				auto const accepted_normal = accepted_normal_impulses[run];
				auto const initial_velocity = v4{1, 0, normal_velocity, 0};
				auto initial_o2w = contact_o2w;
				initial_o2w.pos -= input.m_elapsed_seconds * initial_velocity;
				body.O2W(initial_o2w);
				body.VelocityWS(v4::Zero(), initial_velocity);
				rigid_contact.reset();
				engine.Step(input);
				PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 2);
				PR_EXPECT(engine.LastFeatureStats().m_coupled.m_resources.m_dispatch_count > 0);
				auto const replay_contact = rigid_contact.value();
				PR_EXPECT(replay_contact.m_objA == seed_contact.m_objA && replay_contact.m_objB == seed_contact.m_objB);
				PR_EXPECT(replay_contact.m_child_idA == seed_contact.m_child_idA && replay_contact.m_child_idB == seed_contact.m_child_idB);
				PR_EXPECT(FEqlAbsolute(replay_contact.Point(), seed_contact.Point(), 1.0e-6f));
				PR_EXPECT(FEqlAbsolute(body.O2W().pos, contact_o2w.pos, 1.0e-6f));
				auto const velocity = body.VelocityWS();

				// The solver iteration withdraws any accepted support that leaves the bodies separating, but never more than the accepted preload.
				auto const retained_normal = accepted_normal - Min(Max(normal_velocity + accepted_normal, 0.0f), accepted_normal);
				auto const friction_limit = retained_normal * 0.3f / (1.000001f - 0.3f);
				log_stream() << std::format("Mixed preload: initial vn={:g}, expected accepted Jn={:g}, retained Jn={:g}, final vx={:g}, final vz={:g}, angular speed={:g}, local point delta={:g}\n",
					normal_velocity, accepted_normal, retained_normal, velocity.lin.x, velocity.lin.z, Length(velocity.ang), Length(replay_contact.Point() - seed_contact.Point()));
				PR_EXPECT(FEqlAbsolute(velocity.lin.z, normal_velocity + retained_normal, 2.0e-5f));
				PR_EXPECT(FEqlAbsolute(velocity.lin.x, 1 - friction_limit, 2.0e-5f));
				if (accepted_normal == 0)
				{
					PR_EXPECT(FEqlAbsolute(velocity.ang, v4::Zero(), 1.0e-5f));

					// A rejected preload must not poison the following cache; with ordinary iterations disabled, no fresh normal response is available.
					auto const cache_probe_velocity = v4{0, 0, -0.02f, 0};
					initial_o2w = contact_o2w;
					initial_o2w.pos -= input.m_elapsed_seconds * cache_probe_velocity;
					body.O2W(initial_o2w);
					body.VelocityWS(v4::Zero(), cache_probe_velocity);
					config.solver_iterations = 0;
					engine.Config(config);
					rigid_contact.reset();
					engine.Step(input);
					PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 2);
					PR_EXPECT(FEqlAbsolute(rigid_contact.value().Point(), seed_contact.Point(), 1.0e-6f));
					PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, -0.02f, 2.0e-5f));
				}
				engine.Collisions.reset();
			}
		}

		// Exact torque must hold a sphere on a feasible incline with the default warm-start setting and production material.
		PRUnitTestMethod(DefaultWarmStartSphereHoldOnIncline, Extended)
		{
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Terrain(std::nullopt);
			engine.CylindricalBoundary(std::nullopt);
			engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0.05f, .m_elasticity_tang = 0, .m_elasticity_tors = 0, .m_density = 1});
			auto const angle = DegreesToRadians(10.0f);
			auto const normal = v4{-Sin(angle), 0, Cos(angle), 0};
			auto const uphill = v4{Cos(angle), 0, Sin(angle), 0};
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = collision::ShapeBox{v4{100, 100, 2, 0}};
			auto body = RigidBody{&sphere, m4x4::Translation(v4::Origin() + 0.495f * normal), Inertia::Sphere(0.5f, 1)};
			auto fixed = RigidBody{&ground, m4x4{uphill, v4::YAxis(), normal, v4::Origin() - normal}, Inertia::Infinite()};
			auto bodies = std::array<RigidBody*, 2>{&body, &fixed};
			auto const start = body.O2W().pos;
			auto const gravity = v4{0, 0, -9.81f, 0};
			auto const torque = v4{0, 0.5f * 9.81f * Sin(angle), 0, 0};
			auto max_late_speed = 0.0f;
			auto max_late_slip = 0.0f;
			for (int tick = 0; tick != 120; ++tick)
			{
				body.ApplyForceWS(v8force{torque, gravity});
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
				auto const velocity = body.VelocityWS();
				if (tick >= 60)
				{
					max_late_speed = Max(max_late_speed, Length(velocity.lin));
					max_late_slip = Max(max_late_slip, Abs(Dot3(0.5f * Cross(velocity.ang, normal) - velocity.lin, uphill)));
				}
			}

			// Zero-feedback hold isolates the solver: the old normal-only warm-start path slides downhill and spins despite sufficient traction.
			log_stream() << std::format("Exact hold: late speed={:g}, late slip={:g}, uphill displacement={:g}\n", max_late_speed, max_late_slip, Dot3(body.O2W().pos - start, uphill));
			PR_EXPECT(max_late_speed < 0.05f);
			PR_EXPECT(max_late_slip < 0.05f);
			PR_EXPECT(Abs(Dot3(body.O2W().pos - start, uphill)) < 0.05f);
		}
	};

	// A bounded horizontal velocity motor must carry a sliding capsule across a slope in every direction.
	PRUnitTestClass(SlopeVelocityMotorTests)
	{
		// The motor force far exceeds friction plus the downhill weight component, so each drive direction reaches its target on shallow and steep slopes.
		// The result must not depend on the solver iteration count, because extra iterations must not build up extra contact impulse.
		PRUnitTestMethod(UprightCapsuleReachesDriveVelocity, Extended)
		{
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Terrain(std::nullopt);
			engine.CylindricalBoundary(std::nullopt);

			// An upright capsule with its lower cap at the body origin, matching a walking character.
			auto const radius = 0.25f;
			auto const height = 0.8f;
			auto const mass = 90.0f;
			auto const com_height = 0.5f * height;
			auto capsule = collision::ShapeLine{height - 2 * radius, radius, m4x4::Translation(0, 0, com_height)};
			auto const directions = std::array{v4{+1, 0, 0, 0}, v4{0, +1, 0, 0}, v4{-1, 0, 0, 0}, Normalise(v4{1, 1, 0, 0})};
			auto const cases = std::array{std::pair{13.0f, 8}, std::pair{13.0f, 32}, std::pair{35.0f, 8}};
			for (auto const& [slope_degrees, solver_iterations] : cases)
			{
				// A static box tilted about the world Y axis forms the slope, rising towards +X.
				auto const angle = DegreesToRadians(slope_degrees);
				auto const normal = v4{-Sin(angle), 0, Cos(angle), 0};
				auto const uphill = v4{Cos(angle), 0, Sin(angle), 0};
				auto ground_shape = collision::ShapeBox{v4{200, 200, 2, 0}};
				auto ground = RigidBody{&ground_shape, m4x4{uphill, v4::YAxis(), normal, v4::Origin() - normal}, Inertia::Infinite()};

				for (auto const speed : {1.5f, 2.5f})
				{
					for (auto const& direction : directions)
					{
						// Reset the engine with the iteration count under test.
						ResetEngineForNextTest(engine);
						engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0.05f, .m_elasticity_tang = 0, .m_elasticity_tors = 0, .m_density = 1});
						auto config = EngineConfig{};
						config.solver_iterations = solver_iterations;
						engine.Config(config);

						// Rest the lower cap on the slope at the origin, upright, with no initial motion.
						auto body = RigidBody{};
						body.Shape(collision::shape_cast(&capsule), mass, false);
						body.O2W(m4x4::Translation(v4::Origin() + radius * normal - v4{0, 0, radius, 0}));
						auto bodies = std::array<RigidBody*, 2>{&body, &ground};

						// Posture constraint: world X/Y velocity motors at the centre of mass, free vertical motion, locked tilt and driven yaw.
						auto desc = ConstraintDesc{};
						desc.m_frame_b.m_body = BodyRef::Rigid(body);
						desc.m_frame_b.m_constraint_to_body = m4x4::Translation(0, 0, com_height);
						for (int axis = 0; axis != 2; ++axis)
						{
							desc.D6().m_linear[axis].m_mode = EConstraintAxisMode::Driven;
							desc.D6().m_linear[axis].m_target_velocity = speed * direction[axis];
							desc.D6().m_linear[axis].m_max_force = 3000.0f;
						}
						desc.D6().m_linear[2].m_mode = EConstraintAxisMode::Free;
						desc.D6().m_angular[0].m_mode = EConstraintAxisMode::Locked;
						desc.D6().m_angular[1].m_mode = EConstraintAxisMode::Locked;
						desc.D6().m_angular[2].m_mode = EConstraintAxisMode::Driven;
						desc.D6().m_angular[2].m_max_force = 400.0f;
						auto constraints = ConstraintSet{};
						constraints.Add(desc);

						// Measure the steady horizontal velocity error after the motor has had time to accelerate the body.
						auto max_error = 0.0f;
						auto min_speed = std::numeric_limits<float>::infinity();
						for (int tick = 0; tick != 90; ++tick)
						{
							body.GravityWS(v4{0, 0, -9.81f, 0});
							engine.Step(Engine::StepInput{.m_bodies = bodies, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
							auto const velocity = body.VelocityWS().lin;
							auto const horizontal = v4{velocity.x, velocity.y, 0, 0};
							if (tick >= 30)
							{
								max_error = Max(max_error, Length(horizontal - speed * direction));
								min_speed = Min(min_speed, Length(horizontal));
							}
						}
						log_stream() << std::format("slope motor slope={:g} iterations={} dir=({:g},{:g}) speed={:g}: max error={:.4f} min speed={:.4f}\n", slope_degrees, solver_iterations, direction.x, direction.y, speed, max_error, min_speed);
						PR_EXPECT(max_error < 0.05f);
					}
				}
			}
		}
	};
}
#endif
