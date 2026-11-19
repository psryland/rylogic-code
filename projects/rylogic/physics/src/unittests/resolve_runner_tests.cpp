//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/interop/resolve_runner.h"
#include "src/unittests/shared_gpu.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/shader_code.h"
#include "src/integrator/engine_buffer_cache.h"

namespace pr::physics::tests
{
	// Prove that only empty colours lose work, while every occupied colour retains the full live-contact grid.
	static void CheckColourDispatch(GpuResolver& resolver, ResolveInteropRunner const& oracle)
	{
		auto& job = resolver.m_gpu.m_job;
		auto dispatch = job.m_readback.Alloc<DispatchArguments>(MaxColours);
		job.m_barriers.Transition(resolver.m_r_colour_dispatch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		job.m_cmd_list.CopyBufferRegion(dispatch, resolver.m_r_colour_dispatch.get());
		job.Run();

		// Derive occupancy independently from assignments, rather than trusting the dispatch-generating shader.
		for (auto colour = 0; colour != MaxColours; ++colour)
		{
			auto const occupied = std::ranges::find(oracle.Colours(), static_cast<uint32_t>(colour)) != oracle.Colours().end();
			auto const groups = occupied ? (oracle.ContactOrder().size() + ResolveThreadCount - 1) / ResolveThreadCount : 0U;
			auto const& args = dispatch.ptr<DispatchArguments>()[colour];
			PR_EXPECT(args.ThreadGroupCountX == groups);
			PR_EXPECT(args.ThreadGroupCountY == 1 && args.ThreadGroupCountZ == 1);
			PR_EXPECT(args.ThreadGroupCountX == oracle.ColourDispatch()[colour].ThreadGroupCountX);
		}
	}

	// Hardware regressions for colour-only root updates across successive phases and retained warm-start frames.
	PRUnitTestClass(ResolveRootStateTests)
	{
		// Compare independent axis contacts with the shader interop oracle, including nonzero cached support on the second frame.
		static void CheckColours(int contact_count, int island_count = 1, EColourSweep sweep = EColourSweep::PerColour)
		{
			// Independent hubs retain the same three-axis oracle while covering more than one indirect thread group.
			assert(contact_count >= 1 && contact_count <= 3);
			auto config = EngineConfig{};
			config.push_out_iterations = 3;
			config.solver_iterations = 4;
			config.contact_sort_propagation_scale = 0.0f;
			config.velocity_baumgarte = 0.0f;
			config.deep_penetration_baumgarte_min = 0.0f;
			config.deep_penetration_baumgarte_max = 0.0f;
			config.warm_start_scale = 1.0f;
			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto hub = RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			auto wall = RigidBody{&box, m4x4::Identity(), Inertia::Infinite()};
			hub.VelocityWS(v4::Zero(), v4{1, 2, 3, 0});
			auto initial = std::vector<GpuRigidBody>{};
			auto contacts = std::vector<GpuResolveContact>{};
			auto axes = std::array{v4::XAxis(), v4::YAxis(), v4::ZAxis()};
			for (int island = 0; island != island_count; ++island)
			{
				// Give every island distinct body identities, including static endpoints used by the warm-start cache.
				auto const hub_index = isize(initial);
				initial.push_back(PackDynamics(hub, hub_index));
				for (int index = 0; index != contact_count; ++index)
				{
					// Each hub needs separate nonzero colour root updates in all solver phases.
					auto const wall_index = isize(initial);
					initial.push_back(PackDynamics(wall, wall_index));
					contacts.push_back(GpuResolveContact{
						.axis = axes[index],
						.contact_point = v4::Origin(),
						.manifold = {v4::Origin()},
						.b2a = m4x4::Identity(),
						.body_idx_a = hub_index,
						.body_idx_b = wall_index,
						.mat_id_a = 0,
						.mat_id_b = 0,
						.depth = 0.1f,
						.feature = 1,
					});
				}
			}
			auto materials = std::array{GpuMaterial{.friction_static = 0.0f, .elasticity_norm = 0.0f}};
			auto expected = initial;
			auto expected_contacts = contacts;
			auto oracle = ResolveInteropRunner{config};
			oracle.Run(ResolveRunnerBuffers{1.0f / 60.0f, expected, expected_contacts, materials});
			PR_EXPECT(!oracle.ColourOverflow());
			for (int index = 0; index != isize(contacts); ++index)
			{
				// Validate every island's active colours and require real physical support, not merely matching no-op output.
				PR_EXPECT(Dot3(expected_contacts[index].warmstart_impulse, axes[index % contact_count]) > 0.0f);
				PR_EXPECT(oracle.Colours()[index] < static_cast<uint32_t>(contact_count));
				for (int other = index - index % contact_count; other != index; ++other)
					PR_EXPECT(oracle.Colours()[index] != oracle.Colours()[other]);
			}

			// Keep the resolver cache but restore identical body/contact input so each colour must apply only its own support.
			auto& gpu = SharedTestGpu();
			auto resolver = GpuResolver{gpu, config, &SharedTestShaderCache()};
			resolver.m_colour_sweep = sweep;
			for (int frame = 0; frame != 2; ++frame)
			{
				// The second frame exercises retained warm-start caches with exactly the same input bodies.
				auto actual = initial;
				resolver.Resolve(gpu.m_job, 1.0f / 60.0f, contacts, actual, materials);
				CheckColourDispatch(resolver, oracle);
				for (int index = 0; index != isize(actual); ++index)
				{
					// Compare both transforms and physical momentum after every colour sweep.
					PR_EXPECT(FEqlAbsolute(actual[index].o2w.pos, expected[index].o2w.pos, 1.0e-5f));
					PR_EXPECT(FEqlAbsolute(actual[index].momentum_lin, expected[index].momentum_lin, 1.0e-5f));
					PR_EXPECT(FEqlAbsolute(actual[index].momentum_ang, expected[index].momentum_ang, 1.0e-5f));
				}
			}
		}

		// Empty colour batches must retain the full constants initialized for a single active contact.
		PRUnitTestMethod(SingleColourMatchesInterop, Extended)
		{
			CheckColours(1);
		}

		// Shared-body contacts require distinct colours and exercise nonzero colour offsets in every sweep.
		PRUnitTestMethod(MultipleColoursMatchInterop, Extended)
		{
			CheckColours(3);
		}
	};

	// Hardware coverage for empty grids after selective filtering and rigid/proxy partitioning.
	PRUnitTestClass(ResolveColourFilterTests)
	{
		// Support-only and proxy-only contact sets must leave no occupied colour or stale indirect work.
		PRUnitTestMethod(ExcludedContactsClearRetainedDispatch, Extended)
		{
			using namespace pr::compute;
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto config = EngineConfig{};
			config.contact_sort_propagation_scale = 0.0f;
			auto resolver = GpuResolver{gpu, config, &SharedTestShaderCache()};
			auto shape = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Box(shape.m_radius, 1.0f)};
			auto bodies = std::array{PackDynamics(body, 0), PackDynamics(body, 1)};
			auto contacts = std::array{GpuResolveContact{
				.axis = v4::XAxis(),
				.contact_point = v4::Origin(),
				.manifold = {v4::Origin()},
				.b2a = m4x4::Identity(),
				.body_idx_a = 0,
				.body_idx_b = 1,
				.depth = 0.1f,
				.feature = 1,
			}};
			auto materials = std::array{GpuMaterial{}};
			auto counters = gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job, "Test:ColourCounters");
			auto contact_buffer = gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(1, {}).usage(EUsage::UnorderedAccess), job, "Test:ColourContacts");
			auto body_buffer = gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(2, {}).usage(EUsage::UnorderedAccess), job, "Test:ColourBodies");
			auto dispatch = gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}), job, "Test:ColourDispatch");
			auto upload = [&]<typename Container>(ID3D12Resource* resource, Container const& values)
			{
				using Value = typename Container::value_type;
				auto allocation = job.m_upload.Alloc<Value>(isize(values));
				memcpy(allocation.template ptr<Value>(), values.data(), values.size() * sizeof(Value));
				job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
			};

			// Reuse one resolver across active, proxy-only, active, support-filtered and empty passes.
			for (auto pass = 0; pass != 5; ++pass)
			{
				upload(body_buffer.get(), bodies);
				upload(contact_buffer.get(), contacts);
				upload(counters.get(), std::array{GpuCollisionCounters{.contact_count = pass == 4 ? 0 : 1}});
				upload(dispatch.get(), std::array{D3D12_DISPATCH_ARGUMENTS{1, 1, 1}});
				resolver.Resolve(job, 1.0f / 60.0f, 2, pass == 1 ? 1 : 2, 1, dispatch, counters, contact_buffer, body_buffer, materials, 1.0f, -1, -1, 1.0f, pass == 3);
				auto batches = job.m_readback.Alloc<DispatchArguments>(MaxColours);
				job.m_barriers.Transition(resolver.m_r_colour_dispatch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				job.m_cmd_list.CopyBufferRegion(batches, resolver.m_r_colour_dispatch.get());
				job.Run();
				for (auto colour = 0; colour != MaxColours; ++colour)
				{
					auto const active = (pass == 0 || pass == 2) && colour == 0;
					auto const& args = batches.ptr<DispatchArguments>()[colour];
					PR_EXPECT(args.ThreadGroupCountX == (active ? 1U : 0U));
				}
			}
		}
	};

	// Hardware coverage for lossless CPU recording across small and large parallel contact grids.
	PRUnitTestClass(ResolveBatchRecordingTests)
	{
		// One and several occupied colours retain the established independent shader oracle.
		PRUnitTestMethod(SmallColourSweepsMatchInterop, Extended)
		{
			// Exercise empty colour batches as well as updates to nonzero colour constants.
			TestClass_ResolveRootStateTests::CheckColours(1);
			TestClass_ResolveRootStateTests::CheckColours(3);
		}

		// Thousands of bodies still use a complete multi-group dispatch without small-world assumptions.
		PRUnitTestMethod(ThousandsOfBodiesMatchInterop, Extended)
		{
			// Cover 4,096 bodies and 3,072 contacts through position, velocity, and retained warm-start phases.
			TestClass_ResolveRootStateTests::CheckColours(3, 1024);
		}

		// The single-group fused sweep must reproduce the per-colour oracle, including contacts strided over several passes of the group.
		PRUnitTestMethod(FusedColourSweepsMatchInterop, Extended)
		{
			// Cover one colour, several colours, and more contacts than the group has threads.
			TestClass_ResolveRootStateTests::CheckColours(1, 1, EColourSweep::Fused);
			TestClass_ResolveRootStateTests::CheckColours(3, 1, EColourSweep::Fused);
			TestClass_ResolveRootStateTests::CheckColours(3, 100, EColourSweep::Fused);
		}
	};

	// Hardware coverage for GPU-counted contact sorting with retained allocation and changing current-pass limits.
	PRUnitTestClass(ResolveLiveContactSortTests)
	{
		// Empty, sparse, partial-partition and dense inputs must sort only live keys and preserve the inactive suffix.
		PRUnitTestMethod(RetainedCapacitySortsOnlyLiveContacts, Extended)
		{
			// Isolate key generation and sorting from impulse solving, while using the production root bindings and indirect dispatch.
			using namespace pr::compute;
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto config = EngineConfig{};
			config.solver_iterations = 0;
			config.push_out_iterations = 0;
			config.warm_start_scale = 0;
			config.contact_sort_propagation_scale = 0;
			auto resolver = GpuResolver{gpu, config, &SharedTestShaderCache()};
			auto const capacity = 65536;
			auto shape = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Box(shape.m_radius, 1.0f)};
			auto stationary = PackDynamics(body, 1);
			body.VelocityWS(v4::Zero(), v4{0, 0, 1, 0});
			auto bodies = std::array{PackDynamics(body, 0), stationary};
			auto contacts = std::vector<GpuResolveContact>(capacity, GpuResolveContact{
				.axis = v4::ZAxis(),
				.contact_point = v4::Origin(),
				.manifold = {v4::Origin()},
				.b2a = m4x4::Identity(),
				.body_idx_a = 0,
				.body_idx_b = 1,
				.feature = 1,
			});
			auto materials = std::array{GpuMaterial{}};
			for (auto index = 0; index != capacity; ++index)
			{
				// Repeated, exactly representable depths give distinct negative collision times and equal-key stability across partitions.
				contacts[index].depth = static_cast<float>((index * 17 + index / 5) % 7) / 1024.0f;
			}
			auto counters = gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job, "Test:KeyCounters");
			auto contact_buffer = gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Test:KeyContacts");
			auto body_buffer = gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(2, {}).usage(EUsage::UnorderedAccess), job, "Test:KeyBodies");
			auto dispatch = gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}), job, "Test:KeyDispatch");
			auto upload = [&]<typename Container>(ID3D12Resource* resource, Container const& values)
			{
				// Own each upload until this job's ordinary completion fence.
				using Value = typename Container::value_type;
				auto allocation = job.m_upload.Alloc<Value>(isize(values));
				memcpy(allocation.template ptr<Value>(), values.data(), values.size() * sizeof(Value));
				job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
			};
			upload(contact_buffer.get(), contacts);
			auto const poison = std::vector<float>(capacity, -12345.0f);
			auto const poison_order = std::vector<uint32_t>(capacity, 0xdeadbeefU);
			ID3D12Resource const* sort_scratch = nullptr;
			ID3D12Resource const* payload_scratch = nullptr;
			ID3D12Resource const* histogram = nullptr;
			for (auto count : {capacity, 0, 1, 63, 64, 65, 4097, 7679, 7680, 7681, capacity, 0, 17})
			{
				// Grow once, then retain the large allocation while the current pass becomes compact, including a clamped attempted-count case.
				upload(body_buffer.get(), bodies);
				upload(counters.get(), std::array{GpuCollisionCounters{.contact_count = count == 4097 ? count + 17 : count}});
				upload(dispatch.get(), std::array{D3D12_DISPATCH_ARGUMENTS{
					.ThreadGroupCountX = static_cast<UINT>(std::max(1, (count + ResolveThreadCount - 1) / ResolveThreadCount)),
					.ThreadGroupCountY = 1,
					.ThreadGroupCountZ = 1,
				}});
				if (resolver.m_r_contact_times != nullptr)
				{
					// Poison both streams: any capacity-wide work would either move this prefix poison or overwrite the untouched suffix.
					upload(resolver.m_r_contact_times.get(), poison);
					upload(resolver.m_r_contact_order.get(), poison_order);
				}

				// Read the actual indirect records as well as the streams so correct output cannot conceal capacity-wide dispatch.
				resolver.Resolve(job, 1.0f / 60.0f, 2, 2, std::max(1, count), dispatch, counters, contact_buffer, body_buffer, materials);
				auto keys = job.m_readback.Alloc<float>(capacity);
				auto order = job.m_readback.Alloc<uint32_t>(capacity);
				auto arguments = job.m_readback.Alloc<uint32_t>(ContactSorter::IndirectArgumentBytes / sizeof(uint32_t));
				auto batches = job.m_readback.Alloc<DispatchArguments>(MaxColours);
				job.m_barriers.Transition(resolver.m_r_contact_times.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Transition(resolver.m_r_contact_order.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Transition(resolver.m_contact_sorter.m_indirect_arguments.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				job.m_cmd_list.CopyBufferRegion(keys, resolver.m_r_contact_times.get());
				job.m_cmd_list.CopyBufferRegion(order, resolver.m_r_contact_order.get());
				job.m_cmd_list.CopyBufferRegion(arguments, resolver.m_contact_sorter.m_indirect_arguments.get());
				job.m_barriers.Transition(resolver.m_r_colour_dispatch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				job.m_cmd_list.CopyBufferRegion(batches, resolver.m_r_colour_dispatch.get());
				job.Run();

				// A shared dynamic pair needs one colour per contact, then the established serial fallback. Empty frames must clear every retained argument.
				for (auto colour = 0; colour != MaxColours; ++colour)
				{
					auto const& args = batches.ptr<DispatchArguments>()[colour];
					auto const overflow = count > MaxColours;
					auto const occupied = !overflow && colour < count;
					PR_EXPECT(args.ThreadGroupCountX == (overflow ? (colour == 0 ? 1U : 0U) : (occupied ? 1U : 0U)));
					PR_EXPECT(args.ThreadGroupCountY == 1 && args.ThreadGroupCountZ == 1);
				}

				// With unit relative speed and penetration below one step of motion, earlier collision times correspond to greater depth.
				auto expected = std::vector<uint32_t>(count);
				std::iota(expected.begin(), expected.end(), 0U);
				std::stable_sort(expected.begin(), expected.end(), [&](auto lhs, auto rhs)
				{
					return contacts[lhs].depth > contacts[rhs].depth;
				});
				for (auto index = 0; index != capacity; ++index)
				{
					// Inactive storage must stay untouched even when its keys would sort before every live key.
					if (index < count)
					{
						PR_EXPECT(order.ptr<uint32_t>()[index] == expected[index]);
						PR_EXPECT(FEqlAbsolute(keys.ptr<float>()[index], -contacts[expected[index]].depth, 1.0e-7f));
					}
					else
					{
						PR_EXPECT(keys.ptr<float>()[index] == poison[index]);
						PR_EXPECT(order.ptr<uint32_t>()[index] == poison_order[index]);
					}
				}

				// Every radix pass must use the clamped live count and its partition grid, including zero work after a full-capacity pass.
				auto const blocks = static_cast<uint32_t>((count + 7679) / 7680);
				for (auto pass = 0; pass != ContactSorter::RadixPasses; ++pass)
				{
					auto const* partial = arguments.ptr<uint32_t>() + (pass * ContactSorter::IndirectArgumentsPerPass + 1) * (ContactSorter::IndirectArgumentStride / sizeof(uint32_t));
					auto const* scan = partial + ContactSorter::IndirectArgumentStride / sizeof(uint32_t);
					PR_EXPECT(partial[0] == static_cast<uint32_t>(count) && partial[2] == blocks);
					PR_EXPECT(partial[4] == blocks && partial[5] == 1 && partial[6] == 1);
					PR_EXPECT(scan[2] == blocks && scan[4] == (count != 0 ? 256U : 0U));
				}

				// Shrinking and regrowing the current-pass clamp must not churn the retained scratch allocations.
				auto const& sorter = resolver.m_contact_sorter;
				if (sort_scratch == nullptr)
				{
					sort_scratch = sorter.m_sort[1].get();
					payload_scratch = sorter.m_payload[1].get();
					histogram = sorter.m_pass_histogram.get();
				}
				PR_EXPECT(sorter.m_sort[1].get() == sort_scratch && sorter.m_payload[1].get() == payload_scratch);
				PR_EXPECT(sorter.m_pass_histogram.get() == histogram);
				PR_EXPECT(sorter.m_size == std::max(1, count));
			}
		}

		// Stable sorting must preserve physical output for small scenes and thousands of bodies with retained warm-start impulses.
		PRUnitTestMethod(SolverOutputMatchesInterop, Extended)
		{
			TestClass_ResolveRootStateTests::CheckColours(1);
			TestClass_ResolveRootStateTests::CheckColours(3);
			TestClass_ResolveRootStateTests::CheckColours(3, 1024);
		}
	};

	PRUnitTestClass(ResolveInteropRunnerTests)
	{
		PRUnitTestMethod(ContactPriorityScoresNewtonCradleOrder, Extended)
		{
			auto settings = ContactPrioritySettings{};
			settings.m_depth_bias = 0.0f;

			auto bodies = std::vector<ContactPriorityBody>(11);
			for (int body_idx = 0; body_idx != isize(bodies); ++body_idx)
			{
				bodies[body_idx].m_position_ws = v4{static_cast<float>(body_idx), 0, 0, 1};
				bodies[body_idx].m_inv_mass = 1.0f;
			}
			bodies[0].m_velocity_ws = v4{10, 0, 0, 0};

			auto contacts = std::vector<ContactPriorityContact>{};
			for (int contact_idx = 0; contact_idx != 10; ++contact_idx)
			{
				contacts.push_back(ContactPriorityContact{
					.m_body_idx_a = contact_idx,
					.m_body_idx_b = contact_idx + 1,
					.m_axis_ws = v4{1, 0, 0, 0},
					.m_point_ws = v4{static_cast<float>(contact_idx) + 0.5f, 0, 0, 1},
				});
			}

			auto ideal_order = std::vector<int>(contacts.size());
			std::iota(ideal_order.begin(), ideal_order.end(), 0);
			auto reversed_order = ideal_order;
			std::reverse(reversed_order.begin(), reversed_order.end());

			auto const ideal = EvaluateContactPriority(settings, bodies, contacts, ideal_order);
			auto const reversed = EvaluateContactPriority(settings, bodies, contacts, reversed_order);
			auto const priority_order = ContactPriorityOrder(settings, bodies, contacts);

			PR_EXPECT(ideal.m_total_help_weight > 0.0f);
			PR_EXPECT(ideal.m_score > 0.99f);
			PR_EXPECT(reversed.m_score < 0.01f);
			PR_EXPECT(priority_order == ideal_order);
		}

		PRUnitTestMethod(ContactPriorityScoresSupportLoadOrder, Extended)
		{
			auto settings = ContactPrioritySettings{};
			settings.m_depth_bias = 0.0f;

			auto bodies = std::vector<ContactPriorityBody>(3);
			bodies[0].m_position_ws = v4{0, 0, -1, 1};
			bodies[1].m_position_ws = v4{0, 0, 0, 1};
			bodies[2].m_position_ws = v4{0, 0, 1, 1};
			bodies[0].m_inv_mass = 0.0f;
			bodies[1].m_inv_mass = 1.0f;
			bodies[2].m_inv_mass = 1.0f;
			bodies[2].m_velocity_ws = v4{0, 0, -10, 0};

			auto contacts = std::vector<ContactPriorityContact>{
				ContactPriorityContact{
					.m_body_idx_a = 1,
					.m_body_idx_b = 0,
					.m_axis_ws = v4{0, 0, -1, 0},
					.m_point_ws = v4{0, 0, -0.5f, 1},
				},
				ContactPriorityContact{
					.m_body_idx_a = 1,
					.m_body_idx_b = 2,
					.m_axis_ws = v4{0, 0, +1, 0},
					.m_point_ws = v4{0, 0, +0.5f, 1},
				},
			};

			auto load_first_order = std::vector<int>{1, 0};
			auto ground_first_order = std::vector<int>{0, 1};

			auto const load_first = EvaluateContactPriority(settings, bodies, contacts, load_first_order);
			auto const ground_first = EvaluateContactPriority(settings, bodies, contacts, ground_first_order);
			auto const priority_order = ContactPriorityOrder(settings, bodies, contacts);

			PR_EXPECT(load_first.m_total_help_weight > 0.0f);
			PR_EXPECT(load_first.m_score > 0.99f);
			PR_EXPECT(ground_first.m_score < 0.01f);
			PR_EXPECT(priority_order == load_first_order);
		}

		PRUnitTestMethod(ContactPrioritySortsInteropCradleContacts, Extended)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 0;
			config.contact_sort_shock_iterations = 16;

			auto box = collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}};
			auto rbodies = std::vector<RigidBody>{};
			for (int body_idx = 0; body_idx != 11; ++body_idx)
			{
				rbodies.push_back(RigidBody{&box, m4x4::Translation(static_cast<float>(body_idx), 0, 0), Inertia::Box(box.m_radius, 1.0f)});
				rbodies.back().VelocityWS(v4::Zero(), body_idx == 0 ? v4{10, 0, 0, 0} : v4::Zero());
			}

			auto bodies = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != isize(rbodies); ++body_idx)
				bodies.push_back(PackDynamics(rbodies[body_idx], 0));

			auto contacts = std::vector<GpuResolveContact>{};
			for (int contact_idx = 0; contact_idx != 10; ++contact_idx)
			{
				auto const& body_a = rbodies[contact_idx];
				auto const& body_b = rbodies[contact_idx + 1];
				auto const contact_point = v4{0.5f, 0, 0, 1};
				contacts.push_back(GpuResolveContact{
					.axis = v4{1, 0, 0, 0},
					.contact_point = contact_point,
					.manifold = {contact_point},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = contact_idx,
					.body_idx_b = contact_idx + 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.0f,
					.feature = 1,
				});
			}
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.ComputeShockRanks();
			runner.SortContacts();

			for (int contact_idx = 0; contact_idx != isize(contacts); ++contact_idx)
				PR_EXPECT(runner.ContactOrder()[contact_idx] == static_cast<uint32_t>(contact_idx));

			auto const priority = runner.ContactPriority();
			PR_EXPECT(priority.m_score > 0.99f);
		}

		PRUnitTestMethod(ContactPriorityParallelPropagatesAlongChain, Extended)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 0;
			config.contact_sort_shock_iterations = 16;

			auto box = collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}};
			auto rbodies = std::vector<RigidBody>{};
			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				rbodies.push_back(RigidBody{&box, m4x4::Translation(static_cast<float>(body_idx), 0, 0), Inertia::Box(box.m_radius, 1.0f)});
				rbodies.back().VelocityWS(v4::Zero(), body_idx == 0 ? v4{10, 0, 0, 0} : v4::Zero());
			}

			auto bodies = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != isize(rbodies); ++body_idx)
				bodies.push_back(PackDynamics(rbodies[body_idx], 0));

			auto contacts = std::vector<GpuResolveContact>{};
			for (int contact_idx = 0; contact_idx != 3; ++contact_idx)
			{
				auto const& body_a = rbodies[contact_idx];
				auto const& body_b = rbodies[contact_idx + 1];
				auto const contact_point = v4{0.5f, 0, 0, 1};
				contacts.push_back(GpuResolveContact{
					.axis = v4{1, 0, 0, 0},
					.contact_point = contact_point,
					.manifold = {contact_point},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = contact_idx,
					.body_idx_b = contact_idx + 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.0f,
					.feature = 1,
				});
			}
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};

			auto runner = ResolveInteropRunner{config};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.ComputeShockRanks();

			PR_EXPECT(runner.ContactTimes()[1] < -1e-4f);
			PR_EXPECT(runner.ContactTimes()[2] < -1e-4f);
		}

		PRUnitTestMethod(ContactPrioritySortsInteropSupportContactsBottomToTop, Extended)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 0;
			config.contact_sort_shock_iterations = 16;

			auto box = collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}};
			auto ground = collision::ShapeBox{v4{5.0f, 5.0f, 0.5f, 0}};
			auto rbodies = std::vector<RigidBody>{};
			rbodies.push_back(RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()});
			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				rbodies.push_back(RigidBody{&box, m4x4::Translation(0, 0, 0.5f + static_cast<float>(body_idx)), Inertia::Box(box.m_radius, 1.0f)});
				rbodies.back().GravityWS(v4{0, 0, -9.81f, 0});
			}

			auto bodies = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != isize(rbodies); ++body_idx)
				bodies.push_back(PackDynamics(rbodies[body_idx], 0));

			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = v4{0, 0, +1, 0},
					.contact_point = v4{0, 0, +0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[3].O2W()) * rbodies[4].O2W(),
					.body_idx_a = 3,
					.body_idx_b = 4,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
				GpuResolveContact{
					.axis = v4{0, 0, +1, 0},
					.contact_point = v4{0, 0, +0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[2].O2W()) * rbodies[3].O2W(),
					.body_idx_a = 2,
					.body_idx_b = 3,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
				GpuResolveContact{
					.axis = v4{0, 0, +1, 0},
					.contact_point = v4{0, 0, +0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[1].O2W()) * rbodies[2].O2W(),
					.body_idx_a = 1,
					.body_idx_b = 2,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
				GpuResolveContact{
					.axis = v4{0, 0, -1, 0},
					.contact_point = v4{0, 0, -0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[1].O2W()) * rbodies[0].O2W(),
					.body_idx_a = 1,
					.body_idx_b = 0,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.ComputeShockRanks();
			runner.SortContacts();

			auto const expected_order = std::array<uint32_t, 4>{3, 2, 1, 0};
			for (int order_idx = 0; order_idx != isize(expected_order); ++order_idx)
				PR_EXPECT(runner.ContactOrder()[order_idx] == expected_order[order_idx]);
		}

		PRUnitTestMethod(SplitPositionSolveSeparatesBodies, Extended)
		{
			auto config = EngineConfig{};
			config.solver_iterations = 0;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto body_a = RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&box, m4x4::Translation(0.9f, 0, 0), Inertia::Box(box.m_radius, 1.0f)};
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 0),
			};

			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = v4{1, 0, 0, 0},
					.contact_point = v4{0.45f, 0, 0, 1},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			auto total_correction = config.position_baumgarte * (contacts[0].depth - config.position_slop);
			auto expected_shift = 0.5f * total_correction;
			PR_EXPECT(FEqlAbsolute(bodies[0].o2w.pos.x, -expected_shift, 1e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[1].o2w.pos.x, 0.9f + expected_shift, 1e-5f));
			PR_EXPECT(runner.Colours()[0] == 0);
			PR_EXPECT(runner.ContactOrder()[0] == 0);
		}

		PRUnitTestMethod(ColourExhaustionUsesCoherentFallback, Extended)
		{
			// Disable unrelated corrections so the final contact's velocity impulse is the only observable solver change.
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.velocity_baumgarte = 0.0f;
			config.contact_sort_propagation_scale = 0.0f;
			config.warm_start_scale = 0.0f;

			// One dynamic hub and 33 static leaves require 33 edge colours because every contact shares the hub.
			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto hub = RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			auto leaf = RigidBody{&box, m4x4::Identity(), Inertia::Infinite()};
			hub.VelocityWS(v4::Zero(), v4{1, 0, 0, 0});

			auto bodies = std::vector<GpuRigidBody>{};
			bodies.reserve(MaxColours + 2);
			bodies.push_back(PackDynamics(hub, 0));
			for (int leaf_idx = 0; leaf_idx != MaxColours + 1; ++leaf_idx)
				bodies.push_back(PackDynamics(leaf, leaf_idx + 1));

			// The first 32 contacts are separating but still consume colours. The final contact is closing, so its impulse proves the overflow fallback ran.
			auto contacts = std::vector<GpuResolveContact>{};
			contacts.reserve(MaxColours + 1);
			for (int contact_idx = 0; contact_idx != MaxColours + 1; ++contact_idx)
			{
				auto const axis = contact_idx != MaxColours ? v4{-1, 0, 0, 0} : v4{+1, 0, 0, 0};
				contacts.push_back(GpuResolveContact{
					.axis = axis,
					.contact_point = v4{0, 0, 0, 1},
					.manifold = {v4{0, 0, 0, 1}},
					.b2a = m4x4::Identity(),
					.body_idx_a = 0,
					.body_idx_b = contact_idx + 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				});
			}
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			auto gpu_bodies = bodies;

			// Assign colours in declaration order so the closing contact is the first edge that cannot acquire a free bit.
			auto runner = ResolveInteropRunner{config};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.AssignColours();

			// Overflow invalidates every partial colour so later parallel dispatches cannot repeat work already handled by the serial colour-zero sweep.
			for (auto const colour : runner.Colours())
				PR_EXPECT(colour == MaxColours);

			PR_EXPECT(runner.ColourOverflow());

			// Colour zero owns the serial overflow sweep. The closing sentinel contact must still reduce the hub's positive x momentum.
			auto const momentum_before = bodies[0].momentum_lin.x;
			runner.ResolveVelocity(0);
			runner.Store(buffers);
			PR_EXPECT(std::isfinite(bodies[0].momentum_lin.x));
			PR_EXPECT(bodies[0].momentum_lin.x < momentum_before);

			// Run the complete production D3D12 path to prove its sorted graph also selects the fallback and resolves the enabled closing contact.
			auto& gpu = SharedTestGpu();
			auto gpu_resolver = GpuResolver{gpu, config, &SharedTestShaderCache()};
			gpu_resolver.Resolve(gpu.m_job, buffers.m_dt, contacts, gpu_bodies, materials);

			// Read only the reserved diagnostic slot after the test resolve; production frames never add this readback.
			auto overflow_readback = ReadbackAlloc{};
			{
				gpu.m_job.m_barriers.Transition(gpu_resolver.m_r_colours.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Commit();

				overflow_readback = gpu.m_job.m_readback.Alloc<uint32_t>(1);
				gpu.m_job.m_cmd_list.CopyBufferRegion(overflow_readback, gpu_resolver.m_r_colours.get(), gpu_resolver.m_max_contacts * sizeof(uint32_t));

				gpu.m_job.m_barriers.Transition(gpu_resolver.m_r_colours.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				gpu.m_job.m_barriers.Commit();
			}
			gpu.m_job.Run();

			PR_EXPECT(*overflow_readback.ptr<uint32_t>() != 0);
			PR_EXPECT(std::isfinite(gpu_bodies[0].momentum_lin.x));
			PR_EXPECT(gpu_bodies[0].momentum_lin.x < momentum_before);
		}

		PRUnitTestMethod(RotatedBodyVelocityResolveUsesContactSpace, Extended)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 8;
			config.velocity_baumgarte = 0.0f;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto ground = collision::ShapeBox{v4{10, 10, 0.5f, 0}};
			auto body_a = RigidBody{&box, m4x4::TransformDeg(10, 20, 30, v4{0, 0, 0.5f, 1}), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body_a.VelocityWS(v4::Zero(), v4{0, 0, -10.0f, 0});

			auto const w2a = InvertOrthonormal(body_a.O2W());
			auto const contact_point_ws = v4{0.15f, -0.1f, 0, 1};
			auto const axis_ws = v4{0, 0, -1, 0};
			auto const ke_before = body_a.KineticEnergy();
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			UnpackDynamics(bodies[0], body_a);
			auto const vel_after = body_a.VelocityWS();
			auto const vel_at_contact = vel_after.LinAt(contact_point_ws - body_a.CentreOfMassPositionWS());
			auto const closing_after = Dot3(-vel_at_contact, axis_ws);
			PR_EXPECT(vel_after.lin.z > -9.0f);
			PR_EXPECT(closing_after > -0.1f);
			PR_EXPECT(body_a.KineticEnergy() <= ke_before + 1e-4f);
		}

		PRUnitTestMethod(VelocityBiasSurvivesEnergyGuard, Extended)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.velocity_baumgarte = 0.2f;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto ground = collision::ShapeBox{v4{10, 10, 0.5f, 0}};
			auto body_a = RigidBody{&box, m4x4::Translation(0, 0, 0.5f), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};

			auto const contact_point_ws = v4{0.2f, -0.15f, 0, 1};
			auto const axis_ws = v4{0, 0, -1, 0};
			auto const w2a = InvertOrthonormal(body_a.O2W());
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			UnpackDynamics(bodies[0], body_a);
			auto const vel_after = body_a.VelocityWS();
			auto const vel_at_contact = vel_after.LinAt(contact_point_ws - body_a.CentreOfMassPositionWS());
			auto const sep_speed = Dot3(-vel_at_contact, axis_ws);
			PR_EXPECT(sep_speed > 0.5f);
		}

		// Preserve a legitimate energy-neutral restitution target while the passive-work guard rejects only numerical energy gain.
		PRUnitTestMethod(RestitutionSurvivesPassiveEnergyGuard, Quick)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 2;
			config.velocity_baumgarte = 0.0f;

			auto sphere = collision::ShapeSphere{0.5f};
			auto body_a = RigidBody{&sphere, m4x4::Translation(-0.5f, 0, 0), Inertia::Sphere(sphere.m_radius, 1.0f)};
			auto body_b = RigidBody{&sphere, m4x4::Translation(+0.5f, 0, 0), Inertia::Sphere(sphere.m_radius, 1.0f)};
			body_a.VelocityWS(v4::Zero(), +v4::XAxis());
			body_b.VelocityWS(v4::Zero(), -v4::XAxis());
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};

			// The symmetric contact has a closed-form perfectly elastic result: equal masses exchange velocities without changing energy.
			auto const contact_point = v4{0.5f, 0, 0, 1};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = v4::XAxis(),
					.contact_point = contact_point,
					.manifold = {contact_point},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.0f,
					.feature = 1,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 1.0f,
				},
			};
			auto const energy_before = body_a.KineticEnergy() + body_b.KineticEnergy();

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});
			UnpackDynamics(bodies[0], body_a);
			UnpackDynamics(bodies[1], body_b);

			// A passive clamp that accidentally treats restitution as zero-target work would leave both normal velocities near zero.
			auto const energy_after = body_a.KineticEnergy() + body_b.KineticEnergy();
			PR_EXPECT(FEqlAbsolute(body_a.VelocityWS().lin.x, -1.0f, 2.0e-4f));
			PR_EXPECT(FEqlAbsolute(body_b.VelocityWS().lin.x, +1.0f, 2.0e-4f));
			PR_EXPECT(FEqlAbsolute(body_a.MomentumWS().lin.x + body_b.MomentumWS().lin.x, 0.0f, 2.0e-5f));
			PR_EXPECT(energy_after <= energy_before + 2.0e-4f);
			PR_EXPECT(energy_after >= energy_before - 2.0e-4f);
		}

		// Diagnostic: replicate the failing-frame state of BoxDropEnergyConservation around frame 100.
		// At that point the trace shows the rotated box at z=-0.32, vz=-8 m/s, with contact depth 0.55.
		// The bias should reverse vz to ~+5 m/s but it doesn't — body keeps falling.
		// This test runs ONE Resolve pass on that exact state and asserts vz separates.
		PRUnitTestMethod(BoxDropFailingFrameDiagnostic, Extended)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;     // isolate velocity solver
			config.solver_iterations = 1;       // ONE pass to see what happens
			config.velocity_baumgarte = 0.2f;
			config.penetration_slop = 0.005f;

			auto box = collision::ShapeBox{v4{0.5f, 0.65f, 0.9f, 0}};
			auto ground = collision::ShapeBox{v4{50.0f, 50.0f, 0.5f, 0}};

			// State as captured at frame ~100 of the failing trace
			auto box_o2w = m4x4::Transform(RotationRad<m3x3>(constants<float>::tau_by_8, 0, 0), v4{-3.67f, 0, -0.32f, 1});
			auto body_a = RigidBody{&box, box_o2w, Inertia::Box(box.m_radius, 10.0f)};
			body_a.VelocityWS(v4::Zero(), v4{0.8f, 0, -8.38f, 0});

			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};

			// Contact roughly matches what BoxVsBox SAT would produce: ground top normal in world.
			// Contact point = lowest box corner projected onto ground top.
			auto const contact_point_ws = v4{-3.67f, 0, 0, 1};
			auto const axis_ws = v4{0, 0, -1, 0};  // A=box → B=ground points DOWN
			auto const w2a = InvertOrthonormal(body_a.O2W());

			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.55f,
					.collision_time = 0,
					.feature = 1,
				},
			};
			contacts[0].manifold[0] = w2a * contact_point_ws;
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.3f,
				},
			};

			auto const ke_before = body_a.KineticEnergy();
			auto const vz_before = body_a.VelocityWS().lin.z;

			// Step the simulation iteration-by-iteration to see how velocity evolves
			auto runner = ResolveInteropRunner{config};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.SortContacts();
			runner.AssignColours();

			// Dump file
			auto dump_path = std::filesystem::path("C:/Users/paulryland/.copilot/session-state/1277a797-7dbe-4e04-9ffb-e2e396f6db2a/files/boxdrop_diag_frame.log");
			auto f = std::ofstream(dump_path);
			f << std::format("Failing-frame diagnostic: rotated box at deep penetration\n");
			f << std::format("  vz_before = {:.3f}, KE_before = {:.3f}\n", vz_before, ke_before);

			auto contact_pt_a = contacts[0].contact_point;
			auto axis_a = contacts[0].axis;
			f << std::format("  contact_pt_in_A = ({:.3f}, {:.3f}, {:.3f})\n", contact_pt_a.x, contact_pt_a.y, contact_pt_a.z);
			f << std::format("  axis_in_A      = ({:.3f}, {:.3f}, {:.3f})\n", axis_a.x, axis_a.y, axis_a.z);

			for (int iter = 0; iter != 8; ++iter)
			{
				for (int colour = 0; colour != MaxColours; ++colour)
					runner.ResolveVelocity(colour);

				runner.Store(buffers);
				UnpackDynamics(bodies[0], body_a);
				auto v = body_a.VelocityWS();
				f << std::format("  iter {}: vz={:.3f} vx={:.3f} omega=({:.3f},{:.3f},{:.3f}) KE={:.3f}\n",
					iter, v.lin.z, v.lin.x, v.ang.x, v.ang.y, v.ang.z, body_a.KineticEnergy());
			}
			f.close();

			runner.Store(buffers);
			UnpackDynamics(bodies[0], body_a);
			auto const vel_after = body_a.VelocityWS();
			auto const vz_after = vel_after.lin.z;

			// After resolve, body should be moving UP (vz > 0) — bias should overcome gravity.
			PR_EXPECT(vz_after > 0.0f);
		}
	};

	// Cache identity and duplicate selection must be independent of contact append and GPU insertion order.
	PRUnitTestClass(ResolveWarmStartIdentityTests)
	{
		// A centroid normal impulse, with all other contact inputs fixed.
		static GpuResolveContact Contact(v4 point, float impulse, int child = 0)
		{
			return GpuResolveContact{
				.axis = v4::ZAxis(), .contact_point = point, .manifold = {point}, .b2a = m4x4::Identity(),
				.body_idx_a = 0, .body_idx_b = 1, .feature = 1, .child_idx_a = child,
				.warmstart_impulse = v4{0, 0, impulse, 0},
			};
		}

		PRUnitTestMethod(PermutationsChooseOneDuplicateAndMatchFullIdentity, Quick)
		{
			auto config = EngineConfig{};
			config.warm_start_scale = 1.0f;
			auto runner = ResolveInteropRunner{config};
			auto const contacts = std::array{
				Contact(v4::Origin(), 3), Contact(v4::Origin(), 2),
				Contact(v4{0.5f, 0, 0, 1}, 5), Contact(v4::Origin(), 7, 65536),
			};
			auto order = std::array{0, 1, 2, 3};
			do
			{
				auto permuted = std::vector<GpuResolveContact>{};
				for (auto index : order)
					permuted.push_back(contacts[index]);

				auto const cache = runner.StoreWarmStart(permuted, 8);
				PR_EXPECT(All(runner.LoadWarmStart(cache, contacts[0]) == v4(0, 0, 2, 0)));
				PR_EXPECT(All(runner.LoadWarmStart(cache, contacts[2]) == v4(0, 0, 5, 0)));
				PR_EXPECT(All(runner.LoadWarmStart(cache, contacts[3]) == v4(0, 0, 7, 0)));
			}
			while (std::ranges::next_permutation(order).found);

			// Equal-length directions must not select whichever normal happened to be inserted first.
			auto ties = std::array{contacts[1], contacts[1]};
			ties[1].axis = v4::YAxis();
			ties[1].warmstart_impulse = v4{0, 2, 0, 0};
			for (int pass = 0; pass != 2; ++pass)
			{
				PR_EXPECT(All(runner.LoadWarmStart(runner.StoreWarmStart(ties, 4), contacts[1]) == v4(0, 0, 2, 0)));
				std::ranges::reverse(ties);
			}

			// Overflow of the priority's squared magnitude still has a total raw-bit tie-break among finite impulses.
			auto large = std::array{Contact(v4::Origin(), 3.0e20f), Contact(v4::Origin(), 2.0e20f)};
			for (int pass = 0; pass != 2; ++pass)
			{
				PR_EXPECT(All(runner.LoadWarmStart(runner.StoreWarmStart(large, 4), contacts[1]) == v4(0, 0, 2.0e20f, 0)));
				std::ranges::reverse(large);
			}

			// Force equal hashes while changing each formerly missing identity component.
			auto const seed = std::span{contacts}.first(1);
			for (int changed = 0; changed != 3; ++changed)
			{
				auto cache = runner.StoreWarmStart(seed, 4);
				auto& entry = *std::ranges::find_if(cache, [](auto const& value) { return value.key != 0; });
				switch (changed)
				{
					case 0: { ++entry.point_key_x; break; }
					case 1: { entry.child_idx_a += 65536; break; }
					case 2: { entry.child_idx_b += 65536; break; }
					default: { throw std::logic_error("Unknown identity discriminator"); }
				}
				PR_EXPECT(All(runner.LoadWarmStart(cache, contacts[0]) == v4::Zero()));
			}
		}

		PRUnitTestMethod(LongProbeChainsAndFinitePointRange, Quick)
		{
			auto config = EngineConfig{};
			config.warm_start_scale = 1.0f;
			auto runner = ResolveInteropRunner{config};
			auto contacts = std::vector<GpuResolveContact>{};
			for (int index = 0; index != 48; ++index)
				contacts.push_back(Contact(v4::Origin(), static_cast<float>(49 - index)));

			auto const cache = runner.StoreWarmStart(contacts, 128);
			PR_EXPECT(std::ranges::count_if(cache, [](auto const& entry) { return entry.key != 0; }) == 48);
			PR_EXPECT(All(runner.LoadWarmStart(cache, contacts[0]) == v4(0, 0, 2, 0)));
			std::ranges::reverse(contacts);
			PR_EXPECT(All(runner.LoadWarmStart(runner.StoreWarmStart(contacts, 128), contacts[0]) == v4(0, 0, 2, 0)));

			// Coordinates beyond the old int32 cell range remain distinct, without multiplying the largest finite value.
			auto const largest = std::numeric_limits<float>::max();
			auto const points = std::array{
				Contact(v4{largest, -largest, 0, 1}, 3),
				Contact(v4{std::nextafter(largest, 0.0f), -largest, 0, 1}, 5),
				Contact(v4{131072, -131072, 0, 1}, 7),
				Contact(v4{131072.015625f, -131072, 0, 1}, 9),
			};
			auto const wide = runner.StoreWarmStart(points, 8);
			for (auto const& point : points)
				PR_EXPECT(All(runner.LoadWarmStart(wide, point) == point.warmstart_impulse));

			// Exact grid points stay fixed at both sides of the exponent boundary; signed zeros share a cell.
			for (auto coordinate : {131071.984375f, -131071.984375f, 131072.0f, -131072.0f, -0.0f})
			{
				auto const point = std::array{Contact(v4{coordinate, 0, 0, 1}, 2)};
				auto const entries = runner.StoreWarmStart(point, 2);
				auto const& entry = *std::ranges::find_if(entries, [](auto const& value) { return value.key != 0; });
				PR_EXPECT(entry.point_key_x == std::bit_cast<uint32_t>(coordinate == 0.0f ? 0.0f : coordinate));
			}
		}

		// A structured grid must distribute high coordinate bits into the low power-of-two bucket bits.
		PRUnitTestMethod(DistinctGridPointsHaveDistributedProbeChains, Quick)
		{
			auto config = EngineConfig{};
			config.warm_start_scale = 1.0f;
			auto runner = ResolveInteropRunner{config};
			auto contacts = std::vector<GpuResolveContact>{};
			for (int index = 0; index != 128; ++index)
				contacts.push_back(Contact(v4{index / 64.0f, 0, 0, 1}, static_cast<float>(index + 1)));

			// Recover exact insertion distances and complete-chain lookup work from the production-generated table.
			auto const cache = runner.StoreWarmStart(contacts, 256);
			auto buckets = std::array<bool, 256>{};
			auto insertion_probes = 0;
			auto maximum_lookup_probes = 0;
			for (int slot = 0; slot != 256; ++slot)
			{
				if (cache[slot].key == 0)
					continue;

				auto const home = static_cast<int>(cache[slot].key % 256U);
				buckets[home] = true;
				insertion_probes += (slot + 256 - home) % 256 + 1;
				auto lookup_probes = 1;
				while (cache[(home + lookup_probes - 1) % 256].key != 0)
					++lookup_probes;

				maximum_lookup_probes = std::max(maximum_lookup_probes, lookup_probes);
			}
			auto const distinct_buckets = std::ranges::count(buckets, true);
			log_stream() << "grid128 capacity256 buckets=" << distinct_buckets << " insertion_probes=" << insertion_probes << " max_lookup_probes=" << maximum_lookup_probes << '\n';
			PR_EXPECT(distinct_buckets >= 64);
			PR_EXPECT(insertion_probes <= 512);
			PR_EXPECT(maximum_lookup_probes <= 32);
			for (auto const& contact : contacts)
				PR_EXPECT(All(runner.LoadWarmStart(cache, contact) == contact.warmstart_impulse));
		}

		// Raw GPU cache helpers copy/select hints without solving or publishing physical motion.
		PRUnitTestMethod(HardwareCacheHelpersRetainEveryDuplicate, Extended)
		{
			using namespace pr::compute;
			auto config = EngineConfig{};
			config.warm_start_scale = 1.0f;
			auto replay = ResolveInteropRunner{config};
			auto input = std::vector<GpuResolveContact>{};
			for (int index = 0; index != 48; ++index)
				input.push_back(Contact(v4::Origin(), static_cast<float>(49 - index)));

			// The selected duplicate lies past 32 occupied slots; every GPU store must claim its own output entry.
			auto const previous = replay.StoreWarmStart(input, 128);
			auto const empty = std::vector<GpuWarmStartEntry>(128);
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto signature = RootSig(ERootSigFlags::ComputeOnly).U32<GpuResolveParams>(ECBufReg::b0).SRV(ESRVReg::t0)
				.UAV(EUAVReg::u0).UAV(EUAVReg::u2).UAV(EUAVReg::u8).UAV(EUAVReg::u9).Create(gpu, "CacheHelpers:Signature");
			auto load = ComputePSO(signature.get(), shader_code::load_warm_start).Create(gpu, "CacheHelpers:Load");
			auto store = ComputePSO(signature.get(), shader_code::store_warm_start).Create(gpu, "CacheHelpers:Store");
			auto counters = gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job, "CacheHelpers:Counters");
			auto bodies = gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(2, {}).usage(EUsage::UnorderedAccess), job, "CacheHelpers:Bodies");
			auto contacts = gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(48, {}).usage(EUsage::UnorderedAccess), job, "CacheHelpers:Contacts");
			auto prev = gpu.CreateResource(ResDesc::Buf<GpuWarmStartEntry>(128, {}).usage(EUsage::UnorderedAccess), job, "CacheHelpers:Previous");
			auto curr = gpu.CreateResource(ResDesc::Buf<GpuWarmStartEntry>(128, {}).usage(EUsage::UnorderedAccess), job, "CacheHelpers:Current");
			auto upload = [&]<typename Container>(ID3D12Resource* resource, Container const& values)
			{
				using Value = typename Container::value_type;
				auto allocation = job.m_upload.Alloc<Value>(isize(values));
				memcpy(allocation.template ptr<Value>(), values.data(), values.size() * sizeof(Value));
				job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
			};
			upload(counters.get(), std::array{GpuCollisionCounters{.contact_count = 48}});
			upload(bodies.get(), std::array<GpuRigidBody, 2>{});
			upload(contacts.get(), input);
			upload(prev.get(), previous);
			upload(curr.get(), empty);
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(prev.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(curr.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			job.m_cmd_list.SetPipelineState(load.get());
			job.m_cmd_list.SetComputeRootSignature(signature.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(GpuResolveParams{.max_contacts = 48, .body_count = 2, .warm_start_scale = 1, .warm_start_capacity = 128, .rigid_body_count = 2});
			job.m_cmd_list.AddComputeRootShaderResourceView(counters);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(prev);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(curr);
			job.m_cmd_list.Dispatch((48 + ResolveThreadCount - 1) / ResolveThreadCount, 1, 1);
			job.m_barriers.UAV(contacts.get()).Commit();
			job.m_cmd_list.SetPipelineState(store.get());
			job.m_cmd_list.Dispatch((48 + ResolveThreadCount - 1) / ResolveThreadCount, 1, 1);

			// Check the actual loaded hints and all concurrently inserted output records, independently of the resolver owner.
			auto loaded = job.m_readback.Alloc<GpuResolveContact>(48);
			auto stored = job.m_readback.Alloc<GpuWarmStartEntry>(128);
			job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(curr.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			job.m_cmd_list.CopyBufferRegion(loaded, contacts.get());
			job.m_cmd_list.CopyBufferRegion(stored, curr.get());
			job.Run();
			for (int index = 0; index != 48; ++index)
				PR_EXPECT(All(loaded.ptr<GpuResolveContact>()[index].warmstart_impulse == v4(0, 0, 2, 0)));

			auto const entries = std::span{stored.ptr<GpuWarmStartEntry>(), size_t{128}};
			PR_EXPECT(std::ranges::count_if(entries, [](auto const& entry) { return entry.key != 0; }) == 48);
			PR_EXPECT(All(replay.LoadWarmStart(entries, input[0]) == v4(0, 0, 2, 0)));
		}

		// Zero velocity sweeps cannot publish provisional cache impulses as physical response.
		PRUnitTestMethod(ZeroVelocitySweepsDoNotPublishWarmHints, Extended)
		{
			// Exercise the real resolver owner with previously accepted hints and unchanged free motion.
			auto config = EngineConfig{};
			config.warm_start_scale = 1.0f;
			config.push_out_iterations = 0;
			config.solver_iterations = 0;
			config.contact_sort_propagation_scale = 0.0f;
			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto hub = RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			auto wall = RigidBody{&box, m4x4::Identity(), Inertia::Infinite()};
			hub.VelocityWS(v4::Zero(), v4{0, 0, 100, 0});
			auto const initial = std::array{PackDynamics(hub, 0), PackDynamics(wall, 1)};
			auto bodies = initial;
			auto contacts = std::vector<GpuResolveContact>(48, Contact(v4::Origin(), 0));
			auto materials = std::array{GpuMaterial{.friction_static = 0.0f, .elasticity_norm = 0.0f}};
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto resolver = GpuResolver{gpu, config, &SharedTestShaderCache()};
			resolver.Resolve(job, 1.0f / 60, contacts, bodies, materials);

			// A nonempty previous cache must not cause physical application or promotion when no velocity solve was requested.
			auto cached_contacts = contacts;
			for (int index = 0; index != isize(cached_contacts); ++index)
				cached_contacts[index].warmstart_impulse.z = static_cast<float>(49 - index);

			auto replay = ResolveInteropRunner{config};
			auto const cache = replay.StoreWarmStart(cached_contacts, resolver.m_warm_start_capacity);
			auto upload = job.m_upload.Alloc<GpuWarmStartEntry>(isize(cache));
			std::copy(cache.begin(), cache.end(), upload.ptr<GpuWarmStartEntry>());
			job.m_barriers.Transition(resolver.m_r_warm_start_prev.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyBufferRegion(resolver.m_r_warm_start_prev.get(), 0, upload);
			job.Run();
			bodies = initial;
			resolver.Resolve(job, 1.0f / 60, contacts, bodies, materials);
			PR_EXPECT(All(bodies[0].momentum_lin == initial[0].momentum_lin));
			PR_EXPECT(All(bodies[0].momentum_ang == initial[0].momentum_ang));
			PR_EXPECT(memcmp(&bodies[0].o2w, &initial[0].o2w, sizeof(m4x4)) == 0);
			for (auto const& contact : contacts)
				PR_EXPECT(All(contact.warmstart_impulse == v4::Zero()));

			// Retirement selects the cleared output cache, not the previous guesses or unsolved contact totals.
			auto readback = job.m_readback.Alloc<GpuWarmStartEntry>(resolver.m_warm_start_capacity);
			job.m_barriers.Transition(resolver.m_r_warm_start_prev.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			job.m_cmd_list.CopyBufferRegion(readback, resolver.m_r_warm_start_prev.get());
			job.Run();
			auto const stored = std::span{readback.ptr<GpuWarmStartEntry>(), static_cast<size_t>(resolver.m_warm_start_capacity)};
			PR_EXPECT(std::ranges::count_if(stored, [](auto const& entry) { return entry.key != 0; }) == 0);
			PR_EXPECT(All(replay.LoadWarmStart(stored, contacts[0]) == v4::Zero()));

			// Enabling a real solve still produces and promotes its physical impulse.
			config.solver_iterations = 1;
			bodies = initial;
			contacts.resize(1);
			resolver.Resolve(job, 1.0f / 60, contacts, bodies, materials);
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin.z, 0.0f, 1.0e-5f));
			auto solved_readback = job.m_readback.Alloc<GpuWarmStartEntry>(resolver.m_warm_start_capacity);
			job.m_barriers.Transition(resolver.m_r_warm_start_prev.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			job.m_cmd_list.CopyBufferRegion(solved_readback, resolver.m_r_warm_start_prev.get());
			job.Run();
			auto const solved = std::span{solved_readback.ptr<GpuWarmStartEntry>(), static_cast<size_t>(resolver.m_warm_start_capacity)};
			PR_EXPECT(std::ranges::count_if(solved, [](auto const& entry) { return entry.key != 0; }) == 1);
			PR_EXPECT(All(replay.LoadWarmStart(solved, contacts[0]) == v4(0, 0, 100, 0)));
		}
	};

	// Packed slots may be reused only while their exact persistent owner sequence is unchanged.
	PRUnitTestClass(ContactPackedLayoutTests)
	{
		// Reordering identical live shapes is an identity change even when every geometric cache key still matches.
		PRUnitTestMethod(TracksBodiesTreesReplacementAndEmptyFrames, Quick)
		{
			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto make_body = [&]
			{
				return RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			};
			auto storage = std::array{make_body(), make_body(), make_body(), make_body(), make_body()};
			auto bodies = std::array{&storage[0], &storage[1], &storage[2], &storage[3]};
			auto cache = EngineBufferCache{};
			PR_EXPECT(cache.UpdateContactLayout(bodies, {}));
			cache.NewFrame();
			PR_EXPECT(!cache.UpdateContactLayout(bodies, {}));
			std::ranges::rotate(bodies, bodies.begin() + 2);
			PR_EXPECT(cache.UpdateContactLayout(bodies, {}));
			PR_EXPECT(!cache.UpdateContactLayout(bodies, {}));

			// Relocating one logical owner changes its address but not its identity; never submit both copies together.
			auto relocated = storage[0];
			bodies[2] = &relocated;
			PR_EXPECT(!cache.UpdateContactLayout(bodies, {}));
			bodies[2] = &storage[0];
			PR_EXPECT(!cache.UpdateContactLayout(bodies, {}));
			bodies[3] = &storage[4];
			PR_EXPECT(cache.UpdateContactLayout(bodies, {}));
			PR_EXPECT(cache.UpdateContactLayout(std::span{bodies}.first(3), {}));
			PR_EXPECT(cache.UpdateContactLayout(bodies, {}));

			// Immutable topology makes the ordered tree identities sufficient for every packed link slot.
			auto make_tree = [&]
			{
				auto builder = ArticulationBuilder{};
				builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.2f, 1.0f)});
				return builder.Build();
			};
			auto trees = std::array{make_tree(), make_tree(), make_tree()};
			auto articulations = std::array{&trees[0], &trees[1]};
			PR_EXPECT(cache.UpdateContactLayout(bodies, articulations));
			cache.NewFrame();
			PR_EXPECT(!cache.UpdateContactLayout(bodies, articulations));
			std::ranges::reverse(articulations);
			PR_EXPECT(cache.UpdateContactLayout(bodies, articulations));
			PR_EXPECT(!cache.UpdateContactLayout(bodies, articulations));
			articulations[1] = &trees[2];
			PR_EXPECT(cache.UpdateContactLayout(bodies, articulations));
			PR_EXPECT(cache.UpdateContactLayout(bodies, std::span{articulations}.first(1)));
			PR_EXPECT(cache.UpdateContactLayout({}, articulations));

			// An empty frame or owner reset cannot bridge unrelated uses of the same packed slots.
			PR_EXPECT(cache.UpdateContactLayout({}, {}));
			PR_EXPECT(!cache.UpdateContactLayout({}, {}));
			PR_EXPECT(cache.UpdateContactLayout(bodies, articulations));
			cache.Reset();
			PR_EXPECT(cache.UpdateContactLayout(bodies, articulations));
		}
	};

	// Hardware checks for the production in-place endpoint sort, independent of append indices and segment length.
	PRUnitTestClass(CoupledContactEndpointOrderTests)
	{
		PRUnitTestMethod(CanonicalSegmentsIgnoreAppendOrder, Extended)
		{
			using namespace pr::compute;
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto signature = compute::RootSig(compute::ERootSigFlags::ComputeOnly)
				.U32<GpuCoupledContactParams>(ECBufReg::b0).SRV(ESRVReg::t0)
				.UAV(EUAVReg::u1).UAV(EUAVReg::u5).UAV(EUAVReg::u6)
				.Create(gpu, "ContactOrder:Signature");
			auto pipeline = compute::ComputePSO(signature.get(), shader_code::canonicalise_coupled_contact_endpoints).Create(gpu, "ContactOrder:Pipeline");
			auto upload = [&]<typename Container>(ID3D12Resource* resource, Container const& values)
			{
				using Value = typename Container::value_type;
				auto allocation = job.m_upload.Alloc<Value>(isize(values));
				memcpy(allocation.template ptr<Value>(), values.data(), values.size() * sizeof(Value));
				job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
			};
			for (auto count : {0, 1, 2, 3, 129})
			{
				// The last contacts share a centroid but differ in transform or active manifold data.
				auto sorted = std::vector<GpuResolveContact>{};
				for (int index = 0; index != count; ++index)
				{
					auto contact = TestClass_ResolveWarmStartIdentityTests::Contact(v4{static_cast<float>(index + 1), 0, 0, 1}, 0);
					if (index >= 124)
					{
						contact.contact_point.x = 124;
						contact.b2a[3].x = static_cast<float>(std::min(index - 123, 3));
					}
					sorted.push_back(contact);
				}
				auto counters = gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job, "ContactOrder:Counters");
				auto contacts = gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(std::max(1, count), {}).usage(EUsage::UnorderedAccess), job, "ContactOrder:Contacts");
				auto keys = gpu.CreateResource(ResDesc::Buf<uint32_t>(2 * count + 2, {}).usage(EUsage::UnorderedAccess), job, "ContactOrder:Keys");
				auto order = gpu.CreateResource(ResDesc::Buf<uint32_t>(2 * count + 2, {}).usage(EUsage::UnorderedAccess), job, "ContactOrder:Order");
				for (int permutation = 0; permutation != 3; ++permutation)
				{
					auto input = std::vector<GpuResolveContact>{};
					auto target_keys = std::vector<uint32_t>(2 * count + 2, 0xFFFFFFFFU);
					auto endpoints = std::vector<uint32_t>(2 * count + 2, 0xFFFFFFFFU);
					for (int index = 0; index != count; ++index)
					{
						auto const source = permutation == 0 ? index : permutation == 1 ? count - 1 - index : (31 * index + 7) % count;
						input.push_back(sorted[source]);
						target_keys[index] = 0;
						target_keys[count + index] = 1;
						endpoints[index] = 2 * index;
						endpoints[count + index] = 2 * index + 1;
					}
					upload(counters.get(), std::array{GpuCollisionCounters{.contact_count = count}});
					if (!input.empty())
						upload(contacts.get(), input);

					upload(keys.get(), target_keys);
					upload(order.get(), endpoints);
					job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
					job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
					job.m_barriers.Transition(keys.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
					job.m_barriers.Transition(order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
					job.m_cmd_list.SetPipelineState(pipeline.get());
					job.m_cmd_list.SetComputeRootSignature(signature.get());
					job.m_cmd_list.AddComputeRoot32BitConstants(GpuCoupledContactParams{.max_contacts = std::max(1, count), .target_count = 2});
					job.m_cmd_list.AddComputeRootShaderResourceView(counters);
					job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts);
					job.m_cmd_list.AddComputeRootUnorderedAccessView(keys);
					job.m_cmd_list.AddComputeRootUnorderedAccessView(order);
					job.m_cmd_list.Dispatch(std::max(1, (count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
					auto readback = job.m_readback.Alloc<uint32_t>(isize(endpoints));
					job.m_barriers.Transition(order.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
					job.m_cmd_list.CopyBufferRegion(readback, order.get());
					job.Run();

					// Both sides must follow the authored geometric order, including an odd-index segment leader and untouched invalid tail.
					for (int index = 0; index != 2 * count; ++index)
					{
						auto const endpoint = readback.ptr<uint32_t>()[index];
						PR_EXPECT(endpoint < static_cast<uint32_t>(2 * count));
						PR_EXPECT((endpoint & 1U) == static_cast<uint32_t>(index / count));
						auto const& actual = input[endpoint >> 1];
						auto const& expected = sorted[index % count];
						PR_EXPECT(All(actual.contact_point == expected.contact_point));
						PR_EXPECT(All(actual.b2a[3] == expected.b2a[3]));
						PR_EXPECT(All(actual.manifold[0] == expected.manifold[0]));
					}
					PR_EXPECT(readback.ptr<uint32_t>()[2 * count] == 0xFFFFFFFFU);
					PR_EXPECT(readback.ptr<uint32_t>()[2 * count + 1] == 0xFFFFFFFFU);
				}
			}
		}
	};
}
#endif
