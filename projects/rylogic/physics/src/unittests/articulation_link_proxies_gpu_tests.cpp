//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_gpu_data.h"
#include "src/compute/articulation_link_proxies_gpu.h"
#include "src/compute/articulation_mobility_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return asymmetric link properties and a non-identity shape frame so link and proxy transforms cannot be confused.
		ArticulationLinkDesc ProxyFrameLink(int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.21f + 0.01f * scale, 0.27f + 0.02f * scale, 0.34f + 0.015f * scale, 0},
					0.9f + 0.2f * scale,
					v4{0.01f * scale, -0.007f * scale, 0.005f * scale, 0}),
				.m_shape_to_link = m4x4::Transform(
					Normalise(v4{1, -2, 1, 0}),
					0.09f * scale,
					v4{0.03f * scale, -0.02f, 0.04f, 1}),
			};
		}

		// Return a mixed rotational/translational joint with non-trivial attachment frames and state.
		ArticulationJointDesc ProxyFrameJoint(int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), 0.07f * scale, v4{0.11f * scale, -0.08f, 0.05f, 1}),
				m4x4::Transform(v4::XAxis(), -0.04f * scale, v4{-0.06f, 0.03f * scale, 0.09f, 1}));
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::ZAxis()};
			joint.m_axes[1] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::YAxis()};
			joint.m_initial_position[0] = 0.13f * scale;
			joint.m_initial_position[1] = -0.06f * scale;
			joint.m_initial_velocity[0] = 0.17f - 0.02f * scale;
			joint.m_initial_velocity[1] = -0.11f + 0.01f * scale;
			return joint;
		}

		// Build one fixed or floating three-link branch with observable world transforms at every link.
		Articulation BuildProxyFrameTree(EArticulationRootType root_type, int seed)
		{
			auto builder = ArticulationBuilder{};
			auto const root_to_world = m4x4::Transform(
				Normalise(v4{1, 2, -1, 0}),
				0.19f + 0.03f * seed,
				v4{0.4f * seed, -0.25f, 0.7f, 1});
			auto root = LinkHandle{};
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					root = builder.AddFixedRoot(ProxyFrameLink(seed), root_to_world);
					break;
				}
				case EArticulationRootType::Floating:
				{
					root = builder.AddFloatingRoot(
						ProxyFrameLink(seed),
						root_to_world,
						v8motion{v4{0.1f, -0.2f, 0.3f, 0}, v4{-0.3f, 0.2f, 0.1f, 0}});
					break;
				}
				default:
				{
					throw std::invalid_argument("Proxy-frame test root type is invalid");
				}
			}

			builder.AddLink(root, ProxyFrameJoint(seed + 1), ProxyFrameLink(seed + 1));
			builder.AddLink(root, ProxyFrameJoint(seed + 2), ProxyFrameLink(seed + 2));
			return builder.Build();
		}

		// Reuse one D3D12 device and command job across persistent link-frame tests.
		Gpu& ProxyFrameTestGpu()
		{
			static auto gpu = Gpu{};
			return gpu;
		}
	}

	PRUnitTestClass(ArticulationLinkProxyGpuTests)
	{
		// Dedicated frames survive ABA scratch reuse and retain packed link order for fixed and floating trees.
		PRUnitTestMethod(HardwarePersistentFramesSurviveMobilityPreparation, Extended)
		{
			auto fixed = BuildProxyFrameTree(EArticulationRootType::Fixed, 1);
			auto floating = BuildProxyFrameTree(EArticulationRootType::Floating, 3);
			auto forest = std::array{&fixed, &floating};
			auto upload = PackGpuArticulations(forest);
			auto shape_ids = std::vector<int>(upload.m_links.size(), -1);
			auto bodies = PackGpuArticulationProxies(upload, forest, shape_ids, 0);

			// Record proxy reconstruction followed by mobility preparation, which deliberately replaces shared ABA factor scratch.
			auto& gpu = ProxyFrameTestGpu();
			auto config = EngineConfig{};
			auto integrator = GpuIntegrator{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto proxies = GpuArticulationLinkProxies{aba, config};
			auto mobility = GpuArticulationMobility{aba};
			PR_EXPECT(aba.Upload(gpu.m_job, upload));
			integrator.Upload(gpu.m_job, bodies);
			proxies.Upload(gpu.m_job);
			proxies.Refresh(gpu.m_job, integrator, 0);
			auto const participants = std::array{0, 1};
			PR_EXPECT(mobility.Upload(gpu.m_job, upload, participants));
			mobility.Run(gpu.m_job);

			// Read the dedicated resource only after the shared scratch has been repurposed.
			auto* link_to_world = proxies.LinkToWorld();
			PR_EXPECT(link_to_world != nullptr);
			gpu.m_job.m_barriers.Transition(link_to_world, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			auto readback = gpu.m_job.m_readback.Alloc<GpuConstraintFrame>(isize(upload.m_links));
			gpu.m_job.m_cmd_list.CopyBufferRegion(readback, link_to_world, 0);
			gpu.m_job.Run();

			auto const* frames = readback.ptr<GpuConstraintFrame>();
			for (int articulation_index = 0; articulation_index != isize(forest); ++articulation_index)
			{
				auto const& packed = upload.m_articulations[articulation_index];
				for (int local_link_index = 0; local_link_index != forest[articulation_index]->LinkCount(); ++local_link_index)
				{
					auto const link = forest[articulation_index]->LinkAt(local_link_index);
					auto const actual = UnpackGpuTransform(frames[packed.link_offset + local_link_index]);
					PR_EXPECT(FEqlAbsolute(actual, forest[articulation_index]->LinkToWorld(link), 2.0e-4f));
				}
			}
		}

		// World-space proxy force and torque gather into the matching link-frame ABA wrench without losing the uploaded baseline.
		PRUnitTestMethod(HardwareGatheredWrenchesMatchCpuTransform, Extended)
		{
			auto articulation = BuildProxyFrameTree(EArticulationRootType::Floating, 2);
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto shape_ids = std::vector<int>(upload.m_links.size(), -1);
			auto bodies = PackGpuArticulationProxies(upload, forest, shape_ids, 0);

			// Seed independent world-space proxy accumulators and link-frame articulation baselines across every non-identity shape frame.
			for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
			{
				auto const scale = static_cast<float>(link_index + 1);
				bodies[link_index].force_ang = v4{0.17f * scale, -0.11f * scale, 0.07f * scale, 0};
				bodies[link_index].force_lin = v4{-0.23f * scale, 0.19f * scale, 0.13f * scale, 0};
				upload.m_external_forces[link_index].force_ang = v4{0.03f * scale, 0.05f * scale, -0.02f * scale, 0};
				upload.m_external_forces[link_index].force_lin = v4{-0.04f * scale, 0.01f * scale, 0.06f * scale, 0};
			}

			// Dispatch the production gather pass and retain its per-link wrench stream for an independent CPU comparison.
			auto& gpu = ProxyFrameTestGpu();
			auto config = EngineConfig{};
			auto integrator = GpuIntegrator{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto proxies = GpuArticulationLinkProxies{aba, config};
			PR_EXPECT(aba.Upload(gpu.m_job, upload));
			integrator.Upload(gpu.m_job, bodies);
			proxies.Upload(gpu.m_job);
			auto* gathered_forces = proxies.GatherForces(gpu.m_job, integrator.Bodies().get());
			PR_EXPECT(gathered_forces != nullptr);
			gpu.m_job.m_barriers.Transition(gathered_forces, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			auto readback = gpu.m_job.m_readback.Alloc<GpuFrameForce>(isize(upload.m_links));
			gpu.m_job.m_cmd_list.CopyBufferRegion(readback, gathered_forces, 0);
			gpu.m_job.Run();

			// Reproduce the centre-of-mass torque transport and source-to-link force transform directly from packed CPU values.
			auto const* actual = readback.ptr<GpuFrameForce>();
			for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
			{
				auto const& body = bodies[link_index];
				auto const& baseline = upload.m_external_forces[link_index];
				auto const proxy_to_world = body.o2w;
				auto const world_to_proxy = InvertOrthonormal(proxy_to_world);
				auto const com_offset_ws = proxy_to_world * v4{body.os_com_and_invmass.x, body.os_com_and_invmass.y, body.os_com_and_invmass.z, 0};
				auto const torque_at_proxy_origin_ws = body.force_ang + Cross(com_offset_ws, body.force_lin);
				auto const force_proxy = world_to_proxy.rot * body.force_lin;
				auto const torque_proxy = world_to_proxy.rot * torque_at_proxy_origin_ws;
				auto const shape_to_link = UnpackGpuTransform(upload.m_links[link_index].shape_to_link);
				auto const force_link = shape_to_link.rot * force_proxy;
				auto const torque_link = shape_to_link.rot * torque_proxy + Cross(shape_to_link.pos, force_link);

				PR_EXPECT(FEqlAbsolute(actual[link_index].force_ang, baseline.force_ang + torque_link, 2.0e-5f));
				PR_EXPECT(FEqlAbsolute(actual[link_index].force_lin, baseline.force_lin + force_link, 2.0e-5f));
			}
			PR_EXPECT(proxies.Stats().m_gather_dispatch_count == 1);
		}

		// Link-frame storage is absent for an empty forest and reported exactly when proxies are active.
		PRUnitTestMethod(HardwareOptionalFrameCostIsExplicit, Extended)
		{
			auto& gpu = ProxyFrameTestGpu();
			auto config = EngineConfig{};
			auto aba = GpuArticulationForceAba{gpu};
			auto proxies = GpuArticulationLinkProxies{aba, config};

			// Construction and an empty upload retain no optional proxy resources.
			auto const empty = PackGpuArticulations({});
			PR_EXPECT(!aba.Upload(gpu.m_job, empty));
			proxies.Upload(gpu.m_job);
			PR_EXPECT(proxies.LinkToWorld() == nullptr);
			PR_EXPECT(proxies.Stats().m_external_force_capacity == 0);
			PR_EXPECT(proxies.Stats().m_link_frame_capacity == 0);
			PR_EXPECT(proxies.Stats().m_logical_bytes == 0);
			PR_EXPECT(proxies.Stats().m_allocated_feature_bytes == 0);

			// An active forest reports both the per-link wrench stream and persistent 32-byte frame stream.
			auto articulation = BuildProxyFrameTree(EArticulationRootType::Fixed, 2);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			PR_EXPECT(aba.Upload(gpu.m_job, upload));
			proxies.Upload(gpu.m_job);
			auto const& active_stats = proxies.Stats();
			auto const expected_logical_bytes = upload.m_links.size() * (sizeof(GpuFrameForce) + sizeof(GpuConstraintFrame));
			auto const expected_allocated_bytes =
				static_cast<size_t>(active_stats.m_external_force_capacity) * sizeof(GpuFrameForce) +
				static_cast<size_t>(active_stats.m_link_frame_capacity) * sizeof(GpuConstraintFrame);
			PR_EXPECT(proxies.LinkToWorld() != nullptr);
			PR_EXPECT(active_stats.m_external_force_capacity >= isize(upload.m_links));
			PR_EXPECT(active_stats.m_link_frame_capacity >= isize(upload.m_links));
			PR_EXPECT(active_stats.m_logical_bytes == expected_logical_bytes);
			PR_EXPECT(active_stats.m_allocated_feature_bytes == expected_allocated_bytes);
			gpu.m_job.Run();

			// Returning to an empty forest releases retained high-water storage rather than charging idle users.
			PR_EXPECT(!aba.Upload(gpu.m_job, empty));
			proxies.Upload(gpu.m_job);
			PR_EXPECT(proxies.LinkToWorld() == nullptr);
			PR_EXPECT(proxies.Stats().m_external_force_capacity == 0);
			PR_EXPECT(proxies.Stats().m_link_frame_capacity == 0);
			PR_EXPECT(proxies.Stats().m_logical_bytes == 0);
			PR_EXPECT(proxies.Stats().m_allocated_feature_bytes == 0);
		}
	};
}
#endif
