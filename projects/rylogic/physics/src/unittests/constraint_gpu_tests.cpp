//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics::tests
{
	PRUnitTestClass(ConstraintGpuPackingTests)
	{
		// Preserve stable slots, canonical axis order, endpoint remapping, and compact transfer-layout contracts.
		PRUnitTestMethod(PacksSparsePersistentDescriptors, Quick)
		{
			PR_EXPECT(sizeof(GpuD6ConstraintDesc) == 256);
			PR_EXPECT(sizeof(GpuConstraintEndpoint) == 32);
			PR_EXPECT(sizeof(GpuConstraintBlock) == 32);
			PR_EXPECT(sizeof(GpuConstraintRow) == 96);

			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
			desc.m_frame_a.m_constraint_to_body = m4x4::Transform(v4::ZAxis(), 0.25f, v4{1, 2, 3, 1});
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.m_linear[0].m_target_position = 0.5f;
			desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
			desc.m_angular[2].m_target_velocity = 2.0f;
			desc.m_angular[2].m_max_force = 7.0f;
			desc.m_collide_connected = true;

			auto constraints = ConstraintSet{};
			auto const removed = constraints.Add(desc);
			auto const live = constraints.Add(desc);
			constraints.Remove(removed);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_b, &body_a};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_active_count == 1);
			PR_EXPECT(upload.m_endpoints.size() == 2);
			PR_EXPECT(upload.m_descriptors.size() == 2);
			PR_EXPECT(upload.m_endpoints[removed.m_index].flags == GpuConstraintEndpointFlags_None);
			PR_EXPECT(upload.m_endpoints[removed.m_index].generation != removed.m_generation);
			PR_EXPECT(upload.m_endpoints[live.m_index].body_idx_a == 1);
			PR_EXPECT(upload.m_endpoints[live.m_index].body_idx_b == 0);
			PR_EXPECT(AllSet(upload.m_endpoints[live.m_index].flags, GpuConstraintEndpointFlags_Enabled));
			PR_EXPECT(AllSet(upload.m_endpoints[live.m_index].flags, GpuConstraintEndpointFlags_CollideConnected));
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[0].mode == GpuConstraintAxisMode_Locked);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[0].target_position == 0.5f);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].mode == GpuConstraintAxisMode_Driven);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].target_velocity == 2.0f);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].max_force == 7.0f);
		}

		// Disabled descriptors retain their stable slot without requiring missing endpoint bodies in the submitted frame.
		PRUnitTestMethod(DisabledDescriptorsDoNotResolveEndpoints, Quick)
		{
			auto missing_body = RigidBody{};
			auto submitted_body = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(missing_body);
			desc.m_frame_b.m_body = BodyRef::Rigid(submitted_body);
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.m_enabled = false;

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&submitted_body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_active_count == 0);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_a == -1);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_b == -1);
			PR_EXPECT(!AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
		}
	};
}
#endif
