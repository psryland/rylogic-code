//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/compute/frame_output_gpu.h"
#include "src/compute/physics_types.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	// Check that sparse and full-capacity events select exactly one GPU copy prefix.
	PRUnitTestClass(FrameOutputPredicationTests)
	{
		// Exercise zero events and both sides of each prefix boundary, including a partial final prefix.
		PRUnitTestMethod(SparseEventPrefixesPreserveCountsAndOrder, Quick)
		{
			// Verify data and GPU-generated predicates independently of the byte accounting.
			auto& gpu = SharedTestGpu();
			auto output = GpuFrameOutput{gpu};
			for (auto count : {0, 1, 63, 64, 65, 127, 128, 129, 255, 256, 257, 511, 512, 513, 1023, 1024, 1025, 1499, 1500})
			{
				// Place known features in the GPU event queue without changing the production append path.
				auto const capacity = 1500;
				output.BeginFrame(gpu.m_job, 0, capacity, 1);
				auto header = gpu.m_job.m_upload.Alloc<GpuFrameOutputHeader>(1);
				*header.ptr<GpuFrameOutputHeader>() = GpuFrameOutputHeader{.event_count = count, .event_capacity = capacity};
				gpu.m_job.m_barriers.Transition(output.OutputResource(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(output.OutputResource(), 0, header);
				if (count != 0)
				{
					auto events = gpu.m_job.m_upload.Alloc<GpuCollisionEvent>(count);
					for (auto index = 0; index != count; ++index)
						events.ptr<GpuCollisionEvent>()[index].feature = index + 17;

					gpu.m_job.m_cmd_list.CopyBufferRegion(output.OutputResource(), sizeof(GpuFrameOutputHeader), events);
				}
				gpu.m_job.m_barriers.Transition(output.OutputResource(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
				auto readback = output.GatherAndReadback(gpu.m_job, 0, nullptr);
				// The test-only predicate readback checks that at most one of the six candidate copies can execute.
				auto predicates = gpu.m_job.m_readback.Alloc<uint64_t>(6);
				gpu.m_job.m_barriers.Transition(output.m_r_event_copy_predicates.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(predicates, output.m_r_event_copy_predicates.get(), 0);
				gpu.m_job.Run();

				// Every event must retain its position regardless of the selected transfer size.
				PR_EXPECT(GpuFrameOutput::Header(readback).event_count == count);
				auto const copied = GpuFrameOutput::Events(readback);
				PR_EXPECT(copied.size() == capacity);
				for (auto index = 0; index != count; ++index)
				{
					if (copied[index].feature != index + 17)
						throw std::runtime_error(std::format("Predicated event readback count {}, index {}: expected feature {}, got {}", count, index, index + 17, copied[index].feature));
				}
				auto copied_capacity = count == 0 ? 0 : 64;
				while (copied_capacity < count)
					copied_capacity *= 2;
				copied_capacity = std::min(copied_capacity, capacity);
				auto prefix_capacity = 64;
				for (auto index = 0; index != 6; ++index)
				{
					// Only the smallest sufficient prefix can be enabled; zero events enable none.
					auto candidate = std::min(prefix_capacity, capacity);
					PR_EXPECT(predicates.ptr<uint64_t>()[index] == (count != 0 && copied_capacity == candidate ? 1u : 0u));
					prefix_capacity *= 2;
				}
				PR_EXPECT(GpuFrameOutput::TransferredBytes(readback) == sizeof(GpuFrameOutputHeader) + copied_capacity * sizeof(GpuCollisionEvent));
				PR_EXPECT(output.Stats().m_readback_count == 1);
			}
		}

		// Keep fixed-position diagnostic records readable when no event region was selected.
		PRUnitTestMethod(ZeroEventsStillCopyFixedDiagnostics, Quick)
		{
			auto& gpu = SharedTestGpu();
			auto output = GpuFrameOutput{gpu};
			auto states = gpu.CreateResource(pr::compute::ResDesc::Buf<GpuConstraintBreakState>(1, {}).usage(pr::compute::EUsage::UnorderedAccess), gpu.m_job.m_cmd_list, "Physics:PredicatedTestBreakState");
			auto source = gpu.m_job.m_upload.Alloc<GpuConstraintBreakState>(1);
			*source.ptr<GpuConstraintBreakState>() = GpuConstraintBreakState{.generation = 1234};
			gpu.m_job.m_barriers.Transition(states.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			gpu.m_job.m_cmd_list.CopyBufferRegion(states.get(), 0, source);
			auto breaks = GpuConstraintBreakOutput{.m_states = states.get(), .m_slot_count = 1};
			output.BeginFrame(gpu.m_job, 0, 1500, 1, GpuArticulationMidpointOutput{}, breaks, GpuCoupledConstraintFailureOutput{}, false);
			auto readback = output.GatherAndReadback(gpu.m_job, 0, nullptr, GpuArticulationMidpointOutput{}, breaks, GpuCoupledConstraintFailureOutput{});
			gpu.m_job.Run();

			PR_EXPECT(GpuFrameOutput::Header(readback).event_count == 0);
			PR_EXPECT(GpuFrameOutput::ConstraintBreaks(readback)[0].generation == 1234);
			PR_EXPECT(GpuFrameOutput::TransferredBytes(readback) == sizeof(GpuFrameOutputHeader) + sizeof(GpuConstraintBreakState));
		}

		// Retain the single-copy path for frames without an event subscription.
		PRUnitTestMethod(NoEventCapacityUsesSingleCopy, Quick)
		{
			auto& gpu = SharedTestGpu();
			auto output = GpuFrameOutput{gpu};
			output.BeginFrame(gpu.m_job, 0, 0, 1);
			auto readback = output.GatherAndReadback(gpu.m_job, 0, nullptr);
			gpu.m_job.Run();

			PR_EXPECT(GpuFrameOutput::Header(readback).event_capacity == 0);
			PR_EXPECT(output.Stats().m_dispatch_count == 0);
			PR_EXPECT(output.Stats().m_readback_count == 1);
			PR_EXPECT(GpuFrameOutput::TransferredBytes(readback) == sizeof(GpuFrameOutputHeader));
		}

		// Select one prefix across every large power-of-two boundary at the default event capacity.
		PRUnitTestMethod(LargeEventPrefixesSelectOneCopy, Quick)
		{
			// A sentinel at the last retained slot proves that the selected prefix includes its endpoint.
			auto& gpu = SharedTestGpu();
			auto output = GpuFrameOutput{gpu};
			auto const capacity = 65536;
			for (auto count : {2047, 2048, 2049, 4095, 4096, 4097, 8191, 8192, 8193, 16383, 16384, 16385, 32767, 32768, 32769, 65535, 65536})
			{
				// Only the last event needs initializing to establish copy coverage at this boundary.
				output.BeginFrame(gpu.m_job, 0, capacity, 1);
				auto header = gpu.m_job.m_upload.Alloc<GpuFrameOutputHeader>(1);
				*header.ptr<GpuFrameOutputHeader>() = GpuFrameOutputHeader{.event_count = count, .event_capacity = capacity};
				auto event = gpu.m_job.m_upload.Alloc<GpuCollisionEvent>(1);
				event.ptr<GpuCollisionEvent>()->feature = count + 17;
				gpu.m_job.m_barriers.Transition(output.OutputResource(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(output.OutputResource(), 0, header);
				gpu.m_job.m_cmd_list.CopyBufferRegion(output.OutputResource(), sizeof(GpuFrameOutputHeader) + (count - 1) * sizeof(GpuCollisionEvent), event);
				gpu.m_job.m_barriers.Transition(output.OutputResource(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
				auto readback = output.GatherAndReadback(gpu.m_job, 0, nullptr);
				auto predicates = gpu.m_job.m_readback.Alloc<uint64_t>(11);
				gpu.m_job.m_barriers.Transition(output.m_r_event_copy_predicates.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(predicates, output.m_r_event_copy_predicates.get(), 0);
				gpu.m_job.Run();

				// All eleven flags must match the one prefix that contains the final event.
				auto prefix_capacity = 64;
				for (auto index = 0; index != 11; ++index)
				{
					// Each candidate is exclusive, so a later copy cannot hide an incorrectly enabled earlier one.
					PR_EXPECT(predicates.ptr<uint64_t>()[index] == (prefix_capacity / 2 < count && count <= prefix_capacity ? 1u : 0u));
					prefix_capacity *= 2;
				}
				PR_EXPECT(GpuFrameOutput::Events(readback)[count - 1].feature == count + 17);
				// Byte accounting must agree with the single prefix selected by the GPU.
				auto selected_capacity = 64;
				while (selected_capacity < count)
					selected_capacity *= 2;

				PR_EXPECT(GpuFrameOutput::TransferredBytes(readback) == sizeof(GpuFrameOutputHeader) + selected_capacity * sizeof(GpuCollisionEvent));
			}
		}

		// Keep the largest supported configured prefix complete without relying on a smaller adaptive allocation.
		PRUnitTestMethod(FullConfiguredEventCapacityIsCopied, Extended)
		{
			// Fill the largest configured event queue and inspect the GPU's selection of its final prefix.
			auto& gpu = SharedTestGpu();
			auto output = GpuFrameOutput{gpu};
			auto const capacity = 65536;
			output.BeginFrame(gpu.m_job, 0, capacity, 1);
			auto header = gpu.m_job.m_upload.Alloc<GpuFrameOutputHeader>(1);
			*header.ptr<GpuFrameOutputHeader>() = GpuFrameOutputHeader{.event_count = capacity, .event_capacity = capacity};
			auto events = gpu.m_job.m_upload.Alloc<GpuCollisionEvent>(capacity);
			for (auto index = 0; index != capacity; ++index)
				events.ptr<GpuCollisionEvent>()[index].feature = index + 17;

			gpu.m_job.m_barriers.Transition(output.OutputResource(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			gpu.m_job.m_cmd_list.CopyBufferRegion(output.OutputResource(), 0, header);
			gpu.m_job.m_cmd_list.CopyBufferRegion(output.OutputResource(), sizeof(GpuFrameOutputHeader), events);
			gpu.m_job.m_barriers.Transition(output.OutputResource(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			auto readback = output.GatherAndReadback(gpu.m_job, 0, nullptr);
			auto predicates = gpu.m_job.m_readback.Alloc<uint64_t>(11);
			gpu.m_job.m_barriers.Transition(output.m_r_event_copy_predicates.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			gpu.m_job.m_cmd_list.CopyBufferRegion(predicates, output.m_r_event_copy_predicates.get(), 0);
			gpu.m_job.Run();

			// The first ten prefixes must be skipped; only the full-capacity copy can execute.
			auto const copied = GpuFrameOutput::Events(readback);
			PR_EXPECT(GpuFrameOutput::Header(readback).event_count == capacity);
			PR_EXPECT(copied.front().feature == 17);
			PR_EXPECT(copied[capacity / 2].feature == capacity / 2 + 17);
			PR_EXPECT(copied.back().feature == capacity - 1 + 17);
			PR_EXPECT(GpuFrameOutput::TransferredBytes(readback) == sizeof(GpuFrameOutputHeader) + capacity * sizeof(GpuCollisionEvent));
			for (auto index = 0; index != 11; ++index)
				PR_EXPECT(predicates.ptr<uint64_t>()[index] == (index == 10 ? 1u : 0u));
		}
	};
}
#endif
