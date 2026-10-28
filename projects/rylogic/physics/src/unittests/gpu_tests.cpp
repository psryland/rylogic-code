//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	// Check the resource lifetime guarantees of command list recording.
	PRUnitTestClass(GpuTests)
	{
		// A resource released by its owner while recorded work uses it must stay alive until the GPU has finished that work.
		PRUnitTestMethod(UsedResourceAliveUntilJobCompletes, Quick)
		{
			// Record work that uses a resource. Repeated use within one recording must not add more references.
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto desc = compute::ResDesc::Buf<uint32_t>(16, {}).usage(compute::EUsage::UnorderedAccess);
			auto res = gpu.CreateResource(desc, job, "GpuTests:Used");
			job.m_barriers.Transition(res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
			job.m_cmd_list.Use(res.get());

			// Hold a raw reference so the reference count can be read without touching a released object.
			auto* raw = res.get();
			raw->AddRef();

			// Releasing the owner's reference must leave the command list holding exactly one reference.
			res = nullptr;
			PR_EXPECT(raw->AddRef() == 3 && raw->Release() == 2);

			// 'Close' hands the reference to the allocator pool, and resetting for the next recording after the GPU wait releases it.
			job.Run();
			PR_EXPECT(raw->Release() == 0);
		}

		// Work executed by 'Flush' must precede later recorded work, and the final handle must cover both segments.
		PRUnitTestMethod(FlushExecutesRecordedPrefix, Quick)
		{
			// Two buffers in the COMMON state, so the state tracking restarted by 'Flush' matches the decayed GPU state.
			auto& gpu = SharedTestGpu();
			auto& job = gpu.m_job;
			auto desc = compute::ResDesc::Buf<uint32_t>(4, {}).def_state(D3D12_RESOURCE_STATE_COMMON);
			auto src = gpu.CreateResource(desc, job, "GpuTests:FlushSrc");
			auto dst = gpu.CreateResource(desc, job, "GpuTests:FlushDst");
			std::array<uint32_t, 4> const values = { 1, 2, 3, 0xDEADBEEF };

			// First segment: upload into 'src' and execute it without waiting.
			job.m_barriers.Transition(src.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			UploadData<uint32_t>(job, src.get(), values);
			job.Flush();

			// Second segment: copy 'src' into 'dst' using the data written by the flushed segment, then read it back.
			job.m_barriers.Transition(src.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(dst.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyResource(dst.get(), src.get());
			job.m_barriers.Transition(dst.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			auto readback = job.m_readback.Alloc<uint32_t>(static_cast<int>(values.size()));
			job.m_cmd_list.CopyBufferRegion(readback, dst.get(), 0);

			// Completing the final handle also completes the flushed segment, and flushing is invalid while a submission is pending.
			auto handle = job.Submit();
			PR_THROWS(job.Flush(), std::runtime_error);
			job.Complete(handle);
			auto const* result = readback.ptr<uint32_t>();
			for (size_t i = 0; i != values.size(); ++i)
				PR_EXPECT(result[i] == values[i]);
		}
	};
}
#endif
