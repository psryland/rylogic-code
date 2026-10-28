//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/compute/gpu.h"
#include "pr/compute/gpu_job.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/radix_sort/radix_sort.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"
#include "pr/compute/utility/wrappers.h"
#include "pr/compute/utility/pix.h"
#include "pr/common/resource.h"

namespace pr::physics
{
	using ComGpu = ::pr::compute::Gpu<GpuQueueType>;
	using CmdList = ::pr::compute::CmdList<GpuQueueType>;
	using ComputeStep = ::pr::compute::ComputeStep;
	using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;
	using ReadbackAlloc = ::pr::compute::GpuReadbackBuffer::Allocation;
	using BoundsSorter = ::pr::compute::gpu_radix_sort::GpuRadixSort<float, uint32_t, true, GpuQueueType>;
	using ContactSorter = ::pr::compute::gpu_radix_sort::GpuRadixSort<float, uint32_t, true, GpuQueueType>;
	using ContactEndpointSorter = ::pr::compute::gpu_radix_sort::GpuRadixSort<uint32_t, uint32_t, true, GpuQueueType>;

	struct Gpu
	{
		ComGpu m_gpu; // A GPU wrapper for running compute shaders
		GpuJob m_job; // A GpuJob for running Engine::Step()
		
		// Create an independent compute context, or borrow a matching device and command queue owned by a longer-lived host.
		Gpu(ID3D12Device4* existing_device = nullptr, ID3D12CommandQueue* existing_queue = nullptr)
			: m_gpu(existing_device, existing_queue)
			, m_job(m_gpu, "Physics Engine Job", 0xFF00FFFF, 1)
		{}

		// Allow use as a device
		ID3D12Device4 const* operator -> () const
		{
			return m_gpu.operator ->();
		}
		ID3D12Device4* operator ->()
		{
			return m_gpu.operator ->();
		}
		operator ID3D12Device4 const* () const
		{
			return m_gpu;
		}
		operator ID3D12Device4* ()
		{
			return m_gpu;
		}

		// Allocate a DX resource. See 'compute::Gpu::CreateResource' for how initial data is staged.
		D3DPtr<ID3D12Resource> CreateResource(::pr::compute::ResDesc const& desc, GpuJob& job, std::string_view name)
		{
			return m_gpu.CreateResource(desc, job, name);
		}
	};
}
