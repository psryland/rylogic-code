//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/view3d-12/compute/gpu.h"
#include "pr/view3d-12/compute/gpu_job.h"
#include "pr/view3d-12/compute/compute_pso.h"
#include "pr/view3d-12/compute/compute_step.h"
#include "pr/view3d-12/compute/radix_sort/radix_sort.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_include_handler.h"
#include "pr/view3d-12/shaders/shader_registers.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/common/resource.h"

namespace pr::physics
{
	using ComGpu = rdr12::Gpu<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
	using GpuJob = rdr12::GpuJob<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
	using CmdList = rdr12::CmdList<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
	using ComputeStep = rdr12::ComputeStep;
	using GpuUploadBuffer = rdr12::GpuUploadBuffer;
	using BoundsSorter = rdr12::compute::gpu_radix_sort::GpuRadixSort<float, uint32_t, true, D3D12_COMMAND_LIST_TYPE_COMPUTE>;

	struct Gpu
	{
		ComGpu m_gpu; // A GPU wrapper for running compute shaders
		GpuJob m_job; // A GpuJob for running Engine::Step()
		
		Gpu(ID3D12Device4* existing_device = nullptr)
			: m_gpu(existing_device)
			, m_job(m_gpu, "Physics Engine Job", 0xFF00FFFF, 1)
		{}

		// Implicit conversion to the underlying D3D12 device for convenience.
		operator ID3D12Device4 const* () const
		{
			return m_gpu;
		}
		operator ID3D12Device4* ()
		{
			return m_gpu;
		}
		
		// Access the GPU upload buffer
		GpuUploadBuffer& UploadBuffer()
		{
			return m_gpu.UploadBuffer();
		}

		// Allocate a DX resource
		D3DPtr<ID3D12Resource> CreateResource(rdr12::ResDesc const& desc, rdr12::ComCmdList& cmd_list, std::string_view name)
		{
			return m_gpu.CreateResource(desc, cmd_list, name);
		}
	};
}