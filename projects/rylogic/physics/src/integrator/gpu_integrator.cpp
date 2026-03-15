//*********************************************
// Physics Engine — GPU Integration Implementation
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "pr/physics/rigid_body/rigid_body_dynamics.h"
#include "src/integrator/gpu_integrator.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Thread group size matching the HLSL [numthreads(64, 1, 1)] declaration.
	static constexpr int ThreadGroupSize = 64;

	// Integrate constants
	struct alignas(16) cbIntegrate
	{
		float dt;
		int body_count;
		int pad0;
		int pad1;
	};
	static_assert(sizeof(cbIntegrate) == 16);

	// Register assignments for the root signature
	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto Bodies = EUAVReg::u0;
		inline static constexpr auto Output = EUAVReg::u1;
		inline static constexpr auto AABB_X = EUAVReg::u2;
		inline static constexpr auto AABB_Y = EUAVReg::u3;
		inline static constexpr auto AABB_Z = EUAVReg::u4;
		inline static constexpr auto AABB_Idx = EUAVReg::u5;
	};

	GpuIntegrator::GpuIntegrator(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_integrate()
		, m_r_bodies()
		, m_r_output()
		, m_r_aabb_x()
		, m_r_aabb_y()
		, m_r_aabb_z()
		, m_r_aabb_idx()
		, m_capacity()
	{
		CompileShaders();
	}

	// Compile the integration compute shader
	void GpuIntegrator::CompileShaders()
	{
		// Load the HLSL source from the embedded resource.
		// The resource is embedded by the consuming executable's .rc file.
		auto shader_source = resource::Read<char>(L"INTEGRATE_HLSL", L"TEXT");
		auto compiler = ShaderCompiler{}
			.Source(shader_source)
			.Includes({ new ResourceIncludeHandler, true })
			.EntryPoint(L"CSIntegrate")
			.ShaderModel(L"cs_6_0")
			.Optimise();

		auto bytecode = compiler.Compile();

		// Root signature: root constants (cbIntegrate) + three UAVs (bodies, output, aabbs)
		m_cs_integrate.m_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbIntegrate>(EReg::Params)
			.UAV(EReg::Bodies)
			.UAV(EReg::Output)
			.UAV(EReg::AABB_X)
			.UAV(EReg::AABB_Y)
			.UAV(EReg::AABB_Z)
			.UAV(EReg::AABB_Idx)
			.Create(m_gpu, "Physics:IntegrateSig");

		m_cs_integrate.m_pso = ComputePSO(m_cs_integrate.m_sig.get(), bytecode)
			.Create(m_gpu, "Physics:IntegratePSO");
	}

	// Resize the buffers to hold 'capacity' bodies.
	void GpuIntegrator::ResizeBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_bodies == nullptr || m_capacity < capacity)
		{
			m_r_bodies = m_gpu.CreateResource(ResDesc::Buf<RigidBodyDynamics>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:BodyDynamics");
			m_r_output = m_gpu.CreateResource(ResDesc::Buf<IntegrateDebugOutput>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateDebugOutput");
			m_r_aabb_x = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_X");
			m_r_aabb_y = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Y");
			m_r_aabb_z = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Z");
			m_r_aabb_idx = m_gpu.CreateResource(ResDesc::Buf<int>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Idx");
			m_capacity = capacity;
		}
	}

	// Integrate bodies on GPU
	void GpuIntegrator::Integrate(GpuJob& job, std::span<RigidBodyDynamics> dynamics, float dt)
	{
		// This function runs the integration compute shader, but leaves the results on the GPU
		// for later shaders to make use of. In particular, the broadphase shader needs the aabb_x/y/z
		// and aabb_idx values.

		auto body_count = static_cast<int>(dynamics.size());
		if (body_count == 0)
			return;

		// Ensure the buffers are large enough to hold the body count.
		ResizeBuffers(job.m_cmd_list, body_count);

		// Upload body dynamics to the GPU
		{
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.template Alloc<RigidBodyDynamics>(body_count);
			memcpy(upload.template ptr<RigidBodyDynamics>(), dynamics.data(), body_count * sizeof(RigidBodyDynamics));
			job.m_cmd_list.CopyBufferRegion(m_r_bodies.get(), 0, upload);

			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Dispatch the compute shader
		{
			auto cb = cbIntegrate{ .dt = dt, .body_count = body_count };

			job.m_cmd_list.SetPipelineState(m_cs_integrate.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_integrate.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_x->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_y->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_z->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_idx->GetGPUVirtualAddress());

			auto dispatch_count = (body_count + ThreadGroupSize - 1) / ThreadGroupSize;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(m_r_bodies.get());
			job.m_barriers.UAV(m_r_output.get());
			job.m_barriers.UAV(m_r_aabb_x.get());
			job.m_barriers.UAV(m_r_aabb_y.get());
			job.m_barriers.UAV(m_r_aabb_z.get());
			job.m_barriers.UAV(m_r_aabb_idx.get());
		}

		// Leave output data on the GPU for the next step.
	}

	// Readback data into the provided buffers. 0-length means "don't readback".
	void GpuIntegrator::Readback(GpuJob& job, std::span<RigidBodyDynamics> dynamics, std::span<IntegrateDebugOutput> output, std::span<BBox> aabbs)
	{
		if (!dynamics.empty())
		{
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		}
		if (!output.empty())
		{
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		}
		if (!aabbs.empty())
		{
			job.m_barriers.Transition(m_r_aabb_x.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_aabb_y.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_aabb_z.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		}
		
		job.m_barriers.Commit();
	
		GpuReadbackBuffer::Allocation readback_dynamics;
		GpuReadbackBuffer::Allocation readback_output;
		GpuReadbackBuffer::Allocation readback_aabb_x;
		GpuReadbackBuffer::Allocation readback_aabb_y;
		GpuReadbackBuffer::Allocation readback_aabb_z;

		if (!dynamics.empty())
		{
			readback_dynamics = job.m_readback.template Alloc<RigidBodyDynamics>(int(dynamics.size()));
			job.m_cmd_list.CopyBufferRegion(readback_dynamics, m_r_output.get(), 0);
		}
		if (!output.empty())
		{
			readback_output = job.m_readback.template Alloc<IntegrateDebugOutput>(int(output.size()));
			job.m_cmd_list.CopyBufferRegion(readback_output, m_r_output.get(), 0);
		}
		if (!aabbs.empty())
		{
			readback_aabb_x = job.m_readback.template Alloc<float>(int(2 * aabbs.size()));
			readback_aabb_y = job.m_readback.template Alloc<float>(int(2 * aabbs.size()));
			readback_aabb_z = job.m_readback.template Alloc<float>(int(2 * aabbs.size()));
			job.m_cmd_list.CopyBufferRegion(readback_aabb_x, m_r_aabb_x.get(), 0);
			job.m_cmd_list.CopyBufferRegion(readback_aabb_y, m_r_aabb_y.get(), 0);
			job.m_cmd_list.CopyBufferRegion(readback_aabb_z, m_r_aabb_z.get(), 0);
		}

		if (!dynamics.empty())
		{
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}
		if (!output.empty())
		{
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}
		if (!aabbs.empty())
		{
			job.m_barriers.Transition(m_r_aabb_x.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_y.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_z.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}

		job.Run();

		if (!dynamics.empty())
		{
			memcpy(dynamics.data(), readback_dynamics.template ptr<RigidBodyDynamics>(), dynamics.size() * sizeof(RigidBodyDynamics));
		}
		if (!output.empty())
		{
			memcpy(output.data(), readback_output.template ptr<IntegrateDebugOutput>(), output.size() * sizeof(IntegrateDebugOutput));
		}
		if (!aabbs.empty())
		{
			auto aabb_x = std::span{readback_aabb_x.template ptr<float>(), aabbs.size()};
			auto aabb_y = std::span{readback_aabb_y.template ptr<float>(), aabbs.size()};
			auto aabb_z = std::span{readback_aabb_z.template ptr<float>(), aabbs.size()};

			for (int i = 0; i != aabbs.size(); ++i)
			{
				auto lower = v4{ aabb_x[i * 2 + 0], aabb_y[i * 2 + 0], aabb_z[i * 2 + 0], 1 };
				auto upper = v4{ aabb_x[i * 2 + 1], aabb_y[i * 2 + 1], aabb_z[i * 2 + 1], 1 };
				aabbs[i] = BBox::Make(lower, upper);
			}
		}
	}

	// Custom deleter implementation (GpuIntegrator is complete here)
	void Deleter<GpuIntegrator>::operator()(GpuIntegrator* p) const
	{
		delete p;
	}
}
