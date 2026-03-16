//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/compute/gpu.h"
#include "pr/view3d-12/compute/gpu_job.h"
#include "pr/view3d-12/compute/compute_pso.h"
#include "pr/view3d-12/compute/compute_step.h"
#include "pr/view3d-12/compute/particle_collision/collision_builder.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_include_handler.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/pix.h"

namespace pr::rdr12::compute::particle_collision
{
	template <D3D12_COMMAND_LIST_TYPE QueueType = D3D12_COMMAND_LIST_TYPE_DIRECT>
	struct ParticleCollision
	{
		// Notes:
		//  - Supports 2D or 3D particles
		//  - Supports Euler or Verlet integration
		// 
		// TODO:
		//  - ReadPrimitives/WritePrimitives methods for updating collision
		//  - Support primitives with dynamics (i.e. moving, with mass, accumulate force from particles)
		//  - Spatially partition collision
		using Gpu = Gpu<QueueType>;

		enum class ECullMode :int
		{
			None,
			Plane,
			Sphere,
			SphereInside,
			Box,
			BoxInside,
			MASK = 0x7,
		};

		struct CullData
		{
			v4 Geom[2] = { v4::Zero(), v4::Zero() }; // The culling volume
			ECullMode Mode = ECullMode::None;        // The culling mode
		};
		struct ConfigData
		{
			int NumPrimitives = 0;           // The number of primitives to sort
			int SpatialDimensions = 3;       // The number of spatial dimensions
			v2 Restitution = { 1.0f, 1.0f }; // The coefficient of restitution (normal, tangential)
			CullData Culling;
		};
		struct Setup
		{
			int PrimitiveCapacity;                        // The maximum number of primitives
			ConfigData Config;                            // Runtime configuration for the particle collision
			std::span<Prim const> CollisionInitData = {}; // Initialisation data for the collision
			
			bool valid() const noexcept
			{
				return
					Config.NumPrimitives >= 0 && Config.NumPrimitives <= PrimitiveCapacity &&
					(CollisionInitData.empty() || isize(CollisionInitData) == Config.NumPrimitives);
			}
		};

		Gpu* m_gpu;                            // The gpu instance to use to run the compute shader
		ComputeStep m_cs_integrate;            // Integrate particles forward in time (with collision)
		ComputeStep m_cs_boundaries;           // Detect proximity to boundaries for each particle
		ComputeStep m_cs_culldead;             // Mark culled particles with NaN positions
		D3DPtr<ID3D12Resource> m_r_primitives; // The primitives to collide with
		int m_capacity;                        // The maximum space in the buffers

		// Runtime collision config
		ConfigData Config;
		
		ParticleCollision(Gpu& gpu, std::wstring_view position_layout, std::wstring_view dynamics_layout)
			: m_gpu(&gpu)
			, m_cs_integrate()
			, m_cs_boundaries()
			, m_cs_culldead()
			, m_r_primitives()
			, m_capacity()
			, Config()
		{
			CreateComputeSteps(position_layout, dynamics_layout);
		}

		// (Re)Initialise the particle collision system
		void Init(GraphicsJob& job, Setup const& setup)
		{
			assert(setup.valid());

			// Save the config
			Config = setup.Config;

			// Create the primitives buffer
			{
				ResDesc desc = ResDesc::Buf<Prim>(setup.PrimitiveCapacity, setup.CollisionInitData).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).usage(EUsage::UnorderedAccess);
				m_r_primitives = m_gpu->CreateResource(desc, job.m_cmd_list, "ParticleCollision:Primitives");
				m_capacity = setup.PrimitiveCapacity;
			}
		}

		// Integrate the particle positions (with collision)
		void Integrate(GraphicsJob& job, float dt, int count, float radius, D3DPtr<ID3D12Resource> particles, D3DPtr<ID3D12Resource> dynamics)
		{
			if (count == 0)
				return;

			pix::BeginEvent(job.m_cmd_list.get(), 0xFF209932, "ParticleCollision::Integrate");
			DoIntegrate(job, dt, count, radius, particles, dynamics);
			pix::EndEvent(job.m_cmd_list.get());
		}

		// Find nearby surfaces for particles
		void DetectBoundaries(GraphicsJob& job, int count, float radius, D3DPtr<ID3D12Resource> particles, D3DPtr<ID3D12Resource> dynamics)
		{
			if (count == 0)
				return;

			pix::BeginEvent(job.m_cmd_list.get(), 0xFF209932, "ParticleCollision::DetectBoundaries");
			DoDetectBoundaries(job, count, radius, particles, dynamics);
			pix::EndEvent(job.m_cmd_list.get());
		}

		// Mark culled particles with NaN positions
		void CullDeadParticles(GraphicsJob& job, int count, D3DPtr<ID3D12Resource> particles)
		{
			if (count == 0)
				return;

			pix::BeginEvent(job.m_cmd_list.get(), 0xFF209932, "ParticleCollision::CullDeadParticles");
			DoCullDeadParticles(job, count, particles);
			pix::EndEvent(job.m_cmd_list.get());
		}

	private:

		inline static constexpr int ThreadGroupSize = 1024;
		using float2 = pr::v2;

		struct EReg
		{
			inline static constexpr ECBufReg Sim = ECBufReg::b0;
			inline static constexpr ECBufReg Bound = ECBufReg::b0;
			inline static constexpr ECBufReg Cull = ECBufReg::b0;
			inline static constexpr EUAVReg Particles = EUAVReg::u0;
			inline static constexpr EUAVReg Dynamics = EUAVReg::u1;
			inline static constexpr ESRVReg Primitives = ESRVReg::t0;
		};
		struct cbCollision
		{
			int NumParticles;        // The number of particles
			int NumPrimitives;       // The number of primitives
			int SpatialDimensions;   // The number of spatial dimension
			float TimeStep;          // The time to advance each particle by

			float ParticleRadius;    // The radius of volume that each particle represents
			float pad;

			float2 Restitution;      // The coefficient of restitution (normal, tangential)
		};
		struct cbBoundary
		{
			int NumParticles;      // The number of particles
			int NumPrimitives;     // The number of primitives
			int SpatialDimensions; // The number of spatial dimensions
			float ParticleRadius;  // The radius of volume that each particle represents
		};
		struct cbCull
		{
			v4 Geom[2];       // A plane, sphere, etc used to cull particles (set their positions to nan)
			int Flags;        // [3:0] = 0: No culling, 1: Sphere, 2: Plane, 3: Box
			int NumParticles; // The number of particles to test
		};

		// Create constant buffer data for the collision parameters
		cbCollision CollisionCBuf(float dt, int count, float radius)
		{
			assert(Config.NumPrimitives >= 0 && Config.NumPrimitives <= m_capacity);
			return cbCollision {
				.NumParticles = count,
				.NumPrimitives = Config.NumPrimitives,
				.SpatialDimensions = Config.SpatialDimensions,
				.TimeStep = dt,
				.ParticleRadius = radius,
				.pad = 0,
				.Restitution = Config.Restitution,
			};
		}
		cbBoundary BoundaryCBuf(int count, float radius)
		{
			return cbBoundary{
				.NumParticles = count,
				.NumPrimitives = Config.NumPrimitives,
				.SpatialDimensions = Config.SpatialDimensions,
				.ParticleRadius = radius,
			};
		}
		cbCull CullCBuf(int count)
		{
			return cbCull{
				.Geom = { Config.Culling.Geom[0], Config.Culling.Geom[1] },
				.Flags = static_cast<int>(Config.Culling.Mode) & (int)ECullMode::MASK,
				.NumParticles = count,
			};
		}

		// Create the compute steps
		void CreateComputeSteps(std::wstring_view position_layout, std::wstring_view dynamics_layout)
		{
			ShaderCompiler compiler = ShaderCompiler{}
				.Source(resource::Read<char>(L"PARTICLE_COLLISION_HLSL", L"TEXT"))
				.Includes({ new ResourceIncludeHandler, true })
				.Define(L"POSITION_TYPE", position_layout)
				.Define(L"DYNAMICS_TYPE", dynamics_layout)
				.ShaderModel(L"cs_6_6")
				.Optimise();

			// Integrate
			{
				auto bytecode = compiler.EntryPoint(L"Integrate").Compile();
				m_cs_integrate.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbCollision>(EReg::Sim)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.SRV(EReg::Primitives)
					.Create(*m_gpu, "ParticleCollision:IntegrateSig");
				m_cs_integrate.m_pso = ComputePSO(m_cs_integrate.m_sig.get(), bytecode)
					.Create(*m_gpu, "ParticleCollision:IntegratePSO");
			}

			// Boundaries
			{
				auto bytecode = compiler.EntryPoint(L"DetectBoundaries").Compile();
				m_cs_boundaries.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbBoundary>(EReg::Bound)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.SRV(EReg::Primitives)
					.Create(*m_gpu, "ParticleCollision:BoundariesSig");
				m_cs_boundaries.m_pso = ComputePSO(m_cs_boundaries.m_sig.get(), bytecode)
					.Create(*m_gpu, "ParticleCollision:BoundariesPSO");
			}

			// Cull dead particles
			{
				auto bytecode = compiler.EntryPoint(L"CullDeadParticles").Compile();
				m_cs_culldead.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbCull>(EReg::Cull)
					.UAV(EReg::Particles)
					.Create(*m_gpu, "ParticleCollision:CullDeadSig");
				m_cs_culldead.m_pso = ComputePSO(m_cs_culldead.m_sig.get(), bytecode)
					.Create(*m_gpu, "ParticleCollision:CullDeadPSO");
			}
		}

		// Integrate the particle positions (with collision)
		void DoIntegrate(GraphicsJob& job, float dt, int count, float radius, D3DPtr<ID3D12Resource> positions, D3DPtr<ID3D12Resource> dynamics)
		{
			auto cb_sim = CollisionCBuf(dt, count, radius);

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_integrate.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_integrate.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sim, 0);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(positions->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_primitives->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_sim.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));

			job.m_barriers.UAV(positions.get());
			job.m_barriers.UAV(dynamics.get());
		}

		// Apply resting contact forces
		void DoDetectBoundaries(GraphicsJob& job, int count, float radius, D3DPtr<ID3D12Resource> positions, D3DPtr<ID3D12Resource> dynamics)
		{
			auto cb_bound = BoundaryCBuf(count, radius);

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_boundaries.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_boundaries.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_bound, 0);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(positions->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_primitives->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_bound.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));

			job.m_barriers.UAV(dynamics.get());
		}

		// Mark culled particles with NaN positions
		void DoCullDeadParticles(GraphicsJob& job, int count, D3DPtr<ID3D12Resource> positions)
		{
			auto cb_cull = CullCBuf(count);

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_culldead.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_culldead.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_cull, 0);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(positions->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_cull.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));

			job.m_barriers.UAV(positions.get());
		}
	};
}
