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
#include "pr/view3d-12/compute/radix_sort/radix_sort.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_include_handler.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/pix.h"

namespace pr::rdr12::compute::spatial_partition
{
	inline static constexpr uint32_t FNV_offset_basis32 = 2166136261U;
	inline static constexpr uint32_t FNV_prime32 = 16777619U;

	// Convert a floating point position into a grid cell coordinate
	inline iv3 GridCell(v4 position, float grid_scale)
	{
		return To<iv3>(Ceil(position.xyz * grid_scale));
	}

	// Accumulative hash function
	inline uint32_t Hash(int value, uint32_t hash = FNV_offset_basis32)
	{
		return hash = (value + hash) * FNV_prime32;
	}

	// Generate a hash from a grid cell coordinate.
	inline uint32_t CellHash(iv3 grid, int cell_count)
	{
		auto h1 = Hash(grid.x);
		auto h2 = Hash(grid.y);
		auto h3 = Hash(grid.z);
		constexpr uint32_t prime1 = 73856093;
		constexpr uint32_t prime2 = 19349663;
		constexpr uint32_t prime3 = 83492791;

		// The last cell is reserved for 'nan' positions
		return (h1 * prime1 + h2 * prime2 + h3 * prime3) % (cell_count - 1);
	}

	// Grid-based Spatial Partitioning
	template <D3D12_COMMAND_LIST_TYPE QueueType = D3D12_COMMAND_LIST_TYPE_DIRECT>
	struct SpatialPartition
	{
		// Notes:
		//  - Although this is a "grid" it actually doesn't matter what the grid dimensions are.
		//    Really, it's just hashing positions to a 1D array.
		//  - The 'm_positions' buffer is expected to be provided by the caller. The control its
		//    layout and inform this type by providing a 'position_layout' string. This string should
		//    have this form: "struct PosType { float4 _dummy; float4 pos; float4 _dummy2; }". A field
		//    called 'pos' must exist and be a float4.

		using Gpu = Gpu<QueueType>;
		using GpuRadixSorter = compute::gpu_radix_sort::GpuRadixSort<uint32_t, uint32_t, QueueType>;
		using Cell = struct { int32_t start; int32_t count; };

		struct ConfigData
		{
			// The number of cells in the grid.
			// The last cell is reserved for NaN positions.
			// Primes + 1 are a good choice: 1021+1, 65521+1, 1048573+1, 16777213+1
			int CellCount = 1021 + 1;

			// Scale positions to grid cells. E.g. scale = 10, then 0.1 -> 1, 0.2 -> 2, etc
			float GridScale = 1.0f;
		};
		struct StepOutput
		{
			GpuReadbackBuffer::Allocation m_lookup;
			GpuReadbackBuffer::Allocation m_idx_start;
			GpuReadbackBuffer::Allocation m_idx_count;
			float m_grid_scale;
			int m_cell_count;
			int m_pos_count;
		};
		struct Setup
		{
			int Capacity;
			ConfigData Config;                  // Runtime configuration for the spatial partitioning
		
			bool valid() const noexcept
			{
				return Config.CellCount >= 0 && Config.GridScale > 0;
			}
		};

		Gpu* m_gpu;                         // The gpu instance to use to run the compute shader
		ComputeStep m_init;                 // Reset buffers
		ComputeStep m_populate;             // Populate the grid cells
		ComputeStep m_build;                // Build the lookup data structure
		D3DPtr<ID3D12Resource> m_grid_hash; // The cell hash for each position
		D3DPtr<ID3D12Resource> m_spatial;   // The spatially sorted position indices
		D3DPtr<ID3D12Resource> m_idx_start; // The smallest index for each cell hash value
		D3DPtr<ID3D12Resource> m_idx_count; // The number of particles in each cell
		GpuRadixSorter m_sorter;            // Sort the cell hashes on the GPU
		int m_size;                         // The size that the resources are set up for

		// Partitioning params
		ConfigData Config;
		StepOutput Output;

		SpatialPartition(Gpu& gpu, std::wstring_view position_layout)
			: m_gpu(&gpu)
			, m_init()
			, m_populate()
			, m_build()
			, m_grid_hash()
			, m_spatial()
			, m_idx_start()
			, m_idx_count()
			, m_sorter(gpu)
			, m_size()
			, Config()
			, Output()
		{
			CreateComputeSteps(position_layout);
		}

		// (Re)Initialise the spatial partitioning
		void Init(GraphicsJob& job, Setup const& setup)
		{
			assert(setup.valid());

			Config = setup.Config;

			CreateResourceBuffers(job);
			Resize(job, setup.Capacity);
		}

		// Spatially partition the particles for faster locality testing
		void Update(GraphicsJob& job, int count, D3DPtr<ID3D12Resource> positions, bool readback)
		{
			Output = {};
			if (count == 0)
				return;
			
			pix::BeginEvent(job.m_cmd_list.get(), 0xFFB36529, "SpatialPartition::Update");
			DoUpdate(job, count, positions, readback);
			pix::EndEvent(job.m_cmd_list.get());
		}

	private:

		inline static constexpr int ThreadGroupSize = 1024;

		struct EReg
		{
			inline static constexpr ECBufReg Constants = ECBufReg::b0;
			inline static constexpr ESRVReg Positions = ESRVReg::t0;
			inline static constexpr EUAVReg GridHash = EUAVReg::u0;
			inline static constexpr EUAVReg Spatial = EUAVReg::u1;
			inline static constexpr EUAVReg IdxStart = EUAVReg::u2;
			inline static constexpr EUAVReg IdxCount = EUAVReg::u3;
		};
		struct cbGridPartition
		{
			int NumPositions;
			int CellCount;
			float GridScale;
			int pad;
		};

		// Create constant buffer data for the grid partition parameters
		cbGridPartition GridPartitionCBuf(int count)
		{
			return cbGridPartition{
				.NumPositions = count,
				.CellCount = Config.CellCount,
				.GridScale = Config.GridScale,
				.pad = 0,
			};
		}

		// Compute the compute shaders
		void CreateComputeSteps(std::wstring_view position_layout)
		{
			ShaderCompiler compiler = ShaderCompiler{}
				.Source(resource::Read<char>(L"SPATIAL_PARTITION_HLSL", L"TEXT"))
				.Includes({ new ResourceIncludeHandler, true })
				.Define(L"POSITION_TYPE", position_layout)
				.ShaderModel(L"cs_6_6")
				.Optimise();

			// Init
			{
				auto bytecode = compiler.EntryPoint(L"Init").Compile();
				m_init.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbGridPartition>(EReg::Constants)
					.UAV(EReg::IdxStart)
					.UAV(EReg::IdxCount)
					.Create(*m_gpu, "SpatialPartition:InitSig");
				m_init.m_pso = ComputePSO(m_init.m_sig.get(), bytecode)
					.Create(*m_gpu, "SpatialPartition:InitPSO");
			}

			// Populate
			{
				auto bytecode = compiler.EntryPoint(L"CalculateHashes").Compile();
				m_populate.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbGridPartition>(EReg::Constants)
					.SRV(EReg::Positions)
					.UAV(EReg::GridHash)
					.UAV(EReg::Spatial)
					.Create(*m_gpu, "SpatialPartition:CalculateHashesSig");
				m_populate.m_pso = ComputePSO(m_populate.m_sig.get(), bytecode)
					.Create(*m_gpu, "SpatialPartition:CalculateHashesPSO");
			}

			// Build lookup
			{
				auto bytecode = compiler.EntryPoint(L"BuildLookup").Compile();
				m_build.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbGridPartition>(EReg::Constants)
					.UAV(EReg::GridHash)
					.UAV(EReg::IdxStart)
					.UAV(EReg::IdxCount)
					.Create(*m_gpu, "SpatialPartition:BuildLookupSig");
				m_build.m_pso = ComputePSO(m_build.m_sig.get(), bytecode)
					.Create(*m_gpu, "SpatialPartition:BuildLookupPSO");
			}
		}

		// Create resource buffers
		void CreateResourceBuffers(GraphicsJob& job)
		{
			ResDesc desc = ResDesc::Buf<uint32_t>(Config.CellCount, {}).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).usage(EUsage::UnorderedAccess);
			m_idx_start = m_gpu->CreateResource(desc, job.m_cmd_list, "SpatialPartition:IdxStart");
			m_idx_count = m_gpu->CreateResource(desc, job.m_cmd_list, "SpatialPartition:IdxCount");
		}

		// Ensure the buffers are large enough to partition 'size' positions
		void Resize(GraphicsJob& job, int size)
		{
			if (size <= m_size)
				return;

			// Grid hash
			{
				ResDesc desc = ResDesc::Buf<uint32_t>(size, {}).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).usage(EUsage::UnorderedAccess);
				m_grid_hash = m_gpu->CreateResource(desc, job.m_cmd_list, "SpatialPartition:GridHash");
			}

			// Position indices
			{
				ResDesc desc = ResDesc::Buf<uint32_t>(size, {}).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).usage(EUsage::UnorderedAccess);
				m_spatial = m_gpu->CreateResource(desc, job.m_cmd_list, "SpatialPartition:Spatial");
			}

			// Resize the sorter
			{
				// Point the sort and payload buffers of the sorter to our grid-hash and pos-index
				// buffer so that we don't need to copy data from 'm_grid_hash' to 'm_sort[0]' etc.
				m_sorter.Bind(job.m_cmd_list, size, m_grid_hash, m_spatial);
			}

			m_size = size;
		}

		// Spatially partition the particles for faster locality testing
		void DoUpdate(GraphicsJob& job, int count, D3DPtr<ID3D12Resource> positions, bool readback)
		{
			auto cb_params = GridPartitionCBuf(count);
			auto positions_state0 = job.m_cmd_list.ResState(positions.get()).Mip0State();

			// Ensure the buffer sizes are correct
			Resize(job, count);

			// Our buffers should be read-only for everyone else
			job.m_barriers.Transition(positions.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_grid_hash.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_spatial.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_idx_start.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_idx_count.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

			// Reset the index start/count buffers
			{
				job.m_barriers.Commit();

				job.m_cmd_list.SetPipelineState(m_init.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(m_init.m_sig.get());
				job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_params, 0);
				job.m_cmd_list.SetComputeRootUnorderedAccessView(1, m_idx_start->GetGPUVirtualAddress());
				job.m_cmd_list.SetComputeRootUnorderedAccessView(2, m_idx_count->GetGPUVirtualAddress());
				job.m_cmd_list.Dispatch(DispatchCount({ cb_params.CellCount, 1, 1 }, { ThreadGroupSize, 1, 1 }));

				job.m_barriers.UAV(m_idx_start.get());
				job.m_barriers.UAV(m_idx_count.get());
			}

			// Find the grid cell hash for each position
			{
				job.m_barriers.Commit();

				job.m_cmd_list.SetPipelineState(m_populate.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(m_populate.m_sig.get());
				job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_params, 0);
				job.m_cmd_list.SetComputeRootShaderResourceView(1, positions->GetGPUVirtualAddress());
				job.m_cmd_list.SetComputeRootUnorderedAccessView(2, m_grid_hash->GetGPUVirtualAddress());
				job.m_cmd_list.SetComputeRootUnorderedAccessView(3, m_spatial->GetGPUVirtualAddress());
				job.m_cmd_list.Dispatch(DispatchCount({ cb_params.NumPositions, 1, 1 }, { ThreadGroupSize, 1, 1 }));

				job.m_barriers.UAV(m_grid_hash.get());
				job.m_barriers.UAV(m_spatial.get());
			}

			// Sort the cell hashes and position indices so that they're contiguous
			{
				job.m_barriers.Commit();

				m_sorter.Sort(job.m_cmd_list);

				job.m_barriers.UAV(m_grid_hash.get());
				job.m_barriers.UAV(m_spatial.get());
			}

			// Build the lookup data structure
			{
				job.m_barriers.Commit();

				job.m_cmd_list.SetPipelineState(m_build.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(m_build.m_sig.get());
				job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_params, 0);
				job.m_cmd_list.SetComputeRootUnorderedAccessView(1, m_grid_hash->GetGPUVirtualAddress());
				job.m_cmd_list.SetComputeRootUnorderedAccessView(2, m_idx_start->GetGPUVirtualAddress());
				job.m_cmd_list.SetComputeRootUnorderedAccessView(3, m_idx_count->GetGPUVirtualAddress());
				job.m_cmd_list.Dispatch(DispatchCount({ cb_params.NumPositions, 1, 1 }, { ThreadGroupSize, 1, 1 }));

				job.m_barriers.UAV(m_idx_start.get());
				job.m_barriers.UAV(m_idx_count.get());
			}

			// Read back the index start/count buffers and lookup table
			if (readback)
			{
				Output.m_grid_scale = cb_params.GridScale;
				Output.m_cell_count = cb_params.CellCount;
				Output.m_pos_count = cb_params.NumPositions;

				job.m_barriers.Transition(m_idx_start.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Transition(m_idx_count.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Transition(m_spatial.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Commit();

				{
					auto buf = job.m_readback.Alloc(cb_params.NumPositions * sizeof(uint32_t), alignof(uint32_t));
					job.m_cmd_list.CopyBufferRegion(buf, m_spatial.get());
					Output.m_lookup = std::move(buf);
				}
				{
					auto buf = job.m_readback.Alloc(cb_params.CellCount * sizeof(uint32_t), alignof(uint32_t));
					job.m_cmd_list.CopyBufferRegion(buf, m_idx_start.get());
					Output.m_idx_start = std::move(buf);
				}
				{
					auto buf = job.m_readback.Alloc(cb_params.CellCount * sizeof(uint32_t), alignof(uint32_t));
					job.m_cmd_list.CopyBufferRegion(buf, m_idx_count.get());
					Output.m_idx_count = std::move(buf);
				}
			}

			// Our buffers should be read-only for everyone else
			job.m_barriers.Transition(positions.get(), positions_state0);
			job.m_barriers.Transition(m_grid_hash.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_spatial.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_idx_start.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_idx_count.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}
	};

	// A structure that provides spatial partitioning on the CPU.
	struct SpatialLookup
	{
		// Notes:
		//  - Create a long lived instance of this class
		//  - Run the 'SpatialPartition::Update' on the GPU
		//  - Use the step output to update this structure (after calling job.Run())
		//  - You need 'readback = true' in the call to 'SpatialPartition::Update'

		using Cell = SpatialPartition<>::Cell;
		using StepOutput = SpatialPartition<>::StepOutput;

		std::vector<int32_t> m_spatial;
		std::vector<Cell> m_lookup;
		float m_grid_scale;

		// Update the spatial partitioning lookup data based on output from a 'SpatialPartition::Update' call.
		// Requires 'job.Run()' to have been called on the job used to generate the output.
		void Update(StepOutput const& output)
		{
			m_grid_scale = output.m_grid_scale;

			// The spatially ordered list of particle indices
			m_spatial.resize(output.m_pos_count);
			memcpy(m_spatial.data(), output.m_lookup.ptr<int32_t>(), m_spatial.size() * sizeof(int32_t));

			// The map from cell hash to index start/count
			m_lookup.resize(output.m_cell_count);
			auto idx_start_ptr = output.m_idx_start.ptr<int32_t>();
			auto idx_count_ptr = output.m_idx_count.ptr<int32_t>();
			for (auto& cell : m_lookup)
				cell = { *idx_start_ptr++, *idx_count_ptr++ };
		}

		// Find all particles in the cells overlapping 'volume'.
		template <typename PosType, typename FoundCB> requires std::is_invocable_v<FoundCB, PosType const&>
		void Find(BBox volume, std::span<PosType const> particles, FoundCB found) const
		{
			assert(!m_spatial.empty() && "Requires Update() with 'readback' = true");

			auto cell_count = isize(m_lookup);
			auto lwr = GridCell(volume.Lower(), m_grid_scale);
			auto upr = GridCell(volume.Upper(), m_grid_scale);

			for (auto z = lwr.z; z <= upr.z; ++z)
			{
				for (auto y = lwr.y; y <= upr.y; ++y)
				{
					for (auto x = lwr.x; x <= upr.x; ++x)
					{
						auto cell = iv3(x, y, z);
						auto hash = CellHash(cell, cell_count);
						auto& idx = m_lookup[hash];
						for (int i = idx.start, iend = idx.start + idx.count; i != iend; ++i)
						{
							// Get the particle, but skip cell hash collisions
							auto& particle = particles[m_spatial[i]];
							if (GridCell(particle.m_pos, m_grid_scale) != cell)
								continue;

							found(particle);
						}
					}
				}
			}
		}

		// Find all particles within 'radius' of 'position'
		template <typename PosType, typename FoundCB> requires std::is_invocable_v<FoundCB, PosType const&, float>
		void Find(v4 position, float radius, std::span<PosType const> particles, FoundCB found) const
		{
			auto radius_sq = radius * radius;
			Find(BBox(position, v4(radius)), particles, [=](auto const& particle)
			{
				auto dist_sq = LengthSq(position - particle.m_pos);
				if (dist_sq > radius_sq)
					return;

				found(particle, dist_sq);
			});
		}

		struct Stats
		{
			// The average number and variance of particles per cell
			math::AvrVar<> occupancy;
		};
		Stats GetPerformanceStat() const
		{
			Stats stats;
			for (auto const& cell : m_lookup)
				stats.occupancy.Add(cell.count);

			return stats;
		}
	};
}