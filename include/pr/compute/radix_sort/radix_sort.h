//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Usage:
//  Create a long-lived instance of the GpuRadixSort.
//  Resize it to the size of the data to be sorted.
//  Call the overload of Sort that suits your needs.
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/gpu.h"
#include "pr/compute/gpu_job.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"
#include "pr/compute/utility/barrier_batch.h"
#include "pr/compute/utility/pix.h"

namespace pr::compute::gpu_radix_sort
{
	// Types that can be sorted on the GPU
	template <typename T>
	concept GpuSortableKey = std::is_same_v<T, int> || std::is_same_v<T, uint32_t> || std::is_same_v<T, float>;
	template <typename T>
	concept GpuSortableValue = std::is_same_v<T, int> || std::is_same_v<T, uint32_t> || std::is_same_v<T, float> || std::is_same_v<T, void>;

	// Radix sort on the GPU
	template <GpuSortableKey Key, GpuSortableValue Value, bool Ascending = true, D3D12_COMMAND_LIST_TYPE QueueType = D3D12_COMMAND_LIST_TYPE_DIRECT>
	struct GpuRadixSort
	{
		// Notes:
		//  - This class is set up to be used as part of other GPU tasks.
		//    Have a look at the 'GpuJob' class, it can be used to provide
		//    the gsync, command list, and upload/readback buffers.
		//  - You can replace the 'm_sort[0]' resource with your own resource
		//    if you want to avoid copying data. Just be care with resize.
		//  - This type is intended to be used repeatedly to sort large
		//    numbers of elements. It's not suited for transient sorts.
		//  - Use 'Value = void' if no payload is required, i.e., you just want to sort key values.

		using Gpu = Gpu<QueueType>;
		using CmdList = CmdList<QueueType>;
		using IShaderCache = shader_cache::IShaderCache;

		static constexpr int KeyBits = sizeof(Key) * 8; // 32-bit keys atm
		static constexpr int RadixBits = 8;
		static constexpr int Radix = 1 << RadixBits; // The number of digit bins
		static constexpr int RadixPasses = KeyBits / RadixBits;
		static constexpr int MaxReadBack = 1 << 13;
		static constexpr int MaxDispatchDimension = 65535;
		static constexpr bool HasPayload = !std::is_same_v<Value, void>;
		static constexpr bool SortAscending = Ascending;

		// Full sweep, partial sweep, and scan records are reused by both sweeps within each pass.
		static constexpr uint32_t IndirectArgumentStride = 4 * sizeof(uint32_t) + sizeof(D3D12_DISPATCH_ARGUMENTS);
		static constexpr uint32_t IndirectArgumentsPerPass = 3;
		static constexpr uint32_t IndirectArgumentBytes = RadixPasses * IndirectArgumentsPerPass * IndirectArgumentStride;
		static_assert(IndirectArgumentStride == 28);

		struct EReg
		{
			inline static constexpr auto Constants = hlsl::ECBufReg::b0;
			inline static constexpr auto Sort0 = hlsl::EUAVReg::u0;
			inline static constexpr auto Sort1 = hlsl::EUAVReg::u1;
			inline static constexpr auto Payload0 = hlsl::EUAVReg::u2;
			inline static constexpr auto Payload1 = hlsl::EUAVReg::u3;
			inline static constexpr auto GlobalHistogram = hlsl::EUAVReg::u4;
			inline static constexpr auto PassHistogram = hlsl::EUAVReg::u5;
		};

		struct TuningParams
		{
			std::wstring shader_model = L"cs_6_6";
			int partition_size = 7680;
			int keys_per_thread = 15;
			int part_size = 7680;
			bool use_16bit = true;
		};

		Gpu* m_gpu;

		ComputeStep m_init;
		ComputeStep m_init_payload;
		ComputeStep m_sweep_up;
		ComputeStep m_scan;
		ComputeStep m_sweep_down;
		ComputeStep m_indirect_setup;
		D3DPtr<ID3D12CommandSignature> m_indirect_sweep_up;
		D3DPtr<ID3D12CommandSignature> m_indirect_scan;
		D3DPtr<ID3D12CommandSignature> m_indirect_sweep_down;

		D3DPtr<ID3D12Resource> m_sort[2];
		D3DPtr<ID3D12Resource> m_payload[2];
		D3DPtr<ID3D12Resource> m_pass_histogram;
		D3DPtr<ID3D12Resource> m_global_histogram;
		D3DPtr<ID3D12Resource> m_error_count;
		D3DPtr<ID3D12Resource> m_indirect_arguments;

		TuningParams m_tuning;
		int64_t m_size; // Bound capacity; GPU-counted sorts supply a separate logical length through indirect constants.
		bool m_bound_to_external;

		struct Result
		{
			GpuReadbackBuffer::Allocation keys;
			GpuReadbackBuffer::Allocation values;
		};

		explicit GpuRadixSort(Gpu& gpu, TuningParams const& tuning = {}, IShaderCache* shader_cache = nullptr)
			: m_gpu(&gpu)
			, m_init()
			, m_init_payload()
			, m_sweep_up()
			, m_scan()
			, m_sweep_down()
			, m_indirect_setup()
			, m_indirect_sweep_up()
			, m_indirect_scan()
			, m_indirect_sweep_down()
			, m_sort()
			, m_payload()
			, m_pass_histogram()
			, m_global_histogram()
			, m_error_count()
			, m_indirect_arguments()
			, m_tuning(tuning)
			, m_size()
			, m_bound_to_external()
		{
			shader_cache::ResourceSourceResolver resolver;
			auto compiler = ShaderCompiler{}
				.Cache(shader_cache)
				.Source("src/compute/radix_sort/radix_sort.hlsl", resolver)
				.HlslVersion(EHlslVersion::Hlsl2021)
				.ShaderModel(m_tuning.shader_model)
				.Optimise()
				.Define(L"KEYS_PER_THREAD", std::to_wstring(m_tuning.keys_per_thread))
				.Define(L"PART_SIZE", std::to_wstring(m_tuning.part_size));

			if      constexpr (std::is_same_v<Key, int>)      compiler.Define(L"KEY_TYPE_ID", L"0");
			else if constexpr (std::is_same_v<Key, uint32_t>) compiler.Define(L"KEY_TYPE_ID", L"1");
			else if constexpr (std::is_same_v<Key, float>)    compiler.Define(L"KEY_TYPE_ID", L"2");
			else static_assert(false, "Unsupported key type");

			if      constexpr (std::is_same_v<Value, int>)      compiler.Define(L"PAYLOAD_TYPE_ID", L"0");
			else if constexpr (std::is_same_v<Value, uint32_t>) compiler.Define(L"PAYLOAD_TYPE_ID", L"1");
			else if constexpr (std::is_same_v<Value, float>)    compiler.Define(L"PAYLOAD_TYPE_ID", L"2");
			else static_assert(false, "Unsupported payload type");

			if constexpr (Ascending)  compiler.Define(L"SHOULD_ASCEND");
			if constexpr (HasPayload) compiler.Define(L"SORT_PAIRS", L"1");

			if (m_tuning.use_16bit)
			{
				compiler.Define(L"DIGIT_TYPE", L"uint16_t");
				compiler.Arg(L"-enable-16bit-types");
			}

			// InitRadixSort
			{
				auto bytecode = compiler.EntryPoint(L"InitRadixSort").Compile();
				m_init.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.UAV(EReg::GlobalHistogram)
					.Create(*m_gpu, "GpuRadixSort:InitSig");
				m_init.m_pso = ComputePSO(m_init.m_sig.get(), bytecode)
					.Create(*m_gpu, "GpuRadixSort:InitPSO");
			}

			// InitPayload
			{
				auto bytecode = compiler.EntryPoint(L"InitPayload").Compile();
				m_init_payload.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32(EReg::Constants, 4)
					.UAV(EReg::Payload0)
					.Create(*m_gpu, "GpuRadixSort:InitPayloadSig");
				m_init_payload.m_pso = ComputePSO(m_init_payload.m_sig.get(), bytecode)
					.Create(*m_gpu, "GpuRadixSort:InitPayloadPSO");
			}

			// Sweep Up
			{
				auto bytecode = compiler.EntryPoint(L"SweepUp").Compile();
				m_sweep_up.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32(EReg::Constants, 4)
					.UAV(EReg::Sort0)
					.UAV(EReg::GlobalHistogram)
					.UAV(EReg::PassHistogram)
					.Create(*m_gpu, "GpuRadixSort:SweepUpSig");
				m_sweep_up.m_pso = ComputePSO(m_sweep_up.m_sig.get(), bytecode)
					.Create(*m_gpu, "GpuRadixSort:SweepUpPSO");
			}

			// Scan
			{
				auto bytecode = compiler.EntryPoint(L"Scan").Compile();
				m_scan.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32(EReg::Constants, 4)
					.UAV(EReg::PassHistogram)
					.Create(*m_gpu, "GpuRadixSort:ScanSig");
				m_scan.m_pso = ComputePSO(m_scan.m_sig.get(), bytecode)
					.Create(*m_gpu, "GpuRadixSort:ScanPSO");
			}

			// Sweep Down
			{
				auto bytecode = compiler.EntryPoint(L"SweepDown").Compile();
				m_sweep_down.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32(EReg::Constants, 4)
					.UAV(EReg::Sort0)
					.UAV(EReg::Sort1)
					.UAV(EReg::Payload0)
					.UAV(EReg::Payload1)
					.UAV(EReg::GlobalHistogram)
					.UAV(EReg::PassHistogram)
					.Create(*m_gpu, "GpuRadixSort:SweepDownSig");
				m_sweep_down.m_pso = ComputePSO(m_sweep_down.m_sig.get(), bytecode)
					.Create(*m_gpu, "GpuRadixSort:SweepDownPSO");
			}

		}

		// Create sort-size independent buffers
		void CreateStaticSizeBuffers(CmdList& cmd_list)
		{
			if (m_global_histogram == nullptr)
			{
				ResDesc desc = ResDesc::Buf<Key>(Radix * RadixPasses, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_global_histogram = m_gpu->CreateResource(desc, cmd_list, "RadixSort:histogram");
			}
			if (m_error_count == nullptr)
			{
				ResDesc desc = ResDesc::Buf<Key>(1, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_error_count = m_gpu->CreateResource(desc, cmd_list, "RadixSort:error_count");
			}
		}

		// Bind externally owned streams; 'size' is the CPU-known length or the capacity for a GPU-counted sort.
		// Size must fit a nonnegative int; zero needs no resources. Bound resources must remain alive until recorded work has completed.
		void Bind(CmdList& cmd_list, int64_t size, D3DPtr<ID3D12Resource> sort0, D3DPtr<ID3D12Resource> payload0)
		{
			// Validate capacity before allocating scratch, and avoid zero-width D3D12 resources.
			auto const partitions = PartitionCount(size);
			if (size == 0)
			{
				m_sort[0] = sort0;
				m_payload[0] = payload0;
				m_size = 0;
				m_bound_to_external = true;
				return;
			}
			ValidateStream(sort0.get(), size);
			if constexpr (HasPayload)
				ValidateStream(payload0.get(), size);

			// Scratch storage is stable across count changes and repeated binds at the same capacity.
			CreateStaticSizeBuffers(cmd_list);

			if (size != m_size || m_sort[1] == nullptr || m_payload[1] == nullptr || m_pass_histogram == nullptr)
			{
				ResDesc sort_desc = ResDesc::Buf<Key>(size, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_sort[1] = m_gpu->CreateResource(sort_desc, cmd_list, "RadixSort:sort1");

				using T = std::conditional_t<HasPayload, Value, int>;
				ResDesc payload_desc = ResDesc::Buf<T>(HasPayload ? size : 1, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_payload[1] = m_gpu->CreateResource(payload_desc, cmd_list, "RadixSort:payload1");

				ResDesc histogram_desc = ResDesc::Buf<Key>(s_cast<int64_t>(Radix) * partitions, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_pass_histogram = m_gpu->CreateResource(histogram_desc, cmd_list, "RadixSort:passHistBuffer");
			}

			m_sort[0] = sort0;
			m_payload[0] = payload0;
			m_size = size;
			m_bound_to_external = true;
		}

		// Release every size-dependent and static resource while retaining compiled pipeline state for later reuse.
		void ReleaseBuffers()
		{
			m_sort[0] = nullptr;
			m_sort[1] = nullptr;
			m_payload[0] = nullptr;
			m_payload[1] = nullptr;
			m_pass_histogram = nullptr;
			m_global_histogram = nullptr;
			m_error_count = nullptr;
			m_indirect_arguments = nullptr;
			m_size = 0;
			m_bound_to_external = false;
		}

		// Return the exact number of dispatch commands recorded by one sort at the current bound size.
		int SortDispatchCount() const
		{
			if (m_size == 0)
				return 0;

			auto const thread_blocks = PartitionCount(m_size);
			auto const full_blocks = thread_blocks / MaxDispatchDimension;
			auto const partial_blocks = thread_blocks - full_blocks * MaxDispatchDimension;
			auto const sweep_dispatch_count = (full_blocks != 0 ? 1 : 0) + (partial_blocks != 0 ? 1 : 0);
			return 1 + RadixPasses * (2 * sweep_dispatch_count + 1);
		}

		// Count submitted dispatch records: setup, histogram reset, and indirect dispatches (including zero-group records).
		// This is not the GPU-active dispatch count or the number of ExecuteIndirect calls; no counter readback is performed.
		int IndirectSortDispatchCount() const
		{
			if (m_size == 0)
				return 0;

			auto const sweep_records = PartitionCount(m_size) >= MaxDispatchDimension ? 2 : 1;
			return 2 + RadixPasses * (2 * sweep_records + 1);
		}

		// Return retained bytes owned by the sorter without charging externally bound key and payload streams.
		size_t AllocatedBufferBytes() const
		{
			auto resource_bytes = [](D3DPtr<ID3D12Resource> const& resource)
			{
				return resource != nullptr ? static_cast<size_t>(resource->GetDesc().Width) : 0;
			};
			auto bytes =
				resource_bytes(m_sort[1]) +
				resource_bytes(m_payload[1]) +
				resource_bytes(m_pass_histogram) +
				resource_bytes(m_global_histogram) +
				resource_bytes(m_error_count) +
				resource_bytes(m_indirect_arguments);
			if (!m_bound_to_external)
				bytes += resource_bytes(m_sort[0]) + resource_bytes(m_payload[0]);

			return bytes;
		}

		// Resize the GPU buffers in preparation for sorting 'size' elements
		void Resize(CmdList& cmd_list, int64_t size)
		{
			auto const partitions = PartitionCount(size);
			if (size == m_size && !m_bound_to_external)
				return;

			if (size == 0)
			{
				ReleaseBuffers();
				return;
			}
			CreateStaticSizeBuffers(cmd_list);

			// Create sort-size dependent buffers
			{
				ResDesc desc = ResDesc::Buf<Key>(size, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_sort[0] = m_gpu->CreateResource(desc, cmd_list, "RadixSort:sort0");
				m_sort[1] = m_gpu->CreateResource(desc, cmd_list, "RadixSort:sort1");
			}
			{
				using T = std::conditional_t<HasPayload, Value, int>;
				ResDesc desc = ResDesc::Buf<T>(HasPayload ? size : 1, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_payload[0] = m_gpu->CreateResource(desc, cmd_list, "RadixSort:payload0");
				m_payload[1] = m_gpu->CreateResource(desc, cmd_list, "RadixSort:payload1");
			}
			{
				ResDesc desc = ResDesc::Buf<Key>(s_cast<int64_t>(Radix) * partitions, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_pass_histogram = m_gpu->CreateResource(desc, cmd_list, "RadixSort:passHistBuffer");
			}

			m_size = size;
			m_bound_to_external = false;
		}

		// Sort 'values' by 'keys' in-place
		void Sort(std::span<Key> keys, std::span<Value> values, ComputeJob& job)
		{
			// Upload 'keys' and 'values' to the GPU and then sort them
			auto result = Sort(job.m_cmd_list, keys, values, job.m_upload, job.m_readback);

			// Do the sort and wait for it to complete
			job.Run();

			// Read back the results and update the input arrays
			if (!keys.empty())
			{
				memcpy(keys.data(), result.keys.ptr<Key>(), result.keys.m_size);
			}
			if constexpr (HasPayload)
			{
				if (!values.empty())
					memcpy(values.data(), result.values.ptr<Value>(), result.values.m_size);
			}
		}

		// Sort 'values' by 'keys' using the provided command list
		// Returns Read back buffer allocations that will contain the sorted result once the command list has been executed.
		Result Sort(CmdList& cmd_list, std::span<Key const> keys, std::span<Value const> values, GpuUploadBuffer& upload, GpuReadbackBuffer& readback)
		{
			if (ssize(keys) > m_size)
			{
				throw std::runtime_error("GpuRadixSort::Sort: sort buffer is not large enough. Use 'Resize' first.");
			}
			if constexpr (HasPayload)
			{
				if (keys.size() != values.size())
					throw std::runtime_error("GpuRadixSort::Sort: keys and values must be the same size");
			}
			else
			{
				if (!values.empty())
					throw std::runtime_error("GpuRadixSort::Sort: values provided to keys-only sorter");
			}

			// Empty sorts do not need upload/readback allocations or bound resources.
			if (m_size == 0)
				return {};

			BarrierBatch barriers(cmd_list);
			barriers.Transition(m_sort[0].get(), D3D12_RESOURCE_STATE_COPY_DEST);
			barriers.Transition(m_payload[0].get(), D3D12_RESOURCE_STATE_COPY_DEST);
			barriers.Commit();

			// Copy the keys and values to the GPU. If 'keys' is smaller than 'm_size', pad with 0xFF
			{
				auto buf = upload.Alloc(m_size * sizeof(Key), alignof(Key));
				memcpy(buf.ptr<Key>(), keys.data(), keys.size() * sizeof(Key));
				memset(buf.ptr<Key>() + keys.size(), 0xFF, (m_size - keys.size()) * sizeof(Key));
				cmd_list.CopyBufferRegion(m_sort[0].get(), 0, buf.m_res, buf.m_ofs, buf.m_size);
			}
			if constexpr (HasPayload)
			{
				auto buf = upload.Alloc(m_size * sizeof(Value), alignof(Value));
				memcpy(buf.ptr<Value>(), values.data(), values.size() * sizeof(Value));
				memset(buf.ptr<Key>() + values.size(), 0xFF, (m_size - values.size()) * sizeof(Value));
				cmd_list.CopyBufferRegion(m_payload[0].get(), 0, buf.m_res, buf.m_ofs, buf.m_size);
			}

			barriers.Transition(m_sort[0].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			barriers.Transition(m_payload[0].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			barriers.Commit();

			// Sort the buffers on the GPU
			Sort(cmd_list);

			barriers.Transition(m_sort[0].get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			barriers.Transition(m_payload[0].get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			barriers.Commit();

			Result result = {};

			// Copy the results back to the CPU
			{
				auto buf = readback.Alloc(ssize(keys) * sizeof(Key), alignof(Key));
				cmd_list.CopyBufferRegion(buf.m_res, buf.m_ofs, m_sort[0].get(), 0, buf.m_size);
				result.keys = std::move(buf);
			}
			if constexpr (HasPayload)
			{
				auto buf = readback.Alloc(ssize(values) * sizeof(Value), alignof(Value));
				cmd_list.CopyBufferRegion(buf.m_res, buf.m_ofs, m_payload[0].get(), 0, buf.m_size);
				result.values = std::move(buf);
			}
			
			barriers.Transition(m_sort[0].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Transition(m_payload[0].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Commit();

			return result;
		}

		// Sort the keys/values in 'm_sort[0]/m_payload[0]' assuming they're uploaded to the GPU already.
		// This overload is intended for use when you want to leave the keys/values on the GPU without reading them back.
		void Sort(CmdList& cmd_list)
		{
			RecordSort(cmd_list, false);
		}

		// Sort min(counter, capacity / count_multiplier) * count_multiplier keys, preserving equal-key payload order and the inactive suffix.
		// The counter is a uint32 at a four-byte-aligned offset in a shader-readable buffer; count_multiplier must be nonzero.
		// The counter's tracked resource state is restored. Streams must already be in UAV state, as for the CPU-known overload.
		// The first nonempty-capacity call creates count-only pipeline state and arguments; subsequent calls reuse them.
		// Record reuse on one ordered GPU queue. Keep resources alive and fence before resizing or releasing scratch used by unfinished work.
		void Sort(CmdList& cmd_list, ID3D12Resource* counter, uint64_t counter_offset = 0, uint32_t count_multiplier = 1)
		{
			// An empty capacity is a no-op, including when no counter or streams have been bound.
			if (m_size == 0)
				return;

			// Validate the raw counter view and the shared CPU/shader partition layout before recording any work.
			if (counter == nullptr || count_multiplier == 0)
				throw std::invalid_argument("GpuRadixSort::Sort: a counter and nonzero count multiplier are required");

			auto const desc = counter->GetDesc();
			switch (desc.Dimension)
			{
				case D3D12_RESOURCE_DIMENSION_BUFFER: { break; }
				default: { throw std::invalid_argument("GpuRadixSort::Sort: the counter must reference a buffer"); }
			}
			if ((desc.Flags & D3D12_RESOURCE_FLAG_DENY_SHADER_RESOURCE) != 0 ||
				counter_offset % sizeof(uint32_t) != 0 || counter_offset > desc.Width || desc.Width - counter_offset < sizeof(uint32_t))
				throw std::invalid_argument("GpuRadixSort::Sort: the counter must reference a shader-readable, aligned uint32 within a buffer");

			if (m_tuning.partition_size != m_tuning.part_size)
				throw std::invalid_argument("GpuRadixSort::Sort: CPU and shader partition sizes must agree");

			// CPU-known sorts never create count-only pipeline state or allocate its argument buffer.
			EnsureIndirectResources(cmd_list);

			// A transition orders earlier counter writes; returning arguments to UAV orders reuse after earlier indirect reads.
			BarrierBatch barriers(cmd_list);
			auto const counter_state = cmd_list.ResState(counter).Mip0State();
			auto const transition_counter = (counter_state & D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE) == 0;
			if (transition_counter)
				barriers.Transition(counter, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

			barriers.Transition(m_indirect_arguments.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			barriers.Commit();

			// Keep the raw root view aligned while supporting any four-byte-aligned field within the counter resource.
			auto const view_offset = counter_offset / D3D12_RAW_UAV_SRV_BYTE_ALIGNMENT * D3D12_RAW_UAV_SRV_BYTE_ALIGNMENT;
			std::array<uint32_t, 4> constants = { s_cast<uint32_t>(m_size), count_multiplier, s_cast<uint32_t>(m_tuning.partition_size), s_cast<uint32_t>(counter_offset - view_offset) };
			cmd_list.SetPipelineState(m_indirect_setup.m_pso.get());
			cmd_list.SetComputeRootSignature(m_indirect_setup.m_sig.get());
			cmd_list.SetComputeRoot32BitConstants(0, isize(constants), constants.data(), 0);
			cmd_list.SetComputeRootShaderResourceView(1, counter->GetGPUVirtualAddress() + view_offset);
			cmd_list.SetComputeRootUnorderedAccessView(2, m_indirect_arguments->GetGPUVirtualAddress());
			cmd_list.Dispatch(1, 1, 1);

			// Publish complete records before ExecuteIndirect and leave counter ownership with the caller.
			barriers.Transition(m_indirect_arguments.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			if (transition_counter)
				barriers.Transition(counter, counter_state);

			barriers.Commit();
			RecordSort(cmd_list, true);
		}

		// Initialise the payload buffer to incrementing indices.
		// A common case when creating a lookup map
		void InitPayload(CmdList& cmd_list)
		{
			if (m_size == 0)
				return;

			auto const thread_blocks = PartitionCount(m_size);
			cmd_list.SetPipelineState(m_init_payload.m_pso.get());
			cmd_list.SetComputeRootSignature(m_init_payload.m_sig.get());
			cmd_list.SetComputeRootUnorderedAccessView(1, m_payload[0]->GetGPUVirtualAddress());

			const auto full_blocks = s_cast<uint32_t>(thread_blocks / MaxDispatchDimension);
			if (full_blocks)
			{
				std::array<uint32_t, 4> t = { s_cast<uint32_t>(m_size), 0, thread_blocks, 0 };
				cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
				cmd_list.Dispatch(MaxDispatchDimension, full_blocks, 1);
			}

			const auto partial_blocks = s_cast<uint32_t>(thread_blocks - full_blocks * MaxDispatchDimension);
			if (partial_blocks)
			{
				std::array<uint32_t, 4> t = { s_cast<uint32_t>(m_size), 0, thread_blocks, (full_blocks << 1) | 1 };
				cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
				cmd_list.Dispatch(partial_blocks, 1, 1);
			}
		}

	private:

		// Create count-only state on first use, or restore its argument buffer after ReleaseBuffers.
		void EnsureIndirectResources(CmdList& cmd_list)
		{
			if (m_indirect_arguments != nullptr)
				return;

			// The constructor's shader cache is borrowed only during construction, not retained for deferred compilation.
			if (m_indirect_setup.m_pso == nullptr)
			{
				shader_cache::ResourceSourceResolver resolver;
				auto bytecode = ShaderCompiler{}
					.Source("src/compute/radix_sort/radix_sort.hlsl", resolver)
					.HlslVersion(EHlslVersion::Hlsl2021)
					.ShaderModel(m_tuning.shader_model)
					.Optimise()
					.Define(L"RADIX_SORT_INDIRECT_SETUP", L"1")
					.EntryPoint(L"SetupIndirectSort")
					.Compile();
				auto setup = ComputeStep{};
				setup.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32(EReg::Constants, 4)
					.SRV(hlsl::ESRVReg::t0)
					.UAV(hlsl::EUAVReg::u0)
					.Create(*m_gpu, "GpuRadixSort:IndirectSetupSig");
				setup.m_pso = ComputePSO(setup.m_sig.get(), bytecode)
					.Create(*m_gpu, "GpuRadixSort:IndirectSetupPSO");
				auto sweep_up = CreateIndirectSignature(m_sweep_up.m_sig.get());
				auto scan = CreateIndirectSignature(m_scan.m_sig.get());
				auto sweep_down = CreateIndirectSignature(m_sweep_down.m_sig.get());

				// Publish the pipeline and signatures together so a failed creation can be retried safely.
				m_indirect_setup = std::move(setup);
				m_indirect_sweep_up = std::move(sweep_up);
				m_indirect_scan = std::move(scan);
				m_indirect_sweep_down = std::move(sweep_down);
			}

			// Capacity and counter changes do not change the fixed argument-table allocation.
			auto desc = ResDesc::Buf<uint32_t>(IndirectArgumentBytes / sizeof(uint32_t), {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
			m_indirect_arguments = m_gpu->CreateResource(desc, cmd_list, "RadixSort:indirect_arguments");
		}

		// Return an overflow-safe partition count within the existing signed-size and two-dimensional dispatch limits.
		uint32_t PartitionCount(int64_t size) const
		{
			if (size < 0 || size > std::numeric_limits<int>::max() || m_tuning.partition_size <= 0)
				throw std::out_of_range("GpuRadixSort: size must fit a nonnegative int and partition size must be positive");

			auto const partitions = (size + m_tuning.partition_size - 1) / m_tuning.partition_size;
			if (partitions > int64_t(MaxDispatchDimension) * MaxDispatchDimension)
				throw std::out_of_range("GpuRadixSort: partition count exceeds the two-dimensional dispatch limit");

			return s_cast<uint32_t>(partitions);
		}

		// Require an external stream to hold all bound 32-bit elements and permit UAV access.
		static void ValidateStream(ID3D12Resource* resource, int64_t size)
		{
			if (resource == nullptr)
				throw std::invalid_argument("GpuRadixSort::Bind: a nonempty stream requires a resource");

			auto const desc = resource->GetDesc();
			switch (desc.Dimension)
			{
				case D3D12_RESOURCE_DIMENSION_BUFFER: { break; }
				default: { throw std::invalid_argument("GpuRadixSort::Bind: stream must reference a buffer"); }
			}
			if ((desc.Flags & D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS) == 0 || desc.Width < s_cast<uint64_t>(size) * sizeof(Key))
				throw std::invalid_argument("GpuRadixSort::Bind: stream must be a UAV buffer large enough for the bound capacity");
		}

		// Create a signature tied to the kernel's unchanged root layout: four constants at parameter zero, then dispatch.
		D3DPtr<ID3D12CommandSignature> CreateIndirectSignature(ID3D12RootSignature* root_signature)
		{
			D3D12_INDIRECT_ARGUMENT_DESC arguments[] =
			{
				{ .Type = D3D12_INDIRECT_ARGUMENT_TYPE_CONSTANT, .Constant = { .RootParameterIndex = 0, .DestOffsetIn32BitValues = 0, .Num32BitValuesToSet = 4 } },
				{ .Type = D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH },
			};
			auto const desc = D3D12_COMMAND_SIGNATURE_DESC
			{
				.ByteStride = IndirectArgumentStride,
				.NumArgumentDescs = 2,
				.pArgumentDescs = arguments,
				.NodeMask = 0,
			};
			D3DPtr<ID3D12CommandSignature> signature;
			Check(m_gpu->device()->CreateCommandSignature(&desc, root_signature, __uuidof(ID3D12CommandSignature), reinterpret_cast<void**>(signature.address_of())));
			return signature;
		}

		// Submit a sweep using CPU-known dimensions or GPU-authored full/partial records with the same flattened indexing.
		void DispatchSweep(CmdList& cmd_list, uint32_t radix_shift, uint32_t thread_blocks, ID3D12CommandSignature* indirect_signature)
		{
			auto const full_blocks = thread_blocks / MaxDispatchDimension;
			if (indirect_signature != nullptr)
			{
				// Even at an exact full-capacity boundary, smaller GPU counts can require the partial record.
				auto const offset = (radix_shift / RadixBits * IndirectArgumentsPerPass + (full_blocks != 0 ? 0 : 1)) * IndirectArgumentStride;
				cmd_list.ExecuteIndirect(indirect_signature, full_blocks != 0 ? 2 : 1, m_indirect_arguments.get(), offset);
				return;
			}

			// Preserve the CPU-known full dispatch and the flagged partial dispatch without changing root constants.
			if (full_blocks != 0)
			{
				std::array<uint32_t, 4> constants = { s_cast<uint32_t>(m_size), radix_shift, thread_blocks, 0 };
				cmd_list.SetComputeRoot32BitConstants(0, isize(constants), constants.data(), 0);
				cmd_list.Dispatch(MaxDispatchDimension, full_blocks, 1);
			}
			auto const partial_blocks = thread_blocks % MaxDispatchDimension;
			if (partial_blocks != 0)
			{
				std::array<uint32_t, 4> constants = { s_cast<uint32_t>(m_size), radix_shift, thread_blocks, (full_blocks << 1) | 1 };
				cmd_list.SetComputeRoot32BitConstants(0, isize(constants), constants.data(), 0);
				cmd_list.Dispatch(partial_blocks, 1, 1);
			}
		}

		// Share pass ordering, bindings, ping-pong ownership, and barriers between both count sources.
		void RecordSort(CmdList& cmd_list, bool gpu_counted)
		{
			if (m_size == 0)
				return;

			// Four byte-radix passes perform O(active keys + radix * active partitions) work using O(capacity) retained scratch.
			auto const thread_blocks = PartitionCount(m_size);
			pix::BeginEvent(cmd_list.get(), 0xFF90aa3f, "Gpu Radix Sort");

			// Reset the histogram
			{
				cmd_list.SetPipelineState(m_init.m_pso.get());
				cmd_list.SetComputeRootSignature(m_init.m_sig.get());
				cmd_list.SetComputeRootUnorderedAccessView(0, m_global_histogram->GetGPUVirtualAddress());
				cmd_list.Dispatch(1, 1, 1);
			}

			BarrierBatch barriers(cmd_list);
			barriers.UAV(m_global_histogram.get());
			barriers.Commit();

			// Do the sort
			int i = 0, j = 1;
			for (auto radix_shift = 0U; radix_shift != KeyBits; radix_shift += RadixBits)
			{
				// Sweep Up
				{
					cmd_list.SetPipelineState(m_sweep_up.m_pso.get());
					cmd_list.SetComputeRootSignature(m_sweep_up.m_sig.get());
					cmd_list.SetComputeRootUnorderedAccessView(1, m_sort[i]->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(2, m_global_histogram->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(3, m_pass_histogram->GetGPUVirtualAddress());

					DispatchSweep(cmd_list, radix_shift, thread_blocks, gpu_counted ? m_indirect_sweep_up.get() : nullptr);
				}

				barriers.UAV(m_pass_histogram.get());
				barriers.Commit();

				// Scan
				{
					cmd_list.SetPipelineState(m_scan.m_pso.get());
					cmd_list.SetComputeRootSignature(m_scan.m_sig.get());
					cmd_list.SetComputeRootUnorderedAccessView(1, m_pass_histogram->GetGPUVirtualAddress());
					if (gpu_counted)
					{
						auto const offset = (radix_shift / RadixBits * IndirectArgumentsPerPass + 2) * IndirectArgumentStride;
						cmd_list.ExecuteIndirect(m_indirect_scan.get(), 1, m_indirect_arguments.get(), offset);
					}
					else
					{
						std::array<uint32_t, 4> constants = { 0, 0, thread_blocks, 0 };
						cmd_list.SetComputeRoot32BitConstants(0, isize(constants), constants.data(), 0);
						cmd_list.Dispatch(256, 1, 1);
					}
				}

				barriers.UAV(m_pass_histogram.get());
				barriers.UAV(m_global_histogram.get());
				barriers.Commit();

				// Sweep Down
				{
					cmd_list.SetPipelineState(m_sweep_down.m_pso.get());
					cmd_list.SetComputeRootSignature(m_sweep_down.m_sig.get());
					cmd_list.SetComputeRootUnorderedAccessView(1, m_sort[i]->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(2, m_sort[j]->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(3, m_payload[i]->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(4, m_payload[j]->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(5, m_global_histogram->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(6, m_pass_histogram->GetGPUVirtualAddress());

					DispatchSweep(cmd_list, radix_shift, thread_blocks, gpu_counted ? m_indirect_sweep_down.get() : nullptr);
				}

				barriers.UAV(m_sort[i].get());
				barriers.UAV(m_sort[j].get());
				barriers.UAV(m_payload[i].get());
				barriers.UAV(m_payload[j].get());
				barriers.Commit();

				i = 1 - i;
				j = 1 - j;
			}

			pix::EndEvent(cmd_list.get());
		}

	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::compute::gpu_radix_sort::tests
{
	// Exercise both count sources against independent CPU results and inspect GPU-authored dispatch bounds.
	PRUnitTestClass(GpuRadixSortTests)
	{
		using Sorter = GpuRadixSort<uint32_t, uint32_t, true, D3D12_COMMAND_LIST_TYPE_COMPUTE>;

		// Keep one queue, job, and sorter for each complete sequence of count changes.
		struct Fixture
		{
			ComGpu m_gpu;
			ComputeJob m_job;
			Sorter m_sorter;
			D3DPtr<ID3D12Resource> m_keys;
			D3DPtr<ID3D12Resource> m_values;
			D3DPtr<ID3D12Resource> m_counter;

			// External streams include guard elements beyond the bound capacity.
			explicit Fixture(uint32_t capacity)
				: m_gpu()
				, m_job(m_gpu, "Radix sort tests", 0xff90aa3f)
				, m_sorter(m_gpu)
			{
				auto stream_desc = ResDesc::Buf<uint32_t>(int64_t(capacity) + 32, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				auto counter_desc = ResDesc::Buf<uint32_t>(8, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_keys = m_gpu.CreateResource(stream_desc, m_job.m_cmd_list, "RadixSortTest:keys");
				m_values = m_gpu.CreateResource(stream_desc, m_job.m_cmd_list, "RadixSortTest:values");
				m_counter = m_gpu.CreateResource(counter_desc, m_job.m_cmd_list, "RadixSortTest:counter");
				m_sorter.Bind(m_job.m_cmd_list, capacity, m_keys, m_values);
			}

			// Opt into counted sorting without touching uninitialized key or payload streams.
			void WarmIndirect()
			{
				std::array<uint32_t, 8> counter = {};
				Upload(m_counter.get(), counter);
				m_sorter.Sort(m_job.m_cmd_list, m_counter.get(), 4, 2);
				m_job.Run();
			}

			// Upload a complete test stream and leave it ready for UAV work.
			void Upload(ID3D12Resource* resource, std::span<uint32_t const> values)
			{
				auto& cmd_list = m_job.m_cmd_list;
				BarrierBatch barriers(cmd_list);
				barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST);
				barriers.Commit();
				auto allocation = m_job.m_upload.Alloc(ssize(values) * sizeof(uint32_t), alignof(uint32_t));
				memcpy(allocation.ptr<uint32_t>(), values.data(), values.size_bytes());
				cmd_list.CopyBufferRegion(resource, 0, allocation.m_res, allocation.m_ofs, allocation.m_size);
				barriers.Transition(resource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				barriers.Commit();
			}

			// Snapshot a whole resource without changing the state expected by later recorded work.
			GpuReadbackBuffer::Allocation Readback(ID3D12Resource* resource)
			{
				auto& cmd_list = m_job.m_cmd_list;
				auto const state = cmd_list.ResState(resource).Mip0State();
				BarrierBatch barriers(cmd_list);
				barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_SOURCE);
				barriers.Commit();
				auto allocation = m_job.m_readback.Alloc(resource->GetDesc().Width, alignof(uint32_t));
				cmd_list.CopyBufferRegion(allocation.m_res, allocation.m_ofs, resource, 0, allocation.m_size);
				barriers.Transition(resource, state);
				barriers.Commit();
				return allocation;
			}
		};

		// CPU-only use must leave every count-only pipeline object and resource uncreated.
		static void CheckIndirectUnused(Sorter const& sorter)
		{
			PR_EXPECT(sorter.m_indirect_setup.m_sig == nullptr && sorter.m_indirect_setup.m_pso == nullptr);
			PR_EXPECT(sorter.m_indirect_sweep_up == nullptr && sorter.m_indirect_scan == nullptr && sorter.m_indirect_sweep_down == nullptr);
			PR_EXPECT(sorter.m_indirect_arguments == nullptr);
		}

		// Check all seven DWORDs independently, including the flattened offset of a partial two-dimensional sweep.
		static void CheckArguments(uint32_t const* arguments, uint32_t capacity, uint32_t raw_count, uint32_t multiplier, uint32_t partition_size)
		{
			auto const active = std::min(uint64_t(raw_count), uint64_t(capacity) / multiplier) * multiplier;
			auto const blocks = (active + partition_size - 1) / partition_size;
			auto const full_blocks = blocks / Sorter::MaxDispatchDimension;
			auto const partial_blocks = blocks % Sorter::MaxDispatchDimension;
			for (auto pass = 0U; pass != Sorter::RadixPasses; ++pass)
			{
				auto const* full = arguments + pass * Sorter::IndirectArgumentsPerPass * 7;
				auto const* partial = full + 7;
				auto const* scan = partial + 7;
				PR_EXPECT(full[0] == active && full[1] == pass * Sorter::RadixBits && full[2] == blocks && full[3] == 0);
				PR_EXPECT(full[4] == Sorter::MaxDispatchDimension && full[5] == full_blocks && full[6] == 1);
				PR_EXPECT(partial[0] == active && partial[1] == pass * Sorter::RadixBits && partial[2] == blocks && partial[3] == ((full_blocks << 1) | 1));
				PR_EXPECT(partial[4] == partial_blocks && partial[5] == 1 && partial[6] == 1);
				PR_EXPECT(uint64_t(partial[3] >> 1) * Sorter::MaxDispatchDimension + partial[4] == blocks);
				PR_EXPECT(scan[0] == 0 && scan[1] == 0 && scan[2] == blocks && scan[3] == 0);
				PR_EXPECT(scan[4] == (active != 0 ? 256U : 0U) && scan[5] == 1 && scan[6] == 1);
			}
		}

		// Compare counted and CPU-known sorts, including equal maximum keys, against a stable CPU prefix sort.
		PRUnitTestMethod(ActivePrefixesAndReuse, Extended)
		{
			// Reuse one non-multiple capacity so clamping must leave a final unmatched element untouched for multiplier two.
			constexpr auto capacity = 2U * 7680 + 17;
			constexpr auto sentinel = 0xdeadbeefU;
			auto fixture = Fixture(capacity);
			auto& sorter = fixture.m_sorter;
			auto& cmd_list = fixture.m_job.m_cmd_list;
			CheckIndirectUnused(sorter);
			PR_EXPECT(sorter.AllocatedBufferBytes() == capacity * 8ULL + 3 * 256 * 4 + 1024 * 4 + 4);

			// Only the first counted sort adds the argument table; steady-state calls retain its allocation.
			fixture.WarmIndirect();
			auto const bytes = sorter.AllocatedBufferBytes();
			auto* sort_scratch = sorter.m_sort[1].get();
			auto* payload_scratch = sorter.m_payload[1].get();
			auto* histogram = sorter.m_pass_histogram.get();
			auto* arguments = sorter.m_indirect_arguments.get();
			PR_EXPECT(sorter.SortDispatchCount() == 13);
			PR_EXPECT(sorter.IndirectSortDispatchCount() == 14);
			PR_EXPECT(bytes == capacity * 8ULL + 3 * 256 * 4 + 1024 * 4 + 4 + Sorter::IndirectArgumentBytes);

			// Large-to-empty-to-small changes expose stale histograms, stale arguments, and unwanted suffix writes.
			struct CountCase
			{
				uint32_t raw_count;
				uint32_t multiplier;
				bool equal_keys = false;
				bool cpu_known = false;
			};
			CountCase const cases[] =
			{
				{ capacity, 1 }, { 0, 1 }, { 1, 1 }, { 127, 1 }, { 128, 1 }, { 129, 1 },
				{ 7679, 1 }, { 7680, 1 }, { 7681, 1 }, { capacity, 1, true }, { 0, 2 },
				{ 1, 2 }, { 3839, 2 }, { 3840, 2 }, { 3841, 2 }, { 0xffffffffU, 2 },
				{ 0xffffffffU, 1 }, { 1, 0xffffffffU }, { 0, 1 }, { 17, 1 }, { capacity, 1, false, true },
			};
			for (auto const& test : cases)
			{
				// Distinct payload indices make any loss of equal-key stability observable across partitions.
				std::vector<uint32_t> keys(capacity + 32);
				std::vector<uint32_t> values(keys.size());
				std::vector<uint32_t> scratch(capacity, sentinel);
				std::vector<std::pair<uint32_t, uint32_t>> expected(keys.size());
				uint32_t const palette[] = { 0xffffffffU, 0, 0x80000000U, 0xfffffffeU, 0x01000000U, 0x00010000U, 17, 0x00000100U };
				for (auto i = 0U; i != keys.size(); ++i)
				{
					keys[i] = test.equal_keys ? 0xffffffffU : palette[(i * 17 + i / 5) % std::size(palette)];
					values[i] = i;
					expected[i] = { keys[i], values[i] };
				}
				auto const active = s_cast<size_t>(std::min(uint64_t(test.raw_count), uint64_t(capacity) / test.multiplier) * test.multiplier);
				std::stable_sort(expected.begin(), expected.begin() + active, [](auto const& lhs, auto const& rhs)
				{
					return lhs.first < rhs.first;
				});
				fixture.Upload(fixture.m_keys.get(), keys);
				fixture.Upload(fixture.m_values.get(), values);
				fixture.Upload(sorter.m_sort[1].get(), scratch);
				fixture.Upload(sorter.m_payload[1].get(), scratch);

				// Capture two different counts in one command list to check argument-buffer read/write ordering.
				auto const counter_index = test.raw_count % 2 != 0 ? 5U : 1U;
				auto const counter_offset = counter_index * sizeof(uint32_t);
				std::array<uint32_t, 8> counter = { 0xffffffffU, 0x11111111U, 0x22222222U, 0x33333333U, 0x44444444U, 0x55555555U, 0x66666666U, 0x77777777U };
				counter[counter_index] = 0;
				fixture.Upload(fixture.m_counter.get(), counter);
				sorter.Sort(cmd_list, fixture.m_counter.get(), counter_offset, test.multiplier);
				counter[counter_index] = test.raw_count;
				fixture.Upload(fixture.m_counter.get(), counter);
				sorter.Bind(cmd_list, capacity, fixture.m_keys, fixture.m_values);
				if (test.cpu_known)
					sorter.Sort(cmd_list);
				else
					sorter.Sort(cmd_list, fixture.m_counter.get(), counter_offset, test.multiplier);

				// Readbacks are test-only; the sorter itself never transfers the counter or active length to the CPU.
				PR_EXPECT(cmd_list.ResState(fixture.m_counter.get()).Mip0State() == D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				auto sorted_keys = fixture.Readback(fixture.m_keys.get());
				auto sorted_values = fixture.Readback(fixture.m_values.get());
				auto scratch_keys = fixture.Readback(sorter.m_sort[1].get());
				auto scratch_values = fixture.Readback(sorter.m_payload[1].get());
				auto recorded_arguments = fixture.Readback(sorter.m_indirect_arguments.get());
				auto recorded_counter = fixture.Readback(fixture.m_counter.get());
				fixture.m_job.Run();

				// Compare every external element, including inactive data and guards beyond capacity.
				for (auto i = size_t{}; i != expected.size(); ++i)
				{
					PR_EXPECT(sorted_keys.ptr<uint32_t>()[i] == expected[i].first);
					PR_EXPECT(sorted_values.ptr<uint32_t>()[i] == expected[i].second);
					if (i != 0 && i < active && expected[i - 1].first == expected[i].first)
						PR_EXPECT(sorted_values.ptr<uint32_t>()[i - 1] < sorted_values.ptr<uint32_t>()[i]);
				}
				for (auto i = active; i != capacity; ++i)
				{
					PR_EXPECT(scratch_keys.ptr<uint32_t>()[i] == sentinel);
					PR_EXPECT(scratch_values.ptr<uint32_t>()[i] == sentinel);
				}
				PR_EXPECT(std::equal(counter.begin(), counter.end(), recorded_counter.ptr<uint32_t>()));
				if (!test.cpu_known)
					CheckArguments(recorded_arguments.ptr<uint32_t>(), capacity, test.raw_count, test.multiplier, 7680);

				// Equal capacities retain resource identities as well as their byte accounting.
				PR_EXPECT(sorter.m_size == capacity && sorter.AllocatedBufferBytes() == bytes);
				PR_EXPECT(sorter.m_sort[1].get() == sort_scratch && sorter.m_payload[1].get() == payload_scratch);
				PR_EXPECT(sorter.m_pass_histogram.get() == histogram && sorter.m_indirect_arguments.get() == arguments);
			}

			// Reject invalid inputs before recording GPU work, then release and verify the empty-capacity contract.
			PR_THROWS(sorter.Sort(cmd_list, nullptr), std::invalid_argument);
			PR_THROWS(sorter.Sort(cmd_list, fixture.m_counter.get(), 1), std::invalid_argument);
			PR_THROWS(sorter.Sort(cmd_list, fixture.m_counter.get(), 32), std::invalid_argument);
			PR_THROWS(sorter.Sort(cmd_list, fixture.m_counter.get(), UINT64_MAX), std::invalid_argument);
			PR_THROWS(sorter.Sort(cmd_list, fixture.m_counter.get(), 4, 0), std::invalid_argument);
			PR_THROWS(sorter.Bind(cmd_list, -1, nullptr, nullptr), std::out_of_range);
			PR_THROWS(sorter.Bind(cmd_list, int64_t(std::numeric_limits<int>::max()) + 1, nullptr, nullptr), std::out_of_range);
			PR_THROWS(sorter.Bind(cmd_list, capacity, nullptr, fixture.m_values), std::invalid_argument);
			PR_THROWS(sorter.Bind(cmd_list, capacity + 33, fixture.m_keys, fixture.m_values), std::invalid_argument);
			sorter.ReleaseBuffers();
			PR_EXPECT(sorter.AllocatedBufferBytes() == 0);
			sorter.Bind(cmd_list, 0, nullptr, nullptr);
			sorter.Sort(cmd_list);
			sorter.Sort(cmd_list, nullptr);
			sorter.InitPayload(cmd_list);
			sorter.Sort(std::span<uint32_t>{}, std::span<uint32_t>{}, fixture.m_job);
			PR_EXPECT(sorter.SortDispatchCount() == 0 && sorter.IndirectSortDispatchCount() == 0);
			PR_EXPECT(sorter.AllocatedBufferBytes() == 0);
		}

		// Constructor, Bind, Resize, and CPU-known Sort do not opt into the unused count-source capability.
		PRUnitTestMethod(CpuKnownStaysLazy, Extended)
		{
			auto fixture = Fixture(32);
			auto& sorter = fixture.m_sorter;
			auto& cmd_list = fixture.m_job.m_cmd_list;
			CheckIndirectUnused(sorter);
			fixture.m_job.Run();
			sorter.Resize(cmd_list, 32);
			CheckIndirectUnused(sorter);
			auto const bytes = sorter.AllocatedBufferBytes();

			// A stable CPU-known sort retains its original payload order without any indirect resources.
			std::array<uint32_t, 32> keys;
			std::array<uint32_t, 32> values;
			keys.fill(7);
			for (auto i = 0U; i != values.size(); ++i)
				values[i] = i;

			fixture.Upload(sorter.m_sort[0].get(), keys);
			fixture.Upload(sorter.m_payload[0].get(), values);
			sorter.Sort(cmd_list);
			CheckIndirectUnused(sorter);
			PR_EXPECT(sorter.AllocatedBufferBytes() == bytes);
			auto sorted_keys = fixture.Readback(sorter.m_sort[0].get());
			auto sorted_values = fixture.Readback(sorter.m_payload[0].get());
			fixture.m_job.Run();
			PR_EXPECT(std::equal(keys.begin(), keys.end(), sorted_keys.ptr<uint32_t>()));
			PR_EXPECT(std::equal(values.begin(), values.end(), sorted_values.ptr<uint32_t>()));

			// An empty-capacity counted call also leaves the optional capability cold.
			sorter.ReleaseBuffers();
			sorter.Bind(cmd_list, 0, nullptr, nullptr);
			sorter.Sort(cmd_list, nullptr);
			CheckIndirectUnused(sorter);
			PR_EXPECT(sorter.AllocatedBufferBytes() == 0);
		}

		// Validate multi-dimensional setup around 65,535 partitions without allocating multi-gigabyte key buffers.
		PRUnitTestMethod(LargeIndirectArguments, Extended)
		{
			auto fixture = Fixture(1);
			auto& sorter = fixture.m_sorter;
			auto& cmd_list = fixture.m_job.m_cmd_list;
			fixture.WarmIndirect();
			constexpr auto full_row = 65535U * 7680U;
			uint32_t const capacities[] = { 0, 1, 7679, 7680, 7681, full_row - 1, full_row, full_row + 1, full_row * 2 + 7681, UINT32_MAX };
			for (auto const capacity : capacities)
			{
				for (auto const multiplier : { 1U, 2U, UINT32_MAX })
				{
					// Only the setup kernel consumes these synthetic capacities; no sorting kernel accesses small test streams.
					std::array<uint32_t, 4> counter = { UINT32_MAX, 0, 0, 0 };
					fixture.Upload(fixture.m_counter.get(), counter);
					BarrierBatch barriers(cmd_list);
					barriers.Transition(fixture.m_counter.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
					barriers.Transition(sorter.m_indirect_arguments.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
					barriers.Commit();
					std::array<uint32_t, 4> constants = { capacity, multiplier, 7680, 0 };
					cmd_list.SetPipelineState(sorter.m_indirect_setup.m_pso.get());
					cmd_list.SetComputeRootSignature(sorter.m_indirect_setup.m_sig.get());
					cmd_list.SetComputeRoot32BitConstants(0, isize(constants), constants.data(), 0);
					cmd_list.SetComputeRootShaderResourceView(1, fixture.m_counter->GetGPUVirtualAddress());
					cmd_list.SetComputeRootUnorderedAccessView(2, sorter.m_indirect_arguments->GetGPUVirtualAddress());
					cmd_list.Dispatch(1, 1, 1);
					auto arguments = fixture.Readback(sorter.m_indirect_arguments.get());
					fixture.m_job.Run();
					CheckArguments(arguments.ptr<uint32_t>(), capacity, UINT32_MAX, multiplier, 7680);
				}
			}
		}
	};
}
#endif
