//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Usage:
//  Create a long-lived instance of the GpuRadixSort.
//  Resize it to the size of the data to be sorted.
//  Call the overload of Sort that suits your needs.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/compute/gpu.h"
#include "pr/view3d-12/compute/gpu_job.h"
#include "pr/view3d-12/compute/compute_pso.h"
#include "pr/view3d-12/compute/compute_step.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/pix.h"

namespace pr::rdr12::compute::gpu_radix_sort
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

		static constexpr int KeyBits = sizeof(Key)*8; // 32-bit keys atm
		static constexpr int RadixBits = 8;
		static constexpr int Radix = 1 << RadixBits; // The number of digit bins
		static constexpr int RadixPasses = KeyBits / RadixBits;
		static constexpr int MaxReadBack = 1 << 13;
		static constexpr int MaxDispatchDimension = 65535;
		static constexpr bool HasPayload = !std::is_same_v<Value, void>;
		static constexpr bool SortAscending = Ascending;

		struct EReg
		{
			inline static constexpr auto Constants = ECBufReg::b0;
			inline static constexpr auto Sort0 = EUAVReg::u0;
			inline static constexpr auto Sort1 = EUAVReg::u1;
			inline static constexpr auto Payload0 = EUAVReg::u2;
			inline static constexpr auto Payload1 = EUAVReg::u3;
			inline static constexpr auto GlobalHistogram = EUAVReg::u4;
			inline static constexpr auto PassHistogram = EUAVReg::u5;
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

		D3DPtr<ID3D12Resource> m_sort[2];
		D3DPtr<ID3D12Resource> m_payload[2];
		D3DPtr<ID3D12Resource> m_pass_histogram;
		D3DPtr<ID3D12Resource> m_global_histogram;
		D3DPtr<ID3D12Resource> m_error_count;

		TuningParams m_tuning;
		int64_t m_size;

		struct Result
		{
			GpuReadbackBuffer::Allocation keys;
			GpuReadbackBuffer::Allocation values;
		};

		explicit GpuRadixSort(Gpu& gpu, TuningParams const& tuning = {})
			: m_gpu(&gpu)
			, m_init()
			, m_init_payload()
			, m_sweep_up()
			, m_scan()
			, m_sweep_down()
			, m_sort()
			, m_payload()
			, m_pass_histogram()
			, m_global_histogram()
			, m_error_count()
			, m_tuning(tuning)
			, m_size()
		{
			auto compiler = ShaderCompiler{}
				.Source(resource::Read<char>(L"RADIX_SORT_HLSL", L"TEXT"))
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

		// Bind the given resources for sorting
		void Bind(CmdList& cmd_list, int64_t size, D3DPtr<ID3D12Resource> sort0, D3DPtr<ID3D12Resource> payload0)
		{
			// Create binding-size dependent buffers
			{
				ResDesc desc = ResDesc::Buf<Key>(size, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_sort[0] = sort0;
				m_sort[1] = m_gpu->CreateResource(desc, cmd_list, "RadixSort:sort1");
			}
			{
				using T = std::conditional_t<HasPayload, Value, int>;
				ResDesc desc = ResDesc::Buf<T>(HasPayload ? size : 1, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_payload[0] = payload0;
				m_payload[1] = m_gpu->CreateResource(desc, cmd_list, "RadixSort:payload1");
			}
			{
				auto partitions = DispatchCount(s_cast<int>(size), m_tuning.partition_size);
				ResDesc desc = ResDesc::Buf<Key>(s_cast<int64_t>(Radix) * partitions, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_pass_histogram = m_gpu->CreateResource(desc, cmd_list, "RadixSort:passHistBuffer");
			}

			m_size = size;
		}

		// Resize the GPU buffers in preparation for sorting 'size' elements
		void Resize(CmdList& cmd_list, int64_t size)
		{
			if (size == m_size)
				return;

			// Create sort-size independent buffers
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
				auto partitions = DispatchCount(s_cast<int>(size), m_tuning.partition_size);
				ResDesc desc = ResDesc::Buf<Key>(s_cast<int64_t>(Radix) * partitions, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_pass_histogram = m_gpu->CreateResource(desc, cmd_list, "RadixSort:passHistBuffer");
			}

			m_size = size;
		}

		// Sort 'values' by 'keys' in-place
		void Sort(std::span<Key> keys, std::span<Value> values, ComputeJob& job)
		{
			// Upload 'keys' and 'values' to the GPU and then sort them
			auto result = Sort(job.m_cmd_list, keys, values, job.m_upload, job.m_readback);

			// Do the sort and wait for it to complete
			job.Run();

			// Read back the results and update the input arrays
			{
				memcpy(keys.data(), result.keys.ptr<Key>(), result.keys.m_size);
			}
			if constexpr (HasPayload)
			{
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

			barriers.Transition(m_sort[0].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Transition(m_payload[0].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
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
			const auto thread_blocks = s_cast<uint32_t>(DispatchCount(s_cast<int>(m_size), m_tuning.partition_size));
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

					const auto full_blocks = s_cast<uint32_t>(thread_blocks / MaxDispatchDimension);
					if (full_blocks)
					{
						std::array<uint32_t, 4> t = { s_cast<uint32_t>(m_size), radix_shift, thread_blocks, 0 };
						cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
						cmd_list.Dispatch(MaxDispatchDimension, full_blocks, 1);
					}

					const auto partial_blocks = s_cast<uint32_t>(thread_blocks - full_blocks * MaxDispatchDimension);
					if (partial_blocks)
					{
						std::array<uint32_t, 4> t = { s_cast<uint32_t>(m_size), radix_shift, thread_blocks, (full_blocks << 1) | 1 };
						cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
						cmd_list.Dispatch(partial_blocks, 1, 1);
					}
				}

				barriers.UAV(m_pass_histogram.get());
				barriers.Commit();

				// Scan
				{
					std::array<uint32_t, 4> t = { 0, 0, thread_blocks, 0 };
					cmd_list.SetPipelineState(m_scan.m_pso.get());
					cmd_list.SetComputeRootSignature(m_scan.m_sig.get());
					cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
					cmd_list.SetComputeRootUnorderedAccessView(1, m_pass_histogram->GetGPUVirtualAddress());
					cmd_list.Dispatch(256, 1, 1);
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

					const auto full_blocks = s_cast<uint32_t>(thread_blocks / MaxDispatchDimension);
					if (full_blocks)
					{
						std::array<uint32_t, 4> t = { s_cast<uint32_t>(m_size), radix_shift, thread_blocks, 0 };
						cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
						cmd_list.Dispatch(MaxDispatchDimension, full_blocks, 1);
					}

					const auto partial_blocks = s_cast<uint32_t>(thread_blocks - full_blocks * MaxDispatchDimension);
					if (partial_blocks)
					{
						std::array<uint32_t, 4> t = { s_cast<uint32_t>(m_size), radix_shift, thread_blocks, (full_blocks << 1) | 1 };
						cmd_list.SetComputeRoot32BitConstants(0, isize(t), t.data(), 0);
						cmd_list.Dispatch(partial_blocks, 1, 1);
					}
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

		// Initialise the payload buffer to incrementing indices.
		// A common case when creating a lookup map
		void InitPayload(CmdList& cmd_list) const
		{
			const auto thread_blocks = s_cast<uint32_t>(DispatchCount(s_cast<int>(m_size), m_tuning.partition_size));

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
	};
}
