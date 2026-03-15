// HyperPose Tools
// Copyright (c) 2025
#include "pr/view3d-12/compute/gpu.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/update_resource.h"
#include "pr/view3d-12/utility/utility.h"
#include "pr/view3d-12/utility/pix.h"

namespace pr::rdr12
{
	template <D3D12_COMMAND_LIST_TYPE ListType>
	Gpu<ListType>::Gpu(ID3D12Device4* existing_device, ID3D12CommandQueue* existing_queue)
		: m_device(existing_device, true)
		, m_cmd_queue(existing_queue, true)
		, m_gsync()
		, m_cmd_alloc_pool(m_gsync)
		, m_cmd_list_pool(m_gsync)
		, m_upload_buffer(m_gsync, 1ULL * 1024 * 1024)
	{
		if (existing_device == nullptr)
		{
			// If PIX is enabled, load the GPU capturer dll (no-op if HP_PIX_ENABLED==0)
			pix::LoadLatestWinPixGpuCapturer();

			// Enable the D3D Debug layer (must be before create device)
			if constexpr (IsDebug)
			{
				D3DPtr<ID3D12Debug> dbg;
				Check(D3D12GetDebugInterface(__uuidof(ID3D12Debug), (void**)dbg.address_of()));
				dbg->EnableDebugLayer();

				#define PR_RDR12_GPU_VALIDATION 0
				#if PR_RDR12_GPU_VALIDATION
				{
					#pragma message(HP_FILE_LINE "WARNING: ************************************************** GPU Validation enabled")

					D3DPtr<ID3D12Debug1> dbg1;
					Check(dbg->QueryInterface<ID3D12Debug1>(dbg1.address_of()));
					dbg1->SetEnableGPUBasedValidation(true);
				}
				#endif
			}

			// Create the d3d device
			Check(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, __uuidof(ID3D12Device4), (void**)m_device.address_of()));

			// Force break on D3D Errors
			if constexpr (IsDebug)
			{
				D3DPtr<ID3D12InfoQueue> info;
				Check(m_device->QueryInterface<ID3D12InfoQueue>(info.address_of()));
				Check(info->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_CORRUPTION, TRUE));
				Check(info->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_ERROR, TRUE));
				Check(info->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_WARNING, TRUE));
			}
		}

		// Create the command queue
		if (m_cmd_queue == nullptr)
		{
			D3D12_COMMAND_QUEUE_DESC queue_desc = {
				.Type = ListType,
				.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL,
				.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE,
				.NodeMask = 0,
			};
			Check(m_device->CreateCommandQueue(&queue_desc, __uuidof(ID3D12CommandQueue), (void**)m_cmd_queue.address_of()));
		}

		// Check feature support
		D3D12_FEATURE_DATA_SHADER_MODEL ShaderModel = { .HighestShaderModel = D3D_SHADER_MODEL_5_1 };
		Check(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &ShaderModel, sizeof(D3D12_FEATURE_DATA_SHADER_MODEL)));
		if (ShaderModel.HighestShaderModel < D3D_SHADER_MODEL_5_1)
			throw std::runtime_error("DirectX device does not support Compute Shaders 4.x");

		// Initialise the fence
		m_gsync.Init(m_device.get());
	}

	// Allocate a DX resource
	template <D3D12_COMMAND_LIST_TYPE ListType>
	D3DPtr<ID3D12Resource> Gpu<ListType>::CreateResource(ResDesc const& desc, CmdList<ListType>& cmd_list, std::string_view name)
	{
		D3DPtr<ID3D12Resource> res;
		auto has_init_data = !desc.Data.empty();

		if (desc.Width == 0)
			return res;

		// Buffer resources specify the Width as the size in bytes, even though for textures width is the pixel count.
		ResDesc rd = desc;
		if (desc.Dimension == D3D12_RESOURCE_DIMENSION_BUFFER)
			rd.Width *= desc.ElemStride;

		// Create a GPU visible resource that will hold the created buffer/texture/verts/indices/etc.
		// Create in the COMMON state to prevent a D3D12 warning "Buffers are effectively created in state D3D12_RESOURCE_STATE_COMMON"
		// COMMON state is implicitly promoted to the first state transition.
		assert(desc.Check());
		Check(m_device->CreateCommittedResource(
			&desc.HeapProps, desc.HeapFlags, &rd, D3D12_RESOURCE_STATE_COMMON,
			desc.ClearValue ? &*desc.ClearValue : nullptr,
			__uuidof(ID3D12Resource), (void**)res.address_of()));

		// Assume common state until the resource is initialised
		DefaultResState(res.get(), D3D12_RESOURCE_STATE_COMMON);
		DebugName(res, name);

		// If initialisation data is provided, initialise using an UploadBuffer
		if (has_init_data)
		{
			// Copy the initialisation data for each array slice into the resource
			auto array_length = desc.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE3D ? 1 : s_cast<int>(desc.DepthOrArraySize);
			for (auto i = 0; i != array_length; ++i)
			{
				// Note: here 'desc.Data' is an array of mip-level-zero images.
				// The span of images expected by 'UpdateSubresource' is for each mip level.
				UpdateSubresourceScope map(cmd_list, m_upload_buffer, res.get(), i, 0, 1, desc.DataAlignment);
				map.Write(desc.Data[i], AllSet(desc.MiscFlags, ResDesc::EMiscFlags::PartialInitData));
				map.Commit(EFinalState::Override, desc.DefaultState);
			}
			
			// Generate mip maps for the texture (if needed)
			// 'm_mipmap_gen' should use the same cmd-list as the resource manager, so that mips are generated as
			// textures are created. Remember cmd-lists are executed serially.
			if (desc.MipLevels != 1)
				throw std::runtime_error("Mip map generation not supported");
		}

		// Set the true default state for the resource now that it's initialised
		DefaultResState(res.get(), desc.DefaultState);

		// Transition the resource to the default state
		BarrierBatch barriers(cmd_list);
		barriers.Transition(res.get(), desc.DefaultState);
		barriers.Commit();
		return res;
	}

	template class Gpu<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	template class Gpu<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
}
