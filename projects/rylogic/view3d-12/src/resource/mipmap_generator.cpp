//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/resource/mipmap_generator.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/gpu_sync.h"
#include "pr/view3d-12/utility/utility.h"
#include "pr/view3d-12/utility/root_signature.h"
#include "pr/view3d-12/compute/compute_pso.h"

namespace pr::rdr12
{
	constexpr int HeapCapacityView = 256;
	struct EReg
	{
		inline static constexpr auto Constants = hlsl::ECBufReg::b0;
		inline static constexpr auto SrcTexture = hlsl::ESRVReg::t0;
		inline static constexpr auto DstTexture = hlsl::EUAVReg::u0;
	};
	enum class EMipMapParam { Constants, SrcTexture, DstTexture };
	enum class EMipMapSamp { Samp0 };

	MipMapGenerator::MipMapGenerator(Renderer& rdr, GpuSync& gsync, GfxCmdList& cmd_list)
		:m_rdr(rdr)
		,m_gsync(gsync)
		,m_cmd_list(cmd_list)
		,m_keep_alive(m_gsync)
		,m_view_heap(HeapCapacityView, m_gsync)
		,m_mipmap_sig()
		,m_mipmap_pso()
		,m_flush_required()
	{
		auto device = rdr.D3DDevice();

		// Create a root signature for the MipMap generator compute shader
		m_mipmap_sig = RootSig(ERootSigFlags::ComputeOnly | ERootSigFlags::AllowInputAssemblerInputLayout)
			.U32(EReg::Constants, 2)
			.SRV(EReg::SrcTexture, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
			.UAV(EReg::DstTexture, 1, D3D12_SHADER_VISIBILITY_ALL, D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE)
			.Samp(D3D12_STATIC_SAMPLER_DESC{
				.Filter = D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT,
				.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
				.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
				.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
				.MipLODBias = 0.0f,
				.MaxAnisotropy = 0,
				.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER,
				.BorderColor = D3D12_STATIC_BORDER_COLOR_OPAQUE_BLACK,
				.MinLOD = 0.0f,
				.MaxLOD = D3D12_FLOAT32_MAX,
				.ShaderRegister = 0,
				.RegisterSpace = 0,
				.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL,
				})
			.Create(device, "MipMapGenSig");

		// Create pipeline state object for the compute shader using the root signature.
		m_mipmap_pso = ComputePSO(m_mipmap_sig.get(), shader_code::mipmap_generator_cs)
			.Create(device, "MipMapGenPSO");
	}

	// Generate mip maps for a texture
	void MipMapGenerator::Generate(ID3D12Resource* texture, int mip_first, int mip_count)
	{
		// Get the description of the texture
		auto desc = texture->GetDesc();
		auto dim = iv2(s_cast<int>(desc.Width), s_cast<int>(desc.Height));
		mip_count = std::min(MipCount(dim) - mip_first, mip_count);

		// Mip 0 is the texture itself, we're not generating that
		if (mip_first <= 0)
			throw std::runtime_error("'mip_first' should be >= 1");

		// Check the resource has enough sub-resource space for the mips
		if (desc.MipLevels != 0 && desc.MipLevels < mip_first + mip_count)
			throw std::runtime_error("Resource does not have enough mip levels");

		// Only non-multi-sampled 2D textures are supported
		if (desc.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D ||
			desc.DepthOrArraySize != 1 ||
			desc.SampleDesc.Count > 1)
			throw std::runtime_error("Unsupported resource. Mip-map generation only supports 2D textures");

		// If the resource already supports UAV descriptors, then generate mip-maps in place.
		auto support = m_rdr.Features().Format(desc.Format);
		if (support.CheckUAV() && AllSet(desc.Flags, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS))
		{
			GenerateCore(texture, mip_first, mip_count);
			m_flush_required = true;
			return;
		}

		// Otherwise, we need to generate the mip-maps in a staging resource.
		auto device = m_rdr.D3DDevice();
		auto next_sync_point = m_gsync.NextSyncPoint();
		auto initial_res_state = m_cmd_list.ResState(texture).Mip0State();

		// Describe a resource the same as 'texture' but with UAV support and not RT/DS support.
		D3D12_RESOURCE_DESC staging_desc = desc;
		staging_desc.Flags = SetBits(staging_desc.Flags, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, true);
		staging_desc.Flags = SetBits(staging_desc.Flags, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL, false);

		// Describe a UAV compatible resource that is used to perform mip-mapping.
		// The flags for the UAV description must match that of the staging description in order to allow data inheritance between the aliased textures.
		auto uav_desc = staging_desc;
		uav_desc.Format = ToUAVCompatable(desc.Format);

		// Create a heap to contain the alias of the staging and UAV resource.
		D3DPtr<ID3D12Heap> heap;
		auto descs = { staging_desc, uav_desc };
		auto info = device->GetResourceAllocationInfo(0, s_cast<UINT>(descs.size()), descs.begin());
		D3D12_HEAP_DESC heap_desc = {
			.SizeInBytes = info.SizeInBytes,
			.Properties = {
				.Type = D3D12_HEAP_TYPE_DEFAULT,
				.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
				.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
				.CreationNodeMask = 0,
				.VisibleNodeMask = 0,
			},
			.Alignment = 0, //info.Alignment,
			.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES,
		};
		Check(device->CreateHeap(&heap_desc, __uuidof(ID3D12Heap), (void**)heap.address_of()));
		DebugName(heap, "MipMapGenHeap");
		m_keep_alive.Add(heap.get(), next_sync_point);

		// Create a placed resource that matches the description of the original resource.
		// The original texture is copied to this resource, which is then aliased as a UAV resource.
		D3DPtr<ID3D12Resource> staging;
		Check(device->CreatePlacedResource(heap.get(), 0, &staging_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource), (void**)staging.address_of()));
		DebugName(staging, "MipMapStagingAliasRes");
		m_cmd_list.ResState(staging.get()).Apply(D3D12_RESOURCE_STATE_COMMON);
		m_keep_alive.Add(staging.get(), next_sync_point);

		// Create a UAV resource that is an alias of 'staging'.
		D3DPtr<ID3D12Resource> uav_resource;
		Check(device->CreatePlacedResource(heap.get(), 0, &uav_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource), (void**)uav_resource.address_of()));
		DebugName(uav_resource, "MipMapStagingUAVRes");
		m_cmd_list.ResState(uav_resource.get()).Apply(D3D12_RESOURCE_STATE_COMMON);
		m_keep_alive.Add(uav_resource.get(), next_sync_point);

		// Add an aliasing barrier to say that 'staging' is the currently valid resource.
		// Aliasing textures must have compatible resource states in order to inherit data.
		BarrierBatch barriers(m_cmd_list);
		barriers.Aliasing(nullptr, staging.get());
		barriers.Transition(staging.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		barriers.Transition(texture, D3D12_RESOURCE_STATE_COPY_SOURCE);
		barriers.Commit();

		// Copy the original resource into the staging resource.
		m_cmd_list.CopyResource(staging.get(), texture);

		// Make the UAV resource active. UAV inherits the data from 'staging'
		barriers.Aliasing(staging.get(), uav_resource.get());
		barriers.Commit();

		// Generate mips in the UAV resource.
		GenerateCore(uav_resource.get(), mip_first, mip_count);

		// Make the 'staging' resource active again. 'Staging' inherits data from 'uav_resource'
		barriers.Aliasing(uav_resource.get(), staging.get());
		barriers.Commit();

		barriers.Transition(staging.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		barriers.Transition(texture, D3D12_RESOURCE_STATE_COPY_DEST);
		barriers.Commit();

		// Copy the staging resource back to the original resource.
		m_cmd_list.CopyResource(texture, staging.get());

		// Transition the texture back to the initial state
		barriers.Transition(texture, initial_res_state);
		barriers.Commit();

		m_flush_required = true;
	}

	// Generate mip maps for a resource that supports UAV
	void MipMapGenerator::GenerateCore(ID3D12Resource* uav_resource, int mip_first, int mip_count)
	{
		auto desc = uav_resource->GetDesc();
		auto dim = iv2(s_cast<int>(desc.Width), s_cast<int>(desc.Height));

		// Set root signature and PSO
		m_cmd_list.SetComputeRootSignature(m_mipmap_sig.get());
		m_cmd_list.SetPipelineState(m_mipmap_pso.get());

		// Set the descriptor heap
		auto heaps = { m_view_heap.get() };
		m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });

		// Prepare the SRV/UAV view descriptions (changed for each mip).
		D3D12_SHADER_RESOURCE_VIEW_DESC srv_desc = {
			.Format = desc.Format,
			.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
			.Texture2D = {
				.MostDetailedMip = 0,
				.MipLevels = desc.MipLevels,
				.PlaneSlice = 0,
				.ResourceMinLODClamp = 0.f,
			},
		};
		D3D12_UNORDERED_ACCESS_VIEW_DESC uav_desc = {
			.Format = desc.Format,
			.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D,
			.Texture2D = {
				.MipSlice = 0,
				.PlaneSlice = 0,
			},
		};

		// Loop through the mip-maps copying from the bigger mip-map to the smaller one with down-sampling in a compute shader
		BarrierBatch barriers(m_cmd_list);
		for (auto mip = mip_first; mip != mip_first + mip_count; ++mip)
		{
			// Get the dimensions at 'mip'
			auto dst_w = std::max(dim.x >> mip, 1);
			auto dst_h = std::max(dim.y >> mip, 1);

			// Pass the destination texture pixel size to the shader as constants
			m_cmd_list.SetComputeRoot32BitConstant(EMipMapParam::Constants, 1.0f / dst_w, 0);
			m_cmd_list.SetComputeRoot32BitConstant(EMipMapParam::Constants, 1.0f / dst_h, 1);

			// Create shader resource view for the source texture in the descriptor heap
			auto srv = m_view_heap.Add(uav_resource, srv_desc);
			m_cmd_list.SetComputeRootDescriptorTable(EMipMapParam::SrcTexture, srv);
			barriers.Transition(uav_resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, s_cast<uint32_t>(mip - 1));

			// Create unordered access view for the destination texture in the descriptor heap
			uav_desc.Texture2D.MipSlice = mip;
			auto uav = m_view_heap.Add(uav_resource, uav_desc);
			m_cmd_list.SetComputeRootDescriptorTable(EMipMapParam::DstTexture, uav);
			barriers.Transition(uav_resource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, s_cast<uint32_t>(mip));

			barriers.Commit();

			// Dispatch the compute shader with one thread per 8x8 pixels
			m_cmd_list.Dispatch((dst_w + 7) / 8, (dst_h + 7) / 8, 1);

			// Wait for all accesses to the destination texture UAV to be finished before generating
			// the next mip-map, as it will be the source texture for the next mip-map
			barriers.UAV(uav_resource);
		}
	}
}





// Generating mip maps... 
#if 0
//MipMapTextures is an array containing texture objects that need mip-maps to be generated. It needs a texture resource with mip-maps in D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE state.
//Textures are expected to be POT and in a format supporting unordered access, as well as the D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS set during creation.
//_device is the ID3D12Device
//GetNewCommandList() is supposed to return a new command list in recording state
//SubmitCommandList(commandList) is supposed to submit the command list to the command queue
//MipMapComputeShader is an ID3DBlob of the compiled mip-map compute shader
void D3D12Renderer::Createmip - maps()
{
	//Union used for shader constants
	struct DWParam
	{
		DWParam(FLOAT f) : Float(f) {}
		DWParam(UINT u) : Uint(u) {}

		void operator= (FLOAT f) { Float = f; }
		void operator= (UINT u) { Uint = u; }

		union
		{
			FLOAT Float;
			UINT Uint;
		};
	};

	//Calculate heap size
	uint32 requiredHeapSize = 0;
	MipMapTextures->Enumerate<D3D12Texture>([&](D3D12Texture* texture, size_t index, bool& stop) {
		if (texture->mip - maps > 1)
			requiredHeapSize += texture->mip - maps - 1;
		});

	//No heap size, means that there was either no texture or none that requires any mip-maps
	if (requiredHeapSize == 0)
	{
		MipMapTextures->RemoveAllObjects();
		return;
	}

	//The compute shader expects 2 floats, the source texture and the destination texture
	CD3DX12_DESCRIPTOR_RANGE srvCbvRanges[2];
	CD3DX12_ROOT_PARAMETER rootParameters[3];
	srvCbvRanges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0);
	srvCbvRanges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0, 0);
	rootParameters[0].InitAsConstants(2, 0);
	rootParameters[1].InitAsDescriptorTable(1, &srvCbvRanges[0]);
	rootParameters[2].InitAsDescriptorTable(1, &srvCbvRanges[1]);

	//Static sampler used to get the linearly interpolated color for the mip-maps
	D3D12_STATIC_SAMPLER_DESC samplerDesc = {};
	samplerDesc.Filter = D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT;
	samplerDesc.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
	samplerDesc.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
	samplerDesc.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
	samplerDesc.MipLODBias = 0.0f;
	samplerDesc.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
	samplerDesc.MinLOD = 0.0f;
	samplerDesc.MaxLOD = D3D12_FLOAT32_MAX;
	samplerDesc.MaxAnisotropy = 0;
	samplerDesc.BorderColor = D3D12_STATIC_BORDER_COLOR_OPAQUE_BLACK;
	samplerDesc.ShaderRegister = 0;
	samplerDesc.RegisterSpace = 0;
	samplerDesc.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

	//Create the root signature for the mip-map compute shader from the parameters and sampler above
	ID3DBlob* signature;
	ID3DBlob* error;
	CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;
	rootSignatureDesc.Init(_countof(rootParameters), rootParameters, 1, &samplerDesc, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);
	D3D12SerializeRootSignature(&rootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error);
	ID3D12RootSignature* mip - mapRootSignature;
	_device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&mip - mapRootSignature));

	//Create the descriptor heap with layout: source texture - destination texture
	D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
	heapDesc.NumDescriptors = 2 * requiredHeapSize;
	heapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	heapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	ID3D12DescriptorHeap* descriptorHeap;
	_device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&descriptorHeap));
	UINT descriptorSize = _device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

	//Create pipeline state object for the compute shader using the root signature.
	D3D12_COMPUTE_PIPELINE_STATE_DESC psoDesc = {};
	psoDesc.pRootSignature = mip - mapRootSignature;
	psoDesc.CS = { reinterpret_cast<UINT8*>(MipMapComputeShader->GetBufferPointer()), MipMapComputeShader->GetBufferSize() };
	ID3D12PipelineState* psomip - maps;
	_device->CreateComputePipelineState(&psoDesc, IID_PPV_ARGS(&psomip - maps));


	//Prepare the shader resource view description for the source texture
	D3D12_SHADER_RESOURCE_VIEW_DESC srcTextureSRVDesc = {};
	srcTextureSRVDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srcTextureSRVDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;

	//Prepare the unordered access view description for the destination texture
	D3D12_UNORDERED_ACCESS_VIEW_DESC destTextureUAVDesc = {};
	destTextureUAVDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;

	//Get a new empty command list in recording state
	ID3D12GraphicsCommandList* commandList = GetNewCommandList();

	//Set root signature, pso and descriptor heap
	commandList->SetComputeRootSignature(mip - mapRootSignature);
	commandList->SetPipelineState(psomip - maps);
	commandList->SetDescriptorHeaps(1, &descriptorHeap);

	//CPU handle for the first descriptor on the descriptor heap, used to fill the heap
	CD3DX12_CPU_DESCRIPTOR_HANDLE currentCPUHandle(descriptorHeap->GetCPUDescriptorHandleForHeapStart(), 0, descriptorSize);

	//GPU handle for the first descriptor on the descriptor heap, used to initialize the descriptor tables
	CD3DX12_GPU_DESCRIPTOR_HANDLE currentGPUHandle(descriptorHeap->GetGPUDescriptorHandleForHeapStart(), 0, descriptorSize);

	MipMapTextures->Enumerate<D3D12Texture>([&](D3D12Texture* texture, size_t index, bool& stop) {
		//Skip textures without mip-maps
		if (texture->mip - maps <= 1)
			return;

		//Transition from pixel shader resource to unordered access
		commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(texture->_resource, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS));

		//Loop through the mip-maps copying from the bigger mip-map to the smaller one with downsampling in a compute shader
		for (uint32_t TopMip = 0; TopMip < texture->mip - maps - 1; TopMip++)
		{
			//Get mip-map dimensions
			uint32_t dstWidth = std::max(texture->width >> (TopMip + 1), 1);
			uint32_t dstHeight = std::max(texture->height >> (TopMip + 1), 1);

			//Create shader resource view for the source texture in the descriptor heap
			srcTextureSRVDesc.Format = texture->_format;
			srcTextureSRVDesc.Texture2D.MipLevels = 1;
			srcTextureSRVDesc.Texture2D.MostDetailedMip = TopMip;
			_device->CreateShaderResourceView(texture->_resource, &srcTextureSRVDesc, currentCPUHandle);
			currentCPUHandle.Offset(1, descriptorSize);

			//Create unordered access view for the destination texture in the descriptor heap
			destTextureUAVDesc.Format = texture->_format;
			destTextureUAVDesc.Texture2D.MipSlice = TopMip + 1;
			_device->CreateUnorderedAccessView(texture->_resource, nullptr, &destTextureUAVDesc, currentCPUHandle);
			currentCPUHandle.Offset(1, descriptorSize);

			//Pass the destination texture pixel size to the shader as constants
			commandList->SetComputeRoot32BitConstant(0, DWParam(1.0f / dstWidth).Uint, 0);
			commandList->SetComputeRoot32BitConstant(0, DWParam(1.0f / dstHeight).Uint, 1);

			//Pass the source and destination texture views to the shader via descriptor tables
			commandList->SetComputeRootDescriptorTable(1, currentGPUHandle);
			currentGPUHandle.Offset(1, descriptorSize);
			commandList->SetComputeRootDescriptorTable(2, currentGPUHandle);
			currentGPUHandle.Offset(1, descriptorSize);

			//Dispatch the compute shader with one thread per 8x8 pixels
			commandList->Dispatch(std::max(dstWidth / 8, 1u), std::max(dstHeight / 8, 1u), 1);

			//Wait for all accesses to the destination texture UAV to be finished before generating the next mip-map, as it will be the source texture for the next mip-map
			commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(texture->_resource));
		}

		//When done with the texture, transition it's state back to be a pixel shader resource
		commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(texture->_resource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE));
		});

	//Close and submit the command list
	commandList->Close();
	SubmitCommandList(commandList);

	MipMapTextures->RemoveAllObjects();
}

// Shader
{
	Texture2D<float4> SrcTexture : register(t0);
	RWTexture2D<float4> DstTexture : register(u0);
	SamplerState BilinearClamp : register(s0);

	cbuffer CB : register(b0)
	{
		float2 TexelSize;	// 1.0 / destination dimension
	}

	[numthreads(8, 8, 1)]
	void Generatemip - maps(uint3 DTid : SV_DispatchThreadID)
	{
		//DTid is the thread ID * the values from numthreads above and in this case correspond to the pixels location in number of pixels.
		//As a result texcoords (in 0-1 range) will point at the center between the 4 pixels used for the mip-map.
		float2 texcoords = TexelSize * (DTid.xy + 0.5);

		//The samplers linear interpolation will mix the four pixel values to the new pixels color
		float4 color = SrcTexture.SampleLevel(BilinearClamp, texcoords, 0);

		//Write the final color into the destination texture.
		DstTexture[DTid.xy] = color;
	}
}
#endif