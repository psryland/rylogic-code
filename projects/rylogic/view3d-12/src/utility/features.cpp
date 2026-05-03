//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/features.h"

namespace pr::rdr12
{
	FeatureSupport::FeatureSupport()
		: m_device()
		, MaxFeatureLevel()
		, Options()
		, Options1()
		, Options2()
		, Options3()
		, Options4()
		, Options5()
		, Options6()
		, Options7()
		, Options8()
		, Options9()
		, GPUVASupport()
		, ShaderModel()
		, ProtectedResourceSessionSupport()
		, RootSignature()
		, Architecture1()
		, ShaderCache()
		, CommandQueuePriority()
		, ExistingHeaps()
		, Serialization()
		, CrossNode()
		#if _WIN32_WINNT > _WIN32_WINNT_WIN10
		, Displayable()
		, Options10()
		, Options11()
		, Options12()
		#endif
	{}
	FeatureSupport::FeatureSupport(ID3D12Device* device)
		:FeatureSupport()
	{
		Read(device);
	}
	void FeatureSupport::Read(ID3D12Device* device)
	{
		m_device = device;

		// Initialize static feature support data structures
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &Options, sizeof(Options))))
		{
			Options.DoublePrecisionFloatShaderOps = false;
			Options.OutputMergerLogicOp = false;
			Options.MinPrecisionSupport = D3D12_SHADER_MIN_PRECISION_SUPPORT_NONE;
			Options.TiledResourcesTier = D3D12_TILED_RESOURCES_TIER_NOT_SUPPORTED;
			Options.ResourceBindingTier = static_cast<D3D12_RESOURCE_BINDING_TIER>(0);
			Options.PSSpecifiedStencilRefSupported = false;
			Options.TypedUAVLoadAdditionalFormats = false;
			Options.ROVsSupported = false;
			Options.ConservativeRasterizationTier = D3D12_CONSERVATIVE_RASTERIZATION_TIER_NOT_SUPPORTED;
			Options.MaxGPUVirtualAddressBitsPerResource = 0;
			Options.StandardSwizzle64KBSupported = false;
			Options.CrossNodeSharingTier = D3D12_CROSS_NODE_SHARING_TIER_NOT_SUPPORTED;
			Options.CrossAdapterRowMajorTextureSupported = false;
			Options.VPAndRTArrayIndexFromAnyShaderFeedingRasterizerSupportedWithoutGSEmulation = false;
			Options.ResourceHeapTier = static_cast<D3D12_RESOURCE_HEAP_TIER>(0);
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS1, &Options1, sizeof(Options1))))
		{
			Options1.WaveOps = false;
			Options1.WaveLaneCountMax = 0;
			Options1.WaveLaneCountMin = 0;
			Options1.TotalLaneCount = 0;
			Options1.ExpandedComputeResourceStates = 0;
			Options1.Int64ShaderOps = 0;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS2, &Options2, sizeof(Options2))))
		{
			Options2.DepthBoundsTestSupported = false;
			Options2.ProgrammableSamplePositionsTier = D3D12_PROGRAMMABLE_SAMPLE_POSITIONS_TIER_NOT_SUPPORTED;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS3, &Options3, sizeof(Options3))))
		{
			Options3.CopyQueueTimestampQueriesSupported = false;
			Options3.CastingFullyTypedFormatSupported = false;
			Options3.WriteBufferImmediateSupportFlags = D3D12_COMMAND_LIST_SUPPORT_FLAG_NONE;
			Options3.ViewInstancingTier = D3D12_VIEW_INSTANCING_TIER_NOT_SUPPORTED;
			Options3.BarycentricsSupported = false;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS4, &Options4, sizeof(Options4))))
		{
			Options4.MSAA64KBAlignedTextureSupported = false;
			Options4.Native16BitShaderOpsSupported = false;
			Options4.SharedResourceCompatibilityTier = D3D12_SHARED_RESOURCE_COMPATIBILITY_TIER_0;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &Options5, sizeof(Options5))))
		{
			Options5.SRVOnlyTiledResourceTier3 = false;
			Options5.RenderPassesTier = D3D12_RENDER_PASS_TIER_0;
			Options5.RaytracingTier = D3D12_RAYTRACING_TIER_NOT_SUPPORTED;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS6, &Options6, sizeof(Options6))))
		{
			Options6.AdditionalShadingRatesSupported = false;
			Options6.PerPrimitiveShadingRateSupportedWithViewportIndexing = false;
			Options6.VariableShadingRateTier = D3D12_VARIABLE_SHADING_RATE_TIER_NOT_SUPPORTED;
			Options6.ShadingRateImageTileSize = 0;
			Options6.BackgroundProcessingSupported = false;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &Options7, sizeof(Options7))))
		{
			Options7.MeshShaderTier = D3D12_MESH_SHADER_TIER_NOT_SUPPORTED;
			Options7.SamplerFeedbackTier = D3D12_SAMPLER_FEEDBACK_TIER_NOT_SUPPORTED;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS8, &Options8, sizeof(Options8))))
		{
			Options8.UnalignedBlockTexturesSupported = false;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS9, &Options9, sizeof(Options9))))
		{
			Options9.MeshShaderPipelineStatsSupported = false;
			Options9.MeshShaderSupportsFullRangeRenderTargetArrayIndex = false;
			Options9.AtomicInt64OnGroupSharedSupported = false;
			Options9.AtomicInt64OnTypedResourceSupported = false;
			Options9.DerivativesInMeshAndAmplificationShadersSupported = false;
			Options9.WaveMMATier = D3D12_WAVE_MMA_TIER_NOT_SUPPORTED;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_GPU_VIRTUAL_ADDRESS_SUPPORT, &GPUVASupport, sizeof(GPUVASupport))))
		{
			GPUVASupport.MaxGPUVirtualAddressBitsPerProcess = 0;
			GPUVASupport.MaxGPUVirtualAddressBitsPerResource = 0;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_SHADER_CACHE, &ShaderCache, sizeof(ShaderCache))))
		{
			ShaderCache.SupportFlags = D3D12_SHADER_CACHE_SUPPORT_NONE;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_EXISTING_HEAPS, &ExistingHeaps, sizeof(ExistingHeaps))))
		{
			ExistingHeaps.Supported = false;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_CROSS_NODE, &CrossNode, sizeof(CrossNode))))
		{
			CrossNode.SharingTier = D3D12_CROSS_NODE_SHARING_TIER_NOT_SUPPORTED;
			CrossNode.AtomicShaderInstructions = false;
		}
		#if _WIN32_WINNT > _WIN32_WINNT_WIN10
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_DISPLAYABLE, &Displayable, sizeof(Displayable))))
		{
			Displayable.DisplayableTexture = false;
			Displayable.SharedResourceCompatibilityTier = D3D12_SHARED_RESOURCE_COMPATIBILITY_TIER_0;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS10, &Options10, sizeof(Options10))))
		{
			Options10.MeshShaderPerPrimitiveShadingRateSupported = false;
			Options10.VariableRateShadingSumCombinerSupported = false;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS11, &Options11, sizeof(Options11))))
		{
			Options11.AtomicInt64OnDescriptorHeapResourceSupported = false;
		}
		if (Failed(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS12, &Options12, sizeof(Options12))))
		{
			Options12.MSPrimitivesPipelineStatisticIncludesCulledPrimitives = D3D12_TRI_STATE_UNKNOWN;
			Options12.EnhancedBarriersSupported = false;
		}
		#endif

		// Find the highest shader model supported by the system
		{
			// Check support in descending order
			constexpr D3D_SHADER_MODEL versions[] =
			{
				D3D_HIGHEST_SHADER_MODEL,
				D3D_SHADER_MODEL_6_7,
				D3D_SHADER_MODEL_6_6,
				D3D_SHADER_MODEL_6_5,
				D3D_SHADER_MODEL_6_4,
				D3D_SHADER_MODEL_6_3,
				D3D_SHADER_MODEL_6_2,
				D3D_SHADER_MODEL_6_1,
				D3D_SHADER_MODEL_6_0,
				D3D_SHADER_MODEL_5_1,
			};
			for (auto i = 0; i != _countof(versions); ++i)
			{
				ShaderModel.HighestShaderModel = versions[i];
				auto hr = device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &ShaderModel, sizeof(D3D12_FEATURE_DATA_SHADER_MODEL));
				if (hr == E_INVALIDARG) continue;
				Check(hr);
				break;
			}
		}

		// Find the highest root signature supported
		{
			// Check support in descending order
			constexpr D3D_ROOT_SIGNATURE_VERSION versions[] =
			{
				D3D_ROOT_SIGNATURE_VERSION_1_1,
				D3D_ROOT_SIGNATURE_VERSION_1_0,
				D3D_ROOT_SIGNATURE_VERSION_1,
			};
			for (auto i = 0; i != _countof(versions); ++i)
			{
				RootSignature.HighestVersion = versions[i];
				auto hr = device->CheckFeatureSupport(D3D12_FEATURE_ROOT_SIGNATURE, &RootSignature, sizeof(D3D12_FEATURE_DATA_ROOT_SIGNATURE));
				if (hr == E_INVALIDARG) continue;
				Check(hr);
				break;
			}
		}

		// Find the highest feature level
		{
			// Check against a list of all feature levels present in d3dcommon.h
			constexpr D3D_FEATURE_LEVEL levels[] =
			{
				D3D_FEATURE_LEVEL_12_2,
				D3D_FEATURE_LEVEL_12_1,
				D3D_FEATURE_LEVEL_12_0,
				D3D_FEATURE_LEVEL_11_1,
				D3D_FEATURE_LEVEL_11_0,
				D3D_FEATURE_LEVEL_10_1,
				D3D_FEATURE_LEVEL_10_0,
				D3D_FEATURE_LEVEL_9_3,
				D3D_FEATURE_LEVEL_9_2,
				D3D_FEATURE_LEVEL_9_1,
				D3D_FEATURE_LEVEL_1_0_CORE,
			};

			D3D12_FEATURE_DATA_FEATURE_LEVELS FeatureLevel;
			FeatureLevel.NumFeatureLevels = _countof(levels);
			FeatureLevel.pFeatureLevelsRequested = levels;
			auto hr = device->CheckFeatureSupport(D3D12_FEATURE_FEATURE_LEVELS, &FeatureLevel, sizeof(D3D12_FEATURE_DATA_FEATURE_LEVELS));
			MaxFeatureLevel = Succeeded(hr) ? FeatureLevel.MaxSupportedFeatureLevel : static_cast<D3D_FEATURE_LEVEL>(0);
			if (hr != DXGI_ERROR_UNSUPPORTED) Check(hr);
		}
	}

	// Get Adapter properties
	FeatureSupport::AdapterInfo FeatureSupport::Adapter() const
	{
		auto luid = m_device->GetAdapterLuid();

		D3DPtr<IDXGIFactory4> factory;
		Check(CreateDXGIFactory2(0, __uuidof(IDXGIFactory4), (void**)factory.address_of()));

		D3DPtr<IDXGIAdapter1> adapter;
		Check(factory->EnumAdapterByLuid(luid, __uuidof(IDXGIAdapter1), (void**)adapter.address_of()));

		FeatureSupport::AdapterInfo desc = {};
		Check(adapter->GetDesc1(&desc));
		return desc;
	}

	// True if the adapter is a warp device
	bool FeatureSupport::AdapterInfo::IsWarpDevice() const
	{
		return
			(Flags & DXGI_ADAPTER_FLAG_SOFTWARE) == DXGI_ADAPTER_FLAG_SOFTWARE ||
			_wcsicmp(Description, L"Microsoft Basic Render Driver") == 0;
	}

	// Return the format support
	FeatureSupport::FormatData FeatureSupport::Format(DXGI_FORMAT format) const
	{
		FeatureSupport::FormatData support = { format };
		Check(m_device->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &support, sizeof(support)));
		return support;
	}

	// Return info about the GPU
	FeatureSupport::ComputeInfo FeatureSupport::Compute() const
	{
		return FeatureSupport::ComputeInfo{
			.HighestShaderModel = ShaderModel.HighestShaderModel,
			.SIMDWidth = Options1.WaveLaneCountMin,
			.SIMDMaxWidth = Options1.WaveLaneCountMax,
			.SIMDLaneCount = Options1.TotalLaneCount,
			.SupportsWaveIntrinsics = Options1.WaveOps != 0,
			.Supports16BitTypes = Options4.Native16BitShaderOpsSupported != 0,
		};
	}

	// Return the string representation of the highest shader model supported
	std::string_view FeatureSupport::ComputeInfo::SupportedShaderModel() const
	{
		switch (HighestShaderModel)
		{
			case D3D_SHADER_MODEL_5_1: return "cs_5_1";
			case D3D_SHADER_MODEL_6_0: return "cs_6_0";
			case D3D_SHADER_MODEL_6_1: return "cs_6_1";
			case D3D_SHADER_MODEL_6_2: return "cs_6_2";
			case D3D_SHADER_MODEL_6_3: return "cs_6_3";
			case D3D_SHADER_MODEL_6_4: return "cs_6_4";
			case D3D_SHADER_MODEL_6_5: return "cs_6_5";
			case D3D_SHADER_MODEL_6_6: return "cs_6_6";
			case D3D_SHADER_MODEL_6_7: return "cs_6_7";
			default: return "unknown";
		}
	}

	// Check the given format is supported
	bool FeatureSupport::FormatData::Check(D3D12_FORMAT_SUPPORT1 format) const
	{
		return AllSet(Support1, format);
	}
	bool FeatureSupport::FormatData::Check(D3D12_FORMAT_SUPPORT2 format) const
	{
		return AllSet(Support2, format);
	}
	bool FeatureSupport::FormatData::CheckSRV() const
	{
		return Check(D3D12_FORMAT_SUPPORT1_SHADER_LOAD);
	}
	bool FeatureSupport::FormatData::CheckUAV() const
	{
		return
			Check(D3D12_FORMAT_SUPPORT1_TYPED_UNORDERED_ACCESS_VIEW) &&
			Check(D3D12_FORMAT_SUPPORT2_UAV_TYPED_LOAD) &&
			Check(D3D12_FORMAT_SUPPORT2_UAV_TYPED_STORE);
	}
	bool FeatureSupport::FormatData::CheckRTV() const
	{
		return Check(D3D12_FORMAT_SUPPORT1_RENDER_TARGET);
	}
	bool FeatureSupport::FormatData::CheckDSV() const
	{
		return Check(D3D12_FORMAT_SUPPORT1_DEPTH_STENCIL);
	}
}

