//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/utility/utility.h"
#include "pr/compute/shaders/shader_cache.h"

namespace pr::compute
{
	struct ComputePSO : D3D12_COMPUTE_PIPELINE_STATE_DESC
	{
		// Notes:
		//  - This type helps build a compute pipeline state object

		// Optional cache that reuses driver-compiled pipelines across runs. Not owned.
		shader_cache::IShaderCache* m_cache;
		
		ComputePSO(ID3D12RootSignature* sig, std::span<uint8_t const> bytecode)
			: D3D12_COMPUTE_PIPELINE_STATE_DESC{
				.pRootSignature = sig,
				.CS = {
					.pShaderBytecode = bytecode.data(),
					.BytecodeLength = bytecode.size(),
				},
				.NodeMask = 0,
				.CachedPSO = D3D12_CACHED_PIPELINE_STATE{},
				.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
			}
			, m_cache()
		{}

		// Use 'cache' to reuse the driver-compiled pipeline across runs. 'cache' must outlive the call to 'Create'.
		template <typename Self> Self&& Cache(this Self&& self, shader_cache::IShaderCache* cache)
		{
			self.m_cache = cache;
			return std::forward<Self>(self);
		}

		// Create the compute pipeline state object. 'name' also identifies the pipeline in the cache, if one is set.
		D3DPtr<ID3D12PipelineState> Create(ID3D12Device* device, char const* name)
		{
			// The cache is keyed on 'name' plus the shader bytecode, so names must be unique per root signature.
			auto pso = m_cache != nullptr
				? m_cache->GetOrCreatePipeline(device, *this, name)
				: CreateUncached(device);

			DebugName(pso, name);
			return pso;
		}

	private:

		// Create the pipeline state object without the cache.
		D3DPtr<ID3D12PipelineState> CreateUncached(ID3D12Device* device) const
		{
			D3DPtr<ID3D12PipelineState> pso;
			Check(device->CreateComputePipelineState(this, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));
			return pso;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/compute/gpu.h"
#include "pr/compute/utility/root_signature.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/shaders/shader_cache_fs.h"
namespace pr::compute::tests
{
	PRUnitTestClass(ComputePSOCacheTests)
	{
		// A pipeline created through a file system cache is saved when the cache is destroyed, and loaded by a later cache instead of compiled again.
		PRUnitTestMethod(PipelineLibraryRoundTrip, Extended)
		{
			// Use a private cache directory so earlier runs cannot affect the result.
			auto const cache_dir = std::filesystem::temp_directory_path() / std::format(L"pr_pso_cache_test_{}", ::GetCurrentProcessId());
			std::filesystem::remove_all(cache_dir);
			auto cleanup = Scope<void>([&]
			{
				std::error_code ec;
				std::filesystem::remove_all(cache_dir, ec);
			});

			ComGpu gpu(TestGpuOptions);
			auto const bytecode = ShaderCompiler{}
				.Source("RWStructuredBuffer<uint> buf : register(u0);\n[numthreads(1,1,1)] void main(uint3 id : SV_DispatchThreadID) { buf[id.x] = id.x; }\n")
				.EntryPoint(L"main")
				.ShaderModel(L"cs_6_0")
				.Compile();
			auto sig = RootSig(ERootSigFlags::ComputeOnly).UAV(hlsl::EUAVReg::u0).Create(gpu, "PSOCacheTest:Sig");

			// The first cache compiles the pipeline, stores it, and then reuses it from the in-memory library.
			{
				auto cache = shader_cache::ShaderCacheFS(cache_dir, "test");
				auto pso0 = ComputePSO(sig.get(), bytecode).Cache(&cache).Create(gpu, "PSOCacheTest:PSO");
				PR_EXPECT(pso0 != nullptr);

				// Pipeline libraries are optional driver features. Without one, pipelines are created uncached and there is nothing more to check.
				if (cache.Stats().m_pipeline_uncached != 0)
					return;

				auto pso1 = ComputePSO(sig.get(), bytecode).Cache(&cache).Create(gpu, "PSOCacheTest:PSO");
				PR_EXPECT(pso1 != nullptr);
				PR_EXPECT(cache.Stats().m_pipeline_miss == 1);
				PR_EXPECT(cache.Stats().m_pipeline_hit == 1);
			}

			// A new cache loads the saved library and finds the pipeline without compiling it.
			{
				auto cache = shader_cache::ShaderCacheFS(cache_dir, "test");
				auto pso = ComputePSO(sig.get(), bytecode).Cache(&cache).Create(gpu, "PSOCacheTest:PSO");
				PR_EXPECT(pso != nullptr);
				PR_EXPECT(cache.Stats().m_pipeline_hit == 1);
				PR_EXPECT(cache.Stats().m_pipeline_miss == 0);
			}
		}
	};
}
#endif
