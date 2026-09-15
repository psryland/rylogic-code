// Shared bounded GPU transport for field and separately owned contact proof fixtures.
#pragma once
#include "src/unittests/forward.h"
#include "src/unittests/shared_gpu.h"
#include "pr/physics/terrain/landscape/baseline_surface.h"
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"

namespace pr::physics::tests::terrain_probe
{
	using namespace pr::compute;
	using namespace terrain::landscape;
	using terrain::v2d;
	using shared::BaselineRecipe;
	using shared::BaselineResult;

	// Compile either precision of the canonical public evaluator with the repository's strict DXC wrapper.
	inline std::vector<uint8_t> CompileBaselineProbe(wchar_t const* entry, wchar_t const* model, bool optimise, bool fp32 = false, char const* file = "baseline_probe.hlsl")
	{
		auto const source_dir = std::filesystem::path(__FILE__).parent_path();
		auto const root = source_dir.parent_path().parent_path().parent_path().parent_path().parent_path();
		auto resolver = shader_cache::FileSourceResolver(std::vector<std::filesystem::path>{root / "include", root / "projects\\rylogic", source_dir / "..\\terrain\\landscape"});
		return ShaderCompiler().Source(file, resolver).EntryPoint(entry).ShaderModel(model).HlslVersion(EHlslVersion::Hlsl2021).Arg(L"-Gis").Arg(fp32 ? L"-DPR_TERRAIN_FP32=1" : L"-DPR_TERRAIN_FP32=0").Optimise(optimise).Compile();
	}

	// Reject unsupported hardware rather than silently executing a reduced-precision landscape.
	inline void RequireBaselineDevice(Gpu& gpu)
	{
		auto options = D3D12_FEATURE_DATA_D3D12_OPTIONS{};
		auto options1 = D3D12_FEATURE_DATA_D3D12_OPTIONS1{};
		auto model = D3D12_FEATURE_DATA_SHADER_MODEL{D3D_SHADER_MODEL_6_0};
		Check(gpu->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &options, sizeof(options)));
		Check(gpu->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS1, &options1, sizeof(options1)));
		Check(gpu->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &model, sizeof(model)));
		if (!options.DoublePrecisionFloatShaderOps || !options1.Int64ShaderOps || model.HighestShaderModel < D3D_SHADER_MODEL_6_0)
			throw std::runtime_error("Terrain baseline requires SM6.0, FP64 and Int64ShaderOps");

		// Pipeline creation also validates the DXIL double-precision extension requirements.
		std::printf("Terrain device: FP64=%d Int64=%d SM=%x\n", options.DoublePrecisionFloatShaderOps, options1.Int64ShaderOps, model.HighestShaderModel);

		// Identify the actual D3D12 adapter rather than assuming the machine's primary display adapter.
		auto factory = D3DPtr<IDXGIFactory4>{};
		auto adapter = D3DPtr<IDXGIAdapter1>{};
		Check(CreateDXGIFactory1(__uuidof(IDXGIFactory4), (void**)factory.address_of()));
		Check(factory->EnumAdapterByLuid(gpu->GetAdapterLuid(), __uuidof(IDXGIAdapter1), (void**)adapter.address_of()));
		auto description = DXGI_ADAPTER_DESC1{};
		Check(adapter->GetDesc1(&description));
		std::printf("Terrain adapter: %ls (vendor %04x device %04x)\n", description.Description, description.VendorId, description.DeviceId);

		// Identify the loaded compiler, rather than a potentially different dxc.exe found on PATH.
		auto compiler = D3DPtr<IDxcVersionInfo>{};
		Check(DxcCreateInstance(CLSID_DxcCompiler, __uuidof(IDxcVersionInfo), (void**)compiler.address_of()));
		uint32_t major = 0, minor = 0;
		Check(compiler->GetVersion(&major, &minor));
		auto module = ::GetModuleHandleW(L"dxcompiler.dll");
		if (!module)
			throw std::runtime_error("Loaded DXC module not found");

		std::printf("Terrain DXC: %u.%u module=%ls\n", major, minor, win32::ModuleFileName(module).c_str());
		std::fflush(stdout);
	}

	// Execute one bounded synchronous proof batch; resources live until the shared job has completed.
	inline std::vector<BaselineResult> SampleBaselineGpu(Gpu& gpu, ComputeStep& step, BaselineRecipe const& recipe, std::span<v2d const> positions, double* gpu_ms = nullptr)
	{
		if (positions.size() > 4096)
			throw std::length_error("Terrain proof batch exceeds 4096 queries");
		if (positions.empty())
			return {};

		// Bound counts before any narrowing, multiplication, allocation, or command recording.
		auto const count = static_cast<uint32_t>(positions.size());
		auto output = std::vector<BaselineResult>(count);
		auto& job = gpu.m_job;
		auto recipe_buffer = gpu.CreateResource(ResDesc::Buf<BaselineRecipe>(1, {}), job.m_cmd_list, "Terrain:Recipe");
		auto positions_buffer = gpu.CreateResource(ResDesc::Buf<v2d>(count, {}), job.m_cmd_list, "Terrain:Positions");
		auto results_buffer = gpu.CreateResource(ResDesc::Buf<BaselineResult>(count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Terrain:Results");
		auto recipe_upload = job.m_upload.Alloc<BaselineRecipe>(1);
		auto positions_upload = job.m_upload.Alloc<v2d>(static_cast<int>(count));
		auto readback = job.m_readback.Alloc<BaselineResult>(static_cast<int>(count));
		auto timestamps = job.m_readback.Alloc<uint64_t>(2);
		auto heap = D3DPtr<ID3D12QueryHeap>{};
		auto heap_desc = D3D12_QUERY_HEAP_DESC{D3D12_QUERY_HEAP_TYPE_TIMESTAMP, 2, 0};
		Check(gpu->CreateQueryHeap(&heap_desc, __uuidof(ID3D12QueryHeap), (void**)heap.address_of()));
		uint64_t frequency = 0;
		Check(job.m_queue->GetTimestampFrequency(&frequency));
		*recipe_upload.ptr<BaselineRecipe>() = recipe;
		std::copy(positions.begin(), positions.end(), positions_upload.ptr<v2d>());

		// Upload allocations must belong to the job's fence, not a separate resource-creation queue.
		job.m_barriers.Transition(recipe_buffer.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(positions_buffer.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(recipe_buffer.get(), 0, recipe_upload);
		job.m_cmd_list.CopyBufferRegion(positions_buffer.get(), 0, positions_upload);
		job.m_barriers.Transition(recipe_buffer.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(positions_buffer.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(results_buffer.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();

		// Bind only this batch and guard the final partial threadgroup inside the compute consumer.
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(count);
		job.m_cmd_list.AddComputeRootShaderResourceView(recipe_buffer->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(positions_buffer->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(results_buffer->GetGPUVirtualAddress());
		job.m_cmd_list.get()->EndQuery(heap.get(), D3D12_QUERY_TYPE_TIMESTAMP, 0);
		job.m_cmd_list.Dispatch(static_cast<int>((count + 31) / 32), 1, 1);
		job.m_cmd_list.get()->EndQuery(heap.get(), D3D12_QUERY_TYPE_TIMESTAMP, 1);
		job.m_cmd_list.get()->ResolveQueryData(heap.get(), D3D12_QUERY_TYPE_TIMESTAMP, 0, 2, timestamps.m_res, timestamps.m_ofs);
		job.m_barriers.Transition(results_buffer.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		job.m_cmd_list.CopyBufferRegion(readback, results_buffer.get(), 0);
		job.Run();
		if (gpu_ms)
			*gpu_ms = 1000.0 * static_cast<double>(timestamps.ptr<uint64_t>()[1] - timestamps.ptr<uint64_t>()[0]) / frequency;

		std::copy_n(readback.ptr<BaselineResult>(), count, output.begin());
		return output;
	}
}
