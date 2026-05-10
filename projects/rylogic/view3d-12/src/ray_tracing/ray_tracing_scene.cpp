//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_scene.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_model.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_resource.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/utility/barrier_batch.h"

namespace pr::rdr12
{
	namespace
	{
		struct InstanceBuildInput
		{
			RayTracingSceneStats m_stats;
			vector<D3D12_RAYTRACING_INSTANCE_DESC, 1024> m_instances;
			uint64_t m_signature;

			// Create empty TLAS build input.
			InstanceBuildInput()
				: m_stats()
				, m_instances()
				, m_signature(hash::FNV_offset_basis64)
			{}
		};

		// Copy a View3D object-to-world transform into DXR's row-major 3x4 instance transform.
		void CopyTransform(m4x4 const& o2w, float (&transform)[3][4])
		{
			for (auto row = 0; row != 3; ++row)
			{
				auto const r = o2w.row(row);
				transform[row][0] = r.x;
				transform[row][1] = r.y;
				transform[row][2] = r.z;
				transform[row][3] = r.w;
			}
		}

		// Add POD data to a signature used to decide whether the TLAS must be rebuilt.
		template <typename T>
		void AddSignature(uint64_t& signature, T const& value)
		{
			signature = s_cast<uint64_t>(hash::HashBytes64(&value, &value + 1, signature));
		}

		// Build the static instance descriptors used to create a top-level acceleration structure for 'instances'.
		InstanceBuildInput BuildInstanceInput(ResourceFactory& factory, std::span<BaseInstance const* const> instances)
		{
			InstanceBuildInput result;
			result.m_stats.m_rt_available = factory.rdr().RayTracing().Available();
			result.m_stats.m_scene_instance_count = isize(instances);
			AddSignature(result.m_signature, result.m_stats.m_scene_instance_count);

			if (!result.m_stats.m_rt_available)
				return result;

			auto instance_index = 0;
			for (auto const* inst : instances)
			{
				AddSignature(result.m_signature, reinterpret_cast<uintptr_t>(inst));
				if (inst == nullptr)
				{
					++result.m_stats.m_excluded_no_model_count;
					continue;
				}

				auto const* model_ptr = inst->find<ModelPtr>(EInstComp::ModelPtr);
				if (model_ptr == nullptr || *model_ptr == nullptr)
				{
					++result.m_stats.m_excluded_no_model_count;
					continue;
				}

				// DXR instance transforms are affine 3x4 matrices. Keep the initial eligibility conservative until non-affine handling is explicit.
				auto const flags = GetFlags(*inst);
				auto const& o2w = GetO2W(*inst);
				if (AllSet(flags, EInstFlag::NonAffine) || !IsAffine(o2w))
				{
					++result.m_stats.m_excluded_non_affine_count;
					continue;
				}

				auto& model = *model_ptr->get();
				auto const was_built = model.m_ray_tracing.Built();
				auto const geom_stats = model.m_ray_tracing.Build(factory, model);
				if (geom_stats.m_skinned_model)
				{
					++result.m_stats.m_excluded_skinned_count;
					continue;
				}
				if (!model.m_ray_tracing.Built())
				{
					++result.m_stats.m_excluded_unsupported_count;
					continue;
				}

				// Store enough identity in the signature to catch transform, instance-id, and BLAS lifetime changes without embedding RT state in instances.
				auto const unique_id = UniqueId(*inst);
				auto desc = D3D12_RAYTRACING_INSTANCE_DESC{};
				CopyTransform(o2w, desc.Transform);
				desc.InstanceID = s_cast<UINT>(unique_id != 0 ? unique_id : instance_index) & 0x00FFFFFF;
				desc.InstanceMask = 0xFF;
				desc.InstanceContributionToHitGroupIndex = 0;
				desc.Flags = D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE;
				desc.AccelerationStructure = model.m_ray_tracing.AccelerationStructureAddress();

				AddSignature(result.m_signature, desc);
				AddSignature(result.m_signature, model.m_ray_tracing.Revision());
				result.m_instances.push_back(desc);
				++result.m_stats.m_tlas_instance_count;
				++instance_index;

				if (was_built)
					++result.m_stats.m_blas_cached_count;
				else
					++result.m_stats.m_blas_build_count;
			}
			return result;
		}
	}

	struct RayTracingScene::Data
	{
		D3DPtr<ID3D12Resource> m_tlas;
		D3DPtr<ID3D12Resource> m_scratch;
		D3DPtr<ID3D12Resource> m_instances;
		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO m_prebuild_info;

		// Create an empty heavy-weight TLAS data object.
		Data()
			: m_tlas()
			, m_scratch()
			, m_instances()
			, m_prebuild_info()
		{}
	};

	// Create empty ray tracing scene diagnostics.
	RayTracingSceneStats::RayTracingSceneStats()
		: m_rt_available()
		, m_scene_instance_count()
		, m_tlas_instance_count()
		, m_blas_build_count()
		, m_blas_cached_count()
		, m_excluded_no_model_count()
		, m_excluded_skinned_count()
		, m_excluded_unsupported_count()
		, m_excluded_non_affine_count()
		, m_tlas_size()
		, m_scratch_size()
		, m_instance_desc_size()
	{}

	// Create empty per-scene ray tracing state.
	RayTracingScene::RayTracingScene()
		: m_data()
		, m_stats()
		, m_signature(hash::FNV_offset_basis64)
	{}

	// Move per-scene ray tracing state without copying GPU resource ownership.
	RayTracingScene::RayTracingScene(RayTracingScene&& rhs) noexcept
		: m_data(std::move(rhs.m_data))
		, m_stats(rhs.m_stats)
		, m_signature(rhs.m_signature)
	{}

	// Move per-scene ray tracing state without copying GPU resource ownership.
	RayTracingScene& RayTracingScene::operator =(RayTracingScene&& rhs) noexcept
	{
		if (&rhs == this)
			return *this;

		m_data = std::move(rhs.m_data);
		m_stats = rhs.m_stats;
		m_signature = rhs.m_signature;
		return *this;
	}

	// Destroy the per-scene ray tracing state.
	RayTracingScene::~RayTracingScene() = default;

	// Return true if this scene currently owns a built TLAS.
	bool RayTracingScene::Built() const
	{
		return m_data != nullptr && m_data->m_tlas != nullptr;
	}

	// Return the latest TLAS build diagnostics.
	RayTracingSceneStats const& RayTracingScene::Stats() const
	{
		return m_stats;
	}

	// Return the TLAS GPU virtual address, or zero if no TLAS is built.
	D3D12_GPU_VIRTUAL_ADDRESS RayTracingScene::AccelerationStructureAddress() const
	{
		return Built()
			? m_data->m_tlas->GetGPUVirtualAddress()
			: D3D12_GPU_VIRTUAL_ADDRESS{};
	}

	// Release TLAS resources after deferring GPU lifetime management through the renderer.
	void RayTracingScene::DeferRelease(Renderer& rdr)
	{
		if (m_data == nullptr)
			return;

		rdr.DeferRelease(m_data->m_tlas);
		rdr.DeferRelease(m_data->m_scratch);
		rdr.DeferRelease(m_data->m_instances);
		m_data.reset();
	}

	// Mark the scene's TLAS as invalid because the scene content changed.
	void RayTracingScene::Invalidate(Renderer& rdr)
	{
		DeferRelease(rdr);
		m_signature = hash::FNV_offset_basis64;
	}

	// Build or reuse the scene TLAS for the provided instances.
	RayTracingSceneStats RayTracingScene::Build(ResourceFactory& factory, std::span<BaseInstance const* const> instances)
	{
		auto input = BuildInstanceInput(factory, instances);
		m_stats = input.m_stats;
		if (!m_stats.m_rt_available || input.m_instances.empty())
		{
			DeferRelease(factory.rdr());
			m_signature = input.m_signature;
			return m_stats;
		}

		if (Built() && m_signature == input.m_signature)
		{
			m_stats.m_tlas_size = m_data->m_prebuild_info.ResultDataMaxSizeInBytes;
			m_stats.m_scratch_size = m_data->m_prebuild_info.ScratchDataSizeInBytes;
			m_stats.m_instance_desc_size = input.m_instances.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC);
			return m_stats;
		}

		D3DPtr<ID3D12Device5> device;
		Check(factory.rdr().D3DDevice()->QueryInterface<ID3D12Device5>(device.address_of()));

		// Instance descriptors are normal GPU-read buffers. Only the TLAS result resource needs the special acceleration-structure state.
		auto instance_desc = ResDesc::Buf<D3D12_RAYTRACING_INSTANCE_DESC>(
			isize(input.m_instances),
			std::span<D3D12_RAYTRACING_INSTANCE_DESC const>(input.m_instances.data(), input.m_instances.size()))
			.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		auto instance_buffer = factory.CreateResource(instance_desc, "RayTracing:TLAS instances");

		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
			.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL,
			.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE,
			.NumDescs = s_cast<UINT>(input.m_instances.size()),
			.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
			.InstanceDescs = instance_buffer->GetGPUVirtualAddress(),
		};

		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuild_info = {};
		device->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &prebuild_info);
		if (prebuild_info.ResultDataMaxSizeInBytes == 0)
			throw std::runtime_error("Failed to get TLAS prebuild information");

		DeferRelease(factory.rdr());
		m_data = std::make_unique<Data>();
		m_data->m_prebuild_info = prebuild_info;
		m_data->m_instances = instance_buffer;
		m_signature = input.m_signature;
		m_stats.m_tlas_size = prebuild_info.ResultDataMaxSizeInBytes;
		m_stats.m_scratch_size = prebuild_info.ScratchDataSizeInBytes;
		m_stats.m_instance_desc_size = input.m_instances.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC);

		auto scratch_desc = ResDesc::Buf(s_cast<int64_t>(prebuild_info.ScratchDataSizeInBytes), 1, std::span<std::byte const>{}, 1)
			.usage(EUsage::UnorderedAccess)
			.def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		m_data->m_tlas = CreateRayTracingAccelerationStructure(factory, prebuild_info.ResultDataMaxSizeInBytes, "RayTracing:TLAS");
		m_data->m_scratch = factory.CreateResource(scratch_desc, "RayTracing:TLAS scratch");

		D3DPtr<ID3D12GraphicsCommandList4> cmd_list4;
		Check(factory.CmdList().get()->QueryInterface<ID3D12GraphicsCommandList4>(cmd_list4.address_of()));

		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC build_desc = {
			.DestAccelerationStructureData = m_data->m_tlas->GetGPUVirtualAddress(),
			.Inputs = inputs,
			.SourceAccelerationStructureData = 0,
			.ScratchAccelerationStructureData = m_data->m_scratch->GetGPUVirtualAddress(),
		};
		cmd_list4->BuildRaytracingAccelerationStructure(&build_desc, 0, nullptr);

		BarrierBatch build_barriers(factory.CmdList());
		build_barriers.UAV(m_data->m_tlas.get());
		build_barriers.Commit();

		PR_INFO(PR_DBG_RDR, std::format(
			"Built TLAS: {} instances, {} bytes\n",
			m_stats.m_tlas_instance_count,
			m_stats.m_tlas_size));

		return m_stats;
	}
}
