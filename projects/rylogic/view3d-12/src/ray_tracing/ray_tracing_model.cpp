//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_model.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_resource.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/utility/barrier_batch.h"

namespace pr::rdr12
{
	namespace
	{
		// Return the RT geometry flag to use for a render nugget.
		D3D12_RAYTRACING_GEOMETRY_FLAGS GeometryFlags(Nugget const& nugget)
		{
			return AnySet(nugget.m_nflags, ENuggetFlag::GeometryHasAlpha | ENuggetFlag::TintHasAlpha | ENuggetFlag::TexDiffuseHasAlpha)
				? D3D12_RAYTRACING_GEOMETRY_FLAG_NONE
				: D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
		}
	}

	// Build the triangle descriptors used to create a bottom-level acceleration structure for 'model'.
	RayTracingGeometryBuildInput RayTracingBuildGeometryInput(Model const& model, D3D12_GPU_VIRTUAL_ADDRESS vertex_buffer_address, bool rt_available, bool include_skinned)
	{
		RayTracingGeometryBuildInput result;
		result.m_stats.m_rt_available = rt_available;
		result.m_stats.m_skinned_model = model.m_skin ? true : false;

		// Static BLASes are model-owned, so they must not trace bind-pose vertices for skinned models. Dynamic skinned BLAS callers pass the
		// compute-skinned vertex buffer and opt in via 'include_skinned'.
		if (result.m_stats.m_skinned_model && !include_skinned)
			return result;

		auto model_vrange = Range(0, model.m_vcount);
		auto model_irange = Range(0, model.m_icount);
		for (auto const* nugget = model.m_nuggets.get(); nugget != nullptr; nugget = nugget->m_next.get())
		{
			++result.m_stats.m_nugget_count;

			if (AllSet(nugget->m_nflags, ENuggetFlag::Hidden))
			{
				++result.m_stats.m_hidden_count;
				continue;
			}
			if (nugget->m_topo != ETopo::TriList)
			{
				++result.m_stats.m_unsupported_topology_count;
				continue;
			}
			if (!AllSet(nugget->m_geom, EGeom::Vert))
			{
				++result.m_stats.m_missing_position_count;
				continue;
			}
			if (nugget->m_irange.empty() || nugget->m_irange.size() < 3 || nugget->m_vrange.empty())
			{
				++result.m_stats.m_empty_range_count;
				continue;
			}
			if (!IsWithin(model_vrange, nugget->m_vrange) || !IsWithin(model_irange, nugget->m_irange))
			{
				++result.m_stats.m_invalid_range_count;
				continue;
			}
			if (nugget->m_irange.size() % 3 != 0)
			{
				++result.m_stats.m_non_triangle_index_count;
				continue;
			}

			// Nugget indices are model-relative, so expose the whole vertex buffer to DXR and select nugget geometry with the index range.
			result.m_nuggets.push_back(nugget);
			result.m_geometry.push_back(D3D12_RAYTRACING_GEOMETRY_DESC{
				.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES,
				.Flags = GeometryFlags(*nugget),
				.Triangles = {
					.Transform3x4 = 0,
					.IndexFormat = model.m_ib_view.Format,
					.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT,
					.IndexCount = s_cast<UINT>(nugget->m_irange.size()),
					.VertexCount = s_cast<UINT>(model.m_vcount),
					.IndexBuffer = model.m_ib->GetGPUVirtualAddress() + s_cast<uint64_t>(nugget->m_irange.m_beg) * model.m_istride.size(),
					.VertexBuffer = {
						.StartAddress = vertex_buffer_address + offsetof(Vert, m_vert),
						.StrideInBytes = s_cast<UINT64>(model.m_vstride.size()),
					},
				},
			});
		}
		result.m_stats.m_geometry_count = isize(result.m_geometry);
		return result;
	}

	struct RayTracingModel::Data
	{
		D3DPtr<ID3D12Resource> m_blas;
		D3DPtr<ID3D12Resource> m_scratch;
		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO m_prebuild_info;
		RayTracingGeometryStats m_stats;

		// Create an empty heavy-weight BLAS data object.
		Data()
			: m_blas()
			, m_scratch()
			, m_prebuild_info()
			, m_stats()
		{}
	};

	// Create empty ray tracing geometry diagnostics.
	RayTracingGeometryStats::RayTracingGeometryStats()
		: m_rt_available()
		, m_skinned_model()
		, m_nugget_count()
		, m_geometry_count()
		, m_hidden_count()
		, m_unsupported_topology_count()
		, m_missing_position_count()
		, m_empty_range_count()
		, m_invalid_range_count()
		, m_non_triangle_index_count()
		, m_blas_size()
		, m_scratch_size()
	{}

	// Create empty per-model ray tracing state.
	RayTracingModel::RayTracingModel()
		: m_data()
		, m_revision()
	{}

	// Move per-model ray tracing state without copying GPU resource ownership.
	RayTracingModel::RayTracingModel(RayTracingModel&& rhs) noexcept
		: m_data(std::move(rhs.m_data))
		, m_revision(rhs.m_revision)
	{}

	// Move per-model ray tracing state without copying GPU resource ownership.
	RayTracingModel& RayTracingModel::operator =(RayTracingModel&& rhs) noexcept
	{
		if (&rhs == this)
			return *this;

		m_data = std::move(rhs.m_data);
		m_revision = rhs.m_revision;
		return *this;
	}

	// Destroy the per-model ray tracing state.
	RayTracingModel::~RayTracingModel() = default;

	// Return true if this model currently owns a built BLAS.
	bool RayTracingModel::Built() const
	{
		return m_data != nullptr && m_data->m_blas != nullptr;
	}

	// Return the BLAS GPU virtual address, or zero if no BLAS is built.
	D3D12_GPU_VIRTUAL_ADDRESS RayTracingModel::AccelerationStructureAddress() const
	{
		return Built()
			? m_data->m_blas->GetGPUVirtualAddress()
			: D3D12_GPU_VIRTUAL_ADDRESS{};
	}

	// Return a value that changes whenever the model's BLAS lifetime changes.
	uint64_t RayTracingModel::Revision() const
	{
		return m_revision;
	}

	// Release BLAS resources after deferring GPU lifetime management through the renderer.
	void RayTracingModel::DeferRelease(Renderer& rdr)
	{
		if (m_data == nullptr)
			return;

		rdr.DeferRelease(m_data->m_blas);
		rdr.DeferRelease(m_data->m_scratch);
		m_data.reset();
		++m_revision;
	}

	// Mark the model's BLAS as invalid because the model geometry or nugget layout changed.
	void RayTracingModel::Invalidate(Renderer& rdr)
	{
		DeferRelease(rdr);
	}

	// Analyse model geometry and return RT eligibility diagnostics without allocating GPU resources.
	RayTracingGeometryStats RayTracingModel::Analyse(Model const& model) const
	{
		return RayTracingBuildGeometryInput(model, model.m_vb->GetGPUVirtualAddress(), model.rdr().RayTracing().Available(), false).m_stats;
	}

	// Build the model's static BLAS if ray tracing is available and the model has eligible geometry.
	RayTracingGeometryStats RayTracingModel::Build(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, Model const& model)
	{
		if (m_data != nullptr)
			return m_data->m_stats;

		auto input = RayTracingBuildGeometryInput(model, model.m_vb->GetGPUVirtualAddress(), rdr.RayTracing().Available(), false);
		if (!input.m_stats.m_rt_available || input.m_geometry.empty())
			return input.m_stats;

		D3DPtr<ID3D12Device5> device;
		Check(rdr.D3DDevice()->QueryInterface<ID3D12Device5>(device.address_of()));

		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
			.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL,
			.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE,
			.NumDescs = s_cast<UINT>(input.m_geometry.size()),
			.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
			.pGeometryDescs = input.m_geometry.data(),
		};

		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuild_info = {};
		device->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &prebuild_info);
		if (prebuild_info.ResultDataMaxSizeInBytes == 0)
			throw std::runtime_error(std::format("Failed to get BLAS prebuild information for model '{}'", model.m_name));

		DeferRelease(rdr);
		m_data = std::make_unique<Data>();
		m_data->m_prebuild_info = prebuild_info;
		m_data->m_stats = input.m_stats;
		m_data->m_stats.m_blas_size = prebuild_info.ResultDataMaxSizeInBytes;
		m_data->m_stats.m_scratch_size = prebuild_info.ScratchDataSizeInBytes;

		auto scratch_desc = ResDesc::Buf(s_cast<int64_t>(prebuild_info.ScratchDataSizeInBytes), 1, std::span<std::byte const>{}, 1)
			.usage(EUsage::UnorderedAccess)
			.def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		m_data->m_blas = CreateRayTracingAccelerationStructure(rdr, prebuild_info.ResultDataMaxSizeInBytes, std::format("{}:BLAS", model.m_name));
		m_data->m_scratch = CreateRayTracingResource(rdr, cmd_list, upload, scratch_desc, std::format("{}:BLAS scratch", model.m_name));
		++m_revision;

		BarrierBatch barriers(cmd_list);
		barriers.Transition(model.m_vb.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		barriers.Transition(model.m_ib.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		barriers.Commit();

		D3DPtr<ID3D12GraphicsCommandList4> cmd_list4;
		Check(cmd_list.get()->QueryInterface<ID3D12GraphicsCommandList4>(cmd_list4.address_of()));

		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC build_desc = {
			.DestAccelerationStructureData = m_data->m_blas->GetGPUVirtualAddress(),
			.Inputs = inputs,
			.SourceAccelerationStructureData = 0,
			.ScratchAccelerationStructureData = m_data->m_scratch->GetGPUVirtualAddress(),
		};
		cmd_list4->BuildRaytracingAccelerationStructure(&build_desc, 0, nullptr);

		BarrierBatch build_barriers(cmd_list);
		build_barriers.UAV(m_data->m_blas.get());
		build_barriers.Commit();

		PR_INFO(PR_DBG_RDR, std::format(
			"Built BLAS for model '{}': {} geometries, {} bytes\n",
			model.m_name,
			m_data->m_stats.m_geometry_count,
			m_data->m_stats.m_blas_size));

		return m_data->m_stats;
	}
}
