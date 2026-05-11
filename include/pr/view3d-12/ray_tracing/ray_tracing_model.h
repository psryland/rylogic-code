//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/utility/cmd_list.h"

namespace pr::rdr12
{
	// Diagnostics describing how a model maps to bottom-level acceleration structure geometry.
	struct RayTracingGeometryStats
	{
		bool m_rt_available;
		bool m_skinned_model;
		int m_nugget_count;
		int m_geometry_count;
		int m_hidden_count;
		int m_unsupported_topology_count;
		int m_missing_position_count;
		int m_empty_range_count;
		int m_invalid_range_count;
		int m_non_triangle_index_count;
		uint64_t m_blas_size;
		uint64_t m_scratch_size;

		// Create empty ray tracing geometry diagnostics.
		RayTracingGeometryStats();
	};

	// Geometry descriptors and diagnostics used to build a bottom-level acceleration structure.
	struct RayTracingGeometryBuildInput
	{
		RayTracingGeometryStats m_stats;
		vector<D3D12_RAYTRACING_GEOMETRY_DESC, 8> m_geometry;
	};

	// Build the triangle descriptors used to create a bottom-level acceleration structure for 'model'.
	RayTracingGeometryBuildInput RayTracingBuildGeometryInput(Model const& model, D3D12_GPU_VIRTUAL_ADDRESS vertex_buffer_address, bool rt_available, bool include_skinned);

	// Per-model ray tracing state. Heavy BLAS data is allocated lazily only when ray tracing is used.
	struct RayTracingModel
	{
		struct Data;

	private:

		std::unique_ptr<Data> m_data;
		uint64_t m_revision;

	public:

		// Create empty per-model ray tracing state.
		RayTracingModel();

		// Move per-model ray tracing state without copying GPU resource ownership.
		RayTracingModel(RayTracingModel&& rhs) noexcept;
		RayTracingModel(RayTracingModel const&) = delete;

		// Move per-model ray tracing state without copying GPU resource ownership.
		RayTracingModel& operator =(RayTracingModel&& rhs) noexcept;
		RayTracingModel& operator =(RayTracingModel const&) = delete;

		// Destroy the per-model ray tracing state.
		~RayTracingModel();

		// Return true if this model currently owns a built BLAS.
		bool Built() const;

		// Return the BLAS GPU virtual address, or zero if no BLAS is built.
		D3D12_GPU_VIRTUAL_ADDRESS AccelerationStructureAddress() const;

		// Return a value that changes whenever the model's BLAS lifetime changes.
		uint64_t Revision() const;

		// Release BLAS resources after deferring GPU lifetime management through the renderer.
		void DeferRelease(Renderer& rdr);

		// Mark the model's BLAS as invalid because the model geometry or nugget layout changed.
		void Invalidate(Renderer& rdr);

		// Analyse model geometry and return RT eligibility diagnostics without allocating GPU resources.
		RayTracingGeometryStats Analyse(Model const& model) const;

		// Build the model's static BLAS if ray tracing is available and the model has eligible geometry.
		RayTracingGeometryStats Build(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, Model const& model);
	};
}
