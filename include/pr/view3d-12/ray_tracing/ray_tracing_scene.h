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
	// Diagnostics describing how a scene maps visible instances into a top-level acceleration structure.
	struct RayTracingSceneStats
	{
		bool m_rt_available;
		int m_scene_instance_count;
		int m_tlas_instance_count;
		int m_blas_build_count;
		int m_blas_cached_count;
		int m_excluded_no_model_count;
		int m_excluded_skinned_count;
		int m_excluded_unsupported_count;
		int m_excluded_non_affine_count;
		uint64_t m_tlas_size;
		uint64_t m_scratch_size;
		uint64_t m_instance_desc_size;

		// Create empty ray tracing scene diagnostics.
		RayTracingSceneStats();
	};

	// Per-scene ray tracing state. Heavy TLAS data is allocated lazily only when the RT render step is used.
	struct RayTracingScene
	{
		struct Data;

	private:

		std::unique_ptr<Data> m_data;
		RayTracingSceneStats m_stats;
		uint64_t m_signature;

	public:

		// Create empty per-scene ray tracing state.
		RayTracingScene();

		// Move per-scene ray tracing state without copying GPU resource ownership.
		RayTracingScene(RayTracingScene&& rhs) noexcept;
		RayTracingScene(RayTracingScene const&) = delete;

		// Move per-scene ray tracing state without copying GPU resource ownership.
		RayTracingScene& operator =(RayTracingScene&& rhs) noexcept;
		RayTracingScene& operator =(RayTracingScene const&) = delete;

		// Destroy the per-scene ray tracing state.
		~RayTracingScene();

		// Return true if this scene currently owns a built TLAS.
		bool Built() const;

		// Return the latest TLAS build diagnostics.
		RayTracingSceneStats const& Stats() const;

		// Return the TLAS GPU virtual address, or zero if no TLAS is built.
		D3D12_GPU_VIRTUAL_ADDRESS AccelerationStructureAddress() const;

		// Return the material buffer used by reflection closest-hit shaders.
		ID3D12Resource* MaterialBuffer() const;

		// Return the number of material records in the material buffer.
		int MaterialCount() const;

		// Return the packed vertex buffer used by RT closest-hit shading.
		ID3D12Resource* ShadingVertexBuffer() const;

		// Return the number of packed RT shading vertices.
		int ShadingVertexCount() const;

		// Return the packed 16-bit index buffer used by RT closest-hit shading.
		ID3D12Resource* ShadingIndex16Buffer() const;

		// Return the number of 16-bit indices in the packed RT shading index buffer.
		int ShadingIndex16Count() const;

		// Return the packed 32-bit index buffer used by RT closest-hit shading.
		ID3D12Resource* ShadingIndex32Buffer() const;

		// Return the number of 32-bit indices in the packed RT shading index buffer.
		int ShadingIndex32Count() const;

		// Return the geometry metadata buffer used by RT closest-hit shading.
		ID3D12Resource* ShadingGeometryBuffer() const;

		// Return the number of geometry records in the RT closest-hit shading buffer.
		int ShadingGeometryCount() const;

		// Return the number of geometry records that must fall back to material-only shading.
		int ShadingGeometryFallbackCount() const;

		// Release TLAS and dynamic BLAS resources after deferring GPU lifetime management through the renderer.
		void DeferRelease(Renderer& rdr);

		// Mark the scene's TLAS as invalid because the scene content changed.
		void Invalidate(Renderer& rdr);

		// Build or reuse the scene TLAS for the provided instances.
		RayTracingSceneStats Build(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, std::span<BaseInstance const* const> instances);
	};
}
