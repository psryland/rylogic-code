//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

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

		// Release TLAS resources after deferring GPU lifetime management through the renderer.
		void DeferRelease(Renderer& rdr);

		// Mark the scene's TLAS as invalid because the scene content changed.
		void Invalidate(Renderer& rdr);

		// Build or reuse the scene TLAS for the provided instances.
		RayTracingSceneStats Build(ResourceFactory& factory, std::span<BaseInstance const* const> instances);
	};
}
