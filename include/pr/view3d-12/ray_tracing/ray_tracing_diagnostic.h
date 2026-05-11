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
	struct RayTracingReflectionBuffer;

	enum class ERayTracingScreenPass
	{
		None,
		Diagnostic,
		HardShadows,
		Reflections,
		Caustics,
		ReflectionsAndCaustics,
	};

	// Owns the minimal DXR screen-space pipeline used to visualise TLAS/BLAS coverage and prototype RT lighting features.
	struct RayTracingDiagnostic
	{
		struct Data;

	private:

		std::unique_ptr<Data> m_data;

	public:

		// Create empty diagnostic ray tracing state.
		RayTracingDiagnostic();

		// Move diagnostic ray tracing state without copying GPU resource ownership.
		RayTracingDiagnostic(RayTracingDiagnostic&& rhs) noexcept;
		RayTracingDiagnostic(RayTracingDiagnostic const&) = delete;

		// Move diagnostic ray tracing state without copying GPU resource ownership.
		RayTracingDiagnostic& operator =(RayTracingDiagnostic&& rhs) noexcept;
		RayTracingDiagnostic& operator =(RayTracingDiagnostic const&) = delete;

		// Destroy the diagnostic ray tracing state.
		~RayTracingDiagnostic();

		// Release GPU resources after deferring GPU lifetime management through the renderer.
		void DeferRelease(Renderer& rdr);

		// Create or resize GPU resources needed for the diagnostic pass.
		bool Prepare(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, iv2 output_size, DXGI_FORMAT present_format);

		// Record the ray dispatch and presentation commands for the selected screen-space pass.
		void Record(GfxCmdList& cmd_list, Frame& frame, Scene const& scene, RayTracingScene const& ray_tracing_scene, ERayTracingScreenPass pass, RayTracingReflectionBuffer const* reflections, bool restore_present_state);
	};
}
