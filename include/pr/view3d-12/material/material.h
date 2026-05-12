//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/sortkey.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12
{
	// Per-draw context passed to material passes by render steps.
	struct MaterialPassContext
	{
		ERenderStep m_step_id;                         // The render step requesting material setup.
		Window& m_wnd;                                 // The window that owns descriptor heaps and frame resources.
		Scene const& m_scene;                          // The scene being rendered.
		DrawListElement const& m_dle;                  // The draw-list element being rendered.
		GfxCmdList& m_cmd_list;                        // The command list to bind resources to.
		GpuUploadBuffer& m_upload;                     // Upload buffer for per-material constants.
		PipeStateDesc& m_pipe_state;                   // Mutable pipeline state description for this draw.
		Shader* m_shader;                              // The render step's default shader, if the material wants to use it.
		Texture2D* m_default_tex;                      // Fallback diffuse texture for fixed-function style passes.
		Sampler* m_default_sam;                        // Fallback sampler for fixed-function style passes.
		D3D12_GPU_DESCRIPTOR_HANDLE* m_last_tex;       // Optional cache of the last diffuse texture descriptor bound.
		D3D12_GPU_DESCRIPTOR_HANDLE* m_last_sam;       // Optional cache of the last diffuse sampler descriptor bound.
	};

	// A render-step specific material implementation.
	struct MaterialPass
	{
		virtual ~MaterialPass() = default;

		// Return true if this material pass needs alpha rendering for 'nugget'.
		virtual bool RequiresAlpha(BaseInstance const& inst, Nugget const& nugget) const;

		// Contribute material state to the draw sort key.
		virtual SortKey AddSortKey(ERenderStep step, BaseInstance const& inst, Nugget const& nugget, SortKey key) const;

		// Bind resources and constants needed before the draw call.
		virtual void Bind(MaterialPassContext& ctx) const;

		// Apply material pipeline state once caller-owned PSO overrides have been applied.
		virtual void ApplyPipeline(MaterialPassContext& ctx) const;
	};

	// Base type for C++ client-defined materials.
	struct Material : RefCounted<Material>
	{
		virtual ~Material() = default;

		// Return the material pass to use for 'step', or null if this material is not drawn by that step.
		virtual MaterialPass const* Pass(ERenderStep step) const = 0;

		// Ref-count clean up function.
		static void RefCountZero(RefCounted<Material>* doomed);

	protected:

		// Delete this material instance.
		virtual void Delete();
	};

	// Built-in material that preserves the legacy inline NuggetDesc material behaviour.
	struct SimpleMaterial : Material
	{
		// Return the shared default material used when a nugget does not specify a custom material.
		static Material const& Default();

		// Return the simple material pass for supported render steps.
		MaterialPass const* Pass(ERenderStep step) const override;
	};

	// Return the shared default material used by nuggets with no explicit material.
	Material const& DefaultMaterial();
}
