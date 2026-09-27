//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/sortkey.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12
{
	// Per-draw context passed to material passes by render steps.
	struct MaterialPassContext
	{
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;

		ERenderStep m_step_id;                         // The render step requesting material setup.
		Window& m_wnd;                                 // The window that owns descriptor heaps and frame resources.
		Scene const& m_scene;                          // The scene being rendered.
		CameraTransforms const& m_camera;              // Borrowed camera transforms owned by the current render pass.
		DrawListElement const& m_dle;                  // The draw-list element being rendered.
		Material const& m_material;                    // The material supplying this pass.
		GfxCmdList& m_cmd_list;                        // The command list to bind resources to.
		GpuUploadBuffer& m_upload;                     // Upload buffer for per-material constants.
		PipeStateDesc& m_pipe_state;                   // Mutable pipeline state description for this draw.
		Shader* m_shader;                              // The render step's default shader, if the material wants to use it.
		Texture2D* m_default_tex;                      // Fallback diffuse texture for fixed-function style passes.
		Sampler* m_default_sam;                        // Fallback sampler for fixed-function style passes.
		Descriptor* m_last_tex;                        // Optional draw-batch copy of the last bound diffuse source descriptor.
		Descriptor* m_last_sam;                        // Optional draw-batch copy of the last bound diffuse sampler descriptor.
		bool m_root_signature_changed = false;         // True if the pass changed the command-list graphics root signature.
	};

	// A render-step specific material implementation.
	struct MaterialPass
	{
		virtual ~MaterialPass() = default;

		// Return true if this material pass needs alpha rendering for 'nugget'.
		virtual bool RequiresAlpha(BaseInstance const& inst, Material const& material, Nugget const& nugget) const;

		// Contribute material state to the draw sort key.
		virtual SortKey AddSortKey(ERenderStep step, BaseInstance const& inst, Material const& material, Nugget const& nugget, SortKey key) const;

		// Bind resources and constants needed before the draw call.
		virtual void Bind(MaterialPassContext& ctx) const;

		// Apply material pipeline state once caller-owned PSO overrides have been applied.
		virtual void ApplyPipeline(MaterialPassContext& ctx) const;
	};

	// Bind one material descriptor, returning whether a binding was issued. Optional tracking belongs to one root slot and GPU heap;
	// reset it at a new draw batch or whenever root bindings or descriptor heaps are invalidated. Source descriptors must be valid.
	template <typename CmdList, typename Heap, typename RootParam>
	bool BindMaterialDescriptor(CmdList& cmd_list, Heap& heap, RootParam root_param, Descriptor const& source, Descriptor* last)
	{
		// Compare the copied source identity before hashing or looking it up in the shader-visible heap.
		if (last != nullptr && *last && *last == source)
			return false;

		auto const gpu_descriptor = heap.Add(source);
		cmd_list.SetGraphicsRootDescriptorTable(root_param, gpu_descriptor);
		if (last != nullptr)
			*last = source;

		return true;
	}
}
