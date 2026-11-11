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
	// The source descriptors bound to each root descriptor table of one root signature during a draw batch, so that material passes can skip rebinding
	// a table that already holds the same source. Reset it at a new draw batch, or whenever root bindings or descriptor heaps are invalidated.
	struct BoundDescriptors
	{
		// Root signatures have at most 64 parameters
		std::array<Descriptor, 64> m_slots = {};

		// Forget all bindings, so the next bind of each slot is issued.
		void Reset()
		{
			m_slots.fill(Descriptor{});
		}
	};

	// The draw inputs that select a material pass's pipeline state. Selection reads only these inputs and issues no commands, so render steps can
	// compose the same pipeline for drawing, validation, and prewarming.
	struct MaterialPipelineContext
	{
		ERenderStep m_step_id;                         // The render step whose pipeline is being composed.
		Material const& m_material;                    // The material supplying this pass.
		Nugget const& m_nugget;                        // The nugget being drawn.
	};

	// Per-draw context passed to material passes by render steps for binding resources.
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
		Shader* m_shader;                              // The render step's default shader, if the material wants to use it.
		Texture2D* m_default_tex;                      // Fallback diffuse texture for fixed-function style passes.
		Sampler* m_default_sam;                        // Fallback sampler for fixed-function style passes.
		BoundDescriptors* m_bound;                     // Optional draw-batch record of the material descriptor tables already bound.
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

		// Bind resources and constants needed before the draw call. A pass that changes the command-list graphics root signature sets
		// 'ctx.m_root_signature_changed'.
		virtual void Bind(MaterialPassContext& ctx) const;

		// Apply the material's pipeline state to 'desc', after caller-owned pipeline overrides have been applied. The result must depend only on
		// 'ctx' and 'desc'. Throws if the material cannot be drawn with the pipeline state in 'desc'.
		virtual void SelectPipeline(MaterialPipelineContext const& ctx, PipeStateDesc& desc) const;
	};

	// Bind one material descriptor, returning whether a binding was issued. 'bound' is optional; when given, every binding of the root signature's
	// material slots in the draw batch must go through it, so that a skipped binding always leaves the same source bound. Source descriptors must be valid.
	template <typename CmdList, typename Heap, typename RootParam>
	bool BindMaterialDescriptor(CmdList& cmd_list, Heap& heap, RootParam root_param, Descriptor const& source, BoundDescriptors* bound)
	{
		// Compare the copied source identity before hashing or looking it up in the shader-visible heap.
		auto* last = bound != nullptr ? &bound->m_slots[s_cast<size_t>(root_param)] : nullptr;
		if (last != nullptr && *last && *last == source)
			return false;

		auto const gpu_descriptor = heap.Add(source);
		cmd_list.SetGraphicsRootDescriptorTable(root_param, gpu_descriptor);
		if (last != nullptr)
			*last = source;

		return true;
	}
}
