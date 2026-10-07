//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12
{
	struct RenderForward :RenderStep
	{
		using GfxCmdList = ::pr::compute::GfxCmdList;

		// The forward sub-pass that a group of draws belongs to.
		enum class ESubPass
		{
			// Depth-tested opaque output to the main render targets.
			Opaque,

			// Transparent layer collection into the alpha K-buffer.
			Alpha,

			// Background objects blended behind the faded scene, see scene/far_clip_fade.md.
			Background,
		};

	private:

		shaders::Forward m_shader;
		GfxCmdList m_cmd_list;
		GfxCmdList m_alp_list;
		PipeStateDesc m_reflection_pipe_state;
		PipeStateDesc m_alpha_pipe_state;
		PipeStateDesc m_post_alpha_pipe_state;
		D3DPtr<ID3D12RootSignature> m_fade_signature;   // Root signature of the full-screen far clip fade passes
		D3DPtr<ID3D12PipelineState> m_fade_weight_pso;  // Writes the opaque weight to destination alpha before background objects draw
		D3DPtr<ID3D12PipelineState> m_fade_clear_pso;   // Blends the clear colour over faded samples when there are no background objects
		Texture2DPtr m_default_tex;
		SamplerPtr m_default_sam;
		D3D12_GPU_VIRTUAL_ADDRESS m_elements = {}; // This frame's element constants table, one entry per drawlist element
	public:

		explicit RenderForward(Scene& scene);
		~RenderForward();

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RenderForward;

	private:

		// Reject a draw whose material cannot select its forward pixel shaders, such as a mismatched procedural pixel family.
		void ValidateMaterial(DrawListElement const& dle) const;

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Set up shader resources that are common to all nuggets in this render step.
		void BindFrameResources(GfxCmdList& cmd_list);

		// Set up shader resources used by the alpha collection pass.
		void BindAlphaResources(Frame& frame, GfxCmdList& cmd_list);

		// Add the nuggets in the draw list to 'cmd_list' for rendering. 'first_index' is the position of 'drawlist[0]' in the step's drawlist.
		void DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, int first_index, ESubPass sub_pass);

		// Fade the opaque scene into the background objects in 'background', or into the clear colour when there are none.
		// Leaves the depth buffer writable but changes the render targets, viewport, and root signature; the caller restores them.
		void DrawBackgroundFade(Frame& frame, std::span<DrawListElement const> background, int first_index);

		// Draw a single nugget
		void DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc, bool& pipe_state_bound, int& pipe_state_hash);
	};
}
