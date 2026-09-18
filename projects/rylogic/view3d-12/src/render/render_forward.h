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

	private:

		shaders::Forward m_shader;
		GfxCmdList m_cmd_list;
		GfxCmdList m_alp_list;
		PipeStateDesc m_reflection_pipe_state;
		PipeStateDesc m_alpha_pipe_state;
		PipeStateDesc m_post_alpha_pipe_state;
		Texture2DPtr m_default_tex;
		SamplerPtr m_default_sam;
	
	public:

		explicit RenderForward(Scene& scene);
		~RenderForward();

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RenderForward;

	private:

		friend struct Scene;

		// Reject unsupported material output before a scene option change or frame recording.
		void ValidateFarClipFade();
		void ValidateFarFadeMaterial(DrawListElement const& dle) const;

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Set up shader resources that are common to all nuggets in this render step.
		void BindFrameResources(GfxCmdList& cmd_list);

		// Set up shader resources used by the alpha collection pass.
		void BindAlphaResources(Frame& frame, GfxCmdList& cmd_list);

		// Add the nuggets in the draw list to 'cmd_list' for rendering.
		void DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, bool alpha_pass);

		// Select the stock output contract after overrides, rejecting unsupported far-fade pipelines.
		void ApplyFarFadePipeline(PipeStateDesc& desc, bool alpha_pass) const;

		// Return true only when undeformed model bounds prove there is no fading opaque coverage.
		bool IsBeforeFarFade(DrawListElement const& dle, PipeStateDesc const& desc) const;

		// Draw a single nugget using its resolved scene ordering contract.
		void DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, ESortGroup sort_group, bool alpha_pass, bool fade_world, PipeStateDesc& desc, bool& pipe_state_bound, int& pipe_state_hash);
	};
}
