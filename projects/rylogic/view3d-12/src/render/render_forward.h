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
	private:

		shaders::Forward m_shader;
		GfxCmdList m_cmd_list;
		GfxCmdList m_alpha_cmd_list;
		PipeStateDesc m_alpha_pipe_state;
		Texture2DPtr m_default_tex;
		SamplerPtr m_default_sam;
	
	public:

		explicit RenderForward(Scene& scene);
		~RenderForward();

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RenderForward;

	private:

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Set up shader resources that are common to all nuggets in this render step.
		void BindFrameResources(GfxCmdList& cmd_list);

		void DrawNuggets(GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, bool alpha_pass);

		// Draw a single nugget
		void DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc);
	};
}
