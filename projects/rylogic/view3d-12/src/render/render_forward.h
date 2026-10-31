//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/population/population.h"
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
		};

	private:

		// The procedural sky's low cloud accumulated over frames. See AccumulateLowCloud in procedural_sky.hlsl.
		struct SkyHistory;

		// One population draw: the classified lists of a population and a (LOD, nugget) draw within them.
		struct PopulationDrawRef
		{
			PopulationViewLists const* m_lists;
			PopulationViewDraw const* m_draw;
		};

		shaders::Forward m_shader;
		GfxCmdList m_cmd_list;
		GfxCmdList m_alp_list;
		PipeStateDesc m_reflection_pipe_state;
		PipeStateDesc m_alpha_pipe_state;
		PipeStateDesc m_post_alpha_pipe_state;
		Texture2DPtr m_default_tex;
		SamplerPtr m_default_sam;
		D3D12_GPU_VIRTUAL_ADDRESS m_elements = {}; // This frame's element constants table, one entry per drawlist element
		std::unique_ptr<SkyHistory> m_sky_history;
		std::vector<PopulationViewLists> m_population_lists; // This frame's classification of each scene population
		std::vector<DrawListElement> m_population_dles;      // One element per population draw, referencing the LOD proxies
		std::vector<PopulationDrawRef> m_population_draws;   // Parallel to 'm_population_dles'
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

		// Prepare the sky's low cloud history for this frame: size, swap, and clear the history textures, and set the shader's history constants.
		void PrepareSkyHistory();

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Set up shader resources that are common to all nuggets in this render step.
		void BindFrameResources(GfxCmdList& cmd_list);

		// Set up shader resources used by the alpha collection pass.
		void BindAlphaResources(Frame& frame, GfxCmdList& cmd_list);

		// Classify the scene's populations for the camera, recording the compute work into the opaque command list, and build the population draws.
		void CullPopulations();

		// Draw the classified populations into the opaque targets. Restores the frame resources of the step's drawlist afterwards.
		void DrawPopulations(Frame& frame, PipeStateDesc const& pipe_state);

		// Add the nuggets in the draw list to 'cmd_list' for rendering. 'first_index' is the position of 'drawlist[0]' in the step's drawlist.
		// 'populations' is empty for ordinary draws, or parallel to 'drawlist' for population draws.
		void DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, int first_index, ESubPass sub_pass, std::span<PopulationDrawRef const> populations = {});

		// Draw a single nugget. 'population' is null for an ordinary draw, otherwise the nugget is drawn indirectly for each classified population instance.
		void DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc, bool& pipe_state_bound, int& pipe_state_hash, PopulationDrawRef const* population);
	};
}
