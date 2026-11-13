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

		// The reflection of the opaque scene in the scene's reflection plane. See reflection_plane.hlsl.
		struct ReflectionPlane;

		// One population draw: the classified lists of a population and a (LOD, nugget) draw within them.
		struct PopulationDrawRef
		{
			PopulationViewLists const* m_lists;
			PopulationViewDraw const* m_draw;
		};

		// The populations classified for one view, and their draws
		struct PopulationSet
		{
			std::vector<PopulationViewLists> m_lists; // The classification of each scene population
			std::vector<DrawListElement> m_dles;      // One element per population draw, referencing the LOD proxies, in sort group order
			std::vector<PopulationDrawRef> m_draws;   // Parallel to 'm_dles'
			D3D12_GPU_VIRTUAL_ADDRESS m_elements = {}; // The element constants table for 'm_dles'
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
		std::unique_ptr<ReflectionPlane> m_reflection_plane;
		PopulationSet m_populations;        // This frame's populations classified for the camera
		PopulationSet m_mirror_populations; // This frame's populations classified for the reflection's mirror camera
		bool m_mirrored = false;            // True while drawing through a mirror camera, which reverses the winding of every triangle
	public:

		explicit RenderForward(Scene& scene);
		~RenderForward();

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RenderForward;

	private:

		// Compose the pipeline state for drawing 'nugget' of 'instance' with 'pass' of 'material', starting from the sub-pass default in 'desc'.
		// 'population' selects the population vertex shaders. 'mirrored' swaps front and back face culling, for views that reverse the winding of triangles.
		// Issues no commands. Throws if the material cannot be drawn in the sub-pass.
		void ComposePipeline(PipeStateDesc& desc, Nugget const& nugget, BaseInstance const& instance, Material const& material, MaterialPass const& pass, bool population, bool mirrored) const;

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add the pipeline states of the draw list and the scene's populations, for each sub-pass that can draw them
		void CollectPipelines(std::vector<PipeStateDesc>& descs) const override;

		// Prepare the sky's low cloud history for this frame: size, swap, and clear the history textures, and set the shader's history constants.
		void PrepareSkyHistory();

		// Project the opaque scene into the reflection of the scene's reflection plane, recording the work into the alpha command list. Sets the
		// shader's reflection constants when the reflection is available, otherwise clears them. Call after the opaque pass, before the alpha draws.
		void ProjectReflectionPlane(Frame& frame);

		// True when this frame needs the reflection of the scene's reflection plane: there is a plane, the camera is above it, and there are
		// targets to project into.
		bool ReflectionPlaneNeeded() const;

		// Set up this frame's reflection mirror camera, and size its targets. See 'ReflectionPlane' for what the mirror view is.
		// Call before the populations are culled, which must include the mirror view.
		void PrepareMirror();

		// Draw the opaque scene and the sky through the reflection's mirror camera. Call after the opaque pass, while the drawlist is still locked.
		void DrawMirror(Frame& frame, std::span<DrawListElement const> opaques);

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Set up shader resources that are common to all nuggets in this render step.
		void BindFrameResources(GfxCmdList& cmd_list);

		// Set up shader resources used by the alpha collection pass.
		void BindAlphaResources(Frame& frame, GfxCmdList& cmd_list);

		// Classify the scene's populations for the camera and the mirror camera, recording the compute work into the opaque command list, and build
		// the population draws.
		void CullPopulations();

		// Classify the scene's populations for 'view' into 'set', and build its draws
		void CullPopulations(PopulationSet& set, PopulationViewDesc const& view);

		// Draw the opaque drawlist elements 'opaques' and the populations in 'pops' into the opaque targets, merged in sort group order.
		// Within one sort group, the drawlist elements are drawn before the populations. Populations whose prototype bounding radius is less
		// than 'min_population_radius' are not drawn.
		void DrawOpaques(Frame& frame, PipeStateDesc const& pipe_state, std::span<DrawListElement const> opaques, PopulationSet const& pops, float min_population_radius);

		// Draw the population draws [beg, end) of 'pops' into the opaque targets, leaving out those with a prototype bounding radius less than
		// 'min_population_radius'. Restores the frame resources of the step's drawlist afterwards.
		void DrawPopulations(Frame& frame, PipeStateDesc const& pipe_state, PopulationSet const& pops, int beg, int end, float min_population_radius);

		// Add the nuggets in the draw list to 'cmd_list' for rendering. 'first_index' is the position of 'drawlist[0]' in the step's drawlist.
		// 'populations' is empty for ordinary draws, or parallel to 'drawlist' for population draws.
		void DrawNuggets(Frame& frame, GfxCmdList& cmd_list, PipeStateDesc const& default_pipe_state, std::span<DrawListElement const> drawlist, int first_index, ESubPass sub_pass, std::span<PopulationDrawRef const> populations = {});

		// Draw a single nugget. 'population' is null for an ordinary draw, otherwise the nugget is drawn indirectly for each classified population instance.
		// 'bound_pipe_state' is the state bound to 'cmd_list' while 'pipe_state_bound' is true. Both are updated when the bound state changes.
		void DrawNugget(GfxCmdList& cmd_list, Nugget const& nugget, PipeStateDesc& desc, bool& pipe_state_bound, PipeStateDesc& bound_pipe_state, PopulationDrawRef const* population);
	};
}
