//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
//#include "pr/view3d/render/state_block.h"
#include "pr/view3d-12/render/sortkey.h"
//#include "pr/view3d/render/drawlist_element.h"
//#include "pr/view3d/shaders/shader_set.h"
#include "pr/view3d-12/shaders/shader.h"
//#include "pr/view3d/models/model_buffer.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/utility/pipe_state.h"
//#include "pr/view3d/textures/texture_cube.h"

namespace pr::rdr12
{
	// Notes:
	//  - Shader/Nugget Requirements:
	//    There is some data that is model specific and used by multiple shaders (e.g. topology, geom type, diffuse texture),
	//    these data might as well be in the nuggets to prevent duplication in each shader.
	//    Usability requires that we can add a model (i.e. a collection of nuggets) to any/all render steps automatically.
	//    Normally, render steps have a shader they want to use but sometimes we need to override the shader a render step uses.
	//    We don't want to have to resolve the shaders per frame.
	//
	//  - Render Steps:
	//    Nuggets may be referenced in the draw lists of several render steps. i.e. each render step has
	//    its own draw list, so the same nugget can be pointed to from multiple draw lists.
	//    This leads to the conclusion that a nugget shouldn't contain shader specific data (e.g. why should all nuggets have a
	//    variable only used in one shader from one render step? This wouldn't scale as more shaders/render steps are added)
	//    Shader derived objects are light weight instances of DX shaders. These shader instances contain per-nugget data
	//    (such as line width, projection texture, etc). They can be duplicated as needed.
	//    
	//    Draw list Sorting and sort keys:
	//    Since there is a draw list per render step, each nugget needs a sort key per draw list. These are composed on demand
	//    when the nuggets are added to the render steps:
	//     - nugget sort key has sort group, alpha, and diff texture id set
	//     - per render step (aka draw list)
	//       - hash the sort ids of all shaders together into a shader id and set that in the sort key
	//       - apply sort key overrides from the owning instance (these are needed because the instance might tint with alpha)
	//
	//  - ShaderMap:
	//    A nugget contains a collection of ShaderPtrs as well as model specific data. The shader map contains the pointers
	//    to the shaders to be used by each render step. Users can set these pointers as needed for specific functionally or
	//    leave them as null. When a nugget is added to a render step, the render step ensures that there are appropriate
	//    shaders in the shader map for it to be rendered by that render step. If they're missing it adds them.

	inline static constexpr RdrId AlphaNuggetId = hash::HashCT("AlphaNugget");

	// Flags for nuggets. (sync with View3d.cs ENuggetFlag)
	enum class ENuggetFlag :int
	{
		None = 0,

		// Exclude this nugget when rendering a model
		Hidden = 1 << 0,

		// Set if the geometry data for the nugget contains alpha colours
		GeometryHasAlpha = 1 << 1,

		// Set if the tint colour contains alpha
		TintHasAlpha = 1 << 2,

		// Set if the diffuse texture contains alpha (and we want alpha blending, not just thresholding)
		TexDiffuseHasAlpha = 1 << 3,

		// Excluded from shadow map render steps
		ShadowCastExclude = 1 << 4,

		// Can overlap with other nuggets.
		// Set this flag to true if you want to add a nugget that overlaps the range
		// of an existing nugget. For simple models, overlapping nugget ranges is
		// usually an error, but in advanced cases it isn't.
		RangesCanOverlap = 1 << 5,

		_flags_enum = 0,
	};

	// Nugget initialisation data
	struct NuggetDesc
	{
		// Each overlay can replace stages in the full shader. So, order is important.
		// If multiple overlays replace the same stage, the last one wins.
		struct ShdrOverlay
		{
			// Notes:
			//  - Normally overlays just replace one stage in the base shader, but it is possible for an overlay to have
			//    a full signature and replace all stages. As the overlays are on the nugget, however, both the 'SetupFrame'
			//    and 'SetupElement' method will be called for each nugget.
			mutable ShaderPtr m_overlay = {};  // Individual stages that replace stages in the full shader
			ERenderStep m_rdr_step = ERenderStep::Invalid; // The render step that the shader applies to.
			int pad = {};
		};
		using ShdrOverlays = vector<ShdrOverlay, 4, false>;

		ETopo        m_topo;          // The primitive topology for this nugget
		EGeom        m_geom;          // The valid geometry components within this range
		Texture2DPtr m_tex_diffuse;   // Diffuse texture
		SamplerPtr   m_sam_diffuse;   // The sampler to use with the diffuse texture
		ShdrOverlays m_shdr_overlays; // Shader overlays, applied in order to overlay the base shader
		PipeStates   m_pso;           // A collection of modifications to the pipeline state object description
		RdrId        m_id;            // An id to allow identification of procedurally added nuggets
		ENuggetFlag  m_nflags;        // Flags for boolean properties of the nugget
		Colour32     m_tint;          // Per-nugget tint
		SortKey      m_sort_key;      // A base sort key for this nugget
		float        m_rel_reflec;    // How reflective this nugget is, relative to the instance. Note: 1.0 means the same as the instance (which might be 0)

		// When passed in to Model->CreateNugget(), these ranges should be relative to the model.
		// If the ranges are invalid, they are assumed to mean the entire model.
		Range           m_vrange;
		Range           m_irange;

		NuggetDesc(ETopo topo = ETopo::Undefined, EGeom geom = EGeom::Invalid)
			: m_topo(topo)
			, m_geom(geom)
			, m_tex_diffuse()
			, m_sam_diffuse()
			, m_shdr_overlays()
			, m_pso()
			, m_id(AutoId)
			, m_nflags(ENuggetFlag::None)
			, m_tint(Colour32White)
			, m_sort_key(ESortGroup::Default)
			, m_rel_reflec(1)
			, m_vrange(Range::Reset())
			, m_irange(Range::Reset())
		{}

		// Set the nugget topology
		NuggetDesc& topo(ETopo topology)
		{
			m_topo = topology;
			return *this;
		}

		// Set the nugget vertex format
		NuggetDesc& geom(EGeom geometry)
		{
			m_geom = geometry;
			return *this;
		}

		// Set the vertex range for this nugget
		NuggetDesc& vrange(Range range)
		{
			m_vrange = range;
			return *this;
		}
		NuggetDesc& vrange(int64_t beg, int64_t end)
		{
			return vrange(Range(beg, end));
		}

		// Set the index range for this nugget
		NuggetDesc& irange(Range range)
		{
			m_irange = range;
			return *this;
		}
		NuggetDesc& irange(int64_t beg, int64_t end)
		{
			return irange(Range(beg, end));
		}

		// Add/overide a shader for this nugget
		NuggetDesc& use_shader_overlay(ERenderStep step, ShaderPtr overlay)
		{
			m_shdr_overlays.push_back({overlay, step});
			return *this;
		}

		// Override the pipeline state object for this nugget
		template <EPipeState PS>
		NuggetDesc& pso(pipe_state_field_t<PS> const& value)
		{
			m_pso.Set<PS>(value);
			return *this;
		}

		// Set the diffuse texture for the nugget
		NuggetDesc& tex_diffuse(Texture2DPtr tex)
		{
			m_tex_diffuse = tex;
			return flags(ENuggetFlag::TexDiffuseHasAlpha, AllSet(tex ? tex->m_tflags : ETextureFlag::None, ETextureFlag::HasAlpha));
		}

		// Set the sampler for the diffuse texture
		NuggetDesc& sam_diffuse(SamplerPtr sam)
		{
			m_sam_diffuse = sam;
			return *this;
		}

		// Set the tint colour
		NuggetDesc& tint(Colour32 tint)
		{
			m_tint = tint;
			alpha_tint(HasAlpha(tint));
			return *this;
		}

		// Set the flags
		NuggetDesc& flags(ENuggetFlag flags, bool state = true)
		{
			m_nflags = SetBits(m_nflags, flags, state);
			return *this;
		}
		NuggetDesc& alpha_geom(bool has = true)
		{
			m_nflags = SetBits(m_nflags, ENuggetFlag::GeometryHasAlpha, has);
			return *this;
		}
		NuggetDesc& alpha_tint(bool has = true)
		{
			m_nflags = SetBits(m_nflags, ENuggetFlag::TintHasAlpha, has);
			return *this;
		}
		NuggetDesc& alpha_tex(bool has = true)
		{
			m_nflags = SetBits(m_nflags, ENuggetFlag::TexDiffuseHasAlpha, has);
			return *this;
		}

		// Id for procedurally added nuggets
		NuggetDesc& id(RdrId id)
		{
			m_id = id;
			return *this;
		}

		// Set the sort key for this nugget
		NuggetDesc& sort_key(SortKey key)
		{
			m_sort_key = key;
			return *this;
		}
		NuggetDesc& sort_key(ESortGroup group)
		{
			m_sort_key.Group(group);
			return *this;
		}

		// Set the relative reflectivity for this nugget
		NuggetDesc& rel_reflec(float reflectivity)
		{
			m_rel_reflec = reflectivity;
			return *this;
		}
	};

	// A nugget is a sub range within a model buffer containing any data needed to render
	// that sub range. Not all data is necessarily needed to render each nugget (depends on
	// the shader that the render step uses), but each nugget can be rendered with a single
	// DrawIndexed call for any possible shader.
	struct Nugget :chain::link<Nugget, ChainGroupNugget>, NuggetDesc
	{
		Model*       m_model;         // The model that owns this nugget.
		TNuggetChain m_nuggets;       // The dependent nuggets associated with this nugget.

		Nugget(NuggetDesc const& ndata, Model* model);
		~Nugget();

		// Renderer access
		Renderer& rdr() const;

		// The number of primitives in this nugget
		int64_t PrimCount() const;

		// True if this nugget requires alpha blending
		bool RequiresAlpha() const;
		void UpdateAlphaStates();

		// Get/Set the fill mode for this nugget
		EFillMode FillMode() const;
		void FillMode(EFillMode fill_mode);

		// Get/Set the cull mode for this nugget
		ECullMode CullMode() const;
		void CullMode(ECullMode fill_mode);

		// Delete any dependent nuggets based on 'pred'
		template <typename Pred>
		void DeleteDependent(Pred pred)
		{
			auto nuggets = chain::filter(m_nuggets, pred);
			for (;!nuggets.empty();)
				nuggets.front().Delete();
		}

		// Delete this nugget, removing it from the owning model
		void Delete();

	private:

		// Enable/Disable alpha for this nugget.
		// Alpha can be enabled or disabled independent of the geometry colours or diffuse texture colour.
		// When setting 'Alpha(enable)' be sure to consider all sources of alpha.
		void Alpha(bool enable);
	};
}