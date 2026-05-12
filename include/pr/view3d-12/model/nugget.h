//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/render/sortkey.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12
{
	// Notes:
	//  - Structure:
	//    Models contain a pointer to the head of a singly linked list of nuggets. Each nugget points to the next in the list.
	//    Each nugget can also contain a 'variant' which is a secondary list of nuggets derived from that nugget, e.g.,
	//    
	//        Model -> Nugget -> Nugget -> Nugget -> ...   (horizontal -> are 'm_next' pointers)
	//                   |         |         |             (vertical | are 'm_dependent' pointers)
	//                 Nugget    Nugget    Nugget          ('m_next' pointers of dependent nuggets point to the same next nugget as the root)
	//                   |                   |
	//                 Nugget              Nugget
	//
	//    The base chain of nuggets should be for opaque rendering, these nuggets define the geometry and basic properties of
	//    the nugget model and are used in render steps like shadow casting, ray casts, etc. The dependent nuggets are typically
	//    added for alpha rendering but could be used for other purposes, e.g. to add a second pass for outlines, or for alpha
	//    testing after an initial opaque pass. Dependent nuggets should represent the same geometry as the root nugget but with
	//    different shader properties, e.g. alpha blending instead of opaque, or a different texture.
	//
	//  - Alpha: Models can be instanced and instances can have a tint colour that has alpha. This means we need nuggets
	//    for opaque and nuggets for alpha, potentially at the same time. Alpha nuggets are derived procedurally from the
	//    opaque nuggets and added as dependent nuggets, e.g.,
	//
	//       Model--> Nugget(Default) ---> Nugget(Default) ---> ...
	//                   |                    |
	//                Nugget(AlphaFront)   Nugget(AlphaFront)
	//                   |                    |
	//                Nugget(AlphaBack)    Nugget(AlphaBack)
	//
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
	//  - Draw list Sorting and sort keys:
	//    Since there is a draw list per render step, each nugget needs a sort key per draw list. These are composed on demand
	//    when the nuggets are added to the render steps:
	//     - nugget sort key has sort group, alpha, and diff texture id set
	//     - per render step (i.e. per draw list)
	//       - hash the sort ids of all shaders together into a shader id and set that in the sort key
	//       - apply sort key overrides from the owning instance (these are needed because the instance might tint with alpha)
	// 
	//  - ShaderMap:
	//    A nugget contains a collection of ShaderPtrs as well as model specific data. The shader map contains the pointers
	//    to the shaders to be used by each render step. Users can set these pointers as needed for specific functionally or
	//    leave them as null. When a nugget is added to a render step, the render step ensures that there are appropriate
	//    shaders in the shader map for it to be rendered by that render step. If they're missing it adds them.

	using ENuggetVariant = RdrId;
	inline static constexpr ENuggetVariant DefaultNugget = hash::HashCT("DefaultNugget");
	inline static constexpr ENuggetVariant AlphaNugget = hash::HashCT("AlphaNugget");

	// Flags for nuggets. (sync with View3d.cs ENuggetFlag)
	enum class ENuggetFlag :int
	{
		None = 0,

		// Exclude this nugget when rendering a model
		Hidden = 1 << 0,

		// Set if the geometry data for the nugget contains alpha colours
		GeometryHasAlpha = 1 << 1,

		// Force alpha blending for this nugget. Use this when alpha can be supplied by runtime state outside the material.
		AlphaBlend = 1 << 2,

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
		ETopo          m_topo;          // The primitive topology for this nugget
		EGeom          m_geom;          // The valid geometry components within this range
		MaterialPtr    m_material;      // Immutable material that controls render-step specific setup for this nugget
		PipeStates     m_pso;           // A collection of modifications to the pipeline state object description
		ENuggetVariant m_variant;       // An id to allow identification of procedurally added nugget variants
		ENuggetFlag    m_nflags;        // Flags for boolean properties of the nugget
		SortKey        m_sort_key;      // A base sort key for this nugget

		// When passed in to Model->CreateNugget(), these ranges should be relative to the model.
		// If the ranges are invalid, they are assumed to mean the entire model.
		Range m_vrange;
		Range m_irange;

		NuggetDesc(ETopo topo = ETopo::Undefined, EGeom geom = EGeom::Invalid)
			: m_topo(topo)
			, m_geom(geom)
			, m_material()
			, m_pso()
			, m_variant(DefaultNugget)
			, m_nflags(ENuggetFlag::None)
			, m_sort_key(ESortGroup::Default)
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

		// Get/Set the material for this nugget
		Material const& mat() const
		{
			return m_material != nullptr ? *m_material.get() : rdr12::Material::Default();
		}
		virtual NuggetDesc& mat(MaterialPtr material)
		{
			m_material = material;
			return *this;
		}
		
		// Set the material for the nugget
		template <MaterialType M = MaterialSimple>
		NuggetDesc& mat(std::invocable<M&> auto&& cb)
		{
			RefPtr<M> material(rdr12::New<M>(), true);
			cb(*material.get());
			return mat(static_cast<MaterialPtr>(material));
		}

		// Override the pipeline state object for this nugget
		template <EPipeState PS>
		NuggetDesc& pso(pipe_state_field_t<PS> const& value)
		{
			m_pso.Set<PS>(value);
			return *this;
		}

		// Set the flags
		virtual NuggetDesc& flags(ENuggetFlag flags, bool state = true)
		{
			m_nflags = SetBits(m_nflags, flags, state);
			return *this;
		}
		NuggetDesc& alpha_geom(bool has = true)
		{
			m_nflags = SetBits(m_nflags, ENuggetFlag::GeometryHasAlpha, has);
			return *this;
		}
		NuggetDesc& alpha_blend(bool has = true)
		{
			// Request alpha-blended variants for this nugget even if its current material is opaque.
			m_nflags = SetBits(m_nflags, ENuggetFlag::AlphaBlend, has);
			return *this;
		}

		// Id for procedurally added nuggets
		NuggetDesc& variant(ENuggetVariant v)
		{
			m_variant = v;
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

		// True if this nugget requires alpha blending
		bool RequiresAlpha() const
		{
			return
				AnySet(m_nflags, ENuggetFlag::GeometryHasAlpha | ENuggetFlag::AlphaBlend) ||
				mat().RequiresAlpha();
		}
	};

	// A model nugget (i.e. one indivisable piece of geometry to render)
	struct Nugget : NuggetDesc, RefCounted<Nugget>
	{
		// Notes:
		//  - A nugget is a sub range within a model buffer containing any data needed to render
		//    that sub range. Not all data is necessarily needed to render each nugget (depends on
		//    the shader that the render step uses), but each nugget can be rendered with a single
		//    DrawIndexed call for any possible shader.

		Model* m_model; // The model that owns this nugget.
		NuggetPtr m_next; // The next nugget in the owning model's chain of nuggets.
		NuggetPtr m_dependent; // The dependent nuggets associated with this nugget.

		Nugget(NuggetDesc const& ndata, Model* model);
		Nugget(Nugget&&) = delete;
		Nugget(Nugget const&) = delete;
		Nugget& operator =(Nugget&&) = delete;
		Nugget& operator =(Nugget const&) = delete;

		// Renderer access
		Renderer& rdr() const;

		// Set the material used to render this nugget.
		using NuggetDesc::mat;
		Nugget& mat(MaterialPtr material) override;

		// Set nugget flags
		Nugget& flags(ENuggetFlag flags, bool state = true) override;
		
		// The number of primitives in this nugget
		int64_t PrimCount() const;

		// Get/Set the fill mode for this nugget
		EFillMode FillMode() const;
		void FillMode(EFillMode fill_mode);

		// Get/Set the cull mode for this nugget
		ECullMode CullMode() const;
		void CullMode(ECullMode fill_mode);

		// Enumerate this and all dependent nuggets
		auto Dependents() const
		{
			struct I
			{
				Nugget const* n;
				Nugget const& operator*() const { return *n; }
				I& operator++() { if (n) n = n->m_dependent.get(); return *this; }
				bool operator!=(I const& rhs) const { return n != rhs.n; }
			};
			struct R
			{
				Nugget const* n;
				auto begin() const { return I{ n }; }
				auto begin() { return I{ n }; }
				auto end() const { return I{ nullptr }; }
				auto end() { return I{ nullptr }; }
			};
			return R{ this };
		}
		void Dependents(std::invocable<Nugget&> auto&& fn)
		{
			for (auto& n : Dependents())
				fn(n);
		}

		// Attach a dependent nugget to this nugget. The dependent nugget is added to the front of the dependent chain for this nugget.
		void AddDependent(ResourceFactory& factory, NuggetDesc const& ndata);

		// Remove dependent nuggets of the specified variant.
		void DeleteDependents(ENuggetVariant variant);

		// Enable/Disable alpha variant for this nugget.
		// Alpha can be enabled or disabled independent of the geometry colours or diffuse texture colour.
		// When setting 'Alpha(enable)' be sure to consider all sources of alpha.
		bool HasAlphaVariant() const;
		void AlphaVariant(ResourceFactory& factory, bool enable);

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<Nugget>* doomed);
	};

	// Enumerate a chain of nuggets through their 'm_next' pointers
	inline auto Enumerate(Nugget* nugget)
	{
		struct I
		{
			Nugget* n;
			Nugget& operator*() const { return *n; }
			I& operator++() { if (n) n = n->m_next.get(); return *this; }
			bool operator!=(I const& rhs) const { return n != rhs.n; }
		};
		struct R
		{
			Nugget* n;
			auto begin() const { return I{ n }; }
			auto begin() { return I{ n }; }
			auto end() const { return I{ nullptr }; }
			auto end() { return I{ nullptr }; }
		};
		return R{ nugget };
	}
	inline auto Enumerate(NuggetPtr nugget)
	{
		return Enumerate(nugget.get());
	}
}
