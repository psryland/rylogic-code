//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/resource_store.h"

namespace pr::rdr12
{
	Nugget::Nugget(NuggetDesc const& ndata, Model* model)
		: NuggetDesc(ndata)
		, m_model(model)
		, m_next()
		, m_dependent()
	{
		// Fix the initial pipe state overrides
		m_pso.m_fixed = m_pso.count();

		// Enable alpha if the geometry or the diffuse texture map contains alpha
		if (m_variant == DefaultNugget && RequiresAlpha())
		{
			ResourceFactory factory(model->rdr());
			AlphaVariant(factory, true);
		}
	}

	// Renderer access
	Renderer& Nugget::rdr() const
	{
		return m_model->rdr();
	}

	// Set the material used to render this nugget.
	Nugget& Nugget::mat(MaterialPtr material)
	{
		if (m_material == material)
			return *this;

		auto had_alpha_variant = m_variant == DefaultNugget && HasAlphaVariant();

		NuggetDesc::mat(material);

		// Recreate alpha variants so they inherit the new material. Existing alpha variants are preserved because instance-level alpha can still select them.
		if (m_variant == DefaultNugget && (had_alpha_variant || RequiresAlpha()))
		{
			ResourceFactory factory(rdr());
			AlphaVariant(factory, true);
		}

		m_model->m_ray_tracing.Invalidate(rdr());
		return *this;
	}

	// Set nugget flags
	Nugget& Nugget::flags(ENuggetFlag flags, bool state)
	{
		auto old_flags = m_nflags;
		NuggetDesc::flags(flags, state);
		if (m_nflags == old_flags)
			return *this;

		if (m_variant == DefaultNugget)
		{
			// Dependent variants inherit root nugget visibility/filter state. AlphaBlend is only a root selection hint, so clearing it does not remove existing alpha variants.
			for (auto* dep = m_dependent.get(); dep != nullptr; dep = dep->m_dependent.get())
				dep->NuggetDesc::flags(flags, state);
		}

		// On flags changed, see if an AlphaVariant is needed
		if (m_variant == DefaultNugget && !HasAlphaVariant() && RequiresAlpha())
		{
			ResourceFactory factory(m_model->rdr());
			AlphaVariant(factory, true);
		}

		m_model->m_ray_tracing.Invalidate(m_model->rdr());
		return *this;
	}

	// The number of primitives in this nugget
	int64_t Nugget::PrimCount() const
	{
		return m_irange.empty()
			? ::pr::compute::PrimCount(m_vrange.size(), m_topo)
			: ::pr::compute::PrimCount(m_irange.size(), m_topo);
	}

	// Get/Set the fill mode for this nugget
	EFillMode Nugget::FillMode() const
	{
		auto fill_mode = m_pso.Find<EPipeState::FillMode>();
		return fill_mode ? s_cast<EFillMode>(*fill_mode) : EFillMode::Default;
	}
	void Nugget::FillMode(EFillMode fill_mode)
	{
		if (FillMode() == fill_mode)
			return;

		m_pso.Clear<EPipeState::FillMode>();
		if (fill_mode != EFillMode::Default)
			m_pso.Set<EPipeState::FillMode>(s_cast<D3D12_FILL_MODE>(fill_mode));

		// Apply recursively
		if (m_dependent)
			m_dependent->FillMode(fill_mode);
	}

	// Get/Set the cull mode for this nugget
	ECullMode Nugget::CullMode() const
	{
		auto cull_mode = m_pso.Find<EPipeState::CullMode>();
		return cull_mode ? s_cast<ECullMode>(*cull_mode) : ECullMode::Default;
	}
	void Nugget::CullMode(ECullMode cull_mode)
	{
		// Alpha rendering nuggets already have the cull mode set.
		if (m_variant == AlphaNugget)
			return;

		if (CullMode() == cull_mode)
			return;

		m_pso.Clear<EPipeState::CullMode>();
		if (cull_mode != ECullMode::Default)
			m_pso.Set<EPipeState::CullMode>(s_cast<D3D12_CULL_MODE>(cull_mode));

		// Apply recursively
		if (m_dependent)
			m_dependent->CullMode(cull_mode);
	}

	// Attach a dependent nugget to this nugget. The dependent nugget is added to the end of the dependent chain for this nugget.
	void Nugget::AddDependent(ResourceFactory& factory, NuggetDesc const& ndata)
	{
		auto dep = factory.CreateNugget(ndata, m_model);
		dep->m_next = m_next;
		dep->m_dependent = std::move(m_dependent);
		m_dependent = std::move(dep);
	}

	// Remove dependent nuggets of the specified variant.
	void Nugget::DeleteDependents(ENuggetVariant variant)
	{
		assert(m_variant == DefaultNugget && "Show only be called from the root nugget");

		for (auto* ptr = &m_dependent; *ptr; )
		{
			if ((*ptr)->m_variant == variant)
				*ptr = std::move((*ptr)->m_dependent);
			else
				ptr = &(*ptr)->m_dependent;
		}
	}

	// Enable/Disable alpha for this nugget
	bool Nugget::HasAlphaVariant() const
	{
		// If the first nugget does, then all will
		return std::any_of(Dependents().begin(), Dependents().end(), [](auto& n) { return n.m_variant == AlphaNugget; });
	}
	void Nugget::AlphaVariant(ResourceFactory& factory, bool enable)
	{
		// Should only enable/disable the alpha variant from the default nugget
		assert(m_variant == DefaultNugget && "Show only be called from the root nugget");

		// Don't check if already enabled/disabled because enabling
		// should recreate the alpha variants even if they already exist.

		// Note: typically the alpha variant would be enabled and never disabled because
		// we don't know how many instances are referencing this model/nugget. Any of those
		// instances might still be using the alpha variant.

		// Remove the alpha variants if they exist
		DeleteDependents(AlphaNugget);

		if (!enable)
			return;

		// Create the AlphaFront nugget
		{
			AddDependent(factory,
				NuggetDesc(*this)
				.variant(AlphaNugget)
				.sort_key(ESortGroup::AlphaFront)
				.pso<EPipeState::CullMode>(D3D12_CULL_MODE_BACK)
				.pso<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO)
				.pso<EPipeState::BlendState0>({
					.BlendEnable = TRUE,
					.LogicOpEnable = FALSE,
					.SrcBlend = D3D12_BLEND_SRC_ALPHA,      // Alpha is always drawn over opaque pixels, so the dest
					.DestBlend = D3D12_BLEND_INV_SRC_ALPHA, // alpha is always 1. Blend the RGB using the src alpha.
					.BlendOp = D3D12_BLEND_OP_ADD,          // And write the dest alpha as one
					.SrcBlendAlpha = D3D12_BLEND_ONE,
					.DestBlendAlpha = D3D12_BLEND_ONE,
					.BlendOpAlpha = D3D12_BLEND_OP_MAX,
					.LogicOp = D3D12_LOGIC_OP_CLEAR,
					.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL,
					}));
		}

		// Create the AlphaBack nugget. Only triangle data needs back faces rendered
		if (TopoGroup(m_topo) == ETopoGroup::Triangles)
		{
			AddDependent(factory,
				NuggetDesc(*this)
				.variant(AlphaNugget)
				.sort_key(ESortGroup::AlphaBack)
				.pso<EPipeState::CullMode>(D3D12_CULL_MODE_FRONT)
				.pso<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO)
				.pso<EPipeState::BlendState0>({
					.BlendEnable = TRUE,
					.LogicOpEnable = FALSE,
					.SrcBlend = D3D12_BLEND_SRC_ALPHA,      // Alpha is always drawn over opaque pixels, so the dest
					.DestBlend = D3D12_BLEND_INV_SRC_ALPHA, // alpha is always 1. Blend the RGB using the src alpha.
					.BlendOp = D3D12_BLEND_OP_ADD,          // And write the dest alpha as one
					.SrcBlendAlpha = D3D12_BLEND_ONE,
					.DestBlendAlpha = D3D12_BLEND_ONE,
					.BlendOpAlpha = D3D12_BLEND_OP_MAX,
					.LogicOp = D3D12_LOGIC_OP_CLEAR,
					.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL,
					}));
		}
	}

	// Ref-counting clean up function
	void Nugget::RefCountZero(RefCounted<Nugget>* doomed)
	{
		auto* ngt = static_cast<Nugget*>(doomed);
		ResourceStore::Access store(ngt->m_model->rdr());
		store.Delete(ngt);
	}
}
