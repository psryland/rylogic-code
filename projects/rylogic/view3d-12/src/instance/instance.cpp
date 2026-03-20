//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/sampler/sampler.h"

namespace pr::rdr12
{
	// Return a pointer to the model that this is an instance of.
	ModelPtr const& GetModel(BaseInstance const& inst)
	{
		return inst.get<ModelPtr>(EInstComp::ModelPtr);
	}

	// Return the chain of nuggets to render for this instance
	NuggetPtr GetNuggets(BaseInstance const& inst)
	{
		// Todo: Instances could have their own root nugget pointer component
		auto mdl = GetModel(inst);
		if (mdl == nullptr)
			throw std::runtime_error("Null model pointer");

		return mdl->m_nuggets;
	}

	// Return true if this instance requires alpha blending.
	bool HasAlpha(BaseInstance const& inst)
	{
		// If the instance as an alpha tint, use the alpha nuggets if they exist.
		if (auto* tint = inst.find<Colour32>(EInstComp::TintColour32); tint && HasAlpha(*tint))
			return true;

		// If the instance has a sortkey override that contains alpha, use the alpha nuggets if they exist.
		if (auto* sko = inst.find<SKOverride>(EInstComp::SortkeyOverride); sko && sko->Alpha())
			return true;

		// If the instance has a diffuse texture override that contains alpha, use the alpha nuggets if they exist.
		if (auto* tex = inst.find<Texture2DPtr>(EInstComp::DiffTexture); tex && *tex && (*tex)->Alpha())
			return true;

		return false;
	}

	// Return the instance to world transform for an instance.
	// An instance must have an i2w transform or a shared i2w transform.
	m4x4 const& GetO2W(BaseInstance const& inst)
	{
		auto pi2w = inst.find<m4x4>(EInstComp::I2WTransform);
		if (pi2w)
			return *pi2w;

		auto ppi2w = inst.find<m4x4 const*>(EInstComp::I2WTransformPtr);
		if (ppi2w)
			return **ppi2w;

		auto pi2wf = inst.find<m4x4Func>(EInstComp::I2WTransformFuncPtr);
		if (pi2wf && pi2wf->m_func != nullptr)
			return pi2wf->Txfm();

		return m4x4::Identity();
	}

	// Look for a camera to screen (or instance specific projection) transform for an instance.
	// Returns null if the instance doesn't have one.
	bool FindC2S(BaseInstance const& inst, m4x4& camera_to_screen)
	{
		auto pc2s = inst.find<m4x4>(EInstComp::C2STransform);
		if (pc2s)
		{
			camera_to_screen = *pc2s;
			return true;
		}
				
		auto c2s_optional = inst.find<m4x4>(EInstComp::C2SOptional);
		if (c2s_optional && c2s_optional->x != v4::Zero())
		{
			camera_to_screen = *c2s_optional;
			return true;
		}

		auto ppc2s = inst.find<m4x4 const*>(EInstComp::C2STransformPtr);
		if (ppc2s)
		{
			camera_to_screen = **ppc2s;
			return true;
		}

		auto pc2sf = inst.find<m4x4Func>(EInstComp::C2STransformFuncPtr);
		if (pc2sf && pc2sf->m_func != nullptr)
		{
			camera_to_screen = pc2sf->Txfm();
			return true;
		}

		return false;
	}

	// Return the instance flags associated with 'inst'
	EInstFlag GetFlags(BaseInstance const& inst)
	{
		auto const* flags = inst.find<EInstFlag>(EInstComp::Flags);
		return flags ? *flags : EInstFlag::None;
	}

	// Return the id assigned to this instance, or '0' if not found
	int UniqueId(BaseInstance const& inst)
	{
		auto puid = inst.find<int>(EInstComp::UniqueId);
		return puid ? *puid : 0;
	}

	// Return any pipe state overrides in the instance
	PipeStates const& GetPipeStates(BaseInstance const& inst)
	{
		static PipeStates const NoPipeStates;
		auto pps = inst.find<PipeStates>(EInstComp::PipeStates);
		return pps ? *pps : NoPipeStates;
	}

	// Return the texture override in this instance (if exists)
	Texture2DPtr FindDiffTexture(BaseInstance const& inst)
	{
		auto const* ptex = inst.find<Texture2DPtr>(EInstComp::DiffTexture);
		return ptex ? *ptex : nullptr;
	}

	// Return the sampler override in this instance (if exists)
	SamplerPtr FindDiffTextureSampler(BaseInstance const& inst)
	{
		auto const* psamp = inst.find<SamplerPtr>(EInstComp::DiffTextureSampler);
		return psamp ? *psamp : nullptr;
	}

	// Return the skin override in this instance (if exists)
	PosePtr FindPose(BaseInstance const& inst)
	{
		auto const* pskin = inst.find<PosePtr>(EInstComp::PosePtr);
		return pskin ? *pskin : nullptr;
	}
}