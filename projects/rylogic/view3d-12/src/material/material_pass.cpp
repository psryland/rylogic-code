//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/material/material_pass.h"

namespace pr::rdr12
{
	// Return true if this material pass needs alpha rendering for 'nugget'.
	bool MaterialPass::RequiresAlpha(BaseInstance const&, Material const&, Nugget const&) const
	{
		return false;
	}

	// Contribute material state to the draw sort key.
	SortKey MaterialPass::AddSortKey(ERenderStep, BaseInstance const&, Material const&, Nugget const&, SortKey key) const
	{
		return key;
	}

	// Bind resources and constants needed before the draw call.
	void MaterialPass::Bind(MaterialPassContext&) const
	{}

	// Apply material pipeline state once caller-owned PSO overrides have been applied.
	void MaterialPass::ApplyPipeline(MaterialPassContext&) const
	{}
}
