//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/material/material.h"
#include "pr/view3d-12/material/material_simple.h"

namespace pr::rdr12
{
	// Ref-count clean up function.
	void Material::RefCountZero(RefCounted<Material>* doomed)
	{
		auto* material = static_cast<Material*>(doomed);
		material->Delete();
	}

	// Return a component block for 'component_id', or null if this material does not provide that block.
	void const* Material::Component(RdrId) const
	{
		return nullptr;
	}

	// Delete this material instance.
	void Material::Delete()
	{
		::pr::compute::Delete<Material>(this);
	}

	// Return the shared default material used by nuggets with no explicit material.
	Material const& Material::Default()
	{
		static MaterialSimple material;
		return material;
	}
}
