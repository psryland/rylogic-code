//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/material_pass.h"

namespace pr::rdr12
{
	// Base type for C++ client-defined materials.
	struct Material : RefCounted<Material>
	{
		virtual ~Material() = default;

		// Return the extensible type id for this material.
		virtual RdrId TypeId() const = 0;

		// Return the material pass to use for 'step', or null if this material is not drawn by that step.
		virtual MaterialPass const* Pass(ERenderStep step) const = 0;

		// Create a mutable copy of this material instance.
		virtual RefPtr<Material> Clone() const = 0;

		// Return true if this material requires alpha rendering
		virtual bool RequiresAlpha() const = 0;

		// Return the material colour that should be folded into the shared nugget tint constant.
		virtual Colour TintColour() const;

		// Return a material component block by component type.
		template <materials::ComponentType T> T const* Component() const
		{
			return static_cast<T const*>(Component(T::Id));
		}
		template <materials::ComponentType T> T* Component()
		{
			return const_call(Component<T>());
		}

		// Return a material component block by component type.
		template <materials::ComponentType T> T const& ComponentOrDefault() const
		{
			if (auto const* c = static_cast<T const*>(Component(T::Id)))
				return *c;
			if (auto const* c = static_cast<T const*>(Default().Component(T::Id)))
				return *c;
			throw std::runtime_error("Default material does not provide the requested component");
		}

		// Ref-count clean up function.
		static void RefCountZero(RefCounted<Material>* doomed);

		// Return the shared default material used by nuggets with no explicit material.
		static Material const& Default();

	protected:

		// Return a component block for 'component_id', or null if this material does not provide that block.
		virtual void const* Component(RdrId component_id) const;

		// Delete this material instance.
		virtual void Delete();
	};

	// Concept for concrete material types addressable through material_cast().
	template <typename T>
	concept MaterialType = std::is_base_of_v<Material, T> && requires
	{
		T::MaterialTypeId;
	};
}
