//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/materials/material.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	struct MaterialMap
	{
	private:

		// Materials need to be a contiguous array for GPU upload, but also have stable ids for lookup.
		// Shapes contain a material id
		std::vector<GpuMaterial> m_mats;

		// Range check the material id
		void Check(int id) const
		{
			if (id < 0 || id >= Material::MaxMaterialId)
				throw std::out_of_range("Material ID is out of range");
		}

	public:
		MaterialMap()
			:m_mats(Material::MaxMaterialId)
		{
			// Set a sensible default material at ID 0
			Set(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 1.0f,
			});
		}

		// Add a material to the collection
		void Set(Material mat)
		{
			Check(mat.m_id);
			m_mats[mat.m_id] = GpuMaterial{
				.friction_static = mat.m_friction_static,
				.elasticity_norm = mat.m_elasticity_norm,
				.elasticity_tang = mat.m_elasticity_tang,
				.elasticity_tors = mat.m_elasticity_tors,
				.density = mat.m_density,
			};
		}

		// Access a material by ID
		Material operator[](int id) const
		{
			Check(id);
			return Material{
				id,
				m_mats[id].friction_static,
				m_mats[id].elasticity_norm,
				m_mats[id].elasticity_tang,
				m_mats[id].elasticity_tors,
				m_mats[id].density,
			};
		}

		// IMaterials — return the combined material for two bodies in contact
		Material operator()(int id0, int id1) const
		{
			auto mat0 = (*this)[id0];
			auto mat1 = (*this)[id1];
			return Material::Merge(mat0, mat1);
		}

		// The range of materials
		std::span<GpuMaterial const> span() const
		{
			return m_mats;
		}
	};

	inline void Deleter<MaterialMap>::operator()(MaterialMap* p) const
	{
		delete p;
	}
}
