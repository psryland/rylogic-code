//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once

#include "pr/physics-2/forward.h"
#include "pr/physics-2/materials/imaterials.h"
#include "pr/physics-2/materials/material.h"

namespace pr::physics
{
	struct MaterialMap : IMaterials
	{
	private:
		std::vector<Material> m_mats;

	public:
		MaterialMap()
			:m_mats()
		{
			// Add a default material
			m_mats.push_back(Material{});
		}

		// Add a material to the collection
		void Add(Material const& mat)
		{
			assert("Material id already exists" && (*this)(mat.m_id).m_id == Material::DefaultID);
			m_mats.push_back(mat);
		}

		// Remove a material by id
		void Remove(int id)
		{
			auto at = std::find_if(std::begin(m_mats), std::end(m_mats), [=](Material const& m){ return m.m_id == id; });
			if (at == std::end(m_mats)) return;
			m_mats.erase(at);
		}

		// Access a material by ID
		Material const& operator()(int id) const
		{
			// todo: sort 'm_mats' and binary search
			for (auto& mat : m_mats)
			{
				if (mat.m_id != id) continue;
				return mat;
			}
			return m_mats[0];
		}
		Material& operator()(int id)
		{
			// todo: sort 'm_mats' and binary search
			for (auto& mat : m_mats)
			{
				if (mat.m_id != id) continue;
				return mat;
			}
			return m_mats[0];
		}

		// IMaterials — return the combined material for two bodies in contact
		Material operator()(int id0, int id1) const override
		{
			auto& mat0 = (*this)(id0);
			auto& mat1 = (*this)(id1);
			return Material::Merge(mat0, mat1);
		}
	};
}
