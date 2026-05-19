//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Influence data for a single vertex in a mesh
	struct Skinfluence
	{
		int16_t m_bones[8];
		uint16_t m_weights[8]; // normalised weight [0,1] = [0,65535]
	};

	// Data required to skin a mesh
	struct Skin
	{
		using Descriptor = ::pr::compute::Descriptor;

		// See description in "animation.h"
		D3DPtr<ID3D12Resource> m_res; // Buffer of 'Skinfluence[]'
		Descriptor m_srv;             // SRV of the skin influence buffer
		uint32_t m_skel_id;           // The skeleton that this skin is matched with.
		int m_influence_count;        // The number of influence records in 'm_res'
		int m_max_bone_index;         // The maximum bone index referenced by the packed influence data
		int m_min_skin_index;         // The minimum influence index referenced by the model vertices. Can be -1 for dead vertices.
		int m_max_skin_index;         // The maximum influence index referenced by the model vertices

		Skin();
		Skin(ResourceFactory& factory, std::span<Skinfluence const> verts, uint32_t skel_id, int min_skin_index, int max_skin_index);

		// True if 'has skin'
		explicit operator bool() const
		{
			return m_res != nullptr;
		}
	};
}
