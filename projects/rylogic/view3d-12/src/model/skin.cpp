//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/resource_store.h"

namespace pr::rdr12
{
	using ResDesc = ::pr::compute::ResDesc;

	Skin::Skin()
		: m_res()
		, m_srv()
		, m_skel_id()
		, m_influence_count()
		, m_max_bone_index()
		, m_min_skin_index()
		, m_max_skin_index(-1)
	{
	}
	Skin::Skin(ResourceFactory& factory, std::span<Skinfluence const> verts, uint32_t skel_id, int min_skin_index, int max_skin_index)
		: m_res()
		, m_srv()
		, m_skel_id(skel_id)
		, m_influence_count(isize(verts))
		, m_max_bone_index()
		, m_min_skin_index(min_skin_index)
		, m_max_skin_index(max_skin_index)
	{
		for (auto const& influence : verts)
		{
			for (auto bone_index : influence.m_bones)
				m_max_bone_index = std::max(m_max_bone_index, static_cast<int>(static_cast<uint16_t>(bone_index)));
		}

		ResourceStore::Access store(factory.rdr());

		// Create the buffer for the vertex bone weights
		ResDesc rdesc = ResDesc::Buf<Skinfluence>(isize(verts), verts).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		m_res = factory.CreateResource(rdesc, "skin");

		// Create the skin SRV
		D3D12_SHADER_RESOURCE_VIEW_DESC srv_desc = {
			.Format = DXGI_FORMAT::DXGI_FORMAT_UNKNOWN,
			.ViewDimension = D3D12_SRV_DIMENSION::D3D12_SRV_DIMENSION_BUFFER,
			.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
			.Buffer = {
				.FirstElement = 0,
				.NumElements = s_cast<UINT>(isize(verts)),
				.StructureByteStride = sizeof(Skinfluence),
				.Flags = D3D12_BUFFER_SRV_FLAGS::D3D12_BUFFER_SRV_FLAG_NONE,
			},
		};
		m_srv = store.Descriptors().Create(m_res.get(), srv_desc);
	}
}
