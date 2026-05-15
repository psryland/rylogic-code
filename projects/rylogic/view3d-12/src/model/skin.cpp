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
	{
	}
	Skin::Skin(ResourceFactory& factory, std::span<Skinfluence const> verts, uint32_t skel_id)
		: m_res()
		, m_srv()
		, m_skel_id(skel_id)
	{
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
