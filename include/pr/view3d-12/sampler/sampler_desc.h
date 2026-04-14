//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	struct SamplerDesc
	{
		// Notes:
		//  Use 'AutoId' to auto generate a unique id.
		//  Use SamDesc.Id() to generate an id that will match duplicate samplers.

		RdrId    m_id;    // The id to assign to the created sampler instance.
		SamDesc  m_sdesc; // A description of the sampler to be created.
		string32 m_name;  // Debugging name for the sampler.
		
		SamplerDesc()
			:m_id()
			,m_sdesc()
			,m_name()
		{}

		SamplerDesc(RdrId id, SamDesc const& sd)
			:m_id(id)
			,m_sdesc(sd)
			,m_name()
		{}

		SamplerDesc(EStockSampler id, SamDesc const& sd)
			:m_id(s_cast<RdrId>(id))
			,m_sdesc(sd)
			,m_name()
		{}

		SamplerDesc& name(std::string_view name)
		{
			m_name = name;
			return *this;
		}
	};
}
