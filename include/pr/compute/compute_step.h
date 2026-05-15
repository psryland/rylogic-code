//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute
{
	struct ComputeStep
	{
		D3DPtr<ID3D12RootSignature> m_sig;
		D3DPtr<ID3D12PipelineState> m_pso;
	};
}

