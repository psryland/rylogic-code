//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	enum class EStockShader : RdrId
	{
		#define PR_ENUM(x)\
		x(Invalid         , = InvalidId)\
		x(Forward         ,)\
		x(FwdShaderVS     ,)\
		x(FwdShaderPS     ,)\
		x(FwdRadialFadePS ,)\
		x(ShadowMapVS     ,)\
		x(ShadowMapPS     ,)\
		x(PointSpritesGS  ,)\
		x(ThickLineListGS ,)\
		x(ThickLineStripGS,)\
		x(ArrowHeadGS     ,)\
		x(ShowNormalsGS   ,)
		PR_ENUM_MEMBERS2(PR_ENUM)
	};
	PR_ENUM_REFLECTION2(EStockShader, PR_ENUM);
	#undef PR_ENUM
}