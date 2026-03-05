//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/utility/globalfunctions.h"

// Transform an inertia tensor using the parallel axis theorem.
// 'offset' is the distance from (or toward) the centre of mass (determined by 'translate_type')
// 'inertia' and 'offset' must be in the same frame.
void pr::ph::ParallelAxisTranslateInertia(m3x4& inertia, pr::v4 offset, float mass, ParallelAxisTranslate::Type translate_type)
{
	if (translate_type == ParallelAxisTranslate::TowardCoM)
		mass = -mass;
	
	for (uint32_t i = 0; i != 3; ++i)
	{
		for (uint32_t j = i; j != 3; ++j)
		{
			// For the diagonal elements I = Io + md^2 (away from CoM), Io = I - md^2 (toward CoM)
			// 'd' is the perpendicular component of 'offset'
			if (i == j)
			{
				uint32_t i1 = (i + 1) % 3;
				uint32_t i2 = (i + 2) % 3;
				inertia[i][i] += mass * (offset[i1]*offset[i1] + offset[i2]*offset[i2]);
			}
			
			// For off diagonal elements:
			//  Ixy = Ioxy + mdxdy  (away from CoM), Io = I - mdxdy (toward CoM)
			//  Ixz = Ioxz + mdxdz  (away from CoM), Io = I - mdxdz (toward CoM)
			//  Iyz = Ioyz + mdydz  (away from CoM), Io = I - mdydz (toward CoM)
			else
			{
				float delta = mass * (offset[i] * offset[j]);
				inertia[i][j] += delta;
				inertia[j][i] += delta;
			}
		}
	}
}
