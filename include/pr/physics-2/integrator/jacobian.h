//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/rigid_body/rigid_body.h"

namespace pr::physics
{
	// The Jacobian matrix is a sparse matrix of velocity constraints between two objects.
	// The dimensions of the matrix would be (rows = S, columns = 6N) where 'S' is the number of
	// constraints and 'N' is the number of interacting bodies. Each row is a constraint between
	// two objects, so the only non-zero values on the row are two blocks of 6 values corresponding
	// to the 6 degrees of freedom for each object,
	// e.g. row = '{ ... xi yi zi ai bi ci ... xj yj zj aj bj cj ... }' = a single velocity constraint
	// between objects i and j.
	// For efficiency, I'm using v8 spatial vectors rather than v6.
	struct Jacobian
	{
		struct Idx
		{
			int m_i0;
			int m_i1;
		};
		struct Row
		{
			v8 m_obj0;
			v8 m_obj1;
		};

		pr::vector<Row> m_rows;
		pr::vector<Idx> m_idx;

		Jacobian()
			:m_rows()
			,m_idx()
		{}
	};
}