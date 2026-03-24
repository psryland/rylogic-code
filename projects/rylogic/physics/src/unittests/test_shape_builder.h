//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/shape/shape_builder.h"

namespace pr::physics
{
	PRUnitTestClass(ShapeBuilderTests)
	{
		PRUnitTestMethod(ShapeBuilderTests)
		{
			using namespace pr::collision;

			ShapeBuilder sb;
			sb.AddShape(ShapeBox(v4{0.2f, 0.4f, 0.3f, 0}, m4x4::Translation(0.0f, 0.0f, 0.3f)));
			//sb.AddShape(ShapeBox(pr::v4(0.2f, 0.4f, 0.3f, 0), pr::m4x4::Transform()));
		}
	};
}
#endif
