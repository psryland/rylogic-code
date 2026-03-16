//************************************
// Physics-2 Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics-2/utility/field.h"
namespace pr::physics::tests
{
	PRUnitTestClass(FieldTests)
	{
		PRUnitTestMethod(FieldTests)
		{
			// 2D Vector field, 10cm grid
			Field<2, v2, 0.01f, field::KernelSpike2D> field(0.05f);

			/*
			// Draw the vector field
			pr::ldraw::Builder builder;
			auto& points = builder.Point("Field", 0xFF00FF00).size(0.1f);
			for (int y = 0; y != 100; ++y)
			{
				for (int x = 0; x != 100; ++x)
				{
					auto p = v2(x, y) * 0.01f;
					auto v = field.ValueAt(p);
					points.pt(p, p + v);
				}
			}
			builder.Write("E:/Dump/Field.ldr");
			//*/

			/*
			// Insert some vectors into the field
			std::default_random_engine rng;
			for (int i = 0; i != 1000; ++i)
			{
				auto v = v2::Random(rng, v2(-1000), v2(+1000));

				// Insert an "explosion" at 'v'
				field.ValueAt(v, [&](v2 p)
				{
					auto d = Length(p - v);
					return v2::Normalize(p - v) * (1.0f - d);
				});
			}

			auto v0 = field.get(v2(5, 5));
			PR_EXPECT(v0 == v2::Zero());
			//*/
		}
	};
}
#endif
