//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#if PR_UNITTESTS
#include "pr/collision/ldraw.h"

namespace pr::collision::tests
{
	// Draw the collision scene for a unit test.
	inline void VisualiseCollision(std::filesystem::path filepath, collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
	{
		#if PR_UNITTESTS_VISUALISE
		{
			ldraw::Builder builder;
			builder.Add<ldraw::LdrCollisionShape>("ObjA", 0x80FF8000).shape(a).o2w(a2w);
			builder.Add<ldraw::LdrCollisionShape>("ObjB", 0x800080FF).shape(b).o2w(b2w);
			if (c.contact())
				builder.Add<ldraw::LdrCollisionContact>("Contact").contact(c);

			builder.Save(filepath);
		}
		#endif
		(void)filepath, a, a2w, b, b2w, c;
	}

	// Validate the contact information is as expected, within tolerance.
	inline bool CheckContact(Contact const& c, Contact const& expected, float tol = 1e-4f)
	{
		PR_EXPECT(FEqlAbsolute(c.m_depth, expected.m_depth, tol));
		PR_EXPECT(FEqlRelative(c.m_axis, expected.m_axis, tol));
		PR_EXPECT(FEqlRelative(c.Point(), expected.Point(), tol));

		PR_EXPECT(c.Count() == expected.Count());
		PR_EXPECT(c.m_feature == expected.m_feature);
		for (int i = 0, iend = expected.Count(); i != iend; ++i)
		{
			PR_EXPECT(FEqlRelative(c.m_manifold[i], expected.m_manifold[i], tol));
		}
		return true;
	}
	inline bool CheckContact(Contact const& c, Contact const* expected, float tol = 1e-4f)
	{
		PR_EXPECT(c.contact() == (expected != nullptr));
		if (expected == nullptr)
			return true;

		return CheckContact(c, *expected, tol);
	}
}
#endif
