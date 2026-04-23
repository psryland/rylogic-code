//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/shape/shape_builder.h"

namespace pr::physics::tests
{
	using namespace pr::collision;
	void ForceLink_ShapeBuilder() {}

	PRUnitTestClass(ShapeBuilderTests)
	{
		// Test that single-primitive shapes build correctly and produce valid mass properties.
		PRUnitTestMethod(SingleBox)
		{
			ShapeBuilder sb;
			sb.AddShape(ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* shape = sb.BuildShape(data, mp, model_to_com);

			// A centred box should have CoM at the origin
			PR_EXPECT(FEql(mp.m_centre_of_mass, v4{}));
			PR_EXPECT(mp.m_mass > 0.0f);
			PR_EXPECT(shape != nullptr);
			PR_EXPECT(shape->m_type == EShape::Box);
		}

		PRUnitTestMethod(SingleSphere)
		{
			ShapeBuilder sb;
			sb.AddShape(ShapeSphere(1.0f));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* shape = sb.BuildShape(data, mp, model_to_com);

			PR_EXPECT(FEql(mp.m_centre_of_mass, v4{}));
			PR_EXPECT(mp.m_mass > 0.0f);
			PR_EXPECT(shape != nullptr);
			PR_EXPECT(shape->m_type == EShape::Sphere);
		}

		PRUnitTestMethod(OffsetBox)
		{
			// A box offset from the origin should produce a non-zero model_to_com shift
			ShapeBuilder sb;
			auto offset = v4{1.0f, 0.0f, 0.0f, 0.0f};
			sb.AddShape(ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(offset.w1())));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* shape = sb.BuildShape(data, mp, model_to_com);

			// BuildShape shifts geometry to the CoM frame. The original CoM offset
			// is returned via model_to_com.
			PR_EXPECT(FEqlRelative(model_to_com, offset, 0.001f));
			PR_EXPECT(mp.m_mass > 0.0f);
			PR_EXPECT(shape != nullptr);
		}

		// Test that compound shapes calculate the correct combined CoM and have valid inertia.
		PRUnitTestMethod(CompoundTwoBoxes)
		{
			ShapeBuilder sb;

			// Two equal boxes symmetrically placed about the origin
			sb.AddShape(ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(+1.0f, 0.0f, 0.0f)));
			sb.AddShape(ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(-1.0f, 0.0f, 0.0f)));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* shape = sb.BuildShape(data, mp, model_to_com);

			// By symmetry, the CoM should be at the origin
			PR_EXPECT(FEqlRelative(mp.m_centre_of_mass, v4{}, 0.001f));
			PR_EXPECT(mp.m_mass > 0.0f);
			PR_EXPECT(shape != nullptr);

			// The resulting shape should be an array container
			PR_EXPECT(shape->m_type == EShape::Array);

			// The inertia should be non-zero and valid
			auto inertia = Inertia{mp.m_os_unit_inertia, mp.m_mass};
			PR_EXPECT(inertia.Check());
		}

		PRUnitTestMethod(CompoundAsymmetric)
		{
			ShapeBuilder sb;

			// Two boxes of different sizes, offset so CoM is NOT at origin
			sb.AddShape(ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(+2.0f, 0.0f, 0.0f)));
			sb.AddShape(ShapeBox(v4{0.2f, 0.2f, 0.2f, 0}, m4x4::Translation(-1.0f, 0.0f, 0.0f)));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* shape = sb.BuildShape(data, mp, model_to_com);

			// BuildShape shifts geometry to CoM frame. The model_to_com shift
			// should be positive X (biased toward the larger box).
			PR_EXPECT(model_to_com.x > 0.0f);
			PR_EXPECT(mp.m_mass > 0.0f);
			PR_EXPECT(shape != nullptr);

			// The resulting shape should be an array container
			PR_EXPECT(shape->m_type == EShape::Array);

			// Inertia should be valid
			auto inertia = Inertia{mp.m_os_unit_inertia, mp.m_mass};
			PR_EXPECT(inertia.Check());
		}
	};
}
#endif
