//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shapes.h"
#include "pr/collision/col_sphere_vs_sphere.h"
#include "pr/collision/col_box_vs_sphere.h"
#include "pr/collision/col_box_vs_box.h"
#include "pr/collision/col_line_vs_box.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	using Detect = bool (pr_vectorcall *)(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& c);

	// Function type for collection detection
	inline bool pr_vectorcall CollisionNotImplemented(Shape const&, m4x4 const&, Shape const&, m4x4 const&, Contact&)
	{
		throw std::runtime_error("Collision not implemented");
	}

	// Collide two shapes
	inline bool pr_vectorcall Collide(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		using namespace pr::tri_table;

		// Check the order and index values haven't been changed
		static_assert(int(EShape::Sphere  ) == 0);
		static_assert(int(EShape::Box     ) == 1);
		static_assert(int(EShape::Line    ) == 2);
		static_assert(int(EShape::Triangle) == 3);
		static_assert(int(EShape::Polytope) == 4);
		static_assert(int(EShape::Array   ) == 5);
		static_assert(int(EShape::NumberOf) == 6);

		// Tri-Table of collision functions
		static Detect s_collision_functions[Size(EType::Inclusive, int(EShape::NumberOf))] = 
		{
			SphereVsSphere,          // (0 v 0) - Sphere v Sphere  

			BoxVsSphere,             // (1 v 0) - Box v Sphere
			BoxVsBox,                // (1 v 1) - Box v Box 

			CollisionNotImplemented, // (2 v 0) - Line v Sphere
			LineVsBox,               // (2 v 1) - Line v Box
			CollisionNotImplemented, // (2 v 2) - Line v Line

			CollisionNotImplemented, // (3 v 0) - Triangle v Sphere
			CollisionNotImplemented, // (3 v 1) - Triangle v Box
			CollisionNotImplemented, // (3 v 2) - Triangle v Line
			CollisionNotImplemented, // (3 v 3) - Triangle v Triangle

			CollisionNotImplemented, // (4 v 0) - Polytope v Sphere
			CollisionNotImplemented, // (4 v 1) - Polytope v Box
			CollisionNotImplemented, // (4 v 2) - Polytope v Line
			CollisionNotImplemented, // (4 v 3) - Polytope v Triangle
			CollisionNotImplemented, // (4 v 4) - Polytope v Polytope

			CollisionNotImplemented, // (5 v 0) - Array v Sphere
			CollisionNotImplemented, // (5 v 1) - Array v Box
			CollisionNotImplemented, // (5 v 2) - Array v Line
			CollisionNotImplemented, // (5 v 3) - Array v Triangle
			CollisionNotImplemented, // (5 v 4) - Array v Polytope
			CollisionNotImplemented, // (5 v 5) - Array v Array
		};

		// Get the appropriate collision function
		auto func = s_collision_functions[Index(EType::Inclusive, int(lhs.m_type), int(rhs.m_type))];

		// Ensure the lowest shape type value is 'lhs'
		auto flip = lhs.m_type > rhs.m_type;

		// Test for contact
		auto result = flip
			? func(rhs, r2w, lhs, l2w, contact)
			: func(lhs, l2w, rhs, r2w, contact);
		
		// Flip back
		if (flip)
			contact.flip();

		return result;
	}
}

