//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/maths/maths.h"

namespace pr::collision
{
	// Result of a collision test
	struct Contact
	{
		// Notes:
		//  To find the deepest points on 'lhs','rhs' add/subtract half the 'depth' along 'axis'.
		//  Applied impulses should be equal and opposite, and applied at the same point in space (hence one contact point).

		// The collision normal (normalised) from 'lhs' to 'rhs'
		v4 m_axis;

		// The contact point between 'lhs' and 'rhs'. (Equal to half the penetration depth, along the collision normal)
		v4 m_point;

		// The depth of penetration. Positive values mean overlap
		float m_depth;

		// The material id of the material associated with the contact point on 'lhs'
		int m_mat_idA;

		// The material id of the material associated with the contact point on 'rhs'
		int m_mat_idB;

		int pad;

		Contact()
			:m_axis()
			,m_point()
			,m_depth()
			,m_mat_idA()
			,m_mat_idB()
			,pad()
		{}

		// Reverse the sense of the contact information
		void flip()
		{
			m_axis = -m_axis;
			std::swap(m_mat_idA, m_mat_idB);
		}
	};

	// Base class for penetration function objects
	struct Penetration
	{
		// Notes:
		//  Calculate depth as: 'just-contacting-distance - actual-distance' = positive if overlapping
		//  Initialize 'm_depth_sq' with "max penetration" since we typically want the minimum penetration.
		//  We expect to test at least one separating axis which ensures 'm_depth_sq' is always set to a valid value.

		v4    m_axis;        // The axis of minimum penetration (not normalised)
		float m_axis_len_sq; // The square of the separating axis length
		float m_depth_sq;    // The signed square of the depth of penetration
		int   m_mat_idA;     // The material id of object A
		int   m_mat_idB;     // The material id of object B

		Penetration()
			:m_axis(v4::Zero())
			,m_axis_len_sq(0.0f)
			,m_depth_sq(limits<float>::infinity())
			,m_mat_idA()
			,m_mat_idB()
		{}

		// Boolean test of penetration
		bool Contact() const
		{
			assert("No separating axes have been tested yet" && m_depth_sq != limits<float>::infinity());
			return m_depth_sq > 0;
		}

		// Return the depth of penetration
		float Depth() const
		{
			assert("No separating axes have been tested yet" && m_depth_sq != limits<float>::infinity());
			return SignedSqrt(m_depth_sq);
		}

		// The direction of minimum penetration (normalised)
		v4 SeparatingAxis() const
		{
			assert("No separating axes have been tested yet" && m_depth_sq != limits<float>::infinity());
			return m_axis_len_sq > Sqr(maths::tiny<float>) ? m_axis / Sqrt(m_axis_len_sq) : v4{1,0,0,0};
		}

		// Implemented by derived types.
		// 'depth' is positive if there is penetration.
		// 'sep_axis' is a function that returns the separating axis (for lazy evaluation)
		// The returned separating axis does not have to be normalised but 'depth' is assumed to
		// be in multiples of the separating axis length.
		// Return false to 'quick-out' of collision detection.
		template <typename SepAxis> bool operator()(float depth, SepAxis sep_axis, int mat_idA, int mat_idB) = delete;
	};

	// For boolean 'is penetrating' tests.
	struct TestPenetration :Penetration
	{
		// 'depth' should be positive if there is penetration.
		template <typename SepAxis> bool operator()(float depth, SepAxis, int, int)
		{
			m_depth_sq = Sign(depth);

			// Stop as soon as non-contact is detected
			return m_depth_sq >= 0;
		}
	};

	// Find the separating axis with the minimum penetration (i.e. the most negative depth)
	// This also records the nearest non-penetration (indicated by depth() < 0)
	struct MinPenetration :Penetration
	{
		// 'depth' is positive if there is penetration.
		// 'sep_axis' does not have to be normalised but 'depth' is assumed to be in multiples of the 'sep_axis' length.
		template <typename SepAxis> bool operator()(float depth, SepAxis sep_axis, int mat_idA, int mat_idB)
		{
			// Defer the sqrt by comparing squared depths.
			// Need to preserve the sign however.
			auto axis = sep_axis();
			auto len_sq = LengthSq(axis);
			
			// Skip degenerate axes to avoid division by zero
			if (len_sq < Sqr(maths::tiny<float>))
				return true;
			
			auto d_sq = SignedSqr(depth) / len_sq;
			if (d_sq < m_depth_sq)
			{
				m_axis = axis;
				m_axis_len_sq = len_sq;
				m_depth_sq = d_sq;
				m_mat_idA = mat_idA;
				m_mat_idB = mat_idB;
			}

			// Never quick out, test all separating axes to get the closest point
			return true;
		}
	};

	// Determines contact between objects and records the minimum penetration.
	// Quick-outs if non-contact is detected on any separating axis.
	struct ContactPenetration :MinPenetration
	{
		// 'depth' is positive if there is penetration.
		// 'sep_axis' does not have to be normalised but 'depth' is assumed to be in multiples of the 'sep_axis' length.
		template <typename SepAxis> bool operator()(float depth, SepAxis sep_axis, int mat_idA, int mat_idB)
		{
			if (depth >= 0)
				MinPenetration::operator()(depth, sep_axis, mat_idA, mat_idB);
			else
				m_depth_sq = -1.0f;

			// Stop as soon as non-contact is detected
			return m_depth_sq >= 0;
		}
	};
}