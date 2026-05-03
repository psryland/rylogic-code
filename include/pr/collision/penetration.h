//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/collision/forward.h"

namespace pr::collision
{
	// Result of a collision test
	struct Contact
	{
		// Notes:
		//  - The space for 'm_axis' and 'm_manifold' is whatever the common space of the collision detection is (typically world space).
		//  - To find the deepest points on 'shapeA' or 'shapeB', add/subtract half the 'm_depth' along 'm_axis' from each manifold point respectively.
		static constexpr int MaxManifoldPoints = 4;
		using Manifold = std::array<v4, MaxManifoldPoints>;

		// The collision normal (normalised) from 'shapeA' to 'shapeB' (in world space)
		v4 m_axis = {};

		// The contact manifold between 'shapeA' and 'shapeB'. Positioned at half the penetration depth, along the collision normal (typically in world space).
		// The winding order of the points is such that the face normal matches 'm_axis' (i.e. the points are ordered clockwise when looking along 'm_axis' from 'shapeA' to 'shapeB').
		Manifold m_manifold = {};

		// The contact manifold feature type. The enum value is also the manifold point count.
		EFeature m_feature = {};

		// The depth of penetration. Positive values mean overlap
		float m_depth = {};

		// The material id of the material associated with the contact point on 'shapeA'
		int m_mat_idA = {};

		// The material id of the material associated with the contact point on 'shapeB'
		int m_mat_idB = {};

		// True if this struct represents contact
		bool contact() const
		{
			return m_depth > 0;
		}

		// The number of points in the contact manifold.
		int Count() const
		{
			auto count = static_cast<int>(m_feature);
			return std::clamp(count, 0, MaxManifoldPoints);
		}

		// The points of contact manifold.
		std::span<v4 const> Points() const
		{
			return { m_manifold.data(), static_cast<size_t>(Count()) };
		}

		// The centroid of the contact manifold.
		v4 Point() const
		{
			auto points = Points();
			if (points.empty())
				return v4::Origin();

			auto centre = v4::Zero();
			for (auto const& point : points)
				centre += point;

			return (centre / static_cast<float>(points.size())).w1();
		}

		// Set a single-point manifold.
		void SetPoint(v4 point)
		{
			m_manifold = {};
			m_manifold[0] = point.w1();
			m_feature = EFeature::Vert;
		}

		// Reverse the sense of the contact information
		friend void Flip(Contact& c)
		{
			c.m_axis = -c.m_axis;
			if (auto count = c.Count(); count > 1)
				std::reverse(c.m_manifold.begin(), c.m_manifold.begin() + count);

			std::swap(c.m_mat_idA, c.m_mat_idB);
		}
	};

	// Base class for penetration function objects
	struct Penetration
	{
		// Notes:
		//  - Calculate depth as: 'just-contacting-distance - actual-distance' = positive if overlapping.
		//  - 'm_depth_sq' is initialised with "max penetration" since we typically want the minimum penetration.
		//  - We expect to test at least one separating axis which ensures 'm_depth_sq' is always set to a valid value.

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
			return m_axis_len_sq > Sqr(math::tiny<float>) ? m_axis / Sqrt(m_axis_len_sq) : v4{1,0,0,0};
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
			if (len_sq < Sqr(math::tiny<float>))
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
