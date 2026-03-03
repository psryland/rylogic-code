//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************

#pragma once

#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector4.h"
#include "pr/maths/matrix4x4.h"
#include "pr/maths/bsphere.h"

namespace pr
{
	struct OBox
	{
		enum { Point = 1 << 0, Edge = 1 << 1, Face = 1 << 2, Bits = 1 << 3, Mask = Bits - 1 };
		m4x4 m_box_to_world;
		v4   m_radius;

		OBox() = default;
		OBox(v4 const& centre, v4 const& radii, m3x4 const& ori)
			:m_box_to_world(ori, centre)
			,m_radius(radii)
		{}
		OBox(m4x4 const& box_to_world, v4 const& radii)
			:m_box_to_world(box_to_world)
			,m_radius(radii)
		{}

		// Width of the box
		float SizeX() const
		{
			return 2.0f * m_radius.x;
		}

		// Height of the box
		float SizeY() const
		{
			return 2.0f * m_radius.y;
		}

		// Length of the box
		float SizeZ() const
		{
			return 2.0f * m_radius.z;
		}

		// The centre position of the box
		v4 const& Centre() const
		{
			return m_box_to_world.pos;
		}

		// Diagonal size of the box
		float DiametreSq() const
		{
			return 4.0f * LengthSq(m_radius);
		}
		float Diametre() const
		{
			return Sqrt(DiametreSq());
		}
	};
	static_assert(std::is_trivially_copyable_v<OBox>, "Should be a pod type");
	static_assert(std::alignment_of_v<OBox> == 16, "Should be 16 byte aligned");

	#pragma region Constants
	static OBox const OBoxZero  = {m4x4Identity, v4Zero};
	static OBox const OBoxUnit  = {m4x4Identity, {0.5f, 0.5f, 0.5f, 1.0f}};
	static OBox const OBoxReset = {m4x4Identity, v4Zero};
	#pragma endregion

	#pragma region Operators
	inline bool operator == (OBox const& lhs, OBox const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
	inline bool operator != (OBox const& lhs, OBox const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
	inline bool operator <  (OBox const& lhs, OBox const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
	inline bool operator >  (OBox const& lhs, OBox const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
	inline bool operator <= (OBox const& lhs, OBox const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
	inline bool operator >= (OBox const& lhs, OBox const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }
	inline OBox& pr_vectorcall operator += (OBox& lhs, v4 offset)
	{
		lhs.m_box_to_world.pos += offset;
		return lhs;
	}
	inline OBox& pr_vectorcall operator -= (OBox& lhs, v4 offset)
	{
		lhs.m_box_to_world.pos -= offset;
		return lhs;
	}
	inline OBox pr_vectorcall operator + (OBox const& lhs, v4 offset)
	{
		auto ob = lhs;
		return ob += offset;
	}
	inline OBox pr_vectorcall operator - (OBox const& lhs, v4 offset)
	{
		auto ob = lhs;
		return ob -= offset;
	}
	inline OBox pr_vectorcall operator * (m4x4 const& m, OBox const& ob)
	{
		OBox obox;
		obox.m_box_to_world = m * ob.m_box_to_world;
		obox.m_radius       = ob.m_radius;
		return obox;
	}
	#pragma endregion

	#pragma region Functions

	// Return the volume of the box
	inline float Volume(OBox const& ob)
	{
		return ob.SizeX() * ob.SizeY() * ob.SizeZ();
	}

	// Return the bounding sphere for the box
	inline BSphere GetBSphere(OBox const& ob)
	{
		return BSphere(ob.m_box_to_world.pos, Length(ob.m_radius));
	}

	#pragma endregion
}
