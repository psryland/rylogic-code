//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12::compute::particle_collision
{
	enum class EPrimType : int
	{
		Plane = 0,
		Quad = 1,
		Triangle = 2,
		Ellipse = 3,
		Box = 4,
		Sphere = 5,
		Cylinder = 6,

		_arith_enum = 0,
	};

	struct Prim
	{
		// The object to world space transform for the primitive.
		// Note: No scale or shear is allowed in this transform.
		m4x4 m_o2w = m4x4::Identity();

		// Primitive data
		union {
			v4 _[2]; // Data size

			uint8_t plane;       // no data needed, plane is XY, normal Z
			v2 quad;        // [0].xy = width/height, plane is XY, normal is Z
			v2 triangle[3]; // [0].xy = a, [0].zw = b, [0].xy = c, plane is XY, normal is Z
			v2 ellipse;     // [0].xy = radii, plane is XY, normal is Z
			v4 box;         // [0].xyz = radii, centre is origin
			v4 sphere;      // [0].xyz = radii (actually ellipsoid), centre is origin
			v4 cylinder;    // [0].xy = radii (actually elliptic), [0].z = half-length, centre is origin, main axis is Z
		} m_data = {};

		// flags.x = primitive type
		// flags.y = unused
		// flags.z = unused
		// flags.w = unused
		EPrimType m_type = EPrimType::Plane;
		int pad[3] = {};

		Prim& pos(float x, float y, float z)
		{
			return o2w(m4x4::Translation(x, y, z));
		}
		Prim& pos(v4 pos)
		{
			return o2w(m4x4::Translation(pos));
		}
		Prim& ori(v4 dir, AxisId axis = AxisId::PosZ)
		{
			return ori(m3x4::Rotation(v4(axis), dir));
		}
		Prim& ori(m3x4 const& rot)
		{
			return o2w(rot, v4::Origin());
		}
		Prim& o2w(m3x4 const& rot, v4 pos)
		{
			return o2w(m4x4{ rot, pos });
		}
		Prim& o2w(m4x4 const& o2w)
		{
			m_o2w = o2w * m_o2w;
			return *this;
		}
	};

	struct CollisionBuilder
	{
		std::vector<Prim> m_prims;

		CollisionBuilder()
			: m_prims()
		{}
		CollisionBuilder(CollisionBuilder&&) = default;
		CollisionBuilder(CollisionBuilder const&) = delete;
		CollisionBuilder& operator=(CollisionBuilder&&) = default;
		CollisionBuilder& operator=(CollisionBuilder const&) = delete;
		Prim& Plane()
		{
			m_prims.push_back(Prim{
				.m_data = {.plane = {}},
				.m_type = EPrimType::Plane,
			});
			return m_prims.back();
		}
		Prim& Quad(v2 wh)
		{
			m_prims.push_back(Prim{
				.m_data = {.quad = {wh} },
				.m_type = EPrimType::Quad,
			});
			return m_prims.back();
		}
		Prim& Triangle(v2 a, v2 b, v2 c)
		{
			m_prims.push_back(Prim{
				.m_data = {.triangle = {a, b, c} },
				.m_type = EPrimType::Triangle,
			});
			return m_prims.back();
		}
		Prim& Ellipse(v2 radii)
		{
			m_prims.push_back(Prim{
				.m_data = {.ellipse = {radii} },
				.m_type = EPrimType::Triangle,
			});
			return m_prims.back();
		}
		Prim& Box(v4 dim)
		{
			m_prims.push_back(Prim{
				.m_data = {.box = {dim * 0.5f}},
				.m_type = EPrimType::Box,
			});
			return m_prims.back();
		}
		Prim& Sphere(v4 radii)
		{
			m_prims.push_back(Prim{
				.m_data = {.sphere = radii},
				.m_type = EPrimType::Sphere,
			});
			return m_prims.back();
		}
		Prim& Cylinder(v4 radii)
		{
			m_prims.push_back(Prim{
				.m_data = {.cylinder = radii},
				.m_type = EPrimType::Cylinder,
			});
			return m_prims.back();
		}
		std::span<Prim const> Primitives() const
		{
			return m_prims;
		}
	};
}
