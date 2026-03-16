//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// POD structs mirroring the HLSL layouts for GPU narrow phase collision detection.
// Used as transfer formats between CPU broadphase output and the GPU GJK compute shader.
//
// Pipeline:
//   CPU broadphase → pack shapes/pairs → GPU GJK → readback contacts → CPU impulse resolution
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/collision/shapes.h"

namespace pr::physics
{
	// GPU-friendly representation of a collision shape.
	// All shape types are unified into a single struct with type-specific data fields.
	// Layout matches the HLSL GpuShape struct exactly.
	//
	// For Polytope shapes, the vertex data is stored in a separate vertex buffer
	// referenced by vert_offset and vert_count. No adjacency data is needed —
	// the GPU uses brute-force linear scan for support vertex queries.
	struct alignas(16) GpuShape
	{
		// Shape-to-parent transform. Positions the shape within its parent rigid body.
		// Most shapes use identity here. In C++ this is column-major (x/y/z = basis,
		// w = position). In HLSL with 'row_major float4x4', each row = one C++ column.
		m4x4 s2p;

		// Shape type discriminator. Matches EShape values:
		//   0=Sphere, 1=Box, 2=Line, 3=Triangle, 4=Polytope
		int type;

		// Polytope vertex buffer reference. For non-polytope shapes, these are unused.
		int vert_offset;   // index of first vertex in the shared vertex buffer
		int vert_count;    // number of vertices

		// Material ID for collision response
		int material_id;

		// Type-specific shape data packed into a single float4:
		//   Sphere:   (radius, 0, 0, 0)
		//   Box:      (half_x, half_y, half_z, 0) — half-extents
		//   Line:     (half_length, thickness, 0, 0) — half-length along Z, collision radius
		//   Triangle: unused (vertices stored in vert buffer)
		//   Polytope: unused (vertices stored in vert buffer)
		v4 data;
	};
	static_assert(sizeof(GpuShape) == 96, "GpuShape must be 96 bytes (6 x float4)");

	// A collision pair to test on the GPU.
	// The broadphase identifies overlapping AABB pairs on the CPU, then packs
	// the pair info into this struct for the GPU narrow phase.
	struct alignas(16) GpuCollisionPair
	{
		// Indices into the GpuShape buffer identifying the two shapes to test.
		int shape_idx_a;
		int shape_idx_b;

		// Index of this pair in the original broadphase pair list (for CPU readback mapping)
		int pair_index;
		int pad0;

		// Transform from shape B's space into shape A's space.
		// The GJK algorithm runs with shape A at identity and shape B at b2a.
		// In C++ column-major; HLSL 'row_major float4x4' rows = C++ columns.
		m4x4 b2a;
	};
	static_assert(sizeof(GpuCollisionPair) == 80, "GpuCollisionPair must be 80 bytes (5 x float4)");

	// GPU collision detection output for a single pair.
	// Written by the GJK compute shader when a collision is detected.
	struct alignas(16) GpuContact
	{
		// Collision separating axis (unit normal pointing from A toward B).
		// In objA's space (since GJK runs with A at identity).
		v4 axis;

		// Contact point (in objA's space), midpoint between the two surfaces.
		v4 pt;

		// Penetration depth (positive = overlapping).
		float depth;

		// Index of the pair that produced this contact (maps back to GpuCollisionPair.pair_index).
		int pair_index;

		// Material IDs from each shape, for combined material lookup on CPU.
		int mat_id_a;
		int mat_id_b;
	};
	static_assert(sizeof(GpuContact) == 48, "GpuContact must be 48 bytes (3 x float4)");

	// Counter buffer for atomic contact output.
	// The compute shader increments this atomically to allocate slots in the contact buffer.
	struct alignas(16) GpuCollisionCounters
	{
		uint32_t contact_count;
		uint32_t pad[3];
	};
	static_assert(sizeof(GpuCollisionCounters) == 16);

	// ---- Pack helpers ----

	// Convert CPU collision shapes into the flat GPU format.
	inline GpuShape PackShape(collision::ShapeSphere const& shape)
	{
		GpuShape g = {};
		g.s2p = shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Sphere);
		g.vert_offset = 0;
		g.vert_count = 0;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4(shape.m_radius, 0, 0, 0);
		return g;
	}
	inline GpuShape PackShape(collision::ShapeBox const& shape)
	{
		GpuShape g = {};
		g.s2p = shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Box);
		g.vert_offset = 0;
		g.vert_count = 0;
		g.material_id = shape.m_base.m_material_id;
		g.data = shape.m_radius; // half-extents (xyz), w=0
		return g;
	}
	inline GpuShape PackShape(collision::ShapeLine const& shape)
	{
		GpuShape g = {};
		g.s2p = shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Line);
		g.vert_offset = 0;
		g.vert_count = 0;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4(shape.m_radius, shape.m_thickness, 0, 0);
		return g;
	}
	inline GpuShape PackShape(collision::ShapeTriangle const& shape, int vert_offset)
	{
		// Triangle vertices are packed into the shared vertex buffer.
		// The 3 vertices are stored at vert_offset..vert_offset+2.
		GpuShape g = {};
		g.s2p = shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Triangle);
		g.vert_offset = vert_offset;
		g.vert_count = 3;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4::Zero();
		return g;
	}
	inline GpuShape PackShape(collision::ShapePolytope const& shape, int vert_offset)
	{
		GpuShape g = {};
		g.s2p = shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Polytope);
		g.vert_offset = vert_offset;
		g.vert_count = shape.m_vert_count;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4::Zero();
		return g;
	}
	inline GpuShape PackShapeGeneric(collision::Shape const& shape, std::vector<v4>& vertex_buffer)
	{
		using namespace collision;

		switch (shape.m_type)
		{
			case EShape::Sphere:
			{
				return PackShape(shape_cast<ShapeSphere>(shape));
			}
			case EShape::Box:
			{
				return PackShape(shape_cast<ShapeBox>(shape));
			}
			case EShape::Line:
			{
				return PackShape(shape_cast<ShapeLine>(shape));
			}
			case EShape::Triangle:
			{
				auto& tri = shape_cast<ShapeTriangle>(shape);
				auto offset = static_cast<int>(vertex_buffer.size());

				// Triangle vertices are stored as w=0 offsets in m_v.x/y/z
				vertex_buffer.push_back(tri.m_v.x);
				vertex_buffer.push_back(tri.m_v.y);
				vertex_buffer.push_back(tri.m_v.z);
				return PackShape(tri, offset);
			}
			case EShape::Polytope:
			{
				auto& poly = shape_cast<ShapePolytope>(shape);
				auto offset = static_cast<int>(vertex_buffer.size());

				// Copy polytope vertices into the shared vertex buffer
				for (auto const* v = poly.vert_beg(); v != poly.vert_end(); ++v)
					vertex_buffer.push_back(*v);

				return PackShape(poly, offset);
			}
			default:
			{
				assert(false && "Unsupported shape type for GPU collision");
				return {};
			}
		}
	}
}
