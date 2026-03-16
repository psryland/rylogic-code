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
#include "pr/physics/forward.h"
#include "pr/collision/shapes.h"

namespace pr::physics
{
	static constexpr int IntegrateThreadCount = 64;
	static constexpr int SweepThreadCount = 64;
	static constexpr int CollideThreadCount = 32;
	static constexpr int ResolveThreadCount = 64;

	// GPU-friendly representation of a collision shape.
	// All shape types are unified into a single struct with type-specific data fields.
	// For Triangle and Polytope shapes, the vertex data are stored in a separate vertex buffer
	// referenced by vert_offset and vert_count. No adjacency data is needed — the GPU uses
	// brute-force linear scan for support vertex queries.
	struct alignas(16) GpuShape
	{
		// Shape-to-rigidbody transform. Positions the shape within its rigid body.
		m4x4 s2rb;

		// Shape type. Matches EShape values: 0=Sphere, 1=Box, 2=Line, 3=Triangle, 4=Polytope
		int type;

		// Vertex buffer range reference.
		int vert_offset; // index of the first vertex in the shared vertex buffer
		int vert_count;  // number of vertices

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

	// GPU-friendly contact data for the resolve shader.
	// Contains everything needed to compute and apply the restitution impulse.
	struct alignas(16) GpuResolveContact
	{
		v4 axis;          // collision normal (in A's object space)
		v4 point;         // contact point at estimated collision time (in A's space)
		m4x4 b2a;         // B-to-A transform
		int body_idx_a;   // index into RigidBodyDynamics buffer
		int body_idx_b;   // index into RigidBodyDynamics buffer
		float elasticity; // combined material elasticity (normal)
		float friction;   // combined material static friction
		float depth;      // Penetration depth (positive = overlapping).
		int mat_id_a;     // Material IDs from each shape
		int mat_id_b;     // Material IDs from each shape
		int pad0;
	};
	static_assert(sizeof(GpuResolveContact) == 128, "GpuResolveContact must be 128 bytes for GPU alignment");

	// Counter buffer for atomic contact output.
	// The compute shader increments this atomically to allocate slots in the contact buffer.
	struct alignas(16) GpuCollisionCounters
	{
		int body_count; // The number of bodies/shapes to test
		int pair_count; // The number of potentially colliding objects
		int contact_count; // The number of contact points found
		int pad1;
	};
	static_assert(sizeof(GpuCollisionCounters) == 16);

	// GPU-friendly material properties. Packed for upload to the GPU.
	// The collide shader looks up materials by ID and merges them inline.
	struct alignas(16) GpuMaterial
	{
		float friction_static;
		float elasticity_norm;
		float elasticity_tang;
		float elasticity_tors;
		float density;
		float pad0;
		float pad1;
		float pad2;
	};
	static_assert(sizeof(GpuMaterial) == 32);

	// Output from the GPU integration step for debug validation.
	// One entry per body, written by the compute shader.
	struct alignas(16) GpuIntegrateDiag
	{
		float ke_before; // kinetic energy before integration
		float ke_after;  // kinetic energy after integration
		float pad0;
		float pad1;
	};
	static_assert(sizeof(GpuIntegrateDiag) == 16, "GpuIntegrateDiag must be 16 bytes");

	// Per-pair diagnostic output from the GPU GJK shader.
	// Written by every thread, not just colliding ones. Used for profiling.
	struct alignas(16) GpuPairDiag
	{
		int pair_index;
		int shape_type_a;
		int shape_type_b;
		int gjk_iters;
		int epa_iters;
		int hit;
		int pad0;
		int pad1;
	};
	static_assert(sizeof(GpuPairDiag) == 32);

	// ---- Pack helpers ----

	// Convert CPU collision shapes into the flat GPU format.
	inline GpuShape PackShape(collision::ShapeSphere const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		GpuShape g = {};
		g.s2rb = p2rb * shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Sphere);
		g.vert_offset = 0;
		g.vert_count = 0;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4(shape.m_radius, 0, 0, 0);
		return g;
	}
	inline GpuShape PackShape(collision::ShapeBox const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		GpuShape g = {};
		g.s2rb = p2rb * shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Box);
		g.vert_offset = 0;
		g.vert_count = 0;
		g.material_id = shape.m_base.m_material_id;
		g.data = shape.m_radius; // half-extents (xyz), w=0
		return g;
	}
	inline GpuShape PackShape(collision::ShapeLine const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		GpuShape g = {};
		g.s2rb = p2rb * shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Line);
		g.vert_offset = 0;
		g.vert_count = 0;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4(shape.m_radius, shape.m_thickness, 0, 0);
		return g;
	}
	inline GpuShape PackShape(collision::ShapeTriangle const& shape, int vert_offset, m4x4 const& p2rb = m4x4::Identity())
	{
		// Triangle vertices are packed into the shared vertex buffer.
		// The 3 vertices are stored at vert_offset..vert_offset+2.
		GpuShape g = {};
		g.s2rb = p2rb * shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Triangle);
		g.vert_offset = vert_offset;
		g.vert_count = 3;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4::Zero();
		return g;
	}
	inline GpuShape PackShape(collision::ShapePolytope const& shape, int vert_offset, m4x4 const& p2rb = m4x4::Identity())
	{
		GpuShape g = {};
		g.s2rb = p2rb * shape.m_base.m_s2p;
		g.type = static_cast<int>(collision::EShape::Polytope);
		g.vert_offset = vert_offset;
		g.vert_count = shape.m_vert_count;
		g.material_id = shape.m_base.m_material_id;
		g.data = v4::Zero();
		return g;
	}
	inline GpuShape PackShape(collision::Shape const& shape, std::vector<v4>& vertex_buffer, m4x4 const& p2rb = m4x4::Identity())
	{
		using namespace collision;

		switch (shape.m_type)
		{
			case EShape::Sphere:
			{
				return PackShape(shape_cast<ShapeSphere>(shape), p2rb);
			}
			case EShape::Box:
			{
				return PackShape(shape_cast<ShapeBox>(shape), p2rb);
			}
			case EShape::Line:
			{
				return PackShape(shape_cast<ShapeLine>(shape), p2rb);
			}
			case EShape::Triangle:
			{
				auto& tri = shape_cast<ShapeTriangle>(shape);

				// Triangle vertices are stored as w=0 offsets in m_v.x/y/z
				auto offset = static_cast<int>(vertex_buffer.size());
				vertex_buffer.push_back(tri.m_v.x);
				vertex_buffer.push_back(tri.m_v.y);
				vertex_buffer.push_back(tri.m_v.z);

				return PackShape(tri, offset, p2rb);
			}
			case EShape::Polytope:
			{
				auto& poly = shape_cast<ShapePolytope>(shape);

				// Copy polytope vertices into the shared vertex buffer
				auto offset = static_cast<int>(vertex_buffer.size());
				for (auto const* v = poly.vert_beg(); v != poly.vert_end(); ++v)
					vertex_buffer.push_back(*v);

				return PackShape(poly, offset, p2rb);
			}
			case EShape::Array:
			{
				#if 0
				auto& array = shape_cast<ShapeArray>(shape);

				auto s2rb = p2rb * shape.m_s2p;
				for (auto& subshape : array.shapes())
					PackShape(subshape, vertex_buffer, s2rb);

				return {}; // Array shapes are not directly represented on the GPU, their sub-shapes are packed individually with the array's transform applied.
				#endif
				throw std::runtime_error("not implemented"); // This needs more thought...
			}
			default:
			{
				assert(false && "Unsupported shape type for GPU collision");
				return {};
			}
		}
	}
}
