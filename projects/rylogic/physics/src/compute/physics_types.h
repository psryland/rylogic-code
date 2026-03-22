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
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/rigid_body/state_flags.h"

namespace pr::physics
{
	static constexpr int IntegrateThreadCount = 64;
	static constexpr int SweepThreadCount = 64;
	static constexpr int CollideThreadCount = 32;
	static constexpr int ResolveThreadCount = 64;
	static constexpr int MaxColours = 32;

	// A flat, GPU-friendly representation of the rigid body state needed for integration.
	// Layout matches the HLSL GpuRigidBody struct exactly (13 × float4 = 208 bytes).
	//
	// Memory convention: C++ m4x4 stores columns contiguously (x, y, z, w members).
	// HLSL 'row_major float4x4' stores rows contiguously, so each HLSL row maps to one
	// C++ column vector. Use mul(v, o2w) in HLSL for transforms (row-vector convention).
	struct alignas(16) GpuRigidBody
	{
		// Object-to-world transform. In C++ this is column-major (x/y/z = basis vectors,
		// w = position). In HLSL with 'row_major float4x4', each row = one C++ column.
		m4x4 o2w;

		// World-space spatial momentum [angular, linear], measured at the centre of mass.
		v4 momentum_ang;   // angular momentum (torque accumulation)
		v4 momentum_lin;   // linear momentum

		// World-space external force accumulator [angular, linear], measured at the centre of mass.
		// Zeroed after each integration step so persistent forces must be re-applied each frame.
		v4 force_ang;      // external torque
		v4 force_lin;      // external force

		// World-space gravity. Each body has its own gravity vector to define local "down" for this object.
		// Gravity is applied every frame even for static bodies.
		v4 gravity;

		// Object-space inverse inertia in compact symmetric form.
		// The full 3×3 inverse inertia matrix is reconstructed from diagonal + off-diagonal terms.
		v4 inertia_inv_diagonal;  // {Ixx_inv, Iyy_inv, Izz_inv, 0}
		v4 inertia_inv_products;  // {Ixy_inv, Ixz_inv, Iyz_inv, 0}

		// Object-space CoM offset from model origin, packed with inverse mass.
		// The CoM offset is needed to convert CoM velocity to model-origin position changes.
		v4 os_com_and_invmass;    // {com_x, com_y, com_z, inv_mass}

		// Object-space bounding box for AABB computation in the integrate shader.
		// These allow the GPU to compute world-space AABBs after evolving the transform.
		BBox os_bbox;   // object-space AABB

		// State flags:
		//  1 << 0: Sleep state: 0 = awake, 1 = sleeping.
		int state_flags;

		// The id of the shape for this object
		int shape_id;

		// Scratch: bitmask of graph-colouring colours used by this body.
		// Written by CSComputeCollisionTimes, read by CSAssignColours.
		uint32_t colour_used;

		// The number of valid points in the contact simplex.
		int contact_simplex_count;

		// Contact support simplex (world space, relative to body origin).
		// Up to 4 recent contact points. Only contacts with normals that oppose gravity are recorded.
		v4 contact_simplex[4];
	};
	static_assert((sizeof(GpuRigidBody) & 0xf) == 0, "GpuRigidBody must be a multiple of 16 bytes");
	static_assert(alignof(GpuRigidBody) == 16, "GpuRigidBody must be 16-byte aligned");

	// Pack a RigidBody's dynamic state into the flat GPU buffer format.
	inline GpuRigidBody PackDynamics(RigidBody const& rb, int shape_id)
	{
		auto com = rb.CentreOfMassOS();
		auto result = GpuRigidBody
		{
			.o2w = rb.m_o2w,
			.momentum_ang = rb.m_ws_momentum.ang,
			.momentum_lin = rb.m_ws_momentum.lin,
			.force_ang = rb.m_ws_force.ang,
			.force_lin = rb.m_ws_force.lin,
			.gravity = rb.m_ws_gravity,
			.inertia_inv_diagonal = rb.m_os_inertia_inv.m_diagonal,
			.inertia_inv_products = rb.m_os_inertia_inv.m_products,
			.os_com_and_invmass = v4{com.x, com.y, com.z, rb.m_os_inertia_inv.InvMass()},
			.os_bbox = rb.Shape().m_bbox,
			.state_flags = static_cast<int>(rb.m_state_flags),
			.shape_id = shape_id,
			.colour_used = 0,
			.contact_simplex_count = rb.m_contact_simplex_count,
			.contact_simplex = {
				rb.m_contact_simplex[0],
				rb.m_contact_simplex[1],
				rb.m_contact_simplex[2],
				rb.m_contact_simplex[3]
			},
		};
		return result;
	}
	inline void UnpackDynamics(GpuRigidBody const& dyn, RigidBody& rb)
	{
		rb.O2W(dyn.o2w);

		// Update momentum (the integrator advanced it by the full step)
		rb.MomentumWS(v8force{ dyn.momentum_ang, dyn.momentum_lin });

		// Forces are zeroed by the integrator after the second half-kick
		rb.ZeroForces();

		// Preserve the state flags
		rb.m_state_flags = static_cast<ERigidBodyStateFlags>(dyn.state_flags);

		// Preserve the contact simplex
		rb.m_contact_simplex[0] = dyn.contact_simplex[0];
		rb.m_contact_simplex[1] = dyn.contact_simplex[1];
		rb.m_contact_simplex[2] = dyn.contact_simplex[2];
		rb.m_contact_simplex[3] = dyn.contact_simplex[3];
		rb.m_contact_simplex_count = dyn.contact_simplex_count;
	}

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
		int body_idx_a;  // Rigid body index for A
		int body_idx_b;  // Rigid body index for B
		int shape_idx_a; // Collision shape index for A
		int shape_idx_b; // Collision shape index for B

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
		v4 axis;              // collision normal (in A's object space) (points from A to B)
		v4 contact_point;     // contact point at estimated collision time (in A's space)
		m4x4 b2a;             // B-to-A transform
		int body_idx_a;       // index into GpuRigidBody buffer
		int body_idx_b;       // index into GpuRigidBody buffer
		int mat_id_a;         // Material IDs from each shape
		int mat_id_b;         // Material IDs from each shape
		float depth;          // Penetration depth (positive = overlapping).
		float collision_time; // Estimated sub-step collision time. Written by CSComputeCollisionTimes, read by CSAssignColours.
		int pad0;
		int pad1;
	};
	static_assert(sizeof(GpuResolveContact) == 128, "GpuResolveContact must be 128 bytes for GPU alignment");

	// Counter buffer for atomic contact output.
	// The compute shader increments this atomically to allocate slots in the contact buffer.
	struct alignas(16) GpuCollisionCounters
	{
		int pair_count; // The number of potentially colliding objects
		int contact_count; // The number of contact points found
		int pad0;
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
		int body_idx_a;  // Rigid body index for A
		int body_idx_b;  // Rigid body index for B
		int shape_type_a;
		int shape_type_b;
		int gjk_iters;
		int epa_iters;
		int hit;
		int pad0;
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
