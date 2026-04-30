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
#include "src/compute/physics_types.hlsli"

namespace pr::physics
{
	// Convert CPU collision shapes into the flat GPU format.
	inline GpuShape PackShape(collision::ShapeSphere const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		return GpuShape {
			.s2rb = p2rb * shape.m_base.m_s2r,
			.type = static_cast<int>(collision::EShape::Sphere),
			.vert_offset = 0,
			.vert_count = 0,
			.material_id = shape.m_base.m_material_id,
			.data = v4(shape.m_radius, 0, 0, 0),
		};		
	}
	inline GpuShape PackShape(collision::ShapeBox const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		return GpuShape {
			.s2rb = p2rb * shape.m_base.m_s2r,
			.type = static_cast<int>(collision::EShape::Box),
			.vert_offset = 0,
			.vert_count = 0,
			.material_id = shape.m_base.m_material_id,
			.data = shape.m_radius, // half-extents (xyz), w=0
		};
	}
	inline GpuShape PackShape(collision::ShapeLine const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		return GpuShape {
			.s2rb = p2rb * shape.m_base.m_s2r,
			.type = static_cast<int>(collision::EShape::Line),
			.vert_offset = 0,
			.vert_count = 0,
			.material_id = shape.m_base.m_material_id,
			.data = v4(shape.m_hlength, shape.m_radius, 0, 0),
		};
	}
	inline GpuShape PackShape(collision::ShapeTriangle const& shape, int vert_offset, m4x4 const& p2rb = m4x4::Identity())
	{
		return GpuShape {
			.s2rb = p2rb * shape.m_base.m_s2r,
			.type = static_cast<int>(collision::EShape::Triangle),
			.vert_offset = vert_offset,
			.vert_count = 3, // The 3 vertices are stored at vert_offset..vert_offset+2.
			.material_id = shape.m_base.m_material_id,
			.data = v4::Zero(),
		};
	}
	inline GpuShape PackShape(collision::ShapePolytope const& shape, int vert_offset, m4x4 const& p2rb = m4x4::Identity())
	{
		return GpuShape {
			.s2rb = p2rb * shape.m_base.m_s2r,
			.type = static_cast<int>(collision::EShape::Polytope),
			.vert_offset = vert_offset,
			.vert_count = shape.m_vert_count,
			.material_id = shape.m_base.m_material_id,
			.data = v4::Zero(),
		};
	}
	inline GpuShape PackShape(collision::Shape const& shape, std::vector<v4>& vertex_buffer, m4x4 const& p2rb = m4x4::Identity())
	{
		using namespace collision;

		switch (shape.m_type)
		{
			case EShape::NoShape:
			{
				return GpuShape {
					.s2rb = p2rb * shape.m_s2r,
					.type = static_cast<int>(collision::EShape::NoShape),
					.vert_offset = 0,
					.vert_count = 0,
					.material_id = 0,
					.data = v4::Zero(),
				};
			}
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

				for (auto& subshape : array.shapes())
					PackShape(subshape, vertex_buffer, p2rb);

				return {}; // Array shapes are not directly represented on the GPU; their sub-shapes already carry root-space transforms.
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

	// Pack a RigidBody's dynamic state into the flat GPU buffer format.
	inline GpuRigidBody PackDynamics(RigidBody const& rb, int shape_id)
	{
		return GpuRigidBody {
			.o2w = rb.m_o2w,
			.momentum_ang = rb.m_ws_momentum.ang,
			.momentum_lin = rb.m_ws_momentum.lin,
			.force_ang = rb.m_ws_force.ang,
			.force_lin = rb.m_ws_force.lin,
			.ws_gravity = rb.m_ws_gravity,
			.inertia_inv_diagonal = rb.m_os_inertia_inv.m_diagonal,
			.inertia_inv_products = rb.m_os_inertia_inv.m_products,
			.os_com_and_invmass = v4{rb.CentreOfMassOS().xyz, rb.m_os_inertia_inv.InvMass()},
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
}
namespace pr
{
	template <> struct Convert<collision::Contact, physics::GpuContact>
	{
		static collision::Contact Func(physics::GpuContact const& contact)
		{
			collision::Contact result = {};
			result.m_axis = contact.axis;
			result.m_feature = static_cast<collision::EFeature>(contact.feature);
			result.m_depth = contact.depth;
			for (int i = 0, iend = result.Count(); i != iend; ++i)
				result.m_manifold[i] = contact.manifold[i];
			return result;
		}
	};
}