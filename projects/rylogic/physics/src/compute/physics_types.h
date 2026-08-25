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
	static_assert((sizeof(GpuSleepData) & 0xf) == 0);
	static_assert((sizeof(GpuSleepIsland) & 0xf) == 0);
	static_assert((sizeof(GpuSleepIslandStats) & 0xf) == 0);
	static_assert((sizeof(GpuRigidBody) & 0xf) == 0);
	static_assert((sizeof(GpuShape) & 0xf) == 0);
	static_assert((sizeof(GpuPolytopeFace) & 0xf) == 0);
	static_assert((sizeof(GpuPolytopeEdge) & 0xf) == 0);
	static_assert((sizeof(GpuCollisionPair) & 0xf) == 0);
	static_assert((sizeof(GpuContact) & 0xf) == 0);
	static_assert((sizeof(GpuResolveContact) & 0xf) == 0);
	static_assert((sizeof(GpuWarmStartEntry) & 0xf) == 0);
	static_assert(sizeof(GpuConstraintFrame) == 32);
	static_assert(sizeof(GpuConstraintAxisDesc) == 32);
	static_assert(sizeof(GpuD6ConstraintDesc) == 256);
	static_assert(sizeof(GpuConstraintEndpoint) == 32);
	static_assert(sizeof(GpuCollisionExclusion) == 8);
	static_assert(sizeof(GpuConstraintBlock) == 32);
	static_assert(sizeof(GpuConstraintRow) == 96);
	static_assert(sizeof(GpuConstraintPseudoVelocity) == 32);
	static_assert((sizeof(GpuCollisionCounters) & 0xf) == 0);
	static_assert(sizeof(GpuFrameForce) == 32);
	static_assert(sizeof(GpuArticulation) == 80);
	static_assert(sizeof(GpuArticulationLink) == 160);
	static_assert(sizeof(GpuArticulationDof) == 16);
	static_assert(sizeof(GpuArticulationLevel) == 16);
	static_assert(sizeof(GpuArticulationSpatialVector) == 32);
	static_assert(alignof(GpuArticulationSpatialVector) == 16);
	static_assert(sizeof(GpuArticulationSpatialMatrix) == 192);
	static_assert(alignof(GpuArticulationSpatialMatrix) == 16);
	static_assert(sizeof(GpuArticulationJointMatrix) == 192);
	static_assert(alignof(GpuArticulationJointMatrix) == 16);
	static_assert(sizeof(GpuArticulationAbaDofScratch) == 64);
	static_assert(alignof(GpuArticulationAbaDofScratch) == 16);
	static_assert(sizeof(GpuArticulationAbaScratch) == 336);
	static_assert(alignof(GpuArticulationAbaScratch) == 16);
	static_assert(sizeof(GpuArticulationIntegrationState) == 48);
	static_assert(alignof(GpuArticulationIntegrationState) == 16);
	static_assert(sizeof(GpuFrameOutputHeader) == 64);
	static_assert(sizeof(GpuSubstepOutputState) == 16);
	static_assert(sizeof(GpuCollisionEvent) == sizeof(GpuResolveContact) + 16);
	static_assert((sizeof(GpuSelectiveRefreshMetrics) & 0xf) == 0);
	static_assert((sizeof(GpuMaterial) & 0xf) == 0);

	// Convert a rigid transform to the quaternion-and-position representation shared by GPU descriptors.
	inline GpuConstraintFrame PackGpuTransform(m4x4 const& transform)
	{
		auto const rotation = ToQuat<quat>(transform.rot);
		return GpuConstraintFrame{
			.rotation = float4{rotation.x, rotation.y, rotation.z, rotation.w},
			.position = transform.pos,
		};
	}

	// Reconstruct a rigid transform from the representation shared by GPU descriptors.
	inline m4x4 UnpackGpuTransform(GpuConstraintFrame const& transform)
	{
		auto const rotation = quat{
			transform.rotation.x,
			transform.rotation.y,
			transform.rotation.z,
			transform.rotation.w,
		};
		auto position = v4{
			transform.position.x,
			transform.position.y,
			transform.position.z,
			1.0f,
		};
		return m4x4{ToMatrix<m3x3>(rotation), position};
	}

	// Convert CPU collision shapes into the flat GPU format.
	inline GpuShape PackShape(collision::ShapeSphere const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		auto s2rb = p2rb * shape.m_base.m_s2r;
		return GpuShape {
			.s2rb = s2rb,
			.rb_bbox = s2rb * shape.m_base.m_bbox,
			.type = static_cast<int>(collision::EShape::Sphere),
			.vert_offset = 0,
			.vert_count = 0,
			.material_id = shape.m_base.m_material_id,
			.face_offset = 0,
			.face_count = 0,
			.edge_offset = 0,
			.edge_count = 0,
			.child_offset = 0,
			.child_count = 0,
			.flags = static_cast<int>(shape.m_base.m_flags),
			.pad0 = 0,
			.data = v4(shape.m_radius, 0, 0, 0),
		};		
	}
	inline GpuShape PackShape(collision::ShapeBox const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		auto s2rb = p2rb * shape.m_base.m_s2r;
		return GpuShape {
			.s2rb = s2rb,
			.rb_bbox = s2rb * shape.m_base.m_bbox,
			.type = static_cast<int>(collision::EShape::Box),
			.vert_offset = 0,
			.vert_count = 0,
			.material_id = shape.m_base.m_material_id,
			.face_offset = 0,
			.face_count = 0,
			.edge_offset = 0,
			.edge_count = 0,
			.child_offset = 0,
			.child_count = 0,
			.flags = static_cast<int>(shape.m_base.m_flags),
			.pad0 = 0,
			.data = shape.m_radius, // half-extents (xyz), w=0
		};
	}
	inline GpuShape PackShape(collision::ShapeLine const& shape, m4x4 const& p2rb = m4x4::Identity())
	{
		auto s2rb = p2rb * shape.m_base.m_s2r;
		return GpuShape {
			.s2rb = s2rb,
			.rb_bbox = s2rb * shape.m_base.m_bbox,
			.type = static_cast<int>(collision::EShape::Line),
			.vert_offset = 0,
			.vert_count = 0,
			.material_id = shape.m_base.m_material_id,
			.face_offset = 0,
			.face_count = 0,
			.edge_offset = 0,
			.edge_count = 0,
			.child_offset = 0,
			.child_count = 0,
			.flags = static_cast<int>(shape.m_base.m_flags),
			.pad0 = 0,
			.data = v4(shape.m_hlength, shape.m_radius, 0, 0),
		};
	}
	inline GpuShape PackShape(collision::ShapeTriangle const& shape, int vert_offset, m4x4 const& p2rb = m4x4::Identity())
	{
		auto s2rb = p2rb * shape.m_base.m_s2r;
		return GpuShape {
			.s2rb = s2rb,
			.rb_bbox = s2rb * shape.m_base.m_bbox,
			.type = static_cast<int>(collision::EShape::Triangle),
			.vert_offset = vert_offset,
			.vert_count = 3, // The 3 vertices are stored at vert_offset..vert_offset+2.
			.material_id = shape.m_base.m_material_id,
			.face_offset = 0,
			.face_count = 0,
			.edge_offset = 0,
			.edge_count = 0,
			.child_offset = 0,
			.child_count = 0,
			.flags = static_cast<int>(shape.m_base.m_flags),
			.pad0 = 0,
			.data = v4::Zero(),
		};
	}
	inline GpuShape PackShape(collision::ShapePolytope const& shape, int vert_offset, int face_offset = 0, int edge_offset = 0, m4x4 const& p2rb = m4x4::Identity())
	{
		auto s2rb = p2rb * shape.m_base.m_s2r;
		return GpuShape {
			.s2rb = s2rb,
			.rb_bbox = s2rb * shape.m_base.m_bbox,
			.type = static_cast<int>(collision::EShape::Polytope),
			.vert_offset = vert_offset,
			.vert_count = shape.m_vert_count,
			.material_id = shape.m_base.m_material_id,
			.face_offset = face_offset,
			.face_count = shape.m_face_count,
			.edge_offset = edge_offset,
			.edge_count = shape.m_edge_count,
			.child_offset = 0,
			.child_count = 0,
			.flags = static_cast<int>(shape.m_base.m_flags),
			.pad0 = 0,
			.data = v4::Zero(),
		};
	}
	inline GpuShape PackShape(collision::Shape const& shape, std::vector<v4>& vertex_buffer, std::vector<GpuPolytopeFace>* face_buffer = nullptr, std::vector<GpuPolytopeEdge>* edge_buffer = nullptr, m4x4 const& p2rb = m4x4::Identity())
	{
		using namespace collision;

		switch (shape.m_type)
		{
			case EShape::NoShape:
			{
				auto s2rb = p2rb * shape.m_s2r;
				return GpuShape {
					.s2rb = s2rb,
					.rb_bbox = s2rb * shape.m_bbox,
					.type = static_cast<int>(collision::EShape::NoShape),
					.vert_offset = 0,
					.vert_count = 0,
					.material_id = 0,
					.face_offset = 0,
					.face_count = 0,
					.edge_offset = 0,
					.edge_count = 0,
					.child_offset = 0,
					.child_count = 0,
					.flags = static_cast<int>(shape.m_flags),
					.pad0 = 0,
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
				auto vert_offset = static_cast<int>(vertex_buffer.size());
				for (auto const* v = poly.vert_beg(); v != poly.vert_end(); ++v)
					vertex_buffer.push_back(*v);

				auto face_offset = 0;
				if (face_buffer != nullptr)
				{
					face_offset = static_cast<int>(face_buffer->size());
					for (auto const& face : poly.faces())
					{
						face_buffer->push_back(GpuPolytopeFace{
							.plane = face.m_plane,
							.index0 = face.m_index[0],
							.index1 = face.m_index[1],
							.index2 = face.m_index[2],
							.flags = static_cast<uint32_t>(face.m_flags),
						});
					}
				}

				auto edge_offset = 0;
				if (edge_buffer != nullptr)
				{
					edge_offset = static_cast<int>(edge_buffer->size());
					for (auto const& edge : poly.edges())
					{
						edge_buffer->push_back(GpuPolytopeEdge{
							.direction = edge.m_direction,
							.v0 = edge.m_v0,
							.v1 = edge.m_v1,
							.face0 = edge.m_face0,
							.face1 = edge.m_face1,
							.flags = static_cast<uint32_t>(edge.m_flags),
							.pad0 = 0,
							.pad1 = 0,
							.pad2 = 0,
						});
					}
				}

				return PackShape(poly, vert_offset, face_offset, edge_offset, p2rb);
			}
			case EShape::Array:
			{
				// A compound shape packs as a root entry that references a contiguous run of convex
				// leaf shapes. The leaves themselves are appended by 'PackShapeTree', which is the
				// only caller able to know where they land in the shape buffer; the root's child range
				// is patched there. A root is never used as a narrowphase operand.
				auto s2rb = p2rb * shape.m_s2r;
				return GpuShape {
					.s2rb = s2rb,
					.rb_bbox = s2rb * shape.m_bbox,
					.type = static_cast<int>(collision::EShape::Array),
					.vert_offset = 0,
					.vert_count = 0,
					.material_id = shape.m_material_id,
					.face_offset = 0,
					.face_count = 0,
					.edge_offset = 0,
					.edge_count = 0,
					.child_offset = 0,
					.child_count = 0,
					.flags = static_cast<int>(shape.m_flags),
					.pad0 = 0,
					.data = v4::Zero(),
				};
			}
			default:
			{
				assert(false && "Unsupported shape type for GPU collision");
				return {};
			}
		}
	}

	// Append the convex leaves of 'shape' to 'shapes'.
	// Nested compound shapes are flattened depth-first in declaration order so that a leaf's index
	// within the run is a stable identity for as long as the shape's declaration order is unchanged.
	// Leaf transforms are already shape-to-root, so no parent transform accumulates during descent.
	inline void PackShapeLeaves(collision::Shape const& shape, std::vector<GpuShape>& shapes, std::vector<v4>& vertex_buffer, std::vector<GpuPolytopeFace>* face_buffer, std::vector<GpuPolytopeEdge>* edge_buffer)
	{
		using namespace collision;

		// Descend into nested compounds rather than emitting a root that no narrowphase kernel understands.
		if (shape.m_type == EShape::Array)
		{
			auto const& arr = shape_cast<ShapeArray>(shape);
			for (auto const* child = arr.begin(), *end = arr.end(); child != end; child = next(child))
				PackShapeLeaves(*child, shapes, vertex_buffer, face_buffer, edge_buffer);

			return;
		}

		shapes.push_back(PackShape(shape, vertex_buffer, face_buffer, edge_buffer));
	}

	// Append 'shape' to 'shapes' as a root entry, followed by its flattened convex leaves if it is a compound.
	// Returns the index of the root entry, which is the value stored in 'GpuRigidBody::shape_id'.
	// Single-shape bodies append exactly one entry with 'child_count == 0'; no child data is produced for them.
	// Throws if the compound exceeds 'MaxCompoundChildren' so that capacity pressure is reported rather than silently truncated.
	inline int PackShapeTree(collision::Shape const& shape, std::vector<GpuShape>& shapes, std::vector<v4>& vertex_buffer, std::vector<GpuPolytopeFace>* face_buffer = nullptr, std::vector<GpuPolytopeEdge>* edge_buffer = nullptr)
	{
		auto root_index = static_cast<int>(shapes.size());
		shapes.push_back(PackShape(shape, vertex_buffer, face_buffer, edge_buffer));
		if (shape.m_type != collision::EShape::Array)
			return root_index;

		auto child_offset = static_cast<int>(shapes.size());
		PackShapeLeaves(shape, shapes, vertex_buffer, face_buffer, edge_buffer);
		auto child_count = static_cast<int>(shapes.size()) - child_offset;

		// Reject compounds that exceed the supported bound before they can corrupt child identity or pair budgets.
		if (child_count > MaxCompoundChildren)
			throw std::runtime_error(std::format("Compound collision shape has {} convex children which exceeds the maximum of {}", child_count, MaxCompoundChildren));

		// An empty compound has no leaves to collide, so leave it looking like a shape that generates no pairs.
		shapes[root_index].child_offset = child_offset;
		shapes[root_index].child_count = child_count;
		return root_index;
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
			.os_bbox = rb.Shape().m_s2r * rb.Shape().m_bbox,
			.state_flags = static_cast<int>(rb.m_state_flags),
			.shape_id = shape_id,
			.colour_used = 0,
			.pad0 = 0,
			.sleep = GpuSleepData{
				.timer_s = rb.m_sleep.m_timer_s,
				.island_id = rb.m_sleep.m_island_id,
				.generation = rb.m_sleep.m_generation,
				.flags = rb.m_sleep.m_flags,
			},
		};
	}
	inline void UnpackDynamics(GpuRigidBody const& dyn, RigidBody& rb)
	{
		assert(IsOrthonormal(dyn.o2w));
		rb.m_o2w = dyn.o2w;

		// Update momentum (the integrator advanced it by the full step)
		rb.m_ws_momentum = v8force{ dyn.momentum_ang, dyn.momentum_lin };

		// Forces are zeroed by the integrator after the second half-kick
		rb.m_ws_force = v8force{};

		// Preserve the state flags
		rb.m_state_flags = static_cast<ERigidBodyStateFlags>(dyn.state_flags);

		// Preserve the sleeping state
		rb.m_sleep = SleepData{
			.m_timer_s = dyn.sleep.timer_s,
			.m_island_id = dyn.sleep.island_id,
			.m_generation = dyn.sleep.generation,
			.m_flags = dyn.sleep.flags,
		};
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
