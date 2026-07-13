//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU-ready composite-hull primitive model for the sampled-composite buoyancy backend.
//
// A buoyant body is described by a collision::Shape which is either a single convex primitive
// (Box / Sphere / Triangle / Polytope) or a collision::ShapeArray of such primitives. The sampled
// composite buoyancy backend treats the body as the OR-union of those primitives and samples it
// volumetrically (buoyancy) and over its surface (drag).
//
// This header flattens a collision::Shape into an owned, immutable CompositeHull whose layout is
// already the layout the GPU buffers will use. We flatten/copy at registration time (rather than
// holding a bare Shape pointer) because:
//  - collision shapes use inline trailing memory keyed off m_base.m_size; they can be rebuilt,
//    compacted or relocated, which would dangle a stored pointer or shift the trailing arrays.
//  - the GPU upload in a later phase needs a contiguous, type-stable descriptor array anyway.
//
// Scope (phase 7): capture all geometry the later sampler kernels need without precomputing the
// per-primitive sampling tables (CDF / alias tables). Box and Sphere are fully parameterised
// analytically. Triangle stores its three vertices. Polytope stores its surface verts + face
// topology (planes + vertex triples) and its interior tetrahedralisation (volume verts + tets).
// The volume-/area-weighted sampling CDFs are intentionally NOT built here; they belong to the
// emit-kernel phase that defines their exact GPU layout.
#pragma once
#include <vector>
#include <cstdint>
#include <stdexcept>
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_array.h"

namespace pr::physics::buoyancy
{
	// The convex primitive types the sampled-composite backend understands. Values are fixed so
	// they can be uploaded verbatim and switched on in HLSL.
	enum class EPrimitiveType : int
	{
		Box = 0,
		Sphere = 1,
		Polytope = 2,
		Triangle = 3,
	};

	// GPU-ready descriptor for one convex primitive of a composite hull. The byte layout is the ABI
	// the compute kernels will read, so it is 16-byte aligned and pinned by a static_assert. The
	// offset/count fields index the parallel geometry arrays on the owning CompositeHull; the offsets
	// are absolute element indices into those arrays, while tet/face vertex indices are stored
	// relative to this primitive's own vertex block (see m_tet_ofs / m_face_ofs documentation).
	struct alignas(16) GpuPrimitive
	{
		// Shape-local -> centre-of-mass-root transform (shapes are authored in COM-root space).
		m4x4 m_s2r;

		// EPrimitiveType value for this primitive.
		int m_type;

		// Index of this primitive within its composite (0-based child order). This is the
		// sibling-cull priority: volume samples are owned by the lowest-index primitive that
		// contains them.
		int m_sibling_index;

		// Surface vertices for this primitive in CompositeHull::m_verts (polytope face verts and the
		// triangle's three corners; boxes/spheres store none and use m_params instead).
		int m_vert_ofs;
		int m_vert_count;

		// Interior tetrahedralisation vertices for this primitive in CompositeHull::m_volume_verts
		// (polytopes only; empty otherwise).
		int m_volume_vert_ofs;
		int m_volume_vert_count;

		// Interior tetrahedra for this primitive in CompositeHull::m_tets. Each tet's corner indices
		// are RELATIVE to this primitive's volume-vertex block (add m_volume_vert_ofs to index
		// m_volume_verts).
		int m_tet_ofs;
		int m_tet_count;

		// Outward face planes / face vertex triples for this primitive in CompositeHull::m_face_planes
		// and CompositeHull::m_face_verts (polytopes only). Face vertex indices are RELATIVE to this
		// primitive's surface-vertex block (add m_vert_ofs to index m_verts).
		int m_face_ofs;
		int m_face_count;

		// Padding to keep the integer block at 48 bytes (12 ints) so m_params starts 16-aligned.
		int m_pad0;
		int m_pad1;

		// Analytic parameters: box stores half-extents in (x,y,z); sphere stores its radius in x.
		// Unused for polytope/triangle.
		v4 m_params;
	};
	static_assert(sizeof(GpuPrimitive) == 128, "GpuPrimitive must be 128 bytes for the GPU ABI");
	static_assert((sizeof(GpuPrimitive) & 0xf) == 0, "GpuPrimitive must be 16-byte aligned");

	// An owned, immutable flattening of a collision::Shape into the sampled-composite model. The
	// primitive descriptors index the concatenated geometry arrays. All arrays are empty for a hull
	// built from boxes/spheres only; polytopes contribute surface verts + faces + tet geometry, and
	// triangles contribute their three surface verts.
	struct CompositeHull
	{
		std::vector<GpuPrimitive> m_primitives; // one descriptor per convex primitive (child order)
		std::vector<v4> m_verts;                // surface verts (polytope face verts, triangle corners)
		std::vector<v4> m_volume_verts;         // interior tetrahedralisation verts (polytopes)
		std::vector<iv4> m_tets;                // tet corners, relative to each primitive's volume block
		std::vector<v4> m_face_planes;          // outward face planes (xyz = normal, w = distance)
		std::vector<iv4> m_face_verts;          // face vertex triples, relative to each primitive's vert block

		// Return true when the hull contains no primitives (e.g. an empty ShapeArray).
		bool Empty() const
		{
			return m_primitives.empty();
		}
	};

	namespace impl
	{
		// Append one convex primitive's geometry to 'out', producing its GpuPrimitive descriptor.
		// 'shape' must be a single convex primitive (not a ShapeArray). 'sibling_index' is the
		// primitive's position within the owning composite.
		inline GpuPrimitive FlattenPrimitive(collision::Shape const& shape, int sibling_index, int polytope_tessellation, CompositeHull& out)
		{
			using namespace collision;

			// Common header fields; geometry offsets/counts default to empty and are filled per type.
			auto prim = GpuPrimitive{};
			prim.m_s2r = shape.m_s2r;
			prim.m_sibling_index = sibling_index;
			prim.m_params = v4::Zero();

			switch (shape.m_type)
			{
				case EShape::Box:
				{
					// Boxes are fully analytic: store half-extents, no concatenated geometry.
					auto const& box = shape_cast<ShapeBox>(shape);
					prim.m_type = static_cast<int>(EPrimitiveType::Box);
					prim.m_params = v4{ box.m_radius.x, box.m_radius.y, box.m_radius.z, 0.0f };
					return prim;
				}
				case EShape::Sphere:
				{
					// Spheres are fully analytic: store radius in x, no concatenated geometry.
					auto const& sph = shape_cast<ShapeSphere>(shape);
					prim.m_type = static_cast<int>(EPrimitiveType::Sphere);
					prim.m_params = v4{ sph.m_radius, 0.0f, 0.0f, 0.0f };
					return prim;
				}
				case EShape::Triangle:
				{
					// Triangles are zero-volume; store the three corners as surface verts for drag.
					auto const& tri = shape_cast<ShapeTriangle>(shape);
					prim.m_type = static_cast<int>(EPrimitiveType::Triangle);
					prim.m_vert_ofs = static_cast<int>(out.m_verts.size());
					prim.m_vert_count = 3;
					out.m_verts.push_back(tri.m_v.x);
					out.m_verts.push_back(tri.m_v.y);
					out.m_verts.push_back(tri.m_v.z);
					return prim;
				}
				case EShape::Polytope:
				{
					// Polytopes carry full topology: surface verts + face planes/triples for surface
					// sampling, and an interior tetrahedralisation for volume sampling. Collision-only
					// polytopes normally omit the latter, so derive it without modifying the source shape.
					auto const& poly = shape_cast<ShapePolytope>(shape);
					auto derived_verts = std::vector<v4>{};
					auto derived_tets = std::vector<ShapePolytope::Tet>{};
					if (poly.m_tet_count == 0 || poly.m_volume_vert_count == 0)
					{
						if (polytope_tessellation <= 0)
							throw std::runtime_error("Composite hull polytope is missing its interior tetrahedralisation and no derivation resolution was supplied");

						TessellatePolytope(poly, polytope_tessellation, derived_verts, derived_tets);
						if (derived_verts.empty() || derived_tets.empty())
							throw std::runtime_error("Composite hull polytope interior tetrahedralisation failed");
					}

					prim.m_type = static_cast<int>(EPrimitiveType::Polytope);

					// Surface verts.
					prim.m_vert_ofs = static_cast<int>(out.m_verts.size());
					prim.m_vert_count = poly.m_vert_count;
					for (int i = 0; i != poly.m_vert_count; ++i)
						out.m_verts.push_back(poly.vertex(static_cast<std::size_t>(i)));

					// Faces: outward plane + vertex triple (relative to this primitive's vert block).
					prim.m_face_ofs = static_cast<int>(out.m_face_planes.size());
					prim.m_face_count = poly.m_face_count;
					for (int f = 0; f != poly.m_face_count; ++f)
					{
						auto const& face = poly.face(f);
						out.m_face_planes.push_back(face.m_plane.m_dir_dist);
						out.m_face_verts.push_back(iv4{ face.m_index[0], face.m_index[1], face.m_index[2], 0 });
					}

					// Interior tetrahedralisation verts.
					prim.m_volume_vert_ofs = static_cast<int>(out.m_volume_verts.size());
					prim.m_tet_ofs = static_cast<int>(out.m_tets.size());
					if (!derived_tets.empty())
					{
						prim.m_volume_vert_count = static_cast<int>(derived_verts.size());
						prim.m_tet_count = static_cast<int>(derived_tets.size());
						out.m_volume_verts.insert(out.m_volume_verts.end(), derived_verts.begin(), derived_verts.end());
						for (auto const& tet : derived_tets)
							out.m_tets.push_back(iv4{ tet.m_corner[0], tet.m_corner[1], tet.m_corner[2], tet.m_corner[3] });
					}
					else
					{
						prim.m_volume_vert_count = poly.m_volume_vert_count;
						prim.m_tet_count = poly.m_tet_count;
						for (int i = 0; i != poly.m_volume_vert_count; ++i)
							out.m_volume_verts.push_back(poly.volume_vertex(static_cast<std::size_t>(i)));

						for (int t = 0; t != poly.m_tet_count; ++t)
						{
							auto const& tet = poly.tet(t);
							out.m_tets.push_back(iv4{ tet.m_corner[0], tet.m_corner[1], tet.m_corner[2], tet.m_corner[3] });
						}
					}
					return prim;
				}
				default:
				{
					throw std::runtime_error("Unsupported collision shape type for composite buoyancy hull");
				}
			}
		}
	}

	// Flatten a collision::Shape into an owned CompositeHull. A ShapeArray is decomposed into its
	// child primitives in child order (the sibling-cull priority); any other shape is treated as a
	// single primitive. Missing polytope tetrahedra are derived when 'polytope_tessellation' is
	// positive; otherwise they are rejected.
	inline CompositeHull FlattenShape(collision::Shape const& hull, int polytope_tessellation = 0)
	{
		using namespace collision;

		auto out = CompositeHull{};
		if (hull.m_type == EShape::Array)
		{
			// Iterate the array's children in order; each child becomes one primitive.
			auto const& arr = shape_cast<ShapeArray>(hull);
			auto sibling = 0;
			for (Shape const* s = arr.begin(), *e = arr.end(); s != e; s = next(s), ++sibling)
				out.m_primitives.push_back(impl::FlattenPrimitive(*s, sibling, polytope_tessellation, out));
		}
		else
		{
			out.m_primitives.push_back(impl::FlattenPrimitive(hull, 0, polytope_tessellation, out));
		}
		return out;
	}
}
