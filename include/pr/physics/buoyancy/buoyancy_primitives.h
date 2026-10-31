//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU-ready composite-hull primitive model for the sampled-composite buoyancy backend.
//
// A buoyant body is described by a collision::Shape which is either a single convex primitive
// (Box / Sphere / Capsule / Triangle / Polytope) or a collision::ShapeArray of such primitives. The sampled
// composite buoyancy backend treats the body as the OR-union of those primitives and samples it
// over its surface: water pressure on the wet surface gives buoyancy, and the relative flow gives drag.
//
// This header flattens a collision::Shape into an owned, immutable CompositeHull whose layout is
// already the layout the GPU buffers will use. We flatten/copy at registration time (rather than
// holding a bare Shape pointer) because:
//  - collision shapes use inline trailing memory keyed off m_base.m_size; they can be rebuilt,
//    compacted or relocated, which would dangle a stored pointer or shift the trailing arrays.
//  - the GPU upload in a later phase needs a contiguous, type-stable descriptor array anyway.
//
// Capture all immutable geometry the sampler kernels need. Box, Sphere and Capsule (a collision::ShapeLine with a positive radius)
// are fully parameterised analytically. Triangle stores its three vertices. Polytope stores its surface topology (vertices and
// face planes). Surface quadrature plans are built separately by BuildSurfacePlan.
#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_array.h"
#include "pr/physics/surface/surface_sampling.h"

namespace pr::physics::buoyancy
{
	// Default buoyancy surface sample density, shared by the GPU pass, the CPU oracle and diagnostic overlays. The spacing is the
	// maximum surface cell diameter in metres. Water pressure (buoyancy) and drag are both integrated over these surface cells.
	// BuoyancySampleDensityTests reports the behavioural error of this value against a dense reference; see that test before changing it.
	inline constexpr float DefaultSurfaceSpacing = 0.25f;

	// The fewest surface samples given to a primitive. A light body floats with a shallow draft, so only the few cells near its
	// lowest point are wet. With too few cells their discrete layout gives a wrong floating height and false torques, so small
	// primitives are sampled more finely than the spacing alone would give. Primitives that already have this many samples keep their spacing.
	inline constexpr uint32_t MinSurfaceSamplesPerPrimitive = 128;

	// Return the submerged fraction of a sample cell centred 'signed_height' along up, given the water height at the same
	// planar position and the cell's vertical extent. The fraction rises linearly from 0, when the cell top touches the surface,
	// to 1, when the cell bottom is under it. This makes sampled forces continuous functions of depth. A zero extent gives a step.
	// Mirrored in HLSL by BuoyWetFraction.
	inline float CellWetFraction(float signed_height, float water_height, float cell_height)
	{
		// The tiny floor keeps a zero-height cell a step function instead of 0/0 at the surface.
		return std::clamp((water_height - signed_height) / std::max(cell_height, 1e-6f) + 0.5f, 0.0f, 1.0f);
	}

	// Return the vertical extent of a surface cell with area 'darea' whose unit normal has component 'normal_dot_up' along up.
	// A cell of side sqrt(darea) tilted by angle theta from horizontal spans sqrt(darea) * sin(theta) vertically, so a cell on a
	// steep face wets gradually while a horizontal cell wets all at once. Mirrored in HLSL by BuoySurfaceCellHeight.
	inline float SurfaceCellHeight(float darea, float normal_dot_up)
	{
		return std::sqrt(darea * std::max(0.0f, 1.0f - normal_dot_up * normal_dot_up));
	}

	// The convex primitive types the sampled-composite backend understands. Values are fixed so
	// they can be uploaded verbatim and switched on in HLSL.
	enum class EPrimitiveType : int
	{
		Box = 0,
		Sphere = 1,
		Polytope = 2,
		Triangle = 3,
		Capsule = 4,
	};

	// GPU-ready descriptor for one convex primitive of a composite hull. The byte layout is the ABI
	// the compute kernels will read, so it is 16-byte aligned and pinned by a static_assert. The
	// offset/count fields index the parallel geometry arrays on the owning CompositeHull; the offsets
	// are absolute element indices into those arrays, while face vertex indices are stored
	// relative to this primitive's own vertex block (see m_face_ofs documentation).
	struct alignas(16) GpuPrimitive
	{
		// Shape-local -> centre-of-mass-root transform (shapes are authored in COM-root space).
		m4x4 m_s2r;

		// EPrimitiveType value for this primitive.
		int m_type;

		// Index of this primitive within its composite (0-based child order).
		int m_sibling_index;

		// Surface vertices for this primitive in CompositeHull::m_verts (polytope face verts and the
		// triangle's three corners; boxes/spheres store none and use m_params instead).
		int m_vert_ofs;
		int m_vert_count;

		// Outward face planes / face vertex triples for this primitive in CompositeHull::m_face_planes
		// and CompositeHull::m_face_verts (polytopes only). Face vertex indices are RELATIVE to this
		// primitive's surface-vertex block (add m_vert_ofs to index m_verts).
		int m_face_ofs;
		int m_face_count;

		// Padding to keep the integer block at 32 bytes (8 ints) so m_params starts 16-aligned.
		int m_pad0;
		int m_pad1;

		// Analytic parameters: box stores half-extents in (x,y,z); sphere stores its radius in x;
		// capsule stores its radius in x and the half-length of its Z-axis segment in y. Unused for polytope/triangle.
		v4 m_params;
	};
	static_assert(sizeof(GpuPrimitive) == 112, "GpuPrimitive must be 112 bytes for the GPU ABI");
	static_assert((sizeof(GpuPrimitive) & 0xf) == 0, "GpuPrimitive must be 16-byte aligned");

	// An owned, immutable flattening of a collision::Shape into the sampled-composite model. The
	// primitive descriptors index the concatenated geometry arrays. All arrays are empty for a hull
	// built from boxes/spheres only; polytopes contribute surface verts + faces, and
	// triangles contribute their three surface verts.
	struct CompositeHull
	{
		std::vector<GpuPrimitive> m_primitives; // one descriptor per convex primitive (child order)
		std::vector<v4> m_verts;                // surface verts (polytope face verts, triangle corners)
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
		inline GpuPrimitive FlattenPrimitive(collision::Shape const& shape, int sibling_index, CompositeHull& out)
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
				case EShape::Line:
				{
					// Thick lines are capsules and are fully analytic. A thin line has no volume and no outward surface normal, so it cannot float.
					auto const& line = shape_cast<ShapeLine>(shape);
					if (!(line.m_radius > 0.0f) || !(line.m_hlength >= 0.0f))
						throw std::runtime_error("Composite buoyancy hull lines must be capsules with a positive radius");

					prim.m_type = static_cast<int>(EPrimitiveType::Capsule);
					prim.m_params = v4{ line.m_radius, line.m_hlength, 0.0f, 0.0f };
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
					// Polytopes carry their surface topology: verts + face planes/triples for surface sampling and the sibling inside test.
					auto const& poly = shape_cast<ShapePolytope>(shape);
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
	// single primitive.
	inline CompositeHull FlattenShape(collision::Shape const& hull)
	{
		using namespace collision;

		auto out = CompositeHull{};
		if (hull.m_type == EShape::Array)
		{
			// Iterate the array's children in order; each child becomes one primitive.
			auto const& arr = shape_cast<ShapeArray>(hull);
			auto sibling = 0;
			for (Shape const* s = arr.begin(), *e = arr.end(); s != e; s = next(s), ++sibling)
				out.m_primitives.push_back(impl::FlattenPrimitive(*s, sibling, out));
		}
		else
		{
			out.m_primitives.push_back(impl::FlattenPrimitive(hull, 0, out));
		}
		return out;
	}

	// Build the surface sampling plan for one buoyancy primitive. Cells are at most 'spacing' across, and the spacing is reduced
	// until the primitive has at least MinSurfaceSamplesPerPrimitive samples. A primitive that reaches the minimum at 'spacing'
	// gets exactly surface::BuildPlan(prim, spacing). The GPU pass and the CPU oracle both use this plan.
	inline surface::Plan BuildSurfacePlan(collision::Shape const& prim, float spacing)
	{
		// Most primitives already have enough samples at the requested spacing.
		auto plan = surface::BuildPlan(prim, spacing);

		// The sample count grows about as 1/spacing^2, so scale the spacing by sqrt(count / minimum) and rebuild. Grid boundary
		// nodes make the count grow more slowly than that for coarse grids, so repeat until the minimum is met; this takes a few
		// iterations at most. A plan without area (a zero-size primitive) has no pressure to resolve and cannot be refined.
		auto const has_area = std::ranges::any_of(plan.m_patches, [](surface::SurfacePatch const& patch) { return patch.m_measure > 0.0f; });
		while (has_area && plan.m_count < MinSurfaceSamplesPerPrimitive)
		{
			// Refine the spacing in proportion to the shortfall.
			spacing *= static_cast<float>(std::sqrt(static_cast<double>(std::max(plan.m_count, 1u)) / MinSurfaceSamplesPerPrimitive));
			plan = surface::BuildPlan(prim, spacing);
		}
		return plan;
	}
}
