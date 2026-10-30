//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/shape/mass.h"
#include "pr/physics/shape/inertia.h"

namespace pr::physics
{
	// Return the solid unit inertia for the sphere: 'Ixx = Iyy = Izz = (2/5)mr^2'
	inline m3x3 UnitInertia(ShapeSphere const& shape)
	{
		auto inertia = m3x3{};
		inertia.x.x = (2.0f / 5.0f) * Sqr(shape.m_radius);
		inertia.y.y = inertia.x.x;
		inertia.z.z = inertia.x.x;
		return inertia;
	}

	// Return the unit inertia for the box
	inline m3x3 UnitInertia(ShapeBox const& shape)
	{
		auto inertia = m3x3{};
		inertia.x.x = (1.0f / 3.0f) * (Sqr(shape.m_radius.y) + Sqr(shape.m_radius.z)); // (1/12)m(Y^2 + Z^2)
		inertia.y.y = (1.0f / 3.0f) * (Sqr(shape.m_radius.z) + Sqr(shape.m_radius.x)); // (1/12)m(Z^2 + X^2)
		inertia.z.z = (1.0f / 3.0f) * (Sqr(shape.m_radius.x) + Sqr(shape.m_radius.y)); // (1/12)m(X^2 + Y^2)
		return inertia;
	}

	// Return the unit inertia for the triangle.
	// Computed about the shape origin. SetMassProperties applies the parallel axis
	// theorem to translate from origin to centre of mass using mp.m_centre_of_mass.
	inline m3x3 UnitInertia(ShapeTriangle const& shape)
	{
		// Compute the inertia tensor about the origin using the vertex positions.
		// For a surface element (thin plate), each vertex contributes equally (1/3).
		auto inertia = m3x3{};
		for (int i = 0; i != 3; ++i)
		{
			auto v = shape.m_v[i];
			v.w = 0;
			inertia.x.x += Sqr(v.y) + Sqr(v.z);
			inertia.y.y += Sqr(v.z) + Sqr(v.x);
			inertia.z.z += Sqr(v.x) + Sqr(v.y);
			inertia.x.y -= v.x * v.y;
			inertia.x.z -= v.x * v.z;
			inertia.y.z -= v.y * v.z;
		}

		// Mirror off-diagonal elements (inertia tensor is symmetric)
		inertia.y.x = inertia.x.y;
		inertia.z.x = inertia.x.z;
		inertia.z.y = inertia.y.z;
		return inertia;
	}

	// Return the unit inertia for the line shape.
	// A line with radius is a solid capsule along Z: a cylinder of length L = 2*m_hlength with a hemisphere of radius r at each end.
	// A line with zero radius is a thin rod: Ixx = Iyy = (1/12)*L^2, Izz ≈ 0
	inline m3x3 UnitInertia(ShapeLine const& shape)
	{
		auto L = 2.0f * shape.m_hlength; // Full length
		auto r = shape.m_radius;         // Collision radius (half-thickness)
		auto inertia = m3x3{};
		if (r > math::tiny<float>)
		{
			// Split the unit mass between the cylinder and the two hemispheres in proportion to their volumes.
			auto cylinder_volume = L;
			auto ends_volume = (4.0f / 3.0f) * r;
			auto m_cyl = cylinder_volume / (cylinder_volume + ends_volume);
			auto m_ends = 1.0f - m_cyl;

			// The cylinder uses the solid cylinder formula. Each hemisphere has inertia (2/5)r^2 about its flat face, and its centroid sits
			// 3r/8 from that face. Moving that inertia to the capsule centre, a distance L/2 + 3r/8 away, gives (2/5)r^2 + L^2/4 + 3Lr/8.
			inertia.x.x = m_cyl * (1.0f / 12.0f) * (3.0f * Sqr(r) + Sqr(L)) + m_ends * ((2.0f / 5.0f) * Sqr(r) + 0.25f * Sqr(L) + (3.0f / 8.0f) * L * r);
			inertia.y.y = inertia.x.x;
			inertia.z.z = m_cyl * 0.5f * Sqr(r) + m_ends * (2.0f / 5.0f) * Sqr(r);
		}
		else
		{
			// Thin rod along Z (per unit mass)
			inertia.x.x = (1.0f / 12.0f) * Sqr(L);
			inertia.y.y = inertia.x.x;
			inertia.z.z = math::tiny<float>; // Near-zero to avoid singular inertia
		}
		return inertia;
	}

	// Returns the unit inertia for the polytope, computed about the shape origin.
	// SetMassProperties applies the parallel axis theorem to translate from origin
	// to the centre of mass using mp.m_centre_of_mass.
	// Uses the divergence theorem to integrate x², y², z² etc. over the volume.
	inline m3x3 UnitInertia(ShapePolytope const& shape)
	{
		auto volume   = 0.0f; // Technically this variable accumulates the volume times 6
		auto diagonal = v4{}; // Accumulate matrix main diagonals [x*x, y*y, z*z]
		auto off_diag = v4{}; // Accumulate matrix off-diagonals  [y*z, x*z, x*y]
		for (ShapePolyFace const *f = shape.face_beg(), *fend = shape.face_end(); f != fend; ++f)
		{
			auto a = shape.vertex(f->m_index[0]);
			auto b = shape.vertex(f->m_index[1]);
			auto c = shape.vertex(f->m_index[2]);
			auto vol_x6 = Triple(a, b, c); // Triple product is volume x 6
			volume += vol_x6;

			for (int i = 0, j = 1, k = 2; i != 3; ++i, (++j) %= 3, (++k) %= 3)
			{
				diagonal[i] += (
					a[i] * b[i] +
					b[i] * c[i] +
					c[i] * a[i] +
					a[i] * a[i] +
					b[i] * b[i] +
					c[i] * c[i]
					) * vol_x6; // Divide by 60.0f later
				off_diag[i] += (
					a[j] * b[k] +
					b[j] * c[k] +
					c[j] * a[k] +
					a[j] * c[k] +
					b[j] * a[k] +
					c[j] * b[k] +
					a[j] * a[k] * 2.0f +
					b[j] * b[k] * 2.0f +
					c[j] * c[k] * 2.0f
					) * vol_x6; // Divide by 120.0f later
			}
		}

		// If the polytope is degenerate, use the average vertex position as a point mass
		if (FEql(volume, 0.f))
		{
			auto centre = v4::Zero();
			for (v4 const *v = shape.vert_beg(), *vend = shape.vert_end(); v != vend; ++v) centre += *v;
			centre /= static_cast<float>(shape.m_vert_count);
			return Inertia::Point(1.0f, centre).To3x3();
		}

		// Divide by total volume — gives the per-unit-mass inertia about the origin
		volume   /= 6.0f;
		diagonal /= volume * 60.0f;
		off_diag /= volume * 120.0f;
		auto Io = m3x3{
			v4{diagonal.y + diagonal.z , -off_diag.z             , -off_diag.y           ,0},
			v4{-off_diag.z             , diagonal.x + diagonal.z , -off_diag.x           ,0},
			v4{-off_diag.y             , -off_diag.x             , diagonal.x+diagonal.y ,0}};

		return Io;
	}

	// Transform mass properties from shape-local space into the rigid-body root space.
	inline MassProperties TransformMassProperties(MassProperties mp, m4x4 const& s2r)
	{
		auto local_com = mp.m_centre_of_mass;
		auto root_com = (s2r * local_com.w1()).w0();

		auto inertia = Inertia{mp.m_os_unit_inertia, 1.0f};
		if (LengthSq(local_com) != 0)
			inertia = Translate(inertia, local_com, ETranslateInertia::TowardCoM);

		inertia = Rotate(inertia, s2r.rot);
		if (LengthSq(root_com) != 0)
			inertia = Translate(inertia, root_com, ETranslateInertia::AwayFromCoM);

		mp.m_centre_of_mass = root_com;
		mp.m_os_unit_inertia = inertia.To3x3(1);
		return mp;
	}

	// Return the shape-local mass properties of a thin uniform shell on the surface of a sphere. 'areal_density' is in kg/m².
	inline MassProperties ShellMassProperties(ShapeSphere const& shape, float areal_density)
	{
		// A spherical shell has 'I = (2/3)mr^2' about every axis through its centre
		auto r = shape.m_radius;
		auto area = 2.0f * float(constants<double>::tau) * Sqr(r);
		auto unit_inertia = m3x3::Scale((2.0f / 3.0f) * Sqr(r));
		return MassProperties(unit_inertia, v4{}, area * areal_density);
	}

	// Return the shape-local mass properties of a thin uniform shell on the surface of a box. 'areal_density' is in kg/m².
	inline MassProperties ShellMassProperties(ShapeBox const& shape, float areal_density)
	{
		// Sum the six face plates. For the axis 'h0', with the other half extents 'h1' and 'h2', the two plates normal to h0 contribute
		// (1/3)A(h1²+h2²), and each other pair contributes A times the squared offset of the plate plus (1/3) of the in-plane half extent squared.
		auto h = shape.m_radius;
		auto AxisSecondMoment = [](float h0, float h1, float h2)
		{
			return
				(8.0f / 3.0f) * h1 * h2 * (Sqr(h1) + Sqr(h2)) +
				8.0f * h0 * h2 * (Sqr(h1) + Sqr(h2) / 3.0f) +
				8.0f * h0 * h1 * (Sqr(h1) / 3.0f + Sqr(h2));
		};
		auto area = 8.0f * (h.x * h.y + h.y * h.z + h.z * h.x);
		auto unit_inertia = m3x3{};
		unit_inertia.x.x = AxisSecondMoment(h.x, h.y, h.z) / area;
		unit_inertia.y.y = AxisSecondMoment(h.y, h.z, h.x) / area;
		unit_inertia.z.z = AxisSecondMoment(h.z, h.x, h.y) / area;
		return MassProperties(unit_inertia, v4{}, area * areal_density);
	}

	// Return the shape-local mass properties of a thin uniform shell on the surface of a capsule (a line with a positive radius).
	// 'areal_density' is in kg/m².
	inline MassProperties ShellMassProperties(ShapeLine const& shape, float areal_density)
	{
		// A zero-radius line has no surface to hold a shell
		auto L = 2.0f * shape.m_hlength;
		auto r = shape.m_radius;
		if (!(r > math::tiny<float>))
			throw std::invalid_argument("Shell mass distribution requires a line with a positive radius (a capsule)");

		// Split the unit mass between the open cylinder (area 2πrL) and the two hemispherical ends (area 4πr²) in proportion to their areas.
		auto m_cyl = L / (L + 2.0f * r);
		auto m_ends = 1.0f - m_cyl;

		// An open cylindrical shell has Ixx = r²/2 + L²/12 and Izz = r². A hemispherical shell has (2/3)r² about its flat face centre and its
		// centroid sits r/2 from that face. Moving that inertia to the capsule centre, a distance L/2 + r/2 away, gives (2/3)r² + L²/4 + Lr/2.
		auto unit_inertia = m3x3{};
		unit_inertia.x.x = m_cyl * (0.5f * Sqr(r) + Sqr(L) / 12.0f) + m_ends * ((2.0f / 3.0f) * Sqr(r) + 0.25f * Sqr(L) + 0.5f * L * r);
		unit_inertia.y.y = unit_inertia.x.x;
		unit_inertia.z.z = m_cyl * Sqr(r) + m_ends * (2.0f / 3.0f) * Sqr(r);

		auto area = float(constants<double>::tau) * r * (L + 2.0f * r);
		return MassProperties(unit_inertia, v4{}, area * areal_density);
	}

	// Return the shape-local mass properties of a thin uniform shell on the hull faces of a polytope. 'areal_density' is in kg/m².
	// The inertia is measured about the shape origin, like the solid polytope.
	inline MassProperties ShellMassProperties(ShapePolytope const& shape, float areal_density)
	{
		// Treat each hull face as a uniform triangular plate. A triangle (a, b, c) with area A has first moment A(a+b+c)/3
		// and second moment ∫ x xᵀ dA = (A/12)(aaᵀ + bbᵀ + ccᵀ + ssᵀ), where s = a + b + c.
		auto area = 0.0f;
		auto first_moment = v4{};
		auto diagonal = v4{}; // Accumulate second moments [x*x, y*y, z*z]
		auto off_diag = v4{}; // Accumulate second moments [y*z, x*z, x*y]
		for (ShapePolyFace const *f = shape.face_beg(), *fend = shape.face_end(); f != fend; ++f)
		{
			// Face area and centroid contribution
			auto a = shape.vertex(f->m_index[0]).w0();
			auto b = shape.vertex(f->m_index[1]).w0();
			auto c = shape.vertex(f->m_index[2]).w0();
			auto s = a + b + c;
			auto face_area = 0.5f * Length(Cross(b - a, c - a));
			area += face_area;
			first_moment += face_area * s / 3.0f;

			// Face second moment contribution
			for (int i = 0, j = 1, k = 2; i != 3; ++i, (++j) %= 3, (++k) %= 3)
			{
				diagonal[i] += (face_area / 12.0f) * (a[i] * a[i] + b[i] * b[i] + c[i] * c[i] + s[i] * s[i]);
				off_diag[i] += (face_area / 12.0f) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k] + s[j] * s[k]);
			}
		}
		if (!(area > 0.0f))
			throw std::invalid_argument("Shell mass distribution requires a polytope with a positive surface area");

		// Convert the per-unit-area second moments into the per-unit-mass inertia about the origin
		diagonal /= area;
		off_diag /= area;
		auto unit_inertia = m3x3{
			v4{diagonal.y + diagonal.z , -off_diag.z              , -off_diag.y             , 0},
			v4{-off_diag.z             , diagonal.x + diagonal.z  , -off_diag.x             , 0},
			v4{-off_diag.y             , -off_diag.x              , diagonal.x + diagonal.y , 0}};

		return MassProperties(unit_inertia, (first_moment / area).w0(), area * areal_density);
	}

	// Return the mass properties. See EMassDistribution for the meaning of 'density'.
	inline MassProperties CalcMassProperties(ShapeSphere const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid)
	{
		switch (distribution)
		{
			case EMassDistribution::Solid:
			{
				auto volume = float((2.0/3.0) * constants<double>::tau * shape.m_radius * shape.m_radius * shape.m_radius);
				auto mp = MassProperties(UnitInertia(shape), v4{}, volume * density);
				return TransformMassProperties(mp, shape.m_base.m_s2r);
			}
			case EMassDistribution::Shell:
			{
				return TransformMassProperties(ShellMassProperties(shape, density), shape.m_base.m_s2r);
			}
			default:
			{
				throw std::invalid_argument("Unknown mass distribution");
			}
		}
	}

	// Return the mass properties. See EMassDistribution for the meaning of 'density'.
	inline MassProperties CalcMassProperties(ShapeBox const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid)
	{
		switch (distribution)
		{
			case EMassDistribution::Solid:
			{
				auto volume = 8.0f * shape.m_radius.x * shape.m_radius.y * shape.m_radius.z;
				auto mp = MassProperties(UnitInertia(shape), v4{}, volume * density);
				return TransformMassProperties(mp, shape.m_base.m_s2r);
			}
			case EMassDistribution::Shell:
			{
				return TransformMassProperties(ShellMassProperties(shape, density), shape.m_base.m_s2r);
			}
			default:
			{
				throw std::invalid_argument("Unknown mass distribution");
			}
		}
	}

	// Return the mass properties. A triangle is already a surface, so 'density' is mass per unit area and only the solid distribution is valid.
	inline MassProperties CalcMassProperties(ShapeTriangle const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid)
	{
		// A triangle encloses no volume, so it cannot have a shell
		if (distribution != EMassDistribution::Solid)
			throw std::invalid_argument("Triangle shapes only support the solid mass distribution");

		MassProperties mp;
		mp.m_centre_of_mass  = (1.0f / 3.0f) * (shape.m_v.x + shape.m_v.y + shape.m_v.z).w0();
		mp.m_mass            = 0.5f * Length(Cross(shape.m_v.y - shape.m_v.x, shape.m_v.z - shape.m_v.y)) * density;
		mp.m_os_unit_inertia = UnitInertia(shape);
		return TransformMassProperties(mp, shape.m_base.m_s2r);
	}

	// Return the mass properties for the line shape. See EMassDistribution for the meaning of 'density'.
	// The shell distribution requires a positive radius (a capsule).
	inline MassProperties CalcMassProperties(ShapeLine const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid)
	{
		switch (distribution)
		{
			case EMassDistribution::Solid:
			{
				// A line with radius is a capsule: a cylinder of length 2*m_hlength plus a sphere made of its two hemispherical ends.
				// For zero-thickness lines, use a small minimum volume to avoid zero mass.
				auto L = 2.0f * shape.m_hlength;
				auto r = shape.m_radius;
				auto volume = (r > math::tiny<float>)
					? float(constants<double>::tau_by_2) * Sqr(r) * (L + (4.0f / 3.0f) * r)  // pi * r^2 * L + (4/3) * pi * r^3
					: L * math::tiny<float>;                                                 // Fallback: thin rod with minimal cross-section

				auto mp = MassProperties(UnitInertia(shape), v4{}, volume * density);
				return TransformMassProperties(mp, shape.m_base.m_s2r);
			}
			case EMassDistribution::Shell:
			{
				return TransformMassProperties(ShellMassProperties(shape, density), shape.m_base.m_s2r);
			}
			default:
			{
				throw std::invalid_argument("Unknown mass distribution");
			}
		}
	}

	// Return mass properties for the polytope. See EMassDistribution for the meaning of 'density'.
	inline MassProperties CalcMassProperties(ShapePolytope const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid)
	{
		switch (distribution)
		{
			case EMassDistribution::Solid:
			{
				auto mp = MassProperties(UnitInertia(shape), CalcCentreOfMass(shape), CalcVolume(shape) * density);
				return TransformMassProperties(mp, shape.m_base.m_s2r);
			}
			case EMassDistribution::Shell:
			{
				return TransformMassProperties(ShellMassProperties(shape, density), shape.m_base.m_s2r);
			}
			default:
			{
				throw std::invalid_argument("Unknown mass distribution");
			}
		}
	}

	// Calculate the mass properties of a shape (forward declaration for the array aggregation)
	template <typename = void>
	inline MassProperties CalcMassProperties(Shape const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid);

	// Return mass properties for a composite (array of convex primitives).
	// Each child's CalcMassProperties folds that child's own shape-to-root transform, and a child's
	// transform is absolute within the owning root rather than relative to the array, so the child
	// results are already expressed about the root origin. Because the child inertias all reference
	// that one common point they add directly; masses and first moments (m*com) also sum. The array's
	// own s2r must not be applied again here, or it would be counted twice.
	// The distribution applies to every child, so a shell array is a set of separate closed skins rather than the skin of the union.
	inline MassProperties CalcMassProperties(ShapeArray const& shape, float density, EMassDistribution distribution = EMassDistribution::Solid)
	{
		auto total_mass = 0.0f;
		auto first_moment = v4{}; // Sum of m_i * com_i (about the root origin)
		auto inertia_o = m3x3{};  // Sum of child inertias about the root origin

		for (Shape const* i = shape.begin(), *i_end = shape.end(); i != i_end; i = next(i))
		{
			auto cmp = CalcMassProperties(*i, density, distribution);
			total_mass += cmp.m_mass;
			first_moment += cmp.m_mass * cmp.m_centre_of_mass;
			inertia_o += Inertia{cmp.m_os_unit_inertia, cmp.m_mass}.To3x3();
		}

		MassProperties mp;
		mp.m_mass = total_mass;
		mp.m_centre_of_mass = total_mass > 0.0f ? (first_moment / total_mass).w0() : v4{};
		mp.m_os_unit_inertia = total_mass > 0.0f ? (inertia_o / total_mass) : m3x3{};
		return mp;
	}

	// Calculate the mass properties of a shape. See EMassDistribution for the meaning of 'density'.
	template <typename>
	inline MassProperties CalcMassProperties(Shape const& shape, float density, EMassDistribution distribution)
	{
		switch (shape.m_type)
		{
		default: assert("Unknown primitive type" && false); return MassProperties();
		case EShape::Sphere:   return CalcMassProperties(shape_cast<ShapeSphere>(shape), density, distribution);
		case EShape::Box:      return CalcMassProperties(shape_cast<ShapeBox>(shape), density, distribution);
		case EShape::Line:     return CalcMassProperties(shape_cast<ShapeLine>(shape), density, distribution);
		case EShape::Triangle: return CalcMassProperties(shape_cast<ShapeTriangle>(shape), density, distribution);
		case EShape::Polytope: return CalcMassProperties(shape_cast<ShapePolytope>(shape), density, distribution);
		case EShape::Array:    return CalcMassProperties(shape_cast<ShapeArray>(shape), density, distribution);
		}
	}
}
