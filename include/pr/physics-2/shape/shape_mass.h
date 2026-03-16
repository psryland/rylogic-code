//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/shape/mass.h"
#include "pr/physics-2/shape/inertia.h"

namespace pr::physics
{
	// Return the unit inertia for the sphere
	inline m3x4 UnitInertia(ShapeSphere const& shape)
	{
		// A solid sphere:  'Ixx = Iyy = Izz = (2/5)mr^2'
		// A hollow sphere: 'Ixx = Iyy = Izz = (2/3)mr^2'
		auto inertia = m3x4{};
		inertia.x.x = float(shape.m_hollow ? (2.0/3.0) : (2.0/5.0)) * Sqr(shape.m_radius);
		inertia.y.y = inertia.x.x;
		inertia.z.z = inertia.x.x;
		return inertia;
	}

	// Return the unit inertia for the box
	inline m3x4 UnitInertia(ShapeBox const& shape)
	{
		auto inertia = m3x4{};
		inertia.x.x = (1.0f / 3.0f) * (Sqr(shape.m_radius.y) + Sqr(shape.m_radius.z)); // (1/12)m(Y^2 + Z^2)
		inertia.y.y = (1.0f / 3.0f) * (Sqr(shape.m_radius.z) + Sqr(shape.m_radius.x)); // (1/12)m(Z^2 + X^2)
		inertia.z.z = (1.0f / 3.0f) * (Sqr(shape.m_radius.x) + Sqr(shape.m_radius.y)); // (1/12)m(X^2 + Y^2)
		return inertia;
	}

	// Return the unit inertia for the triangle.
	// Computed about the model origin. SetMassProperties applies the parallel axis
	// theorem to translate from origin to centre of mass using mp.m_centre_of_mass.
	inline m3x4 UnitInertia(ShapeTriangle const& shape)
	{
		// Compute the inertia tensor about the origin using the vertex positions.
		// For a surface element (thin plate), each vertex contributes equally (1/3).
		auto inertia = m3x4{};
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
	// Modelled as a thin rod along Z (length = 2*m_radius) with optional cylindrical thickness.
	// When thickness > 0, uses a solid cylinder inertia: Ixx = Iyy = (1/12)*(3r^2 + L^2), Izz = (1/2)*r^2
	// When thickness == 0, uses a thin rod: Ixx = Iyy = (1/12)*L^2, Izz ≈ 0
	inline m3x4 UnitInertia(ShapeLine const& shape)
	{
		auto L = 2.0f * shape.m_radius;     // Full length
		auto r = shape.m_thickness;          // Collision radius (half-thickness)
		auto inertia = m3x4{};
		if (r > math::tiny<float>)
		{
			// Solid cylinder inertia (per unit mass)
			inertia.x.x = (1.0f / 12.0f) * (3.0f * Sqr(r) + Sqr(L));
			inertia.y.y = inertia.x.x;
			inertia.z.z = 0.5f * Sqr(r);
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

	// Returns the unit inertia for the polytope, computed about the model origin.
	// SetMassProperties applies the parallel axis theorem to translate from origin
	// to the centre of mass using mp.m_centre_of_mass.
	// Uses the divergence theorem to integrate x², y², z² etc. over the volume.
	inline m3x4 UnitInertia(ShapePolytope const& shape)
	{
		// Notes:
		//  Ensure the polytope is in it's final space before calculating its inertia.
		//  Calling 'ShiftCentre' invalidates the inertia matrix.

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
		auto Io = m3x4{
			v4{diagonal.y + diagonal.z , -off_diag.z             , -off_diag.y           ,0},
			v4{-off_diag.z             , diagonal.x + diagonal.z , -off_diag.x           ,0},
			v4{-off_diag.y             , -off_diag.x             , diagonal.x+diagonal.y ,0}};

		return Io;
	}

	// Return the mass properties
	inline MassProperties CalcMassProperties(ShapeSphere const& shape, float density)
	{
		auto volume = float((2.0/3.0) * constants<double>::tau * shape.m_radius * shape.m_radius * shape.m_radius);

		MassProperties mp;
		mp.m_centre_of_mass  = v4{};
		mp.m_mass            = volume * density;
		mp.m_os_unit_inertia = UnitInertia(shape);
		return mp;
	}

	// Return the mass properties
	inline MassProperties CalcMassProperties(ShapeBox const& shape, float density)
	{
		auto volume = 8.0f * shape.m_radius.x * shape.m_radius.y * shape.m_radius.z;

		MassProperties mp;
		mp.m_centre_of_mass  = v4{};
		mp.m_mass            = volume * density;
		mp.m_os_unit_inertia = UnitInertia(shape);
		return mp;
	}

	// Return the mass properties
	inline MassProperties CalcMassProperties(ShapeTriangle const& shape, float density)
	{
		MassProperties mp;
		mp.m_centre_of_mass  = (1.0f / 3.0f) * (shape.m_v.x + shape.m_v.y + shape.m_v.z).w0();
		mp.m_mass            = 0.5f * Length(Cross(shape.m_v.y - shape.m_v.x, shape.m_v.z - shape.m_v.y)) * density;
		mp.m_os_unit_inertia = UnitInertia(shape);
		return mp;
	}

	// Return the mass properties for the line shape
	inline MassProperties CalcMassProperties(ShapeLine const& shape, float density)
	{
		// Volume depends on thickness: cylinder of radius m_thickness and length 2*m_radius.
		// For zero-thickness lines, use a small minimum volume to avoid zero mass.
		auto L = 2.0f * shape.m_radius;
		auto r = shape.m_thickness;
		auto volume = (r > math::tiny<float>)
			? float(constants<double>::tau_by_2) * Sqr(r) * L  // pi * r^2 * L
			: L * math::tiny<float>;                          // Fallback: thin rod with minimal cross-section

		MassProperties mp;
		mp.m_centre_of_mass  = v4{};
		mp.m_mass            = volume * density;
		mp.m_os_unit_inertia = UnitInertia(shape);
		return mp;
	}

	// Return mass properties for the polytope
	inline MassProperties CalcMassProperties(ShapePolytope const& shape, float density)
	{
		MassProperties mp;
		mp.m_centre_of_mass  = CalcCentreOfMass(shape);
		mp.m_mass            = CalcVolume(shape) * density;
		mp.m_os_unit_inertia = UnitInertia(shape);
		return mp;
	}

	// Calculate the mass properties of a shape
	template <typename = void>
	inline MassProperties CalcMassProperties(Shape const& shape, float density)
	{
		switch (shape.m_type)
		{
		default: assert("Unknown primitive type" && false); return MassProperties();
		case EShape::Sphere:   return CalcMassProperties(shape_cast<ShapeSphere>(shape), density);
		case EShape::Box:      return CalcMassProperties(shape_cast<ShapeBox>(shape), density);
		case EShape::Line:     return CalcMassProperties(shape_cast<ShapeLine>(shape), density);
		case EShape::Triangle: return CalcMassProperties(shape_cast<ShapeTriangle>(shape), density);
		case EShape::Polytope: return CalcMassProperties(shape_cast<ShapePolytope>(shape), density);
		}
	}
}
