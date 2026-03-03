//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/shape/mass.h"

namespace pr::physics
{
	// Inertia Matrix
	// The physical meaning of an inertia is the distribution of a rigid body's 
	// mass about a particular point. If that point is the centre of mass of the 
	// body, then the measured inertia is called the "central inertia" of that body. 
	// To write down the inertia, we need to calculate the six scalars of the inertia 
	// matrix, which is a symmetric 3x3 matrix. These scalars must be expressed in 
	// an arbitrary but specified coordinate system. So an Inertia is meaningful only 
	// in conjunction with a particular set of axes, fixed to the body, whose origin 
	// is the point about which the inertia is being measured, and in whose 
	// coordinate system this measurement is being expressed. Note that changing the 
	// reference point results in a new physical quantity, but changing the reference 
	// axes only affects the measured numbers of that quantity. For any reference 
	// point, there is a unique set of reference axes in which the inertia matrix is
	// diagonal; those are called the "principal axes" of the body at that point, and 
	// the resulting diagonal elements are the "principal moments of inertia". When 
	// we speak of an inertia being "in" a frame, we mean the physical quantity 
	// measured about the frame's origin and then expressed in the frame's axes.
	//
	// Changing the coordinate system of an Inertia matrix does not entail a change of
	// physical meaning in the way that shifting it to a different point does.
	// To change coordinates use:
	//    Ib = b2a * Ia * a2b
	//
	// An Inertia is a symmetric matrix and is positive definite for non-singular bodies
	// (that is, a body composed of at least three non-collinear point masses).
	//
	// Note: inertia scales linearly with mass. This means inertia can be stored for a
	// unit mass (=1kg) and scaled when needed.

	// I¯ = I alt+0175

	// Inertia
	// See: RBDA 2.62
	// Inertia as a spatial matrix is a symmetric 6x6 matrix arranged as 2x2 blocks of 3x3 matrices.
	// This type represents the spatial inertia for a simple rigid body (i.e. not articulated) in
	// compact form. In spatial matrix form, the matrix would be:
	//     Io = [Ic + cxTcx , cxT] = [Ic - cxcx , -cx]
	//          [cx         ,   1] = [cx        ,   1]
	//   where:
	//     'Io' is the unit inertia measured about some arbitrary point 'o',
	//     'Ic' is the unit inertia measured about the centre of mass (at 'c'),
	//     'c' is the vector from 'o' back to 'c'
	//     'cx' is the cross product matrix of the vector 'c'
	//     'cxT' is the transpose of 'cx' which is equal to '-cx'.
	//
	// Notes:
	//  - Mass is included in 'Inertia' so that they can be combined with other inertias.
	//  - The inertia matrix is symmetric, so we don't need to store the full matrix.
	//  - The inverse of a symmetric positive definite matrix is also symmetric positive
	//    definite so the inverse of 'Inertia' can be stored the same way.
	//  - 'CoM()' is a vector from the origin of the space that the inertia is in to the
	//    centre of mass. This is really only used with spatial vectors and should be zero
	//    for normal inertia use.
	//  - Using float_inf for infinite mass objects doesn't work well because inf * 0 == NaN.
	//    Instead, use float_max in-place of infinite.
	//  - Infinite inertia matrices are an identity matrix but with 'mass' as float_max.
	//    That way, Invert and other functions don't need to handle special cases.

	// Use the sqrt of float_max as the threshold for infinite mass so that 
	// 'InfiniteMass * InfiniteMass' does not overflow a float. If mass becomes
	// 'inf' then multiplying by 0 creates NaNs.
	constexpr float InfiniteMass = 1.84467435229094026671e19f; // = sqrt(maths::float_max);
	constexpr float ZeroMass = 1.0f / InfiniteMass;

	// Direction for translating an inertia matrix
	enum class ETranslateInertia
	{
		TowardCoM,   // The pointy end of 'offset' is the CoM
		AwayFromCoM, // The base of 'offset' is the CoM
	};

	#pragma region Forwards
	struct Inertia;
	struct InertiaInv;
	Inertia Join(Inertia const& lhs, Inertia const& rhs);
	Inertia Split(Inertia const& lhs, Inertia const& rhs);
	InertiaInv Join(InertiaInv const& lhs, InertiaInv const& rhs);
	InertiaInv Split(InertiaInv const& lhs, InertiaInv const& rhs);
	InertiaInv Invert(Inertia const& inertia);
	Inertia Invert(InertiaInv const& inertia_inv);
	Inertia Rotate(Inertia const& inertia, m3x4 const& a2b);
	InertiaInv Rotate(InertiaInv const& inertia_inv, m3x4 const& a2b);
	Inertia Translate(Inertia const& inertia0, v4 offset, ETranslateInertia direction);
	InertiaInv Translate(InertiaInv const& inertia0_inv, v4 offset, ETranslateInertia direction);
	Inertia Transform(Inertia const& inertia0, m4x4 const& a2b, ETranslateInertia direction);
	InertiaInv Transform(InertiaInv const& inertia0_inv, m4x4 const& a2b, ETranslateInertia direction);
	#pragma endregion

	struct Inertia
	{
		// Notes:
		//  - The 'CoM' is not built into the inertia, it can be freely set to whatever you want.
		//    It's here as a convenience for calculating the inertia, parallel axis translated.
		//    Think of 'CoM' as a vector from your common point (typically the model origin) to
		//    the location of the centre of mass.

		v4 m_diagonal;     // The Ixx, Iyy, Izz terms of the unit inertia at the CoM, Ic.
		v4 m_products;     // The Ixy, Ixz, Iyz terms of the unit inertia at the CoM, Ic.
		v4 m_com_and_mass; // Offset from the origin to the centre of mass, and the mass.

		Inertia();
		Inertia(m3x4 const& unit_inertia, float mass, v4 com = v4{});
		Inertia(v4 diagonal, v4 products, float mass, v4 com = v4{});
		Inertia(float diagonal, float mass, v4 com = v4{});
		Inertia(Inertia const& rhs, v4 com);
		explicit Inertia(Mat6x8<float, Motion, Force> const& inertia, float mass = -1);
		explicit Inertia(MassProperties const& mp);

		// The mass to scale the inertia by
		float Mass() const;
		void Mass(float mass);

		// The inverse mass
		float InvMass() const;
		void InvMass(float invmass);

		// Offset from the origin of the space this inertia is in to the centre of mass.
		// Note: this is *NOT* equivalent to translating the inertia.
		v4 CoM() const;
		void CoM(v4 com);

		// The mass weighted distance from the centre of mass
		v4 MassMoment() const;

		// Return the centre of mass inertia (mass scaled by default, excludes 'com')
		m3x4 Ic3x3(float mass = -1) const;

		// The 3x3 inertia matrix (mass scaled by default, includes 'com')
		m3x4 To3x3(float mass = -1) const;

		// The 6x6 inertia matrix (mass scaled by default)
		Mat6x8f<Motion,Force> To6x6(float mass = -1) const;

		// Sanity check
		bool Check() const;
		static bool Check(m3x4 const& inertia);
		static bool Check(Mat6x8<float, Motion, Force> const& inertia);

		// An immovable object
		static Inertia Infinite();

		// Create an inertia matrix for a point at 'offset'
		static Inertia Point(float mass, v4 offset = v4{});

		// Create an inertia matrix for a sphere at 'offset'
		static Inertia Sphere(float radius, float mass, v4 offset = v4{});

		// Create an inertia matrix for a box at 'offset'
		static Inertia Box(v4 radius, float mass, v4 offset = v4{});

		#pragma region Operators
		// Note: there is no operator + because its definition is ambiguous
		//  Ia + Ib can either mean:
		//      Ia.To3x3() + Ib.To3x3() or Ia.To6x6() + Ib.To6x6() 
		//  or weld two rigid bodies together:
		//      (ma*Ia + mb*Ib)/(mamb) 

		friend bool operator == (Inertia const& lhs, Inertia const& rhs);
		friend bool operator != (Inertia const& lhs, Inertia const& rhs);

		// Multiply a vector by 'inertia'.
		friend v4 operator * (Inertia const& inertia, v4 const& v);

		// Multiply a spatial motion vector by 'inertia'.
		friend v8force operator * (Inertia const& inertia, v8motion const& motion);
		#pragma endregion
	};

	// Inverse Inertia.
	// See: RBDA 2.73
	// The format of the inverse inertia expressed at the centre of mass is:
	//   InvMass * [Ic¯ 0]
	//             [0   1]
	//  where:
	//    'Ic¯' is the inverse of 'Ic', the inertia expressed at the centre of mass,
	// The form of the inverse inertia expressed at an arbitrary point is:
	//  Io¯ = InvMass * [Ic¯   ,       Ic¯cxT] = InvMass * [Ic¯   ,      -Ic¯cx]
	//                  [cxIc¯ , 1 + cxIc¯cxT]             [cxIc¯ , 1 - cxIc¯cx]
	struct InertiaInv
	{
		v4 m_diagonal;        // The Ixx, Iyy, Izz terms of the unit inverse inertia
		v4 m_products;        // The Ixy, Ixz, Iyz terms of the unit inverse inertia
		v4 m_com_and_invmass; // Offset from the origin to the centre of mass, and the inverse mass.

		InertiaInv();
		InertiaInv(m3x4 const& unit_inertia_inv, float invmass, v4 com = v4{});
		InertiaInv(v4 diagonal, v4 products, float invmass, v4 com = v4{});
		InertiaInv(InertiaInv const& rhs, v4 com);
		InertiaInv(Mat6x8<float, Force, Motion> const& inertia_inv, float invmass = -1);

		// The mass to scale the inertia by
		float Mass() const;
		void Mass(float mass);

		// The inverse mass
		float InvMass() const;
		void InvMass(float invmass);

		// Offset to the location to use the inverse inertia
		v4 CoM() const;
		void CoM(v4 com);

		// The centre of mass inverse inertia (mass scaled by default, excludes 'com')
		m3x4 Ic3x3(float inv_mass = -1) const;

		// The mass scaled inverse inertia matrix
		m3x4 To3x3(float inv_mass = -1) const;

		// Return the inverse inertia matrix as a full spatial matrix
		Mat6x8f<Force,Motion> To6x6(float inv_mass = -1) const;

		// Sanity check
		bool Check() const;
		static bool Check(m3x4 const& inertia_inv);
		static bool Check(Mat6x8<float, Force, Motion> const& inertia_inv);

		// An immovable object
		static InertiaInv Zero();

		#pragma region Operators
		// Note: there is no operator + because its definition is ambiguous
		//  Ia + Ib can either mean:
		//      Ia.To3x3() + Ib.To3x3() or Ia.To6x6() + Ib.To6x6() 
		//  or weld two rigid bodies together:
		//      (ma*Ia + mb*Ib)/(mamb) 

		friend bool operator == (InertiaInv const& lhs, InertiaInv const& rhs);
		friend bool operator != (InertiaInv const& lhs, InertiaInv const& rhs);

		// Multiply a vector by 'inertia_inv'.
		friend v4 operator * (InertiaInv const& inertia_inv, v4 const& h);

		// Multiply a spatial force vector by 'inertia_inv' (i.e. F/M = a)
		friend v8motion operator * (InertiaInv const& inertia_inv, v8force const& force);
		#pragma endregion
	};
}
namespace pr
{
	bool FEql(physics::Inertia const& lhs, physics::Inertia const& rhs);
	bool FEql(physics::InertiaInv const& lhs, physics::InertiaInv const& rhs);
	bool FEqlRelative(physics::Inertia const& lhs, physics::Inertia const& rhs, float tol);
	bool FEqlRelative(physics::InertiaInv const& lhs, physics::InertiaInv const& rhs, float tol);
}
