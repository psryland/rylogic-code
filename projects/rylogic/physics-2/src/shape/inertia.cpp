//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics-2/shape/inertia.h"

namespace pr::physics
{
	// Inertia constructors *****************************************

	Inertia::Inertia()
		:m_diagonal(1, 1, 1, 0)
		,m_products(0, 0, 0, 0)
		,m_com_and_mass(0, 0, 0, InfiniteMass)
	{}
	Inertia::Inertia(m3x4 const& unit_inertia, float mass, v4 com)
		:m_diagonal(unit_inertia.x.x, unit_inertia.y.y, unit_inertia.z.z, 0)
		,m_products(unit_inertia.x.y, unit_inertia.x.z, unit_inertia.y.z, 0)
		,m_com_and_mass(com.xyz, mass)
	{
		assert(Check());
	}
	Inertia::Inertia(v4 diagonal, v4 products, float mass, v4 com)
		:m_diagonal(diagonal)
		,m_products(products)
		,m_com_and_mass(com.xyz, mass)
	{
		assert(Check());
	}
	Inertia::Inertia(float diagonal, float mass, v4 com)
		:m_diagonal(diagonal, diagonal, diagonal, 0)
		,m_products()
		,m_com_and_mass(com.xyz, mass)
	{
		assert(Check());
	}
	Inertia::Inertia(Inertia const& rhs, v4 com)
		:m_diagonal(rhs.m_diagonal)
		,m_products(rhs.m_products)
		,m_com_and_mass(com.xyz, rhs.Mass())
	{
		assert(Check());
	}
	Inertia::Inertia(Mat6x8<float, Motion, Force> const& inertia, float mass)
	{
		// If 'mass' is given, 'inertia' is assumed to be a unit inertia
		assert(Inertia::Check(inertia));

		auto m = mass >= 0 ? mass : Trace(inertia.m11) / 3.0f;
		auto cx = (1.0f / m) * inertia.m01;
		auto Ic = (1.0f / m) * inertia.m00 + cx * cx;
		*this = Inertia{Ic, m, v4{cx.y.z, -cx.x.z, cx.x.y, 0}};
	}
	Inertia::Inertia(MassProperties const& mp)
		:Inertia(mp.m_os_unit_inertia, mp.m_mass)
	{}

	// Inertia member functions *************************************

	float Inertia::Mass() const
	{
		return
			m_com_and_mass.w <  ZeroMass ? 0.0f :
			m_com_and_mass.w >= InfiniteMass ? InfiniteMass :
			m_com_and_mass.w;
	}
	void Inertia::Mass(float mass)
	{
		assert("Mass must be positive" && mass >= 0);
		assert(!isnan(mass));
		m_com_and_mass.w =
			mass <  ZeroMass ? 0.0f :
			mass >= InfiniteMass ? InfiniteMass :
			mass;
	}
	float Inertia::InvMass() const
	{
		auto mass = Mass();
		return
			mass <  ZeroMass ? InfiniteMass :
			mass >= InfiniteMass ? 0.0f :
			1.0f / mass;
	}
	void Inertia::InvMass(float invmass)
	{
		assert("Mass must be positive" && invmass >= 0);
		assert(!isnan(invmass));
		m_com_and_mass.w =
			invmass <  ZeroMass ? InfiniteMass :
			invmass >= InfiniteMass ? 0.0f :
			1.0f / invmass;
	}
	v4 Inertia::CoM() const
	{
		return m_com_and_mass.w0();
	}
	void Inertia::CoM(v4 com)
	{
		m_com_and_mass.xyz = com.xyz;
	}
	v4 Inertia::MassMoment() const
	{
		return -Mass() * CoM();
	}
	m3x4 Inertia::Ic3x3(float mass) const
	{
		mass = mass >= 0 ? mass : Mass();
		if (mass < ZeroMass || mass >= InfiniteMass)
			return m3x4::Identity();

		auto dia = mass * m_diagonal;
		auto off = mass * m_products;
		auto Ic = m3x4{
			v4{dia.x, off.x, off.y, 0},
			v4{off.x, dia.y, off.z, 0},
			v4{off.y, off.z, dia.z, 0}};
		return Ic;
	}
	m3x4 Inertia::To3x3(float mass) const
	{
		mass = mass >= 0 ? mass : Mass();
		if (mass < ZeroMass || mass >= InfiniteMass)
			return m3x4::Identity();

		auto Ic = Ic3x3(mass);
		if (CoM() == v4{})
			return Ic;

		auto cx = CPM<m3x4>(CoM());
		auto Io = Ic - mass * cx * cx;
		return Io;
	}
	Mat6x8<float, Motion, Force> Inertia::To6x6(float mass) const
	{
		mass = mass >= 0 ? mass : Mass();
		if (mass < ZeroMass || mass >= InfiniteMass)
			return Mat6x8<float, Motion, Force>{m6x8::Identity()};

		auto Ic = Ic3x3(mass);
		auto cx = CPM<m3x4>(CoM());

		// The m00 block is Ic - mass*cx*cx which is algebraically symmetric (cx*cx is symmetric
		// because (cxcx)^T = cx^T*cx^T = (-cx)(-cx) = cxcx). Float arithmetic in the triple
		// product introduces tiny asymmetry, so explicitly symmetrize.
		auto m00 = Ic - mass * cx * cx;
		m00.x.y = m00.y.x = 0.5f * (m00.x.y + m00.y.x);
		m00.x.z = m00.z.x = 0.5f * (m00.x.z + m00.z.x);
		m00.y.z = m00.z.y = 0.5f * (m00.y.z + m00.z.y);

		auto Io = Mat6x8<float, Motion, Force>{m00, mass * cx, -mass * cx, mass * m3x4::Identity()};
		return Io;
	}
	bool Inertia::Check() const
	{
		return CoM() == v4{} ? Inertia::Check(To3x3()) : Inertia::Check(To6x6());
	}
	bool Inertia::Check(m3x4 const& inertia)
	{
		// Check for any value == NaN
		if (IsNaN(inertia))
			return assert(false), false;

		// Check symmetric
		if (!IsSymmetric(inertia))
			return assert(false), false;

		auto dia = v4{inertia.x.x, inertia.y.y, inertia.z.z, 0};
		auto off = v4{inertia.x.y, inertia.x.z, inertia.y.z, 0};

		// Diagonals of an Inertia matrix must be non-negative
		if (dia.x < 0 || dia.y < 0 || dia.z < 0)
			return assert(false), false;

		// Relative tolerance for floating-point comparisons. Flat objects (e.g. triangles,
		// thin plates) have Izz = Ixx + Iyy exactly, but float arithmetic and inversion
		// roundtrips can push this past exact equality. Use a small relative tolerance
		// based on the trace to avoid false asserts on physically valid inertia tensors.
		auto tol = (dia.x + dia.y + dia.z) * math::tiny<float>;

		// Diagonals of an Inertia matrix must satisfy the triangle inequality: a + b >= c
		if ((dia.x + dia.y + tol) < dia.z ||
			(dia.y + dia.z + tol) < dia.x ||
			(dia.z + dia.x + tol) < dia.y)
			return assert(false), false;

		// The magnitude of a product of inertia was too large to be physical.
		if ((dia.x + tol) < Abs(2 * off.z) ||
			(dia.y + tol) < Abs(2 * off.y) ||
			(dia.z + tol) < Abs(2 * off.x))
			return assert(false), false;

		return true;
	}
	bool Inertia::Check(Mat6x8<float, Motion, Force> const& inertia)
	{
		// Check for any value == NaN
		if (IsNaN(inertia))
			return assert(false), false;

		// Check symmetric
		if (!IsSymmetric(inertia.m00) ||
			!IsSymmetric(inertia.m11) ||
			!IsAntiSymmetric(inertia.m01) ||
			!IsAntiSymmetric(inertia.m10) ||
			!FEql(inertia.m01 + inertia.m10, m3x4{}))
			return assert(false), false;

		// Check 'mass * 1'
		auto m = inertia.m11.x.x;
		if (!FEql(inertia.m11.y.y - m, 0.f) ||
			!FEql(inertia.m11.z.z - m, 0.f))
			return assert(false), false;

		// Check 'mass * cx'
		auto mcx = inertia.m01;
		if (!FEql(Trace(mcx), 0.f) ||
			!IsAntiSymmetric(mcx))
			return assert(false), false;

		// Check 'mass * cxT'
		auto mcxT = inertia.m10;
		if (!FEql(Trace(mcxT), 0.f) ||
			!IsAntiSymmetric(mcxT))
			return assert(false), false;

		// Check 'Ic - mcxcx'
		if (!Check(inertia.m00))
			return assert(false), false;

		return true;
	}

	// Inertia static factories ************************************

	Inertia Inertia::Infinite()
	{
		return Inertia{v4{1, 1, 1, 0}, v4{0, 0, 0, 0}, InfiniteMass};
	}
	Inertia Inertia::Point(float mass, v4 offset)
	{
		auto ib = Inertia{1.0f, mass};
		ib = Translate(ib, offset, ETranslateInertia::AwayFromCoM);
		return ib;
	}
	Inertia Inertia::Sphere(float radius, float mass, v4 offset)
	{
		auto ib = Inertia{(2.0f / 5.0f) * Sqr(radius), mass};
		ib = Translate(ib, offset, ETranslateInertia::AwayFromCoM);
		return ib;
	}
	Inertia Inertia::Box(v4 radius, float mass, v4 offset)
	{
		auto xx = (1.0f / 3.0f) * (Sqr(radius.y) + Sqr(radius.z));
		auto yy = (1.0f / 3.0f) * (Sqr(radius.z) + Sqr(radius.x));
		auto zz = (1.0f / 3.0f) * (Sqr(radius.x) + Sqr(radius.y));
		auto ib = Inertia{v4{xx, yy, zz, 0}, v4{}, mass};
		ib = Translate(ib, offset, ETranslateInertia::AwayFromCoM);
		return ib;
	}
	Inertia Inertia::Line(float half_length, float mass, v4 offset)
	{
		// Thin rod along the Z-axis, centred at the origin.
		// Ixx = Iyy = (1/3)*h², Izz = 0, where h is the half-length.
		// (Derived from (1/12)*L² for full length L = 2h.)
		auto hh = Sqr(half_length);
		auto ii = (1.0f / 3.0f) * hh;
		auto ib = Inertia{v4{ii, ii, 0, 0}, v4{}, mass};
		ib = Translate(ib, offset, ETranslateInertia::AwayFromCoM);
		return ib;
	}
	Inertia Inertia::Triangle(v4 a, v4 b, v4 c, float mass)
	{
		// Thin triangular lamina with uniform surface density.
		// CoM is at the centroid; unit inertia is computed about the CoM.
		auto com = (a + b + c) / 3.0f;
		com.w = 0;
		auto q0 = a - com;
		auto q1 = b - com;
		auto q2 = c - com;

		// Unit inertia about CoM for a triangular lamina:
		//   diagonal_i = (1/12) * Σ_k (qk_j² + qk_k²)
		//   product_ij = -(1/12) * Σ_k (qk_i * qk_j)
		// Derived from the barycentric integral ∫∫ λi·λj dA = A/12 (i≠j), A/6 (i=j),
		// with the simplification that Σqi = 0.
		constexpr auto k = 1.0f / 12.0f;
		auto xx = k * (Sqr(q0.y) + Sqr(q0.z) + Sqr(q1.y) + Sqr(q1.z) + Sqr(q2.y) + Sqr(q2.z));
		auto yy = k * (Sqr(q0.x) + Sqr(q0.z) + Sqr(q1.x) + Sqr(q1.z) + Sqr(q2.x) + Sqr(q2.z));
		auto zz = k * (Sqr(q0.x) + Sqr(q0.y) + Sqr(q1.x) + Sqr(q1.y) + Sqr(q2.x) + Sqr(q2.y));
		auto xy = -k * (q0.x * q0.y + q1.x * q1.y + q2.x * q2.y);
		auto xz = -k * (q0.x * q0.z + q1.x * q1.z + q2.x * q2.z);
		auto yz = -k * (q0.y * q0.z + q1.y * q1.z + q2.y * q2.z);
		return Inertia{v4{xx, yy, zz, 0}, v4{xy, xz, yz, 0}, mass, com};
	}

	// Inertia operators ********************************************

	bool operator == (Inertia const& lhs, Inertia const& rhs)
	{
		return
			lhs.m_diagonal == rhs.m_diagonal &&
			lhs.m_products == rhs.m_products &&
			lhs.m_com_and_mass == rhs.m_com_and_mass;
	}
	bool operator != (Inertia const& lhs, Inertia const& rhs)
	{
		return !(lhs == rhs);
	}
	v4 operator * (Inertia const& inertia, v4 v)
	{
		if (inertia.CoM() == v4{})
			return inertia.To3x3() * v;
		else
			return Translate(inertia, -inertia.CoM(), ETranslateInertia::AwayFromCoM).To3x3() * v;
	}
	v8force operator * (Inertia const& inertia, v8motion const& motion)
	{
		// Typically 'motion' is a velocity or an acceleration.
		// e.g.
		//   I = spatial inertia
		//   v = spatial velocity
		//   h = spatial momentum = I * v
		//   T = kinetic energy = 0.5 * Dot(v, I*v)
		//
		//  h = mass * [Ic - cxcx , cx] * [ang]
		//             [-cx       ,  1]   [lin]

		// Special case when the inertia is in CoM frame.
		if (inertia.CoM() == v4{})
			return v8force{inertia.To3x3() * motion.ang, inertia.Mass() * motion.lin};
		else
			return inertia.To6x6() * motion;
	}

	// InertiaInv constructors **************************************

	InertiaInv::InertiaInv()
		:m_diagonal(1, 1, 1, 0)
		,m_products(0, 0, 0, 0)
		,m_com_and_invmass(0, 0, 0, 0)
	{}
	InertiaInv::InertiaInv(m3x4 const& unit_inertia_inv, float invmass, v4 com)
		:m_diagonal(unit_inertia_inv.x.x, unit_inertia_inv.y.y, unit_inertia_inv.z.z, 0)
		,m_products(unit_inertia_inv.x.y, unit_inertia_inv.x.z, unit_inertia_inv.y.z, 0)
		,m_com_and_invmass(com.xyz, invmass)
	{
		assert(Check());
	}
	InertiaInv::InertiaInv(v4 diagonal, v4 products, float invmass, v4 com)
		:m_diagonal(diagonal)
		,m_products(products)
		,m_com_and_invmass(com.xyz, invmass)
	{
		assert(Check());
	}
	InertiaInv::InertiaInv(InertiaInv const& rhs, v4 com)
		:m_diagonal(rhs.m_diagonal)
		,m_products(rhs.m_products)
		,m_com_and_invmass(com.xyz, rhs.InvMass())
	{
		assert(Check());
	}
	InertiaInv::InertiaInv(Mat6x8<float, Force, Motion> const& inertia_inv, float invmass)
	{
		assert(InertiaInv::Check(inertia_inv));

		auto Ic_inv = inertia_inv.m00;
		auto cx = inertia_inv.m10 * Invert(Ic_inv);
		auto im = invmass >= 0 ? invmass : Trace(inertia_inv.m11 + cx * Ic_inv * cx) / 3.0f;
		*this = InertiaInv{(1 / im) * Ic_inv, im, v4{cx.y.z, -cx.x.z, cx.x.y, 0}};
	}

	// InertiaInv member functions **********************************

	float InertiaInv::Mass() const
	{
		auto im = InvMass();
		return
			im <  ZeroMass ? InfiniteMass :
			im >= InfiniteMass ? 0.0f :
			1.0f / im;
	}
	void InertiaInv::Mass(float mass)
	{
		assert("Mass must be positive" && mass >= 0);
		assert(!isnan(mass));
		m_com_and_invmass.w =
			mass <  ZeroMass ? InfiniteMass :
			mass >= InfiniteMass ? 0.0f :
			1.0f / mass;
	}
	float InertiaInv::InvMass() const
	{
		auto invmass = m_com_and_invmass.w;
		return
			invmass <  ZeroMass ? 0.0f :
			invmass >= InfiniteMass ? InfiniteMass :
			invmass;
	}
	void InertiaInv::InvMass(float invmass)
	{
		assert("Mass must be positive" && invmass >= 0);
		assert(!isnan(invmass));
		m_com_and_invmass.w =
			invmass <  ZeroMass ? 0.0f :
			invmass >= InfiniteMass ? InfiniteMass :
			invmass;
	}
	v4 InertiaInv::CoM() const
	{
		return m_com_and_invmass.w0();
	}
	void InertiaInv::CoM(v4 com)
	{
		m_com_and_invmass.xyz = com.xyz;
	}
	m3x4 InertiaInv::Ic3x3(float inv_mass) const
	{
		inv_mass = inv_mass >= 0 ? inv_mass : InvMass();

		// For immovable bodies (inv_mass ≈ 0), the inverse inertia is zero.
		// For massless bodies (inv_mass → ∞), treat as degenerate.
		if (inv_mass < ZeroMass)
			return m3x4{};
		if (inv_mass >= InfiniteMass)
			return m3x4::Identity();

		auto dia = inv_mass * m_diagonal;
		auto off = inv_mass * m_products;
		auto Ic_inv = m3x4{
			v4{dia.x, off.x, off.y, 0},
			v4{off.x, dia.y, off.z, 0},
			v4{off.y, off.z, dia.z, 0}};
		return Ic_inv;
	}
	m3x4 InertiaInv::To3x3(float inv_mass) const
	{
		inv_mass = inv_mass >= 0 ? inv_mass : InvMass();

		// For immovable bodies (inv_mass ≈ 0), the inverse inertia is zero.
		if (inv_mass < ZeroMass)
			return m3x4{};
		if (inv_mass >= InfiniteMass)
			return m3x4::Identity();

		auto Ic_inv = Ic3x3(inv_mass);
		if (CoM() == v4{})
			return Ic_inv;

		//' Io¯ = (Ic - mcxcx)¯                                  '
		//' Identity: (A + B)¯ = A¯ - (1 + A¯B)¯A¯BA¯            '
		//'   Let A = Ic, B = -mcxcx                             '
		//'  Then:                                               '
		//' Io¯ = Ic¯ + m(1 - mIc¯cxcx)¯Ic¯cxcxIc¯               '
		//'     = Ic¯ + (1/m - Ic¯cxcx)¯Ic¯cxcxIc¯               '

		// This is cheaper
		auto cx = CPM<m3x4>(CoM());
		auto Io = Invert(Ic_inv) - (1.0f / inv_mass) * cx * cx;
		auto Io_inv = Invert(Io);
		return Io_inv;
	}
	Mat6x8<float, Force, Motion> InertiaInv::To6x6(float inv_mass) const
	{
		inv_mass = inv_mass >= 0 ? inv_mass : InvMass();
		if (inv_mass < ZeroMass || inv_mass >= InfiniteMass)
			return Mat6x8<float, Force, Motion>{m6x8::Identity()};

		auto Ic_inv = Ic3x3(inv_mass);
		auto cx = CPM<m3x4>(CoM());

		// The m11 block is cx*Ic_inv*cx which is algebraically symmetric (since Ic_inv is
		// symmetric and cx is antisymmetric: (cSc)^T = c^T S c^T = (-c)S(-c) = cSc). Float
		// arithmetic in the triple product introduces tiny asymmetry, so explicitly symmetrize.
		auto m11 = inv_mass * m3x4::Identity() - cx * Ic_inv * cx;
		m11.x.y = m11.y.x = 0.5f * (m11.x.y + m11.y.x);
		m11.x.z = m11.z.x = 0.5f * (m11.x.z + m11.z.x);
		m11.y.z = m11.z.y = 0.5f * (m11.y.z + m11.z.y);

		auto Io_inv = Mat6x8<float, Force, Motion>{Ic_inv, -Ic_inv * cx, cx * Ic_inv, m11};
		return Io_inv;
	}
	bool InertiaInv::Check() const
	{
		return CoM() == v4{} ? InertiaInv::Check(To3x3()) : InertiaInv::Check(To6x6());
	}
	bool InertiaInv::Check(m3x4 const& inertia_inv)
	{
		// Check for any value == NaN
		if (IsNaN(inertia_inv))
			return assert(false), false;

		// Check symmetric
		if (!IsSymmetric(inertia_inv))
			return assert(false), false;

		auto dia = v4{inertia_inv.x.x, inertia_inv.y.y, inertia_inv.z.z, 0};
		auto off = v4{inertia_inv.x.y, inertia_inv.x.z, inertia_inv.y.z, 0};

		// Diagonals of an inverse inertia matrix must be non-negative
		if (dia.x < 0 || dia.y < 0 || dia.z < 0)
			return assert(false), false;

		// Diagonals of an inverse inertia matrix must satisfy the triangle inequality: a + b >= c
		// Might need to relax 'tol' due to distorted rotation matrices: using: 'Max(Sum(d), 1) * tiny_sqrt'
		//if (!is_inverse && (
		//	(dia.x + dia.y) < dia.z ||
		//	(dia.y + dia.z) < dia.x ||
		//	(dia.z + dia.x) < dia.y))
		//	return assert(false),false;

		// The magnitude of a product of inertia was too large to be physical.
		//if (!is_inverse && (
		//	dia.x < Abs(2 * off.z) || 
		//	dia.y < Abs(2 * off.y) ||
		//	dia.z < Abs(2 * off.x)))
		//	return assert(false),false;

		return true;
	}
	bool InertiaInv::Check(Mat6x8<float, Force, Motion> const& inertia_inv)
	{
		// Check for any value == NaN
		if (IsNaN(inertia_inv))
			return assert("InertiaInv 6x6: NaN" && false), false;

		// Check symmetric blocks: m00 = Ic¯ (symmetric), m11 (symmetric)
		if (!IsSymmetric(inertia_inv.m00) ||
			!IsSymmetric(inertia_inv.m11))
			return assert("InertiaInv 6x6: not symmetric" && false), false;

		// Check 'Ic¯' block (non-negative diags, etc.)
		auto Ic_inv = inertia_inv.m00;
		if (!Check(Ic_inv))
			return assert("InertiaInv 6x6: Ic_inv block" && false), false;

		// Check structural consistency of off-diagonal blocks.
		// The 6x6 InertiaInv has the form:
		//   | Ic¯          -Ic¯*cx        |
		//   | cx*Ic¯       1/m - cx*Ic¯*cx |
		// Since Ic¯ is symmetric and cx is anti-symmetric:
		//   transpose(m10) = transpose(cx*Ic¯) = Ic¯ᵀ*cxᵀ = Ic¯*(-cx) = -Ic¯*cx = m01
		// This relationship holds exactly (no matrix inversion needed) and catches
		// any structural inconsistency between the off-diagonal blocks.
		if (!FEql(inertia_inv.m01, Transpose(inertia_inv.m10)))
			return assert("InertiaInv 6x6: m01 != transpose(m10)" && false), false;

		return true;
	}
	InertiaInv InertiaInv::Zero()
	{
		return InertiaInv{v4{1, 1, 1, 0}, v4{0, 0, 0, 0}, 0};
	}

	// InertiaInv operators ******************************************

	bool operator == (InertiaInv const& lhs, InertiaInv const& rhs)
	{
		return
			lhs.m_diagonal == rhs.m_diagonal &&
			lhs.m_products == rhs.m_products &&
			lhs.m_com_and_invmass == rhs.m_com_and_invmass;
	}
	bool operator != (InertiaInv const& lhs, InertiaInv const& rhs)
	{
		return !(lhs == rhs);
	}
	v4 operator * (InertiaInv const& inertia_inv, v4 h)
	{
		if (inertia_inv.CoM() == v4{})
			return inertia_inv.To3x3() * h;
		else
			return Translate(inertia_inv, -inertia_inv.CoM(), ETranslateInertia::AwayFromCoM).To3x3() * h;
	}
	v8motion operator * (InertiaInv const& inertia_inv, v8force const& force)
	{
		// Special case when the inertia is in CoM frame.
		if (inertia_inv.CoM() == v4{})
			return v8motion{inertia_inv.To3x3() * force.ang, inertia_inv.InvMass() * force.lin};
		else
			return inertia_inv.To6x6() * force;
	}

	// Free functions ***********************************************

	// Add/Subtract two inertias. 'lhs' and 'rhs' must be in the same frame.
	Inertia Join(Inertia const& lhs, Inertia const& rhs)
	{
		// Todo: this is not the correct check, so long as the inertias are in the same frame
		// they can be added after parallel axis transformed to a common point.
		if (lhs.CoM() != rhs.CoM())
			throw std::runtime_error("Inertias must be in the same space");

		auto& Ia = lhs;
		auto& Ib = rhs;

		auto massA = Ia.Mass();
		auto massB = Ib.Mass();
		auto mass = massA + massB;
		auto com = lhs.CoM();

		// Once inertia's are in the same space they can just be added.
		// Since these are normalised inertias however we need to add proportionally.
		// i.e.
		//   U = I/m = unit inertia = inertia / mass
		//   I3 = I1 + I2, I1 = m1U1, I2 = m2U2
		//   I3 = m3U3 = m1U1 + m2U2
		//   U3 = (m1U1 + m2U2)/m3
		Inertia sum = {};
		if (mass < math::tiny<float>)
		{
			sum.m_diagonal = (Ia.m_diagonal + Ib.m_diagonal) / 2.0f;
			sum.m_products = (Ia.m_products + Ib.m_products) / 2.0f;
			sum.m_com_and_mass = v4{ com.xyz, mass };
		}
		else
		{
			sum.m_diagonal = (massA * Ia.m_diagonal + massB * Ib.m_diagonal) / mass;
			sum.m_products = (massA * Ia.m_products + massB * Ib.m_products) / mass;
			sum.m_com_and_mass = v4{ com.xyz, mass };
		}
		return sum;
	}
	Inertia Split(Inertia const& lhs, Inertia const& rhs)
	{
		if (lhs.CoM() != rhs.CoM())
			throw std::runtime_error("Inertias must be in the same space");

		auto& Ia = lhs;
		auto& Ib = rhs;

		auto massA = Ia.Mass();
		auto massB = Ib.Mass();
		auto mass = massA - massB;
		auto com = lhs.CoM();
		
		// The result must still have a positive mass
		if (mass <= 0)
			throw std::runtime_error("Inertia difference is undefined");
		
		Inertia sum = {};
		sum.m_diagonal = (massA*Ia.m_diagonal - massB*Ib.m_diagonal) / mass;
		sum.m_products = (massA*Ia.m_products - massB*Ib.m_products) / mass;
		sum.m_com_and_mass = v4{ com.xyz, mass };
		return sum;
	}

	// Add/Subtract inverse inertias. 'lhs' and 'rhs' must be in the same frame.
	InertiaInv Join(InertiaInv const& lhs, InertiaInv const& rhs)
	{
		if (lhs.CoM() != rhs.CoM())
			throw std::runtime_error("Inertias must be in the same space");

		auto& Ia_inv = lhs;
		auto& Ib_inv = rhs;

		auto massA = Ia_inv.Mass();
		auto massB = Ib_inv.Mass();
		auto mass = massA + massB;
		auto com = lhs.CoM();

		InertiaInv sum = {};
		sum.m_diagonal = (massA*Ia_inv.m_diagonal + massB*Ib_inv.m_diagonal) / mass;
		sum.m_products = (massA*Ia_inv.m_products + massB*Ib_inv.m_products) / mass;
		sum.m_com_and_invmass = v4{ com.xyz, 1 / mass };
		return sum;
	}
	InertiaInv Split(InertiaInv const& lhs, InertiaInv const& rhs)
	{
		if (lhs.CoM() != rhs.CoM())
			throw std::runtime_error("Inertias must be in the same space");

		auto& Ia_inv = lhs;
		auto& Ib_inv = rhs;

		auto massA = Ia_inv.Mass();
		auto massB = Ib_inv.Mass();
		auto mass = massA - massB;
		auto com = lhs.CoM();
		
		// The result must still have a positive mass
		if (mass <= 0)
			throw std::runtime_error("Inertia difference is undefined");
		
		InertiaInv sum = {};
		sum.m_diagonal = (massA*Ia_inv.m_diagonal - massB*Ib_inv.m_diagonal) / mass;
		sum.m_products = (massA*Ia_inv.m_products - massB*Ib_inv.m_products) / mass;
		sum.m_com_and_invmass = v4{com.xyz, 1/mass};
		return sum;
	}

	// Invert inertia
	InertiaInv Invert(Inertia const& inertia)
	{
		auto unit_inertia_inv = Invert(inertia.Ic3x3(1));
		return InertiaInv{unit_inertia_inv, inertia.InvMass(), inertia.CoM()};
	}
	Inertia Invert(InertiaInv const& inertia_inv)
	{
		auto unit_inertia = Invert(inertia_inv.Ic3x3(1));
		return Inertia{unit_inertia, inertia_inv.Mass(), inertia_inv.CoM()};
	}

	// Rotate an inertia in frame 'a' to frame 'b'
	Inertia Rotate(Inertia const& inertia, m3x4 const& a2b)
	{
		// Ib = a2b*Ia*b2a
		// Explicitly symmetrize because R*S*R^T introduces tiny float asymmetry.
		auto b2a = InvertOrthonormal(a2b);
		auto Ic = a2b * inertia.Ic3x3(1) * b2a;
		Ic.x.y = Ic.y.x = 0.5f * (Ic.x.y + Ic.y.x);
		Ic.x.z = Ic.z.x = 0.5f * (Ic.x.z + Ic.z.x);
		Ic.y.z = Ic.z.y = 0.5f * (Ic.y.z + Ic.z.y);
		auto com = a2b * inertia.CoM();
		return Inertia{Ic, inertia.Mass(), com};
	}
	InertiaInv Rotate(InertiaInv const& inertia_inv, m3x4 const& a2b)
	{
		// Ib¯ = (a2b*Ia*b2a)¯ = b2a¯*Ia¯*a2b¯ = a2b*Ia¯*b2a
		// Explicitly symmetrize the result because R*S*R^T introduces tiny float
		// asymmetry when R is not perfectly orthogonal (e.g. after Orthonorm).
		auto b2a = InvertOrthonormal(a2b);
		auto Ic_inv = a2b * inertia_inv.Ic3x3(1) * b2a;
		Ic_inv.x.y = Ic_inv.y.x = 0.5f * (Ic_inv.x.y + Ic_inv.y.x);
		Ic_inv.x.z = Ic_inv.z.x = 0.5f * (Ic_inv.x.z + Ic_inv.z.x);
		Ic_inv.y.z = Ic_inv.z.y = 0.5f * (Ic_inv.y.z + Ic_inv.z.y);
		auto com = a2b * inertia_inv.CoM();
		return InertiaInv{Ic_inv, inertia_inv.InvMass(), com};
	}

	// Returns an inertia translated using the parallel axis theorem.
	// 'offset' is the vector from (or toward) the centre of mass (determined by 'direction').
	// 'offset' must be in the current frame.
	Inertia Translate(Inertia const& inertia0, v4 offset, ETranslateInertia direction)
	{
		//' Io = Ic - cxcx (for unit inertia away from CoM) '
		//' Ic = Io + cxcx (for unit inertia toward CoM)    '
		auto inertia1 = inertia0;
		auto sign = (direction == ETranslateInertia::AwayFromCoM) ? +1.0f : -1.0f;

		// For the diagonal elements:
		//'  I = Io + md² (away from CoM), Io = I - md² (toward CoM) '
		//' 'd' is the perpendicular component of 'offset'
		inertia1.m_diagonal.x += sign * (Sqr(offset.y) + Sqr(offset.z));
		inertia1.m_diagonal.y += sign * (Sqr(offset.z) + Sqr(offset.x));
		inertia1.m_diagonal.z += sign * (Sqr(offset.x) + Sqr(offset.y));

		// For off-diagonal elements: the products of inertia have the OPPOSITE sign
		// from the diagonal terms in the parallel axis theorem. The cxcx matrix has
		// negative diagonals -(cj²+ck²) but positive off-diagonals ci*cj, so when
		// applying Io = Ic - cxcx: diagonals ADD (d²) but off-diagonals SUBTRACT (di*dj).
		//'  Io_xy = Ic_xy - dxdy  (away from CoM) '
		//'  Ic_xy = Io_xy + dxdy  (toward CoM)    '
		inertia1.m_products.x -= sign * (offset.x * offset.y); // xy
		inertia1.m_products.y -= sign * (offset.x * offset.z); // xz
		inertia1.m_products.z -= sign * (offset.y * offset.z); // yz

		// 'com' is mainly used for spatial inertia when multiplying the inertia
		// at a point other than where the inertia was measured at. Translate()
		// moves the measure point, so if 'com' is non-zero, update it to reflect
		// the new offset.
		if (inertia0.CoM() != v4{})
			inertia1.m_com_and_mass.xyz -= sign * offset.xyz;

		return inertia1;
	}
	InertiaInv Translate(InertiaInv const& inertia0_inv, v4 offset, ETranslateInertia direction)
	{
		auto inertia0 = Invert(inertia0_inv);
		auto inertia1 = Translate(inertia0, offset, direction);
		auto inertia1_inv = Invert(inertia1);
		return inertia1_inv;
	}

	// Rotate, then translate an inertia
	Inertia Transform(Inertia const& inertia0, m4x4 const& a2b, ETranslateInertia direction)
	{
		auto inertia1 = inertia0;
		inertia1 = Rotate(inertia1, a2b.rot);
		inertia1 = Translate(inertia1, a2b.pos, direction);
		return inertia1;
	}
	InertiaInv Transform(InertiaInv const& inertia0_inv, m4x4 const& a2b, ETranslateInertia direction)
	{
		auto inertia1_inv = inertia0_inv;
		inertia1_inv = Rotate(inertia1_inv, a2b.rot);
		inertia1_inv = Translate(inertia1_inv, a2b.pos, direction);
		return inertia1_inv;
	}
}
namespace pr
{
	// Approximate equality
	bool FEql(physics::Inertia const& lhs, physics::Inertia const& rhs)
	{
		return
			FEql(lhs.m_diagonal, rhs.m_diagonal) &&
			FEql(lhs.m_products, rhs.m_products) &&
			FEql(lhs.m_com_and_mass, rhs.m_com_and_mass);
	}
	bool FEql(physics::InertiaInv const& lhs, physics::InertiaInv const& rhs)
	{
		return
			FEql(lhs.m_diagonal, rhs.m_diagonal) &&
			FEql(lhs.m_products, rhs.m_products) &&
			FEql(lhs.m_com_and_invmass, rhs.m_com_and_invmass);
	}
	bool FEqlRelative(physics::Inertia const& lhs, physics::Inertia const& rhs, float tol)
	{
		return
			FEqlRelative(lhs.m_diagonal, rhs.m_diagonal, tol) &&
			FEqlRelative(lhs.m_products, rhs.m_products, tol) &&
			FEqlRelative(lhs.m_com_and_mass, rhs.m_com_and_mass, tol);
	}
	bool FEqlRelative(physics::InertiaInv const& lhs, physics::InertiaInv const& rhs, float tol)
	{
		return
			FEqlRelative(lhs.m_diagonal, rhs.m_diagonal, tol) &&
			FEqlRelative(lhs.m_products, rhs.m_products, tol) &&
			FEqlRelative(lhs.m_com_and_invmass, rhs.m_com_and_invmass, tol);
	}
}