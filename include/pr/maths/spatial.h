//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
// Based on Featherstone Rigid Body Dynamics
//
// Spatial algebra uses dual vector spaces M6 and F6 (motion vectors, and force vectors)
// M6 is for velocities, accelerations, etc
// F6 is for forces, moments, momentum, etc.
// Some operators are only defined for (M6,F6), not (M6,M6) or (F6,F6).
//  E.g. Scalar product m.f = work, M6 X F6 => R
// 
// Spatial vectors
// Spatial vectors use 'Plucker coordinates' which are the components in the
// x,y,z directions, and the components of rotation about the x,y,z axes.
// Normally, a rigid body has a linear velocity, v, and an angular velocity, w.
// The spatial velocity is defined as [w,v] = [wx, wy, wz, vx, vy, vz] = [Plucker Coords]
// although, strictly, the order is not important (could be [vx, wx, vy, wy, vz, wz])
// Similarly, the spatial force vector is [T,F] where T = torque, F = linear force.

// Spatial matrices 
//  e.g. Spatial inertia matrix is a mapping from M6 to F6
//
// Spatial transforms are a special case of spatial matrices.
// Spatial transforms have this form:
//   [  R  0]
//   [-d^R R] (d^ represents the cross product matrix associated with the vector d)
// This means a spatial transform can be created from a normal affine transform.
//   m4x4 o2w = [o2w.rot                0      ]
//              [-CPM(o2w.pos)*o2w.rot  o2w.rot] (CPM = cross product matrix)
// If X is a matrix that transforms a to b for M6 vectors, and X* is a matrix that
// performs the same transform for F6 vectors, then X* == X^-T (invert then transpose).
// A spatial transform from A to B for motion vectors = bXa.
// A spatial transform from A to B for force vectors = bX*a.
//  bX*a == bXa^-T (invert then transpose)
// 
// 
#pragma once

#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector4.h"
#include "pr/maths/vector8.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/matrix4x4.h"
#include "pr/maths/matrix6x8.h"

namespace pr::maths::spatial
{
	// Spatial vector spaces
	struct Motion {};
	struct Force {};

	// Type traits
	template <typename T> struct is_motion :std::is_same<T, Motion> {};
	template <typename T> struct is_force  :std::is_same<T, Force> {};
	template <typename T> using enable_if_motion = typename std::enable_if<is_motion<T>::value>::type;
	template <typename T> using enable_if_force  = typename std::enable_if<is_force<T>::value>::type;

	// Spatial vector in the motion vector space.
	// Used for: velocity, acceleration, infinitesimal displacement, directions of motion freedom and constraint
	using v8motion = Vec8f<Motion>;

	// Spatial vector in the force vector space.
	// Used for: momentum, impulse, directions of force freedom and constraint
	using v8force = Vec8f<Force>;

	// General Spatial matrix for Motion and Force vector spaces
	// Notes:
	//  - Transforms for vectors belonging to dual spaces M and F are related like this:
	//     if  X  takes a vector m to m' in space M
	//     and X* takes a vector f to f' in space F
	//     then X* = transpose(inverse(X))
	using m6x8m = Mat6x8f<Motion, Motion>;
	using m6x8f = Mat6x8f<Force, Force>;
	using m6x8mf = Mat6x8f<Motion, Force>;
	using m6x8fm = Mat6x8f<Force, Motion>;

	#pragma region Operators

	// Rotate a spatial motion vector
	template <typename T>
	inline Vec8f<T> pr_vectorcall operator * (Mat3x4<float,Motion,T> const& a2b, Vec8f<Motion> const& vec)
	{
		// [ E    0] * [v.ang] = [E*v.ang             ]
		// [-E*rx E]   [v.lin]   [E*v.lin - E*rx*v.ang] where rx = (0,0,0)
		auto ang_b = m3x4{a2b} * vec.ang;
		auto lin_b = m3x4{a2b} * vec.lin;
		return Vec8f<T>{ang_b, lin_b};
	}
	inline Vec8f<Motion> pr_vectorcall operator * (m3x4 const& a2b, Vec8f<Motion> const& vec)
	{
		return (Mat3x4<float,Motion,Motion> const&)(a2b) * vec;
	}

	// Rotate a spatial force vector
	template <typename T>
	inline Vec8f<T> pr_vectorcall operator * (Mat3x4<float,Force,T> const& a2b, Vec8f<Force> const& vec)
	{
		// [E -E*rx] * [v.ang] = [E*v.ang - E*rx*v.lin]
		// [0     E]   [v.lin]   [E*v.lin             ] where rx = (0,0,0)
		auto lin_b = m3x4{a2b} * vec.lin;
		auto ang_b = m3x4{a2b} * vec.ang;
		return Vec8f<T>{ang_b, lin_b};
	}
	inline Vec8f<Force> pr_vectorcall operator * (m3x4 const& a2b, Vec8f<Force> const& vec)
	{
		return (Mat3x4<float,Force,Force> const&)(a2b) * vec;
	}


	// Transform a spatial motion vector by an affine transform
	template <typename T>
	inline Vec8f<T> pr_vectorcall operator * (Mat4x4<float,Motion,T> const& a2b, Vec8f<Motion> const& vec)
	{
		// [ E    0] * [v.ang] = [E*v.ang             ]
		// [-E*rx E]   [v.lin]   [E*v.lin - E*rx*v.ang]
		pr_assert("'lhs' is not an affine transform" && IsAffine(a2b));
		auto ang_b = m3x4{a2b.rot} * vec.ang;
		auto lin_b = m3x4{a2b.rot} * vec.lin + Cross(a2b.pos, ang_b);
		return Vec8f<T>{ang_b, lin_b};
	}
	inline Vec8f<Motion> pr_vectorcall operator * (m4x4 const& a2b, Vec8f<Motion> const& vec)
	{
		return (Mat4x4<float,Motion,Motion> const&)(a2b) * vec;
	}

	// Transform a spatial force vector by an affine transform
	template <typename T>
	inline Vec8f<T> pr_vectorcall operator * (Mat4x4<float,Force,T> const& a2b, Vec8f<Force> const& vec)
	{
		// [E -E*rx] * [v.ang] = [E*v.ang - E*rx*v.lin]
		// [0     E]   [v.lin]   [E*v.lin             ]
		pr_assert("'lhs' is not an affine transform" && IsAffine(a2b));
		auto lin_b = m3x4{a2b.rot} * vec.lin;
		auto ang_b = m3x4{a2b.rot} * vec.ang + Cross(a2b.pos, lin_b);
		return Vec8f<T>{ang_b, lin_b};
	}
	inline Vec8f<Force> pr_vectorcall operator * (m4x4 const& a2b, Vec8f<Force> const& vec)
	{
		return (Mat4x4<float,Force,Force> const&)(a2b) * vec;
	}

	// Spatial matrix * affine transform
	//template <typename A> inline Mat6x8f<A, Motion> pr_vectorcall operator * (m6x8<> const& lhs, m4x4 const&& rhs)
	//{
	//	// [ E    0] * [m00, m01] = [E*m00     + 0*m10,  E*m01    + 0*m11] = [E*m00           , E*m01           ]
	//	// [-E*rx E]   [m10, m11]   [-E*rx*m00 + E*m10, -E*rx*m01 + E*m11]   [E*(m10 - rx*m00), E*(m11 - rx*m01)]
	//	auto E = lhs.a2b.rot;
	//	auto rx = CPM(lhs.a2b.pos);
	//	return Mat6x8f<A, Motion>(
	//		E * rhs.m00, E * rhs.m01,
	//		E * (rhs.m10 - rx * rhs.m00), E * (rhs.m11 - rx * rhs.m01));
	//}
	//template <typename A> inline Mat6x8f<A, Motion> operator * (Transform<Motion> const& lhs, Mat6x8f<A, Motion> const& rhs)
	//{
	//	// [ E    0] * [m00, m01] = [E*m00     + 0*m10,  E*m01    + 0*m11] = [E*m00           , E*m01           ]
	//	// [-E*rx E]   [m10, m11]   [-E*rx*m00 + E*m10, -E*rx*m01 + E*m11]   [E*(m10 - rx*m00), E*(m11 - rx*m01)]
	//	auto E = lhs.a2b.rot;
	//	auto rx = CPM(lhs.a2b.pos);
	//	return Mat6x8f<A, Motion>(
	//		E * rhs.m00, E * rhs.m01,
	//		E * (rhs.m10 - rx * rhs.m00), E * (rhs.m11 - rx * rhs.m01));
	//}
	#pragma endregion

	#pragma region Functions

	// Spatial dot product
	// The dot product is only defined for Dot(v8motion,v8force) and Dot(v8force,v8motion).
	// e.g Dot(force, velocity) == power delivered
	inline float Dot(Vec8f<Motion> const& lhs, Vec8f<Force> const& rhs)
	{
		// v8motion and v8force are vectors in the dual spaces M and F
		// A property of dual spaces is dot(m,f) = transpose(m)*f
		return Dot3(lhs.ang, rhs.ang) + Dot3(lhs.lin, rhs.lin);
	}
	inline float Dot(Vec8f<Force> const& lhs, Vec8f<Motion> const& rhs)
	{
		return Dot(rhs, lhs);
	}

	// Spatial cross product.
	// There are two cross product operations, one for motion vectors (rx) and one for forces (rx*)
	template <typename T> inline Vec8f<Motion> Cross(Vec8f<T> const& lhs, Vec8f<Motion> const& rhs)
	{
		return Vec8f<Motion>(Cross(lhs.ang, rhs.ang), Cross(lhs.ang, rhs.lin) + Cross(lhs.lin, rhs.ang));
	}
	template <typename T> inline Vec8f<Force> Cross(Vec8f<T> const& lhs, Vec8f<Force> const& rhs)
	{
		return Vec8f<Force>(Cross(lhs.ang, rhs.ang) + Cross(lhs.lin, rhs.lin), Cross(lhs.ang, rhs.lin));
	}

	// Return a motion vector, equal to 'motion', but expressed at a new location equal to the previous location + 'ofs'. 
	inline Vec8f<Motion> Shift(Vec8f<Motion> const& motion, v4 ofs)
	{
		// c.f. RBDS 2.21
		return Vec8f<Motion>(motion.ang, motion.lin + Cross(motion.ang, ofs));
	}

	// Return a force vector, equal to 'force', but expressed at a new location equal to the previous location + 'ofs'.
	inline Vec8f<Force> Shift(Vec8f<Force> const& force, v4 ofs)
	{
		// c.f. RBDS 2.22
		return Vec8f<Force>(force.ang + Cross(force.lin, ofs), force.lin);
	}

	// Shift a spatial acceleration measured at some point to that same spatial
	// quantity but measured at a new point given by an offset from the old one.
	// The shift in location leaves the angular acceleration the same
	// but results in the linear acceleration changing by: a X r + w X (w X r).
	// 'acc' is the spatial acceleration to shift
	// 'avel' is the angular velocity of the frame in which 'acc' is being shifted
	// 'ofs' is the offset from the last position that 'acc' was measured at.
	inline Vec8f<Motion> ShiftAccelerationBy(Vec8f<Motion> const& acc, v4 avel, v4 ofs)
	{
		return Vec8f<Motion>(acc.ang, acc.lin + Cross(acc.ang, ofs) + Cross(avel, Cross(avel, ofs)));
	}

	// Returns the spatial cross product matrix for 'a', for use with motion vectors.
	//' i.e. b = a x m = CPM(a) * m, where m is a motion vector
	inline Mat6x8f<Motion,Motion> CPM(Vec8f<Motion> const& a)
	{
		auto cx_ang = CPM(a.ang);
		auto cx_lin = CPM(a.lin);
		return Mat6x8f<Motion, Motion>(cx_ang, m3x4Zero, cx_lin, cx_ang);
	}

	// Returns the spatial cross product matrix for 'a', for use with force vectors.
	// i.e. b = a x* f = CPM(a) * f, where f is a force vector
	inline Mat6x8f<Force, Force> CPM(Vec8f<Force> const& a)
	{
		auto cx_ang = CPM(a.ang);
		auto cx_lin = CPM(a.lin);
		return Mat6x8f<Force, Force>(cx_ang, cx_lin, m3x4Zero, cx_ang);
	}

	// Create a spatial coordinate transform
	template <typename T> Mat6x8f<T,T> Transform(m4x4 const& a2b);
	template <> inline Mat6x8f<Motion,Motion> Transform<Motion>(m4x4 const& a2b)
	{
		// Note: RBDS shows a transform to be:
		//  [E    0] = motion   [E -Erx]
		//  [-Erx E]    force = [0    E]
		// Matrix multiplies are right to left in this library, so m10 = -rxE here
		return Mat6x8f<Motion,Motion>{a2b.rot, m3x4Zero, CPM(a2b.pos) * a2b.rot, a2b.rot};
	}
	template <> inline Mat6x8f<Force,Force> Transform<Force>(m4x4 const& a2b)
	{
		return Mat6x8f<Force,Force>{a2b.rot, CPM(a2b.pos) * a2b.rot, m3x4Zero, a2b.rot};
	}

	// Spatial inertia matrix
	template <typename T, typename U> Mat6x8f<T,U> Inertia(m3x4 const& unit_inertia, v4 com, float inv_mass);
	template <> inline Mat6x8f<Motion,Force> Inertia(m3x4 const& unit_inertia, v4 com, float mass)
	{
		auto cx = CPM(com);
		auto mcx = mass * cx;
		return Mat6x8f<Motion,Force>(
			mass * unit_inertia - mass * cx * cx, mcx,
			-mcx, m3x4::Scale(mass));
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::maths
{
	PRUnitTest(SpatialTests)
	{
		using namespace spatial;

		{
			v4 vel(1,2,3,0);
			v4 ang(4,3,2,0);
			auto o2w = m4x4::Transform(v4::ZAxis(), float(maths::tau_by_4), v4(1,1,1,1));
		}
		{// Cross Products
			{// Test: CPM(m) * a == m x a
				auto v0 = v8motion(1, 1, 1, 2, 2, 2);
				auto v1 = v8motion(-1, -2, -3, -4, -5, -6);
				auto r0 = Cross(v0, v1);
				auto r1 = CPM(v0) * v1;
				PR_EXPECT(FEql(r0, r1));
			}
			{// Test: CPM(f) * a == f x* a
				auto v0 = v8force(1, 1, 1, 2, 2, 2);
				auto v1 = v8force(-1, -2, -3, -4, -5, -6);
				auto r0 = Cross(v0, v1);
				auto r1 = CPM(v0) * v1;
				PR_EXPECT(FEql(r0, r1));
			}
			{// Test: vx* == -Transpose(vx)
				auto v = v8(-2.3f, +1.3f, 0.9f, -2.2f, 0.0f, -1.0f);
				auto m0 = CPM(static_cast<v8motion>(v)); // vx
				auto m1 = CPM(static_cast<v8force>(v)); // vx*
				auto m2 = Transpose(m1);
				auto m3 = static_cast<m6x8m>(-m2);
				PR_EXPECT(FEql(m0, m3));
			}
		}
		{// Transforms
			auto a2b = m4x4::Transform(v4::ZAxis(), float(maths::tau_by_4), v4{1,1,1,1});
			auto b2c = m4x4::Transform(v4::YAxis(), float(maths::tau_by_8), v4{-1,2,-3,1});
			auto a2c = b2c * a2b;

			auto A2B = m6x8m{a2b.rot, m3x4Zero, CPM(a2b.pos) * a2b.rot, a2b.rot};
			auto B2C = m6x8m{b2c.rot, m3x4Zero, CPM(b2c.pos) * b2c.rot, b2c.rot};
			auto A2C = m6x8m{a2c.rot, m3x4Zero, CPM(a2c.pos) * a2c.rot, a2c.rot};

			auto r = B2C * A2B;
			PR_EXPECT(FEql(A2C, r));
		}
		{// Transforms
			auto a2b = m4x4::Transform(v4::ZAxis(), float(maths::tau_by_4), v4{1,1,1,1});
			auto b2c = m4x4::Transform(v4::YAxis(), float(maths::tau_by_8), v4{-1,2,-3,1});
			auto a2c = b2c * a2b;

			auto A2Bm = Transform<Motion>(a2b);
			auto B2Cm = Transform<Motion>(b2c);
			auto A2Cm = Transform<Motion>(a2c);

			auto Rm = B2Cm * A2Bm;
			PR_EXPECT(FEql(A2Cm, Rm));

			auto A2Bf = Transform<Force>(a2b);
			auto B2Cf = Transform<Force>(b2c);
			auto A2Cf = Transform<Force>(a2c);

			auto Rf = B2Cf * A2Bf;
			PR_EXPECT(FEql(A2Cf, Rf));
		}
		{// Transforming a spatial vector
			std::default_random_engine rng(1);
			std::uniform_real_distribution<double> dist(-constants<double>::tau, constants<double>::tau);

			// For a bunch of points, the velocity should be the same when described in either frame 'a' or frame 'c'
			for (float y = -0.5f; y <= 0.5f; y += 0.25f)
			for (float x = -0.5f; x <= 0.5f; x += 0.25f)
			{
				auto a2c = m4x4::Transform(v4::RandomN(rng, 0), float(dist(rng)), v4::Random(rng, v4::Origin(), 3.0f, 1));
				//auto a2c = m4x4::Translation(v4{1,0,0,1});
				auto c2a = InvertAffine(a2c);

				// A body-fixed point in frame 'a' and the same point in frame 'c'
				auto pt_a = v4{x,y,0.01f,1};
				auto pt_c = a2c * pt_a;
				//auto pt_a = v4{0.3f,-0.8f,0.01f,1};

				auto ang_a = v4{0,0,0.1f,0}; // Angular component in frame 'a'
				auto lin_a = v4{0,0.1f,0,0}; // Linear component in frame 'a'
				auto spv_a = v8motion{ang_a, lin_a};
				auto spf_a = v8force{ang_a, lin_a};

				// In frame 'a', the velocity at any point 'x' is found from:  vel_a = lin_a + Cross(ang_a, x);
				// So 'lin_a' and 'ang_a' are a description of the vector field in frame 'a'. They are not associated
				// with any particular point in frame 'a', but can be thought of as the velocity and angular velocity
				// of a body whose centre of mass is at the origin in frame 'a'.

				// In frame 'c' the body is no longer at the origin. We now want to describe the same vector field in
				// frame 'c' so we need to calculate the 'lin_c' and 'ang_c' that result in the same vector field. The
				// velocity of the origin of frame 'c' can be calculated in frame 'a' by knowing the position of frame 'c'
				// in frame 'a' space. With 'lin_c' and 'ang_c' we can calculate the velocity for any point in the vector
				// field. The velocity calculated in each frame for the same point should have the same velocity

				// a2c.pos is the position of frame 'c' in frame 'a' space.

				{// Motion vectors
					// Calculation using 3-vectors
					auto ang_c = a2c * ang_a;                         // Angular velocity in frame 'c'
					auto lin_c = a2c * lin_a + Cross(a2c.pos, ang_c); // Linear velocity in frame 'c'

					//{
					//	{
					//		std::string str;
					//		{
					//			auto frame = ldr::Frame(str, "FrameA");
					//			ldr::VectorField(str, "field", 0xFF00FF00, v8{ang_a, lin_a}, v4::Origin(), 2.0f);
					//			ldr::Box(str, "pt", 0xFFFFFFFF, 0.05f, pt_a);
					//		}
					//		ldr::Write(str, L"\\dump\\spatial_frameA.ldr");
					//	}
					//	{
					//		std::string str;
					//		{
					//			auto frame = ldr::Frame(str, "FrameC", 0xFF808080, c2a);
					//			ldr::VectorField(str, "field", 0xFFFF0000, v8{ang_c, lin_c}, v4::Origin(), 2.0f);
					//			ldr::Box(str, "pt", 0xFFFFFFFF, 0.05f, pt_c);
					//		}
					//		ldr::Write(str, L"\\dump\\spatial_frameC.ldr");
					//	}
					//}

					{
						// At some point 'pt_a' find the velocity in frame 'a'
						auto vel_a = lin_a + Cross(ang_a, pt_a);

						// The velocity at 'pt_c' in frame 'c'
						auto vel_c = lin_c + Cross(ang_c, pt_c);

						// Should be equivalent to the velocity measured in frame 'a'
						auto VEL_A = c2a * vel_c;
						PR_EXPECT(FEql(VEL_A, vel_a));
					}

					// Calculation using Spatial Transforms
					auto A2C = Transform<Motion>(a2c);
					auto spv1_c = A2C * spv_a;
					{
						auto vel_a = spv_a.LinAt(pt_a);
						auto vel_c = spv1_c.LinAt(pt_c);

						// Should be equivalent to the velocity measured in frame 'a'
						auto VEL_A = c2a * vel_c;
						PR_EXPECT(FEql(VEL_A, vel_a));
					}

					// Calculation using affine transforms
					auto spv2_c = a2c * spv_a;
					{
						auto vel_a = spv_a .LinAt(pt_a);
						auto vel_c = spv2_c.LinAt(pt_c);

						// Should be equivalent to the velocity measured in frame 'a'
						auto VEL_A = c2a * vel_c;
						PR_EXPECT(FEql(VEL_A, vel_a));
					}
				}
				{// Force vectors
					// Calculation using 3-vectors
					auto lin_c = a2c * lin_a;                         // Force in frame 'c'
					auto ang_c = a2c * ang_a - Cross(a2c.pos, lin_c); // Torque in frame 'c'

					{
						// Find the torque in frame 'a' assuming 'lin_a' applied at 'pt_a', 
						auto torque_a = ang_a + Cross(pt_a, lin_a);

						// Find the torque in frame 'c' assuming 'lin_c' applied at 'pt_c', 
						auto torque_c = ang_c + Cross(pt_c, lin_c);

						// Should be equivalent to the torque measured in frame 'a'
						auto TORQUE_A = c2a * torque_c;
						PR_EXPECT(FEql(TORQUE_A, torque_a));
					}

					// Calculation using Spatial Transforms
					auto A2C = Transform<Force>(a2c);
					auto spf1_c = A2C * spf_a;
					{
						auto torque_a = spf_a.AngAt(pt_a);
						auto torque_c = spf1_c.AngAt(pt_c);

						// Should be equivalent to the torque measured in frame 'a'
						auto TORQUE_A = c2a * torque_c;
						PR_EXPECT(FEql(TORQUE_A, torque_a));
					}

					// Calculation using affine transforms
					auto spf2_c = a2c * spf_a;
					{
						auto torque_a = spf_a.AngAt(pt_a);
						auto torque_c = spf2_c.AngAt(pt_c);

						// Should be equivalent to the torque measured in frame 'a'
						auto TORQUE_A = c2a * torque_c;
						PR_EXPECT(FEql(TORQUE_A, torque_a));
					}
				}
			}
		}
		{// Bug test: spatial::Inertia() must include parallel axis term in m00 for non-zero CoM
			// The m00 block should be Io = Ic - m*S(c)², not just Ic.
			// The off-diagonal blocks (m01, m10) and m11 are correct, only m00 is missing the correction.
			auto unit_inertia = m3x4::Scale(0.4f, 0.4f, 0.4f); // sphere radius 1
			auto com = v4{0, 1, 0, 0};
			auto mass = 5.0f;

			auto si = Inertia<Motion,Force>(unit_inertia, com, mass);

			// Expected m00: Io = Ic - m*CPM(c)*CPM(c), where Ic = m * unit_inertia
			auto cx = CPM(com);
			auto Ic = mass * unit_inertia;
			auto Io = Ic - mass * cx * cx;
			PR_EXPECT(FEql(si.m00, Io)); // Bug: si.m00 == Ic, missing the -m*cx*cx term
		}
	}
}
#endif

