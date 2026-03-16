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
//
// Spatial matrices 
//  e.g. Spatial inertia matrix is a mapping from M6 to F6
//
// Spatial transforms are a special case of spatial matrices.
// Spatial transforms have this form (Featherstone convention):
//   [ E    0 ]   where E = rotation, r = position of source frame's origin in target frame
//   [r^E   E ]   r^ = CPM(r) = cross product matrix of r
// This means a spatial transform can be created from a normal affine transform.
//   m4x4 o2w = [o2w.rot                   0  ]
//              [CPM(o2w.pos)*o2w.rot  o2w.rot]  (r = o2w.pos)
// If X is a matrix that transforms a to b for M6 vectors, and X* is a matrix that
// performs the same transform for F6 vectors, then X* == X^-T (invert then transpose).
// A spatial transform from A to B for motion vectors = bXa.
// A spatial transform from A to B for force vectors = bX*a.
//  bX*a == bXa^-T (invert then transpose)
// 
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/types/vector8.h"
#include "pr/math/types/matrix6x8.h"
#include "pr/math/types/matrix4x4.h"
#include "pr/math/types/matrix3x4.h"
#include "pr/math/types/vector4.h"

namespace pr::math::spatial
{
	// Spatial vector spaces
	struct Motion {};
	struct Force {};

	#pragma region Operators

	// Rotate a spatial motion vector
	template <ScalarTypeFP S> constexpr Vec8<S, Motion> pr_vectorcall operator * (Mat3x4<S> const& a2b, Vec8<S, Motion> vec) noexcept
	{
		// [ E    0] * [v.ang] = [E*v.ang              ]
		// [r^E   E]   [v.lin]   [E*v.lin + r^(E*v.ang)] where r^ = (0,0,0) for pure rotation
		auto ang_b = a2b * vec.ang;
		auto lin_b = a2b * vec.lin;
		return Vec8<S, Motion>{ang_b, lin_b};
	}

	// Rotate a spatial force vector
	template <ScalarTypeFP S> constexpr Vec8<S, Force> pr_vectorcall operator * (Mat3x4<S> const& a2b, Vec8<S, Force> vec) noexcept
	{
		// [E   r^E] * [v.ang] = [E*v.ang + r^(E*v.lin)]
		// [0    E ]   [v.lin]   [E*v.lin              ] where r^ = (0,0,0) for pure rotation
		auto lin_b = a2b * vec.lin;
		auto ang_b = a2b * vec.ang;
		return Vec8<S, Force>{ang_b, lin_b};
	}

	// Transform a spatial motion vector by an affine transform
	template <ScalarTypeFP S> constexpr Vec8<S, Motion> pr_vectorcall operator * (Mat4x4<S> const& a2b, Vec8<S, Motion> vec) noexcept
	{
		// [ E    0] * [v.ang] = [E*v.ang              ]
		// [r^E   E]   [v.lin]   [E*v.lin + r^(E*v.ang)] where r = a2b.pos
		pr_assert("'lhs' is not an affine transform" && IsAffine(a2b));
		auto ang_b = a2b.rot * vec.ang;
		auto lin_b = a2b.rot * vec.lin + Cross(a2b.pos, ang_b);
		return Vec8<S, Motion>{ang_b, lin_b};
	}

	// Transform a spatial force vector by an affine transform
	template <ScalarTypeFP S> constexpr Vec8<S, Force> pr_vectorcall operator * (Mat4x4<S> const& a2b, Vec8<S, Force> vec) noexcept
	{
		// [E   r^E] * [v.ang] = [E*v.ang + r^(E*v.lin)]
		// [0    E ]   [v.lin]   [E*v.lin              ] where r = a2b.pos
		pr_assert("'lhs' is not an affine transform" && IsAffine(a2b));
		auto lin_b = a2b.rot * vec.lin;
		auto ang_b = a2b.rot * vec.ang + Cross(a2b.pos, lin_b);
		return Vec8<S, Force>{ang_b, lin_b};
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
	// The dot product is only defined for Dot(motion,force) and Dot(force,motion). e.g Dot(force, velocity) == power delivered
	template <ScalarTypeFP S> constexpr S Dot(Vec8<S, Motion> lhs, Vec8<S, Force> rhs) noexcept
	{
		// motion and force are vectors in the dual spaces M and F
		// A property of dual spaces is dot(m,f) = transpose(m)*f
		return Dot3(lhs.ang, rhs.ang) + Dot3(lhs.lin, rhs.lin);
	}
	template <ScalarTypeFP S> constexpr S Dot(Vec8<S, Force> lhs, Vec8<S, Motion> rhs) noexcept
	{
		return Dot(rhs, lhs);
	}

	// Spatial cross product.
	// There are two cross product operators, one for motion vectors (rx) and one for forces (rx*)
	template <ScalarTypeFP S> constexpr Vec8<S, Motion> pr_vectorcall Cross(Vec8<S, Motion> lhs, Vec8<S, Motion> rhs) noexcept
	{
		return Vec8<S, Motion>{
			Cross(lhs.ang, rhs.ang),
			Cross(lhs.ang, rhs.lin) + Cross(lhs.lin, rhs.ang)
		};
	}
	template <ScalarTypeFP S> constexpr Vec8<S, Force> pr_vectorcall Cross(Vec8<S, Force> lhs, Vec8<S, Force> rhs) noexcept
	{
		return Vec8<S, Force>{
			Cross(lhs.ang, rhs.ang) + Cross(lhs.lin, rhs.lin),
			Cross(lhs.ang, rhs.lin)
		};
	}

	// Return a motion vector, equal to 'motion', but expressed at a new location equal to the previous location + 'ofs'. 
	template <ScalarTypeFP S> constexpr Vec8<S, Motion> pr_vectorcall Shift(Vec8<S, Motion> motion, Vec4<S> ofs) noexcept
	{
		// c.f. RBDS 2.21
		return Vec8<S, Motion>(motion.ang, motion.lin + Cross(motion.ang, ofs));
	}

	// Return a force vector, equal to 'force', but expressed at a new location equal to the previous location + 'ofs'.
	template <ScalarTypeFP S> constexpr Vec8<S, Force> pr_vectorcall Shift(Vec8<S, Force> force, Vec4<S> ofs) noexcept
	{
		// c.f. RBDS 2.22
		return Vec8<S, Force>(force.ang + Cross(force.lin, ofs), force.lin);
	}

	// Shift a spatial acceleration measured at some point to that same spatial
	// quantity but measured at a new point given by an offset from the old one.
	// The shift in location leaves the angular acceleration the same
	// but results in the linear acceleration changing by: a X r + w X (w X r).
	// 'acc' is the spatial acceleration to shift
	// 'avel' is the angular velocity of the frame in which 'acc' is being shifted
	// 'ofs' is the offset from the last position that 'acc' was measured at.
	template <ScalarTypeFP S> constexpr Vec8<S, Motion> pr_vectorcall ShiftAccelerationBy(Vec8<S, Motion> acc, Vec4<S> avel, Vec4<S> ofs) noexcept
	{
		return Vec8<S, Motion>{
			acc.ang,
			acc.lin + Cross(acc.ang, ofs) + Cross(avel, Cross(avel, ofs))
		};
	}

	// Returns the spatial cross product matrix for 'a', for use with motion vectors.
	//' i.e. b = a x m = CPM(a) * m, where m is a motion vector
	template <ScalarTypeFP S> constexpr Mat6x8<S, Motion, Motion> CPM(Vec8<S, Motion> a) noexcept
	{
		auto cx_ang = math::CPM<Mat3x4<S>>(a.ang);
		auto cx_lin = math::CPM<Mat3x4<S>>(a.lin);
		return Mat6x8<S, Motion, Motion>(cx_ang, Zero<Mat3x4<S>>(), cx_lin, cx_ang);
	}

	// Returns the spatial cross product matrix for 'a', for use with force vectors.
	// i.e. b = a x* f = CPM(a) * f, where f is a force vector
	template <ScalarTypeFP S> constexpr Mat6x8<S, Force, Force> CPM(Vec8<S, Force> a) noexcept
	{
		auto cx_ang = math::CPM<Mat3x4<S>>(a.ang);
		auto cx_lin = math::CPM<Mat3x4<S>>(a.lin);
		return Mat6x8<S, Force, Force>(cx_ang, cx_lin, Zero<Mat3x4<S>>(), cx_ang);
	}

	// Create a spatial coordinate transform
	template <typename VecSpace, ScalarTypeFP S> constexpr Mat6x8<S, VecSpace, VecSpace> Transform(Mat4x4<S> const& a2b) noexcept
	{
		// Note: RBDA shows a transform to be (with r = a2b.pos):
		//  [ E    0] = motion         [E   r^E]
		//  [r^E   E]          force = [0    E ]
		if constexpr (std::same_as<VecSpace, Motion>)
			return Mat6x8<S, Motion, Motion>{a2b.rot, Zero<Mat3x4<S>>(), math::CPM<Mat3x4<S>>(a2b.pos) * a2b.rot, a2b.rot};
		else if constexpr (std::same_as<VecSpace, Force>)
			return Mat6x8<S, Force, Force>{a2b.rot, math::CPM<Mat3x4<S>>(a2b.pos) * a2b.rot, Zero<Mat3x4<S>>(), a2b.rot};
		else
			static_assert(std::is_same_v<VecSpace, void>, "Invalid VecSpace");
	}

	// Spatial inertia matrix
	template <ScalarTypeFP S> inline Mat6x8<S, Motion, Force> Inertia(Mat3x4<S> unit_inertia, Vec4<S> com, S mass) noexcept
	{
		auto cx = math::CPM<Mat3x4<S>>(com);
		auto mcx = mass * cx;
		return Mat6x8<S, Motion, Force>{
			mass * unit_inertia - mass * cx * cx, mcx,
			-mcx, Scale<Mat3x4<S>>(mass)
		};
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::math::spatial::tests
{
	PRUnitTestClass(SpatialTests)
	{
		std::default_random_engine rng;
		TestClass_SpatialTests()
			:rng(1)
		{
		//	v4 vel(1,2,3,0);
		//	v4 ang(4,3,2,0);
		//	auto o2w = Mat4x4::Transform(v4::ZAxis(), float(maths::tau_by_4), v4(1,1,1,1));
		}
		PRUnitTestMethod(CrossProducts, float, double)
		{
			using v8motionT = Vec8<T, Motion>;
			using v8forceT = Vec8<T, Force>;

			{// Test: CPM(m) * a == m x a
				auto v0 = v8motionT(1, 1, 1, 2, 2, 2);
				auto v1 = v8motionT(-1, -2, -3, -4, -5, -6);
				auto r0 = Cross(v0, v1);
				auto r1 = CPM(v0) * v1;
				PR_EXPECT(FEql(r0, r1));
			}
			{// Test: CPM(f) * a == f x* a
				auto v0 = v8forceT(1, 1, 1, 2, 2, 2);
				auto v1 = v8forceT(-1, -2, -3, -4, -5, -6);
				auto r0 = Cross(v0, v1);
				auto r1 = CPM(v0) * v1;
				PR_EXPECT(FEql(r0, r1));
			}
			{// Test: vx* == -Transpose(vx)
				auto v = Vec8<T, void>(-2.3f, +1.3f, 0.9f, -2.2f, 0.0f, -1.0f);
				auto m0 = CPM(static_cast<v8motionT>(v)); // vx
				auto m1 = CPM(static_cast<v8forceT>(v)); // vx*
				auto m2 = Transpose(m1);
				auto m3 = static_cast<Mat6x8<T, Motion, Motion>>(-m2);
				PR_EXPECT(FEql(m0, m3));
			}
		}
		PRUnitTestMethod(TransformsTests, float, double)
		{
			using Mat6x8MM = Mat6x8<T, Motion, Motion>;
			using Mat4x4 = Mat4x4<T>;
			using Mat3x4 = Mat3x4<T>;
			using Vec4 = Vec4<T>;

			auto a2b = Mat4x4::Transform(ZAxis<Vec4>(), constants<T>::tau_by_4, Vec4{1,1,1,1});
			auto b2c = Mat4x4::Transform(YAxis<Vec4>(), constants<T>::tau_by_8, Vec4{-1,2,-3,1});
			auto a2c = b2c * a2b;

			auto A2B = Mat6x8MM{a2b.rot, Zero<Mat3x4>(), math::CPM<Mat3x4>(a2b.pos) * a2b.rot, a2b.rot};
			auto B2C = Mat6x8MM{b2c.rot, Zero<Mat3x4>(), math::CPM<Mat3x4>(b2c.pos) * b2c.rot, b2c.rot};
			auto A2C = Mat6x8MM{a2c.rot, Zero<Mat3x4>(), math::CPM<Mat3x4>(a2c.pos) * a2c.rot, a2c.rot};

			auto r = B2C * A2B;
			PR_EXPECT(FEql(A2C, r));
		}
		PRUnitTestMethod(TransformTests2, float, double)
		{
			using Mat4x4 = Mat4x4<T>;
			using Vec4 = Vec4<T>;

			auto a2b = Mat4x4::Transform(ZAxis<Vec4>(), constants<T>::tau_by_4, Vec4{1,1,1,1});
			auto b2c = Mat4x4::Transform(YAxis<Vec4>(), constants<T>::tau_by_8, Vec4{-1,2,-3,1});
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
		PRUnitTestMethod(TransformingSpatialVectors, float, double)
		{
			using v8motionT = Vec8<T, Motion>;
			using v8forceT = Vec8<T, Force>;
			using Mat4x4 = Mat4x4<T>;
			using Mat3x4 = Mat3x4<T>;
			using Vec4 = Vec4<T>;
			using Vec3 = Vec3<T>;

			std::uniform_real_distribution<T> dist(-constants<T>::tau, constants<T>::tau);

			// For a bunch of points, the velocity should be the same when described in either frame 'a' or frame 'c'
			for (T y = -T(0.5); y <= T(0.5); y += T(0.25))
			for (T x = -T(0.5); x <= T(0.5); x += T(0.25))
			{
				auto a2c = Mat4x4::Transform(RandomN<Vec3>(rng).w0(), dist(rng), Random<Vec4>(rng, Origin<Vec4>(), T(3)).w1());
				//auto a2c = Mat4x4::Translation(Vec4{1,0,0,1});
				auto c2a = InvertOrthonormal(a2c);

				// A body-fixed point in frame 'a' and the same point in frame 'c'
				auto pt_a = Vec4{x,y,0.01f,1};
				auto pt_c = a2c * pt_a;
				//auto pt_a = Vec4{0.3f,-0.8f,0.01f,1};

				auto ang_a = Vec4{0,0,0.1f,0}; // Angular component in frame 'a'
				auto lin_a = Vec4{0,0.1f,0,0}; // Linear component in frame 'a'
				auto spv_a = v8motionT{ang_a, lin_a};
				auto spf_a = v8forceT{ang_a, lin_a};

				// In frame 'a', the velocity at any point 'x' is found from:  vel_a = lin_a + Cross(ang_a, x);
				// So 'lin_a' and 'ang_a' are a description of the vector field in frame 'a'. They are not associated
				// with any particular point in frame 'a', but can be thought of as the velocity and angular velocity
				// of a body whose centre of mass is at the origin in frame 'a'.

				// In frame 'c' the body is no longer at the origin. We now want to describe the same vector field in
				// frame 'c' so we need to calculate the 'lin_c' and 'ang_c' that result in the same vector field. The
				// velocity of the origin of frame 'c' can be calculated in frame 'a' by knowing the position of frame 'c'
				// in frame 'a' space. With 'lin_c' and 'ang_c' we can calculate the velocity for any point in the vector
				// field. The velocity calculated in each frame for the same point should have the same velocity

				// a2c.pos is the position of frame 'a' origin in frame 'c' coordinates.

				{// Motion vectors
					// Calculation using 3-vectors
					auto ang_c = a2c * ang_a;                         // Angular velocity in frame 'c'
					auto lin_c = a2c * lin_a + Cross(a2c.pos, ang_c); // Linear velocity in frame 'c'

					//{
					//	{
					//		std::string str;
					//		{
					//			auto frame = ldr::Frame(str, "FrameA");
					//			ldr::VectorField(str, "field", 0xFF00FF00, v8{ang_a, lin_a}, Origin<Vec4>(), 2.0f);
					//			ldr::Box(str, "pt", 0xFFFFFFFF, 0.05f, pt_a);
					//		}
					//		ldr::Write(str, L"\\dump\\spatial_frameA.ldr");
					//	}
					//	{
					//		std::string str;
					//		{
					//			auto frame = ldr::Frame(str, "FrameC", 0xFF808080, c2a);
					//			ldr::VectorField(str, "field", 0xFFFF0000, v8{ang_c, lin_c}, Origin<Vec4>(), 2.0f);
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
					auto lin_c = a2c * lin_a;                          // Force in frame 'c'
					auto ang_c = a2c * ang_a + Cross(a2c.pos, lin_c); // Torque in frame 'c' (Plücker force transform)

					{
						// Verify torque at a point is frame-invariant
						auto spf_c = v8forceT{ang_c, lin_c};
						auto torque_a = spf_a.AngAt(pt_a);
						auto torque_c = spf_c.AngAt(pt_c);

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
		PRUnitTestMethod(ShiftTests, float, double)
		{
			using v8motionT = Vec8<T, Motion>;
			using v8forceT = Vec8<T, Force>;
			using Vec4 = Vec4<T>;

			{// Motion: Shift velocity to an offset point
				auto ang = Vec4{0, 0, T(1), 0};  // spinning about Z
				auto lin = Vec4{0, 0, 0, 0};      // zero velocity at reference point
				auto motion = v8motionT{ang, lin};
				auto ofs = Vec4{T(1), 0, 0, 0};   // point at (1,0,0)

				// Velocity at offset: v + w × r = 0 + (0,0,1)×(1,0,0) = (0,1,0)
				auto shifted = Shift(motion, ofs);
				PR_EXPECT(FEql(shifted.ang, ang)); // angular unchanged
				PR_EXPECT(FEql(shifted.lin, Vec4{0, T(1), 0, 0}));
			}
			{// Force: Shift force reference point
				auto ang = Vec4{0, 0, 0, 0};      // no torque at reference
				auto lin = Vec4{0, 0, T(1), 0};   // force in Z direction
				auto force = v8forceT{ang, lin};
				auto ofs = Vec4{T(1), 0, 0, 0};   // shift to (1,0,0)

				// Torque changes: τ + f × r = 0 + (0,0,1)×(1,0,0) = (0,-1,0)... wait: Cross(f, ofs) = f × ofs
				auto shifted = Shift(force, ofs);
				PR_EXPECT(FEql(shifted.lin, lin)); // force unchanged
				auto expected_ang = Cross(lin, ofs); // f × r
				PR_EXPECT(FEql(shifted.ang, expected_ang));
			}
			{// ShiftAccelerationBy: centripetal + tangential
				auto ang_acc = Vec4{0, 0, T(2), 0}; // angular acceleration about Z
				auto lin_acc = Vec4{0, 0, 0, 0};     // zero linear acceleration at reference
				auto acc = v8motionT{ang_acc, lin_acc};
				auto avel = Vec4{0, 0, T(3), 0};     // angular velocity about Z
				auto ofs = Vec4{T(1), 0, 0, 0};      // offset in X

				// a_lin = 0 + α × r + ω × (ω × r)
				// α × r = (0,0,2) × (1,0,0) = (0,2,0)
				// ω × r = (0,0,3) × (1,0,0) = (0,3,0)
				// ω × (ω × r) = (0,0,3) × (0,3,0) = (-9,0,0)
				// total = (0,2,0) + (-9,0,0) = (-9,2,0)
				auto shifted = ShiftAccelerationBy(acc, avel, ofs);
				PR_EXPECT(FEql(shifted.ang, ang_acc)); // angular unchanged
				PR_EXPECT(FEql(shifted.lin, Vec4{T(-9), T(2), 0, 0}));
			}
		}
		PRUnitTestMethod(BugTest_Inertia, float, double)
		{
			using Mat3x4 = Mat3x4<T>;
			using Vec4 = Vec4<T>;

			// Bug test: spatial::Inertia() must include parallel axis term in m00 for non-zero CoM
			// The m00 block should be Io = Ic - m*S(c)², not just Ic.
			// The off-diagonal blocks (m01, m10) and m11 are correct, only m00 is missing the correction.
			auto unit_inertia = Scale<Mat3x4>(Vec4{ T(0.4), T(0.4), T(0.4), T(0) }); // sphere radius 1
			auto com = Vec4{0, 1, 0, 0};
			auto mass = T(5.0);

			auto si = Inertia(unit_inertia, com, mass);

			// Expected m00: Io = Ic - m*CPM(c)*CPM(c), where Ic = m * unit_inertia
			auto cx = math::CPM<Mat3x4>(com);
			auto Ic = mass * unit_inertia;
			auto Io = Ic - mass * cx * cx;
			PR_EXPECT(FEql(si.m00, Io)); // Bug: si.m00 == Ic, missing the -m*cx*cx term
		}
	};
}
#endif
