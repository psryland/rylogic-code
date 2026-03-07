//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(Matrix)
	{
		std::default_random_engine rng;
		TestClass_Matrix()
			: rng(1)
		{}

		PRUnitTestMethod(ZeroFillIdentity)
		{
			{
				auto m = Matrix<double>::Fill(2, 3, 42.0);
				for (auto v : m.data())
					PR_EXPECT(v == 42);
			}
			{
				auto m = Matrix<double>::Zero(2, 3);
				for (auto v : m.data())
					PR_EXPECT(v == 0);
			}
			{
				auto id = Matrix<float>::Identity(5, 5);
				for (int i = 0; i != 5; ++i)
					for (int j = 0; j != 5; ++j)
						PR_EXPECT(id(i, j) == (i == j ? 1.0f : 0.0f));
			}
		}

		PRUnitTestMethod(LUDecomposition)
		{
			auto m = MatrixLU<double>(4, 4, 
			{
				  1.0, +2.0,  3.0, +1.0,
				  4.0, -5.0,  6.0, +5.0,
				  7.0, +8.0,  9.0, -9.0,
				-10.0, 11.0, 12.0, +0.0,
			});
			auto res = Matrix<double>(4, 4,
			{
				3.0, 0.66666666666667, 0.33333333333333, 0.33333333333333,
				6.0, -9.0, -0.33333333333333, -0.22222222222222,
				9.0, 2.0, -11.333333333333, -0.3921568627451,
				12.0, 3.0, -3.0, -14.509803921569,
			});
			PR_EXPECT(FEql(m.lu, res));
		}

		PRUnitTestMethod(Invert)
		{
			auto m = Matrix<double>(4, 4, { 1, 2, 3, 1, 4, -5, 6, 5, 7, 8, 9, -9, -10, 11, 12, 0 });
			auto inv = Invert(m);
			auto INV = Matrix<double>(4, 4,
			{
				+0.258783783783783810, -0.018918918918918920, +0.018243243243243241, -0.068918918918918923,
				+0.414864864864864790, -0.124324324324324320, -0.022972972972972971, -0.024324324324324322,
				-0.164639639639639650, +0.098198198198198194, +0.036261261261261266, +0.048198198198198199,
				+0.405405405405405430, -0.027027027027027029, -0.081081081081081086, -0.027027027027027025,
			});
			PR_EXPECT(FEql(inv, INV));
		}

		PRUnitTestMethod(InvertCompareWithMat4x4)
		{
			// Verify Matrix<float> invert matches Mat4x4<float> invert
			using v4f = Vec4<float>;
			using m4f = Mat4x4<float>;
			auto lo = v4f(-5.0f, -5.0f, -5.0f, -5.0f);
			auto hi = v4f(+5.0f, +5.0f, +5.0f, +5.0f);
			auto M = m4f{ Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi) };
			Matrix<float> m(M);

			PR_EXPECT(FEql(m, M));
			PR_EXPECT(IsInvertible(m) == IsInvertible(M));

			if (IsInvertible(M))
			{
				auto m1 = Invert(m);
				auto M1 = Invert(M);
				PR_EXPECT(FEql(m1, M1));
			}

			auto m2 = Transpose(m);
			auto M2 = math::Transpose(M);
			PR_EXPECT(FEql(m2, M2));
		}

		PRUnitTestMethod(Multiply)
		{
			// Non-square multiply
			double data0[] = {1, 2, 3, 4, 0.1, 0.2, 0.3, 0.4, -4, -3, -2, -1};
			double data1[] = {1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4};
			double rdata[] = {30, 30, 30, 30, 30, 3, 3, 3, 3, 3, -20, -20, -20, -20, -20};
			auto a2b = Matrix<double>(3, 4, data0);
			auto b2c = Matrix<double>(4, 5, data1);
			auto A2C = Matrix<double>(3, 5, rdata);
			auto a2c = b2c * a2b;
			PR_EXPECT(FEql(a2c, A2C));

			// Compare with Mat4x4 multiply
			using v4f = Vec4<float>;
			using m4f = Mat4x4<float>;
			auto lo = v4f(-5.0f, -5.0f, -5.0f, -5.0f);
			auto hi = v4f(+5.0f, +5.0f, +5.0f, +5.0f);
			auto V0 = Random<v4f>(rng, lo, hi);
			auto M0 = m4f{ Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi) };
			auto M1 = m4f{ Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi), Random<v4f>(rng, lo, hi) };

			auto v0 = Matrix<float>(V0);
			auto m0 = Matrix<float>(M0);
			auto m1 = Matrix<float>(M1);

			PR_EXPECT(FEql(v0, V0));
			PR_EXPECT(FEql(m0, M0));
			PR_EXPECT(FEql(m1, M1));

			auto V2 = M0 * V0;
			auto v2 = m0 * v0;
			PR_EXPECT(FEql(v2, V2));

			auto M2 = M0 * M1;
			auto m2 = m0 * m1;
			PR_EXPECT(FEqlRelative(m2, M2, 0.0001f));
		}

		PRUnitTestMethod(MultiplyRoundTrip)
		{
			const int SZ = 100;
			Matrix<float> m(SZ, SZ);
			for (int k = 0; k != 10; ++k)
			{
				m = Matrix<float>::Random(rng, SZ, SZ, -5.0f, 5.0f);
				if (IsInvertible(m))
				{
					auto m_inv = Invert(m);
					auto i0 = Matrix<float>::Identity(SZ, SZ);
					auto i1 = m * m_inv;
					auto i2 = m_inv * m;

					PR_EXPECT(FEqlRelative(i0, i1, 0.001f));
					PR_EXPECT(FEqlRelative(i0, i2, 0.001f));
					break;
				}
			}
		}

		PRUnitTestMethod(Transpose)
		{
			const int vecs = 4, cmps = 3;
			auto m = Matrix<double>::Random(rng, vecs, cmps, -5.0, 5.0);
			auto t = math::Transpose(m);

			PR_EXPECT(m.vecs() == vecs);
			PR_EXPECT(m.cmps() == cmps);
			PR_EXPECT(t.vecs() == cmps);
			PR_EXPECT(t.cmps() == vecs);

			for (int r = 0; r != vecs; ++r)
				for (int c = 0; c != cmps; ++c)
					PR_EXPECT(m(r, c) == t(c, r));
		}

		PRUnitTestMethod(Resizing)
		{
			auto M = Matrix<double>::Random(rng, 4, 3, -5.0, 5.0);
			auto m = M;
			auto t = math::Transpose(M);

			// Resizing a normal matrix adds more vectors, and preserves data
			PR_EXPECT(m.vecs() == 4);
			PR_EXPECT(m.cmps() == 3);
			m.resize(5);
			PR_EXPECT(m.vecs() == 5);
			PR_EXPECT(m.cmps() == 3);
			for (int r = 0; r != m.vecs(); ++r)
			{
				for (int c = 0; c != m.cmps(); ++c)
				{
					if (r < 4 && c < 3)
						PR_EXPECT(m(r, c) == M(r, c));
					else
						PR_EXPECT(m(r, c) == 0);
				}
			}

			// Resizing a transposed matrix adds more transposed vectors, and preserves data 
			PR_EXPECT(t.vecs() == 3);
			PR_EXPECT(t.cmps() == 4);
			t.resize(5);
			PR_EXPECT(t.vecs() == 5);
			PR_EXPECT(t.cmps() == 4);
			for (int r = 0; r != t.vecs(); ++r)
			{
				for (int c = 0; c != t.cmps(); ++c)
				{
					if (r < 3 && c < 4)
						PR_EXPECT(t(r, c) == M(c, r));
					else
						PR_EXPECT(t(r, c) == 0);
				}
			}
		}

		PRUnitTestMethod(DotProduct)
		{
			auto a = Matrix<float>(1, 3, {1.0f, 2.0f, 3.0f});
			auto b = Matrix<float>(1, 3, {3.0f, 2.0f, 1.0f});
			auto r = Dot(a, b);
			PR_EXPECT(FEql(r, 10.0f));
		}

		PRUnitTestMethod(EigenSymmetricIdentity)
		{
			// Identity matrix: eigenvalues all 1, eigenvectors are axis-aligned
			auto I = Matrix<double>::Identity(3, 3);
			auto result = EigenSymmetric(I);

			PR_EXPECT(result.values.cmps() == 3);
			for (int i = 0; i != 3; ++i)
				PR_EXPECT(std::abs(result.values(0, i) - 1.0) < 1e-10);
		}

		PRUnitTestMethod(EigenSymmetricDiagonal)
		{
			// Diagonal matrix: eigenvalues are the diagonal entries, sorted descending
			auto D = Matrix<double>(3, 3, { 5,0,0, 0,2,0, 0,0,8 });
			auto result = EigenSymmetric(D);

			PR_EXPECT(std::abs(result.values(0, 0) - 8.0) < 1e-10);
			PR_EXPECT(std::abs(result.values(0, 1) - 5.0) < 1e-10);
			PR_EXPECT(std::abs(result.values(0, 2) - 2.0) < 1e-10);
		}

		PRUnitTestMethod(EigenSymmetricKnown3x3)
		{
			// M = [2 1 0; 1 3 1; 0 1 2] has eigenvalues 4, 2, 1.
			auto M = Matrix<double>(3, 3, { 2,1,0, 1,3,1, 0,1,2 });
			auto result = EigenSymmetric(M);

			PR_EXPECT(std::abs(result.values(0, 0) - 4.0) < 1e-8);
			PR_EXPECT(std::abs(result.values(0, 1) - 2.0) < 1e-8);
			PR_EXPECT(std::abs(result.values(0, 2) - 1.0) < 1e-8);

			// Verify eigenvectors: M*v should equal λ*v
			for (int k = 0; k != 3; ++k)
			{
				auto lambda = result.values(0, k);
				for (int r = 0; r != 3; ++r)
				{
					auto mv = 0.0;
					for (int c = 0; c != 3; ++c)
						mv += M(r, c) * result.vectors(c, k);

					auto lv = lambda * result.vectors(r, k);
					PR_EXPECT(std::abs(mv - lv) < 1e-8);
				}
			}
		}

		PRUnitTestMethod(EigenSymmetricLarger)
		{
			// 5×5 symmetric matrix
			auto M = Matrix<double>(5, 5, {
				 4, 1, -2,  2, 0,
				 1, 2,  0,  1, 0,
				-2, 0,  3, -2, 0,
				 2, 1, -2,  5, 0,
				 0, 0,  0,  0, 1,
			});
			auto result = EigenSymmetric(M);

			// Verify A*v = lambda*v for each eigenpair
			for (int k = 0; k != 5; ++k)
			{
				auto lambda = result.values(0, k);
				for (int r = 0; r != 5; ++r)
				{
					auto mv = 0.0;
					for (int c = 0; c != 5; ++c)
						mv += M(r, c) * result.vectors(c, k);

					auto lv = lambda * result.vectors(r, k);
					PR_EXPECT(std::abs(mv - lv) < 1e-6);
				}
			}
		}

		PRUnitTestMethod(EigenSymmetricSingleElement)
		{
			// 1×1 matrix: eigenvalue is the single element
			auto M = Matrix<double>(1, 1, { 7.0 });
			auto result = EigenSymmetric(M);
			PR_EXPECT(result.values.cmps() == 1);
			PR_EXPECT(std::abs(result.values(0, 0) - 7.0) < 1e-10);
		}

		PRUnitTestMethod(EigenSymmetricEmpty)
		{
			// 0×0 matrix: empty result
			auto M = Matrix<double>(0, 0);
			auto result = EigenSymmetric(M);
			PR_EXPECT(result.values.cmps() == 0);
			PR_EXPECT(result.vectors.cmps() == 0);
		}

		PRUnitTestMethod(EigenTopKSmall)
		{
			// Top-2 eigenpairs of a 3×3 matrix
			auto M = Matrix<double>(3, 3, { 2,1,0, 1,3,1, 0,1,2 });
			auto result = EigenTopK(M, 2);

			PR_EXPECT(result.values.cmps() == 2);
			PR_EXPECT(result.vectors.vecs() == 3);
			PR_EXPECT(result.vectors.cmps() == 2);

			PR_EXPECT(std::abs(result.values(0, 0) - 4.0) < 1e-8);
			PR_EXPECT(std::abs(result.values(0, 1) - 2.0) < 1e-8);

			// Verify A*v = lambda*v
			for (int k = 0; k != 2; ++k)
			{
				auto lambda = result.values(0, k);
				for (int r = 0; r != 3; ++r)
				{
					auto mv = 0.0;
					for (int c = 0; c != 3; ++c)
						mv += M(r, c) * result.vectors(c, k);

					auto lv = lambda * result.vectors(r, k);
					PR_EXPECT(std::abs(mv - lv) < 1e-8);
				}
			}
		}
	};
}
#endif
