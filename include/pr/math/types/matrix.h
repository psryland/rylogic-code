//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/constants.h"
#include "pr/math/types/vector4.h"

namespace pr::math
{
	// Dynamically allocated NxM matrix
	template <ScalarType S>
	struct Matrix
	{
		// Notes:
		//  - Matrix has reference semantics because it is potentially a large object.
		//  - Data is stored as contiguous vectors (like m4x4 does, i.e. row major)
		//    Visually, the matrix is displayed with the vectors as columns.
		//    e.g.
		//     [{x}  {y}  {z}]
		//    is:                memory order:
		//     [x.x  y.x  z.x]    [0  4   8]
		//     [x.y  y.y  z.y]    [1  5   9]
		//     [x.z  y.z  z.z]    [2  6  10]
		//     [x.w  y.w  z.w]    [3  7  11]
		//  - 'vec_count' is the number of vectors in the matrix
		//  - 'cmp_count' is the number of components in each vector
		//  - The Row/Column description is confusing as hell because the matrix is displayed
		//    with the vectors as columns, even though the data are stored in rows. So I'm not
		//    using Row/Column notation. 'Vector/Component' notation is less ambiguous.
		//  - Accessors use 'vec' first then 'cmp' so that from left-to-right you select
		//    the vector first then the component.
		//  - The 'transposed' state should not be visible outside of the matrix. The matrix
		//    should look like any other matrix from an interface point-of-view.
		//  - 'transposed' is not a constructor parameter because almost anything is convertible
		//    to bool which causes the wrong constructor to be called. To create a matrix initialised
		//    with transposed data, use the constructor with all parameters

		using value_type = S;
		enum { LocalBufCount = 16 };

	protected:

		S    m_buf[LocalBufCount]; // Local buffer for small matrices
		S*   m_data;               // linear buffer of matrix elements
		int  m_vecs;               // number of vectors (Y dimension, aka rows in row major matrix)
		int  m_cmps;               // number of components per vector (X dimension, aka columns)
		bool m_transposed;         // interpret the matrix as transposed (don't make this public, callers shouldn't know a matrix is transposed)

	public:

		Matrix() noexcept
			: m_data(&m_buf[0])
			, m_vecs(0)
			, m_cmps(0)
			, m_transposed(false)
		{
			resize(0,0);
		}
		Matrix(int vecs, int cmps) noexcept
			:Matrix()
		{
			resize(vecs, cmps, false);
		}
		Matrix(int vecs, int cmps, S const* data) noexcept
			:Matrix(vecs, cmps, std::initializer_list<S>(data, data + vecs * cmps))
		{
		}
		Matrix(int vecs, int cmps, std::initializer_list<S> data) noexcept
			:Matrix(vecs, cmps, data, false)
		{
		}
		Matrix(int vecs, int cmps, std::initializer_list<S> data, bool transposed) noexcept
			:Matrix(!transposed ? vecs : cmps, !transposed ? cmps : vecs)
		{
			pr_assert("Data length mismatch" && int(data.size()) == vecs*cmps);
			memcpy(m_data, data.begin(), sizeof(S) * size_t(vecs * cmps));
			m_transposed = transposed;
		}
		Matrix(Matrix&& rhs) noexcept
			:Matrix()
		{
			if (rhs.local())
			{
				resize(rhs.m_vecs, rhs.m_cmps);
				memcpy(m_data, rhs.m_data, sizeof(S) * size());
				m_transposed = rhs.m_transposed;
			}
			else
			{
				m_vecs = rhs.m_vecs;
				m_cmps = rhs.m_cmps;
				m_data = rhs.m_data;
				m_transposed = rhs.m_transposed;

				rhs.m_data = &rhs.m_buf[0];
				rhs.m_vecs = 0;
				rhs.m_cmps = 0;
				rhs.m_transposed = false;
			}
		}
		Matrix(Matrix const& rhs) noexcept
			:Matrix()
		{
			resize(rhs.m_vecs, rhs.m_cmps);
			memcpy(m_data, rhs.m_data, sizeof(S) * size());
			m_transposed = rhs.m_transposed;
		}
		explicit Matrix(Vec4<S> v) noexcept
			:Matrix(1, 4)
		{
			auto data = m_data;
			*data++ = S(v.x);
			*data++ = S(v.y);
			*data++ = S(v.z);
			*data++ = S(v.w);
		}
		explicit Matrix(Mat4x4<S> const& m) noexcept
			:Matrix(4, 4)
		{
			auto data = m_data;
			*data++ = S(m.x.x); *data++ = S(m.x.y); *data++ = S(m.x.z); *data++ = S(m.x.w);
			*data++ = S(m.y.x); *data++ = S(m.y.y); *data++ = S(m.y.z); *data++ = S(m.y.w);
			*data++ = S(m.z.x); *data++ = S(m.z.y); *data++ = S(m.z.z); *data++ = S(m.z.w);
			*data++ = S(m.w.x); *data++ = S(m.w.y); *data++ = S(m.w.z); *data++ = S(m.w.w);
		}
		~Matrix() noexcept
		{
			resize(0, 0);
		}

		// Assignment
		Matrix& operator = (Matrix&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			if (rhs.local())
			{
				resize(rhs.m_vecs, rhs.m_cmps);
				memcpy(m_data, rhs.m_data, sizeof(S) * size());
				m_transposed = rhs.m_transposed;
			}
			else
			{
				std::swap(m_vecs, rhs.m_vecs);
				std::swap(m_cmps, rhs.m_cmps);
				std::swap(m_data, rhs.m_data);
				std::swap(m_transposed, rhs.m_transposed);
			}
			return *this;
		}
		Matrix& operator = (Matrix const& rhs) noexcept
		{
			if (this == &rhs) return *this;
			resize(rhs.m_vecs, rhs.m_cmps);
			memcpy(m_data, rhs.m_data, sizeof(S) * size());
			return *this;
		}

		// Access this matrix as a 2D array
		S operator()(int vec, int cmp) const noexcept
		{
			if (m_transposed) std::swap(vec, cmp);
			pr_assert(vec >= 0 && vec < m_vecs);
			pr_assert(cmp >= 0 && cmp < m_cmps);
			return m_data[vec * m_cmps + cmp];
		}
		S& operator()(int vec, int cmp) noexcept
		{
			if (m_transposed) std::swap(vec, cmp);
			pr_assert(vec >= 0 && vec < m_vecs);
			pr_assert(cmp >= 0 && cmp < m_cmps);
			return m_data[vec * m_cmps + cmp];
		}

		// Access this matrix assuming is a 1xN or Nx1 vector
		S operator()(int cmp) const
		{
			if (vecs() == 1) return operator()(0, cmp);
			if (cmps() == 1) return operator()(cmp, 0);
			throw std::logic_error("Matrix is not a vector");
		}
		S& operator()(int cmp)
		{
			if (vecs() == 1) return operator()(0, cmp);
			if (cmps() == 1) return operator()(cmp, 0);
			throw std::logic_error("Matrix is not a vector");
		}

		// The number of vectors in the matrix (i.e. Y dimension, aka row count in row-major matrix)
		int vecs() const noexcept
		{
			return !m_transposed ? m_vecs : m_cmps;
		}

		// The number of components per vector in the matrix (i.e. X dimension, aka column count in row-major matrix)
		int cmps() const noexcept
		{
			return !m_transposed ? m_cmps : m_vecs;
		}

		// The total number of elements in the matrix
		int size() const noexcept
		{
			return m_vecs * m_cmps;
		}

		// Access to the linear underlying matrix data
		std::span<S const> data() const noexcept
		{
			return { m_data, static_cast<size_t>(size()) };
		}
		std::span<S> data() noexcept
		{
			return { m_data, static_cast<size_t>(size()) };
		}

		// Access this matrix by vector
		struct VecProxy
		{
			Matrix* m_mat;
			int     m_idx;

			VecProxy(Matrix& m, int idx) noexcept
				:m_mat(&m)
				,m_idx(idx)
			{
				pr_assert(idx >= 0 && idx < m_mat->vecs());
			}
			operator Matrix() const noexcept
			{
				Matrix v(1, m_mat->cmps());
				for (int i = 0, iend = m_mat->cmps(); i != iend; ++i)
					v(0, i) = (*m_mat)(m_idx, i);

				return v;
			}
			VecProxy& operator = (Matrix const& rhs) noexcept
			{
				pr_assert("'rhs' must be a vector" && rhs.vecs() == 1 && rhs.cmps() == m_mat->cmps());

				for (int i = 0, iend = m_mat->cmps(); i != iend; ++i)
					(*m_mat)(m_idx, i) = rhs(0, i);

				return *this;
			}
			S operator[](int i) const noexcept
			{
				return (*m_mat)(m_idx, i);
			}
			S& operator[](int i) noexcept
			{
				return (*m_mat)(m_idx, i);
			}
			std::span<S const> data() const noexcept
			{
				return { &(*m_mat)(m_idx, 0), static_cast<size_t>(m_mat->cmps()) };
			}
			std::span<S> data() noexcept
			{
				return { &(*m_mat)(m_idx, 0), static_cast<size_t>(m_mat->cmps()) };
			}
		};
		VecProxy vec(int i) const noexcept
		{
			return VecProxy(const_cast<Matrix&>(*this), i);
		}

		// Access this matrix by components (transposed vector)
		struct CmpProxy
		{
			Matrix* m_mat;
			int     m_idx;

			CmpProxy(Matrix& m, int idx) noexcept
				:m_mat(&m)
				,m_idx(idx)
			{
				pr_assert(idx >= 0 && idx < m_mat->cmps());
			}
			operator Matrix() const noexcept
			{
				Matrix v(m_mat->vecs(), 1);
				for (int i = 0, iend = m_mat->vecs(); i != iend; ++i)
					v(i, 0) = (*m_mat)(i, m_idx);

				return v;
			}
			CmpProxy& operator = (Matrix const& rhs) noexcept
			{
				pr_assert("'rhs' must be a transposed vector" && rhs.cmps() == 1 && rhs.vecs() == m_mat->vecs());

				for (int i = 0, iend = m_mat->vecs(); i != iend; ++i)
					(*m_mat)(i, m_idx) = rhs(i, 0);

				return *this;
			}
			S operator [](int i) const noexcept
			{
				return (*m_mat)(i, m_idx);
			}
			S& operator [](int i) noexcept
			{
				return (*m_mat)(i, m_idx);
			}
		};
		CmpProxy cmp(int i) const noexcept
		{
			return CmpProxy(const_cast<Matrix&>(*this), i);
		}

		// True if the matrix is square
		bool IsSquare() const noexcept
		{
			return m_vecs == m_cmps;
		}

		// Set this matrix to all zeros
		Matrix& zero() noexcept
		{
			memset(m_data, 0, sizeof(S) * size());
			return *this;
		}

		// Set this matrix to all 'value'
		Matrix& fill(S value) noexcept
		{
			std::fill(m_data, m_data + size(), value);
			return *this;
		}

		// Set this matrix to an identity matrix
		Matrix& identity() noexcept
		{
			zero();
			for (int i = 0, istep = cmps() + 1, iend = size(); i < iend; i += istep) m_data[i] = S(1);
			return *this;
		}

		// Transpose this matrix
		Matrix& transpose() noexcept
		{
			m_transposed = !m_transposed;
			return *this;
		}

		// True if the data of this matrix is locally buffered
		bool local() const noexcept
		{
			return m_data == &m_buf[0];
		}

		// Change the dimensions of the matrix.
		void resize(int vecs, int cmps, bool preserve_data = true) noexcept
		{
			if (m_transposed) std::swap(vecs, cmps);

			// Check if a resize is needed
			auto new_count = size_t(vecs * cmps);
			auto old_count = size_t(m_vecs * m_cmps);
			auto min_count = std::min(new_count, old_count);
			auto min_vecs = std::min(vecs, m_vecs);
			auto min_cmps = std::min(cmps, m_cmps);
			
			// Slocate buffer if necessary.
			auto data =
				new_count == old_count ? m_data :
				new_count > LocalBufCount ? new S[new_count] :
				&m_buf[0];

			// Copy/Initialise data
			if (!preserve_data)
			{
				// Initialise to zeros
				memset(data, 0, sizeof(S) * new_count);
			}
			else if (cmps == m_cmps)
			{
				// Matrix elements are stored as contiguous vectors so adding/removing vectors does not invalidate existing data.
				if (data != m_data) memcpy(data, m_data, sizeof(S) * min_count);
				memset(data + min_count, 0, sizeof(S) * (new_count - min_count));
			}
			else
			{
				// Adding components requires copying per vector.
				memset(data, 0, sizeof(S) * new_count);
				for (int i = 0; i != min_vecs; ++i)
					memcpy(data + i*cmps, m_data + i*m_cmps, sizeof(S) * min_cmps);
			}

			// Update the members
			m_vecs = vecs;
			m_cmps = cmps;
			std::swap(m_data, data);

			// Deallocate the old buffer
			if (data != m_data && data != &m_buf[0])
				delete[] data;
		}

		// Change the number of vectors in the matrix.
		void resize(int size, bool preserve_data = true) noexcept
		{
			resize(size, cmps(), preserve_data);
		}

		// Return an identity matrix of the given dimensions
		static Matrix Zero(int vecs, int cmps) noexcept
		{
			Matrix m(vecs, cmps);
			return m.zero();
		}

		// Return an identity matrix of the given dimensions
		static Matrix Fill(int vecs, int cmps, S value) noexcept
		{
			Matrix m(vecs, cmps);
			return m.fill(value);
		}

		// Return an identity matrix of the given dimensions
		static Matrix Identity(int vecs, int cmps) noexcept
		{
			Matrix m(vecs, cmps);
			return m.identity();
		}

		// Generates the random matrix
		template <typename Rng = std::default_random_engine> static Matrix Random(Rng& rng, int vecs, int cmps, S min_value, S max_value) noexcept
		{
			std::uniform_real_distribution<S> dist(min_value, max_value);

			Matrix<S> m(vecs, cmps);
			for (int i = 0, iend = m.size(); i != iend; ++i)
				m.data()[i] = dist(rng);

			return m;
		}

		#pragma region Operators

		// Unary operators
		friend Matrix<S> operator + (Matrix<S> const& m) noexcept
		{
			return m;
		}
		friend Matrix<S> operator - (Matrix<S> const& m) noexcept
		{
			// If 'm' is transposed, return a transposed matrix for consistency
			Matrix<S> res(m.m_vecs, m.m_cmps);
			res.m_transposed = m.m_transposed;
			for (int i = 0, iend = res.size(); i != iend; ++i)
				res.m_data[i] = -m.m_data[i];
		
			return res;
		}

		// Addition/Subtraction
		friend Matrix<S> operator + (Matrix<S> const& lhs, Matrix<S> const& rhs) noexcept
		{
			pr_assert(lhs.vecs() == rhs.vecs());
			pr_assert(lhs.cmps() == rhs.cmps());

			// If one of the matrices is transposed, we need to add element-by-element
			// If both are transposed (or not), we can vectorise element adding
			// Return a matrix consist with the common transposed state
			if (lhs.m_transposed == rhs.m_transposed)
			{
				// If both are not transposed, we can vectorise element adding
				Matrix<S> res(lhs.m_vecs, lhs.m_cmps);
				res.m_transposed = lhs.m_transposed;
				for (int i = 0, iend = res.size(); i != iend; ++i)
					res.m_data[i] = lhs.m_data[i] + rhs.m_data[i];
				return res;
			}
			else
			{
				// If one of the matrices is transposed, we need to add element-by-element
				Matrix<S> res(lhs.vecs(), lhs.cmps());
				for (int r = 0; r != res.vecs(); ++r)
					for (int c = 0; c != res.cmps(); ++c)
						res(r, c) = lhs(r, c) + rhs(r, c);
				return res;
			}
		}
		friend Matrix<S> operator - (Matrix<S> const& lhs, Matrix<S> const& rhs) noexcept
		{
			pr_assert(lhs.vecs() == rhs.vecs());
			pr_assert(lhs.cmps() == rhs.cmps());

			// If one of the matrices is transposed, we need to subtract element-by-element
			// If both are transposed (or not), we can vectorise element substracting
			// Return a matrix consist with the common transposed state
			if (lhs.m_transposed != rhs.m_transposed)
			{
				// If both are not transposed, we can vectorise element subtracting
				Matrix<S> res(lhs.m_vecs, lhs.m_cmps);
				res.m_transposed = lhs.m_transposed;
				for (int i = 0, iend = res.size(); i != iend; ++i)
					res.m_data[i] = lhs.m_data[i] - rhs.m_data[i];
				return res;
			}
			else
			{
				// If one of the matrices is transposed, we need to subtract element-by-element
				Matrix<S> res(lhs.vecs(), lhs.cmps());
				for (int r = 0; r != res.vecs(); ++r)
					for (int c = 0; c != res.cmps(); ++c)
						res(r, c) = lhs(r, c) - rhs(r, c);
				return res;
			}
		}

		// Multiplication
		friend Matrix<S> operator * (S s, Matrix<S> const& mat) noexcept
		{
			// Preserve the transposed state in the returned matrix
			Matrix<S> res(mat.m_vecs, mat.m_cmps);
			res.m_transposed = mat.m_transposed;
			for (int i = 0, iend = res.size(); i != iend; ++i)
				res.m_data[i] = mat.m_data[i] * s;

			return res;
		}
		friend Matrix<S> operator * (Matrix<S> const& mat, S s) noexcept
		{
			return s * mat;
		}
		friend Matrix<S> operator * (Matrix<S> const& b2c, Matrix<S> const& a2b) noexcept
		{
			// Note:
			//  - The multplication order is the same as for m4x4.
			//    The reason for this order is because matrices are applied from right to left
			//    e.g.
			//       auto Va =             V = vector in space 'a'
			//       auto Vb =       a2b * V = vector in space 'b'
			//       auto Vc = b2c * a2b * V = vector in space 'c'
			//  - The shape of the result is:
			//       [   ]       [       ]       [   ]
			//       [a2c]       [  b2c  ]       [a2b]
			//       [1x3]   =   [  2x3  ]   *   [1x2]
			//       [   ]       [       ]       [   ]
			pr_assert("Wrong matrix dimensions" && a2b.cmps() == b2c.vecs());

			// Result
			Matrix res(a2b.vecs(), b2c.cmps());

			auto msize = std::max(std::max(a2b.vecs(), a2b.cmps()), std::max(b2c.vecs(), b2c.cmps()));
			if (msize < 32)
			{
				// Small matrix multiply
				for (int r = 0; r != res.vecs(); ++r)
					for (int c = 0; c != res.cmps(); ++c)
						for (int k = 0; k != a2b.cmps(); ++k)
							res(r, c) += a2b(r, k) * b2c(k, c);
				
				return res;
			}

			// 'Strassen Multiply'
			int n = 0;
			int size = 1;
			while (msize > size) { size *= 2; n++; };
			int h = size / 2;

			// Temporary smaller square matrices
			const int M = 9;
			std::vector<Matrix> buf(n * M);
			
			//  8x8, 8x8, 8x8, ...
			//  4x4, 4x4, 4x4, ...
			//  2x2, 2x2, 2x2, ...
			//  . . .
			auto field = &buf[0];
			for (int i = 0; i < n - 4; i++)
			{
				auto z = (int)pow(2, n - i - 1);
				for (int j = 0; j < M; j++)
					field[i*M+j] = Matrix(z, z);
			}

			#pragma region Sub Functions
			struct L
			{
				static void SafeAplusBintoC(Matrix const& A, int xa, int ya, Matrix const& B, int xb, int yb, Matrix& C, int sz) noexcept
				{
					for (int r = 0; r < sz; ++r) // rows
					{
						for (int c = 0; c < sz; ++c) // cols
						{
							C(r, c) = 0;
							if (xa + c < A.cmps() && ya + r < A.vecs()) C(r, c) += A(ya + r, xa + c);
							if (xb + c < B.cmps() && yb + r < B.vecs()) C(r, c) += B(yb + r, xb + c);
						}
					}
				}
				static void SafeAminusBintoC(Matrix const& A, int xa, int ya, Matrix const& B, int xb, int yb, Matrix& C, int sz) noexcept
				{
					for (int r = 0; r < sz; ++r) // rows
					{
						for (int c = 0; c < sz; ++c) // cols
						{
							C(r, c) = 0;
							if (xa + c < A.cmps() && ya + r < A.vecs()) C(r, c) += A(ya + r, xa + c);
							if (xb + c < B.cmps() && yb + r < B.vecs()) C(r, c) -= B(yb + r, xb + c);
						}
					}
				}
				static void SafeACopytoC(Matrix const& A, int xa, int ya, Matrix& C, int sz) noexcept
				{
					for (int r = 0; r < sz; ++r) // rows
					{
						for (int c = 0; c < sz; ++c) // cols
						{
							C(r, c) = 0;
							if (xa + c < A.cmps() && ya + r < A.vecs()) C(r, c) += A(ya + r, xa + c);
						}
					}
				}
				static void AplusBintoC(Matrix const& A, int xa, int ya, Matrix const& B, int xb, int yb, Matrix& C, int sz) noexcept
				{
					for (int r = 0; r < sz; ++r) // rows
						for (int c = 0; c < sz; ++c)
							C(r, c) = A(ya + r, xa + c) + B(yb + r, xb + c);
				}
				static void AminusBintoC(Matrix const& A, int xa, int ya, Matrix const& B, int xb, int yb, Matrix& C, int sz) noexcept
				{
					for (int r = 0; r < sz; ++r) // rows
						for (int c = 0; c < sz; ++c)
							C(r, c) = A(ya + r, xa + c) - B(yb + r, xb + c);
				}
				static void ACopytoC(Matrix const& A, int xa, int ya, Matrix& C, int sz) noexcept
				{
					for (int r = 0; r < sz; ++r) // rows
						for (int c = 0; c < sz; ++c)
							C(r, c) = A(ya + r, xa + c);
				}
				static void StrassenMultiplyRun(Matrix const& A, Matrix const& B, Matrix& C, int l, Matrix* f) noexcept
				{
					// A * B into C, level of recursion, matrix field
					// function for square matrix 2^N x 2^N
					auto sz = A.vecs();
					if (sz < 32)
					{
						for (int r = 0; r < C.vecs(); ++r)
						{
							for (int c = 0; c < C.cmps(); ++c)
							{
								C(r, c) = 0;
								for (int k = 0; k < A.cmps(); ++k)
									C(r, c) += A(r, k) * B(k, c);
							}
						}
					}
					else
					{
						auto hh = sz / 2;
						AplusBintoC(A, 0, 0, A, hh, hh, f[l*M + 0], hh);
						AplusBintoC(B, 0, 0, B, hh, hh, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 1], l + 1, f); // (A11 + A22) * (B11 + B22);

						AplusBintoC(A, 0, hh, A, hh, hh, f[l*M + 0], hh);
						ACopytoC(B, 0, 0, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 2], l + 1, f); // (A21 + A22) * B11;

						ACopytoC(A, 0, 0, f[l*M + 0], hh);
						AminusBintoC(B, hh, 0, B, hh, hh, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 3], l + 1, f); //A11 * (B12 - B22);

						ACopytoC(A, hh, hh, f[l*M + 0], hh);
						AminusBintoC(B, 0, hh, B, 0, 0, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 4], l + 1, f); //A22 * (B21 - B11);

						AplusBintoC(A, 0, 0, A, hh, 0, f[l*M + 0], hh);
						ACopytoC(B, hh, hh, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 5], l + 1, f); //(A11 + A12) * B22;

						AminusBintoC(A, 0, hh, A, 0, 0, f[l*M + 0], hh);
						AplusBintoC(B, 0, 0, B, hh, 0, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 6], l + 1, f); //(A21 - A11) * (B11 + B12);

						AminusBintoC(A, hh, 0, A, hh, hh, f[l*M + 0], hh);
						AplusBintoC(B, 0, hh, B, hh, hh, f[l*M + 1], hh);
						StrassenMultiplyRun(f[l*M + 0], f[l*M + 1], f[l*M + 1 + 7], l + 1, f); // (A12 - A22) * (B21 + B22);

						// C11
						for (int r = 0; r < hh; r++) // rows
							for (int c = 0; c < hh; c++) // cols
								C(r, c) = f[l*M + 1 + 1](r, c) + f[l*M + 1 + 4](r, c) - f[l*M + 1 + 5](r, c) + f[l*M + 1 + 7](r, c);

						// C12
						for (int r = 0; r < hh; r++) // rows
							for (int c = hh; c < sz; c++) // cols
								C(r, c) = f[l*M + 1 + 3](r, c - hh) + f[l*M + 1 + 5](r, c - hh);

						// C21
						for (int r = hh; r < sz; r++) // rows
							for (int c = 0; c < hh; c++) // cols
								C(r, c) = f[l*M + 1 + 2](r - hh, c) + f[l*M + 1 + 4](r - hh, c);

						// C22
						for (int r = hh; r < sz; r++) // rows
							for (int c = hh; c < sz; c++) // cols
								C(r, c) = f[l*M + 1 + 1](r - hh, c - hh) - f[l*M + 1 + 2](r - hh, c - hh) + f[l*M + 1 + 3](r - hh, c - hh) + f[l*M + 1 + 6](r - hh, c - hh);
					}
				}
			};
			#pragma endregion

			#pragma region Strassen Multiply

			L::SafeAplusBintoC(a2b, 0, 0, a2b, h, h, field[0*M + 0], h);
			L::SafeAplusBintoC(b2c, 0, 0, b2c, h, h, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 1], 1, field); // (A11 + A22) * (B11 + B22);

			L::SafeAplusBintoC(a2b, 0, h, a2b, h, h, field[0*M + 0], h);
			L::SafeACopytoC(b2c, 0, 0, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 2], 1, field); // (A21 + A22) * B11;

			L::SafeACopytoC(a2b, 0, 0, field[0*M + 0], h);
			L::SafeAminusBintoC(b2c, h, 0, b2c, h, h, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 3], 1, field); //A11 * (B12 - B22);

			L::SafeACopytoC(a2b, h, h, field[0*M + 0], h);
			L::SafeAminusBintoC(b2c, 0, h, b2c, 0, 0, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 4], 1, field); //A22 * (B21 - B11);

			L::SafeAplusBintoC(a2b, 0, 0, a2b, h, 0, field[0*M + 0], h);
			L::SafeACopytoC(b2c, h, h, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 5], 1, field); //(A11 + A12) * B22;

			L::SafeAminusBintoC(a2b, 0, h, a2b, 0, 0, field[0*M + 0], h);
			L::SafeAplusBintoC(b2c, 0, 0, b2c, h, 0, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 6], 1, field); //(A21 - A11) * (B11 + B12);

			L::SafeAminusBintoC(a2b, h, 0, a2b, h, h, field[0*M + 0], h);
			L::SafeAplusBintoC(b2c, 0, h, b2c, h, h, field[0*M + 1], h);
			L::StrassenMultiplyRun(field[0*M + 0], field[0*M + 1], field[0*M + 1 + 7], 1, field); // (A12 - A22) * (B21 + B22);

			// C11
			for (int r = 0; r < std::min(h, res.vecs()); r++) // rows
				for (int c = 0; c < std::min(h, res.cmps()); c++) // cols
					res(r, c) = field[0*M + 1 + 1](r, c) + field[0*M + 1 + 4](r, c) - field[0*M + 1 + 5](r, c) + field[0*M + 1 + 7](r, c);

			// C12
			for (int r = 0; r < std::min(h, res.vecs()); r++) // rows
				for (int c = h; c < std::min(2 * h, res.cmps()); c++) // cols
					res(r, c) = field[0*M + 1 + 3](r, c - h) + field[0*M + 1 + 5](r, c - h);

			// C21
			for (int r = h; r < std::min(2 * h, res.vecs()); r++) // rows
				for (int c = 0; c < std::min(h, res.cmps()); c++) // cols
					res(r, c) = field[0*M + 1 + 2](r - h, c) + field[0*M + 1 + 4](r - h, c);

			// C22
			for (int r = h; r < std::min(2 * h, res.vecs()); r++) // rows
				for (int c = h; c < std::min(2 * h, res.cmps()); c++) // cols
					res(r, c) = field[0*M + 1 + 1](r - h, c - h) - field[0*M + 1 + 2](r - h, c - h) + field[0*M + 1 + 3](r - h, c - h) + field[0*M + 1 + 6](r - h, c - h);

			#pragma endregion 

			return res;
		}
		
		// Equals
		friend bool operator == (Matrix<S> const& lhs, Matrix<S> const& rhs) noexcept
		{
			if (lhs.vecs() != rhs.vecs()) return false;
			if (lhs.cmps() != rhs.cmps()) return false;
			if (lhs.m_transposed == rhs.m_transposed)
				return memcmp(lhs.data(), rhs.data(), sizeof(S) * lhs.size()) == 0;

			// Fall back to element-by-element comparisons
			for (int r = 0, rend = lhs.vecs(); r != rend; ++r)
				for (int c = 0, cend = lhs.cmps(); c != cend; ++c)
					if (lhs(r, c) != rhs(r, c))
						return false;

			return true;
		}
		friend bool operator != (Matrix<S> const& lhs, Matrix<S> const& rhs) noexcept
		{
			return !(lhs == rhs);
		}

		// Value equality
		friend bool FEqlAbsolute(Matrix<S> const& lhs, Matrix<S> const& rhs, S tol) noexcept
		{
			if (lhs.vecs() != rhs.vecs()) return false;
			if (lhs.cmps() != rhs.cmps()) return false;
			if (lhs.m_transposed == rhs.m_transposed)
			{
				// Vectorise compares
				for (int i = 0, iend = lhs.size(); i != iend; ++i)
					if (!FEqlAbsolute(lhs.m_data[i], rhs.m_data[i], tol))
						return false;
			}
			else
			{
				// Element-by-element compares
				for (int r = 0, rend = lhs.vecs(); r != rend; ++r)
					for (int c = 0, cend = lhs.cmps(); c != cend; ++c)
						if (!FEqlAbsolute(lhs(r, c), rhs(r, c), tol))
							return false;
			}

			return true;
		}
		friend bool FEqlRelative(Matrix<S> const& lhs, Matrix<S> const& rhs, S tol) noexcept
		{
			if (lhs.vecs() != rhs.vecs()) return false;
			if (lhs.cmps() != rhs.cmps()) return false;
			if (lhs.m_transposed == rhs.m_transposed)
			{
				// Vectorise compares
				for (int i = 0, iend = lhs.size(); i != iend; ++i)
					if (!FEqlRelative(lhs.m_data[i], rhs.m_data[i], tol))
						return false;
			}
			else
			{
				// Element-by-element compares
				for (int r = 0, rend = lhs.vecs(); r != rend; ++r)
					for (int c = 0, cend = lhs.cmps(); c != cend; ++c)
						if (!FEqlRelative(lhs(r, c), rhs(r, c), tol))
							return false;
			}

			return true;
		}
		friend bool FEql(Matrix<S> const& lhs, Matrix<S> const& rhs) noexcept
		{
			return FEqlRelative(lhs, rhs, tiny<S>);
		}
		friend bool FEqlAbsolute(Matrix<S> const& lhs, Mat4x4<S> const& rhs, float tol) noexcept
		{
			if (lhs.vecs() != 4) return false;
			if (lhs.cmps() != 4) return false;
			return
				FEqlAbsolute(float(lhs(0, 0)), rhs.x.x, tol) && FEqlAbsolute(float(lhs(0, 1)), rhs.x.y, tol) && FEqlAbsolute(float(lhs(0, 2)), rhs.x.z, tol) && FEqlAbsolute(float(lhs(0, 3)), rhs.x.w, tol) &&
				FEqlAbsolute(float(lhs(1, 0)), rhs.y.x, tol) && FEqlAbsolute(float(lhs(1, 1)), rhs.y.y, tol) && FEqlAbsolute(float(lhs(1, 2)), rhs.y.z, tol) && FEqlAbsolute(float(lhs(1, 3)), rhs.y.w, tol) &&
				FEqlAbsolute(float(lhs(2, 0)), rhs.z.x, tol) && FEqlAbsolute(float(lhs(2, 1)), rhs.z.y, tol) && FEqlAbsolute(float(lhs(2, 2)), rhs.z.z, tol) && FEqlAbsolute(float(lhs(2, 3)), rhs.z.w, tol) &&
				FEqlAbsolute(float(lhs(3, 0)), rhs.w.x, tol) && FEqlAbsolute(float(lhs(3, 1)), rhs.w.y, tol) && FEqlAbsolute(float(lhs(3, 2)), rhs.w.z, tol) && FEqlAbsolute(float(lhs(3, 3)), rhs.w.w, tol);
		}
		friend bool FEqlRelative(Matrix<S> const& lhs, Mat4x4<S> const& rhs, float tol) noexcept
		{
			if (lhs.vecs() != 4) return false;
			if (lhs.cmps() != 4) return false;
			return
				FEqlRelative(float(lhs(0, 0)), rhs.x.x, tol) && FEqlRelative(float(lhs(0, 1)), rhs.x.y, tol) && FEqlRelative(float(lhs(0, 2)), rhs.x.z, tol) && FEqlRelative(float(lhs(0, 3)), rhs.x.w, tol) &&
				FEqlRelative(float(lhs(1, 0)), rhs.y.x, tol) && FEqlRelative(float(lhs(1, 1)), rhs.y.y, tol) && FEqlRelative(float(lhs(1, 2)), rhs.y.z, tol) && FEqlRelative(float(lhs(1, 3)), rhs.y.w, tol) &&
				FEqlRelative(float(lhs(2, 0)), rhs.z.x, tol) && FEqlRelative(float(lhs(2, 1)), rhs.z.y, tol) && FEqlRelative(float(lhs(2, 2)), rhs.z.z, tol) && FEqlRelative(float(lhs(2, 3)), rhs.z.w, tol) &&
				FEqlRelative(float(lhs(3, 0)), rhs.w.x, tol) && FEqlRelative(float(lhs(3, 1)), rhs.w.y, tol) && FEqlRelative(float(lhs(3, 2)), rhs.w.z, tol) && FEqlRelative(float(lhs(3, 3)), rhs.w.w, tol);
		}
		friend bool FEql(Matrix<S> const& lhs, Mat4x4<S> const& rhs) noexcept
		{
			return FEqlRelative(lhs, rhs, tiny<S>);
		}
		friend bool FEqlAbsolute(Matrix<S> const& lhs, Vec4<S> rhs, float tol) noexcept
		{
			if (lhs.vecs() != 1 && lhs.cmps() != 1) return false;
			if (lhs.size() != 4) return false;
			return
				FEqlAbsolute(float(lhs.m_data[0]), rhs.x, tol) &&
				FEqlAbsolute(float(lhs.m_data[1]), rhs.y, tol) &&
				FEqlAbsolute(float(lhs.m_data[2]), rhs.z, tol) &&
				FEqlAbsolute(float(lhs.m_data[3]), rhs.w, tol);
		}
		friend bool FEqlRelative(Matrix<S> const& lhs, Vec4<S> rhs, float tol) noexcept
		{
			if (lhs.vecs() != 1 && lhs.cmps() != 1) return false;
			if (lhs.size() != 4) return false;
			return
				FEqlRelative(float(lhs.m_data[0]), rhs.x, tol) &&
				FEqlRelative(float(lhs.m_data[1]), rhs.y, tol) &&
				FEqlRelative(float(lhs.m_data[2]), rhs.z, tol) &&
				FEqlRelative(float(lhs.m_data[3]), rhs.w, tol);
		}
		friend bool FEql(Matrix<S> const& lhs, Vec4<S> rhs) noexcept
		{
			return FEqlRelative(lhs, rhs, tiny<S>);
		}

		#pragma endregion
	};

	// The LU decomposition of a square matrix
	template <ScalarType S>
	struct MatrixLU
	{
		struct LProxy
		{
			Matrix<S> const* m_mat;
			LProxy(Matrix<S> const& m) noexcept
				:m_mat(&m)
			{}
			S operator ()(int vec, int cmp) const noexcept
			{
				pr_assert(vec >= 0 && vec < m_mat->vecs());
				pr_assert(cmp >= 0 && cmp < m_mat->cmps());
				return cmp > vec ? (*m_mat)(vec, cmp) : cmp == vec ? 1 : 0;
			}
		};
		struct UProxy
		{
			Matrix<S> const* m_mat;
			UProxy(Matrix<S> const& m) noexcept
				:m_mat(&m)
			{}
			S operator ()(int vec, int cmp) const noexcept
			{
				pr_assert(vec >= 0 && vec < m_mat->vecs());
				pr_assert(cmp >= 0 && cmp < m_mat->cmps());
				return cmp <= vec ? (*m_mat)(vec, cmp) : 0;
			}
		};
	
		// The L and U matrices are stored in one matrix.
		Matrix<S> lu;

		// Access to the lower diagonal matrix
		LProxy L;

		// Access to the upper diagonal matrix
		UProxy U;

		// The permutation row indices (length == rows())
		std::unique_ptr<int[]> pi;

		// The determinant of the permutation matrix
		S DetOfP;

		MatrixLU(int vecs, int cmps, std::initializer_list<S> data, bool transposed = false)
			:MatrixLU(Matrix<S>(vecs, cmps, data, transposed))
		{}
		MatrixLU(Matrix<S> const& m)
			:lu(m)
			,L(lu)
			,U(lu)
			,pi(new int[m.vecs()])
			,DetOfP(1.0f)
		{
			auto const N = m.IsSquare() ? m.vecs() : throw std::logic_error("LU decomposition is only possible on square matrices");

			// We will store both the L and U matrices in 'base' since we know
			// L has the form: [1 0] and U has the form: [U U]
			//                 [L 1]                     [0 U]
			auto  LL = Matrix<S>::Identity(m.vecs(), m.cmps());
			auto& UU = lu;

			// Initialise the permutation vector
			for (int i = 0; i != N; ++i)
				pi[i] = i;

			// Decompose 'm' into 'LL' and 'UU'
			for (int v = 0; v != N; ++v)
			{
				// Pivoting is used to avoid instability when the pivot is ~0
				// It will probably always be enabled, but this documents it.
				const bool UsePivot = true;
				if constexpr (UsePivot)
				{
					// Find the largest component in the vector 'v' to use as the pivot
					auto p = 0;
					S max = 0;
					for (int i = v; i != N; ++i)
					{
						auto val = abs(UU(v, i));
						if (val <= max) continue;
						max = val;
						p = i;
					}
					if (max == 0)
						throw std::runtime_error("The matrix is singular");

					// Switch the components of all vectors
					if (p != v)
					{
						std::swap(pi[v], pi[p]);
						DetOfP = -DetOfP;
				
						// Switch the components in LL and UU
						for (int i = 0; i != v; ++i)
							std::swap(LL(i, v), LL(i, p));
						for (int i = 0; i != N; ++i)
							std::swap(UU(i, v), UU(i, p));
					}
				}

				// Gaussian eliminate the remaining components of vector 'v'
				for (int c = v + 1; c != N; ++c)
				{
					LL(v, c) = UU(v, c) / UU(v, v);
					for (int i = v; i != N; ++i)
						UU(i, c) -= LL(v, c) * UU(i, v);
				}
			}

			// Combine 'LL' and 'UU' into 'LU' (note UU *is* lu)
			for (int v = 0; v != N; ++v)
				for (int c = v+1; c != N; ++c)
					lu(v, c) = LL(v, c);
		}

		// The matrix dimension (square)
		int dim() const noexcept
		{
			return lu.vecs();
		}

		// Access to the linear underlying matrix data
		S const* data() const noexcept
		{
			return lu.data();
		}
		
		// Access this matrix as a 2D array
		S operator()(bool change, int vec, int cmp) const noexcept
		{
			return lu(vec, cmp);
		}
	};

	// Result of eigenvalue decomposition
	template <typename S>
	struct EigenResult
	{
		Matrix<S> values;  // 1×N row vector of eigenvalues, sorted descending. Access as values(0, i).
		Matrix<S> vectors; // N×N (or N×k) matrix where column i is the eigenvector for values(0, i). Access component r of eigenvector i as vectors(r, i).
	};

	// Return the transpose of matrix 'm'
	template <typename S> inline Matrix<S> Transpose(Matrix<S> const& m) noexcept
	{
		return Matrix<S>(m).transpose();
	}

	// Return the determinant of a matrix
	template <typename S> inline S Determinant(MatrixLU<S> const& m) noexcept
	{
		auto det = m.DetOfP;
		for (int i = 0; i != m.dim(); ++i)
			det *= m.U(i, i);

		return det;
	}
	template <typename S> inline S Determinant(Matrix<S> const& m) noexcept
	{
		return Determinant(MatrixLU<S>(m));
	}

	// Return the dot product of two vectors
	template <typename S> inline S Dot(Matrix<S> const& lhs, Matrix<S> const& rhs) noexcept
	{
		pr_assert("Dot product is between column vectors" && lhs.vecs() == 1 && rhs.vecs() == 1);
		pr_assert("Dot product must be between vectors of the same length" && lhs.cmps() == rhs.cmps());

		S dp = 0;
		for (int i = 0, iend = lhs.cmps(); i != iend; ++i)
			dp += lhs(0, i) * rhs(0, i);
		return dp;
	}

	// True if 'm' has an inverse
	template <typename S> inline bool IsInvertible(MatrixLU<S> const& m) noexcept
	{
		return Determinant(m) != 0;
	}
	template <typename S> inline bool IsInvertible(Matrix<S> const& m) noexcept
	{
		return IsInvertible(MatrixLU<S>(m));
	}

	// Solves for 'x' in 'Ax = v'
	template <typename S> inline Matrix<S> Solve(MatrixLU<S> const& A, Matrix<S> const& v) noexcept
	{
		// e.g. [4x4][1x4] = [1x4]
		pr_assert("Solution vector 'v' has the wrong dimensions" && A.dim() == v.cmps() && v.vecs() == 1);

		// Switch items in 'v' due to permutation matrix
		Matrix<S> a(1, A.dim());
		for (int i = 0; i != A.dim(); ++i)
			a(0, i) = v(0, A.pi[i]);

		// Solve for x in 'L.x = b' assuming 'L' is a lower triangular matrix
		Matrix<S> b(1, A.dim());
		for (int i = 0; i != A.dim(); ++i)
		{
			b(0, i) = a(0, i);
			for (int j = 0; j != i; ++j)
				b(0, i) -= A.L(j, i) * b(0, j);
		}

		// Solve for x in 'Ux = b' assuming 'U' is an upper triangular matrix
		Matrix<S> c(b);
		for (int i = A.dim(); i-- != 0;)
		{
			b(0, i) = c(0, i);
			for (int j = A.dim() - 1; j > i; --j)
				b(0, i) -= A.U(j, i) * b(0, j);

			b(0, i) = b(0, i) / A.U(i, i);
		}

		return b;
	}

	// Solves for 'x' in 'Ax = v'
	template <typename S> inline Matrix<S> Solve(Matrix<S> const& A, Matrix<S> const& v)
	{
		if (!A.IsSquare()) throw std::exception("The matrix is not square");
		return Solve(MatrixLU<S>(A), v);
	}

	// Return the inverse of matrix 'm'
	template <typename S> inline Matrix<S> Invert(MatrixLU<S> const& lu) noexcept
	{
		pr_assert("Matrix has no inverse" && IsInvertible(lu));

		// Inverse of an NxM matrix is a MxN matrix (even though this only works for square matrices)
		Matrix<S> inv(lu.dim(), lu.dim());
		Matrix<S> elem(1, lu.dim());
		for (int i = 0; i != lu.dim(); ++i)
		{
			elem(0, i) = 1;
			inv.vec(i) = Solve(lu, elem);
			elem(0, i) = 0;
		}
		return inv;
	}
	template <typename S> inline Matrix<S> Invert(Matrix<S> const& m) noexcept
	{
		return Invert(MatrixLU<S>(m));
	}

	// Matrix to the power 'pow'
	template <typename S> inline Matrix<S> Power(Matrix<S> const& m, int pow) noexcept
	{
		if (pow == +1) return m;
		if (pow ==  0) return Matrix<S>::Identity(m.vecs(), m.cmps());
		if (pow == -1) return Invert(m);

		auto x = pow < 0 ? Invert(m) : m;
		pow = pow < 0 ? -pow : pow;

		auto ret = Matrix<S>::Identity(m.vecs(), m.cmps());
		while (pow != 0)
		{
			if ((pow & 1) == 1) ret *= x;
			x *= x;
			pow >>= 1;
		}
		return ret;
	}

	// Householder tridiagonalization: Q^T * A * Q = T
	// Matrix convention: A(i,j) = A[row i][col j] in standard notation.
	template <typename S> inline void Tridiagonalize(Matrix<S> const& m, Matrix<S>& diag, Matrix<S>& sub, Matrix<S>& Q) noexcept
	{
		auto const N = m.vecs();
		pr_assert(diag.vecs() == 1 && diag.cmps() == N);
		pr_assert(sub.vecs() == 1 && sub.cmps() >= N);
		pr_assert(Q.vecs() == N && Q.cmps() == N);

		auto A = Matrix<S>(m);
		auto v = Matrix<S>();
		auto p = Matrix<S>();
		auto kk = Matrix<S>();
		auto w = Matrix<S>();

		for (int k = 0; k != N - 2; ++k)
		{
			// Build Householder vector to zero out A[k+2:N-1][k] (column k, below sub-diagonal)
			auto sigma = S(0);
			for (int i = k + 2; i != N; ++i)
				sigma += A(i, k) * A(i, k);

			if (sigma < std::numeric_limits<S>::epsilon() * std::numeric_limits<S>::epsilon())
				continue;

			auto alpha = A(k + 1, k);
			auto norm = std::sqrt(alpha * alpha + sigma);
			auto beta = (alpha >= 0) ? alpha + norm : alpha - norm;

			// v = [1, A[k+2][k]/beta, ..., A[N-1][k]/beta]
			v.resize(1, N - k - 1, false);
			v(0) = S(1);
			for (int i = 1; i != N - k - 1; ++i)
				v(i) = A(k + 1 + i, k) / beta;

			auto tau = S(2) / Dot(v, v);

			// p = tau * A_sub * v, where A_sub = A[k+1:N-1, k+1:N-1]
			p.resize(1, N - k - 1, false);
			for (int i = 0; i != N - k - 1; ++i)
				for (int j = 0; j != N - k - 1; ++j)
					p(i) += A(k + 1 + i, k + 1 + j) * v(j);
			for (auto& pi : p.data())
				pi *= tau;

			// kk = p - (tau/2)*(p·v)*v
			auto pv = Dot(p, v);
			kk.resize(1, N - k - 1, false);//auto kk = std::vector<S>(N - k - 1);
			for (int i = 0; i != N - k - 1; ++i)
				kk(i) = p(0,i) - (tau / S(2)) * pv * v(i);

			// Update A_sub: A_sub[i][j] -= v[i]*kk[j] + kk[i]*v[j]
			for (int i = 0; i != N - k - 1; ++i)
				for (int j = 0; j != N - k - 1; ++j)
					A(k + 1 + i, k + 1 + j) -= v(i) * kk(j) + kk(i) * v(j);

			// Set the sub-diagonal element
			A(k + 1, k) = -(alpha >= 0 ? S(1) : S(-1)) * norm;
			A(k, k + 1) = A(k + 1, k);
			for (int i = k + 2; i != N; ++i)
			{
				A(i, k) = S(0);
				A(k, i) = S(0);
			}

			// Accumulate Q: Q_new = Q * H, where H = I - tau*v*v^T
			// w[i] = sum_j Q[i][k+1+j] * v[j]
			w.resize(1, N, false);
			for (int i = 0; i != N; ++i)
				for (int j = 0; j != N - k - 1; ++j)
					w(i) += Q(i, j + k + 1) * v(j);

			// Q[i][k+1+j] -= tau * v[j] * w[i]
			for (int i = 0; i != N; ++i)
				for (int j = 0; j != N - k - 1; ++j)
					Q(i, j + k + 1) -= tau * v(j) * w(i);
		}

		// Extract diagonal and sub-diagonal
		for (int i = 0; i != N; ++i)
			diag(i) = A(i, i);
		for (int i = 1; i != N; ++i)
			sub(i) = A(i, i - 1);
	}

	// Implicit QL iteration with shifts on a symmetric tridiagonal matrix.
	// Based on the EISPACK tql2 / Numerical Recipes algorithm.
	// sub[i] = T[i][i-1] for i >= 1, sub[0] = 0.
	template <typename S> inline void QLIteration(Matrix<S>& d, Matrix<S>& e, Matrix<S>& Q, int max_iterations) noexcept
	{
		auto const N = static_cast<int>(d.size());

		for (int l = 0; l != N; ++l)
		{
			int iter = 0;
			while (true)
			{
				// Find small sub-diagonal element
				int m = l;
				for (; m < N - 1; ++m)
				{
					auto dd = std::abs(d(m)) + std::abs(d(m + 1));
					if (std::abs(e(m + 1)) + dd == dd)
						break;
				}
				if (m == l)
					break;

				if (++iter > max_iterations)
					break;

				// QL shift
				auto g = (d(l + 1) - d(l)) / (S(2) * e(l + 1));
				auto r = std::sqrt(g * g + S(1));
				g = d(m) - d(l) + e(l + 1) / (g + (g >= 0 ? r : -r));

				auto s = S(1);
				auto c = S(1);
				auto p = S(0);

				// Chase the bulge from m-1 down to l
				for (int i = m - 1; i >= l; --i)
				{
					auto f = s * e(i + 1);
					auto b = c * e(i + 1);

					if (std::abs(f) >= std::abs(g))
					{
						c = g / f;
						r = std::sqrt(c * c + S(1));
						e(i + 2) = f * r;
						s = S(1) / r;
						c *= s;
					}
					else
					{
						s = f / g;
						r = std::sqrt(s * s + S(1));
						e(i + 2) = g * r;
						c = S(1) / r;
						s *= c;
					}

					g = d(i + 1) - p;
					r = (d(i) - g) * s + S(2) * c * b;
					p = s * r;
					d(i + 1) = g + p;
					g = c * r - b;

					// Accumulate eigenvectors: rotate columns i and i+1 of Q
					for (int k = 0; k != N; ++k)
					{
						auto qi1 = Q(k, i + 1);
						Q(k, i + 1) = s * Q(k, i) + c * qi1;
						Q(k, i)     = c * Q(k, i) - s * qi1;
					}
				}

				d(l) -= p;
				e(l + 1) = g;
				e(m + 1) = S(0);
			}
		}
	}

	// Compute all eigenvalues and eigenvectors of a real symmetric matrix using
	// Householder tridiagonalization followed by implicit QL iteration with shifts.
	// The matrix must be square. Only the lower triangle is read (symmetry is assumed).
	// Returns eigenvalues in descending order with corresponding eigenvectors as columns.
	// Note: Matrix(i, j) accesses row i, column j in standard notation (row-major storage).
	template <typename S> EigenResult<S> EigenSymmetric(Matrix<S> const& m, int max_iterations = 200)
	{
		auto const N = m.vecs();
		if (!m.IsSquare())
			throw std::runtime_error("EigenSymmetric requires a square matrix");

		if (N == 0)
			return { Matrix<S>(1, 0), Matrix<S>(0, 0) };

		if (N == 1)
		{
			auto vals = Matrix<S>(1, 1, { m(0, 0) });
			auto vecs = Matrix<S>::Identity(1, 1);
			return { std::move(vals), std::move(vecs) };
		}

		// Phase 1: Householder tridiagonalization
		// Reduce symmetric matrix to tridiagonal form: Q^T * A * Q = T
		auto diag = Matrix<S>::Zero(1, N);
		auto sub = Matrix<S>::Zero(1, N + 1); // +1: QL iteration may access e(N) as scratch
		auto Q = Matrix<S>::Identity(N, N);
		Tridiagonalize(m, diag, sub, Q);

		// Phase 2: Implicit QL iteration on the tridiagonal matrix
		QLIteration(diag, sub, Q, max_iterations);

		// Build result sorted by descending eigenvalue
		auto order = std::vector<int>(N);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b) { return diag(a) > diag(b); });

		auto vals = Matrix<S>(1, N);
		auto vecs = Matrix<S>(N, N);
		for (int i = 0; i != N; ++i)
		{
			vals(i) = diag(order[i]);

			// Copy column order[i] of Q into column i of vecs
			for (int r = 0; r != N; ++r)
				vecs(r, i) = Q(r, order[i]);
		}
		return { std::move(vals), std::move(vecs) };
	}

	// Compute the top-k eigenvalues and eigenvectors of a real symmetric matrix using the Lanczos algorithm.
	// Much faster than full decomposition when k << N (e.g., MDS needs only 3 eigenpairs from a 1000×1000 matrix).
	// Returns eigenvalues in descending order with corresponding eigenvectors as columns.
	template <typename S> EigenResult<S> EigenTopK(Matrix<S> const& m, int k_, int max_iterations = 0)
	{
		auto const N = m.vecs();
		if (!m.IsSquare())
			throw std::runtime_error("EigenTopK requires a square matrix");

		auto const k = std::min(k_, N);
		if (N == 0 || k == 0)
			return { Matrix<S>(1, 0), Matrix<S>(0, 0) };

		// For small matrices or when k is close to N, fall back to full decomposition
		if (N <= 32 || k * 4 >= N * 3)
		{
			EigenResult<S> full = EigenSymmetric(m);

			// Truncate to top-k
			auto const NN = full.vectors.vecs();
			auto vals = Matrix<S>(1, k);
			auto vecs = Matrix<S>(NN, k);
			for (int i = 0; i != k; ++i)
			{
				vals(0, i) = full.values(0, i);
				for (int r = 0; r != NN; ++r)
					vecs(r, i) = full.vectors(r, i);
			}
			return { std::move(vals), std::move(vecs) };
		}

		// Lanczos iteration dimension: must be >= k, use min(2k+10, N) for good convergence
		auto const lanczos_dim = std::min(2 * k + 10, N);
		auto const max_iter = max_iterations > 0 ? max_iterations : 3;

		// Run Lanczos with restarts for better convergence
		auto alpha = Matrix<S>(1, lanczos_dim); // diagonal of tridiagonal T
		auto beta = Matrix<S>::Zero(1, lanczos_dim); // sub-diagonal of tridiagonal T
		auto V = Matrix<S>(lanczos_dim, N);            // Lanczos basis vectors (rows)

		EigenResult<S> best_result = { Matrix<S>(1, 0), Matrix<S>(0, 0) };
		auto best_residual = std::numeric_limits<S>::max();

		auto q = Matrix<S>(1, N); // Starting vector for Lanczos iteration
		auto q_prev = Matrix<S>(1, N); // Previous Lanczos vector for recurrence
		auto w = Matrix<S>(1, N); // Temporary vector for A*q

		for (int restart = 0; restart != max_iter; ++restart)
		{
			// Starting vector: use the first Ritz vector from previous run, or [1,1,...,1]/sqrt(N)
			//auto q = std::vector<S>(N);
			if (restart == 0)
			{
				auto inv_sqrt_n = S(1) / std::sqrt(static_cast<S>(N));
				for (int i = 0; i != N; ++i)
					q(i) = inv_sqrt_n;
			}
			else
			{
				// Use the first Ritz vector from the previous result as starting vector
				for (int i = 0; i != N; ++i)
					q(i) = best_result.vectors(i, 0);
			}

			// Lanczos iteration
			q_prev.zero();
			for (int j = 0; j != lanczos_dim; ++j)
			{
				// Store basis vector
				for (int i = 0; i != N; ++i)
					V(j, i) = q(i);

				// w = A * q (A is symmetric, so A(i,j) = A(j,i))
				w.zero();
				for (int i = 0; i != N; ++i)
					for (int jj = 0; jj != N; ++jj)
						w(i) += m(i, jj) * q(jj);

				// alpha[j] = q^T * w
				alpha(j) = S(0);
				for (int i = 0; i != N; ++i)
					alpha(j) += q(i) * w(i);

				// w = w - alpha[j]*q - beta[j]*q_prev
				for (int i = 0; i != N; ++i)
					w(i) -= alpha(j) * q(i) + (j > 0 ? beta(j) * q_prev(i) : S(0));

				// Full reorthogonalization against all previous Lanczos vectors
				for (int jj = 0; jj <= j; ++jj)
				{
					auto dot = S(0);
					for (int i = 0; i != N; ++i)
						dot += w(i) * V(jj, i);
					for (int i = 0; i != N; ++i)
						w(i) -= dot * V(jj, i);
				}

				// beta[j+1] = ||w||
				auto norm_w = S(0);
				for (int i = 0; i != N; ++i)
					norm_w += w(i) * w(i);
				norm_w = std::sqrt(norm_w);

				if (j + 1 < lanczos_dim)
				{
					beta(j + 1) = norm_w;

					// Prepare next q
					q_prev = q;
					if (norm_w > std::numeric_limits<S>::epsilon() * S(100))
					{
						for (int i = 0; i != N; ++i)
							q(i) = w(i) / norm_w;
					}
					else
					{
						break; // Invariant subspace found
					}
				}
			}

			// Build the tridiagonal matrix T and solve its eigenproblem (small: lanczos_dim × lanczos_dim)
			auto T = Matrix<S>::Zero(lanczos_dim, lanczos_dim);
			for (int i = 0; i != lanczos_dim; ++i)
			{
				T(i, i) = alpha(i);
				if (i + 1 < lanczos_dim)
				{
					T(i, i + 1) = beta(i + 1);
					T(i + 1, i) = beta(i + 1);
				}
			}

			// Full eigendecomposition of small tridiagonal matrix
			auto t_eigen = EigenSymmetric(T);

			// Compute Ritz vectors: eigenvectors in original space = V^T * (T's eigenvectors)
			// t_eigen.vectors(j, i) = j-th component of eigenvector i of T
			// V(j, r) = r-th component of basis vector j
			// ritz_i[r] = sum_j eigvec_i[j] * basis_j[r]
			auto result = EigenResult<S>{};
			result.values = Matrix<S>(1, k);
			result.vectors = Matrix<S>(N, k);

			for (int i = 0; i != k; ++i)
			{
				result.values(0, i) = t_eigen.values(0, i);
				for (int r = 0; r != N; ++r)
				{
					auto val = S(0);
					for (int j = 0; j != lanczos_dim; ++j)
						val += t_eigen.vectors(j, i) * V(j, r);
					result.vectors(r, i) = val;
				}
			}

			// Check convergence via residual norm of the k-th Ritz pair
			auto residual = S(0);
			for (int i = 0; i != k; ++i)
			{
				// Residual for Ritz pair i: ||A*v - lambda*v||
				auto max_res = S(0);
				for (int r = 0; r != N; ++r)
				{
					auto av = S(0);
					for (int c = 0; c != N; ++c)
						av += m(r, c) * result.vectors(c, i);
					auto diff = std::abs(av - result.values(0, i) * result.vectors(r, i));
					max_res = std::max(max_res, diff);
				}
				residual = std::max(residual, max_res);
			}

			if (residual < best_residual)
			{
				best_residual = residual;
				best_result = std::move(result);
			}

			// Converged if residual is small enough
			auto scale = S(0);
			for (int i = 0; i != k; ++i)
				scale = std::max(scale, std::abs(best_result.values(0, i)));
			if (best_residual < std::numeric_limits<S>::epsilon() * scale * S(100))
				break;
		}

		return best_result;
	}
}
