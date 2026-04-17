//***************************************************
// Matrix
//  Copyright (c) Rylogic Ltd 2008
//***************************************************
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using Rylogic.Extn;
using Rylogic.Utility;

namespace Rylogic.Maths
{
	/// <summary>A dynamic NxM matrix</summary>
	[DebuggerDisplay("{Description,nq}")]
	public class Matrix
	{
		// Notes:
		//  - Matrix has reference semantics because it is potentially a large object.
		//  - Data is stored as contiguous vectors (like m4x4 does, i.e. row major)
		//    Visually, the matrix can be displayed with the vectors as rows or columns.
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
		//    with the vectors as rows or columns, even though the data are stored in rows.
		//    So I'm not using Row/Column notation. 'Vector/Component' notation is less ambiguous.
		//  - Accessors use 'vec' first then 'cmp' so that from left-to-right you select
		//    the vector first then the component.

		public Matrix(int vec_count, int cmp_count)
		{
			Vecs = vec_count;
			Cmps = cmp_count;
			Data = new double[Vecs * Cmps];
		}
		public Matrix(int vec_count, int cmp_count, IEnumerable<double> data)
			:this(vec_count, cmp_count)
		{
			var k = 0;
			foreach (var d in data)
			{
				if (k == Data.Length)
					throw new Exception("Excess data when initialising Matrix");

				Data[k++] = d;
			}
			if (k != Data.Length)
				throw new Exception("Insufficient data when initialising Matrix");
		}
		public Matrix(int vec_count, int cmp_count, IEnumerable<float> data)
			:this(vec_count, cmp_count, data.Select(x => (double)x))
		{}
		public Matrix(bool vec, IList<double> data)
			:this(vec ? 1 : data.Count, vec ? data.Count : 1)
		{
			data.CopyTo(Data, 0);
		}
		public Matrix(Matrix rhs)
			:this(rhs.Vecs, rhs.Cmps)
		{
			Array.Copy(rhs.Data, Data, rhs.Data.Length);
		}

		/// <summary>The number of vectors in this matrix</summary>
		public int Vecs { get; }

		/// <summary>The number of components in each vector in this matrix</summary>
		public int Cmps { get; }

		/// <summary>The total number of elements in the matrix</summary>
		public int Size => Data.Length;

		/// <summary>Access the underlying matrix data</summary>
		public double[] Data { get; }

		/// <summary>Access this matrix as a 2D array</summary>
		public double this[int vec, int cmp]
		{
			get
			{
				Util.Assert(vec >= 0 && vec < Vecs);
				Util.Assert(cmp >= 0 && cmp < Cmps);
				return Data[vec * Cmps + cmp];
			}
			set
			{
				Util.Assert(vec >= 0 && vec < Vecs);
				Util.Assert(cmp >= 0 && cmp < Cmps);
				Data[vec * Cmps + cmp] = value;
			}
		}

		/// <summary>Access this matrix by Vector</summary>
		public VecProxy Vec => new(this);
		public class VecProxy
		{
			private Matrix mat;
			public VecProxy(Matrix m) => mat = m;
			public Matrix this[int k]
			{
				get
				{
					var m = new Matrix(1, mat.Cmps);
					for (int c = 0; c != mat.Cmps; ++c)
						m[0, c] = mat[k, c];

					return m;
				}
				set
				{
					if (value.Cmps != mat.Cmps)
						throw new Exception("Incorrect number of components in this vector");

					for (int c = 0; c != mat.Cmps; ++c)
						mat[k, c] = value[0, c];
				}
			}
		}

		/// <summary>Access this matrix by rows of components</summary>
		public CmpProxy Cmp => new(this);
		public class CmpProxy
		{
			private Matrix mat;
			public CmpProxy(Matrix m) => mat = m;
			public Matrix this[int k]
			{
				get
				{
					var m = new Matrix(mat.Vecs, 1);
					for (int r = 0; r != mat.Vecs; ++r)
						m[r, 0] = mat[r, k];

					return m;
				}
				set
				{
					if (value.Vecs != mat.Vecs)
						throw new Exception("Incorrect number of components in this vector");
					for (int r = 0; r != mat.Vecs; ++r)
						mat[r, k] = value[r, 0];
				}
			}
		}

		/// <summary>True if the matrix is square</summary>
		public bool IsSquare => Vecs == Cmps;

		/// <summary>Set this matrix to all zeros</summary>
		public Matrix Zero()
		{
			Array.Clear(Data, 0, Data.Length);
			return this;
		}

		/// <summary>Set this matrix to an identity matrix</summary>
		public Matrix Identity()
		{
			Zero();
			for (int i = 0; i < Data.Length; i += Cmps+1) Data[i] = 1.0;
			return this;
		}

		/// <summary>A pretty string description of the matrix</summary>
		public string Description
		{
			get
			{
				// Show the vectors as rows
				var s = new StringBuilder();
				s.AppendLine($"[{Vecs}x{Cmps}]");
				for (int r = 0; r != Vecs; ++r)
				{
					for (int c = 0; c != Cmps; ++c)
						s.Append($"{this[r,c],5:0.0000} ");

					s.AppendLine();
				}
				return s.ToString();
			}
		}

		/// <summary>ToString</summary>
		public override string ToString()
		{
			return Description;
		}

		/// <summary>Return an identity matrix of the given dimensions</summary>
		public static Matrix Identity(int vec_count, int cmp_count)
		{
			return new Matrix(vec_count, cmp_count).Identity();
		}

		/// <summary>Return a vector from a list of values</summary>
		public static Matrix AsVec(params double[] values)
		{
			return new Matrix(true, values);
		}

		/// <summary>Return a transposed vector from a list of values</summary>
		public static Matrix AsCmp(params double[] values)
		{
			return new Matrix(false, values);
		}

		#region Functions

		/// <summary>Special case equality operators</summary>
		public static bool FEql(Matrix lhs, Matrix rhs)
		{
			if (lhs.Vecs != rhs.Vecs) return false;
			if (lhs.Cmps != rhs.Cmps) return false;
			for (int i = 0; i != lhs.Data.Length; ++i)
				if (!Math_.FEql(lhs.Data[i], rhs.Data[i]))
					return false;

			return true;
		}
		public static bool FEql(Matrix lhs, m4x4 rhs)
		{
			if (lhs.Vecs != 4 && lhs.Cmps != 4) return false;
			return
				Math_.FEql((float)lhs.Data[ 0], rhs.x.x) &&
				Math_.FEql((float)lhs.Data[ 1], rhs.x.y) &&
				Math_.FEql((float)lhs.Data[ 2], rhs.x.z) &&
				Math_.FEql((float)lhs.Data[ 3], rhs.x.w) &&

				Math_.FEql((float)lhs.Data[ 4], rhs.y.x) &&
				Math_.FEql((float)lhs.Data[ 5], rhs.y.y) &&
				Math_.FEql((float)lhs.Data[ 6], rhs.y.z) &&
				Math_.FEql((float)lhs.Data[ 7], rhs.y.w) &&

				Math_.FEql((float)lhs.Data[ 8], rhs.z.x) &&
				Math_.FEql((float)lhs.Data[ 9], rhs.z.y) &&
				Math_.FEql((float)lhs.Data[10], rhs.z.z) &&
				Math_.FEql((float)lhs.Data[11], rhs.z.w) &&

				Math_.FEql((float)lhs.Data[12], rhs.w.x) &&
				Math_.FEql((float)lhs.Data[13], rhs.w.y) &&
				Math_.FEql((float)lhs.Data[14], rhs.w.z) &&
				Math_.FEql((float)lhs.Data[15], rhs.w.w);
		}
		public static bool FEql(Matrix lhs, v4 rhs)
		{
			if (lhs.Vecs != 1 && lhs.Cmps != 1) return false;
			if (lhs.Size != 4) return false;

			return
				Math_.FEql((float)lhs.Data[0], rhs.x) &&
				Math_.FEql((float)lhs.Data[1], rhs.y) &&
				Math_.FEql((float)lhs.Data[2], rhs.z) &&
				Math_.FEql((float)lhs.Data[3], rhs.w);
		}

		/// <summary>Return a new matrix that is the transpose of 'm'</summary>
		public static Matrix Transpose(Matrix m)
		{
			var t = new Matrix(m.Cmps, m.Vecs);
			for (int r = 0; r < m.Vecs; r++)
				for (int c = 0; c < m.Cmps; c++)
					t[c, r] = m[r, c];
			return t;
		}

		/// <summary>Return the determinant of this matrix</summary>
		public static double Determinant(Matrix m)
		{
			return MatrixLU.Determinant(new MatrixLU(m));
		}

		/// <summary>True if 'mat' has an inverse</summary>
		public static bool IsInvertible(Matrix m)
		{
			return MatrixLU.IsInvertible(new MatrixLU(m));
		}

		/// <summary>Solves for x in 'A.x = v'</summary>
		public static Matrix Solve(Matrix A, Matrix v)
		{
			return MatrixLU.Solve(new MatrixLU(A), v);
		}

		/// <summary>Return the inverse of matrix 'm'</summary>
		public static Matrix Invert(Matrix m)
		{
			Util.Assert(m.IsSquare, "Only square matrices are invertible");
			return MatrixLU.Invert(new MatrixLU(m));
		}

		/// <summary>Matrix to the power 'pow'</summary>
		public static Matrix Power(Matrix m, int pow)
		{
			if (pow == +1) return m;
			if (pow ==  0) return Identity(m.Vecs, m.Cmps);
			if (pow == -1) return Invert(m);

			var x = pow < 0 ? Invert(m) : m;
			if (pow < 0) pow *= -1;

			var ret = Identity(m.Vecs, m.Cmps);
			while (pow != 0)
			{
				if ((pow & 1) == 1) ret *= x;
				x *= x;
				pow >>= 1;
			}
			return ret;
		}

		/// <summary>Generates the random matrix</summary>
		public static Matrix Random(int vec_count, int cmp_count, double min, double max, Random rng)
		{
			var m = new Matrix(vec_count, cmp_count);
			for (int r = 0; r != vec_count; ++r)
				for (int c = 0; c != cmp_count; ++c)
					m[r,c] = rng.Double(min, max);

			return m;
		}

		/// <summary>Return the dot product of two row vectors (both must be 1×N)</summary>
		public static double Dot(Matrix lhs, Matrix rhs)
		{
			Util.Assert(lhs.Vecs == 1 && rhs.Vecs == 1, "Dot product is between row vectors");
			Util.Assert(lhs.Cmps == rhs.Cmps, "Dot product must be between vectors of the same length");

			var dp = 0.0;
			for (int i = 0, iend = lhs.Cmps; i != iend; ++i)
				dp += lhs[0, i] * rhs[0, i];
			return dp;
		}

		/// <summary>Householder tridiagonalization: Q^T * A * Q = T</summary>
		public static void Tridiagonalize(Matrix m, Matrix diag, Matrix sub, Matrix Q)
		{
			var N = m.Vecs;
			Util.Assert(diag.Vecs == 1 && diag.Cmps == N);
			Util.Assert(sub.Vecs == 1 && sub.Cmps >= N);
			Util.Assert(Q.Vecs == N && Q.Cmps == N);

			var A = new Matrix(m);
			var v = new Matrix(1, 0);
			var p = new Matrix(1, 0);
			var kk = new Matrix(1, 0);
			var w = new Matrix(1, 0);

			for (int k = 0; k != N - 2; ++k)
			{
				// Build Householder vector to zero out A[k+2:N-1][k] (column k, below sub-diagonal)
				var sigma = 0.0;
				for (int i = k + 2; i != N; ++i)
					sigma += A[i, k] * A[i, k];

				// Machine epsilon for double (~2.22e-16), equivalent to C++ std::numeric_limits<double>::epsilon()
				const double eps = 2.2204460492503131e-16;
				if (sigma < eps * eps)
					continue;

				var alpha = A[k + 1, k];
				var norm = Math.Sqrt(alpha * alpha + sigma);
				var beta = (alpha >= 0) ? alpha + norm : alpha - norm;

				// v = [1, A[k+2][k]/beta, ..., A[N-1][k]/beta]
				v = new Matrix(1, N - k - 1);
				v[0, 0] = 1.0;
				for (int i = 1; i != N - k - 1; ++i)
					v[0, i] = A[k + 1 + i, k] / beta;

				var tau = 2.0 / Dot(v, v);

				// p = tau * A_sub * v, where A_sub = A[k+1:N-1, k+1:N-1]
				p = new Matrix(1, N - k - 1);
				for (int i = 0; i != N - k - 1; ++i)
					for (int j = 0; j != N - k - 1; ++j)
						p[0, i] += A[k + 1 + i, k + 1 + j] * v[0, j];
				for (int i = 0; i != p.Size; ++i)
					p.Data[i] *= tau;

				// kk = p - (tau/2)*(p·v)*v
				var pv = Dot(p, v);
				kk = new Matrix(1, N - k - 1);
				for (int i = 0; i != N - k - 1; ++i)
					kk[0, i] = p[0, i] - (tau / 2.0) * pv * v[0, i];

				// Update A_sub: A_sub[i][j] -= v[i]*kk[j] + kk[i]*v[j]
				for (int i = 0; i != N - k - 1; ++i)
					for (int j = 0; j != N - k - 1; ++j)
						A[k + 1 + i, k + 1 + j] -= v[0, i] * kk[0, j] + kk[0, i] * v[0, j];

				// Set the sub-diagonal element
				A[k + 1, k] = -(alpha >= 0 ? 1.0 : -1.0) * norm;
				A[k, k + 1] = A[k + 1, k];
				for (int i = k + 2; i != N; ++i)
				{
					A[i, k] = 0.0;
					A[k, i] = 0.0;
				}

				// Accumulate Q: Q_new = Q * H, where H = I - tau*v*v^T
				// w[i] = sum_j Q[i][k+1+j] * v[j]
				w = new Matrix(1, N);
				for (int i = 0; i != N; ++i)
					for (int j = 0; j != N - k - 1; ++j)
						w[0, i] += Q[i, j + k + 1] * v[0, j];

				// Q[i][k+1+j] -= tau * v[j] * w[i]
				for (int i = 0; i != N; ++i)
					for (int j = 0; j != N - k - 1; ++j)
						Q[i, j + k + 1] -= tau * v[0, j] * w[0, i];
			}

			// Extract diagonal and sub-diagonal
			for (int i = 0; i != N; ++i)
				diag[0, i] = A[i, i];
			for (int i = 1; i != N; ++i)
				sub[0, i] = A[i, i - 1];
		}

		/// <summary>Implicit QL iteration with shifts on a symmetric tridiagonal matrix</summary>
		public static void QLIteration(Matrix d, Matrix e, Matrix Q, int max_iterations)
		{
			var N = d.Size;

			for (int l = 0; l != N; ++l)
			{
				var iter = 0;
				while (true)
				{
					// Find small sub-diagonal element
					var m = l;
					for (; m < N - 1; ++m)
					{
						var dd = Math.Abs(d[0, m]) + Math.Abs(d[0, m + 1]);
						if (Math.Abs(e[0, m + 1]) + dd == dd)
							break;
					}
					if (m == l)
						break;

					if (++iter > max_iterations)
						break;

					// QL shift
					var g = (d[0, l + 1] - d[0, l]) / (2.0 * e[0, l + 1]);
					var r = Math.Sqrt(g * g + 1.0);
					g = d[0, m] - d[0, l] + e[0, l + 1] / (g + (g >= 0 ? r : -r));

					var s = 1.0;
					var c = 1.0;
					var p = 0.0;

					// Chase the bulge from m-1 down to l
					for (int i = m - 1; i >= l; --i)
					{
						var f = s * e[0, i + 1];
						var b = c * e[0, i + 1];

						if (Math.Abs(f) >= Math.Abs(g))
						{
							c = g / f;
							r = Math.Sqrt(c * c + 1.0);
							e[0, i + 2] = f * r;
							s = 1.0 / r;
							c *= s;
						}
						else
						{
							s = f / g;
							r = Math.Sqrt(s * s + 1.0);
							e[0, i + 2] = g * r;
							c = 1.0 / r;
							s *= c;
						}

						g = d[0, i + 1] - p;
						r = (d[0, i] - g) * s + 2.0 * c * b;
						p = s * r;
						d[0, i + 1] = g + p;
						g = c * r - b;

						// Accumulate eigenvectors: rotate columns i and i+1 of Q
						for (int k = 0; k != N; ++k)
						{
							var qi1 = Q[k, i + 1];
							Q[k, i + 1] = s * Q[k, i] + c * qi1;
							Q[k, i]     = c * Q[k, i] - s * qi1;
						}
					}

					d[0, l] -= p;
					e[0, l + 1] = g;
					e[0, m + 1] = 0.0;
				}
			}
		}

		/// <summary>Result of an eigen decomposition</summary>
		public struct EigenResult
		{
			/// <summary>1×N row vector of eigenvalues, sorted descending. Access as Values[0, i].</summary>
			public Matrix Values;

			/// <summary>N×N (or N×k) matrix where column i is the eigenvector for Values[0, i].</summary>
			public Matrix Vectors;
		}

		/// <summary>
		/// Compute all eigenvalues and eigenvectors of a real symmetric matrix using
		/// Householder tridiagonalization followed by implicit QL iteration with shifts.
		/// Returns eigenvalues in descending order with corresponding eigenvectors as columns.
		/// </summary>
		public static EigenResult EigenSymmetric(Matrix m, int max_iterations = 200)
		{
			var N = m.Vecs;
			if (!m.IsSquare)
				throw new Exception("EigenSymmetric requires a square matrix");

			if (N == 0)
				return new EigenResult { Values = new Matrix(1, 0), Vectors = new Matrix(0, 0) };

			if (N == 1)
			{
				var vals = new Matrix(1, 1, new[] { m[0, 0] });
				var vecs = Identity(1, 1);
				return new EigenResult { Values = vals, Vectors = vecs };
			}

			// Phase 1: Householder tridiagonalization
			var diag = new Matrix(1, N);
			var sub = new Matrix(1, N + 1); // +1: QL iteration may access e(N) as scratch
			var Q = Identity(N, N);
			Tridiagonalize(m, diag, sub, Q);

			// Phase 2: Implicit QL iteration on the tridiagonal matrix
			QLIteration(diag, sub, Q, max_iterations);

			// Build result sorted by descending eigenvalue
			var order = new int[N];
			for (int i = 0; i != N; ++i)
				order[i] = i;
			Array.Sort(order, (a, b) => diag[0, b].CompareTo(diag[0, a]));

			var sorted_vals = new Matrix(1, N);
			var sorted_vecs = new Matrix(N, N);
			for (int i = 0; i != N; ++i)
			{
				sorted_vals[0, i] = diag[0, order[i]];

				// Copy column order[i] of Q into column i of sorted_vecs
				for (int r = 0; r != N; ++r)
					sorted_vecs[r, i] = Q[r, order[i]];
			}
			return new EigenResult { Values = sorted_vals, Vectors = sorted_vecs };
		}

		/// <summary>
		/// Compute the top-k eigenvalues and eigenvectors of a real symmetric matrix using the Lanczos algorithm.
		/// Much faster than full decomposition when k &lt;&lt; N (e.g., MDS needs only 3 eigenpairs from a 1000×1000 matrix).
		/// Returns eigenvalues in descending order with corresponding eigenvectors as columns.
		/// </summary>
		public static EigenResult EigenTopK(Matrix m, int k_, int max_iterations = 0)
		{
			var N = m.Vecs;
			if (!m.IsSquare)
				throw new Exception("EigenTopK requires a square matrix");

			var k = Math.Min(k_, N);
			if (N == 0 || k == 0)
				return new EigenResult { Values = new Matrix(1, 0), Vectors = new Matrix(0, 0) };

			// For small matrices or when k is close to N, fall back to full decomposition
			if (N <= 32 || k * 4 >= N * 3)
			{
				var full = EigenSymmetric(m);

				// Truncate to top-k
				var NN = full.Vectors.Vecs;
				var tvals = new Matrix(1, k);
				var tvecs = new Matrix(NN, k);
				for (int i = 0; i != k; ++i)
				{
					tvals[0, i] = full.Values[0, i];
					for (int r = 0; r != NN; ++r)
						tvecs[r, i] = full.Vectors[r, i];
				}
				return new EigenResult { Values = tvals, Vectors = tvecs };
			}

			// Lanczos iteration dimension: must be >= k, use min(2k+10, N) for good convergence
			var lanczos_dim = Math.Min(2 * k + 10, N);
			var max_iter = max_iterations > 0 ? max_iterations : 3;

			// Run Lanczos with restarts for better convergence
			var alpha = new Matrix(1, lanczos_dim);
			var beta_ = new Matrix(1, lanczos_dim);
			var V = new Matrix(lanczos_dim, N);

			var best_result = new EigenResult { Values = new Matrix(1, 0), Vectors = new Matrix(0, 0) };
			var best_residual = double.MaxValue;

			var q = new Matrix(1, N);
			var q_prev = new Matrix(1, N);
			var w = new Matrix(1, N);

			for (int restart = 0; restart != max_iter; ++restart)
			{
				// Starting vector: use the first Ritz vector from previous run, or a deterministic
				// pseudo-random vector. We deliberately avoid the all-ones vector because it is
				// often exactly the null eigenvector of double-centred / Laplacian-like matrices
				// (e.g. the classical MDS Gram matrix B = -0.5·J·D²·J satisfies B·1 = 0). Starting
				// Lanczos from such a null vector causes immediate collapse: Aq = 0, so w = 0 and
				// all subsequent alpha/beta are zero, returning eigenvalues ≈ 0.
				// A pseudo-random vector has (with overwhelming probability) a non-zero component
				// along every eigenvector, and Lanczos' implicit deflation will correctly drop any
				// null directions as it builds the Krylov subspace.
				if (restart == 0)
				{
					// Fixed seed -> deterministic / reproducible results across runs.
					var rng = new Random(unchecked(0x5EED0000 + N));
					var norm_sq = 0.0;
					for (int i = 0; i != N; ++i)
					{
						var v = rng.NextDouble() - 0.5;
						q[0, i] = v;
						norm_sq += v * v;
					}
					var inv_norm = 1.0 / Math.Sqrt(norm_sq);
					for (int i = 0; i != N; ++i)
						q[0, i] *= inv_norm;
				}
				else
				{
					for (int i = 0; i != N; ++i)
						q[0, i] = best_result.Vectors[i, 0];
				}

				// Lanczos iteration
				q_prev.Zero();
				for (int j = 0; j != lanczos_dim; ++j)
				{
					// Store basis vector
					for (int i = 0; i != N; ++i)
						V[j, i] = q[0, i];

					// w = A * q (A is symmetric, so A(i,j) = A(j,i))
					w.Zero();
					for (int i = 0; i != N; ++i)
						for (int jj = 0; jj != N; ++jj)
							w[0, i] += m[i, jj] * q[0, jj];

					// alpha[j] = q^T * w
					alpha[0, j] = 0.0;
					for (int i = 0; i != N; ++i)
						alpha[0, j] += q[0, i] * w[0, i];

					// w = w - alpha[j]*q - beta[j]*q_prev
					for (int i = 0; i != N; ++i)
						w[0, i] -= alpha[0, j] * q[0, i] + (j > 0 ? beta_[0, j] * q_prev[0, i] : 0.0);

					// Full reorthogonalization against all previous Lanczos vectors
					for (int jj = 0; jj <= j; ++jj)
					{
						var dot = 0.0;
						for (int i = 0; i != N; ++i)
							dot += w[0, i] * V[jj, i];
						for (int i = 0; i != N; ++i)
							w[0, i] -= dot * V[jj, i];
					}

					// beta[j+1] = ||w||
					var norm_w = 0.0;
					for (int i = 0; i != N; ++i)
						norm_w += w[0, i] * w[0, i];
					norm_w = Math.Sqrt(norm_w);

					if (j + 1 < lanczos_dim)
					{
						beta_[0, j + 1] = norm_w;

						// Prepare next q
						q_prev = new Matrix(q);
						if (norm_w > 2.2204460492503131e-16 * 100.0)
						{
							for (int i = 0; i != N; ++i)
								q[0, i] = w[0, i] / norm_w;
						}
						else
						{
							break; // Invariant subspace found
						}
					}
				}

				// Build the tridiagonal matrix T and solve its eigenproblem (small: lanczos_dim × lanczos_dim)
				var T = new Matrix(lanczos_dim, lanczos_dim);
				for (int i = 0; i != lanczos_dim; ++i)
				{
					T[i, i] = alpha[0, i];
					if (i + 1 < lanczos_dim)
					{
						T[i, i + 1] = beta_[0, i + 1];
						T[i + 1, i] = beta_[0, i + 1];
					}
				}

				// Full eigendecomposition of small tridiagonal matrix
				var t_eigen = EigenSymmetric(T);

				// Compute Ritz vectors: eigenvectors in original space = V^T * (T's eigenvectors)
				var result = new EigenResult
				{
					Values = new Matrix(1, k),
					Vectors = new Matrix(N, k),
				};

				for (int i = 0; i != k; ++i)
				{
					result.Values[0, i] = t_eigen.Values[0, i];
					for (int r = 0; r != N; ++r)
					{
						var val = 0.0;
						for (int j = 0; j != lanczos_dim; ++j)
							val += t_eigen.Vectors[j, i] * V[j, r];
						result.Vectors[r, i] = val;
					}
				}

				// Check convergence via residual norm of the k-th Ritz pair
				var residual = 0.0;
				for (int i = 0; i != k; ++i)
				{
					var max_res = 0.0;
					for (int r = 0; r != N; ++r)
					{
						var av = 0.0;
						for (int c = 0; c != N; ++c)
							av += m[r, c] * result.Vectors[c, i];
						var diff = Math.Abs(av - result.Values[0, i] * result.Vectors[r, i]);
						max_res = Math.Max(max_res, diff);
					}
					residual = Math.Max(residual, max_res);
				}

				if (residual < best_residual)
				{
					best_residual = residual;
					best_result = result;
				}

				// Converged if residual is small enough
				var scale = 0.0;
				for (int i = 0; i != k; ++i)
					scale = Math.Max(scale, Math.Abs(best_result.Values[0, i]));
				if (best_residual < 2.2204460492503131e-16 * scale * 100.0)
					break;
			}

			return best_result;
		}

		/// <summary>Parse a matrix from a string</summary>
		public static Matrix Parse(string s)
		{
			// Normalise the matrix string
			{
				// Remove any multiple spaces
				while (s.IndexOf("  ") != -1)
					s = s.Replace("  ", " ");

				// Remove any spaces before or after newlines
				s = s.Replace(" \r\n", "\r\n");
				s = s.Replace("\r\n ", "\r\n");

				// If the data ends in a newline, remove the trailing newline.
				// Make it easier by first replacing \r\n’s with |’s then
				// restore the |’s with \r\n’s
				s = s.Replace("\r\n", "|");
				while (s.LastIndexOf("|") == (s.Length - 1))
					s = s.Substring(0, s.Length - 1);

				s = s.Replace("|", "\r\n");
				s = s.Trim();
			}

			var vecs = Regex.Split(s, "\r\n");
			var m = new Matrix(vecs.Length, vecs[0].Split(' ').Length);
			for (int r = 0; r < vecs.Length; r++)
			{
				var cmps = vecs[r].Split(' ');
				for (int c = 0; c < cmps.Length; c++)
					m[r,c] = double.Parse(cmps[c]);
			}
			return m;
		}

		#endregion

		#region Operators

		// Unary operators
		public static Matrix operator + (Matrix m)
		{
			return m;
		}
		public static Matrix operator - (Matrix m)
		{
			return -1 * m;
		}

		// Addition/Subtraction
		public static Matrix operator + (Matrix lhs, Matrix rhs)
		{
			if (lhs.Vecs != rhs.Vecs || lhs.Cmps != rhs.Cmps)
				throw new Exception("Matrices must have the same dimensions!");

			var r = new Matrix(lhs.Vecs, lhs.Cmps);
			for (int i = 0; i != lhs.Data.Length; ++i)
				r.Data[i] = lhs.Data[i] + rhs.Data[i];

			return r;
		}
		public static Matrix operator - (Matrix lhs, Matrix rhs)
		{
			if (lhs.Vecs != rhs.Vecs || lhs.Cmps != rhs.Cmps)
				throw new Exception("Matrices must have the same dimensions!");

			var r = new Matrix(lhs.Vecs, lhs.Cmps);
			for (int i = 0; i != lhs.Data.Length; ++i)
				r.Data[i] = lhs.Data[i] - rhs.Data[i];

			return r;
		}

		/// <summary>Matrix times scalar</summary>
		public static Matrix operator * (Matrix mat, double s)
		{
			var r = new Matrix(mat.Vecs, mat.Cmps);
			for (int i = 0; i != r.Data.Length; ++i)
				r.Data[i] = mat.Data[i] * s;

			return r;
		}
		public static Matrix operator * (double s, Matrix mat)
		{
			return mat * s;
		}

		/// <summary>Matrix product: a2b = b2c * a2b</summary>
		public static Matrix operator *(Matrix b2c, Matrix a2b)
		{
			// Note:
			//  - The multplication order is the same as for m4x4.
			//  - The shape of the result is:
			//        [  b2c  ]       [a2b]       [a2c]
			//        [  2x3  ]   *   [1x2]   =   [1x3]
			//        [       ]       [   ]       [   ]

			if (a2b.Cmps != b2c.Vecs)
				throw new Exception("Matrix inner dimensions must be the same to multiply them");

			// Result
			var R = new Matrix(a2b.Vecs, b2c.Cmps);

			// Small matrix multiply
			int msize = Math_.Max(a2b.Vecs, a2b.Cmps, b2c.Vecs, b2c.Cmps);
			if (msize < 32)
			{
				for (int i = 0; i != R.Vecs; ++i)
					for (int j = 0; j != R.Cmps; ++j)
						for (int k = 0; k != a2b.Cmps; ++k)
							R[i, j] += a2b[i, k] * b2c[k, j];

				return R;
			}

			// "Strassen Multiply"
			int size = 1; int n = 0;
			while (msize > size) { size *= 2; n++; };
			int h = size / 2;

			//  8x8, 8x8, 8x8, ...
			//  4x4, 4x4, 4x4, ...
			//  2x2, 2x2, 2x2, ...
			//  . . .
			var field = new Matrix[n, 9];
			for (int i = 0; i < n - 4; i++)
			{
				var z = (int)Math.Pow(2, n - i - 1);
				for (int j = 0; j < 9; j++)
					field[i, j] = new Matrix(z, z);
			}

			#region Sub Functions
			static void SafeAplusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int sz)
			{
				for (int i = 0; i < sz; i++) // rows
				{
					for (int j = 0; j < sz; j++) // cols
					{
						C[i, j] = 0;
						if (xa + j < A.Cmps && ya + i < A.Vecs) C[i, j] += A[ya + i, xa + j];
						if (xb + j < B.Cmps && yb + i < B.Vecs) C[i, j] += B[yb + i, xb + j];
					}
				}
			}
			static void SafeAminusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int sz)
			{
				for (int i = 0; i < sz; i++) // rows
				{
					for (int j = 0; j < sz; j++) // cols
					{
						C[i, j] = 0;
						if (xa + j < A.Cmps && ya + i < A.Vecs) C[i, j] += A[ya + i, xa + j];
						if (xb + j < B.Cmps && yb + i < B.Vecs) C[i, j] -= B[yb + i, xb + j];
					}
				}
			}
			static void SafeACopytoC(Matrix A, int xa, int ya, Matrix C, int sz)
			{
				for (int i = 0; i < sz; i++) // rows
				{
					for (int j = 0; j < sz; j++) // cols
					{
						C[i, j] = 0;
						if (xa + j < A.Cmps && ya + i < A.Vecs) C[i, j] += A[ya + i, xa + j];
					}
				}
			}
			static void AplusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int sz)
			{
				for (int i = 0; i < sz; i++) // rows
					for (int j = 0; j < sz; j++)
						C[i, j] = A[ya + i, xa + j] + B[yb + i, xb + j];
			}
			static void AminusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int sz)
			{
				for (int i = 0; i < sz; i++) // rows
					for (int j = 0; j < sz; j++)
						C[i, j] = A[ya + i, xa + j] - B[yb + i, xb + j];
			}
			static void ACopytoC(Matrix A, int xa, int ya, Matrix C, int sz)
			{
				for (int i = 0; i < sz; i++) // rows
					for (int j = 0; j < sz; j++)
						C[i, j] = A[ya + i, xa + j];
			}
			static void StrassenMultiplyRun(Matrix A, Matrix B, Matrix C, int l, Matrix[,] f)
			{
				// A * B into C, level of recursion, matrix field
				// function for square matrix 2^N x 2^N
				var sz = A.Vecs;
				if (sz < 32)
				{
					C.Zero();
					for (int i = 0; i != C.Vecs; ++i)
						for (int j = 0; j != C.Cmps; ++j)
							for (int k = 0; k != A.Cmps; ++k)
								C[i, j] += A[i, k] * B[k, j];

					return;
				}

				var hh = sz / 2;
				AplusBintoC(A, 0, 0, A, hh, hh, f[l, 0], hh);
				AplusBintoC(B, 0, 0, B, hh, hh, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 1], l + 1, f); // (A11 + A22) * (B11 + B22);

				AplusBintoC(A, 0, hh, A, hh, hh, f[l, 0], hh);
				ACopytoC(B, 0, 0, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 2], l + 1, f); // (A21 + A22) * B11;

				ACopytoC(A, 0, 0, f[l, 0], hh);
				AminusBintoC(B, hh, 0, B, hh, hh, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 3], l + 1, f); //A11 * (B12 - B22);

				ACopytoC(A, hh, hh, f[l, 0], hh);
				AminusBintoC(B, 0, hh, B, 0, 0, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 4], l + 1, f); //A22 * (B21 - B11);

				AplusBintoC(A, 0, 0, A, hh, 0, f[l, 0], hh);
				ACopytoC(B, hh, hh, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 5], l + 1, f); //(A11 + A12) * B22;

				AminusBintoC(A, 0, hh, A, 0, 0, f[l, 0], hh);
				AplusBintoC(B, 0, 0, B, hh, 0, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 6], l + 1, f); //(A21 - A11) * (B11 + B12);

				AminusBintoC(A, hh, 0, A, hh, hh, f[l, 0], hh);
				AplusBintoC(B, 0, hh, B, hh, hh, f[l, 1], hh);
				StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 7], l + 1, f); // (A12 - A22) * (B21 + B22);

				// C11
				for (int i = 0; i < hh; i++) // rows
					for (int j = 0; j < hh; j++) // cols
						C[i, j] = f[l, 1 + 1][i, j] + f[l, 1 + 4][i, j] - f[l, 1 + 5][i, j] + f[l, 1 + 7][i, j];

				// C12
				for (int i = 0; i < hh; i++) // rows
					for (int j = hh; j < sz; j++) // cols
						C[i, j] = f[l, 1 + 3][i, j - hh] + f[l, 1 + 5][i, j - hh];

				// C21
				for (int i = hh; i < sz; i++) // rows
					for (int j = 0; j < hh; j++) // cols
						C[i, j] = f[l, 1 + 2][i - hh, j] + f[l, 1 + 4][i - hh, j];

				// C22
				for (int i = hh; i < sz; i++) // rows
					for (int j = hh; j < sz; j++) // cols
						C[i, j] = f[l, 1 + 1][i - hh, j - hh] - f[l, 1 + 2][i - hh, j - hh] + f[l, 1 + 3][i - hh, j - hh] + f[l, 1 + 6][i - hh, j - hh];
			}
			#endregion

			#region Strassen Multiply

			SafeAplusBintoC(a2b, 0, 0, a2b, h, h, field[0, 0], h);
			SafeAplusBintoC(b2c, 0, 0, b2c, h, h, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 1], 1, field); // (A11 + A22) * (B11 + B22);

			SafeAplusBintoC(a2b, 0, h, a2b, h, h, field[0, 0], h);
			SafeACopytoC(b2c, 0, 0, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 2], 1, field); // (A21 + A22) * B11;

			SafeACopytoC(a2b, 0, 0, field[0, 0], h);
			SafeAminusBintoC(b2c, h, 0, b2c, h, h, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 3], 1, field); //A11 * (B12 - B22);

			SafeACopytoC(a2b, h, h, field[0, 0], h);
			SafeAminusBintoC(b2c, 0, h, b2c, 0, 0, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 4], 1, field); //A22 * (B21 - B11);

			SafeAplusBintoC(a2b, 0, 0, a2b, h, 0, field[0, 0], h);
			SafeACopytoC(b2c, h, h, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 5], 1, field); //(A11 + A12) * B22;

			SafeAminusBintoC(a2b, 0, h, a2b, 0, 0, field[0, 0], h);
			SafeAplusBintoC(b2c, 0, 0, b2c, h, 0, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 6], 1, field); //(A21 - A11) * (B11 + B12);

			SafeAminusBintoC(a2b, h, 0, a2b, h, h, field[0, 0], h);
			SafeAplusBintoC(b2c, 0, h, b2c, h, h, field[0, 1], h);
			StrassenMultiplyRun(field[0, 0], field[0, 1], field[0, 1 + 7], 1, field); // (A12 - A22) * (B21 + B22);

			// C11
			for (int i = 0; i < Math.Min(h, R.Vecs); i++) // rows
				for (int j = 0; j < Math.Min(h, R.Cmps); j++) // cols
					R[i, j] = field[0, 1 + 1][i, j] + field[0, 1 + 4][i, j] - field[0, 1 + 5][i, j] + field[0, 1 + 7][i, j];

			// C12
			for (int i = 0; i < Math.Min(h, R.Vecs); i++) // rows
				for (int j = h; j < Math.Min(2 * h, R.Cmps); j++) // cols
					R[i, j] = field[0, 1 + 3][i, j - h] + field[0, 1 + 5][i, j - h];

			// C21
			for (int i = h; i < Math.Min(2 * h, R.Vecs); i++) // rows
				for (int j = 0; j < Math.Min(h, R.Cmps); j++) // cols
					R[i, j] = field[0, 1 + 2][i - h, j] + field[0, 1 + 4][i - h, j];

			// C22
			for (int i = h; i < Math.Min(2 * h, R.Vecs); i++) // rows
				for (int j = h; j < Math.Min(2 * h, R.Cmps); j++) // cols
					R[i, j] = field[0, 1 + 1][i - h, j - h] - field[0, 1 + 2][i - h, j - h] + field[0, 1 + 3][i - h, j - h] + field[0, 1 + 6][i - h, j - h];

			#endregion

			return R;
		}

		#endregion

		#region Equals
		public static bool operator == (Matrix lhs, Matrix rhs)
		{
			return Array_.Equal(lhs.Data, rhs.Data);
		}
		public static bool operator != (Matrix lhs, Matrix rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is Matrix m && m == this;
		}
		public override int GetHashCode()
		{
			return new { Vecs, Cmps, Data }.GetHashCode();
		}
		#endregion
	}

	/// <summary>The LU decomposition of a square matrix</summary>
	[DebuggerDisplay("{Description,nq}")]
	public class MatrixLU
	{
		// Notes:
		//  - The L and U matrices are both stored in 'm_mat'. This means 'LU'
		//    should not be thought of as a logically valid matrix, but as compressed data.

		public MatrixLU(int vec_count, int cmp_count, IEnumerable<double> data)
			:this(new Matrix(vec_count, cmp_count, data))
		{}
		public MatrixLU(Matrix m)
		{
			var N = m.IsSquare ? m.Vecs : throw new Exception("LU decomposition is only possible on square matrices");
			DetOfP = 1;

			// We will store both the L and U matrices in 'mat' since we know
			// L has the form: [1 0] and U has the form: [U U]
			//                 [L 1]                     [0 U]
			var LL = Matrix.Identity(N, N);
			var UU = new Matrix(m);

			// Initialise the unit permutation matrix
			pi = new int[N];
			for (int i = 0; i != pi.Length; ++i)
				pi[i] = i;

			// Decompose 'm' into 'LL' and 'UU'
			for (int v = 0; v != N; ++v)
			{
				// Find the largest component in the vector 'v' to use as the pivot.
				var p = v;
				var max = 0.0;
				for (int i = v; i != N; ++i)
				{
					var val = Math.Abs(UU[v, i]);
					if (val <= max) continue;
					max = val;
					p = i;
				}
				if (max == 0)
					throw new Exception("The matrix is singular");

				// Switch the components of all vectors
				if (p != v)
				{
					Math_.Swap(ref pi[v], ref pi[p]);
					DetOfP = -DetOfP;

					// Swap the components in LL and UU
					for (int i = 0, i0 = v, i1 = p; i != v; ++i, i0 += N, i1 += N)
						Math_.Swap(ref LL.Data[i0], ref LL.Data[i1]);
					for (int i = 0, i0 = v, i1 = p; i != N; ++i, i0 += N, i1 += N)
						Math_.Swap(ref UU.Data[i0], ref UU.Data[i1]);
				}

				// Gaussian eliminate the remaining components of vector 'v'
				for (int c = v + 1; c != N; ++c)
				{
					LL[v,c] = UU[v,c] / UU[v,v];
					for (int i = v; i != N; ++i)
						UU[i,c] -= LL[v,c] * UU[i,v];
				}
			}

			// Combine 'LL' and 'UU' into 'LU'
			LU = UU;
			for (int v = 0; v != N; ++v)
				for (int c = v+1; c != N; ++c)
					LU[v,c] = LL[v,c];

			L = new LProxy(LU);
			U = new UProxy(LU);
		}

		/// <summary>The compressed LU matrix</summary>
		public Matrix LU { get; }

		/// <summary>Matrix dimension (square)</summary>
		public int Dim => LU.Vecs;

		/// <summary>Access the underlying matrix data</summary>
		public double[] Data => LU.Data;

		/// <summary>Access this matrix as a 2D array</summary>
		public double this[int vec, int cmp] => LU[vec, cmp];

		/// <summary>The determinant of the permutation matrix</summary>
		public double DetOfP;

		/// <summary>The permutation row indices</summary>
		public int[] pi;

		/// <summary>Accessor for the lower diagonal matrix</summary>
		public LProxy L;
		public struct LProxy
		{
			private Matrix lu;
			internal LProxy(Matrix lu) => this.lu = lu;
			public double this[int vec, int cmp]
			{
				get
				{
					Util.Assert(vec >= 0 && vec < lu.Vecs);
					Util.Assert(cmp >= 0 && cmp < lu.Cmps);
					return cmp > vec ? lu[vec, cmp] : cmp == vec ? 1 : 0;
				}
			}
		};

		/// <summary>Accessor for the upper diagonal matrix</summary>
		public UProxy U;
		public struct UProxy
		{
			private Matrix lu;
			internal UProxy(Matrix lu) => this.lu = lu;
			public double this[int vec, int cmp]
			{
				get
				{
					Util.Assert(vec >= 0 && vec < lu.Vecs);
					Util.Assert(cmp >= 0 && cmp < lu.Cmps);
					return cmp <= vec ? lu[vec, cmp] : 0;
				}
			}
		};

		/// <summary>A pretty string description of the matrix</summary>
		public string Description => LU.Description;

		#region Functions

		/// <summary>Permutation matrix "P" due to permutation vector "pi"</summary>
		private Matrix PermutationMatrix
		{
			get
			{
				var m = new Matrix(Dim, Dim);
				for (int i = 0; i < Dim; i++)
					m[pi[i], i] = 1;

				return m;
			}
		}

		/// <summary>Return the determinant of this matrix</summary>
		public static double Determinant(MatrixLU lu)
		{
			var det = lu.DetOfP;
			for (int i = 0; i != lu.Dim; ++i)
				det *= lu.U[i,i];

			return det;
		}

		/// <summary>True if 'mat' has an inverse</summary>
		public static bool IsInvertible(MatrixLU lu)
		{
			return !Math_.FEql(Determinant(lu), 0);
		}

		/// <summary>Solves for x in 'A.x = v'</summary>
		public static Matrix Solve(MatrixLU A, Matrix v)
		{
			if (A.Dim != v.Cmps)
				throw new Exception("Solution vector has the wrong dimensions");

			// Switch items in 'v' due to permutation matrix
			var a = new Matrix(1, A.Dim);
			for (int i = 0; i != A.Dim; ++i)
				a[0, i] = v[0, A.pi[i]];

			// Solve for x in 'L.x = b' assuming 'L' is a lower triangular matrix
			var b = new Matrix(1, A.Dim);
			for (int i = 0; i != A.Dim; ++i)
			{
				b[0, i] = a[0, i];
				for (int j = 0; j != i; ++j)
					b[0, i] -= A.L[j, i] * b[0, j];
			}

			// Solve for x in 'U.x = b' assuming 'U' is an upper triangular matrix
			var c = new Matrix(b);
			for (int i = A.Dim; i-- != 0;)
			{
				b[0, i] = c[0, i];
				for (int j = A.Dim - 1; j > i; --j)
					b[0, i] -= A.U[j, i] * b[0, j];

				b[0, i] = b[0, i] / A.U[i, i];
			}

			return b;
		}

		/// <summary>Return the inverse of matrix 'm'</summary>
		public static Matrix Invert(MatrixLU lu)
		{
			Util.Assert(IsInvertible(lu), "Matrix has no inverse");

			var inv = new Matrix(lu.Dim, lu.Dim);
			var elem = new Matrix(1, lu.Dim);
			for (int i = 0; i != lu.Dim; ++i)
			{
				elem[0,i] = 1;
				inv.Vec[i] = Solve(lu, elem);
				elem[0,i] = 0;
			}
			return inv;
		}

		#endregion
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture]
	public class UnitTestMatrix
	{
		[Test]
		public void ValueSemantics()
		{
			var m1 = new Matrix(2, 3, new double[] { 1,2,3, 4,5,6 });
			var m2 = new Matrix(m1);
			Assert.False(ReferenceEquals(m1.Data, m2.Data));
			Assert.True(Equals(m1, m2));
		}

		[Test]
		public void LUDecomposition()
		{
			var m = new MatrixLU(4, 4, new double[] { 1,2,3,1,  4,-5,6,5,  7,8,9,-9,  -10,11,12,0 });
			var res = new Matrix(4, 4, new double[]
			{
				3.0, 0.66666666666667, 0.33333333333333, 0.33333333333333,
				6.0, -9.0, -0.33333333333333, -0.22222222222222,
				9.0, 2.0, -11.333333333333, -0.3921568627451,
				12.0, 3.0, -3.0, -14.509803921569,
			});
			Assert.True(Matrix.FEql(m.LU, res));
		}

		[Test]
		public void Invert()
		{
			var m = new Matrix(4, 4, new double[] { 1, 2, 3, 1, 4, -5, 6, 5, 7, 8, 9, -9, -10, 11, 12, 0 });
			var inv = Matrix.Invert(m);
			var INV = new Matrix(4, 4, new double[]
			{
				+0.258783783783783810, -0.018918918918918920, +0.018243243243243241, -0.068918918918918923,
				+0.414864864864864790, -0.124324324324324320, -0.022972972972972971, -0.024324324324324322,
				-0.164639639639639650, +0.098198198198198194, +0.036261261261261266, +0.048198198198198199,
				+0.405405405405405430, -0.027027027027027029, -0.081081081081081086, -0.027027027027027025,
			});
			Assert.True(Matrix.FEql(inv, INV));
		}

		[Test]
		public void Basic()
		{
			var rng = new Random(1);
			var M = m4x4.Random(-5, +5, rng);

			// Compare with m4x4
			var m = new Matrix(4, 4);
			for (int v = 0; v != 4; ++v)
				for (int c = 0; c != 4; ++c)
					m[v, c] = M[v][c];

			Assert.True(Matrix.FEql(m, M));

			Assert.True(Math_.FEql(M.w.x, m[3,0]));
			Assert.True(Math_.FEql(M.x.w, m[0,3]));
			Assert.True(Math_.FEql(M.z.z, m[2,2]));

			Assert.True(Math_.IsInvertible(M));
			Assert.True(Matrix.IsInvertible(m));

			var M1 = Math_.Invert(M);
			var m1 = Matrix.Invert(m);
			Assert.True(Matrix.FEql(m1, M1));

			var M2 = Math_.Transpose(M);
			var m2 = Matrix.Transpose(m);
			Assert.True(Matrix.FEql(m2, M2));
		}

		[Test]
		public void Multiply0()
		{
			var data0 = new double[]{ 1, 2, 3, 4, 0.1, 0.2, 0.3, 0.4, -4, -3, -2, -1 };
			var data1 = new double[]{ 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4 };
			var rdata = new double[]{ 30, 30, 30, 30, 30, 3, 3, 3, 3, 3, -20, -20, -20, -20, -20 };
			var a2b = new Matrix(3, 4, data0);
			var b2c = new Matrix(4, 5, data1);
			var A2C = new Matrix(3, 5, rdata);
			var a2c = b2c * a2b;
			Assert.True(Matrix.FEql(a2c, A2C));
		}

		[Test]
		public void Multiply1()
		{
			var rng = new Random(1);

			var V0 = v4.Random4(-5, +5, rng);
			var M0 = m4x4.Random(-5, +5, rng);
			var M1 = m4x4.Random(-5, +5, rng);

			var v0 = new Matrix(1, 4, V0.ToArray());
			var m0 = new Matrix(4, 4, M0.ToArray());
			var m1 = new Matrix(4, 4, M1.ToArray());

			Assert.True(Matrix.FEql(m0, M0));
			Assert.True(Matrix.FEql(v0, V0));
			Assert.True(Matrix.FEql(m1, M1));

			var V2 = M0 * V0;
			var v2 = m0 * v0;
			Assert.True(Matrix.FEql(v2, V2));

			var M2 = M0 * M1;
			var m2 = m0 * m1;
			Assert.True(Matrix.FEql(m2, M2));
		}

		[Test]
		public void MultiplyRoundTrip()
		{
			var rng = new Random(1);

			const int SZ = 100;
			var m = new Matrix(SZ, SZ);
			for (int k = 0; k != 10; ++k)
			{
				for (int i = 0; i != m.Vecs; ++i)
					for (int j = 0; j != m.Cmps; ++j)
						m[i, j] = rng.Double(-5.0, +5.0);

				if (Matrix.IsInvertible(m))
				{
					var m_inv = Matrix.Invert(m);

					var i0 = Matrix.Identity(SZ, SZ);
					var i1 = m * m_inv;
					var i2 = m_inv * m;

					Assert.True(Matrix.FEql(i0, i1));
					Assert.True(Matrix.FEql(i0, i2));
					break;
				}
			}
		}

		[Test]
		public void DotProduct()
		{
			var a = new Matrix(1, 3, new double[] { 1.0, 2.0, 3.0 });
			var b = new Matrix(1, 3, new double[] { 3.0, 2.0, 1.0 });
			var r = Matrix.Dot(a, b);
			Assert.True(Math.Abs(r - 10.0) < 1e-10);
		}

		[Test]
		public void EigenSymmetricIdentity()
		{
			// Identity matrix: eigenvalues all 1
			var I = Matrix.Identity(3, 3);
			var result = Matrix.EigenSymmetric(I);

			Assert.True(result.Values.Cmps == 3);
			for (int i = 0; i != 3; ++i)
				Assert.True(Math.Abs(result.Values[0, i] - 1.0) < 1e-10);
		}

		[Test]
		public void EigenSymmetricDiagonal()
		{
			// Diagonal matrix: eigenvalues are the diagonal entries, sorted descending
			var D = new Matrix(3, 3, new double[] { 5, 0, 0, 0, 2, 0, 0, 0, 8 });
			var result = Matrix.EigenSymmetric(D);

			Assert.True(Math.Abs(result.Values[0, 0] - 8.0) < 1e-10);
			Assert.True(Math.Abs(result.Values[0, 1] - 5.0) < 1e-10);
			Assert.True(Math.Abs(result.Values[0, 2] - 2.0) < 1e-10);
		}

		[Test]
		public void EigenSymmetricKnown3x3()
		{
			// M = [2 1 0; 1 3 1; 0 1 2] has eigenvalues 4, 2, 1
			var M = new Matrix(3, 3, new double[] { 2, 1, 0, 1, 3, 1, 0, 1, 2 });
			var result = Matrix.EigenSymmetric(M);

			Assert.True(Math.Abs(result.Values[0, 0] - 4.0) < 1e-8);
			Assert.True(Math.Abs(result.Values[0, 1] - 2.0) < 1e-8);
			Assert.True(Math.Abs(result.Values[0, 2] - 1.0) < 1e-8);

			// Verify eigenvectors: M*v should equal λ*v
			for (int k = 0; k != 3; ++k)
			{
				var lambda = result.Values[0, k];
				for (int r = 0; r != 3; ++r)
				{
					var mv = 0.0;
					for (int c = 0; c != 3; ++c)
						mv += M[r, c] * result.Vectors[c, k];

					var lv = lambda * result.Vectors[r, k];
					Assert.True(Math.Abs(mv - lv) < 1e-8);
				}
			}
		}

		[Test]
		public void EigenSymmetricLarger()
		{
			// 5×5 symmetric matrix
			var M = new Matrix(5, 5, new double[]
			{
				 4, 1, -2,  2, 0,
				 1, 2,  0,  1, 0,
				-2, 0,  3, -2, 0,
				 2, 1, -2,  5, 0,
				 0, 0,  0,  0, 1,
			});
			var result = Matrix.EigenSymmetric(M);

			// Verify A*v = lambda*v for each eigenpair
			for (int k = 0; k != 5; ++k)
			{
				var lambda = result.Values[0, k];
				for (int r = 0; r != 5; ++r)
				{
					var mv = 0.0;
					for (int c = 0; c != 5; ++c)
						mv += M[r, c] * result.Vectors[c, k];

					var lv = lambda * result.Vectors[r, k];
					Assert.True(Math.Abs(mv - lv) < 1e-6);
				}
			}
		}

		[Test]
		public void EigenSymmetricSingleElement()
		{
			// 1×1 matrix: eigenvalue is the single element
			var M = new Matrix(1, 1, new double[] { 7.0 });
			var result = Matrix.EigenSymmetric(M);
			Assert.True(result.Values.Cmps == 1);
			Assert.True(Math.Abs(result.Values[0, 0] - 7.0) < 1e-10);
		}

		[Test]
		public void EigenSymmetricEmpty()
		{
			// 0×0 matrix: empty result
			var M = new Matrix(0, 0);
			var result = Matrix.EigenSymmetric(M);
			Assert.True(result.Values.Cmps == 0);
			Assert.True(result.Vectors.Cmps == 0);
		}

		[Test]
		public void EigenTopKSmall()
		{
			// Top-2 eigenpairs of a 3×3 matrix
			var M = new Matrix(3, 3, new double[] { 2, 1, 0, 1, 3, 1, 0, 1, 2 });
			var result = Matrix.EigenTopK(M, 2);

			Assert.True(result.Values.Cmps == 2);
			Assert.True(result.Vectors.Vecs == 3);
			Assert.True(result.Vectors.Cmps == 2);

			Assert.True(Math.Abs(result.Values[0, 0] - 4.0) < 1e-8);
			Assert.True(Math.Abs(result.Values[0, 1] - 2.0) < 1e-8);

			// Verify A*v = lambda*v
			for (int k = 0; k != 2; ++k)
			{
				var lambda = result.Values[0, k];
				for (int r = 0; r != 3; ++r)
				{
					var mv = 0.0;
					for (int c = 0; c != 3; ++c)
						mv += M[r, c] * result.Vectors[c, k];

					var lv = lambda * result.Vectors[r, k];
					Assert.True(Math.Abs(mv - lv) < 1e-8);
				}
			}
		}

		[Test]
		public void EigenTopKDoubleCentered()
		{
			// Regression test: the classical MDS Gram matrix B = -0.5·J·D²·J has 1 in its null
			// space (B·1 = 0). A Lanczos implementation starting from [1,…,1]/√N collapses to
			// zero eigenvalues. EigenTopK must return the true top eigenvalues regardless.
			// Use N > 32 so the Lanczos path is exercised (small-N falls back to full decomp).

			const int N = 64;

			// Build a distance matrix D from random points in 3D, then double-centre D² -> B.
			var rng = new Random(1234);
			var pts = new double[N * 3];
			for (int i = 0; i != N * 3; ++i)
				pts[i] = rng.NextDouble() * 10.0;

			var D2 = new Matrix(N, N);
			for (int i = 0; i != N; ++i)
			{
				for (int j = 0; j != N; ++j)
				{
					var dx = pts[i * 3 + 0] - pts[j * 3 + 0];
					var dy = pts[i * 3 + 1] - pts[j * 3 + 1];
					var dz = pts[i * 3 + 2] - pts[j * 3 + 2];
					D2[i, j] = dx * dx + dy * dy + dz * dz;
				}
			}

			// Double centre: B = -0.5 * J * D² * J  where  J = I - (1/N)·11ᵀ
			var row_mean = new double[N];
			var col_mean = new double[N];
			var grand_mean = 0.0;
			for (int i = 0; i != N; ++i)
			{
				for (int j = 0; j != N; ++j)
				{
					row_mean[i] += D2[i, j];
					col_mean[j] += D2[i, j];
					grand_mean += D2[i, j];
				}
			}
			for (int i = 0; i != N; ++i) { row_mean[i] /= N; col_mean[i] /= N; }
			grand_mean /= (N * N);

			var B = new Matrix(N, N);
			for (int i = 0; i != N; ++i)
				for (int j = 0; j != N; ++j)
					B[i, j] = -0.5 * (D2[i, j] - row_mean[i] - col_mean[j] + grand_mean);

			// Points were generated in 3D -> B has rank ≤ 3. Ask for top 3 eigenpairs.
			var result = Matrix.EigenTopK(B, 3);
			Assert.True(result.Values.Cmps == 3);

			// For a rank-3 Euclidean configuration, the top 3 eigenvalues must be clearly positive.
			// If Lanczos had collapsed on the null direction, all three would be ~0.
			Assert.True(result.Values[0, 0] > 1.0);
			Assert.True(result.Values[0, 1] > 1.0);
			Assert.True(result.Values[0, 2] > 1.0);

			// And they must be in descending order.
			Assert.True(result.Values[0, 0] >= result.Values[0, 1]);
			Assert.True(result.Values[0, 1] >= result.Values[0, 2]);

			// Verify A*v ≈ λ*v for each returned eigenpair (relative tolerance to allow for
			// Lanczos convergence slack vs the exact EigenSymmetric result).
			for (int k = 0; k != 3; ++k)
			{
				var lambda = result.Values[0, k];
				var err2 = 0.0;
				var norm2 = 0.0;
				for (int r = 0; r != N; ++r)
				{
					var mv = 0.0;
					for (int c = 0; c != N; ++c)
						mv += B[r, c] * result.Vectors[c, k];

					var lv = lambda * result.Vectors[r, k];
					err2 += (mv - lv) * (mv - lv);
					norm2 += lv * lv;
				}
				Assert.True(Math.Sqrt(err2 / norm2) < 1e-4);
			}
		}
	}
}
#endif