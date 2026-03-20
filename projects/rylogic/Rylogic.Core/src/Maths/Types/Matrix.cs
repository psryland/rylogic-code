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
	}
}
#endif