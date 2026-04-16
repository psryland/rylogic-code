//***************************************************
// Multidimensional Scaling (MDS)
//  Copyright (c) Rylogic Ltd 2025
//***************************************************
// Classical (Torgerson) MDS: embeds items into low-dimensional space preserving pairwise distances.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using Rylogic.Maths;

namespace Rylogic.Algorithm.MDS
{
	/// <summary>Configuration for MDS embedding</summary>
	public class Config
	{
		/// <summary>Number of output dimensions (1, 2, or 3). Unused v4 components are zero-filled, w = 1.</summary>
		public int Dimensions { get; set; } = 3;
	}

	/// <summary>Classical (Torgerson) Multidimensional Scaling</summary>
	public static class MultiDimensionalScaling
	{
		/// <summary>
		/// Embed N items into low-dimensional space preserving pairwise distances.
		/// 'dist(items[i], items[j])' must return a double dissimilarity >= 0.
		/// Returns an array of v4 points with w=1. Unused dimensions are zero.
		/// </summary>
		public static v4[] Embed<T>(IReadOnlyList<T> items, Func<T, T, double> dist, Config? config = null)
		{
			var cfg = config ?? new Config();
			Debug.Assert(cfg.Dimensions >= 1 && cfg.Dimensions <= 3);

			if (items.Count == 0)
				return Array.Empty<v4>();

			if (items.Count == 1)
				return new[] { new v4(0, 0, 0, 1) };

			var n = items.Count;
			var dim = Math.Min(cfg.Dimensions, n - 1);

			// Step 1: Build N×N squared distance matrix D²
			var D2 = new double[n * n];
			for (int i = 0; i != n; ++i)
			{
				for (int j = i + 1; j != n; ++j)
				{
					var d = dist(items[i], items[j]);
					var d2 = d * d;
					D2[i * n + j] = d2;
					D2[j * n + i] = d2;
				}
			}

			// Step 2: Double centering to get the inner-product matrix B = -1/2 * J * D² * J
			// where J = I - (1/N) * 11ᵀ. Equivalent to: B[i][j] = -1/2 * (D²[i][j] - row_mean[i] - col_mean[j] + grand_mean)
			var row_mean = new double[n];
			var grand_mean = 0.0;
			for (int i = 0; i != n; ++i)
			{
				for (int j = 0; j != n; ++j)
					row_mean[i] += D2[i * n + j];

				row_mean[i] /= n;
				grand_mean += row_mean[i];
			}
			grand_mean /= n;

			// Step 3: Build the inner-product matrix B as a Matrix and eigendecompose.
			var B = new Matrix(n, n);
			for (int i = 0; i != n; ++i)
				for (int j = 0; j != n; ++j)
					B[i, j] = -0.5 * (D2[i * n + j] - row_mean[i] - row_mean[j] + grand_mean);

			var eigen = Matrix.EigenTopK(B, dim);

			// Step 4: Build output coordinates from top 'dim' eigenvectors scaled by sqrt(eigenvalue)
			var result = new v4[n];
			for (int i = 0; i != n; ++i)
			{
				// Clamp negative eigenvalues to zero (numerical noise from non-Euclidean distances)
				var x = dim >= 1 ? (float)(Math.Sqrt(Math.Max(0.0, eigen.Values[0, 0])) * eigen.Vectors[i, 0]) : 0f;
				var y = dim >= 2 ? (float)(Math.Sqrt(Math.Max(0.0, eigen.Values[0, 1])) * eigen.Vectors[i, 1]) : 0f;
				var z = dim >= 3 ? (float)(Math.Sqrt(Math.Max(0.0, eigen.Values[0, 2])) * eigen.Vectors[i, 2]) : 0f;
				result[i] = new v4(x, y, z, 1);
			}
			return result;
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Algorithm.MDS;

	[TestFixture]
	public class UnitTestMDS
	{
		[Test]
		public void Empty()
		{
			var result = MultiDimensionalScaling.Embed(Array.Empty<int>(), (a, b) => 0.0);
			Assert.True(result.Length == 0);
		}

		[Test]
		public void Single()
		{
			var items = new[] { 42 };
			var result = MultiDimensionalScaling.Embed(items, (a, b) => 0.0);
			Assert.True(result.Length == 1);
			Assert.True(Math.Abs(result[0].w - 1.0f) < 1e-5f);
		}

		[Test]
		public void KnownSquare()
		{
			// Four points forming a unit square. Distances: adjacent=1, diagonal=sqrt(2)
			var pts = new[] { (0f, 0f), (1f, 0f), (1f, 1f), (0f, 1f) };
			var euclidean = new Func<(float, float), (float, float), double>((a, b) =>
			{
				var dx = a.Item1 - b.Item1;
				var dy = a.Item2 - b.Item2;
				return Math.Sqrt(dx * dx + dy * dy);
			});
			var result = MultiDimensionalScaling.Embed(pts, euclidean, new Config { Dimensions = 2 });
			Assert.True(result.Length == 4);

			// Verify pairwise distances are preserved (up to rotation/reflection)
			for (int i = 0; i != 4; ++i)
			{
				for (int j = i + 1; j != 4; ++j)
				{
					var orig_d = euclidean(pts[i], pts[j]);
					var dx = result[i].x - result[j].x;
					var dy = result[i].y - result[j].y;
					var embed_d = Math.Sqrt(dx * dx + dy * dy);
					Assert.True(Math.Abs(orig_d - embed_d) < 0.05);
				}
			}

			// Verify w=1 and z=0 for 2D embedding
			for (int i = 0; i != result.Length; ++i)
			{
				Assert.True(Math.Abs(result[i].z) < 1e-4f);
				Assert.True(Math.Abs(result[i].w - 1.0f) < 1e-5f);
			}
		}
	}
}
#endif
