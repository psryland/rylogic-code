//***************************************************
// Multidimensional Scaling (MDS)
//  Copyright (c) Rylogic Ltd 2025
//***************************************************
// Classical (Torgerson) MDS: embeds items into low-dimensional space preserving pairwise distances.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Rylogic.Maths;

namespace Rylogic.Algorithm.MDS
{
	/// <summary>Classical (Torgerson) Multidimensional Scaling</summary>
	public static class MultiDimensionalScaling
	{
		/// <summary>Configuration for MDS embedding</summary>
		public class Config
		{
			/// <summary>Number of output dimensions (1, 2, or 3). Unused v4 components are zero-filled, w = 1.</summary>
			public int Dimensions = 3;
		}

		/// <summary>
		/// Embed N items into low-dimensional space preserving pairwise distances, where some distances
		/// may be unknown. A 'dist' result of null indicates "this pair's distance is undefined"; such
		/// pairs are imputed as the shortest-path distance through the graph of known pairs (a.k.a. ISOMAP).
		/// This is useful when only a subset of the distance matrix is measured — e.g. a transition graph
		/// where edges exist between a small fraction of pairs.
		/// Throws InvalidOperationException if the graph of known pairs has disconnected components
		/// (i.e. some pair cannot be reached via any chain of known distances).
		/// </summary>
		public static v4[] Embed<T>(IReadOnlyList<T> items, Func<T, T, double?> dist, Config? config = null)
		{
			var cfg = config ?? new Config();
			var n = items.Count;
			if (n == 0) return Array.Empty<v4>();
			if (n == 1) return new[] { new v4(0, 0, 0, 1) };

			var complete = ImputeMissing(items, dist);
			return EmbedFromMatrix(n, complete, cfg);
		}

		/// <summary>
		/// Fill in missing pairwise distances using Floyd-Warshall shortest paths through the graph
		/// of known pairs. Returns an N*N symmetric matrix (row-major) with zero on the diagonal.
		/// A null return from 'dist' means the pair is unknown.
		/// </summary>
		public static double[] ImputeMissing<T>(IReadOnlyList<T> items, Func<T, T, double?> dist)
		{
			var n = items.Count;
			var d = new double[n * n];
			const double INF = double.PositiveInfinity;

			// Seed with direct observations (null -> INF); diagonal = 0
			for (int i = 0; i != n; ++i)
			{
				d[i * n + i] = 0.0;
				for (int j = i + 1; j != n; ++j)
				{
					var v = dist(items[i], items[j]);
					var dv = v.HasValue ? v.Value : INF;
					d[i * n + j] = dv;
					d[j * n + i] = dv;
				}
			}

			// Floyd-Warshall: d[i,j] = min(d[i,j], d[i,k] + d[k,j])
			for (int k = 0; k != n; ++k)
			{
				for (int i = 0; i != n; ++i)
				{
					var dik = d[i * n + k];
					if (double.IsPositiveInfinity(dik)) continue;
					for (int j = 0; j != n; ++j)
					{
						var via = dik + d[k * n + j];
						if (via < d[i * n + j])
							d[i * n + j] = via;
					}
				}
			}

			// Any remaining INF means the graph is disconnected
			for (int i = 0; i != n; ++i)
			{
				for (int j = i + 1; j != n; ++j)
				{
					if (double.IsPositiveInfinity(d[i * n + j]))
						throw new InvalidOperationException($"Cannot impute distance: items [{i}] and [{j}] are not connected via any chain of known pairs.");
				}
			}
			return d;
		}

		/// <summary>
		/// Embed N items into low-dimensional space preserving pairwise distances.
		/// 'dist(items[i], items[j])' must return a double dissimilarity >= 0.
		/// Returns an array of v4 points with w=1. Unused dimensions are zero.
		/// </summary>
		public static v4[] Embed<T>(IReadOnlyList<T> items, Func<T, T, double> dist, Config? config = null)
		{
			var cfg = config ?? new Config();
			Debug.Assert(cfg.Dimensions >= 1 && cfg.Dimensions <= 3);

			var n = items.Count;
			if (n == 0) return Array.Empty<v4>();
			if (n == 1) return new[] { new v4(0, 0, 0, 1) };

			// Build the complete distance matrix and hand off to EmbedFromMatrix
			var D = new double[n * n];
			for (int i = 0; i != n; ++i)
			{
				for (int j = i + 1; j != n; ++j)
				{
					var dv = dist(items[i], items[j]);
					D[i * n + j] = dv;
					D[j * n + i] = dv;
				}
			}
			return EmbedFromMatrix(n, D, cfg);
		}

		/// <summary>
		/// Build the classical MDS Gram matrix B = -1/2 * J * D² * J from an N*N distance matrix D.
		/// J = I - (1/N) * 11ᵀ is the centring matrix. Equivalent to:
		///   B[i,j] = -1/2 * (D²[i,j] - row_mean[i] - col_mean[j] + grand_mean)
		/// </summary>
		private static Matrix BuildGramMatrix(int n, double[] D)
		{
			// Square the distance matrix
			var D2 = new double[n * n];
			for (int i = 0; i != n; ++i)
				for (int j = 0; j != n; ++j)
					D2[i * n + j] = D[i * n + j] * D[i * n + j];

			// Row means + grand mean (D² is symmetric, so col_mean == row_mean)
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

			var B = new Matrix(n, n);
			for (int i = 0; i != n; ++i)
				for (int j = 0; j != n; ++j)
					B[i, j] = -0.5 * (D2[i * n + j] - row_mean[i] - row_mean[j] + grand_mean);

			return B;
		}

		/// <summary>Core Torgerson MDS on an N*N distance matrix (row-major, symmetric, zero diagonal).</summary>
		private static v4[] EmbedFromMatrix(int n, double[] D, Config cfg)
		{
			Debug.Assert(cfg.Dimensions >= 1 && cfg.Dimensions <= 3);
			var dim = Math.Min(cfg.Dimensions, n - 1);

			var B = BuildGramMatrix(n, D);

			// Only the top 'dim' eigenpairs are needed. EigenTopK uses Lanczos for large N,
			// which is O(dim · N²) per restart rather than O(N³) for the full decomposition.
			// Critical for MDS's intended use with large point sets.
			var eigen = Matrix.EigenTopK(B, dim);

			// Build output coordinates from top 'dim' eigenvectors scaled by sqrt(eigenvalue)
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

		/// <summary>
		/// Diagnostic report on the eigenvalue spectrum of the MDS Gram matrix B.
		/// Tells you the theoretical fit ceiling of a classical-MDS embedding at a given dimension,
		/// and flags non-Euclidean distance data (many large negative eigenvalues).
		/// </summary>
		public class SpectrumReport
		{
			/// <summary>All eigenvalues of B, sorted descending (may include negatives for non-Euclidean distances)</summary>
			public double[] Eigenvalues { get; set; } = Array.Empty<double>();

			/// <summary>Sum of positive eigenvalues — total variance available to a Euclidean embedding</summary>
			public double PositiveVariance { get; set; }

			/// <summary>Sum of absolute eigenvalues — total "signal" in B including non-Euclidean deviations</summary>
			public double TotalAbsVariance { get; set; }

			/// <summary>Number of negative eigenvalues (indicator of non-Euclidean distances)</summary>
			public int NegativeCount { get; set; }

			/// <summary>Magnitude of the most-negative eigenvalue (strength of the worst triangle-inequality violation)</summary>
			public double LargestNegative { get; set; }

			/// <summary>
			/// Upper bound on the fraction of variance a 'dim'-dimensional embedding can capture.
			/// Closer to 1 means a clean fit is theoretically possible; low values mean the data
			/// intrinsically needs more dimensions or is strongly non-Euclidean.
			/// </summary>
			public double FitCeiling(int dim)
			{
				if (Eigenvalues.Length == 0 || TotalAbsVariance <= 0) return 0.0;
				double captured = 0.0;
				int k = Math.Min(dim, Eigenvalues.Length);
				for (int i = 0; i != k; ++i)
					captured += Math.Max(0.0, Eigenvalues[i]);
				return captured / TotalAbsVariance;
			}

			/// <summary>Summary string describing the spectrum</summary>
			public string Summary()
			{
				var sb = new StringBuilder();
				sb.AppendLine("MDS eigenvalue spectrum:");
				sb.Append("  Top eigenvalues:   ");
				int show = Math.Min(Eigenvalues.Length, 10);
				for (int i = 0; i != show; ++i)
					sb.Append($"{Eigenvalues[i]:G3} ");
				sb.AppendLine();
				sb.AppendLine($"  Positive variance: {PositiveVariance:G4}");
				sb.AppendLine($"  Total |variance|:  {TotalAbsVariance:G4}");
				sb.AppendLine($"  Negative λ count:  {NegativeCount} (largest |λ|: {LargestNegative:G4})");
				sb.AppendLine($"  Fit ceiling:       1D={FitCeiling(1):P1}  2D={FitCeiling(2):P1}  3D={FitCeiling(3):P1}  5D={FitCeiling(5):P1}  10D={FitCeiling(10):P1}");
				return sb.ToString();
			}
		}

		/// <summary>
		/// Compute the full eigenvalue spectrum of the MDS Gram matrix as a diagnostic.
		/// Use this to decide whether a low-dimensional classical-MDS embedding can possibly fit your
		/// data well, and whether the distances are badly non-Euclidean (many large negative eigenvalues
		/// indicate triangle-inequality violations).
		/// </summary>
		public static SpectrumReport EigenSpectrum<T>(IReadOnlyList<T> items, Func<T, T, double> dist)
		{
			var n = items.Count;
			if (n < 2) return new SpectrumReport();

			var D = new double[n * n];
			for (int i = 0; i != n; ++i)
			{
				for (int j = i + 1; j != n; ++j)
				{
					var d = dist(items[i], items[j]);
					D[i * n + j] = d;
					D[j * n + i] = d;
				}
			}
			return EigenSpectrumFromMatrix(n, D);
		}

		/// <summary>
		/// Eigenvalue spectrum diagnostic with sparse-observation imputation (ISOMAP). Missing pairs
		/// (dist returns null) are filled in via shortest paths before the spectrum is computed.
		/// </summary>
		public static SpectrumReport EigenSpectrum<T>(IReadOnlyList<T> items, Func<T, T, double?> dist)
		{
			var n = items.Count;
			if (n < 2) return new SpectrumReport();

			var D = ImputeMissing(items, dist);
			return EigenSpectrumFromMatrix(n, D);
		}

		/// <summary></summary>
		private static SpectrumReport EigenSpectrumFromMatrix(int n, double[] D)
		{
			var B = BuildGramMatrix(n, D);

			// Full decomposition: we want *all* eigenvalues so we can see the negative ones and the
			// long tail. Diagnostic only, so O(N³) is acceptable.
			var eigen = Matrix.EigenSymmetric(B);

			var report = new SpectrumReport();
			report.Eigenvalues = new double[n];
			for (int i = 0; i != n; ++i)
				report.Eigenvalues[i] = eigen.Values[0, i];

			foreach (var ev in report.Eigenvalues)
			{
				var abs = Math.Abs(ev);
				report.TotalAbsVariance += abs;
				if (ev > 0)
				{
					report.PositiveVariance += ev;
				}
				else if (ev < 0)
				{
					++report.NegativeCount;
					if (abs > report.LargestNegative)
						report.LargestNegative = abs;
				}
			}

			return report;
		}

		/// <summary>Fit quality of an MDS embedding vs the target pairwise distances</summary>
		public class FitReport
		{
			/// <summary>Number of unique off-diagonal pairs compared (N*(N-1)/2)</summary>
			public int PairCount { get; set; }

			/// <summary>Number of pairs that were flagged as "observed" by the optional predicate (equals PairCount if no predicate was supplied)</summary>
			public int ObservedPairCount { get; set; }

			/// <summary>Optimal uniform scale factor: Embedded * Alpha best approximates Target in a least-squares sense</summary>
			public double Alpha { get; set; }

			/// <summary>Kruskal stress-1 over all pairs after applying Alpha. &lt;0.05 excellent, &lt;0.10 good, &lt;0.20 fair, &gt;0.20 poor.</summary>
			public double Stress { get; set; }

			/// <summary>Kruskal stress-1 over just the observed pairs (0 if no predicate was supplied)</summary>
			public double StressObserved { get; set; }

			/// <summary>Pearson correlation coefficient between embedded and target distances</summary>
			public double Pearson { get; set; }

			/// <summary>Mean |Alpha*d_embedded - d_target| over all pairs</summary>
			public double MeanAbsError { get; set; }

			/// <summary>Max |Alpha*d_embedded - d_target| over all pairs</summary>
			public double MaxAbsError { get; set; }

			/// <summary>A summary string describing the fit report</summary>
			public string Summary()
			{
				var summary = new StringBuilder();
				summary.AppendLine("MDS fit report:");
				summary.AppendLine($"  Pairs:           {PairCount} (observed: {ObservedPairCount}, {100.0 * ObservedPairCount / Math.Max(1, PairCount):F1}%)");
				summary.AppendLine($"  Scale alpha:     {Alpha:G4} (embedded * alpha ≈ target)");
				summary.AppendLine($"  Kruskal stress:  {Stress:F4} ({Grade(Stress)}) (all pairs)");
				summary.AppendLine($"                   {StressObserved:F4} ({Grade(StressObserved)}) (observed pairs only)");
				summary.AppendLine($"  Pearson r:       {Pearson:F4}");
				summary.AppendLine($"  Mean |err|:      {MeanAbsError:G4}");
				summary.AppendLine($"  Max  |err|:      {MaxAbsError:G4}");

				return summary.ToString();
				
				// Rough Kruskal interpretation: <0.05 excellent, <0.10 good, <0.20 fair, >0.20 poor
				string Grade(double s) =>
					s < 0.05 ? "Excellent" :
					s < 0.10 ? "Good" :
					s < 0.20 ? "Fair" :
					"Poor";
			}
		}

		/// <summary>
		/// Measure how well an MDS embedding preserves the pairwise distances of the input items.
		/// Because MDS output has an arbitrary scale, we first compute the optimal uniform scale factor
		///   alpha = sum(d_emb * d_tgt) / sum(d_emb^2)
		/// and report stress/Pearson/errors using that scale. The optional 'is_observed' predicate lets
		/// callers distinguish measured ground-truth pairs from padded or imputed pairs; when supplied,
		/// an additional stress figure over just the observed pairs is reported.
		/// </summary>
		public static FitReport MeasureFit<T>(IReadOnlyList<T> items, IReadOnlyList<v4> positions, Func<T, T, double> dist, Func<T, T, bool>? is_observed = null)
		{
			var report = new FitReport();

			var n = items.Count;
			Debug.Assert(positions.Count >= n);
			if (n < 2)
				return report;

			// Pass 1: gather sums needed for the optimal scale
			double sum_emb_tgt = 0, sum_emb_sq = 0, sum_tgt_sq = 0;
			int pair_count = 0;
			int obs_count = 0;
			for (int i = 0; i != n; ++i)
			{
				for (int j = i + 1; j != n; ++j)
				{
					double d_emb = (positions[i] - positions[j]).Length;
					double d_tgt = dist(items[i], items[j]);

					sum_emb_tgt += d_emb * d_tgt;
					sum_emb_sq  += d_emb * d_emb;
					sum_tgt_sq  += d_tgt * d_tgt;
					++pair_count;

					if (is_observed != null && is_observed(items[i], items[j]))
						++obs_count;
				}
			}

			double alpha = sum_emb_sq > 0 ? sum_emb_tgt / sum_emb_sq : 1.0;

			// Pass 2: compute stress, Pearson, and error stats using alpha
			double num_all = 0;
			double num_obs = 0, den_obs = 0;
			double abs_err_sum = 0, abs_err_max = 0;
			double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0, sum_yy = 0;

			for (int i = 0; i != n; ++i)
			{
				for (int j = i + 1; j != n; ++j)
				{
					double d_emb = (positions[i] - positions[j]).Length * alpha;
					double d_tgt = dist(items[i], items[j]);
					double err = d_emb - d_tgt;

					num_all += err * err;
					double aerr = Math.Abs(err);
					abs_err_sum += aerr;
					if (aerr > abs_err_max) abs_err_max = aerr;

					if (is_observed != null && is_observed(items[i], items[j]))
					{
						num_obs += err * err;
						den_obs += d_tgt * d_tgt;
					}

					sum_x  += d_emb;
					sum_y  += d_tgt;
					sum_xy += d_emb * d_tgt;
					sum_xx += d_emb * d_emb;
					sum_yy += d_tgt * d_tgt;
				}
			}

			double stress_all = sum_tgt_sq > 0 ? Math.Sqrt(num_all / sum_tgt_sq) : 0.0;
			double stress_obs = den_obs > 0 ? Math.Sqrt(num_obs / den_obs) : 0.0;

			double pearson = 0.0;
			double cov = sum_xy - (sum_x * sum_y) / pair_count;
			double var_x = sum_xx - (sum_x * sum_x) / pair_count;
			double var_y = sum_yy - (sum_y * sum_y) / pair_count;
			if (var_x > 0 && var_y > 0)
				pearson = cov / Math.Sqrt(var_x * var_y);

			report.PairCount = pair_count;
			report.ObservedPairCount = is_observed != null ? obs_count : pair_count;
			report.Alpha = alpha;
			report.Stress = stress_all;
			report.StressObserved = stress_obs;
			report.Pearson = pearson;
			report.MeanAbsError = abs_err_sum / pair_count;
			report.MaxAbsError = abs_err_max;
			return report;
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
		public void EigenSpectrumPlanarPoints()
		{
			// Points sampled from a 2D plane embedded in 3D space -> top 2 eigenvalues should dominate,
			// the rest should be ~0, and there should be no significant negative eigenvalues.
			var rng = new Random(1);
			var pts = new (double x, double y, double z)[30];
			for (int i = 0; i != pts.Length; ++i)
				pts[i] = (rng.NextDouble() * 10, rng.NextDouble() * 10, 0.0);

			var spectrum = MultiDimensionalScaling.EigenSpectrum(pts, (a, b) =>
			{
				var dx = a.x - b.x; var dy = a.y - b.y; var dz = a.z - b.z;
				return Math.Sqrt(dx * dx + dy * dy + dz * dz);
			});

			Assert.True(spectrum.Eigenvalues.Length == pts.Length);

			// Top 2 eigenvalues capture essentially all the positive variance (planar data).
			Assert.True(spectrum.FitCeiling(2) > 0.99);

			// Euclidean distances -> negative eigenvalues only from numerical noise, all near zero.
			Assert.True(spectrum.LargestNegative < 1e-6 * spectrum.PositiveVariance);
		}

		[Test]
		public void EmbedWithMissingDistances()
		{
			// Square: pretend only adjacent-edge distances are known; diagonals are null.
			// ISOMAP should impute the diagonal as 1+1=2 (path through a corner).
			var pts = new[] { (0f, 0f), (1f, 0f), (1f, 1f), (0f, 1f) };
			Func<(float, float), (float, float), double?> sparse = (a, b) =>
			{
				var dx = a.Item1 - b.Item1;
				var dy = a.Item2 - b.Item2;
				var d = Math.Sqrt(dx * dx + dy * dy);
				return d < 1.1 ? d : (double?)null; // only report unit edges
			};

			// Imputation step: diagonals should be filled as 2 (path-through)
			var mat = MultiDimensionalScaling.ImputeMissing(pts, sparse);
			Assert.True(Math.Abs(mat[0 * 4 + 1] - 1.0) < 1e-6);
			Assert.True(Math.Abs(mat[0 * 4 + 2] - 2.0) < 1e-6);
			Assert.True(Math.Abs(mat[1 * 4 + 3] - 2.0) < 1e-6);

			// Embedding should succeed and produce 4 finite points; exact layout is a compromise
			// because the imputed distances aren't Euclidean-consistent (4 edges of 1 and 2 diagonals of 2).
			var result = MultiDimensionalScaling.Embed(pts, sparse, new MultiDimensionalScaling.Config { Dimensions = 2 });
			Assert.True(result.Length == 4);
			for (int i = 0; i != 4; ++i)
			{
				Assert.True(!float.IsNaN(result[i].x) && !float.IsInfinity(result[i].x));
				Assert.True(!float.IsNaN(result[i].y) && !float.IsInfinity(result[i].y));
			}
		}

		[Test]
		public void ImputeMissingDisconnected()
		{
			// Two components with no bridge -> should throw
			var items = new[] { 0, 1, 2, 3 };
			Func<int, int, double?> dist = (a, b) =>
			{
				if ((a == 0 && b == 1) || (a == 1 && b == 0)) return 1.0;
				if ((a == 2 && b == 3) || (a == 3 && b == 2)) return 1.0;
				return null;
			};
			var threw = false;
			try { MultiDimensionalScaling.ImputeMissing(items, dist); }
			catch (InvalidOperationException) { threw = true; }
			Assert.True(threw);
		}

		[Test]
		public void FitPerfectEmbedding()
		{
			// Unit square — MDS should reconstruct distances well; stress and error should be tiny.
			var pts = new[] { (0f, 0f), (1f, 0f), (1f, 1f), (0f, 1f) };
			var euclidean = new Func<(float, float), (float, float), double>((a, b) =>
			{
				var dx = a.Item1 - b.Item1;
				var dy = a.Item2 - b.Item2;
				return Math.Sqrt(dx * dx + dy * dy);
			});
			var result = MultiDimensionalScaling.Embed(pts, euclidean, new MultiDimensionalScaling.Config { Dimensions = 2 });
			var report = MultiDimensionalScaling.MeasureFit(pts, result, euclidean);

			Assert.True(report.PairCount == 6);
			Assert.True(report.ObservedPairCount == 6);
			Assert.True(Math.Abs(report.Alpha - 1.0) < 0.05);
			Assert.True(report.Stress < 0.05);
			Assert.True(report.Pearson > 0.99);
			Assert.True(report.MaxAbsError < 0.05);
		}

		[Test]
		public void FitWithObservedPredicate()
		{
			// When some pairs are padded with an arbitrary "max" distance, the observed-pair stress
			// should be noticeably lower than the all-pair stress.
			var pts = new[] { (0f, 0f), (1f, 0f), (1f, 1f), (0f, 1f) };
			var real = new Func<(float, float), (float, float), double>((a, b) =>
			{
				var dx = a.Item1 - b.Item1;
				var dy = a.Item2 - b.Item2;
				return Math.Sqrt(dx * dx + dy * dy);
			});

			// Pretend only adjacent pairs are "observed"; diagonals are padded to a large value.
			Func<(float, float), (float, float), bool> adjacent = (a, b) =>
			{
				var d = real(a, b);
				return d < 1.1; // only the 4 unit edges
			};
			Func<(float, float), (float, float), double> padded = (a, b) => adjacent(a, b) ? real(a, b) : 10.0;

			var result = MultiDimensionalScaling.Embed(pts, padded, new MultiDimensionalScaling.Config { Dimensions = 2 });
			var report = MultiDimensionalScaling.MeasureFit(pts, result, padded, adjacent);

			Assert.True(report.PairCount == 6);
			Assert.True(report.ObservedPairCount == 4);
			Assert.True(report.Stress >= 0.0);
			Assert.True(report.StressObserved >= 0.0);
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
			var result = MultiDimensionalScaling.Embed(pts, euclidean, new MultiDimensionalScaling.Config { Dimensions = 2 });
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
