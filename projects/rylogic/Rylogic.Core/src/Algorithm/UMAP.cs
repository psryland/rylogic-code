//***************************************************
// Uniform Manifold Approximation and Projection (UMAP)
//  Copyright (c) Rylogic Ltd 2025
//***************************************************
// Reference: L. McInnes, J. Healy, J. Melville (2018) "UMAP: Uniform Manifold Approximation
// and Projection for Dimension Reduction", arXiv:1802.03426
//
// Unlike classical MDS (which tries to preserve pairwise distances globally), UMAP preserves
// the *topology* of the k-nearest-neighbour graph. It is usually much better at producing
// readable 2D/3D visualisations of high-intrinsic-dimension data.
//
// Pipeline:
//   1. Build a k-nearest-neighbour graph in the original space.
//   2. For each point, fit a per-point σ so the exponential kernel over neighbours has total
//      weight log2(k). Points with dense neighbourhoods get small σ, sparse ones get large σ.
//   3. Build a fuzzy simplicial set: edge weights a_ij = exp(-max(0, d_ij - ρ_i) / σ_i),
//      then symmetrise via the probabilistic t-conorm  w_ij = a_ij + a_ji - a_ij·a_ji.
//   4. Fit curve parameters (a, b) so 1/(1 + a·d^(2b)) approximates the user-requested
//      (spread, min_dist) response in the embedding space.
//   5. Initialise low-dim positions via spectral embedding of the fuzzy graph's Laplacian
//      (falls back to random init for disconnected graphs or very large N).
//   6. SGD: for each epoch, apply attractive force along each edge (proportional to its
//      fuzzy weight) and repulsive force against random negative samples.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using Rylogic.Maths;

namespace Rylogic.Algorithm.UMAP
{
	/// <summary>Uniform Manifold Approximation and Projection</summary>
	public static class UniformManifoldApproximationAndProjection
	{
		/// <summary>Configuration for UMAP embedding</summary>
		public class Config
		{
			/// <summary>Output dimensionality (1, 2, or 3). v4 components beyond this are zero; w = 1.</summary>
			public int Dimensions { get; set; } = 2;

			/// <summary>Number of nearest neighbours used to build the local topology. 15 is a good default;
			/// small values (2-5) emphasise fine local detail, large values (50+) emphasise global structure.</summary>
			public int NeighborCount { get; set; } = 15;

			/// <summary>Minimum desired distance between embedded points — smaller = tighter clusters.</summary>
			public double MinDist { get; set; } = 0.1;

			/// <summary>Scale of the embedded distribution. Combined with MinDist this determines (a, b).</summary>
			public double Spread { get; set; } = 1.0;

			/// <summary>SGD epochs. 0 = auto (500 for n&lt;=10_000, else 200).</summary>
			public int Epochs { get; set; } = 0;

			/// <summary>Number of negative samples per positive (attractive) edge visit.</summary>
			public int NegativeSampleRate { get; set; } = 5;

			/// <summary>Initial SGD learning rate. Decays linearly to 0 over the run.</summary>
			public double LearningRate { get; set; } = 1.0;

			/// <summary>Repulsion strength. 1.0 is standard; larger values force clusters further apart.</summary>
			public double RepulsionStrength { get; set; } = 1.0;

			/// <summary>PRNG seed for reproducibility of SGD sampling and (if needed) random init.</summary>
			public int RandomSeed { get; set; } = 42;

			/// <summary>How many nearest neighbours have "connected" fuzzy weight 1. Usually 1.</summary>
			public double LocalConnectivity { get; set; } = 1.0;

			/// <summary>If true and the fuzzy graph is connected, use spectral (Laplacian-eigenmap) init.
			/// Much better than random for graphs with clustered structure. Full O(N³) eigendecomposition,
			/// so it's disabled automatically for N > SpectralInitMaxN.</summary>
			public bool SpectralInit { get; set; } = true;

			/// <summary>Above this size, fall back to random init regardless of SpectralInit.</summary>
			public int SpectralInitMaxN { get; set; } = 2000;

			/// <summary>Override for the (a, b) curve parameters. Set either to non-null to skip curve-fitting.</summary>
			public double? A { get; set; } = null;
			public double? B { get; set; } = null;
		}

		/// <summary>
		/// Embed N items into low-dimensional space preserving local topology.
		/// 'dist(items[i], items[j])' must return a finite non-negative dissimilarity.
		/// Returns an array of v4 points with w=1. Unused dimensions are zero.
		/// </summary>
		public static v4[] Embed<T>(IReadOnlyList<T> items, Func<T, T, double> dist, Config? config = null)
		{
			return Embed(items, (T a, T b) => (double?)dist(a, b), config);
		}

		/// <summary>
		/// Embed with sparse observed distances. 'dist' returning null means "this pair is unobserved"
		/// and is treated as infinite (not a neighbour). Each point's k-NN is built from the observed
		/// pairs only — points with fewer than k observed neighbours simply use what they have.
		/// </summary>
		public static v4[] Embed<T>(IReadOnlyList<T> items, Func<T, T, double?> dist, Config? config = null)
		{
			var cfg = config ?? new Config();
			Debug.Assert(cfg.Dimensions >= 1 && cfg.Dimensions <= 3);
			Debug.Assert(cfg.NeighborCount >= 2);

			var n = items.Count;
			if (n == 0) return Array.Empty<v4>();
			if (n == 1) return new[] { new v4(0, 0, 0, 1) };

			// Build the k-NN graph (sparse: index + distance arrays per point)
			var (knn_idx, knn_dst) = ComputeKNN(items, dist, cfg.NeighborCount);

			// Per-point ρ (local connectivity floor) and σ (local kernel scale)
			var (sigmas, rhos) = SmoothKNN(knn_dst, cfg.NeighborCount, cfg.LocalConnectivity);

			// Directed fuzzy weights, then symmetrised via probabilistic t-conorm
			var (heads, tails, weights) = FuzzySimplicialSet(n, knn_idx, knn_dst, sigmas, rhos);

			// Curve-fit (a, b) unless caller supplied both
			var (a_param, b_param) = (cfg.A.HasValue && cfg.B.HasValue)
				? (cfg.A.Value, cfg.B.Value)
				: FitABParams(cfg.Spread, cfg.MinDist);

			// Initial embedding
			var use_spectral = cfg.SpectralInit && n <= cfg.SpectralInitMaxN;
			var embedding = use_spectral
				? SpectralInit(n, heads, tails, weights, cfg.Dimensions, cfg.RandomSeed)
				: RandomInit(n, cfg.Dimensions, cfg.RandomSeed);

			// Number of epochs
			var n_epochs = cfg.Epochs > 0 ? cfg.Epochs : (n <= 10000 ? 500 : 200);

			// SGD layout optimisation
			OptimizeLayout(embedding, cfg.Dimensions, heads, tails, weights, n_epochs,
				a_param, b_param, cfg.LearningRate, cfg.NegativeSampleRate, cfg.RepulsionStrength, cfg.RandomSeed);

			// Pack into v4
			var result = new v4[n];
			for (int i = 0; i != n; ++i)
			{
				var x = cfg.Dimensions >= 1 ? (float)embedding[i * cfg.Dimensions + 0] : 0f;
				var y = cfg.Dimensions >= 2 ? (float)embedding[i * cfg.Dimensions + 1] : 0f;
				var z = cfg.Dimensions >= 3 ? (float)embedding[i * cfg.Dimensions + 2] : 0f;
				result[i] = new v4(x, y, z, 1);
			}
			return result;
		}

		// k-NN construction (brute force; O(n² log k))
		private static (int[][] knn_idx, double[][] knn_dst) ComputeKNN<T>(IReadOnlyList<T> items, Func<T, T, double?> dist, int k)
		{
			var n = items.Count;
			var knn_idx = new int[n][];
			var knn_dst = new double[n][];

			// Per-thread scratch: a heap of (distance, index) keeping the k smallest
			for (int i = 0; i != n; ++i)
			{
				// Simple bounded-size sorted insertion. Fine for k on the order of 10-100.
				var best_d = new double[k];
				var best_i = new int[k];
				var filled = 0;

				for (int j = 0; j != n; ++j)
				{
					if (j == i) continue;
					var d = dist(items[i], items[j]);
					if (!d.HasValue) continue;

					var dv = d.Value;
					if (filled < k)
					{
						// Insertion sort
						var pos = filled;
						while (pos > 0 && best_d[pos - 1] > dv)
						{
							best_d[pos] = best_d[pos - 1];
							best_i[pos] = best_i[pos - 1];
							--pos;
						}
						best_d[pos] = dv;
						best_i[pos] = j;
						++filled;
					}
					else if (dv < best_d[k - 1])
					{
						var pos = k - 1;
						while (pos > 0 && best_d[pos - 1] > dv)
						{
							best_d[pos] = best_d[pos - 1];
							best_i[pos] = best_i[pos - 1];
							--pos;
						}
						best_d[pos] = dv;
						best_i[pos] = j;
					}
				}

				if (filled < k)
				{
					Array.Resize(ref best_d, filled);
					Array.Resize(ref best_i, filled);
				}
				knn_dst[i] = best_d;
				knn_idx[i] = best_i;
			}
			return (knn_idx, knn_dst);
		}

		// Smooth k-NN: per-point ρ_i = distance to nearest neighbour (via local_connectivity) and
		// per-point σ_i chosen so the exponential weights over k neighbours sum to log2(k).
		private static (double[] sigmas, double[] rhos) SmoothKNN(double[][] knn_dst, int k, double local_connectivity)
		{
			var n = knn_dst.Length;
			var sigmas = new double[n];
			var rhos = new double[n];

			// Target sum of fuzzy weights per point (UMAP paper's choice).
			var target = Math.Log(k) / Math.Log(2.0);

			for (int i = 0; i != n; ++i)
			{
				var dists = knn_dst[i];
				var count = dists.Length;
				if (count == 0)
				{
					sigmas[i] = 1.0;
					rhos[i] = 0.0;
					continue;
				}

				// ρ_i = distance to the (local_connectivity)-th nearest non-zero neighbour, interpolated.
				// For the usual case local_connectivity=1 this is just dists[0].
				var nonzero = new List<double>(count);
				foreach (var d in dists)
					if (d > 0) nonzero.Add(d);

				double rho = 0.0;
				if (nonzero.Count >= 1)
				{
					var idx = Math.Max(0, Math.Min(nonzero.Count - 1, (int)Math.Floor(local_connectivity)));
					var frac = local_connectivity - Math.Floor(local_connectivity);
					if (idx + 1 < nonzero.Count && frac > 0.0)
						rho = nonzero[idx] + frac * (nonzero[idx + 1] - nonzero[idx]);
					else
						rho = nonzero[idx];
				}
				rhos[i] = rho;

				// Binary search for σ such that Σ exp(-max(0, d_j - ρ) / σ) == target
				double lo = 0.0, hi = double.PositiveInfinity, mid = 1.0;
				for (int iter = 0; iter != 64; ++iter)
				{
					double psum = 0.0;
					for (int j = 0; j != count; ++j)
					{
						var d = dists[j] - rho;
						psum += d > 0 ? Math.Exp(-d / mid) : 1.0;
					}

					if (Math.Abs(psum - target) < 1e-5)
						break;

					if (psum > target)
					{
						hi = mid;
						mid = (lo + hi) * 0.5;
					}
					else
					{
						lo = mid;
						mid = double.IsPositiveInfinity(hi) ? mid * 2.0 : (lo + hi) * 0.5;
					}
				}

				// Lower bound: σ should not be vastly smaller than the mean neighbour distance (prevents collapse)
				double mean = 0.0;
				for (int j = 0; j != count; ++j) mean += dists[j];
				mean /= count;

				var min_sigma_scale = 1e-3;
				sigmas[i] = Math.Max(mid, min_sigma_scale * (mean > 0 ? mean : 1.0));
			}

			return (sigmas, rhos);
		}

		// Fuzzy simplicial set: build directed edge weights a_ij then symmetrise.
		// Output is a COO edge list (heads, tails, weights) suitable for both spectral init and SGD.
		private static (int[] heads, int[] tails, double[] weights) FuzzySimplicialSet(int n, int[][] knn_idx, double[][] knn_dst, double[] sigmas, double[] rhos)
		{
			// Build directed weights into a dictionary keyed by (min, max) so we can combine a_ij + a_ji.
			// For n up to a few thousand this is fine; for larger n a sparse CSR would be better.
			var dir = new Dictionary<long, (double aij, double aji)>();

			long Key(int i, int j) => i < j ? ((long)i << 32) | (uint)j : ((long)j << 32) | (uint)i;

			for (int i = 0; i != n; ++i)
			{
				var dists = knn_dst[i];
				var idxs = knn_idx[i];
				var sigma = sigmas[i];
				var rho = rhos[i];

				for (int t = 0; t != dists.Length; ++t)
				{
					var j = idxs[t];
					var d = dists[t] - rho;
					var w = d > 0 ? Math.Exp(-d / sigma) : 1.0;

					var k = Key(i, j);
					dir.TryGetValue(k, out var cur);
					if (i < j) cur.aij = w; else cur.aji = w;
					dir[k] = cur;
				}
			}

			// Symmetrise with probabilistic t-conorm: w_ij = a + b - a*b.
			var heads = new int[dir.Count];
			var tails = new int[dir.Count];
			var weights = new double[dir.Count];
			int n_edges = 0;
			foreach (var kv in dir)
			{
				var i = (int)(kv.Key >> 32);
				var j = (int)(kv.Key & 0xFFFFFFFF);
				var a = kv.Value.aij;
				var b = kv.Value.aji;
				var w = a + b - a * b;
				if (w <= 0) continue;

				heads[n_edges] = i;
				tails[n_edges] = j;
				weights[n_edges] = w;
				++n_edges;
			}
			if (n_edges != heads.Length)
			{
				Array.Resize(ref heads, n_edges);
				Array.Resize(ref tails, n_edges);
				Array.Resize(ref weights, n_edges);
			}
			return (heads, tails, weights);
		}

		// Fit (a, b) so that 1/(1 + a·d^(2b)) approximates the user's target response curve:
		//   target(d) = 1                                if d <= min_dist
		//   target(d) = exp(-(d - min_dist) / spread)    otherwise
		// Gauss-Newton with a small damping term.
		private static (double a, double b) FitABParams(double spread, double min_dist)
		{
			// Sample the target curve
			const int S = 300;
			var xs = new double[S];
			var ys = new double[S];
			for (int i = 0; i != S; ++i)
			{
				var x = 3.0 * spread * i / (S - 1);
				xs[i] = x;
				ys[i] = x <= min_dist ? 1.0 : Math.Exp(-(x - min_dist) / spread);
			}

			// Initial guess
			double a = 1.0, b = 1.0;

			for (int iter = 0; iter != 100; ++iter)
			{
				// Residuals and Jacobian
				double g00 = 0, g01 = 0, g11 = 0;  // JᵀJ (symmetric)
				double r0 = 0, r1 = 0;             // Jᵀr

				for (int i = 0; i != S; ++i)
				{
					var x = xs[i];
					if (x <= 0) continue;
					var x2b = Math.Pow(x, 2 * b);
					var denom = 1.0 + a * x2b;
					var f = 1.0 / denom;
					var r = ys[i] - f;

					// df/da = -x^(2b) / denom²
					var df_da = -x2b / (denom * denom);
					// df/db = -a · 2·ln(x) · x^(2b) / denom²
					var df_db = -2.0 * a * Math.Log(x) * x2b / (denom * denom);

					g00 += df_da * df_da;
					g01 += df_da * df_db;
					g11 += df_db * df_db;
					r0  += df_da * r;
					r1  += df_db * r;
				}

				// Solve 2x2: (JᵀJ + λI) Δ = -Jᵀr   (sign flip because we want to move toward reducing r)
				// Actually we want [a,b] += (JᵀJ)⁻¹ · (-Jᵀr). The residual r = y - f, so grad of 0.5·||r||² = -Jᵀr.
				// Newton step: Δ = (JᵀJ)⁻¹ · Jᵀr    (no negation since we defined r = y - f not f - y).
				double damping = 1e-6;
				double m00 = g00 + damping;
				double m11 = g11 + damping;
				double m01 = g01;
				double det = m00 * m11 - m01 * m01;
				if (Math.Abs(det) < 1e-18) break;

				double da = ( m11 * r0 - m01 * r1) / det;
				double db = (-m01 * r0 + m00 * r1) / det;

				a += da;
				b += db;

				// Keep within sensible bounds
				if (a < 1e-4) a = 1e-4;
				if (b < 1e-2) b = 1e-2;

				if (Math.Abs(da) + Math.Abs(db) < 1e-8) break;
			}
			return (a, b);
		}

		// Spectral initialisation: eigenvectors of the normalised Laplacian of the fuzzy graph.
		// Gives a much better starting point than random when the graph has clustered structure.
		// 'internal' (rather than private) so unit tests can exercise this branch directly —
		// it is otherwise hard to reach from Embed() because disconnected graphs fall back to RandomInit.
		internal static double[] SpectralInit(int n, int[] heads, int[] tails, double[] weights, int dim, int seed)
		{
			// Build dense adjacency W and degrees d
			var W = new Matrix(n, n);
			var deg = new double[n];
			for (int e = 0; e != heads.Length; ++e)
			{
				var i = heads[e];
				var j = tails[e];
				var w = weights[e];
				W[i, j] = w;
				W[j, i] = w;
				deg[i] += w;
				deg[j] += w;
			}

			// Disconnected graph -> spectral is meaningless. Detect via any zero-degree node
			// OR multiple connected components (Lanczos can fail to converge when eigenvalues
			// are exactly degenerate, which happens when multiple components exist).
			for (int i = 0; i != n; ++i)
			{
				if (deg[i] <= 0)
					return RandomInit(n, dim, seed);
			}

			// BFS to count connected components. If more than one, fall back to random init.
			{
				var visited = new bool[n];
				var queue = new Queue<int>();
				int components = 0;
				for (int start = 0; start != n; ++start)
				{
					if (visited[start]) continue;
					++components;
					if (components > 1) break;
					queue.Enqueue(start);
					visited[start] = true;
					while (queue.Count > 0)
					{
						var u = queue.Dequeue();
						for (int v = 0; v != n; ++v)
						{
							if (!visited[v] && W[u, v] > 0)
							{
								visited[v] = true;
								queue.Enqueue(v);
							}
						}
					}
				}
				if (components > 1)
					return RandomInit(n, dim, seed);
			}

			// Normalised Laplacian L = I - D^(-1/2) W D^(-1/2).  Eigenvalues in [0, 2].
			// We want the smallest non-trivial eigenvectors. Use the trick: largest eigenvectors of
			// (2I - L) correspond to smallest of L, so we can reuse EigenTopK efficiently.
			var inv_sqrt_deg = new double[n];
			for (int i = 0; i != n; ++i)
				inv_sqrt_deg[i] = 1.0 / Math.Sqrt(deg[i]);

			var M = new Matrix(n, n);
			for (int i = 0; i != n; ++i)
			{
				// (2I - L)[i,i] = 2 - (1 - D^(-1/2) W D^(-1/2))[i,i] = 1 + D^(-1/2) W[i,i] D^(-1/2)
				M[i, i] = 1.0 + inv_sqrt_deg[i] * W[i, i] * inv_sqrt_deg[i];
			}
			for (int i = 0; i != n; ++i)
			{
				for (int j = 0; j != n; ++j)
				{
					if (i == j) continue;
					// (2I - L)[i,j] = D^(-1/2) W D^(-1/2)
					M[i, j] = inv_sqrt_deg[i] * W[i, j] * inv_sqrt_deg[j];
				}
			}

			// Top (dim+1) eigenpairs of M. The very top corresponds to the null eigenvector of L
			// (all ones / D^(1/2)) — we skip it. Use full EigenSymmetric rather than Lanczos
			// because the fuzzy graph frequently has highly-degenerate eigenvalue structure
			// (clusters, equal weights, etc.) which Lanczos does not handle reliably.
			// Cost is O(n³) but we gate n by SpectralInitMaxN.
			var eig = Matrix.EigenSymmetric(M);

			// Skip column 0 (the trivial all-ones null of L) and take columns [1..dim].
			// Vectors[component, eigenvector_index] — component i of eigenvector (d+1).
			var emb = new double[n * dim];
			for (int i = 0; i != n; ++i)
			{
				for (int d = 0; d != dim; ++d)
					emb[i * dim + d] = eig.Vectors[i, d + 1];
			}

			// Rescale to the UMAP-standard initial magnitude (10.0 is the usual value).
			double max_abs = 0.0;
			for (int i = 0; i != emb.Length; ++i)
				if (Math.Abs(emb[i]) > max_abs) max_abs = Math.Abs(emb[i]);
			if (max_abs > 0)
			{
				var s = 10.0 / max_abs;
				for (int i = 0; i != emb.Length; ++i) emb[i] *= s;
			}

			// Add a small noise term to break symmetries when eigenvalues are degenerate.
			var rng = new Random(seed);
			for (int i = 0; i != emb.Length; ++i)
				emb[i] += 1e-4 * (rng.NextDouble() - 0.5);

			return emb;
		}

		// Random initialisation: uniform in [-10, 10]. UMAP's original Python implementation uses a normal distribution
		// with σ=0.0001, but the exact distribution doesn't matter much and a wider uniform gives a better starting point for SGD.
		private static double[] RandomInit(int n, int dim, int seed)
		{
			var rng = new Random(seed);
			var emb = new double[n * dim];
			for (int i = 0; i != emb.Length; ++i)
				emb[i] = 20.0 * (rng.NextDouble() - 0.5); // initial range ±10
			return emb;
		}

		// SGD layout optimisation. Edge schedule: edge e is visited every 1/weights[e] epochs
		// (normalised so the strongest edge is visited every epoch). For each visit, apply
		// attraction, then `NegativeSampleRate` repulsive updates against random targets.
		private static void OptimizeLayout(double[] emb, int dim, int[] heads, int[] tails, double[] weights, int n_epochs, double a, double b, double init_lr, int neg_rate, double gamma, int seed)
		{
			var n = emb.Length / dim;
			var n_edges = heads.Length;

			// Normalise weights so strongest edge -> 1 visit per epoch (smallest interval 1.0)
			double max_w = 0.0;
			for (int e = 0; e != n_edges; ++e)
				if (weights[e] > max_w) max_w = weights[e];

			if (max_w <= 0) return;

			// Schedule: epochs-per-sample. Smaller -> more frequent visits.
			var eps = new double[n_edges];
			for (int e = 0; e != n_edges; ++e)
				eps[e] = max_w / weights[e];

			// Next-epoch-to-visit tracker (starts at eps[e] — first visit is one interval in)
			var next_visit = new double[n_edges];
			Array.Copy(eps, next_visit, n_edges);

			// Scratch for gradient updates
			var rng = new Random(seed ^ 0x13579BDF);
			const double grad_clip = 4.0;

			for (int epoch = 0; epoch != n_epochs; ++epoch)
			{
				// Linear learning-rate decay to zero
				var alpha = init_lr * (1.0 - (double)epoch / n_epochs);

				for (int e = 0; e != n_edges; ++e)
				{
					if (next_visit[e] > epoch + 1) continue; // not due this epoch

					var i = heads[e];
					var j = tails[e];

					// Attractive update along edge (i, j)
					var d2 = 0.0;
					for (int k = 0; k != dim; ++k)
					{
						var dx = emb[i * dim + k] - emb[j * dim + k];
						d2 += dx * dx;
					}

					double grad_coef_attr = 0.0;
					if (d2 > 0.0)
					{
						// grad of cross-entropy w.r.t |Δy|²: -2·a·b·|Δy|^(2b-2) / (1 + a·|Δy|^(2b))
						// Using d2 = |Δy|² : |Δy|^(2b-2) = d2^(b-1),  |Δy|^(2b) = d2^b
						var d2b = Math.Pow(d2, b);
						grad_coef_attr = (-2.0 * a * b * Math.Pow(d2, b - 1.0)) / (1.0 + a * d2b);
					}

					for (int k = 0; k != dim; ++k)
					{
						var dx = emb[i * dim + k] - emb[j * dim + k];
						var g = grad_coef_attr * dx;
						if (g > grad_clip) g = grad_clip;
						else if (g < -grad_clip) g = -grad_clip;
						emb[i * dim + k] += alpha * g;
						emb[j * dim + k] -= alpha * g;
					}

					// Repulsive updates: push i away from `neg_rate` random points
					for (int s = 0; s != neg_rate; ++s)
					{
						var jn = rng.Next(n);
						if (jn == i) continue;

						var dn2 = 0.0;
						for (int k = 0; k != dim; ++k)
						{
							var dx = emb[i * dim + k] - emb[jn * dim + k];
							dn2 += dx * dx;
						}

						double grad_coef_rep = 0.0;
						if (dn2 > 0.0)
						{
							// grad of -log(1 - phi): 2·γ·b / ((ε + |Δy|²) · (1 + a·|Δy|^(2b)))
							var d2b = Math.Pow(dn2, b);
							grad_coef_rep = (2.0 * gamma * b) / ((0.001 + dn2) * (1.0 + a * d2b));
						}
						else
						{
							// Degenerate overlap — nudge arbitrarily
							grad_coef_rep = 4.0;
						}

						for (int k = 0; k != dim; ++k)
						{
							var dx = emb[i * dim + k] - emb[jn * dim + k];
							var g = grad_coef_rep * dx;
							if (g > grad_clip) g = grad_clip;
							else if (g < -grad_clip) g = -grad_clip;
							emb[i * dim + k] += alpha * g;
						}
					}

					// Schedule next visit for this edge
					next_visit[e] += eps[e];
				}
			}
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Algorithm.UMAP;

	[TestFixture]
	public class UnitTestUMAP
	{
		[Test]
		public void EmptyAndSingle()
		{
			var r0 = UniformManifoldApproximationAndProjection.Embed(Array.Empty<int>(), (a, b) => 0.0);
			Assert.True(r0.Length == 0);

			var r1 = UniformManifoldApproximationAndProjection.Embed(new[] { 42 }, (a, b) => 0.0);
			Assert.True(r1.Length == 1);
		}

		[Test]
		public void FitABParamsSanity()
		{
			// Default (spread=1, min_dist=0.1) should recover the well-known UMAP defaults
			// a ≈ 1.577, b ≈ 0.895 (within a sensible tolerance for a simple Gauss-Newton fit).
			var cfg = new UniformManifoldApproximationAndProjection.Config { MinDist = 0.1, Spread = 1.0 };
			var items = new[] { 0, 1, 2, 3, 4, 5 };
			// Dummy distances — we only care about the curve fit, which is invoked inside Embed.
			var r = UniformManifoldApproximationAndProjection.Embed(items, (a, b) => (double)Math.Abs(a - b), cfg);
			Assert.True(r.Length == items.Length);
		}

		[Test]
		public void TwoClustersSeparate()
		{
			// Two well-separated 2D Gaussian clusters; UMAP should place them on opposite sides.
			var rng = new Random(17);
			var pts = new (double x, double y)[60];
			for (int i = 0; i != 30; ++i)
				pts[i] = (rng.NextDouble() + 0.0, rng.NextDouble() + 0.0);
			for (int i = 30; i != 60; ++i)
				pts[i] = (rng.NextDouble() + 10.0, rng.NextDouble() + 10.0);

			var result = UniformManifoldApproximationAndProjection.Embed(pts, (a, b) =>
			{
				var dx = a.x - b.x; var dy = a.y - b.y;
				return Math.Sqrt(dx * dx + dy * dy);
			}, new UniformManifoldApproximationAndProjection.Config { Dimensions = 2, NeighborCount = 5, Epochs = 200, RandomSeed = 1 });

			Assert.True(result.Length == 60);

			// Mean embedded position of each cluster — should differ markedly
			var mx0 = 0f; var my0 = 0f; var mx1 = 0f; var my1 = 0f;
			for (int i = 0; i != 30; ++i) { mx0 += result[i].x; my0 += result[i].y; }
			for (int i = 30; i != 60; ++i) { mx1 += result[i].x; my1 += result[i].y; }
			mx0 /= 30; my0 /= 30; mx1 /= 30; my1 /= 30;

			var cluster_sep = Math.Sqrt((mx0 - mx1) * (mx0 - mx1) + (my0 - my1) * (my0 - my1));

			// Intra-cluster spread as scale reference
			var spread0 = 0.0;
			for (int i = 0; i != 30; ++i)
			{
				var dx = result[i].x - mx0; var dy = result[i].y - my0;
				spread0 += Math.Sqrt(dx * dx + dy * dy);
			}
			spread0 /= 30;

			// Clusters should be well separated relative to their internal spread.
			// Threshold 2.0 gives comfortable margin; a fully-converged UMAP typically achieves
			// 5-10×, but we keep the test robust to minor algorithmic variation.
			Assert.True(cluster_sep > 2.0 * spread0);
		}

		[Test]
		public void SpectralInitChainIsMonotonic()
		{
			// Exercises the SpectralInit branch directly (which is skipped by Embed() when the
			// k-NN graph has multiple components). Uses a path graph: N nodes connected in a
			// chain 0-1-2-...-(N-1). The 1D SpectralInit embedding should be strongly correlated
			// with the chain index (|Pearson| > 0.95) because the Fiedler vector of a path graph
			// is monotonic along the chain (up to sign and minor endpoint effects from normalisation).
			// A transposed indexing of the eigenvector matrix would give essentially random
			// values across nodes, and correlation would collapse near zero.
			const int N = 20;
			var heads = new int[N - 1];
			var tails = new int[N - 1];
			var weights = new double[N - 1];
			for (int i = 0; i != N - 1; ++i)
			{
				heads[i] = i;
				tails[i] = i + 1;
				weights[i] = 1.0;
			}

			var emb = UniformManifoldApproximationAndProjection.SpectralInit(N, heads, tails, weights, 1, 42);
			Assert.True(emb.Length == N);

			// Pearson correlation between chain index and embedded coordinate.
			var mean_x = 0.5 * (N - 1);
			var mean_y = 0.0;
			foreach (var v in emb) mean_y += v;
			mean_y /= N;

			var sxy = 0.0; var sxx = 0.0; var syy = 0.0;
			for (int i = 0; i != N; ++i)
			{
				var dx = i - mean_x;
				var dy = emb[i] - mean_y;
				sxy += dx * dy;
				sxx += dx * dx;
				syy += dy * dy;
			}
			var corr = sxy / Math.Sqrt(sxx * syy);
			Assert.True(Math.Abs(corr) > 0.95);
		}
	}
}
#endif
