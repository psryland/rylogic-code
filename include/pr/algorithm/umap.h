//*********************************************
// Uniform Manifold Approximation and Projection (UMAP)
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
// Reference: L. McInnes, J. Healy, J. Melville (2018) "UMAP: Uniform Manifold Approximation
// and Projection for Dimension Reduction", arXiv:1802.03426
//
// Unlike classical MDS (which tries to preserve pairwise distances globally), UMAP preserves
// the *topology* of the k-nearest-neighbour graph. It is usually much better at producing
// readable 2D/3D visualisations of high-intrinsic-dimension data.
//
// Pipeline:
//   1. Build a k-nearest-neighbour graph in the original space.
//   2. For each point, fit a per-point sigma so the exponential kernel over neighbours has total
//      weight log2(k). Points with dense neighbourhoods get small sigma, sparse ones get large sigma.
//   3. Build a fuzzy simplicial set: edge weights a_ij = exp(-max(0, d_ij - rho_i) / sigma_i),
//      then symmetrise via the probabilistic t-conorm  w_ij = a_ij + a_ji - a_ij*a_ji.
//   4. Fit curve parameters (a, b) so 1/(1 + a*d^(2b)) approximates the user-requested
//      (spread, min_dist) response in the embedding space.
//   5. Initialise low-dim positions via spectral embedding of the fuzzy graph's Laplacian
//      (falls back to random init for disconnected graphs or very large N).
//   6. SGD: for each epoch, apply attractive force along each edge (proportional to its
//      fuzzy weight) and repulsive force against random negative samples.
#pragma once
#include <concepts>
#include <type_traits>
#include <span>
#include <vector>
#include <ranges>
#include <optional>
#include <unordered_map>
#include <queue>
#include <random>
#include <limits>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>
#include "pr/math/math.h"

namespace pr::algorithm::umap
{
	// Configuration for UMAP embedding
	struct Config
	{
		// Output dimensionality (1, 2, or 3). v4 components beyond this are zero; w = 1.
		int dimensions = 2;

		// Number of nearest neighbours used to build the local topology. 15 is a good default;
		// small values (2-5) emphasise fine local detail, large values (50+) emphasise global structure.
		int neighbor_count = 15;

		// Minimum desired distance between embedded points - smaller = tighter clusters.
		double min_dist = 0.1;

		// Scale of the embedded distribution. Combined with min_dist this determines (a, b).
		double spread = 1.0;

		// SGD epochs. 0 = auto (500 for n<=10000, else 200).
		int epochs = 0;

		// Number of negative samples per positive (attractive) edge visit.
		int negative_sample_rate = 5;

		// Initial SGD learning rate. Decays linearly to 0 over the run.
		double learning_rate = 1.0;

		// Repulsion strength. 1.0 is standard; larger values force clusters further apart.
		double repulsion_strength = 1.0;

		// PRNG seed for reproducibility of SGD sampling and (if needed) random init.
		int random_seed = 42;

		// How many nearest neighbours have "connected" fuzzy weight 1. Usually 1.
		double local_connectivity = 1.0;

		// If true and the fuzzy graph is connected, use spectral (Laplacian-eigenmap) init.
		// Much better than random for graphs with clustered structure. Full O(N^3) eigendecomposition,
		// so it's disabled automatically for N > spectral_init_max_n.
		bool spectral_init = true;

		// Above this size, fall back to random init regardless of spectral_init.
		int spectral_init_max_n = 2000;

		// Override for the (a, b) curve parameters. Set either to non-nullopt to skip curve-fitting.
		std::optional<double> a = std::nullopt;
		std::optional<double> b = std::nullopt;
	};

	// Concept for a distance function between items returning a plain dissimilarity
	template <typename DistFunc, typename Item>
	concept DistanceFunction =
		std::invocable<DistFunc, Item const&, Item const&> &&
		std::convertible_to<std::invoke_result_t<DistFunc, Item const&, Item const&>, double>;

	// Concept for a distance function that may return std::nullopt for unknown pairs
	template <typename DistFunc, typename Item>
	concept OptionalDistanceFunction =
		std::invocable<DistFunc, Item const&, Item const&> &&
		std::same_as<std::invoke_result_t<DistFunc, Item const&, Item const&>, std::optional<double>>;

	namespace detail
	{
		struct KNN
		{
			// knn_idx[i] = indices of i's nearest neighbours (size <= k)
			// knn_dst[i] = corresponding distances, ascending
			std::vector<std::vector<int>> idx;
			std::vector<std::vector<double>> dst;
		};

		// k-NN construction (brute force; O(n^2 log k)). 'dist' returns std::nullopt to mean
		// "this pair is unobserved" - such pairs are skipped (not treated as far, just missing).
		template <std::ranges::random_access_range Range, typename DistFunc>
		KNN ComputeKNN(Range&& items, DistFunc dist, int k)
		{
			auto const n = static_cast<int>(std::ranges::size(items));
			auto result = KNN{};
			result.idx.resize(n);
			result.dst.resize(n);

			for (int i = 0; i != n; ++i)
			{
				auto best_d = std::vector<double>(k);
				auto best_i = std::vector<int>(k);
				auto filled = 0;

				for (int j = 0; j != n; ++j)
				{
					if (j == i) continue;

					// Coerce to std::optional<double> uniformly
					std::optional<double> d_opt;
					if constexpr (std::same_as<std::invoke_result_t<DistFunc, decltype(items[i]), decltype(items[j])>, std::optional<double>>)
						d_opt = dist(items[i], items[j]);
					else
						d_opt = static_cast<double>(dist(items[i], items[j]));

					if (!d_opt.has_value()) continue;
					auto dv = *d_opt;

					if (filled < k)
					{
						auto pos = filled;
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
						auto pos = k - 1;
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

				best_d.resize(filled);
				best_i.resize(filled);
				result.dst[i] = std::move(best_d);
				result.idx[i] = std::move(best_i);
			}
			return result;
		}

		struct SigmaRho
		{
			std::vector<double> sigmas;
			std::vector<double> rhos;
		};

		// Smooth k-NN: per-point rho_i = distance to nearest neighbour (via local_connectivity) and
		// per-point sigma_i chosen so the exponential weights over k neighbours sum to log2(k).
		inline SigmaRho SmoothKNN(KNN const& knn, int k, double local_connectivity)
		{
			auto const n = static_cast<int>(knn.dst.size());
			auto result = SigmaRho{};
			result.sigmas.resize(n, 1.0);
			result.rhos.resize(n, 0.0);

			auto const target = std::log(static_cast<double>(k)) / std::log(2.0);
			constexpr auto INF = std::numeric_limits<double>::infinity();

			for (int i = 0; i != n; ++i)
			{
				auto const& dists = knn.dst[i];
				auto const count = static_cast<int>(dists.size());
				if (count == 0)
				{
					result.sigmas[i] = 1.0;
					result.rhos[i] = 0.0;
					continue;
				}

				auto nonzero = std::vector<double>{};
				nonzero.reserve(count);
				for (auto d : dists)
					if (d > 0) nonzero.push_back(d);

				auto rho = 0.0;
				if (!nonzero.empty())
				{
					auto const lc_floor = std::floor(local_connectivity);
					auto idx = (std::max)(0, (std::min)(static_cast<int>(nonzero.size()) - 1, static_cast<int>(lc_floor)));
					auto frac = local_connectivity - lc_floor;
					if (idx + 1 < static_cast<int>(nonzero.size()) && frac > 0.0)
						rho = nonzero[idx] + frac * (nonzero[idx + 1] - nonzero[idx]);
					else
						rho = nonzero[idx];
				}
				result.rhos[i] = rho;

				// Binary search for sigma such that Sum exp(-max(0, d_j - rho) / sigma) == target
				auto lo = 0.0, hi = INF, mid = 1.0;
				for (int iter = 0; iter != 64; ++iter)
				{
					auto psum = 0.0;
					for (int j = 0; j != count; ++j)
					{
						auto d = dists[j] - rho;
						psum += d > 0 ? std::exp(-d / mid) : 1.0;
					}

					if (std::abs(psum - target) < 1e-5)
						break;

					if (psum > target)
					{
						hi = mid;
						mid = (lo + hi) * 0.5;
					}
					else
					{
						lo = mid;
						mid = std::isinf(hi) ? mid * 2.0 : (lo + hi) * 0.5;
					}
				}

				// Lower bound: sigma shouldn't be vastly smaller than the mean neighbour distance (prevents collapse)
				auto mean = 0.0;
				for (int j = 0; j != count; ++j) mean += dists[j];
				mean /= count;

				constexpr auto min_sigma_scale = 1e-3;
				result.sigmas[i] = (std::max)(mid, min_sigma_scale * (mean > 0 ? mean : 1.0));
			}

			return result;
		}

		struct FuzzyGraph
		{
			std::vector<int> heads;
			std::vector<int> tails;
			std::vector<double> weights;
		};

		// Fuzzy simplicial set: build directed edge weights a_ij then symmetrise.
		inline FuzzyGraph FuzzySimplicialSet(int n, KNN const& knn, SigmaRho const& sr)
		{
			// Keyed by (min,max) edge endpoints so a_ij and a_ji can be combined.
			struct EdgeWeights { double aij = 0.0; double aji = 0.0; };
			auto dir = std::unordered_map<std::int64_t, EdgeWeights>{};

			auto key = [](int i, int j) -> std::int64_t
			{
				auto lo = i < j ? i : j;
				auto hi = i < j ? j : i;
				return (static_cast<std::int64_t>(lo) << 32) | static_cast<std::uint32_t>(hi);
			};

			for (int i = 0; i != n; ++i)
			{
				auto const& dists = knn.dst[i];
				auto const& idxs = knn.idx[i];
				auto const sigma = sr.sigmas[i];
				auto const rho = sr.rhos[i];

				for (int t = 0; t != static_cast<int>(dists.size()); ++t)
				{
					auto j = idxs[t];
					auto d = dists[t] - rho;
					auto w = d > 0 ? std::exp(-d / sigma) : 1.0;

					auto& cur = dir[key(i, j)];
					if (i < j) cur.aij = w; else cur.aji = w;
				}
			}

			auto out = FuzzyGraph{};
			out.heads.reserve(dir.size());
			out.tails.reserve(dir.size());
			out.weights.reserve(dir.size());

			for (auto const& kv : dir)
			{
				auto i = static_cast<int>(kv.first >> 32);
				auto j = static_cast<int>(kv.first & 0xFFFFFFFFu);
				auto a = kv.second.aij;
				auto b = kv.second.aji;
				auto w = a + b - a * b;
				if (w <= 0) continue;

				out.heads.push_back(i);
				out.tails.push_back(j);
				out.weights.push_back(w);
			}
			return out;
		}

		// Fit (a, b) so 1/(1 + a*d^(2b)) approximates target curve derived from (spread, min_dist).
		inline std::pair<double, double> FitABParams(double spread, double min_dist)
		{
			constexpr int S = 300;
			double xs[S];
			double ys[S];
			for (int i = 0; i != S; ++i)
			{
				auto x = 3.0 * spread * i / (S - 1);
				xs[i] = x;
				ys[i] = x <= min_dist ? 1.0 : std::exp(-(x - min_dist) / spread);
			}

			auto a = 1.0;
			auto b = 1.0;

			for (int iter = 0; iter != 100; ++iter)
			{
				auto g00 = 0.0, g01 = 0.0, g11 = 0.0;
				auto r0 = 0.0, r1 = 0.0;

				for (int i = 0; i != S; ++i)
				{
					auto x = xs[i];
					if (x <= 0) continue;
					auto x2b = std::pow(x, 2 * b);
					auto denom = 1.0 + a * x2b;
					auto f = 1.0 / denom;
					auto r = ys[i] - f;

					auto df_da = -x2b / (denom * denom);
					auto df_db = -2.0 * a * std::log(x) * x2b / (denom * denom);

					g00 += df_da * df_da;
					g01 += df_da * df_db;
					g11 += df_db * df_db;
					r0 += df_da * r;
					r1 += df_db * r;
				}

				constexpr auto damping = 1e-6;
				auto m00 = g00 + damping;
				auto m11 = g11 + damping;
				auto m01 = g01;
				auto det = m00 * m11 - m01 * m01;
				if (std::abs(det) < 1e-18) break;

				auto da = (m11 * r0 - m01 * r1) / det;
				auto db = (-m01 * r0 + m00 * r1) / det;

				a += da;
				b += db;

				if (a < 1e-4) a = 1e-4;
				if (b < 1e-2) b = 1e-2;

				if (std::abs(da) + std::abs(db) < 1e-8) break;
			}
			return { a, b };
		}

		// Random initialisation: uniform in [-10, 10] per component.
		inline std::vector<double> RandomInit(int n, int dim, int seed)
		{
			auto rng = std::minstd_rand(static_cast<std::uint32_t>(seed));
			auto uni = std::uniform_real_distribution<double>(-10.0, 10.0);
			auto emb = std::vector<double>(static_cast<size_t>(n) * dim);
			for (auto& v : emb) v = uni(rng);
			return emb;
		}

		// Spectral initialisation: eigenvectors of the normalised Laplacian of the fuzzy graph.
		inline std::vector<double> SpectralInit(int n, FuzzyGraph const& g, int dim, int seed)
		{
			// Dense adjacency W and degree vector
			auto W = Matrix<double>(n, n);
			auto deg = std::vector<double>(n, 0.0);
			for (int e = 0; e != static_cast<int>(g.heads.size()); ++e)
			{
				auto i = g.heads[e];
				auto j = g.tails[e];
				auto w = g.weights[e];
				W(i, j) = w;
				W(j, i) = w;
				deg[i] += w;
				deg[j] += w;
			}

			// Any isolated node => fall back to random init
			for (int i = 0; i != n; ++i)
			{
				if (deg[i] <= 0)
					return RandomInit(n, dim, seed);
			}

			// BFS to count connected components. >1 => random fallback (Laplacian has degenerate zeros).
			{
				auto visited = std::vector<bool>(n, false);
				auto queue = std::queue<int>{};
				int components = 0;
				for (int start = 0; start != n; ++start)
				{
					if (visited[start]) continue;
					++components;
					if (components > 1) break;
					queue.push(start);
					visited[start] = true;
					while (!queue.empty())
					{
						auto u = queue.front();
						queue.pop();
						for (int v = 0; v != n; ++v)
						{
							if (!visited[v] && W(u, v) > 0)
							{
								visited[v] = true;
								queue.push(v);
							}
						}
					}
				}
				if (components > 1)
					return RandomInit(n, dim, seed);
			}

			// M = 2I - L_norm = I + D^(-1/2) W D^(-1/2) (after the W[i,i]=0 convention).
			// Largest eigenpairs of M correspond to smallest of the normalised Laplacian.
			auto inv_sqrt_deg = std::vector<double>(n);
			for (int i = 0; i != n; ++i)
				inv_sqrt_deg[i] = 1.0 / std::sqrt(deg[i]);

			auto M = Matrix<double>(n, n);
			for (int i = 0; i != n; ++i)
			{
				M(i, i) = 1.0 + inv_sqrt_deg[i] * W(i, i) * inv_sqrt_deg[i];
				for (int j = 0; j != n; ++j)
				{
					if (i == j) continue;
					M(i, j) = inv_sqrt_deg[i] * W(i, j) * inv_sqrt_deg[j];
				}
			}

			// Full symmetric eigendecomposition (gated by spectral_init_max_n in caller).
			auto eig = EigenSymmetric(M);

			// The Laplacian only provides n-1 non-trivial eigenvectors. If the caller asks for
			// more dimensions, keep the extra components from the seeded random init so the output
			// shape stays fixed and the optimiser still starts deterministically.
			auto emb = RandomInit(n, dim, seed);
			auto spectral_dim = (std::min)(dim, n - 1);

			// Skip column 0 (trivial all-ones null of L); take the first spectral_dim columns [1..].
			for (int i = 0; i != n; ++i)
			{
				for (int d = 0; d != spectral_dim; ++d)
					emb[i * dim + d] = eig.vectors(i, d + 1);
			}

			// Rescale and jitter only the spectral dimensions; the padded random dimensions already
			// have the standard UMAP initial scale.
			auto max_abs = 0.0;
			for (int i = 0; i != n; ++i)
				for (int d = 0; d != spectral_dim; ++d)
					max_abs = (std::max)(max_abs, std::abs(emb[i * dim + d]));
			if (max_abs > 0)
			{
				auto s = 10.0 / max_abs;
				for (int i = 0; i != n; ++i)
					for (int d = 0; d != spectral_dim; ++d)
						emb[i * dim + d] *= s;
			}

			// Break ties / symmetries with a tiny jitter.
			auto rng = std::minstd_rand(static_cast<std::uint32_t>(seed));
			auto uni = std::uniform_real_distribution<double>(-0.5, 0.5);
			for (int i = 0; i != n; ++i)
				for (int d = 0; d != spectral_dim; ++d)
					emb[i * dim + d] += 1e-4 * uni(rng);

			return emb;
		}

		// SGD layout optimisation.
		inline void OptimizeLayout(std::vector<double>& emb, int dim, FuzzyGraph const& g,
			int n_epochs, double a, double b, double init_lr, int neg_rate, double gamma, int seed)
		{
			auto const n = static_cast<int>(emb.size() / dim);
			auto const n_edges = static_cast<int>(g.heads.size());

			auto max_w = 0.0;
			for (auto w : g.weights)
				max_w = (std::max)(max_w, w);
			if (max_w <= 0) return;

			auto eps = std::vector<double>(n_edges);
			for (int e = 0; e != n_edges; ++e)
				eps[e] = max_w / g.weights[e];

			auto next_visit = eps; // copy

			auto rng = std::minstd_rand(static_cast<std::uint32_t>(seed ^ 0x13579BDF));
			auto neg_dist = std::uniform_int_distribution<int>(0, n - 1);

			constexpr auto grad_clip = 4.0;

			for (int epoch = 0; epoch != n_epochs; ++epoch)
			{
				auto alpha = init_lr * (1.0 - static_cast<double>(epoch) / n_epochs);

				for (int e = 0; e != n_edges; ++e)
				{
					if (next_visit[e] > epoch + 1) continue;

					auto i = g.heads[e];
					auto j = g.tails[e];

					// Attractive update along edge (i, j)
					auto d2 = 0.0;
					for (int k = 0; k != dim; ++k)
					{
						auto dx = emb[i * dim + k] - emb[j * dim + k];
						d2 += dx * dx;
					}

					auto grad_coef_attr = 0.0;
					if (d2 > 0.0)
					{
						auto d2b = std::pow(d2, b);
						grad_coef_attr = (-2.0 * a * b * std::pow(d2, b - 1.0)) / (1.0 + a * d2b);
					}

					for (int k = 0; k != dim; ++k)
					{
						auto dx = emb[i * dim + k] - emb[j * dim + k];
						auto gg = grad_coef_attr * dx;
						if (gg > grad_clip) gg = grad_clip;
						else if (gg < -grad_clip) gg = -grad_clip;
						emb[i * dim + k] += alpha * gg;
						emb[j * dim + k] -= alpha * gg;
					}

					// Repulsive updates
					for (int s = 0; s != neg_rate; ++s)
					{
						auto jn = neg_dist(rng);
						if (jn == i) continue;

						auto dn2 = 0.0;
						for (int k = 0; k != dim; ++k)
						{
							auto dx = emb[i * dim + k] - emb[jn * dim + k];
							dn2 += dx * dx;
						}

						auto grad_coef_rep = 0.0;
						if (dn2 > 0.0)
						{
							auto d2b = std::pow(dn2, b);
							grad_coef_rep = (2.0 * gamma * b) / ((0.001 + dn2) * (1.0 + a * d2b));
						}
						else
						{
							grad_coef_rep = 4.0;
						}

						for (int k = 0; k != dim; ++k)
						{
							auto dx = emb[i * dim + k] - emb[jn * dim + k];
							auto gg = grad_coef_rep * dx;
							if (gg > grad_clip) gg = grad_clip;
							else if (gg < -grad_clip) gg = -grad_clip;
							emb[i * dim + k] += alpha * gg;
						}
					}

					next_visit[e] += eps[e];
				}
			}
		}

		// Shared entry point taking a k-NN builder callable (which knows whether the dist is dense or optional).
		template <typename KNNBuilder>
		std::vector<v4> EmbedImpl(int n, KNNBuilder build_knn, Config const& cfg)
		{
			assert(cfg.dimensions >= 1 && cfg.dimensions <= 3);
			assert(cfg.neighbor_count >= 2);

			if (n == 0) return {};
			if (n == 1) return { v4{0, 0, 0, 1} };

			auto knn = build_knn();
			auto sr = detail::SmoothKNN(knn, cfg.neighbor_count, cfg.local_connectivity);
			auto graph = detail::FuzzySimplicialSet(n, knn, sr);

			auto [a_param, b_param] = (cfg.a.has_value() && cfg.b.has_value())
				? std::pair<double, double>{ *cfg.a, *cfg.b }
				: detail::FitABParams(cfg.spread, cfg.min_dist);

			auto use_spectral = cfg.spectral_init && n <= cfg.spectral_init_max_n;
			auto embedding = use_spectral
				? detail::SpectralInit(n, graph, cfg.dimensions, cfg.random_seed)
				: detail::RandomInit(n, cfg.dimensions, cfg.random_seed);

			auto n_epochs = cfg.epochs > 0 ? cfg.epochs : (n <= 10000 ? 500 : 200);

			detail::OptimizeLayout(embedding, cfg.dimensions, graph, n_epochs, a_param, b_param,
				cfg.learning_rate, cfg.negative_sample_rate, cfg.repulsion_strength, cfg.random_seed);

			auto result = std::vector<v4>(n);
			for (int i = 0; i != n; ++i)
			{
				auto x = cfg.dimensions >= 1 ? static_cast<float>(embedding[i * cfg.dimensions + 0]) : 0.0f;
				auto y = cfg.dimensions >= 2 ? static_cast<float>(embedding[i * cfg.dimensions + 1]) : 0.0f;
				auto z = cfg.dimensions >= 3 ? static_cast<float>(embedding[i * cfg.dimensions + 2]) : 0.0f;
				result[i] = v4{ x, y, z, 1.0f };
			}
			return result;
		}
	}

	// Embed N items into low-dimensional space preserving local topology.
	// 'dist(items[i], items[j])' must return a finite non-negative dissimilarity.
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && DistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	std::vector<v4> Embed(Range&& items, DistFunc dist, Config const& cfg = {})
	{
		auto const n = static_cast<int>(std::ranges::size(items));
		auto build_knn = [&]() { return detail::ComputeKNN(items, dist, cfg.neighbor_count); };
		return detail::EmbedImpl(n, build_knn, cfg);
	}

	// Embed with sparse observed distances. 'dist' returning std::nullopt means "this pair is unobserved"
	// and is treated as not a neighbour. Each point's k-NN is built from observed pairs only.
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && OptionalDistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	std::vector<v4> Embed(Range&& items, DistFunc dist, Config const& cfg = {})
	{
		auto const n = static_cast<int>(std::ranges::size(items));
		auto build_knn = [&]() { return detail::ComputeKNN(items, dist, cfg.neighbor_count); };
		return detail::EmbedImpl(n, build_knn, cfg);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm
{
	PRUnitTestClass(UMAPTests)
	{
		PRUnitTestMethod(EmptyAndSingle, Quick)
		{
			auto r0 = umap::Embed(std::span<int const>{}, [](int, int) { return 0.0; });
			PR_EXPECT(r0.empty());

			int one[] = { 42 };
			auto r1 = umap::Embed(one, [](int, int) { return 0.0; });
			PR_EXPECT(r1.size() == 1);
		}
		PRUnitTestMethod(FitABParamsSanity, Quick)
		{
			// Default (spread=1, min_dist=0.1) should give the usual UMAP (a,b) ~ (1.577, 0.895).
			auto [a, b] = umap::detail::FitABParams(1.0, 0.1);
			PR_EXPECT(a > 1.0 && a < 2.5);
			PR_EXPECT(b > 0.5 && b < 1.2);
		}
		PRUnitTestMethod(TwoClustersSeparate, Quick)
		{
			// Two well-separated 2D Gaussian-ish clusters; UMAP should separate them.
			struct Pt { double x, y; };
			auto rng = std::minstd_rand(17);
			auto uni = std::uniform_real_distribution<double>(0.0, 1.0);
			auto pts = std::vector<Pt>(60);
			for (int i = 0; i != 30; ++i)
				pts[i] = { uni(rng), uni(rng) };
			for (int i = 30; i != 60; ++i)
				pts[i] = { uni(rng) + 10.0, uni(rng) + 10.0 };

			auto cfg = umap::Config{};
			cfg.dimensions = 2;
			cfg.neighbor_count = 5;
			cfg.epochs = 200;
			cfg.random_seed = 7;

			auto result = umap::Embed(pts, [](Pt const& a, Pt const& b)
			{
				auto dx = a.x - b.x;
				auto dy = a.y - b.y;
				return std::sqrt(dx * dx + dy * dy);
			}, cfg);
			PR_EXPECT(result.size() == 60);

			auto mx0 = 0.0f, my0 = 0.0f, mx1 = 0.0f, my1 = 0.0f;
			for (int i = 0; i != 30; ++i) { mx0 += result[i].x; my0 += result[i].y; }
			for (int i = 30; i != 60; ++i) { mx1 += result[i].x; my1 += result[i].y; }
			mx0 /= 30; my0 /= 30; mx1 /= 30; my1 /= 30;

			auto cluster_sep = std::sqrt((mx0 - mx1) * (mx0 - mx1) + (my0 - my1) * (my0 - my1));

			auto spread0 = 0.0f;
			for (int i = 0; i != 30; ++i)
			{
				auto dx = result[i].x - mx0;
				auto dy = result[i].y - my0;
				spread0 += std::sqrt(dx * dx + dy * dy);
			}
			spread0 /= 30;

			// Separation comfortably larger than intra-cluster spread.
			// Threshold 1.5 accommodates SGD noise across PRNG implementations — fully-converged
			// UMAP typically achieves 3-10× but the result varies with the random sample sequence.
			PR_EXPECT(cluster_sep > 1.5f * spread0);
		}
		PRUnitTestMethod(SpectralInitChainIsMonotonic, Quick)
		{
			// Exercises the SpectralInit branch directly (which is skipped by Embed() when the
			// k-NN graph has multiple components). Uses a path graph: N nodes connected in a
			// chain 0-1-2-...-(N-1). The 1D SpectralInit embedding should be strongly correlated
			// with the chain index (|Pearson| > 0.95) because the Fiedler vector of a path graph
			// is monotonic along the chain (up to sign and minor endpoint effects from normalisation).
			// A transposed indexing of the eigenvector matrix would give essentially random
			// values across nodes, and correlation would collapse near zero.
			constexpr int N = 20;
			auto graph = umap::detail::FuzzyGraph{};
			graph.heads.reserve(N - 1);
			graph.tails.reserve(N - 1);
			graph.weights.reserve(N - 1);
			for (int i = 0; i != N - 1; ++i)
			{
				graph.heads.push_back(i);
				graph.tails.push_back(i + 1);
				graph.weights.push_back(1.0);
			}

			auto emb = umap::detail::SpectralInit(N, graph, 1, 42);
			PR_EXPECT(emb.size() == static_cast<size_t>(N));

			// Pearson correlation between chain index and embedded coordinate.
			auto mean_x = 0.5 * (N - 1);
			auto mean_y = 0.0;
			for (auto v : emb) mean_y += v;
			mean_y /= N;

			auto sxy = 0.0, sxx = 0.0, syy = 0.0;
			for (int i = 0; i != N; ++i)
			{
				auto dx = i - mean_x;
				auto dy = emb[i] - mean_y;
				sxy += dx * dy;
				sxx += dx * dx;
				syy += dy * dy;
			}
			auto corr = sxy / std::sqrt(sxx * syy);
			PR_EXPECT(std::abs(corr) > 0.95);
		}
		PRUnitTestMethod(EmbedPadsSpectralInitWhenDimExceedsNodeCount, Quick)
		{
			// A connected graph with fewer nodes than requested embedding dimensions must still
			// produce a full output vector. The spectral part should be used where available, and
			// the remaining dimensions stay deterministically seeded.
			struct Pt { double x, y; };
			Pt pts[] =
			{
				{ 0.0, 0.0 },
				{ 1.0, 0.0 },
			};

			auto cfg = umap::Config{};
			cfg.dimensions = 3;
			cfg.neighbor_count = 2;
			cfg.spectral_init = true;
			cfg.epochs = 1;
			cfg.random_seed = 7;

			auto result = umap::Embed(pts, [](Pt const& a, Pt const& b)
			{
				auto dx = a.x - b.x;
				auto dy = a.y - b.y;
				return std::sqrt(dx * dx + dy * dy);
			}, cfg);

			PR_EXPECT(result.size() == 2);
			PR_EXPECT(std::isfinite(result[0].x) && std::isfinite(result[0].y) && std::isfinite(result[0].z));
			PR_EXPECT(std::isfinite(result[1].x) && std::isfinite(result[1].y) && std::isfinite(result[1].z));
		}
	};
}
#endif
