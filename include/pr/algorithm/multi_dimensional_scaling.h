//*********************************************
// Multidimensional Scaling (MDS)
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
// Classical (Torgerson) MDS: embeds items into low-dimensional space preserving pairwise distances.
#pragma once
#include <concepts>
#include <type_traits>
#include <span>
#include <vector>
#include <ranges>
#include <string>
#include <sstream>
#include <iomanip>
#include <optional>
#include <limits>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>
#include "pr/math/math.h"

namespace pr::algorithm::mds
{
	// Configuration for MDS embedding
	struct Config
	{
		// Number of output dimensions (1, 2, or 3). Unused v4 components are zero-filled, w = 1.
		int dimensions = 3;
	};

	// Concept for a distance function between items returning a plain dissimilarity
	template <typename DistFunc, typename Item>
	concept DistanceFunction =
		std::invocable<DistFunc, Item const&, Item const&> &&
		std::convertible_to<std::invoke_result_t<DistFunc, Item const&, Item const&>, float>;

	// Concept for a distance function that may return std::nullopt for unknown pairs
	template <typename DistFunc, typename Item>
	concept OptionalDistanceFunction =
		std::invocable<DistFunc, Item const&, Item const&> &&
		std::same_as<std::invoke_result_t<DistFunc, Item const&, Item const&>, std::optional<float>>;

	// Concept for a predicate that says whether a given pair was directly observed (vs imputed/padded)
	template <typename Pred, typename Item>
	concept ObservedPredicate =
		std::invocable<Pred, Item const&, Item const&> &&
		std::convertible_to<std::invoke_result_t<Pred, Item const&, Item const&>, bool>;

	namespace detail
	{
		// Shared core: build B from an N*N distance matrix D (row-major), eigendecompose, write out.
		template <typename S>
		void EmbedFromMatrix(int n, std::span<S const> D, std::span<v4> out, Config const& config)
		{
			assert(static_cast<int>(D.size()) == n * n);
			assert(static_cast<int>(out.size()) >= n);

			auto const dim = (std::min)(config.dimensions, n - 1);

			// Step 1: Squared distances
			auto D2 = std::vector<S>(n * n);
			for (int i = 0; i != n * n; ++i)
				D2[i] = D[i] * D[i];

			// Step 2: Double centering  B = -1/2 * J * D² * J
			auto row_mean = std::vector<S>(n, S(0));
			auto grand_mean = S(0);
			for (int i = 0; i != n; ++i)
			{
				for (int j = 0; j != n; ++j)
					row_mean[i] += D2[i * n + j];

				row_mean[i] /= static_cast<S>(n);
				grand_mean += row_mean[i];
			}
			grand_mean /= static_cast<S>(n);

			auto B = Matrix<S>(n, n);
			for (int i = 0; i != n; ++i)
				for (int j = 0; j != n; ++j)
					B(i, j) = S(-0.5) * (D2[i * n + j] - row_mean[i] - row_mean[j] + grand_mean);

			// Step 3: Top-k eigendecomposition. Lanczos is O(dim·n²) per restart — essential at scale.
			auto eigen = EigenTopK(B, dim);

			// Step 4: Output coordinates from top 'dim' eigenvectors scaled by sqrt(eigenvalue).
			// Clamp negative eigenvalues to zero (numerical noise from non-Euclidean distances).
			for (int i = 0; i != n; ++i)
			{
				out[i] = v4{
					dim >= 1 ? static_cast<float>(std::sqrt((std::max)(S(0), eigen.values(0))) * eigen.vectors(i, 0)) : 0,
					dim >= 2 ? static_cast<float>(std::sqrt((std::max)(S(0), eigen.values(1))) * eigen.vectors(i, 1)) : 0,
					dim >= 3 ? static_cast<float>(std::sqrt((std::max)(S(0), eigen.values(2))) * eigen.vectors(i, 2)) : 0,
					1,
				};
			}
		}
	}

	// Fill in missing pairwise distances using Floyd-Warshall shortest paths through the graph of
	// known pairs (a.k.a. ISOMAP imputation). 'dist' returns std::nullopt when the pair's distance
	// is unknown. Returns an N*N symmetric row-major matrix with zero on the diagonal.
	// Throws std::runtime_error if the graph of known pairs has disconnected components.
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && OptionalDistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	std::vector<float> ImputeMissing(Range&& items, DistFunc dist)
	{
		constexpr auto INF = std::numeric_limits<float>::infinity();
		auto const n = static_cast<int>(std::ranges::size(items));
		auto d = std::vector<float>(n * n);

		// Seed with direct observations (nullopt -> INF); diagonal = 0
		for (int i = 0; i != n; ++i)
		{
			d[i * n + i] = 0.0f;
			for (int j = i + 1; j != n; ++j)
			{
				auto v = dist(items[i], items[j]);
				auto dv = v.has_value() ? *v : INF;
				d[i * n + j] = dv;
				d[j * n + i] = dv;
			}
		}

		// Floyd-Warshall: d[i,j] = min(d[i,j], d[i,k] + d[k,j])
		for (int k = 0; k != n; ++k)
		{
			for (int i = 0; i != n; ++i)
			{
				auto dik = d[i * n + k];
				if (std::isinf(dik)) continue;
				for (int j = 0; j != n; ++j)
				{
					auto via = dik + d[k * n + j];
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
				if (std::isinf(d[i * n + j]))
					throw std::runtime_error("MDS ImputeMissing: items are not connected via any chain of known pairs.");
			}
		}
		return d;
	}

	// Embed N items into low-dimensional space preserving pairwise distances.
	// 'dist(items[i], items[j])' must return a float dissimilarity >= 0.
	// Returns a vector of v4 points with w=1. Unused dimensions are zero.
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && DistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	void Embed(Range&& items, std::span<v4> out, DistFunc dist, Config const& config = {})
	{
		assert(config.dimensions >= 1 && config.dimensions <= 3);
		assert(out.size() >= std::ranges::size(items));

		if (std::ranges::empty(items))
			return;

		if (std::ranges::size(items) == 1)
		{
			out[0] = v4{0, 0, 0, 1};
			return;
		}

		auto const n = static_cast<int>(std::ranges::size(items));

		// Build N*N distance matrix D
		auto D = std::vector<float>(n * n, 0.0f);
		for (int i = 0; i != n; ++i)
		{
			for (int j = i + 1; j != n; ++j)
			{
				auto d = static_cast<float>(dist(items[i], items[j]));
				D[i * n + j] = d;
				D[j * n + i] = d;
			}
		}

		detail::EmbedFromMatrix<float>(n, std::span<float const>{D}, out, config);
	}
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && DistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	std::vector<v4> Embed(Range&& items, DistFunc dist, Config const& config = {})
	{
		auto out = std::vector<v4>(std::ranges::size(items));
		Embed(items, std::span<v4>{out}, dist, config);
		return out;
	}

	// Embed with sparsely observed distances. A 'dist' result of std::nullopt indicates "this pair's
	// distance is undefined"; such pairs are imputed as shortest-path distances through the graph of
	// known pairs (ISOMAP). Throws std::runtime_error if the known-pairs graph is disconnected.
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && OptionalDistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	void Embed(Range&& items, std::span<v4> out, DistFunc dist, Config const& config = {})
	{
		assert(config.dimensions >= 1 && config.dimensions <= 3);
		assert(out.size() >= std::ranges::size(items));

		if (std::ranges::empty(items))
			return;

		if (std::ranges::size(items) == 1)
		{
			out[0] = v4{0, 0, 0, 1};
			return;
		}

		auto const n = static_cast<int>(std::ranges::size(items));
		auto complete = ImputeMissing(items, dist);
		detail::EmbedFromMatrix<float>(n, std::span<float const>{complete}, out, config);
	}
	template <std::ranges::random_access_range Range, typename DistFunc> requires std::ranges::sized_range<Range> && OptionalDistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	std::vector<v4> Embed(Range&& items, DistFunc dist, Config const& config = {})
	{
		auto out = std::vector<v4>(std::ranges::size(items));
		Embed(items, std::span<v4>{out}, dist, config);
		return out;
	}

	// Fit quality of an MDS embedding vs the target pairwise distances.
	struct FitReport
	{
		// Number of unique off-diagonal pairs compared (N*(N-1)/2)
		int pair_count = 0;

		// Number of pairs flagged as "observed" by the optional predicate (equals pair_count if none supplied)
		int observed_pair_count = 0;

		// Optimal uniform scale factor: Embedded * alpha best approximates Target in a least-squares sense
		double alpha = 1.0;

		// Kruskal stress-1 over all pairs after applying alpha. <0.05 excellent, <0.10 good, <0.20 fair, >0.20 poor.
		double stress = 0.0;

		// Kruskal stress-1 over just the observed pairs (0 if no predicate was supplied)
		double stress_observed = 0.0;

		// Pearson correlation coefficient between embedded and target distances
		double pearson = 0.0;

		// Mean |alpha*d_embedded - d_target| over all pairs
		double mean_abs_error = 0.0;

		// Max  |alpha*d_embedded - d_target| over all pairs
		double max_abs_error = 0.0;

		// Summary string describing the fit report
		std::string summary() const
		{
			// Rough Kruskal interpretation: <0.05 excellent, <0.10 good, <0.20 fair, >0.20 poor
			auto grade = [](double s) -> char const*
			{
				return
					s < 0.05 ? "Excellent" :
					s < 0.10 ? "Good" :
					s < 0.20 ? "Fair" :
					"Poor";
			};

			auto ss = std::ostringstream{};
			ss << std::fixed;
			ss << "MDS fit report:\n";
			ss << "  Pairs:           " << pair_count
			   << " (observed: " << observed_pair_count << ", "
			   << std::setprecision(1) << (100.0 * observed_pair_count / (std::max)(1, pair_count))
			   << "%)\n";
			ss << std::setprecision(4);
			ss << "  Scale alpha:     " << alpha << " (embedded * alpha ~= target)\n";
			ss << "  Kruskal stress:  " << stress << " (" << grade(stress) << ") (all pairs)\n";
			ss << "                   " << stress_observed << " (" << grade(stress_observed) << ") (observed pairs only)\n";
			ss << "  Pearson r:       " << pearson << "\n";
			ss << "  Mean |err|:      " << mean_abs_error << "\n";
			ss << "  Max  |err|:      " << max_abs_error << "\n";
			return ss.str();
		}
	};

	// Measure how well an MDS embedding preserves the pairwise distances of the input items.
	// Because MDS output has arbitrary scale, we first compute the optimal uniform scale factor
	//   alpha = sum(d_emb * d_tgt) / sum(d_emb^2)
	// and report stress/Pearson/errors using that scale. The optional 'is_observed' predicate lets
	// callers distinguish measured ground-truth pairs from padded or imputed pairs; when supplied,
	// an additional stress figure over just the observed pairs is reported.
	template <std::ranges::random_access_range Range, typename DistFunc, typename ObservedFunc = std::nullptr_t> requires std::ranges::sized_range<Range> && DistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	FitReport MeasureFit(Range&& items, std::span<v4 const> positions, DistFunc dist, ObservedFunc is_observed = nullptr)
	{
		using Item = std::ranges::range_value_t<Range>;
		constexpr bool has_observed = !std::same_as<ObservedFunc, std::nullptr_t>;
		if constexpr (has_observed)
			static_assert(ObservedPredicate<ObservedFunc, Item>, "is_observed must be callable (Item, Item) -> bool");

		auto report = FitReport{};

		auto const n = static_cast<int>(std::ranges::size(items));
		assert(static_cast<int>(positions.size()) >= n);
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
				double d_emb = static_cast<double>(Length(positions[i] - positions[j]));
				double d_tgt = static_cast<double>(dist(items[i], items[j]));

				sum_emb_tgt += d_emb * d_tgt;
				sum_emb_sq  += d_emb * d_emb;
				sum_tgt_sq  += d_tgt * d_tgt;
				++pair_count;

				if constexpr (has_observed)
				{
					if (is_observed(items[i], items[j]))
						++obs_count;
				}
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
				double d_emb = static_cast<double>(Length(positions[i] - positions[j])) * alpha;
				double d_tgt = static_cast<double>(dist(items[i], items[j]));
				double err = d_emb - d_tgt;

				num_all += err * err;
				double aerr = std::abs(err);
				abs_err_sum += aerr;
				if (aerr > abs_err_max) abs_err_max = aerr;

				if constexpr (has_observed)
				{
					if (is_observed(items[i], items[j]))
					{
						num_obs += err * err;
						den_obs += d_tgt * d_tgt;
					}
				}

				sum_x  += d_emb;
				sum_y  += d_tgt;
				sum_xy += d_emb * d_tgt;
				sum_xx += d_emb * d_emb;
				sum_yy += d_tgt * d_tgt;
			}
		}

		double stress_all = sum_tgt_sq > 0 ? std::sqrt(num_all / sum_tgt_sq) : 0.0;
		double stress_obs = den_obs > 0 ? std::sqrt(num_obs / den_obs) : 0.0;

		double pearson = 0.0;
		double cov = sum_xy - (sum_x * sum_y) / pair_count;
		double var_x = sum_xx - (sum_x * sum_x) / pair_count;
		double var_y = sum_yy - (sum_y * sum_y) / pair_count;
		if (var_x > 0 && var_y > 0)
			pearson = cov / std::sqrt(var_x * var_y);

		report.pair_count = pair_count;
		report.observed_pair_count = has_observed ? obs_count : pair_count;
		report.alpha = alpha;
		report.stress = stress_all;
		report.stress_observed = stress_obs;
		report.pearson = pearson;
		report.mean_abs_error = abs_err_sum / pair_count;
		report.max_abs_error = abs_err_max;
		return report;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/ldraw.h"
namespace pr::algorithm
{
	PRUnitTestClass(MDSTests)
	{
		PRUnitTestMethod(Empty)
		{
			auto result = mds::Embed(std::span<int const>{}, [](int, int) { return 0.0f; });
			PR_EXPECT(result.empty());
		}
		PRUnitTestMethod(Single)
		{
			int items[] = { 42 };
			auto result = mds::Embed(items, [](int, int) { return 0.0f; });
			PR_EXPECT(result.size() == 1);
			PR_EXPECT(std::abs(result[0].w - 1.0f) < 1e-5f);
		}
		PRUnitTestMethod(KnownSquare)
		{
			// Four points forming a unit square. The distance function returns pre-computed distances.
			// Points: (0,0), (1,0), (1,1), (0,1) — distances: adjacent=1, diagonal=sqrt(2)
			struct Point { float x, y; };
			Point pts[] = { {0,0}, {1,0}, {1,1}, {0,1} };

			auto euclidean = [](Point const& a, Point const& b)
			{
				auto dx = a.x - b.x;
				auto dy = a.y - b.y;
				return std::sqrt(dx * dx + dy * dy);
			};
			auto result = mds::Embed(pts, euclidean, { .dimensions = 2 });
			PR_EXPECT(result.size() == 4);

			// Verify pairwise distances are preserved (up to rotation/reflection)
			for (int i = 0; i != 4; ++i)
			{
				for (int j = i + 1; j != 4; ++j)
				{
					auto orig_d = euclidean(pts[i], pts[j]);
					auto dx = result[i].x - result[j].x;
					auto dy = result[i].y - result[j].y;
					auto embed_d = std::sqrt(dx * dx + dy * dy);
					PR_EXPECT(std::abs(orig_d - embed_d) < 0.05f);
				}
			}

			// Verify w=1 and z=0 for 2D embedding
			for (auto& p : result)
			{
				PR_EXPECT(std::abs(p.z) < 1e-4f);
				PR_EXPECT(std::abs(p.w - 1.0f) < 1e-5f);
			}
		}
		PRUnitTestMethod(EmbedWithMissingDistances)
		{
			// Square: only adjacent-edge distances are known; diagonals are unknown (nullopt).
			// ISOMAP should impute the diagonal as 1+1=2 via a path through a corner.
			struct Point { float x, y; };
			Point pts[] = { {0,0}, {1,0}, {1,1}, {0,1} };

			auto sparse = [](Point const& a, Point const& b) -> std::optional<float>
			{
				auto dx = a.x - b.x;
				auto dy = a.y - b.y;
				auto d = std::sqrt(dx * dx + dy * dy);
				if (d < 1.1f) return d; // only report unit edges
				return std::nullopt;
			};

			// Imputation step: diagonals should be filled as 2 (path-through)
			auto mat = mds::ImputeMissing(pts, sparse);
			PR_EXPECT(std::abs(mat[0 * 4 + 1] - 1.0f) < 1e-5f);
			PR_EXPECT(std::abs(mat[0 * 4 + 2] - 2.0f) < 1e-5f);
			PR_EXPECT(std::abs(mat[1 * 4 + 3] - 2.0f) < 1e-5f);

			// Embedding should succeed and produce 4 finite points; exact layout is a compromise
			// because the imputed distances aren't Euclidean-consistent (4 edges of 1 and 2 diagonals of 2).
			auto result = mds::Embed(pts, sparse, { .dimensions = 2 });
			PR_EXPECT(result.size() == 4);
			for (int i = 0; i != 4; ++i)
			{
				PR_EXPECT(std::isfinite(result[i].x));
				PR_EXPECT(std::isfinite(result[i].y));
			}
		}
		PRUnitTestMethod(ImputeMissingDisconnected)
		{
			// Two disconnected pairs: {0,1} and {2,3}. Floyd-Warshall cannot bridge them.
			int items[] = { 0, 1, 2, 3 };
			auto dist = [](int a, int b) -> std::optional<float>
			{
				if ((a == 0 && b == 1) || (a == 1 && b == 0)) return 1.0f;
				if ((a == 2 && b == 3) || (a == 3 && b == 2)) return 1.0f;
				return std::nullopt;
			};

			bool threw = false;
			try { (void)mds::ImputeMissing(items, dist); }
			catch (std::runtime_error const&) { threw = true; }
			PR_EXPECT(threw);
		}
		PRUnitTestMethod(FitPerfectEmbedding)
		{
			// Unit square -> perfect 2D embedding. Stress should be ~0, Pearson ~1.
			struct Point { float x, y; };
			Point pts[] = { {0,0}, {1,0}, {1,1}, {0,1} };

			auto euclidean = [](Point const& a, Point const& b)
			{
				auto dx = a.x - b.x;
				auto dy = a.y - b.y;
				return std::sqrt(dx * dx + dy * dy);
			};

			auto positions = mds::Embed(pts, euclidean, { .dimensions = 2 });
			auto report = mds::MeasureFit(pts, std::span<v4 const>{positions}, euclidean);

			PR_EXPECT(report.pair_count == 6);
			PR_EXPECT(report.observed_pair_count == 6);
			PR_EXPECT(report.stress < 0.01);
			PR_EXPECT(report.pearson > 0.99);
			PR_EXPECT(report.max_abs_error < 1e-3);
		}
		PRUnitTestMethod(FitWithObservedPredicate)
		{
			// Fit report with is_observed flags only edges (unit distance) as observed.
			struct Point { float x, y; };
			Point pts[] = { {0,0}, {1,0}, {1,1}, {0,1} };

			auto euclidean = [](Point const& a, Point const& b)
			{
				auto dx = a.x - b.x;
				auto dy = a.y - b.y;
				return std::sqrt(dx * dx + dy * dy);
			};

			auto positions = mds::Embed(pts, euclidean, { .dimensions = 2 });
			auto is_edge = [&](Point const& a, Point const& b) { return euclidean(a, b) < 1.01f; };
			auto report = mds::MeasureFit(pts, std::span<v4 const>{positions}, euclidean, is_edge);

			PR_EXPECT(report.pair_count == 6);
			PR_EXPECT(report.observed_pair_count == 4); // 4 edges, 2 diagonals
			PR_EXPECT(report.stress_observed < 0.01);
		}
		PRUnitTestMethod(Visualise)
		{
			constexpr int N = 100;

			// Returns a spherical direction vector corresponding to the ith point of a Fibonacci sphere
			auto fib_sphere = [](int i) -> v4
			{
				// Z goes from -1 to +1
				// Using a half step bias so that there is no point at the poles.
				// This prevents degenerates during 'unmapping' and also results in more evenly
				// spaced points. See "Fibonacci grids: A novel approach to global modelling".
				auto z = -1.0 + (2.0 * i + 1.0) / N;

				// Radius at z
				auto r = sqrt(1.0 - z * z);

				// Golden angle increment
				auto theta = i * constants<double>::golden_angle;
				auto x = cos(theta) * r;
				auto y = sin(theta) * r;
				return v4{ (float)x, (float)y, (float)z, 0 };
			};

			auto result = mds::Embed(std::views::iota(0, N), [&](int i, int j) { return Length(fib_sphere(i) - fib_sphere(j)); }, { .dimensions = 3 });
			PR_EXPECT(result.size() == static_cast<size_t>(N));

			#if PR_UNITTESTS_VISUALISE
			{
				ldraw::Builder builder;
				auto& pts = builder.Point("points", 0xFF00CC00).size(4.0f);
				for (auto& p : result)
					pts.pt(p.x, p.y, p.z);

				builder.Save(temp_dir() / "MDS_FibSphere.ldr");
			}
			#endif
		}
		PRUnitTestMethod(Visualise2)
		{
			// Cluster strings by edit distance, embed into 2D, visualise with ldraw.
			// Three clusters of similar strings should separate in the embedding.
			struct Item { char const* text; int cluster; };
			Item items[] = {
				// Cluster 0 (red): short 'a'-heavy strings
				{"aaa", 0}, {"aab", 0}, {"aba", 0}, {"aac", 0}, {"baa", 0}, {"aaab", 0},
				// Cluster 1 (green): 'x'-heavy strings
				{"xxx", 1}, {"xxy", 1}, {"xyx", 1}, {"yxx", 1}, {"xxxy", 1}, {"xxyx", 1},
				// Cluster 2 (blue): longer 'm/n' strings
				{"mmmm", 2}, {"mmmn", 2}, {"mnmm", 2}, {"nmmm", 2}, {"mmnn", 2}, {"mnmn", 2},
			};

			// Simple edit distance (Levenshtein)
			auto edit_distance = [](Item const& a, Item const& b)
			{
				auto sa = std::string_view(a.text);
				auto sb = std::string_view(b.text);
				auto const na = static_cast<int>(sa.size());
				auto const nb = static_cast<int>(sb.size());

				auto prev = std::vector<int>(nb + 1);
				auto curr = std::vector<int>(nb + 1);
				std::iota(prev.begin(), prev.end(), 0);

				for (int i = 1; i <= na; ++i)
				{
					curr[0] = i;
					for (int j = 1; j <= nb; ++j)
					{
						auto cost = (sa[i - 1] == sb[j - 1]) ? 0 : 1;
						curr[j] = (std::min)({curr[j - 1] + 1, prev[j] + 1, prev[j - 1] + cost});
					}
					std::swap(prev, curr);
				}
				return static_cast<float>(prev[nb]);
			};

			mds::Config config;
			config.dimensions = 2;
			auto const n = static_cast<int>(std::size(items));
			auto result = mds::Embed(std::span<Item const>{items, static_cast<size_t>(n)}, edit_distance, config);
			PR_EXPECT(result.size() == static_cast<size_t>(n));

			#if PR_UNITTESTS_VISUALISE
			{
				ldraw::Builder builder;

				// Cluster colours: red, green, blue
				uint32_t colours[] = { 0xFFFF0000, 0xFF00CC00, 0xFF0066FF };

				// Plot each cluster's points
				for (int c = 0; c != 3; ++c)
				{
					auto name = std::string("cluster") + std::to_string(c);
					auto& pts = builder.Point(std::string_view{ name }, colours[c]).size(12.0f);
					for (int i = 0; i != n; ++i)
						if (items[i].cluster == c)
							pts.pt(result[i].x, result[i].y, 0.0f);
				}

				// Draw lines between all items in the same cluster to show grouping
				for (int c = 0; c != 3; ++c)
				{
					auto name = std::string("links") + std::to_string(c);
					auto& lines = builder.Line(std::string_view{ name }, colours[c] & 0x40FFFFFF); // transparent version
					auto cluster_indices = std::vector<int>();
					for (int i = 0; i != n; ++i)
						if (items[i].cluster == c)
							cluster_indices.push_back(i);

					for (int a = 0; a != static_cast<int>(cluster_indices.size()); ++a)
						for (int b = a + 1; b != static_cast<int>(cluster_indices.size()); ++b)
						{
							auto ia = cluster_indices[a];
							auto ib = cluster_indices[b];
							lines.line(
								ldraw::seri::Vec3{ result[ia].x, result[ia].y, 0.0f },
								ldraw::seri::Vec3{ result[ib].x, result[ib].y, 0.0f });
						}
				}

				builder.Save(temp_dir() / "MDS.ldr");
			}
			#endif
		}
	};
}
#endif
