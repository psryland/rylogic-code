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

	// Concept for a distance function between items
	template <typename DistFunc, typename Item>
	concept DistanceFunction = 
		std::invocable<DistFunc, Item const&, Item const&> && 
		std::convertible_to<std::invoke_result_t<DistFunc, Item const&, Item const&>, float>;

	// Embed N items into low-dimensional space preserving pairwise distances.
	// 'dist(items[i], items[j])' must return a float dissimilarity >= 0.
	// Returns a vector of v4 points with w=1. Unused dimensions are zero.
	template <std::ranges::random_access_range Range, typename DistFunc>
		requires std::ranges::sized_range<Range> && DistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	void Embed(Range&& items, std::span<v4> out, DistFunc dist, Config const& config = {})
	{
		assert(config.dimensions >= 1 && config.dimensions <= 3);
		assert(out.size() >= std::ranges::size(items));

		if (std::ranges::empty(items))
		{
			return;
		}
		if (std::ranges::size(items) == 1)
		{
			out[0] = v4{0, 0, 0, 1};
			return;
		}

		auto const n = static_cast<int>(std::ranges::size(items));
		auto const dim = (std::min)(config.dimensions, n - 1);

		// Step 1: Build N×N squared distance matrix D²
		auto D2 = std::vector<float>(n * n, 0.0f);
		for (int i = 0; i != n; ++i)
		{
			for (int j = i + 1; j != n; ++j)
			{
				auto d = static_cast<float>(dist(items[i], items[j]));
				auto d2 = d * d;
				D2[i * n + j] = d2;
				D2[j * n + i] = d2;
			}
		}

		// Step 2: Double centering to get the inner-product matrix B = -1/2 * J * D² * J
		// where J = I - (1/N) * 11ᵀ. Equivalent to: B[i][j] = -1/2 * (D²[i][j] - row_mean[i] - col_mean[j] + grand_mean)
		auto row_mean = std::vector<float>(n, 0.0f);
		auto grand_mean = 0.0f;
		for (int i = 0; i != n; ++i)
		{
			for (int j = 0; j != n; ++j)
				row_mean[i] += D2[i * n + j];

			row_mean[i] /= static_cast<float>(n);
			grand_mean += row_mean[i];
		}
		grand_mean /= static_cast<float>(n);

		// Step 3: Build the inner-product matrix B as a Matrix<float> and eigendecompose.
		// Use EigenTopK for the top 'dim' eigenpairs, which is much faster than full decomposition for large N.
		auto B = Matrix<float>(n, n, D2.data());
		for (int i = 0; i != n; ++i)
			for (int j = 0; j != n; ++j)
				B(i, j) = -0.5f * (D2[i * n + j] - row_mean[i] - row_mean[j] + grand_mean);

		auto eigen = EigenTopK(B, dim);

		// Step 4: Build output coordinates from top 'dim' eigenvectors scaled by sqrt(eigenvalue)
		for (int i = 0; i != n; ++i)
		{
			// Clamp negative eigenvalues to zero (numerical noise from non-Euclidean distances)
			auto pt = v4{
				dim >= 1 ? std::sqrt((std::max)(0.0f, eigen.values(0))) * eigen.vectors(i, 0) : 0,
				dim >= 2 ? std::sqrt((std::max)(0.0f, eigen.values(1))) * eigen.vectors(i, 1) : 0,
				dim >= 3 ? std::sqrt((std::max)(0.0f, eigen.values(2))) * eigen.vectors(i, 2) : 0,
				1,
			};
			out[i] = pt;
		}
	}
	template <std::ranges::random_access_range Range, typename DistFunc>
		requires std::ranges::sized_range<Range>
		      && DistanceFunction<DistFunc, std::ranges::range_value_t<Range>>
	std::vector<v4> Embed(Range&& items, DistFunc dist, Config const& config = {})
	{
		auto out = std::vector<v4>(std::ranges::size(items));
		Embed(items, out, dist, config);
		return out;
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

				builder.Save(temp_dir() / "MDS_FibSphere.ldr", ldraw::ESaveFlags::Pretty);
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

				builder.Save(temp_dir() / "MDS.ldr", ldraw::ESaveFlags::Pretty);
			}
			#endif
		}
	};
}
#endif
