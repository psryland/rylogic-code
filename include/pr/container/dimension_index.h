#pragma once
#include <vector>
#include <algorithm>
#include <execution>
#include <numeric>
#include <concepts>
#include <span>
#include <cassert>

namespace pr::spatial
{
	// Concept for a dimension value
	template <typename S> concept ValueType = requires(S s)
	{
		{ s - s };
		{ s * s };
		{ s < s };
	};

	// Concept for a 'S GetValue(Item const& item, int axis)' function
	template <typename T, typename S, typename Item> concept GetValueFunc = requires(T t)
	{
		ValueType<S>;
		{ t(std::declval<Item const&>(), std::declval<int>()) } -> std::convertible_to<S>;
	};

	// Concept for a 'void Found(Item const&)' function
	template <typename T, typename S, typename Item> concept FoundFunc = requires(T t)
	{
		ValueType<S>;
		{ t(std::declval<Item const&>()) };
	};

	// Concept for a 'void Found(Item const&, S dist_sq)' function
	template <typename T, typename S, typename Item> concept FoundInSphereFunc = requires(T t)
	{
		ValueType<S>;
		{ t(std::declval<Item const&>(), std::declval<S>()) };
	};

	// A index on N dimensions 
	template <int Dimensions, ValueType S = float, std::integral Index = int>
	struct DimensionIndex
	{
		// Notes:
		// - Stores a sorted list of object indices for each dimension.
		//   Storage = sizeof(Index) * Dimensions * items.size()
		// - Search is O(Dimensions * log(N)), however each dimension is searched in parallel.
		using IndexContainer = std::vector<Index>;
		using IndexRange = std::span<Index>;

		// Indices sorted on each dimension.
		IndexContainer m_space[Dimensions];

		DimensionIndex() = default;

		// Spatially partition 'items'
		template <typename Item, GetValueFunc<S, Item> GetValue>
		void Build(std::span<Item const> items, GetValue get_value)
		{
			auto count = items.size();

			// Initialise each dimension with a list of indices
			m_space[0].resize(count);
			std::iota(std::begin(m_space[0]), std::end(m_space[0]), 0);
			for (int i = 1; i != Dimensions; ++i)
				m_space[i] = m_space[0]; // value copy

			// Sort on each dimension
			Update(items, get_value);
		}

		// Re-sort the index for the same number of items
		template <typename Item, GetValueFunc<S, Item> GetValue>
		void Update(std::span<Item const> items, GetValue get_value)
		{
			assert(items.size() == m_space[0].size());

			// For each dimension, sort on the item's position in that dimension
			std::for_each(std::execution::par, std::begin(m_space), std::end(m_space), [&](auto& space)
			{
				auto dim = s_cast<int>(&space - &m_space[0]);
				std::sort(std::begin(space), std::end(space), [&](int a, int b)
				{
					return get_value(items[a], dim) < get_value(items[b], dim);
				});
			});
		}

		// Find items within 'bbox' of 'search'
		template <typename Item, GetValueFunc<S, Item> GetValue, FoundFunc<S, Item> Found>
		void Find(std::span<Item const> items, S const (&search)[Dimensions], S const (&bbox)[Dimensions], GetValue get_value, Found found)
		{
			// The range of items on each dimension that are within 'bbox' of 'search'
			IndexRange ranges[Dimensions];

			// On each dimension, find the range of indices that are within 'bbox' of 'search'
			std::for_each(std::execution::par, std::begin(m_space), std::end(m_space), [&](auto& space)
			{
				auto dim = s_cast<int>(&space - &m_space[0]);
				auto lower = search[dim] - bbox[dim];
				auto upper = search[dim] + bbox[dim];
				auto lo = std::lower_bound(std::begin(space), std::end(space), lower, [&](int a, S b)
				{
					return get_value(items[a], dim) < b;
				});
				auto hi = std::upper_bound(lo, std::end(space), upper, [&](S a, int b)
				{
					return a < get_value(items[b], dim);
				});

				ranges[dim] = { lo, hi };
			});

			// Find the narrowest range
			auto narrowest = std::min_element(std::begin(ranges), std::end(ranges), [](auto const& a, auto const& b)
			{
				return a.size() < b.size();
			});

			// Reduce the results to the intersection on all dimensions
			for (auto& idx : *narrowest)
			{
				// Check if 'idx' is present in all the other dimensions
				auto in_all = std::all_of(std::begin(ranges), std::end(ranges), [&](auto const& range)
				{
					return &range == &*narrowest || std::ranges::find(range, idx) != std::end(range);
				});

				// If not in all ranges, skip
				if (in_all)
					found(items[idx]);
			}
		}

		// Find items within 'radius' of 'search'
		template <typename Item, GetValueFunc<S, Item> GetValue, FoundInSphereFunc<S, Item> Found>
		void Find(std::span<Item const> items, S const (&search)[Dimensions], S radius, GetValue get_value, Found found)
		{
			S r[Dimensions];
			std::fill(std::begin(r), std::end(r), radius);

			auto radius_sq = radius * radius;

			// Convert a box search to a sphere search
			Find(items, search, r, get_value, [=](Item const& item)
			{
				S dist_sq = {};
				for (int i = 0; i != Dimensions; ++i)
				{
					auto diff = get_value(item, i) - search[i];
					dist_sq += diff * diff;
				}
				if (dist_sq < radius_sq)
				{
					found(item, dist_sq);
				}
			});
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"

namespace pr::container
{
	PRUnitTest(DimensionIndexTests)
	{
		std::vector<v2> points =
		{
			{ 0.2f, 0.5f },
			{ 0.6f, 0.2f },
			{ 0.3f, 0.3f },
			{ 0.5f, 0.6f },
			{ 0.1f, 0.1f },
			{ 0.4f, 0.4f },
		};

		auto GetValue = [](v2 const& p, int i)
		{
			return p[i];
		};

		std::vector<v2> results;
		spatial::DimensionIndex<2, float> index;
		index.Build<v2>(points, GetValue);

		{
			results.resize(0);
			index.Find<v2>(points, { 0.3f, 0.3f }, 0.01f, GetValue, [&](v2 const& a, float)
			{
				results.push_back(a);
			});

			PR_EXPECT(results.size() == 1);
			PR_EXPECT(results[0] == points[2]);
		}
		{
			results.resize(0);
			index.Find<v2>(points, { 0.3f, 0.3f }, 0.2f, GetValue, [&](v2 const& a, float)
			{
				results.push_back(a);
			});

			PR_EXPECT(results.size() == 2);
			PR_EXPECT(std::ranges::contains(results, points[2]));
			PR_EXPECT(std::ranges::contains(results, points[5]));
		}
	}
	PRUnitTest(DimensionIndexLdrTests)
	{
		std::random_device rd;
		std::default_random_engine rng(rd());
		std::uniform_real_distribution dist(0.2f, 0.5f);
		rdr12::ldraw::Builder builder;

		const int N = 10000;
		std::vector<v4> points;
		{
			auto& input = builder.Point("points", 0xFFA0A0A0).size(3.0f);
			for (int i = 0; i != N; ++i)
			{
				points.push_back(v3::Random(rng, -v3::One(), +v3::One()).w1());
				input.pt(points.back());
			}
		}

		auto GetValue = [](v4 p, int i)
		{
			return p[i];
		};

		spatial::DimensionIndex<3, float> index;
		index.Build<v4>(points, GetValue);

		auto search = v3::Random(rng, -v3::One(), +v3::One()).w1();
		auto radius = dist(rng);
		builder.Sphere("search", 0x8000FF00).radius(radius).pos(search);

		{
			auto& results = builder.Point("results", 0xFF00FF00).size(10.0f);
			index.Find<v4>(points, search.xyz.arr, radius, GetValue, [&](v4 a, float dist_sq)
			{
				PR_EXPECT(dist_sq < radius * radius + maths::tiny<float>);
				results.pt(a);
			});
		}

		builder.Save("E:/Dump/dimension_index.ldr");
	}
}

#endif
