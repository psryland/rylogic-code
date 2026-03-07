//*****************************************
// Vantage Point Tree
//  Copyright (c) Rylogic Ltd 2025
//*****************************************
#pragma once
#include <span>
#include <concepts>
#include <algorithm>
#include <limits>
#include <cassert>

namespace pr::vp_tree
{
	// Concept for the 'ChooseVP' function
	template <typename T, typename Item> concept ChooseVPFunc = requires(T t)
	{
		// Returns the index of the item to be used as the vantage point
		{ t(std::declval<std::span<Item>>()) } -> std::convertible_to<size_t>;
	};

	// Concept for the 'MeasureDistance' function
	template <typename T, typename Lhs, typename Rhs, typename S> concept MeasureDistanceFunc = std::floating_point<S> && requires(T t)
	{
		{ t(std::declval<Lhs const&>(), std::declval<Rhs const&>()) } -> std::convertible_to<S>;
	};

	// Concept for the 'SaveThreshold' function
	template <typename T, typename Item, typename S> concept SaveThresholdFunc = std::floating_point<S> && requires(T t)
	{
		{ t(std::declval<Item&>(), std::declval<S>()) };
	};

	// Concept for the 'GetThreshold' function
	template <typename T, typename Item, typename S> concept GetThresholdFunc = std::floating_point<S> && requires(T t)
	{
		{ t(std::declval<Item const&>()) } -> std::convertible_to<S>;
	};

	// Concept of an output function for found items
	template <typename T, typename Item, typename S> concept FoundFunc = std::floating_point<S> && requires(T t)
	{
		{ t(std::declval<Item const&>(), std::declval<S>()) };
	};

	// VP Tree functions
	template <typename Item, std::floating_point DistanceType = float>
	struct VPTree
	{
		// Notes:
		//  - The first item in each sub-tree is the vantage point (VP) for that sub-tree.
		//  - The distance threshold is the distance from the VP to the median item in that sub-tree.
		//  - 'Distance' is typically the Euclidean distance but can be any metric distance function (including distance squared).
		//  - This code doesn't require that the threshold value is stored within 'Item'. In fact, the threshold doesn't need to 
		//    be stored at all because it can be calculated on the fly during searching. This is up to the caller.
		//  - 'SearchCentre' is an arbitrary type. Typically, it will be same as 'Item' but isn't required to be.
		//  - A VP tree can be built on const items if 'VPTree<Item const>' is used. In this case the threshold must be stored externally.
		//  - The choice of VP can be arbitrary because we always partition the range 50:50 with the left side being the "near" items
		//    and the right side being the "far" items. However, the choice of VP does affect the efficiency of searching the tree.
		//  - If calculating distance is expensive, consider caching the distances.
		struct Neighbour
		{
			Item const* item;
			DistanceType distance;
		};
		struct Pair
		{
			Item const* item0;
			Item const* item1;
			DistanceType distance;

			friend bool operator == (Pair const& lhs, Pair const& rhs)
			{
				return
					(lhs.item0 == rhs.item0 && lhs.item1 == rhs.item1) ||
					(lhs.item0 == rhs.item1 && lhs.item1 == rhs.item0);
			}
			friend bool operator != (Pair const& lhs, Pair const& rhs)
			{
				return !(lhs == rhs);
			}
		};

		// Build a vp-tree from 'items'
		template
		<
			ChooseVPFunc<Item> ChooseVP,
			MeasureDistanceFunc<Item, Item, DistanceType> MeasureDistance,
			SaveThresholdFunc<Item, DistanceType> SaveThreshold
		>
		static void Build(std::span<Item> items, ChooseVP choose_vp, MeasureDistance measure_distance, SaveThreshold save_threshold)
		{
			struct Builder
			{
				ChooseVP m_choose_vp;
				MeasureDistance m_measure_distance;
				SaveThreshold m_save_threshold;

				Builder(ChooseVP choose_vp, MeasureDistance measure_distance, SaveThreshold save_threshold)
					: m_choose_vp(choose_vp)
					, m_measure_distance(measure_distance)
					, m_save_threshold(save_threshold)
				{}

				// Recursively build a VP tree within 'items'
				void Run(std::span<Item> items, int level)
				{
					if (items.size() <= 1)
						return;

					// Choose an item to be the vantage point and swap this to the front of the range.
					auto vp_idx = m_choose_vp(items);
					assert(vp_idx >= 0 && vp_idx < items.size());
					std::swap(items[0], items[vp_idx]);

					// Split the range. This ensures all values < mid are nearer to the VP than all values > mid.
					// Note: that values equidistant to the VP can be on either side of the split.
					auto mid = items.size() / 2;
					std::nth_element(items.data() + 1, items.data() + mid, items.data() + items.size(), [&](auto const& lhs, auto const& rhs)
					{
						return m_measure_distance(items[0], lhs) < m_measure_distance(items[0], rhs);
					});

					// Record threshold distance at the split point
					auto threshold = m_measure_distance(items[0], items[mid]);
					m_save_threshold(items[0], threshold);

					// Construct recursively
					Run(items.subspan(1, mid - 1), level + 1);
					Run(items.subspan(mid), level + 1);
				}
			};

			Builder builder(choose_vp, measure_distance, save_threshold);
			builder.Run(items, 0);
		}

		// Search a VP tree for all items within 'radius' of 'centre'.
		template
		<
			typename SearchCentre,
			MeasureDistanceFunc<Item, SearchCentre, DistanceType> MeasureDistance,
			GetThresholdFunc<Item, DistanceType> GetThreshold,
			FoundFunc<Item, DistanceType> Found
		>
		static void Find(std::span<Item const> vptree, SearchCentre const& centre, DistanceType radius, MeasureDistance measure_distance, GetThreshold get_threshold, Found found)
		{
			struct Finder
			{
				SearchCentre const& m_search_centre;
				DistanceType m_search_radius;
				MeasureDistance m_measure_distance;
				GetThreshold m_get_threshold;
				Found m_found;

				Finder(SearchCentre const& centre, DistanceType radius, MeasureDistance measure_distance, GetThreshold get_threshold, Found found)
					: m_search_centre(centre)
					, m_search_radius(radius)
					, m_measure_distance(measure_distance)
					, m_get_threshold(get_threshold)
					, m_found(found)
				{}

				// Find items within a (hyper)spherical search volume
				void Run(std::span<Item const> items)
				{
					if (items.empty())
						return;

					// The VP is the first item in the sub-tree. Measure the distance to it from the search centre
					auto distance = m_measure_distance(items[0], m_search_centre);
					if (distance <= m_search_radius)
						m_found(items[0], distance);

					// Bottom of the tree? Time to leave
					if (items.size() <= 1)
						return;

					// Get the threshold distance
					auto threshold = m_get_threshold(items[0]);
					auto mid = items.size() / 2;

					// If 'search_centre - search_radius <= threshold' then we need to search the near sub tree
					if (distance - m_search_radius <= threshold)
						Run(items.subspan(1, mid - 1));

					// If the 'search_centre + search_radius >= threshold' then we need to search the far sub tree
					if (distance + m_search_radius >= threshold)
						Run(items.subspan(mid));
				}
			};

			// Recursive search
			Finder finder(centre, radius, measure_distance, get_threshold, found);
			finder.Run(vptree);
		}

		// Search a VP tree for the 'N' nearest neighbours within 'radius' of 'centre'.
		// Neighbours are returned in order of increasing distance from the search point.
		// Return value is the number of neighbours found (may be less than nearest_out.size()).
		template
		<
			typename SearchCentre,
			MeasureDistanceFunc<Item, SearchCentre, DistanceType> MeasureDistance,
			GetThresholdFunc<Item, DistanceType> GetThreshold
		>
		static size_t FindNearest(std::span<Item const> vptree, SearchCentre const& centre, DistanceType radius, std::span<Neighbour> nearest_out, MeasureDistance measure_distance, GetThreshold get_threshold)
		{
			struct Finder
			{
				std::span<Neighbour> m_nearest;
				SearchCentre const& m_search_centre;
				DistanceType m_search_radius;
				MeasureDistance m_measure_distance;
				GetThreshold m_get_threshold;
				size_t m_count;

				Finder(std::span<Neighbour> nearest, SearchCentre const& centre, DistanceType radius, MeasureDistance measure_distance, GetThreshold get_threshold)
					: m_nearest(nearest)
					, m_search_centre(centre)
					, m_search_radius(radius)
					, m_measure_distance(measure_distance)
					, m_get_threshold(get_threshold)
					, m_count(0)
				{
					assert(!m_nearest.empty());
					for (auto& n : m_nearest)
						n = { nullptr, std::numeric_limits<DistanceType>::infinity() };
				}

				// Find items within a (hyper)spherical search volume
				void Run(std::span<Item const> items)
				{
					if (items.empty())
						return;

					// The VP is the first item in the sub-tree. Measure the distance to it from the search centre
					auto distance = m_measure_distance(items[0], m_search_centre);
					if (distance <= m_search_radius)
						TrackNearest(items[0], distance);

					// Bottom of the tree? Time to leave
					if (items.size() <= 1)
						return;

					// Get the threshold distance
					auto threshold = m_get_threshold(items[0]);
					auto mid = items.size() / 2;

					// If 'search_centre - search_radius <= threshold' then we need to search the near sub tree
					if (distance - m_search_radius <= threshold)
						Run(items.subspan(1, mid - 1));

					// If the 'search_centre + search_radius >= threshold' then we need to search the far sub tree
					if (distance + m_search_radius >= threshold)
						Run(items.subspan(mid));
				}

				// Keep the 'N' nearest neighbours found so far
				void TrackNearest(Item const& item, DistanceType distance)
				{
					// Heap uses > for highest priority. For this case, largest distance has highest priority
					constexpr auto MaxHeap = [](Neighbour const& lhs, Neighbour const& rhs) { return lhs.distance < rhs.distance; };

					// If we have less than 'N' nearest, add it
					if (m_count != m_nearest.size())
					{
						m_nearest[m_count++] = { &item, distance };
						std::push_heap(m_nearest.data(), m_nearest.data() + m_count, MaxHeap);
					}

					// Otherwise, if it is closer than the furthest, replace it
					else if (distance < m_nearest[0].distance)
					{
						std::pop_heap(m_nearest.data(), m_nearest.data() + m_count, MaxHeap);
						m_nearest[m_count - 1] = { &item, distance };
						std::push_heap(m_nearest.data(), m_nearest.data() + m_count, MaxHeap);
					
						// Restrict the search radius to the distance to the furthest nearest neighbour found so far
						m_search_radius = m_nearest[0].distance;
					}
				}
			};

			// Recursive search
			Finder finder(nearest_out, centre, radius, measure_distance, get_threshold);
			finder.Run(vptree);

			// Sort the nearest neighbours into increasing distance order
			std::sort(nearest_out.begin(), nearest_out.begin() + finder.m_count, [](Neighbour const& lhs, Neighbour const& rhs)
			{
				return lhs.distance < rhs.distance;
			});
			return finder.m_count;
		}

		// Find pairs of items that are the nearest to each other
		// Pairs are returned in order of increasing separation.
		// Return value is the number of pairs found (may be less than pairs_out.size()).
		template
		<
			MeasureDistanceFunc<Item, Item, DistanceType> MeasureDistance,
			GetThresholdFunc<Item, DistanceType> GetThreshold
		>
		static size_t Closest(std::span<Item const> vptree, DistanceType radius, std::span<Pair> pairs_out, MeasureDistance measure_distance, GetThreshold get_threshold)
		{
			struct Finder
			{
				std::span<Pair> m_pairs;
				DistanceType m_search_radius;
				MeasureDistance m_measure_distance;
				GetThreshold m_get_threshold;
				size_t m_count;

				Finder(std::span<Pair> pairs, DistanceType radius, MeasureDistance measure_distance, GetThreshold get_threshold)
					: m_pairs(pairs)
					, m_search_radius(radius)
					, m_measure_distance(measure_distance)
					, m_get_threshold(get_threshold)
					, m_count(0)
				{
					assert(!m_pairs.empty());
					for (auto& p : m_pairs)
						p = { nullptr, nullptr, std::numeric_limits<DistanceType>::infinity() };
				}

				// Find nodes with the minimum separation
				void Run(std::span<Item const> items)
				{
					// The brute force method would be to compare each item with all other items, O(N^2).
					// However, we can use the tree to reduce the number of comparisons by not searching
					// sub-trees that can't be closer to the target than the least-nearest pair.
					for (Item const& target : items)
						FindClosest(target, items);
				}

				// Find the closest items to 'target'
				void FindClosest(Item const& target, std::span<Item const> items)
				{
					// We only consider pairs when '&target < &items[...]' to prevent duplicates.
					if (items.empty() || &target >= &items.back())
						return;

					auto distance = m_measure_distance(items[0], target);
					if (distance <= m_search_radius && &target < &items[0])
						TrackNearest(target, items[0], distance);

					// Bottom of the tree? Time to leave
					if (items.size() <= 1)
						return;

					auto threshold = m_get_threshold(items[0]);
					auto mid = items.size() / 2;

					// Search the near sub tree if anything in that tree could be closer than the least nearest pair found so far.
					if (distance - m_search_radius <= threshold)
						FindClosest(target, items.subspan(1, mid - 1));

					// Search the far sub tree if anything in that tree could be closer than the least nearest pair found so far.
					if (distance + m_search_radius >= threshold)
						FindClosest(target, items.subspan(mid));
				}

				// Track the distance at which there are 'N' closer items
				void TrackNearest(Item const& lhs, Item const& rhs, DistanceType distance)
				{
					// Heap uses > for highest priority. For this case, largest separation has highest priority
					constexpr auto MaxHeap = [](Pair const& lhs, Pair const& rhs) { return lhs.distance < rhs.distance; };
					assert("Should only be considering pairs when '&lhs < &rhs', to prevent duplicates" && &lhs < &rhs);

					// If we have less than 'N' pairs, add it
					if (m_count != m_pairs.size())
					{
						m_pairs[m_count++] = { &lhs, &rhs, distance };
						std::push_heap(m_pairs.data(), m_pairs.data() + m_count, MaxHeap);
					}

					// Otherwise, if this pair is closer than the least closest, replace it
					else if (distance < m_pairs[0].distance)
					{
						std::pop_heap(m_pairs.data(), m_pairs.data() + m_count, MaxHeap);
						m_pairs[m_count - 1] = { &lhs, &rhs, distance };
						std::push_heap(m_pairs.data(), m_pairs.data() + m_count, MaxHeap);

						// Restrict the search radius to the distance of the least closest pair found so far
						m_search_radius = m_pairs[0].distance;
					}
				}
			};

			// Recursive search
			Finder finder(pairs_out, radius, measure_distance, get_threshold);
			finder.Run(vptree);

			// Sort the pairs into increasing separation order
			std::sort(pairs_out.begin(), pairs_out.begin() + finder.m_count, [](Pair const& lhs, Pair const& rhs)
			{
				return lhs.distance < rhs.distance;
			});
			return finder.m_count;
		}
	};
}

#if PR_UNITTESTS
#include "pr/math/math.h"
#include "pr/common/unittests.h"
namespace pr::vp_tree::tests
{
	PRUnitTestClass(VPTreeTests)
	{
		using Pt = v3; // sorting threshold in 'z'
		using VPTree = vp_tree::VPTree<Pt, float>;
		std::default_random_engine m_rng;

		TestClass_VPTreeTests()
			:m_rng(1u)
		{}
		void GeneratePoints(std::vector<Pt>& points)
		{
			for (auto& p : points)
				p = Pt{ Random<v2>(m_rng, v2::Zero(), 10.0f), 0 };
		}
		void CheckResults(std::vector<Pt> const& points, v2 centre, float radius, std::set<Pt> const& results)
		{
			// Check all found points are within 'radius' of 'centre' and all not-found points are outside 'radius' of 'centre'
			for (auto const& p : points)
			{
				auto sep = Length(p.xy - centre);
				if (results.find(p) != std::end(results))
					PR_EXPECT(sep <= radius + math::tiny<float>);
				else
					PR_EXPECT(sep >= radius - math::tiny<float>);
			}
		}
		void CheckNearest(std::vector<Pt> const& points, v2 centre, float radius, std::vector<VPTree::Neighbour> const& nearest)
		{
			constexpr auto Contains = [](Pt const& p, std::span<VPTree::Neighbour const> nearest)
			{
				return std::find_if(begin(nearest), end(nearest), [&](VPTree::Neighbour const& n) { return n.item == &p; }) != end(nearest);
			};

			// Check that the nearest neighbours are sorted by increasing distance
			for (size_t i = 1; i < nearest.size(); ++i)
				PR_EXPECT(nearest[i - 1].distance <= nearest[i].distance);

			// Check that all found nearest neighbours are within 'radius' of 'centre'
			for (auto& n : nearest)
				PR_EXPECT(Length(n.item->xy - centre) <= radius);

			// Check that all other points are further away than the furthest nearest neighbour found
			auto limit = std::min(radius, nearest.empty() ? radius : nearest.back().distance);
			for (auto& point : points)
			{
				if (Contains(point, nearest))
					continue;

				auto dist = Length(point.xy - centre);
				PR_EXPECT(dist >= limit);
			}
		}
		void CheckPairs(std::vector<Pt> const& points, float max_separation, std::vector<VPTree::Pair> const& pairs)
		{
			constexpr auto Contains = [](VPTree::Pair const& pair, std::span<VPTree::Pair const> pairs)
			{
				return std::find_if(begin(pairs), end(pairs), [&](VPTree::Pair const& p) { return p == pair; }) != end(pairs);
			};

			// Check that the pairs are sorted by increasing separation
			for (size_t i = 1; i < pairs.size(); ++i)
				PR_EXPECT(pairs[i - 1].distance <= pairs[i].distance);

			// Check that all pairs are closer than 'max_separation'within the search radius
			for (auto& p : pairs)
				PR_EXPECT(p.distance <= max_separation);

			// Check that all other point pairs are further apart than the furthest pair found
			auto limit = pairs.empty() ? max_separation : pairs.back().distance;
			for (size_t i = 0; i < points.size(); ++i)
			{
				for (size_t j = i + 1; j < points.size(); ++j)
				{
					Pt const& a = points[i];
					Pt const& b = points[j];

					VPTree::Pair pair{ &a, &b, Length(a.xy - b.xy) };
					if (Contains(pair, pairs))
						continue;

					PR_EXPECT(pair.distance >= limit);
				}
			}
		}

		PRUnitTestMethod(NormalCase)
		{
			using namespace pr::vp_tree;

			std::vector<Pt> points(100);
			GeneratePoints(points);

			constexpr v2 search_centre = {2.5f, -1.2f};
			constexpr float search_radius = 3.0f;
			constexpr float max_separation = 5.0f;

			// Build the tree in-place in 'points'
			{
				VPTree::Build(points,
					[this](std::span<Pt> items)
					{
						std::uniform_int_distribution<size_t> dist(0, items.size() - 1);
						return dist(m_rng);
					},
					[](Pt const& a, Pt const& b)
					{
						return Length(a.xy - b.xy);
					},
					[](Pt& item, float dist)
					{
						item.z = dist;
					}
				);

				// Draw the tree
				#if 0
				{
					using namespace rdr12::ldraw;

					Builder builder;
					auto& ldr_points = builder.Point("Point", 0xFF0000FF).size(10.0f);
					for (auto const& p : points)
						ldr_points.pt(v4{ p.xy, 0, 1 }, 0xFF0000FF);

					for (auto const& p : points)
					{
						if (!FEql(p.w, 3.f)) continue;
						builder.Circle("search", 0x2000FF00).solid().radius(p.z).pos(v4{ p.xy, p.w * 3, 1 });
					}

					builder.Save("E:/Dump/vptree.ldr", ESaveFlags::Pretty);
				}
				#endif
			}

			// Test search
			{
				std::set<Pt> results;
				VPTree::Find(points, search_centre, search_radius,
					[](Pt const& item, v2 const& search_point)
					{
						return Length(item.xy - search_point);
					},
					[](Pt const& item)
					{
						return item.z;
					},
					[&](Pt const& item, float dist)
					{
						results.insert(item);
						PR_EXPECT(FEql(Length(item.xy - search_centre), dist));
						PR_EXPECT(dist <= search_radius);
					}
				);
				CheckResults(points, search_centre, search_radius, results);

				// Draw results
				#if 0
				{
					using namespace rdr12::ldraw;

					Builder builder;
					builder.Circle("search", 0x8000FF00).radius(search_radius).pos(v4{ search_centre, 0, 1 });

					auto& ldr_points = builder.Point("Points", 0xFF0000FF).size(10.0f);
					for (auto const& p : points)
					{
						auto is_found = results.find(p) != std::end(results);
						ldr_points.pt(v4(p.xy, 0, 1), is_found ? 0xFFFF0000 : 0xFF0000FF);
					}

					builder.Save("E:/Dump/vptree.ldr", ESaveFlags::Pretty);
				}
				#endif
			}

			// Find the nearest N neighbours
			{
				std::vector<VPTree::Neighbour> nearest(5);
				nearest.resize(VPTree::FindNearest(points, search_centre, search_radius, std::span(nearest),
					[](Pt const& item, v2 const& search_point)
					{
						return Length(item.xy - search_point);
					},
					[](Pt const& item)
					{
						return item.z;
					}
				));

				CheckNearest(points, search_centre, search_radius, nearest);
			}

			// Find the closest N pairs
			{
				std::vector<VPTree::Pair> pairs(5);
				pairs.resize(VPTree::Closest(points, max_separation, std::span(pairs),
					[](Pt const& a, Pt const& b)
					{
						return Length(a.xy - b.xy);
					},
					[](Pt const& item)
					{
						return item.z;
					}
				));
				
				CheckPairs(points, max_separation, pairs);
			}
		}
		PRUnitTestMethod(Robustness)
		{
			std::vector<Pt> points;
			std::set<Pt> results;
			std::vector<VPTree::Neighbour> nearest;
			std::vector<VPTree::Pair> pairs;

			for (int i = 0; i < 100; ++i)
			{
				points.resize(100);
				GeneratePoints(points);

				results.clear();
				nearest.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));
				pairs.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));

				v2 const search_centre = Random<v2>(m_rng, v2::Zero(), 7.0f);
				float const search_radius = std::uniform_real_distribution<float>(0.f, 5.0f)(m_rng);

				VPTree::Build(points,
					[this](std::span<Pt> items)
					{
						std::uniform_int_distribution<size_t> dist(0, items.size() - 1);
						return dist(m_rng);
					},
					[](Pt const& a, Pt const& b)
					{
						return Length(a.xy - b.xy);
					},
					[](Pt& item, float dist)
					{
						item.z = dist;
					}
				);

				VPTree::Find(points, search_centre, search_radius,
					[](Pt const& item, v2 const& search_point)
					{
						return Length(item.xy - search_point);
					},
					[](Pt const& item)
					{
						return item.z;
					},
					[&](Pt const& item, float dist)
					{
						results.insert(item);
						PR_EXPECT(FEql(Length(item.xy - search_centre), dist));
						PR_EXPECT(dist <= search_radius);
					}
				);

				CheckResults(points, search_centre, search_radius, results);

				nearest.resize(VPTree::FindNearest(points, search_centre, search_radius, std::span(nearest),
					[](Pt const& item, v2 const& search_point)
					{
						return Length(item.xy - search_point);
					},
					[](Pt const& item)
					{
						return item.z;
					}
				));

				CheckNearest(points, search_centre, search_radius, nearest);

				pairs.resize(VPTree::Closest(points, search_radius, std::span(pairs),
					[](Pt const& a, Pt const& b)
					{
						return Length(a.xy - b.xy);
					},
					[](Pt const& item)
					{
						return item.z;
					}
				));

				CheckPairs(points, search_radius, pairs);
			}
		}
		PRUnitTestMethod(Degenerates)
		{
			std::vector<Pt> points;
			std::set<Pt> results;
			std::vector<VPTree::Neighbour> nearest;
			std::vector<VPTree::Pair> pairs;

			for (int i = 0; ; ++i)
			{
				points.resize(100);

				// Degenerate cases
				bool done = false;
				switch (i)
				{
					case 0: // All points the same
					{
						for (auto& p : points)
							p = Pt{ v2::Zero(), 0 };
						break;
					}
					case 1: // Points on a line
					{
						GeneratePoints(points);
						for (auto& p : points)
							p.y = 0;

						break;
					}
					case 2: // Points on a circle
					{
						GeneratePoints(points);
						for (auto& p : points)
						{
							auto dir = Normalise(p.xy);
							p.x = dir.x * 5.0f;
							p.y = dir.y * 5.0f;
						}
						break;
					}
					default:
					{
						done = true;
						break;
					}
				}
				if (done) break;

				results.clear();
				nearest.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));
				pairs.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));

				v2 const search_centre = Random<v2>(m_rng, v2::Zero(), 7.0f);
				float const search_radius = std::uniform_real_distribution<float>(0.f, 5.0f)(m_rng);

				VPTree::Build(points,
					[this](std::span<Pt> items)
					{
						std::uniform_int_distribution<size_t> dist(0, items.size() - 1);
						return dist(m_rng);
					},
					[](Pt const& a, Pt const& b)
					{
						return Length(a.xy - b.xy);
					},
					[](Pt& item, float dist)
					{
						item.z = dist;
					}
				);

				VPTree::Find(points, search_centre, search_radius,
					[](Pt const& item, v2 const& search_point)
					{
						return Length(item.xy - search_point);
					},
					[](Pt const& item)
					{
						return item.z;
					},
					[&](Pt const& item, float dist)
					{
						results.insert(item);
						PR_EXPECT(FEql(Length(item.xy - search_centre), dist));
						PR_EXPECT(dist <= search_radius);
					}
				);

				CheckResults(points, search_centre, search_radius, results);

				nearest.resize(VPTree::FindNearest(points, search_centre, search_radius, std::span(nearest),
					[](Pt const& item, v2 const& search_point)
					{
						return Length(item.xy - search_point);
					},
					[](Pt const& item)
					{
						return item.z;
					}
				));

				CheckNearest(points, search_centre, search_radius, nearest);

				pairs.resize(VPTree::Closest(points, search_radius, std::span(pairs),
					[](Pt const& a, Pt const& b)
					{
						return Length(a.xy - b.xy);
					},
					[](Pt const& item)
					{
						return item.z;
					}
				));

				CheckPairs(points, search_radius, pairs);
			}
		}
	};
}
#endif
