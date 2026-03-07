//*****************************************
// KD Tree
//  Copyright (c) March 2005 Paul Ryland
//*****************************************
// Updated 2023
#pragma once
#include <span>
#include <concepts>
#include <algorithm>
#include <limits>
#include <cmath>
#include <cassert>

namespace pr::kdtree
{
	// Concept for 'int GetAxis(Item const& item)' function
	template <typename T, typename Item> concept GetAxisFunc = requires(T t)
	{
		{ t(std::declval<Item const&>()) } -> std::convertible_to<int>;
	};

	// Concept for a 'void SetAxis(Item& item, int axis)' function
	template <typename T, typename Item> concept SetAxisFunc = requires(T t)
	{
		{ t(std::declval<Item&>(), std::declval<int>()) };
	};

	// Concept for a 'S GetValue(Item const& item, int axis)' function
	template <typename T, typename Item, typename S> concept GetValueFunc = std::floating_point<S> && requires(T t)
	{
		{ t(std::declval<Item const&>(), std::declval<int>()) } -> std::convertible_to<S>;
	};

	// Concept for a 'void Found(Item const&, S dist_sq)' function
	template <typename T, typename Item, typename S> concept FoundFunc = std::floating_point<S> && requires(T t)
	{
		{ t(std::declval<Item const&>(), std::declval<S>()) };
	};

	// Strategies for selecting the axis to split on
	enum class EStrategy
	{
		AxisByLevel,
		LongestAxis,
	};

	// KDTree methods
	template <int Dim, typename Item, std::floating_point S = float, EStrategy SelectAxis = EStrategy::LongestAxis>
	struct KDTree
	{
		// Notes:
		//  - A KD tree can be built on const items if 'KDTree<N, Item const>' is used.

		static constexpr int Dimensions = Dim;
		using SearchCentre = S[Dim];

		struct Neighbour
		{
			Item const* item;
			S squared_distance;
		};
		struct Pair
		{
			Item const* item0;
			Item const* item1;
			S squared_distance;

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

		// Build a KD tree from 'items'
		template <GetValueFunc<Item, S> GetValue, SetAxisFunc<Item> SetAxis>
		static void Build(std::span<Item> items, GetValue get_value, SetAxis set_axis)
		{
			struct Builder
			{
				GetValue m_get_value;
				SetAxis m_set_axis;
				Builder(GetValue get_value, SetAxis set_axis)
					: m_get_value(get_value)
					, m_set_axis(set_axis)
				{
				}

				// Sort values based on the median value of the longest axis
				void Run(std::span<Item> items, int level)
				{
					if (items.size() <= 1)
						return;

					// Set the axis to split on.
					int split_axis = 0;
					if constexpr (SelectAxis == EStrategy::AxisByLevel)
						split_axis = AxisByLevel(items, level);
					if constexpr (SelectAxis == EStrategy::LongestAxis)
						split_axis = LongestAxis(items, level);

					// Split the range. This ensures all values < mid have lesser value on 'split_axis' than all values > mid.
					auto mid = items.size() / 2;
					std::nth_element(items.data(), items.data() + mid, items.data() + items.size(), [&](auto const& lhs, auto const& rhs)
					{
						return m_get_value(lhs, split_axis) < m_get_value(rhs, split_axis);
					});

					// Record the split axis
					m_set_axis(items[mid], split_axis);

					// Construct recursively
					Run(items.subspan(0, mid), level + 1);
					Run(items.subspan(mid + 1), level + 1);
				};

				// Select the axis simply based on level
				int AxisByLevel(std::span<Item const>, int level)
				{
					return level % Dim;
				}

				// Find the axis with the greatest range
				int LongestAxis(std::span<Item const> items, int)
				{
					auto ptr = items.data();
					auto end = ptr + items.size();

					// Find the bounds over all axes
					S lower[Dim], upper[Dim];
					for (int a = 0; a != Dim; ++a)
					{
						lower[a] = m_get_value(*ptr, a);
						upper[a] = lower[a];
					}
					for (++ptr; ptr != end; ++ptr)
					{
						for (int a = 0; a != Dim; ++a)
						{
							auto value = m_get_value(*ptr, a);
							if (value < lower[a]) lower[a] = value;
							if (value > upper[a]) upper[a] = value;
						}
					}

					// Select the axis with the greatest range
					for (int a = 0; a != Dim; ++a)
						upper[a] -= lower[a];

					int largest = 0;
					for (int a = 1; a != Dim; ++a)
						largest = upper[a] > upper[largest] ? a : largest;

					return largest;
				}
			};

			// Recursive build
			Builder builder(get_value, set_axis);
			builder.Run(items, 0);
		}

		// Search a KD tree for all items within 'radius' of 'centre'.
		template <GetValueFunc<Item, S> GetValue, GetAxisFunc<Item> GetAxis, FoundFunc<Item, S> Found>
		static void Find(std::span<Item const> kdtree, SearchCentre const& centre, S radius, GetValue get_value, GetAxis get_axis, Found found)
		{
			struct Finder
			{
				SearchCentre const& m_centre;
				S m_radius;
				GetValue m_get_value;
				GetAxis m_get_axis;
				Found m_found;

				Finder(SearchCentre const& centre, S radius, GetValue get_value, GetAxis get_axis, Found found)
					: m_centre(centre)
					, m_radius(radius)
					, m_get_value(get_value)
					, m_get_axis(get_axis)
					, m_found(found)
				{}

				// Find nodes within a spherical search region
				void Run(std::span<Item const> items)
				{
					if (items.empty())
						return;

					auto mid = items.size() / 2;
					if (auto dist_sq = MeasureDistanceSq(items[mid]); dist_sq <= m_radius * m_radius)
						m_found(items[mid], dist_sq);

					// Bottom of the tree? Time to leave
					if (items.size() <= 1)
						return;

					// Get the axis to search on
					auto split_axis = m_get_axis(items[mid]);
					auto split_value = m_get_value(items[mid], split_axis);

					// If 'centre[axis] - radius <= split_value' then we need to search the left sub tree
					if (m_centre[split_axis] - m_radius <= split_value)
						Run(items.subspan(0, mid));

					// If 'centre[axis] + radius >= split_value' then we need to search the right sub tree
					if (m_centre[split_axis] + m_radius >= split_value)
						Run(items.subspan(mid + 1));
				}

				// Return the squared distance from the centre to 'item'
				S MeasureDistanceSq(Item const& item) const
				{
					auto dist_sq = S{};
					for (int a = 0; a != Dim; ++a)
					{
						auto dist = m_get_value(item, a) - m_centre[a];
						dist_sq += dist * dist;
					}
					return dist_sq;
				}
			};

			// Recursive search
			Finder finder(centre, radius, get_value, get_axis, found);
			finder.Run(kdtree);
		}

		// Search a KD tree for the 'N' nearest neighbours within 'radius' of 'centre'
		// Neighbours are returned in order of increasing distance from the search point.
		// Return value is the number of neighbours found (may be less than nearest_out.size()).
		template <GetValueFunc<Item, S> GetValue, GetAxisFunc<Item> GetAxis>
		static size_t FindNearest(std::span<Item const> kdtree, SearchCentre const& centre, S radius, std::span<Neighbour> nearest_out, GetValue get_value, GetAxis get_axis)
		{
			struct Finder
			{
				std::span<Neighbour> m_nearest;
				SearchCentre const& m_centre;
				S m_radius;
				GetValue m_get_value;
				GetAxis m_get_axis;
				size_t m_count;

				Finder(std::span<Neighbour> nearest, SearchCentre const& centre, S radius, GetValue get_value, GetAxis get_axis)
					: m_nearest(nearest)
					, m_centre(centre)
					, m_radius(radius)
					, m_get_value(get_value)
					, m_get_axis(get_axis)
					, m_count(0)
				{
					assert(!m_nearest.empty());
					for (auto& n : m_nearest)
						n = { nullptr, std::numeric_limits<S>::infinity() };
				}
				
				// Find nodes within a spherical search region
				void Run(std::span<Item const> items)
				{
					if (items.empty())
						return;

					auto mid = items.size() / 2;
					if (auto dist_sq = MeasureDistanceSq(items[mid]); dist_sq <= m_radius * m_radius)
						TrackNearest(items[mid], dist_sq);

					// Bottom of the tree? Time to leave
					if (items.size() <= 1)
						return;

					// Get the axis to search on
					auto split_axis = m_get_axis(items[mid]);
					auto split_value = m_get_value(items[mid], split_axis);

					// If 'centre[axis] - radius <= split_value' then we need to search the left sub tree
					if (m_centre[split_axis] - m_radius <= split_value)
						Run(items.subspan(0, mid));

					// If 'centre[axis] + radius >= split_value' then we need to search the right sub tree
					if (m_centre[split_axis] + m_radius >= split_value)
						Run(items.subspan(mid + 1));
				}

				// Track the distance at which there are 'N' closer items
				void TrackNearest(Item const& item, S dist_sq)
				{
					// Heap uses > for highest priority. For this case, largest distance has highest priority
					constexpr auto MaxHeap = [](Neighbour const& lhs, Neighbour const& rhs) { return lhs.squared_distance < rhs.squared_distance; };

					// If we have less than 'N' nearest, add it
					if (m_count != m_nearest.size())
					{
						m_nearest[m_count++] = { &item, dist_sq };
						std::push_heap(m_nearest.data(), m_nearest.data() + m_count, MaxHeap);
					}

					// Otherwise, if it is closer than the furthest, replace it
					else if (dist_sq < m_nearest[0].squared_distance)
					{
						std::pop_heap(m_nearest.data(), m_nearest.data() + m_count, MaxHeap);
						m_nearest[m_count - 1] = { &item, dist_sq };
						std::push_heap(m_nearest.data(), m_nearest.data() + m_count, MaxHeap);

						// Restrict the search radius to the distance of the least closest neighbour found so far
						m_radius = std::sqrt(m_nearest[0].squared_distance);
					}
				}

				// Return the squared distance from the centre to 'item'
				S MeasureDistanceSq(Item const& item) const
				{
					S dist_sq = {};
					for (int a = 0; a != Dim; ++a)
					{
						auto dist = m_get_value(item, a) - m_centre[a];
						dist_sq += dist * dist;
					}
					return dist_sq;
				}
			};

			// Recursive search
			Finder finder(nearest_out, centre, radius, get_value, get_axis);
			finder.Run(kdtree);

			// Sort the nearest neighbours into increasing distance order
			std::sort(nearest_out.begin(), nearest_out.begin() + finder.m_count, [](Neighbour const& lhs, Neighbour const& rhs)
			{
				return lhs.squared_distance < rhs.squared_distance;
			});
			return finder.m_count;
		}

		// Find pairs of items that are the nearest to each other
		// Pairs are returned in order of increasing separation.
		// Return value is the number of pairs found (may be less than pairs_out.size()).
		template <GetValueFunc<Item, S> GetValue, GetAxisFunc<Item> GetAxis>
		static size_t Closest(std::span<Item const> kdtree, S radius, std::span<Pair> pairs_out, GetValue get_value, GetAxis get_axis)
		{
			struct Finder
			{
				std::span<Pair> m_pairs;
				S m_radius;
				GetValue m_get_value;
				GetAxis m_get_axis;
				size_t m_count;

				Finder(std::span<Pair> pairs, S radius, GetValue get_value, GetAxis get_axis)
					: m_pairs(pairs)
					, m_radius(radius)
					, m_get_value(get_value)
					, m_get_axis(get_axis)
					, m_count(0)
				{
					assert(!m_pairs.empty());
					for (auto& p : m_pairs)
						p = { nullptr, nullptr, std::numeric_limits<S>::infinity() };
				}

				// Find nodes within the minimum separation
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

					auto mid = items.size() / 2;
					if (auto sep_sq = SeparationSq(target, items[mid]); sep_sq <= m_radius * m_radius && &target < &items[mid])
						TrackClosest(target, items[mid], sep_sq);

					// Bottom of the tree? Time to leave
					if (items.size() <= 1)
						return;

					// Get the axis to search on
					auto split_axis = m_get_axis(items[mid]);
					auto split_value = m_get_value(items[mid], split_axis);
					auto search_value = m_get_value(target, split_axis);

					// If 'centre[axis] - radius <= split_value' then we need to search the left sub tree
					if (search_value - m_radius <= split_value)
						FindClosest(target, items.subspan(0, mid));

					// If 'centre[axis] + radius >= split_value' then we need to search the right sub tree
					if (search_value + m_radius >= split_value)
						FindClosest(target, items.subspan(mid + 1));
				}

				// Track the distance at which there are 'N' closer items
				void TrackClosest(Item const& lhs, Item const& rhs, S sep_sq)
				{
					// Heap uses > for highest priority. For this case, largest separation has highest priority
					constexpr auto MaxHeap = [](Pair const& lhs, Pair const& rhs) { return lhs.squared_distance < rhs.squared_distance; };
					assert("Should only be considering pairs when '&lhs < &rhs', to prevent duplicates" && &lhs < &rhs);

					// If we have less than 'N' nearest, add it
					if (m_count != m_pairs.size())
					{
						m_pairs[m_count++] = { &lhs, &rhs, sep_sq };
						std::push_heap(m_pairs.data(), m_pairs.data() + m_count, MaxHeap);
					}

					// Otherwise, if this pair is closer than the least closest, replace it
					else if (sep_sq < m_pairs[0].squared_distance)
					{
						std::pop_heap(m_pairs.data(), m_pairs.data() + m_count, MaxHeap);
						m_pairs[m_count - 1] = { &lhs, &rhs, sep_sq};
						std::push_heap(m_pairs.data(), m_pairs.data() + m_count, MaxHeap);

						// Restrict the search radius to the distance of the least closest pair found so far
						m_radius = std::sqrt(m_pairs[0].squared_distance);
					}
				}

				// Return the squared distance from the centre to 'item'
				S SeparationSq(Item const& lhs, Item const& rhs) const
				{
					S sep_sq = {};
					for (int a = 0; a != Dim; ++a)
					{
						auto sep = m_get_value(lhs, a) - m_get_value(rhs, a);
						sep_sq += sep * sep;
					}
					return sep_sq;
				}
			};

			// Recursive search
			Finder finder(pairs_out, radius, get_value, get_axis);
			finder.Run(kdtree);

			// Sort the pairs into increasing separation order
			std::sort(pairs_out.begin(), pairs_out.begin() + finder.m_count, [](Pair const& lhs, Pair const& rhs)
			{
				return lhs.squared_distance < rhs.squared_distance;
			});
			return finder.m_count;
		}

		#if 0 // Not yet working
		// Find pairs of items that are the farthest from each other
		template <GetValueFunc<S, Item> GetValue, GetAxisFunc<Item> GetAxis>
		static void Farthest(std::span<Item const> items, std::span<Pair> pairs_out, GetValue get_value, GetAxis get_axis)
		{
			struct L
			{
				struct Bound
				{
					S lower, upper;
					S size() const { return upper - lower; }
				};

				std::span<Pair> m_pairs;
				GetValue m_get_value;
				GetAxis m_get_axis;
				Bound m_bounds[Dim];
				int m_count;

				// Square a value
				S Sqr(S s) const
				{
					return s * s;
				}

				// Returns the lower bound on distance to the 'N'th farthest
				S LeastFarthestSq() const
				{
					return m_count == std::ssize(m_pairs) ? m_pairs[m_count - 1].dist_sq : 0;
				}

				// Returns the squared diameter of the bounds
				S BoundsSizeSq() const
				{
					S size_sq = {};
					for (int a = 0; a != Dim; ++a)
						size_sq += Sqr(m_bounds[a].size());

					return size_sq;
				}

				// Track the distance at which there are 'N' most separated items
				void TrackFarthest(Item const& lhs, Item const& rhs)
				{
					// Find the squared distance from the search point to 'item'
					S dist_sq = {};
					for (int a = 0; a != Dim; ++a)
						dist_sq += Sqr(m_get_value(lhs, a) - m_get_value(rhs, a));

					// If we have less than 'N' fartherest, add it
					if (m_count != std::ssize(m_pairs))
					{
						m_pairs[m_count++] = { &lhs, &rhs, dist_sq };
					}

					// Otherwise, if it is further than the closest, replace it
					else if (dist_sq > m_pairs[m_count - 1].dist_sq)
					{
						m_pairs[m_count - 1] = { &lhs, &rhs, dist_sq };
						std::sort(m_pairs.data(), m_pairs.data() + m_count, [](auto const& lhs, auto const& rhs) { return lhs.dist_sq > rhs.dist_sq; });
					}
				}

				// Find the farthest items to 'target'
				void FindFarthest(Item const& target, Item const* first, Item const* last)
				{
					if (first == last)
						return;

					auto split_point = first + (last - first) / 2;

					// Only compare 'target' with items after 'target' in the collection
					if (&target < split_point)
						TrackFarthest(target, *split_point);

					// Bottom of the tree? Time to leave
					if (last - first <= 1)
						return;

					// Get the axis to search on
					auto split_axis = m_get_axis(*split_point);
					auto split_value = m_get_value(*split_point, split_axis);
					//auto search_value = m_get_value(target, split_axis);
					//auto bounds_sq = BoundsSizeSq() - Sqr(m_bounds[split_axis].size());

					// Search the left sub tree
					if (&target < split_point)
					{
						//auto distance_sq = Sqr(std::max(split_value, search_value) - m_bounds[split_axis].lower);
						//if (bounds_sq + distance_sq > LeastFarthestSq())
						{
							std::swap(m_bounds[split_axis].upper, split_value);
							FindFarthest(target, first, split_point);
							std::swap(m_bounds[split_axis].upper, split_value);
						}
					}

					// Search the right sub tree
					if (&target < last)
					{
						//auto distance_sq = Sqr(m_bounds[split_axis].upper - std::min(split_value, search_value));
						//if (bounds_sq + distance_sq > LeastFarthestSq())
						{
							std::swap(m_bounds[split_axis].lower, split_value);
							FindFarthest(target, split_point + 1, last);
							std::swap(m_bounds[split_axis].lower, split_value);
						}
					}
				}

				// Find the data bounds
				void CalcDataBounds(Item const* first, Item const* last)
				{
					for (int a = 0; a != Dim; ++a)
					{
						m_bounds[a].lower = std::numeric_limits<S>::max();
						m_bounds[a].upper = std::numeric_limits<S>::lowest();
					}
					for (auto target = first; target != last; ++target)
					{
						for (int a = 0; a != Dim; ++a)
						{
							auto value = m_get_value(*target, a);
							if (value < m_bounds[a].lower) m_bounds[a].lower = value;
							if (value > m_bounds[a].upper) m_bounds[a].upper = value;
						}
					}
				}

				// Find nodes within the minimum separation
				void DoFind(Item const* first, Item const* last)
				{
					CalcDataBounds(first, last);

					// The brute force method would be to compare the item with all other items (O(N^2)).
					// However, we can use the tree to reduce the number of comparisons by not searching trees
					// that can't be further from the target than the least-farthest pair.
					for (auto target = first; target != last; ++target)
						FindFarthest(*target, first, last);
				}
			};

			// Recursive search
			L x = { pairs_out, get_value, get_axis, {}, 0 };
			x.DoFind(items.data(), items.data() + items.size());
		}
		#endif
	};
}

#if PR_UNITTESTS
#include "pr/math/math.h"
#include "pr/common/unittests.h"
#include "pr/view3d-12/ldraw/ldraw_builder.h"
namespace pr::container
{
	PRUnitTestClass(KDTreeTests)
	{
		using Pt = v3; // sorting axis in 'z'
		using KDTree = kdtree::KDTree<2, Pt, float>;
		std::default_random_engine m_rng;

		TestClass_KDTreeTests()
			:m_rng(1u)
		{}
		void GenerateRandomPoints(std::vector<Pt>& points)
		{
			for (auto& p : points)
				p = Pt{ Random<v2>(m_rng, v2::Zero(), 10.0f), 0 };
		}
		void GenerateGridPoints(std::vector<Pt>& points)
		{
			// Create a regular grid of points
			for (auto& p : points)
			{
				auto i = &p - points.data();
				p = Pt{ s_cast<float>(i % 10), s_cast<float>(i / 10), 0 };
			}

			// Randomise the order of the points
			std::ranges::shuffle(points, m_rng);
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
		void CheckNearest(std::vector<Pt> const& points, v2 centre, float radius, std::vector<KDTree::Neighbour> const& nearest)
		{
			constexpr auto Contains = [](Pt const& p, std::span<KDTree::Neighbour const> nearest)
			{
				return std::find_if(begin(nearest), end(nearest), [&](KDTree::Neighbour const& n) { return n.item == &p; }) != end(nearest);
			};

			// Check that the nearest neighbours are sorted by increasing distance
			for (size_t i = 1; i < nearest.size(); ++i)
				PR_EXPECT(nearest[i - 1].squared_distance <= nearest[i].squared_distance);

			// Check that all found nearest neighbours are within 'radius' of 'centre'
			for (auto& n : nearest)
				PR_EXPECT(Length(n.item->xy - centre) <= radius);

			// Check that all other points are further away than the furthest nearest neighbour found
			auto limit = std::min(radius, nearest.empty() ? radius : std::sqrt(nearest.back().squared_distance));
			for (auto& point : points)
			{
				if (Contains(point, nearest))
					continue;

				auto dist = Length(point.xy - centre);
				PR_EXPECT(dist >= limit);
			}
		}
		void CheckPairs(std::vector<Pt> const& points, float max_separation, std::vector<KDTree::Pair> const& pairs)
		{
			constexpr auto Contains = [](KDTree::Pair const& pair, std::span<KDTree::Pair const> pairs)
			{
				return std::find_if(begin(pairs), end(pairs), [&](KDTree::Pair const& p) { return p == pair; }) != end(pairs);
			};

			// Check that the pairs are sorted by increasing separation
			for (size_t i = 1; i < pairs.size(); ++i)
				PR_EXPECT(pairs[i - 1].squared_distance <= pairs[i].squared_distance);

			// Check that all pairs are closer than 'max_separation'within the search radius
			for (auto& p : pairs)
				PR_EXPECT(std::sqrt(p.squared_distance) <= max_separation);

			// Check that all other point pairs are further apart than the furthest pair found
			auto limit = pairs.empty() ? max_separation : std::sqrt(pairs.back().squared_distance);
			for (size_t i = 0; i < points.size(); ++i)
			{
				for (size_t j = i + 1; j < points.size(); ++j)
				{
					Pt const& a = points[i];
					Pt const& b = points[j];

					KDTree::Pair pair{ &a, &b, LengthSq(a.xy - b.xy) };
					if (Contains(pair, pairs))
						continue;

					PR_EXPECT(std::sqrt(pair.squared_distance) >= limit);
				}
			}
		}

		PRUnitTestMethod(NormalCase)
		{
			using namespace pr::kdtree;
			
			std::vector<Pt> points(100);
			GenerateGridPoints(points);

			constexpr v2 search_centre = { 4.3f, 6.4f };
			constexpr float search_radius = 3.0f;
			constexpr float max_separation = 4.0f;

			// Build the tree in place
			{
				KDTree::Build(points,
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt& p, int a)
					{
						p.z = static_cast<float>(a);
					}
				);
			}

			// Test search
			{
				std::set<Pt> results;
				KDTree::Find(points, search_centre.arr, search_radius,
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					},
					[&](Pt const& p, float dist_sq)
					{
						results.insert(p);
						PR_EXPECT(Sqrt(dist_sq) < search_radius + math::tiny<float>);
						PR_EXPECT(Length(search_centre - p.xy) < search_radius + math::tiny<float>);
					}
				);

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

					builder.Save("E:/Dump/kdtree.ldr", ESaveFlags::Pretty);
				}
				#endif

				CheckResults(points, search_centre, search_radius, results);
			}

			// Find nearest 'N' neighbours
			{
				// Searching for 'results.size()' nearest should return the same set
				std::vector<KDTree::Neighbour> nearest(5);
				nearest.resize(KDTree::FindNearest(points, search_centre.arr, search_radius, std::span(nearest),
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					}
				));

				#if 1
				{
					using namespace rdr12::ldraw;

					Builder builder;
					builder
						.Point("Search", 0xFF00FF00).size(15).pt(v4::Origin()).pos(v4(search_centre, 0, 1))
						.Circle("search", 0x8000FF00).radius(search_radius);

					auto& ldr_points = builder.Point("Points").size(10.0f);
					for (auto const& p : points)
					{
						auto is_found = std::ranges::find_if(nearest, [&](auto const& x) { return x.item == &p; }) != std::end(nearest);
						ldr_points.pt(p.w1(), is_found ? 0xFFFF0000 : 0xFF0000FF);
					}

					builder.Save("E:/Dump/kdtree.ldr", ESaveFlags::Pretty);
				}
				#endif

				CheckNearest(points, search_centre, search_radius, nearest);
			}

			// Find the closest N pairs
			{
				std::vector<KDTree::Pair> pairs(5);
				pairs.resize(KDTree::Closest(points, max_separation, std::span(pairs),
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					}
				));
				
				CheckPairs(points, max_separation, pairs);

				#if	0
				{
					using namespace rdr12::ldraw;

					Builder builder;

					auto& ldr_points = builder.Point("Points").size(10.0f);
					for (auto const& p : points)
						ldr_points.pt(v4(p, 0.0f, 1), 0xFF0000FF);

					auto& ldr_lines = builder.Line("Closest", 0xFFFF0000);
					for (auto const& p : pairs)
					{
						auto p0 = v4(*p.item0, 0, 1);
						auto p1 = v4(*p.item1, 0, 1);

						ldr_lines.line(p0, p1);
						builder.Sphere("Found", 0x80FF0000).r(sqrt(p.dist_sq) * 0.5f).pos((p0 + p1) * 0.5f);
					}

					builder.Save("E:/Dump/kdtree.ldr", ESaveFlags::Pretty);
				}
				#endif
			}
		}
		PRUnitTestMethod(Robustness)
		{
			std::vector<Pt> points;
			std::set<Pt> results;
			std::vector<KDTree::Neighbour> nearest;
			std::vector<KDTree::Pair> pairs;

			for (int i = 0; i < 100; ++i)
			{
				points.resize(100);
				GenerateRandomPoints(points);

				results.clear();
				nearest.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));
				pairs.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));

				v2 const search_centre = Random<v2>(m_rng, v2::Zero(), 7.0f);
				float const search_radius = std::uniform_real_distribution<float>(0.f, 5.0f)(m_rng);

				KDTree::Build(points,
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt& p, int a)
					{
						p.z = static_cast<float>(a);
					}
				);

				KDTree::Find(points, search_centre.arr, search_radius,
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					},
					[&](Pt const& p, float dist_sq)
					{
						results.insert(p);
						PR_EXPECT(std::sqrt(dist_sq) < search_radius + math::tiny<float>);
						PR_EXPECT(Length(search_centre - p.xy) < search_radius + math::tiny<float>);
					}
				);

				CheckResults(points, search_centre, search_radius, results);

				nearest.resize(KDTree::FindNearest(points, search_centre.arr, search_radius, std::span(nearest),
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					}
				));

				CheckNearest(points, search_centre, search_radius, nearest);

				pairs.resize(KDTree::Closest(points, search_radius, std::span(pairs),
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					}
				));

				CheckPairs(points, search_radius, pairs);
			}
		}
		PRUnitTestMethod(Degenerates)
		{
			std::vector<Pt> points;
			std::set<Pt> results;
			std::vector<KDTree::Neighbour> nearest;
			std::vector<KDTree::Pair> pairs;

			for (int i = 0; i != 2; ++i)
			{
				points.resize(100);
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
						GenerateRandomPoints(points);
						for (auto& p : points)
							p.y = 0;

						break;
					}
					case 2: // Points on a circle
					{
						GenerateRandomPoints(points);
						for (auto& p : points)
						{
							auto dir = Normalise(p.xy);
							p.x = dir.x * 5.0f;
							p.y = dir.y * 5.0f;
						}
						break;
					}
				}

				results.clear();
				nearest.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));
				pairs.resize(std::uniform_int_distribution<size_t>(1, 20)(m_rng));

				v2 const search_centre = Random<v2>(m_rng, v2::Zero(), 7.0f);
				float const search_radius = std::uniform_real_distribution<float>(0.f, 5.0f)(m_rng);

				KDTree::Build(points,
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt& p, int a)
					{
						p.z = static_cast<float>(a);
					}
				);

				KDTree::Find(points, search_centre.arr, search_radius,
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					},
					[&](Pt const& p, float dist_sq)
					{
						results.insert(p);
						PR_EXPECT(Sqrt(dist_sq) < search_radius + math::tiny<float>);
						PR_EXPECT(Length(search_centre - p.xy) < search_radius + math::tiny<float>);
					}
				);

				CheckResults(points, search_centre, search_radius, results);

				nearest.resize(KDTree::FindNearest(points, search_centre.arr, search_radius, std::span(nearest),
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					}
				));

				CheckNearest(points, search_centre, search_radius, nearest);

				pairs.resize(KDTree::Closest(points, search_radius, std::span(pairs),
					[](Pt const& p, int a)
					{
						return p[a];
					},
					[](Pt const& p)
					{
						return static_cast<int>(p.z);
					}
				));

				CheckPairs(points, search_radius, pairs);
			}
		}
		PRUnitTestMethod(Farthest)
		{
			#if 0
			std::default_random_engine rng(1u);
			std::uniform_real_distribution dist_pos(0.0f, 1.0f);

			std::vector<v2> points(100);
			std::bitset<100> pivots;
			for (int i = 0; i != isize(points); ++i)
				points[i] = v2(dist_pos(rng), dist_pos(rng));

			// Build the tree
			KDTree::Build(points,
				[](v2 const& p, int a)
				{
					return p[a];
				},
				[&](v2& p, int a)
				{
					pivots[&p - points.data()] = a != 0;
				}
			);

			// Test search
			std::vector<KDTree::Pair> pairs(10);
			KDTree::Farthest(points, pairs,
				[](v2 const& p, int a)
				{
					return p[a];
				},
				[&](v2 const& p)
				{
					return int(pivots[&p - points.data()]);
				}
			);

			// Check pairs are ordered by decreasing separation
			for (int i = 1; i != isize(pairs); ++i)
			{
				PR_EXPECT(pairs[i - 1].dist_sq >= pairs[i + 0].dist_sq);
			}

			// Check no other pairs are further
			for (auto& objA : points)
			{
				for (auto& objB : points)
				{
					if (&objA == &objB)
						continue;

					auto dist_sq = LengthSq(objA - objB);
					KDTree::Pair pair{ &objA, &objB, dist_sq };

					// If the pair are closer, they should not be in the list
					if (dist_sq < pairs.back().dist_sq)
					{
						auto is_found = std::ranges::find(pairs, pair) != std::end(pairs);
						PR_EXPECT(!is_found);
					}

					// If the pair are further, they should be in the list
					if (dist_sq > pairs.back().dist_sq)
					{
						auto is_found = std::ranges::find(pairs, pair) != std::end(pairs);
						PR_EXPECT(is_found);
					}

					// Don't worry about equal, because the results may or may not include them
				}
			}

			// Show output
			#if	0
			{
				ldr::Builder builder;

				auto& ldr_points = builder.Point("Points", 0xFF0000FF).size(10.0f);
				for (auto const& p : points)
					ldr_points.pt(v4(p, 0.0f, 1));

				auto& ldr_lines = builder.Line("Farthest", 0xFFFFFFFF);
				for (auto const& p : pairs)
				{
					auto p0 = v4(*p.item0, 0, 1);
					auto p1 = v4(*p.item1, 0, 1);

					ldr_lines.line(p0, p1);
				}

				builder.Write("E:/Dump/kdtree.ldr");
			}
			#endif
			#endif
		}
	};
}
#endif
