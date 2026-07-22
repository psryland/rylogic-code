#pragma once
#include <concepts>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <cassert>

namespace pr::algorithm::astar
{
	// Notes:
	//  - A* is a graph search algorithm for finding the cheapest path from a node toward some goal.

	// Types to specialise the algorithm with
	template <typename T>
	concept ConfigType = requires(T t)
	{
		typename T::node_ref; // can be a pointer or index to a graph node.
		typename T::edge_ref; // can be a pointer or index to a graph edge.
		typename T::cost_type; // requires operator <.
		typename T::template vector_type<void>;
		typename T::template hashmap_type<void, void>;
		{ T::NoNode } -> std::convertible_to<typename T::node_ref>;
		{ T::NoEdge } -> std::convertible_to<typename T::edge_ref>;
		{ T::CostMax } -> std::convertible_to<typename T::cost_type>;
	};

	// Data associated with a node in the graph
	template <ConfigType C>
	struct NodeData
	{
		// The measure of how far the node is from the goal (aka. the heuristic value)
		typename C::cost_type heuristic_cost = C::CostMax;

		// True if the node is the goal
		bool is_goal = false;
	};

	// Data associated with an edge in the graph
	template <ConfigType C>
	struct EdgeData
	{
		// The node that the requested edge connects to
		typename C::node_ref target_node = C::NoNode;

		// The cost of including the edge in the path. (aka. the cost value)
		typename C::cost_type edge_cost = {};
	};

	// An item in the returned path
	template <ConfigType C>
	struct PathItem
	{
		// The node that is on the optimal path
		typename C::node_ref node = C::NoNode;

		// The edge to the next node in the path.
		typename C::edge_ref edge = C::NoEdge;

		// The cost to reach this node
		typename C::cost_type cost_to_node = {};
	};

	// Working data set for repeated calls
	template <ConfigType C>
	struct WorkingData
	{
		using node_ref = typename C::node_ref;
		using edge_ref = typename C::edge_ref;
		using cost_type = typename C::cost_type;

		// Search data for a node
		struct SearchData
		{
			// The node that this search data is for
			node_ref node = C::NoNode;

			// The predecessor node that leads to 'node'
			node_ref parent = C::NoNode;

			// The edge on 'parent' that leads to 'node'
			edge_ref parent_edge = C::NoEdge;

			// This is the sum of costs used to reach this node from 'start_node' (a.k.a "cost")
			cost_type cost_to_node = {};

			// This is an estimate of the "distance" (in cost units) that 'node' is from the goal (a.k.a "heuristic")
			cost_type heuristic_cost = C::CostMax;

			// True if this node is the 'goal'. Don't assume that 'heuristic_cost == 0' means at the goal. If all node report 'heuristic_cost' as 0, the algorithm becomes Dynstra's 
			bool is_goal = false;
		};

		// Used in the pending heap to determine the next highest priority node
		struct NodeCost
		{
			node_ref node;
			cost_type total_cost;
		};

		using heap_t = C::template vector_type<NodeCost>; // e.g. std::vector<NodeCost>;
		using lookup_t = C::template hashmap_type<node_ref, SearchData>; // e.g. std::unordered_map<node_ref, SearchData>;

		// Heap for searching nodes in priority order
		class PendingHeap
		{
			heap_t& m_heap;
			lookup_t& m_storage;

			static bool CompMinHeap(NodeCost const& lhs, NodeCost const& rhs)
			{
				return !(lhs.total_cost < rhs.total_cost); // min heap
			}

		public:

			PendingHeap(heap_t& heap, lookup_t& storage)
				: m_heap(heap)
				, m_storage(storage)
			{}
			SearchData const& top() const
			{
				return m_storage.at(m_heap.front().node);
			}
			bool empty() const
			{
				return m_heap.empty();
			}
			auto size() const
			{
				return m_heap.size();
			}
			void push(SearchData const& d, cost_type combined_cost)
			{
				m_heap.push_back(NodeCost{ .node = d.node, .total_cost = combined_cost });
				std::push_heap(m_heap.begin(), m_heap.end(), CompMinHeap);
				m_storage[d.node] = d;
			}
			void pop()
			{
				std::pop_heap(m_heap.begin(), m_heap.end(), CompMinHeap);
				m_heap.pop_back();
			}
			void erase(node_ref node)
			{
				auto comp = [node](auto const& lhs, auto const& rhs) -> bool
				{
					if (lhs.node == node) return false;
					if (rhs.node == node) return true;
					return CompMinHeap(lhs, rhs);
				};

				// Find 'node' in the heap. O(n)
				auto heap_iter = std::find_if(m_heap.begin(), m_heap.end(), [node](NodeCost const& n) { return n.node == node; });
				if (heap_iter != m_heap.end())
				{
					// Erase 'node' from the heap. O(log n)
					std::push_heap(m_heap.begin(), heap_iter + 1, comp);
					std::pop_heap(m_heap.begin(), m_heap.end(), comp);
					m_heap.pop_back();
				}
			}
		};

		heap_t heap;
		lookup_t storage;
		PendingHeap pending;

		WorkingData()
			: heap()
			, storage()
			, pending(heap, storage)
		{}

		// Reset the working data for another use.
		WorkingData& reuse()
		{
			heap.clear();
			storage.clear();
			return *this;
		}
	};

	// A type that reads and measures costs for edges in the graph
	template <typename T>
	concept AdapterType = ConfigType<T> && requires(T t, typename T::node_ref node, typename T::edge_ref edge, typename T::cost_type cost)
	{
		// The container type to return 'PathItem's in
		typename T::path_type;

		// Adapter should supply 'working_data'. Intended for reuse to reduce allocations.
		{ t.astar_working_data() } -> std::convertible_to<WorkingData<T>&>;

		// Return the index of the next edge. If 'edge' == NoEdge, return the first edge. If there are not more edges, return 'NoEdge'
		{ t.NextEdge(node, edge) } -> std::convertible_to<int>;

		// Return the edge data for the given edge index.
		{ t.ReadEdge(node, edge) } -> std::convertible_to<EdgeData<T>>;

		// Measure the heuristic for a node and test if it is the goal node
		{ t.MeasureNode(node) } -> std::convertible_to<NodeData<T>>;

		// Combines the 'cost_to_node' with the 'heuristic_cost' value (since they can be in different units)
		{ t.CombinedCost(cost, cost) } -> std::convertible_to<typename T::cost_type>;
	};

	// Perform an A* search from the given start node to the best node within the given cost threshold.
	// Returns an array of the nodes and edges to take to reach the best match for the heuristic.
	// If the heuristic function always returns 0, then it falls back to Dykstra's algorithm.
	// If true is returned with an empty path, then 'start_node' is the best match.
	template <AdapterType Adapter>
	[[nodiscard]] std::tuple<bool, typename Adapter::path_type> Search(Adapter& adapter, typename Adapter::node_ref start_node, typename Adapter::cost_type cost_threshold)
	{
		using node_ref = typename Adapter::node_ref;
		using cost_type = typename Adapter::cost_type;
		using path_type = typename Adapter::path_type;
		using SearchData = typename WorkingData<Adapter>::SearchData;
		using NodeData = NodeData<Adapter>;
		using EdgeData = EdgeData<Adapter>;
		using PathItem = PathItem<Adapter>;

		// Tracking of the traversal.
		// The 'pending' heap contains nodes still to be explored.
		// The 'completed' map are nodes that have been evaluated.
		auto& search_data = adapter.astar_working_data().reuse();
		auto& storage = search_data.storage;
		auto& pending = search_data.pending;

		// Add a node to the pending node's heap
		auto AddToPending = [&adapter, &pending](SearchData next, bool measure_heuristic_cost)
		{
			if (measure_heuristic_cost)
			{
				auto node_data = adapter.MeasureNode(next.node);
				next.heuristic_cost = node_data.heuristic_cost;
				next.is_goal = node_data.is_goal;
			}
			pending.push(next, adapter.CombinedCost(next.cost_to_node, next.heuristic_cost));
		};

		// Start with 'start_node'
		AddToPending(SearchData{ .node = start_node }, true);

		// Track the closest node to the goal
		auto best_match = pending.top();

		// Search out from 'start_node'
		for (; !pending.empty(); )
		{
			// The current node to explore
			SearchData current = pending.top();
			pending.pop();

			// If 'next' is too expensive, reject immediately
			if (current.cost_to_node > cost_threshold)
			{
				continue;
			}

			// If this is currently the best match, record it
			if (current.heuristic_cost < best_match.heuristic_cost)
			{
				best_match = current;
			}

			// If we've found the goal, stop searching
			if (current.is_goal)
			{
				// We need to wait for the goal node to be next on the pending heap rather
				// than doing a quick out as soon as we find it, because there may be other
				// paths to the goal node that are faster than the first one we find.
				best_match = current;
				break;
			}

			// Search the edges from the current node
			for (auto edge = Adapter::NoEdge; (edge = adapter.NextEdge(current.node, edge)) != Adapter::NoEdge;)
			{
				// An entry for the next node to search
				auto edge_data = adapter.ReadEdge(current.node, edge);
				SearchData next = {
					.node = edge_data.target_node,
					.parent = current.node,
					.parent_edge = edge,
					.cost_to_node = current.cost_to_node + edge_data.edge_cost,
				};

				// Bi-directional graphs include an edge back to the parent. This can be ignored.
				if (next.node == current.parent)
					continue;

				// Track the lowest cost to reach 'next.node'
				if (auto iter = storage.find(next.node); iter != storage.end())
				{
					// 'prev' is a node we've searched before, or is pending exploration
					auto& prev = iter->second;

					// If the cost is lower then we've previously seen, revive it for exploration.
					if (next.cost_to_node < prev.cost_to_node)
					{
						// Technically, we can known in advance if 'next.node' is in the pending list
						// but the lookup and update on each SearchData is probably more expensive than
						// this rarer case of trying to erase a node that isn't in the list.
						pending.erase(next.node);

						// Copy the heuristic value because that shouldn't have changed
						next.heuristic_cost = prev.heuristic_cost;
						next.is_goal = prev.is_goal;
						AddToPending(next, false);
					}

					// Done with this edge
					continue;

				}

				// This node hasn't been seen at all yet so add it to the open list
				AddToPending(next, true);
			}
		}

		// If 'start' is the best match, don't create a path.
		if (best_match.parent == Adapter::NoNode)
			return { best_match.is_goal, {} };

		// Construct the path from 'best_match' to 'start', then reverse the order.
		path_type path;
		path.push_back(PathItem{ .node = best_match.node, .edge = Adapter::NoEdge, .cost_to_node = best_match.cost_to_node });
		for (auto const* i = &best_match; i->parent != Adapter::NoNode;)
		{
			// Record the cumulative cost for the node being appended, not the child we walked from.
			auto parent = storage.find(i->parent);
			assert(parent != storage.end());

			path.push_back(PathItem{ .node = i->parent, .edge = i->parent_edge, .cost_to_node = parent->second.cost_to_node });

			// Find the parent of 'i'
			i = &parent->second;
		}
		std::reverse(begin(path), end(path));

		// Return whether the goal was reached, and the path to the best found option.
		return { best_match.is_goal, path };
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::astar::unittests
{
	PRUnitTestClass(AStarSearchTests)
	{
		// Regression config that matches ConfigType but deliberately has no NullNode alias.
		struct EdgeDataConfig
		{
			using node_ref = int;
			using edge_ref = int;
			using cost_type = float;
			template <typename T> using vector_type = std::vector<T>;
			template <typename K, typename V> using hashmap_type = std::unordered_map<K, V>;
			static constexpr node_ref NoNode = -1;
			static constexpr edge_ref NoEdge = -1;
			static constexpr cost_type CostMax = std::numeric_limits<cost_type>::max();
		};
		static_assert(astar::ConfigType<EdgeDataConfig>);
		static_assert(std::default_initializable<astar::EdgeData<EdgeDataConfig>>);

		struct Graph
		{
			struct Node { float x, y; };
			struct Edge { int a, b; };
			Node const m_nodes[11] = { {-10, -3}, {-7, +2}, {+5, +5}, {+6, -4}, {-1, -7}, {+1, -2}, {-3, +5}, {-4, -1}, {+2, +3}, {+6, +2}, {-5, -5} };
			Edge const m_edges[13] = { {0, 1}, {0, 10}, {1, 6}, {1, 7}, {7, 10}, {10, 4}, {10, 5}, {6, 2}, {8, 5}, {2, 8}, {5, 3}, {2, 9}, {9, 3} };
			std::vector<int> m_adj[_countof(m_nodes)]; // Adjacency data. One 'm_adj[i]' for each node, containing a list of the nodes it's connected to

			Graph()
			{
				// Create the adjacency data
				for (auto const& edge : m_edges)
				{
					m_adj[edge.a].push_back(edge.b);
					m_adj[edge.b].push_back(edge.a);
				}
			}
		};
		struct Adptr
		{
			// astar::ConfigType
			using node_ref = int;
			using edge_ref = int;
			using cost_type = float;
			template <typename T> using vector_type = std::vector<T>;
			template <typename K, typename V> using hashmap_type = std::unordered_map<K, V>;
			static constexpr node_ref NoNode = -1;
			static constexpr edge_ref NoEdge = -1;
			static constexpr cost_type CostMax = std::numeric_limits<cost_type>::max();

			// astar::AdapterType
			using WorkingData = astar::WorkingData<Adptr>;
			using path_type = std::vector<PathItem<Adptr>>;
			using NodeData = astar::NodeData<Adptr>;
			using EdgeData = astar::EdgeData<Adptr>;
			using GraphNode = typename Graph::Node;

			Graph& m_graph;
			GraphNode m_goal;
			WorkingData m_astar_working_data;

			Adptr(Graph& graph, GraphNode const& goal)
				: m_graph(graph)
				, m_goal(goal)
			{
			}

			// Reused data structures for repeated searches
			WorkingData& astar_working_data()
			{
				return m_astar_working_data;
			}

			// The distance between nodes
			static float Distance(Graph::Node const& a, Graph::Node const& b)
			{
				constexpr auto sqr = [](auto x) { return x * x; };
				return std::sqrtf(sqr(a.x - b.x) + sqr(a.y - b.y));
			};

			// Get a reference to the next edge of 'node'.
			edge_ref NextEdge(node_ref node, edge_ref edge) const
			{
				++edge;
				return edge >= 0 && edge < ssize(m_graph.m_adj[node]) ? edge : NoEdge;
			}

			// Return the edge data for the given edge index.
			EdgeData ReadEdge(node_ref node, edge_ref edge) const
			{
				assert(node >= 0 && node < _countof(m_graph.m_adj));
				assert(edge >= 0 && edge < static_cast<int>(ssize(m_graph.m_adj[node])));

				// The node that "nodes[node].edges[edge]" is connected to
				auto next_idx = m_graph.m_adj[node][edge];
				auto const& node0 = m_graph.m_nodes[node];
				auto const& node1 = m_graph.m_nodes[next_idx];

				return EdgeData{
					.target_node = next_idx,
					.edge_cost = Distance(node0, node1),
				};
			}

			// Read the heuristic value for the node (aka. cost-to-goal value)
			NodeData MeasureNode(node_ref node_idx) const
			{
				assert(node_idx >= 0 && node_idx < _countof(m_graph.m_adj));

				auto const& node0 = m_graph.m_nodes[node_idx];
				auto distance = Distance(node0, m_goal);
				return NodeData{
					.heuristic_cost = distance,
					.is_goal = distance < 0.001f,
				};
			}

			// Combines the 'cost_to_node' with the 'cost_to_goal' value (since they can be in different units)
			cost_type CombinedCost(cost_type cost_to_node, cost_type cost_to_goal) const
			{
				return cost_to_node + cost_to_goal;
			}
		};
		static_assert(astar::AdapterType<Adptr>);

		Graph graph;

		// Simple weighted chain used to pin the cumulative-cost reconstruction contract.
		struct WeightedGraph
		{
			struct Edge
			{
				int target;
				float cost;
			};

			std::vector<std::vector<Edge>> adj;

			// Initialise the bidirectional chain with fixed edge costs.
			WeightedGraph()
				: adj(4)
			{
				// Build a simple chain so the expected cumulative cost at each hop is unambiguous.
				adj[0].push_back({ 1, 2.0f });
				adj[1].push_back({ 0, 2.0f });
				adj[1].push_back({ 2, 3.0f });
				adj[2].push_back({ 1, 3.0f });
				adj[2].push_back({ 3, 5.0f });
				adj[3].push_back({ 2, 5.0f });
			}
		};

		// Adapter for the weighted chain regression case.
		struct WeightedAdptr
		{
			using node_ref = int;
			using edge_ref = int;
			using cost_type = float;
			template <typename T> using vector_type = std::vector<T>;
			template <typename K, typename V> using hashmap_type = std::unordered_map<K, V>;
			static constexpr node_ref NoNode = -1;
			static constexpr edge_ref NoEdge = -1;
			static constexpr cost_type CostMax = std::numeric_limits<cost_type>::max();

			using WorkingData = astar::WorkingData<WeightedAdptr>;
			using path_type = std::vector<PathItem<WeightedAdptr>>;
			using NodeData = astar::NodeData<WeightedAdptr>;
			using EdgeData = astar::EdgeData<WeightedAdptr>;

			WeightedGraph& graph;
			WorkingData working_data;
			int goal;

			// Keep references to the graph and goal node for repeated searches.
			WeightedAdptr(WeightedGraph& graph, int goal)
				: graph(graph)
				, goal(goal)
			{
			}

			// Reuse the working data so the regression test exercises the same code path as the real callers.
			WorkingData& astar_working_data()
			{
				return working_data;
			}

			// Advance to the next adjacency entry for the current node.
			edge_ref NextEdge(node_ref node, edge_ref edge) const
			{
				++edge;
				return edge >= 0 && edge < ssize(graph.adj[node]) ? edge : NoEdge;
			}

			// Map the stored edge index to the adjacent node and its cost.
			EdgeData ReadEdge(node_ref node, edge_ref edge) const
			{
				auto const& e = graph.adj[node][edge];
				return EdgeData{
					.target_node = e.target,
					.edge_cost = e.cost,
				};
			}

			// The regression test uses zero heuristic so the search behaves like Dijkstra's algorithm.
			NodeData MeasureNode(node_ref node) const
			{
				return NodeData{
					.heuristic_cost = 0.0f,
					.is_goal = node == goal,
				};
			}

			// Cost-to-node and cost-to-goal share the same units in this regression case.
			cost_type CombinedCost(cost_type cost_to_node, cost_type cost_to_goal) const
			{
				return cost_to_node + cost_to_goal;
			}
		};
		static_assert(astar::AdapterType<WeightedAdptr>);

		// Validate the returned path order for the coordinate-based graph cases below.
		void RunTest(Graph::Node const& start, Graph::Node const& goal, bool should_find, std::span<int const> expected)
		{
			Adptr adapter(graph, goal);
			auto start_index = static_cast<int>(&start - &graph.m_nodes[0]);
			auto [found, path] = astar::Search(adapter, start_index, 100.0f);

			PR_EXPECT(found == should_find);
			PR_EXPECT(path.size() == expected.size());
			for (int i = 0; i != ssize(path); ++i)
			{
				auto const& hop = path[i];
				auto const& adj = graph.m_adj[hop.node];

				PR_EXPECT(hop.node == expected[i]);
				PR_EXPECT(i + 1 == ssize(path) || adj[hop.edge] == path[i + 1].node);
			}
		}

		// Validate the returned cumulative path costs using a simple weighted chain.
		PRUnitTestMethod(PathCostReconstruction)
		{
			WeightedGraph graph;
			WeightedAdptr adapter(graph, 3);
			auto [found, path] = astar::Search(adapter, 0, 100.0f);

			PR_EXPECT(found);
			PR_EXPECT(path.size() == 4);

			int const expected_nodes[] = { 0, 1, 2, 3 };
			float const expected_costs[] = { 0.0f, 2.0f, 5.0f, 10.0f };
			for (int i = 0; i != ssize(path); ++i)
			{
				auto const& hop = path[i];

				PR_EXPECT(hop.node == expected_nodes[i]);
				PR_EXPECT(hop.cost_to_node == expected_costs[i]);
				PR_EXPECT(i + 1 == ssize(path) || graph.adj[hop.node][hop.edge].target == path[i + 1].node);
			}
		}

		PRUnitTestMethod(General)
		{
			{ // Node0 -> Node8
				int const expected[] = { 0, 10, 5, 8 };
				RunTest(graph.m_nodes[0], graph.m_nodes[8], true, expected);
			}
			{ // Node6 -> Node3
				int const expected[] = { 6, 2, 9, 3 };
				RunTest(graph.m_nodes[6], graph.m_nodes[3], true, expected);
			}
			{ // Node4 -> (0,0)
				int const expected[] = { 4, 10, 5 };
				RunTest(graph.m_nodes[4], Graph::Node{ 0, 0 }, false, expected);
			}
			{ // Node7 -> (7,7)
				int const expected[] = { 7, 1, 6, 2 };
				RunTest(graph.m_nodes[7], Graph::Node{ 7, 7 }, false, expected);
			}
			{ // Node7 -> Node7 (degenerate case)
				RunTest(graph.m_nodes[7], graph.m_nodes[7], true, {});
			}
			{ // Node7 -> (-4,0) ("near" node7 degenerate case)
				RunTest(graph.m_nodes[7], Graph::Node{ -4, 0 }, false, {});
			}
		}

		PRUnitTestMethod(EdgeDataDefaultsToNoNode)
		{
			auto edge = astar::EdgeData<EdgeDataConfig>{};
			PR_EXPECT(edge.target_node == EdgeDataConfig::NoNode);
		}
	};
}
#endif
