// HyperPose Tools
// Copyright (c) 2025
#pragma once
#include <vector>
#include <set>
#include <map>
#include <stack>
#include <queue>
#include <algorithm>
#include <numeric>
#include <functional>
#include <cassert>
#include <cstdint>

// Trapping Set Detection for directed graphs.
//
// A trapping set is a subset of nodes from which, once entered, all reachable
// nodes remain within the subset. In the context of PDP networks, trapping sets
// identify regions of the animation graph that cannot be exited once entered.
//
// Algorithm:
//   1. Chain contraction — compress linear chains (in=1, out=1) into super-nodes
//   2. Tarjan's SCC on the super-graph
//   3. Condensation DAG — compute reachable super-node sets per SCC
//   4. Trapping set identification — deduplicate by reachable super-node set
//   5. Signature refinement — merge adjacent super-nodes with matching signatures
//   6. Re-run detection on the refined graph for tighter chains
//   7. Build containment hierarchy + node ownership + statistics
//
// Check unit tests for usage

namespace hyperpose::trapping_sets
{
	// A single trapping set (subgraph that cannot be exited)
	struct TrappingSet
	{
		int id = 0;

		// Chains of original node IDs that are exclusively owned by this trapping set
		// (i.e. not contained in any child trapping set)
		std::vector<std::vector<int>> node_chains;

		// IDs of directly contained child trapping sets
		std::vector<int> child_sets;

		// Total number of original nodes in this trapping set (inclusive of children)
		int total_node_count = 0;
	};

	// Result of trapping set detection
	struct Result
	{
		// All trapping sets, ordered by containment (parents before children).
		// sets[0] is always the full graph.
		std::vector<TrappingSet> sets;

		// Per original-node: the owning trapping set index (smallest containing set)
		std::vector<int> node_owner;

		// Chain statistics
		int chain_count = 0;
		float avg_chain_length = 0.f;
	};

	namespace detail
	{
		// Internal adjacency list graph representation
		struct Graph
		{
			int node_count = 0;
			std::vector<std::vector<int>> adj;

			void Resize(int n)
			{
				node_count = n;
				adj.resize(n);
			}

			void AddEdge(int from, int to)
			{
				adj[from].push_back(to);
			}

			std::vector<std::vector<int>> BuildReverseAdj() const
			{
				std::vector<std::vector<int>> rev(node_count);
				for (int v = 0; v < node_count; ++v)
					for (int w : adj[v])
						rev[w].push_back(v);
				return rev;
			}
		};

		// Chain contraction result
		struct ChainContraction
		{
			Graph super_graph;
			std::vector<std::vector<int>> chains;    // per super-node: ordered chain of original node IDs
			std::vector<int> node_to_super;           // original node → super-node ID
		};

		// Iterative Tarjan's SCC to avoid stack overflow on large graphs
		struct TarjanSCC
		{
			std::vector<int> scc_id;
			int scc_count = 0;

			void Run(Graph const& graph)
			{
				int n = graph.node_count;
				scc_id.assign(n, -1);
				scc_count = 0;

				std::vector<int> index(n, -1);
				std::vector<int> lowlink(n, -1);
				std::vector<bool> on_stack(n, false);
				std::stack<int> stk;
				int next_index = 0;

				for (int v = 0; v < n; ++v)
				{
					if (index[v] != -1)
						continue;

					// Iterative strong-connect starting from v
					struct Frame { int node; int neighbor_idx; };
					std::stack<Frame> call_stack;

					index[v] = lowlink[v] = next_index++;
					stk.push(v);
					on_stack[v] = true;
					call_stack.push({v, 0});

					while (!call_stack.empty())
					{
						auto& frame = call_stack.top();
						int u = frame.node;
						auto const& neighbors = graph.adj[u];

						if (frame.neighbor_idx < static_cast<int>(neighbors.size()))
						{
							int w = neighbors[frame.neighbor_idx];
							frame.neighbor_idx++;

							if (index[w] == -1)
							{
								index[w] = lowlink[w] = next_index++;
								stk.push(w);
								on_stack[w] = true;
								call_stack.push({w, 0});
							}
							else if (on_stack[w])
							{
								lowlink[u] = std::min(lowlink[u], lowlink[w]);
							}
						}
						else
						{
							// All neighbors processed
							if (lowlink[u] == index[u])
							{
								int w;
								do
								{
									w = stk.top();
									stk.pop();
									on_stack[w] = false;
									scc_id[w] = scc_count;
								} while (w != u);
								scc_count++;
							}

							call_stack.pop();

							// Propagate lowlink to parent
							if (!call_stack.empty())
							{
								int parent = call_stack.top().node;
								lowlink[parent] = std::min(lowlink[parent], lowlink[u]);
							}
						}
					}
				}
			}
		};

		// Condensation DAG built from SCC results
		struct CondensationDAG
		{
			int scc_count;
			std::vector<std::set<int>> adj_sets;
			std::vector<std::vector<int>> scc_members; // super-node IDs in each SCC

			CondensationDAG(Graph const& super_graph, std::vector<int> const& scc_id, int scc_count)
				: scc_count(scc_count)
				, adj_sets(scc_count)
				, scc_members(scc_count)
			{
				for (int v = 0; v < super_graph.node_count; ++v)
					scc_members[scc_id[v]].push_back(v);

				for (int v = 0; v < super_graph.node_count; ++v)
					for (int w : super_graph.adj[v])
						if (scc_id[v] != scc_id[w])
							adj_sets[scc_id[v]].insert(scc_id[w]);
			}

			// For each SCC, compute all reachable SCCs and collect their super-node IDs
			std::vector<std::set<int>> ComputeReachableSuperNodeSets() const
			{
				std::vector<std::set<int>> reachable(scc_count);
				for (int c = 0; c < scc_count; ++c)
				{
					std::queue<int> q;
					std::vector<bool> visited(scc_count, false);
					q.push(c);
					visited[c] = true;

					while (!q.empty())
					{
						int cur = q.front();
						q.pop();
						for (int neighbor : adj_sets[cur])
						{
							if (!visited[neighbor])
							{
								visited[neighbor] = true;
								q.push(neighbor);
							}
						}
					}

					for (int s = 0; s < scc_count; ++s)
						if (visited[s])
							for (int sn : scc_members[s])
								reachable[c].insert(sn);
				}
				return reachable;
			}
		};

		// Contract linear chains into super-nodes.
		// A node is chainable if in-degree=1 AND out-degree=1.
		inline ChainContraction ContractChains(Graph const& original)
		{
			int const n = original.node_count;

			// Compute in-degree
			std::vector<int> in_deg(n, 0);
			for (int v = 0; v < n; ++v)
				for (int w : original.adj[v])
					in_deg[w]++;

			auto is_chainable = [&](int v) -> bool
			{
				return in_deg[v] == 1 && static_cast<int>(original.adj[v].size()) == 1;
			};

			auto reverse_adj = original.BuildReverseAdj();

			// Build chains starting from non-chainable nodes
			std::vector<bool> visited(n, false);
			std::vector<std::vector<int>> chains;

			for (int v = 0; v < n; ++v)
			{
				if (is_chainable(v) || visited[v])
					continue;

				for (int w : original.adj[v])
				{
					if (!is_chainable(w) || visited[w])
						continue;

					std::vector<int> chain;
					int cur = w;
					while (is_chainable(cur) && !visited[cur])
					{
						visited[cur] = true;
						chain.push_back(cur);
						cur = original.adj[cur][0];
					}
					if (!chain.empty())
						chains.push_back(std::move(chain));
				}
			}

			// Handle cycle chains (closed loops of chainable nodes)
			for (int v = 0; v < n; ++v)
			{
				if (!is_chainable(v) || visited[v])
					continue;

				std::vector<int> chain;
				int cur = v;
				while (!visited[cur])
				{
					visited[cur] = true;
					chain.push_back(cur);
					cur = original.adj[cur][0];
				}
				if (!chain.empty())
					chains.push_back(std::move(chain));
			}

			// Extend chains at boundaries (absorb adjacent out=1/in=1 nodes)
			for (auto& chain : chains)
			{
				// Forward extension
				while (true)
				{
					int end = chain.back();
					if (static_cast<int>(original.adj[end].size()) != 1)
						break;
					int child = original.adj[end][0];
					if (in_deg[child] != 1 || visited[child])
						break;
					visited[child] = true;
					chain.push_back(child);
				}

				// Backward extension
				while (true)
				{
					int head = chain.front();
					if (in_deg[head] != 1)
						break;
					int parent = reverse_adj[head][0];
					if (static_cast<int>(original.adj[parent].size()) != 1 || visited[parent])
						break;
					visited[parent] = true;
					chain.insert(chain.begin(), parent);
				}
			}

			// Catch additional chains from unvisited nodes via the extension rule
			for (int v = 0; v < n; ++v)
			{
				if (visited[v])
					continue;

				if (static_cast<int>(original.adj[v].size()) != 1)
					continue;
				int child = original.adj[v][0];
				if (in_deg[child] != 1 || visited[child])
					continue;

				std::vector<int> chain;
				visited[v] = true;
				chain.push_back(v);
				visited[child] = true;
				chain.push_back(child);

				// Forward
				while (true)
				{
					int end = chain.back();
					if (static_cast<int>(original.adj[end].size()) != 1)
						break;
					int next = original.adj[end][0];
					if (in_deg[next] != 1 || visited[next])
						break;
					visited[next] = true;
					chain.push_back(next);
				}

				// Backward
				while (true)
				{
					int head = chain.front();
					if (in_deg[head] != 1)
						break;
					int parent = reverse_adj[head][0];
					if (static_cast<int>(original.adj[parent].size()) != 1 || visited[parent])
						break;
					visited[parent] = true;
					chain.insert(chain.begin(), parent);
				}

				chains.push_back(std::move(chain));
			}

			// Assign super-node IDs
			ChainContraction result;
			result.node_to_super.resize(n, -1);

			int super_id = 0;

			// Singleton super-nodes for unchained nodes
			for (int v = 0; v < n; ++v)
			{
				if (!visited[v])
				{
					result.node_to_super[v] = super_id;
					result.chains.push_back({v});
					super_id++;
				}
			}

			// Chain super-nodes
			for (auto const& chain : chains)
			{
				for (int node : chain)
					result.node_to_super[node] = super_id;
				result.chains.push_back(chain);
				super_id++;
			}

			// Build super-graph
			int super_count = super_id;
			result.super_graph.Resize(super_count);

			for (int v = 0; v < n; ++v)
				for (int w : original.adj[v])
				{
					int sv = result.node_to_super[v];
					int sw = result.node_to_super[w];
					if (sv != sw)
						result.super_graph.AddEdge(sv, sw);
				}

			// Deduplicate edges
			for (int sv = 0; sv < super_count; ++sv)
			{
				auto& adj = result.super_graph.adj[sv];
				std::sort(adj.begin(), adj.end());
				adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
			}

			return result;
		}

		// Compute trapping-set signatures for each super-node (set of trapping-set indices it belongs to)
		using SuperNodeSignatures = std::vector<std::set<int>>;

		inline SuperNodeSignatures ComputeSignatures(Graph const& super_graph, TarjanSCC const& tarjan)
		{
			CondensationDAG dag(super_graph, tarjan.scc_id, tarjan.scc_count);
			auto reachable_sets = dag.ComputeReachableSuperNodeSets();

			// Deduplicate trapping sets
			std::map<std::set<int>, int> unique_sets;
			for (int c = 0; c < tarjan.scc_count; ++c)
				if (!unique_sets.contains(reachable_sets[c]))
					unique_sets[reachable_sets[c]] = c;

			// Full graph set
			std::set<int> full_graph;
			for (int i = 0; i < super_graph.node_count; ++i)
				full_graph.insert(i);

			// Collect trapping sets (full graph = index 0)
			std::vector<std::set<int>> ts_sets;
			ts_sets.push_back(full_graph);
			for (auto& [sn_set, _] : unique_sets)
				if (sn_set != full_graph)
					ts_sets.push_back(sn_set);

			// For each super-node, record which trapping sets contain it
			SuperNodeSignatures signatures(super_graph.node_count);
			for (int ts = 0; ts < static_cast<int>(ts_sets.size()); ++ts)
				for (int sn : ts_sets[ts])
					signatures[sn].insert(ts);

			return signatures;
		}

		// Refine chains by merging adjacent super-nodes with matching trapping-set signatures
		inline ChainContraction RefineChains(
			Graph const& original,
			ChainContraction const& initial,
			SuperNodeSignatures const& signatures)
		{
			int const n = original.node_count;
			int const super_count = static_cast<int>(initial.chains.size());

			auto const& sg = initial.super_graph;

			// Build reverse adjacency on super-graph
			std::vector<std::vector<int>> super_rev_adj(super_count);
			for (int sv = 0; sv < super_count; ++sv)
				for (int sw : sg.adj[sv])
					super_rev_adj[sw].push_back(sv);

			// Count same-signature in/out degree
			auto same_sig_out_deg = [&](int sn) -> int
			{
				int count = 0;
				for (int w : sg.adj[sn])
					if (signatures[w] == signatures[sn])
						count++;
				return count;
			};
			auto same_sig_in_deg = [&](int sn) -> int
			{
				int count = 0;
				for (int w : super_rev_adj[sn])
					if (signatures[w] == signatures[sn])
						count++;
				return count;
			};

			// Build maximal paths through same-signature super-nodes
			std::vector<bool> visited(super_count, false);
			std::vector<std::vector<int>> merged_chains;

			for (int sn = 0; sn < super_count; ++sn)
			{
				if (visited[sn] || same_sig_in_deg(sn) == 1)
					continue;

				std::vector<int> path;
				int cur = sn;
				while (cur >= 0 && !visited[cur])
				{
					visited[cur] = true;
					path.push_back(cur);

					int next = -1;
					if (same_sig_out_deg(cur) == 1)
					{
						for (int w : sg.adj[cur])
						{
							if (signatures[w] == signatures[cur])
							{
								if (same_sig_in_deg(w) == 1)
									next = w;
								break;
							}
						}
					}
					cur = next;
				}

				// Flatten: concatenate original node chains
				std::vector<int> merged;
				for (int p : path)
					for (int node : initial.chains[p])
						merged.push_back(node);
				merged_chains.push_back(std::move(merged));
			}

			// Handle unvisited super-nodes (e.g. cycles within same-signature groups)
			for (int sn = 0; sn < super_count; ++sn)
			{
				if (visited[sn])
					continue;

				std::vector<int> path;
				int cur = sn;
				while (!visited[cur])
				{
					visited[cur] = true;
					path.push_back(cur);

					int next = -1;
					if (same_sig_out_deg(cur) == 1)
					{
						for (int w : sg.adj[cur])
						{
							if (signatures[w] == signatures[cur] && !visited[w])
							{
								if (same_sig_in_deg(w) == 1)
									next = w;
								break;
							}
						}
					}
					if (next < 0) break;
					cur = next;
				}

				std::vector<int> merged;
				for (int p : path)
					for (int node : initial.chains[p])
						merged.push_back(node);
				merged_chains.push_back(std::move(merged));
			}

			// Build new contraction result
			ChainContraction result;
			result.node_to_super.resize(n, -1);

			for (int new_sn = 0; new_sn < static_cast<int>(merged_chains.size()); ++new_sn)
			{
				for (int node : merged_chains[new_sn])
					result.node_to_super[node] = new_sn;
				result.chains.push_back(std::move(merged_chains[new_sn]));
			}

			int new_super_count = static_cast<int>(result.chains.size());
			result.super_graph.Resize(new_super_count);

			for (int v = 0; v < n; ++v)
				for (int w : original.adj[v])
				{
					int sv = result.node_to_super[v];
					int sw = result.node_to_super[w];
					if (sv != sw)
						result.super_graph.AddEdge(sv, sw);
				}

			// Deduplicate edges
			for (int sv = 0; sv < new_super_count; ++sv)
			{
				auto& adj = result.super_graph.adj[sv];
				std::sort(adj.begin(), adj.end());
				adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
			}

			return result;
		}

		// Core trapping set detection on a contracted super-graph
		struct RawTrappingSet
		{
			std::set<int> super_nodes;
		};

		struct DetectionResult
		{
			std::vector<RawTrappingSet> raw_sets;         // sorted by size descending, [0] = full graph
			std::vector<std::vector<int>> children;       // containment hierarchy
			std::vector<std::set<int>> exclusive_super;   // per-set: super-nodes not in any child
			std::vector<int> node_owner;                  // per original-node: owning set index
		};

		inline DetectionResult DetectOnContraction(Graph const& original, ChainContraction const& contraction)
		{
			auto const& sg = contraction.super_graph;
			auto const& chains = contraction.chains;

			// Tarjan's SCC on super-graph
			TarjanSCC tarjan;
			tarjan.Run(sg);

			// Condensation DAG + reachable sets
			CondensationDAG dag(sg, tarjan.scc_id, tarjan.scc_count);
			auto reachable_sets = dag.ComputeReachableSuperNodeSets();

			// Deduplicate trapping sets
			std::map<std::set<int>, int> unique_sets;
			for (int c = 0; c < tarjan.scc_count; ++c)
				if (!unique_sets.contains(reachable_sets[c]))
					unique_sets[reachable_sets[c]] = c;

			// Full graph super-node set
			std::set<int> full_graph;
			for (int i = 0; i < sg.node_count; ++i)
				full_graph.insert(i);

			// Collect all trapping sets sorted by size descending
			std::vector<RawTrappingSet> raw_sets;
			raw_sets.push_back({full_graph});

			std::vector<std::pair<std::set<int>, int>> sorted_sets(unique_sets.begin(), unique_sets.end());
			std::ranges::sort(sorted_sets, [](auto const& a, auto const& b) { return a.first.size() > b.first.size(); });
			for (auto const& [sn_set, _] : sorted_sets)
				if (sn_set != full_graph)
					raw_sets.push_back({sn_set});

			// Build containment hierarchy
			int num_sets = static_cast<int>(raw_sets.size());
			std::vector<std::vector<int>> children(num_sets);

			for (int i = 1; i < num_sets; ++i)
			{
				std::vector<int> direct_parents;
				for (int j = i - 1; j >= 0; --j)
				{
					if (!std::includes(raw_sets[j].super_nodes.begin(), raw_sets[j].super_nodes.end(),
					                   raw_sets[i].super_nodes.begin(), raw_sets[i].super_nodes.end()))
						continue;

					bool is_direct = true;
					for (int p : direct_parents)
					{
						if (std::includes(raw_sets[j].super_nodes.begin(), raw_sets[j].super_nodes.end(),
						                  raw_sets[p].super_nodes.begin(), raw_sets[p].super_nodes.end()))
						{
							is_direct = false;
							break;
						}
					}
					if (is_direct)
					{
						direct_parents.push_back(j);
						children[j].push_back(i);
					}
				}
			}

			// Compute exclusive super-nodes per set (not in any child)
			std::vector<std::set<int>> exclusive_super(num_sets);
			for (int i = 0; i < num_sets; ++i)
			{
				std::set<int> child_sn;
				for (int child_id : children[i])
					child_sn.insert(raw_sets[child_id].super_nodes.begin(), raw_sets[child_id].super_nodes.end());

				for (int sn : raw_sets[i].super_nodes)
					if (!child_sn.contains(sn))
						exclusive_super[i].insert(sn);
			}

			// Build node ownership (smallest containing set)
			std::vector<int> node_owner(original.node_count, 0);
			for (int i = 0; i < num_sets; ++i)
				for (int sn : exclusive_super[i])
					for (int node : chains[sn])
						node_owner[node] = i;

			// Topological sort of trapping sets for stable ordering
			// Build dependency graph: parent→child
			std::vector<std::set<int>> dep_adj(num_sets);
			for (int i = 0; i < num_sets; ++i)
				for (int child : children[i])
					dep_adj[i].insert(child);

			// Kahn's topological sort
			std::vector<int> in_deg(num_sets, 0);
			for (int i = 0; i < num_sets; ++i)
				for (int j : dep_adj[i])
					in_deg[j]++;

			std::queue<int> ready;
			for (int i = 0; i < num_sets; ++i)
				if (in_deg[i] == 0)
					ready.push(i);

			std::vector<int> topo_order;
			while (!ready.empty())
			{
				// Emit the largest sets first within each level
				std::vector<int> level;
				while (!ready.empty())
				{
					level.push_back(ready.front());
					ready.pop();
				}
				std::ranges::sort(level, [&](int a, int b) { return raw_sets[a].super_nodes.size() > raw_sets[b].super_nodes.size(); });
				for (int m : level)
				{
					topo_order.push_back(m);
					for (int next : dep_adj[m])
						if (--in_deg[next] == 0)
							ready.push(next);
				}
			}

			// Remap to topological order
			std::vector<int> new_id(num_sets, -1);
			for (int new_idx = 0; new_idx < num_sets; ++new_idx)
				new_id[topo_order[new_idx]] = new_idx;

			DetectionResult result;
			result.raw_sets.resize(num_sets);
			result.children.resize(num_sets);
			result.exclusive_super.resize(num_sets);
			result.node_owner.resize(original.node_count);

			for (int new_idx = 0; new_idx < num_sets; ++new_idx)
			{
				int old_idx = topo_order[new_idx];
				result.raw_sets[new_idx] = std::move(raw_sets[old_idx]);
				result.exclusive_super[new_idx] = std::move(exclusive_super[old_idx]);

				for (int child : children[old_idx])
					result.children[new_idx].push_back(new_id[child]);
				std::ranges::sort(result.children[new_idx]);
			}

			for (int v = 0; v < original.node_count; ++v)
				result.node_owner[v] = new_id[node_owner[v]];

			return result;
		}

		// Expand super-nodes to original nodes
		inline std::set<int> ExpandToOriginalNodes(std::set<int> const& super_nodes, std::vector<std::vector<int>> const& chains)
		{
			std::set<int> nodes;
			for (int sn : super_nodes)
				for (int n : chains[sn])
					nodes.insert(n);
			return nodes;
		}
	}

	// Detect trapping sets in a directed graph.
	//
	// Parameters:
	//   node_count     - Number of nodes in the graph (nodes are indexed 0..node_count-1)
	//   get_successors - Callable: get_successors(int node) returns a range of successor node indices
	//
	// Returns a Result with trapping sets, node ownership, and chain statistics.
	template <typename GetSuccessors>
	Result Detect(int node_count, GetSuccessors get_successors)
	{
		// Build internal adjacency list from the caller's graph
		detail::Graph graph;
		graph.Resize(node_count);
		for (int v = 0; v < node_count; ++v)
			for (int w : get_successors(v))
				graph.AddEdge(v, w);

		// Pass 1: Chain contraction
		auto contraction = detail::ContractChains(graph);

		// Compute trapping-set signatures for refinement
		detail::TarjanSCC tarjan;
		tarjan.Run(contraction.super_graph);
		auto signatures = detail::ComputeSignatures(contraction.super_graph, tarjan);

		// Pass 2: Refine chains by merging same-signature neighbors
		contraction = detail::RefineChains(graph, contraction, signatures);

		// Detect trapping sets on the refined contraction
		auto detection = detail::DetectOnContraction(graph, contraction);

		// Assemble final result
		Result result;
		int num_sets = static_cast<int>(detection.raw_sets.size());
		result.sets.resize(num_sets);
		result.node_owner = std::move(detection.node_owner);

		int total_chain_nodes = 0;
		int chain_count = 0;

		for (int i = 0; i < num_sets; ++i)
		{
			auto& ts = result.sets[i];
			ts.id = i;
			ts.child_sets = std::move(detection.children[i]);
			ts.total_node_count = static_cast<int>(
				detail::ExpandToOriginalNodes(detection.raw_sets[i].super_nodes, contraction.chains).size());

			// Build node chains from exclusive super-nodes
			for (int sn : detection.exclusive_super[i])
			{
				ts.node_chains.push_back(contraction.chains[sn]);
				total_chain_nodes += static_cast<int>(contraction.chains[sn].size());
				chain_count++;
			}
		}

		result.chain_count = chain_count;
		result.avg_chain_length = chain_count > 0 ? static_cast<float>(total_chain_nodes) / chain_count : 0.f;

		return result;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace hyperpose::trapping_sets::unittest
{
	using namespace hyperpose::trapping_sets;

	// Helper: build a get_successors function from an edge list
	inline auto MakeSuccessors(int node_count, std::vector<std::pair<int, int>> const& edges)
	{
		std::vector<std::vector<int>> adj(node_count);
		for (auto [from, to] : edges)
			adj[from].push_back(to);
		return adj;
	}

	// Helper: collect all original nodes owned by a trapping set (including children)
	inline std::set<int> AllNodes(Result const& r, int set_idx)
	{
		std::set<int> nodes;
		for (auto const& chain : r.sets[set_idx].node_chains)
			for (int n : chain)
				nodes.insert(n);
		for (int child : r.sets[set_idx].child_sets)
		{
			auto child_nodes = AllNodes(r, child);
			nodes.insert(child_nodes.begin(), child_nodes.end());
		}
		return nodes;
	}

	PRUnitTestClass(TrappingSetTests)
	{
		PRUnitTestMethod(SingleNode)
		{
			// One node, no edges → one trapping set (the full graph)
			auto adj = MakeSuccessors(1, {});
			auto result = Detect(1, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 1);
			PR_EXPECT(result.node_owner[0] == 0);
		}

		PRUnitTestMethod(LinearChain)
		{
			// A→B→C→D (no cycles) → one trapping set (full graph)
			// Chain contraction should merge the linear portion
			auto adj = MakeSuccessors(4, {{0,1},{1,2},{2,3}});
			auto result = Detect(4, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 4);
			PR_EXPECT(result.chain_count >= 1);

			// All nodes owned by set 0
			for (int i = 0; i != 4; ++i)
				PR_EXPECT(result.node_owner[i] == 0);
		}

		PRUnitTestMethod(SimpleCycle)
		{
			// 0→1→2→0 (simple cycle) → one trapping set (full graph)
			auto adj = MakeSuccessors(3, {{0,1},{1,2},{2,0}});
			auto result = Detect(3, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 3);
		}

		PRUnitTestMethod(SelfLoop)
		{
			// 0→0 (self-loop) → one trapping set
			auto adj = MakeSuccessors(1, {{0,0}});
			auto result = Detect(1, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 1);
		}

		PRUnitTestMethod(ChainIntoCycle)
		{
			// 0→1→2→3→1 (node 0 leads into the cycle 1→2→3→1)
			// Expected: full graph (set 0) + trapping set {1,2,3}
			auto adj = MakeSuccessors(4, {{0,1},{1,2},{2,3},{3,1}});
			auto result = Detect(4, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 2);

			// Set 0 is the full graph
			PR_EXPECT(result.sets[0].total_node_count == 4);

			// Set 1 is the cycle {1,2,3}
			PR_EXPECT(result.sets[1].total_node_count == 3);
			auto cycle_nodes = AllNodes(result, 1);
			PR_EXPECT(cycle_nodes.count(1) == 1);
			PR_EXPECT(cycle_nodes.count(2) == 1);
			PR_EXPECT(cycle_nodes.count(3) == 1);
			PR_EXPECT(cycle_nodes.count(0) == 0);

			// Node 0 is owned by the full graph (set 0), cycle nodes by set 1
			PR_EXPECT(result.node_owner[0] == 0);
			PR_EXPECT(result.node_owner[1] != 0);
			PR_EXPECT(result.node_owner[2] != 0);
			PR_EXPECT(result.node_owner[3] != 0);
		}

		PRUnitTestMethod(TwoDisjointCycles)
		{
			// Cycle A: 0→1→0, Cycle B: 2→3→2
			// No edges between them → full graph + 2 sub trapping sets
			auto adj = MakeSuccessors(4, {{0,1},{1,0},{2,3},{3,2}});
			auto result = Detect(4, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 3);

			// Full graph
			PR_EXPECT(result.sets[0].total_node_count == 4);
			PR_EXPECT(result.sets[0].child_sets.size() == 2);

			// Each child has 2 nodes
			for (int child : result.sets[0].child_sets)
				PR_EXPECT(result.sets[child].total_node_count == 2);
		}

		PRUnitTestMethod(NestedCycles)
		{
			// Outer cycle: 0→1→2→0
			// Inner cycle: 1→3→1 (3 can only reach itself and 1)
			// Node 0→1, 1→2, 2→0, 1→3, 3→1
			// From any node you can reach all others via the outer cycle,
			// but {1,3} forms a trapping sub-set since 3→1 and 1→3 stay within {1,3}?
			// Actually 1→2 exits {1,3}, so {1,3} is NOT a trapping set.
			// Let me redesign: 0→1→2→3→2 and 0→1 with 1→0
			//
			// Better: 0→1→2→0 (outer), 3→4→3 (inner), 2→3 (bridge from outer to inner)
			// Once in {3,4} you can't leave → inner trapping set
			// Once in {0,1,2,3,4} you can't leave → that's the full graph
			// {0,1,2,3,4} has child {3,4}
			auto adj = MakeSuccessors(5, {{0,1},{1,2},{2,0},{2,3},{3,4},{4,3}});
			auto result = Detect(5, [&](int v) { return adj[v]; });

			// Expect: full graph {0-4} → inner {3,4}
			PR_EXPECT(result.sets.size() >= 2);

			// Find the inner set containing nodes 3 and 4
			bool found_inner = false;
			for (int i = 1; i < static_cast<int>(result.sets.size()); ++i)
			{
				auto nodes = AllNodes(result, i);
				if (nodes.count(3) && nodes.count(4) && !nodes.count(0))
				{
					found_inner = true;
					PR_EXPECT(result.sets[i].total_node_count == 2);
				}
			}
			PR_EXPECT(found_inner);
		}

		PRUnitTestMethod(DiamondToSink)
		{
			// 0→1, 0→2, 1→3, 2→3 (diamond converging to sink node 3)
			// No cycles → one trapping set (full graph)
			auto adj = MakeSuccessors(4, {{0,1},{0,2},{1,3},{2,3}});
			auto result = Detect(4, [&](int v) { return adj[v]; });

			// Sink node 3 has no outgoing edges → it's a trivial trapping set
			// Could be: full graph + {3}
			PR_EXPECT(result.sets.size() >= 1);
			PR_EXPECT(result.sets[0].total_node_count == 4);
		}

		PRUnitTestMethod(NodeOwnership)
		{
			// 0→1→2→1 (0 feeds into cycle {1,2})
			// Node 0 owned by full graph, nodes 1,2 owned by inner set
			auto adj = MakeSuccessors(3, {{0,1},{1,2},{2,1}});
			auto result = Detect(3, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 2);

			// Node 0: owned by full graph (set 0)
			PR_EXPECT(result.node_owner[0] == 0);

			// Nodes 1,2: owned by the inner trapping set
			PR_EXPECT(result.node_owner[1] == result.node_owner[2]);
			PR_EXPECT(result.node_owner[1] != 0);
		}

		PRUnitTestMethod(ChainStatistics)
		{
			// 0→1→2→3→4→2 (long chain 0→1 into cycle 2→3→4→2)
			auto adj = MakeSuccessors(5, {{0,1},{1,2},{2,3},{3,4},{4,2}});
			auto result = Detect(5, [&](int v) { return adj[v]; });

			PR_EXPECT(result.chain_count >= 1);
			PR_EXPECT(result.avg_chain_length > 0.f);

			// All 5 nodes accounted for
			PR_EXPECT(result.sets[0].total_node_count == 5);
		}

		PRUnitTestMethod(LargerGraph)
		{
			// 10 nodes: two cycles connected by a bridge
			// Cycle A: 0→1→2→3→0
			// Cycle B: 6→7→8→9→6
			// Bridge: 3→4→5→6
			// Once in cycle B, can't get back to A → cycle B is a trapping set
			auto adj = MakeSuccessors(10, {
				{0,1},{1,2},{2,3},{3,0},   // Cycle A
				{3,4},{4,5},{5,6},          // Bridge A→B
				{6,7},{7,8},{8,9},{9,6},   // Cycle B
			});
			auto result = Detect(10, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets[0].total_node_count == 10);

			// Should find cycle B as a distinct trapping set
			bool found_cycle_b = false;
			for (int i = 1; i < static_cast<int>(result.sets.size()); ++i)
			{
				auto nodes = AllNodes(result, i);
				if (nodes.count(6) && nodes.count(7) && nodes.count(8) && nodes.count(9) && !nodes.count(0))
				{
					found_cycle_b = true;
					break;
				}
			}
			PR_EXPECT(found_cycle_b);

			// Nodes in cycle A and bridge are owned by larger sets
			PR_EXPECT(result.node_owner[0] != result.node_owner[6]);
		}

		PRUnitTestMethod(ContainmentHierarchy)
		{
			// Full graph → set with cycle B+C → cycle C
			// 0→1→0 (cycle A), 1→2, 2→3→2 (cycle B), 3→4, 4→5→4 (cycle C)
			// From cycle A: can reach B,C → not a trapping subset
			// From cycle B: can reach C → {2,3,4,5} is a trapping set
			// From cycle C: can't leave → {4,5} is a trapping set
			auto adj = MakeSuccessors(6, {
				{0,1},{1,0},     // Cycle A
				{1,2},           // A→B
				{2,3},{3,2},     // Cycle B
				{3,4},           // B→C
				{4,5},{5,4},     // Cycle C
			});
			auto result = Detect(6, [&](int v) { return adj[v]; });

			// Expect at least 3 trapping sets: full graph, {2,3,4,5}, {4,5}
			PR_EXPECT(result.sets.size() >= 3);

			// Full graph has children
			PR_EXPECT(!result.sets[0].child_sets.empty());

			// Verify containment: there exists a set with child_sets that includes the innermost cycle
			bool found_nested = false;
			for (auto const& ts : result.sets)
			{
				if (!ts.child_sets.empty() && ts.total_node_count < 6 && ts.total_node_count > 2)
				{
					found_nested = true;
					break;
				}
			}
			PR_EXPECT(found_nested);
		}
	};
}
#endif
