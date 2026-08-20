// HyperPose Tools
// Copyright (c) 2025
#pragma once
#include <vector>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <stack>
#include <queue>
#include <algorithm>
#include <numeric>
#include <iterator>
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

namespace pr::trapping_sets
{
	// Readability aliases for the recurring "vector of int vectors" shapes, named by what they hold:
	//   AdjacencyList - per node/super-node: its list of successor ids (a directed-graph edge list)
	//   NodeChains    - per super-node: the ordered chain of original node ids it contracts
	//   IdSetList     - a list of id sets: reachable/trapping super-node sets, per-SCC members, containment children
	using AdjacencyList = std::vector<std::vector<int>>;
	using NodeChains = std::vector<std::vector<int>>;
	using IdSetList = std::vector<std::vector<int>>;

	// A single trapping set (subgraph that cannot be exited)
	struct TrappingSet
	{
		int id = 0;

		// Chains of original node IDs that are exclusively owned by this trapping set
		// (i.e. not contained in any child trapping set)
		NodeChains node_chains;

		// IDs of directly contained child trapping sets
		std::vector<int> child_sets;

		// Total number of original nodes in this trapping set (inclusive of children)
		int total_node_count = 0;

		// True for sets created by SquashChains. These represent linear funnels
		// (no cycle from end to start) rather than true trapping sets.
		bool linear_chain = false;
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
			AdjacencyList adj;

			void Resize(int n)
			{
				node_count = n;
				adj.resize(n);
			}

			void AddEdge(int from, int to)
			{
				adj[from].push_back(to);
			}

			AdjacencyList BuildReverseAdj() const
			{
				AdjacencyList rev(node_count);
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
			NodeChains chains;    // per super-node: ordered chain of original node IDs
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
				std::vector<int> stk;
				stk.reserve(n);

				// Both DFS stacks are bounded by the node count. Reuse their storage across disconnected roots rather
				// than constructing deque-backed std::stack instances for every root.
				struct Frame { int node; int neighbor_idx; };
				std::vector<Frame> call_stack;
				call_stack.reserve(n);
				int next_index = 0;

				for (int v = 0; v < n; ++v)
				{
					if (index[v] != -1)
						continue;

					// Iterative strong-connect starting from v
					call_stack.clear();

					index[v] = lowlink[v] = next_index++;
					stk.push_back(v);
					on_stack[v] = true;
					call_stack.push_back({v, 0});

					while (!call_stack.empty())
					{
						auto& frame = call_stack.back();
						int u = frame.node;
						auto const& neighbors = graph.adj[u];

						if (frame.neighbor_idx < static_cast<int>(neighbors.size()))
						{
							int w = neighbors[frame.neighbor_idx];
							frame.neighbor_idx++;

							if (index[w] == -1)
							{
								index[w] = lowlink[w] = next_index++;
								stk.push_back(w);
								on_stack[w] = true;
								call_stack.push_back({w, 0});
							}
							else if (on_stack[w])
							{
								// Use index[w], not lowlink[w]. In iterative Tarjan, lowlink[w] may not
								// be finalized yet since we haven't finished backtracking through w's subtree.
								lowlink[u] = std::min(lowlink[u], index[w]);
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
									w = stk.back();
									stk.pop_back();
									on_stack[w] = false;
									scc_id[w] = scc_count;
								} while (w != u);
								scc_count++;
							}

							call_stack.pop_back();

							// Propagate lowlink to parent
							if (!call_stack.empty())
							{
								int parent = call_stack.back().node;
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
			AdjacencyList adj;         // sorted, unique successor SCC ids per SCC (the condensation edges)
			IdSetList scc_members; // super-node IDs in each SCC

			CondensationDAG(Graph const& super_graph, std::vector<int> const& scc_id, int scc_count)
				: scc_count(scc_count)
				, adj(scc_count)
				, scc_members(scc_count)
			{
				for (int v = 0; v < super_graph.node_count; ++v)
					scc_members[scc_id[v]].push_back(v);

				for (int v = 0; v < super_graph.node_count; ++v)
					for (int w : super_graph.adj[v])
						if (scc_id[v] != scc_id[w])
							adj[scc_id[v]].push_back(scc_id[w]);

				// Deduplicate the condensation edges so each SCC's successor list is a sorted, unique set. A sorted
				// vector is cheaper to build and scan than a hash set, and the BFS below only needs to iterate it.
				for (auto& a : adj)
				{
					std::sort(a.begin(), a.end());
					a.erase(std::unique(a.begin(), a.end()), a.end());
				}
			}

			// Return SCCs in deterministic topological order. Reverse DFS post-order keeps tree-shaped descendant
			// regions contiguous, which lets the interval reachability representation stay compact.
			std::vector<int> TopologicalOrder() const
			{
				struct Frame
				{
					int scc;
					int next_child;
				};

				std::vector<bool> visited(scc_count, false);
				std::vector<Frame> stack;
				stack.reserve(scc_count);
				std::vector<int> order;
				order.reserve(scc_count);
				for (int root = 0; root != scc_count; ++root)
				{
					if (visited[root])
						continue;

					visited[root] = true;
					stack.push_back({ root, 0 });
					while (!stack.empty())
					{
						auto& frame = stack.back();
						auto const& children = adj[frame.scc];
						if (frame.next_child != static_cast<int>(children.size()))
						{
							auto const child = children[frame.next_child++];
							if (!visited[child])
							{
								visited[child] = true;
								stack.push_back({ child, 0 });
							}
							continue;
						}

						order.push_back(frame.scc);
						stack.pop_back();
					}
				}
				std::reverse(order.begin(), order.end());
				return order;
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
			NodeChains chains;

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

				// Backward extension - collect then bulk-insert to avoid O(n²) front-insertion
				{
					std::vector<int> prefix;
					int head = chain.front();
					while (true)
					{
						if (in_deg[head] != 1)
							break;
						int parent = reverse_adj[head][0];
						if (static_cast<int>(original.adj[parent].size()) != 1 || visited[parent])
							break;
						visited[parent] = true;
						prefix.push_back(parent);
						head = parent;
					}
					if (!prefix.empty())
					{
						std::reverse(prefix.begin(), prefix.end());
						chain.insert(chain.begin(), prefix.begin(), prefix.end());
					}
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

				// Backward - collect then bulk-insert to avoid O(n²) front-insertion
				{
					std::vector<int> prefix;
					int head = chain.front();
					while (true)
					{
						if (in_deg[head] != 1)
							break;
						int parent = reverse_adj[head][0];
						if (static_cast<int>(original.adj[parent].size()) != 1 || visited[parent])
							break;
						visited[parent] = true;
						prefix.push_back(parent);
						head = parent;
					}
					if (!prefix.empty())
					{
						std::reverse(prefix.begin(), prefix.end());
						chain.insert(chain.begin(), prefix.begin(), prefix.end());
					}
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

		// Refine chains by merging adjacent super-nodes in the same SCC. A super-node's trapping-set membership is
		// the set of SCCs that can reach it; two distinct SCCs cannot have the same membership because each reaches
		// itself, so SCC identity is exactly the signature equality needed here.
		inline ChainContraction RefineChains(Graph const& original, ChainContraction const& initial, std::vector<int> const& scc_ids)
		{
			int const n = original.node_count;
			int const super_count = static_cast<int>(initial.chains.size());

			auto const& sg = initial.super_graph;

			// Build reverse adjacency on super-graph
			AdjacencyList super_rev_adj(super_count);
			for (int sv = 0; sv < super_count; ++sv)
				for (int sw : sg.adj[sv])
					super_rev_adj[sw].push_back(sv);

			// Count same-signature in/out degree
			auto same_sig_out_deg = [&](int sn) -> int
			{
				int count = 0;
				for (int w : sg.adj[sn])
					if (scc_ids[w] == scc_ids[sn])
						count++;
				return count;
			};
			auto same_sig_in_deg = [&](int sn) -> int
			{
				int count = 0;
				for (int w : super_rev_adj[sn])
					if (scc_ids[w] == scc_ids[sn])
						count++;
				return count;
			};

			// Build maximal paths through same-signature super-nodes
			std::vector<bool> visited(super_count, false);
			NodeChains merged_chains;
			merged_chains.reserve(super_count);

			for (int sn = 0; sn < super_count; ++sn)
			{
				if (visited[sn] || same_sig_in_deg(sn) == 1)
					continue;

				std::vector<int> merged;
				int cur = sn;
				while (cur >= 0 && !visited[cur])
				{
					visited[cur] = true;
					merged.insert(merged.end(), initial.chains[cur].begin(), initial.chains[cur].end());

					int next = -1;
					if (same_sig_out_deg(cur) == 1)
					{
						for (int w : sg.adj[cur])
						{
							if (scc_ids[w] == scc_ids[cur])
							{
								if (same_sig_in_deg(w) == 1)
									next = w;
								break;
							}
						}
					}
					cur = next;
				}
				merged_chains.push_back(std::move(merged));
			}

			// Handle unvisited super-nodes (e.g. cycles within same-signature groups)
			for (int sn = 0; sn < super_count; ++sn)
			{
				if (visited[sn])
					continue;

				std::vector<int> merged;
				int cur = sn;
				while (!visited[cur])
				{
					visited[cur] = true;
					merged.insert(merged.end(), initial.chains[cur].begin(), initial.chains[cur].end());

					int next = -1;
					if (same_sig_out_deg(cur) == 1)
					{
						for (int w : sg.adj[cur])
						{
							if (scc_ids[w] == scc_ids[cur] && !visited[w])
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
				merged_chains.push_back(std::move(merged));
			}

			// Build new contraction result
			ChainContraction result;
			result.node_to_super.resize(n, -1);
			result.chains.reserve(merged_chains.size());

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

		// A half-open range of SCC positions in topological order. Reachability sets use sorted, disjoint intervals;
		// chains and tree-shaped DAG regions therefore need one interval rather than one integer per reachable SCC.
		struct IndexInterval
		{
			int begin;
			int end;
		};
		using ReachabilityIntervals = std::vector<IndexInterval>;

		// True when a canonical interval set contains one topological position.
		inline bool Contains(ReachabilityIntervals const& intervals, int position)
		{
			auto const it = std::upper_bound(intervals.begin(), intervals.end(), position, [](int value, IndexInterval const& interval)
			{
				return value < interval.begin;
			});
			return it != intervals.begin() && position < std::prev(it)->end;
		}

		// Sort and merge overlapping or adjacent intervals in place.
		inline void Canonicalize(ReachabilityIntervals& intervals)
		{
			std::sort(intervals.begin(), intervals.end(), [](IndexInterval const& lhs, IndexInterval const& rhs)
			{
				return lhs.begin != rhs.begin ? lhs.begin < rhs.begin : lhs.end < rhs.end;
			});

			auto output_count = size_t{};
			for (auto const interval : intervals)
			{
				if (output_count == 0 || interval.begin > intervals[output_count - 1].end)
				{
					intervals[output_count++] = interval;
				}
				else
				{
					intervals[output_count - 1].end = std::max(intervals[output_count - 1].end, interval.end);
				}
			}
			intervals.resize(output_count);
		}

		// Merge one canonical interval set into another. 'scratch' is reused across calls.
		inline void MergeInto(ReachabilityIntervals& destination, ReachabilityIntervals const& source, ReachabilityIntervals& scratch)
		{
			scratch.clear();
			scratch.reserve(destination.size() + source.size());
			auto Append = [&](IndexInterval interval)
			{
				if (scratch.empty() || interval.begin > scratch.back().end)
					scratch.push_back(interval);
				else
					scratch.back().end = std::max(scratch.back().end, interval.end);
			};

			auto dst = size_t{};
			auto src = size_t{};
			while (dst != destination.size() || src != source.size())
			{
				if (src == source.size() || (dst != destination.size() && destination[dst].begin <= source[src].begin))
					Append(destination[dst++]);
				else
					Append(source[src++]);
			}
			destination.swap(scratch);
		}

		// Result of trapping set detection on a contracted super-graph
		struct DetectionResult
		{
			std::vector<int> total_node_counts; // inclusive original-node count per set
			IdSetList children;                 // direct containment hierarchy
			IdSetList exclusive_super;          // SCC members owned by each set
			std::vector<int> node_owner;         // per original-node: owning set index
		};

		// Detect trapping sets on a contracted super-graph. The result is a containment hierarchy of trapping sets,
		// one per SCC reachability closure plus a synthetic full-graph root when the condensation DAG has multiple
		// sources. Compressed intervals preserve exact reachability without materialising every closure member.
		// 'report_progress' returns false to cancel; 'out_result' is unchanged unless detection completes.
		template <typename ReportProgress>
		bool DetectOnContraction(Graph const& original, ChainContraction const& contraction, ReportProgress& report_progress, DetectionResult& out_result)
		{
			auto const& sg = contraction.super_graph;
			auto const& chains = contraction.chains;

			// Tarjan's SCC on super-graph
			TarjanSCC tarjan;
			tarjan.Run(sg);

			// Build the condensation DAG and give every SCC a stable topological position.
			CondensationDAG dag(sg, tarjan.scc_id, tarjan.scc_count);
			auto const topo_order = dag.TopologicalOrder();
			std::vector<int> topo_position(tarjan.scc_count, -1);
			for (int position = 0; position != tarjan.scc_count; ++position)
				topo_position[topo_order[position]] = position;

			auto const total_work = std::max<int64_t>(2LL * tarjan.scc_count, 1);
			if (!report_progress(0, total_work))
				return false;

			// Build exact reachability closures in reverse topological order. Each successor closure is already
			// canonical, so sorting and coalescing produces a compact exact union for this SCC.
			std::vector<ReachabilityIntervals> reachable(tarjan.scc_count);
			auto completed_work = int64_t{};
			for (auto order_index = tarjan.scc_count; order_index-- != 0;)
			{
				auto const scc = topo_order[order_index];
				auto& intervals = reachable[scc];
				auto required_capacity = size_t{1};
				for (auto const child : dag.adj[scc])
					required_capacity += reachable[child].size();
				intervals.reserve(required_capacity);
				intervals.push_back({ topo_position[scc], topo_position[scc] + 1 });
				for (auto const child : dag.adj[scc])
					intervals.insert(intervals.end(), reachable[child].begin(), reachable[child].end());
				Canonicalize(intervals);

				if (!report_progress(++completed_work, total_work))
					return false;
			}

			// Remove transitive condensation edges. If an earlier outgoing neighbor reaches a later one, its closure
			// already covers that neighbor and the later edge is not a direct containment relation.
			for (auto& edges : dag.adj)
			{
				std::sort(edges.begin(), edges.end(), [&](int lhs, int rhs)
				{
					return topo_position[lhs] < topo_position[rhs];
				});
			}

			IdSetList direct_children(tarjan.scc_count);
			ReachabilityIntervals covered;
			ReachabilityIntervals merge_scratch;
			for (auto const scc : topo_order)
			{
				covered.clear();
				for (auto const child : dag.adj[scc])
				{
					if (Contains(covered, topo_position[child]))
						continue;

					direct_children[scc].push_back(child);
					MergeInto(covered, reachable[child], merge_scratch);
				}

				if (!report_progress(++completed_work, total_work))
					return false;
			}

			// Prefix original-node weights in topological order let each interval report its exact inclusive node count.
			std::vector<int> topological_node_prefix(static_cast<size_t>(tarjan.scc_count) + 1, 0);
			for (int position = 0; position != tarjan.scc_count; ++position)
			{
				auto node_count = 0;
				for (auto const super_node : dag.scc_members[topo_order[position]])
					node_count += static_cast<int>(chains[super_node].size());
				topological_node_prefix[position + 1] = topological_node_prefix[position] + node_count;
			}

			std::vector<int> total_node_counts(tarjan.scc_count, 0);
			for (int scc = 0; scc != tarjan.scc_count; ++scc)
				for (auto const interval : reachable[scc])
					total_node_counts[scc] += topological_node_prefix[interval.end] - topological_node_prefix[interval.begin];

			// A unique source SCC reaches the entire condensation DAG and is the full-graph set. Multiple sources need
			// one synthetic root whose direct children are exactly those incomparable source closures.
			std::vector<int> scc_in_degree(tarjan.scc_count, 0);
			for (int scc = 0; scc != tarjan.scc_count; ++scc)
				for (auto const child : dag.adj[scc])
					++scc_in_degree[child];

			std::vector<int> source_sccs;
			for (int scc = 0; scc != tarjan.scc_count; ++scc)
				if (scc_in_degree[scc] == 0)
					source_sccs.push_back(scc);

			auto const synthetic_root = source_sccs.size() != 1;
			auto const synthetic_root_index = tarjan.scc_count;
			auto const num_sets = tarjan.scc_count + (synthetic_root ? 1 : 0);
			IdSetList children(num_sets);
			IdSetList exclusive_super(num_sets);
			total_node_counts.resize(num_sets, original.node_count);
			for (int scc = 0; scc != tarjan.scc_count; ++scc)
			{
				children[scc] = std::move(direct_children[scc]);
				exclusive_super[scc] = std::move(dag.scc_members[scc]);
			}
			if (synthetic_root)
				children[synthetic_root_index] = source_sccs;

			// Build original-node ownership directly from SCC membership.
			std::vector<int> node_owner(original.node_count, synthetic_root ? synthetic_root_index : source_sccs.front());
			for (int scc = 0; scc != tarjan.scc_count; ++scc)
				for (auto const super_node : exclusive_super[scc])
					for (auto const node : chains[super_node])
						node_owner[node] = scc;

			// Preserve the public parent-before-child ordering, with larger sets first within each level.
			std::vector<int> in_degree(num_sets, 0);
			for (int set_index = 0; set_index != num_sets; ++set_index)
				for (auto const child : children[set_index])
					++in_degree[child];

			std::vector<int> ready;
			std::vector<int> next_ready;
			std::vector<int> set_order;
			ready.reserve(num_sets);
			next_ready.reserve(num_sets);
			set_order.reserve(num_sets);
			for (int set_index = 0; set_index != num_sets; ++set_index)
				if (in_degree[set_index] == 0)
					ready.push_back(set_index);

			while (!ready.empty())
			{
				std::sort(ready.begin(), ready.end(), [&](int lhs, int rhs)
				{
					return total_node_counts[lhs] != total_node_counts[rhs]
						? total_node_counts[lhs] > total_node_counts[rhs]
						: lhs < rhs;
				});
				next_ready.clear();
				for (auto const set_index : ready)
				{
					set_order.push_back(set_index);
					for (auto const child : children[set_index])
						if (--in_degree[child] == 0)
							next_ready.push_back(child);
				}
				ready.swap(next_ready);
			}
			assert(static_cast<int>(set_order.size()) == num_sets);

			std::vector<int> new_id(num_sets, -1);
			for (int new_index = 0; new_index != num_sets; ++new_index)
				new_id[set_order[new_index]] = new_index;

			DetectionResult result;
			result.total_node_counts.resize(num_sets);
			result.children.resize(num_sets);
			result.exclusive_super.resize(num_sets);
			result.node_owner.resize(original.node_count);
			for (int new_index = 0; new_index != num_sets; ++new_index)
			{
				auto const old_index = set_order[new_index];
				result.total_node_counts[new_index] = total_node_counts[old_index];
				result.exclusive_super[new_index] = std::move(exclusive_super[old_index]);
				result.children[new_index].reserve(children[old_index].size());
				for (auto const child : children[old_index])
					result.children[new_index].push_back(new_id[child]);
				std::sort(result.children[new_index].begin(), result.children[new_index].end());
			}
			for (int node = 0; node != original.node_count; ++node)
				result.node_owner[node] = new_id[node_owner[node]];

			if (!report_progress(total_work, total_work))
				return false;

			out_result = std::move(result);
			return true;
		}
	}

	// Detect trapping sets in a directed graph.
	//
	// Parameters:
	//   node_count     - Number of nodes in the graph (nodes are indexed 0..node_count-1)
	//   get_successors - Callable: get_successors(int node) returns a range of successor node indices
	//
	// Returns false when 'report_progress' requests cancellation. 'out_result' is unchanged unless detection completes.
	template <typename GetSuccessors, typename ReportProgress>
	bool Detect(int node_count, GetSuccessors get_successors, ReportProgress report_progress, Result& out_result)
	{
		// Build internal adjacency list from the caller's graph
		detail::Graph graph;
		graph.Resize(node_count);
		for (int v = 0; v < node_count; ++v)
			for (int w : get_successors(v))
				graph.AddEdge(v, w);

		// Pass 1: Chain contraction
		auto contraction = detail::ContractChains(graph);

		// Refine only within SCCs; SCC identity is the exact trapping-membership signature.
		detail::TarjanSCC tarjan;
		tarjan.Run(contraction.super_graph);
		contraction = detail::RefineChains(graph, contraction, tarjan.scc_id);

		// Detect trapping sets on the refined contraction
		detail::DetectionResult detection;
		if (!detail::DetectOnContraction(graph, contraction, report_progress, detection))
			return false;

		// Assemble final result
		Result result;
		int num_sets = static_cast<int>(detection.total_node_counts.size());
		result.sets.resize(num_sets);
		result.node_owner = std::move(detection.node_owner);

		int total_chain_nodes = 0;
		int chain_count = 0;

		for (int i = 0; i < num_sets; ++i)
		{
			auto& ts = result.sets[i];
			ts.id = i;
			ts.child_sets = std::move(detection.children[i]);
			ts.total_node_count = detection.total_node_counts[i];

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

		out_result = std::move(result);
		return true;
	}

	// Detect trapping sets without progress reporting or cancellation.
	template <typename GetSuccessors>
	Result Detect(int node_count, GetSuccessors get_successors)
	{
		Result result;
		auto report_progress = [](int64_t, int64_t) { return true; };
		auto const completed = Detect(node_count, std::move(get_successors), report_progress, result);
		assert(completed); (void)completed;
		return result;
	}

	// Squash chains of singly-linked trapping sets into single sets.
	//
	// A "squashable chain" is a maximal path in the containment tree where each
	// set has exactly one child. These sets are merged into a single set with
	// linear_chain=true, combining their exclusive node_chains into one set.
	//
	// This simplifies deeply nested hierarchies where each level peels off only
	// a few nodes (e.g. long funnels that narrow one node at a time). The merged
	// sets are not true trapping sets (no cycle from end to start), as indicated
	// by the linear_chain flag.
	//
	// Set 0 (the full graph) is never squashed.
	inline Result SquashChains(Result input)
	{
		auto& sets = input.sets;
		int const num_sets = static_cast<int>(sets.size());
		if (num_sets <= 1)
			return input;

		// Track whether each set has one unambiguous parent. Multiple-parent containment is a DAG branch and cannot be traversed as part of a linear chain.
		std::vector<int> parent(num_sets, -1);
		std::vector<int> parent_count(num_sets, 0);
		for (int i = 0; i < num_sets; ++i)
		{
			for (int child : sets[i].child_sets)
			{
				parent[child] = i;
				++parent_count[child];
			}
		}

		// Identify squashable chains.
		// set_chain_id[s] = index into 'chains' if set s belongs to a chain, else -1.
		std::vector<int> set_chain_id(num_sets, -1);
		NodeChains chains;

		// Start from 1 to exclude the root set
		for (int s = 1; s < num_sets; ++s)
		{
			if (set_chain_id[s] >= 0)
				continue;
			if (sets[s].child_sets.size() != 1)
				continue;

			// A chain head is a single-child set whose parent is either the root
			// (set 0), or has multiple children (so this set starts a new chain).
			int const p = parent_count[s] == 1 ? parent[s] : -1;
			if (p > 0 && sets[p].child_sets.size() == 1)
				continue; // Not a chain head; parent will start this chain

			// Walk through single-child links only while the child has no other parent.
			std::vector<int> chain;
			int cur = s;
			while (cur >= 0 && sets[cur].child_sets.size() == 1)
			{
				chain.push_back(cur);
				auto const child = sets[cur].child_sets[0];
				cur = parent_count[child] == 1 ? child : -1;
			}

			// Only squash chains of 2 or more sets
			if (chain.size() >= 2)
			{
				int cid = static_cast<int>(chains.size());
				for (int idx : chain)
					set_chain_id[idx] = cid;
				chains.push_back(std::move(chain));
			}
		}

		if (chains.empty())
			return input;

		// Build new set list. Chain heads become merged sets; chain non-heads are removed.
		Result output;
		std::vector<int> old_to_new(num_sets, -1);

		for (int s = 0; s < num_sets; ++s)
		{
			if (set_chain_id[s] >= 0)
			{
				auto const& chain = chains[set_chain_id[s]];
				if (chain[0] != s)
					continue; // Not the head; absorbed into the merged set

				TrappingSet merged;
				merged.linear_chain = true;

				// Total node count from the head (already includes all descendants)
				merged.total_node_count = sets[chain[0]].total_node_count;

				// Collect exclusive node_chains from all sets in the chain, in order
				for (int idx : chain)
					for (auto& nc : sets[idx].node_chains)
						merged.node_chains.push_back(std::move(nc));

				// The child of the last set in the chain becomes this set's child
				int last = chain.back();
				merged.child_sets = std::move(sets[last].child_sets);

				int new_idx = static_cast<int>(output.sets.size());
				merged.id = new_idx;
				for (int idx : chain)
					old_to_new[idx] = new_idx;

				output.sets.push_back(std::move(merged));
			}
			else
			{
				old_to_new[s] = static_cast<int>(output.sets.size());
				sets[s].id = static_cast<int>(output.sets.size());
				output.sets.push_back(std::move(sets[s]));
			}
		}

		// Remap child_sets to new indices
		for (auto& ts : output.sets)
			for (auto& child : ts.child_sets)
				child = old_to_new[child];

		// Remap node_owner to new set indices
		output.node_owner.resize(input.node_owner.size());
		for (size_t v = 0; v < input.node_owner.size(); ++v)
			output.node_owner[v] = old_to_new[input.node_owner[v]];

		// Recompute chain statistics
		int total_chain_nodes = 0;
		int chain_count = 0;
		for (auto const& ts : output.sets)
		{
			for (auto const& nc : ts.node_chains)
			{
				total_chain_nodes += static_cast<int>(nc.size());
				chain_count++;
			}
		}
		output.chain_count = chain_count;
		output.avg_chain_length = chain_count > 0 ? static_cast<float>(total_chain_nodes) / chain_count : 0.f;

		return output;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::trapping_sets::tests
{
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
		PRUnitTestMethod(SingleNode, Quick)
		{
			// One node, no edges → one trapping set (the full graph)
			auto adj = MakeSuccessors(1, {});
			auto result = Detect(1, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 1);
			PR_EXPECT(result.node_owner[0] == 0);
		}

		PRUnitTestMethod(LinearChain, Quick)
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

		PRUnitTestMethod(SimpleCycle, Quick)
		{
			// 0→1→2→0 (simple cycle) → one trapping set (full graph)
			auto adj = MakeSuccessors(3, {{0,1},{1,2},{2,0}});
			auto result = Detect(3, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 3);
		}

		PRUnitTestMethod(SelfLoop, Quick)
		{
			// 0→0 (self-loop) → one trapping set
			auto adj = MakeSuccessors(1, {{0,0}});
			auto result = Detect(1, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets.size() == 1);
			PR_EXPECT(result.sets[0].total_node_count == 1);
		}

		PRUnitTestMethod(ChainIntoCycle, Quick)
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

		PRUnitTestMethod(TwoDisjointCycles, Quick)
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

		PRUnitTestMethod(NestedCycles, Quick)
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

		PRUnitTestMethod(DiamondToSink, Quick)
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

		PRUnitTestMethod(NodeOwnership, Quick)
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

		PRUnitTestMethod(ChainStatistics, Quick)
		{
			// 0→1→2→3→4→2 (long chain 0→1 into cycle 2→3→4→2)
			auto adj = MakeSuccessors(5, {{0,1},{1,2},{2,3},{3,4},{4,2}});
			auto result = Detect(5, [&](int v) { return adj[v]; });

			PR_EXPECT(result.chain_count >= 1);
			PR_EXPECT(result.avg_chain_length > 0.f);

			// All 5 nodes accounted for
			PR_EXPECT(result.sets[0].total_node_count == 5);
		}

		PRUnitTestMethod(LargerGraph, Quick)
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

		PRUnitTestMethod(ContainmentHierarchy, Quick)
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

		PRUnitTestMethod(SquashChainsBasic, Quick)
		{
			// 0→1→2→3→4→5→3 (long funnel 0→1→2 into cycle 3→4→5→3)
			// The hierarchy should have: full graph → cycle {3,4,5}
			// With intermediate single-child sets along the chain, SquashChains should merge them.
			auto adj = MakeSuccessors(6, {{0,1},{1,2},{2,3},{3,4},{4,5},{5,3}});
			auto result = Detect(6, [&](int v) { return adj[v]; });
			auto squashed = SquashChains(result);

			// All nodes must still be accounted for
			PR_EXPECT(squashed.sets[0].total_node_count == 6);
			PR_EXPECT(squashed.node_owner.size() == 6);

			// Every node should have a valid owner
			for (int i = 0; i != 6; ++i)
				PR_EXPECT(squashed.node_owner[i] >= 0 && squashed.node_owner[i] < static_cast<int>(squashed.sets.size()));
		}

		PRUnitTestMethod(SquashChainsPreservesSimple, Quick)
		{
			// Simple case: 0→1→0 (single cycle, no chain to squash)
			auto adj = MakeSuccessors(2, {{0,1},{1,0}});
			auto result = Detect(2, [&](int v) { return adj[v]; });
			auto squashed = SquashChains(result);

			// Should be identical — nothing to squash
			PR_EXPECT(squashed.sets.size() == result.sets.size());
			PR_EXPECT(squashed.sets[0].total_node_count == 2);
		}

		PRUnitTestMethod(SquashChainsLinearFlag, Quick)
		{
			// Build a graph that creates a chain of single-child trapping sets.
			// Cycle A: 4→5→4 (innermost)
			// 3→4 (leads into cycle A) → trapping set {3,4,5}
			// 2→3 (leads into {3,4,5}) → trapping set {2,3,4,5}
			// 1→2 (leads into {2,3,4,5}) → trapping set {1,2,3,4,5}
			// 0→1 → full graph {0,1,2,3,4,5}
			auto adj = MakeSuccessors(6, {{0,1},{1,2},{2,3},{3,4},{4,5},{5,4}});
			Result result = Detect(6, [&](int v) { return adj[v]; });

			// Before squashing, count single-child sets
			int single_child_count = 0;
			for (int i = 0; i != static_cast<int>(result.sets.size()); ++i)
				if (result.sets[i].child_sets.size() == 1)
					single_child_count++;

			// Squash
			Result squashed = SquashChains(result);

			// After squashing, the chain of single-child sets should be merged
			// At least one set should have linear_chain=true if there were chains to squash
			if (single_child_count >= 2)
			{
				bool has_linear = false;
				for (int i = 0; i != static_cast<int>(squashed.sets.size()); ++i)
					if (squashed.sets[i].linear_chain)
						has_linear = true;
				PR_EXPECT(has_linear);
			}

			// All nodes still accounted for
			PR_EXPECT(squashed.sets[0].total_node_count == 6);

			// Chain statistics should be valid
			PR_EXPECT(squashed.chain_count >= 1);
			PR_EXPECT(squashed.avg_chain_length > 0.f);
		}

		PRUnitTestMethod(SquashChainsMultipleBranches, Quick)
		{
			// Graph with two branches, each leading to a different cycle.
			// Neither branch forms a single-child chain in the hierarchy.
			// 0→1→2→0 (cycle A), 0→3→4→3 (cycle B reachable from A)
			// 0→5→6→5 (cycle C reachable from A)
			// Trapping sets: full graph, {3,4}, {5,6}. No single-child chains.
			auto adj = MakeSuccessors(7, {{0,1},{1,2},{2,0},{0,3},{3,4},{4,3},{0,5},{5,6},{6,5}});
			auto result = Detect(7, [&](int v) { return adj[v]; });
			auto squashed = SquashChains(result);

			// No single-child chains to squash, so result should be unchanged
			PR_EXPECT(squashed.sets.size() == result.sets.size());

			// No linear_chain flags should be set
			for (auto const& ts : squashed.sets)
				PR_EXPECT(ts.linear_chain == false);
		}

		PRUnitTestMethod(SquashChainsSingleNode, Quick)
		{
			// Single node, no edges
			auto adj = MakeSuccessors(1, {});
			auto result = Detect(1, [&](int v) { return adj[v]; });
			auto squashed = SquashChains(result);

			PR_EXPECT(squashed.sets.size() == 1);
			PR_EXPECT(squashed.sets[0].total_node_count == 1);
			PR_EXPECT(squashed.node_owner[0] == 0);
		}

		PRUnitTestMethod(SquashChainsNodeOwnerConsistency, Quick)
		{
			// After squashing, every node must be owned by a valid set,
			// and the owning set must contain that node.
			auto adj = MakeSuccessors(8, {
				{0,1},{1,2},{2,3},{3,4},{4,5},{5,3}, // Chain into cycle
				{6,7},{7,6},                          // Disjoint cycle
			});
			auto result = Detect(8, [&](int v) { return adj[v]; });
			auto squashed = SquashChains(result);

			for (int v = 0; v != 8; ++v)
			{
				int owner = squashed.node_owner[v];
				PR_EXPECT(owner >= 0 && owner < static_cast<int>(squashed.sets.size()));

				// The node should appear in the owning set's node_chains
				bool found = false;
				auto nodes = AllNodes(squashed, owner);
				found = nodes.count(v) > 0;
				PR_EXPECT(found);
			}
		}

		PRUnitTestMethod(TopologicalOrdering, Quick)
		{
			// Verify that parents appear before children in the result.
			// 0→1→2→0, 2→3, 3→4→3 (cycle A contains cycle B)
			auto adj = MakeSuccessors(5, {{0,1},{1,2},{2,0},{2,3},{3,4},{4,3}});
			auto result = Detect(5, [&](int v) { return adj[v]; });

			// For every set, its children should have higher indices
			for (int i = 0; i < static_cast<int>(result.sets.size()); ++i)
				for (int child : result.sets[i].child_sets)
					PR_EXPECT(child > i);
		}

		PRUnitTestMethod(DisjointComponentsNoEdges, Quick)
		{
			// 4 isolated nodes, no edges
			auto adj = MakeSuccessors(4, {});
			auto result = Detect(4, [&](int v) { return adj[v]; });

			// Each node is its own trivial trapping set (singleton, no outgoing edges)
			// Plus the full graph → at least 5 sets (or fewer if singletons are merged)
			PR_EXPECT(result.sets[0].total_node_count == 4);

			// Every node has a valid owner
			for (int i = 0; i != 4; ++i)
				PR_EXPECT(result.node_owner[i] >= 0 && result.node_owner[i] < static_cast<int>(result.sets.size()));
		}

		PRUnitTestMethod(SinkNodeTrappingSet, Quick)
		{
			// 0→1→2, 0→3→2, 2 is a sink node (no outgoing edges)
			// Node 2 forms a trivial trapping set (once there, you can't leave)
			auto adj = MakeSuccessors(4, {{0,1},{1,2},{0,3},{3,2}});
			auto result = Detect(4, [&](int v) { return adj[v]; });

			// Should find node 2 as a trapping sub-set
			bool found_sink = false;
			for (int i = 1; i < static_cast<int>(result.sets.size()); ++i)
			{
				auto nodes = AllNodes(result, i);
				if (nodes.size() == 1 && nodes.count(2))
				{
					found_sink = true;
					break;
				}
			}
			PR_EXPECT(found_sink);
		}

		PRUnitTestMethod(LongChainContraction, Quick)
		{
			// A very long linear chain: 0→1→2→...→19→20→20 (self-loop at end)
			// Chain contraction should merge the linear portion efficiently
			int const N = 21;
			std::vector<std::pair<int,int>> edges;
			for (int i = 0; i != N - 1; ++i)
				edges.push_back({i, i + 1});
			edges.push_back({N - 1, N - 1}); // self-loop at end

			auto adj = MakeSuccessors(N, edges);
			auto result = Detect(N, [&](int v) { return adj[v]; });

			PR_EXPECT(result.sets[0].total_node_count == N);

			// The self-loop node should be in a trapping sub-set
			bool found_loop = false;
			for (int i = 1; i < static_cast<int>(result.sets.size()); ++i)
			{
				auto nodes = AllNodes(result, i);
				if (nodes.count(N - 1))
				{
					found_loop = true;
					break;
				}
			}
			PR_EXPECT(found_loop);

			// Chain statistics: most nodes should be in chains
			PR_EXPECT(result.avg_chain_length > 1.f);
		}

		PRUnitTestMethod(SquashChainsDeepNesting, Quick)
		{
			// Create a deeply nested containment hierarchy:
			// 0→1→2→3→4→5→6→7, 7→8→7 (cycle at the end)
			// Each prefix creates a nested trapping set.
			// SquashChains should collapse the single-child chain.
			auto adj = MakeSuccessors(9, {
				{0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,8},{8,7}
			});
			auto result = Detect(9, [&](int v) { return adj[v]; });
			int original_count = static_cast<int>(result.sets.size());

			auto squashed = SquashChains(result);

			// Squashing should reduce the number of sets
			PR_EXPECT(static_cast<int>(squashed.sets.size()) <= original_count);

			// All 9 nodes still accounted for
			PR_EXPECT(squashed.sets[0].total_node_count == 9);

			// Verify node owners are still valid
			for (int v = 0; v != 9; ++v)
			{
				int owner = squashed.node_owner[v];
				PR_EXPECT(owner >= 0 && owner < static_cast<int>(squashed.sets.size()));
			}
		}
	};
}
#endif
