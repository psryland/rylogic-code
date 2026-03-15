//*********************************************
// Physics Sandbox — GPU Sort-and-Sweep Broadphase
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/collision/gpu_sort_and_sweep.h"
#include "pr/physics-2/rigid_body/rigid_body.h"
#include "pr/physics-2/rigid_body/rigid_body_dynamics.h"

namespace pr::physics
{
	GpuSortAndSweep::GpuSortAndSweep(Gpu& gpu)
		: m_gpu(gpu)
		, m_sorter(gpu.m_gpu)
		, m_keys()
		, m_payloads()
		, m_entity()
	{}

	// Remove all registered bodies
	void GpuSortAndSweep::Clear()
	{
		m_entity.resize(0);
	}

	// Register a body for overlap testing
	void GpuSortAndSweep::Add(RigidBody const& obj)
	{
		m_entity.push_back(&obj);
	}

	// Unregister a body
	void GpuSortAndSweep::Remove(RigidBody const& obj)
	{
		auto at = std::find(std::begin(m_entity), std::end(m_entity), &obj);
		if (at == std::end(m_entity)) return;
		m_entity.erase(at);
	}

	// Enumerate overlapping pairs using pre-computed world-space AABBs from the GPU integrate step.
	void GpuSortAndSweep::EnumOverlappingPairs(GpuJob& job, std::span<IntegrateAABB const> aabbs, std::function<void(RigidBody const&, RigidBody const&)> cb) const
	{
		// TODO: 'aabbs' is not matched up with 'm_entity' in any way.
		// Need to think about how this might work when broadphase is entirely on the gpu... not now though.
		// I think ultimately, the best option is to not expose the Broadphase in the API. All physical objects need collision detection
		// otherwise, they're not physics objects... although neighbouring links in a multi-body might be a special case.  That can be handled
		// by a collision group bit-mask at some point

		assert(m_entity.size() == aabbs.size() && "There is an assumption that all bodies are registered with the boardphase and the order is the same");

		//auto n = static_cast<int>(m_entity.size());
		auto n = static_cast<int>(aabbs.size());
		if (n < 2)
			return;

		// Measure the variance of body positions along each axis to choose the best primary sort axis.
		auto var_sq = v4::Zero();
		auto var_sum = v4::Zero();
		for (auto const& aabb : aabbs)
		{
			auto c = aabb.ws_bbox.m_centre;
			var_sum += c;
			var_sq += c * c;
		}

		// Variance = E[x²] - E[x]² per axis
		auto inv_n = 1.0f / n;
		auto mean = var_sum * inv_n;
		auto variance = var_sq * inv_n - mean * mean;

		// Choose axis with maximum variance
		int axis = 0; // primary axis
		if (variance.y > variance.x) axis = 1;
		if (variance.z > variance[axis]) axis = 2;

		// Pack AABB endpoints for the chosen axis
		// 'keys' is the value to sort on, 'payloads' is the buffer that is actually sorted
		auto endpoint_count = n * 2;
		m_keys.resize(endpoint_count);
		m_payloads.resize(endpoint_count);
		for (auto const& [aabb, i] : with_index(aabbs))
		{
			m_keys[i * 2 + 0] = aabb.ws_bbox.m_centre[axis] - aabb.ws_bbox.m_radius[axis];
			m_keys[i * 2 + 1] = aabb.ws_bbox.m_centre[axis] + aabb.ws_bbox.m_radius[axis];

			// Add the bbox index with the bound type marker (begin/end)
			m_payloads[i * 2 + 0] = static_cast<uint32_t>(i << 1) | 0; // begin
			m_payloads[i * 2 + 1] = static_cast<uint32_t>(i << 1) | 1; // end
		}

		// GPU radix sort by primary axis coordinate
		{
			m_sorter.Resize(job.m_cmd_list, endpoint_count);
			m_sorter.Sort(m_keys, m_payloads, job);
		}

		m_sweep.reserve(n);
		m_sweep.resize(0);

		// CPU sweep with full 3-axis AABB filtering
		for (int i = 0; i != endpoint_count; ++i)
		{
			auto payload = m_payloads[i];
			auto body_idx = static_cast<int>(payload >> 1);
			auto is_end = (payload & 1) != 0;

			// "end" means the body is leaving the active set,
			if (is_end)
			{
				// Unstable erase
				auto at = std::find(begin(m_sweep), end(m_sweep), body_idx);
				if (at != end(m_sweep))
				{
					*at = m_sweep.back();
					m_sweep.pop_back();
				}
			}

			// Compare this aabb with those in the active sweep set.
			else
			{
				auto const& aabb = aabbs[body_idx];
				for (auto other_idx : m_sweep)
				{
					auto const& other = aabbs[other_idx];
				
					// This actually retests the primary axis, but it's a very cheap test compared to the GPU sort and it keeps the code simpler.
					if (!IsIntersection(aabb.ws_bbox, other.ws_bbox))
						continue;

					// This is a weak assumption, that index in 'aabbbs' corresponds to index in 'm_entity'!
					cb(*m_entity[body_idx], *m_entity[other_idx]);
				}

				m_sweep.push_back(body_idx);
			}
		}
	}

	// Custom deleter implementation (GpuIntegrator is complete here)
	void Deleter<GpuSortAndSweep>::operator()(GpuSortAndSweep* p) const
	{
		delete p;
	}
}



	#if 0
	// Enumerate all pairs of bodies whose bounding boxes overlap.
	// Uses GPU radix sort for the primary axis, then CPU sweep with Y/Z filtering.
	void GpuSortAndSweep::EnumOverlappingPairs(GpuJob& job, std::function<void(RigidBody const&, RigidBody const&)> cb) const
	{
		auto n = static_cast<int>(m_entity.size());
		if (n < 2) return;

		// Step 1: Compute world-space AABBs and choose the primary sort axis.
		// The axis with the largest position variance gives the best separation.
		m_bboxes.resize(n);
		auto var_sum = v4::Zero();
		auto var_sq = v4::Zero();
		for (int i = 0; i != n; ++i)
		{
			if (!m_entity[i]->HasShape())
			{
				// Body has no shape — give it a degenerate bbox at its position
				m_bboxes[i] = BBox(m_entity[i]->O2W().pos, v4::Zero());
				continue;
			}
			m_bboxes[i] = m_entity[i]->BBoxWS();
			auto c = m_bboxes[i].Centre();
			var_sum += c;
			var_sq += v4(c.x * c.x, c.y * c.y, c.z * c.z, 0);
		}

		// Variance = E[x²] - E[x]² per axis
		auto inv_n = 1.0f / n;
		auto mean = var_sum * inv_n;
		auto variance = var_sq * inv_n - v4(mean.x * mean.x, mean.y * mean.y, mean.z * mean.z, 0);

		// Choose axis with maximum variance
		int axis = 0;
		if (variance.y > variance.x) axis = 1;
		if (variance.z > variance[axis]) axis = 2;

		// Step 2: Pack AABB endpoints for the chosen axis.
		// Each body produces two entries: a "begin" (min) and "end" (max).
		// Also precompute the secondary axis bounds for the CPU sweep inner loop.
		auto endpoint_count = n * 2;
		m_keys.resize(endpoint_count);
		m_payloads.resize(endpoint_count);
		m_axis_bounds.resize(n);

		int axis_y = (axis + 1) % 3;
		int axis_z = (axis + 2) % 3;

		for (int i = 0; i != n; ++i)
		{
			auto& bb = m_bboxes[i];
			auto lo = bb.Centre()[axis] - bb.Radius()[axis];
			auto hi = bb.Centre()[axis] + bb.Radius()[axis];

			m_keys[i * 2 + 0] = lo;
			m_keys[i * 2 + 1] = hi;
			m_payloads[i * 2 + 0] = static_cast<uint32_t>(i << 1) | 0; // begin
			m_payloads[i * 2 + 1] = static_cast<uint32_t>(i << 1) | 1; // end

			// Precompute secondary axis bounds to avoid repeated Centre/Radius lookups in sweep
			m_axis_bounds[i] = {
				bb.Centre()[axis_y] - bb.Radius()[axis_y],
				bb.Centre()[axis_y] + bb.Radius()[axis_y],
				bb.Centre()[axis_z] - bb.Radius()[axis_z],
				bb.Centre()[axis_z] + bb.Radius()[axis_z],
			};
		}

		// Step 3: GPU radix sort by primary axis coordinate.
		{
			m_sorter.Resize(job.m_cmd_list, endpoint_count);
			m_sorter.Sort(m_keys, m_payloads, job);
		}

		// Step 4: CPU sweep with full 3-axis AABB filtering.
		// Walk the sorted endpoints. "Begin" markers add to the active set,
		// "end" markers remove. For each new "begin", test Y and Z overlap
		// against all currently active bodies.
		auto active = std::vector<int>();
		active.reserve(n);

		for (int i = 0; i != endpoint_count; ++i)
		{
			auto payload = m_payloads[i];
			auto body_idx = static_cast<int>(payload >> 1);
			auto is_end = (payload & 1) != 0;

			if (is_end)
			{
				// Remove from active set (swap-with-last for O(1))
				auto at = std::find(active.begin(), active.end(), body_idx);
				if (at != active.end())
				{
					*at = active.back();
					active.pop_back();
				}
			}
			else
			{
				// "Begin" — test against all active bodies on the remaining two axes
				auto const& new_ab = m_axis_bounds[body_idx];

				for (auto active_idx : active)
				{
					auto const& act_ab = m_axis_bounds[active_idx];

					// Y-axis overlap test
					if (new_ab.lo_y > act_ab.hi_y || new_ab.hi_y < act_ab.lo_y)
						continue;

					// Z-axis overlap test
					if (new_ab.lo_z > act_ab.hi_z || new_ab.hi_z < act_ab.lo_z)
						continue;

					// Full 3-axis AABB overlap — emit pair
					cb(*m_entity[active_idx], *m_entity[body_idx]);
				}

				active.push_back(body_idx);
			}
		}
	}
	#endif
