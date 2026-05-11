//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/shape/inertia.h"

namespace pr::physics
{
	struct BodyHistory
	{
		// Notes:
		//  - Records per-frame state for all bodies in a compact ring buffer. When a body is flagged for investigation
		//    (e.g., falls below ground), the ring buffer history is dumped to the log file, then detailed per-frame logging
		//    continues for that body.
		inline static constexpr int Length = 60; // number of frames to keep in the ring buffer (e.g., 1 second at 60fps)

		// A ring buffer of snapshots for a single body,
		using History = RigidBody[Length];
		using BodyMap = std::unordered_map<int, History>;
		using Tracked = std::unordered_set<int>;

		std::ofstream m_file;
		BodyMap m_bodies;
		Tracked m_tracked;
		int m_frame;

		BodyHistory()
			: m_file()
			, m_bodies()
			, m_tracked()
			, m_frame()
		{
		}

		void Reset()
		{
			if (m_file.is_open())
				m_file.close();

			m_bodies.clear();
			m_tracked.clear();
			m_frame = 0;
		}

		// Snapshot the state of each body at the start of the frame.
		void BeginFrame(std::span<RigidBody* const> bodies)
		{
			++m_frame;
			for (auto& body : bodies)
			{
				auto idx = static_cast<int>(&body - bodies.data());
				m_bodies[idx][m_frame % Length] = *body;
			}

			//if (!m_file.is_open())
			//	return;
			//if (IsActive(0))
			//{
			//	auto msg = std::format("\n=== Begin frame {} with {} bodies ===\n", 0, bodies.size());
			//	m_file.write(msg.data(), msg.size());
			//}
		}

		// Called at the end of a frame to detect anomolous bodies (e.g., fallen below ground) and log their history.
		void EndFrame(std::span<RigidBody* const> bodies, std::invocable<RigidBody const&, RigidBody const&> auto&& anomolous)
		{
			auto tracking = IsTracking();
			for (auto& body : bodies)
			{
				auto idx = static_cast<int>(&body - bodies.data());
				
				// Compare before/after snapshots to look for anomolies.
				auto const& rb0 = m_bodies[idx][m_frame % Length];
				if (anomolous(rb0, *body))
					m_tracked.insert(idx);
				else
					m_tracked.erase(idx);
			}
			if (!IsTracking())
				return;

			// Open the history file and log bodie states
			if (!tracking)
			{
				//if (!m_file.is_open())
				//	m_file.open(filepath, std::ios::out | std::ios::trunc);

				//auto msg = std::format("\n=== End frame {}: {} bodies flagged for tracking ===\n", m_frame, m_tracked.size());
				//m_file.write(msg.data(), msg.size());
			}

			if (m_tracked.empty())
				return;

				
				//	return;
			//if (IsActive(0))
			//{
			//	auto msg = std::format("=== End frame {} ===\n", 0);
			//	m_file.write(msg.data(), msg.size());
			//}
		}

		// True if we're tracking any bodies
		bool IsTracking() const
		{
			return !m_tracked.empty();
		}

/*
		// True if the history is active (log file open and within frame limit).
		bool IsActive(int frame) const
		{
			return m_file.is_open() && frame < m_max_frames;
		}

		// True if the given body index is flagged for detailed logging.
		bool IsTracked(int body_idx) const
		{
			return m_tracked.contains(body_idx);
		}

		// Record a snapshot for every body into the ring buffer. Logs detailed state for tracked bodies.
		void RecordFrame(int frame, RigidBodyRange auto&& bodies)
		{
			if (!m_file.is_open())
				return;

			int idx = 0;
			for (auto const& body : bodies)
			{
				if (idx >= m_body_count)
					break;

				auto pos = body.O2W().w;
				auto inv_mass = body.InvMass();
				auto vel_lin = inv_mass * body.MomentumWS().lin;

				m_ring[idx][m_ring_head % m_ring_size] = Snapshot{ pos, vel_lin, inv_mass };

				if (IsTracked(idx) && frame < m_max_frames)
				{
					auto msg = std::format("[{}] body[{}]: pos=({:.4f},{:.4f},{:.4f}) vel=({:.4f},{:.4f},{:.4f})\n",
						frame, idx, pos.x, pos.y, pos.z, vel_lin.x, vel_lin.y, vel_lin.z);
					m_file.write(msg.data(), msg.size());
				}
				++idx;
			}
			m_ring_head++;
		}

		// Check all bodies and flag any that have fallen below the ground.
		// When a new body is flagged, dump its ring buffer history.
		void CheckForFallenBodies(int frame, RigidBodyRange auto&& bodies, float ground_z)
		{
			if (!m_file.is_open())
				return;

			int idx = 0;
			for (auto const& body : bodies)
			{
				if (idx >= m_body_count) break;
				if (body.Mass() >= InfiniteMass * 0.5f || IsTracked(idx))
				{
					++idx;
					continue;
				}

				auto pos = body.O2W().pos;
				auto half_ext = body.Shape().m_bbox.m_radius;
				auto lowest_z = pos.z - half_ext.z;

				if (lowest_z < ground_z - 0.5f)
				{
					m_tracked.insert(idx);

					auto header = std::format("\n*** body[{}] FELL BELOW GROUND at frame {} (z={:.4f}, lowest={:.4f}) ***\n", idx, frame, pos.z, lowest_z);
					m_file.write(header.data(), header.size());

					auto ring_header = std::format("--- Ring buffer history (most recent {} frames) ---\n", m_ring_size);
					m_file.write(ring_header.data(), ring_header.size());

					auto count = std::min(m_ring_head, m_ring_size);
					auto oldest = (m_ring_head - count + m_ring_size) % m_ring_size;
					for (int i = 0; i != count; ++i)
					{
						auto ring_idx = (oldest + i) % m_ring_size;
						auto& s = m_ring[idx][ring_idx];
						auto history_frame = frame - count + i + 1;
						auto line = std::format("  [{}] pos=({:.4f},{:.4f},{:.4f}) vel=({:.4f},{:.4f},{:.4f})\n",
							history_frame, s.pos.x, s.pos.y, s.pos.z, s.vel_lin.x, s.vel_lin.y, s.vel_lin.z);
						m_file.write(line.data(), line.size());
					}

					auto footer = std::string("--- End history ---\n\n");
					m_file.write(footer.data(), footer.size());
					m_file.flush();
				}
				++idx;
			}
		}
		*/

		// Flush the log file.
		void Flush()
		{
			if (m_file.is_open())
				m_file.flush();
		}
	};
}
