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

		// Compact per-body snapshot for the ring buffer
		struct Snapshot
		{
			v4 pos;
			v4 vel_lin;
			float inv_mass;
			float pad0;
			float pad1;
			float pad2;
		};

		std::ofstream m_file;
		std::vector<std::vector<Snapshot>> m_ring; // [body_idx][ring_pos]
		std::unordered_set<int> m_tracked;         // bodies flagged for detailed logging
		int m_ring_size;
		int m_ring_head;
		int m_body_count;
		int m_max_frames;

		BodyHistory()
			: m_file()
			, m_ring()
			, m_tracked()
			, m_ring_size(0)
			, m_ring_head(0)
			, m_body_count(0)
			, m_max_frames(0)
		{
		}
		~BodyHistory()
		{
			Close();
		}

		// Open a log file and initialize the ring buffer for the given number of bodies.
		void Open(std::filesystem::path filepath, int body_count, int ring_size = 120, int max_frames = 50000)
		{
			Close();
			m_file.open(filepath, std::ios::out | std::ios::trunc);
			m_body_count = body_count;
			m_ring_size = ring_size;
			m_ring_head = 0;
			m_max_frames = max_frames;
			m_tracked.clear();
			m_ring.resize(body_count);
			for (auto& ring : m_ring)
				ring.resize(ring_size, Snapshot{});
		}

		// Close the log file and clear all data.
		void Close()
		{
			if (m_file.is_open())
				m_file.close();

			m_ring.clear();
			m_tracked.clear();
		}

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

		// Flush the log file.
		void Flush()
		{
			if (m_file.is_open())
				m_file.flush();
		}
	};
}
