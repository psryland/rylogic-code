#pragma once
//*********************************************
// Physics Sandbox - Per-Body History Logger
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Records per-frame state for all bodies in a compact ring buffer.
// When a body is flagged for investigation (e.g., falls below ground),
// the ring buffer history is dumped to the log file, then detailed
// per-frame logging continues for that body.
#pragma once
#include <cstdio>
#include <vector>
#include <unordered_set>
#include "pr/math/math.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/shape/inertia.h"

namespace physics_sandbox
{
	struct BodyHistory
	{
		// Compact per-body snapshot for the ring buffer
		struct Snapshot
		{
			pr::v4 pos;
			pr::v4 vel_lin;
			float inv_mass;
		};

		FILE* m_file;
		std::vector<std::vector<Snapshot>> m_ring; // [body_idx][ring_pos]
		std::unordered_set<int> m_tracked;         // bodies flagged for detailed logging
		int m_ring_size;
		int m_ring_head;
		int m_body_count;
		int m_max_frames;

		BodyHistory()
			: m_file(nullptr)
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

		void Open(char const* filepath, int body_count, int ring_size = 120, int max_frames = 50000)
		{
			Close();
			m_file = fopen(filepath, "w");
			m_body_count = body_count;
			m_ring_size = ring_size;
			m_ring_head = 0;
			m_max_frames = max_frames;
			m_tracked.clear();
			m_ring.resize(body_count);
			for (auto& ring : m_ring)
				ring.resize(ring_size, Snapshot{});
		}

		void Close()
		{
			if (m_file)
			{
				fclose(m_file);
				m_file = nullptr;
			}

			m_ring.clear();
			m_tracked.clear();
		}

		bool IsActive(int frame) const
		{
			return m_file != nullptr && frame < m_max_frames;
		}
		bool IsTracked(int body_idx) const
		{
			return m_tracked.contains(body_idx);
		}

		// Record a snapshot for every body into the ring buffer.
		// Logs detailed state for tracked bodies.
		template <typename BodyRange>
		void RecordFrame(int frame, BodyRange const& bodies)
		{
			if (!m_file) return;

			int idx = 0;
			for (auto const& body : bodies)
			{
				if (idx >= m_body_count) break;

				auto pos = body.O2W().w;
				auto inv_mass = body.InvMass();
				auto vel_lin = inv_mass * body.MomentumWS().lin;

				m_ring[idx][m_ring_head % m_ring_size] = Snapshot{ pos, vel_lin, inv_mass };

				if (IsTracked(idx) && frame < m_max_frames)
				{
					fprintf(m_file, "[%d] body[%d]: pos=(%.4f,%.4f,%.4f) vel=(%.4f,%.4f,%.4f)\n",
					frame, idx, pos.x, pos.y, pos.z, vel_lin.x, vel_lin.y, vel_lin.z);
				}
				++idx;
			}
			m_ring_head++;
		}

		// Check all bodies and flag any that have fallen below the ground.
		// When a new body is flagged, dump its ring buffer history.
		template <typename BodyRange>
		void CheckForFallenBodies(int frame, BodyRange const& bodies, float ground_z)
		{
			if (!m_file) return;

			int idx = 0;
			for (auto const& body : bodies)
			{
				if (idx >= m_body_count) break;
				if (body.Mass() >= pr::physics::InfiniteMass * 0.5f || IsTracked(idx)) { ++idx; continue; }

				auto pos = body.O2W().pos;
				auto half_ext = body.Shape().m_bbox.m_radius;
				auto lowest_z = pos.z - half_ext.z;

				if (lowest_z < ground_z - 0.5f)
				{
					m_tracked.insert(idx);
					fprintf(m_file, "\n*** body[%d] FELL BELOW GROUND at frame %d (z=%.4f, lowest=%.4f) ***\n", idx, frame, pos.z, lowest_z);
					fprintf(m_file, "--- Ring buffer history (most recent %d frames) ---\n", m_ring_size);

					auto count = std::min(m_ring_head, m_ring_size);
					auto oldest = (m_ring_head - count + m_ring_size) % m_ring_size;
					for (int i = 0; i != count; ++i)
					{
						auto ring_idx = (oldest + i) % m_ring_size;
						auto& s = m_ring[idx][ring_idx];
						auto history_frame = frame - count + i + 1;
						fprintf(m_file, "  [%d] pos=(%.4f,%.4f,%.4f) vel=(%.4f,%.4f,%.4f)\n",
						history_frame, s.pos.x, s.pos.y, s.pos.z, s.vel_lin.x, s.vel_lin.y, s.vel_lin.z);
					}
					fprintf(m_file, "--- End history ---\n\n");
					fflush(m_file);
				}
				++idx;
			}
		}

		void Flush()
		{
			if (m_file) fflush(m_file);
		}
	};
}
