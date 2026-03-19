//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// Pipeline logger for the GPU physics engine.
// Reads back GPU state at each pipeline stage and logs it to a file.
// Designed for diagnosing collision/resolution bugs by tracing data
// through Integrate → Broadphase → NarrowPhase → Resolve.
//
// Usage:
//   PhysicsLog log;
//   log.Open("dump/physics_pipeline.log");
//   // Each frame, between pipeline stages:
//   log.LogIntegrate(frame, bodies, aabbs, dt);
//   log.LogBroadphase(frame, pairs);
//   log.LogNarrowPhase(frame, contacts, diag);
//   log.EndFrame();
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	struct PhysicsLog
	{
		std::ofstream m_file;
		int m_frame;
		int m_max_frames;
		int m_max_items; // max items to log per category (pairs, contacts, etc.)

		PhysicsLog()
			: m_file()
			, m_frame(0)
			, m_max_frames(200)
			, m_max_items(20)
		{
		}

		void Open(std::filesystem::path filepath, int max_frames = 200, int max_items = 20)
		{
			Close();
			m_file.open(filepath, std::ios::out | std::ios::trunc);
			m_frame = 0;
			m_max_frames = max_frames;
			m_max_items = max_items;
		}

		void Close()
		{
			if (m_file.is_open())
				m_file.close();
		}

		bool IsActive() const
		{
			return m_file.is_open() && m_frame < m_max_frames;
		}

		void LogIntegrate(int body_count, float dt, std::span<GpuRigidBody const> bodies, std::span<BBox const> aabbs)
		{
			if (!IsActive())
				return;

			Write(std::format("=== Frame {}: {} bodies, dt={:.4f} ===\n", m_frame, body_count, dt));
			auto n = std::min(static_cast<int>(bodies.size()), body_count);
			for (int i = 0; i != n; ++i)
			{
				auto& b = bodies[i];
				auto pos = b.o2w.w;
				auto vel = b.os_com_and_invmass.w * b.momentum_lin;
				Write(std::format("  body[{}]: pos=({:.3f},{:.3f},{:.3f}) vel=({:.3f},{:.3f},{:.3f}) inv_mass={:.6f} shape={}\n",
					i, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, b.os_com_and_invmass.w, b.shape_id));
				if (i < static_cast<int>(aabbs.size()))
				{
					Write(std::format("           aabb=({:.3f},{:.3f},{:.3f})-({:.3f},{:.3f},{:.3f})\n",
						aabbs[i].Lower().x, aabbs[i].Lower().y, aabbs[i].Lower().z,
						aabbs[i].Upper().x, aabbs[i].Upper().y, aabbs[i].Upper().z));
				}
			}
		}

		void LogBroadphase(std::span<GpuCollisionPair const> pairs)
		{
			if (!IsActive())
				return;

			Write(std::format("  Broadphase: {} pairs\n", pairs.size()));
			auto n = std::min(static_cast<int>(pairs.size()), m_max_items);
			for (int i = 0; i != n; ++i)
			{
				auto& p = pairs[i];
				Write(std::format("    pair[{}]: body({},{}) shape({},{}) b2a_pos=({:.3f},{:.3f},{:.3f})\n",
					i, p.body_idx_a, p.body_idx_b, p.shape_idx_a, p.shape_idx_b,
					p.b2a.w.x, p.b2a.w.y, p.b2a.w.z));
			}
		}

		void LogNarrowPhase(std::span<GpuResolveContact const> contacts, std::span<GpuPairDiag const> diag)
		{
			if (!IsActive())
				return;

			Write(std::format("  NarrowPhase: {} contacts, {} diag\n", contacts.size(), diag.size()));
			auto n_diag = std::min(static_cast<int>(diag.size()), m_max_items);
			for (int i = 0; i != n_diag; ++i)
			{
				auto& d = diag[i];
				Write(std::format("    diag[{}]: body({},{}) shape({},{}) gjk={} epa={} hit={}\n",
					i, d.body_idx_a, d.body_idx_b, d.shape_type_a, d.shape_type_b,
					d.gjk_iters, d.epa_iters, d.hit));
			}
			auto n_contacts = std::min(static_cast<int>(contacts.size()), m_max_items);
			for (int i = 0; i != n_contacts; ++i)
			{
				auto& c = contacts[i];
				Write(std::format("    contact[{}]: body({},{}) axis=({:.4f},{:.4f},{:.4f}) pt=({:.4f},{:.4f},{:.4f}) depth={:.6f}\n",
					i, c.body_idx_a, c.body_idx_b,
					c.axis.x, c.axis.y, c.axis.z,
					c.contact_point.x, c.contact_point.y, c.contact_point.z, c.depth));
			}
		}

		void EndFrame()
		{
			if (!IsActive())
				return;

			Write("\n");
			m_file.flush();
			++m_frame;
		}

	private:

		void Write(std::string_view text)
		{
			m_file.write(text.data(), text.size());
		}
	};
}
