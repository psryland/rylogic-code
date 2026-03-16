#pragma once
#include "src/forward.h"
#include "src/idemo_scene.h"

namespace pr::fluid
{
	struct Tube2d : IDemoScene
	{
		std::vector<fluid::Particle> m_particles;
		std::vector<fluid::Dynamics> m_dynamics;
		CollisionBuilder m_col;
		ldraw::Builder m_ldr;

		inline static constexpr int ParticleCount = 10000;
		explicit Tube2d(int particle_capacity, float particle_radius)
			: m_col()
			, m_ldr()
			, m_particles(std::min(ParticleCount, particle_capacity))
			, m_dynamics(std::min(ParticleCount, particle_capacity))
		{
			// Floor
			m_ldr.Plane("floor", 0xFFade3ff).wh({ 2*particle_radius, particle_radius }).o2w(m4x4::Transform(Rotation<m3x4>(AxisId::PosZ, AxisId::PosY), v4{ 0, -0.5, 0, 1 }));
			m_col.Plane().o2w(m3x4::Rotation(AxisId::PosZ, AxisId::PosY), v4{ 0, -0.5, 0, 1 });

			// Ceiling
			m_ldr.Plane("ceiling", 0xFFade3ff).wh({ 2*particle_radius, particle_radius }).o2w(m4x4::Transform(Rotation<m3x4>(AxisId::PosZ, AxisId::NegY), v4{ 0, +0.5, 0, 1 }));
			m_col.Plane().o2w(m3x4::Rotation(AxisId::PosZ, AxisId::NegY), v4{ 0, +0.5, 0, 1 });

			// Left Wall
			m_ldr.Plane("left_wall", 0xFFade3ff).wh({ particle_radius, 1 }).o2w(m4x4::Transform(Rotation<m3x4>(AxisId::PosZ, AxisId::PosX), v4{ -particle_radius, 0, 0, 1 }));
			m_col.Plane().o2w(m3x4::Rotation(AxisId::PosZ, AxisId::PosX), v4{ -particle_radius, 0, 0, 1 });

			// Right Wall
			m_ldr.Plane("right_wall", 0xFFade3ff).wh({ particle_radius, 1 }).o2w(m4x4::Transform(Rotation<m3x4>(AxisId::PosZ, AxisId::NegX), v4{ +particle_radius, 0, 0, 1 }));
			m_col.Plane().o2w(m3x4::Rotation(AxisId::PosZ, AxisId::NegX), v4{ +particle_radius, 0, 0, 1 });

			//m_ldr.Plane("cull_plane", 0x80FF0000).wh({ 2,0.5f }).o2w(m3x4::Rotation(AxisId::NegZ, AxisId::PosY), v4{ 0, -0.95f, 0, 1 });
			m_ldr.WrapAsGroup();

			{
				auto& particles = m_particles;
				auto& dynamics = m_dynamics;

				assert(particles.size() == dynamics.size());
				int idx = 0, count = isize(particles);
				auto points = [&](v4 p, v4 v)
				{
					assert(p.w == 1 && v.w == 0);
					if (idx >= isize(particles)) return;
					particles[idx] = fluid::Particle{
						.pos = p,
						.col = v4::One(),
					};
					dynamics[idx] = fluid::Dynamics{
						.vel = v,
						.accel = v4::Zero(),
						.surface = v4{0, 0, 0, particle_radius},
					};
					++idx;
				};

				const float hwidth = particle_radius;
				const float hheight = 0.25f;

				if constexpr (false)
				{
					points(v4(          0, -0.25f, 0, 1), v4::Zero());
					points(v4(          0, -0.35f, 0, 1), v4::Zero());
					points(v4(          0, -0.45f, 0, 1), v4::Zero());
					points(v4(-hwidth*3/4, -0.00f, 0, 1), v4::Zero());
					points(v4(+hwidth*3/4, -0.00f, 0, 1), v4::Zero());
					points(v4(-hwidth*3/4, -0.45f, 0, 1), v4::Zero());
					points(v4(+hwidth*3/4, -0.45f, 0, 1), v4::Zero());
				}
				else
				{
					auto const margin = 0.95f;
					auto hw = hwidth * margin;
					auto hh = hheight * margin;

					// Want to spread N particles evenly over the volume.
					// Area is 2*hwidth * 2*hheight
					// Want to find 'step' such that:
					//   (2*hwidth / step) * (2*hheight / step) = N
					// => step = sqrt(4 * hwidth * hheight / N)
					auto step = Sqrt(4 * hw * hh / count);

					auto x = -hw + step / 2;
					auto y = -hh + step / 2;
					for (int i = 0; i != count; ++i)
					{
						points(v4(x, y, 0, 1), v4::Zero());

						x += step;
						if (x > hw) { x = -hw + step / 2; y += step; }
					}
				}
			}
		}

		// 2D or 3D
		int SpatialDimensions() const override
		{
			return 2;
		}

		// Initial camera position
		void Camera(pr::Camera& camera) const override
		{
			camera.LookAt(v4(0.0f, 0.0f, 2.8f, 1), v4(0, 0.0f, 0, 1), v4(0, 1, 0, 0));
			camera.Align(v4::YAxis());
		}

		// Return the visualisation scene
		std::string LdrScene() const override
		{
			return m_ldr.ToString();
		}

		// Returns initialisation data for the particle positions.
		std::span<fluid::Particle const> Particles() const override
		{
			return m_particles;
		}

		// Returns initialisation data for the particle dynamics.
		std::span<fluid::Dynamics const> Dynamics() const override
		{
			return m_dynamics;
		}

		// Return the collision
		std::span<CollisionPrim const> Collision() const override
		{
			return m_col.Primitives();
		}
		
		// Particle culling
		ParticleCollision::CullData Culling() const override
		{
			return ParticleCollision::CullData{
				.Geom = { v4(0, 1, 0, 0.95f), v4::Zero() },
				.Mode = ParticleCollision::ECullMode::None,
			};
		}

		// Move the probe around
		v4 PositionProbe(gui::Point ss_pt, rdr12::Scene const& scn) const override
		{
			// Set the probe position from a SS point
			// Shoot a ray through the mouse pointer
			auto nss_point = scn.m_viewport.SSPointToNSSPoint(To<v2>(ss_pt));
			auto [pt, dir] = scn.m_cam.NSSPointToWSRay(v4(nss_point, 1, 0));

			// Find where it intersects the XY plane at z = 0
			auto t = (0 - pt.z) / dir.z;
			return v4{ pt.xy + t * dir.xy, 0, 1 };
		}
	};
}
