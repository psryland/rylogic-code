#pragma once
#include "src/forward.h"
#include "src/idemo_scene.h"

namespace pr::fluid
{
	struct Scene3d : IDemoScene
	{
		enum class EFillStyle
		{
			Point,
			Random,
			Lattice,
			Grid,
		};

		std::vector<fluid::Particle> m_particles;
		std::vector<fluid::Dynamics> m_dynamics;
		CollisionBuilder m_col;
		ldraw::Builder m_ldr;

		explicit Scene3d(int particle_count)
			: m_col()
			, m_ldr()
			, m_particles(particle_count)
			, m_dynamics(particle_count)
		{
			ParticleInitData(EFillStyle::Lattice, m_particles, m_dynamics);

			m4x4 o2w;
			//v4 dim;

			// Floor
			o2w = m4x4{ m3x4::Rotation(AxisId::PosZ, AxisId::PosY), v4{ 0, -0.5f, 0, 1 } };
			m_ldr.Plane("floor", 0xFFade3ff).wh({ 1, 1 }).o2w(o2w);
			m_col.Plane().o2w(o2w);

			// Ceiling
			//o2w = m4x4{ m3x4::Rotation(AxisId::PosZ, AxisId::NegY), v4{ 0, +0.5f, 0, 1 } };
			//m_ldr.Plane("ceiling", 0x10ade3ff).wh({ 1, 1 }).o2w(o2w);
			//m_col.Plane().o2w(o2w);

			// Left Wall
			o2w = m4x4{ m3x4::Rotation(AxisId::PosZ, AxisId::PosX), v4{ -0.5f, -0.25f, 0, 1 } };
			m_ldr.Plane("left_wall", 0x40ade3ff).wh({ 1, 0.5f }).o2w(o2w);
			m_col.Plane().o2w(o2w);

			// Right Wall
			o2w = m4x4{ m3x4::Rotation(AxisId::PosZ, AxisId::NegX), v4{ +0.5f, -0.25f, 0, 1 } };
			m_ldr.Plane("right_wall", 0x40ade3ff).wh({ 1, 0.5f }).o2w(o2w);
			m_col.Plane().o2w(o2w);

			// Front Wall
			o2w = m4x4{ m3x4::Rotation(AxisId::PosZ, AxisId::PosZ), v4{ 0, -0.25f, -0.5f, 1 } };
			m_ldr.Plane("left_wall", 0x40ade3ff).wh({ 1, 0.5f }).o2w(o2w);
			m_col.Plane().o2w(o2w);

			// Back Wall
			o2w = m4x4{ m3x4::Rotation(AxisId::PosZ, AxisId::NegZ), v4{ 0, -0.25f, +0.5f, 1 } };
			m_ldr.Plane("right_wall", 0x40ade3ff).wh({ 1, 0.5f }).o2w(o2w);
			m_col.Plane().o2w(o2w);

			// Box in the middle
			//dim = v4(0.3f, 0.2f, 0.1f, 0);
			//o2w = m4x4::Identity();
			//m_ldr.Box("cube", 0x80FFB86D).dim(dim).o2w(o2w);
			//m_col.Box(dim).o2w(o2w);

			m_ldr.WrapAsGroup();
		}

		// 2D or 3D
		int SpatialDimensions() const override
		{
			return 3;
		}

		// Initial camera position
		void Camera(pr::Camera& camera) const override
		{
			camera.LookAt(v4(1.0f, 1.2f, 1.0f, 1), v4(0, 0.0f, 0, 1), v4(0, 1, 0, 0));
			camera.Align(v4::YAxis());
		}

		// Return the visualisation scene
		std::string LdrScene() const override
		{
			return m_ldr.ToString();
		}

		// Returns initialisation data for the particles.
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
				.Geom = { v4::Zero(), v4::Zero() },
				.Mode = ParticleCollision::ECullMode::None,
			};
		}

		// Move the probe around
		v4 PositionProbe(gui::Point ss_pt, rdr12::Scene const& scn) const override
		{
			// Set the probe position from a SS point
			// Shoot a ray through the mouse pointer
			auto d = -scn.m_cam.WorldToCamera().pos.z; // Z-Distance to the origin from the camera
			auto nss_point = scn.m_viewport.SSPointToNSSPoint(To<v2>(ss_pt));
			auto [pt, dir] = scn.m_cam.NSSPointToWSRay(v4(nss_point, d, 0));
			return pt + d * dir;
		}

		// Create particles
		static void ParticleInitData(EFillStyle style, std::span<fluid::Particle> particles, std::span<fluid::Dynamics> dynamics)
		{
			assert(particles.size() == dynamics.size());
			int idx = 0, count = isize(particles);
			auto points = [&](v4 p, v4 v)
			{
				assert(p.w == 1 && v.w == 0);
				particles[idx] = fluid::Particle{
					.pos = p,
					.col = v4::One(),
				};
				dynamics[idx] = fluid::Dynamics{
					.vel = v,
					.accel = v4::Zero(),
					.surface = v4{0,0,0,limits<float>::max()},
				};
				++idx;
			};

			const float hwidth = 0.5f;
			const float hheight = 0.5f;
			const float hdepth = 0.5f;

			switch (style)
			{
				case EFillStyle::Point:
				{
					for (int i = 0; i != count; ++i)
						points(v4(-0.9f, 0, 0, 1), v4(-0.1f, -0.1f, 0, 0));

					break;
				}
				case EFillStyle::Random:
				{
					auto const margin = 0.95f;
					auto hw = hwidth * margin;
					auto hh = hheight * margin;
					auto hd = hdepth * margin;
					auto vx = 0.2f;

					// Uniform distribution over the volume
					std::default_random_engine rng;
					for (int i = 0; i != count; ++i)
					{
						auto pos = Random<v3>(rng, v3(-hw, -hh, -hd), v3(+hw, +hh, +hd)).w1();
						auto vel = Random<v3>(rng, v3(-vx, -vx, -vx), v3(+vx, +vx, +vx)).w0();
						points(pos, vel);
					}
					break;
				}
				case EFillStyle::Lattice:
				{
					auto const margin = 0.95f;
					auto hw = hwidth * margin;
					auto hh = hheight * margin;
					auto hd = hdepth * margin;

					// Want to spread N particles evenly over the volume.
					// Volume is 2*hwidth * 2*hdepth * 2*hheight
					// Want to find 'step' such that:
					//  (2*hwidth/step) * (2*hdepth/step) * (2*hheight/step) = N
					// => step = cubert(8 * hwidth * hdepth * hheight / N)
					auto step = Cubert(8 * hw * hh * hw / count);

					auto x = -hw + step / 2;
					auto y = -hh + step / 2;
					auto z = -hd + step / 2;
					for (int i = 0; i != count; ++i)
					{
						points(v4(x, y, z, 1), v4::Zero());

						x += step;
						if (x > hw) { x = -hw + step / 2; z += step; }
						if (z > hd) { z = -hd + step / 2; y += step; }
					}
					break;
				}
				case EFillStyle::Grid:
				{
					auto const margin = 1.0f;//0.95f;
					auto hw = hwidth * margin;
					auto hh = hheight * margin;
					auto hd = hdepth * margin;
					auto step = 0.1f;

					auto x = -hw + step / 2.0f;
					auto y = -hh + step / 2.0f;
					auto z = -hd + step / 2.0f;
					for (int i = 0; i != count; ++i)
					{
						points(v4(x, y, z, 1), v4::Zero());

						x += step;
						if (x > hw) { x = -hw + step / 2; z += step; }
						if (z > hd) { z = -hd + step / 2; y += step; }
					}
					break;
				}
			}
		}
	};
}
