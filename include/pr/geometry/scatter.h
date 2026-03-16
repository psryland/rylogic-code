//********************************
// Scatter
//  Copyright (c) Rylogic Ltd 2025
//********************************
#pragma once
#include <span>
#include <vector>
#include <execution>
#include <random>
#include <tuple>
#include "pr/math/math.h"

namespace pr::geometry
{
	struct Body
	{
		v4 m_point;
		v4 m_size;
	};
	struct Link
	{
		int m_body0;
		int m_body1;
	};
	struct ScatterParams
	{
		float SpringConstant = 0.01f;
		float CoulombConstant = 10.0f;
		float FrictionConstant = 0.5f;
		float Equilibrium = 0.01f;
	};

	template <int Dim>
	struct Scatterer
	{
		using lock_word_t = uint32_t;

		ScatterParams m_params;
		std::span<Body> m_bodies;
		std::span<Link> m_links;
		std::vector<v4> m_velocities;
		std::vector<v4> m_forces;
		std::vector<std::atomic<lock_word_t>> m_locks;
		std::default_random_engine m_rng;
		bool m_equalibrium;

		Scatterer(std::span<Body> bodies, std::span<Link> links, ScatterParams const& params = {})
			: m_params(params)
			, m_bodies(bodies)
			, m_links(links)
			, m_velocities(m_bodies.size())
			, m_forces(m_bodies.size())
			, m_locks(m_bodies.size() / sizeof(lock_word_t))
			, m_rng()
			, m_equalibrium(false)
		{
		}

		// Step the force simulation
		void Step(float dt = 0.05f)
		{
			CalculateForces();
			Integrate(dt);
		}

		// Accumulate forces on the bodies
		void CalculateForces()
		{
			// This is just a balance between a force that is linear with distance
			// and a force that is quadratic with distance. Since spring simulations
			// easily become unstable, use a modified spring force function.

			// Determine coulomb forces
			std::for_each(std::execution::par, std::begin(m_bodies), std::end(m_bodies), [&](Body const& body0)
			{
				v4 force = {};
				for (auto& body1 : m_bodies)
				{
					if (&body0 == &body1)
						continue;

					// Find the minimum separation and the current separation
					auto [sep, min_sep] = Separation(body0, body1);

					// A coulomb force (F = kQq/r^2) is quadratically proportional to separation distance.
					// To handle nodes of unknown sizes, set all nodes to have the same charge, regardless of size
					// but make the separation equal to the distance between nearest points. Use charge of 1.
					auto dist = std::max(Length(sep) - min_sep, 1.0f);
					auto coulumb = m_params.CoulombConstant / (dist * dist);

					// Add the forces to each node. Half because the force is shared between the two nodes.
					// Yes, I know the 0.5 could be rolled into the CoulombConstant, but it's clearer this way.
					auto f = -0.5f * coulumb * Normalise(sep, v4::Zero());
					assert(IsFinite(f));

					force += f;
				}
				m_forces[&body0 - m_bodies.data()] = force;
			});

			// Determine spring forces
			std::for_each(std::execution::par, std::begin(m_links), std::end(m_links), [&](Link const& link)
			{
				auto& body0 = m_bodies[link.m_body0];
				auto& body1 = m_bodies[link.m_body1];

				// Find the minimum separation and the current separation
				auto [sep, min_sep] = Separation(body0, body1);

				// A spring force (F = -Kx) is linearly proportional to the deviation from the rest length.
				// To handle nodes of unknown sizes, set the rest length to 'min_sep'. To stop the
				// simulation blowing up, make the force zero when less than 'min_sep' and a constant
				// when above some maximum separation.
				auto dist = Clamp(Length(sep) - min_sep, -10 * min_sep, 10 * min_sep);
				auto spring = -m_params.SpringConstant * dist;

				// Add the forces to each node
				auto f = spring * Normalise(sep, v4::Zero());
				assert(IsFinite(f));

				Update(link.m_body0, [&](int i) { m_forces[i] -= f; });
				Update(link.m_body1, [&](int i) { m_forces[i] += f; });
			});

			// Viscosity forces
			std::for_each(std::execution::par, std::begin(m_bodies), std::end(m_bodies), [&](Body const& body)
			{
				auto i = &body - m_bodies.data();
				auto drag = m_params.FrictionConstant * m_velocities[i];
				m_forces[i] -= drag;
			});
		}

		// Advance the simulation
		void Integrate(float dt)
		{
			m_equalibrium = true;

			// Apply forces (2nd order integrator)
			constexpr float mass = 1.0f;
			std::for_each(std::execution::par, std::begin(m_bodies), std::end(m_bodies), [&](Body& body)
			{
				auto i = &body - m_bodies.data();
				auto a = m_forces[i] / mass;
				auto v = m_velocities[i] + 0.5f * dt * a;

				m_equalibrium &= LengthSq(a) < Sqr(m_params.Equilibrium);

				body.m_point += v * dt + 0.5f * a * Sqr(dt);
				m_velocities[i] += a;
				m_forces[i] = v4::Zero();
			});

			// Ensure the centroid is always at (0,0,0)
			auto count = 0;
			auto centre = v4::Origin();
			for (auto const& body : m_bodies) centre += (body.m_point - centre) / (1.0f + count++);
			for (auto& body : m_bodies) body.m_point -= centre;
		}

		// Return the separation vector between two points and the minimum distance in that direction to separate the bodies.
		std::tuple<v4, float> Separation(Body const& b0, Body const& b1)
		{
			v4 sep;
			float min_dist;

			auto vec = b1.m_point - b0.m_point;
			auto size = b1.m_size + b0.m_size;

			// Treat nodes as AABBs. This function needs to be tolerant of overlapping nodes.
			if constexpr (Dim == 3)
			{
				// 'pen' is the penetration depth. Positive for penetration.
				auto pen = size - Abs(vec);

				// The separating axis is the axis with the minimum penetration depth.
				auto i = MinElementIndex(pen.xyz);
				if (pen[i] < 0) // Not overlapping
				{
					sep = vec;
					min_dist = Length(size);
				}
				else
				{
					// Push out of penetration first
					sep = v4::Zero();
					sep[i] = vec[i];
					min_dist = size[i];
				}

				// 'sep' can be zero if the nodes are 2D. The zero-dimension will be the minimum penetration
				if (LengthSq(sep) < math::tiny<float>)
					sep.xyz = RandomN<v3>(m_rng);

				return { sep, min_dist };
			}
			else if constexpr (Dim == 2)
			{
				// Assume the AABBs are aligned with the camera (so 'size' does not need rotating)
				sep = vec;
				sep.z = 0.f;

				if (LengthSq(sep) < math::tiny<float>)
					sep.xy = RandomN<v2>(m_rng);

				// Find the minimum distance along 'sep' needed to separate the bodies
				min_dist = Dot(0.5f * size.xy, Abs(sep.xy)) / Length(sep.xy);

				return { sep, min_dist };
			}
			else
			{
				static_assert(dependent_false<Dim>, "Only 2D and 3D are supported");
			}
		}

		// Async update helper
		template <std::invocable<int> UpdateFn>
		void Update(int i, UpdateFn do_update)
		{
			auto word = i / sizeof(lock_word_t);
			auto bit = 1 << (i % sizeof(lock_word_t));
			for (;;)
			{
				auto expected = ~bit & m_locks[word].load();
				if (m_locks[word].compare_exchange_weak(expected, bit | expected))
					break;
			}
			do_update(i);
			for (;;)
			{
				auto expected = bit | m_locks[word].load();
				if (m_locks[word].compare_exchange_weak(expected, ~bit & expected))
					break;
			}
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::geometry::tests
{
	PRUnitTest(ScatterTests)
	{
		constexpr int Dim = 3;
		std::default_random_engine rng;

		std::vector<Body> bodies(100);
		for (auto& body : bodies)
		{
			body.m_point = Random<v3>(rng, v3::Zero(), 100.0f).w1();
			body.m_size = Abs(Random<v3>(rng, v3(1.0f), v3(5.0f))).w0();
			if constexpr (Dim == 2)
			{
				body.m_point.z = 0.0f;
				body.m_size.z = 0.0f;
			}
		}

		constexpr int LinksPerBody = 3;
		std::vector<Link> links(bodies.size() * LinksPerBody);
		for (auto& link : links)
		{
			do
			{
				link.m_body0 = static_cast<int>(&link - links.data()) % isize(bodies);
				link.m_body1 = std::uniform_int_distribution(0, isize(bodies) - 1)(rng);
			} while (link.m_body0 == link.m_body1);
		}

		Scatterer<Dim> scat(bodies, links, {});
		scat.Step();

		#if PR_UNITTESTS_VISUALISE
		{
			auto LdrDump = [&]
			{
				auto ldr = pr::ldraw::Builder{};
				auto& ldr_bodies = ldr.Group("Body");
				for (auto const& body : bodies)
					ldr_bodies.Box("Box", 0xFF00FF00).box(body.m_size).pos(body.m_point);
				auto& ldr_links = ldr.Line("Link", 0xFF005000);
				for (auto const& link : links)
					ldr_links.line(bodies[link.m_body0].m_point, bodies[link.m_body1].m_point);
				ldr.Save(temp_dir() / "scatter.ldr");
			};
			for (; !scat.m_equalibrium; )
			{
				LdrDump();
				scat.Step();
			}
		}
		#endif
	}
}
#endif