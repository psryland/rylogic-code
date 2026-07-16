//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/ship/ship.h"

namespace las
{
	namespace
	{
		static constexpr float InitialAverageDensityKgM3 = 450.0f;

		// Return the canonical 6 m by 2.5 m by 1.5 m convex hull points. X is bow-to-stern, Y is beam, and Z is up.
		std::array<v4, 12> HullPoints()
		{
			return
			{
				v4{-3.0f, -1.0f, +0.75f, 1.0f},
				v4{-3.0f, +1.0f, +0.75f, 1.0f},
				v4{-3.0f, -0.6f, -0.25f, 1.0f},
				v4{-3.0f, +0.6f, -0.25f, 1.0f},
				v4{-3.0f,  0.0f, -0.75f, 1.0f},
				v4{ 0.0f, -1.25f, +0.75f, 1.0f},
				v4{ 0.0f, +1.25f, +0.75f, 1.0f},
				v4{ 0.0f, -0.7f, -0.3f, 1.0f},
				v4{ 0.0f, +0.7f, -0.3f, 1.0f},
				v4{ 0.0f,  0.0f, -0.75f, 1.0f},
				v4{+3.0f,  0.0f, +0.55f, 1.0f},
				v4{+2.8f,  0.0f, -0.35f, 1.0f},
			};
		}

		// Build the authoritative collision shape from the canonical hull points.
		byte_data<16> BuildHullShape()
		{
			auto points = HullPoints();
			return BuildPolytopeFromPoints(std::span{points});
		}

		// Create the visible hull from the same convex topology used by collision and buoyancy.
		ModelPtr CreateHullModel(Renderer& rdr, ShapePolytope const& hull)
		{
			auto verts = std::vector<v4>{};
			auto normals = std::vector<v4>{};
			auto colours = std::vector<Colour32>{};
			auto indices = std::vector<uint16_t>{};
			verts.reserve(3 * hull.m_face_count);
			normals.reserve(3 * hull.m_face_count);
			colours.reserve(3 * hull.m_face_count);
			indices.reserve(3 * hull.m_face_count);

			// Duplicate triangle vertices so the rudimentary hull keeps crisp chine and deck edges.
			for (auto const& face : hull.faces())
			{
				auto normal = face.m_plane.direction();
				auto colour = normal.z > 0.5f ? Colour32(0xFFE8E4D8) : Colour32(0xFF805020);
				for (int corner = 0; corner != 3; ++corner)
				{
					indices.push_back(s_cast<uint16_t>(verts.size()));
					verts.push_back(hull.vertex(face.m_index[corner]));
					normals.push_back(normal);
					colours.push_back(colour);
				}
			}

			auto nuggets = std::array
			{
				NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Norm | EGeom::Colr)
			};
			auto cdata = MeshCreationData()
				.verts(std::span{verts})
				.indices(std::span<uint16_t const>{indices})
				.normals(std::span{normals})
				.colours(std::span{colours})
				.nuggets(std::span{nuggets});
			ResourceFactory factory(rdr);
			auto model = ModelGenerator::Mesh(factory, cdata);
			factory.FlushToGpu(EGpuFlush::Block);
			return model;
		}
	}

	// Construct the ship body and matching render instance.
	Ship::Ship(Renderer& rdr, PhysicsSystem& physics, v4 location)
		:m_physics(physics)
		,m_body_handle(PhysicsSystem::BodyHandle::Invalid())
		,m_buoyancy_hull()
		,m_hull_volume_m3()
		,m_requested_density_kg_m3(InitialAverageDensityKgM3)
		,m_applied_density_kg_m3(InitialAverageDensityKgM3)
		,m_inst()
	{
		// Start partially immersed so the proof-of-concept reaches its density-controlled equilibrium quickly.
		location.z = 0.2f;
		auto const o2w = m4x4::Translation(location);
		m_inst.m_i2w = o2w;

		// The ship is fully registered with the physics system before construction completes.
		try
		{
			m_body_handle = m_physics.CreateBody(PhysicsSystem::BodyDesc{
				.m_shape_data = BuildHullShape(),
				.m_o2w = o2w,
				.m_density_kg_m3 = InitialAverageDensityKgM3,
				.m_never_sleep = false,
			});

			// Build rendering directly from the physics-owned hull so visible and sampled surfaces cannot diverge.
			auto const& hull = shape_cast<ShapePolytope>(m_physics.BodyShape(m_body_handle));
			m_hull_volume_m3 = CalcVolume(hull);
			m_inst.m_model = CreateHullModel(rdr, hull);
			m_buoyancy_hull = m_physics.RegisterBuoyancyHull(m_body_handle);
		}
		catch (...)
		{
			if (m_physics.IsValid(m_body_handle))
			{
				m_physics.DestroyBody(m_body_handle);
			}
			throw;
		}
	}

	// Destroy the ship's physics registration while the rigid body is still alive.
	Ship::~Ship()
	{
		m_buoyancy_hull.Reset();
		if (m_physics.IsValid(m_body_handle))
		{
			m_physics.DestroyBody(m_body_handle);
		}
	}

	// Return the latest published ship transform.
	m4x4 Ship::O2W() const
	{
		return m_physics.Snapshot(m_body_handle).m_o2w;
	}

	// Access the physics registration handle for this ship's body.
	PhysicsSystem::BodyHandle Ship::PhysicsHandle() const
	{
		return m_body_handle;
	}

	// Request a new average density; the simulation owner applies it before the next physics step.
	void Ship::AverageDensity(float density_kg_m3)
	{
		if (!std::isfinite(density_kg_m3) || density_kg_m3 <= 0.0f)
			throw std::runtime_error("Ship average density must be positive and finite");

		m_requested_density_kg_m3.store(density_kg_m3, std::memory_order_relaxed);
	}

	// Return the requested average ship density.
	float Ship::AverageDensity() const
	{
		return m_requested_density_kg_m3.load(std::memory_order_relaxed);
	}

	// Return the density currently applied to the rigid body's mass properties.
	float Ship::AppliedAverageDensity() const
	{
		return m_applied_density_kg_m3.load(std::memory_order_relaxed);
	}

	// Return the closed convex hull volume.
	float Ship::HullVolume() const
	{
		return m_hull_volume_m3;
	}

	// Return the density-derived rigid-body mass.
	float Ship::Mass() const
	{
		return HullVolume() * AppliedAverageDensity();
	}

	// Apply pending tuning requests on the simulation owner thread.
	void Ship::ApplyTuning()
	{
		auto requested_density = AverageDensity();
		if (requested_density == AppliedAverageDensity())
			return;

		m_physics.SetBodyDensity(m_body_handle, requested_density);
		m_applied_density_kg_m3.store(requested_density, std::memory_order_relaxed);
	}

	// Return the latest read-back buoyancy diagnostics for the ship hull.
	physics::GpuBuoyancy::Diagnostics Ship::BuoyancyDiagnostics() const
	{
		return m_physics.LatestBuoyancyDiagnostics(m_body_handle);
	}

	// Prepare shader constant buffers for rendering.
	void Ship::PrepareRender(v4)
	{
		// The standard forward renderer transforms vertices via m_o2s (= c2s * w2c * o2w)
		// which already handles the camera position via w2c. No manual camera-relative
		// subtraction needed — that would cause double-subtraction.
		m_inst.m_i2w = O2W();
	}

	// Add the ship render instance to the scene.
	void Ship::AddToScene(Scene& scene)
	{
		scene.AddInstance(m_inst);
	}
}
