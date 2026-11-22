//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/game-systems/forward.h"
#include "pr/game-systems/particles/particle_presets.h"
#include "pr/game-systems/particles/particle_system.h"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_stepped_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"
#include "pr/game-systems/particles/particle_debris.hlsli"
#include "pr/game-systems/particles/particle_bird.hlsli"
#include "pr/game-systems/particles/particle_marker.hlsli"
#include "pr/game-systems/particles/particle_spray.hlsli"
#include "pr/game-systems/particles/particle_froth.hlsli"

namespace pr::game_systems::particles
{
	namespace
	{
		using EParticlePreset = pr::view3d::EParticlePreset;
		using EParticleBlend = pr::view3d::EParticleBlend;
		using ParticlePreset = pr::view3d::ParticlePreset;

		// A mesh for the stock mesh batch: its vertices and its triangle list.
		struct Mesh
		{
			std::vector<ParticleMeshVertex> m_verts;
			std::vector<uint32_t> m_indices;
		};

		// The stock mesh draw shaders, with 'mesh' as the batch's shared mesh.
		pr::view3d::ParticleDrawShaders MeshDraw(Mesh const& mesh)
		{
			// The mesh is static, so the pointers stay valid.
			auto draw = ParticleSystem::MeshDrawShaders();
			draw.m_indices = mesh.m_indices.data();
			draw.m_index_count = static_cast<int>(mesh.m_indices.size());
			draw.m_vertex_count = static_cast<int>(mesh.m_verts.size());
			draw.m_vertex_data = mesh.m_verts.data();
			return draw;
		}

		// Add a flat face of 'outline' around 'centre' in the xy plane to 'mesh', seen from both sides. 'outline' is anticlockwise seen from +z.
		void AddTwoSidedFan(Mesh& mesh, std::span<v2 const> outline, v2 centre)
		{
			// Each side has its own vertices, so each side has its own normal and is lit from its own side.
			for (auto side : { +1.0f, -1.0f })
			{
				// The centre is the last vertex of the side. UVs map the xy range [-0.5, 0.5] onto the texture.
				auto base = static_cast<uint32_t>(mesh.m_verts.size());
				auto count = static_cast<uint32_t>(outline.size());
				auto vert = [&](v2 p)
				{
					return ParticleMeshVertex{ .pos = { p.x, p.y, 0.0f }, .u = p.x + 0.5f, .norm = { 0.0f, 0.0f, side }, .v = 0.5f - p.y };
				};
				for (auto p : outline)
					mesh.m_verts.push_back(vert(p));

				mesh.m_verts.push_back(vert(centre));

				// Triangles wind anticlockwise as seen from the side's normal.
				for (uint32_t i = 0; i != count; ++i)
				{
					auto a = base + i;
					auto b = base + (i + 1) % count;
					mesh.m_indices.push_back(base + count);
					mesh.m_indices.push_back(side > 0 ? a : b);
					mesh.m_indices.push_back(side > 0 ? b : a);
				}
			}
		}

		// A leaf: a flat, pointed oval in the xy plane, one unit long along y, seen from both sides.
		Mesh const& LeafMesh()
		{
			// The outline runs anticlockwise from the tip, and the fan centre is a little towards the stem.
			static auto const mesh = []
			{
				auto outline = std::array<v2, 8>
				{{
					{ 0.00f, 0.50f }, { -0.15f, 0.30f }, { -0.18f, 0.05f }, { -0.12f, -0.25f },
					{ 0.00f, -0.50f }, { 0.12f, -0.25f }, { 0.18f, 0.05f }, { 0.15f, 0.30f },
				}};
				auto m = Mesh{};
				AddTwoSidedFan(m, outline, { 0.0f, -0.05f });
				return m;
			}();
			return mesh;
		}

		// A wood chip: a box one unit long along x, half a unit wide along y, and a fifth of a unit thick along z.
		Mesh const& ChipMesh()
		{
			// Each face has its own four vertices, so each face has a flat normal.
			static auto const mesh = []
			{
				auto m = Mesh{};
				auto const half = v3{ 0.5f, 0.25f, 0.1f };
				for (int axis = 0; axis != 3; ++axis)
				{
					for (auto sign : { +1.0f, -1.0f })
					{
						// 'u' x 'v' = 'n', so the corners below run anticlockwise seen from outside the box.
						auto n = v3::Zero(); n[axis] = sign;
						auto u = v3::Zero(); u[(axis + (sign > 0 ? 1 : 2)) % 3] = 1.0f;
						auto v = v3::Zero(); v[(axis + (sign > 0 ? 2 : 1)) % 3] = 1.0f;
						auto base = static_cast<uint32_t>(m.m_verts.size());
						constexpr float corners[4][2] = { {-1, -1}, {+1, -1}, {+1, +1}, {-1, +1} };
						for (auto const& c : corners)
						{
							auto p = (n + u * c[0] + v * c[1]) * half;
							m.m_verts.push_back(ParticleMeshVertex{ .pos = { p.x, p.y, p.z }, .u = 0.5f + 0.5f * c[0], .norm = { n.x, n.y, n.z }, .v = 0.5f - 0.5f * c[1] });
						}
						for (auto i : { 0u, 1u, 2u, 0u, 2u, 3u })
							m.m_indices.push_back(base + i);
					}
				}
				return m;
			}();
			return mesh;
		}

		// A marker ring: a flat annulus in the xy plane, from radius 0.8 to 1, seen from both sides.
		Mesh const& RingMesh()
		{
			// Each side has an inner and an outer vertex at each of the segment angles.
			static auto const mesh = []
			{
				constexpr uint32_t Segments = 32;
				constexpr float Inner = 0.8f;
				auto m = Mesh{};
				for (auto side : { +1.0f, -1.0f })
				{
					// Vertices alternate inner and outer around the ring. U runs around the ring, and v across it.
					auto base = static_cast<uint32_t>(m.m_verts.size());
					for (uint32_t i = 0; i != Segments; ++i)
					{
						auto angle = pr::math::constants<float>::tau * i / Segments;
						auto c = std::cos(angle);
						auto s = std::sin(angle);
						auto u = static_cast<float>(i) / Segments;
						m.m_verts.push_back(ParticleMeshVertex{ .pos = { Inner * c, Inner * s, 0.0f }, .u = u, .norm = { 0.0f, 0.0f, side }, .v = 1.0f });
						m.m_verts.push_back(ParticleMeshVertex{ .pos = { c, s, 0.0f }, .u = u, .norm = { 0.0f, 0.0f, side }, .v = 0.0f });
					}

					// Each segment is a quad from the inner edge to the outer edge, wound anticlockwise as seen from the side's normal.
					for (uint32_t i = 0; i != Segments; ++i)
					{
						auto in0 = base + 2 * i;
						auto out0 = in0 + 1;
						auto in1 = base + 2 * ((i + 1) % Segments);
						auto out1 = in1 + 1;
						auto quad = side > 0
							? std::array<uint32_t, 6>{ in0, out0, out1, in0, out1, in1 }
							: std::array<uint32_t, 6>{ in0, out1, out0, in0, in1, out1 };
						m.m_indices.insert(m.m_indices.end(), quad.begin(), quad.end());
					}
				}
				return m;
			}();
			return mesh;
		}

		// A preset of the stepped sprite type, drawn by the stock sprite batch.
		ParticlePreset SteppedSprite(ParticleSpriteConstants const& constants, ParticleSteppedSpriteEmitter const& params, EParticleBlend blend, float rate, int burst)
		{
			// 'constants' and 'params' are static, so the pointers stay valid.
			return ParticlePreset{
				.m_sim = ParticleSystem::SteppedSpriteSimShaders(),
				.m_draw = ParticleSystem::SpriteDrawShaders(),
				.m_blend = blend,
				.m_record_stride = sizeof(ParticleSpriteRecord),
				.m_state_stride = sizeof(ParticleSteppedSpriteState),
				.m_params_stride = sizeof(ParticleSteppedSpriteEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
				.m_params = &params,
				.m_rate = rate,
				.m_burst = burst,
			};
		}

		// A preset of the debris type, drawn by the stock mesh batch with 'mesh'.
		ParticlePreset Debris(ParticleDebrisConstants const& constants, ParticleDebrisEmitter const& params, Mesh const& mesh, float rate, int burst)
		{
			// Debris pieces are solid.
			return ParticlePreset{
				.m_sim = ParticleSystem::DebrisSimShaders(),
				.m_draw = MeshDraw(mesh),
				.m_blend = EParticleBlend::Opaque,
				.m_record_stride = sizeof(ParticleMeshRecord),
				.m_state_stride = sizeof(ParticleDebrisState),
				.m_params_stride = sizeof(ParticleDebrisEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
				.m_params = &params,
				.m_rate = rate,
				.m_burst = burst,
			};
		}
	}

	// Return the settings of the stock effect 'preset'.
	pr::view3d::ParticlePreset ParticlePresetGet(pr::view3d::EParticlePreset preset)
	{
		// Colours are 0xAARRGGBB. Curve key colours multiply the emitter's colour, so 0x00FFFFFF only fades the alpha.
		switch (preset)
		{
			case EParticlePreset::Fire:
			{
				// Flames fade in, shrink as they rise, and flicker. Drag pulls them along with the wind, so fire leans downwind.
				static ParticleSpriteConstants const constants = {
					.key_count = 4,
					.flicker = 0.3f,
					.flicker_rate = 8.0f,
					.streak = 0.0f,
					.keys = { { 0.0f, 0.5f, 0x00FFFFFF }, { 0.1f, 1.0f, 0xFFFFFFFF }, { 0.5f, 0.9f, 0xC0FFFFFF }, { 1.0f, 0.4f, 0x00FFFFFF } },
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 0.25f },
					.velocity = { 0.0f, 0.0f, 0.6f, 0.4f },
					.accel = { 0.0f, 0.0f, 2.0f, 1.5f },
					.lifetime = { 0.6f, 1.2f },
					.size = { 0.6f, 0.15f },
					.colour0 = 0xFFFFD860,
					.colour1 = 0xFFE03010,
					.radius = 0.05f,
					.collision_set = -1,
					.gravity = 0.0f,
					.die_on_contact = 0,
				};
				return SteppedSprite(constants, params, EParticleBlend::Weighted, 60.0f, 0);
			}
			case EParticlePreset::Sparks:
			{
				// Bright sparks streak along their motion, flicker quickly, and fade out at the end of their life.
				static ParticleSpriteConstants const constants = {
					.key_count = 3,
					.flicker = 0.2f,
					.flicker_rate = 20.0f,
					.streak = 0.03f,
					.keys = { { 0.0f, 1.0f, 0xFFFFFFFF }, { 0.7f, 0.8f, 0xFFFFFFFF }, { 1.0f, 0.5f, 0x00FFFFFF } },
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 0.05f },
					.velocity = { 0.0f, 0.0f, 4.0f, 3.0f },
					.accel = { 0.0f, 0.0f, 0.0f, 0.2f },
					.lifetime = { 0.8f, 1.6f },
					.size = { 0.05f, 0.03f },
					.colour0 = 0xFFFFF0B0,
					.colour1 = 0xFFFF6010,
					.radius = 0.02f,
					.collision_set = -1,
					.gravity = 1.0f,
					.die_on_contact = 0,
				};
				return SteppedSprite(constants, params, EParticleBlend::Sorted, 30.0f, 80);
			}
			case EParticlePreset::Smoke:
			{
				// Puffs fade in, grow, and fade out. A small lift against drag sets a slow rise, and drag carries the smoke with the wind.
				static ParticleSpriteConstants const constants = {
					.key_count = 3,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.0f,
					.keys = { { 0.0f, 0.4f, 0x00FFFFFF }, { 0.15f, 0.7f, 0x90FFFFFF }, { 1.0f, 1.0f, 0x00FFFFFF } },
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 0.3f },
					.velocity = { 0.0f, 0.0f, 0.8f, 0.3f },
					.accel = { 0.0f, 0.0f, 0.6f, 0.8f },
					.lifetime = { 4.0f, 7.0f },
					.size = { 0.4f, 3.0f },
					.colour0 = 0xFF505050,
					.colour1 = 0xFF909090,
					.radius = 0.2f,
					.collision_set = -1,
					.gravity = 0.0f,
					.die_on_contact = 0,
				};
				return SteppedSprite(constants, params, EParticleBlend::Weighted, 8.0f, 0);
			}
			case EParticlePreset::Dust:
			{
				// A burst billows outwards, slows quickly under drag, sinks slowly, and fades as it spreads.
				static ParticleSpriteConstants const constants = {
					.key_count = 3,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.0f,
					.keys = { { 0.0f, 0.4f, 0x00FFFFFF }, { 0.08f, 0.8f, 0xB0FFFFFF }, { 1.0f, 1.0f, 0x00FFFFFF } },
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 2.0f },
					.velocity = { 0.0f, 0.0f, 1.0f, 4.0f },
					.accel = { 0.0f, 0.0f, 0.0f, 1.2f },
					.lifetime = { 4.0f, 8.0f },
					.size = { 1.0f, 5.0f },
					.colour0 = 0xFF9C8C70,
					.colour1 = 0xFFB8AC90,
					.radius = 0.5f,
					.collision_set = -1,
					.gravity = 0.3f,
					.die_on_contact = 0,
				};
				return SteppedSprite(constants, params, EParticleBlend::Weighted, 0.0f, 400);
			}
			case EParticlePreset::Rain:
			{
				// Drops fall at about 9 m/s, where drag balances gravity, and streak along their fall. They die where they land.
				static ParticleSpriteConstants const constants = {
					.key_count = 0,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.04f,
					.keys = {},
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 15.0f },
					.velocity = { 0.0f, 0.0f, -9.0f, 0.5f },
					.accel = { 0.0f, 0.0f, 0.0f, 1.1f },
					.lifetime = { 4.0f, 5.0f },
					.size = { 0.02f, 0.02f },
					.colour0 = 0x90A8B8D0,
					.colour1 = 0x90A8B8D0,
					.radius = 0.01f,
					.collision_set = -1,
					.gravity = 1.0f,
					.die_on_contact = 1,
				};
				return SteppedSprite(constants, params, EParticleBlend::Weighted, 1000.0f, 0);
			}
			case EParticlePreset::Snow:
			{
				// Strong drag holds flakes at about 1 m/s and carries them with the wind. They fade in, and die where they land.
				static ParticleSpriteConstants const constants = {
					.key_count = 2,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.0f,
					.keys = { { 0.0f, 1.0f, 0x00FFFFFF }, { 0.05f, 1.0f, 0xFFFFFFFF } },
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 15.0f },
					.velocity = { 0.0f, 0.0f, -1.0f, 0.3f },
					.accel = { 0.0f, 0.0f, 0.0f, 10.0f },
					.lifetime = { 10.0f, 12.0f },
					.size = { 0.05f, 0.05f },
					.colour0 = 0xE0FFFFFF,
					.colour1 = 0xE0FFFFFF,
					.radius = 0.025f,
					.collision_set = -1,
					.gravity = 1.0f,
					.die_on_contact = 1,
				};
				return SteppedSprite(constants, params, EParticleBlend::Weighted, 400.0f, 0);
			}
			case EParticlePreset::Leaves:
			{
				// Drag holds leaves at a slow fall and carries them with the wind, and flutter swings them about as they fall. Contacts settle them.
				static ParticleDebrisConstants const constants = {
					.grow = 0.3f,
					.shrink = 1.0f,
					.mesh_radius = 0.5f,
				};
				static ParticleDebrisEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 1.0f },
					.velocity = { 0.0f, 0.0f, 0.0f, 0.5f },
					.accel = { 0.0f, 0.0f, 0.0f, 6.0f },
					.lifetime = { 8.0f, 12.0f },
					.size = { 0.12f, 0.2f },
					.spin = { 2.0f, 5.0f },
					.flutter = 4.0f,
					.flutter_rate = 0.7f,
					.colour0 = 0xFF5E8A1E,
					.colour1 = 0xFFC8681C,
					.radius = 0.02f,
					.collision_set = -1,
					.settle = 0.8f,
					.gravity = 1.0f,
				};
				return Debris(constants, params, LeafMesh(), 3.0f, 0);
			}
			case EParticlePreset::Birds:
			{
				// A flock of crow-sized birds circling within about 30 m of the emitter. They fade in and out slowly so the flock changes unnoticed.
				static ParticleBirdConstants const constants = {
					.bank = 0.3f,
					.flap_amplitude = 0.6f,
					.dihedral = 0.12f,
					.fade = 2.0f,
				};
				static ParticleBirdEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 10.0f },
					.orbit = { 8.0f, 20.0f },
					.speed = { 6.0f, 10.0f },
					.bob = { 1.5f, 6.0f },
					.lifetime = { 100.0f, 140.0f },
					.size = 0.6f,
					.flap_rate = 4.0f,
					.colour = 0xFF202020,
				};
				return ParticlePreset{
					.m_sim = ParticleSystem::BirdSimShaders(),
					.m_draw = ParticleSystem::BirdDrawShaders(),
					.m_blend = EParticleBlend::Opaque,
					.m_record_stride = sizeof(ParticleBirdRecord),
					.m_state_stride = sizeof(ParticleBirdState),
					.m_params_stride = sizeof(ParticleBirdEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
					.m_params = &params,
					.m_rate = 0.0f,
					.m_burst = 12,
				};
			}
			case EParticlePreset::WoodChips:
			{
				// Chips are thrown up and out, tumble quickly, and bounce and settle where they land. Little drag, so they fly in arcs.
				static ParticleDebrisConstants const constants = {
					.grow = 0.0f,
					.shrink = 0.5f,
					.mesh_radius = 0.57f,
				};
				static ParticleDebrisEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 0.1f },
					.velocity = { 0.0f, 0.0f, 3.0f, 3.0f },
					.accel = { 0.0f, 0.0f, 0.0f, 0.1f },
					.lifetime = { 3.0f, 5.0f },
					.size = { 0.06f, 0.12f },
					.spin = { 8.0f, 20.0f },
					.flutter = 0.0f,
					.flutter_rate = 0.0f,
					.colour0 = 0xFFE0BC80,
					.colour1 = 0xFFB88848,
					.radius = 0.03f,
					.collision_set = -1,
					.settle = 0.6f,
					.gravity = 1.0f,
				};
				return Debris(constants, params, ChipMesh(), 0.0f, 12);
			}
			case EParticlePreset::Marker:
			{
				// Rings face up, turn slowly, and expand as they fade, so a steady rate gives a train of pulses.
				static ParticleMarkerConstants const constants = {
					.mesh_radius = 1.0f,
				};
				static ParticleMarkerEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 0.0f },
					.normal = { 0.0f, 0.0f, 1.0f, 0.5f },
					.size = { 0.3f, 2.5f },
					.lifetime = { 1.6f, 1.6f },
					.colour0 = 0xFF40E0FF,
					.colour1 = 0x0040E0FF,
					.bob = 0.0f,
					.bob_rate = 0.0f,
				};
				auto const& mesh = RingMesh();
				return ParticlePreset{
					.m_sim = ParticleSystem::MarkerSimShaders(),
					.m_draw = MeshDraw(mesh),
					.m_blend = EParticleBlend::Weighted,
					.m_record_stride = sizeof(ParticleMeshRecord),
					.m_state_stride = sizeof(ParticleMarkerState),
					.m_params_stride = sizeof(ParticleMarkerEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
					.m_params = &params,
					.m_rate = 1.2f,
					.m_burst = 0,
				};
			}
			case EParticlePreset::Spray:
			{
				// Fine droplets leap from breaking crests, streak a little, carry downwind, and fade as they fall back. Many births find no crest, so
				// the rate is the number of tries per second; the spray seen depends on how rough the sea is.
				static ParticleSpriteConstants const constants = {
					.key_count = 3,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.02f,
					.keys = { { 0.0f, 0.6f, 0x00FFFFFF }, { 0.1f, 1.0f, 0xFFFFFFFF }, { 1.0f, 1.4f, 0x00FFFFFF } },
				};
				static ParticleSprayEmitter const params = {
					.area = { 0.0f, 0.0f, 0.0f, 60.0f },
					.launch = { 3.0f, 1.5f, 0.6f, 0.5f },
					.lifetime = { 1.0f, 2.0f },
					.size = { 0.08f, 0.2f },
					.colour0 = 0xE0F4F8FF,
					.colour1 = 0x80E0E8F0,
					.foam_threshold = 0.3f,
					.candidates = 8,
					.gravity = 1.0f,
				};
				return ParticlePreset{
					.m_sim = ParticleSystem::SpraySimShaders(),
					.m_draw = ParticleSystem::SpriteDrawShaders(),
					.m_blend = EParticleBlend::Weighted,
					.m_record_stride = sizeof(ParticleSpriteRecord),
					.m_state_stride = sizeof(ParticleSteppedSpriteState),
					.m_params_stride = sizeof(ParticleSprayEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
					.m_params = &params,
					.m_rate = 2000.0f,
					.m_burst = 0,
				};
			}
			case EParticlePreset::Froth:
			{
				// Patches form on breaking crests, spread, and fade as they drift slowly downwind on the waves.
				static ParticleFrothConstants const constants = {
					.fade_in = 0.5f,
					.fade_out = 2.0f,
				};
				static ParticleFrothEmitter const params = {
					.area = { 0.0f, 0.0f, 0.0f, 60.0f },
					.lifetime = { 4.0f, 8.0f },
					.size = { 0.8f, 2.5f },
					.colour0 = 0xD0F8FAFF,
					.colour1 = 0x90E8EEF4,
					.foam_threshold = 0.3f,
					.candidates = 8,
					.drift = 0.03f,
					.spread = 0.2f,
					.lift = 0.05f,
				};
				return ParticlePreset{
					.m_sim = ParticleSystem::FrothSimShaders(),
					.m_draw = ParticleSystem::FlatSpriteDrawShaders(),
					.m_blend = EParticleBlend::Weighted,
					.m_record_stride = sizeof(ParticleMeshRecord),
					.m_state_stride = sizeof(ParticleFrothState),
					.m_params_stride = sizeof(ParticleFrothEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
					.m_params = &params,
					.m_rate = 300.0f,
					.m_burst = 0,
				};
			}
			default:
			{
				throw std::invalid_argument(std::format("Unknown particle preset: {}", static_cast<int>(preset)));
			}
		}
	}
}
