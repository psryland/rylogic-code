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

		// Return a value in [0, 1) for each integer pair and seed. A fixed hash gives every run the same procedural textures.
		float TexelHash(int x, int y, uint32_t seed)
		{
			// Mix the inputs into one well-scrambled value.
			auto hash = static_cast<uint32_t>(x) * 0x8da6b343U ^ static_cast<uint32_t>(y) * 0xd8163841U ^ seed * 0xcb1ab31fU;
			hash = (hash ^ (hash >> 16)) * 0x7feb352dU;
			hash = (hash ^ (hash >> 15)) * 0x846ca68bU;
			hash ^= hash >> 16;
			return static_cast<float>(hash) / 4294967296.0f;
		}

		// Texels per side of the froth texture.
		constexpr int FrothTextureSize = 256;

		// Return the stock froth texture: a heap of lit lumps of foam. Its alpha is the foam's coverage, solid in the middle of each lump and soft at
		// its edge, with scattered pin holes. The lighting assumes the sprite is upright. Texels are R8G8B8A8 with straight alpha.
		std::vector<uint32_t> const& FrothTexels()
		{
			static auto const texels = []
			{
				// The shared hash with this texture's seeds.
				auto const random = TexelHash;

				// The clump is a heap of round lumps of foam, packed most tightly at its centre. The lumps lie within the inner half of the texture, so the
				// sprite's octagon never cuts them off. Distances are in texture widths.
				struct Lump { float u, v, radius; };
				constexpr auto lump_count = 160;
				auto lumps = std::vector<Lump>{};
				for (auto i = 0; i != lump_count; ++i)
				{
					// Lumps near the centre are larger, so the clump is a dome of foam with smaller lumps around its edge.
					auto const r = 0.22f * std::sqrt(random(i, 0, 11));
					auto const a = 6.2831853f * random(i, 0, 12);
					auto const radius = std::lerp(0.05f, 0.015f, r / 0.22f) * (0.6f + 0.8f * random(i, 0, 13));
					lumps.push_back({ 0.5f + r * std::cos(a), 0.5f + r * std::sin(a), radius });
				}

				// Light comes from above and in front, so each lump has a bright top and a shaded underside. Texture v increases down the sprite.
				auto const lx = -0.35f;
				auto const ly = -0.75f;
				auto const lz = 0.56f;

				constexpr auto n = FrothTextureSize;
				auto result = std::vector<uint32_t>(static_cast<size_t>(n) * n);
				for (auto y = 0; y != n; ++y)
				{
					// Each row of texels.
					for (auto x = 0; x != n; ++x)
					{
						// Find the lump whose surface is nearest the viewer. Each lump is a half sphere, so its height is largest at its middle.
						auto const u = (x + 0.5f) / n;
						auto const v = (y + 0.5f) / n;
						auto height = 0.0f;
						auto cover = 0.0f;
						auto nx = 0.0f;
						auto ny = 0.0f;
						for (auto const& lump : lumps)
						{
							// Keep the lump with the highest surface over this texel.
							auto const dx = (u - lump.u) / lump.radius;
							auto const dy = (v - lump.v) / lump.radius;
							auto const d2 = dx * dx + dy * dy;
							if (d2 >= 1.0f)
								continue;

							auto const h = std::sqrt(1.0f - d2);
							if (h * lump.radius <= height)
								continue;

							height = h * lump.radius;
							cover = h;
							nx = dx;
							ny = dy;
						}

						// A flat layer of foam lies under the lumps and fills the gaps between them, so neighbouring clumps merge into continuous foam.
						// It is solid within the heap and soft at its edge, and its edge wanders in and out around the heap so the clump is not round.
						// Where no lump covers it, the layer faces the viewer and is shaded like a shallow crease.
						auto const ru = u - 0.5f;
						auto const rv = v - 0.5f;
						auto const theta = std::atan2(rv, ru);
						auto const edge = 0.25f + 0.035f * std::sin(3.0f * theta + 1.3f) + 0.025f * std::sin(5.0f * theta + 4.1f) + 0.015f * std::sin(9.0f * theta + 2.2f);
						auto const base = std::clamp((edge - std::sqrt(ru * ru + rv * rv)) / 0.05f, 0.0f, 1.0f);
						auto const on_lump = cover > 0.0f;

						// Shade the lump's surface by its slope to the light, and darken the creases between lumps, where the surface is steep.
						auto const nz = on_lump ? cover : 1.0f;
						auto const lit = std::max(0.0f, nx * lx + ny * ly + nz * lz);
						auto const crease = on_lump ? 0.6f + 0.4f * std::sqrt(cover) : 0.8f;
						auto const grain = 0.9f + 0.1f * random(x, y, 14);
						auto const grey = std::clamp((0.55f + 0.55f * lit) * crease * grain, 0.0f, 1.0f);

						// Each lump is solid in its middle and soft at its edge, so lumps blend into each other and the water without hard outlines.
						// Grain in the alpha speckles those edges, and scattered pin holes break up even the densest foam. Texels outside the heap
						// are empty.
						auto alpha = std::max(std::min(1.0f, 2.5f * std::sqrt(cover)), base) * (0.92f + 0.08f * random(x, y, 15));
						if (random(x, y, 16) < 0.02f)
							alpha *= 0.5f;

						auto const c = static_cast<uint32_t>(std::lround(grey * 255.0f));
						auto const al = static_cast<uint32_t>(std::lround(alpha * 255.0f));
						result[static_cast<size_t>(y) * n + x] = (al << 24) | (c << 16) | (c << 8) | c;
					}
				}
				return result;
			}();
			return texels;
		}

		// Texels per side of the dust texture.
		constexpr int DustTextureSize = 128;

		// Return the stock dust texture: a thin, wispy puff of smoke. It is white, so the sprite's colour tints it, and its alpha is the smoke's density.
		// The smoke is translucent even where it is thickest, and thins smoothly all the way out to near the sprite's rim, so overlapping puffs build
		// up into a cloud without any one puff showing an edge. Texels are R8G8B8A8 with straight alpha.
		std::vector<uint32_t> const& DustTexels()
		{
			static auto const texels = []
			{
				// Smooth noise in [0, 1): random values at the corners of a grid of 'cells' squares, blended smoothly across each square.
				auto const noise = [](float u, float v, int cells, uint32_t seed)
				{
					// Blend the four corners of the square containing (u, v) with a smooth step, so the noise has no creases.
					auto const x = u * cells;
					auto const y = v * cells;
					auto const x0 = static_cast<int>(std::floor(x));
					auto const y0 = static_cast<int>(std::floor(y));
					auto const fx = x - x0;
					auto const fy = y - y0;
					auto const sx = fx * fx * (3.0f - 2.0f * fx);
					auto const sy = fy * fy * (3.0f - 2.0f * fy);
					auto const top = std::lerp(TexelHash(x0, y0, seed), TexelHash(x0 + 1, y0, seed), sx);
					auto const bot = std::lerp(TexelHash(x0, y0 + 1, seed), TexelHash(x0 + 1, y0 + 1, seed), sx);
					return std::lerp(top, bot, sy);
				};

				// Wisps are noise at several scales, with finer scales adding less, so the smoke has soft swirls with fine detail on them.
				auto const wisps = [&](float u, float v, uint32_t seed)
				{
					// Four octaves, normalised back to [0, 1).
					auto sum = 0.0f;
					auto weight = 0.5f;
					auto cells = 4;
					for (auto octave = 0; octave != 4; ++octave, weight *= 0.5f, cells *= 2)
						sum += weight * noise(u, v, cells, seed + octave);

					return sum / 0.9375f;
				};

				// The puff is a loose cluster of broad, soft clumps spread over most of the sprite. Distances are in texture widths.
				struct Clump { float u, v, radius; };
				constexpr auto clump_count = 9;
				auto clumps = std::vector<Clump>{};
				for (auto i = 0; i != clump_count; ++i)
				{
					// Clumps stay inside the rim, so the sprite's octagon never cuts them off.
					auto const r = 0.2f * std::sqrt(TexelHash(i, 0, 21));
					auto const a = 6.2831853f * TexelHash(i, 0, 22);
					auto const radius = 0.14f + 0.1f * TexelHash(i, 0, 23);
					clumps.push_back({ 0.5f + r * std::cos(a), 0.5f + r * std::sin(a), radius });
				}

				constexpr auto n = DustTextureSize;
				auto result = std::vector<uint32_t>(static_cast<size_t>(n) * n);
				for (auto y = 0; y != n; ++y)
				{
					// Each row of texels.
					for (auto x = 0; x != n; ++x)
					{
						// The density is the sum of the clumps' soft falloffs.
						auto const u = (x + 0.5f) / n;
						auto const v = (y + 0.5f) / n;
						auto density = 0.0f;
						for (auto const& clump : clumps)
						{
							auto const du = (u - clump.u) / clump.radius;
							auto const dv = (v - clump.v) / clump.radius;
							density += std::exp(-1.5f * (du * du + dv * dv));
						}

						// The wisps cut the clumps into soft streaks with clear gaps between them, so the puff is ragged rather than round. Coverage
						// approaches full only gradually as the density grows, and is capped at about half opaque, so even the thickest smoke is
						// see-through. A smooth fade across the whole radius brings the smoke to zero just inside the rim.
						auto const ru = u - 0.5f;
						auto const rv = v - 0.5f;
						auto const radial = std::clamp(std::sqrt(ru * ru + rv * rv) / 0.48f, 0.0f, 1.0f);
						auto const edge = (1.0f - radial * radial) * (1.0f - radial * radial);
						auto const streak = std::clamp((wisps(u, v, 31) - 0.2f) / 0.6f, 0.0f, 1.0f);
						auto const swirl = streak * streak * (3.0f - 2.0f * streak);
						auto const thick = (1.0f - std::exp(-density * swirl)) * edge;
						auto const alpha = 0.55f * thick;

						// The smoke is a muted grey, lighter where it is thick and on top of the puff, and varies slightly with the wisps. Texture v
						// increases down the sprite, but the sprite spins, so the shading is gentle.
						auto const shade = std::clamp(0.68f + 0.1f * thick + 0.08f * (wisps(u, v, 41) - 0.5f) - 0.04f * (v - 0.5f), 0.0f, 1.0f);

						auto const c = static_cast<uint32_t>(std::lround(shade * 255.0f));
						auto const al = static_cast<uint32_t>(std::lround(alpha * 255.0f));
						result[static_cast<size_t>(y) * n + x] = (al << 24) | (c << 16) | (c << 8) | c;
					}
				}
				return result;
			}();
			return texels;
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
				.m_collision_set_offset = static_cast<int>(offsetof(ParticleSteppedSpriteEmitter, collision_set)),
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
				.m_collision_set_offset = static_cast<int>(offsetof(ParticleDebrisEmitter, collision_set)),
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
				// A burst of faint, smoke-like puffs is pushed out level with the ground as a ring, and rolls gently upward and swirls around as it
				// slows, spinning with the roll. The curl is half the drag, so the roll fades long before it could turn down. The puffs ignore gravity,
				// so they neither rise nor fall on their own, and the wind carries the fading cloud without bending its roll. Each puff's drag and roll
				// vary, so the cloud spreads and drifts apart in the wind.
				static ParticleSpriteConstants const constants = {
					.key_count = 3,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.0f,
					.keys = { { 0.0f, 0.5f, 0x00FFFFFF }, { 0.1f, 0.75f, 0x22FFFFFF }, { 1.0f, 1.0f, 0x00FFFFFF } },
				};
				static ParticleSteppedSpriteEmitter const params = {
					.position = { 0.0f, 0.0f, 0.0f, 2.0f },
					.velocity = { 0.0f, 0.0f, 0.0f, 0.3f },
					.accel = { 0.0f, 0.0f, 0.0f, 1.2f },
					.lifetime = { 2.0f, 3.5f },
					.size = { 4.0f, 14.0f },
					.colour0 = 0xFF655441,
					.colour1 = 0xFF786C59,
					.radius = 0.5f,
					.collision_set = -1,
					.gravity = 0.0f,
					.die_on_contact = 0,
					.outward = 6.0f,
					.curl = 0.6f,
					.swirl = 1.5f,
					.variety = 0.6f,
				};
				auto dust = SteppedSprite(constants, params, EParticleBlend::Weighted, 0.0f, 400);
				auto const& texels = DustTexels();
				dust.m_texels = texels.data();
				dust.m_texture_size = DustTextureSize;
				return dust;
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
				// Fine droplets are torn from the sea by the wind and thrown up and along it, streaking a little and fading as they fall back. Births are
				// spread evenly over the disc of sea, and only gales make droplets, so spray is a sign of a storm; the rate is the number of tries per
				// second. Droplets are launched upwards with a wide spread of speeds, and weak gravity lets them hang in the air before falling back.
				static ParticleSpriteConstants const constants = {
					.key_count = 3,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.04f,
					.keys = { { 0.0f, 0.6f, 0x00FFFFFF }, { 0.1f, 1.0f, 0xFFFFFFFF }, { 1.0f, 1.4f, 0x00FFFFFF } },
				};
				static ParticleSprayEmitter const params = {
					.area = { 0.0f, 0.0f, 0.0f, 60.0f },
					.launch = { 2.25f, 4.95f, 1.0f, 1.8f },
					.lifetime = { 1.0f, 2.5f },
					.size = { 0.04f, 0.14f },
					.colour0 = 0x52F4F8FF,
					.colour1 = 0x0FE0E8F0,
					.wind_range = { 25.5f, 40.5f },
					.lift = 0.95f,
					.gravity = 0.28f,
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
					.m_rate = 200.0f,
					.m_burst = 0,
				};
			}
			case EParticlePreset::Froth:
			{
				// Clumps of foam swell up where the ocean draws strong crest foam and ride the waves. Births use the ocean's
				// own crest foam (see ParticleWaterSample), so clumps form where the surface foam is added, and follow the ocean's foam settings. Crests
				// travel with the waves while clumps stay behind, so each clump is left on the back of its wave as the crest moves on, and fades over its
				// short linger time there. The foam minimum is low, so clumps form wherever the crest foam is clearly drawn, and the large clumps hide
				// the gaps between births. Clumps stay where they are born, with no drift or spread, and rise slowly out of the wave over their short lives.
				// Clumps rise out of the wave rather than float on it: the draw cuts each clump off at the water's surface, so only its upper part shows.
				// Clumps are wide and flat (squashed along the surface normal), so they lie low on the wave from the side, and they fade out with
				// distance from the camera, where their repeated texture would show.
				// Froth uses weighted blending, so clumps have soft edges and no hard outlines where they fade. The foam texture gives each clump its
				// lumpy shape and outline (see FrothTexels), and fills about half the sprite's width, so the sizes are about twice the clumps seen.
				// The curve only fades the colour, because froth swells and shrinks by its own half sine. The colours fade in over each clump's life.
				// The colour was tuned by eye to match the foam drawn on the ocean surface (see FoamColour in ocean_forward_ps.hlsli), and clumps are lit
				// with the surface normal, so clumps match the surface foam around them.
				static ParticleSpriteConstants const constants = {
					.key_count = 4,
					.flicker = 0.0f,
					.flicker_rate = 0.0f,
					.streak = 0.0f,
					.keys = { { 0.0f, 1.0f, 0x00FFFFFF }, { 0.08f, 1.0f, 0xFFFFFFFF }, { 0.6f, 1.0f, 0xFFFFFFFF }, { 1.0f, 1.0f, 0x00FFFFFF } },
				};
				static ParticleFrothEmitter const params = {
					.area = { 0.0f, 0.0f, 0.0f, 60.0f },
					.lifetime = { 0.5f, 0.5f },
					.size = { 15.0f, 15.0f },
					.colour0 = 0x00CDE1F5,
					.colour1 = 0x7FCDE1F5,
					.foam_min = 0.32f,
					.candidates = 32,
					.drift = 0.0f,
					.spread = 0.0f,
					.lift = 0.0f,
					.rise = 0.5f,
					.linger = 0.15f,
					.fade_near = 30.0f,
					.fade_far = 80.0f,
					.squash = 0.2f,
				};
				auto const& texels = FrothTexels();
				return ParticlePreset{
					.m_sim = ParticleSystem::FrothSimShaders(),
					.m_draw = ParticleSystem::FrothDrawShaders(),
					.m_blend = EParticleBlend::Weighted,
					.m_record_stride = sizeof(ParticleFrothRecord),
					.m_state_stride = sizeof(ParticleFrothState),
					.m_params_stride = sizeof(ParticleFrothEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
					.m_params = &params,
					.m_rate = 15500.0f,
					.m_burst = 0,
					.m_texels = texels.data(),
					.m_texture_size = FrothTextureSize,
				};
			}
			default:
			{
				throw std::invalid_argument(std::format("Unknown particle preset: {}", static_cast<int>(preset)));
			}
		}
	}
}
