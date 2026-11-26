//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/game-systems/forward.h"
#include "pr/game-systems/sky/lightning.h"
#include "game-systems/src/shaders/hlsl/sky/lightning_cbuf.hlsli"

#ifdef NDEBUG
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/release/##file)
#else
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/debug/##file)
#endif

namespace pr::game_systems
{
	namespace shader_code
	{
		// The bolt's build-time shader bytecode
		namespace compiled
		{
			#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(lightning_gs.h)
			#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(lightning_ps.h)
			#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(lightning_weighted_ps.h)
		}
		static compute::ByteCode const lightning_gs(compiled::lightning_gs);
		static compute::ByteCode const lightning_ps(compiled::lightning_ps);
		static compute::ByteCode const lightning_weighted_ps(compiled::lightning_weighted_ps);
	}

	namespace
	{
		// A small, fast random number generator. The same seed always gives the same sequence.
		struct Random
		{
			uint32_t m_state;

			// Mix the seed so that nearby seeds give unrelated sequences. The state must never be zero.
			explicit Random(uint32_t seed)
				: m_state()
			{
				// Scramble the seed bits, then force the state odd so it is never zero.
				auto h = seed * 0x9E3779B9u + 0x7F4A7C15u;
				h ^= h >> 16;
				h *= 0x85EBCA6Bu;
				h ^= h >> 13;
				h *= 0xC2B2AE35u;
				h ^= h >> 16;
				m_state = h | 1u;
			}

			// The next 32 random bits.
			uint32_t Next()
			{
				// Xorshift: three shifts give a full-period sequence over the nonzero states.
				m_state ^= m_state << 13;
				m_state ^= m_state >> 17;
				m_state ^= m_state << 5;
				return m_state;
			}

			// A random value in [0,1).
			float Unit()
			{
				// Use the top 24 bits, which fit exactly in a float.
				return s_cast<float>(Next() >> 8) * (1.0f / 16777216.0f);
			}

			// A random value in [lo,hi).
			float Range(float lo, float hi)
			{
				return lo + (hi - lo) * Unit();
			}

			// A random horizontal unit direction (Z = 0).
			v4 Horizontal()
			{
				auto angle = Range(0.0f, constants<float>::tau);
				return v4(std::cos(angle), std::sin(angle), 0, 0);
			}

			// A random unit direction.
			v4 Direction()
			{
				// Uniform on the sphere: a uniform height and a uniform angle around the vertical.
				auto z = Range(-1.0f, 1.0f);
				auto r = std::sqrt(std::max(0.0f, 1.0f - z * z));
				auto angle = Range(0.0f, constants<float>::tau);
				return v4(r * std::cos(angle), r * std::sin(angle), z, 0);
			}
		};

		// How a channel path is roughened, and where it may go.
		struct PathShape
		{
			// Segments shorter than this are not divided any further, in world units.
			float m_min_length;

			// The largest sideways offset of a segment's midpoint, as a fraction of the segment's length.
			float m_roughness;

			// Scale for the vertical part of the offsets. Paths inside the cloud wander mostly horizontally.
			float m_vertical;

			// The lowest and highest Z that the path may reach.
			float m_z_min;
			float m_z_max;
		};

		// Add the points of a jagged path from 'a' to 'b' to 'points', excluding 'a' and including 'b'.
		// Each segment's midpoint is moved a random distance sideways and the halves are divided again, which gives a path that is jagged at every scale.
		void JaggedPath(Random& rng, v4 const& a, v4 const& b, PathShape const& shape, std::vector<v4>& points)
		{
			// Short segments are kept straight.
			auto ab = b - a;
			auto length = Length(ab);
			if (length <= shape.m_min_length)
			{
				points.push_back(b);
				return;
			}

			// Move the midpoint in a random direction across the segment, by up to 'roughness' of the segment's length.
			auto dir = ab / length;
			auto side = rng.Direction();
			side = side - Dot(side, dir) * dir;
			auto side_len = Length(side);
			side = side_len > 1e-4f ? side / side_len : v4::Zero();
			side.z *= shape.m_vertical;
			auto mid = (a + b) * 0.5f + side * (length * shape.m_roughness * rng.Range(-1.0f, 1.0f));
			mid.z = std::clamp(mid.z, shape.m_z_min, shape.m_z_max);
			mid.w = 1.0f;

			// Divide both halves in the same way.
			JaggedPath(rng, a, mid, shape, points);
			JaggedPath(rng, mid, b, shape, points);
		}

		// Builds the channel tree of a bolt.
		struct BoltBuilder
		{
			std::vector<LightningNode>& m_nodes;
			Random& m_rng;

			// The ground point and the cloud base height.
			v4 m_ground;
			float m_base;

			// The speed of the leader along the channel, in world units per second.
			float m_speed;

			// The expected number of tendrils that grow from the main channel below the cloud.
			static constexpr float TendrilsPerBolt = 12.0f;

			// The deepest level of tendrils, and the expected number of smaller tendrils that grow from one tendril at each level above it.
			static constexpr int TendrilDepthMax = 4;
			static constexpr float SubTendrils[TendrilDepthMax - 1] = { 3.0f, 2.0f, 1.5f };

			// The expected number of dim branches inside the cloud.
			static constexpr float CloudBranches = 4.0f;

			// Tendrils grow this many times faster than the main leader, so most of them are complete when the leader reaches the ground.
			static constexpr float TendrilSpeed = 3.0f;

			// Tendrils stay inside an upright cone with its tip at the ground point, so they spread wide high up and only a little near the ground.
			// This is the angle between the cone's side and the vertical, in degrees.
			static constexpr float TendrilConeAngle = 40.0f;

			// Tendrils that would be shorter than this, in world units, are left out.
			static constexpr float TendrilLengthMin = 10.0f;

			// How far inside the tendril cone the point 'p' is, measured horizontally. Negative outside the cone.
			float ConeMargin(v4 const& p) const
			{
				// The cone's radius grows with height above the ground point.
				auto radius = (p.z - m_ground.z) * std::tan(TendrilConeAngle * constants<float>::tau / 360.0f);
				auto offset = p - m_ground;
				return radius - std::sqrt(offset.x * offset.x + offset.y * offset.y);
			}

			// The distance from 'start', inside the tendril cone, along the downward unit direction 'dir' to the cone's side.
			float DistanceToCone(v4 const& start, v4 const& dir) const
			{
				// The path leaves the cone before it reaches the ground's height, so a root lies between the start and that distance.
				// The margin along a straight path rises and then falls, so a binary search finds where it first leaves the cone.
				auto lo = 0.0f;
				auto hi = (start.z - m_ground.z) / -dir.z;
				for (int i = 0; i != 24; ++i)
				{
					auto mid = 0.5f * (lo + hi);
					if (ConeMargin(start + dir * mid) >= 0)
						lo = mid;
					else
						hi = mid;
				}
				return lo;
			}

			// The chance of a branch growing from each node of the chain of nodes [first, last), so that the expected number of branches is 'expected'.
			// Lightning forks where its path bends, so sharper corners are more likely to branch. Node 'i + 1' continues the chain from node 'i'.
			std::vector<float> CornerChances(int first, int last, float expected) const
			{
				// Measure how sharply the path turns at each node, from 0 for straight on to 1 for doubling back.
				std::vector<float> chances(std::max(0, last - first));
				auto total = 0.0f;
				for (int i = first; i != last; ++i)
				{
					auto const& node = m_nodes[i];
					auto d_in = Normalise(node.m_pos - m_nodes[node.m_parent].m_pos, v4::Zero());
					auto d_out = Normalise(m_nodes[i + 1].m_pos - node.m_pos, v4::Zero());
					chances[i - first] = 0.5f * (1.0f - Dot(d_in, d_out));
					total += chances[i - first];
				}

				// Scale the sharpness so the chances add up to the expected number of branches.
				auto scale = total > 0 ? expected / total : 0.0f;
				for (auto& c : chances)
					c = std::min(1.0f, c * scale);

				return chances;
			}

			// Add a path of points as a chain of nodes that grows from node 'parent'. Returns the index of the first new node.
			// Arrival times continue from the parent's at 'speed_scale' times the leader's speed. Brightness tapers along the chain from 'brightness' to 'brightness * taper'.
			int AddChain(int parent, std::span<v4 const> points, float brightness, float taper, float width, float speed_scale, bool in_cloud)
			{
				// Each point joins the previous one, so arrival times follow the distance along the chain.
				auto speed = m_speed * speed_scale;
				auto first = isize(m_nodes);
				auto prev = parent;
				auto distance = 0.0f;
				auto total = 0.0f;
				for (int i = 0; i != isize(points); ++i)
				{
					auto from = i == 0 ? m_nodes[parent].m_pos : points[i - 1];
					total += Length(points[i] - from);
				}
				for (int i = 0; i != isize(points); ++i)
				{
					// Copy the parent's values, because adding nodes may move the node storage.
					auto from = m_nodes[prev].m_pos;
					auto arrival = m_nodes[parent].m_arrival;
					distance += Length(points[i] - from);
					auto t = total > 0 ? distance / total : 1.0f;
					m_nodes.push_back(LightningNode{
						.m_pos = points[i],
						.m_arrival = arrival + distance / speed,
						.m_brightness = brightness * Lerp(1.0f, taper, t),
						.m_width = width,
						.m_parent = prev,
						.m_in_cloud = in_cloud,
					});
					prev = isize(m_nodes) - 1;
				}
				return first;
			}

			// Grow a faint tendril below the cloud from node 'parent', reaching down and out toward the horizontal direction 'heading' as far as
			// the tendril cone, and maybe smaller tendrils from it. 'depth' is 1 for a tendril of the main channel.
			void Tendril(int parent, v4 const& heading, int depth)
			{
				// The tendril leans 25 to 70 degrees from straight down toward its heading.
				auto start = m_nodes[parent].m_pos;
				auto parent_brightness = m_nodes[parent].m_brightness;
				auto parent_width = m_nodes[parent].m_width;
				auto angle = m_rng.Range(25.0f, 70.0f) * constants<float>::tau / 360.0f;
				auto dir = v4(0, 0, -std::cos(angle), 0) + heading * std::sin(angle);

				// Tendrils reach most of the way to the cone's side, and each level is shorter than the one it grows from. Tendrils that start
				// outside the cone, or have too little room before its side, are left out.
				if (ConeMargin(start) <= 0)
					return;

				auto length = DistanceToCone(start, dir) * m_rng.Range(0.5f, 1.0f) * std::pow(0.7f, s_cast<float>(depth - 1));
				if (length < TendrilLengthMin)
					return;

				auto end = start + dir * length;
				end.w = 1.0f;

				// Long tendrils use longer straight steps, which bounds the number of nodes in each one. Each level is dimmer and thinner than its parent.
				std::vector<v4> points;
				auto min_length = std::max(8.0f, length / 16.0f);
				JaggedPath(m_rng, start, end, PathShape{ .m_min_length = min_length, .m_roughness = 0.3f, .m_vertical = 1.0f, .m_z_min = m_ground.z, .m_z_max = m_base }, points);
				auto brightness = parent_brightness * (depth == 1 ? 0.35f : 0.5f);
				auto width = parent_width * 0.35f;
				auto first = AddChain(parent, points, brightness, 0.3f, width, TendrilSpeed, false);
				auto last = isize(m_nodes);
				if (depth >= TendrilDepthMax)
					return;

				// Smaller tendrils grow from the corners along this one, except its tip, turned up to 60 degrees either side of its heading.
				auto chances = CornerChances(first, last - 1, SubTendrils[depth - 1]);
				for (int i = first; i != last - 1; ++i)
				{
					// Each node may start a smaller tendril, with its heading turned about the vertical.
					if (m_rng.Unit() >= chances[i - first])
						continue;

					auto turn = m_rng.Range(-60.0f, 60.0f) * constants<float>::tau / 360.0f;
					auto c = std::cos(turn);
					auto s = std::sin(turn);
					auto sub_heading = v4(heading.x * c - heading.y * s, heading.x * s + heading.y * c, 0, 0);
					Tendril(i, sub_heading, depth + 1);
				}
			}

			// Grow a dim, mostly horizontal branch inside the cloud from node 'parent'.
			void CloudBranch(int parent)
			{
				// Cloud branches spread sideways through the cloud.
				auto start = m_nodes[parent].m_pos;
				auto width = m_nodes[parent].m_width * 0.4f;
				auto end = start + m_rng.Horizontal() * m_rng.Range(200.0f, 800.0f);
				end.z = std::clamp(start.z + m_rng.Range(-100.0f, 100.0f), m_base, m_base + 800.0f);
				end.w = 1.0f;

				std::vector<v4> points;
				JaggedPath(m_rng, start, end, PathShape{ .m_min_length = 40.0f, .m_roughness = 0.25f, .m_vertical = 0.3f, .m_z_min = m_base, .m_z_max = m_base + 800.0f }, points);
				AddChain(parent, points, 0.5f, 0.4f, width, 1.0f, true);
			}
		};
	}

	// Grow the channel from the cloud to the ground, add branches, and choose the timing of the return strokes.
	LightningBolt::LightningBolt(v4 const& ground, float cloud_base, uint32_t seed)
		: m_nodes()
		, m_origin()
		, m_exit()
		, m_ground(ground.w1())
		, m_leader_time()
		, m_stroke_count()
		, m_stroke_times()
		, m_stroke_strengths()
		, m_duration()
	{
		// Callers validate their inputs; these only catch programming errors.
		assert(IsFinite(ground) && std::isfinite(cloud_base) && cloud_base > ground.z);
		Random rng(seed);

		// The bolt starts high in the cloud, some way from the strike, and travels through the cloud to leave the cloud base near the strike.
		auto start_dir = rng.Horizontal();
		m_origin = m_ground + start_dir * rng.Range(500.0f, 3000.0f);
		m_origin.z = cloud_base + rng.Range(200.0f, 600.0f);
		m_exit = m_ground + rng.Horizontal() * rng.Range(100.0f, 300.0f);
		m_exit.z = cloud_base;

		// Roughen the main channel: wide wandering inside the cloud, and finer, mostly downward steps in the air below it.
		std::vector<v4> cloud_points;
		std::vector<v4> air_points;
		JaggedPath(rng, m_origin, m_exit, PathShape{ .m_min_length = 40.0f, .m_roughness = 0.2f, .m_vertical = 0.3f, .m_z_min = cloud_base, .m_z_max = cloud_base + 800.0f }, cloud_points);
		JaggedPath(rng, m_exit, m_ground, PathShape{ .m_min_length = 15.0f, .m_roughness = 0.22f, .m_vertical = 1.0f, .m_z_min = m_ground.z, .m_z_max = cloud_base }, air_points);

		// The leader crosses the whole main channel in 50 to 90 milliseconds. Branches grow at the same speed.
		auto main_length = 0.0f;
		auto prev = m_origin;
		for (auto const& p : cloud_points)
		{
			main_length += Length(p - prev);
			prev = p;
		}
		for (auto const& p : air_points)
		{
			main_length += Length(p - prev);
			prev = p;
		}
		m_leader_time = rng.Range(0.05f, 0.09f);

		// Add the main channel. Its last cloud point is the exit, which is outside the cloud.
		auto main_width = rng.Range(2.5f, 3.5f);
		m_nodes.reserve(8192);
		m_nodes.push_back(LightningNode{ .m_pos = m_origin, .m_arrival = 0, .m_brightness = 1, .m_width = main_width, .m_parent = -1, .m_in_cloud = true });
		BoltBuilder builder{ .m_nodes = m_nodes, .m_rng = rng, .m_ground = m_ground, .m_base = cloud_base, .m_speed = main_length / m_leader_time };
		auto cloud_first = builder.AddChain(0, cloud_points, 1.0f, 1.0f, main_width, 1.0f, true);
		m_nodes.back().m_in_cloud = false;
		auto air_first = builder.AddChain(isize(m_nodes) - 1, air_points, 1.0f, 1.0f, main_width, 1.0f, false);
		auto main_end = isize(m_nodes);

		// Add a few dim branches inside the cloud. The expected number does not depend on how finely the channel is divided.
		auto cloud_chance = BoltBuilder::CloudBranches / s_cast<float>(std::max(1, air_first - 1 - cloud_first));
		for (int i = cloud_first; i != air_first - 1; ++i)
		{
			if (rng.Unit() < cloud_chance)
				builder.CloudBranch(i);
		}

		// Add faint tendrils below the cloud, spreading in random directions from the corners of the main channel, all the way down to the ground.
		auto tendril_chances = builder.CornerChances(air_first, main_end - 1, BoltBuilder::TendrilsPerBolt);
		for (int i = air_first; i != main_end - 1; ++i)
		{
			if (rng.Unit() < tendril_chances[i - air_first])
				builder.Tendril(i, rng.Horizontal(), 1);
		}
		assert(isize(m_nodes) < 0xFFFF && "Lightning bolt has too many nodes for 16-bit indices");

		// One to three return strokes. The first is the brightest; later ones follow the same channel after short gaps.
		m_stroke_count = 1 + s_cast<int>(rng.Next() % StrokesMax);
		m_stroke_times[0] = m_leader_time;
		m_stroke_strengths[0] = 1.0f;
		for (int i = 1; i != m_stroke_count; ++i)
		{
			m_stroke_times[i] = m_stroke_times[i - 1] + rng.Range(0.06f, 0.15f);
			m_stroke_strengths[i] = rng.Range(0.4f, 0.8f);
		}
		// The strike lasts through the first stroke's hold, then fades over the same time whatever the hold.
		m_duration = m_stroke_times[m_stroke_count - 1] + StrokeHold + rng.Range(0.5f, 0.7f);
	}

	// The leader glows dimly while it grows. Each return stroke flashes, holds and decays, on top of a steady fade of the whole channel.
	LightningLight LightningBolt::LightAt(float age) const
	{
		// Nothing is visible before the strike starts or after it ends.
		if (!(age >= 0 && age < m_duration))
			return LightningLight{ .m_glow = 0, .m_fade = 0, .m_dazzle = 0, .m_flash = 0, .m_leader = false };

		// The leader is faint and does not dazzle.
		if (age < m_leader_time)
			return LightningLight{ .m_glow = 0.12f, .m_fade = 0, .m_dazzle = 0, .m_flash = 0, .m_leader = true };

		// Sum the decaying flashes of the strokes that have started. Each stroke dazzles fully for a moment, then the dazzle decays.
		auto pulse = 0.0f;
		auto dazzle = 0.0f;
		for (int i = 0; i != m_stroke_count; ++i)
		{
			auto since = age - m_stroke_times[i];
			if (since < 0)
				continue;

			pulse += m_stroke_strengths[i] * std::exp(-std::max(since - StrokeHold, 0.0f) / StrokeDecay);
			auto held = since < DazzleHold ? 1.0f : std::exp(-(since - DazzleHold) / DazzleDecay);
			dazzle = std::max(dazzle, m_stroke_strengths[i] * held);
		}

		// The channel keeps its full width through the first stroke's hold. Its steady light then fades to nothing at the end of the strike.
		auto fade = std::max(age - m_leader_time - StrokeHold, 0.0f) / (m_duration - m_leader_time - StrokeHold);
		auto glow = 0.4f * std::sqrt(1.0f - fade) + 0.6f * pulse;
		return LightningLight{ .m_glow = glow, .m_fade = fade, .m_dazzle = std::min(dazzle, 1.0f), .m_flash = std::min(pulse, 1.0f), .m_leader = false };
	}

	// Replaces the forward geometry and pixel stages, and keeps the forward vertex shader, using the overlay constant buffer slot.
	struct LightningShader : Shader
	{
		sky::CBufLightning m_cbuf;

		// An icy white bolt that is not yet visible.
		explicit LightningShader(Renderer& rdr)
			: Shader(rdr)
			, m_cbuf{
				.screen_dim = v4::Zero(),
				.colour = v4(0.85f, 0.92f, 1.0f, 1.0f),
				.age = 0,
				.leader_time = 0,
				.glow = 0,
				.fade = 0,
				.min_pixels = 2.0f,
				.thin_pixels = 0.5f,
				.pad1 = 0,
				.pad2 = 0,
			}
		{
			// The pixel stage is chosen per forward sub-pass: weighted transparency normally, or the stock alpha blend without transparency buffers.
			m_code = ShaderCode{
				.VS = rdr12::shader_code::none,
				.PS = rdr12::shader_code::none,
				.DS = rdr12::shader_code::none,
				.HS = rdr12::shader_code::none,
				.GS = shader_code::lightning_gs,
				.CS = rdr12::shader_code::none,
			};
			m_pixel_family = ForwardPixelFamily{
				.m_code = { shader_code::lightning_ps, rdr12::shader_code::none, rdr12::shader_code::none, shader_code::lightning_weighted_ps },
				.m_replaces = &rdr12::shader_code::forward_family,
			};
		}

		// Bind the bolt's constants, with the size of the render target and the camera's scale for this draw.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& cam, DrawListElement const*) override
		{
			// The camera-to-screen projection's X scale converts a width at clip w = 1 to a fraction of half the screen width.
			auto size = To<v2>(scene.wnd().RenderTargetSize());
			auto c2s = cam.m_w2s * InvertOrthonormal(cam.m_w2c);
			m_cbuf.screen_dim = v4(size.x, size.y, c2s.x.x * size.x * 0.5f, 0);
			auto gpu_address = upload.Add(m_cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true).VirtualAddress();
			cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(shaders::fwd::ERootParam::CBufScreenSpace), gpu_address);
		}
	};

	// One segment per non-origin node, from its parent to it, with its chain neighbours for joining. The vertex colour carries the node's timing and appearance to the shaders.
	LightningStrike::LightningStrike(ResourceFactory& factory, LightningBolt bolt, double start_time)
		: m_bolt(std::move(bolt))
		, m_start_time(start_time)
		, m_inst()
		, m_shader()
	{
		// Vertices are the nodes. Channel inside the cloud is seen dimly through it.
		auto const& nodes = m_bolt.m_nodes;
		auto count = isize(nodes);
		assert(count >= 2 && count < 0xFFFF);
		ModelGenerator::Buffers<Vert> buf;
		buf.Reset(count, 0, 0, sizeof(uint16_t));
		buf.m_bbox = BBox::Reset();
		for (int i = 0; i != count; ++i)
		{
			auto const& node = nodes[i];
			auto& v = buf.m_vcont[i];
			v.m_vert = node.m_pos;
			v.m_diff = Colour(node.m_arrival, node.m_brightness, node.m_width, node.m_in_cloud ? 0.3f : 1.0f);
			v.m_norm = v4::Zero();
			v.m_tex0 = v2::Zero();
			v.m_idx0 = iv2::Zero();
			Grow(buf.m_bbox, node.m_pos);
		}
		// Each segment is drawn with its neighbours in the same chain, so the shader can divide the round ends that they share. A chain continues
		// through a node into the node added right after it, because AddChain adds the nodes of a chain in order. Branches start elsewhere.
		// Where a chain does not continue, the segment's own end node stands in for the neighbour, which gives that end a whole round end.
		auto next_in_chain = [&](int i)
		{
			return i + 1 != count && nodes[i + 1].m_parent == i ? i + 1 : i;
		};
		for (int i = 0; i != count; ++i)
		{
			// The origin has no segment of its own.
			auto parent = nodes[i].m_parent;
			if (parent < 0)
				continue;

			auto prev = next_in_chain(parent) == i && nodes[parent].m_parent >= 0 ? nodes[parent].m_parent : parent;
			buf.m_icont.push_back(s_cast<uint16_t>(prev));
			buf.m_icont.push_back(s_cast<uint16_t>(parent));
			buf.m_icont.push_back(s_cast<uint16_t>(i));
			buf.m_icont.push_back(s_cast<uint16_t>(next_in_chain(i)));
		}

		// The material retains the shader, so its pointer remains valid for the lifetime of the model.
		// The bolt is transparent, so it draws with the other transparent surfaces and blends correctly over water. It draws in the weighted
		// (order-independent) transparency group, which shows it in front of the clouds but hides it behind nearer opaque geometry (see lightning.hlsl).
		auto shdr = Shader::Create<LightningShader>(factory.rdr());
		m_shader = shdr.get();
		buf.m_ncont.push_back(
			NuggetDesc(ETopo::LineListAdj, EGeom::Vert | EGeom::Colr)
				.flags(ENuggetFlag::ShadowCastExclude | ENuggetFlag::GeometryHasAlpha | ENuggetFlag::AlphaWeighted)
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		ModelGenerator::Cache cache{buf};
		m_inst.m_model = ModelGenerator::Create<Vert>(factory, cache);
		m_inst.m_i2w = m4x4::Identity();
	}

	// Choose the light for this moment and pass it to the shader.
	LightningLight LightningStrike::Show(float age)
	{
		auto light = m_bolt.LightAt(age);
		m_shader->m_cbuf.age = age;
		m_shader->m_cbuf.leader_time = m_bolt.m_leader_time;
		m_shader->m_cbuf.glow = light.m_glow;
		m_shader->m_cbuf.fade = light.m_fade;
		return light;
	}
}
