#include "pr/physics/utility/ldraw.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "src/scene/scene.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	namespace
	{
		constexpr auto ContactPriorityFallbackColour = Colour32(0xFFFF8000U);

		using Clock = std::chrono::steady_clock;

		double ElapsedMs(Clock::time_point beg, Clock::time_point end)
		{
			return std::chrono::duration<double, std::milli>(end - beg).count();
		}
		void AddProfile(physics::Engine::StepProfile& lhs, physics::Engine::StepProfile const& rhs)
		{
			lhs.m_new_frame_ms += rhs.m_new_frame_ms;
			lhs.m_pack_ms += rhs.m_pack_ms;
			lhs.m_upload_ms += rhs.m_upload_ms;
			lhs.m_external_forces_ms += rhs.m_external_forces_ms;
			lhs.m_integrate_ms += rhs.m_integrate_ms;
			lhs.m_sleepwake_ms += rhs.m_sleepwake_ms;
			lhs.m_broadphase_ms += rhs.m_broadphase_ms;
			lhs.m_collide_ms += rhs.m_collide_ms;
			lhs.m_resolve_ms += rhs.m_resolve_ms;
			lhs.m_selective_ms += rhs.m_selective_ms;
			lhs.m_sleepupdate_ms += rhs.m_sleepupdate_ms;
			lhs.m_readback_ms += rhs.m_readback_ms;
			lhs.m_gpu_run_ms += rhs.m_gpu_run_ms;
			lhs.m_gpu_prepare_ms += rhs.m_gpu_prepare_ms;
			lhs.m_gpu_execute_ms += rhs.m_gpu_execute_ms;
			lhs.m_gpu_wait_ms += rhs.m_gpu_wait_ms;
			lhs.m_gpu_reset_ms += rhs.m_gpu_reset_ms;
			lhs.m_unpack_ms += rhs.m_unpack_ms;
			lhs.m_readback_access_ms += rhs.m_readback_access_ms;
			lhs.m_body_readback_copy_ms += rhs.m_body_readback_copy_ms;
			lhs.m_contact_readback_copy_ms += rhs.m_contact_readback_copy_ms;
			lhs.m_collision_events_ms += rhs.m_collision_events_ms;
			lhs.m_sleep_island_unpack_ms += rhs.m_sleep_island_unpack_ms;
			lhs.m_body_unpack_ms += rhs.m_body_unpack_ms;
			lhs.m_unpack_diagnostics_ms += rhs.m_unpack_diagnostics_ms;
		}
		bool SameVec(v4 const& lhs, v4 const& rhs)
		{
			return std::memcmp(&lhs, &rhs, sizeof(v4)) == 0;
		}
		Colour32 ContactPriorityColour(int order_idx, int order_count)
		{
			auto const t = order_count > 1 ? static_cast<float>(order_idx) / static_cast<float>(order_count - 1) : 0.0f;
			return Colour32(0, static_cast<int>(128.0f + 127.0f * std::clamp(t, 0.0f, 1.0f)), 255, 255);
		}
		// Return the water mesh extent, using the scene bounds when the scene leaves the visual size unspecified.
		v2 WaterExtent(scene_loader::WaterDesc const& water, BBox const& scene_bbox)
		{
			if (water.size.x > 0.0f && water.size.y > 0.0f)
				return water.size;

			if (scene_bbox.valid())
			{
				auto extent = 4.0f * Max(scene_bbox.Radius().xy, v2{ 5.0f, 5.0f });
				if (extent.x > 0.0f && extent.y > 0.0f && IsFinite(extent))
					return extent;
			}

			return v2{ 20.0f, 20.0f };
		}
		bool SameShapeDesc(scene_loader::BodyDesc const& lhs, scene_loader::BodyDesc const& rhs)
		{
			if (lhs.shape_type != rhs.shape_type)
				return false;

			switch (lhs.shape_type)
			{
				case scene_loader::BodyDesc::EShape::Box:
				{
					return SameVec(lhs.box_dimensions, rhs.box_dimensions);
				}
				case scene_loader::BodyDesc::EShape::Sphere:
				{
					return lhs.sphere_radius == rhs.sphere_radius;
				}
				case scene_loader::BodyDesc::EShape::Line:
				{
					return lhs.line_length == rhs.line_length && lhs.line_thickness == rhs.line_thickness;
				}
				case scene_loader::BodyDesc::EShape::Triangle:
				{
					return SameVec(lhs.tri_verts[0], rhs.tri_verts[0]) && SameVec(lhs.tri_verts[1], rhs.tri_verts[1]) && SameVec(lhs.tri_verts[2], rhs.tri_verts[2]);
				}
				case scene_loader::BodyDesc::EShape::Polytope:
				{
					if (lhs.polytope_verts.size() != rhs.polytope_verts.size())
						return false;

					return std::memcmp(lhs.polytope_verts.data(), rhs.polytope_verts.data(), lhs.polytope_verts.size() * sizeof(v4)) == 0;
				}
				default:
				{
					throw std::runtime_error("Unknown shape type in scene description");
				}
			}
		}
		void AppendShape(byte_data<16>& shape_buffer, scene_loader::BodyDesc const& bd)
		{
			auto ofs = shape_buffer.size();
			switch (bd.shape_type)
			{
				case scene_loader::BodyDesc::EShape::Box:
				{
					shape_buffer.push_back(collision::ShapeBox(bd.box_dimensions));
					break;
				}
				case scene_loader::BodyDesc::EShape::Sphere:
				{
					shape_buffer.push_back(collision::ShapeSphere(bd.sphere_radius));
					break;
				}
				case scene_loader::BodyDesc::EShape::Line:
				{
					shape_buffer.push_back(collision::ShapeLine(bd.line_length, bd.line_thickness));
					break;
				}
				case scene_loader::BodyDesc::EShape::Triangle:
				{
					shape_buffer.push_back(collision::ShapeTriangle(bd.tri_verts[0], bd.tri_verts[1], bd.tri_verts[2]));
					break;
				}
				case scene_loader::BodyDesc::EShape::Polytope:
				{
					shape_buffer.push_back(collision::BuildPolytopeFromPoints(bd.polytope_verts));
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown shape type in scene description");
				}
			}

			// Pad to 16-byte alignment and update the shape's m_size to include the padding.
			// collision::next() uses m_size to advance the shape pointer, so it must account
			// for any alignment padding between shapes.
			shape_buffer.pad_to(16);
			shape_buffer.at_byte_ofs<collision::Shape>(ofs).m_size = s_cast<int>(shape_buffer.size() - ofs);
		}
		m4x4 PrimitiveShapeToBody(Body const& body)
		{
			using namespace collision;

			auto& shape = body.Shape();
			switch (shape.m_type)
			{
				case EShape::Sphere:
				{
					auto& sphere = shape_cast<ShapeSphere>(shape);
					return sphere.m_base.m_s2r * m4x4::Scale(sphere.m_radius, v4::Origin());
				}
				case EShape::Box:
				{
					auto& box = shape_cast<ShapeBox>(shape);
					return box.m_base.m_s2r * m4x4::Scale(box.m_radius.x, box.m_radius.y, box.m_radius.z, v4::Origin());
				}
				case EShape::Line:
				{
					auto& line = shape_cast<ShapeLine>(shape);
					if (line.m_radius != 0)
						return line.m_base.m_s2r * m4x4::Scale(line.m_radius, line.m_radius, line.m_hlength, v4::Origin());

					return line.m_base.m_s2r * m4x4::Scale(1.0f, 1.0f, line.m_hlength, v4::Origin());
				}
				default:
				{
					throw std::runtime_error("Unsupported shared primitive shape type");
				}
			}
		}
	}

	Scene::Scene(rdr12::Renderer* rdr)
		: m_rdr(rdr)
		, m_shader_cache(AppDataPath() / "shader_cache", "physics-sandbox")
		, m_physics(physics::EngineConfig{}, &m_shader_cache, rdr ? rdr->d3d() : nullptr)
		, m_box(v4{ 2, 2, 2, 0 })
		, m_body()
		, m_shape_buffer()
		, m_gpu_buoyancy()
		, m_buoyancy_hulls()
		, m_buoyancy_body_indices()
		, m_buoyancy_generation()
		, m_buoyancy_debug_shapes()
		, m_show_buoyancy_debug(false)
		, m_buoyancy_debug_gfx()
		, m_gravity(v4::Zero())
		, m_kill_zone_height(-100.0f)
		, m_physics_substeps(1)
		, m_allow_sleeping(true)
		, m_ground_gfx()
		, m_water()
		, m_water_gfx()
		, m_env_map()
		, m_sky_gfx()
		, m_origin_gfx()
		, m_contacts_gfx()
		, m_visual_mode(EVisualMode::Normal)
		, m_collision_sub()
		, m_show_contacts(true)
		, m_clock()
		, m_step_pending(false)
		, m_pending_elapsed_seconds()
		, m_pending_substeps()
		, m_pending_step_profile()
		, m_current_scenario()
		, m_diag()
		, m_step_count()
	{
		UpdateCollisionReadback();

		// Create a coordinate frame at the origin for visual reference
		if (m_rdr)
		{
			ldraw::Builder ldr;
			ldr.CoordFrame("origin").scale(3.0f).width(2.0f);
			auto result = rdr12::ldraw::Parse(*m_rdr, ldr.ToBinary());
			if (!result.m_objects.empty())
				m_origin_gfx = result.m_objects.front();
		}
	}

	// Release all scene-owned diagnostic buoyancy resources before replacing bodies or the module.
	void Scene::ClearBuoyancy()
	{
		m_buoyancy_hulls.clear();
		m_buoyancy_body_indices.clear();
		m_buoyancy_debug_shapes.clear();
		m_buoyancy_debug_gfx = nullptr;
		m_gpu_buoyancy.reset();
		++m_buoyancy_generation;
	}

	// Create diagnostic buoyancy hulls described by a loaded scene.
	void Scene::ConfigureBuoyancy(scene_loader::SceneDesc const& scene_desc)
	{
		if (scene_desc.buoyancy_hulls.empty())
			return;

		auto body_lookup = std::unordered_map<std::string, int>{};
		body_lookup.reserve(scene_desc.bodies.size());
		for (int body_index = 0; body_index != isize(scene_desc.bodies); ++body_index)
		{
			auto const& body_name = scene_desc.bodies[body_index].name;
			if (body_name.empty())
				throw std::runtime_error("Buoyancy-enabled scenes require named bodies");

			auto const [_, inserted] = body_lookup.emplace(body_name, body_index);
			if (!inserted)
				throw std::runtime_error(pr::FmtS("Buoyancy-enabled scene has duplicate body name '%s'", body_name.c_str()));
		}

		m_gpu_buoyancy = std::make_unique<physics::GpuBuoyancy>(
			m_physics.Device(),
			m_physics,
			physics::GpuBuoyancy::Config{},
			[this](int stable_body_index)
			{
				return stable_body_index >= 0 && stable_body_index < isize(m_body)
					? stable_body_index
					: -1;
			},
			[this](int stable_body_index)
			{
				auto body_state = physics::GpuBuoyancy::BodyState{};
				if (stable_body_index < 0 || stable_body_index >= isize(m_body))
					return body_state;

				auto const& body = m_body[stable_body_index];
				body_state.m_o2w = body.O2W();
				body_state.m_centre_of_mass_os = body.CentreOfMassOS();
				body_state.m_ws_gravity = body.GravityWS();
				body_state.m_valid = true;
				return body_state;
			});
		if (scene_desc.water)
			m_gpu_buoyancy->SetWaterSurface(scene_desc.water->surface);

		m_buoyancy_hulls.reserve(scene_desc.buoyancy_hulls.size());
		m_buoyancy_body_indices.reserve(scene_desc.buoyancy_hulls.size());
		for (auto const& hull : scene_desc.buoyancy_hulls)
		{
			auto const iter = body_lookup.find(hull.body_name);
			if (iter == body_lookup.end())
				throw std::runtime_error(pr::FmtS("Buoyancy hull references unknown body '%s'", hull.body_name.c_str()));

			// Build a transient collision shape from the hull description and register it on the
			// sampled-composite backend, which flattens it into the volume-sample primitive set.
			// The shape is only read during registration, so the locals below only need to stay
			// alive across the RegisterCompositeHull call. Polytope hulls are tessellated so the
			// volume sampler has interior tets to integrate over (untessellated polytopes throw).
			auto& body = m_body[iter->second];
			physics::GpuBuoyancy::Registration registration;

			// Retain a copy of the registered collision shape so BuildBuoyancyDebugGfx() can re-run the
			// CPU oracle over the same geometry. Filled per shape-type case below and moved into the
			// parallel m_buoyancy_debug_shapes array after registration.
			byte_data<16> debug_shape;
			switch (hull.shape_type)
			{
				case scene_loader::BuoyancyHullDesc::EShape::Box:
				{
					auto hull_shape = collision::ShapeBox(hull.dimensions);
					registration = m_gpu_buoyancy->RegisterCompositeHull(body, iter->second, m_buoyancy_generation, collision::shape_cast(hull_shape));
					debug_shape.push_back(hull_shape);
					DbgLog("  Buoyancy: body '%s' composite box hull dimensions=(%.3f, %.3f, %.3f)\n", hull.body_name.c_str(), hull.dimensions.x, hull.dimensions.y, hull.dimensions.z);
					break;
				}
				case scene_loader::BuoyancyHullDesc::EShape::Sphere:
				{
					auto hull_shape = collision::ShapeSphere(hull.radius);
					registration = m_gpu_buoyancy->RegisterCompositeHull(body, iter->second, m_buoyancy_generation, collision::shape_cast(hull_shape));
					debug_shape.push_back(hull_shape);
					DbgLog("  Buoyancy: body '%s' composite sphere hull radius=%.3f\n", hull.body_name.c_str(), hull.radius);
					break;
				}
				case scene_loader::BuoyancyHullDesc::EShape::Polytope:
				{
					auto hull_buffer = collision::BuildPolytopeFromPoints(hull.polytope_verts, m4x4::Identity(), 0, collision::Shape::EFlags::None, hull.tessellation);
					auto const& hull_shape = hull_buffer.as<collision::ShapePolytope>();
					registration = m_gpu_buoyancy->RegisterCompositeHull(body, iter->second, m_buoyancy_generation, collision::shape_cast(hull_shape));
					DbgLog("  Buoyancy: body '%s' composite polytope hull verts=%d tets=%d\n", hull.body_name.c_str(), s_cast<int>(hull.polytope_verts.size()), hull_shape.m_tet_count);
					debug_shape = std::move(hull_buffer);
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown buoyancy hull shape type in scene description");
				}
			}
			m_buoyancy_hulls.push_back(std::move(registration));
			m_buoyancy_body_indices.push_back(iter->second);
			m_buoyancy_debug_shapes.push_back(std::move(debug_shape));
		}
	}

	// Create the scene-owned water mesh visual from the loaded water surface description.
	void Scene::CreateWaterGfx(scene_loader::WaterDesc const& water, BBox const& scene_bbox)
	{
		if (m_rdr == nullptr)
			return;

		auto const extent = WaterExtent(water, scene_bbox);
		m_water_gfx = std::make_unique<WaterVisual>(*m_rdr, water, extent);

		// Build the procedural environment cube + matching skybox model. The same six face images feed both
		// the cube map (sampled by reflective surfaces) and the skybox model (rendered as a backdrop centred
		// on the camera). Faces are stored as sRGB-encoded BGRA8 bytes, with the SRV format cast to *_SRGB
		// so the GPU decodes back to linear on sample.
		rdr12::ResourceFactory factory(*m_rdr);
		procedural_sky::SkyDesc sky_desc{};
		auto sky_faces = procedural_sky::GenerateSkyFaces(256, sky_desc);

		// Wrap each face as an Image so the resource factory can build a Texture2D / Texture cube.
		std::array<::pr::compute::Image, 6> face_images = {};
		for (size_t i = 0; i != face_images.size(); ++i)
			face_images[i] = sky_faces[i];

		// Cube map: single GPU resource shared as scene.m_global_envmap.
		auto cube_desc = rdr12::TextureDesc(rdr12::AutoId, ::pr::compute::ResDesc()).name("env_sky").srv_format(DXGI_FORMAT_B8G8R8A8_UNORM_SRGB);
		m_env_map = factory.CreateTextureCube(std::span<::pr::compute::Image const>(face_images.data(), face_images.size()), cube_desc);

		// Build 6 individual Texture2Ds for the skybox model. The skybox shader path uses a Texture2D per
		// nugget (one nugget per cube face), so we deliberately create separate textures rather than aliasing
		// the cube map. This matches how ModelGenerator::SkyboxSixSidedCube expects to be fed.
		rdr12::Texture2DPtr face_textures[6] = {};
		for (int i = 0; i != 6; ++i)
		{
			auto src = ::pr::compute::Image(face_images[i].m_dim.x, face_images[i].m_dim.y, face_images[i].m_data.vptr, face_images[i].m_format);
			auto tdesc = rdr12::TextureDesc(rdr12::AutoId, ::pr::compute::ResDesc::Tex2D(src, 1)).name(pr::FmtS("env_sky_face%d", i)).srv_format(DXGI_FORMAT_B8G8R8A8_UNORM_SRGB);
			face_textures[i] = factory.CreateTexture2D(tdesc);
		}

		// The cube only needs to remain between the camera's clip planes because it is centred on the camera.
		// It is rendered as a depth-neutral backdrop below, so its radius does not limit visible scene depth.
		auto sky_model = rdr12::ModelGenerator::SkyboxSixSidedCube(factory, face_textures, 100.0f);
		sky_model->m_name = "sky_box";

		// Wrap the model in an LdrObject so AddToScene + p2w plumbing matches the rest of the scene visuals.
		auto sky_obj = rdr12::ldraw::LdrObjectPtr(new rdr12::ldraw::LdrObject(rdr12::ldraw::ELdrObject::Custom, nullptr, pr::GenerateGUID()), true);
		sky_obj->m_model = sky_model;
		sky_obj->m_name = "sky_box";

		// A skybox is a backdrop rather than a scene boundary. Drawing it before opaque geometry without
		// writing depth allows objects beyond the cube radius to remain visible instead of being occluded.
		sky_obj->Flags(rdr12::ldraw::ELdrFlags::NoZWrite, true);
		m_sky_gfx = sky_obj;

	}

	// Return the latest diagnostic buoyancy records for scene-registered hulls.
	std::vector<physics::GpuBuoyancy::Diagnostics> Scene::BuoyancyDiagnostics() const
	{
		auto diagnostics = std::vector<physics::GpuBuoyancy::Diagnostics>{};
		if (m_gpu_buoyancy == nullptr)
			return diagnostics;

		diagnostics.reserve(m_buoyancy_body_indices.size());
		for (auto body_index : m_buoyancy_body_indices)
			diagnostics.push_back(m_gpu_buoyancy->LatestDiagnostics(body_index, m_buoyancy_generation));

		return diagnostics;
	}

	// Rebuild the buoyancy sample-cloud debug overlay (m_buoyancy_debug_gfx) by re-running the
	// deterministic CPU oracle (SampleHull) over every registered hull and emitting an LDraw object:
	// sample points coloured by classification, surface-sample normal whiskers, and per-primitive +
	// total buoyancy force/torque arrows. This is an approximate debugging aid, not a frame-exact
	// mirror of the GPU pass: it samples the post-step body transform with the current clock, so it
	// lags the GPU buoyancy result by roughly one frame. Only valid for the current flat-ocean scene
	// (gravity ~ -Z); a tilted-gravity scene would need a gravity-derived water frame here.
	void Scene::BuildBuoyancyDebugGfx()
	{
		using namespace pr::physics::buoyancy;

		m_buoyancy_debug_gfx = nullptr;
		if (m_rdr == nullptr || m_gpu_buoyancy == nullptr || !m_water.has_value() || m_buoyancy_debug_shapes.empty())
			return;

		// Adapter exposing the scene's WaterSurface through the sampler's water-field concept. Only
		// valid for the default flat WaterFrame{} (up=+Z, t0=+X, t1=+Y, ref=origin): the sampler calls
		// Height/Gradient with planar coords (u,v) = (sample.x, sample.y) and treats the returned value
		// as the signed height along 'up'. EvaluateHeight returns the absolute world-Z surface height,
		// which equals the signed height along +Z from the origin only because ref=origin and up=+Z.
		struct WaterAdapter
		{
			physics::GpuBuoyancy::WaterSurface const* m_surface;
			float m_time;
			float Height(v2 uv) const { return m_surface->EvaluateHeight(uv, m_time); }
			v2 Gradient(v2 uv) const { return m_surface->EvaluateGradient(uv, m_time); }
			v4 Velocity(v4 pos_ws) const { return m_surface->EvaluateVelocity(pos_ws, m_time); }
		};
		auto const water = WaterAdapter{ &m_water->surface, static_cast<float>(m_clock) };

		// Match the buoyancy module's runtime fluid configuration so the visualised forces track the
		// GPU pass as closely as the one-frame lag allows.
		auto const cfg = SamplerConfig{ .m_fluid_density = 1000.0f, .m_drag_time_constant_s = 3.0f, .m_quadratic_drag_coefficient = 1.05f };

		// Map a sample classification to a display colour.
		auto colour_for = [](ESampleKind kind) -> uint32_t
		{
			switch (kind)
			{
				case ESampleKind::VolumeWet:     return 0xFF3060FFU; // blue: contributes buoyancy
				case ESampleKind::VolumeDry:     return 0x40808080U; // faint grey: above water
				case ESampleKind::VolumeCulled:  return 0xFF802020U; // dark red: owned by a lower sibling
				case ESampleKind::SurfaceActive: return 0xFF30FF30U; // green: contributes drag
				case ESampleKind::SurfaceDry:    return 0x40404040U; // faint dark: above water
				case ESampleKind::SurfaceCulled: return 0xFFFF8020U; // orange: interior (non-union) surface
				default:                         return 0xFFFFFFFFU;
			}
		};

		// Sampling + display tuning. The oracle is run at a fixed sample budget; the resulting cloud is
		// decimated at draw time so dense hulls stay renderable. Forces are large (~2e4 N) so arrows are
		// scaled down to world units.
		constexpr int VolumeSamples = 8192;
		constexpr int SurfaceSamples = 8192;
		constexpr int MaxDrawSamples = 16384;
		constexpr float ForceScale = 1.0e-4f;  // N    -> world units
		constexpr float TorqueScale = 1.0e-4f; // N.m  -> world units
		constexpr float NormalLen = 0.1f;      // length of surface-normal whiskers

		// The sample cloud lives inside (volume samples) and on the surface of the opaque body meshes, so
		// without disabling the depth test it would be entirely occluded by the bodies. Render the whole
		// debug group on top (no z-test) so the wet/dry classification cloud and force arrows are visible
		// through the geometry. no_ztest on the group is inherited by the child points/lines.
		ldraw::Builder ldr;
		auto& grp = ldr.Group("buoyancy_debug");
		auto& pts = grp.Point("samples", 0xFFFFFFFFU).size(5.0f).style(ldraw::seri::PointStyle{"Square"});
		auto& normals = grp.Line("normals", 0xFF30FF30U);
		auto& arrows = grp.Line("forces", 0xFFFFFFFFU)/*.arrow("Fwd")*/.per_item_colour();
		//grp.no_ztest();
		//pts.no_ztest();
		//normals.no_ztest();
		//arrows.no_ztest();

		auto any_geometry = false;
		for (size_t h = 0; h != m_buoyancy_debug_shapes.size(); ++h)
		{
			auto const& shape = m_buoyancy_debug_shapes[h].as<collision::Shape>();
			auto const& body = m_body[m_buoyancy_body_indices[h]];

			// The floater bodies have their centre of mass at the model origin, so O2W (model->world)
			// coincides with the sampler's COM-root->world transform and the body's linear velocity is
			// already the velocity at the centre of mass.
			auto const vel = body.VelocityWS();
			auto state = BodyState{};
			state.m_o2w = body.O2W();
			state.m_gravity_ws = body.GravityWS();
			state.m_vel_lin_ws = vel.lin;
			state.m_omega_ws = vel.ang;

			// The flat-water adapter is only valid when gravity is ~ -Z. Assert so a future tilted-gravity
			// scene fails loudly here instead of silently mis-classifying samples.
			assert("BuildBuoyancyDebugGfx assumes a flat -Z gravity water frame" &&
				(Sqr(state.m_gravity_ws.x) + Sqr(state.m_gravity_ws.y)) <= 1e-3f * Max(1e-6f, LengthSq(state.m_gravity_ws.w0())));

			auto debug = SampleDebug{};
			auto const result = SampleHull(shape, static_cast<uint32_t>(h), state, WaterFrame{}, water, cfg, VolumeSamples, SurfaceSamples, &debug);

			// Decimated sample cloud, with green whiskers on active surface samples.
			auto const stride = std::max<size_t>(1, debug.m_samples.size() / MaxDrawSamples);
			for (size_t i = 0; i < debug.m_samples.size(); i += stride)
			{
				auto const& rec = debug.m_samples[i];
				pts.pt(rec.m_pos_ws, colour_for(rec.m_kind));
				if (rec.m_kind == ESampleKind::SurfaceActive)
					normals.line(rec.m_pos_ws, rec.m_pos_ws + NormalLen * rec.m_normal_ws, 0xFF30FF30U);
				any_geometry = true;
			}

			// Per-primitive buoyancy force arrows at each primitive's wet centroid (cyan).
			for (size_t k = 0; k != debug.m_prim_buoy_force_ws.size(); ++k)
			{
				auto const f = debug.m_prim_buoy_force_ws[k];
				if (LengthSq(f.w0()) <= 0.0f)
					continue;

				auto const c = debug.PrimWetCentre(k);
				arrows.line(c, c + ForceScale * f, 0xFF00FFFFU);
				any_geometry = true;
			}

			// Total buoyancy force arrow at the centre of buoyancy (yellow) and torque arrow (magenta).
			if (result.m_valid)
			{
				auto const cob = result.m_centre_buoyancy_ws;
				arrows.line(cob, cob + ForceScale * result.m_buoyancy_force_ws, 0xFFFFFF00U);
				arrows.line(cob, cob + TorqueScale * result.m_buoyancy_torque_ws, 0xFFFF00FFU);
				any_geometry = true;
			}
		}

		if (!any_geometry)
			return;

		auto result = rdr12::ldraw::Parse(*m_rdr, ldr.ToBinary());
		if (!result.m_objects.empty())
			m_buoyancy_debug_gfx = result.m_objects.front();
	}

	// Reset the simulation to the current scenario's initial conditions
	void Scene::Reset()
	{
		if (m_step_pending)
			throw std::runtime_error("Scene::Reset cannot run while a step is pending");

		m_clock = 0;
		m_step_count = 0;
		m_diag.Reset();
		m_gravity = v4::Zero();
		m_kill_zone_height = -100.0f;
		m_physics_substeps = 1;
		m_physics.Config(physics::EngineConfig{
			.sleeping_enabled = m_allow_sleeping,
		});

		ClearBuoyancy();

		// The engine caches caller-owned shapes/bodies by pointer. Drop those references before reusing scene storage.
		m_physics.ResetCaches();

		// Clean up the ground plane visual
		m_ground_gfx = nullptr;
		m_water.reset();
		m_water_gfx = nullptr;
		m_env_map = nullptr;
		m_sky_gfx = nullptr;

		// Release any shapes owned by a previously loaded JSON scene.
		m_body.resize(0);
		m_shape_buffer.resize(0);
		UpdateCollisionReadback();

		// Set up perfectly elastic, frictionless material for clean collision tests
		m_physics.Material(physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = 0.0f,
			.m_elasticity_norm = 1.0f,
			.m_elasticity_tang = 0.0f,
			.m_elasticity_tors = 0.0f,
		});
	}

	// Advance the simulation synchronously by one time step.
	bool Scene::Step(double elapsed_seconds)
	{
		BeginStep(elapsed_seconds);
		return CompleteStep();
	}

	// Submit the first physics substep without waiting for its GPU results.
	void Scene::BeginStep(double elapsed_seconds)
	{
		if (m_step_pending)
			throw std::runtime_error("Scene::BeginStep called while a previous step is pending");

		auto const step_beg = Clock::now();
		m_step_pending = true;
		m_pending_elapsed_seconds = elapsed_seconds;
		m_pending_substeps = std::max(m_physics_substeps, 1);
		m_pending_step_profile = {};
		m_diag.occurred = false;

		try
		{
			auto const dt = static_cast<float>(elapsed_seconds / m_pending_substeps);
			BeginPhysicsSubstep(dt, m_clock, m_pending_step_profile);
			m_pending_step_profile.m_total_ms = ElapsedMs(step_beg, Clock::now());
		}
		catch (...)
		{
			m_step_pending = false;
			throw;
		}
	}

	// Complete a submitted step, including dependent substeps that cannot overlap rendering.
	bool Scene::CompleteStep()
	{
		if (!m_step_pending)
			throw std::runtime_error("Scene::CompleteStep called without a pending step");

		auto const complete_beg = Clock::now();
		try
		{
			CompletePhysicsSubstep(m_pending_step_profile);

			auto const dt = static_cast<float>(m_pending_elapsed_seconds / m_pending_substeps);
			for (int substep = 1; substep != m_pending_substeps; ++substep)
			{
				auto const time_s = m_clock + m_pending_elapsed_seconds * static_cast<double>(substep) / static_cast<double>(m_pending_substeps);
				BeginPhysicsSubstep(dt, time_s, m_pending_step_profile);
				CompletePhysicsSubstep(m_pending_step_profile);
			}
			if (m_diag.occurred)
				++m_diag.count;

			++m_step_count;
			m_clock += m_pending_elapsed_seconds;

			#ifdef PR_PHYSICS_DIAGNOSTICS
			{
				// If a collision occurred this step, capture post-impulse snapshots.
				// Detailed logging is only done for the two-body test scenarios (not file-loaded scenes).
				if (m_diag.occurred && std::ssize(m_body) == 2)
				{
					m_diag.after[0] = BodySnapshot::Capture(m_body[0]);
					m_diag.after[1] = BodySnapshot::Capture(m_body[1]);
					LogCollisionDiagnostics();
				}
			}
			#endif

			// Freeze escaped dynamic bodies before their values grow large enough to lose useful precision.
			auto const kill_beg = Clock::now();
			for (int i = 0; i != std::ssize(m_body); ++i)
			{
				auto mass = m_body[i].Mass();
				if (mass >= physics::InfiniteMass * 0.5f)
					continue;

				auto pos = m_body[i].O2W().pos;
				if (pos.z < m_kill_zone_height)
				{
					m_body[i].ZeroMomentum();
					m_body[i].ZeroForces();
				}
			}
			auto const kill_end = Clock::now();

			m_pending_step_profile.m_total_ms += ElapsedMs(complete_beg, kill_end);
			m_pending_step_profile.m_kill_zone_ms = ElapsedMs(kill_beg, kill_end);
			m_last_step_profile = m_pending_step_profile;
			m_step_pending = false;
			return m_diag.occurred;
		}
		catch (...)
		{
			m_step_pending = false;
			throw;
		}
	}

	// Return true while a scene step is waiting for GPU completion.
	bool Scene::StepPending() const
	{
		return m_step_pending;
	}

	// Prepare fallback colours before collision readback supplies the completed substep's contacts.
	void Scene::PrepareStepVisuals()
	{
		switch (m_visual_mode)
		{
			case EVisualMode::Normal:
			{
				break;
			}
			case EVisualMode::ContactPriority:
			{
				SetContactPriorityFallbackGfx();
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown visual mode");
			}
		}
	}

	// Apply gravity and submit one physics substep without waiting for its result.
	void Scene::BeginPhysicsSubstep(float dt, double time_s, StepProfile& profile)
	{
		// Forces are cleared by integration, so gravity must be restored before every substep.
		auto const gravity_beg = Clock::now();
		if (LengthSq(m_gravity) != 0)
		{
			for (auto& body : m_body)
				body.GravityWS(m_gravity);
		}
		profile.m_gravity_ms += ElapsedMs(gravity_beg, Clock::now());

		auto const physics_beg = Clock::now();
		m_physics.BeginStep(dt, std::span{ m_body }, time_s);
		profile.m_physics_ms += ElapsedMs(physics_beg, Clock::now());
	}

	// Complete one submitted physics substep and accumulate its profiling and collision results.
	void Scene::CompletePhysicsSubstep(StepProfile& profile)
	{
		PrepareStepVisuals();

		auto const physics_beg = Clock::now();
		m_physics.CompleteStep();
		if (m_gpu_buoyancy != nullptr)
			m_gpu_buoyancy->CompleteStep();
		profile.m_physics_ms += ElapsedMs(physics_beg, Clock::now());
		AddProfile(profile.m_engine, m_physics.LastStepProfile());
		if (m_physics.LastCollisionStats().LastContactCount() != 0)
			m_diag.occurred = true;
	}

	// Configure bodies for the current scenario. All test scenarios use no external
	// forces so that collisions can be validated against analytic predictions.
	void Scene::SetupScenario(EScenario scenario)
	{
		ClearBuoyancy();
		m_water.reset();
		m_water_gfx = nullptr;
		m_env_map = nullptr;
		m_sky_gfx = nullptr;

		// The engine caches caller-owned shapes/bodies by pointer. Drop those references before reusing scene storage.
		m_physics.ResetCaches();

		m_body.resize(0);
		m_body.push_back(Body(m_rdr));
		m_body.push_back(Body(m_rdr));
		auto& objA = m_body[0];
		auto& objB = m_body[1];
		objA.m_colour = Colour32(0xFFFFA040U);
		objB.m_colour = Colour32(0xFF40A0FFU);

		// Common setup: zero forces/momentum
		for (int i = 0; i != std::ssize(m_body); ++i)
		{
			m_body[i].ZeroForces();
			m_body[i].ZeroMomentum();
		}

		// Load the scenario
		switch (scenario)
		{
			case EScenario::Sandbox:
			{
				// Default sandbox: two boxes approaching each other gently
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +2.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -2.0f, 0, 0, 0 });
				break;
			}
			case EScenario::HeadOnEqualMass:
			{
				// Two equal-mass boxes approaching each other along X.
				// Elastic collision should swap velocities exactly.
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, 0, 0, 0 });
				break;
			}
			case EScenario::HeadOnDiffMass:
			{
				// Mass 10 hits mass 5 head-on along X.
				// v1' = (m1-m2)/(m1+m2)*v1 + 2*m2/(m1+m2)*v2
				// v2' = 2*m1/(m1+m2)*v1 + (m2-m1)/(m1+m2)*v2
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 5.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, 0, 0, 0 });
				break;
			}
			case EScenario::StationaryTarget:
			{
				// Moving box hits a stationary box (classic billiard scenario)
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, 0, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4::Zero());
				break;
			}
			case EScenario::OffCenter:
			{
				// Off-center hit: boxes offset in Y, collision induces rotation.
				// Body A approaches along X but is offset in Y so the contact
				// point is not aligned with the centres of mass.
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, +0.8f, 0));
				objB.O2W(m4x4::Translation( +5.0f, 0, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, 0, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4::Zero());
				break;
			}
			case EScenario::Oblique:
			{
				// Oblique collision: bodies approaching at an angle
				objA.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objB.Shape(m_box, physics::Inertia::Box(v4{ 1, 1, 1, 0 }, 10.0f));
				objA.O2W(m4x4::Translation( -5.0f, -2.0f, 0));
				objB.O2W(m4x4::Translation( +5.0f, +2.0f, 0));
				objA.VelocityWS(v4::Zero(), v4{ +3.0f, +1.0f, 0, 0 });
				objB.VelocityWS(v4::Zero(), v4{ -3.0f, -1.0f, 0, 0 });
				break;
			}
		}

		auto mat = m_physics.Material(0);

		DbgLog("\n--- Reset: Scenario %d [%s] ---\n", static_cast<int>(scenario), ScenarioName(scenario));
		DbgLog("  Material: elasticity_norm=%.2f friction=%.2f\n", mat.m_elasticity_norm, mat.m_friction_static);
		for (int i = 0; i != std::ssize(m_body); ++i)
		{
			auto snap = BodySnapshot::Capture(m_body[i]);
			snap.Log(FmtS("Body %d (initial)", i));
		}
		auto total_p = m_body[0].MomentumWS().lin + m_body[1].MomentumWS().lin;
		DbgLog("  Total lin momentum: (%.4f, %.4f, %.4f)\n", total_p.x, total_p.y, total_p.z);
		DbgLog("  Total KE: %.6f\n", m_body[0].KineticEnergy() + m_body[1].KineticEnergy());

		m_current_scenario = scenario;
		UpdateCollisionReadback();
	}

	// Load a scene from a JSON file.
	// Replaces the current scenario with bodies defined in the file.
	// Shapes are heap-allocated and owned by m_owned_shapes.
	void Scene::LoadScene(scene_loader::SceneDesc scene_desc)
	{
		auto const load_beg = Clock::now();
		auto mark = load_beg;
		m_last_load_profile = {};
		m_last_load_profile.m_has_renderer = m_rdr != nullptr;

		// Reset simulation state
		m_clock = 0;
		m_step_count = 0;
		m_diag.Reset();

		// The engine caches caller-owned shapes/bodies by pointer. Drop those references before reusing scene storage.
		ClearBuoyancy();
		m_physics.ResetCaches();

		// Clean up ground plane visual from previous scene
		m_ground_gfx = nullptr;
		m_water = scene_desc.water;
		m_water_gfx = nullptr;
		m_env_map = nullptr;
		m_sky_gfx = nullptr;

		// Clear existing bodies and owned shapes
		m_body.resize(0);
		m_shape_buffer.resize(0);

		// Apply gravity from the scene file
		m_gravity = scene_desc.gravity;
		m_physics_substeps = scene_desc.physics_substeps;
		m_physics.Config(physics::EngineConfig{
			.max_collision_pairs = scene_desc.physics_max_collision_pairs,
			.sleeping_enabled = m_allow_sleeping,
			.solver_iterations = scene_desc.physics_solver_iterations,
			.push_out_iterations = scene_desc.physics_position_iterations,
			.broadphase_aabb_margin = scene_desc.physics_broadphase_aabb_margin,
			.contact_sort_propagation_scale = scene_desc.physics_contact_sort_propagation_scale,
			.contact_sort_shock_iterations = scene_desc.physics_contact_sort_shock_iterations,
			.contact_slop_scale = scene_desc.physics_contact_slop_scale,
			.support_contact_slop_scale = scene_desc.physics_support_contact_slop_scale,
			.warm_start_scale = scene_desc.physics_warm_start_scale,
			.selective_refresh_passes = scene_desc.physics_selective_refresh_passes,
			.selective_refresh_max_pairs = scene_desc.physics_selective_refresh_max_pairs,
			.selective_refresh_body_limit = scene_desc.physics_selective_refresh_body_limit,
			.selective_refresh_contact_limit = scene_desc.physics_selective_refresh_contact_limit,
			.selective_refresh_solver_iterations = scene_desc.physics_selective_refresh_solver_iterations,
			.selective_refresh_position_iterations = scene_desc.physics_selective_refresh_position_iterations,
			.selective_refresh_bias_scale = scene_desc.physics_selective_refresh_bias_scale,
			.selective_refresh_restitution_scale = scene_desc.physics_selective_refresh_restitution_scale,
			.selective_refresh_adaptive_body_limit = scene_desc.physics_selective_refresh_adaptive_body_limit,
			.selective_refresh_adaptive_solver_iterations = scene_desc.physics_selective_refresh_adaptive_solver_iterations,
			.selective_refresh_support_only = scene_desc.physics_selective_refresh_support_only,
			.selective_refresh_resolve_support_only = scene_desc.physics_selective_refresh_resolve_support_only,
			.selective_refresh_depth_slop = scene_desc.physics_selective_refresh_depth_slop,
			.selective_refresh_support_depth_slop = scene_desc.physics_selective_refresh_support_depth_slop,
			.selective_refresh_closing_speed_slop = scene_desc.physics_selective_refresh_closing_speed_slop,
			.selective_refresh_support_alignment = scene_desc.physics_selective_refresh_support_alignment,
			.selective_refresh_aabb_margin = scene_desc.physics_selective_refresh_aabb_margin,
		});

		// Set the kill zone well below the ground plane. Bodies that fall below
		// this height are frozen to prevent them from corrupting the simulation.
		m_kill_zone_height = (scene_desc.ground ? scene_desc.ground->height : 0) - 50.0f;

		// Apply material properties from the scene file
		m_physics.Material(physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = scene_desc.friction,
			.m_elasticity_norm = scene_desc.elasticity,
			.m_elasticity_tang = 0.0f,
			.m_elasticity_tors = 0.0f,
		});
		auto const prepare_end = Clock::now();
		m_last_load_profile.m_prepare_ms = ElapsedMs(mark, prepare_end);
		mark = prepare_end;

		// Count total bodies: scene bodies + optional ground plane body
		auto num_scene_bodies = static_cast<int>(scene_desc.bodies.size());
		auto total_bodies = num_scene_bodies + (scene_desc.ground ? 1 : 0);
		m_last_load_profile.m_body_count = total_bodies;
		auto scene_bbox = CalculateSceneBBox(scene_desc);
		const auto ground_thickness = 10.0f;
		auto scene_rng = std::default_random_engine(scene_desc.seed);
		auto const bbox_end = Clock::now();
		m_last_load_profile.m_bbox_ms = ElapsedMs(mark, bbox_end);
		mark = bbox_end;

		// Shapes for the bodies in the scene. Generated bodies deliberately reuse a small shape palette, and this de-duplicates identical
		// descriptors across the whole scene so collision shapes are shared instead of rebuilt per body.
		auto shape_lookup = std::vector<int>(num_scene_bodies, -1);
		auto unique_shape_body_index = std::vector<int>{};
		unique_shape_body_index.reserve(num_scene_bodies);
		for (auto i = 0; i != num_scene_bodies; ++i)
		{
			auto const& bd = scene_desc.bodies[i];
			for (auto j = 0; j != isize(unique_shape_body_index); ++j)
			{
				if (SameShapeDesc(bd, scene_desc.bodies[unique_shape_body_index[j]]))
				{
					shape_lookup[i] = j;
					break;
				}
			}

			if (shape_lookup[i] == -1)
			{
				shape_lookup[i] = isize(unique_shape_body_index);
				unique_shape_body_index.push_back(i);
			}
		}

		m_last_load_profile.m_shape_count = isize(unique_shape_body_index) + (scene_desc.ground ? 1 : 0);
		{
			m_shape_buffer.reserve(m_last_load_profile.m_shape_count * 512);
			for (auto body_index : unique_shape_body_index)
				AppendShape(m_shape_buffer, scene_desc.bodies[body_index]);

			// Create a collision shape for the ground plane
			if (scene_desc.ground)
			{
				// Create the ground plane body as a large thin box with infinite mass.
				v2 extent = scene_desc.ground->size;
				if (LengthSq(extent) == 0) extent = v2(10.0f * Length(scene_bbox.Radius().xy));
				auto bd = scene_loader::BodyDesc{};
				bd.shape_type = scene_loader::BodyDesc::EShape::Box;
				bd.box_dimensions = v4{ extent.x, extent.y, ground_thickness, 0 };
				AppendShape(m_shape_buffer, bd);
			}
		}
		auto const shapes_end = Clock::now();
		m_last_load_profile.m_shapes_ms = ElapsedMs(mark, shapes_end);
		mark = shapes_end;

		// Bodies from the scene description.
		{
			auto shape_ptrs = std::vector<collision::Shape const*>{};
			shape_ptrs.reserve(m_last_load_profile.m_shape_count);
			for (auto shape_ptr = m_shape_buffer.data<collision::Shape>(); shape_ptr != nullptr && isize(shape_ptrs) != m_last_load_profile.m_shape_count; shape_ptr = collision::next(shape_ptr))
				shape_ptrs.push_back(shape_ptr);
			if (isize(shape_ptrs) != m_last_load_profile.m_shape_count)
				throw std::runtime_error("Scene shape buffer ended before all shapes were read");

			// Phase 1: Create bodies WITHOUT the renderer so the ShapeChange handler doesn't
			// try to create graphics yet. This avoids dangling pointer issues during the
			// construction loop (graphics creation calls AddShape which reads the shape data).
			m_body.reserve(total_bodies);
			for (auto body_index = 0; body_index != num_scene_bodies; ++body_index)
			{
				auto const& bd = scene_desc.bodies[body_index];
				Body body(nullptr);
				auto o2w = m4x4::TransformDeg(bd.rotation.x, bd.rotation.y, bd.rotation.z, bd.position);
				body.O2W(o2w);
				body.Shape(shape_ptrs[shape_lookup[body_index]], bd.mass);
				body.VelocityWS(bd.angular_velocity, bd.velocity);
				if (bd.sleeping)
					body.Sleep();
				body.m_colour = bd.colour ? *bd.colour : RandomRGB(scene_rng, 0.0f, 1.0f);
				m_body.push_back(std::move(body));
			}

			// Create the ground plane body as a large thin box with infinite mass.
			// The box is thin in Z (0.5 units) and wide in XY, centred at the ground height.
			if (scene_desc.ground)
			{
				Body ground(nullptr);
				ground.O2W(m4x4::Translation(0, 0, scene_desc.ground->height - 0.5f * ground_thickness));
				ground.Shape(shape_ptrs.back(), -1.0f);
				ground.m_colour = scene_desc.ground->colour ? *scene_desc.ground->colour : RandomRGB(scene_rng, 0.0f, 1.0f);
				m_body.push_back(std::move(ground));
			}

			// Scene files can instantiate objects directly asleep. Build those initial islands explicitly during load so Engine::Step() can
			// assume the sleep/wake state is already coherent and avoid scanning for missing islands every frame.
			m_physics.UpdateSleepIslands(m_body);
		}
		ConfigureBuoyancy(scene_desc);
		UpdateCollisionReadback();
		auto const bodies_end = Clock::now();
		m_last_load_profile.m_bodies_ms = ElapsedMs(mark, bodies_end);
		mark = bodies_end;

		// Create the graphics now that all bodies and shapes are stable in memory.
		// Build a single LDraw script containing all body shapes, then parse it in one
		// call. This is dramatically faster than parsing each body individually because
		// it amortises renderer overhead (shader cache lookups, resource pool, etc.).
		if (m_rdr)
		{
			using namespace pr::ldraw;

			// Share canonical renderer models for primitive shapes and keep exact prototypes only for geometry that cannot be represented by
			// a simple scale transform. Renderer model creation dominates large scene loads, so this avoids reparsing hundreds of boxes and spheres.
			auto use_box_prototype = false;
			auto use_sphere_prototype = false;
			auto use_thick_line_prototype = false;
			auto use_thin_line_prototype = false;
			auto exact_prototype_lookup = std::vector<int>(total_bodies, -1);
			auto exact_prototype_body_index = std::vector<int>{};
			for (int i = 0; i != total_bodies; ++i)
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				switch (body.Shape().m_type)
				{
					case collision::EShape::Box:
					{
						use_box_prototype = true;
						break;
					}
					case collision::EShape::Sphere:
					{
						use_sphere_prototype = true;
						break;
					}
					case collision::EShape::Line:
					{
						auto& line = collision::shape_cast<collision::ShapeLine>(body.Shape());
						if (line.m_radius != 0)
							use_thick_line_prototype = true;
						else
							use_thin_line_prototype = true;
						break;
					}
					default:
					{
						for (int j = 0; j != isize(exact_prototype_body_index); ++j)
						{
							auto const prototype_body = exact_prototype_body_index[j];
							if (i < num_scene_bodies && prototype_body < num_scene_bodies && SameShapeDesc(scene_desc.bodies[i], scene_desc.bodies[prototype_body]))
							{
								exact_prototype_lookup[i] = j;
								break;
							}
						}
						if (exact_prototype_lookup[i] == -1)
						{
							exact_prototype_lookup[i] = isize(exact_prototype_body_index);
							exact_prototype_body_index.push_back(i);
						}
						break;
					}
				}
			}

			Builder builder;
			auto prototype_count = 0;
			auto const box_prototype_name = std::string("ShapeBox");
			auto const sphere_prototype_name = std::string("ShapeSphere");
			auto const thick_line_prototype_name = std::string("ShapeThickLine");
			auto const thin_line_prototype_name = std::string("ShapeThinLine");
			if (use_box_prototype)
			{
				builder.Box(box_prototype_name).box(2, 2, 2).hide();
				++prototype_count;
			}
			if (use_sphere_prototype)
			{
				builder.Sphere(sphere_prototype_name).sphere(1).facets(5).hide();
				++prototype_count;
			}
			if (use_thick_line_prototype)
			{
				builder.Cylinder(thick_line_prototype_name).cylinder(2, 1).facets(1, 50).end_caps().hide();
				++prototype_count;
			}
			if (use_thin_line_prototype)
			{
				builder.Line(thin_line_prototype_name).line(v4(0, 0, -1, 1), v4(0, 0, +1, 1)).hide();
				++prototype_count;
			}

			auto exact_prototype_names = std::vector<std::string>{};
			exact_prototype_names.reserve(exact_prototype_body_index.size());
			for (int i = 0; i != isize(exact_prototype_body_index); ++i)
			{
				auto const body_index = exact_prototype_body_index[i];
				auto const prototype_name = std::format("ShapeExact{}", i);
				exact_prototype_names.push_back(prototype_name);
				builder.Add<LdrCollisionShape>(prototype_name).shape(m_body[body_index].Shape()).hide();
				++prototype_count;
			}

			for (int i = 0; i != total_bodies; ++i)
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				auto prototype_name = std::string_view{};
				body.m_gfx_o2b = m4x4::Identity();
				switch (body.Shape().m_type)
				{
					case collision::EShape::Box:
					{
						prototype_name = box_prototype_name;
						body.m_gfx_o2b = PrimitiveShapeToBody(body);
						break;
					}
					case collision::EShape::Sphere:
					{
						prototype_name = sphere_prototype_name;
						body.m_gfx_o2b = PrimitiveShapeToBody(body);
						break;
					}
					case collision::EShape::Line:
					{
						auto& line = collision::shape_cast<collision::ShapeLine>(body.Shape());
						prototype_name = line.m_radius != 0 ? std::string_view(thick_line_prototype_name) : std::string_view(thin_line_prototype_name);
						body.m_gfx_o2b = PrimitiveShapeToBody(body);
						break;
					}
					default:
					{
						prototype_name = exact_prototype_names[exact_prototype_lookup[i]];
						break;
					}
				}

				builder.Instance(std::format("Body{}", i), body.m_colour.argb)
					.address(prototype_name)
					.group_tint(body.m_colour.argb)
					.o2w(body.O2W() * body.m_gfx_o2b);
			}
			auto const ldraw_build_end = Clock::now();
			m_last_load_profile.m_ldraw_build_ms = ElapsedMs(mark, ldraw_build_end);
			mark = ldraw_build_end;

			// Parse all shapes in one batch
			auto ldr_script = builder.ToBinary();
			m_last_load_profile.m_ldraw_byte_count = ldr_script.size();
			auto const ldraw_serialise_end = Clock::now();
			m_last_load_profile.m_ldraw_serialise_ms = ElapsedMs(mark, ldraw_serialise_end);
			mark = ldraw_serialise_end;

			auto result = rdr12::ldraw::Parse(*m_rdr, ldr_script);
			m_last_load_profile.m_ldraw_object_count = static_cast<int>(result.m_objects.size());
			auto const ldraw_parse_end = Clock::now();
			m_last_load_profile.m_ldraw_parse_ms = ElapsedMs(mark, ldraw_parse_end);
			mark = ldraw_parse_end;

			// Assign each parsed instance object to its corresponding body. The prototype objects are first in the result list.
			for (int i = 0; i != total_bodies; ++i)
			{
				auto& body = m_body[i];
				if (!body.HasShape())
					continue;

				auto const obj_idx = prototype_count + i;
				if (obj_idx < static_cast<int>(result.m_objects.size()))
					body.m_gfx = result.m_objects[obj_idx];

				body.UpdateGfx();
			}
			auto const ldraw_assign_end = Clock::now();
			m_last_load_profile.m_ldraw_assign_ms = ElapsedMs(mark, ldraw_assign_end);
			mark = ldraw_assign_end;
		}
		if (m_water)
			CreateWaterGfx(*m_water, scene_bbox);
		// Logging
		{
			auto mat = m_physics.Material(0);

			DbgLog("\n--- Loaded scene from: %ls ---\n", scene_desc.filepath.c_str());
			if (!scene_desc.description.empty())
				DbgLog("  Description: %s\n", scene_desc.description.c_str());
			DbgLog("  Bodies: %d\n", static_cast<int>(m_body.size()));
			DbgLog("  Gravity: (%.2f, %.2f, %.2f)\n", m_gravity.x, m_gravity.y, m_gravity.z);
			DbgLog("  Ground: %s (height=%.2f)\n", scene_desc.ground ? "yes" : "no", scene_desc.ground ? scene_desc.ground->height : 0.0f);
			DbgLog("  Water: %s (level=%.2f waves=%d)\n", scene_desc.water ? "yes" : "no", scene_desc.water ? scene_desc.water->surface.m_level : 0.0f, scene_desc.water ? isize(scene_desc.water->surface.m_waves) : 0);
			DbgLog("  Material: elasticity=%.2f friction=%.2f\n", mat.m_elasticity_norm, mat.m_friction_static);
			for (int i = 0; i != std::ssize(m_body); ++i)
			{
				auto snap = BodySnapshot::Capture(m_body[i]);
				auto name = (i < static_cast<int>(scene_desc.bodies.size())) ? scene_desc.bodies[i].name.c_str() : "ground";
				snap.Log(FmtS("Body %d '%s'", i, name));
			}
		}
		auto const logging_end = Clock::now();
		m_last_load_profile.m_logging_ms = ElapsedMs(mark, logging_end);
		m_last_load_profile.m_total_ms = ElapsedMs(load_beg, logging_end);
	}

	// Log comprehensive collision diagnostics and analytic comparisons
	void Scene::LogCollisionDiagnostics()
	{
		DbgLog("\n========================================\n");
		DbgLog("=== COLLISION #%d [%s] at t=%.4f ===\n", m_diag.count, ScenarioName(m_current_scenario), m_clock);
		DbgLog("========================================\n");

		// Contact info
		DbgLog("Contact:\n");
		DbgLog("  point_ws = (%.4f, %.4f, %.4f)\n",
			m_diag.contact_point_ws.x, m_diag.contact_point_ws.y, m_diag.contact_point_ws.z);
		DbgLog("  normal_ws = (%.4f, %.4f, %.4f)\n",
			m_diag.contact_normal_ws.x, m_diag.contact_normal_ws.y, m_diag.contact_normal_ws.z);
		DbgLog("  depth = %.6f\n", m_diag.depth);

		// Pre-impulse state
		DbgLog("Pre-impulse:\n");
		m_diag.before[0].Log("Body A");
		m_diag.before[1].Log("Body B");
		auto pre_total_p = m_diag.before[0].momentum.lin + m_diag.before[1].momentum.lin;
		auto pre_total_ke = m_diag.before[0].ke + m_diag.before[1].ke;
		DbgLog("  Total lin momentum: (%.4f, %.4f, %.4f)\n", pre_total_p.x, pre_total_p.y, pre_total_p.z);
		DbgLog("  Total KE: %.6f\n", pre_total_ke);

		// Angular momentum about world origin = spin (at CoM) + orbital (com × p).
		// MomentumWS().ang is the spin angular momentum at each body's centre of mass.
		// We must add the orbital term com × (m*v) for a correct system total.
		auto ang_mom_about_origin = [](BodySnapshot const& s)
		{
			auto orbital = Cross(s.com_pos, s.momentum.lin);
			return s.momentum.ang + orbital;
		};
		auto pre_total_L = ang_mom_about_origin(m_diag.before[0]) + ang_mom_about_origin(m_diag.before[1]);
		DbgLog("  Total ang momentum (about origin): (%.4f, %.4f, %.4f)\n", pre_total_L.x, pre_total_L.y, pre_total_L.z);

		// Post-impulse state
		DbgLog("Post-impulse:\n");
		m_diag.after[0].Log("Body A");
		m_diag.after[1].Log("Body B");
		auto post_total_p = m_diag.after[0].momentum.lin + m_diag.after[1].momentum.lin;
		auto post_total_ke = m_diag.after[0].ke + m_diag.after[1].ke;
		DbgLog("  Total lin momentum: (%.4f, %.4f, %.4f)\n", post_total_p.x, post_total_p.y, post_total_p.z);
		DbgLog("  Total KE: %.6f\n", post_total_ke);

		auto post_total_L = ang_mom_about_origin(m_diag.after[0]) + ang_mom_about_origin(m_diag.after[1]);
		DbgLog("  Total ang momentum (about origin): (%.4f, %.4f, %.4f)\n", post_total_L.x, post_total_L.y, post_total_L.z);

		// Conservation checks
		auto dp = post_total_p - pre_total_p;
		auto dL = post_total_L - pre_total_L;
		auto dke = post_total_ke - pre_total_ke;
		auto dL_pct = Length(pre_total_L) > 0.01f ? 100.0f * Length(dL) / Length(pre_total_L) : 0.0f;
		DbgLog("Conservation:\n");
		DbgLog("  Delta lin momentum: (%.6f, %.6f, %.6f) |dp|=%.6f\n", dp.x, dp.y, dp.z, Length(dp));
		DbgLog("  Delta ang momentum: (%.6f, %.6f, %.6f) |dL|=%.6f (%.2f%%)\n", dL.x, dL.y, dL.z, Length(dL), dL_pct);
		DbgLog("  Delta KE: %.6f (%.2f%%)\n", dke, pre_total_ke > 0 ? 100.0f * dke / pre_total_ke : 0.0f);

		// Pass/fail thresholds.
		// Angular momentum conservation is approximate due to sub-step time correction.
		bool momentum_ok = Length(dp) < 0.01f;
		auto ang_tol = Max(0.01f, Length(pre_total_L) * 0.05f);
		bool ang_momentum_ok = Length(dL) < ang_tol;
		auto ke_tol = Max(0.01f, 0.01f * Abs(pre_total_ke));
		bool ke_ok = Abs(dke) <= ke_tol;
		DbgLog("  Lin Momentum conserved: %s\n", momentum_ok ? "PASS" : "*** FAIL ***");
		DbgLog("  Ang Momentum conserved: %s%s\n", ang_momentum_ok ? "PASS" : "*** FAIL ***", (Length(dL) > 0.01f && ang_momentum_ok) ? " (within sub-step tolerance)" : "");
		DbgLog("  KE conserved (elastic): %s\n", ke_ok ? "PASS" : "*** FAIL ***");

		// Analytic predictions for 1D head-on elastic collisions (scenarios 1-3)
		if (m_current_scenario == EScenario::HeadOnEqualMass ||
			m_current_scenario == EScenario::HeadOnDiffMass ||
			m_current_scenario == EScenario::StationaryTarget)
		{
			LogAnalyticComparison();
		}

		DbgLog("========================================\n\n");
	}

	// Compare post-collision velocities to the analytic solution for 1D elastic collision.
	// For perfectly elastic collision:
	//   v1' = ((m1-m2)*v1 + 2*m2*v2) / (m1+m2)
	//   v2' = ((m2-m1)*v2 + 2*m1*v1) / (m1+m2)
	void Scene::LogAnalyticComparison()
	{
		auto m1 = m_diag.before[0].mass;
		auto m2 = m_diag.before[1].mass;
		auto v1 = m_diag.before[0].lin_vel.x;
		auto v2 = m_diag.before[1].lin_vel.x;

		auto v1_pred = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
		auto v2_pred = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);

		auto v1_actual = m_diag.after[0].lin_vel.x;
		auto v2_actual = m_diag.after[1].lin_vel.x;

		DbgLog("Analytic comparison (1D elastic, X component):\n");
		DbgLog("  v1': predicted=%.4f  actual=%.4f  error=%.6f\n", v1_pred, v1_actual, Abs(v1_pred - v1_actual));
		DbgLog("  v2': predicted=%.4f  actual=%.4f  error=%.6f\n", v2_pred, v2_actual, Abs(v2_pred - v2_actual));

		auto vy1 = m_diag.after[0].lin_vel.y;
		auto vz1 = m_diag.after[0].lin_vel.z;
		auto vy2 = m_diag.after[1].lin_vel.y;
		auto vz2 = m_diag.after[1].lin_vel.z;
		DbgLog("  v1_yz: (%.6f, %.6f)  v2_yz: (%.6f, %.6f)\n", vy1, vz1, vy2, vz2);

		auto w1 = m_diag.after[0].ang_vel;
		auto w2 = m_diag.after[1].ang_vel;
		DbgLog("  ang_vel1: (%.6f, %.6f, %.6f)  ang_vel2: (%.6f, %.6f, %.6f)\n",
			w1.x, w1.y, w1.z, w2.x, w2.y, w2.z);

		bool x_ok = Abs(v1_pred - v1_actual) < 0.05f && Abs(v2_pred - v2_actual) < 0.05f;
		bool yz_ok = Abs(vy1) < 0.05f && Abs(vz1) < 0.05f && Abs(vy2) < 0.05f && Abs(vz2) < 0.05f;
		bool no_spin = Length(w1) < 0.05f && Length(w2) < 0.05f;
		DbgLog("  X velocity match: %s\n", x_ok ? "PASS" : "*** FAIL ***");
		DbgLog("  Y/Z remain zero:  %s\n", yz_ok ? "PASS" : "*** FAIL ***");
		DbgLog("  No angular vel:   %s\n", no_spin ? "PASS" : "*** FAIL ***");
	}

	// Run all scenarios in sequence without rendering, log results for each.
	void Scene::RunAllTests()
	{
		DbgLog("\n################################################################\n");
		DbgLog("### AUTO-TEST: Running all scenarios\n");
		DbgLog("################################################################\n");

		auto const dt = 1.0f / 100.0f;
		auto const max_steps = 5000;

		for (int s = 1; s <= 5; ++s)
		{
			Reset();
			SetupScenario(static_cast<EScenario>(s));

			for (int step = 0; step < max_steps && m_diag.count == 0; ++step)
			{
				m_diag.occurred = false;

				auto bodies = std::span(m_body);
				m_physics.Step(dt, bodies);
				m_clock += dt;

				if (m_diag.occurred)
				{
					m_diag.after[0] = BodySnapshot::Capture(m_body[0]);
					m_diag.after[1] = BodySnapshot::Capture(m_body[1]);
					LogCollisionDiagnostics();
				}

				// NaN guard
				auto pos = m_body[0].O2W().pos;
				if (!std::isfinite(pos.x))
				{
					DbgLog("!!! NaN detected in scenario %d at step %d\n", s, step);
					break;
				}
			}

			if (m_diag.count == 0)
				DbgLog("!!! Scenario %d: No collision after %d steps!\n", s, max_steps);
		}

		DbgLog("\n################################################################\n");
		DbgLog("### AUTO-TEST COMPLETE\n");
		DbgLog("################################################################\n");
	}

	// Dump the current scene to an LDraw file for offline analysis. This is useful for
	void Scene::Dump()
	{
		using namespace pr::ldraw;

		ldraw::Builder builder;
		builder.Add<LdrRigidBody>("body0", 0x8000FF00).rigid_body(m_body[0]);
		builder.Add<LdrRigidBody>("body1", 0x10FF0000).rigid_body(m_body[1]);
		builder.Save(L"dump\\physics_dump.ldr");
	}

	// Create/update the graphics objects for
	void Scene::UpdateCollisionGfx(std::span<physics::RbContact const> contacts)
	{
		// Contact graphics are currently disabled. Keep this path opt-in because reading detailed
		// contacts back from the GPU has a measurable cost in large scenes.
		(void)contacts;
	}

	EVisualMode Scene::VisualMode() const
	{
		return m_visual_mode;
	}

	void Scene::VisualMode(EVisualMode mode)
	{
		auto const changed = m_visual_mode != mode;
		if (!changed)
			return;

		m_visual_mode = mode;
		switch (m_visual_mode)
		{
			case EVisualMode::Normal:
			{
				ClearContactPriorityGfx();
				break;
			}
			case EVisualMode::ContactPriority:
			{
				SetContactPriorityFallbackGfx();
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown visual mode");
			}
		}
		UpdateCollisionReadback();
	}

	bool Scene::AllowSleeping() const
	{
		return m_allow_sleeping;
	}

	void Scene::AllowSleeping(bool allow_sleeping)
	{
		m_allow_sleeping = allow_sleeping;

		auto engine_config = m_physics.Config();
		engine_config.sleeping_enabled = m_allow_sleeping;
		m_physics.Config(engine_config);
	}

	void Scene::UpdateContactPriorityGfx(std::span<physics::RbContact const> contacts)
	{
		if (contacts.empty())
			return;

		auto body_lookup = std::unordered_map<physics::RigidBody const*, int>{};
		body_lookup.reserve(m_body.size());
		for (int body_idx = 0; body_idx != isize(m_body); ++body_idx)
		{
			auto const& body = m_body[body_idx];
			body_lookup.emplace(static_cast<physics::RigidBody const*>(&body), body_idx);
		}

		auto first_contact_order = std::vector<int>(m_body.size(), -1);
		auto non_static_contact_count = 0;
		for (int contact_order = 0; contact_order != isize(contacts); ++contact_order)
		{
			auto const& contact = contacts[contact_order];
			auto const iter_a = body_lookup.find(contact.m_objA);
			auto const iter_b = body_lookup.find(contact.m_objB);
			if (iter_a == body_lookup.end() || iter_b == body_lookup.end())
				throw std::runtime_error("Contact priority visualisation received a contact for an unknown body");

			auto const body_idx_a = iter_a->second;
			auto const body_idx_b = iter_b->second;
			if (AllSet(m_body[body_idx_a].StateFlags(), physics::ERigidBodyStateFlags::Static) ||
				AllSet(m_body[body_idx_b].StateFlags(), physics::ERigidBodyStateFlags::Static))
				continue;

			if (first_contact_order[body_idx_a] == -1)
				first_contact_order[body_idx_a] = non_static_contact_count;
			if (first_contact_order[body_idx_b] == -1)
				first_contact_order[body_idx_b] = non_static_contact_count;

			++non_static_contact_count;
		}

		for (int body_idx = 0; body_idx != isize(m_body); ++body_idx)
		{
			if (first_contact_order[body_idx] == -1)
				continue;

			m_body[body_idx].PriorityColour(ContactPriorityColour(first_contact_order[body_idx], non_static_contact_count), true);
		}
	}

	void Scene::ClearContactPriorityGfx()
	{
		for (auto& body : m_body)
			body.PriorityColour(Colour32White, false);
	}

	void Scene::SetContactPriorityFallbackGfx()
	{
		for (auto& body : m_body)
			body.PriorityColour(ContactPriorityFallbackColour, true);
	}

	bool Scene::NeedsCollisionReadback() const
	{
		switch (m_visual_mode)
		{
			case EVisualMode::Normal:
			{
				#if PR_PHYSICS_DIAGNOSTICS
				// Two-body scenarios use detailed contact callbacks to populate the analytic collision log. Larger file-loaded diagnostics
				// normally only need collision counters, so avoid reading and constructing every contact unless another subscriber asks for it.
				return std::ssize(m_body) == 2;
				#else
				return false;
				#endif
			}
			case EVisualMode::ContactPriority:
			{
				return true;
			}
			default:
			{
				throw std::runtime_error("Unknown visual mode");
			}
		}
	}

	void Scene::UpdateCollisionReadback()
	{
		auto const needs_readback = NeedsCollisionReadback();
		if (needs_readback && !m_collision_sub)
		{
			m_collision_sub = m_physics.Collisions += [&](auto&, std::span<physics::RbContact const> contacts)
			{
				UpdateCollisionGfx(contacts);

				switch (m_visual_mode)
				{
					case EVisualMode::Normal:
					{
						break;
					}
					case EVisualMode::ContactPriority:
					{
						UpdateContactPriorityGfx(contacts);
						break;
					}
					default:
					{
						throw std::runtime_error("Unknown visual mode");
					}
				}

				#if PR_PHYSICS_DIAGNOSTICS
				if (std::ssize(m_body) == 2 && !contacts.empty())
				{
					m_diag.before[0] = BodySnapshot::Capture(m_body[0]);
					m_diag.before[1] = BodySnapshot::Capture(m_body[1]);

					auto const& c = contacts.front();
					m_diag.contact_point_ws = c.m_objA->O2W() * c.m_point_at_t;
					m_diag.contact_normal_ws = (c.m_objA->O2W().rot * c.m_axis).w0();
					m_diag.depth = c.m_depth;
				}
				#endif
			};
		}
		else if (!needs_readback && m_collision_sub)
		{
			m_physics.Collisions -= m_collision_sub;
		}
	}

	// Calculate the bounding box for the scene (excluding terrain)
	BBox Scene::CalculateSceneBBox(scene_loader::SceneDesc const& scene_desc) const
	{
		auto bbox = BBox::Reset();
		for (auto const& bd : scene_desc.bodies)
		{
			auto pos = bd.position;
			auto rad = v4::Zero();
			switch (bd.shape_type)
			{
				case scene_loader::BodyDesc::EShape::Box:      rad = bd.box_dimensions * 0.5f; break;
				case scene_loader::BodyDesc::EShape::Sphere:   rad = v4(bd.sphere_radius, bd.sphere_radius, bd.sphere_radius, 0); break;
				case scene_loader::BodyDesc::EShape::Line:     rad = v4(0, 0, bd.line_length * 0.5f, 0); break;
				case scene_loader::BodyDesc::EShape::Triangle: rad = v4(1, 1, 1, 0); break;
				case scene_loader::BodyDesc::EShape::Polytope:
				{
					for (auto const& v : bd.polytope_verts)
						Grow(bbox, (pos + v.w0()).w1());
					continue;
				}
				default: break;
			}
			Grow(bbox, BBox(pos, rad));
		}
		return bbox;
	}
}

#if PR_UNITTESTS
namespace physics_sandbox::tests
{
	namespace
	{
		struct ReloadBodyState
		{
			v4 m_pos;
			v8motion m_vel;
		};

		scene_loader::SceneDesc BoxScene(v4 const& dimensions, float z)
		{
			auto scene_desc = scene_loader::SceneDesc{};
			scene_desc.description = "Scene reload cache test";
			scene_desc.gravity = v4::Zero();
			scene_desc.ground = scene_loader::GroundPlaneDesc{
				.size = v2{ 10.0f, 10.0f },
				.height = 0.0f,
			};
			scene_desc.bodies.push_back(scene_loader::BodyDesc{
				.name = "box",
				.shape_type = scene_loader::BodyDesc::EShape::Box,
				.box_dimensions = dimensions,
				.mass = 1.0f,
				.position = v4{ 0.0f, 0.0f, z, 1.0f },
			});
			return scene_desc;
		}

		std::vector<ReloadBodyState> RunScene(Scene& scene, scene_loader::SceneDesc scene_desc, int step_count)
		{
			scene.LoadScene(std::move(scene_desc));

			auto const dt = 1.0 / 60.0;
			for (int step = 0; step != step_count; ++step)
				scene.Step(dt);

			auto state = std::vector<ReloadBodyState>();
			state.reserve(s_cast<size_t>(std::ssize(scene.m_body)));
			for (auto const& body : scene.m_body)
			{
				state.push_back(ReloadBodyState{
					.m_pos = body.O2W().pos,
					.m_vel = body.VelocityWS(),
				});
			}

			return state;
		}
		void ExpectSameState(char const* label, std::vector<ReloadBodyState> const& baseline, std::vector<ReloadBodyState> const& actual)
		{
			PR_EXPECT(baseline.size() == actual.size());
			for (int i = 0; i != std::ssize(baseline) && i != std::ssize(actual); ++i)
			{
				auto const pos_delta = Length(baseline[i].m_pos - actual[i].m_pos);
				auto const lin_vel_delta = Length(baseline[i].m_vel.lin - actual[i].m_vel.lin);
				auto const ang_vel_delta = Length(baseline[i].m_vel.ang - actual[i].m_vel.ang);
				if (pos_delta >= 0.001f || lin_vel_delta >= 0.001f || ang_vel_delta >= 0.001f)
				{
					DbgLog("Scene reload mismatch %s body %d: pos_delta=%g lin_vel_delta=%g ang_vel_delta=%g\n", label, i, pos_delta, lin_vel_delta, ang_vel_delta);
					PR_EXPECT(false);
				}
			}
		}
	}

	PRUnitTestClass(SceneReloadTests)
	{
		PRUnitTestMethod(LoadSceneClearsEngineCachedShapeState)
		{
			auto scene = Scene(nullptr);
			auto short_box = BoxScene(v4{ 1.0f, 1.0f, 1.0f, 0.0f }, 0.6f);
			auto tall_box = BoxScene(v4{ 1.0f, 1.0f, 4.0f, 0.0f }, 2.0f);

			auto const baseline = RunScene(scene, short_box, 1);
			scene.m_physics.ResetCaches();
			(void)RunScene(scene, tall_box, 1);
			auto const reloaded = RunScene(scene, short_box, 1);

			ExpectSameState("short_box_after_tall_box", baseline, reloaded);
		}
	};
}
#endif
