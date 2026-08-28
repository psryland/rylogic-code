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

		// Accumulate one engine frame profile into the sandbox's reporting interval.
		void AddProfile(physics::Engine::StepProfile& lhs, physics::Engine::StepProfile const& rhs)
		{
			lhs.m_new_frame_ms += rhs.m_new_frame_ms;
			lhs.m_pack_ms += rhs.m_pack_ms;
			lhs.m_constraint_pack_ms += rhs.m_constraint_pack_ms;
			lhs.m_articulation_pack_ms += rhs.m_articulation_pack_ms;
			lhs.m_upload_ms += rhs.m_upload_ms;
			lhs.m_constraint_upload_ms += rhs.m_constraint_upload_ms;
			lhs.m_articulation_upload_ms += rhs.m_articulation_upload_ms;
			lhs.m_external_forces_ms += rhs.m_external_forces_ms;
			lhs.m_integrate_ms += rhs.m_integrate_ms;
			lhs.m_articulation_integrate_ms += rhs.m_articulation_integrate_ms;
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
			lhs.m_articulation_unpack_ms += rhs.m_articulation_unpack_ms;
			lhs.m_unpack_diagnostics_ms += rhs.m_unpack_diagnostics_ms;
			lhs.m_substep_count += rhs.m_substep_count;
			lhs.m_submission_count += rhs.m_submission_count;
			lhs.m_wait_count += rhs.m_wait_count;
			lhs.m_readback_copy_count += rhs.m_readback_copy_count;
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
				auto const radius_from_origin = Abs(scene_bbox.Centre().xy) + scene_bbox.Radius().xy;
				auto extent = 4.0f * Max(radius_from_origin, v2{ 5.0f, 5.0f });
				if (extent.x > 0.0f && extent.y > 0.0f && IsFinite(extent))
					return extent;
			}

			return v2{ 20.0f, 20.0f };
		}

		// Return the requested ground size, or derive a generous origin-centred extent that contains all constructed geometry.
		v2 GroundExtent(scene_loader::GroundPlaneDesc const& ground, BBox const& scene_bbox)
		{
			if (ground.size.x > 0.0f && ground.size.y > 0.0f)
				return ground.size;

			if (scene_bbox.valid())
			{
				auto const radius_from_origin = Abs(scene_bbox.Centre().xy) + scene_bbox.Radius().xy;
				auto const extent = v2(10.0f * Length(radius_from_origin));
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
		, m_articulation()
		, m_constraints()
		, m_body_ptrs()
		, m_articulation_ptrs()
		, m_articulation_visuals()
		, m_shape_buffer()
		, m_gpu_buoyancy()
		, m_buoyancy_hulls()
		, m_buoyancy_body_indices()
		, m_buoyancy_generation()
		, m_show_buoyancy_debug(false)
		, m_buoyancy_debug_gfx()
		, m_gravity(v4::Zero())
		, m_kill_zone_height(-100.0f)
		, m_physics_substeps(1)
		, m_allow_sleeping(true)
		, m_ground_body_index(-1)
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
		, m_pending_tick_count()
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
		m_buoyancy_debug_gfx = nullptr;
		m_gpu_buoyancy.reset();
		++m_buoyancy_generation;
	}

	// Release all caller-owned simulation objects in dependency order after engine work is complete.
	void Scene::ClearSimulationObjects()
	{
		if (m_step_pending)
			throw std::runtime_error("Scene objects cannot be replaced while a step is pending");

		// Remove external-force registrations and engine-side pointer caches before invalidating their caller-owned targets.
		ClearBuoyancy();
		m_physics.ResetCaches();
		m_constraints = physics::ConstraintSet{};
		m_ground_body_index = -1;

		// Release renderer bindings and non-owning pointer views before their underlying dynamics objects.
		m_articulation_visuals.clear();
		m_body_ptrs.clear();
		m_articulation_ptrs.clear();
		m_body.clear();
		m_articulation.clear();

		// Shapes are released last because both rigid bodies and articulation descriptors refer to them by pointer.
		m_shape_buffer.clear();
	}

	// Rebuild stable pointer spans after all object containers have reached their final addresses.
	void Scene::RebuildStepInputs()
	{
		m_body_ptrs.clear();
		m_body_ptrs.reserve(m_body.size());
		for (auto& body : m_body)
			m_body_ptrs.push_back(&body);

		m_articulation_ptrs.clear();
		m_articulation_ptrs.reserve(m_articulation.size());
		for (auto& articulation : m_articulation)
			m_articulation_ptrs.push_back(&articulation);

		m_physics.UpdateSleepIslands(m_body_ptrs);
	}

	// Register every dynamic scene body for buoyancy when the scene contains water.
	void Scene::ConfigureBuoyancy(scene_loader::SceneDesc const& scene_desc)
	{
		if (!scene_desc.water)
			return;

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
		m_gpu_buoyancy->SetWaterSurface(scene_desc.water->surface);

		// Ground and other infinite-mass bodies cannot respond to buoyancy. All dynamic bodies use
		// their existing collision shapes, so scene descriptions need no parallel hull geometry.
		auto buoyant_link_count = 0;
		for (auto const& articulation : scene_desc.articulations)
		{
			buoyant_link_count += articulation.m_root.m_buoyant ? 1 : 0;
			for (auto const& link : articulation.m_links)
				buoyant_link_count += link.m_buoyant ? 1 : 0;
		}
		m_buoyancy_hulls.reserve(scene_desc.bodies.size() + buoyant_link_count);
		m_buoyancy_body_indices.reserve(scene_desc.bodies.size());
		for (auto body_index = 0; body_index != isize(scene_desc.bodies); ++body_index)
		{
			auto& body = m_body[body_index];
			if (!body.HasShape() || body.Mass() >= physics::InfiniteMass * 0.5f)
				continue;

			m_buoyancy_hulls.push_back(m_gpu_buoyancy->RegisterCompositeHull(body, body_index, m_buoyancy_generation));
			m_buoyancy_body_indices.push_back(body_index);
		}

		// Articulation buoyancy is opt-in per link because each registration participates through the complete tree response.
		for (auto articulation_index = 0; articulation_index != isize(scene_desc.articulations); ++articulation_index)
		{
			auto const& source = scene_desc.articulations[articulation_index];
			auto& articulation = m_articulation[articulation_index];
			auto register_link = [&](scene_loader::ArticulationChildDesc const& link, int link_index)
			{
				if (!link.m_buoyant)
					return;

				auto const stable_index = 1'000'000 + isize(m_buoyancy_hulls);
				m_buoyancy_hulls.push_back(m_gpu_buoyancy->RegisterCompositeHull(
					articulation,
					articulation.LinkAt(link_index),
					stable_index,
					m_buoyancy_generation));
			};

			register_link(source.m_root, 0);
			for (auto link_index = 0; link_index != isize(source.m_links); ++link_index)
				register_link(source.m_links[link_index], link_index + 1);
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
		if (m_rdr == nullptr || m_gpu_buoyancy == nullptr || !m_water.has_value() || m_buoyancy_body_indices.empty())
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
			v2 PressureGradient(v2 uv, float gravity) const { return m_surface->EvaluatePressureGradient(uv, m_time, gravity); }
			v4 Velocity(v4 pos_ws) const { return m_surface->EvaluateVelocity(pos_ws, m_time); }
		};
		auto const water = WaterAdapter{ &m_water->surface, static_cast<float>(m_clock) };

		// Match the buoyancy module's runtime fluid configuration so the visualised forces track the
		// GPU pass as closely as the one-frame lag allows.
		auto const& gpu_cfg = m_gpu_buoyancy->GetConfig();
		auto const cfg = SamplerConfig{
			.m_fluid_density = gpu_cfg.m_fluid_density,
			.m_linear_drag_time_constant_s = gpu_cfg.m_linear_drag_time_constant_s,
			.m_angular_drag_time_constant_s = gpu_cfg.m_angular_drag_time_constant_s,
			.m_quadratic_drag_coefficient = gpu_cfg.m_quadratic_drag_coefficient,
			.m_tangential_drag_coefficient = gpu_cfg.m_tangential_drag_coefficient,
		};

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
		for (size_t h = 0; h != m_buoyancy_body_indices.size(); ++h)
		{
			auto const& body = m_body[m_buoyancy_body_indices[h]];
			auto const* shape = &body.Shape();

			// The collision polytope intentionally omits volume-only tetrahedra. Rebuild them only
			// while the expensive CPU debug overlay is enabled so ordinary scene loading and stepping
			// retain the compact collision representation.
			auto derived_shape = byte_data<16>{};
			if (shape->m_type == collision::EShape::Polytope)
			{
				auto const& poly = collision::shape_cast<collision::ShapePolytope>(*shape);
				if (poly.m_tet_count == 0)
				{
					derived_shape = collision::BuildPolytopeFromPoints(poly.verts(), poly.m_base.m_s2r, poly.m_base.m_material_id, poly.m_base.m_flags, m_gpu_buoyancy->GetConfig().m_polytope_tessellation);
					shape = &derived_shape.as<collision::Shape>();
				}
			}

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
			auto const result = SampleHull(*shape, static_cast<uint32_t>(h), state, WaterFrame{}, water, cfg, VolumeSamples, SurfaceSamples, &debug);

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

		ClearSimulationObjects();

		// Clean up the ground plane visual
		m_ground_gfx = nullptr;
		m_water.reset();
		m_water_gfx = nullptr;
		m_env_map = nullptr;
		m_sky_gfx = nullptr;

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

	// Submit one frame spanning one or more fixed scene ticks without adding another GPU boundary.
	void Scene::BeginStep(double elapsed_seconds, int tick_count)
	{
		if (m_step_pending)
			throw std::runtime_error("Scene::BeginStep called while a previous step is pending");

		// Preserve each scene's configured solver interval when fixed ticks are batched into one submitted frame.
		auto const substeps_per_tick = std::max(m_physics_substeps, 1);
		auto const max_internal_substeps = m_physics.Config().max_internal_substeps;
		if (tick_count < 1 || tick_count > max_internal_substeps / substeps_per_tick)
			throw std::runtime_error("Scene::BeginStep tick count exceeds the engine's internal substep capacity");
		auto const substep_count = tick_count * substeps_per_tick;

		auto const step_beg = Clock::now();
		m_step_pending = true;
		m_pending_elapsed_seconds = elapsed_seconds;
		m_pending_tick_count = tick_count;
		m_pending_step_profile = {};
		m_diag.occurred = false;

		try
		{
			BeginPhysicsFrame(static_cast<float>(elapsed_seconds), substep_count, m_clock, m_pending_step_profile);
			m_pending_step_profile.m_total_ms = ElapsedMs(step_beg, Clock::now());
		}
		catch (...)
		{
			m_step_pending = false;
			throw;
		}
	}

	// Complete a submitted frame after all internal substeps have executed on the GPU.
	bool Scene::CompleteStep()
	{
		if (!m_step_pending)
			throw std::runtime_error("Scene::CompleteStep called without a pending step");

		auto const complete_beg = Clock::now();
		try
		{
			CompletePhysicsFrame(m_pending_step_profile);

			if (m_diag.occurred)
				++m_diag.count;

			m_step_count += m_pending_tick_count;
			m_clock += m_pending_elapsed_seconds;

			#ifdef PR_PHYSICS_DIAGNOSTICS
			{
				// If a collision occurred this step, capture post-impulse snapshots.
				// Detailed logging is only done for the two-body test scenarios (not file-loaded scenes).
				if (m_diag.occurred && std::ssize(m_body) == 2 && m_articulation.empty())
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

			// Floating trees that escape a deliberately bounded demo are frozen as complete dynamical units.
			for (auto& articulation : m_articulation)
			{
				if (articulation.RootType() == physics::EArticulationRootType::Floating && articulation.RootToWorld().pos.z < m_kill_zone_height)
					articulation.Sleep();
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

	// Apply frame-constant gravity and submit all requested internal GPU substeps without waiting.
	void Scene::BeginPhysicsFrame(float dt, int substep_count, double time_s, StepProfile& profile)
	{
		// The engine snapshots this frame force and restores it before every GPU-resident substep.
		auto const gravity_beg = Clock::now();
		if (LengthSq(m_gravity) != 0)
		{
			for (auto& body : m_body)
				body.GravityWS(m_gravity);
			for (auto& articulation : m_articulation)
			{
				for (auto link_index = 0; link_index != articulation.LinkCount(); ++link_index)
					articulation.GravityWS(articulation.LinkAt(link_index), m_gravity);
			}
		}
		profile.m_gravity_ms += ElapsedMs(gravity_beg, Clock::now());

		auto const physics_beg = Clock::now();
		m_physics.BeginStep(physics::Engine::StepInput{
			.m_bodies = m_body_ptrs,
			.m_articulations = m_articulation_ptrs,
			.m_constraints = m_constraints.Count() != 0 ? &m_constraints : nullptr,
			.m_elapsed_seconds = dt,
			.m_substep_count = substep_count,
			.m_time_s = time_s,
		});
		profile.m_physics_ms += ElapsedMs(physics_beg, Clock::now());
	}

	// Complete one submitted frame and accumulate its profiling and collision results.
	void Scene::CompletePhysicsFrame(StepProfile& profile)
	{
		PrepareStepVisuals();

		auto const physics_beg = Clock::now();
		m_physics.CompleteStep();
		if (m_gpu_buoyancy != nullptr)
			m_gpu_buoyancy->CompleteStep();
		profile.m_physics_ms += ElapsedMs(physics_beg, Clock::now());
		AddProfile(profile.m_engine, m_physics.LastStepProfile());
		UpdateArticulationGfx();
		if (m_physics.LastCollisionStats().LastContactCount() != 0)
			m_diag.occurred = true;
	}

	// Configure bodies for the current scenario. All test scenarios use no external
	// forces so that collisions can be validated against analytic predictions.
	void Scene::SetupScenario(EScenario scenario)
	{
		ClearSimulationObjects();
		m_water.reset();
		m_water_gfx = nullptr;
		m_env_map = nullptr;
		m_sky_gfx = nullptr;

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
		RebuildStepInputs();
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
		ClearSimulationObjects();

		// Clean up ground plane visual from previous scene
		m_ground_gfx = nullptr;
		m_water = scene_desc.water;
		m_water_gfx = nullptr;
		m_env_map = nullptr;
		m_sky_gfx = nullptr;

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

		// Count rigid bodies and shaped articulation links before allocating stable collision storage.
		auto num_scene_bodies = static_cast<int>(scene_desc.bodies.size());
		auto total_bodies = num_scene_bodies + (scene_desc.ground ? 1 : 0);
		auto articulation_link_count = 0;
		auto physical_descs = std::vector<scene_loader::BodyDesc const*>{};
		physical_descs.reserve(num_scene_bodies);
		for (auto const& body : scene_desc.bodies)
			physical_descs.push_back(&body);
		for (auto const& articulation : scene_desc.articulations)
		{
			++articulation_link_count;
			if (articulation.m_root.m_has_shape)
				physical_descs.push_back(&articulation.m_root.m_body);
			for (auto const& link : articulation.m_links)
			{
				++articulation_link_count;
				if (link.m_has_shape)
					physical_descs.push_back(&link.m_body);
			}
		}
		auto const shaped_articulation_link_count = isize(physical_descs) - num_scene_bodies;
		m_last_load_profile.m_body_count = total_bodies + articulation_link_count;
		auto scene_bbox = BBox::Reset();
		const auto ground_thickness = 10.0f;
		auto scene_rng = std::default_random_engine(scene_desc.seed);
		auto const bbox_end = Clock::now();
		m_last_load_profile.m_bbox_ms = ElapsedMs(mark, bbox_end);
		mark = bbox_end;

		// Shapes for the bodies in the scene. Generated bodies deliberately reuse a small shape palette, and this de-duplicates identical
		// descriptors across the whole scene so collision shapes are shared instead of rebuilt per body.
		auto shape_lookup = std::vector<int>(physical_descs.size(), -1);
		auto unique_shape_desc_index = std::vector<int>{};
		unique_shape_desc_index.reserve(physical_descs.size());
		for (auto i = 0; i != isize(physical_descs); ++i)
		{
			auto const& physical_desc = *physical_descs[i];
			for (auto j = 0; j != isize(unique_shape_desc_index); ++j)
			{
				if (SameShapeDesc(physical_desc, *physical_descs[unique_shape_desc_index[j]]))
				{
					shape_lookup[i] = j;
					break;
				}
			}

			if (shape_lookup[i] == -1)
			{
				shape_lookup[i] = isize(unique_shape_desc_index);
				unique_shape_desc_index.push_back(i);
			}
		}

		m_last_load_profile.m_shape_count = isize(unique_shape_desc_index) + (scene_desc.ground ? 1 : 0);
		{
			m_shape_buffer.reserve(m_last_load_profile.m_shape_count * 512);
			for (auto physical_desc_index : unique_shape_desc_index)
				AppendShape(m_shape_buffer, *physical_descs[physical_desc_index]);

			// Create a collision shape for the ground plane
			if (scene_desc.ground)
			{
				// Create the ground plane body as a large thin box with infinite mass.
				auto bd = scene_loader::BodyDesc{};
				bd.shape_type = scene_loader::BodyDesc::EShape::Box;
				auto const extent = GroundExtent(*scene_desc.ground, BBox::Reset());
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
				auto o2w = bd.z_axis
					? m4x4::Transform(v4::ZAxis(), *bd.z_axis, bd.position)
					: m4x4::TransformDeg(bd.rotation.x, bd.rotation.y, bd.rotation.z, bd.position);
				body.O2W(o2w);
				auto const* shape = shape_ptrs[shape_lookup[body_index]];

				// Resolve density after the final collision shape exists so generated scale variants
				// retain equal density using the rigid body's existing mass-property path.
				if (bd.density)
					body.Shape(shape, *bd.density, true);
				else
					body.Shape(shape, bd.mass);

				body.VelocityWS(bd.angular_velocity, bd.velocity);
				body.NeverSleep(bd.never_sleep);
				if (bd.sleeping)
					body.Sleep();
				body.m_colour = bd.colour ? *bd.colour : RandomRGB(scene_rng, 0.0f, 1.0f);
				m_body.push_back(std::move(body));
			}

			// Map each shaped articulation link to its de-duplicated immutable collision shape.
			auto articulation_shapes = std::vector<collision::Shape const*>{};
			articulation_shapes.reserve(shaped_articulation_link_count);
			for (auto physical_desc_index = num_scene_bodies; physical_desc_index != isize(physical_descs); ++physical_desc_index)
				articulation_shapes.push_back(shape_ptrs[shape_lookup[physical_desc_index]]);
			BuildMultibodyObjects(scene_desc, articulation_shapes);

			// Size automatic terrain from fully constructed rigid and articulated geometry, whose transforms now include shape and joint frames.
			auto const bbox_beg = Clock::now();
			scene_bbox = CalculateSceneBBox();
			m_last_load_profile.m_bbox_ms = ElapsedMs(bbox_beg, Clock::now());

			// Create the ground body after resizing its placeholder shape without invalidating shape pointers retained by simulation objects.
			if (scene_desc.ground)
			{
				auto& ground_shape = collision::shape_cast<collision::ShapeBox>(const_cast<collision::Shape&>(*shape_ptrs.back()));
				if (LengthSq(scene_desc.ground->size) == 0.0f)
				{
					auto const extent = GroundExtent(*scene_desc.ground, scene_bbox);
					ground_shape.m_radius = v4{0.5f * extent.x, 0.5f * extent.y, 0.5f * ground_thickness, 0.0f};
					ground_shape.m_base.m_bbox = collision::CalcBBox(ground_shape);
				}

				Body ground(nullptr);
				ground.O2W(m4x4::Translation(0, 0, scene_desc.ground->height - 0.5f * ground_thickness));
				ground.Shape(shape_ptrs.back(), -1.0f);
				ground.m_colour = scene_desc.ground->colour ? *scene_desc.ground->colour : RandomRGB(scene_rng, 0.0f, 1.0f);
				m_body.push_back(std::move(ground));
				m_ground_body_index = isize(m_body) - 1;
			}

			// Scene files can instantiate objects directly asleep. Build those initial islands explicitly during load so Engine::Step() can
			// assume the sleep/wake state is already coherent and avoid scanning for missing islands every frame.
			RebuildStepInputs();
		}
		auto const buoyancy_beg = Clock::now();
		ConfigureBuoyancy(scene_desc);
		m_last_load_profile.m_buoyancy_ms = ElapsedMs(buoyancy_beg, Clock::now());
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

			// Articulation links retain their own immutable collision geometry and follow link kinematics after each completed step.
			for (auto visual_index = 0; visual_index != isize(m_articulation_visuals); ++visual_index)
			{
				auto const& visual = m_articulation_visuals[visual_index];
				builder.Add<LdrCollisionShape>(std::format("ArticulationLink{}", visual_index), visual.m_colour.argb)
					.shape(*visual.m_shape);
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

			// Parsed articulation objects follow all shape prototypes and rigid-body instances in the batched result.
			auto const articulation_object_beg = prototype_count + total_bodies;
			for (auto visual_index = 0; visual_index != isize(m_articulation_visuals); ++visual_index)
			{
				auto const object_index = articulation_object_beg + visual_index;
				if (object_index < isize(result.m_objects))
					m_articulation_visuals[visual_index].m_gfx = result.m_objects[object_index];
			}
			UpdateArticulationGfx();
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

	// Synchronise all articulation-link graphics with the latest accepted simulation state.
	void Scene::UpdateArticulationGfx()
	{
		for (auto& visual : m_articulation_visuals)
			visual.UpdateGfx(m_articulation);
	}

	// Add visible articulation-link graphics to the current renderer draw list.
	void Scene::AddArticulationsToScene(rdr12::Scene& scene, m4x4 const& w2c, Frustum const& frustum, v2 const& clip_planes)
	{
		for (auto const& visual : m_articulation_visuals)
			visual.AddToScene(scene, m_articulation, w2c, frustum, clip_planes);
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
			// Articulation contacts use engine-owned proxy bodies; colour whichever ordinary rigid endpoint is present and leave link visuals unchanged.
			if (iter_a == body_lookup.end() && iter_b == body_lookup.end())
				continue;

			if (iter_a != body_lookup.end())
			{
				auto const body_idx = iter_a->second;
				if (!AllSet(m_body[body_idx].StateFlags(), physics::ERigidBodyStateFlags::Static) && first_contact_order[body_idx] == -1)
					first_contact_order[body_idx] = non_static_contact_count;
			}
			if (iter_b != body_lookup.end())
			{
				auto const body_idx = iter_b->second;
				if (!AllSet(m_body[body_idx].StateFlags(), physics::ERigidBodyStateFlags::Static) && first_contact_order[body_idx] == -1)
					first_contact_order[body_idx] = non_static_contact_count;
			}

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
				return std::ssize(m_body) == 2 && m_articulation.empty();
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
				if (std::ssize(m_body) == 2 && m_articulation.empty() && !contacts.empty())
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

	// Calculate world-space bounds for the constructed rigid bodies and shaped articulation links, excluding terrain.
	BBox Scene::CalculateSceneBBox() const
	{
		auto bbox = BBox::Reset();
		for (int body_index = 0; body_index != isize(m_body); ++body_index)
		{
			if (body_index == m_ground_body_index || !m_body[body_index].HasShape())
				continue;

			Grow(bbox, m_body[body_index].BBoxWS());
		}

		for (auto const& articulation : m_articulation)
		{
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				auto const& link_desc = articulation.LinkDescription(link);
				if (link_desc.m_shape == nullptr)
					continue;

				auto const shape_to_world = articulation.LinkToWorld(link) * link_desc.m_shape_to_link;
				Grow(bbox, shape_to_world * (link_desc.m_shape->m_s2r * link_desc.m_shape->m_bbox));
			}
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
		PRUnitTestMethod(LoadSceneClearsEngineCachedShapeState, Quick)
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

	PRUnitTestClass(SceneBoundsTests)
	{
		PRUnitTestMethod(IncludesArticulationShapeTransformsInAutomaticGroundExtent, Quick)
		{
			// An articulation-only scene exercises the bounds path that previously saw only top-level rigid-body descriptors.
			auto scene_desc = scene_loader::SceneDesc{};
			scene_desc.ground = scene_loader::GroundPlaneDesc{};
			auto articulation = scene_loader::ArticulationDesc{};
			articulation.m_name = "offset_root";
			articulation.m_root_to_world = m4x4::Translation(10.0f, 0.0f, 2.0f);
			articulation.m_root.m_body.name = "root";
			articulation.m_root.m_body.shape_type = scene_loader::BodyDesc::EShape::Box;
			articulation.m_root.m_body.box_dimensions = v4{2.0f, 4.0f, 6.0f, 0.0f};
			articulation.m_root.m_body.mass = 1.0f;
			articulation.m_root.m_shape_to_link = m4x4::Translation(5.0f, 0.0f, 0.0f);
			scene_desc.articulations.push_back(std::move(articulation));

			// Runtime bounds and the generated origin-centred ground must both contain the transformed link geometry.
			auto scene = Scene(nullptr);
			scene.LoadScene(std::move(scene_desc));
			auto const bbox = scene.CalculateSceneBBox();
			auto const& ground_shape = collision::shape_cast<collision::ShapeBox>(scene.m_body[scene.m_ground_body_index].Shape());
			PR_EXPECT(FEql(bbox.Centre(), v4{15.0f, 0.0f, 2.0f, 1.0f}));
			PR_EXPECT(FEql(bbox.Radius(), v4{1.0f, 2.0f, 3.0f, 0.0f}));
			PR_EXPECT(ground_shape.m_radius.x >= Abs(bbox.Centre().x) + bbox.Radius().x);
			PR_EXPECT(ground_shape.m_radius.y >= Abs(bbox.Centre().y) + bbox.Radius().y);
		}
	};
}
#endif
