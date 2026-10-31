#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Create an unlit white 10x10 quad at the model origin, facing the camera. The object is not added to the window.
	api::Object PopulationQuad(Fixture& fixture)
	{
		// Vertex-only geometry so the instance tint alone decides the pixel colour
		api::Vertex verts[] =
		{
			{{-5,-5,0,1}, {}, {}, 0xFFFFFFFF, 0},
			{{+5,-5,0,1}, {}, {}, 0xFFFFFFFF, 0},
			{{+5,+5,0,1}, {}, {}, 0xFFFFFFFF, 0},
			{{-5,+5,0,1}, {}, {}, 0xFFFFFFFF, 0},
		};
		UINT16 indices[] = {0,1,2,0,2,3};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = api::EGeom::Vert;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_tint = 0xFFFFFFFF;
		auto object = View3D_ObjectCreate("PopulationQuad", 0xFFFFFFFF, 4, 6, 1, verts, indices, &nugget, GUID{});
		Require(object != nullptr, "Population quad creation failed");
		fixture.m_objects.push_back(object);
		return object;
	}

	// Make an enabled instance record translated to (x, y, z) with 'tint'
	api::PopulationInstance PopulationInstanceAt(float x, float y, float z, unsigned tint)
	{
		// Rows of an affine transform with identity rotation
		auto instance = api::PopulationInstance{};
		instance.m_o2w[0] = api::Vec4{1, 0, 0, x};
		instance.m_o2w[1] = api::Vec4{0, 1, 0, y};
		instance.m_o2w[2] = api::Vec4{0, 0, 1, z};
		instance.m_tint = tint;
		instance.m_flags = api::EPopulationInstanceFlags::Enabled;
		return instance;
	}

	// Constants of the 'population_producer' test shader
	struct PopulationProducerConstants
	{
		uint32_t m_count;    // Instances to write
		uint32_t m_live;     // Live count to publish for block 0
		uint32_t m_columns;  // Grid columns
		uint32_t m_tint;     // Instance tint, 0xAARRGGBB
		float m_x, m_y, m_z; // Position of instance 0
		float m_spacing;     // Grid spacing
	};

	// Queue the 'population_producer' test shader on 'population'
	void PopulationProduce(api::Population population, PopulationProducerConstants const& constants)
	{
		// One thread per instance, in groups of 64
		auto groups = static_cast<int>((constants.m_count + 63) / 64);
		View3D_PopulationProduce(population, compiled::population_producer, sizeof(compiled::population_producer), &constants, sizeof(constants), std::max(groups, 1), 1, 1);
	}

	// Replace the default directional light with a single shadow casting point light at 'position'. Returns the light index.
	int PopulationShadowLightAdd(Fixture& fixture, api::Vec4 position)
	{
		// Turn off the default light and ambient so only the point light lights the scene
		auto main_light = View3D_LightGet(fixture.m_window, 0);
		main_light.m_on = FALSE;
		View3D_LightSet(fixture.m_window, 0, main_light);
		View3D_AmbientSet(fixture.m_window, 0xFF000000);

		// A wide spot cone pointing into the scene, with a range that covers all test geometry
		auto light = api::Light{};
		light.m_type = api::ELight::Point;
		light.m_position = position;
		light.m_direction = api::Vec4{0, 0, -1, 0};
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_specular_power = 1;
		light.m_intensity = 1;
		light.m_range = 500;
		light.m_falloff = 0;
		light.m_inner_angle = 0.5f;
		light.m_outer_angle = 1.2f;
		light.m_cast_shadow = 1.0f;
		light.m_cam_relative = FALSE;
		light.m_on = TRUE;
		auto light_index = View3D_LightAdd(fixture.m_window, light);
		fixture.CheckErrors();
		return light_index;
	}

	// Render with cached shadow views, and require the result to match a render without caching, so stale cached views fail with 'message'
	std::vector<unsigned char> CachedShadowImage(Fixture& fixture, char const* message)
	{
		// Two cached renders, so the second reuses the cached views when nothing changed
		auto settings = View3D_ShadowSettingsGet(fixture.m_window);
		settings.m_cache_views = TRUE;
		View3D_ShadowSettingsSet(fixture.m_window, settings);
		fixture.Image();
		auto cached = fixture.Image();

		// The uncached reference
		settings.m_cache_views = FALSE;
		View3D_ShadowSettingsSet(fixture.m_window, settings);
		auto uncached = fixture.Image();
		fixture.CheckErrors();
		Require(ImageDifference(cached, uncached) == 0, message);
		return cached;
	}

	// Create a population LOD that generates a 10x10 quad from SV_VertexID in procedural forward and shadow shaders.
	// The quad is moved by the offset in 'm_positions[0]' of the procedural constants, which can change without re-publishing the population.
	api::Object PopulationProceduralQuad(Fixture& fixture)
	{
		// Forward and shadow shaders share the constants, so one update moves the quad in both passes
		auto constants = ProceduralVertexConstants{
			.m_positions = { api::Vec4{0, 0, 0, 0}, api::Vec4{}, api::Vec4{} },
			.m_colour = api::Vec4{1, 1, 1, 1},
		};
		auto forward = ProceduralVertexShader(api::ERenderStep::ForwardRender, compiled::population_procedural_forward, sizeof(compiled::population_procedural_forward), constants);
		auto shadow = ProceduralVertexShader(api::ERenderStep::ShadowMap, compiled::population_procedural_shadow, sizeof(compiled::population_procedural_shadow), constants);
		Require(forward != nullptr && shadow != nullptr, "Population procedural shader creation failed");
		fixture.m_shaders.push_back(forward);
		fixture.m_shaders.push_back(shadow);

		// Vertex IDs 0..3 select the quad corners. The bounds include every offset the tests use.
		auto placeholder = api::Vertex{};
		uint32_t indices[] = {0, 1, 2, 0, 2, 3};
		auto nugget = api::Nugget{};
		nugget.m_topo = api::ETopo::TriList;
		nugget.m_geom = api::EGeom::Vert;
		nugget.m_v0 = 0;
		nugget.m_v1 = 1;
		nugget.m_cull_mode = api::ECullMode::None;
		nugget.m_tint = 0xFFFFFFFF;
		nugget.m_shaders[0] = api::Nugget::Shader{forward, api::ERenderStep::ForwardRender, 0};
		nugget.m_shaders[1] = api::Nugget::Shader{shadow, api::ERenderStep::ShadowMap, 0};
		auto options = api::ObjectCreateOptions{
			.m_struct_size = sizeof(api::ObjectCreateOptions),
			.m_version = api::ObjectCreateOptions::CurrentVersion,
			.m_vertex_source = api::EVertexSource::ProceduralVertexId,
			.m_vcount_logical = 4,
			.m_bbox = api::BBox{ .centre = {0, 0, 0, 1}, .radius = {30, 5, 1, 0} },
		};
		auto object = View3D_ObjectCreateU32("PopulationProceduralQuad", 0xFFFFFFFF, 1, 6, 1, &placeholder, indices, &nugget, options, GUID{});
		Require(object != nullptr, "Population procedural quad creation failed");
		fixture.m_objects.push_back(object);
		return object;
	}

	// Set the offset of a 'PopulationProceduralQuad'
	void PopulationProceduralOffset(api::Object object, float x, float y, float z)
	{
		// Every procedural shader on the object reads the same constants
		auto constants = ProceduralVertexConstants{
			.m_positions = { api::Vec4{x, y, z, 0}, api::Vec4{}, api::Vec4{} },
			.m_colour = api::Vec4{1, 1, 1, 1},
		};
		View3D_ObjectProceduralConstantsSet(object, &constants, sizeof(constants));
	}

	// Verify GPU-culled population rendering: block live counts, instance flags, draw distance, and scene removal
	void PopulationTests(int samples)
	{
		// The orthographic camera maps world x in [-50,50] to pixels [0,128]. Instances at x = -30 and x = +30 land near pixels 26 and 102.
		Fixture fixture(samples);
		auto quad = PopulationQuad(fixture);
		auto lods = api::PopulationLod{ .m_object = quad, .m_min_size = 0, .m_transition = 0 };
		auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 0));
		Require(prototype != nullptr, "Population prototype creation failed");
		auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 8, 2, 0));
		Require(population != nullptr, "Population creation failed");
		fixture.CheckErrors();

		// Two instances in one block, whose bound contains both instance spheres
		api::PopulationInstance instances[] =
		{
			PopulationInstanceAt(-30, 0, -50, 0xFFFF0000),
			PopulationInstanceAt(+30, 0, -50, 0xFF00FF00),
		};
		auto bbox = api::BBox{ .centre = {0, 0, -50, 1}, .radius = {40, 10, 10, 0} };
		View3D_PopulationInstancesUpdate(population.get(), 0, instances, 2);
		View3D_PopulationBlockSet(population.get(), 0, bbox, 0, 4, 2);
		View3D_WindowAddPopulation(fixture.m_window, population.get());
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 1, 0, 0, 26, 64);
			Expect(image, 0, 1, 0, 102, 64);
			Expect(image, 0, 0, 0, 64, 64);
		}

		// Only the first 'live_count' slots of a block are drawn
		View3D_PopulationBlockSet(population.get(), 0, bbox, 0, 4, 1);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 1, 0, 0, 26, 64);
			Expect(image, 0, 0, 0, 102, 64);
		}

		// Disabled instances are skipped, and instance updates are shown from the next frame
		View3D_PopulationBlockSet(population.get(), 0, bbox, 0, 4, 2);
		instances[0].m_flags = api::EPopulationInstanceFlags::None;
		instances[1].m_tint = 0xFF0000FF;
		View3D_PopulationInstancesUpdate(population.get(), 0, instances, 2);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 0, 0, 0, 26, 64);
			Expect(image, 0, 0, 1, 102, 64);
		}

		// A second block of the same population draws alongside the first
		auto extra = PopulationInstanceAt(0, 30, -50, 0xFFFFFFFF);
		View3D_PopulationInstancesUpdate(population.get(), 4, &extra, 1);
		View3D_PopulationBlockSet(population.get(), 1, api::BBox{ .centre = {0, 30, -50, 1}, .radius = {10, 10, 10, 0} }, 4, 4, 1);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 1, 1, 1, 64, 26);
			Expect(image, 0, 0, 1, 102, 64);
		}

		// Removing a block stops drawing its instances
		View3D_PopulationBlockRemove(population.get(), 1);
		fixture.CheckErrors();
		Expect(fixture.Image(), 0, 0, 0, 64, 26);

		// Removing the population from the window clears it
		View3D_WindowRemovePopulation(fixture.m_window, population.get());
		fixture.CheckErrors();
		Expect(fixture.Image(), 0, 0, 0, 102, 64);

		// Instances beyond the prototype draw distance are not drawn. The instances are 50 units from the camera.
		auto near_prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 20));
		auto near_population = api::PopulationPtr(View3D_PopulationCreate(near_prototype.get(), 8, 2, 0));
		Require(near_population != nullptr, "Population creation failed");
		View3D_PopulationInstancesUpdate(near_population.get(), 0, instances, 2);
		View3D_PopulationBlockSet(near_population.get(), 0, bbox, 0, 4, 2);
		View3D_WindowAddPopulation(fixture.m_window, near_population.get());
		fixture.CheckErrors();
		Expect(fixture.Image(), 0, 0, 0, 102, 64);
		View3D_WindowRemovePopulation(fixture.m_window, near_population.get());

		// Invalid contracts are reported rather than drawn
		View3D_PopulationBlockSet(population.get(), 0, bbox, 6, 4, 2);
		Require(!fixture.m_errors.empty(), "A block beyond the instance capacity was accepted");
		fixture.m_errors.clear();

		fixture.CheckDebugLayer();
		std::cout << "PASS population block live counts, instance flags, draw distance, and removal: MSAA " << samples << '\n';
	}

	// Verify that populations cast shadows, including from outside the camera view, and that cached shadow views follow population changes
	void PopulationShadowTests(int samples)
	{
		// A lit receiver at depth 60 and a population caster: one 10x10 quad instance at depth 40.
		// A point light at (30,0,-20) projects the caster at x = 0 onto the receiver over x in [-40,-20] and y in [-10,10], which is pixels [13,38] x [51,77].
		Fixture fixture(samples);
		auto receiver = fixture.Quad(60, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		(void)receiver;
		auto quad = PopulationQuad(fixture);
		auto lods = api::PopulationLod{ .m_object = quad, .m_min_size = 0, .m_transition = 0 };
		auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 0));
		auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 4, 1, 0));
		Require(population != nullptr, "Population creation failed");
		auto light_index = PopulationShadowLightAdd(fixture, api::Vec4{30, 0, -20, 1});
		auto image = [&](char const* message)
		{
			// Every image in this test checks the cached shadow views against an uncached render
			return CachedShadowImage(fixture, message);
		};
		auto left_shadow = [](std::vector<unsigned char> const& img) { return RegionRed(img, 18, 34, 56, 72); };
		auto right_shadow = [](std::vector<unsigned char> const& img) { return RegionRed(img, 80, 100, 56, 72); };

		// The receiver without any caster
		auto unshadowed = image("Cached shadow views differ without casters");

		// A population instance casts a shadow
		auto instance = PopulationInstanceAt(0, 0, -40, 0xFFFFFFFF);
		View3D_PopulationInstancesUpdate(population.get(), 0, &instance, 1);
		View3D_PopulationBlockSet(population.get(), 0, api::BBox{ .centre = {0, 0, -40, 1}, .radius = {10, 10, 10, 0} }, 0, 4, 1);
		View3D_WindowAddPopulation(fixture.m_window, population.get());
		fixture.CheckErrors();
		auto shadowed = image("Cached shadow views missed an added population");
		Require(left_shadow(shadowed) < 0.5f * left_shadow(unshadowed), "Population shadow missing");
		Require(std::abs(right_shadow(shadowed) - right_shadow(unshadowed)) < 0.02f, "Population shadow darkened a region the caster cannot shade");

		// Removing the block removes the shadow, and restoring it restores the shadow
		View3D_PopulationBlockRemove(population.get(), 0);
		Require(ImageDifference(image("Cached shadow views missed a block removal"), unshadowed) == 0, "Population shadow remained after its block was removed");
		View3D_PopulationBlockSet(population.get(), 0, api::BBox{ .centre = {0, 0, -40, 1}, .radius = {10, 10, 10, 0} }, 0, 4, 1);
		Require(ImageDifference(image("Cached shadow views missed a block change"), shadowed) == 0, "Population shadow differs after its block was restored");

		// A caster outside the camera view still casts a shadow into the view. With the light at (100,0,-20), an instance at x = 60
		// shades the receiver over x in [10,30]. The block moves with the instance, so the shadow follows an instance update.
		auto light = View3D_LightGet(fixture.m_window, light_index);
		light.m_position = api::Vec4{100, 0, -20, 1};
		View3D_LightSet(fixture.m_window, light_index, light);
		View3D_PopulationBlockRemove(population.get(), 0);
		auto side_lit = image("Cached shadow views differ for the side light");
		instance = PopulationInstanceAt(60, 0, -40, 0xFFFFFFFF);
		View3D_PopulationInstancesUpdate(population.get(), 0, &instance, 1);
		View3D_PopulationBlockSet(population.get(), 0, api::BBox{ .centre = {60, 0, -40, 1}, .radius = {10, 10, 10, 0} }, 0, 4, 1);
		auto side_shadowed = image("Cached shadow views missed an instance update");
		Require(right_shadow(side_shadowed) < 0.5f * right_shadow(side_lit), "Shadow missing from a population outside the camera view");

		// A producer that empties the block invalidates cached shadow views, so the shadow disappears
		PopulationProduce(population.get(), PopulationProducerConstants{ .m_count = 0, .m_live = 0, .m_columns = 1, .m_tint = 0xFFFFFFFF, .m_x = 60, .m_y = 0, .m_z = -40, .m_spacing = 0 });
		Require(ImageDifference(image("Cached shadow views missed a producer update"), side_lit) == 0, "Population shadow remained after a producer emptied its block");

		// A producer that restores the instance restores the shadow
		PopulationProduce(population.get(), PopulationProducerConstants{ .m_count = 1, .m_live = 1, .m_columns = 1, .m_tint = 0xFFFFFFFF, .m_x = 60, .m_y = 0, .m_z = -40, .m_spacing = 0 });
		Require(ImageDifference(image("Cached shadow views missed a producer update"), side_shadowed) == 0, "Population shadow differs after a producer restored it");

		// Removing the population from the window removes its shadow
		View3D_WindowRemovePopulation(fixture.m_window, population.get());
		Require(ImageDifference(image("Cached shadow views missed a population removal"), side_lit) == 0, "Population shadow remained after the population was removed");

		fixture.CheckDebugLayer();
		std::cout << "PASS population shadows, off-screen casters, and cached shadow views: MSAA " << samples << '\n';
	}

	// Verify GPU producers: generated instances and live counts, ordering with CPU updates, discarded out-of-range writes, payloads, and argument checks
	void PopulationProducerTests(int samples)
	{
		// A population whose only block starts empty, so everything drawn comes from the producer.
		// The producer places a 2-column grid from (-30,0,-50) with spacing 60, so instances 0 and 1 land near pixels (26,64) and (102,64).
		Fixture fixture(samples);
		auto quad = PopulationQuad(fixture);
		auto lods = api::PopulationLod{ .m_object = quad, .m_min_size = 0, .m_transition = 0 };
		auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 0));
		auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 8, 1, 0));
		Require(population != nullptr, "Population creation failed");
		auto bbox = api::BBox{ .centre = {0, 30, -50, 1}, .radius = {40, 40, 10, 0} };
		View3D_PopulationBlockSet(population.get(), 0, bbox, 0, 4, 0);
		View3D_WindowAddPopulation(fixture.m_window, population.get());
		fixture.CheckErrors();
		Expect(fixture.Image(), 0, 0, 0, 26, 64);

		// The producer writes instances and the block's live count
		auto constants = PopulationProducerConstants{ .m_count = 2, .m_live = 2, .m_columns = 2, .m_tint = 0xFFFF0000, .m_x = -30, .m_y = 0, .m_z = -50, .m_spacing = 60 };
		PopulationProduce(population.get(), constants);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 1, 0, 0, 26, 64);
			Expect(image, 1, 0, 0, 102, 64);
			Expect(image, 0, 0, 0, 64, 64);
		}

		// A later dispatch replaces the generated content
		constants.m_live = 1;
		constants.m_tint = 0xFF00FF00;
		PopulationProduce(population.get(), constants);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 0, 1, 0, 26, 64);
			Expect(image, 0, 0, 0, 102, 64);
		}

		// Dispatches and CPU updates apply in the order they were queued
		constants.m_live = 2;
		constants.m_tint = 0xFFFF0000;
		PopulationProduce(population.get(), constants);
		auto blue = PopulationInstanceAt(30, 0, -50, 0xFF0000FF);
		View3D_PopulationInstancesUpdate(population.get(), 1, &blue, 1);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 1, 0, 0, 26, 64);
			Expect(image, 0, 0, 1, 102, 64);
		}
		View3D_PopulationInstancesUpdate(population.get(), 1, &blue, 1);
		PopulationProduce(population.get(), constants);
		fixture.CheckErrors();
		Expect(fixture.Image(), 1, 0, 0, 102, 64);

		// Writes beyond the instance buffer and the null payload view are discarded, and a live count beyond the block capacity is clamped.
		// The first four grid cells (two rows) are drawn; the second row lies at y = 60, outside the camera view.
		constants.m_count = 20;
		constants.m_live = 10;
		constants.m_tint = 0xFFFFFFFF;
		PopulationProduce(population.get(), constants);
		fixture.CheckErrors();
		{
			auto image = fixture.Image();
			Expect(image, 1, 1, 1, 26, 64);
			Expect(image, 1, 1, 1, 102, 64);
		}

		// A producer can also write a population payload
		auto payload_population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 4, 1, 16));
		Require(payload_population != nullptr, "Population creation failed");
		View3D_PopulationBlockSet(payload_population.get(), 0, api::BBox{ .centre = {0, -30, -50, 1}, .radius = {10, 10, 10, 0} }, 0, 4, 0);
		View3D_WindowAddPopulation(fixture.m_window, payload_population.get());
		PopulationProduce(payload_population.get(), PopulationProducerConstants{ .m_count = 1, .m_live = 1, .m_columns = 1, .m_tint = 0xFF00FF00, .m_x = 0, .m_y = -30, .m_z = -50, .m_spacing = 0 });
		fixture.CheckErrors();
		Expect(fixture.Image(), 0, 1, 0, 64, 102);

		// Invalid producer arguments are reported
		auto bytes = std::span{ compiled::population_producer };
		View3D_PopulationProduce(population.get(), bytes.data(), 0, &constants, sizeof(constants), 1, 1, 1);
		Require(!fixture.m_errors.empty(), "Empty producer bytecode was accepted");
		fixture.m_errors.clear();
		View3D_PopulationProduce(population.get(), bytes.data(), bytes.size(), &constants, 0, 1, 1, 1);
		Require(!fixture.m_errors.empty(), "Empty producer constants were accepted");
		fixture.m_errors.clear();
		View3D_PopulationProduce(population.get(), bytes.data(), bytes.size(), &constants, sizeof(constants), 0, 1, 1);
		Require(!fixture.m_errors.empty(), "An empty producer dispatch was accepted");
		fixture.m_errors.clear();
		Require(View3D_PopulationCreate(prototype.get(), 4, 1, 6) == nullptr, "A payload size that is not a multiple of 4 was accepted");
		fixture.m_errors.clear();

		fixture.CheckDebugLayer();
		std::cout << "PASS population producers, update ordering, discarded writes, and payloads: MSAA " << samples << '\n';
	}

	// Read the diagnostics of 'population', failing the test if the call fails
	api::PopulationStats PopulationStats(Fixture& fixture, api::Population population)
	{
		auto stats = api::PopulationStats{};
		Require(View3D_PopulationStats(population, stats) != FALSE, "Reading population diagnostics failed");
		fixture.CheckErrors();
		return stats;
	}

	// Verify the population diagnostics: CPU state, memory and upload accounting, GPU classification counters, contract errors, and capacity boundaries
	void PopulationStatsTests()
	{
		// The orthographic fixture view covers x/y in [-50,50]. Block 0 is in view, block 1 is outside it, and block 2 is not set.
		Fixture fixture(1);
		auto quad = PopulationQuad(fixture);
		auto lods = api::PopulationLod{ .m_object = quad, .m_min_size = 0, .m_transition = 0 };
		auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 0));
		auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 8, 3, 16));
		Require(population != nullptr, "Population creation failed");

		// Nothing has been classified before the population is drawn
		auto stats = PopulationStats(fixture, population.get());
		Require(stats.m_instance_capacity == 8 && stats.m_block_capacity == 3, "Population capacities are wrong");
		Require(stats.m_resident_bytes == 8 * 64 + 3 * 4 + 16, "Population resident bytes are wrong");
		Require(stats.m_views == 0 && stats.m_active_blocks == 0 && stats.m_upload_bytes == 0, "A new population reported activity");

		// Three of four instances are enabled in the visible block; the hidden block's instance is never tested
		api::PopulationInstance instances[] =
		{
			PopulationInstanceAt(-30, 0, -50, 0xFFFF0000),
			PopulationInstanceAt(+30, 0, -50, 0xFF00FF00),
			PopulationInstanceAt(0, +30, -50, 0xFF0000FF),
			PopulationInstanceAt(0, -30, -50, 0xFFFFFFFF),
			PopulationInstanceAt(200, 0, -50, 0xFFFFFFFF),
		};
		instances[3].m_flags = api::EPopulationInstanceFlags::None;
		View3D_PopulationInstancesUpdate(population.get(), 0, instances, 5);
		View3D_PopulationBlockSet(population.get(), 0, api::BBox{ .centre = {0, 0, -50, 1}, .radius = {40, 40, 10, 0} }, 0, 4, 4);
		View3D_PopulationBlockSet(population.get(), 1, api::BBox{ .centre = {200, 0, -50, 1}, .radius = {10, 10, 10, 0} }, 4, 4, 1);
		View3D_WindowAddPopulation(fixture.m_window, population.get());
		fixture.Image();
		stats = PopulationStats(fixture, population.get());
		Require(stats.m_active_blocks == 2 && stats.m_admitted_instances == 8, "Population block accounting is wrong");
		Require(stats.m_upload_bytes == 3 * 4 + 5 * 64 + 2 * 4, "Population upload bytes are wrong");
		Require(stats.m_scratch_bytes > 0 && stats.m_scratch_high_water >= stats.m_scratch_bytes, "Population scratch accounting is wrong");
		Require(stats.m_views == 1, "Population diagnostics did not complete for the camera view");
		Require(stats.m_blocks_rejected == 1 && stats.m_blocks_overflow == 0, "Population block rejection count is wrong");
		Require(stats.m_instances_tested == 4 && stats.m_entries_emitted == 3 && stats.m_entries_per_lod[0] == 3, "Population classification counters are wrong");
		Require(stats.m_indirect_draws == 1 && stats.m_live_count_violations == 0, "Population draw or contract counters are wrong");

		// A producer that publishes a live count beyond the block capacity is clamped and reported, and producer work is counted
		PopulationProduce(population.get(), PopulationProducerConstants{ .m_count = 4, .m_live = 10, .m_columns = 2, .m_tint = 0xFFFFFFFF, .m_x = -30, .m_y = -30, .m_z = -50, .m_spacing = 60 });
		fixture.Image();
		stats = PopulationStats(fixture, population.get());
		Require(stats.m_producer_dispatches == 1, "Population producer dispatches were not counted");
		Require(stats.m_live_count_violations == 1 && stats.m_instances_tested == 4 && stats.m_entries_emitted == 4, "A live count beyond the block capacity was not clamped and reported");

		// Exact capacity boundaries: a block ending at the instance capacity with every slot live is accepted; one slot further is not
		View3D_PopulationBlockSet(population.get(), 2, api::BBox{ .centre = {0, 0, -50, 1}, .radius = {40, 40, 10, 0} }, 4, 4, 4);
		fixture.CheckErrors();
		View3D_PopulationBlockSet(population.get(), 2, api::BBox{ .centre = {0, 0, -50, 1}, .radius = {40, 40, 10, 0} }, 5, 4, 4);
		Require(!fixture.m_errors.empty(), "A block beyond the instance capacity was accepted");
		fixture.m_errors.clear();
		View3D_PopulationBlockSet(population.get(), 2, api::BBox{ .centre = {0, 0, -50, 1}, .radius = {40, 40, 10, 0} }, 4, 4, 5);
		Require(!fixture.m_errors.empty(), "A live count beyond the block capacity was accepted");
		fixture.m_errors.clear();
		View3D_PopulationBlockSet(population.get(), 3, api::BBox{ .centre = {0, 0, -50, 1}, .radius = {40, 40, 10, 0} }, 0, 1, 1);
		Require(!fixture.m_errors.empty(), "A block index beyond the block capacity was accepted");
		fixture.m_errors.clear();
		View3D_PopulationInstancesUpdate(population.get(), 7, instances, 1);
		fixture.CheckErrors();
		View3D_PopulationInstancesUpdate(population.get(), 7, instances, 2);
		Require(!fixture.m_errors.empty(), "An instance update beyond the instance capacity was accepted");
		fixture.m_errors.clear();
		View3D_PopulationPayloadUpdate(population.get(), 12, instances, 4);
		fixture.CheckErrors();
		View3D_PopulationPayloadUpdate(population.get(), 12, instances, 8);
		Require(!fixture.m_errors.empty(), "A payload update beyond the payload size was accepted");
		fixture.m_errors.clear();

		// Blocks beyond the visible block limit (65535) of one classification are reported rather than drawn
		constexpr int Blocks = 65536;
		auto many = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), Blocks, Blocks, 0));
		Require(many != nullptr, "Population creation failed");
		for (int i = 0; i != Blocks; ++i)
			View3D_PopulationBlockSet(many.get(), i, api::BBox{ .centre = {0, 0, -50, 1}, .radius = {10, 10, 10, 0} }, i, 1, 0);

		View3D_WindowAddPopulation(fixture.m_window, many.get());
		fixture.Image();
		stats = PopulationStats(fixture, many.get());
		Require(stats.m_blocks_overflow == 1 && stats.m_blocks_rejected == 0 && stats.m_instances_tested == 0, "Visible block overflow was not reported");
		View3D_WindowRemovePopulation(fixture.m_window, many.get());

		fixture.CheckDebugLayer();
		std::cout << "PASS population diagnostics, contract errors, and capacity boundaries\n";
	}

	// Verify GPU visibility and LOD selection against an independent CPU reference, under a perspective camera
	void PopulationReferenceTests()
	{
		// A 90 degree perspective camera at the origin looking down -z. The frustum side planes are |x| = -z and |y| = -z, and the projected radius
		// in pixels of a sphere of radius 'r' at distance 'd' is r * (ImageSize / 2) / d, because tan(45) = 1.
		Fixture fixture(1);
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		constexpr float RightAngle = 1.57079633f;
		View3D_CameraFovSet(fixture.m_window, api::Vec2{ RightAngle, RightAngle });
		View3D_CameraClipPlanesSet(fixture.m_window, 1, 1000, api::EClipPlanes::Both);
		fixture.CheckErrors();

		// Three LODs of the 10x10 quad, whose bounding sphere radius is sqrt(50). LOD 1 fades into LOD 2 over projected radii [6, 9).
		auto quad = PopulationQuad(fixture);
		api::PopulationLod lods[] =
		{
			{ .m_object = quad, .m_min_size = 16, .m_transition = 0 },
			{ .m_object = quad, .m_min_size = 6, .m_transition = 0.5f },
			{ .m_object = quad, .m_min_size = 2, .m_transition = 0 },
		};
		auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(lods, 3, 0));
		Require(prototype != nullptr, "Population prototype creation failed");

		// The reference classification of one instance: the LOD lists it appears in. Instances near a decision boundary are not generated, so float
		// rounding cannot change the result. 'margin' is false for such instances.
		auto const radius = std::sqrt(50.0f);
		auto const half_size = 0.5f * ImageSize;
		struct Reference { int lod0; int lod1; bool margin; };
		auto reference = [&](float x, float y, float z) -> Reference
		{
			// Visible spheres are inside all four side planes. Plane distances are scaled by 1/sqrt(2) because the plane normals have length sqrt(2).
			auto margin = true;
			auto inside = true;
			for (auto side : { -z - x, -z + x, -z - y, -z + y })
			{
				auto d = side / std::sqrt(2.0f);
				margin &= std::abs(d + radius) > 1.0f;
				inside &= d >= -radius;
			}
			if (!inside)
				return { -1, -1, margin };

			// The most detailed LOD whose minimum size the projected radius reaches, plus the next LOD inside a transition band.
			auto size = radius * half_size / std::sqrt(x * x + y * y + z * z);
			for (auto edge : { 16.0f, 9.0f, 6.0f, 2.0f })
				margin &= std::abs(size - edge) > 0.03f * edge;

			if (size >= 16) return { 0, -1, margin };
			if (size >= 9) return { 1, -1, margin };
			if (size >= 6) return { 1, 2, margin };
			if (size >= 2) return { 2, -1, margin };
			return { -1, -1, margin };
		};

		// Generate a deterministic scatter in blocks of 64 slots, spanning near and far, inside and outside the view. Every tenth instance is disabled,
		// and the last block is exactly full (live == capacity == the remaining instance capacity).
		constexpr int BlockSlots = 64, BlockCount = 6;
		auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), BlockSlots * BlockCount, BlockCount, 0));
		Require(population != nullptr, "Population creation failed");
		auto rng = uint32_t{ 12345 };
		auto random = [&](float lo, float hi)
		{
			rng = rng * 1664525u + 1013904223u;
			return lo + (hi - lo) * static_cast<float>(rng >> 8) / 16777216.0f;
		};
		int64_t expected_lod[3] = {};
		int64_t expected_tested = 0;
		for (int b = 0; b != BlockCount; ++b)
		{
			// Each block's live instances are packed at its start, and the block bound contains every live sphere
			auto live = b == BlockCount - 1 ? BlockSlots : 40 + 4 * b;
			auto block = std::vector<api::PopulationInstance>{};
			auto lo = api::Vec4{ +1e9f, +1e9f, +1e9f, 1 };
			auto hi = api::Vec4{ -1e9f, -1e9f, -1e9f, 1 };
			for (; static_cast<int>(block.size()) != live;)
			{
				// Reject candidates near a decision boundary
				auto z = random(-300.0f, -5.0f);
				auto x = random(-1.5f, 1.5f) * -z;
				auto y = random(-1.5f, 1.5f) * -z;
				auto ref = reference(x, y, z);
				if (!ref.margin)
					continue;

				// Record the instance and its reference classification
				auto index = b * BlockSlots + static_cast<int>(block.size());
				block.push_back(PopulationInstanceAt(x, y, z, 0xFFFFFFFF));
				if (index % 10 == 9)
				{
					block.back().m_flags = api::EPopulationInstanceFlags::None;
				}
				else
				{
					if (ref.lod0 >= 0) ++expected_lod[ref.lod0];
					if (ref.lod1 >= 0) ++expected_lod[ref.lod1];
				}
				lo = api::Vec4{ std::min(lo.x, x - radius), std::min(lo.y, y - radius), std::min(lo.z, z - radius), 1 };
				hi = api::Vec4{ std::max(hi.x, x + radius), std::max(hi.y, y + radius), std::max(hi.z, z + radius), 1 };
			}

			// The blocks overlap the view, so the GPU tests every live slot
			View3D_PopulationInstancesUpdate(population.get(), b * BlockSlots, block.data(), live);
			auto bbox = api::BBox{ .centre = { 0.5f * (lo.x + hi.x), 0.5f * (lo.y + hi.y), 0.5f * (lo.z + hi.z), 1 }, .radius = { 0.5f * (hi.x - lo.x), 0.5f * (hi.y - lo.y), 0.5f * (hi.z - lo.z), 0 } };
			View3D_PopulationBlockSet(population.get(), b, bbox, b * BlockSlots, BlockSlots, live);
			expected_tested += live;
		}
		View3D_WindowAddPopulation(fixture.m_window, population.get());
		fixture.CheckErrors();

		// The GPU classification matches the reference per LOD
		fixture.Image();
		auto stats = PopulationStats(fixture, population.get());
		Require(stats.m_views == 1 && stats.m_blocks_rejected == 0, "Population reference blocks were rejected");
		Require(stats.m_instances_tested == expected_tested, "Population reference tested count differs from the CPU reference");
		Require(expected_lod[0] > 0 && expected_lod[1] > 0 && expected_lod[2] > 0, "Population reference scatter does not cover every LOD");
		for (int lod = 0; lod != 3; ++lod)
		{
			// Report the first LOD that differs
			if (stats.m_entries_per_lod[lod] != expected_lod[lod])
				std::cout << "LOD " << lod << ": GPU " << stats.m_entries_per_lod[lod] << ", reference " << expected_lod[lod] << '\n';

			Require(stats.m_entries_per_lod[lod] == expected_lod[lod], "Population LOD classification differs from the CPU reference");
		}
		Require(stats.m_entries_emitted == expected_lod[0] + expected_lod[1] + expected_lod[2], "Population emitted count differs from the CPU reference");
		Require(stats.m_indirect_draws == 3, "Population draws one indirect draw per (LOD, nugget)");

		fixture.CheckDebugLayer();
		std::cout << "PASS population visibility and LOD selection match the CPU reference (" << stats.m_entries_emitted << " entries)\n";
	}

	// Verify procedural vertex shaders on population LODs: per-vertex deformation from procedural constants in the forward and shadow passes,
	// and shadow views that follow constant changes without a population update.
	void PopulationProceduralVertexTests(int samples)
	{
		// A red instance at (-30,0,-50). The orthographic camera maps world x in [-50,50] to pixels [0,128].
		{
			Fixture fixture(samples);
			auto quad = PopulationProceduralQuad(fixture);
			auto lods = api::PopulationLod{ .m_object = quad, .m_min_size = 0, .m_transition = 0 };
			auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 0));
			auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 4, 1, 0));
			Require(population != nullptr, "Population with a procedural vertex LOD was rejected");
			auto instance = PopulationInstanceAt(-30, 0, -50, 0xFFFF0000);
			View3D_PopulationInstancesUpdate(population.get(), 0, &instance, 1);
			View3D_PopulationBlockSet(population.get(), 0, api::BBox{ .centre = {-20, 0, -50, 1}, .radius = {20, 10, 10, 0} }, 0, 4, 1);
			View3D_WindowAddPopulation(fixture.m_window, population.get());
			fixture.CheckErrors();
			{
				auto image = fixture.Image();
				Expect(image, 1, 0, 0, 26, 64);
				Expect(image, 0, 0, 0, 51, 64);
			}

			// Moving the generated vertices by 20 units moves the instance to world x = -10, near pixel 51
			PopulationProceduralOffset(quad, 20, 0, 0);
			fixture.CheckErrors();
			{
				auto image = fixture.Image();
				Expect(image, 0, 0, 0, 26, 64);
				Expect(image, 1, 0, 0, 51, 64);
			}
			fixture.CheckDebugLayer();
		}

		// A lit receiver at depth 60 and a caster instance at (60,0,-40), outside the camera view. With the light at (100,0,-20), a caster at x = c
		// shades the receiver over x in [2c-110, 2c-90]: pixels [80,100] for c = 60, and pixels [56,72] for c = 50.
		{
			Fixture fixture(samples);
			auto receiver = fixture.Quad(60, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
			(void)receiver;
			PopulationShadowLightAdd(fixture, api::Vec4{100, 0, -20, 1});
			auto middle_shadow = [](std::vector<unsigned char> const& img) { return RegionRed(img, 56, 72, 56, 72); };
			auto right_shadow = [](std::vector<unsigned char> const& img) { return RegionRed(img, 80, 100, 56, 72); };
			auto lit = CachedShadowImage(fixture, "Cached shadow views differ without casters");

			// The procedural caster casts a shadow
			auto quad = PopulationProceduralQuad(fixture);
			auto lods = api::PopulationLod{ .m_object = quad, .m_min_size = 0, .m_transition = 0 };
			auto prototype = api::PopulationPrototypePtr(View3D_PopulationPrototypeCreate(&lods, 1, 0));
			auto population = api::PopulationPtr(View3D_PopulationCreate(prototype.get(), 4, 1, 0));
			Require(population != nullptr, "Population with a procedural vertex LOD was rejected");
			auto instance = PopulationInstanceAt(60, 0, -40, 0xFFFFFFFF);
			View3D_PopulationInstancesUpdate(population.get(), 0, &instance, 1);
			View3D_PopulationBlockSet(population.get(), 0, api::BBox{ .centre = {55, 0, -40, 1}, .radius = {15, 10, 10, 0} }, 0, 4, 1);
			View3D_WindowAddPopulation(fixture.m_window, population.get());
			auto shadowed = CachedShadowImage(fixture, "Cached shadow views missed a procedural population");
			Require(right_shadow(shadowed) < 0.5f * right_shadow(lit), "Procedural population shadow missing");
			Require(std::abs(middle_shadow(shadowed) - middle_shadow(lit)) < 0.02f, "Procedural population shadow darkened a region the caster cannot shade");

			// Changing only the procedural constants moves the shadow, and cached shadow views follow the change
			PopulationProceduralOffset(quad, -10, 0, 0);
			auto moved = CachedShadowImage(fixture, "Cached shadow views missed a procedural constant change");
			Require(middle_shadow(moved) < 0.5f * middle_shadow(lit), "Procedural population shadow did not follow the generated vertices");
			Require(std::abs(right_shadow(moved) - right_shadow(lit)) < 0.02f, "Procedural population shadow remained at its old position");
			fixture.CheckDebugLayer();
		}
		std::cout << "PASS population procedural vertex shaders in forward and shadow passes: MSAA " << samples << '\n';
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_Population, Extended | Serial)
	{
		render_tests::PopulationTests(1);
		render_tests::PopulationTests(4);
	}
	PRUnitTest(View3d12_PopulationShadows, Extended | Serial)
	{
		render_tests::PopulationShadowTests(1);
		render_tests::PopulationShadowTests(4);
	}
	PRUnitTest(View3d12_PopulationProducer, Extended | Serial)
	{
		render_tests::PopulationProducerTests(1);
		render_tests::PopulationProducerTests(4);
	}
	PRUnitTest(View3d12_PopulationStats, Extended | Serial)
	{
		render_tests::PopulationStatsTests();
	}
	PRUnitTest(View3d12_PopulationReference, Extended | Serial)
	{
		render_tests::PopulationReferenceTests();
	}
	PRUnitTest(View3d12_PopulationProceduralVertex, Extended | Serial)
	{
		render_tests::PopulationProceduralVertexTests(1);
		render_tests::PopulationProceduralVertexTests(4);
	}
}
