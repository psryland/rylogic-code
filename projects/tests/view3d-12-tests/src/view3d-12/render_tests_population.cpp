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

		// Replace the default directional light with a single shadow casting point light
		auto main_light = View3D_LightGet(fixture.m_window, 0);
		main_light.m_on = FALSE;
		View3D_LightSet(fixture.m_window, 0, main_light);
		View3D_AmbientSet(fixture.m_window, 0xFF000000);
		auto light = api::Light{};
		light.m_type = api::ELight::Point;
		light.m_position = api::Vec4{30, 0, -20, 1};
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

		// Render with cached shadow views, then check that the result matches a render without caching, so stale views are detected
		auto settings = View3D_ShadowSettingsGet(fixture.m_window);
		auto image = [&](char const* message)
		{
			// Two cached renders, so the second reuses the cached views when nothing changed
			settings.m_cache_views = TRUE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			fixture.Image();
			auto cached = fixture.Image();
			settings.m_cache_views = FALSE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			auto uncached = fixture.Image();
			fixture.CheckErrors();
			Require(ImageDifference(cached, uncached) == 0, message);
			return cached;
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
}
