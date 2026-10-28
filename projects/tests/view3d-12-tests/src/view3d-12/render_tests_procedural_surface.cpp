#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Prove a caller-supplied forward pixel family replaces every rasterised forward pixel shader while keeping the stock outputs.
	void ProceduralPixelFamilyTests()
	{
		// The fixture family rotates the stock colour channels (rgb -> gbr), so stock green renders red.
		Fixture fixture(1);
		std::cout << "Procedural pixel family\n";
		std::array<api::ShaderByteCode, api::ProceduralBinding::ForwardPixelCount> family = {{
			{compiled::procedural_pixel_opaque, sizeof(compiled::procedural_pixel_opaque)},
			{compiled::procedural_pixel_reflection_attrs, sizeof(compiled::procedural_pixel_reflection_attrs)},
			{compiled::procedural_pixel_alpha_collect, sizeof(compiled::procedural_pixel_alpha_collect)},
		}};
		auto options_at = [&family](float depth, ProceduralVertexConstants& constants)
		{
			// Place a large generated triangle at the requested forward depth, covering the sampled centre pixel.
			constants = ProceduralVertexConstants{
				.m_positions = {{-40, -40, -depth, 1}, {40, -40, -depth, 1}, {0, 40, -depth, 1}},
				.m_colour = {0, 1, 0, 1},
			};
			auto options = api::ShaderOptions{
				.m_struct_size = sizeof(api::ShaderOptions),
				.m_version = api::ShaderOptions::CurrentVersion,
				.m_stage = api::EShaderStage::Vertex,
				.m_bytecode = compiled::procedural_vertex_forward,
				.m_bytecode_size = sizeof(compiled::procedural_vertex_forward),
				.m_dbg_name = "ProceduralPixelFamilyTest",
				.m_procedural = {
					.m_rdr_step = api::ERenderStep::ForwardRender,
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
				},
			};
			std::ranges::copy(family, std::begin(options.m_procedural.m_forward_pixel));
			return options;
		};

		// Malformed families fail through the public error callback without publishing a shader.
		auto expect_shader_error = [&fixture](api::ShaderOptions const& options, std::string_view expected)
		{
			// Require both a null handle and the diagnostic for the violated family contract.
			fixture.m_errors.clear();
			Require(View3D_ShaderCreate(options) == nullptr, "Malformed procedural pixel family was accepted");
			Require(!fixture.m_errors.empty() && fixture.m_errors.back().find(expected) != std::string::npos, "Malformed pixel family reported the wrong error");
		};
		auto constants = ProceduralVertexConstants{};
		auto bad = options_at(10, constants);
		bad.m_procedural.m_forward_pixel[1] = {};
		expect_shader_error(bad, "all supplied or all omitted");
		bad = options_at(10, constants);
		bad.m_procedural.m_rdr_step = api::ERenderStep::ShadowMap;
		expect_shader_error(bad, "only supported for the ForwardRender");
		bad = options_at(10, constants);
		bad.m_procedural.m_forward_pixel[2].m_size = 3;
		expect_shader_error(bad, "pixel shader bytecode");

		// A pixel-stage shader has no vertex bytecode and requires a forward family.
		auto pixel_only = options_at(10, constants);
		pixel_only.m_stage = api::EShaderStage::Pixel;
		pixel_only.m_bytecode = nullptr;
		pixel_only.m_bytecode_size = 0;
		bad = pixel_only;
		bad.m_bytecode = compiled::procedural_vertex_forward;
		bad.m_bytecode_size = sizeof(compiled::procedural_vertex_forward);
		expect_shader_error(bad, "not as vertex bytecode");
		bad = pixel_only;
		std::ranges::fill(bad.m_procedural.m_forward_pixel, api::ShaderByteCode{});
		expect_shader_error(bad, "require a forward pixel family");
		bad = pixel_only;
		bad.m_procedural.m_rdr_step = api::ERenderStep::RayCast;
		expect_shader_error(bad, "only supported for the ForwardRender");
		fixture.m_errors.clear();

		// Render one generated triangle per case, each with its own copied constants.
		auto draw = [&](float depth, unsigned tint)
		{
			// The geometry contract is the same as the vertex-only fixtures; only the pixel stage differs.
			fixture.Clear();
			auto shader_constants = ProceduralVertexConstants{};
			auto shader = View3D_ShaderCreate(options_at(depth, shader_constants));
			Require(shader != nullptr, "Procedural pixel family shader creation failed");
			fixture.m_shaders.push_back(shader);
			auto placeholder = api::Vertex{};
			uint32_t indices[] = {70000, 70001, 70002};
			auto nugget = api::Nugget{};
			nugget.m_topo = api::ETopo::TriList;
			nugget.m_geom = api::EGeom::Vert;
			nugget.m_v1 = 1;
			nugget.m_cull_mode = api::ECullMode::None;
			nugget.m_tint = tint;
			nugget.m_shaders[0] = api::Nugget::Shader{shader, api::ERenderStep::ForwardRender, 0};
			auto object_options = api::ObjectCreateOptions{
				.m_struct_size = sizeof(api::ObjectCreateOptions),
				.m_version = api::ObjectCreateOptions::CurrentVersion,
				.m_vertex_source = api::EVertexSource::ProceduralVertexId,
				.m_vcount_logical = 70003,
				.m_bbox = {{0, 0, -depth, 1}, {40, 40, 0, 0}},
			};
			auto object = View3D_ObjectCreateU32("ProceduralPixelFamily", 0xFFFFFFFF, 1, 3, 1, &placeholder, indices, &nugget, object_options, GUID{});
			Require(object != nullptr, "Procedural pixel family object creation failed");
			fixture.m_objects.push_back(object);
			View3D_WindowAddObject(fixture.m_window, object);
			fixture.CheckErrors();
			return fixture.Image();
		};

		// Opaque and transparent materials use the family's opaque and alpha-collect entry points.
		Expect(draw(10, 0xFFFFFFFF), 1, 0, 0);
		Expect(draw(10, 0x80FFFFFF), 0.5f, 0, 0);
		fixture.CheckErrors();

		// A pixel-only family keeps the stock vertex shader, so it shades an ordinary buffered mesh.
		fixture.Clear();
		auto pixel_shader = View3D_ShaderCreate(pixel_only);
		Require(pixel_shader != nullptr, "Pixel-only procedural shader creation failed");
		fixture.m_shaders.push_back(pixel_shader);
		fixture.Quad(10, 0xFFFFFFFF, 45, pixel_shader, 0, 0xFF00FF00);
		Expect(fixture.Image(), 1, 0, 0);
		fixture.CheckErrors();

		// The forward pixel model must be one of the published material models.
		bad = pixel_only;
		bad.m_procedural.m_forward_pixel_model = static_cast<api::EForwardPixelModel>(7);
		expect_shader_error(bad, "forward pixel model");
		fixture.m_errors.clear();

		// A PBR family replaces the stock PBR entry points of a promoted procedural-surface material.
		std::array<api::ShaderByteCode, api::ProceduralBinding::ForwardPixelCount> pbr_family = {{
			{compiled::procedural_pixel_pbr_opaque, sizeof(compiled::procedural_pixel_pbr_opaque)},
			{compiled::procedural_pixel_pbr_reflection_attrs, sizeof(compiled::procedural_pixel_pbr_reflection_attrs)},
			{compiled::procedural_pixel_pbr_alpha_collect, sizeof(compiled::procedural_pixel_pbr_alpha_collect)},
		}};
		auto pbr_options = pixel_only;
		pbr_options.m_procedural.m_forward_pixel_model = api::EForwardPixelModel::Pbr;
		std::ranges::copy(pbr_family, std::begin(pbr_options.m_procedural.m_forward_pixel));
		auto pbr_shader = View3D_ShaderCreate(pbr_options);
		Require(pbr_shader != nullptr, "PBR procedural pixel family shader creation failed");
		fixture.m_shaders.push_back(pbr_shader);

		// Render the same flat-coloured procedural surface with and without the PBR family; the family output is the stock output with rotated channels.
		auto surface = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Soil);
		surface.m_colour0 = surface.m_colour1 = surface.m_colour2 = surface.m_colour3 = 0xFF20C040;
		surface.m_normal_strength = 0.0f;
		auto draw_surface = [&](api::Shader shader, float depth)
		{
			// Promote an ordinary normal-carrying quad to the stock PBR procedural-surface material.
			fixture.Clear();
			auto object = fixture.Quad(depth, 0xFFFFFFFF, 45, shader, 0, 0xFFFFFFFF, false, true);
			View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
			fixture.CheckErrors();
			return fixture.Image();
		};
		auto expect_rotated = [](std::vector<unsigned char> const& image, std::vector<unsigned char> const& stock)
		{
			// Compare the centre pixel in linear colour.
			auto pixel = (ImageSize / 2 * ImageSize + ImageSize / 2) * 4;
			Require(Linear(stock[pixel + 1]) > 0.1f, "Stock PBR procedural surface did not render its colour");
			Expect(image, Linear(stock[pixel + 1]), Linear(stock[pixel + 2]), Linear(stock[pixel + 0]));
		};
		auto stock_image = draw_surface(nullptr, 10);
		expect_rotated(draw_surface(pbr_shader, 10), stock_image);
		fixture.CheckErrors();

		// Promotion to PBR rejects a family written for simple materials.
		fixture.Clear();
		auto simple_promoted = fixture.Quad(10, 0xFFFFFFFF, 45, pixel_shader, 0, 0xFFFFFFFF, false, true);
		View3D_ObjectNuggetProceduralSurfaceSet(simple_promoted, surface, nullptr, 0);
		Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("simple-material procedural pixel shaders") != std::string::npos, "PBR promotion accepted a simple-material pixel family");
		fixture.m_errors.clear();

		// A PBR family drawn with a simple material fails at render time instead of shading with the wrong material constants.
		fixture.Clear();
		fixture.Quad(10, 0xFFFFFFFF, 45, pbr_shader);
		View3D_WindowRender(fixture.m_window);
		Require(!fixture.m_errors.empty() && fixture.m_errors.back().find("PBR forward pixel families require") != std::string::npos, "A simple material accepted a PBR pixel family");
		fixture.m_errors.clear();
		fixture.Clear();
		fixture.CheckDebugLayer();
		std::cout << "PASS procedural pixel family\n";
	}

	// Exercise the public procedural surface API through real forward rendering and framebuffer readback.
	void ProceduralSurfaceTests()
	{
		// Use one bounded fixture and replace its scene between independent material contracts.
		Fixture fixture(1);
		auto object = fixture.Quad(10, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto presets = std::array{
			api::EProceduralSurfacePreset::Soil,
			api::EProceduralSurfacePreset::Grass,
			api::EProceduralSurfacePreset::Sand,
			api::EProceduralSurfacePreset::Rock,
			api::EProceduralSurfacePreset::Snow,
		};
		auto images = std::vector<std::vector<unsigned char>>{};
		for (auto preset : presets)
		{
			auto surface = View3D_ProceduralSurfacePreset(preset);
			surface.m_feature_scale = 7.0f;
			surface.m_seed = 0x12345678u;
			View3D_ObjectNuggetProceduralSurfaceSet(object, surface, nullptr, 0);
			fixture.CheckErrors();
			images.push_back(fixture.Image());
		}
		for (auto i = size_t{1}; i != images.size(); ++i)
			Require(ImageDifference(images[0], images[i]) > 1000, "Procedural presets did not produce distinct rendered surfaces");

		// Repeated frames and camera translation along the view axis must not move a world-coordinate field.
		auto default_light = SceneLightingGet(fixture.m_window);
		auto ambient_light = default_light;
		ambient_light.m_ambient = 0xFFFFFFFF;
		ambient_light.m_diffuse = 0xFF000000;
		ambient_light.m_specular = 0xFF000000;
		ambient_light.m_on = TRUE;
		ambient_light.m_intensity = 1.0f;
		SceneLightingSet(fixture.m_window, ambient_light);
		auto soil = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Soil);
		soil.m_feature_scale = 5.0f;
		soil.m_seed = 42;
		View3D_ObjectNuggetProceduralSurfaceSet(object, soil, nullptr, 0);
		auto deterministic = fixture.Image();
		Require(deterministic == fixture.Image(), "Procedural output changed between identical GPU frames");
		api::Mat4x4 camera{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,2,1}};
		View3D_CameraToWorldSet(fixture.m_window, camera);
		Require(deterministic == fixture.Image(), "Camera movement changed stable world-coordinate procedural output");
		camera.w.z = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);

		// Seed and palette changes must affect the GPU-evaluated albedo rather than a generated texture resource.
		auto changed_seed = soil;
		changed_seed.m_seed = 43;
		View3D_ObjectNuggetProceduralSurfaceSet(object, changed_seed, nullptr, 0);
		Require(ImageDifference(deterministic, fixture.Image()) > 1000, "Procedural seed did not affect rendered output");
		auto red = soil;
		red.m_colour0 = red.m_colour1 = red.m_colour2 = red.m_colour3 = 0xFFFF2020;
		View3D_ObjectNuggetProceduralSurfaceSet(object, red, nullptr, 0);
		auto red_image = fixture.Image();
		auto green = red;
		green.m_colour0 = green.m_colour1 = green.m_colour2 = green.m_colour3 = 0xFF20FF20;
		View3D_ObjectNuggetProceduralSurfaceSet(object, green, nullptr, 0);
		Require(ImageDifference(red_image, fixture.Image()) > 1000, "Procedural albedo palette did not affect rendered output");

		// Normal strength and roughness ranges independently affect lit PBR output without UVs or tangent streams.
		SceneLightingSet(fixture.m_window, default_light);
		auto channels = View3D_ProceduralSurfacePreset(api::EProceduralSurfacePreset::Rock);
		channels.m_feature_scale = 8.0f;
		channels.m_colour0 = channels.m_colour1 = channels.m_colour2 = channels.m_colour3 = 0xFF808080;
		channels.m_normal_strength = 0.0f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		auto flat_normal = fixture.Image();
		channels.m_normal_strength = 1.2f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		Require(ImageDifference(flat_normal, fixture.Image()) > 100, "Procedural normal channel did not affect rendered output");
		channels.m_normal_strength = 0.0f;
		channels.m_roughness_min = channels.m_roughness_max = 0.04f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		auto smooth = fixture.Image();
		channels.m_roughness_min = channels.m_roughness_max = 1.0f;
		View3D_ObjectNuggetProceduralSurfaceSet(object, channels, nullptr, 0);
		Require(ImageDifference(smooth, fixture.Image()) > 50, "Procedural roughness channel did not affect rendered output");

		// Object-space coordinates move with the object rather than being anchored to the world field.
		SceneLightingSet(fixture.m_window, ambient_light);
		auto object_space = soil;
		object_space.m_coordinate_space = api::EProceduralCoordinateSpace::Object;
		View3D_ObjectNuggetProceduralSurfaceSet(object, object_space, nullptr, 0);
		auto object_reference = fixture.Image();
		api::Mat4x4 translated{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {1000,0,0,1}};
		View3D_ObjectO2WSet(object, translated, nullptr);
		camera.w.x = 1000;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		auto moved_object = fixture.Image();
		auto object_difference = size_t{};
		for (auto y = 24; y != 104; ++y)
		{
			for (auto x = 24; x != 104; ++x)
			{
				auto pixel = (y * ImageSize + x) * 4;
				if (object_reference[pixel + 0] != moved_object[pixel + 0] || object_reference[pixel + 1] != moved_object[pixel + 1] || object_reference[pixel + 2] != moved_object[pixel + 2])
					++object_difference;
			}
		}
		Require(object_difference < 100, "Object-coordinate procedural field did not move with the object");
		translated.w.x = 0;
		View3D_ObjectO2WSet(object, translated, nullptr);
		camera.w.x = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		SceneLightingSet(fixture.m_window, default_light);

		// A large translated object can retain local detail by selecting a matching caller-owned coordinate origin.
		auto large = soil;
		View3D_ObjectNuggetProceduralSurfaceSet(object, large, nullptr, 0);
		fixture.Image();
		large.m_coordinate_origin = {1000000, 0, 0, 1};
		View3D_ObjectNuggetProceduralSurfaceSet(object, large, nullptr, 0);
		translated.w.x = 1000000;
		View3D_ObjectO2WSet(object, translated, nullptr);
		camera.w.x = 1000000;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		auto large_image = fixture.Image();
		Require(large_image == fixture.Image(), "Large-coordinate procedural output was not deterministic");
		auto varied_pixels = size_t{};
		auto reference_pixel = (64 * ImageSize + 64) * 4;
		for (auto y = 24; y != 104; ++y)
		{
			for (auto x = 24; x != 104; ++x)
			{
				auto pixel = (y * ImageSize + x) * 4;
				if (large_image[reference_pixel + 0] != large_image[pixel + 0] || large_image[reference_pixel + 1] != large_image[pixel + 1] || large_image[reference_pixel + 2] != large_image[pixel + 2])
					++varied_pixels;
			}
		}
		Require(varied_pixels > 1000, "Large-coordinate procedural output lost local detail");

		// A broad field must not repeat like a wrapped 2D texture over separated image regions.
		auto non_repeating = large_image;
		auto patch_equal = true;
		for (auto y = 40; y != 88 && patch_equal; ++y)
		{
			for (auto x = 20; x != 44; ++x)
			{
				auto lhs = (y * ImageSize + x) * 4;
				auto rhs = (y * ImageSize + x + 64) * 4;
				if (non_repeating[lhs + 0] != non_repeating[rhs + 0] || non_repeating[lhs + 1] != non_repeating[rhs + 1] || non_repeating[lhs + 2] != non_repeating[rhs + 2])
				{
					patch_equal = false;
					break;
				}
			}
		}
		Require(!patch_equal, "Procedural field repeated across separated framebuffer regions");

		// Adjacent independently owned meshes must match the same world-coordinate field on both sides of their shared boundary.
		camera.w.x = 0;
		View3D_CameraToWorldSet(fixture.m_window, camera);
		SceneLightingSet(fixture.m_window, ambient_light);
		fixture.Clear();
		auto full = fixture.QuadRect(-45, 45, 10);
		View3D_ObjectNuggetProceduralSurfaceSet(full, soil, nullptr, 0);
		auto continuous_reference = fixture.Image();
		fixture.Clear();
		auto left = fixture.QuadRect(-45, 0, 10);
		auto right = fixture.QuadRect(0, 45, 10);
		View3D_ObjectNuggetProceduralSurfaceSet(left, soil, nullptr, 0);
		View3D_ObjectNuggetProceduralSurfaceSet(right, soil, nullptr, 0);
		auto split = fixture.Image();
		auto continuity_difference = size_t{};
		for (auto y = 8; y != ImageSize - 8; ++y)
		{
			for (auto x = 8; x != ImageSize - 8; ++x)
			{
				if (x >= 62 && x <= 65)
					continue;

				auto pixel = (y * ImageSize + x) * 4;
				if (continuous_reference[pixel + 0] != split[pixel + 0] || continuous_reference[pixel + 1] != split[pixel + 1] || continuous_reference[pixel + 2] != split[pixel + 2])
					++continuity_difference;
			}
		}
		Require(continuity_difference == 0, "World-coordinate procedural field changed across a mesh boundary");
		SceneLightingSet(fixture.m_window, default_light);

		// Promotion preserves an ordinary material's alpha path so procedural colour still composites as authored.
		fixture.Clear();
		SceneLightingSet(fixture.m_window, ambient_light);
		fixture.Quad(12, 0xFF0000FF);
		auto transparent = fixture.Quad(10, 0x80FFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		red.m_normal_strength = 0.0f;
		View3D_ObjectNuggetProceduralSurfaceSet(transparent, red, nullptr, 0);
		auto alpha_image = fixture.Image();
		auto alpha_pixel = (64 * ImageSize + 64) * 4;
		Require(alpha_image[alpha_pixel + 0] > 40 && alpha_image[alpha_pixel + 2] > 40, "Procedural assignment discarded ordinary alpha blending");

		// Custom shader overlays are rejected because stock PBR promotion cannot preserve caller-supplied shader stages.
		fixture.Clear();
		auto custom = fixture.Quad(10, 0xFFFFFFFF, 45, fixture.CustomShader(true), 0, 0xFFFFFFFF, false, true);
		View3D_ObjectNuggetProceduralSurfaceSet(custom, soil, nullptr, 0);
		Require(!fixture.m_errors.empty(), "Procedural assignment silently discarded a custom shader overlay");
		Require(fixture.m_errors.back().find("custom shader overlays") != std::string::npos, "Custom shader rejection reported the wrong diagnostic");
		fixture.m_errors.clear();
		api::ProceduralSurface round_trip{};
		Require(!View3D_ObjectNuggetProceduralSurfaceGet(custom, round_trip, nullptr, 0), "Rejected custom shader assignment changed the material");

		// Public round-trip and clear operations preserve ordinary material ownership.
		fixture.Clear();
		SceneLightingSet(fixture.m_window, ambient_light);
		auto round_trip_object = fixture.QuadRect(-45, 0, 10);
		View3D_ObjectNuggetProceduralSurfaceSet(round_trip_object, soil, nullptr, 0);
		Require(View3D_ObjectNuggetProceduralSurfaceGet(round_trip_object, round_trip, nullptr, 0), "Procedural surface getter did not find assigned state");
		Require(round_trip.m_seed == soil.m_seed && round_trip.m_feature_scale == soil.m_feature_scale, "Procedural surface getter changed caller parameters");

		// Unsupported DXR secondary-hit shading must report a material diagnostic instead of silently using flat PBR values.
		if (View3D_WindowRayTracingInfoGet(fixture.m_window).m_available)
		{
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			View3D_WindowRender(fixture.m_window);
			Require(!fixture.m_errors.empty(), "DXR silently accepted a procedural secondary-hit material");
			Require(fixture.m_errors.back().find("procedural surface") != std::string::npos, "DXR reported the wrong procedural material diagnostic");
			fixture.m_errors.clear();
			View3D_WindowRayTracingEnabledSet(fixture.m_window, FALSE);
		}
		View3D_ObjectNuggetProceduralSurfaceClear(round_trip_object, nullptr, 0);
		Require(!View3D_ObjectNuggetProceduralSurfaceGet(round_trip_object, round_trip, nullptr, 0), "Procedural surface clear retained assigned state");
		fixture.CheckDebugLayer();
		std::cout << "PASS procedural presets, determinism, channels, large coordinates, and non-repetition\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_ProceduralSurface, Extended | Serial)
	{
		render_tests::ProceduralSurfaceTests();
	}
	PRUnitTest(View3d12_ProceduralPixelFamily, Extended | Serial)
	{
		render_tests::ProceduralPixelFamilyTests();
	}
}
