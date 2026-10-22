#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Check that detail normals tilt lighting only while layers are set, and can be removed.
	void DetailNormalsTests()
	{
		// A lit white quad faces the camera, with a directional light from the -X side so a normal tilt along X changes its brightness.
		auto fixture = Fixture(1);
		auto object = fixture.Quad(10, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto light = SceneLightingGet(fixture.m_window);
		light.m_type = api::ELight::Directional;
		light.m_direction = api::Vec4{0.70710678f, 0, -0.70710678f, 0};
		light.m_ambient = 0xFF000000;
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 1;
		light.m_cast_shadow = 0;
		light.m_on = TRUE;
		SceneLightingSet(fixture.m_window, light);
		auto brightness = [](std::vector<unsigned char> const& image)
		{
			// The quad is grey, so one channel at the centre measures its lighting.
			return Linear(image[(64 * ImageSize + 64) * 4 + 1]);
		};
		auto flat = fixture.Image();

		// A uniform slope map of +1 along u, 0 along v, with half the mean square slope (0.5) in blue. Wrapping matches the tileable maps
		// the feature is designed for.
		auto options = api::TextureOptions{};
		options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
		options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
		options.m_mips = 0;
		options.m_multisamp = {1, 0};
		options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
		std::array<uint32_t, 4> pixels; pixels.fill(0xFF8080FFU);
		auto texture = api::TexturePtr(View3D_TextureCreate(2, 2, pixels.data(), sizeof(pixels), options));
		auto sampler = api::SamplerPtr(View3D_SamplerCreate(api::SamplerOptions{D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP, "DetailNormals"}));
		fixture.CheckErrors();
		Require(texture != nullptr && sampler != nullptr, "Detail-normal resources were not created");

		// A map without layers leaves the surface unchanged.
		View3D_ObjectNuggetDetailNormalsSet(object, texture.get(), sampler.get(), nullptr, 0);
		fixture.CheckErrors();
		Require(ImageDifference(flat, fixture.Image()) < 100, "Detail normals without layers changed the surface");

		// Height rising along +X with gradient 0.5 tilts the normal to (-0.5, 0, 1). Diffuse lighting from 45 degrees then rises by
		// cos(45 - atan(0.5)) / cos(45), about 1.342, relative to the flat surface.
		auto layer = api::DetailNormalLayer{};
		layer.m_row_u = api::Vec4{1, 0, 0, 0};
		layer.m_row_v = api::Vec4{0, 1, 0, 0};
		layer.m_height_scale = 0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &layer, 1, 0.0f);
		fixture.CheckErrors();
		auto tilted = fixture.Image();
		Require(std::abs(brightness(tilted) / brightness(flat) - 1.342f) < 0.03f, (std::string("Detail-normal layer tilted the lighting normal incorrectly: flat ") + std::to_string(brightness(flat)) + ", tilted " + std::to_string(brightness(tilted))).c_str());

		// Weight noise varies the tilt across the quad, which spans several noise cells. Without weight noise the frequency has no effect.
		auto noisy_layer = layer;
		noisy_layer.m_weight_noise = 1.0f;
		noisy_layer.m_noise_frequency = 0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &noisy_layer, 1, 0.0f);
		fixture.CheckErrors();
		auto noisy_difference = ImageDifference(tilted, fixture.Image());
		Require(noisy_difference > 100, (std::string("Detail-normal weight noise did not vary the surface: difference ") + std::to_string(noisy_difference)).c_str());
		noisy_layer.m_weight_noise = 0.0f;
		View3D_ObjectDetailNormalLayersSet(object, &noisy_layer, 1, 0.0f);
		fixture.CheckErrors();
		Require(ImageDifference(tilted, fixture.Image()) < 100, "Noise frequency changed the surface without weight noise");

		// Warp shifts where a non-uniform map is sampled. Columns of opposite u slope make the shift visible, while a uniform map would hide it.
		std::array<uint32_t, 4> stripes = {0xFF8080FFU, 0xFF808000U, 0xFF8080FFU, 0xFF808000U};
		auto striped = api::TexturePtr(View3D_TextureCreate(2, 2, stripes.data(), sizeof(stripes), options));
		Require(striped != nullptr, "Striped detail-normal map was not created");
		auto stripe_layer = layer;
		stripe_layer.m_noise_frequency = 0.5f;
		View3D_ObjectNuggetDetailNormalsSet(object, striped.get(), sampler.get(), nullptr, 0);
		View3D_ObjectDetailNormalLayersSet(object, &stripe_layer, 1, 0.0f);
		fixture.CheckErrors();
		auto unwarped = fixture.Image();
		stripe_layer.m_warp = 0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &stripe_layer, 1, 0.0f);
		fixture.CheckErrors();
		auto warp_difference = ImageDifference(unwarped, fixture.Image());
		Require(warp_difference > 100, (std::string("Detail-normal warp did not move the map: difference ") + std::to_string(warp_difference)).c_str());
		View3D_ObjectNuggetDetailNormalsSet(object, texture.get(), sampler.get(), nullptr, 0);
		View3D_ObjectDetailNormalLayersSet(object, &stripe_layer, 1, 0.0f);
		fixture.CheckErrors();
		Require(ImageDifference(tilted, fixture.Image()) < 100, "Warp changed a uniform map");

		// Removing the layers restores the flat surface without replacing the material.
		View3D_ObjectDetailNormalLayersSet(object, nullptr, 0, 0.0f);
		fixture.CheckErrors();
		Require(ImageDifference(flat, fixture.Image()) < 100, "Clearing detail-normal layers did not restore the surface");

		// Invalid inputs are reported and leave the active state unchanged.
		auto layers = std::array<api::DetailNormalLayer, 5>{layer, layer, layer, layer, layer};
		View3D_ObjectDetailNormalLayersSet(object, layers.data(), 5, 0.0f);
		Require(!fixture.m_errors.empty(), "Too many detail-normal layers were accepted");
		fixture.m_errors.clear();
		View3D_ObjectDetailNormalLayersSet(object, &layer, 1, -0.1f);
		Require(!fixture.m_errors.empty(), "A negative base slope variance was accepted");
		fixture.m_errors.clear();
		auto bad_noise = layer;
		bad_noise.m_weight_noise = 0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &bad_noise, 1, 0.0f);
		Require(!fixture.m_errors.empty(), "Weight noise without a positive noise frequency was accepted");
		fixture.m_errors.clear();
		auto bad_warp = layer;
		bad_warp.m_warp = 0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &bad_warp, 1, 0.0f);
		Require(!fixture.m_errors.empty(), "Warp without a positive noise frequency was accepted");
		fixture.m_errors.clear();
		bad_warp.m_noise_frequency = 0.5f;
		bad_warp.m_warp = -0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &bad_warp, 1, 0.0f);
		Require(!fixture.m_errors.empty(), "A negative warp was accepted");
		fixture.m_errors.clear();
		View3D_ObjectNuggetDetailNormalsSet(object, texture.get(), nullptr, nullptr, 0);
		Require(!fixture.m_errors.empty(), "A detail-normal map without a sampler was accepted");
		fixture.m_errors.clear();

		// Removing the component restores the stock shader; layer updates then report that the object has no detail normals.
		View3D_ObjectDetailNormalLayersSet(object, &layer, 1, 0.0f);
		View3D_ObjectNuggetDetailNormalsSet(object, nullptr, nullptr, nullptr, 0);
		fixture.CheckErrors();
		Require(ImageDifference(flat, fixture.Image()) < 100, "Removing detail normals did not restore the surface");
		View3D_ObjectDetailNormalLayersSet(object, &layer, 1, 0.0f);
		Require(!fixture.m_errors.empty(), "Layers were accepted by an object without detail normals");
		fixture.m_errors.clear();

		// Translucent surfaces render through the alpha collection pass, which must apply the same tilt. Blending over the black background
		// scales the brightness equally with and without the tilt, so the ratio is unchanged.
		View3D_WindowRemoveObject(fixture.m_window, object);
		auto translucent = fixture.Quad(10, 0x80FFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto translucent_flat = fixture.Image();
		View3D_ObjectNuggetDetailNormalsSet(translucent, texture.get(), sampler.get(), nullptr, 0);
		View3D_ObjectDetailNormalLayersSet(translucent, &layer, 1, 0.0f);
		fixture.CheckErrors();
		auto translucent_tilted = fixture.Image();
		Require(std::abs(brightness(translucent_tilted) / brightness(translucent_flat) - 1.342f) < 0.05f, (std::string("Translucent detail-normal layer tilted the lighting normal incorrectly: flat ") + std::to_string(brightness(translucent_flat)) + ", tilted " + std::to_string(brightness(translucent_tilted))).c_str());

		// Any GPU validation error fails the fixture.
		fixture.CheckDebugLayer();
		std::cout << "PASS detail normals\n";
	}

	// Check that a detail strength map scales the layers over the surface, can be visualised, and can be removed.
	void DetailStrengthMapTests()
	{
		// The same lit quad and uniform slope map as DetailNormalsTests, with one layer of gradient 0.5.
		auto fixture = Fixture(1);
		auto object = fixture.Quad(10, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto light = SceneLightingGet(fixture.m_window);
		light.m_type = api::ELight::Directional;
		light.m_direction = api::Vec4{0.70710678f, 0, -0.70710678f, 0};
		light.m_ambient = 0xFF000000;
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 1;
		light.m_cast_shadow = 0;
		light.m_on = TRUE;
		SceneLightingSet(fixture.m_window, light);
		auto brightness = [](std::vector<unsigned char> const& image)
		{
			// The quad is grey, so one channel at the centre measures its lighting.
			return Linear(image[(64 * ImageSize + 64) * 4 + 1]);
		};
		auto flat = fixture.Image();

		auto options = api::TextureOptions{};
		options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
		options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
		options.m_mips = 0;
		options.m_multisamp = {1, 0};
		options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
		std::array<uint32_t, 4> pixels; pixels.fill(0xFF8080FFU);
		auto texture = api::TexturePtr(View3D_TextureCreate(2, 2, pixels.data(), sizeof(pixels), options));
		auto sampler = api::SamplerPtr(View3D_SamplerCreate(api::SamplerOptions{D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP, "DetailNormals"}));
		View3D_ObjectNuggetDetailNormalsSet(object, texture.get(), sampler.get(), nullptr, 0);
		auto layer = api::DetailNormalLayer{};
		layer.m_row_u = api::Vec4{1, 0, 0, 0};
		layer.m_row_v = api::Vec4{0, 1, 0, 0};
		layer.m_height_scale = 0.5f;
		View3D_ObjectDetailNormalLayersSet(object, &layer, 1, 0.0f);
		fixture.CheckErrors();
		auto tilted = fixture.Image();

		// A new map has strength (1, 0) everywhere, so it leaves the layers unchanged.
		auto map = api::DetailStrengthMapPtr(View3D_DetailStrengthMapCreate(2, 1));
		Require(map != nullptr, "Detail strength map was not created");
		View3D_ObjectDetailStrengthMapSet(object, map.get());
		fixture.CheckErrors();
		Require(ImageDifference(tilted, fixture.Image()) < 100, "A default detail strength map changed the surface");

		// Strength 2 doubles the gradient to 1, tilting the normal to (-1, 0, 1), so the 45 degree light hits it face on: 1 / cos(45) = 1.414.
		auto strong = std::array<api::Vec4, 2>{api::Vec4{2, 0, 1, 0}, api::Vec4{2, 0, 1, 0}};
		View3D_DetailStrengthMapUpload(map.get(), strong.data(), 2);
		fixture.CheckErrors();
		auto doubled = fixture.Image();
		Require(std::abs(brightness(doubled) / brightness(flat) - 1.414f) < 0.03f, (std::string("Detail strength 2 scaled the tilt incorrectly: flat ") + std::to_string(brightness(flat)) + ", doubled " + std::to_string(brightness(doubled))).c_str());

		// Strength 0 removes the tilt.
		auto none = std::array<api::Vec4, 2>{api::Vec4{0, 0, 1, 0}, api::Vec4{0, 0, 1, 0}};
		View3D_DetailStrengthMapUpload(map.get(), none.data(), 2);
		fixture.CheckErrors();
		Require(ImageDifference(flat, fixture.Image()) < 100, "Detail strength 0 did not remove the tilt");

		// Texels of strength 0 and 2 across the quad are filtered to 1 at its centre, which the projection puts midway between the texel centres.
		auto ramp = std::array<api::Vec4, 2>{api::Vec4{0, 0, 1, 0}, api::Vec4{2, 0, 1, 0}};
		View3D_DetailStrengthMapUpload(map.get(), ramp.data(), 2);
		View3D_DetailStrengthMapProjectionSet(map.get(), api::Vec4{0.1f, 0, 0, 0.5f}, api::Vec4{0, 0.1f, 0, 0.5f});
		fixture.CheckErrors();
		auto ramped = fixture.Image();
		Require(std::abs(brightness(ramped) / brightness(tilted) - 1.0f) < 0.03f, (std::string("Detail strength was not filtered across the map: tilted ") + std::to_string(brightness(tilted)) + ", ramped " + std::to_string(brightness(ramped))).c_str());
		Require(ImageDifference(tilted, ramped) > 100, "Detail strength did not vary across the surface");

		// Visualisation replaces the shaded colour, and turning it off restores the shading.
		View3D_DetailStrengthMapVisualiseSet(map.get(), TRUE);
		fixture.CheckErrors();
		Require(ImageDifference(ramped, fixture.Image()) > 100, "Detail strength visualisation did not change the surface");
		View3D_DetailStrengthMapVisualiseSet(map.get(), FALSE);
		fixture.CheckErrors();
		Require(ImageDifference(ramped, fixture.Image()) < 100, "Turning off detail strength visualisation did not restore the surface");

		// A half turn reverses the gradient to (-0.5, 0, 0), tilting the normal away from the light: (1 - 0.5) / sqrt(1.25) / cos(45) = 0.447.
		auto reversed = std::array<api::Vec4, 2>{api::Vec4{1, 0, -1, 0}, api::Vec4{1, 0, -1, 0}};
		View3D_DetailStrengthMapUpload(map.get(), reversed.data(), 2);
		fixture.CheckErrors();
		auto turned = fixture.Image();
		Require(std::abs(brightness(turned) / brightness(flat) - 0.447f) < 0.03f, (std::string("A detail strength half turn did not reverse the tilt: flat ") + std::to_string(brightness(flat)) + ", turned " + std::to_string(brightness(turned))).c_str());

		// Texels turned by a quarter turn either way blend almost equally at the centre, where their sideways gradients nearly cancel. Either
		// texel alone tilts the normal across the light (1 / sqrt(1.25) = 0.894), and an unturned layer tilts it toward the light (1.342).
		auto opposed = std::array<api::Vec4, 2>{api::Vec4{1, 0, 0, 1}, api::Vec4{1, 0, 0, -1}};
		View3D_DetailStrengthMapUpload(map.get(), opposed.data(), 2);
		fixture.CheckErrors();
		auto blended = fixture.Image();
		Require(std::abs(brightness(blended) / brightness(flat) - 1.0f) < 0.03f, (std::string("Turned detail strength texels were not blended: flat ") + std::to_string(brightness(flat)) + ", blended " + std::to_string(brightness(blended))).c_str());
		View3D_DetailStrengthMapUpload(map.get(), ramp.data(), 2);
		fixture.CheckErrors();

		// Invalid inputs are reported and leave the map unchanged.
		auto negative = std::array<api::Vec4, 2>{api::Vec4{-1, 0, 1, 0}, api::Vec4{1, 0, 1, 0}};
		View3D_DetailStrengthMapUpload(map.get(), negative.data(), 2);
		Require(!fixture.m_errors.empty(), "A negative detail strength was accepted");
		fixture.m_errors.clear();
		View3D_DetailStrengthMapUpload(map.get(), strong.data(), 1);
		Require(!fixture.m_errors.empty(), "A detail strength upload of the wrong size was accepted");
		fixture.m_errors.clear();
		auto unnormalised = std::array<api::Vec4, 2>{api::Vec4{1, 0, 2, 0}, api::Vec4{1, 0, 1, 0}};
		View3D_DetailStrengthMapUpload(map.get(), unnormalised.data(), 2);
		Require(!fixture.m_errors.empty(), "A detail strength turn that is not a unit vector was accepted");
		fixture.m_errors.clear();
		View3D_DetailStrengthMapProjectionSet(map.get(), api::Vec4{0.1f, 0, 0, 0}, api::Vec4{0.2f, 0, 0, 0});
		Require(!fixture.m_errors.empty(), "A detail strength projection with parallel rows was accepted");
		fixture.m_errors.clear();
		Require(View3D_DetailStrengthMapCreate(0, 1) == nullptr && !fixture.m_errors.empty(), "A zero-width detail strength map was accepted");
		fixture.m_errors.clear();
		Require(ImageDifference(ramped, fixture.Image()) < 100, "Invalid detail strength inputs changed the surface");

		// Removing the map restores the unscaled layers. The object keeps its own reference, so releasing the handle first is safe.
		map.reset();
		fixture.CheckErrors();
		Require(ImageDifference(ramped, fixture.Image()) < 100, "Releasing the detail strength map handle changed the surface");
		View3D_ObjectDetailStrengthMapSet(object, nullptr);
		fixture.CheckErrors();
		Require(ImageDifference(tilted, fixture.Image()) < 100, "Removing the detail strength map did not restore the layers");

		// Any GPU validation error fails the fixture.
		fixture.CheckDebugLayer();
		std::cout << "PASS detail strength map\n";
	}

	// Check that a model-space normal map replaces the vertex normal, and that removing it restores the surface.
	void ModelNormalMapTests()
	{
		// A lit white quad faces the camera, with a directional light from the -X side so a normal tilt along X changes its brightness.
		auto fixture = Fixture(1);
		auto object = fixture.Quad(10, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, true, true);
		auto light = SceneLightingGet(fixture.m_window);
		light.m_type = api::ELight::Directional;
		light.m_direction = api::Vec4{0.70710678f, 0, -0.70710678f, 0};
		light.m_ambient = 0xFF000000;
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 1;
		light.m_cast_shadow = 0;
		light.m_on = TRUE;
		SceneLightingSet(fixture.m_window, light);
		auto brightness = [](std::vector<unsigned char> const& image)
		{
			// The quad is grey, so one channel at the centre measures its lighting.
			return Linear(image[(64 * ImageSize + 64) * 4 + 1]);
		};

		// An empty normal-map slot promotes the nugget to PBR without changing its normal, giving the flat reference.
		View3D_ObjectNuggetNormalMapSet(object, nullptr, nullptr, api::ENormalMapSpace::Model, 1.0f, nullptr, 0);
		fixture.CheckErrors();
		auto flat = fixture.Image();

		// A uniform map holding the model-space normal (-0.5, 0, 1) normalised. X encodes as 70 of 255, Y as 128, and Z is rebuilt.
		// Diffuse lighting from 45 degrees then rises by cos(45 - atan(0.5)) / cos(45), about 1.342, relative to the flat surface.
		auto options = api::TextureOptions{};
		options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
		options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
		options.m_mips = 0;
		options.m_multisamp = {1, 0};
		options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
		std::array<uint32_t, 4> pixels; pixels.fill(0xFF008046U);
		auto texture = api::TexturePtr(View3D_TextureCreate(2, 2, pixels.data(), sizeof(pixels), options));
		auto sampler = api::SamplerPtr(View3D_SamplerCreate(api::SamplerOptions{D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, "ModelNormalMap"}));
		fixture.CheckErrors();
		Require(texture != nullptr && sampler != nullptr, "Model normal-map resources were not created");
		View3D_ObjectNuggetNormalMapSet(object, texture.get(), sampler.get(), api::ENormalMapSpace::Model, 1.0f, nullptr, 0);
		fixture.CheckErrors();
		auto tilted = fixture.Image();
		Require(std::abs(brightness(tilted) / brightness(flat) - 1.342f) < 0.03f, (std::string("Model normal map tilted the lighting normal incorrectly: flat ") + std::to_string(brightness(flat)) + ", tilted " + std::to_string(brightness(tilted))).c_str());

		// Invalid inputs are reported and leave the active map unchanged.
		View3D_ObjectNuggetNormalMapSet(object, texture.get(), nullptr, api::ENormalMapSpace::Model, 1.0f, nullptr, 0);
		Require(!fixture.m_errors.empty(), "A normal map without a sampler was accepted");
		fixture.m_errors.clear();
		View3D_ObjectNuggetNormalMapSet(object, texture.get(), sampler.get(), api::ENormalMapSpace::Model, std::numeric_limits<float>::infinity(), nullptr, 0);
		Require(!fixture.m_errors.empty(), "A non-finite normal-map scale was accepted");
		fixture.m_errors.clear();
		View3D_ObjectNuggetNormalMapSet(object, texture.get(), sampler.get(), static_cast<api::ENormalMapSpace>(-1), 1.0f, nullptr, 0);
		Require(!fixture.m_errors.empty(), "An unknown normal-map space was accepted");
		fixture.m_errors.clear();
		Require(ImageDifference(tilted, fixture.Image()) < 100, "A rejected normal-map update changed the surface");

		// Removing the map restores the flat surface.
		View3D_ObjectNuggetNormalMapSet(object, nullptr, nullptr, api::ENormalMapSpace::Model, 1.0f, nullptr, 0);
		fixture.CheckErrors();
		Require(ImageDifference(flat, fixture.Image()) < 100, "Removing the model normal map did not restore the surface");

		// Any GPU validation error fails the fixture.
		fixture.CheckDebugLayer();
		std::cout << "PASS model normal map\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_DetailNormals, Extended)
	{
		render_tests::DetailNormalsTests();
	}
	PRUnitTest(View3d12_DetailStrengthMap, Extended)
	{
		render_tests::DetailStrengthMapTests();
	}
	PRUnitTest(View3d12_ModelNormalMap, Extended)
	{
		render_tests::ModelNormalMapTests();
	}
}
