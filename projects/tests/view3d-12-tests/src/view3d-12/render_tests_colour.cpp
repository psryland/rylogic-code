#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Read the packed blend value through the current single-value public API.
	api::Colour View3D_ObjectColourBlendColourGet(api::Object object, char const* name)
	{
		return ::View3D_ObjectColourBlendGet(object, name);
	}

	// Decode the packed alpha byte as the public linear blend weight.
	float View3D_ObjectColourBlendAmountGet(api::Object object, char const* name)
	{
		return ((::View3D_ObjectColourBlendGet(object, name) >> 24) & 0xFFU) / 255.0f;
	}

	// Validate the RGB override API and GPU output.
	void ColourBlendTests(int samples)
	{
		// The shared shader expression preserves alpha and the disabled value exactly, including HDR RGB.
		using colour_blend_tests::SurfaceColourBlend;
		auto const surface = pr::v4(1.5f, 0.25f, 0.75f, 0.37f);
		for (auto amount : {0.0f, 0.5f, 1.0f})
		{
			// Weights affect only RGB, not the original floating-point opacity.
			auto result = SurfaceColourBlend(surface, pr::v4(0.25f, 0.5f, 1.0f, amount));
			Require(result.w == surface.w, "Surface blend changed alpha");
			Require(result.x == surface.x + (0.25f - surface.x) * amount, "Surface blend weight differs");
		}

		// Use non-white vertices and material colour to detect accidental CPU tint/vertex multiplication.
		Fixture fixture(samples);
		auto quad = fixture.Quad(50, 0xFF808080, 45, nullptr, 0, 0xFF808080);
		auto original = fixture.Image();
		Require(View3D_ObjectColourBlendAmountGet(quad, nullptr) == 0, "New object blend is not disabled");
		auto const original_tint = View3D_ObjectColourGet(quad, FALSE, nullptr);
		auto const original_material = View3D_ObjectNuggetTintGet(quad, nullptr, 0);
		auto const original_flags = View3D_ObjectNuggetFlagsGet(quad, nullptr, 0);
		auto const original_sort = View3D_ObjectSortGroupGet(quad, nullptr);
		auto source = Linear(original[(64 * ImageSize + 64) * 4]);
		for (auto alpha : {0U, 128U, 255U, 0U})
		{
			// Packed alpha controls only RGB interpolation and never makes this opaque surface transparent.
			auto amount = alpha / 255.0f;
			auto blend = (alpha << 24) | 0x004080C0U;
			View3D_ObjectColourBlendSet(quad, blend, nullptr);
			auto image = fixture.Image();
			Expect(image, source + (Linear(64) - source) * amount, source + (Linear(128) - source) * amount, source + (Linear(192) - source) * amount);
			Require(View3D_ObjectColourBlendColourGet(quad, nullptr) == blend, "Packed target and weight did not round-trip");
			Require(View3D_ObjectColourBlendAmountGet(quad, nullptr) == amount, "Weight did not round-trip");
			Require(View3D_ObjectColourGet(quad, FALSE, nullptr) == original_tint, "Override mutated tint");
			Require(View3D_ObjectNuggetTintGet(quad, nullptr, 0) == original_material, "Override mutated material");
			Require(View3D_ObjectNuggetFlagsGet(quad, nullptr, 0) == original_flags, "Override mutated nugget flags");
			Require(View3D_ObjectSortGroupGet(quad, nullptr) == original_sort, "Override changed sort group");
			if (amount == 0)
				Require(image == original, "Zero override did not preserve exact pixels");
		}

		// Every alpha byte is valid and decodes linearly, with no sRGB conversion or separate weight storage.
		for (auto alpha = 0U; alpha != 256U; ++alpha)
		{
			// Exhaust the packed weight domain without requiring 256 GPU readbacks.
			auto blend = (alpha << 24) | 0x004080C0U;
			View3D_ObjectColourBlendSet(quad, blend, nullptr);
			Require(View3D_ObjectColourBlendColourGet(quad, nullptr) == blend, "Packed blend changed bytes");
			Require(View3D_ObjectColourBlendAmountGet(quad, nullptr) == alpha / 255.0f, "Packed weight did not decode as UNORM8");
		}

		// Texture transfer happens before the blend: encoded sRGB 0x80 is about 0.216 linear, not 0.502.
		fixture.Clear();
		auto textured = fixture.Quad(50, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, true);
		auto texture_options = api::TextureOptions{};
		texture_options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
		texture_options.m_usage = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
		texture_options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
		texture_options.m_clear_value.Format = texture_options.m_format;
		texture_options.m_mips = 1;
		texture_options.m_multisamp = {1, 0};
		texture_options.m_t2s = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
		unsigned texture_pixel = 0xFF808080;
		auto texture = View3D_TextureCreate(1, 1, &texture_pixel, sizeof(texture_pixel), texture_options);
		Require(texture != nullptr, "Texture creation failed");
		View3D_ObjectSetTexture(textured, texture, nullptr);
		View3D_TextureRelease(texture);
		for (auto alpha : {0U, 128U, 255U})
		{
			// A white override replaces even a dark texel, rather than multiplying it again.
			auto amount = alpha / 255.0f;
			View3D_ObjectColourBlendSet(textured, (alpha << 24) | 0x00FFFFFFU, nullptr);
			auto expected = Linear(128) + (1 - Linear(128)) * amount;
			Expect(fixture.Image(), expected, expected, expected);
		}

		// Lit simple and PBR surfaces must match an equivalent authored base colour at full override.
		auto light = SceneLightingGet(fixture.m_window);
		light.m_ambient = 0xFF808080;
		light.m_diffuse = light.m_specular = 0xFF000000;
		light.m_on = TRUE;
		light.m_intensity = 1;
		SceneLightingSet(fixture.m_window, light);
		for (auto pbr : {false, true})
		{
			// Keep geometry and lighting identical while varying only the material's linear RGB.
			auto plane = [&](char const* colour)
			{
				// The normal-bearing plane exercises lighting, unlike the unlit vertex-colour fixture above.
				auto script = pbr
					? std::string("*Plane Lit { *Data {90 90} *Material {*BaseColour{") + colour + "} *Metallic{0} *Roughness{1}} *o2w{*pos{0 0 -50}}}"
					: std::string("*Plane Lit ") + colour + " { *Data {90 90} *o2w{*pos{0 0 -50}}}";
				auto object = View3D_ObjectCreateLdrA(script.c_str(), FALSE, nullptr, nullptr);
				Require(object != nullptr, "Lit plane creation failed");
				fixture.m_objects.push_back(object);
				View3D_WindowAddObject(fixture.m_window, object);
				return object;
			};
			fixture.Clear();
			plane("FF004000");
			auto expected = fixture.Image();
			Require(expected[(64 * ImageSize + 64) * 4 + 1] > 0, "Lit reference did not render");
			fixture.Clear();
			auto overridden = plane("FF800000");
			View3D_ObjectColourBlendSet(overridden, 0xFF004000, nullptr);
			Require(expected == fixture.Image(), "Override was not applied before simple/PBR lighting");
		}

		// Existing alpha and K-buffer ordering must survive RGB replacement and reversed submission order.
		fixture.Clear();
		auto back = fixture.Quad(60, 0xFF0000FF);
		auto front = fixture.Quad(40, 0x80FF0000);
		auto const midpoint = 128.0f / 255;
		View3D_ObjectColourBlendSet(front, 0x8000FF00, nullptr);
		Expect(fixture.Image(), midpoint * (1 - midpoint), midpoint * midpoint, 1 - midpoint);
		View3D_ObjectColourBlendSet(front, 0xFF00FF00, nullptr);
		auto ordered = fixture.Image();
		Expect(ordered, 0, 128.0f / 255, 127.0f / 255);
		fixture.Clear();
		View3D_WindowAddObject(fixture.m_window, front);
		View3D_WindowAddObject(fixture.m_window, back);
		Require(ordered == fixture.Image(), "RGB override changed sorted alpha output");

		// Put the coloured target behind the camera so only a real reflected hit can show its RGB.
		if (View3D_WindowRayTracingInfoGet(fixture.m_window).m_available)
		{
			// The same target remains in the TLAS while its override changes, exercising reusable material-buffer updates.
			fixture.Clear();
			auto mirror = View3D_ObjectCreateLdrA("*Plane Mirror FF000000 {*Data{90 90} *Reflectivity{1} *o2w{*pos{0 0 -50}}}", FALSE, nullptr, nullptr);
			Require(mirror != nullptr, "Mirror creation failed");
			fixture.m_objects.push_back(mirror);
			View3D_WindowAddObject(fixture.m_window, mirror);
			auto reflected = fixture.Quad(-10, 0xFF000000);
			View3D_RayTracingPropertiesSet(fixture.m_window, {api::ERayTracingFeature::Reflections, 1});
			View3D_WindowRayTracingEnabledSet(fixture.m_window, TRUE);
			Require(View3D_WindowRayTracingEnabledGet(fixture.m_window), "Ray tracing did not enable");
			for (auto alpha : {0U, 128U, 255U, 0U})
			{
				// The hidden target is unlit, so reflected RGB directly reveals the blend weight and transfer function.
				auto amount = alpha / 255.0f;
				View3D_ObjectColourBlendSet(reflected, (alpha << 24) | 0x00008000U, nullptr);
				Expect(fixture.Image(), 0, Linear(128) * amount, 0);
			}
			View3D_WindowRayTracingEnabledSet(fixture.m_window, FALSE);
		}
		else
		{
			// Hardware availability is reported rather than mistaking a raster-only pass for reflection coverage.
			std::cout << "SKIP colour blend reflected-hit test: DXR unavailable\n";
		}

		// Name selection and cloning operate on instance state, including model-less hierarchy roots.
		auto root = View3D_ObjectCreateLdrA("*Group Root {*Sphere Child {1} *Sphere Other {1}}", FALSE, nullptr, nullptr);
		Require(root != nullptr, "Hierarchy creation failed");
		fixture.m_objects.push_back(root);
		View3D_ObjectColourBlendSet(root, 0x80123456, nullptr);
		Require(View3D_ObjectColourBlendAmountGet(root, "Child") == 0, "Self selection affected a child");
		View3D_ObjectColourBlendSet(root, 0xFFABCDEF, "Child");
		Require(View3D_ObjectColourBlendAmountGet(root, nullptr) == midpoint, "Named selection affected root");
		Require(View3D_ObjectColourBlendAmountGet(root, "Other") == 0, "Named selection affected sibling");
		View3D_ObjectColourBlendSet(root, 0x80373737, "");
		auto clone = View3D_ObjectCreateInstance(root);
		Require(clone != nullptr, "Clone creation failed");
		fixture.m_objects.push_back(clone);
		Require(View3D_ObjectColourBlendAmountGet(clone, "Child") == midpoint, "Clone lost child override");
		View3D_ObjectColourBlendSet(clone, 0xFFFFFFFF, "");
		Require(View3D_ObjectColourBlendColourGet(root, "Child") == 0x80373737, "Clone mutated source override");
		View3D_ObjectResetColour(root, "");
		Require(View3D_ObjectColourBlendAmountGet(root, "Child") == midpoint, "Tint reset changed independent override");
		View3D_ObjectUpdate(root, L"*Group Root {}", api::EUpdateObject::Colour);
		Require(View3D_ObjectColourBlendAmountGet(root, nullptr) == 0, "Colour replacement did not reset override");
		fixture.CheckErrors();

		// Missing selections return the same disabled packed value as a new object.
		Require(View3D_ObjectColourBlendColourGet(root, "Missing") == 0, "Missing selection returned an active packed override");
		Require(View3D_ObjectColourBlendAmountGet(root, "Missing") == 0, "Missing selection returned an active weight");
		fixture.CheckDebugLayer();
		std::cout << "PASS packed colour blend identity/UNORM8 weights/linear RGB/opacity/sorting/scope/clone/reset: MSAA " << samples << '\n';
	}

	// Check that translucent surfaces composite in depth order regardless of add order, and that back faces are still drawn.
	void AlphaLayeringTests(int samples)
	{
		// Half-transparent red behind half-transparent green, over black: red = 0.5 * 0.5, green = 0.5.
		auto fixture = Fixture(samples);
		fixture.Quad(20, 0x80FF0000);
		fixture.Quad(10, 0x8000FF00);
		Expect(fixture.Image(), 0.25f, 0.5f, 0);

		// Reversing the add order must not change the depth-ordered result.
		fixture.Clear();
		fixture.Quad(10, 0x8000FF00);
		fixture.Quad(20, 0x80FF0000);
		Expect(fixture.Image(), 0.25f, 0.5f, 0);

		// Turn the near quad around so it faces away from the camera; a single cull-none alpha draw must still include it.
		fixture.Clear();
		fixture.Quad(20, 0x80FF0000);
		auto back = fixture.Quad(10, 0x8000FF00);
		api::Mat4x4 turned{{-1,0,0,0}, {0,1,0,0}, {0,0,-1,0}, {0,0,-20,1}};
		View3D_ObjectO2WSet(back, turned, nullptr);
		fixture.CheckErrors();
		Expect(fixture.Image(), 0.25f, 0.5f, 0);

		// Any GPU validation error fails the fixture.
		fixture.CheckDebugLayer();
		std::cout << "PASS alpha layering (" << samples << "x)\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_ColourBlend, Extended | Serial)
	{
		render_tests::ColourBlendTests(1);
		render_tests::ColourBlendTests(4);
	}
	PRUnitTest(View3d12_AlphaLayering, Extended | Serial)
	{
		render_tests::AlphaLayeringTests(1);
		render_tests::AlphaLayeringTests(4);
	}
}
