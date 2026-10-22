#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Average linear red over an image rectangle
	float RegionRed(std::vector<unsigned char> const& image, int x0, int x1, int y0, int y1)
	{
		// Red is enough because the test lights and surfaces are grey
		auto sum = 0.0f;
		for (auto y = y0; y != y1; ++y)
		{
			for (auto x = x0; x != x1; ++x)
				sum += Linear(image[4 * (y * ImageSize + x)]);
		}
		return sum / ((x1 - x0) * (y1 - y0));
	}

	// Verify shadow atlas rendering for point and spot lights, batching across viewport arrays, and the shadow settings API
	void ShadowTests(int samples)
	{
		// A receiver behind a narrow caster, lit from the side so the shadow falls on the visible part of the receiver.
		// The caster covers world x in [-10,10] at depth 40. A light at (30,0,-20) projects it onto the receiver at depth 60 over x in [-50,-10].
		Fixture fixture(samples);
		auto receiver = fixture.Quad(60, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto caster = fixture.Quad(40, 0xFFFFFFFF, 10, nullptr, 0, 0xFFFFFFFF, false, true);
		(void)receiver;

		// Image regions (pixels) inside the expected shadow and on the lit receiver outside it
		auto shadow_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 15, 35, 40, 88); };
		auto lit_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 90, 110, 40, 88); };

		// Replace the default directional light with a single point light that casts shadows
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
		auto point_index = View3D_LightAdd(fixture.m_window, light);
		fixture.CheckErrors();

		// Settings round trip with the documented defaults
		auto settings = View3D_ShadowSettingsGet(fixture.m_window);
		Require(settings.m_atlas_size == 4096 && settings.m_max_shadow_lights == 4, "Unexpected default shadow settings");

		// Compare the image with and without the caster in the shadow pass
		auto shadow_difference = [&](char const* message)
		{
			// The caster must darken its shadow region without changing the lit region
			auto with_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
			auto without_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
			fixture.CheckErrors();
			Require(shadow_region(with_shadow) < 0.5f * shadow_region(without_shadow), message);
			Require(std::abs(lit_region(with_shadow) - lit_region(without_shadow)) < 0.02f, "Shadow darkened a region the caster cannot shade");
			return ImageDifference(with_shadow, without_shadow);
		};

		// Point lights render six cube face views
		shadow_difference("Point light cube shadow missing");

		// Spot lights render one perspective view
		light.m_type = api::ELight::Spot;
		light.m_direction = api::Vec4{-0.6f, 0, -0.8f, 0};
		View3D_LightSet(fixture.m_window, point_index, light);
		shadow_difference("Spot light shadow missing");

		// Four point shadow lights need 24 views, more than one viewport batch. The first light's shadow must survive batching.
		// The extra lights are far above the scene so they light the receiver evenly without shading the tested regions.
		light.m_type = api::ELight::Point;
		View3D_LightSet(fixture.m_window, point_index, light);
		for (int i = 0; i != 3; ++i)
		{
			auto extra = light;
			extra.m_position = api::Vec4{-40.0f + 40.0f * i, 60, 200, 1};
			extra.m_intensity = 0.1f;
			View3D_LightAdd(fixture.m_window, extra);
		}
		fixture.CheckErrors();
		shadow_difference("Point light shadow missing when shadow views span more than one batch");

		// Zero shadow lights disables shadows, making the caster flag irrelevant
		settings.m_max_shadow_lights = 0;
		View3D_ShadowSettingsSet(fixture.m_window, settings);
		{
			auto disabled = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
			auto excluded = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
			fixture.CheckErrors();
			Require(ImageDifference(disabled, excluded) == 0, "Shadows rendered with zero shadow lights");
		}

		// A smaller atlas recreates the atlas and still renders shadows
		settings.m_max_shadow_lights = 4;
		settings.m_atlas_size = 1024;
		View3D_ShadowSettingsSet(fixture.m_window, settings);
		Require(View3D_ShadowSettingsGet(fixture.m_window).m_atlas_size == 1024, "Shadow settings did not round trip");
		shadow_difference("Shadow missing after atlas resize");

		// Invalid settings are reported and leave the current settings unchanged
		auto invalid = settings;
		invalid.m_atlas_size = 1000;
		View3D_ShadowSettingsSet(fixture.m_window, invalid);
		Require(!fixture.m_errors.empty(), "Non power-of-two atlas size was accepted");
		fixture.m_errors.clear();
		Require(View3D_ShadowSettingsGet(fixture.m_window).m_atlas_size == 1024, "Rejected shadow settings changed the scene");
		std::cout << "PASS point/spot shadow views, viewport batching, shadow light limit, atlas resize, and settings validation: MSAA " << samples << '\n';
	}

	// Verify directional light cascades, the shadow edge filter sizes, and shadow view caching
	void DirectionalShadowTests(int samples)
	{
		// A receiver behind a narrow caster, lit by a directional light travelling along (-1.5, 0, -1).
		// The caster covers world x in [-10,10] at depth 40, so its shadow on the receiver at depth 60 covers x in [-40,-20].
		Fixture fixture(samples);
		auto receiver = fixture.Quad(60, 0xFFFFFFFF, 45, nullptr, 0, 0xFFFFFFFF, false, true);
		auto caster = fixture.Quad(40, 0xFFFFFFFF, 10, nullptr, 0, 0xFFFFFFFF, false, true);
		(void)receiver;

		// Image regions (pixels) inside the expected shadow and on the lit receiver outside it
		auto shadow_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 15, 35, 40, 88); };
		auto lit_region = [](std::vector<unsigned char> const& image) { return RegionRed(image, 90, 110, 40, 88); };

		// Make the main light a world-space directional light that casts shadows
		View3D_AmbientSet(fixture.m_window, 0xFF000000);
		auto light = View3D_LightGet(fixture.m_window, 0);
		light.m_type = api::ELight::Directional;
		light.m_direction = api::Vec4{-0.8320503f, 0, -0.5547002f, 0};
		light.m_diffuse = 0xFFFFFFFF;
		light.m_specular = 0xFF000000;
		light.m_intensity = 1;
		light.m_cast_shadow = 1.0f;
		light.m_cam_relative = FALSE;
		light.m_on = TRUE;
		View3D_LightSet(fixture.m_window, 0, light);
		fixture.CheckErrors();

		// The documented defaults
		auto settings = View3D_ShadowSettingsGet(fixture.m_window);
		Require(settings.m_cascade_count == 3 && settings.m_directional_resolution == 1024 && settings.m_filter_size == 5 && settings.m_shadow_distance == 0 && settings.m_cache_views != FALSE, "Unexpected default cascade settings");

		// Compare the image with and without the caster in the shadow pass
		auto shadow_difference = [&](char const* message)
		{
			// The caster must darken its shadow region without changing the lit region
			auto with_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, TRUE, nullptr);
			auto without_shadow = fixture.Image();
			View3D_ObjectFlagsSet(caster, api::ELdrFlags::ShadowCastExclude, FALSE, nullptr);
			fixture.CheckErrors();
			Require(shadow_region(with_shadow) < 0.5f * shadow_region(without_shadow), message);
			Require(std::abs(lit_region(with_shadow) - lit_region(without_shadow)) < 0.02f, "Shadow darkened a region the caster cannot shade");
		};

		// Every cascade count and both filter sizes render the shadow
		for (int cascades = 1; cascades <= 4; ++cascades)
		{
			for (int filter : { 5, 7 })
			{
				settings.m_cascade_count = cascades;
				settings.m_filter_size = filter;
				View3D_ShadowSettingsSet(fixture.m_window, settings);
				fixture.CheckErrors();
				shadow_difference("Directional cascade shadow missing");
			}
		}

		// Cached views must match views rendered every frame, both for a static scene and after a caster moves
		auto render_pair = [&](char const* message)
		{
			// Render twice with caching so the second image uses cached views, then once without caching
			settings.m_cache_views = TRUE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			fixture.Image();
			auto cached = fixture.Image();
			settings.m_cache_views = FALSE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			auto uncached = fixture.Image();
			settings.m_cache_views = TRUE;
			View3D_ShadowSettingsSet(fixture.m_window, settings);
			fixture.CheckErrors();
			Require(ImageDifference(cached, uncached) == 0, message);
			return cached;
		};
		settings.m_cascade_count = 3;
		settings.m_filter_size = 5;
		auto before = render_pair("Cached shadow views differ from rendered views in a static scene");
		api::Mat4x4 moved{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {8,0,0,1}};
		View3D_ObjectO2WSet(caster, moved, nullptr);
		auto first_after_move = fixture.Image();
		auto after = render_pair("Cached shadow views were not updated after a caster moved");
		Require(ImageDifference(before, after) > 50, "Moving the caster did not move its shadow");
		Require(ImageDifference(first_after_move, after) == 0, "The first frame after a caster moved used stale cached views");

		// Invalid cascade and filter settings are rejected
		auto invalid = settings;
		invalid.m_cascade_count = 5;
		View3D_ShadowSettingsSet(fixture.m_window, invalid);
		Require(!fixture.m_errors.empty(), "Cascade count above the maximum was accepted");
		fixture.m_errors.clear();
		invalid = settings;
		invalid.m_filter_size = 6;
		View3D_ShadowSettingsSet(fixture.m_window, invalid);
		Require(!fixture.m_errors.empty(), "Unsupported filter size was accepted");
		fixture.m_errors.clear();
		std::cout << "PASS directional cascades, filter sizes, shadow view caching, and cascade settings validation: MSAA " << samples << '\n';
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_Shadows, Extended)
	{
		render_tests::ShadowTests(1);
		render_tests::ShadowTests(4);
	}
	PRUnitTest(View3d12_DirectionalShadows, Extended)
	{
		render_tests::DirectionalShadowTests(1);
		render_tests::DirectionalShadowTests(4);
	}
}
