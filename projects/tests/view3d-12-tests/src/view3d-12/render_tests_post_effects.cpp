#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Verify that output dithering is off by default, adds bounded per-pixel noise that keeps the mean colour, and turns off cleanly.
	void DitherTests(int samples)
	{
		// Dithering is disabled by default so exact-colour output is unchanged for existing applications.
		std::cout << "Output dithering, MSAA " << samples << std::endl;
		Fixture fixture(samples);
		Require(View3D_DitherAmountGet(fixture.m_window) == 0.0f, "Dithering is not disabled by default");

		// An opaque grey exercises the forward output, and a translucent white exercises the K-buffer layer and alpha resolve.
		// The translucent case is dithered twice (layer packing and resolve), so it may move by more than one 8-bit step.
		struct Case { unsigned colour; int max_step; char const* name; };
		for (auto const& test : {Case{0xFF808080U, 1, "opaque"}, Case{0x80FFFFFFU, 3, "translucent"}})
		{
			// Render the same scene without and with dithering.
			fixture.Clear();
			fixture.Quad(50, test.colour);
			View3D_DitherAmountSet(fixture.m_window, 0.0f);
			auto baseline = fixture.Image();
			View3D_DitherAmountSet(fixture.m_window, 1.0f);
			Require(View3D_DitherAmountGet(fixture.m_window) == 1.0f, "Dither amount was not stored");
			auto dithered = fixture.Image();
			fixture.CheckErrors();

			// Compare the quad interior. Dithering must change a good share of pixels, keep each one near the
			// undithered value, and keep the average within one 8-bit rounding step of the undithered value.
			auto changed = 0;
			auto count = 0;
			auto sum = 0.0;
			for (auto y = 20; y != 108; ++y)
			{
				for (auto x = 20; x != 108; ++x)
				{
					// Check every colour channel of this pixel.
					for (auto c = 0; c != 3; ++c)
					{
						auto i = (y * ImageSize + x) * 4 + c;
						auto delta = int(dithered[i]) - int(baseline[i]);
						Require(std::abs(delta) <= test.max_step, (std::string("Dither noise too large: ") + test.name).c_str());
						changed += delta != 0 ? 1 : 0;
						sum += delta;
						++count;
					}
				}
			}
			Require(changed * 10 > count, (std::string("Dithering changed too few pixels: ") + test.name).c_str());
			Require(std::abs(sum / count) < 0.6, (std::string("Dithering shifted the mean colour: ") + test.name).c_str());

			// Turning dithering off restores the exact undithered image.
			View3D_DitherAmountSet(fixture.m_window, 0.0f);
			Require(baseline == fixture.Image(), (std::string("Disabling dithering did not restore the original pixels: ") + test.name).c_str());
		}
		fixture.CheckDebugLayer();
		std::cout << "PASS output dithering default, bounds, mean, and toggle: MSAA " << samples << std::endl;
	}

	// Verify the underwater post-effect settings API, its tint and depth fog, and its exclusion of UI overlays.
	void PostEffectTests(int samples)
	{
		// Defaults are disabled, and invalid settings are rejected without changing the current settings.
		std::cout << "Post-effect defaults and validation, MSAA " << samples << std::endl;
		Fixture fixture(samples);
		auto defaults = View3D_PostEffectUnderwaterGet(fixture.m_window);
		Require(!defaults.m_enabled && defaults.m_tint == 0xFFA6D9F2U && defaults.m_fog_colour == 0xFF0A384DU && defaults.m_visibility == 40.0f, "Underwater defaults differ");
		Require(defaults.m_distortion_amplitude == 0.002f && defaults.m_distortion_frequency == 6.0f && defaults.m_distortion_speed == 0.25f, "Underwater distortion defaults differ");
		auto invalid = api::UnderwaterProps{.m_enabled = TRUE, .m_visibility = 0.0f};
		Require(!View3D_PostEffectUnderwaterSet(fixture.m_window, invalid), "DLL accepted zero visibility");
		invalid = api::UnderwaterProps{.m_enabled = TRUE, .m_distortion_frequency = std::numeric_limits<float>::quiet_NaN()};
		Require(!View3D_PostEffectUnderwaterSet(fixture.m_window, invalid), "DLL accepted NaN frequency");
		invalid = api::UnderwaterProps{.m_enabled = TRUE, .m_distortion_amplitude = -1.0f};
		Require(!View3D_PostEffectUnderwaterSet(fixture.m_window, invalid), "DLL accepted negative amplitude");
		invalid = api::UnderwaterProps{.m_enabled = TRUE, .m_surface = api::Vec4{0, 0, 0, 1}};
		Require(!View3D_PostEffectUnderwaterSet(fixture.m_window, invalid), "DLL accepted a surface without a normal");
		Require(fixture.m_errors.size() == 4, "DLL did not report each invalid setting");
		fixture.m_errors.clear();
		Require(!View3D_PostEffectUnderwaterGet(fixture.m_window).m_enabled, "Failed setter mutated settings");

		// Magenta tint and green fog keep each term separately visible on a white surface.
		// Visibility is chosen so that fog is exactly 0.5 at a distance of 50.
		auto props = api::UnderwaterProps{
			.m_enabled = TRUE,
			.m_tint = 0xFFFF00FFU,
			.m_fog_colour = 0xFF00FF00U,
			.m_visibility = 150.0f / std::log(2.0f),
			.m_distortion_amplitude = 0.0f,
		};
		auto underwater = [&](bool enabled)
		{
			// Apply through the DLL API used by applications.
			props.m_enabled = enabled;
			Require(View3D_PostEffectUnderwaterSet(fixture.m_window, props) != FALSE, "Underwater settings rejected");
			fixture.CheckErrors();
		};
		auto fog = [&](float distance)
		{
			// Mirror the documented fog curve.
			return 1.0f - std::exp(-3.0f * distance / props.m_visibility);
		};

		// A disabled effect leaves the image unchanged, and disabling it again restores the original pixels.
		std::cout << "Tint, fog, and toggle" << std::endl;
		fixture.Quad(50, 0xFFFFFFFF);
		auto baseline = fixture.Image();
		Expect(baseline, 1,1,1);
		underwater(true);
		auto enabled = fixture.Image();
		Expect(enabled, 0.5f,0.5f,0.5f);
		Expect(enabled, 0,1,0, 2,2);
		underwater(false);
		Require(baseline == fixture.Image(), "Disabling the underwater effect did not restore the original pixels");

		// Fog grows with distance from the camera.
		std::cout << "Depth fog" << std::endl;
		underwater(true);
		for (auto depth : {10.0f, 90.0f})
		{
			fixture.Clear();
			fixture.Quad(depth, 0xFFFFFFFF);
			auto f = fog(depth);
			Expect(fixture.Image(), 1 - f, f, 1 - f);
		}

		// Perspective cameras use the straight-line distance, so off-axis pixels are more fogged than the centre.
		std::cout << "Perspective distance" << std::endl;
		fixture.Clear();
		fixture.Quad(50, 0xFFFFFFFF);
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{1.5707963268f, 1.5707963268f});
		auto perspective = fixture.Image();
		Expect(perspective, 0.5f,0.5f,0.5f);
		auto x = (100.5f / ImageSize * 2 - 1) * 50;
		auto f = fog(std::sqrt(50 * 50 + x * x));
		Expect(perspective, 1 - f, f, 1 - f, 100,64);
		View3D_CameraOrthographicSet(fixture.m_window, TRUE);
		View3D_CameraViewRectAtDistanceSet(fixture.m_window, api::Vec2{100,100}, 1.0f);

		// Distortion moves samples but cannot change the colour of a uniform region.
		std::cout << "Distortion" << std::endl;
		props.m_distortion_amplitude = 0.01f;
		underwater(true);
		Expect(fixture.Image(), 0.5f,0.5f,0.5f);
		props.m_distortion_amplitude = 0.0f;

		// With a surface plane, a horizontal surface through the middle of the orthographic near plane splits the view at the waterline.
		// Pixels above it keep the scene colour, and pixels below it are tinted and fully fogged.
		std::cout << "Surface plane" << std::endl;
		props.m_surface = api::Vec4{0, 1, 0, 0};
		underwater(true);
		auto split = fixture.Image();
		Expect(split, 1,1,1, 64,32);
		Expect(split, 0.5f,0.5f,0.5f, 64,96);
		Expect(split, 0,0,0, 2,2);
		Expect(split, 0,1,0, 2,125);

		// The distortion does not pull colour across the waterline.
		props.m_distortion_amplitude = 0.05f;
		underwater(true);
		split = fixture.Image();
		Expect(split, 1,1,1, 64,62);
		Expect(split, 0.5f,0.5f,0.5f, 64,66);
		props.m_distortion_amplitude = 0.0f;

		// With a fade depth, the effect strength rises smoothly with the depth of each pixel's near-plane point below the surface.
		// The orthographic near plane spans y in [-50, 50], so rows below the middle are at depth 100 * (row + 0.5) / ImageSize - 50.
		props.m_fade_depth = 50.0f;
		underwater(true);
		auto faded = fixture.Image();
		Expect(faded, 1,1,1, 64,32);
		for (auto row : {80, 96, 120})
		{
			auto t = std::clamp((100.0f * (row + 0.5f) / ImageSize - 50.0f) / props.m_fade_depth, 0.0f, 1.0f);
			auto strength = t * t * (3 - 2 * t);
			auto c = 1 - 0.5f * strength;
			Expect(faded, c,c,c, 64,row);
		}
		props.m_fade_depth = 0.0f;

		// Waterline offsets tilt the real surface: it rises 25 above the plane at the left edge of the near plane and falls 25 below it at the
		// right edge. Each column's waterline therefore sits at near-plane height 25 - 50 * u, with u the viewport-normalised column position.
		// The offsets move only the waterline, so pixels below it but above the plane are tinted without fog.
		for (auto j = 0; j != 8; ++j)
		{
			for (auto i = 0; i != 8; ++i)
				props.m_waterline_offsets[j * 8 + i] = 25.0f - 50.0f * i / 7.0f;
		}
		underwater(true);
		auto sloped = fixture.Image();
		Expect(sloped, 1,1,1, 16,32);
		Expect(sloped, 1,0,1, 16,50);
		Expect(sloped, 1,1,1, 112,70);
		Expect(sloped, 0.5f,0.5f,0.5f, 112,100);
		std::ranges::fill(props.m_waterline_offsets, 0.0f);

		// Infinite visibility disables the fog, leaving only the tint.
		props.m_visibility = std::numeric_limits<float>::infinity();
		underwater(true);
		Expect(fixture.Image(), 1,0,1, 64,96);
		props.m_visibility = 150.0f / std::log(2.0f);

		// With the whole near plane above the surface, the pass is skipped and the image is unchanged, even where rays
		// descend into the water.
		props.m_surface = api::Vec4{0, 0, 1, 30};
		underwater(true);
		auto above = fixture.Image();
		Expect(above, 1,1,1);
		Expect(above, 0,0,0, 2,2);

		// A camera in water whose view ray rises through the surface part way fogs only the submerged length.
		props.m_surface = api::Vec4{0, 0, -1, -30};
		underwater(true);
		f = fog(30);
		Expect(fixture.Image(), 1 - f, f, 1 - f);

		// From below the surface in perspective, rays that rise through it are fogged only up to the surface, and open sky stays visible.
		props.m_surface = api::Vec4{0, 1, 0, -10};
		underwater(true);
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{1.5707963268f, 1.5707963268f});
		auto looking_up = fixture.Image();
		Expect(looking_up, 0.5f,0.5f,0.5f);
		auto ndc_y = 1 - 2 * 16.5f / ImageSize;
		f = fog(10 / (ndc_y * 50) * std::sqrt(50 * 50 + ndc_y * 50 * ndc_y * 50));
		Expect(looking_up, 1 - f, f, 1 - f, 64,16);
		ndc_y = 1 - 2 * 2.5f / ImageSize;
		f = fog(10 * std::sqrt(1 + ndc_y * ndc_y) / ndc_y);
		Expect(looking_up, 0, f, 0, 64,2);
		View3D_CameraOrthographicSet(fixture.m_window, TRUE);
		View3D_CameraViewRectAtDistanceSet(fixture.m_window, api::Vec2{100,100}, 1.0f);
		props.m_surface = api::Vec4{};
		underwater(true);

		// Retained screen UI is drawn after post-processing, so the overlay marker keeps its colour.
		std::cout << "Overlay exclusion" << std::endl;
		fixture.AttachOverlay();
		auto overlay = fixture.Image();
		Expect(overlay, 0,1,0, 10,10);
		Expect(overlay, 0.5f,0.5f,0.5f);
		underwater(false);
		Expect(fixture.Image(), 1,1,1);

		// Any GPU validation error fails the fixture.
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS underwater post-effect: MSAA " << samples << '\n';
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_PostEffects, Extended)
	{
		render_tests::PostEffectTests(1);
		render_tests::PostEffectTests(4);
	}
	PRUnitTest(View3d12_Dither, Extended)
	{
		render_tests::DitherTests(1);
		render_tests::DitherTests(4);
	}
}
