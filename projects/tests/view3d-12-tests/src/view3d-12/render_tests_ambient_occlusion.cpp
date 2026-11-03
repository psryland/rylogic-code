#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Verify the ambient occlusion settings API, darkening beside a raised strip, no darkening of flat surfaces, the background, or
	// transparent surfaces, and exact restoration when it is disabled.
	void AmbientOcclusionTests(int samples)
	{
		// Defaults are disabled, and invalid settings are rejected without changing the current settings.
		std::cout << "Ambient occlusion defaults and validation, MSAA " << samples << std::endl;
		Fixture fixture(samples);
		auto defaults = View3D_AmbientOcclusionGet(fixture.m_window);
		Require(!defaults.m_enabled && defaults.m_radius == 0.5f && defaults.m_intensity == 1.0f && defaults.m_falloff == 0.6f, "Ambient occlusion defaults differ");
		Require(defaults.m_max_screen_radius == 0.1f && defaults.m_quality == api::EAOQuality::Medium && defaults.m_resolution == api::EAOResolution::Half && !defaults.m_debug_view, "Ambient occlusion sampling defaults differ");
		auto invalid = api::AmbientOcclusionProps{.m_enabled = TRUE, .m_radius = 0.0f};
		Require(!View3D_AmbientOcclusionSet(fixture.m_window, invalid), "DLL accepted a zero radius");
		invalid = api::AmbientOcclusionProps{.m_enabled = TRUE, .m_falloff = 1.5f};
		Require(!View3D_AmbientOcclusionSet(fixture.m_window, invalid), "DLL accepted a falloff above one");
		invalid = api::AmbientOcclusionProps{.m_enabled = TRUE, .m_intensity = std::numeric_limits<float>::quiet_NaN()};
		Require(!View3D_AmbientOcclusionSet(fixture.m_window, invalid), "DLL accepted a NaN intensity");
		invalid = api::AmbientOcclusionProps{.m_enabled = TRUE, .m_quality = static_cast<api::EAOQuality>(7)};
		Require(!View3D_AmbientOcclusionSet(fixture.m_window, invalid), "DLL accepted an unknown quality");
		Require(fixture.m_errors.size() == 4, "DLL did not report each invalid setting");
		fixture.m_errors.clear();
		Require(!View3D_AmbientOcclusionGet(fixture.m_window).m_enabled, "Failed setter mutated settings");

		// The orthographic view is 100 units across 128 pixels. The strip is 10 units above the floor, so occluders are at least 10 units away.
		// A radius of 20 units, with the falloff starting at 15, and no screen limit lets the strip occlude the floor within about 25 pixels of its edges.
		auto props = api::AmbientOcclusionProps{
			.m_enabled = TRUE,
			.m_radius = 20.0f,
			.m_intensity = 1.0f,
			.m_falloff = 0.25f,
			.m_max_screen_radius = 1.0f,
		};
		auto ambient_occlusion = [&](bool enabled)
		{
			// Apply through the DLL API used by applications.
			props.m_enabled = enabled;
			Require(View3D_AmbientOcclusionSet(fixture.m_window, props) != FALSE, "Ambient occlusion settings rejected");
			fixture.CheckErrors();
		};

		// Column brightness, averaged over the middle rows to smooth the noise pattern. The strip covers x in [-20, 20], so its right edge
		// is at pixel 89.6. Pixels 91 and 92 are on the floor about 2 units beside the strip, pixel 116 is on the floor out of range of the strip (the floor's
		// edge is at pixel 121.6), and pixel 84 is on top of the strip near its edge.
		auto column = [](std::vector<unsigned char> const& image, int x)
		{
			return RegionRed(image, x, x + 2, 40, 88);
		};

		// A flat surface facing the camera has nothing to occlude it, so it and the background are unchanged.
		std::cout << "Flat surface" << std::endl;
		fixture.Quad(50, 0xFFFFFFFF);
		ambient_occlusion(true);
		auto flat = fixture.Image();
		Expect(flat, 1,1,1);
		Expect(flat, 1,1,1, 12,64);
		Expect(flat, 0,0,0, 2,2);

		// The floor beside the raised strip is darkened, while the floor out of range and the top of the strip are not.
		// Every quality and resolution must give the same qualitative result.
		std::cout << "Raised strip" << std::endl;
		fixture.Clear();
		fixture.Quad(60, 0xFFFFFFFF);
		fixture.Quad(50, 0xFFFFFFFF, 20);
		ambient_occlusion(false);
		auto baseline = fixture.Image();
		for (auto resolution : {api::EAOResolution::Half, api::EAOResolution::Full})
		{
			for (auto quality : {api::EAOQuality::Low, api::EAOQuality::Medium, api::EAOQuality::High})
			{
				// Compare the three regions of the strip scene.
				props.m_resolution = resolution;
				props.m_quality = quality;
				ambient_occlusion(true);
				auto occluded = fixture.Image();
				auto beside = column(occluded, 91);
				auto distant = column(occluded, 116);
				auto top = column(occluded, 84);
				std::cout << "  resolution " << int(resolution) << " quality " << int(quality) << ": beside " << beside << ", distant " << distant << ", top " << top << std::endl;
				Require(beside < 0.85f, "The floor beside the strip is not darkened");
				Require(distant > 0.97f, "The floor out of range of the strip is darkened");
				Require(top > 0.97f, "The top of the strip is darkened");
				Expect(occluded, 0,0,0, 2,2);
			}
		}
		props.m_resolution = api::EAOResolution::Half;
		props.m_quality = api::EAOQuality::Medium;

		// More intensity darkens more, and zero intensity leaves the image unchanged.
		std::cout << "Intensity" << std::endl;
		ambient_occlusion(true);
		auto normal = column(fixture.Image(), 91);
		props.m_intensity = 3.0f;
		ambient_occlusion(true);
		Require(column(fixture.Image(), 91) < normal - 0.05f, "Higher intensity did not darken more");
		props.m_intensity = 0.0f;
		ambient_occlusion(true);
		Require(ImageDifference(baseline, fixture.Image()) == 0, "Zero intensity changed the image");
		props.m_intensity = 1.0f;

		// The debug view shows the occlusion term as grey, so the floor beside the strip is grey even where the scene is not lit.
		std::cout << "Debug view" << std::endl;
		props.m_debug_view = TRUE;
		ambient_occlusion(true);
		auto debug = fixture.Image();
		Require(column(debug, 91) < 0.85f && column(debug, 116) > 0.97f, "Debug view does not show the occlusion term");
		props.m_debug_view = FALSE;

		// Disabling the effect restores the exact original pixels.
		ambient_occlusion(false);
		Require(ImageDifference(baseline, fixture.Image()) == 0, "Disabling ambient occlusion did not restore the original pixels");

		// A perspective camera must reconstruct the same surfaces: the floor beside the strip is darkened and the flat floor is not.
		// The strip's right edge (x = 20, depth 50) and the floor it hides (x = 24, depth 60) both project to pixel 64 + 64 * 20 / (50 * tan(0.75)) = 91.5.
		// The floor at pixel 92 is about 5 units from the strip's edge, at pixel 112 about 22 units, and the floor's own edges are at pixels 13 and 115.
		std::cout << "Perspective" << std::endl;
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{1.5f, 1.5f});
		{
			// A flat floor seen at up to about 40 degrees off-axis is not darkened anywhere.
			fixture.Clear();
			fixture.Quad(60, 0xFFFFFFFF);
			ambient_occlusion(true);
			auto flat_floor = fixture.Image();
			Require(column(flat_floor, 16) > 0.99f && column(flat_floor, 64) > 0.99f && column(flat_floor, 110) > 0.99f, "Perspective: a flat floor seen off-axis is darkened");
		}
		{
			// Compare the floor beside the strip and out of its range. The strip is half the height of the gap to its edge, so the darkening is weaker than in the orthographic case.
			fixture.Quad(50, 0xFFFFFFFF, 20);
			ambient_occlusion(true);
			auto perspective = fixture.Image();
			auto beside = column(perspective, 92);
			auto distant = column(perspective, 112);
			std::cout << "  beside " << beside << ", distant " << distant << std::endl;
			Require(beside < 0.9f, "Perspective: the floor beside the strip is not darkened");
			Require(distant > 0.97f, "Perspective: the floor out of range of the strip is darkened");
		}
		View3D_CameraOrthographicSet(fixture.m_window, TRUE);
		View3D_CameraViewRectAtDistanceSet(fixture.m_window, api::Vec2{100,100}, 1.0f);
		fixture.Clear();
		fixture.Quad(60, 0xFFFFFFFF);
		fixture.Quad(50, 0xFFFFFFFF, 20);

		// A translucent pane in front of the scene is composited after the occlusion is applied. The floor seen through it is still darkened
		// beside the strip, and the pane itself does not darken the flat floor behind it.
		std::cout << "Transparent pane" << std::endl;
		fixture.Quad(30, 0x80FFFFFF);
		ambient_occlusion(false);
		auto glass = fixture.Image();
		ambient_occlusion(true);
		auto glass_occluded = fixture.Image();
		Require(column(glass_occluded, 91) < column(glass, 91) - 0.05f, "The floor behind the pane is not darkened beside the strip");
		Require(std::abs(column(glass_occluded, 116) - column(glass, 116)) < 0.02f, "The pane changed the flat floor behind it");
		Require(std::abs(column(glass_occluded, 84) - column(glass, 84)) < 0.02f, "The pane changed the top of the strip behind it");

		// Any GPU validation error fails the fixture.
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
		std::cout << "PASS ambient occlusion: MSAA " << samples << '\n';
	}

	// Measure the frame cost of ambient occlusion at 4K with 4x MSAA. Each frame is rendered and then waited for, so the GPU work of
	// consecutive frames does not overlap and the difference in frame time between AO on and off is an upper bound on the AO GPU cost.
	// Batches alternate between on and off so that GPU clock changes affect both equally, and medians reject occasional slow frames.
	void AmbientOcclusionCost()
	{
		// A 4K, 4x MSAA target with a perspective view of a dense box field, so every pixel has depth and many depth edges.
		std::cout << "Ambient occlusion cost at 3840x2160, MSAA 4" << std::endl;
		Fixture fixture(4);
		View3D_WindowBackBufferSizeSet(fixture.m_window, SIZE{3840, 2160}, FALSE);
		View3D_WindowViewportSet(fixture.m_window, api::Viewport{0, 0, 3840, 2160, 0, 1, 3840, 2160});
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{1.6f, 0.9f});
		View3D_CameraClipPlanesSet(fixture.m_window, 0.1f, 200, api::EClipPlanes::Both);
		View3D_CameraPositionSet(fixture.m_window, api::Vec4{0, -30, 12, 1}, api::Vec4{0, 0, 0, 1}, api::Vec4{0, 0, 1, 0});
		{
			// One object holds all of the boxes. Each item is 'width height depth x y z'.
			auto script = std::string("*BoxList field FF80A0C0 { *Data { ");
			for (auto j = 0; j != 40; ++j)
			{
				for (auto i = 0; i != 40; ++i)
				{
					// Vary the box heights so the depth buffer has many edges at different depths
					auto h = 0.5f + 0.5f * ((i * 7 + j * 3) % 5);
					script += std::format("0.8 0.8 {} {} {} {} ", h, (i - 19.5f) * 1.5f, (j - 19.5f) * 1.5f, h * 0.5f);
				}
			}
			script += "} }";
			auto field = View3D_ObjectCreateLdrA(script.c_str(), FALSE, nullptr, nullptr);
			auto floor = View3D_ObjectCreateLdrA("*Plane floor FFC0C0C0 { *Data {200 200} *AxisId {+3} }", FALSE, nullptr, nullptr);
			Require(field != nullptr && floor != nullptr, "Failed to create the cost scene");
			fixture.m_objects.push_back(field);
			fixture.m_objects.push_back(floor);
			View3D_WindowAddObject(fixture.m_window, field);
			View3D_WindowAddObject(fixture.m_window, floor);
		}
		fixture.CheckErrors();

		// Render 'count' frames, waiting for each one, and return the frame times in milliseconds
		auto frame_times = [&](int count)
		{
			// Time the CPU recording plus the GPU execution of each frame
			auto times = std::vector<double>{};
			for (auto i = 0; i != count; ++i)
			{
				auto t0 = std::chrono::steady_clock::now();
				View3D_WindowRender(fixture.m_window);
				View3D_WindowGSyncWait(fixture.m_window);
				auto t1 = std::chrono::steady_clock::now();
				times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
			}
			return times;
		};
		auto median = [](std::vector<double> times)
		{
			std::ranges::sort(times);
			return times[times.size() / 2];
		};

		// Measure each quality and resolution against AO off
		for (auto resolution : {api::EAOResolution::Half, api::EAOResolution::Full})
		{
			for (auto quality : {api::EAOQuality::Low, api::EAOQuality::Medium, api::EAOQuality::High})
			{
				// Warm up, creating the AO resources, then alternate batches
				auto props = api::AmbientOcclusionProps{.m_enabled = TRUE, .m_quality = quality, .m_resolution = resolution};
				Require(View3D_AmbientOcclusionSet(fixture.m_window, props) != FALSE, "Ambient occlusion settings rejected");
				frame_times(20);
				auto off = std::vector<double>{};
				auto on = std::vector<double>{};
				for (auto round = 0; round != 10; ++round)
				{
					// One batch with AO off, one with it on
					props.m_enabled = FALSE;
					View3D_AmbientOcclusionSet(fixture.m_window, props);
					std::ranges::copy(frame_times(20), std::back_inserter(off));
					props.m_enabled = TRUE;
					View3D_AmbientOcclusionSet(fixture.m_window, props);
					std::ranges::copy(frame_times(20), std::back_inserter(on));
				}
				auto cost = median(on) - median(off);
				std::cout << std::format("  resolution {} quality {}: frame {:.3f} ms off, {:.3f} ms on, AO cost {:.3f} ms", int(resolution), int(quality), median(off), median(on), cost) << std::endl;

				// The default settings must meet the 1 ms budget
				if (resolution == api::EAOResolution::Half && quality == api::EAOQuality::Medium)
					Require(cost < 1.0, "Ambient occlusion at the default settings costs 1 ms or more at 4K with 4x MSAA");
			}
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_AmbientOcclusion, Extended | Serial)
	{
		render_tests::AmbientOcclusionTests(1);
		render_tests::AmbientOcclusionTests(4);
	}
	PRUnitTest(View3d12_AmbientOcclusionCost, Stress | Serial)
	{
		render_tests::AmbientOcclusionCost();
	}
}
