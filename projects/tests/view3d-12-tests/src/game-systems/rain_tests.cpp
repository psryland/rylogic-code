//*********************************************
// Game Systems Rain Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Tests of the camera-centred rain through its View3D ABI: density follows the intensity, the field fills space everywhere, and invalid settings are rejected.
#include "../view3d-12/forward.h"
#include "../view3d-12/render_test_support.h"

namespace render_tests
{
	// Rain drawn around a perspective camera: none at zero intensity, more as the intensity rises, and the same density wherever and however fast the camera moves.
	void RainRenderTests()
	{
		// A perspective camera looking north (+Y) with Z up. Bright opaque drops make every drawn streak easy to count against the empty background.
		Fixture fixture(1);
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{ 1.5707963268f, 1.5707963268f });
		auto place_camera = [&](float x, float y, float z)
		{
			// Keep the same orientation, so only the position changes
			View3D_CameraToWorldSet(fixture.m_window, api::Mat4x4{ {1, 0, 0, 0}, {0, 0, 1, 0}, {0, -1, 0, 0}, {x, y, z, 1} });
		};
		place_camera(0, 0, 2);
		auto baseline = fixture.Image();

		auto settings = api::RainSettings{};
		settings.m_colour = api::Vec4{ 1, 1, 1, 1 };
		settings.m_opacity = 1.0f;
		settings.m_time = 100.0;
		auto rain = View3D_ObjectCreateRain("Rain", settings, nullptr);
		Require(rain != nullptr, "Rain creation failed");
		fixture.m_objects.push_back(rain);
		View3D_WindowAddObject(fixture.m_window, rain);
		fixture.CheckErrors();

		// Count the pixels clearly brighter than the same pixel of the empty scene.
		auto wet_pixels = [&]
		{
			// Compare the red channel, which the white drops raise
			auto image = fixture.Image();
			auto count = 0;
			for (size_t i = 0; i < image.size(); i += 4)
			{
				if (Linear(image[i]) > Linear(baseline[i]) + 0.05f)
					++count;
			}
			return count;
		};
		auto update = [&]
		{
			// Every update must be accepted
			Require(View3D_ObjectUpdateRain(rain, settings) == TRUE, "Rain update failed");
		};

		// Zero intensity draws nothing.
		update();
		Require(wet_pixels() == 0, "Rain at zero intensity drew drops");

		// Light rain draws some drops, and a downpour draws many more.
		settings.m_intensity = 0.2f;
		update();
		auto light = wet_pixels();
		settings.m_intensity = 1.0f;
		update();
		auto heavy = wet_pixels();
		Require(light > 0, "Light rain drew no drops");
		Require(heavy > 2 * light, std::format("A downpour drew {} pixels, light rain {}", heavy, light).c_str());

		// The rain fills all of space: far from the origin, the camera sees about the same density.
		place_camera(12345.6f, -9876.5f, 300.0f);
		auto distant = wet_pixels();
		Require(distant > heavy / 2 && distant < heavy * 2, std::format("Far from the origin the downpour drew {} pixels, at the origin {}", distant, heavy).c_str());

		// A fast camera sees streaks of the same rain rather than an empty volume.
		settings.m_camera_velocity = api::Vec4{ 0, 150, 0, 0 };
		update();
		auto fast = wet_pixels();
		Require(fast > heavy / 2, std::format("A fast camera saw {} pixels of a downpour that drew {}", fast, heavy).c_str());

		// Time moves the drops: the image changes, but the density does not.
		settings.m_camera_velocity = api::Vec4{};
		settings.m_time += 0.25;
		update();
		auto later = wet_pixels();
		Require(later > distant / 2 && later < distant * 2, std::format("The downpour drew {} pixels after falling, {} before", later, distant).c_str());

		// A clip plane above the camera hides every drop below it, and a zero plane hides none.
		settings.m_clip_plane = api::Vec4{ 0, 0, 1, -1000.0f };
		update();
		Require(wet_pixels() == 0, "Rain below the clip plane was drawn");
		settings.m_clip_plane = api::Vec4{};
		update();
		Require(wet_pixels() == later, "Clearing the clip plane did not restore the rain");
		fixture.CheckErrors();

		// Invalid settings and non-rain objects are rejected with an error, leaving the rain unchanged.
		auto quad = fixture.Quad(50, 0xFFFFFFFF);
		auto expect_rejected = [&](BOOL result, char const* message)
		{
			// Each rejected call must return FALSE and report why.
			Require(result == FALSE && !fixture.m_errors.empty(), message);
			fixture.m_errors.clear();
		};
		auto bad = settings;
		bad.m_intensity = 1.5f;
		expect_rejected(View3D_ObjectUpdateRain(rain, bad), "Rain with an intensity above 1 was accepted");
		bad = settings;
		bad.m_fall_velocity = api::Vec4{};
		expect_rejected(View3D_ObjectUpdateRain(rain, bad), "Rain without a fall velocity was accepted");
		bad = settings;
		bad.m_time = std::numeric_limits<double>::quiet_NaN();
		expect_rejected(View3D_ObjectUpdateRain(rain, bad), "Rain at a non-finite time was accepted");
		bad = settings;
		bad.m_clip_plane = api::Vec4{ 0, 0, 1, std::numeric_limits<float>::infinity() };
		expect_rejected(View3D_ObjectUpdateRain(rain, bad), "Rain with a non-finite clip plane was accepted");
		bad = settings;
		bad.m_streak_time = 0.0f;
		expect_rejected(View3D_ObjectCreateRain("BadRain", bad, nullptr) == nullptr ? FALSE : TRUE, "Rain with no streak time was created");
		expect_rejected(View3D_ObjectUpdateRain(quad, settings), "A rain update of a non-rain object was accepted");
		Require(wet_pixels() == later, "A rejected update changed the rain");
		fixture.CheckDebugLayer();
	}
}

namespace pr::unittests::game_systems
{
	PRUnitTest(GameSystems_RainRender, Extended | Serial)
	{
		render_tests::RainRenderTests();
	}
}
