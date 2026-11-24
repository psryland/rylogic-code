//*********************************************
// Game Systems Lightning Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Tests of the procedural fork lightning: the shape and timing of a bolt, and drawing strikes through the procedural sky's View3D ABI.
#include "../view3d-12/forward.h"
#include "../view3d-12/render_test_support.h"
#include "pr/game-systems/sky/lightning.h"
#include "pr/game-systems/sky/procedural_sky.h"

namespace render_tests
{
	using namespace pr;
	using namespace pr::game_systems;

	// The cloud base used by the bolt shape tests, and the ground point the bolts strike.
	constexpr float TestCloudBase = 1500.0f;
	static v4 const TestGround = v4(100.0f, -200.0f, 20.0f, 1.0f);

	// The same seed must always grow the same bolt, and different seeds different bolts.
	void LightningBoltDeterminismTests()
	{
		// Compare every node of two bolts grown from the same inputs.
		auto a = LightningBolt(TestGround, TestCloudBase, 1234);
		auto b = LightningBolt(TestGround, TestCloudBase, 1234);
		Require(a.m_nodes.size() == b.m_nodes.size(), "The same seed grew bolts with different node counts");
		for (size_t i = 0; i != a.m_nodes.size(); ++i)
		{
			// Positions, timing and tree links must match exactly.
			auto const& na = a.m_nodes[i];
			auto const& nb = b.m_nodes[i];
			Require(All(na.m_pos == nb.m_pos) && na.m_arrival == nb.m_arrival && na.m_parent == nb.m_parent && na.m_brightness == nb.m_brightness, "The same seed grew different nodes");
		}
		Require(a.m_leader_time == b.m_leader_time && a.m_stroke_count == b.m_stroke_count && a.m_duration == b.m_duration, "The same seed chose different timing");

		// A different seed grows a different channel.
		auto c = LightningBolt(TestGround, TestCloudBase, 4321);
		Require(c.m_nodes.size() != a.m_nodes.size() || Any(c.m_origin != a.m_origin), "Different seeds grew the same bolt");
	}

	// Every bolt is a tree from its origin in the cloud to the ground, grown by a leader that reaches the ground at the leader time.
	void LightningBoltShapeTests()
	{
		for (uint32_t seed = 0; seed != 64; ++seed)
		{
			// The origin is the root, inside the cloud above the cloud base.
			auto bolt = LightningBolt(TestGround, TestCloudBase, seed);
			Require(!bolt.m_nodes.empty() && bolt.m_nodes[0].m_parent == -1 && bolt.m_nodes[0].m_arrival == 0, "The bolt's first node is not the origin");
			Require(bolt.m_origin.z > TestCloudBase && bolt.m_exit.z == TestCloudBase && All(bolt.m_ground == TestGround), "The bolt's key points are misplaced");

			// Every node grows from an earlier node, is reached no sooner than its parent, and lies between the ground and the top of the cloud.
			auto ground_index = -1;
			for (int i = 1; i != isize(bolt.m_nodes); ++i)
			{
				auto const& node = bolt.m_nodes[i];
				Require(node.m_parent >= 0 && node.m_parent < i, "A node grows from a later node");
				Require(node.m_arrival >= bolt.m_nodes[node.m_parent].m_arrival, "A node is reached before its parent");
				Require(node.m_pos.z >= TestGround.z - 1e-3f && node.m_pos.z <= TestCloudBase + 800.0f + 1e-3f, "A node is below the ground or above the cloud");
				Require(node.m_brightness > 0 && node.m_brightness <= 1 && node.m_width > 0, "A node has an invalid brightness or width");
				Require(!node.m_in_cloud || node.m_pos.z >= TestCloudBase - 1e-3f, "A node inside the cloud is below the cloud base");
				if (All(node.m_pos == bolt.m_ground))
					ground_index = i;
			}

			// The main channel ends at the ground, which the leader reaches at the leader time.
			Require(ground_index > 0, "The bolt does not reach the ground");
			Require(std::abs(bolt.m_nodes[ground_index].m_arrival - bolt.m_leader_time) <= 1e-4f * bolt.m_leader_time + 1e-6f, "The leader does not reach the ground at the leader time");
			Require(bolt.m_stroke_count >= 1 && bolt.m_stroke_count <= LightningBolt::StrokesMax, "The bolt has an invalid number of return strokes");
			Require(bolt.m_stroke_times[0] == bolt.m_leader_time && bolt.m_duration > bolt.m_stroke_times[bolt.m_stroke_count - 1], "The return strokes are mistimed");

			// The channel forks into many faint tendrils, so it has many more tips than the main channel's one, all dimmer than the main channel.
			std::vector<int> children(bolt.m_nodes.size());
			for (int i = 1; i != isize(bolt.m_nodes); ++i)
				++children[bolt.m_nodes[i].m_parent];

			auto tips = 0;
			for (int i = 1; i != isize(bolt.m_nodes); ++i)
			{
				// Every tip other than the ground end of the main channel belongs to a dimmer tendril or cloud branch.
				if (children[i] != 0)
					continue;

				++tips;
				Require(i == ground_index || bolt.m_nodes[i].m_brightness < 0.5f, "A channel tip is as bright as the main channel");
			}
			Require(tips >= 20, std::format("The bolt has only {} channel tips", tips).c_str());
		}
	}

	// The strike's light goes through the leader, the dazzling return stroke, and the fade, and is dark outside the strike.
	void LightningBoltLightTests()
	{
		for (uint32_t seed = 0; seed != 16; ++seed)
		{
			// Nothing is visible before the strike or after it ends.
			auto bolt = LightningBolt(TestGround, TestCloudBase, seed);
			Require(bolt.LightAt(-0.01f).m_glow == 0 && bolt.LightAt(bolt.m_duration).m_glow == 0, "The bolt is visible outside its strike");

			// The leader is faint, flagged, and does not dazzle or light the ground.
			auto leader = bolt.LightAt(0.5f * bolt.m_leader_time);
			Require(leader.m_leader && leader.m_glow > 0 && leader.m_glow < 0.5f && leader.m_dazzle == 0 && leader.m_flash == 0, "The leader has the wrong light");

			// The first return stroke is at full brightness, fully dazzles, and fully lights the ground.
			auto stroke = bolt.LightAt(bolt.m_leader_time);
			Require(!stroke.m_leader && stroke.m_glow >= 1.0f - 1e-4f && stroke.m_dazzle == 1.0f && stroke.m_flash >= 1.0f - 1e-4f && stroke.m_fade == 0, "The first return stroke has the wrong light");

			// The first stroke holds the bolt at full brightness and width before it fades. Later strokes start no earlier than the hold's end.
			auto held = bolt.LightAt(bolt.m_leader_time + 0.9f * LightningBolt::StrokeHold);
			auto released = bolt.LightAt(bolt.m_leader_time + LightningBolt::StrokeHold + 0.02f);
			Require(held.m_glow >= 1.0f - 1e-4f && held.m_flash >= 1.0f - 1e-4f && held.m_fade == 0 && released.m_fade > 0, "The first return stroke does not hold before fading");

			// The dazzle passes long before the end of the strike, while the fading channel is still visible.
			auto late = bolt.LightAt(bolt.m_duration - 0.1f);
			Require(late.m_glow > 0 && late.m_dazzle < 0.05f && late.m_fade > 0 && late.m_fade < 1, "The end of the strike has the wrong light");
		}
	}

	// Strikes drawn by the procedural sky: visible at the return stroke, gone after the strike, and rejected for invalid ground points.
	void LightningRenderTests()
	{
		// A perspective camera at the ground looking north (+Y) with Z up. The strike is 3 km north, so its channel below the 1500 m cloud base
		// fills the upper middle of the image. Zero exposure leaves the cloudless sky black, so any light comes from the bolt.
		Fixture fixture(1);
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{ 1.5707963268f, 1.5707963268f });
		View3D_CameraToWorldSet(fixture.m_window, api::Mat4x4{ {1, 0, 0, 0}, {0, 0, 1, 0}, {0, -1, 0, 0}, {0, 0, 10, 1} });
		auto settings = api::ProceduralSkySettings{};
		settings.m_exposure = 0.0f;
		settings.m_lightning_time = 10.0;
		auto sky = View3D_ObjectCreateProceduralSky("LightningSky", settings, nullptr);
		Require(sky != nullptr, "Lightning sky creation failed");
		fixture.m_objects.push_back(sky);
		View3D_WindowAddObject(fixture.m_window, sky);
		fixture.CheckErrors();

		// Count the pixels clearly brighter than the same pixel of 'baseline'.
		auto lit_pixels = [](std::vector<unsigned char> const& image, std::vector<unsigned char> const& baseline)
		{
			// Compare the red channel, which the icy white bolt fills strongly.
			auto count = 0;
			for (size_t i = 0; i < image.size(); i += 4)
			{
				if (Linear(image[i]) > Linear(baseline[i]) + 0.2f)
					++count;
			}
			return count;
		};

		// The sky without a strike, and no dazzle.
		auto baseline = fixture.Image();
		auto dazzle = -1.0f;
		Require(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 10.1, 1.0f, &dazzle) && dazzle == 0, "A sky with no strikes dazzles");

		// A strike starts at the sky's lightning time. Its first return stroke begins within 90 ms, so the whole channel is lit 100 ms later.
		Require(View3D_ObjectProceduralSkyStrike(sky, api::Vec4{ 0, 3000, 0, 1 }, 7) == TRUE, "A valid strike was rejected");
		settings.m_lightning_time = 10.1;
		Require(View3D_ObjectUpdateProceduralSky(sky, settings) == TRUE, "Sky update failed");
		auto lit = lit_pixels(fixture.Image(), baseline);
		Require(lit >= 20, std::format("The return stroke lit only {} pixels", lit).c_str());
		Require(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 10.1, 0.25f, &dazzle) && dazzle > 0.9f, "The return stroke does not dazzle");
		Require(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 9.9, 0.25f, &dazzle) && dazzle == 0, "The strike dazzles before it starts");

		// Looking south, the whole bolt is far outside the view, so it dazzles only the away fraction as much.
		auto full = dazzle;
		Require(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 10.1, 1.0f, &full) && full > 0.9f, "The return stroke does not dazzle");
		View3D_CameraToWorldSet(fixture.m_window, api::Mat4x4{ {-1, 0, 0, 0}, {0, 0, 1, 0}, {0, 1, 0, 0}, {0, 0, 10, 1} });
		Require(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 10.1, 0.25f, &dazzle) && std::abs(dazzle - 0.25f * full) < 1e-4f,
			std::format("A strike behind the camera dazzles {} of {}", dazzle, full).c_str());
		View3D_CameraToWorldSet(fixture.m_window, api::Mat4x4{ {1, 0, 0, 0}, {0, 0, 1, 0}, {0, -1, 0, 0}, {0, 0, 10, 1} });

		// The return strokes light the ground at the strike point. The leader does not, and neither does a strike that has not started.
		api::Vec4 ground_lights[ProceduralSky::StrikesMax] = {};
		auto count = View3D_ObjectProceduralSkyGroundLights(sky, 10.1, ground_lights, ProceduralSky::StrikesMax);
		Require(count == 1 && ground_lights[0].w > 0 && ground_lights[0].w <= 1, std::format("The return stroke has {} ground lights", count).c_str());
		Require(std::abs(ground_lights[0].x) < 0.01f && std::abs(ground_lights[0].y - 3000) < 0.01f && std::abs(ground_lights[0].z) < 0.01f, "The ground light is not at the strike point");
		Require(View3D_ObjectProceduralSkyGroundLights(sky, 10.0, ground_lights, ProceduralSky::StrikesMax) == 0, "The leader lights the ground");
		Require(View3D_ObjectProceduralSkyGroundLights(sky, 9.9, ground_lights, ProceduralSky::StrikesMax) == 0, "The strike lights the ground before it starts");

		// After the strike ends, nothing of it is drawn and it no longer dazzles.
		settings.m_lightning_time = 15.0;
		Require(View3D_ObjectUpdateProceduralSky(sky, settings) == TRUE, "Sky update failed");
		lit = lit_pixels(fixture.Image(), baseline);
		Require(lit == 0, std::format("An ended strike still lit {} pixels", lit).c_str());
		Require(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 15.0, 1.0f, &dazzle) && dazzle == 0, "An ended strike still dazzles");
		Require(View3D_ObjectProceduralSkyGroundLights(sky, 15.0, ground_lights, ProceduralSky::StrikesMax) == 0, "An ended strike still lights the ground");
		fixture.CheckErrors();

		// A strike that replaces a drawn strike must not release a model that the window still draws. Fill every slot with visible strikes,
		// draw them, then strike again before the next draw, which replaces one of the drawn strikes.
		settings.m_lightning_time = 20.0;
		Require(View3D_ObjectUpdateProceduralSky(sky, settings) == TRUE, "Sky update failed");
		for (int i = 0; i != ProceduralSky::StrikesMax; ++i)
			Require(View3D_ObjectProceduralSkyStrike(sky, api::Vec4{ 200.0f * i - 300.0f, 3000, 0, 1 }, 100 + i) == TRUE, "A valid strike was rejected");

		settings.m_lightning_time = 20.1;
		Require(View3D_ObjectUpdateProceduralSky(sky, settings) == TRUE, "Sky update failed");
		lit = lit_pixels(fixture.Image(), baseline);
		Require(lit >= 20, std::format("The full set of strikes lit only {} pixels", lit).c_str());
		Require(View3D_ObjectProceduralSkyStrike(sky, api::Vec4{ 0, 3000, 0, 1 }, 200) == TRUE, "A strike replacing a drawn strike was rejected");
		lit = lit_pixels(fixture.Image(), baseline);
		Require(lit >= 20, std::format("The strikes after a replacement lit only {} pixels", lit).c_str());
		fixture.CheckErrors();

		// A ground point above the cloud base, a non-finite point, a non-sky object, and a missing output are rejected with an error.
		auto quad = fixture.Quad(50, 0xFFFFFFFF);
		auto expect_rejected = [&](BOOL result, char const* message)
		{
			// Each rejected call must return FALSE and report why.
			Require(result == FALSE && !fixture.m_errors.empty(), message);
			fixture.m_errors.clear();
		};
		expect_rejected(View3D_ObjectProceduralSkyStrike(sky, api::Vec4{ 0, 3000, 2000, 1 }, 7), "A strike above the cloud base was accepted");
		expect_rejected(View3D_ObjectProceduralSkyStrike(sky, api::Vec4{ std::numeric_limits<float>::quiet_NaN(), 0, 0, 1 }, 7), "A non-finite strike was accepted");
		expect_rejected(View3D_ObjectProceduralSkyStrike(quad, api::Vec4{ 0, 3000, 0, 1 }, 7), "A strike on a non-sky object was accepted");
		expect_rejected(View3D_ObjectProceduralSkyDazzle(quad, fixture.m_window, 10.0, 1.0f, &dazzle), "Dazzle of a non-sky object was accepted");
		expect_rejected(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 10.0, 1.0f, nullptr), "Dazzle without an output was accepted");
		expect_rejected(View3D_ObjectProceduralSkyDazzle(sky, fixture.m_window, 10.0, 1.5f, &dazzle), "A dazzle away scale above 1 was accepted");
		expect_rejected(View3D_ObjectProceduralSkyGroundLights(quad, 10.0, ground_lights, ProceduralSky::StrikesMax) == -1 ? FALSE : TRUE, "Ground lights of a non-sky object were accepted");
		expect_rejected(View3D_ObjectProceduralSkyGroundLights(sky, 10.0, nullptr, 1) == -1 ? FALSE : TRUE, "Ground lights without an output were accepted");
		fixture.CheckDebugLayer();
	}
}

namespace pr::unittests::game_systems
{
	PRUnitTest(GameSystems_LightningBolt, Quick)
	{
		render_tests::LightningBoltDeterminismTests();
		render_tests::LightningBoltShapeTests();
		render_tests::LightningBoltLightTests();
	}
	PRUnitTest(GameSystems_LightningRender, Extended | Serial)
	{
		render_tests::LightningRenderTests();
	}
}
