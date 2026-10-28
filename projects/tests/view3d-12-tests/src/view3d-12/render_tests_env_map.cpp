#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Check that 'EnvMapCaptureExclude' objects are rejected by scenes that exclude capture content, and that a capture with them runs.
	void EnvMapCaptureExcludeTests()
	{
		// An unlit red quad fills the view against the black background.
		auto fixture = Fixture(1);
		auto object = fixture.Quad(10, 0xFFFF0000);
		Expect(fixture.Image(), 1, 0, 0);

		// The flag alone does not affect ordinary scenes.
		View3D_ObjectFlagsSet(object, api::ELdrFlags::EnvMapCaptureExclude, TRUE, nullptr);
		fixture.CheckErrors();
		Expect(fixture.Image(), 1, 0, 0);

		// A scene that excludes capture content, as the capture scene does, does not draw the flagged object.
		auto& scene = fixture.m_window->m_scene;
		scene.m_inst_exclude = pr::rdr12::EInstFlag::EnvMapCaptureExclude;
		Expect(fixture.Image(), 0, 0, 0);
		View3D_ObjectFlagsSet(object, api::ELdrFlags::EnvMapCaptureExclude, FALSE, nullptr);
		fixture.CheckErrors();
		Expect(fixture.Image(), 1, 0, 0);
		scene.m_inst_exclude = pr::rdr12::EInstFlag::None;

		// Capturing with a flagged object must complete without errors.
		View3D_ObjectFlagsSet(object, api::ELdrFlags::EnvMapCaptureExclude, TRUE, nullptr);
		auto cube = api::CubeMapPtr(View3D_CubeMapCreate(32));
		Require(cube != nullptr, "Cube map creation failed");
		View3D_WindowEnvMapCapture(fixture.m_window, cube.get(), api::Vec4{0, 0, 0, 1});
		fixture.CheckErrors();
		Expect(fixture.Image(), 1, 0, 0);

		// Any GPU validation error fails the fixture.
		fixture.CheckDebugLayer();
		std::cout << "PASS environment map capture exclusion\n";
	}

	// Check that the environment map probe completes a cube every six updates and fades each new cube in over the previous one
	void EnvMapProbeTests()
	{
		// An unlit red quad fills the view against the black background
		auto fixture = Fixture(1);
		fixture.Quad(10, 0xFFFF0000);
		auto& scene = fixture.m_window->m_scene;
		auto update = [&](int count)
		{
			// Each update renders one face of the capture cube
			for (int i = 0; i != count; ++i)
				View3D_WindowEnvMapProbeUpdate(fixture.m_window, api::Vec4{0, 0, 0, 1});

			fixture.CheckErrors();
		};

		// No environment map is bound until the first cube is complete
		View3D_WindowEnvMapProbeSet(fixture.m_window, 32, TRUE);
		fixture.CheckErrors();
		Require(scene.m_global_envmap == nullptr, "Probe bound an environment map before any capture");
		update(5);
		Require(scene.m_global_envmap == nullptr, "Probe bound an incomplete cube");

		// The first complete cube is used alone because there is nothing to fade from
		update(1);
		auto* first = scene.m_global_envmap.get();
		Require(first != nullptr && scene.m_global_envmap_prev == nullptr, "First complete cube not bound alone");
		Expect(fixture.Image(), 1, 0, 0);

		// The second complete cube starts fully weighted to the first, then fades in over the next five updates
		update(6);
		Require(scene.m_global_envmap.get() != first && scene.m_global_envmap_prev.get() == first, "Second cube did not replace the first");
		Require(scene.m_global_envmap_blend == 0.0f, "Fade did not start from the previous cube");
		update(1);
		Require(std::abs(scene.m_global_envmap_blend - 0.2f) < 1e-6f, "Fade did not advance by one step");
		Expect(fixture.Image(), 1, 0, 0);
		update(4);
		Require(scene.m_global_envmap_blend == 1.0f, "Fade did not finish within one cycle");

		// The third cube was captured into a spare cube, and the first cube, now the oldest, is unbound so that the next capture can reuse it
		update(1);
		Require(scene.m_global_envmap.get() != first && scene.m_global_envmap_prev.get() != first, "Probe did not retire the oldest cube");
		update(6);
		Require(scene.m_global_envmap.get() == first, "Probe did not reuse the oldest cube");

		// Parallax bounds make each new cube store distances in its distance cube, scaled by the bounds' half-diagonal. Rendering with the marching lookup must complete without errors.
		View3D_WindowEnvMapParallaxBoundsSet(fixture.m_window, pr::view3d::BBox{ .centre = {0, 0, 0, 1}, .radius = {3, 0, 4, 0} });
		fixture.CheckErrors();
		update(12);
		Require(scene.m_global_envmap->m_distance_scale == 5.0f && scene.m_global_envmap_prev->m_distance_scale == 5.0f, "Probe cubes did not record the distance scale");
		Require(scene.m_global_envmap->m_distance != nullptr && scene.m_global_envmap_prev->m_distance != nullptr, "Probe cubes did not create distance cubes");
		Expect(fixture.Image(), 1, 0, 0);

		// Setting an explicit environment map disables the probe and removes the fade, but keeps the probe's cubes
		auto* probe = fixture.m_window->m_envmap_probe.get();
		View3D_WindowEnvMapSet(fixture.m_window, nullptr);
		fixture.CheckErrors();
		Require(scene.m_global_envmap == nullptr && scene.m_global_envmap_prev == nullptr && !fixture.m_window->m_envmap_probe_enabled, "Environment map set did not disable the probe");
		Require(fixture.m_window->m_envmap_probe.get() == probe, "Environment map set released the probe's resources");

		// Sizing a disabled probe creates its resources but leaves the explicit environment map in place
		auto explicit_map = api::CubeMapPtr(View3D_CubeMapCreate(32));
		View3D_WindowEnvMapSet(fixture.m_window, explicit_map.get());
		View3D_WindowEnvMapProbeSet(fixture.m_window, 16, FALSE);
		fixture.CheckErrors();
		probe = fixture.m_window->m_envmap_probe.get();
		Require(scene.m_global_envmap.get() == explicit_map.get() && probe != nullptr && !fixture.m_window->m_envmap_probe_enabled, "Sizing a disabled probe changed the environment map");

		// Enabling the sized probe reuses its cubes and restarts the capture, so no map is bound until a new cube is complete
		View3D_WindowEnvMapProbeSet(fixture.m_window, 16, TRUE);
		fixture.CheckErrors();
		Require(scene.m_global_envmap == nullptr && fixture.m_window->m_envmap_probe.get() == probe, "Enabling the probe did not restart its capture");
		update(6);
		Require(scene.m_global_envmap != nullptr && scene.m_global_envmap_prev == nullptr, "Restarted probe did not complete a cube");

		// A face size of zero releases the probe
		View3D_WindowEnvMapProbeSet(fixture.m_window, 0, FALSE);
		fixture.CheckErrors();
		Require(scene.m_global_envmap == nullptr && fixture.m_window->m_envmap_probe == nullptr, "Probe was not released");

		// Any GPU validation error fails the fixture.
		fixture.CheckDebugLayer();
		std::cout << "PASS environment map probe\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_EnvMapCaptureExclude, Extended | Serial)
	{
		render_tests::EnvMapCaptureExcludeTests();
	}
	PRUnitTest(View3d12_EnvMapProbe, Extended | Serial)
	{
		render_tests::EnvMapProbeTests();
	}
}
