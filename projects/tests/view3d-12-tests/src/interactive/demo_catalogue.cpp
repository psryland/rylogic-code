//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include <array>
#include <cctype>
#include <stdexcept>
#include "demo.h"

namespace view3d_test
{
	// Demo factories, defined in the demos_*.cpp files
	std::unique_ptr<IDemo> CreateAudioBoxDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateUiGalleryDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateHitTestDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateShadowsDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateSkyboxDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateFarClipFadeDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateUnderwaterDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateDitherDemo(DemoContext const& ctx);
	std::unique_ptr<IDemo> CreateWaveGridDemo(DemoContext const& ctx);

	// All demos, in menu order
	static std::array<DemoInfo, 9> const s_catalogue =
	{{
		{L"Scene", "audio_box", L"Spatial Audio", L"A tone plays from the rotating box. O toggles occlusion. E/R/T/Space change the step mode.", &CreateAudioBoxDemo},
		{L"Scene", "ui_gallery", L"UI Gallery", L"The view3d UI controls. The slider changes the size of the box.", &CreateUiGalleryDemo},
		{L"Scene", "hit_test", L"Hit Testing", L"Shift+Left Click on an object to move the coordinate frame to the hit point.", &CreateHitTestDemo},
		{L"Lighting", "shadows", L"Shadows", L"Shadow casting from directional, point, and spot lights. Use the controls to change the light and shadow settings.", &CreateShadowsDemo},
		{L"Lighting", "skybox", L"Sky and Reflections", L"Cube map and procedural skies, with optional environment map reflections on the spheres.", &CreateSkyboxDemo},
		{L"Effects", "far_clip_fade", L"Far Clip Fade", L"Objects fade out as they approach the far clip plane. Zoom out to see the effect.", &CreateFarClipFadeDemo},
		{L"Effects", "underwater", L"Underwater", L"The underwater post effect, with a water surface plane at a controllable height.", &CreateUnderwaterDemo},
		{L"Effects", "dither", L"Dither", L"Dithering to hide colour banding in smooth gradients.", &CreateDitherDemo},
		{L"Procedural", "wave_grid", L"Wave Grid", L"A grid animated by a procedural vertex shader. Vertex positions are computed from the vertex index.", &CreateWaveGridDemo},
	}};

	// All demos, in menu order
	std::span<DemoInfo const> DemoCatalogue()
	{
		return s_catalogue;
	}

	// Find a demo by name, ignoring case and treating '-' and '_' as equal
	DemoInfo const* FindDemo(std::string_view name)
	{
		// Normalise one character so that name comparisons ignore case and separator style
		auto norm = [](char c)
		{
			return c == '-' ? '_' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		};

		// Linear search is fine for a handful of demos
		for (auto const& info : s_catalogue)
		{
			if (info.m_name.size() != name.size())
				continue;

			auto match = true;
			for (size_t i = 0; match && i != name.size(); ++i)
				match = norm(info.m_name[i]) == norm(name[i]);

			if (match)
				return &info;
		}
		return nullptr;
	}

	namespace demo
	{
		SceneObjects::SceneObjects(pr::view3d::Window window)
			: m_window(window)
			, m_objects()
		{
		}

		SceneObjects::~SceneObjects()
		{
			// Deleting an object also removes it from every window
			for (auto obj : m_objects)
				View3D_ObjectDelete(obj);
		}

		// Create an object from LDraw script and add it to the window
		pr::view3d::Object SceneObjects::Add(char const* ldr_script)
		{
			// View3D reports script errors through the error callback and returns null
			auto obj = View3D_ObjectCreateLdrA(ldr_script, FALSE, nullptr, nullptr);
			if (obj == nullptr)
				throw std::runtime_error(std::string("Failed to create object from script: ") + ldr_script);

			return Add(obj);
		}

		// Take ownership of 'object' and add it to the window
		pr::view3d::Object SceneObjects::Add(pr::view3d::Object object)
		{
			// Record ownership first so the object is deleted even if adding to the window fails
			m_objects.push_back(object);
			View3D_WindowAddObject(m_window, object);
			return object;
		}
	}
}

#if PR_UNITTESTS
#include <set>
#include "pr/common/unittests.h"
namespace view3d_test::unittests
{
	PRUnitTest(View3d12_DemoCatalogue, Quick)
	{
		// Every demo has a unique name, a factory, and can be found by its own name
		auto names = std::set<std::string_view>{};
		for (auto const& info : DemoCatalogue())
		{
			PR_EXPECT(info.m_create != nullptr);
			PR_EXPECT(names.insert(info.m_name).second);
			PR_EXPECT(FindDemo(info.m_name) == &info);
		}

		// Lookup ignores case and separator style, and rejects unknown or partial names
		PR_EXPECT(FindDemo("Far-Clip_FADE") == FindDemo("far_clip_fade"));
		PR_EXPECT(FindDemo("far_clip_fade") != nullptr);
		PR_EXPECT(FindDemo("far_clip") == nullptr);
		PR_EXPECT(FindDemo("") == nullptr);
	}
}
#endif
