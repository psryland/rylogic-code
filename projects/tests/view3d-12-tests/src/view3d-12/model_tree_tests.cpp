#include "forward.h"
#include "render_test_support.h"

namespace render_tests
{
	// Write a P3D file containing two root meshes with the given names.
	void WriteTwoMeshP3D(std::filesystem::path const& filepath, std::string_view name0, std::string_view name1)
	{
		// Each mesh is one triangle with a single nugget, which is enough to produce a model tree node.
		using namespace pr::geometry::p3d;
		File file;
		file.m_version = Version;
		file.m_scene.m_materials.push_back(Material{ "mat", pr::ColourWhite });
		for (auto name : { name0, name1 })
		{
			Mesh mesh{ std::string(name) };
			mesh.m_bbox = pr::BBox{ pr::v4::Origin(), pr::v4{1, 1, 0, 0} };
			mesh.m_vert.assign({ pr::v4{-1, -1, 0, 1}, pr::v4{+1, -1, 0, 1}, pr::v4{0, +1, 0, 1} });

			Nugget nug{ pr::geometry::ETopo::TriList, pr::geometry::EGeom::Vert, "mat", sizeof(uint16_t) };
			nug.m_vidx.append<uint16_t>({ 0, 1, 2 });
			mesh.m_nugget.emplace_back(std::move(nug));
			file.m_scene.m_meshes.push_back(std::move(mesh));
		}

		std::ofstream out(filepath, std::ofstream::binary);
		Write(out, file, EFlags::Default);
		Require(out.good(), "Failed to write the P3D test model");
	}

	// Model tree node names must outlive the model loader's temporary source data.
	void ModelTreeNameTests()
	{
		// Load a two-root P3D model, which creates a child object per mesh named from the loader's data.
		Fixture fixture(1);
		auto filepath = std::filesystem::temp_directory_path() / std::format("view3d12-model-tree-names-{}.p3d", GetCurrentProcessId());
		WriteTwoMeshP3D(filepath, "first-part", "second-part-with-a-longer-name");
		auto script = std::format("*Model model {{ *FilePath {{\"{}\"}} }}", filepath.generic_string());
		auto model = View3D_ObjectCreateLdrA(script.c_str(), FALSE, nullptr, nullptr);
		std::filesystem::remove(filepath);
		Require(model != nullptr, "Model failed to load");
		fixture.m_objects.push_back(model);
		fixture.CheckErrors();

		// The child names are read after loading finishes, so they must be owned by the objects.
		Require(View3D_ObjectChildCount(model) == 2, "Expected one child object per root mesh");
		std::string_view expected[] = { "first-part", "second-part-with-a-longer-name" };
		for (int i = 0; i != 2; ++i)
		{
			auto name = View3D_ObjectNameGet(View3D_ObjectGetChildByIndex(model, i));
			Require(std::string_view(name.ptr, name.size) == expected[i], "Model tree child name was not preserved");
		}
		std::cout << "PASS model tree names\n";
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_ModelTreeNames, Extended | Serial)
	{
		render_tests::ModelTreeNameTests();
	}
}
