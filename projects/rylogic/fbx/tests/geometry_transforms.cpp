// Standalone reader regression: compile this translation unit with ufbx.c and the FBX project's include paths and defines.
#include <iostream>
#include <sstream>
#include "../src/fbx.cpp"

namespace
{
	// Retain emitted hierarchy transforms without borrowing names from the reader.
	struct ModelOutput :IReadOutput
	{
		std::vector<m4x4> m_transforms;
		std::vector<int> m_levels;

		// Capture all mesh instances for comparison with ufbx geometry-space transforms.
		void CreateModel(std::span<MeshTree const> tree) override
		{
			for (auto const& node : tree)
			{
				m_transforms.push_back(node.m_o2p);
				m_levels.push_back(node.m_level);
			}
		}
	};

	// Fail in both Debug and Release rather than depending on assert configuration.
	void Require(bool condition, char const* message)
	{
		if (!condition)
			throw std::runtime_error(message);
	}

	// Exercise shared geometry, nested mesh transforms, and a node without a geometry transform.
	void TestGeometryTransforms()
	{
		char const* source = R"fbx(
; FBX 7.4.0 project file
FBXHeaderExtension: {
	FBXHeaderVersion: 1003
	FBXVersion: 7400
}
Objects: {
	Geometry: 1, "Geometry::Triangle", "Mesh" {
		Vertices: *9 { a: 0,0,0,1,0,0,0,1,0 }
		PolygonVertexIndex: *3 { a: 0,1,-3 }
	}
	Model: 2, "Model::Parent", "Mesh" {
		Properties70: {
			P: "Lcl Translation", "Lcl Translation", "", "A", 10,20,30
			P: "GeometricTranslation", "Vector3D", "Vector", "", 2,3,4
			P: "GeometricRotation", "Vector3D", "Vector", "", 0,0,35
			P: "GeometricScaling", "Vector3D", "Vector", "", 2,3,4
		}
	}
	Model: 3, "Model::Child", "Mesh" {
		Properties70: {
			P: "Lcl Translation", "Lcl Translation", "", "A", 5,6,7
			P: "GeometricTranslation", "Vector3D", "Vector", "", -4,2,1
			P: "GeometricRotation", "Vector3D", "Vector", "", 20,0,0
		}
	}
	Model: 4, "Model::Grandchild", "Mesh" {
		Properties70: {
			P: "Lcl Translation", "Lcl Translation", "", "A", 1,2,3
		}
	}
}
Connections: {
	C: "OO",1,2
	C: "OO",1,3
	C: "OO",1,4
	C: "OO",2,0
	C: "OO",3,2
	C: "OO",4,3
}
)fbx";

		// Preserve geometric transforms so the test exercises the reader rather than ufbx helper-node conversion.
		ufbx_error error = {};
		auto scene = std::unique_ptr<ufbx_scene, decltype(&ufbx_free_scene)>(ufbx_load_memory(source, std::strlen(source), nullptr, &error), &ufbx_free_scene);
		if (!scene)
			throw std::runtime_error(To<std::string>(error));

		auto parent = NullCheck(ufbx_find_node(scene.get(), "Parent"), "Missing parent fixture");
		auto child = NullCheck(ufbx_find_node(scene.get(), "Child"), "Missing child fixture");
		auto grandchild = NullCheck(ufbx_find_node(scene.get(), "Grandchild"), "Missing grandchild fixture");
		Require(parent->has_geometry_transform && child->has_geometry_transform && !grandchild->has_geometry_transform, "Fixture lost its geometry transforms");
		ModelOutput output;
		ReadOptions options = {};
		Reader reader(*scene, options, output);
		reader.ReadGeometry();
		Require(output.m_transforms.size() == 3 && output.m_levels == std::vector<int>{0, 1, 2}, "Wrong emitted hierarchy");

		// The root includes its geometry offset, but no ancestor geometry offset may leak into a descendant.
		auto root_to_world = output.m_transforms[0];
		auto child_to_world = root_to_world * output.m_transforms[1];
		auto grandchild_to_world = child_to_world * output.m_transforms[2];
		Require(FEqlAbsolute(root_to_world, To<m4x4>(parent->geometry_to_world), 1e-4f), "Root geometry transform omitted");
		Require(FEqlAbsolute(child_to_world, To<m4x4>(child->geometry_to_world), 1e-4f), "Parent geometry transform leaked into child");
		Require(FEqlAbsolute(grandchild_to_world, To<m4x4>(grandchild->geometry_to_world), 1e-4f), "Child geometry transform leaked into grandchild");

		// Skinning uses the same vertex space as static geometry, including non-uniform geometric scale.
		auto bone_to_mesh = To<m4x4>(reader.EvaluateBoneToMesh(*scene->anim, grandchild, parent, 0.0));
		auto expected = Invert(To<m4x4>(parent->geometry_to_world)) * To<m4x4>(grandchild->node_to_world);
		Require(FEqlAbsolute(bone_to_mesh, expected, 1e-4f), "Animated bone is not in mesh geometry space");

		// Identity geometry transforms retain the existing node-space path.
		auto bone_to_plain_mesh = To<m4x4>(reader.EvaluateBoneToMesh(*scene->anim, child, grandchild, 0.0));
		auto expected_plain = Invert(To<m4x4>(grandchild->node_to_world)) * To<m4x4>(child->node_to_world);
		Require(FEqlAbsolute(bone_to_plain_mesh, expected_plain, 1e-4f), "Identity geometry transform changed animation space");
	}
}

// Run the isolated importer regression without requiring the renderer or a GUI process.
int main()
{
	try
	{
		TestGeometryTransforms();
		std::cout << "FBX geometry-transform regressions passed\n";
		return 0;
	}
	catch (std::exception const& ex)
	{
		std::cerr << ex.what() << '\n';
		return 1;
	}
}
