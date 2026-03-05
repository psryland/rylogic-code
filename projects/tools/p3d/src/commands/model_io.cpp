//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#include "src/forward.h"
#include "pr/geometry/p3d.h"
#include "pr/geometry/3ds.h"
#include "pr/geometry/stl.h"
#include "pr/geometry/obj.h"

using namespace pr;
using namespace pr::script;
using namespace pr::geometry;

static_assert(std::is_move_constructible_v<p3d::File>);

// Populate the p3d data structures from a p3d file
std::unique_ptr<p3d::File> CreateFromP3D(std::filesystem::path const& filepath)
{
	std::ifstream src(filepath, std::ifstream::binary);
	auto p3d = p3d::Read(src);
	return std::unique_ptr<p3d::File>(new p3d::File(std::move(p3d)));
}

// Populates the p3d data structures from a 3ds file
std::unique_ptr<p3d::File> CreateFrom3DS(std::filesystem::path const& filepath)
{
	// Open the 3ds file
	std::ifstream src(filepath, std::ifstream::binary);

	// Material lookup
	std::unordered_map<std::string, max_3ds::Material> mats;
	auto matlookup = [&](std::string_view name) { return mats.at(std::string(name)); };

	// Read the materials from the 3ds file and add them to a map.
	// We'll only add the materials that are actually used to the p3d scene.
	max_3ds::ReadMaterials(src, [&](max_3ds::Material const& m)
	{
		mats[m.m_name] = m;
		return false;
	});

	// Read the tri mesh objects from the 3ds file
	p3d::File p3d;
	max_3ds::ReadObjects(src, [&](max_3ds::Object const& o)
	{
		// If the object has no verts or faces, then ignore
		if (o.m_mesh.m_vert.empty() || o.m_mesh.m_face.empty())
			return false;

		// Add a mesh to the scene
		p3d.m_scene.m_meshes.emplace_back(o.m_name);
		auto& mesh = p3d.m_scene.m_meshes.back();

		// Reserve space
		mesh.m_vert.reserve(o.m_mesh.m_vert.size());
		mesh.m_diff.reserve(o.m_mesh.m_vert.size());
		mesh.m_norm.reserve(o.m_mesh.m_vert.size());
		mesh.m_tex0.reserve(o.m_mesh.m_vert.size());
		mesh.m_nugget.reserve(o.m_mesh.m_matgroup.size());

		// Bounding box / transform
		mesh.m_bbox = BBox::Reset();
		mesh.m_o2p = o.m_mesh.m_o2p;
		auto bb = [&](v4 v) { Grow(mesh.m_bbox, v); return v; };

		p3d::IdxBuf vidx(sizeof(uint16_t));
		auto mesh_geom = EGeom::Vert;

		// Get the 3ds code to extract the verts/faces/normals/nuggets
		max_3ds::CreateModel(o, matlookup,
			[&](v4 p, Colour const& c, v4 n, v2 const& t) // vertex out
			{
				mesh.add_vert({p, c, n, t});
			},
			[&](uint16_t i0, uint16_t i1, uint16_t i2) // index out
			{
				vidx.push_back<uint16_t>(i0);
				vidx.push_back<uint16_t>(i1);
				vidx.push_back<uint16_t>(i2);
			},
			[&](ETopo topo, EGeom geom, max_3ds::Material const& mat, Range<size_t>, Range<size_t>) // nugget out
			{
				mesh_geom |= geom;

				p3d::Nugget nug(topo, geom, mat.m_name);
				nug.m_vidx = std::move(vidx);
				mesh.m_nugget.emplace_back(std::move(nug));

				vidx = p3d::IdxBuf{sizeof(uint16_t)};
			});

		// Add the used materials to the p3d scene
		for (auto& nug : mesh.m_nugget)
		{
			if (pr::contains_if(p3d.m_scene.m_materials, [&](p3d::Material const& m) { return m.m_id == nug.m_mat; }))
				continue;

			// Add the material
			auto const& mat_3ds = matlookup(nug.m_mat);
			p3d.m_scene.m_materials.emplace_back(mat_3ds.m_name, mat_3ds.m_diffuse);
			auto& mat = p3d.m_scene.m_materials.back();

			// Add the texture filepaths to the material
			for (auto& tex : mat_3ds.m_textures)
			{
				// todo: translate 3ds tiling flags to p3d
				mat.m_textures.emplace_back(tex.m_filepath);
			}
		}

		// Don't stop, gimme more objects
		return false;
	});
	return std::make_unique<p3d::File>(std::move(p3d));
}

// Populates the p3d data structures from a STL file
std::unique_ptr<p3d::File> CreateFromSTL(std::filesystem::path const& filepath)
{
	std::ifstream src(filepath, std::ifstream::binary);
	stl::Options opts = {};

	p3d::File p3d;
	stl::Read(src, opts, [&](stl::Model const& o)
	{
		// Add a mesh to the scene
		p3d.m_scene.m_meshes.emplace_back(o.m_header);
		auto& mesh = p3d.m_scene.m_meshes.back();
		auto vcount = int(o.m_verts.size());

		// Bounding box
		mesh.m_bbox = BBox::Reset();
		auto bb = [&](v4 v) { Grow(mesh.m_bbox, v); return v; };

		// Copy the verts
		mesh.m_vert.reserve(vcount);
		mesh.m_norm.reserve(vcount);
		for (int i = 0; i != vcount; ++i)
		{
			mesh.m_vert.push_back(bb(o.m_verts[i]));
			mesh.m_norm.push_back(o.m_norms[i / 3]);
		}

		// Generate a single nugget
		p3d::Nugget nug = {};
		nug.m_topo = ETopo::TriList;
		nug.m_geom = EGeom::Vert | EGeom::Norm;
		nug.m_mat = "default";

		// Generate the indices
		nug.m_vidx.resize(vcount, vcount > 0xFFFF ? sizeof(uint32_t) : sizeof(uint16_t));
		auto ibuf = nug.m_vidx.begin<int>();
		for (int i = 0; vcount-- != 0;)
			*ibuf++ = i++;

		mesh.m_nugget.emplace_back(std::move(nug));

		// Generate a material
		p3d::Material mat("default", ColourWhite);
		p3d.m_scene.m_materials.push_back(mat);
	});
	return std::make_unique<p3d::File>(std::move(p3d));
}

// Populates the p3d data structures from an obj file
std::unique_ptr<p3d::File> CreateFromOBJ(std::filesystem::path const& filepath)
{
	std::ifstream src(filepath);
	obj::Options opts = {};

	p3d::File p3d;
	obj::Read(src, opts, [&](obj::Model const& o)
		{
			(void)o;
		});
	return std::make_unique<p3d::File>(std::move(p3d));
}

// Write 'p3d' as a p3d format file
void WriteP3d(std::unique_ptr<p3d::File> const& p3d, std::filesystem::path const& outfile, pr::geometry::p3d::EFlags flags)
{
	std::ofstream ofile(outfile, std::ofstream::binary);
	p3d::Write(ofile, *p3d, flags);
}

// Write 'p3d' as a cpp source file
void WriteCpp(std::unique_ptr<p3d::File> const& p3d, std::filesystem::path const& outfile, std::string indent)
{
	std::ofstream ofile(outfile);
	p3d::WriteAsCode(ofile, *p3d, indent.c_str());
}

// Write 'p3d' as ldr script
void WriteLdr(std::unique_ptr<p3d::File> const& p3d, std::filesystem::path const& outfile, std::string indent)
{
	std::ofstream ofile(outfile);
	p3d::WriteAsScript(ofile, *p3d, indent.c_str());
}
