//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#include "src/forward.h"
#include "pr/geometry/p3d.h"
#include "pr/geometry/3ds.h"
#include "pr/geometry/stl.h"
#include "pr/geometry/obj.h"
#include "pr/geometry/gltf.h"

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

namespace
{
	// Return the nearest legacy address mode for a glTF wrap mode, so that readers which only
	// understand the single address mode still behave sensibly.
	p3d::Texture::EAddrMode ToAddrMode(ETextureWrap wrap)
	{
		switch (wrap)
		{
			case ETextureWrap::Repeat:
			{
				return p3d::Texture::EAddrMode::Wrap;
			}
			case ETextureWrap::MirroredRepeat:
			{
				return p3d::Texture::EAddrMode::Mirror;
			}
			case ETextureWrap::ClampToEdge:
			{
				return p3d::Texture::EAddrMode::Clamp;
			}
			default:
			{
				throw std::runtime_error("Unknown texture wrap mode");
			}
		}
	}

	// Convert a glTF alpha mode to the p3d equivalent
	p3d::Material::EAlphaMode ToAlphaMode(gltf::EAlphaMode mode)
	{
		switch (mode)
		{
			case gltf::EAlphaMode::Opaque:
			{
				return p3d::Material::EAlphaMode::Opaque;
			}
			case gltf::EAlphaMode::Mask:
			{
				return p3d::Material::EAlphaMode::Mask;
			}
			case gltf::EAlphaMode::Blend:
			{
				return p3d::Material::EAlphaMode::Blend;
			}
			default:
			{
				throw std::runtime_error("Unknown glTF alpha mode");
			}
		}
	}

	// Return the file extension that matches an image mime type
	char const* ImageExtension(std::string_view mime_type)
	{
		if (mime_type == "image/png") return ".png";
		if (mime_type == "image/jpeg") return ".jpg";
		if (mime_type == "image/webp") return ".webp";
		if (mime_type == "image/ktx2") return ".ktx2";
		return ".bin";
	}

	// Builds a p3d file from the meshes and materials emitted by the glTF reader.
	struct GltfToP3d :gltf::IReadOutput
	{
		// P3D references textures by string id rather than by embedded bytes, so images are written
		// out beside the model and named by a hash of their content. Two models that share an image
		// then name the same file, which is what allows a packaging step to store it only once.
		p3d::File& m_p3d;
		std::filesystem::path m_texture_dir;
		std::unordered_map<uint32_t, size_t> m_mesh_index;
		std::unordered_map<uint32_t, p3d::Str16> m_mat_ids;

		GltfToP3d(p3d::File& out, std::filesystem::path texture_dir)
			:m_p3d(out)
			,m_texture_dir(std::move(texture_dir))
			,m_mesh_index()
			,m_mat_ids()
		{}

		// Return the p3d id for a glTF material, adding the material to the scene the first time it is seen.
		p3d::Str16 MaterialId(gltf::Material const& mat)
		{
			if (auto it = m_mat_ids.find(mat.m_mat_id); it != m_mat_ids.end())
				return it->second;

			// Str16 truncates, so a long or duplicated name has to fall back to something unique.
			auto id = p3d::Str16(mat.m_name);
			auto taken = [this, &id] { return pr::contains_if(m_mat_ids, [&id](auto const& kv) { return kv.second == id; }); };
			if (std::string_view(id).empty() || taken())
				id = p3d::Str16(std::format("mat{}", mat.m_mat_id));

			m_mat_ids[mat.m_mat_id] = id;
			m_p3d.m_scene.m_materials.emplace_back(std::string_view(id), mat.m_base_colour);

			auto& out = m_p3d.m_scene.m_materials.back();
			out.m_metallic = mat.m_metallic;
			out.m_roughness = mat.m_roughness;
			out.m_emissive = mat.m_emissive;
			out.m_alpha_mode = ToAlphaMode(mat.m_alpha_mode);
			out.m_alpha_cutoff = mat.m_alpha_cutoff;
			out.m_double_sided = mat.m_double_sided;

			AddTexture(out, mat.m_base_colour_texture, p3d::Texture::EType::Diffuse, mat.m_alpha_mode != gltf::EAlphaMode::Opaque);
			AddTexture(out, mat.m_metallic_roughness_texture, p3d::Texture::EType::MetallicRoughness, false);
			AddTexture(out, mat.m_emissive_texture, p3d::Texture::EType::Emissive, false);
			AddTexture(out, mat.m_normal_texture, p3d::Texture::EType::NormalMap, false);
			return id;
		}

		// Add one texture slot to 'mat', extracting the image if the glTF file embeds it.
		void AddTexture(p3d::Material& mat, geometry::TextureRef const& src, p3d::Texture::EType type, bool has_alpha)
		{
			if (!src)
				return;

			p3d::Texture tex;
			tex.m_filepath = src.m_data.empty() ? src.m_uri : ExtractImage(src);
			tex.m_type = type;
			tex.m_addr_mode = ToAddrMode(src.m_wrap_s);
			tex.m_flags = has_alpha ? p3d::Texture::EFlags::Alpha : p3d::Texture::EFlags::None;
			tex.m_wrap_s = src.m_wrap_s;
			tex.m_wrap_t = src.m_wrap_t;
			tex.m_min_filter = src.m_min_filter;
			tex.m_mag_filter = src.m_mag_filter;
			tex.m_texcoord = src.m_texcoord;
			tex.m_scale = src.m_scale;
			tex.m_uv_transform = src.m_uv_transform;
			mat.m_textures.push_back(std::move(tex));
		}

		// Write an embedded image to the texture directory and return the id that names it.
		std::string ExtractImage(geometry::TextureRef const& src)
		{
			auto hash = s_cast<uint64_t>(hash::HashBytes64(src.m_data.data(), src.m_data.data() + src.m_data.size()));
			auto name = std::format("{:016x}{}", hash, ImageExtension(src.m_mime_type));
			auto filepath = m_texture_dir / name;

			// The name is derived from the content, so an existing file is already the right image.
			if (!exists(filepath))
			{
				create_directories(m_texture_dir);
				std::ofstream out(filepath, std::ofstream::binary);
				out.write(reinterpret_cast<char const*>(src.m_data.data()), s_cast<std::streamsize>(src.m_data.size()));
			}
			return name;
		}

		// Convert one glTF mesh into a p3d mesh
		void CreateMesh(gltf::Mesh const& mesh, std::span<gltf::Material const> materials) override
		{
			m_mesh_index[mesh.m_mesh_id] = m_p3d.m_scene.m_meshes.size();
			m_p3d.m_scene.m_meshes.emplace_back(std::string(mesh.m_name));

			auto& out = m_p3d.m_scene.m_meshes.back();
			out.m_bbox = mesh.m_bbox;
			out.m_vert.reserve(mesh.m_vbuf.size());

			for (auto const& v : mesh.m_vbuf)
				out.add_vert(p3d::FatVert(v.m_vert, v.m_colr, v.m_norm, v.m_tex0));

			// P3D nuggets own their indices, so each nugget takes a copy of its slice of the shared
			// index buffer. The glTF indices are already absolute into the vertex buffer.
			auto vcount = mesh.m_vbuf.size();
			auto idx_stride = vcount > 0xFFFF ? sizeof(uint32_t) : sizeof(uint16_t);
			out.m_nugget.reserve(mesh.m_nbuf.size());
			for (auto const& n : mesh.m_nbuf)
			{
				auto mat_id = n.m_mat_id < materials.size() ? MaterialId(materials[n.m_mat_id]) : p3d::Str16();

				p3d::Nugget nug(n.m_topo, n.m_geom, std::string_view(mat_id));
				nug.m_vidx = p3d::IdxBuf(s_cast<int>(idx_stride));
				for (auto i = n.m_irange.m_beg; i != n.m_irange.m_end; ++i)
				{
					if (idx_stride == sizeof(uint32_t))
						nug.m_vidx.push_back<uint32_t>(s_cast<uint32_t>(mesh.m_ibuf[i]));
					else
						nug.m_vidx.push_back<uint16_t>(s_cast<uint16_t>(mesh.m_ibuf[i]));
				}
				out.m_nugget.emplace_back(std::move(nug));
			}
		}

		// Apply the node transforms from the scene graph to the meshes they instance.
		void CreateModel(std::span<gltf::MeshTree const> mesh_tree) override
		{
			// Node transforms are relative to the parent node, so accumulate down the hierarchy
			// using the level of each node to know which parent it belongs to.
			pr::vector<m4x4> parent = { m4x4::Identity() };
			for (auto const& node : mesh_tree)
			{
				auto level = s_cast<size_t>(node.m_level);
				parent.resize(level + 1, m4x4::Identity());

				auto n2r = parent[level] * node.m_o2p;
				parent.push_back(n2r);

				auto it = m_mesh_index.find(node.m_mesh_id);
				if (it != m_mesh_index.end())
					m_p3d.m_scene.m_meshes[it->second].m_o2p = n2r;
			}
		}
	};
}

// Populates the p3d data structures from a glTF or GLB file.
// Embedded images are written into 'texture_dir' and referenced by a content-derived id.
std::unique_ptr<p3d::File> CreateFromGLTF(std::filesystem::path const& filepath, std::filesystem::path const& texture_dir)
{
	p3d::File p3d;

	// Loading by path lets the reader resolve relative buffer and image URIs in .gltf files.
	// The default error handler throws, which is what the tool wants.
	auto path_str = filepath.string();
	gltf::Scene scene(path_str.c_str());

	// P3D stores geometry and materials only, so skeletons and animations are not requested.
	gltf::ReadOptions opts = {};
	opts.m_parts = gltf::EParts::Materials | gltf::EParts::Meshes | gltf::EParts::NodeHierarchy;

	GltfToP3d out(p3d, texture_dir);
	scene.Read(out, opts);

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
