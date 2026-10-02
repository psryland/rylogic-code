//************************************************************************
//
// A Machine for turning X File files into Geometry Objects
//
//************************************************************************

#include "pr/common/min_max_fix.h"
#include <d3d9.h>
#include <dxfile.h>
#include "pr/common/assert.h"
#include "pr/common/predicate.h"
#include "pr/common/guid.h"
#include "pr/common/fmt.h"
#include "pr/storage/xfile/prxfiletemplates.h"
#include "pr/storage/xfile/xloader.h"
#include "pr/storage/xfile/xfile.h"
#include "pr/storage/xfile/xfileinternal.h"

using namespace pr;
using namespace pr::xfile;
using namespace pr::xfile::impl;

// Constructor
XLoader::XLoader(const void* custom_templates, unsigned int custom_templates_size)
:m_d3d_xfile(0)
{
	// Create an X file interface
	Verify(D3DXFileCreate(&m_d3d_xfile.m_ptr));

	// Register the direct x templates
	Verify(m_d3d_xfile->RegisterTemplates((void*)D3DTemplates, D3DTemplateBytes));

	// Register custum x file templates if there are any
	// Warning: this will fail if the null terminator is included in the custom templates
	// It should be const char templates[] = "..."; unsigned int templates_size = sizeof(templates) - 1;
	if (custom_templates_size)
		Verify(m_d3d_xfile->RegisterTemplates((void*)custom_templates, custom_templates_size));

	// Register more custum x file templates if there are any
#	pragma warning (disable : 4127) // conditional expression is constant
	if (CustomTemplateGUIDArray_Count)
		Verify(m_d3d_xfile->RegisterTemplates((void*)CustomTemplates, CustomTemplates_Bytes));
#	pragma warning (default : 4127) // conditional expression is constant
}

// The function that does the work
EResult XLoader::Load(const char *xfilepath, Geometry& geometry, const TGUIDSet* partial_load_set)
{
	// Reset the internal buffers
	m_xfilepath = xfilepath;
	m_vertex.clear();
	m_normal.clear();
	m_colour.clear();
	m_tex_coord.clear();
	m_material.clear();
	m_face.clear();

	// Copy the filename
	geometry.m_name = xfilepath;

	// Get a pointer to the partial load set
	m_partial_load_set = partial_load_set;

	// Create the enum object
	D3DPtr<ID3DXFileEnumObject> enum_object;
	if (Failed(m_d3d_xfile->CreateEnumObject(xfilepath, D3DXF_FILELOAD_FROMFILE, &enum_object.m_ptr)))
		return EResult_EnumerateFileFailed;
	
	try
	{
		// Enumerate the top level objects
		SIZE_T num_children = 0;
		Verify(enum_object->GetChildren(&num_children));
		for (std::size_t c = 0; c != num_children; ++c)
		{
			// Get the object
			D3DPtr<ID3DXFileData> child = 0;
			if (Failed(enum_object->GetChild(c, &child.m_ptr))) { throw xfile::Exception(EResult_GetChildFailed); }

			// Get the type of object
			GUID guid = GetGUID(child);
			if (!IsInLoadSet(guid)) { continue; }

			if (IsEqualGUID(guid, TID_D3DRMFrame))
			{
				LoadFrame(child, geometry);
			}
			else
			{
				PR_INFO(PR_DBG_XFILE, Fmt("Ignoring top level template '%s'\n", GUIDToString(guid).c_str()).c_str());
			}
		}
	}
	catch (pr::xfile::Exception const& e)
	{
		return e.code();
	}
	return EResult_Success;
}

// Returns true if a guid should be loaded
bool XLoader::IsInLoadSet(GUID guid) const
{
	if (!m_partial_load_set) return true;
	return m_partial_load_set->find(guid) != m_partial_load_set->end();
}

// Load a frame from the x file
void XLoader::LoadFrame(D3DPtr<ID3DXFileData> data, Geometry& geometry)
{
	geometry.m_frame.push_back(Frame());
	Frame& frame = geometry.m_frame.back();
	frame.m_name = GetName(data);
	frame.m_transform.identity();
	frame.m_mesh.m_geom_type = geom::EInvalid;

	// Enumerate the child objects
	std::size_t num_children = GetNumChildren(data);
	for (std::size_t c = 0; c != num_children; ++c)
	{
		// Get the object
		D3DPtr<ID3DXFileData> child = 0;
		if (Failed(data->GetChild(c, &child.m_ptr))) { throw xfile::Exception(EResult_GetChildFailed); }

		// Get the type of object
		GUID guid = GetGUID(child);
		if (!IsInLoadSet(guid)) { continue; }

		if (IsEqualGUID(guid, TID_D3DRMFrameTransformMatrix))
		{
			LoadFrameTransform(child, frame.m_transform);
		}
		else if (IsEqualGUID(guid, TID_D3DRMMesh))
		{
			LoadMesh(child, frame.m_mesh);
		}
		else
		{
			PR_INFO(PR_DBG_XFILE, Fmt("Ignoring frame level template '%s'\n", GUIDToString(guid).c_str()).c_str());
		}
	}
}

// Load the frame transformation matrix
void XLoader::LoadFrameTransform(D3DPtr<ID3DXFileData> data, m4x4& transform)
{
	transform.identity();

	XData xdata(data);
	if (xdata.m_size != sizeof(m4x4)) { throw xfile::Exception(EResult_DataSizeInvalid); }

	for (int j = 0; j < 4; ++j)
	{
		for(int i = 0; i < 4; ++i )
		{
			transform[j][i] = xdata.m_ptr.m_float[j * 4 + i];
		}
	}
}

// Read a mesh from the xfile
void XLoader::LoadMesh(D3DPtr<ID3DXFileData> data, Mesh& mesh)
{
	XData xdata(data);

	uint num_vertices = *xdata.m_ptr.m_uint++;
	m_vertex.clear();
	m_vertex.resize(num_vertices);
	for (uint v = 0; v < num_vertices; ++v)
	{
		v4& vertex = m_vertex[v];
		vertex[0] = *xdata.m_ptr.m_float++;
		vertex[1] = *xdata.m_ptr.m_float++;
		vertex[2] = *xdata.m_ptr.m_float++;
		vertex[3] = 1.0f;
	}
	if (num_vertices > 0) mesh.m_geom_type |= geom::EVertex;

	uint num_faces = *xdata.m_ptr.m_uint++;
	m_face.clear();
	m_face.resize(num_faces);
	for (uint f = 0; f < num_faces; ++f)
	{
		uint num_indices = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, 3);
		for (uint i = 0; i < num_indices; ++i)
		{
			uint vert_index           = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, (uint)m_vertex.size() - 1);
			m_face[f].m_vert_index[i] = vert_index;
			m_face[f].m_tex_index[i]  = vert_index;
		}
	}

	// Enumerate the child objects
	std::size_t num_children = GetNumChildren(data);
	for (std::size_t c = 0; c != num_children; ++c)
	{
		// Get the object
		D3DPtr<ID3DXFileData> child = 0;
		if (Failed(data->GetChild(c, &child.m_ptr))) { throw xfile::Exception(EResult_GetChildFailed); }

		// Get the type of object
		GUID guid = GetGUID(child);
		if (!IsInLoadSet(guid)) { continue; }

		if (IsEqualGUID(guid, TID_D3DRMMeshNormals))
		{
			LoadMeshNormal(child);
			if (!m_normal.empty()) mesh.m_geom_type |= geom::ENormal;
		}
		else if (IsEqualGUID(guid, TID_D3DRMMeshMaterialList))
		{
			LoadMeshMaterial(child);
		}
		else if (IsEqualGUID(guid, TID_D3DRMMeshVertexColors))
		{
			LoadMeshVertexColours(child);
			if (!m_colour.empty()) mesh.m_geom_type |= geom::EColour;
		}
		else if (IsEqualGUID(guid, TID_D3DRMMeshTextureCoords))
		{
			LoadMeshTexCoords(child);
			if (!m_tex_coord.empty()) mesh.m_geom_type |= geom::ETexture;
		}
		else
		{
			PR_INFO(PR_DBG_XFILE, Fmt("Ignoring mesh level template '%s'\n", GUIDToString(guid).c_str()).c_str());
		}
	}

	CompleteMesh(mesh);
}

// Fill in a meshes vertex normals
void XLoader::LoadMeshNormal(D3DPtr<ID3DXFileData> data)
{
	XData xdata(data);

	uint num_normals = *xdata.m_ptr.m_uint++;
	m_normal.clear();
	m_normal.resize(num_normals);
	for (uint n = 0; n < num_normals; ++n)
	{
		v4& norm = m_normal[n];		
		norm[0] = *xdata.m_ptr.m_float++;
		norm[1] = *xdata.m_ptr.m_float++;
		norm[2] = *xdata.m_ptr.m_float++;
		norm[3] = 0.0f;
	}

	uint num_faces = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, (uint)m_face.size());
	for (uint f = 0; f < num_faces; ++f)
	{
		uint num_indices = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, 3);
		for (uint i = 0; i < num_indices; ++i)
		{
			uint norm_index = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, (uint)m_normal.size() - 1);
			m_face[f].m_norm_index[i] = norm_index;
		}
	}
}

// Fill in a meshes materials
void XLoader::LoadMeshMaterial(D3DPtr<ID3DXFileData> data)
{
	XData xdata(data);

	uint num_materials = *xdata.m_ptr.m_uint++;
	m_material.clear();
	m_material.resize(num_materials);

	uint num_faces = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, (uint)m_face.size());
	for (uint f = 0; f < num_faces; ++f)
	{
		uint mat_index		  = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, num_materials - 1);
		m_face[f].m_mat_index = mat_index;
	}

	// Enumerate the child objects
	uint material_index = 0;
	std::size_t num_children = GetNumChildren(data);
	for (std::size_t c = 0; c != num_children; ++c)
	{
		// Get the object
		D3DPtr<ID3DXFileData> child = 0;
		if (Failed(data->GetChild(c, &child.m_ptr))) { throw xfile::Exception(EResult_GetChildFailed); }

		// Get the type of object
		GUID guid = GetGUID(child);
		if (!IsInLoadSet(guid)) { continue; }

		if (IsEqualGUID(guid, TID_D3DRMMaterial))
		{
			if (material_index == m_material.size()) { continue; }
			LoadMaterial(child, m_material[material_index]);
			++material_index;
		}
		else
		{
			PR_INFO(PR_DBG_XFILE, Fmt("Ignoring mesh material level template '%s'\n", GUIDToString(guid).c_str()).c_str());
		}
	}
}

// Read a material
void XLoader::LoadMaterial(D3DPtr<ID3DXFileData> data, Material& material)
{
	XData xdata(data);

	material.m_ambient.r  = *xdata.m_ptr.m_float++;
	material.m_ambient.g  = *xdata.m_ptr.m_float++;
	material.m_ambient.b  = *xdata.m_ptr.m_float++;
	material.m_ambient.a  = *xdata.m_ptr.m_float++;
	material.m_diffuse.r  = material.m_ambient.r;
	material.m_diffuse.g  = material.m_ambient.g;
	material.m_diffuse.b  = material.m_ambient.b;
	material.m_diffuse.a  = material.m_ambient.a;
	material.m_specpower  = *xdata.m_ptr.m_float++;
	material.m_specular.r = *xdata.m_ptr.m_float++;
	material.m_specular.g = *xdata.m_ptr.m_float++;
	material.m_specular.b = *xdata.m_ptr.m_float++;
	material.m_specular.a = 1.0f;
	*xdata.m_ptr.m_float++;// emissive
	*xdata.m_ptr.m_float++;
	*xdata.m_ptr.m_float++;

	// Enumerate the child objects
	std::size_t num_children = GetNumChildren(data);
	for (std::size_t c = 0; c != num_children; ++c)
	{
		// Get the object
		D3DPtr<ID3DXFileData> child = 0;
		if (Failed(data->GetChild(c, &child.m_ptr))) { throw xfile::Exception(EResult_GetChildFailed); }

		// Get the type of object
		GUID guid = GetGUID(child);
		if (!IsInLoadSet(guid)) { continue; }

		if (IsEqualGUID(guid, TID_D3DRMTextureFilename))
		{
			material.m_texture.push_back(Texture());
			LoadTextureFilename(child, material.m_texture.back());
		}
		else
		{
			PR_INFO(PR_DBG_XFILE, Fmt("Ignoring material level template '%s'\n", GUIDToString(guid).c_str()).c_str());
		}
	}
}

// Read a texture filename
void XLoader::LoadTextureFilename(D3DPtr<ID3DXFileData> data, Texture& texture)
{
	XData xdata(data);
	texture.m_filename = xdata.m_ptr.m_str;

	// Attempt to resolve the filename
	if (!pr::filesys::FileExists(texture.m_filename))
	{
		std::string path = pr::filesys::GetDirectory(m_xfilepath);
		path += "/";
		path += texture.m_filename;
		pr::filesys::Canonicalise(path);
		if (pr::filesys::FileExists(path))
			texture.m_filename = path;
	}
}

// Read the vertex colours
void XLoader::LoadMeshVertexColours(D3DPtr<ID3DXFileData> data)
{
	XData xdata(data);

	uint num_colours = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, (uint)m_vertex.size());
	m_colour.clear();
	m_colour.resize(num_colours);
	for (uint c = 0; c < num_colours; ++c)
	{
		uint vertex_index = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, num_colours - 1);
		float r = *xdata.m_ptr.m_float++;
		float g = *xdata.m_ptr.m_float++;
		float b = *xdata.m_ptr.m_float++;
		float a = *xdata.m_ptr.m_float++;
		m_colour[vertex_index].set(r, g, b, a);
	}
}

// Fill in a meshes texture coordinates
void XLoader::LoadMeshTexCoords(D3DPtr<ID3DXFileData> data)
{
	XData xdata(data);

	uint num_vertices = Clamp<uint>(*xdata.m_ptr.m_uint++, 0, (uint)m_vertex.size());
	m_tex_coord.clear();
	m_tex_coord.resize(num_vertices);
	for (uint v = 0; v < num_vertices; ++v)
	{
		v2& coord = m_tex_coord[v];
		coord[0] = *xdata.m_ptr.m_float++;
		coord[1] = *xdata.m_ptr.m_float++;
	}
}

// Finish of the mesh
void XLoader::CompleteMesh(Mesh& mesh)	
{
	// Make an array of vertices
	uint num_faces = (uint)m_face.size();
	uint num_vertices = num_faces * 3;
	if (!num_vertices) return;
	m_x_vertex.clear();
	m_x_vertex.resize(num_vertices);
	m_px_vertex.clear();
	m_px_vertex.resize(num_vertices);

	XVertex*  xvertex  = &m_x_vertex[0];
	XVertex** pxvertex = &m_px_vertex[0];
	for (uint f = 0; f < num_faces; ++f)
	{
		for (uint v = 0; v < 3; ++v, ++xvertex, ++pxvertex)
		{
			*pxvertex = xvertex;
			xvertex->m_index_position = XVertex::INVALID;

			if (m_face[f].m_vert_index[v] < m_vertex.size())
			{
				xvertex->m_vertex_index		= m_face[f].m_vert_index[v];
				xvertex->m_vertex			= m_vertex[xvertex->m_vertex_index];
			}
			else
			{
				xvertex->m_vertex_index		= 0;
				xvertex->m_vertex			= pr::v4Origin;
			}
			if (m_face[f].m_norm_index[v] < m_normal.size())
			{
				xvertex->m_normal_index		= m_face[f].m_norm_index[v];
				xvertex->m_normal			= m_normal[xvertex->m_normal_index];
			}
			else
			{
				xvertex->m_normal_index		= 0;
				xvertex->m_normal			= v4ZAxis;
			}
			if (m_face[f].m_vert_index[v] < m_colour.size())
			{
				xvertex->m_colour_index		= m_face[f].m_vert_index[v];
				xvertex->m_colour			= m_colour[m_face[f].m_vert_index[v]];
			}
			else
			{
				xvertex->m_colour_index		= 0;
				xvertex->m_colour			= 0xFFFFFFFF;
			}
			if (m_face[f].m_tex_index[v] < m_tex_coord.size())
			{
				xvertex->m_tex_vertex_index	= m_face[f].m_tex_index[v];
				xvertex->m_tex_vertex		= m_tex_coord[xvertex->m_tex_vertex_index];
			}
			else
			{
				xvertex->m_tex_vertex_index	= 0;
				xvertex->m_tex_vertex		= pr::v2Zero;
			}
		}
	}

	std::sort(m_px_vertex.begin(), m_px_vertex.end(), XVertexPointersPred());

	m_px_vertex[0]->m_index_position = 0;
	XVertex** vert0 = &m_px_vertex[0];
	XVertex** vert1 = &m_px_vertex[1];
	uint num_unique_verts = 1;
	for (uint v = 1; v < num_vertices; ++v, ++vert0, ++vert1)
	{
		if (*(*vert1) == *(*vert0))
		{
			(*vert1)->m_index_position = (*vert0)->m_index_position;
		}
		else
		{
			(*vert1)->m_index_position = num_unique_verts;
			++num_unique_verts;
		}
	}

	// Copy the vertices into the mesh
	mesh.m_vertex.clear();
	mesh.m_vertex.resize(num_unique_verts);
	pxvertex     = &m_px_vertex[0];
	Vert* vertex = &mesh.m_vertex[0];
	for (uint v = 0; v < num_unique_verts; ++v, ++vertex)
	{
		while ((*pxvertex)->m_index_position < v)
		{
			++pxvertex;
			PR_ASSERT(PR_DBG_XFILE, pxvertex <= &m_px_vertex[num_vertices - 1], "");
		}
		*vertex = *static_cast<pr::Vert*>(*pxvertex);
		++pxvertex;
	}

	// Copy the faces into the mesh
	mesh.m_face.clear();
	mesh.m_face.resize(num_faces);
	for (uint f = 0; f < num_faces; ++f)
	{
		XFace&		xface	= m_face[f];
		Face&	face	= mesh.m_face[f];

		face.m_flags		= 0;
		face.m_mat_index	= xface.m_mat_index;
		for (int i = 0; i < 3; ++i)
		{
			// Update the indices of the faces while copying them into the mesh
			face.m_vert_index[i] = (WORD)m_x_vertex[f * 3 + i].m_index_position;
			PR_ASSERT(PR_DBG_XFILE, face.m_vert_index[i] < mesh.m_vertex.size(), "");
		}
	}

	// Copy the materials into the mesh
	mesh.m_material.clear();
	mesh.m_material.resize(m_material.size());
	for (uint m = 0; m < m_material.size(); ++m)
	{
		mesh.m_material[m] = m_material[m];
	}
}
