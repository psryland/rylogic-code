//************************************************************************
// XFile Saver
//  Copyright (C) Rylogic Ltd 2009
//************************************************************************

#include "pr/common/min_max_fix.h"
#include <d3d9.h>
#include <dxfile.h>
#include "pr/common/assert.h"
#include "pr/common/hresult.h"
#include "pr/storage/xfile/prxfiletemplates.h"
#include "pr/storage/xfile/xsaver.h"
#include "pr/storage/xfile/xfile.h"
#include "pr/storage/xfile/xfileinternal.h"

using namespace pr;
using namespace pr::xfile;

// The function that does the work
EResult XSaver::Save(const Geometry& geometry, const char* xfilename, const TGUIDSet* partial_save_set, const void* custom_templates, unsigned int custom_templates_size)
{
	// Create an X file interface, a new one of these is needed after each save
	D3DPtr<ID3DXFile> d3d_xfile;
	Verify(D3DXFileCreate(&d3d_xfile.m_ptr));

	// Register the direct x templates
	Verify(d3d_xfile->RegisterTemplates((void*)D3DTemplates, D3DTemplateBytes));

	// Register custum x file templates if there are any
	// Warning: this will fail if the null terminator is included in the custom templates
	// It should be const char templates[] = "..."; unsigned int templates_size = sizeof(templates) - 1;
	if( custom_templates_size )
	{
		Verify(d3d_xfile->RegisterTemplates((void*)custom_templates, custom_templates_size));
	}

	// Register more custum x file templates if there are any
	#pragma warning (disable : 4127) // conditional expression is constant
	if( CustomTemplateGUIDArray_Count )
	{
		Verify(d3d_xfile->RegisterTemplates((void*)CustomTemplates, CustomTemplates_Bytes));
	}
	#pragma warning (default : 4127) // conditional expression is constant

	// Use the filename provided or the one in the geometry
	if( !xfilename )	m_output_filename = geometry.m_name;
	else				m_output_filename = xfilename;

	// Add the ".x" extension
	pr::filesys::RmvExtension(m_output_filename);
	m_output_filename += ".x";

	// Get a pointer to the partial save set
	m_partial_save_set = partial_save_set;

	// Create the save object
	D3DPtr<ID3DXFileSaveObject> save_object;
	if( Failed(d3d_xfile->CreateSaveObject(m_output_filename.c_str(), D3DXF_FILESAVE_TOFILE, D3DXF_FILEFORMAT_TEXT, &save_object.m_ptr)) )
	{
		return EResult_FailedToCreateSaveObject;
	}

	try
	{
		// Save each top level frame
		std::size_t num_frames = geometry.m_frame.size();
		for( std::size_t f = 0; f < num_frames; ++f )
		{
			SaveFrame(save_object, geometry.m_frame[f]);
		}

		// Save the x file to disk
		if( Failed(save_object->Save()) ) { throw xfile::Exception(EResult_SaveFailed); }
	}
	catch( EResult& result )
	{
		return result;
	}
	return EResult_Success;
}

// Returns true if a guid should be saved
bool XSaver::IsInSaveSet(GUID guid) const
{
	if( !m_partial_save_set ) return true;
	return m_partial_save_set->find(guid) != m_partial_save_set->end();
}

// Write the frame into the x file
void XSaver::SaveFrame(D3DPtr<ID3DXFileSaveObject> save_object, const Frame& frame)
{
	if( !IsInSaveSet(TID_D3DRMFrame) ) return;

	D3DPtr<ID3DXFileSaveData> data;
	if( Failed(save_object->AddDataObject(TID_D3DRMFrame, frame.m_name.c_str(), 0, 0, 0, &data.m_ptr)) ) { throw xfile::Exception(EResult_AddDataFailed); }

	SaveFrameTransform(data, frame.m_transform);
	SaveMesh          (data, frame.m_mesh);
}

// Write the frame transform into the x file
void XSaver::SaveFrameTransform(D3DPtr<ID3DXFileSaveData> parent, const m4x4& transform)
{
	if( !IsInSaveSet(TID_D3DRMFrameTransformMatrix) ) return;

	D3DPtr<ID3DXFileSaveData> child;
	if( Failed(parent->AddDataObject(TID_D3DRMFrameTransformMatrix, 0, 0, sizeof(m4x4), &transform[0][0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }
}

// Write the mesh and its children into the x file
void XSaver::SaveMesh(D3DPtr<ID3DXFileSaveData> parent, const Mesh& mesh)
{
	if( !IsInSaveSet(TID_D3DRMMesh) ) return;

	std::size_t num_vertices = mesh.m_vertex.size();
	std::size_t num_faces    = mesh.m_face.size();

	// Prepare the data in a buffer
	std::size_t buffer_size = 0;
	if( num_vertices ) { buffer_size += 1 + num_vertices * 3; }
	if( num_faces    ) { buffer_size += 1 + num_faces * 4; }

	m_buffer.clear();
	m_buffer.reserve(buffer_size);

	if( num_vertices )
	{
		m_buffer.push_back((uint)num_vertices);
		for( std::size_t v = 0; v < num_vertices; ++v )
		{
			m_buffer.push_back(FtoUint(mesh.m_vertex[v].m_vertex[0]));
			m_buffer.push_back(FtoUint(mesh.m_vertex[v].m_vertex[1]));
			m_buffer.push_back(FtoUint(mesh.m_vertex[v].m_vertex[2]));
		}
	}

	if( num_faces )
	{
		m_buffer.push_back((uint)num_faces);
		for( std::size_t f = 0; f < num_faces; ++f )
		{
			const Face& face = mesh.m_face[f];
			m_buffer.push_back(3);
			m_buffer.push_back(face.m_vert_index[0]);
			m_buffer.push_back(face.m_vert_index[1]);
			m_buffer.push_back(face.m_vert_index[2]);
		}
	}

	if( !m_buffer.empty() )
	{
		D3DPtr<ID3DXFileSaveData> child;
		if( Failed(parent->AddDataObject(TID_D3DRMMesh, 0, 0, m_buffer.size() * sizeof(uint), &m_buffer[0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }

		SaveMeshMaterials(child, mesh);
		if (mesh.m_geom_type & geom::ENormal ) SaveMeshNormals  (child, mesh);
		if (mesh.m_geom_type & geom::EColour ) SaveMeshColours  (child, mesh);
		if (mesh.m_geom_type & geom::ETexture) SaveMeshTexCoords(child, mesh);
	}
}

// Write the vertex normals into the xfile
void XSaver::SaveMeshNormals(D3DPtr<ID3DXFileSaveData> parent, const Mesh& mesh)
{
	if( !IsInSaveSet(TID_D3DRMMeshNormals) ) return;

	std::size_t num_normals = mesh.m_vertex.size();
	std::size_t num_faces   = mesh.m_face.size();

	// Prepare the data in a buffer
	std::size_t buffer_size = 0;
	if( num_normals ) { buffer_size += 1 + num_normals * 3; }
	if( num_faces   ) { buffer_size += 1 + num_faces * 4; }

	m_buffer.clear();
	m_buffer.reserve(buffer_size);

	if( num_normals )
	{
		m_buffer.push_back((uint)num_normals);
		for( std::size_t n = 0; n < num_normals; ++n )
		{
			m_buffer.push_back(FtoUint(mesh.m_vertex[n].m_normal[0]));
			m_buffer.push_back(FtoUint(mesh.m_vertex[n].m_normal[1]));
			m_buffer.push_back(FtoUint(mesh.m_vertex[n].m_normal[2]));
		}
	}

	if( num_faces )
	{
		m_buffer.push_back((uint)num_faces);
		for( std::size_t f = 0; f < num_faces; ++f )
		{
			m_buffer.push_back(3);
			m_buffer.push_back(mesh.m_face[f].m_vert_index[0]);
			m_buffer.push_back(mesh.m_face[f].m_vert_index[1]);
			m_buffer.push_back(mesh.m_face[f].m_vert_index[2]);
		}
	}

	if( !m_buffer.empty() )
	{
		D3DPtr<ID3DXFileSaveData> child;
		if( Failed(parent->AddDataObject(TID_D3DRMMeshNormals, 0, 0, m_buffer.size() * sizeof(uint), &m_buffer[0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }
	}
}

// Write the materials into the xfile
void XSaver::SaveMeshMaterials(D3DPtr<ID3DXFileSaveData> parent, const Mesh& mesh)
{
	if( !IsInSaveSet(TID_D3DRMMeshMaterialList) ) return;

	// Prepare the data in a buffer
	std::size_t num_materials = mesh.m_material.size();
	std::size_t num_faces     = mesh.m_face.size();

	// Prepare the data in a buffer
	std::size_t buffer_size = 0;
	if( num_materials ) { buffer_size += 1; }
	if( num_faces     ) { buffer_size += 1 + num_faces; }

	m_buffer.clear();
	m_buffer.reserve(buffer_size);

	if( num_materials )
	{
		m_buffer.push_back((uint)num_materials);
		if( num_faces )
		{
			m_buffer.push_back((uint)num_faces);
			for( std::size_t f = 0; f < num_faces; ++f )
			{
				m_buffer.push_back(mesh.m_face[f].m_mat_index);
			}
		}
	}

	if( !m_buffer.empty() )
	{
		D3DPtr<ID3DXFileSaveData> child;
		if( Failed(parent->AddDataObject(TID_D3DRMMeshMaterialList, 0, 0, m_buffer.size() * sizeof(uint), &m_buffer[0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }

		for( std::size_t m = 0; m < num_materials; ++m )
		{
			SaveMaterial(child, mesh.m_material[m]);
		}
	}
}

// Write the vertex colours into the xfile
void XSaver::SaveMeshColours(D3DPtr<ID3DXFileSaveData> parent, const Mesh& mesh)
{
	if( !IsInSaveSet(TID_D3DRMMeshVertexColors) ) return;

	std::size_t num_colours = mesh.m_vertex.size();

	// Prepare the data in a buffer
	std::size_t buffer_size = 0;
	if( num_colours ) { buffer_size += 1 + num_colours * 5; }

	m_buffer.clear();
	m_buffer.reserve(buffer_size);

	if( num_colours )
	{
		m_buffer.push_back((uint)num_colours);
		for( std::size_t c = 0; c < num_colours; ++c )
		{
			Colour colour;
			colour = mesh.m_vertex[c].m_colour;

			m_buffer.push_back((uint)c);
			m_buffer.push_back(FtoUint(colour.r));
			m_buffer.push_back(FtoUint(colour.g));
			m_buffer.push_back(FtoUint(colour.b));
			m_buffer.push_back(FtoUint(colour.a));
		}
	}
	if( !m_buffer.empty() )
	{
		D3DPtr<ID3DXFileSaveData> child;
		if( Failed(parent->AddDataObject(TID_D3DRMMeshVertexColors, 0, 0, m_buffer.size() * sizeof(uint), &m_buffer[0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }
	}
}

// Write the texture coordinates into the xfile
void XSaver::SaveMeshTexCoords(D3DPtr<ID3DXFileSaveData> parent, const Mesh& mesh)
{
	if( !IsInSaveSet(TID_D3DRMMeshTextureCoords) ) return;

	std::size_t num_vertices = mesh.m_vertex.size();

	// Prepare the data in a buffer
	std::size_t buffer_size = 0;
	if( num_vertices ) { buffer_size += 1 + num_vertices * 2; }

	m_buffer.clear();
	m_buffer.reserve(buffer_size);

	if( num_vertices )
	{
		m_buffer.push_back((uint)num_vertices);
		for( std::size_t v = 0; v < num_vertices; ++v )
		{
			m_buffer.push_back(FtoUint(mesh.m_vertex[v].m_tex_vertex[0]));
			m_buffer.push_back(FtoUint(mesh.m_vertex[v].m_tex_vertex[1]));
		}
	}

	if( !m_buffer.empty() )
	{
		D3DPtr<ID3DXFileSaveData> child;
		if( Failed(parent->AddDataObject(TID_D3DRMMeshTextureCoords, 0, 0, m_buffer.size() * sizeof(uint), &m_buffer[0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }
	}
}

// Write the material descriptions into the xfile
void XSaver::SaveMaterial(D3DPtr<ID3DXFileSaveData> parent, const Material& material)
{
	if( !IsInSaveSet(TID_D3DRMMaterial) ) return;

	m_buffer.clear();
	m_buffer.reserve(12);
	
	// Prepare the data in a buffer
	m_buffer.push_back(FtoUint(material.m_diffuse.r));
	m_buffer.push_back(FtoUint(material.m_diffuse.g));
	m_buffer.push_back(FtoUint(material.m_diffuse.b));
	m_buffer.push_back(FtoUint(material.m_diffuse.a));
	m_buffer.push_back(FtoUint(material.m_specpower));
	m_buffer.push_back(FtoUint(material.m_specular.r));
	m_buffer.push_back(FtoUint(material.m_specular.g));
	m_buffer.push_back(FtoUint(material.m_specular.b));
	m_buffer.push_back(FtoUint(0.0f)); // emissive
	m_buffer.push_back(FtoUint(0.0f));
	m_buffer.push_back(FtoUint(0.0f));
	
	D3DPtr<ID3DXFileSaveData> child;
	if( Failed(parent->AddDataObject(TID_D3DRMMaterial, 0, 0, m_buffer.size() * sizeof(uint), &m_buffer[0], &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }

	std::size_t num_textures = material.m_texture.size();
	for( std::size_t i = 0; i < num_textures; ++i )
	{
		SaveSubmaterial(child, material.m_texture[i]);
	}
}

// Write the texture filename into the xfile
void XSaver::SaveSubmaterial(D3DPtr<ID3DXFileSaveData> parent, const Texture& texture)
{
	if( !IsInSaveSet(TID_D3DRMTextureFilename) ) return;

	if( texture.m_filename.size() == 0 ) return;
	const char* str = texture.m_filename.c_str();

	D3DPtr<ID3DXFileSaveData> child;
	if( Failed(parent->AddDataObject(TID_D3DRMTextureFilename, 0, 0, texture.m_filename.size() + 1, str, &child.m_ptr)) )	{ throw xfile::Exception(EResult_AddDataFailed); }
}
