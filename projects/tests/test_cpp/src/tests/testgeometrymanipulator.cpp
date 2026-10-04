//*****************************************
//*****************************************
#include "test.h"
#include "pr/common/fmt.h"
#include "pr/str/prstring.h"
#include "pr/filesys/file.h"
#include "pr/geometry/mesh_tools.h"
#include "pr/storage/xfile/xfile.h"
#include "pr/linedrawer/ldr_helper2.h"

namespace TestGeometryManipulator
{
	using namespace pr;

	// Convert a stanford ply file to an x file
	void PlyToXFile(char const* filename);
	
	void Run()
	{
		//PlyToXFile("c:/documents and settings/paul/my documents/coding/artwork/stanford bunny/reconstruction/bun_zipper.ply");
		//PlyToXFile("c:/documents and settings/paul/my documents/coding/artwork/stanford bunny/reconstruction/bun_zipper_res2.ply");
		//PlyToXFile("c:/documents and settings/paul/my documents/coding/artwork/stanford bunny/reconstruction/bun_zipper_res3.ply");
		//PlyToXFile("c:/documents and settings/paul/my documents/coding/artwork/stanford bunny/reconstruction/bun_zipper_res4.ply");
	}

	// Convert a stanford ply file to an x file
	void PlyToXFile(char const* filename)
	{
		struct Header
		{
			Header() : m_valid(false), m_num_verts(0), m_num_faces(0) {}
			bool		 m_valid;
			unsigned int m_num_verts;
			unsigned int m_num_faces;
		};

		std::string fname = filename;
		filesys::RmvExtension(fname) += ".x";

		Geometry geometry;
		geometry.m_name = filesys::GetFiletitle(fname);

		pr::FilePtr ply = fopen(filename, "rt");
		std::string line(512, 0);

		// Read the header
		Header hdr;
		while( fgets(&line[0], (int)line.size(), ply) )
		{
			if     ( str::EqualNI(line, "ply", 3) )				{ hdr.m_valid = true; }
			else if( str::EqualNI(line, "element vertex", 14) )	{ line = line.substr(14); str::ExtractIntC(hdr.m_num_verts, 10, line.c_str()); }
			else if( str::EqualNI(line, "element face", 12) )		{ line = line.substr(12); str::ExtractIntC(hdr.m_num_faces, 10, line.c_str()); }
			else if( str::EqualNI(line, "end_header", 10) )		{ break; }
		}
		if( !hdr.m_valid ) { printf("Error: header invalid\n"); return; }

		geometry.m_frame.push_back(Frame());
		pr::Frame& frame = geometry.m_frame.back();
		frame.m_name = geometry.m_name;
		frame.m_transform.identity();
		pr::Mesh& mesh = frame.m_mesh;
		mesh.m_vertex.reserve(hdr.m_num_verts);
		mesh.m_face.reserve(hdr.m_num_faces);
		mesh.m_geom_type = pr::geom::EVN;

		// Read verts
		pr::Vert vert;
		vert.m_vertex.w = 1.0f;
		vert.m_normal.zero();
		vert.m_colour = pr::Colour32White;
		vert.m_tex_vertex.zero();
		for( unsigned int v = 0; v != hdr.m_num_verts; ++v )
		{
			std::string line(512, 0);
			if( !fgets(&line[0], (int)line.size(), ply) ||
				!str::ExtractRealC(vert.m_vertex.x, line.c_str()) ||
				!str::ExtractRealC(vert.m_vertex.y, line.c_str()) ||
				!str::ExtractRealC(vert.m_vertex.z, line.c_str()) )
			{ printf("Failed to read vertex %d\n", v); return; }
			mesh.m_vertex.push_back(vert);
		}

		// Read faces
		pr::Face face;
		face.m_mat_index = 0;
		face.m_flags = 0;
		for( unsigned int f = 0; f != hdr.m_num_faces; ++f )
		{
			uint num_indices;
			std::string line(512, 0);
			if( !fgets(&line[0], (int)line.size(), ply) ||
				!str::ExtractIntC(num_indices, 10, line.c_str()) ||
				num_indices != 3 ||
				!str::ExtractIntC(face.m_vert_index[0], 10, line.c_str()) ||
				!str::ExtractIntC(face.m_vert_index[1], 10, line.c_str()) ||
				!str::ExtractIntC(face.m_vert_index[2], 10, line.c_str()) )
			{ printf("Failed to read face %d\n", f); return; }
			mesh.m_face.push_back(face);
		}

		geometry::GenerateNormals(mesh);

		// Save as an x file
		xfile::Save(geometry, fname.c_str());
	}

}//namespace TestGeometryManipulator
