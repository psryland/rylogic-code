//*****************************************
//*****************************************
#include "test.h"
#include <windows.h>
#include <algorithm>
#include "pr/maths/maths.h"
#include "pr/maths/perlinnoise.h"
#include "pr/maths/convexhull.h"
#include "pr/maths/triangulate.h"
#include "pr/image/image.h"
#include "pr/filesys/autofile.h"
#include "pr/linedrawer/ldr_helper2.h"
#include "pr/storage/xfile/xfile.h"
//#include "pr/physics/shape/convexdecompose/shapeconvexdecompose.h"

using namespace pr;

//namespace pr { namespace ph { namespace convex_decompose {
//extern void DumpMesh(convex_decompose::Mesh const& mesh, char const* colour);
//}}}
//
//namespace TestConvexDecomposition
//{
//	struct Face { std::size_t i[3]; };
//	namespace test0
//	{
//		v4 verts[] = 
//		{
//			{0.0f, 0.0f, 0.0f, 1.0f},
//			{1.0f, 0.0f, 0.0f, 1.0f},
//			{0.0f, 1.0f, 0.0f, 1.0f},
//			{1.0f, 1.0f, 0.0f, 1.0f},
//			{0.0f, 0.0f, 1.0f, 1.0f},
//			{1.0f, 0.0f, 1.0f, 1.0f},
//			{0.0f, 1.0f, 1.0f, 1.0f},
//			{1.0f, 1.0f, 1.0f, 1.0f}
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] = 
//		{
//			{ 0,  3,  1}, { 0,  2,  3},
//			{ 1,  7,  5}, { 1,  3,  7},
//			{ 5,  6,  4}, { 5,  7,  6},
//			{ 4,  2,  0}, { 4,  6,  2},
//			{ 3,  6,  7}, { 3,  2,  6},
//			{ 4,  1,  5}, { 4,  0,  1}
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace test0
//	namespace test1
//	{
//		v4 verts[] = 
//		{
//			{0.0f, 0.0f, 0.0f, 1.0f},
//			{1.0f, 0.0f, 0.0f, 1.0f},
//			{0.0f, 1.0f, 0.0f, 1.0f},
//			{1.0f, 1.0f, 0.0f, 1.0f},
//			{1.0f, 0.0f, 2.0f, 1.0f},
//			{2.0f, 0.0f, 2.0f, 1.0f},
//			{1.0f, 1.0f, 2.0f, 1.0f},
//			{2.0f, 1.0f, 2.0f, 1.0f},
//			{0.0f, 0.0f, 3.0f, 1.0f},
//			{2.0f, 0.0f, 3.0f, 1.0f},
//			{0.0f, 1.0f, 3.0f, 1.0f},
//			{2.0f, 1.0f, 3.0f, 1.0f},
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] = 
//		{
//			{ 0,  3,  1}, { 0,  2,  3},
//			{ 1,  6,  4}, { 1,  3,  6},
//			{ 4,  7,  5}, { 4,  6,  7},
//			{ 5, 11,  9}, { 5,  7, 11},
//			{ 9, 10,  8}, { 9, 11, 10},
//			{ 8,  2,  0}, { 8, 10,  2},
//			{ 2,  6,  3}, { 2, 10,  6},
//			{ 6, 11,  7}, { 6, 10, 11},
//			{ 0,  4,  8}, { 0,  1,  4},
//			{ 8,  5,  9}, { 8,  4,  5}
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace test1
//	namespace test2
//	{
//		v4 verts[] =
//		{
//			{0.0f, 0.0f, 0.0f, 1.0f},
//			{1.0f, 0.0f, 0.0f, 1.0f},
//			{0.0f, 1.0f, 0.0f, 1.0f},
//			{1.0f, 1.0f, 0.0f, 1.0f},
//
//			{0.5f, 0.0f, 3.0f, 1.0f},
//			{1.0f, 0.5f, 3.0f, 1.0f},
//			{0.0f, 0.5f, 3.0f, 1.0f},
//			{0.5f, 1.0f, 3.0f, 1.0f},
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] = 
//		{
//			{ 0,  3,  1}, { 0,  2,  3},
//			{ 1,  7,  5}, { 1,  3,  7},
//			{ 5,  6,  4}, { 5,  7,  6},
//			{ 4,  2,  0}, { 4,  6,  2},
//			{ 0,  5,  4}, { 0,  1,  5},
//			{ 3,  6,  7}, { 3,  2,  6}
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace test2
//	namespace test3
//	{
//		v4 verts[] = 
//		{
//			{0.0f, 0.0f, 0.0f, 1.0f},
//			{3.0f, 0.0f, 0.0f, 1.0f},
//			{0.0f, 0.0f, 3.0f, 1.0f},
//			{3.0f, 0.0f, 3.0f, 1.0f},
//			{1.0f, 1.0f, 1.0f, 1.0f},
//			{2.0f, 1.0f, 1.0f, 1.0f},
//			{1.0f, 1.0f, 2.0f, 1.0f},
//			{2.0f, 1.0f, 2.0f, 1.0f},
//			{1.0f, 2.0f, 1.0f, 1.0f},
//			{2.0f, 2.0f, 1.0f, 1.0f},
//			{1.0f, 2.0f, 2.0f, 1.0f},
//			{2.0f, 2.0f, 2.0f, 1.0f},
//			{0.0f, 3.0f, 0.0f, 1.0f},
//			{3.0f, 3.0f, 0.0f, 1.0f},
//			{0.0f, 3.0f, 3.0f, 1.0f},
//			{3.0f, 3.0f, 3.0f, 1.0f}
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] = 
//		{
//			{ 0,  5,  1}, { 0,  4,  5},
//			{ 1,  7,  3}, { 1,  5,  7},
//			{ 3,  6,  2}, { 3,  7,  6},
//			{ 2,  4,  0}, { 2,  6,  4},
//			{ 4,  9,  5}, { 4,  8,  9},
//			{ 5, 11,  7}, { 5,  9, 11},
//			{ 7, 10,  6}, { 7, 11, 10},
//			{ 6,  8,  4}, { 6, 10,  8},
//			{ 8, 13,  9}, { 8, 12, 13},
//			{ 9, 15, 11}, { 9, 13, 15},
//			{11, 14, 10}, {11, 15, 14},
//			{10, 12,  8}, {10, 14, 12},
//			{12, 15, 13}, {12, 14, 15},
//			{ 2,  1,  3}, { 2,  0,  1}
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace test3
//	namespace test4
//	{
//		v4 verts[] = 
//		{
//			{0.0f, 0.0f, 0.0f, 1.0f},
//			{2.0f, 0.0f, 0.0f, 1.0f},
//			{0.0f, 0.0f, 2.0f, 1.0f},
//			{2.0f, 0.0f, 2.0f, 1.0f},
//			{0.0f, 1.0f, 0.0f, 1.0f},
//			{2.0f, 1.0f, 0.0f, 1.0f},
//			{0.0f, 1.0f, 2.0f, 1.0f},
//			{2.0f, 1.0f, 2.0f, 1.0f},
//			{1.0f, 0.5f, 1.0f, 1.0f}
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] = 
//		{
//			{ 0,  5,  1}, { 0,  4,  5},
//			{ 1,  7,  3}, { 1,  5,  7},
//			{ 3,  6,  2}, { 3,  7,  6},
//			{ 2,  4,  0}, { 2,  6,  4},
//			{ 4,  8,  5}, { 5,  8,  7},
//			{ 7,  8,  6}, { 6,  8,  4},
//			{ 2,  1,  3}, { 2,  0,  1}
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace test4
//	namespace test5
//	{
//		std::size_t const vert_count = 20;
//		std::size_t const face_count = 50;
//		v4			verts[vert_count];
//		std::size_t num_verts = 0;
//		Face		faces[face_count];
//		std::size_t num_faces = 0;
//		inline void Create()
//		{
//			for( int i = 0; i != vert_count; ++i )
//				verts[i].Random3(1.0f, 1.5f, 1.0f);
//
//			hull::DefaultFaceContainer face_cont;
//			num_verts = ConvexHull3D(&verts[0], &verts[0] + vert_count, face_cont) - verts;
//			num_faces = face_cont.size();
//			if( num_faces > face_count ) { PR_ASSERT(1, false); }
//			
//			Face* face = faces;
//			for( hull::Face* f = face_cont.begin(), *f_end = face_cont.end(); f != f_end; ++f, ++face )
//			{
//				face->i[0] = f->m_vert[0] - verts;
//				face->i[1] = f->m_vert[1] - verts;
//				face->i[2] = f->m_vert[2] - verts;
//			}
//
//			// Dent the hull
//			for( int i = 0; i != 10; ++i )
//			{
//				v4& v = verts[Rand(0, (int)num_verts)];
//				v *= 0.8f;
//				v.w = 1.0f;
//			}
//		}
//		inline void Destroy() {}
//	}//namespace test5
//	namespace test6
//	{
//		v4 verts[] =
//		{
//			{0.000000f, 0.000000f, 0.0f, 1.0f},
//			{1.000000f, 0.000000f, 0.0f, 1.0f},
//			{0.500000f, 0.500000f, 0.0f, 1.0f},
//			{0.000000f, 0.000000f, 1.0f, 1.0f},
//			{0.877583f, 0.479426f, 1.0f, 1.0f},
//			{0.199079f, 0.678504f, 1.0f, 1.0f},
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] =
//		{
//			{0, 2, 1}, {3, 4, 5},
//			{0, 4, 3}, {0, 1, 4},
//			{1, 5, 4}, {1, 2, 5},
//			{2, 3, 5}, {2, 0, 3},
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace test6
//	namespace testcar
//	{
//		v4 verts[] =
//		{
//			{ -1.0f,  0.0f,  2.0f, 1.0f}, // bottom
//			{  0.0f,  0.0f,  2.0f, 1.0f},
//			{  1.0f,  0.0f,  2.0f, 1.0f},
//			{ -1.0f,  0.0f,  1.0f, 1.0f},
//			{  0.0f,  0.0f,  1.0f, 1.0f},
//			{  1.0f,  0.0f,  1.0f, 1.0f},
//			{ -1.0f,  0.0f,  0.0f, 1.0f},
//			{  0.0f,  0.0f,  0.0f, 1.0f},
//			{  1.0f,  0.0f,  0.0f, 1.0f},
//			{ -1.0f,  0.0f, -1.0f, 1.0f},
//			{  0.0f,  0.0f, -1.0f, 1.0f},
//			{  1.0f,  0.0f, -1.0f, 1.0f},
//
//			{ -1.0f,  1.0f,  2.0f, 1.0f}, // middle
//			{  0.0f,  1.0f,  2.0f, 1.0f},
//			{  1.0f,  1.0f,  2.0f, 1.0f},
//			{ -1.0f,  1.0f,  1.0f, 1.0f},
//			{  0.0f,  1.0f,  1.0f, 1.0f},
//			{  1.0f,  1.0f,  1.0f, 1.0f},
//			{ -1.0f,  1.0f,  0.0f, 1.0f},
//			{  1.0f,  1.0f,  0.0f, 1.0f},
//			{ -1.0f,  1.0f, -1.0f, 1.0f},
//			{  0.0f,  1.0f, -1.0f, 1.0f},
//			{  1.0f,  1.0f, -1.0f, 1.0f},
//
//			{ -0.5f,  1.6f,  0.5f, 1.0f}, // top
//			{  0.5f,  1.6f,  0.5f, 1.0f},
//			{ -0.5f,  1.6f, -0.5f, 1.0f},
//			{  0.5f,  1.6f, -0.5f, 1.0f},
//			{  0.0f,  1.6f,  0.0f, 1.0f},
//		};
//		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
//		Face faces[] =
//		{
//			{ 0,  3,  4}, { 0,  4,  1},	// bottom
//			{ 1,  4,  5}, { 1,  5,  2},
//			{ 3,  6,  7}, { 3,  7,  4},
//			{ 4,  7,  8}, { 4,  8,  5},
//			{ 6,  9, 10}, { 6, 10,  7},
//			{ 7, 10, 11}, { 7, 11,  8},
//
//			{ 0,  1, 13}, { 0, 13, 12}, // middle
//			{ 1,  2, 14}, { 1, 14, 13},
//			{ 2,  5, 17}, { 2, 17, 14},
//			{ 5,  8, 19}, { 5, 19, 17},
//			{ 8, 11, 22}, { 8, 22, 19},
//			{11, 10, 21}, {11, 21, 22},
//			{10,  9, 20}, {10, 20, 21},
//			{ 9,  6, 18}, { 9, 18, 20},
//			{ 6,  3, 15}, { 6, 15, 18},
//			{ 3,  0, 12}, { 3, 12, 15},
//
//			{12, 16, 15}, {12, 13, 16},	// top
//			{13, 17, 16}, {13, 14, 17},			
//			{15, 16, 23}, {23, 16, 24}, {16, 17, 24},
//			{17, 19, 24}, {24, 19, 26}, {19, 22, 26},
//			{22, 21, 26}, {26, 21, 25}, {21, 20, 25},
//			{20, 18, 25}, {25, 18, 23}, {18, 15, 23},
//			{23, 24, 27}, {24, 26, 27},
//			{26, 25, 27}, {25, 23, 27},
//		};
//		std::size_t const num_faces = sizeof(faces) / sizeof(faces[0]);
//		inline void Create() {}
//		inline void Destroy() {}
//	}//namespace testcar
//	
//	//namespace test7
//	//{
//	//	v4*			verts;
//	//	std::size_t num_verts = 0;
//	//	Face*		faces;
//	//	std::size_t num_faces = 0;
//	//	inline void Create()
//	//	{
//	//		verts = test6::verts;
//	//		num_verts = test6::num_verts;
//	//		faces = test6::faces;
//	//		num_faces = test6::num_faces;
//
//	//		Geometry xfile;
//	//		//xfile::Load("c:/documents and settings/paul/my documents/coding/artwork/Stanford Bunny/bun_zipper_res4.x", xfile);
//	//		xfile::Load("c:/documents and settings/paul/my documents/coding/artwork/geosphere_1.x", xfile);
//	//		
//	//		pr::Mesh& mesh = xfile.m_frame[0].m_mesh;
//	//		geometry::ReduceMesh(mesh);
//	//		geometry::GenerateNormals(mesh);
//	//		xfile::Save(xfile, "C:/deleteme/reduced.x");
//
//	//		num_verts = mesh.m_vertex.size();
//	//		num_faces = mesh.m_face.size();
//	//		verts = new v4[num_verts];
//	//		faces = new Face[num_faces];
//
//	//		BoundingBox bbox;
//	//		for( std::size_t i = 0; i != num_verts; ++i )
//	//		{
//	//			verts[i] = mesh.m_vertex[i].m_vertex;
//	//			Encompase(bbox, verts[i]);
//	//		}
//	//		int i = bbox.m_radius.LargestElement3();
//	//		float scale = 1.0f / bbox.m_radius[i];
//	//		for( std::size_t i = 0; i != num_verts; ++i )
//	//		{
//	//			verts[i] *= scale;
//	//			verts[i].w = 1.0f;
//	//		}
//
//	//		for( std::size_t i = 0; i != num_faces; ++i )
//	//		{
//	//			faces[i].i[0] = mesh.m_face[i].m_vert_index[0];
//	//			faces[i].i[1] = mesh.m_face[i].m_vert_index[1];
//	//			faces[i].i[2] = mesh.m_face[i].m_vert_index[2];
//	//		}
//
//	//		// Add dents
//	//		for( int i = 0; i != 10; ++i )
//	//		{
//	//			//v4& v = verts[Rand(0, (int)num_verts)];
//	//			//v *= 0.8f;
//	//			//v.w = 1.0f;
//	//		}
//	//	}
//	//	inline void Destroy()
//	//	{
//	//		delete [] verts;
//	//		delete [] faces;
//	//	}
//	//}//namespace test7
//
//	void Run()
//	{
//		using namespace test6;
//		Create();
//		
//		// Centre model around origin
//		float min_x = 0.0f, max_x = 0.0f;
//		{
//			float count = 0.0f;
//			v4 model_centre = v4Zero;
//			for( v4* v = verts; v != verts + num_verts; ++v )	{ model_centre += *v; count += 1.0f; }
//			model_centre /= count;
//			model_centre.w = 0.0f;
//			for( v4* v = verts; v != verts + num_verts; ++v )	{ *v -= model_centre; min_x = Minimise(min_x, v->x); max_x = Maximise(max_x, v->x); }
//		}
//
//		// Construct a mesh in 'mesh'
//		ph::convex_decompose::VertContainer mesh_vert_cont;
//		ph::convex_decompose::Mesh mesh(&mesh_vert_cont);
//		ph::convex_decompose::CreateMesh(verts, num_verts, faces[0].i, num_faces, mesh);
//		
//		// Generate convex pieces of 'mesh' in 'polytopes' and 'poly_vert_cont'
//		ph::convex_decompose::VertContainer poly_vert_cont;
//		ph::convex_decompose::TMesh polytopes;
//		ph::ConvexDecompose(mesh, poly_vert_cont, polytopes);
//
//		// ldr output
//		{
//			StartFile("C:/Deleteme/Deformable_polytopes.pr_script");
//			//ph::GroupStartCyclic("Decom", 0, 0.5f);
//			ph::convex_decompose::DumpMesh(mesh, "80A00000");
//			ldr::GroupStart("Decomposition");//, 0, 1.0f);
//			ldr::Position(v4::make(max_x + (max_x - min_x) + 0.2f, 0.0f, 0.0f, 1.0f));
//			for( ph::convex_decompose::TMesh::iterator i = polytopes.begin(), i_end = polytopes.end(); i != i_end; ++i )
//			{
//				float count = 0;
//				v4 centre = v4Zero;
//				for( ph::convex_decompose::Vert* v = i->vert_first(); v; v = i->vert_next(v) )
//				{	centre += v->m_pos; count += 1.0f; }
//				centre /= count;
//				centre.w = 0.0f;
//				if( !FEqlZero3(centre) ) centre.GetNormal3();
//				for( ph::convex_decompose::Vert* v = i->vert_first(); v; v = i->vert_next(v) )
//				{	v->m_pos += centre * 0.2f; }
//				ph::convex_decompose::DumpMesh(*i, "8000A000");
//			}		
//			ldr::GroupEnd();
//			//ldr::GroupEnd();
//			EndFile();
//		}
//		
//		Destroy();
//	}// Run()
//}//namespace TestConvexDecomposition

namespace TestTriangulate
{
	struct Edge	{ std::size_t m_i0, m_i1; };
	namespace polygon1
	{
		v4 verts[] = 
		{
			{-2.0f,  2.0f, 0.0f, 1.0f},
			{ 2.0f,  2.5f, 0.0f, 1.0f},
			{ 0.0f,  1.0f, 0.0f, 1.0f},
			{ 0.0f,  0.0f, 0.0f, 1.0f},
			{-1.0f, -1.0f, 0.0f, 1.0f},
			{ 1.0f, -1.0f, 0.0f, 1.0f},
			{ 0.0f, -0.5f, 0.0f, 1.0f},
			{-2.0f, -2.0f, 0.0f, 1.0f},
			{ 2.0f, -2.5f, 0.0f, 1.0f}
		};
		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
		Edge edges[] =
		{
			{0, 7},
			{7, 6},
			{6, 8},
			{8, 1},
			{1, 0},
			{2, 5},
			{5, 3},
			{3, 4},
			{4, 2}
		};
		std::size_t const num_edges = sizeof(edges) / sizeof(edges[0]);
	}
	namespace polygon2
	{
		v4 verts[] = 
		{
			{-0.5f, -0.5f, 0.0f, 1.0f},
			{ 0.0f,  0.0f, 0.0f, 1.0f},
			{ 0.0f,  0.5f, 0.0f, 1.0f},
			{ 1.0f, -1.0f, 0.0f, 1.0f}
		};
		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
		Edge edges[] =
		{
			{0, 1},
			{1, 3},
			{3, 2},
			{2, 0}
		};
		std::size_t const num_edges = sizeof(edges) / sizeof(edges[0]);
	}
	namespace polygon3
	{
		v4 verts[] = 
		{
			{-1.0f,  0.3f, 0.0f, 1.0f},
			{ 0.5f,  1.0f, 0.0f, 1.0f},
			{ 1.0f, -1.0f, 0.0f, 1.0f},
			{ 0.0f, -0.25f, 0.0f, 1.0f},
			{ 0.0f,  0.5f, 0.0f, 1.0f},
			{ 0.5f, -0.5f, 0.0f, 1.0f}
		};
		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
		Edge edges[] =
		{
			{0, 2},
			{2, 1},
			{1, 0},
			{3, 4},
			{4, 5},
			{5, 3}
		};
		std::size_t const num_edges = sizeof(edges) / sizeof(edges[0]);
	}
	namespace polygon4
	{
		v4 verts[] = 
		{
			{ 1.0f,  0.0f, 0.0f, 1.0f},
			{ 2.0f,  1.0f, 0.0f, 1.0f},
			{ 1.5f,  2.5f, 0.0f, 1.0f},
			{ 1.5f,  1.5f, 0.0f, 1.0f},
			{ 1.0f,  3.0f, 0.0f, 1.0f},
			{ 1.0f,  2.0f, 0.0f, 1.0f},
			{ 0.0f,  4.0f, 0.0f, 1.0f}
		};
		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
		Edge edges[] =
		{
			{0, 1},
			{1, 2},
			{3, 4},
			{2, 3},
			{5, 6},
			{4, 5},
			{6, 0}
		};
		std::size_t const num_edges = sizeof(edges) / sizeof(edges[0]);
	}
	namespace polygon5
	{
		v4 verts[] = 
		{
			{ 0.0f,  0.0f, 0.0f, 1.0f},
			{ 1.0f,  1.0f, 0.0f, 1.0f},
			{ 0.0f,  2.0f, 0.0f, 1.0f},
			
			{ 0.1f,  1.0f, 0.0f, 1.0f},
			{ 0.4f,  1.4f, 0.0f, 1.0f},
			{ 0.4f,  0.6f, 0.0f, 1.0f},
			
			{ 0.5f,  1.3f, 0.0f, 1.0f},
			{ 0.9f,  1.0f, 0.0f, 1.0f},
			{ 0.5f,  0.7f, 0.0f, 1.0f},

			{ 0.2f,  1.1f, 0.0f, 1.0f},
			{ 0.2f,  0.9f, 0.0f, 1.0f},
			{ 0.3f,  1.0f, 0.0f, 1.0f},

			{ -0.1f,  0.0f, 0.0f, 1.0f},
			{ -0.1f,  2.0f, 0.0f, 1.0f},
			{ -0.5f,  1.0f, 0.0f, 1.0f},
		};
		std::size_t const num_verts = sizeof(verts) / sizeof(verts[0]);
		Edge edges[] =
		{
			{0, 1},
			{1, 2},
			{2, 0},

			{3, 4},
			{4, 5},
			{5, 3},
			
			{6, 7},
			{7, 8},
			{8, 6},

			{9, 10},
			{10, 11},
			{11, 9},

			{12, 13},
			{13, 14},
			{14, 12}
		};
		std::size_t const num_edges = sizeof(edges) / sizeof(edges[0]);
	}

	struct Tris
	{
		std::vector<std::size_t> m_tri_list;
		void TriangulationFace(std::size_t i0, std::size_t i1, std::size_t i2, bool)
		{
			m_tri_list.push_back(i0);
			m_tri_list.push_back(i1);
			m_tri_list.push_back(i2);
		}
	};

	void Run()
	{
		// test separate and nested polygons
		using namespace polygon5;
		Tris mesh;
		
		m4x4 rot = Rotation4x4(v4XAxis, maths::pi * 4.0f / 8.0f, v4Origin);
		for( std::size_t i = 0; i != num_verts; ++i )
			verts[i] = rot * verts[i];

		Triangulate<0,2>(verts, num_verts, edges, num_edges, mesh);
	}//Run()
}//namespace TestTriangulate

//namespace TestConvexHull
//{
//	v4 verts[] =
//	{
//		{0.0f,  0.0f,  0.0f, 1.0f},
//		{1.0f,  0.0f,  0.0f, 1.0f},
//		{0.0f,  1.0f,  0.0f, 1.0f},
//		{1.0f,  1.0f,  0.0f, 1.0f},
//		{0.0f,  0.0f,  1.0f, 1.0f},
//		{1.0f,  0.0f,  1.0f, 1.0f},
//		{0.0f,  1.0f,  1.0f, 1.0f},
//		{1.0f,  1.0f,  1.0f, 1.0f},
//		{2.0f,  0.0f,  2.0f, 1.0f},
//		{3.0f,  1.0f,  2.0f, 1.0f},
//		{-1.0f, 0.0f, -2.0f, 1.0f}
//	};
//	std::size_t vert_count = sizeof(verts)/sizeof(verts[0]);
//
//	void Run()
//	{
//		hull::DefaultFaceContainer face_cont;
//		v4* hull_end = ConvexHull3D(&verts[0], &verts[0] + vert_count, face_cont);
//		impl::DumpVerts(&verts[0], hull_end);
//	}//Run()
//}//namespace TestConvexHull

//namespace TestPerlinNoise
//{
//	void Run()
//	{
//		const uint Width	= 256;
//		const uint Height	= 256;
//
//		image::Context ctx = image::Context::make(GetConsoleWindow());
//
//		image::ImageInfo info;
//		info.Width				= Width;
//		info.Height				= Height;
//		info.Format				= D3DFMT_L8;
//		info.MipLevels			= 2;
//		info.ImageFileFormat	= D3DXIFF_BMP;
//		info.m_filename			= "C:/DeleteMe.bmp";
//		
//		image::Image2D img = image::Create2DImage(ctx, info);
//
//		PerlinNoiseGenerator perlin(42);
//
//		float freq = 32.0f;
//		float amp  = 0.8f;
//		float offset = 0.5f;
//		{
//			image::Lock lock;
//			image::Image2D::iterator iter = img.Lock(lock, 0);
//			for( uint x = 0; x < Width; ++x )
//			{
//				for( uint y = 0; y < Height; ++y )
//				{
//					float value = perlin.Noise(freq * x / Width, freq * y / Height, 0.0) * amp + offset;
//					iter(x, y)	= (BYTE)Clamp<float>(value * 0xFF, 0.0, 0xFF);
//				}
//			}
//		}
//		{
//			image::Lock lock;
//			image::Image2D::iterator iter = img.Lock(lock, 1);
//			while( iter.IsValid() )
//			{
//				*iter = 0x80;
//				++iter;
//			}
//		}
//		image::Save2DImage(img);
//	}//Run()
//}// namespace TestPerlinNoise

namespace TestMaths
{
	void Run()
	{
		//TestConvexDecomposition::Run();
		//TestTriangulate::Run();
		//TestConvexHull::Run();
		//TestPerlinNoise::Run();
	}
}//namespace TestMaths




//// Recursively partition the mesh, when a convex piece is found, add it to 'polytopes'
//void Impl::ConvexDecompose(TEdges& edges)
//{
//	LDR_OUTPUT(PR_DBG_CVX_DECOM, DumpEdges(m_mesh, edges);)
//
//	// Classify the edges into sets
//	TNbrs perimeter_verts;
//	std::size_t num_edges = edges.size();
//	for( std::size_t e = 0; e != num_edges; ++e )
//	{
//		Edge& edge = edges[e];
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, StartFile("C:/Deleteme/Deformable_edge.pr_script");)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, ldr::Line("Edge", "FFFFFF00", GetVert(edge.m_i0), GetVert(edge.m_i1));)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, EndFile();)
//
//		// If the edge lies in the plane...
//		else if( start_on_plane && end_on_plane )
//		{
//			// Display the perimeter edges
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, AppendFile("C:/Deleteme/Deformable_perimedges.pr_script");)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, ldr::Line("PerimeterEdge", "FFFFFFFF", GetVert(edge.m_i0), GetVert(edge.m_i1));)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, EndFile();)
//
//			// Save the indices for the verts that lie in the plane
//			TNbrs::iterator iter;
//			iter = std::lower_bound(perimeter_verts.begin(), perimeter_verts.end(), edge.m_i0);
//			if( iter == perimeter_verts.end() || *iter != edge.m_i0 ) perimeter_verts.insert(iter, edge.m_i0);
//			iter = std::lower_bound(perimeter_verts.begin(), perimeter_verts.end(), edge.m_i1);
//			if( iter == perimeter_verts.end() || *iter != edge.m_i1 ) perimeter_verts.insert(iter, edge.m_i1);
//
//			edges.erase(edges.begin() + e);
//			--e; num_edges = edges.size();
//		}
//
//		// If the start or end of the line does not lie in the plane, look for an earlier edge 
//		// that shares a vertex and add it to that set, otherwise begin a new set
//		else if( !start_on_plane || !end_on_plane )
//		{
//			Edge const* adjoining_start_edge = !start_on_plane ? FindEdgeSharingVert(edges, 0, e, edge.m_i0) : 0;
//			Edge const* adjoining_end_edge   = !end_on_plane   ? FindEdgeSharingVert(edges, 0, e, edge.m_i1) : 0;
//			
//			// If 'edge' connects two different sets, set all of the set ids to be the same
//			if( adjoining_start_edge && adjoining_end_edge && adjoining_start_edge->m_set_id != adjoining_end_edge->m_set_id )
//			{
//				for( std::size_t f = 0; f <= e; ++f )
//				{
//					Edge& earlier = edges[f];
//					if( earlier.m_set_id == adjoining_end_edge->m_set_id )
//						earlier.m_set_id = adjoining_start_edge->m_set_id;
//				}
//				edge.m_set_id = adjoining_start_edge->m_set_id;
//			}
//			else if( adjoining_start_edge )	{ edge.m_set_id = adjoining_start_edge->m_set_id; }
//			else if( adjoining_end_edge )	{ edge.m_set_id = adjoining_end_edge->m_set_id; }
//			else							{ edge.m_set_id = ++m_set_id; }
//		}
//
//		// Display the sets
//		#if PR_DBG_CVX_DECOM == 1
//		LDR_OUTPUT(PR_DBG_CVX_DECOM, StartFile("C:/Deleteme/Deformable_edgeset.pr_script");)
//		for( std::size_t f = 0; f <= e; ++f )
//		{
//			Edge const& edge = edges[f];
//			unsigned int colour = 0xFF000000 | ((0xFF * edge.m_set_id / 4) << 8);
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, ldr::Line("Edge", FmtS("%X", colour), GetVert(edge.m_i0), GetVert(edge.m_i1));)
//		}
//		LDR_OUTPUT(PR_DBG_CVX_DECOM, EndFile();)
//		#endif//PR_DBG_CVX_DECOM==1
//	}
//
//	// Sort the edges by set id
//	std::sort(edges.begin(), edges.end(), BySetId());
//
//	// Call ConvexDecompose on each set
//	Edge divider;
//	divider.set(0,0);
//	divider.m_set_id = edges.begin()->m_set_id + 1;
//	TEdges::iterator set_begin = edges.begin();
//	TEdges::iterator set_end   = std::lower_bound(set_begin, edges.end(), divider, BySetId());
//	while( set_begin != set_end )
//	{
//		TEdges set(set_begin, set_end);
//
//		// Make the set a closed mesh
//		CloseMesh(set, perimeter_verts, split_plane);
//
//		// The set of edges forms a new mesh, call ConvexDecompose on the set
//		ConvexDecompose(set);
//
//			// Redraw the split plane so we know what recursion level we're at
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, StartFile("C:/Deleteme/Deformable_splitplane.pr_script");)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, ldr::Plane("SplitPlane", "800080FF", split_plane, 5.0f);)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, EndFile();)
//
//		divider.m_set_id	= set_end != edges.end() ? set_end->m_set_id + 1 : divider.m_set_id + 1;
//		set_begin			= set_end;
//		set_end				= std::lower_bound(set_begin, edges.end(), divider, BySetId());
//	}
//}
//
//// Find an edge that has a 'common_vert_idx' as one of it's vertices
//Edge const* Impl::FindEdgeSharingVert(TEdges const& edges, std::size_t begin, std::size_t end, std::size_t common_vert_idx)
//{
//	for( ; begin != end; --end )
//	{
//		Edge const& earlier = edges[end - 1];
//		if( earlier.m_i0 == common_vert_idx || earlier.m_i1 == common_vert_idx )
//			return &earlier;
//	}
//	return 0;
//}
//
//// Close the mesh 'set' by generating a polygon for the cut made by the split plane.
//// The polygon should be closed but will need triangulating
//void Impl::CloseMesh(TEdges& set, TNbrs const& perimeter_verts, Plane const& plane)
//{
//	// Create a closed polygon
//	TEdges polygon;
//	GeneratePolygon(set, perimeter_verts, polygon);
//
//	// Turn the polygon into a set of triangles
//	TriangulatePolygon(polygon, plane);
//}
//
//// Create a closed polygon for the cut made through the mesh by 'split_plane'
//// Note, we cannot use the edges that lie in the plane (and are dropped when partitioning
//// the mesh) because in general we cannot tell which set these edges are connected to. 
//void Impl::GeneratePolygon(TEdges const& set, TNbrs const& perimeter_verts, TEdges& polygon)
//{
//	LDR_OUTPUT(PR_DBG_CVX_DECOM, StartFile("C:/Deleteme/Deformable_perimedges.pr_script");EndFile();)
//
//	// Find an edge in 'set' where one of its vertices is a perimeter vertex
//	std::size_t start = InvalidVertIndex, perim = InvalidVertIndex;
//	for( TEdges::const_iterator e = set.begin(), e_end = set.end(); e != e_end; ++e )
//	{
//		bool i0_is_perimeter_vert = Con|tains(perimeter_verts, e->m_i0);
//		bool i1_is_perimeter_vert = Contains(perimeter_verts, e->m_i1);
//		if     ( !i0_is_perimeter_vert && i1_is_perimeter_vert )	{ start = e->m_i0; perim = e->m_i1; break; }
//		else if( !i1_is_perimeter_vert && i0_is_perimeter_vert )	{ start = e->m_i1; perim = e->m_i0; break; }
//	}
//	PR_ASSERT(PR_DBG_PHYSICS, start != InvalidVertIndex && perim != InvalidVertIndex);
//
//	// Using the other vertex as a starting point, find the nbr that represents this edge
//	TNbrs* nbrs = &m_mesh.m_vert[start].m_nbrs;
//	TNbrs::const_iterator n = std::find(nbrs->begin(), nbrs->end(), perim);		PR_ASSERT(PR_DBG_PHYSICS, n != nbrs->end());
//	std::size_t first = perim;
//	bool closed = false;
//	do
//	{
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, StartFile("C:/Deleteme/Deformable_edge.pr_script");)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, ldr::Line("Edge", "FFFFFF00", GetVert(start), GetVert(perim));)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, EndFile();)
//
//		// look at the next nbr, if this vertex is a perimeter vertex add a polygon edge
//		if( ++n == nbrs->end() ) n = nbrs->begin();
//		if( *n == first ) closed = true;
//		if( Contains(perimeter_verts, *n) )
//		{
//			Edge perim_edge;
//			perim_edge.m_i0 = perim;
//			perim_edge.m_i1 = *n;
//			polygon.push_back(perim_edge);
//			perim = *n;
//
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, AppendFile("C:/Deleteme/Deformable_perimedges.pr_script");)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, ldr::Line("PerimeterEdge", "FFFFFFFF", GetVert(perim_edge.m_i0), GetVert(perim_edge.m_i1));)
//			LDR_OUTPUT(PR_DBG_CVX_DECOM, EndFile();)
//		}
//		// If not, make this neighbour the starting point and find the nbr that represents this edge
//		else
//		{
//			start = *n;
//			nbrs = &m_mesh.m_vert[start].m_nbrs;
//			n = std::find(nbrs->begin(), nbrs->end(), perim);	PR_ASSERT(PR_DBG_PHYSICS, n != nbrs->end());
//		}
//	}
//	while( !closed );
//}
