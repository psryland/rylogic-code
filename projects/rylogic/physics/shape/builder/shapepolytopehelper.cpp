//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/builder/shapepolytopehelper.h"
#include "pr/physics/shape/shapepolytope.h"
#include "pr/common/cast.h"
#include "pr/container/vector.h"
#include "pr/geometry/convex_hull.h"

//#define PH_SHAPE_POLYTOPE_LDR_OUTPUT 1
#ifdef PH_SHAPE_POLYTOPE_LDR_OUTPUT
#pragma message(__FILE__"(12) : PH_SHAPE_POLYTOPE_LDR_OUTPUT is defined")
#endif

using namespace pr;
using namespace pr::ph;

struct ShapePolytopeNbrsEx
{
	pr::vector<PolyIdx> m_nbr;
	v4                 m_normal;

	ShapePolytopeNbrsEx() :m_normal(v4::Zero()) {}
};

namespace pr
{
	namespace hull
	{
		inline void SetFace(ShapePolyFace& face, uint32_t a, uint32_t b, uint32_t c)
		{
			face.m_index[0] = s_cast<uint8_t>(a);
			face.m_index[1] = s_cast<uint8_t>(b);
			face.m_index[2] = s_cast<uint8_t>(c);
			face.pad = 0;
		}
		inline void GetFace(ShapePolyFace const& face, uint32_t& a, uint32_t& b, uint32_t& c)
		{
			a = face.m_index[0];
			b = face.m_index[1];
			c = face.m_index[2];
		}
	}
}

// Use the faces to generate neighbour indices for each vertex
void GenerateNeighbours(v4 const* verts, std::size_t num_verts, ShapePolyFace const* faces, std::size_t num_faces, pr::vector<ShapePolytopeNbrsEx>& neighbours)
{
	neighbours.resize(num_verts);
	for( ShapePolyFace const* f = faces, *f_end = faces + num_faces; f != f_end; ++f )
	{
		// Calculate the face normal
		v4 edge0 = verts[f->m_index[1]] - verts[f->m_index[0]];
		v4 edge1 = verts[f->m_index[2]] - verts[f->m_index[0]];
		v4 norm  = Normalise(Cross(edge0, edge1), v4::Zero());

		// For each vertex in each face, add the other face vertices as neighbours
		for( std::size_t i = 0, j = 1, k = 2; i != 3; ++i, j=(j+1)%3, k=(k+1)%3 )
		{
			// Add neighbour data
			ShapePolytopeNbrsEx& nbrhdr = neighbours[f->m_index[i]];
			PR_ASSERT(PR_DBG_PHYSICS, f->m_index[i] != f->m_index[j], "A vertex cannot be a neighbour of itself");
			PR_ASSERT(PR_DBG_PHYSICS, f->m_index[i] != f->m_index[k], "A vertex cannot be a neighbour of itself");
			if( std::find(nbrhdr.m_nbr.begin(), nbrhdr.m_nbr.end(), f->m_index[j]) == nbrhdr.m_nbr.end() ) { nbrhdr.m_nbr.push_back(f->m_index[j]); }
			if( std::find(nbrhdr.m_nbr.begin(), nbrhdr.m_nbr.end(), f->m_index[k]) == nbrhdr.m_nbr.end() ) { nbrhdr.m_nbr.push_back(f->m_index[k]); }
			nbrhdr.m_normal += norm;
		}
	}
}

// Generate an artificial neighbour for each vertex
// Returns the total number of neighbours in 'nbr_count'
void GenerateArtificialNeighbours(v4 const* verts, std::size_t num_verts, pr::vector<ShapePolytopeNbrsEx>& neighbours, std::size_t& nbr_count)
{
	// For each vertex, find the vertex on the opposite side of the polytope and set this as an 'artificial' neighbour
	for( std::size_t i = 0; i != num_verts; ++i )
	{
		ShapePolytopeNbrsEx& nbrhdr = neighbours[i];
		if( nbrhdr.m_nbr.empty() ) continue;

		// Flip the normal so it points inward
		nbrhdr.m_normal = -nbrhdr.m_normal;
		nbrhdr.m_normal.w = 0.0f;
		nbrhdr.m_normal = Normalise(nbrhdr.m_normal);

		// Find the most extreme vert
		v4 const* vert = verts;
		for( v4 const* v = verts + 1, *v_end = verts + num_verts; v < v_end; ++v )
		{
			if( Dot3(*v - *vert, nbrhdr.m_normal) > 0.0f )
				vert = &*v;
		}
		PolyIdx idx = static_cast<PolyIdx>(vert - verts);
		PR_ASSERT(PR_DBG_PHYSICS, idx != i, "A vertex cannot be a neighbour of itself");

		// Add the artificial neighbour
		if( std::find(nbrhdr.m_nbr.begin(), nbrhdr.m_nbr.end(), idx) == nbrhdr.m_nbr.end() )
		{
			// Add the artificial neighbour to the front so that it is considered first
			nbrhdr.m_nbr.insert(nbrhdr.m_nbr.begin(), idx);
		}

		// Count the total number of neighbours
		nbr_count += nbrhdr.m_nbr.size();
	}
}

// Serialise the polytope data into 'data'
ShapePolytope& Serialise(ShapePolytopeHelper& helper
						 ,v4 const* verts					,std::size_t vert_count
						 ,ShapePolyFace const* faces		,std::size_t face_count
						 ,ShapePolytopeNbrsEx const* neighbours	,std::size_t total_nbr_count
						 ,m4x4 const&	shape_to_model
						 ,MaterialId	material_id
						 ,uint32_t			flags)
{
	helper.m_data.resize(
		sizeof(ShapePolytope) +
		vert_count*sizeof(v4) +
		face_count*sizeof(ShapePolyFace) +
		vert_count*sizeof(ShapePolyNbrs) +
		total_nbr_count*sizeof(PolyIdx));

	ShapePolytope& poly = helper.get();
	poly.set(vert_count, face_count, helper.m_data.size(), shape_to_model, material_id, flags);

	// Add the verts
	uint8_t* ptr = reinterpret_cast<uint8_t*>(&helper.m_data[sizeof(ShapePolytope)]);
	memcpy(ptr, verts, vert_count*sizeof(v4));
	ptr += vert_count*sizeof(v4);

	// Add the faces
	memcpy(ptr, faces, face_count*sizeof(ShapePolyFace));
	ptr += face_count*sizeof(ShapePolyFace);

	// Add the neighbour headers
	ShapePolyNbrs*& nbrhdr = reinterpret_cast<ShapePolyNbrs*&>(ptr);
	uint16_t byte_offset = static_cast<uint16_t>(vert_count*sizeof(ShapePolyNbrs));
	for( std::size_t i = 0; i != vert_count; ++i )
	{
		nbrhdr->m_count = static_cast<uint16_t>(neighbours[i].m_nbr.size());
		nbrhdr->m_first = byte_offset;
		byte_offset -= sizeof(ShapePolyNbrs);
		byte_offset += nbrhdr->m_count*sizeof(PolyIdx);
		++nbrhdr;
	}

	// Add the neighbour data
	for( std::size_t i = 0; i != vert_count; ++i )
	{
		ShapePolytopeNbrsEx const& nbr = neighbours[i];
		PR_ASSERT(PR_DBG_PHYSICS, !nbr.m_nbr.empty(), "All vertices must have neighbours");
		memcpy(ptr, &nbr.m_nbr[0], nbr.m_nbr.size());
		ptr += nbr.m_nbr.size()*sizeof(PolyIdx);
	}

	// This is done when the shape is added to the shape builder
	//// Shift the polytope vertices to the centre of mass and adjust the shape to model transform
	//MassProperties mp;
	//CalcMassProperties(poly, 1.0f, mp);
	//ShiftCentre(poly, mp.m_centre_of_mass - v4::Origin());

	#if PH_SHAPE_POLYTOPE_LDR_OUTPUT == 1
	{
		PR_ASSERT(PR_DBG_PHYSICS, ptr - &helper.m_data[0] == (std::ptrdiff_t)helper.m_data.size());
		ldr::Polytope("poly", "FF00FF00", poly.vert_begin(), poly.vert_end());

		// Verify the polytope
		for( v4 const* v = poly.vert_begin(), *v_end = poly.vert_end(); v != v_end; ++v )
		{}
		for( ShapePolyFace const* f = poly.face_begin(), *f_end = poly.face_end(); f != f_end; ++f )
		{
			PR_ASSERT(PR_DBG_PHYSICS, f->m_index[0] < poly.m_vert_count);
			PR_ASSERT(PR_DBG_PHYSICS, f->m_index[1] < poly.m_vert_count);
			PR_ASSERT(PR_DBG_PHYSICS, f->m_index[2] < poly.m_vert_count);
		}
		for( std::size_t n = 0; n != poly.m_vert_count; ++n )
		{
			ldr::GroupStart(Fmt("Neighbours_%d", n).c_str());
			ldr::Box("nbr", "FFFF0000", poly.vertex(n), 0.04f);
			ShapePolyNbrs& nbr = poly.neighbour(n);
			for( PolyIdx const* i = nbr.begin(), *i_end = nbr.end(); i != i_end; ++i )
			{
				ldr::Box("nbr", "FFFFFF00", poly.vertex(*i), 0.03f);
				PR_ASSERT(PR_DBG_PHYSICS, *i < poly.m_vert_count);
			}
			ldr::GroupEnd();
		}
	}
	#endif//PH_SHAPE_POLYTOPE_LDR_OUTPUT == 1
	return poly;
}

// Use an array of verts to create a polytope.
ShapePolytope& ShapePolytopeHelper::set(v4 const* verts, std::size_t num_verts, m4x4 const& shape_to_model, MaterialId material_id, uint32_t flags)
{
	PR_ASSERT(PR_DBG_PHYSICS, num_verts >= 4, "");
	PR_ASSERT(1, false, "Step through me");

	std::size_t vert_count, face_count;

	// Generate faces by taking the convex hull of the verts
	pr::vector<uint32_t>			vindex(num_verts);
	pr::vector<ShapePolyFace>	faces(2 * (num_verts - 2));
	std::generate(vindex.begin(), vindex.end(), pr::ArithmeticSequence<uint32_t>(0,1));
	ConvexHull(verts, &vindex[0], &vindex[0] + vindex.size(), &faces[0], &faces[0] + faces.size(), vert_count, face_count);
	vindex.resize(vert_count);
	faces .resize(face_count);
	PR_ASSERT(PR_DBG_PHYSICS, vert_count <= limits<PolyIdx>::max(), "Polytope contains too many vertices");
	PR_ASSERT(PR_DBG_PHYSICS, face_count > 0, "Polytope has no faces");

	// Copy the verts
	pr::vector<v4> verts_copy(vert_count);
	pr::vector<v4>::iterator v_out = verts_copy.begin();
	for( pr::vector<uint32_t>::const_iterator i = vindex.begin(), i_end = vindex.end(); i != i_end; ++i, ++v_out )
	{
		*v_out = verts[*i];
	}

	// Generate neighbour info
	pr::vector<ShapePolytopeNbrsEx> neighbours;
	GenerateNeighbours(&verts_copy[0], vert_count, &faces[0], face_count, neighbours);

	// Add an artificial neighbour
	std::size_t nbr_count = 0;
	GenerateArtificialNeighbours(&verts_copy[0], vert_count, neighbours, nbr_count);

	// Create the polytope in 'm_data'.
	return Serialise(*this
		,&verts_copy[0] ,vert_count
		,&faces[0]      ,face_count
		,&neighbours[0] ,nbr_count
		,shape_to_model ,material_id ,flags);
}

// Use an array of verts and faces to create a polytope. Verts and faces must be convex
ShapePolytope& ShapePolytopeHelper::set(v4 const* verts, std::size_t num_verts, ShapePolyFace const* faces, std::size_t num_faces, m4x4 const& shape_to_model, MaterialId material_id, uint32_t flags)
{
	PR_ASSERT(PR_DBG_PHYSICS, num_verts >= 4, "");

	// Generate neighbour info
	pr::vector<ShapePolytopeNbrsEx> neighbours;
	GenerateNeighbours(verts, num_verts, faces, num_faces, neighbours);

	// Add an artificial neighbour
	std::size_t nbr_count = 0;
	GenerateArtificialNeighbours(verts, num_verts, neighbours, nbr_count);

	// Create the polytope in 'm_data'.
	return Serialise(*this
		,verts			,num_verts
		,faces			,num_faces
		,&neighbours[0] ,nbr_count
		,shape_to_model ,material_id ,flags);
}
