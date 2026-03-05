//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/shape/shapepolytope.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/collision/contactmanifold.h"
#include "physics/utility/profile.h"

#define PR_PH_DBG_SUPVERT  0

using namespace pr;
using namespace pr::ph;

// Construct the shape
ShapePolytope& ShapePolytope::set(std::size_t vert_count, std::size_t face_count, std::size_t size_in_bytes, m4x4 const& shape_to_model, MaterialId material_id, uint32_t flags)
{
	m_base.set(EShape_Polytope, size_in_bytes, shape_to_model, material_id, flags);
	m_vert_count = static_cast<uint32_t>(vert_count);
	m_face_count = static_cast<uint32_t>(face_count);
	CalcBBox(*this, m_base.m_bbox);
	return *this;
}

// Return the volume of the polytope
float pr::ph::CalcVolume(ShapePolytope const& shape)
{
	float volume = 0;
	for (ShapePolyFace const* f = shape.face_begin(), *f_end = shape.face_end(); f != f_end; ++f)
	{
		v4 a = shape.vertex(f->m_index[0]);
		v4 b = shape.vertex(f->m_index[1]);
		v4 c = shape.vertex(f->m_index[2]);
		volume += Triple(a, b, c); // Triple product is volume x 6
	}
	if (volume < maths::tiny<float>)
	{
		PR_INFO(PR_DBG_PHYSICS, FmtS("PRPhysics: Shape %s with volume = %f\n", GetShapeTypeStr(shape.m_base.m_type), volume));
		volume = maths::tiny<float>;
	}
	return volume / 6.0f;
}

// Return the centre of mass position of the polytope
v4 pr::ph::CalcCentreOfMass(ShapePolytope const& shape)
{
	float volume = 0;
	v4 centre_of_mass = v4::Zero();
	for (ShapePolyFace const* f = shape.face_begin(), *f_end = shape.face_end(); f != f_end; ++f)
	{
		v4 a = shape.vertex(f->m_index[0]);
		v4 b = shape.vertex(f->m_index[1]);
		v4 c = shape.vertex(f->m_index[2]);
		float vol_x6 = Triple(a, b, c); // Triple product is volume x 6
		centre_of_mass += vol_x6 * (a + b + c);	// Divide by 4 at end
		volume += vol_x6;
	}
	if (volume < maths::tiny<float>)
	{
		PR_INFO(PR_DBG_PHYSICS, FmtS("PRPhysics: Shape %s with volume = %f\n", GetShapeTypeStr(shape.m_base.m_type), volume));
		volume = Abs(volume + maths::tiny<float>);
	}
	centre_of_mass /= volume * 4.0f;
	centre_of_mass.w = 0.0f;	// 'centre_of_mass' is an offset from the current model origin
	return centre_of_mass;
}

// Shift the verts of the polytope so they are centred on a new position
void pr::ph::ShiftCentre(ShapePolytope& shape, v4& shift)
{
	PR_ASSERT(PR_DBG_PHYSICS, shift.w == 0.0f, "");
	if( FEql(shift,pr::v4::Zero()) ) return;
	for( v4 *v = shape.vert_begin(), *v_end = shape.vert_end(); v != v_end; ++v )
	    *v -= shift;
	shape.m_base.m_shape_to_model.pos += shift;
	shift = pr::v4::Zero();
}

// Return the bounding box for a polytope
BBox& pr::ph::CalcBBox(ShapePolytope const& shape, BBox& bbox)
{
	bbox.reset();
	for( v4 const *v = shape.vert_begin(), *v_end = shape.vert_end(); v != v_end; ++v )
	    Grow(bbox, *v);
	return bbox;
}

// Return the inertia tensor for the polytope.
// Note: The polytope must be in the correct space before calculating its inertia
// (i.e. at the centre of mass, or not). Calculating the inertia then translating
// it does not give the same result (with this code at least).
m3x4 pr::ph::CalcInertiaTensor(ShapePolytope const& shape)
{
	// Assume mass == 1.0, you can multiply by mass later.
	// For improved accuracy the next 3 variables, the determinant vol, and its calculation should be changed to double
	float	volume = 0;		// Technically this variable accumulates the volume times 6
	v4		diagonal_integrals = v4::Zero();	// Accumulate matrix main diagonal integrals [x*x, y*y, z*z]
	v4		off_diagonal_integrals = v4::Zero();	// Accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]
	for (ShapePolyFace const* f = shape.face_begin(), *f_end = shape.face_end(); f != f_end; ++f)
	{
		v4 a = shape.vertex(f->m_index[0]);
		v4 b = shape.vertex(f->m_index[1]);
		v4 c = shape.vertex(f->m_index[2]);
		float vol_x6 = Triple(a, b, c); // Triple product is volume x 6
		volume += vol_x6;
		for (int i = 0, j = 1, k = 2; i != 3; ++i, (++j) %= 3, (++k) %= 3)
		{
			diagonal_integrals[i] += (
				a[i] * b[i] +
				b[i] * c[i] +
				c[i] * a[i] +
				a[i] * a[i] +
				b[i] * b[i] +
				c[i] * c[i]
				) * vol_x6; // Divide by 60.0f later
			off_diagonal_integrals[i] += (
				a[j] * b[k] +
				b[j] * c[k] +
				c[j] * a[k] +
				a[j] * c[k] +
				b[j] * a[k] +
				c[j] * b[k] +
				a[j] * a[k] * 2.0f +
				b[j] * b[k] * 2.0f +
				c[j] * c[k] * 2.0f
				) * vol_x6; // Divide by 120.0f later
		}
	}
	if (volume < maths::tiny<float>)
	{
		PR_INFO(PR_DBG_PHYSICS, FmtS("PRPhysics: Shape %s with volume = %f\n", GetShapeTypeStr(shape.m_base.m_type), volume));
		volume = maths::tiny<float>;
	}

	volume /= 6.0f;
	diagonal_integrals /= volume * 60.0f;  // Divide by total volume
	off_diagonal_integrals /= volume * 120.0f;
	return m3x4(
		v4(diagonal_integrals.y + diagonal_integrals.z, -off_diagonal_integrals.z, -off_diagonal_integrals.y, 0),
		v4(-off_diagonal_integrals.z, diagonal_integrals.x + diagonal_integrals.z, -off_diagonal_integrals.x, 0),
		v4(-off_diagonal_integrals.y, -off_diagonal_integrals.x, diagonal_integrals.x + diagonal_integrals.y, 0));
}

// Return mass properties for the polytope
MassProperties& pr::ph::CalcMassProperties(ShapePolytope const& shape, float density, MassProperties& mp)
{
	mp.m_centre_of_mass		= CalcCentreOfMass(shape);
	mp.m_mass				= CalcVolume(shape) * density;
	mp.m_os_inertia_tensor	= CalcInertiaTensor(shape);
	return mp;
}

// Return a support vertex for a polytope
v4 pr::ph::SupportVertex(ShapePolytope const& shape, v4 direction, std::size_t hint_vert_id, std::size_t& sup_vert_id)
{
	PR_DECLARE_PROFILE(PR_PROFILE_SUPPORT_VERTS, phSupVertPoly);
	PR_PROFILE_SCOPE(PR_PROFILE_SUPPORT_VERTS, phSupVertPoly);

	PR_ASSERT(PR_DBG_PHYSICS, hint_vert_id < shape.m_vert_count, "Invalid hint vertex index");
	PR_ASSERT(PR_DBG_PHYSICS, Length(direction) > maths::tiny<float>, "Direction is too short");
		PR_EXPAND(PR_PH_DBG_SUPVERT, StartFile("C:/DeleteMe/collision_supverttrace.pr_script");)
		PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::PhShape("polytope", "8000FF00", shape, m4x4::Identity());)
		PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Line("sup_direction", "FFFFFF00", v4::Origin(), direction);)
		PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::GroupStart("SupportVertexTrace");)

	// Find the support vertex using a 'hill-climbing' search
	// Start at the hint vertex and look for a neighbour that is more extreme in the
	// support direction. When no neighbours are closer we've found the support vertex
	sup_vert_id = (hint_vert_id < shape.m_vert_count) * hint_vert_id; // Make sure we don't get an invalid id to start with
	v4 const* support_vertex = shape.vert_begin() + sup_vert_id;
	v4 const* nearest_vertex;
	float	  sup_dist = Dot3(*support_vertex, direction);
	bool	  use_first_nbr = true;
	PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Box("start", "FF00FFFF", *support_vertex, 0.05f);)
	do
	{
		nearest_vertex = support_vertex;
		ShapePolyNbrs const& nbrhdr = shape.nbr(sup_vert_id);
		for( uint8_t const *n = nbrhdr.begin() + !use_first_nbr, *n_end = nbrhdr.end(); n != n_end; ++n )
		{
			// There are two possible ways we can do this, either by moving to the
			// first neighbour that is more extreme or by testing all neighbours.
			// The disadvantages are searching a non-optimal path to the support
			// vertex or searching excessive neighbours respectively.
			// Test in batches of 4 as a trade off
			if( use_first_nbr || n_end - n < 4 )
			{
	 			use_first_nbr = false;
				float dist = Dot3(shape.vertex(*n), direction);
				if( dist > sup_dist + maths::tiny<float> )
				{
					sup_vert_id	   = *n;
					sup_dist	   = dist;
					support_vertex = shape.vert_begin() + sup_vert_id;
					PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Line("to", "FF0000FF", *nearest_vertex, *support_vertex);)
					PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Box("v", "FF0000FF", *support_vertex, 0.05f);)
					break;
				}
			}
			else
			{
				m4x4 nbrs;
				nbrs.x = shape.vertex(*(n    ));
				nbrs.y = shape.vertex(*(n + 1));
				nbrs.z = shape.vertex(*(n + 2));
				nbrs.w = shape.vertex(*(n + 3));
				nbrs = Transpose(nbrs);
				v4 dots = nbrs * direction;

				std::size_t id = sup_vert_id;
				if( dots.x > sup_dist )		{ sup_dist = dots.x; sup_vert_id = *(n + 0); }
				if( dots.y > sup_dist )		{ sup_dist = dots.y; sup_vert_id = *(n + 1); }
				if( dots.z > sup_dist )		{ sup_dist = dots.z; sup_vert_id = *(n + 2); }
				if( dots.w > sup_dist )		{ sup_dist = dots.w; sup_vert_id = *(n + 3); }
				if( sup_vert_id != id )
				{
					support_vertex = shape.vert_begin() + sup_vert_id;
					PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Line("to", "FF0000FF", *nearest_vertex, *support_vertex);)
					PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Box("v", "FF0000FF", *support_vertex, 0.05f);)
					break;
				}
				n += 3;
			}
		}
	}
	while( support_vertex != nearest_vertex );
	PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::GroupEnd();)
	PR_EXPAND(PR_PH_DBG_SUPVERT, EndFile();)

	//// Check that we've found the most extreme vertex
	//#if PR_DBG_PHYSICS == 1
	//v4 const* sup_vert = support_vertex;
	//for( v4 const *v = shape.vert_begin(), *v_end = shape.vert_end(); v != v_end; ++v )
	//	if( Dot3(*v - *sup_vert, direction) > maths::tiny<float> )
	//		sup_vert = v;
	//PR_ASSERT(PR_DBG_PHYSICS, support_vertex == sup_vert);
	//#endif//PR_DBG_PHYSICS == 1
	return *support_vertex;
}

// Returns the longest/shortest axis of a polytope in 'direction' (in polytope space)
// Searching starts at 'hint_vert_id'. The spanning vertices are 'vert_id0' and 'vert_id1'
// 'major' is true for the longest axis, false for the shortest axis
void pr::ph::GetAxis(ShapePolytope const& shape, v4& direction, std::size_t hint_vert_id, std::size_t& vert_id0, std::size_t& vert_id1, bool major)
{
	PR_ASSERT(PR_DBG_PHYSICS, hint_vert_id < shape.m_vert_count, "");

	float eps = major ? maths::tiny<float> : -maths::tiny<float>;
	vert_id0 = hint_vert_id;
	v4 const* V1 = shape.vert_begin() + (vert_id0);
	v4 const* V2 = shape.vert_begin() + (*shape.nbr(vert_id0).begin());	// The first neighbour is always the most distant
	direction = *V1 - *V2;
	float span_lenSq = LengthSq(direction);
	do
	{
		hint_vert_id = vert_id0;

		// Look for a neighbour with a longer span
		ShapePolyNbrs const& nbr = shape.nbr(vert_id0);
		for( uint8_t const *n = nbr.begin() + 1, *n_end = nbr.end(); n < n_end; ++n )
		{
			v4 const* v1 = shape.vert_begin() + (*n);
			v4 const* v2 = shape.vert_begin() + (*shape.nbr(*n).begin());
			v4 span = *v1 - *v2;
			float lenSq = LengthSq(span);
			if( (lenSq > span_lenSq + eps) == major )
			{
				span_lenSq = lenSq;
				direction = span;
				vert_id0 = *n;
				break;
			}
		}
	}
	while( hint_vert_id != vert_id0 );
	vert_id1 = *shape.nbr(vert_id0).begin();
}

// Return the number of vertices in a polytope
uint32_t pr::ph::VertCount(ShapePolytope const& shape)
{
	return shape.m_vert_count;
}

// Return the number of edges in a polytope
uint32_t pr::ph::EdgeCount(ShapePolytope const& shape)
{
	// The number of edges in the polytope is the number of
	// neighbours minus the artificial neighbours over 2.
	uint32_t nbr_count = 0;
	for( ShapePolyNbrs const* n = shape.nbr_begin(), *n_end = shape.nbr_end(); n != n_end; ++n )
		nbr_count += n->m_count;
	return (nbr_count - shape.m_vert_count) / 2;
}

// Return the number of faces in a polytope
uint32_t pr::ph::FaceCount(ShapePolytope const& shape)
{
	// Use Euler's formula: F - E + V = 2. => F = 2 + E - V
	return 2 + EdgeCount(shape) - shape.m_vert_count;
}

// Generate the verts of a polytope. 'verts' should point to a buffer of v4's with
// a length equal to the value returned from 'VertCount'
void pr::ph::GenerateVerts(ShapePolytope const& shape, v4* verts, v4* verts_end)
{
	PR_ASSERT(PR_DBG_PHYSICS, uint32_t(verts_end - verts) >= VertCount(shape), "Vert buffer too small"); (void)verts_end;
	memcpy(verts, shape.vert_begin(), sizeof(v4) * shape.m_vert_count);
}

// Generate the edges of a polytope from the verts and their neighbours. 'edges' should
// point to a buffer of 2*the number of edges returned from 'EdgeCount'
void pr::ph::GenerateEdges(ShapePolytope const& shape, v4* edges, v4* edges_end)
{
	PR_ASSERT(PR_DBG_PHYSICS, uint32_t(edges_end - edges) >= 2 * EdgeCount(shape), "Edge buffer too small");

	uint32_t vert_index = 0;
	uint32_t nbr_index = 1;

	ShapePolyNbrs const* nbrs = &shape.nbr(vert_index);
	while( vert_index < shape.m_vert_count && edges + 2 <= edges_end )
	{
		*edges++ = shape.vertex(vert_index);
		*edges++ = shape.vertex(nbrs->begin()[nbr_index]);

		// Increment 'vert_index' and 'nbr_index' to refer
		// to the next edge. Only consider edges for which
		// the neighbouring vertex has a higher value. This
		// ensures we only add each edge once.
		do
		{
			if( ++nbr_index == nbrs->m_count )
			{
				if( ++vert_index == shape.m_vert_count )
					break;

				nbrs = &shape.nbr(vert_index);
				nbr_index = 1;
			}
		}
		while( nbrs->begin()[nbr_index] < vert_index );
	}
}

namespace
{
	struct Edge	{ uint32_t m_i0, m_i1; };
	inline bool operator == (Edge const& lhs, Edge const& rhs) { return (lhs.m_i0 == rhs.m_i0 && lhs.m_i1 == rhs.m_i1) || (lhs.m_i0 == rhs.m_i1 && lhs.m_i1 == rhs.m_i0); }
}//namespace polytope

// Generate faces for a polytope from the verts and their neighbours.
void pr::ph::GenerateFaces(ShapePolytope const& shape, uint32_t* faces, uint32_t* faces_end)
{
	// Helper object to fill the remaining faces with degenerates.
	// Since the verts of the polytope may not all be on the convex hull we may
	// generate less faces than 'faces_end - faces'
	struct FillRemaining
	{
		uint32_t *&m_faces, *m_faces_end;
		FillRemaining& operator = (FillRemaining const&) {return *this;}// no copying
		FillRemaining(uint32_t*& faces, uint32_t* faces_end) : m_faces(faces), m_faces_end(faces_end)
		{
			if( m_faces != m_faces_end )
				*m_faces = 0;
		}
		~FillRemaining() // Fill the remaining faces with degenerates
		{
			while( m_faces != m_faces_end )
				*m_faces++ = 0;
		}
	} fill_remaining(faces, faces_end);

	// Record the start address
	uint32_t* faces_start = faces;

	// Create the starting faces and handle cases for polygons with less than 3 verts
	for( uint32_t i = 0; i != 3; ++i )
	{
		if( faces == faces_end || i == shape.m_vert_count ) return;
		*faces++ = i;
	}
	for( uint32_t i = 3; i-- != 0; )
	{
		if( faces == faces_end ) return;
		*faces++ = i;
	}

	uint32_t const edge_stack_size = 50;
	pr::Stack<Edge, edge_stack_size> edges;

	// Generate the convex hull
	for( uint32_t i = 3; i != shape.m_vert_count; ++i )
	{
		v4 v = shape.vertex(i);
		for( uint32_t* f = faces_start, *f_end = faces; f != f_end; f += 3 )
		{
			v4 a = shape.vertex(*(f + 0));
			v4 b = shape.vertex(*(f + 1));
			v4 c = shape.vertex(*(f + 2));

			// If 'v' is in front of this face add its edges to the edge stack and remove the face
			if( Triple(v - a, b - a, c - a) >= 0.0f )
			{
				// Add the edges of this face to the edge stack (remove duplicates)
				Edge ed = {*(f + 2), *(f + 0)};
				for( uint32_t j = 0; j != 3; ++j, ed.m_i0 = ed.m_i1, ed.m_i1 = *(f + j) )
				{
					// Look for this edge in the stack
					Edge* e = edges.begin();
					for( ; e != edges.end() && !(*e == ed); ++e ) {}
					if( e == edges.end() )	edges.push(ed);		// Add the unique edge
					else					*e = edges.pop();	// Erase the duplicate
					if( edges.size() == edge_stack_size )
					{
						PR_ASSERT(PR_DBG_PHYSICS, false, "Edge stack not big enough. GenerateFaces aborted early");
						return;
					}
				}

				// Remove the face
				faces -= 3;
				*(f + 0) = *(faces + 0);
				*(f + 1) = *(faces + 1);
				*(f + 2) = *(faces + 2);
				f -= 3;
				f_end -= 3;
			}
		}

		// Add new faces for any edges that are in the edge stack
		while( !edges.empty() )
		{
			if( faces + 3 > faces_end ) return;
			Edge e = edges.pop();
			*faces++ = i;
			*faces++ = e.m_i0;
			*faces++ = e.m_i1;
		}
	}
}

// Remove the face data from a polytope
void pr::ph::StripFaces(ShapePolytope& shape)
{
	if( shape.m_face_count == 0 )
		return;

	uint8_t* base = reinterpret_cast<uint8_t*>(&shape);
	uint8_t* src  = reinterpret_cast<uint8_t*>(&shape.nbr(0));
	uint8_t* dst  = reinterpret_cast<uint8_t*>( shape.face_begin());
	std::size_t size = shape.m_base.m_size;

	std::size_t bytes_to_move = size - (src - base);
	std::size_t bytes_removed = shape.m_face_count * sizeof(ShapePolyFace);

	// Move the remainder of the polytope data back over the face data.
	memmove(dst, src, bytes_to_move);
	shape.m_base.m_size -= bytes_removed;
	shape.m_face_count = 0;
}

// Validate a polytope. Always returns true but performs assert checks
bool pr::ph::Validate(ShapePolytope const& shape, bool check_com)
{
	shape;check_com;
	#if PR_DBG_PHYSICS
	uint32_t num_real_nbrs = 0;
	for( uint32_t i = 0; i != shape.m_vert_count; ++i )
	{
		// Check the neighbours of each vertex.
		ShapePolyNbrs const& nbrs = shape.nbr(i);

		// All polytope verts should have an artificial neighbour plus >0 real neighbours
		PR_ASSERT(PR_DBG_PHYSICS, nbrs.m_count > 1, "");

		// Count the number of real neighbours in the polytope
		num_real_nbrs += (nbrs.m_count - 1);

		// Check each neighbour
		for( PolyIdx const *j = nbrs.begin(); j != nbrs.end(); ++j )
		{
			// Check that the neighbour refers to a vert in the polytope
			PR_ASSERT(PR_DBG_PHYSICS, *j < shape.m_vert_count, "");

			// Check that the neighbour refers to a different vert in the polytope
			PR_ASSERT(PR_DBG_PHYSICS, *j != i, "");

			// Check that there is a neighbour in both directions between 'i' and 'j'
			ShapePolyNbrs const& nbr_nbrs = shape.nbr(*j);
			bool found = j == nbrs.begin();// artificial neighbours don't point back
			for( PolyIdx const *k = nbr_nbrs.begin(); k != nbr_nbrs.end() && !found; ++k )
			{	found = *k == i; }
			PR_ASSERT(PR_DBG_PHYSICS, found, "");

			// Check that all neighbours (apart from the artificial neighbour) are unique
			if( j != nbrs.begin() )
			{
				for( PolyIdx const* k = j + 1; k != nbrs.end(); ++k )
				{	PR_ASSERT(PR_DBG_PHYSICS, *k != *j, ""); }
			}
		}
	}

	// Check the polytope describes a closed polyhedron
	if( shape.m_face_count != 0 )
	{
		PR_ASSERT(PR_DBG_PHYSICS, shape.m_face_count - (num_real_nbrs / 2) + shape.m_vert_count == 2,
			"The polytope is not a closed polyhedron!");
	}

	// Check the polytope is in centre of mass frame
	if( check_com )
	{
		//m3x4 inertia = CalcInertiaTensor(shape);
	}

	#endif//PR_DBG_PHYSICS
	return true;
}
