//*********************************************
// TetraMesh
//	copyright Paul Ryland May 2007
//*********************************************
//
//A "TetraMesh" in this file refers to a data structure containing a number of
// tetrahedra with face adjacency information. It's analogous to a triangle mesh
// with edge adjacency. The main functions in this file are:
// ConstrainVertexDisplacement - Allows vertices in the mesh to be moved without
//  turning any of the tetrahedral inside out.
// Decompose - Turns an arbitrary tetramesh into convex polytopes. These polytopes
//  are then suitable for polytope collision detection.
//
// Note: this software is confidential and property of Rylogic Limited and cannot
// be used without the express permission of the author.

#include "pr/container/chain.h"
#include "pr/container/byte_data.h"
#include "pr/geometry/tetramesh.h"
#include "pr/geometry/closest_point.h"

#if PR_DBG_GEOM_TETRAMESH == 1
	#include "pr/common/Fmt.h"
	#include "pr/common/Console.h"
	#pragma message("Q:/SDK/pr/pr/geometry/TetraMesh.h: PR_DBG_GEOM_TETRAMESH is defined")
#endif//PR_DBG_GEOM_TETRAMESH == 1

#if PR_LDR_TETRAMESH == 1
	#pragma message("Q:/SDK/pr/pr/geometry/TetraMesh.h: PR_LDR_TETRAMESH is defined")
	#include "pr/common/Fmt.h"
	#include "pr/common/Console.h"
	#include "pr/filesys/Autofile.h"
	#include "pr/linedrawer/ldr_helper.h"
	static bool pr_ldr_tetramesh_output_enable = true;
#endif//PR_LDR_TETRAMESH == 1

#if PR_PROFILE_TETRAMESH == 1
	#pragma message("Q:/SDK/pr/pr/geometry/TetraMesh.h: PR_PROFILE_TETRAMESH is defined")
#endif//PR_PROFILE_TETRAMESH == 1

namespace pr
{
	namespace tetramesh
	{
		struct Edge
		{
			tetramesh::VIndex	m_i0;			// The start vertex of the edge
			tetramesh::VIndex	m_i1;			// The end vertex of the edge
			tetramesh::Face		m_Lface;		// The face to the left of the edge
			tetramesh::Face		m_Rface;		// The face to the right of the edge
			float				m_concavity;	// A measure of the concavity of the edge. +ve=concave, -ve=convex
		};
		inline bool operator == (Edge const& lhs, Edge const& rhs) { return lhs.m_i0 == rhs.m_i0 && lhs.m_i1 == rhs.m_i1; }
		inline bool operator <  (Edge const& lhs, Edge const& rhs) { return lhs.m_i0 != rhs.m_i0 ? lhs.m_i0 < rhs.m_i0 : lhs.m_i1 < rhs.m_i1; }

		struct EdgeCache
		{
			enum { Size = 50 };
			EdgeCache()									{ clear(); }
			void		clear()							{ m_num_edges = 0; m_overflowed = false; }
			Edge const* begin() const					{ return m_edge; }
			Edge*		begin()							{ return m_edge; }
			Edge const* end() const						{ return m_edge + m_num_edges; }
			Edge*		end()							{ return m_edge + m_num_edges; }
			Edge const* find(Edge const& edge) const	{ return std::lower_bound(begin(), end(), edge); }
			void		insert(Edge const* iter, Edge const& edge);
			Edge	m_edge[Size];				// Cached concave edges
			TSize	m_num_edges;				// The number of edges in the cache
			bool	m_overflowed;				// True if the edge cache overflows and concave edges are dropped
		};

		bool	IsExternalFace		(tetramesh::Face const& face);
		bool	IsExternalFace		(tetramesh::Tetra const& tetra, CIndex nbr_idx);
		bool	IsBoundaryFace		(tetramesh::Mesh const& mesh, tetramesh::Face const& face);
		bool	IsBoundaryFace		(tetramesh::Mesh const& mesh, Tetra const& tetra, CIndex nbr_idx);
		int		Compare				(tetramesh::Mesh const& mesh, Tetra const& tetra, Plane const& plane, float tolerance);

		void	InitTetras			(tetramesh::Mesh& mesh, Tetra& consider);
		void	AddConcaveEdges		(tetramesh::Mesh const& mesh, Tetra const& tetra, CIndex nbr_idx, float convex_tolerance, EdgeCache& edge_cache);
		void	CacheConcaveEdges	(tetramesh::Mesh const& mesh, Tetra const& consider, float convex_tolerance, EdgeCache& edge_cache);
		void	MeasureConcavity	(tetramesh::Mesh const& mesh, Edge& edge);
		bool	FindSplitPlane		(tetramesh::Mesh const& mesh, EdgeCache const& edge_cache, float convex_tolerance, Plane& split_plane, TIndex& start);
		bool	FindMostConcaveEdge	(EdgeCache const& edge_cache, Edge& concave_edge);
		void	PartitionTetras		(tetramesh::Mesh& mesh, Tetra& polytope, Plane const& split_plane, float convex_tolerance, TIndex start, TSize search_id);
		void	UpdateEdgeCache		(tetramesh::Mesh const& mesh, Tetra const& polytope, Tetra const& consider, EdgeCache& edge_cache, float convex_tolerance);
		void	GeneratePolytopes	(tetramesh::Mesh& mesh, Tetra& polytope, IPolytopeGenerator& gen, TSize& next_poly_id, TSize search_id);
		void	GeneratePolytope	(tetramesh::Mesh& mesh, Tetra& polytope, IPolytopeGenerator& gen);

		// Debugging functions
		PR_EXPAND(PR_LDR_TETRAMESH, void DumpSplitPlane	(tetramesh::Mesh const& mesh, Plane const& split_plane, TIndex start, char const* colour, char const* filename);)
		PR_EXPAND(PR_LDR_TETRAMESH, void DumpSet		(tetramesh::Mesh const& mesh, Tetra const& tetras, float scale, char const* colour, char const* filename);)
		PR_EXPAND(PR_LDR_TETRAMESH, void DumpEdge		(tetramesh::Mesh const& mesh, Edge const& edge, char const* colour, char const* filename);)
		PR_EXPAND(PR_LDR_TETRAMESH, void DumpEdges		(tetramesh::Mesh const& mesh, EdgeCache const& edge_cache, char const* colour, char const* filename);)
	}//namespace tetramesh
}//namespace pr

using namespace pr;
using namespace pr::tetramesh;

// Inline implementation *****************************************

// Insert an edge into the edge cache
void EdgeCache::insert(Edge const* iter, Edge const& edge)
{
	// Do this instead of: Edge* nc_iter = begin() + (iter - begin());
	Edge* nc_iter = const_cast<Edge*>(iter);

	if( m_num_edges == EdgeCache::Size )
	{
		// If we add more edges than there's room then we'll be dropping some
		// of the concave edges. In this case we'll need to rescan for concave
		// edges when the cache is empty to be sure we got them all
		m_overflowed = true;

		// Find the least concave edge and, if 'edge' is more concave, erase it
		Edge const* least_concave = &edge;
		for( Edge const* e = begin(); e != end(); ++e )
		{
			if( e->m_concavity < least_concave->m_concavity )
				least_concave = e;
		}

		// If 'edge' is less concave than the edges already in the cache, then we don't need to add it
		if( least_concave == &edge )
			return;

		// Erase 'least_concave'
		memmove(const_cast<Edge*>(least_concave), least_concave + 1, (end() - (least_concave + 1)) * sizeof(Edge));
		--m_num_edges;
		if( nc_iter > least_concave ) --nc_iter;
	}

	// Insert 'edge'
	memmove(nc_iter + 1, nc_iter, (end() - nc_iter) * sizeof(Edge));
	*nc_iter = edge;
	++m_num_edges;
}

// Returns true if 'face' is an external face
inline bool pr::tetramesh::IsExternalFace(tetramesh::Face const& face)
{
	return face.m_tetra1 == ExtnFace;
}
inline bool pr::tetramesh::IsExternalFace(tetramesh::Tetra const& tetra, CIndex nbr_idx)
{
	return tetra.m_nbrs[nbr_idx] == ExtnFace;
}

// Returns true if 'face' or the neighbour at position 'nbr_idx' is on a boundary
inline bool pr::tetramesh::IsBoundaryFace(tetramesh::Mesh const& mesh, tetramesh::Face const& face)
{
	return IsExternalFace(face) || mesh.m_tetra[face.m_tetra0].m_poly_id != mesh.m_tetra[face.m_tetra1].m_poly_id;
}
inline bool pr::tetramesh::IsBoundaryFace(tetramesh::Mesh const& mesh, tetramesh::Tetra const& tetra, CIndex nbr_idx)
{
	return	IsExternalFace(tetra, nbr_idx) || mesh.m_tetra[tetra.m_nbrs[nbr_idx]].m_poly_id != tetra.m_poly_id;
}

// Compare a tetrahedron with a plane.
// Returns '1' if the tetrahedron is entirely in front of the plane,
// '0' if it spans the plane, and '-1' if it is behind the plane
inline int pr::tetramesh::Compare(tetramesh::Mesh const& mesh, tetramesh::Tetra const& tetra, Plane const& plane, float tolerance)
{
	m4x4 cnrs;
	cnrs.x = mesh.m_verts[tetra.m_cnrs[0]];
	cnrs.y = mesh.m_verts[tetra.m_cnrs[1]];
	cnrs.z = mesh.m_verts[tetra.m_cnrs[2]];
	cnrs.w = mesh.m_verts[tetra.m_cnrs[3]];
	Transpose4x4(cnrs);
	v4 dots = cnrs * plane;
	return	(dots.x >= -tolerance && dots.y >= -tolerance && dots.z >= -tolerance && dots.w >= -tolerance) -
			(dots.x <=  tolerance && dots.y <=  tolerance && dots.z <=  tolerance && dots.w <=  tolerance);
}

// Return the required size in bytes for a tetramesh with 'num_verts' and 'num_tetra'
std::size_t pr::tetramesh::Sizeof(std::size_t num_verts, std::size_t num_tetra)
{
	return sizeof(tetramesh::Mesh) + sizeof(v4) * num_verts + sizeof(Tetra) * num_tetra;
}

// Construct a tetramesh in the provided buffer
void pr::tetramesh::Create(tetramesh::Mesh& tmesh, v4 const* verts, std::size_t num_verts, VIndex const* tetra, std::size_t num_tetra)
{
	// Copy the verts
	memcpy(tmesh.m_verts, verts, sizeof(v4) * num_verts);

	// Copy the tetras and generate neighbours
	TFaces faces;
	for( Tetra* t = tmesh.m_tetra, *t_end = t + num_tetra; t != t_end; ++t )
	{
		t->m_cnrs[0] = *tetra++;
		t->m_cnrs[1] = *tetra++;
		t->m_cnrs[2] = *tetra++;
		t->m_cnrs[3] = *tetra++;
		t->m_nbrs[0] = ExtnFace;
		t->m_nbrs[1] = ExtnFace;
		t->m_nbrs[2] = ExtnFace;
		t->m_nbrs[3] = ExtnFace;

		// Add the faces
		for( CIndex n = 0; n != NumCnrs; ++n )
		{
			tetramesh::Face face = t->OppFace(n);
			face.m_order = GetFaceIndexOrder(face);
			face.m_tetra0 = static_cast<TIndex>(t - tmesh.m_tetra);
			face.m_tetra1 = ExtnFace;
			TFaces::iterator iter = std::lower_bound(faces.begin(), faces.end(), face);
			if( iter == faces.end() || !(*iter == face) )	faces.insert(iter, face);
			else											iter->m_tetra1 = face.m_tetra0;
		}
	}

	// Generate neighbour info from the faces
	for( TFaces::const_iterator i = faces.begin(), i_end = faces.end(); i != i_end; ++i )
	{
		tetramesh::Face const& face = *i;
		if( face.m_tetra0 != ExtnFace && face.m_tetra1 != ExtnFace )
		{
			Tetra& tetra0		= tmesh.m_tetra[face.m_tetra0];
			Tetra& tetra1		= tmesh.m_tetra[face.m_tetra1];
			CIndex nbr0			= tetra0.CnrIndex(tetra0.OppVIndex(face));
			CIndex nbr1			= tetra1.CnrIndex(tetra1.OppVIndex(face));
			tetra0.m_nbrs[nbr0] = face.m_tetra1;
			tetra1.m_nbrs[nbr1] = face.m_tetra0;
		}
	}

	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Validate(tmesh), "");
}

// A predicate that returns true if a face is an external face
struct Pred_ExternalFace
{
	bool operator ()(tetramesh::Face const& face) const	{ return IsExternalFace(face); }
};

// A predicate that returns true if a face is a boundary face
struct Pred_BoundaryFace
{
	tetramesh::Mesh const* m_mesh;
	Pred_BoundaryFace(tetramesh::Mesh const& mesh) : m_mesh(&mesh) {}
	bool operator ()(tetramesh::Face const& face) const	{ return IsBoundaryFace(*m_mesh, face); }
};

// Decompose **************************************
// Decompose a tetrahedral mesh into convex polytopes
void pr::tetramesh::Decompose(tetramesh::Mesh& mesh, IPolytopeGenerator& gen, float convex_tolerance)
{
	// Validate the mesh
	PR_ASSERT(PR_LDR_TETRAMESH, Validate(mesh), "");
	PR_EXPAND(PR_LDR_TETRAMESH, DumpMesh(mesh, 1.0f, "8000FF00", "mesh");)
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, Decompose);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, Decompose);

	// Generally speaking, the way this works is to set all tetras to the poly_id
	// of the next polytope to be created (so to start with, all tetras are poly_id == 0)
	// We choose a split plane and chop off tetras from the current consider list
	// moving them into the rejects list (rejects get their poly_id set to 'next_poly_id')
	// Once the consider list is convex, we make a polytope from it, then start again
	// with the rejects from the previous iteration.
	TSize search_id = 0;
	TSize next_poly_id = 1;

	// Initialise two chains, one for the tetras we need to consider,
	// the other for those we've rejected
	Tetra rejected; pr::chain::Init(rejected);
	Tetra consider; pr::chain::Init(consider);
	InitTetras(mesh, consider);
	PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, consider, 1.0f, "8000a000", "consider");)

	// Build a cache of the concave edges to save time when finding a split plane
	// This cache will be maintained for the portion of the mesh in front of each
	// split plane but not behind. The reason for this is that tetras behind the
	// split plane are not guaranteed to be connected so removing tetras can
	// invalidate some of the cached edges. For example, consider three adjacent
	// tetras where the middle one is removed.
	EdgeCache edge_cache;
	CacheConcaveEdges(mesh, consider, convex_tolerance, edge_cache);
	PR_EXPAND(PR_LDR_TETRAMESH, DumpEdges(mesh, edge_cache, "FFFFFF00", "edges");)

	// While there are tetras still left to consider...
	for(;;)
	{
		PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, consider, 1.0f, "8000a000", "consider");)
		PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, rejected, 1.0f, "80800000", "rejected");)
		PR_EXPAND(PR_LDR_TETRAMESH, ClearFile("C:/deleteme/tetramesh_poly.pr_script");)
		PR_EXPAND(PR_LDR_TETRAMESH, ClearFile("C:/deleteme/tetramesh_splitplane.pr_script");)
		PR_EXPAND(PR_DBG_GEOM_TETRAMESH, for( Tetra const* t = pod_chain::Begin(consider); t != pod_chain::End(consider); t = t->m_next) )
		PR_EXPAND(PR_DBG_GEOM_TETRAMESH, { PR_ASSERT(PR_DBG_GEOM_TETRAMESH, t->m_poly_id == next_poly_id - 1, "Every tetra being considered should have its poly_id equal to the current polytope being created"); })

		// If the cache contains concave edges we want to split 'consider'
		TIndex start;
		Plane  split_plane;
		if( FindSplitPlane(mesh, edge_cache, convex_tolerance, split_plane, start) )
		{
			// Partition the tetras into those in front of the split plane and connected to 'start',
			// and those that are not (i.e. behind/spanning the split plane or not connected to 'start')
			// Those that "are" will be added to 'polytope'.
			Tetra polytope;	pr::chain::Init(polytope);
			polytope.m_poly_id = next_poly_id - 1;
			PartitionTetras(mesh, polytope, split_plane, convex_tolerance, start, search_id++);
			PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, polytope, 0.99f, "FFFF8000", "poly"));

			// It shouldn't possible to move all of the tetras from 'consider' into
			// 'polytope' as the split plane must be between at least two tetras.
			// This is guaranteed if the edge is concave and all tetras have a
			// positive volume. If this assert fires we'll end up with an infinite loop.
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, !pod_chain::Empty(consider), "");

			// Mark those remaining in the consider list with the next
			// polytope id as they will be moved to the the rejected list
			for (pr::chain::Iter<Tetra> iter(consider); iter; ++iter)
				iter->m_poly_id = next_poly_id;
			
			// Remove cached edges that are not part of 'polytope' and add any
			// new concave edges for the edges that surround the faces on the
			// interface between 'polytope' and 'consider'
			UpdateEdgeCache(mesh, polytope, consider, edge_cache, convex_tolerance);

			// Transfer those that remain in 'consider' into the rejected list
			// and set 'polytope' as the new set for considering
			pr::chain::Join(consider, rejected);
			pr::chain::Remove(consider);
			pr::chain::Insert(polytope, consider);
			pr::chain::Remove(polytope);
		}
		// If a split plane cannot be found then there can be no concave edges
		// in the set. This means that the set contains one or more convex pieces.
		// Generate a polytope for each convex piece.
		else
		{
			// If the rejects chain is empty then 'consider' contains the remaining
			// polytopes to be created. In this case only, it's possible that the
			// consider chain contains more than one convex piece, so use a more
			// sophisticated method for generating polytopes
			if (!pr::chain::Empty(rejected))
			{
				GeneratePolytope(mesh, consider, gen);
				next_poly_id++;

				// The rejects now become the consider chain
				PR_ASSERT(PR_DBG_GEOM_TETRAMESH, pod_chain::Empty(consider), "");
				pr::chain::Insert(rejected, consider);
				pr::chain::Remove(rejected);
				PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, consider, 1.0f, "8000a000", "consider");)

				// Generate a new cache of concave edges for the new consider chain
				edge_cache.clear();
				CacheConcaveEdges(mesh, consider, convex_tolerance, edge_cache);
				PR_EXPAND(PR_LDR_TETRAMESH, DumpEdges(mesh, edge_cache, "FFFFFF00", "edges");)
			}
			else
			{
				GeneratePolytopes(mesh, consider, gen, next_poly_id, search_id++);
				break;
			}
		}
	}

	pr::chain::Remove(consider);
	pr::chain::Remove(rejected);
	PR_EXPAND(PR_DBG_GEOM_TETRAMESH, pr::cons().Print(Fmt("%d polytopes from %d tetras\n", next_poly_id - 1, mesh.m_num_tetra)));
}

// Initialise the consider list of tetrahedrons
void pr::tetramesh::InitTetras(tetramesh::Mesh& mesh, Tetra& consider)
{
	// Reset all the id's in 'mesh' and add each tetra to the consider chain
	for( Tetra *t = mesh.m_tetra, *t_end = t + mesh.m_num_tetra; t != t_end; ++t )
	{
		t->m_poly_id = 0;
		t->m_id = IdNotSet;
		pr::chain::Init(*t);
		pr::chain::Insert(consider, *t);
	}
}

// Add any concave edges that surround the face opposite 'tetra.m_nbr[nbr_idx]' to 'edge_cache'.
void pr::tetramesh::AddConcaveEdges(tetramesh::Mesh const& mesh, Tetra const& tetra, CIndex nbr_idx, float convex_tolerance, EdgeCache& edge_cache)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, AddCCEdges);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, AddCCEdges);

	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, IsBoundaryFace(mesh, tetra, nbr_idx), "");
	PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, tetra, 1.0f, "800000FF", "tetra");)
	PR_EXPAND(PR_LDR_TETRAMESH, DumpFace (mesh, tetra.OppFace(nbr_idx), "FF0000FF", "face");)

	tetramesh::Edge edge;
	tetramesh::Face& face = edge.m_Lface;
	tetramesh::Face& nbr  = edge.m_Rface;

	// Get the boundary face
	face		  = tetra.OppFace(nbr_idx);
	face.m_tetra0 = static_cast<TIndex>(&tetra - mesh.m_tetra);
	face.m_tetra1 = tetra.m_nbrs[nbr_idx];
	face.m_order  = GetFaceIndexOrder(face);

	// Create a predicate for finding boundary faces
	Pred_BoundaryFace is_boundary_face(mesh);

	// Test each edge of this external face for being concave
	for( int i = 0; i != 3; ++i )
	{
		// Prevent face pairs being tested twice by only considering
		// edges where the VIndices are in increasing order
		edge.m_i0 = face.m_i[(i+1)%3];
		edge.m_i1 = face.m_i[(i+2)%3];
		if( edge.m_i1 < edge.m_i0 )
			continue;

		// If this edge already exists in the cache, skip it.
		Edge const* iter = edge_cache.find(edge);
		if( iter != edge_cache.end() && *iter == edge )
			continue;

		// Find the neighbouring boundary face
		nbr = GetNeighbouringFace(mesh, face, i, is_boundary_face);
		PR_EXPAND(PR_LDR_TETRAMESH, DumpFace(mesh, nbr, "FFFF0000", "nbr");)

		// Measure the concavity of the edge
		MeasureConcavity(mesh, edge);

		// If the edge is concave, insert it into the cache
		if( edge.m_concavity > convex_tolerance )
		{
			edge_cache.insert(iter, edge);
		}
	}
}

// Loop through the boundary faces of the tetras in the 'consider' list looking for the concave edges
void pr::tetramesh::CacheConcaveEdges(tetramesh::Mesh const& mesh, Tetra const& consider, float convex_tolerance, EdgeCache& edge_cache)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, CacheCCEdges);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, CacheCCEdges);

	for (pr::chain::Iter<Tetra const> t(consider); t; ++t)
	{
		Tetra const& tetra = *t;
		//PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, tetra, 1.0f, "80FF0000", "tetra");)

		// If the tetra contains external faces check the concavity of the edges
		// surrounding these faces. Save the most concave ones in the edge_cache.
		for( CIndex n = 0; n != NumCnrs; ++n )
		{
			if( !IsBoundaryFace(mesh, tetra, n) ) continue;
			AddConcaveEdges(mesh, tetra, n, convex_tolerance, edge_cache);
		}
	}
}

// Measure the concavity of an edge.
void pr::tetramesh::MeasureConcavity(tetramesh::Mesh const& mesh, Edge& edge)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, MeasureCcv);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, MeasureCcv);

	// Get the vertex indices of the 'other two' vertices.
	// These are the verts opposite the edge in each of the connected faces
	VIndex Lidx = edge.m_Lface.m_i[0] + edge.m_Lface.m_i[1] + edge.m_Lface.m_i[2] - edge.m_i0 - edge.m_i1;
	VIndex Ridx = edge.m_Rface.m_i[0] + edge.m_Rface.m_i[1] + edge.m_Rface.m_i[2] - edge.m_i0 - edge.m_i1;

	// Get the verts
	v4 const& a = mesh.m_verts[edge.m_i0];
	v4 const& b = mesh.m_verts[edge.m_i1];
	v4 const& c = mesh.m_verts[Lidx];
	v4 const& d = mesh.m_verts[Ridx];

	// Use the volume of the tetrahedron formed by these verts to decide whether the edge is concave
	// Don't use this as a measure of concavity however as it doesn't find highly concave verts that
	// are connected to only slightly concave edges.
	edge.m_concavity = -Volume(a, b, c, d);
	if( edge.m_concavity > maths::tiny )
	{
		// Measure concavity as the minimum distance between the edge and the 'bridge'
		v4 pt0, pt1;
		ClosestPoint_LineSegmentToLineSegment(a, b, c, d, pt0, pt1);
		edge.m_concavity = Length3(pt0 - pt1);

		PR_EXPAND(PR_LDR_TETRAMESH, StartFile("C:/Deleteme/tetramesh_dir.pr_script");)
		PR_EXPAND(PR_LDR_TETRAMESH, ldr::Line("bridge", "FF00FF00", c, d);)
		PR_EXPAND(PR_LDR_TETRAMESH, ldr::Line("nearest_points", "FF00FFFF", pt0, pt1);)
		PR_EXPAND(PR_LDR_TETRAMESH, ldr::LineD("bisect", "FFFFFF00", 0.5f*(a+b), Normalise3(pt0 - pt1));)
		PR_EXPAND(PR_LDR_TETRAMESH, EndFile();)
	}
}

// An object that finds the best face to use as a split plane
// Search for a face that bisects the edge and has the neighbouring
// face vertices as close as possible to the plane
struct SplitPlaneFinder
{
	tetramesh::Mesh const&	m_mesh;
	Tetra const&			m_tetraL;
	Tetra const&			m_tetraR;
	float					m_tolerance;
	v4						m_split_plane;
	TIndex					m_start;

	SplitPlaneFinder(tetramesh::Mesh const& mesh, tetramesh::Edge const& edge, float convex_tolerance)
	:m_mesh(mesh)
	,m_tetraL(mesh.m_tetra[edge.m_Lface.m_tetra0])
	,m_tetraR(mesh.m_tetra[edge.m_Rface.m_tetra0])
	,m_tolerance(convex_tolerance)
	,m_split_plane(-GetPlane(mesh, edge.m_Lface))
	,m_start(edge.m_Lface.m_tetra0)
	{
		PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, SPFinder);
		PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, SPFinder);

		// Try to use the left face as a split plane. If the right hand tetra is classed
		// as behind the split plane then we're done. If not, then start iterating from
		// the right hand face until the left hand tetra is in front of the split plane.
		if( Compare(mesh, m_tetraR, m_split_plane, m_tolerance) > 0 )
		{
			// This is the index of the edge within 'edge.m_Rface'
			// (equal to the index of the vertex opposite the edge within 'edge.m_Rface')
			CIndex edge_index = (edge.m_Rface.m_i[1] == edge.m_i1) ? 0 :
								(edge.m_Rface.m_i[2] == edge.m_i1) ? 1 : 2;

			GetNeighbouringFace(mesh, edge.m_Rface, edge_index, *this);
		}

		// We need to guarantee that the split plane will divide the left and right tetras.
		// The code assumes 'm_tetraL' is in front of the split plane so we need to ensure
		// 'm_tetraR' is classified as spanning or behind the plane.
		PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Compare(mesh, m_tetraL, m_split_plane, convex_tolerance) >= 0, "");
		PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Compare(mesh, m_tetraR, m_split_plane, convex_tolerance) <= 0, "");
		PR_EXPAND(PR_LDR_TETRAMESH, DumpSplitPlane(mesh, m_split_plane, m_start, "800080FF", "splitplane");)
	}	
	bool operator ()(tetramesh::Face const& face)
	{
		PR_EXPAND(PR_LDR_TETRAMESH, DumpFace(m_mesh, face, "FF00FF00", "nbr"));

		m_split_plane = GetPlane(m_mesh, face);
		m_start = face.m_tetra1;
		return Compare(m_mesh, m_tetraL, m_split_plane, m_tolerance) >= 0;
	}
	SplitPlaneFinder(SplitPlaneFinder const&);
	SplitPlaneFinder& operator =(SplitPlaneFinder const&);
};

// Search the edge cache for the most concave edge.
// Returns true if a concave edge was found
bool pr::tetramesh::FindMostConcaveEdge(EdgeCache const& edge_cache, Edge& concave_edge)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, FindMCE);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, FindMCE);

	concave_edge.m_concavity = 0.0f;
	Edge const* most_concave = &concave_edge;
	for( Edge const* e = edge_cache.begin(), *e_end = edge_cache.end(); e != e_end; ++e )
	{	
		if( e->m_concavity < most_concave->m_concavity ) continue;
		most_concave = e;
	}
	if( most_concave != &concave_edge )
	{
		concave_edge = *most_concave;
		return true;
	}
	return false;	
}

// Chooses a suitable plane for partitioning the tetramesh.
// The returned plane must have at least one but not all tetras in front of it.
// Ideally we want as many concave edges as possible to lie in the returned split plane
// as this will help reduce the number of polytopes created. However, finding this
// is a fairly expensive exercise. Also a plane that passes through two or more concave
// edges may not have a tetra completely to one side of it. Settle for using the most concave edge.
// Returns true if a split plane was found (i.e. if there are concave edges in edge_cache)
// 'split_plane' is the plane to used to partition the tetramesh
// 'start' is the index of a tetradehron that is completely in front of the plane
// Note: it should always be possible to find a plane that sub divides the mesh
// if a concave edge exists and all tetrahedrals in the mesh have a positive volume.
bool pr::tetramesh::FindSplitPlane(tetramesh::Mesh const& mesh, EdgeCache const& edge_cache, float convex_tolerance, Plane& split_plane, TIndex& start)
{
	// Find the most concave edge
	Edge concave_edge;
	if( !FindMostConcaveEdge(edge_cache, concave_edge) )
		return false;

	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, mesh.m_tetra[concave_edge.m_Lface.m_tetra0].m_poly_id == mesh.m_tetra[concave_edge.m_Rface.m_tetra0].m_poly_id,
		"Concave edges should not exist between faces that are part of tetras belonging to different polytopes");
	PR_EXPAND(PR_DBG_GEOM_TETRAMESH, Edge cpy = concave_edge; MeasureConcavity(mesh, cpy);)
	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, FEql(cpy.m_concavity, concave_edge.m_concavity) && cpy.m_concavity > 0.0f,
		"Only concave edges should be in the cache. 'concave_edge' should have the correct concavity measurement");

	PR_EXPAND(PR_LDR_TETRAMESH, DumpEdge(mesh, concave_edge, "FFFFFF00", "edge");)
				
	// Find a split plane that is constrained in one axis to 'concave_edge'
	SplitPlaneFinder sp_finder(mesh, concave_edge, convex_tolerance);
	split_plane = sp_finder.m_split_plane;
	start		= sp_finder.m_start;
	return true;
}

// Add tetras that are connected to 'start' and in front of 'split_plane' to 'polytope'
void pr::tetramesh::PartitionTetras(tetramesh::Mesh& mesh, Tetra& polytope, Plane const& split_plane, float convex_tolerance, TIndex start, TSize search_id)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, Partition);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, Partition);

	// Mark the starting tetra
	mesh.m_tetra[start].m_id = search_id;

	// Recursively consider the neighbours of 'start' for being part of the polytope.
	TTIndices stack;
	stack.push_back(start);
	while( !stack.empty() )
	{
		TIndex t_idx = stack.back(); stack.pop_back();
		Tetra& tetra = mesh.m_tetra[t_idx];
		//PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Compare(mesh, tetra, split_plane, convex_tolerance) > 0, "'tetra' is behind the split plane, it's supposed to be in front");
		pr::chain::Insert(polytope, tetra); // Move this tetra into 'polytope'

		//PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, tetra, 1.0f, "800000FF", "tetra"));
		//PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, polytope, 1.0f, "FFFF8000", "poly"));

		// Add the neighbours to the stack if they are in front of the split plane
		// (they must be connected to start as they are neighbours)
		for( CIndex n = 0; n != NumNbrs; ++n )
		{
			// If the neighbour belongs to another polytope don't consider it
			if( IsBoundaryFace(mesh, tetra, n) ) continue;

			Tetra& nbr = mesh.m_tetra[tetra.m_nbrs[n]];
			//PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, nbr, 1.0f, "8000FF00", "nbr"));

			// If we're tested this tetra before, skip it
			if( nbr.m_id == search_id ) continue;
			nbr.m_id = search_id;

			// If the neighbour is behind or spanning the split
			// plane then it can't be part of the new polytope
			if( Compare(mesh, nbr, split_plane, convex_tolerance) <= 0 ) continue;

			// Add the neighbour to the stack
			stack.push_back(tetra.m_nbrs[n]);
		}
	}
}

// Remove cached edges that are not part of 'polytope' and add any
// new concave edges for the edges that surround the faces on the
// interface between 'polytope' and 'consider'
void pr::tetramesh::UpdateEdgeCache(tetramesh::Mesh const& mesh, Tetra const& polytope, Tetra const& consider, EdgeCache& edge_cache, float convex_tolerance)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, UpdateECache);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, UpdateECache);

	// Remove any edges that are not part of 'polytope' i.e. keep edges
	// that are between tetras that both belong to polytope.
	Edge* edge_out = edge_cache.begin();
	Edge* edge_end = edge_cache.end();
	edge_cache.m_num_edges = 0;
	for( Edge* e = edge_out; e != edge_end; ++e )
	{
		Tetra const& Ltetra = mesh.m_tetra[e->m_Lface.m_tetra0];
		Tetra const& Rtetra = mesh.m_tetra[e->m_Rface.m_tetra0];
		//PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, Ltetra, 1.0f, "FF0000FF", "Ltetra"));
		//PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, Rtetra, 1.0f, "FFFF0000", "Rtetra"));
		if( Ltetra.m_poly_id == polytope.m_poly_id && Rtetra.m_poly_id == polytope.m_poly_id )
		{
			*edge_out++ = *e;
			++edge_cache.m_num_edges;
		}
	}

	// Now, we want to add any concave edges that are on the boundary between 'polytope' and 'consider'
	// Consider only the tetras in 'consider' and look for faces that are on the boundary with 'polytope'
	for (pr::chain::Iter<Tetra const> t(consider); t; ++t)
	{
		for( CIndex n = 0; n != NumCnrs; ++n )
		{
			if (IsExternalFace(*t, n) ) continue;

			Tetra const& tetra = *t;
			Tetra const& nbr   = mesh.m_tetra[tetra.m_nbrs[n]];
			if( nbr.m_poly_id != polytope.m_poly_id ) continue;

			// This is the neighbour index of 'tetra' from 'nbr's point of view
			CIndex m = nbr.NbrIndex(static_cast<TIndex>(t - mesh.m_tetra));
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, t == &mesh.m_tetra[nbr.m_nbrs[m]], "");
			AddConcaveEdges(mesh, nbr, m, convex_tolerance, edge_cache);
		}
	}

	// If there are no concave edges in the cache but the cache overflowed at some point
	// then we need to rescan 'polytope' for concave edges to make sure we got them all.
	if( edge_cache.m_num_edges == 0 && edge_cache.m_overflowed )
	{
		edge_cache.clear();
		CacheConcaveEdges(mesh, polytope, convex_tolerance, edge_cache);
	}

	PR_EXPAND(PR_LDR_TETRAMESH, DumpEdges(mesh, edge_cache, "FFFFFF00", "edges"));
}

struct VertRemapper
{
	tetramesh::Mesh const&	m_mesh;
	IPolytopeGenerator&		m_gen;
	VIndex*					m_map;
	VIndex					m_num_verts;
	VertRemapper(tetramesh::Mesh const&	mesh, IPolytopeGenerator& gen, VIndex* map, std::size_t map_size)
	:m_mesh(mesh)
	,m_gen(gen)
	,m_map(map)
	,m_num_verts(0)
	{
		memset(m_map, 0xFF, sizeof(VIndex) * map_size);
	}
	VIndex operator()(VIndex idx)
	{
		if( m_map[idx] == VIndex(~0) )
		{
			m_map[idx] = m_num_verts++;
			m_gen.AddPolytopeVert(m_mesh.m_verts[idx]);
		}
		return m_map[idx];
	}
	VertRemapper(VertRemapper const&);
	VertRemapper& operator=(VertRemapper const&);
};

// Generate polytopes from the convex sets in 'polytope'
void pr::tetramesh::GeneratePolytopes(tetramesh::Mesh& mesh, Tetra& polytopes, IPolytopeGenerator& gen, TSize& next_poly_id, TSize search_id)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, GenPolys);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, GenPolys);

	// Repeat until all the tetras in 'polytopes' have been added to a polytope
	while (!pr::chain::Empty(polytopes))
	{
		TSize poly_id		= next_poly_id - 1;
		Tetra* start_tetra	= &polytopes;
		TIndex start		= static_cast<TIndex>(start_tetra - mesh.m_tetra);
		start_tetra->m_id	= search_id;

		VertRemapper map(mesh, gen, static_cast<VIndex*>(alloca(mesh.m_num_verts * sizeof(VIndex))), mesh.m_num_verts);

		gen.BeginPolytope();
		Tetra poly; pr::chain::Init(poly);

		// Recursively consider the neighbours of 'start' for being part of the polytope
		TTIndices stack;
		stack.push_back(start);
		while( !stack.empty() )
		{
			TIndex t_idx = stack.back(); stack.pop_back();
			Tetra& tetra = mesh.m_tetra[t_idx];
			pr::chain::Insert(poly, tetra);	// Move the tetra into the polytope
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Volume(mesh, tetra) > 0.0f, "");
			PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, tetra, 1.0f, "80FFFF00", "tetra"));
			PR_EXPAND(PR_LDR_TETRAMESH, DumpSet(mesh, poly, 1.0f, "FFFF8000", "poly"));

			// Add the neighbours to the stack
			for( CIndex n = 0; n != NumNbrs; ++n )
			{
				// Each boundary face is a face of the polytope
				if( IsBoundaryFace(mesh, tetra, n) )
				{
					tetramesh::Face face = tetra.OppFace(n);
					gen.AddPolytopeFace(map(face.m_i[0]), map(face.m_i[1]), map(face.m_i[2]));
					continue;
				}

				Tetra& nbr = mesh.m_tetra[tetra.m_nbrs[n]];
				PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, nbr, 1.0f, "8000FF00", "nbr"));

				// If we're tested this tetra before, skip it
				if( nbr.m_id == search_id ) continue;
				nbr.m_id = search_id;

				// Add the neighbour to the stack
				stack.push_back(tetra.m_nbrs[n]);
			}
		}

		// Set the poly ids 
		for (pr::chain::Iter<Tetra> t(poly); t; ++t)
			t->m_poly_id = poly_id;

		gen.EndPolytope();
		pr::chain::Remove(poly);
		++next_poly_id;
	}
}

// Generate a polytope using the boundary faces in 'polytope'
void pr::tetramesh::GeneratePolytope(tetramesh::Mesh& mesh, Tetra& polytope, IPolytopeGenerator& gen)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, GenPoly);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, GenPoly);

	VertRemapper map(mesh, gen, static_cast<VIndex*>(alloca(mesh.m_num_verts * sizeof(VIndex))), mesh.m_num_verts);

	gen.BeginPolytope();
	for (pr::chain::Iter<Tetra> tetra(polytope); tetra; ++tetra)
	{
		PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Volume(mesh, *tetra) > 0.0f, "");
		for( CIndex n = 0; n != NumNbrs; ++n )
		{
			// Each boundary face is a face of the polytope
			if( !IsBoundaryFace(mesh, *tetra, n) ) continue;
			tetramesh::Face face = tetra->OppFace(n);
			gen.AddPolytopeFace(map(face.m_i[0]), map(face.m_i[1]), map(face.m_i[2]));
		}
	}
	gen.EndPolytope();
	pr::chain::Remove(polytope);
}

// Finds the indices of tetras that surround a vertex.
// Vertex is given as "the 'cnr_idx'th corner of tetra 'tetra_idx'"
// This can be called repeatedly for different vertices	to accumulate tetra indices
void NbrFinder::Find(tetramesh::Mesh const& mesh, TIndex tetra_idx, CIndex cnr_idx)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, FindNbrTetra);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, FindNbrTetra);

	// If this tetra is already in the list of nbrs then don't consider it
	TTIndices::iterator iter = std::lower_bound(m_nbrs.begin(), m_nbrs.end(), tetra_idx);
	if( iter != m_nbrs.end() && *iter == tetra_idx ) return;
	
	// Add a new neighbour
	m_nbrs.insert(iter, tetra_idx);

	// Recursively add the neighbouring tetras that share
	// the vertex 'mesh.m_tetra.m_cnrs[cnr_idx]'
	Tetra const& tetra = mesh.m_tetra[tetra_idx];
	VIndex		 v_idx = tetra.m_cnrs[cnr_idx];
	for( CIndex n = 1; n != NumNbrs; ++n )
	{
		CIndex idx	   = (cnr_idx + n) % NumNbrs;
		TIndex nbr_idx = tetra.m_nbrs[idx];
		if( nbr_idx == ExtnFace ) continue;
		
		Tetra const& nbr = mesh.m_tetra[nbr_idx];
		Find(mesh, nbr_idx, nbr.CnrIndex(v_idx));
	}
}

// Calculates the allowable displacement that can be applied to a vertex
// within the limits of the surrounding tetrahedra
// 'mesh' is the tetramesh containing the vertex
// 'tetra_idx' is the index of a tetra that contains the vertex to be moved as one of it's corners
// 'cnr_idx' is the index of the corner in the tetra to be moved
// 'displacement' is the amount you want to move the vertex by
// 'min_volume' is the minimum volume of any adjoining tetra if the displacement is applied
// Returns a scale factor for 'displacement' that will ensure adjoining tetrahedra have at
// least 'min_volume'. Note: if a negative value is returned then the mesh contains tetras
// with volumes least than 'min_volume'
// Be careful about adding 'maths::tiny's in here, volumes can be smaller that this error tolerance
float pr::tetramesh::ConstrainVertexDisplacement(tetramesh::Mesh const& mesh, TIndex tetra_idx, CIndex cnr_idx, v4 const& displacement, float min_volume)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, ConstrainVert);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, ConstrainVert);

	// Check the entire mesh for positive volume
	// If this fires then 'min_volume' is too big or the mesh is not set up correctly
	#if PR_DBG_GEOM_TETRAMESH == 1
	for( Tetra *i = mesh.m_tetra, *i_end = mesh.m_tetra + mesh.m_num_tetra; i != i_end; ++i )
	{ PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Volume(mesh, *i) > min_volume, ""); }
	#endif//PR_DBG_GEOM_TETRAMESH == 1

	// Find the tetras surrounding this vert
	NbrFinder nbrs;
	nbrs.Find(mesh, tetra_idx, cnr_idx);
	//#if PR_LDR_TETRAMESH == 1
	//{StartFile("C:/Deleteme/tetramesh_tetranbrs.pr_script");
	//for( TTIndices::const_iterator i = nbrs.m_nbrs.begin(), i_end = nbrs.m_nbrs.end(); i != i_end; ++i )
	//	DumpTetra(mesh, mesh.m_tetra[*i], "80FFFF00", 0);
	//EndFile();}
	//#endif//PR_LDR_TETRAMESH == 1

	// Note: this code is very susceptible to robustness errors. There are two issues,
	// Firstly, the value 'scale' has error which can mean that the point
	// 'vert_new = vert_old + scale*displacement' does not result in a volume greater
	// than 'min_volume' even if 'vert_new' could lie exactly on the line segment 'displacement'.
	// Secondly, because the point 'vert_new' is almost never on the line segment 'displacement'
	// this means the direction of 'displacement' effectively changes whenever a new 'vert_new'
	// is calculated. Changing the direction means that other tetras that had volumes above
	// 'min_volume' may no longer do so.
	// To combat these problems 'scale' is calculated in such a way to guarantee
	// a volume > 'min_volume' and all tetras are retested if 'scale' changes. Because
	// this last step could result in many iterations, if a valid displacement cannot be
	// found within 'N' iterations then we give up and say the vert cannot be displaced, i.e. scale = 0.

	VIndex vert_idx = mesh.m_tetra[tetra_idx].m_cnrs[cnr_idx];
	v4 vert_old		= mesh.m_verts[vert_idx];
	v4 vert_ideal	= vert_old + displacement;
	v4 vert_new		= vert_ideal;
	float scale		= 1.0f;
	bool scale_changed = true;
	
	//PR_EXPAND(PR_LDR_TETRAMESH, StartFile("C:/Deleteme/tetramesh_dir.pr_script");)
	//PR_EXPAND(PR_LDR_TETRAMESH, ldr::Line("Displacement", "FFFF00FF", vert_old, vert_new); EndFile();)

	for( int iterations = 0; scale_changed && iterations != 3; ++iterations )
	{
		scale_changed = false;

		// Visit all tetras that share 'vert_idx'
		for( TTIndices::const_iterator i = nbrs.m_nbrs.begin(), i_end = nbrs.m_nbrs.end(); i != i_end; ++i )
		{
			Tetra const& tetra	= mesh.m_tetra[*i];
			CIndex		c_idx	= tetra.CnrIndex(vert_idx);

			// Preserve the order of the vertices used to calculate volume
			// If we don't do this Volume(a,b,c,d) != Volume(c,d,a,b) and therefore
			// (Volume(a,b,c,d) > min_volume) == (Volume(c,d,a,b) > min_volume)
			// is not guaranteed
			v4 cnrs[4];
			cnrs[0] = mesh.m_verts[tetra.m_cnrs[0]];
			cnrs[1] = mesh.m_verts[tetra.m_cnrs[1]];
			cnrs[2] = mesh.m_verts[tetra.m_cnrs[2]];
			cnrs[3] = mesh.m_verts[tetra.m_cnrs[3]];
			cnrs[c_idx] = vert_new;
			float vol = Volume(cnrs[0], cnrs[1], cnrs[2], cnrs[3]);
			
			if( vol <= min_volume )
			{
				//PR_EXPAND(PR_LDR_TETRAMESH, DumpTetra(mesh, tetra, "FFFF0000", "nbr");)

				// Calculate the volume at the start and end point of the line assuming
				// scale == 1 to give the best possible accuracy of the gradient of volume vs. scale
				cnrs[c_idx] = vert_old;		float vol0 = Volume(cnrs[0], cnrs[1], cnrs[2], cnrs[3]);
				cnrs[c_idx] = vert_ideal;	float vol1 = Volume(cnrs[0], cnrs[1], cnrs[2], cnrs[3]);
				float denom = vol1 - vol0;
				float numer = min_volume - vol0;
				PR_ASSERT(PR_DBG_GEOM_TETRAMESH, vol0 > min_volume, "");
					
				// Technically, if the displacement causes no change in volume then we shouldn't be
				// in here, the original tetra must have a volume less than 'min_volume'
				if( FEqlZero(denom) )
				{
					scale = 0.0f;
					break;
				}
				// Linearly interpolate between 'vol0' and 'vol1' to find a value for 'scale'
				// that will result in a volume greater than 'min_volume'. Mathematically,
				// the equation should be: scale = (min_volume - vol0) / (vol1 - vol0) but
				// this isn't robust due to floating point issues.
				else
				{
					scale_changed = true;
					scale = Clamp((numer - maths::tiny*denom) / denom, 0.0f, scale);
					vert_new = vert_old + scale * displacement;

					//PR_EXPAND(PR_LDR_TETRAMESH, StartFile("C:/Deleteme/tetramesh_dir.pr_script");)
					//PR_EXPAND(PR_LDR_TETRAMESH, ldr::Line("Displacement", "FFFF00FF", vert_old, vert_new); EndFile();)

					//// Re test the neightbours
					//PR_EXPAND(PR_LDR_TETRAMESH, for( TTIndices::const_iterator j = nbrs.m_nbrs.begin(), j_end = nbrs.m_nbrs.end(); j != j_end; ++j ))
					//{ PR_ASSERT(PR_LDR_TETRAMESH, Volume(mesh, mesh.m_tetra[*j]) > min_volume, ""); }

					//PR_EXPAND(PR_LDR_TETRAMESH, const_cast<tetramesh::Mesh&>(mesh).m_verts[vert_idx] = vert_new;)
					//PR_EXPAND(PR_LDR_TETRAMESH, DumpMesh(mesh, "8000FF00", "deform");)
					//PR_EXPAND(PR_LDR_TETRAMESH, const_cast<tetramesh::Mesh&>(mesh).m_verts[vert_idx] = vert_old;)
				}
			}
		}
	}
	// If a valid solution could not be found quickly, don't allow any displacement
	if( scale_changed )
		scale = 0.0f;

	// Assume 'vert_idx' is updated to 'vert_new' and retest all tetras for positive volume
	#if PR_LDR_TETRAMESH == 1
	vert_new = vert_old + scale * displacement;
	const_cast<tetramesh::Mesh&>(mesh).m_verts[vert_idx] = vert_new;
	// Re test the neightbours
	for( TTIndices::const_iterator j = nbrs.m_nbrs.begin(), j_end = nbrs.m_nbrs.end(); j != j_end; ++j )
	{ PR_ASSERT(PR_LDR_TETRAMESH, Volume(mesh, mesh.m_tetra[*j]) > min_volume, ""); }
	const_cast<tetramesh::Mesh&>(mesh).m_verts[vert_idx] = vert_old;
	#endif//PR_LDR_TETRAMESH == 1

	return scale;
}

// Do a self consistency check on 'mesh'
// Checks the neighbours and vertex order for the tetras in the mesh
bool pr::tetramesh::Validate(tetramesh::Mesh const& mesh)
{
	PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, Validate);
	PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, Validate);

	tetramesh::TIndex t_idx = 0;
	for( Tetra const *i = mesh.m_tetra, *i_end = mesh.m_tetra + mesh.m_num_tetra; i != i_end; ++i, ++t_idx )
	{
		Tetra const& tetra = *i;
		for( CIndex n = 0; n != NumCnrs; ++n )
		{
			if( tetra.m_cnrs[n] >= mesh.m_num_verts )
			{
				PR_ASSERT(PR_DBG_GEOM_TETRAMESH, false, FmtS("Tetra %d, corner %d refers to a vertex that doesn't exist", t_idx, n));
				return false;
			}
		}
		for( CIndex n = 0; n != NumNbrs; ++n )
		{
			if( tetra.m_nbrs[n] != ExtnFace && tetra.m_nbrs[n] >= mesh.m_num_tetra )
			{
				PR_ASSERT(PR_DBG_GEOM_TETRAMESH, false, FmtS("Tetra %d, neighbour %d refers to a tetrahedron that doesn't exist", t_idx, n));
				return false;
			}
		}
		float vol = Volume(mesh, tetra);
		if( vol <= 0.0f )		{ PR_ASSERT(PR_DBG_GEOM_TETRAMESH, false, FmtS("Tetra %d is inside out", t_idx)); return false; }
		if( vol < maths::tiny )	{ PR_INFO  (PR_DBG_GEOM_TETRAMESH, FmtS("Tetra %d has a very small volume", t_idx)); }

		for( CIndex m, n = 0; n != NumNbrs; ++n )
		{
			if( tetra.m_nbrs[n] == ExtnFace ) continue;
			Tetra const& nbr = mesh.m_tetra[tetra.m_nbrs[n]];
			for( m = 0; m != NumNbrs; ++m  )
			{
				if( nbr.m_nbrs[m] != t_idx ) continue;
				break;
			}
			if( m == NumNbrs )
			{
				PR_ASSERT(PR_DBG_GEOM_TETRAMESH, false, FmtS("Tetra %d, neighbour %d is to a tetra that does not link back", t_idx, n));
				return false;
			}
			else
			{
				Face f0, f1;
				f0.m_i[0]	= tetra.m_cnrs[FaceIndex[n][0]];
				f0.m_i[1]	= tetra.m_cnrs[FaceIndex[n][1]];
				f0.m_i[2]	= tetra.m_cnrs[FaceIndex[n][2]];
				f0.m_order	= GetFaceIndexOrder(f0);
				f1.m_i[0]	= nbr.m_cnrs[FaceIndex[m][0]];
				f1.m_i[1]	= nbr.m_cnrs[FaceIndex[m][1]];
				f1.m_i[2]	= nbr.m_cnrs[FaceIndex[m][2]];
				f1.m_order	= GetFaceIndexOrder(f1);
				if( !(f0 == f1) )
				{
					PR_ASSERT(PR_DBG_GEOM_TETRAMESH, false, FmtS("Tetra %d, neighbour %d or tetra %d, neightbour %d is incorrect given the vertex order", t_idx, n, tetra.m_nbrs[n], m));
					return false;
				}
			}
		}
	}
	return true;
}

// Returns the memory requirements for a rectangular tetramesh.
// This function should be used to set up a tetramesh::Mesh object
// with the correct array sizes before calling 'Generate()'
// 'width', 'height', and 'depth' are the dimensions in cubes (there are 5 tetra per cube)
void pr::tetramesh::SizeOfTetramesh(TSize width, TSize height, TSize depth, TSize& num_verts, TSize& num_tetra)
{
	num_verts = (width+1) * (height+1) * (depth+1);
	num_tetra = 5 * width * height * depth;
}

// Generate a rectangular tetramesh
// 'width', 'height', and 'depth' are the dimensions in cubes (there are 5 tetra per cube)
// 'size_w', 'size_h', and 'size_d' are the sizes of the cubes
void pr::tetramesh::Generate(tetramesh::Mesh& mesh, TSize width, TSize height, TSize depth, float size_w, float size_h, float size_d)
{
	TSize num_verts;
	TSize num_tetra;
	SizeOfTetramesh(width, height, depth, num_verts, num_tetra);
	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, mesh.m_num_verts == num_verts, "Vertex array too small for a rectangular mesh of this size");
	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, mesh.m_num_tetra == num_tetra, "Tetra array too small for a rectangular mesh of this size");

	float half_w = size_w / 2.0f;
	float half_h = size_h / 2.0f;
	float half_d = size_d / 2.0f;

	v4*    v = mesh.m_verts;
	Tetra* t = mesh.m_tetra;
	for( VIndex d = 0; d != depth + 1; ++d )
	{
		for( VIndex h = 0; h != height + 1; ++h )
		{
			for( VIndex w = 0; w != width + 1; ++w )
			{
				// Generate the verts
				(*v++).set(w*size_w - half_w, h*size_h  - half_h, d*size_d  - half_d, 1.0f);

				// Generate the tetras
				if( w != width && h != height && d != depth )
				{
					int sign = (-2*(w%2)+1) * (-2*(h%2)+1) * (-2*(d%2)+1);

					VIndex first_t = static_cast<VIndex>(t - mesh.m_tetra);
					VIndex first_v = static_cast<VIndex>(d*(height+1)*(width+1) + h*(width+1) + w);
					VIndex v[8]  =
					{
						static_cast<VIndex>(first_v                                ) ,static_cast<VIndex>(first_v+1                                ),
						static_cast<VIndex>(first_v+(1+width)                      ) ,static_cast<VIndex>(first_v+1+(1+width)                      ),
						static_cast<VIndex>(first_v+(1+width)*(1+height)           ) ,static_cast<VIndex>(first_v+1+(1+width)*(1+height)           ),
						static_cast<VIndex>(first_v+(1+width)+(1+width)*(1+height) ) ,static_cast<VIndex>(first_v+1+(1+width)+(1+width)*(1+height) )
					};
					TIndex nbr[12] = 
					{
						static_cast<TIndex>(first_t - 4                  ) ,static_cast<TIndex>(first_t + 5                  ) ,static_cast<TIndex>(first_t - 2                ) ,static_cast<TIndex>(first_t + 7                  ),
						static_cast<TIndex>(first_t - sign*5*width       ) ,static_cast<TIndex>(first_t + sign*5*width + 1   ) ,static_cast<TIndex>(first_t + sign*5*width + 2 ) ,static_cast<TIndex>(first_t - sign*5*width + 3   ),
						static_cast<TIndex>(first_t - 5*width*height + 2 ) ,static_cast<TIndex>(first_t - 5*width*height + 3 ) ,static_cast<TIndex>(first_t + 5*width*height   ) ,static_cast<TIndex>(first_t + 5*width*height + 1 )
					};
					if( w == 0 )		{ nbr[ 0] = nbr[ 2] = ExtnFace; }
					if( w == width-1 )	{ nbr[ 1] = nbr[ 3] = ExtnFace; }
					if( d == 0 )		{ nbr[ 8] = nbr[ 9] = ExtnFace; }
					if( d == depth-1 )	{ nbr[10] = nbr[11] = ExtnFace; }

					if( sign > 0 )
					{
						if( h == 0 )		{ nbr[ 4] = nbr[ 7] = ExtnFace; }
						if( h == height-1)	{ nbr[ 5] = nbr[ 6] = ExtnFace; }
						(*t++).Set(v[0], v[4], v[2], v[1], first_t+4, nbr[ 8], nbr[ 4], nbr[ 0]);
						(*t++).Set(v[3], v[7], v[1], v[2], first_t+4, nbr[ 9], nbr[ 5], nbr[ 1]);
						(*t++).Set(v[6], v[2], v[4], v[7], first_t+4, nbr[10], nbr[ 6], nbr[ 2]);
						(*t++).Set(v[5], v[1], v[7], v[4], first_t+4, nbr[11], nbr[ 7], nbr[ 3]);
						(*t++).Set(v[1], v[2], v[7], v[4], first_t+2, first_t+3, first_t, first_t+1);
					}
					else
					{
						if( h == 0 )		{ nbr[ 5] = nbr[ 6] = ExtnFace; }
						if( h == height-1)	{ nbr[ 4] = nbr[ 7] = ExtnFace; }
						(*t++).Set(v[2], v[3], v[0], v[6], first_t+4, nbr[ 0], nbr[ 4], nbr[ 8]);
						(*t++).Set(v[1], v[0], v[3], v[5], first_t+4, nbr[ 1], nbr[ 5], nbr[ 9]);
						(*t++).Set(v[4], v[5], v[6], v[0], first_t+4, nbr[ 2], nbr[ 6], nbr[10]);
						(*t++).Set(v[7], v[6], v[5], v[3], first_t+4, nbr[ 3], nbr[ 7], nbr[11]);
						(*t++).Set(v[0], v[3], v[5], v[6], first_t+3, first_t+2, first_t, first_t+1);
					}
				}
			}
		}
	}
	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, Validate(mesh), "");
}

#if PR_LDR_TETRAMESH == 1
void pr::tetramesh::DumpFace(tetramesh::Mesh const& mesh, Face const& face, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	v4 a = mesh.m_verts[face.m_i[0]];
	v4 b = mesh.m_verts[face.m_i[1]];
	v4 c = mesh.m_verts[face.m_i[2]];
	ldr::Triangle("Face", colour, a, b, c);
	if( filename ) EndFile();
}
void pr::tetramesh::DumpTetra(tetramesh::Mesh const& mesh, Tetra const& tetra, float scale, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	v4 a = mesh.m_verts[tetra.m_cnrs[0]];
	v4 b = mesh.m_verts[tetra.m_cnrs[1]];
	v4 c = mesh.m_verts[tetra.m_cnrs[2]];
	v4 d = mesh.m_verts[tetra.m_cnrs[3]];
	v4 centre = (a + b + c + d) / 4.0f;
	a -= centre;
	b -= centre;
	c -= centre;
	d -= centre;
	ldr::GroupStart(FmtS("tetra_%d_poly(%d)", &tetra - &mesh.m_tetra[0], tetra.m_poly_id));
	ldr::Triangle("triA", colour, centre + scale*a , centre + scale*b, centre + scale*c);
	ldr::Triangle("triB", colour, centre + scale*a , centre + scale*c, centre + scale*d);
	ldr::Triangle("triC", colour, centre + scale*a , centre + scale*d, centre + scale*b);
	ldr::Triangle("triD", colour, centre + scale*d , centre + scale*c, centre + scale*b);
	ldr::GroupEnd();
	if( filename ) EndFile();
}
void pr::tetramesh::DumpMesh(tetramesh::Mesh const& mesh, float scale, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	ldr::GroupStart("mesh");
	for( Tetra const *t = mesh.m_tetra, *t_end = mesh.m_tetra + mesh.m_num_tetra; t != t_end; ++t )
		DumpTetra(mesh, *t, scale, colour, 0);
	ldr::GroupEnd();
	if( filename ) EndFile();
}
void pr::tetramesh::DumpSplitPlane(tetramesh::Mesh const& mesh, Plane const& split_plane, TIndex start, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	v4 vert = 0.25f * ( mesh.m_verts[mesh.m_tetra[start].m_cnrs[0]] +
						mesh.m_verts[mesh.m_tetra[start].m_cnrs[1]] +
						mesh.m_verts[mesh.m_tetra[start].m_cnrs[2]] +
						mesh.m_verts[mesh.m_tetra[start].m_cnrs[3]]);
	ldr::Plane("split_plane", colour, split_plane, vert, 5.0f);
	DumpTetra(mesh, mesh.m_tetra[start], 0.99f, "80FF0000", 0);
	if( filename ) EndFile();
}
void pr::tetramesh::DumpSet(tetramesh::Mesh const& mesh, Tetra const& tetras, float scale, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	ldr::GroupStart("tetra_set");
	for( Tetra const* t = pod_chain::Begin(tetras); t != pod_chain::End(tetras); t = t->m_next )
	{
		DumpTetra(mesh, *t, scale, colour, 0);
	}
	ldr::GroupEnd();
	if( filename ) EndFile();
}
void pr::tetramesh::DumpEdge(tetramesh::Mesh const& mesh, Edge const& edge, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	v4 const& a = mesh.m_verts[edge.m_i0];
	v4 const& b = mesh.m_verts[edge.m_i1];
	ldr::Line("edge", colour, a, b);
	ldr::Nest();
	DumpFace(mesh, edge.m_Lface, "FFFF0000", 0);
	DumpFace(mesh, edge.m_Rface, "FF0000FF", 0);
	ldr::UnNest();
	if( filename ) EndFile();
}
void pr::tetramesh::DumpEdges(tetramesh::Mesh const& mesh, EdgeCache const& edge_cache, char const* colour, char const* filename)
{
	if( !pr_ldr_tetramesh_output_enable ) return;
	if( filename ) StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename));
	ldr::GroupStart("edges");
	for( Edge const *e = edge_cache.begin(), *e_end = edge_cache.end(); e != e_end; ++e )
	{
		ldr::Line("edge", colour, mesh.m_verts[e->m_i0], mesh.m_verts[e->m_i1]);
	}
	ldr::GroupEnd();
	if( filename ) EndFile();
}
#endif//PR_LDR_TETRAMESH == 1
