//*********************************************
// TetraMesh
//	copyright Paul Ryland May 2007
//*********************************************
// Note: Very Important:
//  The format of a tetrahedron is:
//				b
//			  / | \
//			 / _a_ \
//			/_-   -_\
//		   c---------d
//	Vertex 'a' is above the CCW triangle 'b,c,d'
//	That is TetraVolume(a,b,c,d)==Dot(a-b, Cross(b-c, c-d)) is positive
//	The faces <a,b,c> <a,c,d> <a,d,b> <d,c,b> have outward facing normals
//		(e.g. Cross(b-a, c-b) points out of the tetrahedron)
//	Notice: TetraVolume(a,b,c,d) = -TetraVolume(b,c,d,a) = TetraVolume(c,d,a,b) = -TetraVolume(d,a,b,c)
//
//	Neighbour indices A,B,C,D are ordered so that neighbour 'A' refers
//	to the face that is opposite vertex 'a', neighbour 'B' is opposite
//	vertex 'b', etc.

//	Equilateral tetrahedron face normals:
//	Centre->a:	(      0.0f,        1.0f,       0.0f, 0.0f),
//	Centre->b:	(      0.0f, -0.3333333f,  0.942809f, 0.0f),
//	Centre->c:	( 0.816497f, -0.3333333f, -0.471404f, 0.0f),
//	Centre->d:	(-0.816497f, -0.3333333f, -0.471404f, 0.0f)

#ifndef PR_GEOM_TETRAMESH_H
#define PR_GEOM_TETRAMESH_H

#include "pr/common/min_max_fix.h"
#include "pr/common/assert.h"
#include "pr/common/profile.h"
#include "pr/common/profile_manager.h"
#include "pr/container/vector.h"
#include "pr/math/math.h"

#define PR_DBG_GEOM_TETRAMESH 0 //PR_DBG
#define PR_LDR_TETRAMESH 0
#define PR_PROFILE_TETRAMESH 0

namespace pr
{
	namespace tetramesh
	{
		typedef unsigned int VIndex;	// Indices into the array of verts
		typedef unsigned int TIndex;	// Indices into the array of tetras
		typedef unsigned int CIndex;	// Indices into the corners or neighbours within a tetra
		typedef std::size_t  TSize;
		TIndex const ExtnFace = 0xFFFFFFFF;
		TSize  const IdNotSet = 0xFFFFFFFF;
		CIndex const NumCnrs = 4;
		CIndex const NumNbrs = 4;
		CIndex const FaceIndex[4][3] = {{3, 2, 1},  {0, 2, 3},  {0, 3, 1},  {0, 1, 2}};

		// A face within the tetramesh. If 'm_tetra1' == ExtnFace then it is an external face of the mesh
		struct Face
		{
			VIndex m_i[3];   // The verts of the face
			TIndex m_tetra0; // The tetra on the "from" side of the face
			TIndex m_tetra1; // The tetra on the "to" side of the face
			Plane  m_plane;  // The plane for this face
			int    m_order;  // 3 bitpacked indices describing the face indices in ascending order (see the operator ==())
		};
		bool operator == (tetramesh::Face const& lhs, tetramesh::Face const& rhs);
		bool operator <  (tetramesh::Face const& lhs, tetramesh::Face const& rhs);

		// An element in the tetramesh
		struct Tetra
		{
			void	Set(VIndex a, VIndex b, VIndex c, VIndex d, TIndex A, VIndex B, VIndex C, VIndex D);
			bool	HasExtnFace() const;
			CIndex	CnrIndex(VIndex vert_idx) const;
			TIndex	NbrIndex(TIndex tetra_idx) const;
			VIndex	OppVIndex(Face const& face) const;
			Face	OppFace(CIndex cnr_idx) const;
			Face	OppFaceByVIndex(VIndex vert_idx) const;

			VIndex	m_cnrs[NumCnrs];	// The four corners of the tetrahedron
			TIndex	m_nbrs[NumNbrs];	// The four adjoining tetrahedrons
			TSize	m_poly_id;			// An identifier for the polytope this tetra belongs to
			TSize	m_id;				// A general purpose id used for grouping tetras
			Tetra	*m_next, *m_prev;	// A double link used for creating chains of tetras
		};
		bool operator == (tetramesh::Tetra const& lhs, tetramesh::Tetra const& rhs);

		// Tetrahedral mesh
		struct Mesh
		{
			v4*		m_verts;			// Array of vertices within the mesh
			Tetra*	m_tetra;			// Array of tetrahedra
			TSize	m_num_verts;		// The length of the array pointed to by 'm_verts'
			TSize	m_num_tetra;		// The length of the array pointed to by 'm_tetra'
		};

		typedef pr::vector<VIndex> TVIndices;
		typedef pr::vector<TIndex> TTIndices;
		typedef pr::vector<Face>	TFaces;

		// Interface for creating polytopes from the decomposition
		struct IPolytopeGenerator
		{
			virtual ~IPolytopeGenerator() {}
			virtual void BeginPolytope() = 0;
			virtual void AddPolytopeVert(v4 position) = 0;
			virtual void AddPolytopeFace(VIndex a, VIndex b, VIndex c) = 0;
			virtual void EndPolytope() = 0;
		};

		// Helper object for finding the tetras surrounding a vertex
		struct NbrFinder
		{
			// Finds the indices of tetras that surround a vertex.
			// Vertex is given as "the 'cnr_idx'th corner of tetra 'tetra_idx'"
			// This can be called repeatedly for different vertices	to accumulate tetra indices
			void Find(tetramesh::Mesh const& mesh, TIndex tetra_idx, CIndex cnr_idx);
			TTIndices m_nbrs;
		};

		// Return the size in bytes required for a tetramesh with 'num_verts' and 'num_tetra'
		std::size_t Sizeof(std::size_t num_verts, std::size_t num_tetra);

		// Construct a tetramesh from an array of verts and tetra
		void Create(tetramesh::Mesh& tmesh, v4 const* verts, std::size_t num_verts, VIndex const* tetra, std::size_t num_tetra);
		
		// Decompose a tetrahedral mesh into convex polytopes
		void Decompose(tetramesh::Mesh& mesh, IPolytopeGenerator& gen, float convex_tolerance);

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
		float ConstrainVertexDisplacement(tetramesh::Mesh const& mesh, TIndex tetra_idx, CIndex cnr_idx, v4 displacement, float min_volume);

		// Validate the mesh. Does self consistency checks on the mesh.
		// Used for debugging many.
		bool Validate(tetramesh::Mesh const& mesh);

        // Returns the memory requirements for a rectangular tetramesh.
        // This function should be used to set up a tetramesh::Mesh object
        // with the correct array sizes before calling 'Generate()'
        // 'width', 'height', and 'depth' are the dimensions in cubes (there are 5 tetra per cube)
        void SizeOfTetramesh(TSize width, TSize height, TSize depth, TSize& num_verts, TSize& num_tetra);

        // Generate a rectangular tetramesh
        // 'width', 'height', and 'depth' are the dimensions in cubes (there are 5 tetra per cube)
        // 'size_w', 'size_h', and 'size_d' are the sizes of the cubes
        void Generate(tetramesh::Mesh& mesh, TSize width, TSize height, TSize depth, float size_w, float size_h, float size_d);

		// Tetra operations
		float	Volume(v4 a, v4 b, v4 c, v4 d);
		float	Volume(tetramesh::Mesh const& mesh, VIndex a, VIndex b, VIndex c, VIndex d);
		float	Volume(tetramesh::Mesh const& mesh, tetramesh::Tetra const& tetra);

		// Face operations
		int		GetFaceIndexOrder	(tetramesh::Face const& face);
		Plane	GetPlane			(tetramesh::Mesh const& mesh, tetramesh::Face const& face);
		v4		GetFaceCentre		(tetramesh::Mesh const& mesh, tetramesh::Face const& face);
		template <typename Pred> tetramesh::Face GetNeighbouringFace(tetramesh::Mesh const& mesh, tetramesh::Face const& face, int i, Pred& pred);

		#if PR_LDR_TETRAMESH == 1
		void DumpFace (tetramesh::Mesh const& mesh, Face const& face, char const* colour, char const* filename);
		void DumpTetra(tetramesh::Mesh const& mesh, Tetra const& tetra, float scale, char const* colour, char const* filename);
		void DumpMesh (tetramesh::Mesh const& mesh, float scale, char const* colour, char const* filename);
		#endif//PR_LDR_TETRAMESH == 1

		// Inline implementation *****************************************
		// Set the vert indices and neighbour indices for a tetra
		inline void Tetra::Set(VIndex a, VIndex b, VIndex c, VIndex d, TIndex A, VIndex B, VIndex C, VIndex D)
		{
			m_cnrs[0] = a; m_cnrs[1] = b; m_cnrs[2] = c; m_cnrs[3] = d;
			m_nbrs[0] = A; m_nbrs[1] = B; m_nbrs[2] = C; m_nbrs[3] = D;
		}

		// Returns true if this tetra contains an external face
		inline bool Tetra::HasExtnFace() const
		{
			return m_nbrs[0] == ExtnFace || m_nbrs[1] == ExtnFace || m_nbrs[2] == ExtnFace || m_nbrs[3] == ExtnFace;
		}

		// Returns the corner index for the corner that uses vertex 'vert_idx'
		inline CIndex Tetra::CnrIndex(VIndex vert_idx) const
		{
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, vert_idx == m_cnrs[0] || vert_idx == m_cnrs[1] || vert_idx == m_cnrs[2] || vert_idx == m_cnrs[3], "Tetra does not contain this vertex index");
			return	(m_cnrs[0] == vert_idx) ? 0 :
					(m_cnrs[1] == vert_idx) ? 1 :
					(m_cnrs[2] == vert_idx) ? 2 : 3;
		}

		// Returns the neighbour index for the neighbour with tetra index 'tetra_idx'
		inline TIndex Tetra::NbrIndex(TIndex tetra_idx) const
		{
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, tetra_idx == m_nbrs[0] || tetra_idx == m_nbrs[1] || tetra_idx == m_nbrs[2] || tetra_idx == m_nbrs[3], "Tetra does not contain this neighbour index");
			return	(m_nbrs[0] == tetra_idx) ? 0 :
					(m_nbrs[1] == tetra_idx) ? 1 :
					(m_nbrs[2] == tetra_idx) ? 2 : 3;
		}

		// Returns the index of the vertex opposite 'face'
		inline VIndex Tetra::OppVIndex(Face const& face) const
		{
			VIndex opp_idx = m_cnrs[0] + m_cnrs[1] + m_cnrs[2] + m_cnrs[3] - face.m_i[0] - face.m_i[1] - face.m_i[2];
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, opp_idx == m_cnrs[0] || opp_idx == m_cnrs[1] || opp_idx == m_cnrs[2] || opp_idx == m_cnrs[3], "");
			return opp_idx;
		}

		// Returns the face opposite the vertex at corner 'cnr_idx'
		// Note: this is not the opposite of OppVIndex
		inline Face Tetra::OppFace(CIndex cnr_idx) const
		{
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, cnr_idx < NumCnrs, "");
			CIndex flip = (cnr_idx%2);
			tetramesh::Face face;
			face.m_i[0]		= m_cnrs[(cnr_idx+2+!flip)%NumCnrs];
			face.m_i[1]		= m_cnrs[(cnr_idx+2+ flip)%NumCnrs];
			face.m_i[2]		= m_cnrs[(cnr_idx+1      )%NumCnrs];
			return face;
		}

		// Searches for vertex index 'vert_idx' in the indices of the tetra then returns the opposite face
		inline Face Tetra::OppFaceByVIndex(VIndex vert_idx) const
		{
			return OppFace(CnrIndex(vert_idx));
		}

		// Compare faces
		inline bool operator == (pr::tetramesh::Face const& lhs, pr::tetramesh::Face const& rhs)
		{
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, lhs.m_order == GetFaceIndexOrder(lhs), "");
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, rhs.m_order == GetFaceIndexOrder(rhs), "");
			return	lhs.m_i[(lhs.m_order>>4)&3] == rhs.m_i[(rhs.m_order>>4)&3] &&
					lhs.m_i[(lhs.m_order>>2)&3] == rhs.m_i[(rhs.m_order>>2)&3] &&
					lhs.m_i[(lhs.m_order   )&3] == rhs.m_i[(rhs.m_order   )&3];
		}
		inline bool operator <  (pr::tetramesh::Face const& lhs, pr::tetramesh::Face const& rhs)
		{
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, lhs.m_order == GetFaceIndexOrder(lhs), "");
			PR_ASSERT(PR_DBG_GEOM_TETRAMESH, rhs.m_order == GetFaceIndexOrder(rhs), "");
			if   ( lhs.m_i[(lhs.m_order>>4)&3] != rhs.m_i[(rhs.m_order>>4)&3] ) return lhs.m_i[(lhs.m_order>>4)&3] < rhs.m_i[(rhs.m_order>>4)&3];
			if   ( lhs.m_i[(lhs.m_order>>2)&3] != rhs.m_i[(rhs.m_order>>2)&3] ) return lhs.m_i[(lhs.m_order>>2)&3] < rhs.m_i[(rhs.m_order>>2)&3];
			return lhs.m_i[(lhs.m_order   )&3] <  rhs.m_i[(rhs.m_order   )&3];
		}
		inline bool operator == (pr::tetramesh::Tetra const& lhs, pr::tetramesh::Tetra const& rhs)
		{
			return	lhs.m_cnrs[0] == rhs.m_cnrs[0] &&
					lhs.m_cnrs[1] == rhs.m_cnrs[1] &&
					lhs.m_cnrs[2] == rhs.m_cnrs[2] &&
					lhs.m_cnrs[3] == rhs.m_cnrs[3];
		}

		// Return the volume of a tetra (actually volume*6 but I only care about relative volumes)
		inline float Volume(v4 a, v4 b, v4 c, v4 d)
		{
			return Dot3(a-b, Cross(b-c, c-d));
		}
		inline float Volume(tetramesh::Mesh const& mesh, VIndex a, VIndex b, VIndex c, VIndex d)
		{
			return Volume(mesh.m_verts[a], mesh.m_verts[b], mesh.m_verts[c], mesh.m_verts[d]);
		}
		inline float Volume(tetramesh::Mesh const& mesh, tetramesh::Tetra const& tetra)
		{
			return Volume(mesh, tetra.m_cnrs[0], tetra.m_cnrs[1], tetra.m_cnrs[2], tetra.m_cnrs[3]);
		}

		// Return the order in which the face indices should be compared
		inline int GetFaceIndexOrder(pr::tetramesh::Face const& face)
		{
			//order[a<b==0][b<c==0][c<a==0] => impossible
			//order[a<b==0][b<c==0][c<a==1] => c,b,a = 2<<4 | 1<<2 | 0<<0 = 36
			//order[a<b==0][b<c==1][c<a==0] => b,a,c = 1<<4 | 0<<2 | 2<<0 = 18
			//order[a<b==0][b<c==1][c<a==1] => b,c,a = 1<<4 | 2<<2 | 0<<0 = 24
			//order[a<b==1][b<c==0][c<a==0] => a,c,b = 0<<4 | 2<<2 | 1<<0 = 9
			//order[a<b==1][b<c==0][c<a==1] => c,a,b = 2<<4 | 0<<2 | 1<<0 = 33
			//order[a<b==1][b<c==1][c<a==0] => a,b,c = 0<<4 | 1<<2 | 2<<0 = 6
			//order[a<b==1][b<c==1][c<a==1] => impossible
			static int const order[2][2][2] = {{{-1, 36}, {18, 24}}, {{9, 33}, {6, -1}}};
			return order[face.m_i[0] < face.m_i[1]]
						[face.m_i[1] < face.m_i[2]]
						[face.m_i[2] < face.m_i[0]];
		}

		// Return a plane for 'face'
		inline Plane GetPlane(tetramesh::Mesh const& mesh, tetramesh::Face const& face)
		{
			return plane::make(mesh.m_verts[face.m_i[0]], mesh.m_verts[face.m_i[1]], mesh.m_verts[face.m_i[2]]);
		}

		// Return the position of the centre of the face
		inline v4 GetFaceCentre(tetramesh::Mesh const& mesh, tetramesh::Face const& face)
		{
			return (mesh.m_verts[face.m_i[0]] + mesh.m_verts[face.m_i[1]] + mesh.m_verts[face.m_i[2]]) / 3.0f;
		}

		// This function iterates through the faces connected to an edge.
		// It searches "into" the tetra that 'face' belongs to, around the edge opposite the i'th vertex of the face.
		// 'pred' should return false to continue searching, true to end the search at the current face
		// Returns the face that stopped the iteration. If 'pred' never returns false, iteration stops
		// when an external face or the original face is found
		// Example pred:
		//	struct Pred_ExteriorFace { bool operator ()(tetramesh::Face const&) const {return false;} };
		template <typename Pred>
		tetramesh::Face GetNeighbouringFace(tetramesh::Mesh const& mesh, tetramesh::Face const& face, int i, Pred& pred)
		{
			PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, GetNbringFace);
			PR_PROFILE_SCOPE  (PR_PROFILE_TETRAMESH, GetNbringFace);

			tetramesh::Face iter = face;
			iter.m_i[(i+1)%3]	= face.m_i[(i+2)%3];
			iter.m_i[(i+2)%3]	= face.m_i[(i+1)%3];
			iter.m_tetra0		= face.m_tetra1;
			iter.m_tetra1		= face.m_tetra0;
			Tetra const* tetra  = &mesh.m_tetra[iter.m_tetra1];
			for( CIndex n = 0; n != NumCnrs; ++n )
			{
				if( tetra->m_cnrs[n] != iter.m_i[i] ) continue;
				iter.m_i[i]		= tetra->OppVIndex(iter);
				iter.m_tetra0	= iter.m_tetra1;
				iter.m_tetra1	= tetra->m_nbrs[n];
				iter.m_order	= GetFaceIndexOrder(iter);
				if( pred(iter) || iter.m_tetra1 == ExtnFace || iter == face ) break;
				
				// Restart the for loop with a new tetra
				n = CIndex(-1);
				tetra = &mesh.m_tetra[iter.m_tetra1];
			}
			return iter;
		}
	}//namespace tetramesh
}//namespace pr

#endif//PR_GEOM_TETRAMESH_H
