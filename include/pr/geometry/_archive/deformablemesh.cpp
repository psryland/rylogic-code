//*********************************************
// DeformableMesh
//  Copyright (C) Paul Ryland May 2007
//*********************************************

#include <algorithm>
#include "pr/common/profile.h"
#include "pr/common/cast.h"
#include "pr/geometry/tetramesh.h"
#include "pr/geometry/deformablemesh.h"

#if PR_LDR_TETRAMESH == 1
#include "pr/common/Fmt.h"
#include "pr/filesys/Autofile.h"
#include "pr/LineDrawerHelper/LineDrawerHelper.h"
#endif

namespace pr
{
	namespace deformable
	{
		float const MinDisplacement = 0.01f; // Displacements must be greater than this otherwise they are ignored
		float const MinDisplacementSq = Sqr(MinDisplacement);

		// Per vertex deformation data
		struct VertData
		{
			v4                m_base_pos;     // The initial position of the vertex before deformation
			v4                m_displacement; // Impulse applied to the vertex
			tetramesh::TIndex m_tetra_idx;    // The index of a tetra that uses this vertex
			DefVData          m_vdata;        // Per vertex deformation data
		};

		// Returns the magnitude of a displacement for 'position'. This function assumes
		// position is a normalised position, i.e. non-zero values are only returned from
		// this function if 'position' has a length in the range [0,1]
		inline float DentFunction(v4 const& position)
		{
			float t = Clamp(Length3(position), 0.0f, 1.0f);
			return 1.0f - (3.0f - 2.0f*t)*t*t; // Based on the blending function 3t^2 - 2t^3
		}

		// Displaces the vertices given in 'vert_indices' by the 'm_displacement' set for each vertex
		void Deform(deformable::Mesh& mesh, float min_volume, tetramesh::VIndex* vert_indices, tetramesh::TSize num_vert_indices);

		// Dump
		#if PR_LDR_TETRAMESH == 1
		void DumpImpact(m4x4 const& shape, char const* filename);
		void DumpDentSurface(m4x4 const& shape, float plasticity, float z, char const* name, char const* colour, char const* filename);
		void DumpDisplacements(deformable::Mesh const& mesh, char const* colour, char const* filename);
		#endif

	}
}

using namespace pr;
using namespace pr::tetramesh;
using namespace pr::deformable;

// Return the size in bytes required for a deformable mesh
std::size_t pr::deformable::Sizeof(std::size_t num_verts, std::size_t num_tetra)
{
	return	sizeof(deformable::Mesh)
		+	sizeof(VertData) * num_verts
		-	sizeof(tetramesh::Mesh)
		+	tetramesh::Sizeof(num_verts, num_tetra);
}

// Create a deformable mesh.
// 'verts' and 'vdata' should point to an array of data 'num_verts' long
// 'tetra' should point to an array of tetra indices 4*num_tetra long
deformable::Mesh& pr::deformable::Create(std::size_t num_verts, v4 const* verts, DefVData const* vdata, std::size_t num_tetra, VIndex const* tetra, void* buffer)
{
	deformable::Mesh& dmesh	= *static_cast<deformable::Mesh*>(buffer);
	tetramesh::Mesh& tmesh	= dmesh.m_tetra_mesh;
	tmesh.m_num_verts		= num_verts;
	tmesh.m_num_tetra		= num_tetra;
	tmesh.m_verts			= reinterpret_cast<v4*>		 (&dmesh + 1);
	tmesh.m_tetra			= reinterpret_cast<Tetra*>	 (tmesh.m_verts + tmesh.m_num_verts);
	dmesh.m_vert_data		= reinterpret_cast<VertData*>(tmesh.m_tetra + tmesh.m_num_tetra);
	tetramesh::Create(tmesh, verts, num_verts, tetra, num_tetra);

	// Copy per vertex deformation data
	VertData* vd = dmesh.m_vert_data;
	for( std::size_t i = 0; i != num_verts; ++i, ++vd )
	{
		vd->m_base_pos		= dmesh.m_tetra_mesh.m_verts[i];
		vd->m_displacement	= pr::v4Zero;
		vd->m_tetra_idx		= 0;
		vd->m_vdata			= vdata[i];
	}

	// Assign a tetra idx to each vertex of the mesh.
	// It doesn't matter which tetra each vertex refers to, as long as it refers to at least one
	for( Tetra const* t = tmesh.m_tetra, *t_end = t + tmesh.m_num_tetra; t != t_end; ++t )
	{
		for( CIndex n = 0; n != NumCnrs; ++n )
		{
			dmesh.m_vert_data[t->m_cnrs[n]].m_tetra_idx = static_cast<TIndex>(t - tmesh.m_tetra);
		}
	}
	return dmesh;
}
deformable::Mesh& pr::deformable::Create(std::size_t num_verts, v4 const* verts, DefVData const* vdata, std::size_t num_tetra, VIndex const* tetra, ByteCont& buffer)
{
	buffer.resize(Sizeof(num_verts, num_tetra));
	return Create(num_verts, verts, vdata, num_tetra, tetra, &buffer[0]);
}

struct Pred_BySmallestDisplacement
{
	deformable::Mesh const* m_mesh;
	Pred_BySmallestDisplacement(deformable::Mesh const& mesh) : m_mesh(&mesh) {}
	bool operator ()(VIndex lhs, VIndex rhs) const { return	Length3Sq(m_mesh->m_vert_data[lhs].m_displacement) < Length3Sq(m_mesh->m_vert_data[rhs].m_displacement); }
};

// Deform 'mesh' according to the displacements set in each vertex.
// This function can be used after client code has initialised the
// 'm_displacement' member for each VertData in 'mesh'
// 'min_volume' is the minimum volume all tetras must have after deformation
void pr::deformable::Deform(deformable::Mesh& mesh, float min_volume, VIndex* vert_indices, TSize num_vert_indices)
{
	//PR_DECLARE_PROFILE(PR_PROFILE_TETRAMESH, TM_Deform);
	//PR_PROFILE_SCOPE(PR_PROFILE_TETRAMESH, TM_Deform);
	PR_EXPAND(PR_LDR_TETRAMESH, DumpMesh(mesh.m_tetra_mesh, 1.0f, "8000FF00", "deform");)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDisplacements(mesh, "FFFF8000", "displacements");)

	// Sort the array of indices by smallest displacement first.
	std::sort(vert_indices, vert_indices + num_vert_indices, Pred_BySmallestDisplacement(mesh));
	
	// Move each vertex within the bounds of it's surrounding tetrahedra
	int const max_iterations = 3;
	TSize next_i_end = num_vert_indices;
	for( int iterations = 0; next_i_end != 0 && iterations != max_iterations; ++iterations )
	{
		// Use 'vert_indices' as a ring buffer (sort of). Each vertex that
		// cannot be displaced by it's full amount is added back to
		// 'vert_indices' and 'next_i_end' incremented. We'll try to
		// displace them again on the next pass.
		TSize i_end = next_i_end;
		next_i_end = 0;
		for( TSize i = 0; i != i_end; ++i )
		{
			VIndex		 v_idx = vert_indices[i];
			VertData&	 vdata = mesh.m_vert_data[v_idx];
			v4&			 vert  = mesh.m_tetra_mesh.m_verts[v_idx];
			Tetra const& tetra = mesh.m_tetra_mesh.m_tetra[vdata.m_tetra_idx];

			// Ignore very small displacements
			float displace_sq = Length3Sq(vdata.m_displacement);
			if( displace_sq < Sqr(MinDisplacement) )
			{
				vdata.m_displacement = pr::v4Zero;
				continue;
			}
			
			// Clamp to the max displacement limit defined in 'vdata'
			v4 new_pos = vert + vdata.m_displacement;
			float dist_sq = Length3Sq(new_pos - vdata.m_base_pos);
			if( dist_sq > Sqr(vdata.m_vdata.m_max_displacement) )
			{
				new_pos = vdata.m_base_pos + (new_pos - vdata.m_base_pos) * (vdata.m_vdata.m_max_displacement / Sqrt(dist_sq));
				vdata.m_displacement = new_pos - vert;
				displace_sq = Length3Sq(vdata.m_displacement);
			}

			// Clamp to the surrounding tetrahedra
			float scale = ConstrainVertexDisplacement(mesh.m_tetra_mesh, vdata.m_tetra_idx, tetra.CnrIndex(v_idx), vdata.m_displacement, min_volume);
			v4 displacement = scale * vdata.m_displacement;

			// Update the vertex position
			vert += displacement;
			vdata.m_displacement -= displacement;

			// If we have not displaced the vertex as much as
			// wanted add its index back into 'vert_indices'
			if( displace_sq * Sqr(1.0f - scale) < Sqr(MinDisplacement) || iterations == max_iterations - 1 )
			{
				vdata.m_displacement = pr::v4Zero;
			}
			else
			{
				vert_indices[next_i_end++] = v_idx;
			}

			//PR_EXPAND(PR_LDR_TETRAMESH, DumpMesh(mesh.m_tetra_mesh, "tetramesh_deform");)
			//PR_EXPAND(PR_LDR_TETRAMESH, DumpDisplacements(mesh, step_size);)
		}
	}
	PR_EXPAND(PR_LDR_TETRAMESH, DumpMesh(mesh.m_tetra_mesh, 1.0f, "8000FF00", "deform");)
	PR_ASSERT(PR_LDR_TETRAMESH, tetramesh::Validate(mesh.m_tetra_mesh), "");
}

// Deform 'mesh'. All parameters are in mesh space
// 'shape' defines the shape of the deformation as follows:
//	- the pos axis is the location on the mesh for the origin of the deformation (i.e. the point of impact)		
//	- the z axis should be in the direction of the deforming force or impulse. It's length
//	  represents the range over which the deformation occurs, i.e. given,
//	  pt = 'shape.pos' + t * shape.z, 'pt's for t >= 1 are not deformed, t < 1 are.
//	- the x,y axes form a basis for the width and height of the deformation on the surface
//	  their lengths represent the range of the deformation in these directions similarly to shape.z
//	- the axes in 'shape' do not need to be orthogonal but 'shape' must be invertable.
// 'plasticity' is a scale factor for how much the mesh deforms within 'shape'. A value of 0.0f means
//    no verts are displaced, a value of 1.0f means all verts within the shape will end up on the
//    surface defining the maximum range of the deformation.
// 'min_volume' is the minimum volume each tetra must have after deformation
// We use a normalised denting function that is non-zero in the range x=[-1,1], y=[-1,1], z=[-1,1]
// and zero outside of this range. The transform 'shape' can be thought of as the transform
// from 'mesh' space to this normalised denting function space.
void pr::deformable::Deform(deformable::Mesh& mesh, m4x4 const& shape, float plasticity, float min_volume)
{
	PR_EXPAND(PR_LDR_TETRAMESH, DumpMesh(mesh.m_tetra_mesh, 1.0f, "8000FF00", "deform");)
	PR_EXPAND(PR_LDR_TETRAMESH, DumpImpact(shape, "impact");)
	PR_EXPAND(PR_LDR_TETRAMESH, StartFile("C:/Deleteme/tetramesh_dentrange.pr_script");)
	PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.0f, "range0.0", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.1f, "range0.1", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.2f, "range0.2", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.3f, "range0.3", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.4f, "range0.4", "808080FF", 0);)
	PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.5f, "range0.5", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.6f, "range0.6", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.7f, "range0.7", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.8f, "range0.8", "808080FF", 0);)
	//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 0.9f, "range0.9", "808080FF", 0);)
	PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, 1.0f, "range1.0", "808080FF", 0);)
	PR_EXPAND(PR_LDR_TETRAMESH, EndFile();)

	// Create a buffer of the vert indices we've added displacements to
	TVIndices vert_indices;
	
	// Get a transform from mesh space to normalised deform function space
	PR_ASSERT(PR_DBG_GEOM_TETRAMESH, shape.IsInvertable(), "The provided shape matrix is degenerate");
	m4x4 mesh_to_df = Invert(shape);
	float max_displacement = Length3(shape.z);

	// Loop over the verts setting the displacements
	//v4 avr_displacement				= v4Zero;
	v4 const* v		= mesh.m_tetra_mesh.m_verts;
	VertData* vdata	= mesh.m_vert_data;
	for( VIndex i = 0, i_end = static_cast<VIndex>(mesh.m_tetra_mesh.m_num_verts); i != i_end; ++i, ++v, ++vdata )
	{
		// Transform the vert to dent function space
		v4 pos = mesh_to_df * *v;

		// Find the displacement for the vertex at this position
		float disp = plasticity * DentFunction(pos);
		if( disp * max_displacement > MinDisplacement )
		{
			vdata->m_displacement = disp * shape.z;
			//avr_displacement += vdata->m_displacement;
			vert_indices.push_back(i);

			//PR_EXPAND(PR_LDR_TETRAMESH, DumpDentSurface(shape, plasticity, pos.z, "dent", "80FF8080", "dent");)
			//PR_EXPAND(PR_LDR_TETRAMESH, DumpDisplacements(mesh, "FFFF8000","displacements");)
		}
	}
	
	// Call the generalised deform function
	if( !vert_indices.empty() )
	{
		//avr_displacement /= float(vert_indices.size());
		//vdata = mesh.m_vert_data.begin();
		//for( TVIndices::const_iterator i = vert_indices.begin(), i_end = vert_indices.end(); i != i_end; ++i, ++vdata )
		//	vdata->m_displacement -= avr_displacement;

		PR_EXPAND(PR_LDR_TETRAMESH, DumpDisplacements(mesh, "FFFF8000","displacements");)
		Deform(mesh, min_volume, &vert_indices[0], vert_indices.size());
	}
}

#if PR_LDR_TETRAMESH == 1
void pr::deformable::DumpImpact(m4x4 const& shape, char const* filename)
{
	if( filename ) { StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename)); }
	ldr::Matrix4x4("Impact", "FFFFFFFF", shape);
	if( filename ) { EndFile(); }
}

// Dumps the dent cross-section for a given z value
void pr::deformable::DumpDentSurface(m4x4 const& shape, float plasticity, float z, char const* name, char const* colour, char const* filename)
{
	if( filename ) { StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename)); }

	std::string str = FmtS("*SurfaceWHD %s %s { 21 21 \n", name, colour);
	v4 pos = v4Zero; pos.z = z;
	for( pos.y = -1.0f; pos.y <= 1.0f; pos.y += 0.09999f )
	{
		for( pos.x = -1.0f; pos.x <= 1.0f; pos.x += 0.09999f )
		{	
			float disp = plasticity * DentFunction(pos);
			str += FmtS("%f %f %f\n", pos.x, pos.y, pos.z + disp);
		}
	}
	str += ldr::Txfm(shape) + "}\n";
	ldr::g_output->Print(str);
	if( filename ) { EndFile(); }
}
void pr::deformable::DumpDisplacements(deformable::Mesh const& mesh, char const* colour, char const* filename)
{
	if( filename ) { StartFile(FmtS("C:/Deleteme/tetramesh_%s.pr_script", filename)); }
	ldr::GroupStart("Displacements");
	v4 const*		v     = mesh.m_tetra_mesh.m_verts;
	VertData const* vdata = mesh.m_vert_data;
	for( TSize i = 0; i != mesh.m_tetra_mesh.m_num_verts; ++i, ++v, ++vdata )
	{
		ldr::Box(FmtS("Vert_%d", i), colour, *v, 0.05f);
		ldr::Nest();
		ldr::LineD("Disp", colour, v4Zero, vdata->m_displacement);
		ldr::UnNest();
	}
	ldr::GroupEnd();
	if( filename ) { EndFile(); }
}
#endif//PR_LDR_TETRAMESH == 1
