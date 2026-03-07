//*********************************************
// DeformableMesh
//  Copyright (C) Paul Ryland May 2007
//*********************************************

#ifndef PR_GEOM_DEFORMABLE_MESH_H
#define PR_GEOM_DEFORMABLE_MESH_H

#include "pr/common/min_max_fix.h"
#include "pr/container/byte_data.h"
#include "pr/math/math.h"
#include "pr/geometry/tetramesh.h"

namespace pr
{
	namespace deformable
	{
		// Forwards
		struct VertData;

		// The deformable mesh
		struct Mesh
		{
			tetramesh::Mesh	m_tetra_mesh;				// The tetrahedral mesh we will be deforming
			VertData*		m_vert_data;				// Extra vertex data (one for each vert in the tetramesh)
		};

		// Per vertex deformation data
		struct DefVData
		{
			float m_max_displacement;
			// Add more stuff
		};

		// Create a deformable mesh.
		// 'verts' and 'vdata' should point to an array of data 'num_verts' long
		// 'tetra' should point to an array of tetra indices 4*num_tetra long
		std::size_t Sizeof(std::size_t num_verts, std::size_t num_tetra);
		deformable::Mesh& Create(std::size_t num_verts, v4 const* verts, DefVData const* vdata, std::size_t num_tetra, tetramesh::VIndex const* tetra, void* buffer);
		deformable::Mesh& Create(std::size_t num_verts, v4 const* verts, DefVData const* vdata, std::size_t num_tetra, tetramesh::VIndex const* tetra, ByteCont& buffer);

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
		void Deform(deformable::Mesh& mesh, m4x4 const& shape, float plasticity, float min_volume);

	}// namespace deformable
}//namespace pr

#endif//PR_GEOM_DEFORMABLE_MESH_H
