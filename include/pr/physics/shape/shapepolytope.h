//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_SHAPE_POLYTOPE_H
#define PR_PHYSICS_SHAPE_POLYTOPE_H

#include "pr/physics/shape/shape.h"

namespace pr
{
	namespace ph
	{
		typedef uint8_t PolyIdx;

		struct ShapePolyFace
		{
			using VIndex = PolyIdx;

			PolyIdx m_index[3];
			PolyIdx pad;

			VIndex vindex(int i) const { return m_index[i]; }
			VIndex& vindex(int i) { return m_index[i]; }
		};
		struct ShapePolyNbrs
		{
			uint16_t m_first; // Byte offset to the first neighbour
			uint16_t m_count; // Number of neighbours

			PolyIdx const* begin() const              { return reinterpret_cast<PolyIdx const*>(this) + m_first; }
			PolyIdx*       begin()                    { return reinterpret_cast<PolyIdx*      >(this) + m_first; }
			PolyIdx const* end() const                { return begin() + m_count; }
			PolyIdx*       end()                      { return begin() + m_count; }
			PolyIdx const& nbr(std::size_t idx) const { return begin()[idx]; }
			PolyIdx&       nbr(std::size_t idx)       { return begin()[idx]; }
		};

		// Polytope shape
		struct ShapePolytope
		{
			Shape	m_base;
			uint32_t	m_vert_count;
			uint32_t	m_face_count;
			//v4			m_vert[m_vert_count];
			//ShapePolyFace m_face[m_face_count];
			//ShapePolyNbrs m_nbrs[m_vert_count];
			//PolyIdx		m_neighbour[...]

			enum { EShapeType = EShape_Polytope };

			v4 const*				vert_begin() const					{ return reinterpret_cast<v4 const*>(this + 1); }
			v4*						vert_begin()						{ return reinterpret_cast<v4*      >(this + 1); }
			v4 const*				vert_end() const					{ return vert_begin() + m_vert_count; }
			v4*						vert_end()							{ return vert_begin() + m_vert_count; }
			v4						vertex(std::size_t idx) const		{ return vert_begin()[idx]; }
			v4&						vertex(std::size_t idx)				{ return vert_begin()[idx]; }
			v4						opp_vertex(std::size_t idx) const	{ return vert_begin()[*nbr(idx).begin()]; }
			v4&						opp_vertex(std::size_t idx)			{ return vert_begin()[*nbr(idx).begin()]; }

			ShapePolyFace const*	face_begin() const					{ return reinterpret_cast<ShapePolyFace const*>(vert_end()); }
			ShapePolyFace*			face_begin()						{ return reinterpret_cast<ShapePolyFace*      >(vert_end()); }
			ShapePolyFace const*	face_end() const					{ return face_begin() + m_face_count; }
			ShapePolyFace*			face_end()							{ return face_begin() + m_face_count; }
			ShapePolyFace const&	face(std::size_t idx) const			{ return face_begin()[idx]; }
			ShapePolyFace&			face(std::size_t idx)				{ return face_begin()[idx]; }

			ShapePolyNbrs const*	nbr_begin() const					{ return reinterpret_cast<ShapePolyNbrs const*>(face_end()); }
			ShapePolyNbrs*			nbr_begin()							{ return reinterpret_cast<ShapePolyNbrs*      >(face_end()); }
			ShapePolyNbrs const*	nbr_end() const						{ return nbr_begin() + m_vert_count; }
			ShapePolyNbrs*			nbr_end()							{ return nbr_begin() + m_vert_count; }
			ShapePolyNbrs const&	nbr(std::size_t idx) const			{ return nbr_begin()[idx]; }
			ShapePolyNbrs&			nbr(std::size_t idx)				{ return nbr_begin()[idx]; }

			static ShapePolytope	make(std::size_t vert_count, std::size_t face_count, std::size_t size_in_bytes, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags) { ShapePolytope p; p.set(vert_count, face_count, size_in_bytes, shape_to_model, material_id, flags); return p; }
			ShapePolytope&			set (std::size_t vert_count, std::size_t face_count, std::size_t size_in_bytes, const m4x4& shape_to_model, MaterialId material_id, uint32_t flags);
			operator Shape const&() const	{ return m_base; }
			operator Shape& ()				{ return m_base; }
		};
		// Notes:
		//	- Sharing vertex buffers between polytopes isn't possible
		//    because each polytope needs to shift it's verts into CoM frame.

		// Shape functions
		float			CalcVolume			(ShapePolytope const& shape);
		v4				CalcCentreOfMass	(ShapePolytope const& shape);
		void			ShiftCentre			(ShapePolytope& shape, v4& shift);
		BBox&			CalcBBox			(ShapePolytope const& shape, BBox& bbox);
		m3x4			CalcInertiaTensor	(ShapePolytope const& shape);
		MassProperties& CalcMassProperties	(ShapePolytope const& shape, float density, MassProperties& mp);
		v4				SupportVertex		(ShapePolytope const& shape, v4 direction, std::size_t hint_vert_id, std::size_t& sup_vert_id);
		void			GetAxis				(ShapePolytope const& shape, v4& direction, std::size_t hint_vertex_id, std::size_t& vert_id0, std::size_t& vert_id1, bool major);
		uint32_t		VertCount			(ShapePolytope const& shape);
		uint32_t		EdgeCount			(ShapePolytope const& shape);
		uint32_t		FaceCount			(ShapePolytope const& shape);
		void			GenerateVerts		(ShapePolytope const& shape, v4* verts, v4* verts_end);
		void			GenerateEdges		(ShapePolytope const& shape, v4* edges, v4* edges_end);
		void			GenerateFaces		(ShapePolytope const& shape, uint32_t* faces, uint32_t* faces_end);
		void			StripFaces			(ShapePolytope& shape);
		bool			Validate			(ShapePolytope const& shape, bool check_com);
	}
}

#endif
