//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_LDR_HELPER_PRIVATE_H
#define PR_PHYSICS_LDR_HELPER_PRIVATE_H

#if PH_LDR_OUTPUT

#include "pr/physics/utility/ldrhelper.h"
#include "physics/collision/simplex.h"

namespace pr
{
	namespace ldr
	{
		template <typename TStr> void PhSimplex(char const* name, unsigned int colour, ph::mesh_vs_mesh::Simplex const& simplex, TStr& str)
		{
			str += Fmt("*Group %s FFFFFFFF\n{\n", name);
			for( std::size_t i = 0; i != simplex.m_num_vertices; ++i )
			{
				str += Fmt("*Sphere s %08X { 0.02f *Position { %3.3f %3.3f %3.3f } }\n"
					,colour
					,simplex.m_vertex[i].m_r.x, simplex.m_vertex[i].m_r.y, simplex.m_vertex[i].m_r.z);

				for( int j = i - 1; j >= 0; --j )
				{
					str += Fmt("*Line l %08X { %3.3f %3.3f %3.3f  %3.3f %3.3f %3.3f }\n"
						,colour
						,simplex.m_vertex[i].m_r.x, simplex.m_vertex[i].m_r.y, simplex.m_vertex[i].m_r.z
						,simplex.m_vertex[j].m_r.x, simplex.m_vertex[j].m_r.y, simplex.m_vertex[j].m_r.z
						);
				}
			}
			str += "}\n";
		}
		template <typename TStr> void PhMinkowski(char const* name, unsigned int colour, ph::Shape const& shapeA, m4x4 const& a2w, ph::Shape const& shapeB, m4x4 const& b2w, TStr& str)
		{
			Geometry sph; geometry::GenerateGeosphere(sph, 1.0f, 4);
			m3x4 w2a = a2w.Getm3x4().InvertAffine();
			m3x4 w2b = b2w.Getm3x4().InvertAffine();
			std::size_t p_id = 0, q_id = 0;
			TVertexCont& verts = sph.m_frame[0].m_mesh.m_vertex;
			for( TVertexCont::iterator v = verts.begin(), v_end = verts.end(); v != v_end; ++v )
			{
				v4 direction = v->m_vertex - v4::Origin();
				v4 p = a2w * SupportVertex(shapeA, w2a *  direction, p_id, p_id);
				v4 q = b2w * SupportVertex(shapeB, w2b * -direction, q_id, q_id);
				v->m_vertex = p - q + v4::Origin();
			}
			PRMesh(name, colour, sph.m_frame[0].m_mesh, str);
		}
		template <typename TStr> void phTrackingVert(ph::mesh_vs_mesh::TrackVert const* trk, int num, TStr& str)
		{
			float distance = Dot3(vert.m_r, vert.m_direction);
			v4 offset = vert.m_r - distance * vert.m_direction;
			LineD ("vert"	, "FFFFFF00", v4::Origin(), vert.m_direction * distance, str);
			Sphere("pt"		, colour, vert.m_r, 0.001f, str);
			LineD ("x"		, colour, vert.m_r - offset, offset, str);
			LineD ("norm"	, colour, vert.m_r, vert.m_direction * 0.2f, str);
		}
		template <typename TStr> void phTrackingVert(ph::mesh_vs_mesh::Vert const& vert, unsigned int colour, TStr& str)
		{
			for( int i = 0; i != num; ++i )
			{
				GroupStart(Fmt("trk_%d", i).c_str(), str);
				phTrackingVert(trk[i].vert(), "FF00FF00", str);
				GroupEnd(str);
			}
		}
	}
}

#endif

#endif
