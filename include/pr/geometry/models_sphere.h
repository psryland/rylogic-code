//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Returns the number of verts and number of indices needed to hold geometry for a sphere
	constexpr BufSizes SphereSize(int wedges, int layers)
	{
		if (wedges < 3) wedges = 3;
		if (layers < 2) layers = 2;
		return
		{
			(wedges + 1) * (layers + 1),
			3 * wedges * (2 * layers - 2),
		};
	}

	// Returns the number of verts and number of indices needed to hold geometry for a geosphere
	constexpr BufSizes GeosphereSize(int divisions)
	{
		return
		{
			3 + 10 * Pow2(2 * divisions) + 11 * Pow2(divisions),
			3 * 10 * Pow2(2 * divisions + 1),
		};
	}

	namespace geosphere
	{
		using VIndex = int;
		struct GeosphereVert
		{
			v4 m_vert;
			v4 m_norm;
			float m_ang;
			bool m_pole;
		};
		struct GeosphereFace
		{
			VIndex m_vidx[3];
		};
		struct Adjacent
		{
			VIndex m_adjacent; // The adjacent vertex (not necessarily at the same recursion level)
			VIndex m_child;    // The vertex between the associated vertex and 'm_adjacent' in the recursion level below 'm_adjacent'
			Adjacent()
				:m_adjacent()
				, m_child()
			{
			}
			Adjacent(VIndex adj, VIndex child)
				:m_adjacent(adj)
				, m_child(child)
			{
			}
		};
		using TVertCont = pr::vector<GeosphereVert, 1024>;
		using TFaceCont = pr::vector<GeosphereFace, 1024>;
		using TAdjacent = pr::vector<Adjacent>;  // A collection of adjacent vertices
		using TVertexLookupCont = pr::vector<TAdjacent>; // A map from vertex -> adjacent vertices

		// A struct to hold all of the generation data
		struct CreateGeosphereData
		{
			TVertexLookupCont m_adjacent;
			TVertCont* m_vcont = {};
			TFaceCont* m_fcont = {};
			v4                m_radius = {};
			int               m_divisions = {};
		};

		// Create a vertex and add it to the vertex container
		// Returns the index position of the vertex
		inline VIndex AddVertex(v4 const& norm, float ang, bool pole, CreateGeosphereData& data)
		{
			PR_ASSERT(PR_DBG, IsNormal(norm), "");

			// Add the vertex
			GeosphereVert vertex;
			vertex.m_vert = (data.m_radius * norm).w1();
			vertex.m_norm = norm;
			vertex.m_ang = ang;
			vertex.m_pole = pole;
			data.m_vcont->push_back(vertex);

			// Add an entry in the adjacency map
			data.m_adjacent.push_back(TAdjacent());
			return s_cast<VIndex>(data.m_vcont->size() - 1);
		}

		// Get the vertex that has these two vertices as parents
		inline VIndex GetVertex(VIndex parent0, VIndex parent1, CreateGeosphereData& data)
		{
			// Get the containers of adjacent vertex for each of parent0, and parent1
			// Note: Not using the lowest index value here because we want to minimise
			// the lengths of the adjacency containers by adding any new adjacency info
			// to the shortest container
			auto& adj0 = data.m_adjacent[parent0];
			auto& adj1 = data.m_adjacent[parent1];

			// Try and find 'parent1' in 'adj0' or 'parent0' in 'adj1'
			for (auto i = std::begin(adj0), iend = std::end(adj0); i != iend; ++i)
				if (i->m_adjacent == parent1) return i->m_child;
			for (auto i = std::begin(adj1), iend = std::end(adj1); i != iend; ++i)
				if (i->m_adjacent == parent0) return i->m_child;

			// If no child is found, add one
			auto& v0 = (*data.m_vcont)[parent0];
			auto& v1 = (*data.m_vcont)[parent1];
			auto norm = Normalise(v0.m_norm + v1.m_norm);
			auto ang = v0.m_pole ? v1.m_ang : v1.m_pole ? v0.m_ang : (v0.m_ang + v1.m_ang) * 0.5f; // Use the average angle unless one of the verts is a pole
			auto new_vidx = AddVertex(norm, ang, false, data);

			// Add an entry to the shortest adjacency map for this new child
			if (adj0.size() < adj1.size())
				adj0.push_back(Adjacent(parent1, new_vidx));
			else
				adj1.push_back(Adjacent(parent0, new_vidx));

			return new_vidx;
		}

		// Recursively add a face
		inline void AddFace(VIndex V00, VIndex V11, VIndex V22, VIndex level, CreateGeosphereData& data)
		{
			PR_ASSERT(PR_DBG, V00 < s_cast<VIndex>(data.m_vcont->size()), "");
			PR_ASSERT(PR_DBG, V11 < s_cast<VIndex>(data.m_vcont->size()), "");
			PR_ASSERT(PR_DBG, V22 < s_cast<VIndex>(data.m_vcont->size()), "");

			if (level == data.m_divisions)
			{
				GeosphereFace face;
				face.m_vidx[0] = V00;
				face.m_vidx[1] = V11;
				face.m_vidx[2] = V22;
				data.m_fcont->push_back(face);
			}
			else
			{
				auto V01 = GetVertex(V00, V11, data);
				auto V12 = GetVertex(V11, V22, data);
				auto V20 = GetVertex(V22, V00, data);
				AddFace(V00, V01, V20, level + 1, data);
				AddFace(V01, V11, V12, level + 1, data);
				AddFace(V20, V12, V22, level + 1, data);
				AddFace(V01, V12, V20, level + 1, data);
			}
		}

		// Create an Icosahedron and recursively subdivide the triangles
		inline void CreateIcosahedron(CreateGeosphereData& data)
		{
			constexpr float A = 2.0f / (1.0f + Sqr(maths::golden_ratiof));
			constexpr float H1 = 1.0f - A;
			constexpr float H2 = -1.0f + A;
			constexpr float R = float(SqrtCT(1.0 - H1 * H1));
			constexpr float dAng = float(maths::tau / 5.0);
			static float const ua[] = { 0.0f,0.2f,0.4f,0.6f,0.8f,1.0f,1.2f };
			static float const ub[] = { 0.1f,0.3f,0.5f,0.7f,0.9f,1.1f,1.3f };

			// Add the vertices
			float ang1 = 0.0f, ang2 = dAng * 0.5f;
			for (uint32_t w = 0; w != 6; ++w, ang1 += dAng, ang2 += dAng)
			{
				auto norm_a = v4(R * Cos(ang1), R * Sin(ang1), H1, 0.0f);
				auto norm_b = v4(R * Cos(ang2), R * Sin(ang2), H2, 0.0f);
				AddVertex(v4::ZAxis(), ub[w], true, data);
				AddVertex(norm_a, ua[w], false, data);
				AddVertex(norm_b, ub[w], false, data);
				AddVertex(-v4::ZAxis(), ua[w + 1], true, data);
			}

			// Add the faces
			for (VIndex i = 0, ibase = 0; i != 5; ++i, ibase += 4)
			{
				AddFace(ibase + 0, ibase + 1, ibase + 5, 0, data);
				AddFace(ibase + 1, ibase + 2, ibase + 5, 0, data);
				AddFace(ibase + 5, ibase + 2, ibase + 6, 0, data);
				AddFace(ibase + 6, ibase + 2, ibase + 3, 0, data);
			}
		}
	}

	// Generate an ellipsoid geosphere
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Geosphere(v4 const& radius, int divisions, Colour32 colour, VOut vout, IOut iout)
	{
		Props props;
		props.m_bbox = BBox(pr::v4Origin, radius);
		props.m_geom = EGeom::Vert | EGeom::Colr | EGeom::Norm | EGeom::Tex0;
		props.m_has_alpha = HasAlpha(colour);

		// Preallocate buffers to compile the geosphere into
		auto [vcount, icount] = GeosphereSize(divisions);
		geosphere::TVertCont verts; verts.reserve(vcount);
		geosphere::TFaceCont faces; faces.reserve(icount / 3);

		geosphere::CreateGeosphereData data;
		data.m_vcont = &verts;
		data.m_fcont = &faces;
		data.m_radius = radius;
		data.m_divisions = divisions;
		data.m_adjacent.reserve(vcount);
		geosphere::CreateIcosahedron(data);

		PR_ASSERT(PR_DBG, static_cast<int>(verts.size()) == vcount, "Number of verts in geosphere calculated incorrectly");
		PR_ASSERT(PR_DBG, static_cast<int>(faces.size()) == icount / 3, "Number of faces in geosphere calculated incorrectly");

		// Output the verts and indices
		for (auto i = std::begin(verts), iend = std::end(verts); i != iend; ++i)
		{
			vout(i->m_vert, colour, i->m_norm, v2(i->m_ang, (1.0f - i->m_norm.z) * 0.5f));
		}
		for (auto i = std::begin(faces), iend = std::end(faces); i != iend; ++i)
		{
			iout(i->m_vidx[0]);
			iout(i->m_vidx[1]);
			iout(i->m_vidx[2]);
		}

		return props;
	}

	// Generate a spherical geosphere
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Geosphere(float radius, int divisions, Colour32 colour, VOut vout, IOut iout)
	{
		return Geosphere(v4(radius, radius, radius, 0.0f), divisions, colour, vout, iout);
	}

	// Generate a standard sphere
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Sphere(v4 const& radius, int wedges, int layers, Colour32 colour, VOut vout, IOut iout)
	{
		Props props;
		props.m_bbox = BBox(pr::v4Origin, radius);
		props.m_has_alpha = colour.a != 0xFF;

		if (wedges < 3) wedges = 3;
		if (layers < 2) layers = 2;

		// Verts
		for (int w = 0; w <= wedges; ++w)
		{
			auto norm = v4::ZAxis();
			auto uv = v2(float(w + 0.5f) / wedges, 0.0f);
			vout((radius * norm).w1(), colour, norm, uv);

			for (int l = 1; l < layers; ++l)
			{
				auto a = float(maths::tauf * w / wedges);
				auto b = float(maths::tau_by_2f * l / layers);
				norm = v4(Cos(a) * Sin(b), Sin(a) * Sin(b), Cos(b), 0.0f);
				uv = v2(float(w) / wedges, (1.0f - norm.z) * 0.5f);
				vout((radius * norm).w1(), colour, norm, uv);
			}

			norm = -v4::ZAxis();
			uv = v2(float(w + 0.5f) / wedges, 1.0f);
			vout((radius * norm).w1(), colour, norm, uv);
		}

		// Faces
		int ibase = 0, ilayer = 0, verts_per_wedge = 1 + layers;
		for (int w = 0; w != wedges; ++w, ibase += verts_per_wedge, ilayer = ibase)
		{
			iout(ilayer + 0);
			iout(ilayer + 1);
			iout(ilayer + 1 + verts_per_wedge);
			++ilayer;
			for (int l = 1; l != layers - 1; ++l)
			{
				iout(ilayer + 0);
				iout(ilayer + 1);
				iout(ilayer + 0 + verts_per_wedge);
				iout(ilayer + 0 + verts_per_wedge);
				iout(ilayer + 1);
				iout(ilayer + 1 + verts_per_wedge);
				++ilayer;
			}
			iout(ilayer + 0 + verts_per_wedge);
			iout(ilayer + 0);
			iout(ilayer + 1);
		}

		return props;
	}
}
