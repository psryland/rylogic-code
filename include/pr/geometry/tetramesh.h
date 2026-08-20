//*********************************************
// TetraMesh
//  Copyright (c) Rylogic Ltd 2007, 2026
//*********************************************
// A tetrahedral mesh: an array of vertices plus an array of tetrahedra that index
// into them, with explicit face-adjacency between neighbouring tetrahedra.
//
// Tetrahedron ordering convention (carried over from the original 2007 implementation):
//
//              b
//            / | \
//           / _a_ \
//          /_-   -_\
//         c---------d
//
//  - Vertex 'a' (corner 0) is above the CCW triangle (b,c,d).
//  - That means Volume6(a,b,c,d) == Dot3(a-b, Cross(b-c, c-d)) is positive.
//  - The faces <a,b,c> <a,c,d> <a,d,b> <d,c,b> have outward facing normals
//    (e.g. Cross(b-a, c-b) points out of the tetrahedron).
//  - Neighbour indices A,B,C,D are ordered so that neighbour 'A' refers to the
//    face opposite vertex 'a', neighbour 'B' is opposite vertex 'b', and so on.
//    (i.e. neighbour 'i' shares the face that does NOT use corner 'i'.)
//
// This is a clean-room modernisation of the original module. It keeps the proven
// ordering convention and the regular 5-tetrahedra-per-cube grid generator, but drops
// the legacy raw-pointer storage, manual allocation, convex decomposition, and the
// deformable-mesh machinery, none of which are needed by current consumers.
#pragma once
#include <cstdint>
#include <array>
#include <algorithm>
#include "pr/math/math.h"
#include "pr/container/vector.h"

namespace pr::tetramesh
{
	// Sentinel stored in Tetra::m_nbr to indicate an external (boundary) face with no neighbour.
	inline constexpr int NoNeighbour = -1;

	// The local corner indices forming the outward-facing face opposite each corner.
	// FaceCorners[i] lists the three corners of the face opposite corner 'i', wound so
	// that the face normal points out of the tetrahedron.
	inline constexpr int FaceCorners[4][3] = { {3, 2, 1}, {0, 2, 3}, {0, 3, 1}, {0, 1, 2} };

	// A single tetrahedron: four corner vertex indices and four face-neighbour tetra indices.
	struct Tetra
	{
		int m_corner[4]; // Indices into Mesh::m_verts (corners a,b,c,d).
		int m_nbr[4];    // Adjacent tetra index per face (NoNeighbour for a boundary face).
	};

	// A tetrahedral mesh with value semantics.
	struct Mesh
	{
		pr::vector<v4>    m_verts; // The vertices referenced by the tetrahedra.
		pr::vector<Tetra> m_tets;  // The tetrahedra of the mesh.
	};

	// Returns six times the signed volume of the tetra (a,b,c,d).
	// Positive when the ordering convention (vertex 'a' above CCW triangle b,c,d) holds.
	// Six-times volume is the natural output of the scalar triple product; callers that
	// need the true volume divide by six (see Volume below).
	inline float Volume6(v4 a, v4 b, v4 c, v4 d)
	{
		return Dot3(a - b, Cross(b - c, c - d));
	}

	// Returns the true signed volume of the tetra (a,b,c,d).
	inline float Volume(v4 a, v4 b, v4 c, v4 d)
	{
		return Volume6(a, b, c, d) / 6.0f;
	}

	// Returns the true signed volume of a tetra within a mesh.
	inline float Volume(Mesh const& mesh, Tetra const& tetra)
	{
		return Volume(
			mesh.m_verts[tetra.m_corner[0]],
			mesh.m_verts[tetra.m_corner[1]],
			mesh.m_verts[tetra.m_corner[2]],
			mesh.m_verts[tetra.m_corner[3]]);
	}

	// Returns the total volume of the whole mesh (sum of all tetra volumes).
	inline float Volume(Mesh const& mesh)
	{
		auto total = 0.0f;
		for (auto const& tetra : mesh.m_tets)
			total += Volume(mesh, tetra);

		return total;
	}

	// Returns the vertex indices of the face opposite corner 'face_index' of a tetra,
	// wound so the face normal points out of the tetrahedron.
	inline std::array<int, 3> FaceVerts(Tetra const& tetra, int face_index)
	{
		return {
			tetra.m_corner[FaceCorners[face_index][0]],
			tetra.m_corner[FaceCorners[face_index][1]],
			tetra.m_corner[FaceCorners[face_index][2]],
		};
	}

	// Returns the number of vertices and tetrahedra a regular grid of the given
	// dimensions (measured in cubes) will contain. There are 5 tetrahedra per cube.
	inline void GridSize(int width, int height, int depth, int& num_verts, int& num_tets)
	{
		num_verts = (width + 1) * (height + 1) * (depth + 1);
		num_tets = 5 * width * height * depth;
	}

	// Generate a regular axis-aligned grid of well-shaped tetrahedra.
	// 'width', 'height', 'depth' are the grid dimensions measured in cubes (>= 1).
	// 'size_w', 'size_h', 'size_d' are the edge lengths of each cube.
	// Each cube is split into five tetrahedra, with the split diagonal alternating in a
	// checkerboard pattern so that adjacent cubes share compatible faces (this is what
	// keeps the interior tetrahedra well-shaped and the shared faces consistent).
	inline Mesh Generate(int width, int height, int depth, float size_w, float size_h, float size_d)
	{
		assert(width >= 1 && height >= 1 && depth >= 1 && "Grid dimensions must be at least one cube");

		Mesh mesh;
		int num_verts, num_tets;
		GridSize(width, height, depth, num_verts, num_tets);
		mesh.m_verts.reserve(num_verts);
		mesh.m_tets.reserve(num_tets);

		// The grid is offset by half a cube so that cube centres (rather than corners)
		// straddle nicely; the caller is expected to position/transform the grid as needed.
		auto const half_w = size_w / 2.0f;
		auto const half_h = size_h / 2.0f;
		auto const half_d = size_d / 2.0f;

		for (int d = 0; d != depth + 1; ++d)
		{
			for (int h = 0; h != height + 1; ++h)
			{
				for (int w = 0; w != width + 1; ++w)
				{
					// Emit the grid vertex at this lattice point.
					mesh.m_verts.push_back(v4(w * size_w - half_w, h * size_h - half_h, d * size_d - half_d, 1.0f));

					// Emit the five tetrahedra of the cube whose minimum corner is this lattice point.
					if (w != width && h != height && d != depth)
					{
						// The split diagonal alternates with the parity of the cube coordinates so that
						// neighbouring cubes meet on matching faces.
						auto const sign = (-2 * (w % 2) + 1) * (-2 * (h % 2) + 1) * (-2 * (d % 2) + 1);

						auto const first_t = static_cast<int>(mesh.m_tets.size());
						auto const first_v = d * (height + 1) * (width + 1) + h * (width + 1) + w;

						// The eight cube corner vertex indices.
						int const v[8] =
						{
							first_v,                                          first_v + 1,
							first_v + (1 + width),                            first_v + 1 + (1 + width),
							first_v + (1 + width) * (1 + height),             first_v + 1 + (1 + width) * (1 + height),
							first_v + (1 + width) + (1 + width) * (1 + height), first_v + 1 + (1 + width) + (1 + width) * (1 + height),
						};

						// The twelve candidate neighbour tetra indices into adjacent cubes. Out-of-grid
						// references are overwritten with NoNeighbour by the boundary guards below.
						int nbr[12] =
						{
							first_t - 4,                  first_t + 5,                  first_t - 2,                first_t + 7,
							first_t - sign * 5 * width,   first_t + sign * 5 * width + 1, first_t + sign * 5 * width + 2, first_t - sign * 5 * width + 3,
							first_t - 5 * width * height + 2, first_t - 5 * width * height + 3, first_t + 5 * width * height, first_t + 5 * width * height + 1,
						};
						if (w == 0)         { nbr[0] = nbr[2] = NoNeighbour; }
						if (w == width - 1) { nbr[1] = nbr[3] = NoNeighbour; }
						if (d == 0)         { nbr[8] = nbr[9] = NoNeighbour; }
						if (d == depth - 1) { nbr[10] = nbr[11] = NoNeighbour; }

						if (sign > 0)
						{
							if (h == 0)          { nbr[4] = nbr[7] = NoNeighbour; }
							if (h == height - 1) { nbr[5] = nbr[6] = NoNeighbour; }
							mesh.m_tets.push_back(Tetra{ {v[0], v[4], v[2], v[1]}, {first_t + 4, nbr[8],  nbr[4], nbr[0]} });
							mesh.m_tets.push_back(Tetra{ {v[3], v[7], v[1], v[2]}, {first_t + 4, nbr[9],  nbr[5], nbr[1]} });
							mesh.m_tets.push_back(Tetra{ {v[6], v[2], v[4], v[7]}, {first_t + 4, nbr[10], nbr[6], nbr[2]} });
							mesh.m_tets.push_back(Tetra{ {v[5], v[1], v[7], v[4]}, {first_t + 4, nbr[11], nbr[7], nbr[3]} });
							mesh.m_tets.push_back(Tetra{ {v[1], v[2], v[7], v[4]}, {first_t + 2, first_t + 3, first_t, first_t + 1} });
						}
						else
						{
							if (h == 0)          { nbr[5] = nbr[6] = NoNeighbour; }
							if (h == height - 1) { nbr[4] = nbr[7] = NoNeighbour; }
							mesh.m_tets.push_back(Tetra{ {v[2], v[3], v[0], v[6]}, {first_t + 4, nbr[0], nbr[4], nbr[8]} });
							mesh.m_tets.push_back(Tetra{ {v[1], v[0], v[3], v[5]}, {first_t + 4, nbr[1], nbr[5], nbr[9]} });
							mesh.m_tets.push_back(Tetra{ {v[4], v[5], v[6], v[0]}, {first_t + 4, nbr[2], nbr[6], nbr[10]} });
							mesh.m_tets.push_back(Tetra{ {v[7], v[6], v[5], v[3]}, {first_t + 4, nbr[3], nbr[7], nbr[11]} });
							mesh.m_tets.push_back(Tetra{ {v[0], v[3], v[5], v[6]}, {first_t + 3, first_t + 2, first_t, first_t + 1} });
						}
					}
				}
			}
		}
		return mesh;
	}

	// Self-consistency checks on a mesh. Returns true if the mesh is internally consistent.
	// Verifies: corner/neighbour indices are in range, every tetra has positive volume
	// (none are inside-out or degenerate), neighbour links are reciprocal, and each shared
	// face references the same three vertices from both tetrahedra.
	inline bool Validate(Mesh const& mesh)
	{
		auto const num_verts = static_cast<int>(mesh.m_verts.size());
		auto const num_tets = static_cast<int>(mesh.m_tets.size());

		for (int t_idx = 0; t_idx != num_tets; ++t_idx)
		{
			auto const& tetra = mesh.m_tets[t_idx];

			// Corner indices must reference existing vertices.
			for (int n = 0; n != 4; ++n)
			{
				if (tetra.m_corner[n] < 0 || tetra.m_corner[n] >= num_verts)
					return false;
			}

			// Neighbour indices must be NoNeighbour or reference an existing tetra.
			for (int n = 0; n != 4; ++n)
			{
				if (tetra.m_nbr[n] != NoNeighbour && (tetra.m_nbr[n] < 0 || tetra.m_nbr[n] >= num_tets))
					return false;
			}

			// Tetra must not be inside-out or degenerate.
			if (Volume(mesh, tetra) <= 0.0f)
				return false;

			// Neighbour links must be reciprocal and reference the same shared face.
			for (int n = 0; n != 4; ++n)
			{
				if (tetra.m_nbr[n] == NoNeighbour)
					continue;

				auto const& nbr = mesh.m_tets[tetra.m_nbr[n]];

				// Find which of the neighbour's faces links back to this tetra.
				int m = 0;
				for (; m != 4; ++m)
				{
					if (nbr.m_nbr[m] == t_idx)
						break;
				}
				if (m == 4)
					return false; // Neighbour does not link back.

				// The two shared faces must reference the same three vertices (any winding).
				auto f0 = FaceVerts(tetra, n);
				auto f1 = FaceVerts(nbr, m);
				std::sort(f0.begin(), f0.end());
				std::sort(f1.begin(), f1.end());
				if (f0 != f1)
					return false;
			}
		}
		return true;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::tetramesh
{
	PRUnitTest(TetraMeshTests, Quick)
	{
		// Volume sign and magnitude of a canonical unit tetra
		{
			// Corners chosen so the ordering convention gives a positive volume.
			auto a = v4(0, 0, 1, 1);
			auto b = v4(0, 0, 0, 1);
			auto c = v4(1, 0, 0, 1);
			auto d = v4(0, 1, 0, 1);
			PR_EXPECT(Volume6(a, b, c, d) > 0.0f);

			// True volume of this tetra is 1/6.
			PR_EXPECT(FEql(Volume(a, b, c, d), 1.0f / 6.0f));

			// Swapping two corners flips the sign.
			PR_EXPECT(Volume6(b, a, c, d) < 0.0f);
		}

		// Grid sizing
		{
			int nv, nt;
			GridSize(2, 3, 4, nv, nt);
			PR_EXPECT(nv == 3 * 4 * 5);
			PR_EXPECT(nt == 5 * 2 * 3 * 4);
		}

		// A single cube generates five positive-volume tetrahedra summing to the cube volume
		{
			auto mesh = Generate(1, 1, 1, 2.0f, 2.0f, 2.0f);
			PR_EXPECT(static_cast<int>(mesh.m_verts.size()) == 8);
			PR_EXPECT(static_cast<int>(mesh.m_tets.size()) == 5);
			for (auto const& t : mesh.m_tets)
				PR_EXPECT(Volume(mesh, t) > 0.0f);

			// Sum of tet volumes must equal the cube volume (2*2*2 = 8).
			PR_EXPECT(FEql(Volume(mesh), 8.0f));
			PR_EXPECT(Validate(mesh));
		}

		// A multi-cube grid is well-formed, conserves volume, and passes validation
		{
			auto const W = 3, H = 2, D = 4;
			auto const sw = 1.5f, sh = 0.75f, sd = 1.0f;
			auto mesh = Generate(W, H, D, sw, sh, sd);

			int nv, nt;
			GridSize(W, H, D, nv, nt);
			PR_EXPECT(static_cast<int>(mesh.m_verts.size()) == nv);
			PR_EXPECT(static_cast<int>(mesh.m_tets.size()) == nt);

			for (auto const& t : mesh.m_tets)
				PR_EXPECT(Volume(mesh, t) > 0.0f);

			// Total volume equals the grid volume.
			auto const expected = (W * sw) * (H * sh) * (D * sd);
			PR_EXPECT(FEql(Volume(mesh), expected));

			// Mesh is internally consistent (indices, winding, reciprocal neighbour links, shared faces).
			PR_EXPECT(Validate(mesh));
		}

		// Boundary faces are exactly the exterior faces of the grid box
		{
			auto const W = 2, H = 2, D = 2;
			auto mesh = Generate(W, H, D, 1.0f, 1.0f, 1.0f);

			// Count faces with no neighbour; these should tile the six outer faces of the box.
			// Each unit square face of the box is covered by tetra faces; rather than reconstruct
			// the tiling, assert every boundary face's three verts lie on a single box face plane.
			auto const lo = v4(-0.5f, -0.5f, -0.5f, 1.0f);
			auto const hi = v4(W - 0.5f, H - 0.5f, D - 0.5f, 1.0f);
			int boundary_faces = 0;
			for (auto const& t : mesh.m_tets)
			{
				for (int n = 0; n != 4; ++n)
				{
					if (t.m_nbr[n] != NoNeighbour)
						continue;

					++boundary_faces;
					auto f = FaceVerts(t, n);
					auto p0 = mesh.m_verts[f[0]];
					auto p1 = mesh.m_verts[f[1]];
					auto p2 = mesh.m_verts[f[2]];

					// All three verts must share a coordinate equal to one of the box extents.
					auto on_box =
						(FEql(p0.x, lo.x) && FEql(p1.x, lo.x) && FEql(p2.x, lo.x)) ||
						(FEql(p0.x, hi.x) && FEql(p1.x, hi.x) && FEql(p2.x, hi.x)) ||
						(FEql(p0.y, lo.y) && FEql(p1.y, lo.y) && FEql(p2.y, lo.y)) ||
						(FEql(p0.y, hi.y) && FEql(p1.y, hi.y) && FEql(p2.y, hi.y)) ||
						(FEql(p0.z, lo.z) && FEql(p1.z, lo.z) && FEql(p2.z, lo.z)) ||
						(FEql(p0.z, hi.z) && FEql(p1.z, hi.z) && FEql(p2.z, hi.z));
					PR_EXPECT(on_box);
				}
			}
			PR_EXPECT(boundary_faces > 0);
		}
	}
}
#endif
