//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once

#include "pr/geometry/common.h"
#include "pr/geometry/models_sphere.h"

namespace pr::geometry
{
	// Notes:
	//  - Skyboxes and environment maps are closely related, typically both share the same texture.
	//  - DX texture cubes are made from 6 textures as they would appear on the *outside* of a box.
	//    however skyboxes are always viewed from inside. Because of this, skybox models should all
	//    use front face culling so that the texture mapping is the same as for environment maps.

	// Returns the number of verts and number of indices needed to hold geometry for a geosphere.
	constexpr BufSizes SkyboxGeosphereSize(int divisions)
	{
		return GeosphereSize(divisions);
	}

	// Returns the number of verts and number of indices needed to hold geometry for a five sided cubic dome.
	constexpr BufSizes SkyboxFiveSidedCubicDomeSize()
	{
		return {12, 30};
	}

	// Returns the number of verts and number of indices needed to hold geometry for a six sided cube.
	constexpr BufSizes SkyboxSixSidedCubeSize()
	{
		return {24, 36};
	}

	// Creates a geosphere.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props SkyboxGeosphere(float radius, int divisions, Colour32 colour, VOut vout, IOut iout)
	{
		// Remember to use front face culling
		return Geosphere(radius, divisions, colour, vout, iout);
	}

	// Creates a five sided cube (bottom face removed).
	// The texture coordinates expect a 't' shaped texture where the centre half
	// of the texture is the top (+y), and the l,t,r,b quarters are the walls.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props SkyboxFiveSidedCubicDome(float radius, Colour32 colour, VOut vout, IOut iout)
	{
		// The texture should be like:
		//        |    |    
		//     ___|_-Z_|____
		//        |    |   
		//     -X | +Y | +X 
		//     ___|____|____
		//        | +Z |
		//        |    |
		// *Viewed from the outside*. Imagine a table cloth draped over a table.
		// Remember to use front face culling.

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Tex0;
		props.m_bbox = BBox(v4::Origin(), v4{radius});
		props.m_has_alpha = HasAlpha(colour);

		// Set the verts
		float const s = radius;
		vout(v4{-s, +s, +s, 1}, colour, v4::Zero(), v2{0.25f, 0.75f}); //0 // +Y
		vout(v4{+s, +s, +s, 1}, colour, v4::Zero(), v2{0.75f, 0.75f}); //1
		vout(v4{+s, +s, -s, 1}, colour, v4::Zero(), v2{0.75f, 0.25f}); //2
		vout(v4{-s, +s, -s, 1}, colour, v4::Zero(), v2{0.25f, 0.25f}); //3
		vout(v4{-s, -s, +s, 1}, colour, v4::Zero(), v2{0.25f, 0.75f}); //4 // +Z
		vout(v4{+s, -s, +s, 1}, colour, v4::Zero(), v2{0.25f, 1.25f}); //5
		vout(v4{+s, -s, +s, 1}, colour, v4::Zero(), v2{1.25f, 0.75f}); //6 // +X
		vout(v4{+s, -s, -s, 1}, colour, v4::Zero(), v2{1.25f, 0.25f}); //7
		vout(v4{+s, -s, -s, 1}, colour, v4::Zero(), v2{0.75f, -0.25f}); //8 // -Z
		vout(v4{-s, -s, -s, 1}, colour, v4::Zero(), v2{0.25f, -0.25f}); //9
		vout(v4{-s, -s, -s, 1}, colour, v4::Zero(), v2{-0.25f, 0.25f}); //10 // -X
		vout(v4{-s, -s, +s, 1}, colour, v4::Zero(), v2{-0.25f, 0.75f}); //11

		// Set the faces
		int const indices[] =
		{
			0, 1, 2, 0, 2, 3,
			0, 4, 5, 0, 5, 1,
			1, 6, 7, 1, 7, 2,
			2, 8, 9, 2, 9, 3,
			3, 10, 11, 3, 11, 0,
		};
		for (int i = 0; i != _countof(indices); ++i)
			iout(indices[i]);

		return props;
	}

	// Creates a cube.
	// The texture coordinates expect one texture per face.
	// Face order is: +x, -x, +y, -y, +z, -z (same order as cube map textures)
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props SkyboxSixSidedCube(float radius, Colour32 colour, VOut vout, IOut iout)
	{
		// The texture should be like:
		//        |    |    
		//     ___|_+Y_|________
		//        |    |   	|
		//     -X | +Z | +X | -Z
		//     ___|____|____|___
		//        | -Y |
		//        |    |
		// *Viewed from the outside*. +Z is the nearest face, the others wrap away from you around a box.
		// Remember to use front face culling.

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Tex0;
		props.m_bbox = BBox(v4::Origin(), v4{radius});
		props.m_has_alpha = HasAlpha(colour);

		// Set the verts
		float const s = radius;
		vout(v4{+s, +s, +s, 1}, colour, v4::Zero(), v2{0, 0}); //  0 // +X
		vout(v4{+s, -s, +s, 1}, colour, v4::Zero(), v2{0, 1}); //  1
		vout(v4{+s, -s, -s, 1}, colour, v4::Zero(), v2{1, 1}); //  2
		vout(v4{+s, +s, -s, 1}, colour, v4::Zero(), v2{1, 0}); //  3
		vout(v4{-s, +s, -s, 1}, colour, v4::Zero(), v2{0, 0}); //  4 // -X
		vout(v4{-s, -s, -s, 1}, colour, v4::Zero(), v2{0, 1}); //  5
		vout(v4{-s, -s, +s, 1}, colour, v4::Zero(), v2{1, 1}); //  6
		vout(v4{-s, +s, +s, 1}, colour, v4::Zero(), v2{1, 0}); //  7
		vout(v4{-s, +s, -s, 1}, colour, v4::Zero(), v2{0, 0}); //  8 // +Y
		vout(v4{-s, +s, +s, 1}, colour, v4::Zero(), v2{0, 1}); //  9
		vout(v4{+s, +s, +s, 1}, colour, v4::Zero(), v2{1, 1}); // 10
		vout(v4{+s, +s, -s, 1}, colour, v4::Zero(), v2{1, 0}); // 11
		vout(v4{-s, -s, +s, 1}, colour, v4::Zero(), v2{0, 0}); // 12 // -Y
		vout(v4{-s, -s, -s, 1}, colour, v4::Zero(), v2{0, 1}); // 13
		vout(v4{+s, -s, -s, 1}, colour, v4::Zero(), v2{1, 1}); // 14
		vout(v4{+s, -s, +s, 1}, colour, v4::Zero(), v2{1, 0}); // 15
		vout(v4{-s, +s, +s, 1}, colour, v4::Zero(), v2{0, 0}); // 16 // +Z
		vout(v4{-s, -s, +s, 1}, colour, v4::Zero(), v2{0, 1}); // 17
		vout(v4{+s, -s, +s, 1}, colour, v4::Zero(), v2{1, 1}); // 18
		vout(v4{+s, +s, +s, 1}, colour, v4::Zero(), v2{1, 0}); // 19
		vout(v4{+s, +s, -s, 1}, colour, v4::Zero(), v2{0, 0}); // 20 // -Z
		vout(v4{+s, -s, -s, 1}, colour, v4::Zero(), v2{0, 1}); // 21
		vout(v4{-s, -s, -s, 1}, colour, v4::Zero(), v2{1, 1}); // 22
		vout(v4{-s, +s, -s, 1}, colour, v4::Zero(), v2{1, 0}); // 23

		// Set the faces
		int const indices[] =
		{
			0, 1, 2, 0, 2, 3, // 0 - 6
			4, 5, 6, 4, 6, 7, // 6 - 12
			8, 9, 10, 8, 10, 11, // 12 - 18
			12, 13, 14, 12, 14, 15, // 18 - 24
			16, 17, 18, 16, 18, 19, // 24 - 30
			20, 21, 22, 20, 22, 23, // 30 - 36
		};
		for (int i = 0; i != _countof(indices); ++i)
			iout(indices[i]);

		return props;
	}
}
