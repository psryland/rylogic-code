//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

// Terrain is a system that can return points of contact and normals for
// a collection of test spheres.
// The physics engine will test objects against the terrain by talking to
// this interface. There will be a number of stock terrain implementations
// plus clients can create there own.

#pragma once
#ifndef PR_PHYSICS_TERRAIN_H
#define PR_PHYSICS_TERRAIN_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		namespace terrain
		{
			// An volume to test for intersection with the terrain
			struct Sample
			{
				pr::v4      m_point;            // A point in world space
				float       m_radius;           // The radius of the sample sphere to test
			};
			
			// A result of sampling the terrain
			struct Result
			{
				Sample*     m_sample;           // The sample that this is a result for
				std::size_t m_sample_index;     // The index of the sample
				pr::v4      m_terrain_point;    // The point on the terrain surface
				pr::v4      m_normal;           // The normal of the terrain surface
				uint32_t    m_material_id;      // The material id of the terrain surface
			};
			
			//// A helper object for containing a number of results
			//template <std::size_t MaxResults> struct Results
			//{
			//	Result      m_result[MaxResults];
			//	std::size_t m_num_results;
			//	
			//	Results() : m_num_results(0) {}
			//	bool operator()(pr::ph::terrain::Result const& result)
			//	{
			//		m_result[m_num_results++] = result;
			//		return m_num_results < MaxResults;
			//	}
			//};
		}
		
		// Function to call with a result from the terrain system
		// Returns true if more contacts can be added.
		// If false is returned, the call to 'CollideSpheres' should return.
		typedef bool (*TerrainContact)(terrain::Result const& result, void* context);
		
		// Interface to a terrain object
		struct ITerrain
		{
			virtual ~ITerrain() {}
			
			// Collides a number of spheres against the terrain surface.
			// Each sphere can add as many contacts as it likes.
			// 'terrain_contact_cb' should only be called for actual contacts with the terrain.
			virtual void CollideSpheres(terrain::Sample* points, std::size_t num_points, TerrainContact terrain_contact_cb, void* context) = 0;
			
			// Collide a whole shape against the terrain surface
			virtual void CollideShape(Shape const& shape, m4x4 const& o2w, ContactManifold& manifold) = 0;
			
			// 'terr_ftr' should only be called for actual contacts
			//  struct TerrainContactFtr
			//  {
			//      bool operator()(pr::ph::terrain::Result const& result) { return true; } // Return true for more results
			//  };
			template <typename TCallbackFtr> void CollideSpheresFtr(terrain::Sample* points, std::size_t num_points, TCallbackFtr& terr_ftr)
			{
				CollideSpheres(points, num_points, ITerrain::CollideSpheresCB<TCallbackFtr>, &terr_ftr);
			}
			template <typename TCallbackFtr> static bool CollideSpheresCB(terrain::Result const& result, void* context)
			{
				return static_cast<TCallbackFtr*>(context)->operator()(result);
			}
		};
	}
}

#endif
















// The terrain system is assumed to be a 3D system. All lookups consist
// of a point and direction. To use a 2D height field with this system
// the client terrain system can ignore 'm_direction' and always set
// 'm_fraction' to 0.0f for a collision.
//
//  Depth vs Fraction:
//      Depth is the distance in meters that 'm_position' is from the
//      terrain in the direction of 'm_direction'.
//      Fraction is the fraction along 'm_direction' that is the collision
//      with the terrain.
//      When comparing terrain lookups, fraction is tested first. If not 0.0
//      or 1.0 then the deepest terrain lookup is the one that will collide
//      first. If 0.0 or 1.0 then m_depth is used.
//
// This structure is used for many different kinds of lookups controlled by the enum flags.
//
// Notes:
//  - 'm_direction' is not necessarily normalised.
//  - If 'm_direction' is zero, 'm_fraction' should be zero if 'm_position' is below ground.
//      'm_depth' is up to the client code but should be self consistent.
//  - If eQuickOut is set then the terrain function may return after detecting no collision
//      regardless of other flags.
//  - If a flag is set then the corresponding member should be set by the client code unless
//      eQuickOut is set.
//  - The physics engine can determine the position of the terrian using
//      m_position + m_fraction * m_direction
//  - 'm_fraction' should be in the range 0.0 to 1.0
//  - If 'm_fraction' == 0.0f then 'm_depth' should be >= 0.0
//  - 'm_collision' should be true if 'm_fraction' < 1.0f
//
//#ifndef PR_PHYSICS_TERRAIN_H
//#define PR_PHYSICS_TERRAIN_H
//
//#include "pr/maths/maths.h"
//#include "pr/Physics/Forward.h"
//
//namespace pr
//{
//  namespace ph
//  {
//      struct Terrain
//      {
//          //// The presence of a flag is intended to guarantee a valid value for
//          //// the member it represents.
//          //enum LookupType
//          //{
//          //  eCheck          = 0x01,     // Boolean test for collision
//          //  eNormal         = 0x02,     // Want the collision normal
//          //  eDepth          = 0x04,     // Want the depth of 'm_position' relative to the terrain.
//          //  eFraction       = 0x08,     // Want the fraction of 'm_direction' at the intersection
//          //  eMaterial       = 0x10,     // Want the material of the terrain
//          //  eQuickOut       = 0x20 | eCheck,    // If present means data is only wanted if there is a collision
//          //  eFull           = eCheck | eNormal | eDepth | eFraction | eMaterial,
//          //  eFullQuickOut   = eFull | eQuickOut,
//          //};
//
//          //void  SetNoCollision()                            { m_depth = -limits<float>::max(); m_fraction = 1.0f; m_collision = false; }
//          //bool  IsDeeperThan(const Terrain& other) const    { return (!FEql(m_fraction, other.m_fraction)) ? (m_fraction < other.m_fraction) : (m_depth > other.m_depth); }
//          //v4        Intersect() const                           { return m_position + m_fraction * m_direction; }
//
//          //// In
//          //uint32    m_lookup_type;          // The type of terrain lookup to do
//          //v4        m_position;             // The location that terrain is required for
//          //v4        m_direction;            // The line segment from 'm_position' to use for a terrain intersection
//          //
//          //// Out
//          //bool  m_collision;            // True if there is an intersection with the terrain
//          //v4        m_normal;               // The terrain normal at the intersection
//          //float m_depth;                // The depth that 'm_position' is below the terrain. -ve = above, +ve = below
//          //float m_fraction;             // The fraction of 'm_direction' at the point of collision. 0.0f = immediate (first) collision, 1.0 = last collision
//          //uint32_t  m_material_index;       // The material that the terrain is made out of
//      };
//
//      void TerrainCollisionBox        (const Primitive& prim, const m4x4& obj2w, Contact& contact);
//      void TerrainCollisionCylinder   (const Primitive& prim, const m4x4& obj2w, Contact& contact);
//      void TerrainCollisionSphere     (const Primitive& prim, const m4x4& obj2w, Contact& contact);
//  }//namespace ph
//}//namespace pr
//
//#endif//PR_PHYSICS_TERRAIN_H
