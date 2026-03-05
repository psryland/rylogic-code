//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/broadphase/broadphasesnp.h"
#include "pr/physics/ray/ray.h"
#include "physics/broadphase/bppair.h"
#include "physics/utility/assert.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::geometry;

BPSweepAndPrune::BPSweepAndPrune()
:m_axis(0)
,m_sorted(false)
,m_enumerating(false)
{}

// Sort the entity array on 'm_axis' if needed
inline void BPSweepAndPrune::Sort()
{
	PR_ASSERT(PR_DBG_PHYSICS, m_entity.size() > 1, "");
	PR_ASSERT(PR_DBG_PHYSICS, !m_enumerating || m_sorted, "We should not be sorting the entities while enumerating pairs");

	// No point in sorting zero or one elements
	auto pred = [=](BPEntity const* lhs, BPEntity const* rhs) { return lhs->m_bbox->Lower(m_axis) < rhs->m_bbox->Lower(m_axis); };
	m_sorted = m_sorted || (std::sort(std::begin(m_entity), std::end(m_entity), pred), true);

	#if PR_DBG_PHYSICS
	for (BPEntityCont::iterator i = m_entity.begin(), i_end = m_entity.end() - 1; i != i_end; ++i)
		PR_ASSERT(PR_DBG_PHYSICS, (*i)->m_bbox->Lower(m_axis) <= (*(i+1))->m_bbox->Lower(m_axis), "");
	#endif
}

// Add/Remove a broadphase entity to the broadphase.
// The entity should be a member of the object you want broadphase'd
void BPSweepAndPrune::Add(BPEntity& entity)
{
	PR_ASSERT(PR_DBG_PHYSICS, !m_enumerating, "Do not modify the broadphase while pair enumeration is happening");
	PR_ASSERT(PR_DBG_PHYSICS, std::find(m_entity.begin(), m_entity.end(), &entity) == m_entity.end(), "Object already in broadphase");
	m_entity.push_back(&entity);
	entity.m_broadphase = this;
	m_sorted = false;
}

// Add/Remove a broadphase entity to the broadphase.
// The entity should be a member of the object you want broadphase'd
void BPSweepAndPrune::Remove(BPEntity& entity)
{
	PR_ASSERT(PR_DBG_PHYSICS, !m_enumerating, "Do not modify the broadphase while pair enumeration is happening");

	// Doing a binary search for 'entity', we need the array to be sorted for this
	if (m_entity.size() > 1) Sort();

	// There may be several targets with the same lower bound.
	auto pred = [=](BPEntity const* lhs, BPEntity const* rhs) { return lhs->m_bbox->Lower(m_axis) < rhs->m_bbox->Lower(m_axis); };
	auto target = std::lower_bound(m_entity.begin(), m_entity.end(), &entity, pred);
	for (; target != m_entity.end() && *target != &entity && !pred(&entity, *target); ++target){}
	PR_ASSERT(PR_DBG_PHYSICS, target != m_entity.end() && *target == &entity, "Object not in broadphase");

	m_entity.erase_fast(target);
	entity.m_broadphase = 0;
	m_sorted = false;
}

// Empty the broadphase.
void BPSweepAndPrune::RemoveAll()
{
	PR_ASSERT(PR_DBG_PHYSICS, !m_enumerating, "Do not modify the broadphase while pair enumeration is happening");
	for (BPEntityCont::iterator i = m_entity.begin(), i_end = m_entity.end(); i != i_end; ++i)
		(*i)->m_broadphase = 0;
	m_entity.clear();
}

// Iterate over the colliding pairs
void BPSweepAndPrune::EnumPairs(EnumPairsFunc func, void* context)
{
	PR_DECLARE_PROFILE(PR_PROFILE_BROADPHASE, phBPSnPEnumPairs);
	PR_PROFILE_SCOPE(PR_PROFILE_BROADPHASE, phBPSnPEnumPairs);

	// Can't have pairs with one or zero elements
	if (m_entity.size() <= 1)
		return;

	// Sort the vector of BPEntities
	Sort();

	// Set a debug flag to catch re-entrant use of the broadphase
	// In most cases it will be fine however we cannot resort the entities
	// during enumeration
	PR_EXPAND(PR_DBG_PHYSICS, auto enumer = pr::Scope<void>(
		[&] { m_enumerating = true; },
		[&] { m_enumerating = false; }));

	// Sweep the array looking for overlaps
	v4 sum = v4::Zero(), sum_sq = v4::Zero();
	for (BPEntityCont::const_iterator i = m_entity.begin(), i_end = m_entity.end(); i != i_end; ++i)
	{
		BPEntity const& entityA = **i;
		BBox const& bboxA = *entityA.m_bbox;

		// Compute sums so we can measure the variance of the bbox centres
		sum    += bboxA.Centre();
		sum_sq += Sqr(bboxA.Centre());

		// Scan forward testing for overlap until we find a bbox whose min is greater than entity's max
		for (BPEntityCont::const_iterator j = i + 1; j != i_end; ++j)
		{
			BPEntity const& entityB = **j;
			BBox const& bboxB = *entityB.m_bbox;

			// Stop testing if entityB's min is greater than entityA's max
			if (bboxA.Upper(m_axis) < bboxB.Lower(m_axis))
				break;

			if (intersect::BBoxVsBBox(bboxA, bboxB))
			{
				BPPair pair;
				pair.m_objectA = &entityA;
				pair.m_objectB = &entityB;
				func(pair, context);
			}
		}
	}

	// Compute the variance
	v4 variance = sum_sq - Sqr(sum) / static_cast<float>(m_entity.size());
	m_axis = MaxElementIndex(variance.xyz);
}

// Enumerate all overlaps with 'entity'
void BPSweepAndPrune::EnumPairs(EnumPairsFunc func, BPEntity const& entity, void* context)
{
	PR_DECLARE_PROFILE(PR_PROFILE_BROADPHASE, phBPSnPSingleOverlap);
	PR_PROFILE_SCOPE(PR_PROFILE_BROADPHASE, phBPSnPSingleOverlap);

	// Sort the vector of BPEntities
	if (m_entity.size() > 1)
		Sort();

	// Set a debug flag to catch re-entrant use of the broadphase
	// In most cases it will be fine however we cannot resort the entities during enumeration
	PR_EXPAND(PR_DBG_PHYSICS, auto enumer = pr::Scope<void>(
		[&] { m_enumerating = true; },
		[&] { m_enumerating = false; }));

	BPEntity const& entityB = entity;
	BBox const& bboxB = *entityB.m_bbox;

	// Sweep the array looking for overlaps
	for (BPEntityCont::const_iterator i = m_entity.begin(), i_end = m_entity.end(); i != i_end; ++i)
	{
		BPEntity const& entityA = **i;
		BBox const& bboxA = *entityA.m_bbox;

		// Stop testing if entityB's max is less than entityA's min
		if (bboxB.Upper(m_axis) < bboxA.Lower(m_axis))
			break;

		// If there is an overlap on all axes
		if (bboxA.Upper(m_axis) > bboxB.Lower(m_axis) && intersect::BBoxVsBBox(bboxA, bboxB))
		{
			BPPair pair;
			pair.m_objectA = &entityA;
			pair.m_objectB = &entityB;
			func(pair, context);
		}
	}
}

// Enumerate all overlaps with 'ray'
void BPSweepAndPrune::EnumPairs(EnumPairsFunc func, Ray const& ray, void* context)
{
	PR_DECLARE_PROFILE(PR_PROFILE_BROADPHASE, phBPSnPRay);
	PR_PROFILE_SCOPE(PR_PROFILE_BROADPHASE, phBPSnPRay);

	// Sort the vector of BPEntities
	if (m_entity.size() > 1)
		Sort();

	// Set a debug flag to catch re-entrant use of the broadphase
	// In most cases it will be fine however we cannot resort the entities during enumeration
	PR_EXPAND(PR_DBG_PHYSICS, auto enumer = pr::Scope<void>(
		[&] { m_enumerating = true; },
		[&] { m_enumerating = false; }));

	float ray_max = pr::Max(ray.m_point[m_axis], ray.m_point[m_axis] + ray.m_direction[m_axis]);

	// Sweep the array looking for overlaps with 'ray'
	for (BPEntityCont::const_iterator i = m_entity.begin(), i_end = m_entity.end(); i != i_end; ++i)
	{
		BPEntity const& entityA = **i;
		BBox const& bboxA = *entityA.m_bbox;

		// Stop testing if 'ray_max' is less than entityA's min
		if (ray_max < bboxA.Lower(m_axis))
			break;

		// If there is an overlap on all axes
		if (intersect::LineVsBoundingBox(ray.m_point, ray.m_point + ray.m_direction, bboxA))
		{
			BPPair pair;
			pair.m_objectA   = &entityA;
			pair.m_objB_void = &ray;
			func(pair, context);
		}
	}
}
