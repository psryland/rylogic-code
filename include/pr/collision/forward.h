//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <algorithm>
#include <array>
#include <type_traits>
#include <concepts>
#include <cassert>
#include <span>
#include <tuple>

#include "pr/math/math.h"
#include "pr/common/cast.h"
#include "pr/common/scope.h"
#include "pr/common/alloca.h"
#include "pr/common/fmt.h"
#include "pr/container/vector.h"
#include "pr/container/tri_table.h"
#include "pr/geometry/point.h"
#include "pr/geometry/closest_point.h"
#include "pr/geometry/intersect.h"

namespace pr::collision
{
	struct Shape;
	struct ShapeSphere;
	struct ShapeBox;
	struct ShapeLine;
	struct ShapePolytope;
	struct ShapeTriangle;
	struct ShapeArray;
	struct Contact;
	struct Ray;
	struct RayCastResult;
	struct Penetration;
	struct TestPenetration;
	struct MinPenetration;
	struct ContactPenetration;

	// Support features of a collision shape
	enum class EFeature :int
	{
		// Note
		//  - static_cast<int>(EFeature) is used as the number of points returned from 'SupportFeature()'
		None = 0,
		Vert = 1,
		Edge = 2,
		Face = 3, // Anyhing >= 3 is a face
		Tri  = 3,
		Quad = 4,
		// higher order faces are supported
	};
	static constexpr int EFeatureBits = 3;
	static constexpr int EFeatureMask = (1 << EFeatureBits) - 1;
	static constexpr int FeaturePolygonMaxSides = 8;
}

