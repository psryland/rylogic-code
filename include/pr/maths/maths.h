//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

// Define switches:
// PR_MATHS_USE_DIRECTMATH
// PR_MATHS_USE_INTRINSICS
// Also remember NOMINMAX

#define NEW_MATHS 0

#if NEW_MATHS
#include "pr/math_new/math.h"
namespace pr
{
	using namespace math;

	namespace maths
	{
		namespace spatial
		{
			using namespace ::pr::math::spatial;
		}
	}

	using v2 = math::Vec2<float>;
	using v3 = math::Vec3<float>;
	using v4 = math::Vec4<float>;
	using v8 = math::Vec8<float, void>;
	using iv2 = math::Vec2<int32_t>;
	using iv3 = math::Vec3<int32_t>;
	using iv4 = math::Vec4<int32_t>;
	using quat = math::Quat<float>;
	using xform = math::Xform<float>;
	using m2x2 = math::Mat2x2<float>;
	using m3x4 = math::Mat3x4<float>;
	using m4x4 = math::Mat4x4<float>;
	using m6x8 = math::Mat6x8<float, void, void>;
	using BBox = math::BoundingBox<float>;
	using BSphere = math::BoundingSphere<float>;
	using OBox = math::OrientedBox<float>;
	using Plane = math::Plane3<float>;
	using Frustum = math::Frustum3<float>;

	using v8motion = math::Vec8<float, math::spatial::Motion>;
	using v8force = math::Vec8<float, math::spatial::Force>;
}
#else

#include "pr/common/cast.h"

#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/limits.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector2.h"
#include "pr/maths/vector3.h"
#include "pr/maths/vector4.h"
#include "pr/maths/vector8.h"
#include "pr/maths/quaternion.h"
#include "pr/maths/matrix2x2.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/matrix4x4.h"
#include "pr/maths/matrix6x8.h"
#include "pr/maths/matrix.h"
#include "pr/maths/half.h"
#include "pr/maths/transform.h"
#include "pr/maths/bbox.h"
#include "pr/maths/bsphere.h"
#include "pr/maths/oriented_box.h"
#include "pr/maths/rectangle.h"
#include "pr/maths/plane.h"
#include "pr/maths/frustum.h"
#include "pr/maths/axis_id.h"
#include "pr/maths/conversion.h"
#include "pr/maths/spline.h"
#include "pr/maths/polynomial.h"
#include "pr/maths/spatial.h"
#include "pr/maths/stat.h"
#include "pr/maths/interpolate.h"
#include "pr/maths/dynamics.h"
#include "pr/maths/constants_vector.h"

#endif
