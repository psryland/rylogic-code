//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/types/matrix4x4.h"
#include "pr/math/types/vector4.h"
#include "pr/math/types/vector2.h"
#include "pr/math/primitives/plane.h"
#include "pr/math/primitives/bsphere.h"
#include "pr/math/primitives/bbox.h"

namespace pr::math
{
	template <ScalarTypeFP S>
	struct Frustum3
	{
		// Notes:
		//  - The frustum grows down the negative z axis, i.e. the z value of the apex is more
		//    positive than the far plane. This is because cameras generally look down the -z axis
		//    in right-handed space so that the x axis is to the right and the y axis is up.
		//  - The frustum plane normals are stored with the far plane at (0,0,0). However, it is
		//    more convenient to have the apex at (0,0,0) so the public interface should expect
		//    the apex at (0,0,0) and internally offset by 'zfar'.
		//  - 'zfar' is a positive value when within the frustum and negative when behind, to be
		//    consistent with camera near/far planes. 'zfar' is 'the distance to', not 'the z value of'.
		//  - There is no reason the frustum needs to be symmetrical, any 4 inward normals are valid.
		//    However, the -Z axis is still considered the main axis of the frustum, even if a face
		//    normal has a positive z component.
		//  - This type expects left/right to have no Y component, and bottom/top to have no X
		//    component. There are optimisations based on this assumption.
		//  - "Left" is from the apex looking down the -z axis. The left plane normal of frustum
		//    typically has a positive X component and -Z component.
		//  - There is no 'Frustum3 operator * (Mat4x4, Frustum3)' because this type assumes the -z
		//    is the main axes, and the left/right planes have no Y component, etc. Transforming
		//    the planes doesn't work as expected anyway.
		using Mat4x4 = Mat4x4<S>;
		using Vec4 = Vec4<S>;
		using Vec2 = Vec2<S>;
		using Plane = Plane3<S>;

		// The order of planes in the frustum
		enum class EPlane
		{
			XPos = 0, // left
			XNeg = 1, // right
			YPos = 2, // bottom
			YNeg = 3, // top
			ZFar = 4, // far plane
			NumberOf,
		};

		// The inward pointing planes of the faces of the frustum, transposed.
		Mat4x4 m_Tplanes;

		// Create a frustum from 'width' and 'height' at 'z' from the apex
		static Frustum3 MakeWH(S width, S height, S z, S zfar) noexcept
		{
			pr_assert(z > 0 && "The focus plane should be a positive distance from the apex");
			pr_assert(zfar >= 0 && "The far plane should be a positive distance from the apex");

			Frustum3 f;
			f.m_Tplanes.x = Vec4(+z, 0.0f, -width * 0.5f, 0.0f); // left
			f.m_Tplanes.y = Vec4(-z, 0.0f, -width * 0.5f, 0.0f); // right
			f.m_Tplanes.z = Vec4(0.0f, +z, -height * 0.5f, 0.0f); // bottom
			f.m_Tplanes.w = Vec4(0.0f, -z, -height * 0.5f, 0.0f); // top
			f.m_Tplanes.x = Normalise(f.m_Tplanes.x);
			f.m_Tplanes.y = Normalise(f.m_Tplanes.y);
			f.m_Tplanes.z = Normalise(f.m_Tplanes.z);
			f.m_Tplanes.w = Normalise(f.m_Tplanes.w);
			f.m_Tplanes = Transpose(f.m_Tplanes);
			f.zfar(zfar);
			return f;
		}
		static Frustum3 MakeWH(Vec2 area, S z, S zfar) noexcept
		{
			return MakeWH(area.x, area.y, z, zfar);
		}

		// Create a frustum from vertical 'field-of-view' and an 'aspect' ratio
		static Frustum3 MakeFA(S fovY, S aspect, S zfar) noexcept
		{
			auto h = S(2) * std::tan(S(0.5) * fovY);
			return MakeWH(aspect * h, h, S(1), zfar);
		}

		// Create a frustum from horizontal and vertical fields of view
		static Frustum3 MakeFovXY(S fovX, S fovY, S zfar) noexcept
		{
			auto w = S(2) * std::tan(S(0.5) * fovX);
			auto h = S(2) * std::tan(S(0.5) * fovY);
			return MakeWH(w, h, S(1), zfar);
		}

		// Create an orthographic "frustum" that has the zfar at infinity and parallel side planes
		static Frustum3 MakeOrtho(S width, S height) noexcept
		{
			Frustum3 f;
			f.m_Tplanes.x = Vec4(+1.0f, 0.0f, 0.0f, width * 0.5f); // left
			f.m_Tplanes.y = Vec4(-1.0f, 0.0f, 0.0f, width * 0.5f); // right
			f.m_Tplanes.z = Vec4(0.0f, +1.0, 0.0f, height * 0.5f); // bottom
			f.m_Tplanes.w = Vec4(0.0f, -1.0, 0.0f, height * 0.5f); // top
			f.m_Tplanes = Transpose(f.m_Tplanes);
			return f;
		}
		static Frustum3 MakeOrtho(Vec2 area) noexcept
		{
			return MakeOrtho(area.x, area.y);
		}

		// Create from a projection matrix
		static Frustum3 MakeFromProjection(Mat4x4 const& c2s) noexcept
		{
			Frustum3 f;
			if (c2s.w.w == 1) // If orthographic
			{
				auto w = 2.0f / c2s.x.x;
				auto h = 2.0f / c2s.y.y;
				f = MakeOrtho(w, h);
			}
			else // Otherwise perspective
			{
				auto rh = -Sign(c2s.z.w);
				auto zn = rh * c2s.w.z / c2s.z.z;
				auto zf = zn * c2s.z.z / (rh + c2s.z.z);
				auto w = 2.0f * zn / c2s.x.x;
				auto h = 2.0f * zn / c2s.y.y;
				f = MakeWH(w, h, zn, zf);
			}
			return f;
		}

		// Return the projection matrix for this Frustum3
		Mat4x4 projection(S zn, S zf) const noexcept
		{
			auto wh = area(zn);
			return orthographic()
				? ProjectionOrthographic<Mat4x4>(wh.x, wh.y, zn, zf, true)
				: ProjectionPerspective<Mat4x4>(wh.x, wh.y, zn, zf, true);
		}
		Mat4x4 projection(Vec2 nf) const noexcept
		{
			return projection(nf.x, nf.y);
		}

		// True if this is an orthographic frustum
		constexpr bool orthographic() const noexcept
		{
			// If none of the plane normals have a z component,
			// then they are all parallel to the z axis.
			return m_Tplanes.z == Zero<Vec4>();
		}

		// Get/Set the distance to the far clip plane.
		S zfar() const noexcept
		{
			// Orthographic frusta don't have a far plane
			return !orthographic()
				? -CompSum(m_Tplanes.w / m_Tplanes.z) / S(4)
				: std::numeric_limits<S>::infinity();
		}
		void zfar(S z) noexcept
		{
			if (!orthographic())
				m_Tplanes.w = -z * m_Tplanes.z;
		}

		// Get/Set the X field of view
		S fovX() const noexcept
		{
			// The FovX is the angle between the left/right plane normals: Cos(ang) = dot(l,r)
			auto l = Vec4(m_Tplanes.x.x, m_Tplanes.y.x, m_Tplanes.z.x, 0);
			auto r = Vec4(m_Tplanes.x.y, m_Tplanes.y.y, m_Tplanes.z.y, 0);
			auto fov = std::acos(Clamp(Dot(l, -r), S(-1), S(1)));
			return fov;
		}

		// Get the Y field of view
		S fovY() const noexcept
		{
			// The FovY is the angle between the bottom/top plane normals: Cos(ang) = dot(b,t)
			auto b = Vec4(m_Tplanes.x.z, m_Tplanes.y.z, m_Tplanes.z.z, 0);
			auto t = Vec4(m_Tplanes.x.w, m_Tplanes.y.w, m_Tplanes.z.w, 0);
			auto fov = std::acos(Clamp(Dot(b, -t), S(-1), S(1)));
			return fov;
		}

		// Get/Set the aspect ratio for the frustum
		constexpr S aspect() const noexcept
		{
			// Not using width/height here because if zfar is zero it will be
			// 0/0 even though the aspect ratio is still actually valid.
			if (orthographic())
			{
				return
					(m_Tplanes.w.x + m_Tplanes.w.y) /
					(m_Tplanes.w.z + m_Tplanes.w.w);
			}
			else
			{
				return
					(m_Tplanes.z.y / m_Tplanes.x.y - m_Tplanes.z.x / m_Tplanes.x.x) /
					(m_Tplanes.z.w / m_Tplanes.y.w - m_Tplanes.z.z / m_Tplanes.y.z);
			}
		}

		// Get the frustum width/height at 'z' distance from the apex.
		Vec2 area(S z) const noexcept
		{
			auto w = m_Tplanes.w.x / m_Tplanes.x.x - m_Tplanes.w.y / m_Tplanes.x.y;
			auto h = m_Tplanes.w.z / m_Tplanes.y.z - m_Tplanes.w.w / m_Tplanes.y.w;
			if (orthographic())
			{
				return Vec2(w, h);
			}
			else
			{
				auto z0 = zfar();
				return z0 != 0 ? Vec2(w * z / z0, h * z / z0) : Zero<Vec2>();
			}
		}

		// Return the planes of the frustum
		constexpr Mat4x4 planes() const noexcept
		{
			return Transpose(m_Tplanes);
		}
		Plane plane(EPlane plane_index) const noexcept
		{
			switch (plane_index)
			{
				case EPlane::XPos: return Plane(m_Tplanes.x.x, m_Tplanes.y.x, m_Tplanes.z.x, m_Tplanes.w.x);
				case EPlane::XNeg: return Plane(m_Tplanes.x.y, m_Tplanes.y.y, m_Tplanes.z.y, m_Tplanes.w.y);
				case EPlane::YPos: return Plane(m_Tplanes.x.z, m_Tplanes.y.z, m_Tplanes.z.z, m_Tplanes.w.z);
				case EPlane::YNeg: return Plane(m_Tplanes.x.w, m_Tplanes.y.w, m_Tplanes.z.w, m_Tplanes.w.w);
				case EPlane::ZFar: return Plane{0, 0, 1, 0};
				default: pr_assert(false && "Invalid plane index"); return Plane{ 0, 0, 1, 0 };
			}
		}

		// Return a matrix containing the inward pointing face normals or the frustum sides.
		// Order is the same as EPlane (i.e. x=left, y=right, z=bottom, w=top). The far plane is always (0,0,1,0)
		constexpr Mat4x4 face_normals() const noexcept
		{
			auto norms = m_Tplanes;
			norms.w = Zero<Vec4>();
			return Transpose(norms);
		}
		constexpr Vec4 face_normal(EPlane plane_index) const noexcept
		{
			auto p = static_cast<Vec4>(plane(plane_index));
			return p.w0();
		}

		// Return a matrix containing the direction vectors of the frustum edges. These are the four ray directions
		// that start at the camera and lie at the intersections of the frustum planes.
		Mat4x4 edges() const noexcept
		{
			// Direction vectors for the corners
			auto norms = face_normals();
			return Mat4x4(
				Cross(norms.z, norms.x),  // BL
				Cross(norms.x, norms.w),  // TL
				Cross(norms.w, norms.y),  // TR
				Cross(norms.y, norms.z)); // BR
		}

		// Clip the infinite line that passes through 's' with direction 'd' to this frustum.
		// 's' and 'd' must be in frustum space where the frustum apex is at (0,0,0) and grows down the -z axis. (i.e. camera space).
		// 'accumulative' means 't0' and 't1' are expected to be initialised already, otherwise they are initialised to an infinite line.
		// Returns the parametric values 't0' and 't1' and true if t0 < t1 i.e. some of the line is within the frustum.
		bool pr_vectorcall clip(Vec4 const& s, Vec4 const& d, bool accumulative, S& t0, S& t1, bool include_zfar) const noexcept
		{
			// Frustum3 stores the clip planes such that the far plane is actually at (0,0,0) and the apex is at zfar along the +Z axis.
			// Shift 's' and 'd' by 'zfar' so that callers can treat the frustum apex as if it was at (0,0,0).
			auto z = zfar();
			auto a = s + Vec4(0, 0, z, 0);
			auto b = a + d;

			// Initialise the parametric values if this is not an accumulative clip
			if (!accumulative)
			{
				t0 = std::numeric_limits<S>::lowest();
				t1 = std::numeric_limits<S>::max();
			}

			// Clip to the far plane. Orthographic frustums don't have a far plane and are really a rectilinear channel.
			if (include_zfar && z != 0)
			{
				// If the line is not parallel to the far plane
				if (!FEql(a.z, b.z))
				{
					if (b.z > a.z) t0 = Max(t0, -a.z / (b.z - a.z));
					else           t1 = Min(t1, -a.z / (b.z - a.z));
				}
				else if (a.z < 0)
				{
					t1 = t0;
					return false;
				}
			}

			// Get the dot products of a section of the line agains the frustum planes
			auto d0 = (*this) * a;
			auto d1 = (*this) * b;
			auto interval = d1 - d0;

			// Reduce the parametric interval
			for (int i = 0; i != 4; ++i)
			{
				// If the line is not parallel to the far plane
				if (!FEql(interval[i], S(0)))
				{
					if (d1[i] > d0[i]) t0 = Max(t0, -d0[i] / interval[i]);
					else               t1 = Min(t1, -d0[i] / interval[i]);
				}
				// If behind the plane, then wholly clipped
				else if (d0[i] < 0)
				{
					t1 = t0;
					break;
				}
			}

			// Return true if any portion of the line is within the frustum
			return t0 < t1;
		}

		#pragma region Operators
		friend bool operator == (Frustum3 const& lhs, Frustum3 const& rhs) noexcept
		{
			return memcmp(&lhs, &rhs, sizeof(lhs)) == 0;
		}
		friend bool operator != (Frustum3 const& lhs, Frustum3 const& rhs) noexcept
		{
			return memcmp(&lhs, &rhs, sizeof(lhs)) != 0;
		}
		friend bool operator <  (Frustum3 const& lhs, Frustum3 const& rhs) noexcept
		{
			return memcmp(&lhs, &rhs, sizeof(lhs)) < 0;
		}
		friend bool operator >  (Frustum3 const& lhs, Frustum3 const& rhs) noexcept
		{
			return memcmp(&lhs, &rhs, sizeof(lhs)) > 0;
		}
		friend bool operator <= (Frustum3 const& lhs, Frustum3 const& rhs) noexcept
		{
			return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0;
		}
		friend bool operator >= (Frustum3 const& lhs, Frustum3 const& rhs) noexcept
		{
			return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0;
		}
		friend Vec4 pr_vectorcall operator * (Frustum3 const& lhs, Vec4 rhs) noexcept
		{
			// Returns the signed distance of 'rhs' from each face of the frustum
			return lhs.m_Tplanes * rhs;
		}
		// Do not implement Frustum3 = Mat4x4 * Frustum3. Frustums cannot be rotated or moved.
		#pragma endregion
	};

	// Floating point comparisons. *WARNING* 'tol' is an absolute tolerance. Returns true if a is in the range (b-tol,b+tol)
	template <ScalarTypeFP S>
	constexpr bool FEqlAbsolute(Frustum3<S> const& a, Frustum3<S> const& b, S tol) noexcept
	{
		return FEqlAbsolute(a.m_Tplanes, b.m_Tplanes, tol);
	}

	// *WARNING* 'tol' is a relative tolerance, relative to the largest of 'a' or 'b'
	template <ScalarTypeFP S>
	constexpr bool FEqlRelative(Frustum3<S> const& a, Frustum3<S> const& b, S tol) noexcept
	{
		return FEqlRelative<>(a.m_Tplanes, b.m_Tplanes, tol);
	}

	// FEqlRelative using 'tinyf'. Returns true if a in the range (b - max(a,b)*tiny, b + max(a,b)*tiny)
	template <ScalarTypeFP S>
	constexpr bool FEql(Frustum3<S> const& a, Frustum3<S> const& b) noexcept
	{
		// Don't add a 'tol' parameter because it looks like the function should perform a == b +- tol, which isn't what it does.
		return FEql<>(a.m_Tplanes, b.m_Tplanes);
	}

	// Returns the corners of the frustum (in frustum space) at a given 'z' distance (i.e. apex at (0,0,0). Far plane at (0,0,-zfar)). Return order: x=lb, y=lt, z=rt, w=rb
	template <ScalarTypeFP S>
	inline Mat4x4<S> Corners(Frustum3<S> const& frustum, S z) noexcept
	{
		pr_assert(z >= 0 && "'z' should be a positive distance from the apex");
		auto edges = frustum.edges();
		
		if (frustum.orthographic())
		{
			return Mat4x4<S>(
				Vec4<S>(-frustum.m_Tplanes.w.x, -frustum.m_Tplanes.w.z, -z, 1),
				Vec4<S>(-frustum.m_Tplanes.w.x, +frustum.m_Tplanes.w.w, -z, 1),
				Vec4<S>(+frustum.m_Tplanes.w.y, +frustum.m_Tplanes.w.w, -z, 1),
				Vec4<S>(+frustum.m_Tplanes.w.y, -frustum.m_Tplanes.w.z, -z, 1));
		}
		else
		{
			// Each edge vector has length == 1. Find the length of each edge
			// when projected onto the Z axis then scale by z.
			auto lengths = Transpose(edges) * Vec4<S>(0, 0, -1, 0);
			auto s = z / lengths;
			auto origin = Vec4<S>(0, 0, 0, 1);
			auto corners = Mat4x4<S>(edges.x * s.x + origin, edges.y * s.y + origin, edges.z * s.z + origin, edges.w * s.w + origin);
			return corners;
		}
	}

	// Returns the corners of the frustum at the zfar plane.
	template <ScalarTypeFP S>
	inline Mat4x4<S> Corners(Frustum3<S> const& frustum) noexcept
	{
		// Warning: calling this on orthographic frusta is probably a bug.. unless Z isn't used.
		return Corners(frustum, frustum.zfar());
	}

	// Return true if any part of a point, box, or sphere is within 'frustum'.
	// Use 'zf == 0' to use the frustum's far plane, or 'zf == -1' to ignore the far plane.
	template <ScalarTypeFP S> inline bool pr_vectorcall IsWithin(Frustum3<S> const& frustum, Vec4<S> point, S radius, Vec2<S> nf = Zero<Vec2<S>>()) noexcept
	{
		auto pt = point;
		auto frustum_apex = S(0);
		S znear = 0, zfar = 0;

		// Orthographic frusta don't have a zfar distance
		if (!frustum.orthographic())
		{
			// Remember zn and zf are "the distance to", not "the z coordinate of"
			frustum_apex = frustum.zfar();

			// Shift 'pt' so that the frustum apex is at (0,0,frustum_apex)
			pt.z += frustum_apex;

			// Get the z coordinate of the clip planes
			znear = frustum_apex - nf.x;
			zfar = nf.y > 0 ? frustum_apex - nf.y : nf.y == 0 ? S(0) : -std::numeric_limits<S>::infinity();
		}
		// Orthographic frusta have their "apex" at (0,0,0)
		else
		{
			// Get the z coordinate of the clip planes
			znear = -nf.x;
			zfar = nf.y > 0 ? -nf.y : -std::numeric_limits<S>::infinity();
		}

		// Test against the near plane
		if (pt.z - radius > znear)
			return false;

		// Test against the far plane (only if given)
		if (pt.z + radius < zfar)
			return false;

		// Dot product of 'point' with each plane gives the signed distance to each plane.
		// Increase each distance by 'radius'. This is not strictly correct because we're
		// effectively expanding the frustum by 'radius' not the sphere. It doesn't work near edges.
		auto dots = frustum.m_Tplanes * pt + Vec4<S>(radius);

		// If all dot products are >= 0 then some part of the sphere is within the frustum
		return dots == Abs(dots); 
	}
	template <ScalarTypeFP S> inline bool pr_vectorcall IsWithin(Frustum3<S> const& frustum, BoundingSphere<S> bsphere, Vec2<S> nf = Zero<Vec2<S>>()) noexcept
	{
		pr_assert(bsphere.valid() && "Invalid bsphere used in 'IsWithin' test against frustum");
		return IsWithin(frustum, bsphere.Centre(), bsphere.Radius(), nf);
	}
	template <ScalarTypeFP S> inline bool pr_vectorcall IsWithin(Frustum3<S> const& frustum, BoundingBox<S> bbox, Vec2<S> nf = Zero<Vec2<S>>()) noexcept
	{
		pr_assert(bbox.valid() && "Invalid bbox used in 'IsWithin' test against frustum");

		auto bb = bbox;
		auto frustum_apex = S(0);
		S znear = 0, zfar = 0;

		// Orthographic frusta don't have a zfar distance
		if (!frustum.orthographic())
		{
			// Remember zn and zf are "the distance to", not "the z coordinate of"
			frustum_apex = frustum.zfar();

			// Shift 'bbox' so that the frustum apex is at (0,0,frustum_apex)
			bb.m_centre.z += frustum_apex;

			// Get the z coordinate of the clip planes
			znear = frustum_apex - nf.x;
			zfar = nf.y > 0 ? frustum_apex - nf.y : nf.y == 0 ? S(0) : -std::numeric_limits<S>::infinity();
		}
		// Orthographic frusta have their "apex" at (0,0,0)
		else
		{
			// Get the z coordinate of the clip planes
			znear = -nf.x;
			zfar = nf.y > 0 ? -nf.y : -std::numeric_limits<S>::infinity();
		}

		// Test against the near plane
		if (bb.LowerZ() > znear)
			return false;

		// Test against the far plane (only if given)
		if (bb.UpperZ() < zfar)
			return false;

		// The bbox and frustum are both axis aligned, so the test is basically a 2D quad intersection test.
		// Only need to test the cross section of the bbox and the frustum at the minimum z value.
		auto z = Clamp(bb.LowerZ(), zfar, znear);
		auto wh = S(0.5) * frustum.area(frustum_apex - z);

		// This assumes the frustum is symmetric....
		return
			bb.LowerX() >= -wh.x && bb.UpperX() <= +wh.x &&
			bb.LowerY() >= -wh.y && bb.UpperY() <= +wh.y;
	}

	// Grow a frustum (i.e. move it along +f2w.Z growing zfar while preserving fov/aspect) so that 'ws_pt','ws_bbox', or 'ws_sphere' are within the frustum.
	template <ScalarTypeFP S> inline void pr_vectorcall Grow(Frustum3<S>& frustum, Mat4x4<S>& f2w, Vec2<S>& nf, Vec4<S> ws_pt, S radius) noexcept
	{
		// By similar triangles:
		//   zfar1 / zfar0 = (n.w + Dot(n,pt)) / n.w
		// where:
		//   zfar0 = the current zfar distance
		//   zfar1 = the new zfar distance needed to enclose 'pt'
		//   n.w   = is a frustum plane distance from the origin
		//   n     = is a frustum plane 4-vector
		// However, zfar0 can be 0.0.
		// So instead, make a copy of 'frustum' and set zfar0 to 1.0.
		// Calculate the zfar1 value for all planes and choose the maximum.
		pr_assert(!frustum.orthographic() && "No amount of shifting along z can change what is within an orthographic frustum");

		// The caller assumes (0,0,0) is the apex and the far plane is (0,0,-zfar).
		// Transform 'ws_pt' to frustum space, then offset to be relative to (0,0,zfar)
		auto frustum_zfar = frustum.zfar();
		auto pt = InvertOrthonormal(f2w) * ws_pt;
		pt.z += frustum_zfar;
		
		// If 'pt' is beyond the far plane, extend the far plane
		if (pt.z - radius < 0)
		{
			frustum_zfar -= pt.z - radius;
			pt.z = radius;
		}

		// Take a copy of 'frustum' and set its zfar to 1.0 (i.e. zfar0 == 1)
		auto tnorms = frustum.m_Tplanes;
		tnorms.w = -tnorms.z;

		// Get the signed distance to all frustum planes
		auto planes = Transpose(tnorms);
		auto dst = Vec4<S>{
			Dot(planes.x, pt - radius * planes.x.w0()),
			Dot(planes.y, pt - radius * planes.y.w0()),
			Dot(planes.z, pt - radius * planes.z.w0()),
			Dot(planes.w, pt - radius * planes.w.w0()),
		};

		// Get the new zfar distance according to each plane
		auto zfar4 = (tnorms.w - dst) / tnorms.w;
		auto zfar = MaxElement(zfar4);
		auto dzfar = Max(S(0), zfar - frustum_zfar);
		frustum_zfar += dzfar;
		frustum.zfar(frustum_zfar);

		// Update the f2w transform and clip planes
		f2w.pos += dzfar * f2w.z;
		nf.x = Min(nf.x + dzfar, frustum_zfar - (pt.z + radius));
		nf.y = Max(nf.y + dzfar, frustum_zfar - (pt.z - radius));
	}
	template <ScalarTypeFP S> inline void pr_vectorcall Grow(Frustum3<S>& frustum, Mat4x4<S>& f2w, Vec2<S>& nf, BoundingSphere<S> ws_bsphere) noexcept
	{
		pr_assert(!frustum.orthographic() && "No amount of shifting along z can change what is within an orthographic frustum");
		return Grow(frustum, f2w, nf, ws_bsphere.Centre(), ws_bsphere.Radius());
	}
	template <ScalarTypeFP S> inline void pr_vectorcall Grow(Frustum3<S>& frustum, Mat4x4<S>& f2w, Vec2<S>& nf, BoundingBox<S> ws_bbox) noexcept
	{
		pr_assert(!frustum.orthographic() && "No amount of shifting along z can change what is within an orthographic frustum");

		// The caller assumes (0,0,0) is the apex and the far plane is (0,0,-zfar).
		// Transform 'ws_bbox' to frustum space, then offset to be relative to (0,0,zfar)
		auto frustum_zfar = frustum.zfar();
		auto bbox = InvertOrthonormal(f2w) * ws_bbox;
		bbox.m_centre.z += frustum_zfar;
		
		// If 'pt' is beyond the far plane, extend the far plane
		auto pt = SupportPoint(bbox, -Vec4<S>(0, 0, 1, 0));
		if (pt.z < 0)
		{
			frustum_zfar -= pt.z;
			bbox.m_centre.z = bbox.m_radius.z;
		}

		// Take a copy of 'frustum' and set its zfar to 1.0 (i.e. zfar0 == 1)
		auto tnorms = frustum.m_Tplanes;
		tnorms.w = -tnorms.z;

		// Get the signed distance to all frustum planes
		auto planes = Transpose(tnorms);
		auto dst = Vec4<S>{
			Dot(planes.x, SupportPoint(bbox, -planes.x.w0())),
			Dot(planes.y, SupportPoint(bbox, -planes.y.w0())),
			Dot(planes.z, SupportPoint(bbox, -planes.z.w0())),
			Dot(planes.w, SupportPoint(bbox, -planes.w.w0())),
		};

		// Get the new zfar distance according to each plane
		auto zfar4 = (tnorms.w - dst) / tnorms.w;
		auto zfar = MaxElement(zfar4);
		auto dzfar = Max(S(0), zfar - frustum_zfar);
		frustum_zfar += dzfar;
		frustum.zfar(frustum_zfar);

		// Update the f2w transform
		f2w.pos += dzfar * f2w.z;
		nf.x = Min(nf.x + dzfar, frustum_zfar - bbox.UpperZ());
		nf.y = Max(nf.y + dzfar, frustum_zfar - bbox.LowerZ());
	}
	template <ScalarTypeFP S> inline void pr_vectorcall Grow(Frustum3<S>& frustum, Mat4x4<S>& f2w, Vec4<S> ws_pt, S radius) noexcept
	{
		auto nf = Zero<Vec2<S>>();
		Grow(frustum, f2w, nf, ws_pt, radius);
	}
	template <ScalarTypeFP S> inline void pr_vectorcall Grow(Frustum3<S>& frustum, Mat4x4<S>& f2w, BoundingSphere<S> ws_bsphere) noexcept
	{
		auto nf = Zero<Vec2<S>>();
		Grow(frustum, f2w, nf, ws_bsphere);
	}
	template <ScalarTypeFP S> inline void pr_vectorcall Grow(Frustum3<S>& frustum, Mat4x4<S>& f2w, BoundingBox<S> ws_bbox) noexcept
	{
		auto nf = Zero<Vec2<S>>();
		Grow(frustum, f2w, nf, ws_bbox);
	}

	// Include 'f2w * frustum' in 'bbox'
	template <ScalarTypeFP S> inline BoundingBox<S>& pr_vectorcall Grow(BoundingBox<S>& bbox, Frustum3<S> const& frustum, Mat4x4<S> const& f2w = Identity<Mat4x4<S>>()) noexcept
	{
		auto corner = Corners(frustum);
		Grow(bbox, f2w.pos);
		Grow(bbox, f2w * corner.x);
		Grow(bbox, f2w * corner.y);
		Grow(bbox, f2w * corner.z);
		Grow(bbox, f2w * corner.w);
		return bbox;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(FrustumTests)
	{
		std::default_random_engine rng;
		TestClass_FrustumTests()
			:rng(5)
		{}

		PRUnitTestMethod(ProjectionRoundTrip, float, double)
		{
			auto f = Frustum3<T>::MakeFA(constants<T>::tau_by_8, T(1.75), T(10.0));
			auto p = f.projection(T(2), T(10));
			auto F = Frustum3<T>::MakeFromProjection(p);
			auto P = F.projection(T(2), T(10));

			PR_EXPECT(FEql(F, f));
			PR_EXPECT(FEql(P, p));
		}
		PRUnitTestMethod(FAFrustum, float, double)
		{
			auto f = Frustum3<T>::MakeFA(constants<T>::tau_by_8, T(1.75), T(10.0));
			
			// ZDist
			f.zfar(T(5));
			PR_EXPECT(f.zfar() == T(5));
			f.zfar(T(10));
			PR_EXPECT(f.zfar() == T(10));

			// aspect = tan(fovX/2) / tan(fovY/2)
			PR_EXPECT(FEql(f.aspect(), T(1.75)));

			// FOV
			PR_EXPECT(FEql(f.fovY(), constants<T>::tau_by_8));
			PR_EXPECT(FEql(f.fovX(), T(2) * std::atan(f.aspect() * std::tan(f.fovY() / T(2)))));

			// width/height
			auto wh = f.area(T(1));
			PR_EXPECT(FEql(wh.x, T(2) * std::tan(f.fovX() / T(2))));
			PR_EXPECT(FEql(wh.y, T(2) * std::tan(f.fovY() / T(2))));

			f.zfar(T(2));

			// IsWithin
			// In front/behind test
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-1), 1}, T(0)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{0, 0, T(0.1), 1}, T(0)));

			// ZFar test
			PR_EXPECT(!IsWithin(f, Vec4<T>{0, 0, T(-2.1), 1}, T(0)));
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-2.1), 1}, T(0), Vec2<T>{0, -1}));

			// Random points
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.4), T(-0.2), T(-0.8), 1}, T(0)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{T(-0.3), T(+0.4), T(-0.8), 1}, T(0)));

			// Radius test
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.6), T(-0.4), T(-0.8), 1}, T(0.07)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{T(+0.6), T(-0.4), T(-0.8), 1}, T(0.06)));

			// GetCorners
			auto corners = Corners(f, T(1));
			auto tol = T(0.0001);
			PR_EXPECT(FEqlAbsolute(corners.x, Vec4<T>(T(-0.724874), T(-0.414214), T(-1), 1), tol));
			PR_EXPECT(FEqlAbsolute(corners.y, Vec4<T>(T(-0.724874), T(+0.414214), T(-1), 1), tol));
			PR_EXPECT(FEqlAbsolute(corners.z, Vec4<T>(T(+0.724874), T(+0.414214), T(-1), 1), tol));
			PR_EXPECT(FEqlAbsolute(corners.w, Vec4<T>(T(+0.724874), T(-0.414214), T(-1), 1), tol));
		}
		PRUnitTestMethod(WHFrustum, float, double)
		{
			auto f = Frustum3<T>::MakeWH(Vec2<T>(T(16), T(9)), T(10), T(20));
			
			// ZDist
			PR_EXPECT(f.zfar() == T(20));

			// aspect = tan(fovX/2) / tan(fovY/2)
			PR_EXPECT(FEql(f.aspect(), T(16) / T(9)));

			// FOV
			PR_EXPECT(FEql(f.fovX(), T(2) * std::atan(T(8) / T(10))));
			PR_EXPECT(FEql(f.fovY(), T(2) * std::atan(T(4.5) / T(10))));

			// width/height
			auto wh = f.area(T(1));
			PR_EXPECT(FEql(wh.x, T(1.6)));
			PR_EXPECT(FEql(wh.y, T(0.9)));

			f.zfar(T(2));

			// IsWithin
			// In front/behind test
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-1), 1}, T(0)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{0, 0, T(+0.1), 1}, T(0)));
			
			// ZFar test
			PR_EXPECT(!IsWithin(f, Vec4<T>{0, 0, T(-2.1), 1}, T(0)));
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-2.1), 1}, T(0), Vec2<T>{0, -1}));
			
			// Random points
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.6), T(-0.2), T(-0.8), 1}, T(0)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{T(+0.8), T(-0.5), T(-0.7), 1}, T(0)));
			
			// Radius test
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.8), T(-0.5), T(-0.7), 1}, T(0.23)));
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.8), T(-0.5), T(-0.7), 1}, T(0.20))); // This should be false but isn't because 'radius' actually expands the frustum not the sphere.
			
			// GetCorners
			auto corners = Corners(f, T(2));
			PR_EXPECT(FEql(corners.x, Vec4<T>(T(-1.6), T(-0.9), T(-2), 1)));
			PR_EXPECT(FEql(corners.y, Vec4<T>(T(-1.6), T(+0.9), T(-2), 1)));
			PR_EXPECT(FEql(corners.z, Vec4<T>(T(+1.6), T(+0.9), T(-2), 1)));
			PR_EXPECT(FEql(corners.w, Vec4<T>(T(+1.6), T(-0.9), T(-2), 1)));
		}
		PRUnitTestMethod(OrthoFrustum, float, double)
		{
			auto f = Frustum3<T>::MakeOrtho(Vec2<T>(T(1.6), T(0.9)));

			// zdist
			PR_EXPECT(FEql(f.zfar(), std::numeric_limits<T>::infinity()));

			// aspect = tan(fovX/2) / tan(fovY/2)
			PR_EXPECT(FEql(f.aspect(), T(1.6) / T(0.9)));

			// fov
			PR_EXPECT(FEql(f.fovX(), T(0)));
			PR_EXPECT(FEql(f.fovY(), T(0)));

			// width/height
			auto wh = f.area(T(1));
			PR_EXPECT(FEql(wh.x, T(1.6)));
			PR_EXPECT(FEql(wh.y, T(0.9)));

			// IsWithin
			// In front/behind test
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-1), 1}, T(0)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{0, 0, T(+0.1), 1}, T(0)));
			
			// zfar test
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-2.1), 1}, T(0)));
			PR_EXPECT(IsWithin(f, Vec4<T>{0, 0, T(-2.1), 1}, T(0), Vec2<T>{0, -1}));
			
			// Random points
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.3), T(-0.44), T(-0.8), 1}, T(0)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{T(+0.3), T(-0.46), T(-0.8), 1}, T(0)));
			
			// Radius test
			PR_EXPECT(IsWithin(f, Vec4<T>{T(+0.3), T(-0.5), T(-0.8), 1}, T(0.06)));
			PR_EXPECT(!IsWithin(f, Vec4<T>{T(+0.3), T(-0.5), T(-0.8), 1}, T(0.04)));

			// GetCorners
			auto corners = Corners(f, T(2));
			PR_EXPECT(FEql(corners.x, Vec4<T>(T(-0.8), T(-0.45), T(-2), 1)));
			PR_EXPECT(FEql(corners.y, Vec4<T>(T(-0.8), T(+0.45), T(-2), 1)));
			PR_EXPECT(FEql(corners.z, Vec4<T>(T(+0.8), T(+0.45), T(-2), 1)));
			PR_EXPECT(FEql(corners.w, Vec4<T>(T(+0.8), T(-0.45), T(-2), 1)));
		}
		PRUnitTestMethod(GrowWithPoints, float, double)
		{
			using Vec4 = Vec4<T>;
			using Vec2 = Vec2<T>;
			using Mat4x4 = Mat4x4<T>;

			std::vector<Vec4> pts;
			for (int i = 0; i != 50; ++i)
			{
				auto pt = Random<Vec4>(rng, Vec4(T(-2), T(-2), T(-2), T(1)), Vec4(T(2), T(2), T(2), T(1)));
				pts.push_back(pt);
			}

			auto nf = Zero<Vec2>();
			auto f2w = Identity<Mat4x4>();
			auto f = Frustum3<T>::MakeFA(constants<T>::tau_by_8, T(1), T(0));
			PR_EXPECT(f.zfar() == T(0));

			for (auto& pt : pts)
			{
				Grow(f, f2w, nf, pt, T(0));
			}
			//{
			//	ldr::Builder b;
			//	b.Frustum3("f", 0x8000FF00).frustum(f).nf(nf).o2w(f2w);
			//	for (auto& pt : pts)
			//		b.Sphere("s", 0xFFFF0000).r(0.05).pos(pt);
			//	b.Write("P:\\dump\\frustum.ldr");
			//}
			for (auto& pt : pts)
			{
				auto within = IsWithin(f, InvertOrthonormal(f2w) * pt, T(0.001), nf);
				PR_EXPECT(within);
			}
		}
		PRUnitTestMethod(GrowWithBBoxes, float, double)
		{
			using Vec4 = Vec4<T>;
			using Vec2 = Vec2<T>;
			using Mat4x4 = Mat4x4<T>;
			using BBoxT = BoundingBox<T>;

			std::vector<BBoxT> bboxes;
			for (int i = 0; i != 50; ++i)
			{
				auto hi = T(i) * T(0.1);
				auto centre = Random<Vec4>(rng, Vec4(-hi, -hi, -hi, T(1)), Vec4(hi, hi, hi, T(1)));
				auto radius = Abs(Random<Vec4>(rng, Vec4(T(0.3), T(0.3), T(0.3), T(0)), Vec4(T(0.5), T(0.5), T(0.5), T(0))));
				bboxes.push_back(BBoxT(centre, radius));
			}

			auto nf = Zero<Vec2>();
			auto f2w = Identity<Mat4x4>();
			auto f = Frustum3<T>::MakeFA(constants<T>::tau_by_8, T(1), T(0));
			PR_EXPECT(f.zfar() == T(0));
			for (auto& bb : bboxes)
			{
				Grow(f, f2w, nf, bb);
			}
			//{
			//	ldr::Builder b;
			//	b.Frustum3("f", 0x8000FF00).frustum(f).nf(nf).o2w(f2w);
			//	for (auto& bb : bboxes)
			//		b.Box("s", 0xFFFF0000).bbox(bb);
			//	b.Write("P:\\dump\\frustum.ldr");
			//}
			for (auto& bb : bboxes)
			{
				auto within = IsWithin(f, InvertOrthonormal(f2w) * bb, nf);
				PR_EXPECT(within);
			}
		}
		PRUnitTestMethod(GrowWithSpheres, float, double)
		{
			using Vec4 = Vec4<T>;
			using Vec2 = Vec2<T>;
			using Mat4x4 = Mat4x4<T>;
			using BSphereT = BoundingSphere<T>;

			std::uniform_real_distribution<T> rdist(T(0.3), T(0.5));
			std::vector<BSphereT> spheres;
			for (int i = 0; i != 50; ++i)
			{
				auto hi = T(i) * T(0.2);
				auto centre = Random<Vec4>(rng, Vec4(-hi, -hi, -hi, T(1)), Vec4(hi, hi, hi, T(1)));
				spheres.push_back(BSphereT(centre, rdist(rng)));
			}

			auto nf = Zero<Vec2>();
			auto f2w = Identity<Mat4x4>();
			auto f = Frustum3<T>::MakeFA(constants<T>::tau_by_8, T(1), T(0));
			PR_EXPECT(f.zfar() == T(0));
			for (auto& bs : spheres)
			{
				Grow(f, f2w, nf, bs);
			}
			//{
			//	ldr::Builder b;
			//	b.Frustum3("f", 0x8000FF00).frustum(f).nf(nf).o2w(f2w);
			//	for (auto& bs : spheres)
			//		b.Sphere("s", 0xFFFF0000).bsphere(bs);
			//	b.Write("P:\\dump\\frustum.ldr");
			//}
			for (auto& bs : spheres)
			{
				auto within = IsWithin(f, InvertOrthonormal(f2w) * bs, nf);
				PR_EXPECT(within);
			}
		}
		template <ScalarTypeFP S> void DumpFrustum(Frustum3<S> const& f, Mat4x4<S> const& f2w) noexcept
		{
			(void)f,f2w;
			//ldr::Builder b;
			//b.Frustum3("f", 0xFF00FF00).frustum(f).o2w(f2w);
			//b.Write("P:\\dump\\frustum.ldr");
			//auto c = GetCorners(f);
			//
			//std::string s;
			//ldr::Triangle(s, "left", 0xFFFFFFFF, v4::Origin(), c.x, c.y);
			//ldr::Triangle(s, "top", 0xFFFFFFFF, v4::Origin(), c.y, c.z);
			//ldr::Triangle(s, "right", 0xFFFFFFFF, v4::Origin(), c.z, c.w);
			//ldr::Triangle(s, "bottom", 0xFFFFFFFF, v4::Origin(), c.w, c.x);
			//ldr::Plane(s, "left", 0xFFFFFFFF, f.plane(Frustum3::EPlane::XPos), v4::Origin(), zfar);
			//ldr::Plane(s, "right", 0xFFFFFFFF, f.plane(Frustum3::EPlane::XNeg), v4::Origin(), zfar);
			//ldr::Plane(s, "bottom", 0xFFFFFFFF, f.plane(Frustum3::EPlane::YPos), v4::Origin(), zfar);
			//ldr::Plane(s, "top", 0xFFFFFFFF, f.plane(Frustum3::EPlane::YNeg), v4::Origin(), zfar);
			//ldr::Write(s, "P:\\dump\\frustum.ldr");
		}
	};
}
#endif

