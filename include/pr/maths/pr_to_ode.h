//*****************************************************************************
// ODE/pr compatibility
//	Copyright (c) Rylogic 2015
//*****************************************************************************

#pragma once

#include <ode/ode.h>
#include "pr/maths/maths.h"

struct dV4
{
	dVector3 m_pos;
	operator dVector3 const&() const { return m_pos; }
};
struct dM4x4
{
	dMatrix3 m_rot;
	dVector3 m_pos;
};

namespace pr
{
	namespace impl
	{
		// Copy vectors/matrices
		template <typename Real1, typename Real2> inline void copy(Real1* dst, Real2 const* src, int count)
		{
			while (count--)
				*dst++ = Real1(*src++);
		}
	}
	
	// Convert a pr::v4 to a dVector3 
	inline dV4 ode(pr::v4 vec)
	{
		dV4 v = {{vec.x, vec.y, vec.z, vec.w}};
		return v;
	}
	
	// Convert a dVector3 to a pr::v4
	inline pr::v4 ode(dVector3 const& vec, float w)
	{
		return pr::v4(vec[0], vec[1], vec[2], w);
	}
	
	// Convert a pr::m4x4 to a dM4x4
	inline dM4x4 ode(pr::m4x4 const& o2w)
	{
		dM4x4 m;
		impl::copy(m.m_rot, Transpose3x3(o2w).x.arr, 12);
		impl::copy(m.m_pos, o2w.pos.arr, 4);
		return m;
	}
	
	// Convert a position and orientation to a pr::m4x4
	inline pr::m4x4 ode(dReal const* pos, dReal const* rot)
	{
		pr::m4x4 o2w;
		impl::copy(o2w.x.arr, rot, 12);
		impl::copy(o2w.pos.arr, pos, 4);
		o2w.x.w = o2w.y.w = o2w.z.w = 0.0f; o2w.w.w = 1.0f;
		return pr::Transpose3x3(o2w);
	}
	
	// Convert a dM4x4 to a pr::m4x4
	inline pr::m4x4 ode(dM4x4 const& o2w)
	{
		return ode(o2w.m_pos, o2w.m_rot);
	}
	
	// Convert ode geometry into pr geometry
	template <int dGeomClass> struct OdeShape {};
	template <> struct OdeShape<dSphereClass>
	{
		static pr::BSphere make(dGeomID geom, pr::m4x4 const& o2w)
		{
			auto radius = dGeomSphereGetRadius(geom);
			return pr::BSphere(o2w.pos, radius);
		}
	};
	template <> struct OdeShape<dBoxClass>
	{
		static pr::OBox make(dGeomID geom, pr::m4x4 const& o2w)
		{
			dVector3 d;
			auto radius = (dGeomBoxGetLengths(geom, d), ode(d,0.0f));
			return pr::OBox(o2w.pos, radius, o2w.rot);
		}
	};
}
