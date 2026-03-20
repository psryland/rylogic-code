using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Rylogic.Maths
{
	public static partial class Geometry
	{
		/// <summary>
		/// Return the intercept between a 2d line that passes through 'a' and 'b' and another
		/// that passes through 'c' and 'd'. Returns true if the lines intersect, false if they don't.
		/// Returns the point of intersect in </summary>
		public static bool Intersect(v2 a, v2 b, v2 c, v2 d, out v2 intersect)
		{
			v2 ab = b - a;
			v2 cd = d - c;
			float denom = ab.x * cd.y - ab.y * cd.x;
			if (Math_.FEql(denom, 0.0f))
			{
				intersect = v2.Zero;
				return false;
			}

			float e = b.x * a.y - b.y * a.x;
			float f = d.x * c.y - d.y * c.x;
			intersect.x = (cd.x * e - ab.x * f) / denom;
			intersect.y = (cd.y * e - ab.y * f) / denom;
			return true;
		}
		public static bool Intersect(v2 a, v2 b, v2 c, v2 d)
		{
			v2 intersect;
			return Intersect(a,b,c,d, out intersect);
		}
	}
}
