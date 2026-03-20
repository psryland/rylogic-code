//***************************************************
// Bounding Box
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;

namespace Rylogic.Maths
{
	public static partial class Geometry
	{
		/// <summary>
		/// Clip a line segment to a bounding box returning the parametric values of the intersection.
		/// Returns false if the line does not intersect with the bounding box.
		/// Assumes initial values of 't0' and 't1' have been set.</summary>
		public static bool Clip(BBox bbox, v4 point_, v4 direction_, ref float t0, ref float t1)
		{
			// Convert v4's to float arrays for efficient indexing
			var lower = (float[])bbox.Min;
			var upper = (float[])bbox.Max;
			var point = (float[])point_;
			var direction = (float[])direction_;

			// For all three slabs
			for (int i = 0; i != 3; ++i)
			{
				if (Math.Abs(direction[i]) < Math_.TinyF)
				{
					// Ray is parallel to slab. No hit if origin not within slab
					if (point[i] < lower[i] || point[i] > upper[i])
						return false;
				}
				else
				{
					// Compute intersection t value of ray with near and far plane of slab
					float u0 = (lower[i] - point[i]) / direction[i];
					float u1 = (upper[i] - point[i]) / direction[i];

					// Make u0 be intersection with near plane, u1 with far plane
					if (u0 > u1) Math_.Swap(ref u0, ref u1);

					// Record the tightest bounds on the line segment
					if (u0 > t0) t0 = u0;
					if (u1 < t1) t1 = u1;

					// Exit with no collision as soon as slab intersection becomes empty
					if (t0 > t1)
						return false;
				}
			}

			// Ray intersects all 3 slabs
			return true;
		}
	}
}
