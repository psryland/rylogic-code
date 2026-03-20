//***************************************************
// Bounding Box
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;
using System.Drawing;

namespace Rylogic.Maths
{
	/// <summary>Bounding box</summary>
	[Serializable]
	[DebuggerDisplay("{Description,nq}")]
	public struct BBox
	{
		// Constructors
		public BBox(v4 centre, v4 radius)
		{
			Centre = centre;
			Radius = radius;
		}
		
		/// <summary>Reset to the invalid BBox</summary>
		public void reset()
		{
			this = Reset;
		}

		/// <summary>Reset to a unit cube: C=v4Origin, R=0.5</summary>
		public void unit()
		{
			this = Unit;
		}

		/// <summary>Assign the bounding box centre and radius</summary>
		public void set(v4 centre, v4 radius)
		{
			Centre = centre;
			Radius = radius;
		}

		/// <summary>Construct from min and max points (also works if min > max)</summary>
		public static BBox From(v4 min, v4 max)
		{
			return new BBox((max + min) * 0.5f, Math_.Abs(max - min) * 0.5f);
		}

		/// <summary>Constants</summary>
		public static BBox Unit { get; } = new BBox(v4.Origin, new v4(0.5f, 0.5f, 0.5f, 0));
		public static BBox Reset { get; } = new BBox(v4.Origin, new v4(-1, -1, -1, 0));

		// Binary operators
		public static BBox operator +(BBox lhs, v4 offset) { lhs.Centre += offset; return lhs; }
		public static BBox operator -(BBox lhs, v4 offset) { lhs.Centre -= offset; return lhs; }
		public static BBox operator *(BBox lhs, float s) { lhs.Radius *= s; return lhs; }
		public static BBox operator /(BBox lhs, float s) { lhs.Radius /= s; return lhs; }
		public static BBox operator *(m4x4 m, BBox rhs)
		{
			Debug.Assert(rhs.IsValid, "Transforming an invalid bounding box");
			var bb = new BBox(m.pos, v4.Zero);
			var mat = Math_.Transpose3x3(m);
			bb.Centre.x += Math_.Dot(mat.x, rhs.Centre);
			bb.Radius.x += Math_.Dot(Math_.Abs(mat.x), rhs.Radius);
			bb.Centre.y += Math_.Dot(mat.y, rhs.Centre);
			bb.Radius.y += Math_.Dot(Math_.Abs(mat.y), rhs.Radius);
			bb.Centre.z += Math_.Dot(mat.z, rhs.Centre);
			bb.Radius.z += Math_.Dot(Math_.Abs(mat.z), rhs.Radius);
			return bb;
		}

		// Equality operators
		public static bool operator ==(BBox lhs, BBox rhs) { return lhs.Centre == rhs.Centre; }
		public static bool operator !=(BBox lhs, BBox rhs) { return !(lhs == rhs); }
		public override bool Equals(object? o) { return o is BBox box && box == this; }
		public override int GetHashCode() => new { Centre, Radius }.GetHashCode();

		/// <summary>Returns true if the bounding box represents a point or volume</summary>
		public bool IsValid => Radius.x >= 0f && Radius.y >= 0f && Radius.z >= 0f &&
			Math_.IsFinite(Radius.LengthSq) && Math_.IsFinite(Centre.LengthSq);

		/// <summary>Returns true if this bbox encloses a single point</summary>
		public bool IsPoint => Radius == v4.Zero;

		/// <summary>Minimum X bound</summary>
		public float MinX => Centre.x - Radius.x;

		/// <summary>Maximum X bound</summary>
		public float MaxX => Centre.x + Radius.x;

		/// <summary>Minimum Y bound</summary>
		public float MinY => Centre.y - Radius.y;

		/// <summary>Maximum Y bound</summary>
		public float MaxY => Centre.y + Radius.y;

		/// <summary>Minimum Z bound</summary>
		public float MinZ => Centre.z - Radius.z;

		/// <summary>Maximum Z bound</summary>
		public float MaxZ => Centre.z + Radius.z;

		/// <summary>Returns the lower corner of the bounding box</summary>
		public v4 Min => Centre - Radius;

		/// <summary>Returns the upper corner of the bounding box</summary>
		public v4 Max => Centre + Radius;

		/// <summary>Returns the lower dimension for an axis of the bounding box</summary>
		public float Lower(int axis) => Centre[axis] - Radius[axis];

		/// <summary>Returns the upper dimension of an axis of the bounding box</summary>
		public float Upper(int axis) => Centre[axis] + Radius[axis];

		/// <summary>Gets the x dimension of the bounding box</summary>
		public float SizeX => 2f * Radius.x;

		/// <summary>Gets the y dimension of the bounding box</summary>
		public float SizeY => 2f * Radius.y;

		/// <summary>Gets the z dimension of the bounding box</summary>
		public float SizeZ => 2f * Radius.z;

		/// <summary>Gets the xy plane dimension of the bounding box</summary>
		public float SizeXY => 2f * Math_.Length0(Radius.x, Radius.y);

		/// <summary>Gets the yz plane dimension of the bounding box</summary>
		public float SizeYZ => 2f * Math_.Length0(Radius.y, Radius.z);

		/// <summary>Gets the zx plane dimension of the bounding box</summary>
		public float SizeZX => 2f * Math_.Length0(Radius.z, Radius.x);

		/// <summary>Get/Sets the centre point of the bounding box</summary>
		public v4 Centre;

		/// <summary>Get/Sets the dimensions of the bounding box</summary>
		public v4 Radius;

		/// <summary>Gets the squared length of the diagonal of the bounding box</summary>
		public float DiametreSq => 4.0f * Radius.LengthSq;

		/// <summary>Gets the length of the diagonal of the bounding box</summary>
		public float Diametre => Math_.Sqrt(DiametreSq);

		/// <summary>Gets the volume of the bounding box</summary>
		public float Volume => SizeX * SizeY * SizeZ;

		/// <summary>Returns a corner of the bounding box. 'corner'</summary>
		public v4 GetCorner(uint corner)
		{
			Debug.Assert(corner < 8, "Invalid corner index");
			uint x = ((corner >> 0) & 0x1) * 2 - 1;
			uint y = ((corner >> 1) & 0x1) * 2 - 1;
			uint z = ((corner >> 2) & 0x1) * 2 - 1;
			return new v4(Centre.x + x*Radius.x, Centre.y + y*Radius.y, Centre.z + z*Radius.z, 1.0f);
		}

		/// <summary>Returns true if 'point' is within this bounding box (within 'tol'erance)</summary>
		public bool IsWithin(v4 point, float tol)
		{
			return
				Math.Abs(point.x - Centre.x) <= Radius.x + tol &&
				Math.Abs(point.y - Centre.y) <= Radius.y + tol &&
				Math.Abs(point.z - Centre.z) <= Radius.z + tol;
		}

		/// <summary>Returns true if 'bbox' is within this bounding box (within 'tol'erance)</summary>
		public bool IsWithin(BBox bbox, float tol)
		{
			return
				Math.Abs(bbox.Centre.x - Centre.x) <= (Radius.x - bbox.Radius.x + tol) &&
				Math.Abs(bbox.Centre.y - Centre.y) <= (Radius.y - bbox.Radius.y + tol) &&
				Math.Abs(bbox.Centre.z - Centre.z) <= (Radius.z - bbox.Radius.z + tol);
		}

		/// <summary>Returns true if 'rhs' intersects with this bounding box</summary>
		public static bool IsIntersection(BBox lhs, BBox rhs)
		{
			return
				Math.Abs(lhs.Centre.x - rhs.Centre.x) <= (lhs.Radius.x + rhs.Radius.x) &&
				Math.Abs(lhs.Centre.y - rhs.Centre.y) <= (lhs.Radius.y + rhs.Radius.y) &&
				Math.Abs(lhs.Centre.z - rhs.Centre.z) <= (lhs.Radius.z + rhs.Radius.z);
		}

		/// <summary>Returns a bbox that encloses 'lhs' and 'point'</summary>
		public static BBox Union(BBox bbox, v4 point)
		{
			var bb = bbox;
			for (int i = 0; i != 3; ++i)
			{
				if (bb.Radius[i] < 0.0f)
				{
					bb.Centre[i] = point[i];
					bb.Radius[i] = 0.0f;
				}
				else
				{
					var signed_dist = point[i] - bb.Centre[i];
					var length      = Math.Abs(signed_dist);
					if (length > bb.Radius[i])
					{
						float new_radius = (length + bb.Radius[i]) / 2.0f;
						bb.Centre[i] += signed_dist * (new_radius - bb.Radius[i]) / length;
						bb.Radius[i] = new_radius;
					}
				}
			}
			return bb;
		}

		/// <summary>Expands 'lhs' to include 'rhs'. Returns **rhs**</summary>
		public static v4 Grow(ref BBox bbox, v4 point)
		{
			// Const version returns lhs, non-const returns rhs!
			bbox = Union(bbox, point);
			return point;
		}

		/// <summary>Returns a bbox that encloses 'lhs' and 'rhs'</summary>
		public static BBox Union(BBox lhs, BBox rhs)
		{
			// Const version returns lhs, non-const returns rhs!
			// Don't treat !rhs.IsValid as an error, it's the only way to grow an empty bbox
			var bb = lhs;
			if (!rhs.IsValid) return bb;
			bb = Union(bb, rhs.Centre + rhs.Radius);
			bb = Union(bb, rhs.Centre - rhs.Radius);
			return bb;
		}

		/// <summary>Expands 'lhs' to include 'rhs'. Returns **rhs**</summary>
		public static BBox Grow(ref BBox lhs, BBox rhs)
		{
			// Const version returns lhs, non-const returns rhs!
			// Don't treat !rhs.IsValid as an error, it's the only way to grow an empty bbox
			if (!rhs.IsValid) return rhs;
			lhs = Union(lhs, rhs.Centre + rhs.Radius);
			lhs = Union(lhs, rhs.Centre - rhs.Radius);
			return rhs;
		}

		/// <summary>Create a RectangleF from the X,Y axes of this bounding box</summary>
		public RectangleF ToRectXY() => new(MinX, MinY, SizeX, SizeY);

		/// <summary></summary>
		public string Description => $"C={Centre} R={Radius}";
	}
}