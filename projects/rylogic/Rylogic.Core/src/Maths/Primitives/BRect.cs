//***************************************************
// Bounding Rectangle
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;
using System.Drawing;

namespace Rylogic.Maths
{
	/// <summary>Bounding box</summary>
	[Serializable]
	public struct BRect
	{
		private v2 m_centre;
		private v2 m_radius;

		// Constructors
		public BRect(v2 centre, v2 radius)       { m_centre = centre; m_radius = radius; }
		public void  set(v2 centre, v2 radius)   { m_centre = centre; m_radius = radius; }
		public override string ToString()        { return "Centre=" + m_centre.ToString() + " Radius=" + m_radius.ToString(); }

		// Static types
		private readonly static BRect m_zero  = new(v2.Zero, v2.Zero);
		private readonly static BRect m_unit  = new(v2.Zero, new v2(0.5f, 0.5f));
		private readonly static BRect m_reset = new(v2.Zero, new v2(-1, -1));
		public static BRect Zero  { get { return m_zero; } }
		public static BRect Unit  { get { return m_unit; } }
		public static BRect Reset { get { return m_reset; } }

		// Binary operators
		public static BRect operator + (BRect lhs, v2 offset) { lhs.m_centre += offset; return lhs; }
		public static BRect operator - (BRect lhs, v2 offset) { lhs.m_centre -= offset; return lhs; }
		public static BRect operator * (BRect lhs, float s)   { lhs.m_radius *= s; return lhs; }
		public static BRect operator / (BRect lhs, float s)   { lhs.m_radius /= s; return lhs; }
		//public static BRect operator * (m3x4 m, BRect rhs)
		//{
		//	Debug.Assert(rhs.IsValid, "Transforming an invalid bounding rectangle");
		//	BRect bb = new BRect(m.p, v2.Zero);
		//	m2x2 mat = m2x2.Transpose2x2(m);
		//	for (int i = 0; i != 2; ++i)
		//	{
		//		bb.m_centre[i] += v2.Dot2(       mat[i] , rhs.m_centre);
		//		bb.m_radius[i] += v2.Dot2(v2.Abs(mat[i]), rhs.m_radius);
		//	}
		//	return bb;
		//}

		// Equality operators
		public static bool operator ==(BRect lhs, BRect rhs)
		{
			return lhs.m_centre == rhs.m_centre;
		}
		public static bool operator !=(BRect lhs, BRect rhs)
		{
			return !(lhs == rhs);
		}
		public override bool Equals(object? o)
		{
			return o is BRect && (BRect)o == this;
		}
		public override int GetHashCode()
		{
			return new { m_centre, m_radius }.GetHashCode();
		}

		// Conversion
		public static implicit operator BRect(RectangleF r)
		{
			return new BRect(v2.From(r.Location) + v2.From(r.Size)/2f, Math_.Abs(v2.From(r.Size)) / 2f);
		}
		public static implicit operator RectangleF(BRect r)
		{
			return new RectangleF(r.m_centre.x - r.m_radius.x, r.m_centre.y - r.m_radius.y, 2*r.m_radius.x, 2*r.m_radius.y);
		}
		public Rectangle ToRectangle()
		{
			return new Rectangle(
				(int)Math.Round(m_centre.x - m_radius.x),
				(int)Math.Round(m_centre.y - m_radius.y),
				(int)Math.Round(2*m_radius.x),
				(int)Math.Round(2*m_radius.y));
		}

		public static BRect From(Rectangle r)
		{
			return new BRect(v2.From(r.Location) + v2.From(r.Size)/2f, Math_.Abs(v2.From(r.Size)) / 2f);
		}
		
		/// <summary>Return a bounding rect about the given points</summary>
		public static BRect FromBounds(params v2[] points)
		{
			var br = BRect.Reset;
			foreach (var pt in points)
				br.Grow(pt);
			return br;
		}

		/// <summary>Returns true if the bounding rectangle represents a point or volume</summary>
		public bool IsValid
		{
			get { return m_radius.x >= 0f && m_radius.y >= 0f; }
		}

		/// <summary>Get/Set the lower corner of the bounding rectangle</summary>
		public v2 Lower
		{
			get { return m_centre - m_radius; }
			set
			{
				if (!IsValid)
				{
					m_centre = value;
					m_radius = v2.Zero;
				}
				else
				{
					var upper = Upper;
					var lower = value;
					m_centre = (lower + upper) / 2f;
					m_radius = upper - m_centre;
				}
			}
		}

		/// <summary>Get/Set the upper corner of the bounding rectangle</summary>
		public v2 Upper
		{
			get { return m_centre + m_radius; }
			set
			{
				if (!IsValid)
				{
					m_centre = value;
					m_radius = v2.Zero;
				}
				else
				{
					var upper = value;
					var lower = Lower;
					m_centre = (lower + upper) / 2f;
					m_radius = upper - m_centre;
				}
			}
		}

		/// <summary>Get/Set the x dimension of the bounding rectangle</summary>
		public float SizeX
		{
			get { return 2.0f * m_radius.x; }
			set { m_radius.x = value * 0.5f; }
		}

		/// <summary>Gets the y dimension of the bounding rectangle</summary>
		public float SizeY
		{
			get { return 2.0f * m_radius.y; }
			set { m_radius.y = value * 0.5f; }
		}

		/// <summary>Get/Set the size of the rect</summary>
		public v2 Size
		{
			get { return new v2(SizeX, SizeY); }
			set { SizeX = value.x; SizeY = value.y; }
		}

		/// <summary>Get/Sets the centre point of the bounding rectangle</summary>
		public v2 Centre
		{
			get { return m_centre; }
			set { m_centre = value; }
		}

		/// <summary>Get/Sets the dimensions of the bounding rectangle</summary>
		public v2 Radius
		{
			get { return m_radius; }
			set { m_radius = value; }
		}

		/// <summary>Gets the squared length of the diagonal of the bounding rectangle</summary>
		public float DiametreSq
		{
			get { return 4.0f * m_radius.LengthSq; }
		}

		/// <summary>Gets the length of the diagonal of the bounding rectangle</summary>
		public float Diametre
		{
			get { return Math_.Sqrt(DiametreSq); }
		}

		/// <summary>Gets the area of the bounding rect</summary>
		public float Area
		{
			get { return SizeX * SizeY; }
		}

		/// <summary>Returns a corner of the bounding rectangle. 'corner'</summary>
		public v2 GetCorner(uint corner)
		{
			Debug.Assert(corner < 4, "Invalid corner index");
			uint x = ((corner >> 0) & 0x1) * 2 - 1;
			uint y = ((corner >> 1) & 0x1) * 2 - 1;
			return new v2(m_centre.x + x*m_radius.x, m_centre.y + y*m_radius.y);
		}

		/// <summary>Expands the bounding rectangle to include 'point'</summary>
		public void Grow(params v2[] points)
		{
			foreach (var point in points)
			{
				for (int i = 0; i != 2; ++i)
				{
					if (m_radius[i] < 0.0f)
					{
						m_centre[i] = point[i];
						m_radius[i] = 0.0f;
					}
					else
					{
						float signed_dist = point[i] - m_centre[i];
						float length      = Math.Abs(signed_dist);
						if (length > m_radius[i])
						{
							float new_radius = (length + m_radius[i]) / 2.0f;
							m_centre[i] += signed_dist * (new_radius - m_radius[i]) / length;
							m_radius[i] = new_radius;
						}
					}
				}
			}
		}

		/// <summary>Expands the bounding rectangle to include 'rhs'</summary>
		public void Grow(params BRect[] areas)
		{
			foreach (var area in areas)
			{
				Debug.Assert(area.IsValid, "Encompasing an invalid bounding rectangle");
				Grow(area.m_centre + area.m_radius);
				Grow(area.m_centre - area.m_radius);
			}
		}

		/// <summary>Returns true if 'point' is within this bounding rectangle (within 'tol'erance)</summary>
		public bool IsWithin(v2 point, float tol = 0f)
		{
			return
				Math.Abs(point.x - m_centre.x) <= m_radius.x + tol &&
				Math.Abs(point.y - m_centre.y) <= m_radius.y + tol;
		}

		/// <summary>Returns true if 'brect' is within this bounding rectangle (within 'tol'erance)</summary>
		public bool IsWithin(BRect bbox, float tol = 0f)
		{
			return
				Math.Abs(bbox.m_centre.x - m_centre.x) <= (m_radius.x - bbox.m_radius.x + tol) &&
				Math.Abs(bbox.m_centre.y - m_centre.y) <= (m_radius.y - bbox.m_radius.y + tol);
		}

		/// <summary>Returns true if 'rhs' intersects with this bounding rectangle</summary>
		public static bool IsIntersection(BRect lhs, BRect rhs)
		{
			return
				Math.Abs(lhs.m_centre.x - rhs.m_centre.x) <= (lhs.m_radius.x + rhs.m_radius.x) &&
				Math.Abs(lhs.m_centre.y - rhs.m_centre.y) <= (lhs.m_radius.y + rhs.m_radius.y);
		}

		/// <summary>Returns the bounding rectangle shifted by [dx,dy]</summary>
		public BRect Shifted(v2 offset)
		{
			return new BRect(m_centre + offset, m_radius);
		}
		public BRect Shifted(float dx, float dy)
		{
			return Shifted(new v2(dx, dy));
		}

		/// <summary>Increase the size of the bounding rect by 'delta'. i.e. grows by half 'delta' in each direction</summary>
		public BRect Inflate(v2 delta)
		{
			return new BRect(m_centre, m_radius + delta/2f);
		}
		public BRect Inflate(float dx, float dy)
		{
			return Inflate(new v2(dx, dy));
		}
	}
}