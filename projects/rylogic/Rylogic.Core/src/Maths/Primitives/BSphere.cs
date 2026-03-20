//***************************************************
// Bounding Sphere
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Diagnostics;

namespace Rylogic.Maths
{
	/// <summary>Bounding sphere</summary>
	[Serializable]
	[DebuggerDisplay("{Description,nq}")]
	public struct BSphere
	{
		private v4 m_ctr_rad;

		public BSphere(v4 centre, float radius)
		{
			m_ctr_rad = new v4(centre.xyz, radius);
		}
		
		/// <summary>Get/Sets the centre point of the bounding box</summary>
		public v4 Centre
		{
			get => m_ctr_rad.w1;
			set => m_ctr_rad.xyz = value.xyz;
		}

		/// <summary>Get/Sets the dimensions of the bounding box</summary>
		public float Radius
		{
			get => m_ctr_rad.w;
			set => m_ctr_rad.w = value;
		}

		/// <summary>Constants</summary>
		public static BSphere Unit { get; } = new BSphere(v4.Origin, 0.5f);
		public static BSphere Reset { get; } = new BSphere(v4.Origin, -1.0f);
		
		// Binary operators
		public static BSphere operator +(BSphere lhs, v4 offset) { lhs.Centre += offset; return lhs; }
		public static BSphere operator -(BSphere lhs, v4 offset) { lhs.Centre -= offset; return lhs; }
		public static BSphere operator *(BSphere lhs, float s) { lhs.Radius *= s; return lhs; }
		public static BSphere operator /(BSphere lhs, float s) { lhs.Radius /= s; return lhs; }
		public static BSphere operator *(m4x4 m, BSphere rhs)
		{
			Debug.Assert(rhs.IsValid, "Transforming an invalid bounding sphere");
			var bs = new BSphere(m.pos, 0f);
			var mat = Math_.Transpose3x3(m);
			
			bs.Centre = m * bs.Centre;
			bs.Radius = (m * new v4(0f, 0f, bs.Radius, 0f)).z; // Scale
			return bs;
		}

		// Equality operators
		public static bool operator ==(BSphere lhs, BSphere rhs) { return lhs.Centre == rhs.Centre; }
		public static bool operator !=(BSphere lhs, BSphere rhs) { return !(lhs == rhs); }
		public override bool Equals(object? o) { return o is BSphere sphere && sphere == this; }
		public override int GetHashCode() => new { Centre, Radius }.GetHashCode();

		/// <summary>Returns true if the bounding box represents a point or volume</summary>
		public bool IsValid => Radius >= 0f && Math_.IsFinite(Radius) && Math_.IsFinite(Centre.LengthSq);

		/// <summary></summary>
		public string Description => $"C={Centre} R={Radius}";
	}
}
