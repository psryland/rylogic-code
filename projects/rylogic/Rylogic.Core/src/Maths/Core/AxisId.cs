using System;
using Rylogic.Attrib;

namespace Rylogic.Maths
{
	public enum EAxisId
	{
		None = 0,
		[Desc("+X Axis")] PosX = +1,
		[Desc("-X Axis")] NegX = -1,
		[Desc("+Y Axis")] PosY = +2,
		[Desc("-Y Axis")] NegY = -2,
		[Desc("+Z Axis")] PosZ = +3,
		[Desc("-Z Axis")] NegZ = -3,
	}

	public struct AxisId
	{
		public AxisId(EAxisId id)
		{
			m_id = 0;
			Id = id;
		}

		/// <summary></summary>
		public EAxisId Id
		{
			get => m_id;
			set => m_id = Math.Abs((int)value) <= 3 ? value : throw new Exception("Invalid axis id. Must one of 0, ±1, ±2, ±3 corresponding to None, ±X, ±Y, ±Z respectively");
		}
		private EAxisId m_id;

		/// <summary>Get/Set the axis associated with this id</summary>
		public v4 Axis
		{
			get
			{
				switch (Id)
				{
				case EAxisId.PosX: return v4.XAxis;
				case EAxisId.NegX: return v4.XAxis;
				case EAxisId.PosY: return v4.YAxis;
				case EAxisId.NegY: return v4.YAxis;
				case EAxisId.PosZ: return v4.ZAxis;
				case EAxisId.NegZ: return v4.ZAxis;
				default: throw new Exception($"{Id} is not a valid axis id");
				}
			}
			set
			{
				if (value == +v4.XAxis) { Id = EAxisId.PosX; return; }
				if (value == -v4.XAxis) { Id = EAxisId.NegX; return; }
				if (value == +v4.YAxis) { Id = EAxisId.PosY; return; }
				if (value == -v4.YAxis) { Id = EAxisId.NegY; return; }
				if (value == +v4.ZAxis) { Id = EAxisId.PosZ; return; }
				if (value == -v4.ZAxis) { Id = EAxisId.NegZ; return; }
				throw new Exception($"{value} is not a named axis");
			}
		}

		/// <summary>Try to convert 'id' to an axis id</summary>
		public static bool Try(int id, out AxisId axis)
		{
			axis = default;
			if (Math.Abs(id) < 1 || Math.Abs(id) > 3)
				return false;

			axis = id;
			return true;
		}

		/// <summary></summary>
		public static implicit operator AxisId(EAxisId id) { return new AxisId(id); }
		public static implicit operator AxisId(int id) { return new AxisId((EAxisId)id); }
		public static implicit operator EAxisId(AxisId id) { return id.Id; }
		public static implicit operator int(AxisId id) { return (int)id.Id; }
		public static implicit operator v4(AxisId id) { return id.Axis; }

		/// <summary></summary>
		public override string ToString() => ((int)Id).ToString();
	}
}
