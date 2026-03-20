using System;
using Rylogic.Extn;

namespace Rylogic.Maths
{
	public static partial class Math_
	{
		/// <summary>Add XML serialisation support for graphics types</summary>
		public static XmlConfig SupportRylogicMathsTypes(this XmlConfig cfg)
		{
			Xml_.ToMap[typeof(v2)] = (obj, node) =>
			{
				var vec = (v2)obj;
				node.SetValue(vec.ToString());
				return node;
			};
			Xml_.AsMap[typeof(v2)] = (elem, type, instance) =>
			{
				return v2.Parse(elem.Value);
			};

			Xml_.ToMap[typeof(v3)] = (obj, node) =>
			{
				var vec = (v3)obj;
				node.SetValue(vec.ToString());
				return node;
			};
			Xml_.AsMap[typeof(v3)] = (elem, type, instance) =>
			{
				return v3.Parse(elem.Value);
			};

			Xml_.ToMap[typeof(v4)] = (obj, node) =>
			{
				var vec = (v4)obj;
				node.SetValue(vec.ToString());
				return node;
			};
			Xml_.AsMap[typeof(v4)] = (elem, type, instance) =>
			{
				return v4.Parse(elem.Value);
			};

			Xml_.ToMap[typeof(m4x4)] = (obj, node) =>
			{
				var mat = (m4x4)obj;
				node.Add(
					mat.x.ToXml(nameof(m4x4.x), false),
					mat.y.ToXml(nameof(m4x4.y), false),
					mat.z.ToXml(nameof(m4x4.z), false),
					mat.w.ToXml(nameof(m4x4.w), false));
				return node;
			};
			Xml_.AsMap[typeof(m4x4)] = (elem, type, instance) =>
			{
				var x = elem.Element(nameof(m4x4.x)).As<v4>();
				var y = elem.Element(nameof(m4x4.y)).As<v4>();
				var z = elem.Element(nameof(m4x4.z)).As<v4>();
				var w = elem.Element(nameof(m4x4.w)).As<v4>();
				return new m4x4(x, y, z, w);
			};

			return cfg;
		}

		/// <summary>True if 'ty' is a vector type</summary>
		public static bool IsVectorType(Type ty)
		{
			return
				ty == typeof(v2) ||
				ty == typeof(v3) ||
				ty == typeof(v4) ||
				(ty.IsGenericType && ty.GetGenericTypeDefinition() == typeof(v8<>));
		}

		/// <summary>True if 'ty' is a matrix type</summary>
		public static bool IsMatrixType(Type ty)
		{
			return
				ty == typeof(m2x2) ||
				ty == typeof(m3x4) ||
				ty == typeof(m4x4) ||
				ty == typeof(Matrix) ||
				(ty.IsGenericType && ty.GetGenericTypeDefinition() == typeof(m6x8<,>));
		}

		/// <summary>True if 'ty' is one of 'm2x2', 'm3x4', 'm4x4'</summary>
		public static bool IsVecMatType(Type ty)
		{
			return IsVectorType(ty) || IsMatrixType(ty);
		}
	}
}