//***************************************************
// Enum Extensions
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Linq.Expressions;
using System.Reflection;
using System.Text;
using Rylogic.Attrib;
using Rylogic.Common;
using Rylogic.Utility;

namespace Rylogic.Extn
{
	public static class Enum_
	{
		/// <summary>Enumerate the set flags in this enum</summary>
		public static IEnumerable<T> EnumFlags<T>(this T e) where T : Enum
		{
			if (!typeof(T).IsEnum || !typeof(T).HasAttribute<FlagsAttribute>())
				throw new ArgumentException("This is only valid on 'Flags' enums");
			
			foreach (var mask in Bit.EnumBitMasks(Util.ConvertTo<ulong>(e)))
				yield return (T)Enum.ToObject(typeof(T), mask);
		}

		/// <summary>Convert the enum value to a string containing spaces</summary>
		public static string ToPrettyString(this Enum e)
		{
			return e.ToStringFast().Txfm(Str_.ECapitalise.UpperCase, Str_.ECapitalise.DontChange, Str_.ESeparate.Add, " ");
		}

		/// <summary>A faster overload of ToString</summary>
		public static string ToStringFast(this Enum e)
		{
			var ty = typeof(Enum<>).MakeGenericType(e.GetType());
			var str = (string?)ty.InvokeMember(nameof(e.ToString), BindingFlags.InvokeMethod, null, null, new object[] { e }) ?? string.Empty;
			return str;
		}

		/// <summary>Return the index of this enum value within the enum (for non-sequential enums)
		/// Use: (MyEnum)Enum.GetValues(typeof(MyEnum)).GetValue(i) for the reverse of this method</summary>
		public static int ToIndex(this Enum e)
		{
			int i = 0;
			foreach (var v in Enum.GetValues(e.GetType()))
			{
				if (v != null && e.GetHashCode() == v.GetHashCode()) return i;
				++i;
			}
			return -1;
		}

		/// <summary>Test for this enum value within the set of provided arguments</summary>
		public static bool Within(this Enum e, params Enum[] args)
		{
			return args.Contains(e);
		}
	}
	public static class Enum<T>
		where T : struct, IConvertible
	{
		static Enum()
		{
			var type = typeof(T);
			if (!type.IsEnum)
				throw new ArgumentException("Generic Enum type works only with enums");

			var names = Enum.GetNames(type);
			var values = (T[])Enum.GetValues(type);
			if (type.GetCustomAttributes(typeof(FlagsAttribute), false).Length != 0)
			{
				Converter = new FlagsEnumConverter(names, values);
			}
			else
			{
				Converter = values.Where((t, i) => Convert.ToInt32(t) != i).Any()
					? new DictionaryEnumConverter(names, values)
					: (EnumConverter)new ArrayEnumConverter(names);
			}
		}

		// Some of this came from:
		//  http://www.codeproject.com/KB/dotnet/enum.aspx
		//  Author: Boris Dongarov (ideafixxxer)
		private static readonly EnumConverter Converter;

		#region ToString Converters

		#region EnumConverter
		private abstract class EnumConverter
		{
			public abstract string ToStringInternal(long value);
			public abstract long   ParseInternal(string value, bool ignoreCase, bool parseNumber);
			public abstract bool   TryParseInternal(string value, bool ignore_case, bool parse_number, out long result);
		}
		#endregion

		#region ArrayEnumConverter
		class ArrayEnumConverter :EnumConverter
		{
			private readonly string[] m_names = Enum.GetNames(typeof(T));

			public ArrayEnumConverter(string[] names)
			{
				m_names = names;
			}
			public override string ToStringInternal(long value)
			{
				return value >= 0 && value < m_names.Length ? m_names[value] : value.ToString(CultureInfo.InvariantCulture);
			}
			public override long ParseInternal(string value, bool ignoreCase, bool parseNumber)
			{
				if (value == null) throw new ArgumentNullException("value");
				if (value.Length == 0) throw new ArgumentException("Value is empty", "value");

				var f = value[0];
				if (parseNumber && (Char.IsDigit(f) || f == '+' || f == '-'))
					return Int32.Parse(value);

				var string_comparison = ignoreCase ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal;
				for (var i = 0; i < m_names.Length; i++)
					if (m_names[i].Equals(value, string_comparison))
						return i;

				throw new ArgumentException("Enum value wasn't found", "value");
			}
			public override bool TryParseInternal(string value, bool ignore_case, bool parse_number, out long result)
			{
				result = 0;
				if (string.IsNullOrEmpty(value))
					return false;

				var f = value[0];
				if (parse_number && (Char.IsDigit(f) || f == '+' || f == '-'))
				{
					int i;
					if (Int32.TryParse(value, out i))
					{
						result = i;
						return true;
					}
					return false;
				}
				var string_comparison = ignore_case ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal;
				for (var i = 0; i < m_names.Length; i++)
				{
					if (!m_names[i].Equals(value,string_comparison)) continue;
					result = i;
					return true;
				}
				return false;
			}
		}
		#endregion

		#region DictionaryEnumCovnerter
		private class DictionaryEnumConverter : EnumConverter
		{
			protected readonly Dictionary<long, string> m_dic;

			public DictionaryEnumConverter(string[] names, T[] values)
			{
				m_dic = new Dictionary<long, string>(names.Length);
				
				// If the same value occurs multiple times in the enum, overwrite
				for (var j = 0; j < names.Length; j++)
					m_dic[Convert.ToInt64(values[j], null)] = names[j];
			}
			public override string ToStringInternal(long value)
			{
				return m_dic.TryGetValue(value, out var n) ? n : value.ToString(CultureInfo.InvariantCulture);
			}
			public override long ParseInternal(string value, bool ignoreCase, bool parseNumber)
			{
				if (value == null) throw new ArgumentNullException("value");
				if (value.Length == 0) throw new ArgumentException("Value is empty", "value");

				var f = value[0];
				if (parseNumber && (char.IsDigit(f) || f == '+' || f == '-'))
					return int.Parse(value);

				var string_comparison = ignoreCase ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal;
				foreach (var pair in m_dic)
				{
					if (pair.Value.Equals(value, string_comparison))
						return pair.Key;
				}
				throw new ArgumentException("Enum value wasn't found", "value");
			}
			public override bool TryParseInternal(string value, bool ignore_case, bool parse_number, out long result)
			{
				result = 0;
				if (string.IsNullOrEmpty(value))
					return false;

				var f = value[0];
				if (parse_number && (char.IsDigit(f) || f == '+' || f == '-'))
				{
					int i;
					if (int.TryParse(value, out i))
					{
						result = i;
						return true;
					}
					return false;
				}
				var string_comparison = ignore_case ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal;
				foreach (var pair in m_dic)
				{
					if (!pair.Value.Equals(value,string_comparison)) continue;
					result = pair.Key;
					return true;
				}
				return false;
			}
		}
		#endregion

		#region FlagsEnumConverter
		private class FlagsEnumConverter : DictionaryEnumConverter
		{
			private readonly long[] m_values;

			public FlagsEnumConverter(string[] names, T[] values) :base(names, values)
			{
				m_values = new long[values.Length];
				for (var i = 0; i < values.Length; i++)
					m_values[i] = values[i].ToInt64(null);
			}
			public override string ToStringInternal(long value)
			{
				if (m_dic.TryGetValue(value, out var n)) return n;
				var sb = new StringBuilder();
				const string sep = ", ";
				for (var i = m_values.Length - 1; i >= 0; i--)
				{
					var v = m_values[i];
					if (v == 0) continue;
					if ((v & value) != v) continue;
					value &= ~v;
					sb.Insert(0, sep).Insert(0, m_dic[(int)v]);
				}
				return value == 0 && sb.Length > sep.Length ? sb.ToString(0, sb.Length - sep.Length) : value.ToString(CultureInfo.InvariantCulture);
			}
			public override long ParseInternal(string value, bool ignoreCase, bool parseNumber)
			{
				var parts = value.Split(new[]{"|"}, StringSplitOptions.RemoveEmptyEntries);
				if (parts.Length == 1) return base.ParseInternal(value, ignoreCase, parseNumber);
				var val = 0L;
				foreach (var part in parts)
				{
					var t = base.ParseInternal(part.Trim(), ignoreCase, parseNumber);
					val |= t;
				}
				return val;
			}
			public override bool TryParseInternal(string value, bool ignore_case, bool parse_number, out long result)
			{
				var parts = value.Split(new[]{"|"}, StringSplitOptions.RemoveEmptyEntries);
				if (parts.Length == 1) return base.TryParseInternal(value, ignore_case, parse_number, out result);
				var val = 0L;
				foreach (var part in parts)
				{
					long t;
					if (!base.TryParseInternal(part.Trim(), ignore_case, parse_number, out t))
					{
						result = 0;
						return false;
					}
					val |= t;
				}
				result = val;
				return true;
			}
		}
		#endregion

		#endregion

		/// <summary>Converts enum value to string</summary>
		/// <returns>If <paramref name="value"/> is defined, the enum member name; otherwise the string representation of the <paramref name="value"/>.
		/// If <see cref="FlagsAttribute"/> is applied, can return a delimited list of values</returns>
		public static string ToString(long value)
		{
			return Converter.ToStringInternal(value);
		}

		/// <summary>Converts enum value to string</summary>
		/// <returns>If <paramref name="value"/> is defined, the enum member name; otherwise the string representation of the <paramref name="value"/>.
		/// If <see cref="FlagsAttribute"/> is applied, can return comma-separated list of values</returns>
		public static string ToString(T value)
		{
			return Converter.ToStringInternal(value.ToInt64(null));
		}

		/// <summary>Convert from a long value to an enum value</summary>
		public static T ToValue(long value)
		{
			return (T)Enum.ToObject(typeof(T), value);
		}

		/// <summary>Convert a string to an enum values, throws on failure</summary>
		public static T Parse(string value, bool ignore_case = false, bool parse_numeric = true)
		{
			return (T) Enum.ToObject(typeof(T), Converter.ParseInternal(value, ignore_case, parse_numeric));
		}

		/// <summary>Convert a string into an enum value</summary>
		public static T? TryParse(string value, bool ignoreCase, bool parseNumeric)
		{
			return Converter.TryParseInternal(value, ignoreCase, parseNumeric, out var ir)
				? (T?)Enum.ToObject(typeof(T), ir) : null;
		}
		public static T? TryParse(string value, bool ignoreCase)
		{
			return Converter.TryParseInternal(value, ignoreCase, true, out var ir)
				? (T?)Enum.ToObject(typeof(T), ir) : null;
		}
		public static T? TryParse(string value)
		{
			return Converter.TryParseInternal(value, false, true, out var ir)
				? (T?)Enum.ToObject(typeof(T), ir) : null;
		}

		/// <summary>Returns an indication whether a constant with a specified value exists in a specified enumeration.</summary>
		public static bool IsDefined(object value)
		{
			return Enum.IsDefined(typeof(T), value);
		}

		/// <summary>Returns the next enum value after 'value'. Note: this is really enum abuse. Use sparingly</summary>
		public static T Cycle(T src)
		{
			var arr = (T[])Enum.GetValues(typeof(T));
			int i = Array.IndexOf(arr, src) + 1;
			return (i >= 0 && i < arr.Length) ? arr[i] : arr[0];
		}

		/// <summary>
		/// Returns all enum field names as a collection.
		/// Note: Names are returned in value order, not declaration order</summary>
		public static IEnumerable<string> Names
		{
			get { return Enum.GetNames(typeof(T)); }
		}

		/// <summary>
		/// Returns all values of the enum as a collection.
		/// Note: duplicate values are returned, but the member name will be the first name with that value.
		/// Note: Values are returned in value order, not declaration order</summary>
		public static IEnumerable<T> Values
		{
			get { return Enum.GetValues(typeof(T)).Cast<T>(); }
		}

		/// <summary>Returns all values of the enum as an object[]</summary>
		public static object[] ValuesArray
		{
			get { return Values.Cast<object>().ToArray(); }
		}

		/// <summary>
		/// Returns all Name,Value pairs of the enum as a collection.
		/// 'Items.ToList()' can be assigned to the DataSource of binding controls.
		/// Items are returned in declaration order</summary>
		public static IEnumerable<Item> Items
		{
			get
			{
				var fields = typeof(T).GetFields(BindingFlags.Public|BindingFlags.Static);
				return fields.Select(x => new Item(x));
			}
		}
		public class Item
		{
			private readonly FieldInfo m_fi;
			internal Item(FieldInfo fi)
			{
				m_fi = fi;
				Name = fi.Name;
				Value = (T)fi.GetValue(null)!;
			}

			/// <summary>The name of the enum member</summary>
			public string Name { get; }

			/// <summary>The value of the enum member</summary>
			public T Value { get; }

			/// <summary>The associated description string of the enum member</summary>
			public string? Desc => DescAttr.Desc(typeof(T), m_fi);

			/// <summary>The associated AssocAttribute of the enum member</summary>
			public TAssoc Assoc<TAssoc>(string? name = null)
			{
				return Assoc_.Assoc<TAssoc>(m_fi, name);
			}

			/// <summary>Try to get the associated value of the enum member</summary>
			public bool TryAssoc<TAssoc>(out TAssoc assoc, string? name = null)
			{
				return Assoc_.TryAssoc(m_fi, out assoc, name);
			}

			public override string ToString() => Desc ?? Name;
			public static implicit operator T(Item item) { return item.Value; }
		}

		/// <summary>Returns a list of strings DescAttributes attributed to the enum type</summary>
		public static IEnumerable<string> Desc
		{
			get { return typeof(T).AllDesc(); }
		}

		/// <summary>Returns the objects of type 'TAssoc' associated with each enum member using the AssocAttribute</summary>
		public static IEnumerable<TAssoc> Assoc<TAssoc>(string? name = null)
		{
			return Items.Select(x => x.Assoc<TAssoc>(name));
		}

		/// <summary>Return the enum value with the given associated value. Null if there is no matching enum value</summary>
		public static T? FromAssoc<TAssoc>(TAssoc assoc, string? name = null)
			where TAssoc : notnull
		{
			if (m_assoc_reverse_lookup == null)
			{
				var map = new Dictionary<TAssoc, T>();
				foreach (var item in Items)
				{
					if (item.TryAssoc<TAssoc>(out var a, name))
						map.Add(a, item.Value);
				}
				m_assoc_reverse_lookup = map;
			}

			// Lookup the enum member from the associated value
			var lookup = (Dictionary<TAssoc, T>)m_assoc_reverse_lookup;
			return lookup.TryGetValue(assoc, out var value) ? (T?)value : null;
		}
		private static object? m_assoc_reverse_lookup;

		/// <summary>Returns the number of members in the enumeration</summary>
		public static int Count
		{
			get { return typeof(T).GetFields(BindingFlags.Public|BindingFlags.Static).Count(); }
		}

		/// <summary>Return the result of OR'ing all enum values together</summary>
		public static T All
		{
			get
			{
				// Throws if 'T' does not have the Flags attribute
				if (typeof(T).FindAttribute<FlagsAttribute>() == null)
					return default(T);

				var all = default(T);
				foreach (var i in Values)
					all = BitwiseOr(all, i);

				return (T)Convert.ChangeType(all, typeof(T));
			}
		}

		/// <summary>Bitwise OR two enum values together</summary>
		private static T BitwiseOr(T a, T b)
		{
			if (m_impl_or == null)
			{
				// Declare the parameters
				var paramA = Expression.Parameter(typeof(T), "a");
				var paramB = Expression.Parameter(typeof(T), "b");

				// OR the parameters together
				var body = Expression.Convert(
					Expression.Or(
						Expression.Convert(paramA, typeof(long)),
						Expression.Convert(paramB, typeof(long)))
						,typeof(T));

				// Compile it
				m_impl_or = Expression.Lambda<Func<T,T,T>>(body, paramA, paramB).Compile();
			}
			return m_impl_or(a,b);
		}
		private static Func<T,T,T>? m_impl_or;
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Extn;

	[TestFixture] public class TestEnumExtns
	{
		public enum ArrayEnum
		{
			One,
			Two,
			Three,
		}
		public enum SparceEnum
		{
			One = 1,
			Four = 4,
			Ten = 10,
		}
		public enum EnumWithDuplicates
		{
			One  = 1,
			Won  = 1,
			Juan = 1,
			Two  = 2,
			To   = 2,
			Too  = 2,
		}
		[Flags] public enum FlagsEnum
		{
			A = 1 << 0,
			B = 1 << 1,
			C = 1 << 2
		}
		public enum Tags
		{
			SomeCompoundName,
			ABBRValue01
		}
		[Flags] public enum BigFlags :uint
		{
			A = 0x80000000,
			B = 0xC0000000,
			C = 0xE0000000,
		}

		[Test] public void TestToString1()
		{
			Assert.Equal(ArrayEnum.One   .ToString(), ArrayEnum.One   .ToStringFast());
			Assert.Equal(ArrayEnum.Two   .ToString(), ArrayEnum.Two   .ToStringFast());
			Assert.Equal(ArrayEnum.Three .ToString(), ArrayEnum.Three .ToStringFast());
		}
		[Test] public void TestToString2()
		{
			Assert.Equal(SparceEnum.One  .ToString(), SparceEnum.One  .ToStringFast());
			Assert.Equal(SparceEnum.Four .ToString(), SparceEnum.Four .ToStringFast());
			Assert.Equal(SparceEnum.Ten  .ToString(), SparceEnum.Ten  .ToStringFast());
		}
		[Test] public void TestToString3()
		{
			var f = FlagsEnum.A;
			Assert.Equal(f.ToString(), f.ToStringFast());
			f = FlagsEnum.A|FlagsEnum.B;
			Assert.Equal(f.ToString(), f.ToStringFast());
			f = FlagsEnum.A|FlagsEnum.C;
			Assert.Equal(f.ToString(), f.ToStringFast());
			f = FlagsEnum.A|FlagsEnum.B|FlagsEnum.C;
			Assert.Equal(f.ToString(), f.ToStringFast());
		}
		[Test] public void TestToString4()
		{
			Assert.Equal("Some Compound Name", Tags.SomeCompoundName.ToPrettyString());
			Assert.Equal("ABBR Value 01", Tags.ABBRValue01.ToPrettyString());
		}
		[Test] public void TestToString5()
		{
			Assert.Equal(BigFlags.A.ToString(), BigFlags.A.ToStringFast());
			Assert.Equal(BigFlags.B.ToString(), BigFlags.B.ToStringFast());
			Assert.Equal(BigFlags.C.ToString(), BigFlags.C.ToStringFast());
		}
		[Test] public void TestEnumAll()
		{
			var all = Enum<FlagsEnum>.All;
			Assert.Equal(FlagsEnum.A|FlagsEnum.B|FlagsEnum.C, all);
		}
		[Test] public void TestItems()
		{
			var items = Enum<SparceEnum>.Items.ToArray();
			Assert.True(items[0] == SparceEnum.One);
			Assert.True(items[1] == SparceEnum.Four);
			Assert.True(items[2] == SparceEnum.Ten);

			var items2 = Enum<EnumWithDuplicates>.Items.ToArray();
			Assert.True(items2[0] == EnumWithDuplicates.One );
			Assert.True(items2[1] == EnumWithDuplicates.Won );
			Assert.True(items2[2] == EnumWithDuplicates.Juan);
			Assert.True(items2[3] == EnumWithDuplicates.Two );
			Assert.True(items2[4] == EnumWithDuplicates.To  );
			Assert.True(items2[5] == EnumWithDuplicates.Too );
		}
	}
}
#endif
