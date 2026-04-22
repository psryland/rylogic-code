namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System;
using System.Collections.Generic;
using EnvDTE;

/// <summary>
/// Reads integer scalar types (signed/unsigned, 8/16/32/64 bit) and char.
/// Returns the value boxed in its natural CLR type so consumers can pattern-match
/// or use the dynamic dispatcher freely.
/// </summary>
public class ScalarReader : ITypeReader
{
	// Map of type name → (size in bytes, native parser, managed parser).
	// Both parsers return a boxed value of the appropriate CLR type.
	private static readonly Dictionary<string, ScalarSpec> NativeSpecs = new()
	{
		["char"]               = new ScalarSpec(1, b => (sbyte)b[0]),
		["signed char"]        = new ScalarSpec(1, b => (sbyte)b[0]),
		["unsigned char"]      = new ScalarSpec(1, b => b[0]),
		["short"]              = new ScalarSpec(2, b => BitConverter.ToInt16(b, 0)),
		["short int"]          = new ScalarSpec(2, b => BitConverter.ToInt16(b, 0)),
		["unsigned short"]     = new ScalarSpec(2, b => BitConverter.ToUInt16(b, 0)),
		["unsigned short int"] = new ScalarSpec(2, b => BitConverter.ToUInt16(b, 0)),
		["int"]                = new ScalarSpec(4, b => BitConverter.ToInt32(b, 0)),
		["unsigned int"]       = new ScalarSpec(4, b => BitConverter.ToUInt32(b, 0)),
		["long"]               = new ScalarSpec(4, b => BitConverter.ToInt32(b, 0)),
		["unsigned long"]      = new ScalarSpec(4, b => BitConverter.ToUInt32(b, 0)),
		["long long"]          = new ScalarSpec(8, b => BitConverter.ToInt64(b, 0)),
		["unsigned long long"] = new ScalarSpec(8, b => BitConverter.ToUInt64(b, 0)),
		["__int8"]             = new ScalarSpec(1, b => (sbyte)b[0]),
		["__int16"]            = new ScalarSpec(2, b => BitConverter.ToInt16(b, 0)),
		["__int32"]            = new ScalarSpec(4, b => BitConverter.ToInt32(b, 0)),
		["__int64"]            = new ScalarSpec(8, b => BitConverter.ToInt64(b, 0)),
		["int8_t"]             = new ScalarSpec(1, b => (sbyte)b[0]),
		["int16_t"]            = new ScalarSpec(2, b => BitConverter.ToInt16(b, 0)),
		["int32_t"]            = new ScalarSpec(4, b => BitConverter.ToInt32(b, 0)),
		["int64_t"]            = new ScalarSpec(8, b => BitConverter.ToInt64(b, 0)),
		["uint8_t"]            = new ScalarSpec(1, b => b[0]),
		["uint16_t"]           = new ScalarSpec(2, b => BitConverter.ToUInt16(b, 0)),
		["uint32_t"]           = new ScalarSpec(4, b => BitConverter.ToUInt32(b, 0)),
		["uint64_t"]           = new ScalarSpec(8, b => BitConverter.ToUInt64(b, 0)),
		["wchar_t"]            = new ScalarSpec(2, b => (char)BitConverter.ToUInt16(b, 0)),
	};

	// Managed integer/char types. Both C#-keyword form ("int") and CLR form ("System.Int32"/"Int32") are accepted.
	// Managed reads parse the GetExpression text rather than RPM, so size isn't used.
	private static readonly Dictionary<string, Func<Debugger, string, object?>> ManagedReaders = new()
	{
		["sbyte"]          = (d, e) => ToSByte(ManagedFieldReader.ReadInt32(d, e)),
		["byte"]           = (d, e) => ToByte(ManagedFieldReader.ReadInt32(d, e)),
		["short"]          = (d, e) => ToInt16(ManagedFieldReader.ReadInt32(d, e)),
		["ushort"]         = (d, e) => ToUInt16(ManagedFieldReader.ReadInt32(d, e)),
		["int"]            = (d, e) => ManagedFieldReader.ReadInt32(d, e),
		["uint"]           = (d, e) => ToUInt32(ManagedFieldReader.ReadInt64(d, e)),
		["long"]           = (d, e) => ManagedFieldReader.ReadInt64(d, e),
		["ulong"]          = (d, e) => ToUInt64(ManagedFieldReader.ReadInt64(d, e)),
		["char"]           = (d, e) => ReadChar(d, e),

		// CLR / VB names
		["System.SByte"]   = (d, e) => ToSByte(ManagedFieldReader.ReadInt32(d, e)),
		["SByte"]          = (d, e) => ToSByte(ManagedFieldReader.ReadInt32(d, e)),
		["System.Byte"]    = (d, e) => ToByte(ManagedFieldReader.ReadInt32(d, e)),
		["Byte"]           = (d, e) => ToByte(ManagedFieldReader.ReadInt32(d, e)),
		["System.Int16"]   = (d, e) => ToInt16(ManagedFieldReader.ReadInt32(d, e)),
		["Int16"]          = (d, e) => ToInt16(ManagedFieldReader.ReadInt32(d, e)),
		["System.UInt16"]  = (d, e) => ToUInt16(ManagedFieldReader.ReadInt32(d, e)),
		["UInt16"]         = (d, e) => ToUInt16(ManagedFieldReader.ReadInt32(d, e)),
		["System.Int32"]   = (d, e) => ManagedFieldReader.ReadInt32(d, e),
		["Int32"]          = (d, e) => ManagedFieldReader.ReadInt32(d, e),
		["System.UInt32"]  = (d, e) => ToUInt32(ManagedFieldReader.ReadInt64(d, e)),
		["UInt32"]         = (d, e) => ToUInt32(ManagedFieldReader.ReadInt64(d, e)),
		["System.Int64"]   = (d, e) => ManagedFieldReader.ReadInt64(d, e),
		["Int64"]          = (d, e) => ManagedFieldReader.ReadInt64(d, e),
		["System.UInt64"]  = (d, e) => ToUInt64(ManagedFieldReader.ReadInt64(d, e)),
		["UInt64"]         = (d, e) => ToUInt64(ManagedFieldReader.ReadInt64(d, e)),
		["System.Char"]    = (d, e) => ReadChar(d, e),
		["Char"]           = (d, e) => ReadChar(d, e),
	};

	/// <inheritdoc/>
	public bool CanRead(ELanguage lang, string type_name) => lang switch
	{
		ELanguage.Native => NativeSpecs.ContainsKey(type_name),
		ELanguage.Managed => ManagedReaders.ContainsKey(type_name),
		ELanguage.Unknown => false,
		_ => throw new ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language"),
	};

	/// <inheritdoc/>
	public object? Read(Debugger debugger, ELanguage lang, string type_name, string expr)
	{
		switch (lang)
		{
			case ELanguage.Native:
			{
				return NativeSpecs.TryGetValue(type_name, out var spec) ? ReadNative(debugger, expr, spec) : null;
			}
			case ELanguage.Managed:
			{
				return ManagedReaders.TryGetValue(type_name, out var fn) ? fn(debugger, expr) : null;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}

	private static object? ReadNative(Debugger debugger, string expr, ScalarSpec spec)
	{
		var data = DebugMemoryReader.ReadBytes(debugger, expr, spec.Size);
		return data != null ? spec.Parse(data) : null;
	}

	// --- Char read (managed strings render chars as 'a' or numeric code) ---

	private static object? ReadChar(Debugger debugger, string expr)
	{
		var raw = ManagedFieldReader.ReadRaw(debugger, expr);
		if (string.IsNullOrEmpty(raw))
			return null;

		var s = raw!.Trim();

		// Common forms: 'a', 'a' 97, 97 'a', or just 97.
		// Find the quoted character if present.
		var i0 = s.IndexOf('\'');
		var i1 = i0 >= 0 ? s.IndexOf('\'', i0 + 1) : -1;
		if (i0 >= 0 && i1 > i0 + 1)
			return s[i0 + 1];

		// Otherwise fall back to numeric parse.
		if (int.TryParse(s, System.Globalization.NumberStyles.Integer, System.Globalization.CultureInfo.InvariantCulture, out var n))
			return (char)n;

		return null;
	}

	// --- Range-narrowing helpers -------------------------------------------------

	private static object? ToSByte(int? v) => v.HasValue ? (sbyte)v.Value : null;
	private static object? ToByte(int? v) => v.HasValue ? (byte)v.Value : null;
	private static object? ToInt16(int? v) => v.HasValue ? (short)v.Value : null;
	private static object? ToUInt16(int? v) => v.HasValue ? (ushort)v.Value : null;
	private static object? ToUInt32(long? v) => v.HasValue ? (uint)v.Value : null;
	private static object? ToUInt64(long? v) => v.HasValue ? unchecked((ulong)v.Value) : null;

	private readonly struct ScalarSpec
	{
		public readonly int Size;
		public readonly Func<byte[], object> Parse;
		public ScalarSpec(int size, Func<byte[], object> parse) { Size = size; Parse = parse; }
	}
}
