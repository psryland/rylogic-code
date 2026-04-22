namespace Rylogic.LDrawVisualiser.Core;

using System;
using System.Globalization;
using EnvDTE;
using Microsoft.VisualStudio.Shell;

/// <summary>
/// Reads primitive values from a managed (.NET) debuggee using DTE.Debugger.GetExpression().
/// This is the correct strategy for managed code: ReadProcessMemory is unsafe against the
/// GC heap (objects can move, headers/syncblocks are present, layouts may not be blittable
/// at the offsets seen by Marshal). Per-field GetExpression is slower but always correct.
/// </summary>
public static class ManagedFieldReader
{
	/// <summary>Read a single-precision float from 'expr'</summary>
	public static float? ReadSingle(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var value = ReadString(debugger, expr);
		if (value == null) return null;
		return float.TryParse(Clean(value), NumberStyles.Float, CultureInfo.InvariantCulture, out var f) ? f : null;
	}

	/// <summary>Read a double-precision float from 'expr'</summary>
	public static double? ReadDouble(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var value = ReadString(debugger, expr);
		if (value == null) return null;
		return double.TryParse(Clean(value), NumberStyles.Float, CultureInfo.InvariantCulture, out var d) ? d : null;
	}

	/// <summary>Read a 32-bit signed integer from 'expr'</summary>
	public static int? ReadInt32(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var value = ReadString(debugger, expr);
		if (value == null) return null;
		return TryParseInt(Clean(value), out var i) ? i : null;
	}

	/// <summary>Read a 64-bit signed integer from 'expr'</summary>
	public static long? ReadInt64(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var value = ReadString(debugger, expr);
		if (value == null) return null;
		var clean = Clean(value);

		if (clean.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
			return long.TryParse(clean.Substring(2), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var l) ? l : null;

		return long.TryParse(clean, NumberStyles.Integer, CultureInfo.InvariantCulture, out var ll) ? ll : null;
	}

	/// <summary>Read a boolean from 'expr'</summary>
	public static bool? ReadBool(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var value = ReadString(debugger, expr);
		if (value == null) return null;
		return bool.TryParse(Clean(value), out var b) ? b : null;
	}

	/// <summary>Read a string from 'expr' (strips surrounding quotes for managed strings)</summary>
	public static string? ReadString(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();

		var dbg_expr = debugger.GetExpression(expr);
		if (!dbg_expr.IsValidValue)
			return null;

		var value = dbg_expr.Value;

		// Managed string expressions render as "\"hello\"" — unwrap the surrounding quotes.
		if (value.Length >= 2 && value[0] == '"' && value[value.Length - 1] == '"')
			value = value.Substring(1, value.Length - 2);

		return value;
	}

	/// <summary>Read raw display text without any cleaning (use only when other helpers don't fit)</summary>
	public static string? ReadRaw(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var dbg_expr = debugger.GetExpression(expr);
		return dbg_expr.IsValidValue ? dbg_expr.Value : null;
	}

	/// <summary>Get the type-name of a debug expression, or null if it cannot be evaluated</summary>
	public static string? TypeOf(Debugger debugger, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();
		var dbg_expr = debugger.GetExpression(expr);
		return dbg_expr.IsValidValue ? dbg_expr.Type : null;
	}

	/// <summary>Clean natvis/format suffixes (e.g. trailing 'f', 'D', wrapping braces)</summary>
	private static string Clean(string value)
	{
		if (string.IsNullOrEmpty(value))
			return value;

		value = value.Trim();

		// Strip surrounding braces from natvis-style display (e.g. "{1.5}").
		if (value.Length >= 2 && value[0] == '{' && value[value.Length - 1] == '}')
			value = value.Substring(1, value.Length - 2).Trim();

		// Strip trailing C# numeric suffixes (f, F, d, D, m, M, l, L, u, U).
		while (value.Length > 1)
		{
			var c = value[value.Length - 1];
			if (c == 'f' || c == 'F' || c == 'd' || c == 'D' || c == 'm' || c == 'M' ||
				c == 'l' || c == 'L' || c == 'u' || c == 'U')
				value = value.Substring(0, value.Length - 1);
			else
				break;
		}

		return value;
	}

	/// <summary>Parse an int allowing decimal or hex (0x...) form</summary>
	private static bool TryParseInt(string value, out int result)
	{
		if (value.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
			return int.TryParse(value.Substring(2), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out result);

		return int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out result);
	}
}
