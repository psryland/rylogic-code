using System;
using System.Dynamic;
using System.Globalization;
using EnvDTE;
using Microsoft.VisualStudio.Shell;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>
	/// A DynamicObject that bridges property access chains to DTE.Debugger.GetExpression() calls.
	/// Usage: vars.obj1.m_o2w.x.x → GetExpression("obj1.m_o2w.x.x") → parsed float value.
	/// Struct members return a new DebugProxy for further chaining.
	/// Leaf scalars are parsed and returned as typed values.
	/// </summary>
	public class DebugProxy : DynamicObject
	{
		private readonly Debugger m_debugger;
		private readonly string m_path;

		public DebugProxy(Debugger debugger, string path = "")
		{
			m_debugger = debugger;
			m_path = path;
		}

		public override bool TryGetMember(GetMemberBinder binder, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var expr = string.IsNullOrEmpty(m_path) ? binder.Name : $"{m_path}.{binder.Name}";
			result = Evaluate(expr);
			return true;
		}

		public override bool TryGetIndex(GetIndexBinder binder, object[] indexes, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var index_expr = string.Join(",", indexes);
			var expr = $"{m_path}[{index_expr}]";
			result = Evaluate(expr);
			return true;
		}

		public override bool TryInvokeMember(InvokeMemberBinder binder, object?[]? args, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			// ReadShapeBytes("expr") — read a collision shape's raw bytes from debuggee memory
			if (binder.Name == "ReadShapeBytes" && args?.Length == 1 && args[0] is string expr)
			{
				result = DebugMemoryReader.ReadShapeBytes(m_debugger, expr);
				return true;
			}

			// ReadBytes("expr", size) — read arbitrary raw bytes from debuggee memory
			if (binder.Name == "ReadBytes" && args?.Length == 2 && args[0] is string expr2 && args[1] is int size)
			{
				result = DebugMemoryReader.ReadBytes(m_debugger, expr2, size);
				return true;
			}

			result = null;
			return false;
		}

		public override bool TryConvert(ConvertBinder binder, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			// Allow implicit conversion to numeric types by evaluating the current path
			if (!string.IsNullOrEmpty(m_path))
			{
				var dbg_expr = m_debugger.GetExpression(m_path);
				if (dbg_expr.IsValidValue)
				{
					var value = CleanNumericString(dbg_expr.Value);
					if (binder.Type == typeof(float) && float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var f))
					{
						result = f;
						return true;
					}
					if (binder.Type == typeof(double) && double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
					{
						result = d;
						return true;
					}
					if (binder.Type == typeof(int) && TryParseInt(value, out var i))
					{
						result = i;
						return true;
					}
					if (binder.Type == typeof(string))
					{
						result = dbg_expr.Value;
						return true;
					}
				}
			}

			result = null;
			return false;
		}

		private object Evaluate(string expr)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var dbg_expr = m_debugger.GetExpression(expr);
			if (!dbg_expr.IsValidValue)
			{
				// Return a proxy that will also fail gracefully on further access
				return new DebugProxy(m_debugger, expr);
			}

			var value = CleanNumericString(dbg_expr.Value);

			// Try to parse as numeric leaf values
			if (float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var f))
				return f;

			if (TryParseInt(value, out var i))
				return i;

			// Check for boolean
			if (bool.TryParse(value, out var b))
				return b;

			// Not a scalar — return a new proxy for further chaining
			return new DebugProxy(m_debugger, expr);
		}

		/// <summary>Clean natvis-formatted values (strip suffixes like 'f', braces, type prefixes)</summary>
		private static string CleanNumericString(string value)
		{
			if (string.IsNullOrEmpty(value))
				return value;

			value = value.Trim();

			// Strip trailing 'f' suffix (e.g., "1.5f")
			if (value.EndsWith("f", StringComparison.OrdinalIgnoreCase) && value.Length > 1)
				value = value.Substring(0, value.Length - 1);

			// Strip surrounding braces from natvis display (e.g., "{1.5}")
			if (value.StartsWith("{") && value.EndsWith("}"))
				value = value.Substring(1, value.Length - 2).Trim();

			return value;
		}

		private static bool TryParseInt(string value, out int result)
		{
			// Handle hex format from debugger (e.g., "0x0000000a")
			if (value.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
				return int.TryParse(value.Substring(2), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out result);

			return int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out result);
		}

		public override string ToString()
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			if (string.IsNullOrEmpty(m_path))
				return "<DebugProxy:root>";

			var dbg_expr = m_debugger.GetExpression(m_path);
			return dbg_expr.IsValidValue ? dbg_expr.Value : $"<invalid:{m_path}>";
		}
	}
}
