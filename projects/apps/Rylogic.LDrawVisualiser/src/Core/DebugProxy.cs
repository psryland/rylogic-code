using System;
using System.Collections.Generic;
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
		private readonly TypeHandler m_type_handler;

		public DebugProxy(Debugger debugger, string path = "")
		{
			m_debugger = debugger;
			m_path = path;
			m_type_handler = new(debugger);
		}

		// Internal ctor used when chaining (TryGetMember/TryGetIndex/Evaluate) and when
		// script-registered readers create a sub-proxy — share the owning TypeHandler so
		// that script-registered readers and built-ins remain in scope across the chain.
		internal DebugProxy(Debugger debugger, string path, TypeHandler type_handler)
		{
			m_debugger = debugger;
			m_path = path;
			m_type_handler = type_handler;
		}

		/// <inheritdoc/>
		public override bool TryGetMember(GetMemberBinder binder, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			// Intercept RegisterReader as a property-style access too, so that scripts can
			// hold on to it as a delegate if they prefer that style. The DLR will normally
			// route 'vars.RegisterReader(...)' through TryInvokeMember though, so this is
			// only a convenience.
			if (binder.Name == "RegisterReader" && string.IsNullOrEmpty(m_path))
			{
				result = (Action<string, Func<dynamic, object?>>)((ty, fn) => m_type_handler.Register(ty, fn));
				return true;
			}

			var expr = string.IsNullOrEmpty(m_path) ? binder.Name : $"{m_path}.{binder.Name}";
			result = Evaluate(expr);
			return true;
		}

		/// <inheritdoc/>
		public override bool TryGetIndex(GetIndexBinder binder, object[] indexes, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var index_expr = string.Join(",", indexes);
			var expr = $"{m_path}[{index_expr}]";
			result = Evaluate(expr);
			return true;
		}

		/// <inheritdoc/>
		public override bool TryInvokeMember(InvokeMemberBinder binder, object?[]? args, out object? result)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			// ReadBytes("expr", size) — read arbitrary raw bytes from debuggee memory
			if (binder.Name == "ReadBytes" && args?.Length == 2 && args[0] is string expr2 && args[1] is int size)
			{
				result = DebugMemoryReader.ReadBytes(m_debugger, expr2, size);
				return true;
			}

			// RegisterReader("TypeName", v => ...) — install a script-supplied reader for
			// the exact debugger-reported type name. The lambda receives a DebugProxy
			// positioned at the matched expression and returns the parsed value (any type).
			// Registrations live for the duration of this script evaluation only.
			if (binder.Name == "RegisterReader" && args?.Length == 2 && args[0] is string ty_name && args[1] is Delegate del)
			{
				m_type_handler.Register(ty_name, v => del.DynamicInvoke(v));
				result = null;
				return true;
			}

			// Print(...) / PrintLine(...) — write to the "LDraw Visualiser" pane in the
			// VS Output window. Accepts any number of args; null-safe ToString on each,
			// joined without separators (matching Console.Write semantics).
			if (binder.Name == "Print" || binder.Name == "PrintLine")
			{
				var text = args == null ? string.Empty : string.Concat(System.Linq.Enumerable.Select(args, a => a?.ToString() ?? "null"));
				if (binder.Name == "PrintLine")
					OutputPane.WriteLine(text);
				else
					OutputPane.Write(text);
				result = null;
				return true;
			}

			result = null;
			return false;
		}

		/// <inheritdoc/>
		public override bool TryConvert(ConvertBinder binder, out object? result)
		{
			result = null;

			// This looks at the target type being converted to, and if it's a known type,
			// it tries to parse the current expression value as that type.
			ThreadHelper.ThrowIfNotOnUIThread();
			if (string.IsNullOrEmpty(m_path))
				return false;

			var dbg_expr = m_debugger.GetExpression(m_path);
			if (!dbg_expr.IsValidValue)
				return false;

			// Allow implicit conversion to numeric types by evaluating the current path
			result = m_type_handler.Dispatch(dbg_expr.Type, m_path);
			return result != null;
		}

		/// <summary></summary>
		private object Evaluate(string expr)
		{
			ThreadHelper.ThrowIfNotOnUIThread();
			Expression dbg_expr = m_debugger.GetExpression(expr);

			// Return a proxy that will also fail gracefully on further access
			if (!dbg_expr.IsValidValue)
				return new DebugProxy(m_debugger, expr, m_type_handler);

			// Handle known types
			if (dbg_expr.Type is string ty && ty.Length != 0)
			{
				var result = m_type_handler.Dispatch(ty, expr);
				if (result != null)
					return result;
			}

			// Not known return a new proxy for further chaining
			return new DebugProxy(m_debugger, expr, m_type_handler);
		}

		/// <inheritdoc/>
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
