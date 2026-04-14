using System;
using System.Dynamic;
using Rylogic.Maths;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>
	/// A DynamicObject that returns sensible default values for all property accesses.
	/// Used as the 'vars' object when no debugger session is active, allowing scripts
	/// to run outside of break mode for testing and preview purposes.
	/// </summary>
	public class NullProxy : DynamicObject
	{
		public static readonly NullProxy Instance = new();

		public override bool TryGetMember(GetMemberBinder binder, out object? result)
		{
			// Return a new NullProxy for chaining (e.g. vars.obj.m_o2w.x)
			result = Instance;
			return true;
		}

		public override bool TryGetIndex(GetIndexBinder binder, object[] indexes, out object? result)
		{
			result = Instance;
			return true;
		}

		public override bool TryConvert(ConvertBinder binder, out object? result)
		{
			// Return sensible defaults for known Rylogic math types
			if (binder.Type == typeof(m4x4))
			{
				result = m4x4.Identity;
				return true;
			}
			if (binder.Type == typeof(v4))
			{
				result = v4.Zero;
				return true;
			}

			// Numeric defaults
			if (binder.Type == typeof(float))
			{
				result = 0f;
				return true;
			}
			if (binder.Type == typeof(double))
			{
				result = 0.0;
				return true;
			}
			if (binder.Type == typeof(int))
			{
				result = 0;
				return true;
			}
			if (binder.Type == typeof(bool))
			{
				result = false;
				return true;
			}
			if (binder.Type == typeof(string))
			{
				result = "";
				return true;
			}

			// For any other value type, return default
			if (binder.Type.IsValueType)
			{
				result = Activator.CreateInstance(binder.Type);
				return true;
			}

			result = null;
			return true;
		}

		public override bool TryInvokeMember(InvokeMemberBinder binder, object?[]? args, out object? result)
		{
			// Allow method calls on the proxy to chain without throwing
			result = Instance;
			return true;
		}

		public override string ToString() => "<NullProxy:no-debugger>";
	}
}
