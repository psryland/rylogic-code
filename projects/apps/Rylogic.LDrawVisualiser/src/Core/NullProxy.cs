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

		// Implicit conversion operators allow the DLR to resolve method overloads
		// when a NullProxy is passed as an argument (e.g. builder.o2w(vars.o2w)).
		// TryConvert only fires for explicit casts, not method argument binding.
		public static implicit operator m4x4(NullProxy _) => m4x4.Identity;
		public static implicit operator m3x3(NullProxy _) => m3x3.Identity;
		public static implicit operator m2x2(NullProxy _) => m2x2.Identity;
		public static implicit operator Quat(NullProxy _) => Quat.Identity;
		public static implicit operator v4(NullProxy _) => v4.Zero;
		public static implicit operator v3(NullProxy _) => v3.Zero;
		public static implicit operator v2(NullProxy _) => v2.Zero;
		public static implicit operator decimal(NullProxy _) => 0m;
		public static implicit operator double(NullProxy _) => 0.0;
		public static implicit operator float(NullProxy _) => 0f;
		public static implicit operator ulong(NullProxy _) => 0;
		public static implicit operator long(NullProxy _) => 0;
		public static implicit operator uint(NullProxy _) => 0;
		public static implicit operator int(NullProxy _) => 0;
		public static implicit operator ushort(NullProxy _) => 0;
		public static implicit operator short(NullProxy _) => 0;
		public static implicit operator byte(NullProxy _) => 0;
		public static implicit operator sbyte(NullProxy _) => 0;
		public static implicit operator char(NullProxy _) => '\0';
		public static implicit operator bool(NullProxy _) => false;
		public static implicit operator string(NullProxy _) => string.Empty;
		public static implicit operator byte[](NullProxy _) => [];

		/// <inheritdoc/>
		public override bool TryGetMember(GetMemberBinder binder, out object? result)
		{
			result = Instance;
			return true;
		}

		/// <inheritdoc/>
		public override bool TryGetIndex(GetIndexBinder binder, object[] indexes, out object? result)
		{
			result = Instance;
			return true;
		}

		/// <inheritdoc/>
		public override bool TryConvert(ConvertBinder binder, out object? result)
		{
			if (binder.Type == typeof(string)) { result = string.Empty; return true; }
			if (binder.Type == typeof(m4x4)) { result = m4x4.Identity; return true; }
			if (binder.Type == typeof(m3x3)) { result = m3x3.Identity; return true; }
			if (binder.Type == typeof(m2x2)) { result = m2x2.Identity; return true; }
			if (binder.Type == typeof(Quat)) { result = Quat.Identity; return true; }
			if (binder.Type == typeof(v4)) { result = v4.Zero; return true; }
			if (binder.Type == typeof(v3)) { result = v3.Zero; return true; }
			if (binder.Type == typeof(v2)) { result = v2.Zero; return true; }
			if (binder.Type.IsValueType) // For any other value type, return default
			{
				result = Activator.CreateInstance(binder.Type);
				return true;
			}
			result = null;
			return true;
		}

		/// <inheritdoc/>
		public override bool TryInvokeMember(InvokeMemberBinder binder, object?[]? args, out object? result)
		{
			// ReadBytes — return an empty buffer
			if (binder.Name == "ReadBytes")
			{
				var size = args?.Length >= 2 && args[1] is int s ? s : 0;
				result = new byte[size];
				return true;
			}

			// Print/PrintLine — still write to the output pane even when no debugger is
			// attached, so script authors can sanity-check their formatting without being
			// in break mode.
			if (binder.Name == "Print" || binder.Name == "PrintLine")
			{
				Microsoft.VisualStudio.Shell.ThreadHelper.ThrowIfNotOnUIThread();
				var text = args == null ? string.Empty : string.Concat(System.Linq.Enumerable.Select(args, a => a?.ToString() ?? "null"));
				if (binder.Name == "PrintLine")
					OutputPane.WriteLine(text);
				else
					OutputPane.Write(text);
				result = null;
				return true;
			}

			result = Instance;
			return true;
		}

		/// <inheritdoc/>
		public override string ToString() => "<NullProxy:no-debugger>";
	}
}
