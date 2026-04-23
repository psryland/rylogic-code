namespace Rylogic.LDrawVisualiser.Core
{
	using System.Collections.Generic;

	/// <summary>
	/// Per-expression cache of the last successfully-read value, keyed by full expression
	/// path (e.g. "obj.m_o2w.pos"). Lets the visualiser keep rendering with the most
	/// recent valid value when the underlying expression goes out of scope — typically
	/// when the user steps into a callee and locals from a higher stack frame disappear.
	///
	/// Lifetime: instance-owned by the tool window so it persists across script runs and
	/// across step events. Cleared when the debug session ends (the values would refer to
	/// a different process by then) and on demand from scripts via vars.ClearCache().
	///
	/// Note: keys are bare expression strings — there is no per-stack-frame qualification.
	/// If two unrelated functions both reference 'p', stepping between them will briefly
	/// surface the other function's last value. Acceptable trade-off for a debug aid.
	/// </summary>
	public class ExpressionCache
	{
		private readonly Dictionary<string, object> m_cache = new();

		/// <summary>Store 'value' as the last-known value for 'expr'. Null values are ignored
		/// so a transient null read doesn't wipe a previously-good cached value.</summary>
		public void Store(string expr, object? value)
		{
			if (string.IsNullOrEmpty(expr) || value == null)
				return;

			m_cache[expr] = value;
		}

		/// <summary>Look up the last-known value for 'expr'. Returns true if present.</summary>
		public bool TryGet(string expr, out object? value)
		{
			if (m_cache.TryGetValue(expr, out var hit))
			{
				value = hit;
				return true;
			}
			value = null;
			return false;
		}

		/// <summary>True if 'expr' has any cached value.</summary>
		public bool Contains(string expr) => !string.IsNullOrEmpty(expr) && m_cache.ContainsKey(expr);

		/// <summary>Drop everything. Called when the debug session ends and on demand.</summary>
		public void Clear() => m_cache.Clear();

		/// <summary>Number of cached entries (diagnostic).</summary>
		public int Count => m_cache.Count;
	}
}
