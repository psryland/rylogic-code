using System;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Extn
{
	/// <summary>Helpers for scheduling work through the current managed thread context.</summary>
	public static class SynchronizationContext_
	{
		/// <summary>Post an action to the current synchronization context after a delay.</summary>
		public static void BeginInvokeDelayed(Action action, TimeSpan delay)
		{
			var context = SynchronizationContext.Current ?? throw new InvalidOperationException("A synchronization context is required to schedule delayed work.");
			context.BeginInvokeDelayed(action, delay);
		}

		/// <summary>Post an action to a synchronization context after a delay.</summary>
		public static void BeginInvokeDelayed(this SynchronizationContext context, Action action, TimeSpan delay)
		{
			if (context == null)
				throw new ArgumentNullException(nameof(context));
			if (action == null)
				throw new ArgumentNullException(nameof(action));
			if (delay < TimeSpan.Zero)
				throw new ArgumentOutOfRangeException(nameof(delay), delay, "The delay cannot be negative.");

			// Delay off-thread so the owning context remains responsive, then marshal the action back to it.
			_ = PostDelayedAsync(context, action, delay);
		}

		/// <summary>Wait for the requested delay without capturing a caller context.</summary>
		private static async Task PostDelayedAsync(SynchronizationContext context, Action action, TimeSpan delay)
		{
			if (delay != TimeSpan.Zero)
				await Task.Delay(delay).ConfigureAwait(false);

			context.Post(_ => action(), null);
		}
	}
}
