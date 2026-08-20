//***************************************************
// Event Batcher
//  Copyright (c) Rylogic Ltd 2011
//***************************************************
using System;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Common
{
	/// <summary>Batch events by time</summary>
	public sealed class EventBatcher : IDisposable
	{
		// Notes:
		// Used to group bursts of events into a few events.
		// EventBatcher does the following:
		// - Trigger on the first event received, (optional, see TriggerOnFirst)
		// - Collect subsequent events,
		// - Trigger every 'Delay' interval if events have been received since the last trigger

		private readonly CancellationTokenSource m_shutdown;
		private readonly SynchronizationContext m_sync_context;
		private bool m_disposed;

		/// <summary>The number of times Signal has been called since 'Action' was last raised</summary>
		private int m_count;

		public EventBatcher()
			: this(TimeSpan.FromMilliseconds(10))
		{ }
		public EventBatcher(EventHandler action)
			: this(action, TimeSpan.FromMilliseconds(10))
		{ }
		public EventBatcher(EventHandler action, TimeSpan delay)
			: this(() => action(null, EventArgs.Empty), delay)
		{ }
		public EventBatcher(Action action)
			: this(action, TimeSpan.FromMilliseconds(10))
		{ }
		public EventBatcher(Action action, TimeSpan delay)
			: this(delay, CurrentContext(), action)
		{ }
		public EventBatcher(Action action, TimeSpan delay, SynchronizationContext sync_context)
			: this(delay, sync_context, action)
		{ }
		public EventBatcher(TimeSpan delay)
			: this(delay, CurrentContext())
		{ }
		public EventBatcher(TimeSpan delay, SynchronizationContext sync_context)
			: this(delay, sync_context, null)
		{ }
		private EventBatcher(TimeSpan delay, SynchronizationContext sync_context, Action? action)
		{
			if (delay < TimeSpan.Zero)
				throw new ArgumentOutOfRangeException(nameof(delay), delay, "The delay cannot be negative.");

			m_shutdown = new CancellationTokenSource();
			m_sync_context = sync_context ?? throw new ArgumentNullException(nameof(sync_context));
			m_disposed = false;
			Delay = delay;
			m_count = 0;
			Immediate = false;
			TriggerOnFirst = false;

			if (action != null)
				Action += action;
		}
		public void Dispose()
		{
			if (m_disposed)
				return;

			m_disposed = true;
			Action = null;
			m_shutdown.Cancel();
		}

		/// <summary>The callback called when events have been signalled</summary>
		public event Action? Action;

		/// <summary>If true, 'Action' will be called on the first signal in a batch. If false, will be called after 'Delay'</summary>
		public bool TriggerOnFirst { get; set; }

		/// <summary>The time between subsequent Action invocations</summary>
		public TimeSpan Delay { get; set; }

		/// <summary>The thread context in which to invoke <see cref="Action"/>.</summary>
		public SynchronizationContext SynchronizationContext => m_sync_context;

		/// <summary>Toggle switch for batching on/off</summary>
		public bool Immediate
		{
			get => m_immediate || AllImmediate;
			set => m_immediate = value;
		}
		private bool m_immediate;

		/// <summary>Global switch to make all event batcher's immediate or not (use for debugging)</summary>
		public static bool AllImmediate { get; set; }

		/// <summary>
		/// Signal the event. Signal can be called multiple times.
		/// Note: this can be called from any thread, the resulting event will be marshalled
		/// to the synchronization context provided in the constructor of the event batcher.</summary>
		public void Signal(object? sender = null, EventArgs? args = null)
		{
			// Signals can race with disposal because event producers may be running on background threads.
			if (Action == null || m_disposed)
				return;

			// Immediate mode remains synchronous while preserving the owning thread contract.
			if (Immediate)
			{
				InvokeSynchronously();
				return;
			}

			// Only the first signal schedules work; later signals are represented by the shared count.
			if (Interlocked.Increment(ref m_count) != 1)
				return;

			if (TriggerOnFirst)
				m_sync_context.Post(_ => InvokeIfActive(), null);

			_ = CompleteBatchAsync(m_shutdown.Token);
		}

		/// <summary>
		/// Signal the event synchronously.
		/// Note: this can be called from any thread, the resulting event will be marshalled
		/// to the synchronization context provided in the constructor of the event batcher.</summary>
		public void SignalImmediate(object? sender = null, EventArgs? args = null)
		{
			if (Action == null || m_disposed)
				return;

			InvokeSynchronously();
		}

		/// <summary>Capture the caller's synchronization context for the convenience constructors.</summary>
		private static SynchronizationContext CurrentContext()
		{
			return SynchronizationContext.Current ?? throw new InvalidOperationException("EventBatcher requires a synchronization context.");
		}

		/// <summary>Complete the current batch after its collection interval.</summary>
		private async Task CompleteBatchAsync(CancellationToken shutdown)
		{
			try
			{
				await Task.Delay(Delay, shutdown).ConfigureAwait(false);
			}
			catch (OperationCanceledException) when (shutdown.IsCancellationRequested)
			{
				return;
			}

			// Reset before posting so a concurrent signal starts a distinct following batch.
			var count = Interlocked.Exchange(ref m_count, 0);
			if (count > 1 || (count == 1 && !TriggerOnFirst))
				m_sync_context.Post(_ => InvokeIfActive(), null);
		}

		/// <summary>Invoke the current action unless disposal cancelled the batch.</summary>
		private void InvokeIfActive()
		{
			if (!m_disposed)
				Action?.Invoke();
		}

		/// <summary>Invoke on the owning context and wait for completion.</summary>
		private void InvokeSynchronously()
		{
			if (ReferenceEquals(SynchronizationContext.Current, m_sync_context))
				InvokeIfActive();
			else
				m_sync_context.Send(_ => InvokeIfActive(), null);
		}
	}

	/// <summary>Batch events per message pump loop</summary>
	public struct Trigger
	{
		// Notes:
		// - Much simpler than an EventBatcher
		// How to use:
		//  private Trigger m_update;
		//	public void TriggerUpdate()
		//	{
		//		if (m_update.Pending) return; // remember ' || !IsHandleCreated' if using Control.BeginInvoke
		//		m_update.Signal();
		//
		//		BeginInvoke(DoUpdate);
		//		void DoUpdate()
		//		{
		//			m_update.Actioned();
		//			...
		//			if (m_update.Pending)
		//				return; // abort, another trigger has been set
		//		}
		//	}
		// or:
		//	public void TriggerUpdate()
		//	{
		//		if (m_update.Pending) return;
		//		m_update.Signal();
		//		ThreadPool.QueueUserWorkItem(_ =>
		//		{
		//			m_update.Actioned();
		//			...
		//			if (m_update.Pending)
		//				return; // abort, another trigger has been set
		//	
		//			BeginInvoke(MergeResults);
		//			void MergeResults()
		//			{
		//				if (m_update.Pending) return;
		//				...
		//			}
		//		});
		//	}
		public readonly bool Pending => m_issue != m_in_progress;
		private volatile int m_issue;
		private volatile int m_in_progress;

		public void Signal()
		{
			++m_issue;
		}
		public void Actioned()
		{
			m_in_progress = m_issue;
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Common;

	[TestFixture] public class TestEventBatcher
	{
		[Test] public void TestEventBatch()
		{
			var count = new int[2];
			var thread_id = Environment.CurrentManagedThreadId;
			using var mre_eb1 = new ManualResetEvent(false);
			using var mre_eb2 = new ManualResetEvent(false);
			var sync_context = new TestSynchronizationContext();
			var previous_sync_context = SynchronizationContext.Current;
			SynchronizationContext.SetSynchronizationContext(sync_context);
			try
			{
				// Not trigger on first, expect one call after the delay period
				using var eb1 = new EventBatcher(() =>
				{
					if (thread_id != Environment.CurrentManagedThreadId)
						throw new Exception("Event Batch should be called in the thread context that the batcher was created in");

					++count[0];
					mre_eb1.Set();
				}){TriggerOnFirst = false};

				// Trigger on first, expect one call at the start, and one after the delay period
				using var eb2 = new EventBatcher(() =>
				{
					if (thread_id != Environment.CurrentManagedThreadId)
						throw new Exception("Event Batch should be called in the thread context that the batcher was created in");

					++count[1];
					eb1.Signal();
					mre_eb2.Set();
				}){TriggerOnFirst = true};

				// Track and join the producer so no test-owned work can outlive its synchronization primitives.
				var signal_task = Task.Run(() =>
				{
					for (var i = 0; i != 10; ++i)
						eb2.Signal();
				});
				signal_task.GetAwaiter().GetResult();

				// Pump the test context until both delayed batches have returned to the owning thread.
				sync_context.RunUntil(() => count[0] == 1 && count[1] == 2, TimeSpan.FromSeconds(5));

				// Don't Signal() from this thread, need to test cross-thread support

				Assert.Equal(true, mre_eb1.WaitOne(0));
				Assert.Equal(true, mre_eb2.WaitOne(0));
				Assert.Equal(1, count[0]); // !TriggerOnFirst
				Assert.Equal(2, count[1]); //  TriggerOnFirst
			}
			finally
			{
				// Keep the unit-test runner isolated from the synthetic context.
				SynchronizationContext.SetSynchronizationContext(previous_sync_context);
			}
		}

		/// <summary>A deterministic single-thread synchronization context for batching tests.</summary>
		private sealed class TestSynchronizationContext : SynchronizationContext
		{
			private readonly AutoResetEvent m_ready = new(false);
			private readonly System.Collections.Concurrent.ConcurrentQueue<(SendOrPostCallback callback, object? state)> m_queue = new();

			/// <summary>Queue work for the owning test thread.</summary>
			public override void Post(SendOrPostCallback callback, object? state)
			{
				m_queue.Enqueue((callback, state));
				m_ready.Set();
			}

			/// <summary>Run queued work until the completion condition is met.</summary>
			public void RunUntil(Func<bool> complete, TimeSpan timeout)
			{
				var expiry = DateTime.UtcNow + timeout;
				for (; !complete(); )
				{
					if (m_queue.TryDequeue(out var work))
					{
						work.callback(work.state);
						continue;
					}
					if (DateTime.UtcNow >= expiry)
						throw new TimeoutException("Timed out waiting for synchronization-context work.");

					m_ready.WaitOne(TimeSpan.FromMilliseconds(10));
				}
			}
		}
	}
}

#endif
