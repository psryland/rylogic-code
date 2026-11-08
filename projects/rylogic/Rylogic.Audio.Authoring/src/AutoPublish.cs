using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// Collects edited sound IDs and publishes them together once editing pauses. Each <see cref="Touch"/> restarts the pause, so a burst of edits makes one
/// publish. Publishes never overlap: IDs touched while a publish runs are published by the next one, after another pause. Every member is thread-safe.
/// </summary>
public sealed class AutoPublish : IDisposable
{
	private readonly object m_lock = new();
	private readonly TimeSpan m_delay;
	private readonly Func<IReadOnlyList<string>, Task> m_publish;
	private readonly Timer m_timer;
	private List<string> m_waiting = [];
	private List<string> m_running = [];

	/// <summary>
	/// Call 'publish' with the edited sound IDs, in first-edit order, once 'delay' passes without another edit. 'publish' runs on the thread pool and must
	/// report its own failures, because nothing observes an exception it throws.
	/// </summary>
	public AutoPublish(TimeSpan delay, Func<IReadOnlyList<string>, Task> publish)
	{
		m_delay = delay;
		m_publish = publish;
		m_timer = new Timer(_ => _ = PublishWaiting(), null, Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);
	}

	/// <summary>The sound IDs waiting for the pause to end.</summary>
	public IReadOnlyList<string> Waiting
	{
		get
		{
			lock (m_lock)
				return [.. m_waiting];
		}
	}

	/// <summary>The sound IDs being published now. Empty when no publish is running.</summary>
	public IReadOnlyList<string> Running
	{
		get
		{
			lock (m_lock)
				return [.. m_running];
		}
	}

	/// <summary>Record an edit to 'sound_id' and restart the pause.</summary>
	public void Touch(string sound_id)
	{
		// The ID is published once however often it is edited
		lock (m_lock)
		{
			if (!m_waiting.Contains(sound_id))
				m_waiting.Add(sound_id);

			m_timer.Change(m_delay, Timeout.InfiniteTimeSpan);
		}
	}

	/// <summary>Stop the timer. Waiting IDs are not published.</summary>
	public void Dispose()
	{
		m_timer.Dispose();
	}

	/// <summary>Publish the waiting IDs, unless a publish is already running; that one restarts the pause when it ends.</summary>
	private async Task PublishWaiting()
	{
		// Take the waiting IDs as the running set
		IReadOnlyList<string> ids;
		lock (m_lock)
		{
			if (m_running.Count != 0 || m_waiting.Count == 0)
				return;

			(m_running, m_waiting) = (m_waiting, []);
			ids = [.. m_running];
		}

		// IDs edited during the publish wait for another pause, because the edits may still be going on
		try
		{
			await m_publish(ids);
		}
		finally
		{
			lock (m_lock)
			{
				m_running = [];
				if (m_waiting.Count != 0)
					m_timer.Change(m_delay, Timeout.InfiniteTimeSpan);
			}
		}
	}
}
