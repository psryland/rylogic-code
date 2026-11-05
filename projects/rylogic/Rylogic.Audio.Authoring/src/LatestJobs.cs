using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>The outcome of a submitted job.</summary>
public enum EJobOutcome
{
	/// <summary>The job finished and was still the newest for its slot, so its result was delivered.</summary>
	Completed,

	/// <summary>The job failed and was still the newest for its slot, so its error was delivered.</summary>
	Failed,

	/// <summary>A newer job for the same slot was submitted, so this job's result was discarded.</summary>
	Superseded,
}

/// <summary>
/// Runs background work in named slots, where only the newest job of each slot may deliver a result. Submitting a job cancels the slot's previous job,
/// and a superseded job can never replace a newer result, whatever order the jobs finish in.
/// </summary>
public sealed class LatestJobs
{
	/// <summary>A slot's newest job.</summary>
	private sealed class Slot
	{
		public long Generation;
		public CancellationTokenSource Cancel = new();
		public string Description = string.Empty;
		public bool Running;
	}

	private readonly Dictionary<string, Slot> m_slots = [];
	private readonly SemaphoreSlim m_concurrency;
	private long m_next_generation;

	/// <summary>Allow up to 'max_concurrency' jobs to do work at once; others wait their turn.</summary>
	public LatestJobs(int max_concurrency)
	{
		m_concurrency = new SemaphoreSlim(max_concurrency, max_concurrency);
	}

	/// <summary>The descriptions of the newest jobs that are queued or running, by slot.</summary>
	public IReadOnlyDictionary<string, string> Pending
	{
		get
		{
			lock (m_slots)
				return m_slots.Where(x => x.Value.Running).ToDictionary(x => x.Key, x => x.Value.Description);
		}
	}

	/// <summary>
	/// Run 'work' as the newest job in 'slot'. 'on_completed' or 'on_failed' is called only if no newer job has been submitted to the slot by then, and
	/// is called while no other job of the slot can deliver, so callers can publish the result directly.
	/// </summary>
	public async Task<EJobOutcome> Run<T>(string slot, string description, Func<CancellationToken, Task<T>> work, Action<T> on_completed, Action<Exception> on_failed)
	{
		// Become the slot's newest job, cancelling whatever was there
		Slot state;
		long generation;
		CancellationToken cancel;
		lock (m_slots)
		{
			if (!m_slots.TryGetValue(slot, out state!))
				m_slots[slot] = state = new Slot();

			state.Cancel.Cancel();
			state.Cancel.Dispose();
			state.Cancel = new CancellationTokenSource();
			state.Generation = generation = ++m_next_generation;
			state.Description = description;
			state.Running = true;
			cancel = state.Cancel.Token;
		}

		// Do the work outside the lock; cancellation is how a superseded job stops early
		T result = default!;
		Exception? error = null;
		try
		{
			await m_concurrency.WaitAsync(cancel);
			try
			{
				result = await work(cancel);
			}
			finally
			{
				m_concurrency.Release();
			}
		}
		catch (Exception ex)
		{
			error = ex;
		}

		// Deliver only if still newest; the check and the delivery share the lock so a newer job cannot deliver in between
		lock (m_slots)
		{
			if (state.Generation != generation)
				return EJobOutcome.Superseded;

			state.Running = false;
			if (error != null)
			{
				on_failed(error);
				return EJobOutcome.Failed;
			}
			on_completed(result);
			return EJobOutcome.Completed;
		}
	}
}
