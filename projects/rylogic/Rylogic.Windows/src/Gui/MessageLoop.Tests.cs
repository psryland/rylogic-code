#if PR_UNITTESTS
using System;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Common;
using Rylogic.Interop.Win32;
using Rylogic.Windows.Gui;

namespace Rylogic.UnitTests;

/// <summary>Deadline waits must exclude completed callback work without changing step or message-pump policy.</summary>
[TestFixture]
public class TestSimMessageLoop
{
	/// <summary>Short callback work consumes the current period for both variable and fixed loops.</summary>
	[Test]
	public void CallbackTimeConsumesDeadline()
	{
		// Use a stopped entry clock and start it inside the callback, avoiding assumptions about initial thread scheduling.
		foreach (var variable in new[] { false, true })
		{
			var pump = new SimMessageLoop();
			var clock = Field<Stopwatch>(pump, "m_clock");
			var callback_end = 0L;
			var elapsed = TimeSpan.Zero;
			pump.AddLoop(1, variable, dt =>
			{
				clock.Start();
				Thread.Sleep(40);
				callback_end = clock.ElapsedMilliseconds;
				elapsed = dt;
			});
			var loop = Field<SimMessageLoop.LoopCont>(pump, "m_loop")[0];
			loop.Clock = -1000;
			var timeout = pump.StepLoops();
			clock.Stop();
			Assert.Equal(TimeSpan.FromMilliseconds(1000), elapsed);
			Assert.Equal(0L, loop.Clock);
			Assert.True(timeout >= 0 && timeout <= Math.Max(0L, loop.NextStepTime - callback_end));
		}
	}

	/// <summary>Overruns yield to the message pump immediately instead of adding another quantized period.</summary>
	[Test]
	public void OverrunReturnsZeroWithoutExtraCallback()
	{
		// The captured entry time continues to bound callbacks even though the clock advances during their work.
		foreach (var variable in new[] { false, true })
		{
			var pump = new SimMessageLoop();
			var clock = Field<Stopwatch>(pump, "m_clock");
			var calls = 0;
			pump.AddLoop(60, variable, _ =>
			{
				++calls;
				clock.Start();
				Thread.Sleep(32);
			});
			var loop = Field<SimMessageLoop.LoopCont>(pump, "m_loop")[0];
			Assert.Equal(16, loop.StepRateMS);
			loop.Clock = -16;
			Assert.Equal(0, pump.StepLoops());
			clock.Stop();
			Assert.Equal(1, calls);
			Assert.Equal(0L, loop.Clock);
		}
	}

	/// <summary>Fixed catch-up retains the caller's cap; variable elapsed collapses lateness into one callback.</summary>
	[Test]
	public void CatchUpPreservesCapAndVariableElapsed()
	{
		// A stopped stopwatch provides deterministic overdue input without a production clock abstraction.
		foreach (var variable in new[] { false, true })
		{
			var pump = new SimMessageLoop(3);
			var calls = 0;
			var elapsed_ms = 0.0;
			pump.AddLoop(60, variable, elapsed =>
			{
				++calls;
				elapsed_ms += elapsed.TotalMilliseconds;
			});
			var loop = Field<SimMessageLoop.LoopCont>(pump, "m_loop")[0];
			loop.Clock = -160;
			Assert.Equal(variable ? 16 : 0, pump.StepLoops());
			Assert.Equal(variable ? 1 : 3, calls);
			Assert.Equal(variable ? 160.0 : 48.0, elapsed_ms);
			Assert.Equal(variable ? 0L : -112L, loop.Clock);
		}
	}

	/// <summary>Posted messages are drained between bounded callback batches without a visible window.</summary>
	[Test]
	public void PumpDrainsPostedMessages()
	{
		// Post on this fixture's thread only; no desktop window or input focus is involved.
		User32.PeekMessage(out _, IntPtr.Zero, 0, 0, 0);
		var pump = new SimMessageLoop(1);
		var calls = 0;
		var filter = new ProbeFilter(() => calls);
		pump.AddMessageFilter(filter);
		pump.AddLoop(60, true, _ =>
		{
			++calls;
			Assert.Equal(1, User32.PostThreadMessage(GetCurrentThreadId(), 0x8001, 0, 0));
			User32.PostQuitMessage(37);
		});
		Assert.Equal(37, pump.Run());
		Assert.Equal(1, filter.CallsAtMessage);
		Assert.Equal(1, calls);
	}

	/// <summary>A due loop with a ready handle waits for one signal and consumes it when it steps.</summary>
	[Test]
	public void ReadyHandleGatesDueLoop()
	{
		// A stopped clock keeps the loop due but before its ready deadline, so only the semaphore controls the step.
		using var ready = new Semaphore(0, 1);
		var pump = new SimMessageLoop();
		var calls = 0;
		pump.AddLoop(60, true, ready.SafeWaitHandle.DangerousGetHandle(), _ => ++calls);
		var loop = Field<SimMessageLoop.LoopCont>(pump, "m_loop")[0];
		loop.Clock = -16;
		Assert.Equal(16, pump.StepLoops());
		Assert.Equal(0, calls);
		Assert.Equal(1, Field<System.Collections.Generic.List<SimMessageLoop.Loop>>(pump, "m_blocked").Count);

		// Signalling the handle releases exactly one step and the step consumes the signal.
		ready.Release();
		Assert.Equal(16, pump.StepLoops());
		Assert.Equal(1, calls);
		Assert.False(ready.WaitOne(0));
		Assert.False(loop.ReadyAcquired);
	}

	/// <summary>A loop whose handle is never signalled still steps one period after it became due.</summary>
	[Test]
	public void ReadyDeadlineStepsWithoutSignal()
	{
		// The stopped clock sits exactly on the ready deadline.
		using var ready = new Semaphore(0, 1);
		var pump = new SimMessageLoop();
		var calls = 0;
		pump.AddLoop(60, true, ready.SafeWaitHandle.DangerousGetHandle(), _ => ++calls);
		var loop = Field<SimMessageLoop.LoopCont>(pump, "m_loop")[0];
		loop.Clock = -32;
		Assert.Equal(16, pump.StepLoops());
		Assert.Equal(1, calls);
		Assert.Equal(0L, loop.Clock);
	}

	/// <summary>The pump wakes on a ready handle signalled from another thread, before the ready deadline.</summary>
	[Test]
	public void PumpWakesOnReadyHandle()
	{
		// A one-second period makes the ready deadline far later than the signal, so an early step proves the handle woke the pump.
		User32.PeekMessage(out _, IntPtr.Zero, 0, 0, 0);
		using var ready = new Semaphore(0, 1);
		var pump = new SimMessageLoop(1);
		var clock = Field<Stopwatch>(pump, "m_clock");
		var step_time = 0L;
		pump.AddLoop(1, true, ready.SafeWaitHandle.DangerousGetHandle(), _ =>
		{
			step_time = clock.ElapsedMilliseconds;
			User32.PostQuitMessage(5);
		});
		Field<SimMessageLoop.LoopCont>(pump, "m_loop")[0].Clock = -1000;
		using var signaller = new Timer(_ => ready.Release(), null, 50, Timeout.Infinite);
		Assert.Equal(5, pump.Run());
		Assert.True(step_time >= 40 && step_time < 900);
		Assert.False(ready.WaitOne(0));
	}

	/// <summary>Read only the scheduler's own private fixture state, not runtime stopwatch implementation fields.</summary>
	private static T Field<T>(SimMessageLoop pump, string name)
	{
		return (T)(typeof(SimMessageLoop).GetField(name, BindingFlags.Instance | BindingFlags.NonPublic)?.GetValue(pump) ?? throw new InvalidOperationException(name));
	}

	/// <summary>Consume the fixture's thread message before ordinary dispatch.</summary>
	private sealed class ProbeFilter(Func<int> calls) : IMessageFilter
	{
		public int CallsAtMessage { get; private set; }
		public bool TranslateMessage(ref Win32.MESSAGE msg)
		{
			if (msg.message != 0x8001)
				return false;

			CallsAtMessage = calls();
			return true;
		}
	}

	/// <summary>Identify only the fixture's message-pump thread.</summary>
	[DllImport("kernel32.dll")]
	private static extern int GetCurrentThreadId();
}
#endif
