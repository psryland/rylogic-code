using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
using Rylogic.Interop.Win32;

namespace Rylogic.Windows.Gui
{
	// An interface for types that need to handle messages from the message
	// loop before TranslateMessage is called. Typically these are dialog windows
	// or windows with keyboard accelerators that need to call 'IsDialogMessage'
	// or 'TranslateAccelerator'
	public interface IMessageFilter
	{
		// Implementers should return true to halt processing of the message.
		// Typically, if you're just observing messages as they go past, return false.
		// If you're a dialog return the result of IsDialogMessage()
		// If you're a window with accelerators, return the result of TranslateAccelerator()
		bool TranslateMessage(ref Win32.MESSAGE msg);
	};

	/// <summary>Base class and basic implementation of a message loop</summary>
	public class MessageLoop : IMessageFilter
	{
		/// <summary>The collection of message filters filtering messages in this loop</summary>
		private readonly List<IMessageFilter> m_filters = [];

		public MessageLoop()
		{
			m_filters.Add(this);
		}

		// Subclasses should replace this method
		public virtual int Run()
		{
			Win32.MESSAGE msg;
			for (int result; (result = User32.GetMessage(out msg, IntPtr.Zero, 0, 0)) != 0;)
			{
				// GetMessage returns negative values for errors
				if (result <= 0)
					throw new Win32Exception("GetMessage failed");

				HandleMessage(ref msg);
			}
			return (int)msg.wparam;
		}

		// Add an instance that needs to handle messages before TranslateMessage is called
		public virtual void AddMessageFilter(IMessageFilter filter)
		{
			// Preserve this loop as the terminal fallback after every externally registered filter.
			m_filters.Insert(m_filters.Count - 1, filter);
		}

		// Remove a message filter from the chain of filters for this message loop
		public virtual void RemoveMessageFilter(IMessageFilter filter)
		{
			m_filters.Remove(filter);
		}

		// Pass the message to each filter. The last filter is this message loop which always handles the message.
		protected void HandleMessage(ref Win32.MESSAGE msg)
		{
			foreach (var filter in m_filters)
				if (filter.TranslateMessage(ref msg))
					break;
		}

		// The message loop is always the last filter in the chain
		public virtual bool TranslateMessage(ref Win32.MESSAGE msg)
		{
			User32.TranslateMessage(ref msg);
			User32.DispatchMessage(ref msg);
			return true;
		}
	}

	// Message loop that also manages and runs a priority queue of simulation loops 
	public class SimMessageLoop(int max_loop_steps) : MessageLoop
	{
		// Notes:
		//  - A message loop designed for simulation applications
		//    This loop sleeps the thread until the next frame is due or until messages arrive.
		//  - A loop can have a ready handle, such as an event signalled when a render target is free. A due loop then waits for its handle
		//    while still pumping messages, so the step starts when the resource is free instead of blocking inside the step.
		public delegate void StepFunc(TimeSpan elapsed);

		/// <summary>A loop represents a process that should be run at a given frame rate</summary>
		public class Loop(StepFunc step, int step_rate_ms, bool variable, IntPtr ready_handle)
		{
			public StepFunc Step = step;                    // The function to step in the loop
			public long Clock = 0;                          // The time this loop was last stepped (in ms)
			public int StepRateMS = step_rate_ms;           // (Minimum) step rate
			public bool IsVariable = variable;              // Variable step rate
			public IntPtr ReadyHandle = ready_handle;       // Optional handle that must be signalled before a due loop steps. Zero for timer-only loops
			public bool ReadyAcquired = false;              // True once a wait on 'ReadyHandle' has succeeded for the next step
			public Buf8 AvrStepTime = new();                // Last 8 execution times of the loop (in ms, capped at 255)
			public long NextStepTime => Clock + StepRateMS; // The next time to run this loop
			public long ReadyDeadline => NextStepTime + StepRateMS; // The time a due loop steps without its ready handle
			internal readonly IntPtr[] m_ready_handles = [ready_handle]; // Reusable wait array for polling 'ReadyHandle'
			public struct Buf8
			{
				public ulong u64;
				public void Add(byte v) => u64 = (u64 << 8 & ~0xFFUL) | v;
				public readonly double Avr() => BitConverter.GetBytes(u64).Average(x => (double)x);
			}

			/// <summary>True when a due loop may step at 'now'. A successful poll consumes one signal from 'ReadyHandle'.</summary>
			public bool IsReady(long now)
			{
				// Timer-only loops, and loops whose signal has already been consumed, are ready as soon as they are due.
				if (ReadyHandle == IntPtr.Zero || ReadyAcquired)
					return true;

				// Poll the handle without waiting. Successful waits on auto-reset objects consume the signal, so remember it.
				const int WAIT_OBJECT_0 = 0;
				if (User32.MsgWaitForMultipleObjects(1, m_ready_handles, false, 0, 0) == WAIT_OBJECT_0)
				{
					ReadyAcquired = true;
					return true;
				}

				// Step without the signal after one extra period, because a step that produced no work (e.g. nothing was presented) never signals it.
				return now >= ReadyDeadline;
			}
		}

		public class LoopCont : List<Loop> { };
		public class LoopOrder : List<int> { };

		private readonly int m_max_loop_steps = max_loop_steps; // The maximum number of loops to step before checking for messages
		private readonly Stopwatch m_clock = new();             // High performance clock
		private readonly LoopCont m_loop = [];                  // The loops to execute
		private readonly LoopOrder m_order = [];                // A priority queue of loops. The loop at position 0 is the next to be stepped
		private readonly List<Loop> m_blocked = [];             // Due loops waiting for their ready handles, in wait-array order after the deadline timer
		private long m_last_step_loops = 0;                     // The last time StepLoops was called.

		public SimMessageLoop()
			:this(10)
		{
		}

		/// <summary>Add a loop with an integer-millisecond period (60 Hz gives 16 ms). Variable loops receive elapsed entry-time;
		/// fixed loops advance by one period per callback. Callback work consumes the current deadline rather than adding a fixed delay.</summary>
		public void AddLoop(float frame_rate, bool variable, StepFunc step)
		{
			AddLoop(frame_rate, variable, IntPtr.Zero, step);
		}

		/// <summary>
		/// Add a loop that steps once it is due and 'ready_handle' is signalled, so the period is the minimum interval between steps.
		/// The loop consumes one signal per step by waiting on the handle, which suits auto-reset events and semaphores.
		/// If the handle is not signalled within one period after the step is due, the loop steps anyway, at half the requested rate, so steps that
		/// do not release the handle cannot stall the loop. The caller owns 'ready_handle' and must keep it valid while the loop runs.
		/// Zero gives a timer-only loop.
		/// </summary>
		public void AddLoop(float frame_rate, bool variable, IntPtr ready_handle, StepFunc step)
		{
			m_loop.Add(new Loop(step, (int)(1000f / frame_rate), variable, ready_handle));
			m_order.Add(m_loop.Count - 1);
		}

		/// <summary>Run the thread message pump while maintaining the desired loop rates</summary>
		public override int Run()
		{
			// Keep frame deadlines independent of the system's coarse message-wait timer resolution.
			using var deadline_timer = Kernel32.CreateWaitableTimerEx(IntPtr.Zero, null, Win32.EWaitableTimerCreateFlags.HIGH_RESOLUTION, Win32.ETimerAccess.MODIFY_STATE | Win32.ETimerAccess.SYNCHRONIZE);
			if (deadline_timer.IsInvalid)
				throw new Win32Exception(Marshal.GetLastWin32Error(), "Creating the high-resolution simulation-loop timer failed.");

			// The wait array holds the deadline timer followed by the ready handles of any blocked loops.
			var wait_handles = new IntPtr[1 + m_loop.Count];
			wait_handles[0] = deadline_timer.DangerousGetHandle();
			m_clock.Start();
			m_last_step_loops = 0;

			// Run the message pump loop
			for (; ; )
			{
				// Step any pending loops and get the time till the next loop to be stepped.
				var timeout = StepLoops();

				// Arm a high-resolution deadline without preventing queued window messages or ready handles from waking the owner thread earlier.
				for (var i = 0; i != m_blocked.Count; ++i)
					wait_handles[1 + i] = m_blocked[i].ReadyHandle;

				var wait_result = WaitForMessagesOrDeadline(deadline_timer, wait_handles, 1 + m_blocked.Count, timeout);
				if (wait_result == -1)
					throw new Win32Exception("MsgWaitForMultipleObjects failed");

				// A signalled ready handle has been consumed by the wait, so record it against its loop.
				var blocked_index = wait_result - 1;
				if (blocked_index >= 0 && blocked_index < m_blocked.Count)
					m_blocked[blocked_index].ReadyAcquired = true;

				for (Win32.MESSAGE msg; User32.PeekMessage(out msg, IntPtr.Zero, 0, 0, Win32.EPeekMessageFlags.Remove);)
				{
					// Exit the message pump?
					if (msg.message == Win32.WM_QUIT)
						return (int)msg.wparam;

					// Pump the message
					HandleMessage(ref msg);
				}
			}
		}

		/// <summary>Wait until the exact relative frame deadline, a ready handle, or new window input becomes available.</summary>
		private static int WaitForMessagesOrDeadline(SafeWaitHandle deadline_timer, IntPtr[] wait_handles, int wait_count, int timeout_milliseconds)
		{
			// Poll without arming the timer when callbacks remain overdue.
			if (timeout_milliseconds <= 0)
				return User32.MsgWaitForMultipleObjects(0, null, false, 0, Win32.QS_ALLPOSTMESSAGE | Win32.QS_ALLINPUT | Win32.QS_ALLEVENTS);

			// Replace any earlier deadline with one relative high-resolution timer interval.
			var due_time = -(long)timeout_milliseconds * TimeSpan.TicksPerMillisecond;
			if (!Kernel32.SetWaitableTimer(deadline_timer, ref due_time, 0, IntPtr.Zero, IntPtr.Zero, false))
				throw new Win32Exception(Marshal.GetLastWin32Error(), "Arming the high-resolution simulation-loop timer failed.");

			return User32.MsgWaitForMultipleObjects(wait_count, wait_handles, false, -1, Win32.QS_ALLPOSTMESSAGE | Win32.QS_ALLINPUT | Win32.QS_ALLEVENTS);
		}

		// Call 'Step' on all loops that are pending
		// Returns the time in milliseconds until the next loop needs to be stepped
		public int StepLoops()
		{
			m_blocked.Clear();
			if (m_loop.Count == 0)
				return int.MaxValue;

			var now = m_clock.ElapsedMilliseconds;
			var dt = now - m_last_step_loops;
			m_last_step_loops = now;

			// Check the StepLoops function is being called frequently enough.
			// If not, it's probably due to a blocking windows message handler
			//foreach (var loop in m_loop)
			//{
			//	if (dt < loop.StepRateMS * m_max_loop_steps) continue;
			//	Debug.WriteLine($"SimMessagePump: WARNING - {dt}ms between StepLoops() calls");
			//}

			// Step all loops that are pending
			for (var i = 0; i != m_max_loop_steps; ++i)
			{
				// Sort by soonest to step. Smaller values need to be stepped sooner
				m_order.Sort(NextToRun);
				int NextToRun(int lhs, int rhs) => m_loop[lhs].NextStepTime.CompareTo(m_loop[rhs].NextStepTime);

				// Find the soonest due loop that is ready. Due loops still waiting on their ready handle are skipped so they cannot delay other loops.
				var loop = (Loop?)null;
				var wake_time = long.MaxValue;
				m_blocked.Clear();
				foreach (var index in m_order)
				{
					// Loops are sorted by due time, so the first loop that is not yet due bounds the wait for all remaining loops.
					var candidate = m_loop[index];
					if (candidate.NextStepTime > m_last_step_loops)
					{
						wake_time = Math.Min(wake_time, candidate.NextStepTime);
						break;
					}

					// Step the first due loop that is ready. Otherwise wait for its handle, or for the deadline that lets it step without one.
					if (candidate.IsReady(m_last_step_loops))
					{
						loop = candidate;
						break;
					}
					wake_time = Math.Min(wake_time, candidate.ReadyDeadline);
					m_blocked.Add(candidate);
				}
				if (loop == null)
				{
					// Callback work has already consumed part of this deadline; do not add another full period before pumping.
					return (int)Math.Max(0L, Math.Min(wake_time - m_clock.ElapsedMilliseconds, int.MaxValue));
				}

				// Elapsed time for the loop step, either a fixed value or the wall time since last stepped
				var elapsed_ms = loop.IsVariable ? m_last_step_loops - loop.Clock : loop.StepRateMS;

				// Step the loop. The ready signal is consumed by this step.
				var t0 = m_clock.ElapsedMilliseconds;
				loop.ReadyAcquired = false;
				loop.Step(TimeSpan.FromMilliseconds(elapsed_ms));
				loop.Clock += elapsed_ms;
				var t1 = m_clock.ElapsedMilliseconds - t0;
				loop.AvrStepTime.Add((byte)Math.Min(255L, t1));

				//if (loop.AvrStepTime.Avr() < loop.StepRateMS) continue;
				//Debug.WriteLine($"SimMessagePump: WARNING - long step: {loop.AvrStepTime.Avr() * 100.0 / loop.StepRateMS}%");
			}

			// If we get here, the loops are taking too long. Return a timeout of 0 to indicate
			// loops still need stepping. This allows the message queue still to be processed though.
			// Loop at 'm_avr' to see the last 8 loop execution times in ms.
			//OutputDebugStringA("SimMessagePump: WARNING - loops are staving the message queue\n");
			m_blocked.Clear();
			return 0;
		}
	};
}
