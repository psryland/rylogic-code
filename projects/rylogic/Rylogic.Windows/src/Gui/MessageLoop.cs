using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
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
			m_filters.Add(filter);
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
		public delegate void StepFunc(TimeSpan elapsed);

		/// <summary>A loop represents a process that should be run at a given frame rate</summary>
		public class Loop(StepFunc step, int step_rate_ms, bool variable)
		{
			public StepFunc Step = step;                    // The function to step in the loop
			public long Clock = 0;                          // The time this loop was last stepped (in ms)
			public int StepRateMS = step_rate_ms;           // (Minimum) step rate
			public bool IsVariable = variable;              // Variable step rate
			public Buf8 AvrStepTime = new();                // Last 8 execution times of the loop (in ms, capped at 255)
			public long NextStepTime => Clock + StepRateMS; // The next time to run this loop
			public struct Buf8
			{
				public ulong u64;
				public void Add(byte v) => u64 = (u64 << 8 & ~0xFFUL) | v;
				public readonly double Avr() => BitConverter.GetBytes(u64).Average(x => (double)x);
			}
		}

		public class LoopCont : List<Loop> { };
		public class LoopOrder : List<int> { };

		private readonly int m_max_loop_steps = max_loop_steps; // The maximum number of loops to step before checking for messages
		private readonly Stopwatch m_clock = new();             // High performance clock
		private readonly LoopCont m_loop = [];                  // The loops to execute
		private readonly LoopOrder m_order = [];                // A priority queue of loops. The loop at position 0 is the next to be stepped
		private long m_last_step_loops = 0;                     // The last time StepLoops was called.

		public SimMessageLoop()
			:this(10)
		{
		}

		/// <summary>Add a loop to be stepped by this simulation message pump. if 'variable' is true, 'step_rate_ms' means minimum step rate</summary>
		public void AddLoop(float frame_rate, bool variable, StepFunc step)
		{
			m_loop.Add(new Loop(step, (int)(1000f / frame_rate), variable));
			m_order.Add(m_loop.Count - 1);
		}

		/// <summary>Run the thread message pump while maintaining the desired loop rates</summary>
		public override int Run()
		{
			// Set the start time
			m_clock.Start();
			m_last_step_loops = 0;

			// Run the message pump loop
			for (; ; )
			{
				// Step any pending loops and get the time till the next loop to be stepped.
				var timeout = StepLoops();

				// Check for messages and pump any received until
				var wait_result = User32.MsgWaitForMultipleObjects(0, null, true, timeout, Win32.QS_ALLPOSTMESSAGE | Win32.QS_ALLINPUT | Win32.QS_ALLEVENTS);
				if (wait_result == -1)
					throw new Win32Exception("MsgWaitForMultipleObjects failed");
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

		// Call 'Step' on all loops that are pending
		// Returns the time in milliseconds until the next loop needs to be stepped
		public int StepLoops()
		{
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

				// Get the next due to be stepped
				var loop = m_loop[m_order[0]];
				var time_till_step = (int)(loop.NextStepTime - m_last_step_loops);
				if (time_till_step > 0)
					return time_till_step;

				// Elapsed time for the loop step, either a fixed value or the wall time since last stepped
				var elapsed_ms = loop.IsVariable ? m_last_step_loops - loop.Clock : loop.StepRateMS;

				// Step the loop
				var t0 = m_clock.ElapsedMilliseconds;
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
			return 0;
		}
	};
}