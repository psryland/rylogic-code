using System;
using System.Diagnostics;
using Rylogic.Interop.Win32;

namespace Rylogic.Common
{
	// Common event arg types

	/// <summary>Event containing a message string</summary>
	public class MessageEventArgs :EventArgs
	{
		public MessageEventArgs(string msg)
		{
			Message = msg;
		}

		/// <summary>The message</summary>
		public string Message { get; }
	}

	/// <summary>Event containing a single value</summary>
	public class ValueEventArgs :EventArgs
	{
		public ValueEventArgs(object value)
		{
			Value = value;
		}

		/// <summary>The value</summary>
		public object Value { get; }
	}

	/// <summary>Event containing a single value</summary>
	public class ValueEventArgs<T> :EventArgs
	{
		public ValueEventArgs(T value)
		{
			Value = value;
		}

		/// <summary>The value</summary>
		public T Value { get; }
	}

	/// <summary>Event signalled before and then after an event</summary>
	public class PrePostEventArgs :EventArgs
	{
		public PrePostEventArgs(bool after)
		{
			After = after;
		}

		/// <summary>True if this event is logically "before" something is about to happen</summary>
		public bool Before => !After;

		/// <summary>True if this event is logically "after" something has happened</summary>
		public bool After { get; }
	}

	/// <summary>Event signalling a changed value, providing both the old and new values</summary>
	public class ValueChangedEventArgs<T> : EventArgs
	{
		public ValueChangedEventArgs(T nue, T old)
		{
			New = nue;
			Old = old;
		}

		public T New { get; }
		public T Old { get; }
	}

	/// <summary>Event args for message handlers</summary>
	[DebuggerDisplay("Description,nq")]
	public class WndProcEventArgs :EventArgs
	{
		public WndProcEventArgs(IntPtr hwnd, int message, IntPtr wparam, IntPtr lparam)
		{
			Hwnd = hwnd;
			Message = message;
			WParam = wparam;
			LParam = lparam;
			Handled = false;
			Result = IntPtr.Zero;
		}

		/// <summary>Window handle</summary>
		public IntPtr Hwnd { get; }

		/// <summary>Window message</summary>
		public int Message { get; }

		/// <summary>WParam</summary>
		public IntPtr WParam { get; }

		/// <summary>LParam</summary>
		public IntPtr LParam { get; }

		/// <summary>Message handled (prevents passing to DefWindowProc)</summary>
		public bool Handled { get; set; }

		/// <summary>The result returned to the message sender when <see cref="Handled"/> is true.</summary>
		public IntPtr Result { get; set; }

		/// <summary></summary>
		public string Description => Win32.MsgIdToString(Message);
	}
}
