using System;
using System.Runtime.InteropServices;
using EnvDTE;
using Microsoft.VisualStudio.Shell;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>
	/// Reads raw memory from the debuggee process using Win32 ReadProcessMemory.
	/// Used to extract native C++ structs (e.g. collision shapes) from a paused debug session.
	/// </summary>
	public static class DebugMemoryReader
	{
		private const int PROCESS_VM_READ = 0x0010;
		private const int MaxShapeSize = 16 * 1024 * 1024; // 16 MB sanity limit

		[DllImport("kernel32.dll", SetLastError = true)]
		private static extern IntPtr OpenProcess(int dwDesiredAccess, bool bInheritHandle, int dwProcessId);

		[DllImport("kernel32.dll", SetLastError = true)]
		private static extern bool ReadProcessMemory(IntPtr hProcess, IntPtr lpBaseAddress, byte[] lpBuffer, int dwSize, out int lpNumberOfBytesRead);

		[DllImport("kernel32.dll", SetLastError = true)]
		private static extern bool CloseHandle(IntPtr hObject);

		/// <summary>
		/// Read a collision shape's raw bytes from the debuggee process.
		/// Uses a two-pass approach: read the Shape header to get m_size, then read the full buffer.
		/// </summary>
		/// <param name="debugger">The DTE debugger (must be in break mode)</param>
		/// <param name="expression">A native expression that evaluates to a shape lvalue (e.g. "box", "body.m_shape")</param>
		/// <returns>A byte array containing the complete shape data, or null on failure</returns>
		public static byte[]? ReadShapeBytes(Debugger debugger, string expression)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			// Get the address of the shape in the debuggee's memory
			var addr_expr = debugger.GetExpression($"(unsigned long long)(&({expression}))");
			if (!addr_expr.IsValidValue)
				throw new InvalidOperationException($"Cannot evaluate address of '{expression}'. Ensure it is an addressable lvalue in the current frame.");

			if (!ulong.TryParse(addr_expr.Value.Trim(), out var address))
				throw new InvalidOperationException($"Cannot parse address from '{addr_expr.Value}'.");

			// Get the debuggee process ID
			var process = debugger.CurrentProcess;
			if (process == null)
				throw new InvalidOperationException("No debuggee process available.");

			var pid = process.ProcessID;
			var h_process = OpenProcess(PROCESS_VM_READ, false, pid);
			if (h_process == IntPtr.Zero)
				throw new InvalidOperationException($"Cannot open debuggee process (PID {pid}) for reading.");

			try
			{
				// Pass 1: Read the Shape header to determine total size.
				// The Shape header contains m_size which gives the total byte count
				// including any trailing variable-length data (polytopes, arrays).
				var header_size = Marshal.SizeOf<Rylogic.Collision.Shape>();
				var header_buf = new byte[header_size];
				if (!ReadProcessMemory(h_process, (IntPtr)address, header_buf, header_size, out var bytes_read) || bytes_read != header_size)
					throw new InvalidOperationException($"Failed to read Shape header ({header_size} bytes) from address 0x{address:X}.");

				// Parse the header to get m_size and validate
				var header_handle = GCHandle.Alloc(header_buf, GCHandleType.Pinned);
				Rylogic.Collision.Shape header;
				try { header = Marshal.PtrToStructure<Rylogic.Collision.Shape>(header_handle.AddrOfPinnedObject()); }
				finally { header_handle.Free(); }

				// Validate the shape header
				if (header.m_type < Rylogic.Collision.EShape.Sphere || header.m_type > Rylogic.Collision.EShape.Array)
					throw new InvalidOperationException($"Invalid shape type: {(int)header.m_type}. The expression '{expression}' may not be a collision shape.");

				var total_size = header.m_size;
				if (total_size < header_size)
					throw new InvalidOperationException($"Shape m_size ({total_size}) is smaller than the header ({header_size}). Possible layout mismatch.");

				if (total_size > MaxShapeSize)
					throw new InvalidOperationException($"Shape m_size ({total_size}) exceeds safety limit ({MaxShapeSize}). Possible corrupt data.");

				// Pass 2: Read the full shape data
				var data = new byte[total_size];
				if (!ReadProcessMemory(h_process, (IntPtr)address, data, total_size, out bytes_read) || bytes_read != total_size)
					throw new InvalidOperationException($"Failed to read shape data ({total_size} bytes) from address 0x{address:X}. Only {bytes_read} bytes read.");

				return data;
			}
			finally
			{
				CloseHandle(h_process);
			}
		}

		/// <summary>
		/// Read raw bytes from the debuggee process at the given expression's address.
		/// </summary>
		/// <param name="debugger">The DTE debugger (must be in break mode)</param>
		/// <param name="expression">A native expression that evaluates to an addressable lvalue</param>
		/// <param name="size">Number of bytes to read</param>
		public static byte[]? ReadBytes(Debugger debugger, string expression, int size)
		{
			ThreadHelper.ThrowIfNotOnUIThread();

			var addr_expr = debugger.GetExpression($"(unsigned long long)(&({expression}))");
			if (!addr_expr.IsValidValue)
				throw new InvalidOperationException($"Cannot evaluate address of '{expression}'.");

			if (!ulong.TryParse(addr_expr.Value.Trim(), out var address))
				throw new InvalidOperationException($"Cannot parse address from '{addr_expr.Value}'.");

			var process = debugger.CurrentProcess;
			if (process == null)
				throw new InvalidOperationException("No debuggee process available.");

			var pid = process.ProcessID;
			var h_process = OpenProcess(PROCESS_VM_READ, false, pid);
			if (h_process == IntPtr.Zero)
				throw new InvalidOperationException($"Cannot open debuggee process (PID {pid}) for reading.");

			try
			{
				var data = new byte[size];
				if (!ReadProcessMemory(h_process, (IntPtr)address, data, size, out var bytes_read) || bytes_read != size)
					throw new InvalidOperationException($"Failed to read {size} bytes from address 0x{address:X}.");

				return data;
			}
			finally
			{
				CloseHandle(h_process);
			}
		}
	}
}
