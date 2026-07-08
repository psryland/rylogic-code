using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace HantekScope.Device
{
	/// <summary>
	/// Minimal native WinUSB device wrapper using SetupAPI for enumeration and
	/// winusb.dll for bulk transfers. The device must be bound to the WinUSB
	/// driver (e.g. via Zadig). Synchronous I/O with a per-pipe transfer timeout
	/// suits the paced polling the scope expects.
	/// </summary>
	public sealed class WinUsbDevice :IDisposable
	{
		private SafeFileHandle? m_file;
		private IntPtr m_winusb = IntPtr.Zero;
		private byte[] m_pipes = Array.Empty<byte>();
		private uint m_transfer_timeout_ms = 1000;

		/// <summary>True once the device is open and the WinUSB interface is initialised.</summary>
		public bool IsOpen => m_winusb != IntPtr.Zero;

		/// <summary>
		/// Locate and open the first present device matching <paramref name="interface_guid"/>
		/// whose device path also carries the given VID/PID. Sets a transfer timeout
		/// on the supplied pipes so reads/writes never block indefinitely.
		/// </summary>
		public void Open(Guid interface_guid, ushort vid, ushort pid, byte[] pipes, uint timeout_ms)
		{
			var path = FindDevicePath(interface_guid, vid, pid) ??
				throw new InvalidOperationException(
					$"No present WinUSB device matched VID_{vid:X4}&PID_{pid:X4} on interface {interface_guid:B}. Is it bound to WinUSB?");

			// Open the device. WinUSB requires the handle to be opened with
			// FILE_FLAG_OVERLAPPED; transfers are still issued synchronously by passing
			// a NULL OVERLAPPED pointer, in which case WinUSB waits internally and the
			// per-pipe PIPE_TRANSFER_TIMEOUT bounds each transfer.
			m_file = CreateFile(path, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, IntPtr.Zero, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, IntPtr.Zero);
			if (m_file.IsInvalid)
				throw new Win32Exception(Marshal.GetLastWin32Error(), "CreateFile on the WinUSB device failed");

			if (!WinUsb_Initialize(m_file, out m_winusb))
			{
				var err = Marshal.GetLastWin32Error();
				m_file.Dispose();
				m_file = null;
				m_winusb = IntPtr.Zero;
				throw new Win32Exception(err, $"WinUsb_Initialize failed (Win32 error {err})");
			}

			// Bound every transfer on the named pipes so a stalled device cannot hang the
			// app, and let WinUSB automatically clear a halted pipe. Without AUTO_CLEAR_STALL
			// a single endpoint stall latches: the pipe stays halted and every subsequent
			// transfer fails until it is reset. Enabling it makes WinUSB issue the
			// CLEAR_FEATURE(HALT) that the in-box class drivers do for us.
			foreach (var pipe in pipes)
			{
				var timeout = timeout_ms;
				WinUsb_SetPipePolicy(m_winusb, pipe, PIPE_TRANSFER_TIMEOUT, sizeof(uint), ref timeout);

				byte enable = 1;
				WinUsb_SetPipePolicy(m_winusb, pipe, AUTO_CLEAR_STALL, sizeof(byte), ref enable);
			}
			m_pipes = pipes;
			m_transfer_timeout_ms = timeout_ms;

			// Start each session from a clean endpoint state. When a previous session ends
			// abruptly (e.g. the process is killed mid-transfer during development) the device
			// can be left with a half-submitted OUT transfer or an undrained IN response still
			// queued. Aborting cancels any lingering host-side transfers and ResetPipe issues
			// CLEAR_FEATURE(HALT), which also resets the device-side data toggle so host and
			// device agree from the first transfer.
			foreach (var pipe in pipes)
			{
				WinUsb_AbortPipe(m_winusb, pipe);
				WinUsb_ResetPipe(m_winusb, pipe);
			}
		}

		/// <summary>
		/// Prime a bulk IN pipe by reading until it is empty, returning the number of bytes
		/// drained. After a prior session ends mid-exchange the firmware can be left waiting
		/// for the host to complete an IN transaction, and until that read is attempted it
		/// NAKs every OUT write, so the next command times out. Attempting a read completes
		/// that pending transaction (and drains any stale response bytes, though there are
		/// often none), after which OUT writes are serviced again. Uses a short per-read
		/// timeout so an already-empty pipe returns promptly, then restores the caller's
		/// transfer timeout.
		/// </summary>
		public int FlushInput(byte pipe_id, int max_reads = 16)
		{
			if (m_winusb == IntPtr.Zero)
				throw new InvalidOperationException("Device is not open");

			uint short_timeout = 60;
			WinUsb_SetPipePolicy(m_winusb, pipe_id, PIPE_TRANSFER_TIMEOUT, sizeof(uint), ref short_timeout);
			try
			{
				var buffer = new byte[512];
				var total = 0;
				for (var i = 0; i != max_reads; ++i)
				{
					// A failed read here is expected once the pipe is empty (the short timeout
					// expires with nothing to return), so stop rather than treat it as an error.
					if (!WinUsb_ReadPipe(m_winusb, pipe_id, buffer, (uint)buffer.Length, out var transferred, IntPtr.Zero))
						break;
					if (transferred == 0)
						break;
					total += (int)transferred;
				}
				return total;
			}
			finally
			{
				var restore = m_transfer_timeout_ms;
				WinUsb_SetPipePolicy(m_winusb, pipe_id, PIPE_TRANSFER_TIMEOUT, sizeof(uint), ref restore);
			}
		}

		/// <summary>Write a buffer to a bulk OUT pipe. Throws on error; returns bytes transferred.</summary>
		public int Write(byte pipe_id, byte[] data)
		{
			if (m_winusb == IntPtr.Zero)
				throw new InvalidOperationException("Device is not open");

			if (!WinUsb_WritePipe(m_winusb, pipe_id, data, (uint)data.Length, out var transferred, IntPtr.Zero))
			{
				var err = Marshal.GetLastWin32Error();

				// Reset the pipe so a stall/toggle mismatch does not persist into the next
				// transfer, then surface the underlying Win32 code (not just "failed") so the
				// real cause is visible rather than hidden behind a generic message.
				WinUsb_ResetPipe(m_winusb, pipe_id);
				throw new Win32Exception(err, $"WinUsb_WritePipe(0x{pipe_id:X2}) failed: [{err}] {new Win32Exception(err).Message}");
			}

			return (int)transferred;
		}

		/// <summary>
		/// Read up to <paramref name="length"/> bytes from a bulk IN pipe. A timeout
		/// returns an empty array (the device simply had nothing ready); other errors throw.
		/// </summary>
		public byte[] Read(byte pipe_id, int length)
		{
			if (m_winusb == IntPtr.Zero)
				throw new InvalidOperationException("Device is not open");

			var buffer = new byte[length];
			if (!WinUsb_ReadPipe(m_winusb, pipe_id, buffer, (uint)length, out var transferred, IntPtr.Zero))
			{
				var err = Marshal.GetLastWin32Error();

				// A semaphore/IO timeout just means no data arrived in the window.
				if (err == ERROR_SEM_TIMEOUT || err == ERROR_TIMEOUT)
					return Array.Empty<byte>();

				// Any other failure may have halted the pipe; reset it so the stall does not
				// latch into every following read, and surface the real Win32 code.
				WinUsb_ResetPipe(m_winusb, pipe_id);
				throw new Win32Exception(err, $"WinUsb_ReadPipe(0x{pipe_id:X2}) failed: [{err}] {new Win32Exception(err).Message}");
			}

			if ((int)transferred == length)
				return buffer;

			// Short read — return exactly what arrived.
			var result = new byte[transferred];
			Array.Copy(buffer, result, (int)transferred);
			return result;
		}

		/// <summary>Release the WinUSB interface and the device handle.</summary>
		public void Dispose()
		{
			if (m_winusb != IntPtr.Zero)
			{
				// Cancel any in-flight transfers before releasing so the device is not left
				// with a half-completed transfer that would time out the next session's writes.
				foreach (var pipe in m_pipes)
					WinUsb_AbortPipe(m_winusb, pipe);

				WinUsb_Free(m_winusb);
				m_winusb = IntPtr.Zero;
			}
			if (m_file != null)
			{
				m_file.Dispose();
				m_file = null;
			}
		}

		/// <summary>
		/// Enumerate device interfaces for the given class GUID and return the path
		/// of the first present device whose path carries the requested VID/PID.
		/// Matching on the path (rather than trusting the GUID alone) keeps this
		/// robust when several devices share the Zadig-assigned interface GUID.
		/// </summary>
		private static string? FindDevicePath(Guid interface_guid, ushort vid, ushort pid)
		{
			var dev_info = SetupDiGetClassDevs(ref interface_guid, IntPtr.Zero, IntPtr.Zero, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
			if (dev_info == INVALID_HANDLE_VALUE)
				throw new Win32Exception(Marshal.GetLastWin32Error(), "SetupDiGetClassDevs failed");

			try
			{
				var vid_pid = $"vid_{vid:x4}&pid_{pid:x4}";

				var iface = new SP_DEVICE_INTERFACE_DATA();
				iface.cbSize = Marshal.SizeOf<SP_DEVICE_INTERFACE_DATA>();

				for (uint index = 0; SetupDiEnumDeviceInterfaces(dev_info, IntPtr.Zero, ref interface_guid, index, ref iface); ++index)
				{
					var path = GetInterfacePath(dev_info, ref iface);
					if (path != null && path.ToLowerInvariant().Contains(vid_pid))
						return path;
				}
				return null;
			}
			finally
			{
				SetupDiDestroyDeviceInfoList(dev_info);
			}
		}

		/// <summary>Resolve the device path string for one enumerated interface (two-call pattern).</summary>
		private static string? GetInterfacePath(IntPtr dev_info, ref SP_DEVICE_INTERFACE_DATA iface)
		{
			// First call discovers the required buffer size.
			SetupDiGetDeviceInterfaceDetail(dev_info, ref iface, IntPtr.Zero, 0, out var required, IntPtr.Zero);
			if (required == 0)
				return null;

			var buffer = Marshal.AllocHGlobal((int)required);
			try
			{
				// The detail struct is { DWORD cbSize; WCHAR DevicePath[1]; }. The cbSize
				// field must be the fixed-part size, which is 8 on 64-bit and 6 on 32-bit
				// due to packing — a long-standing SetupAPI quirk. The path itself always
				// starts at offset 4 (just past the DWORD).
				Marshal.WriteInt32(buffer, IntPtr.Size == 8 ? 8 : 6);

				if (!SetupDiGetDeviceInterfaceDetail(dev_info, ref iface, buffer, required, out _, IntPtr.Zero))
					return null;

				return Marshal.PtrToStringUni(buffer + 4);
			}
			finally
			{
				Marshal.FreeHGlobal(buffer);
			}
		}

		#region Native interop

		private const uint DIGCF_PRESENT = 0x02;
		private const uint DIGCF_DEVICEINTERFACE = 0x10;
		private const uint GENERIC_READ = 0x80000000;
		private const uint GENERIC_WRITE = 0x40000000;
		private const uint FILE_SHARE_READ = 0x01;
		private const uint FILE_SHARE_WRITE = 0x02;
		private const uint OPEN_EXISTING = 0x03;
		private const uint FILE_ATTRIBUTE_NORMAL = 0x80;
		private const uint FILE_FLAG_OVERLAPPED = 0x40000000;
		private const uint PIPE_TRANSFER_TIMEOUT = 0x03;
		private const uint AUTO_CLEAR_STALL = 0x02;
		private const int ERROR_SEM_TIMEOUT = 121;
		private const int ERROR_TIMEOUT = 1460;
		private static readonly IntPtr INVALID_HANDLE_VALUE = new(-1);

		[StructLayout(LayoutKind.Sequential)]
		private struct SP_DEVICE_INTERFACE_DATA
		{
			public int cbSize;
			public Guid InterfaceClassGuid;
			public int Flags;
			public IntPtr Reserved;
		}

		[DllImport("setupapi.dll", SetLastError = true)]
		private static extern IntPtr SetupDiGetClassDevs(ref Guid class_guid, IntPtr enumerator, IntPtr hwnd_parent, uint flags);

		[DllImport("setupapi.dll", SetLastError = true)]
		private static extern bool SetupDiDestroyDeviceInfoList(IntPtr dev_info);

		[DllImport("setupapi.dll", SetLastError = true)]
		private static extern bool SetupDiEnumDeviceInterfaces(IntPtr dev_info, IntPtr dev_info_data, ref Guid interface_class_guid, uint member_index, ref SP_DEVICE_INTERFACE_DATA dev_iface_data);

		[DllImport("setupapi.dll", SetLastError = true, CharSet = CharSet.Unicode)]
		private static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr dev_info, ref SP_DEVICE_INTERFACE_DATA dev_iface_data, IntPtr detail, uint detail_size, out uint required_size, IntPtr dev_info_data);

		[DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
		private static extern SafeFileHandle CreateFile(string file_name, uint access, uint share, IntPtr security, uint disposition, uint flags, IntPtr template);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_Initialize(SafeFileHandle device_handle, out IntPtr interface_handle);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_Free(IntPtr interface_handle);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_WritePipe(IntPtr interface_handle, byte pipe_id, byte[] buffer, uint length, out uint transferred, IntPtr overlapped);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_ReadPipe(IntPtr interface_handle, byte pipe_id, byte[] buffer, uint length, out uint transferred, IntPtr overlapped);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_SetPipePolicy(IntPtr interface_handle, byte pipe_id, uint policy_type, uint value_length, ref uint value);

		[DllImport("winusb.dll", SetLastError = true, EntryPoint = "WinUsb_SetPipePolicy")]
		private static extern bool WinUsb_SetPipePolicy(IntPtr interface_handle, byte pipe_id, uint policy_type, uint value_length, ref byte value);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_ResetPipe(IntPtr interface_handle, byte pipe_id);

		[DllImport("winusb.dll", SetLastError = true)]
		private static extern bool WinUsb_AbortPipe(IntPtr interface_handle, byte pipe_id);

		#endregion
	}
}
