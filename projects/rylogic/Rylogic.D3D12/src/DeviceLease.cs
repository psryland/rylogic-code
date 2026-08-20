using System;
using System.Runtime.InteropServices;
using System.Threading;

namespace Rylogic.D3D12;

/// <summary>Owns one COM reference to an ID3D12Device interface.</summary>
public sealed class DeviceLease : IDisposable
{
	private SafeDeviceHandle? m_handle;

	/// <summary>Adopt an ID3D12Device pointer whose COM reference is already owned by the caller.</summary>
	internal DeviceLease(IntPtr owned_reference)
	{
		if (owned_reference == IntPtr.Zero)
			throw new ArgumentException("A device lease requires a valid ID3D12Device reference.", nameof(owned_reference));

		m_handle = new SafeDeviceHandle(owned_reference);
	}

	/// <summary>True after this lease has released its device reference.</summary>
	public bool IsDisposed
	{
		get
		{
			var handle = Volatile.Read(ref m_handle);
			return handle == null || handle.IsClosed;
		}
	}

	/// <summary>Create an independent lease to the same device.</summary>
	public DeviceLease Clone()
	{
		using var borrowed = Borrow();

		// The cloned lease owns a distinct COM reference after the temporary SafeHandle pin is released.
		Marshal.AddRef(borrowed.Handle);
		return new DeviceLease(borrowed.Handle);
	}

	/// <summary>Release this lease's device reference.</summary>
	public void Dispose()
	{
		Interlocked.Exchange(ref m_handle, null)?.Dispose();
		GC.SuppressFinalize(this);
	}

	/// <summary>Pin the SafeHandle while a friend assembly passes the device to native code.</summary>
	internal BorrowedDevice Borrow()
	{
		var handle = Volatile.Read(ref m_handle) ?? throw new ObjectDisposedException(nameof(DeviceLease));
		return new BorrowedDevice(handle);
	}

	/// <summary>Keeps the leased COM reference alive while native code borrows its pointer.</summary>
	internal sealed class BorrowedDevice : IDisposable
	{
		private SafeDeviceHandle? m_owner;

		/// <summary>Pin the owning SafeHandle and expose its pointer only to friend assemblies.</summary>
		internal BorrowedDevice(SafeDeviceHandle owner)
		{
			var add_ref = false;
			try
			{
				owner.DangerousAddRef(ref add_ref);
				if (!add_ref)
					throw new ObjectDisposedException(nameof(DeviceLease));

				m_owner = owner;
				Handle = owner.DangerousGetHandle();
			}
			catch
			{
				if (add_ref)
					owner.DangerousRelease();

				throw;
			}
		}

		/// <summary>The pinned ID3D12Device pointer.</summary>
		internal IntPtr Handle { get; }

		/// <summary>Release the temporary SafeHandle pin.</summary>
		public void Dispose()
		{
			Interlocked.Exchange(ref m_owner, null)?.DangerousRelease();
		}
	}

	/// <summary>Releases the adopted ID3D12Device COM reference.</summary>
	internal sealed class SafeDeviceHandle : SafeHandle
	{
		/// <summary>Adopt an existing COM reference.</summary>
		internal SafeDeviceHandle(IntPtr owned_reference)
			: base(IntPtr.Zero, true)
		{
			SetHandle(owned_reference);
		}

		/// <inheritdoc/>
		public override bool IsInvalid
		{
			get
			{
				return handle == IntPtr.Zero;
			}
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			Marshal.Release(handle);
			handle = IntPtr.Zero;
			return true;
		}
	}
}
