using System;
using System.Runtime.InteropServices;
using System.Threading;

namespace Rylogic.D3D12;

/// <summary>Owns one COM reference to a Direct3D 12 interface. Derived types name the interface.</summary>
public abstract class ComLease : IDisposable
{
	private SafeComHandle? m_handle;

	/// <summary>Adopt an interface pointer whose COM reference is already owned by the caller.</summary>
	private protected ComLease(IntPtr owned_reference)
	{
		// A lease always refers to a live interface.
		if (owned_reference == IntPtr.Zero)
			throw new ArgumentException("A COM lease requires a valid interface reference.", nameof(owned_reference));

		m_handle = new SafeComHandle(owned_reference);
	}

	/// <summary>True after this lease has released its reference.</summary>
	public bool IsDisposed
	{
		get
		{
			var handle = Volatile.Read(ref m_handle);
			return handle == null || handle.IsClosed;
		}
	}

	/// <summary>Release this lease's reference.</summary>
	public void Dispose()
	{
		Interlocked.Exchange(ref m_handle, null)?.Dispose();
		GC.SuppressFinalize(this);
	}

	/// <summary>Pin the SafeHandle while a friend assembly passes the interface to native code.</summary>
	internal Borrowed Borrow()
	{
		var handle = Volatile.Read(ref m_handle) ?? throw new ObjectDisposedException(GetType().Name);
		return new Borrowed(handle);
	}

	/// <summary>Take an additional COM reference, for a derived type's Clone.</summary>
	private protected IntPtr AddRef()
	{
		using var borrowed = Borrow();

		// The new reference outlives the temporary SafeHandle pin.
		Marshal.AddRef(borrowed.Handle);
		return borrowed.Handle;
	}

	/// <summary>Keeps the leased COM reference alive while native code borrows its pointer.</summary>
	internal sealed class Borrowed : IDisposable
	{
		private SafeComHandle? m_owner;

		/// <summary>Pin the owning SafeHandle and expose its pointer only to friend assemblies.</summary>
		internal Borrowed(SafeComHandle owner)
		{
			// Undo a successful pin if reading the handle fails.
			var add_ref = false;
			try
			{
				owner.DangerousAddRef(ref add_ref);
				if (!add_ref)
					throw new ObjectDisposedException(nameof(ComLease));

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

		/// <summary>The pinned interface pointer.</summary>
		internal IntPtr Handle { get; }

		/// <summary>Release the temporary SafeHandle pin.</summary>
		public void Dispose()
		{
			Interlocked.Exchange(ref m_owner, null)?.DangerousRelease();
		}
	}

	/// <summary>Releases the adopted COM reference.</summary>
	internal sealed class SafeComHandle : SafeHandle
	{
		/// <summary>Adopt an existing COM reference.</summary>
		internal SafeComHandle(IntPtr owned_reference)
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

/// <summary>Owns one COM reference to an ID3D12Device interface.</summary>
public sealed class DeviceLease : ComLease
{
	/// <summary>Adopt an ID3D12Device pointer whose COM reference is already owned by the caller.</summary>
	internal DeviceLease(IntPtr owned_reference)
		: base(owned_reference)
	{}

	/// <summary>Create an independent lease to the same device.</summary>
	public DeviceLease Clone()
	{
		return new DeviceLease(AddRef());
	}
}

/// <summary>Owns one COM reference to an ID3D12Fence interface.</summary>
public sealed class FenceLease : ComLease
{
	/// <summary>Adopt an ID3D12Fence pointer whose COM reference is already owned by the caller.</summary>
	internal FenceLease(IntPtr owned_reference)
		: base(owned_reference)
	{}

	/// <summary>Create an independent lease to the same fence.</summary>
	public FenceLease Clone()
	{
		return new FenceLease(AddRef());
	}
}

/// <summary>Owns one COM reference to an ID3D12Resource interface.</summary>
public sealed class ResourceLease : ComLease
{
	/// <summary>Adopt an ID3D12Resource pointer whose COM reference is already owned by the caller.</summary>
	internal ResourceLease(IntPtr owned_reference)
		: base(owned_reference)
	{}

	/// <summary>Create an independent lease to the same resource.</summary>
	public ResourceLease Clone()
	{
		return new ResourceLease(AddRef());
	}
}
