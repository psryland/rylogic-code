//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using Rylogic.Utility;
using HSampler = System.IntPtr;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		/// <summary>Texture resource wrapper</summary>
		public sealed class Sampler :IDisposable
		{
			// Notes:
			// - Samplers aren't mutable, just create a new one

			private readonly bool m_owned;

			/// <summary>Create a texture from an existing texture resource</summary>
			internal Sampler(HSampler handle, bool owned)
			{
				// Note: 'handle' is not the ID3D11Texture2D handle, it's the internal View3DTexture pointer.
				m_owned = owned;
				Handle = handle;
			}

			/// <summary>Construct an uninitialised texture</summary>
			public Sampler()
				: this(new SamplerOptions())
			{}
			public Sampler(SamplerOptions options)
			{
				m_owned = true;
				Handle = View3D_SamplerCreate(ref options);
				if (Handle == HSampler.Zero)
					throw new Exception($"Failed to create sampler");
				
				//View3D_TextureSetFilterAndAddrMode(Handle, options.Filter, options.AddrU, options.AddrV);
			}
			
			/// <inheritdoc/>
			public void Dispose()
			{
				Util.BreakIf(Util.IsGCFinalizerThread, "Disposing in the GC finalizer thread");
				if (Handle == HSampler.Zero) return;
				if (m_owned) View3D_SamplerRelease(Handle);
				Handle = HSampler.Zero;
			}

			/// <summary>View3d sampler handle</summary>
			public HSampler Handle;

			/// <summary>Create a sampler instance from a stock sampler</summary>
			public static Sampler FromStock(EStockSampler sam)
			{
				// Stock resources are owned by the renderer, so the handle is borrowed
				return new Sampler(View3D_SamplerCreateStock(sam), owned: false);
			}

			/// <summary>Implicit conversion to handle</summary>
			public static implicit operator HSampler(Sampler sam)
			{
				return sam.Handle;
			}

			#region Equals
			public static bool operator == (Sampler? lhs, Sampler? rhs)
			{
				return ReferenceEquals(lhs,rhs) || Equals(lhs, rhs);
			}
			public static bool operator != (Sampler? lhs, Sampler? rhs)
			{
				return !(lhs == rhs);
			}
			public bool Equals(Sampler? rhs)
			{
				return rhs != null && Handle == rhs.Handle;
			}
			public override bool Equals(object? rhs)
			{
				return Equals(rhs as Sampler);
			}
			public override int GetHashCode()
			{
				return Handle.GetHashCode();
			}
			#endregion
		}
	}
}
