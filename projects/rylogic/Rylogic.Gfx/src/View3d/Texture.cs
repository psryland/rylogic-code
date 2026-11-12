//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.InteropServices;
using Rylogic.Common;
using Rylogic.Utility;
using HTexture = System.IntPtr;
using HWND = System.IntPtr;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		/// <summary>Texture resource wrapper</summary>
		public sealed class Texture :IDisposable
		{
			private readonly bool m_owned;

			/// <summary>Create a texture from an existing texture resource</summary>
			internal Texture(HTexture handle, bool owned)
			{
				// Note: 'handle' is not the ID3D11Texture2D handle, it's the internal View3DTexture pointer.
				m_owned = owned;
				Handle = handle;

				if (Handle != IntPtr.Zero)
					Info = View3D_TextureGetInfo(Handle);
				else
					Info = new ImageInfo();
			}

			/// <summary>Construct an uninitialised texture</summary>
			public Texture(int width, int height)
				:this(width, height, IntPtr.Zero, 0, new TextureOptions())
			{}
			public Texture(int width, int height, TextureOptions options)
				:this(width, height, IntPtr.Zero, 0, options)
			{}
			public Texture(int width, int height, IntPtr data, int data_size, TextureOptions options)
			{
				m_owned = true;
				Handle = View3D_TextureCreate(width, height, data, data_size, ref options);
				if (Handle == HTexture.Zero)
					throw new Exception($"Failed to create {width}x{height} texture");
				
				Info = View3D_TextureGetInfo(Handle);
			}

			/// <summary>Construct a texture from a resource (file, embedded resource, or stock asset)</summary>
			public Texture(string resource)
				:this(resource, 0, 0, new TextureOptions())
			{}
			public Texture(string resource, TextureOptions options)
				:this(resource, 0, 0, options)
			{}
			public Texture(string resource, int width, int height)
				:this(resource, width, height, new TextureOptions())
			{}
			public Texture(string resource, int width, int height, TextureOptions options)
			{
				m_owned = true;
				Handle = View3D_TextureCreateFromUri(resource, width, height, ref options);
				if (Handle == HTexture.Zero)
					throw new Exception($"Failed to create texture from {resource}");

				Info = View3D_TextureGetInfo(Handle);
			}

			/// <inheritdoc/>
			public void Dispose()
			{
				Util.BreakIf(Util.IsGCFinalizerThread, "Disposing in the GC finalizer thread");
				if (Handle == HTexture.Zero) return;
				if (m_owned) View3D_TextureRelease(Handle);
				Handle = HTexture.Zero;
			}

			/// <summary>View3d texture handle</summary>
			public HTexture Handle;

			/// <summary>Texture format information</summary>
			public ImageInfo Info;

			/// <summary>Get/Set the texture size. Set does not preserve the texture content</summary>
			public Size Size
			{
				get => new((int)Info.Width, (int)Info.Height);
				set
				{
					if (Size == value) return;
					Resize(value.Width, value.Height);
				}
			}

			/// <summary>User Data</summary>
			public object? Tag { get; set; }

			/// <summary>The current ref count of this texture</summary>
			private ulong RefCount => View3D_TextureRefCount(Handle);

			/// <summary>Resize the texture</summary>
			public void Resize(int width, int height)
			{
				View3D_TextureResize(Handle, (ulong)width, (uint)height, 0);
				Info = View3D_TextureGetInfo(Handle);
			}

#if false //todo
			/// <summary>Fill a surface of this texture from a file</summary>
			public void LoadSurface(string tex_filepath, int level)
			{
				View3D_TextureLoadSurface(Handle, level, tex_filepath, null, null, EFilter.D3D11_FILTER_MIN_MAG_MIP_LINEAR, 0);
				Info = View3D_TextureGetInfo(Handle);
			}

			/// <summary>Fill a surface of this texture from a file</summary>
			public void LoadSurface(string tex_filepath, int level, EFilter filter, uint colour_key)
			{
				View3D_TextureLoadSurface(Handle, level, tex_filepath, null, null, filter, colour_key);
				Info = View3D_TextureGetInfo(Handle);
			}

			/// <summary>Fill a surface of this texture from a file</summary>
			public void LoadSurface(string tex_filepath, int level, Rectangle src_rect, Rectangle dst_rect, EFilter filter, uint colour_key)
			{
				View3D_TextureLoadSurface(Handle, level, tex_filepath, new []{dst_rect}, new []{src_rect}, filter, colour_key);
				Info = View3D_TextureGetInfo(Handle);
			}
#endif

			/// <summary>Get/Set the private data of this texture by unique Id</summary>
			public PrivateDataProxy PrivateData => new(this);
			public PrivateDataPointerProxy PrivateDataPointer => new(this);

			/// <summary>Create a texture instance from a stock texture</summary>
			public static Texture FromStock(EStockTexture tex)
			{
				// Stock resources are owned by the renderer, so the handle is borrowed
				return new Texture(View3D_TextureCreateStock(tex), owned: false);
			}

			/// <summary>Return properties of the texture</summary>
			public static ImageInfo GetInfo(string tex_filepath)
			{
				var res = View3D_TextureGetInfoFromFile(tex_filepath, out var info);
				if (res != EResult.Success) throw new Exception(res);
				return info;
			}

			/// <summary>Copy a multi-sampled resource into a non-multi-sampled resource.</summary>
			public static void ResolveAA(Texture dst, Texture src)
			{
				View3D_TextureResolveAA(dst.Handle, src.Handle);
			}

			/// <summary>Create a Texture instance from a shared d3d resource (created on a different d3d device)</summary>
			public static Texture FromShared(IntPtr shared_resource, TextureOptions options)
			{
				throw new NotImplementedException();
#if false // todo
				// Not all of the texture options are used, just sampler description, has alpha, and dbg name
				return new Texture(View3D_TextureFromShared(shared_resource, ref options.Data), owned:true);
#endif
			}

			/// <summary>Create a render target texture based on a shared Dx9 texture</summary>
			public static Texture Dx9RenderTarget(HWND hwnd, int width, int height, TextureOptions options, out IntPtr shared_handle)
			{
				// Not all of the texture options are used, just format, sampler description, has alpha, and dbg name
				if (hwnd == IntPtr.Zero)
					throw new Exception("DirectX 9 requires a window handle");

				// Try to create the texture. This can fail if the window handle is 'ready'
				var handle = View3D_CreateDx9RenderTarget(hwnd, (uint)width, (uint)height, ref options, out shared_handle);
				if (handle == IntPtr.Zero)
					throw new Exception("Failed to create DirectX 9 render target texture");

				return new Texture(handle, owned: true);
			}
			public static Texture Dx9RenderTarget(HWND hwnd, int width, int height, TextureOptions options)
			{
				return Dx9RenderTarget(hwnd, width, height, options, out var _);
			}

			/// <summary>Lock the texture for drawing on</summary>
			[DebuggerHidden]
			public Lock LockSurface(bool discard)
			{
				// This is a method to prevent the debugger evaluating it cause multiple GetDC calls
				return new Lock(this, discard);
			}

			#region Helper Classes

			/// <summary>An RAII object used to lock the texture for drawing with GDI+ methods</summary>
			public sealed class Lock :IDisposable
			{
#if false //todo
				private readonly HTexture m_tex;
#endif
				/// <summary>
				/// Lock 'tex' making 'Gfx' available.
				/// Note: if 'tex' is the render target of a window, you need to call Window.RestoreRT when finished</summary>
				public Lock(Texture tex, bool discard)
				{
					throw new NotImplementedException();
#if false //todo
					m_tex = tex.Handle;
					var dc = View3D_TextureGetDC(m_tex, discard);
					if (dc == IntPtr.Zero) throw new Exception("Failed to get Texture DC. Check the texture is a GdiCompatible texture");
					Gfx = Graphics.FromHdc(dc);
					Size = tex.Size;
#endif
				}
				public void Dispose()
				{
					throw new NotImplementedException();
#if false //todo
					View3D_TextureReleaseDC(m_tex);
#endif
				}

				/// <summary>GDI+ graphics interface</summary>
				public Graphics Gfx { get; }

				/// <summary>The size of the texture surface</summary>
				public Size Size { get; }
			}

			/// <summary>Proxy object for accessing the private data of a texture</summary>
			public class PrivateDataProxy
			{
				private readonly HTexture m_handle;
				internal PrivateDataProxy(Texture tex)
				{
					m_handle = tex.Handle;
				}
				public byte[] this[Guid id]
				{
					// 'size' should be the size of the data pointed to by 'data'
					get
					{
						// Request the size of the available private data
						var size = 0U;
						View3d_TexturePrivateDataGet(m_handle, id, ref size, IntPtr.Zero);
						if (size == 0)
							return Array.Empty<byte>();

						// Create a buffer and read the private data
						var buffer = new byte[size];
						using var buf = Marshal_.Pin(buffer, GCHandleType.Pinned);
						View3d_TexturePrivateDataGet(m_handle, id, ref size, buf.Pointer);
						return buffer;
					}
					set
					{
						using var buf = Marshal_.Pin(value, GCHandleType.Pinned);
						View3d_TexturePrivateDataSet(m_handle, id, (uint)(value?.Length ?? 0), buf.Pointer);
					}
				}
			}
			public class PrivateDataPointerProxy
			{
				private readonly HTexture m_handle;
				internal PrivateDataPointerProxy(Texture tex)
				{
					m_handle = tex.Handle;
				}
				public IntPtr this[Guid id]
				{
					// 'size' should be the size of the data pointed to by 'data'
					get
					{
						// Request the size of the available private data
						var size = 0U;
						View3d_TexturePrivateDataGet(m_handle, id, ref size, IntPtr.Zero);
						if (size == 0)
							return IntPtr.Zero;

						// Create a buffer and read the private data
						var buffer = new byte[size];
						using var buf = Marshal_.Pin(buffer, GCHandleType.Pinned);
						View3d_TexturePrivateDataGet(m_handle, id, ref size, buf.Pointer);

						if (size == 8)
							return new IntPtr(BitConverter.ToInt64(buffer, 0));
						if (size == 4)
							return new IntPtr(BitConverter.ToInt32(buffer, 0));

						throw new Exception("Private data is not a pointer type");
					}
					set
					{
						View3d_TexturePrivateDataIFSet(m_handle, id, value);
					}
				}
			}

			/// <summary>Labels for private data GUIDs</summary>
			public struct UserData
			{
				/// <summary>Unique IDs for types of private data attached to textures</summary>
				public static readonly Guid Surface0Pointer = new("6EE0154E-DEAD-4E2F-869B-E4D15CA29787");
			}

#endregion

			/// <summary>Implicit conversion to handle</summary>
			public static implicit operator HTexture(Texture tex)
			{
				return tex.Handle;
			}

			#region Equals
			public static bool operator == (Texture? lhs, Texture? rhs)
			{
				return ReferenceEquals(lhs,rhs) || Equals(lhs, rhs);
			}
			public static bool operator != (Texture? lhs, Texture? rhs)
			{
				return !(lhs == rhs);
			}
			public bool Equals(Texture? rhs)
			{
				return rhs != null && Handle == rhs.Handle;
			}
			public override bool Equals(object? rhs)
			{
				return Equals(rhs as Texture);
			}
			public override int GetHashCode()
			{
				return Handle.GetHashCode();
			}
			#endregion
		}
	}
}
