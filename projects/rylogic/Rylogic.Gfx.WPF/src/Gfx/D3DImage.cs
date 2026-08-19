using System;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Windows;
using System.Windows.Interop;
using System.Windows.Media.Imaging;
using Rylogic.Common;
using Rylogic.Utility;
using Size = System.Drawing.Size;

namespace Rylogic.Gfx
{
	public class D3DImage : System.Windows.Interop.D3DImage, IDisposable
	{
		// Notes:
		//  - This is a re-imaged implementation of Microsoft.Wpf.DirectX.Interop.D3D11Image.
		//  - D3DImage does not own the back buffer it uses. It expects the caller to provide the
		//    back buffer and manage its lifetime. This class *does* manage the back buffer because
		//    it needs to be created as a shared Dx9 render target which is then opened as a Dx11
		//    texture.
		//  - Dx9 cannot be used without a window handle.
		//  - PixelWidth/PixelHeight are the pixel dimensions of the render target. These sizes are
		//    cached in this object so that they can be set before the render target is created.
		//  - The FrontBuffer is a texture with the same dimensions as the target image size.
		//    'RenderTarget' is a texture that may be larger so that anti aliasing can be supported.
		//
		//  Usage:
		//   Assign an instance of this object as the Source of a WPF Image control.
		//   Attach a handler to RenderTargetChanged
		//      Assign the render target to your View3d.Window using SetRT and SaveAsMainRT
		//      This window will be an off-screen window.
		//   Assign the window handle as soon as it's available.
		//      In a window class, use OnSourceInitialized.
		//          d3d_image.WindowOwner = new WindowInteropHelper(this).Handle;
		//      In a custom control, use Loaded/Unloaded with:
		//          var win = System.Windows.Window.GetWindow(this);
		//          d3d_image.WindowOwner = new WindowInteropHelper(win).Handle;
		//  When the Image changes size, use:
		//     var size = new Size(img.DesiredSize.Width, img.DesiredSize.Height).TransformToDevice(img);
		//     var width  = (int)Math.Max(1, Math.Round(size.Width));
		//     var height = (int)Math.Max(1, Math.Round(size.Height));
		//     d3d_image.SetRenderTargetSize(new(width, height));
		private const View3d.EFormat RenderTargetFormat = View3d.EFormat.DXGI_FORMAT_B8G8R8A8_UNORM;
		private Size m_dim_pixels;

		~D3DImage()
		{
			Dispose(false);
		}
		public D3DImage()
			: this(IntPtr.Zero, new(16, 16), new(1f, 1f))
		{ }
		public D3DImage(IntPtr hwnd, Size dim, PointF dpi_scale, int? multi_sampling = null)
			: base(dpi_scale.X * 96.0, dpi_scale.Y * 96.0)
		{
			m_dim_pixels = dim;
			m_multi_sampling.Count = multi_sampling ?? 4;
			m_multi_sampling.Quality = View3d.MultiSamp.BestQuality(m_multi_sampling.Count, RenderTargetFormat);
			WindowOwner = hwnd; // Can be null
			TryCreateRenderTarget(m_dim_pixels);
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool _)
		{
			FrontBuffer = null;
			WindowOwner = IntPtr.Zero;
		}
		protected override Freezable CreateInstanceCore()
		{
			return new D3DImage();
		}

		/// <summary>The window handle (HWND) of the window that hosts the D3DImage</summary>
		public IntPtr WindowOwner
		{
			get => (IntPtr)GetValue(WindowOwnerProperty);
			set => SetValue(WindowOwnerProperty, value);
		}
		private static void WindowOwnerChanged(DependencyObject sender, DependencyPropertyChangedEventArgs args)
		{
			var image = (D3DImage)sender;
			if (args.OldValue is IntPtr old_hwnd && old_hwnd != IntPtr.Zero)
			{
				// The window handle is changing. Tear-down Dx9
				image.FrontBuffer = null;
			}
			if (args.NewValue is IntPtr new_hwnd && new_hwnd != IntPtr.Zero)
			{
				// A window handle has been assigned. Setup the Dx9 render target
				image.TryCreateRenderTarget(image.m_dim_pixels);
			}
		}
		public static DependencyProperty WindowOwnerProperty = DependencyProperty.Register(nameof(WindowOwner), typeof(IntPtr), typeof(D3DImage), new UIPropertyMetadata(IntPtr.Zero, new PropertyChangedCallback(WindowOwnerChanged)));

		/// <summary>The render target multi-sampling</summary>
		public int MultiSampling
		{
			get => m_multi_sampling.Count;
			set
			{
				if (Equals(m_multi_sampling.Count, value))
					return;

				m_multi_sampling.Count = value;
				m_multi_sampling.Quality = View3d.MultiSamp.BestQuality(value, RenderTargetFormat);
				TryCreateRenderTarget(m_dim_pixels);
			}
		}
		private View3d.MultiSamp m_multi_sampling;

		/// <summary>The DPI scaling when the RenderTarget was last resized</summary>
		public PointF DpiScale => new(PixelWidth / (float)Width, PixelHeight / (float)Height);

		/// <summary>The Dx9 render target that matches the area on screen</summary>
		public View3d.Texture? FrontBuffer
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					// Remove the render target
					using (LockScope())
						SetBackBuffer(D3DResourceType.IDirect3DSurface9, IntPtr.Zero);

					Util.Dispose(ref field);
				}
				field = value;
				if (field != null)
				{
					var ptr = field.PrivateDataPointer[View3d.Texture.UserData.Surface0Pointer];

					// Set the render target as the Dx9 surface
					using (LockScope())
						SetBackBuffer(D3DResourceType.IDirect3DSurface9, ptr, true);
				}

				// Notify of a new front buffer
				FrontBufferChanged?.Invoke(this, EventArgs.Empty);
			}
		}

		/// <summary>Raised when the front buffer is changed</summary>
		public event EventHandler? FrontBufferChanged;

		/// <summary>The size of the back buffer in pixels needed for MSAA and DPI</summary>
		public Size RequiredBackBufferSize => new(PixelWidth, PixelHeight);

		/// <summary>Set the dimensions of the render target</summary>
		public void SetRenderTargetSize(Size dim_pixels)
		{
			// If the render target size has changed, recreate it
			if (FrontBuffer == null || PixelWidth != dim_pixels.Width || PixelHeight != dim_pixels.Height)
				TryCreateRenderTarget(dim_pixels);
		}

		/// <summary>Copy from the Render Target to the Front Buffer</summary>
		public void Flip()
		{
			if (FrontBuffer == null)
				return;

			// Tell the 'ImageSource' what parts of the image are new (i.e. all of it)
			// See the Docs for 'AddDirtyRect'. This is how to tell the D3DImage that the texture content has changed.
			using (LockScope())
				AddDirtyRect(new Int32Rect(0, 0, m_dim_pixels.Width, m_dim_pixels.Height));
		}

		/// <summary>RAII Scope for locking the d3d image</summary>
		private Scope LockScope()
		{
			return Scope.Create(() => Lock(), () => Unlock());
		}

		/// <summary>Create a new render target if possible</summary>
		private void TryCreateRenderTarget(Size dim_pixels)
		{
			// Cannot create the render target until a window handle
			// has been assigned because Dx9 requires a window handle.
			if (WindowOwner == IntPtr.Zero || dim_pixels == Size.Empty)
				return;

			try
			{
				var opts = new View3d.TextureOptions(
					format: RenderTargetFormat,
					mips: 1,
					usage: View3d.EResFlags.AllowRenderTarget,
					dbg_name: "D3DImage RenderTarget FB");

				// Create the Dx9 render target of the required size;
				var new_rt = View3d.Texture.Dx9RenderTarget(WindowOwner, dim_pixels.Width, dim_pixels.Height, opts);

				// Set the new front buffer size before changing 'FrontBuffer'
				m_dim_pixels = dim_pixels;
				FrontBuffer = new_rt;
			}
			catch (Exception ex)
			{
				System.Diagnostics.Debug.WriteLine($"Failed to create render target: {ex.Message}");
				if (System.Diagnostics.Debugger.IsAttached)
					System.Diagnostics.Debugger.Break();
			}
		}

		/// <summary>True if the front buffer is available</summary>
		public new bool IsFrontBufferAvailable => base.IsFrontBufferAvailable || (bool)m_is_sw_fallback_enabled.GetValue(this)!;
		private static readonly FieldInfo m_is_sw_fallback_enabled = typeof(System.Windows.Interop.D3DImage).GetField("_isSoftwareFallbackEnabled", BindingFlags.NonPublic | BindingFlags.Instance) ?? throw new MissingFieldException("'D3DImage._isSoftwareFallbackEnabled' is missing");

		/// <summary></summary>
		private new void SetBackBuffer(D3DResourceType backBufferType, IntPtr backBuffer, bool enableSoftwareFallback)
		{
			// Hide the back buffer method because this object controls setting the back buffer
			base.SetBackBuffer(backBufferType, backBuffer, enableSoftwareFallback);
		}
		private new void SetBackBuffer(D3DResourceType backBufferType, IntPtr backBuffer)
		{
			// Hide the back buffer method because this object controls setting the back buffer
			base.SetBackBuffer(backBufferType, backBuffer);
		}

		/// <summary>Save the contents of this D3DImage to a file</summary>
		public void Save(string filepath)
		{
			var bmp_source = CopyBackBuffer();
			using (var file = new FileStream(filepath, FileMode.Create))
			{
				switch (Path_.Extn(filepath).ToLowerInvariant())
				{
					case ".png":
					{
						var encoder = new PngBitmapEncoder();
						encoder.Frames.Add(BitmapFrame.Create(bmp_source));
						encoder.Save(file);
						break;
					}
					case ".jpg":
					{
						var encoder = new JpegBitmapEncoder();
						encoder.Frames.Add(BitmapFrame.Create(bmp_source));
						encoder.Save(file);
						break;
					}
					case ".bmp":
					{
						var encoder = new BmpBitmapEncoder();
						encoder.Frames.Add(BitmapFrame.Create(bmp_source));
						encoder.Save(file);
						break;
					}
					default:
					{
						throw new NotSupportedException("Image format not supported");
					}
				}
			}
		}
	}
}
