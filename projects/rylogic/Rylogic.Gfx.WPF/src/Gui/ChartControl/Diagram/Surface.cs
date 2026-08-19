using System;
using System.Diagnostics;
using System.Drawing.Drawing2D;
using System.Xml.Linq;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	public sealed class Surface :IDisposable
	{
		// Notes:
		//  - A wrapper around a view3d texture
		public Surface(int sx, int sy, string? dbg_name = null)
		{
			sx = (int)Math.Max(1.0, sx + 0.5);
			sy = (int)Math.Max(1.0, sy + 0.5);
			var opts = new View3d.TextureOptions
			{
				DbgName = dbg_name ?? string.Empty,
				ColourKey = 0,
				//GdiCompatible = true,
				//AddrU = View3d.EAddrMode.D3D11_TEXTURE_ADDRESS_CLAMP,
				//AddrV = View3d.EAddrMode.D3D11_TEXTURE_ADDRESS_CLAMP,
			};
			Surf = new View3d.Texture(sx, sy, opts);// D3D11_FILTER_ANISOTROPIC});
		}
		public Surface(v2 sz, string? dbg_name = null)
			: this(sz.xi, sz.yi, dbg_name)
		{ }
		public Surface(XElement node)
			: this(node.Element(XmlTag.Size).As<v2>())
		{ }
		public void Dispose()
		{
			Surf = null!;
		}

		/// <summary>Export to XML</summary>
		public XElement ToXml(XElement node)
		{
			node.Add2(XmlTag.Size, Size, false);
			return node;
		}

		/// <summary>Texture format information</summary>
		public View3d.ImageInfo Info => Surf.Info;

		/// <summary>Get/Set the size of the texture</summary>
		public v2 Size
		{
			get => v2.From(Surf.Size);
			set
			{
				// Get the size in pixels
				var tex_size = value.ToSize();
				if (Surf == null || Surf.Size == tex_size)
					return;

				// Ensure positive dimensions
				if (tex_size.Width <= 0) tex_size.Width = 1;
				if (tex_size.Height <= 0) tex_size.Height = 1;
				Surf.Size = tex_size;
			}
		}

		/// <summary>The texture surface</summary>
		public View3d.Texture Surf
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>Lock the texture for drawing on.</summary>
		public View3d.Texture.Lock LockSurface(bool discard)
		{
			var lck = Surf.LockSurface(discard);
			lck.Gfx.SmoothingMode = SmoothingMode.AntiAlias;
			lck.Gfx.CompositingMode = CompositingMode.SourceOver;
			lck.Gfx.CompositingQuality = CompositingQuality.HighQuality;
			return lck;
		}
	}
}
