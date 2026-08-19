using System;
using System.Windows;
using Rylogic.Gui.WPF;

namespace Rylogic.Extn
{
	public static class View3d_
	{
		/// <summary>Set the render target size of this D3D11Image from the given visual</summary>
		public static void SetRenderTargetSize(this Gfx.D3DImage d3d_image, FrameworkElement element, double scale = 1)
		{
			// Ensure the Window that contains 'element' is also the owner of 'd3d_image'
			d3d_image.WindowOwner = Window.GetWindow(element).Hwnd();

			// Scale for DPI
			var size = new Size(element.DesiredSize.Width, element.DesiredSize.Height).TransformToDevice(element);
			var width  = (int)Math.Max(1, scale * Math.Round(size.Width));
			var height = (int)Math.Max(1, scale * Math.Round(size.Height));
			d3d_image.SetRenderTargetSize(new(width, height));
		}
	}
}
