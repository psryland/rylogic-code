using System.Windows;
using Rylogic.Gui.WPF;
using Rylogic.Maths;
using Rylogic.Utility;

namespace LDraw.Dialogs
{
	public partial class AboutUI :Window
	{
		public AboutUI(Window owner)
		{
			InitializeComponent();
			Owner = owner;
			Icon = owner?.Icon;

			Accept = Command.Create(this, AcceptInternal);

			DataContext = this;

			var anim = $"*RootAnimation {{ *Style {{Continuous}} *Period {{1}} *AngVelocity {{ 1 1 1 }} }}";
			var src = m_view.View3d.LoadScriptFromString(
				$"*Sphere s FF0000FF {{ *Data {{0.6}} }}\n" +
				$"*Box b0 FFFF0000 {{ *Data {{1.003}} {anim} }}\n" +
				$"*Box b1 FF00FF00 {{ *Data {{1.002}} {anim} }}\n" +
				$"*Box b2 FF0000FF {{ *Data {{1.001}} {anim} }}\n");

			m_view.Window.AddObjects(src.ContextId);
			m_view.Window.SetLightSource(v4.Origin, new v4(-1, -1, -2, 0), true);
			m_view.Camera.SetPosition(new v4(2, 2, 2, 1));
			Closed += delegate
			{
				m_view.Dispose();
			};
		}

		/// <summary></summary>
		public string AboutInfo =>
			$"{Util.AppProductName} - Scripted 3D object viewer.\r\n" +
			$"Version: {Util.AppVersion}\r\n" +
			$"{Util.AppCopyright}\r\n";

		/// <summary>OK button</summary>
		public Command Accept { get; }
		private void AcceptInternal()
		{
			Close();
		}
	}
}
