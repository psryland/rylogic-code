using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms.Integration;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using Microsoft.VisualStudio.Shell;
using Rylogic.Interop.Win32;

namespace Rylogic.LDrawVisualiser
{
	/// <summary>
	/// Base class for VS option pages that host WPF content.
	/// Credit: https://github.com/dwmkerr/switch/
	/// </summary>
	[ComVisible(true)]
	public abstract class UIElementDialogPage : DialogPage
	{
		static UIElementDialogPage()
		{
			EventManager.RegisterClassHandler(typeof(ComboBox), DialogKeyPendingEvent, (EventHandler<DialogKeyEventArgs>)HandleComboBoxDialogKey);
			void HandleComboBoxDialogKey(object sender, DialogKeyEventArgs e)
			{
				var combo_box = (ComboBox)sender;
				if ((e.Key == Key.Enter || e.Key == Key.Escape) && combo_box.IsDropDownOpen)
					e.Handled = true;
			}
		}

		public static readonly RoutedEvent DialogKeyPendingEvent = EventManager.RegisterRoutedEvent(
			"DialogKeyPending", RoutingStrategy.Bubble, typeof(EventHandler<DialogKeyEventArgs>), typeof(UIElementDialogPage));

		[Browsable(false), DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		protected override System.Windows.Forms.IWin32Window Window
		{
			get
			{
				if (m_element_host == null)
				{
					m_element_host = new DialogPageElementHost { Dock = System.Windows.Forms.DockStyle.Fill };
					var child = CreateChild();
					if (child != null)
					{
						TextOptions.SetTextFormattingMode(child, TextFormattingMode.Display);
						HookChildHwndSource(child);
						m_element_host.Child = child;
					}
				}
				return m_element_host;
			}
		}
		private ElementHost? m_element_host;

		protected abstract UIElement CreateChild();

		private void HookChildHwndSource(UIElement child)
		{
			PresentationSource.AddSourceChangedHandler(child, OnSourceChanged);
			void OnSourceChanged(object sender, SourceChangedEventArgs e)
			{
				if (e.OldSource is HwndSource old_source) old_source.RemoveHook(SourceHook);
				if (e.NewSource is HwndSource new_source) new_source.AddHook(SourceHook);
			}
			IntPtr SourceHook(IntPtr hwnd, int msg, IntPtr wParam, IntPtr lParam, ref bool handled)
			{
				switch (msg)
				{
				case Win32.WM_GETDLGCODE:
					{
						int dlg_code = Win32.DLGC_WANTARROWS | Win32.DLGC_WANTTAB | Win32.DLGC_WANTCHARS;
						if (Keyboard.FocusedElement is IInputElement current_element)
						{
							var args = new DialogKeyEventArgs(DialogKeyPendingEvent, KeyInterop.KeyFromVirtualKey(wParam.ToInt32()));
							current_element.RaiseEvent(args);
							if (args.Handled)
								dlg_code |= Win32.DLGC_WANTALLKEYS;
						}
						handled = true;
						return new IntPtr(dlg_code);
					}
				}
				return IntPtr.Zero;
			}
		}

		private class DialogPageElementHost : ElementHost
		{
			protected override void WndProc(ref System.Windows.Forms.Message m)
			{
				base.WndProc(ref m);
			}
		}
	}

	public class DialogKeyEventArgs : RoutedEventArgs
	{
		public DialogKeyEventArgs(RoutedEvent evt, Key key) : base(evt) => Key = key;
		public Key Key { get; }
	}
}
