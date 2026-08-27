using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Gui.WPF.NotifyIcon;
using Rylogic.Interop.Win32;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public class NotificationIcon :FrameworkElement, IDisposable
	{
		// Usage:
		//  - In your main window XAML, add:
		//       <!-- Notification Tray -->
		//       <gui:NotificationIcon
		//           ContextMenu="{StaticResource NotificationCMenu}"
		//           />
		//           <!-- Optional:
		//           Visibility = "Visible"
		//           IconSource="{StaticResource tray_icon}"
		//           LeftClickCommand="{Binding SomeOtherFunction}"
		//           RightClickCommand="{Binding SomeOtherFunction}"
		//           -->
		//  - The NotificationIcon.DataContext is the owning window.
		//  - The NotificationIcon.ContextMenu.DataContext is set to 'this' if not already set.
		//    To access commands on the owning window use:
		//          <MenuItem Header="Exit" Command="{Binding Owner.CloseApp}"/>
		//  - In your main window, remember to dispose the NotificationIcon (or it gets left behind)
		//    Use: Gui_.DisposeChildren(this); in your main window Closed() override.
		//  - If you want the NotificationIcon to inherit the icon from the owning window, you need to set
		//    the 'Icon' property in the MainWindow.xaml. The icon set from the project settings doesn't seem
		//    to be available in OnInitialize().
		//    
		// Notes:
		//  - In your window.Closing handler, set 'Cancel' and hide the window

		private readonly object m_lock = new();

		public NotificationIcon()
		{
			MessageSink = WPFUtil.IsDesignMode ? WindowMessageSink.CreateEmpty() : new WindowMessageSink();
			IconData = Win32.NOTIFYICONDATA.New(MessageSink.HWnd, WindowMessageSink.CallbackMessageId, Win32.ENotifyIconVersion.Win95);

			// Commands
			ShowCMenu = Command.Create(this, ShowCMenuInternal);
			ShowOwner = Command.Create(this, ShowOwnerInternal);

			// Default actions
			LeftClickCommand = ShowOwner;
			RightClickCommand = ShowCMenu;
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool disposing)
		{
			IsIconCreated = false;
			MessageSink = null!;
		}
		protected override void OnInitialized(EventArgs e)
		{
			base.OnInitialized(e);

			// Inherit the owning window's icon. The 'Icon' property needs to be
			// set in the owning windows XAML for this to work.
			if (Icon == null && Owner?.Icon is ImageSource icon)
				IconSource = icon;

			// Create the system tray icon
			IsIconCreated = true;
		}

		/// <summary>The owning window</summary>
		public Window? Owner => Window.GetWindow(this);

		/// <summary>Get/Set whether the tray icon is created in the shell</summary>
		public bool IsIconCreated
		{
			get;
			set
			{
				value &= !WPFUtil.IsDesignMode;
				if (field == value) return;
				if (field)
				{
					lock (m_lock)
					{
						IconData.uFlags = Win32.ENotifyIconDataMembers.Message;
						Shell32.NotifyIcon(Win32.ENotifyCommand.Delete, ref IconData);
						field = false;
					}
				}
				if (value)
				{
					lock (m_lock)
					{
						// Add the icon to the shell
						IconData.uFlags = Win32.ENotifyIconDataMembers.Message | Win32.ENotifyIconDataMembers.Icon | Win32.ENotifyIconDataMembers.Tip;
						var status = Shell32.NotifyIcon(Win32.ENotifyCommand.Add, ref IconData);
						if (!status)
						{
							// Couldn't create the icon - we can assume this is because explorer is not running (yet!)
							// -> try a bit later again rather than throwing an exception. Typically, if the windows
							// shell is being loaded later, this method is being re-invoked from OnTaskbarCreated
							// (we could also retry after a delay, but that's currently YAGNI)
							return;
						}

						// Set to most recent version
						IconData.VersionOrTimeout = (uint)Win32.ENotifyIconVersion.Vista;
						if (!Shell32.NotifyIcon(Win32.ENotifyCommand.SetVersion, ref IconData))
						{
							IconData.VersionOrTimeout = (uint)Win32.ENotifyIconVersion.Win2000;
							if (!Shell32.NotifyIcon(Win32.ENotifyCommand.SetVersion, ref IconData))
							{
								IconData.VersionOrTimeout = (uint)Win32.ENotifyIconVersion.Win95;
								if (!Shell32.NotifyIcon(Win32.ENotifyCommand.SetVersion, ref IconData))
								{
									throw new Win32Exception("Failed to set the notification icon version");
								}
							}
						}

						// Set the version in the message sink
						MessageSink.Version = (Win32.ENotifyIconVersion)IconData.VersionOrTimeout;
						field = true;
					}
				}
			}
		}

		/// <summary>Window message handler for messages from the notification icon</summary>
		private WindowMessageSink MessageSink
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.MouseButton -= HandleMouseButton;
					field.TaskbarCreated -= HandleTaskbarCreated;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.TaskbarCreated += HandleTaskbarCreated;
					field.MouseButton += HandleMouseButton;
				}

				// Handlers
				void HandleTaskbarCreated(object? sender, EventArgs e)
				{
					// Recreate the icon
					IsIconCreated = false;
					IsIconCreated = true;
				}
				void HandleMouseButton(object sender, MouseButtonEventArgs e)
				{
					switch (e.ChangedButton)
					{
						case MouseButton.Left:
						{
							if (LeftClickCommand is ICommand cmd && cmd.CanExecute(null) && e.RoutedEvent == Mouse.MouseUpEvent)
								cmd.Execute(null);
							break;
						}
						case MouseButton.Middle:
						{
							break;
						}
						case MouseButton.Right:
						{
							if (RightClickCommand is ICommand cmd && cmd.CanExecute(null) && e.RoutedEvent == Mouse.MouseUpEvent)
								cmd.Execute(null);
							break;
						}
						case MouseButton.XButton1:
						{
							break;
						}
						case MouseButton.XButton2:
						{
							break;
						}
					}
				}
			}
		} = null!;

		/// <summary>Represents the current icon data.</summary>
		private Win32.NOTIFYICONDATA IconData;

		/// <summary>
		/// Gets or sets the icon to be displayed. This is not a dependency property. If you want to assign
		/// the property through XAML, please use the <see cref="IconSource"/> dependency property.</summary>
		[Browsable(false)]
		public System.Drawing.Icon? Icon
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				IconData.hIcon = field?.Handle ?? IntPtr.Zero;
				IconData.uFlags = Win32.ENotifyIconDataMembers.Icon;
				Shell32.NotifyIcon(Win32.ENotifyCommand.Modify, ref IconData);
			}
		}

		/// <summary>Open the context menu</summary>
		public ICommand ShowCMenu { get; }
		private void ShowCMenuInternal()
		{
			if (ContextMenu is ContextMenu cmenu)
			{
				cmenu.DataContext ??= this;
				cmenu.IsOpen = true;
			}
		}

		/// <summary>Make the owner window visible and in focus</summary>
		public ICommand ShowOwner { get; }
		private void ShowOwnerInternal()
		{
			if (Owner is Window owner)
			{
				owner.Visibility = Visibility.Visible;
				owner.WindowState = WindowState.Normal;
				owner.Focus();
			}
		}

		/// <summary>Resolves an image source and updates the <see cref="Icon" /> property accordingly.</summary>
		[Category(CategoryName)]
		[Description("Sets the displayed task bar icon.")]
		public ImageSource? IconSource
		{
			get => (ImageSource?)GetValue(IconSourceProperty);
			set => SetValue(IconSourceProperty, value);
		}
		private void IconSource_Changed(ImageSource? new_value)
		{
			// Resolving the ImageSource at design time is unlikely to work
			if (!WPFUtil.IsDesignMode)
				Icon = new_value?.ToIcon();
		}
		public static readonly DependencyProperty IconSourceProperty = Gui_.DPRegister<NotificationIcon>(nameof(IconSource), null, Gui_.EDPFlags.None);

		/// <summary>A tool tip text that is being displayed if no custom <see cref="ToolTip"/> was set or if custom tool tips are not supported.</summary>
		[Category(CategoryName)]
		[Description("Alternative to a fully blown ToolTip, which is only displayed on Vista and above.")]
		public string ToolTipText
		{
			get => (string)GetValue(ToolTipTextProperty);
			set => SetValue(ToolTipTextProperty, value);
		}
		private void ToolTipText_Changed(string new_value)
		{
			IconData.uFlags = Win32.ENotifyIconDataMembers.Tip;
			IconData.szTip = new_value;
			Shell32.NotifyIcon(Win32.ENotifyCommand.Modify, ref IconData);
		}
		public static readonly DependencyProperty ToolTipTextProperty = Gui_.DPRegister<NotificationIcon>(nameof(ToolTipText), string.Empty, Gui_.EDPFlags.None);

		/// <summary>Associates a command that is being executed if the tray icon is being left-clicked.</summary>
		[Category(CategoryName)]
		[Description("A command that is being executed if the tray icon is being left-clicked.")]
		public ICommand? LeftClickCommand
		{
			get => (ICommand?)GetValue(LeftClickCommandProperty);
			set => SetValue(LeftClickCommandProperty, value);
		}
		public static readonly DependencyProperty LeftClickCommandProperty = Gui_.DPRegister<NotificationIcon>(nameof(LeftClickCommand), null, Gui_.EDPFlags.None);

		/// <summary>Associates a command that is being executed if the tray icon is being right-clicked.</summary>
		[Category(CategoryName)]
		[Description("A command that is being executed if the tray icon is being left-clicked.")]
		public ICommand? RightClickCommand
		{
			get => (ICommand?)GetValue(RightClickCommandProperty);
			set => SetValue(RightClickCommandProperty, value);
		}
		public static readonly DependencyProperty RightClickCommandProperty = Gui_.DPRegister<NotificationIcon>(nameof(RightClickCommand), null, Gui_.EDPFlags.None);

		/// <summary>Category name that is set on designer properties.</summary>
		public const string CategoryName = "NotifyIcon";
	}
}
