using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Xml.Linq;
using Rylogic.Extn;
using Rylogic.Interop.Win32;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	using DockContainerDetail;

	/// <summary>
	/// Provides the implementation of the docking functionality.
	/// Classes that implement IDockable have one of these as a public property.</summary>
	[DebuggerDisplay("DockControl {TabText}")]
	public sealed class DockControl : IDisposable, INotifyPropertyChanged
	{
		///<remarks>
		/// Cut'n'Paste Boilerplate
		/// Inherit:
		///   IDockable
		///
		/// Constructor:
		///  DockControl = new DockControl(this, name)
		///  {
		///  	TabText = name, // optional
		///  	ShowTitle = false, // optional
		///  	DefaultDockLocation = new DockContainer.DockLocation(new[]{EDockSite.Centre}, 1), // optional
		///  	TabCMenu = CreateTabCMenu(), // optional
		///  };
		///
		/// Destructor:
		///  DockControl = null;
		///
		/// Property:
		///  /// <summary>Provides support for the DockContainer</summary>
		///  [Browsable(false)]
		///  [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
		///  public DockControl DockControl
		///  {
		///  	[DebuggerStepThrough] get { return m_dock_control; }
		///  	private set
		///  	{
		///  		if (m_dock_control == value) return;
		///  		Util.Dispose(ref m_dock_control);
		///  		m_dock_control = value;
		///  	}
		///  }
		///  private DockControl m_dock_control;
		///  </remarks>

		/// <summary>Create the docking functionality helper.</summary>
		/// <param name="owner">The control that docking is being provided for</param>
		/// <param name="persist_name">The name to use for this instance when saving the layout to XML</param>
		public DockControl(FrameworkElement owner, string persist_name)
		{
			if (!(owner is IDockable))
				throw new Exception("'owner' must implement IDockable");

			Owner = owner;
			PersistName = persist_name;
			DefaultDockLocation = new DockContainer.DockLocation();
			TabText = PersistName;
			TabIcon = null;

			DockAddresses = new Dictionary<Branch, EDockSite[]>();
		}
		public void Dispose()
		{
			Remove();
		}

		/// <summary>Get the control we're providing docking functionality for</summary>
		public FrameworkElement Owner { get; set; }

		/// <summary>Get/Set the dock container that manages this content.</summary>
		public DockContainer? DockContainer
		{
			get;
			set
			{
				if (field == value) return;
				var old = field;
				var nue = value;
				if (field != null)
				{
					field.ActiveContentChanged -= HandleActiveContentChanged;
					field.Forget(this);
				}
				field = value;
				if (field != null)
				{
					field.Remember(this);
					field.ActiveContentChanged += HandleActiveContentChanged;
				}
				DockContainerChanged?.Invoke(this, new DockContainerChangedEventArgs(old, nue));

				// Handlers
				void HandleActiveContentChanged(object? sender, ActiveContentChangedEventArgs e)
				{
					if (e.ContentNew == Owner || e.ContentOld == Owner)
					{
						ActiveChanged?.Invoke(this, e);

						// If the tab is flash when we're activated, switch back to active
						if (IsActiveContent && TabState == ETabState.Flashing)
							TabState = ETabState.Active;
					}
				}
			}
		}

		/// <summary>Raised when the pane this dockable is on is changing (possibly to null)</summary>
		public event EventHandler? PaneChanged;

		/// <summary>Raised when 'Close' is selected from the tab context menu</summary>
		public event EventHandler? Closed;

		/// <summary>Raised when this becomes the active content</summary>
		public event EventHandler<ActiveContentChangedEventArgs>? ActiveChanged;

		/// <summary>Raised during 'ToXml' to allow clients to add extra data to the XML data for this object</summary>
		public event EventHandler<DockContainerSavingLayoutEventArgs>? SavingLayout;

		/// <summary>Raised when a layout is being applied to the dock container</summary>
		public event EventHandler<DockContainerLoadingLayoutEventArgs>? LoadingLayout;

		/// <summary>Raised when this DockControl is assigned to a dock container (or possibly to null)</summary>
		public event EventHandler<DockContainerChangedEventArgs>? DockContainerChanged;

		/// <summary>Hide this item in the dock container. (Use Remove() to make remove it completely)</summary>
		public void Close()
		{
			// Don't close if not allowed
			if (!AllowClose)
				return;

			// Although removed from the UI, the dock container still remembers this instance.
			// The 'Windows' menu can be used to reopen it. If 'DestroyOnClose' is true or the
			// instance is 'Removed', then it can't be reopened.
			// Record where the content was so that 'FindAndShow' can restore it there. Floating windows
			// are discarded when they become empty, so floating content records the window bounds instead.
			if (DockPane != null)
			{
				RestoreLocation = TreeHost is FloatingWindow fw
					? new DockContainer.DockLocation(DockAddress, ContentIndex, float_bounds: fw.Bounds)
					: CurrentDockLocation;
			}
			DockPane = null;
			Closed?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Where 'DockContainer.FindAndShow' restores this content when it is not in any dock pane (null to use its last dock address)</summary>
		public DockContainer.DockLocation? RestoreLocation { get; set; }

		/// <summary>Remove this DockControl from whatever dock container it is in</summary>
		public void Remove()
		{
			DockPane = null;
			DockContainer = null;
		}

		/// <summary>If true, the instance is removed and disposed when closed</summary>
		public bool DestroyOnClose
		{
			get;
			set
			{
				if (field == value) return;
				if (field)
				{
					Closed -= HandleClose;
				}
				field = value;
				if (field)
				{
					Closed += HandleClose;
				}
				NotifyPropertyChanged(nameof(DestroyOnClose));

				// Handler
				void HandleClose(object? sender, EventArgs args)
				{
					Closed -= HandleClose;
					if (field)
					{
						Remove();
						if (Owner is IDisposable disposable)
							disposable.Dispose();
					}
				}
			}
		} = false;

		/// <summary>True if the dockable can be 'closed'</summary>
		public bool AllowClose
		{
			get => m_allow_close && (DockContainer?.Options.AllowUserCloseTabs ?? true);
			set
			{
				if (m_allow_close == value) return;
				m_allow_close = value;
				NotifyPropertyChanged(nameof(AllowClose));
			}
		}
		private bool m_allow_close = true;

		/// <summary>The dock container, auto-hide panel, or floating window that this content is docked within</summary>
		internal ITreeHost? TreeHost => DockPane?.TreeHost;

		/// <summary>Gets 'Owner' as an IDockable</summary>
		public IDockable Dockable => (IDockable)Owner;

		/// <summary>Get/Set the pane that this content resides within.</summary>
		public DockPane? DockPane
		{
			get => m_pane;
			set
			{
				if (m_pane == value) return;
				if (m_pane != null)
				{
					m_pane.AllContent.Remove(this);
				}
				if (value != null)
				{
					value.AllContent.Add(this);
				}
			}
		}
		private DockPane? m_pane;
		internal void SetDockPaneInternal(DockPane? pane)
		{
			if (m_pane == pane)
				return;

			// Calling 'DockControl.set_DockPane' changes the Content list in the DockPane
			// which calls this method to change the DockPane on the content without recursively
			// changing the content list.
			m_pane = pane;

			// Record the dock container that 'pane' belongs to.
			// Don't set 'DockContainer' to null, only change it. The dock container remembers content
			// that has been added, so that it can restore it to a previous dock pane.
			var dc = m_pane?.DockContainer;
			if (dc != null && DockContainer != dc)
				DockContainer = dc;

			// Content in a pane is shown, so it no longer needs a restore location
			if (m_pane != null)
				RestoreLocation = null;

			// If a pane is given, save the dock address for the root branch (if the pane is within a tree)
			if (m_pane != null && m_pane.ParentBranch != null && m_pane.RootBranch != null)
				DockAddresses[m_pane.RootBranch] = DockAddress;

			// Notify of pane changed
			NotifyPaneChanged();
		}
		internal void NotifyPaneChanged()
		{
			PaneChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>The name to use for this instance when saving layout to XML</summary>
		public string PersistName { get; set; }

		/// <summary>The dock location to use if not otherwise given</summary>
		public DockContainer.DockLocation DefaultDockLocation { get; set; }

		/// <summary>Get the current dock location</summary>
		public DockContainer.DockLocation CurrentDockLocation
		{
			get
			{
				if (TreeHost is FloatingWindow fw)
					return new DockContainer.DockLocation(DockAddress, ContentIndex, float_window_id: fw.Id);

				if (TreeHost is AutoHidePanel ah)
					return new DockContainer.DockLocation(DockAddress, ContentIndex, auto_hide: ah.DockSite);

				return new DockContainer.DockLocation(DockAddress, ContentIndex);
			}
		}

		/// <summary>Get/Set the dock site within the branch that contains 'DockPane'</summary>
		public EDockSite DockSite
		{
			get => DockPane?.DockSite ?? EDockSite.Centre;
			set
			{
				if (DockSite == value) return;

				// Must have an existing pane
				if (DockPane == null || DockPane.ParentBranch == null)
					throw new Exception("Can only change the dock site of this item after it has been added to a dock container");

				// Move this content to a different dock site within 'branch'
				var branch = DockPane.ParentBranch;
				var pane = branch.DockPane(value);
				pane.AllContent.Add(this);
			}
		}

		/// <summary>The full location of this content in the tree of docked windows from Root branch to 'DockPane'</summary>
		public EDockSite[] DockAddress => DockPane != null ? DockPane.DockAddress : new[] { EDockSite.None };

		/// <summary>The index of this dockable within the content of 'DockPane'</summary>
		public int ContentIndex => DockPane?.AllContent.IndexOf(this) ?? int.MaxValue;

		/// <summary>Get/Set whether this content is in a floating window</summary>
		public bool IsFloating
		{
			get => TreeHost is FloatingWindow;
			set
			{
				if (IsFloating == value) return;
				if (DockContainer == null)
					throw new ArgumentNullException("An associated DockContainer is required");

				// Float this dockable
				if (value)
				{
					// Get a floating window (preferably one that has hosted this item before)
					var fw = DockContainer.FloatingWindows.GetOrAdd(x => DockAddresses.ContainsKey(x.Root));

					// Get the dock address associated with this floating window
					var hs = DockAddressFor(fw);

					// Add this dockable to the floating window
					fw.Add(Dockable, hs);

					// Ensure the floating window is visible
					if (!fw.IsVisible)
						fw.Show();

					fw.FitToContent();
				}

				// Dock this content back in the dock container
				else
				{
					// Get the dock address associated with the dock container
					var hs = DockAddressFor(DockContainer);

					// Return this content to the dock container, and refit the floating window it left
					var old_fw = (FloatingWindow?)TreeHost;
					DockContainer.Add(Dockable, hs);
					old_fw?.FitToContent();
				}

				// Copy pane state to the new pane
				if (DockPane != null)
				{
					//if (strip_location != null)
					//	DockPane.TabStripCtrl.StripLocation = strip_location.Value;
				}
			}
		}

		/// <summary>Get/Set whether this content is in an auto-hide panel</summary>
		public bool IsAutoHide
		{
			get => TreeHost is AutoHidePanel;
			set
			{
				if (IsAutoHide == value) return;
				if (DockContainer == null)
					throw new ArgumentNullException("An associated DockContainer is required");

				// Auto hide this dockable
				if (value)
				{
					// If currently docked in the centre, auto hide to the right
					var side = DockPane?.AutoHideSide ?? EDockSite.Centre;
					if (side == EDockSite.Centre) side = EDockSite.Right;

					// Get the auto-hide panel associated with this dock site
					var ah = DockContainer.AutoHidePanels[side];

					// Add this dockable to the auto-hide panel
					ah.Add(Dockable);
				}

				// Dock this content back in the dock container
				else
				{
					var old_host = TreeHost as AutoHidePanel;

					// Get the dock address associated with the dock container
					var hs = DockAddressFor(DockContainer);

					// Return this content to the dock container
					DockContainer.Add(Dockable, hs);

					// Hide the auto hide panel so that it doesn't obscure the now pinned content
					if (old_host != null)
						old_host.PoppedOut = false;
				}
			}
		}

		/// <summary>True if this content is visible somewhere in the dock container or it's floating windows</summary>
		public bool IsVisible => IsActiveContentInPane && (DockPane?.IsVisible ?? false);

		/// <summary>A record of where this pane was located in a dock container, auto hide window, or floating window</summary>
		internal EDockSite[] DockAddressFor(ITreeHost tree)
		{
			if (DockContainer == null)
				throw new ArgumentNullException("An associated DockContainer is required");

			return DockAddresses.GetOrAdd(tree.Root, x =>
			{
				if (DefaultDockLocation.FloatingWindowId != null)
				{
					// If the default dock location is a floating window, and 'root' is
					// the root branch of that floating window, then return the default address.
					// Otherwise, return sensible defaults appropriate for 'root'
					var fw = DockContainer.FloatingWindows.Get(DefaultDockLocation.FloatingWindowId.Value);
					if (fw != null)
					{
						if (fw.Root == tree.Root) return DefaultDockLocation.Address;
						return new[] { EDockSite.Centre };
					}
				}
				else if (DefaultDockLocation.AutoHide != null)
				{
					// If the default dock location is an auto hide window, and 'root' is
					// the root branch of that auto hide window, then return the default address.
					// If 'root' is a floating window, return Centre
					// If 'root' is the dock container, return the edge site that matches the auto hide panel's dock site
					var ah = DockContainer.AutoHidePanels[DefaultDockLocation.AutoHide.Value];
					if (ah != null)
					{
						if (ah.Root == tree.Root) return DefaultDockLocation.Address;
						if (tree is FloatingWindow) return new[] { EDockSite.Centre };
						return new[] { ah.DockSite };
					}
				}
				else if (DockContainer.Root == tree.Root)
				{
					// If the default dock location is the dock container and 'root' is
					// the root branch of the dock container, then return the default address
					return DefaultDockLocation.Address;
				}
				// Otherwise, default to the centre dock site
				return new[] { EDockSite.Centre };
			});
		}
		private Dictionary<Branch, EDockSite[]> DockAddresses { get; }

		/// <summary>Return the tab button associated with this content. (Null when not added to a DockPane)</summary>
		internal TabButton? TabButton => DockPane?.TabStrip.Buttons.FirstOrDefault(x => x.DockControl == this);

		/// <summary>Get/Set whether the dock pane title control is visible while this content is active. Null means inherit default</summary>
		public bool? ShowTitle { get; set; }

		/// <summary>Get/Set whether the location of the tab strip control while this content is active. Null means inherit default</summary>
		public EDockSite? TabStripLocation { get; set; }

		/// <summary>The text to display on the title bar. Defaults to 'TabText'</summary>
		public string CaptionText
		{
			get => m_caption_text ?? TabText;
			set
			{
				if (m_caption_text == value) return;
				m_caption_text = value;
				NotifyPropertyChanged(nameof(CaptionText));
			}
		}
		private string? m_caption_text;

		/// <summary>The text to display on the tab. Defaults to 'Owner.Text'</summary>
		public string TabText
		{
			get => m_tab_text ?? (Owner as Window)?.Title ?? string.Empty;
			set
			{
				if (m_tab_text == value) return;
				m_tab_text = value;
				NotifyPropertyChanged(nameof(TabText));
				NotifyPropertyChanged(nameof(CaptionText));
			}
		}
		private string? m_tab_text;

		/// <summary>The icon to display on the tab. Defaults to '(Owner as Window)?.Icon'</summary>
		public ImageSource? TabIcon
		{
			get => m_icon ?? (Owner as Window)?.Icon;
			set
			{
				if (m_icon == value) return;
				m_icon = value;
				NotifyPropertyChanged(nameof(TabIcon));
			}
		}
		private ImageSource? m_icon;

		/// <summary>A tool tip to display when the mouse hovers over the tab for this content</summary>
		public string? TabToolTip
		{
			get => m_tab_tt ?? TabText;
			set => m_tab_tt = value;
		}
		private string? m_tab_tt;

		/// <summary>A context menu to display when the tab for this content is right clicked</summary>
		public ContextMenu? TabCMenu
		{
			get => m_tab_cmenu ?? DefaultTabCMenu();
			set
			{
				if (m_tab_cmenu == value) return;
				m_tab_cmenu = value;
				NotifyPropertyChanged(nameof(TabCMenu));
			}
		}
		public ContextMenu? m_tab_cmenu;

		/// <summary>The current state of the tab button associated with this content</summary>
		public ETabState TabState
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(TabState));
			}
		}

		/// <summary>The preferred sizing mode when this content it the visible content within a Pane</summary>
		public EDockResizeMode ResizeMode
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(ResizeMode));
			}
		}

		/// <summary>
		/// True if a floating window that hosts this content should resize to fit the content when content is floated, or added to or removed from the window.
		/// A floating window only fits its content when all of its content has this flag set. Use for content with a natural size, not content that stretches to fill.</summary>
		public bool FloatingSizeToContent
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(FloatingSizeToContent));
			}
		}

		/// <summary>Creates a default context menu for the tab. Use: TabCMenu = DefaultTabCMenu()</summary>
		public ContextMenu? DefaultTabCMenu()
		{
			var cmenu = new ContextMenu();
			if (AllowClose)
			{
				var opt = cmenu.Items.Add2(new MenuItem { Header = "Close" });
				opt.Click += delegate { Close(); };
			}
			return cmenu.Items.Count != 0 ? cmenu : null;
		}

		/// <summary>True if this content is the active content within the owning dock container</summary>
		public bool IsActiveContent
		{
			get { return DockContainer?.ActiveContent == this; }
			set
			{
				if (value && DockContainer != null)
					DockContainer.ActiveContent = this;
			}
		}

		/// <summary>True if this content is the active content within its containing dock pane</summary>
		public bool IsActiveContentInPane
		{
			get { return DockPane?.ContentView.CurrentItem == this; }
			set
			{
				if (!value || DockPane == null) return;
				DockPane.ContentView.MoveCurrentTo(this);
			}
		}

		/// <summary>Save/Restore the control (possibly child control) that had input focus when the content was last active</summary>
		internal void SaveFocus()
		{
			// Save the WPF element with keyboard focus
			m_kb_focus = Owner.IsKeyboardFocusWithin ? Keyboard.FocusedElement : null;

			// If focus is held by a child win32 control, record it
			var hwnd = User32.GetFocus();
			m_win32_focus = User32.IsChild(Owner.Hwnd(), hwnd) ? hwnd : IntPtr.Zero;
		}
		internal void RestoreFocus()
		{
			// Only restore focus if 'm_kb_focus' is still a child of 'Owner'
			if (m_kb_focus is DependencyObject dep && Owner.IsVisualDescendant(dep))
				Keyboard.Focus(m_kb_focus);

			// Restore the win32 focus if the control is still a child of the WPF window
			if (m_win32_focus != IntPtr.Zero && User32.IsChild(Owner.Hwnd(), m_win32_focus))
				User32.SetFocus(m_win32_focus);

			m_kb_focus = null;
			m_win32_focus = IntPtr.Zero;
		}
		private IInputElement? m_kb_focus;
		private IntPtr m_win32_focus;

		/// <summary>Save to XML</summary>
		public XElement ToXml(XElement node)
		{
			// Allow clients to add extra data that will be provided in LoadLayout
			var user = new XElement(XmlTag.UserData);
			SavingLayout?.Invoke(this, new DockContainerSavingLayoutEventArgs(user));

			// Add the name to identify the content
			node.Add2(XmlTag.Name, PersistName, false);
			node.Add2(XmlTag.Type, Owner.GetType().FullName, false);
			if (DockPane == null && RestoreLocation != null)
			{
				// Closed content records where it should be restored to
				RestoreLocation.ToXml(node);
				node.Add2(XmlTag.Closed, true, false);
			}
			else
			{
				CurrentDockLocation.ToXml(node);
			}
			node.Add2(user);

			return node;
		}
		public void LoadLayout(XElement user_data)
		{
			LoadingLayout?.Invoke(this, new DockContainerLoadingLayoutEventArgs(user_data));
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}

	/// <summary>A wrapper control that hosts a control and implements IDockable</summary>
	public sealed class Dockable : DockPanel, IDisposable, IDockable
	{
		public Dockable(Control hostee, string persist_name, DockContainer.DockLocation? location = null)
		{
			MinWidth = hostee.MinWidth;
			MinHeight = hostee.MinHeight;
			MaxWidth = hostee.MaxWidth;
			MaxHeight = hostee.MaxHeight;
			Children.Add(hostee);

			DockControl = new DockControl(this, persist_name) { TabText = persist_name };
			if (location != null)
				DockControl.DefaultDockLocation = location;
		}
		public void Dispose()
		{
			DockControl = null!;
		}

		/// <summary>Provides support for the DockContainer</summary>
		public DockControl DockControl
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;
	}
}
