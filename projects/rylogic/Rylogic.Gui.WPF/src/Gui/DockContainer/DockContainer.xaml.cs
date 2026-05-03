using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Xml.Linq;
using Microsoft.Win32;
using Rylogic.Extn;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	using DockContainerDetail;

	/// <summary>A dock container is the parent control that manages docking of controls that implement IDockable.</summary>
	public sealed partial class DockContainer : DockPanel, ITreeHost, IDisposable
	{
		// Notes:
		//  - The dock container does not own the content. To clean up disposable content
		//    use 'Util.DisposeRange(m_dc.AllContent.OfType<IDisposable>());'
		//  - Don't be tempted to add a 'DisposeContent' flag because it's ambiguous when
		//    dispose should be called on the content. 

		/// <summary>The number of valid dock sites</summary>
		private const int DockSiteCount = 5;

		/// <summary>Create a new dock container</summary>
		static DockContainer()
		{
			// DockContainer settings using WPF types
			Xml_.Config.SupportWPFTypes();
		}
		public DockContainer()
		{
			InitializeComponent();
			Options = new OptionsData();
			m_all_content = new HashSet<DockControl>();
			ActiveContentManager = new ActiveContentManager(this);

			// The auto hide tab strips dock around the edge of the main dock container
			// but remain hidden unless there are auto hide panels with content.
			// The rest of the docking area is within 'centre' which contains the 
			// root branch (filling the canvas) and four AutoHidePanels.

			// Add a Canvas for the centre area and add the root branch (filling the canvas)
			var centre = new Canvas { Name = "DockContainerCentre" };

			// Create the auto hide panels
			AutoHidePanels = new AutoHidePanelCollection(this, centre);

			// Create a collection for floating windows
			FloatingWindows = new FloatingWindowCollection(this);

			// Add the centre content
			Children.Add2(centre);
			Root = centre.Children.Add2(new Branch(this, DockSizeData.Quarters));
			Root.SetBinding(Branch.WidthProperty, new Binding(nameof(Canvas.ActualWidth)) { Source = centre });
			Root.SetBinding(Branch.HeightProperty, new Binding(nameof(Canvas.ActualHeight)) { Source = centre });
			
			// Ensure the default centre pane exists
			Root.PruneBranches();
			ActivePane = (DockPane?)Root.Descendants[EDockSite.Centre].Item ?? throw new ArgumentNullException("Expected a DockPane");

			// Create Commands
			CmdLoadLayout = Command.Create(this, CmdLoadLayoutInternal);
			CmdSaveLayout = Command.Create(this, CmdSaveLayoutInternal);
			CmdResetLayout = Command.Create(this, CmdResetLayoutInternal);
		}
		public void Dispose()
		{
			FloatingWindows = null!;
			AutoHidePanels = null!;
			Root = null!;
		}

		/// <summary>The dock container that owns this instance</summary>
		DockContainer ITreeHost.DockContainer => this;

		/// <summary>Options for the dock container</summary>
		public OptionsData Options { get; }

		/// <summary>Manages events and changing of active pane/content</summary>
		internal ActiveContentManager ActiveContentManager { get; }

		/// <summary>Get/Set the globally active content (i.e. owned by this dock container). This will cause the pane that the content is on to also become active.</summary>
		[Browsable(false)]
		public IDockable? ActiveDockable
		{
			// Note: this setter isn't called when the user clicks on some content, ActivePane.set is called.
			get => ActiveContent?.Dockable;
			set => ActiveContent = value?.DockControl;
		}

		/// <summary>Get/Set the globally active content (across the main control, all floating windows, and all auto hide panels)</summary>
		[Browsable(false)]
		public DockControl? ActiveContent
		{
			get => ActiveContentManager.ActiveContent;
			set => ActiveContentManager.ActiveContent = value;
		}

		/// <summary>Get/Set the globally active pane (i.e. owned by this dock container). Note, this pane may be within the dock container, a floating window, or an auto hide window</summary>
		[Browsable(false)]
		public DockPane? ActivePane
		{
			get => ActiveContentManager.ActivePane;
			set => ActiveContentManager.ActivePane = value;
		}

		/// <summary>Return all tree hosts associated with this dock container</summary>
		internal IEnumerable<ITreeHost> AllTreeHosts
		{
			get
			{
				yield return this;
				foreach (var ahp in AutoHidePanels)
					yield return ahp;
				foreach (var fw in FloatingWindows.Where(x => x.IsVisible))
					yield return fw;
			}
		}

		/// <summary>Enumerate through all panes managed by this container</summary>
		public IEnumerable<DockPane> AllPanes
		{
			get
			{
				// Return all the panes hosted in the dock container
				foreach (var p in Root.AllPanes)
					yield return p;

				// Return all panes in the auto hide windows
				foreach (var ah in AutoHidePanels)
					foreach (var p in ah.Root.AllPanes)
						yield return p;

				// Return all panes hosted in the floating windows
				foreach (var fw in FloatingWindows)
					foreach (var p in fw.Root.AllPanes)
						yield return p;
			}
		}

		/// <summary>Enumerate all IDockable's managed by this container</summary>
		public IEnumerable<IDockable> AllContent => AllContentInternal.Select(x => x.Dockable);
		internal IEnumerable<DockControl> AllContentInternal => m_all_content;
		private readonly HashSet<DockControl> m_all_content;

		/// <summary>Return the instances of 'TContent' contained in the dock container</summary>
		public IEnumerable<TContent> FindContent<TContent>() => AllContent.OfType<TContent>();

		/// <summary>Return the dock pane at the given dock site or null if the site does not contain a dock pane</summary>
		public DockPane? GetPane(params EDockSite[] location)
		{
			return Root.DescendantAt(location)?.Item as DockPane;
		}

		/// <summary>The centre pane of the root level tree</summary>
		public DockPane RootPane => Root.DockPane(EDockSite.Centre);

		/// <summary>Returns a reference to the dock sizes at a given level in the tree. 'location' should point to a branch otherwise an exception is thrown. location.Length == 0 returns the root level sizes</summary>
		public DockSizeData DockSizeAt(params EDockSite[] location)
		{
			return Root.DockSizeAt(location);
		}

		/// <summary>Save a reference to 'dockable' in this dock container</summary>
		internal void Remember(DockControl dc)
		{
			m_all_content.Add(dc);
		}

		/// <summary>Release the reference to 'dockable' in this dock container</summary>
		internal void Forget(DockControl dc)
		{
			m_all_content.Remove(dc);
		}

		/// <summary>The root of the tree in this dock container</summary>
		internal Branch Root
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.TreeChanged -= HandleTreeChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.TreeChanged += HandleTreeChanged;
				}

				/// <summary>Handler for when panes/branches are added/removed from the tree</summary>
				void HandleTreeChanged(object? sender, TreeChangedEventArgs obj)
				{
					switch (obj.Action)
					{
					case TreeChangedEventArgs.EAction.ActiveContent:
						{
							// Active content notifications come from DockPanes when the visible content
							// changes. If the pane is the active pane, we need to make pane's visible
							// content the active content.
							if (obj.DockPane == ActivePane)
								ActiveContent = obj.DockControl;
							break;
						}
					case TreeChangedEventArgs.EAction.Added:
					case TreeChangedEventArgs.EAction.Removed:
						{
							var affected =
								obj.DockControl != null ? new[] { obj.DockControl } :
								obj.DockPane != null ? obj.DockPane.AllContent :
								obj.Branch != null ? obj.Branch.AllContent :
								Array.Empty<DockControl>();

							// Notify 'PaneChanged' for the affected content whenever the tree is changed
							foreach (var dc in affected)
								dc.NotifyPaneChanged();

							// Notify globally that the layout has changed
							NotifyLayoutChanged();
							break;
						}
					}
				}
			}
		} = null!;
		Branch ITreeHost.Root => Root;

		// Tip: to add auto hide dockables use:
		//  m_dc.Add(new ThingUI(), EDockSite.Right).IsAutoHide = true;
		//  To have the auto hide panel not popped out on start up, make sure
		//  it isn't the current active content.

		/// <summary>Add a dockable instance to this branch at the position described by 'location'.</summary>
		internal DockPane Add(DockControl dc, int index, params EDockSite[] location)
		{
			dc = dc ?? throw new ArgumentNullException(nameof(dc), "'dockable' or 'dockable.DockControl' cannot be 'null'");

			// If no location is given, add the dockable to its default location
			return location.Length == 0
				? Add(dc, dc.DefaultDockLocation)
				: Root.Add(dc, index, location);
		}
		public DockPane Add(IDockable dockable, int index, params EDockSite[] location)
		{
			return Add(dockable.DockControl, index, location);
		}
		public DockPane Add(IDockable dockable, params EDockSite[] location)
		{
			return Add(dockable, int.MaxValue, location);
		}
		public TDockable Add2<TDockable>(TDockable dockable, int index, params EDockSite[] location)
			where TDockable : IDockable
		{
			Add(dockable, index, location);
			return dockable;
		}
		public TDockable Add2<TDockable>(TDockable dockable, params EDockSite[] location)
			where TDockable : IDockable
		{
			return Add2(dockable, int.MaxValue, location);
		}

		/// <summary>Add the dockable to the dock container, auto hide panel, or float window, as described by 'loc'</summary>
		internal DockPane Add(DockControl dc, DockLocation loc)
		{
			dc = dc ?? throw new ArgumentNullException(nameof(dc), "'dockable' or 'dockable.DockControl' cannot be 'null'");
			loc ??= dc.DefaultDockLocation;

			DockPane pane;
			if (loc.FloatingWindowId != null)
			{
				// If a floating window id is given, locate the floating window and add to that
				var id = loc.FloatingWindowId.Value;
				var fw = FloatingWindows.GetOrAdd(id);
				pane = fw.Add(dc, loc.Index, loc.Address);
				fw.Show();
			}
			else if (loc.AutoHide != null)
			{
				// If an auto hide site is given, add to the auto hide panel
				var side = loc.AutoHide.Value;
				pane = AutoHidePanels[side].Root.Add(dc, loc.Index, loc.Address);
				dc.DefaultDockLocation.AutoHide = loc.AutoHide;
			}
			else
			{
				// Otherwise, dock to the dock container
				pane = Root.Add(dc, loc.Index, loc.Address);
				dc.DefaultDockLocation.AutoHide = null;
			}
			return pane;
		}
		public DockPane Add(IDockable dockable, DockLocation loc)
		{
			return Add(dockable.DockControl, loc);
		}
		public TDockable Add2<TDockable>(TDockable dockable, DockLocation loc)
			where TDockable : IDockable
		{
			Add(dockable, loc);
			return dockable;
		}

		/// <summary>Remove a dockable from this dock container</summary>
		public void Remove(DockControl? dc)
		{
			// Idempotent remove
			if (dc?.DockContainer == null)
				return;

			// Throw if 'dc' is not in this container
			if (dc.DockContainer != this)
				throw new InvalidOperationException("The given dockable is not managed by this dock container");

			// The DockControl object can actually remove itself, but this method gives extra checking
			dc.Remove();
		}
		public void Remove(IDockable? dockable)
		{
			// Idempotent remove
			if (dockable == null) return;
			Remove(dockable.DockControl);
		}

		/// <summary>Raised whenever the active pane changes in this dock container or associated floating window or auto hide panel</summary>
		public event EventHandler<ActivePaneChangedEventArgs>? ActivePaneChanged
		{
			add { ActiveContentManager.ActivePaneChanged += value; }
			remove { ActiveContentManager.ActivePaneChanged -= value; }
		}

		/// <summary>Raised whenever the active content changes in this dock container or associated floating window or auto hide panel</summary>
		public event EventHandler<ActiveContentChangedEventArgs>? ActiveContentChanged
		{
			add { ActiveContentManager.ActiveContentChanged += value; }
			remove { ActiveContentManager.ActiveContentChanged -= value; }
		}

		/// <summary>Raised when a pane is added, removed, or moved within the tree of branches</summary>
		public event EventHandler<EventArgs>? LayoutChanged;
		public void NotifyLayoutChanged()
		{
			if (m_layout_changed_notify_pending) return;
			m_layout_changed_notify_pending = true;
			Dispatcher.BeginInvoke(new Action(() =>
			{
				// Close any floating windows that no longer have content
				FloatingWindows?.PurgeCachedFloatingWindows();

				LayoutChanged?.Invoke(this, EventArgs.Empty);
				m_layout_changed_notify_pending = false;
			}));
		}
		private bool m_layout_changed_notify_pending;

		/// <summary>Initiate dragging of a pane or content</summary>
		internal static void DragBegin(DockControl draggee, Point ss_start_pt)
		{
			// Create a form for displaying the dock site locations and handling the drop of a pane or content
			new DragHandler(draggee, ss_start_pt).ShowDialog();
		}
		internal static void DragBegin(DockPane draggee, Point ss_start_pt)
		{
			// Create a form for displaying the dock site locations and handling the drop of a pane or content
			new DragHandler(draggee, ss_start_pt).ShowDialog();
		}

		/// <summary>The floating windows associated with this dock container</summary>
		public FloatingWindowCollection FloatingWindows
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>Panels that auto hide when their contained tree does not contain the active pane</summary>
		public AutoHidePanelCollection AutoHidePanels
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>A command to load a UI layout</summary>
		public ICommand CmdLoadLayout { get; }
		private void CmdLoadLayoutInternal()
		{
			// Prompt for a layout file
			var fd = new OpenFileDialog { Title = "Load layout", Filter = Util.FileDialogFilter("Layout Files", "*.xml") };
			if (fd.ShowDialog(Window.GetWindow(this)) != true)
				return;

			// Load the layout
			try { LoadLayout(XDocument.Load(fd.FileName).Root); }
			catch (Exception ex)
			{
				MessageBox.Show(Window.GetWindow(this), $"Layout could not be loaded\r\n{ex.Message}", "Load Layout Failed", MessageBoxButton.OK, MessageBoxImage.Error);
			}
		}

		/// <summary>A command to save the UI layout</summary>
		public ICommand CmdSaveLayout { get; private set; }
		private void CmdSaveLayoutInternal()
		{
			var fd = new SaveFileDialog { Title = "Save layout", Filter = Util.FileDialogFilter("Layout Files", "*.xml"), FileName = "Layout.xml", DefaultExt = "XML" };
			if (fd.ShowDialog(Window.GetWindow(this)) != true)
				return;

			try { SaveLayout().Save(fd.FileName); }
			catch (Exception ex)
			{
				MessageBox.Show(Window.GetWindow(this), $"Layout could not be saved\r\n{ex.Message}", "Save Layout Failed", MessageBoxButton.OK, MessageBoxImage.Error);
			}
		}

		/// <summary>A command to reset the UI layout</summary>
		public ICommand CmdResetLayout { get; private set; }
		private void CmdResetLayoutInternal()
		{
			ResetLayout();
		}

		/// <summary>Creates an instance of a menu with options to select specific content</summary>
		public MenuItem WindowsMenu(string menu_name = "Windows")
		{
			var menu = new MenuItem { Header = menu_name };
			menu.Click += RepopulateMenu;
			return menu;
		}

		/// <summary>Creates an instance of a menu with options to select specific content</summary>
		public ContextMenu WindowsCMenu()
		{
			var cmenu = new ContextMenu();
			cmenu.Opened += RepopulateMenu;
			return cmenu;
		}

		/// <summary>Handler for dynamically populating a menu item or context menu with the known panes</summary>
		private void RepopulateMenu(object? s, EventArgs a)
		{
			var items =
				s is ContextMenu cmenu ? cmenu.Items :
				s is MenuItem menu ? menu.Items :
				throw new Exception("Unsupported menu type");

			items.Clear();

			// Add a menu item for each content
			foreach (var content in AllContentInternal.OrderBy(x => x.TabText))
			{
				var header = new StackPanel { Orientation = Orientation.Horizontal, MaxHeight = 22 };
				header.Children.Add(new Image { Source = content.TabIcon, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(2) });
				header.Children.Add(new TextBlock { Text = content.TabText, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(3,0,0,0) });

				var opt = items.Add2(new MenuItem
				{
					Header = header,
					IsChecked = content == ActiveContent,
				});
				opt.Click += (ss, aa) =>
				{
					FindAndShow(content);
				};

				// If the content has a tab context menu, show the context menu on right click on the menu option
				if (content.TabCMenu != null)
				{
					opt.MouseDown += HandleMouseDown;
					void HandleMouseDown(object? ss, MouseButtonEventArgs aa)
					{
						if (aa.RightButton != MouseButtonState.Pressed) return;
						content.TabCMenu.IsOpen = true;
					}
				}
			}

			// Add a separator, then the layout items
			items.Add(new Separator());
			items.Add(new MenuItem { Header = "Load Layout", Command = CmdLoadLayout });
			items.Add(new MenuItem { Header = "Save Layout", Command = CmdSaveLayout });
			items.Add(new MenuItem { Header = "Reset Layout", Command = CmdResetLayout });
		}

		/// <summary>Make 'content' visible to the user, by making it's containing window visible, on-screen, popped-out, etc</summary>
		public void FindAndShow(IDockable content)
		{
			FindAndShow(content.DockControl);
		}
		public void FindAndShow(DockControl dc)
		{
			if (dc.DockContainer != this)
				throw new Exception($"Cannot show content '{dc.TabText}', it is not managed by this DockContainer");

			// Find the window that contains 'content'
			var pane = dc.DockPane;
			if (pane == null)
			{
				// Restore 'content' to the dock container
				var address = dc.DockAddressFor(this);
				Add(dc, int.MaxValue, address);
				pane = dc.DockPane;
				if (pane == null)
					throw new Exception($"Cannot show content '{dc.TabText}'. Could not add it to a visible dock pane");
			}

			// Make the selected item the visible item on the pane
			pane.VisibleContent = dc;

			// Make it the active item in the dock container
			if (pane.TreeHost != null)
			{
				ActiveContentManager.ActiveContent = dc;

				// If the container is an auto hide panel, make sure it's popped out
				if (pane.TreeHost is AutoHidePanel ah)
				{
					ah.PoppedOut = true;
				}

				// If the container is a floating window, make sure it's visible and on-screen
				if (pane.TreeHost is FloatingWindow fw)
				{
					fw.Bounds = Gui_.OnScreen(fw.Bounds);
					fw.Visibility = Visibility.Visible;
					if (fw.WindowState == WindowState.Minimized)
						fw.WindowState = WindowState.Normal;
				}
			}
		}
		public void FindAndShow(Type type)
		{
			foreach (var item in AllContent.OfType(type).Cast<IDockable>())
				FindAndShow(item);
		}

		/// <summary>Get the bounds of a dock site. 'rect' is the available area. 'docked_mask' are the visible dock sites within 'rect'</summary>
		internal static Rect DockSiteBounds(EDockSite docksite, Rect rect, EDockMask docked_mask, DockSizeData dock_site_sizes)
		{
			// Get the size of each dock site relative to 'rect'
			var site_size = dock_site_sizes.GetSizesForRect(rect, docked_mask | (EDockMask)(1 << (int)docksite));
			var r = docksite switch
			{
				EDockSite.Centre => rect,
				EDockSite.Left   => new Rect(rect.Left, rect.Top, site_size.Left, rect.Height),
				EDockSite.Right  => new Rect(rect.Right - site_size.Right, rect.Top, site_size.Right, rect.Height),
				EDockSite.Top    => new Rect(rect.Left, rect.Top, rect.Width, site_size.Top),
				EDockSite.Bottom => new Rect(rect.Left, rect.Bottom - site_size.Bottom, rect.Width, site_size.Bottom),
				_ => throw new Exception($"No bounds value for dock zone {docksite}"),
			};

			// Remove areas from 'r' for sites with EDockSite values greater than 'location' (if present).
			// Note: order of subtracting areas is important, subtract from highest priority to lowest
			// so that the result is always still a rectangle.
			for (var i = (EDockSite)DockSiteCount - 1; i > docksite; --i)
			{
				if (!docked_mask.HasFlag((EDockMask)(1 << (int)i))) continue;
				var sub = DockSiteBounds(i, rect, docked_mask, dock_site_sizes);
				r = Rect_.Subtract(r, sub);
				if (r.Width < 0) r.Width = 0;
				if (r.Height < 0) r.Height = 0;
			}

			// Return the bounds of the dock zone for 'location'
			return r;
		}

		/// <summary>Convert a dock site to a dock panel dock location</summary>
		internal static Dock ToDock(EDockSite ds)
		{
			return ds switch
			{
				EDockSite.Left   => Dock.Left,
				EDockSite.Right  => Dock.Right,
				EDockSite.Top    => Dock.Top,
				EDockSite.Bottom => Dock.Bottom,
				_ => throw new Exception($"Cannot convert {ds} to a DockPanel 'Dock' value"),
			};
		}

		/// <summary>Save state to XML</summary>
		public XElement ToXml(XElement node)
		{
			// Loop through the floating windows assigning an incrementing ID to each one with content
			var id = 0;
			foreach (var fw in FloatingWindows.Where(x => !x.Root.Empty))
				fw.Id = id++;

			// Record the version number of this data format
			node.Add2(XmlTag.Version, 1, false);

			// Record the state of the hosted content
			var content_node = node.Add2(new XElement(XmlTag.Contents));
			foreach (var content in AllContentInternal)
				content_node.Add2(XmlTag.Content, content, false);

			// Record the name of the active content
			if (ActiveContent != null)
				node.Add2(XmlTag.Active, ActiveContent.PersistName, false);

			// Record the structure of the main tree
			node.Add2(XmlTag.Tree, Root, false);

			// Record position information for the floating windows
			var fw_node = node.Add2(new XElement(XmlTag.FloatingWindows));
			foreach (var fw in FloatingWindows.Where(x => !x.Root.Empty))
				fw_node.Add2(XmlTag.FloatingWindow, fw, false);

			return node;
		}

		/// <summary>Record the layout of this dock container</summary>
		public XElement SaveLayout()
		{
			return ToXml(new XElement(XmlTag.DockContainerLayout));
		}

		/// <summary>
		/// Update the layout of this dock container using the data in 'node'
		/// Use 'content_factory' to create content on demand during loading.
		/// Signature: 'content_factory(string persist_name, string type_name, XElement user_data)'</summary>
		public void LoadLayout(XElement? node, Func<string, string, XElement, DockControl?>? content_factory = null)
		{
			// Null is allowed to make first-run scenarios easier
			if (node == null)
				return;
			if (node.Name != XmlTag.DockContainerLayout)
				throw new Exception("XML data does not contain dock container layout information");

			// Get a collection of the content currently loaded
			var all_content = AllContentInternal.ToDictionary(x => x.PersistName, x => x);

			// For each content node find a content object with matching PersistName and
			// move it to the location indicated in the saved state.
			foreach (var content_node in node.Elements(XmlTag.Contents, XmlTag.Content))
			{
				// Find a content object with a matching persistence name
				var name = content_node.Element(XmlTag.Name).As<string>(string.Empty);
				var type = content_node.Element(XmlTag.Type).As<string>(string.Empty);
				var udat = content_node.Element(XmlTag.UserData) ?? new XElement(XmlTag.UserData);

				DockControl? content;
				if (all_content.TryGetValue(name, out content) || (content = content_factory?.Invoke(name, type, udat)) != null)
				{
					var loc = new DockLocation(content_node);
					if (loc.Address.First() != EDockSite.None)
						Add(content, loc);
					else
						content.DockPane = null;

					// Inform the dock control that layout is being loaded
					content.LoadLayout(udat);
				}
			}

			// Load the tree layout
			var tree_node = node.Element(XmlTag.Tree);
			if (tree_node != null)
				Root.ApplyState(tree_node);

			// Apply layout to floating windows
			foreach (var fw_node in node.Elements(XmlTag.FloatingWindows, XmlTag.FloatingWindow))
			{
				var id = fw_node.Element(XmlTag.Id).As<int>();
				var fw = FloatingWindows.GetOrAdd(id);
				if (fw != null)
					fw.ApplyState(fw_node);
			}

			// Restore the active content
			var active = node.Element(XmlTag.Active)?.As<string>();
			if (active != null && all_content.TryGetValue(active, out var active_content))
			{
				if (active_content.DockPane != null)
					ActiveContent = active_content;
			}
		}

		/// <summary>Move all content to it's default dock address</summary>
		public void ResetLayout()
		{
			// Adding each dockable without an address causes it to be moved to its default location
			foreach (var dc in AllContentInternal.ToArray())
				Add(dc, dc.DefaultDockLocation);
		}

		/// <summary>Self consistency check (for debugging)</summary>
		public bool ValidateTree()
		{
			try
			{
				Root.ValidateTree();
				foreach (var ah in AutoHidePanels)
				{
					if (ah.Root.Descendants[EDockSite.Left].Item != null ||
						ah.Root.Descendants[EDockSite.Top].Item != null ||
						ah.Root.Descendants[EDockSite.Right].Item != null ||
						ah.Root.Descendants[EDockSite.Bottom].Item != null)
						throw new Exception($"Auto hide panel {ah.DockSite} has items not in the centre pane");

					ah.Root.ValidateTree();
				}
				foreach (var fw in FloatingWindows)
				{
					fw.Root.ValidateTree();
				}

				return true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine(ex.MessageFull());
				return false;
			}
		}

		/// <summary>Output the tree structure</summary>
		public string DumpTree()
		{
			var sb = new StringBuilder();
			Root.DumpTree(sb, 0, string.Empty);
			return sb.ToString();
		}

		public sealed class AutoHidePanelCollection : IDisposable, IEnumerable<AutoHidePanel>
		{
			public AutoHidePanelCollection(DockContainer dc, Canvas centre)
			{
				m_auto_hide = new AutoHidePanel[DockSiteCount];
				for (var i = (int)EDockSite.Left; i != m_auto_hide.Length; ++i)
				{
					var ds = (EDockSite)i;
					var ah = new AutoHidePanel(dc, ds);

					// Add the auto hide panel to the centre canvas
					m_auto_hide[i] = centre.Children.Add2(ah);

					// Add the associated tab strip to the top-level dock panel
					dc.Children.Add2(ah.TabStrip);
					DockContainer.SetDock(ah.TabStrip, DockContainer.ToDock(ds));

					// Position the auto hide panels within 'centre'
					// The auto hide panel is a 3 column/row grid, where the centre column/row is the grid
					// splitter and one panel is the dock pane, the other is empty and transparent.
					// So, all auto hide panels span the entire centre canvas.
					ah.SetBinding(AutoHidePanel.WidthProperty, new Binding(nameof(Canvas.ActualWidth)) { Source = centre });
					ah.SetBinding(AutoHidePanel.HeightProperty, new Binding(nameof(Canvas.ActualHeight)) { Source = centre });
				}
				Canvas.SetLeft(m_auto_hide[(int)EDockSite.Left], 0);
				Canvas.SetTop(m_auto_hide[(int)EDockSite.Left], 0);
				Canvas.SetLeft(m_auto_hide[(int)EDockSite.Top], 0);
				Canvas.SetTop(m_auto_hide[(int)EDockSite.Top], 0);
				Canvas.SetRight(m_auto_hide[(int)EDockSite.Right], 0);
				Canvas.SetTop(m_auto_hide[(int)EDockSite.Right], 0);
				Canvas.SetLeft(m_auto_hide[(int)EDockSite.Bottom], 0);
				Canvas.SetBottom(m_auto_hide[(int)EDockSite.Bottom], 0);
			}
			public void Dispose()
			{
				Util.DisposeAll(m_auto_hide);
			}

			/// <summary>Get the auto hide panel by dock site</summary>
			public AutoHidePanel this[EDockSite ds]
			{
				get
				{
					if (ds >= EDockSite.Left && ds < EDockSite.None)
						return m_auto_hide[(int)ds];
					else
						throw new Exception($"No auto hide panel for dock site {ds}");
				}
			}
			private readonly AutoHidePanel[] m_auto_hide;

			#region IEnumerable
			public IEnumerator<AutoHidePanel> GetEnumerator()
			{
				return m_auto_hide.Skip(1).GetEnumerator();
			}
			IEnumerator IEnumerable.GetEnumerator()
			{
				return GetEnumerator();
			}
			#endregion
		}
		public sealed class FloatingWindowCollection :IDisposable, IEnumerable<FloatingWindow>
		{
			private readonly DockContainer m_dc;
			private readonly List<FloatingWindow> m_floaters;
			public FloatingWindowCollection(DockContainer dc)
			{
				m_dc = dc;
				m_floaters = new List<FloatingWindow>();
			}
			public void Dispose()
			{
				foreach (var fw in m_floaters) fw.Close();
				m_floaters.Clear();
			}

			/// <summary>Return the floating window with Id = 'id'</summary>
			public FloatingWindow Get(int id)
			{
				return m_floaters.First(x => x.Id == id);
			}

			/// <summary>Returns an existing floating window with id 'id', or a new floating window with the Id set to 'id'</summary>
			public FloatingWindow GetOrAdd(int id)
			{
				var fw = GetOrAdd(x => x.Id == id);
				fw.Id = id;
				return fw;
			}

			/// <summary>Returns an existing floating window that satisfies 'pred', or a new floating window</summary>
			public FloatingWindow GetOrAdd(Func<FloatingWindow, bool> pred)
			{
				var fw = m_floaters.FirstOrDefault(pred);
				fw ??= m_floaters.Add2(new FloatingWindow(m_dc));
				return fw;
			}

			/// <summary>Remove a floating window</summary>
			public bool Remove(FloatingWindow win)
			{
				return m_floaters.Remove(win);
			}

			/// <summary>Release floating windows that have no content</summary>
			public void PurgeCachedFloatingWindows()
			{
				foreach (var fw in m_floaters.ToArray())
				{
					if (fw.AllContent.Any()) continue;
					m_floaters.Remove(fw);
					fw.Close();
				}
			}

			#region IEnumerable
			public IEnumerator<FloatingWindow> GetEnumerator()
			{
				return m_floaters.GetEnumerator();
			}
			IEnumerator IEnumerable.GetEnumerator()
			{
				return GetEnumerator();
			}
			#endregion
		}
	}
}
