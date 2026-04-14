using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Xml.Linq;
using Rylogic.Extn;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.DockContainerDetail
{
	/// <summary>A floating window that hosts a tree of dock panes</summary>
	[DebuggerDisplay("FloatingWindow")]
	public partial class FloatingWindow : Window, ITreeHost, IPinnable
	{
		public FloatingWindow(DockContainer dc)
		{
			// Don't set 'Owner' to 'Window.GetWindow(dc)', 'dc' may not have an owner
			// window yet. Also, it's nice to allow floating windows behind the main window.
			InitializeComponent();
			Content = new DockPanel { LastChildFill = true };

			DockContainer = dc;
			PinState = new PinData(this, EPin.Centre, pinned: false);
			Root = new Branch(dc, DockSizeData.Quarters);

			SizeChanged += delegate { DockContainer.NotifyLayoutChanged(); };
		}
		protected override void OnClosing(CancelEventArgs e)
		{
			// Move all the content back to the main dock container
			foreach (var dc in AllContent.ToArray())
				dc.IsFloating = false;

			base.OnClosing(e);
		}
		protected override void OnClosed(EventArgs e)
		{
			Root = null!;
			DockContainer = null!;
			base.OnClosed(e);
		}

		/// <summary>An identifier for a floating window</summary>
		public int Id { get; set; }

		/// <summary>The dock container that owns this floating window</summary>
		public DockContainer DockContainer
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.ActiveContentChanged -= HandleActiveContentChanged;
					field.FloatingWindows?.Remove(this);
				}
				field = value;
				if (field != null)
				{
					field.ActiveContentChanged += HandleActiveContentChanged;
				}

				/// <summary>Handler for when the active content changes</summary>
				void HandleActiveContentChanged(object? sender, ActiveContentChangedEventArgs e)
				{
					// Update the title whenever active content changes within this floating window
					if (ActiveContentManager.ActivePane?.RootBranch == Root)
						UpdateTitle();
				}
			}
		} = null!;
		DockContainer ITreeHost.DockContainer => DockContainer;

		/// <summary>The window content as a control container</summary>
		private Panel ContentPanel => (Panel)Content;

		/// <summary>Support pinning this window</summary>
		public PinData PinState { get; }

		/// <summary>The root level branch of the tree in this floating window</summary>
		internal Branch Root
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.TreeChanged -= HandleTreeChanged;
					ContentPanel.Children.Remove(field);
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					ContentPanel.Children.Add(field);
					field.TreeChanged += HandleTreeChanged;
				}

				/// <summary>Handler for when panes are added/removed from the tree</summary>
				void HandleTreeChanged(object? sender, TreeChangedEventArgs args)
				{
					switch (args.Action)
					{
					case TreeChangedEventArgs.EAction.Added:
					case TreeChangedEventArgs.EAction.Removed:
						{
							UpdateTitle();
							DockContainer.NotifyLayoutChanged();
							break;
						}
					}
				}
			}
		} = null!;
		Branch ITreeHost.Root => Root;

		/// <summary>Manages events and changing of active pane/content</summary>
		private ActiveContentManager ActiveContentManager => DockContainer.ActiveContentManager;

		/// <summary>Enumerate the dockables in this sub-tree (breadth first, order = order of EDockSite)</summary>
		public IEnumerable<DockControl> AllContent => Root.AllContent;

		/// <summary>Update the floating window title to reflect its content</summary>
		private void UpdateTitle()
		{
			var app_name = GetWindow(DockContainer)?.Title ?? string.Empty;
			var centre_content = Root.Descendants[EDockSite.Centre]?.Item switch
			{
				DockPane pane => pane.VisibleContent?.TabText,
				Branch branch => branch.AllContent.FirstOrDefault()?.TabText,
				_ => null,
			};

			Title = centre_content != null ? $"{app_name}: {centre_content}" : app_name;
			Icon = AllContent.FirstOrDefault()?.TabIcon ?? GetWindow(DockContainer)?.Icon;
		}

		/// <summary>The current screen location and size of this window</summary>
		public Rect Bounds
		{
			get => new(Left, Top, Width, Height);
			set
			{
				Left = value.Left;
				Top = value.Top;
				Width = value.Width;
				Height = value.Height;
			}
		}

		/// <summary>Add a dockable instance to this branch at the position described by 'location'.</summary>
		internal DockPane Add(DockControl dc, int index, params EDockSite[] location)
		{
			if (dc == null)
				throw new ArgumentNullException(nameof(dc), "'dockable' or 'dockable.DockControl' cannot be 'null'");

			return Root.Add(dc, index, location);
		}
		public DockPane Add(IDockable dockable, int index, params EDockSite[] location)
		{
			return Add(dockable.DockControl, index, location);
		}
		public DockPane Add(IDockable dockable, params EDockSite[] location)
		{
			var addr = location.Length != 0 ? location : new[] { EDockSite.Centre };
			return Add(dockable, int.MaxValue, addr);
		}

		/// <summary>Save state to XML</summary>
		public XElement ToXml(XElement node)
		{
			// Save the ID assigned to this window
			node.Add2(XmlTag.Id, Id, false);

			//				// Save whether the floating window is pinned to the dock container
			//				node.Add2(XmlTag.Pinned, PinWindow, false);

			// Save the screen-space location of the floating window. If pinned, save the offset bounds
			var bnds = Bounds;
			//				if (PinWindow) bnds = bnds.Shifted(-TargetFrame.Left, -TargetFrame.Top);
			node.Add2(XmlTag.Bounds, bnds, false);

			// Save whether the floating window is shown or now
			node.Add2(XmlTag.Visible, IsVisible, false);

			// Save the tree structure of the floating window
			node.Add2(XmlTag.Tree, Root, false);
			return node;
		}

		/// <summary>Apply state to this floating window</summary>
		public void ApplyState(XElement node)
		{
			//				// Restore the pinned state
			//				var pinned = node.Element(XmlTag.Pinned)?.As<bool>();
			//				if (pinned != null)
			//					PinWindow = pinned.Value;

			// Move the floating window to the saved position (clamped by the virtual screen)
			var bounds = node.Element(XmlTag.Bounds)?.As<Rect>();
			if (bounds != null)
			{
				// If 'PinWindow' is set, then the bounds are relative to the parent window
				var bnds = bounds.Value;
				//					if (PinWindow) bnds = bnds.Shifted(TargetFrame.Left, TargetFrame.Top);
				Bounds = Gui_.OnScreen(bnds);
			}

			// Update the tree layout
			var tree_node = node.Element(XmlTag.Tree);
			if (tree_node != null)
				Root.ApplyState(tree_node);

			// Restore visibility
			var visible = node.Element(XmlTag.Visible)?.As<bool>();
			if (visible != null)
				Visibility = visible.Value ? Visibility.Visible : Visibility.Collapsed;
		}
	}
}
