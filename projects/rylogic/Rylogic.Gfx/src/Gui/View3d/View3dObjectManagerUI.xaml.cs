using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	/// <summary>A flat row in the virtualised tree list, wrapping a View3d.Object.</summary>
	public sealed class FlatTreeNode : INotifyPropertyChanged
	{
		public FlatTreeNode(View3d.Object obj, int depth, bool has_children)
		{
			Object = obj;
			Depth = depth;
			HasChildren = has_children;
			IsExpanded = false;
		}

		/// <summary>The underlying scene object</summary>
		public View3d.Object Object { get; }

		/// <summary>Nesting depth (0 = root)</summary>
		public int Depth { get; }

		/// <summary>True if this node has child objects</summary>
		public bool HasChildren { get; }

		/// <summary>Indent margin for the Name column, based on depth</summary>
		public Thickness Indent => new(Depth * 10, 0, 0, 0);

		/// <summary>Expand/collapse state</summary>
		public bool IsExpanded
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(IsExpanded));
			}
		}

		// Forwarded properties from View3d.Object
		public string Name => Object.Name;
		public string Type => Object.Type;
		public bool Visible
		{
			get => Object.Visible;
			set { Object.Visible = value; NotifyPropertyChanged(nameof(Visible)); }
		}
		public View3d.ELdrFlags Flags => Object.Flags;
		public string Colour => Object.Colour.ToString();
		public Colour32 ColourValue => Object.Colour;

		/// <summary>Refresh forwarded properties after the underlying object changes</summary>
		public void Refresh()
		{
			NotifyPropertyChanged(nameof(Name));
			NotifyPropertyChanged(nameof(Type));
			NotifyPropertyChanged(nameof(Visible));
			NotifyPropertyChanged(nameof(Flags));
			NotifyPropertyChanged(nameof(Colour));
			NotifyPropertyChanged(nameof(ColourValue));
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}

	public partial class View3dObjectManagerUI : UserControl, IDisposable, INotifyPropertyChanged
	{
		public View3dObjectManagerUI(View3d.Window window, IEnumerable<Guid>? exclude = null)
		{
			InitializeComponent();

			ObjectManager = new View3d.ObjectManager(window, exclude ?? Array.Empty<Guid>());
			FlatNodes = [];
			VisibleNodes = new ListCollectionView(FlatNodes);

			ApplyFilter = Command.Create(this, ApplyFilterInternal);
			SetVisibility = Command.Create(this, SetVisibilityInternal);
			SetWireframe = Command.Create(this, SetWireframeInternal);
			ToggleExpand = Command.Create(this, ToggleExpandInternal);
			ExpandAll = Command.Create(this, ExpandAllInternal);
			CollapseAll = Command.Create(this, CollapseAllInternal);
			InvertSelection = Command.Create(this, InvertSelectionInternal);
			ToggleShowNormals = Command.Create(this, ToggleShowNormalsInternal);
			FocusPatternFilter = Command.Create(this, FocusPatternFilterInternal);
			ShowMoveObjects = Command.Create(this, ShowMoveObjectsInternal);

			DataContext = this;
			RebuildFlatList();

			// Wire up column header right-click for column visibility toggling
			m_list.AddHandler(GridViewColumnHeader.LoadedEvent, new RoutedEventHandler(OnColumnHeaderLoaded));
		}
		public void Dispose()
		{
			ObjectManager = null!;
			GC.SuppressFinalize(this);
		}

		/// <summary>Saved widths for hidden columns, keyed by column header text</summary>
		private readonly Dictionary<string, double> m_hidden_column_widths = [];

		/// <summary>Attach a context menu to each column header for toggling column visibility</summary>
		private void OnColumnHeaderLoaded(object sender, RoutedEventArgs e)
		{
			if (e.OriginalSource is not GridViewColumnHeader header || header.Column == null) return;
			if (m_list.View is not GridView grid_view) return;

			header.ContextMenu = CreateColumnVisibilityCMenu(grid_view);
		}

		/// <summary>Create a context menu listing all columns with checkboxes to show/hide them</summary>
		private ContextMenu CreateColumnVisibilityCMenu(GridView grid_view)
		{
			var cmenu = new ContextMenu();
			foreach (var col in grid_view.Columns)
			{
				var header_text = col.Header as string ?? "?";
				var item = new MenuItem
				{
					Header = header_text,
					IsCheckable = true,
					IsChecked = col.Width != 0,
					Tag = col,
				};
				item.Checked += (s, a) =>
				{
					if (item.Tag is GridViewColumn c && m_hidden_column_widths.TryGetValue(c.Header as string ?? "", out var w))
						c.Width = w;
				};
				item.Unchecked += (s, a) =>
				{
					if (item.Tag is GridViewColumn c)
					{
						m_hidden_column_widths[c.Header as string ?? ""] = c.ActualWidth;
						c.Width = 0;
					}
				};
				cmenu.Items.Add(item);
			}
			return cmenu;
		}

		/// <summary>The view model for the object manager behaviour</summary>
		public View3d.ObjectManager ObjectManager
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandlePropertyChanged;
					View3d.Object.ObjectChanged -= HandleObjectPropertyChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					View3d.Object.ObjectChanged += HandleObjectPropertyChanged;
					field.PropertyChanged += HandlePropertyChanged;
				}

				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					if (e.PropertyName == nameof(View3d.ObjectManager.Objects))
						RebuildFlatList();
				}
				void HandleObjectPropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					if (sender is not View3d.Object obj)
						return;

					// Refresh matching nodes or nodes that have 'obj' as a parent
					foreach (var node in FlatNodes)
					{
						if (node.Object == obj)
							node.Refresh();
						else if (obj.IsDescendant(node.Object))
							node.Refresh();
					}

					// If an object within this window has changed, refresh the 3D view
					if (ObjectManager.Window.HasObject(obj, search_children: true))
						ObjectManager.Window.Invalidate();
				}
			}
		} = null!;

		/// <summary>The flat list of all nodes (expanded tree)</summary>
		private ObservableCollection<FlatTreeNode> FlatNodes { get; }

		/// <summary>The filtered/visible view bound to the ListView</summary>
		public ICollectionView VisibleNodes { get; private set; }

		/// <summary>Access to the object container</summary>
		public IList<View3d.Object> RootObjects => ObjectManager.Objects;

		/// <summary>Selected nodes from the ListView</summary>
		public HashSet<FlatTreeNode> SelectedNodes { get; } = [];

		/// <summary>Selected objects (for visibility/wireframe commands)</summary>
		public HashSet<View3d.Object> SelectedObjects => ObjectManager.SelectedObjects;

		/// <summary>The first selected object (for property binding)</summary>
		public View3d.Object? FirstSelected => SelectedObjects.FirstOrDefault();

		/// <summary>Refresh the 3D scene</summary>
		private void Invalidate() => ObjectManager.Window.Invalidate();

		/// <summary>The current filter pattern (null = no filter)</summary>
		private Pattern? m_filter;

		#region Flat list management

		/// <summary>Rebuild the flat node list from the object tree</summary>
		private void RebuildFlatList()
		{
			// Save expanded state
			var expanded = new HashSet<IntPtr>();
			foreach (var node in FlatNodes)
			{
				if (node.IsExpanded)
					expanded.Add(node.Object.Handle);
			}

			FlatNodes.Clear();
			foreach (var obj in RootObjects)
				AddNodeRecursive(obj, 0, expanded);

			ApplyFilterToNodes();
		}

		/// <summary>Recursively add nodes for an object and its children</summary>
		private void AddNodeRecursive(View3d.Object obj, int depth, HashSet<IntPtr>? expanded)
		{
			var children = obj.Children;
			var has_children = children.Count != 0;
			var node = new FlatTreeNode(obj, depth, has_children);

			// Restore expanded state
			if (expanded != null && expanded.Contains(obj.Handle))
				node.IsExpanded = true;

			FlatNodes.Add(node);

			// Only add children if expanded
			if (node.IsExpanded)
			{
				foreach (var child in children)
					AddNodeRecursive(child, depth + 1, expanded);
			}
		}

		#endregion

		#region Expand / Collapse

		/// <summary>Toggle expand/collapse for a node</summary>
		public Command ToggleExpand { get; }
		private void ToggleExpandInternal(object? parameter)
		{
			if (parameter is not FlatTreeNode node || !node.HasChildren) return;

			if (node.IsExpanded)
				CollapseNode(node);
			else
				ExpandNode(node);
		}

		private void ExpandNode(FlatTreeNode node)
		{
			if (!node.HasChildren || node.IsExpanded) return;
			node.IsExpanded = true;

			// Insert children after this node
			var index = FlatNodes.IndexOf(node) + 1;
			foreach (var child_obj in node.Object.Children)
			{
				var child_children = child_obj.Children;
				var child_node = new FlatTreeNode(child_obj, node.Depth + 1, child_children.Count != 0);
				FlatNodes.Insert(index++, child_node);
			}

			ApplyFilterToNodes();
		}

		private void CollapseNode(FlatTreeNode node)
		{
			if (!node.IsExpanded) return;
			node.IsExpanded = false;

			// Remove all descendants (nodes immediately after with greater depth)
			var index = FlatNodes.IndexOf(node) + 1;
			while (index < FlatNodes.Count && FlatNodes[index].Depth > node.Depth)
				FlatNodes.RemoveAt(index);

			ApplyFilterToNodes();
		}

		/// <summary>Expand all nodes</summary>
		public Command ExpandAll { get; }
		private void ExpandAllInternal()
		{
			// Rebuild with everything expanded
			var all_expanded = new HashSet<IntPtr>();
			void CollectAll(View3d.Object obj)
			{
				all_expanded.Add(obj.Handle);
				foreach (var child in obj.Children)
					CollectAll(child);
			}
			foreach (var obj in RootObjects)
				CollectAll(obj);

			FlatNodes.Clear();
			foreach (var obj in RootObjects)
				AddNodeRecursive(obj, 0, all_expanded);

			ApplyFilterToNodes();
		}

		/// <summary>Collapse all nodes</summary>
		public Command CollapseAll { get; }
		private void CollapseAllInternal()
		{
			FlatNodes.Clear();
			foreach (var obj in RootObjects)
				AddNodeRecursive(obj, 0, null);

			ApplyFilterToNodes();
		}

		#endregion

		#region Filter

		/// <summary>Apply filter pattern</summary>
		public Command ApplyFilter { get; }
		private void ApplyFilterInternal(object? parameter)
		{
			if (parameter is Pattern pattern && pattern.Expr.Length != 0 && pattern.IsValid)
				m_filter = pattern;
			else
				m_filter = null;

			// When a filter is applied, expand all so that matching descendants are discoverable
			if (m_filter != null)
				ExpandAllInternal();

			ApplyFilterToNodes();
		}

		/// <summary>
		/// Apply the current filter to the flat list. Matching nodes and their ancestors are visible.
		/// </summary>
		private void ApplyFilterToNodes()
		{
			if (m_filter == null)
			{
				VisibleNodes.Filter = null;
				return;
			}

			// Mark which nodes should be visible (match or ancestor of a match)
			var visible = new HashSet<int>();
			for (int i = 0; i != FlatNodes.Count; ++i)
			{
				if (!m_filter.IsMatch(FlatNodes[i].Name))
					continue;

				// This node matches — mark it and all its ancestors visible
				visible.Add(i);

				// Walk backward to find ancestors (nodes with strictly decreasing depth)
				var target_depth = FlatNodes[i].Depth - 1;
				for (int j = i - 1; j >= 0 && target_depth >= 0; --j)
				{
					if (FlatNodes[j].Depth == target_depth)
					{
						visible.Add(j);
						--target_depth;
					}
				}
			}

			VisibleNodes.Filter = obj =>
			{
				if (obj is not FlatTreeNode node) return false;
				var idx = FlatNodes.IndexOf(node);
				return visible.Contains(idx);
			};
		}

		#endregion

		#region Visibility commands

		/// <summary>Show/Hide objects</summary>
		public Command SetVisibility { get; }
		private void SetVisibilityInternal(object? parameter)
		{
			if (parameter is not ESetVisibleCmd vis)
				throw new Exception($"SetVisible parameter '{parameter}' is invalid");

			switch (vis)
			{
				case ESetVisibleCmd.ShowAll:
				case ESetVisibleCmd.HideAll:
				{
					foreach (var obj in RootObjects)
						obj.FlagsSet(View3d.ELdrFlags.Hidden, vis == ESetVisibleCmd.HideAll, string.Empty);
					break;
				}
				case ESetVisibleCmd.ShowSelected:
				case ESetVisibleCmd.HideSelected:
				case ESetVisibleCmd.ToggleSelected:
				{
					foreach (var obj in SelectedObjects)
					{
						var hidden =
							vis == ESetVisibleCmd.ShowSelected ? false :
							vis == ESetVisibleCmd.HideSelected ? true :
							!obj.Flags.HasFlag(View3d.ELdrFlags.Hidden);

						obj.FlagsSet(View3d.ELdrFlags.Hidden, hidden, null);
					}
					break;
				}
				case ESetVisibleCmd.ShowOthers:
				case ESetVisibleCmd.HideOthers:
				case ESetVisibleCmd.ToggleOthers:
				{
					foreach (var obj in RootObjects)
					{
						obj.Apply(x =>
						{
							if (SelectedObjects.Contains(x)) return true;
							var hidden =
								vis == ESetVisibleCmd.ShowOthers ? false :
								vis == ESetVisibleCmd.HideOthers ? true :
								!obj.Flags.HasFlag(View3d.ELdrFlags.Hidden);

							x.FlagsSet(View3d.ELdrFlags.Hidden, hidden, null);
							return true;
						}, string.Empty);
					}
					break;
				}
				default:
				{
					throw new Exception($"Unknown visibility command {vis}");
				}
			}
			Invalidate();
		}

		/// <summary>Switch between wireframe and solid</summary>
		public Command SetWireframe { get; }
		private void SetWireframeInternal(object? parameter)
		{
			if (parameter is string wf && int.TryParse(wf, out var wireframe))
			{
				foreach (var x in SelectedObjects)
				{
					var wire = wireframe == +1 || (wireframe == 0 && !Bit.AllSet(x.Flags, View3d.ELdrFlags.Wireframe));
					x.FlagsSet(View3d.ELdrFlags.Wireframe, wire, string.Empty);
				}
				Invalidate();
				return;
			}
			throw new Exception($"SetWireframe parameter '{parameter}' is invalid. Expected +1, 0, or -1");
		}

		#endregion

		#region Selection

		/// <summary>Handle selection changes from the ListView</summary>
		public void HandleSelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			foreach (var item in e.RemovedItems)
			{
				if (item is FlatTreeNode node)
				{
					node.Object.Flags = Bit.SetBits(node.Object.Flags, View3d.ELdrFlags.Selected, false);
					SelectedObjects.Remove(node.Object);
					SelectedNodes.Remove(node);
				}
			}
			foreach (var item in e.AddedItems)
			{
				if (item is FlatTreeNode node)
				{
					node.Object.Flags = Bit.SetBits(node.Object.Flags, View3d.ELdrFlags.Selected, true);
					SelectedObjects.Add(node.Object);
					SelectedNodes.Add(node);
				}
			}

			NotifyPropertyChanged(nameof(FirstSelected));
			Invalidate();
		}

		/// <summary>Invert the selection status of each object</summary>
		public Command InvertSelection { get; }
		private void InvertSelectionInternal()
		{
			HashSet<View3d.Object> inverted = [];
			foreach (var obj in RootObjects)
			{
				obj.Apply(x =>
				{
					x.FlagsSet(View3d.ELdrFlags.Selected, !SelectedObjects.Contains(x), string.Empty);
					if (!SelectedObjects.Contains(x)) inverted.Add(x);
					return true;
				}, string.Empty);
			}
			SelectedObjects.Clear();
			SelectedObjects.UnionWith(inverted);
		}

		/// <summary>Toggle show normals mode on selected objects</summary>
		public Command ToggleShowNormals { get; }
		private void ToggleShowNormalsInternal()
		{
			bool? show = null;
			foreach (var obj in SelectedObjects)
			{
				show = show ?? !obj.ShowNormals;
				obj.ShowNormals = show.Value;
			}
			Invalidate();
		}

		/// <summary>Show the Move Objects tool window</summary>
		public Command ShowMoveObjects { get; }
		private void ShowMoveObjectsInternal()
		{
			if (SelectedObjects.Count == 0) return;

			// Snapshot the selected objects at launch time — the Move UI is independent from here on
			var objects = SelectedObjects.ToList();
			var ui = new View3dMoveObjectsUI(Window.GetWindow(this), objects, ObjectManager.Window);
			ui.Show();
			ui.Focus();
		}

		#endregion

		/// <summary>Move input focus to the filter bar</summary>
		public Command FocusPatternFilter { get; }
		private void FocusPatternFilterInternal()
		{
			if (FindName("m_pattern_filter") is FrameworkElement pattern_filter &&
				pattern_filter.FindName("PART_TextBox") is TextBox text_box)
			{
				text_box.Focus();
				text_box.SelectAll();
			}
		}

		/// <summary>Handles the eye toggle button click to force a 3D scene redraw</summary>
		private void OnEyeToggleClick(object sender, RoutedEventArgs e)
		{
			Invalidate();
		}

		/// <summary>Handle double-click on the colour cell to show a colour picker</summary>
		private void OnColourDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (e.ClickCount != 2) return;
			if (sender is not FrameworkElement fe || fe.DataContext is not FlatTreeNode node) return;

			var picker = new ColourPickerUI(System.Windows.Window.GetWindow(this), node.Object.Colour);
			picker.ColorChanged += (s, args) =>
			{
				node.Object.ColourSet(args.Colour);
				node.Refresh();
				Invalidate();
			};
			picker.ShowDialog();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}

	/// <summary>Set visibility commands</summary>
	public enum ESetVisibleCmd
	{
		ShowAll,
		HideAll,
		ShowSelected,
		HideSelected,
		ToggleSelected,
		ShowOthers,
		HideOthers,
		ToggleOthers,
	}
}
