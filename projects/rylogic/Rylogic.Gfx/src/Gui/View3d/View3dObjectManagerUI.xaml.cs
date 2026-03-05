using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class View3dObjectManagerUI : Window, INotifyPropertyChanged
	{
		public View3dObjectManagerUI(Window owner, View3d.Window window, IEnumerable<Guid>? exclude = null)
		{
			InitializeComponent();
			Owner = owner;
			Icon = Owner?.Icon;

			PinState = new PinData(this, EPin.Centre, pinned: false);
			ObjectManager = new View3d.ObjectManager(window, exclude ?? Array.Empty<Guid>());
			ObjectsView = new ListCollectionView(ObjectManager.Objects);
			ObjectsView.SortDescriptions.Add(new SortDescription(nameof(View3d.Object.Name), ListSortDirection.Ascending));

			ApplyFilter = Command.Create(this, ApplyFilterInternal);
			SetVisibility = Command.Create(this, SetVisibilityInternal);
			SetWireframe = Command.Create(this, SetWireframeInternal);
			UpdateSelected = Command.Create(this, UpdateSelectedInternal);
			InvertSelection = Command.Create(this, InvertSelectionInternal);
			ToggleShowNormals = Command.Create(this, ToggleShowNormalsInternal);
			FocusPatternFilter = Command.Create(this, FocusPatternFilterInternal);

			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			ObjectManager = null!;
			PinState = null;
			base.OnClosed(e);
		}

		/// <summary>Pinned window support</summary>
		private PinData? PinState
		{
			get;
			set
			{
				if (field == value) return;
				Util.Dispose(ref field);
				field = value;
			}
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

				// Handler
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(View3d.ObjectManager.Objects):
						{
							ObjectsView.Refresh();
							break;
						}
					}
				}
				void HandleObjectPropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					if (sender is not View3d.Object obj)
						return;

					switch (e.PropertyName)
					{
						case nameof(View3d.Object.Flags):
						{
							NotifyPropertyChanged(nameof(FirstSelected));
							break;
						}
					}

					// If an object within this window has changed, refresh
					if (ObjectManager.Window.HasObject(obj, search_children: true))
						ObjectManager.Window.Invalidate();
				}
			}
		} = null!;

		/// <summary>Access to the object container</summary>
		public IList<View3d.Object> RootObjects => ObjectManager.Objects;
		public HashSet<View3d.Object> SelectedObjects => ObjectManager.SelectedObjects;

		/// <summary>True if one or more objects is selected</summary>
		public View3d.Object? FirstSelected => SelectedObjects.FirstOrDefault();

		/// <summary>A view of the top-level objects in the scene</summary>
		public ICollectionView ObjectsView { get; private set; }

		/// <summary>Refresh</summary>
		private void Invalidate() => ObjectManager.Window.Invalidate();

		/// <summary></summary>
		public Command ApplyFilter { get; }
		private void ApplyFilterInternal(object? parameter)
		{
			if (parameter is Pattern pattern && pattern.Expr.Length != 0 && pattern.IsValid)
				ObjectsView.Filter = x => pattern.IsMatch(((View3d.Object)x).Name);
			else
				ObjectsView.Filter = null;
		}

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

		/// <summary>Set the 'selected' state for the objects</summary>
		public Command UpdateSelected { get; }
		private void UpdateSelectedInternal(object? parameter)
		{
			if (parameter is SelectionChangedEventArgs args)
			{
				foreach (var x in args.RemovedItems.Cast<View3d.Object>())
				{
					x.Flags = Bit.SetBits(x.Flags, View3d.ELdrFlags.Selected, false);
					SelectedObjects.Remove(x);
				}
				foreach (var x in args.AddedItems.Cast<View3d.Object>())
				{
					x.Flags = Bit.SetBits(x.Flags, View3d.ELdrFlags.Selected, true);
					SelectedObjects.Add(x);
				}

				Invalidate();
			}
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
			return;
		}

		/// <summary>Move input focus to the filter bar</summary>
		public Command FocusPatternFilter { get; }
		private void FocusPatternFilterInternal()
		{
			// Find the PatternFilter control by name and focus it
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
