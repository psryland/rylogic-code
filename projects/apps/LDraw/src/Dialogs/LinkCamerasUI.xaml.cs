using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Data;
using LDraw.UI;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gui.WPF;
using Rylogic.Maths;

namespace LDraw.Dialogs
{
	public partial class LinkCamerasUI :Window, INotifyPropertyChanged
	{
		public LinkCamerasUI(Window owner, Model model)
		{
			InitializeComponent();
			Owner = owner;
			Icon = Owner?.Icon;

			SourceScenes = new ListCollectionView(new List<SceneWrapper>());
			TargetScenes = new ListCollectionView(new List<SceneWrapper>());
			Model = model;

			ResetLinks = Command.Create(this, ResetLinksInternal);
			Accept = Command.Create(this, AcceptInternal);
			PopulateAvavailableScenes();
			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			foreach (var scene in Model.Scenes)
				scene.CleanupLinks();
			
			Model = null!;
			base.OnClosed(e);
		}

		/// <summary>The scene we're setting links for</summary>
		private Model Model
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.Scenes.CollectionChanged -= HandleScenesCollectionChanged;
				}
				field = value;
				if (field != null)
				{
					field.Scenes.CollectionChanged += HandleScenesCollectionChanged;
				}

				// Handlers
				void HandleScenesCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
				{
					PopulateAvavailableScenes();
				}
			}
		} = null!;

		/// <summary>The scene to get navigation events from</summary>
		public SceneWrapper? Source
		{
			get => SourceScenes.CurrentAs<SceneWrapper>();
			set
			{
				if (Source == value) return;
				SourceScenes.MoveCurrentToOrFirst(value);

				// Update the scene lists so 'Source' isn't in the Target list
				PopulateAvavailableScenes();
			}
		}

		/// <summary>The scene that will depend on 'Source'</summary>
		public SceneWrapper? Target
		{
			get => TargetScenes.CurrentAs<SceneWrapper>();
			set
			{
				if (Target == value) return;
				TargetScenes.MoveCurrentToOrFirst(value);
				
				// Everything changes if the Target changes
				NotifyPropertyChanged(string.Empty);
			}
		}

		/// <summary>True if linking occurs in both directions</summary>
		public bool BiDirectional
		{
			get => m_bidirectional;
			set
			{
				if (BiDirectional == value) return;
				m_bidirectional = value;
				PopulateAvavailableScenes();
				NotifyPropertyChanged(nameof(BiDirectional));
			}
		}
		private bool m_bidirectional;

		/// <summary>Binding helpers</summary>
		public bool XPanLinked
		{
			get => Has(ELinkCameras.LeftRight);
			set => Set(ELinkCameras.LeftRight, value, nameof(XPanLinked));
		}
		public bool YPanLinked
		{
			get => Has(ELinkCameras.UpDown);
			set => Set(ELinkCameras.UpDown, value, nameof(YPanLinked));
		}
		public bool ZPanLinked
		{
			get => Has(ELinkCameras.InOut);
			set => Set(ELinkCameras.InOut, value, nameof(ZPanLinked));
		}
		public bool RotateLinked
		{
			get => Has(ELinkCameras.Rotate);
			set => Set(ELinkCameras.Rotate, value, nameof(RotateLinked));
		}
		public bool XAxisLinked
		{
			get => Has(ELinkAxes.XAxis);
			set => Set(ELinkAxes.XAxis, value, nameof(XAxisLinked));
		}
		public bool YAxisLinked
		{
			get => Has(ELinkAxes.YAxis);
			set => Set(ELinkAxes.YAxis, value, nameof(YAxisLinked));
		}
		private bool Has(ELinkCameras cam)
		{
			return FindLink(Source, Target) is ChartLink link && link.CamLink.HasFlag(cam); ;
		}
		private bool Has(ELinkAxes axis)
		{
			return FindLink(Source, Target) is ChartLink link && link.AxisLink.HasFlag(axis);
		}
		private void Set(ELinkCameras cam, bool value, string prop_name)
		{
			if (Has(cam) == value) return;
			if (Source?.SceneUI is not SceneUI source) return;
			if (Target?.SceneUI is not SceneUI target) return;
			
			var link0 = GetLink(source, target);
			link0.CamLink = Bit.SetBits(link0.CamLink, cam, value);
			if (BiDirectional)
			{
				var link1 = GetLink(target, source);
				link1.CamLink = link0.CamLink;
			}

			UpdateSelected();
			NotifyPropertyChanged(prop_name);
		}
		private void Set(ELinkAxes axis, bool value, string prop_name)
		{
			if (Has(axis) == value) return;
			if (Source?.SceneUI is not SceneUI source) return;
			if (Target?.SceneUI is not SceneUI target) return;

			var link0 = GetLink(source, target);
			link0.AxisLink = Bit.SetBits(link0.AxisLink, axis, value);
			if (BiDirectional)
			{
				var link1 = GetLink(target, source);
				link1.AxisLink = link0.AxisLink;
			}

			UpdateSelected();
			NotifyPropertyChanged(prop_name);
		}

		/// <summary>The available scenes</summary>
		public ICollectionView SourceScenes { get; }
		public ICollectionView TargetScenes { get; }
		private void PopulateAvavailableScenes()
		{
			var source = Source;
			var target = Target;

			{// Populate the source list with all available
				var list = (List<SceneWrapper>)SourceScenes.SourceCollection;
				list.Sync(Model.Scenes.Select(x => new SceneWrapper(x)));
				if (SourceScenes.CurrentItem == null) SourceScenes.MoveCurrentToFirst();
				SourceScenes.Refresh();
			}
			{// Populate the target list with all except the source
				var list = (List<SceneWrapper>)TargetScenes.SourceCollection;
				list.Sync(Model.Scenes.Except(Source).Select(x => new SceneWrapper(x)));
				if (TargetScenes.CurrentItem == null) TargetScenes.MoveCurrentToFirst();
				TargetScenes.Refresh();
			}

			// Apply bi-directional
			if (m_bidirectional)
			{
				// Ensure every link from Source to Target is bidirectional.
				// 'PopulateAvailableScenes' is called when the source changes
				// so only the current source needs to be considered.
				foreach (var src in SourceScenes.Cast<SceneWrapper>())
				foreach (var tgt in TargetScenes.Cast<SceneWrapper>())
				{
					var link0 = FindLink(src, tgt);
					var link1 = FindLink(tgt, src);
					if (link0 == null && link1 == null)
						continue;

					link0 ??= GetLink(src!, tgt!);
					link1 ??= GetLink(tgt!, src!);

					link0.CamLink  = link1.CamLink  = link0.CamLink  | link1.CamLink;
					link0.AxisLink = link1.AxisLink = link0.AxisLink | link1.AxisLink;
				}
			}
			else
			{
				// Remove the links from Target to Source
				var src = source;
				foreach (var tgt in TargetScenes.Cast<SceneWrapper>())
				{
					var link0 = FindLink(src, tgt);
					var link1 = FindLink(tgt, src);
					if (link0 == null || link1 == null)
						continue;

					link1.CamLink  = Bit.SetBits(link1.CamLink,  link0.CamLink,  false);
					link1.AxisLink = Bit.SetBits(link1.AxisLink, link0.AxisLink, false); 
				}
			}

			// Update selected flags
			UpdateSelected();

			// Everything changes if the Target changes
			NotifyPropertyChanged(string.Empty);
		}

		/// <summary>Update the selected flag on the scene wrappers to reflect linked scenes</summary>
		private void UpdateSelected()
		{
			var source = Source;
			foreach (var target in TargetScenes.Cast<SceneWrapper>())
				target.Selected = source != null && HasLink(source, target);
		}

		/// <summary>Get or create a link between 'Scene' and the selected 'Target'</summary>
		public static ChartLink? FindLink(SceneUI? source, SceneUI? target)
		{
			if (source == null || target == null) return null;
			return source.Links.FirstOrDefault(x => x.Target.TryGetTarget(out var chart) && chart == target.SceneView);
		}
		public static ChartLink GetLink(SceneUI source, SceneUI target)
		{
			return FindLink(source, target) ?? source.Links.Add2(new ChartLink(source.SceneView, target.SceneView));
		}

		/// <summary>True if there is a link between 'lhs' and 'rhs'</summary>
		public static bool HasLink(SceneUI? lhs, SceneUI? rhs)
		{
			return
				(FindLink(lhs, rhs) is ChartLink link0 && !(link0.CamLink == ELinkCameras.None && link0.AxisLink == ELinkAxes.None)) ||
				(FindLink(rhs, lhs) is ChartLink link1 && !(link1.CamLink == ELinkCameras.None && link1.AxisLink == ELinkAxes.None));
		}

		/// <summary>Remove all links</summary>
		public Command ResetLinks { get; }
		private void ResetLinksInternal()
		{
			foreach (var scn in Model.Scenes)
				scn.Links.Clear();
			
			GC.Collect();
			PopulateAvavailableScenes();
		}

		/// <summary>Close the dialog</summary>
		public Command Accept { get; }
		private void AcceptInternal()
		{
			Close();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
