using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using Rylogic.Extn;
using Rylogic.Gui.WPF;

namespace LDraw.UI
{
	public partial class SceneSelectorUI : UserControl, INotifyPropertyChanged
	{
		public SceneSelectorUI()
		{
			InitializeComponent();
			// Don't bind DataContext, we need to inherit it
		}

		/// <summary>The source</summary>
		public Source? Source
		{
			get => (Source?)GetValue(SourceProperty);
			set => SetValue(SourceProperty, value);
		}
		private void Source_Changed(Source? nue, Source? old)
		{
			if (old is not null)
			{
				old.PropertyChanged -= HandleSourcePropertyChanged;
			}
			if (nue is not null)
			{
				nue.PropertyChanged += HandleSourcePropertyChanged;

				PopulateAvailableScenes(nue);
				NotifyPropertyChanged(nameof(SelectedScenesDescription));
			}

			void PopulateAvailableScenes(Source source)
			{
				var scenes = (List<SceneWrapper>)AvailableScenes.SourceCollection;
				scenes.Assign(source.AvailableScenes.Select(x => new SceneWrapper(this, x)) ?? []);
				AvailableScenes.Refresh();
			}
			void HandleSourcePropertyChanged(object? sender, PropertyChangedEventArgs e)
			{
				switch (e.PropertyName)
				{
					case nameof(Source.AvailableScenes):
					{
						var source = (Source?)sender ?? throw new Exception("Expected sender to be a Source");
						PopulateAvailableScenes(source);
						
						// 'SelectedScenesDescription' can change if the number of scenes decreases
						NotifyPropertyChanged(nameof(SelectedScenesDescription));
						break;
					}
					case nameof(Source.SelectedScenes):
					{
						NotifyPropertyChanged(nameof(SelectedScenesDescription));
						break;
					}
				}
			}
		}
		public static readonly DependencyProperty SourceProperty = Gui_.DPRegister<SceneSelectorUI>(nameof(Source), null, Gui_.EDPFlags.None);

		/// <summary>The selected scenes (as a single string)</summary>
		public string SelectedScenesDescription
		{
			get => (Source?.SelectedScenes.Count() ?? 0) switch
			{
				0 => "None",
				1 => Source!.SelectedScenes.First().SceneName,
				_ => $"In {Source!.SelectedScenes.Count()} Scenes",
			};
		}

		/// <summary>The available scenes</summary>
		public ICollectionView AvailableScenes { get; } = new ListCollectionView(new List<SceneWrapper>());

		/// <summary>Handle checkbox clicks to prevent ComboBox from closing</summary>
		private void CheckBox_Click(object sender, RoutedEventArgs e)
		{
			// Keep the dropdown open
			m_scenes_combo.IsDropDownOpen = true;

			// Prevent the ComboBox from closing when clicking checkboxes
			e.Handled = true;
		}

		/// <summary>Handle mouse down on ComboBox items to prevent selection</summary>
		private void ComboBoxItem_PreviewMouseDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			// Prevent the item from being "selected" in the traditional sense
			e.Handled = true;

			// Find the checkbox within this item and toggle it
			if (sender is ComboBoxItem item &&
				Gui_.FindVisualChild<CheckBox>(item) is CheckBox checkBox)	
			{
				checkBox.IsChecked = !checkBox.IsChecked;
			}

			// Keep the dropdown open
			m_scenes_combo.IsDropDownOpen = true;
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>Scene wrapper for selection support</summary>
		public class SceneWrapper(SceneSelectorUI me, SceneUI scene) : INotifyPropertyChanged
		{
			public string Name => scene.SceneName;
			public bool Selected
			{
				get => me.Source?.SelectedScenes.Any(x => x.SceneName == Name) ?? false;
				set
				{
					if (me.Source is null)
						return;

					me.Source.ShowInScenes([scene], value);
					NotifyPropertyChanged(nameof(Selected));
					me.NotifyPropertyChanged(nameof(me.SelectedScenesDescription));
				}
			}

			/// <inheritdoc/>
			public event PropertyChangedEventHandler? PropertyChanged;
			private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}