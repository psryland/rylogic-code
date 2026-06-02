using System;
using System.ComponentModel;
using LDraw.UI;
using Rylogic.Gui.WPF;

namespace LDraw
{
	/// <summary>Binding wrapper for a scene</summary>
	public class SceneWrapper :IChartProxy, INotifyPropertyChanged
	{
		// Notes:
		//  - This wrapper is needed because when UIElement objects are used as the items
		//    of a combo box it treats them as child controls, becoming their parent.

		public static readonly SceneWrapper NullScene = new(null, null);
		public SceneWrapper(SceneUI? scene, Action<SceneUI, bool>? selected_cb = null)
		{
			SceneUI = scene;
			m_selected_cb = selected_cb;
		}

		/// <summary>The wrapped scene</summary>
		public SceneUI? SceneUI { get; }

		/// <summary>Access the wrapped chart</summary>
		public ChartControl? Chart => SceneUI?.SceneView;

		/// <summary>The name of the wrapped scene</summary>
		public string Name => SceneUI?.SceneName ?? "None";

		/// <summary>True if the scene is selected</summary>
		public bool Selected
		{
			get => m_selected;
			set
			{
				if (m_selected == value || SceneUI == null) return;
				m_selected = value;
				m_selected_cb?.Invoke(SceneUI, value);
				NotifyPropertyChanged(nameof(Selected));
			}
		}
		private bool m_selected;
		private readonly Action<SceneUI, bool>? m_selected_cb;

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name)
		{
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}

		/// <summary></summary>
		public static implicit operator SceneUI?(SceneWrapper? x) => x?.SceneUI;

		#region Equals
		public override bool Equals(object? obj)
		{
			return obj is SceneWrapper wrapper && ReferenceEquals(SceneUI, wrapper.SceneUI);
		}
		public override int GetHashCode()
		{
			return SceneUI?.GetHashCode() ?? 0;
		}
		#endregion
	}
}
