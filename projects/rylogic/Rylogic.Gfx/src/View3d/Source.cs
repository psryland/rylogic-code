using System;
using System.ComponentModel;
using System.Linq;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		/// <summary>Reference type wrapper of a view3d light</summary>
		public class Source :IDisposable, INotifyPropertyChanged
		{
			public Source(Guid context_id, View3d view3d)
			{
				View3d = view3d;
				m_context_id = context_id;
			}
			public void Dispose()
			{
				View3d = null!;
				GC.SuppressFinalize(this);
			}

			/// <summary>The view3d instance this source belongs to</summary>
			public View3d View3d
			{
				get => m_view3d;
				private set
				{
					if (m_view3d == value) return;
					if (m_view3d != null)
					{
					}
					m_view3d = value;
					if (m_view3d != null)
					{
					}
				}
			}
			private View3d m_view3d = null!;

			/// <summary>The context Id associated with the source</summary>
			public Guid ContextId => m_context_id;
			private Guid m_context_id;

			/// <summary>The inner spot light cone angle (in degrees)</summary>
			public string Name
			{
				get => View3D_SourceNameGetBStr(ref m_context_id);
				set
				{
					if (Name == value) return;
					View3D_SourceNameSet(ref m_context_id, value);
					NotifyPropertyChanged(nameof(Name));
				}
			}

			/// <summary>Source info</summary>
			public SourceInfo Info
			{
				get => new SourceInfo(View3D_SourceInfo(ref m_context_id));
			}

			/// <summary>Remove and clean up this data source</summary>
			public void Remove()
			{
				View3D_SourceDelete(ref m_context_id);
			}

			/// <summary>Reload objects from this source</summary>
			public void Reload()
			{
				View3D_SourceReload(ref m_context_id);
			}

			/// <summary>Raised when this source has changed</summary>
			public event EventHandler? SourceChanged
			{
				add
				{
					if (StoreChangedInternal == null)
						View3d.OnStoreChange += HandleStoreChanged;

					StoreChangedInternal += value;
				}
				remove
				{
					StoreChangedInternal -= value;

					if (StoreChangedInternal == null)
						View3d.OnStoreChange -= HandleStoreChanged;
				}
			}
			private event EventHandler? StoreChangedInternal;
			private void HandleStoreChanged(object? sender, StoreChangedEventArgs e)
			{
				if (!e.ContextIds.Contains(ContextId))
					return;

				StoreChangedInternal?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Notify property value changed</summary>
			public event PropertyChangedEventHandler? PropertyChanged;
			private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}
