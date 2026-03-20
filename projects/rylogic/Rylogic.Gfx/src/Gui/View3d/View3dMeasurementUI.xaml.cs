using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class View3dMeasurementUI : Window, INotifyPropertyChanged
	{
		public View3dMeasurementUI(View3dControl owner)
		{
			InitializeComponent();
			Owner = GetWindow(owner);

			View3dCtrl = owner;
			Measurement = new Measurement(owner.Window);
			PinState = new PinData(this, EPin.TopRight, pinned: true);

			// Set up commands
			SetSpotColour = Command.Create(this, SetSpotColourInternal);
			ReferenceFrames = new ListCollectionView(Enum<Measurement.EReferenceFrame>.ValuesArray);

			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			Measurement = null!;
			View3dCtrl = null!;
			PinState = null;
			base.OnClosed(e);
		}

		/// <summary>The View3d Control that contains the 3d scene</summary>
		public View3dControl View3dCtrl
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.MouseDown -= HandleMouseDown;
					field.MouseMove -= HandleMouseMove;
					field.MouseUp -= HandleMouseUp;
				}
				field = value;
				if (field != null)
				{
					field.MouseUp += HandleMouseUp;
					field.MouseMove += HandleMouseMove;
					field.MouseDown += HandleMouseDown;
				}
				void HandleMouseDown(object sender, MouseButtonEventArgs e)
				{
					if (e.ChangedButton == MouseButton.Left && e.LeftButton == MouseButtonState.Pressed)
						Measurement.MouseDown(e.GetPosition(field).ToV2());
				}
				void HandleMouseMove(object sender, MouseEventArgs e)
				{
					Measurement.MouseMove(e.GetPosition(field).ToV2());
				}
				void HandleMouseUp(object sender, MouseButtonEventArgs e)
				{
					if (e.ChangedButton == MouseButton.Left && e.LeftButton == MouseButtonState.Released)
						Measurement.MouseUp();
				}
			}
		} = null!;

		/// <summary>The view model for the measurement behaviour</summary>
		public Measurement Measurement
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.PropertyChanged -= HandlePropertyChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.PropertyChanged += HandlePropertyChanged;
				}

				// Handler
				void HandlePropertyChanged(object? sender, PropertyChangedEventArgs e)
				{
					switch (e.PropertyName)
					{
						case nameof(Measurement.SnapMode):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SnapToVerts)));
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SnapToEdges)));
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SnapToFaces)));
							break;
						}
						case nameof(Measurement.Hit0):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(BegPoint)));
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(BegPointDetails)));
							break;
						}
						case nameof(Measurement.Hit1):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(EndPoint)));
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(EndPointDetails)));
							break;
						}
						case nameof(Measurement.ActiveHit):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(StartActive)));
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(EndActive)));
							break;
						}
						case nameof(Measurement.BegSpotColour):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(BegSpotColourBrush)));
							break;
						}
						case nameof(Measurement.EndSpotColour):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(EndSpotColourBrush)));
							break;
						}
						case nameof(Measurement.SnapDistance):
						{
							PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SnapDistance)));
							break;
						}
					}
				}
			}
		} = null!;

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

		/// <summary>Snap to distance</summary>
		public double SnapDistance
		{
			get => Measurement.SnapDistance;
			set => Measurement.SnapDistance = value;
		}

		/// <summary>Snap bindings</summary>
		public bool SnapToVerts
		{
			get => Measurement.SnapMode.HasFlag(View3d.ESnapMode.Verts);
			set => Measurement.SnapMode = Bit.SetBits(Measurement.SnapMode, View3d.ESnapMode.Verts, value);
		}
		public bool SnapToEdges
		{
			get => Measurement.SnapMode.HasFlag(View3d.ESnapMode.Edges);
			set => Measurement.SnapMode = Bit.SetBits(Measurement.SnapMode, View3d.ESnapMode.Edges, value);
		}
		public bool SnapToFaces
		{
			get => Measurement.SnapMode.HasFlag(View3d.ESnapMode.Faces);
			set => Measurement.SnapMode = Bit.SetBits(Measurement.SnapMode, View3d.ESnapMode.Faces, value);
		}

		/// <summary>Binding for which set point to use</summary>
		public bool StartActive
		{
			get => Measurement.ActiveHit == Measurement.Hit0;
			set => Measurement.ActiveHit = value ? Measurement.Hit0 : null;
		}
		public bool EndActive
		{
			get => Measurement.ActiveHit == Measurement.Hit1;
			set => Measurement.ActiveHit = value ? Measurement.Hit1 : null;
		}

		/// <summary>The start and end points</summary>
		public v3 BegPoint
		{
			get => Measurement.BegPoint.xyz;
			set => Measurement.BegPoint = new v4(value, 1);
		}
		public v3 EndPoint
		{
			get => Measurement.EndPoint.xyz;
			set => Measurement.EndPoint = new v4(value, 1);
		}

		/// <summary>Binding for the spot colour</summary>
		public Brush BegSpotColourBrush => new SolidColorBrush(Measurement.BegSpotColour.ToMediaColor());
		public Brush EndSpotColourBrush => new SolidColorBrush(Measurement.EndSpotColour.ToMediaColor());

		/// <summary>Information about the start/end points</summary>
		public string BegPointDetails =>
			$"Object: {Measurement.Hit0.Obj?.Name ?? "---"}\n" +
			$"Snap: {Measurement.Hit0.SnapType}";
		public string EndPointDetails =>
			$"Object: {Measurement.Hit1.Obj?.Name ?? "---"}\n" +
			$"Snap: {Measurement.Hit1.SnapType}";

		/// <summary>The available measurement reference frames</summary>
		public ICollectionView ReferenceFrames
		{
			get;
			private set
			{
				if (field == value) return;
				if (field != null)
				{
					field.CurrentChanged -= HandleReferenceFrameChanged;
				}
				field = value;
				if (field != null)
				{
					ReferenceFrames.MoveCurrentTo(Measurement.ReferenceFrame);
					field.CurrentChanged += HandleReferenceFrameChanged;
				}

				// Handlers
				void HandleReferenceFrameChanged(object? sender, EventArgs e)
				{
					Measurement.ReferenceFrame = (Measurement.EReferenceFrame)ReferenceFrames.CurrentItem;
				}
			}
		} = null!;

		/// <summary>The measurement results</summary>
		public IBindingList Results => Measurement.Results;

		/// <summary>Display the colour picker</summary>
		private void HandleEditSpotColour(object? sender, RoutedEventArgs e)
		{
			MessageBox.Show("Not Done Yet");
		}

		/// <summary>Event handler that updates the bound source when enter is pressed</summary>
		private void UpdateBindingOnEnterPressed(object? sender, KeyEventArgs e)
		{
			if (e.Key != Key.Enter || !(sender is TextBox tb))
				return;

			var binding = BindingOperations.GetBindingExpression(tb, TextBox.TextProperty);
			if (binding != null)
				binding.UpdateSource();
		}

		/// <summary>Change the spot colour for the start or end point</summary>
		public ICommand SetSpotColour { get; } = null!;
		private void SetSpotColourInternal(object? x)
		{
			if (x is not string spot)
				return;

			if (spot == "beg")
			{
				var dlg = new ColourPickerUI { Owner = this, Colour = Measurement.BegSpotColour };
				if (dlg.ShowDialog() == true)
					Measurement.BegSpotColour = dlg.Colour;
			}
			if (spot == "end")
			{
				var dlg = new ColourPickerUI { Owner = this, Colour = Measurement.EndSpotColour };
				if (dlg.ShowDialog() == true)
					Measurement.EndSpotColour = dlg.Colour;
			}
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
