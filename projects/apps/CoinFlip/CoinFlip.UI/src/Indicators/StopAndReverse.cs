using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Xml.Linq;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Maths;
using Rylogic.Utility;

namespace CoinFlip.UI.Indicators
{
	[Indicator]
	public class StopAndReverse :Indicator<StopAndReverse>
	{
		public StopAndReverse()
		{
			AFStart = 0.02;
			AFStep = 0.02;
			AFMax = 0.2;
			Size = 10.0;
			Style = EPointStyle.Circle;
		}
		public StopAndReverse(XElement node)
			:base(node)
		{ }

		/// <summary>The starting value for the acceleration factor</summary>
		public double AFStart
		{
			get => get<double>(nameof(AFStart));
			set => set(nameof(AFStart), value);
		}

		/// <summary>The increment for the acceleration factor</summary>
		public double AFStep
		{
			get => get<double>(nameof(AFStep));
			set => set(nameof(AFStep), value);
		}

		/// <summary>The maximum value of the acceleration factor</summary>
		public double AFMax
		{
			get => get<double>(nameof(AFMax));
			set => set(nameof(AFMax), value);
		}

		/// <summary>The size of the SAR point grpahics</summary>
		public double Size
		{
			get => get<double>(nameof(Size));
			set => set(nameof(Size), value);
		}

		/// <summary>The style of points to draw</summary>
		public EPointStyle Style
		{
			get => get<EPointStyle>(nameof(Style));
			set => set(nameof(Style), value);
		}

		/// <summary>The label to use when displaying this indicator</summary>
		public override string Label => $"SAR {Name.Surround("(", ")")}";

		/// <summary>Create a view of this indicator for displaying on a chart</summary>
		public override IIndicatorView CreateView(IChartView chart)
		{
			return new View(this, chart);
		}

		/// <summary>A single SAR point</summary>
		[DebuggerDisplay("{Description,nq}")]
		public class SARPoint
		{
			public SARPoint(int candle_index, long ts, double value, ETrend trend)
			{
				Debug.Assert(trend != ETrend.Ranging);
				CandleIndex = candle_index;
				Value = value;
				Timestamp = ts;
				Trend = trend;
			}

			/// <summary>The index in the instrument that this point corresponds to</summary>
			public double CandleIndex { get; }

			/// <summary>The price value of the SAR at 'Timestamp'</summary>
			public double Value { get; }

			/// <summary>The candle timestamp that this MA point corresponds to</summary>
			public long Timestamp { get; }

			/// <summary>Either bullish or bearist. (Should never be ranging)</summary>
			public ETrend Trend { get; }

			/// <summary>The SAR point in chart space</summary>
			public v4 Point => new((float)CandleIndex, (float)Value, 0, 1f);

			/// <summary>Guess at whether double or long values are used</summary>
			private string Description => $"{CandleIndex} {Value}";
		}

		/// <summary>A SAR data set based on an instrument</summary>
		public class Context :IDisposable
		{
			// Notes:
			//  - Calculate the SAR for the full range (or perhaps limit to 1,000,000).
			//    It's not worth the memory/time/effort limiting the data to a range.

			private readonly List<SARPoint> m_data;
			public Context(StopAndReverse sar, Instrument instrument)
			{
				m_data = new List<SARPoint>();
				Options = sar;
				Instrument = instrument;

				Reset();
				Update();
			}
			public void Dispose()
			{
				Instrument = null!;
				Options = null!;
			}

			/// <summary>The moving average this context is based on</summary>
			private StopAndReverse Options
			{
				get;
				set
				{
					if (field == value) return;
					if (field != null)
					{
						field.SettingChange -= HandleSettingChange;
					}
					field = value;
					if (field != null)
					{
						field.SettingChange += HandleSettingChange;
					}

					// Handler
					void HandleSettingChange(object? sender, SettingChangeEventArgs e)
					{
						if (e.Before) return;
						switch (e.Key)
						{
							case nameof(StopAndReverse.AFStart):
							case nameof(StopAndReverse.AFStep):
							case nameof(StopAndReverse.AFMax):
							{
								Reset();
								Update();
								break;
							}
						}
					}
				}
			} = null!;

			/// <summary>The instrument to calculate the moving average over</summary>
			public Instrument Instrument
			{
				get;
				set
				{
					if (field == value) return;
					if (field != null)
					{
						field.DataChanged -= HandleDataChanged;
					}
					field = value;
					if (field != null)
					{
						field.DataChanged += HandleDataChanged;
					}

					// Handler
					void HandleDataChanged(object? sender, DataEventArgs e)
					{
						Update();
					}
				}
			} = null!;

			/// <summary>The number of points in the MA</summary>
			public int Count => m_data.Count;

			/// <summary>Basic list access</summary>
			public SARPoint this[int i] => m_data[i];

			/// <summary>The candle range spanned by this context</summary>
			public RangeF CandleRange => Count != 0 ? new RangeF(m_data.Front().CandleIndex, m_data.Back().CandleIndex + 1) : RangeF.Invalid;

			/// <summary>Returns the nearest index position of 'candle_index' within the data</summary>
			public int IndexOf(double candle_index) => m_data.BinarySearch(pt => pt.CandleIndex.CompareTo(candle_index), find_insert_position: true);

			/// <summary>Reset the data</summary>
			public void Reset()
			{
				m_data.Clear();
			}

			/// <summary>Update to match the current state of 'Instrument'</summary>
			public void Update()
			{
				// Notes:
				//  - This is based on Welles Wilder's "Parabolic Time/Price System" (page 11)
				//  - The algorithm calculates the SAR for the next period, so that SAR for index 'i' is based on
				//    the SAR for index 'i-1' and the candle for 'i-1' (and i-2 for the limits)

				// No instrument data
				if (Instrument.Count == 0)
				{
					Reset();
					return;
				}

				// The instrument count can go backwards when back testing
				if (Instrument.Count < m_data.Count)
				{
					Reset();
				}

				// Add the first SAR if 'm_data' is empty
				if (m_data.Count == 0)
				{
					var candle = Instrument[0];
					m_data.Add(new SARPoint(0, candle.Timestamp, candle.Bullish ? candle.Low : candle.High, candle.Bullish ? ETrend.Bullish : ETrend.Bearish));
				}

				// Get the range of new data
				var range = new RangeI(m_data.Count, Instrument.Count);

				// Get the current trend
				var trend = m_data[m_data.Count - 1].Trend;

				// Find the first SAR of the current trend
				int idx_first = m_data.Count - 1, idx_last = idx_first;
				for (; idx_first != 0 && m_data[idx_first - 1].Trend == trend; --idx_first) {}
				m_data.Resize(idx_first + 1);

				// Calculate the remaining SARs
				var af = Options.AFStart;
				var last = m_data[idx_first];
				var extreme = last.Trend == ETrend.Bullish ? Instrument[idx_first].High : Instrument[idx_first].Low;
				for (int i = idx_first + 1; i != Instrument.Count; ++i)
				{
					// Calculate the SAR for index 'i' using the candle at 'i-1'
					var candle0 = Instrument[i];
					var candle1 = Instrument[i - 1];
					var candle2 = i > 1 ? Instrument[i - 2] : candle1;

					// Find the i'th SAR
					SARPoint sar;
					switch (trend)
					{
					default: throw new Exception($"Invalid trend value: {last.Trend}");
					case ETrend.Bullish:
						{
							// Only accelerate if a new extreme is made
							if (candle1.High > extreme)
							{
								extreme = candle1.High;
								af = Math.Min(af + Options.AFStep, Options.AFMax);
							}
							
							// The SAR price increment
							var diff = af * (extreme - last.Value);
							var value = last.Value + diff;

							// Limit the sar value to the lowest of the current and previous candle
							if (value > candle1.Low) value = candle1.Low;
							if (value > candle2.Low) value = candle2.Low;
							if (value < last.Value) value = last.Value;

							// If the current candle has crossed the SAR value, trigger a trend change
							if (candle0.Low < value)
							{
								idx_first = i;
								trend = ETrend.Bearish;
								value = extreme;
								extreme = Math.Min(candle0.Low, candle1.Low);
								af = Options.AFStart;
							}

							// Create the SAR point
							sar = new SARPoint(i, candle0.Timestamp, value, trend);
							break;
						}
					case ETrend.Bearish:
						{
							// Only accelerate if a new extreme is made
							if (candle1.Low < extreme)
							{
								extreme = candle1.Low;
								af = Math.Min(af + Options.AFStep, Options.AFMax);
							}

							// The SAR price decrement
							var diff = af * (last.Value - extreme);
							var value = last.Value - diff;

							// Limit the sar value to the highest of the current and previous candle
							if (value < candle1.High) value = candle1.High;
							if (value < candle2.High) value = candle2.High;
							if (value > last.Value) value = last.Value;

							// If the current candle has crossed the SAR value, trigger a trend change
							if (candle0.High > value)
							{
								idx_first = i;
								trend = ETrend.Bullish;
								value = extreme;
								extreme = Math.Max(candle0.High, candle1.High);
								af = Options.AFStart;
							}

							// Create the SAR point
							sar = new SARPoint(i, candle0.Timestamp, value, trend);
							break;
						}
					}

					// Add the new SAR point
					m_data.Add(sar);
					last = sar;
				}

				// Notify
				DataChanged?.Invoke(this, new DataChangedEventArgs(range));
			}

			/// <summary>Raised when data is changed</summary>
			public event EventHandler<DataChangedEventArgs>? DataChanged;
		}

		/// <summary>A view of the indicator</summary>
		public class View :IndicatorView
		{
			public View(StopAndReverse sar, IChartView chart)
				:base(sar.Id, nameof(StopAndReverse), chart, sar)
			{
				Cache = new ChartGfxCache(CreatePiece);
				Data = new Context(sar, chart.Instrument ?? throw new NullReferenceException("Chart instrument is null"));
				Data.DataChanged += (s, a) =>
				{
					Cache.Invalidate(a.Range);
				};
			}
			protected override void Dispose(bool disposing)
			{
				Cache = null!;
				Util.Dispose(Data);
				base.Dispose(disposing);
			}

			/// <summary>The indicator data source</summary>
			private StopAndReverse SAR => (StopAndReverse)Indicator;

			/// <summary>The moving average data</summary>
			private Context Data { get; }

			/// <summary>A cache of graphics objects than span X-axis ranges</summary>
			private ChartGfxCache Cache
			{
				get;
				set
				{
					if (field == value) return;
					Util.Dispose(ref field!);
					field = value;
				}
			} = null!;

			/// <summary>Create graphics for an X-range spanning 'x'</summary>
			private SARPiece CreatePiece(double x, RangeF missing)
			{
				// The available data
				var data_range = Data.CandleRange;

				// If 'x' is before the available data, return a null graphics piece up to the start of available data
				if (x < data_range.Beg)
					return new SARPiece(new RangeF(missing.Beg, Math.Min(data_range.Beg, missing.End)));

				// If 'x' is after the available data, return a null graphics piece from the end of available data
				if (x >= data_range.End)
					return new SARPiece(new RangeF(Math.Max(data_range.End, missing.Beg), missing.End));

				// Generate an index range based on 'missing', limited to the block size
				const int PieceBlockSize = 512;
				var idx  = (int)Math.Floor(x);
				var idx0 = (int)Math_.Max(data_range.Beg, missing.Beg, idx - PieceBlockSize);
				var idx1 = (int)Math_.Min(data_range.End, missing.End, idx + PieceBlockSize);
				var count = idx1 - idx0;

				m_vbuf.Resize(count);
				m_ibuf.Resize(count);
				m_nbuf.Resize(2);

				// Create the geometry
				int vert = 0, indx = 0;
				for (int i = idx0; i != idx1; ++i)
				{
					var v = vert;
					m_vbuf[vert++] = new View3d.Vertex(new v4((float)Data[i].CandleIndex, (float)Data[i].Value, 0f, 1f));
					m_ibuf[indx++] = (ushort)v;
				}

				// Add a nugget for the points
				{
					var mat = View3d.Material.New();
					mat.m_diff_tex = 
						SAR.Style == EPointStyle.Square ? IntPtr.Zero :
						SAR.Style == EPointStyle.Circle ? View3d.Texture.FromStock(View3d.EStockTexture.WhiteSpot).Handle :
						SAR.Style == EPointStyle.Triangle ? View3d.Texture.FromStock(View3d.EStockTexture.WhiteTriangle).Handle :
						throw new Exception($"Unknown point style: {SAR.Style}");
					mat.Use(View3d.ERenderStep.ForwardRender, View3d.EShaderGS.PointSpritesGS, $"*PointSize {{{SAR.Size} {SAR.Size}}} *Depth {{{false}}}");
					var flags = SAR.Colour.HasAlpha ? View3d.ENuggetFlag.AlphaBlend : View3d.ENuggetFlag.None;
					m_nbuf[0] = new View3d.Nugget(View3d.ETopo.PointList, View3d.EGeom.Vert | View3d.EGeom.Tex0, flags: flags, mat: mat);
				}

				// Add a nugget for the glow
				{
					var mat = View3d.Material.New();
					mat.m_diff_tex = View3d.Texture.FromStock(View3d.EStockTexture.WhiteSpot).Handle;
					mat.Use(View3d.ERenderStep.ForwardRender, View3d.EShaderGS.PointSpritesGS, $"*PointSize {{{SAR.Size + 2 * GlowRadius} {SAR.Size + 2*GlowRadius}}} *Depth {{{false}}}");
					var flags = View3d.ENuggetFlag.AlphaBlend | View3d.ENuggetFlag.Hidden;
					m_nbuf[1] = new View3d.Nugget(View3d.ETopo.PointList, View3d.EGeom.Vert | View3d.EGeom.Tex0, range_overlaps: true, flags: flags, mat: mat);
				}

				// Graphics object for the block of SARs
				var gfx = new View3d.Object($"{Name}-[{data_range.Beg},{data_range.End})", 0xFFFFFFFF, m_vbuf.Count, m_ibuf.Count, m_nbuf.Count, m_vbuf.ToArray(), m_ibuf.ToArray(), m_nbuf.ToArray(), Id);
				return new SARPiece(new RangeI(idx0, idx1), gfx);
			}

			/// <summary>Update when indicator settings change</summary>
			protected override void HandleSettingChange(object? sender, SettingChangeEventArgs e)
			{
				base.HandleSettingChange(sender, e);
				if (e.After)
				{
					switch (e.Key)
					{
						case nameof(StopAndReverse.Size):
						case nameof(StopAndReverse.Style):
						{
							Cache.Invalidate();
							break;
						}
					}
					Invalidate();
				}
			}

			/// <summary>Update the transforms for the graphics model</summary>
			protected override void UpdateSceneCore(View3d.Window window, View3d.Camera camera)
			{
				base.UpdateSceneCore(window, camera);
				var chart = Chart ?? throw new NullReferenceException("Chart is null");
				
				chart.Scene.RemoveObjects(new[] { SAR.Id }, 1, 0);

				// No data, no SAR
				if (Data.Count == 0 || !Visible)
					return;
				
				// Get the range required for display
				var range = Data.CandleRange.Intersect(chart.XAxis.Range);
				
				// Add each graphics piece. Really, 'Cache.Get' should be
				// called in UpdateGfxCore but this seems to be fast enough.
				foreach (var piece in Cache.Get(range).OfType<SARPiece>())
				{
					// Ignore null graphics pieces (places where there is no data)
					if (piece.NoGfx)
						continue;

					// Set the colour of the SAR points
					piece.Points.NuggetTintSet(SAR.Colour, index: 0);
					piece.Points.NuggetTintSet(SAR.Colour.Alpha(Selected ? 0.25 : 0.15), index: 1);

					// Show or hide the glow based on selected/hovered state
					piece.Points.NuggetFlagsSet(View3d.ENuggetFlag.Hidden, !(Hovered || Selected), index:1);

					// Add to the scene
					chart.Scene.AddObject(piece.Points);
				}
			}

			/// <summary>Display the options UI</summary>
			protected override void ShowOptionsUICore()
			{
				if (m_ui == null)
				{
					m_ui = new StopAndReverseUI(Window.GetWindow(Chart), SAR);
					m_ui.Closed += delegate { m_ui = null; };
					m_ui.Show();
				}
				m_ui.Focus();
			}
			private StopAndReverseUI? m_ui;

			/// <summary>Hit test this indicator</summary>
			public override ChartControl.HitTestResult.Hit? HitTest(v4 chart_point, v2 scene_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, View3d.Camera cam)
			{
				if (Data.Count == 0 || !Visible)
					return null;

				// Get the nearest SAR point to 'chart_point'
				var candle_index = (int)Math.Round(chart_point.x);
				if (candle_index < 0 || candle_index >= Data.Count)
					return null;

				var chart = Chart ?? throw new NullReferenceException("Chart is null");

				// Test for proximaty to the SAR point
				var sar_pt = Data[candle_index].Point;
				var sar_pt_scn = chart.ChartToScene(sar_pt);
				if ((sar_pt_scn - scene_point).LengthSq > Chart.Options.MinSelectionDistanceSq)
					return null;

				return new ChartControl.HitTestResult.Hit(this, sar_pt, null);
			}

			/// <summary>Graphics for a piece of the moving average line</summary>
			private class SARPiece :IChartGfxPiece
			{
				public SARPiece(RangeF range)
				{
					Range = range;
					Points = null!;
				}
				public SARPiece(RangeF range, View3d.Object gfx)
				{
					Range = range;
					Points = gfx;
				}
				public void Dispose()
				{
					Points = null!;
				}

				/// <summary>True if this is a null graphics piece</summary>
				public bool NoGfx => Points == null;

				/// <summary>The X-Axis span covered by this piece</summary>
				public RangeF Range { get; }

				/// <summary>The point to draw at the SAR value</summary>
				public View3d.Object Points
				{
					get;
					set
					{
						if (field == value) return;
						Util.Dispose(ref field!);
						field = value;
					}
				} = null!;
			}
		}

		/// <summary>Event args for Context.DataChanged</summary>
		public class DataChangedEventArgs :EventArgs
		{
			public DataChangedEventArgs(RangeI range)
			{
				Range = range;
			}

			/// <summary>The X Range of the data that has changed</summary>
			public RangeI Range { get; }
		}

		/// <summary>Returns a mouse op instance for creating the indicator</summary>
		public static IIndicator Create(CandleChart _) => new StopAndReverse();
	}
}
