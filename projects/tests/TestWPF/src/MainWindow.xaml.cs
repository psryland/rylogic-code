using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media.Imaging;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.Native;
using Rylogic.Gui.WPF;
using Rylogic.Scintilla;
using Rylogic.Windows.Gui;

namespace TestWPF
{
	/// <summary>Interaction logic for MainWindow.xaml</summary>
	public partial class MainWindow : Window, INotifyPropertyChanged
	{
		static MainWindow()
		{
			View3d.LoadDll();
			Sci.LoadDll();
		}
		public MainWindow()
		{
			InitializeComponent();
			Things = new List<Thing>
			{
				new("One"  ),
				new("Two"  ),
				new("Three"),
				new("Four" ),
				new("Five" ),
				new("Six"  ),
				new("Seven"),
				new("Eight"),
				new("Nine" ),
				new("Ten"  ),
			};
			ThingsView = new ListCollectionView(Things);

			View3d = View3d.Create();
			View3d.Error += (object? sender, View3d.ErrorEventArgs e) => MsgBox.Show(this, e.Message, "View3d Error");
#if DEBUG
			PresentationTraceSources.DataBindingSource.Switch.Level = SourceLevels.Critical;
#endif
			m_recent_files.Add("One", false);
			m_recent_files.Add("Two", false);
			m_recent_files.Add("Three");
			m_recent_files.RecentFileSelected += s => Debug.WriteLine(s);

			ShowBitArrayUI = Command.Create(this, () =>
			{
				new BitArrayUI().Show();
			});
			ShowBrowsePathUI = Command.Create(this, () =>
			{
				new BrowsePathUI().Show();
			});
			ShowColourPicker = Command.Create(this, () =>
			{
				new ColourPickerUI().Show();
			});
			ShowChart = Command.Create(this, () =>
			{
				new ChartUI().Show();
			});
			ShowDataGrid = Command.Create(this, () =>
			{
				new DataGridUI().Show();
			});
			//ShowDiagram = Command.Create(this, () =>
			//{
			//	new DiagramUI().Show();
			//});
			ShowDiagram2 = Command.Create(this, () =>
			{
				new Diagram2UI().Show();
			});
			ShowDirectionPicker = Command.Create(this, () =>
			{
				var win = new Window
				{
					Owner = this,
					WindowStartupLocation = WindowStartupLocation.CenterOwner,
				};
				win.Content = new DirectionPicker();
				win.ShowDialog();
			});
			ShowDockContainer = Command.Create(this, () =>
			{
				new DockContainerUI().Show();
			});
			ShowJoystick = Command.Create(this, () =>
			{
				new JoystickUI().Show();
			});
			ShowMsgBox = Command.Create(this, () =>
			{
				var msg =
				"Informative isn't it\nThis is a really really really long message to test the automatic resizing of the dialog window to a desirable aspect ratio. " +
				"It's intended to be used for displaying error messages that can sometimes be really long. Once I had a message that was so long, it made the message " +
				"box extend off the screen and you couldn't click the OK button. That was a real pain so that's why I've added this auto aspect ratio fixing thing. " +
				"Hopefully it'll do the job in all cases and I'll never have to worry about it again...\n Hopefully...";
				var dlg = new MsgBox(this, msg, "Massage Box", MsgBox.EButtons.YesNoCancel, MsgBox.EIcon.Exclamation) { ShowAlwaysCheckbox = true };
				dlg.ShowDialog();
			});
			ShowListUI = Command.Create(this, () =>
			{
				var dlg = new ListUI(this)
				{
					Title = "Listing to the Left",
					Prompt = "Select anything you like",
					SelectionMode = SelectionMode.Multiple,
					AllowCancel = false,
				};
				dlg.Items.AddRange(new[] { "One", "Two", "Three", "Phooore, was that you?" });
				if (dlg.ShowDialog() == true)
					MsgBox.Show(this, $"{string.Join(",", dlg.SelectedItems.Cast<string>())}! Good Choice!", "Result", MsgBox.EButtons.OK);
			});
			ShowLogUI = Command.Create(this, () =>
			{
				var log_ui = new LogControl { LogEntryPattern = new Regex(@"^(?<Tag>.*?)\|(?<Level>.*?)\|(?<Timestamp>.*?)\|(?<Message>.*)", RegexOptions.Singleline | RegexOptions.Multiline | RegexOptions.CultureInvariant | RegexOptions.Compiled) };
				var dlg = new Window { Title = "Log UI", Content = log_ui, ResizeMode = ResizeMode.CanResizeWithGrip };
				dlg.Show();
			});
			ShowPatternEditor = Command.Create(this, () =>
			{
				new PatternEditorUI().Show();
			});
			ShowProgressUI = Command.Create(this, () =>
			{
				var dlg = new ProgressUI(this, "Test Progress", "Testicles", System.Drawing.SystemIcons.Exclamation.ToBitmapSource(), (u, _, p) =>
				{
					for (int i = 0, iend = 100; !u.CancelPending && i != iend; ++i)
					{
						p(new ProgressUI.UserState
						{
							Description = $"Testicle: {i / 10}",
							FractionComplete = 1.0 * i / iend,
							ProgressBarText = $"I'm up to {i}/{iend}",
						});
						Thread.Sleep(100);
					}
				})
				{ AllowCancel = true };

				// Modal
				using (dlg)
				{
					var res = dlg.ShowDialog(500);
					if (res == true) MessageBox.Show("Completed");
					if (res == false) MessageBox.Show("Cancelled");
				}

				// Non-modal
				//dlg.Show();
			});
			ShowPromptUI = Command.Create(this, () =>
			{
				var dlg = new PromptUI(this)
				{
					Title = "Prompting isn't it...",
					Prompt = "I'll have what she's having. Really long message\r\nwith new lines in and \r\n other stuff\r\n\r\nEnter a positive number",
					Value = "Really long value as well, that hopefully wraps around",
					Units = "kgs",
					Validate = x => double.TryParse(x, out var v) && v >= 0 ? ValidationResult.ValidResult : new ValidationResult(false, "Enter a positive number"),
					Image = (BitmapImage)FindResource("pencil"),
					MultiLine = true,
				};
				if (dlg.ShowDialog() == true)
					double.Parse(dlg.Value);
			});
			ShowRadialProgressUI = Command.Create(this, () =>
			{
				new RadialProgressUI { Owner = this }.Show();
			});
			ShowTextEditorUI = Command.Create(this, () =>
			{
				new TextEditorUI { Owner = this }.Show();
			});
			ShowToolWindow = Command.Create(this, () =>
			{
				new ToolUI { Owner = this }.Show();
			});
			ScintillaUI = Command.Create(this, () =>
			{
				new ScintillaUI().Show();
			});
			ShowView3DUI = Command.Create(this, () =>
			{
				new View3dUI().Show();
			});
			ShowTreeGridUI = Command.Create(this, () =>
			{
				new TreeGridUI().Show();
			});
			ShowTreeViewMSUI = Command.Create(this, () =>
			{
				new TreeViewMSUI().Show();
			});
			ShowVT100UI = Command.Create(this, () =>
			{
				new VT100UI().Show();
			});
			ShowWin32Window = Command.Create(this, () =>
			{
				using var application = new Win32Application(new Win32ApplicationOptions { Title = "Win32 Window" });
				application.Run();
			});
			Exit = Command.Create(this, Close);
			DataContext = this;
		}
		protected override void OnClosed(EventArgs e)
		{
			View3d.Dispose();
			base.OnClosed(e);
		}

		private View3d View3d { get; }

		public Command ShowBitArrayUI { get; }
		public Command ShowBrowsePathUI { get; }
		public Command ShowColourPicker { get; }
		public Command ShowChart { get; }
		public Command ShowDataGrid { get; }
		//public Command ShowDiagram { get; }
		public Command ShowDiagram2 { get; }
		public Command ShowDirectionPicker { get; }
		public Command ShowDockContainer { get; }
		public Command ShowJoystick { get; }
		public Command ShowMsgBox { get; }
		public Command ShowListUI { get; }
		public Command ShowLogUI { get; }
		public Command ShowPatternEditor { get; }
		public Command ShowProgressUI { get; }
		public Command ShowPromptUI { get; }
		public Command ShowRadialProgressUI { get; }
		public Command ShowTextEditorUI { get; }
		public Command ShowToolWindow { get; }
		public Command ScintillaUI { get; }
		public Command ShowView3DUI { get; }
		public Command ShowTreeGridUI { get; }
		public Command ShowTreeViewMSUI { get; }
		public Command ShowVT100UI { get; }
		public Command ShowWin32Window { get; }
		public Command Exit { get; }

		/// <summary>Test collection</summary>
		public ICollectionView ThingsView { get; }
		private List<Thing> Things { get; }

		/// <summary>Some enum value</summary>
		public EEnum EnumValue
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(EnumValue));
			}
		}

		/// <summary>Comma separated list of selected things</summary>
		public string SelectedDescription => string.Join(", ", Things.Where(x => x.IsChecked).Select(x => x.Name));

		/// <summary>Modify the strings collection</summary>
		private void ComboBoxAutoComplete_Update(object sender, EventArgs e)
		{
			var cb = (ComboBox)sender;
			var view = (ICollectionView)cb.ItemsSource;
			var list = (List<Thing>)view.SourceCollection;

			list.Clear();
			list.AddRange(Things.Where(x => x.Name.Contains(cb.Text)));
			view.Refresh();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary></summary>
		[DebuggerDisplay("{Name,nq}")]
		private class Thing
		{
			public Thing(string name)
			{
				Name = name;
			}
			public string Name { get; }
			public bool IsChecked { get; set; }
		}
	}
	public enum EEnum
	{
		Zero,
		One,
		Two,
		Three,
	}
}
