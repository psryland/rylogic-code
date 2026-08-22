using System;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using Rylogic.Db;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WinForms;
using Rylogic.Scintilla;

namespace TestCS
{
	public class MainUI : Form
	{
		private MenuStrip m_menu;
		private ToolStripMenuItem m_menu_file;
		private ToolStripMenuItem m_menu_file_exit;
		private ToolStripMenuItem m_menu_tests;
		private ToolStripMenuItem m_menu_tests_view3d;
		private ToolStripMenuItem m_menu_tests_colourwheel;
		private ToolStripMenuItem m_menu_tests_hintballoon;
		private ToolStripMenuItem m_menu_tests_toolform;
		private ToolStripMenuItem m_menu_tests_treegrid;
		private ToolStripMenuItem m_menu_tests_checked_listbox;
		private ToolStripMenuItem m_menu_tests_diagramcontrol;
		private ToolStripMenuItem m_menu_tests_toolstrip_positions;
		private ToolStripMenuItem m_menu_tests_view3d_editor;
		private ToolStripMenuItem m_menu_tests_dgv;
		private ToolStripMenuItem m_menu_tests_helpui;
		private ToolStripMenuItem m_menu_tests_subclassed_controls;
		private ToolStripMenuItem m_menu_tests_rtb;
		private ToolStripMenuItem m_menu_tests_web_browser;
		private ToolStripMenuItem m_menu_tests_scintilla;
		private ToolStripMenuItem m_menu_tests_checked_groupbox;
		private ToolStripMenuItem m_menu_tests_dock_panel;
		private ToolStripMenuItem m_menu_tests_vt100;
		private ToolStripMenuItem m_menu_tests_message_box;
		private ToolStripMenuItem m_menu_tests_combo_box;
		private ToolStripMenuItem m_menu_tests_chart;
		private ToolStripMenuItem m_menu_tests_listbox;
		private ToolStripMenuItem m_menu_tests_bluetooth_ui;
		private ToolStripMenuItem m_menu_tests_log_ui;
		private ToolStripMenuItem m_menu_tests_prompt_ui;
		private ToolStripMenuItem m_menu_tests_graphcontrol;

		/// <summary>The main entry point for the application.</summary>
		[STAThread] static void Main()
		{
			// Note! Running this in the debugger causes this to be run as a 32bit
			// process regardless of the selected solution platform
			Debug.WriteLine($"\n    {Application.ExecutablePath} is a {(Environment.Is64BitProcess?"64":"32")}bit process\n");

			Sci.LoadDll();
			Sqlite.LoadDll();
			View3d.LoadDll();

			Application.EnableVisualStyles();
			Application.SetCompatibleTextRenderingDefault(false);
			Application.Run(new MainUI());
			//Application.Run(new DgvUI());
		}

		public MainUI()
		{
			InitializeComponent();
			Shown += (s, a) =>
			{
				//Location = new Point(150,150);
				Location = new Point(2150, 150);
			};

			m_menu_file_exit.Click += (s, a) =>
			{
				Close();
			};

			m_menu_tests_bluetooth_ui.Click += (s,a) =>
			{
				new BluetoothUI() { ShowRadioSelector = false }.ShowDialog(this);
			};

			m_menu_tests_chart.Click += (s,a) =>
			{
				new ChartUI().Show(this);
			};

			m_menu_tests_checked_groupbox.Click += (s, a) =>
			{
				new CheckedGroupBoxUI().Show(this);
			};

			m_menu_tests_checked_listbox.Click += (s, a) =>
			{
				new CheckedListBoxUI().Show(this);
			};

			m_menu_tests_colourwheel.Click += (s, a) =>
			{
				new FormColourWheel().Show(this);
			};

			m_menu_tests_combo_box.Click += (s, a) =>
			{
				new ComboBoxUI().Show(this);
			};

			m_menu_tests_diagramcontrol.Click += (s, a) =>
			{
				new DiagramControlUI().Show(this);
			};

			m_menu_tests_dgv.Click += (s, a) =>
			{
				new DgvUI().Show(this);
			};

			m_menu_tests_dock_panel.Click += (s, a) =>
			{
				new DockPanelUI().Show(this);
			};

			m_menu_tests_graphcontrol.Click += (s, a) =>
			{
				new GraphControlUI().Show(this);
			};

			m_menu_tests_helpui.Click += (s, a) =>
			{
				new HelpUI().Show(this);
			};

			m_menu_tests_hintballoon.Click += (s, a) =>
			{
				new FormHintBalloon().Show(this);
			};

			m_menu_tests_listbox.Click += (s,a) =>
			{
				new ListBoxUI().Show(this);
			};

			m_menu_tests_log_ui.Click += (s,a) =>
			{
				new LogUI().Show(this);
			};

			m_menu_tests_message_box.Click += (s,a) =>
			{
				using (var dlg = new MsgBox("This is a MsgBox", "Title", MessageBoxButtons.AbortRetryIgnore, MessageBoxIcon.Information))
				{
					dlg.Panel.BackColor = SystemColors.ControlLight;
					dlg.TextBox.BorderStyle = BorderStyle.None;
					dlg.NeutralBtnText = "NewTroll";
					dlg.NeutralBtn.DialogResult = DialogResult.None;
					dlg.ShowDialog();
				}
			};

			m_menu_tests_prompt_ui.Click += (s,a) =>
			{
				var combo = new PromptUI
				{
					Title = "Input Mode",
					PromptText = "Select the input mode",
					Mode = PromptUI.EMode.OptionSelect,
					Options = Enum<PromptUI.EInputType>.Values,
				};
				combo.ShowDialog(this);
				var value = new PromptUI
				{
					Title = "Value Input",
					PromptText = "Type in something",
					InputType = (PromptUI.EInputType)combo.Value,
				};
				value.ShowDialog(this);
			};

			m_menu_tests_scintilla.Click += (s, a) =>
			{
				new ScintillaUI().Show(this);
			};

			m_menu_tests_subclassed_controls.Click += (s, a) =>
			{
				new SubclassedControlsUI().Show(this);
			};

			m_menu_tests_toolform.Click += (s, a) =>
			{
				new ToolFormUI(this).Show(this);
			};

			m_menu_tests_toolstrip_positions.Click += (s, a) =>
			{
				new ToolStripPositionsUI().Show(this);
			};

			m_menu_tests_treegrid.Click += (s, a) =>
			{
				new TreeGridUI().Show(this);
			};

			m_menu_tests_rtb.Click += (s, a) =>
			{
				new RichTextBoxUI().Show(this);
			};

			m_menu_tests_view3d.Click += (s, a) =>
			{
				new FormView3d().Show(this);
			};

			m_menu_tests_view3d_editor.Click += (s, a) =>
			{
				new LdrEditorUI().Show(this);
			};

			m_menu_tests_vt100.Click += (s, a) =>
			{
				new VT100UI().Show(this);
			};

			m_menu_tests_web_browser.Click += (s, a) =>
			{
				new WebBrowserUI().Show(this);
			};
		}

		#region Windows Form Designer generated code

		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		/// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
		protected override void Dispose(bool disposing)
		{
			if (disposing && (components != null))
			{
				components.Dispose();
			}
			base.Dispose(disposing);
		}

		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			this.m_menu = new System.Windows.Forms.MenuStrip();
			this.m_menu_file = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_file_exit = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_bluetooth_ui = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_chart = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_checked_groupbox = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_checked_listbox = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_colourwheel = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_combo_box = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_dgv = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_diagramcontrol = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_dock_panel = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_graphcontrol = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_helpui = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_hintballoon = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_listbox = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_log_ui = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_message_box = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_prompt_ui = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_rtb = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_scintilla = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_subclassed_controls = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_toolform = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_toolstrip_positions = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_treegrid = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_view3d = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_view3d_editor = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_vt100 = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu_tests_web_browser = new System.Windows.Forms.ToolStripMenuItem();
			this.m_menu.SuspendLayout();
			this.SuspendLayout();
			// 
			// m_menu
			// 
			this.m_menu.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.m_menu_file,
            this.m_menu_tests});
			this.m_menu.Location = new System.Drawing.Point(0, 0);
			this.m_menu.Name = "m_menu";
			this.m_menu.Size = new System.Drawing.Size(219, 24);
			this.m_menu.TabIndex = 0;
			this.m_menu.Text = "m_menu";
			// 
			// m_menu_file
			// 
			this.m_menu_file.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.m_menu_file_exit});
			this.m_menu_file.Name = "m_menu_file";
			this.m_menu_file.Size = new System.Drawing.Size(37, 20);
			this.m_menu_file.Text = "&File";
			// 
			// m_menu_file_exit
			// 
			this.m_menu_file_exit.Name = "m_menu_file_exit";
			this.m_menu_file_exit.Size = new System.Drawing.Size(92, 22);
			this.m_menu_file_exit.Text = "E&xit";
			// 
			// m_menu_tests
			// 
			this.m_menu_tests.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.m_menu_tests_bluetooth_ui,
            this.m_menu_tests_chart,
            this.m_menu_tests_checked_groupbox,
            this.m_menu_tests_checked_listbox,
            this.m_menu_tests_colourwheel,
            this.m_menu_tests_combo_box,
            this.m_menu_tests_dgv,
            this.m_menu_tests_diagramcontrol,
            this.m_menu_tests_dock_panel,
            this.m_menu_tests_graphcontrol,
            this.m_menu_tests_helpui,
            this.m_menu_tests_hintballoon,
            this.m_menu_tests_listbox,
            this.m_menu_tests_log_ui,
            this.m_menu_tests_message_box,
            this.m_menu_tests_prompt_ui,
            this.m_menu_tests_rtb,
            this.m_menu_tests_scintilla,
            this.m_menu_tests_subclassed_controls,
            this.m_menu_tests_toolform,
            this.m_menu_tests_toolstrip_positions,
            this.m_menu_tests_treegrid,
            this.m_menu_tests_view3d,
            this.m_menu_tests_view3d_editor,
            this.m_menu_tests_vt100,
            this.m_menu_tests_web_browser});
			this.m_menu_tests.Name = "m_menu_tests";
			this.m_menu_tests.Size = new System.Drawing.Size(45, 20);
			this.m_menu_tests.Text = "&Tests";
			// 
			// m_menu_tests_bluetooth_ui
			// 
			this.m_menu_tests_bluetooth_ui.Name = "m_menu_tests_bluetooth_ui";
			this.m_menu_tests_bluetooth_ui.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_bluetooth_ui.Text = "&BluetoothUI";
			// 
			// m_menu_tests_chart
			// 
			this.m_menu_tests_chart.Name = "m_menu_tests_chart";
			this.m_menu_tests_chart.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_chart.Text = "&Chart";
			// 
			// m_menu_tests_checked_groupbox
			// 
			this.m_menu_tests_checked_groupbox.Name = "m_menu_tests_checked_groupbox";
			this.m_menu_tests_checked_groupbox.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_checked_groupbox.Text = "&CheckedGroupBox";
			// 
			// m_menu_tests_checked_listbox
			// 
			this.m_menu_tests_checked_listbox.Name = "m_menu_tests_checked_listbox";
			this.m_menu_tests_checked_listbox.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_checked_listbox.Text = "CheckedListBox";
			// 
			// m_menu_tests_colourwheel
			// 
			this.m_menu_tests_colourwheel.Name = "m_menu_tests_colourwheel";
			this.m_menu_tests_colourwheel.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_colourwheel.Text = "&ColourWheel";
			// 
			// m_menu_tests_combo_box
			// 
			this.m_menu_tests_combo_box.Name = "m_menu_tests_combo_box";
			this.m_menu_tests_combo_box.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_combo_box.Text = "&ComboBox";
			// 
			// m_menu_tests_dgv
			// 
			this.m_menu_tests_dgv.Name = "m_menu_tests_dgv";
			this.m_menu_tests_dgv.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_dgv.Text = "&DataGridView";
			// 
			// m_menu_tests_diagramcontrol
			// 
			this.m_menu_tests_diagramcontrol.Name = "m_menu_tests_diagramcontrol";
			this.m_menu_tests_diagramcontrol.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_diagramcontrol.Text = "&DiagramControl";
			// 
			// m_menu_tests_dock_panel
			// 
			this.m_menu_tests_dock_panel.Name = "m_menu_tests_dock_panel";
			this.m_menu_tests_dock_panel.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_dock_panel.Text = "&DockPanel";
			// 
			// m_menu_tests_graphcontrol
			// 
			this.m_menu_tests_graphcontrol.Name = "m_menu_tests_graphcontrol";
			this.m_menu_tests_graphcontrol.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_graphcontrol.Text = "&GraphControl";
			// 
			// m_menu_tests_helpui
			// 
			this.m_menu_tests_helpui.Name = "m_menu_tests_helpui";
			this.m_menu_tests_helpui.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_helpui.Text = "&Help UI";
			// 
			// m_menu_tests_hintballoon
			// 
			this.m_menu_tests_hintballoon.Name = "m_menu_tests_hintballoon";
			this.m_menu_tests_hintballoon.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_hintballoon.Text = "HintBalloon";
			// 
			// m_menu_tests_listbox
			// 
			this.m_menu_tests_listbox.Name = "m_menu_tests_listbox";
			this.m_menu_tests_listbox.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_listbox.Text = "&ListBox";
			// 
			// m_menu_tests_log_ui
			// 
			this.m_menu_tests_log_ui.Name = "m_menu_tests_log_ui";
			this.m_menu_tests_log_ui.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_log_ui.Text = "&Log UI";
			// 
			// m_menu_tests_message_box
			// 
			this.m_menu_tests_message_box.Name = "m_menu_tests_message_box";
			this.m_menu_tests_message_box.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_message_box.Text = "&Message Box";
			// 
			// m_menu_tests_prompt_ui
			// 
			this.m_menu_tests_prompt_ui.Name = "m_menu_tests_prompt_ui";
			this.m_menu_tests_prompt_ui.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_prompt_ui.Text = "&Prompt UI";
			// 
			// m_menu_tests_rtb
			// 
			this.m_menu_tests_rtb.Name = "m_menu_tests_rtb";
			this.m_menu_tests_rtb.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_rtb.Text = "&Rich Text Box";
			// 
			// m_menu_tests_scintilla
			// 
			this.m_menu_tests_scintilla.Name = "m_menu_tests_scintilla";
			this.m_menu_tests_scintilla.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_scintilla.Text = "&Scintilla";
			// 
			// m_menu_tests_subclassed_controls
			// 
			this.m_menu_tests_subclassed_controls.Name = "m_menu_tests_subclassed_controls";
			this.m_menu_tests_subclassed_controls.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_subclassed_controls.Text = "&Subclassed Controls";
			// 
			// m_menu_tests_toolform
			// 
			this.m_menu_tests_toolform.Name = "m_menu_tests_toolform";
			this.m_menu_tests_toolform.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_toolform.Text = "&Tool Form";
			// 
			// m_menu_tests_toolstrip_positions
			// 
			this.m_menu_tests_toolstrip_positions.Name = "m_menu_tests_toolstrip_positions";
			this.m_menu_tests_toolstrip_positions.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_toolstrip_positions.Text = "ToolStripPositions";
			// 
			// m_menu_tests_treegrid
			// 
			this.m_menu_tests_treegrid.Name = "m_menu_tests_treegrid";
			this.m_menu_tests_treegrid.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_treegrid.Text = "&Tree Grid";
			// 
			// m_menu_tests_view3d
			// 
			this.m_menu_tests_view3d.Name = "m_menu_tests_view3d";
			this.m_menu_tests_view3d.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_view3d.Text = "&View3d";
			// 
			// m_menu_tests_view3d_editor
			// 
			this.m_menu_tests_view3d_editor.Name = "m_menu_tests_view3d_editor";
			this.m_menu_tests_view3d_editor.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_view3d_editor.Text = "&View3d Editor";
			// 
			// m_menu_tests_vt100
			// 
			this.m_menu_tests_vt100.Name = "m_menu_tests_vt100";
			this.m_menu_tests_vt100.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_vt100.Text = "&VT100";
			// 
			// m_menu_tests_web_browser
			// 
			this.m_menu_tests_web_browser.Name = "m_menu_tests_web_browser";
			this.m_menu_tests_web_browser.Size = new System.Drawing.Size(180, 22);
			this.m_menu_tests_web_browser.Text = "&Web Browser";
			// 
			// MainUI
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size(219, 37);
			this.Controls.Add(this.m_menu);
			this.MainMenuStrip = this.m_menu;
			this.Name = "MainUI";
			this.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
			this.Text = "TestApp";
			this.m_menu.ResumeLayout(false);
			this.m_menu.PerformLayout();
			this.ResumeLayout(false);
			this.PerformLayout();

		}

		#endregion
	}
}
