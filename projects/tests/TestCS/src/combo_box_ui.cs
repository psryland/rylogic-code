using System;
using System.Drawing;
using System.Windows.Forms;
using Rylogic.Container;
using Rylogic.Extn;
using Rylogic.Gui.WinForms;
using ComboBox = Rylogic.Gui.WinForms.ComboBox;
using Util = Rylogic.Utility.Util;

namespace TestCS
{
	public class ComboBoxUI :Form
	{
		private BindingSource<string> m_bs;

		#region UI Elements
		private ComboBox m_cb0;
		private Label m_lbl_cb0;
		private Label m_lbl_selected_item;
		private Button m_btn_read;
		private TextBox m_tb_selected_item;
		#endregion

		public ComboBoxUI()
		{
			InitializeComponent();
			m_bs = new BindingSource<string> {DataSource = new BindingListEx<string>()};
			m_bs.Add("Won");
			m_bs.Add("Too");
			m_bs.Add("Free");
			m_bs.Add("Fore");
			m_bs.Add("Vive");
			m_bs.Add("Secs");

			m_cb0.AutoCompleteMode = AutoCompleteMode.SuggestAppend;
			m_cb0.AutoCompleteSource = AutoCompleteSource.ListItems;
			m_cb0.DataSource = m_bs;
			m_cb0.SelectedIndexChanged += (s,a) =>
			{
				ReadSelection();
			};

			m_btn_read.Click += (s,a) =>
			{
				ReadSelection();
			};
		}
		protected override void Dispose(bool disposing)
		{
			Util.Dispose(ref components);
			base.Dispose(disposing);
		}

		private void ReadSelection()
		{
			m_tb_selected_item.Text = (string)m_cb0.SelectedItem ?? "<null>";
			m_tb_selected_item.BackColor = Color.LightGreen;
			SynchronizationContext_.BeginInvokeDelayed(() => m_tb_selected_item.BackColor = Color.White, TimeSpan.FromMilliseconds(200));
		}

		#region Windows Form Designer generated code
		private System.ComponentModel.IContainer components = null;
		private void InitializeComponent()
		{
			this.m_cb0 = new Rylogic.Gui.WinForms.ComboBox();
			this.m_lbl_cb0 = new System.Windows.Forms.Label();
			this.m_lbl_selected_item = new System.Windows.Forms.Label();
			this.m_tb_selected_item = new System.Windows.Forms.TextBox();
			this.m_btn_read = new System.Windows.Forms.Button();
			this.SuspendLayout();
			// 
			// m_cb0
			// 
			this.m_cb0.DisplayProperty = null;
			this.m_cb0.FormattingEnabled = true;
			this.m_cb0.Location = new System.Drawing.Point(152, 12);
			this.m_cb0.Name = "m_cb0";
			this.m_cb0.PreserveSelectionThruFocusChange = false;
			this.m_cb0.Size = new System.Drawing.Size(167, 21);
			this.m_cb0.TabIndex = 0;
			// 
			// m_lbl_cb0
			// 
			this.m_lbl_cb0.AutoSize = true;
			this.m_lbl_cb0.Location = new System.Drawing.Point(12, 9);
			this.m_lbl_cb0.Name = "m_lbl_cb0";
			this.m_lbl_cb0.Size = new System.Drawing.Size(134, 39);
			this.m_lbl_cb0.TabIndex = 1;
			this.m_lbl_cb0.Text = "Auto complete, selected\r\nitem matches the displayed\r\ntext";
			// 
			// m_lbl_selected_item
			// 
			this.m_lbl_selected_item.AutoSize = true;
			this.m_lbl_selected_item.Location = new System.Drawing.Point(12, 59);
			this.m_lbl_selected_item.Name = "m_lbl_selected_item";
			this.m_lbl_selected_item.Size = new System.Drawing.Size(75, 13);
			this.m_lbl_selected_item.TabIndex = 2;
			this.m_lbl_selected_item.Text = "Selected Item:";
			// 
			// m_tb_selected_item
			// 
			this.m_tb_selected_item.Location = new System.Drawing.Point(152, 56);
			this.m_tb_selected_item.Name = "m_tb_selected_item";
			this.m_tb_selected_item.Size = new System.Drawing.Size(167, 20);
			this.m_tb_selected_item.TabIndex = 3;
			// 
			// m_btn_read
			// 
			this.m_btn_read.Location = new System.Drawing.Point(193, 82);
			this.m_btn_read.Name = "m_btn_read";
			this.m_btn_read.Size = new System.Drawing.Size(126, 23);
			this.m_btn_read.TabIndex = 4;
			this.m_btn_read.Text = "Read Selection";
			this.m_btn_read.UseVisualStyleBackColor = true;
			// 
			// ComboBoxUI
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size(331, 309);
			this.Controls.Add(this.m_btn_read);
			this.Controls.Add(this.m_tb_selected_item);
			this.Controls.Add(this.m_lbl_selected_item);
			this.Controls.Add(this.m_lbl_cb0);
			this.Controls.Add(this.m_cb0);
			this.Name = "ComboBoxUI";
			this.Text = "combo_box_ui";
			this.ResumeLayout(false);
			this.PerformLayout();

		}
		#endregion
	}
}
