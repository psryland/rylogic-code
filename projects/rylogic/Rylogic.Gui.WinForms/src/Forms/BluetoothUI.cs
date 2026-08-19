using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.Linq;
using System.Windows.Forms;
using Microsoft.Win32.SafeHandles;
using Rylogic.Attrib;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WinForms
{
	public class BluetoothUI :Form
	{
		#region UI Elements
		private Button m_btn_cancel;
		private Button m_btn_ok;
		private ToolTip m_tt;
		private CheckBox m_chk_show_unknown;
		private CheckBox m_chk_show_remembered;
		private CheckBox m_chk_show_connected;
		private CheckBox m_chk_show_paired;
		private CheckBox m_chk_discoverable;
		private ComboBox m_cb_radio;
		private ListBox m_lb_devices;
		private Button m_btn_show_bt_cpl;
		private Timer m_timer;
		private Button m_btn_pair;
		#endregion

		public BluetoothUI()
		{
			InitializeComponent();

			PopulateRadios();
			ShowDevices = Bluetooth.EOptions.ReturnAll;

			SetupUI();
			UpdateUI();
		}
		protected override void Dispose(bool disposing)
		{
			Util.Dispose(ref components);
			base.Dispose(disposing);
		}

		/// <summary>Show/Hide the radios combo and discoverable check box</summary>
		public bool ShowRadioSelector
		{
			get { return m_cb_radio.Visible; }
			set
			{
				m_cb_radio.Visible = value;
				m_chk_discoverable.Visible = value;
			}
		}

		/// <summary>Flags for devices to display</summary>
		public Bluetooth.EOptions ShowDevices
		{
			get { return m_show_devices; }
			set
			{
				if (m_show_devices == value) return;
				m_show_devices = value;
				m_chk_show_connected    .Checked = m_show_devices.HasFlag(Bluetooth.EOptions.ReturnConnected);
				m_chk_show_paired.Checked = m_show_devices.HasFlag(Bluetooth.EOptions.ReturnAuthenticated);
				m_chk_show_remembered   .Checked = m_show_devices.HasFlag(Bluetooth.EOptions.ReturnRemembered);
				m_chk_show_unknown      .Checked = m_show_devices.HasFlag(Bluetooth.EOptions.ReturnUnknown);
				PopulateDevices();
			}
		}
		private Bluetooth.EOptions m_show_devices;

		/// <summary>Make the bluetooth radio discoverable</summary>
		public bool Discoverable
		{
			get { return Radio?.Discoverable ?? false; }
			set
			{
				if (Radio == null) return;
				Radio.Discoverable = value;
				m_chk_discoverable.Checked = value;
				SynchronizationContext_.BeginInvokeDelayed(() => UpdateUI(), TimeSpan.FromMilliseconds(1000));
			}
		}

		/// <summary>The bluetooth radio to use.</summary>
		public Bluetooth.Radio Radio
		{
			get;
			private set
			{
				if (field == value) return;
				field = value;
				PopulateDevices();
				UpdateUI();
			}
		}

		/// <summary>The selected device</summary>
		public Bluetooth.Device Device
		{
			get;
			private set
			{
				if (field == value) return;
				field = value;
				UpdateUI();
			}
		}

		/// <summary>The image list for devices (too allow users at add custom ones)</summary>
		public ImageList DeviceImageList { get; private set; }

		/// <summary>User provided function for returning the image key for the given device</summary>
		public Func<Bluetooth.Device, string> GetDeviceImageKey { get; set; }

		/// <summary>Set up the UI</summary>
		private void SetupUI()
		{
			// Icons
			{
				DeviceImageList = new ImageList(components);
				DeviceImageList.TransparentColor = Color.Transparent;
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Miscellaneous.ToString(), Resources.bt_misc);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Computer     .ToString(), Resources.bt_laptop);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Phone        .ToString(), Resources.bt_phone);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.LanAccess    .ToString(), Resources.bt_lan_access);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Audio        .ToString(), Resources.bt_audio);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Peripheral   .ToString(), Resources.bt_peripheral);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Imaging      .ToString(), Resources.bt_imaging);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Wearable     .ToString(), Resources.bt_wearable);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Toy          .ToString(), Resources.bt_misc);
				DeviceImageList.Images.Add(Bluetooth.EClassOfDeviceMajor.Unclassified .ToString(), Resources.bt_misc);
			}

			// Radio
			m_cb_radio.ToolTip(m_tt, "Select a specific bluetooth radio");
			PopulateRadios();
			m_cb_radio.Format += (s,a) =>
			{
				var radio = a.ListItem as Bluetooth.Radio;
				a.Value = radio?.Name ?? (string)a.ListItem;
			};
			m_cb_radio.SelectedIndexChanged += (s,a) =>
			{
				Radio = m_cb_radio.SelectedItem as Bluetooth.Radio;
			};

			// Control panel
			m_btn_show_bt_cpl.Click += (s,a) =>
			{
				Process.Start(new ProcessStartInfo("control", "bthprops.cpl"));
			};

			// Discoverable
			m_chk_discoverable.ToolTip(m_tt, "Check to allow other devices to discovery this PC");
			m_chk_discoverable.Checked = Discoverable;
			m_chk_discoverable.CheckedChanged += (s,a) =>
			{
				Discoverable = m_chk_discoverable.Checked;
			};

			// Show connected devices
			m_chk_show_connected.ToolTip(m_tt, "Show devices that have connected with this system");
			m_chk_show_connected.Checked = ShowDevices.HasFlag(Bluetooth.EOptions.ReturnConnected);
			m_chk_show_connected.CheckedChanged += (s,a) =>
			{
				ShowDevices = Bit.SetBits(ShowDevices, Bluetooth.EOptions.ReturnConnected, m_chk_show_connected.Checked);
			};

			// Show authenticated devices
			m_chk_show_paired.ToolTip(m_tt, "Show devices that have paired with this system");
			m_chk_show_paired.Checked = ShowDevices.HasFlag(Bluetooth.EOptions.ReturnAuthenticated);
			m_chk_show_paired.CheckedChanged += (s,a) =>
			{
				ShowDevices = Bit.SetBits(ShowDevices, Bluetooth.EOptions.ReturnAuthenticated, m_chk_show_paired.Checked);
			};

			// Show remembered devices
			m_chk_show_remembered.ToolTip(m_tt, "Show devices that have connected with this system in the past");
			m_chk_show_remembered.Checked = ShowDevices.HasFlag(Bluetooth.EOptions.ReturnRemembered);
			m_chk_show_remembered.CheckedChanged += (s,a) =>
			{
				ShowDevices = Bit.SetBits(ShowDevices, Bluetooth.EOptions.ReturnRemembered, m_chk_show_remembered.Checked);
			};

			// Show unknown devices
			m_chk_show_unknown.ToolTip(m_tt, "Show discovered devices that have not yet connected or paired with this system");
			m_chk_show_unknown.Checked = ShowDevices.HasFlag(Bluetooth.EOptions.ReturnUnknown);
			m_chk_show_unknown.CheckedChanged += (s,a) =>
			{
				ShowDevices = Bit.SetBits(ShowDevices, Bluetooth.EOptions.ReturnUnknown, m_chk_show_unknown.Checked);
			};

			// Found devices
			m_lb_devices.SelectedIndexChanged += (s,a) =>
			{
				Device = (Bluetooth.Device)m_lb_devices.SelectedItem;
			};
			m_lb_devices.DrawItem += (s,a) =>
			{
				DrawBtDevice(a);
			};
			m_lb_devices.MouseUp += (s,a) =>
			{
				if (a.Button == MouseButtons.Right)
				{
					var idx = m_lb_devices.IndexFromPoint(a.Location);
					if (idx >= 0)
					{
						m_lb_devices.SelectedIndex = idx;
						ShowCMenu(a);
					}
				}
			};

			// Pair/Forget
			m_btn_pair.Click += (s,a) =>
			{
				if (m_btn_pair.Text == PairBtn.Disconnect)
					DisconnectDevice();
				if (m_btn_pair.Text == PairBtn.Pair)
					PairDevice();
			};

			// Timer for polling while bluetooth is disabled
			m_timer.Interval = 1000;
			m_timer.Tick += (s,a) =>
			{
				PopulateRadios();
				PopulateDevices();
			};
			m_timer.Enabled = true;
		}

		/// <summary>Update the state of UI elements</summary>
		private void UpdateUI(object sender = null, EventArgs args = null)
		{
			using (Scope.Create(() => ++m_updating_ui, () => --m_updating_ui))
			{
				var enabled = Radio != null;

				m_cb_radio           .Enabled = enabled;
				m_chk_discoverable   .Enabled = enabled;
				m_lb_devices         .Enabled = enabled;
				m_chk_show_connected .Enabled = enabled;
				m_chk_show_paired    .Enabled = enabled;
				m_chk_show_remembered.Enabled = enabled;
				m_chk_show_unknown   .Enabled = enabled;
				m_btn_pair           .Enabled = enabled;
				m_btn_ok             .Enabled = enabled;

				m_chk_discoverable.Checked = Discoverable;

				if (Device != null)
				{
					m_btn_pair.Enabled = true;
					if (Device.IsConnected)
					{
						m_btn_pair.ToolTip(m_tt, $"Disconnect from {Device.Name}");
						m_btn_pair.Text = PairBtn.Disconnect;
						m_btn_pair.Visible = true;
					}
					else if (!Device.IsPaired)
					{
						m_btn_pair.ToolTip(m_tt, $"Pair with {Device.Name}");
						m_btn_pair.Text = PairBtn.Pair;
						m_btn_pair.Visible = true;
					}
					else
					{
						m_btn_pair.Visible = false;
					}
				}
				else
				{
					m_btn_pair.Visible = false;
				}
			}
		}
		private int m_updating_ui;

		/// <summary>Create a context menu for the device list</summary>
		private void ShowCMenu(MouseEventArgs args)
		{
			var cmenu = new ContextMenuStrip();
			if (Device.IsConnected)
			{
				var opt = cmenu.Items.Add2(new ToolStripMenuItem("Disconnect"));
				opt.Click += (s,a) => DisconnectDevice();
			}
			if (!Device.IsPaired)
			{
				var opt = cmenu.Items.Add2(new ToolStripMenuItem("Pair"));
				opt.Click += (s,a) => PairDevice();
			}
			if (Device.IsRemembered)
			{
				var opt = cmenu.Items.Add2(new ToolStripMenuItem("Forget"));
				opt.Click += (s,a) => ForgetDevice();
			}
			cmenu.Show(m_lb_devices, args.Location);
		}

		/// <summary>Populate the combo of BT radios</summary>
		private void PopulateRadios()
		{
			// Get the available radios
			var radios = Bluetooth.Radios().ToList();
			if (radios.Count != 0)
				radios.Insert(0, new Bluetooth.Radio(new SafeFileHandle(IntPtr.Zero, false)));

			// Update the combo of radios
			if (radios.Count == 0)
			{
				m_cb_radio.DataSource = new[] { "Bluetooth Disabled" };
				Radio = null;
			}
			else
			{
				// Compare to the existing list so the combo isn't changed unnecessarily
				var curr = m_cb_radio.DataSource as List<Bluetooth.Radio>;
				if (curr == null || !curr.SequenceEqual(radios, Cmp<Bluetooth.Radio>.From((l,r) => l.Name.CompareTo(r.Name))))
				{
					m_cb_radio.DataSource = radios;

					// Select the same radio again
					var name = Radio?.Name;
					Radio = radios.FirstOrDefault(x => x.Name == name) ?? radios[0];
				}
			}
		}

		/// <summary>Populate the list view of bluetooth devices</summary>
		private void PopulateDevices()
		{
			var devices = Bluetooth.Devices(ShowDevices).ToList();
			var curr = m_lb_devices.DataSource as List<Bluetooth.Device>;

			// Update the list of devices
			if (curr == null || !curr.SequenceEqual(devices, Cmp<Bluetooth.Device>.From((l,r) => l.Name.CompareTo(r.Name))))
			{
				m_lb_devices.DataSource = devices;

				// Select the same device again
				var name = Device?.Name;
				Device = devices.FirstOrDefault(x => x.Name == name);
			}
		}

		/// <summary>Draw the bluetooth device item in the list view</summary>
		private void DrawBtDevice(DrawItemEventArgs a)
		{
			if (a.Index < 0 || a.Index >= m_lb_devices.Items.Count) return;
			var device = (Bluetooth.Device)m_lb_devices.Items[a.Index];
			var x = a.Bounds.Left + 1;
			var y = a.Bounds.Top + 1;

			// Get the image key for the device type
			var img_key = GetDeviceImageKey != null
				? GetDeviceImageKey(device)
				: device.ClassOfDeviceMajor.ToString();
			var img = DeviceImageList.Images.ContainsKey(img_key)
				? DeviceImageList.Images[img_key]
				: DeviceImageList.Images[Bluetooth.EClassOfDeviceMajor.Miscellaneous.ToString()];

			// Cell background
			if (a.State.HasFlag(DrawItemState.Selected))
				using (var bsh = new SolidBrush(Color_.FromArgb(0xfff0fafe)))
					a.Graphics.FillRectangle(bsh, a.Bounds);
			else
				a.DrawBackground();

			// Device image
			using (a.Graphics.SaveState())
			{
				a.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
				a.Graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;
				a.Graphics.PixelOffsetMode = PixelOffsetMode.HighQuality;

				var aspect = (float)img.Width / img.Height;
				var cx = aspect * (a.Bounds.Height - 2);
				var cy = 1f     * (a.Bounds.Height - 2);

				a.Graphics.DrawImage(img, new RectangleF(x, y, cx, cy));
				x = (int)(x + cx + 4);
			}

			// Device name
			using (var font = m_lb_devices.Font.Dup(em_size:12f, style:FontStyle.Bold))
			{
				a.Graphics.DrawString(device.Name, font, Brushes.Black, new Point(x, y));
				y += font.Height;
			}

			// Device state
			using (var font = m_lb_devices.Font.Dup(em_size: 8f))
			{
				a.Graphics.DrawString(device.StatusString, font, Brushes.Blue, new Point(x, y));
				y += font.Height;
			}

			// Last used/seen
			using (var font = m_lb_devices.Font.Dup(em_size: 8f))
			{
				var s = $"Last Used: {device.LastUsed.ToString("G", CultureInfo.CurrentCulture)}";
				a.Graphics.DrawString(s, font, Brushes.Gray, new Point(x, y));
				y += font.Height;

				s = $"Last Seen: {device.LastSeen.ToString("G", CultureInfo.CurrentCulture)}";
				a.Graphics.DrawString(s, font, Brushes.Gray, new Point(x, y));
				y += font.Height;
			}

			// Focus rect
			a.DrawFocusRectangle();
		}

		/// <summary>Pair the currently selected device</summary>
		private void PairDevice()
		{
			if (Device == null) return;
			try
			{
				Device.Pair(Handle, Radio);
			}
			catch (Exception ex)
			{
				MsgBox.Show(this, $"Pairing Failed\r\n{ex.Message}", "Bluetooth Pairing", MessageBoxButtons.OK, MessageBoxIcon.Error);
			}
		}

		/// <summary>Forget the currently selected device</summary>
		private void ForgetDevice()
		{
			try
			{
				Device.Forget();
			}
			catch (Exception ex)
			{
				MsgBox.Show(this, ex.Message, "Bluetooth", MessageBoxButtons.OK, MessageBoxIcon.Error);
			}
		}

		/// <summary>Disconnect a connected bluetooth device</summary>
		private void DisconnectDevice()
		{
			try
			{
				Device.Disconnect();
			}
			catch (Exception ex)
			{
				MsgBox.Show(this, ex.Message, "Bluetooth", MessageBoxButtons.OK, MessageBoxIcon.Error);
			}
		}

		private class PairBtn
		{
			public const string Disconnect = "Disconnect";
			public const string Pair = "Pair";
		}
		#region Windows Form Designer generated code
		private System.ComponentModel.IContainer components = null;
		private void InitializeComponent()
		{
			this.components = new System.ComponentModel.Container();
			this.m_btn_cancel = new System.Windows.Forms.Button();
			this.m_btn_ok = new System.Windows.Forms.Button();
			this.m_btn_pair = new System.Windows.Forms.Button();
			this.m_tt = new System.Windows.Forms.ToolTip(this.components);
			this.m_chk_show_unknown = new System.Windows.Forms.CheckBox();
			this.m_chk_show_remembered = new System.Windows.Forms.CheckBox();
			this.m_chk_show_connected = new System.Windows.Forms.CheckBox();
			this.m_chk_show_paired = new System.Windows.Forms.CheckBox();
			this.m_chk_discoverable = new System.Windows.Forms.CheckBox();
			this.m_lb_devices = new Rylogic.Gui.WinForms.ListBox();
			this.m_btn_show_bt_cpl = new System.Windows.Forms.Button();
			this.m_timer = new System.Windows.Forms.Timer(this.components);
			this.m_cb_radio = new Rylogic.Gui.WinForms.ComboBox();
			this.SuspendLayout();
			// 
			// m_btn_cancel
			// 
			this.m_btn_cancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.m_btn_cancel.DialogResult = System.Windows.Forms.DialogResult.Cancel;
			this.m_btn_cancel.Location = new System.Drawing.Point(267, 268);
			this.m_btn_cancel.Name = "m_btn_cancel";
			this.m_btn_cancel.Size = new System.Drawing.Size(75, 23);
			this.m_btn_cancel.TabIndex = 3;
			this.m_btn_cancel.Text = "Cancel";
			this.m_btn_cancel.UseVisualStyleBackColor = true;
			// 
			// m_btn_ok
			// 
			this.m_btn_ok.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.m_btn_ok.DialogResult = System.Windows.Forms.DialogResult.OK;
			this.m_btn_ok.Location = new System.Drawing.Point(186, 268);
			this.m_btn_ok.Name = "m_btn_ok";
			this.m_btn_ok.Size = new System.Drawing.Size(75, 23);
			this.m_btn_ok.TabIndex = 4;
			this.m_btn_ok.Text = "OK";
			this.m_btn_ok.UseVisualStyleBackColor = true;
			// 
			// m_btn_pair
			// 
			this.m_btn_pair.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
			this.m_btn_pair.Location = new System.Drawing.Point(12, 268);
			this.m_btn_pair.Name = "m_btn_pair";
			this.m_btn_pair.Size = new System.Drawing.Size(75, 23);
			this.m_btn_pair.TabIndex = 5;
			this.m_btn_pair.Text = "Pair";
			this.m_btn_pair.UseVisualStyleBackColor = true;
			// 
			// m_chk_show_unknown
			// 
			this.m_chk_show_unknown.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.m_chk_show_unknown.AutoSize = true;
			this.m_chk_show_unknown.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
			this.m_chk_show_unknown.Location = new System.Drawing.Point(198, 245);
			this.m_chk_show_unknown.Name = "m_chk_show_unknown";
			this.m_chk_show_unknown.Size = new System.Drawing.Size(144, 17);
			this.m_chk_show_unknown.TabIndex = 6;
			this.m_chk_show_unknown.Text = "Show Unknown Devices";
			this.m_chk_show_unknown.UseVisualStyleBackColor = true;
			// 
			// m_chk_show_remembered
			// 
			this.m_chk_show_remembered.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.m_chk_show_remembered.AutoSize = true;
			this.m_chk_show_remembered.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
			this.m_chk_show_remembered.Location = new System.Drawing.Point(181, 227);
			this.m_chk_show_remembered.Name = "m_chk_show_remembered";
			this.m_chk_show_remembered.Size = new System.Drawing.Size(161, 17);
			this.m_chk_show_remembered.TabIndex = 7;
			this.m_chk_show_remembered.Text = "Show Remembered Devices";
			this.m_chk_show_remembered.UseVisualStyleBackColor = true;
			// 
			// m_chk_show_connected
			// 
			this.m_chk_show_connected.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.m_chk_show_connected.AutoSize = true;
			this.m_chk_show_connected.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
			this.m_chk_show_connected.Location = new System.Drawing.Point(25, 227);
			this.m_chk_show_connected.Name = "m_chk_show_connected";
			this.m_chk_show_connected.Size = new System.Drawing.Size(150, 17);
			this.m_chk_show_connected.TabIndex = 8;
			this.m_chk_show_connected.Text = "Show Connected Devices";
			this.m_chk_show_connected.UseVisualStyleBackColor = true;
			// 
			// m_chk_show_paired
			// 
			this.m_chk_show_paired.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.m_chk_show_paired.AutoSize = true;
			this.m_chk_show_paired.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
			this.m_chk_show_paired.Location = new System.Drawing.Point(47, 245);
			this.m_chk_show_paired.Name = "m_chk_show_paired";
			this.m_chk_show_paired.Size = new System.Drawing.Size(128, 17);
			this.m_chk_show_paired.TabIndex = 9;
			this.m_chk_show_paired.Text = "Show Paired Devices";
			this.m_chk_show_paired.UseVisualStyleBackColor = true;
			// 
			// m_chk_discoverable
			// 
			this.m_chk_discoverable.AutoSize = true;
			this.m_chk_discoverable.Location = new System.Drawing.Point(154, 16);
			this.m_chk_discoverable.Name = "m_chk_discoverable";
			this.m_chk_discoverable.Size = new System.Drawing.Size(88, 17);
			this.m_chk_discoverable.TabIndex = 10;
			this.m_chk_discoverable.Text = "Discoverable";
			this.m_chk_discoverable.UseVisualStyleBackColor = true;
			// 
			// m_lb_devices
			// 
			this.m_lb_devices.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
			this.m_lb_devices.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
			this.m_lb_devices.DrawMode = System.Windows.Forms.DrawMode.OwnerDrawFixed;
			this.m_lb_devices.Font = new System.Drawing.Font("Tahoma", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.m_lb_devices.FormattingEnabled = true;
			this.m_lb_devices.IntegralHeight = false;
			this.m_lb_devices.ItemHeight = 64;
			this.m_lb_devices.Location = new System.Drawing.Point(10, 39);
			this.m_lb_devices.Name = "m_lb_devices";
			this.m_lb_devices.Size = new System.Drawing.Size(332, 182);
			this.m_lb_devices.TabIndex = 12;
			// 
			// m_btn_show_bt_cpl
			// 
			this.m_btn_show_bt_cpl.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
			this.m_btn_show_bt_cpl.Location = new System.Drawing.Point(248, 12);
			this.m_btn_show_bt_cpl.Name = "m_btn_show_bt_cpl";
			this.m_btn_show_bt_cpl.Size = new System.Drawing.Size(95, 23);
			this.m_btn_show_bt_cpl.TabIndex = 13;
			this.m_btn_show_bt_cpl.Text = "Control Panel";
			this.m_btn_show_bt_cpl.UseVisualStyleBackColor = true;
			// 
			// m_timer
			// 
			this.m_timer.Interval = 1000;
			// 
			// m_cb_radio
			// 
			this.m_cb_radio.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.m_cb_radio.FormattingEnabled = true;
			this.m_cb_radio.Location = new System.Drawing.Point(9, 14);
			this.m_cb_radio.Name = "m_cb_radio";
			this.m_cb_radio.PreserveSelectionThruFocusChange = false;
			this.m_cb_radio.Size = new System.Drawing.Size(139, 21);
			this.m_cb_radio.TabIndex = 11;
			// 
			// BluetoothUI
			// 
			this.AcceptButton = this.m_btn_ok;
			this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.CancelButton = this.m_btn_cancel;
			this.ClientSize = new System.Drawing.Size(354, 303);
			this.Controls.Add(this.m_btn_show_bt_cpl);
			this.Controls.Add(this.m_lb_devices);
			this.Controls.Add(this.m_cb_radio);
			this.Controls.Add(this.m_chk_discoverable);
			this.Controls.Add(this.m_chk_show_paired);
			this.Controls.Add(this.m_chk_show_connected);
			this.Controls.Add(this.m_chk_show_remembered);
			this.Controls.Add(this.m_chk_show_unknown);
			this.Controls.Add(this.m_btn_pair);
			this.Controls.Add(this.m_btn_ok);
			this.Controls.Add(this.m_btn_cancel);
			this.Icon = Resources.bluetooth_ico;
			this.MinimumSize = new System.Drawing.Size(370, 200);
			this.Name = "BluetoothUI";
			this.Text = "Choose a Bluetooth Device";
			this.ResumeLayout(false);
			this.PerformLayout();

		}
		#endregion
	}
}
