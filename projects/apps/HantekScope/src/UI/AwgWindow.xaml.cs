using System;
using System.Globalization;
using System.Windows;
using System.Windows.Input;
using HantekScope.Device;
using HantekScope.Model;

namespace HantekScope.UI
{
	/// <summary>
	/// Non-modal control panel for the scope's built-in DDS waveform generator. It
	/// exposes the four generator settings that take effect while the device streams in
	/// DSO mode — waveform shape, frequency, peak-to-peak amplitude and (square-wave)
	/// duty cycle — and pushes each change to the ScopeModel, which writes the generator
	/// registers on its acquisition thread. DC offset and output-enable are deliberately
	/// omitted: they only work from the device's DDS mode, which the live trace can't use.
	/// </summary>
	public partial class AwgWindow :Window
	{
		private readonly ScopeModel m_model;

		// Frequency unit multipliers, parallel to the unit combo's items.
		private static readonly (string Label, double Factor)[] FrequencyUnits =
		{
			("Hz", 1.0),
			("kHz", 1_000.0),
			("MHz", 1_000_000.0),
		};

		// Set while the controls are being seeded from the model so the change handlers
		// don't echo those programmatic updates straight back as new requests.
		private bool m_loading;

		public AwgWindow(ScopeModel model)
		{
			m_model = model;
			InitializeComponent();

			// Populate the fixed choice lists. The waveform enum names (Square/Ramp/Sine/
			// Trapezia/Arbitrary) read well directly, so they double as the display text.
			m_cmb_waveform.ItemsSource = Enum.GetValues<EDdsWaveform>();

			var unit_labels = new string[FrequencyUnits.Length];
			for (var i = 0; i != FrequencyUnits.Length; ++i)
				unit_labels[i] = FrequencyUnits[i].Label;
			m_cmb_freq_unit.ItemsSource = unit_labels;

			SeedFromModel();
		}

		/// <summary>Load the current generator settings from the model into the controls.</summary>
		private void SeedFromModel()
		{
			m_loading = true;
			try
			{
				m_cmb_waveform.SelectedItem = m_model.DdsWaveform;

				// Choose a display unit that keeps the shown frequency value readable, then
				// fill the value box to match.
				var unit_index = ChooseFrequencyUnit(m_model.DdsFrequencyHz);
				m_cmb_freq_unit.SelectedIndex = unit_index;
				m_txt_frequency.Text = FormatFrequency(m_model.DdsFrequencyHz, unit_index);

				m_txt_amplitude.Text = m_model.DdsAmplitudeVpp.ToString("0.###", CultureInfo.InvariantCulture);

				m_sld_duty.Value = m_model.DdsDutyPercent;
				UpdateDutyLabel();

				UpdateDutyEnabled();
			}
			finally
			{
				m_loading = false;
			}
		}

		/// <summary>Pick the largest frequency unit that keeps the displayed value at least 1.</summary>
		private static int ChooseFrequencyUnit(long hz)
		{
			// Walk from the largest unit down; the first whose factor divides into a value
			// of 1 or more gives the most compact readable display (e.g. 1000 Hz -> 1 kHz).
			for (var i = FrequencyUnits.Length - 1; i > 0; --i)
			{
				if (hz >= FrequencyUnits[i].Factor)
					return i;
			}
			return 0;
		}

		/// <summary>Format a frequency in Hz as a value string in the given unit.</summary>
		private static string FormatFrequency(long hz, int unit_index)
		{
			var value = hz / FrequencyUnits[unit_index].Factor;
			return value.ToString("0.######", CultureInfo.InvariantCulture);
		}

		/// <summary>Send the current waveform selection to the model.</summary>
		private void OnWaveformChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
		{
			if (m_loading || m_cmb_waveform.SelectedItem is not EDdsWaveform waveform)
				return;

			m_model.RequestDdsWaveform(waveform);
			UpdateDutyEnabled();
		}

		/// <summary>Commit the frequency when the unit changes (the value box is unit-relative).</summary>
		private void OnFrequencyUnitChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
		{
			if (m_loading)
				return;

			CommitFrequency();
		}

		/// <summary>Commit the frequency on Enter so it doesn't wait for focus to leave.</summary>
		private void OnFrequencyKeyDown(object sender, KeyEventArgs e)
		{
			if (e.Key == Key.Enter)
				CommitFrequency();
		}

		/// <summary>Commit the frequency when the value box loses focus.</summary>
		private void OnFrequencyCommitted(object sender, RoutedEventArgs e)
		{
			CommitFrequency();
		}

		/// <summary>Parse the value box in the selected unit and request the frequency in Hz.</summary>
		private void CommitFrequency()
		{
			if (m_loading)
				return;

			// Ignore unparsable text (leave the last valid value in place); the box is
			// re-normalised from the model's clamped value so it can't drift out of range.
			if (!double.TryParse(m_txt_frequency.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
				return;

			var unit_index = Math.Max(0, m_cmb_freq_unit.SelectedIndex);
			var hz = (long)Math.Round(value * FrequencyUnits[unit_index].Factor);
			m_model.RequestDdsFrequency(hz);

			// Reflect the clamped value the model actually accepted.
			m_loading = true;
			m_txt_frequency.Text = FormatFrequency(m_model.DdsFrequencyHz, unit_index);
			m_loading = false;
		}

		/// <summary>Commit the amplitude on Enter.</summary>
		private void OnAmplitudeKeyDown(object sender, KeyEventArgs e)
		{
			if (e.Key == Key.Enter)
				CommitAmplitude();
		}

		/// <summary>Commit the amplitude when its box loses focus.</summary>
		private void OnAmplitudeCommitted(object sender, RoutedEventArgs e)
		{
			CommitAmplitude();
		}

		/// <summary>Parse the amplitude box (Vpp) and request it, then reflect the clamped value.</summary>
		private void CommitAmplitude()
		{
			if (m_loading)
				return;

			if (!double.TryParse(m_txt_amplitude.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var vpp))
				return;

			m_model.RequestDdsAmplitudeVpp(vpp);

			m_loading = true;
			m_txt_amplitude.Text = m_model.DdsAmplitudeVpp.ToString("0.###", CultureInfo.InvariantCulture);
			m_loading = false;
		}

		/// <summary>Send the duty-cycle slider value to the model as the user drags it.</summary>
		private void OnDutyChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
		{
			// The slider raises ValueChanged once during XAML load (its Minimum coerces the
			// default value) before the named fields are assigned, so ignore that early call.
			if (m_lbl_duty == null)
				return;

			UpdateDutyLabel();

			if (m_loading)
				return;

			m_model.RequestDdsDuty((int)Math.Round(m_sld_duty.Value));
		}

		/// <summary>Keep the duty read-out in step with the slider.</summary>
		private void UpdateDutyLabel()
		{
			m_lbl_duty.Text = $"{(int)Math.Round(m_sld_duty.Value)} %";
		}

		/// <summary>Duty cycle only applies to the square wave, so disable it for other shapes.</summary>
		private void UpdateDutyEnabled()
		{
			var is_square = m_cmb_waveform.SelectedItem is EDdsWaveform.Square;
			m_sld_duty.IsEnabled = is_square;
			m_lbl_duty.Opacity = is_square ? 1.0 : 0.45;
		}
	}
}
