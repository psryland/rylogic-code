using System;
using System.Collections.Generic;
using System.Threading;
using HantekScope.Device;

namespace HantekScope.Model
{
	/// <summary>One acquired sample column: a time position plus each channel's volts.</summary>
	public readonly record struct Sample(double XMs, double Ch1V, double Ch2V);

	/// <summary>
	/// Owns the Hantek device and a background acquisition thread. The thread polls
	/// waveform packets at the device's preferred pace and pushes decoded samples
	/// into a thread-safe pending buffer; the UI drains that buffer on its own
	/// render timer, so acquisition rate and render frame-rate are decoupled.
	///
	/// Vertical (volts/div) and horizontal (timebase) resolution can be changed live:
	/// the UI requests a desired register index from any thread, and the acquisition
	/// thread applies the change with a register write before the next read, keeping
	/// all USB I/O on the one thread.
	/// </summary>
	public sealed class ScopeModel :IDisposable
	{
		// Default acquisition resolution: timebase index 14 (200us/div) and volts/div
		// index 6 (1V), matching the init sequence the device is left in.
		private const int DefaultTimebaseIndex = 14;
		private const int DefaultVoltsDivIndex = 6;
		private const bool TwoChannels = true;

		private readonly object m_sync = new();
		private readonly List<Sample> m_pending = new();

		// Resolution indices. 'desired' is what the UI has asked for (writable from any
		// thread under 'm_sync'); 'applied' is what the device is currently programmed
		// to (only the acquisition thread touches it). The loop reconciles the two.
		private int m_desired_timebase_index = DefaultTimebaseIndex;
		private int m_desired_volts_div_index = DefaultVoltsDivIndex;
		private volatile int m_applied_timebase_index = DefaultTimebaseIndex;
		private volatile int m_applied_volts_div_index = DefaultVoltsDivIndex;

		private HantekDevice? m_device;
		private Thread? m_thread;
		private volatile bool m_run;
		private long m_sample_index;

		/// <summary>Seconds per sample at the applied timebase (dt = timebase / 100).</summary>
		public double SampleIntervalS => HantekProtocol.TimebaseTable[m_applied_timebase_index] / HantekProtocol.SamplesPerDivision;

		/// <summary>Volts-per-division currently applied (for the Y scale).</summary>
		public double VoltsPerDiv => HantekProtocol.VoltsDivTable[m_applied_volts_div_index];

		/// <summary>The applied volts/div register index.</summary>
		public int VoltsDivIndex => m_applied_volts_div_index;

		/// <summary>The applied timebase register index.</summary>
		public int TimebaseIndex => m_applied_timebase_index;

		/// <summary>True while the acquisition thread is running.</summary>
		public bool IsRunning => m_run;

		/// <summary>Raised (on the acquisition thread) with a human-readable status or error.</summary>
		public event Action<string>? StatusChanged;

		/// <summary>Raised (on the acquisition thread) once identity has been read after opening.</summary>
		public event Action<string, string, string>? IdentityRead;

		/// <summary>Raised (on the acquisition thread) after the applied resolution changes.</summary>
		public event Action? ConfigChanged;

		/// <summary>
		/// Request a volts/div register index. Safe to call from any thread; the change
		/// is applied by the acquisition thread before its next read. Out-of-range
		/// indices are ignored so the UI can pass a snapped value without re-checking.
		/// </summary>
		public void RequestVoltsDivIndex(int index)
		{
			if (!HantekProtocol.VoltsDivTable.ContainsKey(index))
				return;

			lock (m_sync)
				m_desired_volts_div_index = index;
		}

		/// <summary>Request a timebase register index. Safe to call from any thread (see RequestVoltsDivIndex).</summary>
		public void RequestTimebaseIndex(int index)
		{
			if (!HantekProtocol.TimebaseTable.ContainsKey(index))
				return;

			lock (m_sync)
				m_desired_timebase_index = index;
		}

		/// <summary>
		/// Open the device and start acquiring. Opening + the init sequence run on
		/// the background thread because they perform blocking USB I/O.
		/// </summary>
		public void Start()
		{
			if (m_run)
				return;

			m_run = true;
			m_thread = new Thread(AcquisitionLoop) { IsBackground = true, Name = "HantekAcquisition" };
			m_thread.Start();
		}

		/// <summary>Stop acquiring and close the device.</summary>
		public void Stop()
		{
			m_run = false;
			m_thread?.Join(2000);
			m_thread = null;

			m_device?.Dispose();
			m_device = null;
		}

		/// <summary>
		/// Move all samples accumulated since the last call out of the pending buffer.
		/// Called by the UI render timer; returns quickly under a short lock.
		/// </summary>
		public List<Sample> DrainPending()
		{
			lock (m_sync)
			{
				if (m_pending.Count == 0)
					return new List<Sample>();

				var drained = new List<Sample>(m_pending);
				m_pending.Clear();
				return drained;
			}
		}

		/// <summary>Release resources.</summary>
		public void Dispose()
		{
			Stop();
		}

		/// <summary>
		/// Background thread: open, initialise, read identity, then poll waveform
		/// packets at the device's pace, applying any pending resolution change before
		/// each read and decoding the result into pending samples.
		/// </summary>
		private void AcquisitionLoop()
		{
			try
			{
				m_device = new HantekDevice();
				m_device.Open();
				StatusChanged?.Invoke("Initialising…");

				m_device.RunInitSequence();

				// The init sequence leaves the device at the default resolution.
				m_applied_timebase_index = DefaultTimebaseIndex;
				m_applied_volts_div_index = DefaultVoltsDivIndex;

				var id = m_device.ReadIdentity();
				IdentityRead?.Invoke(id.Serial, id.Firmware, id.Version);
				StatusChanged?.Invoke($"Running — {id.Serial} fw {id.Firmware}");

				while (m_run)
				{
					ApplyPendingConfig();

					var packet = m_device.ReadWaveformPacket();
					if (packet.Length != 0)
						DecodePacket(packet);

					Thread.Sleep(HantekProtocol.WaveformPollIntervalMs);
				}
			}
			catch (Exception ex)
			{
				m_run = false;
				StatusChanged?.Invoke($"Error: {ex.Message}");
			}
			finally
			{
				m_device?.Dispose();
				m_device = null;
			}
		}

		/// <summary>
		/// Reconcile the applied resolution with the latest UI request by writing the
		/// relevant registers. Runs on the acquisition thread between reads so the
		/// device is reprogrammed without contending with the UI thread.
		/// </summary>
		private void ApplyPendingConfig()
		{
			if (m_device == null)
				return;

			int desired_vdiv, desired_timebase;
			lock (m_sync)
			{
				desired_vdiv = m_desired_volts_div_index;
				desired_timebase = m_desired_timebase_index;
			}

			var changed = false;

			// Volts/div is per-channel; program both so the two traces share a scale.
			if (desired_vdiv != m_applied_volts_div_index)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch1VoltsDiv, desired_vdiv);
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch2VoltsDiv, desired_vdiv);
				m_applied_volts_div_index = desired_vdiv;
				changed = true;
			}

			if (desired_timebase != m_applied_timebase_index)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Timebase, desired_timebase);
				m_applied_timebase_index = desired_timebase;
				changed = true;
			}

			// A resolution change restarts the time origin so framed displays redraw
			// from a clean t=0 at the new scale.
			if (changed)
			{
				m_sample_index = 0;
				ConfigChanged?.Invoke();
			}
		}

		/// <summary>Deinterleave a packet, convert codes to volts, and queue the samples.</summary>
		private void DecodePacket(byte[] packet)
		{
			var (ch1, ch2) = HantekDevice.Deinterleave(packet, TwoChannels);
			var count = ch1.Count;

			// Snapshot the applied scale for this packet so all samples decode coherently.
			var dt_ms = SampleIntervalS * 1000.0;
			var vdiv = VoltsPerDiv;

			var batch = new List<Sample>(count);
			for (var i = 0; i != count; ++i)
			{
				// X advances by the running sample index so consecutive packets form
				// one contiguous stream on the acquisition-time axis.
				var x_ms = (m_sample_index + i) * dt_ms;
				var v1 = HantekProtocol.CodeToDivisions(ch1[i]) * vdiv;
				var v2 = i < ch2.Count ? HantekProtocol.CodeToDivisions(ch2[i]) * vdiv : double.NaN;
				batch.Add(new Sample(x_ms, v1, v2));
			}
			m_sample_index += count;

			lock (m_sync)
			{
				m_pending.AddRange(batch);
			}
		}
	}
}
