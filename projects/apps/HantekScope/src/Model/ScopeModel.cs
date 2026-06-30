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
	/// </summary>
	public sealed class ScopeModel :IDisposable
	{
		// Acquisition runs with both channels enabled (the init sequence turns CH2
		// on) and at timebase index 14, which fixes the per-sample time step.
		private const int TimebaseIndex = 14;
		private const int VoltsDivIndex = 6;
		private const bool TwoChannels = true;

		private readonly object m_sync = new();
		private readonly List<Sample> m_pending = new();
		private HantekDevice? m_device;
		private Thread? m_thread;
		private volatile bool m_run;
		private long m_sample_index;

		/// <summary>Seconds per sample at the configured timebase (dt = timebase / 100).</summary>
		public double SampleIntervalS => HantekProtocol.TimebaseTable[TimebaseIndex] / 100.0;

		/// <summary>Volts-per-division currently configured (for the Y scale).</summary>
		public double VoltsPerDiv => HantekProtocol.VoltsDivTable[VoltsDivIndex];

		/// <summary>True while the acquisition thread is running.</summary>
		public bool IsRunning => m_run;

		/// <summary>Raised (on the acquisition thread) with a human-readable status or error.</summary>
		public event Action<string>? StatusChanged;

		/// <summary>Raised (on the acquisition thread) once identity has been read after opening.</summary>
		public event Action<string, string, string>? IdentityRead;

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
		/// packets at the device's pace and decode them into pending samples.
		/// </summary>
		private void AcquisitionLoop()
		{
			try
			{
				m_device = new HantekDevice();
				m_device.Open();
				StatusChanged?.Invoke("Initialising…");

				m_device.RunInitSequence();

				var id = m_device.ReadIdentity();
				IdentityRead?.Invoke(id.Serial, id.Firmware, id.Version);
				StatusChanged?.Invoke($"Running — {id.Serial} fw {id.Firmware}");

				var dt_ms = SampleIntervalS * 1000.0;
				var vdiv = VoltsPerDiv;

				while (m_run)
				{
					var packet = m_device.ReadWaveformPacket();
					if (packet.Length != 0)
						DecodePacket(packet, dt_ms, vdiv);

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

		/// <summary>Deinterleave a packet, convert codes to volts, and queue the samples.</summary>
		private void DecodePacket(byte[] packet, double dt_ms, double vdiv)
		{
			var (ch1, ch2) = HantekDevice.Deinterleave(packet, TwoChannels);
			var count = ch1.Count;

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
