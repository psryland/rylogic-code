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
	/// All live-adjustable scope settings (per-channel enable/coupling/probe, shared
	/// volts-div and timebase, and the trigger) follow a desired/applied pattern: the
	/// UI mutates the 'desired' config from any thread under a lock, and the
	/// acquisition thread reconciles it into the 'applied' config with register writes
	/// before each read, keeping all USB I/O on the one thread.
	/// </summary>
	public sealed class ScopeModel :IDisposable
	{
		// Default acquisition resolution: timebase index 14 (200us/div) and volts/div
		// index 6 (1V), matching the init sequence the device is left in.
		private const int DefaultTimebaseIndex = 14;
		private const int DefaultVoltsDivIndex = 6;

		private readonly object m_sync = new();
		private readonly List<Sample> m_pending = new();

		// 'm_desired' is what the UI has asked for (written from any thread under
		// 'm_sync'); 'm_applied' is what the device is currently programmed to (owned by
		// the acquisition thread, published back under 'm_sync' for the UI's read-side
		// props). The loop reconciles the two before each waveform read.
		private Config m_desired = Config.Default;
		private Config m_applied = Config.Default;

		private HantekDevice? m_device;
		private Thread? m_thread;
		private volatile bool m_run;
		private long m_sample_index;

		/// <summary>Seconds per sample at the applied timebase (dt = timebase / 100).</summary>
		public double SampleIntervalS
		{
			get { lock (m_sync) return HantekProtocol.TimebaseTable[m_applied.TimebaseIndex] / HantekProtocol.SamplesPerDivision; }
		}

		/// <summary>Volts-per-division currently applied (for the Y scale).</summary>
		public double VoltsPerDiv
		{
			get { lock (m_sync) return HantekProtocol.VoltsDivTable[m_applied.VoltsDivIndex]; }
		}

		/// <summary>The applied volts/div register index.</summary>
		public int VoltsDivIndex
		{
			get { lock (m_sync) return m_applied.VoltsDivIndex; }
		}

		/// <summary>The applied timebase register index.</summary>
		public int TimebaseIndex
		{
			get { lock (m_sync) return m_applied.TimebaseIndex; }
		}

		/// <summary>Desired CH1 enabled state (reflects the user's selection).</summary>
		public bool Ch1Enabled
		{
			get { lock (m_sync) return m_desired.Ch1Enabled; }
		}

		/// <summary>Desired CH2 enabled state (reflects the user's selection).</summary>
		public bool Ch2Enabled
		{
			get { lock (m_sync) return m_desired.Ch2Enabled; }
		}

		/// <summary>Desired CH1 input coupling.</summary>
		public ECoupling Ch1Coupling
		{
			get { lock (m_sync) return m_desired.Ch1Coupling; }
		}

		/// <summary>Desired CH2 input coupling.</summary>
		public ECoupling Ch2Coupling
		{
			get { lock (m_sync) return m_desired.Ch2Coupling; }
		}

		/// <summary>Desired CH1 probe attenuation.</summary>
		public EProbeScale Ch1Probe
		{
			get { lock (m_sync) return m_desired.Ch1Probe; }
		}

		/// <summary>Desired CH2 probe attenuation.</summary>
		public EProbeScale Ch2Probe
		{
			get { lock (m_sync) return m_desired.Ch2Probe; }
		}

		/// <summary>Desired trigger source channel.</summary>
		public ETriggerSource TriggerSource
		{
			get { lock (m_sync) return m_desired.TriggerSource; }
		}

		/// <summary>Desired trigger edge slope.</summary>
		public ETriggerSlope TriggerSlope
		{
			get { lock (m_sync) return m_desired.TriggerSlope; }
		}

		/// <summary>Desired trigger sweep mode.</summary>
		public ETriggerSweep TriggerSweep
		{
			get { lock (m_sync) return m_desired.TriggerSweep; }
		}

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
				m_desired.VoltsDivIndex = index;
		}

		/// <summary>Request a timebase register index. Safe to call from any thread (see RequestVoltsDivIndex).</summary>
		public void RequestTimebaseIndex(int index)
		{
			if (!HantekProtocol.TimebaseTable.ContainsKey(index))
				return;

			lock (m_sync)
				m_desired.TimebaseIndex = index;
		}

		/// <summary>
		/// Request a channel's enabled state. At least one channel must stay enabled
		/// (the waveform stream needs a source), so a request to disable the last
		/// enabled channel is ignored.
		/// </summary>
		public void RequestChannelEnabled(int channel, bool on)
		{
			lock (m_sync)
			{
				switch (channel)
				{
					case 1:
					{
						if (!on && !m_desired.Ch2Enabled)
							return;
						m_desired.Ch1Enabled = on;
						break;
					}
					case 2:
					{
						if (!on && !m_desired.Ch1Enabled)
							return;
						m_desired.Ch2Enabled = on;
						break;
					}
					default:
						throw new ArgumentOutOfRangeException(nameof(channel));
				}
			}
		}

		/// <summary>Request a channel's input coupling.</summary>
		public void RequestCoupling(int channel, ECoupling coupling)
		{
			lock (m_sync)
			{
				switch (channel)
				{
					case 1: m_desired.Ch1Coupling = coupling; break;
					case 2: m_desired.Ch2Coupling = coupling; break;
					default: throw new ArgumentOutOfRangeException(nameof(channel));
				}
			}
		}

		/// <summary>Request a channel's probe attenuation.</summary>
		public void RequestProbe(int channel, EProbeScale probe)
		{
			lock (m_sync)
			{
				switch (channel)
				{
					case 1: m_desired.Ch1Probe = probe; break;
					case 2: m_desired.Ch2Probe = probe; break;
					default: throw new ArgumentOutOfRangeException(nameof(channel));
				}
			}
		}

		/// <summary>Request the trigger source channel.</summary>
		public void RequestTriggerSource(ETriggerSource source)
		{
			lock (m_sync)
				m_desired.TriggerSource = source;
		}

		/// <summary>Request the trigger edge slope.</summary>
		public void RequestTriggerSlope(ETriggerSlope slope)
		{
			lock (m_sync)
				m_desired.TriggerSlope = slope;
		}

		/// <summary>Request the trigger sweep mode.</summary>
		public void RequestTriggerSweep(ETriggerSweep sweep)
		{
			lock (m_sync)
				m_desired.TriggerSweep = sweep;
		}

		/// <summary>
		/// Request a trigger level at a given voltage (as shown on the chart). The code
		/// is computed from the currently applied volts/div, the source channel's probe
		/// ratio, and the source channel's vertical-position origin, so the level tracks
		/// the same voltage the user clicked regardless of the current scale.
		/// </summary>
		public void RequestTriggerLevelVolts(double volts)
		{
			lock (m_sync)
			{
				var volts_per_div = HantekProtocol.VoltsDivTable[m_applied.VoltsDivIndex];
				var source = m_desired.TriggerSource;
				var probe = source == ETriggerSource.Ch2 ? m_desired.Ch2Probe : m_desired.Ch1Probe;
				m_desired.TriggerLevelCode = HantekProtocol.VoltsToTriggerCode(volts, volts_per_div, probe, source);
			}
		}

		/// <summary>
		/// Request the trigger horizontal position at a given time offset (seconds,
		/// relative to the trigger origin). Computed from the applied timebase.
		/// </summary>
		public void RequestTriggerTimeOffset(double seconds)
		{
			lock (m_sync)
			{
				var seconds_per_div = HantekProtocol.TimebaseTable[m_applied.TimebaseIndex];
				m_desired.TriggerHPosCode = HantekProtocol.TimeToHTriggerCode(seconds, seconds_per_div);
			}
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
		/// packets at the device's pace, applying any pending config change before
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

				// The init sequence leaves the device in the default configuration.
				// Reconciliation against 'm_desired' on the first loop re-applies any
				// settings the user changed in a prior run.
				lock (m_sync)
					m_applied = Config.Default;

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
		/// Reconcile the applied config with the latest UI request by writing the
		/// registers whose values differ. Runs on the acquisition thread between reads
		/// so the device is reprogrammed without contending with the UI thread.
		/// </summary>
		private void ApplyPendingConfig()
		{
			if (m_device == null)
				return;

			Config desired, applied;
			lock (m_sync)
			{
				desired = m_desired;
				applied = m_applied;
			}

			// 'geometry_changed' covers changes that alter the sample stream's packing
			// or time/volts mapping, so the framed display must restart from a clean
			// t=0 at the new scale. Coupling and the trigger don't affect decoding.
			var geometry_changed = false;

			// Per-channel enable. A change flips the packet packing (interleaved vs
			// contiguous), so it counts as a geometry change.
			if (desired.Ch1Enabled != applied.Ch1Enabled)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch1Enable, desired.Ch1Enabled ? 1 : 0);
				applied.Ch1Enabled = desired.Ch1Enabled;
				geometry_changed = true;
			}
			if (desired.Ch2Enabled != applied.Ch2Enabled)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch2Enable, desired.Ch2Enabled ? 1 : 0);
				applied.Ch2Enabled = desired.Ch2Enabled;
				geometry_changed = true;
			}

			// Volts/div is shared; program both channels so the two traces share a scale.
			if (desired.VoltsDivIndex != applied.VoltsDivIndex)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch1VoltsDiv, desired.VoltsDivIndex);
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch2VoltsDiv, desired.VoltsDivIndex);
				applied.VoltsDivIndex = desired.VoltsDivIndex;
				geometry_changed = true;
			}

			if (desired.TimebaseIndex != applied.TimebaseIndex)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Timebase, desired.TimebaseIndex);
				applied.TimebaseIndex = desired.TimebaseIndex;
				geometry_changed = true;
			}

			// Probe rescales the displayed volts (host-side), so a change rescales the
			// plotted amplitude; treat it as a geometry change so the Y fit refreshes.
			if (desired.Ch1Probe != applied.Ch1Probe)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch1Probe, (long)desired.Ch1Probe);
				applied.Ch1Probe = desired.Ch1Probe;
				geometry_changed = true;
			}
			if (desired.Ch2Probe != applied.Ch2Probe)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch2Probe, (long)desired.Ch2Probe);
				applied.Ch2Probe = desired.Ch2Probe;
				geometry_changed = true;
			}

			// Coupling and the trigger registers don't affect how samples decode, so
			// they're written without forcing a display reset.
			if (desired.Ch1Coupling != applied.Ch1Coupling)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch1Coupling, (long)desired.Ch1Coupling);
				applied.Ch1Coupling = desired.Ch1Coupling;
			}
			if (desired.Ch2Coupling != applied.Ch2Coupling)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.Ch2Coupling, (long)desired.Ch2Coupling);
				applied.Ch2Coupling = desired.Ch2Coupling;
			}
			if (desired.TriggerSource != applied.TriggerSource)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.TriggerSource, (long)desired.TriggerSource);
				applied.TriggerSource = desired.TriggerSource;
			}
			if (desired.TriggerSlope != applied.TriggerSlope)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.TriggerSlope, (long)desired.TriggerSlope);
				applied.TriggerSlope = desired.TriggerSlope;
			}
			if (desired.TriggerSweep != applied.TriggerSweep)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.TriggerSweep, (long)desired.TriggerSweep);
				applied.TriggerSweep = desired.TriggerSweep;
			}
			if (desired.TriggerLevelCode != applied.TriggerLevelCode)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.TriggerLevel, desired.TriggerLevelCode);
				applied.TriggerLevelCode = desired.TriggerLevelCode;
			}
			if (desired.TriggerHPosCode != applied.TriggerHPosCode)
			{
				m_device.WriteRegister(ECategory.Dso, (byte)EDsoRegister.TriggerHPos, desired.TriggerHPosCode);
				applied.TriggerHPosCode = desired.TriggerHPosCode;
			}

			// Publish the reconciled applied config back for the UI's read-side props.
			lock (m_sync)
				m_applied = applied;

			// A geometry change restarts the time origin so framed displays redraw from
			// a clean t=0 at the new scale, and refreshes the UI's resolution readout.
			if (geometry_changed)
			{
				m_sample_index = 0;
				ConfigChanged?.Invoke();
			}
		}

		/// <summary>Deinterleave a packet, convert codes to volts, and queue the samples.</summary>
		private void DecodePacket(byte[] packet)
		{
			// Snapshot the applied scale for this packet so all samples decode coherently.
			Config applied;
			lock (m_sync)
				applied = m_applied;

			var (ch1, ch2) = HantekDevice.Deinterleave(packet, applied.Ch1Enabled, applied.Ch2Enabled);
			var count = Math.Max(ch1.Count, ch2.Count);

			var dt_ms = HantekProtocol.TimebaseTable[applied.TimebaseIndex] / HantekProtocol.SamplesPerDivision * 1000.0;
			var vdiv = HantekProtocol.VoltsDivTable[applied.VoltsDivIndex];
			var probe1 = HantekProtocol.ProbeRatio(applied.Ch1Probe);
			var probe2 = HantekProtocol.ProbeRatio(applied.Ch2Probe);

			var batch = new List<Sample>(count);
			for (var i = 0; i != count; ++i)
			{
				// X advances by the running sample index so consecutive packets form
				// one contiguous stream on the acquisition-time axis. A channel that is
				// disabled (empty list) yields NaN so the render step skips it.
				var x_ms = (m_sample_index + i) * dt_ms;
				var v1 = i < ch1.Count ? HantekProtocol.CodeToDivisions(ch1[i]) * vdiv * probe1 : double.NaN;
				var v2 = i < ch2.Count ? HantekProtocol.CodeToDivisions(ch2[i]) * vdiv * probe2 : double.NaN;
				batch.Add(new Sample(x_ms, v1, v2));
			}
			m_sample_index += count;

			lock (m_sync)
			{
				m_pending.AddRange(batch);
			}
		}

		/// <summary>
		/// The full set of live-adjustable scope settings. Held as a value type so the
		/// desired and applied states can be snapshotted and compared field-by-field.
		/// </summary>
		private struct Config
		{
			public bool Ch1Enabled;
			public bool Ch2Enabled;
			public int VoltsDivIndex;
			public int TimebaseIndex;
			public ECoupling Ch1Coupling;
			public ECoupling Ch2Coupling;
			public EProbeScale Ch1Probe;
			public EProbeScale Ch2Probe;
			public ETriggerSource TriggerSource;
			public ETriggerSlope TriggerSlope;
			public ETriggerSweep TriggerSweep;
			public int TriggerLevelCode;
			public int TriggerHPosCode;

			/// <summary>The configuration the init sequence programs (see HantekDevice.InitFrames).</summary>
			public static Config Default => new()
			{
				Ch1Enabled = true,
				Ch2Enabled = true,
				VoltsDivIndex = DefaultVoltsDivIndex,
				TimebaseIndex = DefaultTimebaseIndex,
				Ch1Coupling = ECoupling.AC,
				Ch2Coupling = ECoupling.AC,
				Ch1Probe = EProbeScale.X1,
				Ch2Probe = EProbeScale.X1,
				TriggerSource = ETriggerSource.Ch1,
				TriggerSlope = ETriggerSlope.Rising,
				TriggerSweep = ETriggerSweep.Auto,
				TriggerLevelCode = HantekProtocol.Ch1VPosZeroCode,
				TriggerHPosCode = HantekProtocol.HTriggerCentreCode,
			};
		}
	}
}
