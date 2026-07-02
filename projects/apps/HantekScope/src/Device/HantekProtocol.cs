using System;
using System.Collections.Generic;

namespace HantekScope.Device
{
	/// <summary>
	/// Instrument selector carried in the category byte of every command frame.
	/// The 2D42 is a 3-in-1 device and the category picks which sub-instrument a
	/// register refers to.
	/// </summary>
	public enum ECategory :byte
	{
		Dso = 0x00,
		Dmm = 0x01,
		Dds = 0x02,
		System = 0x03,
	}

	/// <summary>Access direction in a command frame.</summary>
	public enum EAccess :byte
	{
		Write = 0x00,
		Read = 0x01,
	}

	/// <summary>
	/// DSO (category 0x00) registers. Per-channel controls are organised as blocks
	/// of stride 6: CH1 at base 0x00, CH2 at base 0x06, with the same layout in
	/// each block; globals live from 0x0c upward.
	/// </summary>
	public enum EDsoRegister :byte
	{
		Ch1Enable = 0x00,
		Ch1Coupling = 0x01,
		Ch1Probe = 0x02,
		Ch1VoltsDiv = 0x04,
		Ch1VPos = 0x05,
		Ch2Enable = 0x06,
		Ch2Coupling = 0x07,
		Ch2Probe = 0x08,
		Ch2VoltsDiv = 0x0A,
		Ch2VPos = 0x0B,
		Timebase = 0x0E,
		TriggerHPos = 0x0F,
		TriggerSource = 0x10,
		TriggerSlope = 0x11,
		TriggerSweep = 0x12,
		TriggerLevel = 0x14,
		GetWaveform = 0x16,
	}

	/// <summary>Channel input coupling (per-channel register base + 1). Confirmed by capture 10-ch1-coupling.</summary>
	public enum ECoupling :byte
	{
		AC = 0,
		DC = 1,
		GND = 2,
	}

	/// <summary>
	/// Probe attenuation as a base-10 exponent (per-channel register base + 2). The
	/// device stores the exponent; the attenuation factor is 10^exponent. It rescales
	/// only the displayed volts/div and trigger readout, never the raw ADC codes.
	/// </summary>
	public enum EProbeScale :byte
	{
		X1 = 0,
		X10 = 1,
		X100 = 2,
		X1000 = 3,
	}

	/// <summary>Trigger source channel (global register 0x10).</summary>
	public enum ETriggerSource :byte
	{
		Ch1 = 0,
		Ch2 = 1,
	}

	/// <summary>Trigger edge slope (global register 0x11).</summary>
	public enum ETriggerSlope :byte
	{
		Rising = 0,
		Falling = 1,
		Both = 2,
	}

	/// <summary>Trigger sweep mode (global register 0x12). This model has no "single" sweep.</summary>
	public enum ETriggerSweep :byte
	{
		Auto = 0,
		Normal = 1,
	}

	/// <summary>System / identity registers (category 0x03).</summary>
	public enum ESystemRegister :byte
	{
		ModeSelect = 0x00, // 0=DSO, 1=DMM, 2=DDS
		Status = 0x07,
		Serial = 0x09,
		Firmware = 0x0A,
		Version = 0x0C,
	}

	/// <summary>
	/// Protocol-level constants and frame helpers for the Hantek 2D42. Pure
	/// build/parse logic with no I/O; the device class owns the USB transfers.
	/// See the reverse-engineering notes in the 'hantek' repo (docs/protocol.md).
	/// </summary>
	public static class HantekProtocol
	{
		// USB identifiers and bulk endpoints (single vendor-specific interface).
		public const ushort VID = 0x0483;
		public const ushort PID = 0x2D42;
		public const byte EpOut = 0x02;
		public const byte EpIn = 0x81;

		// Each GET WAVEFORM returns one 64-byte bulk packet. With two channels
		// active the bytes interleave 32 CH1 + 32 CH2 samples; with one channel
		// they are 64 contiguous samples of that channel.
		public const int WaveformPacketBytes = 64;

		// The device paces acquisition by GET and preserves stream continuity, but
		// only when polled at a steady, moderate rate. The official software polls
		// at ~50 ms; reading faster returns aliased snapshots instead of a
		// contiguous stream.
		public const int WaveformPollIntervalMs = 50;

		// Sample encoding: EP 0x81 waveform bytes are 8-bit unsigned ADC codes. The
		// device digitises the trace after applying each channel's vertical position,
		// so a channel's 0 V does not sit at a fixed code — it tracks that channel's
		// vertical-position register (see AdcZeroCodeForVPos). The vertical scale is a
		// fixed number of codes per screen division.
		public const double AdcCodesPerDiv = 24.6;
		public const int ScreenVDiv = 8; // vertical divisions on the display grid

		// Per-channel 0 V ADC code as a function of the channel's vertical-position
		// register. Because the ADC samples the positioned trace, the code that means
		// 0 V shifts with the vertical position. These constants are a linear fit
		// through two measured operating points (means over 6000 samples each):
		//   CH1 vpos 149 -> 0 V code 182,  CH2 vpos 49 -> 0 V code 80.
		// giving slope ~1.02 code per vpos code and a ~30 code origin. Moving a vpos
		// register shifts the signal codes and this 0 V code together by the same
		// amount, so decode (which subtracts this 0 V code) keeps returning true volts
		// regardless of position; the fit only needs to be accurate enough that the
		// residual (from any non-linearity, worst when extrapolating far from 149/49)
		// stays small. Re-measure at more vpos points if large drags show visible drift.
		public const double AdcZeroVPosSlope = 1.02;
		public const double AdcZeroVPosOffset = 30.0;

		// Full vertical span the 8-bit ADC can represent, in screen divisions. The
		// code range is 0..255, so the signed span is 256 codes, which at
		// 'AdcCodesPerDiv' codes/div is ~10.4 divisions. Choosing volts/div so that
		// the visible Y span fills this many divisions puts the trace at the finest
		// resolution the ADC offers for the current zoom level.
		public const double AdcFullScaleDiv = 256.0 / AdcCodesPerDiv;

		// Horizontal divisions spanned by one full waveform record. The device keeps
		// a fixed record and the per-sample interval is timebase/100 (i.e. 100 samples
		// per division), so a 1200-sample record covers 1200/100 = 12 divisions.
		// Choosing the timebase so the visible time span fills this many divisions
		// puts the time axis at the finest sample resolution for the current zoom.
		public const int SamplesPerDivision = 100;
		public const int WaveformRecordSamples = 1200;
		public const double RecordDivisions = (double)WaveformRecordSamples / SamplesPerDivision;

		// GET WAVEFORM parameters: (requested points, points, divisions) =
		// (1200, 1200, 20) little-endian. Constant regardless of timebase — the
		// device decimates internally and always returns the record in 64-byte slices.
		public static readonly byte[] WaveformParams = { 0xB0, 0x04, 0xB0, 0x04, 0x14 };

		// Screen-position coordinate system. The UI position registers (vertical
		// position, horizontal trigger position, trigger level) are expressed as
		// screen-position codes at ~25 codes/division — a coordinate system distinct
		// from the raw ADC sample bytes (see docs/protocol.md §3.6/§3.7). The trigger
		// level tracks the source channel's vertical-position code as its 0 V origin,
		// and the horizontal trigger position is centred (0 divisions of delay) at 150.
		public const double ScreenCodesPerDiv = 25.0;
		public const int HTriggerCentreCode = 150;
		public const int HTriggerMaxCode = 300;

		// Per-channel vertical-position codes that the trigger level uses as its 0 V
		// origin. These match the values the init sequence programs (CH1 vpos 149,
		// CH2 vpos 49); the init trigger-level write (149) equals the CH1 vpos, so a
		// trigger at the channel's 0 V is exactly its vpos code.
		public const int Ch1VPosZeroCode = 149;
		public const int Ch2VPosZeroCode = 49;

		/// <summary>
		/// Volts-per-division for each volts/div register index (1-2-5 sequence).
		/// The register carries the index; the value is the selected V/div.
		/// Indices 2..6 (50mV..1V) are confirmed from captures; the entries marked
		/// "unconfirmed" extend the 1-2-5 pattern up and down so axis-zoom
		/// auto-resolution has headroom, and should be verified against a capture.
		/// </summary>
		public static readonly IReadOnlyDictionary<int, double> VoltsDivTable = new Dictionary<int, double>
		{
			[0] = 0.01,  // unconfirmed (extrapolated 1-2-5)
			[1] = 0.02,  // unconfirmed (extrapolated 1-2-5)
			[2] = 0.05,
			[3] = 0.1,
			[4] = 0.2,
			[5] = 0.5,
			[6] = 1.0,
			[7] = 2.0,   // unconfirmed (extrapolated 1-2-5)
			[8] = 5.0,   // unconfirmed (extrapolated 1-2-5)
			[9] = 10.0,  // unconfirmed (extrapolated 1-2-5)
		};

		/// <summary>
		/// Seconds-per-division for each timebase register index (1-2-5 sequence).
		/// Indices 7..16 (1µs..1ms) are confirmed; the entries marked "unconfirmed"
		/// extend the pattern toward slower timebases so zoom-out has headroom, and
		/// should be verified against a capture.
		/// </summary>
		public static readonly IReadOnlyDictionary<int, double> TimebaseTable = new Dictionary<int, double>
		{
			[7] = 1e-6,
			[8] = 2e-6,
			[9] = 5e-6,
			[10] = 10e-6,
			[11] = 20e-6,
			[12] = 50e-6,
			[13] = 100e-6,
			[14] = 200e-6,
			[15] = 500e-6,
			[16] = 1e-3,
			[17] = 2e-3,   // unconfirmed (extrapolated 1-2-5)
			[18] = 5e-3,   // unconfirmed (extrapolated 1-2-5)
			[19] = 10e-3,  // unconfirmed (extrapolated 1-2-5)
			[20] = 20e-3,  // unconfirmed (extrapolated 1-2-5)
			[21] = 50e-3,  // unconfirmed (extrapolated 1-2-5)
			[22] = 100e-3, // unconfirmed (extrapolated 1-2-5)
		};

		/// <summary>
		/// Pick the volts/div register index whose value is closest (in log space) to
		/// 'volts'. Log-space distance treats the 1-2-5 steps even-handedly so a target
		/// midway between two steps snaps to the nearer multiplicative neighbour.
		/// </summary>
		public static int NearestVoltsDivIndex(double volts)
		{
			return NearestIndex(VoltsDivTable, volts);
		}

		/// <summary>Pick the timebase register index whose seconds/div is closest (in log space) to 'seconds'.</summary>
		public static int NearestTimebaseIndex(double seconds)
		{
			return NearestIndex(TimebaseTable, seconds);
		}

		/// <summary>Find the table key whose value is multiplicatively closest to 'target'.</summary>
		private static int NearestIndex(IReadOnlyDictionary<int, double> table, double target)
		{
			// Guard against non-positive targets (a degenerate axis span); fall back to
			// the smallest step so the caller still gets a valid index.
			if (!(target > 0.0))
			{
				var smallest = int.MaxValue;
				var smallest_val = double.MaxValue;
				foreach (var kv in table)
				{
					if (kv.Value < smallest_val) { smallest_val = kv.Value; smallest = kv.Key; }
				}
				return smallest;
			}

			var best_index = 0;
			var best_dist = double.MaxValue;
			var log_target = Math.Log(target);
			foreach (var kv in table)
			{
				var dist = Math.Abs(Math.Log(kv.Value) - log_target);
				if (dist < best_dist)
				{
					best_dist = dist;
					best_index = kv.Key;
				}
			}
			return best_index;
		}

		/// <summary>
		/// Build a command frame for EP 0x02. Layout:
		/// seq | total_len | category | access | register | payload, where
		/// total_len is the whole frame length (5 + payload length). Register
		/// writes carry a 5-byte little-endian value; parameterless reads carry none.
		/// </summary>
		public static byte[] BuildFrame(byte seq, ECategory category, EAccess access, byte register, byte[]? payload = null)
		{
			payload ??= Array.Empty<byte>();
			var frame = new byte[5 + payload.Length];
			frame[0] = seq;
			frame[1] = (byte)(5 + payload.Length);
			frame[2] = (byte)category;
			frame[3] = (byte)access;
			frame[4] = register;
			Array.Copy(payload, 0, frame, 5, payload.Length);
			return frame;
		}

		/// <summary>Encode a register value as the 5-byte little-endian payload.</summary>
		public static byte[] EncodeValue(long value)
		{
			var payload = new byte[5];
			for (var i = 0; i != 5; ++i)
			{
				payload[i] = (byte)(value & 0xFF);
				value >>= 8;
			}
			return payload;
		}

		/// <summary>
		/// Pull the longest printable ASCII run out of a framed read response.
		/// Identity responses arrive wrapped in marker bytes whose leading/trailing
		/// framing varies, so rather than parse the framing we just extract the
		/// longest printable run, which is the serial / firmware string.
		/// </summary>
		public static string ExtractAscii(byte[] response)
		{
			var best = string.Empty;
			var current = new System.Text.StringBuilder();

			// Walk the bytes, accumulating printable runs and keeping the longest.
			foreach (var b in response)
			{
				if (b >= 32 && b < 127)
				{
					current.Append((char)b);
				}
				else
				{
					if (current.Length > best.Length)
						best = current.ToString();
					current.Clear();
				}
			}
			if (current.Length > best.Length)
				best = current.ToString();

			return best;
		}

		/// <summary>The 0 V ADC code for a channel programmed to the given vertical-position register.</summary>
		public static double AdcZeroCodeForVPos(int vpos_code)
		{
			return AdcZeroVPosSlope * vpos_code + AdcZeroVPosOffset;
		}

		/// <summary>Convert a raw 8-bit ADC sample to a signed division offset from the channel's 0 V code.</summary>
		public static double CodeToDivisions(byte code, double zero_code)
		{
			return (code - zero_code) / AdcCodesPerDiv;
		}

		/// <summary>The linear attenuation factor for a probe scale exponent (10^exponent).</summary>
		public static double ProbeRatio(EProbeScale probe)
		{
			return Math.Pow(10.0, (int)probe);
		}

		/// <summary>The factory (home) vertical-position register code a channel starts at.</summary>
		public static int VPosHomeCode(int channel)
		{
			switch (channel)
			{
				case 1: return Ch1VPosZeroCode;
				case 2: return Ch2VPosZeroCode;
				default: throw new ArgumentOutOfRangeException(nameof(channel));
			}
		}

		/// <summary>
		/// Convert a signed threshold voltage (in the channel's true signal volts, i.e.
		/// already probe-scaled) to the trigger-level register code. The trigger level
		/// sits on the screen-position scale (~25 codes/div) with the source channel's
		/// current vertical-position code as its 0 V origin, so the caller passes that
		/// channel's live vpos code as 'zero_code' (it moves as the channel is repositioned).
		/// The probe ratio is divided back out because it only affects the displayed volts,
		/// not the underlying screen scale. The result is clamped to the 8-bit register range.
		/// </summary>
		public static int VoltsToTriggerCode(double volts, double volts_per_div, EProbeScale probe, int zero_code)
		{
			var raw_volts = volts / ProbeRatio(probe);
			var code = zero_code + raw_volts / volts_per_div * ScreenCodesPerDiv;
			return (int)Math.Round(Math.Clamp(code, 0.0, 255.0));
		}

		/// <summary>
		/// Convert a channel's vertical-position offset (the volts by which its baseline is
		/// moved up the chart, in true signal volts) to the vertical-position register code.
		/// The register is on the screen-position scale (~25 codes/div) with the channel's
		/// home 0 V code as origin; the probe ratio is divided back out because it only
		/// affects displayed volts, not the underlying screen scale. Clamped to the 8-bit
		/// register range. Note decode reads back the moved 0 V code via AdcZeroCodeForVPos,
		/// so the signal and its zero shift together and the decoded volts stay true.
		/// </summary>
		public static int VoltsToVPosCode(double offset_volts, double volts_per_div, EProbeScale probe, int home_code)
		{
			var raw_volts = offset_volts / ProbeRatio(probe);
			var code = home_code + raw_volts / volts_per_div * ScreenCodesPerDiv;
			return (int)Math.Round(Math.Clamp(code, 0.0, 255.0));
		}

		/// <summary>
		/// Convert a time offset (seconds, relative to the trigger origin) to the
		/// horizontal trigger-position register code. The position is centred at 150
		/// (zero delay) on the screen-position scale (~25 codes/div) and clamped to the
		/// device's 0..300 span.
		/// </summary>
		public static int TimeToHTriggerCode(double seconds, double seconds_per_div)
		{
			if (!(seconds_per_div > 0.0))
				return HTriggerCentreCode;

			var code = HTriggerCentreCode + seconds / seconds_per_div * ScreenCodesPerDiv;
			return (int)Math.Round(Math.Clamp(code, 0.0, HTriggerMaxCode));
		}
	}
}
