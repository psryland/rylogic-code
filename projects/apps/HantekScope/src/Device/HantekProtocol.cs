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

		// Sample encoding: EP 0x81 waveform bytes are 8-bit unsigned ADC codes.
		// Zero volts sits near the middle of the range; the vertical scale is a
		// fixed number of codes per screen division.
		public const double AdcZeroCode = 128.0;
		public const double AdcCodesPerDiv = 24.6;
		public const int ScreenVDiv = 8; // vertical divisions on the display grid

		// GET WAVEFORM parameters: (requested points, points, divisions) =
		// (1200, 1200, 20) little-endian. Constant regardless of timebase — the
		// device decimates internally and always returns the record in 64-byte slices.
		public static readonly byte[] WaveformParams = { 0xB0, 0x04, 0xB0, 0x04, 0x14 };

		/// <summary>
		/// Volts-per-division for each volts/div register index (1-2-5 sequence).
		/// The register carries the index; the value is the selected V/div.
		/// </summary>
		public static readonly IReadOnlyDictionary<int, double> VoltsDivTable = new Dictionary<int, double>
		{
			[2] = 0.05,
			[3] = 0.1,
			[4] = 0.2,
			[5] = 0.5,
			[6] = 1.0,
		};

		/// <summary>
		/// Seconds-per-division for each timebase register index (1-2-5 sequence).
		/// Only the commonly-used entries are listed; the device accepts the full range.
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
		};

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

		/// <summary>Convert a raw 8-bit ADC sample to a signed division offset from zero.</summary>
		public static double CodeToDivisions(byte code)
		{
			return (code - AdcZeroCode) / AdcCodesPerDiv;
		}
	}
}
