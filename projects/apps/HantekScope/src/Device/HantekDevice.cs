using System;
using System.Collections.Generic;
using System.Threading;

namespace HantekScope.Device
{
	/// <summary>
	/// Driver for the Hantek 2D42 over WinUSB. Wraps the raw bulk endpoints in a
	/// typed API: open/close, register read/write, identity, the official
	/// initialisation sequence, and single waveform-packet reads. Acquisition
	/// pacing is the caller's responsibility (see HantekProtocol.WaveformPollIntervalMs).
	/// </summary>
	public sealed class HantekDevice :IDisposable
	{
		// GUID_DEVINTERFACE_USB_DEVICE - the standard USB device interface class that
		// the USB stack publishes for every device, regardless of function driver. Zadig
		// also assigns a random per-install WinUSB interface GUID, but that varies from
		// machine to machine; enumerating on this generic class and matching VID/PID in
		// the device path (see WinUsbDevice.FindDevicePath) discovers the scope on any PC
		// without a machine-specific constant to keep in sync.
		private static readonly Guid InterfaceGuid = new("A5DCBF10-6530-11D2-901F-00C04FB951ED");

		// The official software's startup sequence, captured verbatim. Replaying it
		// puts the device into a known-good acquisition state and reads the identity /
		// capability registers. The leading seq byte is rewritten at send time, and
		// read frames (access byte == 0x01) have their IN response drained before the
		// next frame or the bulk endpoints desync. Defaults baked in: both channels
		// enabled at 1 V/div, trigger CH1 rising auto, timebase index 14 (200 us/div),
		// generator a 1 kHz / 2 Vpp sine. Vertical position is deliberately NOT written:
		// the app adopts whatever vpos the scope already holds (see ScopeModel), so the
		// two per-channel vpos frames are omitted from this sequence.
		private static readonly string[] InitFrames =
		{
			"000a0301070000000000", // read cat03 reg07 status
			"000a0001150000000000", // read cat00 reg15 capability struct
			"000a03010a0000000000", // read cat03 reg0a firmware
			"000a03010c0000000000", // read cat03 reg0c version
			"00250301090000000000", // read cat03 reg09 serial (large response buffer)
			"000a0000000100000000", // CH1 enable = 1
			"000a0000010000000000", // CH1 coupling = 0 (AC)
			"000a0000020000000000", // CH1 probe = 0 (x1)
			"000a0000030000000000", // CH1 reg+3 = 0
			"000a0000040600000000", // CH1 volts/div = 6 (1 V)
			"000a0000060100000000", // CH2 enable = 1
			"000a0000070000000000", // CH2 coupling = 0 (AC)
			"000a0000080000000000", // CH2 probe = 0 (x1)
			"000a0000090000000000", // CH2 reg+3 = 0
			"000a00000a0600000000", // CH2 volts/div = 6 (1 V)
			"000a0000146400000000", // trigger level = 100 (display centre)
			"000a00000f9600000000", // horizontal trigger position = 150
			"000a0000100000000000", // trigger source = 0 (CH1)
			"000a0000120000000000", // trigger sweep = 0 (auto)
			"000a0000110000000000", // trigger slope = 0 (rising)
			"000a00000e0e00000000", // timebase = 14 (200 us/div)
			"000a0300000000000000", // mode select = DSO (0)
			"000a0300000000000000", // mode select = DSO (0) again
			"000a0200000200000000", // DDS waveform = sine (2)
			"000a020001e803000000", // DDS frequency = 1000 Hz
			"000a020002d007000000", // DDS amplitude = 2000 mVpp
			"000a0200030000000000", // DDS offset = 0
			"000a0200080100000000", // DDS output enable = 1
		};

		private readonly WinUsbDevice m_usb = new();
		private byte m_seq;

		/// <summary>True once the underlying WinUSB device is open.</summary>
		public bool IsOpen => m_usb.IsOpen;

		/// <summary>Find the scope, open it, and bound transfer timeouts on both pipes.</summary>
		public void Open()
		{
			var pipes = new[] { HantekProtocol.EpIn, HantekProtocol.EpOut };
			m_usb.Open(InterfaceGuid, HantekProtocol.VID, HantekProtocol.PID, pipes, 1000);

			// Prime the IN endpoint before issuing the first command. After an unclean prior
			// close the firmware can be part-way through an exchange, waiting for the host to
			// read an IN transaction; until that read is attempted it NAKs every OUT write, so
			// the first command times out. A single read completes that pending transaction
			// (and drains any stale response bytes), after which OUT writes are serviced again.
			m_usb.FlushInput(HantekProtocol.EpIn);
		}

		/// <summary>Release the device.</summary>
		public void Dispose()
		{
			m_usb.Dispose();
		}

		/// <summary>Rolling 8-bit command sequence counter.</summary>
		private byte NextSeq()
		{
			var seq = m_seq;
			m_seq = (byte)(m_seq + 1);
			return seq;
		}

		/// <summary>Send a pre-built command frame on EP 0x02 with a fresh seq byte.</summary>
		private void WriteFrame(byte[] frame)
		{
			frame[0] = NextSeq();
			m_usb.Write(HantekProtocol.EpOut, frame);
		}

		/// <summary>Read one bulk IN transfer; an empty array means the read timed out.</summary>
		private byte[] ReadIn(int length = HantekProtocol.WaveformPacketBytes)
		{
			return m_usb.Read(HantekProtocol.EpIn, length);
		}

		/// <summary>Write a register value (5-byte little-endian payload).</summary>
		public void WriteRegister(ECategory category, byte register, long value)
		{
			var frame = HantekProtocol.BuildFrame(0, category, EAccess.Write, register, HantekProtocol.EncodeValue(value));
			WriteFrame(frame);
		}

		/// <summary>Issue a parameterless register read and return the raw response.</summary>
		public byte[] ReadRegister(ECategory category, byte register, int length = 64)
		{
			var frame = HantekProtocol.BuildFrame(0, category, EAccess.Read, register);
			WriteFrame(frame);
			return ReadIn(length);
		}

		/// <summary>
		/// Read a single DSO register's value byte, returning -1 if a valid response can't
		/// be obtained. A register-read response is 5 bytes: 0x55, length, category,
		/// register, value. The device echoes the category and register it is answering, so
		/// the response is validated against the request; if it doesn't match (e.g. a stale
		/// response is still queued in the IN pipe after an unclean prior session) the read
		/// is retried, which drains the stale frame and realigns the request/response pairing.
		/// </summary>
		public int ReadDsoRegisterValue(EDsoRegister register, int attempts = 4)
		{
			for (var attempt = 0; attempt != attempts; ++attempt)
			{
				var resp = ReadRegister(ECategory.Dso, (byte)register, 64);
				if (resp.Length >= 5 && resp[0] == 0x55 && resp[2] == (byte)ECategory.Dso && resp[3] == (byte)register)
					return resp[4];
			}
			return -1;
		}

		/// <summary>Read serial / firmware / version identity strings.</summary>
		public (string Serial, string Firmware, string Version) ReadIdentity()
		{
			var serial = HantekProtocol.ExtractAscii(ReadRegister(ECategory.System, (byte)ESystemRegister.Serial));
			var firmware = HantekProtocol.ExtractAscii(ReadRegister(ECategory.System, (byte)ESystemRegister.Firmware));
			var version = HantekProtocol.ExtractAscii(ReadRegister(ECategory.System, (byte)ESystemRegister.Version));
			return (serial, firmware, version);
		}

		/// <summary>
		/// Replay the official startup sequence to reach a valid acquisition state.
		/// Read frames have their IN response drained before continuing.
		/// </summary>
		public void RunInitSequence()
		{
			foreach (var hx in InitFrames)
			{
				var frame = HexToBytes(hx);
				WriteFrame(frame);

				// Frame layout byte 3 is the access field; drain reads so the endpoints stay aligned.
				if (frame[3] == (byte)EAccess.Read)
					ReadIn(64);

				Thread.Sleep(5);
			}

			// Let the first acquisition settle before the caller starts polling.
			Thread.Sleep(100);
		}

		/// <summary>
		/// Issue one GET WAVEFORM and return its raw sample packet (typically 64 bytes).
		/// The device paces acquisition by GET, so the caller must poll at a steady
		/// rate (see HantekProtocol.WaveformPollIntervalMs); this performs exactly one
		/// request/response round-trip.
		/// </summary>
		public byte[] ReadWaveformPacket()
		{
			var frame = HantekProtocol.BuildFrame(0, ECategory.Dso, EAccess.Read, (byte)EDsoRegister.GetWaveform, HantekProtocol.WaveformParams);
			WriteFrame(frame);
			return ReadIn(HantekProtocol.WaveformPacketBytes);
		}

		/// <summary>
		/// Split a raw waveform packet into per-channel sample lists according to which
		/// channels are enabled. With both channels active the bytes interleave
		/// CH1,CH2,CH1,CH2…; with a single channel active the whole packet is contiguous
		/// samples of that one channel (so a CH2-only stream fills the CH2 list, leaving
		/// CH1 empty). With neither enabled both lists are empty.
		/// </summary>
		public static (List<byte> Ch1, List<byte> Ch2) Deinterleave(byte[] packet, bool ch1_on, bool ch2_on)
		{
			var ch1 = new List<byte>();
			var ch2 = new List<byte>();

			if (ch1_on && ch2_on)
			{
				for (var i = 0; i < packet.Length; ++i)
				{
					if ((i & 1) == 0)
						ch1.Add(packet[i]);
					else
						ch2.Add(packet[i]);
				}
			}
			else if (ch1_on)
			{
				ch1.AddRange(packet);
			}
			else if (ch2_on)
			{
				ch2.AddRange(packet);
			}

			return (ch1, ch2);
		}

		/// <summary>Parse a hex string of an init frame into bytes.</summary>
		private static byte[] HexToBytes(string hex)
		{
			var bytes = new byte[hex.Length / 2];
			for (var i = 0; i != bytes.Length; ++i)
				bytes[i] = Convert.ToByte(hex.Substring(i * 2, 2), 16);
			return bytes;
		}
	}
}
