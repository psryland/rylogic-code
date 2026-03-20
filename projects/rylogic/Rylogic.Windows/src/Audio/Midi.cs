// Virtual MIDI port
// by "Tobias Erichsen"

namespace Rylogic.Audio;

using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;

public static class Midi
{
	// MIDI Note Numbers for Different Octaves
	//  Middle 'C' = C3, (sometimes C4, but C3 is more common)
	// ================================================================================ 
	// Octave  |  Note Numbers
	// --------+-----------------------------------------------------------------------
	//         |   C    C#     D    D#     E     F    F#     G    G#     A    A#     B
	// --------+-----------------------------------------------------------------------
	//    -2   |   0     1     2     3     4     5     6     7     8     9    10    11
	//    -1   |  12    13    14    15    16    17    18    19    20    21    22    23
	//     0   |  24    25    26    27    28    29    30    31    32    33    34    35
	//     1   |  36    37    38    39    40    41    42    43    44    45    46    47
	//     2   |  48    49    50    51    52    53    54    55    56    57    58    59
	//     3   |  60    61    62    63    64    65    66    67    68    69    70    71
	//     4   |  72    73    74    75    76    77    78    79    80    81    82    83
	//     5   |  84    85    86    87    88    89    90    91    92    93    94    95
	//     6   |  96    97    98    99   100   101   102   103   104   105   106   107
	//     7   | 108   109   110   111   112   113   114   115   116   117   118   119
	//     8   | 120   121   122   123   124   125   126   127
		
	/// <summary>Note names</summary>
	public static readonly string[] Names = new [] { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

	/// <summary>Convert a note name to it's MIDI note id</summary>
	public static byte NoteId(string note)
	{
		// Expected forms: C2, C#-2, A#10, etc
		var name = char.ToUpperInvariant(note[0]);
		var sharp = note[1] == '#' ? 1 : 0;
		var octave = Math_.Clamp(int.Parse(note.Substring(1 + sharp)) + 2, 0, 8);
		switch (name)
		{
		default: throw new Exception($"Invalid MIDI note name {name}");
		case 'C': return (byte)( 0 + sharp + octave * 12);
		case 'D': return (byte)( 2 + sharp + octave * 12);
		case 'E': return (byte)( 4         + octave * 12);
		case 'F': return (byte)( 5 + sharp + octave * 12);
		case 'G': return (byte)( 7 + sharp + octave * 12);
		case 'A': return (byte)( 9 + sharp + octave * 12);
		case 'B': return (byte)(11 + sharp + octave * 12);
		}
	}

	/// <summary>Convert a midi note id to it's note name</summary>
	public static string NoteName(byte id)
	{
		var num = id & 0x7F;
		var note = num % 12;
		var octave = (num / 12) - 2;
		switch (note)
		{
		default: throw new Exception($"Invalid note {note}");
		case 0:  return $"C{octave}";
		case 1:  return $"C#{octave}";
		case 2:  return $"D{octave}";
		case 3:  return $"D#{octave}";
		case 4:  return $"E{octave}";
		case 5:  return $"F{octave}";
		case 6:  return $"F#{octave}";
		case 7:  return $"G{octave}";
		case 8:  return $"G#{octave}";
		case 9:  return $"A{octave}";
		case 10: return $"A#{octave}";
		case 11: return $"B{octave}";
		}
	}

	/// <summary>Event types</summary>
	public enum EEvent :byte
	{
		// See: https://www.midi.org/specifications/item/table-1-summary-of-midi-message

		/// <summary>
		/// 1000nnnn 0kkkkkkk 0vvvvvvv = Note Off event.
		/// This message is sent when a note is released (ended). (kkkkkkk) is the key (note) number. (vvvvvvv) is the velocity.</summary>
		NoteOff = 0x80,

		/// <summary>
		/// 1001nnnn 0kkkkkkk 0vvvvvvv = Note On event.
		/// This message is sent when a note is depressed (start). (kkkkkkk) is the key (note) number. (vvvvvvv) is the velocity.</summary>
		NoteOn = 0x90,

		/// <summary>
		/// 1010nnnn 0kkkkkkk 0vvvvvvv = Polyphonic Key Pressure (Aftertouch).
		/// This message is most often sent by pressing down on the key after it "bottoms out".
		/// (kkkkkkk) is the key (note) number. (vvvvvvv) is the pressure value.</summary>
		PolyphonicKeyPressure = 0xA0,

		/// <summary>
		/// 1011nnnn 0ccccccc 0vvvvvvv = Control Change.
		/// This message is sent when a controller value changes. Controllers include devices such as pedals and levers.
		/// Controller numbers 120-127 are reserved as "Channel Mode Messages" (below). (ccccccc) is the controller number (0-119).
		/// (vvvvvvv) is the controller value (0-127).</summary>
		ControlChange = 0xB0,

		/// <summary>
		/// 1100nnnn 0ppppppp = Program Change.
		/// This message sent when the patch number changes. (ppppppp) is the new program number.</summary>
		ProgramChange = 0xC0,

		/// <summary>
		/// 1101nnnn 0vvvvvvv = Channel Pressure (After-touch).
		/// This message is most often sent by pressing down on the key after it "bottoms out".
		/// This message is different from polyphonic after-touch. Use this message to send the single greatest
		/// pressure value (of all the current depressed keys). (vvvvvvv) is the pressure value.</summary>
		ChannelPressure = 0xD0,

		/// <summary>
		/// 1110nnnn 0lllllll 0mmmmmmm = Pitch Bend Change.
		/// This message is sent to indicate a change in the pitch bender (wheel or lever, typically).
		/// The pitch bender is measured by a fourteen bit value. Centre (no pitch change) is 2000H.
		/// Sensitivity is a function of the receiver, but may be set using RPN 0. (lllllll) are the least significant 7 bits.
		/// (mmmmmmm) are the most significant 7 bits.</summary>
		PitchBend = 0xE0,

		/// <summary>
		/// 11110000 = System Common Message</summary>
		System = 0xF0,
	}

	/// <summary>Control mode messages</summary>
	public enum EControlModeMessage :byte
	{
		// For reserved controller numbers 120-127 in 'ControlChange' events

		/// <summary>
		/// All Sound Off.
		/// When All Sound Off is received all oscillators will turn off, and their volume envelopes
		/// are set to zero as soon as possible. c = 120, v = 0</summary>
		AllSoundOff = 120,

		/// <summary>
		/// Reset All Controllers.
		/// When Reset All Controllers is received, all controller values are reset to their default values.
		/// (See specific Recommended Practices for defaults). c = 121, v = x. 'x' Value must be zero unless
		/// otherwise allowed in a specific Recommended Practice.</summary>
		ResetAllControllers = 121,

		/// <summary>
		/// Local Control.
		/// When Local Control is Off, all devices on a given channel will respond only to data received over MIDI.
		/// Played data, etc. will be ignored. Local Control On restores the functions of the normal controllers.
		/// c = 122, v = 0: Local Control Off, c = 122, v = 127: Local Control On</summary>
		LocalControl = 122,

		/// <summary>
		/// All Notes Off.
		/// When an All Notes Off is received, all oscillators will turn off. c = 123, v = 0:
		/// All Notes Off (See text for description of actual mode commands.)</summary>
		AllNotesOff = 123,

		/// <summary>c = 124, v = 0: Omni Mode Off</summary>
		OmniModeOff = 124,

		/// <summary>c = 125, v = 0: Omni Mode On</summary>
		OmniModeOn = 125,

		/// <summary>c = 126, v = M: Mono Mode On (Poly Off) where M is the number of channels (Omni Off) or 0 (Omni On)</summary>
		MonoModeOn = 126,

		/// <summary>c = 127, v = 0: Poly Mode On (Mono Off) (Note: These four messages also cause All Notes Off)</summary>
		PolyModeOn = 127,
	}

	/// <summary>System common and real time message types</summary>
	public enum ESystemMessage :byte
	{
		#region Common messages

		/// <summary>
		/// 11110000 0iiiiiii [0iiiiiii 0iiiiiii] 0ddddddd --- --- 0ddddddd 11110111 = System Exclusive. a.k.a "SysEx"
		/// This message type allows manufacturers to create their own messages (such as bulk dumps, patch parameters,
		/// and other non-spec data) and provides a mechanism for creating additional MIDI Specification messages.
		/// The Manufacturer's ID code (assigned by MMA or AMEI) is either 1 byte (0iiiiiii) or 3 bytes (0iiiiiii 0iiiiiii 0iiiiiii).
		/// Two of the 1 Byte IDs are reserved for extensions called Universal Exclusive Messages, which are not manufacturer-specific.
		/// If a device recognizes the ID code as its own (or as a supported Universal message) it will listen to the rest of the message (0ddddddd).
		/// Otherwise, the message will be ignored. (Note: Only Real-Time messages may be interleaved with a System Exclusive.)</summary>
		Custom = 0x00,

		/// <summary>
		/// 11110001 0nnndddd = MIDI Time Code Quarter Frame.
		/// nnn = Message Type, dddd = Values</summary>
		MIDITimeCodeQuarterFrame = 0x01,

		/// <summary>
		/// 11110010 0lllllll 0mmmmmmm = Song Position Pointer. 
		/// This is an internal 14 bit register that holds the number of MIDI beats (1 beat = six MIDI clocks)
		/// since the start of the song. l is the LSB, m the MSB.</summary>
		SongPositionPointer = 0x02,

		/// <summary>
		/// 11110011 0sssssss = Song Select.
		/// The Song Select specifies which sequence or song is to be played.</summary>
		SongSelected = 0x03,

		/// <summary>Reserved</summary>
		Reserved0 = 0x04,
			
		/// <summary>Reserved</summary>
		Reserved1 = 0x05,

		/// <summary>
		/// 11110110 = Tune Request.
		/// Upon receiving a Tune Request, all analogue synthesizers should tune their oscillators.</summary>
		TuneRequest = 0x06,

		/// <summary>
		/// 11110111 = End of Exclusive.
		/// Used to terminate a System Exclusive dump (see above).</summary>
		EndOfExclusive = 0x07,

		#endregion

		#region Real Time Messages

		/// <summary>
		/// 11111000 = Timing Clock.
		/// Sent 24 times per quarter note when synchronization is required (see text).</summary>
		TimingClock = 0x08,

		/// <summary>Reserved</summary>
		Reserved2 = 0x09,

		/// <summary>
		/// 11111010 = Start.
		/// Start the current sequence playing. (This message will be followed with Timing Clocks).</summary>
		Start = 0x0A,

		/// <summary>
		/// 11111011 = Continue.
		/// Continue at the point the sequence was Stopped.</summary>
		Continue = 0x0B,

		/// <summary>
		/// 11111100 = Stop.
		/// Stop the current sequence.</summary>
		Stop = 0x0C,

		/// <summary>Reserved</summary>
		Reserved3 = 0x0D,

		/// <summary>
		/// 11111110 = Active Sensing.
		/// This message is intended to be sent repeatedly to tell the receiver that a connection is alive.
		/// Use of this message is optional. When initially received, the receiver will expect to receive another
		/// Active Sensing message each 300ms (max), and if it does not then it will assume that the connection
		/// has been terminated. At termination, the receiver will turn off all voices and return to normal
		/// (non-active sensing) operation.</summary>
		ActiveSensing = 0x0E,

		/// <summary>
		/// 11111111 = Reset.
		/// Reset all receivers in the system to power-up status.
		/// This should be used sparingly, preferably under manual control.
		/// In particular, it should not be sent on power-up.</summary>
		Reset = 0x0F,

		#endregion
	}

	/// <summary>MIDI command</summary>
	[DebuggerDisplay("{Description}")]
	public struct Command
	{
		private byte[] m_command;
		public Command(byte[] command)
		{
			if (command.Length < 2)
				throw new Exception("Invalid MIDI command");

			m_command = command;
		}

		/// <summary>The MIDI event</summary>
		public EEvent Event
		{
			get { return (EEvent)(m_command[0] & 0xF0); }
			set { m_command[0] = Bit.SetBits(m_command[0], (byte)0xF0, (byte)value); }
		}

		/// <summary>The channel that the message is for [0,15]</summary>
		public byte Channel
		{
			get
			{
				Debug.Assert(Event != EEvent.System, "Not valid for this event type");
				return (byte)(m_command[0] & 0x0F);
			}
			set
			{
				Debug.Assert(Event != EEvent.System, "Not valid for this event type");
				m_command[0] = Bit.SetBits(m_command[0], (byte)0x0F, value);
			}
		}

		/// <summary>The note that the event is for</summary>
		public byte NoteId
		{
			get
			{
				Debug.Assert(Event.Within(EEvent.NoteOff, EEvent.NoteOn ,EEvent.PolyphonicKeyPressure), "Not valid for this event type");
				return (byte)(m_command[1] & 0x7F);
			}
			set
			{
				Debug.Assert(Event.Within(EEvent.NoteOff, EEvent.NoteOn ,EEvent.PolyphonicKeyPressure), "Not valid for this event type");
				m_command[1] = Bit.SetBits(m_command[1], (byte)0x7F, value);
			}
		}

		/// <summary>The velocity of the node [0,0x7F]</summary>
		public byte Velocity
		{
			get
			{
				Debug.Assert(Event.Within(EEvent.NoteOff, EEvent.NoteOn ,EEvent.PolyphonicKeyPressure), "Not valid for this event type");
				return (byte)(m_command[2] & 0x7F);
			}
			set
			{
				Debug.Assert(Event.Within(EEvent.NoteOff, EEvent.NoteOn ,EEvent.PolyphonicKeyPressure), "Not valid for this event type");
				m_command[2] = Bit.SetBits(m_command[2], (byte)0x7f, value);
			}
		}

		/// <summary>Velocity normalised to [0,1]</summary>
		public float VelocityN
		{
			get { return (float)Velocity / 0x7F; }
			set { Velocity = (byte)Math_.Clamp(value * 0x7F, 0, 0x7F); }
		}

		/// <summary>The controller that the event is for (e.g. pedal, levers, etc) [0,119]. For [120,127] use 'EControlModeMessage'</summary>
		public byte ControllerId
		{
			get
			{
				Debug.Assert(Event == EEvent.ControlChange, "Not valid for this event type");
				return (byte)(m_command[1] & 0x7F);
			}
			set
			{
				Debug.Assert(Event == EEvent.ControlChange, "Not valid for this event type");
				m_command[1] = Bit.SetBits(m_command[1], (byte)0x7F, value);
			}
		}

		/// <summary>The value of the controller [0,0x7F]</summary>
		public byte ControllerValue
		{
			get
			{
				Debug.Assert(Event == EEvent.ControlChange, "Not valid for this event type");
				return (byte)(m_command[2] & 0x7F);
			}
			set
			{
				Debug.Assert(Event == EEvent.ControlChange, "Not valid for this event type");
				m_command[2] = Bit.SetBits(m_command[2], (byte)0x7f, value);
			}
		}

		/// <summary>The patch number associated with a ProgramChange event [0,0x7F]</summary>
		public byte ProgramId
		{
			get
			{
				Debug.Assert(Event == EEvent.ProgramChange, "Not valid for this event type");
				return (byte)(m_command[1] & 0x7F);
			}
			set
			{
				Debug.Assert(Event == EEvent.ProgramChange, "Not valid for this event type");
				m_command[1] = Bit.SetBits(m_command[1], (byte)0x7F, value);
			}
		}

		/// <summary>The channel pressure [0,0x7F]</summary>
		public byte ChannelPressure
		{
			get
			{
				Debug.Assert(Event.Within(EEvent.NoteOff, EEvent.NoteOn ,EEvent.PolyphonicKeyPressure), "Not valid for this event type");
				return (byte)(m_command[2] & 0x7F);
			}
			set
			{
				Debug.Assert(Event.Within(EEvent.NoteOff, EEvent.NoteOn ,EEvent.PolyphonicKeyPressure), "Not valid for this event type");
				m_command[2] = Bit.SetBits(m_command[2], (byte)0x7f, value);
			}
		}

		/// <summary>Pitch bend value</summary>
		public short PitchBend
		{
			get
			{
				Debug.Assert(Event == EEvent.PitchBend, "Not valid for this event type");
				var value =
					((m_command[1] & 0x7F)     ) |
					((m_command[2] & 0x7F) << 7);
				return (short)(value - 0x2000);
			}
			set
			{
				Debug.Assert(Event == EEvent.PitchBend, "Not valid for this event type");
				var val = value + 0x2000;
				m_command[1] = Bit.SetBits(m_command[1], (byte)0x7F, (byte)((val     ) & 0x7F));
				m_command[2] = Bit.SetBits(m_command[2], (byte)0x7F, (byte)((val >> 7) & 0x7F));
			}
		}

		/// <summary>Pitch bend value normalised to [-1, +1]</summary>
		public double PitchBendN
		{
			get { return Math_.Frac(-0x2000, (double)PitchBend, 0x1fff) * 2.0 - 1.0; }
			set { PitchBend = (short)Math_.Lerp(-0x2000, 0x1fff, Math_.Clamp(0.5*value + 1.0, 0.0, 1.0)); }
		}

		/// <summary>True if this is a control mode message</summary>
		public bool IsControlModeMessage
		{
			get { return Event == EEvent.ControlChange && ControllerId >= 120 && ControllerId <= 127; }
		}

		/// <summary>The control mode message for 'EEvent.ControlChange' events when ControllerId is [120,127]</summary>
		public EControlModeMessage ControlModeMessage
		{
			get
			{
				Debug.Assert(IsControlModeMessage, "Not valid for this event type");
				return (EControlModeMessage)ControllerId;
			}
			set
			{
				Debug.Assert(IsControlModeMessage, "Not valid for this event type");
				ControllerId = (byte)value;
			}
		}

		/// <summary>The control mode value for 'EEvent.ControlChange' events when ControllerId is [120,127]</summary>
		public byte ControlModeValue
		{
			get
			{
				Debug.Assert(IsControlModeMessage, "Not valid for this event type");
				return (byte)(m_command[2] & 0x7F);
			}
			set
			{
				Debug.Assert(IsControlModeMessage, "Not valid for this event type");
				m_command[2] = Bit.SetBits(m_command[2], (byte)0x7F, value);
			}
		}

		/// <summary></summary>
		public string Description
		{
			get
			{
				switch (Event)
				{
				default: return $"Unknown MIDI command: {m_command[0]:X2}-{m_command[1]:X2}-{m_command[2]:X2}";
				case EEvent.NoteOff:
				case EEvent.NoteOn:
				case EEvent.PolyphonicKeyPressure:
					return $"{Event}  Ch:{Channel+1}  Note:{NoteName(NoteId)}({NoteId})  Vel:{VelocityN}";
				case EEvent.ControlChange:
					return IsControlModeMessage
						? $"{Event}  Ch:{Channel+1}  Msg:{ControlModeMessage}  Value:{ControlModeValue}"
						: $"{Event}  Ch:{Channel+1}  Id:{ControllerId}  Value:{ControllerValue}";
				case EEvent.ProgramChange:
					return $"{Event}  Ch:{Channel+1}  Id:{ProgramId}";
				case EEvent.ChannelPressure:
					return $"{Event}  Ch:{Channel+1}  Value:{ChannelPressure}";
				case EEvent.PitchBend:
					return $"{Event}  Ch:{Channel+1}  Bend:{PitchBendN}";
				case EEvent.System:
					return $"{Event} ";
				}
			}
		}

		/// <summary>Access the raw midi message bytes</summary>
		public byte[] Bytes
		{
			get { return m_command; }
		}

		/// <summary></summary>
		public override string ToString()
		{
			return Description;
		}
	}
}

public class VirtualMidi :IDisposable
{
	/// <summary>Default size of SysEx buffer</summary>
	private const uint DefaultSysExSize = 65536;
	private const uint MaxReadProcessIds = 17;

	/// <summary>Handle to the virtual MIDI port instance</summary>
	private IntPtr m_handle;

	[Flags] public enum EFlags :uint
	{
		/// <summary>Parse incoming data into single, valid MIDI-commands</summary>
		ParseRX = 1,

		/// <summary>Parse outgoing data into single, valid MIDI-commands</summary>
		ParseTX = 2,

		/// <summary>Only the "midi-out" part of the port is created</summary>
		InstantiateRXOnly = 4,

		/// <summary>Only the "midi-in" part of the port is created</summary>
		InstantiateTXOnly = 8,

		/// <summary>A bidirectional port is created</summary>
		InstantiateBoth = InstantiateTXOnly | InstantiateRXOnly,
	}

	/// <summary>Create a virtual MIDI port</summary>
	public VirtualMidi(string port_name, uint max_sysex_length = DefaultSysExSize, EFlags flags = EFlags.ParseRX)
		:this(port_name, null, IntPtr.Zero, max_sysex_length, flags)
	{}
	public VirtualMidi(string port_name, uint max_sysex_length, EFlags flags, Guid manufacturer, Guid product)
		: this(port_name, null, IntPtr.Zero, max_sysex_length, flags, manufacturer, product)
	{ }
	public VirtualMidi(string port_name, Callback? cb, IntPtr ctx, uint max_sysex_length = DefaultSysExSize, EFlags flags = EFlags.ParseRX)
	{
		var handle = VirtualMIDICreatePortEx2(port_name, cb, ctx, max_sysex_length, (uint)flags);
		Check(handle != IntPtr.Zero);
		m_handle = handle;
		m_read_buffer = new byte[max_sysex_length];
		m_read_process_ids = new UInt64[17];
	}
	public VirtualMidi(string port_name, Callback? cb, IntPtr ctx, uint max_sysex_length, EFlags flags, Guid manufacturer, Guid product)
	{
		var handle = VirtualMIDICreatePortEx3(port_name, cb, ctx, max_sysex_length, (uint)flags, ref manufacturer, ref product);
		Check(handle != IntPtr.Zero);
		m_handle = handle;
		m_read_buffer = new byte[max_sysex_length];
		m_read_process_ids = new UInt64[17];
	}
	public void Dispose()
	{
		Dispose(true);
		GC.SuppressFinalize(this);
	}
	protected virtual void Dispose(bool _)
	{
		if (m_handle != IntPtr.Zero)
		{
			VirtualMIDIClosePort(m_handle);
			m_handle = IntPtr.Zero;
		}
	}
	~VirtualMidi()
	{
		Dispose(false);
	}

	/// <summary>"Turn Off" the virtual midi device</summary>
	public void Shutdown()
	{
		Check(VirtualMIDIShutdown(m_handle));
	}

	/// <summary>Logging mode</summary>
	public static ELogMode LogMode
	{
		//get { }
		set { VirtualMIDILogging((uint)value); }
	}
	[Flags] public enum ELogMode :uint
	{
		None = 0,

		/// <summary>Log internal stuff (port enable, disable...)</summary>
		Misc = 1 << 0,

		/// <summary>Log data received from the driver</summary>
		RX = 1 << 1,

		/// <summary>Log data sent to the driver </summary>
		TX = 1 << 2,

		/// <summary>Log all data</summary>
		All = Misc | RX | TX,
	}

	/// <summary>Send a MIDI command</summary>
	public void SendCommand(Midi.Command command)
	{
		Check(VirtualMIDISendData(m_handle, command.Bytes, (uint)command.Bytes.Length));
	}

	/// <summary>Read a MIDI command</summary>
	public Midi.Command GetCommand()
	{
		var length = (uint)m_read_buffer.Length;
		Check(VirtualMIDIGetData(m_handle, m_read_buffer, ref length));

		var out_bytes = new byte[length];
		Array.Copy(m_read_buffer, out_bytes, out_bytes.Length);
		return new Midi.Command(out_bytes);
	}
	private byte[] m_read_buffer;

	/// <summary></summary>
	public ulong[] ProcessIds
	{
		get
		{
			var length = (uint)m_read_process_ids.Length * sizeof(ulong);
			Check(VirtualMIDIGetProcesses(m_handle, m_read_process_ids, ref length));

			var out_ids = new UInt64[length / sizeof(ulong)];
			Array.Copy(m_read_process_ids, out_ids, out_ids.Length);
			return out_ids;
		}
	}
	private ulong[] m_read_process_ids;

	/// <summary>Handled returned boolean results with an exception</summary>
	private void Check(bool success)
	{
		if (success) return;
		throw new Exception((Exception.EErrorCode)Marshal.GetLastWin32Error());
	}

	#region Exception

	/// <summary>MIDI exceptions</summary>
	[Serializable()]
	public class Exception :System.Exception
	{
		/// <summary>Win32 error codes that the native teVirtualMIDI-driver is using to communicate specific problems to the application</summary>
		public enum EErrorCode
		{
			PATH_NOT_FOUND    = 3,
			INVALID_HANDLE    = 6,
			TOO_MANY_CMDS     = 56,
			TOO_MANY_SESS     = 69,
			INVALID_NAME      = 123,
			MOD_NOT_FOUND     = 126,
			BAD_ARGUMENTS     = 160,
			ALREADY_EXISTS    = 183,
			OLD_WIN_VERSION   = 1150,
			REVISION_MISMATCH = 1306,
			ALIAS_EXISTS      = 1379,
		}

		public Exception()
			:base()
		{}
		public Exception(EErrorCode reason_code)
			:base(ReasonCodeToString(reason_code))
		{
			ReasonCode = reason_code;
		}
		public Exception(string message)
			:base(message)
		{}
		public Exception(string message, System.Exception inner)
			:base(message, inner)
		{}
		protected Exception(System.Runtime.Serialization.SerializationInfo info, System.Runtime.Serialization.StreamingContext context)
		{}

		/// <summary>The Win32 error code</summary>
		public EErrorCode ReasonCode { get; set; }

		/// <summary>The reason, as a string</summary>
		public string ReasonMessage
		{
			get { return ReasonCodeToString(ReasonCode); }
		}

		/// <summary>Convert the Win32 error code to a string</summary>
		private static string ReasonCodeToString(EErrorCode reason_code)
		{
			switch (reason_code)
			{
			default:                           return $"Unspecified virtualMIDI-error: {reason_code}";
			case EErrorCode.OLD_WIN_VERSION:   return "Your Windows-version is too old for dynamic MIDI-port creation.";
			case EErrorCode.INVALID_NAME:      return "You need to specify at least 1 character as MIDI-port name!";
			case EErrorCode.ALREADY_EXISTS:    return "The name for the MIDI-port you specified is already in use!";
			case EErrorCode.ALIAS_EXISTS:      return "The name for the MIDI-port you specified is already in use!";
			case EErrorCode.PATH_NOT_FOUND:    return "Possibly the teVirtualMIDI-driver has not been installed!";
			case EErrorCode.MOD_NOT_FOUND:     return "The teVirtualMIDIxx.dll could not be loaded!";
			case EErrorCode.REVISION_MISMATCH: return "The teVirtualMIDIxx.dll and teVirtualMIDI.sys driver differ in version!";
			case EErrorCode.TOO_MANY_SESS:     return "Maximum number of ports reached";
			case EErrorCode.INVALID_HANDLE:    return "Port not enabled";
			case EErrorCode.TOO_MANY_CMDS:     return "MIDI-command too large";
			case EErrorCode.BAD_ARGUMENTS:     return "Invalid flags specified";
			}
		}
	}

	#endregion

	#region DLL Interop

	// Retrieve version-info from DLL
	static VirtualMidi()
	{
		{
			var vers = VirtualMIDIGetVersion(out var major, out var minor, out var release, out var build);
			Version = new Version(major, minor, build, release);
			VersionString = Marshal.PtrToStringAuto(vers) ?? string.Empty;
		}
		{
			var vers = VirtualMIDIGetDriverVersion(out var major, out var minor, out var release, out var build);
			DriverVersion = new Version(major, minor, build, release);
			DriverVersionString = Marshal.PtrToStringAuto(vers) ?? string.Empty;
		}
	}

	/// <summary>TeVirtualMIDI interface DLL, either 32 or 64 bit</summary>
	private const string DllName = "teVirtualMIDI.dll";

	/// <summary>DLL version</summary>
	public static Version Version { get; private set; }

	/// <summary>DLL version string</summary>
	public static string VersionString { get; private set; }

	/// <summary>Driver version</summary>
	public static Version DriverVersion { get; private set; }

	/// <summary>Driver version string</summary>
	public static string DriverVersionString { get; private set; }

	/// <summary>Callback for CreatePortEx</summary>
	public delegate void Callback (IntPtr handle, [MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] byte[] midi_data_bytes, uint length, IntPtr ctx);

	/// <summary>
	/// VirtualMIDICreatePortEx2
	/// You can specify a name for the device to be created. Each named port can only exist once on a system.
	/// 
	/// When the application terminates, the port will be deleted (or if the public front-end of the port is already in use by a DAW-application,
	/// it will become inactive - giving back appropriate errors to the application using this port.
	/// 
	/// In addition to the name, you can supply a callback-interface, which will be called for all MIDI-data received by the virtual-midi port.
	/// You can also provide instance-data, which will also be handed back within the callback, to have the ability to reference port-specific
	/// data-structures within your application.
	/// 
	/// If you specify "NULL" for the callback function, you will not receive any callback, but can call the blocking function "virtualMIDIGetData"
	/// to retrieve received MIDI-data/commands.  This is especially useful if one wants to interface this library to managed code like .NET or
	/// Java, where callbacks into managed code are potentially complex or dangerous.  A call to virtualMIDIGetData when a callback has been
	/// set during the creation will return with "ERROR_INVALID_FUNCTION".
	/// 
	/// If you specified TE_VM_FLAGS_PARSE_RX in the flags parameter, you will always get one fully valid, pre-parsed MIDI-command in each callback.
	/// In maxSysexLength you should specify a value that is large enough for the maximum size of SysEx that you expect to receive.  SysEx-commands
	/// larger than the value specified here will be discarded and not sent to the user.  Real time-MIDI-commands will never be "intermingled" with
	/// other commands (either normal or SysEx) in this mode.  If a real time-MIDI-command is detected, it is sent to the application before the
	/// command that it was intermingled with.
	/// 
	/// If you specify a maxSysexLength smaller than 2, you will receive fully valid pre-parsed MIDI-commands, but no SysEx-commands, since a
	/// SysEx-command must be at least composed of 0xf0 + 0xf7 (start and end of SysEx).  Since the parser will never be able to construct a
	/// valid SysEx, you will receive none - but all other MIDI-commands will be parsed out and sent to you.
	/// 
	/// When a NULL-pointer is handed back to the application, creation failed.  You can check GetLastError() to find out the specific problem
	/// why the port could not be created.
	/// Note: VirtualMIDICreatePort / VirtualMIDICreatePortEx are deprecated</summary> 
	[DllImport(DllName, EntryPoint = "virtualMIDICreatePortEx2", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern IntPtr VirtualMIDICreatePortEx2(string portName, Callback? callback, IntPtr dwCallbackInstance, UInt32 maxSysexLength, UInt32 flags);

	///<summary>
	/// VirtualMIDICreatePortEx3
	/// This version of the function adds the ability to provide two pointers to GUIDs that can be used to set the manufacturer and product guids
	/// that an application using the public port can retrieve using midiInGetDevCaps or midiOutGetDevCaps with the extended device-capability-
	/// structures (MIDIINCAPS2 and MIDIOUTCAPS2).  If those pointers are set to NULL, the default-guids of the teVirtualMIDI-driver will be used.</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDICreatePortEx3", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern IntPtr VirtualMIDICreatePortEx3(string portName, Callback? callback, IntPtr dwCallbackInstance, UInt32 maxSysexLength, UInt32 flags, ref Guid manufacturer, ref Guid product);

	/// <summary>
	/// With this function, you can close a virtual MIDI-port again, after you have instantiated it.
	/// After the return from this function, no more callbacks will be received.
	/// Beware: do not call this function from within the midi-port-data-callback.  This may result in a deadlock!</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDIClosePort", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern void VirtualMIDIClosePort(IntPtr instance);

	/// <summary>
	/// With this function you can abort the created midi port. This may be useful in case you want
	/// to use the virtualMIDIGetData function which is blocking until it gets data. After this call has been
	/// issued, the port will be shut-down and any further call (other than virtualMIDIClosePort) will fail.</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDIShutdown", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern Boolean VirtualMIDIShutdown(IntPtr instance);

	///<summary>
	/// With this function you can send a buffer of MIDI-data to the driver / the application that opened the virtual-MIDI-port.
	/// If this function returns false, you may check GetLastError() to find out what caused the problem.
	///
	/// This function should always be called with a single complete and valid midi-command (1-3 octets, or possibly more
	/// for SysEx).  SysEx-commands should not be split!  Real time-MIDI-commands shall not be intermingled with other MIDI-
	/// commands, but sent separately!
	///
	/// The data-size that can be used to send data to the virtual ports may be limited in size to prevent
	/// an erratic application to allocate too much of the limited kernel-memory thus interfering with
	/// system-stability.  The current limit is 512kb.</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDISendData", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern Boolean VirtualMIDISendData(IntPtr midiPort, byte[] midiDataBytes, UInt32 length);

	/// <summary>
	/// With this function you can use virtualMIDI without usage of callbacks.  This is especially interesting
	/// if you want to interface the DLL to managed environments like Java or .NET where callbacks from native
	/// to managed code are more complex.
	///
	/// To use it, you need to open a virtualMIDI-port specifying NULL as callback.  If you have specified a
	/// callback when opening the port, this function will fail - you cannot mix callbacks & reading via this
	/// function.
	///
	/// You need to provide a buffer large enough to retrieve the amount of data available.  Otherwise the
	/// function will fail and return to you the necessary size in the length parameter.  If you specify
	/// midiDataBytes to be NULL, the function will succeed but only return the size of buffer necessary
	/// to retrieve the next MIDI-packet.
	///
	/// virtualMIDIGetData will block until a complete block of data is available.  Depending on the fact if
	/// you have specified to parse data into valid commands or just chunks of unprocessed data, you will
	/// either receive the unparsed chunk (possibly containing multiple MIDI-commands), or a single, fully
	/// valid MIDI-command.  In both cases, the length parameter will be filled with the length of data retrieved.
	///
	/// You may only call virtualMIDIGetData once concurrently.  A call to this function will fail if another
	/// call to this function is still not completed.</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDIGetData", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern Boolean VirtualMIDIGetData(IntPtr midiPort, [Out] byte[] midiDataBytes, ref UInt32 length);

	/// <summary>
	/// With this function an application can find out the process-ids of all applications
	/// that are currently using this virtual MIDI port
	/// A pointer to an array of ULONG64s must be supplied.  Currently no more than 16 process ids are returned
	/// Before calling the length is the size of the buffer provided by the application in bytes
	/// After calling the length is the number of process-ids returned times sizeof(ULONG64)</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDIGetProcesses", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern Boolean VirtualMIDIGetProcesses(IntPtr midiPort, [Out] UInt64[] processIds, ref UInt32 length);

	/// <summary>
	/// With this function you can retrieve the version of the driver that you are using.
	/// In addition you will receive the version-number as a wide-string constant as return-value.</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDIGetVersion", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern IntPtr VirtualMIDIGetVersion(out ushort major, out ushort minor, out ushort release, out ushort build);

	/// <summary>
	/// With this function you can retrieve the version of the driver that you are using.
	/// In addition you will receive the version-number as a wide-string constant as return-value.</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDIGetDriverVersion", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern IntPtr VirtualMIDIGetDriverVersion(out ushort major, out ushort minor, out ushort release, out ushort build);

	/// <summary>
	/// With this function logging can be activated into DbgView.
	/// Please specify a bitmask made up form binary "or"ed values from TE_VM_LOGGING_XXX</summary>
	[DllImport(DllName, EntryPoint = "virtualMIDILogging", SetLastError = true, CharSet = CharSet.Unicode)]
	private static extern UInt32 VirtualMIDILogging(UInt32 loggingMask);

	#endregion
}
