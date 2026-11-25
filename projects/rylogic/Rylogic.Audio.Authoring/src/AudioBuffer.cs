using System;
using System.Buffers.Binary;
using System.IO;

namespace Rylogic.Audio.Authoring;

/// <summary>Uncompressed audio as interleaved 32-bit float samples. Authoring always works at <see cref="SampleRate"/>.</summary>
public sealed class AudioBuffer
{
	/// <summary>The sample rate of all authoring audio, in Hz.</summary>
	public const int SampleRate = 48000;

	/// <summary>Interleaved samples: frame 'f', channel 'c' is at 'f * Channels + c'.</summary>
	public float[] Samples { get; }

	/// <summary>The number of channels (1 or 2).</summary>
	public int Channels { get; }

	/// <summary>
	/// The first frame of the loop region, which always runs to the end of the audio. When set, a looping sound plays the whole audio once, then
	/// repeats only the loop region. Null when the whole audio loops.
	/// </summary>
	public int? LoopStart { get; }

	public AudioBuffer(float[] samples, int channels, int? loop_start = null)
	{
		// Whole frames only, so every channel has the same length
		if (channels is < 1 or > 2)
			throw new ArgumentOutOfRangeException(nameof(channels), channels, "Authoring audio is mono or stereo.");
		if (samples.Length % channels != 0)
			throw new ArgumentException("The sample count must be a whole number of frames.", nameof(samples));
		if (loop_start is int start && (start < 0 || start >= samples.Length / channels))
			throw new ArgumentOutOfRangeException(nameof(loop_start), loop_start, "The loop start must be a frame within the audio.");

		Samples = samples;
		Channels = channels;
		LoopStart = loop_start;
	}

	/// <summary>The number of frames (samples per channel).</summary>
	public int Frames => Samples.Length / Channels;

	/// <summary>The length in seconds.</summary>
	public double Seconds => (double)Frames / SampleRate;

	/// <summary>A deep copy.</summary>
	public AudioBuffer Clone()
	{
		return new AudioBuffer((float[])Samples.Clone(), Channels, LoopStart);
	}

	/// <summary>Convert a time in seconds to a frame index, clamped to the buffer.</summary>
	public int FrameAt(double seconds)
	{
		return (int)Math.Clamp(Math.Round(seconds * SampleRate), 0, Frames);
	}
}

/// <summary>Reads and writes 32-bit float WAV files.</summary>
public static class WavFile
{
	/// <summary>The size of the 'smpl' chunk written for a loop start, including its 8 byte chunk header: a 36 byte header and one 24 byte loop.</summary>
	private const int SampleChunkSize = 8 + 36 + 24;

	/// <summary>
	/// Write 'buffer' as a 32-bit float WAV, so identical audio always gives identical bytes. The only metadata is a 'smpl' chunk holding one forward
	/// loop from <see cref="AudioBuffer.LoopStart"/> to the last frame, when a loop start is set.
	/// </summary>
	public static byte[] Encode(AudioBuffer buffer)
	{
		// RIFF header, 'fmt ' chunk for IEEE float, then the 'data' chunk
		var data_size = buffer.Samples.Length * 4;
		var smpl_size = buffer.LoopStart is null ? 0 : SampleChunkSize;
		var bytes = new byte[44 + data_size + smpl_size];
		var span = bytes.AsSpan();
		"RIFF"u8.CopyTo(span);
		BinaryPrimitives.WriteInt32LittleEndian(span[4..], 36 + data_size + smpl_size);
		"WAVE"u8.CopyTo(span[8..]);
		"fmt "u8.CopyTo(span[12..]);
		BinaryPrimitives.WriteInt32LittleEndian(span[16..], 16);
		BinaryPrimitives.WriteInt16LittleEndian(span[20..], 3);
		BinaryPrimitives.WriteInt16LittleEndian(span[22..], (short)buffer.Channels);
		BinaryPrimitives.WriteInt32LittleEndian(span[24..], AudioBuffer.SampleRate);
		BinaryPrimitives.WriteInt32LittleEndian(span[28..], AudioBuffer.SampleRate * buffer.Channels * 4);
		BinaryPrimitives.WriteInt16LittleEndian(span[32..], (short)(buffer.Channels * 4));
		BinaryPrimitives.WriteInt16LittleEndian(span[34..], 32);
		"data"u8.CopyTo(span[36..]);
		BinaryPrimitives.WriteInt32LittleEndian(span[40..], data_size);

		// Samples are little-endian floats on every supported platform
		for (int i = 0; i != buffer.Samples.Length; ++i)
			BinaryPrimitives.WriteSingleLittleEndian(span[(44 + i * 4)..], buffer.Samples[i]);

		// The 'smpl' chunk follows the data. Unused fields stay zero, which also selects a forward loop that repeats as many times as the player asks.
		if (buffer.LoopStart is int loop_start)
		{
			var smpl = span[(44 + data_size)..];
			"smpl"u8.CopyTo(smpl);
			BinaryPrimitives.WriteInt32LittleEndian(smpl[4..], SampleChunkSize - 8);
			BinaryPrimitives.WriteInt32LittleEndian(smpl[(8 + 12)..], (int)Math.Round(1e9 / AudioBuffer.SampleRate));
			BinaryPrimitives.WriteInt32LittleEndian(smpl[(8 + 28)..], 1);
			BinaryPrimitives.WriteInt32LittleEndian(smpl[(8 + 36 + 8)..], loop_start);
			BinaryPrimitives.WriteInt32LittleEndian(smpl[(8 + 36 + 12)..], buffer.Frames - 1);
		}
		return bytes;
	}

	/// <summary>
	/// Read a 48 kHz 32-bit float WAV from 'stream'. The data chunk may have an unknown size (as written by a pipe), in which case the samples run to the
	/// end of the stream.
	/// </summary>
	public static AudioBuffer Decode(Stream stream)
	{
		// Read everything, because a piped WAV cannot be seeked
		using var memory = new MemoryStream();
		stream.CopyTo(memory);
		var bytes = memory.GetBuffer().AsSpan(0, (int)memory.Length);
		if (bytes.Length < 12 || !bytes[..4].SequenceEqual("RIFF"u8) || !bytes[8..12].SequenceEqual("WAVE"u8))
			throw new InvalidDataException("Not a WAV file.");

		// Walk every chunk, because the 'smpl' chunk may come before or after the data
		var channels = 0;
		var samples = (float[]?)null;
		var loop_start = (int?)null;
		for (var offset = 12; offset + 8 <= bytes.Length;)
		{
			var id = bytes.Slice(offset, 4);
			var size = BinaryPrimitives.ReadUInt32LittleEndian(bytes[(offset + 4)..]);
			var body = offset + 8;
			if (id.SequenceEqual("fmt "u8))
			{
				// Only the format authoring produces is accepted, so a mismatch is reported rather than misread
				var format = BinaryPrimitives.ReadInt16LittleEndian(bytes[body..]);
				channels = BinaryPrimitives.ReadInt16LittleEndian(bytes[(body + 2)..]);
				var rate = BinaryPrimitives.ReadInt32LittleEndian(bytes[(body + 4)..]);
				var bits = BinaryPrimitives.ReadInt16LittleEndian(bytes[(body + 14)..]);
				if (format is not 3 and not -2 || bits != 32 || rate != AudioBuffer.SampleRate)
					throw new InvalidDataException($"Expected 48 kHz 32-bit float WAV, found format {format}, {bits} bits, {rate} Hz.");
			}
			else if (id.SequenceEqual("data"u8))
			{
				// A pipe writes an unknown (maximum) size, so take whatever remains
				if (channels == 0)
					throw new InvalidDataException("WAV data chunk before its format chunk.");

				var available = bytes.Length - body;
				var length = size > available ? available : (int)size;
				length -= length % (4 * channels);
				samples = new float[length / 4];
				for (int i = 0; i != samples.Length; ++i)
					samples[i] = BinaryPrimitives.ReadSingleLittleEndian(bytes[(body + i * 4)..]);
			}
			else if (id.SequenceEqual("smpl"u8) && size >= 36 + 24 && BinaryPrimitives.ReadInt32LittleEndian(bytes[(body + 28)..]) != 0)
			{
				// Only the first loop is used; its region runs to the end of the audio, so its end is not needed
				loop_start = BinaryPrimitives.ReadInt32LittleEndian(bytes[(body + 36 + 8)..]);
			}
			offset = body + (int)Math.Min((long)size + (size & 1), bytes.Length - body);
		}
		if (samples is null)
			throw new InvalidDataException("WAV file has no data chunk.");

		return new AudioBuffer(samples, channels, loop_start);
	}

	/// <summary>Read a 48 kHz 32-bit float WAV file.</summary>
	public static AudioBuffer Read(string path)
	{
		using var stream = File.OpenRead(path);
		return Decode(stream);
	}

	/// <summary>
	/// The loop start, in seconds, of the first loop in the 'smpl' chunk of the WAV file at 'path', in any sample format. Null when the file has no loop,
	/// so the whole sound loops, or is not a WAV file.
	/// </summary>
	public static double? ReadLoopStart(string path)
	{
		// Walk the chunk headers, seeking over chunk bodies so large files are not read
		using var stream = File.OpenRead(path);
		using var reader = new BinaryReader(stream);
		if (stream.Length < 12)
			return null;

		var riff = reader.ReadUInt32();
		reader.ReadUInt32();
		if (riff != 0x46464952 || reader.ReadUInt32() != 0x45564157)
			return null;

		var rate = 0;
		var loop_start = (int?)null;
		while (stream.Position + 8 <= stream.Length)
		{
			// Chunks are padded to an even size; the 'fmt ' sample rate converts the loop's frame index to seconds
			var id = reader.ReadUInt32();
			var size = reader.ReadUInt32();
			var next = stream.Position + size + (size & 1);
			if (id == 0x20746d66 && size >= 8)
			{
				reader.ReadInt32();
				rate = reader.ReadInt32();
			}
			else if (id == 0x6c706d73 && size >= 36 + 24)
			{
				reader.ReadBytes(28);
				var count = reader.ReadInt32();
				reader.ReadBytes(4 + 8);
				if (count != 0)
					loop_start = reader.ReadInt32();
			}
			stream.Position = Math.Min(next, stream.Length);
		}
		return loop_start is int start && rate > 0 ? (double)start / rate : null;
	}
}
