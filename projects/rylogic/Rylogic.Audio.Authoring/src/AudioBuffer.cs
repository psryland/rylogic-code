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

	public AudioBuffer(float[] samples, int channels)
	{
		// Whole frames only, so every channel has the same length
		if (channels is < 1 or > 2)
			throw new ArgumentOutOfRangeException(nameof(channels), channels, "Authoring audio is mono or stereo.");
		if (samples.Length % channels != 0)
			throw new ArgumentException("The sample count must be a whole number of frames.", nameof(samples));

		Samples = samples;
		Channels = channels;
	}

	/// <summary>The number of frames (samples per channel).</summary>
	public int Frames => Samples.Length / Channels;

	/// <summary>The length in seconds.</summary>
	public double Seconds => (double)Frames / SampleRate;

	/// <summary>A deep copy.</summary>
	public AudioBuffer Clone()
	{
		return new AudioBuffer((float[])Samples.Clone(), Channels);
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
	/// <summary>Write 'buffer' as a 32-bit float WAV with no metadata, so identical audio always gives identical bytes.</summary>
	public static byte[] Encode(AudioBuffer buffer)
	{
		// RIFF header, 'fmt ' chunk for IEEE float, then the 'data' chunk
		var data_size = buffer.Samples.Length * 4;
		var bytes = new byte[44 + data_size];
		var span = bytes.AsSpan();
		"RIFF"u8.CopyTo(span);
		BinaryPrimitives.WriteInt32LittleEndian(span[4..], 36 + data_size);
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

		// Walk the chunks to find the format and the data
		var channels = 0;
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
				var samples = new float[length / 4];
				for (int i = 0; i != samples.Length; ++i)
					samples[i] = BinaryPrimitives.ReadSingleLittleEndian(bytes[(body + i * 4)..]);

				return new AudioBuffer(samples, channels);
			}
			offset = body + (int)Math.Min((long)size + (size & 1), bytes.Length - body);
		}
		throw new InvalidDataException("WAV file has no data chunk.");
	}

	/// <summary>Read a 48 kHz 32-bit float WAV file.</summary>
	public static AudioBuffer Read(string path)
	{
		using var stream = File.OpenRead(path);
		return Decode(stream);
	}
}
