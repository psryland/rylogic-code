using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio;

/// <summary>An immutable, defensively-copied encoded audio payload ready to create a clip or stream.</summary>
public sealed class AudioData
{
	private readonly byte[] m_bytes;

	/// <summary>Adopt an already-owned, already-validated byte array without a further copy.</summary>
	private AudioData(byte[] bytes, EAudioEncoding encoding)
	{
		m_bytes = bytes;
		Encoding = encoding;
	}

	/// <summary>The encoding of the payload.</summary>
	public EAudioEncoding Encoding { get; }

	/// <summary>The number of encoded bytes.</summary>
	public int Length
	{
		get
		{
			return m_bytes.Length;
		}
	}

	/// <summary>A read-only view of the encoded payload suitable for passing to the native engine.</summary>
	internal ReadOnlySpan<byte> Span
	{
		get
		{
			return m_bytes;
		}
	}

	/// <summary>Create audio data from an in-memory wave-encoded payload. The payload is copied.</summary>
	public static AudioData FromWave(ReadOnlySpan<byte> wave_bytes)
	{
		return new AudioData(CopyNonEmpty(wave_bytes), EAudioEncoding.Wave);
	}

	/// <summary>Create audio data from an in-memory wave-encoded payload. The payload is copied.</summary>
	public static AudioData FromWave(ReadOnlyMemory<byte> wave_bytes)
	{
		return FromWave(wave_bytes.Span);
	}

	/// <summary>Create audio data from an in-memory Ogg Vorbis-encoded payload. The payload is copied.</summary>
	public static AudioData FromOggVorbis(ReadOnlySpan<byte> ogg_bytes)
	{
		return new AudioData(CopyNonEmpty(ogg_bytes), EAudioEncoding.OggVorbis);
	}

	/// <summary>Create audio data from an in-memory Ogg Vorbis-encoded payload. The payload is copied.</summary>
	public static AudioData FromOggVorbis(ReadOnlyMemory<byte> ogg_bytes)
	{
		return FromOggVorbis(ogg_bytes.Span);
	}

	/// <summary>Read a file synchronously and detect its encoding from its leading magic bytes.</summary>
	public static AudioData Load(string path)
	{
		if (path == null)
			throw new ArgumentNullException(nameof(path));

		var bytes = File.ReadAllBytes(path);
		return FromDetectedEncoding(bytes, path);
	}

	/// <summary>
	/// Read a file asynchronously and detect its encoding from its leading magic bytes. Performs file I/O only; creating
	/// a clip or stream from the result always requires a further synchronous call on the engine's owner thread.
	/// </summary>
	public static async Task<AudioData> LoadAsync(string path, CancellationToken cancellation_token = default)
	{
		if (path == null)
			throw new ArgumentNullException(nameof(path));

		// A FileStream opened for asynchronous I/O plus a manual read loop keeps this compatible with net481, which has
		// no File.ReadAllBytesAsync overload.
		using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 4096, useAsync: true);
		var length = checked((int)stream.Length);
		var bytes = new byte[length];
		var offset = 0;
		while (offset < length)
		{
			var read = await stream.ReadAsync(bytes, offset, length - offset, cancellation_token).ConfigureAwait(false);
			if (read == 0)
				throw new EndOfStreamException($"Unexpected end of file while reading '{path}'.");

			offset += read;
		}

		return FromDetectedEncoding(bytes, path);
	}

	/// <summary>Detect an encoding from leading magic bytes and adopt the buffer without a further copy.</summary>
	private static AudioData FromDetectedEncoding(byte[] bytes, string path)
	{
		if (IsWave(bytes))
			return new AudioData(bytes, EAudioEncoding.Wave);
		if (IsOggVorbis(bytes))
			return new AudioData(bytes, EAudioEncoding.OggVorbis);

		throw new NotSupportedException($"'{path}' is not a recognised wave or Ogg Vorbis file.");
	}

	/// <summary>True when the buffer begins with a RIFF/WAVE header.</summary>
	private static bool IsWave(byte[] bytes)
	{
		return bytes.Length >= 12
			&& bytes[0] == (byte)'R' && bytes[1] == (byte)'I' && bytes[2] == (byte)'F' && bytes[3] == (byte)'F'
			&& bytes[8] == (byte)'W' && bytes[9] == (byte)'A' && bytes[10] == (byte)'V' && bytes[11] == (byte)'E';
	}

	/// <summary>True when the buffer begins with an Ogg page header.</summary>
	private static bool IsOggVorbis(byte[] bytes)
	{
		return bytes.Length >= 4
			&& bytes[0] == (byte)'O' && bytes[1] == (byte)'g' && bytes[2] == (byte)'g' && bytes[3] == (byte)'S';
	}

	/// <summary>Copy a non-empty span into a new owned array.</summary>
	private static byte[] CopyNonEmpty(ReadOnlySpan<byte> bytes)
	{
		if (bytes.Length == 0)
			throw new ArgumentException("Audio data must not be empty.", nameof(bytes));

		return bytes.ToArray();
	}
}
