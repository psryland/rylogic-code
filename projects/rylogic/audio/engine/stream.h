//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Ogg Vorbis decode state for one resident streaming voice. Not part of the public API.
//
// A Stream owns the complete compressed file and decodes it incrementally on a background
// worker thread into small PCM chunks, which the engine owner thread submits to an XAudio2
// source voice. All members below are protected entirely by the single Engine::Impl::m_streams_mutex;
// this type performs no locking of its own, so every access - from either the worker or the
// owner thread - must happen while that mutex is held.
#pragma once
#include "pr/audio/forward.h"
#include "pr/audio/types.h"
#include "audio/engine/handle_util.h"

namespace pr::audio
{
	// One bounded, fully-decoded block of interleaved 16-bit PCM audio.
	struct PcmChunk
	{
		std::vector<std::byte> m_samples;
		std::uint32_t m_frame_count;
	};

	// Present a fixed in-memory byte buffer to libvorbisfile as a seekable stream source.
	class MemorySource
	{
		std::span<std::byte const> m_bytes;
		std::int64_t m_pos;

	public:
		explicit MemorySource(std::span<std::byte const> bytes);

		// Return the ov_open_callbacks table bound to the static member functions below.
		static ov_callbacks Callbacks();

	private:
		static std::size_t Read(void* ptr, std::size_t size, std::size_t nmemb, void* datasource);
		static int Seek(void* datasource, ogg_int64_t offset, int whence);
		static int Close(void* datasource);
		static long Tell(void* datasource);
	};

	// Decode state and XAudio2 submission bookkeeping for one resident Ogg Vorbis stream.
	struct Stream
	{
		// Record callback state without invoking user code from XAudio2 threads. The engine never flags a
		// stream buffer as end-of-stream, so completion is instead detected once decode has permanently
		// ended and every submitted buffer has been reconciled through m_completed_count.
		struct Callback : IXAudio2VoiceCallback
		{
			std::atomic<std::uint32_t> m_completed_count;
			std::atomic<std::uint32_t> m_queued_count;
			std::atomic<bool> m_underrun;

			Callback();

			// Clear submission bookkeeping after a source voice is destroyed or its queue is discarded.
			void Reset();

			void STDMETHODCALLTYPE OnVoiceProcessingPassStart(UINT32 bytes_required_this_pass) override;
			void STDMETHODCALLTYPE OnVoiceProcessingPassEnd() override {}
			void STDMETHODCALLTYPE OnStreamEnd() override {}
			void STDMETHODCALLTYPE OnBufferStart(void*) override {}
			void STDMETHODCALLTYPE OnBufferEnd(void*) override;
			void STDMETHODCALLTYPE OnLoopEnd(void*) override {}
			void STDMETHODCALLTYPE OnVoiceError(void*, HRESULT) override {}
		};

		StreamDesc m_desc;
		std::vector<std::byte> m_compressed; // Complete copied compressed file, owned for the stream's lifetime.
		MemorySource m_source;
		OggVorbis_File m_ogg;
		std::uint32_t m_channel_count;
		std::uint32_t m_sample_rate;
		std::uint32_t m_block_align; // channel_count * sizeof(std::int16_t)
		std::uint64_t m_pcm_total;
		std::uint32_t m_loops_remaining; // AUDIO_INFINITE_LOOP for an unbounded loop, else remaining automatic replays.
		bool m_decode_ended; // True once decode has produced its final chunk (loops exhausted or a corrupt read).
		bool m_decode_error; // True if decode ended because of corrupt compressed data rather than a clean EOF.
		std::deque<PcmChunk> m_pending; // Decoded, not yet submitted to XAudio2 (worker-produced, owner-consumed).
		std::uint64_t m_pending_frames; // Total frames currently queued in m_pending.

		// Owner-thread-only playback bookkeeping (never touched by the decode worker).
		EPlaybackState m_playback;
		std::deque<PcmChunk> m_inflight; // Submitted to XAudio2, not yet completed (FIFO; relies on XAudio2's per-voice completion order).
		std::uint64_t m_position_base; // PCM frames consumed before the currently in-flight and pending chunks.
		std::uint32_t m_underrun_count;
		Callback m_callback;
		XAudioVoicePtr<IXAudio2SourceVoice> m_source_voice;
		bool m_source_started;

		// Open an Ogg Vorbis decoder over a complete copy of a compressed file. Throws Exception(UnsupportedFormat)
		// if the header cannot be parsed.
		explicit Stream(StreamDesc const& desc, std::vector<std::byte> ogg_file);
		~Stream();
		Stream(Stream const&) = delete;
		Stream& operator=(Stream const&) = delete;

		// Return the total number of frames currently buffered ahead of playback (pending plus in-flight).
		std::uint64_t BufferedFrameCount() const;

		// Decode one bounded chunk of PCM if the stream is not already at a permanent end. Returns true if a
		// real chunk of audio was appended to m_pending, or false if none was produced (either because decode
		// had already ended, or because this call is what discovered the end - m_decode_ended is then set).
		bool DecodeOneChunk(std::size_t target_bytes);

		// Remove and return the next pending decoded chunk, or nullopt if none is ready yet.
		std::optional<PcmChunk> PopPendingChunk();

		// Reposition decoding to a PCM frame, discarding any buffered-but-unsubmitted audio.
		void Seek(std::uint64_t pcm_position);
	};
}
