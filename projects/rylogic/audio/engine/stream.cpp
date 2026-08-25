//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/audio/forward.h"
#include "pr/audio/engine.h"
#include "audio/engine/stream.h"

namespace pr::audio
{
	MemorySource::MemorySource(std::span<std::byte const> bytes)
		: m_bytes(bytes)
		, m_pos(0)
	{}

	// Return the ov_open_callbacks table bound to the static member functions below.
	ov_callbacks MemorySource::Callbacks()
	{
		return ov_callbacks{
			.read_func = &MemorySource::Read,
			.seek_func = &MemorySource::Seek,
			.close_func = &MemorySource::Close,
			.tell_func = &MemorySource::Tell,
		};
	}

	// Copy up to size*nmemb bytes from the current cursor, returning the number of whole elements copied.
	std::size_t MemorySource::Read(void* ptr, std::size_t size, std::size_t nmemb, void* datasource)
	{
		if (size == 0)
			return 0;

		auto& self = *static_cast<MemorySource*>(datasource);
		auto available = static_cast<std::size_t>(self.m_bytes.size() - self.m_pos);
		auto requested = nmemb > std::numeric_limits<std::size_t>::max() / size
			? std::numeric_limits<std::size_t>::max()
			: size * nmemb;
		auto count = std::min(requested, available) / size;
		auto bytes = count * size;
		if (bytes != 0)
		{
			std::memcpy(ptr, self.m_bytes.data() + self.m_pos, bytes);
			self.m_pos += static_cast<std::int64_t>(bytes);
		}
		return count;
	}

	// Reposition the cursor within the fixed buffer, matching fseek's whence semantics.
	int MemorySource::Seek(void* datasource, ogg_int64_t offset, int whence)
	{
		auto& self = *static_cast<MemorySource*>(datasource);
		auto target = std::int64_t{};
		switch (whence)
		{
			case SEEK_SET: { target = offset; break; }
			case SEEK_CUR: { target = self.m_pos + offset; break; }
			case SEEK_END: { target = static_cast<std::int64_t>(self.m_bytes.size()) + offset; break; }
			default: { return -1; }
		}
		if (target < 0 || target > static_cast<std::int64_t>(self.m_bytes.size()))
			return -1;

		self.m_pos = target;
		return 0;
	}

	int MemorySource::Close(void*)
	{
		return 0;
	}

	long MemorySource::Tell(void* datasource)
	{
		auto& self = *static_cast<MemorySource*>(datasource);
		return static_cast<long>(self.m_pos);
	}

	Stream::Callback::Callback()
		: m_completed_count(0)
		, m_queued_count(0)
		, m_underrun(false)
	{}

	// Clear submission bookkeeping after a source voice is destroyed or its queue is discarded.
	void Stream::Callback::Reset()
	{
		m_completed_count.store(0, std::memory_order_release);
		m_queued_count.store(0, std::memory_order_release);
		m_underrun.store(false, std::memory_order_release);
	}

	void STDMETHODCALLTYPE Stream::Callback::OnVoiceProcessingPassStart(UINT32 bytes_required_this_pass)
	{
		// A refill request is an underrun only after XAudio2 has consumed every submitted buffer.
		if (bytes_required_this_pass != 0 && m_queued_count.load(std::memory_order_acquire) == 0)
			m_underrun.store(true, std::memory_order_relaxed);
	}

	void STDMETHODCALLTYPE Stream::Callback::OnBufferEnd(void*)
	{
		// XAudio2 completes buffers for one voice strictly in submission order, so a plain count is
		// enough for the owner thread to know how many m_inflight chunks have finished.
		auto queued = m_queued_count.load(std::memory_order_acquire);
		while (queued != 0 && !m_queued_count.compare_exchange_weak(queued, queued - 1, std::memory_order_acq_rel))
		{}
		m_completed_count.fetch_add(1, std::memory_order_acq_rel);
	}

	// Open an Ogg Vorbis decoder over a complete copy of a compressed file. Throws Exception(UnsupportedFormat)
	// if the header cannot be parsed.
	Stream::Stream(StreamDesc const& desc, std::vector<std::byte> ogg_file)
		: m_desc(desc)
		, m_compressed(std::move(ogg_file))
		, m_source(std::span<std::byte const>(m_compressed))
		, m_ogg()
		, m_channel_count(0)
		, m_sample_rate(0)
		, m_block_align(0)
		, m_pcm_total(0)
		, m_loops_remaining(desc.loop_count)
		, m_decode_ended(false)
		, m_decode_error(false)
		, m_pending()
		, m_pending_frames(0)
		, m_playback(EPlaybackState::Stopped)
		, m_inflight()
		, m_position_base(0)
		, m_underrun_count(0)
		, m_callback()
		, m_source_voice()
		, m_source_started(false)
	{
		if (ov_open_callbacks(&m_source, &m_ogg, nullptr, 0, MemorySource::Callbacks()) != 0)
			throw Exception(EStatus::UnsupportedFormat, "Ogg Vorbis stream could not be opened");

		auto* info = ov_info(&m_ogg, -1);
		if (info == nullptr || info->channels <= 0 || info->rate <= 0)
		{
			ov_clear(&m_ogg);
			throw Exception(EStatus::UnsupportedFormat, "Ogg Vorbis stream has no valid audio bitstream");
		}

		m_channel_count = static_cast<std::uint32_t>(info->channels);
		m_sample_rate = static_cast<std::uint32_t>(info->rate);
		m_block_align = m_channel_count * static_cast<std::uint32_t>(sizeof(std::int16_t));

		auto total = ov_pcm_total(&m_ogg, -1);
		m_pcm_total = total >= 0 ? static_cast<std::uint64_t>(total) : 0;
	}

	Stream::~Stream()
	{
		ov_clear(&m_ogg);
	}

	// Return the total number of frames currently buffered ahead of playback (pending plus in-flight).
	std::uint64_t Stream::BufferedFrameCount() const
	{
		auto frames = m_pending_frames;
		for (auto const& chunk : m_inflight)
			frames += chunk.m_frame_count;
		return frames;
	}

	// Return the frames queued after XAudio2's current buffer, whose consumed fraction is not observable.
	std::uint64_t Stream::BufferedSuccessorFrameCount() const
	{
		auto frames = m_pending_frames;
		if (!m_inflight.empty())
		{
			for (auto i = std::size_t{1}; i != m_inflight.size(); ++i)
				frames += m_inflight[i].m_frame_count;
		}
		return frames;
	}

	bool Stream::DecodeOneChunk(std::size_t target_bytes)
	{
		if (m_decode_ended)
			return false;

		// Bound the retry count so a degenerate zero-length loop or a run of holes cannot spin forever.
		for (auto attempt = 0; attempt != 4; ++attempt)
		{
			auto buffer = std::vector<std::byte>(target_bytes);
			auto bitstream = int{};
			auto read = ov_read(&m_ogg, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0, 2, 1, &bitstream);
			if (read > 0)
			{
				buffer.resize(static_cast<std::size_t>(read));
				m_pending.push_back(PcmChunk{
					.m_samples = std::move(buffer),
					.m_frame_count = static_cast<std::uint32_t>(static_cast<std::size_t>(read) / m_block_align),
				});
				m_pending_frames += m_pending.back().m_frame_count;
				return true;
			}
			if (read == 0)
			{
				// Clean end of the compressed stream: loop back to the start, or end decoding permanently.
				if (m_loops_remaining == AUDIO_INFINITE_LOOP || m_loops_remaining != 0)
				{
					if (m_loops_remaining != AUDIO_INFINITE_LOOP)
						--m_loops_remaining;

					ov_pcm_seek(&m_ogg, 0);
					continue;
				}

				m_decode_ended = true;
				return false;
			}
			if (read == OV_HOLE)
				continue; // Recoverable: retry the read at the current position.

			// Any other negative result indicates the compressed data itself is corrupt.
			m_decode_ended = true;
			m_decode_error = true;
			return false;
		}

		m_decode_ended = true;
		m_decode_error = true;
		return false;
	}

	std::optional<PcmChunk> Stream::PopPendingChunk()
	{
		if (m_pending.empty())
			return std::nullopt;

		auto chunk = std::move(m_pending.front());
		m_pending.pop_front();
		m_pending_frames -= chunk.m_frame_count;
		return chunk;
	}

	void Stream::Seek(std::uint64_t pcm_position)
	{
		auto clamped = m_pcm_total != 0 ? std::min(pcm_position, m_pcm_total) : pcm_position;
		if (ov_pcm_seek(&m_ogg, static_cast<ogg_int64_t>(clamped)) != 0)
			throw Exception(EStatus::InternalError, "Ogg Vorbis stream seek failed");

		m_pending.clear();
		m_pending_frames = 0;
		m_position_base = clamped;
		m_decode_ended = false;
		m_decode_error = false;
	}
}
