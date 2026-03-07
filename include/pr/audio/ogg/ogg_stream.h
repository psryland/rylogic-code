//*********************************************
// Ogg Player
//  Copyright (c) Rylogic Ltd 2007
//*********************************************
#pragma once

#include <dsound.h>                  // from the directx 8 sdk
#include "vorbis/codec.h"            // from the vorbis sdk
#include "vorbis/vorbisfile.h"       // also :)
#include "pr/common/assert.h"
#include "pr/common/exception.h"
#include "pr/common/hresult.h"
#include "pr/common/d3dptr.h"
#include "pr/audio/directsound/sound.h"
#include "pr/audio/directsound/player.h"

namespace pr::sound
{
	enum class EOggVorbisResult
	{
		Ok = 0,
		False = OV_FALSE,
		EoF = OV_EOF,
		Hole = OV_HOLE,
		Read = OV_EREAD,
		FAULT = OV_EFAULT,
		Impl = OV_EIMPL,
		Inval = OV_EINVAL,
		NotVorbis = OV_ENOTVORBIS,
		BadHeader = OV_EBADHEADER,
		Version = OV_EVERSION,
		NotAudio = OV_ENOTAUDIO,
		BadPacket = OV_EBADPACKET,
		BadLink = OV_EBADLINK,
		NoSeek = OV_ENOSEEK,
	};
	using OggException = pr::Exception<EOggVorbisResult>;
		
	// A data stream that decodes Ogg file data
	struct OggDataStream :IDataStream
	{
		mutable OggVorbis_File m_ogg; // The vorbis file interface
		IDataStream* m_src; // The data stream containing the raw ogg file data

		OggDataStream() :m_ogg(), m_src()
		{
			ov_clear(&m_ogg);
		}
		OggDataStream(char const* filepath) :m_ogg(), m_src()
		{
			ov_clear(&m_ogg);
			Load(*new MemDataStream(filepath, true));
		}

		// Return a sound buffer appropriate for the ogg data stream. If size == 0 then a size is choosen automatically
		D3DPtr<IDirectSoundBuffer8> CreateBuffer(D3DPtr<IDirectSound8>& dsound, size_t size = 0) const
		{
			PR_ASSERT(PR_DBG_SND, m_src, "'Load()' must be called first");
			int const ogg_bps = 16; // ogg vorbis is always 16 bit
			int const update_rate = 10; // updates_per_sec
			vorbis_info const* vi = ov_info(&m_ogg, -1);
			if (size == 0) size = GetMinRequiredBufferSize(update_rate, vi->channels, vi->rate, ogg_bps);
			return sound::CreateBuffer(dsound, size, vi->channels, vi->rate, ogg_bps);
		}

		// Load an ogg file
		void Load(IDataStream& src)
		{
			if (m_src) m_src->IDataStream_Close();
			m_src = &src;

			// Open the ogg file with provided data stream callbacks
			ov_callbacks cb = {};
			cb.read_func = ReadCB;
			cb.seek_func = SeekCB;
			cb.tell_func = TellCB;
			cb.close_func = CloseCB;
			auto res = ov_open_callbacks(this, &m_ogg, 0, 0, cb);
			if (res < 0) throw OggException(static_cast<EOggVorbisResult>(res), "Failed to open ogg data stream");
		}

	private:

		// IDataStream interface - these methods read from decoded ogg data
		size_t IDataStream_Read(void* ptr, size_t byte_size, size_t count)
		{
			int bitstream;
			auto res = ov_read(&m_ogg, (char*)ptr, int(byte_size * count), 0, 2, 1, &bitstream);
			if (res < 0) throw OggException(static_cast<EOggVorbisResult>(res), "Error in ogg data stream");
			return res;
		}
		int IDataStream_Seek(long offset, int seek_from)
		{
			switch (seek_from)
			{
			default: PR_ASSERT(PR_DBG_SND, false, ""); return -1;
			case SEEK_SET: ov_pcm_seek(&m_ogg, offset); break;
			case SEEK_CUR: ov_pcm_seek(&m_ogg, ov_pcm_tell(&m_ogg) + offset); break;
			case SEEK_END: ov_pcm_seek(&m_ogg, ov_pcm_total(&m_ogg, -1) - offset); break;
			}
			return 0;
		}
		long IDataStream_Tell() const
		{
			return long(ov_pcm_tell(&m_ogg));
		}
		void IDataStream_Close()
		{
			ov_clear(&m_ogg);
			m_src = 0;
		}

		// Ogg callback functions - these read data from 'm_src' for the ogg lib to decode
		static size_t ReadCB(void* ptr, size_t byte_size, size_t count, void* ctx)
		{
			return static_cast<OggDataStream*>(ctx)->m_src->IDataStream_Read(ptr, byte_size, count);
		}
		static int SeekCB(void* ctx, ogg_int64_t offset, int seek_from)
		{
			return static_cast<OggDataStream*>(ctx)->m_src->IDataStream_Seek(long(offset), seek_from);
		}
		static long TellCB(void* ctx)
		{
			return static_cast<OggDataStream*>(ctx)->m_src->IDataStream_Tell();
		}
		static int CloseCB(void* ctx)
		{
			static_cast<OggDataStream*>(ctx)->m_src->IDataStream_Close(); return 0;
		}
	};
}