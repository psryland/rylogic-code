//*********************************************
// DSound
//  Copyright (c) Rylogic Ltd 2007
//*********************************************
// Notes:
//   - DirectSound8 is deprecated, prefer XAudio2
#pragma once
#include <mmreg.h>
#include <dsound.h>
#include "pr/common/assert.h"
#include "pr/common/d3dptr.h"
#include "pr/common/hresult.h"
#include "pr/math/math.h"

#ifndef PR_DBG_SND
#define PR_DBG_SND PR_DBG
#endif

namespace pr::sound
{
	// RAII sound buffer lock
	struct Lock
	{
		D3DPtr<IDirectSoundBuffer8> m_buf;
		uint8_t* m_ptr0;
		uint8_t* m_ptr1;
		DWORD m_size0;
		DWORD m_size1;

		// Flags:
		//  0 - Lock from [offset, offset+count)
		//  DSBLOCK_FROMWRITECURSOR - Start the lock at the write cursor. The offset parameter is ignored.
		//  DSBLOCK_ENTIREBUFFER - Lock the entire buffer. The count parameter is ignored.
		Lock(D3DPtr<IDirectSoundBuffer8>& buf, size_t offset, size_t count, size_t flags = 0) :m_buf(buf)
		{
			for(;;)
			{
				HRESULT res = m_buf->Lock(DWORD(offset), DWORD(count), (void**)&m_ptr0, &m_size0, (void**)&m_ptr1, &m_size1, DWORD(flags));
				if (res == DSERR_BUFFERLOST) { pr::Check(m_buf->Restore()); continue; }
				pr::Check(res); break;
			}
		}
		~Lock()
		{
			pr::Check(m_buf->Unlock(m_ptr0, m_size0, m_ptr1, m_size1));
		}
	};

	// Helper function for initialising dsound for an app
	// Device:
	//   0 = use the primary sound driver
	// Flags:
	//  DSSCL_EXCLUSIVE - For DirectX 8.0 and later, has the same effect as DSSCL_PRIORITY. For previous versions, sets the application to
	//      the exclusive level. This means that when it has the input focus, the application will be the only one audible; sounds from
	//      applications with the DSBCAPS_GLOBALFOCUS flag set will be muted. With this level, it also has all the privileges of the
	//      DSSCL_PRIORITY level. DirectSound will restore the hardware format, as specified by the most recent call to the SetFormat
	//      method, after the application gains the input focus.
	//  DSSCL_NORMAL - Sets the normal level. This level has the smoothest multitasking and resource-sharing behavior, but because it
	//      does not allow the primary buffer format to change, output is restricted to the default 8-bit format.
	//  DSSCL_PRIORITY - Sets the priority level. Applications with this cooperative level can call the SetFormat and Compact methods.
	//  DSSCL_WRITEPRIMARY - Sets the write-primary level. The application has write access to the primary buffer. No secondary buffers
	//      can be played. This level cannot be set if the DirectSound driver is being emulated for the device; that is, if the GetCaps
	//      method returns the DSCAPS_EMULDRIVER flag in the DSCAPS structure.
	inline D3DPtr<IDirectSound8> InitDSound(HWND hwnd, GUID const* device = 0, DWORD coop_flags = DSSCL_EXCLUSIVE)
	{
		D3DPtr<IDirectSound8> dsound;
		Check(::DirectSoundCreate8(device, dsound.address_of(), 0));
		Check(dsound->Initialize(0));
		Check(dsound->SetCooperativeLevel(hwnd, coop_flags));
		return dsound;
	}

	// Return the direct sound caps
	inline DSCAPS GetCaps(D3DPtr<IDirectSound8> dsound)
	{
		DSCAPS caps;
		Check(dsound->GetCaps(&caps));
		return caps;
	}

	// Helper function for allocating a dsound buffer
	// Flags:
	//  DSBCAPS_CTRL3D - The buffer has 3D control capability.
	//  DSBCAPS_CTRLFREQUENCY - The buffer has frequency control capability.
	//  DSBCAPS_CTRLFX - The buffer supports effects processing.
	//  DSBCAPS_CTRLPAN - The buffer has pan control capability.
	//  DSBCAPS_CTRLVOLUME - The buffer has volume control capability.
	//  DSBCAPS_CTRLPOSITIONNOTIFY - The buffer has position notification capability. See the Remarks for DSCBUFFERDESC.
	//  DSBCAPS_GETCURRENTPOSITION2 - The buffer uses the new behavior of the play cursor when IDirectSoundBuffer8::GetCurrentPosition
	//        is called. In the first version of DirectSound, the play cursor was significantly ahead of the actual playing
	//        sound on emulated sound cards; it was directly behind the write cursor. Now, if the DSBCAPS_GETCURRENTPOSITION2
	//        flag is specified, the application can get a more accurate play cursor. If this flag is not specified, the old
	//        behavior is preserved for compatibility. This flag affects only emulated devices; if a DirectSound driver is
	//        present, the play cursor is accurate for DirectSound in all versions of DirectX.
	//  DSBCAPS_GLOBALFOCUS - The buffer is a global sound buffer. With this flag set, an application using DirectSound can continue to
	//        play its buffers if the user switches focus to another application, even if the new application uses DirectSound.
	//        The one exception is if you switch focus to a DirectSound application that uses the DSSCL_WRITEPRIMARY flag for
	//        its cooperative level. In this case, the global sounds from other applications will not be audible.
	//  DSBCAPS_LOCDEFER - The buffer can be assigned to a hardware or software resource at play time, or when IDirectSoundBuffer8::AcquireResources is called.
	//  DSBCAPS_LOCHARDWARE - The buffer uses hardware mixing.
	//  DSBCAPS_LOCSOFTWARE - The buffer is in software memory and uses software mixing.
	//  DSBCAPS_MUTE3DATMAXDISTANCE - The sound is reduced to silence at the maximum distance. The buffer will stop playing when the
	//        maximum distance is exceeded, so that processor time is not wasted. Applies only to software buffers.
	// DSBCAPS_PRIMARYBUFFER - The buffer is a primary buffer.
	// DSBCAPS_STATIC - The buffer is in on-board hardware memory.
	// DSBCAPS_STICKYFOCUS - The buffer has sticky focus. If the user switches to another application not using DirectSound, the buffer
	//        is still audible. However, if the user switches to another DirectSound application, the buffer is muted.
	// DSBCAPS_TRUEPLAYPOSITION - Force IDirectSoundBuffer8::GetCurrentPosition to return the buffer's true play position.
	//        This flag is only valid in Windows Vista.
	// 3D Algorithms:
	//  DS3DALG_DEFAULT - DirectSound uses the default algorithm. In most cases this is DS3DALG_NO_VIRTUALIZATION.
	//        On WDM drivers, if the user has selected a surround sound speaker configuration in Control Panel,
	//        the sound is panned among the available directional speakers.
	//        Applies to software mixing only. Available on WDM or Vxd Drivers.
	//  DS3DALG_NO_VIRTUALIZATION - 3D output is mapped onto normal left and right stereo panning.
	//        At 90 degrees to the left, the sound is coming out of only the left speaker; at 90 degrees to the right,
	//        sound is coming out of only the right speaker. The vertical axis is ignored except for scaling of volume due
	//        to distance. Doppler shift and volume scaling are still applied, but the 3D filtering is not performed on this
	//        buffer. This is the most efficient software implementation, but provides no virtual 3D audio effect.
	//        When the DS3DALG_NO_VIRTUALIZATION algorithm is specified, HRTF processing will not be done.
	//        Because DS3DALG_NO_VIRTUALIZATION uses only normal stereo panning, a buffer created with this algorithm may be
	//        accelerated by a 2D hardware voice if no free 3D hardware voices are available.
	//        Applies to software mixing only. Available on WDM or Vxd Drivers.
	//  DS3DALG_HRTF_FULL - The 3D API is processed with the high quality 3D audio algorithm. This algorithm gives the highest quality 3D
	//        audio effect, but uses more CPU cycles. Applies to software mixing only. Available on Microsoft Windows 98 Second Edition and later operating systems when using WDM drivers.
	//  DS3DALG_HRTF_LIGHT - The 3D API is processed with the efficient 3D audio algorithm. This algorithm gives a good 3D audio effect,
	//        but uses fewer CPU cycles than DS3DALG_HRTF_FULL. Applies to software mixing only. Available on Windows 98 Second Edition and later operating systems when using WDM drivers.
	inline D3DPtr<IDirectSoundBuffer8> CreateBuffer(
		D3DPtr<IDirectSound8>& dsound,
		size_t size,
		int channels,
		int samples_per_sec,
		int bits_per_sample,
		DWORD flags = DSBCAPS_CTRLVOLUME, // Only add flags for what you need
		GUID _3dalg = GUID_NULL,          // Set this if you use 'DSBCAPS_CTRL3D'
		WORD format = WAVE_FORMAT_PCM,    // Standard non-compressed sound data
		DWORD block_align = 1,            // must be set if format is not 'WAVE_FORMAT_PCM'
		DWORD avr_bytes_per_sec = 1       // must be set if format is not 'WAVE_FORMAT_PCM'
	){
		// Set the wave format
		WAVEFORMATEX wf = {};
		wf.cbSize          = sizeof(wf);
		wf.wFormatTag      = format;
		wf.nChannels       = WORD(channels);
		wf.nSamplesPerSec  = samples_per_sec;
		wf.wBitsPerSample  = WORD(bits_per_sample);
		wf.nBlockAlign     = WORD(wf.wFormatTag == WAVE_FORMAT_PCM ? (wf.nChannels * wf.wBitsPerSample)/8 : block_align);
		wf.nAvgBytesPerSec = DWORD(wf.wFormatTag == WAVE_FORMAT_PCM ? (wf.nSamplesPerSec * wf.nBlockAlign) : avr_bytes_per_sec);

		// Set up the buffer description
		DSBUFFERDESC desc = {};
		desc.dwSize          = sizeof(desc);
		desc.dwBufferBytes   = DWORD(size);
		desc.dwFlags         = flags;
		desc.lpwfxFormat     = &wf;
		desc.guid3DAlgorithm = _3dalg;

		// Get a standard buffer
		D3DPtr<IDirectSoundBuffer> buf;
		Check(dsound->CreateSoundBuffer(&desc, buf.address_of(), 0));

		// Query for the IDirectSoundBuffer8 interface
		D3DPtr<IDirectSoundBuffer8> buf8;
		Check(buf->QueryInterface(IID_IDirectSoundBuffer8, (void**)buf8.address_of()));
		return buf8;
	}

	// Set the volume level for a sample (in 0->1 range)
	inline void SetVolume(D3DPtr<IDirectSoundBuffer8>& buf, float vol)
	{
		PR_ASSERT(PR_DBG_SND, 0.0f <= vol && vol <= 1.0f, "'vol' must be in the range [0,1]");
		double v = Clamp(vol, 0.0f, 1.0f) * log10(double(DSBVOLUME_MAX - DSBVOLUME_MIN));
		Check(buf->SetVolume(-long(pow(10.0, v))));
	}

	// Return the allocated size of a dsound buffer
	inline size_t GetBufferSize(D3DPtr<IDirectSoundBuffer8>& buf)
	{
		if (!buf) return 0;
		Lock lock(buf, 0, 0, DSBLOCK_ENTIREBUFFER);
		return lock.m_size0;
	}

	// Return the required buffer size for the given format at the given update rate
	inline size_t GetMinRequiredBufferSize(int updates_per_sec, int channels, int samples_per_sec, int bits_per_sample)
	{
		size_t bytes_per_sec = samples_per_sec * (bits_per_sample/2) * channels;
		return 2 * bytes_per_sec / updates_per_sec; // *2 because we only ever half fill the buffer
	}
}
