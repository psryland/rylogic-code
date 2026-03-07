//***************************************************************************************************
// Audio
//  Copyright (c) Rylogic Ltd 2017
//***************************************************************************************************
#pragma once
#include "pr/audio/forward.h"

namespace pr::audio
{
	// Settings for constructing the audio manager
	struct Settings
	{
		// Set the device id using SystemConfig (or leave as null for the default device)
		wchar_t const* m_device_id;
		uint32_t m_channels;
		uint32_t m_sample_rate;

		Settings()
			:m_device_id(nullptr)
			,m_channels(XAUDIO2_DEFAULT_CHANNELS)
			,m_sample_rate(XAUDIO2_DEFAULT_SAMPLERATE)
		{}
	};

	// Audio manager state variables
	struct State
	{
		Settings m_settings;
		pr::RefPtr<IXAudio2> m_xaudio;
		audio::VoicePtr<IXAudio2MasteringVoice> m_master;

		State(Settings const& settings);
		~State();
	};

	// The "Renderer" of Audio
	class AudioManager :State
	{
		// Notes:
		//  - A voice wraps a buffer of audio data. There are source voices, submix voices, and mastering voices
		//    arranged like this:
		//       source_voice1 -->  submix_voice --> mastering voice --> hardware
		//       source_voice2 --------^                 ^
		//       source_voice3 --------------------------+
		//  - Source voices to *not* copy the audio data, user code must keep the audio data in scope until
		//    indicated by the 'IXAudio2VoiceCallback::OnBufferEnd' callback.

		std::recursive_mutex m_mutex;
		AllocationsTracker<Sound> m_dbg_mem_snd;

		// Delete methods that models/model buffers call to clean themselves up
		friend struct Sound;
		void Delete(Sound* sound);

	public:

		explicit AudioManager(Settings const& settings = Settings());
		~AudioManager();

		// Synchronise access to XAudio2 interfaces
		class Lock
		{
			AudioManager& m_mgr;
			std::lock_guard<std::recursive_mutex> m_lock;

		public:

			Lock(AudioManager& mgr)
				: m_mgr(mgr)
				, m_lock(mgr.m_mutex)
			{}
		};

		// Load and play an audio file synchronously.
		// If the audio contains loops, 'loop_count' indicates how many times to loop
		void PlaySynchronous(std::filesystem::path const& filepath, int loop_count = 0) const;

		// Create a sound instance
		SoundPtr CreateSound();

		// Raised when a sound is deleted
		EventHandler<Sound&, EmptyArgs const&> SoundDeleted;
	};
}