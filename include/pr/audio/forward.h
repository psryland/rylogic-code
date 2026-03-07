//***************************************************************************************************
// Audio
//  Copyright (c) Rylogic Ltd 2017
//***************************************************************************************************
#pragma once

#ifndef _WIN32_WINNT 
#define _WIN32_WINNT _WIN32_WINNT_WIN8
#elif _WIN32_WINNT < _WIN32_WINNT_WIN8 
#error "_WIN32_WINNT >= _WIN32_WINNT_WIN8 required"
#endif

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <regex>
#include <stdexcept>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cassert>

#include <sdkddkver.h>
#include <windows.h>
#include <mmreg.h>
#include <xaudio2.h>

#include "pr/common/min_max_fix.h"
#include "pr/common/assert.h"
#include "pr/common/fmt.h"
#include "pr/common/log.h"
#include "pr/common/hresult.h"
#include "pr/common/refptr.h"
#include "pr/common/event_handler.h"
#include "pr/common/static_callback.h"
#include "pr/common/cast.h"
#include "pr/common/scope.h"
#include "pr/common/flags_enum.h"
#include "pr/common/allocator.h"
#include "pr/math/math.h"
#include "pr/common/bit_fields.h"
#include "pr/algorithm/algorithm.h"
#include "pr/storage/xml.h"
#include "pr/win32/win32.h"

#define PR_DBG_AUDIO PR_DBG

namespace pr::audio
{
	struct State;
	struct Settings;
	struct Sound;
	class AudioManager;
	using SoundPtr = pr::RefPtr<Sound>;
	using ReportErrorCB = StaticCB<void, wchar_t const*>;
	template <typename T> using Allocator = pr::aligned_alloc<T>;
	template <typename T> using alloc_traits = std::allocator_traits<Allocator<T>>;

	// Sample rates (in samples/sec)
	struct ESampleRate
	{
		// Not strongly typed because other sample rate values are also valid
		enum type_t
		{
			_11025 = 11025,
			_22050 = 22050,
			_44100 = 44100,
			_88200 = 88200,
		};

		int value;
		constexpr ESampleRate(int rate)
			:value(rate)
		{}
		constexpr ESampleRate(type_t rate)
			:value(rate)
		{}
		constexpr operator int() const
		{
			return value;
		}
	};

	// Known wave formats (WAVE form wFormatTag IDs)
	enum class EWaveFormat :WORD
	{
		UNKNOWN                    = WAVE_FORMAT_UNKNOWN,                    // Microsoft Corporation
		PCM                        = WAVE_FORMAT_PCM,                        //
		ADPCM                      = WAVE_FORMAT_ADPCM,                      // Microsoft Corporation
		IEEE_FLOAT                 = WAVE_FORMAT_IEEE_FLOAT,                 // Microsoft Corporation
		VSELP                      = WAVE_FORMAT_VSELP,                      // Compaq Computer Corp.
		IBM_CVSD                   = WAVE_FORMAT_IBM_CVSD,                   // IBM Corporation
		ALAW                       = WAVE_FORMAT_ALAW,                       // Microsoft Corporation
		MULAW                      = WAVE_FORMAT_MULAW,                      // Microsoft Corporation
		DTS                        = WAVE_FORMAT_DTS,                        // Microsoft Corporation
		DRM                        = WAVE_FORMAT_DRM,                        // Microsoft Corporation
		WMAVOICE9                  = WAVE_FORMAT_WMAVOICE9,                  // Microsoft Corporation
		WMAVOICE10                 = WAVE_FORMAT_WMAVOICE10,                 // Microsoft Corporation
		OKI_ADPCM                  = WAVE_FORMAT_OKI_ADPCM,                  // OKI
		DVI_ADPCM                  = WAVE_FORMAT_DVI_ADPCM,                  // Intel Corporation
		IMA_ADPCM                  = WAVE_FORMAT_IMA_ADPCM,                  //  Intel Corporation
		MEDIASPACE_ADPCM           = WAVE_FORMAT_MEDIASPACE_ADPCM,           // Videologic
		SIERRA_ADPCM               = WAVE_FORMAT_SIERRA_ADPCM,               // Sierra Semiconductor Corp
		G723_ADPCM                 = WAVE_FORMAT_G723_ADPCM,                 // Antex Electronics Corporation
		DIGISTD                    = WAVE_FORMAT_DIGISTD,                    // DSP Solutions, Inc.
		DIGIFIX                    = WAVE_FORMAT_DIGIFIX,                    // DSP Solutions, Inc.
		DIALOGIC_OKI_ADPCM         = WAVE_FORMAT_DIALOGIC_OKI_ADPCM,         // Dialogic Corporation
		MEDIAVISION_ADPCM          = WAVE_FORMAT_MEDIAVISION_ADPCM,          // Media Vision, Inc.
		CU_CODEC                   = WAVE_FORMAT_CU_CODEC,                   // Hewlett-Packard Company
		HP_DYN_VOICE               = WAVE_FORMAT_HP_DYN_VOICE,               // Hewlett-Packard Company
		YAMAHA_ADPCM               = WAVE_FORMAT_YAMAHA_ADPCM,               // Yamaha Corporation of America
		SONARC                     = WAVE_FORMAT_SONARC,                     // Speech Compression
		DSPGROUP_TRUESPEECH        = WAVE_FORMAT_DSPGROUP_TRUESPEECH,        // DSP Group, Inc
		ECHOSC1                    = WAVE_FORMAT_ECHOSC1,                    // Echo Speech Corporation
		AUDIOFILE_AF36             = WAVE_FORMAT_AUDIOFILE_AF36,             // Virtual Music, Inc.
		APTX                       = WAVE_FORMAT_APTX,                       // Audio Processing Technology
		AUDIOFILE_AF10             = WAVE_FORMAT_AUDIOFILE_AF10,             // Virtual Music, Inc.
		PROSODY_1612               = WAVE_FORMAT_PROSODY_1612,               // Aculab plc
		LRC                        = WAVE_FORMAT_LRC,                        // Merging Technologies S.A.
		DOLBY_AC2                  = WAVE_FORMAT_DOLBY_AC2,                  // Dolby Laboratories
		GSM610                     = WAVE_FORMAT_GSM610,                     // Microsoft Corporation
		MSNAUDIO                   = WAVE_FORMAT_MSNAUDIO,                   // Microsoft Corporation
		ANTEX_ADPCME               = WAVE_FORMAT_ANTEX_ADPCME,               // Antex Electronics Corporation
		CONTROL_RES_VQLPC          = WAVE_FORMAT_CONTROL_RES_VQLPC,          // Control Resources Limited
		DIGIREAL                   = WAVE_FORMAT_DIGIREAL,                   // DSP Solutions, Inc.
		DIGIADPCM                  = WAVE_FORMAT_DIGIADPCM,                  // DSP Solutions, Inc.
		CONTROL_RES_CR10           = WAVE_FORMAT_CONTROL_RES_CR10,           // Control Resources Limited
		NMS_VBXADPCM               = WAVE_FORMAT_NMS_VBXADPCM,               // Natural MicroSystems
		CS_IMAADPCM                = WAVE_FORMAT_CS_IMAADPCM,                // Crystal Semiconductor IMA ADPCM
		ECHOSC3                    = WAVE_FORMAT_ECHOSC3,                    // Echo Speech Corporation
		ROCKWELL_ADPCM             = WAVE_FORMAT_ROCKWELL_ADPCM,             // Rockwell International
		ROCKWELL_DIGITALK          = WAVE_FORMAT_ROCKWELL_DIGITALK,          // Rockwell International
		XEBEC                      = WAVE_FORMAT_XEBEC,                      // Xebec Multimedia Solutions Limited
		G721_ADPCM                 = WAVE_FORMAT_G721_ADPCM,                 // Antex Electronics Corporation
		G728_CELP                  = WAVE_FORMAT_G728_CELP,                  // Antex Electronics Corporation
		MSG723                     = WAVE_FORMAT_MSG723,                     // Microsoft Corporation
		INTEL_G723_1               = WAVE_FORMAT_INTEL_G723_1,               // Intel Corp.
		INTEL_G729                 = WAVE_FORMAT_INTEL_G729,                 // Intel Corp.
		SHARP_G726                 = WAVE_FORMAT_SHARP_G726,                 // Sharp
		MPEG                       = WAVE_FORMAT_MPEG,                       // Microsoft Corporation
		RT24                       = WAVE_FORMAT_RT24,                       // InSoft, Inc.
		PAC                        = WAVE_FORMAT_PAC,                        // InSoft, Inc.
		MPEGLAYER3                 = WAVE_FORMAT_MPEGLAYER3,                 // ISO/MPEG Layer3 Format Tag
		LUCENT_G723                = WAVE_FORMAT_LUCENT_G723,                // Lucent Technologies
		CIRRUS                     = WAVE_FORMAT_CIRRUS,                     // Cirrus Logic
		ESPCM                      = WAVE_FORMAT_ESPCM,                      // ESS Technology
		VOXWARE                    = WAVE_FORMAT_VOXWARE,                    // Voxware Inc
		CANOPUS_ATRAC              = WAVE_FORMAT_CANOPUS_ATRAC,              // Canopus, co., Ltd.
		G726_ADPCM                 = WAVE_FORMAT_G726_ADPCM,                 // APICOM
		G722_ADPCM                 = WAVE_FORMAT_G722_ADPCM,                 // APICOM
		DSAT                       = WAVE_FORMAT_DSAT,                       // Microsoft Corporation
		DSAT_DISPLAY               = WAVE_FORMAT_DSAT_DISPLAY,               // Microsoft Corporation
		VOXWARE_BYTE_ALIGNED       = WAVE_FORMAT_VOXWARE_BYTE_ALIGNED,       // Voxware Inc
		VOXWARE_AC8                = WAVE_FORMAT_VOXWARE_AC8,                // Voxware Inc
		VOXWARE_AC10               = WAVE_FORMAT_VOXWARE_AC10,               // Voxware Inc
		VOXWARE_AC16               = WAVE_FORMAT_VOXWARE_AC16,               // Voxware Inc
		VOXWARE_AC20               = WAVE_FORMAT_VOXWARE_AC20,               // Voxware Inc
		VOXWARE_RT24               = WAVE_FORMAT_VOXWARE_RT24,               // Voxware Inc
		VOXWARE_RT29               = WAVE_FORMAT_VOXWARE_RT29,               // Voxware Inc
		VOXWARE_RT29HW             = WAVE_FORMAT_VOXWARE_RT29HW,             // Voxware Inc
		VOXWARE_VR12               = WAVE_FORMAT_VOXWARE_VR12,               // Voxware Inc
		VOXWARE_VR18               = WAVE_FORMAT_VOXWARE_VR18,               // Voxware Inc
		VOXWARE_TQ40               = WAVE_FORMAT_VOXWARE_TQ40,               // Voxware Inc
		VOXWARE_SC3                = WAVE_FORMAT_VOXWARE_SC3,                // Voxware Inc
		VOXWARE_SC3_1              = WAVE_FORMAT_VOXWARE_SC3_1,              // Voxware Inc
		SOFTSOUND                  = WAVE_FORMAT_SOFTSOUND,                  // Softsound, Ltd.
		VOXWARE_TQ60               = WAVE_FORMAT_VOXWARE_TQ60,               // Voxware Inc
		MSRT24                     = WAVE_FORMAT_MSRT24,                     // Microsoft Corporation
		G729A                      = WAVE_FORMAT_G729A,                      // AT&T Labs, Inc.
		MVI_MVI2                   = WAVE_FORMAT_MVI_MVI2,                   // Motion Pixels
		DF_G726                    = WAVE_FORMAT_DF_G726,                    // DataFusion Systems (Pty) (Ltd)
		DF_GSM610                  = WAVE_FORMAT_DF_GSM610,                  // DataFusion Systems (Pty) (Ltd)
		ISIAUDIO                   = WAVE_FORMAT_ISIAUDIO,                   // Iterated Systems, Inc.
		ONLIVE                     = WAVE_FORMAT_ONLIVE,                     // OnLive! Technologies, Inc.
		MULTITUDE_FT_SX20          = WAVE_FORMAT_MULTITUDE_FT_SX20,          // Multitude Inc.
		INFOCOM_ITS_G721_ADPCM     = WAVE_FORMAT_INFOCOM_ITS_G721_ADPCM,     // Infocom
		CONVEDIA_G729              = WAVE_FORMAT_CONVEDIA_G729,              // Convedia Corp.
		CONGRUENCY                 = WAVE_FORMAT_CONGRUENCY,                 // Congruency Inc.
		SBC24                      = WAVE_FORMAT_SBC24,                      // Siemens Business Communications Sys
		DOLBY_AC3_SPDIF            = WAVE_FORMAT_DOLBY_AC3_SPDIF,            // Sonic Foundry
		MEDIASONIC_G723            = WAVE_FORMAT_MEDIASONIC_G723,            // MediaSonic
		PROSODY_8KBPS              = WAVE_FORMAT_PROSODY_8KBPS,              // Aculab plc
		ZYXEL_ADPCM                = WAVE_FORMAT_ZYXEL_ADPCM,                // ZyXEL Communications, Inc.
		PHILIPS_LPCBB              = WAVE_FORMAT_PHILIPS_LPCBB,              // Philips Speech Processing
		PACKED                     = WAVE_FORMAT_PACKED,                     // Studer Professional Audio AG
		MALDEN_PHONYTALK           = WAVE_FORMAT_MALDEN_PHONYTALK,           // Malden Electronics Ltd.
		RACAL_RECORDER_GSM         = WAVE_FORMAT_RACAL_RECORDER_GSM,         // Racal recorders
		RACAL_RECORDER_G720_A      = WAVE_FORMAT_RACAL_RECORDER_G720_A,      // Racal recorders
		RACAL_RECORDER_G723_1      = WAVE_FORMAT_RACAL_RECORDER_G723_1,      // Racal recorders
		RACAL_RECORDER_TETRA_ACELP = WAVE_FORMAT_RACAL_RECORDER_TETRA_ACELP, // Racal recorders
		NEC_AAC                    = WAVE_FORMAT_NEC_AAC,                    // NEC Corp.
		RAW_AAC1                   = WAVE_FORMAT_RAW_AAC1,                   // For Raw AAC, with format block AudioSpecificConfig() (as defined by MPEG-4), that follows WAVEFORMATEX
		RHETOREX_ADPCM             = WAVE_FORMAT_RHETOREX_ADPCM,             // Rhetorex Inc.
		IRAT                       = WAVE_FORMAT_IRAT,                       // BeCubed Software Inc.
		VIVO_G723                  = WAVE_FORMAT_VIVO_G723,                  // Vivo Software
		VIVO_SIREN                 = WAVE_FORMAT_VIVO_SIREN,                 // Vivo Software
		PHILIPS_CELP               = WAVE_FORMAT_PHILIPS_CELP,               // Philips Speech Processing
		PHILIPS_GRUNDIG            = WAVE_FORMAT_PHILIPS_GRUNDIG,            // Philips Speech Processing
		DIGITAL_G723               = WAVE_FORMAT_DIGITAL_G723,               // Digital Equipment Corporation
		SANYO_LD_ADPCM             = WAVE_FORMAT_SANYO_LD_ADPCM,             // Sanyo Electric Co., Ltd.
		SIPROLAB_ACEPLNET          = WAVE_FORMAT_SIPROLAB_ACEPLNET,          // Sipro Lab Telecom Inc.
		SIPROLAB_ACELP4800         = WAVE_FORMAT_SIPROLAB_ACELP4800,         // Sipro Lab Telecom Inc.
		SIPROLAB_ACELP8V3          = WAVE_FORMAT_SIPROLAB_ACELP8V3,          // Sipro Lab Telecom Inc.
		SIPROLAB_G729              = WAVE_FORMAT_SIPROLAB_G729,              // Sipro Lab Telecom Inc.
		SIPROLAB_G729A             = WAVE_FORMAT_SIPROLAB_G729A,             // Sipro Lab Telecom Inc.
		SIPROLAB_KELVIN            = WAVE_FORMAT_SIPROLAB_KELVIN,            // Sipro Lab Telecom Inc.
		VOICEAGE_AMR               = WAVE_FORMAT_VOICEAGE_AMR,               // VoiceAge Corp.
		G726ADPCM                  = WAVE_FORMAT_G726ADPCM,                  // Dictaphone Corporation
		DICTAPHONE_CELP68          = WAVE_FORMAT_DICTAPHONE_CELP68,          // Dictaphone Corporation
		DICTAPHONE_CELP54          = WAVE_FORMAT_DICTAPHONE_CELP54,          // Dictaphone Corporation
		QUALCOMM_PUREVOICE         = WAVE_FORMAT_QUALCOMM_PUREVOICE,         // Qualcomm, Inc.
		QUALCOMM_HALFRATE          = WAVE_FORMAT_QUALCOMM_HALFRATE,          // Qualcomm, Inc.
		TUBGSM                     = WAVE_FORMAT_TUBGSM,                     // Ring Zero Systems, Inc.
		MSAUDIO1                   = WAVE_FORMAT_MSAUDIO1,                   // Microsoft Corporation
		WMAUDIO2                   = WAVE_FORMAT_WMAUDIO2,                   // Microsoft Corporation
		WMAUDIO3                   = WAVE_FORMAT_WMAUDIO3,                   // Microsoft Corporation
		WMAUDIO_LOSSLESS           = WAVE_FORMAT_WMAUDIO_LOSSLESS,           // Microsoft Corporation
		WMASPDIF                   = WAVE_FORMAT_WMASPDIF,                   // Microsoft Corporation
		XMA2                       = 0x0166,                                 // Microsoft Corporation XBox One
		UNISYS_NAP_ADPCM           = WAVE_FORMAT_UNISYS_NAP_ADPCM,           // Unisys Corp.
		UNISYS_NAP_ULAW            = WAVE_FORMAT_UNISYS_NAP_ULAW,            // Unisys Corp.
		UNISYS_NAP_ALAW            = WAVE_FORMAT_UNISYS_NAP_ALAW,            // Unisys Corp.
		UNISYS_NAP_16K             = WAVE_FORMAT_UNISYS_NAP_16K,             // Unisys Corp.
		SYCOM_ACM_SYC008           = WAVE_FORMAT_SYCOM_ACM_SYC008,           // SyCom Technologies
		SYCOM_ACM_SYC701_G726L     = WAVE_FORMAT_SYCOM_ACM_SYC701_G726L,     // SyCom Technologies
		SYCOM_ACM_SYC701_CELP54    = WAVE_FORMAT_SYCOM_ACM_SYC701_CELP54,    // SyCom Technologies
		SYCOM_ACM_SYC701_CELP68    = WAVE_FORMAT_SYCOM_ACM_SYC701_CELP68,    // SyCom Technologies
		KNOWLEDGE_ADVENTURE_ADPCM  = WAVE_FORMAT_KNOWLEDGE_ADVENTURE_ADPCM,  // Knowledge Adventure, Inc.
		FRAUNHOFER_IIS_MPEG2_AAC   = WAVE_FORMAT_FRAUNHOFER_IIS_MPEG2_AAC,   // Fraunhofer IIS
		DTS_DS                     = WAVE_FORMAT_DTS_DS,                     // Digital Theatre Systems, Inc.
		CREATIVE_ADPCM             = WAVE_FORMAT_CREATIVE_ADPCM,             // Creative Labs, Inc
		CREATIVE_FASTSPEECH8       = WAVE_FORMAT_CREATIVE_FASTSPEECH8,       // Creative Labs, Inc
		CREATIVE_FASTSPEECH10      = WAVE_FORMAT_CREATIVE_FASTSPEECH10,      // Creative Labs, Inc
		UHER_ADPCM                 = WAVE_FORMAT_UHER_ADPCM,                 // UHER informatic GmbH
		ULEAD_DV_AUDIO             = WAVE_FORMAT_ULEAD_DV_AUDIO,             // Ulead Systems, Inc.
		ULEAD_DV_AUDIO_1           = WAVE_FORMAT_ULEAD_DV_AUDIO_1,           // Ulead Systems, Inc.
		QUARTERDECK                = WAVE_FORMAT_QUARTERDECK,                // Quarterdeck Corporation
		ILINK_VC                   = WAVE_FORMAT_ILINK_VC,                   // I-link Worldwide
		RAW_SPORT                  = WAVE_FORMAT_RAW_SPORT,                  // Aureal Semiconductor
		ESST_AC3                   = WAVE_FORMAT_ESST_AC3,                   // ESS Technology, Inc.
		GENERIC_PASSTHRU           = WAVE_FORMAT_GENERIC_PASSTHRU,           //
		IPI_HSX                    = WAVE_FORMAT_IPI_HSX,                    // Interactive Products, Inc.
		IPI_RPELP                  = WAVE_FORMAT_IPI_RPELP,                  // Interactive Products, Inc.
		CS2                        = WAVE_FORMAT_CS2,                        // Consistent Software
		SONY_SCX                   = WAVE_FORMAT_SONY_SCX,                   // Sony Corp.
		SONY_SCY                   = WAVE_FORMAT_SONY_SCY,                   // Sony Corp.
		SONY_ATRAC3                = WAVE_FORMAT_SONY_ATRAC3,                // Sony Corp.
		SONY_SPC                   = WAVE_FORMAT_SONY_SPC,                   // Sony Corp.
		TELUM_AUDIO                = WAVE_FORMAT_TELUM_AUDIO,                // Telum Inc.
		TELUM_IA_AUDIO             = WAVE_FORMAT_TELUM_IA_AUDIO,             // Telum Inc.
		NORCOM_VOICE_SYSTEMS_ADPCM = WAVE_FORMAT_NORCOM_VOICE_SYSTEMS_ADPCM, // Norcom Electronics Corp.
		FM_TOWNS_SND               = WAVE_FORMAT_FM_TOWNS_SND,               // Fujitsu Corp.
		MICRONAS                   = WAVE_FORMAT_MICRONAS,                   // Micronas Semiconductors, Inc.
		MICRONAS_CELP833           = WAVE_FORMAT_MICRONAS_CELP833,           // Micronas Semiconductors, Inc.
		BTV_DIGITAL                = WAVE_FORMAT_BTV_DIGITAL,                // Brooktree Corporation
		INTEL_MUSIC_CODER          = WAVE_FORMAT_INTEL_MUSIC_CODER,          // Intel Corp.
		INDEO_AUDIO                = WAVE_FORMAT_INDEO_AUDIO,                // Ligos
		QDESIGN_MUSIC              = WAVE_FORMAT_QDESIGN_MUSIC,              // QDesign Corporation
		ON2_VP7_AUDIO              = WAVE_FORMAT_ON2_VP7_AUDIO,              // On2 Technologies
		ON2_VP6_AUDIO              = WAVE_FORMAT_ON2_VP6_AUDIO,              // On2 Technologies
		VME_VMPCM                  = WAVE_FORMAT_VME_VMPCM,                  // AT&T Labs, Inc.
		TPC                        = WAVE_FORMAT_TPC,                        // AT&T Labs, Inc.
		LIGHTWAVE_LOSSLESS         = WAVE_FORMAT_LIGHTWAVE_LOSSLESS,         // Clearjump
		OLIGSM                     = WAVE_FORMAT_OLIGSM,                     // Ing C. Olivetti & C., S.p.A.
		OLIADPCM                   = WAVE_FORMAT_OLIADPCM,                   // Ing C. Olivetti & C., S.p.A.
		OLICELP                    = WAVE_FORMAT_OLICELP,                    // Ing C. Olivetti & C., S.p.A.
		OLISBC                     = WAVE_FORMAT_OLISBC,                     // Ing C. Olivetti & C., S.p.A.
		OLIOPR                     = WAVE_FORMAT_OLIOPR,                     // Ing C. Olivetti & C., S.p.A.
		LH_CODEC                   = WAVE_FORMAT_LH_CODEC,                   // Lernout & Hauspie
		LH_CODEC_CELP              = WAVE_FORMAT_LH_CODEC_CELP,              // Lernout & Hauspie
		LH_CODEC_SBC8              = WAVE_FORMAT_LH_CODEC_SBC8,              // Lernout & Hauspie
		LH_CODEC_SBC12             = WAVE_FORMAT_LH_CODEC_SBC12,             // Lernout & Hauspie
		LH_CODEC_SBC16             = WAVE_FORMAT_LH_CODEC_SBC16,             // Lernout & Hauspie
		NORRIS                     = WAVE_FORMAT_NORRIS,                     // Norris Communications, Inc.
		ISIAUDIO_2                 = WAVE_FORMAT_ISIAUDIO_2,                 // ISIAudio
		SOUNDSPACE_MUSICOMPRESS    = WAVE_FORMAT_SOUNDSPACE_MUSICOMPRESS,    // AT&T Labs, Inc.
		MPEG_ADTS_AAC              = WAVE_FORMAT_MPEG_ADTS_AAC,              // Microsoft Corporation
		MPEG_RAW_AAC               = WAVE_FORMAT_MPEG_RAW_AAC,               // Microsoft Corporation
		MPEG_LOAS                  = WAVE_FORMAT_MPEG_LOAS,                  // Microsoft Corporation (MPEG-4 Audio Transport Streams (LOAS/LATM)
		NOKIA_MPEG_ADTS_AAC        = WAVE_FORMAT_NOKIA_MPEG_ADTS_AAC,        // Microsoft Corporation
		NOKIA_MPEG_RAW_AAC         = WAVE_FORMAT_NOKIA_MPEG_RAW_AAC,         // Microsoft Corporation
		VODAFONE_MPEG_ADTS_AAC     = WAVE_FORMAT_VODAFONE_MPEG_ADTS_AAC,     // Microsoft Corporation
		VODAFONE_MPEG_RAW_AAC      = WAVE_FORMAT_VODAFONE_MPEG_RAW_AAC,      // Microsoft Corporation
		MPEG_HEAAC                 = WAVE_FORMAT_MPEG_HEAAC,                 // Microsoft Corporation (MPEG-2 AAC or MPEG-4 HE-AAC v1/v2 streams with any payload (ADTS, ADIF, LOAS/LATM, RAW). Format block includes MP4 AudioSpecificConfig() -- see HEAACWAVEFORMAT below
		VOXWARE_RT24_SPEECH        = WAVE_FORMAT_VOXWARE_RT24_SPEECH,        // Voxware Inc.
		SONICFOUNDRY_LOSSLESS      = WAVE_FORMAT_SONICFOUNDRY_LOSSLESS,      // Sonic Foundry
		INNINGS_TELECOM_ADPCM      = WAVE_FORMAT_INNINGS_TELECOM_ADPCM,      // Innings Telecom Inc.
		LUCENT_SX8300P             = WAVE_FORMAT_LUCENT_SX8300P,             // Lucent Technologies
		LUCENT_SX5363S             = WAVE_FORMAT_LUCENT_SX5363S,             // Lucent Technologies
		CUSEEME                    = WAVE_FORMAT_CUSEEME,                    // CUSeeMe
		NTCSOFT_ALF2CM_ACM         = WAVE_FORMAT_NTCSOFT_ALF2CM_ACM,         // NTCSoft
		DVM                        = WAVE_FORMAT_DVM,                        // FAST Multimedia AG
		DTS2                       = WAVE_FORMAT_DTS2,                       //
		MAKEAVIS                   = WAVE_FORMAT_MAKEAVIS,                   //
		DIVIO_MPEG4_AAC            = WAVE_FORMAT_DIVIO_MPEG4_AAC,            // Divio, Inc.
		NOKIA_ADAPTIVE_MULTIRATE   = WAVE_FORMAT_NOKIA_ADAPTIVE_MULTIRATE,   // Nokia
		DIVIO_G726                 = WAVE_FORMAT_DIVIO_G726,                 // Divio, Inc.
		LEAD_SPEECH                = WAVE_FORMAT_LEAD_SPEECH,                // LEAD Technologies
		LEAD_VORBIS                = WAVE_FORMAT_LEAD_VORBIS,                // LEAD Technologies
		WAVPACK_AUDIO              = WAVE_FORMAT_WAVPACK_AUDIO,              // xiph.org
		ALAC                       = WAVE_FORMAT_ALAC,                       // Apple Lossless
		OGG_VORBIS_MODE_1          = WAVE_FORMAT_OGG_VORBIS_MODE_1,          // Ogg Vorbis
		OGG_VORBIS_MODE_2          = WAVE_FORMAT_OGG_VORBIS_MODE_2,          // Ogg Vorbis
		OGG_VORBIS_MODE_3          = WAVE_FORMAT_OGG_VORBIS_MODE_3,          // Ogg Vorbis
		OGG_VORBIS_MODE_1_PLUS     = WAVE_FORMAT_OGG_VORBIS_MODE_1_PLUS,     // Ogg Vorbis
		OGG_VORBIS_MODE_2_PLUS     = WAVE_FORMAT_OGG_VORBIS_MODE_2_PLUS,     // Ogg Vorbis
		OGG_VORBIS_MODE_3_PLUS     = WAVE_FORMAT_OGG_VORBIS_MODE_3_PLUS,     // Ogg Vorbis
		THREECOM_NBX               = WAVE_FORMAT_3COM_NBX,                   // 3COM Corp.
		OPUS                       = WAVE_FORMAT_OPUS,                       // Opus
		FAAD_AAC                   = WAVE_FORMAT_FAAD_AAC,                   //
		AMR_NB                     = WAVE_FORMAT_AMR_NB,                     // AMR Narrowband
		AMR_WB                     = WAVE_FORMAT_AMR_WB,                     // AMR Wideband
		AMR_WP                     = WAVE_FORMAT_AMR_WP,                     // AMR Wideband Plus
		GSM_AMR_CBR                = WAVE_FORMAT_GSM_AMR_CBR,                // GSMA/3GPP
		GSM_AMR_VBR_SID            = WAVE_FORMAT_GSM_AMR_VBR_SID,            // GSMA/3GPP
		COMVERSE_INFOSYS_G723_1    = WAVE_FORMAT_COMVERSE_INFOSYS_G723_1,    // Comverse Infosys
		COMVERSE_INFOSYS_AVQSBC    = WAVE_FORMAT_COMVERSE_INFOSYS_AVQSBC,    // Comverse Infosys
		COMVERSE_INFOSYS_SBC       = WAVE_FORMAT_COMVERSE_INFOSYS_SBC,       // Comverse Infosys
		SYMBOL_G729_A              = WAVE_FORMAT_SYMBOL_G729_A,              // Symbol Technologies
		VOICEAGE_AMR_WB            = WAVE_FORMAT_VOICEAGE_AMR_WB,            // VoiceAge Corp.
		INGENIENT_G726             = WAVE_FORMAT_INGENIENT_G726,             // Ingenient Technologies, Inc.
		MPEG4_AAC                  = WAVE_FORMAT_MPEG4_AAC,                  // ISO/MPEG-4
		ENCORE_G726                = WAVE_FORMAT_ENCORE_G726,                // Encore Software
		ZOLL_ASAO                  = WAVE_FORMAT_ZOLL_ASAO,                  // ZOLL Medical Corp.
		SPEEX_VOICE                = WAVE_FORMAT_SPEEX_VOICE,                // xiph.org
		VIANIX_MASC                = WAVE_FORMAT_VIANIX_MASC,                // Vianix LLC
		WM9_SPECTRUM_ANALYZER      = WAVE_FORMAT_WM9_SPECTRUM_ANALYZER,      // Microsoft
		WMF_SPECTRUM_ANAYZER       = WAVE_FORMAT_WMF_SPECTRUM_ANAYZER,       // Microsoft
		GSM_610                    = WAVE_FORMAT_GSM_610,                    //
		GSM_620                    = WAVE_FORMAT_GSM_620,                    //
		GSM_660                    = WAVE_FORMAT_GSM_660,                    //
		GSM_690                    = WAVE_FORMAT_GSM_690,                    //
		GSM_ADAPTIVE_MULTIRATE_WB  = WAVE_FORMAT_GSM_ADAPTIVE_MULTIRATE_WB,  //
		POLYCOM_G722               = WAVE_FORMAT_POLYCOM_G722,               // Polycom
		POLYCOM_G728               = WAVE_FORMAT_POLYCOM_G728,               // Polycom
		POLYCOM_G729_A             = WAVE_FORMAT_POLYCOM_G729_A,             // Polycom
		POLYCOM_SIREN              = WAVE_FORMAT_POLYCOM_SIREN,              // Polycom
		GLOBAL_IP_ILBC             = WAVE_FORMAT_GLOBAL_IP_ILBC,             // Global IP
		RADIOTIME_TIME_SHIFT_RADIO = WAVE_FORMAT_RADIOTIME_TIME_SHIFT_RADIO, // RadioTime
		NICE_ACA                   = WAVE_FORMAT_NICE_ACA,                   // Nice Systems
		NICE_ADPCM                 = WAVE_FORMAT_NICE_ADPCM,                 // Nice Systems
		VOCORD_G721                = WAVE_FORMAT_VOCORD_G721,                // Vocord Telecom
		VOCORD_G726                = WAVE_FORMAT_VOCORD_G726,                // Vocord Telecom
		VOCORD_G722_1              = WAVE_FORMAT_VOCORD_G722_1,              // Vocord Telecom
		VOCORD_G728                = WAVE_FORMAT_VOCORD_G728,                // Vocord Telecom
		VOCORD_G729                = WAVE_FORMAT_VOCORD_G729,                // Vocord Telecom
		VOCORD_G729_A              = WAVE_FORMAT_VOCORD_G729_A,              // Vocord Telecom
		VOCORD_G723_1              = WAVE_FORMAT_VOCORD_G723_1,              // Vocord Telecom
		VOCORD_LBC                 = WAVE_FORMAT_VOCORD_LBC,                 // Vocord Telecom
		NICE_G728                  = WAVE_FORMAT_NICE_G728,                  // Nice Systems
		FRACE_TELECOM_G729         = WAVE_FORMAT_FRACE_TELECOM_G729,         // France Telecom
		CODIAN                     = WAVE_FORMAT_CODIAN,                     // CODIAN
		FLAC                       = WAVE_FORMAT_FLAC,                       // flac.sourceforge.net
		EXTENSIBLE                 = WAVE_FORMAT_EXTENSIBLE,                 // Microsoft
	};

	// Speaker Positions for dwChannelMask in WAVEFORMATEXTENSIBLE:
	enum class ESpeakerPosition :DWORD
	{
		FrontLeft          = SPEAKER_FRONT_LEFT,
		FrontRight         = SPEAKER_FRONT_RIGHT,
		FrontCentre        = SPEAKER_FRONT_CENTER,
		LowFrequency       = SPEAKER_LOW_FREQUENCY,
		BackLeft           = SPEAKER_BACK_LEFT,
		BackRight          = SPEAKER_BACK_RIGHT,
		FrontLeftOfCentre  = SPEAKER_FRONT_LEFT_OF_CENTER,
		FrontRightOfCentre = SPEAKER_FRONT_RIGHT_OF_CENTER,
		BackCentre         = SPEAKER_BACK_CENTER,
		SideLeft           = SPEAKER_SIDE_LEFT,
		SideRight          = SPEAKER_SIDE_RIGHT,
		TopCentre          = SPEAKER_TOP_CENTER,
		TopFrontLeft       = SPEAKER_TOP_FRONT_LEFT,
		TopFrontCentre     = SPEAKER_TOP_FRONT_CENTER,
		TopFrontRight      = SPEAKER_TOP_FRONT_RIGHT,
		TopBackLeft        = SPEAKER_TOP_BACK_LEFT,
		TopBackCentre      = SPEAKER_TOP_BACK_CENTER,
		TopBackRight       = SPEAKER_TOP_BACK_RIGHT,
		Reserved           = SPEAKER_RESERVED, // Bit mask locations reserved for future use
		All                = SPEAKER_ALL, // Used to specify that any possible permutation of speaker configurations
		_flags_enum = 0,
	};

	// Union of all of the different wave format structures
	union WaveFormatsU
	{
		WAVEFORMAT m_wf;
		WAVEFORMATEX m_wfx;
		PCMWAVEFORMAT m_pcm;
		WAVEFORMATEXTENSIBLE m_wfext;
		ADPCMWAVEFORMAT m_adpcm;
		DRMWAVEFORMAT m_drm;
		DVIADPCMWAVEFORMAT m_dviadp;
	};

	// Ownership pointer for 'IXAudio2Voice' instances
	struct DestroyVoice { void operator()(IXAudio2Voice* x) { x->DestroyVoice(); } };
	template <typename TVoice> using VoicePtr = std::unique_ptr<TVoice, DestroyVoice>;
}