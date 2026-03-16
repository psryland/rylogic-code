// Implementation based on:
//   SPDX-License-Identifier: MIT
//   Copyright Thomas Smith 3/13/2024
//   https://github.com/b0nes164/GPUSorting

// Type ID constants for preprocessor branching.
// DXC's preprocessor doesn't support token-pasting macros in #if comparisons, so we use numeric IDs instead of the TYPE_(x) trick.
#define TYPE_ID_INT   0
#define TYPE_ID_UINT  1
#define TYPE_ID_FLOAT 2

#ifndef KEY_TYPE_ID
#define KEY_TYPE_ID TYPE_ID_INT
#endif
#ifndef PAYLOAD_TYPE_ID
#define PAYLOAD_TYPE_ID TYPE_ID_INT
#endif
#ifndef KEYS_PER_THREAD
#define KEYS_PER_THREAD 15
#endif
#ifndef PART_SIZE
#define PART_SIZE 7680U
#endif
#ifndef TOTAL_SHARED_MEM
#define TOTAL_SHARED_MEM 7936U
#endif
#ifndef SHOULD_ASCEND
#define SHOULD_ASCEND 1
#endif
#ifndef DUSE_16_BIT
#define DUSE_16_BIT 1
#endif
#ifndef FIXED_WAVE_SIZE
#define FIXED_WAVE_SIZE // [WaveSize(32)] //Lock RDNA to 32, we want WGP's not CU's
#endif
#ifndef SORT_PAIRS
#define SORT_PAIRS 1
#endif
#ifndef SORT_ASCENDING
#define SORT_ASCENDING 1
#endif
#ifndef DIGIT_TYPE
#define DIGIT_TYPE uint
#endif

// Constants and Defines
static const uint KeyBits              = 32U;                 // Number of bits in the key
static const uint RadixBits            = 8U;                  // Number of bits in the radix
static const uint Radix                = 1U << RadixBits;     // Number of digit bins
static const uint RadixMask            = Radix - 1;           // Mask of digit bins
static const uint HalfRadix            = Radix / 2;           // For smaller waves where bit packing is necessary
static const uint HalfRadixMask        = HalfRadix - 1;       // For smaller waves where bit packing is necessary
static const uint RadixPasses          = KeyBits / RadixBits; // The number of passes needed to read the whole key
static const uint InitDimension        = Radix * RadixPasses; // The number of threads in the Init kernel
static const uint InitPayloadDimension = 128U;                // The number of threads in the InitPayload threadblock
static const uint SweepUpDimension     = 128U;                // The number of threads in a Upsweep threadblock
static const uint ScanDimension        = 128U;                // The number of threads in a Scan threadblock
static const uint SweepDownDimension   = 512U;                // The number of threads in a Downsweep threadblock. 512 or 256 supported
static const uint MaxDispatchDimension = 65535U;              // The max value of any given dispatch dimension

// Valid values are 5, 7, 15
static const uint KeysPerThread = KEYS_PER_THREAD;

// Valid values are 1792, 2560, 3584, 3840, 7680
static const uint PartSize = PART_SIZE;

// Valid values are 4096, 7936
static const uint TotalSharedMemory = TOTAL_SHARED_MEM;

// Valid values are 'int', 'uint', 'float'
#if   KEY_TYPE_ID == TYPE_ID_INT
	typedef int key_t;
#elif KEY_TYPE_ID == TYPE_ID_UINT
	typedef uint key_t;
#elif KEY_TYPE_ID == TYPE_ID_FLOAT
	typedef float key_t;
#else
	#error "Unknown KEY_TYPE_ID"
#endif
#if   PAYLOAD_TYPE_ID == TYPE_ID_INT
	typedef int payload_t;
#elif PAYLOAD_TYPE_ID == TYPE_ID_UINT
	typedef uint payload_t;
#elif PAYLOAD_TYPE_ID == TYPE_ID_FLOAT
	typedef float payload_t;
#else
	#error "Unknown PAYLOAD_TYPE_ID"
#endif

// Valid values are 'uint' or uint16_t'
typedef DIGIT_TYPE digit_t;

// Sorting constants
cbuffer cbGpuSorting : register(b0)
{
	uint NumKeys; // Number of keys to sort
	uint RadixShift;
	uint ThreadBlocks;
	uint Partial;
};

// Buffers
RWStructuredBuffer<key_t> m_sort0 : register(u0);
RWStructuredBuffer<key_t> m_sort1 : register(u1);
RWStructuredBuffer<payload_t> m_payload0 : register(u2);
RWStructuredBuffer<payload_t> m_payload1 : register(u3);
RWStructuredBuffer<uint> m_global_histogram : register(u4); // buffer holding device level offsets for each binning pass
RWStructuredBuffer<uint> m_pass_histogram : register(u5);   // buffer used to store reduced sums of partition tiles

// Group shared memory
groupshared uint gs_sweep_down[TotalSharedMemory]; // Shared memory for DigitBinningPass and SweepDown kernels
groupshared uint gs_sweep_up[Radix * 2]; // Shared memory for SweepUp
groupshared uint gs_scan[ScanDimension]; // Shared memory for the scan

struct KeyStruct
{
	uint k[KeysPerThread];
};
struct OffsetStruct
{
	digit_t o[KeysPerThread];
};
struct DigitStruct
{
	digit_t d[KeysPerThread];
};

// Convert the key type to 'uint'
inline uint KeyToUInt(key_t x)
{
	// Radix Tricks by Michael Herf: http://stereopsis.com/radix.html
	#if	KEY_TYPE_ID == TYPE_ID_UINT
		return x;
	#elif KEY_TYPE_ID == TYPE_ID_INT
		return asuint(x ^ 0x80000000);
	#elif KEY_TYPE_ID == TYPE_ID_FLOAT
		uint mask = -((int) (asuint(x) >> 31)) | 0x80000000;
		return asuint(x) ^ mask;
	#else
		#error "Unsupported key type"
		return 0;
	#endif
}
inline key_t ToKey(uint x)
{
	#if	KEY_TYPE_ID == TYPE_ID_UINT
		return x;
	#elif KEY_TYPE_ID == TYPE_ID_INT
		return asint(x ^ 0x80000000);
	#elif KEY_TYPE_ID == TYPE_ID_FLOAT
		uint mask = ((x >> 31) - 1) | 0x80000000;
		return asfloat(x ^ mask);
	#else
		#error "Unsupported key type"
		return 0;
	#endif
}
inline uint PayloadToUInt(payload_t x)
{
	// Radix Tricks by Michael Herf: http://stereopsis.com/radix.html
	#if	PAYLOAD_TYPE_ID == TYPE_ID_UINT
		return x;
	#elif PAYLOAD_TYPE_ID == TYPE_ID_INT
		return asuint(x ^ 0x80000000);
	#elif PAYLOAD_TYPE_ID == TYPE_ID_FLOAT
		uint mask = -((int) (asuint(x) >> 31)) | 0x80000000;
		return asuint(x) ^ mask;
	#else
		#error "Unsupported payload type"
		return 0;
	#endif
}
inline payload_t ToPayload(uint x)
{
	#if	PAYLOAD_TYPE_ID == TYPE_ID_UINT
		return x;
	#elif PAYLOAD_TYPE_ID == TYPE_ID_INT
		return asint(x ^ 0x80000000);
	#elif PAYLOAD_TYPE_ID == TYPE_ID_FLOAT
		uint mask = ((x >> 31) - 1) | 0x80000000;
		return asfloat(x ^ mask);
	#else
		#error "Unsupported payload type"
		return 0;
	#endif
}
inline digit_t ToDigit(uint x)
{
	return digit_t(x);
}

// Get the group id (Allowing for partial dispatches)
inline uint FlattenGpId(uint3 gpid)
{
	return (Partial & 1) ?
		gpid.x + (Partial >> 1) * MaxDispatchDimension :
		gpid.x + gpid.y * MaxDispatchDimension;
}

// Get the wave index from the global thread id
inline uint GetWaveIndex(uint gtid)
{
	return gtid / WaveGetLaneCount();
}

// Read a key from the sort buffer
inline void LoadKey(inout uint key, uint device_index)
{
	key = KeyToUInt(m_sort0[device_index]);
}

// Write a key to the output sort buffer
inline void WriteKey(uint device_index, uint group_shared_index)
{
	m_sort1[device_index] = ToKey(gs_sweep_down[group_shared_index]);
}

// Read a payload from the payload buffer
inline void LoadPayload(inout uint payload, uint device_index)
{
	payload = PayloadToUInt(m_payload0[device_index]);
}

// Write a payload to the output payload buffer
inline void WritePayload(uint device_index, uint group_shared_index)
{
	m_payload1[device_index] = ToPayload(gs_sweep_down[group_shared_index]);
}

// Index for reverse iteration of the keys
inline uint DescendingIndex(uint device_index)
{
	return NumKeys - device_index - 1;
}

// Wave histogram size
inline uint WaveHistsSizeWGE16()
{
	return SweepDownDimension / WaveGetLaneCount() * Radix;
}
inline uint WaveHistsSizeWLT16()
{
	return TotalSharedMemory;
}

// Get the offset into the global histogram
inline uint GlobalHistOffset()
{
	return RadixShift << 5;
}

// Extract a Radix size digit from a key
inline uint ExtractDigit(uint key, uint shift)
{
	return key >> shift & RadixMask; // todo, check presedence
}
inline uint ExtractDigit(uint key)
{
	return ExtractDigit(key, RadixShift);
}

// Count the number of digits for each radix bin
inline void HistogramDigitCounts(uint gtid, uint gpid)
{
	// histogram, 64 threads to a histogram
	const uint hist_offset = gtid / 64 * Radix;
	const uint partition_end = gpid == ThreadBlocks - 1 ? NumKeys : (gpid + 1) * PartSize;
	for (uint i = gtid + gpid * PartSize; i < partition_end; i += SweepUpDimension)
		InterlockedAdd(gs_sweep_up[ExtractDigit(KeyToUInt(m_sort0[i])) + hist_offset], 1);
}

// Reduce and pass to tile histogram
inline void ReduceWriteDigitCounts(uint gtid, uint gpid)
{
	for (uint i = gtid; i < Radix; i += SweepUpDimension)
	{
		gs_sweep_up[i] += gs_sweep_up[i + Radix];
		m_pass_histogram[i * ThreadBlocks + gpid] = gs_sweep_up[i];
		gs_sweep_up[i] += WavePrefixSum(gs_sweep_up[i]);
	}
}

// Exclusive scan over digit counts, then atomically add to global hist
inline void GlobalHistExclusiveScanWGE16(uint gtid)
{
	GroupMemoryBarrierWithGroupSync();
		
	if (gtid < (Radix / WaveGetLaneCount()))
		gs_sweep_up[(gtid + 1) * WaveGetLaneCount() - 1] += WavePrefixSum(gs_sweep_up[(gtid + 1) * WaveGetLaneCount() - 1]);
	
	GroupMemoryBarrierWithGroupSync();
		
	// Atomically add to global histogram
	const uint global_histogram_offset = GlobalHistOffset();
	const uint lane_mask = WaveGetLaneCount() - 1;
	const uint circular_lane_shift = WaveGetLaneIndex() + 1 & lane_mask;
	for (uint i = gtid; i < Radix; i += SweepUpDimension)
	{
		const uint index = circular_lane_shift + (i & ~lane_mask);
		const uint value =
			(WaveGetLaneIndex() != lane_mask ? gs_sweep_up[i] : 0) +
			(i >= WaveGetLaneCount() ? WaveReadLaneAt(gs_sweep_up[i - 1], 0) : 0);

		InterlockedAdd(m_global_histogram[index + global_histogram_offset], value);
	}
}
inline void GlobalHistExclusiveScanWLT16(uint gtid)
{
	const uint global_histogram_offset = GlobalHistOffset();
	if (gtid < WaveGetLaneCount())
	{
		const uint circular_lane_shift = WaveGetLaneIndex() + 1 & WaveGetLaneCount() - 1;
		InterlockedAdd(m_global_histogram[circular_lane_shift + global_histogram_offset], circular_lane_shift ? gs_sweep_up[gtid] : 0);
	}
	
	GroupMemoryBarrierWithGroupSync();
		
	const uint log_lane = countbits(WaveGetLaneCount() - 1);
	uint offset = log_lane;
	uint j = WaveGetLaneCount();

	for (; j < (Radix >> 1); j <<= log_lane)
	{
		if (gtid < (Radix >> offset))
		{
			gs_sweep_up[((gtid + 1) << offset) - 1] +=
				WavePrefixSum(gs_sweep_up[((gtid + 1) << offset) - 1]);
		}
		
		GroupMemoryBarrierWithGroupSync();
			
		for (uint i = gtid + j; i < Radix; i += SweepUpDimension)
		{
			if ((i & ((j << log_lane) - 1)) >= j)
			{
				if (i < (j << log_lane))
				{
					const uint value = WaveReadLaneAt(gs_sweep_up[((i >> offset) << offset) - 1], 0) + ((i & (j - 1)) ? gs_sweep_up[i - 1] : 0);
					InterlockedAdd(m_global_histogram[i + global_histogram_offset], value);
				}
				else
				{
					if ((i + 1) & (j - 1))
					{
						gs_sweep_up[i] += WaveReadLaneAt(gs_sweep_up[((i >> offset) << offset) - 1], 0);
					}
				}
			}
		}
		offset += log_lane;
	}
	
	GroupMemoryBarrierWithGroupSync();
		
	// If Radix is not a power of lanecount
	for (uint i = gtid + j; i < Radix; i += SweepUpDimension)
	{
		const uint value = WaveReadLaneAt(gs_sweep_up[((i >> offset) << offset) - 1], 0) + ((i & (j - 1)) ? gs_sweep_up[i - 1] : 0);
		InterlockedAdd(m_global_histogram[i + global_histogram_offset], value);
	}
}

// Scan
inline void ExclusiveThreadBlockScanFullWGE16(uint gtid, uint lane_mask, uint circular_lane_shift, uint part_end, uint device_offset, inout uint reduction)
{
	for (uint i = gtid; i < part_end; i += ScanDimension)
	{
		gs_scan[gtid] = m_pass_histogram[i + device_offset];
		gs_scan[gtid] += WavePrefixSum(gs_scan[gtid]);

		GroupMemoryBarrierWithGroupSync();

		if (gtid < ScanDimension / WaveGetLaneCount())
		{
			gs_scan[(gtid + 1) * WaveGetLaneCount() - 1] += WavePrefixSum(gs_scan[(gtid + 1) * WaveGetLaneCount() - 1]);
		}

		GroupMemoryBarrierWithGroupSync();

		m_pass_histogram[circular_lane_shift + (i & ~lane_mask) + device_offset] =
			(WaveGetLaneIndex() != lane_mask ? gs_scan[gtid.x] : 0) +
			(gtid.x >= WaveGetLaneCount() ? WaveReadLaneAt(gs_scan[gtid.x - 1], 0) : 0) +
			reduction;

		reduction += gs_scan[ScanDimension - 1];

		GroupMemoryBarrierWithGroupSync();
	}
}
inline void ExclusiveThreadBlockScanPartialWGE16(uint gtid, uint lane_mask, uint circular_lane_shift, uint part_end, uint device_offset, uint reduction)
{
	uint i = gtid + part_end;
	if (i < ThreadBlocks)
		gs_scan[gtid] = m_pass_histogram[device_offset + i];

	gs_scan[gtid] += WavePrefixSum(gs_scan[gtid]);

	GroupMemoryBarrierWithGroupSync();
			
	if (gtid < ScanDimension / WaveGetLaneCount())
	{
		gs_scan[(gtid + 1) * WaveGetLaneCount() - 1] += WavePrefixSum(gs_scan[(gtid + 1) * WaveGetLaneCount() - 1]);
	}

	GroupMemoryBarrierWithGroupSync();
		
	const uint index = circular_lane_shift + (i & ~lane_mask);
	if (index < ThreadBlocks)
	{
		m_pass_histogram[index + device_offset] =
			(WaveGetLaneIndex() != lane_mask ? gs_scan[gtid.x] : 0) +
			(gtid.x >= WaveGetLaneCount() ? gs_scan[(gtid.x & ~lane_mask) - 1] : 0) +
			reduction;
	}
}
inline void ExclusiveThreadBlockScanFullWLT16(uint gtid, uint partitions, uint device_offset, uint log_lane, uint circular_lane_shift, inout uint reduction)
{
	for (uint k = 0; k < partitions; ++k)
	{
		gs_scan[gtid] = m_pass_histogram[gtid + k * ScanDimension + device_offset];
		gs_scan[gtid] += WavePrefixSum(gs_scan[gtid]);

		GroupMemoryBarrierWithGroupSync();

		if (gtid < WaveGetLaneCount())
		{
			m_pass_histogram[circular_lane_shift + k * ScanDimension + device_offset] = (circular_lane_shift ? gs_scan[gtid] : 0) + reduction;
		}
			
		uint offset = log_lane;
		uint j = WaveGetLaneCount();
		for (; j < (ScanDimension >> 1); j <<= log_lane)
		{
			if (gtid < (ScanDimension >> offset))
			{
				gs_scan[((gtid + 1) << offset) - 1] += WavePrefixSum(gs_scan[((gtid + 1) << offset) - 1]);
			}
			
			GroupMemoryBarrierWithGroupSync();
			
			if ((gtid & ((j << log_lane) - 1)) >= j)
			{
				if (gtid < (j << log_lane))
				{
					m_pass_histogram[gtid + k * ScanDimension + device_offset] =
						WaveReadLaneAt(gs_scan[((gtid >> offset) << offset) - 1], 0) + ((gtid & (j - 1)) ? gs_scan[gtid - 1] : 0) + reduction;
				}
				else
				{
					if ((gtid + 1) & (j - 1))
					{
						gs_scan[gtid] += WaveReadLaneAt(gs_scan[((gtid >> offset) << offset) - 1], 0);
					}
				}
			}
			offset += log_lane;
		}
		
		GroupMemoryBarrierWithGroupSync();
		
		// If ScanDimension is not a power of lanecount
		for (uint i = gtid + j; i < ScanDimension; i += ScanDimension)
		{
			m_pass_histogram[i + k * ScanDimension + device_offset] =
				WaveReadLaneAt(gs_scan[((i >> offset) << offset) - 1], 0) + ((i & (j - 1)) ? gs_scan[i - 1] : 0) + reduction;
		}
			
		reduction +=
			WaveReadLaneAt(gs_scan[ScanDimension - 1], 0) +
			WaveReadLaneAt(gs_scan[(((ScanDimension - 1) >> offset) << offset) - 1], 0);

		GroupMemoryBarrierWithGroupSync();
	}
}
inline void ExclusiveThreadBlockScanParitalWLT16(uint gtid, uint partitions, uint device_offset, uint log_lane, uint circular_lane_shift, uint reduction)
{
	const uint final_part_size = ThreadBlocks - partitions * ScanDimension;
	if (gtid < final_part_size)
	{
		gs_scan[gtid] = m_pass_histogram[gtid + partitions * ScanDimension + device_offset];
		gs_scan[gtid] += WavePrefixSum(gs_scan[gtid]);
	}

	GroupMemoryBarrierWithGroupSync();

	if (gtid < WaveGetLaneCount() && circular_lane_shift < final_part_size)
	{
		m_pass_histogram[circular_lane_shift + partitions * ScanDimension + device_offset] =
			(circular_lane_shift ? gs_scan[gtid] : 0) + reduction;
	}
		
	uint offset = log_lane;
	for (uint j = WaveGetLaneCount(); j < final_part_size; j <<= log_lane)
	{
		if (gtid < (final_part_size >> offset))
		{
			gs_scan[((gtid + 1) << offset) - 1] +=
				WavePrefixSum(gs_scan[((gtid + 1) << offset) - 1]);
		}

		GroupMemoryBarrierWithGroupSync();

		if ((gtid & ((j << log_lane) - 1)) >= j && gtid < final_part_size)
		{
			if (gtid < (j << log_lane))
			{
				m_pass_histogram[gtid + partitions * ScanDimension + device_offset] =
					WaveReadLaneAt(gs_scan[((gtid >> offset) << offset) - 1], 0) +
					((gtid & (j - 1)) ? gs_scan[gtid - 1] : 0) + reduction;
			}
			else
			{
				if ((gtid + 1) & (j - 1))
				{
					gs_scan[gtid] +=
						WaveReadLaneAt(gs_scan[((gtid >> offset) << offset) - 1], 0);
				}
			}
		}
		offset += log_lane;
	}
}
inline void ExclusiveThreadBlockScanWGE16(uint gtid, uint gpid)
{
	const uint lane_mask = WaveGetLaneCount() - 1;
	const uint circular_lane_shift = WaveGetLaneIndex() + 1 & lane_mask;
	const uint partions_end = ThreadBlocks / ScanDimension * ScanDimension;
	const uint device_offset = gpid * ThreadBlocks;
	
	uint reduction = 0;
	ExclusiveThreadBlockScanFullWGE16(gtid, lane_mask, circular_lane_shift, partions_end, device_offset, reduction);
	ExclusiveThreadBlockScanPartialWGE16(gtid, lane_mask, circular_lane_shift, partions_end, device_offset, reduction);
}
inline void ExclusiveThreadBlockScanWLT16(uint gtid, uint gpid)
{
	const uint log_lane = countbits(WaveGetLaneCount() - 1);
	const uint circular_lane_shift = WaveGetLaneIndex() + 1 & WaveGetLaneCount() - 1;
	const uint partitions = ThreadBlocks / ScanDimension;
	const uint device_offset = gpid * ThreadBlocks;
	
	uint reduction = 0;
	ExclusiveThreadBlockScanFullWLT16(gtid, partitions, device_offset, log_lane, circular_lane_shift, reduction);	
	ExclusiveThreadBlockScanParitalWLT16(gtid, partitions, device_offset, log_lane, circular_lane_shift, reduction);
}

// Sweep Down
inline void ClearWaveHistogarms(uint gtid)
{
	const uint hists_end = WaveGetLaneCount() >= 16 ? WaveHistsSizeWGE16() : WaveHistsSizeWLT16();
	for (uint i = gtid; i < hists_end; i += SweepDownDimension)
		gs_sweep_down[i] = 0;
}
inline uint SerialIterations()
{
	// If the size of a wave is too small, we do not have enough space in shared memory to assign
	// a histogram to each wave, so instead, some operations are peformed serially.
	return (SweepDownDimension / WaveGetLaneCount() + 31) >> 5;
}
inline void LoadDummyKey(inout uint key)
{
	key = 0xffffffff;
}

inline uint DeviceOffsetWGE16(uint gtid, uint part_index)
{
	uint SubPartSizeWGE16 = KeysPerThread * WaveGetLaneCount();
	uint SharedOffsetWGE16 = WaveGetLaneIndex() +
		GetWaveIndex(gtid) * SubPartSizeWGE16;

	return SharedOffsetWGE16 + part_index * PartSize;
}
inline uint DeviceOffsetWLT16(uint gtid, uint part_index, uint serial_iterations)
{
	uint SubPartSizeWLT16 = KeysPerThread * WaveGetLaneCount() * serial_iterations;
	uint SharedOffsetWLT16 = WaveGetLaneIndex() +
		(GetWaveIndex(gtid) / serial_iterations * SubPartSizeWLT16) +
		(GetWaveIndex(gtid) % serial_iterations * WaveGetLaneCount());

	return SharedOffsetWLT16 + part_index * PartSize;
}

inline KeyStruct LoadKeysWGE16(uint gtid, uint part_index)
{
	KeyStruct keys;
	
	[unroll]
	for (uint i = 0, t = DeviceOffsetWGE16(gtid, part_index); i < KeysPerThread; ++i, t += WaveGetLaneCount())
		LoadKey(keys.k[i], t);

	return keys;
}
inline KeyStruct LoadKeysWLT16(uint gtid, uint part_index, uint serial_iterations)
{
	KeyStruct keys;

	[unroll]
	for (uint i = 0, t = DeviceOffsetWLT16(gtid, part_index, serial_iterations); i < KeysPerThread; ++i, t += WaveGetLaneCount() * serial_iterations)
		LoadKey(keys.k[i], t);

	return keys;
}
inline KeyStruct LoadKeysPartialWGE16(uint gtid, uint part_index)
{
	KeyStruct keys;

	[unroll]
	for (uint i = 0, t = DeviceOffsetWGE16(gtid, part_index); i < KeysPerThread; ++i, t += WaveGetLaneCount())
	{
		if (t < NumKeys)
			LoadKey(keys.k[i], t);
		else
			LoadDummyKey(keys.k[i]);
	}

	return keys;
}
inline KeyStruct LoadKeysPartialWLT16(uint gtid, uint part_index, uint serial_iterations)
{
	KeyStruct keys;

	[unroll]
	for (uint i = 0, t = DeviceOffsetWLT16(gtid, part_index, serial_iterations); i < KeysPerThread; ++i, t += WaveGetLaneCount() * serial_iterations)
	{
		if (t < NumKeys)
			LoadKey(keys.k[i], t);
		else
			LoadDummyKey(keys.k[i]);
	}

	return keys;
}
inline KeyStruct LoadPayloadsWGE16(uint gtid, uint part_index)
{
	KeyStruct payloads;
	
	[unroll]
	for (uint i = 0, t = DeviceOffsetWGE16(gtid, part_index); i < KeysPerThread; ++i, t += WaveGetLaneCount())
		LoadPayload(payloads.k[i], t);
	
	return payloads;
}
inline KeyStruct LoadPayloadsWLT16(uint gtid, uint part_index, uint serial_iterations)
{
	KeyStruct payloads;
	
	[unroll]
	for (uint i = 0, t = DeviceOffsetWLT16(gtid, part_index, serial_iterations); i < KeysPerThread; ++i, t += WaveGetLaneCount() * serial_iterations)
		LoadPayload(payloads.k[i], t);
	
	return payloads;

}
inline KeyStruct LoadPayloadsPartialWGE16(uint gtid, uint part_index)
{
	KeyStruct payloads;
	
	[unroll]
	for (uint i = 0, t = DeviceOffsetWGE16(gtid, part_index); i < KeysPerThread; ++i, t += WaveGetLaneCount())
	{
		if (t < NumKeys)
			LoadPayload(payloads.k[i], t);
	}
	
	return payloads;

}
inline KeyStruct LoadPayloadsPartialWLT16(uint gtid, uint part_index, uint serial_iterations)
{
	KeyStruct payloads;
	
	[unroll]
	for (uint i = 0, t = DeviceOffsetWLT16(gtid, part_index, serial_iterations); i < KeysPerThread; ++i, t += WaveGetLaneCount() * serial_iterations)
	{
		if (t < NumKeys)
			LoadPayload(payloads.k[i], t);
	}
	
	return payloads;
}

inline void WarpLevelMultiSplitWGE16(uint key, uint wave_parts, inout uint4 wave_flags)
{
	[unroll]
	for (uint k = 0; k < RadixBits; ++k)
	{
		const bool t = key >> (k + RadixShift) & 1;
		const uint4 ballot = WaveActiveBallot(t);
		for (uint wave_part = 0; wave_part < wave_parts; ++wave_part)
			wave_flags[wave_part] &= (t ? 0 : 0xffffffff) ^ ballot[wave_part];
	}
}
inline void WarpLevelMultiSplitWLT16(uint key, inout uint wave_flags)
{
	[unroll]
	for (uint k = 0; k < RadixBits; ++k)
	{
		const bool t = key >> (k + RadixShift) & 1;
		wave_flags &= (t ? 0 : 0xffffffff) ^ (uint) WaveActiveBallot(t);
	}
}

inline uint FindLowestRankPeer(uint4 wave_flags, uint wave_parts)
{
	uint lowest_rank_peer = 0;
	for (uint wave_part = 0; wave_part < wave_parts; ++wave_part)
	{
		uint fbl = firstbitlow(wave_flags[wave_part]);
		if (fbl == 0xffffffff)
			lowest_rank_peer += 32;
		else
			return lowest_rank_peer + fbl;
	}
	return 0; //will never happen
}
inline void CountPeerBits(inout uint peer_bits, inout uint total_bits, uint4 wave_flags, uint wave_parts)
{
	for (uint wave_part = 0; wave_part < wave_parts; ++wave_part)
	{
		if (WaveGetLaneIndex() >= wave_part * 32)
		{
			const uint ltMask = WaveGetLaneIndex() >= (wave_part + 1) * 32 ? 0xffffffff : (1U << (WaveGetLaneIndex() & 31)) - 1;
			peer_bits += countbits(wave_flags[wave_part] & ltMask);
		}
		total_bits += countbits(wave_flags[wave_part]);
	}
}
inline uint CountPeerBitsWLT16(uint wave_flags, uint ltMask)
{
	return countbits(wave_flags & ltMask);
}
inline uint ExtractPackedIndex(uint key)
{
	return key >> (RadixShift + 1) & HalfRadixMask;
}
inline uint ExtractPackedShift(uint key)
{
	return (key >> RadixShift & 1) ? 16 : 0;
}
inline uint ExtractPackedValue(uint packed, uint key)
{
	return packed >> ExtractPackedShift(key) & 0xffff;
}

inline OffsetStruct RankKeysWGE16(uint gtid, KeyStruct keys)
{
	OffsetStruct offsets;
	
	const uint wave_parts = (WaveGetLaneCount() + 31) / 32;

	[unroll]
	for (uint i = 0; i < KeysPerThread; ++i)
	{
		uint4 wave_flags = (WaveGetLaneCount() & 31) ? (1U << WaveGetLaneCount()) - 1 : 0xffffffff;
		WarpLevelMultiSplitWGE16(keys.k[i], wave_parts, wave_flags);
		
		const uint index = ExtractDigit(keys.k[i]) + (GetWaveIndex(gtid.x) * Radix);
		const uint lowest_rank_peer = FindLowestRankPeer(wave_flags, wave_parts);
		
		uint peer_bits = 0;
		uint total_bits = 0;
		CountPeerBits(peer_bits, total_bits, wave_flags, wave_parts);
		
		uint pre_increment_value;
		if (peer_bits == 0)
			InterlockedAdd(gs_sweep_down[index], total_bits, pre_increment_value);
		offsets.o[i] = ToDigit(WaveReadLaneAt(pre_increment_value, lowest_rank_peer) + peer_bits);
	}

	return offsets;
}
inline OffsetStruct RankKeysWLT16(uint gtid, KeyStruct keys, uint serial_iterations)
{
	OffsetStruct offsets;
	const uint ltMask = (1U << WaveGetLaneIndex()) - 1;
	
	[unroll]
	for (uint i = 0; i < KeysPerThread; ++i)
	{
		uint wave_flags = (1U << WaveGetLaneCount()) - 1;
		WarpLevelMultiSplitWLT16(keys.k[i], wave_flags);
		
		const uint index = ExtractPackedIndex(keys.k[i]) + (GetWaveIndex(gtid.x) / serial_iterations * HalfRadix);
		const uint peer_bits = CountPeerBitsWLT16(wave_flags, ltMask);
		for (uint k = 0; k < serial_iterations; ++k)
		{
			if (GetWaveIndex(gtid.x) % serial_iterations == k)
				offsets.o[i] = ToDigit(ExtractPackedValue(gs_sweep_down[index], keys.k[i]) + peer_bits);
			
			GroupMemoryBarrierWithGroupSync();
			if (GetWaveIndex(gtid.x) % serial_iterations == k && peer_bits == 0)
			{
				InterlockedAdd(gs_sweep_down[index],
					countbits(wave_flags) << ExtractPackedShift(keys.k[i]));
			}
			GroupMemoryBarrierWithGroupSync();
		}
	}
	
	return offsets;
}

inline uint WaveHistInclusiveScanCircularShiftWGE16(uint gtid)
{
	uint histogram_reduction = gs_sweep_down[gtid];
	for (uint i = gtid + Radix; i < WaveHistsSizeWGE16(); i += Radix)
	{
		histogram_reduction += gs_sweep_down[i];
		gs_sweep_down[i] = histogram_reduction - gs_sweep_down[i];
	}
	return histogram_reduction;
}
inline uint WaveHistInclusiveScanCircularShiftWLT16(uint gtid)
{
	uint histogram_reduction = gs_sweep_down[gtid];
	for (uint i = gtid + HalfRadix; i < WaveHistsSizeWLT16(); i += HalfRadix)
	{
		histogram_reduction += gs_sweep_down[i];
		gs_sweep_down[i] = histogram_reduction - gs_sweep_down[i];
	}
	return histogram_reduction;
}
inline void WaveHistReductionExclusiveScanWGE16(uint gtid, uint histogram_reduction)
{
	if (gtid < Radix)
	{
		const uint lane_mask = WaveGetLaneCount() - 1;
		gs_sweep_down[((WaveGetLaneIndex() + 1) & lane_mask) + (gtid & ~lane_mask)] = histogram_reduction;
	}

	GroupMemoryBarrierWithGroupSync();

	if (gtid < Radix / WaveGetLaneCount())
		gs_sweep_down[gtid * WaveGetLaneCount()] = WavePrefixSum(gs_sweep_down[gtid * WaveGetLaneCount()]);

	GroupMemoryBarrierWithGroupSync();

	if (gtid < Radix && WaveGetLaneIndex())
		gs_sweep_down[gtid] += WaveReadLaneAt(gs_sweep_down[gtid - 1], 1);
}
inline void WaveHistReductionExclusiveScanWLT16(uint gtid)
{
	// inclusive/exclusive prefix sum up the histograms,
	// use a blelloch scan for in place packed exclusive
	uint shift = 1;
	for (uint j0 = Radix >> 2; j0 > 0; j0 >>= 1)
	{
		GroupMemoryBarrierWithGroupSync();
		if (gtid < j0)
		{
			gs_sweep_down[((((gtid << 1) + 2) << shift) - 1) >> 1] +=
				gs_sweep_down[((((gtid << 1) + 1) << shift) - 1) >> 1] & 0xffff0000;
		}
		shift++;
	}
	
	GroupMemoryBarrierWithGroupSync();
				
	if (gtid == 0)
		gs_sweep_down[HalfRadix - 1] &= 0xffff;

	for (uint j1 = 1; j1 < Radix >> 1; j1 <<= 1)
	{
		--shift;
		GroupMemoryBarrierWithGroupSync();
		if (gtid < j1)
		{
			const uint t = ((((gtid << 1) + 1) << shift) - 1) >> 1;
			const uint t2 = ((((gtid << 1) + 2) << shift) - 1) >> 1;
			const uint t3 = gs_sweep_down[t];
			gs_sweep_down[t] = (gs_sweep_down[t] & 0xffff) | (gs_sweep_down[t2] & 0xffff0000);
			gs_sweep_down[t2] += t3 & 0xffff0000;
		}
	}

	GroupMemoryBarrierWithGroupSync();

	if (gtid < HalfRadix)
	{
		const uint t = gs_sweep_down[gtid];
		gs_sweep_down[gtid] = (t >> 16) + (t << 16) + (t & 0xffff0000);
	}
}

inline void UpdateOffsetsWGE16(uint gtid, inout OffsetStruct offsets, KeyStruct keys)
{
	if (gtid >= WaveGetLaneCount())
	{
		const uint t = GetWaveIndex(gtid) * Radix;
		[unroll]
		for (uint i = 0; i < KeysPerThread; ++i)
		{
			const uint t2 = ExtractDigit(keys.k[i]);
			offsets.o[i] += ToDigit(gs_sweep_down[t2 + t] + gs_sweep_down[t2]);
		}
	}
	else
	{
		[unroll]
		for (uint i = 0; i < KeysPerThread; ++i)
			offsets.o[i] += ToDigit(gs_sweep_down[ExtractDigit(keys.k[i])]);
	}
}
inline void UpdateOffsetsWLT16(uint gtid, uint serial_iterations, inout OffsetStruct offsets, KeyStruct keys)
{
	if (gtid >= WaveGetLaneCount() * serial_iterations)
	{
		const uint t = GetWaveIndex(gtid) / serial_iterations * HalfRadix;
		[unroll]
		for (uint i = 0; i < KeysPerThread; ++i)
		{
			const uint t2 = ExtractPackedIndex(keys.k[i]);
			offsets.o[i] += ToDigit(ExtractPackedValue(gs_sweep_down[t2 + t] + gs_sweep_down[t2], keys.k[i]));
		}
	}
	else
	{
		[unroll]
		for (uint i = 0; i < KeysPerThread; ++i)
			offsets.o[i] += ToDigit(ExtractPackedValue(gs_sweep_down[ExtractPackedIndex(keys.k[i])], keys.k[i]));
	}
}

inline void LoadThreadBlockReductions(uint gtid, uint gpid, uint exclusive_histogram_reduction)
{
	if (gtid < Radix)
	{
		gs_sweep_down[gtid + PartSize] =
			m_global_histogram[gtid + GlobalHistOffset()] +
			m_pass_histogram[gtid * ThreadBlocks + gpid] - exclusive_histogram_reduction;
	}
}

inline void ScatterKeysShared(OffsetStruct offsets, KeyStruct keys)
{
	[unroll]
	for (uint i = 0; i < KeysPerThread; ++i)
		gs_sweep_down[offsets.o[i]] = keys.k[i];
}
inline void ScatterPayloadsShared(OffsetStruct offsets, KeyStruct payloads)
{
	ScatterKeysShared(offsets, payloads);
}
inline void ScatterPairsKeyPhaseAscending(uint gtid, inout DigitStruct digits)
{
	[unroll]
	for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
	{
		digits.d[i] = ToDigit(ExtractDigit(gs_sweep_down[t]));
		WriteKey(gs_sweep_down[uint(digits.d[i]) + PartSize] + t, t);
	}
}
inline void ScatterPairsKeyPhaseDescending(uint gtid, inout DigitStruct digits)
{
	if (RadixShift == 24)
	{
		[unroll]
		for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
		{
			digits.d[i] = ToDigit(ExtractDigit(gs_sweep_down[t]));
			WriteKey(DescendingIndex(gs_sweep_down[uint(digits.d[i]) + PartSize]
			+t),
			t);
		}
	}
	else
	{
		ScatterPairsKeyPhaseAscending(gtid, digits);
	}
}
inline void ScatterPairsKeyPhaseAscendingPartial(uint gtid, uint final_part_size, inout DigitStruct digits)
{
	[unroll]
	for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
	{
		if (t < final_part_size)
		{
			digits.d[i] = ToDigit(ExtractDigit(gs_sweep_down[t]));
			WriteKey(gs_sweep_down[uint(digits.d[i]) + PartSize] + t, t);
		}
	}
}
inline void ScatterPairsKeyPhaseDescendingPartial(uint gtid, uint final_part_size, inout DigitStruct digits)
{
	if (RadixShift == 24)
	{
		[unroll]
		for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
		{
			if (t < final_part_size)
			{
				digits.d[i] = ToDigit(ExtractDigit(gs_sweep_down[t]));
				WriteKey(DescendingIndex(gs_sweep_down[uint(digits.d[i]) + PartSize] + t), t);
			}
		}
	}
	else
	{
		ScatterPairsKeyPhaseAscendingPartial(gtid, final_part_size, digits);
	}
}
inline void ScatterPayloadsAscending(uint gtid, DigitStruct digits)
{
	[unroll]
	for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
		WritePayload(gs_sweep_down[uint(digits.d[i]) + PartSize] + t, t);
}
inline void ScatterPayloadsDescending(uint gtid, DigitStruct digits)
{
	if (RadixShift == 24)
	{
		[unroll]
		for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
			WritePayload(DescendingIndex(gs_sweep_down[uint(digits.d[i]) + PartSize] + t), t);
	}
	else
	{
		ScatterPayloadsAscending(gtid, digits);
	}
}
inline void ScatterPayloadsAscendingPartial(uint gtid, uint final_part_size, DigitStruct digits)
{
	[unroll]
	for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
	{
		if (t < final_part_size)
			WritePayload(gs_sweep_down[uint(digits.d[i]) + PartSize] + t, t);
	}
}
inline void ScatterPayloadsDescendingPartial(uint gtid, uint final_part_size, DigitStruct digits)
{
	if (RadixShift == 24)
	{
		[unroll]
		for (uint i = 0, t = gtid; i < KeysPerThread; ++i, t += SweepDownDimension)
		{
			if (t < final_part_size)
				WritePayload(DescendingIndex(gs_sweep_down[uint(digits.d[i]) + PartSize] + t), t);
		}
	}
	else
	{
		ScatterPayloadsAscendingPartial(gtid, final_part_size, digits);
	}
}
inline void ScatterKeysOnlyDeviceAscending(uint gtid)
{
	for (uint i = gtid; i < PartSize; i += SweepDownDimension)
		WriteKey(gs_sweep_down[ExtractDigit(gs_sweep_down[i]) + PartSize] + i, i);
}
inline void ScatterKeysOnlyDeviceDescending(uint gtid)
{
	if (RadixShift == 24)
	{
		for (uint i = gtid; i < PartSize; i += SweepDownDimension)
			WriteKey(DescendingIndex(gs_sweep_down[ExtractDigit(gs_sweep_down[i]) + PartSize] + i), i);
	}
	else
	{
		ScatterKeysOnlyDeviceAscending(gtid);
	}
}
inline void ScatterKeysOnlyDevicePartialAscending(uint gtid, uint final_part_size)
{
	for (uint i = gtid; i < PartSize; i += SweepDownDimension)
	{
		if (i < final_part_size)
			WriteKey(gs_sweep_down[ExtractDigit(gs_sweep_down[i]) + PartSize] + i, i);
	}
}
inline void ScatterKeysOnlyDevicePartialDescending(uint gtid, uint final_part_size)
{
	if (RadixShift == 24)
	{
		for (uint i = gtid; i < PartSize; i += SweepDownDimension)
		{
			if (i < final_part_size)
				WriteKey(DescendingIndex(gs_sweep_down[ExtractDigit(gs_sweep_down[i]) + PartSize] + i), i);
		}
	}
	else
	{
		ScatterKeysOnlyDevicePartialAscending(gtid, final_part_size);
	}
}
inline void ScatterDevice(uint gtid, uint part_index, OffsetStruct offsets)
{
	#if SORT_PAIRS
	{
		DigitStruct digits;
	
		#if SORT_ASCENDING
		ScatterPairsKeyPhaseAscending(gtid, digits);
		#else
		ScatterPairsKeyPhaseDescending(gtid, digits);
		#endif

		GroupMemoryBarrierWithGroupSync();
	
		KeyStruct payloads;
		if (WaveGetLaneCount() >= 16)
			payloads = LoadPayloadsWGE16(gtid, part_index);
		else
			payloads = LoadPayloadsWLT16(gtid, part_index, SerialIterations());
		
		ScatterPayloadsShared(offsets, payloads);

		GroupMemoryBarrierWithGroupSync();
	
		#if SORT_ASCENDING
		ScatterPayloadsAscending(gtid, digits);
		#else
		ScatterPayloadsDescending(gtid, digits);
		#endif
	}
	#else
	{
		#if SORT_ASCENDING
		ScatterKeysOnlyDeviceAscending(gtid);
		#else
		ScatterKeysOnlyDeviceDescending(gtid);
		#endif
	}
	#endif
}
inline void ScatterDevicePartial(uint gtid, uint part_index, OffsetStruct offsets)
{
	#if SORT_PAIRS
	{
		DigitStruct digits;
		const uint final_part_size = NumKeys - part_index * PartSize;
		#if SORT_ASCENDING
		ScatterPairsKeyPhaseAscendingPartial(gtid, final_part_size, digits);
		#else
		ScatterPairsKeyPhaseDescendingPartial(gtid, final_part_size, digits);
		#endif

		GroupMemoryBarrierWithGroupSync();
	
		KeyStruct payloads;
		if (WaveGetLaneCount() >= 16)
			payloads = LoadPayloadsPartialWGE16(gtid, part_index);
		else
			payloads = LoadPayloadsPartialWLT16(gtid, part_index, SerialIterations());

		ScatterPayloadsShared(offsets, payloads);

		GroupMemoryBarrierWithGroupSync();
	
		#if SORT_ASCENDING
		ScatterPayloadsAscendingPartial(gtid, final_part_size, digits);
		#else
		ScatterPayloadsDescendingPartial(gtid, final_part_size, digits);
		#endif
	}
	#else
	{
		const uint final_part_size = NumKeys - part_index * PartSize;
		#if SORT_ASCENDING
		ScatterKeysOnlyDevicePartialAscending(gtid, final_part_size);
		#else
		ScatterKeysOnlyDevicePartialDescending(gtid, final_part_size);
		#endif
	}
	#endif
}

// INIT KERNEL
[numthreads(InitDimension, 1, 1)]
void InitRadixSort(int3 dtid : SV_DispatchThreadID)
{
	// Clear the global histogram, as we will be adding to it atomically
	m_global_histogram[dtid.x] = 0;
}

// INIT PAYLOAD KERNEL
[numthreads(InitPayloadDimension, 1, 1)]
void InitPayload(int3 dtid : SV_DispatchThreadID, uint3 gpid : SV_GroupID)
{
	const uint group = FlattenGpId(gpid);
	const uint partition_end = group == ThreadBlocks - 1 ? NumKeys : (group + 1) * PartSize;
	for (uint i = dtid.x + group * PartSize; i < partition_end; i += InitPayloadDimension)
		m_payload0[i] = payload_t(i);
}

// SWEEP UP KERNEL
[numthreads(SweepUpDimension, 1, 1)]
void SweepUp(uint3 gtid : SV_GroupThreadID, uint3 gpid : SV_GroupID)
{
	// Clear shared memory
	const uint end = Radix * 2;
	for (uint i = gtid.x; i < end; i += SweepUpDimension)
		gs_sweep_up[i] = 0;
	
	GroupMemoryBarrierWithGroupSync();

	HistogramDigitCounts(gtid.x, FlattenGpId(gpid));
	
	GroupMemoryBarrierWithGroupSync();
	
	ReduceWriteDigitCounts(gtid.x, FlattenGpId(gpid));
	
	if (WaveGetLaneCount() >= 16)
		GlobalHistExclusiveScanWGE16(gtid.x);
	
	if (WaveGetLaneCount() < 16)
		GlobalHistExclusiveScanWLT16(gtid.x);
}

// SCAN KERNEL
[numthreads(ScanDimension, 1, 1)]
void Scan(uint3 gtid : SV_GroupThreadID, uint3 gpid : SV_GroupID)
{
	// Scan does not need flattening of gids
	if (WaveGetLaneCount() >= 16)
		ExclusiveThreadBlockScanWGE16(gtid.x, gpid.x);

	if (WaveGetLaneCount() < 16)
		ExclusiveThreadBlockScanWLT16(gtid.x, gpid.x);
}

// SWEEP DOWN KERNEL
FIXED_WAVE_SIZE
[numthreads(SweepDownDimension, 1, 1)]
void SweepDown(uint3 gtid : SV_GroupThreadID, uint3 gpid : SV_GroupID)
{
	KeyStruct keys;
	OffsetStruct offsets;

	ClearWaveHistogarms(gtid.x);

	if (FlattenGpId(gpid) < ThreadBlocks - 1)
	{
		if (WaveGetLaneCount() >= 16)
			keys = LoadKeysWGE16(gtid.x, FlattenGpId(gpid));
		
		if (WaveGetLaneCount() < 16)
			keys = LoadKeysWLT16(gtid.x, FlattenGpId(gpid), SerialIterations());
	}
	if (FlattenGpId(gpid) == ThreadBlocks - 1)
	{
		if (WaveGetLaneCount() >= 16)
			keys = LoadKeysPartialWGE16(gtid.x, FlattenGpId(gpid));
		
		if (WaveGetLaneCount() < 16)
			keys = LoadKeysPartialWLT16(gtid.x, FlattenGpId(gpid), SerialIterations());
	}

	uint exclusive_histogram_reduction;

	if (WaveGetLaneCount() >= 16)
	{
		GroupMemoryBarrierWithGroupSync();

		offsets = RankKeysWGE16(gtid.x, keys);
		
		GroupMemoryBarrierWithGroupSync();
		
		uint histogram_reduction;
		if (gtid.x < Radix)
		{
			histogram_reduction = WaveHistInclusiveScanCircularShiftWGE16(gtid.x);
			histogram_reduction += WavePrefixSum(histogram_reduction); //take advantage of barrier to begin scan
		}
		
		GroupMemoryBarrierWithGroupSync();

		WaveHistReductionExclusiveScanWGE16(gtid.x, histogram_reduction);

		GroupMemoryBarrierWithGroupSync();

		UpdateOffsetsWGE16(gtid.x, offsets, keys);
		if (gtid.x < Radix)
			exclusive_histogram_reduction = gs_sweep_down[gtid.x]; //take advantage of barrier to grab value

		GroupMemoryBarrierWithGroupSync();
	}
	if (WaveGetLaneCount() < 16)
	{
		offsets = RankKeysWLT16(gtid.x, keys, SerialIterations());
			
		if (gtid.x < HalfRadix)
		{
			uint histogram_reduction = WaveHistInclusiveScanCircularShiftWLT16(gtid.x);
			gs_sweep_down[gtid.x] = histogram_reduction + (histogram_reduction << 16); //take advantage of barrier to begin scan
		}
			
		WaveHistReductionExclusiveScanWLT16(gtid.x);

		GroupMemoryBarrierWithGroupSync();
			
		UpdateOffsetsWLT16(gtid.x, SerialIterations(), offsets, keys);
		if (gtid.x < Radix) //take advantage of barrier to grab value
			exclusive_histogram_reduction = gs_sweep_down[gtid.x >> 1] >> ((gtid.x & 1) ? 16 : 0) & 0xffff;

		GroupMemoryBarrierWithGroupSync();
	}
	
	ScatterKeysShared(offsets, keys);
	LoadThreadBlockReductions(gtid.x, FlattenGpId(gpid), exclusive_histogram_reduction);
	GroupMemoryBarrierWithGroupSync();
	
	if (FlattenGpId(gpid) < ThreadBlocks - 1)
		ScatterDevice(gtid.x, FlattenGpId(gpid), offsets);
		
	if (FlattenGpId(gpid) == ThreadBlocks - 1)
		ScatterDevicePartial(gtid.x, FlattenGpId(gpid), offsets);
}
