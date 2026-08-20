//*****************************************
// Zip Compression
//	Rylogic 2019
//*****************************************
#pragma once

namespace pr::storage
{
	namespace zip
	{
		// Compression levels
		enum class ELevel
		{
			Min = 0,
			L0 = Min,
			L1,
			L2,
			L3,
			L4,
			L5,
			L6,
			L7,
			L8,
			L9,
			L10,
			L11,
			Max = L11
		};

		// Use this function to get the minimum size of the
		// 'compressed' buffer that should be passed to 'Compress()'
		size_t GetCompressionBufferSize(size_t data_length);

		// Use this function to read the decompressed size of the data
		size_t GetDecompressedSize(void const* compressed_data);

		// Use this function to read the actual size of the compressed data
		size_t GetCompressedSize(void const* compressed_data);

		// Compress/Decompress data 
		void Compress(void const* data, size_t data_length, void* compressed, ELevel level = ELevel::L4);
		void Decompress(void const* data, size_t data_length, void* decompressed, size_t decompressed_length);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::storage
{
	PRUnitTest(ZipTests, Quick)
	{
		std::string input = "This is a string to be compressed compressed compressed, oh, This is a string to be compressed compressed compressed";
		std::vector<unsigned char> buf(zip::GetCompressionBufferSize(input.size()));
		zip::Compress(input.data(), input.size(), buf.data(), zip::ELevel::L11);
		buf.resize(zip::GetCompressedSize(buf.data()));

		PR_CHECK(buf.size() < input.size(), true);

		std::string output(zip::GetDecompressedSize(buf.data()), '\0');
		zip::Decompress(buf.data(), buf.size(), output.data(), output.size());

		PR_CHECK(input, output);
	}
}
#endif