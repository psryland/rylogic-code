//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Functions for loading a DDS texture.
//
// Note these functions are useful as a light-weight runtime loader for DDS files.
// For a full-featured DDS file reader, writer, and texture processing pipeline see
// the 'Texconv' sample and the 'DirectXTex' library.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// Copyright (c) Microsoft Corporation. All rights reserved.
// http://go.microsoft.com/fwlink/?LinkId=248926
// http://go.microsoft.com/fwlink/?LinkId=248929
#include "pr/view3d-12/texture/texture_loader.h"
#include <d3d9types.h>
//#include <memory>
//#include <filesystem>
//#include <algorithm>
//#include <assert.h>
//#include <stdint.h>
//#include <dxgiformat.h>

namespace pr::rdr12
{
	using ImageWithData = LoadedImageResult::ImageWithData;
	using ::pr::compute::BitsPerPixel;

	// DDS file structure definitions. See DDS.h in the 'Texconv' sample and the 'DirectXTex' library
	namespace dds
	{
		// "DDS "
		static uint32_t const Sentinal = 0x20534444;

		enum class EHeaderFlags : uint32_t
		{
			PIXELFORMAT = 0x00000001, // DDSD_PIXELFORMAT
			HEIGHT      = 0x00000002, // DDSD_HEIGHT
			WIDTH       = 0x00000004, // DDSD_WIDTH
			PITCH       = 0x00000008, // DDSD_PITCH
			CAPS        = 0x00001000, // DDSD_CAPS
			TEXTURE     = 0x00001007, // DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH | DDSD_PIXELFORMAT
			MIPMAP      = 0x00020000, // DDSD_MIPMAPCOUNT
			LINEARSIZE  = 0x00080000, // DDSD_LINEARSIZE
			VOLUME      = 0x00800000, // DDSD_DEPTH
			_flags_enum = 0,
		};
		enum class EPixelFormatFlags : uint32_t
		{
			ALPHAPIXELS = 0x00000001, // DDPF_ALPHAPIXELS
			ALPHA       = 0x00000002, // DDPF_ALPHA
			FOURCC      = 0x00000004, // DDPF_FOURCC
			PAL8        = 0x00000020, // DDPF_PALETTEINDEXED8
			RGB         = 0x00000040, // DDPF_RGB
			RGBA        = 0x00000041, // DDPF_RGB | DDPF_ALPHAPIXELS
			LUMINANCE   = 0x00020000, // DDPF_LUMINANCE
			LUMINANCEA  = 0x00020001, // DDPF_LUMINANCE | DDPF_ALPHAPIXELS
			_flags_enum = 0,
		};
		enum class ECaps : uint32_t
		{
			CUBEMAP = 0x00000008, // DDSCAPS_COMPLEX
			TEXTURE = 0x00001000, // DDSCAPS_TEXTURE
			MIPMAP  = 0x00400008, // DDSCAPS_COMPLEX | DDSCAPS_MIPMAP
			_flags_enum = 0,
		};
		enum class ECaps2 : uint32_t
		{
			CUBEMAP = 0x00000200, // DDSCAPS2_CUBEMAP
			CUBEMAP_POSITIVEX = 0x00000600, // DDSCAPS2_CUBEMAP | DDSCAPS2_CUBEMAP_POSITIVEX
			CUBEMAP_NEGATIVEX = 0x00000a00, // DDSCAPS2_CUBEMAP | DDSCAPS2_CUBEMAP_NEGATIVEX
			CUBEMAP_POSITIVEY = 0x00001200, // DDSCAPS2_CUBEMAP | DDSCAPS2_CUBEMAP_POSITIVEY
			CUBEMAP_NEGATIVEY = 0x00002200, // DDSCAPS2_CUBEMAP | DDSCAPS2_CUBEMAP_NEGATIVEY
			CUBEMAP_POSITIVEZ = 0x00004200, // DDSCAPS2_CUBEMAP | DDSCAPS2_CUBEMAP_POSITIVEZ
			CUBEMAP_NEGATIVEZ = 0x00008200, // DDSCAPS2_CUBEMAP | DDSCAPS2_CUBEMAP_NEGATIVEZ
			CUBEMAP_ALLFACES = CUBEMAP_POSITIVEX | CUBEMAP_NEGATIVEX | CUBEMAP_POSITIVEY | CUBEMAP_NEGATIVEY | CUBEMAP_POSITIVEZ | CUBEMAP_NEGATIVEZ,
			VOLUME = 0x00200000, // DDSCAPS2_VOLUME
			_flags_enum = 0,
		};

		struct PixelFormat
		{
			uint32_t    size;
			uint32_t    flags;
			uint32_t    fourCC;
			uint32_t    RGBBitCount;
			uint32_t    RBitMask;
			uint32_t    GBitMask;
			uint32_t    BBitMask;
			uint32_t    ABitMask;
		};
		struct Header
		{
			uint32_t    size;
			uint32_t    flags;
			uint32_t    height;
			uint32_t    width;
			uint32_t    pitchOrLinearSize;
			uint32_t    depth; // only if DDS_HEADER_FLAGS_VOLUME is set in flags
			uint32_t    mipMapCount;
			uint32_t    reserved1[11];
			PixelFormat ddspf;
			uint32_t    caps;
			uint32_t    caps2;
			uint32_t    caps3;
			uint32_t    caps4;
			uint32_t    reserved2;
		};
		struct HeaderDXT10
		{
			DXGI_FORMAT dxgiFormat;
			uint32_t    resourceDimension;
			uint32_t    miscFlag; // D3D10_RESOURCE_MISC_FLAG
			uint32_t    array_size;
			uint32_t    reserved;
		};

		struct Image
		{
			Header const* header;
			std::span<uint8_t const> bits;
			std::shared_ptr<uint8_t[]> data;
		};
	}

	// True if 'data' points at DDS data (probably)
	bool IsDDSData(std::span<uint8_t const> img)
	{
		return img.size() >= 4 && *reinterpret_cast<uint32_t const*>(img.data()) == dds::Sentinal;
	}

	// Convert a DDS pixel format to a DXGI format
	static DXGI_FORMAT GetDXGIFormat(dds::PixelFormat const& ddpf)
	{
		auto IsBitmask = [&ddpf](uint32_t r, uint32_t g, uint32_t b, uint32_t a)
		{
			return ddpf.RBitMask == r && ddpf.GBitMask == g && ddpf.BBitMask == b && ddpf.ABitMask == a;
		};

		if (AllSet(ddpf.flags, dds::EPixelFormatFlags::RGB))
		{
			// Note that sRGB formats are written using the "DX10" extended header
			switch (ddpf.RGBBitCount)
			{
				case 32:
				{
					// No DXGI format maps to IsBitmask(0x000000ff,0x0000ff00,0x00ff0000,0x00000000) aka D3DFMT_X8B8G8R8
					// Note that many common DDS reader/writers (including D3DX) swap the
					// the RED/BLUE masks for 10:10:10:2 formats. We assumme
					// below that the 'backwards' header mask is being used since it is most
					// likely written by D3DX. The more robust solution is to use the 'DX10'
					// header extension and specify the DXGI_FORMAT_R10G10B10A2_UNORM format directly
					if (IsBitmask(0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000))
						return DXGI_FORMAT_R8G8B8A8_UNORM;

					if (IsBitmask(0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000))
						return DXGI_FORMAT_B8G8R8A8_UNORM;

					if (IsBitmask(0x00ff0000, 0x0000ff00, 0x000000ff, 0x00000000))
						return DXGI_FORMAT_B8G8R8X8_UNORM;

					// For 'correct' writers, this should be 0x000003ff,0x000ffc00,0x3ff00000 for RGB data
					if (IsBitmask(0x3ff00000, 0x000ffc00, 0x000003ff, 0xc0000000))
						return DXGI_FORMAT_R10G10B10A2_UNORM;

					// No DXGI format maps to IsBitmask(0x000003ff,0x000ffc00,0x3ff00000,0xc0000000) aka D3DFMT_A2R10G10B10
					if (IsBitmask(0x0000ffff, 0xffff0000, 0x00000000, 0x00000000))
						return DXGI_FORMAT_R16G16_UNORM;

					// Only 32-bit color channel format in D3D9 was R32F. D3DX writes this out as a FourCC of 114
					if (IsBitmask(0xffffffff, 0x00000000, 0x00000000, 0x00000000))
						return DXGI_FORMAT_R32_FLOAT;

					break;
				}
				case 24:
				{
					// No 24bpp DXGI formats aka D3DFMT_R8G8B8
					break;
				}
				case 16:
				{
					// No DXGI format maps to IsBitmask(0x7c00,0x03e0,0x001f,0x0000) aka D3DFMT_X1R5G5B5
					// No DXGI format maps to IsBitmask(0x0f00,0x00f0,0x000f,0x0000) aka D3DFMT_X4R4G4B4
					// No 3:3:2, 3:3:2:8, or palette DXGI formats aka D3DFMT_A8R3G3B2, D3DFMT_R3G3B2, D3DFMT_P8, D3DFMT_A8P8, etc.
					if (IsBitmask(0x7c00, 0x03e0, 0x001f, 0x8000))
						return DXGI_FORMAT_B5G5R5A1_UNORM;

					if (IsBitmask(0xf800, 0x07e0, 0x001f, 0x0000))
						return DXGI_FORMAT_B5G6R5_UNORM;

					if (IsBitmask(0x0f00, 0x00f0, 0x000f, 0xf000))
						return DXGI_FORMAT_B4G4R4A4_UNORM;

					break;
				}
			}
		}
		else if (AllSet(ddpf.flags, dds::EPixelFormatFlags::LUMINANCE))
		{
			switch (ddpf.RGBBitCount)
			{
				case 8:
				{
					// No DXGI format maps to IsBitmask(0x0f,0x00,0x00,0xf0) aka D3DFMT_A4L4

					// D3DX10/11 writes this out as DX10 extension
					if (IsBitmask(0x000000ff, 0x00000000, 0x00000000, 0x00000000))
						return DXGI_FORMAT_R8_UNORM;

					break;
				}
				case 16:
				{
					// D3DX10/11 writes this out as DX10 extension
					if (IsBitmask(0x0000ffff, 0x00000000, 0x00000000, 0x00000000))
						return DXGI_FORMAT_R16_UNORM;

					// D3DX10/11 writes this out as DX10 extension
					if (IsBitmask(0x000000ff, 0x00000000, 0x00000000, 0x0000ff00))
						return DXGI_FORMAT_R8G8_UNORM;

					break;
				}
			}
		}
		else if (AllSet(ddpf.flags, dds::EPixelFormatFlags::ALPHA))
		{
			switch (ddpf.RGBBitCount)
			{
				case 8:
				{
					return DXGI_FORMAT_A8_UNORM;
				}
			}
		}
		else if (AllSet(ddpf.flags, dds::EPixelFormatFlags::FOURCC))
		{
			switch (ddpf.fourCC)
			{
				case ::pr::compute::MakeFourCC('D', 'X', 'T', '1'):
					return DXGI_FORMAT_BC1_UNORM;
				case ::pr::compute::MakeFourCC('D', 'X', 'T', '3'):
					return DXGI_FORMAT_BC2_UNORM;
				case ::pr::compute::MakeFourCC('D', 'X', 'T', '5'):
					return DXGI_FORMAT_BC3_UNORM;

					// While pre-mulitplied alpha isn't directly supported by the DXGI formats,
					// they are basically the same as these BC formats so they can be mapped
				case ::pr::compute::MakeFourCC('D', 'X', 'T', '2'):
					return DXGI_FORMAT_BC2_UNORM;
				case ::pr::compute::MakeFourCC('D', 'X', 'T', '4'):
					return DXGI_FORMAT_BC3_UNORM;
				case ::pr::compute::MakeFourCC('A', 'T', 'I', '1'):
					return DXGI_FORMAT_BC4_UNORM;
				case ::pr::compute::MakeFourCC('B', 'C', '4', 'U'):
					return DXGI_FORMAT_BC4_UNORM;
				case ::pr::compute::MakeFourCC('B', 'C', '4', 'S'):
					return DXGI_FORMAT_BC4_SNORM;
				case ::pr::compute::MakeFourCC('A', 'T', 'I', '2'):
					return DXGI_FORMAT_BC5_UNORM;
				case ::pr::compute::MakeFourCC('B', 'C', '5', 'U'):
					return DXGI_FORMAT_BC5_UNORM;
				case ::pr::compute::MakeFourCC('B', 'C', '5', 'S'):
					return DXGI_FORMAT_BC5_SNORM;

					// BC6H and BC7 are written using the "DX10" extended header
				case ::pr::compute::MakeFourCC('R', 'G', 'B', 'G'):
					return DXGI_FORMAT_R8G8_B8G8_UNORM;
				case ::pr::compute::MakeFourCC('G', 'R', 'G', 'B'):
					return DXGI_FORMAT_G8R8_G8B8_UNORM;

				case D3DFMT_A16B16G16R16:
					return DXGI_FORMAT_R16G16B16A16_UNORM;
				case D3DFMT_Q16W16V16U16:
					return DXGI_FORMAT_R16G16B16A16_SNORM;
				case D3DFMT_R16F:
					return DXGI_FORMAT_R16_FLOAT;
				case D3DFMT_G16R16F:
					return DXGI_FORMAT_R16G16_FLOAT;
				case D3DFMT_A16B16G16R16F:
					return DXGI_FORMAT_R16G16B16A16_FLOAT;
				case D3DFMT_R32F:
					return DXGI_FORMAT_R32_FLOAT;
				case D3DFMT_G32R32F:
					return DXGI_FORMAT_R32G32_FLOAT;
				case D3DFMT_A32B32G32R32F:
					return DXGI_FORMAT_R32G32B32A32_FLOAT;
			}
		}
		return DXGI_FORMAT_UNKNOWN;
	}

	// Load the DDS file data from 'filepath' into 'dds_data' and return pointers to the
	// header and data + size. Also, performs validation on the file can contained data.
	static dds::Image LoadTextureDataFromFile(std::filesystem::path const& filepath)
	{
		using namespace std::literals;

		// Sanity checks
		if (!std::filesystem::exists(filepath))
			throw std::runtime_error("File '"s + filepath.string() + "' does not exist");

		// Get the file size
		auto size = size_t(std::filesystem::file_size(filepath));

		// Need at least enough data to fill the header and magic number to be a valid DDS
		if (size < sizeof(dds::Header) + sizeof(uint32_t))
			throw std::runtime_error("File '"s + filepath.string() + "' is not a valid DDS file. Size is too small");

		// create enough space for the file data
		dds::Image img = {};
		img.data.reset(new uint8_t[size]);

		// Open the file (shared read)
		std::ifstream file(filepath, std::ios::binary, _SH_DENYWR);
		if (!file.good())
			throw std::runtime_error("Failed to open file: "s + filepath.string());

		// Read the data in
		if (!file.read(reinterpret_cast<char*>(img.data.get()), size).good())
			throw std::runtime_error("Read error reading data from "s + filepath.string());

		// DDS files always start with the same magic number ("DDS ")
		auto magic_sentinal = *reinterpret_cast<uint32_t const*>(img.data.get());
		if (magic_sentinal != dds::Sentinal)
			throw std::runtime_error("DDS file '"s + filepath.string() + "' is invalid. Sentinal not found");

		// Verify header to validate DDS file
		img.header = reinterpret_cast<dds::Header*>(img.data.get() + sizeof(uint32_t));
		if (img.header->size != sizeof(dds::Header) || img.header->ddspf.size != sizeof(dds::PixelFormat))
			throw std::runtime_error("DDS file '"s + filepath.string() + "' is invalid. Header corrupt");

		// Check for DX10 extension
		auto is_DXT10 = false;
		if (AllSet(img.header->ddspf.flags, dds::EPixelFormatFlags::FOURCC) && ::pr::compute::MakeFourCC('D', 'X', '1', '0') == img.header->ddspf.fourCC)
		{
			// Must be long enough for both headers and magic value
			if (size < sizeof(dds::Header) + sizeof(uint32_t) + sizeof(dds::HeaderDXT10) )
				throw std::runtime_error("DDS file '"s + filepath.string() + "' is invalid. Header claims DX10 but file size is too small");

			is_DXT10 = true;
		}

		// Offset to the start of the file data
		auto offset = sizeof(uint32_t) + sizeof(dds::Header) + (is_DXT10 ? sizeof(dds::HeaderDXT10) : 0);
		img.bits = std::span<uint8_t const>{ img.data.get() + offset, size - offset };

		// Return the image
		return img;
	}

	// Return an array of 'Image's and a resource description from DDS image data.
	static LoadedImageResult LoadDDS(dds::Image const& img, int mips, bool is_cube_map, int max_dimension)
	{
		auto resource_dimension = D3D12_RESOURCE_DIMENSION_UNKNOWN;
		auto format = DXGI_FORMAT_UNKNOWN;
		auto array_size = 1;

		// Sanity check DDS data and determine image dimension and array size
		if (AllSet(img.header->ddspf.flags, dds::EPixelFormatFlags::FOURCC) && ::pr::compute::MakeFourCC('D', 'X', '1', '0') == img.header->ddspf.fourCC)
		{
			// DX10 header
			auto& d3d10ext = *reinterpret_cast<dds::HeaderDXT10 const *>(img.header + 1);

			// Sanity checks
			array_size = d3d10ext.array_size;
			if (array_size == 0)
				throw std::runtime_error("Corrupt DDS image. DXT10 Header claims array size of 0");

			format = d3d10ext.dxgiFormat;
			if (BitsPerPixel(format) == 0)
				throw std::runtime_error(FmtS("DDS image format (%d) not supported", format));

			// Sanity checks
			resource_dimension = static_cast<D3D12_RESOURCE_DIMENSION>(d3d10ext.resourceDimension);
			switch (resource_dimension)
			{
				case D3D12_RESOURCE_DIMENSION_TEXTURE1D:
				{
					if (AllSet(img.header->flags, dds::EHeaderFlags::HEIGHT) && img.header->height != 1)
						throw std::runtime_error(FmtS("Corrupt DDS image. 1D textures should have a height of 1. Height was %d", img.header->height));

					break;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE2D:
				{
					constexpr uint32_t D3D10_RESOURCE_MISC_TEXTURECUBE = 0x4L;
					if (AllSet(d3d10ext.miscFlag, D3D10_RESOURCE_MISC_TEXTURECUBE) != is_cube_map)
						throw std::runtime_error(FmtS("Image %s a cube map but %s expected to be", (is_cube_map ? "is" : "was not"), (is_cube_map ? "is not" : "was")));

					if (is_cube_map)
						array_size *= 6;

					break;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE3D:
				{
					if (!AllSet(img.header->flags, dds::EHeaderFlags::VOLUME))
						throw std::runtime_error("Unsupported DDS format. 3D volume textures not supported");

					if (array_size > 1)
						throw std::runtime_error("Unsupported DDS format. 3D texture arrays are not supported");

					break;
				}
				default:
				{
					throw std::runtime_error(FmtS("DDS image with resource dimension %d not supported.", resource_dimension));
				}
			}
		}
		else
		{
			// Determine texture type
			format = GetDXGIFormat(img.header->ddspf);

			// Perform sanity checks
			if (format == DXGI_FORMAT_UNKNOWN)
				throw std::runtime_error(FmtS("Unsupported DDS format. Pixel format %d cannot be converted to a DXGI format", img.header->ddspf));
			if (BitsPerPixel(format) == 0)
				throw std::runtime_error(FmtS("DDS image format (%d) not supported", format));

			// Note: there's no way for a legacy Direct3D 9 DDS to express a '1D' texture
			if (AllSet(img.header->flags, dds::EHeaderFlags::VOLUME))
			{
				resource_dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
			}
			else
			{
				resource_dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;

				if (AllSet(img.header->caps2, dds::ECaps2::CUBEMAP) != is_cube_map)
					throw std::runtime_error(FmtS("Image %s a cube map but %s expected to be", (is_cube_map?"is":"was not"), (is_cube_map?"is not":"was")));

				// We require all six faces to be defined
				if (is_cube_map && !AllSet(img.header->caps2, dds::ECaps2::CUBEMAP_ALLFACES))
					throw std::runtime_error("Unsupported DDS format. Cube-map texture does not include all 6 faces");

				if (is_cube_map)
					array_size = 6;
			}
		}

		// Bound sizes (for security purposes we don't trust DDS file metadata larger than the D3D 11.x hardware requirements)
		auto mip_count = std::max(std::min(mips, s_cast<int>(img.header->mipMapCount)), 1);
		if (mip_count > D3D12_REQ_MIP_LEVELS)
			throw std::runtime_error(FmtS("Unsupported DDS format. Texture contains (%d) mip levels which exceeds the DX11 limit (%d).", mip_count, D3D12_REQ_MIP_LEVELS));

		// More sanity checks
		switch (resource_dimension)
		{
			case D3D12_RESOURCE_DIMENSION_TEXTURE1D:
			{
				if (array_size > D3D12_REQ_TEXTURE1D_ARRAY_AXIS_DIMENSION)
					throw std::runtime_error(FmtS("Unsupported DDS format. 1D texture array size (%d) exceeds array size limit (%d)", array_size, D3D12_REQ_TEXTURE1D_ARRAY_AXIS_DIMENSION));
				if (img.header->width > D3D12_REQ_TEXTURE1D_U_DIMENSION)
					throw std::runtime_error(FmtS("Unsupported DDS format. 1D texture size (%d) exceeds dimension limit (%d)", img.header->width, D3D12_REQ_TEXTURE1D_U_DIMENSION));
				break;
			}
			case D3D12_RESOURCE_DIMENSION_TEXTURE2D:
			{
				if (is_cube_map)
				{
					if (array_size > D3D12_REQ_TEXTURE2D_ARRAY_AXIS_DIMENSION)
						throw std::runtime_error(FmtS("Unsupported DDS format. Cube map texture array size (%d) exceeds array size limit (%d)", array_size, D3D12_REQ_TEXTURE2D_ARRAY_AXIS_DIMENSION));
					if (img.header->width > D3D12_REQ_TEXTURECUBE_DIMENSION || img.header->height > D3D12_REQ_TEXTURECUBE_DIMENSION)
						throw std::runtime_error(FmtS("Unsupported DDS format. Cube map texture dimensions (%dx%d) exceeds size limits (%dx%d)", img.header->width, img.header->height, D3D12_REQ_TEXTURECUBE_DIMENSION, D3D12_REQ_TEXTURECUBE_DIMENSION));
				}
				else if (array_size > D3D12_REQ_TEXTURE2D_ARRAY_AXIS_DIMENSION)
				{
					throw std::runtime_error(FmtS("Unsupported DDS format. 2D texture array size (%d) exceeds array size limit (%d)", array_size, D3D12_REQ_TEXTURE2D_ARRAY_AXIS_DIMENSION));
				}
				else if (img.header->width > D3D12_REQ_TEXTURE2D_U_OR_V_DIMENSION || img.header->height > D3D12_REQ_TEXTURE2D_U_OR_V_DIMENSION)
				{
					throw std::runtime_error(FmtS("Unsupported DDS format. 2D texture dimensions (%dx%d) exceeds size limits (%dx%d)", img.header->width, img.header->height, D3D12_REQ_TEXTURE2D_U_OR_V_DIMENSION, D3D12_REQ_TEXTURE2D_U_OR_V_DIMENSION));
				}
				break;
			}
			case D3D12_RESOURCE_DIMENSION_TEXTURE3D:
			{
				if (array_size > 1)
					throw std::runtime_error(FmtS("Unsupported DDS format. 3D texture array size (%d) exceeds array size limit (%d)", array_size, 1));
				if (img.header->width > D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION || img.header->height > D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION || img.header->depth > D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION)
					throw std::runtime_error(FmtS("Unsupported DDS format. 3D texture dimensions (%dx%dx%d) exceeds size limits (%dx%dx%d)", img.header->width, img.header->height, img.header->depth, D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION, D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION, D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION));
				break;
			}
		}

		auto bits_beg = img.bits.data();
		auto bits_end = bits_beg + img.bits.size();
		auto is_array = array_size > 1;
		auto NextMip = [=](iv3 d)
		{
			return iv3{
				std::max<int>(d.x >> 1, 1),
				std::max<int>(d.y >> 1, 1),
				is_array ? 1 : std::max<int>(d.z >> 1, 1)};
		};

		LoadedImageResult result;
		result.images.reserve(array_size);
		for (auto i = 0; i != array_size; ++i)
		{
			// The dimensions of the dds image
			auto dim = iv3{
				s_cast<int>(img.header->width),
				s_cast<int>(img.header->height),
				is_array ? 1 : s_cast<int>(img.header->depth)
			};

			// Split into mips
			ImageWithData* parent = nullptr;
			for (auto m = 0; m != mip_count; ++m, dim = NextMip(dim))
			{
				// Only add mips with dimensions <= 'max_dimension'
				if (mip_count > 1 && // There are multiple mips
					max_dimension != 0 && // There is a restriction on the mip size
					(dim.x > max_dimension || dim.y > max_dimension || dim.z > max_dimension)) // One of the dimensions is too large
					continue;

				// Get the frame size for this mip
				auto frame_size = img.header->pitchOrLinearSize * dim.y * dim.z;
				if (bits_beg + frame_size > bits_end)
					throw std::runtime_error("Insufficient image data provided");

				// Copy the data into storage within 'image'
				auto image = ImageWithData(dim.x, dim.y, dim.z, std::shared_ptr<uint8_t[]>(new uint8_t[frame_size]), format);
				memcpy(image.m_data.as<uint8_t>(), bits_beg, frame_size);
				bits_beg += frame_size;

				// Attach the image to the mip chain
				if (parent == nullptr)
				{
					result.images.push_back(std::move(image));
					parent = &result.images.back();
				}
				else
				{
					parent->m_mip.reset(new ImageWithData(std::move(image)));
					parent = parent->m_mip.get();
				}
			}
		}

		// Generate the resource description
		// Note: this is returning a description of each image in the array, not a description of the array itself.
		result.desc = D3D12_RESOURCE_DESC {
			.Dimension = resource_dimension,
			.Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
			.Width = s_cast<UINT64>(img.header->width),
			.Height = s_cast<UINT>(img.header->height),
			.DepthOrArraySize = s_cast<UINT16>(is_array ? 1 : img.header->depth),
			.MipLevels = s_cast<UINT16>(result.images.ssize() / array_size), // The new mip count (as a result of the 'max_dimension' limit),
			.Format = format,
			.SampleDesc = {1, 0},
			.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
			.Flags = D3D12_RESOURCE_FLAG_NONE,
		};

		return result;
	}

	// Load an image from a DDS image data, either in memory or on disk.
	LoadedImageResult LoadDDS(std::span<uint8_t const> mem, int mips, bool is_cube_map, int max_dimension)
	{
		if (mem.size() == 0)
			throw std::runtime_error("Texture data must be provided");
		if (mem.size() < sizeof(dds::Header) + sizeof(uint32_t))
			throw std::runtime_error("Texture data is too small to be a DDS image");
		if (*reinterpret_cast<uint32_t const*>(mem.data()) != dds::Sentinal)
			throw std::runtime_error("Texture data is not DDS image data");

		dds::Image img = {};
		img.header = reinterpret_cast<dds::Header const*>(mem.data() + sizeof(uint32_t));
		if (img.header->size != sizeof(dds::Header) || img.header->ddspf.size != sizeof(dds::PixelFormat))
			throw std::runtime_error("DDS data is corrupt. Header size is invalid");

		// Check for DX10 extension
		auto is_DXT10 = AllSet(img.header->ddspf.flags, dds::EPixelFormatFlags::FOURCC) && ::pr::compute::MakeFourCC('D', 'X', '1', '0') == img.header->ddspf.fourCC;
		if (is_DXT10 && mem.size() < sizeof(dds::Header) + sizeof(uint32_t) + sizeof(dds::HeaderDXT10))
			throw std::runtime_error("DDS data is corrupt. Header indicates DX10 but the data size is too small");

		auto offset = sizeof(dds::Header) + sizeof(uint32_t) + (is_DXT10 ? sizeof(dds::HeaderDXT10) : 0);
		img.bits = std::span{ mem.data() + offset, mem.size() - offset };

		return std::move(LoadDDS(img, mips, is_cube_map, max_dimension));
	}
	LoadedImageResult LoadDDS(std::filesystem::path const& filepath, int mips, bool is_cube_map, int max_dimension)
	{
		if (filepath.empty())
			throw std::runtime_error("Texture filepath must be provided");
		if (!std::filesystem::exists(filepath))
			throw std::runtime_error(FmtS("Texture file '%S' does not exist", filepath.c_str()));

		auto img = LoadTextureDataFromFile(filepath);
		return std::move(LoadDDS(img, mips, is_cube_map, max_dimension));
	}
}
