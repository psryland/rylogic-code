//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Functions for loading a WIC image (auto-generating mips if possible).
//
// Note: Assumes application has already called CoInitializeEx
//
// Warning: CreateWICTexture* functions are not thread-safe
//  if given a d3dContext instance for auto-gen mipmap support.
//
// Note these functions are useful for images created as simple 2D textures. For
// more complex resources, DDSTextureLoader is an excellent light-weight runtime loader.
// For a full-featured DDS file reader, writer, and texture processing pipeline see
// the 'Texconv' sample and the 'DirectXTex' library.
//
// We could load multi-frame images (TIFF/GIF) into a texture array.
// For now, we just load the first frame (note: DirectXTex supports multi-frame images)
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// Copyright (c) Microsoft Corporation. All rights reserved.
/// http://go.microsoft.com/fwlink/?LinkId=248926
// http://go.microsoft.com/fwlink/?LinkId=248929
#include "pr/view3d-12/texture/texture_loader.h"
#include "pr/view3d-12/utility/features.h"
#include <wincodec.h>

//#include "pr/view3d/util/wrappers.h"
//#include <memory>
//#include <dxgiformat.h>
//#include <assert.h>
//#include <stdint.h>

namespace pr::rdr12
{
	using ImageWithData = LoadedImageResult::ImageWithData;

	namespace wic
	{
		// WIC Pixel Format Translation Data
		struct Translate
		{
			WICPixelFormatGUID wic;
			DXGI_FORMAT format;
		};
		static Translate const g_formats[] =
		{
			{ GUID_WICPixelFormat128bppRGBAFloat, DXGI_FORMAT_R32G32B32A32_FLOAT },

			{ GUID_WICPixelFormat64bppRGBAHalf, DXGI_FORMAT_R16G16B16A16_FLOAT },
			{ GUID_WICPixelFormat64bppRGBA, DXGI_FORMAT_R16G16B16A16_UNORM },

			{ GUID_WICPixelFormat32bppRGBA, DXGI_FORMAT_R8G8B8A8_UNORM },
			{ GUID_WICPixelFormat32bppBGRA, DXGI_FORMAT_B8G8R8A8_UNORM }, // DXGI 1.1
			{ GUID_WICPixelFormat32bppBGR, DXGI_FORMAT_B8G8R8X8_UNORM }, // DXGI 1.1

			{ GUID_WICPixelFormat32bppRGBA1010102XR, DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM }, // DXGI 1.1
			{ GUID_WICPixelFormat32bppRGBA1010102, DXGI_FORMAT_R10G10B10A2_UNORM },
			{ GUID_WICPixelFormat32bppRGBE, DXGI_FORMAT_R9G9B9E5_SHAREDEXP },

			{ GUID_WICPixelFormat16bppBGRA5551, DXGI_FORMAT_B5G5R5A1_UNORM },
			{ GUID_WICPixelFormat16bppBGR565, DXGI_FORMAT_B5G6R5_UNORM },

			{ GUID_WICPixelFormat32bppGrayFloat, DXGI_FORMAT_R32_FLOAT },
			{ GUID_WICPixelFormat16bppGrayHalf, DXGI_FORMAT_R16_FLOAT },
			{ GUID_WICPixelFormat16bppGray, DXGI_FORMAT_R16_UNORM },
			{ GUID_WICPixelFormat8bppGray, DXGI_FORMAT_R8_UNORM },

			{ GUID_WICPixelFormat8bppAlpha, DXGI_FORMAT_A8_UNORM },

			#if _WIN32_WINNT >= _WIN32_WINNT_WIN8
			{ GUID_WICPixelFormat96bppRGBFloat, DXGI_FORMAT_R32G32B32_FLOAT },
			#endif
		};

		// WIC Pixel Format nearest conversion table
		struct Convert
		{
			GUID source;
			GUID target;
		};
		static Convert const g_convert[] =
		{
			// Note target GUID in this conversion table must be one of those directly supported formats (above).
			{ GUID_WICPixelFormatBlackWhite, GUID_WICPixelFormat8bppGray }, // DXGI_FORMAT_R8_UNORM

			{ GUID_WICPixelFormat1bppIndexed, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat2bppIndexed, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat4bppIndexed, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat8bppIndexed, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM

			{ GUID_WICPixelFormat2bppGray, GUID_WICPixelFormat8bppGray }, // DXGI_FORMAT_R8_UNORM
			{ GUID_WICPixelFormat4bppGray, GUID_WICPixelFormat8bppGray }, // DXGI_FORMAT_R8_UNORM

			{ GUID_WICPixelFormat16bppGrayFixedPoint, GUID_WICPixelFormat16bppGrayHalf }, // DXGI_FORMAT_R16_FLOAT
			{ GUID_WICPixelFormat32bppGrayFixedPoint, GUID_WICPixelFormat32bppGrayFloat }, // DXGI_FORMAT_R32_FLOAT

			{ GUID_WICPixelFormat16bppBGR555, GUID_WICPixelFormat16bppBGRA5551 }, // DXGI_FORMAT_B5G5R5A1_UNORM

			{ GUID_WICPixelFormat32bppBGR101010, GUID_WICPixelFormat32bppRGBA1010102 }, // DXGI_FORMAT_R10G10B10A2_UNORM

			{ GUID_WICPixelFormat24bppBGR, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat24bppRGB, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat32bppPBGRA, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat32bppPRGBA, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM

			{ GUID_WICPixelFormat48bppRGB, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat48bppBGR, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat64bppBGRA, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat64bppPRGBA, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat64bppPBGRA, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM

			{ GUID_WICPixelFormat48bppRGBFixedPoint, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			{ GUID_WICPixelFormat48bppBGRFixedPoint, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			{ GUID_WICPixelFormat64bppRGBAFixedPoint, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			{ GUID_WICPixelFormat64bppBGRAFixedPoint, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			{ GUID_WICPixelFormat64bppRGBFixedPoint, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			{ GUID_WICPixelFormat64bppRGBHalf, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			{ GUID_WICPixelFormat48bppRGBHalf, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT

			{ GUID_WICPixelFormat96bppRGBFixedPoint, GUID_WICPixelFormat128bppRGBAFloat }, // DXGI_FORMAT_R32G32B32A32_FLOAT
			{ GUID_WICPixelFormat128bppPRGBAFloat, GUID_WICPixelFormat128bppRGBAFloat }, // DXGI_FORMAT_R32G32B32A32_FLOAT
			{ GUID_WICPixelFormat128bppRGBFloat, GUID_WICPixelFormat128bppRGBAFloat }, // DXGI_FORMAT_R32G32B32A32_FLOAT
			{ GUID_WICPixelFormat128bppRGBAFixedPoint, GUID_WICPixelFormat128bppRGBAFloat }, // DXGI_FORMAT_R32G32B32A32_FLOAT
			{ GUID_WICPixelFormat128bppRGBFixedPoint, GUID_WICPixelFormat128bppRGBAFloat }, // DXGI_FORMAT_R32G32B32A32_FLOAT

			{ GUID_WICPixelFormat32bppCMYK, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat64bppCMYK, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat40bppCMYKAlpha, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat80bppCMYKAlpha, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM

			#if _WIN32_WINNT >= _WIN32_WINNT_WIN8
			{ GUID_WICPixelFormat32bppRGB, GUID_WICPixelFormat32bppRGBA }, // DXGI_FORMAT_R8G8B8A8_UNORM
			{ GUID_WICPixelFormat64bppRGB, GUID_WICPixelFormat64bppRGBA }, // DXGI_FORMAT_R16G16B16A16_UNORM
			{ GUID_WICPixelFormat64bppPRGBAHalf, GUID_WICPixelFormat64bppRGBAHalf }, // DXGI_FORMAT_R16G16B16A16_FLOAT
			#endif
			// We don't support n-channel formats
		};
	}

	// Get the WIC imaging factory instance
	IWICImagingFactory* GetWIC()
	{
		static IWICImagingFactory* s_factory = []
		{
			IWICImagingFactory* factory;
			Check(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, __uuidof(IWICImagingFactory), (void**)&factory));
			return factory;
		}();
		return s_factory;
	}

	// Convert a WIC format GUID to a DXGI format id.
	DXGI_FORMAT WICToDXGI(WICPixelFormatGUID const& pf, bool include_convertible, WICPixelFormatGUID* convert_guid)
	{
		// Find a direct match for pixel format
		for (size_t i = 0; i < _countof(wic::g_formats); ++i)
		{
			if (memcmp(&wic::g_formats[i].wic, &pf, sizeof(WICPixelFormatGUID)) != 0) continue;
			if (convert_guid != nullptr) *convert_guid = pf;
			return wic::g_formats[i].format;
		}

		// Fall back to formats that are convertible to 'pf'
		if (include_convertible)
		{
			for (size_t i = 0; i < _countof(wic::g_convert); ++i)
			{
				if (memcmp(&wic::g_convert[i].source, &pf, sizeof(WICPixelFormatGUID)) != 0) continue;
				if (convert_guid != nullptr) *convert_guid = wic::g_convert[i].target;
				return WICToDXGI(wic::g_convert[i].target, false, nullptr);
			}
		}

		return DXGI_FORMAT_UNKNOWN;
	}

	// Return the number of bits per pixel for the given WIC pixel format
	int WICBitsPerPixel(WICPixelFormatGUID const& guid)
	{
		auto wic = GetWIC();

		RefPtr<IWICComponentInfo> cinfo;
		Check(wic->CreateComponentInfo(guid, cinfo.address_of()));

		WICComponentType type;
		Check(cinfo->GetComponentType(&type));
		if (type != WICPixelFormat)
			return 0;

		RefPtr<IWICPixelFormatInfo> pfinfo;
		Check(cinfo->QueryInterface(__uuidof(IWICPixelFormatInfo), reinterpret_cast<void**>(&pfinfo)));

		UINT bpp;
		Check(pfinfo->GetBitsPerPixel(&bpp));
		return s_cast<int>(bpp);
	}

	// Return an array of 'Image's and a resource description from DDS image data.
	LoadedImageResult LoadWIC(vector<RefPtr<IWICBitmapFrameDecode>> const& frames, int mips, int max_dimension, FeatureSupport const* features)
	{
		if (frames.empty())
			throw std::runtime_error("No image frames provided");

		// Assume the image properties are the same for all images in the array
		auto const& first = frames[0];

		// Read the image dimensions
		UINT width, height;
		Check(first->GetSize(&width, &height));
		assert(width > 0 && height > 0);

		// Determine the maximum texture dimension
		if (features != nullptr)
		{
			// This is a bit conservative because the hardware could support larger textures than
			// the Feature Level defined minimums, but doing it this way is much easier and more
			// performant for WIC than the 'fail and retry' model used by DDSTextureLoader
			max_dimension =
				max_dimension != 0 ? max_dimension :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_9_1 ? D3D_FL9_1_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_9_2 ? D3D_FL9_1_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_9_3 ? D3D_FL9_3_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_10_0 ? D3D10_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_10_1 ? D3D10_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_11_0 ? D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				features->MaxFeatureLevel == D3D_FEATURE_LEVEL_11_1 ? D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION :
				D3D12_REQ_TEXTURE2D_U_OR_V_DIMENSION;
		}
		else
		{
			max_dimension = limits<int>::max();
		}

		// Clamp the texture dimensions to the maximum, maintaining aspect ratio
		iv3 dim = {s_cast<int>(width), s_cast<int>(height), 1}; // WIC only supports 2D images
		if (dim.x > max_dimension || dim.y > max_dimension)
		{
			auto ar = static_cast<double>(height) / static_cast<double>(width);
			if (width > height)
			{
				dim.x = max_dimension;
				dim.y = static_cast<int>(max_dimension * ar);
			}
			else
			{
				dim.y = max_dimension;
				dim.x = static_cast<int>(max_dimension / ar);
			}
			assert(dim.x <= max_dimension && dim.y <= max_dimension);
		}

		// Determine the pixel format
		WICPixelFormatGUID src_format, dst_format;
		Check(first->GetPixelFormat(&src_format));
		auto format = WICToDXGI(src_format, true, &dst_format);
		if (format == DXGI_FORMAT_UNKNOWN)
			throw std::runtime_error("Pixel format is not supported");

		// Determine the bits per pixel
		auto bpp = WICBitsPerPixel(dst_format);
		if (bpp == 0)
			throw std::runtime_error("Could not determine bits per pixel from the pixel format");

		// Verify our target format is supported by the current device
		// (handles WDDM 1.0 or WDDM 1.1 device driver cases as well as DirectX 11.0 Runtime without 16bpp format support)
		for (; features != nullptr; )
		{
			auto support = features->Format(format);
			if (AllSet(support.Support1, D3D12_FORMAT_SUPPORT1_TEXTURE2D))
				break;

			// Try BGRA
			support = features->Format(DXGI_FORMAT_B8G8R8A8_UNORM);
			if (!AllSet(support.Support1, D3D12_FORMAT_SUPPORT1_TEXTURE2D))
			{
				dst_format = GUID_WICPixelFormat32bppBGRA;
				format = DXGI_FORMAT_B8G8R8A8_UNORM;
				bpp = 32;
				break;
			}

			// Fall back to RGBA 32-bit format which is supported by all devices
			dst_format = GUID_WICPixelFormat32bppRGBA;
			format = DXGI_FORMAT_R8G8B8A8_UNORM;
			bpp = 32;
			break;
		}

		auto pitch = (dim.x * bpp + 7) / 8;
		auto frame_size = pitch * dim.y;
		auto conversion_needed = src_format != dst_format;
		auto resize_needed = dim.x != s_cast<int>(width) || dim.y != s_cast<int>(height);
		auto is_array = frames.ssize() != 1;
		mips = mips == 0 ? ::pr::compute::MipCount(dim.x, dim.y) : mips;

		LoadedImageResult result;

		// Load the image frames
		for (auto f = 0; f != frames.ssize(); ++f)
		{
			auto& frame = frames[f];
			auto image = ImageWithData(dim.x, dim.y, is_array ? 1 : dim.z, std::shared_ptr<uint8_t[]>(new uint8_t[frame_size]), format);

			if (!conversion_needed && !resize_needed)
			{
				// No format conversion or resize needed
				Check(frame->CopyPixels(0, static_cast<UINT>(pitch), static_cast<UINT>(frame_size), image.m_data.as<uint8_t>()));
			}
			else
			{
				RefPtr<IWICBitmapScaler> scaler;
				RefPtr<IWICFormatConverter> converter;
				auto wic = GetWIC();

				// Create a resizer if needed
				if (resize_needed)
				{
					WICPixelFormatGUID pf;
					Check(wic->CreateBitmapScaler(scaler.address_of()));
					Check(scaler->Initialize(const_cast<IWICBitmapFrameDecode*>(frame.get()), s_cast<UINT>(dim.x), s_cast<UINT>(dim.y), WICBitmapInterpolationModeFant));
					Check(scaler->GetPixelFormat(&pf));
					conversion_needed = pf != dst_format;
				}

				// Create a format converter if needed
				if (conversion_needed)
				{
					Check(wic->CreateFormatConverter(converter.address_of()));
					Check(converter->Initialize(const_cast<IWICBitmapFrameDecode*>(frame.get()), dst_format, WICBitmapDitherTypeErrorDiffusion, 0, 0, WICBitmapPaletteTypeCustom));
				}

				// Copy the data with optional reformat and resize
				Check(converter->CopyPixels(0, s_cast<UINT>(pitch), s_cast<UINT>(frame_size), image.m_data.as<uint8_t>()));
			}
			result.images.push_back(std::move(image));
		}

		// Create the texture description.
		// Note: this is returning a description of each image in the array, not a description of the array itself.
		result.desc = D3D12_RESOURCE_DESC{
			.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
			.Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
			.Width = s_cast<UINT64>(dim.x),
			.Height = s_cast<UINT>(dim.y),
			.DepthOrArraySize = s_cast<UINT16>(is_array ? 1 : dim.z),
			.MipLevels = s_cast<UINT16>(mips),
			.Format = format,
			.SampleDesc = {1, 0},
			.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
			.Flags = D3D12_RESOURCE_FLAG_NONE,
		};

		return result;
	}

	// Load an image from a WIC image, either in memory or on disk.
	LoadedImageResult LoadWIC(std::span<std::span<uint8_t const>> images, int mips, int max_dimension, FeatureSupport const* features)
	{
		if (images.empty())
			throw std::runtime_error("Texture file data is invalid");

		auto wic = GetWIC();

		// Load each image frame
		vector<RefPtr<IWICBitmapFrameDecode>> frames;
		for (int i = 0, iend = s_cast<int>(images.size()); i != iend; ++i)
		{
			auto const& img = images[i];

			// Create input stream for memory
			RefPtr<IWICStream> stream;
			Check(wic->CreateStream(stream.address_of()));
			Check(stream->InitializeFromMemory(const_cast<uint8_t*>(img.data()), static_cast<DWORD>(img.size())));

			// Initialize WIC image decoder
			RefPtr<IWICBitmapDecoder> decoder;
			Check(wic->CreateDecoderFromStream(stream.get(), 0, WICDecodeMetadataCacheOnDemand, decoder.address_of()));

			// Get the first frame in the image
			RefPtr<IWICBitmapFrameDecode> frame;
			Check(decoder->GetFrame(0, frame.address_of()));
			frames.push_back(frame);
		}

		// Create the texture
		return LoadWIC(frames, mips, max_dimension, features);
	}
	LoadedImageResult LoadWIC(std::span<std::filesystem::path const> filepaths, int mips, int max_dimension, FeatureSupport const* features)
	{
		auto wic = GetWIC();

		// Load each image
		vector<RefPtr<IWICBitmapFrameDecode>> frames;
		for (auto& path : filepaths)
		{
			// Initialize WIC image decoder
			RefPtr<IWICBitmapDecoder> decoder;
			Check(wic->CreateDecoderFromFilename(path.c_str(), 0, GENERIC_READ, WICDecodeMetadataCacheOnDemand, decoder.address_of()));

			// Get the first frame in the image
			RefPtr<IWICBitmapFrameDecode> frame;
			Check(decoder->GetFrame(0, frame.address_of()));

			// Add the frame
			frames.push_back(frame);
		}

		// Create the texture
		return LoadWIC(frames, mips, max_dimension, features);
	}
}
