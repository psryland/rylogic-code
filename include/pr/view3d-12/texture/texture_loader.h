//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Notes:
	//  - These functions convert image files into an 'ImageWithData' object, *not* into
	//    ID3D12Resources because resource initialisation requires command lists.
	//  - LoadWIC no longer automatically generates mip-maps. A function on the ResourceManager will have to do it.
	//  - DDS textures can contain arrays of images, where as WIC images are simple 2D bitmaps.
	//  - WIC functions support arrays by filepath pattern or by array of raw data. Array textures all have the same dimensions.
	//  - Arrays of 3D textures are not supported.
	
	
	// Use structured binding. i.e. auto [image, desc] = LoadDDS(...);
	struct LoadedImageResult
	{
		using ImageWithData = ::pr::compute::ImageWithData;
		using ResDesc = ::pr::compute::ResDesc;

		vector<ImageWithData> images; // Each image is an array slice (not a mip map)
		ResDesc desc;
	};

	// True if 'data' points at DDS data (probably)
	bool IsDDSData(std::span<uint8_t const> img);

	// Load an image from a DDS image, either in memory or on disk.
	// 'colour_space' hints whether texels should be sampled as sRGB-encoded colour data (default) or linear-space numeric data.
	// For DDS, an explicit '_SRGB' format in the file header wins; otherwise the hint promotes a matching '_UNORM' to '_UNORM_SRGB' when Srgb.
	LoadedImageResult LoadDDS(std::span<uint8_t const> mem, int mips = 0, bool is_cube_map = false, int max_dimension = 0, EColourSpace colour_space = EColourSpace::Srgb);
	LoadedImageResult LoadDDS(std::filesystem::path const& filepath, int mips = 0, bool is_cube_map = false, int max_dimension = 0, EColourSpace colour_space = EColourSpace::Srgb);

	// Load an image from a WIC image, either in memory or on disk.
	// 'colour_space' picks between the '_UNORM' (Linear) and '_UNORM_SRGB' (Srgb) DXGI format variant for the decoded pixel format.
	// PNG/JPG/etc. files carry no reliable colour-space tag we honour, so the caller must say what the texels represent.
	LoadedImageResult LoadWIC(std::span<std::span<uint8_t const>> images, int mips = 0, int max_dimension = 0, FeatureSupport const* features = nullptr, EColourSpace colour_space = EColourSpace::Srgb);
	LoadedImageResult LoadWIC(std::span<std::filesystem::path const> filepaths, int mips = 0, int max_dimension = 0, FeatureSupport const* features = nullptr, EColourSpace colour_space = EColourSpace::Srgb);

	// Load 'DDS, JPG, PNG, TGA, GIF, or BMP' image data, either from memory or disk.
	// 'data' is an in-memory image file.
	// 'images' is an array of equal dimension in-memory image files.
	// 'filepath' is a single image file.
	// 'filepaths' is a sorted list of equal dimension image files that make up the elements in a texture array or cube map.
	// Cubemap image order is: px, nx, py, ny, pz, nz
	// DDS images natively support cube maps and array textures so only single DDS images are supported. (See Texassemble.exe for creating DDS textures)
	// 'colour_space' is forwarded to the underlying DDS/WIC loader.
	inline LoadedImageResult LoadImageData(std::span<std::filesystem::path const> filepaths, int mips = 0, bool is_cube_map = false, int max_dimension = 0, FeatureSupport const* features = nullptr, EColourSpace colour_space = EColourSpace::Srgb)
	{
		if (filepaths.empty())
			throw std::runtime_error("At least one image is required");

		// If the file is a DDS file, use the faster DDS loader.
		// This does not support some DDS formats tho, so might be worth trying the 'directxtex' DDS loader
		auto extn = filepaths[0].extension();
		auto dds = extn.compare(L".dds") == 0;

		if (dds)
		{
			if (filepaths.size() != 1)
				throw std::runtime_error("Only single DDS textures are supported since they natively support texture arrays and cube maps");

			return LoadDDS(filepaths[0], mips, is_cube_map, max_dimension, colour_space);
		}
		else
		{
			if (is_cube_map && filepaths.size() != 6)
				throw std::runtime_error("Expected 6 images for a cube map");

			return LoadWIC(filepaths, mips, max_dimension, features, colour_space);
		}
	}
	inline LoadedImageResult LoadImageData(std::filesystem::path const& filepath, int mips = 0, bool is_cube_map = false, int max_dimension = 0, FeatureSupport const* features = nullptr, EColourSpace colour_space = EColourSpace::Srgb)
	{
		return LoadImageData({ &filepath, 1 }, mips, is_cube_map, max_dimension, features, colour_space);
	}
	inline LoadedImageResult LoadImageData(std::span<std::span<uint8_t const>> const& images, int mips = 0, bool is_cube_map = false, int max_dimension = 0, FeatureSupport const* features = nullptr, EColourSpace colour_space = EColourSpace::Srgb)
	{
		if (images.empty())
			throw std::runtime_error("At least one image is required");

		// If the data is a DDS file, use the faster DDS loader.
		// This does not support some DDS formats tho, so might be worth trying the 'directxtex' DDS loader
		if (IsDDSData(images[0]))
		{
			if (images.size() != 1)
				throw std::runtime_error("Only single DDS textures are supported since they natively support texture arrays and cube maps");

			return LoadDDS(images[0], mips, is_cube_map, max_dimension, colour_space);
		}
		else
		{
			if (is_cube_map && images.size() != 6)
				throw std::runtime_error("Expected 6 images for a cube map");

			return LoadWIC(images, mips, max_dimension, features, colour_space);
		}
	}
	inline LoadedImageResult LoadImageData(std::span<uint8_t const> data, int mips = 0, bool is_cube_map = false, int max_dimension = 0, FeatureSupport const* features = nullptr, EColourSpace colour_space = EColourSpace::Srgb)
	{
		return LoadImageData(std::span{ &data, 1 }, mips, is_cube_map, max_dimension, features, colour_space);
	}
}
