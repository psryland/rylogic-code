//*********************************************
// Renderer
//  Copyright (c) Rylogic Ltd 2012
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/view3d-12/utility/utility.h"

namespace pr::rdr12
{
	// A basic image description
	struct Image
	{
		// Image:
		//
		//   +----- +-slice-+
		//   B    +-slice-+ |
		//   L  +-slice-+ | |
		//   O  | [row] | | |
		//   C  | [row] | |-+
		//   K  | [row] |-+
		//   +- +-------+
		//
		// Notes:
		//  - Images can be generalised to a 1D, 2D, or 3D buffer of any type. E.g. a vertex buffer is a 1D image of Verts.
		//  - row pitch is the number of bytes per row of the image.
		//  - slice pitch is the number of bytes per 2D slice (i.e. normally the image size in bytes,
		//    but if the image is an array, then this is the size of one image in the array)
		//  - block pitch is the number of bytes for the image.
		//  - 'Image' does not store mip levels. Use an array of Images to represent a mip chain.

		union data_t
		{
			using factory_t = void const* (*)(void* ctx, void* mem);

			void const* vptr;
			uint8_t const* bptr;
			uint16_t const* u16ptr;
			uint32_t const* u32ptr;
			v4 const* v4ptr;
			Vert const* vertptr;
			template <typename T> T* as() { return static_cast<T*>(const_cast<void*>(vptr)); }
			explicit operator bool() const { return vptr != nullptr; }
			bool operator ==(nullptr_t) const { return vptr == nullptr; }
			bool operator !=(nullptr_t) const { return vptr != nullptr; }
		};

		iv3 m_dim;            // x = width, y = height, z = depth
		iv3 m_pitch;          // x = row pitch, y = slice pitch, z = block pitch
		data_t m_data;        // A pointer to the image data.
		DXGI_FORMAT m_format; // The pixel format of the image

		Image()
			:m_dim()
			,m_pitch()
			,m_data()
			,m_format(DXGI_FORMAT_B8G8R8A8_UNORM)
		{}

		// Construct explicit
		Image(iv3 dim, iv3 pitch, void const* data = nullptr, DXGI_FORMAT fmt = DXGI_FORMAT_B8G8R8A8_UNORM)
			: m_dim(dim)
			, m_pitch(pitch)
			, m_data{.vptr = data}
			, m_format(fmt)
		{}

		// Construct a 1D Image.
		Image(int w, void const* data = nullptr, DXGI_FORMAT fmt = DXGI_FORMAT_B8G8R8A8_UNORM)
			: m_dim(w, 1, 1)
			, m_pitch(w, w, w)
			, m_data{.vptr = data}
			, m_format(fmt)
		{}

		// Construct a 2D Image.
		// 'pixels' should point to data in the format 'fmt' for the base image (dimensions should be m_row_pitch x m_slice_pitch, use Pitch())
		Image(int w, int h, void const* pixels = nullptr, DXGI_FORMAT fmt = DXGI_FORMAT_B8G8R8A8_UNORM)
			: m_dim(w, h, 1)
			, m_pitch(Pitch(m_dim, fmt))
			, m_data{.vptr = pixels}
			, m_format(fmt)
		{}

		// Construct a 3D Image.
		// 'pixels' should point to data in the format 'fmt' for the base image (dimensions should be m_row_pitch x m_slice_pitch, use Pitch())
		Image(int w, int h, int d, void const* pixels = nullptr, DXGI_FORMAT fmt = DXGI_FORMAT_B8G8R8A8_UNORM)
			: m_dim(w, h, d)
			, m_pitch(Pitch(m_dim, fmt))
			, m_data{.vptr = pixels}
			, m_format(fmt)
		{}

		// Construct a 1D buffer
		Image(void const* data, int64_t count, int element_size_in_bytes)
			: m_dim(s_cast<int>(count), 1, 1)
			, m_pitch(s_cast<int>(count * element_size_in_bytes))
			, m_data{.vptr = data}
			, m_format(DXGI_FORMAT_R8_UNORM)
		{
			if (s_cast<int64_t>(count) * s_cast<int64_t>(element_size_in_bytes) > limits<int>::max())
				throw std::overflow_error("Initialisation data too large");
		}

		// Element size in bytes
		int ElemStride() const
		{
			return m_pitch.x / m_dim.x;
		}

		// The image size in bytes
		int SizeInBytes() const
		{
			assert(m_pitch.z >= m_pitch.x && m_pitch.z >= m_pitch.y);
			return m_pitch.z;
		}

		// Access a slice (i.e. a Z plane) in the image
		Image Slice(int z) const
		{
			if (z < 0 || z >= m_dim.z)
				throw std::runtime_error("Slice index out of range");

			return Image(
				iv3(m_dim.x, m_dim.y, 1),
				iv3(m_pitch.x, m_pitch.y, m_pitch.y), // Not a bug, z = 1 so pitch.z == pitch.y
				m_data.bptr + m_pitch.y * z, m_format);
		}

		// Access a row (i.e. a Y line) in the image
		Image Row(int y, int z = 0) const
		{
			if (y < 0 || y >= m_dim.y)
				throw std::runtime_error("Row index out of range");

			return Image(m_dim.x, m_data.bptr + m_pitch.x * y + m_pitch.y * z, m_format);
		}

		// Convert to Dx12 types
		operator D3D12_SUBRESOURCE_DATA() const
		{
			D3D12_SUBRESOURCE_DATA data = {};
			data.pData = m_data.vptr;
			data.RowPitch = s_cast<LONG_PTR>(m_pitch.x);
			data.SlicePitch = s_cast<LONG_PTR>(m_pitch.y);
			return data;
		}
		operator D3D12_MEMCPY_DEST()
		{
			D3D12_MEMCPY_DEST data = {};
			data.pData = m_data.as<void*>();
			data.RowPitch = s_cast<LONG_PTR>(m_pitch.x);
			data.SlicePitch = s_cast<LONG_PTR>(m_pitch.y);
			return data;
		}
	};

	// An image that owns it's data
	struct ImageWithData :Image
	{
		using local_data_t = std::shared_ptr<uint8_t[]>;
		using mip_t = std::shared_ptr<ImageWithData>;

		local_data_t m_bits;
		mip_t m_mip; // Linked list of mip levels

		ImageWithData()
			:Image()
			,m_bits()
			,m_mip()
		{}
		ImageWithData(ImageWithData const&) = default;
		ImageWithData& operator =(ImageWithData const&) = default;

		// Construct a 1D image
		ImageWithData(int w, std::shared_ptr<uint8_t[]> data, DXGI_FORMAT fmt = DXGI_FORMAT_UNKNOWN)
			:ImageWithData(w, 1, 1, data, fmt)
		{}

		// Construct a 2D image
		ImageWithData(int w, int h, std::shared_ptr<uint8_t[]> data, DXGI_FORMAT fmt = DXGI_FORMAT_B8G8R8A8_UNORM)
			:ImageWithData(w, h, 1, data, fmt)
		{}

		// Construct a 3D image
		ImageWithData(int w, int h, int d, std::shared_ptr<uint8_t[]> data, DXGI_FORMAT fmt = DXGI_FORMAT_B8G8R8A8_UNORM)
			:Image(w, h, d, data.get(), fmt)
			,m_bits(data)
		{}

		// Construct a 1D buffer
		ImageWithData(std::shared_ptr<uint8_t[]> data, int count, int element_size_in_bytes)
			:Image(data.get(), count, element_size_in_bytes)
			, m_bits(data)
		{}
	
		// Copy/Assign from Image
		ImageWithData(Image const& rhs)
			:ImageWithData(rhs.m_dim.x, rhs.m_dim.y, rhs.m_dim.z, std::shared_ptr<uint8_t[]>(new uint8_t[rhs.SizeInBytes()]), rhs.m_format)
		{
			memcpy(m_bits.get(), rhs.m_data.vptr, rhs.SizeInBytes());
		}
		ImageWithData& operator =(Image const& rhs)
		{
			if (&rhs == this) return *this;
			*this = ImageWithData(rhs);
			return *this;
		}
	};
}
